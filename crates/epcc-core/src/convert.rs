//! Converting one file: reader → filters → writer, each stage on its own thread.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::thread;
use std::time::{Duration, Instant};

use crossbeam_channel::bounded;

use crate::error::IoContext;
use crate::format::{PointReader, WriteRequest};
use crate::model::{Attributes, CloudInfo, ExtraField, PointChunk, ScanGroups};
use crate::output::{ClaimedPaths, OutputNames, PendingOutput};
use crate::reduce::{PointFilter, filters_for};
use crate::settings::{ScanMode, Settings};
use crate::{Error, Format, Progress, Result};

/// Chunks that may wait between two stages. Keeps memory flat while letting stages overlap.
const QUEUE_CHUNKS: usize = 2;

#[derive(Clone, Debug, PartialEq)]
pub struct ConversionJob {
    pub input: PathBuf,
    pub format: Format,
    pub output_folder: PathBuf,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Outcome {
    Converted(ConversionReport),
    /// The output already exists and the settings say to skip it.
    Skipped(PathBuf),
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ConversionReport {
    pub outputs: Vec<PathBuf>,
    pub points_read: u64,
    pub points_written: u64,
    pub warnings: Vec<String>,
    pub duration: Duration,
}

/// Converts one file. `claimed` is shared by jobs running at the same time so they never write
/// to the same output path.
pub fn convert(
    job: &ConversionJob,
    settings: &Settings,
    progress: &Progress,
    claimed: &ClaimedPaths,
) -> Result<Outcome> {
    let started = Instant::now();
    settings.validate()?;
    let source_format = Format::from_path(&job.input)
        .ok_or_else(|| Error::Unsupported(format!("{}: unknown file type", job.input.display())))?;
    std::fs::create_dir_all(&job.output_folder).at(&job.output_folder)?;
    let names = OutputNames {
        folder: &job.output_folder,
        stem: output_stem(&job.input),
        extension: job.format.extension(),
        input: &job.input,
        existing: settings.general.existing_files,
        claimed,
    };
    let split_scans = !job.format.stores_scans() && settings.processing.scans == ScanMode::Split;

    // A merged output's name is known up front, so a file to skip is never even read.
    let merged_path = if split_scans {
        None
    } else {
        match names.pick(None) {
            Some(path) => Some(path),
            None => return Ok(Outcome::Skipped(names.folder.join(format!("{}.{}", names.stem, names.extension)))),
        }
    };

    let reader = source_format.open(&job.input, &settings.import)?;
    let cloud = output_cloud(reader.info(), job.format, settings, progress);
    let filters = filters_for(&settings.processing, &job.output_folder);

    let write = |chunks: &mut dyn Iterator<Item = PointChunk>| -> Result<Vec<(usize, PendingOutput)>> {
        let write_one = |output: &PendingOutput, chunks: &mut dyn Iterator<Item = PointChunk>| {
            let request = WriteRequest {
                path: &output.temporary,
                cloud: &cloud,
                settings: &settings.export,
                source: &job.input,
                source_format,
            };
            job.format.create_writer(&request)?.write(chunks, progress)
        };
        if let Some(path) = &merged_path {
            let output = PendingOutput::for_path(path);
            write_one(&output, chunks)?;
            return Ok(vec![(0, output)]);
        }
        // Some formats only discover their scans while reading, so the names are chosen once
        // all scans are written.
        let mut outputs = Vec::new();
        let mut scans = ScanGroups::new(chunks);
        while let Some((scan, mut group)) = scans.next_group() {
            let output = PendingOutput::unnamed(names.folder, &names.stem, names.extension, scan);
            write_one(&output, &mut group)?;
            outputs.push((scan, output));
        }
        Ok(outputs)
    };

    progress.reserve_after_reading(job.format.work_after_reading());
    let counts = run_pipeline(reader, filters, cloud.attributes, write, progress)?;
    let several = counts.outputs.len() > 1;
    let mut outputs = Vec::new();
    for (scan, output) in counts.outputs {
        let path = match &merged_path {
            Some(path) => Some(path.clone()),
            None => names.pick(several.then(|| cloud.scan(scan).name).as_deref()),
        };
        if let Some(path) = path {
            output.commit(&path)?;
            outputs.push(path);
        }
    }
    progress.set_fraction(1.0);

    let report = ConversionReport {
        outputs,
        points_read: counts.read,
        points_written: counts.written,
        warnings: progress.warnings(),
        duration: started.elapsed(),
    };
    if settings.general.write_log {
        write_log(job, &report)?;
    }
    Ok(Outcome::Converted(report))
}

/// The cloud as the output will describe it: only the attributes the target format can store
/// and the user wants.
fn output_cloud(source: &CloudInfo, target: Format, settings: &Settings, progress: &Progress) -> CloudInfo {
    let wanted = source.attributes.without(settings.processing.drop);
    let attributes = wanted.intersection(target.stored_attributes());
    let lost = wanted.without(attributes);
    if !lost.is_empty() {
        let mut names: Vec<String> = lost.names().into_iter().map(String::from).collect();
        if lost.has_extra()
            && let Some(extra) = names.last_mut()
        {
            *extra = format!("{extra} ({})", field_names(&source.extra_fields));
        }
        progress.warn(format!("{} cannot store {}; left out.", target.name(), names.join(", ")));
    }
    let reduced = settings.processing.min_spacing.is_some() || settings.processing.every_nth.is_some_and(|n| n > 1);
    CloudInfo {
        attributes,
        point_count: if reduced { None } else { source.point_count },
        extra_fields: if attributes.has_extra() { source.extra_fields.clone() } else { Vec::new() },
        ..source.clone()
    }
}

/// The names of `fields` for a message, the first few of them if there are many.
pub(crate) fn field_names(fields: &[ExtraField]) -> String {
    const SHOWN: usize = 8;
    let mut names: Vec<&str> = fields.iter().take(SHOWN).map(|field| field.name.as_str()).collect();
    if fields.len() > SHOWN {
        names.push("...");
    }
    names.join(", ")
}

struct PipelineCounts<T> {
    read: u64,
    written: u64,
    outputs: T,
}

/// Runs reader, filters and writer concurrently, connected by small queues.
///
/// When a stage fails, the stages before it notice that their queue is closed and stop, so the
/// error that is returned is always the one that caused the stop.
fn run_pipeline<T>(
    reader: Box<dyn PointReader>,
    mut filters: Vec<Box<dyn PointFilter>>,
    attributes: Attributes,
    write: impl FnOnce(&mut dyn Iterator<Item = PointChunk>) -> Result<T>,
    progress: &Progress,
) -> Result<PipelineCounts<T>> {
    let (read_tx, read_rx) = bounded::<PointChunk>(QUEUE_CHUNKS);
    let (filtered_tx, filtered_rx) = bounded::<PointChunk>(QUEUE_CHUNKS);

    thread::scope(|scope| {
        let reading = scope.spawn(move || {
            let mut read = 0u64;
            let mut emit = |chunk: PointChunk| {
                progress.check_cancelled()?;
                read += chunk.len() as u64;
                read_tx.send(chunk).map_err(|_| Error::OutputStopped)
            };
            reader.read(&mut emit, progress).map(|()| read)
        });

        let filtering = scope.spawn(move || -> Result<()> {
            for mut chunk in read_rx {
                for filter in &mut filters {
                    filter.apply(&mut chunk)?;
                }
                if chunk.is_empty() {
                    continue;
                }
                chunk.conform_to(attributes);
                filtered_tx.send(chunk).map_err(|_| Error::OutputStopped)?;
            }
            Ok(())
        });

        let mut written = 0u64;
        let mut counted = filtered_rx.iter().inspect(|chunk| written += chunk.len() as u64);
        let outputs = write(&mut counted);
        drop(counted);
        drop(filtered_rx); // unblocks the earlier stages if writing stopped early

        let read = reading.join().unwrap_or_else(|_| Err(Error::invalid("the reader crashed")));
        let filtered = filtering.join().unwrap_or_else(|_| Err(Error::invalid("point filtering crashed")));
        let outputs = outputs?;
        filtered?;
        let read = read?;
        progress.check_cancelled()?;
        Ok(PipelineCounts { read, written, outputs })
    })
    .map_err(|error| match error {
        // A stage only stops for this reason if a later stage failed without an error, which
        // happens when the user cancels.
        Error::OutputStopped => Error::Cancelled,
        error => error,
    })
}

fn output_stem(input: &Path) -> String {
    let stem = Format::stem_of(input);
    if stem.is_empty() { "output".into() } else { stem }
}

/// Writes `<output>.log` next to the first output.
fn write_log(job: &ConversionJob, report: &ConversionReport) -> Result<()> {
    let Some(first) = report.outputs.first() else {
        return Ok(());
    };
    let mut text = String::new();
    let _ = writeln!(text, "EPCC {}", crate::VERSION);
    let _ = writeln!(text, "Input:    {}", job.input.display());
    for output in &report.outputs {
        let _ = writeln!(text, "Output:   {}", output.display());
    }
    let _ = writeln!(text, "Points:   {} read, {} written", report.points_read, report.points_written);
    let _ = writeln!(text, "Duration: {:.1} s", report.duration.as_secs_f64());
    for warning in &report.warnings {
        let _ = writeln!(text, "Warning:  {warning}");
    }
    let mut path = first.clone().into_os_string();
    path.push(".log");
    let path = PathBuf::from(path);
    std::fs::write(&path, text).map_err(|source| Error::Io { path, source })
}
