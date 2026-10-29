//! `epcc`: the command line front end, for terminals and scripts.
//!
//! Exit codes: 0 when every file was converted or skipped, 1 when any file failed, 2 for
//! invalid arguments or settings.

mod args;
mod inputs;

use std::io::{IsTerminal, Write};
use std::path::Path;
use std::process::ExitCode;
use std::sync::Arc;
use std::time::Duration;

use args::{Command, ConvertArgs};
use epcc_core::{ConversionJob, Format, Outcome, Progress, Result, Settings, run_batch};

fn main() -> ExitCode {
    let command = match args::parse(std::env::args_os().skip(1)) {
        Ok(command) => command,
        Err(message) => {
            eprintln!("epcc: {message}\n\n{}", args::USAGE);
            return ExitCode::from(2);
        }
    };
    match command {
        Command::Help => println!("{}", args::USAGE),
        Command::Version => println!("epcc {}", epcc_core::VERSION),
        Command::ListFormats => list_formats(),
        Command::Convert(convert) => return run(&convert),
    }
    ExitCode::SUCCESS
}

fn list_formats() {
    for format in Format::ALL {
        let reads = if format.input_extensions().is_empty() {
            format!("folders with {}", format.input_file_names().join(" or "))
        } else {
            format!(".{}", format.input_extensions().join(", ."))
        };
        println!("{:<6} {:<26} reads {reads}", format.name().to_lowercase(), format.description());
    }
}

fn run(args: &ConvertArgs) -> ExitCode {
    let settings = match settings_for(args) {
        Ok(settings) => settings,
        Err(message) => {
            eprintln!("epcc: {message}");
            return ExitCode::from(2);
        }
    };
    let files = match inputs::expand(&args.inputs) {
        Ok(files) => files,
        Err(message) => {
            eprintln!("epcc: {message}");
            return ExitCode::from(2);
        }
    };
    let jobs: Vec<ConversionJob> = files
        .into_iter()
        .map(|input| ConversionJob {
            output_folder: output_folder(args, &settings, &input),
            input,
            format: args.format,
        })
        .collect();

    let progress: Vec<Arc<Progress>> = jobs.iter().map(|_| Arc::new(Progress::new())).collect();
    let show_progress = !args.quiet && std::io::stderr().is_terminal();
    let quiet = args.quiet;
    let results = std::thread::scope(|scope| {
        let batch = scope.spawn(|| {
            run_batch(&jobs, &settings, &progress, &|index, result| {
                report(&jobs[index].input, result, quiet, show_progress);
            })
        });
        while show_progress && !batch.is_finished() {
            draw_progress(&progress);
            std::thread::sleep(Duration::from_millis(200));
        }
        batch.join().expect("conversion thread panicked")
    });
    if show_progress {
        clear_progress_line();
    }

    let failed = results.iter().filter(|result| result.is_err()).count();
    if !quiet && jobs.len() > 1 {
        eprintln!("{} of {} files converted, {} failed.", jobs.len() - failed, jobs.len(), failed);
    }
    if failed > 0 { ExitCode::from(1) } else { ExitCode::SUCCESS }
}

fn settings_for(args: &ConvertArgs) -> std::result::Result<Settings, String> {
    let mut settings = if args.factory_defaults {
        Settings::default()
    } else {
        Settings::load().map_err(|error| format!("{error} (use --factory-defaults to ignore it)"))?
    };
    args.apply(&mut settings)?;
    settings.validate().map_err(|error| error.to_string())?;
    Ok(settings)
}

/// `--output`, else the saved default folder, else the input's own folder.
fn output_folder(args: &ConvertArgs, settings: &Settings, input: &Path) -> std::path::PathBuf {
    let saved = settings.general.output_folder.as_ref().filter(|_| settings.general.use_output_folder);
    args.output_folder
        .as_ref()
        .or(saved)
        .cloned()
        .unwrap_or_else(|| input.parent().map(Path::to_path_buf).unwrap_or_default())
}

fn report(input: &Path, result: &Result<Outcome>, quiet: bool, progress_shown: bool) {
    if progress_shown {
        clear_progress_line();
    }
    match result {
        Ok(Outcome::Converted(report)) if !quiet => {
            for output in &report.outputs {
                eprintln!("OK       {} -> {}", input.display(), output.display());
            }
            eprintln!(
                "         {} points read, {} written, {:.1} s",
                report.points_read,
                report.points_written,
                report.duration.as_secs_f64()
            );
            for warning in &report.warnings {
                eprintln!("         warning: {warning}");
            }
        }
        Ok(Outcome::Skipped(existing)) if !quiet => {
            eprintln!("SKIPPED  {} ({} exists)", input.display(), existing.display());
        }
        Ok(_) => {}
        Err(error) => eprintln!("FAILED   {}: {error}", input.display()),
    }
}

fn draw_progress(progress: &[Arc<Progress>]) {
    let total: f32 = progress.iter().map(|p| p.fraction()).sum();
    let percent = 100.0 * total / progress.len() as f32;
    let width = 30;
    let filled = (percent / 100.0 * width as f32) as usize;
    eprint!("\r[{}{}] {percent:5.1} %", "#".repeat(filled), " ".repeat(width - filled.min(width)));
    let _ = std::io::stderr().flush();
}

fn clear_progress_line() {
    eprint!("\r{:50}\r", "");
}
