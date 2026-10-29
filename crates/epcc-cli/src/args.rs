//! Command line parsing. Options override the saved settings for this run only.

use std::path::PathBuf;
use std::str::FromStr;

use epcc_core::Format;
use epcc_core::settings::{ExistingFiles, LasVersion, LengthUnit, ScanMode, Settings};
use lexopt::prelude::*;

pub const USAGE: &str = "\
Usage: epcc [options] <input>... --to <format>

Converts point cloud files. Inputs can be files, folders or patterns like *.e57.
A Potree folder counts as one input; Potree output is a folder too.
Settings saved by the EPCC window are used unless an option overrides them.

Options:
  -t, --to <format>        Output format: las, laz, copc, e57, pts, xyz, ply, potree, rcs
  -o, --output <folder>    Output folder (default: the saved one, else next to each input)
      --min-spacing <m>    Keep no two points closer than this many metres
      --every <n>          Keep only every n-th point
      --drop <list>        Leave out attributes: intensity, color, normals, classification, time,
                           returns, lidar (scan angle, point source, user data, flags), extra
      --scans <mode>       merge or split: one file, or one per scan, for formats without scans
      --if-exists <mode>   rename, overwrite or skip
      --precision <m>      LAS/LAZ/COPC/Potree coordinate precision in metres
      --las-version <v>    auto, 1.2 or 1.4
      --decimals <n>       Decimal places for PTS and XYZ output
      --units <unit>       Unit of PTS/XYZ input: m, cm, mm, ft, usft, in
      --columns <list>     Column layout of XYZ input, e.g. x,y,z,i,r,g,b
      --keep-images        Copy images when converting E57 to E57
      --no-rcs-normals     Do not estimate normals for RCS output when the source has none
      --log                Write a .log file next to each output
      --jobs <n>           Files converted at the same time (0 = automatic)
      --factory-defaults   Ignore the saved settings
  -q, --quiet              Only print errors
      --formats            List the supported formats
  -h, --help               Show this help
  -V, --version            Show the version";

pub enum Command {
    Convert(ConvertArgs),
    ListFormats,
    Help,
    Version,
}

pub struct ConvertArgs {
    pub inputs: Vec<PathBuf>,
    pub format: Format,
    pub output_folder: Option<PathBuf>,
    pub factory_defaults: bool,
    pub quiet: bool,
    /// Applied on top of the saved (or factory) settings.
    pub overrides: Vec<Override>,
}

/// One option that changes a setting.
pub enum Override {
    MinSpacing(f64),
    EveryNth(u64),
    Drop(Vec<String>),
    Scans(ScanMode),
    IfExists(ExistingFiles),
    Precision(f64),
    LasVersion(LasVersion),
    Decimals(u8),
    Units(LengthUnit),
    Columns(String),
    KeepImages,
    NoRcsNormals,
    Log,
    Jobs(usize),
}

impl ConvertArgs {
    pub fn apply(&self, settings: &mut Settings) -> Result<(), String> {
        for option in &self.overrides {
            match option {
                Override::MinSpacing(metres) => settings.processing.min_spacing = Some(*metres),
                Override::EveryNth(n) => settings.processing.every_nth = Some(*n),
                Override::Drop(names) => {
                    for name in names {
                        let drop = &mut settings.processing.drop;
                        match name.as_str() {
                            "intensity" => drop.intensity = true,
                            "color" | "colour" | "rgb" => drop.color = true,
                            "normal" | "normals" => drop.normal = true,
                            "classification" | "class" => drop.classification = true,
                            "time" => drop.time = true,
                            "returns" | "return" => drop.returns = true,
                            "lidar" => drop.lidar = true,
                            "extra" | "extra-values" | "extra-bytes" => drop.extra_bytes = 1,
                            other => return Err(format!("--drop: unknown attribute \"{other}\"")),
                        }
                    }
                }
                Override::Scans(mode) => settings.processing.scans = *mode,
                Override::IfExists(mode) => settings.general.existing_files = *mode,
                Override::Precision(metres) => settings.export.las.precision = *metres,
                Override::LasVersion(version) => settings.export.las.version = *version,
                Override::Decimals(decimals) => {
                    settings.export.pts.decimals = *decimals;
                    settings.export.xyz.decimals = *decimals;
                }
                Override::Units(unit) => settings.import.text_units = *unit,
                Override::Columns(columns) => settings.import.text_columns = columns.clone(),
                Override::KeepImages => settings.export.e57.keep_images = true,
                Override::NoRcsNormals => settings.export.rcs.estimate_normals = false,
                Override::Log => settings.general.write_log = true,
                Override::Jobs(jobs) => settings.general.parallel_files = *jobs,
            }
        }
        Ok(())
    }
}

pub fn parse(args: impl IntoIterator<Item = std::ffi::OsString>) -> Result<Command, String> {
    parse_inner(lexopt::Parser::from_args(args)).map_err(|error| error.to_string())
}

fn parse_inner(mut parser: lexopt::Parser) -> Result<Command, lexopt::Error> {
    let mut inputs = Vec::new();
    let mut format = None;
    let mut output_folder = None;
    let mut factory_defaults = false;
    let mut quiet = false;
    let mut overrides = Vec::new();

    while let Some(arg) = parser.next()? {
        match arg {
            Short('h') | Long("help") => return Ok(Command::Help),
            Short('V') | Long("version") => return Ok(Command::Version),
            Long("formats") => return Ok(Command::ListFormats),
            Short('t') | Long("to") => {
                let name = parser.value()?.string()?;
                format =
                    Some(Format::from_name(&name).ok_or_else(|| format!("unknown format \"{name}\"; see --formats"))?);
            }
            Short('o') | Long("output") => output_folder = Some(PathBuf::from(parser.value()?)),
            Long("min-spacing") => overrides.push(Override::MinSpacing(number(&mut parser)?)),
            Long("every") => overrides.push(Override::EveryNth(number(&mut parser)?)),
            Long("drop") => {
                let list = parser.value()?.string()?.to_ascii_lowercase();
                overrides.push(Override::Drop(list.split(',').map(|name| name.trim().to_string()).collect()));
            }
            Long("scans") => overrides
                .push(Override::Scans(choice(&mut parser, &[("merge", ScanMode::Merge), ("split", ScanMode::Split)])?)),
            Long("if-exists") => overrides.push(Override::IfExists(choice(
                &mut parser,
                &[
                    ("rename", ExistingFiles::Rename),
                    ("overwrite", ExistingFiles::Overwrite),
                    ("skip", ExistingFiles::Skip),
                ],
            )?)),
            Long("precision") => overrides.push(Override::Precision(number(&mut parser)?)),
            Long("las-version") => overrides.push(Override::LasVersion(choice(
                &mut parser,
                &[("auto", LasVersion::Auto), ("1.2", LasVersion::V1_2), ("1.4", LasVersion::V1_4)],
            )?)),
            Long("decimals") => overrides.push(Override::Decimals(number(&mut parser)?)),
            Long("units") => {
                let name = parser.value()?.string()?;
                let unit = LengthUnit::from_name(&name).ok_or_else(|| format!("unknown unit \"{name}\""))?;
                overrides.push(Override::Units(unit));
            }
            Long("columns") => overrides.push(Override::Columns(parser.value()?.string()?)),
            Long("keep-images") => overrides.push(Override::KeepImages),
            Long("no-rcs-normals") => overrides.push(Override::NoRcsNormals),
            Long("log") => overrides.push(Override::Log),
            Long("jobs") => overrides.push(Override::Jobs(number(&mut parser)?)),
            Long("factory-defaults") => factory_defaults = true,
            Short('q') | Long("quiet") => quiet = true,
            Value(value) => inputs.push(PathBuf::from(value)),
            _ => return Err(arg.unexpected()),
        }
    }

    if inputs.is_empty() {
        return Err("no input files given".into());
    }
    let format = format.ok_or("no output format given; use --to <format>")?;
    Ok(Command::Convert(ConvertArgs { inputs, format, output_folder, factory_defaults, quiet, overrides }))
}

fn number<T>(parser: &mut lexopt::Parser) -> Result<T, lexopt::Error>
where
    T: FromStr,
    T::Err: std::error::Error + Send + Sync + 'static,
{
    parser.value()?.parse()
}

fn choice<T: Copy>(parser: &mut lexopt::Parser, options: &[(&str, T)]) -> Result<T, lexopt::Error> {
    let value = parser.value()?.string()?.to_ascii_lowercase();
    options.iter().find(|(name, _)| *name == value).map(|&(_, option)| option).ok_or_else(|| {
        let names: Vec<&str> = options.iter().map(|(name, _)| *name).collect();
        format!("\"{value}\" is not one of {}", names.join(", ")).into()
    })
}
