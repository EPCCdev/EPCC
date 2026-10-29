//! User settings, shared by the command line and the desktop interface.
//!
//! Every value has a built-in default (the `Default` impls below). The settings file only
//! overrides them, and values missing from the file fall back to the defaults, so older files
//! keep working when new settings are added.
//!
//! The file lives next to the program, which keeps it portable. If that folder is not writable
//! (e.g. Program Files), the user's configuration folder is used instead.

use std::fs;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::error::IoContext;
use crate::model::Attributes;
use crate::{Error, Format, Result};

const FILE_NAME: &str = "epcc-settings.toml";
/// The subfolder of the download that holds the command line tool.
const COMMAND_LINE_FOLDER: &str = "command-line";

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    pub general: GeneralSettings,
    pub import: ImportSettings,
    pub processing: ProcessingSettings,
    pub export: ExportSettings,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct GeneralSettings {
    /// Folder the file picker opens in.
    pub input_folder: Option<PathBuf>,
    /// Default output folder. Without one, outputs go next to their input file.
    pub output_folder: Option<PathBuf>,
    /// Whether exports go to the default output folder or to a folder picked per export.
    pub use_output_folder: bool,
    /// The output format selected last.
    pub output_format: Format,
    pub existing_files: ExistingFiles,
    /// Write a `.log` file next to every output.
    pub write_log: bool,
    /// How many files a batch converts at the same time; 0 picks a value from the CPU count.
    pub parallel_files: usize,
}

impl Default for GeneralSettings {
    fn default() -> Self {
        GeneralSettings {
            input_folder: None,
            output_folder: None,
            use_output_folder: true,
            output_format: Format::Laz,
            existing_files: ExistingFiles::Rename,
            write_log: false,
            parallel_files: 0,
        }
    }
}

/// What to do when an output file already exists. The input file is never overwritten.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ExistingFiles {
    /// Add a number to the new file's name: `scan (1).laz`.
    #[default]
    Rename,
    Overwrite,
    Skip,
}

impl ExistingFiles {
    pub const ALL: [ExistingFiles; 3] = [ExistingFiles::Rename, ExistingFiles::Overwrite, ExistingFiles::Skip];

    pub fn label(self) -> &'static str {
        match self {
            ExistingFiles::Rename => "Keep both (add a number)",
            ExistingFiles::Overwrite => "Overwrite",
            ExistingFiles::Skip => "Skip the file",
        }
    }
}

/// How text inputs (PTS, XYZ) are read. Other formats describe themselves.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ImportSettings {
    pub text_units: LengthUnit,
    /// Column layout of XYZ files, e.g. `x,y,z,r,g,b`. Empty means detect it from the file.
    pub text_columns: String,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum LengthUnit {
    #[default]
    Metres,
    Centimetres,
    Millimetres,
    Feet,
    #[serde(rename = "us-survey-feet")]
    UsSurveyFeet,
    Inches,
}

impl LengthUnit {
    pub const ALL: [LengthUnit; 6] = [
        LengthUnit::Metres,
        LengthUnit::Centimetres,
        LengthUnit::Millimetres,
        LengthUnit::Feet,
        LengthUnit::UsSurveyFeet,
        LengthUnit::Inches,
    ];

    pub fn in_metres(self) -> f64 {
        match self {
            LengthUnit::Metres => 1.0,
            LengthUnit::Centimetres => 0.01,
            LengthUnit::Millimetres => 0.001,
            LengthUnit::Feet => 0.3048,
            LengthUnit::UsSurveyFeet => 1200.0 / 3937.0,
            LengthUnit::Inches => 0.0254,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            LengthUnit::Metres => "Metres",
            LengthUnit::Centimetres => "Centimetres",
            LengthUnit::Millimetres => "Millimetres",
            LengthUnit::Feet => "Feet",
            LengthUnit::UsSurveyFeet => "US survey feet",
            LengthUnit::Inches => "Inches",
        }
    }

    pub fn from_name(name: &str) -> Option<LengthUnit> {
        let unit = match name.to_ascii_lowercase().as_str() {
            "m" | "metres" | "meters" => LengthUnit::Metres,
            "cm" | "centimetres" | "centimeters" => LengthUnit::Centimetres,
            "mm" | "millimetres" | "millimeters" => LengthUnit::Millimetres,
            "ft" | "feet" => LengthUnit::Feet,
            "usft" | "us-survey-feet" => LengthUnit::UsSurveyFeet,
            "in" | "inches" => LengthUnit::Inches,
            _ => return None,
        };
        Some(unit)
    }
}

/// Applies to every export, whatever the output format.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ProcessingSettings {
    /// Minimum distance between kept points, in metres. `None` keeps every point.
    pub min_spacing: Option<f64>,
    /// Keep only every n-th point. `None` keeps every point.
    pub every_nth: Option<u64>,
    /// Attributes left out of the output.
    pub drop: Attributes,
    /// For formats that cannot store several scans: one file, or one file per scan.
    pub scans: ScanMode,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ScanMode {
    #[default]
    Merge,
    Split,
}

/// Per-format export options.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ExportSettings {
    /// Used for both LAS and LAZ.
    pub las: LasSettings,
    pub e57: E57Settings,
    pub pts: TextSettings,
    pub xyz: XyzSettings,
    pub ply: PlySettings,
    pub rcs: RcsSettings,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct LasSettings {
    pub version: LasVersion,
    /// Coordinate resolution in metres. LAS stores coordinates as integers of this step.
    pub precision: f64,
}

impl Default for LasSettings {
    fn default() -> Self {
        LasSettings { version: LasVersion::Auto, precision: 0.001 }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum LasVersion {
    /// LAS 1.2 for the widest compatibility, LAS 1.4 when the cloud needs it.
    #[default]
    Auto,
    #[serde(rename = "1.2")]
    V1_2,
    #[serde(rename = "1.4")]
    V1_4,
}

impl LasVersion {
    pub const ALL: [LasVersion; 3] = [LasVersion::Auto, LasVersion::V1_2, LasVersion::V1_4];

    pub fn label(self) -> &'static str {
        match self {
            LasVersion::Auto => "Automatic",
            LasVersion::V1_2 => "1.2",
            LasVersion::V1_4 => "1.4",
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct E57Settings {
    /// Copy panorama and camera images when converting E57 to E57. They are often most of the
    /// file size, and no other format can hold them.
    pub keep_images: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct TextSettings {
    /// Decimal places written for coordinates.
    pub decimals: u8,
}

impl Default for TextSettings {
    fn default() -> Self {
        TextSettings { decimals: 3 }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct XyzSettings {
    pub decimals: u8,
    pub separator: Separator,
}

impl Default for XyzSettings {
    fn default() -> Self {
        XyzSettings { decimals: 3, separator: Separator::Space }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Separator {
    #[default]
    Space,
    Comma,
    Semicolon,
    Tab,
}

impl Separator {
    pub const ALL: [Separator; 4] = [Separator::Space, Separator::Comma, Separator::Semicolon, Separator::Tab];

    pub fn byte(self) -> u8 {
        match self {
            Separator::Space => b' ',
            Separator::Comma => b',',
            Separator::Semicolon => b';',
            Separator::Tab => b'\t',
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Separator::Space => "Space",
            Separator::Comma => "Comma",
            Separator::Semicolon => "Semicolon",
            Separator::Tab => "Tab",
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PlySettings {
    pub encoding: PlyEncoding,
    pub coordinates: CoordinateType,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum PlyEncoding {
    #[default]
    Binary,
    Ascii,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CoordinateType {
    /// 32-bit floats when they are precise enough for the cloud's coordinates, else 64-bit.
    #[default]
    Auto,
    Float32,
    Float64,
}

impl CoordinateType {
    pub const ALL: [CoordinateType; 3] = [CoordinateType::Auto, CoordinateType::Float32, CoordinateType::Float64];

    pub fn label(self) -> &'static str {
        match self {
            CoordinateType::Auto => "Automatic",
            CoordinateType::Float32 => "32-bit float",
            CoordinateType::Float64 => "64-bit float",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct RcsSettings {
    /// Estimate normals from the points when the source has none.
    pub estimate_normals: bool,
}

impl Default for RcsSettings {
    fn default() -> Self {
        RcsSettings { estimate_normals: true }
    }
}

impl Settings {
    /// Loads the saved settings. A missing file gives the defaults.
    pub fn load() -> Result<Settings> {
        let Some(path) = existing_settings_file() else {
            return Ok(Settings::default());
        };
        let text = fs::read_to_string(&path).at(&path)?;
        toml::from_str(&text).map_err(|error| Error::InvalidSetting(format!("{}: {}", path.display(), error.message())))
    }

    /// Saves next to the program, or in the user's configuration folder if that fails.
    /// Returns where the file was written.
    pub fn save(&self) -> Result<PathBuf> {
        let text = toml::to_string_pretty(self).map_err(|error| Error::InvalidSetting(error.to_string()))?;
        let mut last_error = None;
        for path in candidate_files() {
            match write_file(&path, &text) {
                Ok(()) => return Ok(path),
                Err(error) => last_error = Some(error),
            }
        }
        Err(last_error.unwrap_or_else(|| Error::InvalidSetting("no folder to save settings in".into())))
    }

    /// Checks values that the types alone cannot guarantee.
    pub fn validate(&self) -> Result<()> {
        if let Some(spacing) = self.processing.min_spacing
            && !(spacing > 0.0 && spacing.is_finite())
        {
            return Err(Error::InvalidSetting("Minimum point spacing must be greater than 0.".into()));
        }
        if self.processing.every_nth == Some(0) {
            return Err(Error::InvalidSetting("\"Keep every n-th point\" must be at least 1.".into()));
        }
        let precision = self.export.las.precision;
        if !(precision > 0.0 && precision <= 1.0) {
            return Err(Error::InvalidSetting("LAS precision must be between 0 and 1 metre.".into()));
        }
        for decimals in [self.export.pts.decimals, self.export.xyz.decimals] {
            if decimals > 9 {
                return Err(Error::InvalidSetting("Text formats support at most 9 decimals.".into()));
            }
        }
        Ok(())
    }
}

fn write_file(path: &Path, text: &str) -> Result<()> {
    if let Some(folder) = path.parent() {
        fs::create_dir_all(folder).at(folder)?;
    }
    fs::write(path, text).at(path)
}

fn existing_settings_file() -> Option<PathBuf> {
    candidate_files().into_iter().find(|path| path.is_file())
}

/// Where the settings file may live, most preferred first.
fn candidate_files() -> Vec<PathBuf> {
    let program_folder = std::env::current_exe().ok().and_then(|exe| exe.parent().map(program_folder_of));
    let next_to_program = program_folder.map(|folder| folder.join(FILE_NAME));
    let in_user_config = user_config_folder().map(|folder| folder.join(FILE_NAME));
    next_to_program.into_iter().chain(in_user_config).collect()
}

/// The folder of the portable program, which the executables in the download share: the
/// command line tool sits in a subfolder of it, and on macOS the window program inside
/// `EPCC.app/Contents/MacOS`.
fn program_folder_of(executable_folder: &Path) -> PathBuf {
    let ancestor = |levels: usize| executable_folder.ancestors().nth(levels).map(Path::to_path_buf);
    let name_of = |levels: usize| executable_folder.ancestors().nth(levels).and_then(Path::file_name);
    let in_app_bundle = name_of(0).is_some_and(|name| name == "MacOS")
        && name_of(1).is_some_and(|name| name == "Contents")
        && name_of(2).is_some_and(|name| name.to_string_lossy().ends_with(".app"));
    let found = if in_app_bundle {
        ancestor(3)
    } else if name_of(0).is_some_and(|name| name == COMMAND_LINE_FOLDER) {
        ancestor(1)
    } else {
        None
    };
    found.unwrap_or_else(|| executable_folder.to_path_buf())
}

fn user_config_folder() -> Option<PathBuf> {
    let env_path = |name: &str| std::env::var_os(name).filter(|value| !value.is_empty()).map(PathBuf::from);
    if cfg!(windows) {
        env_path("APPDATA").map(|folder| folder.join("EPCC"))
    } else if cfg!(target_os = "macos") {
        env_path("HOME").map(|home| home.join("Library/Application Support/EPCC"))
    } else {
        env_path("XDG_CONFIG_HOME")
            .or_else(|| env_path("HOME").map(|home| home.join(".config")))
            .map(|folder| folder.join("epcc"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn partial_file_falls_back_to_defaults() {
        let settings: Settings = toml::from_str("[export.las]\nprecision = 0.0001\n").unwrap();
        assert_eq!(settings.export.las.precision, 0.0001);
        assert_eq!(settings.export.las.version, LasVersion::Auto);
        assert_eq!(settings.general, GeneralSettings::default());
    }

    #[test]
    fn the_command_line_tool_shares_the_program_folder() {
        let top = Path::new("EPCC");
        assert_eq!(program_folder_of(&top.join(COMMAND_LINE_FOLDER)), top);
        assert_eq!(program_folder_of(top), top);
        assert_eq!(program_folder_of(&top.join("other")), top.join("other"));
        assert_eq!(program_folder_of(&top.join("EPCC.app").join("Contents").join("MacOS")), top);
    }

    #[test]
    fn settings_survive_a_round_trip() {
        let mut settings = Settings::default();
        settings.processing.min_spacing = Some(0.005);
        settings.processing.drop.normal = true;
        settings.processing.drop.extra_bytes = 1;
        settings.general.output_folder = Some(PathBuf::from("out"));
        let text = toml::to_string_pretty(&settings).unwrap();
        assert!(text.contains("extra_values = true"), "{text}");
        assert_eq!(toml::from_str::<Settings>(&text).unwrap(), settings);
    }
}
