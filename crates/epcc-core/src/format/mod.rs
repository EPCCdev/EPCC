//! The supported file formats.
//!
//! Adding a format means adding a module with a reader and/or writer, and a variant to
//! [`Format`]. The compiler then points out every `match` that needs to learn about it.

mod copc;
mod e57;
mod las;
mod ply;
mod potree;
mod pts;
mod rcs;
mod text;
mod xyz;

use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::model::{Attributes, CloudInfo, PointChunk};
use crate::settings::{ExportSettings, ImportSettings};
use crate::{Progress, Result};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Format {
    Las,
    #[default]
    Laz,
    E57,
    Pts,
    Xyz,
    Ply,
    Copc,
    Rcs,
    Potree,
}

impl Format {
    pub const ALL: [Format; 9] = [
        Format::Las,
        Format::Laz,
        Format::Copc,
        Format::E57,
        Format::Pts,
        Format::Xyz,
        Format::Ply,
        Format::Potree,
        Format::Rcs,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Format::Las => "LAS",
            Format::Laz => "LAZ",
            Format::Copc => "COPC",
            Format::E57 => "E57",
            Format::Pts => "PTS",
            Format::Xyz => "XYZ",
            Format::Ply => "PLY",
            Format::Rcs => "RCS",
            Format::Potree => "Potree",
        }
    }

    pub fn description(self) -> &'static str {
        match self {
            Format::Las => "LAS (uncompressed)",
            Format::Laz => "LAZ (compressed LAS)",
            Format::Copc => "COPC (LAZ for streaming viewers)",
            Format::E57 => "E57 (ASTM E2807)",
            Format::Pts => "PTS (Leica text)",
            Format::Xyz => "XYZ (text columns)",
            Format::Ply => "PLY (points only)",
            Format::Rcs => "RCS (experimental)",
            Format::Potree => "Potree (web viewer folder)",
        }
    }

    /// The extension outputs get. Potree outputs are folders, named like files.
    pub fn extension(self) -> &'static str {
        match self {
            Format::Las => "las",
            Format::Laz => "laz",
            Format::Copc => "copc.laz",
            Format::E57 => "e57",
            Format::Pts => "pts",
            Format::Xyz => "xyz",
            Format::Ply => "ply",
            Format::Rcs => "rcs",
            Format::Potree => potree::FOLDER_EXTENSION,
        }
    }

    /// Whether outputs are folders rather than files.
    pub fn writes_folders(self) -> bool {
        self == Format::Potree
    }

    /// All extensions recognised on input.
    pub fn input_extensions(self) -> &'static [&'static str] {
        match self {
            Format::Las => &["las"],
            Format::Laz => &["laz"],
            Format::Copc => &["copc.laz"],
            Format::E57 => &["e57"],
            Format::Pts => &["pts"],
            Format::Xyz => &["xyz", "txt", "asc", "csv"],
            Format::Ply => &["ply"],
            Format::Rcs => &["rcs"],
            // Recognised by their folder or its description file instead, see `from_path`.
            Format::Potree => &[],
        }
    }

    /// The format of a file, or of a folder that holds a Potree cloud.
    pub fn from_path(path: &Path) -> Option<Format> {
        if potree::is_cloud(path) {
            return Some(Format::Potree);
        }
        if path.is_dir() {
            return None;
        }
        recognised_extension(path).map(|(format, _)| format)
    }

    /// File names, besides extensions, that a file picker should offer.
    pub fn input_file_names(self) -> &'static [&'static str] {
        match self {
            Format::Potree => &[potree::METADATA_FILE, potree::V1_FILE],
            _ => &[],
        }
    }

    /// The file name without its format's extension, so `scan.copc.laz` gives `scan`. A Potree
    /// cloud is named after its folder.
    pub fn stem_of(path: &Path) -> String {
        if potree::is_cloud(path) {
            return potree::cloud_name(path);
        }
        let name = path.file_name().map(|name| name.to_string_lossy().into_owned()).unwrap_or_default();
        match recognised_extension(path) {
            Some((_, extension)) => name[..name.len() - extension.len() - 1].to_string(),
            None => path.file_stem().map(|stem| stem.to_string_lossy().into_owned()).unwrap_or_default(),
        }
    }

    pub fn from_name(name: &str) -> Option<Format> {
        Format::ALL.into_iter().find(|format| format.name().eq_ignore_ascii_case(name))
    }

    /// The attributes this format can store.
    pub fn stored_attributes(self) -> Attributes {
        let base = Attributes { intensity: true, color: true, ..Attributes::NONE };
        match self {
            Format::Las | Format::Laz | Format::Copc => {
                Attributes { classification: true, time: true, returns: true, lidar: true, extra_bytes: 1, ..base }
            }
            Format::E57 => Attributes { normal: true, time: true, ..base },
            Format::Pts => base,
            Format::Xyz => Attributes { normal: true, ..base },
            Format::Ply | Format::Potree => Attributes::ALL,
            Format::Rcs => Attributes { normal: true, ..base },
        }
    }

    /// Whether one file can hold several scans, each with its own name and pose.
    pub fn stores_scans(self) -> bool {
        self == Format::E57
    }

    /// The part of a conversion's work that happens after the input is read. Formats that
    /// sort their points spatially can only start once they have seen all of them.
    pub fn work_after_reading(self) -> f32 {
        match self {
            Format::Copc | Format::Potree => 0.75,
            Format::Rcs => 0.6,
            _ => 0.0,
        }
    }

    pub fn open(self, path: &Path, settings: &ImportSettings) -> Result<Box<dyn PointReader>> {
        match self {
            Format::Las | Format::Laz | Format::Copc => las::open(path),
            Format::E57 => e57::open(path),
            Format::Pts => pts::open(path, settings),
            Format::Xyz => xyz::open(path, settings),
            Format::Ply => ply::open(path),
            Format::Rcs => rcs::open(path),
            Format::Potree => potree::open(path),
        }
    }

    pub fn create_writer(self, request: &WriteRequest) -> Result<Box<dyn PointWriter>> {
        match self {
            Format::Las => las::create(request, false),
            Format::Laz => las::create(request, true),
            Format::Copc => copc::create(request),
            Format::E57 => e57::create(request),
            Format::Pts => pts::create(request),
            Format::Xyz => xyz::create(request),
            Format::Ply => ply::create(request),
            Format::Rcs => rcs::create(request),
            Format::Potree => potree::create(request),
        }
    }
}

/// Whether `folder` holds a point cloud as a folder format writes it.
pub(crate) fn is_cloud_folder(folder: &Path) -> bool {
    folder.is_dir() && potree::is_cloud(folder)
}

/// The format of `path` and the extension that identified it. The longest matching extension
/// wins, so `.copc.laz` is COPC rather than LAZ.
fn recognised_extension(path: &Path) -> Option<(Format, &'static str)> {
    let name = path.file_name()?.to_str()?.to_ascii_lowercase();
    Format::ALL
        .into_iter()
        .flat_map(|format| format.input_extensions().iter().map(move |&extension| (format, extension)))
        .filter(|(_, extension)| name.len() > extension.len() + 1 && name.ends_with(&format!(".{extension}")))
        .max_by_key(|(_, extension)| extension.len())
}

impl std::fmt::Display for Format {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.name())
    }
}

/// An opened input file.
pub trait PointReader: Send {
    fn info(&self) -> &CloudInfo;

    /// Streams every point into `emit`, scan after scan, and reports progress. Stops at the first
    /// error `emit` returns.
    fn read(self: Box<Self>, emit: &mut dyn FnMut(PointChunk) -> Result<()>, progress: &Progress) -> Result<()>;
}

/// A started output file.
pub trait PointWriter: Send {
    /// Writes every chunk and finishes the file. Chunks carry exactly the attributes of
    /// [`WriteRequest::cloud`].
    fn write(self: Box<Self>, chunks: &mut dyn Iterator<Item = PointChunk>, progress: &Progress) -> Result<()>;
}

/// Everything a writer needs to know before the first point arrives.
pub struct WriteRequest<'a> {
    /// Where to write. This is a temporary name; the caller renames it when the file is complete.
    /// Formats that write folders create the folder there.
    pub path: &'a Path,
    pub cloud: &'a CloudInfo,
    pub settings: &'a ExportSettings,
    /// The file being converted, for writers that copy extra data from their own format.
    pub source: &'a Path,
    pub source_format: Format,
}
