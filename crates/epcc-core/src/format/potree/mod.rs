//! Potree: a folder holding a point cloud as an octree, for the Potree web viewer.
//!
//! **Potree 2.0** is written and read. The folder holds three files:
//! - `metadata.json`: point count, bounding cube, spacing, the scale and offset of the integer
//!   positions, the hierarchy's first chunk size, the encoding and the attribute list;
//! - `octree.bin`: the points of every node, one node after another. A node's points are
//!   records of all attributes in the order of the list, little-endian;
//! - `hierarchy.bin`: the nodes, 22 bytes each (type, child mask, point count, byte offset and
//!   size in `octree.bin`), in chunks a few levels deep. Each chunk lists its nodes breadth
//!   first, children in octant order; a node at a chunk's lower edge that has children is a
//!   proxy pointing to the chunk below it.
//!
//! Like in COPC, every point is in exactly one node; inner nodes hold a thinned-out sample of
//! the points below them. Only the uncompressed `DEFAULT` encoding is written and read.
//!
//! **Potree 1.x** folders (`cloud.js` with one file per node) are read, see [`v1`].

mod reader;
mod v1;
mod writer;

use std::fs;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

pub(super) use writer::create;

use super::PointReader;
use crate::model::ScalarType;
use crate::{Error, Result};

pub(super) const METADATA_FILE: &str = "metadata.json";
const HIERARCHY_FILE: &str = "hierarchy.bin";
const OCTREE_FILE: &str = "octree.bin";
/// The description file of a Potree 1.x folder.
pub(super) const V1_FILE: &str = "cloud.js";
/// What output folders are named after: `scan.potree`.
pub(super) const FOLDER_EXTENSION: &str = "potree";

/// Bytes per node in `hierarchy.bin`.
const HIERARCHY_ENTRY_SIZE: usize = 22;
const NODE_NORMAL: u8 = 0;
const NODE_LEAF: u8 = 1;
const NODE_PROXY: u8 = 2;

/// Whether `path` is a Potree cloud: a folder holding one, or its `metadata.json` or
/// `cloud.js`. A `metadata.json` only counts next to an `octree.bin`, since the name is common.
pub(crate) fn is_cloud(path: &Path) -> bool {
    if path.is_dir() {
        return is_v2_folder(path) || path.join(V1_FILE).is_file();
    }
    let Some(name) = path.file_name().map(|name| name.to_string_lossy().to_ascii_lowercase()) else {
        return false;
    };
    let folder = path.parent().unwrap_or(Path::new(""));
    (name == METADATA_FILE && is_v2_folder(folder)) || (name == V1_FILE && path.is_file())
}

fn is_v2_folder(folder: &Path) -> bool {
    folder.join(METADATA_FILE).is_file() && folder.join(OCTREE_FILE).is_file()
}

/// The folder of the cloud at `path`, which is the folder or its description file.
fn folder_of(path: &Path) -> PathBuf {
    let is_description = path.file_name().is_some_and(|name| {
        let name = name.to_string_lossy();
        name.eq_ignore_ascii_case(METADATA_FILE) || name.eq_ignore_ascii_case(V1_FILE)
    });
    match path.parent() {
        Some(parent) if is_description && !path.is_dir() => parent.to_path_buf(),
        None if is_description => PathBuf::from("."),
        _ => path.to_path_buf(),
    }
}

/// The cloud's name: its folder's name without `.potree`.
pub(crate) fn cloud_name(path: &Path) -> String {
    let folder = folder_of(path);
    let name = match folder.file_name() {
        Some(name) => name.to_string_lossy().into_owned(),
        None => fs::canonicalize(&folder)
            .ok()
            .and_then(|folder| folder.file_name().map(|name| name.to_string_lossy().into_owned()))
            .unwrap_or_default(),
    };
    let suffix = format!(".{FOLDER_EXTENSION}");
    match name.len().checked_sub(suffix.len()) {
        Some(end) if end > 0 && name[end..].eq_ignore_ascii_case(&suffix) => name[..end].to_string(),
        _ => name,
    }
}

pub(super) fn open(path: &Path) -> Result<Box<dyn PointReader>> {
    let folder = folder_of(path);
    let named_v1 = path.file_name().is_some_and(|name| name.eq_ignore_ascii_case(V1_FILE));
    if !named_v1 && folder.join(METADATA_FILE).is_file() {
        reader::open(&folder)
    } else if folder.join(V1_FILE).is_file() {
        v1::open(&folder)
    } else {
        Err(Error::invalid(format!("{}: no {METADATA_FILE} or {V1_FILE} found", folder.display())))
    }
}

/// `metadata.json` of Potree 2.0. Fields are listed in the order they are written.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Metadata {
    version: String,
    name: String,
    #[serde(default)]
    description: String,
    points: u64,
    /// The coordinate system. Written as OGC WKT when the cloud has one; other writers may
    /// use other notations.
    #[serde(default)]
    projection: String,
    hierarchy: HierarchyInfo,
    offset: [f64; 3],
    scale: [f64; 3],
    spacing: f64,
    #[serde(rename = "boundingBox")]
    bounding_box: BoundingBox,
    encoding: String,
    attributes: Vec<AttributeInfo>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct HierarchyInfo {
    #[serde(rename = "firstChunkSize")]
    first_chunk_size: u64,
    #[serde(rename = "stepSize")]
    step_size: u32,
    depth: u32,
}

/// The root cube of the octree.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
struct BoundingBox {
    min: [f64; 3],
    max: [f64; 3],
}

/// One entry of the attribute list: `num_elements` values of `element_size` bytes each.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct AttributeInfo {
    name: String,
    #[serde(default)]
    description: String,
    size: usize,
    #[serde(rename = "numElements")]
    num_elements: usize,
    #[serde(rename = "elementSize")]
    element_size: usize,
    #[serde(rename = "type")]
    kind: String,
    /// Smallest and largest stored value of each element, which viewers use as the default
    /// range for colouring by the attribute.
    #[serde(default)]
    min: Vec<f64>,
    #[serde(default)]
    max: Vec<f64>,
    /// Stored numbers `n` stand for `n * scale + offset`; only written for extra values that
    /// have them.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    scale: Option<Scaling>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    offset: Option<Scaling>,
}

/// A scale or offset: one number, or one per element.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
enum Scaling {
    One(f64),
    PerElement(Vec<f64>),
}

impl Scaling {
    fn first(&self) -> Option<f64> {
        match self {
            Scaling::One(value) => Some(*value),
            Scaling::PerElement(values) => values.first().copied(),
        }
    }
}

/// What an attribute of the list means to EPCC.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Meaning {
    Position,
    Intensity,
    ReturnNumber,
    NumberOfReturns,
    /// The four LAS classification flags: synthetic, key point, withheld, overlap.
    ClassificationFlags,
    ScannerChannel,
    ScanDirection,
    EdgeOfFlightLine,
    Classification,
    UserData,
    /// In steps of 0.006 degrees, as LAS 1.4 stores it.
    ScanAngle,
    /// In whole degrees, as LAS 1.0 to 1.3 store it. Only read.
    ScanAngleRank,
    PointSourceId,
    GpsTime,
    Rgb,
    NormalX,
    NormalY,
    NormalZ,
    /// All three normal components in one attribute. Only read.
    Normal,
    /// Extra value number `n` of the cloud.
    Extra(usize),
}

/// The attribute names Potree viewers and converters use. The first name of a meaning is the
/// one written.
const NAMES: [(&str, Meaning); 21] = [
    ("position", Meaning::Position),
    ("POSITION_CARTESIAN", Meaning::Position),
    ("intensity", Meaning::Intensity),
    ("return number", Meaning::ReturnNumber),
    ("number of returns", Meaning::NumberOfReturns),
    ("classification flags", Meaning::ClassificationFlags),
    ("scanner channel", Meaning::ScannerChannel),
    ("scan direction flag", Meaning::ScanDirection),
    ("edge of flight line", Meaning::EdgeOfFlightLine),
    ("classification", Meaning::Classification),
    ("user data", Meaning::UserData),
    ("scan angle", Meaning::ScanAngle),
    ("scan angle rank", Meaning::ScanAngleRank),
    ("point source id", Meaning::PointSourceId),
    ("gps-time", Meaning::GpsTime),
    ("rgb", Meaning::Rgb),
    ("rgba", Meaning::Rgb),
    ("NormalX", Meaning::NormalX),
    ("NormalY", Meaning::NormalY),
    ("NormalZ", Meaning::NormalZ),
    ("normal", Meaning::Normal),
];

impl Meaning {
    fn of_name(name: &str) -> Option<Meaning> {
        NAMES.iter().find(|(known, _)| known.eq_ignore_ascii_case(name)).map(|&(_, meaning)| meaning)
    }

    fn name(self) -> &'static str {
        NAMES.iter().find(|(_, meaning)| *meaning == self).map_or("", |(name, _)| name)
    }
}

/// The type names of the attribute list.
const TYPE_NAMES: [(&str, ScalarType); 12] = [
    ("int8", ScalarType::I8),
    ("uint8", ScalarType::U8),
    ("int16", ScalarType::I16),
    ("uint16", ScalarType::U16),
    ("int32", ScalarType::I32),
    ("uint32", ScalarType::U32),
    ("int64", ScalarType::I64),
    ("uint64", ScalarType::U64),
    ("float", ScalarType::F32),
    ("double", ScalarType::F64),
    ("float32", ScalarType::F32),
    ("float64", ScalarType::F64),
];

fn type_name(scalar: ScalarType) -> &'static str {
    TYPE_NAMES.iter().find(|(_, known)| *known == scalar).map_or("", |(name, _)| name)
}

fn scalar_of(name: &str) -> Option<ScalarType> {
    TYPE_NAMES.iter().find(|(known, _)| known.eq_ignore_ascii_case(name)).map(|&(_, scalar)| scalar)
}

/// Writes `value` as a little-endian `scalar` at the start of `out`. Integers are rounded and
/// saturate at their type's limits.
fn put_scalar(scalar: ScalarType, value: f64, out: &mut [u8]) {
    macro_rules! put {
        ($value:expr) => {{
            let bytes = $value.to_le_bytes();
            out[..bytes.len()].copy_from_slice(&bytes);
        }};
    }
    let whole = value.round();
    match scalar {
        ScalarType::U8 => put!(whole as u8),
        ScalarType::I8 => put!(whole as i8),
        ScalarType::U16 => put!(whole as u16),
        ScalarType::I16 => put!(whole as i16),
        ScalarType::U32 => put!(whole as u32),
        ScalarType::I32 => put!(whole as i32),
        ScalarType::U64 => put!(whole as u64),
        ScalarType::I64 => put!(whole as i64),
        ScalarType::F32 => put!(value as f32),
        ScalarType::F64 => put!(value),
    }
}

/// One node of `hierarchy.bin`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct HierarchyEntry {
    kind: u8,
    child_mask: u8,
    points: u32,
    byte_offset: u64,
    byte_size: u64,
}

impl HierarchyEntry {
    fn write_to(&self, out: &mut Vec<u8>) {
        out.push(self.kind);
        out.push(self.child_mask);
        out.extend_from_slice(&self.points.to_le_bytes());
        out.extend_from_slice(&self.byte_offset.to_le_bytes());
        out.extend_from_slice(&self.byte_size.to_le_bytes());
    }

    fn read_from(bytes: &[u8]) -> HierarchyEntry {
        let u64_at = |at: usize| u64::from_le_bytes(bytes[at..at + 8].try_into().expect("8 bytes"));
        HierarchyEntry {
            kind: bytes[0],
            child_mask: bytes[1],
            points: u32::from_le_bytes(bytes[2..6].try_into().expect("4 bytes")),
            byte_offset: u64_at(6),
            byte_size: u64_at(14),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cloud_names_drop_the_folder_extension() {
        assert_eq!(cloud_name(Path::new("out/scan.potree")), "scan");
        assert_eq!(cloud_name(Path::new("out/Scan.POTREE")), "Scan");
        assert_eq!(cloud_name(Path::new("out/site")), "site");
        assert_eq!(cloud_name(Path::new("out/.potree")), ".potree");
        assert_eq!(cloud_name(Path::new("out/site.potree/metadata.json")), "site");
        assert_eq!(cloud_name(Path::new("out/old/cloud.js")), "old");
    }

    #[test]
    fn names_and_types_map_both_ways() {
        for (name, meaning) in NAMES {
            assert_eq!(Meaning::of_name(name), Some(meaning));
            assert_eq!(Meaning::of_name(&name.to_uppercase()), Some(meaning));
        }
        assert_eq!(Meaning::Rgb.name(), "rgb");
        assert_eq!(Meaning::of_name("amplitude"), None);
        for scalar in ScalarType::ALL {
            assert_eq!(scalar_of(type_name(scalar)), Some(scalar));
        }
    }

    #[test]
    fn hierarchy_entries_take_22_bytes() {
        let entry =
            HierarchyEntry { kind: 2, child_mask: 0b1010_0001, points: 70_000, byte_offset: 1 << 40, byte_size: 99 };
        let mut bytes = Vec::new();
        entry.write_to(&mut bytes);
        assert_eq!(bytes.len(), HIERARCHY_ENTRY_SIZE);
        assert_eq!(HierarchyEntry::read_from(&bytes), entry);
    }

    #[test]
    fn scalars_round_and_saturate() {
        let mut out = [0u8; 8];
        put_scalar(ScalarType::I16, -2.6, &mut out);
        assert_eq!(i16::from_le_bytes([out[0], out[1]]), -3);
        put_scalar(ScalarType::U8, 300.0, &mut out);
        assert_eq!(out[0], 255);
        put_scalar(ScalarType::F64, 0.1, &mut out);
        assert_eq!(f64::from_le_bytes(out), 0.1);
    }
}
