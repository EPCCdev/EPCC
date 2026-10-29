//! Reading Potree 1.x: `cloud.js` describes the cloud, and every node is a file of its own,
//! named after its path from the root (`r`, `r0`, `r05`, ...) in folders below `octreeDir`.
//!
//! Node files are either LAS/LAZ files, read as such, or binary records (versions 1.7 and
//! later): positions as unsigned integers relative to the node's cube, then the attributes
//! `cloud.js` lists. Since every point is in exactly one node, reading every node file gives
//! the whole cloud; the `.hrc` hierarchy files are not needed.

use std::fs;
use std::path::{Path, PathBuf};

use serde::Deserialize;
use serde_json::Value;

use super::{V1_FILE, cloud_name};
use crate::error::IoContext;
use crate::format::{PointReader, las};
use crate::model::{Attributes, CHUNK_POINTS, CloudInfo, PointChunk, PointRecord, ScanInfo, color_from_u8};
use crate::{Error, Progress, Result};

#[derive(Debug, Deserialize)]
struct CloudJs {
    version: String,
    #[serde(rename = "octreeDir", default = "default_octree_dir")]
    octree_dir: String,
    #[serde(default)]
    projection: String,
    #[serde(default)]
    points: Option<u64>,
    #[serde(rename = "boundingBox")]
    bounding_box: Bounds,
    #[serde(rename = "pointAttributes")]
    point_attributes: Value,
    #[serde(default = "default_scale")]
    scale: f64,
}

fn default_octree_dir() -> String {
    "data".into()
}

fn default_scale() -> f64 {
    0.01
}

#[derive(Clone, Copy, Debug, Deserialize)]
struct Bounds {
    lx: f64,
    ly: f64,
    lz: f64,
    ux: f64,
    uy: f64,
    uz: f64,
}

/// The attributes of binary node records.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Attribute {
    Position,
    Rgba,
    Intensity,
    Classification,
    ReturnNumber,
    NumberOfReturns,
    SourceId,
    GpsTime,
    NormalSpheremapped,
    NormalOct16,
    Normal,
    /// Values for the viewer only (point indices, spacing), skipped.
    Skipped(usize),
}

impl Attribute {
    fn of_name(name: &str) -> Option<Attribute> {
        let attribute = match name {
            "POSITION_CARTESIAN" => Attribute::Position,
            "COLOR_PACKED" | "RGBA_PACKED" => Attribute::Rgba,
            "INTENSITY" => Attribute::Intensity,
            "CLASSIFICATION" => Attribute::Classification,
            "RETURN_NUMBER" => Attribute::ReturnNumber,
            "NUMBER_OF_RETURNS" => Attribute::NumberOfReturns,
            "SOURCE_ID" => Attribute::SourceId,
            "GPS_TIME" => Attribute::GpsTime,
            "NORMAL_SPHEREMAPPED" => Attribute::NormalSpheremapped,
            "NORMAL_OCT16" => Attribute::NormalOct16,
            "NORMAL" | "NORMAL_FLOATS" => Attribute::Normal,
            "INDICES" | "SPACING" => Attribute::Skipped(4),
            _ => return None,
        };
        Some(attribute)
    }

    fn size(self) -> usize {
        match self {
            Attribute::Position | Attribute::Normal => 12,
            Attribute::Rgba => 4,
            Attribute::Intensity | Attribute::SourceId | Attribute::NormalSpheremapped | Attribute::NormalOct16 => 2,
            Attribute::Classification | Attribute::ReturnNumber | Attribute::NumberOfReturns => 1,
            Attribute::GpsTime => 8,
            Attribute::Skipped(size) => size,
        }
    }
}

/// How node files store their points.
enum NodeFormat {
    /// LAS or LAZ files.
    Las,
    Binary(Vec<Attribute>),
}

pub(super) fn open(folder: &Path) -> Result<Box<dyn PointReader>> {
    let path = folder.join(V1_FILE);
    let text = fs::read_to_string(&path).at(&path)?;
    let cloud: CloudJs =
        serde_json::from_str(&text).map_err(|error| Error::invalid(format!("{}: {error}", path.display())))?;
    let nodes = node_files(&folder.join(&cloud.octree_dir))?;
    let format = match &cloud.point_attributes {
        Value::String(kind) if kind.eq_ignore_ascii_case("LAS") || kind.eq_ignore_ascii_case("LAZ") => NodeFormat::Las,
        Value::Array(list) => NodeFormat::Binary(binary_attributes(list, &path)?),
        other => return Err(Error::invalid(format!("{}: unknown point attributes {other}", path.display()))),
    };
    let version: f64 = cloud.version.trim().parse().unwrap_or(0.0);
    if matches!(format, NodeFormat::Binary(_)) && version < 1.7 {
        return Err(Error::Unsupported(format!(
            "{}: binary Potree files before version 1.7 are not supported",
            path.display()
        )));
    }

    let mut info = match (&format, nodes.first()) {
        (NodeFormat::Las, Some(first)) => las::open(&first.path)?.info().clone(),
        (NodeFormat::Las, None) => CloudInfo::default(),
        (NodeFormat::Binary(attributes), _) => {
            CloudInfo { attributes: binary_cloud(attributes), ..CloudInfo::default() }
        }
    };
    info.scans = vec![ScanInfo::named(cloud_name(folder))];
    info.point_count = cloud.points;
    let projection = cloud.projection.trim();
    if projection.contains('[') {
        info.crs_wkt = Some(projection.to_string());
    }
    Ok(Box::new(V1Reader { info, cloud, format, nodes }))
}

fn binary_attributes(list: &[Value], path: &Path) -> Result<Vec<Attribute>> {
    let attributes = list
        .iter()
        .map(|entry| {
            let name = match entry {
                Value::String(name) => Some(name.as_str()),
                Value::Object(fields) => fields.get("name").and_then(Value::as_str),
                _ => None,
            };
            name.and_then(Attribute::of_name).ok_or_else(|| {
                Error::Unsupported(format!("{}: the point attribute {entry} is not supported", path.display()))
            })
        })
        .collect::<Result<Vec<_>>>()?;
    if attributes.first() != Some(&Attribute::Position) {
        return Err(Error::invalid(format!("{}: the points have no position first", path.display())));
    }
    Ok(attributes)
}

fn binary_cloud(attributes: &[Attribute]) -> Attributes {
    let has = |wanted: &[Attribute]| attributes.iter().any(|attribute| wanted.contains(attribute));
    Attributes {
        intensity: has(&[Attribute::Intensity]),
        color: has(&[Attribute::Rgba]),
        normal: has(&[Attribute::Normal, Attribute::NormalSpheremapped, Attribute::NormalOct16]),
        classification: has(&[Attribute::Classification]),
        time: has(&[Attribute::GpsTime]),
        returns: has(&[Attribute::ReturnNumber, Attribute::NumberOfReturns]),
        lidar: has(&[Attribute::SourceId]),
        extra_bytes: 0,
    }
}

/// A node file and the octant of each level on the way to its node.
struct NodeFile {
    path: PathBuf,
    octants: Vec<u8>,
}

/// Every node file below `folder`, root first, then level by level.
fn node_files(folder: &Path) -> Result<Vec<NodeFile>> {
    let mut files = Vec::new();
    let mut folders = vec![folder.to_path_buf()];
    while let Some(current) = folders.pop() {
        for entry in fs::read_dir(&current).at(&current)? {
            let path = entry.at(&current)?.path();
            if path.is_dir() {
                folders.push(path);
                continue;
            }
            let extension = path.extension().map(|extension| extension.to_string_lossy().to_ascii_lowercase());
            if !matches!(extension.as_deref(), Some("bin" | "las" | "laz")) {
                continue;
            }
            let stem = path.file_stem().map(|stem| stem.to_string_lossy().into_owned()).unwrap_or_default();
            let Some(digits) = stem.strip_prefix('r') else { continue };
            if digits.bytes().all(|digit| (b'0'..=b'7').contains(&digit)) {
                files.push(NodeFile { octants: digits.bytes().map(|digit| digit - b'0').collect(), path });
            }
        }
    }
    files.sort_by(|a, b| (a.octants.len(), &a.octants).cmp(&(b.octants.len(), &b.octants)));
    Ok(files)
}

struct V1Reader {
    info: CloudInfo,
    cloud: CloudJs,
    format: NodeFormat,
    nodes: Vec<NodeFile>,
}

impl PointReader for V1Reader {
    fn info(&self) -> &CloudInfo {
        &self.info
    }

    fn read(self: Box<Self>, emit: &mut dyn FnMut(PointChunk) -> Result<()>, progress: &Progress) -> Result<()> {
        let node_count = self.nodes.len() as u64;
        let attributes = self.info.attributes;
        let new_chunk = || PointChunk::with_capacity(0, attributes, CHUNK_POINTS);
        let mut chunk = new_chunk();
        for (done, node) in self.nodes.iter().enumerate() {
            match &self.format {
                NodeFormat::Las => {
                    let reader = las::open(&node.path)?;
                    reader.read(
                        &mut |mut part| {
                            part.scan = 0;
                            part.conform_to(attributes);
                            emit(part)
                        },
                        &Progress::new(),
                    )?;
                }
                NodeFormat::Binary(fields) => {
                    let bytes = fs::read(&node.path).at(&node.path)?;
                    decode_binary(&bytes, fields, &self.node_min(&node.octants), self.cloud.scale, &mut chunk);
                    if chunk.len() >= CHUNK_POINTS {
                        emit(std::mem::replace(&mut chunk, new_chunk()))?;
                    }
                }
            }
            progress.check_cancelled()?;
            progress.set_done_of(done as u64 + 1, node_count);
        }
        if !chunk.is_empty() {
            emit(chunk)?;
        }
        Ok(())
    }
}

impl V1Reader {
    /// The low corner of a node's box, found by halving the cloud's box once per level.
    fn node_min(&self, octants: &[u8]) -> [f64; 3] {
        let bounds = &self.cloud.bounding_box;
        let mut min = [bounds.lx, bounds.ly, bounds.lz];
        let mut size = [bounds.ux - bounds.lx, bounds.uy - bounds.ly, bounds.uz - bounds.lz];
        for &octant in octants {
            for (axis, bit) in [4, 2, 1].into_iter().enumerate() {
                size[axis] /= 2.0;
                if octant & bit != 0 {
                    min[axis] += size[axis];
                }
            }
        }
        min
    }
}

/// Appends the points of a binary node file to `chunk`. A partial record at the end is ignored.
fn decode_binary(bytes: &[u8], fields: &[Attribute], node_min: &[f64; 3], scale: f64, chunk: &mut PointChunk) {
    let size: usize = fields.iter().map(|field| field.size()).sum();
    for record in bytes.chunks_exact(size) {
        let mut point = PointRecord::default();
        let mut at = 0;
        for &field in fields {
            let bytes = &record[at..at + field.size()];
            let u16_at = |at: usize| u16::from_le_bytes([bytes[at], bytes[at + 1]]);
            let u32_at = |at: usize| u32::from_le_bytes(bytes[at..at + 4].try_into().expect("4 bytes"));
            match field {
                Attribute::Position => {
                    point.position = std::array::from_fn(|axis| node_min[axis] + f64::from(u32_at(axis * 4)) * scale);
                }
                Attribute::Rgba => point.color = [bytes[0], bytes[1], bytes[2]].map(color_from_u8),
                Attribute::Intensity => point.intensity = f32::from(u16_at(0)) / 65535.0,
                Attribute::Classification => point.classification = bytes[0],
                Attribute::ReturnNumber => point.returns.number = bytes[0],
                Attribute::NumberOfReturns => point.returns.count = bytes[0],
                Attribute::SourceId => point.lidar.point_source = u16_at(0),
                Attribute::GpsTime => point.time = f64::from_le_bytes(bytes.try_into().expect("8 bytes")),
                Attribute::NormalSpheremapped => point.normal = spheremap_normal(bytes[0], bytes[1]),
                Attribute::NormalOct16 => point.normal = octahedral_normal(bytes[0], bytes[1]),
                Attribute::Normal => point.normal = std::array::from_fn(|axis| f32::from_bits(u32_at(axis * 4))),
                Attribute::Skipped(_) => {}
            }
            at += field.size();
        }
        chunk.push(&point);
    }
}

/// Maps a byte onto -1..1.
fn signed_unit(byte: u8) -> f32 {
    f32::from(byte) / 255.0 * 2.0 - 1.0
}

/// Decodes a normal stored with the sphere map transform: `p`, the first two components scaled
/// by `1 / sqrt(2 (1 + z))`, gives the normal `(2 p sqrt(1 - |p|²), 1 - 2 |p|²)`.
fn spheremap_normal(u: u8, v: u8) -> [f32; 3] {
    let p = [signed_unit(u), signed_unit(v)];
    let rest = (1.0 - (p[0] * p[0] + p[1] * p[1])).max(0.0);
    let factor = 2.0 * rest.sqrt();
    [p[0] * factor, p[1] * factor, 2.0 * rest - 1.0]
}

/// Decodes a normal stored as a point on an octahedron, unfolded onto a square.
fn octahedral_normal(u: u8, v: u8) -> [f32; 3] {
    let (mut x, mut y) = (signed_unit(u), signed_unit(v));
    let z = 1.0 - x.abs() - y.abs();
    if z < 0.0 {
        (x, y) = ((1.0 - y.abs()) * x.signum(), (1.0 - x.abs()) * y.signum());
    }
    let length = (x * x + y * y + z * z).sqrt();
    if length > 0.0 { [x / length, y / length, z / length] } else { [0.0, 0.0, 1.0] }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_unit(normal: [f32; 3], want: [f32; 3]) {
        for (got, want) in normal.iter().zip(want) {
            assert!((got - want).abs() < 0.02, "{normal:?} != {want:?}");
        }
    }

    #[test]
    fn packed_normals_decode_to_unit_vectors() {
        // The centre of both encodings is straight up.
        assert_unit(spheremap_normal(128, 128), [0.0, 0.0, 1.0]);
        assert_unit(octahedral_normal(128, 128), [0.0, 0.0, 1.0]);
        // The octahedron's corners on the square's edges are the horizontal axes.
        assert_unit(octahedral_normal(255, 128), [1.0, 0.0, 0.0]);
        assert_unit(octahedral_normal(128, 0), [0.0, -1.0, 0.0]);
        // Its square's corners fold onto straight down.
        assert_unit(octahedral_normal(255, 255), [0.0, 0.0, -1.0]);
        // The sphere map's edge circle is straight down, half way out is horizontal.
        assert_unit(spheremap_normal(255, 128), [0.0, 0.0, -1.0]);
        let half = (0.5f32.sqrt() + 1.0) / 2.0 * 255.0;
        assert_unit(spheremap_normal(half.round() as u8, 128), [1.0, 0.0, 0.0]);
    }

    #[test]
    fn nodes_are_found_root_first() {
        let folder = std::env::temp_dir().join(format!("epcc-potree-v1-{}", std::process::id()));
        let _ = fs::remove_dir_all(&folder);
        fs::create_dir_all(folder.join("r/012")).unwrap();
        for name in ["r/r.bin", "r/r4.bin", "r/r01.bin", "r/r.hrc", "r/012/r0123.bin", "r/012/r0123.hrc", "r/notes.bin"]
        {
            fs::write(folder.join(name), b"").unwrap();
        }
        let nodes = node_files(&folder).unwrap();
        let octants: Vec<Vec<u8>> = nodes.iter().map(|node| node.octants.clone()).collect();
        assert_eq!(octants, [vec![], vec![4], vec![0, 1], vec![0, 1, 2, 3]]);
        fs::remove_dir_all(&folder).unwrap();
    }
}
