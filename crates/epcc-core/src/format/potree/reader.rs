//! Reading Potree 2.0: the hierarchy says where each node's points are in `octree.bin`; the
//! nodes are read in file order.

use std::fs::{self, File};
use std::io::{BufReader, Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};

use rustc_hash::FxHashSet;

use super::{
    AttributeInfo, HIERARCHY_ENTRY_SIZE, HIERARCHY_FILE, HierarchyEntry, METADATA_FILE, Meaning, Metadata, NODE_PROXY,
    OCTREE_FILE, cloud_name, scalar_of,
};
use crate::error::IoContext;
use crate::format::PointReader;
use crate::model::{
    Attributes, CHUNK_POINTS, CloudInfo, ExtraField, LidarDetails, PointChunk, PointRecord, ScalarType, ScanInfo,
    color_from_u8,
};
use crate::octree::{MAX_DEPTH, NodeKey};
use crate::{Error, Progress, Result};

/// The unit of stored scan angles, in degrees, as in LAS 1.4.
const SCAN_ANGLE_STEP: f32 = 0.006;

pub(super) fn open(folder: &Path) -> Result<Box<dyn PointReader>> {
    let metadata_path = folder.join(METADATA_FILE);
    let text = fs::read_to_string(&metadata_path).at(&metadata_path)?;
    let metadata: Metadata =
        serde_json::from_str(&text).map_err(|error| Error::invalid(format!("{}: {error}", metadata_path.display())))?;
    if !metadata.version.starts_with("2.") {
        return Err(Error::Unsupported(format!(
            "{}: Potree version {} is not supported",
            metadata_path.display(),
            metadata.version
        )));
    }
    if !metadata.encoding.eq_ignore_ascii_case("DEFAULT") {
        return Err(Error::Unsupported(format!(
            "{}: the {} encoding is not supported, only the uncompressed DEFAULT encoding",
            metadata_path.display(),
            metadata.encoding
        )));
    }
    let records = Records::new(&metadata)?;
    let hierarchy_path = folder.join(HIERARCHY_FILE);
    let hierarchy = fs::read(&hierarchy_path).at(&hierarchy_path)?;
    let nodes = read_hierarchy(&hierarchy, metadata.hierarchy.first_chunk_size)
        .map_err(|error| Error::invalid(format!("{}: {error}", hierarchy_path.display())))?;

    let name = if metadata.name.trim().is_empty() { cloud_name(folder) } else { metadata.name.clone() };
    let info = CloudInfo {
        attributes: records.attributes,
        scans: vec![ScanInfo::named(name)],
        point_count: Some(metadata.points),
        // Other notations, such as proj4 strings, are not WKT and are left out.
        crs_wkt: Some(metadata.projection.trim().to_string()).filter(|projection| projection.contains('[')),
        extra_fields: records.extra_fields.clone(),
    };
    Ok(Box::new(PotreeReader { octree_path: folder.join(OCTREE_FILE), info, records, nodes }))
}

/// A node with points, as the hierarchy describes it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct NodeData {
    pub key: NodeKey,
    pub points: u64,
    pub byte_offset: u64,
    pub byte_size: u64,
}

/// Reads every node of `hierarchy.bin`, following proxies into the chunks they point to.
pub(super) fn read_hierarchy(bytes: &[u8], first_chunk_size: u64) -> std::result::Result<Vec<NodeData>, String> {
    let mut nodes = Vec::new();
    let mut chunks = vec![(NodeKey::ROOT, 0, first_chunk_size)];
    let mut seen = FxHashSet::default();
    while let Some((root, offset, size)) = chunks.pop() {
        if !seen.insert(offset) {
            return Err("a hierarchy chunk is listed twice".into());
        }
        let chunk = usize::try_from(offset)
            .ok()
            .zip(usize::try_from(size).ok())
            .and_then(|(offset, size)| bytes.get(offset..offset.checked_add(size)?))
            .ok_or("the hierarchy points past the end of the file")?;
        if chunk.is_empty() || chunk.len() % HIERARCHY_ENTRY_SIZE != 0 {
            return Err(format!("a hierarchy chunk of {} bytes is not a whole number of nodes", chunk.len()));
        }
        // The nodes of a chunk are listed breadth first, so each node's key is known from the
        // child masks of the nodes before it.
        let mut keys = vec![root];
        let entries = chunk.as_chunks::<HIERARCHY_ENTRY_SIZE>().0.iter().map(|entry| HierarchyEntry::read_from(entry));
        for (index, entry) in entries.enumerate() {
            let key = *keys.get(index).ok_or("a hierarchy chunk lists more nodes than the child masks say")?;
            if entry.kind == NODE_PROXY {
                if index == 0 {
                    return Err("a hierarchy chunk starts with a proxy".into());
                }
                chunks.push((key, entry.byte_offset, entry.byte_size));
                continue;
            }
            if entry.child_mask != 0 && key.level >= MAX_DEPTH {
                return Err(format!("the octree is deeper than {MAX_DEPTH} levels"));
            }
            // A node without bytes has no points, whatever its count says.
            let points = if entry.byte_size == 0 { 0 } else { u64::from(entry.points) };
            nodes.push(NodeData { key, points, byte_offset: entry.byte_offset, byte_size: entry.byte_size });
            keys.extend((0..8).filter(|octant| entry.child_mask & (1 << octant) != 0).map(|octant| key.child(octant)));
        }
        if keys.len() != chunk.len() / HIERARCHY_ENTRY_SIZE {
            return Err("a hierarchy chunk lists fewer nodes than the child masks say".into());
        }
    }
    Ok(nodes)
}

/// How the attributes lie in a point record.
struct Records {
    fields: Vec<Field>,
    size: usize,
    scale: [f64; 3],
    offset: [f64; 3],
    attributes: Attributes,
    extra_fields: Vec<ExtraField>,
    /// Whether the colours only use 0 to 255, as many writers store them.
    eight_bit_color: bool,
}

#[derive(Clone, Copy, Debug)]
struct Field {
    meaning: Meaning,
    scalar: ScalarType,
    /// Where the field starts in a record.
    at: usize,
    size: usize,
    /// For extra values: where they go among a point's extra bytes.
    extra_at: usize,
}

impl Records {
    fn new(metadata: &Metadata) -> Result<Records> {
        let mut fields = Vec::new();
        let mut extra_fields = Vec::new();
        let mut at = 0;
        for attribute in &metadata.attributes {
            let scalar = scalar_of(&attribute.kind).ok_or_else(|| {
                Error::Unsupported(format!(
                    "Potree attribute \"{}\" has the unknown type {}",
                    attribute.name, attribute.kind
                ))
            })?;
            if attribute.size != attribute.num_elements * scalar.size() || attribute.element_size != scalar.size() {
                return Err(Error::invalid(format!(
                    "Potree attribute \"{}\" has an inconsistent size",
                    attribute.name
                )));
            }
            let meaning = Meaning::of_name(&attribute.name).filter(|&meaning| fits(meaning, attribute, scalar));
            fields.push(Field {
                meaning: meaning.unwrap_or(Meaning::Extra(0)),
                scalar,
                at,
                size: attribute.size,
                extra_at: 0,
            });
            at += attribute.size;
        }
        // Normal components only count together; a lone one is an extra value.
        let has = |meaning: Meaning| fields.iter().any(|field| field.meaning == meaning);
        let normal_parts = has(Meaning::NormalX) && has(Meaning::NormalY) && has(Meaning::NormalZ);
        let mut extra_at = 0;
        for (field, attribute) in fields.iter_mut().zip(&metadata.attributes) {
            let lone_part =
                matches!(field.meaning, Meaning::NormalX | Meaning::NormalY | Meaning::NormalZ) && !normal_parts;
            if matches!(field.meaning, Meaning::Extra(_)) || lone_part {
                field.meaning = Meaning::Extra(extra_fields.len());
                field.extra_at = extra_at;
                extra_at += field.size;
                extra_fields.push(extra_field(attribute, field.scalar));
            }
        }
        let has = |meaning: Meaning| fields.iter().any(|field| field.meaning == meaning);
        if !has(Meaning::Position) {
            return Err(Error::invalid("Potree metadata lists no position attribute"));
        }
        let attributes = Attributes {
            intensity: has(Meaning::Intensity),
            color: has(Meaning::Rgb),
            normal: has(Meaning::Normal) || normal_parts,
            classification: has(Meaning::Classification),
            time: has(Meaning::GpsTime),
            returns: has(Meaning::ReturnNumber) || has(Meaning::NumberOfReturns),
            lidar: [
                Meaning::ClassificationFlags,
                Meaning::ScannerChannel,
                Meaning::ScanDirection,
                Meaning::EdgeOfFlightLine,
                Meaning::UserData,
                Meaning::ScanAngle,
                Meaning::ScanAngleRank,
                Meaning::PointSourceId,
            ]
            .into_iter()
            .any(has),
            extra_bytes: u16::try_from(extra_at)
                .map_err(|_| Error::Unsupported("Potree points with more than 65535 bytes of extra values".into()))?,
        };
        let eight_bit_color = metadata.attributes.iter().any(|attribute| {
            Meaning::of_name(&attribute.name) == Some(Meaning::Rgb)
                && !attribute.max.is_empty()
                && attribute.max.iter().all(|&max| max <= 255.0)
        });
        Ok(Records {
            fields,
            size: at,
            scale: metadata.scale,
            offset: metadata.offset,
            attributes,
            extra_fields,
            eight_bit_color,
        })
    }

    /// Appends the points of one node's records to `chunk`.
    fn decode(&self, records: &[u8], chunk: &mut PointChunk) {
        for record in records.chunks_exact(self.size) {
            let mut point = PointRecord::default();
            for field in &self.fields {
                let width = field.scalar.size();
                let value = |element: usize| field.scalar.decode(&record[field.at + element * width..]);
                let lidar = &mut point.lidar;
                match field.meaning {
                    Meaning::Position => {
                        point.position = std::array::from_fn(|axis| value(axis) * self.scale[axis] + self.offset[axis]);
                    }
                    Meaning::Intensity => {
                        // Computed like the LAS reader does, so equal values give equal points.
                        let full: f32 = match field.scalar {
                            ScalarType::U8 => 255.0,
                            ScalarType::F32 | ScalarType::F64 => 1.0,
                            _ => 65535.0,
                        };
                        point.intensity = (value(0) as f32 / full).clamp(0.0, 1.0);
                    }
                    Meaning::ReturnNumber => point.returns.number = value(0) as u8,
                    Meaning::NumberOfReturns => point.returns.count = value(0) as u8,
                    Meaning::ClassificationFlags => lidar.flags |= value(0) as u8 & LidarDetails::CLASS_FLAGS,
                    Meaning::ScannerChannel => {
                        lidar.flags |= (value(0) as u8) << LidarDetails::CHANNEL_SHIFT & LidarDetails::CHANNEL;
                    }
                    Meaning::ScanDirection if value(0) != 0.0 => lidar.flags |= LidarDetails::SCAN_DIRECTION,
                    Meaning::EdgeOfFlightLine if value(0) != 0.0 => lidar.flags |= LidarDetails::EDGE_OF_FLIGHT_LINE,
                    Meaning::ScanDirection | Meaning::EdgeOfFlightLine => {}
                    Meaning::Classification => point.classification = value(0) as u8,
                    Meaning::UserData => lidar.user_data = value(0) as u8,
                    Meaning::ScanAngle => lidar.scan_angle = value(0) as f32 * SCAN_ANGLE_STEP,
                    Meaning::ScanAngleRank => lidar.scan_angle = value(0) as f32,
                    Meaning::PointSourceId => lidar.point_source = value(0) as u16,
                    Meaning::GpsTime => point.time = value(0),
                    Meaning::Rgb => {
                        point.color = std::array::from_fn(|channel| {
                            let stored = value(channel) as u16;
                            if self.eight_bit_color { color_from_u8(stored.min(255) as u8) } else { stored }
                        });
                    }
                    Meaning::NormalX => point.normal[0] = value(0) as f32,
                    Meaning::NormalY => point.normal[1] = value(0) as f32,
                    Meaning::NormalZ => point.normal[2] = value(0) as f32,
                    Meaning::Normal => point.normal = std::array::from_fn(|axis| value(axis) as f32),
                    Meaning::Extra(_) => {}
                }
            }
            chunk.push(&point);
            if let Some(extra) = &mut chunk.extra {
                let index = extra.bytes.len() / extra.width - 1;
                let target = extra.get_mut(index);
                for field in self.fields.iter().filter(|field| matches!(field.meaning, Meaning::Extra(_))) {
                    target[field.extra_at..field.extra_at + field.size]
                        .copy_from_slice(&record[field.at..field.at + field.size]);
                }
            }
        }
    }
}

/// Whether an attribute has the shape its name suggests. One that does not is read as an extra
/// value.
fn fits(meaning: Meaning, attribute: &AttributeInfo, scalar: ScalarType) -> bool {
    let float = matches!(scalar, ScalarType::F32 | ScalarType::F64);
    match meaning {
        Meaning::Position => attribute.num_elements == 3 && !float,
        Meaning::Rgb => attribute.num_elements >= 3,
        Meaning::Normal => attribute.num_elements == 3 && float,
        Meaning::NormalX | Meaning::NormalY | Meaning::NormalZ => attribute.num_elements == 1 && float,
        _ => attribute.num_elements == 1,
    }
}

/// An attribute EPCC has no column for, kept as an extra value.
fn extra_field(attribute: &AttributeInfo, scalar: ScalarType) -> ExtraField {
    let single = attribute.num_elements == 1;
    let first = |scaling: &Option<super::Scaling>| scaling.as_ref().and_then(super::Scaling::first);
    ExtraField {
        name: attribute.name.clone(),
        description: attribute.description.clone(),
        size: attribute.size as u16,
        scalar: single.then_some(scalar),
        scale: first(&attribute.scale).filter(|_| single).unwrap_or(1.0),
        offset: first(&attribute.offset).filter(|_| single).unwrap_or(0.0),
        las_descriptor: None,
    }
}

struct PotreeReader {
    octree_path: PathBuf,
    info: CloudInfo,
    records: Records,
    nodes: Vec<NodeData>,
}

impl PointReader for PotreeReader {
    fn info(&self) -> &CloudInfo {
        &self.info
    }

    fn read(self: Box<Self>, emit: &mut dyn FnMut(PointChunk) -> Result<()>, progress: &Progress) -> Result<()> {
        let path = &self.octree_path;
        let file = File::open(path).at(path)?;
        let file_size = file.metadata().at(path)?.len();
        let mut file = BufReader::with_capacity(1 << 20, file);
        let mut nodes: Vec<NodeData> = self.nodes.into_iter().filter(|node| node.points > 0).collect();
        nodes.sort_by_key(|node| node.byte_offset);
        let total: u64 = nodes.iter().map(|node| node.points).sum();
        let attributes = self.info.attributes;
        let new_chunk = || PointChunk::with_capacity(0, attributes, CHUNK_POINTS);
        let mut chunk = new_chunk();
        let mut bytes = Vec::new();
        let mut done = 0;
        for node in nodes {
            let expected = node.points.checked_mul(self.records.size as u64);
            let fits_file = node.byte_offset.checked_add(node.byte_size).is_some_and(|end| end <= file_size);
            if expected != Some(node.byte_size) || !fits_file {
                return Err(Error::invalid(format!(
                    "{}: node {} does not hold {} points of {} bytes",
                    path.display(),
                    node_name(node.key),
                    node.points,
                    self.records.size
                )));
            }
            bytes.resize(node.byte_size as usize, 0);
            file.seek(SeekFrom::Start(node.byte_offset)).at(path)?;
            file.read_exact(&mut bytes).at(path)?;
            self.records.decode(&bytes, &mut chunk);
            if chunk.len() >= CHUNK_POINTS {
                emit(std::mem::replace(&mut chunk, new_chunk()))?;
            }
            done += node.points;
            progress.set_done_of(done, total);
        }
        if !chunk.is_empty() {
            emit(chunk)?;
        }
        Ok(())
    }
}

/// The name Potree gives a node: `r` and the octant of each level below the root.
pub(super) fn node_name(key: NodeKey) -> String {
    let mut name = String::from("r");
    for level in 1..=key.level {
        name.push(char::from(b'0' + key.ancestor_at(level).octant()));
    }
    name
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn node_names_spell_the_path_from_the_root() {
        assert_eq!(node_name(NodeKey::ROOT), "r");
        let key = NodeKey::ROOT.child(5).child(0).child(7);
        assert_eq!(node_name(key), "r507");
    }

    #[test]
    fn broken_hierarchies_are_reported() {
        let entry = |kind, child_mask, byte_offset, byte_size| {
            let mut bytes = Vec::new();
            HierarchyEntry { kind, child_mask, points: 1, byte_offset, byte_size }.write_to(&mut bytes);
            bytes
        };
        // The root says it has two children, but only one follows.
        let mut bytes = entry(0, 0b11, 0, 10);
        bytes.extend(entry(1, 0, 10, 10));
        assert!(read_hierarchy(&bytes, 44).unwrap_err().contains("fewer nodes"));
        // A proxy pointing back at the root chunk.
        let mut bytes = entry(0, 0b1, 0, 10);
        bytes.extend(entry(NODE_PROXY, 0, 0, 44));
        assert!(read_hierarchy(&bytes, 44).unwrap_err().contains("twice"));
        assert!(read_hierarchy(&bytes, 66).unwrap_err().contains("past the end"));
    }
}
