//! Writing Potree 2.0. The octree is built by [`crate::octree`]; this module lays out the nodes'
//! points, the hierarchy and the metadata.

use std::collections::VecDeque;
use std::fs::{self, File};
use std::io::{BufWriter, ErrorKind, Write};
use std::path::{Path, PathBuf};
use std::sync::Mutex;

use rustc_hash::FxHashMap;

use super::{
    AttributeInfo, BoundingBox, HIERARCHY_ENTRY_SIZE, HIERARCHY_FILE, HierarchyEntry, HierarchyInfo, METADATA_FILE,
    Meaning, Metadata, NODE_LEAF, NODE_NORMAL, NODE_PROXY, OCTREE_FILE, Scaling, put_scalar, type_name,
};
use crate::error::IoContext;
use crate::format::{PointWriter, WriteRequest};
use crate::model::{CloudInfo, ExtraField, LidarDetails, PointRecord, ScalarType};
use crate::octree::{self, Layout, NodeEntry, NodeKey, NodeSink, OctreeParams};
use crate::{Error, Format, PointChunk, Progress, Result};

/// The octree shape. Inner nodes keep one point per cell of a 128³ grid, which gives a sample
/// spaced evenly like a Poisson-disk sample, at a fraction of its cost, and depends only on the
/// input. Leaves hold at most 20 000 points, so a viewer loads detail in small steps.
const PARAMS: OctreeParams = OctreeParams {
    grid_cells: 128,
    max_leaf_points: 20_000,
    max_subtree_points: 5_000_000,
    max_depth: octree::MAX_DEPTH,
};

/// Levels per chunk of `hierarchy.bin`, as other Potree writers use.
const STEP_SIZE: u8 = 4;
/// The unit of stored scan angles, in degrees, as in LAS 1.4.
const SCAN_ANGLE_STEP: f64 = 0.006;
const SCAN_ANGLE_LIMIT: f64 = 30_000.0;

pub(in crate::format) fn create(request: &WriteRequest) -> Result<Box<dyn PointWriter>> {
    let folder = request.path.to_path_buf();
    // A folder of this temporary name can only be left over from an interrupted run.
    match fs::remove_dir_all(&folder) {
        Err(error) if error.kind() != ErrorKind::NotFound => return Err(Error::Io { path: folder, source: error }),
        _ => {}
    }
    fs::create_dir_all(&folder).at(&folder)?;
    let work_folder = folder.parent().map_or_else(|| PathBuf::from("."), Path::to_path_buf);
    Ok(Box::new(PotreeWriter {
        folder,
        work_folder,
        cloud: request.cloud.clone(),
        precision: request.settings.las.precision,
        name: Format::stem_of(request.source),
    }))
}

struct PotreeWriter {
    folder: PathBuf,
    work_folder: PathBuf,
    cloud: CloudInfo,
    precision: f64,
    name: String,
}

impl PointWriter for PotreeWriter {
    fn write(self: Box<Self>, chunks: &mut dyn Iterator<Item = PointChunk>, progress: &Progress) -> Result<()> {
        octree::build(chunks, self.cloud.attributes, &PARAMS, &self.work_folder, progress, |layout| {
            PotreeFolder::create(&self.folder, layout, self.precision, &self.cloud, &self.name)
        })
    }
}

/// One attribute of the point records.
#[derive(Clone, Debug, PartialEq)]
struct Field {
    meaning: Meaning,
    name: String,
    description: String,
    scalar: ScalarType,
    elements: usize,
    /// For extra values: where the field starts among a point's extra bytes.
    extra_at: usize,
    /// For extra values: scale and offset of the stored numbers.
    scaling: Option<(f64, f64)>,
}

impl Field {
    fn standard(meaning: Meaning, scalar: ScalarType, elements: usize) -> Field {
        Field {
            meaning,
            name: meaning.name().to_string(),
            description: String::new(),
            scalar,
            elements,
            extra_at: 0,
            scaling: None,
        }
    }

    fn size(&self) -> usize {
        self.scalar.size() * self.elements
    }
}

/// The attributes of the records, in the order they are stored.
fn fields_for(cloud: &CloudInfo) -> Vec<Field> {
    use Meaning::*;
    use ScalarType::*;
    let attributes = cloud.attributes;
    let mut fields = vec![Field::standard(Position, I32, 3)];
    let mut add = |present: bool, list: &[(Meaning, ScalarType, usize)]| {
        if present {
            fields.extend(list.iter().map(|&(meaning, scalar, elements)| Field::standard(meaning, scalar, elements)));
        }
    };
    add(attributes.intensity, &[(Intensity, U16, 1)]);
    add(attributes.returns, &[(ReturnNumber, U8, 1), (NumberOfReturns, U8, 1)]);
    add(
        attributes.lidar,
        &[(ClassificationFlags, U8, 1), (ScannerChannel, U8, 1), (ScanDirection, U8, 1), (EdgeOfFlightLine, U8, 1)],
    );
    add(attributes.classification, &[(Classification, U8, 1)]);
    add(attributes.lidar, &[(UserData, U8, 1), (ScanAngle, I16, 1), (PointSourceId, U16, 1)]);
    add(attributes.time, &[(GpsTime, F64, 1)]);
    add(attributes.color, &[(Rgb, U16, 3)]);
    add(attributes.normal, &[(NormalX, F32, 1), (NormalY, F32, 1), (NormalZ, F32, 1)]);
    if attributes.has_extra() {
        fields.extend(extra_fields(&cloud.extra_fields, usize::from(attributes.extra_bytes), &fields));
    }
    fields
}

/// The extra values as attributes. Numbers are stored with their own type; anything else as
/// bytes. Names that are taken get a number.
fn extra_fields(extra: &[ExtraField], width: usize, standard: &[Field]) -> Vec<Field> {
    let mut taken: Vec<String> = standard.iter().map(|field| field.name.to_ascii_lowercase()).collect();
    let mut fields = Vec::new();
    let mut at = 0;
    let undescribed = width.saturating_sub(usize::from(ExtraField::total_size(extra)));
    let rest = (undescribed > 0).then(|| ExtraField {
        size: undescribed as u16,
        scalar: None,
        ..ExtraField::number("extra bytes", ScalarType::U8)
    });
    for (index, field) in extra.iter().chain(&rest).enumerate() {
        let size = usize::from(field.size);
        let (scalar, elements) = match field.scalar {
            Some(scalar) if size % scalar.size() == 0 => (scalar, size / scalar.size()),
            _ => (ScalarType::U8, size),
        };
        let base = if field.name.trim().is_empty() { format!("extra {}", index + 1) } else { field.name.clone() };
        let name = (1..)
            .map(|number| if number == 1 { base.clone() } else { format!("{base} {number}") })
            .find(|name| Meaning::of_name(name).is_none() && !taken.contains(&name.to_ascii_lowercase()))
            .expect("some numbered name is free");
        taken.push(name.to_ascii_lowercase());
        let scaled = field.scalar.is_some() && field.is_scaled();
        fields.push(Field {
            meaning: Meaning::Extra(index),
            name,
            description: field.description.clone(),
            scalar,
            elements,
            extra_at: at,
            scaling: scaled.then_some((field.scale, field.offset)),
        });
        at += size;
    }
    fields
}

/// The smallest and largest stored value of each element of a field.
type Range = Vec<(f64, f64)>;

const EMPTY: (f64, f64) = (f64::INFINITY, f64::NEG_INFINITY);

fn widen(range: &mut (f64, f64), value: f64) {
    if value.is_finite() {
        *range = (range.0.min(value), range.1.max(value));
    }
}

/// The Potree folder being written, which is the octree's node sink.
struct PotreeFolder {
    folder: PathBuf,
    octree_path: PathBuf,
    octree: BufWriter<File>,
    fields: Vec<Field>,
    record_size: usize,
    scale: f64,
    offset: [f64; 3],
    /// Value ranges of every field, widened as nodes are encoded.
    ranges: Mutex<Vec<Range>>,
    name: String,
    projection: String,
}

impl PotreeFolder {
    fn create(folder: &Path, layout: &Layout, precision: f64, cloud: &CloudInfo, name: &str) -> Result<PotreeFolder> {
        let fields = fields_for(cloud);
        let octree_path = folder.join(OCTREE_FILE);
        let octree = BufWriter::with_capacity(1 << 20, File::create(&octree_path).at(&octree_path)?);
        let potree = PotreeFolder {
            folder: folder.to_path_buf(),
            octree_path,
            octree,
            record_size: fields.iter().map(Field::size).sum(),
            ranges: Mutex::new(fields.iter().map(|field| vec![EMPTY; field.elements]).collect()),
            fields,
            scale: precision,
            offset: layout.extent.min.map(f64::floor),
            name: name.to_string(),
            projection: cloud.crs_wkt.clone().unwrap_or_default(),
        };
        potree.quantize(layout.extent.max)?; // fails before the octree is built if the cloud is too large
        Ok(potree)
    }

    fn quantize(&self, position: [f64; 3]) -> Result<[f64; 3]> {
        let mut quantized = [0.0; 3];
        for axis in 0..3 {
            let steps = ((position[axis] - self.offset[axis]) / self.scale).round();
            if steps.abs() > f64::from(i32::MAX) {
                return Err(Error::InvalidSetting(format!(
                    "The cloud is too large for a precision of {} m. Use a coarser precision.",
                    self.scale
                )));
            }
            quantized[axis] = steps;
        }
        Ok(quantized)
    }

    /// The values of one standard field of `point`, before they are stored.
    fn values(&self, meaning: Meaning, point: &PointRecord) -> Result<[f64; 3]> {
        let flag = |bit: u8| f64::from(u8::from(point.lidar.flags & bit != 0));
        let one = |value: f64| [value, 0.0, 0.0];
        let values = match meaning {
            Meaning::Position => self.quantize(point.position)?,
            Meaning::Intensity => one(f64::from(point.intensity.clamp(0.0, 1.0)) * 65535.0),
            Meaning::ReturnNumber => one(f64::from(point.returns.number)),
            Meaning::NumberOfReturns => one(f64::from(point.returns.count)),
            Meaning::ClassificationFlags => one(f64::from(point.lidar.flags & LidarDetails::CLASS_FLAGS)),
            Meaning::ScannerChannel => one(f64::from(point.lidar.scanner_channel())),
            Meaning::ScanDirection => one(flag(LidarDetails::SCAN_DIRECTION)),
            Meaning::EdgeOfFlightLine => one(flag(LidarDetails::EDGE_OF_FLIGHT_LINE)),
            Meaning::Classification => one(f64::from(point.classification)),
            Meaning::UserData => one(f64::from(point.lidar.user_data)),
            Meaning::ScanAngle => {
                one((f64::from(point.lidar.scan_angle) / SCAN_ANGLE_STEP).clamp(-SCAN_ANGLE_LIMIT, SCAN_ANGLE_LIMIT))
            }
            Meaning::PointSourceId => one(f64::from(point.lidar.point_source)),
            Meaning::GpsTime => one(point.time),
            Meaning::Rgb => point.color.map(f64::from),
            Meaning::NormalX => one(f64::from(point.normal[0])),
            Meaning::NormalY => one(f64::from(point.normal[1])),
            Meaning::NormalZ => one(f64::from(point.normal[2])),
            Meaning::ScanAngleRank | Meaning::Normal | Meaning::Extra(_) => {
                unreachable!("never among the written fields")
            }
        };
        Ok(values)
    }

    fn metadata(&self, layout: &Layout, ranges: Vec<Range>, hierarchy: HierarchyInfo) -> Metadata {
        let attributes = self
            .fields
            .iter()
            .zip(ranges)
            .map(|(field, range)| {
                let ends = |pick: fn(&(f64, f64)) -> f64| -> Vec<f64> {
                    range
                        .iter()
                        .enumerate()
                        .map(|(axis, ends)| {
                            let value = if ends.0 > ends.1 { 0.0 } else { pick(ends) };
                            match field.meaning {
                                Meaning::Position => value * self.scale + self.offset[axis],
                                _ => value,
                            }
                        })
                        .collect()
                };
                let per_element = |value: f64| Scaling::PerElement(vec![value; field.elements]);
                AttributeInfo {
                    name: field.name.clone(),
                    description: field.description.clone(),
                    size: field.size(),
                    num_elements: field.elements,
                    element_size: field.scalar.size(),
                    kind: type_name(field.scalar).to_string(),
                    min: ends(|ends| ends.0),
                    max: ends(|ends| ends.1),
                    scale: field.scaling.map(|(scale, _)| per_element(scale)),
                    offset: field.scaling.map(|(_, offset)| per_element(offset)),
                }
            })
            .collect();
        Metadata {
            version: "2.0".into(),
            name: self.name.clone(),
            description: String::new(),
            points: layout.point_count,
            projection: self.projection.clone(),
            hierarchy,
            offset: self.offset,
            scale: [self.scale; 3],
            spacing: layout.root_spacing,
            bounding_box: BoundingBox { min: layout.cube.min, max: layout.cube.max() },
            encoding: "DEFAULT".into(),
            attributes,
        }
    }
}

impl NodeSink for PotreeFolder {
    fn encode(&self, _key: NodeKey, points: &PointChunk) -> Result<Vec<u8>> {
        let mut bytes = vec![0; points.len() * self.record_size];
        let mut ranges: Vec<Range> = self.fields.iter().map(|field| vec![EMPTY; field.elements]).collect();
        for (index, record) in bytes.chunks_exact_mut(self.record_size).enumerate() {
            let point = points.record(index);
            let extra = points.extra.as_ref().map(|extra| extra.get(index));
            let mut at = 0;
            for (field, range) in self.fields.iter().zip(&mut ranges) {
                let out = &mut record[at..at + field.size()];
                let width = field.scalar.size();
                match field.meaning {
                    Meaning::Extra(_) => {
                        let extra = extra.expect("chunks carry the cloud's extra values");
                        out.copy_from_slice(&extra[field.extra_at..field.extra_at + field.size()]);
                    }
                    meaning => {
                        let values = self.values(meaning, &point)?;
                        for (element, &value) in values.iter().take(field.elements).enumerate() {
                            put_scalar(field.scalar, value, &mut out[element * width..]);
                        }
                    }
                }
                for (element, range) in range.iter_mut().enumerate() {
                    widen(range, field.scalar.decode(&out[element * width..]));
                }
                at += field.size();
            }
        }
        let mut totals = self.ranges.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
        for (total, node) in totals.iter_mut().flatten().zip(ranges.iter().flatten()) {
            widen(total, node.0);
            widen(total, node.1);
        }
        Ok(bytes)
    }

    fn append(&mut self, _key: NodeKey, bytes: &[u8]) -> Result<()> {
        self.octree.write_all(bytes).at(&self.octree_path)
    }

    fn finish(mut self, layout: &Layout, nodes: &[NodeEntry]) -> Result<()> {
        self.octree.flush().at(&self.octree_path)?;
        let hierarchy = Hierarchy::new(nodes)?;
        let (bytes, first_chunk_size) = hierarchy.bytes();
        let hierarchy_path = self.folder.join(HIERARCHY_FILE);
        fs::write(&hierarchy_path, bytes).at(&hierarchy_path)?;

        let info = HierarchyInfo { first_chunk_size, step_size: u32::from(STEP_SIZE), depth: hierarchy.depth() };
        let ranges = std::mem::take(&mut *self.ranges.lock().unwrap_or_else(|poisoned| poisoned.into_inner()));
        let metadata = self.metadata(layout, ranges, info);
        let mut text = serde_json::to_string_pretty(&metadata).map_err(|error| Error::invalid(error.to_string()))?;
        text.push('\n');
        let metadata_path = self.folder.join(METADATA_FILE);
        fs::write(&metadata_path, text).at(&metadata_path)
    }
}

/// A node as `hierarchy.bin` lists it.
#[derive(Clone, Copy, Debug, Default)]
struct Node {
    points: u32,
    byte_offset: u64,
    byte_size: u64,
    /// Bit `n` is set when child `n` exists, numbered like [`NodeKey::child`].
    children: u8,
}

/// Every node of the octree, with the ones that only connect others filled in.
struct Hierarchy {
    nodes: FxHashMap<NodeKey, Node>,
}

impl Hierarchy {
    fn new(entries: &[NodeEntry]) -> Result<Hierarchy> {
        let mut nodes = FxHashMap::default();
        nodes.insert(NodeKey::ROOT, Node::default());
        for entry in entries {
            let points = u32::try_from(entry.point_count)
                .map_err(|_| Error::Unsupported("Potree nodes hold at most 4 294 967 295 points.".into()))?;
            let byte_offset = if entry.byte_size == 0 { 0 } else { entry.byte_offset };
            let node = nodes.entry(entry.key).or_default();
            *node = Node { points, byte_offset, byte_size: entry.byte_size, ..*node };
            let mut child = entry.key;
            while let Some(parent) = child.parent() {
                nodes.entry(parent).or_default().children |= 1 << child.octant();
                child = parent;
            }
        }
        Ok(Hierarchy { nodes })
    }

    fn depth(&self) -> u32 {
        self.nodes.keys().map(|key| u32::from(key.level)).max().unwrap_or(0)
    }

    fn children(&self, key: NodeKey) -> impl Iterator<Item = NodeKey> {
        let mask = self.nodes[&key].children;
        (0..8).filter(move |octant| mask & (1 << octant) != 0).map(move |octant| key.child(octant))
    }

    /// The chunks of `hierarchy.bin`, root chunk first, each as its nodes in breadth-first
    /// order. A chunk reaches `STEP_SIZE` levels below its first node; nodes there that have
    /// children start chunks of their own.
    fn chunks(&self) -> Vec<Vec<NodeKey>> {
        let mut chunks = Vec::new();
        let mut roots = VecDeque::from([NodeKey::ROOT]);
        while let Some(root) = roots.pop_front() {
            let mut members = Vec::new();
            let mut queue = VecDeque::from([root]);
            while let Some(key) = queue.pop_front() {
                members.push(key);
                if self.is_proxy(root, key) {
                    roots.push_back(key);
                } else if key.level - root.level < STEP_SIZE {
                    queue.extend(self.children(key));
                }
            }
            chunks.push(members);
        }
        chunks
    }

    /// Whether `key`, listed in the chunk of `root`, stands for a chunk of its own.
    fn is_proxy(&self, root: NodeKey, key: NodeKey) -> bool {
        key.level - root.level == STEP_SIZE && self.nodes[&key].children != 0
    }

    /// The bytes of `hierarchy.bin` and the size of its first chunk.
    fn bytes(&self) -> (Vec<u8>, u64) {
        let chunks = self.chunks();
        let mut chunk_at = FxHashMap::default();
        let mut offset = 0;
        for members in &chunks {
            let size = (members.len() * HIERARCHY_ENTRY_SIZE) as u64;
            chunk_at.insert(members[0], (offset, size));
            offset += size;
        }
        let mut bytes = Vec::with_capacity(offset as usize);
        for members in &chunks {
            for &key in members {
                let node = self.nodes[&key];
                let entry = if self.is_proxy(members[0], key) {
                    let (byte_offset, byte_size) = chunk_at[&key];
                    HierarchyEntry {
                        kind: NODE_PROXY,
                        child_mask: node.children,
                        points: node.points,
                        byte_offset,
                        byte_size,
                    }
                } else {
                    HierarchyEntry {
                        kind: if node.children != 0 { NODE_NORMAL } else { NODE_LEAF },
                        child_mask: node.children,
                        points: node.points,
                        byte_offset: node.byte_offset,
                        byte_size: node.byte_size,
                    }
                };
                entry.write_to(&mut bytes);
            }
        }
        (bytes, chunks[0].len() as u64 * HIERARCHY_ENTRY_SIZE as u64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Attributes;

    fn entry(level: u8, x: u32, y: u32, z: u32, points: u64) -> NodeEntry {
        NodeEntry { key: NodeKey { level, x, y, z }, point_count: points, byte_offset: points * 10, byte_size: points }
    }

    #[test]
    fn deep_branches_are_split_into_chunks() {
        // A single branch down to level 9: chunks start at levels 0, 4 and 8.
        let entries: Vec<NodeEntry> = (0..=9).rev().map(|level| entry(level, 0, 0, 0, u64::from(level) + 1)).collect();
        let hierarchy = Hierarchy::new(&entries).unwrap();
        let chunks = hierarchy.chunks();
        let levels: Vec<Vec<u8>> = chunks.iter().map(|chunk| chunk.iter().map(|key| key.level).collect()).collect();
        assert_eq!(levels, [vec![0, 1, 2, 3, 4], vec![4, 5, 6, 7, 8], vec![8, 9]]);
        assert_eq!(hierarchy.depth(), 9);

        let (bytes, first) = hierarchy.bytes();
        assert_eq!(first, 5 * 22);
        assert_eq!(bytes.len(), 12 * 22);
        let at = |index: usize| HierarchyEntry::read_from(&bytes[index * 22..]);
        assert_eq!((at(4).kind, at(4).byte_offset, at(4).byte_size), (NODE_PROXY, 5 * 22, 5 * 22));
        assert_eq!((at(5).kind, at(5).points, at(5).byte_offset), (NODE_NORMAL, 5, 50));
        assert_eq!((at(11).kind, at(11).child_mask), (NODE_LEAF, 0));
    }

    #[test]
    fn missing_ancestors_connect_the_nodes() {
        let hierarchy = Hierarchy::new(&[entry(2, 3, 0, 1, 7), entry(0, 0, 0, 0, 2)]).unwrap();
        let chunk = &hierarchy.chunks()[0];
        assert_eq!(chunk.len(), 3);
        let parent = NodeKey { level: 1, x: 1, y: 0, z: 0 };
        assert_eq!(hierarchy.nodes[&NodeKey::ROOT].children, 1 << 4);
        assert_eq!(hierarchy.nodes[&parent].children, 1 << 5);
        assert_eq!(hierarchy.nodes[&parent].points, 0);
    }

    #[test]
    fn extra_value_names_never_clash() {
        let mut cloud = CloudInfo {
            attributes: Attributes { intensity: true, extra_bytes: 9, ..Attributes::NONE },
            ..CloudInfo::default()
        };
        cloud.extra_fields = vec![
            ExtraField::number("Intensity", ScalarType::U16),
            ExtraField::number("", ScalarType::F32),
            ExtraField { size: 2, scalar: None, ..ExtraField::number("raw", ScalarType::U8) },
        ];
        let fields = fields_for(&cloud);
        let names: Vec<&str> = fields.iter().map(|field| field.name.as_str()).collect();
        assert_eq!(names, ["position", "intensity", "Intensity 2", "extra 2", "raw", "extra bytes"]);
        let raw = &fields[4];
        assert_eq!((raw.scalar, raw.elements, raw.extra_at), (ScalarType::U8, 2, 6));
        assert_eq!((fields[5].elements, fields[5].extra_at), (1, 8));
    }
}
