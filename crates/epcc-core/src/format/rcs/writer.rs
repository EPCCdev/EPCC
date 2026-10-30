//! Writing RCS, out of core.
//!
//! 1. **Stage**: the points are spilled to a temporary file while their extent and intensity
//!    range are measured. A point closer than 0.5 mm to the last kept one is dropped here.
//! 2. **Partition**: the root cube follows from the extent, and the octree is fixed down to a
//!    set level. Each point updates the nodes it passes through and goes to the temporary file
//!    of its node at that level.
//! 3. **Leaves**: those nodes become leaves in the order their first point arrived. A node with
//!    more points than a leaf may hold is split, its children queued at the end. Leaves are
//!    built on all cores, a batch at a time, and their blocks appended to a temporary file.
//! 4. **Assemble**: header, leaf table and node table are written in front of the blocks.
//!
//! Memory holds one batch of leaves (a few million points) and the node table; everything else
//! is on disk, next to the output, and removed when the conversion ends.

use std::collections::VecDeque;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

use rayon::prelude::*;

use super::leaf::{self, Block, Point, child_min, octant};
use super::{
    HEADER_SIZE, LEAF_RECORD_SIZE, MAGIC, NODE_RECORD_SIZE, SECTION_ENTRY_SIZE, SECTION_LEAVES, SECTION_NODES,
    SECTION_POINTS, SECTION_PROPERTIES, SECTION_TABLE, header, leaf_record, normals, put_f64s, put_u32s,
};
use crate::error::IoContext;
use crate::format::{PointWriter, WriteRequest};
use crate::model::{Attributes, PointChunk};
use crate::octree::drop_invalid_positions;
use crate::octree::staging::{LogWriter, PointFile, PointFileWriter, Segment, TempFolder, decode_records};
use crate::{Error, Progress, Result};

/// A point closer than this to the last kept point is dropped on import, in metres.
const MIN_POINT_DISTANCE: f64 = 0.0005;
/// Below this half-extent plus 1 cm, the root cube is the extent scaled by 1.44 and the leaves
/// are at [`SMALL_LEAF_DEPTH`].
const LARGE_EXTENT: f64 = 262.0;
const SMALL_LEAF_DEPTH: usize = 4;
/// Larger clouds get a root cube of 1.44 times this unit times a power of two, and leaves of
/// about [`LARGE_LEAF_EDGE`].
const LARGE_ROOT_UNIT: f64 = 327.375;
const LARGE_LEAF_EDGE: f64 = 235.71;
/// The node table holds at most this many levels.
const MAX_LEVELS: usize = 32;
/// The deepest fixed leaf level whose path fits into 64 bits.
const MAX_LEAF_DEPTH: usize = 21;
/// Points of the leaves built at the same time.
const BATCH_POINTS: u64 = 2_000_000;
/// A node's temporary records are written once they fill a page, or all at once when the
/// pages together reach the buffer size.
const PAGE_BYTES: usize = 1 << 20;
const BUFFER_BYTES: usize = 128 << 20;
/// The part of the work after reading that partitioning takes; building the leaves takes the
/// rest.
const PARTITION_SHARE: f32 = 0.3;
const NO_NODE: u32 = u32::MAX;

pub fn create(request: &WriteRequest) -> Result<Box<dyn PointWriter>> {
    let work_folder = request.path.parent().map_or_else(|| PathBuf::from("."), Path::to_path_buf);
    Ok(Box::new(RcsWriter {
        path: request.path.to_path_buf(),
        work_folder,
        attributes: request.cloud.attributes,
        estimate_normals: request.settings.rcs.estimate_normals,
    }))
}

struct RcsWriter {
    path: PathBuf,
    work_folder: PathBuf,
    attributes: Attributes,
    estimate_normals: bool,
}

impl PointWriter for RcsWriter {
    fn write(self: Box<Self>, chunks: &mut dyn Iterator<Item = PointChunk>, progress: &Progress) -> Result<()> {
        let temp = TempFolder::create(&self.work_folder)?;
        let (staged, stats) = stage(chunks, self.attributes, temp.path(), progress)?;
        if stats.count == 0 {
            return Err(Error::invalid("RCS: there are no points to write"));
        }
        let point_count = u32::try_from(stats.count)
            .map_err(|_| Error::Unsupported(format!("RCS files hold at most {} points", u32::MAX)))?;
        let frame = Frame::new(&stats, self.attributes, self.estimate_normals)?;
        let mut tree = Tree::new(&frame);
        tree.partition(staged, &frame, temp.path(), progress)?;
        let blocks_path = temp.path().join("blocks.bin");
        let leaves = tree.build_leaves(&frame, &blocks_path, temp.path(), progress)?;
        let nodes = tree.node_section()?;
        write_file(&self.path, &frame, point_count, &leaves, &nodes, &blocks_path)?;
        progress.set_after_reading_done(1.0);
        Ok(())
    }
}

/// What staging measures.
struct Stats {
    count: u64,
    min: [f64; 3],
    max: [f64; 3],
    intensity_min: f32,
    intensity_max: f32,
    any_intensity: bool,
}

/// Spills the points to a temporary file, leaving out repeated ones, and measures them.
fn stage(
    chunks: &mut dyn Iterator<Item = PointChunk>,
    attributes: Attributes,
    folder: &Path,
    progress: &Progress,
) -> Result<(PointFile, Stats)> {
    let mut writer = PointFileWriter::create(&folder.join("staged.bin"), attributes)?;
    let mut stats = Stats {
        count: 0,
        min: [f64::INFINITY; 3],
        max: [f64::NEG_INFINITY; 3],
        intensity_min: f32::INFINITY,
        intensity_max: f32::NEG_INFINITY,
        any_intensity: false,
    };
    let mut last_kept = None;
    for mut chunk in chunks {
        progress.check_cancelled()?;
        drop_invalid_positions(&mut chunk, progress);
        drop_repeated(&mut chunk, &mut last_kept);
        stats.count += chunk.len() as u64;
        for position in &chunk.positions {
            for (axis, &value) in position.iter().enumerate() {
                stats.min[axis] = stats.min[axis].min(value);
                stats.max[axis] = stats.max[axis].max(value);
            }
        }
        for &value in chunk.intensity.iter().flatten() {
            stats.intensity_min = stats.intensity_min.min(value);
            stats.intensity_max = stats.intensity_max.max(value);
            stats.any_intensity |= value != 0.0;
        }
        writer.write(&chunk)?;
    }
    Ok((writer.finish()?, stats))
}

/// Drops every point closer than [`MIN_POINT_DISTANCE`] to the last point kept before it.
fn drop_repeated(chunk: &mut PointChunk, last_kept: &mut Option<[f64; 3]>) {
    let mut keep = Vec::with_capacity(chunk.len());
    for &position in &chunk.positions {
        let repeated = last_kept.is_some_and(|last: [f64; 3]| {
            let d = [position[0] - last[0], position[1] - last[1], position[2] - last[2]];
            (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt() < MIN_POINT_DISTANCE
        });
        if !repeated {
            *last_kept = Some(position);
        }
        keep.push(!repeated);
    }
    if keep.contains(&false) {
        chunk.retain(&keep);
    }
}

/// How the 8-bit intensity is made.
#[derive(Clone, Copy, Debug)]
enum IntensityRule {
    /// The source intensity, scaled from its range onto 0..255.
    Scaled { min: f64, max: f64 },
    /// Without intensity, the height, scaled from the cloud's range onto 0..255.
    Height { min: f64, max: f64 },
    /// The same value for every point: 0 for constant intensity, 255 for a flat cloud without.
    Constant(u8),
}

/// The cloud's frame and the conversion of its points, fixed once all points are staged.
struct Frame {
    centre: [f64; 3],
    half_size: f64,
    leaf_depth: usize,
    attributes: Attributes,
    intensity: IntensityRule,
    /// Whether the intensity is real rather than missing or all zero.
    has_intensity: bool,
    /// The source intensity range, as the properties record it.
    intensity_range: [f32; 2],
    estimate_normals: bool,
}

impl Frame {
    fn new(stats: &Stats, attributes: Attributes, estimate_normals: bool) -> Result<Frame> {
        let centre: [f64; 3] = std::array::from_fn(|axis| (stats.min[axis] + stats.max[axis]) / 2.0);
        let extent = (0..3).map(|axis| (stats.max[axis] - stats.min[axis]) / 2.0).fold(f64::NEG_INFINITY, f64::max);
        let small = extent + 0.01 < LARGE_EXTENT;
        let half_size = if small {
            (extent + 0.01) / (1.0 / 1.44)
        } else {
            let mut unit = LARGE_ROOT_UNIT;
            while 1.44 * unit < extent {
                unit *= 2.0;
            }
            1.44 * unit
        };
        let leaf_depth =
            if small { SMALL_LEAF_DEPTH } else { (2.0 * half_size / LARGE_LEAF_EDGE).log2().round() as usize };
        if leaf_depth > MAX_LEAF_DEPTH {
            return Err(Error::Unsupported("RCS: the cloud is too large in extent".into()));
        }
        let has_intensity = attributes.intensity && stats.any_intensity;
        let intensity = if has_intensity {
            let (min, max) = (f64::from(stats.intensity_min), f64::from(stats.intensity_max));
            if max > min { IntensityRule::Scaled { min, max } } else { IntensityRule::Constant(0) }
        } else if stats.max[2] > stats.min[2] {
            IntensityRule::Height { min: stats.min[2], max: stats.max[2] }
        } else {
            IntensityRule::Constant(255)
        };
        let intensity_range =
            if attributes.intensity { [stats.intensity_min, stats.intensity_max] } else { [0.0, 0.0] };
        Ok(Frame {
            centre,
            half_size,
            leaf_depth,
            attributes,
            intensity,
            has_intensity,
            intensity_range,
            estimate_normals: estimate_normals && !attributes.normal,
        })
    }

    fn root_min(&self) -> [f64; 3] {
        [-self.half_size; 3]
    }

    fn root_edge(&self) -> f64 {
        2.0 * self.half_size
    }

    /// The chunk's points as the writer stores them.
    fn points(&self, chunk: &PointChunk) -> Vec<Point> {
        (0..chunk.len()).into_par_iter().map(|index| self.point(chunk, index)).collect()
    }

    fn point(&self, chunk: &PointChunk, index: usize) -> Point {
        let absolute = chunk.positions[index];
        let position = std::array::from_fn(|axis| absolute[axis] - self.centre[axis]);
        let scaled = |value: f64, min: f64, max: f64| (255.0 * (value - min) / (max - min)).floor() as u8;
        let intensity = match self.intensity {
            IntensityRule::Scaled { min, max } => {
                chunk.intensity.as_ref().map_or(0, |values| scaled(f64::from(values[index]), min, max))
            }
            IntensityRule::Height { min, max } => scaled(absolute[2], min, max),
            IntensityRule::Constant(value) => value,
        };
        // Without colour, the intensity is stored as grey.
        let [red, green, blue] =
            chunk.color.as_ref().map_or([intensity; 3], |values| values[index].map(|c| (c >> 8) as u8));
        let normal = chunk.normal.as_ref().map_or(0, |values| normals::encode(values[index].map(f64::from)));
        Point { position, absolute, rgbi: [red, green, blue, intensity], normal }
    }
}

/// A node of the octree.
struct Node {
    min: [f64; 3],
    edge: f64,
    depth: usize,
    children: [u32; 8],
    /// Running average of the colour and intensity of every point in the node's cube, in input
    /// order: each point halves the distance to its own value.
    rgbi: [u8; 4],
    /// The source normal code of the last point in the cube.
    normal: u16,
    points: u64,
    role: Role,
    /// Where the node's points wait until it becomes a leaf or is split.
    bucket: Option<PointFile>,
    /// While points are distributed, the node's file among the files being filled.
    slot: usize,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Role {
    Inner,
    /// The leaf with this index in the leaf table.
    Leaf(u32),
    /// Not yet decided.
    Open,
}

impl Node {
    fn new(min: [f64; 3], edge: f64, depth: usize, role: Role) -> Node {
        Node {
            min,
            edge,
            depth,
            children: [NO_NODE; 8],
            rgbi: [0; 4],
            normal: 0,
            points: 0,
            role,
            bucket: None,
            slot: 0,
        }
    }

    fn add(&mut self, point: &Point) {
        if self.points == 0 {
            self.rgbi = point.rgbi;
        } else {
            for (average, value) in self.rgbi.iter_mut().zip(point.rgbi) {
                *average = ((u16::from(*average) + u16::from(value)) >> 1) as u8;
            }
        }
        self.normal = point.normal;
        self.points += 1;
    }
}

struct Tree {
    nodes: Vec<Node>,
    /// The nodes at the fixed leaf level, in the order their first point arrived.
    fixed: Vec<u32>,
}

impl Tree {
    fn new(frame: &Frame) -> Tree {
        Tree { nodes: vec![Node::new(frame.root_min(), frame.root_edge(), 0, Role::Inner)], fixed: Vec::new() }
    }

    /// Adds child `octant` to `parent` and returns its index.
    fn add_child(&mut self, parent: u32, octant: usize, role: Role) -> u32 {
        let node = &self.nodes[parent as usize];
        let half = node.edge / 2.0;
        let child = Node::new(child_min(node.min, octant, half), half, node.depth + 1, role);
        let index = self.nodes.len() as u32;
        self.nodes.push(child);
        self.nodes[parent as usize].children[octant] = index;
        index
    }

    /// Passes every staged point down to the fixed level, updating the nodes on its way, and
    /// gives each node at that level a temporary file of its points.
    fn partition(&mut self, staged: PointFile, frame: &Frame, folder: &Path, progress: &Progress) -> Result<()> {
        let depth = frame.leaf_depth;
        let size = staged.record_size();
        let total = staged.point_count;
        let mut buckets = Buckets::create(&folder.join("cells.log"))?;
        let mut path_nodes = vec![0u32; depth + 1];
        let mut bucket_of_path = 0;
        let mut last_path = None;
        let mut done = 0u64;
        staged.read_records(&mut |records| {
            progress.check_cancelled()?;
            let chunk = decode_records(records, frame.attributes);
            let points = frame.points(&chunk);
            let paths: Vec<u64> = points
                .par_iter()
                .map(|point| path_of(point.position, frame.root_min(), frame.root_edge(), depth))
                .collect();
            for (index, (point, &path)) in points.iter().zip(&paths).enumerate() {
                if last_path != Some(path) {
                    for level in 0..depth {
                        let octant = ((path >> (3 * (depth - 1 - level))) & 7) as usize;
                        let parent = path_nodes[level];
                        let mut child = self.nodes[parent as usize].children[octant];
                        if child == NO_NODE {
                            let role = if level + 1 < depth { Role::Inner } else { Role::Open };
                            child = self.add_child(parent, octant, role);
                            if level + 1 == depth {
                                self.nodes[child as usize].slot = self.fixed.len();
                                self.fixed.push(child);
                                buckets.add();
                            }
                        }
                        path_nodes[level + 1] = child;
                    }
                    bucket_of_path = self.nodes[path_nodes[depth] as usize].slot;
                    last_path = Some(path);
                }
                for &node in &path_nodes {
                    self.nodes[node as usize].add(point);
                }
                buckets.push(bucket_of_path, &records[index * size..(index + 1) * size])?;
            }
            done += chunk.len() as u64;
            progress.set_after_reading_done(PARTITION_SHARE * done as f32 / total as f32);
            Ok(())
        })?;
        staged.remove()?;
        for (&node, file) in self.fixed.iter().zip(buckets.finish(frame.attributes)?) {
            self.nodes[node as usize].bucket = Some(file);
        }
        Ok(())
    }

    /// Turns the open nodes into leaves, splitting the ones with too many points, and writes
    /// their blocks to `blocks_path`. Returns the leaf table.
    fn build_leaves(
        &mut self,
        frame: &Frame,
        blocks_path: &Path,
        folder: &Path,
        progress: &Progress,
    ) -> Result<LeafTable> {
        let mut blocks = BufWriter::with_capacity(8 << 20, File::create(blocks_path).at(blocks_path)?);
        let mut table = LeafTable::default();
        let mut queue: VecDeque<u32> = self.fixed.iter().copied().collect();
        let total = self.nodes[0].points;
        let mut done = 0u64;
        while let Some(&front) = queue.front() {
            progress.check_cancelled()?;
            if self.nodes[front as usize].points > leaf::MAX_POINTS as u64 {
                queue.pop_front();
                let children = self.split(front, frame, folder)?;
                queue.extend(children);
                continue;
            }
            let mut batch = Vec::new();
            let mut batch_points = 0;
            while let Some(&node) = queue.front() {
                let points = self.nodes[node as usize].points;
                if points > leaf::MAX_POINTS as u64 || (!batch.is_empty() && batch_points + points > BATCH_POINTS) {
                    break;
                }
                queue.pop_front();
                batch.push(node);
                batch_points += points;
            }
            let inputs: Vec<(PointFile, [f64; 3], f64)> = batch
                .iter()
                .map(|&node| {
                    let node = &mut self.nodes[node as usize];
                    (node.bucket.take().expect("an open node has its points"), node.min, node.edge)
                })
                .collect();
            let built: Vec<Block> = inputs
                .into_par_iter()
                .map(|(file, min, edge)| {
                    let chunk = file.read_all()?;
                    file.remove()?;
                    leaf::build(min, edge, &frame.points(&chunk), frame.estimate_normals)
                })
                .collect::<Result<_>>()?;
            for (&node, block) in batch.iter().zip(&built) {
                let node = &mut self.nodes[node as usize];
                node.role = Role::Leaf(table.count);
                blocks.write_all(&block.bytes).at(blocks_path)?;
                table.add(node.min, node.edge, block);
            }
            done += batch_points;
            progress.set_after_reading_done(PARTITION_SHARE + (1.0 - PARTITION_SHARE) * done as f32 / total as f32);
        }
        blocks.flush().at(blocks_path)?;
        Ok(table)
    }

    /// Splits a node into its children, in the order their first point arrived.
    fn split(&mut self, node: u32, frame: &Frame, folder: &Path) -> Result<Vec<u32>> {
        let parent = &mut self.nodes[node as usize];
        if parent.depth + 1 >= MAX_LEVELS {
            return Err(Error::Unsupported(format!(
                "RCS: more than {} points lie too close together to be stored",
                leaf::MAX_POINTS
            )));
        }
        parent.role = Role::Inner;
        let (min, half) = (parent.min, parent.edge / 2.0);
        let file = parent.bucket.take().expect("an open node has its points");
        let size = file.record_size();
        let mut buckets = Buckets::create(&folder.join(format!("split-{node}.log")))?;
        let mut children = Vec::new();
        file.read_records(&mut |records| {
            let chunk = decode_records(records, frame.attributes);
            for (index, point) in frame.points(&chunk).iter().enumerate() {
                let octant = octant(point.position, min, half);
                let mut child = self.nodes[node as usize].children[octant];
                if child == NO_NODE {
                    child = self.add_child(node, octant, Role::Open);
                    self.nodes[child as usize].slot = children.len();
                    children.push(child);
                    buckets.add();
                }
                let child = &mut self.nodes[child as usize];
                child.add(point);
                buckets.push(child.slot, &records[index * size..(index + 1) * size])?;
            }
            Ok(())
        })?;
        file.remove()?;
        for (&child, file) in children.iter().zip(buckets.finish(frame.attributes)?) {
            self.nodes[child as usize].bucket = Some(file);
        }
        Ok(children)
    }

    /// The node section: nodes per level, then every node breadth first, each level's
    /// children grouped by parent in octant order.
    fn node_section(&self) -> Result<Vec<u8>> {
        let mut levels: Vec<Vec<u32>> = vec![vec![0]];
        loop {
            let next: Vec<u32> = levels[levels.len() - 1]
                .iter()
                .flat_map(|&node| self.nodes[node as usize].children.into_iter().filter(|&child| child != NO_NODE))
                .collect();
            if next.is_empty() {
                break;
            }
            levels.push(next);
        }
        if levels.len() > MAX_LEVELS {
            return Err(Error::Unsupported("RCS: the octree would be deeper than 32 levels".into()));
        }
        let node_count: usize = levels.iter().map(Vec::len).sum();
        let mut bytes = vec![0u8; 4 * MAX_LEVELS];
        let counts: Vec<u32> = levels.iter().map(|level| level.len() as u32).collect();
        put_u32s(&mut bytes, 0, &counts);
        bytes.reserve(node_count * NODE_RECORD_SIZE);
        for level in &levels {
            let mut next_position = 0i32;
            for &index in level {
                let node = &self.nodes[index as usize];
                let mut record = [0u8; NODE_RECORD_SIZE];
                let [red, green, blue, intensity] = node.rgbi;
                let [normal_low, normal_high] = node.normal.to_le_bytes();
                let mask = (0..8).filter(|&octant| node.children[octant] != NO_NODE).fold(0u8, |m, o| m | 1 << o);
                record[..8].copy_from_slice(&[red, green, blue, intensity, normal_low, normal_high, mask, 0]);
                let max: [f64; 3] = std::array::from_fn(|axis| node.min[axis] + node.edge);
                put_f64s(&mut record, 8, &node.min);
                put_f64s(&mut record, 32, &max);
                let tail = match node.role {
                    Role::Leaf(leaf) => [0, leaf as i32, -1, 0],
                    Role::Inner => {
                        let tail = [0, -1, next_position, 0];
                        next_position += mask.count_ones() as i32;
                        tail
                    }
                    Role::Open => return Err(Error::invalid("RCS: a node was left without points")),
                };
                for (slot, value) in tail.iter().enumerate() {
                    record[56 + 4 * slot..60 + 4 * slot].copy_from_slice(&value.to_le_bytes());
                }
                bytes.extend_from_slice(&record);
            }
        }
        Ok(bytes)
    }
}

/// The octant at every level below the root, three bits per level, the root's first.
fn path_of(position: [f64; 3], root_min: [f64; 3], root_edge: f64, depth: usize) -> u64 {
    let (mut min, mut edge, mut path) = (root_min, root_edge, 0u64);
    for _ in 0..depth {
        let half = edge / 2.0;
        let octant = octant(position, min, half);
        path = path << 3 | octant as u64;
        min = child_min(min, octant, half);
        edge = half;
    }
    path
}

/// Temporary point files for many nodes, sharing one log.
struct Buckets {
    log: LogWriter,
    pages: Vec<Vec<u8>>,
    segments: Vec<Vec<Segment>>,
    buffered: usize,
}

impl Buckets {
    fn create(path: &Path) -> Result<Buckets> {
        Ok(Buckets { log: LogWriter::create(path)?, pages: Vec::new(), segments: Vec::new(), buffered: 0 })
    }

    fn add(&mut self) {
        self.pages.push(Vec::new());
        self.segments.push(Vec::new());
    }

    fn push(&mut self, bucket: usize, record: &[u8]) -> Result<()> {
        self.pages[bucket].extend_from_slice(record);
        self.buffered += record.len();
        if self.pages[bucket].len() >= PAGE_BYTES {
            self.flush(bucket)?;
        } else if self.buffered >= BUFFER_BYTES {
            for bucket in 0..self.pages.len() {
                self.flush(bucket)?;
            }
        }
        Ok(())
    }

    fn flush(&mut self, bucket: usize) -> Result<()> {
        let page = std::mem::take(&mut self.pages[bucket]);
        if !page.is_empty() {
            self.log.append(&page, &mut self.segments[bucket])?;
            self.buffered -= page.len();
        }
        Ok(())
    }

    /// One point file per bucket, in the order they were added.
    fn finish(mut self, attributes: Attributes) -> Result<Vec<PointFile>> {
        for bucket in 0..self.pages.len() {
            self.flush(bucket)?;
        }
        let log = self.log.finish()?;
        Ok(self.segments.into_iter().map(|segments| log.point_file(attributes, segments)).collect())
    }
}

/// The leaf section as it grows, and the bounds of all points.
#[derive(Default)]
struct LeafTable {
    count: u32,
    records: Vec<u8>,
    blocks_size: u64,
    points_min: Option<[f64; 3]>,
    points_max: Option<[f64; 3]>,
}

impl LeafTable {
    fn add(&mut self, min: [f64; 3], edge: f64, block: &Block) {
        let mut record = [0u8; LEAF_RECORD_SIZE];
        let max: [f64; 3] = std::array::from_fn(|axis| min[axis] + edge);
        put_f64s(&mut record, leaf_record::CUBE_MIN, &min);
        put_f64s(&mut record, leaf_record::CUBE_MAX, &max);
        put_f64s(&mut record, leaf_record::POINTS_MIN, &block.points_min);
        put_f64s(&mut record, leaf_record::POINTS_MAX, &block.points_max);
        let levels = block.inner_per_level.len() as u32;
        put_u32s(&mut record, leaf_record::LEVELS, &[levels, block.stored as u32, block.bytes.len() as u32]);
        put_u32s(&mut record, leaf_record::INNER_PER_LEVEL, &block.inner_per_level);
        put_u32s(&mut record, leaf_record::LEAVES_PER_LEVEL, &block.points_per_level);
        self.records.extend_from_slice(&record);
        self.count += 1;
        self.blocks_size += block.bytes.len() as u64;
        let merge = |bound: &mut Option<[f64; 3]>, values: [f64; 3], pick: fn(f64, f64) -> f64| {
            *bound = Some(bound.map_or(values, |old| std::array::from_fn(|axis| pick(old[axis], values[axis]))));
        };
        merge(&mut self.points_min, block.points_min, f64::min);
        merge(&mut self.points_max, block.points_max, f64::max);
    }

    fn section(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(8 + self.records.len());
        bytes.extend_from_slice(&self.count.to_le_bytes());
        bytes.extend_from_slice(&0u32.to_le_bytes());
        bytes.extend_from_slice(&self.records);
        bytes
    }
}

/// The properties section: tagged values whose meaning is only partly known; these are the
/// values every known file has, with the intensity range filled in.
fn properties(frame: &Frame) -> Vec<u8> {
    use std::f64::consts::{FRAC_PI_2, PI};
    let mut bytes = 11u32.to_le_bytes().to_vec();
    let mut add = |key: u32, data: &[u8]| {
        bytes.extend_from_slice(&key.to_le_bytes());
        bytes.extend_from_slice(&(data.len() as u32).to_le_bytes());
        bytes.extend_from_slice(data);
    };
    let [intensity_min, intensity_max] = frame.intensity_range;
    add(100, &[1]);
    add(101, &intensity_max.to_le_bytes());
    add(102, &intensity_min.to_le_bytes());
    add(201, &0f32.to_le_bytes());
    add(202, &0f32.to_le_bytes());
    add(203, &PI.to_le_bytes());
    add(204, &(-PI).to_le_bytes());
    add(205, &FRAC_PI_2.to_le_bytes());
    add(206, &(-FRAC_PI_2).to_le_bytes());
    add(301, &[1]);
    let mut name = [0u8; 256];
    name[0] = 1;
    add(302, &name);
    bytes
}

fn write_file(
    path: &Path,
    frame: &Frame,
    point_count: u32,
    leaves: &LeafTable,
    nodes: &[u8],
    blocks_path: &Path,
) -> Result<()> {
    let leaf_section = leaves.section();
    let properties = properties(frame);
    let h = frame.half_size;
    let mut head = vec![0u8; HEADER_SIZE];
    head[..8].copy_from_slice(MAGIC);
    put_u32s(&mut head, 8, &[3, 2]);
    put_f64s(&mut head, header::CENTRE, &frame.centre);
    put_f64s(&mut head, header::SCALE, &[1.0; 3]);
    put_f64s(&mut head, header::CENTRE_COPY, &frame.centre);
    put_f64s(&mut head, header::ROOT_CUBE, &[-h, -h, -h, h, h, h]);
    let (min, max) = (leaves.points_min.unwrap_or_default(), leaves.points_max.unwrap_or_default());
    put_f64s(&mut head, header::BOUNDS, &[min[0], min[1], min[2], max[0], max[1], max[2]]);
    // Import codes of a LAS source, the layout this writer reproduces whatever the source.
    put_u32s(&mut head, header::IMPORT_CODES, &[11, 1]);
    put_u32s(&mut head, header::POINT_COUNT, &[point_count]);
    head[header::HAS_COLOR] = u8::from(frame.attributes.color);
    head[header::HAS_NORMALS] = 1;
    head[header::HAS_INTENSITY] = u8::from(frame.has_intensity);

    let sections = [
        (SECTION_LEAVES, leaf_section.len() as u64),
        (SECTION_NODES, nodes.len() as u64),
        (SECTION_POINTS, leaves.blocks_size),
        (SECTION_PROPERTIES, properties.len() as u64),
    ];
    put_u32s(&mut head, SECTION_TABLE, &[sections.len() as u32]);
    let mut offset = HEADER_SIZE as u64;
    for (index, (kind, size)) in sections.into_iter().enumerate() {
        let at = SECTION_TABLE + 4 + SECTION_ENTRY_SIZE * index;
        for (slot, value) in [kind, offset, size].into_iter().enumerate() {
            head[at + 8 * slot..at + 8 * slot + 8].copy_from_slice(&value.to_le_bytes());
        }
        offset += size;
    }
    let guid = guid_for(&[&head, &leaf_section, nodes, &properties]);
    head[header::GUID..header::GUID + guid.len()].copy_from_slice(guid.as_bytes());

    let mut file = BufWriter::with_capacity(8 << 20, File::create(path).at(path)?);
    file.write_all(&head).at(path)?;
    file.write_all(&leaf_section).at(path)?;
    file.write_all(nodes).at(path)?;
    let mut blocks = File::open(blocks_path).at(blocks_path)?;
    std::io::copy(&mut blocks, &mut file).at(path)?;
    file.write_all(&properties).at(path)?;
    file.flush().at(path)
}

/// The scan's identifier: a version 4 style GUID in braces, with underscores for dashes.
/// It is derived from the file's content, so the same input always gives the same file.
fn guid_for(parts: &[&[u8]]) -> String {
    // 128-bit FNV-1a.
    const PRIME: u128 = 0x0000_0000_0100_0000_0000_0000_0000_013B;
    let mut hash: u128 = 0x6C62_272E_07BB_0142_62B8_2175_6295_C58D;
    for part in parts {
        for &byte in *part {
            hash ^= u128::from(byte);
            hash = hash.wrapping_mul(PRIME);
        }
    }
    let hex = format!("{hash:032x}");
    let variant = ["8", "9", "a", "b"][(hash >> 60 & 3) as usize];
    format!("{{{}_{}_4{}_{}{}_{}}}", &hex[0..8], &hex[8..12], &hex[13..16], variant, &hex[17..20], &hex[20..32])
}
