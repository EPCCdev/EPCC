//! Builds an octree out of core, for the formats that store one (COPC, Potree). RCS, whose
//! tree is laid out differently, reuses only the temporary point files of `staging`.
//!
//! The steps, each bounded in memory however large the cloud is:
//! 1. **Stage**: the incoming points are spilled to a temporary file while their extent is
//!    measured, because the octree's cube is only known once every point was seen.
//! 2. **Partition**: the staged points are split into subtrees small enough to build in memory.
//! 3. **Index**: each subtree is built in memory. Nodes with too many points are split; every
//!    inner node then takes a thinned-out sample of its children's points.
//! 4. **Top**: the nodes above the subtrees are sampled the same way, up to the root, each as
//!    soon as the subtrees below it are built.
//!
//! Every point ends up in exactly one node. Finished nodes go to a [`NodeSink`], which encodes
//! them in the output format.

mod extent;
mod index;
mod key;
mod partition;
pub(crate) mod staging;
#[cfg(test)]
mod tests;

use std::iter::Peekable;
use std::path::Path;

use rayon::prelude::*;

pub use extent::Extent;
pub use key::{Cube, NodeKey};

use self::index::{BuiltNode, OpenNode};
use self::partition::SubtreeFile;
use self::staging::{PointFile, PointFileWriter, TempFolder};
use crate::model::{Attributes, PointChunk};
use crate::{Error, Progress, Result};

/// The deepest level node keys can address: one integer cell per axis must fit into 32 bits.
pub const MAX_DEPTH: u8 = 31;

/// The part of the work after reading that partitioning takes; building the nodes takes the rest.
const PARTITION_SHARE: f32 = 0.35;

/// How the octree is shaped. Each output format picks the values its viewers expect.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OctreeParams {
    /// An inner node keeps at most one point per cell of a grid with this many cells along each
    /// edge of the node.
    pub grid_cells: u32,
    /// A node with more points than this is split into children.
    pub max_leaf_points: usize,
    /// The largest subtree built in memory at once.
    pub max_subtree_points: usize,
    /// Nodes are never split below this depth, whatever their point count. Points closer
    /// together than a node at this depth stay in one leaf.
    pub max_depth: u8,
}

impl Default for OctreeParams {
    fn default() -> Self {
        OctreeParams { grid_cells: 128, max_leaf_points: 100_000, max_subtree_points: 5_000_000, max_depth: MAX_DEPTH }
    }
}

impl OctreeParams {
    /// Checks that the parameters describe an octree that can be built.
    pub fn validate(&self) -> Result<()> {
        let problem = if self.grid_cells < 2 {
            "the sampling grid needs at least 2 cells per edge".to_string()
        } else if self.max_leaf_points == 0 {
            "leaves must hold at least one point".to_string()
        } else if self.max_subtree_points < self.max_leaf_points {
            "a subtree must hold at least as many points as a leaf".to_string()
        } else if self.max_depth > MAX_DEPTH {
            format!("the depth {} is above the limit of {MAX_DEPTH}", self.max_depth)
        } else {
            return Ok(());
        };
        Err(Error::InvalidSetting(format!("Invalid octree parameters: {problem}.")))
    }
}

/// What is known about the octree once all points are staged, before the first node is built.
#[derive(Clone, Debug, PartialEq)]
pub struct Layout {
    pub cube: Cube,
    /// The tight bounds of the points, as opposed to the cube around them, and the ranges of
    /// their attributes.
    pub extent: Extent,
    pub point_count: u64,
    pub attributes: Attributes,
    /// Distance between the root node's grid cells.
    pub root_spacing: f64,
}

/// A node of the finished octree, for writing the hierarchy at the end.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NodeEntry {
    pub key: NodeKey,
    pub point_count: u64,
    /// Position of the node's bytes, counted from the first byte any node was given.
    pub byte_offset: u64,
    /// Zero for a node without points.
    pub byte_size: u64,
}

/// Turns finished nodes into an output format.
///
/// `encode` runs on many threads at once; `append` receives the results one by one, in the
/// order they belong in the file. That order depends only on the input; the root comes last.
/// Nodes without points, which only connect the hierarchy to the nodes below them, are never
/// encoded or appended: they appear only in the list given to `finish`.
///
/// The chunks a sink receives carry the build's attributes and scan 0.
pub trait NodeSink: Sync {
    fn encode(&self, key: NodeKey, points: &PointChunk) -> Result<Vec<u8>>;
    fn append(&mut self, key: NodeKey, bytes: &[u8]) -> Result<()>;
    /// Completes the output. `nodes` lists every node of the octree, in append order.
    fn finish(self, layout: &Layout, nodes: &[NodeEntry]) -> Result<()>;
}

/// Builds the octree of all points in `chunks` and hands its nodes to the sink made by
/// `make_sink`. Temporary files go into a new folder inside `work_folder`, which is removed
/// again whether the build succeeds or not.
///
/// The chunks must all carry exactly `attributes`. Points without a finite position are left
/// out with a warning. Progress is reported as the work after reading.
pub fn build<S: NodeSink>(
    chunks: &mut dyn Iterator<Item = PointChunk>,
    attributes: Attributes,
    params: &OctreeParams,
    work_folder: &Path,
    progress: &Progress,
    make_sink: impl FnOnce(&Layout) -> Result<S>,
) -> Result<()> {
    params.validate()?;
    let temp = TempFolder::create(work_folder)?;
    let (staged, extent) = stage(chunks, attributes, temp.path(), progress)?;
    let layout = Layout::new(&staged, extent, params);
    let mut output = NodeOutput::new(make_sink(&layout)?);
    if staged.point_count > 0 {
        let report = |fraction: f32| progress.set_after_reading_done(PARTITION_SHARE * fraction);
        let subtrees = partition::split(staged, &layout.cube, params, temp.path(), progress, &report)?;
        build_nodes(subtrees, &layout, params, &mut output, progress)?;
    }
    progress.set_after_reading_done(1.0);
    output.finish(&layout)
}

impl Layout {
    fn new(staged: &PointFile, extent: Extent, params: &OctreeParams) -> Layout {
        let cube = Cube::enclosing(extent.min, extent.max);
        Layout {
            cube,
            extent,
            point_count: staged.point_count,
            attributes: staged.attributes,
            root_spacing: cube.size / f64::from(params.grid_cells),
        }
    }
}

/// Spills all incoming points to a temporary file and measures them on the way.
fn stage(
    chunks: &mut dyn Iterator<Item = PointChunk>,
    attributes: Attributes,
    folder: &Path,
    progress: &Progress,
) -> Result<(PointFile, Extent)> {
    let mut writer = PointFileWriter::create(&folder.join("staged.bin"), attributes)?;
    let mut extent = Extent::empty(attributes);
    for mut chunk in chunks {
        progress.check_cancelled()?;
        drop_invalid_positions(&mut chunk, progress);
        extent.include(&chunk);
        writer.write(&chunk)?;
    }
    let staged = writer.finish()?;
    let extent = extent.finished(staged.point_count);
    Ok((staged, extent))
}

/// Leaves out the points whose position is infinite or not a number: they have no place in the
/// octree.
pub(crate) fn drop_invalid_positions(chunk: &mut PointChunk, progress: &Progress) {
    let is_finite = |position: &[f64; 3]| position.iter().all(|value| value.is_finite());
    if chunk.positions.par_iter().all(is_finite) {
        return;
    }
    let keep: Vec<bool> = chunk.positions.iter().map(is_finite).collect();
    chunk.retain(&keep);
    progress.warn("Points with an invalid position (infinite or not a number) were left out.");
}

/// The sink, and the record of every node handed to it.
struct NodeOutput<S> {
    sink: S,
    entries: Vec<NodeEntry>,
    next_offset: u64,
}

struct EncodedNode {
    key: NodeKey,
    point_count: u64,
    bytes: Vec<u8>,
}

impl<S: NodeSink> NodeOutput<S> {
    fn new(sink: S) -> Self {
        NodeOutput { sink, entries: Vec::new(), next_offset: 0 }
    }

    fn write(&mut self, nodes: Vec<BuiltNode>) -> Result<()> {
        let encoded = self.encode(nodes)?;
        self.append(encoded)
    }

    /// Encodes the nodes on all threads. The results keep the order of `nodes`.
    fn encode(&self, nodes: Vec<BuiltNode>) -> Result<Vec<EncodedNode>> {
        nodes
            .into_par_iter()
            .map(|node| {
                let bytes =
                    if node.points.is_empty() { Vec::new() } else { self.sink.encode(node.key, &node.points)? };
                Ok(EncodedNode { key: node.key, point_count: node.points.len() as u64, bytes })
            })
            .collect()
    }

    fn append(&mut self, nodes: Vec<EncodedNode>) -> Result<()> {
        for node in nodes {
            if node.point_count > 0 {
                self.sink.append(node.key, &node.bytes)?;
            }
            let byte_size = node.bytes.len() as u64;
            self.entries.push(NodeEntry {
                key: node.key,
                point_count: node.point_count,
                byte_offset: self.next_offset,
                byte_size,
            });
            self.next_offset += byte_size;
        }
        Ok(())
    }

    fn finish(self, layout: &Layout) -> Result<()> {
        self.sink.finish(layout, &self.entries)
    }
}

/// A subtree once built: its finished nodes, encoded, and its root, which the levels above may
/// still take points from.
struct BuiltSubtree {
    root: NodeKey,
    nodes: Vec<EncodedNode>,
    open_root: OpenNode,
}

/// Builds every subtree and the levels above them, writing each node as soon as it is final.
///
/// Subtrees are taken in depth-first order, so a node above them can be sampled as soon as the
/// last subtree below it is built, and only the open nodes along the current path are kept.
/// Several subtrees are built at once, as many as fit into the size of one maximal subtree.
fn build_nodes<S: NodeSink>(
    mut subtrees: Vec<SubtreeFile>,
    layout: &Layout,
    params: &OctreeParams,
    output: &mut NodeOutput<S>,
    progress: &Progress,
) -> Result<()> {
    subtrees.sort_by_key(|subtree| depth_first_order(subtree.root));
    let mut subtrees = subtrees.into_iter().peekable();
    let mut path = OpenPath::default();
    let mut points_done = 0;
    while subtrees.peek().is_some() {
        progress.check_cancelled()?;
        let batch = next_batch(&mut subtrees, params.max_subtree_points as u64);
        points_done += batch.iter().map(|subtree| subtree.points.point_count).sum::<u64>();
        let shared_output = &*output;
        let built: Vec<BuiltSubtree> = batch
            .into_par_iter()
            .map(|subtree| load_and_build(subtree, layout, params, shared_output, progress))
            .collect::<Result<_>>()?;
        for subtree in built {
            output.append(subtree.nodes)?;
            path.close_before(Some(subtree.root), &layout.cube, params, output)?;
            path.open(subtree.root, subtree.open_root);
        }
        let built_share = points_done as f32 / layout.point_count as f32;
        progress.set_after_reading_done(PARTITION_SHARE + (1.0 - PARTITION_SHARE) * built_share);
    }
    path.finish(&layout.cube, params, output)
}

/// The next subtrees to build together: as many as fit into `budget` points, at least one.
fn next_batch(subtrees: &mut Peekable<impl Iterator<Item = SubtreeFile>>, budget: u64) -> Vec<SubtreeFile> {
    let mut batch: Vec<SubtreeFile> = subtrees.next().into_iter().collect();
    let mut points = batch.iter().map(|subtree| subtree.points.point_count).sum::<u64>();
    while let Some(subtree) = subtrees.next_if(|subtree| points + subtree.points.point_count <= budget) {
        points += subtree.points.point_count;
        batch.push(subtree);
    }
    batch
}

fn load_and_build<S: NodeSink>(
    subtree: SubtreeFile,
    layout: &Layout,
    params: &OctreeParams,
    output: &NodeOutput<S>,
    progress: &Progress,
) -> Result<BuiltSubtree> {
    let points = subtree.points.read_all()?;
    subtree.points.remove()?;
    let built = index::build_subtree(subtree.root, &layout.cube, points, params, progress)?;
    Ok(BuiltSubtree { root: subtree.root, nodes: output.encode(built.nodes)?, open_root: built.root })
}

/// Orders nodes depth first: a node's whole region comes before the next node beside it.
fn depth_first_order(key: NodeKey) -> u128 {
    let shift = MAX_DEPTH - key.level;
    let cell = [key.x, key.y, key.z].map(|coordinate| u64::from(coordinate) << shift);
    (0..MAX_DEPTH)
        .rev()
        .fold(0, |code, bit| cell.iter().fold(code, |code, coordinate| code << 1 | u128::from((coordinate >> bit) & 1)))
}

/// The open nodes above the subtrees built so far, in depth-first order. Each is the root of a
/// region that is completely built, waiting for its siblings so their parent can be sampled.
#[derive(Default)]
struct OpenPath(Vec<(NodeKey, OpenNode)>);

impl OpenPath {
    fn open(&mut self, key: NodeKey, node: OpenNode) {
        self.0.push((key, node));
    }

    /// Samples every parent whose region lies completely before `next`, the root of the next
    /// subtree in depth-first order, and writes the children that are then final. `None`
    /// samples all parents up to the root.
    fn close_before<S: NodeSink>(
        &mut self,
        next: Option<NodeKey>,
        cube: &Cube,
        params: &OctreeParams,
        output: &mut NodeOutput<S>,
    ) -> Result<()> {
        while let Some(parent) = self.completed_parent(next) {
            let first_child = self.0.iter().rposition(|(key, _)| key.parent() != Some(parent)).map_or(0, |at| at + 1);
            let children = self.0.split_off(first_child);
            let (node, finished) = index::sample_family(parent, cube, children, params);
            output.write(finished)?;
            self.0.push((parent, node));
        }
        Ok(())
    }

    /// The parent of the last open node, if `next` lies outside it, so that all its children
    /// are open.
    fn completed_parent(&self, next: Option<NodeKey>) -> Option<NodeKey> {
        let parent = self.0.last()?.0.parent()?;
        match next {
            Some(next) if parent.contains(next) => None,
            _ => Some(parent),
        }
    }

    /// Samples the remaining parents and writes the root.
    fn finish<S: NodeSink>(mut self, cube: &Cube, params: &OctreeParams, output: &mut NodeOutput<S>) -> Result<()> {
        self.close_before(None, cube, params, output)?;
        let root: Vec<BuiltNode> = self
            .0
            .into_iter()
            .filter(|(_, node)| node.is_needed())
            .map(|(key, node)| BuiltNode { key, points: node.points })
            .collect();
        output.write(root)
    }
}
