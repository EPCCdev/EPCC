//! Splitting the staged cloud into subtrees that fit in memory.
//!
//! A node's points are counted on a dense grid of the nodes a few levels below it, the grid
//! level. Summing eight cells at a time gives the count of every node in between, and the
//! subtrees are the largest nodes whose points fit the limit. Each grid cell then knows its
//! subtree, so finding a point's subtree is one lookup. A grid cell that still holds too many
//! points, as happens when far outliers stretch the cube, is split the same way in turn.
//!
//! The subtrees of one split share a log: each subtree fills a page in memory, and a writer
//! thread appends every full page to the log.

use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};
use std::thread;

use crossbeam_channel::{Receiver, Sender, bounded, unbounded};
use rayon::prelude::*;

use super::OctreeParams;
use super::key::{Cube, NodeKey};
use super::staging::{LogWriter, PointFile, Segment, position_of};
use crate::model::TASK_POINTS;
use crate::{Error, Progress, Result};

/// The most levels below the node being split that its points are counted at: 128³ cells.
const GRID_DEPTH: u8 = 7;
/// The share of a split's progress that counting takes; distributing takes the rest.
const COUNTING_SHARE: f32 = 0.3;
/// Memory for the pages of all subtrees together, unless that makes pages smaller than
/// [`MIN_PAGE_BYTES`].
const PAGE_MEMORY_BYTES: usize = 256 << 20;
const MIN_PAGE_BYTES: usize = 8 << 10;
const MAX_PAGE_BYTES: usize = 4 << 20;
/// Full pages waiting for the log writer before the splitting waits as well.
const PAGES_IN_FLIGHT: usize = 16;
/// Marks the grid cells outside every subtree. They hold no points.
const NO_SUBTREE: u32 = u32::MAX;

/// A subtree and its points.
#[derive(Debug)]
pub(super) struct SubtreeFile {
    pub root: NodeKey,
    pub points: PointFile,
}

/// Splits the staged points into subtrees of at most `params.max_subtree_points` each (only
/// coincident points at `params.max_depth` may exceed it), consuming `staged`. Sorted by root
/// key; within each subtree the points keep their staged order.
///
/// The subtree points go to logs in `folder`. A log is deleted once none of its subtrees is left;
/// a subtree that is split again keeps its log until then.
pub(super) fn split(
    staged: PointFile,
    cube: &Cube,
    params: &OctreeParams,
    folder: &Path,
    progress: &Progress,
    report: &dyn Fn(f32),
) -> Result<Vec<SubtreeFile>> {
    let mut unfinished = staged.point_count;
    let mut pending = vec![SubtreeFile { root: NodeKey::ROOT, points: staged }];
    let mut finished = Vec::new();
    let mut reached = 0.0;
    while let Some(node) = pending.pop() {
        // Each split gets the share of the remaining progress its points have among those not
        // in a final subtree yet.
        let span = (1.0 - reached) * fraction(node.points.point_count, unfinished);
        let split =
            split_node(node, cube, params, folder, PAGE_MEMORY_BYTES, progress, &|done| report(reached + span * done))?;
        reached += span;
        let subtrees = match split {
            Split::Into(subtrees) => subtrees,
            Split::Coincident(node) => {
                unfinished -= node.points.point_count;
                finished.push(node);
                continue;
            }
        };
        for subtree in subtrees {
            if is_too_large(&subtree, params) {
                pending.push(subtree);
            } else {
                unfinished -= subtree.points.point_count;
                finished.push(subtree);
            }
        }
    }
    finished.sort_unstable_by_key(|subtree| subtree.root);
    report(1.0);
    Ok(finished)
}

/// Whether `subtree` must be split again: it holds too many points, which only a grid cell
/// can, and the nodes below it may still be split.
fn is_too_large(subtree: &SubtreeFile, params: &OctreeParams) -> bool {
    subtree.points.point_count > params.max_subtree_points as u64 && subtree.root.level < params.max_depth
}

/// What splitting a node gave.
enum Split {
    /// The node's subtrees, which may still be too large.
    Into(Vec<SubtreeFile>),
    /// The node itself: its points fit into a node at the depth limit, so splitting would only
    /// pass them down a chain of nodes. The index step keeps such points together in one leaf too.
    Coincident(SubtreeFile),
}

/// Splits `node` into subtrees once, without looking at whether they fit, and removes its points.
fn split_node(
    node: SubtreeFile,
    cube: &Cube,
    params: &OctreeParams,
    folder: &Path,
    page_memory: usize,
    progress: &Progress,
    report: &dyn Fn(f32),
) -> Result<Split> {
    let plan = plan(node.root, &node.points, cube, params, progress, &|done| report(COUNTING_SHARE * done))?;
    if plan.span.fits_into(cube.node_size(params.max_depth)) {
        return Ok(Split::Coincident(node));
    }
    let log = folder.join(log_name(node.root));
    let distributing = |done| report(COUNTING_SHARE + (1.0 - COUNTING_SHARE) * done);
    let subtrees = distribute(&node.points, cube, &plan, &log, page_memory, progress, &distributing)?;
    node.points.remove()?;
    Ok(Split::Into(subtrees))
}

/// How the points of one node are split into subtrees. Together the subtrees hold every point;
/// none contains another.
#[derive(Clone, Debug, PartialEq)]
struct Plan {
    grid: Grid,
    /// Where the node's points lie.
    span: Span,
    /// Sorted by root.
    subtrees: Vec<PlannedSubtree>,
    /// For each grid cell, the index of its subtree in `subtrees`, or [`NO_SUBTREE`].
    cell_subtree: Vec<u32>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct PlannedSubtree {
    root: NodeKey,
    point_count: u64,
}

impl Plan {
    /// The index in `subtrees` of the subtree holding `position`, or [`NO_SUBTREE`].
    fn subtree_of(&self, cube: &Cube, position: [f64; 3]) -> u32 {
        self.grid.cell_at(cube, position).map_or(NO_SUBTREE, |cell| self.cell_subtree[cell])
    }
}

/// Counts the points of `root`'s node on a grid and merges grid cells into as few subtrees as
/// possible, each holding at most `params.max_subtree_points`. A grid cell above that limit
/// becomes a subtree of its own.
fn plan(
    root: NodeKey,
    points: &PointFile,
    cube: &Cube,
    params: &OctreeParams,
    progress: &Progress,
    report: &dyn Fn(f32),
) -> Result<Plan> {
    let grid = Grid::below(root, points.point_count, params.max_depth);
    let (cells, span) = count_cells(points, cube, grid, progress, report)?;
    let counts = CountPyramid::build(grid, cells);
    let counted = counts.of(root);
    if counted != points.point_count {
        return Err(Error::invalid(format!("counted {counted} of {} temporary points", points.point_count)));
    }

    let mut subtrees = Vec::new();
    select_subtrees(root, &counts, params.max_subtree_points as u64, &mut subtrees);
    subtrees.sort_unstable_by_key(|subtree| subtree.root);
    let cell_subtree = cell_lookup(grid, &subtrees);
    Ok(Plan { grid, span, subtrees, cell_subtree })
}

/// Counts the points in each grid cell, and measures where they lie.
fn count_cells(
    points: &PointFile,
    cube: &Cube,
    grid: Grid,
    progress: &Progress,
    report: &dyn Fn(f32),
) -> Result<(Vec<u64>, Span)> {
    let cells: Vec<AtomicU64> = (0..grid.cell_count()).map(|_| AtomicU64::new(0)).collect();
    let size = points.record_size();
    let mut span = Span::EMPTY;
    let mut done = 0;
    points.read_records(&mut |records| {
        progress.check_cancelled()?;
        let block_span = records
            .par_chunks(size * TASK_POINTS)
            .map(|task| count_task(task, size, cube, grid, &cells))
            .try_reduce(|| Span::EMPTY, |a, b| Ok(a.union(b)))?;
        span = span.union(block_span);
        done += (records.len() / size) as u64;
        report(fraction(done, points.point_count));
        Ok(())
    })?;
    Ok((cells.into_iter().map(AtomicU64::into_inner).collect(), span))
}

/// Adds one task's records to the grid and returns where they lie. Runs of points in the same
/// cell, the usual case for scans, are added at once.
fn count_task(records: &[u8], size: usize, cube: &Cube, grid: Grid, cells: &[AtomicU64]) -> Result<Span> {
    let positions: Vec<[f64; 3]> = records.chunks_exact(size).map(position_of).collect();
    let task_cells: Vec<Option<usize>> = positions.iter().map(|&position| grid.cell_at(cube, position)).collect();
    for run in task_cells.chunk_by(|a, b| a == b) {
        let cell = run[0].ok_or_else(|| Error::invalid("a temporary point lies outside its subtree"))?;
        cells[cell].fetch_add(run.len() as u64, Ordering::Relaxed);
    }
    Ok(positions.iter().fold(Span::EMPTY, |span, &position| span.including(position)))
}

/// The bounding box of some points.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Span {
    min: [f64; 3],
    max: [f64; 3],
}

impl Span {
    const EMPTY: Span = Span { min: [f64::INFINITY; 3], max: [f64::NEG_INFINITY; 3] };

    fn including(self, position: [f64; 3]) -> Span {
        self.union(Span { min: position, max: position })
    }

    fn union(self, other: Span) -> Span {
        Span {
            min: std::array::from_fn(|axis| self.min[axis].min(other.min[axis])),
            max: std::array::from_fn(|axis| self.max[axis].max(other.max[axis])),
        }
    }

    /// Whether the box holds points and is at most `size` long along every axis.
    fn fits_into(self, size: f64) -> bool {
        (0..3).all(|axis| self.min[axis] <= self.max[axis] && self.max[axis] - self.min[axis] <= size)
    }
}

/// The nodes from `root` down to a few levels below it, numbered level by level.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Grid {
    root: NodeKey,
    /// How many levels the grid cells lie below the root.
    depth: u8,
}

impl Grid {
    /// The grid for splitting `root`, which holds `point_count` points: about one cell per point,
    /// which is fine enough and costs no more than reading the points. It lies at least one level
    /// below the root and at most [`GRID_DEPTH`], but not below `max_depth`, where nodes stop
    /// being split.
    fn below(root: NodeKey, point_count: u64, max_depth: u8) -> Grid {
        let levels_for_one_per_cell = (u64::BITS - point_count.saturating_sub(1).leading_zeros()).div_ceil(3) as u8;
        let depth = levels_for_one_per_cell.clamp(1, GRID_DEPTH).min(max_depth.saturating_sub(root.level));
        Grid { root, depth }
    }

    /// The level of the grid cells.
    fn level(self) -> u8 {
        self.root.level + self.depth
    }

    fn cell_count(self) -> usize {
        nodes_below(self.depth)
    }

    /// The grid cell that holds `position`, or `None` if it lies outside the root.
    fn cell_at(self, cube: &Cube, position: [f64; 3]) -> Option<usize> {
        let cell = cube.key_at(position, self.level());
        self.root.contains(cell).then(|| self.index(cell))
    }

    /// The position of `node`, which lies in the root, among the root's nodes at its level, `x`
    /// varying slowest.
    fn index(self, node: NodeKey) -> usize {
        let depth = u32::from(node.level - self.root.level);
        let local = |value: u32, root: u32| (value - (root << depth)) as usize;
        local(node.x, self.root.x) << (2 * depth) | local(node.y, self.root.y) << depth | local(node.z, self.root.z)
    }

    /// The node at `index` among the root's nodes `depth` levels below it; the inverse of
    /// [`Grid::index`].
    fn node(self, depth: u8, index: usize) -> NodeKey {
        let shift = u32::from(depth);
        let mask = (1 << shift) - 1;
        let global = |root: u32, local: usize| (root << shift) + local as u32;
        NodeKey {
            level: self.root.level + depth,
            x: global(self.root.x, index >> (2 * shift)),
            y: global(self.root.y, (index >> shift) & mask),
            z: global(self.root.z, index & mask),
        }
    }
}

/// How many nodes lie `depth` levels below a node.
fn nodes_below(depth: u8) -> usize {
    1 << (3 * u32::from(depth))
}

/// The point count of every node of a grid, from its root down to its cells.
struct CountPyramid {
    grid: Grid,
    /// The counts of the nodes each number of levels below the root; the last are the cells.
    levels: Vec<Vec<u64>>,
}

impl CountPyramid {
    /// Sums the cell counts up to the root.
    fn build(grid: Grid, cells: Vec<u64>) -> CountPyramid {
        let mut levels = vec![Vec::new(); usize::from(grid.depth)];
        levels.push(cells);
        let mut counts = CountPyramid { grid, levels };
        for depth in (0..grid.depth).rev() {
            let sums = (0..nodes_below(depth))
                .into_par_iter()
                .map(|index| {
                    let node = grid.node(depth, index);
                    (0..8).map(|octant| counts.of(node.child(octant))).sum()
                })
                .collect();
            counts.levels[usize::from(depth)] = sums;
        }
        counts
    }

    /// The number of points in `node`, which must lie in the root and not below the cells.
    fn of(&self, node: NodeKey) -> u64 {
        self.levels[usize::from(node.level - self.grid.root.level)][self.grid.index(node)]
    }
}

/// Adds the subtrees that cover `node` to `subtrees`: the node itself if its points fit `limit`
/// or it is a grid cell, otherwise the subtrees of its children. Empty nodes need none.
fn select_subtrees(node: NodeKey, counts: &CountPyramid, limit: u64, subtrees: &mut Vec<PlannedSubtree>) {
    let point_count = counts.of(node);
    if point_count == 0 {
        return;
    }
    if point_count <= limit || node.level == counts.grid.level() {
        subtrees.push(PlannedSubtree { root: node, point_count });
        return;
    }
    for octant in 0..8 {
        select_subtrees(node.child(octant), counts, limit, subtrees);
    }
}

/// For each grid cell, the index of the subtree that contains it.
fn cell_lookup(grid: Grid, subtrees: &[PlannedSubtree]) -> Vec<u32> {
    let mut lookup = vec![NO_SUBTREE; grid.cell_count()];
    for (subtree, planned) in subtrees.iter().enumerate() {
        let root = planned.root;
        let shift = grid.level() - root.level;
        let span = 1 << shift;
        let [x0, y0, z0] = [root.x, root.y, root.z].map(|value| value << shift);
        for x in x0..x0 + span {
            for y in y0..y0 + span {
                let start = grid.index(NodeKey { level: grid.level(), x, y, z: z0 });
                lookup[start..start + span as usize].fill(subtree as u32);
            }
        }
    }
    lookup
}

fn fraction(done: u64, total: u64) -> f32 {
    if total == 0 { 1.0 } else { done as f32 / total as f32 }
}

/// Moves every point of `source` into its subtree in a new log at `log_path`. The subtrees come
/// back in the order of `plan.subtrees`; within each the points keep their order in `source`.
/// The subtree pages take about `page_memory` bytes together.
fn distribute(
    source: &PointFile,
    cube: &Cube,
    plan: &Plan,
    log_path: &Path,
    page_memory: usize,
    progress: &Progress,
    report: &dyn Fn(f32),
) -> Result<Vec<SubtreeFile>> {
    let subtrees = plan.subtrees.len();
    let page_bytes = page_bytes(subtrees, source.record_size(), page_memory);
    let (full_tx, full_rx) = bounded(PAGES_IN_FLIGHT);
    let (spare_tx, spare_rx) = unbounded();
    let writer = LogWriter::create(log_path)?;
    let (writer, segments) = thread::scope(|scope| -> Result<_> {
        let writing = scope.spawn(move || write_pages(writer, subtrees, full_rx, spare_tx));
        let mut pages = Pages { pages: vec![Vec::new(); subtrees], page_bytes, full: full_tx, spare: spare_rx };
        let split = split_records(source, cube, plan, &mut pages, progress, report).and_then(|()| pages.flush_all());
        drop(pages); // lets the writer finish
        let written = writing.join().unwrap_or_else(|_| Err(Error::invalid("writing the subtree log crashed")));
        // A failed writer makes splitting stop as well, so its error is the cause.
        let written = written?;
        split?;
        Ok(written)
    })?;

    let log = writer.finish()?;
    plan.subtrees
        .iter()
        .zip(segments)
        .map(|(planned, segments)| {
            let points = log.point_file(source.attributes, segments);
            if points.point_count != planned.point_count {
                return Err(Error::invalid(format!(
                    "{:?}: {} points distributed instead of {}",
                    planned.root, points.point_count, planned.point_count
                )));
            }
            Ok(SubtreeFile { root: planned.root, points })
        })
        .collect()
}

/// The size of each subtree's page, a whole number of records: all pages together take about
/// `memory`, within the page size limits.
fn page_bytes(subtrees: usize, record_size: usize, memory: usize) -> usize {
    let bytes = (memory / subtrees.max(1)).clamp(MIN_PAGE_BYTES, MAX_PAGE_BYTES);
    (bytes / record_size).max(1) * record_size
}

/// Reads the records of `source` and hands each one to the page of its subtree.
fn split_records(
    source: &PointFile,
    cube: &Cube,
    plan: &Plan,
    pages: &mut Pages,
    progress: &Progress,
    report: &dyn Fn(f32),
) -> Result<()> {
    let size = source.record_size();
    let mut done = 0;
    source.read_records(&mut |records| {
        progress.check_cancelled()?;
        let owners: Vec<u32> =
            records.par_chunks_exact(size).map(|record| plan.subtree_of(cube, position_of(record))).collect();
        let mut start = 0;
        for run in owners.chunk_by(|a, b| a == b) {
            if run[0] == NO_SUBTREE {
                return Err(Error::invalid("a temporary point lies outside every subtree"));
            }
            let end = start + run.len();
            pages.push(run[0] as usize, &records[start * size..end * size])?;
            start = end;
        }
        done += owners.len() as u64;
        report(fraction(done, source.point_count));
        Ok(())
    })
}

/// A full page on its way to the log, with the index of its subtree.
type FullPage = (usize, Vec<u8>);

/// One page of records per subtree. A full page goes to the log writer, which sends it back
/// empty for reuse.
struct Pages {
    /// Without capacity until the subtree's first records arrive.
    pages: Vec<Vec<u8>>,
    page_bytes: usize,
    full: Sender<FullPage>,
    spare: Receiver<Vec<u8>>,
}

impl Pages {
    /// Adds `records` to the page of `subtree`, sending the page on whenever it fills up.
    fn push(&mut self, subtree: usize, mut records: &[u8]) -> Result<()> {
        while !records.is_empty() {
            if self.pages[subtree].capacity() == 0 {
                self.pages[subtree] = self.spare.try_recv().unwrap_or_else(|_| Vec::with_capacity(self.page_bytes));
            }
            let page = &mut self.pages[subtree];
            let (now, later) = records.split_at((self.page_bytes - page.len()).min(records.len()));
            page.extend_from_slice(now);
            records = later;
            if page.len() == self.page_bytes {
                let page = std::mem::take(page);
                self.send(subtree, page)?;
            }
        }
        Ok(())
    }

    /// Sends every page that holds records.
    fn flush_all(&mut self) -> Result<()> {
        for subtree in 0..self.pages.len() {
            if !self.pages[subtree].is_empty() {
                let page = std::mem::take(&mut self.pages[subtree]);
                self.send(subtree, page)?;
            }
        }
        Ok(())
    }

    fn send(&self, subtree: usize, page: Vec<u8>) -> Result<()> {
        self.full.send((subtree, page)).map_err(|_| Error::OutputStopped)
    }
}

/// Appends the pages to the log in the order they arrive and sends them back empty. Returns
/// the log and the segments of each subtree.
fn write_pages(
    mut log: LogWriter,
    subtrees: usize,
    full: Receiver<FullPage>,
    spare: Sender<Vec<u8>>,
) -> Result<(LogWriter, Vec<Vec<Segment>>)> {
    let mut segments = vec![Vec::new(); subtrees];
    for (subtree, mut page) in full {
        log.append(&page, &mut segments[subtree])?;
        page.clear();
        // Once splitting is done, nobody takes the page back; it is freed instead.
        let _ = spare.send(page);
    }
    Ok((log, segments))
}

fn log_name(root: NodeKey) -> String {
    format!("{}-{}-{}-{}.log", root.level, root.x, root.y, root.z)
}

#[cfg(test)]
mod tests {
    use std::cell::RefCell;
    use std::fs;

    use super::*;
    use crate::model::{Attributes, PointChunk};
    use crate::octree::staging::{PointFileWriter, TempFolder};

    /// Stages `positions` in two chunks. Each point's time is its index, so its original place
    /// can be told after it moved.
    fn stage(folder: &TempFolder, positions: &[[f64; 3]]) -> PointFile {
        let attributes = Attributes { time: true, ..Attributes::NONE };
        let mut writer = PointFileWriter::create(&folder.path().join("staged.points"), attributes).unwrap();
        let middle = positions.len() / 3;
        for range in [0..middle, middle..positions.len()] {
            let mut chunk = PointChunk::new(0, attributes);
            chunk.positions = positions[range.clone()].to_vec();
            chunk.time = Some(range.map(|index| index as f64).collect());
            writer.write(&chunk).unwrap();
        }
        writer.finish().unwrap()
    }

    fn random() -> impl FnMut() -> f64 {
        let mut state = 0x9e37_79b9_7f4a_7c15u64;
        move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 11) as f64 / (1u64 << 53) as f64
        }
    }

    /// A deterministic cloud in a 100 m cube: uniform points and a dense blob.
    fn uneven_cloud() -> Vec<[f64; 3]> {
        let mut next = random();
        let mut points: Vec<[f64; 3]> = (0..60_000).map(|_| [next() * 100.0, next() * 100.0, next() * 100.0]).collect();
        points.extend((0..30_000).map(|_| [20.0 + next(), 70.0 + next(), 5.0 + next()]));
        points.extend([[0.0; 3], [100.0; 3], [50.0, 50.0, 50.0]]); // corners and centre borders
        points
    }

    /// [`uneven_cloud`] and many copies of a single position, which no amount of splitting can
    /// separate.
    fn test_cloud() -> Vec<[f64; 3]> {
        let mut points = uneven_cloud();
        points.extend((0..5_000).map(|_| [42.0, 42.0, 42.0]));
        points
    }

    fn test_cube() -> Cube {
        Cube::enclosing([0.0; 3], [100.0; 3])
    }

    fn params(max_subtree_points: usize) -> OctreeParams {
        OctreeParams { max_subtree_points, ..OctreeParams::default() }
    }

    fn make_plan(staged: &PointFile, limit: usize) -> Plan {
        plan(NodeKey::ROOT, staged, &test_cube(), &params(limit), &Progress::new(), &|_| {}).unwrap()
    }

    fn records(file: &PointFile) -> Vec<u8> {
        let mut bytes = Vec::new();
        file.read_records(&mut |records| {
            bytes.extend_from_slice(records);
            Ok(())
        })
        .unwrap();
        bytes
    }

    /// Checks that `subtrees` hold exactly the points of `cloud`, each in its subtree's node and
    /// in staged order.
    fn check_subtrees(subtrees: &[SubtreeFile], cloud: &[[f64; 3]], cube: &Cube) {
        let mut seen = vec![false; cloud.len()];
        for subtree in subtrees {
            let points = subtree.points.read_all().unwrap();
            assert_eq!(points.len() as u64, subtree.points.point_count);
            let times = points.time.as_ref().unwrap();
            assert!(times.is_sorted_by(|a, b| a < b), "staged order is kept");
            for (&position, &time) in points.positions.iter().zip(times) {
                let index = time as usize;
                assert_eq!(cloud[index], position);
                assert!(!std::mem::replace(&mut seen[index], true), "point {index} is in two subtrees");
                assert_eq!(cube.key_at(position, subtree.root.level), subtree.root);
            }
        }
        assert!(seen.iter().all(|&seen| seen));
    }

    #[test]
    fn small_clouds_are_one_subtree() {
        let folder = TempFolder::create(&std::env::temp_dir()).unwrap();
        let staged = stage(&folder, &test_cloud());
        let plan = make_plan(&staged, 1_000_000);
        assert_eq!(plan.subtrees, [PlannedSubtree { root: NodeKey::ROOT, point_count: staged.point_count }]);
    }

    #[test]
    fn subtrees_cover_the_cloud_without_overlap() {
        let folder = TempFolder::create(&std::env::temp_dir()).unwrap();
        let cloud = test_cloud();
        let staged = stage(&folder, &cloud);
        let limit = 2_000;
        let plan = make_plan(&staged, limit);
        let grid_level = plan.grid.level();

        assert!(plan.subtrees.len() > 8);
        assert!(plan.subtrees.is_sorted_by_key(|subtree| subtree.root));
        assert_eq!(plan.subtrees.iter().map(|subtree| subtree.point_count).sum::<u64>(), staged.point_count);
        for (index, &PlannedSubtree { root, point_count }) in plan.subtrees.iter().enumerate() {
            assert!(point_count > 0);
            assert!(point_count <= limit as u64 || root.level == grid_level, "{root:?} holds {point_count}");
            let inside = cloud.iter().filter(|&&p| root.contains(test_cube().key_at(p, grid_level))).count();
            assert_eq!(inside as u64, point_count);
            for other in &plan.subtrees[index + 1..] {
                assert!(!root.contains(other.root) && !other.root.contains(root), "{root:?} and {other:?} overlap");
            }
        }
        let duplicates = test_cube().key_at([42.0; 3], grid_level);
        assert!(plan.subtrees.iter().any(|subtree| subtree.root == duplicates), "the crowded cell is a subtree");
    }

    #[test]
    fn empty_clouds_have_no_subtrees() {
        let folder = TempFolder::create(&std::env::temp_dir()).unwrap();
        let staged = stage(&folder, &[]);
        assert!(make_plan(&staged, 10).subtrees.is_empty());
        let split = split(staged, &test_cube(), &params(10), folder.path(), &Progress::new(), &|_| {}).unwrap();
        assert!(split.is_empty());
    }

    #[test]
    fn cell_indices_round_trip() {
        let root = NodeKey { level: 3, x: 5, y: 0, z: 7 };
        let grid = Grid::below(root, 1 << 30, 20);
        assert_eq!(grid.level(), 3 + GRID_DEPTH);
        for index in [0, 1, 127, 128, 12_345, grid.cell_count() - 1] {
            let node = grid.node(GRID_DEPTH, index);
            assert!(grid.root.contains(node));
            assert_eq!(grid.index(node), index);
        }
        assert_eq!(Grid::below(root, 1 << 30, 5).level(), 5, "not below the depth limit");
        assert_eq!(Grid::below(root, 4_096, 20).depth, 4, "about one cell per point");
        assert_eq!(Grid::below(root, 4_097, 20).depth, 5);
        assert_eq!(Grid::below(root, 1, 20).depth, 1, "at least one level below the root");
    }

    #[test]
    fn log_reading_returns_each_subtrees_points_in_staged_order() {
        let folder = TempFolder::create(&std::env::temp_dir()).unwrap();
        let cloud = test_cloud();
        let staged = stage(&folder, &cloud);
        let plan = make_plan(&staged, 2_000);
        let log = folder.path().join("subtrees.log");
        let reported = std::cell::Cell::new(0.0);
        // The smallest pages, so the subtrees' segments interleave in the log.
        let subtrees =
            distribute(&staged, &test_cube(), &plan, &log, 0, &Progress::new(), &|f| reported.set(f)).unwrap();
        assert_eq!(reported.get(), 1.0);

        assert_eq!(
            subtrees.iter().map(|subtree| subtree.root).collect::<Vec<_>>(),
            plan.subtrees.iter().map(|s| s.root).collect::<Vec<_>>()
        );
        check_subtrees(&subtrees, &cloud, &test_cube());
        drop(subtrees);
        assert!(!log.exists(), "the log goes with its last subtree");
    }

    #[test]
    fn distribution_is_deterministic_whatever_the_page_size() {
        let folder = TempFolder::create(&std::env::temp_dir()).unwrap();
        let staged = stage(&folder, &test_cloud());
        let plan = make_plan(&staged, 2_000);
        let run = |page_memory| {
            let log = folder.path().join(format!("{page_memory}.log"));
            let subtrees =
                distribute(&staged, &test_cube(), &plan, &log, page_memory, &Progress::new(), &|_| {}).unwrap();
            subtrees.iter().map(|subtree| records(&subtree.points)).collect::<Vec<_>>()
        };
        let one_page_each = run(PAGE_MEMORY_BYTES);
        assert_eq!(run(PAGE_MEMORY_BYTES), one_page_each);
        assert_eq!(run(0), one_page_each);
        assert_eq!(run(1 << 20), one_page_each);
        assert_eq!(one_page_each.iter().map(Vec::len).sum::<usize>(), test_cloud().len() * staged.record_size());
    }

    #[test]
    fn a_far_outlier_does_not_make_an_oversized_subtree() {
        let mut cloud: Vec<[f64; 3]> = uneven_cloud().iter().map(|p| p.map(|value| value + 500_000.0)).collect();
        cloud.push([0.0; 3]);
        let cube = Cube::enclosing([0.0; 3], [500_100.0; 3]);
        let limit = 5_000;
        let run = || {
            let folder = TempFolder::create(&std::env::temp_dir()).unwrap();
            let staged = stage(&folder, &cloud);
            let reported = RefCell::new(Vec::new());
            let subtrees = split(staged, &cube, &params(limit), folder.path(), &Progress::new(), &|f| {
                reported.borrow_mut().push(f)
            })
            .unwrap();
            let reported = reported.into_inner();
            assert!(reported.is_sorted(), "progress only grows");
            assert_eq!(reported.last(), Some(&1.0));

            assert!(subtrees.is_sorted_by_key(|subtree| subtree.root));
            assert!(subtrees.iter().all(|subtree| subtree.points.point_count <= limit as u64));
            assert!(subtrees.iter().any(|subtree| subtree.root.level > GRID_DEPTH), "some were split again");
            check_subtrees(&subtrees, &cloud, &cube);
            subtrees.iter().map(|subtree| (subtree.root, records(&subtree.points))).collect::<Vec<_>>()
        };
        assert_eq!(run(), run(), "splitting again is deterministic");
    }

    #[test]
    fn only_coincident_points_at_the_depth_limit_exceed_the_limit() {
        let folder = TempFolder::create(&std::env::temp_dir()).unwrap();
        let cloud = test_cloud();
        let staged = stage(&folder, &cloud);
        let params = OctreeParams { max_depth: 10, ..params(1_000) };
        let subtrees = split(staged, &test_cube(), &params, folder.path(), &Progress::new(), &|_| {}).unwrap();
        let oversized: Vec<NodeKey> =
            subtrees.iter().filter(|subtree| subtree.points.point_count > 1_000).map(|subtree| subtree.root).collect();
        assert_eq!(oversized, [test_cube().key_at([42.0; 3], 10)]);
        check_subtrees(&subtrees, &cloud, &test_cube());
    }

    #[test]
    fn many_subtrees_share_one_log() {
        let folder = TempFolder::create(&std::env::temp_dir()).unwrap();
        let mut next = random();
        let cloud: Vec<[f64; 3]> = (0..300_000).map(|_| [next() * 100.0, next() * 100.0, next() * 100.0]).collect();
        let staged = stage(&folder, &cloud);
        let started = std::time::Instant::now();
        let subtrees = split(staged, &test_cube(), &params(100), folder.path(), &Progress::new(), &|_| {}).unwrap();
        println!("{} subtrees in {:.2?}", subtrees.len(), started.elapsed());

        assert!(subtrees.len() > 4_000, "{} subtrees", subtrees.len());
        let files: Vec<_> = fs::read_dir(folder.path()).unwrap().map(|entry| entry.unwrap().file_name()).collect();
        assert_eq!(files, ["0-0-0-0.log"], "the staged file is gone and the subtrees share one log");
        check_subtrees(&subtrees, &cloud, &test_cube());
    }

    #[test]
    fn cancelling_stops_both_steps() {
        let folder = TempFolder::create(&std::env::temp_dir()).unwrap();
        let staged = stage(&folder, &test_cloud());
        let plan = make_plan(&staged, 2_000);
        let progress = Progress::new();
        progress.cancel();
        let planned = super::plan(NodeKey::ROOT, &staged, &test_cube(), &params(2_000), &progress, &|_| {});
        assert!(matches!(planned, Err(Error::Cancelled)));
        let log = folder.path().join("subtrees.log");
        let distributed = distribute(&staged, &test_cube(), &plan, &log, PAGE_MEMORY_BYTES, &progress, &|_| {});
        assert!(matches!(distributed, Err(Error::Cancelled)));
        let split = split(staged, &test_cube(), &params(2_000), folder.path(), &progress, &|_| {});
        assert!(matches!(split, Err(Error::Cancelled)));
    }
}
