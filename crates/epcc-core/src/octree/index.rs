//! Building octree nodes in memory: splitting crowded nodes and sampling inner nodes.

use rayon::prelude::*;

use super::OctreeParams;
use super::key::{Cube, NodeKey};
use crate::model::PointChunk;
use crate::{Progress, Result};

/// Nodes with at least this many points build their children in parallel. Below it, spreading
/// the work over threads costs more than it gains.
const PARALLEL_POINTS: usize = 1 << 16;

/// A node whose points are final.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct BuiltNode {
    pub key: NodeKey,
    pub points: PointChunk,
}

/// A node whose parent may still take points from it.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct OpenNode {
    pub points: PointChunk,
    /// Whether any node below this one was finished.
    pub has_descendants: bool,
}

impl OpenNode {
    fn leaf(points: PointChunk) -> OpenNode {
        OpenNode { points, has_descendants: false }
    }

    /// A node without points is only written when the hierarchy needs it to reach the nodes
    /// below.
    pub fn is_needed(&self) -> bool {
        !self.points.is_empty() || self.has_descendants
    }
}

/// A subtree built from all of its points.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct Subtree {
    /// Every finished node below the root, sorted by key.
    pub nodes: Vec<BuiltNode>,
    /// The root. Not final yet: the levels above may still take some of its points.
    pub root: OpenNode,
}

impl Subtree {
    fn leaf(points: PointChunk) -> Subtree {
        Subtree { nodes: Vec::new(), root: OpenNode::leaf(points) }
    }
}

/// Builds the subtree rooted at `root` from `points`, which all lie in `root`'s node of `cube`.
///
/// A node with more than `params.max_leaf_points` points is split into its eight children,
/// unless it is at `params.max_depth` or its points are a cluster that would all go to the same
/// child and fit into a node at `params.max_depth`. After its children are built, an inner
/// node takes its sample from them with [`sample_family`].
pub(super) fn build_subtree(
    root: NodeKey,
    cube: &Cube,
    points: PointChunk,
    params: &OctreeParams,
    progress: &Progress,
) -> Result<Subtree> {
    let builder = Builder { cube, params, progress };
    let mut subtree = builder.build(root, points)?;
    subtree.nodes.sort_unstable_by_key(|node| node.key);
    Ok(subtree)
}

/// Gives `parent` its sample from its open `children`, which are all direct children of
/// `parent`, sorted by key. Returns the parent as an open node and the children, which are
/// finished now, sorted by key. Children left without points and without descendants are
/// dropped, so the parent has descendants exactly when any child is returned.
pub(super) fn sample_family(
    parent: NodeKey,
    cube: &Cube,
    mut children: Vec<(NodeKey, OpenNode)>,
    params: &OctreeParams,
) -> (OpenNode, Vec<BuiltNode>) {
    let points = sample(parent, cube, &mut children, params);
    let finished: Vec<BuiltNode> = children
        .into_iter()
        .filter(|(_, node)| node.is_needed())
        .map(|(key, node)| BuiltNode { key, points: node.points })
        .collect();
    (OpenNode { points, has_descendants: !finished.is_empty() }, finished)
}

/// Moves `parent`'s sample out of the points of `children`, its direct children sorted by key.
///
/// The parent's node is divided into a grid of `params.grid_cells` cells along each edge, and
/// each occupied cell contributes the point nearest its centre. Ties go to the earlier child,
/// then to the earlier point, so the result depends only on the input.
fn sample(parent: NodeKey, cube: &Cube, children: &mut [(NodeKey, OpenNode)], params: &OctreeParams) -> PointChunk {
    debug_assert!(children.iter().all(|(child, _)| child.parent() == Some(parent)));
    debug_assert!(children.is_sorted_by_key(|(child, _)| *child));
    let grid = Grid::new(&cube.of(parent), params.grid_cells);
    let candidates = grid.ranked_candidates(children);

    let mut selected = vec![false; candidates.len()];
    for cell in candidates.chunk_by(|a, b| a.cell == b.cell) {
        selected[cell[0].point] = true;
    }
    take_selected(children, &selected)
}

/// What all nodes of one subtree build share.
struct Builder<'a> {
    cube: &'a Cube,
    params: &'a OctreeParams,
    progress: &'a Progress,
}

impl Builder<'_> {
    /// Builds node `key` from `points` and everything below it. The nodes are not sorted yet.
    fn build(&self, key: NodeKey, points: PointChunk) -> Result<Subtree> {
        self.progress.check_cancelled()?;
        if points.len() <= self.params.max_leaf_points || key.level >= self.params.max_depth {
            return Ok(Subtree::leaf(points));
        }
        let octants = self.octants(key, &points);
        if self.is_cluster(&points, &octants) {
            return Ok(Subtree::leaf(points));
        }

        // A minimum length of all children keeps small nodes on the current thread.
        let min_children_per_thread = if points.len() >= PARALLEL_POINTS { 1 } else { usize::MAX };
        let built: Vec<(NodeKey, Subtree)> = split(key, points, &octants)
            .into_par_iter()
            .with_min_len(min_children_per_thread)
            .map(|(child, points)| Ok((child, self.build(child, points)?)))
            .collect::<Result<_>>()?;

        let mut nodes = Vec::new();
        let mut children = Vec::with_capacity(built.len());
        for (child, subtree) in built {
            nodes.extend(subtree.nodes);
            children.push((child, subtree.root));
        }
        let (root, finished) = sample_family(key, self.cube, children, self.params);
        nodes.extend(finished);
        Ok(Subtree { nodes, root })
    }

    /// The octant of the child of `key`'s node each point falls into.
    fn octants(&self, key: NodeKey, points: &PointChunk) -> Vec<u8> {
        let level = key.level + 1;
        points
            .positions
            .par_iter()
            .map(|&position| {
                let child = self.cube.key_at(position, level);
                debug_assert_eq!(child.parent(), Some(key), "point outside its node");
                child.octant()
            })
            .collect()
    }

    /// Whether splitting `key`'s node would put all its points into one child although they fit
    /// into a node at the depth limit. Such points are (nearly) coincident: no node above that
    /// limit would separate them, so splitting would only make a chain of nodes that each pass one
    /// point up, and they stay together instead.
    fn is_cluster(&self, points: &PointChunk, octants: &[u8]) -> bool {
        let one_child = octants.iter().all(|&octant| octant == octants[0]);
        one_child && fits_into(&points.positions, self.cube.node_size(self.params.max_depth))
    }
}

/// Distributes the points of `key`'s node among its children by their `octants`. Returns the
/// children that got points, sorted by key.
fn split(key: NodeKey, points: PointChunk, octants: &[u8]) -> Vec<(NodeKey, PointChunk)> {
    (0..8)
        .zip(points.split_by(octants, 8))
        .filter(|(_, points)| !points.is_empty())
        .map(|(octant, points)| (key.child(octant), points))
        .collect()
}

/// Whether the bounding box of `positions` is at most `size` long along every axis.
fn fits_into(positions: &[[f64; 3]], size: f64) -> bool {
    let Some(&first) = positions.first() else {
        return true;
    };
    let (mut min, mut max) = (first, first);
    positions.iter().all(|position| {
        for axis in 0..3 {
            min[axis] = min[axis].min(position[axis]);
            max[axis] = max[axis].max(position[axis]);
        }
        (0..3).all(|axis| max[axis] - min[axis] <= size)
    })
}

/// Moves the selected points out of the children into one chunk, ordered by child and then by
/// position within the child. `selected` covers the children's points one after another.
fn take_selected(children: &mut [(NodeKey, OpenNode)], selected: &[bool]) -> PointChunk {
    let Some((_, first)) = children.first() else {
        return PointChunk::default();
    };
    let count = selected.iter().filter(|&&taken| taken).count();
    let mut sample = PointChunk::with_capacity(first.points.scan, first.points.attributes(), count);
    let mut start = 0;
    for (_, node) in children {
        let end = start + node.points.len();
        sample.append(node.points.take(&selected[start..end]));
        start = end;
    }
    sample
}

/// The grid an inner node takes its sample on.
struct Grid {
    min: [f64; 3],
    cells_per_unit: f64,
    cells: u32,
}

/// A point competing for its grid cell. Sorted, each cell's candidates come together and the
/// winner first: the one nearest the centre, then the earliest.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct Candidate {
    cell: u64,
    /// Squared distance from the cell's centre, in cells, as bits. Non-negative floating point
    /// numbers order like their bits.
    distance: u64,
    /// Index into the children's points taken one after another, so a smaller index is an
    /// earlier child or an earlier point of the same child.
    point: usize,
}

impl Grid {
    fn new(node: &Cube, cells: u32) -> Grid {
        let cells = cells.max(1);
        Grid { min: node.min, cells_per_unit: f64::from(cells) / node.size, cells }
    }

    /// Where the point with index `point` and `position` falls on the grid. Positions outside
    /// the node count to the nearest cell.
    fn candidate(&self, position: [f64; 3], point: usize) -> Candidate {
        let last = f64::from(self.cells - 1);
        let mut cell = 0;
        let mut distance = 0.0;
        for (value, min) in position.into_iter().zip(self.min) {
            let scaled = (value - min) * self.cells_per_unit;
            let index = scaled.floor().clamp(0.0, last);
            cell = cell * u64::from(self.cells) + index as u64;
            distance += (scaled - index - 0.5).powi(2);
        }
        Candidate { cell, distance: f64::to_bits(distance), point }
    }

    /// The candidates of all the children's points, sorted.
    fn ranked_candidates(&self, children: &[(NodeKey, OpenNode)]) -> Vec<Candidate> {
        let total = children.iter().map(|(_, node)| node.points.len()).sum();
        let mut candidates = Vec::with_capacity(total);
        let mut start = 0;
        for (_, node) in children {
            let located = node.points.positions.par_iter().enumerate();
            candidates.par_extend(located.map(|(index, &position)| self.candidate(position, start + index)));
            start += node.points.len();
        }
        candidates.par_sort_unstable();
        candidates
    }
}

#[cfg(test)]
mod tests {
    use std::collections::HashSet;

    use super::*;
    use crate::Error;
    use crate::model::{Attributes, PointRecord};

    /// Every test point carries a colour that identifies it, to check that attributes travel
    /// with their positions.
    const ATTRIBUTES: Attributes = Attributes { color: true, ..Attributes::NONE };

    const CUBE: Cube = Cube { min: [0.0; 3], size: 10.0 };

    /// A deterministic pseudo-random number source (xorshift).
    struct Random(u64);

    impl Random {
        /// Uniform in 0..1.
        fn unit(&mut self) -> f64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            (self.0 >> 11) as f64 / (1u64 << 53) as f64
        }
    }

    fn identity_color(index: usize) -> [u16; 3] {
        [index as u16, (index >> 16) as u16, (index >> 32) as u16]
    }

    fn chunk(positions: impl IntoIterator<Item = [f64; 3]>) -> PointChunk {
        let mut chunk = PointChunk::new(3, ATTRIBUTES);
        for (index, position) in positions.into_iter().enumerate() {
            chunk.push(&PointRecord { position, color: identity_color(index), ..PointRecord::default() });
        }
        chunk
    }

    /// `count` pseudo-random points spread over [`CUBE`].
    fn random_positions(count: usize, seed: u64) -> Vec<[f64; 3]> {
        let mut random = Random(seed);
        (0..count).map(|_| std::array::from_fn(|_| CUBE.size * random.unit())).collect()
    }

    fn random_points(count: usize, seed: u64) -> PointChunk {
        chunk(random_positions(count, seed))
    }

    fn params(max_leaf_points: usize, grid_cells: u32) -> OctreeParams {
        OctreeParams { grid_cells, max_leaf_points, ..OctreeParams::default() }
    }

    fn build(points: PointChunk, params: &OctreeParams) -> Subtree {
        build_subtree(NodeKey::ROOT, &CUBE, points, params, &Progress::new()).unwrap()
    }

    /// The points of all chunks as (position bits, colour), sorted.
    fn sorted_points<'a>(chunks: impl IntoIterator<Item = &'a PointChunk>) -> Vec<([u64; 3], [u16; 3])> {
        let mut points: Vec<_> = chunks
            .into_iter()
            .flat_map(|chunk| {
                let colors = chunk.color.as_ref().unwrap();
                chunk.positions.iter().zip(colors).map(|(position, &color)| (position.map(f64::to_bits), color))
            })
            .collect();
        points.sort_unstable();
        points
    }

    fn colors(chunk: &PointChunk) -> Vec<u16> {
        chunk.color.as_ref().unwrap().iter().map(|color| color[0]).collect()
    }

    fn has_one_point_per_cell(key: NodeKey, points: &PointChunk, params: &OctreeParams) -> bool {
        let grid = Grid::new(&CUBE.of(key), params.grid_cells);
        let cells: HashSet<u64> = points.positions.iter().map(|&position| grid.candidate(position, 0).cell).collect();
        cells.len() == points.len()
    }

    fn fits_into_the_finest_node(points: &PointChunk, params: &OctreeParams) -> bool {
        fits_into(&points.positions, CUBE.node_size(params.max_depth))
    }

    /// Checks everything a built subtree of [`NodeKey::ROOT`] promises.
    fn check(subtree: &Subtree, input: &PointChunk, params: &OctreeParams) {
        let all = subtree.nodes.iter().map(|node| &node.points).chain([&subtree.root.points]);
        assert_eq!(sorted_points(all), sorted_points([input]), "points lost, duplicated or mixed up");
        assert!(subtree.nodes.is_sorted_by(|a, b| a.key < b.key), "nodes not sorted or not unique");
        assert_eq!(subtree.root.has_descendants, !subtree.nodes.is_empty());

        let keys: HashSet<NodeKey> = subtree.nodes.iter().map(|node| node.key).chain([NodeKey::ROOT]).collect();
        let parents: HashSet<NodeKey> = subtree.nodes.iter().filter_map(|node| node.key.parent()).collect();
        let nodes = subtree.nodes.iter().map(|node| (node.key, &node.points));
        for (key, points) in nodes.chain([(NodeKey::ROOT, &subtree.root.points)]) {
            assert!(key.parent().is_none_or(|parent| keys.contains(&parent)), "{key:?} has no parent");
            assert!(points.positions.iter().all(|&position| CUBE.key_at(position, key.level) == key));
            assert!(key == NodeKey::ROOT || !points.is_empty() || parents.contains(&key), "{key:?} empty leaf");
            if parents.contains(&key) {
                assert!(has_one_point_per_cell(key, points, params), "{key:?} has two points in a cell");
            } else if points.len() > params.max_leaf_points && key.level < params.max_depth {
                let sampled_or_cluster =
                    has_one_point_per_cell(key, points, params) || fits_into_the_finest_node(points, params);
                assert!(sampled_or_cluster, "{key:?} is too large for a leaf");
            }
        }
    }

    #[test]
    fn every_point_lands_in_exactly_one_node() {
        let points = random_points(50_000, 1);
        let params = params(1_000, 16);
        let subtree = build(points.clone(), &params);
        check(&subtree, &points, &params);
        assert!(subtree.nodes.len() > 8);
    }

    #[test]
    fn small_nodes_give_several_levels() {
        let points = random_points(50_000, 2);
        let params = params(20, 4);
        let subtree = build(points.clone(), &params);
        check(&subtree, &points, &params);
        let deepest = subtree.nodes.iter().map(|node| node.key.level).max().unwrap();
        assert!(deepest >= 4, "only {deepest} levels");
    }

    #[test]
    fn default_params_split_a_large_node() {
        let points = random_points(300_000, 3);
        let params = OctreeParams::default();
        let subtree = build(points.clone(), &params);
        check(&subtree, &points, &params);
        assert!(!subtree.nodes.is_empty());
    }

    #[test]
    fn subtrees_below_the_root_work() {
        let key = NodeKey { level: 2, x: 1, y: 3, z: 0 };
        let node = CUBE.of(key);
        let mut random = Random(4);
        let points = chunk((0..10_000).map(|_| std::array::from_fn(|axis| node.min[axis] + node.size * random.unit())));
        let params = params(200, 8);
        let subtree = build_subtree(key, &CUBE, points.clone(), &params, &Progress::new()).unwrap();
        let all = subtree.nodes.iter().map(|node| &node.points).chain([&subtree.root.points]);
        assert_eq!(sorted_points(all), sorted_points([&points]));
        assert!(subtree.root.has_descendants);
        assert!(subtree.nodes.iter().all(|node| key.contains(node.key) && node.key != key));
        for node in &subtree.nodes {
            assert!(node.points.positions.iter().all(|&position| CUBE.key_at(position, node.key.level) == node.key));
        }
    }

    #[test]
    fn builds_do_not_depend_on_threads() {
        let points = random_points(100_000, 5);
        let params = params(2_000, 16);
        let parallel = build(points.clone(), &params);
        let single_thread = rayon::ThreadPoolBuilder::new().num_threads(1).build().unwrap();
        let sequential = single_thread.install(|| build(points.clone(), &params));
        assert_eq!(parallel, sequential);
        assert_eq!(parallel, build(points, &params));
    }

    #[test]
    fn identical_points_stay_in_one_leaf() {
        let count = 200_000;
        let points = chunk(std::iter::repeat_n([3.3, 4.4, 5.5], count));
        let params = params(1_000, 128);
        let subtree = build(points.clone(), &params);
        check(&subtree, &points, &params);
        assert!(subtree.nodes.is_empty(), "{} nodes below the root", subtree.nodes.len());
        assert_eq!(subtree.root, OpenNode { points, has_descendants: false });
    }

    /// Points around (3.3, 4.4, 5.5) up to `spread` apart, among others spread over the cube.
    fn cluster_among_others(spread: f64) -> PointChunk {
        let mut random = Random(8);
        let cluster: Vec<[f64; 3]> =
            (0..20_000).map(|_| std::array::from_fn(|axis| [3.3, 4.4, 5.5][axis] + spread * random.unit())).collect();
        chunk(cluster.into_iter().chain(random_positions(20_000, 9)))
    }

    #[test]
    fn coincident_points_stop_splitting() {
        // 0.1 nm apart, closer than a node at the depth limit (10 m / 2^31, about 5 nm).
        let points = cluster_among_others(1e-10);
        let params = params(1_000, 16);
        let subtree = build(points.clone(), &params);
        check(&subtree, &points, &params);

        // Without the cluster rule the cluster would be split down to the maximum depth.
        let deepest = subtree.nodes.iter().map(|node| node.key.level).max().unwrap();
        assert!(deepest <= 10, "{deepest} levels");
        let largest = subtree.nodes.iter().map(|node| node.points.len()).max().unwrap();
        assert!(largest > 19_900, "the cluster was split up: {largest} points in the largest node");
    }

    #[test]
    fn a_tiny_cluster_is_split_like_any_other() {
        // 1 µm across: within one sampling cell of every node down to level 20, but separable.
        let points = cluster_among_others(1e-6);
        let params = params(1_000, 16);
        let subtree = build(points.clone(), &params);
        check(&subtree, &points, &params);
        let parents: HashSet<NodeKey> = subtree.nodes.iter().filter_map(|node| node.key.parent()).collect();
        let leaves = subtree.nodes.iter().filter(|node| !parents.contains(&node.key));
        assert!(leaves.into_iter().all(|leaf| leaf.points.len() <= params.max_leaf_points));
    }

    #[test]
    fn cancelling_stops_the_build() {
        let progress = Progress::new();
        progress.cancel();
        let result = build_subtree(NodeKey::ROOT, &CUBE, random_points(1_000, 6), &params(100, 4), &progress);
        assert!(matches!(result, Err(Error::Cancelled)));
    }

    const UNIT: Cube = Cube { min: [0.0; 3], size: 1.0 };

    fn open(points: PointChunk, has_descendants: bool) -> OpenNode {
        OpenNode { points, has_descendants }
    }

    /// The first and the last child of the root of [`UNIT`], with the given points.
    fn corner_children(first: &[[f64; 3]], last: &[[f64; 3]]) -> Vec<(NodeKey, OpenNode)> {
        let mut last_points = chunk(last.iter().copied());
        last_points.color.as_mut().unwrap().iter_mut().for_each(|color| color[0] += 100);
        vec![
            (NodeKey::ROOT.child(0), open(chunk(first.iter().copied()), false)),
            (NodeKey::ROOT.child(7), open(last_points, false)),
        ]
    }

    #[test]
    fn sample_takes_the_point_nearest_each_cell_centre() {
        // With two cells per edge, each child is one cell; the centres are at 0.25 and 0.75.
        let mut children = corner_children(&[[0.1; 3], [0.26, 0.24, 0.25], [0.4; 3]], &[[0.9; 3], [0.7, 0.8, 0.75]]);
        let sampled = sample(NodeKey::ROOT, &UNIT, &mut children, &params(1, 2));
        assert_eq!(colors(&sampled), [1, 101]);
        assert_eq!(colors(&children[0].1.points), [0, 2]);
        assert_eq!(colors(&children[1].1.points), [100]);
        assert_eq!(children[0].1.points.positions, [[0.1; 3], [0.4; 3]]);
    }

    #[test]
    fn sample_ties_go_to_the_earlier_child_then_the_earlier_point() {
        // One cell with its centre at 0.5; all three points are equally far from it.
        let mut children = corner_children(&[[0.25, 0.375, 0.25], [0.375, 0.25, 0.25]], &[[0.75, 0.625, 0.75]]);
        let sampled = sample(NodeKey::ROOT, &UNIT, &mut children, &params(1, 1));
        assert_eq!(colors(&sampled), [0]);
        assert_eq!(colors(&children[0].1.points), [1]);
        assert_eq!(colors(&children[1].1.points), [100]);

        let mut children = corner_children(&[], &[[0.75, 0.625, 0.75], [0.625, 0.75, 0.75]]);
        let sampled = sample(NodeKey::ROOT, &UNIT, &mut children, &params(1, 1));
        assert_eq!(colors(&sampled), [100]);
        assert_eq!(colors(&children[1].1.points), [101]);
    }

    #[test]
    fn sample_family_drops_children_left_empty_and_without_descendants() {
        // With two cells per edge, each child is one cell.
        let [first, second, last] = [0, 1, 7].map(|octant| NodeKey::ROOT.child(octant));
        let children = vec![
            (first, open(chunk([[0.1; 3]]), false)),
            (second, open(PointChunk::new(3, ATTRIBUTES), true)),
            (last, open(chunk([[0.9; 3], [0.7, 0.8, 0.75]]), false)),
        ];
        let (parent, finished) = sample_family(NodeKey::ROOT, &UNIT, children, &params(1, 2));
        assert_eq!(parent.points.positions, [[0.1; 3], [0.7, 0.8, 0.75]]);
        assert!(parent.has_descendants);
        assert_eq!(finished.iter().map(|node| node.key).collect::<Vec<_>>(), [second, last]);
        assert!(finished[0].points.is_empty());
        assert_eq!(finished[1].points.positions, [[0.9; 3]]);

        let (parent, finished) =
            sample_family(NodeKey::ROOT, &UNIT, vec![(first, open(chunk([[0.1; 3]]), false))], &params(1, 2));
        assert_eq!(parent, open(chunk([[0.1; 3]]), false));
        assert!(finished.is_empty());
    }

    #[test]
    #[ignore = "timing; run with --release -- --ignored --nocapture"]
    fn time_five_million_points() {
        let points = random_points(5_000_000, 7);
        let start = std::time::Instant::now();
        let subtree = build(points, &OctreeParams::default());
        println!("5M points: {:?}, {} nodes", start.elapsed(), subtree.nodes.len());
    }
}
