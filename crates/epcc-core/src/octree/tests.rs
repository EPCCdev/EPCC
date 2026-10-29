//! End-to-end tests of [`build`]: they feed a cloud through all steps into a sink that keeps the
//! nodes in memory, then check the octree as a format writer would see it.

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;
use std::sync::Mutex;

use super::*;
use crate::Error;

/// Small limits, so a test cloud spans several subtrees and several levels above them.
const PARAMS: OctreeParams =
    OctreeParams { grid_cells: 16, max_leaf_points: 2_000, max_subtree_points: 20_000, max_depth: 12 };

/// Each point's index is stored in its colour, so every point can be traced through the build.
fn cloud(count: usize) -> Vec<PointChunk> {
    let attributes = Attributes { color: true, ..Attributes::NONE };
    let mut state = 0x2545_f491_4f6c_dd1du64;
    let mut random = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state >> 11) as f64 / (1u64 << 53) as f64
    };
    let mut chunks = Vec::new();
    for start in (0..count).step_by(50_000) {
        let mut chunk = PointChunk::with_capacity(0, attributes, 50_000);
        for index in start..count.min(start + 50_000) {
            // A floor, a wall and some scattered points: dense surfaces like a real scan.
            let position = match index % 3 {
                0 => [random() * 40.0, random() * 30.0, 0.0],
                1 => [random() * 40.0, 0.0, random() * 10.0],
                _ => [random() * 40.0, random() * 30.0, random() * 10.0],
            };
            chunk.positions.push(position.map(|value| value + 500_000.0));
            chunk.color.as_mut().unwrap().push(index_to_color(index));
        }
        chunks.push(chunk);
    }
    chunks
}

fn index_to_color(index: usize) -> [u16; 3] {
    [index as u16, (index >> 16) as u16, (index >> 32) as u16]
}

fn color_to_index(color: [u16; 3]) -> usize {
    color[0] as usize | (color[1] as usize) << 16 | (color[2] as usize) << 32
}

/// Stores each node's points as raw positions and point indices.
struct MemorySink<'a> {
    nodes: Vec<(NodeKey, Vec<u8>)>,
    result: &'a Mutex<Option<Built>>,
}

#[derive(Debug, PartialEq)]
struct Built {
    layout: Layout,
    entries: Vec<NodeEntry>,
    nodes: Vec<(NodeKey, Vec<u8>)>,
}

impl NodeSink for MemorySink<'_> {
    fn encode(&self, key: NodeKey, points: &PointChunk) -> Result<Vec<u8>> {
        assert!(!points.is_empty(), "{key:?}: nodes without points are never encoded");
        let mut bytes = Vec::new();
        for (position, color) in points.positions.iter().zip(points.color.as_ref().unwrap()) {
            position.iter().for_each(|value| bytes.extend_from_slice(&value.to_le_bytes()));
            bytes.extend_from_slice(&(color_to_index(*color) as u64).to_le_bytes());
        }
        Ok(bytes)
    }

    fn append(&mut self, key: NodeKey, bytes: &[u8]) -> Result<()> {
        self.nodes.push((key, bytes.to_vec()));
        Ok(())
    }

    fn finish(self, layout: &Layout, nodes: &[NodeEntry]) -> Result<()> {
        *self.result.lock().unwrap() =
            Some(Built { layout: layout.clone(), entries: nodes.to_vec(), nodes: self.nodes });
        Ok(())
    }
}

/// The points of one encoded node: (index, position).
fn decode(bytes: &[u8]) -> Vec<(usize, [f64; 3])> {
    let (records, _) = bytes.as_chunks::<32>();
    records
        .iter()
        .map(|record| {
            let value = |at: usize| f64::from_le_bytes(record[at..at + 8].try_into().unwrap());
            let index = u64::from_le_bytes(record[24..32].try_into().unwrap()) as usize;
            (index, [value(0), value(8), value(16)])
        })
        .collect()
}

struct WorkFolder(PathBuf);

impl WorkFolder {
    fn new(name: &str) -> WorkFolder {
        let path = std::env::temp_dir().join(format!("epcc-octree-test-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&path);
        std::fs::create_dir_all(&path).unwrap();
        WorkFolder(path)
    }

    fn is_empty(&self) -> bool {
        std::fs::read_dir(&self.0).unwrap().next().is_none()
    }
}

impl Drop for WorkFolder {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn run(chunks: Vec<PointChunk>, params: &OctreeParams, folder: &WorkFolder, progress: &Progress) -> Result<Built> {
    let result = Mutex::new(None);
    let attributes = Attributes { color: true, ..Attributes::NONE };
    build(&mut chunks.into_iter(), attributes, params, &folder.0, progress, |_| {
        Ok(MemorySink { nodes: Vec::new(), result: &result })
    })?;
    Ok(result.into_inner().unwrap().expect("finish was called"))
}

/// Checks everything a format writer relies on.
fn check_octree(built: &Built, count: usize, params: &OctreeParams) {
    let cube = built.layout.cube;
    assert_eq!(built.layout.point_count, count as u64);

    let keys: BTreeSet<NodeKey> = built.entries.iter().map(|entry| entry.key).collect();
    assert_eq!(keys.len(), built.entries.len(), "every node is listed once");
    assert!(keys.contains(&NodeKey::ROOT));
    for key in &keys {
        if let Some(parent) = key.parent() {
            assert!(keys.contains(&parent), "{key:?} has no parent node");
        }
    }
    let has_children = |key: NodeKey| keys.iter().any(|other| other.parent() == Some(key));

    let mut appended = built.nodes.iter();
    let mut next_offset = 0;
    let mut seen = vec![false; count];
    for entry in &built.entries {
        assert_eq!(entry.byte_offset, next_offset);
        next_offset += entry.byte_size;
        if entry.point_count == 0 {
            assert_eq!(entry.byte_size, 0);
            assert!(has_children(entry.key), "{:?} has neither points nor children", entry.key);
            continue;
        }
        let (key, bytes) = appended.next().expect("every node with points is appended");
        assert_eq!(entry.key, *key, "nodes are appended in list order");
        assert_eq!(entry.byte_size, bytes.len() as u64);

        let points = decode(bytes);
        assert_eq!(entry.point_count, points.len() as u64);
        if !has_children(*key) && points.len() > params.max_leaf_points && key.level < params.max_depth {
            let finest = cube.node_size(params.max_depth);
            assert!(fits_into(&points, finest), "{key:?} is a crowded leaf whose points could be split");
        }
        for (index, position) in points {
            assert!(!seen[index], "point {index} is in two nodes");
            seen[index] = true;
            assert_eq!(cube.key_at(position, key.level), *key, "point {index} lies outside {key:?}");
        }
    }
    assert!(appended.next().is_none(), "only listed nodes are appended");
    assert!(seen.iter().all(|&seen| seen), "every point is in a node");
}

/// Whether the points lie within a cube of edge `size`.
fn fits_into(points: &[(usize, [f64; 3])], size: f64) -> bool {
    (0..3).all(|axis| {
        let values = points.iter().map(|(_, position)| position[axis]);
        let (min, max) = values.fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), v| (min.min(v), max.max(v)));
        max - min <= size
    })
}

#[test]
fn builds_a_complete_octree_from_many_subtrees() {
    let folder = WorkFolder::new("many");
    let built = run(cloud(300_000), &PARAMS, &folder, &Progress::new()).unwrap();
    check_octree(&built, 300_000, &PARAMS);
    assert!(built.entries.iter().map(|entry| entry.key.level).max().unwrap() >= 4);
    assert!(folder.is_empty(), "temporary files are removed");
}

#[test]
fn small_cloud_is_a_single_subtree() {
    let folder = WorkFolder::new("single");
    let built = run(cloud(10_000), &PARAMS, &folder, &Progress::new()).unwrap();
    check_octree(&built, 10_000, &PARAMS);
    let positions: Vec<[f64; 3]> = cloud(10_000).iter().flat_map(|chunk| chunk.positions.clone()).collect();
    let min = std::array::from_fn(|axis| positions.iter().map(|p| p[axis]).fold(f64::INFINITY, f64::min));
    assert_eq!(built.layout.extent.min, min);
    assert_eq!(built.layout.cube.min, min);
    assert_eq!(built.layout.root_spacing, built.layout.cube.size / 16.0);
}

#[test]
fn same_input_gives_the_same_octree() {
    let folder = WorkFolder::new("deterministic");
    let first = run(cloud(120_000), &PARAMS, &folder, &Progress::new()).unwrap();
    let second = run(cloud(120_000), &PARAMS, &folder, &Progress::new()).unwrap();
    assert!(first == second);
}

#[test]
fn identical_points_do_not_split_endlessly() {
    let folder = WorkFolder::new("duplicates");
    let mut chunks = cloud(30_000);
    for chunk in &mut chunks {
        chunk.positions.iter_mut().for_each(|position| *position = [1.0, 2.0, 3.0]);
    }
    let built = run(chunks, &PARAMS, &folder, &Progress::new()).unwrap();
    check_octree(&built, 30_000, &PARAMS);
    assert!(built.entries.iter().all(|entry| entry.point_count > 0), "no chain of empty nodes");
}

#[test]
fn empty_input_gives_an_empty_octree() {
    let folder = WorkFolder::new("empty");
    let built = run(Vec::new(), &PARAMS, &folder, &Progress::new()).unwrap();
    assert_eq!(built.layout.point_count, 0);
    assert!(built.entries.is_empty());
    assert!(folder.is_empty());
}

#[test]
fn cancelling_removes_temporary_files() {
    let folder = WorkFolder::new("cancel");
    let progress = Progress::new();
    progress.cancel();
    let result = run(cloud(100_000), &PARAMS, &folder, &progress);
    assert!(matches!(result, Err(Error::Cancelled)));
    assert!(folder.is_empty());
}

#[test]
fn a_failing_sink_removes_temporary_files() {
    struct FailingSink;
    impl NodeSink for FailingSink {
        fn encode(&self, _key: NodeKey, _points: &PointChunk) -> Result<Vec<u8>> {
            Err(Error::InvalidData("encoding failed".into()))
        }
        fn append(&mut self, _key: NodeKey, _bytes: &[u8]) -> Result<()> {
            Ok(())
        }
        fn finish(self, _layout: &Layout, _nodes: &[NodeEntry]) -> Result<()> {
            Ok(())
        }
    }
    let folder = WorkFolder::new("failing");
    let attributes = Attributes { color: true, ..Attributes::NONE };
    let result =
        build(&mut cloud(60_000).into_iter(), attributes, &PARAMS, &folder.0, &Progress::new(), |_| Ok(FailingSink));
    assert!(matches!(result, Err(Error::InvalidData(_))));
    assert!(folder.is_empty());
}

#[test]
fn levels_above_the_subtrees_thin_out_towards_the_root() {
    let folder = WorkFolder::new("levels");
    let built = run(cloud(300_000), &PARAMS, &folder, &Progress::new()).unwrap();
    let mut per_level: BTreeMap<u8, u64> = BTreeMap::new();
    for entry in &built.entries {
        *per_level.entry(entry.key.level).or_default() += entry.point_count;
    }
    let root = built.entries.iter().find(|entry| entry.key == NodeKey::ROOT).unwrap();
    assert!(root.point_count > 0);
    assert!(root.point_count <= u64::from(PARAMS.grid_cells).pow(3));
    assert!(per_level[&0] < per_level[&2], "deeper levels hold more points: {per_level:?}");
}

/// Run with `cargo test --release -p epcc-core time_twenty -- --ignored --nocapture`.
#[test]
#[ignore = "timing, not a check"]
fn time_twenty_million_points() {
    struct CountingSink;
    impl NodeSink for CountingSink {
        fn encode(&self, _key: NodeKey, points: &PointChunk) -> Result<Vec<u8>> {
            Ok(vec![0; points.len()])
        }
        fn append(&mut self, _key: NodeKey, _bytes: &[u8]) -> Result<()> {
            Ok(())
        }
        fn finish(self, layout: &Layout, nodes: &[NodeEntry]) -> Result<()> {
            println!("{} points in {} nodes", layout.point_count, nodes.len());
            Ok(())
        }
    }
    let folder = WorkFolder::new("timing");
    let chunks = cloud(20_000_000);
    let attributes = Attributes { color: true, ..Attributes::NONE };
    let started = std::time::Instant::now();
    build(&mut chunks.into_iter(), attributes, &OctreeParams::default(), &folder.0, &Progress::new(), |_| {
        println!("staged after {:.2?}", started.elapsed());
        Ok(CountingSink)
    })
    .unwrap();
    println!("total {:.2?}", started.elapsed());
}

#[test]
fn points_without_a_finite_position_are_left_out() {
    let folder = WorkFolder::new("invalid");
    let mut chunks = cloud(20_000);
    chunks[0].positions[3] = [f64::NAN, 0.0, 0.0];
    chunks[0].positions[7] = [0.0, f64::INFINITY, 0.0];
    let progress = Progress::new();
    let built = run(chunks, &PARAMS, &folder, &progress).unwrap();
    assert_eq!(built.layout.point_count, 19_998);
    assert!(built.layout.cube.size < 100.0, "the invalid points do not stretch the cube");
    assert_eq!(progress.warnings().len(), 1);
}

#[test]
fn a_far_outlier_keeps_the_octree_complete() {
    let folder = WorkFolder::new("outlier");
    let mut chunks = cloud(100_000);
    chunks[0].positions[0] = [0.0; 3];
    // The cube is now 12,500 times the cloud, so the cloud only splits a dozen levels down.
    let params = OctreeParams { max_depth: MAX_DEPTH, ..PARAMS };
    let built = run(chunks, &params, &folder, &Progress::new()).unwrap();
    check_octree(&built, 100_000, &params);
    let leaves =
        built.entries.iter().filter(|entry| !built.entries.iter().any(|other| other.key.parent() == Some(entry.key)));
    assert!(leaves.clone().count() > 8, "the cloud is split although it is small next to the cube");
    assert!(leaves.into_iter().all(|leaf| leaf.point_count as usize <= params.max_leaf_points));
}

#[test]
fn the_layout_reports_attribute_ranges() {
    let folder = WorkFolder::new("ranges");
    let built = run(cloud(1_000), &PARAMS, &folder, &Progress::new()).unwrap();
    let extent = &built.layout.extent;
    let colors = extent.color.expect("the cloud has colour");
    assert_eq!((colors[0].min, colors[0].max), (0.0, 999.0), "the colour holds the point index");
    assert_eq!(extent.intensity, None);
    assert_eq!(extent.time, None);
}

#[test]
fn unusable_parameters_are_refused() {
    let folder = WorkFolder::new("parameters");
    for params in [
        OctreeParams { grid_cells: 1, ..PARAMS },
        OctreeParams { max_leaf_points: 0, ..PARAMS },
        OctreeParams { max_subtree_points: 10, ..PARAMS },
        OctreeParams { max_depth: MAX_DEPTH + 1, ..PARAMS },
    ] {
        assert!(matches!(run(cloud(10), &params, &folder, &Progress::new()), Err(Error::InvalidSetting(_))));
    }
    assert!(folder.is_empty());
}
