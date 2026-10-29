//! Structure checks on written files: every count, index and bound must agree with the others.

use std::path::Path;

use super::*;
use crate::Progress;
use crate::format::{Format, WriteRequest};
use crate::model::{Attributes, CloudInfo, PointChunk, PointRecord, ScanInfo};
use crate::octree::staging::TempFolder;
use crate::settings::ExportSettings;

/// Deterministic pseudo-random numbers in 0..1.
fn random(seed: u64) -> impl FnMut() -> f64 {
    let mut state = seed | 1;
    move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state >> 11) as f64 / (1u64 << 53) as f64
    }
}

fn cloud(positions: impl IntoIterator<Item = [f64; 3]>, attributes: Attributes) -> PointChunk {
    let mut chunk = PointChunk::new(0, attributes);
    for (index, position) in positions.into_iter().enumerate() {
        chunk.push(&PointRecord {
            position,
            intensity: (index % 101) as f32 / 100.0,
            color: [(index * 7) as u16, (index * 13) as u16, (index * 31) as u16].map(|c| c.wrapping_mul(97)),
            ..PointRecord::default()
        });
    }
    chunk
}

/// Writes `chunks` as RCS and returns the file.
fn write(chunks: Vec<PointChunk>, estimate_normals: bool) -> Vec<u8> {
    let folder = TempFolder::create(&std::env::temp_dir()).unwrap();
    let path = folder.path().join("cloud.rcs");
    let attributes = chunks[0].attributes();
    let cloud = CloudInfo {
        attributes,
        scans: vec![ScanInfo::named("cloud")],
        point_count: None,
        crs_wkt: None,
        extra_fields: Vec::new(),
    };
    let mut settings = ExportSettings::default();
    settings.rcs.estimate_normals = estimate_normals;
    let request = WriteRequest {
        path: &path,
        cloud: &cloud,
        settings: &settings,
        source: Path::new("cloud.xyz"),
        source_format: Format::Xyz,
    };
    Format::Rcs.create_writer(&request).unwrap().write(&mut chunks.into_iter(), &Progress::new()).unwrap();
    std::fs::read(&path).unwrap()
}

/// What [`check`] found out about a file.
#[derive(Debug)]
struct Summary {
    header_points: u32,
    stored_points: u64,
    leaves: usize,
    node_levels: usize,
    half_size: f64,
    leaf_edges: Vec<f64>,
    largest_leaf: u64,
}

/// Checks the file's internal consistency and returns a summary.
fn check(file: &[u8]) -> Summary {
    assert_eq!(&file[..8], MAGIC);
    assert_eq!(read_u32(file, SECTION_TABLE), 4);
    let mut sections = std::collections::HashMap::new();
    let mut expected_offset = HEADER_SIZE as u64;
    for (index, kind) in [SECTION_LEAVES, SECTION_NODES, SECTION_POINTS, SECTION_PROPERTIES].into_iter().enumerate() {
        let entry = SECTION_TABLE + 4 + SECTION_ENTRY_SIZE * index;
        assert_eq!(read_u64(file, entry), kind);
        let (offset, size) = (read_u64(file, entry + 8), read_u64(file, entry + 16));
        assert_eq!(offset, expected_offset, "sections follow each other");
        expected_offset += size;
        sections.insert(kind, &file[offset as usize..(offset + size) as usize]);
    }
    assert_eq!(expected_offset, file.len() as u64, "the last section ends the file");
    let guid = std::str::from_utf8(&file[header::GUID..header::GUID + 38]).unwrap();
    assert!(guid.starts_with('{') && guid.ends_with('}') && guid.matches('_').count() == 4, "{guid}");

    let [h, ..] = read_f64s::<6>(file, header::ROOT_CUBE);
    let half_size = -h;
    let leaves = sections[&SECTION_LEAVES];
    let points = sections[&SECTION_POINTS];
    let leaf_count = read_u32(leaves, 0) as usize;
    assert_eq!(leaves.len(), 8 + leaf_count * LEAF_RECORD_SIZE);

    let mut block_start = 0;
    let mut stored_points = 0;
    let mut leaf_edges = Vec::new();
    let mut largest_leaf = 0;
    let mut cubes = Vec::new();
    for leaf in 0..leaf_count {
        let record = &leaves[8 + leaf * LEAF_RECORD_SIZE..8 + (leaf + 1) * LEAF_RECORD_SIZE];
        let min = read_f64s::<3>(record, leaf_record::CUBE_MIN);
        let max = read_f64s::<3>(record, leaf_record::CUBE_MAX);
        let (points_min, points_max) =
            (read_f64s::<3>(record, leaf_record::POINTS_MIN), read_f64s::<3>(record, leaf_record::POINTS_MAX));
        let levels = read_u32(record, leaf_record::LEVELS) as usize;
        let count = read_u32(record, leaf_record::POINT_COUNT) as usize;
        let size = read_u32(record, leaf_record::BLOCK_SIZE) as usize;
        assert!((1..=leaf::MAX_POINTS).contains(&count));
        let inner: Vec<u32> =
            (0..levels).map(|level| read_u32(record, leaf_record::INNER_PER_LEVEL + 4 * level)).collect();
        let point_words: Vec<u32> =
            (0..levels).map(|level| read_u32(record, leaf_record::LEAVES_PER_LEVEL + 4 * level)).collect();
        let block = &points[block_start..block_start + size];
        block_start += size;
        let word_count = (size - POINT_RECORD_SIZE * count) / 4;
        assert_eq!(word_count as u32, inner.iter().sum::<u32>() + point_words.iter().sum::<u32>());

        // The inner octree: level by level, children counted from each level's start.
        let mut referenced = vec![false; count];
        let mut at = 0;
        let mut level_size = 1;
        for level in 0..levels {
            assert_eq!(level_size, inner[level] + point_words[level], "leaf {leaf} level {level}");
            let mut children = 0;
            for _ in 0..level_size {
                let value = read_u32(block, 4 * at);
                at += 1;
                if value & word::POINT != 0 {
                    let index = (value & !word::POINT) as usize;
                    assert!(!std::mem::replace(&mut referenced[index], true), "record referenced twice");
                } else {
                    let mask = value >> word::MASK_SHIFT;
                    assert!(mask != 0 && mask < 256);
                    assert_eq!(value & ((1 << word::MASK_SHIFT) - 1), children, "first child index");
                    children += mask.count_ones();
                }
            }
            level_size = children;
        }
        assert_eq!(level_size, 0, "the last level has no children");
        assert!(referenced.iter().all(|&seen| seen), "every record is referenced");

        let records = &block[4 * word_count..];
        for record in records.as_chunks::<POINT_RECORD_SIZE>().0 {
            let packed = read_u64(record, 0);
            assert_eq!(packed >> 54, 0);
            for axis in 0..3 {
                let position = min[axis] + ((packed >> (18 * axis)) & FIELD_MASK) as f64 * QUANTUM;
                assert!(position >= points_min[axis] - 0.0006 && position <= points_max[axis] + 0.0006);
                assert!(position <= max[axis] + 0.0006);
            }
        }
        stored_points += count as u64;
        largest_leaf = largest_leaf.max(count as u64);
        leaf_edges.push(max[0] - min[0]);
        cubes.push(record[..48].to_vec());
    }
    assert_eq!(block_start, points.len(), "the blocks fill the point section");

    // The node table: children grouped by parent, leaves pointing at their leaf record.
    let nodes = sections[&SECTION_NODES];
    let counts: Vec<usize> = (0..32).map(|level| read_u32(nodes, 4 * level) as usize).collect();
    let node_levels = counts.iter().take_while(|&&count| count > 0).count();
    assert_eq!(nodes.len(), 128 + NODE_RECORD_SIZE * counts.iter().sum::<usize>());
    let mut leaf_seen = vec![false; leaf_count];
    let mut index = 0;
    for level in 0..node_levels {
        let mut children = 0;
        for _ in 0..counts[level] {
            let node = &nodes[128 + NODE_RECORD_SIZE * index..128 + NODE_RECORD_SIZE * (index + 1)];
            index += 1;
            let mask = node[6];
            let tail: Vec<i32> =
                (0..4).map(|slot| i32::from_le_bytes(node[56 + 4 * slot..60 + 4 * slot].try_into().unwrap())).collect();
            if mask == 0 {
                let leaf = tail[1] as usize;
                assert_eq!(tail[2], -1);
                assert!(!std::mem::replace(&mut leaf_seen[leaf], true));
                assert_eq!(&node[8..56], &cubes[leaf][..], "a leaf node has its leaf record's cube");
            } else {
                assert_eq!((tail[1], tail[2]), (-1, children as i32));
                children += mask.count_ones() as usize;
            }
        }
        assert_eq!(children, counts.get(level + 1).copied().unwrap_or(0), "level {level}");
    }
    assert!(leaf_seen.iter().all(|&seen| seen), "every leaf has a node");

    Summary {
        header_points: read_u32(file, header::POINT_COUNT),
        stored_points,
        leaves: leaf_count,
        node_levels,
        half_size,
        leaf_edges,
        largest_leaf,
    }
}

fn colour_and_intensity() -> Attributes {
    Attributes { intensity: true, color: true, ..Attributes::NONE }
}

#[test]
fn random_clouds_give_consistent_files() {
    let mut next = random(1);
    let positions: Vec<[f64; 3]> =
        (0..30_000).map(|_| [1000.0 + 20.0 * next(), -50.0 + 20.0 * next(), 5.0 * next()]).collect();
    let summary = check(&write(vec![cloud(positions, colour_and_intensity())], true));
    assert_eq!(summary.header_points, 30_000);
    assert!(summary.stored_points > 29_000 && summary.stored_points <= 30_000, "{summary:?}");
    assert_eq!(summary.node_levels, 5, "leaves sit at level 4");
}

#[test]
fn single_points_make_a_file() {
    let summary = check(&write(vec![cloud([[1.0, 2.0, 3.0]], Attributes::NONE)], true));
    assert_eq!((summary.header_points, summary.stored_points, summary.leaves), (1, 1, 1));
}

#[test]
fn points_repeated_within_half_a_millimetre_are_dropped() {
    let mut positions = Vec::new();
    for index in 0..1000 {
        let base = [index as f64 * 0.01, 0.0, 0.0];
        positions.push(base);
        positions.push([base[0] + 0.0002, 0.0001, 0.0]);
    }
    let all = cloud(positions, Attributes::NONE);
    // Split into chunks so a repeat falls on the first point of a chunk.
    let chunks: Vec<PointChunk> = (0..4)
        .map(|part| {
            let mut chunk = all.clone();
            chunk.retain(&(0..2000).map(|index| index / 500 == part).collect::<Vec<_>>());
            chunk
        })
        .collect();
    let summary = check(&write(chunks, false));
    assert_eq!(summary.header_points, 1000);
}

#[test]
fn dense_leaves_are_split() {
    let mut next = random(2);
    // 600 000 points in a 10 cm block, which far outliers make a small part of the cloud.
    let mut positions: Vec<[f64; 3]> =
        (0..20).map(|_| [10.0 * next() - 5.0, 10.0 * next() - 5.0, 10.0 * next() - 5.0]).collect();
    positions.extend((0..600_000).map(|_| [0.21 + 0.1 * next(), 0.21 + 0.1 * next(), 0.21 + 0.1 * next()]));
    let summary = check(&write(vec![cloud(positions, colour_and_intensity())], true));
    assert_eq!(summary.header_points, 600_020);
    assert!(summary.node_levels > 5, "the dense leaf was split: {summary:?}");
    assert!(summary.largest_leaf <= leaf::MAX_POINTS as u64);
}

#[test]
fn large_clouds_get_a_power_of_two_root() {
    let mut next = random(3);
    let positions: Vec<[f64; 3]> = (0..5_000).map(|_| [2000.0 * next(), 300.0 * next(), 10.0 * next()]).collect();
    let summary = check(&write(vec![cloud(positions, colour_and_intensity())], true));
    // Half-extent 1000 m: 1.44 × 327.375 × 4.
    assert_eq!(summary.half_size, 1.44 * 327.375 * 4.0);
    for edge in summary.leaf_edges {
        assert!((edge - 235.71).abs() < 0.1, "leaf edge {edge}");
    }
}

#[test]
fn small_clouds_get_a_scaled_root() {
    let positions = [[0.0, 0.0, 0.0], [4.0, 1.0, 1.0]];
    let summary = check(&write(vec![cloud(positions, Attributes::NONE)], true));
    assert!((summary.half_size - 1.44 * 2.01).abs() < 1e-12, "{summary:?}");
}

#[test]
fn identical_input_gives_identical_files() {
    let mut next = random(4);
    let positions: Vec<[f64; 3]> = (0..5_000).map(|_| [next(), next(), next()]).collect();
    let chunk = cloud(positions, colour_and_intensity());
    assert!(write(vec![chunk.clone()], true) == write(vec![chunk], true));
}
