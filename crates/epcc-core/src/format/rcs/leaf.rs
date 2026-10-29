//! One leaf's block in the points section: the points are thinned to one per small cell, the
//! survivors indexed by an octree of their own and stored in a fixed shuffled order.

use std::sync::OnceLock;

use super::{FIELD_MASK, POINT_RECORD_SIZE, QUANTUM, leaf_record, normals, word};
use crate::{Error, Result};

/// A leaf is split into cells halving its edge as long as they stay at least this size, in
/// metres, so its thinning cells are 1.5 to 3 mm (only tiny leaves, always split once, go below).
const THINNING_CELL: f64 = 0.0015;
/// The order records are stored in: record `r` starts as point `r` and is swapped with the
/// record at `RECORD_ORDER[r]`, for `r` from 1 up. One entry per point a leaf can hold.
static RECORD_ORDER: &[u8] = include_bytes!("record_order.bin");
/// The most points a leaf may hold before it is split.
pub const MAX_POINTS: usize = RECORD_ORDER.len() / 4;

/// A point as the RCS writer needs it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Point {
    /// Position relative to the scan centre.
    pub position: [f64; 3],
    /// Red, green, blue and intensity, 8 bits each.
    pub rgbi: [u8; 4],
    /// The source's normal code; 0 when the source has no normals.
    pub normal: u16,
}

/// A finished block and what the leaf record says about it.
pub struct Block {
    pub bytes: Vec<u8>,
    pub stored: usize,
    /// Inner and point words per level of the leaf's own octree.
    pub inner_per_level: Vec<u32>,
    pub points_per_level: Vec<u32>,
    /// Bounds of all of the leaf's points, the thinned-out ones included.
    pub points_min: [f64; 3],
    pub points_max: [f64; 3],
}

/// Builds the block of the leaf with cube `min`, `edge` and these points, in input order.
/// With `estimate_normals`, normals are estimated from the stored points instead of taken from
/// the source.
pub fn build(min: [f64; 3], edge: f64, points: &[Point], estimate_normals: bool) -> Result<Block> {
    let (points_min, points_max) = bounds(points);

    // Split at least once, then as long as the cells stay at least the thinning size.
    let mut levels = 1;
    while edge / 2f64.powi(levels + 1) >= THINNING_CELL {
        levels += 1;
    }
    let cell = edge / 2f64.powi(levels);
    let cell_of = |position: [f64; 3]| -> [i64; 3] {
        std::array::from_fn(|axis| ((position[axis] - min[axis]) / cell).floor() as i64)
    };

    // The point nearest its cell's centre survives; of equally near ones, the first.
    let cells: Vec<[i64; 3]> = points.iter().map(|point| cell_of(point.position)).collect();
    let mut ranked: Vec<(usize, f64)> = points
        .iter()
        .zip(&cells)
        .enumerate()
        .map(|(index, (point, key))| {
            let mut distance = 0.0;
            for axis in 0..3 {
                let centre = min[axis] + (key[axis] as f64 + 0.5) * cell;
                let d = point.position[axis] - centre;
                distance += d * d;
            }
            (index, distance)
        })
        .collect();
    ranked.sort_unstable_by(|&(a, da), &(b, db)| cells[a].cmp(&cells[b]).then(da.total_cmp(&db)).then(a.cmp(&b)));
    let mut kept: Vec<usize> = Vec::new();
    for (rank, &(index, _)) in ranked.iter().enumerate() {
        if rank == 0 || cells[index] != cells[ranked[rank - 1].0] {
            kept.push(index);
        }
    }
    kept.sort_unstable();
    if kept.len() < points.len() {
        // A thinned leaf lists its survivors by descending Morton code of their cell.
        let morton = |key: [i64; 3]| {
            let mut code = 0u64;
            for bit in 0..levels {
                for (axis, value) in key.iter().enumerate() {
                    code |= (((value >> bit) & 1) as u64) << (3 * bit as u32 + axis as u32);
                }
            }
            code
        };
        kept.sort_by_key(|&index| std::cmp::Reverse(morton(cells[index])));
    }
    let stored = kept.len();

    // The leaf's own octree, breadth first. A cell with one point is a point word.
    let mut words_per_level: Vec<Vec<Word>> = Vec::new();
    let mut level: Vec<Cell> = vec![Cell { min, edge, start: 0, len: stored }];
    let mut members: Vec<u32> = (0..stored as u32).collect();
    while !level.is_empty() {
        let mut next_level = Vec::new();
        let mut next_members = Vec::with_capacity(members.len());
        let mut words = Vec::with_capacity(level.len());
        for cell in &level {
            let own = &members[cell.start..cell.start + cell.len];
            if own.len() == 1 {
                words.push(Word::Point(own[0]));
                continue;
            }
            let half = cell.edge / 2.0;
            let octants: Vec<usize> =
                own.iter().map(|&member| octant(points[kept[member as usize]].position, cell.min, half)).collect();
            let first_child = next_level.len() as u32;
            let mut mask = 0u32;
            for child in 0..8 {
                let start = next_members.len();
                next_members.extend(own.iter().zip(&octants).filter(|&(_, &o)| o == child).map(|(&member, _)| member));
                let len = next_members.len() - start;
                if len > 0 {
                    mask |= 1 << child;
                    next_level.push(Cell { min: child_min(cell.min, child, half), edge: half, start, len });
                }
            }
            words.push(Word::Inner { mask, first_child });
        }
        words_per_level.push(words);
        level = next_level;
        members = next_members;
    }
    if words_per_level.len() > leaf_record::INNER_LEVELS {
        return Err(Error::invalid("RCS: a leaf's octree would be deeper than 32 levels"));
    }

    // Record r holds kept[order[r]].
    let order = record_order(stored);
    let mut record_of = vec![0u32; stored];
    for (record, &member) in order.iter().enumerate() {
        record_of[member as usize] = record as u32;
    }
    let normal_codes: Vec<u16> = if estimate_normals {
        let positions: Vec<[f64; 3]> = kept.iter().map(|&index| points[index].position).collect();
        normals::estimate_codes(&positions)
    } else {
        kept.iter().map(|&index| points[index].normal).collect()
    };

    let word_count: usize = words_per_level.iter().map(Vec::len).sum();
    let mut bytes = Vec::with_capacity(4 * word_count + POINT_RECORD_SIZE * stored);
    let mut inner_per_level = Vec::with_capacity(words_per_level.len());
    let mut points_per_level = Vec::with_capacity(words_per_level.len());
    for words in &words_per_level {
        let mut inner = 0;
        for word in words {
            let value = match *word {
                Word::Point(member) => word::POINT | record_of[member as usize],
                Word::Inner { mask, first_child } => {
                    inner += 1;
                    mask << word::MASK_SHIFT | first_child
                }
            };
            bytes.extend_from_slice(&value.to_le_bytes());
        }
        inner_per_level.push(inner);
        points_per_level.push(words.len() as u32 - inner);
    }
    for &member in &order {
        let point = &points[kept[member as usize]];
        bytes.extend_from_slice(&point_record(point, min, normal_codes[member as usize]));
    }
    Ok(Block { bytes, stored, inner_per_level, points_per_level, points_min, points_max })
}

/// A cell of the leaf's own octree; its points are `members[start..start + len]`.
struct Cell {
    min: [f64; 3],
    edge: f64,
    start: usize,
    len: usize,
}

enum Word {
    /// Stands for the kept point with this index.
    Point(u32),
    Inner {
        mask: u32,
        first_child: u32,
    },
}

/// The octant of `position` in the cube at `min` with half edge `half`: bit 0 for x, 1 for y,
/// 2 for z, set for the upper half.
pub fn octant(position: [f64; 3], min: [f64; 3], half: f64) -> usize {
    (0..3).map(|axis| usize::from(position[axis] >= min[axis] + half) << axis).sum()
}

/// The lower corner of child `octant` of the cube at `min` with half edge `half`.
pub fn child_min(min: [f64; 3], octant: usize, half: f64) -> [f64; 3] {
    std::array::from_fn(|axis| min[axis] + ((octant >> axis) & 1) as f64 * half)
}

/// The record a point is stored as, its position relative to the leaf corner `min`.
fn point_record(point: &Point, min: [f64; 3], normal: u16) -> [u8; POINT_RECORD_SIZE] {
    let mut packed = 0u64;
    for (axis, (value, corner)) in point.position.iter().zip(min).enumerate() {
        // The offset in millimetres is rounded to single precision before it is rounded to
        // the grid, like the consuming applications' own files.
        let millimetres = ((value - corner) * (1.0 / QUANTUM)) as f32;
        let field = (millimetres + 0.5).floor().clamp(0.0, FIELD_MASK as f32) as u64;
        packed |= field << (18 * axis);
    }
    let mut record = [0u8; POINT_RECORD_SIZE];
    record[0..8].copy_from_slice(&packed.to_le_bytes());
    let [red, green, blue, intensity] = point.rgbi;
    record[8..12].copy_from_slice(&[intensity, blue, green, red]);
    record[12..14].copy_from_slice(&normal.to_le_bytes());
    record
}

/// The stored order of `count` records: entry `r` is the index of the kept point in record `r`.
fn record_order(count: usize) -> Vec<u32> {
    static TABLE: OnceLock<Vec<u32>> = OnceLock::new();
    let table =
        TABLE.get_or_init(|| RECORD_ORDER.as_chunks::<4>().0.iter().map(|&bytes| u32::from_le_bytes(bytes)).collect());
    let mut order: Vec<u32> = (0..count as u32).collect();
    if count > table.len() {
        return order;
    }
    for (record, &other) in table.iter().enumerate().take(count).skip(1) {
        order.swap(record, other as usize);
    }
    order
}

fn bounds(points: &[Point]) -> ([f64; 3], [f64; 3]) {
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for point in points {
        for axis in 0..3 {
            min[axis] = min[axis].min(point.position[axis]);
            max[axis] = max[axis].max(point.position[axis]);
        }
    }
    (min, max)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_record_order_table_is_a_valid_swap_sequence() {
        assert_eq!(MAX_POINTS, 524_286);
        let order = record_order(MAX_POINTS);
        let mut seen = vec![false; MAX_POINTS];
        for &index in &order {
            assert!(!std::mem::replace(&mut seen[index as usize], true));
        }
        assert_eq!(record_order(5), [3, 1, 4, 2, 0]);
    }

    #[test]
    fn close_points_are_thinned_to_one_per_cell() {
        let point = |x: f64| Point { position: [x, 0.0101, 0.0101], ..Point::default() };
        // Edge 0.1 m: cells of 0.1 / 64 = 1.5625 mm. Both points share the first cell; the
        // one nearer its centre stays.
        let block = build([0.0; 3], 0.1, &[point(0.0002), point(0.0008), point(0.05)], false).unwrap();
        assert_eq!(block.stored, 2);
        assert_eq!(block.points_min[0], 0.0002);
        assert_eq!(block.points_max[0], 0.05);
    }
}
