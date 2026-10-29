//! Timing of the minimum spacing filter on large generated clouds. Not part of the normal test
//! run; start it with
//!
//! ```text
//! cargo test --release -p epcc-core --test spacing_bench -- --ignored --nocapture
//! ```
//!
//! Each run prints the kept point count and a checksum over the kept positions in output order,
//! so results can be compared between versions.

use std::time::{Duration, Instant};

use epcc_core::model::{Attributes, CHUNK_POINTS, PointChunk};
use epcc_core::reduce::{MinSpacing, PointFilter};

/// A small deterministic random number generator.
struct Random(u64);

impl Random {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// A surface scanned row by row: rows along x, 2 mm apart in y, 2 mm between points in a row,
/// with a little noise. `reverse` scans the rows in the opposite direction.
fn rows(count: u64, seed: u64, reverse: bool) -> impl Iterator<Item = PointChunk> {
    const ROW_POINTS: u64 = 5_000;
    const STEP: f64 = 0.002;
    let rows_total = count / ROW_POINTS;
    let mut random = Random(seed);
    let mut produced = 0u64;
    std::iter::from_fn(move || {
        if produced >= count {
            return None;
        }
        let len = (count - produced).min(CHUNK_POINTS as u64);
        let mut positions = Vec::with_capacity(len as usize);
        for index in produced..produced + len {
            let mut row = index / ROW_POINTS;
            if reverse {
                row = rows_total - 1 - row;
            }
            let along = index % ROW_POINTS;
            let x = along as f64 * STEP + (random.next() - 0.5) * 0.001;
            let y = row as f64 * STEP + (random.next() - 0.5) * 0.001;
            let z = 0.05 * (x * 3.0).sin() + 0.05 * (y * 2.0).cos() + (random.next() - 0.5) * 0.0005;
            positions.push([500_000.0 + x, 5_400_000.0 + y, 300.0 + z]);
        }
        produced += len;
        Some(PointChunk { positions, ..PointChunk::new(0, Attributes::NONE) })
    })
}

struct Outcome {
    kept: u64,
    checksum: u64,
    filtering: Duration,
}

fn run(spacing: f64, chunks: impl Iterator<Item = PointChunk>) -> Outcome {
    let mut filter = MinSpacing::new(spacing);
    let mut outcome = Outcome { kept: 0, checksum: 0xcbf2_9ce4_8422_2325, filtering: Duration::ZERO };
    for mut chunk in chunks {
        let started = Instant::now();
        filter.apply(&mut chunk).unwrap();
        outcome.filtering += started.elapsed();
        outcome.kept += chunk.len() as u64;
        for position in &chunk.positions {
            for value in position {
                outcome.checksum = (outcome.checksum ^ value.to_bits()).wrapping_mul(0x0100_0000_01b3);
            }
        }
    }
    outcome
}

fn report(name: &str, read: u64, outcome: &Outcome) {
    println!(
        "{name}: {read} points read, {} kept, checksum {:016x}, filtering {:.2} s",
        outcome.kept,
        outcome.checksum,
        outcome.filtering.as_secs_f64()
    );
}

#[test]
#[ignore = "takes minutes; run explicitly"]
fn one_sweep_of_100_million_points() {
    let count = 100_000_000;
    let outcome = run(0.003, rows(count, 1, false));
    report("one sweep", count, &outcome);
}

#[test]
#[ignore = "takes minutes; run explicitly"]
fn two_sweeps_of_50_million_points() {
    let half = 50_000_000;
    let outcome = run(0.002, rows(half, 2, false).chain(rows(half, 3, true)));
    report("two sweeps", 2 * half, &outcome);
}

/// Points in random order over a 20 m × 20 m surface: the worst case for the blocks on disk.
fn random_order(count: u64, seed: u64) -> impl Iterator<Item = PointChunk> {
    let mut random = Random(seed);
    let mut produced = 0u64;
    std::iter::from_fn(move || {
        if produced >= count {
            return None;
        }
        let len = (count - produced).min(CHUNK_POINTS as u64);
        let positions = (0..len).map(|_| [random.next() * 20.0, random.next() * 20.0, random.next() * 0.001]).collect();
        produced += len;
        Some(PointChunk { positions, ..PointChunk::new(0, Attributes::NONE) })
    })
}

#[test]
#[ignore = "takes minutes; run explicitly"]
fn random_order_of_30_million_points() {
    let count = 30_000_000;
    let outcome = run(0.003, random_order(count, 4));
    report("random order", count, &outcome);
}
