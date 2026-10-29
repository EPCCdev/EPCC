//! Point reduction: filters that drop points from the stream between reader and writer.
//!
//! Filters see the points in file order and decide deterministically, so the same input and
//! settings always give the same output. Kept points are never moved or averaged.

mod spacing;

use std::path::Path;

pub use spacing::MinSpacing;

use crate::Result;
use crate::model::PointChunk;
use crate::settings::ProcessingSettings;

pub trait PointFilter: Send {
    /// Removes the dropped points from `chunk`.
    fn apply(&mut self, chunk: &mut PointChunk) -> Result<()>;
}

/// The filters the settings ask for, cheapest first. Filters that need temporary files put them
/// into a new folder inside `work_folder` and remove it when they are dropped.
pub fn filters_for(settings: &ProcessingSettings, work_folder: &Path) -> Vec<Box<dyn PointFilter>> {
    let mut filters: Vec<Box<dyn PointFilter>> = Vec::new();
    if let Some(n) = settings.every_nth.filter(|&n| n > 1) {
        filters.push(Box::new(EveryNth::new(n)));
    }
    if let Some(spacing) = settings.min_spacing {
        filters.push(Box::new(MinSpacing::with_work_folder(spacing, work_folder)));
    }
    filters
}

/// Keeps the first point and then every n-th one.
pub struct EveryNth {
    n: u64,
    seen: u64,
}

impl EveryNth {
    pub fn new(n: u64) -> Self {
        EveryNth { n: n.max(1), seen: 0 }
    }
}

impl PointFilter for EveryNth {
    fn apply(&mut self, chunk: &mut PointChunk) -> Result<()> {
        let keep: Vec<bool> = (0..chunk.len() as u64).map(|i| (self.seen + i).is_multiple_of(self.n)).collect();
        self.seen += chunk.len() as u64;
        chunk.retain(&keep);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Attributes;

    fn chunk(positions: Vec<[f64; 3]>) -> PointChunk {
        PointChunk { positions, ..PointChunk::new(0, Attributes::NONE) }
    }

    #[test]
    fn every_nth_continues_across_chunks() {
        let mut filter = EveryNth::new(3);
        let mut first = chunk((0..4).map(|i| [i as f64, 0.0, 0.0]).collect());
        let mut second = chunk((4..8).map(|i| [i as f64, 0.0, 0.0]).collect());
        filter.apply(&mut first).unwrap();
        filter.apply(&mut second).unwrap();
        let kept: Vec<f64> = first.positions.iter().chain(&second.positions).map(|p| p[0]).collect();
        assert_eq!(kept, [0.0, 3.0, 6.0]);
    }
}
