//! Measuring the staged points: their bounds and the range of every attribute they carry.

use rayon::prelude::*;

use crate::model::{Attributes, PointChunk, TASK_POINTS, ValueRange};

/// The bounds of all points and the smallest and largest value of each attribute. An attribute
/// the cloud does not carry, or a cloud without points, has no range.
#[derive(Clone, Debug, PartialEq)]
pub struct Extent {
    pub min: [f64; 3],
    pub max: [f64; 3],
    pub intensity: Option<ValueRange>,
    /// Red, green and blue.
    pub color: Option<[ValueRange; 3]>,
    pub classification: Option<ValueRange>,
    pub time: Option<ValueRange>,
}

const EMPTY: ValueRange = ValueRange { min: f64::INFINITY, max: f64::NEG_INFINITY };

impl Extent {
    /// The extent of no points yet, ready to be widened by [`Extent::include`].
    pub(super) fn empty(attributes: Attributes) -> Extent {
        Extent {
            min: [EMPTY.min; 3],
            max: [EMPTY.max; 3],
            intensity: attributes.intensity.then_some(EMPTY),
            color: attributes.color.then_some([EMPTY; 3]),
            classification: attributes.classification.then_some(EMPTY),
            time: attributes.time.then_some(EMPTY),
        }
    }

    /// Widens the extent to hold every point of `chunk`.
    pub(super) fn include(&mut self, chunk: &PointChunk) {
        for axis in 0..3 {
            let range = range_of(&chunk.positions, |position| position[axis]);
            self.min[axis] = self.min[axis].min(range.min);
            self.max[axis] = self.max[axis].max(range.max);
        }
        widen(&mut self.intensity, chunk.intensity.as_deref(), f64::from);
        widen(&mut self.classification, chunk.classification.as_deref(), f64::from);
        widen(&mut self.time, chunk.time.as_deref(), |time| time);
        if let (Some(ranges), Some(colors)) = (&mut self.color, &chunk.color) {
            for (channel, range) in ranges.iter_mut().enumerate() {
                *range = union(*range, range_of(colors, |color| f64::from(color[channel])));
            }
        }
    }

    /// What is reported once all points are seen: an extent without points becomes the origin,
    /// without ranges.
    pub(super) fn finished(self, point_count: u64) -> Extent {
        if point_count > 0 {
            return self;
        }
        Extent { min: [0.0; 3], max: [0.0; 3], intensity: None, color: None, classification: None, time: None }
    }
}

fn widen<T: Copy + Send + Sync>(range: &mut Option<ValueRange>, values: Option<&[T]>, value: impl Fn(T) -> f64 + Sync) {
    if let (Some(range), Some(values)) = (range, values) {
        *range = union(*range, range_of(values, value));
    }
}

fn range_of<T: Copy + Send + Sync>(values: &[T], value: impl Fn(T) -> f64 + Sync) -> ValueRange {
    values
        .par_chunks(TASK_POINTS)
        .map(|task| {
            task.iter().fold(EMPTY, |range, &item| {
                let value = value(item);
                ValueRange { min: range.min.min(value), max: range.max.max(value) }
            })
        })
        .reduce(|| EMPTY, union)
}

fn union(a: ValueRange, b: ValueRange) -> ValueRange {
    ValueRange { min: a.min.min(b.min), max: a.max.max(b.max) }
}
