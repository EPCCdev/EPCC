use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use crate::{Error, Result};

/// Shared between a running conversion and whoever watches it: progress, cancellation and the
/// warnings collected along the way.
#[derive(Debug, Default)]
pub struct Progress {
    fraction_bits: AtomicU32,
    /// The part of the work that comes after reading, from 0 to 1. Zero for most writers, which
    /// write while the input is read.
    after_reading_bits: AtomicU32,
    cancelled: AtomicBool,
    warnings: Mutex<Vec<String>>,
}

impl Progress {
    pub fn new() -> Self {
        Self::default()
    }

    /// How much of the conversion is done, from 0 to 1.
    pub fn fraction(&self) -> f32 {
        f32::from_bits(self.fraction_bits.load(Ordering::Relaxed))
    }

    pub fn set_fraction(&self, fraction: f32) {
        let clamped = fraction.clamp(0.0, 1.0);
        self.fraction_bits.store(clamped.to_bits(), Ordering::Relaxed);
    }

    /// Reports reading progress: `done` of `total` input points or bytes.
    pub fn set_done_of(&self, done: u64, total: u64) {
        if total > 0 {
            self.set_fraction(done as f32 / total as f32 * (1.0 - self.after_reading()));
        }
    }

    /// Declares that `share` of the work happens after the input is read, for writers that
    /// only start once they have seen every point.
    pub fn reserve_after_reading(&self, share: f32) {
        self.after_reading_bits.store(share.clamp(0.0, 1.0).to_bits(), Ordering::Relaxed);
    }

    /// Reports progress of the work after reading, from 0 to 1.
    pub fn set_after_reading_done(&self, fraction: f32) {
        let share = self.after_reading();
        self.set_fraction(1.0 - share + share * fraction.clamp(0.0, 1.0));
    }

    fn after_reading(&self) -> f32 {
        f32::from_bits(self.after_reading_bits.load(Ordering::Relaxed))
    }

    pub fn cancel(&self) {
        self.cancelled.store(true, Ordering::Relaxed);
    }

    pub fn is_cancelled(&self) -> bool {
        self.cancelled.load(Ordering::Relaxed)
    }

    pub fn check_cancelled(&self) -> Result<()> {
        if self.is_cancelled() { Err(Error::Cancelled) } else { Ok(()) }
    }

    /// Records a warning once; repeating the same message has no effect.
    pub fn warn(&self, message: impl Into<String>) {
        let message = message.into();
        let mut warnings = self.lock_warnings();
        if !warnings.contains(&message) {
            warnings.push(message);
        }
    }

    pub fn warnings(&self) -> Vec<String> {
        self.lock_warnings().clone()
    }

    fn lock_warnings(&self) -> std::sync::MutexGuard<'_, Vec<String>> {
        self.warnings.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
    }
}
