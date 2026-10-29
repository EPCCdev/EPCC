//! EPCC core: converts point clouds between file formats.
//!
//! Every reader turns its file into a stream of [`PointChunk`]s, and every writer consumes such a
//! stream. Converting is therefore always "reader → filters → writer", see [`convert`].

pub mod batch;
pub mod convert;
pub mod error;
pub mod format;
pub mod model;
pub mod octree;
pub mod progress;
pub mod reduce;
pub mod settings;

mod output;

pub use batch::run_batch;
pub use convert::{ConversionJob, ConversionReport, Outcome, convert};
pub use error::{Error, Result};
pub use format::Format;
pub use model::{Attributes, CloudInfo, PointChunk};
pub use output::ClaimedPaths;
pub use progress::Progress;
pub use settings::Settings;

/// Program version, shown in the user interfaces.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
