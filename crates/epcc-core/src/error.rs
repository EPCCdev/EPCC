use std::path::{Path, PathBuf};

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("{path}: {source}")]
    Io { path: PathBuf, source: std::io::Error },

    #[error("LAS/LAZ: {0}")]
    Las(#[from] las::Error),

    #[error("E57: {0}")]
    E57(#[from] e57::Error),

    #[error("{0}")]
    InvalidData(String),

    #[error("{0}")]
    Unsupported(String),

    #[error("{0}")]
    InvalidSetting(String),

    #[error("cancelled")]
    Cancelled,

    /// A pipeline stage stopped because the stage after it gave up. The real cause is reported
    /// by that later stage, so this never reaches the user.
    #[error("output stopped")]
    OutputStopped,
}

pub type Result<T> = std::result::Result<T, Error>;

impl Error {
    pub fn invalid(message: impl Into<String>) -> Self {
        Error::InvalidData(message.into())
    }
}

/// Attaches the file path to I/O errors, so messages say which file failed.
pub trait IoContext<T> {
    fn at(self, path: &Path) -> Result<T>;
}

impl<T> IoContext<T> for std::io::Result<T> {
    fn at(self, path: &Path) -> Result<T> {
        self.map_err(|source| Error::Io { path: path.to_path_buf(), source })
    }
}
