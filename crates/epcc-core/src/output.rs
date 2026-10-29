//! Choosing output file names, and writing outputs so that a failed or cancelled conversion
//! never leaves a half-written file behind.
//!
//! Outputs are files, or folders for formats that write several files (Potree). Both are
//! handled alike; only a folder that holds a point cloud is ever replaced.

use std::collections::HashSet;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Mutex;

use crate::Result;
use crate::error::IoContext;
use crate::settings::ExistingFiles;

/// Output paths taken by running conversions, so parallel jobs never pick the same name.
#[derive(Debug, Default)]
pub struct ClaimedPaths(Mutex<HashSet<PathBuf>>);

impl ClaimedPaths {
    fn try_claim(&self, path: &Path) -> bool {
        let mut claimed = self.0.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
        claimed.insert(path.to_path_buf())
    }
}

/// Picks output paths for one input file.
pub(crate) struct OutputNames<'a> {
    pub folder: &'a Path,
    pub stem: String,
    pub extension: &'static str,
    pub input: &'a Path,
    pub existing: ExistingFiles,
    pub claimed: &'a ClaimedPaths,
}

impl OutputNames<'_> {
    /// The path for the output, or for one scan of it when `suffix` is given. `None` means the
    /// file exists and the settings say to skip it.
    pub fn pick(&self, suffix: Option<&str>) -> Option<PathBuf> {
        let base = match suffix {
            Some(suffix) => format!("{}_{}", self.stem, sanitize(suffix)),
            None => self.stem.clone(),
        };
        let path = self.folder.join(format!("{base}.{}", self.extension));
        let is_input = same_file(&path, self.input) || is_inside(self.input, &path);
        let exists = path.exists();
        match self.existing {
            ExistingFiles::Skip if exists => return None,
            ExistingFiles::Overwrite if !is_input && replaceable(&path) && self.claimed.try_claim(&path) => {
                return Some(path);
            }
            _ if !exists && self.claimed.try_claim(&path) => return Some(path),
            _ => {}
        }
        (1..)
            .map(|number| self.folder.join(format!("{base} ({number}).{}", self.extension)))
            .find(|candidate| !candidate.exists() && self.claimed.try_claim(candidate))
    }
}

fn same_file(a: &Path, b: &Path) -> bool {
    match (fs::canonicalize(a), fs::canonicalize(b)) {
        (Ok(a), Ok(b)) => a == b,
        _ => false,
    }
}

/// Whether `path` lies inside the existing folder `folder`, like a Potree cloud's
/// `metadata.json` inside its folder.
fn is_inside(path: &Path, folder: &Path) -> bool {
    match (fs::canonicalize(path), fs::canonicalize(folder)) {
        (Ok(path), Ok(folder)) => folder.is_dir() && path.starts_with(folder),
        _ => false,
    }
}

/// Whether an existing output may be replaced: any file, but only a folder that holds a point
/// cloud, never one with other content that happens to have the output's name.
fn replaceable(path: &Path) -> bool {
    !path.is_dir() || crate::format::is_cloud_folder(path)
}

/// Removes a file, or a folder with everything in it.
fn remove(path: &Path) -> std::io::Result<()> {
    if fs::symlink_metadata(path)?.is_dir() { fs::remove_dir_all(path) } else { fs::remove_file(path) }
}

/// Keeps scan names usable as part of a file name on every system.
fn sanitize(name: &str) -> String {
    let cleaned: String =
        name.chars().map(|c| if c.is_alphanumeric() || matches!(c, '-' | '_' | '.') { c } else { '_' }).collect();
    cleaned.trim_matches(['_', '.']).chars().take(80).collect()
}

/// An output being written under a temporary name. It only appears under its real name after
/// [`PendingOutput::commit`]; dropping it without committing deletes the temporary file.
pub(crate) struct PendingOutput {
    pub temporary: PathBuf,
    committed: bool,
}

impl PendingOutput {
    /// Writes to `<final name>.epcc-part`, a file or a folder.
    pub fn for_path(path: &Path) -> Self {
        let mut temporary = path.as_os_str().to_owned();
        temporary.push(".epcc-part");
        PendingOutput { temporary: temporary.into(), committed: false }
    }

    /// Writes to a temporary file or folder in `folder` whose final name is chosen later.
    pub fn unnamed(folder: &Path, stem: &str, extension: &str, number: usize) -> Self {
        PendingOutput { temporary: folder.join(format!("{stem}.{extension}.{number}.epcc-part")), committed: false }
    }

    pub fn commit(mut self, path: &Path) -> Result<()> {
        if fs::symlink_metadata(path).is_ok() {
            if !replaceable(path) {
                return Err(crate::Error::InvalidData(format!(
                    "{}: a folder of this name exists and is not a point cloud; it was left as it is",
                    path.display()
                )));
            }
            remove(path).at(path)?;
        }
        fs::rename(&self.temporary, path).at(path)?;
        self.committed = true;
        Ok(())
    }
}

impl Drop for PendingOutput {
    fn drop(&mut self) {
        if !self.committed {
            let _ = remove(&self.temporary);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_point_cloud_folders_are_replaced() {
        let folder = std::env::temp_dir().join(format!("epcc-output-{}", std::process::id()));
        let _ = fs::remove_dir_all(&folder);
        let cloud = folder.join("scan.potree");
        let other = folder.join("notes.potree");
        fs::create_dir_all(&cloud).unwrap();
        fs::create_dir_all(&other).unwrap();
        fs::write(cloud.join("cloud.js"), "{}").unwrap();
        fs::write(other.join("notes.txt"), "keep").unwrap();
        assert!(replaceable(&cloud) && !replaceable(&other));
        assert!(replaceable(&folder.join("missing.las")));
        assert!(is_inside(&cloud.join("cloud.js"), &cloud) && !is_inside(&other, &cloud));

        let pending = PendingOutput::unnamed(&folder, "scan", "potree", 0);
        fs::create_dir(&pending.temporary).unwrap();
        assert!(pending.commit(&other).is_err());
        assert!(other.join("notes.txt").is_file(), "the unrelated folder is kept");
        let pending = PendingOutput::unnamed(&folder, "scan", "potree", 1);
        fs::create_dir(&pending.temporary).unwrap();
        fs::write(pending.temporary.join("metadata.json"), "{}").unwrap();
        pending.commit(&cloud).unwrap();
        assert!(cloud.join("metadata.json").is_file() && !cloud.join("cloud.js").exists());
        let names: Vec<_> = fs::read_dir(&folder).unwrap().map(|entry| entry.unwrap().file_name()).collect();
        assert_eq!(names.len(), 2, "temporary folders are gone: {names:?}");
        fs::remove_dir_all(&folder).unwrap();
    }

    #[test]
    fn scan_names_become_safe_file_names() {
        assert_eq!(sanitize("Scan 001/East: wall"), "Scan_001_East__wall");
        assert_eq!(sanitize("..hidden"), "hidden");
    }
}
