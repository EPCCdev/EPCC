//! Turns the input arguments into a list of files. Windows shells do not expand `*.e57`, so
//! patterns are expanded here; folders stand for the supported files inside them, except a
//! Potree folder, which is one point cloud.

use std::fs;
use std::path::{Path, PathBuf};

use epcc_core::Format;

pub fn expand(arguments: &[PathBuf]) -> Result<Vec<PathBuf>, String> {
    let mut files = Vec::new();
    for argument in arguments {
        let found = if Format::from_path(argument) == Some(Format::Potree) {
            vec![argument.clone()]
        } else if argument.is_dir() {
            supported_files_in(argument, |_| true)?
        } else if is_pattern(argument) {
            let pattern = argument.file_name().map(|name| name.to_string_lossy().into_owned()).unwrap_or_default();
            let folder = argument.parent().filter(|p| !p.as_os_str().is_empty()).unwrap_or(Path::new("."));
            supported_files_in(folder, |name| matches_pattern(&pattern, name))?
        } else if argument.is_file() {
            vec![argument.clone()]
        } else {
            return Err(format!("{}: no such file", argument.display()));
        };
        if found.is_empty() {
            return Err(format!("{}: no supported point cloud files", argument.display()));
        }
        files.extend(found);
    }
    Ok(files)
}

fn is_pattern(path: &Path) -> bool {
    path.file_name().is_some_and(|name| name.to_string_lossy().contains(['*', '?']))
}

fn supported_files_in(folder: &Path, name_filter: impl Fn(&str) -> bool) -> Result<Vec<PathBuf>, String> {
    let entries = fs::read_dir(folder).map_err(|error| format!("{}: {error}", folder.display()))?;
    let mut files: Vec<PathBuf> = entries
        .filter_map(|entry| entry.ok().map(|entry| entry.path()))
        .filter(|path| Format::from_path(path).is_some())
        .filter(|path| path.file_name().is_some_and(|name| name_filter(&name.to_string_lossy())))
        .collect();
    files.sort();
    Ok(files)
}

/// Matches `*` (any run of characters) and `?` (one character), ignoring case.
fn matches_pattern(pattern: &str, name: &str) -> bool {
    let pattern: Vec<char> = pattern.to_lowercase().chars().collect();
    let name: Vec<char> = name.to_lowercase().chars().collect();
    let (mut p, mut n) = (0, 0);
    let mut backtrack: Option<(usize, usize)> = None;
    while n < name.len() {
        match pattern.get(p) {
            Some('*') => {
                backtrack = Some((p, n));
                p += 1;
            }
            Some(&c) if c == '?' || c == name[n] => {
                p += 1;
                n += 1;
            }
            _ => match backtrack {
                Some((star, matched)) => {
                    p = star + 1;
                    n = matched + 1;
                    backtrack = Some((star, matched + 1));
                }
                None => return false,
            },
        }
    }
    pattern[p..].iter().all(|&c| c == '*')
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn potree_folders_are_one_input() {
        let folder = std::env::temp_dir().join(format!("epcc-inputs-{}", std::process::id()));
        let cloud = folder.join("site.potree");
        let _ = fs::remove_dir_all(&folder);
        fs::create_dir_all(&cloud).unwrap();
        for name in ["metadata.json", "hierarchy.bin", "octree.bin"] {
            fs::write(cloud.join(name), "").unwrap();
        }
        fs::write(folder.join("scan.las"), "").unwrap();
        assert_eq!(expand(std::slice::from_ref(&cloud)).unwrap(), std::slice::from_ref(&cloud));
        assert_eq!(expand(std::slice::from_ref(&folder)).unwrap(), [folder.join("scan.las"), cloud.clone()]);
        fs::remove_dir_all(&folder).unwrap();
    }

    #[test]
    fn wildcard_patterns() {
        assert!(matches_pattern("*.e57", "Scan 01.E57"));
        assert!(matches_pattern("scan_??.las", "scan_07.las"));
        assert!(!matches_pattern("scan_??.las", "scan_7.las"));
        assert!(matches_pattern("*a*b*", "xxaxxbxx"));
        assert!(!matches_pattern("*.laz", "cloud.las"));
    }
}
