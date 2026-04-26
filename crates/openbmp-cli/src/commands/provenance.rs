//! `openbmp check-provenance <data/>` — Phase-1 stub.
//!
//! Walks `root` and reports any data file lacking a `provenance.md` in
//! the same directory. Phase-2 work (`docs/data-provenance.md
//! § Machine Checks`) replaces this with full sidecar parsing and hash
//! recomputation.

use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};

use crate::error::CliError;

/// Outcome of `openbmp check-provenance`.
#[derive(Debug)]
pub struct ProvenanceReport {
    /// Number of files visited.
    pub files_seen: usize,
    /// Files that lack a sibling `provenance.md`. Sorted, deterministic.
    pub missing_provenance: Vec<PathBuf>,
}

/// Entry point.
///
/// # Errors
///
/// Returns [`CliError::Io`] when the root cannot be walked.
pub fn run(root: &Path) -> Result<ProvenanceReport, CliError> {
    let mut files_seen = 0_usize;
    let mut missing = BTreeSet::new();

    walk(root, &mut |file_path| -> Result<(), CliError> {
        files_seen += 1;
        if file_name(file_path).is_some_and(|name| name == "provenance.md") {
            return Ok(());
        }
        let provenance = match file_path.parent() {
            Some(dir) => dir.join("provenance.md"),
            None => return Ok(()),
        };
        if !provenance.exists() {
            missing.insert(file_path.to_path_buf());
        }
        Ok(())
    })?;

    Ok(ProvenanceReport {
        files_seen,
        missing_provenance: missing.into_iter().collect(),
    })
}

fn file_name(path: &Path) -> Option<&str> {
    path.file_name().and_then(|n| n.to_str())
}

fn walk<F>(root: &Path, on_file: &mut F) -> Result<(), CliError>
where
    F: FnMut(&Path) -> Result<(), CliError>,
{
    if !root.exists() {
        return Err(CliError::Io {
            path: root.to_path_buf(),
            source: std::io::Error::new(
                std::io::ErrorKind::NotFound,
                "provenance root does not exist",
            ),
        });
    }
    if root.is_file() {
        return on_file(root);
    }

    let entries = fs::read_dir(root).map_err(|source| CliError::Io {
        path: root.to_path_buf(),
        source,
    })?;
    let mut paths: Vec<PathBuf> = Vec::new();
    for entry in entries {
        let entry = entry.map_err(|source| CliError::Io {
            path: root.to_path_buf(),
            source,
        })?;
        paths.push(entry.path());
    }
    // Deterministic recursion order.
    paths.sort();

    for path in paths {
        if path.is_dir() {
            walk(&path, on_file)?;
        } else {
            on_file(&path)?;
        }
    }
    Ok(())
}
