//! `openbmp check-provenance <data/>`.
//!
//! Walks `root` and reports any source data file lacking a `provenance.md` in
//! the same directory. Generated scenario `out/` directories are ignored.

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
            if is_generated_output_dir(&path) {
                continue;
            }
            walk(&path, on_file)?;
        } else {
            on_file(&path)?;
        }
    }
    Ok(())
}

fn is_generated_output_dir(path: &Path) -> bool {
    file_name(path).is_some_and(|name| name == "out")
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use std::fs;

    use tempfile::tempdir;

    use super::*;

    #[test]
    fn reports_files_without_sibling_provenance() {
        let root = tempdir().expect("tempdir");
        let missing = root.path().join("missing.dat");
        fs::write(&missing, b"data").expect("write missing");
        let covered_dir = root.path().join("covered");
        fs::create_dir(&covered_dir).expect("covered dir");
        fs::write(covered_dir.join("table.dat"), b"data").expect("write covered");
        fs::write(covered_dir.join("provenance.md"), "# provenance\n").expect("write provenance");

        let report = run(root.path()).expect("provenance report");

        assert_eq!(report.files_seen, 3);
        assert_eq!(report.missing_provenance, vec![missing]);
    }

    #[test]
    fn skips_generated_out_directories() {
        let root = tempdir().expect("tempdir");
        fs::write(root.path().join("scenario.toml"), "name = \"fixture\"\n")
            .expect("write scenario");
        fs::write(
            root.path().join("provenance.md"),
            "files:\n  - scenario.toml\n",
        )
        .expect("write provenance");
        let out = root.path().join("out");
        fs::create_dir(&out).expect("out dir");
        fs::write(out.join("telemetry.csv"), "time_s,x\n0,0\n").expect("write output");

        let report = run(root.path()).expect("provenance report");

        assert_eq!(report.files_seen, 2);
        assert!(report.missing_provenance.is_empty());
    }
}
