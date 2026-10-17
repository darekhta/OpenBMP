//! Shared filesystem preparation for command outputs.

use std::fs::{File, create_dir_all};
use std::path::Path;

use crate::error::CliError;

/// Create a nonempty parent directory without creating the output file.
pub(super) fn ensure_parent_dir(path: &Path) -> Result<(), CliError> {
    if let Some(parent) = path.parent()
        && !parent.as_os_str().is_empty()
    {
        create_dir_all(parent).map_err(|source| CliError::Io {
            path: parent.to_path_buf(),
            source,
        })?;
    }
    Ok(())
}

/// Create or truncate an output file without creating parent directories.
pub(super) fn open_for_write(path: &Path) -> Result<File, CliError> {
    File::create(path).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })
}

#[cfg(test)]
mod tests {
    use std::error::Error;
    use std::fs;

    use super::*;

    #[test]
    fn parent_creation_accepts_paths_without_a_parent() -> Result<(), Box<dyn Error>> {
        ensure_parent_dir(Path::new(""))?;
        ensure_parent_dir(Path::new("report.json"))?;
        Ok(())
    }

    #[test]
    fn parent_creation_is_nested_and_idempotent() -> Result<(), Box<dyn Error>> {
        let root = tempfile::tempdir()?;
        let parent = root.path().join("nested").join("reports");
        let output = parent.join("report.json");

        ensure_parent_dir(&output)?;
        ensure_parent_dir(&output)?;

        assert!(parent.is_dir());
        assert!(!output.exists());
        Ok(())
    }

    #[test]
    fn parent_creation_reports_the_parent_path() -> Result<(), Box<dyn Error>> {
        let root = tempfile::tempdir()?;
        let parent = root.path().join("blocked");
        fs::write(&parent, b"existing file")?;
        let output = parent.join("report.json");

        assert!(matches!(
            ensure_parent_dir(&output),
            Err(CliError::Io { path, .. }) if path == parent
        ));
        Ok(())
    }

    #[test]
    fn opening_creates_and_truncates_the_output() -> Result<(), Box<dyn Error>> {
        let root = tempfile::tempdir()?;
        let output = root.path().join("report.json");

        drop(open_for_write(&output)?);
        assert!(output.is_file());

        fs::write(&output, b"previous contents")?;
        drop(open_for_write(&output)?);
        assert!(fs::read(&output)?.is_empty());
        Ok(())
    }

    #[test]
    fn opening_preserves_missing_parent_errors() -> Result<(), Box<dyn Error>> {
        let root = tempfile::tempdir()?;
        let parent = root.path().join("missing");
        let output = parent.join("report.json");

        assert!(matches!(
            open_for_write(&output),
            Err(CliError::Io { path, .. }) if path == output
        ));
        assert!(!parent.exists());
        Ok(())
    }
}
