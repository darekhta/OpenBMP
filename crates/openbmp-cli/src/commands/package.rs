//! `openbmp package ...` mission-package commands.

use std::path::Path;

use openbmp_sil::{MaterializeSidecarsReport, MissionPackage, PackageCheckReport};

use crate::error::CliError;

/// Validate a mission package manifest.
///
/// # Errors
///
/// Returns [`CliError`] for package, scenario, IO, or hash failures.
pub fn check(path: &Path) -> Result<PackageCheckReport, CliError> {
    Ok(MissionPackage::load(path)?.check()?)
}

/// Generate package sidecars declared by the manifest.
///
/// # Errors
///
/// Returns [`CliError`] for package, scenario, I-load, or IO failures.
pub fn materialize_sidecars(path: &Path) -> Result<MaterializeSidecarsReport, CliError> {
    Ok(MissionPackage::load(path)?.materialize_sidecars()?)
}
