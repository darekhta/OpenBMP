//! `openbmp check <scenario.toml>` — parse + validate, no run.

use std::path::Path;

use openbmp_scenario::Scenario;

use crate::error::CliError;

/// Outcome of `openbmp check`.
#[derive(Debug)]
pub struct CheckReport {
    /// Scenario name (from the `[meta]` table).
    pub scenario_name: String,
    /// Validation label declared by the scenario.
    pub validation_label: String,
    /// Resolved telemetry / data-package paths, in deterministic order.
    pub resolved_paths: Vec<(String, String)>,
    /// Resolved external input files with SHA-256 digests, in
    /// deterministic order.
    pub resolved_files: Vec<ResolvedFileEntry>,
}

/// One entry in [`CheckReport::resolved_files`].
#[derive(Debug)]
pub struct ResolvedFileEntry {
    /// Field path (e.g., `aero.deck`, `propulsion.motor.file`).
    pub field: String,
    /// Resolved absolute path.
    pub path: String,
    /// Lower-case hex SHA-256 digest.
    pub sha256_hex: String,
}

/// Entry point.
///
/// # Errors
///
/// Returns [`CliError::Scenario`] for parse or validation failure, and
/// for `ResolvedFile` failures (missing file, SHA-256 pin mismatch,
/// invalid pin) when the scenario references external files.
pub fn run(scenario_path: &Path) -> Result<CheckReport, CliError> {
    let scenario = Scenario::from_file(scenario_path)?;
    let resolved_paths = scenario
        .resolved_paths()
        .into_iter()
        .map(|(key, path)| (key, path.display().to_string()))
        .collect();
    let resolved_files = scenario
        .resolved_files()?
        .into_iter()
        .map(|(field, file)| ResolvedFileEntry {
            field,
            path: file.path.display().to_string(),
            sha256_hex: file.sha256_hex,
        })
        .collect();
    Ok(CheckReport {
        scenario_name: scenario.document.meta.name.clone(),
        validation_label: format!("{:?}", scenario.document.meta.validation),
        resolved_paths,
        resolved_files,
    })
}
