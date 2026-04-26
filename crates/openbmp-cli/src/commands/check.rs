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
}

/// Entry point.
///
/// # Errors
///
/// Returns [`CliError::Scenario`] for parse or validation failure.
pub fn run(scenario_path: &Path) -> Result<CheckReport, CliError> {
    let scenario = Scenario::from_file(scenario_path)?;
    let resolved_paths = scenario
        .resolved_paths()
        .into_iter()
        .map(|(key, path)| (key, path.display().to_string()))
        .collect();
    Ok(CheckReport {
        scenario_name: scenario.document.meta.name.clone(),
        validation_label: format!("{:?}", scenario.document.meta.validation),
        resolved_paths,
    })
}
