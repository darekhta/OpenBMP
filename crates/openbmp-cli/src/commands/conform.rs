//! `openbmp conform <scenario...>` — fork-runnable conformance smoke suite.

use std::path::{Path, PathBuf};

use openbmp_runner as runner;
use openbmp_scenario::Scenario;

use crate::commands::check;
use crate::error::CliError;

/// Outcome of one scenario in a conformance run.
#[derive(Debug)]
pub struct ConformanceScenarioReport {
    /// Scenario path supplied by the caller.
    pub path: PathBuf,
    /// Scenario name from `[meta]`.
    pub scenario_name: String,
    /// Scenario validation label.
    pub validation_label: String,
    /// Final step index reached.
    pub final_step: u64,
    /// Final simulation time in seconds.
    pub final_time_s: f64,
    /// Stop reason label.
    pub stop_label: String,
}

/// Aggregate conformance report.
#[derive(Debug)]
pub struct ConformanceReport {
    /// Per-scenario reports in input order.
    pub scenarios: Vec<ConformanceScenarioReport>,
}

/// Run conformance over the supplied scenarios.
///
/// The command intentionally writes no telemetry files. It validates
/// the scenario and then exercises the normal runner path, which is
/// enough for a downstream fork to prove its checkout can parse,
/// construct, and execute the shared OpenBMP scenarios.
///
/// # Errors
///
/// Returns [`CliError`] for scenario validation or runner failures.
pub fn run(scenarios: &[PathBuf]) -> Result<ConformanceReport, CliError> {
    let mut reports = Vec::with_capacity(scenarios.len());
    for path in scenarios {
        reports.push(run_one(path)?);
    }
    Ok(ConformanceReport { scenarios: reports })
}

fn run_one(path: &Path) -> Result<ConformanceScenarioReport, CliError> {
    let check = check::run(path)?;
    let scenario = Scenario::from_file(path)?;
    let outcome = runner::run(&scenario)?;
    Ok(ConformanceScenarioReport {
        path: path.to_path_buf(),
        scenario_name: check.scenario_name,
        validation_label: check.validation_label,
        final_step: outcome.final_step,
        final_time_s: outcome.final_time_s,
        stop_label: outcome.stop_reason.label().to_owned(),
    })
}
