//! `openbmp footprint-mc <scenario.toml>` — run offline footprint
//! Monte-Carlo post-processing and write declared sample/summary files.

use std::fs::{File, create_dir_all};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

use openbmp_runner::{self as runner, RunnerError};
use openbmp_scenario::Scenario;

use crate::error::CliError;

/// Outcome of `openbmp footprint-mc`.
#[derive(Debug)]
pub struct FootprintMonteCarloCliReport {
    /// Requested sample count.
    pub samples_requested: u32,
    /// Successfully propagated sample count.
    pub samples_succeeded: usize,
    /// Failed sample count.
    pub samples_failed: usize,
    /// Output paths that were written, sorted lexicographically.
    pub written: Vec<PathBuf>,
}

/// Run the configured footprint Monte-Carlo analysis.
///
/// # Errors
///
/// Returns [`CliError`] when the scenario lacks
/// `[landing_footprint.monte_carlo]`, the runner rejects the analysis,
/// or writing any declared output fails.
pub fn run(scenario_path: &Path) -> Result<FootprintMonteCarloCliReport, CliError> {
    let scenario = Scenario::from_file(scenario_path)?;
    let configured_samples = scenario
        .document
        .landing_footprint
        .as_ref()
        .and_then(|footprint| footprint.monte_carlo.as_ref())
        .map(|monte_carlo| monte_carlo.samples)
        .ok_or_else(|| {
            CliError::Run(RunnerError::UnsupportedScenario {
                what: "footprint-mc requires [landing_footprint.monte_carlo]".to_owned(),
            })
        })?;
    let report = runner::footprint::landing_footprint_monte_carlo_for_initial_state(&scenario)?
        .ok_or_else(|| {
            CliError::Run(RunnerError::UnsupportedScenario {
                what: "footprint-mc requires [landing_footprint.monte_carlo]".to_owned(),
            })
        })?;

    let resolved = scenario.resolved_paths();
    let mut written = Vec::new();
    if let Some(path) = resolved.get("landing_footprint.monte_carlo.output.samples_csv") {
        ensure_parent_dir(path)?;
        let writer = BufWriter::new(open_for_write(path)?);
        report.samples.write_csv(writer)?;
        written.push(path.clone());
    }
    if let Some(path) = resolved.get("landing_footprint.monte_carlo.output.samples_parquet") {
        ensure_parent_dir(path)?;
        let writer = BufWriter::new(open_for_write(path)?);
        report.samples.write_parquet(writer)?;
        written.push(path.clone());
    }
    if let Some(path) = resolved.get("landing_footprint.monte_carlo.output.summary_toml") {
        ensure_parent_dir(path)?;
        let mut writer = BufWriter::new(open_for_write(path)?);
        writer
            .write_all(report.summary_toml.as_bytes())
            .map_err(|source| CliError::Io {
                path: path.clone(),
                source,
            })?;
        written.push(path.clone());
    }
    written.sort();

    Ok(FootprintMonteCarloCliReport {
        samples_requested: configured_samples,
        samples_succeeded: report.result.samples.len(),
        samples_failed: report.result.failures.len(),
        written,
    })
}

fn ensure_parent_dir(path: &Path) -> Result<(), CliError> {
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

fn open_for_write(path: &Path) -> Result<File, CliError> {
    File::create(path).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })
}
