//! `openbmp run <scenario.toml>` — load, run, write declared outputs.

use std::fs::{File, create_dir_all};
use std::io::BufWriter;
use std::path::{Path, PathBuf};

use openbmp_scenario::Scenario;

use crate::error::CliError;
use crate::runner;

/// Outcome of `openbmp run`, surfaced for snapshot tests and library
/// consumers.
#[derive(Debug)]
pub struct RunReport {
    /// Final step index reached.
    pub final_step: u64,
    /// Final simulation time in seconds.
    pub final_time_s: f64,
    /// Stop reason label (e.g., `"end-time"`).
    pub stop_label: &'static str,
    /// Output paths that were written, sorted by archive type.
    pub written: Vec<PathBuf>,
}

/// Entry point.
///
/// # Errors
///
/// Returns [`CliError`] for any scenario, simulation, telemetry, or IO
/// failure.
pub fn run(scenario_path: &Path) -> Result<RunReport, CliError> {
    let scenario = Scenario::from_file(scenario_path)?;
    let outcome = runner::run(&scenario)?;

    let mut written = Vec::new();
    let outputs = scenario.resolved_paths();
    for (key, path) in outputs {
        if !key.starts_with("telemetry.output.") {
            continue;
        }
        let kind = key.trim_start_matches("telemetry.output.");
        ensure_parent_dir(&path)?;
        let writer = BufWriter::new(open_for_write(&path)?);
        match kind {
            "csv" => outcome.table.write_csv(writer)?,
            "json" => outcome.table.write_json(writer)?,
            "parquet" => outcome.table.write_parquet(writer)?,
            other => {
                return Err(CliError::UnsupportedScenario {
                    what: format!("telemetry.output.{other} export"),
                });
            }
        }
        written.push(path);
    }
    written.sort();

    Ok(RunReport {
        final_step: outcome.final_step,
        final_time_s: outcome.final_time_s,
        stop_label: outcome.stop_reason.label(),
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
