//! `openbmp run <scenario.toml>` — load, run, write declared outputs.
//!
//! Output paths come from the scenario file by default. The
//! `--output-csv`, `--output-json`, and `--output-parquet` flags
//! override the scenario's declared path for that archive type. An
//! override always takes precedence; if a kind has no scenario path
//! and no override, that archive is not written. The scenario schema
//! still requires at least one declared telemetry output before CLI
//! overrides are applied.

use std::fs::{File, create_dir_all};
use std::io::BufWriter;
use std::path::{Path, PathBuf};

use openbmp_runner as runner;
use openbmp_scenario::Scenario;

use crate::error::CliError;

/// Output-path overrides, one per archive kind.
#[derive(Debug, Default, Clone)]
pub struct OutputOverrides {
    /// CSV override.
    pub csv: Option<PathBuf>,
    /// JSON override.
    pub json: Option<PathBuf>,
    /// Parquet override.
    pub parquet: Option<PathBuf>,
}

impl OutputOverrides {
    /// Construct from CLI flags.
    #[must_use]
    pub const fn new(
        csv: Option<PathBuf>,
        json: Option<PathBuf>,
        parquet: Option<PathBuf>,
    ) -> Self {
        Self { csv, json, parquet }
    }
}

/// Outcome of `openbmp run`, surfaced for snapshot tests and library
/// consumers.
#[derive(Debug)]
pub struct RunReport {
    /// Final step index reached.
    pub final_step: u64,
    /// Final simulation time in seconds.
    pub final_time_s: f64,
    /// Stop reason label (e.g., `"end-time"`). Owned because
    /// scenario-declared mission stop actions carry runtime labels
    /// that are not `'static`.
    pub stop_label: String,
    /// Output paths that were written, sorted lexicographically.
    pub written: Vec<PathBuf>,
}

/// Entry point with no overrides — uses the scenario's declared
/// telemetry paths.
///
/// # Errors
///
/// Returns [`CliError`] for any scenario, simulation, telemetry, or IO
/// failure.
pub fn run(scenario_path: &Path) -> Result<RunReport, CliError> {
    run_with_overrides(scenario_path, &OutputOverrides::default())
}

/// Entry point with output overrides applied per archive kind.
///
/// An override replaces the scenario's declared path for that kind.
/// If a kind has no override and no scenario path, that archive is
/// not written.
///
/// # Errors
///
/// Returns [`CliError`] for any scenario, simulation, telemetry, or IO
/// failure.
pub fn run_with_overrides(
    scenario_path: &Path,
    overrides: &OutputOverrides,
) -> Result<RunReport, CliError> {
    let scenario = Scenario::from_file(scenario_path)?;
    let outcome = runner::run(&scenario)?;

    let resolved = scenario.resolved_paths();
    let csv_path = overrides
        .csv
        .clone()
        .or_else(|| resolved.get("telemetry.output.csv").cloned());
    let json_path = overrides
        .json
        .clone()
        .or_else(|| resolved.get("telemetry.output.json").cloned());
    let parquet_path = overrides
        .parquet
        .clone()
        .or_else(|| resolved.get("telemetry.output.parquet").cloned());

    let mut written = Vec::new();
    if let Some(path) = csv_path {
        ensure_parent_dir(&path)?;
        let writer = BufWriter::new(open_for_write(&path)?);
        outcome.table.write_csv(writer)?;
        written.push(path);
    }
    if let Some(path) = json_path {
        ensure_parent_dir(&path)?;
        let writer = BufWriter::new(open_for_write(&path)?);
        outcome.table.write_json(writer)?;
        written.push(path);
    }
    if let Some(path) = parquet_path {
        ensure_parent_dir(&path)?;
        let writer = BufWriter::new(open_for_write(&path)?);
        outcome.table.write_parquet(writer)?;
        written.push(path);
    }
    written.sort();

    Ok(RunReport {
        final_step: outcome.final_step,
        final_time_s: outcome.final_time_s,
        stop_label: outcome.stop_reason.label().to_owned(),
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
