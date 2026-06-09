//! `openbmp sil ...` native SIL testbench commands.

use std::fs;
use std::path::{Path, PathBuf};

use openbmp_sil::{
    BusFrameEntry, EvidenceBundle, RunUntilTarget, SignalReadReport, SilError, SilRunReport,
    SilTestbench, capture_bus_frames, read_signal, write_evidence_bundle,
};

use crate::error::CliError;

/// Run a package case through the native SIL API.
///
/// # Errors
///
/// Returns [`CliError`] for package, scenario, runner, evidence, or IO
/// failures.
pub fn run(
    package: &Path,
    case: Option<&str>,
    evidence_dir: Option<&Path>,
) -> Result<(SilRunReport, Option<PathBuf>), CliError> {
    let bench = SilTestbench::load(package)?;
    let report = bench.run(case)?;
    let evidence_path = match evidence_dir {
        Some(path) => Some(write_evidence_bundle(path, &report.evidence)?),
        None => None,
    };
    Ok((report, evidence_path))
}

/// Step a package case for a fixed number of ticks.
///
/// # Errors
///
/// Returns [`CliError`] for package, scenario, runner, evidence, or IO
/// failures.
pub fn step(
    package: &Path,
    case: Option<&str>,
    ticks: u64,
    evidence_dir: Option<&Path>,
) -> Result<(SilRunReport, Option<PathBuf>), CliError> {
    let bench = SilTestbench::load(package)?;
    let report = bench.step(case, ticks)?;
    let evidence_path = match evidence_dir {
        Some(path) => Some(write_evidence_bundle(path, &report.evidence)?),
        None => None,
    };
    Ok((report, evidence_path))
}

/// Run a package case until a selected target.
///
/// # Errors
///
/// Returns [`CliError`] for target, package, scenario, runner, evidence, or IO
/// failures.
pub fn run_until(
    package: &Path,
    case: Option<&str>,
    target: RunUntilTarget,
    evidence_dir: Option<&Path>,
) -> Result<(SilRunReport, Option<PathBuf>), CliError> {
    let bench = SilTestbench::load(package)?;
    let report = bench.run_until(case, target)?;
    let evidence_path = match evidence_dir {
        Some(path) => Some(write_evidence_bundle(path, &report.evidence)?),
        None => None,
    };
    Ok((report, evidence_path))
}

/// Run a package case and read one telemetry channel.
///
/// # Errors
///
/// Returns [`CliError`] for package, runner, or channel failures.
pub fn read_channel(
    package: &Path,
    case: Option<&str>,
    ticks: Option<u64>,
    channel: &str,
    max_samples: usize,
) -> Result<SignalReadReport, CliError> {
    let bench = SilTestbench::load(package)?;
    let report = match ticks {
        Some(ticks) => bench.step(case, ticks)?,
        None => bench.run(case)?,
    };
    Ok(read_signal(&report, channel, max_samples)?)
}

/// Run a package case and capture host-SIL bus/evidence frames.
///
/// # Errors
///
/// Returns [`CliError`] for package, runner, JSON, or IO failures.
pub fn capture_bus(
    package: &Path,
    case: Option<&str>,
    ticks: Option<u64>,
    output: Option<&Path>,
) -> Result<Vec<BusFrameEntry>, CliError> {
    let bench = SilTestbench::load(package)?;
    let report = match ticks {
        Some(ticks) => bench.step(case, ticks)?,
        None => bench.run(case)?,
    };
    let frames = capture_bus_frames(&report);
    if let Some(path) = output {
        let json =
            serde_json::to_string_pretty(&frames).map_err(|source| CliError::DiffReportJson {
                path: path.to_path_buf(),
                source,
            })?;
        fs::write(path, json).map_err(|source| CliError::Io {
            path: path.to_path_buf(),
            source,
        })?;
    }
    Ok(frames)
}

/// Build a run-until target from CLI options.
///
/// # Errors
///
/// Returns [`CliError`] when the option set does not name exactly one target.
pub fn run_until_target(
    time_s: Option<f64>,
    event: Option<String>,
    phase: Option<String>,
) -> Result<RunUntilTarget, CliError> {
    match (time_s, event, phase) {
        (Some(time_s), None, None) => Ok(RunUntilTarget::TimeS(time_s)),
        (None, Some(event), None) => Ok(RunUntilTarget::Event(event)),
        (None, None, Some(phase)) => Ok(RunUntilTarget::Phase(phase)),
        _ => Err(CliError::Sil(SilError::RunUntil {
            summary: "specify exactly one of --time-s, --event, or --phase".to_owned(),
        })),
    }
}

/// Run a package case and always write evidence.
///
/// # Errors
///
/// Returns [`CliError`] for package, scenario, runner, evidence, or IO
/// failures.
pub fn evidence(package: &Path, case: Option<&str>, output: &Path) -> Result<PathBuf, CliError> {
    let (report, _) = run(package, case, None)?;
    Ok(write_evidence_bundle(output, &report.evidence)?)
}

/// Read an evidence manifest.
///
/// # Errors
///
/// Returns [`CliError`] when the manifest cannot be read or parsed.
pub fn verdict(path: &Path) -> Result<EvidenceBundle, CliError> {
    let text = fs::read_to_string(path).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    serde_json::from_str(&text).map_err(|source| CliError::DiffReportJson {
        path: path.to_path_buf(),
        source,
    })
}
