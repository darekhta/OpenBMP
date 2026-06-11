//! `openbmp footprint-mc <scenario.toml>` — run offline footprint
//! Monte-Carlo post-processing and write declared sample/summary files.

use std::fs::{File, create_dir_all};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

use openbmp_mc::{CampaignCredibilityReport, NestedScalarAnalysisReport};
use openbmp_runner::{self as runner, RunnerError};
use openbmp_scenario::Scenario;
use sha2::{Digest, Sha256};

use crate::commands::mc::{self, McCredibilityOptions};
use crate::error::CliError;

/// Outcome of `openbmp footprint-mc`.
#[derive(Debug)]
pub struct FootprintMonteCarloCliReport {
    /// Requested sample count.
    pub samples_requested: u32,
    /// Completed sample count, including both successes and failures.
    pub samples_completed: usize,
    /// Successfully propagated sample count.
    pub samples_succeeded: usize,
    /// Failed sample count.
    pub samples_failed: usize,
    /// Whether all requested samples completed and final outputs were written.
    pub complete: bool,
    /// Output paths that were written, sorted lexicographically.
    pub written: Vec<PathBuf>,
    /// Optional campaign UQ/credibility evidence.
    pub credibility: Option<CampaignCredibilityReport>,
    /// Markdown report path, when written.
    pub credibility_report_md: Option<PathBuf>,
}

/// Run the configured footprint Monte-Carlo analysis.
///
/// # Errors
///
/// Returns [`CliError`] when the scenario lacks
/// `[landing_footprint.monte_carlo]`, the runner rejects the analysis,
/// or writing any declared output fails.
pub fn run(
    scenario_path: &Path,
    checkpoint_json: Option<&Path>,
    max_new_samples: Option<u32>,
    toolchain_profile: &str,
    credibility: Option<McCredibilityOptions<'_>>,
) -> Result<FootprintMonteCarloCliReport, CliError> {
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
    let mut written = Vec::new();
    let report = if let Some(checkpoint_json) = checkpoint_json {
        let scenario_sha256 = sha256_hex(&read_file(scenario_path)?);
        let checkpoint =
            runner::footprint::landing_footprint_monte_carlo_for_initial_state_checkpointed(
                &scenario,
                runner::footprint::FootprintMonteCarloCheckpointOptions {
                    checkpoint_json,
                    scenario_sha256: &scenario_sha256,
                    toolchain_profile,
                    max_new_samples: max_new_samples.unwrap_or(configured_samples),
                },
            )?
            .ok_or_else(|| {
                CliError::Run(RunnerError::UnsupportedScenario {
                    what: "footprint-mc requires [landing_footprint.monte_carlo]".to_owned(),
                })
            })?;
        written.push(checkpoint_json.to_path_buf());
        let complete = checkpoint.report.is_some();
        if let Some(report) = checkpoint.report {
            Some((
                report,
                checkpoint.completed as usize,
                checkpoint.sample_count,
                complete,
            ))
        } else {
            return Ok(FootprintMonteCarloCliReport {
                samples_requested: checkpoint.sample_count,
                samples_completed: checkpoint.completed as usize,
                samples_succeeded: 0,
                samples_failed: 0,
                complete,
                written,
                credibility: None,
                credibility_report_md: None,
            });
        }
    } else {
        let report = runner::footprint::landing_footprint_monte_carlo_for_initial_state(&scenario)?
            .ok_or_else(|| {
                CliError::Run(RunnerError::UnsupportedScenario {
                    what: "footprint-mc requires [landing_footprint.monte_carlo]".to_owned(),
                })
            })?;
        Some((
            report,
            configured_samples as usize,
            configured_samples,
            true,
        ))
    };
    let Some((report, completed, requested, complete)) = report else {
        return Err(CliError::Run(RunnerError::UnsupportedScenario {
            what: "footprint-mc requires [landing_footprint.monte_carlo]".to_owned(),
        }));
    };
    let manifest_credibility = if credibility.is_none() {
        scenario_credibility_options(&scenario)?
    } else {
        None
    };
    let manifest_credibility = manifest_credibility
        .as_ref()
        .map(|(uq_toml, floor, report_md)| McCredibilityOptions {
            uq_toml,
            floor: *floor,
            report_md: report_md.as_deref(),
        });
    let credibility = credibility
        .or(manifest_credibility)
        .map(mc::evaluate_credibility_options)
        .transpose()?;
    let (credibility, credibility_report_md) = match credibility {
        Some((report, report_path)) => (Some(report), report_path),
        None => (None, None),
    };
    if let Some(path) = &credibility_report_md {
        written.push(path.clone());
    }

    let resolved = scenario.resolved_paths();
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
    if let Some(nested) = &report.nested
        && let Some(path) = resolved.get("landing_footprint.monte_carlo.nested.pbox_csv")
    {
        ensure_parent_dir(path)?;
        write_pbox_csv(path, &nested.analysis)?;
        written.push(path.clone());
    }
    written.sort();

    Ok(FootprintMonteCarloCliReport {
        samples_requested: requested,
        samples_completed: completed,
        samples_succeeded: report.result.samples.len(),
        samples_failed: report.result.failures.len(),
        complete,
        written,
        credibility,
        credibility_report_md,
    })
}

fn scenario_credibility_options(
    scenario: &Scenario,
) -> Result<Option<(PathBuf, openbmp_uq::CredibilityLevel, Option<PathBuf>)>, CliError> {
    let Some(uq) = scenario
        .document
        .landing_footprint
        .as_ref()
        .and_then(|footprint| footprint.monte_carlo.as_ref())
        .and_then(|monte_carlo| monte_carlo.uq.as_ref())
    else {
        return Ok(None);
    };
    let floor = uq
        .credibility_floor
        .as_deref()
        .map(mc::parse_credibility_floor)
        .transpose()?
        .unwrap_or(openbmp_uq::CredibilityLevel::L0);
    Ok(Some((
        scenario.resolve_path(&uq.budget_toml),
        floor,
        uq.report_md
            .as_ref()
            .map(|path| scenario.resolve_path(path)),
    )))
}

fn read_file(path: &Path) -> Result<Vec<u8>, CliError> {
    std::fs::read(path).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })
}

fn write_pbox_csv(path: &Path, report: &NestedScalarAnalysisReport) -> Result<(), CliError> {
    let mut writer = BufWriter::new(open_for_write(path)?);
    writer
        .write_all(b"support,lower_cdf,upper_cdf\n")
        .map_err(|source| CliError::Io {
            path: path.to_path_buf(),
            source,
        })?;
    for ((support, lower), upper) in report
        .pbox
        .support
        .iter()
        .zip(&report.pbox.lower_cdf)
        .zip(&report.pbox.upper_cdf)
    {
        writeln!(writer, "{support:.17e},{lower:.17e},{upper:.17e}").map_err(|source| {
            CliError::Io {
                path: path.to_path_buf(),
                source,
            }
        })?;
    }
    Ok(())
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut out = String::with_capacity(digest.len() * 2);
    for byte in digest {
        use std::fmt::Write as _;
        let _ = write!(out, "{byte:02x}");
    }
    out
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
