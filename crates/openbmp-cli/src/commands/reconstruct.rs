//! Offline reconstruction and filter-consistency evidence commands.

use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use openbmp_core::{ChannelId, StepIndex};
use openbmp_runner::sil::{FcObservation, SilMonitor};
use openbmp_scenario::Scenario;
use openbmp_sensors::SensorTruth;
use openbmp_telemetry::{TelemetryRow, TelemetryTable, TelemetryValue, TelemetryValueKind};
use openbmp_testkit::reconstruction::{
    ConsistencyReport, SyntheticReconstructionReport, chi_square_95_bounds, consistency_report,
    synthetic_linear_reconstruction_report,
};

use crate::CliError;
use crate::cli::TrajectoryTolerancePack;

/// CLI report for the synthetic linear-Gaussian reconstruction fixture.
#[derive(Clone, Debug, PartialEq)]
pub struct SyntheticLinearReconstructionCliReport {
    /// Testkit reconstruction evidence.
    pub reconstruction: SyntheticReconstructionReport,
    /// Evidence TOML path, when requested and written.
    pub output_toml: Option<PathBuf>,
}

/// NIS consistency summary for one FC measurement family.
#[derive(Clone, Debug, PartialEq)]
pub struct FcNisConsistencyReport {
    /// Chi-square consistency report.
    pub report: ConsistencyReport,
    /// Arithmetic mean of the observed chi-square statistics.
    pub mean_chi2: f64,
    /// Maximum observed chi-square statistic.
    pub max_chi2: f64,
}

/// NEES consistency summary for one FC estimate block.
#[derive(Clone, Debug, PartialEq)]
pub struct FcNeesConsistencyReport {
    /// Chi-square consistency report.
    pub report: ConsistencyReport,
    /// Arithmetic mean of the observed chi-square statistics.
    pub mean_chi2: f64,
    /// Maximum observed chi-square statistic.
    pub max_chi2: f64,
}

/// CLI report for a monitored FC scenario observation run.
#[derive(Clone, Debug, PartialEq)]
pub struct FcObservationConsistencyCliReport {
    /// Scenario path that was observed.
    pub scenario_path: PathBuf,
    /// Number of kernel ticks seen by the monitor.
    pub ticks_observed: u64,
    /// Number of ticks carrying an estimator-status snapshot.
    pub estimator_samples: u64,
    /// Ticks where the estimator reported a rejected innovation.
    pub innovation_rejected_ticks: u64,
    /// Ticks where the estimator reported attitude under-observability.
    pub attitude_under_observable_ticks: u64,
    /// Maximum reported covariance condition proxy.
    pub max_covariance_condition_proxy: f64,
    /// Per-measurement NIS consistency summaries.
    pub nis_reports: Vec<FcNisConsistencyReport>,
    /// Per-estimate-block NEES consistency summaries.
    pub nees_reports: Vec<FcNeesConsistencyReport>,
    /// Evidence TOML path, when requested and written.
    pub output_toml: Option<PathBuf>,
}

/// CLI report for a deterministic trajectory export.
#[derive(Clone, Debug, PartialEq)]
pub struct TrajectoryExportCliReport {
    /// Scenario path that was run.
    pub scenario_path: PathBuf,
    /// CSV output path.
    pub output_csv: PathBuf,
    /// Number of complete trajectory samples written.
    pub samples: usize,
    /// Final simulation time in seconds.
    pub final_time_s: f64,
    /// Stop reason label from the scenario run.
    pub stop_label: String,
}

/// CLI report for a deterministic trajectory compare mapping export.
#[derive(Clone, Debug, PartialEq)]
pub struct TrajectoryMappingCliReport {
    /// Mapping output path.
    pub output_toml: PathBuf,
    /// Tolerance pack used.
    pub pack: TrajectoryTolerancePack,
    /// Time interpolation tolerance in seconds.
    pub time_tolerance_s: f64,
    /// Position component absolute tolerance in meters.
    pub position_tolerance_m: f64,
    /// Velocity component absolute tolerance in meters per second.
    pub velocity_tolerance_m_s: f64,
    /// Number of metrics written.
    pub metrics: usize,
}

/// CLI report for a LOCAL-only reconstruction workflow manifest.
#[derive(Clone, Debug, PartialEq)]
pub struct LocalWorkflowCliReport {
    /// Workflow manifest output path.
    pub output_toml: PathBuf,
    /// Scenario path used by the workflow.
    pub scenario_path: PathBuf,
    /// Local OpenBMP trajectory CSV path.
    pub trajectory_csv: PathBuf,
    /// Local compare-telemetry mapping TOML path.
    pub mapping_toml: PathBuf,
    /// Local external reference CSV path. This is never read or copied.
    pub external_reference_csv: PathBuf,
    /// Tolerance pack referenced by the workflow.
    pub pack: TrajectoryTolerancePack,
    /// Deterministic local commands recorded in the workflow.
    pub commands: Vec<String>,
}

/// Run the synthetic linear-Gaussian RTS/GN/NEES-NIS fixture.
///
/// # Errors
///
/// Returns [`CliError`] when the fixture fails, the acceptance gate
/// fails, or the optional TOML evidence cannot be written.
pub fn run_synthetic_linear(
    output_toml: Option<&Path>,
) -> Result<SyntheticLinearReconstructionCliReport, CliError> {
    let reconstruction =
        synthetic_linear_reconstruction_report().map_err(|err| CliError::CodeVerification {
            summary: err.to_string(),
        })?;
    let mut report = SyntheticLinearReconstructionCliReport {
        reconstruction,
        output_toml: None,
    };

    if let Some(path) = output_toml {
        write_report_toml(path, &report.reconstruction)?;
        report.output_toml = Some(path.to_path_buf());
    }

    if !report.reconstruction.passed {
        return Err(CliError::CodeVerification {
            summary: format!(
                "synthetic reconstruction failed acceptance: NEES={:.6}, NIS={:.6}, batch_error={:.12e}",
                report.reconstruction.nees.in_bounds_fraction,
                report.reconstruction.nis.in_bounds_fraction,
                report.reconstruction.batch_update_error_norm,
            ),
        });
    }

    Ok(report)
}

/// Run a scenario with the read-only SIL monitor and reduce estimator
/// innovation histories into chi-square NIS summaries.
///
/// # Errors
///
/// Returns [`CliError`] when the scenario fails to load/run, no FC
/// estimator diagnostics are observed, no innovation samples are
/// available, or the optional TOML evidence cannot be written.
pub fn run_observe_fc(
    scenario_path: &Path,
    output_toml: Option<&Path>,
) -> Result<FcObservationConsistencyCliReport, CliError> {
    let scenario = Scenario::from_file(scenario_path)?;
    let mut monitor = FcInnovationMonitor::default();
    openbmp_runner::run_with_monitor(&scenario, &mut monitor)?;
    let mut report = monitor.into_report(scenario_path.to_path_buf())?;

    if let Some(path) = output_toml {
        write_observe_fc_report_toml(path, &report)?;
        report.output_toml = Some(path.to_path_buf());
    }

    Ok(report)
}

/// Run a scenario and export primary-body ECI trajectory samples as CSV.
///
/// # Errors
///
/// Returns [`CliError`] when the scenario fails to load/run, required
/// trajectory channels are absent or non-float, no complete trajectory
/// samples are observed, or the CSV cannot be written.
pub fn run_export_trajectory(
    scenario_path: &Path,
    output_csv: &Path,
) -> Result<TrajectoryExportCliReport, CliError> {
    let scenario = Scenario::from_file(scenario_path)?;
    let outcome = openbmp_runner::run(&scenario)?;
    let samples = write_trajectory_csv(output_csv, &outcome.table)?;
    Ok(TrajectoryExportCliReport {
        scenario_path: scenario_path.to_path_buf(),
        output_csv: output_csv.to_path_buf(),
        samples,
        final_time_s: outcome.final_time_s,
        stop_label: outcome.stop_reason.label().to_owned(),
    })
}

/// Write a compare-telemetry mapping for exported trajectory CSVs.
///
/// # Errors
///
/// Returns [`CliError`] when the mapping cannot be written.
pub fn run_export_trajectory_mapping(
    output_toml: &Path,
    pack: TrajectoryTolerancePack,
) -> Result<TrajectoryMappingCliReport, CliError> {
    let tolerances = TrajectoryMappingTolerances::for_pack(pack);
    write_trajectory_mapping_toml(output_toml, tolerances)?;
    Ok(TrajectoryMappingCliReport {
        output_toml: output_toml.to_path_buf(),
        pack,
        time_tolerance_s: tolerances.time_tolerance_s,
        position_tolerance_m: tolerances.position_tolerance_m,
        velocity_tolerance_m_s: tolerances.velocity_tolerance_m_s,
        metrics: 6,
    })
}

/// Write a LOCAL-only pseudo-flight/code-to-code reconstruction workflow.
///
/// The manifest records how to export an OpenBMP trajectory, write the matching
/// compare-telemetry mapping, and compare against a local external CSV. The
/// external reference path is recorded only; it is not read, copied, or
/// generated by this command.
///
/// # Errors
///
/// Returns [`CliError`] when the workflow TOML cannot be written.
pub fn run_local_workflow(
    scenario_path: &Path,
    output_toml: &Path,
    trajectory_csv: &Path,
    mapping_toml: &Path,
    external_reference_csv: &Path,
    pack: TrajectoryTolerancePack,
) -> Result<LocalWorkflowCliReport, CliError> {
    let commands = local_workflow_commands(
        scenario_path,
        trajectory_csv,
        mapping_toml,
        external_reference_csv,
        pack,
    );
    let report = LocalWorkflowCliReport {
        output_toml: output_toml.to_path_buf(),
        scenario_path: scenario_path.to_path_buf(),
        trajectory_csv: trajectory_csv.to_path_buf(),
        mapping_toml: mapping_toml.to_path_buf(),
        external_reference_csv: external_reference_csv.to_path_buf(),
        pack,
        commands,
    };
    write_local_workflow_toml(output_toml, &report)?;
    Ok(report)
}

fn write_report_toml(path: &Path, report: &SyntheticReconstructionReport) -> Result<(), CliError> {
    ensure_parent_dir(path)?;
    let mut out = String::new();
    let _ = writeln!(out, "[reconstruction]");
    let _ = writeln!(out, "case = \"synthetic_linear_gaussian\"");
    let _ = writeln!(out, "method = \"rts_smoother_and_batch_gauss_newton\"");
    let _ = writeln!(out, "passed = {}", report.passed);
    let _ = writeln!(
        out,
        "smoothed_states = {}",
        toml_float_array(&report.smoothed_states)
    );
    let _ = writeln!(
        out,
        "batch_update = {}",
        toml_float_array(&report.batch_update)
    );
    let _ = writeln!(
        out,
        "batch_update_error_norm = {:.17e}",
        report.batch_update_error_norm
    );
    write_consistency_table(&mut out, "nees", &report.nees);
    write_consistency_table(&mut out, "nis", &report.nis);

    fs::write(path, out).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })
}

fn write_local_workflow_toml(path: &Path, report: &LocalWorkflowCliReport) -> Result<(), CliError> {
    ensure_parent_dir(path)?;
    let mut out = String::new();
    let _ = writeln!(out, "[local_workflow]");
    let _ = writeln!(out, "kind = \"bet_code_to_code_local\"");
    let _ = writeln!(out, "local_only = true");
    let _ = writeln!(out, "committed_external_telemetry = false");
    let _ = writeln!(
        out,
        "scenario = {}",
        toml_path_string(&report.scenario_path)
    );
    let _ = writeln!(
        out,
        "openbmp_trajectory_csv = {}",
        toml_path_string(&report.trajectory_csv)
    );
    let _ = writeln!(
        out,
        "mapping_toml = {}",
        toml_path_string(&report.mapping_toml)
    );
    let _ = writeln!(
        out,
        "external_reference_csv = {}",
        toml_path_string(&report.external_reference_csv)
    );
    let _ = writeln!(
        out,
        "tolerance_pack = {}",
        toml_string(trajectory_pack_cli_name(report.pack))
    );
    let _ = writeln!(out, "commands = [");
    for command in &report.commands {
        let _ = writeln!(out, "  {},", toml_string(command));
    }
    let _ = writeln!(out, "]");
    let _ = writeln!(out);
    let _ = writeln!(out, "[local_workflow.guardrails]");
    let _ = writeln!(out, "external_reference_is_local_only = true");
    let _ = writeln!(out, "external_reference_is_not_read_by_this_command = true");
    let _ = writeln!(out, "external_reference_is_not_copied_into_repo = true");
    let _ = writeln!(out, "not_a_flight_or_targeting_input = true");
    fs::write(path, out).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })
}

fn local_workflow_commands(
    scenario_path: &Path,
    trajectory_csv: &Path,
    mapping_toml: &Path,
    external_reference_csv: &Path,
    pack: TrajectoryTolerancePack,
) -> Vec<String> {
    vec![
        format!(
            "openbmp reconstruct export-trajectory {} --output-csv {}",
            shell_arg(scenario_path),
            shell_arg(trajectory_csv)
        ),
        format!(
            "openbmp reconstruct export-trajectory-mapping --output-toml {} --pack {}",
            shell_arg(mapping_toml),
            trajectory_pack_cli_name(pack)
        ),
        format!(
            "openbmp compare-telemetry {} {} --mapping {}",
            shell_arg(scenario_path),
            shell_arg(external_reference_csv),
            shell_arg(mapping_toml)
        ),
    ]
}

fn trajectory_pack_cli_name(pack: TrajectoryTolerancePack) -> &'static str {
    match pack {
        TrajectoryTolerancePack::Strict => "strict",
        TrajectoryTolerancePack::LeoResearch => "leo-research",
    }
}

fn write_trajectory_csv(path: &Path, table: &TelemetryTable) -> Result<usize, CliError> {
    ensure_parent_dir(path)?;
    let channels = TrajectoryChannels::from_table(table)?;
    let mut out = String::from(
        "time_s,step,position_x_eci_m,position_y_eci_m,position_z_eci_m,\
velocity_x_eci_m_s,velocity_y_eci_m_s,velocity_z_eci_m_s\n",
    );
    let mut samples = 0_usize;
    for row in table.rows() {
        if let Some(sample) = trajectory_sample(row, &channels)? {
            samples += 1;
            let _ = writeln!(
                out,
                "{:.17e},{},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e}",
                row.time.as_seconds(),
                row.step.value(),
                sample.position_eci_m[0],
                sample.position_eci_m[1],
                sample.position_eci_m[2],
                sample.velocity_eci_m_s[0],
                sample.velocity_eci_m_s[1],
                sample.velocity_eci_m_s[2],
            );
        }
    }
    if samples == 0 {
        return Err(CliError::CodeVerification {
            summary: "trajectory export found no complete primary ECI position/velocity rows"
                .to_owned(),
        });
    }
    fs::write(path, out).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    Ok(samples)
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct TrajectoryMappingTolerances {
    time_tolerance_s: f64,
    position_tolerance_m: f64,
    velocity_tolerance_m_s: f64,
}

impl TrajectoryMappingTolerances {
    const fn for_pack(pack: TrajectoryTolerancePack) -> Self {
        match pack {
            TrajectoryTolerancePack::Strict => Self {
                time_tolerance_s: 0.0,
                position_tolerance_m: 1.0e-9,
                velocity_tolerance_m_s: 1.0e-12,
            },
            TrajectoryTolerancePack::LeoResearch => Self {
                time_tolerance_s: 0.1,
                position_tolerance_m: 10.0,
                velocity_tolerance_m_s: 0.01,
            },
        }
    }
}

fn write_trajectory_mapping_toml(
    path: &Path,
    tolerances: TrajectoryMappingTolerances,
) -> Result<(), CliError> {
    ensure_parent_dir(path)?;
    let mut out = String::new();
    let _ = writeln!(out, "[reference]");
    let _ = writeln!(out, "time_column = \"time_s\"");
    let _ = writeln!(out);
    let _ = writeln!(out, "[comparison]");
    let _ = writeln!(
        out,
        "time_tolerance_s = {:.17e}",
        tolerances.time_tolerance_s
    );
    write_trajectory_mapping_metric(
        &mut out,
        "position_x_eci_m",
        "position_x_eci_m",
        "position_x_m",
        tolerances.position_tolerance_m,
    );
    write_trajectory_mapping_metric(
        &mut out,
        "position_y_eci_m",
        "position_y_eci_m",
        "position_y_m",
        tolerances.position_tolerance_m,
    );
    write_trajectory_mapping_metric(
        &mut out,
        "position_z_eci_m",
        "position_z_eci_m",
        "position_z_m",
        tolerances.position_tolerance_m,
    );
    write_trajectory_mapping_metric(
        &mut out,
        "velocity_x_eci_m_s",
        "velocity_x_eci_m_s",
        "velocity_x_m_s",
        tolerances.velocity_tolerance_m_s,
    );
    write_trajectory_mapping_metric(
        &mut out,
        "velocity_y_eci_m_s",
        "velocity_y_eci_m_s",
        "velocity_y_m_s",
        tolerances.velocity_tolerance_m_s,
    );
    write_trajectory_mapping_metric(
        &mut out,
        "velocity_z_eci_m_s",
        "velocity_z_eci_m_s",
        "velocity_z_m_s",
        tolerances.velocity_tolerance_m_s,
    );
    fs::write(path, out).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })
}

fn write_trajectory_mapping_metric(
    out: &mut String,
    id: &str,
    reference_column: &str,
    actual_channel: &str,
    tolerance_abs: f64,
) {
    let _ = writeln!(out);
    let _ = writeln!(out, "[[metrics]]");
    let _ = writeln!(out, "id = {}", toml_string(id));
    let _ = writeln!(out, "reference_column = {}", toml_string(reference_column));
    let _ = writeln!(out, "tolerance_abs = {:.17e}", tolerance_abs);
    let _ = writeln!(
        out,
        "actual = {{ kind = \"channel\", name = {} }}",
        toml_string(actual_channel)
    );
}

fn write_observe_fc_report_toml(
    path: &Path,
    report: &FcObservationConsistencyCliReport,
) -> Result<(), CliError> {
    ensure_parent_dir(path)?;
    let mut out = String::new();
    let _ = writeln!(out, "[fc_observation]");
    let _ = writeln!(
        out,
        "scenario = {}",
        toml_string(&report.scenario_path.display().to_string())
    );
    let _ = writeln!(out, "ticks_observed = {}", report.ticks_observed);
    let _ = writeln!(out, "estimator_samples = {}", report.estimator_samples);
    let _ = writeln!(
        out,
        "innovation_rejected_ticks = {}",
        report.innovation_rejected_ticks
    );
    let _ = writeln!(
        out,
        "attitude_under_observable_ticks = {}",
        report.attitude_under_observable_ticks
    );
    let _ = writeln!(
        out,
        "max_covariance_condition_proxy = {:.17e}",
        report.max_covariance_condition_proxy
    );
    let _ = writeln!(out, "nis_report_count = {}", report.nis_reports.len());
    let _ = writeln!(out, "nees_report_count = {}", report.nees_reports.len());
    for nis in &report.nis_reports {
        let _ = writeln!(out);
        let _ = writeln!(out, "[[fc_observation.nis]]");
        let _ = writeln!(out, "label = {}", toml_string(&nis.report.label));
        let _ = writeln!(out, "dof = {}", nis.report.bounds.dof);
        let _ = writeln!(out, "confidence = {:.17e}", nis.report.bounds.confidence);
        let _ = writeln!(out, "lower = {:.17e}", nis.report.bounds.lower);
        let _ = writeln!(out, "upper = {:.17e}", nis.report.bounds.upper);
        let _ = writeln!(out, "samples = {}", nis.report.samples);
        let _ = writeln!(out, "in_bounds = {}", nis.report.in_bounds);
        let _ = writeln!(
            out,
            "in_bounds_fraction = {:.17e}",
            nis.report.in_bounds_fraction
        );
        let _ = writeln!(out, "mean_chi2 = {:.17e}", nis.mean_chi2);
        let _ = writeln!(out, "max_chi2 = {:.17e}", nis.max_chi2);
    }
    for nees in &report.nees_reports {
        let _ = writeln!(out);
        let _ = writeln!(out, "[[fc_observation.nees]]");
        let _ = writeln!(out, "label = {}", toml_string(&nees.report.label));
        let _ = writeln!(out, "dof = {}", nees.report.bounds.dof);
        let _ = writeln!(out, "confidence = {:.17e}", nees.report.bounds.confidence);
        let _ = writeln!(out, "lower = {:.17e}", nees.report.bounds.lower);
        let _ = writeln!(out, "upper = {:.17e}", nees.report.bounds.upper);
        let _ = writeln!(out, "samples = {}", nees.report.samples);
        let _ = writeln!(out, "in_bounds = {}", nees.report.in_bounds);
        let _ = writeln!(
            out,
            "in_bounds_fraction = {:.17e}",
            nees.report.in_bounds_fraction
        );
        let _ = writeln!(out, "mean_chi2 = {:.17e}", nees.mean_chi2);
        let _ = writeln!(out, "max_chi2 = {:.17e}", nees.max_chi2);
    }
    fs::write(path, out).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })
}

fn write_consistency_table(
    out: &mut String,
    key: &str,
    report: &openbmp_testkit::reconstruction::ConsistencyReport,
) {
    let _ = writeln!(out);
    let _ = writeln!(out, "[reconstruction.{key}]");
    let _ = writeln!(out, "dof = {}", report.bounds.dof);
    let _ = writeln!(out, "confidence = {:.17e}", report.bounds.confidence);
    let _ = writeln!(out, "lower = {:.17e}", report.bounds.lower);
    let _ = writeln!(out, "upper = {:.17e}", report.bounds.upper);
    let _ = writeln!(out, "samples = {}", report.samples);
    let _ = writeln!(out, "in_bounds = {}", report.in_bounds);
    let _ = writeln!(
        out,
        "in_bounds_fraction = {:.17e}",
        report.in_bounds_fraction
    );
}

fn ensure_parent_dir(path: &Path) -> Result<(), CliError> {
    if let Some(parent) = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        fs::create_dir_all(parent).map_err(|source| CliError::Io {
            path: parent.to_path_buf(),
            source,
        })?;
    }
    Ok(())
}

fn toml_float_array(values: &[f64]) -> String {
    let mut out = String::from("[");
    for (index, value) in values.iter().enumerate() {
        if index > 0 {
            out.push_str(", ");
        }
        let _ = write!(out, "{value:.17e}");
    }
    out.push(']');
    out
}

fn toml_string(value: &str) -> String {
    let escaped = value.replace('\\', "\\\\").replace('"', "\\\"");
    format!("\"{escaped}\"")
}

fn toml_path_string(path: &Path) -> String {
    toml_string(&path.display().to_string())
}

fn shell_arg(path: &Path) -> String {
    let raw = path.display().to_string();
    if raw
        .chars()
        .all(|ch| ch.is_ascii_alphanumeric() || matches!(ch, '/' | '.' | '_' | '-' | ':'))
    {
        raw
    } else {
        format!("'{}'", raw.replace('\'', "'\\''"))
    }
}

#[derive(Clone, Copy, Debug)]
struct TrajectoryChannels {
    position_x: ChannelId,
    position_y: ChannelId,
    position_z: ChannelId,
    velocity_x: ChannelId,
    velocity_y: ChannelId,
    velocity_z: ChannelId,
}

impl TrajectoryChannels {
    fn from_table(table: &TelemetryTable) -> Result<Self, CliError> {
        Ok(Self {
            position_x: require_float_channel(table, "position_x_m")?,
            position_y: require_float_channel(table, "position_y_m")?,
            position_z: require_float_channel(table, "position_z_m")?,
            velocity_x: require_float_channel(table, "velocity_x_m_s")?,
            velocity_y: require_float_channel(table, "velocity_y_m_s")?,
            velocity_z: require_float_channel(table, "velocity_z_m_s")?,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct TrajectorySample {
    position_eci_m: [f64; 3],
    velocity_eci_m_s: [f64; 3],
}

fn require_float_channel(
    table: &TelemetryTable,
    name: &'static str,
) -> Result<ChannelId, CliError> {
    let channel = table
        .schema()
        .channels()
        .iter()
        .find(|channel| channel.name == name)
        .ok_or_else(|| CliError::CodeVerification {
            summary: format!("trajectory export requires telemetry channel {name:?}"),
        })?;
    if channel.value_kind != TelemetryValueKind::Float64 {
        return Err(CliError::CodeVerification {
            summary: format!("trajectory export requires telemetry channel {name:?} to be float64"),
        });
    }
    Ok(channel.id)
}

fn trajectory_sample(
    row: &TelemetryRow,
    channels: &TrajectoryChannels,
) -> Result<Option<TrajectorySample>, CliError> {
    let Some(position_x) = optional_f64(row, channels.position_x)? else {
        return Ok(None);
    };
    let Some(position_y) = optional_f64(row, channels.position_y)? else {
        return Ok(None);
    };
    let Some(position_z) = optional_f64(row, channels.position_z)? else {
        return Ok(None);
    };
    let Some(velocity_x) = optional_f64(row, channels.velocity_x)? else {
        return Ok(None);
    };
    let Some(velocity_y) = optional_f64(row, channels.velocity_y)? else {
        return Ok(None);
    };
    let Some(velocity_z) = optional_f64(row, channels.velocity_z)? else {
        return Ok(None);
    };
    Ok(Some(TrajectorySample {
        position_eci_m: [position_x, position_y, position_z],
        velocity_eci_m_s: [velocity_x, velocity_y, velocity_z],
    }))
}

fn optional_f64(row: &TelemetryRow, channel: ChannelId) -> Result<Option<f64>, CliError> {
    match row.get(channel) {
        Some(TelemetryValue::Float64(value)) => Ok(Some(*value)),
        Some(_) => Err(CliError::CodeVerification {
            summary: format!(
                "trajectory export expected float64 row value for channel {}",
                channel.value()
            ),
        }),
        None => Ok(None),
    }
}

#[derive(Clone, Debug, Default)]
struct FcInnovationMonitor {
    ticks_observed: u64,
    estimator_samples: u64,
    innovation_rejected_ticks: u64,
    attitude_under_observable_ticks: u64,
    max_covariance_condition_proxy: f64,
    gnss_nis: Vec<f64>,
    baro_nis: Vec<f64>,
    mag_nis: Vec<f64>,
    position_nees: Vec<f64>,
    velocity_nees: Vec<f64>,
    attitude_nees: Vec<f64>,
}

impl FcInnovationMonitor {
    fn into_report(
        self,
        scenario_path: PathBuf,
    ) -> Result<FcObservationConsistencyCliReport, CliError> {
        if self.ticks_observed == 0 {
            return Err(CliError::CodeVerification {
                summary: "FC observation monitor saw no ticks; scenario may not declare [fc]"
                    .to_owned(),
            });
        }
        if self.estimator_samples == 0 {
            return Err(CliError::CodeVerification {
                summary: "FC observation monitor saw no estimator.status samples".to_owned(),
            });
        }

        let mut nis_reports = Vec::new();
        push_nis_report(&mut nis_reports, "gnss_nis", 6, &self.gnss_nis)?;
        push_nis_report(&mut nis_reports, "baro_nis", 1, &self.baro_nis)?;
        push_nis_report(&mut nis_reports, "mag_nis", 3, &self.mag_nis)?;
        if nis_reports.is_empty() {
            return Err(CliError::CodeVerification {
                summary: "FC observation monitor saw estimator status but no innovation samples"
                    .to_owned(),
            });
        }
        let mut nees_reports = Vec::new();
        push_nees_report(&mut nees_reports, "position_nees", 3, &self.position_nees)?;
        push_nees_report(&mut nees_reports, "velocity_nees", 3, &self.velocity_nees)?;
        push_nees_report(&mut nees_reports, "attitude_nees", 3, &self.attitude_nees)?;
        if nees_reports.is_empty() {
            return Err(CliError::CodeVerification {
                summary: "FC observation monitor saw estimator status but no covariance-backed NEES samples"
                    .to_owned(),
            });
        }

        Ok(FcObservationConsistencyCliReport {
            scenario_path,
            ticks_observed: self.ticks_observed,
            estimator_samples: self.estimator_samples,
            innovation_rejected_ticks: self.innovation_rejected_ticks,
            attitude_under_observable_ticks: self.attitude_under_observable_ticks,
            max_covariance_condition_proxy: self.max_covariance_condition_proxy,
            nis_reports,
            nees_reports,
            output_toml: None,
        })
    }
}

impl SilMonitor for FcInnovationMonitor {
    fn observe(&mut self, _step: StepIndex, truth: &SensorTruth, observation: &FcObservation) {
        self.ticks_observed += 1;
        if let Some(estimator) = observation.estimator {
            self.estimator_samples += 1;
            if estimator.innovation_rejected {
                self.innovation_rejected_ticks += 1;
            }
            if estimator.attitude_under_observable {
                self.attitude_under_observable_ticks += 1;
            }
            if estimator.covariance_condition_proxy.is_finite() {
                self.max_covariance_condition_proxy = self
                    .max_covariance_condition_proxy
                    .max(estimator.covariance_condition_proxy);
            }
            if estimator.gnss_updated_this_tick {
                self.gnss_nis.push(estimator.gnss_chi2);
            }
            if estimator.baro_updated_this_tick {
                self.baro_nis.push(estimator.baro_chi2);
            }
            if estimator.mag_updated_this_tick {
                self.mag_nis.push(estimator.mag_chi2);
            }
            if let Some(position) = observation.position {
                let pos_error = position.position_eci_m - truth.position_eci.vector;
                if let Some(nees) = diagonal_nees(pos_error, estimator.position_variance_eci_m2) {
                    self.position_nees.push(nees);
                }
                let vel_error = position.velocity_eci_m_s - truth.velocity_eci.vector;
                if let Some(nees) = diagonal_nees(vel_error, estimator.velocity_variance_eci_m2_s2)
                {
                    self.velocity_nees.push(nees);
                }
            }
            if let Some(attitude) = observation.attitude {
                let att_error = attitude_error_vector_rad(
                    attitude.q_body_to_eci_xyzw,
                    &truth.attitude_eci_to_body,
                );
                if let Some(nees) = diagonal_nees(att_error, estimator.attitude_variance_rad2) {
                    self.attitude_nees.push(nees);
                }
            }
        }
    }
}

fn push_nis_report(
    reports: &mut Vec<FcNisConsistencyReport>,
    label: &'static str,
    dof: usize,
    samples: &[f64],
) -> Result<(), CliError> {
    if samples.is_empty() {
        return Ok(());
    }
    let bounds = chi_square_95_bounds(dof).map_err(|err| CliError::CodeVerification {
        summary: err.to_string(),
    })?;
    let report =
        consistency_report(label, samples, bounds).map_err(|err| CliError::CodeVerification {
            summary: err.to_string(),
        })?;
    let sum: f64 = samples.iter().sum();
    let mean_chi2 = sum / samples.len() as f64;
    let max_chi2 = samples.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    reports.push(FcNisConsistencyReport {
        report,
        mean_chi2,
        max_chi2,
    });
    Ok(())
}

fn push_nees_report(
    reports: &mut Vec<FcNeesConsistencyReport>,
    label: &'static str,
    dof: usize,
    samples: &[f64],
) -> Result<(), CliError> {
    if samples.is_empty() {
        return Ok(());
    }
    let bounds = chi_square_95_bounds(dof).map_err(|err| CliError::CodeVerification {
        summary: err.to_string(),
    })?;
    let report =
        consistency_report(label, samples, bounds).map_err(|err| CliError::CodeVerification {
            summary: err.to_string(),
        })?;
    let sum: f64 = samples.iter().sum();
    let mean_chi2 = sum / samples.len() as f64;
    let max_chi2 = samples.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    reports.push(FcNeesConsistencyReport {
        report,
        mean_chi2,
        max_chi2,
    });
    Ok(())
}

fn diagonal_nees(error: Vector3<f64>, variance: [f64; 3]) -> Option<f64> {
    if variance
        .iter()
        .all(|value| value.is_finite() && *value > 0.0)
        && error.iter().all(|value| value.is_finite())
    {
        Some(
            error[0] * error[0] / variance[0]
                + error[1] * error[1] / variance[1]
                + error[2] * error[2] / variance[2],
        )
    } else {
        None
    }
}

fn attitude_error_vector_rad(
    q_body_to_eci_xyzw: [f64; 4],
    truth_eci_to_body: &UnitQuaternion<f64>,
) -> Vector3<f64> {
    let [x, y, z, w] = q_body_to_eci_xyzw;
    let q_est = UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z));
    (q_est * truth_eci_to_body).scaled_axis()
}

#[cfg(test)]
mod tests {
    use std::error::Error;

    use crate::commands::compare_telemetry;

    use super::*;

    #[test]
    fn synthetic_linear_reconstruction_passes_and_writes_toml() -> Result<(), Box<dyn Error>> {
        let temp = tempfile::tempdir()?;
        let output = temp.path().join("reconstruction").join("synthetic.toml");

        let report = run_synthetic_linear(Some(&output))?;

        assert!(report.reconstruction.passed);
        assert_eq!(report.output_toml.as_deref(), Some(output.as_path()));
        let text = fs::read_to_string(&output)?;
        let _: toml::Value = toml::from_str(&text)?;
        assert!(text.contains("synthetic_linear_gaussian"));
        assert!(text.contains("[reconstruction.nees]"));
        assert!(text.contains("[reconstruction.nis]"));
        Ok(())
    }

    #[test]
    fn observe_fc_reduces_estimator_innovation_history() -> Result<(), Box<dyn Error>> {
        let temp = tempfile::tempdir()?;
        let output = temp.path().join("observe-fc.toml");
        let scenario = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../scenarios/closed-loop-attitude-hold/scenario.toml");

        let report = run_observe_fc(&scenario, Some(&output))?;

        assert!(report.ticks_observed > 0);
        assert!(report.estimator_samples > 0);
        assert!(!report.nis_reports.is_empty());
        assert!(!report.nees_reports.is_empty());
        assert_eq!(report.output_toml.as_deref(), Some(output.as_path()));
        let text = fs::read_to_string(&output)?;
        let _: toml::Value = toml::from_str(&text)?;
        assert!(text.contains("[[fc_observation.nis]]"));
        assert!(text.contains("[[fc_observation.nees]]"));
        Ok(())
    }

    #[test]
    fn export_trajectory_writes_code_to_code_csv() -> Result<(), Box<dyn Error>> {
        let temp = tempfile::tempdir()?;
        let output = temp.path().join("trajectory.csv");
        let scenario = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../scenarios/closed-loop-attitude-hold/scenario.toml");

        let report = run_export_trajectory(&scenario, &output)?;

        assert!(report.samples > 0);
        assert_eq!(report.output_csv, output);
        let text = fs::read_to_string(&report.output_csv)?;
        assert!(
            text.starts_with("time_s,step,position_x_eci_m,position_y_eci_m,position_z_eci_m,")
        );
        assert_eq!(text.lines().count(), report.samples + 1);
        Ok(())
    }

    #[test]
    fn export_trajectory_mapping_round_trips_with_compare_telemetry() -> Result<(), Box<dyn Error>>
    {
        let temp = tempfile::tempdir()?;
        let csv = temp.path().join("trajectory.csv");
        let mapping = temp.path().join("trajectory-map.toml");
        let scenario = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../scenarios/closed-loop-attitude-hold/scenario.toml");

        let export = run_export_trajectory(&scenario, &csv)?;
        let mapping_report =
            run_export_trajectory_mapping(&mapping, TrajectoryTolerancePack::Strict)?;
        let compare = compare_telemetry::run(&scenario, &csv, &mapping)?;

        assert!(export.samples > 0);
        assert_eq!(mapping_report.metrics, 6);
        assert!(compare.passed(), "{compare:#?}");
        assert_eq!(compare.metrics.len(), 6);
        Ok(())
    }

    #[test]
    fn local_workflow_writes_local_only_manifest_without_copying_reference()
    -> Result<(), Box<dyn Error>> {
        let temp = tempfile::tempdir()?;
        let workflow = temp.path().join("local-bet-workflow.toml");
        let trajectory = temp.path().join("openbmp-trajectory.csv");
        let mapping = temp.path().join("trajectory-map.toml");
        let external = temp.path().join("local external.csv");
        let scenario = Path::new("scenarios/closed-loop-attitude-hold/scenario.toml");

        let report = run_local_workflow(
            scenario,
            &workflow,
            &trajectory,
            &mapping,
            &external,
            TrajectoryTolerancePack::LeoResearch,
        )?;

        assert_eq!(report.output_toml, workflow);
        assert_eq!(report.commands.len(), 3);
        assert!(!trajectory.exists());
        assert!(!mapping.exists());
        assert!(!external.exists());

        let text = fs::read_to_string(&report.output_toml)?;
        let parsed: toml::Value = toml::from_str(&text)?;
        assert_eq!(
            parsed["local_workflow"]["kind"].as_str(),
            Some("bet_code_to_code_local")
        );
        assert_eq!(parsed["local_workflow"]["local_only"].as_bool(), Some(true));
        assert_eq!(
            parsed["local_workflow"]["committed_external_telemetry"].as_bool(),
            Some(false)
        );
        assert_eq!(
            parsed["local_workflow"]["tolerance_pack"].as_str(),
            Some("leo-research")
        );
        assert_eq!(
            parsed["local_workflow"]["guardrails"]["external_reference_is_not_read_by_this_command"]
                .as_bool(),
            Some(true)
        );
        assert!(text.contains("openbmp reconstruct export-trajectory"));
        assert!(text.contains("openbmp compare-telemetry"));
        Ok(())
    }
}
