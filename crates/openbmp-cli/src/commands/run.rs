//! `openbmp run <scenario.toml>` — load, run, write declared outputs.
//!
//! Output paths come from the scenario file by default. The
//! `--output-csv`, `--output-json`, and `--output-parquet` flags
//! override the scenario's declared path for that archive type. An
//! override always takes precedence; if a kind has no scenario path
//! and no override, that archive is not written. The scenario schema
//! still requires at least one declared telemetry output before CLI
//! overrides are applied.

use std::io::BufWriter;
use std::path::{Path, PathBuf};

use openbmp_rundata::verdict::VerdictRuleDocument;
use openbmp_rundata::{
    ArtifactRef, GateResult, RunIndex, RunLabel, RunRecord, append_record_jsonl, channel_digests,
};
use openbmp_runner as runner;
use openbmp_scenario::Scenario;

use super::digest::sha256_bytes;
use super::output::{ensure_parent_dir, open_for_write};
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

/// Optional run-data output controls.
#[derive(Debug, Clone)]
pub struct RunDataOptions {
    /// JSONL run index path to append.
    pub jsonl: Option<PathBuf>,
    /// Parquet run-index mirror path to write.
    pub parquet: Option<PathBuf>,
    /// Stable code/config id.
    pub code_id: String,
    /// Verdict-rule TOML document to evaluate after the run.
    pub verdict_rules: Option<PathBuf>,
    /// Verdict report JSON path to write.
    pub verdict_json: Option<PathBuf>,
    /// Optional JSONL run index whose last record is the baseline.
    pub verdict_baseline_jsonl: Option<PathBuf>,
    /// Stable labels attached to the run record.
    pub labels: Vec<RunLabel>,
    /// Gate results attached to the run record.
    pub gates: Vec<GateResult>,
}

impl RunDataOptions {
    /// Construct from CLI flags.
    #[must_use]
    pub fn new(
        jsonl: Option<PathBuf>,
        parquet: Option<PathBuf>,
        code_id: String,
        verdict_rules: Option<PathBuf>,
        verdict_json: Option<PathBuf>,
        verdict_baseline_jsonl: Option<PathBuf>,
    ) -> Self {
        Self {
            jsonl,
            parquet,
            code_id,
            verdict_rules,
            verdict_json,
            verdict_baseline_jsonl,
            labels: Vec::new(),
            gates: Vec::new(),
        }
    }

    const fn disabled() -> Self {
        Self {
            jsonl: None,
            parquet: None,
            code_id: String::new(),
            verdict_rules: None,
            verdict_json: None,
            verdict_baseline_jsonl: None,
            labels: Vec::new(),
            gates: Vec::new(),
        }
    }

    const fn enabled(&self) -> bool {
        self.jsonl.is_some()
            || self.parquet.is_some()
            || self.verdict_rules.is_some()
            || self.verdict_json.is_some()
            || self.verdict_baseline_jsonl.is_some()
    }
}

/// Entry point with an already parsed scenario.
///
/// This preserves the scenario's source-directory context while allowing
/// callers such as campaign execution to set in-memory deterministic run
/// parameters before the runner sees the document.
///
/// # Errors
///
/// Returns [`CliError`] for any scenario, simulation, telemetry, run-data, or
/// IO failure.
pub fn run_loaded_with_options(
    scenario_path: &Path,
    scenario: Scenario,
    overrides: &OutputOverrides,
    run_data: &RunDataOptions,
) -> Result<RunReport, CliError> {
    run_scenario_with_options(scenario_path, scenario, overrides, run_data)
}

/// Run-data record summary written by `openbmp run`.
#[derive(Debug, Clone, Eq, PartialEq)]
pub struct RunRecordReport {
    /// Stable run id.
    pub run_id: String,
    /// SHA-256 digest of the canonical JSONL record line.
    pub record_sha256_hex: String,
}

/// Run-data verdict report summary written by `openbmp run`.
#[derive(Debug, Clone, Eq, PartialEq)]
pub struct RunVerdictReport {
    /// Whether every verdict passed.
    pub passed: bool,
    /// Number of verdict records in the report.
    pub verdict_count: usize,
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
    /// Optional host realtime pacing report.
    pub realtime: Option<runner::rt::RealtimeRunReport>,
    /// Optional FC actuator command-stream digest report.
    pub actuator_stream: Option<runner::determinism::ActuatorStreamReport>,
    /// Optional AFTS forward-containment evidence report.
    pub afts: Option<runner::afts::AftsRunReport>,
    /// Optional run-data record evidence.
    pub run_record: Option<RunRecordReport>,
    /// Optional run-data verdict report evidence.
    pub verdict_report: Option<RunVerdictReport>,
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
    run_with_options(scenario_path, overrides, &RunDataOptions::disabled())
}

/// Entry point with telemetry overrides and optional run-data outputs.
///
/// # Errors
///
/// Returns [`CliError`] for any scenario, simulation, telemetry, run-data, or
/// IO failure.
pub fn run_with_options(
    scenario_path: &Path,
    overrides: &OutputOverrides,
    run_data: &RunDataOptions,
) -> Result<RunReport, CliError> {
    validate_run_data_options(run_data)?;
    let scenario = Scenario::from_file(scenario_path)?;
    run_scenario_with_options(scenario_path, scenario, overrides, run_data)
}

fn run_scenario_with_options(
    scenario_path: &Path,
    scenario: Scenario,
    overrides: &OutputOverrides,
    run_data: &RunDataOptions,
) -> Result<RunReport, CliError> {
    validate_run_data_options(run_data)?;

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
    let mut telemetry_artifacts = Vec::new();
    if let Some(path) = csv_path {
        ensure_parent_dir(&path)?;
        let writer = BufWriter::new(open_for_write(&path)?);
        outcome.table.write_csv(writer)?;
        telemetry_artifacts.push(path.clone());
        written.push(path);
    }
    if let Some(path) = json_path {
        ensure_parent_dir(&path)?;
        let writer = BufWriter::new(open_for_write(&path)?);
        outcome.table.write_json(writer)?;
        telemetry_artifacts.push(path.clone());
        written.push(path);
    }
    if let Some(path) = parquet_path {
        ensure_parent_dir(&path)?;
        let writer = BufWriter::new(open_for_write(&path)?);
        outcome.table.write_parquet(writer)?;
        telemetry_artifacts.push(path.clone());
        written.push(path);
    }

    let run_record = if run_data.enabled() {
        let acoustics_artifacts = write_acoustics_run_data_artifacts(
            &outcome,
            scenario_path,
            scenario.document.time.seed,
            run_data,
            &mut written,
        )?;
        let record = build_run_record(
            scenario_path,
            scenario.document.time.seed,
            &outcome.table,
            &telemetry_artifacts,
            &acoustics_artifacts,
            run_data,
        )?;
        let report = RunRecordReport {
            run_id: record.run_id.clone(),
            record_sha256_hex: record.record_sha256_hex()?,
        };
        let index = write_run_data_indexes(&record, run_data, &mut written)?;
        if let Some(path) = &run_data.parquet {
            ensure_parent_dir(path)?;
            let writer = BufWriter::new(open_for_write(path)?);
            index.write_parquet(writer)?;
            written.push(path.clone());
        }
        Some(report)
    } else {
        None
    };
    let verdict_report = write_verdict_report(run_data, &outcome.table, &mut written)?;
    written.sort();

    Ok(RunReport {
        final_step: outcome.final_step,
        final_time_s: outcome.final_time_s,
        stop_label: outcome.stop_reason.label().to_owned(),
        written,
        realtime: outcome.realtime,
        actuator_stream: outcome.actuator_stream,
        afts: outcome.afts,
        run_record,
        verdict_report,
    })
}

fn validate_run_data_options(run_data: &RunDataOptions) -> Result<(), CliError> {
    match (&run_data.verdict_rules, &run_data.verdict_json) {
        (None, None) => Ok(()),
        (Some(_), Some(_)) => Ok(()),
        (Some(_), None) => Err(CliError::RunDataConfig {
            summary: "--run-verdict-rules requires --run-verdict-json".to_owned(),
        }),
        (None, Some(_)) => Err(CliError::RunDataConfig {
            summary: "--run-verdict-json requires --run-verdict-rules".to_owned(),
        }),
    }
}

fn write_verdict_report(
    run_data: &RunDataOptions,
    table: &openbmp_telemetry::TelemetryTable,
    written: &mut Vec<PathBuf>,
) -> Result<Option<RunVerdictReport>, CliError> {
    let (Some(rules_path), Some(output_path)) = (&run_data.verdict_rules, &run_data.verdict_json)
    else {
        return Ok(None);
    };

    let rules = std::fs::read_to_string(rules_path).map_err(|source| CliError::Io {
        path: rules_path.clone(),
        source,
    })?;
    let rules = VerdictRuleDocument::from_toml_str(&rules)?;
    let baseline = match &run_data.verdict_baseline_jsonl {
        Some(path) => Some(load_latest_baseline_record(path)?),
        None => None,
    };
    let report = rules.evaluate(table, baseline.as_ref());
    let passed = report.passed();
    let verdict_count = report.verdicts.len();
    let bytes = report.to_json_bytes()?;

    ensure_parent_dir(output_path)?;
    std::fs::write(output_path, bytes).map_err(|source| CliError::Io {
        path: output_path.clone(),
        source,
    })?;
    written.push(output_path.clone());

    Ok(Some(RunVerdictReport {
        passed,
        verdict_count,
    }))
}

fn load_latest_baseline_record(path: &Path) -> Result<RunRecord, CliError> {
    let bytes = std::fs::read(path).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    let index = RunIndex::from_jsonl_bytes(&bytes)?;
    let Some(record) = index.records().last() else {
        return Err(CliError::RunDataConfig {
            summary: format!(
                "baseline run index `{}` contains no records",
                path.display()
            ),
        });
    };
    Ok(record.clone())
}

fn build_run_record(
    scenario_path: &Path,
    seed: u64,
    table: &openbmp_telemetry::TelemetryTable,
    written_telemetry: &[PathBuf],
    extra_artifacts: &[ArtifactRef],
    run_data: &RunDataOptions,
) -> Result<RunRecord, CliError> {
    let scenario_sha256_hex = sha256_file(scenario_path)?;
    let mut artifacts = vec![ArtifactRef::new(
        "scenario",
        scenario_path.display().to_string(),
        scenario_sha256_hex.clone(),
    )?];
    for path in written_telemetry {
        artifacts.push(ArtifactRef::new(
            telemetry_artifact_role(path),
            path.display().to_string(),
            sha256_file(path)?,
        )?);
    }
    artifacts.extend(extra_artifacts.iter().cloned());
    RunRecord::new(
        scenario_sha256_hex,
        seed,
        run_data.code_id.clone(),
        channel_digests(table),
        artifacts,
        run_data.labels.clone(),
        run_data.gates.clone(),
    )
    .map_err(CliError::from)
}

fn write_acoustics_run_data_artifacts(
    outcome: &runner::RunOutcome,
    scenario_path: &Path,
    seed: u64,
    run_data: &RunDataOptions,
    written: &mut Vec<PathBuf>,
) -> Result<Vec<ArtifactRef>, CliError> {
    let Some(acoustics) = &outcome.acoustics else {
        return Ok(Vec::new());
    };

    let base_dir = run_data_artifact_root(run_data, scenario_path, seed).join("acoustics");
    let mut artifacts = Vec::new();

    if let Some(ascent) = &acoustics.ascent {
        write_optional_run_data_artifact(
            &base_dir,
            Path::new("ascent/p95_50_environment.csv"),
            "acoustics_ascent_p95_50_environment_csv",
            &ascent.p95_50_environment_csv,
            written,
            &mut artifacts,
        )?;
        write_optional_run_data_artifact(
            &base_dir,
            Path::new("ascent/p95_50_wilks_sizing.csv"),
            "acoustics_ascent_p95_50_wilks_sizing_csv",
            &ascent.p95_50_wilks_sizing_csv,
            written,
            &mut artifacts,
        )?;
    }

    if let Some(fairing) = &acoustics.fairing_internal {
        write_optional_run_data_artifact(
            &base_dir,
            Path::new("fairing_internal/p95_50_environment.csv"),
            "acoustics_fairing_internal_p95_50_environment_csv",
            &fairing.p95_50_environment_csv,
            written,
            &mut artifacts,
        )?;
        write_optional_run_data_artifact(
            &base_dir,
            Path::new("fairing_internal/p95_50_sample_trace.csv"),
            "acoustics_fairing_internal_p95_50_sample_trace_csv",
            &fairing.p95_50_sample_trace_csv,
            written,
            &mut artifacts,
        )?;
        write_optional_run_data_artifact(
            &base_dir,
            Path::new("fairing_internal/p95_50_wilks_sizing.csv"),
            "acoustics_fairing_internal_p95_50_wilks_sizing_csv",
            &fairing.p95_50_wilks_sizing_csv,
            written,
            &mut artifacts,
        )?;
    }

    if let Some(response) = &acoustics.response {
        write_optional_run_data_artifact(
            &base_dir,
            Path::new("response/loads_handoff.csv"),
            "acoustics_response_loads_handoff_csv",
            &response.loads_handoff_csv,
            written,
            &mut artifacts,
        )?;
        write_optional_run_data_artifact(
            &base_dir,
            Path::new("response/p95_50_acceleration_psd.csv"),
            "acoustics_response_p95_50_acceleration_psd_csv",
            &response.p95_50_acceleration_psd_csv,
            written,
            &mut artifacts,
        )?;
        write_optional_run_data_artifact(
            &base_dir,
            Path::new("response/p95_50_modal_force_psd.csv"),
            "acoustics_response_p95_50_modal_force_psd_csv",
            &response.p95_50_modal_force_psd_csv,
            written,
            &mut artifacts,
        )?;
        write_optional_run_data_artifact(
            &base_dir,
            Path::new("response/p95_50_sample_trace.csv"),
            "acoustics_response_p95_50_sample_trace_csv",
            &response.p95_50_sample_trace_csv,
            written,
            &mut artifacts,
        )?;
        write_optional_run_data_artifact(
            &base_dir,
            Path::new("response/p95_50_wilks_sizing.csv"),
            "acoustics_response_p95_50_wilks_sizing_csv",
            &response.p95_50_wilks_sizing_csv,
            written,
            &mut artifacts,
        )?;
    }

    if let Some(structural) = &outcome.structural_acoustics {
        write_optional_run_data_artifact(
            &base_dir,
            Path::new("structural/first_bending_response.csv"),
            "structural_acoustics_first_bending_response_csv",
            &structural.band_response_csv,
            written,
            &mut artifacts,
        )?;
        write_optional_run_data_artifact(
            &base_dir,
            Path::new("structural/first_bending_line_load_recovery.csv"),
            "structural_acoustics_first_bending_line_load_recovery_csv",
            &structural.line_load_recovery_csv,
            written,
            &mut artifacts,
        )?;
    }

    Ok(artifacts)
}

fn write_optional_run_data_artifact(
    base_dir: &Path,
    relative_path: &Path,
    role: &'static str,
    bytes: &[u8],
    written: &mut Vec<PathBuf>,
    artifacts: &mut Vec<ArtifactRef>,
) -> Result<(), CliError> {
    if bytes.is_empty() {
        return Ok(());
    }

    let path = base_dir.join(relative_path);
    ensure_parent_dir(&path)?;
    std::fs::write(&path, bytes).map_err(|source| CliError::Io {
        path: path.clone(),
        source,
    })?;
    artifacts.push(ArtifactRef::new(
        role,
        path.display().to_string(),
        sha256_bytes(bytes),
    )?);
    written.push(path);
    Ok(())
}

fn run_data_artifact_root(run_data: &RunDataOptions, scenario_path: &Path, seed: u64) -> PathBuf {
    let output_dir = run_data
        .jsonl
        .as_deref()
        .or(run_data.parquet.as_deref())
        .or(run_data.verdict_json.as_deref())
        .and_then(Path::parent)
        .filter(|parent| !parent.as_os_str().is_empty())
        .map(Path::to_path_buf)
        .or_else(|| {
            scenario_path
                .parent()
                .filter(|parent| !parent.as_os_str().is_empty())
                .map(Path::to_path_buf)
        })
        .unwrap_or_else(|| PathBuf::from("."));
    let scenario_stem = sanitized_file_stem(scenario_path);
    output_dir
        .join("artifacts")
        .join(format!("{scenario_stem}-seed-{seed:016x}"))
}

fn sanitized_file_stem(path: &Path) -> String {
    let stem = path
        .file_stem()
        .and_then(|stem| stem.to_str())
        .unwrap_or("scenario");
    let sanitized: String = stem
        .chars()
        .map(|ch| match ch {
            'a'..='z' | 'A'..='Z' | '0'..='9' | '.' | '-' | '_' => ch,
            _ => '_',
        })
        .collect();
    if sanitized.is_empty() {
        "scenario".to_owned()
    } else {
        sanitized
    }
}

fn write_run_data_indexes(
    record: &RunRecord,
    run_data: &RunDataOptions,
    written: &mut Vec<PathBuf>,
) -> Result<RunIndex, CliError> {
    if let Some(path) = &run_data.jsonl {
        ensure_parent_dir(path)?;
        let existing = match std::fs::read(path) {
            Ok(bytes) => bytes,
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => Vec::new(),
            Err(source) => {
                return Err(CliError::Io {
                    path: path.clone(),
                    source,
                });
            }
        };
        let updated = append_record_jsonl(&existing, record)?;
        std::fs::write(path, &updated).map_err(|source| CliError::Io {
            path: path.clone(),
            source,
        })?;
        written.push(path.clone());
        return RunIndex::from_jsonl_bytes(&updated).map_err(CliError::from);
    }

    let mut index = RunIndex::new();
    index.append(record.clone())?;
    Ok(index)
}

fn telemetry_artifact_role(path: &Path) -> &'static str {
    match path.extension().and_then(|extension| extension.to_str()) {
        Some("csv") => "telemetry_csv",
        Some("json") => "telemetry_json",
        Some("parquet") => "telemetry_parquet",
        _ => "telemetry",
    }
}

fn sha256_file(path: &Path) -> Result<String, CliError> {
    let bytes = std::fs::read(path).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    Ok(sha256_bytes(&bytes))
}
