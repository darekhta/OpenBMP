//! `openbmp campaign ...` run-data planning and execution commands.

use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};

use openbmp_rundata::campaign::{
    CampaignError, CampaignManifest, CampaignOverlayMode, CampaignRunSpec, CampaignShard,
    CampaignVerdictStatus, CampaignVerdictSummary, aggregate_campaign_verdicts,
    expand_campaign_plan, partition_campaign_runs,
};
use openbmp_rundata::trend::{
    CODE_ORDINAL_LABEL_KEY, FAMILY_LABEL_KEY, TrendMetricSpec, ci_gate_failures,
    evaluate_trend_store, generate_scoreboard_markdown, generate_trend_store_from_index,
};
use openbmp_rundata::verdict::VerdictReport;
use openbmp_rundata::{GateResult, RunIndex, RunLabel, RunRecord};
use openbmp_scenario::Scenario;
use rayon::ThreadPoolBuilder;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use toml_edit::{DocumentMut, Item, Table};

use crate::CliError;
use crate::commands::run::{self as run_command, OutputOverrides, RunDataOptions};

/// Report returned after writing a deterministic campaign plan.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CampaignPlanCliReport {
    /// Input manifest path.
    pub manifest_path: PathBuf,
    /// Campaign id from the manifest.
    pub campaign_id: String,
    /// Number of expanded runs.
    pub run_count: usize,
    /// Plan JSON path.
    pub plan_json: PathBuf,
    /// Optional shard JSON path.
    pub shards_json: Option<PathBuf>,
    /// Worker count used for shard JSON.
    pub worker_count: Option<u64>,
}

/// Report returned after executing a campaign.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct CampaignRunCliReport {
    /// Artifact schema id.
    pub schema_version: String,
    /// Campaign id from the manifest.
    pub campaign_id: String,
    /// Number of executed runs.
    pub run_count: usize,
    /// Plan JSON path.
    pub plan_json: PathBuf,
    /// Run index JSONL path.
    pub run_index_jsonl: PathBuf,
    /// Optional run index Parquet path.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub run_index_parquet: Option<PathBuf>,
    /// Summary JSON path.
    pub summary_json: PathBuf,
    /// Per-run execution summaries.
    pub runs: Vec<CampaignRunEntryReport>,
    /// Aggregate verdict summary when the manifest declared verdict rules.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub verdict_summary: Option<CampaignVerdictSummary>,
    /// Trend artifacts when the manifest declared trend specs.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub trend_summary: Option<CampaignTrendCliReport>,
    /// Stable campaign-level CI failure strings from verdict and trend gates.
    pub ci_failures: Vec<String>,
}

/// Trend artifacts written by campaign execution.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct CampaignTrendCliReport {
    /// Trend-store JSON path.
    pub trend_store_json: PathBuf,
    /// Drift-report JSON path.
    pub drift_report_json: PathBuf,
    /// Markdown scoreboard path.
    pub scoreboard_md: PathBuf,
    /// Stable CI failure strings for fail-on-drift reports.
    pub ci_failures: Vec<String>,
}

/// Summary for one executed campaign run.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct CampaignRunEntryReport {
    /// Stable run key from the expanded campaign plan.
    pub run_key: String,
    /// Scenario id from the manifest.
    pub scenario_id: String,
    /// Deterministic seed applied to the parsed scenario before execution.
    pub seed: u64,
    /// Final simulation step.
    pub final_step: u64,
    /// Final simulation time, seconds.
    pub final_time_s: f64,
    /// Runner stop label.
    pub stop_label: String,
    /// Run-data record id.
    pub run_id: String,
    /// Run-data record SHA-256 digest.
    pub record_sha256_hex: String,
    /// Optional per-run verdict JSON path.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub verdict_json: Option<PathBuf>,
    /// Whether the per-run verdict report passed, when present.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub verdict_passed: Option<bool>,
}

/// Expand a campaign manifest and write deterministic planning artifacts.
///
/// # Errors
///
/// Returns [`CliError`] when the manifest cannot be read, parsed, expanded, or
/// written.
pub fn plan(
    manifest_path: &Path,
    output_plan_json: &Path,
    output_shards_json: Option<&Path>,
    workers: u64,
) -> Result<CampaignPlanCliReport, CliError> {
    let manifest_text = fs::read_to_string(manifest_path).map_err(|source| CliError::Io {
        path: manifest_path.to_path_buf(),
        source,
    })?;
    let manifest = CampaignManifest::from_toml_str(&manifest_text).map_err(campaign_error)?;
    let plan = expand_campaign_plan(&manifest).map_err(campaign_error)?;

    ensure_parent_dir(output_plan_json)?;
    let plan_bytes = plan.to_json_bytes().map_err(campaign_error)?;
    fs::write(output_plan_json, plan_bytes).map_err(|source| CliError::Io {
        path: output_plan_json.to_path_buf(),
        source,
    })?;

    let shards_json = if let Some(path) = output_shards_json {
        let shards = partition_campaign_runs(&plan, workers).map_err(campaign_error)?;
        ensure_parent_dir(path)?;
        let bytes = shards_json_bytes(&shards, path)?;
        fs::write(path, bytes).map_err(|source| CliError::Io {
            path: path.to_path_buf(),
            source,
        })?;
        Some(path.to_path_buf())
    } else {
        None
    };

    Ok(CampaignPlanCliReport {
        manifest_path: manifest_path.to_path_buf(),
        campaign_id: plan.campaign_id,
        run_count: plan.runs.len(),
        plan_json: output_plan_json.to_path_buf(),
        shards_json,
        worker_count: output_shards_json.map(|_| workers),
    })
}

/// Execute a campaign manifest.
///
/// # Errors
///
/// Returns [`CliError`] when the manifest cannot be planned, any run fails, or
/// any deterministic artifact cannot be written.
pub fn run(
    manifest_path: &Path,
    output_dir: &Path,
    run_code_id: &str,
    code_ordinal: u64,
    write_index_parquet: bool,
    workers: u64,
) -> Result<CampaignRunCliReport, CliError> {
    let worker_count = validate_worker_count(workers)?;
    let manifest_text = fs::read_to_string(manifest_path).map_err(|source| CliError::Io {
        path: manifest_path.to_path_buf(),
        source,
    })?;
    let manifest = CampaignManifest::from_toml_str(&manifest_text).map_err(campaign_error)?;
    let plan = expand_campaign_plan(&manifest).map_err(campaign_error)?;
    let manifest_dir = manifest_path.parent().unwrap_or_else(|| Path::new("."));
    let overlay_files = overlay_file_map(&manifest, manifest_dir);

    fs::create_dir_all(output_dir).map_err(|source| CliError::Io {
        path: output_dir.to_path_buf(),
        source,
    })?;
    let plan_json = output_dir.join("plan.json");
    fs::write(&plan_json, plan.to_json_bytes().map_err(campaign_error)?).map_err(|source| {
        CliError::Io {
            path: plan_json.clone(),
            source,
        }
    })?;

    let run_index_jsonl = output_dir.join("run-index.jsonl");
    if run_index_jsonl.exists() {
        fs::remove_file(&run_index_jsonl).map_err(|source| CliError::Io {
            path: run_index_jsonl.clone(),
            source,
        })?;
    }
    let verdict_rules = manifest
        .verdict_rules
        .as_deref()
        .map(|path| resolve_manifest_path(manifest_dir, path));
    let trend_specs = manifest
        .trend_specs
        .as_deref()
        .map(|path| read_trend_specs(&resolve_manifest_path(manifest_dir, path)))
        .transpose()?;
    let context = CampaignRunContext {
        manifest_dir,
        output_dir,
        verdict_rules: verdict_rules.as_deref(),
        overlay_files: &overlay_files,
        run_code_id,
        code_ordinal,
    };

    let executed_runs = execute_campaign_runs(&plan.runs, &context, worker_count)?;
    let mut index = RunIndex::new();
    for executed in &executed_runs {
        index.append(executed.record.clone())?;
    }
    fs::write(&run_index_jsonl, index.to_jsonl_bytes()?).map_err(|source| CliError::Io {
        path: run_index_jsonl.clone(),
        source,
    })?;

    let run_index_parquet = if write_index_parquet {
        let parquet_path = output_dir.join("run-index.parquet");
        ensure_parent_dir(&parquet_path)?;
        let file = fs::File::create(&parquet_path).map_err(|source| CliError::Io {
            path: parquet_path.clone(),
            source,
        })?;
        index.write_parquet(file)?;
        Some(parquet_path)
    } else {
        None
    };

    let verdict_reports = executed_runs
        .iter()
        .filter_map(|run| run.verdict_report.clone())
        .collect::<Vec<_>>();
    let verdict_summary = if verdict_reports.is_empty() {
        None
    } else {
        Some(aggregate_campaign_verdicts(&verdict_reports).map_err(campaign_error)?)
    };

    let trend_summary = if let Some(specs) = trend_specs {
        Some(write_campaign_trends(
            output_dir,
            &run_index_jsonl,
            specs.as_slice(),
        )?)
    } else {
        None
    };
    let ci_failures = campaign_ci_failures(verdict_summary.as_ref(), trend_summary.as_ref());

    let summary_json = output_dir.join("summary.json");
    let run_reports = executed_runs
        .into_iter()
        .map(|executed| executed.report)
        .collect::<Vec<_>>();
    let report = CampaignRunCliReport {
        schema_version: "openbmp.rundata.campaign_run.v1".to_owned(),
        campaign_id: plan.campaign_id,
        run_count: run_reports.len(),
        plan_json,
        run_index_jsonl,
        run_index_parquet,
        summary_json: summary_json.clone(),
        runs: run_reports,
        verdict_summary,
        trend_summary,
        ci_failures,
    };
    write_json(&summary_json, &report)?;
    Ok(report)
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CampaignTrendSpecDocument {
    metrics: Vec<TrendMetricSpec>,
}

fn read_trend_specs(path: &Path) -> Result<Vec<TrendMetricSpec>, CliError> {
    let text = fs::read_to_string(path).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    let document: CampaignTrendSpecDocument =
        toml::from_str(&text).map_err(|source| CliError::RunDataConfig {
            summary: format!("campaign trend specs `{}`: {source}", path.display()),
        })?;
    Ok(document.metrics)
}

fn write_campaign_trends(
    output_dir: &Path,
    run_index_jsonl: &Path,
    specs: &[TrendMetricSpec],
) -> Result<CampaignTrendCliReport, CliError> {
    let index_bytes = fs::read(run_index_jsonl).map_err(|source| CliError::Io {
        path: run_index_jsonl.to_path_buf(),
        source,
    })?;
    let index = RunIndex::from_jsonl_bytes(&index_bytes)?;
    let store = generate_trend_store_from_index(&index, specs).map_err(trend_error)?;
    let reports = evaluate_trend_store(&store);
    let ci_failures = ci_gate_failures(&reports);

    let trend_store_json = output_dir.join("trend-store.json");
    fs::write(
        &trend_store_json,
        store.to_json_bytes().map_err(trend_error)?,
    )
    .map_err(|source| CliError::Io {
        path: trend_store_json.clone(),
        source,
    })?;

    let drift_report_json = output_dir.join("drift-report.json");
    write_json(&drift_report_json, &reports)?;

    let scoreboard_md = output_dir.join("trend-scoreboard.md");
    fs::write(&scoreboard_md, generate_scoreboard_markdown(&reports)).map_err(|source| {
        CliError::Io {
            path: scoreboard_md.clone(),
            source,
        }
    })?;

    Ok(CampaignTrendCliReport {
        trend_store_json,
        drift_report_json,
        scoreboard_md,
        ci_failures,
    })
}

fn campaign_ci_failures(
    verdict_summary: Option<&CampaignVerdictSummary>,
    trend_summary: Option<&CampaignTrendCliReport>,
) -> Vec<String> {
    let mut failures = Vec::new();
    if let Some(summary) = verdict_summary
        && summary.status == CampaignVerdictStatus::Fail
    {
        failures.push(format!(
            "campaign verdict summary failed: failed_report_count={} report_count={}",
            summary.failed_report_count, summary.report_count,
        ));
    }
    if let Some(trend) = trend_summary {
        failures.extend(trend.ci_failures.iter().cloned());
    }
    failures
}

fn validate_worker_count(workers: u64) -> Result<usize, CliError> {
    if workers == 0 {
        return Err(CliError::RunDataConfig {
            summary: "campaign run worker_count must be positive".to_owned(),
        });
    }
    usize::try_from(workers).map_err(|_| CliError::RunDataConfig {
        summary: format!("campaign run worker_count {workers} does not fit this platform"),
    })
}

fn execute_campaign_runs(
    runs: &[CampaignRunSpec],
    context: &CampaignRunContext<'_>,
    worker_count: usize,
) -> Result<Vec<ExecutedCampaignRun>, CliError> {
    let pool = ThreadPoolBuilder::new()
        .num_threads(worker_count)
        .build()
        .map_err(|source| CliError::RunDataConfig {
            summary: format!("campaign worker pool: {source}"),
        })?;
    pool.install(|| {
        runs.par_iter()
            .map(|spec| execute_run_spec(spec, context))
            .collect()
    })
}

struct ExecutedCampaignRun {
    report: CampaignRunEntryReport,
    record: RunRecord,
    verdict_report: Option<VerdictReport>,
}

struct CampaignRunContext<'a> {
    manifest_dir: &'a Path,
    output_dir: &'a Path,
    verdict_rules: Option<&'a Path>,
    overlay_files: &'a BTreeMap<String, ResolvedOverlayFile>,
    run_code_id: &'a str,
    code_ordinal: u64,
}

struct ResolvedOverlayFile {
    path: PathBuf,
    mode: CampaignOverlayMode,
}

fn execute_run_spec(
    spec: &CampaignRunSpec,
    context: &CampaignRunContext<'_>,
) -> Result<ExecutedCampaignRun, CliError> {
    let scenario_path = resolve_manifest_path(context.manifest_dir, &spec.scenario_path);
    let run_dir = context
        .output_dir
        .join("runs")
        .join(sanitize_path_component(&spec.run_key));
    fs::create_dir_all(&run_dir).map_err(|source| CliError::Io {
        path: run_dir.clone(),
        source,
    })?;
    let (scenario_artifact_path, mut scenario) =
        load_run_scenario(&scenario_path, spec, &run_dir, context.overlay_files)?;
    scenario.document.time.seed = spec.seed;

    let overrides = OutputOverrides::new(
        Some(run_dir.join("telemetry.csv")),
        Some(run_dir.join("telemetry.json")),
        Some(run_dir.join("telemetry.parquet")),
    );
    let local_run_index_jsonl = run_dir.join("run-index.jsonl");
    if local_run_index_jsonl.exists() {
        fs::remove_file(&local_run_index_jsonl).map_err(|source| CliError::Io {
            path: local_run_index_jsonl.clone(),
            source,
        })?;
    }
    let verdict_json = context.verdict_rules.map(|_| run_dir.join("verdict.json"));
    let mut run_data = RunDataOptions::new(
        Some(local_run_index_jsonl.clone()),
        None,
        context.run_code_id.to_owned(),
        context.verdict_rules.map(Path::to_path_buf),
        verdict_json.clone(),
        None,
    );
    run_data.labels = run_labels(spec, context.code_ordinal)?;
    run_data.gates = vec![GateResult::new("campaign-run", true, "run completed")?];

    let report = run_command::run_loaded_with_options(
        &scenario_artifact_path,
        scenario,
        &overrides,
        &run_data,
    )?;
    let Some(record) = report.run_record else {
        return Err(CliError::RunDataConfig {
            summary: format!(
                "campaign run `{}` did not produce a run record",
                spec.run_key
            ),
        });
    };
    let verdict_passed = report.verdict_report.map(|verdict| verdict.passed);
    let local_index = read_single_record_index(&local_run_index_jsonl)?;
    let verdict_report = read_optional_verdict_report(verdict_json.as_deref())?;
    Ok(ExecutedCampaignRun {
        report: CampaignRunEntryReport {
            run_key: spec.run_key.clone(),
            scenario_id: spec.scenario_id.clone(),
            seed: spec.seed,
            final_step: report.final_step,
            final_time_s: report.final_time_s,
            stop_label: report.stop_label,
            run_id: record.run_id,
            record_sha256_hex: record.record_sha256_hex,
            verdict_json,
            verdict_passed,
        },
        record: local_index,
        verdict_report,
    })
}

fn read_single_record_index(path: &Path) -> Result<RunRecord, CliError> {
    let bytes = fs::read(path).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    let index = RunIndex::from_jsonl_bytes(&bytes)?;
    let [record] = index.records() else {
        return Err(CliError::RunDataConfig {
            summary: format!(
                "campaign run local index `{}` must contain exactly one record",
                path.display(),
            ),
        });
    };
    Ok(record.clone())
}

fn read_optional_verdict_report(path: Option<&Path>) -> Result<Option<VerdictReport>, CliError> {
    let Some(path) = path else {
        return Ok(None);
    };
    let bytes = fs::read(path).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    serde_json::from_slice::<VerdictReport>(&bytes)
        .map(Some)
        .map_err(|source| CliError::RunDataCampaignJson {
            path: path.to_path_buf(),
            source,
        })
}

fn load_run_scenario(
    scenario_path: &Path,
    spec: &CampaignRunSpec,
    run_dir: &Path,
    overlay_files: &BTreeMap<String, ResolvedOverlayFile>,
) -> Result<(PathBuf, Scenario), CliError> {
    let Some(overlay_id) = &spec.overlay else {
        return Ok((
            scenario_path.to_path_buf(),
            Scenario::from_file(scenario_path)?,
        ));
    };
    let Some(overlay_file) = overlay_files.get(overlay_id) else {
        return Err(CliError::RunDataConfig {
            summary: format!("campaign run overlay `{overlay_id}` has no [[overlay_files]] path"),
        });
    };
    let base = fs::read_to_string(scenario_path).map_err(|source| CliError::Io {
        path: scenario_path.to_path_buf(),
        source,
    })?;
    let overlay = fs::read_to_string(&overlay_file.path).map_err(|source| CliError::Io {
        path: overlay_file.path.clone(),
        source,
    })?;
    let combined = apply_overlay_toml(&base, &overlay, overlay_file.mode, &overlay_file.path)?;
    let scenario_artifact_path = run_dir.join("scenario.toml");
    fs::write(&scenario_artifact_path, combined.as_bytes()).map_err(|source| CliError::Io {
        path: scenario_artifact_path.clone(),
        source,
    })?;
    let source_dir = scenario_path.parent().map(Path::to_path_buf);
    let scenario = Scenario::from_toml_str_with_source_dir(&combined, source_dir)?;
    Ok((scenario_artifact_path, scenario))
}

fn run_labels(spec: &CampaignRunSpec, code_ordinal: u64) -> Result<Vec<RunLabel>, CliError> {
    let mut labels = spec
        .labels
        .iter()
        .map(|label| RunLabel::new(label.key.clone(), label.value.clone()))
        .collect::<Result<Vec<_>, _>>()?;
    labels.push(RunLabel::new("run_key", spec.run_key.clone())?);
    labels.push(RunLabel::new(FAMILY_LABEL_KEY, spec.scenario_id.clone())?);
    labels.push(RunLabel::new(
        CODE_ORDINAL_LABEL_KEY,
        code_ordinal.to_string(),
    )?);
    Ok(labels)
}

fn overlay_file_map(
    manifest: &CampaignManifest,
    manifest_dir: &Path,
) -> BTreeMap<String, ResolvedOverlayFile> {
    manifest
        .overlay_files
        .iter()
        .map(|overlay| {
            (
                overlay.id.clone(),
                ResolvedOverlayFile {
                    path: resolve_manifest_path(manifest_dir, &overlay.path),
                    mode: overlay.mode,
                },
            )
        })
        .collect()
}

fn shards_json_bytes(shards: &[CampaignShard], path: &Path) -> Result<Vec<u8>, CliError> {
    serde_json::to_vec(shards).map_err(|source| CliError::RunDataCampaignJson {
        path: path.to_path_buf(),
        source,
    })
}

fn campaign_error(error: CampaignError) -> CliError {
    CliError::RunDataConfig {
        summary: format!("campaign manifest: {error}"),
    }
}

fn trend_error(error: openbmp_rundata::trend::TrendError) -> CliError {
    CliError::RunDataConfig {
        summary: format!("campaign trend: {error}"),
    }
}

fn resolve_manifest_path(manifest_dir: &Path, path: &str) -> PathBuf {
    let path = Path::new(path);
    if path.is_absolute() {
        path.to_path_buf()
    } else {
        manifest_dir.join(path)
    }
}

fn sanitize_path_component(value: &str) -> String {
    let sanitized = value
        .chars()
        .map(|ch| match ch {
            'a'..='z' | 'A'..='Z' | '0'..='9' | '.' | '-' | '_' => ch,
            _ => '_',
        })
        .collect::<String>();
    if sanitized.is_empty() {
        "run".to_owned()
    } else {
        sanitized
    }
}

fn append_overlay_toml(base_scenario: &str, overlay_toml: &str) -> String {
    let mut out = String::with_capacity(base_scenario.len() + overlay_toml.len() + 2);
    out.push_str(base_scenario.trim_end());
    out.push_str("\n\n");
    out.push_str(overlay_toml.trim_start());
    if !out.ends_with('\n') {
        out.push('\n');
    }
    out
}

fn apply_overlay_toml(
    base_scenario: &str,
    overlay_toml: &str,
    mode: CampaignOverlayMode,
    overlay_path: &Path,
) -> Result<String, CliError> {
    match mode {
        CampaignOverlayMode::Append => Ok(append_overlay_toml(base_scenario, overlay_toml)),
        CampaignOverlayMode::Merge => merge_overlay_toml(base_scenario, overlay_toml, overlay_path),
    }
}

fn merge_overlay_toml(
    base_scenario: &str,
    overlay_toml: &str,
    overlay_path: &Path,
) -> Result<String, CliError> {
    let mut base: DocumentMut =
        base_scenario
            .parse()
            .map_err(|source| CliError::RunDataConfig {
                summary: format!(
                    "campaign overlay merge `{}` base scenario TOML: {source}",
                    overlay_path.display()
                ),
            })?;
    let overlay: DocumentMut = overlay_toml
        .parse()
        .map_err(|source| CliError::RunDataConfig {
            summary: format!(
                "campaign overlay merge `{}` overlay TOML: {source}",
                overlay_path.display()
            ),
        })?;
    merge_overlay_table(base.as_table_mut(), overlay.as_table());
    let mut out = base.to_string();
    if !out.ends_with('\n') {
        out.push('\n');
    }
    Ok(out)
}

fn merge_overlay_table(base: &mut Table, overlay: &Table) {
    for (key, overlay_item) in overlay.iter() {
        if let Some(base_item) = base.get_mut(key) {
            merge_overlay_item(base_item, overlay_item);
        } else {
            base.insert(key, overlay_item.clone());
        }
    }
}

fn merge_overlay_item(base: &mut Item, overlay: &Item) {
    match overlay {
        Item::Table(overlay_table) => {
            if let Some(base_table) = base.as_table_mut() {
                merge_overlay_table(base_table, overlay_table);
            } else {
                *base = overlay.clone();
            }
        }
        Item::ArrayOfTables(overlay_tables) => {
            if let Some(base_tables) = base.as_array_of_tables_mut() {
                for table in overlay_tables {
                    base_tables.push(table.clone());
                }
            } else {
                *base = overlay.clone();
            }
        }
        _ => {
            *base = overlay.clone();
        }
    }
}

fn write_json<T: Serialize>(path: &Path, value: &T) -> Result<(), CliError> {
    ensure_parent_dir(path)?;
    let bytes = serde_json::to_vec(value).map_err(|source| CliError::RunDataCampaignJson {
        path: path.to_path_buf(),
        source,
    })?;
    fs::write(path, bytes).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })
}

fn ensure_parent_dir(path: &Path) -> Result<(), CliError> {
    if let Some(parent) = path.parent()
        && !parent.as_os_str().is_empty()
    {
        fs::create_dir_all(parent).map_err(|source| CliError::Io {
            path: parent.to_path_buf(),
            source,
        })?;
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use openbmp_rundata::campaign::CampaignPlan;
    use std::path::{Path, PathBuf};
    use tempfile::Builder;

    const MANIFEST: &str = r#"
campaign_id = "cli-campaign"
seeds = [2, 1, 2]
overlays = ["hot", "cold", "hot"]

[[scenarios]]
id = "drop"
path = "scenarios/drop.toml"
"#;

    const VERDICT_RULES: &str = r#"
[[rules]]
kind = "limit"
id = "position-final"
channel = "position_z_m"
metric = "final"
min = 500.0
max = 520.0
"#;

    const FAILING_VERDICT_RULES: &str = r#"
[[rules]]
kind = "limit"
id = "position-final"
channel = "position_z_m"
metric = "final"
min = 900.0
"#;

    const TREND_SPECS: &str = r#"
[[metrics]]
id = "drop-position-final"
family_id = "drop"
channel = "position_z_m"
metric = "final"
fail_on_drift = true

[metrics.detector]
kind = "page_hinkley"
warmup = 1
delta = 0.0
lambda = 1000.0
alpha = 1.0
direction = "upward"
"#;

    fn workspace_root() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .and_then(Path::parent)
            .map(Path::to_path_buf)
            .expect("workspace root")
    }

    #[test]
    fn campaign_plan_writes_plan_and_shards() {
        let temp = Builder::new()
            .prefix("openbmp_campaign_plan")
            .tempdir()
            .unwrap();
        let manifest = temp.path().join("campaign.toml");
        let plan_json = temp.path().join("out/plan.json");
        let shards_json = temp.path().join("out/shards.json");
        fs::write(&manifest, MANIFEST).unwrap();

        let report = plan(&manifest, &plan_json, Some(&shards_json), 2).unwrap();

        assert_eq!(report.campaign_id, "cli-campaign");
        assert_eq!(report.run_count, 4);
        assert_eq!(report.worker_count, Some(2));
        assert!(plan_json.exists());
        assert!(shards_json.exists());

        let plan = CampaignPlan::from_json_bytes(&fs::read(plan_json).unwrap()).unwrap();
        let shards: Vec<CampaignShard> =
            serde_json::from_slice(&fs::read(shards_json).unwrap()).unwrap();
        let mut planned = plan
            .runs
            .iter()
            .map(|run| run.run_key.clone())
            .collect::<Vec<_>>();
        let mut sharded = shards
            .into_iter()
            .flat_map(|shard| shard.runs)
            .map(|run| run.run_key)
            .collect::<Vec<_>>();
        planned.sort();
        sharded.sort();
        assert_eq!(planned, sharded);
    }

    #[test]
    fn campaign_plan_rejects_zero_workers_for_shards() {
        let temp = Builder::new()
            .prefix("openbmp_campaign_plan_bad_workers")
            .tempdir()
            .unwrap();
        let manifest = temp.path().join("campaign.toml");
        let plan_json = temp.path().join("plan.json");
        let shards_json = temp.path().join("shards.json");
        fs::write(&manifest, MANIFEST).unwrap();

        let err = plan(&manifest, &plan_json, Some(&shards_json), 0).unwrap_err();

        assert!(matches!(err, CliError::RunDataConfig { .. }));
        assert!(
            err.to_string().contains("worker_count"),
            "unexpected error: {err}",
        );
    }

    #[test]
    fn campaign_run_rejects_zero_workers() {
        let temp = Builder::new()
            .prefix("openbmp_campaign_run_bad_workers")
            .tempdir()
            .unwrap();
        let manifest = temp.path().join("campaign.toml");
        let output = temp.path().join("out");

        let err = run(&manifest, &output, "test-code", 0, false, 0).unwrap_err();

        assert!(matches!(err, CliError::RunDataConfig { .. }));
        assert!(
            err.to_string().contains("worker_count"),
            "unexpected error: {err}",
        );
    }

    #[test]
    fn campaign_run_executes_no_overlay_manifest_and_aggregates_verdicts() {
        let temp = Builder::new()
            .prefix("openbmp_campaign_run")
            .tempdir()
            .unwrap();
        let scenario =
            workspace_root().join("scenarios/analytic-toy/constant-acceleration-drop.toml");
        let verdict = temp.path().join("verdict.toml");
        let manifest = temp.path().join("campaign.toml");
        let output = temp.path().join("out");
        fs::write(&verdict, VERDICT_RULES).unwrap();
        fs::write(
            &manifest,
            format!(
                "campaign_id = \"cli-run\"\nseeds = [2, 1]\nverdict_rules = '{}'\n\n[[scenarios]]\nid = \"drop\"\npath = '{}'\n",
                verdict.display(),
                scenario.display(),
            ),
        )
        .unwrap();

        let report = run(&manifest, &output, "test-code", 7, true, 1).unwrap();

        assert_eq!(report.campaign_id, "cli-run");
        assert_eq!(report.run_count, 2);
        assert!(report.run_index_jsonl.exists());
        assert!(report.run_index_parquet.as_ref().unwrap().exists());
        assert!(report.summary_json.exists());
        let verdict_summary = report.verdict_summary.expect("verdict summary");
        assert_eq!(verdict_summary.report_count, 2);
        assert_eq!(verdict_summary.failed_report_count, 0);
        assert!(report.ci_failures.is_empty());
        for run in report.runs {
            assert_eq!(run.verdict_passed, Some(true));
            assert!(run.verdict_json.expect("verdict path").exists());
        }
    }

    #[test]
    fn campaign_run_records_ci_failure_for_failed_verdict_summary() {
        let temp = Builder::new()
            .prefix("openbmp_campaign_run_ci_fail")
            .tempdir()
            .unwrap();
        let scenario =
            workspace_root().join("scenarios/analytic-toy/constant-acceleration-drop.toml");
        let verdict = temp.path().join("verdict.toml");
        let manifest = temp.path().join("campaign.toml");
        let output = temp.path().join("out");
        fs::write(&verdict, FAILING_VERDICT_RULES).unwrap();
        fs::write(
            &manifest,
            format!(
                "campaign_id = \"cli-ci-fail\"\nseeds = [1]\nverdict_rules = '{}'\n\n[[scenarios]]\nid = \"drop\"\npath = '{}'\n",
                verdict.display(),
                scenario.display(),
            ),
        )
        .unwrap();

        let report = run(&manifest, &output, "test-code", 7, false, 1).unwrap();

        let verdict_summary = report.verdict_summary.as_ref().expect("verdict summary");
        assert_eq!(verdict_summary.status, CampaignVerdictStatus::Fail);
        assert_eq!(report.ci_failures.len(), 1);
        assert!(report.ci_failures[0].contains("campaign verdict summary failed"));
        let summary: CampaignRunCliReport =
            serde_json::from_slice(&fs::read(&report.summary_json).unwrap()).unwrap();
        assert_eq!(summary.ci_failures, report.ci_failures);
    }

    #[test]
    fn campaign_run_artifacts_are_byte_identical_across_worker_counts() {
        let temp = Builder::new()
            .prefix("openbmp_campaign_run_workers")
            .tempdir()
            .unwrap();
        let scenario =
            workspace_root().join("scenarios/analytic-toy/constant-acceleration-drop.toml");
        let verdict = temp.path().join("verdict.toml");
        let trend = temp.path().join("trend.toml");
        let manifest = temp.path().join("campaign.toml");
        let output = temp.path().join("out");
        fs::write(&verdict, VERDICT_RULES).unwrap();
        fs::write(&trend, TREND_SPECS).unwrap();
        fs::write(
            &manifest,
            format!(
                "campaign_id = \"cli-worker-run\"\nseeds = [2, 1]\nverdict_rules = '{}'\ntrend_specs = '{}'\n\n[[scenarios]]\nid = \"drop\"\npath = '{}'\n",
                verdict.display(),
                trend.display(),
                scenario.display(),
            ),
        )
        .unwrap();

        let baseline = run(&manifest, &output, "test-code", 9, true, 1).unwrap();
        let baseline_bytes = campaign_artifact_bytes(&output, &baseline);
        for worker_count in [4, 16] {
            let report = run(&manifest, &output, "test-code", 9, true, worker_count).unwrap();
            let candidate_bytes = campaign_artifact_bytes(&output, &report);
            assert_eq!(
                baseline_bytes.keys().collect::<Vec<_>>(),
                candidate_bytes.keys().collect::<Vec<_>>()
            );
            for (path, bytes) in &baseline_bytes {
                assert_eq!(
                    bytes, &candidate_bytes[path],
                    "artifact changed at {worker_count} workers: {path}",
                );
            }
        }
    }

    #[test]
    fn campaign_run_executes_overlay_file_manifest_and_records_materialized_scenario() {
        let temp = Builder::new()
            .prefix("openbmp_campaign_run_overlay_file")
            .tempdir()
            .unwrap();
        let scenario =
            workspace_root().join("scenarios/analytic-toy/constant-acceleration-drop.toml");
        let verdict = temp.path().join("verdict.toml");
        let overlay = temp.path().join("wind-overlay.toml");
        let manifest = temp.path().join("campaign.toml");
        let output = temp.path().join("out");
        fs::write(&verdict, VERDICT_RULES).unwrap();
        fs::write(
            &overlay,
            "[wind]\nkind = \"constant\"\nwind_ned_m_s = [1.0, 0.0, 0.0]\n",
        )
        .unwrap();
        fs::write(
            &manifest,
            format!(
                "campaign_id = \"cli-overlay-run\"\nseeds = [1]\noverlays = [\"wind\"]\nverdict_rules = '{}'\n\n[[overlay_files]]\nid = \"wind\"\npath = '{}'\n\n[[scenarios]]\nid = \"drop\"\npath = '{}'\n",
                verdict.display(),
                overlay.display(),
                scenario.display(),
            ),
        )
        .unwrap();

        let report = run(&manifest, &output, "test-code", 11, false, 1).unwrap();

        assert_eq!(report.run_count, 1);
        let run_entry = report.runs.first().expect("run entry");
        let scenario_artifact = output
            .join("runs")
            .join(sanitize_path_component(&run_entry.run_key))
            .join("scenario.toml");
        assert!(scenario_artifact.exists());
        assert!(
            fs::read_to_string(&scenario_artifact)
                .unwrap()
                .contains("wind_ned_m_s = [1.0, 0.0, 0.0]")
        );
        let index = openbmp_rundata::RunIndex::from_jsonl_bytes(
            &fs::read(&report.run_index_jsonl).unwrap(),
        )
        .unwrap();
        let record = index.records().first().expect("run record");
        let scenario_ref = record
            .artifacts
            .iter()
            .find(|artifact| artifact.role == "scenario")
            .expect("scenario artifact");
        assert_eq!(scenario_ref.path, scenario_artifact.display().to_string());
        assert_eq!(record.scenario_sha256_hex, scenario_ref.sha256_hex);
        assert!(
            record
                .labels
                .iter()
                .any(|label| { label.key == "overlay" && label.value == "wind" })
        );
        assert!(
            record
                .labels
                .iter()
                .any(|label| { label.key == CODE_ORDINAL_LABEL_KEY && label.value == "11" })
        );
    }

    #[test]
    fn merge_overlay_toml_recursively_merges_tables_and_appends_array_tables() {
        let base = r#"
[time]
start_s = 0.0
stop_s = 10.0
dt_s = 1.0

[[scenario_script.events]]
id = "base"
"#;
        let overlay = r#"
[time]
stop_s = 5.0

[[scenario_script.events]]
id = "overlay"
"#;

        let merged = merge_overlay_toml(base, overlay, Path::new("overlay.toml")).unwrap();
        let document: toml::Value = toml::from_str(&merged).unwrap();

        assert_eq!(document["time"]["start_s"].as_float(), Some(0.0));
        assert_eq!(document["time"]["stop_s"].as_float(), Some(5.0));
        assert_eq!(document["time"]["dt_s"].as_float(), Some(1.0));
        let events = document["scenario_script"]["events"]
            .as_array()
            .expect("events");
        assert_eq!(events.len(), 2);
        assert_eq!(events[0]["id"].as_str(), Some("base"));
        assert_eq!(events[1]["id"].as_str(), Some("overlay"));
    }

    #[test]
    fn campaign_run_executes_merge_overlay_file_manifest_and_records_materialized_scenario() {
        let temp = Builder::new()
            .prefix("openbmp_campaign_run_merge_overlay_file")
            .tempdir()
            .unwrap();
        let source_scenario =
            workspace_root().join("scenarios/analytic-toy/constant-acceleration-drop.toml");
        let scenario = temp.path().join("base-with-wind.toml");
        let overlay = temp.path().join("wind-overlay.toml");
        let manifest = temp.path().join("campaign.toml");
        let output = temp.path().join("out");
        let base = fs::read_to_string(&source_scenario).unwrap();
        fs::write(
            &scenario,
            format!(
                "{}\n\n[wind]\nkind = \"constant\"\nwind_ned_m_s = [0.0, 0.0, 0.0]\n",
                base.trim_end()
            ),
        )
        .unwrap();
        fs::write(
            &overlay,
            "[wind]\nkind = \"constant\"\nwind_ned_m_s = [2.0, 0.0, 0.0]\n",
        )
        .unwrap();
        fs::write(
            &manifest,
            format!(
                "campaign_id = \"cli-merge-overlay-run\"\nseeds = [1]\noverlays = [\"wind\"]\n\n[[overlay_files]]\nid = \"wind\"\npath = '{}'\nmode = \"merge\"\n\n[[scenarios]]\nid = \"drop\"\npath = '{}'\n",
                overlay.display(),
                scenario.display(),
            ),
        )
        .unwrap();

        let report = run(&manifest, &output, "test-code", 12, false, 1).unwrap();

        assert_eq!(report.run_count, 1);
        let run_entry = report.runs.first().expect("run entry");
        let scenario_artifact = output
            .join("runs")
            .join(sanitize_path_component(&run_entry.run_key))
            .join("scenario.toml");
        let materialized = fs::read_to_string(scenario_artifact).unwrap();
        assert_eq!(materialized.matches("[wind]").count(), 1);
        assert!(materialized.contains("wind_ned_m_s = [2.0, 0.0, 0.0]"));
        assert!(!materialized.contains("wind_ned_m_s = [0.0, 0.0, 0.0]"));
    }

    #[test]
    fn campaign_run_rejects_invalid_merge_overlay_toml() {
        let temp = Builder::new()
            .prefix("openbmp_campaign_run_bad_merge_overlay")
            .tempdir()
            .unwrap();
        let scenario =
            workspace_root().join("scenarios/analytic-toy/constant-acceleration-drop.toml");
        let overlay = temp.path().join("bad-overlay.toml");
        let manifest = temp.path().join("campaign.toml");
        let output = temp.path().join("out");
        fs::write(&overlay, "[wind\n").unwrap();
        fs::write(
            &manifest,
            format!(
                "campaign_id = \"cli-bad-merge-overlay-run\"\nseeds = [1]\noverlays = [\"wind\"]\n\n[[overlay_files]]\nid = \"wind\"\npath = '{}'\nmode = \"merge\"\n\n[[scenarios]]\nid = \"drop\"\npath = '{}'\n",
                overlay.display(),
                scenario.display(),
            ),
        )
        .unwrap();

        let err = run(&manifest, &output, "test-code", 12, false, 1).unwrap_err();

        assert!(matches!(err, CliError::RunDataConfig { .. }));
        assert!(
            err.to_string().contains("campaign overlay merge")
                && err.to_string().contains("overlay TOML"),
            "unexpected error: {err}",
        );
    }

    #[test]
    fn campaign_run_writes_trend_artifacts_from_manifest_specs() {
        let temp = Builder::new()
            .prefix("openbmp_campaign_run_trend")
            .tempdir()
            .unwrap();
        let scenario =
            workspace_root().join("scenarios/analytic-toy/constant-acceleration-drop.toml");
        let trend = temp.path().join("trend.toml");
        let manifest = temp.path().join("campaign.toml");
        let output = temp.path().join("out");
        fs::write(&trend, TREND_SPECS).unwrap();
        fs::write(
            &manifest,
            format!(
                "campaign_id = \"cli-trend-run\"\nseeds = [1]\ntrend_specs = '{}'\n\n[[scenarios]]\nid = \"drop\"\npath = '{}'\n",
                trend.display(),
                scenario.display(),
            ),
        )
        .unwrap();

        let report = run(&manifest, &output, "trend-code", 5, false, 1).unwrap();
        let trend_summary = report.trend_summary.expect("trend summary");

        assert!(trend_summary.trend_store_json.exists());
        assert!(trend_summary.drift_report_json.exists());
        assert!(trend_summary.scoreboard_md.exists());
        assert!(trend_summary.ci_failures.is_empty());
        assert!(report.ci_failures.is_empty());
        let scoreboard = fs::read_to_string(trend_summary.scoreboard_md).unwrap();
        assert!(scoreboard.contains("drop | position_z_m.final"));
        let store = openbmp_rundata::trend::TrendStore::from_json_bytes(
            &fs::read(trend_summary.trend_store_json).unwrap(),
        )
        .unwrap();
        assert_eq!(store.series.len(), 1);
        assert_eq!(store.series[0].points[0].ordinal, 5);
    }

    #[test]
    fn campaign_run_rejects_overlay_without_file() {
        let temp = Builder::new()
            .prefix("openbmp_campaign_run_overlay")
            .tempdir()
            .unwrap();
        let manifest = temp.path().join("campaign.toml");
        let output = temp.path().join("out");
        fs::write(&manifest, MANIFEST).unwrap();

        let err = run(&manifest, &output, "test-code", 0, false, 1).unwrap_err();

        assert!(matches!(err, CliError::RunDataConfig { .. }));
        assert!(
            err.to_string().contains("has no [[overlay_files]] path"),
            "unexpected error: {err}",
        );
    }

    fn campaign_artifact_bytes(
        output: &Path,
        report: &CampaignRunCliReport,
    ) -> BTreeMap<String, Vec<u8>> {
        let mut artifacts = BTreeMap::new();
        read_artifact(output, &mut artifacts, "plan.json");
        read_artifact(output, &mut artifacts, "run-index.jsonl");
        read_artifact(output, &mut artifacts, "run-index.parquet");
        read_artifact(output, &mut artifacts, "summary.json");
        read_artifact(output, &mut artifacts, "trend-store.json");
        read_artifact(output, &mut artifacts, "drift-report.json");
        read_artifact(output, &mut artifacts, "trend-scoreboard.md");
        for run in &report.runs {
            let base = format!("runs/{}", sanitize_path_component(&run.run_key));
            read_artifact(output, &mut artifacts, &format!("{base}/telemetry.csv"));
            read_artifact(output, &mut artifacts, &format!("{base}/telemetry.json"));
            read_artifact(output, &mut artifacts, &format!("{base}/telemetry.parquet"));
            read_artifact(output, &mut artifacts, &format!("{base}/run-index.jsonl"));
            read_artifact(output, &mut artifacts, &format!("{base}/verdict.json"));
            read_artifact(output, &mut artifacts, &format!("{base}/scenario.toml"));
        }
        artifacts
    }

    fn read_artifact(output: &Path, artifacts: &mut BTreeMap<String, Vec<u8>>, relative: &str) {
        let path = output.join(relative);
        if path.exists() {
            artifacts.insert(relative.to_owned(), fs::read(path).unwrap());
        }
    }
}
