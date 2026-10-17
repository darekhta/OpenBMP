//! Deterministic campaign manifest expansion.
//!
//! The campaign layer parses a declarative scenario/seed/overlay matrix and
//! expands it into stable run specifications. It is the file/planning
//! substrate for a future CLI executor; it does not launch simulations itself.

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::verdict::VerdictReport;
use crate::{RundataError, require_non_empty};

/// Campaign plan schema version.
pub const CAMPAIGN_SCHEMA_VERSION: &str = "openbmp.rundata.campaign.v1";

/// Errors returned by campaign manifest helpers.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum CampaignError {
    /// TOML parsing failed.
    #[error("campaign TOML error: {message}")]
    Toml {
        /// Error message.
        message: String,
    },
    /// JSON serialization or parsing failed.
    #[error("campaign JSON error: {message}")]
    Json {
        /// Error message.
        message: String,
    },
    /// A required string field was empty after trimming.
    #[error("{field} must not be empty")]
    EmptyField {
        /// Field name.
        field: &'static str,
    },
    /// At least one scenario is required.
    #[error("campaign manifest requires at least one scenario")]
    EmptyScenarios,
    /// At least one seed is required.
    #[error("campaign manifest requires at least one seed")]
    EmptySeeds,
    /// Scenario ids must be unique.
    #[error("duplicate campaign scenario id {id}")]
    DuplicateScenarioId {
        /// Duplicate scenario id.
        id: String,
    },
    /// Overlay file ids must be unique.
    #[error("duplicate campaign overlay file id {id}")]
    DuplicateOverlayId {
        /// Duplicate overlay file id.
        id: String,
    },
    /// Worker count must be positive.
    #[error("worker_count must be positive")]
    InvalidWorkerCount,
    /// At least one verdict report is required for aggregation.
    #[error("campaign verdict aggregation requires at least one report")]
    EmptyVerdictReports,
}

impl From<toml::de::Error> for CampaignError {
    fn from(error: toml::de::Error) -> Self {
        Self::Toml {
            message: error.to_string(),
        }
    }
}

impl From<serde_json::Error> for CampaignError {
    fn from(error: serde_json::Error) -> Self {
        Self::Json {
            message: error.to_string(),
        }
    }
}

impl From<RundataError> for CampaignError {
    fn from(error: RundataError) -> Self {
        match error {
            RundataError::EmptyField { field } => Self::EmptyField { field },
            other => Self::Json {
                message: other.to_string(),
            },
        }
    }
}

/// Declarative campaign manifest.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CampaignManifest {
    /// Stable campaign id.
    pub campaign_id: String,
    /// Scenario entries.
    #[serde(default)]
    pub scenarios: Vec<CampaignScenario>,
    /// Seeds to run for every scenario/overlay.
    #[serde(default)]
    pub seeds: Vec<u64>,
    /// Optional overlay ids to run for every scenario/seed.
    #[serde(default)]
    pub overlays: Vec<String>,
    /// Optional TOML fragment files keyed by overlay id for CLI execution.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub overlay_files: Vec<CampaignOverlayFile>,
    /// Optional verdict-rule document path.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub verdict_rules: Option<String>,
    /// Optional trend-spec document path.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub trend_specs: Option<String>,
}

impl CampaignManifest {
    /// Parse and validate a campaign manifest from TOML.
    ///
    /// # Errors
    ///
    /// Returns [`CampaignError`] when the TOML is malformed or validation
    /// fails.
    pub fn from_toml_str(input: &str) -> Result<Self, CampaignError> {
        let manifest: Self = toml::from_str(input)?;
        manifest.validate()?;
        Ok(manifest)
    }

    fn validate(&self) -> Result<(), CampaignError> {
        require_string("campaign_id", self.campaign_id.clone())?;
        if self.scenarios.is_empty() {
            return Err(CampaignError::EmptyScenarios);
        }
        if self.seeds.is_empty() {
            return Err(CampaignError::EmptySeeds);
        }
        let mut ids = std::collections::BTreeSet::new();
        for scenario in &self.scenarios {
            require_string("scenario.id", scenario.id.clone())?;
            require_string("scenario.path", scenario.path.clone())?;
            if !ids.insert(scenario.id.clone()) {
                return Err(CampaignError::DuplicateScenarioId {
                    id: scenario.id.clone(),
                });
            }
        }
        for overlay in &self.overlays {
            require_string("overlay", overlay.clone())?;
        }
        let mut overlay_ids = std::collections::BTreeSet::new();
        for overlay_file in &self.overlay_files {
            require_string("overlay_files.id", overlay_file.id.clone())?;
            require_string("overlay_files.path", overlay_file.path.clone())?;
            if !overlay_ids.insert(overlay_file.id.clone()) {
                return Err(CampaignError::DuplicateOverlayId {
                    id: overlay_file.id.clone(),
                });
            }
        }
        if let Some(path) = &self.verdict_rules {
            require_string("verdict_rules", path.clone())?;
        }
        if let Some(path) = &self.trend_specs {
            require_string("trend_specs", path.clone())?;
        }
        Ok(())
    }
}

/// One campaign scenario entry.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CampaignScenario {
    /// Stable scenario id.
    pub id: String,
    /// Scenario path.
    pub path: String,
}

/// Overlay application mode for a campaign overlay file.
#[derive(Clone, Copy, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CampaignOverlayMode {
    /// Append the overlay TOML text after the base scenario.
    #[default]
    Append,
    /// Parse both documents and merge the overlay into the base scenario.
    Merge,
}

fn is_default_overlay_mode(mode: &CampaignOverlayMode) -> bool {
    *mode == CampaignOverlayMode::Append
}

/// Overlay TOML file keyed by overlay id.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CampaignOverlayFile {
    /// Overlay id referenced by [`CampaignManifest::overlays`].
    pub id: String,
    /// TOML overlay path.
    pub path: String,
    /// Overlay application mode.
    #[serde(default, skip_serializing_if = "is_default_overlay_mode")]
    pub mode: CampaignOverlayMode,
}

/// Expanded campaign plan.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CampaignPlan {
    /// Plan schema version.
    pub schema_version: String,
    /// Stable campaign id.
    pub campaign_id: String,
    /// Expanded run specifications.
    pub runs: Vec<CampaignRunSpec>,
}

impl CampaignPlan {
    /// Serialize as deterministic JSON bytes.
    ///
    /// # Errors
    ///
    /// Returns [`CampaignError::Json`] if serialization fails.
    pub fn to_json_bytes(&self) -> Result<Vec<u8>, CampaignError> {
        Ok(serde_json::to_vec(self)?)
    }

    /// Parse a JSON campaign plan.
    ///
    /// # Errors
    ///
    /// Returns [`CampaignError::Json`] if parsing fails.
    pub fn from_json_bytes(bytes: &[u8]) -> Result<Self, CampaignError> {
        Ok(serde_json::from_slice(bytes)?)
    }
}

/// One expanded run in a campaign plan.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CampaignRunSpec {
    /// Stable run key.
    pub run_key: String,
    /// Scenario id.
    pub scenario_id: String,
    /// Scenario path.
    pub scenario_path: String,
    /// Seed for this run.
    pub seed: u64,
    /// Overlay id, if any.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub overlay: Option<String>,
    /// Deterministic labels attached to the eventual run record.
    pub labels: Vec<CampaignLabel>,
}

/// Deterministic campaign label.
#[derive(Clone, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CampaignLabel {
    /// Label key.
    pub key: String,
    /// Label value.
    pub value: String,
}

/// Worker shard assignment.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CampaignShard {
    /// Zero-based worker index.
    pub worker_index: u64,
    /// Runs assigned to this worker.
    pub runs: Vec<CampaignRunSpec>,
}

/// Campaign verdict aggregation status.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CampaignVerdictStatus {
    /// Every report passed.
    Pass,
    /// At least one report failed.
    Fail,
}

/// Aggregated campaign verdict summary.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CampaignVerdictSummary {
    /// Aggregated status.
    pub status: CampaignVerdictStatus,
    /// Number of reports aggregated.
    pub report_count: u64,
    /// Number of reports that did not pass.
    pub failed_report_count: u64,
}

/// Expand a campaign manifest into a deterministic run plan.
///
/// # Errors
///
/// Returns [`CampaignError`] when the manifest is invalid.
pub fn expand_campaign_plan(manifest: &CampaignManifest) -> Result<CampaignPlan, CampaignError> {
    manifest.validate()?;
    let mut scenarios = manifest.scenarios.clone();
    scenarios.sort_by(|a, b| a.id.cmp(&b.id).then_with(|| a.path.cmp(&b.path)));
    let mut seeds = manifest.seeds.clone();
    seeds.sort_unstable();
    seeds.dedup();
    let mut overlays = if manifest.overlays.is_empty() {
        vec![None]
    } else {
        let mut overlays = manifest
            .overlays
            .iter()
            .map(|overlay| Some(overlay.clone()))
            .collect::<Vec<_>>();
        overlays.sort();
        overlays.dedup();
        overlays
    };
    overlays.sort();

    let mut runs = Vec::new();
    for scenario in scenarios {
        for seed in &seeds {
            for overlay in &overlays {
                runs.push(run_spec(
                    &manifest.campaign_id,
                    &scenario,
                    *seed,
                    overlay.clone(),
                )?);
            }
        }
    }
    Ok(CampaignPlan {
        schema_version: CAMPAIGN_SCHEMA_VERSION.to_owned(),
        campaign_id: manifest.campaign_id.clone(),
        runs,
    })
}

/// Partition a campaign plan into deterministic round-robin worker shards.
///
/// # Errors
///
/// Returns [`CampaignError::InvalidWorkerCount`] when `worker_count` is zero.
pub fn partition_campaign_runs(
    plan: &CampaignPlan,
    worker_count: u64,
) -> Result<Vec<CampaignShard>, CampaignError> {
    if worker_count == 0 {
        return Err(CampaignError::InvalidWorkerCount);
    }
    let mut shards = (0..worker_count)
        .map(|worker_index| CampaignShard {
            worker_index,
            runs: Vec::new(),
        })
        .collect::<Vec<_>>();
    for (index, run) in plan.runs.iter().enumerate() {
        let worker_index = index % worker_count as usize;
        shards[worker_index].runs.push(run.clone());
    }
    Ok(shards)
}

/// Aggregate per-run verdict reports into a campaign verdict summary.
///
/// # Errors
///
/// Returns [`CampaignError::EmptyVerdictReports`] when no reports are supplied.
pub fn aggregate_campaign_verdicts(
    reports: &[VerdictReport],
) -> Result<CampaignVerdictSummary, CampaignError> {
    if reports.is_empty() {
        return Err(CampaignError::EmptyVerdictReports);
    }
    let failed_report_count = reports.iter().filter(|report| !report.passed()).count() as u64;
    Ok(CampaignVerdictSummary {
        status: if failed_report_count == 0 {
            CampaignVerdictStatus::Pass
        } else {
            CampaignVerdictStatus::Fail
        },
        report_count: reports.len() as u64,
        failed_report_count,
    })
}

fn run_spec(
    campaign_id: &str,
    scenario: &CampaignScenario,
    seed: u64,
    overlay: Option<String>,
) -> Result<CampaignRunSpec, CampaignError> {
    let overlay_key = overlay.as_deref().unwrap_or("default");
    let run_key = format!("{campaign_id}.{}.seed-{seed}.{overlay_key}", scenario.id);
    let mut labels = vec![
        label("campaign", campaign_id)?,
        label("scenario", &scenario.id)?,
        label("seed", seed.to_string())?,
        label("overlay", overlay_key)?,
    ];
    labels.sort();
    Ok(CampaignRunSpec {
        run_key,
        scenario_id: scenario.id.clone(),
        scenario_path: scenario.path.clone(),
        seed,
        overlay,
        labels,
    })
}

fn label(key: impl Into<String>, value: impl Into<String>) -> Result<CampaignLabel, CampaignError> {
    Ok(CampaignLabel {
        key: require_string("label.key", key.into())?,
        value: require_string("label.value", value.into())?,
    })
}

fn require_string(field: &'static str, value: String) -> Result<String, CampaignError> {
    require_non_empty(field, value).map_err(|_| CampaignError::EmptyField { field })
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::verdict::{VERDICT_SCHEMA_VERSION, VerdictRecord, VerdictStatus};

    fn manifest() -> CampaignManifest {
        CampaignManifest::from_toml_str(include_str!("../tests/fixtures/campaign-manifest.toml"))
            .unwrap()
    }

    #[test]
    fn campaign_manifest_expands_to_stable_plan_golden() {
        let plan = expand_campaign_plan(&manifest()).unwrap();
        let decoded = CampaignPlan::from_json_bytes(&plan.to_json_bytes().unwrap()).unwrap();
        let golden = include_str!("../tests/fixtures/campaign-plan.json");
        let golden = golden.strip_suffix('\n').unwrap_or(golden);

        assert_eq!(decoded, plan);
        assert_eq!(plan.runs.len(), 8);
        assert_eq!(
            std::str::from_utf8(&plan.to_json_bytes().unwrap()).unwrap(),
            golden
        );
    }

    #[test]
    fn campaign_partition_is_worker_count_independent_after_flattening() {
        let plan = expand_campaign_plan(&manifest()).unwrap();
        let expected = plan
            .runs
            .iter()
            .map(|run| run.run_key.clone())
            .collect::<Vec<_>>();

        for worker_count in [1, 4, 16] {
            let mut flattened = partition_campaign_runs(&plan, worker_count)
                .unwrap()
                .into_iter()
                .flat_map(|shard| shard.runs)
                .map(|run| run.run_key)
                .collect::<Vec<_>>();
            flattened.sort();
            let mut sorted_expected = expected.clone();
            sorted_expected.sort();
            assert_eq!(flattened, sorted_expected);
        }
    }

    #[test]
    fn failing_rule_fails_campaign_verdict_summary() {
        let pass = VerdictReport {
            schema_version: VERDICT_SCHEMA_VERSION.to_owned(),
            verdicts: vec![VerdictRecord {
                id: "ok".to_owned(),
                status: VerdictStatus::Pass,
                measured: None,
                expected: None,
                lower: None,
                upper: None,
                detail: "passed".to_owned(),
            }],
        };
        let fail = VerdictReport {
            schema_version: VERDICT_SCHEMA_VERSION.to_owned(),
            verdicts: vec![VerdictRecord {
                id: "bad".to_owned(),
                status: VerdictStatus::Fail,
                measured: None,
                expected: None,
                lower: None,
                upper: None,
                detail: "failed".to_owned(),
            }],
        };

        let summary = aggregate_campaign_verdicts(&[pass, fail]).unwrap();

        assert_eq!(summary.status, CampaignVerdictStatus::Fail);
        assert_eq!(summary.report_count, 2);
        assert_eq!(summary.failed_report_count, 1);
    }

    #[test]
    fn campaign_helpers_fail_closed_on_bad_inputs() {
        let empty =
            CampaignManifest::from_toml_str("campaign_id = 'x'\nseeds = [1]\n").unwrap_err();
        assert_eq!(empty, CampaignError::EmptyScenarios);

        let duplicate = CampaignManifest::from_toml_str(
            "campaign_id = 'x'\nseeds = [1]\n[[scenarios]]\nid = 'a'\npath = 'a.toml'\n[[scenarios]]\nid = 'a'\npath = 'b.toml'\n",
        )
        .unwrap_err();
        assert!(matches!(
            duplicate,
            CampaignError::DuplicateScenarioId { .. }
        ));

        let duplicate_overlay = CampaignManifest::from_toml_str(
            "campaign_id = 'x'\nseeds = [1]\noverlays = ['wind']\n[[overlay_files]]\nid = 'wind'\npath = 'a.toml'\n[[overlay_files]]\nid = 'wind'\npath = 'b.toml'\n[[scenarios]]\nid = 'a'\npath = 'a.toml'\n",
        )
        .unwrap_err();
        assert!(matches!(
            duplicate_overlay,
            CampaignError::DuplicateOverlayId { .. }
        ));

        let default_overlay_mode = CampaignManifest::from_toml_str(
            "campaign_id = 'x'\nseeds = [1]\noverlays = ['wind']\n[[overlay_files]]\nid = 'wind'\npath = 'a.toml'\n[[scenarios]]\nid = 'a'\npath = 'a.toml'\n",
        )
        .unwrap();
        assert_eq!(
            default_overlay_mode.overlay_files[0].mode,
            CampaignOverlayMode::Append
        );

        let merge_overlay_mode = CampaignManifest::from_toml_str(
            "campaign_id = 'x'\nseeds = [1]\noverlays = ['wind']\n[[overlay_files]]\nid = 'wind'\npath = 'a.toml'\nmode = 'merge'\n[[scenarios]]\nid = 'a'\npath = 'a.toml'\n",
        )
        .unwrap();
        assert_eq!(
            merge_overlay_mode.overlay_files[0].mode,
            CampaignOverlayMode::Merge
        );

        let bad_overlay_mode = CampaignManifest::from_toml_str(
            "campaign_id = 'x'\nseeds = [1]\noverlays = ['wind']\n[[overlay_files]]\nid = 'wind'\npath = 'a.toml'\nmode = 'patch'\n[[scenarios]]\nid = 'a'\npath = 'a.toml'\n",
        )
        .unwrap_err();
        assert!(matches!(bad_overlay_mode, CampaignError::Toml { .. }));

        let plan = expand_campaign_plan(&manifest()).unwrap();
        let workers = partition_campaign_runs(&plan, 0).unwrap_err();
        assert_eq!(workers, CampaignError::InvalidWorkerCount);

        let reports = aggregate_campaign_verdicts(&[]).unwrap_err();
        assert_eq!(reports, CampaignError::EmptyVerdictReports);
    }
}
