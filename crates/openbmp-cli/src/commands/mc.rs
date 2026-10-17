//! Monte Carlo utility commands.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::fs;
use std::fs::create_dir_all;
use std::path::Path;
use std::path::PathBuf;

use openbmp_mc::{
    BernoulliStats, CampaignCredibilityReport, CheckpointStoreError, CorrelationMatrix,
    CrossEntropyIs, DesignMatrix, FileCheckpointStore, ImanConover, LatinHypercube,
    MonteCarloError, NestedLowerTailRequirement, NestedScalarAnalysisReport, OwenScramble,
    PropulsionFaultLibrary, PropulsionFaultTemplate, RareEventEstimateReport,
    ScalarCampaignCheckpoint, ScalarCampaignReport, ScalarOutcome, SobolSequence, SubsetSimulation,
    SyntheticLinearLimitState, Welford, analyze_nested_scalar_samples,
    evaluate_campaign_credibility, run_scalar_campaign_resumable, wilks_one_sided_confidence,
    wilks_one_sided_n, wilks_two_sided_confidence, wilks_two_sided_n,
};
use openbmp_scenario::{MonteCarloConfig, Scenario};
use openbmp_telemetry::{TelemetryTable, TelemetryValue};
use openbmp_uq::{
    CorrelatedErrorBudget, CorrelationMatrix as UqCorrelationMatrix, CredibilityFactor,
    CredibilityLevel, CredibilityRecord, UncertaintyClass, UncertaintySource,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::CliError;
use crate::commands::campaign;

const ACOUSTICS_ASCENT_CAMPAIGN_DIMENSION_ID: u32 = 22_006;

/// Summary report for `openbmp mc summarize`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct McSummaryReport {
    /// Number of scalar samples summarized.
    pub samples: u64,
    /// Welford mean of the scalar quantity.
    pub mean: f64,
    /// Unbiased sample variance, if at least two samples were present.
    pub sample_variance: Option<f64>,
    /// Monte Carlo standard error of the mean, if at least two samples
    /// were present.
    pub standard_error: Option<f64>,
    /// Optional Bernoulli success summary.
    pub success: Option<McSuccessSummary>,
}

/// Bernoulli success summary for `openbmp mc summarize`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct McSuccessSummary {
    /// Number of Bernoulli trials.
    pub trials: u64,
    /// Number of successful trials.
    pub successes: u64,
    /// Observed success fraction.
    pub fraction: f64,
    /// Requested confidence level.
    pub confidence: f64,
    /// Lower Clopper-Pearson confidence bound.
    pub lower: f64,
    /// Upper Clopper-Pearson confidence bound.
    pub upper: f64,
}

/// Wilks tolerance-bound side.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WilksSide {
    /// One-sided first-order tolerance bound.
    OneSided,
    /// Two-sided min/max tolerance interval.
    TwoSided,
}

impl WilksSide {
    /// Human-readable label.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::OneSided => "one-sided",
            Self::TwoSided => "two-sided",
        }
    }
}

/// Summary report for `openbmp mc wilks`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct McWilksReport {
    /// One-sided or two-sided Wilks bound.
    pub side: WilksSide,
    /// Requested population coverage proportion.
    pub coverage: f64,
    /// Requested confidence level.
    pub confidence: f64,
    /// Minimum sample count satisfying the Wilks inequality.
    pub samples: u64,
    /// Confidence attained at `samples`.
    pub attained_confidence: f64,
}

/// Summary report for `openbmp mc lhs`.
#[derive(Clone, Debug, PartialEq)]
pub struct McLhsReport {
    /// Number of sample rows written.
    pub samples: u64,
    /// Number of dispersion dimensions written.
    pub dimensions: u32,
    /// Output CSV path.
    pub output_csv: PathBuf,
}

/// Summary report for `openbmp mc sobol`.
#[derive(Clone, Debug, PartialEq)]
pub struct McSobolReport {
    /// Number of sample rows written.
    pub samples: u64,
    /// Number of dispersion dimensions written.
    pub dimensions: u32,
    /// Optional Owen scramble seed used for this design.
    pub scramble_seed: Option<u64>,
    /// Output CSV path.
    pub output_csv: PathBuf,
}

/// Deterministic design family for generated ascent acoustic trace rows.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum McAcousticsAscentTraceDesign {
    /// Latin hypercube design keyed by campaign seed.
    LatinHypercube,
    /// Unscrambled Sobol base design.
    Sobol,
    /// Owen-scrambled Sobol design keyed by campaign seed.
    OwenSobol,
}

/// Options for `openbmp mc acoustics-ascent-trace`.
#[derive(Clone, Debug)]
pub struct McAcousticsAscentTraceOptions<'a> {
    /// Number of trace samples to materialize.
    pub samples: u64,
    /// Campaign seed for LHS or Owen-scrambled Sobol designs.
    pub campaign_seed: u64,
    /// Design family.
    pub design: McAcousticsAscentTraceDesign,
    /// Inclusive lower dynamic pressure bound, Pa.
    pub dynamic_pressure_min_pa: f64,
    /// Exclusive upper dynamic pressure bound in practice because unit-cube
    /// design values lie in `[0, 1)`.
    pub dynamic_pressure_max_pa: f64,
    /// Inclusive lower Mach bound.
    pub mach_min: f64,
    /// Exclusive upper Mach bound in practice because unit-cube design values
    /// lie in `[0, 1)`.
    pub mach_max: f64,
    /// Prefix for deterministic sample ids.
    pub sample_prefix: &'a str,
    /// Optional per-row class override.
    pub class_id: Option<&'a str>,
    /// Output TOML snippet path.
    pub output_toml: &'a Path,
}

/// Summary report for `openbmp mc acoustics-ascent-trace`.
#[derive(Clone, Debug, PartialEq)]
pub struct McAcousticsAscentTraceReport {
    /// Number of generated trace samples.
    pub samples: u64,
    /// Design family.
    pub design: McAcousticsAscentTraceDesign,
    /// Output TOML snippet path.
    pub output_toml: PathBuf,
}

/// Options for `openbmp mc acoustics-response-samples`.
#[derive(Clone, Debug)]
pub struct McAcousticsResponseSamplesOptions<'a> {
    /// Number of response samples to materialize.
    pub samples: u64,
    /// Campaign seed for LHS or Owen-scrambled Sobol designs.
    pub campaign_seed: u64,
    /// Design family.
    pub design: McAcousticsAscentTraceDesign,
    /// Inclusive lower panel surface mass bound, kg/m^2.
    pub surface_mass_min_kg_m2: f64,
    /// Exclusive upper panel surface mass bound in practice because unit-cube
    /// design values lie in `[0, 1)`.
    pub surface_mass_max_kg_m2: f64,
    /// Inclusive lower responding area bound, m^2.
    pub area_min_m2: f64,
    /// Exclusive upper responding area bound in practice because unit-cube
    /// design values lie in `[0, 1)`.
    pub area_max_m2: f64,
    /// Inclusive lower modal shape participation factor bound.
    pub mode_shape_min: f64,
    /// Exclusive upper modal shape participation factor bound in practice
    /// because unit-cube design values lie in `[0, 1)`.
    pub mode_shape_max: f64,
    /// Prefix for deterministic sample ids.
    pub sample_prefix: &'a str,
    /// Optional per-row response class override.
    pub class_id: Option<&'a str>,
    /// Output TOML snippet path.
    pub output_toml: &'a Path,
}

/// Summary report for `openbmp mc acoustics-response-samples`.
#[derive(Clone, Debug, PartialEq)]
pub struct McAcousticsResponseSamplesReport {
    /// Number of generated response samples.
    pub samples: u64,
    /// Design family.
    pub design: McAcousticsAscentTraceDesign,
    /// Output TOML snippet path.
    pub output_toml: PathBuf,
}

/// Options for `openbmp mc aero-deck-draw-overlays`.
#[derive(Clone, Debug)]
pub struct McAeroDeckDrawOverlaysOptions<'a> {
    /// Number of deck-draw overlays to materialize.
    pub samples: u64,
    /// Campaign-level deterministic seed.
    pub campaign_seed: u64,
    /// First sample index to materialize.
    pub first_sample_index: u64,
    /// First RNG dimension id used by the draw.
    pub dimension_id_base: u32,
    /// Exponential Mach correlation length in Mach units. When omitted,
    /// deck-declared uncertainty correlation metadata is used at runtime.
    pub mach_correlation_length: Option<f64>,
    /// Optional exponential angle-of-attack correlation length, in degrees.
    pub alpha_correlation_length_deg: Option<f64>,
    /// Optional exponential sideslip correlation length, in degrees.
    pub beta_correlation_length_deg: Option<f64>,
    /// Prefix for deterministic overlay ids and filenames.
    pub overlay_prefix: &'a str,
    /// Directory receiving per-sample overlay TOML files.
    pub output_dir: &'a Path,
    /// Campaign-manifest TOML fragment listing the generated overlays.
    pub manifest_toml: &'a Path,
}

/// Summary report for `openbmp mc aero-deck-draw-overlays`.
#[derive(Clone, Debug, PartialEq)]
pub struct McAeroDeckDrawOverlaysReport {
    /// Number of generated deck-draw overlays.
    pub samples: u64,
    /// Campaign-level deterministic seed.
    pub campaign_seed: u64,
    /// First sample index materialized.
    pub first_sample_index: u64,
    /// Directory receiving per-sample overlay TOML files.
    pub output_dir: PathBuf,
    /// Campaign-manifest TOML fragment listing the generated overlays.
    pub manifest_toml: PathBuf,
    /// Generated overlay file paths.
    pub overlay_paths: Vec<PathBuf>,
}

/// Options for `openbmp mc aero-deck-draw-campaign`.
#[derive(Clone, Debug)]
pub struct McAeroDeckDrawCampaignOptions<'a> {
    /// Base scenario TOML path. The scenario must be schema-v3 and deck- or
    /// buildup-backed for `[aero.deck_draw]` overlays to validate.
    pub scenario_path: &'a Path,
    /// Number of deck-draw samples to execute.
    pub samples: u64,
    /// Campaign-level deterministic deck-draw seed.
    pub campaign_seed: u64,
    /// First sample index to execute.
    pub first_sample_index: u64,
    /// First RNG dimension id used by the draw.
    pub dimension_id_base: u32,
    /// Exponential Mach correlation length in Mach units. When omitted,
    /// deck-declared uncertainty correlation metadata is used at runtime.
    pub mach_correlation_length: Option<f64>,
    /// Optional exponential angle-of-attack correlation length, in degrees.
    pub alpha_correlation_length_deg: Option<f64>,
    /// Optional exponential sideslip correlation length, in degrees.
    pub beta_correlation_length_deg: Option<f64>,
    /// Prefix for deterministic overlay ids and filenames.
    pub overlay_prefix: &'a str,
    /// Stable generated campaign id.
    pub campaign_id: &'a str,
    /// Stable generated scenario id.
    pub scenario_id: &'a str,
    /// Scenario seed used for each campaign run.
    pub run_seed: u64,
    /// Output artifact root for generated overlays, manifest, and campaign run.
    pub output_dir: &'a Path,
    /// Stable code/config id recorded in generated campaign run-data records.
    pub run_code_id: &'a str,
    /// Code ordinal label recorded for trend-store consumers.
    pub code_ordinal: u64,
    /// Also write a deterministic Parquet mirror of the campaign run index.
    pub write_index_parquet: bool,
    /// Maximum worker threads used for generated campaign execution.
    pub workers: u64,
    /// Optional UQ credibility evidence for this generated campaign.
    pub credibility: Option<McCredibilityOptions<'a>>,
}

/// Summary report for `openbmp mc aero-deck-draw-campaign`.
#[derive(Clone, Debug, PartialEq)]
pub struct McAeroDeckDrawCampaignReport {
    /// Number of executed deck-draw samples.
    pub samples: u64,
    /// Generated overlay report.
    pub overlays: McAeroDeckDrawOverlaysReport,
    /// Generated full campaign manifest path.
    pub campaign_manifest_toml: PathBuf,
    /// Output directory passed to the campaign runner.
    pub campaign_output_dir: PathBuf,
    /// Existing campaign-run report.
    pub campaign: campaign::CampaignRunCliReport,
    /// Optional campaign UQ/credibility evidence.
    pub credibility: Option<CampaignCredibilityReport>,
    /// Markdown report path, when written.
    pub credibility_report_md: Option<PathBuf>,
}

/// Options for `openbmp mc acoustics-ascent-campaign`.
#[derive(Clone, Debug)]
pub struct McAcousticsAscentCampaignOptions<'a> {
    /// Base scenario TOML path with an `[acoustics.ascent]` block.
    pub scenario_path: &'a Path,
    /// Number of generated trace samples to evaluate.
    pub samples: u64,
    /// Campaign seed for LHS or Owen-scrambled Sobol designs.
    pub campaign_seed: u64,
    /// Design family.
    pub design: McAcousticsAscentTraceDesign,
    /// Inclusive lower dynamic pressure bound, Pa.
    pub dynamic_pressure_min_pa: f64,
    /// Exclusive upper dynamic pressure bound in practice because unit-cube
    /// design values lie in `[0, 1)`.
    pub dynamic_pressure_max_pa: f64,
    /// Inclusive lower Mach bound.
    pub mach_min: f64,
    /// Exclusive upper Mach bound in practice because unit-cube design values
    /// lie in `[0, 1)`.
    pub mach_max: f64,
    /// Prefix for deterministic sample ids.
    pub sample_prefix: &'a str,
    /// Optional per-row class override.
    pub class_id: Option<&'a str>,
    /// Output CSV path for the evaluated ascent trace samples.
    pub output_csv: &'a Path,
    /// Optional directory for evaluated campaign CSV artifacts.
    pub output_artifact_dir: Option<&'a Path>,
    /// Optional full generated scenario path.
    pub output_scenario_toml: Option<&'a Path>,
    /// Optional scalar checkpoint JSON for resumable per-sample evaluation.
    pub checkpoint_json: Option<&'a Path>,
    /// Maximum missing samples to evaluate into the checkpoint this invocation.
    pub max_new_samples: Option<u64>,
    /// Worker threads for checkpointed sample evaluation.
    pub workers: usize,
    /// Toolchain/determinism profile bound into checkpoint metadata.
    pub toolchain_profile: &'a str,
}

/// Summary report for `openbmp mc acoustics-ascent-campaign`.
#[derive(Clone, Debug, PartialEq)]
pub struct McAcousticsAscentCampaignReport {
    /// Base scenario TOML path.
    pub scenario_path: PathBuf,
    /// Number of generated/evaluated trace samples.
    pub samples: u64,
    /// Design family.
    pub design: McAcousticsAscentTraceDesign,
    /// Output CSV path.
    pub output_csv: PathBuf,
    /// Optional directory for evaluated campaign CSV artifacts.
    pub output_artifact_dir: Option<PathBuf>,
    /// Deterministic evaluated artifact paths written under `output_artifact_dir`.
    pub artifact_paths: Vec<PathBuf>,
    /// Deterministic JSON manifest for evaluated campaign artifacts.
    pub artifact_manifest_json: Option<PathBuf>,
    /// Optional full generated scenario path.
    pub output_scenario_toml: Option<PathBuf>,
    /// Optional scalar checkpoint path used for resumable sample evaluation.
    pub checkpoint_json: Option<PathBuf>,
    /// Number of samples recorded in the checkpoint or final run.
    pub completed_samples: u64,
    /// Whether the campaign completed and final outputs were written.
    pub complete: bool,
    /// P95/50 OASPL from the runner, when available.
    pub p95_50_oaspl_db: Option<f64>,
    /// One-sided Wilks sample count required for the P95/50 target.
    pub p95_50_wilks_required_sample_count: Option<u64>,
    /// Whether the generated sample count satisfies the P95/50 Wilks floor.
    pub p95_50_wilks_sample_count_sufficient: Option<bool>,
    /// Fairing-internal P95/50 OASPL from downstream transfer, when available.
    pub fairing_internal_p95_50_oaspl_db: Option<f64>,
    /// Fairing-internal one-sided Wilks sample count required for P95/50.
    pub fairing_internal_p95_50_wilks_required_sample_count: Option<u64>,
    /// Whether the generated sample count satisfies the fairing P95/50 Wilks floor.
    pub fairing_internal_p95_50_wilks_sample_count_sufficient: Option<bool>,
    /// Response P95/50 integrated acceleration mean-square, when available.
    pub response_p95_50_total_mean_square_acceleration_m2_s4: Option<f64>,
    /// Response P95/50 integrated modal-force mean-square, when available.
    pub response_p95_50_total_mean_square_force_n2: Option<f64>,
    /// Response one-sided Wilks sample count required for P95/50.
    pub response_p95_50_wilks_required_sample_count: Option<u64>,
    /// Whether the generated sample count satisfies the response P95/50 Wilks floor.
    pub response_p95_50_wilks_sample_count_sufficient: Option<bool>,
    /// Peak sample id selected by the runner.
    pub peak_sample_id: Option<String>,
    /// Peak sample OASPL selected by the runner.
    pub peak_sample_oaspl_db: f64,
}

/// Optional response-side sample generator for
/// `openbmp mc acoustics-response-campaign`.
#[derive(Clone, Debug)]
pub struct McAcousticsResponseCampaignSamplesOptions<'a> {
    /// Inclusive lower panel surface mass bound, kg/m^2.
    pub surface_mass_min_kg_m2: f64,
    /// Exclusive upper panel surface mass bound in practice because unit-cube
    /// design values lie in `[0, 1)`.
    pub surface_mass_max_kg_m2: f64,
    /// Inclusive lower responding area bound, m^2.
    pub area_min_m2: f64,
    /// Exclusive upper responding area bound in practice because unit-cube
    /// design values lie in `[0, 1)`.
    pub area_max_m2: f64,
    /// Inclusive lower modal shape participation factor bound.
    pub mode_shape_min: f64,
    /// Exclusive upper modal shape participation factor bound in practice
    /// because unit-cube design values lie in `[0, 1)`.
    pub mode_shape_max: f64,
    /// Prefix for deterministic response sample ids.
    pub sample_prefix: &'a str,
    /// Optional per-row response class override.
    pub class_id: Option<&'a str>,
}

/// Options for `openbmp mc acoustics-response-campaign`.
#[derive(Clone, Debug)]
pub struct McAcousticsResponseCampaignOptions<'a> {
    /// Base scenario TOML path with an `[acoustics.response]` block.
    pub scenario_path: &'a Path,
    /// Number of generated trace samples to evaluate.
    pub samples: u64,
    /// Campaign seed for LHS or Owen-scrambled Sobol designs.
    pub campaign_seed: u64,
    /// Design family.
    pub design: McAcousticsAscentTraceDesign,
    /// Inclusive lower dynamic pressure bound, Pa.
    pub dynamic_pressure_min_pa: f64,
    /// Exclusive upper dynamic pressure bound in practice because unit-cube
    /// design values lie in `[0, 1)`.
    pub dynamic_pressure_max_pa: f64,
    /// Inclusive lower Mach bound.
    pub mach_min: f64,
    /// Exclusive upper Mach bound in practice because unit-cube design values
    /// lie in `[0, 1)`.
    pub mach_max: f64,
    /// Prefix for deterministic ascent sample ids.
    pub sample_prefix: &'a str,
    /// Optional per-row ascent class override.
    pub class_id: Option<&'a str>,
    /// Optional response-side sample generator.
    pub response_samples: Option<McAcousticsResponseCampaignSamplesOptions<'a>>,
    /// Output CSV path for the evaluated ascent trace samples.
    pub output_csv: &'a Path,
    /// Optional directory for evaluated campaign CSV artifacts.
    pub output_artifact_dir: Option<&'a Path>,
    /// Optional full generated scenario path.
    pub output_scenario_toml: Option<&'a Path>,
    /// Optional scalar checkpoint JSON for resumable per-sample evaluation.
    pub checkpoint_json: Option<&'a Path>,
    /// Maximum missing samples to evaluate into the checkpoint this invocation.
    pub max_new_samples: Option<u64>,
    /// Worker threads for checkpointed sample evaluation.
    pub workers: usize,
    /// Toolchain/determinism profile bound into checkpoint metadata.
    pub toolchain_profile: &'a str,
}

/// Summary report for `openbmp mc acoustics-response-campaign`.
pub type McAcousticsResponseCampaignReport = McAcousticsAscentCampaignReport;

/// Summary report for `openbmp mc iman-conover`.
#[derive(Clone, Debug, PartialEq)]
pub struct McImanConoverReport {
    /// Number of sample rows written.
    pub samples: u64,
    /// Number of dispersion dimensions written.
    pub dimensions: u32,
    /// Output CSV path.
    pub output_csv: PathBuf,
}

/// Summary report for `openbmp mc resume-scalar`.
#[derive(Clone, Debug, PartialEq)]
pub struct McResumeScalarReport {
    /// Number of completed samples now present in the checkpoint.
    pub completed: u64,
    /// Total campaign sample count declared by the checkpoint.
    pub sample_count: u64,
    /// Checkpoint JSON path.
    pub checkpoint_json: PathBuf,
    /// Final summary when the checkpoint is complete.
    pub summary: Option<McSummaryReport>,
}

/// Summary report for `openbmp mc summarize` with optional UQ evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct McScalarSummaryReport {
    /// Scalar sample statistics.
    pub summary: McSummaryReport,
    /// Optional campaign UQ/credibility evidence.
    pub credibility: Option<CampaignCredibilityReport>,
    /// Markdown report path, when written.
    pub credibility_report_md: Option<PathBuf>,
}

/// Summary report for `openbmp mc nested-summarize`.
#[derive(Clone, Debug, PartialEq)]
pub struct McNestedSummaryReport {
    /// Nested scalar p-box and variance split.
    pub analysis: NestedScalarAnalysisReport,
    /// Optional p-box CSV path, when written.
    pub pbox_csv: Option<PathBuf>,
}

/// Options for `openbmp mc rare-event`.
#[derive(Clone, Copy, Debug)]
pub struct McRareEventScenarioOptions<'a> {
    /// Scenario TOML path.
    pub scenario_path: &'a Path,
    /// Optional deterministic TOML evidence output.
    pub output_toml: Option<&'a Path>,
}

/// Summary report for `openbmp mc rare-event`.
#[derive(Clone, Debug, PartialEq)]
pub struct McRareEventScenarioReport {
    /// Scenario TOML path.
    pub scenario_path: PathBuf,
    /// Campaign seed used by the estimators.
    pub seed: u64,
    /// Synthetic limit-state label.
    pub limit_state_label: String,
    /// Standard-normal input dimension.
    pub dimension: usize,
    /// Rare-event estimator reports, in scenario declaration order.
    pub estimates: Vec<RareEventEstimateReport>,
    /// TOML evidence path, when written.
    pub output_toml: Option<PathBuf>,
}

/// Optional UQ credibility inputs for MC summary reports.
#[derive(Clone, Copy, Debug)]
pub struct McCredibilityOptions<'a> {
    /// UQ budget TOML path.
    pub uq_toml: &'a Path,
    /// Required binding credibility floor.
    pub floor: CredibilityLevel,
    /// Optional Markdown report output path.
    pub report_md: Option<&'a Path>,
}

/// Complete UQ credibility inputs expected by MC campaign drivers.
///
/// This contract keeps optional campaign sidecars and runner-gathered upstream
/// UQ on the same path, so future drivers do not need bespoke merge rules.
#[derive(Clone, Copy, Debug)]
pub struct CampaignCredibilityInputs<'a> {
    /// Optional campaign UQ sidecar declaration.
    pub sidecar: Option<McCredibilityOptions<'a>>,
    /// Runner-gathered upstream deck UQ budget.
    pub upstream_uq: &'a CorrelatedErrorBudget,
}

impl<'a> CampaignCredibilityInputs<'a> {
    /// Build MC campaign credibility inputs from a sidecar and upstream budget.
    #[must_use]
    pub const fn new(
        sidecar: Option<McCredibilityOptions<'a>>,
        upstream_uq: &'a CorrelatedErrorBudget,
    ) -> Self {
        Self {
            sidecar,
            upstream_uq,
        }
    }
}

#[derive(Clone, Debug)]
struct ResolvedMcCredibilityOptions {
    budget: CorrelatedErrorBudget,
    floor: CredibilityLevel,
    report_md: Option<PathBuf>,
}

impl McResumeScalarReport {
    /// Whether the checkpoint has all required samples.
    #[must_use]
    pub const fn is_complete(&self) -> bool {
        self.summary.is_some()
    }
}

/// Summary report for `openbmp mc propulsion-faults`.
#[derive(Clone, Debug, PartialEq)]
pub struct McPropulsionFaultCampaignReport {
    /// Number of scenario samples executed.
    pub samples: u64,
    /// Number of samples with at least one activated scheduled fault.
    pub activated_samples: u64,
    /// Sample CSV path written by the campaign.
    pub output_csv: PathBuf,
    /// Terminal metric and success summary for the campaign.
    pub summary: McSummaryReport,
    /// Optional campaign UQ/credibility evidence.
    pub credibility: Option<CampaignCredibilityReport>,
    /// Markdown report path, when written.
    pub credibility_report_md: Option<PathBuf>,
}

/// Options for an end-to-end propulsion-fault Monte Carlo campaign.
#[derive(Clone, Copy, Debug)]
pub struct McPropulsionFaultCampaignOptions<'a> {
    /// Base scenario TOML path.
    pub scenario_path: &'a Path,
    /// Fault-library TOML path.
    pub library_toml: &'a Path,
    /// Number of samples to execute.
    pub samples: u64,
    /// Campaign seed for deterministic materialization.
    pub campaign_seed: u64,
    /// Dimension id for the deterministic fault activation stream.
    pub dimension_id: u32,
    /// Telemetry channel read from the terminal row.
    pub metric_channel: &'a str,
    /// Optional inclusive lower success bound for the terminal metric.
    pub success_min: Option<f64>,
    /// Optional inclusive upper success bound for the terminal metric.
    pub success_max: Option<f64>,
    /// Output CSV path for one row per sample.
    pub output_csv: &'a Path,
    /// Confidence level for the success interval.
    pub confidence: f64,
    /// Optional UQ credibility evidence for this campaign.
    pub credibility: Option<McCredibilityOptions<'a>>,
}

#[derive(Debug, Deserialize)]
struct PropulsionFaultLibraryDocument {
    #[serde(default)]
    faults: Vec<PropulsionFaultTemplate>,
    #[serde(default)]
    uq: Option<PropulsionFaultLibraryUqDocument>,
}

#[derive(Debug, Deserialize)]
struct PropulsionFaultLibraryUqDocument {
    budget_toml: PathBuf,
    #[serde(default)]
    credibility_floor: Option<String>,
    #[serde(default)]
    report_md: Option<PathBuf>,
}

#[derive(Debug, Deserialize)]
struct UqBudgetDocument {
    #[serde(default)]
    sources: Vec<UqSourceDocument>,
    correlation: Option<UqCorrelationDocument>,
}

#[derive(Debug)]
struct CampaignArtifactWriteReport {
    artifact_paths: Vec<PathBuf>,
    manifest_path: Option<PathBuf>,
}

#[derive(Debug, Serialize)]
struct CampaignArtifactManifest {
    schema: &'static str,
    artifacts: Vec<CampaignArtifactManifestEntry>,
}

#[derive(Debug, Serialize)]
struct CampaignArtifactManifestEntry {
    role: &'static str,
    path: String,
    sha256_hex: String,
}

#[derive(Debug, Deserialize)]
struct UqSourceDocument {
    id: String,
    one_sigma: f64,
    class: UqClassDocument,
    justification: String,
    #[serde(default)]
    credibility: BTreeMap<String, UqCredibilityScoreDocument>,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum UqClassDocument {
    Aleatory,
    Epistemic,
}

#[derive(Debug, Deserialize)]
struct UqCredibilityScoreDocument {
    level: u8,
    evidence: String,
}

#[derive(Debug, Deserialize)]
struct UqCorrelationDocument {
    matrix: Vec<Vec<f64>>,
}

/// Generate and write a deterministic Latin hypercube unit-cube design CSV.
///
/// # Errors
///
/// Returns [`CliError`] when the requested design is invalid or the CSV cannot
/// be written.
pub fn write_latin_hypercube(
    samples: u64,
    dimensions: u32,
    seed: u64,
    output_csv: &Path,
) -> Result<McLhsReport, CliError> {
    let design = LatinHypercube
        .generate(seed, samples, dimensions)
        .map_err(|err| CliError::MonteCarlo {
            summary: err.to_string(),
        })?;

    let header = unit_cube_headers(dimensions);
    write_design_matrix(output_csv, &header, &design)?;

    Ok(McLhsReport {
        samples,
        dimensions,
        output_csv: output_csv.to_path_buf(),
    })
}

/// Generate and write an unscrambled Sobol unit-cube design CSV.
///
/// # Errors
///
/// Returns [`CliError`] when the requested design is invalid or the CSV cannot
/// be written.
pub fn write_sobol(
    samples: u64,
    dimensions: u32,
    scramble_seed: Option<u64>,
    output_csv: &Path,
) -> Result<McSobolReport, CliError> {
    let design = match scramble_seed {
        Some(seed) => OwenScramble::new(seed).generate(samples, dimensions),
        None => SobolSequence.generate(samples, dimensions),
    }
    .map_err(|err| CliError::MonteCarlo {
        summary: err.to_string(),
    })?;
    let header = unit_cube_headers(dimensions);
    write_design_matrix(output_csv, &header, &design)?;

    Ok(McSobolReport {
        samples,
        dimensions,
        scramble_seed,
        output_csv: output_csv.to_path_buf(),
    })
}

/// Generate a deterministic schema-v3 ascent acoustics trace TOML snippet.
///
/// # Errors
///
/// Returns [`CliError`] when the design request or physical ranges are invalid,
/// or when the output TOML cannot be written.
pub fn write_acoustics_ascent_trace(
    options: McAcousticsAscentTraceOptions<'_>,
) -> Result<McAcousticsAscentTraceReport, CliError> {
    validate_acoustics_ascent_trace_options(&options)?;
    let design = acoustics_ascent_trace_design(&options)?;
    let bytes = acoustics_ascent_trace_toml(&options, &design)?.into_bytes();
    ensure_parent_dir(options.output_toml)?;
    fs::write(options.output_toml, bytes).map_err(|source| CliError::Io {
        path: options.output_toml.to_path_buf(),
        source,
    })?;
    Ok(McAcousticsAscentTraceReport {
        samples: options.samples,
        design: options.design,
        output_toml: options.output_toml.to_path_buf(),
    })
}

/// Generate a deterministic schema-v3 vibroacoustic response TOML snippet.
///
/// # Errors
///
/// Returns [`CliError`] when the design request or physical ranges are invalid,
/// or when the output TOML cannot be written.
pub fn write_acoustics_response_samples(
    options: McAcousticsResponseSamplesOptions<'_>,
) -> Result<McAcousticsResponseSamplesReport, CliError> {
    validate_acoustics_response_samples_options(&options)?;
    let design = acoustics_response_samples_design(&options)?;
    let bytes = acoustics_response_samples_toml(&options, &design)?.into_bytes();
    ensure_parent_dir(options.output_toml)?;
    fs::write(options.output_toml, bytes).map_err(|source| CliError::Io {
        path: options.output_toml.to_path_buf(),
        source,
    })?;
    Ok(McAcousticsResponseSamplesReport {
        samples: options.samples,
        design: options.design,
        output_toml: options.output_toml.to_path_buf(),
    })
}

/// Generate deterministic per-sample aero deck-draw campaign overlays.
///
/// # Errors
///
/// Returns [`CliError`] when the schedule request is invalid, when the sample
/// index range overflows, or when any overlay/manifest TOML cannot be written.
pub fn write_aero_deck_draw_overlays(
    options: McAeroDeckDrawOverlaysOptions<'_>,
) -> Result<McAeroDeckDrawOverlaysReport, CliError> {
    validate_aero_deck_draw_overlays_options(&options)?;
    create_dir_all(options.output_dir).map_err(|source| CliError::Io {
        path: options.output_dir.to_path_buf(),
        source,
    })?;
    ensure_parent_dir(options.manifest_toml)?;

    let overlay_prefix = options.overlay_prefix.trim();
    let mut overlay_paths = Vec::new();
    let mut manifest = String::new();
    let _ = writeln!(
        manifest,
        "# Generated by openbmp mc aero-deck-draw-overlays."
    );
    let _ = writeln!(
        manifest,
        "# Paste this fragment into an openbmp campaign manifest."
    );
    let _ = writeln!(manifest, "overlays = [");

    for offset in 0..options.samples {
        let sample_index = options
            .first_sample_index
            .checked_add(offset)
            .ok_or_else(|| CliError::MonteCarlo {
                summary: "aero deck-draw sample index range overflows u64".to_owned(),
            })?;
        let overlay_id = format!("{overlay_prefix}-{sample_index:06}");
        let file_name = format!("{overlay_id}.toml");
        let overlay_path = options.output_dir.join(&file_name);
        let overlay_toml = aero_deck_draw_overlay_toml(&options, sample_index);
        fs::write(&overlay_path, overlay_toml).map_err(|source| CliError::Io {
            path: overlay_path.clone(),
            source,
        })?;
        let _ = writeln!(manifest, "  {},", toml_string(&overlay_id));
        overlay_paths.push(overlay_path);
    }

    let _ = writeln!(manifest, "]");
    for (path_index, overlay_path) in overlay_paths.iter().enumerate() {
        let sample_index = options.first_sample_index + path_index as u64;
        let overlay_id = format!("{overlay_prefix}-{sample_index:06}");
        let file_name = overlay_path
            .file_name()
            .and_then(|name| name.to_str())
            .ok_or_else(|| CliError::MonteCarlo {
                summary: "generated aero deck-draw overlay filename is not UTF-8".to_owned(),
            })?;
        let path =
            campaign_manifest_overlay_path(options.manifest_toml, options.output_dir, file_name);
        let _ = writeln!(manifest);
        let _ = writeln!(manifest, "[[overlay_files]]");
        let _ = writeln!(manifest, "id = {}", toml_string(&overlay_id));
        let _ = writeln!(manifest, "path = {}", toml_string(&path));
        let _ = writeln!(manifest, "mode = \"merge\"");
    }

    fs::write(options.manifest_toml, manifest).map_err(|source| CliError::Io {
        path: options.manifest_toml.to_path_buf(),
        source,
    })?;

    Ok(McAeroDeckDrawOverlaysReport {
        samples: options.samples,
        campaign_seed: options.campaign_seed,
        first_sample_index: options.first_sample_index,
        output_dir: options.output_dir.to_path_buf(),
        manifest_toml: options.manifest_toml.to_path_buf(),
        overlay_paths,
    })
}

/// Generate and execute a deterministic aero deck-draw campaign.
///
/// # Errors
///
/// Returns [`CliError`] when schedule generation, generated-manifest writing,
/// or the underlying campaign runner fails.
pub fn run_aero_deck_draw_campaign(
    options: McAeroDeckDrawCampaignOptions<'_>,
) -> Result<McAeroDeckDrawCampaignReport, CliError> {
    validate_aero_deck_draw_campaign_options(&options)?;
    create_dir_all(options.output_dir).map_err(|source| CliError::Io {
        path: options.output_dir.to_path_buf(),
        source,
    })?;

    let base_scenario = Scenario::from_file(options.scenario_path)?;
    let resolved_files = base_scenario.resolved_files()?;
    if let Some(required_dimension_count) =
        openbmp_runner::aero::correlated_deck_draw_required_dimensions(
            &base_scenario.document,
            &resolved_files,
        )?
    {
        validate_aero_deck_draw_campaign_dimension_capacity(
            options.dimension_id_base,
            required_dimension_count,
        )?;
    }

    let overlay_dir = options.output_dir.join("overlays");
    let overlay_fragment_toml = options.output_dir.join("aero-deck-draw-overlays.toml");
    let overlays = write_aero_deck_draw_overlays(McAeroDeckDrawOverlaysOptions {
        samples: options.samples,
        campaign_seed: options.campaign_seed,
        first_sample_index: options.first_sample_index,
        dimension_id_base: options.dimension_id_base,
        mach_correlation_length: options.mach_correlation_length,
        alpha_correlation_length_deg: options.alpha_correlation_length_deg,
        beta_correlation_length_deg: options.beta_correlation_length_deg,
        overlay_prefix: options.overlay_prefix,
        output_dir: &overlay_dir,
        manifest_toml: &overlay_fragment_toml,
    })?;

    let campaign_manifest_toml = options.output_dir.join("campaign.toml");
    let manifest_text =
        aero_deck_draw_campaign_manifest_toml(&options, &overlays, &campaign_manifest_toml)?;
    fs::write(&campaign_manifest_toml, manifest_text).map_err(|source| CliError::Io {
        path: campaign_manifest_toml.clone(),
        source,
    })?;

    let upstream_uq = openbmp_runner::aero::upstream_uq_budget(&base_scenario.document)?;
    let credibility = evaluate_campaign_credibility_contract(CampaignCredibilityInputs::new(
        options.credibility,
        &upstream_uq,
    ))?;
    let (credibility, credibility_report_md) = match credibility {
        Some((report, report_path)) => (Some(report), report_path),
        None => (None, None),
    };

    let campaign_output_dir = options.output_dir.join("campaign-run");
    let campaign = campaign::run(
        &campaign_manifest_toml,
        &campaign_output_dir,
        options.run_code_id,
        options.code_ordinal,
        options.write_index_parquet,
        options.workers,
    )?;
    Ok(McAeroDeckDrawCampaignReport {
        samples: options.samples,
        overlays,
        campaign_manifest_toml,
        campaign_output_dir,
        campaign,
        credibility,
        credibility_report_md,
    })
}

/// Generate an MC-designed ascent trace, execute the normal scenario runner,
/// and write the evaluated per-sample acoustic trace CSV.
///
/// # Errors
///
/// Returns [`CliError`] for invalid campaign options, malformed base scenario
/// TOML, scenario/runner failures, missing ascent reports, or output IO
/// failures.
pub fn run_acoustics_ascent_campaign(
    options: McAcousticsAscentCampaignOptions<'_>,
) -> Result<McAcousticsAscentCampaignReport, CliError> {
    run_acoustics_ascent_campaign_with_extra_overlay(options, None)
}

fn run_acoustics_ascent_campaign_with_extra_overlay(
    options: McAcousticsAscentCampaignOptions<'_>,
    extra_overlay_toml: Option<&str>,
) -> Result<McAcousticsAscentCampaignReport, CliError> {
    let trace_output_path = options.output_scenario_toml.unwrap_or(options.output_csv);
    let trace_options = McAcousticsAscentTraceOptions {
        samples: options.samples,
        campaign_seed: options.campaign_seed,
        design: options.design,
        dynamic_pressure_min_pa: options.dynamic_pressure_min_pa,
        dynamic_pressure_max_pa: options.dynamic_pressure_max_pa,
        mach_min: options.mach_min,
        mach_max: options.mach_max,
        sample_prefix: options.sample_prefix,
        class_id: options.class_id,
        output_toml: trace_output_path,
    };
    validate_acoustics_ascent_trace_options(&trace_options)?;
    validate_acoustics_ascent_campaign_checkpoint_options(&options)?;
    let base_scenario =
        fs::read_to_string(options.scenario_path).map_err(|source| CliError::Io {
            path: options.scenario_path.to_path_buf(),
            source,
        })?;
    validate_base_acoustics_ascent_campaign_scenario(&base_scenario)?;
    let design = acoustics_ascent_trace_design(&trace_options)?;
    let trace_toml = acoustics_ascent_trace_toml(&trace_options, &design)?;
    let mut campaign_scenario = append_acoustics_ascent_trace_overlay(&base_scenario, &trace_toml);
    if let Some(extra_overlay_toml) = extra_overlay_toml {
        campaign_scenario =
            append_acoustics_ascent_trace_overlay(&campaign_scenario, extra_overlay_toml);
    }
    let campaign_scenario_sha256_hex = sha256_bytes(campaign_scenario.as_bytes());
    let source_dir = options.scenario_path.parent().map(Path::to_path_buf);

    if let Some(checkpoint_json) = options.checkpoint_json {
        return run_checkpointed_acoustics_ascent_campaign(
            &options,
            &trace_options,
            &base_scenario,
            &campaign_scenario,
            &campaign_scenario_sha256_hex,
            &source_dir,
            &design,
            checkpoint_json,
        );
    }

    finish_acoustics_ascent_campaign(
        &options,
        &campaign_scenario,
        &source_dir,
        options.samples,
        None,
    )
}

/// Generate an MC-designed ascent acoustic trace, require a downstream
/// `[acoustics.response]` declaration, and report response P95/50 evidence.
///
/// # Errors
///
/// Returns [`CliError`] for the same conditions as
/// [`run_acoustics_ascent_campaign`], and additionally when the base scenario
/// lacks `[acoustics.response]` or a completed run does not emit response
/// P95/50 summary evidence.
pub fn run_acoustics_response_campaign(
    options: McAcousticsResponseCampaignOptions<'_>,
) -> Result<McAcousticsResponseCampaignReport, CliError> {
    let base_scenario =
        fs::read_to_string(options.scenario_path).map_err(|source| CliError::Io {
            path: options.scenario_path.to_path_buf(),
            source,
        })?;
    validate_base_acoustics_response_campaign_scenario(
        &base_scenario,
        options.response_samples.is_some(),
    )?;

    let response_samples_toml = options
        .response_samples
        .as_ref()
        .map(|response_samples| {
            acoustics_response_campaign_samples_toml(&options, response_samples)
        })
        .transpose()?;
    let ascent_options = acoustics_response_campaign_ascent_options(&options);
    let report = run_acoustics_ascent_campaign_with_extra_overlay(
        ascent_options,
        response_samples_toml.as_deref(),
    )?;
    if report.complete
        && (report
            .response_p95_50_total_mean_square_acceleration_m2_s4
            .is_none()
            || report.response_p95_50_total_mean_square_force_n2.is_none()
            || report.response_p95_50_wilks_required_sample_count.is_none())
    {
        return Err(CliError::MonteCarlo {
            summary: "acoustics response campaign scenario did not emit response P95/50 evidence"
                .to_owned(),
        });
    }
    Ok(report)
}

fn acoustics_response_campaign_ascent_options<'a>(
    options: &'a McAcousticsResponseCampaignOptions<'a>,
) -> McAcousticsAscentCampaignOptions<'a> {
    McAcousticsAscentCampaignOptions {
        scenario_path: options.scenario_path,
        samples: options.samples,
        campaign_seed: options.campaign_seed,
        design: options.design,
        dynamic_pressure_min_pa: options.dynamic_pressure_min_pa,
        dynamic_pressure_max_pa: options.dynamic_pressure_max_pa,
        mach_min: options.mach_min,
        mach_max: options.mach_max,
        sample_prefix: options.sample_prefix,
        class_id: options.class_id,
        output_csv: options.output_csv,
        output_artifact_dir: options.output_artifact_dir,
        output_scenario_toml: options.output_scenario_toml,
        checkpoint_json: options.checkpoint_json,
        max_new_samples: options.max_new_samples,
        workers: options.workers,
        toolchain_profile: options.toolchain_profile,
    }
}

fn acoustics_response_campaign_samples_toml(
    options: &McAcousticsResponseCampaignOptions<'_>,
    response_samples: &McAcousticsResponseCampaignSamplesOptions<'_>,
) -> Result<String, CliError> {
    let output_toml = options.output_scenario_toml.unwrap_or(options.output_csv);
    let sample_options = McAcousticsResponseSamplesOptions {
        samples: options.samples,
        campaign_seed: options.campaign_seed,
        design: options.design,
        surface_mass_min_kg_m2: response_samples.surface_mass_min_kg_m2,
        surface_mass_max_kg_m2: response_samples.surface_mass_max_kg_m2,
        area_min_m2: response_samples.area_min_m2,
        area_max_m2: response_samples.area_max_m2,
        mode_shape_min: response_samples.mode_shape_min,
        mode_shape_max: response_samples.mode_shape_max,
        sample_prefix: response_samples.sample_prefix,
        class_id: response_samples.class_id,
        output_toml,
    };
    validate_acoustics_response_samples_options(&sample_options)?;
    let design = acoustics_response_samples_design(&sample_options)?;
    acoustics_response_samples_toml(&sample_options, &design)
}

#[allow(clippy::too_many_arguments)]
fn run_checkpointed_acoustics_ascent_campaign(
    options: &McAcousticsAscentCampaignOptions<'_>,
    trace_options: &McAcousticsAscentTraceOptions<'_>,
    base_scenario: &str,
    campaign_scenario: &str,
    campaign_scenario_sha256_hex: &str,
    source_dir: &Option<PathBuf>,
    design: &DesignMatrix,
    checkpoint_json: &Path,
) -> Result<McAcousticsAscentCampaignReport, CliError> {
    let max_new_samples = options.max_new_samples.unwrap_or(options.samples);
    let store = FileCheckpointStore::new(checkpoint_json.to_path_buf());
    let mut checkpoint = match store.load().map_err(checkpoint_store_error)? {
        Some(checkpoint) => {
            checkpoint
                .validate_campaign(
                    options.campaign_seed,
                    options.samples,
                    ACOUSTICS_ASCENT_CAMPAIGN_DIMENSION_ID,
                    campaign_scenario_sha256_hex,
                    options.toolchain_profile,
                )
                .map_err(monte_carlo_error)?;
            checkpoint
        }
        None => ScalarCampaignCheckpoint::new(
            options.campaign_seed,
            options.samples,
            ACOUSTICS_ASCENT_CAMPAIGN_DIMENSION_ID,
            campaign_scenario_sha256_hex.to_owned(),
            options.toolchain_profile.to_owned(),
        )
        .map_err(monte_carlo_error)?,
    };

    let checkpoint_report = run_scalar_campaign_resumable(
        &mut checkpoint,
        options.workers,
        max_new_samples,
        |sample_index, _rng| {
            evaluate_acoustics_ascent_campaign_checkpoint_sample(
                base_scenario,
                source_dir.as_deref(),
                trace_options,
                design,
                sample_index,
            )
        },
    )
    .map_err(monte_carlo_error)?;
    store.save(&checkpoint).map_err(checkpoint_store_error)?;

    let completed_samples = checkpoint.completed_count();
    if checkpoint_report.is_none() {
        return Ok(incomplete_acoustics_ascent_campaign_report(
            options,
            checkpoint_json,
            completed_samples,
        ));
    }

    finish_acoustics_ascent_campaign(
        options,
        campaign_scenario,
        source_dir,
        completed_samples,
        Some(checkpoint_json),
    )
}

fn finish_acoustics_ascent_campaign(
    options: &McAcousticsAscentCampaignOptions<'_>,
    campaign_scenario: &str,
    source_dir: &Option<PathBuf>,
    completed_samples: u64,
    checkpoint_json: Option<&Path>,
) -> Result<McAcousticsAscentCampaignReport, CliError> {
    if let Some(path) = options.output_scenario_toml {
        ensure_parent_dir(path)?;
        fs::write(path, campaign_scenario.as_bytes()).map_err(|source| CliError::Io {
            path: path.to_path_buf(),
            source,
        })?;
    }

    let scenario = Scenario::from_toml_str_with_source_dir(campaign_scenario, source_dir.clone())?;
    let outcome = openbmp_runner::run(&scenario)?;
    let ascent = outcome
        .acoustics
        .as_ref()
        .and_then(|acoustics| acoustics.ascent.as_ref())
        .ok_or_else(|| CliError::MonteCarlo {
            summary: "acoustics ascent campaign scenario did not emit RunOutcome.acoustics.ascent"
                .to_owned(),
        })?;
    let expected_samples = usize::try_from(options.samples).map_err(|_| CliError::MonteCarlo {
        summary: "acoustics ascent campaign sample count does not fit usize".to_owned(),
    })?;
    if ascent.sample_count != expected_samples {
        return Err(CliError::MonteCarlo {
            summary: format!(
                "acoustics ascent campaign emitted {} samples, expected {}",
                ascent.sample_count, expected_samples
            ),
        });
    }
    let fairing_internal = outcome
        .acoustics
        .as_ref()
        .and_then(|acoustics| acoustics.fairing_internal.as_ref());
    let response = outcome
        .acoustics
        .as_ref()
        .and_then(|acoustics| acoustics.response.as_ref());
    ensure_parent_dir(options.output_csv)?;
    fs::write(options.output_csv, &ascent.sample_trace_csv).map_err(|source| CliError::Io {
        path: options.output_csv.to_path_buf(),
        source,
    })?;
    let artifact_report = write_acoustics_ascent_campaign_artifacts(options, &outcome)?;

    Ok(McAcousticsAscentCampaignReport {
        scenario_path: options.scenario_path.to_path_buf(),
        samples: options.samples,
        design: options.design,
        output_csv: options.output_csv.to_path_buf(),
        output_artifact_dir: options.output_artifact_dir.map(Path::to_path_buf),
        artifact_paths: artifact_report.artifact_paths,
        artifact_manifest_json: artifact_report.manifest_path,
        output_scenario_toml: options.output_scenario_toml.map(Path::to_path_buf),
        checkpoint_json: checkpoint_json.map(Path::to_path_buf),
        completed_samples,
        complete: true,
        p95_50_oaspl_db: ascent.p95_50_oaspl_db,
        p95_50_wilks_required_sample_count: ascent.p95_50_wilks_required_sample_count,
        p95_50_wilks_sample_count_sufficient: ascent.p95_50_wilks_sample_count_sufficient,
        fairing_internal_p95_50_oaspl_db: fairing_internal
            .and_then(|fairing| fairing.p95_50_oaspl_db),
        fairing_internal_p95_50_wilks_required_sample_count: fairing_internal
            .and_then(|fairing| fairing.p95_50_wilks_required_sample_count),
        fairing_internal_p95_50_wilks_sample_count_sufficient: fairing_internal
            .and_then(|fairing| fairing.p95_50_wilks_sample_count_sufficient),
        response_p95_50_total_mean_square_acceleration_m2_s4: response
            .and_then(|response| response.p95_50_total_mean_square_acceleration_m2_s4),
        response_p95_50_total_mean_square_force_n2: response
            .and_then(|response| response.p95_50_total_mean_square_force_n2),
        response_p95_50_wilks_required_sample_count: response
            .and_then(|response| response.p95_50_wilks_required_sample_count),
        response_p95_50_wilks_sample_count_sufficient: response
            .and_then(|response| response.p95_50_wilks_sample_count_sufficient),
        peak_sample_id: ascent.peak_sample_id.clone(),
        peak_sample_oaspl_db: ascent.peak_sample_oaspl_db,
    })
}

fn incomplete_acoustics_ascent_campaign_report(
    options: &McAcousticsAscentCampaignOptions<'_>,
    checkpoint_json: &Path,
    completed_samples: u64,
) -> McAcousticsAscentCampaignReport {
    McAcousticsAscentCampaignReport {
        scenario_path: options.scenario_path.to_path_buf(),
        samples: options.samples,
        design: options.design,
        output_csv: options.output_csv.to_path_buf(),
        output_artifact_dir: options.output_artifact_dir.map(Path::to_path_buf),
        artifact_paths: Vec::new(),
        artifact_manifest_json: None,
        output_scenario_toml: options.output_scenario_toml.map(Path::to_path_buf),
        checkpoint_json: Some(checkpoint_json.to_path_buf()),
        completed_samples,
        complete: false,
        p95_50_oaspl_db: None,
        p95_50_wilks_required_sample_count: None,
        p95_50_wilks_sample_count_sufficient: None,
        fairing_internal_p95_50_oaspl_db: None,
        fairing_internal_p95_50_wilks_required_sample_count: None,
        fairing_internal_p95_50_wilks_sample_count_sufficient: None,
        response_p95_50_total_mean_square_acceleration_m2_s4: None,
        response_p95_50_total_mean_square_force_n2: None,
        response_p95_50_wilks_required_sample_count: None,
        response_p95_50_wilks_sample_count_sufficient: None,
        peak_sample_id: None,
        peak_sample_oaspl_db: f64::NAN,
    }
}

fn write_acoustics_ascent_campaign_artifacts(
    options: &McAcousticsAscentCampaignOptions<'_>,
    outcome: &openbmp_runner::RunOutcome,
) -> Result<CampaignArtifactWriteReport, CliError> {
    let Some(base_dir) = options.output_artifact_dir else {
        return Ok(CampaignArtifactWriteReport {
            artifact_paths: Vec::new(),
            manifest_path: None,
        });
    };
    let Some(acoustics) = &outcome.acoustics else {
        return write_campaign_artifact_manifest(base_dir, Vec::new(), Vec::new());
    };
    let mut written = Vec::new();
    let mut manifest_entries = Vec::new();

    if let Some(ascent) = &acoustics.ascent {
        write_optional_campaign_artifact(
            base_dir,
            "acoustics_ascent_p95_50_environment_csv",
            Path::new("ascent/p95_50_environment.csv"),
            &ascent.p95_50_environment_csv,
            &mut written,
            &mut manifest_entries,
        )?;
        write_optional_campaign_artifact(
            base_dir,
            "acoustics_ascent_p95_50_wilks_sizing_csv",
            Path::new("ascent/p95_50_wilks_sizing.csv"),
            &ascent.p95_50_wilks_sizing_csv,
            &mut written,
            &mut manifest_entries,
        )?;
    }

    if let Some(fairing) = &acoustics.fairing_internal {
        write_optional_campaign_artifact(
            base_dir,
            "acoustics_fairing_internal_p95_50_environment_csv",
            Path::new("fairing_internal/p95_50_environment.csv"),
            &fairing.p95_50_environment_csv,
            &mut written,
            &mut manifest_entries,
        )?;
        write_optional_campaign_artifact(
            base_dir,
            "acoustics_fairing_internal_p95_50_sample_trace_csv",
            Path::new("fairing_internal/p95_50_sample_trace.csv"),
            &fairing.p95_50_sample_trace_csv,
            &mut written,
            &mut manifest_entries,
        )?;
        write_optional_campaign_artifact(
            base_dir,
            "acoustics_fairing_internal_p95_50_wilks_sizing_csv",
            Path::new("fairing_internal/p95_50_wilks_sizing.csv"),
            &fairing.p95_50_wilks_sizing_csv,
            &mut written,
            &mut manifest_entries,
        )?;
    }

    if let Some(response) = &acoustics.response {
        write_optional_campaign_artifact(
            base_dir,
            "acoustics_response_loads_handoff_csv",
            Path::new("response/loads_handoff.csv"),
            &response.loads_handoff_csv,
            &mut written,
            &mut manifest_entries,
        )?;
        write_optional_campaign_artifact(
            base_dir,
            "acoustics_response_p95_50_acceleration_psd_csv",
            Path::new("response/p95_50_acceleration_psd.csv"),
            &response.p95_50_acceleration_psd_csv,
            &mut written,
            &mut manifest_entries,
        )?;
        write_optional_campaign_artifact(
            base_dir,
            "acoustics_response_p95_50_modal_force_psd_csv",
            Path::new("response/p95_50_modal_force_psd.csv"),
            &response.p95_50_modal_force_psd_csv,
            &mut written,
            &mut manifest_entries,
        )?;
        write_optional_campaign_artifact(
            base_dir,
            "acoustics_response_p95_50_sample_trace_csv",
            Path::new("response/p95_50_sample_trace.csv"),
            &response.p95_50_sample_trace_csv,
            &mut written,
            &mut manifest_entries,
        )?;
        write_optional_campaign_artifact(
            base_dir,
            "acoustics_response_p95_50_wilks_sizing_csv",
            Path::new("response/p95_50_wilks_sizing.csv"),
            &response.p95_50_wilks_sizing_csv,
            &mut written,
            &mut manifest_entries,
        )?;
    }

    if let Some(structural) = &outcome.structural_acoustics {
        write_optional_campaign_artifact(
            base_dir,
            "structural_acoustics_first_bending_response_csv",
            Path::new("structural/first_bending_response.csv"),
            &structural.band_response_csv,
            &mut written,
            &mut manifest_entries,
        )?;
        write_optional_campaign_artifact(
            base_dir,
            "structural_acoustics_first_bending_line_load_recovery_csv",
            Path::new("structural/first_bending_line_load_recovery.csv"),
            &structural.line_load_recovery_csv,
            &mut written,
            &mut manifest_entries,
        )?;
    }

    written.sort();
    manifest_entries.sort_by(|left, right| left.path.cmp(&right.path));
    write_campaign_artifact_manifest(base_dir, written, manifest_entries)
}

fn write_optional_campaign_artifact(
    base_dir: &Path,
    role: &'static str,
    relative_path: &Path,
    bytes: &[u8],
    written: &mut Vec<PathBuf>,
    manifest_entries: &mut Vec<CampaignArtifactManifestEntry>,
) -> Result<(), CliError> {
    if bytes.is_empty() {
        return Ok(());
    }
    let path = base_dir.join(relative_path);
    ensure_parent_dir(&path)?;
    fs::write(&path, bytes).map_err(|source| CliError::Io {
        path: path.clone(),
        source,
    })?;
    written.push(path);
    manifest_entries.push(CampaignArtifactManifestEntry {
        role,
        path: relative_path.to_string_lossy().replace('\\', "/"),
        sha256_hex: sha256_bytes(bytes),
    });
    Ok(())
}

fn write_campaign_artifact_manifest(
    base_dir: &Path,
    artifact_paths: Vec<PathBuf>,
    manifest_entries: Vec<CampaignArtifactManifestEntry>,
) -> Result<CampaignArtifactWriteReport, CliError> {
    let manifest = CampaignArtifactManifest {
        schema: "openbmp.acoustics_ascent_campaign_artifacts.v1",
        artifacts: manifest_entries,
    };
    let mut bytes =
        serde_json::to_vec_pretty(&manifest).map_err(|source| CliError::MonteCarlo {
            summary: format!("could not serialize acoustics campaign artifact manifest: {source}"),
        })?;
    bytes.push(b'\n');
    let manifest_path = base_dir.join("manifest.json");
    ensure_parent_dir(&manifest_path)?;
    fs::write(&manifest_path, bytes).map_err(|source| CliError::Io {
        path: manifest_path.clone(),
        source,
    })?;
    Ok(CampaignArtifactWriteReport {
        artifact_paths,
        manifest_path: Some(manifest_path),
    })
}

/// Read a design matrix and write an Iman-Conover reordered design CSV.
///
/// `design_csv` must have a `sample_index` first column followed by unit-cube
/// design columns. `correlation_csv` is read without headers and must contain
/// a square numeric target correlation matrix with the same dimension count.
///
/// # Errors
///
/// Returns [`CliError`] when either CSV is malformed, the correlation matrix is
/// invalid, or the output CSV cannot be written.
pub fn write_iman_conover(
    design_csv: &Path,
    correlation_csv: &Path,
    output_csv: &Path,
) -> Result<McImanConoverReport, CliError> {
    let (headers, design) = read_design_matrix(design_csv)?;
    let target = read_correlation_matrix(correlation_csv, design.columns())?;
    let induced = ImanConover::new(target)
        .apply(&design)
        .map_err(|err| CliError::MonteCarlo {
            summary: err.to_string(),
        })?;
    write_design_matrix(output_csv, &headers, &induced)?;
    Ok(McImanConoverReport {
        samples: induced.rows(),
        dimensions: induced.columns(),
        output_csv: output_csv.to_path_buf(),
    })
}

/// Compute a Wilks sample-size report.
///
/// # Errors
///
/// Returns [`CliError`] when `coverage` or `confidence` is invalid.
pub fn wilks_sample_size(
    side: WilksSide,
    coverage: f64,
    confidence: f64,
) -> Result<McWilksReport, CliError> {
    let (samples, attained_confidence) = match side {
        WilksSide::OneSided => {
            let samples =
                wilks_one_sided_n(coverage, confidence).map_err(|err| CliError::MonteCarlo {
                    summary: err.to_string(),
                })?;
            let attained_confidence =
                wilks_one_sided_confidence(coverage, samples).map_err(|err| {
                    CliError::MonteCarlo {
                        summary: err.to_string(),
                    }
                })?;
            (samples, attained_confidence)
        }
        WilksSide::TwoSided => {
            let samples =
                wilks_two_sided_n(coverage, confidence).map_err(|err| CliError::MonteCarlo {
                    summary: err.to_string(),
                })?;
            let attained_confidence =
                wilks_two_sided_confidence(coverage, samples).map_err(|err| {
                    CliError::MonteCarlo {
                        summary: err.to_string(),
                    }
                })?;
            (samples, attained_confidence)
        }
    };

    Ok(McWilksReport {
        side,
        coverage,
        confidence,
        samples,
        attained_confidence,
    })
}

/// Resume a scalar checkpoint from a precomputed scalar sample CSV.
///
/// This utility is intended for deterministic campaign evidence workflows where
/// sample outcomes have already been materialized. It records at most
/// `max_new_samples` missing sample indices, saves the checkpoint, and returns a
/// final summary once all samples are complete.
///
/// # Errors
///
/// Returns [`CliError`] for checkpoint IO/JSON/content errors, CSV errors,
/// missing or malformed columns, invalid metadata, or invalid Monte Carlo
/// statistics.
#[allow(clippy::too_many_arguments)]
pub fn resume_scalar_checkpoint(
    samples_csv: &Path,
    checkpoint_json: &Path,
    value_column: &str,
    success_column: Option<&str>,
    sample_count: u64,
    campaign_seed: u64,
    dimension_id: u32,
    scenario_sha256: &str,
    toolchain_profile: &str,
    max_new_samples: u64,
    workers: usize,
    confidence: f64,
) -> Result<McResumeScalarReport, CliError> {
    let rows = read_scalar_rows(samples_csv, value_column, success_column)?;
    let row_map = rows
        .into_iter()
        .map(|row| (row.sample_index, (row.value, row.success)))
        .collect::<BTreeMap<_, _>>();
    let store = FileCheckpointStore::new(checkpoint_json.to_path_buf());
    let mut checkpoint = match store.load().map_err(checkpoint_store_error)? {
        Some(checkpoint) => {
            checkpoint
                .validate_campaign(
                    campaign_seed,
                    sample_count,
                    dimension_id,
                    scenario_sha256,
                    toolchain_profile,
                )
                .map_err(monte_carlo_error)?;
            checkpoint
        }
        None => ScalarCampaignCheckpoint::new(
            campaign_seed,
            sample_count,
            dimension_id,
            scenario_sha256.to_owned(),
            toolchain_profile.to_owned(),
        )
        .map_err(monte_carlo_error)?,
    };

    let report = run_scalar_campaign_resumable(
        &mut checkpoint,
        workers,
        max_new_samples,
        |sample_index, _rng| {
            let Some((value, success)) = row_map.get(&sample_index) else {
                return Err(MonteCarloError::InvalidInput {
                    field: "sample_index",
                });
            };
            ScalarOutcome::new(*value, *success)
        },
    )
    .map_err(monte_carlo_error)?;
    store.save(&checkpoint).map_err(checkpoint_store_error)?;

    let summary = report
        .map(|report| summary_from_campaign_report(&report, success_column.is_some(), confidence))
        .transpose()?;
    Ok(McResumeScalarReport {
        completed: checkpoint.completed_count(),
        sample_count: checkpoint.sample_count(),
        checkpoint_json: checkpoint_json.to_path_buf(),
        summary,
    })
}

/// Summarize scalar Monte Carlo sample rows from CSV.
///
/// If the file contains a `sample_index` column it is used to preserve
/// the declared sample indices; otherwise row order defines the sample
/// index. `success_column`, when present, accepts `true`/`false` or
/// `1`/`0`.
///
/// # Errors
///
/// Returns [`CliError`] for IO, CSV, missing columns, malformed scalar
/// values, malformed success values, or invalid confidence.
pub fn summarize(
    samples_csv: &Path,
    value_column: &str,
    success_column: Option<&str>,
    confidence: f64,
) -> Result<McSummaryReport, CliError> {
    let rows = read_scalar_rows(samples_csv, value_column, success_column)?;
    let value_stats =
        Welford::from_indexed_samples(rows.iter().map(|row| (row.sample_index, row.value)))
            .map_err(monte_carlo_error)?;

    let success = success_column
        .map(|_| success_summary(rows.iter().map(|row| row.success), confidence))
        .transpose()?;

    Ok(McSummaryReport {
        samples: value_stats.count(),
        mean: value_stats.mean(),
        sample_variance: value_stats.sample_variance(),
        standard_error: value_stats.standard_error(),
        success,
    })
}

/// Summarize scalar sample rows and optionally attach UQ credibility evidence.
///
/// # Errors
///
/// Returns [`CliError`] for scalar summary errors, malformed UQ TOML, failed UQ
/// validation, Markdown report write failures, or a binding credibility level
/// below the requested floor.
pub fn summarize_with_credibility(
    samples_csv: &Path,
    value_column: &str,
    success_column: Option<&str>,
    confidence: f64,
    credibility: Option<McCredibilityOptions<'_>>,
) -> Result<McScalarSummaryReport, CliError> {
    let summary = summarize(samples_csv, value_column, success_column, confidence)?;
    let credibility = evaluate_campaign_credibility_contract(CampaignCredibilityInputs::new(
        credibility,
        &CorrelatedErrorBudget::default(),
    ))?;
    let (credibility, credibility_report_md) = match credibility {
        Some((report, report_path)) => (Some(report), report_path),
        None => (None, None),
    };

    Ok(McScalarSummaryReport {
        summary,
        credibility,
        credibility_report_md,
    })
}

/// Summarize nested aleatory/epistemic scalar sample rows from CSV.
///
/// The CSV must contain one scalar quantity of interest column and one
/// epistemic-condition index column. All rows with the same epistemic index are
/// treated as the inner aleatory sample set for that condition.
///
/// # Errors
///
/// Returns [`CliError`] for CSV errors, missing or malformed columns, invalid
/// nested samples, invalid lower-tail requirement inputs, or p-box CSV write
/// failures.
pub fn summarize_nested(
    samples_csv: &Path,
    epistemic_column: &str,
    value_column: &str,
    threshold: f64,
    minimum_probability: f64,
    pbox_csv: Option<&Path>,
) -> Result<McNestedSummaryReport, CliError> {
    let conditional_samples =
        read_nested_scalar_samples(samples_csv, epistemic_column, value_column)?;
    let analysis = analyze_nested_scalar_samples(
        &conditional_samples,
        Some(NestedLowerTailRequirement {
            threshold,
            minimum_probability,
        }),
    )
    .map_err(monte_carlo_error)?;
    let pbox_path = pbox_csv.map(Path::to_path_buf);
    if let Some(path) = &pbox_path {
        write_pbox_csv(path, &analysis)?;
    }
    Ok(McNestedSummaryReport {
        analysis,
        pbox_csv: pbox_path,
    })
}

/// Run synthetic rare-event estimators declared in a scenario
/// `[monte_carlo]` block.
///
/// # Errors
///
/// Returns [`CliError`] for scenario parse/validation errors, missing
/// `[monte_carlo]`, invalid estimator construction, estimation failures, or
/// TOML evidence write failures.
pub fn run_rare_event_scenario(
    options: McRareEventScenarioOptions<'_>,
) -> Result<McRareEventScenarioReport, CliError> {
    let scenario = Scenario::from_file(options.scenario_path)?;
    let config = scenario
        .document
        .monte_carlo
        .as_ref()
        .ok_or_else(|| CliError::MonteCarlo {
            summary: "rare-event run requires [monte_carlo]".to_owned(),
        })?;
    let limit_state_config = config
        .limit_state
        .as_ref()
        .ok_or_else(|| CliError::MonteCarlo {
            summary: "rare-event run requires [monte_carlo.limit_state]".to_owned(),
        })?;
    let dimension =
        usize::try_from(limit_state_config.dimension).map_err(|_| CliError::MonteCarlo {
            summary: "monte_carlo.limit_state.dimension does not fit usize".to_owned(),
        })?;
    let limit_state = SyntheticLinearLimitState::new(limit_state_config.beta, dimension)
        .map_err(monte_carlo_error)?;
    let seed = config.seed.unwrap_or(scenario.document.time.seed);
    let estimates = run_rare_event_estimators(config, seed, &limit_state)?;
    let output_toml = options.output_toml.map(Path::to_path_buf);
    let report = McRareEventScenarioReport {
        scenario_path: options.scenario_path.to_path_buf(),
        seed,
        limit_state_label: limit_state_config.label.clone(),
        dimension,
        estimates,
        output_toml,
    };
    if let Some(path) = &report.output_toml {
        write_rare_event_report_toml(path, &report)?;
    }
    Ok(report)
}

fn run_rare_event_estimators(
    config: &MonteCarloConfig,
    seed: u64,
    limit_state: &SyntheticLinearLimitState,
) -> Result<Vec<RareEventEstimateReport>, CliError> {
    let mut estimates = Vec::new();
    if let Some(subset) = config.subset_simulation.as_ref() {
        let samples_per_level =
            usize::try_from(subset.samples_per_level).map_err(|_| CliError::MonteCarlo {
                summary: "monte_carlo.subset_simulation.samples_per_level does not fit usize"
                    .to_owned(),
            })?;
        let estimator = SubsetSimulation::new(
            seed,
            samples_per_level,
            subset.conditional_probability,
            subset.max_levels,
            subset.proposal_sigma,
            subset.dimension_id,
        )
        .map_err(monte_carlo_error)?;
        estimates.push(estimator.estimate(limit_state).map_err(monte_carlo_error)?);
    }
    if let Some(cross_entropy) = config.cross_entropy.as_ref() {
        let samples = usize::try_from(cross_entropy.samples).map_err(|_| CliError::MonteCarlo {
            summary: "monte_carlo.cross_entropy.samples does not fit usize".to_owned(),
        })?;
        let estimator = CrossEntropyIs::new(
            seed,
            samples,
            cross_entropy.elite_fraction,
            cross_entropy.iterations,
            cross_entropy.smoothing,
            cross_entropy.min_std_dev,
            cross_entropy.dimension_id,
        )
        .map_err(monte_carlo_error)?;
        estimates.push(estimator.estimate(limit_state).map_err(monte_carlo_error)?);
    }
    Ok(estimates)
}

/// Parse an MC credibility floor label.
///
/// # Errors
///
/// Returns [`CliError`] when the label is not `l0`..`l4` or `0`..`4`.
pub fn parse_credibility_floor(value: &str) -> Result<CredibilityLevel, CliError> {
    match value.trim().to_ascii_lowercase().as_str() {
        "0" | "l0" => Ok(CredibilityLevel::L0),
        "1" | "l1" => Ok(CredibilityLevel::L1),
        "2" | "l2" => Ok(CredibilityLevel::L2),
        "3" | "l3" => Ok(CredibilityLevel::L3),
        "4" | "l4" => Ok(CredibilityLevel::L4),
        _ => Err(CliError::MonteCarlo {
            summary: format!("invalid credibility floor `{value}`; expected l0..l4"),
        }),
    }
}

/// Evaluate optional MC UQ credibility evidence and write the Markdown report.
///
/// # Errors
///
/// Returns [`CliError`] when the UQ TOML is malformed, UQ validation fails,
/// the Markdown report cannot be written, or the binding credibility level is
/// below the requested floor.
fn evaluate_credibility_options(
    options: McCredibilityOptions<'_>,
) -> Result<(CampaignCredibilityReport, Option<PathBuf>), CliError> {
    let resolved = resolve_credibility_options(options)?;
    evaluate_credibility_budget(
        &resolved.budget,
        resolved.floor,
        resolved.report_md.as_deref(),
    )
}

/// Evaluate optional MC UQ sidecar evidence plus runner-gathered upstream UQ.
///
/// If both inputs are present, upstream sources are appended as independent
/// sources after the sidecar budget. If only upstream UQ is present, it is
/// still evaluated with an `L0` floor so campaign drivers can surface
/// discipline-deck credibility without bespoke merge logic.
///
/// # Errors
///
/// Returns [`CliError`] when the sidecar is malformed, source ids conflict,
/// UQ validation fails, the Markdown report cannot be written, or the binding
/// credibility level is below the requested floor.
pub fn evaluate_campaign_credibility_contract(
    inputs: CampaignCredibilityInputs<'_>,
) -> Result<Option<(CampaignCredibilityReport, Option<PathBuf>)>, CliError> {
    if inputs.upstream_uq.sources.is_empty() {
        return inputs.sidecar.map(evaluate_credibility_options).transpose();
    }

    let resolved = inputs
        .sidecar
        .map(resolve_credibility_options)
        .transpose()?;
    if let Some(mut resolved) = resolved {
        append_independent_uq_budget(&mut resolved.budget, inputs.upstream_uq)?;
        return evaluate_credibility_budget(
            &resolved.budget,
            resolved.floor,
            resolved.report_md.as_deref(),
        )
        .map(Some);
    }

    evaluate_credibility_budget(inputs.upstream_uq, CredibilityLevel::L0, None).map(Some)
}

/// Evaluate optional MC UQ sidecar evidence plus runner-gathered upstream UQ.
///
/// Prefer [`evaluate_campaign_credibility_contract`] for new campaign drivers;
/// this compatibility wrapper keeps older call sites on the same contract.
///
/// # Errors
///
/// Returns [`CliError`] when the sidecar is malformed, source ids conflict,
/// UQ validation fails, the Markdown report cannot be written, or the binding
/// credibility level is below the requested floor.
pub fn evaluate_campaign_credibility_inputs(
    credibility: Option<McCredibilityOptions<'_>>,
    gathered_upstream_uq: &CorrelatedErrorBudget,
) -> Result<Option<(CampaignCredibilityReport, Option<PathBuf>)>, CliError> {
    evaluate_campaign_credibility_contract(CampaignCredibilityInputs::new(
        credibility,
        gathered_upstream_uq,
    ))
}

fn resolve_credibility_options(
    options: McCredibilityOptions<'_>,
) -> Result<ResolvedMcCredibilityOptions, CliError> {
    Ok(ResolvedMcCredibilityOptions {
        budget: read_uq_budget(options.uq_toml)?,
        floor: options.floor,
        report_md: options.report_md.map(Path::to_path_buf),
    })
}

fn evaluate_credibility_budget(
    budget: &CorrelatedErrorBudget,
    floor: CredibilityLevel,
    report_md: Option<&Path>,
) -> Result<(CampaignCredibilityReport, Option<PathBuf>), CliError> {
    let report = evaluate_campaign_credibility(budget, floor).map_err(monte_carlo_error)?;
    let report_path = report_md.map(Path::to_path_buf);
    if let Some(path) = &report_path {
        write_markdown_report(path, &report.markdown)?;
    }
    if !report.accepted {
        return Err(CliError::MonteCarlo {
            summary: format!(
                "credibility binding level {} is below requested floor {}",
                report.binding_level.as_label(),
                report.floor.as_label()
            ),
        });
    }
    Ok((report, report_path))
}

/// Execute a sampled scheduled-propulsion-fault campaign through the normal
/// scenario parser and runner, then write one terminal-metric row per sample.
///
/// # Errors
///
/// Returns [`CliError`] for IO, malformed fault-library TOML, invalid Monte
/// Carlo inputs, scenario validation failures, runner failures, telemetry
/// channel lookup failures, or output CSV write failures.
pub fn run_propulsion_fault_campaign(
    options: McPropulsionFaultCampaignOptions<'_>,
) -> Result<McPropulsionFaultCampaignReport, CliError> {
    validate_propulsion_fault_campaign_options(&options)?;

    let base_scenario =
        fs::read_to_string(options.scenario_path).map_err(|source| CliError::Io {
            path: options.scenario_path.to_path_buf(),
            source,
        })?;
    let library_text = fs::read_to_string(options.library_toml).map_err(|source| CliError::Io {
        path: options.library_toml.to_path_buf(),
        source,
    })?;
    let document: PropulsionFaultLibraryDocument =
        toml::from_str(&library_text).map_err(|err| CliError::MonteCarlo {
            summary: format!("fault library TOML error: {err}"),
        })?;
    let manifest_credibility = if options.credibility.is_none() {
        propulsion_fault_library_credibility_options(options.library_toml, document.uq.as_ref())?
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
    let credibility = options.credibility.or(manifest_credibility);
    let library = PropulsionFaultLibrary::new(document.faults).map_err(monte_carlo_error)?;
    let source_dir = options.scenario_path.parent().map(Path::to_path_buf);

    let mut rows = Vec::new();
    let mut gathered_upstream_uq = CorrelatedErrorBudget::default();
    for sample_index in 0..options.samples {
        let overlay =
            library.materialize(options.campaign_seed, sample_index, options.dimension_id);
        let scenario_text = append_propulsion_fault_overlay(&base_scenario, &overlay);
        let scenario = Scenario::from_toml_str_with_source_dir(&scenario_text, source_dir.clone())?;
        let outcome = openbmp_runner::run(&scenario)?;
        append_independent_uq_sources(&mut gathered_upstream_uq, &outcome.upstream_uq.sources)?;
        let value = final_f64_channel(&outcome.table, options.metric_channel)?;
        let success = terminal_metric_success(value, options.success_min, options.success_max);
        let fault_ids = overlay
            .rules()
            .iter()
            .map(|rule| rule.id.as_str())
            .collect::<Vec<_>>()
            .join(";");
        rows.push(PropulsionFaultCampaignRow {
            sample_index,
            fault_ids,
            activated_fault_count: overlay.rules().len(),
            value,
            success,
        });
    }

    let value_stats =
        Welford::from_indexed_samples(rows.iter().map(|row| (row.sample_index, row.value)))
            .map_err(monte_carlo_error)?;
    let success = success_summary(rows.iter().map(|row| row.success), options.confidence)?;
    let credibility = evaluate_campaign_credibility_contract(CampaignCredibilityInputs::new(
        credibility,
        &gathered_upstream_uq,
    ))?;
    let (credibility, credibility_report_md) = match credibility {
        Some((report, report_path)) => (Some(report), report_path),
        None => (None, None),
    };
    write_propulsion_fault_campaign_csv(options.output_csv, &rows)?;

    Ok(McPropulsionFaultCampaignReport {
        samples: options.samples,
        activated_samples: rows
            .iter()
            .filter(|row| row.activated_fault_count > 0)
            .count() as u64,
        output_csv: options.output_csv.to_path_buf(),
        summary: McSummaryReport {
            samples: value_stats.count(),
            mean: value_stats.mean(),
            sample_variance: value_stats.sample_variance(),
            standard_error: value_stats.standard_error(),
            success: Some(success),
        },
        credibility,
        credibility_report_md,
    })
}

fn propulsion_fault_library_credibility_options(
    library_toml: &Path,
    uq: Option<&PropulsionFaultLibraryUqDocument>,
) -> Result<Option<(PathBuf, CredibilityLevel, Option<PathBuf>)>, CliError> {
    let Some(uq) = uq else {
        return Ok(None);
    };
    if uq.budget_toml.as_os_str().is_empty() {
        return Err(CliError::MonteCarlo {
            summary: "fault library UQ budget_toml cannot be empty".to_owned(),
        });
    }
    let floor = uq
        .credibility_floor
        .as_deref()
        .map(parse_credibility_floor)
        .transpose()?
        .unwrap_or(CredibilityLevel::L0);
    if uq
        .report_md
        .as_ref()
        .is_some_and(|path| path.as_os_str().is_empty())
    {
        return Err(CliError::MonteCarlo {
            summary: "fault library UQ report_md cannot be empty".to_owned(),
        });
    }
    let base_dir = library_toml.parent();
    Ok(Some((
        resolve_library_path(base_dir, &uq.budget_toml),
        floor,
        uq.report_md
            .as_ref()
            .map(|path| resolve_library_path(base_dir, path)),
    )))
}

fn resolve_library_path(base_dir: Option<&Path>, path: &Path) -> PathBuf {
    if path.is_absolute() {
        return path.to_path_buf();
    }
    base_dir.map_or_else(|| path.to_path_buf(), |base_dir| base_dir.join(path))
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct ScalarCsvRow {
    sample_index: u64,
    value: f64,
    success: bool,
}

#[derive(Clone, Debug, PartialEq)]
struct PropulsionFaultCampaignRow {
    sample_index: u64,
    fault_ids: String,
    activated_fault_count: usize,
    value: f64,
    success: bool,
}

fn validate_propulsion_fault_campaign_options(
    options: &McPropulsionFaultCampaignOptions<'_>,
) -> Result<(), CliError> {
    if options.samples == 0 {
        return Err(CliError::MonteCarlo {
            summary: "propulsion fault campaign requires at least one sample".to_owned(),
        });
    }
    if options.metric_channel.trim().is_empty() {
        return Err(CliError::MonteCarlo {
            summary: "metric channel cannot be empty".to_owned(),
        });
    }
    if options.success_min.is_some_and(|value| !value.is_finite()) {
        return Err(CliError::MonteCarlo {
            summary: "success_min must be finite".to_owned(),
        });
    }
    if options.success_max.is_some_and(|value| !value.is_finite()) {
        return Err(CliError::MonteCarlo {
            summary: "success_max must be finite".to_owned(),
        });
    }
    if let (Some(min), Some(max)) = (options.success_min, options.success_max)
        && min > max
    {
        return Err(CliError::MonteCarlo {
            summary: "success_min cannot exceed success_max".to_owned(),
        });
    }
    Ok(())
}

fn append_propulsion_fault_overlay(
    base_scenario: &str,
    overlay: &openbmp_mc::PropulsionFaultOverlay,
) -> String {
    if overlay.is_empty() {
        return base_scenario.to_owned();
    }
    let fragment = overlay.to_toml_fragment();
    let fragment = if declares_propulsion_faults_table(base_scenario) {
        fragment
            .strip_prefix("[propulsion.faults]\n\n")
            .unwrap_or(&fragment)
    } else {
        fragment.as_str()
    };

    let mut scenario = base_scenario.trim_end().to_owned();
    scenario.push_str("\n\n");
    scenario.push_str(fragment);
    scenario
}

fn declares_propulsion_faults_table(base_scenario: &str) -> bool {
    base_scenario.lines().any(|line| {
        let line = line.trim();
        line == "[propulsion.faults]" || line == "[[propulsion.faults.rules]]"
    })
}

fn final_f64_channel(table: &TelemetryTable, channel_name: &str) -> Result<f64, CliError> {
    let channel = table
        .schema()
        .channels()
        .iter()
        .find(|channel| channel.name == channel_name)
        .ok_or_else(|| CliError::MonteCarlo {
            summary: format!("missing telemetry channel `{channel_name}`"),
        })?;
    let row = table.rows().last().ok_or_else(|| CliError::MonteCarlo {
        summary: "scenario produced no telemetry rows".to_owned(),
    })?;
    match row.get(channel.id) {
        Some(TelemetryValue::Float64(value)) => Ok(*value),
        Some(TelemetryValue::Int64(value)) => Ok(*value as f64),
        Some(value) => Err(CliError::MonteCarlo {
            summary: format!(
                "telemetry channel `{channel_name}` is not numeric at terminal row (got {})",
                value.kind()
            ),
        }),
        None => Err(CliError::MonteCarlo {
            summary: format!("terminal row is missing telemetry channel `{channel_name}`"),
        }),
    }
}

fn terminal_metric_success(value: f64, min: Option<f64>, max: Option<f64>) -> bool {
    if min.is_some_and(|bound| value < bound) {
        return false;
    }
    if max.is_some_and(|bound| value > bound) {
        return false;
    }
    true
}

fn write_propulsion_fault_campaign_csv(
    output_csv: &Path,
    rows: &[PropulsionFaultCampaignRow],
) -> Result<(), CliError> {
    ensure_parent_dir(output_csv)?;
    let mut writer = csv::Writer::from_path(output_csv).map_err(|source| CliError::Csv {
        path: output_csv.to_path_buf(),
        source,
    })?;
    writer
        .write_record([
            "sample_index",
            "fault_ids",
            "activated_fault_count",
            "value",
            "success",
        ])
        .map_err(|source| CliError::Csv {
            path: output_csv.to_path_buf(),
            source,
        })?;
    for row in rows {
        writer
            .write_record([
                row.sample_index.to_string(),
                row.fault_ids.clone(),
                row.activated_fault_count.to_string(),
                format!("{:.17e}", row.value),
                row.success.to_string(),
            ])
            .map_err(|source| CliError::Csv {
                path: output_csv.to_path_buf(),
                source,
            })?;
    }
    writer.flush().map_err(|source| CliError::Io {
        path: output_csv.to_path_buf(),
        source,
    })
}

fn read_uq_budget(path: &Path) -> Result<CorrelatedErrorBudget, CliError> {
    let text = fs::read_to_string(path).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    let document: UqBudgetDocument = toml::from_str(&text).map_err(|err| CliError::MonteCarlo {
        summary: format!("UQ TOML error: {err}"),
    })?;
    let sources = document
        .sources
        .into_iter()
        .map(uq_source_from_document)
        .collect::<Result<Vec<_>, _>>()?;
    let correlation = document
        .correlation
        .map(|correlation| uq_correlation_from_document(correlation, sources.len()))
        .transpose()?;
    Ok(CorrelatedErrorBudget {
        sources,
        correlation,
    })
}

fn append_independent_uq_sources(
    budget: &mut CorrelatedErrorBudget,
    incoming: &[UncertaintySource],
) -> Result<(), CliError> {
    append_independent_uq_budget(
        budget,
        &CorrelatedErrorBudget {
            sources: incoming.to_vec(),
            correlation: None,
        },
    )
}

fn append_independent_uq_budget(
    budget: &mut CorrelatedErrorBudget,
    incoming: &CorrelatedErrorBudget,
) -> Result<(), CliError> {
    let mut added: Vec<UncertaintySource> = Vec::new();
    let mut added_indices = Vec::new();
    let mut skipped_duplicate = false;
    for (index, source) in incoming.sources.iter().enumerate() {
        if let Some(existing) = budget
            .sources
            .iter()
            .chain(added.iter())
            .find(|existing| existing.source_id == source.source_id)
        {
            if existing != source {
                return Err(CliError::MonteCarlo {
                    summary: format!(
                        "conflicting UQ source `{}` gathered from runner",
                        source.source_id
                    ),
                });
            }
            skipped_duplicate = true;
            continue;
        }
        added_indices.push(index);
        added.push(source.clone());
    }
    if added.is_empty() {
        return Ok(());
    }
    if skipped_duplicate && incoming.correlation.is_some() {
        return Err(CliError::MonteCarlo {
            summary: "cannot merge partially duplicate correlated upstream UQ budget".to_owned(),
        });
    }
    expand_correlation_for_independent_budget(budget, incoming, &added_indices)?;
    budget.sources.extend(added);
    Ok(())
}

fn expand_correlation_for_independent_budget(
    budget: &mut CorrelatedErrorBudget,
    incoming: &CorrelatedErrorBudget,
    added_indices: &[usize],
) -> Result<(), CliError> {
    let old_correlation = budget.correlation.take();
    if old_correlation.is_none() && incoming.correlation.is_none() {
        return Ok(());
    }
    let old_count = budget.sources.len();
    let added_count = added_indices.len();
    let new_count = old_count + added_count;
    let mut values = vec![0.0; new_count * new_count];
    for index in 0..new_count {
        values[index * new_count + index] = 1.0;
    }
    for row in 0..old_count {
        for column in 0..old_count {
            values[row * new_count + column] = old_correlation
                .as_ref()
                .map(|correlation| {
                    correlation
                        .get(row, column)
                        .ok_or_else(|| CliError::MonteCarlo {
                            summary: "UQ correlation matrix dimension mismatch".to_owned(),
                        })
                })
                .transpose()?
                .unwrap_or(if row == column { 1.0 } else { 0.0 });
        }
    }
    for (new_row, incoming_row) in added_indices.iter().copied().enumerate() {
        for (new_column, incoming_column) in added_indices.iter().copied().enumerate() {
            let row = old_count + new_row;
            let column = old_count + new_column;
            values[row * new_count + column] = incoming
                .correlation
                .as_ref()
                .map(|correlation| {
                    correlation
                        .get(incoming_row, incoming_column)
                        .ok_or_else(|| CliError::MonteCarlo {
                            summary: "UQ correlation matrix dimension mismatch".to_owned(),
                        })
                })
                .transpose()?
                .unwrap_or({
                    if incoming_row == incoming_column {
                        1.0
                    } else {
                        0.0
                    }
                });
        }
    }
    budget.correlation =
        Some(
            UqCorrelationMatrix::new(new_count, values).map_err(|err| CliError::MonteCarlo {
                summary: err.to_string(),
            })?,
        );
    Ok(())
}

fn uq_source_from_document(document: UqSourceDocument) -> Result<UncertaintySource, CliError> {
    let mut credibility = CredibilityRecord::new();
    for (key, score) in document.credibility {
        let factor = CredibilityFactor::from_key(&key).ok_or_else(|| CliError::MonteCarlo {
            summary: format!("unknown credibility factor `{key}`"),
        })?;
        let level =
            CredibilityLevel::from_value(score.level).ok_or_else(|| CliError::MonteCarlo {
                summary: format!(
                    "credibility factor `{}` has invalid level {}; expected 0..4",
                    factor.as_key(),
                    score.level
                ),
            })?;
        credibility = credibility.with_score(factor, level, score.evidence);
    }
    Ok(UncertaintySource {
        source_id: document.id,
        one_sigma: document.one_sigma,
        class: match document.class {
            UqClassDocument::Aleatory => UncertaintyClass::Aleatory,
            UqClassDocument::Epistemic => UncertaintyClass::Epistemic,
        },
        credibility,
        justification: document.justification,
    })
}

fn uq_correlation_from_document(
    document: UqCorrelationDocument,
    source_count: usize,
) -> Result<UqCorrelationMatrix, CliError> {
    if document.matrix.len() != source_count {
        return Err(CliError::MonteCarlo {
            summary: format!(
                "UQ correlation matrix has {} rows, expected {source_count}",
                document.matrix.len()
            ),
        });
    }
    let mut values = Vec::with_capacity(source_count.saturating_mul(source_count));
    for (row_index, row) in document.matrix.into_iter().enumerate() {
        if row.len() != source_count {
            return Err(CliError::MonteCarlo {
                summary: format!(
                    "UQ correlation row {row_index} has {} columns, expected {source_count}",
                    row.len()
                ),
            });
        }
        values.extend(row);
    }
    UqCorrelationMatrix::new(source_count, values).map_err(|err| CliError::MonteCarlo {
        summary: err.to_string(),
    })
}

fn write_markdown_report(path: &Path, content: &str) -> Result<(), CliError> {
    ensure_parent_dir(path)?;
    fs::write(path, content).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })
}

fn write_pbox_csv(path: &Path, report: &NestedScalarAnalysisReport) -> Result<(), CliError> {
    ensure_parent_dir(path)?;
    let mut writer = csv::Writer::from_path(path).map_err(|source| CliError::Csv {
        path: path.to_path_buf(),
        source,
    })?;
    writer
        .write_record(["support", "lower_cdf", "upper_cdf"])
        .map_err(|source| CliError::Csv {
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
        writer
            .write_record([
                format!("{support:.17e}"),
                format!("{lower:.17e}"),
                format!("{upper:.17e}"),
            ])
            .map_err(|source| CliError::Csv {
                path: path.to_path_buf(),
                source,
            })?;
    }
    writer.flush().map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })
}

fn write_rare_event_report_toml(
    path: &Path,
    report: &McRareEventScenarioReport,
) -> Result<(), CliError> {
    ensure_parent_dir(path)?;
    let mut out = String::new();
    let _ = writeln!(out, "[rare_event]");
    let _ = writeln!(
        out,
        "scenario = {}",
        toml_string(&report.scenario_path.display().to_string())
    );
    let _ = writeln!(out, "seed = {}", report.seed);
    let _ = writeln!(
        out,
        "limit_state_label = {}",
        toml_string(&report.limit_state_label)
    );
    let _ = writeln!(out, "dimension = {}", report.dimension);
    let _ = writeln!(out, "estimate_count = {}", report.estimates.len());
    for estimate in &report.estimates {
        let _ = writeln!(out);
        let _ = writeln!(out, "[[rare_event.estimates]]");
        let _ = writeln!(out, "method = {}", toml_string(estimate.method.as_str()));
        let _ = writeln!(
            out,
            "limit_state_label = {}",
            toml_string(estimate.limit_state_label)
        );
        let _ = writeln!(out, "dimension = {}", estimate.dimension);
        let _ = writeln!(out, "evaluations = {}", estimate.evaluations);
        let _ = writeln!(
            out,
            "failure_probability = {:.17e}",
            estimate.failure_probability
        );
        let _ = writeln!(
            out,
            "coefficient_of_variation = {:.17e}",
            estimate.coefficient_of_variation
        );
        if let Some(probability) = estimate.analytic_probability {
            let _ = writeln!(out, "analytic_probability = {probability:.17e}");
        }
        if let Some(error) = estimate.abs_log10_error {
            let _ = writeln!(out, "abs_log10_error = {error:.17e}");
        }
    }
    for estimate in &report.estimates {
        for level in &estimate.subset_levels {
            let _ = writeln!(out);
            let _ = writeln!(out, "[[rare_event.subset_levels]]");
            let _ = writeln!(out, "method = {}", toml_string(estimate.method.as_str()));
            let _ = writeln!(out, "level = {}", level.level);
            let _ = writeln!(out, "threshold = {:.17e}", level.threshold);
            let _ = writeln!(
                out,
                "conditional_probability = {:.17e}",
                level.conditional_probability
            );
            let _ = writeln!(out, "failures = {}", level.failures);
        }
        for iteration in &estimate.cross_entropy_iterations {
            let _ = writeln!(out);
            let _ = writeln!(out, "[[rare_event.cross_entropy_iterations]]");
            let _ = writeln!(out, "method = {}", toml_string(estimate.method.as_str()));
            let _ = writeln!(out, "iteration = {}", iteration.iteration);
            let _ = writeln!(out, "elite_threshold = {:.17e}", iteration.elite_threshold);
            let _ = writeln!(out, "mean = {}", toml_f64_array(&iteration.mean));
            let _ = writeln!(out, "std_dev = {}", toml_f64_array(&iteration.std_dev));
        }
    }
    fs::write(path, out).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })
}

fn toml_f64_array(values: &[f64]) -> String {
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
    let mut out = String::from("\"");
    for character in value.chars() {
        match character {
            '\\' => out.push_str("\\\\"),
            '"' => out.push_str("\\\""),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            other => out.push(other),
        }
    }
    out.push('"');
    out
}

fn aero_deck_draw_overlay_toml(
    options: &McAeroDeckDrawOverlaysOptions<'_>,
    sample_index: u64,
) -> String {
    let mut out = String::new();
    let _ = writeln!(out, "# Generated by openbmp mc aero-deck-draw-overlays.");
    let _ = writeln!(
        out,
        "# Merge this file into a schema-v3 scenario for sample_index {sample_index}."
    );
    let _ = writeln!(out, "[aero.deck_draw]");
    let _ = writeln!(out, "campaign_seed = {}", options.campaign_seed);
    let _ = writeln!(out, "sample_index = {sample_index}");
    let _ = writeln!(out, "dimension_id_base = {}", options.dimension_id_base);
    if let Some(mach_correlation_length) = options.mach_correlation_length {
        let _ = writeln!(
            out,
            "mach_correlation_length = {mach_correlation_length:.17e}"
        );
    }
    if let Some(alpha_correlation_length_deg) = options.alpha_correlation_length_deg {
        let _ = writeln!(
            out,
            "alpha_correlation_length_deg = {alpha_correlation_length_deg:.17e}"
        );
    }
    if let Some(beta_correlation_length_deg) = options.beta_correlation_length_deg {
        let _ = writeln!(
            out,
            "beta_correlation_length_deg = {beta_correlation_length_deg:.17e}"
        );
    }
    out
}

fn aero_deck_draw_campaign_manifest_toml(
    options: &McAeroDeckDrawCampaignOptions<'_>,
    overlays: &McAeroDeckDrawOverlaysReport,
    manifest_toml: &Path,
) -> Result<String, CliError> {
    let overlay_prefix = options.overlay_prefix.trim();
    let mut out = String::new();
    let _ = writeln!(out, "# Generated by openbmp mc aero-deck-draw-campaign.");
    let _ = writeln!(
        out,
        "campaign_id = {}",
        toml_string(options.campaign_id.trim())
    );
    let _ = writeln!(out, "seeds = [{}]", options.run_seed);
    let _ = writeln!(out, "overlays = [");
    for offset in 0..options.samples {
        let sample_index = options
            .first_sample_index
            .checked_add(offset)
            .ok_or_else(|| CliError::MonteCarlo {
                summary: "aero deck-draw sample index range overflows u64".to_owned(),
            })?;
        let overlay_id = format!("{overlay_prefix}-{sample_index:06}");
        let _ = writeln!(out, "  {},", toml_string(&overlay_id));
    }
    let _ = writeln!(out, "]");

    for (offset, overlay_path) in overlays.overlay_paths.iter().enumerate() {
        let sample_index = options
            .first_sample_index
            .checked_add(offset as u64)
            .ok_or_else(|| CliError::MonteCarlo {
                summary: "aero deck-draw sample index range overflows u64".to_owned(),
            })?;
        let overlay_id = format!("{overlay_prefix}-{sample_index:06}");
        let path = campaign_manifest_relative_path(manifest_toml, overlay_path);
        let _ = writeln!(out);
        let _ = writeln!(out, "[[overlay_files]]");
        let _ = writeln!(out, "id = {}", toml_string(&overlay_id));
        let _ = writeln!(out, "path = {}", toml_string(&path));
        let _ = writeln!(out, "mode = \"merge\"");
    }

    let scenario_path = campaign_manifest_relative_path(manifest_toml, options.scenario_path);
    let _ = writeln!(out);
    let _ = writeln!(out, "[[scenarios]]");
    let _ = writeln!(out, "id = {}", toml_string(options.scenario_id.trim()));
    let _ = writeln!(out, "path = {}", toml_string(&scenario_path));
    Ok(out)
}

fn campaign_manifest_overlay_path(
    manifest_toml: &Path,
    output_dir: &Path,
    file_name: &str,
) -> String {
    let overlay_path = output_dir.join(file_name);
    campaign_manifest_relative_path(manifest_toml, &overlay_path)
}

fn campaign_manifest_relative_path(manifest_toml: &Path, target_path: &Path) -> String {
    if let Some(manifest_dir) = manifest_toml
        .parent()
        .filter(|path| !path.as_os_str().is_empty())
        && let Ok(relative_path) = target_path.strip_prefix(manifest_dir)
    {
        return relative_path.to_string_lossy().into_owned();
    }
    target_path.to_string_lossy().into_owned()
}

fn read_nested_scalar_samples(
    samples_csv: &Path,
    epistemic_column: &str,
    value_column: &str,
) -> Result<Vec<Vec<f64>>, CliError> {
    let mut reader = csv::Reader::from_path(samples_csv).map_err(|source| CliError::Csv {
        path: samples_csv.to_path_buf(),
        source,
    })?;
    let headers = reader
        .headers()
        .map_err(|source| CliError::Csv {
            path: samples_csv.to_path_buf(),
            source,
        })?
        .clone();
    let epistemic_index = column_index(&headers, epistemic_column)?;
    let value_index = column_index(&headers, value_column)?;

    let mut grouped = BTreeMap::<u64, Vec<f64>>::new();
    for (row_number, row) in reader.records().enumerate() {
        let row = row.map_err(|source| CliError::Csv {
            path: samples_csv.to_path_buf(),
            source,
        })?;
        let epistemic = parse_u64(
            row.get(epistemic_index).unwrap_or_default(),
            epistemic_column,
            row_number,
        )?;
        let value = parse_f64(
            row.get(value_index).unwrap_or_default(),
            value_column,
            row_number,
        )?;
        grouped.entry(epistemic).or_default().push(value);
    }
    if grouped.is_empty() {
        return Err(CliError::MonteCarlo {
            summary: "nested sample CSV contains no data rows".to_owned(),
        });
    }
    Ok(grouped.into_values().collect())
}

fn read_scalar_rows(
    samples_csv: &Path,
    value_column: &str,
    success_column: Option<&str>,
) -> Result<Vec<ScalarCsvRow>, CliError> {
    let mut reader = csv::Reader::from_path(samples_csv).map_err(|source| CliError::Csv {
        path: samples_csv.to_path_buf(),
        source,
    })?;
    let headers = reader
        .headers()
        .map_err(|source| CliError::Csv {
            path: samples_csv.to_path_buf(),
            source,
        })?
        .clone();
    let value_index = column_index(&headers, value_column)?;
    let success_index = success_column
        .map(|column| column_index(&headers, column))
        .transpose()?;
    let sample_index = headers.iter().position(|header| header == "sample_index");

    let mut rows = Vec::new();
    for (row_number, row) in reader.records().enumerate() {
        let row = row.map_err(|source| CliError::Csv {
            path: samples_csv.to_path_buf(),
            source,
        })?;
        let index = match sample_index {
            Some(column) => parse_u64(
                row.get(column).unwrap_or_default(),
                "sample_index",
                row_number,
            )?,
            None => row_number as u64,
        };
        let value = parse_f64(
            row.get(value_index).unwrap_or_default(),
            value_column,
            row_number,
        )?;
        let success = match success_index {
            Some(column) => parse_success(
                row.get(column).unwrap_or_default(),
                success_column.unwrap_or("success"),
                row_number,
            )?,
            None => true,
        };
        rows.push(ScalarCsvRow {
            sample_index: index,
            value,
            success,
        });
    }
    if rows.is_empty() {
        return Err(CliError::MonteCarlo {
            summary: "sample CSV contains no data rows".to_owned(),
        });
    }
    rows.sort_by_key(|row| row.sample_index);
    for window in rows.windows(2) {
        if window[0].sample_index == window[1].sample_index {
            return Err(CliError::MonteCarlo {
                summary: format!("duplicate sample_index {}", window[0].sample_index),
            });
        }
    }
    Ok(rows)
}

fn summary_from_campaign_report(
    report: &ScalarCampaignReport,
    include_success: bool,
    confidence: f64,
) -> Result<McSummaryReport, CliError> {
    let success = if include_success {
        Some(success_summary(
            report.samples.iter().map(|sample| sample.success),
            confidence,
        )?)
    } else {
        None
    };
    Ok(McSummaryReport {
        samples: report.value_stats.count(),
        mean: report.value_stats.mean(),
        sample_variance: report.value_stats.sample_variance(),
        standard_error: report.value_stats.standard_error(),
        success,
    })
}

fn success_summary(
    successes: impl IntoIterator<Item = bool>,
    confidence: f64,
) -> Result<McSuccessSummary, CliError> {
    let mut trials = 0_u64;
    let mut success_count = 0_u64;
    for success in successes {
        trials += 1;
        if success {
            success_count += 1;
        }
    }
    let stats = BernoulliStats::new(trials, success_count).map_err(monte_carlo_error)?;
    let interval = stats
        .clopper_pearson(confidence)
        .map_err(monte_carlo_error)?;
    Ok(McSuccessSummary {
        trials: stats.trials(),
        successes: stats.successes(),
        fraction: stats.fraction().unwrap_or(0.0),
        confidence: interval.confidence,
        lower: interval.lower,
        upper: interval.upper,
    })
}

fn monte_carlo_error(err: MonteCarloError) -> CliError {
    CliError::MonteCarlo {
        summary: err.to_string(),
    }
}

fn checkpoint_store_error(err: CheckpointStoreError) -> CliError {
    CliError::MonteCarlo {
        summary: err.to_string(),
    }
}

fn column_index(headers: &csv::StringRecord, name: &str) -> Result<usize, CliError> {
    headers
        .iter()
        .position(|header| header == name)
        .ok_or_else(|| CliError::MonteCarlo {
            summary: format!("missing column `{name}`"),
        })
}

fn parse_u64(value: &str, column: &str, row_number: usize) -> Result<u64, CliError> {
    value.parse::<u64>().map_err(|_| CliError::MonteCarlo {
        summary: format!("row {row_number}: column `{column}` is not an unsigned integer"),
    })
}

fn parse_f64(value: &str, column: &str, row_number: usize) -> Result<f64, CliError> {
    let parsed = value.parse::<f64>().map_err(|_| CliError::MonteCarlo {
        summary: format!("row {row_number}: column `{column}` is not a finite float"),
    })?;
    if parsed.is_finite() {
        Ok(parsed)
    } else {
        Err(CliError::MonteCarlo {
            summary: format!("row {row_number}: column `{column}` is not a finite float"),
        })
    }
}

fn parse_success(value: &str, column: &str, row_number: usize) -> Result<bool, CliError> {
    match value.trim().to_ascii_lowercase().as_str() {
        "true" | "1" => Ok(true),
        "false" | "0" => Ok(false),
        _ => Err(CliError::MonteCarlo {
            summary: format!("row {row_number}: column `{column}` is not true/false or 1/0"),
        }),
    }
}

fn unit_cube_headers(dimensions: u32) -> Vec<String> {
    let mut header = Vec::new();
    header.push("sample_index".to_owned());
    for dimension in 0..dimensions {
        header.push(format!("u{dimension}"));
    }
    header
}

fn validate_acoustics_ascent_trace_options(
    options: &McAcousticsAscentTraceOptions<'_>,
) -> Result<(), CliError> {
    if options.samples < 2 {
        return Err(CliError::MonteCarlo {
            summary: "acoustics ascent trace generation requires at least two samples".to_owned(),
        });
    }
    validate_positive_range(
        "dynamic_pressure_pa",
        options.dynamic_pressure_min_pa,
        options.dynamic_pressure_max_pa,
    )?;
    validate_non_negative_range("mach", options.mach_min, options.mach_max)?;
    if options.sample_prefix.trim().is_empty() {
        return Err(CliError::MonteCarlo {
            summary: "sample_prefix must not be empty".to_owned(),
        });
    }
    if let Some(class_id) = options.class_id
        && class_id.trim().is_empty()
    {
        return Err(CliError::MonteCarlo {
            summary: "class_id must not be empty when provided".to_owned(),
        });
    }
    Ok(())
}

fn validate_acoustics_response_samples_options(
    options: &McAcousticsResponseSamplesOptions<'_>,
) -> Result<(), CliError> {
    if options.samples < 2 {
        return Err(CliError::MonteCarlo {
            summary: "acoustics response sample generation requires at least two samples"
                .to_owned(),
        });
    }
    validate_positive_range(
        "surface_mass_kg_m2",
        options.surface_mass_min_kg_m2,
        options.surface_mass_max_kg_m2,
    )?;
    validate_positive_range("area_m2", options.area_min_m2, options.area_max_m2)?;
    validate_finite_range(
        "mode_shape_factor",
        options.mode_shape_min,
        options.mode_shape_max,
    )?;
    if options.sample_prefix.trim().is_empty() {
        return Err(CliError::MonteCarlo {
            summary: "sample_prefix must not be empty".to_owned(),
        });
    }
    if let Some(class_id) = options.class_id
        && class_id.trim().is_empty()
    {
        return Err(CliError::MonteCarlo {
            summary: "class_id must not be empty when provided".to_owned(),
        });
    }
    Ok(())
}

fn validate_aero_deck_draw_overlays_options(
    options: &McAeroDeckDrawOverlaysOptions<'_>,
) -> Result<(), CliError> {
    if options.samples == 0 {
        return Err(CliError::MonteCarlo {
            summary: "aero deck-draw overlay generation requires at least one sample".to_owned(),
        });
    }
    if options
        .first_sample_index
        .checked_add(options.samples - 1)
        .is_none()
    {
        return Err(CliError::MonteCarlo {
            summary: "aero deck-draw sample index range overflows u64".to_owned(),
        });
    }
    validate_optional_aero_deck_draw_correlation_length(
        "mach_correlation_length",
        options.mach_correlation_length,
    )?;
    validate_optional_aero_deck_draw_correlation_length(
        "alpha_correlation_length_deg",
        options.alpha_correlation_length_deg,
    )?;
    validate_optional_aero_deck_draw_correlation_length(
        "beta_correlation_length_deg",
        options.beta_correlation_length_deg,
    )?;
    if options.dimension_id_base.checked_add(2).is_none() {
        return Err(CliError::MonteCarlo {
            summary: "dimension_id_base must leave room for CN, CD, and CM RNG dimensions"
                .to_owned(),
        });
    }
    validate_overlay_prefix(options.overlay_prefix)
}

fn validate_aero_deck_draw_campaign_options(
    options: &McAeroDeckDrawCampaignOptions<'_>,
) -> Result<(), CliError> {
    if options.campaign_id.trim().is_empty() {
        return Err(CliError::MonteCarlo {
            summary: "campaign_id must not be empty".to_owned(),
        });
    }
    if options.scenario_id.trim().is_empty() {
        return Err(CliError::MonteCarlo {
            summary: "scenario_id must not be empty".to_owned(),
        });
    }
    if options.run_code_id.trim().is_empty() {
        return Err(CliError::MonteCarlo {
            summary: "run_code_id must not be empty".to_owned(),
        });
    }
    validate_aero_deck_draw_overlays_options(&McAeroDeckDrawOverlaysOptions {
        samples: options.samples,
        campaign_seed: options.campaign_seed,
        first_sample_index: options.first_sample_index,
        dimension_id_base: options.dimension_id_base,
        mach_correlation_length: options.mach_correlation_length,
        alpha_correlation_length_deg: options.alpha_correlation_length_deg,
        beta_correlation_length_deg: options.beta_correlation_length_deg,
        overlay_prefix: options.overlay_prefix,
        output_dir: options.output_dir,
        manifest_toml: options.output_dir,
    })
}

fn validate_optional_aero_deck_draw_correlation_length(
    field: &str,
    value: Option<f64>,
) -> Result<(), CliError> {
    if let Some(value) = value
        && !(value.is_finite() && value > 0.0)
    {
        return Err(CliError::MonteCarlo {
            summary: format!("{field} must be finite and positive when supplied"),
        });
    }
    Ok(())
}

fn validate_aero_deck_draw_campaign_dimension_capacity(
    dimension_id_base: u32,
    required_dimension_count: u32,
) -> Result<(), CliError> {
    let Some(highest_offset) = required_dimension_count.checked_sub(1) else {
        return Err(CliError::MonteCarlo {
            summary: "resolved aero deck reported zero required deck-draw dimensions".to_owned(),
        });
    };
    if dimension_id_base.checked_add(highest_offset).is_none() {
        return Err(CliError::MonteCarlo {
            summary: format!(
                "dimension_id_base must leave room for {required_dimension_count} aero deck-draw RNG dimensions required by the resolved deck"
            ),
        });
    }
    Ok(())
}

fn validate_overlay_prefix(prefix: &str) -> Result<(), CliError> {
    let prefix = prefix.trim();
    if prefix.is_empty() {
        return Err(CliError::MonteCarlo {
            summary: "overlay_prefix must not be empty".to_owned(),
        });
    }
    if prefix == "." || prefix == ".." {
        return Err(CliError::MonteCarlo {
            summary: "overlay_prefix must not be a relative path sentinel".to_owned(),
        });
    }
    if !prefix
        .chars()
        .all(|ch| ch.is_ascii_alphanumeric() || ch == '-' || ch == '_' || ch == '.')
    {
        return Err(CliError::MonteCarlo {
            summary: "overlay_prefix may contain only ASCII letters, digits, '.', '-', or '_'"
                .to_owned(),
        });
    }
    Ok(())
}

fn validate_base_acoustics_ascent_campaign_scenario(input: &str) -> Result<(), CliError> {
    let parsed: toml::Value = toml::from_str(input).map_err(|err| CliError::MonteCarlo {
        summary: format!("base scenario TOML error: {err}"),
    })?;
    let ascent = parsed
        .get("acoustics")
        .and_then(|value| value.get("ascent"))
        .ok_or_else(|| CliError::MonteCarlo {
            summary: "acoustics ascent campaign requires base scenario [acoustics.ascent]"
                .to_owned(),
        })?;
    if ascent
        .get("samples")
        .and_then(toml::Value::as_array)
        .is_some_and(|samples| !samples.is_empty())
    {
        return Err(CliError::MonteCarlo {
            summary: "acoustics ascent campaign base scenario must not already declare [[acoustics.ascent.samples]]".to_owned(),
        });
    }
    Ok(())
}

fn validate_base_acoustics_response_campaign_scenario(
    input: &str,
    generating_response_samples: bool,
) -> Result<(), CliError> {
    let parsed: toml::Value = toml::from_str(input).map_err(|err| CliError::MonteCarlo {
        summary: format!("base scenario TOML error: {err}"),
    })?;
    let response = parsed
        .get("acoustics")
        .and_then(|value| value.get("response"))
        .ok_or_else(|| CliError::MonteCarlo {
            summary: "acoustics response campaign requires base scenario [acoustics.response]"
                .to_owned(),
        })?;
    if generating_response_samples
        && response
            .get("samples")
            .and_then(toml::Value::as_array)
            .is_some_and(|samples| !samples.is_empty())
    {
        return Err(CliError::MonteCarlo {
            summary: "acoustics response campaign base scenario must not already declare [[acoustics.response.samples]] when response sample generation is enabled".to_owned(),
        });
    }
    Ok(())
}

fn validate_acoustics_ascent_campaign_checkpoint_options(
    options: &McAcousticsAscentCampaignOptions<'_>,
) -> Result<(), CliError> {
    if options.checkpoint_json.is_none() && options.max_new_samples.is_some() {
        return Err(CliError::MonteCarlo {
            summary: "acoustics ascent campaign --max-new-samples requires --checkpoint-json"
                .to_owned(),
        });
    }
    if options.checkpoint_json.is_some() && options.toolchain_profile.trim().is_empty() {
        return Err(CliError::MonteCarlo {
            summary: "acoustics ascent campaign toolchain_profile must not be empty".to_owned(),
        });
    }
    Ok(())
}

fn validate_positive_range(field: &'static str, min: f64, max: f64) -> Result<(), CliError> {
    if !min.is_finite() || !max.is_finite() || min <= 0.0 || max <= min {
        return Err(CliError::MonteCarlo {
            summary: format!("{field} range must be finite with 0 < min < max"),
        });
    }
    Ok(())
}

fn validate_non_negative_range(field: &'static str, min: f64, max: f64) -> Result<(), CliError> {
    if !min.is_finite() || !max.is_finite() || min < 0.0 || max <= min {
        return Err(CliError::MonteCarlo {
            summary: format!("{field} range must be finite with 0 <= min < max"),
        });
    }
    Ok(())
}

fn validate_finite_range(field: &'static str, min: f64, max: f64) -> Result<(), CliError> {
    if !min.is_finite() || !max.is_finite() || max <= min {
        return Err(CliError::MonteCarlo {
            summary: format!("{field} range must be finite with min < max"),
        });
    }
    Ok(())
}

fn acoustics_ascent_trace_design(
    options: &McAcousticsAscentTraceOptions<'_>,
) -> Result<DesignMatrix, CliError> {
    match options.design {
        McAcousticsAscentTraceDesign::LatinHypercube => LatinHypercube
            .generate(options.campaign_seed, options.samples, 2)
            .map_err(monte_carlo_error),
        McAcousticsAscentTraceDesign::Sobol => SobolSequence
            .generate(options.samples, 2)
            .map_err(monte_carlo_error),
        McAcousticsAscentTraceDesign::OwenSobol => OwenScramble::new(options.campaign_seed)
            .generate(options.samples, 2)
            .map_err(monte_carlo_error),
    }
}

fn acoustics_response_samples_design(
    options: &McAcousticsResponseSamplesOptions<'_>,
) -> Result<DesignMatrix, CliError> {
    match options.design {
        McAcousticsAscentTraceDesign::LatinHypercube => LatinHypercube
            .generate(options.campaign_seed, options.samples, 3)
            .map_err(monte_carlo_error),
        McAcousticsAscentTraceDesign::Sobol => SobolSequence
            .generate(options.samples, 3)
            .map_err(monte_carlo_error),
        McAcousticsAscentTraceDesign::OwenSobol => OwenScramble::new(options.campaign_seed)
            .generate(options.samples, 3)
            .map_err(monte_carlo_error),
    }
}

fn acoustics_ascent_trace_toml(
    options: &McAcousticsAscentTraceOptions<'_>,
    design: &DesignMatrix,
) -> Result<String, CliError> {
    let mut out = String::new();
    let _ = writeln!(out, "# Generated by openbmp mc acoustics-ascent-trace.");
    let _ = writeln!(
        out,
        "# Paste these rows under a schema-v3 [acoustics.ascent] block."
    );
    for sample_index in 0..options.samples {
        let row = acoustics_ascent_trace_sample_toml(options, design, sample_index)?;
        let _ = writeln!(out);
        out.push_str(&row);
    }
    Ok(out)
}

fn acoustics_response_samples_toml(
    options: &McAcousticsResponseSamplesOptions<'_>,
    design: &DesignMatrix,
) -> Result<String, CliError> {
    let mut out = String::new();
    let _ = writeln!(out, "# Generated by openbmp mc acoustics-response-samples.");
    let _ = writeln!(
        out,
        "# Paste these rows under a schema-v3 [acoustics.response] block."
    );
    for sample_index in 0..options.samples {
        let row = acoustics_response_sample_toml(options, design, sample_index)?;
        let _ = writeln!(out);
        out.push_str(&row);
    }
    Ok(out)
}

fn acoustics_ascent_trace_sample_toml(
    options: &McAcousticsAscentTraceOptions<'_>,
    design: &DesignMatrix,
    sample_index: u64,
) -> Result<String, CliError> {
    let q_unit = design
        .get(sample_index, 0)
        .ok_or_else(|| CliError::MonteCarlo {
            summary: format!("internal ascent trace design is missing q sample {sample_index}"),
        })?;
    let mach_unit = design
        .get(sample_index, 1)
        .ok_or_else(|| CliError::MonteCarlo {
            summary: format!("internal ascent trace design is missing Mach sample {sample_index}"),
        })?;
    let dynamic_pressure_pa = map_unit_interval(
        q_unit,
        options.dynamic_pressure_min_pa,
        options.dynamic_pressure_max_pa,
    );
    let mach = map_unit_interval(mach_unit, options.mach_min, options.mach_max);
    let mut out = String::new();
    let _ = writeln!(out, "[[acoustics.ascent.samples]]");
    let _ = writeln!(
        out,
        "sample_id = {}",
        toml_string(&format!(
            "{}-{sample_index:06}",
            options.sample_prefix.trim()
        ))
    );
    if let Some(class_id) = options.class_id {
        let _ = writeln!(out, "class_id = {}", toml_string(class_id.trim()));
    }
    let _ = writeln!(out, "dynamic_pressure_pa = {dynamic_pressure_pa:.17e}");
    let _ = writeln!(out, "mach = {mach:.17e}");
    Ok(out)
}

fn acoustics_response_sample_toml(
    options: &McAcousticsResponseSamplesOptions<'_>,
    design: &DesignMatrix,
    sample_index: u64,
) -> Result<String, CliError> {
    let surface_mass_unit = design
        .get(sample_index, 0)
        .ok_or_else(|| CliError::MonteCarlo {
            summary: format!(
                "internal response sample design is missing surface-mass sample {sample_index}"
            ),
        })?;
    let area_unit = design
        .get(sample_index, 1)
        .ok_or_else(|| CliError::MonteCarlo {
            summary: format!(
                "internal response sample design is missing area sample {sample_index}"
            ),
        })?;
    let mode_shape_unit = design
        .get(sample_index, 2)
        .ok_or_else(|| CliError::MonteCarlo {
            summary: format!(
                "internal response sample design is missing mode-shape sample {sample_index}"
            ),
        })?;
    let surface_mass_kg_m2 = map_unit_interval(
        surface_mass_unit,
        options.surface_mass_min_kg_m2,
        options.surface_mass_max_kg_m2,
    );
    let area_m2 = map_unit_interval(area_unit, options.area_min_m2, options.area_max_m2);
    let mode_shape_factor = map_unit_interval(
        mode_shape_unit,
        options.mode_shape_min,
        options.mode_shape_max,
    );
    let mut out = String::new();
    let _ = writeln!(out, "[[acoustics.response.samples]]");
    let _ = writeln!(
        out,
        "sample_id = {}",
        toml_string(&format!(
            "{}-{sample_index:06}",
            options.sample_prefix.trim()
        ))
    );
    if let Some(class_id) = options.class_id {
        let _ = writeln!(out, "class_id = {}", toml_string(class_id.trim()));
    }
    let _ = writeln!(out, "surface_mass_kg_m2 = {surface_mass_kg_m2:.17e}");
    let _ = writeln!(out, "area_m2 = {area_m2:.17e}");
    let _ = writeln!(out, "mode_shape_factor = {mode_shape_factor:.17e}");
    Ok(out)
}

fn evaluate_acoustics_ascent_campaign_checkpoint_sample(
    base_scenario: &str,
    source_dir: Option<&Path>,
    trace_options: &McAcousticsAscentTraceOptions<'_>,
    design: &DesignMatrix,
    sample_index: u64,
) -> Result<ScalarOutcome, MonteCarloError> {
    let sample_toml = acoustics_ascent_trace_sample_toml(trace_options, design, sample_index)
        .map_err(|_| MonteCarloError::InvalidInput {
            field: "acoustics_ascent_campaign_sample",
        })?;
    let sample_scenario = append_acoustics_ascent_trace_overlay(base_scenario, &sample_toml);
    let scenario = Scenario::from_toml_str_with_source_dir(
        &sample_scenario,
        source_dir.map(Path::to_path_buf),
    )
    .map_err(|_| MonteCarloError::InvalidInput {
        field: "acoustics_ascent_campaign_scenario",
    })?;
    let outcome = openbmp_runner::run(&scenario).map_err(|_| MonteCarloError::InvalidInput {
        field: "acoustics_ascent_campaign_runner",
    })?;
    let ascent = outcome
        .acoustics
        .as_ref()
        .and_then(|acoustics| acoustics.ascent.as_ref())
        .ok_or(MonteCarloError::InvalidInput {
            field: "acoustics_ascent_campaign_ascent_report",
        })?;
    if ascent.sample_count != 1 {
        return Err(MonteCarloError::InvalidInput {
            field: "acoustics_ascent_campaign_sample_count",
        });
    }
    ScalarOutcome::new(ascent.peak_sample_oaspl_db, true)
}

fn append_acoustics_ascent_trace_overlay(base_scenario: &str, trace_toml: &str) -> String {
    let mut out = String::with_capacity(base_scenario.len() + trace_toml.len() + 2);
    out.push_str(base_scenario.trim_end());
    out.push_str("\n\n");
    out.push_str(trace_toml.trim_start());
    if !out.ends_with('\n') {
        out.push('\n');
    }
    out
}

fn map_unit_interval(unit: f64, min: f64, max: f64) -> f64 {
    min + unit * (max - min)
}

fn sha256_bytes(bytes: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    let digest = hasher.finalize();
    let mut out = String::with_capacity(digest.len() * 2);
    for byte in digest {
        let _ = write!(out, "{byte:02x}");
    }
    out
}

fn read_design_matrix(path: &Path) -> Result<(Vec<String>, DesignMatrix), CliError> {
    let mut reader = csv::Reader::from_path(path).map_err(|source| CliError::Csv {
        path: path.to_path_buf(),
        source,
    })?;
    let headers = reader
        .headers()
        .map_err(|source| CliError::Csv {
            path: path.to_path_buf(),
            source,
        })?
        .iter()
        .map(str::to_owned)
        .collect::<Vec<_>>();
    if headers.len() < 2
        || headers
            .first()
            .is_none_or(|header| header != "sample_index")
    {
        return Err(CliError::MonteCarlo {
            summary: "design CSV must start with `sample_index` and at least one design column"
                .to_owned(),
        });
    }
    let dimensions = u32::try_from(headers.len() - 1).map_err(|_| CliError::MonteCarlo {
        summary: "design CSV has too many columns".to_owned(),
    })?;

    let mut values = Vec::new();
    let mut rows = 0_u64;
    for (row_number, row) in reader.records().enumerate() {
        let row = row.map_err(|source| CliError::Csv {
            path: path.to_path_buf(),
            source,
        })?;
        parse_u64(row.get(0).unwrap_or_default(), "sample_index", row_number)?;
        for dimension in 0..usize::try_from(dimensions).map_err(|_| CliError::MonteCarlo {
            summary: "design dimension count is too large".to_owned(),
        })? {
            values.push(parse_f64(
                row.get(dimension + 1).unwrap_or_default(),
                headers[dimension + 1].as_str(),
                row_number,
            )?);
        }
        rows = rows.checked_add(1).ok_or_else(|| CliError::MonteCarlo {
            summary: "design CSV has too many rows".to_owned(),
        })?;
    }

    let design =
        DesignMatrix::new(rows, dimensions, values).map_err(|err| CliError::MonteCarlo {
            summary: err.to_string(),
        })?;
    Ok((headers, design))
}

fn read_correlation_matrix(path: &Path, dimensions: u32) -> Result<CorrelationMatrix, CliError> {
    let mut reader = csv::ReaderBuilder::new()
        .has_headers(false)
        .from_path(path)
        .map_err(|source| CliError::Csv {
            path: path.to_path_buf(),
            source,
        })?;
    let dimension_count = usize::try_from(dimensions).map_err(|_| CliError::MonteCarlo {
        summary: "correlation matrix dimension is too large".to_owned(),
    })?;
    let mut rows = 0_usize;
    let mut values = Vec::new();
    for row in reader.records() {
        let row = row.map_err(|source| CliError::Csv {
            path: path.to_path_buf(),
            source,
        })?;
        if row.len() != dimension_count {
            return Err(CliError::MonteCarlo {
                summary: format!(
                    "correlation row {rows} has {} columns, expected {dimension_count}",
                    row.len()
                ),
            });
        }
        for column in 0..dimension_count {
            values.push(parse_f64(
                row.get(column).unwrap_or_default(),
                "correlation",
                rows,
            )?);
        }
        rows += 1;
    }
    if rows != dimension_count {
        return Err(CliError::MonteCarlo {
            summary: format!("correlation matrix has {rows} rows, expected {dimension_count}"),
        });
    }
    CorrelationMatrix::new(dimensions, values).map_err(|err| CliError::MonteCarlo {
        summary: err.to_string(),
    })
}

fn write_design_matrix(
    output_csv: &Path,
    headers: &[String],
    design: &DesignMatrix,
) -> Result<(), CliError> {
    if headers.len()
        != usize::try_from(design.columns()).map_err(|_| CliError::MonteCarlo {
            summary: "design dimension count is too large".to_owned(),
        })? + 1
    {
        return Err(CliError::MonteCarlo {
            summary: "design header does not match dimension count".to_owned(),
        });
    }
    ensure_parent_dir(output_csv)?;
    let mut writer = csv::Writer::from_path(output_csv).map_err(|source| CliError::Csv {
        path: output_csv.to_path_buf(),
        source,
    })?;
    writer
        .write_record(headers)
        .map_err(|source| CliError::Csv {
            path: output_csv.to_path_buf(),
            source,
        })?;
    for row in 0..design.rows() {
        let mut record = Vec::new();
        record.push(row.to_string());
        for dimension in 0..design.columns() {
            let value = design
                .get(row, dimension)
                .ok_or_else(|| CliError::MonteCarlo {
                    summary: format!("missing design value row={row} dimension={dimension}"),
                })?;
            record.push(format!("{value:.17e}"));
        }
        writer
            .write_record(&record)
            .map_err(|source| CliError::Csv {
                path: output_csv.to_path_buf(),
                source,
            })?;
    }
    writer.flush().map_err(|source| CliError::Io {
        path: output_csv.to_path_buf(),
        source,
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

#[cfg(test)]
mod tests {
    use std::error::Error;
    use std::fs;

    use super::*;

    const MINIMAL: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/analytic-toy/constant-acceleration-drop.toml"
    ));

    const RARE_EVENT_BLOCK: &str = r#"
[monte_carlo]
seed = 1234

[monte_carlo.limit_state]
kind = "synthetic_linear"
label = "synthetic-limit-state"
beta = 2.0
dimension = 3

[monte_carlo.subset_simulation]
samples_per_level = 512
conditional_probability = 0.1
max_levels = 4
proposal_sigma = 0.8
dimension_id = 19

[monte_carlo.cross_entropy]
samples = 512
elite_fraction = 0.1
iterations = 4
smoothing = 0.8
min_std_dev = 0.2
dimension_id = 23
"#;

    #[test]
    fn rare_event_scenario_runs_estimators_and_writes_toml() -> Result<(), Box<dyn Error>> {
        let temp = tempfile::tempdir()?;
        let scenario_path = temp.path().join("rare-event.toml");
        let output_toml = temp.path().join("out").join("rare-event-report.toml");
        let scenario = format!(
            "{}\n{RARE_EVENT_BLOCK}",
            MINIMAL.replace("openbmp.scenario = 2", "openbmp.scenario = 3")
        );
        fs::write(&scenario_path, scenario)?;

        let report = run_rare_event_scenario(McRareEventScenarioOptions {
            scenario_path: &scenario_path,
            output_toml: Some(&output_toml),
        })?;

        assert_eq!(report.seed, 1234);
        assert_eq!(report.limit_state_label, "synthetic-limit-state");
        assert_eq!(report.dimension, 3);
        assert_eq!(report.estimates.len(), 2);
        assert_eq!(report.estimates[0].method.as_str(), "subset_simulation");
        assert_eq!(
            report.estimates[1].method.as_str(),
            "cross_entropy_importance_sampling"
        );
        assert!(report.estimates[0].analytic_probability.is_some());
        assert_eq!(report.output_toml.as_deref(), Some(output_toml.as_path()));

        let text = fs::read_to_string(output_toml)?;
        let _: toml::Value = toml::from_str(&text)?;
        assert!(text.contains("[[rare_event.estimates]]"));
        assert!(text.contains("subset_simulation"));
        assert!(text.contains("cross_entropy_importance_sampling"));
        assert!(text.contains("synthetic-limit-state"));
        Ok(())
    }

    #[test]
    fn rare_event_scenario_requires_monte_carlo_block() -> Result<(), Box<dyn Error>> {
        let temp = tempfile::tempdir()?;
        let scenario_path = temp.path().join("minimal.toml");
        fs::write(
            &scenario_path,
            MINIMAL.replace("openbmp.scenario = 2", "openbmp.scenario = 3"),
        )?;

        let result = run_rare_event_scenario(McRareEventScenarioOptions {
            scenario_path: &scenario_path,
            output_toml: None,
        });

        assert!(
            matches!(result, Err(CliError::MonteCarlo { ref summary })
                if summary == "rare-event run requires [monte_carlo]"),
            "got {result:?}",
        );
        Ok(())
    }
}
