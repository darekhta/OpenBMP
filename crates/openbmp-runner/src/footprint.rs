//! Offline range-safety landing-footprint helpers.
//!
//! This module is post-processing only: it turns a parsed
//! `[landing_footprint]` scenario block plus a caller-supplied
//! ballistic state into a forward landing prediction. It is not
//! registered as a flight-controller job and it never produces an
//! actuator command.

use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};

use nalgebra::Vector3;
use openbmp_core::{ChannelId, DeterministicRng, SimTime, StepIndex};
use openbmp_mc::{
    NestedLowerTailRequirement, NestedScalarAnalysisReport, analyze_nested_scalar_samples,
};
use openbmp_physics::profile::{
    BallisticState, ConstantGravityRangeSafetyFootprint, FootprintDispersionInput,
    FootprintEnvironment, FootprintGeodeticOrigin, FootprintMonteCarloInput,
    FootprintMonteCarloResult, FootprintSample, FootprintSampleFailure, FootprintSampleInput,
    ForwardSimulationProvenance, ForwardSimulationSource, LandingFootprint,
    NumericalGravityRangeSafetyFootprint, RangeSafetyFootprint,
    constant_gravity_footprint_monte_carlo, numerical_gravity_footprint_monte_carlo,
};
use openbmp_physics::{Egm2008ZonalGravity, J2Gravity, STANDARD_GRAVITY_M_S2, WGS84_J2};
use openbmp_scenario::{
    LandingFootprintConfig, LandingFootprintMethod, LandingFootprintMonteCarloConfig,
    LandingFootprintMonteCarloDistribution, LandingFootprintMonteCarloNestedMetric,
    LandingFootprintMonteCarloUncertaintyClass, LandingFootprintMonteCarloWindConfig, ModelRole,
    Scenario, ScenarioDocument, ScenarioError,
};
use openbmp_telemetry::{TelemetryChannel, TelemetryRow, TelemetrySchema, TelemetryTable};
use openbmp_uq::CorrelatedErrorBudget;
use serde::{Deserialize, Serialize};

use crate::RunnerError;

/// Compute the configured offline landing footprint for `state`.
///
/// Returns `Ok(None)` when the scenario does not declare a
/// `[landing_footprint]` block.
///
/// # Errors
///
/// Returns [`RunnerError`] when the scenario block is internally
/// inconsistent or the physics model rejects the supplied state /
/// environment.
pub fn landing_footprint_for_state(
    scenario: &Scenario,
    state: &BallisticState,
) -> Result<Option<LandingFootprint>, RunnerError> {
    let Some(config) = scenario.document.landing_footprint.as_ref() else {
        return Ok(None);
    };
    let env = footprint_environment(&scenario.document, config)?;
    let footprint = match config.method {
        LandingFootprintMethod::ConstantGravity => {
            ConstantGravityRangeSafetyFootprint.landing_footprint(state, &env)?
        }
        LandingFootprintMethod::J2 => {
            let gravity = build_j2_gravity(&scenario.document)?;
            NumericalGravityRangeSafetyFootprint::new(gravity).landing_footprint(state, &env)?
        }
        LandingFootprintMethod::Egm2008 => {
            let gravity = Egm2008ZonalGravity::wgs84_egm2008_zonal();
            NumericalGravityRangeSafetyFootprint::new(gravity).landing_footprint(state, &env)?
        }
    };
    Ok(Some(footprint))
}

/// Runner-side Monte-Carlo footprint report ready for CLI export.
#[derive(Debug)]
pub struct FootprintMonteCarloReport {
    /// Physics result with sample cloud and summary statistics.
    pub result: FootprintMonteCarloResult,
    /// Runner-gathered upstream UQ sources from discipline models that
    /// feed this campaign.
    pub upstream_uq: CorrelatedErrorBudget,
    /// Deterministic tabular sample cloud for CSV/Parquet export.
    pub samples: TelemetryTable,
    /// Deterministic TOML summary text.
    pub summary_toml: String,
    /// Optional nested aleatory/epistemic analysis evidence.
    pub nested: Option<FootprintNestedMonteCarloReport>,
}

/// Runner-side nested footprint Monte-Carlo analysis evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct FootprintNestedMonteCarloReport {
    /// Scalar metric reduced into the p-box.
    pub metric: LandingFootprintMonteCarloNestedMetric,
    /// P-box and aleatory/epistemic variance report.
    pub analysis: NestedScalarAnalysisReport,
}

/// Options for opt-in footprint Monte-Carlo checkpoint/resume.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FootprintMonteCarloCheckpointOptions<'a> {
    /// JSON checkpoint file to load and update.
    pub checkpoint_json: &'a Path,
    /// Scenario digest bound into the checkpoint metadata.
    pub scenario_sha256: &'a str,
    /// Toolchain / determinism profile bound into the checkpoint metadata.
    pub toolchain_profile: &'a str,
    /// Maximum not-yet-completed samples to propagate during this invocation.
    pub max_new_samples: u32,
}

/// Result of a checkpointed footprint Monte-Carlo invocation.
#[derive(Debug)]
pub struct FootprintMonteCarloCheckpointReport {
    /// Total configured sample count.
    pub sample_count: u32,
    /// Completed samples, including both successes and failures.
    pub completed: u32,
    /// Complete report, present only when all samples have completed and at
    /// least one sample propagated successfully.
    pub report: Option<FootprintMonteCarloReport>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
struct FootprintCheckpointFile {
    schema_version: u32,
    method: String,
    campaign_seed: u64,
    sample_count: u32,
    scenario_sha256: String,
    toolchain_profile: String,
    nominal: Option<StoredLandingFootprint>,
    samples: Vec<StoredFootprintSample>,
    failures: Vec<StoredFootprintFailure>,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
struct StoredFootprintSample {
    sample_index: u32,
    landing: StoredLandingFootprint,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Eq, Serialize)]
struct StoredFootprintFailure {
    sample_index: u32,
    reason: String,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
struct StoredLandingFootprint {
    downrange_m_bits: u64,
    crossrange_m_bits: u64,
    bearing_rad_bits: u64,
    time_to_cull_s_bits: u64,
    latitude_deg_bits: Option<u64>,
    longitude_deg_bits: Option<u64>,
}

impl From<LandingFootprint> for StoredLandingFootprint {
    fn from(landing: LandingFootprint) -> Self {
        Self {
            downrange_m_bits: landing.downrange_m.to_bits(),
            crossrange_m_bits: landing.crossrange_m.to_bits(),
            bearing_rad_bits: landing.bearing_rad.to_bits(),
            time_to_cull_s_bits: landing.time_to_cull_s.to_bits(),
            latitude_deg_bits: landing.latitude_deg.map(f64::to_bits),
            longitude_deg_bits: landing.longitude_deg.map(f64::to_bits),
        }
    }
}

impl From<StoredLandingFootprint> for LandingFootprint {
    fn from(landing: StoredLandingFootprint) -> Self {
        Self {
            downrange_m: f64::from_bits(landing.downrange_m_bits),
            crossrange_m: f64::from_bits(landing.crossrange_m_bits),
            bearing_rad: f64::from_bits(landing.bearing_rad_bits),
            time_to_cull_s: f64::from_bits(landing.time_to_cull_s_bits),
            latitude_deg: landing.latitude_deg_bits.map(f64::from_bits),
            longitude_deg: landing.longitude_deg_bits.map(f64::from_bits),
            dispersion_ellipse: None,
        }
    }
}

impl FootprintCheckpointFile {
    const SCHEMA_VERSION: u32 = 1;

    fn new(
        method: &str,
        campaign_seed: u64,
        sample_count: u32,
        scenario_sha256: &str,
        toolchain_profile: &str,
    ) -> Self {
        Self {
            schema_version: Self::SCHEMA_VERSION,
            method: method.to_owned(),
            campaign_seed,
            sample_count,
            scenario_sha256: scenario_sha256.to_owned(),
            toolchain_profile: toolchain_profile.to_owned(),
            nominal: None,
            samples: Vec::new(),
            failures: Vec::new(),
        }
    }

    fn validate(
        &self,
        method: &str,
        campaign_seed: u64,
        sample_count: u32,
        scenario_sha256: &str,
        toolchain_profile: &str,
    ) -> Result<(), RunnerError> {
        self.validate_content()?;
        if self.method != method {
            return Err(footprint_checkpoint_error("metadata mismatch in method"));
        }
        if self.campaign_seed != campaign_seed {
            return Err(footprint_checkpoint_error(
                "metadata mismatch in campaign_seed",
            ));
        }
        if self.sample_count != sample_count {
            return Err(footprint_checkpoint_error(
                "metadata mismatch in sample_count",
            ));
        }
        if self.scenario_sha256 != scenario_sha256 {
            return Err(footprint_checkpoint_error(
                "metadata mismatch in scenario_sha256",
            ));
        }
        if self.toolchain_profile != toolchain_profile {
            return Err(footprint_checkpoint_error(
                "metadata mismatch in toolchain_profile",
            ));
        }
        Ok(())
    }

    fn validate_content(&self) -> Result<(), RunnerError> {
        if self.schema_version != Self::SCHEMA_VERSION {
            return Err(footprint_checkpoint_error(
                "metadata mismatch in schema_version",
            ));
        }
        if self.sample_count == 0 {
            return Err(footprint_checkpoint_error("sample_count must be positive"));
        }
        if self.scenario_sha256.is_empty() {
            return Err(footprint_checkpoint_error(
                "scenario_sha256 must not be empty",
            ));
        }
        if self.toolchain_profile.is_empty() {
            return Err(footprint_checkpoint_error(
                "toolchain_profile must not be empty",
            ));
        }
        let mut completed = BTreeSet::new();
        for sample in &self.samples {
            if sample.sample_index >= self.sample_count {
                return Err(footprint_checkpoint_error(
                    "sample index exceeds sample_count",
                ));
            }
            if !completed.insert(sample.sample_index) {
                return Err(footprint_checkpoint_error("duplicate sample index"));
            }
        }
        for failure in &self.failures {
            if failure.sample_index >= self.sample_count {
                return Err(footprint_checkpoint_error(
                    "failure index exceeds sample_count",
                ));
            }
            if !completed.insert(failure.sample_index) {
                return Err(footprint_checkpoint_error("duplicate sample index"));
            }
        }
        Ok(())
    }

    fn completed_count(&self) -> Result<u32, RunnerError> {
        self.validate_content()?;
        let count = self
            .samples
            .len()
            .checked_add(self.failures.len())
            .ok_or_else(|| footprint_checkpoint_error("completed sample count overflow"))?;
        u32::try_from(count)
            .map_err(|_| footprint_checkpoint_error("completed sample count overflow"))
    }

    fn missing_indices(&self, limit: u32) -> Vec<u32> {
        let completed = self
            .samples
            .iter()
            .map(|sample| sample.sample_index)
            .chain(self.failures.iter().map(|failure| failure.sample_index))
            .collect::<BTreeSet<_>>();
        (0..self.sample_count)
            .filter(|sample_index| !completed.contains(sample_index))
            .take(limit as usize)
            .collect()
    }

    fn record_batch(&mut self, result: &FootprintMonteCarloResult) -> Result<(), RunnerError> {
        self.validate_content()?;
        let nominal = StoredLandingFootprint::from(result.nominal);
        if let Some(existing) = self.nominal {
            if existing != nominal {
                return Err(footprint_checkpoint_error("nominal footprint changed"));
            }
        } else {
            self.nominal = Some(nominal);
        }

        let mut completed = self
            .samples
            .iter()
            .map(|sample| sample.sample_index)
            .chain(self.failures.iter().map(|failure| failure.sample_index))
            .collect::<BTreeSet<_>>();
        for sample in &result.samples {
            if !completed.insert(sample.sample_index) {
                return Err(footprint_checkpoint_error("duplicate sample index"));
            }
            self.samples.push(StoredFootprintSample {
                sample_index: sample.sample_index,
                landing: StoredLandingFootprint::from(sample.landing),
            });
        }
        for failure in &result.failures {
            if !completed.insert(failure.sample_index) {
                return Err(footprint_checkpoint_error("duplicate sample index"));
            }
            self.failures.push(StoredFootprintFailure {
                sample_index: failure.sample_index,
                reason: failure.reason.clone(),
            });
        }
        self.samples.sort_by_key(|sample| sample.sample_index);
        self.failures.sort_by_key(|failure| failure.sample_index);
        Ok(())
    }

    fn to_report(
        &self,
        confidence_levels: &[f64],
    ) -> Result<FootprintMonteCarloResult, RunnerError> {
        self.validate_content()?;
        if self.completed_count()? != self.sample_count {
            return Err(footprint_checkpoint_error("checkpoint is incomplete"));
        }
        let nominal = self
            .nominal
            .ok_or_else(|| footprint_checkpoint_error("checkpoint has no nominal footprint"))?;
        let samples = self
            .samples
            .iter()
            .map(|sample| FootprintSample {
                sample_index: sample.sample_index,
                landing: LandingFootprint::from(sample.landing),
            })
            .collect::<Vec<_>>();
        let failures = self
            .failures
            .iter()
            .map(|failure| FootprintSampleFailure {
                sample_index: failure.sample_index,
                reason: failure.reason.clone(),
            })
            .collect::<Vec<_>>();
        Ok(FootprintMonteCarloResult::from_samples(
            LandingFootprint::from(nominal),
            samples,
            failures,
            confidence_levels,
        )?)
    }
}

fn load_or_create_footprint_checkpoint(
    path: &Path,
    method: &str,
    campaign_seed: u64,
    sample_count: u32,
    scenario_sha256: &str,
    toolchain_profile: &str,
) -> Result<FootprintCheckpointFile, RunnerError> {
    let text = match fs::read_to_string(path) {
        Ok(text) => text,
        Err(source) if source.kind() == std::io::ErrorKind::NotFound => {
            return Ok(FootprintCheckpointFile::new(
                method,
                campaign_seed,
                sample_count,
                scenario_sha256,
                toolchain_profile,
            ));
        }
        Err(source) => {
            return Err(footprint_checkpoint_error(format!(
                "failed to read {}: {source}",
                path.display()
            )));
        }
    };
    let checkpoint = serde_json::from_str::<FootprintCheckpointFile>(&text).map_err(|source| {
        footprint_checkpoint_error(format!("failed to parse {}: {source}", path.display()))
    })?;
    checkpoint.validate(
        method,
        campaign_seed,
        sample_count,
        scenario_sha256,
        toolchain_profile,
    )?;
    Ok(checkpoint)
}

fn save_footprint_checkpoint(
    path: &Path,
    checkpoint: &FootprintCheckpointFile,
) -> Result<(), RunnerError> {
    checkpoint.validate_content()?;
    let text = serde_json::to_string_pretty(checkpoint).map_err(|source| {
        footprint_checkpoint_error(format!("failed to serialize {}: {source}", path.display()))
    })?;
    if let Some(parent) = path.parent()
        && !parent.as_os_str().is_empty()
    {
        fs::create_dir_all(parent).map_err(|source| {
            footprint_checkpoint_error(format!("failed to create {}: {source}", parent.display()))
        })?;
    }
    let tmp_path = footprint_checkpoint_tmp_path(path);
    fs::write(&tmp_path, text).map_err(|source| {
        footprint_checkpoint_error(format!("failed to write {}: {source}", tmp_path.display()))
    })?;
    fs::rename(&tmp_path, path).map_err(|source| {
        footprint_checkpoint_error(format!("failed to replace {}: {source}", path.display()))
    })?;
    Ok(())
}

fn footprint_checkpoint_tmp_path(path: &Path) -> PathBuf {
    match path.extension().and_then(|extension| extension.to_str()) {
        Some(extension) => path.with_extension(format!("{extension}.tmp")),
        None => path.with_extension("tmp"),
    }
}

fn footprint_checkpoint_error(what: impl Into<String>) -> RunnerError {
    RunnerError::UnsupportedScenario {
        what: format!("footprint checkpoint: {}", what.into()),
    }
}

/// Build the nominal burnout state from the scenario's initial state.
///
/// This is the CLI default for `openbmp footprint-mc`: scenarios whose
/// initial state is the coast/burnout handoff can run without an
/// external telemetry extractor. Library callers that already have a
/// burnout state should call [`landing_footprint_monte_carlo_for_state`].
pub fn nominal_ballistic_state_from_scenario(
    scenario: &Scenario,
) -> Result<BallisticState, RunnerError> {
    let ballistic_coefficient_m2_kg = scenario
        .document
        .landing_footprint
        .as_ref()
        .and_then(|footprint| footprint.monte_carlo.as_ref())
        .and_then(|monte_carlo| monte_carlo.ballistic_coefficient.as_ref())
        .map_or(0.0, |bc| bc.nominal_m2_kg);
    Ok(ballistic_state_from_forward_simulation(
        scenario.document.vehicle.initial_position_eci_m,
        scenario.document.vehicle.initial_velocity_eci_m_s,
        ballistic_coefficient_m2_kg,
        SimTime::from_seconds(scenario.document.time.start_s),
        ForwardSimulationSource::ScenarioInitialState,
    )?)
}

/// Run configured Monte-Carlo footprint analysis from the scenario
/// initial state.
///
/// Returns `Ok(None)` when `[landing_footprint.monte_carlo]` is not
/// declared.
///
/// # Errors
///
/// Returns [`RunnerError`] when the scenario, sampler, or physics
/// propagator rejects the analysis.
pub fn landing_footprint_monte_carlo_for_initial_state(
    scenario: &Scenario,
) -> Result<Option<FootprintMonteCarloReport>, RunnerError> {
    let state = nominal_ballistic_state_from_scenario(scenario)?;
    landing_footprint_monte_carlo_for_state(scenario, &state)
}

/// Run configured Monte-Carlo footprint analysis with opt-in checkpoint/resume
/// from the scenario initial state.
///
/// Returns `Ok(None)` when `[landing_footprint.monte_carlo]` is not declared.
///
/// # Errors
///
/// Returns [`RunnerError`] when scenario loading, checkpoint validation, file
/// IO, JSON parsing, sampling, or propagation fails.
pub fn landing_footprint_monte_carlo_for_initial_state_checkpointed(
    scenario: &Scenario,
    options: FootprintMonteCarloCheckpointOptions<'_>,
) -> Result<Option<FootprintMonteCarloCheckpointReport>, RunnerError> {
    let state = nominal_ballistic_state_from_scenario(scenario)?;
    landing_footprint_monte_carlo_for_state_checkpointed(scenario, &state, options)
}

/// Run configured Monte-Carlo footprint analysis from a caller-supplied
/// nominal burnout state.
///
/// Returns `Ok(None)` when `[landing_footprint.monte_carlo]` is not
/// declared.
///
/// # Errors
///
/// Returns [`RunnerError`] when the scenario, sampler, or physics
/// propagator rejects the analysis.
pub fn landing_footprint_monte_carlo_for_state(
    scenario: &Scenario,
    state: &BallisticState,
) -> Result<Option<FootprintMonteCarloReport>, RunnerError> {
    let Some(config) = scenario.document.landing_footprint.as_ref() else {
        return Ok(None);
    };
    let Some(monte_carlo) = config.monte_carlo.as_ref() else {
        return Ok(None);
    };
    let env = footprint_environment(&scenario.document, config)?;
    let input = monte_carlo_input(&scenario.document, &env, monte_carlo, state)?;
    let result = match config.method {
        LandingFootprintMethod::ConstantGravity => {
            constant_gravity_footprint_monte_carlo(&env, &input)?
        }
        LandingFootprintMethod::J2 => {
            let gravity = build_j2_gravity(&scenario.document)?;
            numerical_gravity_footprint_monte_carlo(&gravity, &env, &input)?
        }
        LandingFootprintMethod::Egm2008 => {
            let gravity = Egm2008ZonalGravity::wgs84_egm2008_zonal();
            numerical_gravity_footprint_monte_carlo(&gravity, &env, &input)?
        }
    };
    Ok(Some(monte_carlo_report(
        monte_carlo,
        result,
        footprint_upstream_uq_budget(scenario)?,
    )?))
}

/// Run configured Monte-Carlo footprint analysis from a caller-supplied nominal
/// burnout state with opt-in checkpoint/resume.
///
/// Returns `Ok(None)` when `[landing_footprint.monte_carlo]` is not declared.
///
/// # Errors
///
/// Returns [`RunnerError`] when scenario loading, checkpoint validation, file
/// IO, JSON parsing, sampling, or propagation fails.
pub fn landing_footprint_monte_carlo_for_state_checkpointed(
    scenario: &Scenario,
    state: &BallisticState,
    options: FootprintMonteCarloCheckpointOptions<'_>,
) -> Result<Option<FootprintMonteCarloCheckpointReport>, RunnerError> {
    let Some(config) = scenario.document.landing_footprint.as_ref() else {
        return Ok(None);
    };
    let Some(monte_carlo) = config.monte_carlo.as_ref() else {
        return Ok(None);
    };
    let env = footprint_environment(&scenario.document, config)?;
    let input = monte_carlo_input(&scenario.document, &env, monte_carlo, state)?;
    let seed = monte_carlo.seed.unwrap_or(scenario.document.time.seed);
    let report = checkpointed_footprint_monte_carlo(
        scenario,
        config.method,
        &env,
        monte_carlo,
        &input,
        seed,
        options,
    )?;
    Ok(Some(report))
}

fn checkpointed_footprint_monte_carlo(
    scenario: &Scenario,
    method: LandingFootprintMethod,
    env: &FootprintEnvironment,
    config: &LandingFootprintMonteCarloConfig,
    input: &FootprintMonteCarloInput,
    seed: u64,
    options: FootprintMonteCarloCheckpointOptions<'_>,
) -> Result<FootprintMonteCarloCheckpointReport, RunnerError> {
    if options.max_new_samples == 0 {
        return Err(footprint_checkpoint_error(
            "max_new_samples must be greater than zero",
        ));
    }
    if options.scenario_sha256.is_empty() {
        return Err(footprint_checkpoint_error(
            "scenario_sha256 must not be empty",
        ));
    }
    if options.toolchain_profile.is_empty() {
        return Err(footprint_checkpoint_error(
            "toolchain_profile must not be empty",
        ));
    }

    let method = footprint_method_label(method);
    let mut checkpoint = load_or_create_footprint_checkpoint(
        options.checkpoint_json,
        method,
        seed,
        config.samples,
        options.scenario_sha256,
        options.toolchain_profile,
    )?;
    checkpoint.validate(
        method,
        seed,
        config.samples,
        options.scenario_sha256,
        options.toolchain_profile,
    )?;

    let missing = checkpoint.missing_indices(options.max_new_samples);
    if !missing.is_empty() {
        let missing = missing.into_iter().collect::<BTreeSet<_>>();
        let batch_samples = input
            .samples
            .iter()
            .copied()
            .filter(|sample| missing.contains(&sample.sample_index))
            .collect::<Vec<_>>();
        let mut batch_input = input.clone();
        batch_input.samples = batch_samples;
        let batch =
            run_footprint_monte_carlo_for_method(&scenario.document, method, env, &batch_input)?;
        checkpoint.record_batch(&batch)?;
        save_footprint_checkpoint(options.checkpoint_json, &checkpoint)?;
    } else {
        save_footprint_checkpoint(options.checkpoint_json, &checkpoint)?;
    }

    let completed = checkpoint.completed_count()?;
    let report = if completed == config.samples {
        let result = checkpoint.to_report(&input.confidence_levels)?;
        let upstream_uq = footprint_upstream_uq_budget(scenario)?;
        Some(monte_carlo_report(config, result, upstream_uq)?)
    } else {
        None
    };
    Ok(FootprintMonteCarloCheckpointReport {
        sample_count: config.samples,
        completed,
        report,
    })
}

fn monte_carlo_report(
    config: &LandingFootprintMonteCarloConfig,
    result: FootprintMonteCarloResult,
    upstream_uq: CorrelatedErrorBudget,
) -> Result<FootprintMonteCarloReport, RunnerError> {
    let nested = nested_monte_carlo_report(config, &result)?;
    let samples = monte_carlo_samples_table(&result)?;
    let summary_toml = monte_carlo_summary_toml(config, &result, nested.as_ref());
    Ok(FootprintMonteCarloReport {
        result,
        upstream_uq,
        samples,
        summary_toml,
        nested,
    })
}

fn footprint_upstream_uq_budget(scenario: &Scenario) -> Result<CorrelatedErrorBudget, RunnerError> {
    let document = &scenario.document;
    let resolved_files = scenario.resolved_files()?;
    let mut upstream_uq = crate::structural::StructuralRack::build(document)?.upstream_uq_budget();
    upstream_uq
        .sources
        .extend(crate::aero::upstream_uq_budget(document)?.sources);
    upstream_uq
        .sources
        .extend(crate::aerothermal::upstream_uq_budget(document)?.sources);
    upstream_uq.sources.extend(
        crate::engines::EngineRack::build(document, &resolved_files)?
            .upstream_uq_budget()
            .sources,
    );
    Ok(upstream_uq)
}

fn nested_monte_carlo_report(
    config: &LandingFootprintMonteCarloConfig,
    result: &FootprintMonteCarloResult,
) -> Result<Option<FootprintNestedMonteCarloReport>, RunnerError> {
    let Some(nested) = &config.nested else {
        return Ok(None);
    };
    let mut conditional_samples = vec![Vec::new(); nested.epistemic_samples as usize];
    for sample in &result.samples {
        let epistemic_index = sample.sample_index / nested.aleatory_samples;
        let group =
            conditional_samples
                .get_mut(epistemic_index as usize)
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: format!(
                        "landing_footprint.monte_carlo.nested sample index {} is outside configured epistemic grid",
                        sample.sample_index,
                    ),
                })?;
        group.push(nested_metric_value(nested.metric, result, sample));
    }
    if let Some((index, _)) = conditional_samples
        .iter()
        .enumerate()
        .find(|(_, group)| group.is_empty())
    {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "landing_footprint.monte_carlo.nested epistemic condition {index} has no successful aleatory samples",
            ),
        });
    }
    let analysis = analyze_nested_scalar_samples(
        &conditional_samples,
        Some(NestedLowerTailRequirement {
            threshold: nested.threshold,
            minimum_probability: nested.minimum_probability,
        }),
    )
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("landing_footprint.monte_carlo.nested analysis failed: {err}"),
    })?;
    Ok(Some(FootprintNestedMonteCarloReport {
        metric: nested.metric,
        analysis,
    }))
}

fn nested_metric_value(
    metric: LandingFootprintMonteCarloNestedMetric,
    result: &FootprintMonteCarloResult,
    sample: &FootprintSample,
) -> f64 {
    match metric {
        LandingFootprintMonteCarloNestedMetric::DownrangeM => sample.landing.downrange_m,
        LandingFootprintMonteCarloNestedMetric::CrossrangeM => sample.landing.crossrange_m,
        LandingFootprintMonteCarloNestedMetric::RadialOffsetFromNominalM => radial_distance_m(
            sample.landing.downrange_m - result.nominal.downrange_m,
            sample.landing.crossrange_m - result.nominal.crossrange_m,
        ),
        LandingFootprintMonteCarloNestedMetric::RadialDistanceFromMeanM => radial_distance_m(
            sample.landing.downrange_m - result.mean_downrange_m,
            sample.landing.crossrange_m - result.mean_crossrange_m,
        ),
    }
}

fn run_footprint_monte_carlo_for_method(
    document: &ScenarioDocument,
    method: &str,
    env: &FootprintEnvironment,
    input: &FootprintMonteCarloInput,
) -> Result<FootprintMonteCarloResult, RunnerError> {
    match method {
        "constant_gravity" => Ok(constant_gravity_footprint_monte_carlo(env, input)?),
        "j2" => {
            let gravity = build_j2_gravity(document)?;
            Ok(numerical_gravity_footprint_monte_carlo(
                &gravity, env, input,
            )?)
        }
        "egm2008" => {
            let gravity = Egm2008ZonalGravity::wgs84_egm2008_zonal();
            Ok(numerical_gravity_footprint_monte_carlo(
                &gravity, env, input,
            )?)
        }
        other => Err(footprint_checkpoint_error(format!(
            "unsupported footprint method in checkpoint path: {other}"
        ))),
    }
}

fn footprint_method_label(method: LandingFootprintMethod) -> &'static str {
    match method {
        LandingFootprintMethod::ConstantGravity => "constant_gravity",
        LandingFootprintMethod::J2 => "j2",
        LandingFootprintMethod::Egm2008 => "egm2008",
    }
}

fn monte_carlo_input(
    document: &ScenarioDocument,
    env: &FootprintEnvironment,
    config: &LandingFootprintMonteCarloConfig,
    nominal_state: &BallisticState,
) -> Result<FootprintMonteCarloInput, RunnerError> {
    let seed = config.seed.unwrap_or(document.time.seed);
    let mut nominal = *nominal_state;
    if let Some(ballistic_coefficient) = &config.ballistic_coefficient {
        nominal = ballistic_state_from_forward_simulation(
            nominal.position_eci_m(),
            nominal.velocity_eci_m_s(),
            ballistic_coefficient.nominal_m2_kg,
            nominal.time(),
            ForwardSimulationSource::MonteCarloSample,
        )?;
    }
    let mut samples = Vec::with_capacity(config.samples as usize);
    let base_wind_ned_m_s = base_wind_ned_m_s(document);
    for sample_index in 0..config.samples {
        let nested_index = NestedFootprintSampleIndex::from_flat_index(sample_index, config);
        let state = sampled_state(seed, nested_index, config, nominal)?;
        let wind_eci_m_s = sampled_wind(
            seed,
            nested_index,
            config.wind.as_ref(),
            env,
            base_wind_ned_m_s,
        );
        samples.push(FootprintSampleInput {
            sample_index,
            state,
            wind_eci_m_s,
        });
    }
    Ok(FootprintMonteCarloInput::new(
        nominal,
        samples,
        config.confidence_levels.clone(),
    ))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct NestedFootprintSampleIndex {
    epistemic: u32,
    aleatory: u32,
}

impl NestedFootprintSampleIndex {
    fn from_flat_index(flat: u32, config: &LandingFootprintMonteCarloConfig) -> Self {
        if let Some(nested) = &config.nested {
            Self {
                epistemic: flat / nested.aleatory_samples,
                aleatory: flat % nested.aleatory_samples,
            }
        } else {
            Self {
                epistemic: flat,
                aleatory: flat,
            }
        }
    }

    const fn for_class(self, uncertainty_class: LandingFootprintMonteCarloUncertaintyClass) -> u32 {
        match uncertainty_class {
            LandingFootprintMonteCarloUncertaintyClass::Aleatory => self.aleatory,
            LandingFootprintMonteCarloUncertaintyClass::Epistemic => self.epistemic,
        }
    }
}

fn sampled_state(
    seed: u64,
    sample_index: NestedFootprintSampleIndex,
    config: &LandingFootprintMonteCarloConfig,
    mut nominal: BallisticState,
) -> Result<BallisticState, RunnerError> {
    if let Some(burnout) = &config.burnout_state {
        let source_sample_index = sample_index.for_class(burnout.uncertainty_class);
        let mut position_eci_m = nominal.position_eci_m();
        let mut velocity_eci_m_s = nominal.velocity_eci_m_s();
        let mut time = nominal.time();
        if let Some(position_sigma_eci_m) = burnout.position_sigma_eci_m {
            for (axis, sigma) in position_sigma_eci_m.iter().copied().enumerate() {
                position_eci_m[axis] +=
                    sample_normal(seed, source_sample_index, axis as u32, sigma);
            }
        }
        if let Some(speed_sigma_m_s) = burnout.speed_sigma_m_s {
            velocity_eci_m_s = sampled_speed_magnitude(
                seed,
                source_sample_index,
                3,
                velocity_eci_m_s,
                speed_sigma_m_s,
            );
        }
        if let Some(time_sigma_s) = burnout.time_sigma_s {
            let t = time.as_seconds() + sample_normal(seed, source_sample_index, 6, time_sigma_s);
            time = SimTime::from_seconds(t);
        }
        nominal = ballistic_state_from_forward_simulation(
            position_eci_m,
            velocity_eci_m_s,
            nominal.ballistic_coefficient_m2_kg(),
            time,
            ForwardSimulationSource::MonteCarloSample,
        )?;
    }
    if let Some(ballistic_coefficient) = &config.ballistic_coefficient {
        let source_sample_index = sample_index.for_class(ballistic_coefficient.uncertainty_class);
        let mut sampled_ballistic_coefficient_m2_kg = sample_distribution(
            seed,
            source_sample_index,
            7,
            ballistic_coefficient.nominal_m2_kg,
            ballistic_coefficient.sigma_m2_kg,
            ballistic_coefficient.distribution,
        );
        if let Some(min) = ballistic_coefficient.min_m2_kg
            && sampled_ballistic_coefficient_m2_kg < min
        {
            sampled_ballistic_coefficient_m2_kg = min;
        }
        if let Some(max) = ballistic_coefficient.max_m2_kg
            && sampled_ballistic_coefficient_m2_kg > max
        {
            sampled_ballistic_coefficient_m2_kg = max;
        }
        nominal = ballistic_state_from_forward_simulation(
            nominal.position_eci_m(),
            nominal.velocity_eci_m_s(),
            sampled_ballistic_coefficient_m2_kg,
            nominal.time(),
            ForwardSimulationSource::MonteCarloSample,
        )?;
    }
    Ok(nominal)
}

fn ballistic_state_from_forward_simulation(
    position_eci_m: [f64; 3],
    velocity_eci_m_s: [f64; 3],
    ballistic_coefficient_m2_kg: f64,
    time: SimTime,
    source: ForwardSimulationSource,
) -> Result<BallisticState, openbmp_physics::PhysicsError> {
    let provenance = ForwardSimulationProvenance::for_state(
        position_eci_m,
        velocity_eci_m_s,
        ballistic_coefficient_m2_kg,
        time,
        source,
    );
    BallisticState::from_forward_simulation(
        position_eci_m,
        velocity_eci_m_s,
        ballistic_coefficient_m2_kg,
        time,
        provenance,
    )
}

fn sampled_speed_magnitude(
    seed: u64,
    sample_index: u32,
    component: u32,
    velocity_eci_m_s: [f64; 3],
    sigma_m_s: f64,
) -> [f64; 3] {
    let speed_m_s = norm3(velocity_eci_m_s);
    if speed_m_s <= f64::EPSILON {
        return velocity_eci_m_s;
    }
    let delta_speed_m_s = sample_normal(seed, sample_index, component, sigma_m_s);
    let sampled_speed_m_s = (speed_m_s + delta_speed_m_s).max(0.0);
    let scale = sampled_speed_m_s / speed_m_s;
    [
        velocity_eci_m_s[0] * scale,
        velocity_eci_m_s[1] * scale,
        velocity_eci_m_s[2] * scale,
    ]
}

fn norm3(values: [f64; 3]) -> f64 {
    (values[0] * values[0] + values[1] * values[1] + values[2] * values[2]).sqrt()
}

fn sampled_wind(
    seed: u64,
    sample_index: NestedFootprintSampleIndex,
    wind: Option<&LandingFootprintMonteCarloWindConfig>,
    env: &FootprintEnvironment,
    base_wind_ned_m_s: [f64; 3],
) -> [f64; 3] {
    let base_wind_eci_m_s = local_ned_to_propagation_frame(env, base_wind_ned_m_s);
    let Some(wind) = wind else {
        return base_wind_eci_m_s;
    };
    let source_sample_index = sample_index.for_class(wind.uncertainty_class);
    let mut wind_ned_m_s = base_wind_ned_m_s;
    if let Some(scale_sigma) = wind.speed_scale_sigma {
        let scale = 1.0 + sample_normal(seed, source_sample_index, 11, scale_sigma);
        for component in &mut wind_ned_m_s {
            *component *= scale;
        }
    }
    if let Some(members) = &wind.ensemble_members_ned_m_s {
        let mut rng = footprint_sample_rng(seed, source_sample_index, 12);
        let index = (rng.next_u64() as usize) % members.len();
        wind_ned_m_s = members[index];
    }
    if let Some(sigma_ned_m_s) = wind.sigma_ned_m_s {
        for (axis, sigma) in sigma_ned_m_s.iter().copied().enumerate() {
            wind_ned_m_s[axis] += sample_normal(seed, source_sample_index, 8 + axis as u32, sigma);
        }
    }
    local_ned_to_propagation_frame(env, wind_ned_m_s)
}

fn sample_distribution(
    seed: u64,
    sample_index: u32,
    component: u32,
    mean: f64,
    sigma: f64,
    distribution: LandingFootprintMonteCarloDistribution,
) -> f64 {
    match distribution {
        LandingFootprintMonteCarloDistribution::Normal => {
            mean + sample_normal(seed, sample_index, component, sigma)
        }
        LandingFootprintMonteCarloDistribution::Uniform => {
            let mut rng = footprint_sample_rng(seed, sample_index, component);
            let unit = sample_unit_open(&mut rng);
            mean + (2.0 * unit - 1.0) * 3.0_f64.sqrt() * sigma
        }
    }
}

fn sample_normal(seed: u64, sample_index: u32, component: u32, sigma: f64) -> f64 {
    if sigma <= f64::EPSILON {
        return 0.0;
    }
    let mut rng = footprint_sample_rng(seed, sample_index, component);
    sigma * sample_standard_normal(&mut rng)
}

fn footprint_sample_rng(seed: u64, sample_index: u32, component: u32) -> DeterministicRng {
    let mut raw = [0u8; 32];
    raw[0..8].copy_from_slice(&seed.to_le_bytes());
    raw[8..16].copy_from_slice(&u64::from(sample_index).to_le_bytes());
    raw[24..28].copy_from_slice(&component.to_le_bytes());
    raw[28..32].copy_from_slice(b"FPMC");
    DeterministicRng::from_raw_seed(raw)
}

fn sample_standard_normal(rng: &mut DeterministicRng) -> f64 {
    let u1 = sample_unit_open(rng);
    let u2 = sample_unit_open(rng);
    let radius = (-2.0 * u1.ln()).sqrt();
    let theta = 2.0 * core::f64::consts::PI * u2;
    radius * theta.cos()
}

fn sample_unit_open(rng: &mut DeterministicRng) -> f64 {
    const TWO_NEG_53: f64 = 1.0 / ((1_u64 << 53) as f64);
    let bits = rng.next_u64() >> 11;
    ((bits as f64) + 0.5) * TWO_NEG_53
}

fn base_wind_ned_m_s(document: &ScenarioDocument) -> [f64; 3] {
    document
        .wind
        .as_ref()
        .and_then(|wind| wind.wind_ned_m_s)
        .unwrap_or([0.0, 0.0, 0.0])
}

fn local_ned_to_propagation_frame(env: &FootprintEnvironment, wind_ned_m_s: [f64; 3]) -> [f64; 3] {
    let origin = Vector3::new(
        env.launch_origin_eci_m[0],
        env.launch_origin_eci_m[1],
        env.launch_origin_eci_m[2],
    );
    let north_m_s = wind_ned_m_s[0];
    let east_m_s = wind_ned_m_s[1];
    let down_m_s = wind_ned_m_s[2];
    let origin_norm = origin.norm();
    if origin_norm <= f64::EPSILON {
        return [north_m_s, east_m_s, -down_m_s];
    }
    let up = origin / origin_norm;
    let spin_axis = Vector3::new(0.0, 0.0, 1.0);
    let mut east = spin_axis.cross(&up);
    if east.norm() <= f64::EPSILON {
        east = Vector3::new(1.0, 0.0, 0.0).cross(&up);
    }
    let east_norm = east.norm();
    if east_norm <= f64::EPSILON {
        return [north_m_s, east_m_s, -down_m_s];
    }
    let east = east / east_norm;
    let north = up.cross(&east);
    let down = -up;
    let wind = north * north_m_s + east * east_m_s + down * down_m_s;
    [wind.x, wind.y, wind.z]
}

#[allow(clippy::too_many_lines)]
fn monte_carlo_samples_table(
    result: &FootprintMonteCarloResult,
) -> Result<TelemetryTable, RunnerError> {
    let downrange =
        TelemetryChannel::<f64>::new(ChannelId::new(1), "downrange_m", "m", None::<String>)?;
    let crossrange =
        TelemetryChannel::<f64>::new(ChannelId::new(2), "crossrange_m", "m", None::<String>)?;
    let bearing =
        TelemetryChannel::<f64>::new(ChannelId::new(3), "bearing_rad", "rad", None::<String>)?;
    let time_to_cull =
        TelemetryChannel::<f64>::new(ChannelId::new(4), "time_to_cull_s", "s", None::<String>)?;
    let latitude =
        TelemetryChannel::<f64>::new(ChannelId::new(16), "latitude_deg", "deg", None::<String>)?;
    let longitude =
        TelemetryChannel::<f64>::new(ChannelId::new(17), "longitude_deg", "deg", None::<String>)?;
    let offset_downrange_from_nominal = TelemetryChannel::<f64>::new(
        ChannelId::new(18),
        "offset_downrange_from_nominal_m",
        "m",
        None::<String>,
    )?;
    let offset_crossrange_from_nominal = TelemetryChannel::<f64>::new(
        ChannelId::new(19),
        "offset_crossrange_from_nominal_m",
        "m",
        None::<String>,
    )?;
    let radial_offset_from_nominal = TelemetryChannel::<f64>::new(
        ChannelId::new(20),
        "radial_offset_from_nominal_m",
        "m",
        None::<String>,
    )?;
    let offset_downrange_from_mean = TelemetryChannel::<f64>::new(
        ChannelId::new(21),
        "offset_downrange_from_mean_m",
        "m",
        None::<String>,
    )?;
    let offset_crossrange_from_mean = TelemetryChannel::<f64>::new(
        ChannelId::new(22),
        "offset_crossrange_from_mean_m",
        "m",
        None::<String>,
    )?;
    let radial_distance_from_mean = TelemetryChannel::<f64>::new(
        ChannelId::new(23),
        "radial_distance_from_mean_m",
        "m",
        None::<String>,
    )?;
    let channels = vec![
        downrange.metadata().clone(),
        crossrange.metadata().clone(),
        bearing.metadata().clone(),
        time_to_cull.metadata().clone(),
        latitude.metadata().clone(),
        longitude.metadata().clone(),
        offset_downrange_from_nominal.metadata().clone(),
        offset_crossrange_from_nominal.metadata().clone(),
        radial_offset_from_nominal.metadata().clone(),
        offset_downrange_from_mean.metadata().clone(),
        offset_crossrange_from_mean.metadata().clone(),
        radial_distance_from_mean.metadata().clone(),
    ];
    let schema = TelemetrySchema::new(channels)?;
    let mut table = TelemetryTable::new(schema);
    for sample in &result.samples {
        let mut row = TelemetryRow::new(
            SimTime::ZERO,
            StepIndex::new(u64::from(sample.sample_index)),
        )?;
        row.insert(&downrange, sample.landing.downrange_m)?;
        row.insert(&crossrange, sample.landing.crossrange_m)?;
        row.insert(&bearing, sample.landing.bearing_rad)?;
        row.insert(&time_to_cull, sample.landing.time_to_cull_s)?;
        if let Some(value) = sample.landing.latitude_deg {
            row.insert(&latitude, value)?;
        }
        if let Some(value) = sample.landing.longitude_deg {
            row.insert(&longitude, value)?;
        }
        let nominal_downrange_offset_m = sample.landing.downrange_m - result.nominal.downrange_m;
        let nominal_crossrange_offset_m = sample.landing.crossrange_m - result.nominal.crossrange_m;
        row.insert(&offset_downrange_from_nominal, nominal_downrange_offset_m)?;
        row.insert(&offset_crossrange_from_nominal, nominal_crossrange_offset_m)?;
        row.insert(
            &radial_offset_from_nominal,
            radial_distance_m(nominal_downrange_offset_m, nominal_crossrange_offset_m),
        )?;
        let mean_downrange_offset_m = sample.landing.downrange_m - result.mean_downrange_m;
        let mean_crossrange_offset_m = sample.landing.crossrange_m - result.mean_crossrange_m;
        row.insert(&offset_downrange_from_mean, mean_downrange_offset_m)?;
        row.insert(&offset_crossrange_from_mean, mean_crossrange_offset_m)?;
        row.insert(
            &radial_distance_from_mean,
            radial_distance_m(mean_downrange_offset_m, mean_crossrange_offset_m),
        )?;
        table.push_row(row)?;
    }
    Ok(table)
}

fn monte_carlo_summary_toml(
    config: &LandingFootprintMonteCarloConfig,
    result: &FootprintMonteCarloResult,
    nested: Option<&FootprintNestedMonteCarloReport>,
) -> String {
    let mut out = String::new();
    push_summary_integer(&mut out, "samples_requested", u64::from(config.samples));
    push_summary_integer(&mut out, "samples_succeeded", result.samples.len() as u64);
    push_summary_integer(&mut out, "samples_failed", result.failures.len() as u64);
    push_summary_line(&mut out, "mean_downrange_m", result.mean_downrange_m);
    push_summary_line(&mut out, "mean_crossrange_m", result.mean_crossrange_m);
    push_summary_line(
        &mut out,
        "covariance_downrange_downrange_m2",
        result.covariance_downrange_downrange_m2,
    );
    push_summary_line(
        &mut out,
        "covariance_downrange_crossrange_m2",
        result.covariance_downrange_crossrange_m2,
    );
    push_summary_line(
        &mut out,
        "covariance_crossrange_crossrange_m2",
        result.covariance_crossrange_crossrange_m2,
    );
    out.push_str("\n[nominal_footprint]\n");
    push_summary_line(&mut out, "downrange_m", result.nominal.downrange_m);
    push_summary_line(&mut out, "crossrange_m", result.nominal.crossrange_m);
    push_summary_line(&mut out, "bearing_rad", result.nominal.bearing_rad);
    push_summary_line(&mut out, "time_to_cull_s", result.nominal.time_to_cull_s);
    out.push_str("\n[dispersion_statistics]\n");
    push_summary_line(
        &mut out,
        "radial_dispersion_p50_m",
        result.radial_dispersion_p50_m,
    );
    push_summary_line(
        &mut out,
        "mean_offset_downrange_from_nominal_m",
        result.mean_offset_downrange_from_nominal_m,
    );
    push_summary_line(
        &mut out,
        "mean_offset_crossrange_from_nominal_m",
        result.mean_offset_crossrange_from_nominal_m,
    );
    push_summary_line(
        &mut out,
        "mean_radial_offset_from_nominal_m",
        result.mean_radial_offset_from_nominal_m,
    );
    out.push_str("\n[dispersion_ellipse]\n");
    push_summary_line(
        &mut out,
        "one_sigma_semi_major_m",
        result.dispersion_ellipse.one_sigma_semi_major_m,
    );
    push_summary_line(
        &mut out,
        "one_sigma_semi_minor_m",
        result.dispersion_ellipse.one_sigma_semi_minor_m,
    );
    push_summary_line(
        &mut out,
        "three_sigma_semi_major_m",
        result.dispersion_ellipse.three_sigma_semi_major_m,
    );
    push_summary_line(
        &mut out,
        "three_sigma_semi_minor_m",
        result.dispersion_ellipse.three_sigma_semi_minor_m,
    );
    push_summary_line(
        &mut out,
        "orientation_rad",
        result.dispersion_ellipse.orientation_rad,
    );
    for quantile in &result.quantiles {
        out.push_str("\n[[quantiles]]\n");
        push_summary_line(&mut out, "confidence_level", quantile.confidence_level);
        push_summary_line(&mut out, "radial_distance_m", quantile.radial_distance_m);
    }
    for quantile in &result.nominal_radial_error_quantiles {
        out.push_str("\n[[nominal_radial_offset_quantiles]]\n");
        push_summary_line(&mut out, "confidence_level", quantile.confidence_level);
        push_summary_line(
            &mut out,
            "radial_offset_from_nominal_m",
            quantile.radial_distance_m,
        );
    }
    if let Some(nested) = nested {
        out.push_str("\n[nested_uq]\n");
        out.push_str("metric = ");
        push_toml_string(&mut out, nested.metric.as_str());
        out.push('\n');
        push_summary_integer(
            &mut out,
            "epistemic_samples",
            nested.analysis.epistemic_samples as u64,
        );
        push_summary_integer(
            &mut out,
            "min_aleatory_samples",
            nested.analysis.min_aleatory_samples as u64,
        );
        push_summary_integer(
            &mut out,
            "max_aleatory_samples",
            nested.analysis.max_aleatory_samples as u64,
        );
        push_summary_line(
            &mut out,
            "variance_aleatory",
            nested.analysis.variance_split.aleatory,
        );
        push_summary_line(
            &mut out,
            "variance_epistemic",
            nested.analysis.variance_split.epistemic,
        );
        push_summary_line(
            &mut out,
            "variance_total",
            nested.analysis.variance_split.total,
        );
        if let Some(probability) = nested.analysis.lower_bound_probability {
            push_summary_line(&mut out, "lower_bound_probability", probability);
        }
        if let Some(passed) = nested.analysis.requirement_passed {
            out.push_str("requirement_passed = ");
            out.push_str(if passed { "true\n" } else { "false\n" });
        }
    }
    for failure in &result.failures {
        out.push_str("\n[[failures]]\n");
        out.push_str(&format!("sample_index = {}\n", failure.sample_index));
        out.push_str("reason = ");
        push_toml_string(&mut out, &failure.reason);
        out.push('\n');
    }
    out
}

fn radial_distance_m(downrange_m: f64, crossrange_m: f64) -> f64 {
    (downrange_m * downrange_m + crossrange_m * crossrange_m).sqrt()
}

fn push_summary_line(out: &mut String, key: &str, value: f64) {
    out.push_str(key);
    out.push_str(" = ");
    out.push_str(&format!("{value:.17e}\n"));
}

fn push_summary_integer(out: &mut String, key: &str, value: u64) {
    out.push_str(key);
    out.push_str(" = ");
    out.push_str(&value.to_string());
    out.push('\n');
}

fn push_toml_string(out: &mut String, value: &str) {
    out.push('"');
    for c in value.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '"' => out.push_str("\\\""),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            _ => out.push(c),
        }
    }
    out.push('"');
}

fn footprint_environment(
    document: &ScenarioDocument,
    config: &LandingFootprintConfig,
) -> Result<FootprintEnvironment, RunnerError> {
    let gravity_m_s2 = match config.method {
        LandingFootprintMethod::ConstantGravity => constant_gravity_m_s2(document)?,
        LandingFootprintMethod::J2 | LandingFootprintMethod::Egm2008 => STANDARD_GRAVITY_M_S2,
    };
    let launch_origin_eci_m = launch_origin_eci_m(document, config);
    let geodetic_origin = if config.geodetic_output.includes_geodetic() {
        let origin = document
            .frames
            .as_ref()
            .and_then(|frames| frames.local_origin.as_ref())
            .ok_or_else(|| {
                RunnerError::Scenario(ScenarioError::InconsistentSection {
                    field_a: "landing_footprint.geodetic_output".to_owned(),
                    value_a: "reviewed_recovery_coordinates".to_owned(),
                    field_b: "frames.local_origin".to_owned(),
                    value_b: "missing".to_owned(),
                })
            })?;
        Some(FootprintGeodeticOrigin {
            latitude_deg: origin.latitude_deg,
            longitude_deg: origin.longitude_deg,
            height_m: origin.height_m,
        })
    } else {
        None
    };
    Ok(FootprintEnvironment {
        cull_altitude_m: config.cull_altitude_m,
        gravity_m_s2,
        launch_origin_eci_m,
        geodetic_origin,
        dispersion: config
            .dispersion
            .as_ref()
            .map(|dispersion| FootprintDispersionInput {
                one_sigma_semi_major_m: dispersion.one_sigma_semi_major_m,
                one_sigma_semi_minor_m: dispersion.one_sigma_semi_minor_m,
                orientation_rad: dispersion.orientation_rad,
            }),
    })
}

fn launch_origin_eci_m(document: &ScenarioDocument, config: &LandingFootprintConfig) -> [f64; 3] {
    match config.method {
        LandingFootprintMethod::ConstantGravity => [
            document.vehicle.initial_position_eci_m[0],
            document.vehicle.initial_position_eci_m[1],
            config.cull_altitude_m,
        ],
        LandingFootprintMethod::J2 | LandingFootprintMethod::Egm2008 => {
            if config.geodetic_output.includes_geodetic()
                && let Some(origin) = document
                    .frames
                    .as_ref()
                    .and_then(|frames| frames.local_origin.as_ref())
                && let Ok(origin) = openbmp_physics::LocalGeodeticOrigin::new_degrees(
                    origin.latitude_deg,
                    origin.longitude_deg,
                    origin.height_m,
                )
            {
                let ecef = origin.to_ecef_position();
                [ecef.vector.x, ecef.vector.y, ecef.vector.z]
            } else {
                document.vehicle.initial_position_eci_m
            }
        }
    }
}

fn constant_gravity_m_s2(document: &ScenarioDocument) -> Result<f64, RunnerError> {
    if document.environment.gravity != "constant" {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "landing_footprint.method = \"constant_gravity\" requires \
                 environment.gravity = \"constant\"; got `{}`",
                document.environment.gravity
            ),
        });
    }
    document.environment.gravity_m_s2.ok_or_else(|| {
        RunnerError::Scenario(ScenarioError::MissingRequiredField {
            field: "environment.gravity_m_s2".to_owned(),
            role: ModelRole::Gravity,
            name: "constant".to_owned(),
        })
    })
}

fn build_j2_gravity(document: &ScenarioDocument) -> Result<J2Gravity, RunnerError> {
    let mu = document
        .environment
        .mu_m3_s2
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "environment.mu_m3_s2 missing for j2 footprint gravity".to_owned(),
        })?;
    let r_e = document
        .environment
        .r_e_m
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "environment.r_e_m missing for j2 footprint gravity".to_owned(),
        })?;
    let j2 = document.environment.j2.unwrap_or(WGS84_J2);
    Ok(J2Gravity::new(mu, r_e, j2)?)
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use openbmp_core::SimTime;
    use openbmp_physics::WGS84_A_M;

    const COAST_FOOTPRINT_SCENARIO: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../openbmp-scenario/tests/fixtures/coast-footprint-valid.toml"
    ));
    const COAST_FOOTPRINT_MC_SCENARIO: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../openbmp-scenario/tests/fixtures/coast-footprint-mc-valid.toml"
    ));
    const MINIMAL_SCENARIO: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/analytic-toy/constant-acceleration-drop.toml"
    ));

    #[test]
    fn runner_computes_configured_landing_footprint() {
        let scenario = Scenario::from_toml_str(COAST_FOOTPRINT_SCENARIO).unwrap();
        let state = ballistic_state_from_forward_simulation(
            [0.0, 0.0, 100.0],
            [5.0, 2.0, 0.0],
            0.0,
            SimTime::from_seconds(0.0),
            ForwardSimulationSource::RunnerTelemetryState,
        )
        .unwrap();
        let footprint = landing_footprint_for_state(&scenario, &state)
            .unwrap()
            .unwrap();
        assert!(footprint.downrange_m > 0.0);
        assert!(footprint.crossrange_m > 0.0);
        assert!(footprint.dispersion_ellipse.is_some());
    }

    #[test]
    fn runner_computes_j2_landing_footprint() {
        let surface_position = format!("initial_position_eci_m = [{WGS84_A_M:.1}, 0.0, 0.0]");
        let gravity_config = format!("mu_m3_s2 = 398600441800000.0\nr_e_m = {WGS84_A_M:.1}");
        let toml = COAST_FOOTPRINT_SCENARIO
            .replace(
                "initial_position_eci_m = [0.0, 0.0, 0.0]",
                &surface_position,
            )
            .replace(r#"gravity = "constant""#, r#"gravity = "j2""#)
            .replace("gravity_m_s2 = 9.80665", &gravity_config)
            .replace(r#"method = "constant_gravity""#, r#"method = "j2""#);
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let state = ballistic_state_from_forward_simulation(
            [WGS84_A_M + 1_000.0, 0.0, 0.0],
            [0.0, 100.0, 0.0],
            0.0,
            SimTime::from_seconds(0.0),
            ForwardSimulationSource::RunnerTelemetryState,
        )
        .unwrap();
        let footprint = landing_footprint_for_state(&scenario, &state)
            .unwrap()
            .unwrap();
        assert!(footprint.time_to_cull_s > 0.0);
        assert!(footprint.downrange_m > 0.0);
        assert!(footprint.dispersion_ellipse.is_some());
    }

    #[test]
    fn runner_computes_egm2008_landing_footprint() {
        let surface_position = format!("initial_position_eci_m = [{WGS84_A_M:.1}, 0.0, 0.0]");
        let toml = COAST_FOOTPRINT_SCENARIO
            .replace(
                "initial_position_eci_m = [0.0, 0.0, 0.0]",
                &surface_position,
            )
            .replace(r#"gravity = "constant""#, r#"gravity = "egm2008""#)
            .replace("gravity_m_s2 = 9.80665\n", "")
            .replace(r#"method = "constant_gravity""#, r#"method = "egm2008""#);
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let state = ballistic_state_from_forward_simulation(
            [WGS84_A_M + 1_000.0, 0.0, 0.0],
            [0.0, 100.0, 0.0],
            0.0,
            SimTime::from_seconds(0.0),
            ForwardSimulationSource::RunnerTelemetryState,
        )
        .unwrap();
        let footprint = landing_footprint_for_state(&scenario, &state)
            .unwrap()
            .unwrap();
        assert!(footprint.time_to_cull_s > 0.0);
        assert!(footprint.downrange_m > 0.0);
        assert!(footprint.dispersion_ellipse.is_some());
    }

    #[test]
    fn runner_returns_none_without_footprint_config() {
        let scenario = Scenario::from_toml_str(MINIMAL_SCENARIO).unwrap();
        let state = ballistic_state_from_forward_simulation(
            [0.0, 0.0, 100.0],
            [0.0, 0.0, 0.0],
            0.0,
            SimTime::from_seconds(0.0),
            ForwardSimulationSource::RunnerTelemetryState,
        )
        .unwrap();
        assert!(
            landing_footprint_for_state(&scenario, &state)
                .unwrap()
                .is_none()
        );
    }

    #[test]
    fn runner_footprint_monte_carlo_is_byte_identical_for_same_seed() {
        let scenario = Scenario::from_toml_str(COAST_FOOTPRINT_MC_SCENARIO).unwrap();
        let first = landing_footprint_monte_carlo_for_initial_state(&scenario)
            .unwrap()
            .unwrap();
        let second = landing_footprint_monte_carlo_for_initial_state(&scenario)
            .unwrap()
            .unwrap();
        assert_eq!(first.summary_toml, second.summary_toml);
        assert!(first.summary_toml.contains("[dispersion_statistics]"));
        assert!(first.summary_toml.contains("radial_dispersion_p50_m"));
        assert!(
            first
                .summary_toml
                .contains("[[nominal_radial_offset_quantiles]]")
        );
        let mut first_csv = Vec::new();
        let mut second_csv = Vec::new();
        first.samples.write_csv(&mut first_csv).unwrap();
        second.samples.write_csv(&mut second_csv).unwrap();
        assert_eq!(first_csv, second_csv);
        let csv_text = String::from_utf8(first_csv).unwrap();
        assert!(csv_text.contains("radial_offset_from_nominal_m"));
        assert!(!csv_text.contains("position_x_eci_m"));
        assert!(!csv_text.contains("velocity_x_eci_m_s"));
        assert!(!csv_text.contains("ballistic_coefficient_m2_kg"));
        assert!(!csv_text.contains("wind_x_m_s"));
    }

    #[test]
    fn runner_footprint_nested_monte_carlo_reports_pbox_and_variance_split() {
        let toml = COAST_FOOTPRINT_MC_SCENARIO
            .replace(
                "[landing_footprint.monte_carlo.wind]\nkind = \"constant\"",
                "[landing_footprint.monte_carlo.wind]\nuncertainty_class = \"epistemic\"\nkind = \"constant\"",
            )
            + "\n[landing_footprint.monte_carlo.nested]\n\
               epistemic_samples = 4\n\
               aleatory_samples = 8\n\
               metric = \"radial_offset_from_nominal_m\"\n\
               threshold = 1.0e9\n\
               minimum_probability = 1.0\n";
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let report = landing_footprint_monte_carlo_for_initial_state(&scenario)
            .unwrap()
            .unwrap();
        let nested = report.nested.as_ref().unwrap();
        assert_eq!(
            nested.metric,
            LandingFootprintMonteCarloNestedMetric::RadialOffsetFromNominalM,
        );
        assert_eq!(nested.analysis.epistemic_samples, 4);
        assert_eq!(nested.analysis.min_aleatory_samples, 8);
        assert_eq!(nested.analysis.max_aleatory_samples, 8);
        assert_eq!(
            nested.analysis.lower_bound_probability.map(f64::to_bits),
            Some(1.0_f64.to_bits()),
        );
        assert_eq!(nested.analysis.requirement_passed, Some(true));
        assert!(nested.analysis.variance_split.total >= 0.0);
        assert!(report.summary_toml.contains("[nested_uq]"));
        assert!(
            report
                .summary_toml
                .contains("metric = \"radial_offset_from_nominal_m\"")
        );
        assert!(report.summary_toml.contains("requirement_passed = true"));
    }

    #[test]
    fn runner_footprint_monte_carlo_bc_sigma_changes_spread() {
        let narrow = Scenario::from_toml_str(COAST_FOOTPRINT_MC_SCENARIO).unwrap();
        let wide_toml =
            COAST_FOOTPRINT_MC_SCENARIO.replace("sigma_m2_kg = 0.002", "sigma_m2_kg = 0.01");
        let wide = Scenario::from_toml_str(&wide_toml).unwrap();
        let narrow = landing_footprint_monte_carlo_for_initial_state(&narrow)
            .unwrap()
            .unwrap();
        let wide = landing_footprint_monte_carlo_for_initial_state(&wide)
            .unwrap()
            .unwrap();
        assert!(
            (wide.result.dispersion_ellipse.one_sigma_semi_major_m
                - narrow.result.dispersion_ellipse.one_sigma_semi_major_m)
                .abs()
                > 1.0e-9
        );
    }

    #[test]
    fn runner_footprint_monte_carlo_zero_uncertainty_collapses_to_nominal() {
        let toml = COAST_FOOTPRINT_MC_SCENARIO
            .replace(
                "sigma_ned_m_s = [2.0, 1.0, 0.0]",
                "sigma_ned_m_s = [0.0, 0.0, 0.0]",
            )
            .replace("sigma_m2_kg = 0.002", "sigma_m2_kg = 0.0")
            .replace(
                "position_sigma_eci_m = [1.0, 1.0, 0.5]",
                "position_sigma_eci_m = [0.0, 0.0, 0.0]",
            )
            .replace("speed_sigma_m_s = 0.5", "speed_sigma_m_s = 0.0")
            .replace("time_sigma_s = 0.01", "time_sigma_s = 0.0");
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let report = landing_footprint_monte_carlo_for_initial_state(&scenario)
            .unwrap()
            .unwrap();
        assert!(report.result.radial_dispersion_p50_m < 1.0e-12);
        assert!(report.result.mean_radial_offset_from_nominal_m < 1.0e-12);
        for sample in &report.result.samples {
            assert!(
                (sample.landing.downrange_m - report.result.nominal.downrange_m).abs() < 1.0e-12
            );
            assert!(
                (sample.landing.crossrange_m - report.result.nominal.crossrange_m).abs() < 1.0e-12
            );
        }
    }
}
