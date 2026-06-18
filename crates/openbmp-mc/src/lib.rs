//! Deterministic Monte Carlo campaign primitives.
//!
//! This crate is the reusable L7 substrate for OpenBMP Monte Carlo
//! campaigns. It owns campaign-layer sampling and reducers only: no
//! flight-software crate depends on it, and it does not alter the
//! locked simulation kernel operand order.

use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};

use nalgebra::DMatrix;
use openbmp_core::{DeterministicRng, FpEnvironment, FpEnvironmentDirty};
use openbmp_uq::{
    CorrelatedErrorBudget, CredibilityLevel, ProbabilityBox, UqError, ValidationStatus,
    VarianceSplit,
};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use statrs::distribution::{Beta, ContinuousCDF, Normal};
use thiserror::Error;

mod rare_event_sealed {
    pub trait LimitStateSealed {}
}

/// Error returned by Monte Carlo statistics helpers.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum MonteCarloError {
    /// A sample, probability, or confidence level was NaN, infinite,
    /// or outside the documented interval.
    #[error("{field} is outside the Monte Carlo statistics envelope")]
    InvalidInput {
        /// Field name that failed validation.
        field: &'static str,
    },
    /// Two samples declared the same deterministic campaign index.
    #[error("duplicate Monte Carlo sample index {index}")]
    DuplicateSampleIndex {
        /// Duplicate sample index.
        index: u64,
    },
    /// The beta inverse-CDF used by Clopper-Pearson rejected its
    /// parameters.
    #[error("Clopper-Pearson beta inverse failed for n={trials}, k={successes}")]
    InvalidBinomialInterval {
        /// Number of Bernoulli trials.
        trials: u64,
        /// Number of successful Bernoulli trials.
        successes: u64,
    },
    /// The current thread's floating-point environment does not satisfy the
    /// OpenBMP bit-stable profile.
    #[error(
        "floating-point environment is dirty ({observed:?}); cannot guarantee bit-stable Monte Carlo replay"
    )]
    FpEnvironmentDirty {
        /// Observed decoded floating-point environment.
        observed: FpEnvironment,
    },
    /// Checkpoint metadata does not match the requested campaign.
    #[error("checkpoint metadata mismatch in {field}")]
    CheckpointMismatch {
        /// Metadata field that did not match.
        field: &'static str,
    },
    /// Checkpoint finalization was requested before all samples completed.
    #[error("checkpoint is incomplete ({completed}/{required} samples completed)")]
    CheckpointIncomplete {
        /// Completed sample count.
        completed: u64,
        /// Required sample count.
        required: u64,
    },
    /// UQ budget or credibility evidence failed validation.
    #[error("UQ error: {0}")]
    Uq(#[from] UqError),
}

impl From<FpEnvironmentDirty> for MonteCarloError {
    fn from(error: FpEnvironmentDirty) -> Self {
        Self::FpEnvironmentDirty {
            observed: error.observed,
        }
    }
}

/// Error returned by the JSON file checkpoint store.
#[derive(Debug, Error)]
pub enum CheckpointStoreError {
    /// Reading, writing, creating a parent directory, or replacing the
    /// checkpoint file failed.
    #[error("checkpoint IO error: {path}")]
    Io {
        /// Path involved in the failed file operation.
        path: PathBuf,
        /// Source IO error.
        #[source]
        source: std::io::Error,
    },
    /// Checkpoint JSON could not be serialized or parsed.
    #[error("checkpoint JSON error: {path}")]
    Json {
        /// Path involved in the failed JSON operation.
        path: PathBuf,
        /// Source JSON error.
        #[source]
        source: serde_json::Error,
    },
    /// Checkpoint content failed Monte Carlo domain validation.
    #[error("checkpoint content error")]
    Content(#[from] MonteCarloError),
}

/// Deterministic per-sample random stream for a Monte Carlo campaign.
///
/// The stream is backed by [`DeterministicRng::for_mc_sample`], with
/// an optional cached Box-Muller normal variate. A given
/// `(campaign_seed, sample_index, dimension_id)` tuple always produces
/// the same stream regardless of worker count or scheduling order.
#[derive(Clone, Debug)]
pub struct SampleRng {
    inner: DeterministicRng,
    spare_normal: Option<f64>,
}

/// Credibility and UQ summary attached to an MC campaign report.
#[derive(Clone, Debug, PartialEq)]
pub struct CampaignCredibilityReport {
    /// Aggregate one-sigma value over all uncertainty sources.
    pub aggregate_one_sigma: f64,
    /// Aggregate one-sigma value over aleatory sources.
    pub aleatory_one_sigma: f64,
    /// Aggregate one-sigma value over epistemic sources.
    pub epistemic_one_sigma: f64,
    /// Binding 7009B-shaped credibility level across all sources.
    pub binding_level: CredibilityLevel,
    /// Public OpenBMP validation label corresponding to `binding_level`.
    pub legacy_label: ValidationStatus,
    /// Required floor for this campaign verdict.
    pub floor: CredibilityLevel,
    /// `true` when `binding_level >= floor`.
    pub accepted: bool,
    /// Deterministic Markdown evidence summary.
    pub markdown: String,
}

/// Lower-tail probability requirement for a nested scalar campaign.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NestedLowerTailRequirement {
    /// Scalar threshold `y*` in `P(y <= y*)`.
    pub threshold: f64,
    /// Required lower-bound probability.
    pub minimum_probability: f64,
}

/// P-box and aleatory/epistemic variance report for nested scalar samples.
#[derive(Clone, Debug, PartialEq)]
pub struct NestedScalarAnalysisReport {
    /// Number of epistemic outer-loop conditions.
    pub epistemic_samples: usize,
    /// Minimum aleatory inner-loop samples under any epistemic condition.
    pub min_aleatory_samples: usize,
    /// Maximum aleatory inner-loop samples under any epistemic condition.
    pub max_aleatory_samples: usize,
    /// Conditional-CDF p-box envelope.
    pub pbox: ProbabilityBox,
    /// Law-of-total-variance split.
    pub variance_split: VarianceSplit,
    /// Lower p-box probability at the requested threshold, when supplied.
    pub lower_bound_probability: Option<f64>,
    /// Requirement verdict evaluated on the lower p-box bound, when supplied.
    pub requirement_passed: Option<bool>,
}

/// Standard-normal coordinate vector passed to synthetic rare-event limit
/// states.
#[derive(Clone, Debug, PartialEq)]
pub struct StandardNormalPoint {
    coordinates: Vec<f64>,
}

impl StandardNormalPoint {
    /// Construct a finite standard-normal point.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] when the coordinate vector is
    /// empty or contains a non-finite value.
    pub fn new(coordinates: Vec<f64>) -> Result<Self, MonteCarloError> {
        if coordinates.is_empty() {
            return Err(MonteCarloError::InvalidInput {
                field: "limit_state.dimension",
            });
        }
        if coordinates.iter().any(|value| !value.is_finite()) {
            return Err(MonteCarloError::InvalidInput {
                field: "limit_state.coordinates",
            });
        }
        Ok(Self { coordinates })
    }

    /// Coordinate slice.
    #[must_use]
    pub fn coordinates(&self) -> &[f64] {
        &self.coordinates
    }

    /// Number of standard-normal coordinates.
    #[must_use]
    pub fn dimension(&self) -> usize {
        self.coordinates.len()
    }
}

/// Closed, synthetic rare-event limit-state interface.
///
/// The vocabulary is intentionally forward-only and standard-normal: callers
/// can exercise rare-event algorithms on analytic synthetic cases, but this API
/// does not bind any result to a named vehicle, ground aimpoint, or operational
/// loss metric.
pub trait LimitState: rare_event_sealed::LimitStateSealed {
    /// Stable synthetic label for evidence records.
    fn label(&self) -> &'static str;

    /// Standard-normal input dimension.
    fn dimension(&self) -> usize;

    /// Limit-state value `g(x)`. Failure is defined as `g(x) <= 0`.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError`] if the point is outside the limit-state
    /// domain.
    fn evaluate(&self, point: &StandardNormalPoint) -> Result<f64, MonteCarloError>;

    /// Optional analytic failure probability for synthetic comparison.
    fn analytic_failure_probability(&self) -> Option<f64> {
        None
    }
}

/// Analytic linear standard-normal limit state
/// `g(x) = beta - (1 / sqrt(d)) * sum(x_i)`.
///
/// Failure probability is exactly `Phi(-beta)`, so it is suitable for
/// deterministic synthetic validation of rare-event estimators.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SyntheticLinearLimitState {
    beta: f64,
    dimension: usize,
}

impl SyntheticLinearLimitState {
    /// Build a linear synthetic limit state.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] when `beta` is non-finite or
    /// not positive, or when `dimension == 0`.
    pub fn new(beta: f64, dimension: usize) -> Result<Self, MonteCarloError> {
        if !beta.is_finite() || beta <= 0.0 {
            return Err(MonteCarloError::InvalidInput {
                field: "limit_state.beta",
            });
        }
        if dimension == 0 {
            return Err(MonteCarloError::InvalidInput {
                field: "limit_state.dimension",
            });
        }
        Ok(Self { beta, dimension })
    }

    /// Reliability index `beta`.
    #[must_use]
    pub const fn beta(&self) -> f64 {
        self.beta
    }

    /// Analytic failure probability `Phi(-beta)`.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] if the normal distribution
    /// helper rejects its parameters.
    pub fn exact_failure_probability(&self) -> Result<f64, MonteCarloError> {
        standard_normal().map(|normal| normal.cdf(-self.beta))
    }
}

impl LimitState for SyntheticLinearLimitState {
    fn label(&self) -> &'static str {
        "synthetic-limit-state"
    }

    fn dimension(&self) -> usize {
        self.dimension
    }

    fn evaluate(&self, point: &StandardNormalPoint) -> Result<f64, MonteCarloError> {
        if point.dimension() != self.dimension {
            return Err(MonteCarloError::InvalidInput {
                field: "limit_state.dimension",
            });
        }
        let scale = (self.dimension as f64).sqrt();
        let projection = point.coordinates().iter().sum::<f64>() / scale;
        Ok(self.beta - projection)
    }

    fn analytic_failure_probability(&self) -> Option<f64> {
        self.exact_failure_probability().ok()
    }
}

impl rare_event_sealed::LimitStateSealed for SyntheticLinearLimitState {}

/// Rare-event estimator method.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RareEventMethod {
    /// Au-Beck style subset simulation with deterministic modified-Metropolis
    /// chains in standard-normal coordinates.
    SubsetSimulation,
    /// Cross-entropy Gaussian importance sampling with deterministic elite
    /// updates.
    CrossEntropyImportanceSampling,
}

impl RareEventMethod {
    /// Stable report label.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::SubsetSimulation => "subset_simulation",
            Self::CrossEntropyImportanceSampling => "cross_entropy_importance_sampling",
        }
    }
}

/// One subset-simulation threshold row.
#[derive(Clone, Debug, PartialEq)]
pub struct SubsetSimulationLevel {
    /// Zero-based level index.
    pub level: u32,
    /// Adaptive threshold on `g(x)`, with failure at `g <= 0`.
    pub threshold: f64,
    /// Conditional probability factor contributed by this level.
    pub conditional_probability: f64,
    /// Number of failures under the true `g <= 0` event at this level.
    pub failures: usize,
}

/// One cross-entropy adaptation row.
#[derive(Clone, Debug, PartialEq)]
pub struct CrossEntropyIteration {
    /// Zero-based iteration index.
    pub iteration: u32,
    /// Elite threshold on `g(x)`.
    pub elite_threshold: f64,
    /// Proposal mean after smoothing.
    pub mean: Vec<f64>,
    /// Proposal standard deviation after smoothing.
    pub std_dev: Vec<f64>,
}

/// Rare-event probability estimate with synthetic analytic comparison.
#[derive(Clone, Debug, PartialEq)]
pub struct RareEventEstimateReport {
    /// Estimator method.
    pub method: RareEventMethod,
    /// Synthetic limit-state label. This must remain
    /// `"synthetic-limit-state"` for the built-in limit states.
    pub limit_state_label: &'static str,
    /// Standard-normal dimension.
    pub dimension: usize,
    /// Total limit-state evaluations.
    pub evaluations: u64,
    /// Estimated failure probability.
    pub failure_probability: f64,
    /// Estimated coefficient of variation for the probability estimate.
    pub coefficient_of_variation: f64,
    /// Analytic probability, when supplied by the synthetic limit state.
    pub analytic_probability: Option<f64>,
    /// Absolute log10 error versus analytic probability, when available.
    pub abs_log10_error: Option<f64>,
    /// Subset-simulation threshold trace.
    pub subset_levels: Vec<SubsetSimulationLevel>,
    /// Cross-entropy adaptation trace.
    pub cross_entropy_iterations: Vec<CrossEntropyIteration>,
}

/// Au-Beck style subset-simulation estimator configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SubsetSimulation {
    /// Deterministic campaign seed.
    pub campaign_seed: u64,
    /// Number of samples per conditional level.
    pub samples_per_level: usize,
    /// Target conditional probability per intermediate level, usually `0.1`.
    pub conditional_probability: f64,
    /// Maximum number of conditional levels.
    pub max_levels: u32,
    /// Random-walk proposal sigma for the modified-Metropolis sampler.
    pub proposal_sigma: f64,
    /// Dimension id used to separate deterministic RNG streams.
    pub dimension_id: u32,
}

impl SubsetSimulation {
    /// Construct a validated subset-simulation configuration.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] when the sample count, level
    /// count, conditional probability, or proposal sigma is outside the
    /// supported synthetic envelope.
    pub fn new(
        campaign_seed: u64,
        samples_per_level: usize,
        conditional_probability: f64,
        max_levels: u32,
        proposal_sigma: f64,
        dimension_id: u32,
    ) -> Result<Self, MonteCarloError> {
        if samples_per_level < 2 {
            return Err(MonteCarloError::InvalidInput {
                field: "subset.samples_per_level",
            });
        }
        validate_probability("subset.conditional_probability", conditional_probability)?;
        if max_levels == 0 {
            return Err(MonteCarloError::InvalidInput {
                field: "subset.max_levels",
            });
        }
        if !proposal_sigma.is_finite() || proposal_sigma <= 0.0 {
            return Err(MonteCarloError::InvalidInput {
                field: "subset.proposal_sigma",
            });
        }
        Ok(Self {
            campaign_seed,
            samples_per_level,
            conditional_probability,
            max_levels,
            proposal_sigma,
            dimension_id,
        })
    }

    /// Run subset simulation against a synthetic limit state.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError`] when the limit state or estimator
    /// configuration is invalid.
    pub fn estimate<L: LimitState>(
        &self,
        limit_state: &L,
    ) -> Result<RareEventEstimateReport, MonteCarloError> {
        validate_limit_state(limit_state)?;
        let elite_count = elite_count(self.samples_per_level, self.conditional_probability)?;
        let mut samples = initial_standard_normal_samples(
            self.campaign_seed,
            self.samples_per_level,
            limit_state.dimension(),
            self.dimension_id,
        )?;
        let mut evaluations = self.samples_per_level as u64;
        let mut probability_prefix = 1.0;
        let mut levels = Vec::new();

        for level in 0..self.max_levels {
            let mut values = evaluate_limit_state(limit_state, &samples)?;
            values.sort_by(|left, right| left.1.total_cmp(&right.1));
            let failures = values.iter().filter(|(_, value)| *value <= 0.0).count();
            let threshold = values[elite_count - 1].1;
            if threshold <= 0.0 {
                let conditional_probability = failures as f64 / self.samples_per_level as f64;
                let probability = probability_prefix * conditional_probability;
                let mut levels = levels;
                levels.push(SubsetSimulationLevel {
                    level,
                    threshold,
                    conditional_probability,
                    failures,
                });
                return rare_event_report(
                    RareEventMethod::SubsetSimulation,
                    limit_state,
                    evaluations,
                    probability,
                    subset_cov_bound(&levels, self.samples_per_level),
                    levels,
                    Vec::new(),
                );
            }

            probability_prefix *= self.conditional_probability;
            levels.push(SubsetSimulationLevel {
                level,
                threshold,
                conditional_probability: self.conditional_probability,
                failures,
            });
            let seeds = values
                .iter()
                .take(elite_count)
                .map(|(index, _)| samples[*index].clone())
                .collect::<Vec<_>>();
            samples = conditional_mma_samples(
                limit_state,
                &seeds,
                threshold,
                ConditionalMmaConfig {
                    sample_count: self.samples_per_level,
                    campaign_seed: self.campaign_seed,
                    dimension_id: self.dimension_id,
                    level,
                    proposal_sigma: self.proposal_sigma,
                },
            )?;
            evaluations = evaluations
                .checked_add(self.samples_per_level as u64)
                .ok_or(MonteCarloError::InvalidInput {
                    field: "subset.evaluations",
                })?;
        }

        Err(MonteCarloError::InvalidInput {
            field: "subset.max_levels",
        })
    }
}

/// Cross-entropy Gaussian importance-sampling configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CrossEntropyIs {
    /// Deterministic campaign seed.
    pub campaign_seed: u64,
    /// Number of samples per adaptation/final-estimation batch.
    pub samples: usize,
    /// Elite fraction used for Gaussian proposal updates.
    pub elite_fraction: f64,
    /// Number of proposal-adaptation iterations.
    pub iterations: u32,
    /// Exponential smoothing factor applied to proposal parameters.
    pub smoothing: f64,
    /// Lower bound for proposal standard deviations.
    pub min_std_dev: f64,
    /// Dimension id used to separate deterministic RNG streams.
    pub dimension_id: u32,
}

impl CrossEntropyIs {
    /// Construct a validated CE-IS configuration.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] when any control parameter is
    /// outside the supported synthetic envelope.
    pub fn new(
        campaign_seed: u64,
        samples: usize,
        elite_fraction: f64,
        iterations: u32,
        smoothing: f64,
        min_std_dev: f64,
        dimension_id: u32,
    ) -> Result<Self, MonteCarloError> {
        if samples < 2 {
            return Err(MonteCarloError::InvalidInput {
                field: "ce.samples",
            });
        }
        validate_probability("ce.elite_fraction", elite_fraction)?;
        if iterations == 0 {
            return Err(MonteCarloError::InvalidInput {
                field: "ce.iterations",
            });
        }
        validate_probability("ce.smoothing", smoothing)?;
        if !min_std_dev.is_finite() || min_std_dev <= 0.0 {
            return Err(MonteCarloError::InvalidInput {
                field: "ce.min_std_dev",
            });
        }
        Ok(Self {
            campaign_seed,
            samples,
            elite_fraction,
            iterations,
            smoothing,
            min_std_dev,
            dimension_id,
        })
    }

    /// Run CE-IS against a synthetic limit state.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError`] when sampling, adaptation, or weighted
    /// estimation leaves the supported synthetic envelope.
    pub fn estimate<L: LimitState>(
        &self,
        limit_state: &L,
    ) -> Result<RareEventEstimateReport, MonteCarloError> {
        validate_limit_state(limit_state)?;
        let dimension = limit_state.dimension();
        let elite_count = elite_count(self.samples, self.elite_fraction)?;
        let mut mean = vec![0.0; dimension];
        let mut std_dev = vec![1.0; dimension];
        let mut iterations = Vec::new();

        for iteration in 0..self.iterations {
            let samples = proposal_samples(
                self.campaign_seed,
                self.samples,
                self.dimension_id,
                iteration,
                &mean,
                &std_dev,
            )?;
            let mut values = evaluate_limit_state(limit_state, &samples)?;
            values.sort_by(|left, right| left.1.total_cmp(&right.1));
            let elite_threshold = values[elite_count - 1].1;
            let elites = if elite_threshold <= 0.0 {
                values
                    .iter()
                    .filter(|(_, value)| *value <= 0.0)
                    .map(|(index, _)| &samples[*index])
                    .collect::<Vec<_>>()
            } else {
                values
                    .iter()
                    .take(elite_count)
                    .map(|(index, _)| &samples[*index])
                    .collect::<Vec<_>>()
            };
            let (elite_mean, elite_std_dev) =
                diagonal_gaussian_from_elites(&elites, self.min_std_dev)?;
            for index in 0..dimension {
                mean[index] =
                    (1.0 - self.smoothing) * mean[index] + self.smoothing * elite_mean[index];
                std_dev[index] = ((1.0 - self.smoothing) * std_dev[index]
                    + self.smoothing * elite_std_dev[index])
                    .max(self.min_std_dev);
            }
            iterations.push(CrossEntropyIteration {
                iteration,
                elite_threshold,
                mean: mean.clone(),
                std_dev: std_dev.clone(),
            });
            if elite_threshold <= 0.0 {
                break;
            }
        }

        let samples = proposal_samples(
            self.campaign_seed,
            self.samples,
            self.dimension_id,
            self.iterations,
            &mean,
            &std_dev,
        )?;
        let mut weighted = Vec::with_capacity(samples.len());
        let mut estimate = 0.0;
        for sample in &samples {
            let value = limit_state.evaluate(sample)?;
            let weight = standard_normal_to_diagonal_proposal_weight(sample, &mean, &std_dev)?;
            let contribution = if value <= 0.0 { weight } else { 0.0 };
            estimate += contribution;
            weighted.push(contribution);
        }
        estimate /= self.samples as f64;
        if !estimate.is_finite() || estimate <= 0.0 {
            return Err(MonteCarloError::InvalidInput {
                field: "ce.failure_probability",
            });
        }
        let covariance = weighted
            .iter()
            .map(|value| {
                let delta = *value - estimate;
                delta * delta
            })
            .sum::<f64>()
            / (self.samples.saturating_sub(1) as f64);
        let standard_error = (covariance / self.samples as f64).sqrt();
        rare_event_report(
            RareEventMethod::CrossEntropyImportanceSampling,
            limit_state,
            (iterations.len() as u64 + 1) * self.samples as u64,
            estimate,
            standard_error / estimate,
            Vec::new(),
            iterations,
        )
    }
}

/// Evaluate a source-tagged UQ budget for campaign reporting and floor checks.
///
/// # Errors
///
/// Returns [`MonteCarloError`] when the budget is malformed or contains no
/// sources.
pub fn evaluate_campaign_credibility(
    budget: &CorrelatedErrorBudget,
    floor: CredibilityLevel,
) -> Result<CampaignCredibilityReport, MonteCarloError> {
    let aggregate_one_sigma = budget.aggregate_one_sigma()?;
    let aleatory_one_sigma = budget.aleatory_one_sigma()?;
    let epistemic_one_sigma = budget.epistemic_one_sigma()?;
    let binding = budget
        .binding_credibility()
        .ok_or(MonteCarloError::InvalidInput {
            field: "credibility.sources",
        })?;
    let binding_level = binding.binding_level();
    let legacy_label = binding.legacy_label();
    let accepted = binding_level >= floor;
    let markdown = campaign_credibility_markdown(
        aggregate_one_sigma,
        aleatory_one_sigma,
        epistemic_one_sigma,
        floor,
        accepted,
        &binding,
    );
    Ok(CampaignCredibilityReport {
        aggregate_one_sigma,
        aleatory_one_sigma,
        epistemic_one_sigma,
        binding_level,
        legacy_label,
        floor,
        accepted,
        markdown,
    })
}

/// Reduce nested aleatory/epistemic scalar samples to p-box and variance split.
///
/// `conditional_samples[epistemic_index][aleatory_index]` is the scalar QoI for
/// one inner-loop aleatory draw under a fixed outer-loop epistemic condition.
/// If a requirement is supplied, it is evaluated against the lower p-box bound.
///
/// # Errors
///
/// Returns [`MonteCarloError`] when nested samples, p-box construction, variance
/// decomposition, or requirement probability inputs are invalid.
pub fn analyze_nested_scalar_samples(
    conditional_samples: &[Vec<f64>],
    requirement: Option<NestedLowerTailRequirement>,
) -> Result<NestedScalarAnalysisReport, MonteCarloError> {
    let pbox = ProbabilityBox::from_conditional_samples(conditional_samples)?;
    let variance_split = VarianceSplit::from_conditional_samples(conditional_samples)?;
    let min_aleatory_samples =
        conditional_samples
            .iter()
            .map(Vec::len)
            .min()
            .ok_or(MonteCarloError::InvalidInput {
                field: "nested.epistemic_samples",
            })?;
    let max_aleatory_samples =
        conditional_samples
            .iter()
            .map(Vec::len)
            .max()
            .ok_or(MonteCarloError::InvalidInput {
                field: "nested.epistemic_samples",
            })?;
    let (lower_bound_probability, requirement_passed) = if let Some(requirement) = requirement {
        let probability = pbox.lower_cdf_at(requirement.threshold)?;
        let passed = pbox.verify_lower_tail_probability(
            requirement.threshold,
            requirement.minimum_probability,
        )?;
        (Some(probability), Some(passed))
    } else {
        (None, None)
    };

    Ok(NestedScalarAnalysisReport {
        epistemic_samples: conditional_samples.len(),
        min_aleatory_samples,
        max_aleatory_samples,
        pbox,
        variance_split,
        lower_bound_probability,
        requirement_passed,
    })
}

fn rare_event_report<L: LimitState>(
    method: RareEventMethod,
    limit_state: &L,
    evaluations: u64,
    failure_probability: f64,
    coefficient_of_variation: f64,
    subset_levels: Vec<SubsetSimulationLevel>,
    cross_entropy_iterations: Vec<CrossEntropyIteration>,
) -> Result<RareEventEstimateReport, MonteCarloError> {
    validate_probability("rare_event.failure_probability", failure_probability)?;
    if !coefficient_of_variation.is_finite() || coefficient_of_variation < 0.0 {
        return Err(MonteCarloError::InvalidInput {
            field: "rare_event.coefficient_of_variation",
        });
    }
    let analytic_probability = limit_state.analytic_failure_probability();
    let abs_log10_error =
        analytic_probability.map(|analytic| (failure_probability.log10() - analytic.log10()).abs());
    if abs_log10_error.is_some_and(|value| !value.is_finite()) {
        return Err(MonteCarloError::InvalidInput {
            field: "rare_event.abs_log10_error",
        });
    }
    Ok(RareEventEstimateReport {
        method,
        limit_state_label: limit_state.label(),
        dimension: limit_state.dimension(),
        evaluations,
        failure_probability,
        coefficient_of_variation,
        analytic_probability,
        abs_log10_error,
        subset_levels,
        cross_entropy_iterations,
    })
}

fn validate_limit_state<L: LimitState>(limit_state: &L) -> Result<(), MonteCarloError> {
    if limit_state.label() != "synthetic-limit-state" {
        return Err(MonteCarloError::InvalidInput {
            field: "limit_state.label",
        });
    }
    if limit_state.dimension() == 0 {
        return Err(MonteCarloError::InvalidInput {
            field: "limit_state.dimension",
        });
    }
    Ok(())
}

fn standard_normal() -> Result<Normal, MonteCarloError> {
    Normal::new(0.0, 1.0).map_err(|_| MonteCarloError::InvalidInput {
        field: "standard_normal",
    })
}

fn elite_count(sample_count: usize, fraction: f64) -> Result<usize, MonteCarloError> {
    let count = (sample_count as f64 * fraction).ceil() as usize;
    if count < 2 || count >= sample_count {
        return Err(MonteCarloError::InvalidInput {
            field: "rare_event.elite_count",
        });
    }
    Ok(count)
}

fn initial_standard_normal_samples(
    campaign_seed: u64,
    sample_count: usize,
    dimension: usize,
    dimension_id: u32,
) -> Result<Vec<StandardNormalPoint>, MonteCarloError> {
    (0..sample_count)
        .map(|sample_index| {
            let mut rng = SampleRng::new(campaign_seed, sample_index as u64, dimension_id);
            let coordinates = (0..dimension)
                .map(|_| rng.standard_normal())
                .collect::<Vec<_>>();
            StandardNormalPoint::new(coordinates)
        })
        .collect()
}

fn proposal_samples(
    campaign_seed: u64,
    sample_count: usize,
    dimension_id: u32,
    iteration: u32,
    mean: &[f64],
    std_dev: &[f64],
) -> Result<Vec<StandardNormalPoint>, MonteCarloError> {
    validate_diagonal_proposal(mean, std_dev)?;
    (0..sample_count)
        .map(|sample_index| {
            let stream_index = (u64::from(iteration) << 32) | sample_index as u64;
            let mut rng = SampleRng::new(campaign_seed, stream_index, dimension_id);
            let coordinates = mean
                .iter()
                .zip(std_dev)
                .map(|(mu, sigma)| mu + sigma * rng.standard_normal())
                .collect::<Vec<_>>();
            StandardNormalPoint::new(coordinates)
        })
        .collect()
}

fn evaluate_limit_state<L: LimitState>(
    limit_state: &L,
    samples: &[StandardNormalPoint],
) -> Result<Vec<(usize, f64)>, MonteCarloError> {
    let mut values = Vec::with_capacity(samples.len());
    for (index, sample) in samples.iter().enumerate() {
        let value = limit_state.evaluate(sample)?;
        if !value.is_finite() {
            return Err(MonteCarloError::InvalidInput {
                field: "limit_state.g",
            });
        }
        values.push((index, value));
    }
    Ok(values)
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct ConditionalMmaConfig {
    sample_count: usize,
    campaign_seed: u64,
    dimension_id: u32,
    level: u32,
    proposal_sigma: f64,
}

fn conditional_mma_samples<L: LimitState>(
    limit_state: &L,
    seeds: &[StandardNormalPoint],
    threshold: f64,
    config: ConditionalMmaConfig,
) -> Result<Vec<StandardNormalPoint>, MonteCarloError> {
    if seeds.is_empty() {
        return Err(MonteCarloError::InvalidInput {
            field: "subset.seeds",
        });
    }
    let mut output = Vec::with_capacity(config.sample_count);
    for (chain_index, seed) in seeds.iter().enumerate() {
        let base = config.sample_count / seeds.len();
        let extra = usize::from(chain_index < config.sample_count % seeds.len());
        let chain_len = base + extra;
        let mut current = seed.clone();
        for step in 0..chain_len {
            let stream_index =
                (u64::from(config.level) << 48) | ((chain_index as u64) << 24) | step as u64;
            let mut rng = SampleRng::new(config.campaign_seed, stream_index, config.dimension_id);
            let mut proposal = current.coordinates().to_vec();
            for coordinate in &mut proposal {
                let candidate = *coordinate + config.proposal_sigma * rng.standard_normal();
                let log_acceptance = 0.5 * (*coordinate * *coordinate - candidate * candidate);
                if rng.uniform01() <= log_acceptance.exp().min(1.0) {
                    *coordinate = candidate;
                }
            }
            let candidate = StandardNormalPoint::new(proposal)?;
            if limit_state.evaluate(&candidate)? <= threshold {
                current = candidate;
            }
            output.push(current.clone());
        }
    }
    output.truncate(config.sample_count);
    Ok(output)
}

fn subset_cov_bound(levels: &[SubsetSimulationLevel], sample_count: usize) -> f64 {
    let variance = levels
        .iter()
        .map(|level| {
            let probability = level.conditional_probability;
            (1.0 - probability) / (sample_count as f64 * probability)
        })
        .sum::<f64>();
    variance.sqrt()
}

fn diagonal_gaussian_from_elites(
    elites: &[&StandardNormalPoint],
    min_std_dev: f64,
) -> Result<(Vec<f64>, Vec<f64>), MonteCarloError> {
    let Some(first) = elites.first() else {
        return Err(MonteCarloError::InvalidInput { field: "ce.elites" });
    };
    let dimension = first.dimension();
    let mut mean = vec![0.0; dimension];
    for elite in elites {
        if elite.dimension() != dimension {
            return Err(MonteCarloError::InvalidInput {
                field: "ce.dimension",
            });
        }
        for (index, value) in elite.coordinates().iter().enumerate() {
            mean[index] += *value;
        }
    }
    for value in &mut mean {
        *value /= elites.len() as f64;
    }
    let mut variance = vec![0.0; dimension];
    for elite in elites {
        for (index, value) in elite.coordinates().iter().enumerate() {
            let delta = *value - mean[index];
            variance[index] += delta * delta;
        }
    }
    let std_dev = variance
        .into_iter()
        .map(|value| ((value / elites.len() as f64).sqrt()).max(min_std_dev))
        .collect::<Vec<_>>();
    validate_diagonal_proposal(&mean, &std_dev)?;
    Ok((mean, std_dev))
}

fn validate_diagonal_proposal(mean: &[f64], std_dev: &[f64]) -> Result<(), MonteCarloError> {
    if mean.is_empty() || mean.len() != std_dev.len() {
        return Err(MonteCarloError::InvalidInput {
            field: "ce.dimension",
        });
    }
    if mean.iter().any(|value| !value.is_finite()) {
        return Err(MonteCarloError::InvalidInput { field: "ce.mean" });
    }
    if std_dev
        .iter()
        .any(|value| !value.is_finite() || *value <= 0.0)
    {
        return Err(MonteCarloError::InvalidInput {
            field: "ce.std_dev",
        });
    }
    Ok(())
}

fn standard_normal_to_diagonal_proposal_weight(
    point: &StandardNormalPoint,
    mean: &[f64],
    std_dev: &[f64],
) -> Result<f64, MonteCarloError> {
    validate_diagonal_proposal(mean, std_dev)?;
    if point.dimension() != mean.len() {
        return Err(MonteCarloError::InvalidInput {
            field: "ce.dimension",
        });
    }
    let mut log_weight = 0.0;
    for ((x, mu), sigma) in point.coordinates().iter().zip(mean).zip(std_dev) {
        let z = (*x - *mu) / *sigma;
        log_weight += sigma.ln() + 0.5 * (z * z - *x * *x);
    }
    let weight = log_weight.exp();
    if weight.is_finite() {
        Ok(weight)
    } else {
        Err(MonteCarloError::InvalidInput {
            field: "ce.importance_weight",
        })
    }
}

fn campaign_credibility_markdown(
    aggregate_one_sigma: f64,
    aleatory_one_sigma: f64,
    epistemic_one_sigma: f64,
    floor: CredibilityLevel,
    accepted: bool,
    binding: &openbmp_uq::CredibilityRecord,
) -> String {
    use std::fmt::Write;

    let mut out = String::new();
    let _ = writeln!(out, "# Monte Carlo credibility report");
    let _ = writeln!(out);
    let _ = writeln!(
        out,
        "> Academic simulation evidence; not validated for operational flight."
    );
    let _ = writeln!(out);
    let _ = writeln!(out, "- aggregate_1sigma = {aggregate_one_sigma:.12e}");
    let _ = writeln!(out, "- aleatory_1sigma = {aleatory_one_sigma:.12e}");
    let _ = writeln!(out, "- epistemic_1sigma = {epistemic_one_sigma:.12e}");
    let _ = writeln!(out, "- floor = {}", floor.as_label());
    let _ = writeln!(out, "- accepted = {accepted}");
    let _ = writeln!(out);
    out.push_str(&binding.render_markdown());
    out
}

impl SampleRng {
    /// Construct a sample stream from a campaign seed, sample index,
    /// and dimension id.
    #[must_use]
    pub fn new(campaign_seed: u64, sample_index: u64, dimension_id: u32) -> Self {
        Self {
            inner: DeterministicRng::for_mc_sample(campaign_seed, sample_index, dimension_id),
            spare_normal: None,
        }
    }

    /// Return the next `u64` from the sample stream.
    #[must_use]
    pub fn next_u64(&mut self) -> u64 {
        self.inner.next_u64()
    }

    /// Uniform variate in `[0, 1)`, generated from the top 53 bits of
    /// the next deterministic `u64`.
    #[must_use]
    pub fn uniform01(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Standard-normal variate via Box-Muller with one cached sibling
    /// value. The cached value is local to this stream and therefore
    /// does not perturb any other sample or dimension.
    #[must_use]
    pub fn standard_normal(&mut self) -> f64 {
        if let Some(z) = self.spare_normal.take() {
            return z;
        }
        let u1 = self.uniform01().max(f64::MIN_POSITIVE);
        let u2 = self.uniform01();
        let r = (-2.0 * u1.ln()).sqrt();
        let theta = std::f64::consts::TAU * u2;
        self.spare_normal = Some(r * theta.sin());
        r * theta.cos()
    }
}

/// Engine fault payloads supported by deterministic MC propulsion fault
/// libraries.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum PropulsionFaultPayload {
    /// Engine commanded off and never restarted.
    HardOff,
    /// Thrust multiplier fault.
    OverThrust {
        /// Finite non-negative thrust multiplier.
        factor: f64,
    },
    /// Ignition over-pressure transient.
    HardStartOverpressure {
        /// Finite multiplier at least one.
        factor: f64,
        /// Positive finite duration from ignition start.
        duration_s: f64,
    },
    /// Pump-cavitation thrust loss multiplier.
    CavitationThrustLoss {
        /// Finite multiplier in `[0, 1]`.
        factor: f64,
    },
    /// Frozen gimbal command.
    GimbalLocked {
        /// Locked pitch angle in radians.
        pitch_rad: f64,
        /// Locked yaw angle in radians.
        yaw_rad: f64,
    },
}

impl PropulsionFaultPayload {
    fn validate(&self) -> Result<(), MonteCarloError> {
        match self {
            Self::HardOff => Ok(()),
            Self::OverThrust { factor } => {
                if factor.is_finite() && *factor >= 0.0 {
                    Ok(())
                } else {
                    Err(MonteCarloError::InvalidInput {
                        field: "fault.factor",
                    })
                }
            }
            Self::HardStartOverpressure { factor, duration_s } => {
                if !factor.is_finite() || *factor < 1.0 {
                    return Err(MonteCarloError::InvalidInput {
                        field: "fault.factor",
                    });
                }
                if duration_s.is_finite() && *duration_s > 0.0 {
                    Ok(())
                } else {
                    Err(MonteCarloError::InvalidInput {
                        field: "fault.duration_s",
                    })
                }
            }
            Self::CavitationThrustLoss { factor } => {
                if factor.is_finite() && (0.0..=1.0).contains(factor) {
                    Ok(())
                } else {
                    Err(MonteCarloError::InvalidInput {
                        field: "fault.factor",
                    })
                }
            }
            Self::GimbalLocked { pitch_rad, yaw_rad } => {
                if pitch_rad.is_finite() && yaw_rad.is_finite() {
                    Ok(())
                } else {
                    Err(MonteCarloError::InvalidInput {
                        field: "fault.gimbal",
                    })
                }
            }
        }
    }

    fn to_toml_inline(&self) -> String {
        match self {
            Self::HardOff => "{ kind = \"hard_off\" }".to_owned(),
            Self::OverThrust { factor } => {
                format!(
                    "{{ kind = \"over_thrust\", factor = {} }}",
                    toml_f64(*factor)
                )
            }
            Self::HardStartOverpressure { factor, duration_s } => format!(
                "{{ kind = \"hard_start_overpressure\", factor = {}, duration_s = {} }}",
                toml_f64(*factor),
                toml_f64(*duration_s)
            ),
            Self::CavitationThrustLoss { factor } => format!(
                "{{ kind = \"cavitation_thrust_loss\", factor = {} }}",
                toml_f64(*factor)
            ),
            Self::GimbalLocked { pitch_rad, yaw_rad } => format!(
                "{{ kind = \"gimbal_locked\", pitch_rad = {}, yaw_rad = {} }}",
                toml_f64(*pitch_rad),
                toml_f64(*yaw_rad)
            ),
        }
    }
}

/// One probabilistic scheduled propulsion fault template in an MC library.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
pub struct PropulsionFaultTemplate {
    /// Stable scheduled rule id.
    pub id: String,
    /// Scenario-text engine id targeted by this rule.
    pub engine_id: String,
    /// Kernel step at which the scheduled rule injects.
    pub start_step: u64,
    /// Per-sample activation probability in `[0, 1]`.
    pub probability: f64,
    /// Engine fault payload to materialize when the rule activates.
    pub fault: PropulsionFaultPayload,
}

impl PropulsionFaultTemplate {
    fn validate(&self) -> Result<(), MonteCarloError> {
        validate_toml_string_field(&self.id, "fault.id")?;
        validate_toml_string_field(&self.engine_id, "fault.engine_id")?;
        if !self.probability.is_finite() || !(0.0..=1.0).contains(&self.probability) {
            return Err(MonteCarloError::InvalidInput {
                field: "fault.probability",
            });
        }
        self.fault.validate()
    }

    fn materialize(&self) -> ScheduledPropulsionFaultRule {
        ScheduledPropulsionFaultRule {
            id: self.id.clone(),
            engine_id: self.engine_id.clone(),
            start_step: self.start_step,
            fault: self.fault.clone(),
        }
    }
}

/// Deterministic library of scheduled propulsion fault templates.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
pub struct PropulsionFaultLibrary {
    templates: Vec<PropulsionFaultTemplate>,
}

impl PropulsionFaultLibrary {
    /// Construct a validated fault library.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] when a template has an invalid
    /// id, probability, payload, or duplicate id.
    pub fn new(templates: Vec<PropulsionFaultTemplate>) -> Result<Self, MonteCarloError> {
        let mut ids = BTreeSet::new();
        for template in &templates {
            template.validate()?;
            if !ids.insert(template.id.as_str()) {
                return Err(MonteCarloError::InvalidInput { field: "fault.id" });
            }
        }
        Ok(Self { templates })
    }

    /// Fault templates in deterministic materialization order.
    #[must_use]
    pub fn templates(&self) -> &[PropulsionFaultTemplate] {
        &self.templates
    }

    /// Materialize the scheduled propulsion faults active for one MC sample.
    ///
    /// A fresh deterministic stream is derived from
    /// `(campaign_seed, sample_index, dimension_id)`. The resulting overlay is
    /// independent of worker count and can be appended to scenario TOML before
    /// normal parsing and validation.
    #[must_use]
    pub fn materialize(
        &self,
        campaign_seed: u64,
        sample_index: u64,
        dimension_id: u32,
    ) -> PropulsionFaultOverlay {
        let mut rng = SampleRng::new(campaign_seed, sample_index, dimension_id);
        let mut rules = Vec::new();
        for template in &self.templates {
            if rng.uniform01() < template.probability {
                rules.push(template.materialize());
            }
        }
        PropulsionFaultOverlay { rules }
    }
}

/// One materialized scheduled propulsion fault rule for a sampled case.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
pub struct ScheduledPropulsionFaultRule {
    /// Stable scheduled rule id.
    pub id: String,
    /// Scenario-text engine id targeted by this rule.
    pub engine_id: String,
    /// Kernel step at which the scheduled rule injects.
    pub start_step: u64,
    /// Engine fault payload.
    pub fault: PropulsionFaultPayload,
}

/// Materialized scheduled propulsion fault overlay for one MC sample.
#[derive(Clone, Debug, Default, PartialEq, Deserialize, Serialize)]
pub struct PropulsionFaultOverlay {
    rules: Vec<ScheduledPropulsionFaultRule>,
}

impl PropulsionFaultOverlay {
    /// Materialized rules in deterministic order.
    #[must_use]
    pub fn rules(&self) -> &[ScheduledPropulsionFaultRule] {
        &self.rules
    }

    /// `true` when no faults activated for this sample.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.rules.is_empty()
    }

    /// Render this overlay as an appendable TOML fragment using the scenario
    /// `[propulsion.faults]` schema.
    #[must_use]
    pub fn to_toml_fragment(&self) -> String {
        if self.rules.is_empty() {
            return String::new();
        }
        let mut out = String::from("[propulsion.faults]\n");
        for rule in &self.rules {
            out.push_str("\n[[propulsion.faults.rules]]\n");
            out.push_str("id = ");
            out.push_str(&toml_string(&rule.id));
            out.push('\n');
            out.push_str("engine_id = ");
            out.push_str(&toml_string(&rule.engine_id));
            out.push('\n');
            out.push_str(&format!("start_step = {}\n", rule.start_step));
            out.push_str("fault = ");
            out.push_str(&rule.fault.to_toml_inline());
            out.push('\n');
        }
        out
    }
}

fn validate_toml_string_field(value: &str, field: &'static str) -> Result<(), MonteCarloError> {
    if value.trim().is_empty() || value.chars().any(char::is_control) {
        return Err(MonteCarloError::InvalidInput { field });
    }
    Ok(())
}

fn toml_string(value: &str) -> String {
    let mut out = String::from("\"");
    for ch in value.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            _ => out.push(ch),
        }
    }
    out.push('"');
    out
}

fn toml_f64(value: f64) -> String {
    let text = value.to_string();
    if text.contains('.') || text.contains('e') || text.contains('E') {
        text
    } else {
        format!("{text}.0")
    }
}

/// Row-major design matrix with points in the unit cube.
///
/// Rows are samples and columns are dispersion dimensions. Values produced by
/// OpenBMP DoE generators lie in `[0, 1)` and are intended to be mapped through
/// each dispersion input's inverse CDF by caller-owned campaign code.
#[derive(Clone, Debug, PartialEq)]
pub struct DesignMatrix {
    rows: u64,
    columns: u32,
    values: Vec<f64>,
}

impl DesignMatrix {
    /// Construct a row-major design matrix after validating dimensions and
    /// entries.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] when the dimensions are zero,
    /// the storage shape does not match, or any entry is outside `[0, 1)`.
    pub fn new(rows: u64, columns: u32, values: Vec<f64>) -> Result<Self, MonteCarloError> {
        let capacity = design_capacity(rows, columns)?;
        if values.len() != capacity {
            return Err(MonteCarloError::InvalidInput {
                field: "design values",
            });
        }
        if values
            .iter()
            .any(|value| !value.is_finite() || !(0.0..1.0).contains(value))
        {
            return Err(MonteCarloError::InvalidInput {
                field: "design values",
            });
        }
        Ok(Self {
            rows,
            columns,
            values,
        })
    }

    /// Number of sample rows.
    #[must_use]
    pub const fn rows(&self) -> u64 {
        self.rows
    }

    /// Number of dispersion columns.
    #[must_use]
    pub const fn columns(&self) -> u32 {
        self.columns
    }

    /// Row-major matrix values.
    #[must_use]
    pub fn values(&self) -> &[f64] {
        &self.values
    }

    /// Return one design value by row and column.
    #[must_use]
    pub fn get(&self, row: u64, column: u32) -> Option<f64> {
        if row >= self.rows || column >= self.columns {
            return None;
        }
        let row = usize::try_from(row).ok()?;
        let column = usize::try_from(column).ok()?;
        let columns = usize::try_from(self.columns).ok()?;
        self.values.get(row.checked_mul(columns)? + column).copied()
    }

    /// Return one column's values in row order.
    #[must_use]
    pub fn column_values(&self, column: u32) -> Option<Vec<f64>> {
        if column >= self.columns {
            return None;
        }
        let rows = usize::try_from(self.rows).ok()?;
        let columns = usize::try_from(self.columns).ok()?;
        let column = usize::try_from(column).ok()?;
        let mut values = Vec::with_capacity(rows);
        for row in 0..rows {
            values.push(*self.values.get(row.checked_mul(columns)? + column)?);
        }
        Some(values)
    }
}

/// McKay-Beckman-Conover Latin Hypercube Sampling generator.
///
/// The generator creates one sample in every marginal `1/N` stratum for each
/// dimension. Column permutations and within-stratum jitters are derived from
/// [`DeterministicRng::for_mc_sample`] through [`SampleRng`], so the design is
/// byte-stable for a fixed `(campaign_seed, sample_count, dimension_count)`.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct LatinHypercube;

impl LatinHypercube {
    /// Generate a Latin hypercube design in `[0, 1)`.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] when the sample count or
    /// dimension count is zero, or when the requested matrix cannot fit in
    /// memory indexing.
    pub fn generate(
        &self,
        campaign_seed: u64,
        sample_count: u64,
        dimension_count: u32,
    ) -> Result<DesignMatrix, MonteCarloError> {
        latin_hypercube(campaign_seed, sample_count, dimension_count)
    }
}

/// Native Sobol low-discrepancy sequence generator.
///
/// Direction numbers are parsed from the provenance-pinned Joe/Kuo
/// `new-joe-kuo-6.21201` data asset under `data/sobol/`.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct SobolSequence;

impl SobolSequence {
    /// Maximum dimension supported by the Joe/Kuo direction-number table.
    pub const MAX_DIMENSIONS: u32 = SOBOL_MAX_DIMENSIONS;

    /// Generate an unscrambled Sobol design in `[0, 1)`.
    ///
    /// Row zero is the all-zero Sobol point. Callers that need to avoid exact
    /// zero for inverse-CDF mapping should skip or clamp row zero at the
    /// dispersion layer.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] when the sample count or
    /// dimension count is zero, the requested dimension exceeds the built-in
    /// direction table, or the requested matrix cannot fit in memory indexing.
    pub fn generate(
        &self,
        sample_count: u64,
        dimension_count: u32,
    ) -> Result<DesignMatrix, MonteCarloError> {
        sobol(sample_count, dimension_count)
    }

    /// Generate a nested-uniform Owen-scrambled Sobol design in `[0, 1)`.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] when the sample count or
    /// dimension count is invalid for [`Self::generate`].
    pub fn generate_owen_scrambled(
        &self,
        sample_count: u64,
        dimension_count: u32,
        scramble_seed: u64,
    ) -> Result<DesignMatrix, MonteCarloError> {
        owen_scrambled_sobol(sample_count, dimension_count, scramble_seed)
    }
}

/// Deterministic Owen-style nested-uniform binary scramble.
///
/// The scramble bit for each Sobol output bit is keyed by
/// `(scramble_seed, dimension, bit_index, prior_scrambled_prefix)`. This keeps
/// independent deterministic randomization at every binary-tree node while
/// preserving the Sobol net shape under a fixed seed.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct OwenScramble {
    seed: u64,
}

impl OwenScramble {
    /// Construct a deterministic scramble from a campaign-local seed.
    #[must_use]
    pub const fn new(seed: u64) -> Self {
        Self { seed }
    }

    /// Scramble seed.
    #[must_use]
    pub const fn seed(&self) -> u64 {
        self.seed
    }

    /// Apply the scramble to an existing Sobol design request.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] when the Sobol design request
    /// is invalid.
    pub fn generate(
        &self,
        sample_count: u64,
        dimension_count: u32,
    ) -> Result<DesignMatrix, MonteCarloError> {
        owen_scrambled_sobol(sample_count, dimension_count, self.seed)
    }
}

/// Generate a deterministic Latin hypercube design in `[0, 1)`.
///
/// # Errors
///
/// Returns [`MonteCarloError::InvalidInput`] when the sample count or
/// dimension count is zero, or when the requested matrix cannot fit in memory
/// indexing.
pub fn latin_hypercube(
    campaign_seed: u64,
    sample_count: u64,
    dimension_count: u32,
) -> Result<DesignMatrix, MonteCarloError> {
    let capacity = design_capacity(sample_count, dimension_count)?;
    let rows = usize::try_from(sample_count).map_err(|_| MonteCarloError::InvalidInput {
        field: "sample_count",
    })?;
    let columns = usize::try_from(dimension_count).map_err(|_| MonteCarloError::InvalidInput {
        field: "dimension_count",
    })?;
    let mut values = vec![0.0; capacity];
    for column in 0..columns {
        let column_u32 = u32::try_from(column).map_err(|_| MonteCarloError::InvalidInput {
            field: "dimension_count",
        })?;
        let dimension_id =
            0x4C48_0000_u32
                .checked_add(column_u32)
                .ok_or(MonteCarloError::InvalidInput {
                    field: "dimension_count",
                })?;
        let mut rng = SampleRng::new(campaign_seed, 0, dimension_id);
        let mut strata: Vec<u64> = (0..sample_count).collect();
        for index in (1..rows).rev() {
            let swap = bounded_index(&mut rng, (index + 1) as u64)? as usize;
            strata.swap(index, swap);
        }
        for (row, stratum) in strata.into_iter().enumerate() {
            let jitter = rng.uniform01();
            let value = (stratum as f64 + jitter) / sample_count as f64;
            values[row * columns + column] = value;
        }
    }
    DesignMatrix::new(sample_count, dimension_count, values)
}

/// Generate an unscrambled native Sobol design in `[0, 1)`.
///
/// # Errors
///
/// Returns [`MonteCarloError::InvalidInput`] when the sample count or dimension
/// count is zero, the requested dimension exceeds the Joe/Kuo direction-number
/// asset, or the requested matrix cannot fit in memory indexing.
pub fn sobol(sample_count: u64, dimension_count: u32) -> Result<DesignMatrix, MonteCarloError> {
    let columns = usize::try_from(dimension_count).map_err(|_| MonteCarloError::InvalidInput {
        field: "dimension_count",
    })?;
    let integers = sobol_integer_matrix(sample_count, dimension_count)?;
    DesignMatrix::new(
        sample_count,
        dimension_count,
        integers
            .into_iter()
            .map(unit_from_u64)
            .take(design_capacity(sample_count, dimension_count)?)
            .collect::<Vec<_>>(),
    )
    .and_then(|design| {
        if usize::try_from(design.columns()).ok() == Some(columns) {
            Ok(design)
        } else {
            Err(MonteCarloError::InvalidInput {
                field: "dimension_count",
            })
        }
    })
}

/// Generate a deterministic Owen-scrambled Sobol design in `[0, 1)`.
///
/// # Errors
///
/// Returns [`MonteCarloError::InvalidInput`] when the sample count or dimension
/// count is invalid for [`sobol`].
pub fn owen_scrambled_sobol(
    sample_count: u64,
    dimension_count: u32,
    scramble_seed: u64,
) -> Result<DesignMatrix, MonteCarloError> {
    let columns = usize::try_from(dimension_count).map_err(|_| MonteCarloError::InvalidInput {
        field: "dimension_count",
    })?;
    let integers = sobol_integer_matrix(sample_count, dimension_count)?;
    let mut values = Vec::with_capacity(integers.len());
    for (index, value) in integers.into_iter().enumerate() {
        let dimension =
            u32::try_from(index % columns).map_err(|_| MonteCarloError::InvalidInput {
                field: "dimension_count",
            })?;
        values.push(unit_from_u64(owen_scramble_u64(
            value,
            scramble_seed,
            dimension,
        )));
    }
    DesignMatrix::new(sample_count, dimension_count, values)
}

fn sobol_integer_matrix(
    sample_count: u64,
    dimension_count: u32,
) -> Result<Vec<u64>, MonteCarloError> {
    let capacity = design_capacity(sample_count, dimension_count)?;
    if dimension_count > SOBOL_MAX_DIMENSIONS {
        return Err(MonteCarloError::InvalidInput {
            field: "dimension_count",
        });
    }
    let rows = usize::try_from(sample_count).map_err(|_| MonteCarloError::InvalidInput {
        field: "sample_count",
    })?;
    let columns = usize::try_from(dimension_count).map_err(|_| MonteCarloError::InvalidInput {
        field: "dimension_count",
    })?;
    let mut values = vec![0_u64; capacity];
    let directions = sobol_directions(dimension_count)?;
    let mut state = vec![0_u64; columns];
    for row in 0..rows {
        if row > 0 {
            let bit = row.trailing_zeros() as usize;
            for column in 0..columns {
                state[column] ^= directions[column][bit];
            }
        }
        for column in 0..columns {
            values[row * columns + column] = state[column];
        }
    }
    Ok(values)
}

fn unit_from_u64(value: u64) -> f64 {
    value as f64 / (u64::MAX as f64 + 1.0)
}

fn owen_scramble_u64(value: u64, seed: u64, dimension: u32) -> u64 {
    let mut out = 0_u64;
    let mut prefix = 0_u64;
    for bit in 0..SOBOL_BITS {
        let input_bit = (value >> (SOBOL_BITS - 1 - bit)) & 1;
        let flip = owen_node_bit(seed, dimension, bit as u32, prefix);
        let scrambled = input_bit ^ flip;
        out |= scrambled << (SOBOL_BITS - 1 - bit);
        prefix = (prefix << 1) | scrambled;
    }
    out
}

fn owen_node_bit(seed: u64, dimension: u32, bit: u32, prefix: u64) -> u64 {
    let mut state = seed ^ 0xA511_E9B3_DD23_4B91;
    state ^= u64::from(dimension).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    state ^= u64::from(bit).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    state ^= prefix
        .rotate_left(bit % 64)
        .wrapping_mul(0x94D0_49BB_1331_11EB);
    splitmix64(state) & 1
}

fn splitmix64(mut state: u64) -> u64 {
    state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

const SOBOL_BITS: usize = 64;
const SOBOL_MAX_DIMENSIONS: u32 = 21_201;
const JOE_KUO_DIRECTION_NUMBERS: &str = include_str!("../../../data/sobol/new-joe-kuo-6.21201");

#[derive(Clone, Debug)]
struct SobolDirectionSpec {
    degree: usize,
    coefficients: u32,
    initial: Vec<u64>,
}

fn sobol_directions(dimension_count: u32) -> Result<Vec<[u64; SOBOL_BITS]>, MonteCarloError> {
    if dimension_count == 0 || dimension_count > SOBOL_MAX_DIMENSIONS {
        return Err(MonteCarloError::InvalidInput {
            field: "dimension_count",
        });
    }
    let mut directions = Vec::with_capacity(usize::try_from(dimension_count).map_err(|_| {
        MonteCarloError::InvalidInput {
            field: "dimension_count",
        }
    })?);
    let mut first = [0_u64; SOBOL_BITS];
    for (bit, value) in first.iter_mut().enumerate() {
        *value = 1_u64 << (SOBOL_BITS - 1 - bit);
    }
    directions.push(first);

    let mut lines = JOE_KUO_DIRECTION_NUMBERS.lines();
    let header = lines.next().ok_or(MonteCarloError::InvalidInput {
        field: "sobol directions",
    })?;
    if !header.split_whitespace().take(3).eq(["d", "s", "a"]) {
        return Err(MonteCarloError::InvalidInput {
            field: "sobol directions",
        });
    }

    for expected_dimension in 2..=dimension_count {
        let line = lines.next().ok_or(MonteCarloError::InvalidInput {
            field: "sobol directions",
        })?;
        let spec = parse_sobol_direction_spec(line, expected_dimension)?;
        let mut direction = [0_u64; SOBOL_BITS];
        for (bit, initial) in spec.initial.iter().enumerate() {
            direction[bit] = initial << (SOBOL_BITS - 1 - bit);
        }
        for bit in spec.degree..SOBOL_BITS {
            let mut value =
                direction[bit - spec.degree] ^ (direction[bit - spec.degree] >> spec.degree);
            for offset in 1..spec.degree {
                let coefficient_bit = (spec.coefficients >> (spec.degree - 1 - offset)) & 1;
                if coefficient_bit == 1 {
                    value ^= direction[bit - offset];
                }
            }
            direction[bit] = value;
        }
        directions.push(direction);
    }
    Ok(directions)
}

fn parse_sobol_direction_spec(
    line: &str,
    expected_dimension: u32,
) -> Result<SobolDirectionSpec, MonteCarloError> {
    let mut fields = line.split_whitespace();
    let dimension = parse_sobol_u32(fields.next(), "sobol dimension")?;
    if dimension != expected_dimension {
        return Err(MonteCarloError::InvalidInput {
            field: "sobol dimension",
        });
    }
    let degree = parse_sobol_usize(fields.next(), "sobol degree")?;
    if degree == 0 || degree >= SOBOL_BITS {
        return Err(MonteCarloError::InvalidInput {
            field: "sobol degree",
        });
    }
    let coefficients = parse_sobol_u32(fields.next(), "sobol coefficients")?;
    let mut initial = Vec::with_capacity(degree);
    for _ in 0..degree {
        initial.push(u64::from(parse_sobol_u32(fields.next(), "sobol initial")?));
    }
    if fields.next().is_some() {
        return Err(MonteCarloError::InvalidInput {
            field: "sobol directions",
        });
    }
    Ok(SobolDirectionSpec {
        degree,
        coefficients,
        initial,
    })
}

fn parse_sobol_u32(value: Option<&str>, field: &'static str) -> Result<u32, MonteCarloError> {
    value
        .ok_or(MonteCarloError::InvalidInput { field })?
        .parse::<u32>()
        .map_err(|_| MonteCarloError::InvalidInput { field })
}

fn parse_sobol_usize(value: Option<&str>, field: &'static str) -> Result<usize, MonteCarloError> {
    usize::try_from(parse_sobol_u32(value, field)?)
        .map_err(|_| MonteCarloError::InvalidInput { field })
}

/// Square target correlation matrix for Iman-Conover rank induction.
///
/// The matrix must be finite, symmetric, have unit diagonal, contain
/// off-diagonal coefficients in `[-1, 1]`, and be positive definite.
#[derive(Clone, Debug, PartialEq)]
pub struct CorrelationMatrix {
    size: u32,
    values: Vec<f64>,
}

impl CorrelationMatrix {
    /// Construct and validate a row-major correlation matrix.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] when the shape is invalid,
    /// entries are non-finite or outside the correlation envelope, or the
    /// matrix is not positive definite.
    pub fn new(size: u32, values: Vec<f64>) -> Result<Self, MonteCarloError> {
        if size == 0 {
            return Err(MonteCarloError::InvalidInput {
                field: "correlation size",
            });
        }
        let size_usize = usize::try_from(size).map_err(|_| MonteCarloError::InvalidInput {
            field: "correlation size",
        })?;
        if values.len()
            != size_usize
                .checked_mul(size_usize)
                .ok_or(MonteCarloError::InvalidInput {
                    field: "correlation values",
                })?
        {
            return Err(MonteCarloError::InvalidInput {
                field: "correlation values",
            });
        }
        for row in 0..size_usize {
            for column in 0..size_usize {
                let value = values[row * size_usize + column];
                if !value.is_finite() || !(-1.0..=1.0).contains(&value) {
                    return Err(MonteCarloError::InvalidInput {
                        field: "correlation values",
                    });
                }
                if row == column && (value - 1.0).abs() > 1.0e-12 {
                    return Err(MonteCarloError::InvalidInput {
                        field: "correlation diagonal",
                    });
                }
                let transpose = values[column * size_usize + row];
                if (value - transpose).abs() > 1.0e-12 {
                    return Err(MonteCarloError::InvalidInput {
                        field: "correlation symmetry",
                    });
                }
            }
        }
        let matrix = DMatrix::from_row_slice(size_usize, size_usize, &values);
        if matrix.cholesky().is_none() {
            return Err(MonteCarloError::InvalidInput {
                field: "correlation positive definite",
            });
        }
        Ok(Self { size, values })
    }

    /// Identity correlation matrix of a given size.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] when `size == 0`.
    pub fn identity(size: u32) -> Result<Self, MonteCarloError> {
        let size_usize = usize::try_from(size).map_err(|_| MonteCarloError::InvalidInput {
            field: "correlation size",
        })?;
        let mut values = vec![
            0.0;
            size_usize.checked_mul(size_usize).ok_or(
                MonteCarloError::InvalidInput {
                    field: "correlation values",
                }
            )?
        ];
        for index in 0..size_usize {
            values[index * size_usize + index] = 1.0;
        }
        Self::new(size, values)
    }

    /// Matrix dimension.
    #[must_use]
    pub const fn size(&self) -> u32 {
        self.size
    }

    /// Row-major matrix values.
    #[must_use]
    pub fn values(&self) -> &[f64] {
        &self.values
    }

    /// Return one matrix entry.
    #[must_use]
    pub fn get(&self, row: u32, column: u32) -> Option<f64> {
        if row >= self.size || column >= self.size {
            return None;
        }
        let row = usize::try_from(row).ok()?;
        let column = usize::try_from(column).ok()?;
        let size = usize::try_from(self.size).ok()?;
        self.values.get(row.checked_mul(size)? + column).copied()
    }

    fn cholesky_lower(&self) -> Result<DMatrix<f64>, MonteCarloError> {
        let size = usize::try_from(self.size).map_err(|_| MonteCarloError::InvalidInput {
            field: "correlation size",
        })?;
        DMatrix::from_row_slice(size, size, &self.values)
            .cholesky()
            .map(|factor| factor.l())
            .ok_or(MonteCarloError::InvalidInput {
                field: "correlation positive definite",
            })
    }
}

/// Iman-Conover rank-correlation induction.
///
/// Applying this transform reorders each column of an existing design matrix
/// according to correlated van-der-Waerden scores. Column marginal values are
/// preserved exactly; only cross-column row pairing changes.
#[derive(Clone, Debug, PartialEq)]
pub struct ImanConover {
    target: CorrelationMatrix,
}

impl ImanConover {
    /// Construct an Iman-Conover transform for a target correlation matrix.
    #[must_use]
    pub const fn new(target: CorrelationMatrix) -> Self {
        Self { target }
    }

    /// Target correlation matrix.
    #[must_use]
    pub const fn target(&self) -> &CorrelationMatrix {
        &self.target
    }

    /// Apply the transform to a design matrix.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] when the target dimension does
    /// not match the design, the design has fewer than two rows, or the normal
    /// scores cannot be generated.
    pub fn apply(&self, design: &DesignMatrix) -> Result<DesignMatrix, MonteCarloError> {
        iman_conover(design, &self.target)
    }
}

/// Apply Iman-Conover rank-correlation induction to a design matrix.
///
/// # Errors
///
/// Returns [`MonteCarloError::InvalidInput`] when the target dimension does not
/// match the design, the design has fewer than two rows, or the normal scores
/// cannot be generated.
pub fn iman_conover(
    design: &DesignMatrix,
    target: &CorrelationMatrix,
) -> Result<DesignMatrix, MonteCarloError> {
    if target.size() != design.columns() {
        return Err(MonteCarloError::InvalidInput {
            field: "correlation size",
        });
    }
    if design.rows() < 2 {
        return Err(MonteCarloError::InvalidInput {
            field: "sample_count",
        });
    }

    let rows = usize::try_from(design.rows()).map_err(|_| MonteCarloError::InvalidInput {
        field: "sample_count",
    })?;
    let columns = usize::try_from(design.columns()).map_err(|_| MonteCarloError::InvalidInput {
        field: "dimension_count",
    })?;
    let normal_scores = van_der_waerden_scores(rows)?;
    let mut source_scores = vec![0.0; rows * columns];
    for column in 0..columns {
        let values = design
            .column_values(column as u32)
            .ok_or(MonteCarloError::InvalidInput {
                field: "dimension_count",
            })?;
        let order = sorted_indices(&values);
        for (rank, row) in order.into_iter().enumerate() {
            source_scores[row * columns + column] = normal_scores[rank];
        }
    }

    let source = DMatrix::from_row_slice(rows, columns, &source_scores);
    let lower = target.cholesky_lower()?;
    let correlated = source * lower.transpose();
    let mut out = vec![0.0; rows * columns];
    for column in 0..columns {
        let original =
            design
                .column_values(column as u32)
                .ok_or(MonteCarloError::InvalidInput {
                    field: "dimension_count",
                })?;
        let mut sorted_original = original;
        sorted_original.sort_by(total_f64_order);
        let correlated_column: Vec<f64> = (0..rows).map(|row| correlated[(row, column)]).collect();
        let target_order = sorted_indices(&correlated_column);
        for (rank, row) in target_order.into_iter().enumerate() {
            out[row * columns + column] = sorted_original[rank];
        }
    }
    DesignMatrix::new(design.rows(), design.columns(), out)
}

fn van_der_waerden_scores(rows: usize) -> Result<Vec<f64>, MonteCarloError> {
    let normal = Normal::new(0.0, 1.0).map_err(|_| MonteCarloError::InvalidInput {
        field: "normal score",
    })?;
    let denominator = rows.checked_add(1).ok_or(MonteCarloError::InvalidInput {
        field: "sample_count",
    })? as f64;
    let mut scores = Vec::with_capacity(rows);
    for rank in 0..rows {
        let probability = (rank + 1) as f64 / denominator;
        let score = normal.inverse_cdf(probability);
        if !score.is_finite() {
            return Err(MonteCarloError::InvalidInput {
                field: "normal score",
            });
        }
        scores.push(score);
    }
    Ok(scores)
}

fn sorted_indices(values: &[f64]) -> Vec<usize> {
    let mut indices: Vec<usize> = (0..values.len()).collect();
    indices.sort_by(|left, right| {
        total_f64_order(&values[*left], &values[*right]).then_with(|| left.cmp(right))
    });
    indices
}

fn total_f64_order(left: &f64, right: &f64) -> std::cmp::Ordering {
    left.total_cmp(right)
}

fn design_capacity(rows: u64, columns: u32) -> Result<usize, MonteCarloError> {
    if rows == 0 {
        return Err(MonteCarloError::InvalidInput {
            field: "sample_count",
        });
    }
    if columns == 0 {
        return Err(MonteCarloError::InvalidInput {
            field: "dimension_count",
        });
    }
    let rows = usize::try_from(rows).map_err(|_| MonteCarloError::InvalidInput {
        field: "sample_count",
    })?;
    let columns = usize::try_from(columns).map_err(|_| MonteCarloError::InvalidInput {
        field: "dimension_count",
    })?;
    rows.checked_mul(columns)
        .ok_or(MonteCarloError::InvalidInput {
            field: "design values",
        })
}

fn bounded_index(rng: &mut SampleRng, upper_exclusive: u64) -> Result<u64, MonteCarloError> {
    if upper_exclusive == 0 {
        return Err(MonteCarloError::InvalidInput {
            field: "upper_exclusive",
        });
    }
    let upper = u128::from(upper_exclusive);
    let zone = ((u128::from(u64::MAX) + 1) / upper) * upper;
    loop {
        let value = u128::from(rng.next_u64());
        if value < zone {
            return Ok((value % upper) as u64);
        }
    }
}

/// Streaming Welford state `(n, mean, M2)`.
///
/// `M2` is the running sum of squared deviations from the mean. Use
/// [`Self::sample_variance`] for the unbiased estimator and
/// [`Self::population_variance`] for the population estimator.
#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct Welford {
    n: u64,
    mean: f64,
    m2: f64,
}

impl Welford {
    /// Empty Welford state.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            n: 0,
            mean: 0.0,
            m2: 0.0,
        }
    }

    /// Number of samples consumed.
    #[must_use]
    pub const fn count(&self) -> u64 {
        self.n
    }

    /// Running mean. Returns `0.0` for an empty state.
    #[must_use]
    pub const fn mean(&self) -> f64 {
        self.mean
    }

    /// Running `M2` sum of squared deviations from the mean.
    #[must_use]
    pub const fn m2(&self) -> f64 {
        self.m2
    }

    /// Add one finite sample.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] when `value` is not finite.
    pub fn update(&mut self, value: f64) -> Result<(), MonteCarloError> {
        if !value.is_finite() {
            return Err(MonteCarloError::InvalidInput { field: "value" });
        }
        let next_n = self.n.checked_add(1).ok_or(MonteCarloError::InvalidInput {
            field: "sample count",
        })?;
        let delta = value - self.mean;
        let next_mean = self.mean + delta / next_n as f64;
        let delta2 = value - next_mean;
        let next_m2 = self.m2 + delta * delta2;
        if !next_mean.is_finite() || !next_m2.is_finite() {
            return Err(MonteCarloError::InvalidInput { field: "value" });
        }
        self.n = next_n;
        self.mean = next_mean;
        self.m2 = next_m2;
        Ok(())
    }

    /// Merge two Welford states using the Chan-Golub-LeVeque
    /// parallel-variance formula.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] if the merged state
    /// would overflow or become non-finite.
    pub fn merge(self, other: Self) -> Result<Self, MonteCarloError> {
        if self.n == 0 {
            return Ok(other);
        }
        if other.n == 0 {
            return Ok(self);
        }
        let n = self
            .n
            .checked_add(other.n)
            .ok_or(MonteCarloError::InvalidInput {
                field: "sample count",
            })?;
        let delta = other.mean - self.mean;
        let mean = self.mean + delta * (other.n as f64 / n as f64);
        let m2 = self.m2 + other.m2 + delta * delta * (self.n as f64 * other.n as f64 / n as f64);
        if !mean.is_finite() || !m2.is_finite() {
            return Err(MonteCarloError::InvalidInput { field: "value" });
        }
        Ok(Self { n, mean, m2 })
    }

    /// Build a Welford state by sequentially updating from finite
    /// values in the provided order.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] when any value is
    /// non-finite.
    pub fn from_values(values: impl IntoIterator<Item = f64>) -> Result<Self, MonteCarloError> {
        let mut state = Self::new();
        for value in values {
            state.update(value)?;
        }
        Ok(state)
    }

    /// Build an order-independent Welford state from `(sample_index,
    /// value)` pairs.
    ///
    /// Samples are sorted by `sample_index`, duplicate indices are
    /// rejected, and the final state is reduced through a fixed
    /// index-keyed merge tree. This makes the aggregate independent of
    /// input iteration order and ready for parallel campaign fan-out.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] for non-finite values
    /// and [`MonteCarloError::DuplicateSampleIndex`] for duplicate
    /// indices.
    pub fn from_indexed_samples(
        samples: impl IntoIterator<Item = (u64, f64)>,
    ) -> Result<Self, MonteCarloError> {
        let mut samples: Vec<(u64, f64)> = samples.into_iter().collect();
        samples.sort_by_key(|(index, _)| *index);
        for window in samples.windows(2) {
            if window[0].0 == window[1].0 {
                return Err(MonteCarloError::DuplicateSampleIndex { index: window[0].0 });
            }
        }
        let mut leaves = Vec::with_capacity(samples.len());
        for (_, value) in samples {
            let mut leaf = Self::new();
            leaf.update(value)?;
            leaves.push(leaf);
        }
        fixed_merge_tree(leaves)
    }

    /// Unbiased sample variance. Returns `None` until at least two
    /// samples have been consumed.
    #[must_use]
    pub fn sample_variance(&self) -> Option<f64> {
        (self.n >= 2).then(|| self.m2 / (self.n - 1) as f64)
    }

    /// Monte Carlo standard error of the mean. Returns `None` until
    /// at least two samples have been consumed.
    #[must_use]
    pub fn standard_error(&self) -> Option<f64> {
        self.sample_variance()
            .map(|variance| (variance / self.n as f64).sqrt())
    }

    /// Population variance. Returns `None` for an empty state.
    #[must_use]
    pub fn population_variance(&self) -> Option<f64> {
        (self.n > 0).then(|| self.m2 / self.n as f64)
    }
}

fn fixed_merge_tree(mut states: Vec<Welford>) -> Result<Welford, MonteCarloError> {
    if states.is_empty() {
        return Ok(Welford::new());
    }
    while states.len() > 1 {
        let mut next = Vec::with_capacity(states.len().div_ceil(2));
        let mut chunks = states.chunks_exact(2);
        for pair in &mut chunks {
            next.push(pair[0].merge(pair[1])?);
        }
        if let [tail] = chunks.remainder() {
            next.push(*tail);
        }
        states = next;
    }
    Ok(states[0])
}

/// Bernoulli success-count statistics.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct BernoulliStats {
    trials: u64,
    successes: u64,
}

impl BernoulliStats {
    /// Construct a Bernoulli count state.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] when `successes > trials`.
    pub fn new(trials: u64, successes: u64) -> Result<Self, MonteCarloError> {
        if successes > trials {
            return Err(MonteCarloError::InvalidInput { field: "successes" });
        }
        Ok(Self { trials, successes })
    }

    /// Total number of Bernoulli trials.
    #[must_use]
    pub const fn trials(&self) -> u64 {
        self.trials
    }

    /// Number of successful Bernoulli trials.
    #[must_use]
    pub const fn successes(&self) -> u64 {
        self.successes
    }

    /// Observed success fraction. Returns `None` when `trials == 0`.
    #[must_use]
    pub fn fraction(&self) -> Option<f64> {
        (self.trials > 0).then(|| self.successes as f64 / self.trials as f64)
    }

    /// Two-sided exact Clopper-Pearson interval for the success probability.
    ///
    /// `confidence` must lie in `(0, 1)`. Edge cases are handled by
    /// the exact limits: zero successes has lower bound `0`, and all
    /// successes has upper bound `1`.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] for invalid confidence
    /// or zero trials, and [`MonteCarloError::InvalidBinomialInterval`]
    /// if the beta distribution rejects its parameters.
    pub fn clopper_pearson(
        &self,
        confidence: f64,
    ) -> Result<ClopperPearsonInterval, MonteCarloError> {
        validate_probability("confidence", confidence)?;
        if self.trials == 0 {
            return Err(MonteCarloError::InvalidInput { field: "trials" });
        }
        let alpha = 1.0 - confidence;
        let k = self.successes;
        let n = self.trials;
        let lower = if k == 0 {
            0.0
        } else {
            beta_inverse(k as f64, (n - k + 1) as f64, 0.5 * alpha, n, k)?
        };
        let upper = if k == n {
            1.0
        } else {
            beta_inverse((k + 1) as f64, (n - k) as f64, 1.0 - 0.5 * alpha, n, k)?
        };
        Ok(ClopperPearsonInterval {
            confidence,
            lower,
            upper,
        })
    }
}

fn beta_inverse(
    alpha: f64,
    beta: f64,
    probability: f64,
    trials: u64,
    successes: u64,
) -> Result<f64, MonteCarloError> {
    let distribution = Beta::new(alpha, beta)
        .map_err(|_| MonteCarloError::InvalidBinomialInterval { trials, successes })?;
    let value = distribution.inverse_cdf(probability);
    if value.is_finite() {
        Ok(value)
    } else {
        Err(MonteCarloError::InvalidBinomialInterval { trials, successes })
    }
}

/// Exact two-sided Clopper-Pearson confidence interval.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClopperPearsonInterval {
    /// Requested confidence level.
    pub confidence: f64,
    /// Lower success-probability bound.
    pub lower: f64,
    /// Upper success-probability bound.
    pub upper: f64,
}

fn validate_probability(field: &'static str, value: f64) -> Result<(), MonteCarloError> {
    if value.is_finite() && value > 0.0 && value < 1.0 {
        Ok(())
    } else {
        Err(MonteCarloError::InvalidInput { field })
    }
}

/// Wilks one-sided tolerance-bound sample size.
///
/// Returns the smallest `N` such that the maximum or minimum order statistic
/// covers at least `coverage` of the population with the requested
/// `confidence`.
///
/// # Errors
///
/// Returns [`MonteCarloError::InvalidInput`] when `coverage` or `confidence`
/// is not finite or does not lie in `(0, 1)`.
pub fn wilks_one_sided_n(coverage: f64, confidence: f64) -> Result<u64, MonteCarloError> {
    validate_probability("coverage", coverage)?;
    validate_probability("confidence", confidence)?;
    let estimate = (1.0 - confidence).ln() / coverage.ln();
    if !estimate.is_finite() || estimate > u64::MAX as f64 {
        return Err(MonteCarloError::InvalidInput {
            field: "sample_count",
        });
    }
    let mut n = estimate.ceil().max(1.0) as u64;
    while wilks_one_sided_confidence(coverage, n)? < confidence {
        n = n.checked_add(1).ok_or(MonteCarloError::InvalidInput {
            field: "sample_count",
        })?;
    }
    while n > 1 && wilks_one_sided_confidence(coverage, n - 1)? >= confidence {
        n -= 1;
    }
    Ok(n)
}

/// Wilks two-sided tolerance-interval sample size.
///
/// Returns the smallest `N` satisfying
/// `1 - coverage^N - N * (1 - coverage) * coverage^(N - 1) >= confidence`
/// for the sample min/max interval.
///
/// # Errors
///
/// Returns [`MonteCarloError::InvalidInput`] when `coverage` or `confidence`
/// is not finite or does not lie in `(0, 1)`.
pub fn wilks_two_sided_n(coverage: f64, confidence: f64) -> Result<u64, MonteCarloError> {
    validate_probability("coverage", coverage)?;
    validate_probability("confidence", confidence)?;
    let mut high = 2_u64;
    while wilks_two_sided_confidence(coverage, high)? < confidence {
        high = high.checked_mul(2).ok_or(MonteCarloError::InvalidInput {
            field: "sample_count",
        })?;
    }
    let mut low = 1_u64;
    while low + 1 < high {
        let mid = low + (high - low) / 2;
        if wilks_two_sided_confidence(coverage, mid)? >= confidence {
            high = mid;
        } else {
            low = mid;
        }
    }
    Ok(high)
}

/// Confidence attained by a one-sided first-order Wilks bound.
///
/// # Errors
///
/// Returns [`MonteCarloError::InvalidInput`] when `coverage` is invalid or
/// `sample_count == 0`.
pub fn wilks_one_sided_confidence(
    coverage: f64,
    sample_count: u64,
) -> Result<f64, MonteCarloError> {
    validate_probability("coverage", coverage)?;
    if sample_count == 0 {
        return Err(MonteCarloError::InvalidInput {
            field: "sample_count",
        });
    }
    Ok(1.0 - coverage.powf(sample_count as f64))
}

/// Confidence attained by a two-sided min/max Wilks interval.
///
/// # Errors
///
/// Returns [`MonteCarloError::InvalidInput`] when `coverage` is invalid or
/// `sample_count == 0`.
pub fn wilks_two_sided_confidence(
    coverage: f64,
    sample_count: u64,
) -> Result<f64, MonteCarloError> {
    validate_probability("coverage", coverage)?;
    if sample_count == 0 {
        return Err(MonteCarloError::InvalidInput {
            field: "sample_count",
        });
    }
    if sample_count == 1 {
        return Ok(0.0);
    }
    let n = sample_count as f64;
    let missed = coverage.powf(n - 1.0) * (coverage + n * (1.0 - coverage));
    Ok((1.0 - missed).clamp(0.0, 1.0))
}

/// Campaign stop state for convergence-gated Monte Carlo runs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CampaignStop {
    /// Continue running; no stop condition is satisfied yet.
    Continue,
    /// The convergence gate held for its configured window.
    Converged {
        /// Sample index at the end of the first converged window.
        at: u64,
    },
    /// The campaign reached its requested maximum sample count.
    ReachedN,
    /// The campaign is still below the minimum Wilks sample floor.
    WilksFloor,
}

/// Relative half-width convergence gate for a scalar campaign metric.
///
/// The gate uses a normal-approximation confidence interval around the Welford
/// mean. It converges only when the relative half-width is at or below
/// `rel_half_width_tol` for `window` consecutive trace rows.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConvergenceGate {
    rel_half_width_tol: f64,
    window: u64,
    confidence: f64,
}

impl ConvergenceGate {
    /// Construct a convergence gate.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] when `rel_half_width_tol`
    /// is not finite and positive, when `window == 0`, or when `confidence`
    /// does not lie in `(0, 1)`.
    pub fn new(
        rel_half_width_tol: f64,
        window: u64,
        confidence: f64,
    ) -> Result<Self, MonteCarloError> {
        if !rel_half_width_tol.is_finite() || rel_half_width_tol <= 0.0 {
            return Err(MonteCarloError::InvalidInput {
                field: "rel_half_width_tol",
            });
        }
        if window == 0 {
            return Err(MonteCarloError::InvalidInput { field: "window" });
        }
        validate_probability("confidence", confidence)?;
        Ok(Self {
            rel_half_width_tol,
            window,
            confidence,
        })
    }

    /// Relative half-width tolerance.
    #[must_use]
    pub const fn rel_half_width_tol(&self) -> f64 {
        self.rel_half_width_tol
    }

    /// Consecutive trace rows required for convergence.
    #[must_use]
    pub const fn window(&self) -> u64 {
        self.window
    }

    /// Confidence level used for the normal-approximation half-width.
    #[must_use]
    pub const fn confidence(&self) -> f64 {
        self.confidence
    }

    /// Return the relative CI half-width for one trace row.
    ///
    /// Returns `Ok(None)` until the trace row has a standard error or when
    /// the row mean is zero and the absolute half-width is non-zero.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] if the configured confidence
    /// cannot be converted to a normal quantile.
    pub fn relative_half_width(
        &self,
        row: &WelfordTraceRow,
    ) -> Result<Option<f64>, MonteCarloError> {
        let Some(standard_error) = row.standard_error else {
            return Ok(None);
        };
        let half_width = self.z_score()? * standard_error;
        if row.mean == 0.0 {
            return Ok((half_width == 0.0).then_some(0.0));
        }
        Ok(Some(half_width / row.mean.abs()))
    }

    /// Return the first sample index where the convergence window is satisfied.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] if `window` does not fit in
    /// memory indexing or the configured confidence cannot be converted to a
    /// normal quantile.
    pub fn first_converged_at(
        &self,
        trace: &[WelfordTraceRow],
    ) -> Result<Option<u64>, MonteCarloError> {
        let window = usize::try_from(self.window)
            .map_err(|_| MonteCarloError::InvalidInput { field: "window" })?;
        if trace.len() < window {
            return Ok(None);
        }
        for rows in trace.windows(window) {
            let mut converged = true;
            for row in rows {
                let Some(relative) = self.relative_half_width(row)? else {
                    converged = false;
                    break;
                };
                if relative > self.rel_half_width_tol {
                    converged = false;
                    break;
                }
            }
            if converged {
                return Ok(Some(rows[window - 1].sample_index));
            }
        }
        Ok(None)
    }

    /// Classify the current campaign stop state.
    ///
    /// `wilks_floor` prevents convergence from being declared before the
    /// distribution-free tolerance-interval sample floor is reached, while
    /// `max_samples` identifies an exhausted campaign budget.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] when `max_samples == 0`,
    /// `wilks_floor == 0`, the trace is malformed, or the configured
    /// confidence cannot be converted to a normal quantile.
    pub fn stop_state(
        &self,
        trace: &[WelfordTraceRow],
        max_samples: u64,
        wilks_floor: u64,
    ) -> Result<CampaignStop, MonteCarloError> {
        if max_samples == 0 {
            return Err(MonteCarloError::InvalidInput {
                field: "max_samples",
            });
        }
        if wilks_floor == 0 {
            return Err(MonteCarloError::InvalidInput {
                field: "wilks_floor",
            });
        }
        let Some(last) = trace.last() else {
            return Ok(CampaignStop::Continue);
        };
        if last.count < wilks_floor {
            return Ok(CampaignStop::WilksFloor);
        }
        if let Some(at) = self.first_converged_at(trace)? {
            return Ok(CampaignStop::Converged { at });
        }
        if last.count >= max_samples {
            Ok(CampaignStop::ReachedN)
        } else {
            Ok(CampaignStop::Continue)
        }
    }

    fn z_score(&self) -> Result<f64, MonteCarloError> {
        let normal = Normal::new(0.0, 1.0).map_err(|_| MonteCarloError::InvalidInput {
            field: "confidence",
        })?;
        let quantile = 0.5 + 0.5 * self.confidence;
        let z = normal.inverse_cdf(quantile);
        if z.is_finite() {
            Ok(z)
        } else {
            Err(MonteCarloError::InvalidInput {
                field: "confidence",
            })
        }
    }
}

/// Scalar outcome produced by one Monte Carlo sample.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScalarOutcome {
    /// Scalar quantity of interest for continuous statistics.
    pub value: f64,
    /// Binary success/failure outcome for Bernoulli statistics.
    pub success: bool,
}

impl ScalarOutcome {
    /// Construct a finite scalar outcome.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] when `value` is not finite.
    pub fn new(value: f64, success: bool) -> Result<Self, MonteCarloError> {
        if !value.is_finite() {
            return Err(MonteCarloError::InvalidInput { field: "value" });
        }
        Ok(Self { value, success })
    }
}

/// Indexed scalar sample captured in a campaign report.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScalarSample {
    /// Deterministic sample index.
    pub sample_index: u64,
    /// Scalar quantity of interest.
    pub value: f64,
    /// Binary success/failure outcome.
    pub success: bool,
}

/// One row of a Welford convergence trace.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WelfordTraceRow {
    /// Sample index just consumed.
    pub sample_index: u64,
    /// Number of samples included in this trace row.
    pub count: u64,
    /// Running mean after this sample.
    pub mean: f64,
    /// Running unbiased sample variance after this sample.
    pub sample_variance: Option<f64>,
    /// Running standard error of the mean after this sample.
    pub standard_error: Option<f64>,
}

/// Deterministic scalar campaign report.
#[derive(Clone, Debug, PartialEq)]
pub struct ScalarCampaignReport {
    /// Per-sample outcomes in sample-index order.
    pub samples: Vec<ScalarSample>,
    /// Final Welford statistics for `samples[*].value`.
    pub value_stats: Welford,
    /// Final Bernoulli success-count statistics.
    pub success_stats: BernoulliStats,
    /// Welford convergence trace in sample-index order.
    pub convergence_trace: Vec<WelfordTraceRow>,
}

impl ScalarCampaignReport {
    /// Exact Clopper-Pearson interval for the report's success fraction.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError`] when `confidence` is invalid.
    pub fn success_interval(
        &self,
        confidence: f64,
    ) -> Result<ClopperPearsonInterval, MonteCarloError> {
        self.success_stats.clopper_pearson(confidence)
    }
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
struct ScalarCheckpointSubtree {
    start_index: u64,
    end_index: u64,
    value_stats: Welford,
    successes: u64,
}

/// Serializable scalar campaign checkpoint.
///
/// The checkpoint records campaign identity, the completed-index set, and
/// reducer subtrees. Schema version 1 stores one reducer leaf per completed
/// sample; finalization reconstructs the report through the same fixed
/// index-keyed merge tree used by normal campaign execution.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct ScalarCampaignCheckpoint {
    schema_version: u32,
    campaign_seed: u64,
    sample_count: u64,
    dimension_id: u32,
    scenario_sha256: String,
    toolchain_profile: String,
    subtrees: Vec<ScalarCheckpointSubtree>,
}

impl ScalarCampaignCheckpoint {
    /// Current JSON checkpoint schema version.
    pub const SCHEMA_VERSION: u32 = 1;

    /// Construct an empty checkpoint for one deterministic scalar campaign.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::InvalidInput`] when `sample_count == 0`, or
    /// when `scenario_sha256` / `toolchain_profile` is empty.
    pub fn new(
        campaign_seed: u64,
        sample_count: u64,
        dimension_id: u32,
        scenario_sha256: impl Into<String>,
        toolchain_profile: impl Into<String>,
    ) -> Result<Self, MonteCarloError> {
        if sample_count == 0 {
            return Err(MonteCarloError::InvalidInput {
                field: "sample_count",
            });
        }
        let scenario_sha256 = scenario_sha256.into();
        if scenario_sha256.is_empty() {
            return Err(MonteCarloError::InvalidInput {
                field: "scenario_sha256",
            });
        }
        let toolchain_profile = toolchain_profile.into();
        if toolchain_profile.is_empty() {
            return Err(MonteCarloError::InvalidInput {
                field: "toolchain_profile",
            });
        }
        Ok(Self {
            schema_version: Self::SCHEMA_VERSION,
            campaign_seed,
            sample_count,
            dimension_id,
            scenario_sha256,
            toolchain_profile,
            subtrees: Vec::new(),
        })
    }

    /// Campaign seed declared by this checkpoint.
    #[must_use]
    pub const fn campaign_seed(&self) -> u64 {
        self.campaign_seed
    }

    /// Number of samples required to complete the campaign.
    #[must_use]
    pub const fn sample_count(&self) -> u64 {
        self.sample_count
    }

    /// Monte Carlo dimension id used for per-sample streams.
    #[must_use]
    pub const fn dimension_id(&self) -> u32 {
        self.dimension_id
    }

    /// Scenario digest bound to this checkpoint.
    #[must_use]
    pub fn scenario_sha256(&self) -> &str {
        &self.scenario_sha256
    }

    /// Toolchain / determinism profile bound to this checkpoint.
    #[must_use]
    pub fn toolchain_profile(&self) -> &str {
        &self.toolchain_profile
    }

    /// Number of completed samples recorded in the checkpoint.
    #[must_use]
    pub fn completed_count(&self) -> u64 {
        u64::try_from(self.subtrees.len()).unwrap_or(u64::MAX)
    }

    /// Completed sample indices in increasing order.
    #[must_use]
    pub fn completed_indices(&self) -> Vec<u64> {
        let mut indices = self
            .subtrees
            .iter()
            .map(|subtree| subtree.start_index)
            .collect::<Vec<_>>();
        indices.sort_unstable();
        indices
    }

    /// Whether all declared samples have completed.
    #[must_use]
    pub fn is_complete(&self) -> bool {
        self.completed_count() == self.sample_count
    }

    /// Validate this checkpoint against requested campaign metadata.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::CheckpointMismatch`] when campaign identity
    /// fields differ, or another [`MonteCarloError`] when checkpoint content is
    /// malformed.
    pub fn validate_campaign(
        &self,
        campaign_seed: u64,
        sample_count: u64,
        dimension_id: u32,
        scenario_sha256: &str,
        toolchain_profile: &str,
    ) -> Result<(), MonteCarloError> {
        self.validate_internal()?;
        if self.campaign_seed != campaign_seed {
            return Err(MonteCarloError::CheckpointMismatch {
                field: "campaign_seed",
            });
        }
        if self.sample_count != sample_count {
            return Err(MonteCarloError::CheckpointMismatch {
                field: "sample_count",
            });
        }
        if self.dimension_id != dimension_id {
            return Err(MonteCarloError::CheckpointMismatch {
                field: "dimension_id",
            });
        }
        if self.scenario_sha256 != scenario_sha256 {
            return Err(MonteCarloError::CheckpointMismatch {
                field: "scenario_sha256",
            });
        }
        if self.toolchain_profile != toolchain_profile {
            return Err(MonteCarloError::CheckpointMismatch {
                field: "toolchain_profile",
            });
        }
        Ok(())
    }

    /// Record one completed scalar sample as a reducer leaf.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError`] when the sample index is out of range, the
    /// value is invalid, or the sample index is already complete.
    pub fn record_sample(&mut self, sample: ScalarSample) -> Result<(), MonteCarloError> {
        self.record_samples([sample])
    }

    /// Finalize a complete checkpoint into a campaign report.
    ///
    /// # Errors
    ///
    /// Returns [`MonteCarloError::CheckpointIncomplete`] if not all samples are
    /// present, or another [`MonteCarloError`] if checkpoint content is invalid.
    pub fn to_report(&self) -> Result<ScalarCampaignReport, MonteCarloError> {
        self.validate_internal()?;
        if !self.is_complete() {
            return Err(MonteCarloError::CheckpointIncomplete {
                completed: self.completed_count(),
                required: self.sample_count,
            });
        }
        let mut samples = self
            .subtrees
            .iter()
            .map(|subtree| ScalarSample {
                sample_index: subtree.start_index,
                value: subtree.value_stats.mean(),
                success: subtree.successes == 1,
            })
            .collect::<Vec<_>>();
        samples.sort_by_key(|sample| sample.sample_index);
        scalar_campaign_report_from_samples(samples)
    }

    fn record_samples(
        &mut self,
        samples: impl IntoIterator<Item = ScalarSample>,
    ) -> Result<(), MonteCarloError> {
        self.validate_internal()?;
        let mut seen = self
            .subtrees
            .iter()
            .map(|subtree| subtree.start_index)
            .collect::<BTreeSet<_>>();
        for sample in samples {
            if sample.sample_index >= self.sample_count {
                return Err(MonteCarloError::InvalidInput {
                    field: "sample_index",
                });
            }
            if !seen.insert(sample.sample_index) {
                return Err(MonteCarloError::DuplicateSampleIndex {
                    index: sample.sample_index,
                });
            }
            let mut value_stats = Welford::new();
            value_stats.update(sample.value)?;
            self.subtrees.push(ScalarCheckpointSubtree {
                start_index: sample.sample_index,
                end_index: sample.sample_index,
                value_stats,
                successes: u64::from(sample.success),
            });
        }
        self.subtrees
            .sort_by_key(|subtree| (subtree.start_index, subtree.end_index));
        Ok(())
    }

    fn missing_indices(&self, limit: usize) -> Result<Vec<u64>, MonteCarloError> {
        self.validate_internal()?;
        let completed = self
            .completed_indices()
            .into_iter()
            .collect::<BTreeSet<_>>();
        Ok((0..self.sample_count)
            .filter(|sample_index| !completed.contains(sample_index))
            .take(limit)
            .collect())
    }

    fn validate_internal(&self) -> Result<(), MonteCarloError> {
        if self.schema_version != Self::SCHEMA_VERSION {
            return Err(MonteCarloError::CheckpointMismatch {
                field: "schema_version",
            });
        }
        if self.sample_count == 0 {
            return Err(MonteCarloError::InvalidInput {
                field: "sample_count",
            });
        }
        if self.scenario_sha256.is_empty() {
            return Err(MonteCarloError::InvalidInput {
                field: "scenario_sha256",
            });
        }
        if self.toolchain_profile.is_empty() {
            return Err(MonteCarloError::InvalidInput {
                field: "toolchain_profile",
            });
        }

        let mut seen = BTreeSet::new();
        for subtree in &self.subtrees {
            if subtree.start_index != subtree.end_index
                || subtree.start_index >= self.sample_count
                || subtree.value_stats.count() != 1
                || !subtree.value_stats.mean().is_finite()
                || subtree.value_stats.m2().to_bits() != 0.0_f64.to_bits()
                || subtree.successes > 1
            {
                return Err(MonteCarloError::InvalidInput {
                    field: "checkpoint",
                });
            }
            if !seen.insert(subtree.start_index) {
                return Err(MonteCarloError::DuplicateSampleIndex {
                    index: subtree.start_index,
                });
            }
        }
        Ok(())
    }
}

/// JSON file store for scalar campaign checkpoints.
///
/// Saves are written through a sibling temporary file and then renamed over the
/// target path, so readers either observe the previous complete checkpoint or
/// the new complete checkpoint.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FileCheckpointStore {
    path: PathBuf,
}

impl FileCheckpointStore {
    /// Construct a file checkpoint store for `path`.
    #[must_use]
    pub fn new(path: impl Into<PathBuf>) -> Self {
        Self { path: path.into() }
    }

    /// Checkpoint file path.
    #[must_use]
    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Load the checkpoint file if it exists.
    ///
    /// # Errors
    ///
    /// Returns [`CheckpointStoreError`] when reading, parsing, or checkpoint
    /// content validation fails.
    pub fn load(&self) -> Result<Option<ScalarCampaignCheckpoint>, CheckpointStoreError> {
        let text = match fs::read_to_string(&self.path) {
            Ok(text) => text,
            Err(source) if source.kind() == std::io::ErrorKind::NotFound => return Ok(None),
            Err(source) => {
                return Err(CheckpointStoreError::Io {
                    path: self.path.clone(),
                    source,
                });
            }
        };
        let checkpoint =
            serde_json::from_str::<ScalarCampaignCheckpoint>(&text).map_err(|source| {
                CheckpointStoreError::Json {
                    path: self.path.clone(),
                    source,
                }
            })?;
        checkpoint.validate_internal()?;
        Ok(Some(checkpoint))
    }

    /// Save a validated checkpoint to the store path.
    ///
    /// # Errors
    ///
    /// Returns [`CheckpointStoreError`] when validation, serialization, parent
    /// directory creation, writing, or replacement fails.
    pub fn save(&self, checkpoint: &ScalarCampaignCheckpoint) -> Result<(), CheckpointStoreError> {
        checkpoint.validate_internal()?;
        let text = serde_json::to_string_pretty(checkpoint).map_err(|source| {
            CheckpointStoreError::Json {
                path: self.path.clone(),
                source,
            }
        })?;
        if let Some(parent) = self.path.parent()
            && !parent.as_os_str().is_empty()
        {
            fs::create_dir_all(parent).map_err(|source| CheckpointStoreError::Io {
                path: parent.to_path_buf(),
                source,
            })?;
        }
        let tmp_path = checkpoint_tmp_path(&self.path);
        fs::write(&tmp_path, text).map_err(|source| CheckpointStoreError::Io {
            path: tmp_path.clone(),
            source,
        })?;
        fs::rename(&tmp_path, &self.path).map_err(|source| CheckpointStoreError::Io {
            path: self.path.clone(),
            source,
        })?;
        Ok(())
    }
}

fn checkpoint_tmp_path(path: &Path) -> PathBuf {
    match path.extension().and_then(|extension| extension.to_str()) {
        Some(extension) => path.with_extension(format!("{extension}.tmp")),
        None => path.with_extension("tmp"),
    }
}

/// Run a serial deterministic scalar Monte Carlo campaign.
///
/// Each sample receives a fresh [`SampleRng`] derived from
/// `(campaign_seed, sample_index, dimension_id)`. The final Welford
/// state is computed through the same fixed index-keyed merge tree used
/// by [`Welford::from_indexed_samples`]; the convergence trace is
/// intentionally sequential because it records the visible progression
/// by increasing sample index.
///
/// # Errors
///
/// Returns [`MonteCarloError`] when `sample_count == 0`, when the
/// evaluator returns an invalid outcome, or when the aggregate
/// statistics reject their inputs.
pub fn run_scalar_campaign<F>(
    campaign_seed: u64,
    sample_count: u64,
    dimension_id: u32,
    mut evaluate: F,
) -> Result<ScalarCampaignReport, MonteCarloError>
where
    F: FnMut(u64, &mut SampleRng) -> Result<ScalarOutcome, MonteCarloError>,
{
    if sample_count == 0 {
        return Err(MonteCarloError::InvalidInput {
            field: "sample_count",
        });
    }

    let capacity = usize::try_from(sample_count).map_err(|_| MonteCarloError::InvalidInput {
        field: "sample_count",
    })?;
    let mut samples = Vec::with_capacity(capacity);
    for sample_index in 0..sample_count {
        let mut rng = SampleRng::new(campaign_seed, sample_index, dimension_id);
        let outcome = evaluate(sample_index, &mut rng)?;
        samples.push(ScalarSample {
            sample_index,
            value: outcome.value,
            success: outcome.success,
        });
    }
    scalar_campaign_report_from_samples(samples)
}

/// Run a deterministic scalar Monte Carlo campaign over a bounded rayon worker
/// pool.
///
/// Each sample is still keyed only by `(campaign_seed, sample_index,
/// dimension_id)`. `worker_count` controls evaluation fan-out, but the final
/// report is sorted by sample index and reduced through the fixed merge tree, so
/// `worker_count ∈ {1,2,4,8}` produces identical report bytes for a deterministic
/// evaluator.
///
/// # Errors
///
/// Returns [`MonteCarloError`] when `sample_count == 0`, `worker_count == 0`,
/// the worker pool cannot be built, the evaluator returns an invalid outcome, or
/// the aggregate statistics reject their inputs.
pub fn run_scalar_campaign_parallel<F>(
    campaign_seed: u64,
    sample_count: u64,
    dimension_id: u32,
    worker_count: usize,
    evaluate: F,
) -> Result<ScalarCampaignReport, MonteCarloError>
where
    F: Fn(u64, &mut SampleRng) -> Result<ScalarOutcome, MonteCarloError> + Send + Sync,
{
    run_scalar_campaign_parallel_with_fp_check(
        campaign_seed,
        sample_count,
        dimension_id,
        worker_count,
        evaluate,
        || FpEnvironment::assert_current_clean().map_err(MonteCarloError::from),
    )
}

fn run_scalar_campaign_parallel_with_fp_check<F, C>(
    campaign_seed: u64,
    sample_count: u64,
    dimension_id: u32,
    worker_count: usize,
    evaluate: F,
    check_fp_environment: C,
) -> Result<ScalarCampaignReport, MonteCarloError>
where
    F: Fn(u64, &mut SampleRng) -> Result<ScalarOutcome, MonteCarloError> + Send + Sync,
    C: Fn() -> Result<(), MonteCarloError> + Send + Sync,
{
    if sample_count == 0 {
        return Err(MonteCarloError::InvalidInput {
            field: "sample_count",
        });
    }
    if worker_count == 0 {
        return Err(MonteCarloError::InvalidInput {
            field: "worker_count",
        });
    }
    let capacity = usize::try_from(sample_count).map_err(|_| MonteCarloError::InvalidInput {
        field: "sample_count",
    })?;
    let mut sample_indices = Vec::with_capacity(capacity);
    sample_indices.extend(0..sample_count);
    let samples = run_scalar_samples_parallel_with_fp_check(
        campaign_seed,
        dimension_id,
        worker_count,
        sample_indices,
        evaluate,
        check_fp_environment,
    )?;
    if samples.len() != capacity {
        return Err(MonteCarloError::InvalidInput {
            field: "sample_count",
        });
    }
    scalar_campaign_report_from_samples(samples)
}

/// Resume a scalar campaign checkpoint for up to `max_new_samples` new samples.
///
/// Completed samples are skipped. Newly evaluated samples are added to the
/// checkpoint as reducer leaves, and a final [`ScalarCampaignReport`] is
/// returned once the checkpoint is complete. Repeated calls with any chunking
/// pattern produce the same final report bits as [`run_scalar_campaign_parallel`]
/// for a deterministic evaluator.
///
/// # Errors
///
/// Returns [`MonteCarloError`] when the checkpoint is malformed,
/// `worker_count == 0`, `max_new_samples == 0`, the worker FP environment is
/// dirty, the evaluator fails, or final aggregate statistics reject their
/// inputs.
pub fn run_scalar_campaign_resumable<F>(
    checkpoint: &mut ScalarCampaignCheckpoint,
    worker_count: usize,
    max_new_samples: u64,
    evaluate: F,
) -> Result<Option<ScalarCampaignReport>, MonteCarloError>
where
    F: Fn(u64, &mut SampleRng) -> Result<ScalarOutcome, MonteCarloError> + Send + Sync,
{
    if max_new_samples == 0 {
        return Err(MonteCarloError::InvalidInput {
            field: "max_new_samples",
        });
    }
    checkpoint.validate_internal()?;
    if checkpoint.is_complete() {
        return Ok(Some(checkpoint.to_report()?));
    }
    let limit = usize::try_from(max_new_samples).unwrap_or(usize::MAX);
    let sample_indices = checkpoint.missing_indices(limit)?;
    let samples = run_scalar_samples_parallel_with_fp_check(
        checkpoint.campaign_seed,
        checkpoint.dimension_id,
        worker_count,
        sample_indices,
        evaluate,
        || FpEnvironment::assert_current_clean().map_err(MonteCarloError::from),
    )?;
    checkpoint.record_samples(samples)?;
    if checkpoint.is_complete() {
        Ok(Some(checkpoint.to_report()?))
    } else {
        Ok(None)
    }
}

fn run_scalar_samples_parallel_with_fp_check<F, C>(
    campaign_seed: u64,
    dimension_id: u32,
    worker_count: usize,
    sample_indices: Vec<u64>,
    evaluate: F,
    check_fp_environment: C,
) -> Result<Vec<ScalarSample>, MonteCarloError>
where
    F: Fn(u64, &mut SampleRng) -> Result<ScalarOutcome, MonteCarloError> + Send + Sync,
    C: Fn() -> Result<(), MonteCarloError> + Send + Sync,
{
    if worker_count == 0 {
        return Err(MonteCarloError::InvalidInput {
            field: "worker_count",
        });
    }
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(worker_count)
        .build()
        .map_err(|_| MonteCarloError::InvalidInput {
            field: "worker_count",
        })?;
    pool.install(|| {
        sample_indices
            .into_par_iter()
            .map(|sample_index| {
                check_fp_environment()?;
                let mut rng = SampleRng::new(campaign_seed, sample_index, dimension_id);
                let outcome = evaluate(sample_index, &mut rng)?;
                Ok(ScalarSample {
                    sample_index,
                    value: outcome.value,
                    success: outcome.success,
                })
            })
            .collect::<Result<Vec<_>, MonteCarloError>>()
    })
}

fn scalar_campaign_report_from_samples(
    mut samples: Vec<ScalarSample>,
) -> Result<ScalarCampaignReport, MonteCarloError> {
    if samples.is_empty() {
        return Err(MonteCarloError::InvalidInput {
            field: "sample_count",
        });
    }
    samples.sort_by_key(|sample| sample.sample_index);
    for window in samples.windows(2) {
        if window[0].sample_index == window[1].sample_index {
            return Err(MonteCarloError::DuplicateSampleIndex {
                index: window[0].sample_index,
            });
        }
    }

    let mut convergence_trace = Vec::with_capacity(samples.len());
    let mut trace_stats = Welford::new();
    let mut successes = 0_u64;
    for sample in &samples {
        trace_stats.update(sample.value)?;
        if sample.success {
            successes += 1;
        }
        convergence_trace.push(WelfordTraceRow {
            sample_index: sample.sample_index,
            count: trace_stats.count(),
            mean: trace_stats.mean(),
            sample_variance: trace_stats.sample_variance(),
            standard_error: trace_stats.standard_error(),
        });
    }

    let value_stats = Welford::from_indexed_samples(
        samples
            .iter()
            .map(|sample| (sample.sample_index, sample.value)),
    )?;
    let sample_count = u64::try_from(samples.len()).map_err(|_| MonteCarloError::InvalidInput {
        field: "sample_count",
    })?;
    let success_stats = BernoulliStats::new(sample_count, successes)?;
    Ok(ScalarCampaignReport {
        samples,
        value_stats,
        success_stats,
        convergence_trace,
    })
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use openbmp_uq::{CredibilityFactor, CredibilityRecord, UncertaintyClass, UncertaintySource};
    use std::fmt::Write as _;
    use tempfile::Builder;

    const MC_ENGINE_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "mc-propulsion-fault-library"
description = "Synthetic MC propulsion fault overlay parser fixture."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.2
dt_s = 0.1
seed = 17

[vehicle]
kind = "rigid_body"
initial_position_eci_m = [0.0, 0.0, 10.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]
initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
initial_angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "mc-propulsion-fault-library"

[[vehicle.assembly.bodies]]
id = "core"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 100.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[10.0, 0.0, 0.0], [0.0, 10.0, 0.0], [0.0, 0.0, 10.0]]

[[vehicle.assembly.engines]]
id = "main"
mounted_to = "core"
kind = { kind = "liquid_engine" }
mount_point_body_m = [0.0, 0.0, 0.0]
limits = { max_thrust_n = 1000.0, isp_s = 250.0, ignition_transient_s = 0.0, shutdown_transient_s = 0.0, max_gimbal_rad = 0.0 }

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 0.0
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity", "thrust"]

[telemetry]
output.csv = "out/mc-propulsion-fault-library.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    #[test]
    fn sample_rng_is_deterministic_and_domain_separated() {
        let mut a = SampleRng::new(0x1234, 7, 2);
        let mut b = SampleRng::new(0x1234, 7, 2);
        let mut c = SampleRng::new(0x1234, 7, 3);
        let same: Vec<u64> = (0..8).map(|_| a.next_u64()).collect();
        assert_eq!(same, (0..8).map(|_| b.next_u64()).collect::<Vec<_>>());
        assert_ne!(same, (0..8).map(|_| c.next_u64()).collect::<Vec<_>>());
    }

    #[test]
    fn propulsion_fault_library_materializes_parseable_overlay() {
        let library = PropulsionFaultLibrary::new(vec![
            PropulsionFaultTemplate {
                id: "mc-main-hardoff".to_owned(),
                engine_id: "main".to_owned(),
                start_step: 4,
                probability: 1.0,
                fault: PropulsionFaultPayload::HardOff,
            },
            PropulsionFaultTemplate {
                id: "mc-main-overthrust".to_owned(),
                engine_id: "main".to_owned(),
                start_step: 5,
                probability: 0.0,
                fault: PropulsionFaultPayload::OverThrust { factor: 1.1 },
            },
        ])
        .unwrap();

        let overlay = library.materialize(0xBEEF, 3, 12);
        assert_eq!(overlay.rules().len(), 1);
        assert_eq!(overlay.rules()[0].id, "mc-main-hardoff");
        let fragment = overlay.to_toml_fragment();
        assert!(fragment.contains("[propulsion.faults]"));
        assert!(fragment.contains("id = \"mc-main-hardoff\""));
        assert!(fragment.contains("fault = { kind = \"hard_off\" }"));
        assert!(!fragment.contains("mc-main-overthrust"));

        let scenario_toml = format!("{MC_ENGINE_SCENARIO}\n{fragment}");
        let scenario = openbmp_scenario::Scenario::from_toml_str(&scenario_toml).unwrap();
        let faults = scenario
            .document
            .propulsion
            .as_ref()
            .and_then(|propulsion| propulsion.faults.as_ref())
            .unwrap();
        assert_eq!(faults.rules.len(), 1);
        assert_eq!(faults.rules[0].id, "mc-main-hardoff");
        assert_eq!(faults.rules[0].engine_id, "main");
        assert_eq!(faults.rules[0].start_step, 4);
    }

    #[test]
    fn propulsion_fault_library_is_deterministic_and_validated() {
        let library = PropulsionFaultLibrary::new(vec![PropulsionFaultTemplate {
            id: "sampled-hardoff".to_owned(),
            engine_id: "main".to_owned(),
            start_step: 4,
            probability: 0.5,
            fault: PropulsionFaultPayload::HardOff,
        }])
        .unwrap();
        assert_eq!(
            library.materialize(0xCAFE, 7, 19),
            library.materialize(0xCAFE, 7, 19)
        );

        assert!(matches!(
            PropulsionFaultLibrary::new(vec![PropulsionFaultTemplate {
                id: "bad-probability".to_owned(),
                engine_id: "main".to_owned(),
                start_step: 1,
                probability: 1.01,
                fault: PropulsionFaultPayload::HardOff,
            }]),
            Err(MonteCarloError::InvalidInput {
                field: "fault.probability"
            })
        ));
        assert!(matches!(
            PropulsionFaultLibrary::new(vec![PropulsionFaultTemplate {
                id: "bad-factor".to_owned(),
                engine_id: "main".to_owned(),
                start_step: 1,
                probability: 1.0,
                fault: PropulsionFaultPayload::CavitationThrustLoss { factor: 1.2 },
            }]),
            Err(MonteCarloError::InvalidInput {
                field: "fault.factor"
            })
        ));
        assert!(matches!(
            PropulsionFaultLibrary::new(vec![
                PropulsionFaultTemplate {
                    id: "duplicate".to_owned(),
                    engine_id: "main".to_owned(),
                    start_step: 1,
                    probability: 1.0,
                    fault: PropulsionFaultPayload::HardOff,
                },
                PropulsionFaultTemplate {
                    id: "duplicate".to_owned(),
                    engine_id: "main".to_owned(),
                    start_step: 2,
                    probability: 1.0,
                    fault: PropulsionFaultPayload::HardOff,
                },
            ]),
            Err(MonteCarloError::InvalidInput { field: "fault.id" })
        ));
    }

    #[test]
    fn latin_hypercube_stratifies_each_column() {
        let sample_count = 8_u64;
        let dimension_count = 3_u32;
        let design = latin_hypercube(0x5151, sample_count, dimension_count).unwrap();
        assert_eq!(design.rows(), sample_count);
        assert_eq!(design.columns(), dimension_count);
        assert_eq!(
            design.values().len(),
            sample_count as usize * dimension_count as usize
        );
        for column in 0..dimension_count {
            let mut strata = vec![false; sample_count as usize];
            for row in 0..sample_count {
                let value = design.get(row, column).unwrap();
                assert!((0.0..1.0).contains(&value));
                let stratum = (value * sample_count as f64).floor() as usize;
                assert!(
                    !strata[stratum],
                    "duplicate stratum {stratum} in column {column}"
                );
                strata[stratum] = true;
            }
            assert!(strata.into_iter().all(|seen| seen));
        }
    }

    #[test]
    fn latin_hypercube_is_deterministic_and_seeded() {
        let lhs = LatinHypercube;
        let a = lhs.generate(0xA5A5, 6, 2).unwrap();
        let b = lhs.generate(0xA5A5, 6, 2).unwrap();
        let c = lhs.generate(0xA5A6, 6, 2).unwrap();
        assert_eq!(a, b);
        assert_ne!(a.values(), c.values());
    }

    #[test]
    fn latin_hypercube_rejects_empty_shapes() {
        assert!(matches!(
            latin_hypercube(0, 0, 1),
            Err(MonteCarloError::InvalidInput {
                field: "sample_count"
            }),
        ));
        assert!(matches!(
            latin_hypercube(0, 1, 0),
            Err(MonteCarloError::InvalidInput {
                field: "dimension_count"
            }),
        ));
    }

    #[test]
    fn sobol_matches_reference_first_points_2d() {
        let design = sobol(8, 2).unwrap();
        let expected: [(f64, f64); 8] = [
            (0.0, 0.0),
            (0.5, 0.5),
            (0.75, 0.25),
            (0.25, 0.75),
            (0.375, 0.375),
            (0.875, 0.875),
            (0.625, 0.125),
            (0.125, 0.625),
        ];
        for (row, (u0, u1)) in expected.into_iter().enumerate() {
            assert_eq!(design.get(row as u64, 0).unwrap().to_bits(), u0.to_bits());
            assert_eq!(design.get(row as u64, 1).unwrap().to_bits(), u1.to_bits());
        }
    }

    #[test]
    fn sobol_rejects_unsupported_dimensions() {
        assert!(matches!(
            SobolSequence.generate(4, SobolSequence::MAX_DIMENSIONS + 1),
            Err(MonteCarloError::InvalidInput {
                field: "dimension_count"
            }),
        ));
    }

    #[test]
    fn sobol_loads_provenance_pinned_joe_kuo_asset() {
        let design = sobol(1, SobolSequence::MAX_DIMENSIONS).unwrap();
        assert_eq!(design.rows(), 1);
        assert_eq!(design.columns(), SobolSequence::MAX_DIMENSIONS);
        assert_eq!(
            JOE_KUO_DIRECTION_NUMBERS.lines().count(),
            SobolSequence::MAX_DIMENSIONS as usize
        );
    }

    #[test]
    fn owen_scrambled_sobol_is_deterministic_and_seeded() {
        let a = owen_scrambled_sobol(8, 2, 0x0E11).unwrap();
        let b = SobolSequence.generate_owen_scrambled(8, 2, 0x0E11).unwrap();
        let c = OwenScramble::new(0x0E12).generate(8, 2).unwrap();
        let base = sobol(8, 2).unwrap();
        assert_eq!(a, b);
        assert_ne!(a.values(), c.values());
        assert_ne!(a.values(), base.values());
        assert!(a.values().iter().all(|value| (0.0..1.0).contains(value)));
    }

    #[test]
    fn iman_conover_preserves_marginals_and_induces_correlation() {
        let design = latin_hypercube(0x1C0E, 512, 2).unwrap();
        let target = CorrelationMatrix::new(2, vec![1.0, 0.75, 0.75, 1.0]).unwrap();
        let induced = ImanConover::new(target).apply(&design).unwrap();

        assert_eq!(
            sorted_column_bits(&design, 0),
            sorted_column_bits(&induced, 0)
        );
        assert_eq!(
            sorted_column_bits(&design, 1),
            sorted_column_bits(&induced, 1)
        );
        let correlation = pearson_columns(&induced, 0, 1);
        assert!(
            (correlation - 0.75).abs() <= 0.05,
            "correlation {correlation} did not approach target 0.75"
        );
    }

    #[test]
    fn iman_conover_rejects_invalid_targets() {
        assert!(matches!(
            CorrelationMatrix::new(2, vec![1.0, 1.0, 1.0, 1.0]),
            Err(MonteCarloError::InvalidInput {
                field: "correlation positive definite"
            }),
        ));
        let design = latin_hypercube(0x1C0E, 8, 2).unwrap();
        let target = CorrelationMatrix::identity(3).unwrap();
        assert!(matches!(
            iman_conover(&design, &target),
            Err(MonteCarloError::InvalidInput {
                field: "correlation size"
            }),
        ));
    }

    #[test]
    fn welford_matches_two_pass_variance() {
        let values = [-2.0, -0.5, 0.0, 1.5, 3.0, 7.25, 11.0, -4.5, 2.25, 0.75];
        let stats = Welford::from_values(values).unwrap();
        let mean = values.iter().sum::<f64>() / values.len() as f64;
        let two_pass = values
            .iter()
            .map(|value| {
                let d = *value - mean;
                d * d
            })
            .sum::<f64>()
            / (values.len() - 1) as f64;
        let got = stats.sample_variance().unwrap();
        assert!((got - two_pass).abs() <= two_pass.abs() * 1.0e-12);
    }

    #[test]
    fn indexed_welford_is_order_independent() {
        let samples = [
            (4, 3.0),
            (1, -1.0),
            (3, 2.0),
            (0, 9.0),
            (2, 0.5),
            (5, -4.25),
        ];
        let mut reversed = samples;
        reversed.reverse();
        let a = Welford::from_indexed_samples(samples).unwrap();
        let b = Welford::from_indexed_samples(reversed).unwrap();
        assert_eq!(a.count(), b.count());
        assert_eq!(a.mean().to_bits(), b.mean().to_bits());
        assert_eq!(a.m2().to_bits(), b.m2().to_bits());
    }

    #[test]
    fn indexed_welford_rejects_duplicate_indices() {
        assert!(matches!(
            Welford::from_indexed_samples([(0, 1.0), (0, 2.0)]),
            Err(MonteCarloError::DuplicateSampleIndex { index: 0 }),
        ));
    }

    #[test]
    fn clopper_pearson_matches_known_symmetric_case() {
        let stats = BernoulliStats::new(10, 5).unwrap();
        let interval = stats.clopper_pearson(0.95).unwrap();
        assert!((interval.lower - 0.187_086_028_447_404_5).abs() < 1.0e-13);
        assert!((interval.upper - 0.812_913_971_552_595_5).abs() < 1.0e-13);
    }

    #[test]
    fn clopper_pearson_handles_edge_counts() {
        let zero = BernoulliStats::new(10, 0)
            .unwrap()
            .clopper_pearson(0.95)
            .unwrap();
        let all = BernoulliStats::new(10, 10)
            .unwrap()
            .clopper_pearson(0.95)
            .unwrap();
        assert_eq!(zero.lower.to_bits(), 0.0_f64.to_bits());
        assert!(zero.upper > 0.0 && zero.upper < 1.0);
        assert!(all.lower > 0.0 && all.lower < 1.0);
        assert_eq!(all.upper.to_bits(), 1.0_f64.to_bits());
    }

    #[test]
    fn wilks_sizing_matches_reference_case() {
        let coverage = 0.99865;
        let confidence = 0.90;
        let one_sided = wilks_one_sided_n(coverage, confidence).unwrap();
        let two_sided = wilks_two_sided_n(coverage, confidence).unwrap();
        assert_eq!(one_sided, 1705);
        assert_eq!(two_sided, 2880);
        assert!(wilks_one_sided_confidence(coverage, one_sided).unwrap() >= confidence);
        assert!(wilks_one_sided_confidence(coverage, one_sided - 1).unwrap() < confidence);
        assert!(wilks_two_sided_confidence(coverage, two_sided).unwrap() >= confidence);
        assert!(wilks_two_sided_confidence(coverage, two_sided - 1).unwrap() < confidence);
    }

    #[test]
    fn wilks_sizing_rejects_invalid_probabilities() {
        assert!(matches!(
            wilks_one_sided_n(1.0, 0.9),
            Err(MonteCarloError::InvalidInput { field: "coverage" }),
        ));
        assert!(matches!(
            wilks_two_sided_n(0.9, 0.0),
            Err(MonteCarloError::InvalidInput {
                field: "confidence"
            }),
        ));
    }

    #[test]
    fn convergence_gate_tracks_window_and_wilks_floor() {
        let trace = [
            WelfordTraceRow {
                sample_index: 0,
                count: 1,
                mean: 100.0,
                sample_variance: None,
                standard_error: None,
            },
            WelfordTraceRow {
                sample_index: 1,
                count: 2,
                mean: 100.0,
                sample_variance: Some(8.0),
                standard_error: Some(2.0),
            },
            WelfordTraceRow {
                sample_index: 2,
                count: 3,
                mean: 100.0,
                sample_variance: Some(0.0048),
                standard_error: Some(0.04),
            },
            WelfordTraceRow {
                sample_index: 3,
                count: 4,
                mean: 100.0,
                sample_variance: Some(0.0036),
                standard_error: Some(0.03),
            },
        ];
        let gate = ConvergenceGate::new(0.001, 2, 0.95).unwrap();
        assert_eq!(gate.first_converged_at(&trace).unwrap(), Some(3));
        assert_eq!(
            gate.stop_state(&trace[..2], 4, 3).unwrap(),
            CampaignStop::WilksFloor,
        );
        assert_eq!(
            gate.stop_state(&trace, 4, 2).unwrap(),
            CampaignStop::Converged { at: 3 },
        );
        let strict_gate = ConvergenceGate::new(0.0001, 2, 0.95).unwrap();
        assert_eq!(
            strict_gate.stop_state(&trace, 4, 2).unwrap(),
            CampaignStop::ReachedN,
        );
    }

    #[test]
    fn scalar_campaign_reports_samples_summary_and_trace() {
        let report = run_scalar_campaign(0xCAFE, 4, 0, |sample_index, rng| {
            let jitter = rng.uniform01();
            ScalarOutcome::new(sample_index as f64 + jitter, sample_index != 1)
        })
        .unwrap();
        assert_eq!(report.samples.len(), 4);
        assert_eq!(report.convergence_trace.len(), 4);
        assert_eq!(report.value_stats.count(), 4);
        assert_eq!(report.success_stats.successes(), 3);
        assert_eq!(report.convergence_trace[3].count, 4);
        assert!(report.convergence_trace[3].standard_error.is_some());
        let interval = report.success_interval(0.95).unwrap();
        assert!(interval.lower < report.success_stats.fraction().unwrap());
        assert!(interval.upper > report.success_stats.fraction().unwrap());
    }

    #[test]
    fn scalar_campaign_rejects_zero_samples() {
        assert!(matches!(
            run_scalar_campaign(0, 0, 0, |_, _| ScalarOutcome::new(0.0, true)),
            Err(MonteCarloError::InvalidInput {
                field: "sample_count"
            }),
        ));
    }

    #[test]
    fn scalar_campaign_parallel_matches_serial_for_worker_counts() {
        let serial = run_scalar_campaign(0x5EED, 129, 7, scalar_parallel_test_outcome).unwrap();
        for worker_count in [1, 2, 4, 8] {
            let parallel =
                run_scalar_campaign_parallel(0x5EED, 129, 7, worker_count, |sample_index, rng| {
                    scalar_parallel_test_outcome(sample_index, rng)
                })
                .unwrap();
            assert_scalar_campaign_reports_bit_identical(&serial, &parallel);
        }
    }

    #[test]
    fn scalar_campaign_report_bytes_match_across_worker_counts() {
        let serial = run_scalar_campaign(0xB17E, 65, 4, scalar_parallel_test_outcome).unwrap();
        let serial_bytes = campaign_report_bytes(&serial);
        for worker_count in [1, 2, 4, 8] {
            let parallel =
                run_scalar_campaign_parallel(0xB17E, 65, 4, worker_count, |sample_index, rng| {
                    scalar_parallel_test_outcome(sample_index, rng)
                })
                .unwrap();
            assert_eq!(serial_bytes, campaign_report_bytes(&parallel));
        }
    }

    #[test]
    fn scalar_campaign_report_bytes_match_pinned_golden() {
        let report = run_scalar_campaign(0xB17E, 65, 4, scalar_parallel_test_outcome).unwrap();
        let digest = fnv1a64(&campaign_report_bytes(&report));
        assert_eq!(format!("{digest:016x}"), "e5c90b1286aebc3d");
    }

    #[test]
    fn campaign_credibility_report_accepts_floor_and_splits_sources() {
        let budget = credibility_budget();
        let report = evaluate_campaign_credibility(&budget, CredibilityLevel::L2).unwrap();
        assert!(report.accepted);
        assert_eq!(report.binding_level, CredibilityLevel::L2);
        assert_eq!(report.legacy_label, ValidationStatus::ValidatedToy);
        assert!((report.aggregate_one_sigma - 5.0).abs() < 1.0e-12);
        assert!((report.aleatory_one_sigma - 3.0).abs() < 1.0e-12);
        assert!((report.epistemic_one_sigma - 4.0).abs() < 1.0e-12);
        assert!(report.markdown.contains("Monte Carlo credibility report"));
        assert!(report.markdown.contains("accepted = true"));
    }

    #[test]
    fn campaign_credibility_report_refuses_floor_above_binding_level() {
        let budget = credibility_budget();
        let report = evaluate_campaign_credibility(&budget, CredibilityLevel::L3).unwrap();
        assert!(!report.accepted);
        assert_eq!(report.binding_level, CredibilityLevel::L2);
        assert_eq!(report.floor, CredibilityLevel::L3);
        assert!(report.markdown.contains("accepted = false"));
    }

    #[test]
    fn nested_scalar_analysis_reports_pbox_and_variance_split() {
        let conditional = vec![vec![-3.0, 1.0], vec![-1.0, 3.0]];
        let report = analyze_nested_scalar_samples(
            &conditional,
            Some(NestedLowerTailRequirement {
                threshold: 1.0,
                minimum_probability: 0.5,
            }),
        )
        .unwrap();

        assert_eq!(report.epistemic_samples, 2);
        assert_eq!(report.min_aleatory_samples, 2);
        assert_eq!(report.max_aleatory_samples, 2);
        assert!((report.variance_split.aleatory - 4.0).abs() < 1.0e-12);
        assert!((report.variance_split.epistemic - 1.0).abs() < 1.0e-12);
        assert!((report.variance_split.total - 5.0).abs() < 1.0e-12);
        assert!(report.variance_split.identity_residual() <= 1.0e-12);
        assert_eq!(report.lower_bound_probability, Some(0.5));
        assert_eq!(report.requirement_passed, Some(true));
    }

    #[test]
    fn nested_scalar_requirement_uses_lower_pbox_bound_not_mean_cdf() {
        let conditional = vec![vec![0.0, 0.0], vec![10.0, 10.0]];
        let report = analyze_nested_scalar_samples(
            &conditional,
            Some(NestedLowerTailRequirement {
                threshold: 0.0,
                minimum_probability: 0.5,
            }),
        )
        .unwrap();

        assert_eq!(
            report.pbox.lower_cdf_at(0.0).unwrap().to_bits(),
            0.0_f64.to_bits()
        );
        assert_eq!(
            report.pbox.upper_cdf_at(0.0).unwrap().to_bits(),
            1.0_f64.to_bits()
        );
        assert_eq!(report.lower_bound_probability, Some(0.0));
        assert_eq!(report.requirement_passed, Some(false));
    }

    #[test]
    fn synthetic_linear_limit_state_reports_exact_normal_tail() {
        let limit_state = SyntheticLinearLimitState::new(3.0, 4).unwrap();
        let exact = limit_state.exact_failure_probability().unwrap();
        assert!(
            (exact - 0.001_349_898_031_630_095_9).abs() < 1.0e-13,
            "{exact:.17e}",
        );
        let point = StandardNormalPoint::new(vec![1.5, 1.5, 1.5, 1.5]).unwrap();
        assert_eq!(
            limit_state.evaluate(&point).unwrap().to_bits(),
            0.0_f64.to_bits()
        );
    }

    #[test]
    fn subset_simulation_recovers_synthetic_linear_tail_and_replays() {
        let limit_state = SyntheticLinearLimitState::new(3.0, 4).unwrap();
        let estimator = SubsetSimulation::new(0x5EED, 4096, 0.1, 6, 0.8, 19).unwrap();
        let first = estimator.estimate(&limit_state).unwrap();
        let second = estimator.estimate(&limit_state).unwrap();

        assert_eq!(first, second);
        assert_eq!(first.method, RareEventMethod::SubsetSimulation);
        assert_eq!(first.limit_state_label, "synthetic-limit-state");
        assert_eq!(first.dimension, 4);
        assert!(first.subset_levels.len() >= 2);
        assert!(first.cross_entropy_iterations.is_empty());
        assert!(first.failure_probability > 0.0);
        assert!(first.coefficient_of_variation < 0.2, "{first:?}");
        assert!(first.abs_log10_error.unwrap() <= 0.3, "{:?}", first,);
    }

    #[test]
    fn cross_entropy_is_recovers_synthetic_linear_tail_and_replays() {
        let limit_state = SyntheticLinearLimitState::new(3.0, 4).unwrap();
        let estimator = CrossEntropyIs::new(0xCE15, 4096, 0.1, 5, 0.8, 0.2, 23).unwrap();
        let first = estimator.estimate(&limit_state).unwrap();
        let second = estimator.estimate(&limit_state).unwrap();

        assert_eq!(first, second);
        assert_eq!(
            first.method,
            RareEventMethod::CrossEntropyImportanceSampling,
        );
        assert_eq!(first.limit_state_label, "synthetic-limit-state");
        assert!(first.subset_levels.is_empty());
        assert!(!first.cross_entropy_iterations.is_empty());
        assert!(first.cross_entropy_iterations.len() <= 5);
        assert!(first.failure_probability > 0.0);
        assert!(first.coefficient_of_variation < 0.2, "{first:?}");
        assert!(first.abs_log10_error.unwrap() <= 0.3, "{:?}", first,);
    }

    #[test]
    fn rare_event_estimators_reject_non_synthetic_limit_state_label() {
        struct AimpointLimitState;

        impl rare_event_sealed::LimitStateSealed for AimpointLimitState {}

        impl LimitState for AimpointLimitState {
            fn label(&self) -> &'static str {
                "ground-aimpoint"
            }

            fn dimension(&self) -> usize {
                1
            }

            fn evaluate(&self, point: &StandardNormalPoint) -> Result<f64, MonteCarloError> {
                Ok(1.0 - point.coordinates()[0])
            }
        }

        let estimator = SubsetSimulation::new(0xBAD, 128, 0.1, 3, 0.8, 0).unwrap();
        assert!(matches!(
            estimator.estimate(&AimpointLimitState),
            Err(MonteCarloError::InvalidInput {
                field: "limit_state.label",
            }),
        ));
    }

    #[test]
    fn scalar_campaign_checkpoint_resume_matches_one_shot() {
        let one_shot = run_scalar_campaign_parallel(0x5EED, 129, 7, 8, |sample_index, rng| {
            scalar_parallel_test_outcome(sample_index, rng)
        })
        .unwrap();
        let mut checkpoint =
            ScalarCampaignCheckpoint::new(0x5EED, 129, 7, "scenario-a", "x86_64-clean").unwrap();

        assert!(
            run_scalar_campaign_resumable(&mut checkpoint, 2, 17, scalar_parallel_test_outcome)
                .unwrap()
                .is_none()
        );
        assert_eq!(checkpoint.completed_count(), 17);

        let mut resumed = None;
        for (worker_count, max_new_samples) in [(4, 13), (1, 29), (8, 31), (2, 100)] {
            resumed = run_scalar_campaign_resumable(
                &mut checkpoint,
                worker_count,
                max_new_samples,
                scalar_parallel_test_outcome,
            )
            .unwrap();
            if resumed.is_some() {
                break;
            }
        }
        let resumed = resumed.unwrap();
        assert_eq!(checkpoint.completed_count(), 129);
        assert_scalar_campaign_reports_bit_identical(&one_shot, &resumed);
    }

    #[test]
    fn scalar_campaign_file_checkpoint_roundtrips_and_checks_metadata() {
        let temp = Builder::new()
            .prefix("openbmp_mc_checkpoint")
            .tempdir()
            .unwrap();
        let path = temp.path().join("campaign.checkpoint.json");
        let store = FileCheckpointStore::new(path);
        assert!(store.load().unwrap().is_none());

        let mut checkpoint =
            ScalarCampaignCheckpoint::new(0xA11CE, 4, 3, "scenario-a", "x86_64-clean").unwrap();
        checkpoint
            .record_sample(ScalarSample {
                sample_index: 2,
                value: 42.0,
                success: true,
            })
            .unwrap();
        store.save(&checkpoint).unwrap();

        let loaded = store.load().unwrap().unwrap();
        assert_eq!(loaded.campaign_seed(), checkpoint.campaign_seed());
        assert_eq!(loaded.sample_count(), checkpoint.sample_count());
        assert_eq!(loaded.dimension_id(), checkpoint.dimension_id());
        assert_eq!(loaded.scenario_sha256(), checkpoint.scenario_sha256());
        assert_eq!(loaded.toolchain_profile(), checkpoint.toolchain_profile());
        assert_eq!(loaded.completed_indices(), vec![2]);
        assert!(matches!(
            loaded.validate_campaign(0xA11CE, 4, 3, "scenario-b", "x86_64-clean"),
            Err(MonteCarloError::CheckpointMismatch {
                field: "scenario_sha256"
            }),
        ));
    }

    #[test]
    fn scalar_campaign_checkpoint_rejects_incomplete_finalization() {
        let checkpoint =
            ScalarCampaignCheckpoint::new(0x5EED, 2, 0, "scenario-a", "x86_64-clean").unwrap();
        assert!(matches!(
            checkpoint.to_report(),
            Err(MonteCarloError::CheckpointIncomplete {
                completed: 0,
                required: 2,
            }),
        ));
    }

    #[test]
    fn scalar_campaign_parallel_rejects_zero_workers() {
        assert!(matches!(
            run_scalar_campaign_parallel(0, 1, 0, 0, |_, _| ScalarOutcome::new(0.0, true)),
            Err(MonteCarloError::InvalidInput {
                field: "worker_count"
            }),
        ));
    }

    #[test]
    fn scalar_campaign_parallel_checks_worker_fp_environment() {
        let dirty = FpEnvironment::from_x86_64_mxcsr(1 << 15);
        assert!(matches!(
            run_scalar_campaign_parallel_with_fp_check(
                0,
                16,
                0,
                4,
                |_, _| ScalarOutcome::new(1.0, true),
                || Err(MonteCarloError::FpEnvironmentDirty { observed: dirty }),
            ),
            Err(MonteCarloError::FpEnvironmentDirty { observed }) if observed == dirty,
        ));
    }

    #[test]
    fn scalar_campaign_report_from_samples_rejects_duplicate_indices() {
        assert!(matches!(
            scalar_campaign_report_from_samples(vec![
                ScalarSample {
                    sample_index: 2,
                    value: 1.0,
                    success: true,
                },
                ScalarSample {
                    sample_index: 2,
                    value: 3.0,
                    success: false,
                },
            ]),
            Err(MonteCarloError::DuplicateSampleIndex { index: 2 }),
        ));
    }

    fn scalar_parallel_test_outcome(
        sample_index: u64,
        rng: &mut SampleRng,
    ) -> Result<ScalarOutcome, MonteCarloError> {
        let jitter = rng.uniform01();
        let normal = rng.standard_normal();
        let value = (sample_index as f64 * 0.25) + jitter + normal * 0.01;
        ScalarOutcome::new(value, sample_index % 5 != 2)
    }

    fn credibility_budget() -> CorrelatedErrorBudget {
        let mut record = CredibilityRecord::new();
        for factor in CredibilityFactor::ALL {
            record = record.with_score(factor, CredibilityLevel::L2, format!("V-{factor:?}"));
        }
        CorrelatedErrorBudget {
            sources: vec![
                UncertaintySource {
                    source_id: "wind".to_owned(),
                    one_sigma: 3.0,
                    class: UncertaintyClass::Aleatory,
                    credibility: record.clone(),
                    justification: "synthetic wind evidence".to_owned(),
                },
                UncertaintySource {
                    source_id: "aero".to_owned(),
                    one_sigma: 4.0,
                    class: UncertaintyClass::Epistemic,
                    credibility: record,
                    justification: "synthetic aero evidence".to_owned(),
                },
            ],
            correlation: None,
        }
    }

    fn assert_scalar_campaign_reports_bit_identical(
        left: &ScalarCampaignReport,
        right: &ScalarCampaignReport,
    ) {
        assert_eq!(left.samples.len(), right.samples.len());
        for (left, right) in left.samples.iter().zip(right.samples.iter()) {
            assert_eq!(left.sample_index, right.sample_index);
            assert_eq!(left.value.to_bits(), right.value.to_bits());
            assert_eq!(left.success, right.success);
        }

        assert_eq!(left.value_stats.count(), right.value_stats.count());
        assert_eq!(
            left.value_stats.mean().to_bits(),
            right.value_stats.mean().to_bits()
        );
        assert_eq!(
            left.value_stats.m2().to_bits(),
            right.value_stats.m2().to_bits()
        );
        assert_eq!(
            optional_f64_bits(left.value_stats.sample_variance()),
            optional_f64_bits(right.value_stats.sample_variance()),
        );
        assert_eq!(
            optional_f64_bits(left.value_stats.standard_error()),
            optional_f64_bits(right.value_stats.standard_error()),
        );

        assert_eq!(left.success_stats.trials(), right.success_stats.trials());
        assert_eq!(
            left.success_stats.successes(),
            right.success_stats.successes()
        );
        assert_eq!(
            optional_f64_bits(left.success_stats.fraction()),
            optional_f64_bits(right.success_stats.fraction()),
        );

        assert_eq!(left.convergence_trace.len(), right.convergence_trace.len());
        for (left, right) in left
            .convergence_trace
            .iter()
            .zip(right.convergence_trace.iter())
        {
            assert_eq!(left.sample_index, right.sample_index);
            assert_eq!(left.count, right.count);
            assert_eq!(left.mean.to_bits(), right.mean.to_bits());
            assert_eq!(
                optional_f64_bits(left.sample_variance),
                optional_f64_bits(right.sample_variance),
            );
            assert_eq!(
                optional_f64_bits(left.standard_error),
                optional_f64_bits(right.standard_error),
            );
        }
    }

    fn campaign_report_bytes(report: &ScalarCampaignReport) -> Vec<u8> {
        let mut out = String::new();
        let _ = writeln!(out, "openbmp.scalar_campaign_report.v1");
        let _ = writeln!(out, "samples,{}", report.samples.len());
        let _ = write!(
            out,
            "value_stats,{},{:016x},{:016x},",
            report.value_stats.count(),
            report.value_stats.mean().to_bits(),
            report.value_stats.m2().to_bits()
        );
        push_optional_f64_bits(&mut out, report.value_stats.sample_variance());
        out.push(',');
        push_optional_f64_bits(&mut out, report.value_stats.standard_error());
        out.push('\n');
        let _ = write!(
            out,
            "success_stats,{},{}",
            report.success_stats.trials(),
            report.success_stats.successes()
        );
        out.push(',');
        push_optional_f64_bits(&mut out, report.success_stats.fraction());
        out.push('\n');
        for sample in &report.samples {
            let _ = writeln!(
                out,
                "sample,{},{:016x},{}",
                sample.sample_index,
                sample.value.to_bits(),
                u8::from(sample.success)
            );
        }
        for row in &report.convergence_trace {
            let _ = write!(
                out,
                "trace,{},{},{:016x},",
                row.sample_index,
                row.count,
                row.mean.to_bits()
            );
            push_optional_f64_bits(&mut out, row.sample_variance);
            out.push(',');
            push_optional_f64_bits(&mut out, row.standard_error);
            out.push('\n');
        }
        out.into_bytes()
    }

    fn push_optional_f64_bits(out: &mut String, value: Option<f64>) {
        if let Some(bits) = optional_f64_bits(value) {
            let _ = write!(out, "{bits:016x}");
        } else {
            out.push_str("none");
        }
    }

    fn fnv1a64(bytes: &[u8]) -> u64 {
        const FNV_OFFSET_BASIS_64: u64 = 0xcbf2_9ce4_8422_2325;
        const FNV_PRIME_64: u64 = 0x100_0000_01b3;

        let mut hash = FNV_OFFSET_BASIS_64;
        for byte in bytes {
            hash ^= u64::from(*byte);
            hash = hash.wrapping_mul(FNV_PRIME_64);
        }
        hash
    }

    fn optional_f64_bits(value: Option<f64>) -> Option<u64> {
        value.map(f64::to_bits)
    }

    fn sorted_column_bits(design: &DesignMatrix, column: u32) -> Vec<u64> {
        let mut values = design.column_values(column).unwrap();
        values.sort_by(total_f64_order);
        values.into_iter().map(f64::to_bits).collect()
    }

    fn pearson_columns(design: &DesignMatrix, left: u32, right: u32) -> f64 {
        let left = design.column_values(left).unwrap();
        let right = design.column_values(right).unwrap();
        let left_mean = left.iter().sum::<f64>() / left.len() as f64;
        let right_mean = right.iter().sum::<f64>() / right.len() as f64;
        let mut covariance = 0.0;
        let mut left_m2 = 0.0;
        let mut right_m2 = 0.0;
        for (left, right) in left.iter().zip(right.iter()) {
            let dl = left - left_mean;
            let dr = right - right_mean;
            covariance += dl * dr;
            left_m2 += dl * dl;
            right_m2 += dr * dr;
        }
        covariance / (left_m2 * right_m2).sqrt()
    }
}
