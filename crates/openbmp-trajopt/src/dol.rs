//! Day-of-launch I-load redesign helpers.
//!
//! This module implements the WP-19.2/WP-19.3/WP-19.5/WP-19.6 toy DOL method tier:
//! given a quality-controlled spliced wind profile, recenter pitch/yaw steering
//! bias tables over a load-critical altitude window, emit an I-load update
//! artifact, fit deterministic load-indicator surrogates for downstream
//! screening, apply synthetic persistence knockdowns, screen declared
//! launch-window slots against synthetic catalog states, and latch those gates
//! into a commit-readiness artifact.
//! The terminal target is the closed [`TerminalCondition`] enum from
//! `openbmp-physics`; this module deliberately adds no new target vocabulary.

use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;
use core::fmt::Write as _;

use openbmp_physics::WGS84_A_M;
use openbmp_physics::profile::TerminalCondition;
use openbmp_physics::wind::SplicedWindProfile;
use serde::{Deserialize, Serialize};

use crate::iload::{
    GainAxis, GainTable, ILoadHeader, ILoadObjectiveClass, ILoadPayload, ReferenceProfileSample,
    SynthesisMetadata, TerminalConditionKind, TrajoptError,
};

/// DOL update artifact schema version.
pub const DOL_UPDATE_SCHEMA_VERSION: u16 = 1;

/// DOL load-indicator surrogate schema version.
pub const DOL_LOAD_INDICATOR_SURROGATE_SCHEMA_VERSION: u16 = 1;

/// DOL go/no-go report-card schema version.
pub const DOL_REPORT_CARD_SCHEMA_VERSION: u16 = 1;

/// DOL persistence/knockdown deck schema version.
pub const DOL_PERSISTENCE_KNOCKDOWN_SCHEMA_VERSION: u16 = 1;

/// DOL launch-COLA screening schema version.
pub const DOL_COLA_SCREENING_SCHEMA_VERSION: u16 = 1;

/// DOL commit-readiness schema version.
pub const DOL_COMMIT_READINESS_SCHEMA_VERSION: u16 = 1;

const INDICATOR_FEATURE_COUNT: usize = 4;
const REPORT_TOLERANCE: f64 = 1.0e-9;

/// Load-critical altitude window used for wind recentering.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DolLoadWindow {
    /// Lower altitude, metres.
    pub min_altitude_m: f64,
    /// Upper altitude, metres.
    pub max_altitude_m: f64,
}

impl DolLoadWindow {
    /// Construct a load window.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the window is not finite and strictly
    /// increasing.
    pub fn new(min_altitude_m: f64, max_altitude_m: f64) -> Result<Self, TrajoptError> {
        let window = Self {
            min_altitude_m,
            max_altitude_m,
        };
        window.validate()?;
        Ok(window)
    }

    fn validate(self) -> Result<(), TrajoptError> {
        if !self.min_altitude_m.is_finite() || !self.max_altitude_m.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL load-window altitudes must be finite",
            });
        }
        if self.max_altitude_m <= self.min_altitude_m {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL load window must be strictly increasing",
            });
        }
        Ok(())
    }

    fn contains(self, altitude_m: f64) -> bool {
        altitude_m >= self.min_altitude_m && altitude_m <= self.max_altitude_m
    }
}

/// Inputs to the DOL I-load redesign helper.
#[derive(Clone, Debug, PartialEq)]
pub struct DolRedesignInput<'a> {
    /// Spliced measured wind profile.
    pub spliced_wind: &'a SplicedWindProfile,
    /// Closed terminal condition inherited from doc `07`.
    pub terminal_condition: TerminalCondition,
    /// Deterministic synthesis seed recorded in the I-load header.
    pub synthesis_seed: u64,
    /// Opaque scenario or design-case digest.
    pub scenario_digest: String,
    /// Source revision recorded in the I-load metadata.
    pub source_revision: String,
    /// Producer name recorded in the I-load header.
    pub producer: String,
    /// Design-profile digest or provenance pin.
    pub design_profile_digest: String,
    /// Solver provenance label.
    pub solver_provenance: String,
    /// Load-critical altitude window.
    pub load_window: DolLoadWindow,
    /// Reference design airspeed through the load window, m/s.
    pub airspeed_m_s: f64,
    /// Desired mean angle of attack, rad.
    pub alpha_reference_rad: f64,
    /// Desired mean sideslip, rad.
    pub beta_reference_rad: f64,
    /// Existing pitch steering bias, rad.
    pub stale_pitch_bias_rad: f64,
    /// Existing yaw steering bias, rad.
    pub stale_yaw_bias_rad: f64,
    /// Scalar load-angle limit for the toy q-alpha/q-beta surrogate, rad.
    pub load_limit_angle_rad: f64,
    /// Terminal/insertion residual norm supplied by the design pass, m.
    pub insertion_residual_norm_m: f64,
    /// Accepted terminal/insertion residual limit, m.
    pub insertion_residual_limit_m: f64,
}

/// Deterministic DOL I-load update result.
#[derive(Clone, Debug, PartialEq)]
pub struct ILoadUpdate {
    /// Existing I-load payload carrying gain tables and reference profile.
    pub payload: ILoadPayload,
    /// DOL-specific report artifact.
    pub artifact: ILoadUpdateArtifact,
}

/// DOL-specific I-load update report.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ILoadUpdateArtifact {
    /// DOL artifact schema version.
    pub schema_version: u16,
    /// Terminal-condition family used by the design pass.
    pub terminal_condition_kind: TerminalConditionKind,
    /// Design-profile digest or provenance pin.
    pub design_profile_digest: String,
    /// Solver provenance label.
    pub solver_provenance: String,
    /// SHA-256 digest of the encoded updated I-load payload.
    pub iload_sha256_hex: String,
    /// Samples used inside the load window.
    pub load_window_sample_count: usize,
    /// Existing pitch steering bias, rad.
    pub stale_pitch_bias_rad: f64,
    /// Existing yaw steering bias, rad.
    pub stale_yaw_bias_rad: f64,
    /// Recentered pitch steering bias, rad.
    pub corrected_pitch_bias_rad: f64,
    /// Recentered yaw steering bias, rad.
    pub corrected_yaw_bias_rad: f64,
    /// Minimum stale load-angle margin over the load window, rad.
    pub stale_min_margin_rad: f64,
    /// Minimum redesigned load-angle margin over the load window, rad.
    pub redesigned_min_margin_rad: f64,
    /// Fraction of evaluated samples whose redesigned margin is no worse than
    /// the stale margin.
    pub redesigned_not_worse_fraction: f64,
    /// Terminal/insertion residual norm supplied by the design pass, m.
    pub insertion_residual_norm_m: f64,
    /// Accepted terminal/insertion residual limit, m.
    pub insertion_residual_limit_m: f64,
}

/// One sample used to fit or validate the DOL load-indicator surrogate.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LoadIndicatorSample {
    /// Dynamic pressure at the load station, Pa.
    pub dynamic_pressure_pa: f64,
    /// Angle of attack at the station, rad.
    pub alpha_rad: f64,
    /// Sideslip at the station, rad.
    pub beta_rad: f64,
    /// Representative gimbal angle or control-margin proxy, rad.
    pub gimbal_angle_rad: f64,
    /// Full-model station bending moment target, N*m.
    pub station_moment_n_m: f64,
}

/// Reduced DOL load indicators for one sample.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LoadIndicatorVector {
    /// q-alpha indicator, Pa*rad.
    pub q_alpha_pa_rad: f64,
    /// q-beta indicator, Pa*rad.
    pub q_beta_pa_rad: f64,
    /// Gimbal indicator, rad.
    pub gimbal_angle_rad: f64,
}

/// Inputs to the deterministic DOL load-indicator fit pipeline.
#[derive(Clone, Debug, PartialEq)]
pub struct LoadIndicatorFitInput<'a> {
    /// Calibration matrix drawn from the full in-repo loads recovery path.
    pub calibration_samples: &'a [LoadIndicatorSample],
    /// Held-out matrix used only for validation.
    pub heldout_samples: &'a [LoadIndicatorSample],
    /// Expected upstream deck hash from the checked-in fit deck.
    pub expected_deck_hash: String,
    /// Actual upstream deck hash supplied by the caller.
    pub actual_deck_hash: String,
    /// Minimum accepted Pearson correlation on held-out samples.
    pub min_correlation: f64,
    /// Maximum accepted unconservative absolute-moment error, N*m.
    pub max_unconservative_error_n_m: f64,
}

/// Fitted reduced DOL load-indicator surrogate.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LoadIndicatorSurrogate {
    /// Surrogate schema version.
    pub schema_version: u16,
    /// Upstream deck hash guarded by the drift check.
    pub deck_hash: String,
    /// Fit intercept, N*m.
    pub intercept_n_m: f64,
    /// q-alpha coefficient, N*m/(Pa*rad).
    pub q_alpha_n_m_per_pa_rad: f64,
    /// q-beta coefficient, N*m/(Pa*rad).
    pub q_beta_n_m_per_pa_rad: f64,
    /// Gimbal coefficient, N*m/rad.
    pub gimbal_n_m_per_rad: f64,
    /// Root-mean-square calibration residual, N*m.
    pub rms_residual_n_m: f64,
    /// Number of calibration samples used by the fit.
    pub calibration_sample_count: usize,
}

/// Held-out validation evidence for a fitted DOL load-indicator surrogate.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LoadIndicatorValidationReport {
    /// Number of held-out samples.
    pub heldout_sample_count: usize,
    /// Pearson correlation between surrogate and full-model moments.
    pub correlation: f64,
    /// Maximum absolute moment error over held-out samples, N*m.
    pub max_abs_error_n_m: f64,
    /// Maximum unconservative error in absolute moment, N*m.
    pub max_unconservative_error_n_m: f64,
    /// Required minimum correlation.
    pub min_required_correlation: f64,
    /// Accepted maximum unconservative error, N*m.
    pub max_allowed_unconservative_error_n_m: f64,
    /// Whether the supplied deck hash matched the expected hash.
    pub drift_gate_passed: bool,
}

/// Result of fitting and validating a DOL load-indicator surrogate.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LoadIndicatorFitResult {
    /// Fitted surrogate coefficients.
    pub surrogate: LoadIndicatorSurrogate,
    /// Held-out validation evidence.
    pub validation: LoadIndicatorValidationReport,
}

/// Band-wise wind-persistence statistics used by WP-19.5 knockdowns.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DolPersistenceDeck {
    /// Persistence deck schema version.
    pub schema_version: u16,
    /// Provenance or deck hash used by drift gates.
    pub deck_hash: String,
    /// Human-readable provenance label.
    pub provenance: String,
    /// Altitude-banded persistence statistics.
    pub bands: Vec<DolPersistenceBand>,
}

impl DolPersistenceDeck {
    /// Validate deck shape and monotone altitude bands.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the deck is malformed.
    pub fn validate(&self) -> Result<(), TrajoptError> {
        validate_persistence_deck(self)
    }

    /// Return the wind-change standard deviation for one altitude and lead
    /// time.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the deck, altitude, or lead time is
    /// invalid.
    pub fn wind_sigma_m_s(&self, altitude_m: f64, lead_time_s: f64) -> Result<f64, TrajoptError> {
        self.validate()?;
        validate_nonnegative_finite(
            lead_time_s,
            "DOL persistence lead time must be finite and non-negative",
        )?;
        let Some(band) = persistence_band_for_altitude(self, altitude_m) else {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL persistence altitude is outside the deck bands",
            });
        };
        Ok(band.wind_sigma_m_s(lead_time_s))
    }
}

/// One altitude band in a DOL wind-persistence deck.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DolPersistenceBand {
    /// Lower band altitude, m.
    pub min_altitude_m: f64,
    /// Upper band altitude, m.
    pub max_altitude_m: f64,
    /// Wind-change 1-sigma at zero lead time, m/s.
    pub sigma_wind_0_m_s: f64,
    /// Square-root lead-time growth coefficient, m/s/sqrt(s).
    pub sigma_wind_sqrt_lead_m_s_per_sqrt_s: f64,
}

impl DolPersistenceBand {
    /// Compute wind-change 1-sigma at the supplied lead time.
    #[must_use]
    pub fn wind_sigma_m_s(self, lead_time_s: f64) -> f64 {
        self.sigma_wind_0_m_s + self.sigma_wind_sqrt_lead_m_s_per_sqrt_s * lead_time_s.sqrt()
    }
}

/// One indicator to knock down for wind persistence and dispersions.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DolPersistenceIndicatorInput {
    /// Stable indicator name.
    pub name: String,
    /// Indicator altitude used to select the persistence band, m.
    pub altitude_m: f64,
    /// Nominal indicator margin before statistical knockdowns, rad.
    pub nominal_margin_rad: f64,
    /// Local sensitivity from wind-change magnitude to margin angle, rad/(m/s).
    pub wind_sensitivity_rad_per_m_s: f64,
    /// Gust 1-sigma allowance at the indicator, m/s.
    pub gust_sigma_m_s: f64,
    /// System dispersion 1-sigma already expressed in margin angle, rad.
    pub system_sigma_rad: f64,
}

/// Inputs for deterministic DOL persistence knockdown evaluation.
#[derive(Clone, Debug, PartialEq)]
pub struct DolPersistenceKnockdownInput<'a> {
    /// Persistence statistics deck.
    pub deck: &'a DolPersistenceDeck,
    /// Lead time between design profile and launch commit, s.
    pub lead_time_s: f64,
    /// Indicator margins to knock down.
    pub indicators: &'a [DolPersistenceIndicatorInput],
    /// Required post-knockdown margin, rad.
    pub min_required_margin_rad: f64,
}

/// Knockdown result for one DOL indicator.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DolPersistenceIndicatorReport {
    /// Stable indicator name.
    pub name: String,
    /// Indicator altitude used to select the persistence band, m.
    pub altitude_m: f64,
    /// Nominal indicator margin before statistical knockdowns, rad.
    pub nominal_margin_rad: f64,
    /// Wind-change 1-sigma from the selected deck band, m/s.
    pub persistence_sigma_m_s: f64,
    /// Persistence contribution to margin knockdown, rad.
    pub persistence_knockdown_rad: f64,
    /// Gust contribution to margin knockdown, rad.
    pub gust_knockdown_rad: f64,
    /// System contribution to margin knockdown, rad.
    pub system_knockdown_rad: f64,
    /// RSS-combined total knockdown, rad.
    pub total_knockdown_rad: f64,
    /// Margin after subtracting the RSS knockdown, rad.
    pub knocked_down_margin_rad: f64,
    /// Whether the knocked-down margin meets the declared threshold.
    pub passed: bool,
}

/// Deterministic DOL persistence knockdown report.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DolPersistenceKnockdownReport {
    /// Persistence deck schema version.
    pub schema_version: u16,
    /// Provenance or deck hash used by drift gates.
    pub deck_hash: String,
    /// Lead time between design profile and launch commit, s.
    pub lead_time_s: f64,
    /// Required post-knockdown margin, rad.
    pub min_required_margin_rad: f64,
    /// Per-indicator knockdown evidence.
    pub indicators: Vec<DolPersistenceIndicatorReport>,
    /// Minimum knocked-down margin across all indicators, rad.
    pub min_knocked_down_margin_rad: f64,
    /// Latched go/no-go verdict for the knocked-down margins.
    pub verdict: DolReportVerdict,
    /// First binding failed indicator, if any.
    pub binding_indicator: Option<String>,
}

/// One synthetic wind-change sample used to validate a persistence deck.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DolPersistenceTruthSample {
    /// Sample altitude, m.
    pub altitude_m: f64,
    /// Lead time represented by this wind-change sample, s.
    pub lead_time_s: f64,
    /// Realized scalar wind-change magnitude, m/s.
    pub delta_wind_m_s: f64,
}

/// Inputs for synthetic persistence-deck validation.
#[derive(Clone, Debug, PartialEq)]
pub struct DolPersistenceTruthValidationInput<'a> {
    /// Persistence statistics deck.
    pub deck: &'a DolPersistenceDeck,
    /// Synthetic truth wind-change samples.
    pub samples: &'a [DolPersistenceTruthSample],
    /// Accepted absolute sample / sigma ratio.
    pub sigma_multiplier: f64,
}

/// Synthetic persistence-deck validation evidence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DolPersistenceTruthValidationReport {
    /// Number of checked samples.
    pub sample_count: usize,
    /// Accepted absolute sample / sigma ratio.
    pub sigma_multiplier: f64,
    /// Maximum observed absolute sample / sigma ratio.
    pub max_abs_sigma_ratio: f64,
    /// Whether every sample stayed inside the declared sigma envelope.
    pub passed: bool,
}

/// One trajectory sample used by deterministic DOL launch-COLA screening.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DolColaTrajectorySample {
    /// Time since the slot's launch epoch, s.
    pub time_s: f64,
    /// Planned vehicle ECI position, m.
    pub position_eci_m: [f64; 3],
}

/// One launch-window slot to screen.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DolColaWindowSlot {
    /// Stable slot id.
    pub slot_id: String,
    /// Slot launch epoch relative to the catalog epoch convention, s.
    pub launch_time_s: f64,
    /// Planned vehicle trajectory samples relative to launch.
    pub trajectory: Vec<DolColaTrajectorySample>,
}

/// One declared synthetic catalog object state.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DolColaCatalogObject {
    /// Stable catalog object id.
    pub object_id: String,
    /// State epoch, s.
    pub epoch_s: f64,
    /// ECI position at `epoch_s`, m.
    pub position_eci_m: [f64; 3],
    /// Constant ECI velocity used by the synthetic screening model, m/s.
    pub velocity_eci_m_s: [f64; 3],
    /// Object hard-body or screening radius, m.
    pub hard_body_radius_m: f64,
}

/// Inputs for deterministic DOL launch-COLA screening.
#[derive(Clone, Debug, PartialEq)]
pub struct DolColaScreeningInput<'a> {
    /// Launch-window slots to screen.
    pub slots: &'a [DolColaWindowSlot],
    /// Declared catalog objects.
    pub catalog_objects: &'a [DolColaCatalogObject],
    /// Vehicle keepout radius applied to every encounter, m.
    pub vehicle_keepout_radius_m: f64,
}

/// Closest-approach evidence for one slot/object pair.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DolColaEncounterReport {
    /// Slot id.
    pub slot_id: String,
    /// Catalog object id.
    pub object_id: String,
    /// Slot-relative trajectory time at closest approach, s.
    pub trajectory_time_s: f64,
    /// Absolute time at closest approach, s.
    pub closest_time_s: f64,
    /// Closest approach distance, m.
    pub miss_distance_m: f64,
    /// Vehicle keepout radius, m.
    pub vehicle_keepout_radius_m: f64,
    /// Object hard-body or screening radius, m.
    pub object_radius_m: f64,
    /// Miss distance minus combined keepout, m.
    pub clearance_m: f64,
    /// Whether this encounter blocks the slot.
    pub blocked: bool,
}

/// Per-slot launch-COLA screening evidence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DolColaSlotReport {
    /// Slot id.
    pub slot_id: String,
    /// Slot launch epoch, s.
    pub launch_time_s: f64,
    /// Closest per-object encounters for the slot.
    pub encounters: Vec<DolColaEncounterReport>,
    /// Minimum clearance across objects, m.
    pub min_clearance_m: f64,
    /// Whether any object blocks this slot.
    pub blocked: bool,
    /// First binding object, if blocked.
    pub binding_object: Option<String>,
}

/// Deterministic launch-COLA screening report.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DolColaScreeningReport {
    /// Launch-COLA screening schema version.
    pub schema_version: u16,
    /// Vehicle keepout radius, m.
    pub vehicle_keepout_radius_m: f64,
    /// Per-slot screening rows.
    pub slots: Vec<DolColaSlotReport>,
    /// Number of blocked launch-window slots.
    pub blocked_slot_count: usize,
    /// Latched go/no-go verdict for the slot table.
    pub verdict: DolReportVerdict,
    /// First binding slot, if any.
    pub binding_slot: Option<String>,
    /// First binding catalog object, if any.
    pub binding_object: Option<String>,
}

/// Latched DOL report-card verdict.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum DolReportVerdict {
    /// All declared checks passed.
    Go,
    /// At least one declared check failed; this verdict latches.
    NoGo,
}

impl DolReportVerdict {
    /// Return the monotone latched verdict.
    #[must_use]
    pub const fn latch(self, next: Self) -> Self {
        match (self, next) {
            (Self::NoGo, _) | (_, Self::NoGo) => Self::NoGo,
            (Self::Go, Self::Go) => Self::Go,
        }
    }

    const fn as_str(self) -> &'static str {
        match self {
            Self::Go => "go",
            Self::NoGo => "no_go",
        }
    }
}

/// One design-vs-verification indicator comparison.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DolVerificationIndicator {
    /// Stable indicator name.
    pub name: String,
    /// Design/surrogate indicator value.
    pub design_value: f64,
    /// Value recovered by the verification pass.
    pub verified_value: f64,
    /// Accepted absolute design-vs-verify error band.
    pub allowed_abs_error: f64,
}

/// Report row for one verified indicator.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DolVerificationIndicatorReport {
    /// Stable indicator name.
    pub name: String,
    /// Design/surrogate indicator value.
    pub design_value: f64,
    /// Value recovered by the verification pass.
    pub verified_value: f64,
    /// Absolute verification error.
    pub abs_error: f64,
    /// Accepted absolute error band.
    pub allowed_abs_error: f64,
    /// Whether this indicator passed its consistency band.
    pub passed: bool,
}

/// Inputs to build a deterministic DOL go/no-go report card.
#[derive(Clone, Debug, PartialEq)]
pub struct DolReportCardInput<'a> {
    /// Stable report identifier.
    pub report_id: String,
    /// Verification-source provenance, such as runner/SIL mode.
    pub verification_provenance: String,
    /// DOL redesign artifact being verified.
    pub update_artifact: &'a ILoadUpdateArtifact,
    /// Indicator comparisons recovered from the verification pass.
    pub indicators: &'a [DolVerificationIndicator],
    /// Minimum accepted redesigned load-margin, rad.
    pub min_required_margin_rad: f64,
    /// Minimum accepted not-worse sample fraction.
    pub min_not_worse_fraction: f64,
}

/// Deterministic DOL go/no-go report card.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DolReportCard {
    /// Report-card schema version.
    pub schema_version: u16,
    /// Stable report identifier.
    pub report_id: String,
    /// Verification-source provenance.
    pub verification_provenance: String,
    /// Design-profile digest inherited from the update artifact.
    pub design_profile_digest: String,
    /// Solver provenance inherited from the update artifact.
    pub solver_provenance: String,
    /// SHA-256 digest of the encoded updated I-load payload.
    pub iload_sha256_hex: String,
    /// Load-window sample count inherited from the update artifact.
    pub load_window_sample_count: usize,
    /// Redesigned minimum load-margin, rad.
    pub redesigned_min_margin_rad: f64,
    /// Required minimum load-margin, rad.
    pub min_required_margin_rad: f64,
    /// Redesigned not-worse sample fraction.
    pub redesigned_not_worse_fraction: f64,
    /// Required not-worse sample fraction.
    pub min_not_worse_fraction: f64,
    /// Insertion residual norm, m.
    pub insertion_residual_norm_m: f64,
    /// Insertion residual limit, m.
    pub insertion_residual_limit_m: f64,
    /// Per-indicator consistency results.
    pub indicators: Vec<DolVerificationIndicatorReport>,
    /// Latched go/no-go verdict.
    pub verdict: DolReportVerdict,
    /// First binding failed indicator or check.
    pub binding_indicator: Option<String>,
}

/// Inputs to build a fail-closed DOL launch-commit readiness artifact.
#[derive(Clone, Debug, PartialEq)]
pub struct DolCommitReadinessInput<'a> {
    /// Stable readiness artifact identifier.
    pub readiness_id: String,
    /// DOL I-load verification report card.
    pub report_card: &'a DolReportCard,
    /// Wind-persistence and dispersion knockdown report.
    pub persistence: &'a DolPersistenceKnockdownReport,
    /// Launch-COLA slot screening report.
    pub cola: &'a DolColaScreeningReport,
}

/// One gate row in a DOL launch-commit readiness artifact.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DolCommitGateReport {
    /// Gate name.
    pub gate: String,
    /// Gate-local artifact id or provenance key.
    pub artifact_id: String,
    /// Latched gate verdict.
    pub verdict: DolReportVerdict,
    /// First binding check for the gate, if any.
    pub binding_detail: Option<String>,
    /// Name of the summary value.
    pub summary_key: String,
    /// Gate summary value.
    pub summary_value: f64,
}

/// Deterministic DOL launch-commit readiness artifact.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DolCommitReadinessReport {
    /// Commit-readiness schema version.
    pub schema_version: u16,
    /// Stable readiness artifact identifier.
    pub readiness_id: String,
    /// DOL report-card id consumed by this artifact.
    pub report_card_id: String,
    /// Verification-source provenance inherited from the report card.
    pub verification_provenance: String,
    /// Design-profile digest inherited from the report card.
    pub design_profile_digest: String,
    /// SHA-256 digest of the encoded updated I-load payload.
    pub iload_sha256_hex: String,
    /// Ordered gate evidence rows.
    pub gates: Vec<DolCommitGateReport>,
    /// Minimum post-knockdown persistence margin, rad.
    pub min_persistence_margin_rad: f64,
    /// Number of launch-COLA slots blocked by declared catalog objects.
    pub blocked_cola_slot_count: usize,
    /// Overall latched launch-commit verdict.
    pub verdict: DolReportVerdict,
    /// First binding gate, if any.
    pub binding_gate: Option<String>,
    /// First binding gate-local detail, if any.
    pub binding_detail: Option<String>,
}

/// Redesign the pitch/yaw I-load bias tables for one spliced wind profile.
///
/// # Errors
///
/// Returns [`TrajoptError`] when inputs are invalid, no samples fall inside the
/// load window, the insertion residual exceeds its declared limit, or the
/// produced I-load payload fails validation.
pub fn design_dol_iload_update(input: DolRedesignInput<'_>) -> Result<ILoadUpdate, TrajoptError> {
    validate_input(&input)?;
    let samples = load_window_samples(input.spliced_wind, input.load_window)?;
    let mut induced_alpha_sum = 0.0_f64;
    let mut induced_beta_sum = 0.0_f64;
    let mut stale_min_margin_rad = f64::INFINITY;

    for sample in &samples {
        let induced = induced_angles(*sample, input.airspeed_m_s);
        induced_alpha_sum += induced.alpha_rad;
        induced_beta_sum += induced.beta_rad;
        let stale_alpha = input.stale_pitch_bias_rad + induced.alpha_rad;
        let stale_beta = input.stale_yaw_bias_rad + induced.beta_rad;
        stale_min_margin_rad = stale_min_margin_rad.min(load_margin_rad(
            stale_alpha,
            stale_beta,
            input.alpha_reference_rad,
            input.beta_reference_rad,
            input.load_limit_angle_rad,
        ));
    }

    let count = samples.len() as f64;
    let mean_induced_alpha_rad = induced_alpha_sum / count;
    let mean_induced_beta_rad = induced_beta_sum / count;
    let corrected_pitch_bias_rad = input.alpha_reference_rad - mean_induced_alpha_rad;
    let corrected_yaw_bias_rad = input.beta_reference_rad - mean_induced_beta_rad;

    let mut redesigned_min_margin_rad = f64::INFINITY;
    let mut not_worse_count = 0usize;
    for sample in &samples {
        let induced = induced_angles(*sample, input.airspeed_m_s);
        let stale_alpha = input.stale_pitch_bias_rad + induced.alpha_rad;
        let stale_beta = input.stale_yaw_bias_rad + induced.beta_rad;
        let stale_margin = load_margin_rad(
            stale_alpha,
            stale_beta,
            input.alpha_reference_rad,
            input.beta_reference_rad,
            input.load_limit_angle_rad,
        );
        let redesigned_alpha = corrected_pitch_bias_rad + induced.alpha_rad;
        let redesigned_beta = corrected_yaw_bias_rad + induced.beta_rad;
        let redesigned_margin = load_margin_rad(
            redesigned_alpha,
            redesigned_beta,
            input.alpha_reference_rad,
            input.beta_reference_rad,
            input.load_limit_angle_rad,
        );
        if redesigned_margin >= stale_margin {
            not_worse_count += 1;
        }
        redesigned_min_margin_rad = redesigned_min_margin_rad.min(redesigned_margin);
    }

    let breakpoints = pseudo_time_breakpoints(&samples, input.airspeed_m_s);
    let reference_profile = reference_profile(&samples, input.airspeed_m_s);
    let pitch_values = vec![corrected_pitch_bias_rad; breakpoints.len()];
    let yaw_values = vec![corrected_yaw_bias_rad; breakpoints.len()];
    let payload = ILoadPayload {
        header: ILoadHeader::new_with_objective(
            &input.terminal_condition,
            ILoadObjectiveClass::DayOfLaunchRecenter,
            input.synthesis_seed,
            input.producer,
        ),
        metadata: SynthesisMetadata {
            scenario_digest: input.scenario_digest,
            source_revision: input.source_revision,
            method: "dol_spliced_wind_recenter".to_owned(),
        },
        event_bindings_postcard: Vec::new(),
        gain_tables: vec![
            GainTable {
                name: "dol_pitch_bias_rad".to_owned(),
                axis: GainAxis::Time,
                breakpoints: breakpoints.clone(),
                values: pitch_values,
            },
            GainTable {
                name: "dol_yaw_bias_rad".to_owned(),
                axis: GainAxis::Time,
                breakpoints,
                values: yaw_values,
            },
        ],
        reference_profile,
        optimality_certificate: None,
    };
    payload.validate()?;

    Ok(ILoadUpdate {
        artifact: ILoadUpdateArtifact {
            schema_version: DOL_UPDATE_SCHEMA_VERSION,
            terminal_condition_kind: (&input.terminal_condition).into(),
            design_profile_digest: input.design_profile_digest,
            solver_provenance: input.solver_provenance,
            iload_sha256_hex: String::new(),
            load_window_sample_count: samples.len(),
            stale_pitch_bias_rad: input.stale_pitch_bias_rad,
            stale_yaw_bias_rad: input.stale_yaw_bias_rad,
            corrected_pitch_bias_rad,
            corrected_yaw_bias_rad,
            stale_min_margin_rad,
            redesigned_min_margin_rad,
            redesigned_not_worse_fraction: not_worse_count as f64 / samples.len() as f64,
            insertion_residual_norm_m: input.insertion_residual_norm_m,
            insertion_residual_limit_m: input.insertion_residual_limit_m,
        },
        payload,
    })
}

/// Compute reduced q-alpha/q-beta/gimbal load indicators for one sample.
#[must_use]
pub fn load_indicator_vector(sample: LoadIndicatorSample) -> LoadIndicatorVector {
    LoadIndicatorVector {
        q_alpha_pa_rad: sample.dynamic_pressure_pa * sample.alpha_rad,
        q_beta_pa_rad: sample.dynamic_pressure_pa * sample.beta_rad,
        gimbal_angle_rad: sample.gimbal_angle_rad,
    }
}

/// Evaluate a fitted DOL load-indicator surrogate for one sample.
#[must_use]
pub fn evaluate_load_indicator_surrogate(
    surrogate: &LoadIndicatorSurrogate,
    sample: LoadIndicatorSample,
) -> f64 {
    let indicators = load_indicator_vector(sample);
    surrogate.intercept_n_m
        + surrogate.q_alpha_n_m_per_pa_rad * indicators.q_alpha_pa_rad
        + surrogate.q_beta_n_m_per_pa_rad * indicators.q_beta_pa_rad
        + surrogate.gimbal_n_m_per_rad * indicators.gimbal_angle_rad
}

/// Fit and validate a deterministic DOL load-indicator surrogate.
///
/// # Errors
///
/// Returns [`TrajoptError`] when the sample matrices are malformed, the
/// upstream deck hash fails the drift gate, the least-squares system is
/// singular, or held-out correlation / unconservative-error thresholds fail.
pub fn fit_load_indicator_surrogate(
    input: LoadIndicatorFitInput<'_>,
) -> Result<LoadIndicatorFitResult, TrajoptError> {
    validate_indicator_fit_input(&input)?;
    let coefficients = fit_indicator_coefficients(input.calibration_samples)?;
    let surrogate = LoadIndicatorSurrogate {
        schema_version: DOL_LOAD_INDICATOR_SURROGATE_SCHEMA_VERSION,
        deck_hash: input.actual_deck_hash,
        intercept_n_m: coefficients[0],
        q_alpha_n_m_per_pa_rad: coefficients[1],
        q_beta_n_m_per_pa_rad: coefficients[2],
        gimbal_n_m_per_rad: coefficients[3],
        rms_residual_n_m: calibration_rms(input.calibration_samples, coefficients),
        calibration_sample_count: input.calibration_samples.len(),
    };
    let validation = validate_indicator_surrogate(
        &surrogate,
        input.heldout_samples,
        input.min_correlation,
        input.max_unconservative_error_n_m,
    )?;
    Ok(LoadIndicatorFitResult {
        surrogate,
        validation,
    })
}

/// Build the deterministic DOL go/no-go report card.
///
/// # Errors
///
/// Returns [`TrajoptError`] when provenance, thresholds, or indicator values are
/// malformed.
pub fn build_dol_report_card(input: DolReportCardInput<'_>) -> Result<DolReportCard, TrajoptError> {
    validate_report_card_input(&input)?;
    let mut verdict = DolReportVerdict::Go;
    let mut binding_indicator = None;

    if input.update_artifact.redesigned_min_margin_rad < input.min_required_margin_rad {
        verdict = verdict.latch(DolReportVerdict::NoGo);
        binding_indicator = Some("redesigned_min_margin_rad".to_owned());
    }
    if input.update_artifact.redesigned_not_worse_fraction < input.min_not_worse_fraction {
        verdict = verdict.latch(DolReportVerdict::NoGo);
        binding_indicator.get_or_insert_with(|| "redesigned_not_worse_fraction".to_owned());
    }
    if input.update_artifact.insertion_residual_norm_m
        > input.update_artifact.insertion_residual_limit_m
    {
        verdict = verdict.latch(DolReportVerdict::NoGo);
        binding_indicator.get_or_insert_with(|| "insertion_residual_norm_m".to_owned());
    }

    let mut reports = Vec::with_capacity(input.indicators.len());
    for indicator in input.indicators {
        let abs_error = (indicator.verified_value - indicator.design_value).abs();
        let passed = abs_error <= indicator.allowed_abs_error;
        if !passed {
            verdict = verdict.latch(DolReportVerdict::NoGo);
            binding_indicator.get_or_insert_with(|| indicator.name.clone());
        }
        reports.push(DolVerificationIndicatorReport {
            name: indicator.name.clone(),
            design_value: indicator.design_value,
            verified_value: indicator.verified_value,
            abs_error,
            allowed_abs_error: indicator.allowed_abs_error,
            passed,
        });
    }

    Ok(DolReportCard {
        schema_version: DOL_REPORT_CARD_SCHEMA_VERSION,
        report_id: input.report_id,
        verification_provenance: input.verification_provenance,
        design_profile_digest: input.update_artifact.design_profile_digest.clone(),
        solver_provenance: input.update_artifact.solver_provenance.clone(),
        iload_sha256_hex: input.update_artifact.iload_sha256_hex.clone(),
        load_window_sample_count: input.update_artifact.load_window_sample_count,
        redesigned_min_margin_rad: input.update_artifact.redesigned_min_margin_rad,
        min_required_margin_rad: input.min_required_margin_rad,
        redesigned_not_worse_fraction: input.update_artifact.redesigned_not_worse_fraction,
        min_not_worse_fraction: input.min_not_worse_fraction,
        insertion_residual_norm_m: input.update_artifact.insertion_residual_norm_m,
        insertion_residual_limit_m: input.update_artifact.insertion_residual_limit_m,
        indicators: reports,
        verdict,
        binding_indicator,
    })
}

/// Render a byte-stable CSV report-card artifact.
#[must_use]
pub fn dol_report_card_csv(report: &DolReportCard) -> String {
    let mut output = String::new();
    let _ = writeln!(output, "openbmp.dol_report_card,{}", report.schema_version);
    let _ = writeln!(output, "report_id,{}", report.report_id);
    let _ = writeln!(
        output,
        "verification_provenance,{}",
        report.verification_provenance
    );
    let _ = writeln!(
        output,
        "design_profile_digest,{}",
        report.design_profile_digest
    );
    let _ = writeln!(output, "solver_provenance,{}", report.solver_provenance);
    let _ = writeln!(output, "iload_sha256_hex,{}", report.iload_sha256_hex);
    let _ = writeln!(
        output,
        "load_window_sample_count,{}",
        report.load_window_sample_count
    );
    let _ = writeln!(
        output,
        "redesigned_min_margin_rad,{:.12e}",
        report.redesigned_min_margin_rad
    );
    let _ = writeln!(
        output,
        "min_required_margin_rad,{:.12e}",
        report.min_required_margin_rad
    );
    let _ = writeln!(
        output,
        "redesigned_not_worse_fraction,{:.12e}",
        report.redesigned_not_worse_fraction
    );
    let _ = writeln!(
        output,
        "min_not_worse_fraction,{:.12e}",
        report.min_not_worse_fraction
    );
    let _ = writeln!(
        output,
        "insertion_residual_norm_m,{:.12e}",
        report.insertion_residual_norm_m
    );
    let _ = writeln!(
        output,
        "insertion_residual_limit_m,{:.12e}",
        report.insertion_residual_limit_m
    );
    let binding = report.binding_indicator.as_deref().unwrap_or("");
    let _ = writeln!(output, "verdict,{},{}", report.verdict.as_str(), binding);
    let _ = writeln!(
        output,
        "indicator,design_value,verified_value,abs_error,allowed_abs_error,passed"
    );
    for indicator in &report.indicators {
        let _ = writeln!(
            output,
            "{},{:.12e},{:.12e},{:.12e},{:.12e},{}",
            indicator.name,
            indicator.design_value,
            indicator.verified_value,
            indicator.abs_error,
            indicator.allowed_abs_error,
            indicator.passed
        );
    }
    output
}

/// Build the fail-closed DOL launch-commit readiness artifact.
///
/// # Errors
///
/// Returns [`TrajoptError`] when any required gate report is malformed or has an
/// unsupported schema version.
pub fn build_dol_commit_readiness(
    input: DolCommitReadinessInput<'_>,
) -> Result<DolCommitReadinessReport, TrajoptError> {
    validate_commit_readiness_input(&input)?;

    let cola_binding_detail = cola_binding_detail(input.cola);
    let gates = vec![
        DolCommitGateReport {
            gate: "report_card".to_owned(),
            artifact_id: input.report_card.report_id.clone(),
            verdict: input.report_card.verdict,
            binding_detail: input.report_card.binding_indicator.clone(),
            summary_key: "redesigned_min_margin_rad".to_owned(),
            summary_value: input.report_card.redesigned_min_margin_rad,
        },
        DolCommitGateReport {
            gate: "persistence".to_owned(),
            artifact_id: input.persistence.deck_hash.clone(),
            verdict: input.persistence.verdict,
            binding_detail: input.persistence.binding_indicator.clone(),
            summary_key: "min_knocked_down_margin_rad".to_owned(),
            summary_value: input.persistence.min_knocked_down_margin_rad,
        },
        DolCommitGateReport {
            gate: "cola".to_owned(),
            artifact_id: "launch_cola_screening".to_owned(),
            verdict: input.cola.verdict,
            binding_detail: cola_binding_detail,
            summary_key: "blocked_slot_count".to_owned(),
            summary_value: input.cola.blocked_slot_count as f64,
        },
    ];

    let mut verdict = DolReportVerdict::Go;
    let mut binding_gate = None;
    let mut binding_detail = None;
    for gate in &gates {
        verdict = verdict.latch(gate.verdict);
        if gate.verdict == DolReportVerdict::NoGo && binding_gate.is_none() {
            binding_gate = Some(gate.gate.clone());
            binding_detail = gate.binding_detail.clone();
        }
    }

    Ok(DolCommitReadinessReport {
        schema_version: DOL_COMMIT_READINESS_SCHEMA_VERSION,
        readiness_id: input.readiness_id,
        report_card_id: input.report_card.report_id.clone(),
        verification_provenance: input.report_card.verification_provenance.clone(),
        design_profile_digest: input.report_card.design_profile_digest.clone(),
        iload_sha256_hex: input.report_card.iload_sha256_hex.clone(),
        gates,
        min_persistence_margin_rad: input.persistence.min_knocked_down_margin_rad,
        blocked_cola_slot_count: input.cola.blocked_slot_count,
        verdict,
        binding_gate,
        binding_detail,
    })
}

/// Render a byte-stable DOL launch-commit readiness CSV artifact.
#[must_use]
pub fn dol_commit_readiness_csv(report: &DolCommitReadinessReport) -> String {
    let mut output = String::new();
    let _ = writeln!(
        output,
        "openbmp.dol_commit_readiness,{}",
        report.schema_version
    );
    let _ = writeln!(output, "readiness_id,{}", report.readiness_id);
    let _ = writeln!(output, "report_card_id,{}", report.report_card_id);
    let _ = writeln!(
        output,
        "verification_provenance,{}",
        report.verification_provenance
    );
    let _ = writeln!(
        output,
        "design_profile_digest,{}",
        report.design_profile_digest
    );
    let _ = writeln!(output, "iload_sha256_hex,{}", report.iload_sha256_hex);
    let _ = writeln!(
        output,
        "min_persistence_margin_rad,{:.12e}",
        report.min_persistence_margin_rad
    );
    let _ = writeln!(
        output,
        "blocked_cola_slot_count,{}",
        report.blocked_cola_slot_count
    );
    let binding_gate = report.binding_gate.as_deref().unwrap_or("");
    let binding_detail = report.binding_detail.as_deref().unwrap_or("");
    let _ = writeln!(
        output,
        "verdict,{},{},{}",
        report.verdict.as_str(),
        binding_gate,
        binding_detail
    );
    let _ = writeln!(
        output,
        "gate,artifact_id,verdict,binding_detail,summary_key,summary_value"
    );
    for gate in &report.gates {
        let binding_detail = gate.binding_detail.as_deref().unwrap_or("");
        let _ = writeln!(
            output,
            "{},{},{},{},{},{:.12e}",
            gate.gate,
            gate.artifact_id,
            gate.verdict.as_str(),
            binding_detail,
            gate.summary_key,
            gate.summary_value
        );
    }
    output
}

/// Evaluate DOL wind-persistence and dispersion knockdowns.
///
/// # Errors
///
/// Returns [`TrajoptError`] when the deck, lead time, thresholds, or indicator
/// inputs are malformed.
pub fn evaluate_persistence_knockdowns(
    input: DolPersistenceKnockdownInput<'_>,
) -> Result<DolPersistenceKnockdownReport, TrajoptError> {
    validate_persistence_knockdown_input(&input)?;
    let mut reports = Vec::with_capacity(input.indicators.len());
    let mut min_knocked_down_margin_rad = f64::INFINITY;
    let mut verdict = DolReportVerdict::Go;
    let mut binding_indicator = None;

    for indicator in input.indicators {
        let persistence_sigma_m_s = input
            .deck
            .wind_sigma_m_s(indicator.altitude_m, input.lead_time_s)?;
        let sensitivity = indicator.wind_sensitivity_rad_per_m_s.abs();
        let persistence_knockdown_rad = sensitivity * persistence_sigma_m_s;
        let gust_knockdown_rad = sensitivity * indicator.gust_sigma_m_s;
        let system_knockdown_rad = indicator.system_sigma_rad;
        let total_knockdown_rad = rss3(
            persistence_knockdown_rad,
            gust_knockdown_rad,
            system_knockdown_rad,
        );
        let knocked_down_margin_rad = indicator.nominal_margin_rad - total_knockdown_rad;
        let passed = knocked_down_margin_rad >= input.min_required_margin_rad;
        if !passed {
            verdict = verdict.latch(DolReportVerdict::NoGo);
            binding_indicator.get_or_insert_with(|| indicator.name.clone());
        }
        min_knocked_down_margin_rad = min_knocked_down_margin_rad.min(knocked_down_margin_rad);
        reports.push(DolPersistenceIndicatorReport {
            name: indicator.name.clone(),
            altitude_m: indicator.altitude_m,
            nominal_margin_rad: indicator.nominal_margin_rad,
            persistence_sigma_m_s,
            persistence_knockdown_rad,
            gust_knockdown_rad,
            system_knockdown_rad,
            total_knockdown_rad,
            knocked_down_margin_rad,
            passed,
        });
    }

    Ok(DolPersistenceKnockdownReport {
        schema_version: DOL_PERSISTENCE_KNOCKDOWN_SCHEMA_VERSION,
        deck_hash: input.deck.deck_hash.clone(),
        lead_time_s: input.lead_time_s,
        min_required_margin_rad: input.min_required_margin_rad,
        indicators: reports,
        min_knocked_down_margin_rad,
        verdict,
        binding_indicator,
    })
}

/// Validate synthetic wind-change samples against a persistence deck.
///
/// # Errors
///
/// Returns [`TrajoptError`] when the deck, sample set, or sigma multiplier is
/// malformed. Out-of-envelope samples return a report with `passed = false`.
pub fn validate_persistence_truth_samples(
    input: DolPersistenceTruthValidationInput<'_>,
) -> Result<DolPersistenceTruthValidationReport, TrajoptError> {
    input.deck.validate()?;
    if input.samples.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL persistence truth validation requires at least one sample",
        });
    }
    if !input.sigma_multiplier.is_finite() || input.sigma_multiplier <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL persistence truth sigma multiplier must be positive",
        });
    }
    let mut max_abs_sigma_ratio = 0.0_f64;
    for sample in input.samples {
        if !sample.altitude_m.is_finite()
            || !sample.lead_time_s.is_finite()
            || sample.lead_time_s < 0.0
            || !sample.delta_wind_m_s.is_finite()
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL persistence truth samples must be finite with non-negative lead time",
            });
        }
        let sigma = input
            .deck
            .wind_sigma_m_s(sample.altitude_m, sample.lead_time_s)?;
        if sigma <= f64::EPSILON {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL persistence truth sample selected a zero-sigma band",
            });
        }
        max_abs_sigma_ratio = max_abs_sigma_ratio.max(sample.delta_wind_m_s.abs() / sigma);
    }
    Ok(DolPersistenceTruthValidationReport {
        sample_count: input.samples.len(),
        sigma_multiplier: input.sigma_multiplier,
        max_abs_sigma_ratio,
        passed: max_abs_sigma_ratio <= input.sigma_multiplier,
    })
}

/// Screen launch-window slots against declared synthetic catalog states.
///
/// # Errors
///
/// Returns [`TrajoptError`] when slots, trajectories, catalog objects, or
/// keepout values are malformed.
pub fn screen_launch_cola(
    input: DolColaScreeningInput<'_>,
) -> Result<DolColaScreeningReport, TrajoptError> {
    validate_cola_input(&input)?;
    let mut slot_reports = Vec::with_capacity(input.slots.len());
    let mut blocked_slot_count = 0usize;
    let mut verdict = DolReportVerdict::Go;
    let mut binding_slot = None;
    let mut binding_object = None;

    for slot in input.slots {
        let mut encounters = Vec::with_capacity(input.catalog_objects.len());
        let mut min_clearance_m = f64::INFINITY;
        let mut slot_blocked = false;
        let mut slot_binding_object = None;

        for object in input.catalog_objects {
            let encounter =
                closest_encounter_for_slot(slot, object, input.vehicle_keepout_radius_m)?;
            if encounter.blocked {
                slot_blocked = true;
                slot_binding_object.get_or_insert_with(|| encounter.object_id.clone());
            }
            min_clearance_m = min_clearance_m.min(encounter.clearance_m);
            encounters.push(encounter);
        }

        if slot_blocked {
            blocked_slot_count += 1;
            verdict = verdict.latch(DolReportVerdict::NoGo);
            binding_slot.get_or_insert_with(|| slot.slot_id.clone());
            if binding_object.is_none() {
                binding_object = slot_binding_object.clone();
            }
        }

        slot_reports.push(DolColaSlotReport {
            slot_id: slot.slot_id.clone(),
            launch_time_s: slot.launch_time_s,
            encounters,
            min_clearance_m,
            blocked: slot_blocked,
            binding_object: slot_binding_object,
        });
    }

    Ok(DolColaScreeningReport {
        schema_version: DOL_COLA_SCREENING_SCHEMA_VERSION,
        vehicle_keepout_radius_m: input.vehicle_keepout_radius_m,
        slots: slot_reports,
        blocked_slot_count,
        verdict,
        binding_slot,
        binding_object,
    })
}

fn validate_input(input: &DolRedesignInput<'_>) -> Result<(), TrajoptError> {
    input
        .terminal_condition
        .validate()
        .map_err(|_| TrajoptError::InvalidPayload {
            reason: "DOL terminal condition is invalid",
        })?;
    input.load_window.validate()?;
    if input.producer.trim().is_empty()
        || input.scenario_digest.trim().is_empty()
        || input.source_revision.trim().is_empty()
        || input.design_profile_digest.trim().is_empty()
        || input.solver_provenance.trim().is_empty()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL provenance strings must be non-empty",
        });
    }
    for value in [
        input.airspeed_m_s,
        input.alpha_reference_rad,
        input.beta_reference_rad,
        input.stale_pitch_bias_rad,
        input.stale_yaw_bias_rad,
        input.load_limit_angle_rad,
        input.insertion_residual_norm_m,
        input.insertion_residual_limit_m,
    ] {
        if !value.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL numeric inputs must be finite",
            });
        }
    }
    if input.airspeed_m_s <= 0.0 || input.load_limit_angle_rad <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL airspeed and load-angle limit must be positive",
        });
    }
    if input.insertion_residual_norm_m < 0.0
        || input.insertion_residual_limit_m < 0.0
        || input.insertion_residual_norm_m > input.insertion_residual_limit_m
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL insertion residual must be non-negative and within limit",
        });
    }
    if input.spliced_wind.samples.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL spliced wind profile must be nonempty",
        });
    }
    Ok(())
}

fn load_window_samples(
    profile: &SplicedWindProfile,
    window: DolLoadWindow,
) -> Result<Vec<openbmp_physics::wind::SplicedWindSample>, TrajoptError> {
    let samples: Vec<_> = profile
        .samples
        .iter()
        .copied()
        .filter(|sample| window.contains(sample.altitude_m))
        .collect();
    if samples.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL load window contains no spliced wind samples",
        });
    }
    for sample in &samples {
        if !sample.altitude_m.is_finite()
            || !sample.wind_ned_m_s.vector.x.is_finite()
            || !sample.wind_ned_m_s.vector.y.is_finite()
            || !sample.wind_ned_m_s.vector.z.is_finite()
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL spliced wind samples must be finite",
            });
        }
    }
    if !samples
        .windows(2)
        .all(|window| window[0].altitude_m < window[1].altitude_m)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL spliced wind samples must be strictly ascending",
        });
    }
    Ok(samples)
}

fn validate_indicator_fit_input(input: &LoadIndicatorFitInput<'_>) -> Result<(), TrajoptError> {
    if input.expected_deck_hash.trim().is_empty() || input.actual_deck_hash.trim().is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL indicator deck hashes must be non-empty",
        });
    }
    if input.expected_deck_hash != input.actual_deck_hash {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL indicator fit deck drifted",
        });
    }
    if !input.min_correlation.is_finite()
        || input.min_correlation < 0.0
        || input.min_correlation > 1.0
        || !input.max_unconservative_error_n_m.is_finite()
        || input.max_unconservative_error_n_m < 0.0
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL indicator validation thresholds are invalid",
        });
    }
    validate_indicator_samples(
        input.calibration_samples,
        INDICATOR_FEATURE_COUNT,
        "DOL indicator calibration matrix is underdetermined",
    )?;
    validate_indicator_samples(
        input.heldout_samples,
        1,
        "DOL indicator held-out matrix is empty",
    )
}

fn validate_report_card_input(input: &DolReportCardInput<'_>) -> Result<(), TrajoptError> {
    if input.report_id.trim().is_empty() || input.verification_provenance.trim().is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL report-card provenance strings must be non-empty",
        });
    }
    if !input.min_required_margin_rad.is_finite()
        || !input.min_not_worse_fraction.is_finite()
        || input.min_not_worse_fraction < 0.0
        || input.min_not_worse_fraction > 1.0
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL report-card thresholds are invalid",
        });
    }
    let artifact = input.update_artifact;
    if artifact.design_profile_digest.trim().is_empty()
        || artifact.solver_provenance.trim().is_empty()
        || artifact.iload_sha256_hex.trim().is_empty()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL report-card artifact provenance strings must be non-empty",
        });
    }
    for value in [
        artifact.stale_pitch_bias_rad,
        artifact.stale_yaw_bias_rad,
        artifact.corrected_pitch_bias_rad,
        artifact.corrected_yaw_bias_rad,
        artifact.stale_min_margin_rad,
        artifact.redesigned_min_margin_rad,
        artifact.redesigned_not_worse_fraction,
        artifact.insertion_residual_norm_m,
        artifact.insertion_residual_limit_m,
    ] {
        if !value.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL report-card artifact fields must be finite",
            });
        }
    }
    if artifact.redesigned_not_worse_fraction < 0.0
        || artifact.redesigned_not_worse_fraction > 1.0
        || artifact.insertion_residual_norm_m < 0.0
        || artifact.insertion_residual_limit_m < 0.0
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL report-card artifact fractions and residuals are invalid",
        });
    }
    if input.indicators.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL report-card requires at least one indicator",
        });
    }
    for indicator in input.indicators {
        if indicator.name.trim().is_empty()
            || !indicator.design_value.is_finite()
            || !indicator.verified_value.is_finite()
            || !indicator.allowed_abs_error.is_finite()
            || indicator.allowed_abs_error < 0.0
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL report-card indicators are invalid",
            });
        }
    }
    Ok(())
}

fn validate_commit_readiness_input(
    input: &DolCommitReadinessInput<'_>,
) -> Result<(), TrajoptError> {
    if input.readiness_id.trim().is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL commit readiness id must be non-empty",
        });
    }
    validate_report_card_report(input.report_card)?;
    validate_persistence_knockdown_report(input.persistence)?;
    validate_cola_screening_report(input.cola)
}

fn validate_report_card_report(report: &DolReportCard) -> Result<(), TrajoptError> {
    if report.schema_version != DOL_REPORT_CARD_SCHEMA_VERSION {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL report-card schema version is unsupported",
        });
    }
    if report.report_id.trim().is_empty()
        || report.verification_provenance.trim().is_empty()
        || report.design_profile_digest.trim().is_empty()
        || report.solver_provenance.trim().is_empty()
        || report.iload_sha256_hex.trim().is_empty()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL report-card strings must be non-empty",
        });
    }
    if report.load_window_sample_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL report-card load window sample count must be nonzero",
        });
    }
    for value in [
        report.redesigned_min_margin_rad,
        report.min_required_margin_rad,
        report.redesigned_not_worse_fraction,
        report.min_not_worse_fraction,
        report.insertion_residual_norm_m,
        report.insertion_residual_limit_m,
    ] {
        if !value.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL report-card summary values must be finite",
            });
        }
    }
    if report.redesigned_not_worse_fraction < 0.0
        || report.redesigned_not_worse_fraction > 1.0
        || report.min_not_worse_fraction < 0.0
        || report.min_not_worse_fraction > 1.0
        || report.insertion_residual_norm_m < 0.0
        || report.insertion_residual_limit_m < 0.0
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL report-card fractions and residuals are invalid",
        });
    }
    if report.indicators.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL report-card report requires at least one indicator",
        });
    }

    let mut expected_verdict = DolReportVerdict::Go;
    let mut expected_binding = None::<String>;
    if report.redesigned_min_margin_rad < report.min_required_margin_rad {
        expected_verdict = expected_verdict.latch(DolReportVerdict::NoGo);
        expected_binding = Some("redesigned_min_margin_rad".to_owned());
    }
    if report.redesigned_not_worse_fraction < report.min_not_worse_fraction {
        expected_verdict = expected_verdict.latch(DolReportVerdict::NoGo);
        expected_binding.get_or_insert_with(|| "redesigned_not_worse_fraction".to_owned());
    }
    if report.insertion_residual_norm_m > report.insertion_residual_limit_m {
        expected_verdict = expected_verdict.latch(DolReportVerdict::NoGo);
        expected_binding.get_or_insert_with(|| "insertion_residual_norm_m".to_owned());
    }
    for indicator in &report.indicators {
        if indicator.name.trim().is_empty()
            || !indicator.design_value.is_finite()
            || !indicator.verified_value.is_finite()
            || !indicator.abs_error.is_finite()
            || !indicator.allowed_abs_error.is_finite()
            || indicator.allowed_abs_error < 0.0
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL report-card indicator rows are invalid",
            });
        }
        let expected_abs_error = (indicator.verified_value - indicator.design_value).abs();
        if (expected_abs_error - indicator.abs_error).abs() > REPORT_TOLERANCE {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL report-card indicator absolute error is inconsistent",
            });
        }
        let expected_passed = indicator.abs_error <= indicator.allowed_abs_error;
        if indicator.passed != expected_passed {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL report-card indicator pass flag is inconsistent",
            });
        }
        if !indicator.passed {
            expected_verdict = expected_verdict.latch(DolReportVerdict::NoGo);
            expected_binding.get_or_insert_with(|| indicator.name.clone());
        }
    }
    validate_report_verdict_consistency(
        report.verdict,
        expected_verdict,
        report.binding_indicator.as_deref(),
        expected_binding.as_deref(),
        "DOL report-card verdict is inconsistent",
    )
}

fn validate_persistence_knockdown_report(
    report: &DolPersistenceKnockdownReport,
) -> Result<(), TrajoptError> {
    if report.schema_version != DOL_PERSISTENCE_KNOCKDOWN_SCHEMA_VERSION {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL persistence report schema version is unsupported",
        });
    }
    if report.deck_hash.trim().is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL persistence report deck hash must be non-empty",
        });
    }
    validate_nonnegative_finite(
        report.lead_time_s,
        "DOL persistence report lead time must be finite and non-negative",
    )?;
    if !report.min_required_margin_rad.is_finite()
        || !report.min_knocked_down_margin_rad.is_finite()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL persistence report summary values must be finite",
        });
    }
    if report.indicators.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL persistence report requires at least one indicator",
        });
    }

    let mut expected_verdict = DolReportVerdict::Go;
    let mut expected_binding = None::<String>;
    let mut computed_min = f64::INFINITY;
    for indicator in &report.indicators {
        if indicator.name.trim().is_empty()
            || !indicator.altitude_m.is_finite()
            || !indicator.nominal_margin_rad.is_finite()
            || !indicator.persistence_sigma_m_s.is_finite()
            || !indicator.persistence_knockdown_rad.is_finite()
            || !indicator.gust_knockdown_rad.is_finite()
            || !indicator.system_knockdown_rad.is_finite()
            || !indicator.total_knockdown_rad.is_finite()
            || !indicator.knocked_down_margin_rad.is_finite()
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL persistence report indicator rows are invalid",
            });
        }
        let expected_passed = indicator.knocked_down_margin_rad >= report.min_required_margin_rad;
        if indicator.passed != expected_passed {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL persistence report pass flag is inconsistent",
            });
        }
        if !indicator.passed {
            expected_verdict = expected_verdict.latch(DolReportVerdict::NoGo);
            expected_binding.get_or_insert_with(|| indicator.name.clone());
        }
        computed_min = computed_min.min(indicator.knocked_down_margin_rad);
    }
    if (computed_min - report.min_knocked_down_margin_rad).abs() > REPORT_TOLERANCE {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL persistence report minimum margin is inconsistent",
        });
    }
    validate_report_verdict_consistency(
        report.verdict,
        expected_verdict,
        report.binding_indicator.as_deref(),
        expected_binding.as_deref(),
        "DOL persistence report verdict is inconsistent",
    )
}

fn validate_cola_screening_report(report: &DolColaScreeningReport) -> Result<(), TrajoptError> {
    if report.schema_version != DOL_COLA_SCREENING_SCHEMA_VERSION {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL COLA report schema version is unsupported",
        });
    }
    validate_nonnegative_finite(
        report.vehicle_keepout_radius_m,
        "DOL COLA report keepout radius must be finite and non-negative",
    )?;
    if report.slots.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL COLA report requires at least one slot",
        });
    }

    let mut blocked_slot_count = 0usize;
    let mut expected_binding_slot = None::<String>;
    let mut expected_binding_object = None::<String>;
    for slot in &report.slots {
        if slot.slot_id.trim().is_empty()
            || !slot.launch_time_s.is_finite()
            || !slot.min_clearance_m.is_finite()
            || slot.encounters.is_empty()
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL COLA slot report rows are invalid",
            });
        }
        let mut computed_min_clearance_m = f64::INFINITY;
        let mut computed_blocked = false;
        let mut computed_binding_object = None::<String>;
        for encounter in &slot.encounters {
            if encounter.slot_id.trim().is_empty()
                || encounter.object_id.trim().is_empty()
                || !encounter.trajectory_time_s.is_finite()
                || !encounter.closest_time_s.is_finite()
                || !encounter.miss_distance_m.is_finite()
                || !encounter.vehicle_keepout_radius_m.is_finite()
                || !encounter.object_radius_m.is_finite()
                || !encounter.clearance_m.is_finite()
            {
                return Err(TrajoptError::InvalidPayload {
                    reason: "DOL COLA encounter report rows are invalid",
                });
            }
            if encounter.slot_id != slot.slot_id {
                return Err(TrajoptError::InvalidPayload {
                    reason: "DOL COLA encounter slot id is inconsistent",
                });
            }
            let expected_blocked = encounter.clearance_m < 0.0;
            if encounter.blocked != expected_blocked {
                return Err(TrajoptError::InvalidPayload {
                    reason: "DOL COLA encounter blocked flag is inconsistent",
                });
            }
            if encounter.blocked {
                computed_blocked = true;
                computed_binding_object.get_or_insert_with(|| encounter.object_id.clone());
            }
            computed_min_clearance_m = computed_min_clearance_m.min(encounter.clearance_m);
        }
        if (computed_min_clearance_m - slot.min_clearance_m).abs() > REPORT_TOLERANCE {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL COLA slot minimum clearance is inconsistent",
            });
        }
        if slot.blocked != computed_blocked {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL COLA slot blocked flag is inconsistent",
            });
        }
        if slot.binding_object.as_deref() != computed_binding_object.as_deref() {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL COLA slot binding object is inconsistent",
            });
        }
        if slot.blocked {
            blocked_slot_count += 1;
            expected_binding_slot.get_or_insert_with(|| slot.slot_id.clone());
            if expected_binding_object.is_none() {
                expected_binding_object = slot.binding_object.clone();
            }
        }
    }
    if blocked_slot_count != report.blocked_slot_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL COLA blocked slot count is inconsistent",
        });
    }
    let expected_verdict = if blocked_slot_count == 0 {
        DolReportVerdict::Go
    } else {
        DolReportVerdict::NoGo
    };
    validate_report_verdict_consistency(
        report.verdict,
        expected_verdict,
        report.binding_slot.as_deref(),
        expected_binding_slot.as_deref(),
        "DOL COLA report verdict is inconsistent",
    )?;
    if report.binding_object.as_deref() != expected_binding_object.as_deref() {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL COLA report binding object is inconsistent",
        });
    }
    Ok(())
}

fn validate_report_verdict_consistency(
    actual_verdict: DolReportVerdict,
    expected_verdict: DolReportVerdict,
    actual_binding: Option<&str>,
    expected_binding: Option<&str>,
    reason: &'static str,
) -> Result<(), TrajoptError> {
    if actual_verdict != expected_verdict || actual_binding != expected_binding {
        return Err(TrajoptError::InvalidPayload { reason });
    }
    if actual_verdict == DolReportVerdict::NoGo {
        if actual_binding.is_none_or(|binding| binding.trim().is_empty()) {
            return Err(TrajoptError::InvalidPayload { reason });
        }
    } else if actual_binding.is_some() {
        return Err(TrajoptError::InvalidPayload { reason });
    }
    Ok(())
}

fn cola_binding_detail(report: &DolColaScreeningReport) -> Option<String> {
    match (&report.binding_slot, &report.binding_object) {
        (Some(slot), Some(object)) => Some(format!("{slot}/{object}")),
        (Some(slot), None) => Some(slot.clone()),
        _ => None,
    }
}

fn validate_persistence_deck(deck: &DolPersistenceDeck) -> Result<(), TrajoptError> {
    if deck.schema_version != DOL_PERSISTENCE_KNOCKDOWN_SCHEMA_VERSION {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL persistence deck schema version is unsupported",
        });
    }
    if deck.deck_hash.trim().is_empty() || deck.provenance.trim().is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL persistence deck provenance strings must be non-empty",
        });
    }
    if deck.bands.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL persistence deck requires at least one altitude band",
        });
    }
    let mut previous_max = None::<f64>;
    for band in &deck.bands {
        if !band.min_altitude_m.is_finite() || !band.max_altitude_m.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL persistence deck band altitudes must be finite",
            });
        }
        if band.max_altitude_m <= band.min_altitude_m {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL persistence deck bands must be strictly increasing",
            });
        }
        if let Some(previous_max) = previous_max
            && band.min_altitude_m < previous_max
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL persistence deck bands must not overlap",
            });
        }
        validate_nonnegative_finite(
            band.sigma_wind_0_m_s,
            "DOL persistence deck sigma values must be finite and non-negative",
        )?;
        validate_nonnegative_finite(
            band.sigma_wind_sqrt_lead_m_s_per_sqrt_s,
            "DOL persistence deck sigma values must be finite and non-negative",
        )?;
        previous_max = Some(band.max_altitude_m);
    }
    Ok(())
}

fn validate_persistence_knockdown_input(
    input: &DolPersistenceKnockdownInput<'_>,
) -> Result<(), TrajoptError> {
    input.deck.validate()?;
    validate_nonnegative_finite(
        input.lead_time_s,
        "DOL persistence lead time must be finite and non-negative",
    )?;
    if !input.min_required_margin_rad.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL persistence minimum required margin must be finite",
        });
    }
    if input.indicators.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL persistence knockdown requires at least one indicator",
        });
    }
    for indicator in input.indicators {
        if indicator.name.trim().is_empty()
            || !indicator.altitude_m.is_finite()
            || !indicator.nominal_margin_rad.is_finite()
            || !indicator.wind_sensitivity_rad_per_m_s.is_finite()
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL persistence indicator fields must be finite and named",
            });
        }
        validate_nonnegative_finite(
            indicator.gust_sigma_m_s,
            "DOL persistence indicator sigmas must be finite and non-negative",
        )?;
        validate_nonnegative_finite(
            indicator.system_sigma_rad,
            "DOL persistence indicator sigmas must be finite and non-negative",
        )?;
        if persistence_band_for_altitude(input.deck, indicator.altitude_m).is_none() {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL persistence indicator altitude is outside the deck bands",
            });
        }
    }
    Ok(())
}

fn validate_cola_input(input: &DolColaScreeningInput<'_>) -> Result<(), TrajoptError> {
    validate_nonnegative_finite(
        input.vehicle_keepout_radius_m,
        "DOL COLA vehicle keepout radius must be finite and non-negative",
    )?;
    if input.slots.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL COLA screening requires at least one window slot",
        });
    }
    if input.catalog_objects.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL COLA screening requires at least one catalog object",
        });
    }
    for slot in input.slots {
        validate_cola_slot(slot)?;
    }
    for object in input.catalog_objects {
        validate_cola_object(object)?;
    }
    Ok(())
}

fn validate_cola_slot(slot: &DolColaWindowSlot) -> Result<(), TrajoptError> {
    if slot.slot_id.trim().is_empty() || !slot.launch_time_s.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL COLA slot id and launch time must be finite and non-empty",
        });
    }
    if slot.trajectory.len() < 2 {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL COLA slot trajectory requires at least two samples",
        });
    }
    for sample in &slot.trajectory {
        if !sample.time_s.is_finite() || !finite_vec3(sample.position_eci_m) {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL COLA trajectory samples must be finite",
            });
        }
    }
    if !slot
        .trajectory
        .windows(2)
        .all(|window| window[0].time_s < window[1].time_s)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL COLA trajectory sample times must be strictly increasing",
        });
    }
    Ok(())
}

fn validate_cola_object(object: &DolColaCatalogObject) -> Result<(), TrajoptError> {
    if object.object_id.trim().is_empty()
        || !object.epoch_s.is_finite()
        || !finite_vec3(object.position_eci_m)
        || !finite_vec3(object.velocity_eci_m_s)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL COLA catalog object fields must be finite and named",
        });
    }
    validate_nonnegative_finite(
        object.hard_body_radius_m,
        "DOL COLA catalog object radius must be finite and non-negative",
    )
}

fn closest_encounter_for_slot(
    slot: &DolColaWindowSlot,
    object: &DolColaCatalogObject,
    vehicle_keepout_radius_m: f64,
) -> Result<DolColaEncounterReport, TrajoptError> {
    let mut best_time_s = f64::NAN;
    let mut best_trajectory_time_s = f64::NAN;
    let mut best_miss_distance_m = f64::INFINITY;

    for window in slot.trajectory.windows(2) {
        let start = window[0];
        let end = window[1];
        let duration_s = end.time_s - start.time_s;
        if duration_s <= 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL COLA trajectory segment duration must be positive",
            });
        }
        let vehicle_velocity = scale3(
            sub3(end.position_eci_m, start.position_eci_m),
            1.0 / duration_s,
        );
        let segment_abs_start_s = slot.launch_time_s + start.time_s;
        let object_start = add3(
            object.position_eci_m,
            scale3(
                object.velocity_eci_m_s,
                segment_abs_start_s - object.epoch_s,
            ),
        );
        let relative_position = sub3(start.position_eci_m, object_start);
        let relative_velocity = sub3(vehicle_velocity, object.velocity_eci_m_s);
        let relative_speed_sq = dot3(relative_velocity, relative_velocity);
        let closest_dt_s = if relative_speed_sq <= f64::EPSILON {
            0.0
        } else {
            (-dot3(relative_position, relative_velocity) / relative_speed_sq).clamp(0.0, duration_s)
        };
        let miss_distance_m = norm3(add3(
            relative_position,
            scale3(relative_velocity, closest_dt_s),
        ));
        if miss_distance_m < best_miss_distance_m {
            best_miss_distance_m = miss_distance_m;
            best_trajectory_time_s = start.time_s + closest_dt_s;
            best_time_s = segment_abs_start_s + closest_dt_s;
        }
    }

    let combined_keepout_m = vehicle_keepout_radius_m + object.hard_body_radius_m;
    let clearance_m = best_miss_distance_m - combined_keepout_m;
    Ok(DolColaEncounterReport {
        slot_id: slot.slot_id.clone(),
        object_id: object.object_id.clone(),
        trajectory_time_s: best_trajectory_time_s,
        closest_time_s: best_time_s,
        miss_distance_m: best_miss_distance_m,
        vehicle_keepout_radius_m,
        object_radius_m: object.hard_body_radius_m,
        clearance_m,
        blocked: clearance_m < 0.0,
    })
}

fn validate_nonnegative_finite(value: f64, reason: &'static str) -> Result<(), TrajoptError> {
    if !value.is_finite() || value < 0.0 {
        return Err(TrajoptError::InvalidPayload { reason });
    }
    Ok(())
}

fn persistence_band_for_altitude(
    deck: &DolPersistenceDeck,
    altitude_m: f64,
) -> Option<DolPersistenceBand> {
    if !altitude_m.is_finite() {
        return None;
    }
    deck.bands
        .iter()
        .enumerate()
        .find(|(index, band)| {
            let is_last = *index + 1 == deck.bands.len();
            altitude_m >= band.min_altitude_m
                && (altitude_m < band.max_altitude_m
                    || (is_last && altitude_m <= band.max_altitude_m))
        })
        .map(|(_, band)| *band)
}

fn rss3(a: f64, b: f64, c: f64) -> f64 {
    (a * a + b * b + c * c).sqrt()
}

fn finite_vec3(value: [f64; 3]) -> bool {
    value.iter().all(|component| component.is_finite())
}

fn add3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale3(value: [f64; 3], scale: f64) -> [f64; 3] {
    [value[0] * scale, value[1] * scale, value[2] * scale]
}

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm3(value: [f64; 3]) -> f64 {
    dot3(value, value).sqrt()
}

fn validate_indicator_samples(
    samples: &[LoadIndicatorSample],
    min_len: usize,
    short_reason: &'static str,
) -> Result<(), TrajoptError> {
    if samples.len() < min_len {
        return Err(TrajoptError::InvalidPayload {
            reason: short_reason,
        });
    }
    for sample in samples {
        if !sample.dynamic_pressure_pa.is_finite()
            || sample.dynamic_pressure_pa < 0.0
            || !sample.alpha_rad.is_finite()
            || !sample.beta_rad.is_finite()
            || !sample.gimbal_angle_rad.is_finite()
            || !sample.station_moment_n_m.is_finite()
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "DOL indicator samples must be finite with non-negative dynamic pressure",
            });
        }
    }
    Ok(())
}

fn fit_indicator_coefficients(
    samples: &[LoadIndicatorSample],
) -> Result<[f64; INDICATOR_FEATURE_COUNT], TrajoptError> {
    let mut normal = [[0.0_f64; INDICATOR_FEATURE_COUNT + 1]; INDICATOR_FEATURE_COUNT];
    for sample in samples {
        let features = indicator_features(*sample);
        for row in 0..INDICATOR_FEATURE_COUNT {
            for col in 0..INDICATOR_FEATURE_COUNT {
                normal[row][col] += features[row] * features[col];
            }
            normal[row][INDICATOR_FEATURE_COUNT] += features[row] * sample.station_moment_n_m;
        }
    }
    solve_indicator_normal_equations(normal)
}

#[allow(clippy::needless_range_loop)]
fn solve_indicator_normal_equations(
    mut matrix: [[f64; INDICATOR_FEATURE_COUNT + 1]; INDICATOR_FEATURE_COUNT],
) -> Result<[f64; INDICATOR_FEATURE_COUNT], TrajoptError> {
    for pivot in 0..INDICATOR_FEATURE_COUNT {
        let mut pivot_row = pivot;
        let mut pivot_abs = matrix[pivot][pivot].abs();
        for row in (pivot + 1)..INDICATOR_FEATURE_COUNT {
            let candidate_abs = matrix[row][pivot].abs();
            if candidate_abs > pivot_abs {
                pivot_abs = candidate_abs;
                pivot_row = row;
            }
        }
        if !pivot_abs.is_finite() || pivot_abs <= 1.0e-12 {
            return Err(TrajoptError::SingularSystem {
                reason: "DOL indicator normal equations are singular",
            });
        }
        if pivot_row != pivot {
            matrix.swap(pivot, pivot_row);
        }
        let divisor = matrix[pivot][pivot];
        for col in pivot..=INDICATOR_FEATURE_COUNT {
            matrix[pivot][col] /= divisor;
        }
        for row in 0..INDICATOR_FEATURE_COUNT {
            if row == pivot {
                continue;
            }
            let factor = matrix[row][pivot];
            for col in pivot..=INDICATOR_FEATURE_COUNT {
                matrix[row][col] -= factor * matrix[pivot][col];
            }
        }
    }
    Ok([
        matrix[0][INDICATOR_FEATURE_COUNT],
        matrix[1][INDICATOR_FEATURE_COUNT],
        matrix[2][INDICATOR_FEATURE_COUNT],
        matrix[3][INDICATOR_FEATURE_COUNT],
    ])
}

fn calibration_rms(
    samples: &[LoadIndicatorSample],
    coefficients: [f64; INDICATOR_FEATURE_COUNT],
) -> f64 {
    let sum_sq = samples.iter().fold(0.0_f64, |sum, sample| {
        let residual =
            sample.station_moment_n_m - evaluate_indicator_coefficients(coefficients, *sample);
        sum + residual * residual
    });
    (sum_sq / samples.len() as f64).sqrt()
}

fn validate_indicator_surrogate(
    surrogate: &LoadIndicatorSurrogate,
    samples: &[LoadIndicatorSample],
    min_correlation: f64,
    max_allowed_unconservative_error_n_m: f64,
) -> Result<LoadIndicatorValidationReport, TrajoptError> {
    let predictions: Vec<_> = samples
        .iter()
        .map(|sample| evaluate_load_indicator_surrogate(surrogate, *sample))
        .collect();
    let actual: Vec<_> = samples
        .iter()
        .map(|sample| sample.station_moment_n_m)
        .collect();
    let correlation = pearson_correlation(&actual, &predictions);
    let mut max_abs_error_n_m = 0.0_f64;
    let mut max_unconservative_error_n_m = 0.0_f64;
    for (sample, predicted) in samples.iter().zip(predictions.iter()) {
        let error = (sample.station_moment_n_m - predicted).abs();
        max_abs_error_n_m = max_abs_error_n_m.max(error);
        let unconservative = (sample.station_moment_n_m.abs() - predicted.abs()).max(0.0);
        max_unconservative_error_n_m = max_unconservative_error_n_m.max(unconservative);
    }
    if correlation < min_correlation {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL indicator surrogate correlation is below threshold",
        });
    }
    if max_unconservative_error_n_m > max_allowed_unconservative_error_n_m {
        return Err(TrajoptError::InvalidPayload {
            reason: "DOL indicator surrogate is unconservative",
        });
    }
    Ok(LoadIndicatorValidationReport {
        heldout_sample_count: samples.len(),
        correlation,
        max_abs_error_n_m,
        max_unconservative_error_n_m,
        min_required_correlation: min_correlation,
        max_allowed_unconservative_error_n_m,
        drift_gate_passed: true,
    })
}

fn evaluate_indicator_coefficients(
    coefficients: [f64; INDICATOR_FEATURE_COUNT],
    sample: LoadIndicatorSample,
) -> f64 {
    let features = indicator_features(sample);
    coefficients
        .iter()
        .zip(features.iter())
        .fold(0.0_f64, |sum, (coefficient, feature)| {
            sum + coefficient * feature
        })
}

fn indicator_features(sample: LoadIndicatorSample) -> [f64; INDICATOR_FEATURE_COUNT] {
    let indicators = load_indicator_vector(sample);
    [
        1.0,
        indicators.q_alpha_pa_rad,
        indicators.q_beta_pa_rad,
        indicators.gimbal_angle_rad,
    ]
}

fn pearson_correlation(actual: &[f64], predicted: &[f64]) -> f64 {
    let mean_actual = mean(actual);
    let mean_predicted = mean(predicted);
    let mut covariance = 0.0_f64;
    let mut actual_var = 0.0_f64;
    let mut predicted_var = 0.0_f64;
    for (actual_value, predicted_value) in actual.iter().zip(predicted.iter()) {
        let da = actual_value - mean_actual;
        let dp = predicted_value - mean_predicted;
        covariance += da * dp;
        actual_var += da * da;
        predicted_var += dp * dp;
    }
    if actual_var <= f64::EPSILON || predicted_var <= f64::EPSILON {
        let all_equal = actual
            .iter()
            .zip(predicted.iter())
            .all(|(actual_value, predicted_value)| (actual_value - predicted_value).abs() <= 1e-12);
        return if all_equal { 1.0 } else { 0.0 };
    }
    covariance / (actual_var * predicted_var).sqrt()
}

fn mean(values: &[f64]) -> f64 {
    values.iter().sum::<f64>() / values.len() as f64
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct InducedAngles {
    alpha_rad: f64,
    beta_rad: f64,
}

fn induced_angles(
    sample: openbmp_physics::wind::SplicedWindSample,
    airspeed_m_s: f64,
) -> InducedAngles {
    InducedAngles {
        alpha_rad: (-sample.wind_ned_m_s.vector.z).atan2(airspeed_m_s),
        beta_rad: sample.wind_ned_m_s.vector.y.atan2(airspeed_m_s),
    }
}

fn load_margin_rad(
    alpha_rad: f64,
    beta_rad: f64,
    alpha_reference_rad: f64,
    beta_reference_rad: f64,
    limit_rad: f64,
) -> f64 {
    let alpha_error = alpha_rad - alpha_reference_rad;
    let beta_error = beta_rad - beta_reference_rad;
    limit_rad - (alpha_error * alpha_error + beta_error * beta_error).sqrt()
}

fn pseudo_time_breakpoints(
    samples: &[openbmp_physics::wind::SplicedWindSample],
    airspeed_m_s: f64,
) -> Vec<f64> {
    let min_altitude_m = samples[0].altitude_m;
    samples
        .iter()
        .map(|sample| (sample.altitude_m - min_altitude_m) / airspeed_m_s)
        .collect()
}

fn reference_profile(
    samples: &[openbmp_physics::wind::SplicedWindSample],
    airspeed_m_s: f64,
) -> Vec<ReferenceProfileSample> {
    let min_altitude_m = samples[0].altitude_m;
    samples
        .iter()
        .map(|sample| ReferenceProfileSample {
            time_s: (sample.altitude_m - min_altitude_m) / airspeed_m_s,
            radius_m: WGS84_A_M + sample.altitude_m,
            speed_m_s: airspeed_m_s,
            flight_path_angle_rad: 0.0,
        })
        .collect()
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use openbmp_core::{Ned, Velocity3};
    use openbmp_physics::wind::{
        MeasuredWindProfile, MeasuredWindSample, SpliceRule, SplicedWindProfile,
        SyntheticWindFamilyConfig, WindProfileProvenance, WindSource,
    };

    fn provenance(id: &str) -> WindProfileProvenance {
        WindProfileProvenance::new(id, "inline://dol-test", None, false).unwrap()
    }

    fn measured(source: WindSource, samples: Vec<MeasuredWindSample>) -> MeasuredWindProfile {
        MeasuredWindProfile::new(source, 0.0, samples, provenance(source.as_str())).unwrap()
    }

    fn spliced_profile() -> SplicedWindProfile {
        let lower = measured(
            WindSource::Balloon,
            vec![
                MeasuredWindSample::new(0.0, 5.0, 3.0, -2.0, 1.0),
                MeasuredWindSample::new(1_000.0, 7.0, 5.0, -3.0, 1.0),
                MeasuredWindSample::new(2_000.0, 9.0, 7.0, -4.0, 1.0),
            ],
        );
        let upper = measured(
            WindSource::DopplerProfiler,
            vec![
                MeasuredWindSample::new(1_000.0, 8.0, 6.0, -3.5, 1.0),
                MeasuredWindSample::new(2_000.0, 10.0, 8.0, -4.5, 1.0),
                MeasuredWindSample::new(3_000.0, 12.0, 10.0, -5.0, 1.0),
            ],
        );
        SplicedWindProfile::from_profiles(
            &lower,
            &upper,
            SpliceRule::new(1_000.0, 2_000.0).unwrap(),
        )
        .unwrap()
    }

    fn input_for(profile: &SplicedWindProfile) -> DolRedesignInput<'_> {
        DolRedesignInput {
            spliced_wind: profile,
            terminal_condition: TerminalCondition::ApogeeRadius {
                radius_m: 7_578_000.0,
            },
            synthesis_seed: 19,
            scenario_digest: "sha256:test".to_owned(),
            source_revision: "test".to_owned(),
            producer: "openbmp-dol-test".to_owned(),
            design_profile_digest: "sha256:wind".to_owned(),
            solver_provenance: "dol_spliced_wind_recenter:test".to_owned(),
            load_window: DolLoadWindow::new(0.0, 3_000.0).unwrap(),
            airspeed_m_s: 250.0,
            alpha_reference_rad: 0.0,
            beta_reference_rad: 0.0,
            stale_pitch_bias_rad: 0.03,
            stale_yaw_bias_rad: -0.03,
            load_limit_angle_rad: 0.05,
            insertion_residual_norm_m: 0.01,
            insertion_residual_limit_m: 1.0,
        }
    }

    fn indicator_sample(index: usize) -> LoadIndicatorSample {
        let i = index as f64;
        let dynamic_pressure_pa = 12_000.0 + 900.0 * i + 25.0 * i * i;
        let alpha_rad = -0.025 + 0.0035 * i;
        let beta_rad = 0.018 - 0.0017 * i + 0.0001 * i * i;
        let gimbal_angle_rad = -0.020 + 0.0025 * i;
        let vector = load_indicator_vector(LoadIndicatorSample {
            dynamic_pressure_pa,
            alpha_rad,
            beta_rad,
            gimbal_angle_rad,
            station_moment_n_m: 0.0,
        });
        let station_moment_n_m = 1_200.0 + 0.82 * vector.q_alpha_pa_rad
            - 0.37 * vector.q_beta_pa_rad
            + 2_500.0 * vector.gimbal_angle_rad;
        LoadIndicatorSample {
            dynamic_pressure_pa,
            alpha_rad,
            beta_rad,
            gimbal_angle_rad,
            station_moment_n_m,
        }
    }

    fn indicator_fit_input<'a>(
        calibration_samples: &'a [LoadIndicatorSample],
        heldout_samples: &'a [LoadIndicatorSample],
    ) -> LoadIndicatorFitInput<'a> {
        LoadIndicatorFitInput {
            calibration_samples,
            heldout_samples,
            expected_deck_hash: "sha256:openbmp-dol-load-indicator-surrogate-v1".to_owned(),
            actual_deck_hash: "sha256:openbmp-dol-load-indicator-surrogate-v1".to_owned(),
            min_correlation: 0.95,
            max_unconservative_error_n_m: 1.0e-6,
        }
    }

    fn persistence_deck() -> DolPersistenceDeck {
        DolPersistenceDeck {
            schema_version: DOL_PERSISTENCE_KNOCKDOWN_SCHEMA_VERSION,
            deck_hash: "sha256:openbmp-dol-persistence-knockdown-v1".to_owned(),
            provenance: "synthetic-openbmp-wp19.5-method-fixture".to_owned(),
            bands: vec![
                DolPersistenceBand {
                    min_altitude_m: 0.0,
                    max_altitude_m: 1_000.0,
                    sigma_wind_0_m_s: 2.0,
                    sigma_wind_sqrt_lead_m_s_per_sqrt_s: 0.1,
                },
                DolPersistenceBand {
                    min_altitude_m: 1_000.0,
                    max_altitude_m: 3_000.0,
                    sigma_wind_0_m_s: 3.0,
                    sigma_wind_sqrt_lead_m_s_per_sqrt_s: 0.12,
                },
            ],
        }
    }

    fn persistence_indicator(nominal_margin_rad: f64) -> DolPersistenceIndicatorInput {
        DolPersistenceIndicatorInput {
            name: "q_alpha_station".to_owned(),
            altitude_m: 500.0,
            nominal_margin_rad,
            wind_sensitivity_rad_per_m_s: 0.002,
            gust_sigma_m_s: 4.0,
            system_sigma_rad: 0.005,
        }
    }

    fn cola_object() -> DolColaCatalogObject {
        DolColaCatalogObject {
            object_id: "synthetic-catalog-001".to_owned(),
            epoch_s: 0.0,
            position_eci_m: [500.0, 100.0, 0.0],
            velocity_eci_m_s: [0.0, 0.0, 0.0],
            hard_body_radius_m: 10.0,
        }
    }

    fn cola_slot(slot_id: &str, y_m: f64) -> DolColaWindowSlot {
        DolColaWindowSlot {
            slot_id: slot_id.to_owned(),
            launch_time_s: 0.0,
            trajectory: vec![
                DolColaTrajectorySample {
                    time_s: 0.0,
                    position_eci_m: [0.0, y_m, 0.0],
                },
                DolColaTrajectorySample {
                    time_s: 100.0,
                    position_eci_m: [1_000.0, y_m, 0.0],
                },
            ],
        }
    }

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "actual={actual:.16e} expected={expected:.16e}"
        );
    }

    fn toml_vec3(value: &toml::Value) -> [f64; 3] {
        let values = value.as_array().unwrap();
        [
            values[0].as_float().unwrap(),
            values[1].as_float().unwrap(),
            values[2].as_float().unwrap(),
        ]
    }

    fn report_artifact() -> ILoadUpdateArtifact {
        ILoadUpdateArtifact {
            schema_version: DOL_UPDATE_SCHEMA_VERSION,
            terminal_condition_kind: TerminalConditionKind::ApogeeRadius,
            design_profile_digest: "sha256:wind".to_owned(),
            solver_provenance: "dol_spliced_wind_recenter:test".to_owned(),
            iload_sha256_hex: "sha256:iload".to_owned(),
            load_window_sample_count: 3,
            stale_pitch_bias_rad: 0.03,
            stale_yaw_bias_rad: -0.03,
            corrected_pitch_bias_rad: 0.01,
            corrected_yaw_bias_rad: -0.01,
            stale_min_margin_rad: -0.01,
            redesigned_min_margin_rad: 0.02,
            redesigned_not_worse_fraction: 1.0,
            insertion_residual_norm_m: 0.01,
            insertion_residual_limit_m: 1.0,
        }
    }

    fn report_indicators() -> Vec<DolVerificationIndicator> {
        vec![
            DolVerificationIndicator {
                name: "q_alpha_station".to_owned(),
                design_value: 12.0,
                verified_value: 12.05,
                allowed_abs_error: 0.1,
            },
            DolVerificationIndicator {
                name: "station_moment".to_owned(),
                design_value: 5_000.0,
                verified_value: 5_000.5,
                allowed_abs_error: 1.0,
            },
        ]
    }

    fn report_card_input<'a>(
        artifact: &'a ILoadUpdateArtifact,
        indicators: &'a [DolVerificationIndicator],
    ) -> DolReportCardInput<'a> {
        DolReportCardInput {
            report_id: "dol-report-card-test".to_owned(),
            verification_provenance: "runner_6dof_synthetic".to_owned(),
            update_artifact: artifact,
            indicators,
            min_required_margin_rad: 0.0,
            min_not_worse_fraction: 0.9,
        }
    }

    fn report_card() -> DolReportCard {
        let artifact = report_artifact();
        let indicators = report_indicators();
        build_dol_report_card(report_card_input(&artifact, &indicators)).unwrap()
    }

    fn persistence_report(nominal_margin_rad: f64) -> DolPersistenceKnockdownReport {
        let deck = persistence_deck();
        let indicators = vec![persistence_indicator(nominal_margin_rad)];
        evaluate_persistence_knockdowns(DolPersistenceKnockdownInput {
            deck: &deck,
            lead_time_s: 100.0,
            indicators: &indicators,
            min_required_margin_rad: 0.0,
        })
        .unwrap()
    }

    fn cola_report(slot_y_m: f64, vehicle_keepout_radius_m: f64) -> DolColaScreeningReport {
        let slots = vec![cola_slot("slot", slot_y_m)];
        let objects = vec![cola_object()];
        screen_launch_cola(DolColaScreeningInput {
            slots: &slots,
            catalog_objects: &objects,
            vehicle_keepout_radius_m,
        })
        .unwrap()
    }

    fn commit_readiness_input<'a>(
        report_card: &'a DolReportCard,
        persistence: &'a DolPersistenceKnockdownReport,
        cola: &'a DolColaScreeningReport,
    ) -> DolCommitReadinessInput<'a> {
        DolCommitReadinessInput {
            readiness_id: "dol-commit-readiness-test".to_owned(),
            report_card,
            persistence,
            cola,
        }
    }

    #[test]
    fn dol_recenter_improves_min_margin_and_emits_iload_tables() {
        let profile = spliced_profile();
        let update = design_dol_iload_update(input_for(&profile)).unwrap();
        assert!(
            update.artifact.redesigned_min_margin_rad >= update.artifact.stale_min_margin_rad,
            "{:?}",
            update.artifact
        );
        assert_eq!(update.payload.gain_tables.len(), 2);
        assert_eq!(update.payload.gain_tables[0].name, "dol_pitch_bias_rad");
        assert_eq!(update.payload.gain_tables[1].name, "dol_yaw_bias_rad");
        assert_eq!(
            update.payload.reference_profile.len(),
            update.artifact.load_window_sample_count
        );
        assert_eq!(
            update.artifact.terminal_condition_kind,
            TerminalConditionKind::ApogeeRadius
        );
    }

    #[test]
    fn insertion_residual_fails_closed() {
        let profile = spliced_profile();
        let mut input = input_for(&profile);
        input.insertion_residual_norm_m = 2.0;
        input.insertion_residual_limit_m = 1.0;
        assert!(design_dol_iload_update(input).is_err());
    }

    #[test]
    fn synthetic_family_redesign_is_not_worse_for_declared_fraction() {
        let family = SyntheticWindFamilyConfig {
            seed: 0x1919,
            member_count: 8,
            released_at_s: 0.0,
            altitude_min_m: 0.0,
            altitude_max_m: 4_000.0,
            altitude_step_m: 500.0,
            base_wind_ned_m_s: Velocity3::<Ned>::new(5.0, 2.0, -1.0),
            shear_ned_per_s: Velocity3::<Ned>::new(0.001, 0.0005, 0.0002),
            amplitude_ned_m_s: Velocity3::<Ned>::new(4.0, 3.0, 1.0),
        };
        let profiles = family.generate().unwrap();
        let mut passing = 0usize;
        for profile in profiles {
            let spliced = SplicedWindProfile {
                lower_source: profile.source,
                upper_source: profile.source,
                samples: profile
                    .samples
                    .iter()
                    .map(|sample| openbmp_physics::wind::SplicedWindSample {
                        altitude_m: sample.altitude_m,
                        wind_ned_m_s: sample.wind_ned_m_s,
                        lower_weight_unit: 1.0,
                        upper_weight_unit: 0.0,
                    })
                    .collect(),
            };
            let update = design_dol_iload_update(input_for(&spliced)).unwrap();
            if update.artifact.redesigned_min_margin_rad >= update.artifact.stale_min_margin_rad {
                passing += 1;
            }
        }
        assert!(passing >= 7, "passing={passing}");
    }

    #[test]
    fn load_indicator_surrogate_fit_validates_heldout_matrix() {
        let calibration_samples: Vec<_> = (0..9).map(indicator_sample).collect();
        let heldout_samples: Vec<_> = (9..15).map(indicator_sample).collect();
        let result = fit_load_indicator_surrogate(indicator_fit_input(
            &calibration_samples,
            &heldout_samples,
        ))
        .unwrap();
        assert!(result.validation.drift_gate_passed);
        assert!(result.validation.correlation > 0.999_999_999);
        assert!(result.validation.max_unconservative_error_n_m <= 1.0e-6);
        assert!(
            (result.surrogate.intercept_n_m - 1_200.0).abs() <= 1.0e-6,
            "{:?}",
            result.surrogate
        );
        assert!((result.surrogate.q_alpha_n_m_per_pa_rad - 0.82).abs() <= 1.0e-9);
        assert!((result.surrogate.q_beta_n_m_per_pa_rad + 0.37).abs() <= 1.0e-9);
        assert!((result.surrogate.gimbal_n_m_per_rad - 2_500.0).abs() <= 1.0e-5);
    }

    #[test]
    fn load_indicator_surrogate_drift_gate_fails_closed() {
        let calibration_samples: Vec<_> = (0..9).map(indicator_sample).collect();
        let heldout_samples: Vec<_> = (9..15).map(indicator_sample).collect();
        let mut input = indicator_fit_input(&calibration_samples, &heldout_samples);
        input.actual_deck_hash = "sha256:changed-upstream-deck".to_owned();
        assert!(fit_load_indicator_surrogate(input).is_err());
    }

    #[test]
    fn load_indicator_fit_deck_declares_thresholds_and_hash() {
        let deck: toml::Value = toml::from_str(include_str!(
            "../../../data/wind/dol-load-indicator-surrogate-v1.toml"
        ))
        .unwrap();
        assert_eq!(
            deck["surrogate"]["schema_version"].as_integer().unwrap(),
            i64::from(DOL_LOAD_INDICATOR_SURROGATE_SCHEMA_VERSION)
        );
        assert_eq!(
            deck["surrogate"]["deck_hash"].as_str().unwrap(),
            "sha256:openbmp-dol-load-indicator-surrogate-v1"
        );
        assert!(deck["validation"]["min_correlation"].as_float().unwrap() >= 0.95);
    }

    #[test]
    fn persistence_knockdown_rss_matches_hand_computed_fixture() {
        let deck = persistence_deck();
        let indicators = vec![persistence_indicator(0.05)];
        let report = evaluate_persistence_knockdowns(DolPersistenceKnockdownInput {
            deck: &deck,
            lead_time_s: 100.0,
            indicators: &indicators,
            min_required_margin_rad: 0.0,
        })
        .unwrap();
        assert_eq!(report.verdict, DolReportVerdict::Go);
        let row = &report.indicators[0];
        assert_close(row.persistence_sigma_m_s, 3.0, 1.0e-12);
        assert_close(row.persistence_knockdown_rad, 0.006, 1.0e-12);
        assert_close(row.gust_knockdown_rad, 0.008, 1.0e-12);
        assert_close(row.system_knockdown_rad, 0.005, 1.0e-12);
        let expected_total = (0.006_f64 * 0.006 + 0.008_f64 * 0.008 + 0.005_f64 * 0.005).sqrt();
        assert_close(row.total_knockdown_rad, expected_total, 1.0e-15);
        assert_close(row.knocked_down_margin_rad, 0.05 - expected_total, 1.0e-15);
    }

    #[test]
    fn persistence_knockdown_margins_decrease_with_lead_time() {
        let deck = persistence_deck();
        let indicators = vec![persistence_indicator(0.05)];
        let leads = [0.0, 100.0, 400.0];
        let margins: Vec<_> = leads
            .iter()
            .map(|lead_time_s| {
                evaluate_persistence_knockdowns(DolPersistenceKnockdownInput {
                    deck: &deck,
                    lead_time_s: *lead_time_s,
                    indicators: &indicators,
                    min_required_margin_rad: 0.0,
                })
                .unwrap()
                .min_knocked_down_margin_rad
            })
            .collect();
        assert!(margins[0] >= margins[1], "{margins:?}");
        assert!(margins[1] >= margins[2], "{margins:?}");
    }

    #[test]
    fn persistence_knockdown_latches_no_go_on_binding_indicator() {
        let deck = persistence_deck();
        let indicators = vec![persistence_indicator(0.005)];
        let report = evaluate_persistence_knockdowns(DolPersistenceKnockdownInput {
            deck: &deck,
            lead_time_s: 100.0,
            indicators: &indicators,
            min_required_margin_rad: 0.0,
        })
        .unwrap();
        assert_eq!(report.verdict, DolReportVerdict::NoGo);
        assert_eq!(report.binding_indicator.as_deref(), Some("q_alpha_station"));
        assert!(!report.indicators[0].passed);
    }

    #[test]
    fn persistence_deck_rejects_overlapping_or_nonfinite_bands() {
        let mut deck = persistence_deck();
        deck.bands[1].min_altitude_m = 900.0;
        assert!(deck.validate().is_err());
        deck = persistence_deck();
        deck.bands[0].sigma_wind_0_m_s = f64::NAN;
        assert!(deck.validate().is_err());
    }

    #[test]
    fn persistence_deck_fixture_validates_synthetic_truth_samples() {
        let deck: toml::Value = toml::from_str(include_str!(
            "../../../data/wind/dol-persistence-knockdown-v1.toml"
        ))
        .unwrap();
        assert_eq!(
            deck["deck"]["schema_version"].as_integer().unwrap(),
            i64::from(DOL_PERSISTENCE_KNOCKDOWN_SCHEMA_VERSION)
        );
        assert_eq!(
            deck["deck"]["deck_hash"].as_str().unwrap(),
            "sha256:openbmp-dol-persistence-knockdown-v1"
        );
        let bands: Vec<_> = deck["bands"]
            .as_array()
            .unwrap()
            .iter()
            .map(|band| DolPersistenceBand {
                min_altitude_m: band["min_altitude_m"].as_float().unwrap(),
                max_altitude_m: band["max_altitude_m"].as_float().unwrap(),
                sigma_wind_0_m_s: band["sigma_wind_0_m_s"].as_float().unwrap(),
                sigma_wind_sqrt_lead_m_s_per_sqrt_s: band["sigma_wind_sqrt_lead_m_s_per_sqrt_s"]
                    .as_float()
                    .unwrap(),
            })
            .collect();
        let persistence_deck = DolPersistenceDeck {
            schema_version: DOL_PERSISTENCE_KNOCKDOWN_SCHEMA_VERSION,
            deck_hash: deck["deck"]["deck_hash"].as_str().unwrap().to_owned(),
            provenance: deck["deck"]["provenance"].as_str().unwrap().to_owned(),
            bands,
        };
        let truth_samples: Vec<_> = deck["synthetic_truth"]
            .as_array()
            .unwrap()
            .iter()
            .map(|sample| DolPersistenceTruthSample {
                altitude_m: sample["altitude_m"].as_float().unwrap(),
                lead_time_s: sample["lead_time_s"].as_float().unwrap(),
                delta_wind_m_s: sample["delta_wind_m_s"].as_float().unwrap(),
            })
            .collect();
        let report = validate_persistence_truth_samples(DolPersistenceTruthValidationInput {
            deck: &persistence_deck,
            samples: &truth_samples,
            sigma_multiplier: deck["deck"]["sigma_multiplier"].as_float().unwrap(),
        })
        .unwrap();
        assert_eq!(report.sample_count, 3);
        assert!(report.passed, "{report:?}");
        assert!(report.max_abs_sigma_ratio <= 3.0);
    }

    #[test]
    fn cola_closed_form_conjunction_matches_linear_geometry() {
        let slots = vec![cola_slot("slot", 0.0)];
        let objects = vec![cola_object()];
        let report = screen_launch_cola(DolColaScreeningInput {
            slots: &slots,
            catalog_objects: &objects,
            vehicle_keepout_radius_m: 50.0,
        })
        .unwrap();
        assert_eq!(report.verdict, DolReportVerdict::Go);
        let encounter = &report.slots[0].encounters[0];
        assert_close(encounter.trajectory_time_s, 50.0, 1.0e-12);
        assert_close(encounter.closest_time_s, 50.0, 1.0e-12);
        assert_close(encounter.miss_distance_m, 100.0, 1.0e-12);
        assert_close(encounter.clearance_m, 40.0, 1.0e-12);
    }

    #[test]
    fn cola_slot_table_blocks_binding_object() {
        let slots = vec![cola_slot("clear", 500.0), cola_slot("blocked", 0.0)];
        let objects = vec![cola_object()];
        let report = screen_launch_cola(DolColaScreeningInput {
            slots: &slots,
            catalog_objects: &objects,
            vehicle_keepout_radius_m: 120.0,
        })
        .unwrap();
        assert_eq!(report.verdict, DolReportVerdict::NoGo);
        assert_eq!(report.blocked_slot_count, 1);
        assert_eq!(report.binding_slot.as_deref(), Some("blocked"));
        assert_eq!(
            report.binding_object.as_deref(),
            Some("synthetic-catalog-001")
        );
        assert!(!report.slots[0].blocked);
        assert!(report.slots[1].blocked);
    }

    #[test]
    fn cola_screening_rejects_malformed_inputs() {
        let objects = vec![cola_object()];
        let mut bad_slot = cola_slot("bad", 0.0);
        bad_slot.trajectory[1].time_s = 0.0;
        assert!(
            screen_launch_cola(DolColaScreeningInput {
                slots: &[bad_slot],
                catalog_objects: &objects,
                vehicle_keepout_radius_m: 50.0,
            })
            .is_err()
        );

        let slots = vec![cola_slot("slot", 0.0)];
        assert!(
            screen_launch_cola(DolColaScreeningInput {
                slots: &slots,
                catalog_objects: &[],
                vehicle_keepout_radius_m: 50.0,
            })
            .is_err()
        );
        assert!(
            screen_launch_cola(DolColaScreeningInput {
                slots: &slots,
                catalog_objects: &objects,
                vehicle_keepout_radius_m: -1.0,
            })
            .is_err()
        );
    }

    #[test]
    fn cola_fixture_declares_deterministic_clear_and_blocked_slots() {
        let deck: toml::Value = toml::from_str(include_str!(
            "../../../data/cola/launch-cola-screening-v1.toml"
        ))
        .unwrap();
        assert_eq!(
            deck["deck"]["schema_version"].as_integer().unwrap(),
            i64::from(DOL_COLA_SCREENING_SCHEMA_VERSION)
        );
        assert_eq!(
            deck["deck"]["deck_hash"].as_str().unwrap(),
            "sha256:openbmp-dol-cola-screening-v1"
        );
        let objects: Vec<_> = deck["catalog_objects"]
            .as_array()
            .unwrap()
            .iter()
            .map(|object| DolColaCatalogObject {
                object_id: object["object_id"].as_str().unwrap().to_owned(),
                epoch_s: object["epoch_s"].as_float().unwrap(),
                position_eci_m: toml_vec3(&object["position_eci_m"]),
                velocity_eci_m_s: toml_vec3(&object["velocity_eci_m_s"]),
                hard_body_radius_m: object["hard_body_radius_m"].as_float().unwrap(),
            })
            .collect();
        let slots: Vec<_> = deck["slots"]
            .as_array()
            .unwrap()
            .iter()
            .map(|slot| DolColaWindowSlot {
                slot_id: slot["slot_id"].as_str().unwrap().to_owned(),
                launch_time_s: slot["launch_time_s"].as_float().unwrap(),
                trajectory: slot["trajectory"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|sample| DolColaTrajectorySample {
                        time_s: sample["time_s"].as_float().unwrap(),
                        position_eci_m: toml_vec3(&sample["position_eci_m"]),
                    })
                    .collect(),
            })
            .collect();
        let report = screen_launch_cola(DolColaScreeningInput {
            slots: &slots,
            catalog_objects: &objects,
            vehicle_keepout_radius_m: deck["deck"]["vehicle_keepout_radius_m"].as_float().unwrap(),
        })
        .unwrap();
        assert_eq!(report.schema_version, DOL_COLA_SCREENING_SCHEMA_VERSION);
        assert_eq!(report.slots.len(), 2);
        assert_eq!(report.blocked_slot_count, 1);
        assert_eq!(report.slots[0].slot_id, "clear-slot");
        assert!(!report.slots[0].blocked);
        assert_eq!(report.slots[1].slot_id, "blocked-slot");
        assert!(report.slots[1].blocked);
    }

    #[test]
    fn report_card_csv_is_byte_stable_and_go() {
        let artifact = report_artifact();
        let indicators = report_indicators();
        let report = build_dol_report_card(report_card_input(&artifact, &indicators)).unwrap();
        assert_eq!(report.verdict, DolReportVerdict::Go);
        assert_eq!(
            dol_report_card_csv(&report),
            include_str!("../tests/expected/dol-report-card-v1.csv")
        );
    }

    #[test]
    fn report_card_latches_no_go_on_surrogate_breach() {
        let artifact = report_artifact();
        let mut indicators = report_indicators();
        indicators[1].verified_value = 5_010.0;
        let report = build_dol_report_card(report_card_input(&artifact, &indicators)).unwrap();
        assert_eq!(report.verdict, DolReportVerdict::NoGo);
        assert_eq!(report.binding_indicator.as_deref(), Some("station_moment"));
        assert!(!report.indicators[1].passed);
    }

    #[test]
    fn report_card_verdict_latching_is_monotone() {
        assert_eq!(
            DolReportVerdict::Go.latch(DolReportVerdict::Go),
            DolReportVerdict::Go
        );
        assert_eq!(
            DolReportVerdict::Go.latch(DolReportVerdict::NoGo),
            DolReportVerdict::NoGo
        );
        assert_eq!(
            DolReportVerdict::NoGo.latch(DolReportVerdict::Go),
            DolReportVerdict::NoGo
        );
        assert_eq!(
            DolReportVerdict::NoGo.latch(DolReportVerdict::NoGo),
            DolReportVerdict::NoGo
        );
    }

    #[test]
    fn report_card_rejects_empty_indicator_set() {
        let artifact = report_artifact();
        assert!(build_dol_report_card(report_card_input(&artifact, &[])).is_err());
    }

    #[test]
    fn commit_readiness_csv_latches_all_go_gates() {
        let report_card = report_card();
        let persistence = persistence_report(0.05);
        let cola = cola_report(500.0, 50.0);
        let readiness =
            build_dol_commit_readiness(commit_readiness_input(&report_card, &persistence, &cola))
                .unwrap();
        assert_eq!(readiness.verdict, DolReportVerdict::Go);
        assert_eq!(readiness.binding_gate, None);
        assert_eq!(readiness.gates.len(), 3);
        assert_eq!(
            dol_commit_readiness_csv(&readiness),
            include_str!("../tests/expected/dol-commit-readiness-v1.csv")
        );
    }

    #[test]
    fn commit_readiness_latches_report_card_before_downstream_gates() {
        let artifact = report_artifact();
        let mut indicators = report_indicators();
        indicators[1].verified_value = 5_010.0;
        let report_card = build_dol_report_card(report_card_input(&artifact, &indicators)).unwrap();
        let persistence = persistence_report(0.005);
        let cola = cola_report(0.0, 120.0);
        let readiness =
            build_dol_commit_readiness(commit_readiness_input(&report_card, &persistence, &cola))
                .unwrap();
        assert_eq!(readiness.verdict, DolReportVerdict::NoGo);
        assert_eq!(readiness.binding_gate.as_deref(), Some("report_card"));
        assert_eq!(readiness.binding_detail.as_deref(), Some("station_moment"));
    }

    #[test]
    fn commit_readiness_latches_persistence_before_cola() {
        let report_card = report_card();
        let persistence = persistence_report(0.005);
        let cola = cola_report(0.0, 120.0);
        let readiness =
            build_dol_commit_readiness(commit_readiness_input(&report_card, &persistence, &cola))
                .unwrap();
        assert_eq!(readiness.verdict, DolReportVerdict::NoGo);
        assert_eq!(readiness.binding_gate.as_deref(), Some("persistence"));
        assert_eq!(readiness.binding_detail.as_deref(), Some("q_alpha_station"));
        assert_eq!(
            readiness.gates[2].binding_detail.as_deref(),
            Some("slot/synthetic-catalog-001")
        );
    }

    #[test]
    fn commit_readiness_rejects_empty_id_and_inconsistent_gate_reports() {
        let report_card = report_card();
        let persistence = persistence_report(0.05);
        let cola = cola_report(500.0, 50.0);
        let mut input = commit_readiness_input(&report_card, &persistence, &cola);
        input.readiness_id = String::new();
        assert!(build_dol_commit_readiness(input).is_err());

        let mut malformed_report_card = report_card;
        malformed_report_card.verdict = DolReportVerdict::NoGo;
        assert!(
            build_dol_commit_readiness(commit_readiness_input(
                &malformed_report_card,
                &persistence,
                &cola
            ))
            .is_err()
        );
    }
}
