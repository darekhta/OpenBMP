//! Flight-profile trait surfaces (deferred schema stubs).
//!
//! Trait definitions for the multi-phase flight-profile work described
//! in `docs/flight-profiles-architecture.md` and its companions. These
//! are **signatures only** — no implementations ship yet; each lands
//! with validation evidence per `docs/roadmap.md`. They sit beside
//! [`crate::gravity`], [`crate::atmosphere`], and [`crate::reentry`] as
//! peers and follow the same `Result<_, PhysicsError>` discipline.
//!
//! # Safety posture
//!
//! Every method here answers a *forward* physics question — given a
//! vehicle and a trajectory, what happens — and never the inverse
//! operational question — given a place to reach, what to do.
//!
//! - [`AscentReferenceGenerator`] produces a *reference attitude the
//!   autopilot tracks*; it consumes only the vehicle's own state and
//!   accepts no geographic location.
//! - [`RangeSafetyFootprint`] reports *where an unpowered body is
//!   predicted to come down* in range-relative coordinates, for
//!   recovery and range-safety planning. It accepts no desired landing
//!   location and emits no steering command.
//! - [`StageSeparationModel`] and [`EntryCorridorReference`] are the
//!   staging and lifting-entry surfaces.
//!
//! See `docs/profile-vocabulary-and-guardrails.md` for the binding
//! guardrails this module is built under.

use openbmp_core::SimTime;

use crate::error::PhysicsError;

/// Default absolute tolerance for checking linear momentum residuals
/// across stage separation, in kg*m/s.
pub const STAGE_SEPARATION_MOMENTUM_TOLERANCE_KG_M_S: f64 = 1.0e-9;

/// Inertial / atmospheric state sampled during powered ascent. All
/// quantities are already carried on the simulation bus.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AscentState {
    /// Geometric altitude above the WGS84 ellipsoid (m).
    pub altitude_m: f64,
    /// Inertial speed magnitude (m/s).
    pub inertial_speed_m_s: f64,
    /// Flight-path angle above the local horizon (rad).
    pub flight_path_angle_rad: f64,
    /// Dynamic pressure (Pa).
    pub dynamic_pressure_pa: f64,
    /// Remaining mass fraction in `[0, 1]`.
    pub mass_fraction: f64,
}

/// Generated powered-ascent reference: the attitude (and optional rate)
/// the three-loop autopilot should track. It is a reference, not a
/// guidance solution to any location.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AscentReference {
    /// Reference body-to-ECI attitude quaternion `[x, y, z, w]`.
    pub q_body_to_eci_xyzw: [f64; 4],
    /// Optional reference body angular rate (rad/s), if the method
    /// supplies a feed-forward rate.
    pub body_rate_rad_s: Option<[f64; 3]>,
}

/// Produces a powered-ascent attitude reference (gravity-turn,
/// pitch-program, or the reserved explicit reference). See
/// `docs/ascent-guidance.md`.
pub trait AscentReferenceGenerator {
    /// Reference attitude for the current ascent state.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when the state is outside the method's
    /// validity envelope or produces a non-finite reference.
    fn ascent_reference(
        &self,
        state: &AscentState,
        time: SimTime,
    ) -> Result<AscentReference, PhysicsError>;
}

/// Ballistic state of an unpowered body at a point on its arc, used to
/// seed a range-safety footprint prediction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BallisticState {
    /// ECI position `[x, y, z]` (m).
    pub position_eci_m: [f64; 3],
    /// ECI velocity `[x, y, z]` (m/s).
    pub velocity_eci_m_s: [f64; 3],
    /// Ballistic coefficient `B = C_d · A / m` (m²/kg).
    pub ballistic_coefficient_m2_kg: f64,
    /// Simulation time at this state.
    pub time: SimTime,
}

/// Environment selection for a footprint propagation (which gravity and
/// atmosphere envelopes apply, and the optional launch-site origin used
/// only to map a range-relative prediction onto recovery coordinates).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FootprintEnvironment {
    /// Cull altitude (m): propagation stops at or below this altitude
    /// (typically ground level, 0.0).
    pub cull_altitude_m: f64,
    /// Whether a launch-site geodetic origin is declared, enabling the
    /// optional geodetic output. When `false`, only range-relative
    /// output is produced — never a fabricated geographic coordinate.
    pub has_launch_origin: bool,
}

/// Predicted landing footprint of an unpowered body, in range-relative
/// coordinates. This is a recovery / range-safety output, not a
/// targeting or accuracy claim. See `docs/ballistic-coast-and-apogee.md`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LandingFootprint {
    /// Nominal downrange distance from the launch origin (m).
    pub downrange_m: f64,
    /// Nominal crossrange distance from the launch origin (m).
    pub crossrange_m: f64,
    /// 1-σ semi-major axis of the dispersion ellipse (m).
    pub dispersion_semi_major_m: f64,
    /// 1-σ semi-minor axis of the dispersion ellipse (m).
    pub dispersion_semi_minor_m: f64,
}

/// Reports where an unpowered body is predicted to come down, for
/// recovery and range-safety planning. Forward-only and offline: it
/// accepts no desired landing location and produces no guidance
/// command.
pub trait RangeSafetyFootprint {
    /// Predicted nominal landing point and dispersion for a body
    /// propagated from `state` to the cull altitude.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when the state or environment is
    /// outside the propagation's validity envelope.
    fn landing_footprint(
        &self,
        state: &BallisticState,
        env: &FootprintEnvironment,
    ) -> Result<LandingFootprint, PhysicsError>;
}

/// Partitioned rigid-body states produced by a stage separation:
/// the continuing stack and the departing stage. See
/// `docs/staging-and-separation.md`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SeparationOutcome {
    /// Continuing-stack velocity increment in the body frame (m/s).
    pub stack_delta_v_body_m_s: [f64; 3],
    /// Departing-stage velocity increment in the body frame (m/s).
    pub stage_delta_v_body_m_s: [f64; 3],
}

/// Momentum-conserving stage-separation impulse model.
///
/// The model carries the requested continuing-stack and departing-stage
/// body-frame velocity increments and validates
/// `m_stack * dv_stack + m_stage * dv_stage = 0` when
/// [`StageSeparationModel::separate`] is called with concrete masses.
#[derive(Clone, Copy, Debug, PartialEq)]
#[allow(clippy::struct_field_names)] // unit suffixes are part of the public physics convention.
pub struct MomentumConservingStageSeparation {
    stack_delta_v_body_m_s: [f64; 3],
    stage_delta_v_body_m_s: [f64; 3],
    tolerance_kg_m_s: f64,
}

impl MomentumConservingStageSeparation {
    /// Construct a separation model from body-frame delta-V vectors.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] when any vector
    /// component is non-finite or `tolerance_kg_m_s` is non-finite /
    /// negative.
    pub fn new(
        stack_delta_v_body_m_s: [f64; 3],
        stage_delta_v_body_m_s: [f64; 3],
        tolerance_kg_m_s: f64,
    ) -> Result<Self, PhysicsError> {
        require_finite_vec3(
            stack_delta_v_body_m_s,
            "stack separation delta-v components must be finite",
        )?;
        require_finite_vec3(
            stage_delta_v_body_m_s,
            "stage separation delta-v components must be finite",
        )?;
        if !tolerance_kg_m_s.is_finite() || tolerance_kg_m_s < 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "stage-separation momentum tolerance must be finite and non-negative",
            });
        }
        Ok(Self {
            stack_delta_v_body_m_s,
            stage_delta_v_body_m_s,
            tolerance_kg_m_s,
        })
    }

    /// Construct the no-impulse split model.
    #[must_use]
    pub const fn no_impulse() -> Self {
        Self {
            stack_delta_v_body_m_s: [0.0, 0.0, 0.0],
            stage_delta_v_body_m_s: [0.0, 0.0, 0.0],
            tolerance_kg_m_s: STAGE_SEPARATION_MOMENTUM_TOLERANCE_KG_M_S,
        }
    }

    /// Continuing-stack body-frame delta-V vector (m/s).
    #[must_use]
    pub const fn stack_delta_v_body_m_s(&self) -> [f64; 3] {
        self.stack_delta_v_body_m_s
    }

    /// Departing-stage body-frame delta-V vector (m/s).
    #[must_use]
    pub const fn stage_delta_v_body_m_s(&self) -> [f64; 3] {
        self.stage_delta_v_body_m_s
    }

    /// Momentum-conservation tolerance (kg*m/s).
    #[must_use]
    pub const fn tolerance_kg_m_s(&self) -> f64 {
        self.tolerance_kg_m_s
    }

    /// Compute the linear-momentum residual vector
    /// `m_stack * dv_stack + m_stage * dv_stage`, in kg*m/s.
    ///
    /// Operand order is fixed per component and uses only explicit
    /// multiply / add operations.
    #[must_use]
    pub fn momentum_residual_kg_m_s(&self, stack_mass_kg: f64, stage_mass_kg: f64) -> [f64; 3] {
        [
            stack_mass_kg * self.stack_delta_v_body_m_s[0]
                + stage_mass_kg * self.stage_delta_v_body_m_s[0],
            stack_mass_kg * self.stack_delta_v_body_m_s[1]
                + stage_mass_kg * self.stage_delta_v_body_m_s[1],
            stack_mass_kg * self.stack_delta_v_body_m_s[2]
                + stage_mass_kg * self.stage_delta_v_body_m_s[2],
        ]
    }

    /// Euclidean norm of [`Self::momentum_residual_kg_m_s`].
    #[must_use]
    pub fn momentum_residual_norm_kg_m_s(&self, stack_mass_kg: f64, stage_mass_kg: f64) -> f64 {
        let r = self.momentum_residual_kg_m_s(stack_mass_kg, stage_mass_kg);
        let mut sum = 0.0_f64;
        sum += r[0] * r[0];
        sum += r[1] * r[1];
        sum += r[2] * r[2];
        sum.sqrt()
    }
}

impl Default for MomentumConservingStageSeparation {
    fn default() -> Self {
        Self::no_impulse()
    }
}

/// Computes the momentum-conserving state partition across a commanded
/// stage separation.
pub trait StageSeparationModel {
    /// Partition the pre-separation composite into the continuing stack
    /// and the departing stage.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when masses are non-positive or the
    /// requested separation impulse violates momentum conservation
    /// beyond tolerance.
    fn separate(
        &self,
        stack_mass_kg: f64,
        stage_mass_kg: f64,
    ) -> Result<SeparationOutcome, PhysicsError>;
}

impl StageSeparationModel for MomentumConservingStageSeparation {
    fn separate(
        &self,
        stack_mass_kg: f64,
        stage_mass_kg: f64,
    ) -> Result<SeparationOutcome, PhysicsError> {
        require_positive_mass(stack_mass_kg, "stack mass must be finite and positive")?;
        require_positive_mass(stage_mass_kg, "stage mass must be finite and positive")?;
        let residual = self.momentum_residual_norm_kg_m_s(stack_mass_kg, stage_mass_kg);
        if !residual.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "stage-separation momentum residual is non-finite",
            });
        }
        if residual > self.tolerance_kg_m_s {
            return Err(PhysicsError::InvalidParameter {
                reason: "stage-separation impulse violates momentum conservation tolerance",
            });
        }
        Ok(SeparationOutcome {
            stack_delta_v_body_m_s: self.stack_delta_v_body_m_s,
            stage_delta_v_body_m_s: self.stage_delta_v_body_m_s,
        })
    }
}

/// Entry-corridor limits steering a lifting entry: the bank-angle
/// reference is driven by these limits, never by a location. See
/// `docs/descent-and-entry-profiles.md`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EntryCorridor {
    /// Maximum allowable stagnation-point heat rate (W/m²).
    pub max_heat_rate_w_m2: f64,
    /// Maximum allowable deceleration load factor (g).
    pub max_load_factor_g: f64,
    /// Flight-path-angle corridor half-width (rad).
    pub flight_path_angle_band_rad: f64,
}

/// Produces a bank-angle reference for a lifting entry from corridor
/// limits and the current entry state. Corridor-driven, never
/// target-driven.
pub trait EntryCorridorReference {
    /// Reference bank angle (rad) for the current entry state.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when the state lies outside the
    /// corridor's feasible band.
    fn bank_reference_rad(
        &self,
        corridor: &EntryCorridor,
        flight_path_angle_rad: f64,
        time: SimTime,
    ) -> Result<f64, PhysicsError>;
}

fn require_positive_mass(value: f64, reason: &'static str) -> Result<(), PhysicsError> {
    if value.is_finite() && value > 0.0 {
        return Ok(());
    }
    Err(PhysicsError::InvalidParameter { reason })
}

fn require_finite_vec3(value: [f64; 3], reason: &'static str) -> Result<(), PhysicsError> {
    if value.iter().all(|component| component.is_finite()) {
        return Ok(());
    }
    Err(PhysicsError::InvalidParameter { reason })
}

#[cfg(test)]
#[allow(clippy::float_cmp, clippy::unwrap_used)]
mod tests {
    use super::{
        MomentumConservingStageSeparation, STAGE_SEPARATION_MOMENTUM_TOLERANCE_KG_M_S,
        StageSeparationModel,
    };
    use crate::PhysicsError;

    #[test]
    fn no_impulse_conserves_momentum_for_any_positive_masses() {
        let model = MomentumConservingStageSeparation::no_impulse();
        let outcome = model.separate(10.0, 2.5).unwrap();
        assert_eq!(outcome.stack_delta_v_body_m_s, [0.0, 0.0, 0.0]);
        assert_eq!(outcome.stage_delta_v_body_m_s, [0.0, 0.0, 0.0]);
        assert_eq!(model.momentum_residual_norm_kg_m_s(10.0, 2.5), 0.0);
    }

    #[test]
    fn asymmetric_masses_require_inverse_delta_v_ratio() {
        let model = MomentumConservingStageSeparation::new([0.0, 0.0, 0.25], [0.0, 0.0, -1.0], 0.0)
            .unwrap();
        let outcome = model.separate(4.0, 1.0).unwrap();
        assert_eq!(outcome.stack_delta_v_body_m_s, [0.0, 0.0, 0.25]);
        assert_eq!(outcome.stage_delta_v_body_m_s, [0.0, 0.0, -1.0]);
        assert_eq!(model.momentum_residual_kg_m_s(4.0, 1.0), [0.0, 0.0, 0.0]);
    }

    #[test]
    fn vector_components_all_participate_in_residual() {
        let model =
            MomentumConservingStageSeparation::new([0.5, -0.25, 0.125], [-1.0, 0.5, -0.25], 0.0)
                .unwrap();
        assert_eq!(model.momentum_residual_kg_m_s(2.0, 1.0), [0.0, 0.0, 0.0]);
        model.separate(2.0, 1.0).unwrap();
    }

    #[test]
    fn rejects_non_positive_masses() {
        let model = MomentumConservingStageSeparation::no_impulse();
        assert!(matches!(
            model.separate(0.0, 1.0),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            model.separate(1.0, -1.0),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn rejects_non_finite_delta_v() {
        let err = MomentumConservingStageSeparation::new(
            [0.0, f64::NAN, 0.0],
            [0.0, 0.0, 0.0],
            STAGE_SEPARATION_MOMENTUM_TOLERANCE_KG_M_S,
        )
        .unwrap_err();
        assert!(matches!(err, PhysicsError::InvalidParameter { .. }));
    }

    #[test]
    fn rejects_momentum_residual_above_tolerance() {
        let model =
            MomentumConservingStageSeparation::new([0.0, 0.0, 1.0], [0.0, 0.0, -1.0], 1.0e-12)
                .unwrap();
        assert!(matches!(
            model.separate(2.0, 1.0),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn accepts_residual_at_tolerance_boundary() {
        let model =
            MomentumConservingStageSeparation::new([0.0, 0.0, 1.0], [0.0, 0.0, -1.0], 1.0).unwrap();
        model.separate(2.0, 1.0).unwrap();
    }
}
