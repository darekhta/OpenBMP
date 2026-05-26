//! Flight-profile trait surfaces (deferred schema stubs).
//!
//! Trait definitions and the first consumed profile helpers for the
//! multi-phase flight-profile work described in
//! `docs/flight-profiles-architecture.md` and its companions. They sit beside
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

use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};
use openbmp_core::SimTime;

use crate::error::PhysicsError;
use crate::frames::WGS84_A_M;

const MIN_DIRECTION_NORM: f64 = 1.0e-12;
const QUATERNION_NORM_TOLERANCE: f64 = 1.0e-9;
const MIN_FOOTPRINT_GRAVITY_M_S2: f64 = 1.0e-12;
const MIN_LONGITUDE_COSINE: f64 = 1.0e-12;
const MIN_ENTRY_CORRIDOR_BAND_RAD: f64 = 1.0e-12;
const MIN_ENTRY_BANK_LIMIT_RAD: f64 = 1.0e-12;

/// Default minimum inertial speed for gravity-turn alignment (m/s).
pub const GRAVITY_TURN_MINIMUM_SPEED_M_S: f64 = 1.0e-6;

/// Default absolute tolerance for checking linear momentum residuals
/// across stage separation, in kg*m/s.
pub const STAGE_SEPARATION_MOMENTUM_TOLERANCE_KG_M_S: f64 = 1.0e-9;

/// Inertial / atmospheric state sampled during powered ascent. All
/// quantities are already carried on the simulation bus.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AscentState {
    /// ECI position `[x, y, z]` (m).
    pub position_eci_m: [f64; 3],
    /// ECI velocity `[x, y, z]` (m/s).
    pub velocity_eci_m_s: [f64; 3],
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

impl AscentState {
    /// Validate the state carried into an ascent reference generator.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when any field is non-finite, speed or
    /// dynamic pressure is negative, or mass fraction lies outside
    /// `[0, 1]`.
    pub fn validate(&self) -> Result<(), PhysicsError> {
        require_finite_vec3(
            self.position_eci_m,
            "ascent position components must be finite",
        )?;
        require_finite_vec3(
            self.velocity_eci_m_s,
            "ascent velocity components must be finite",
        )?;
        if !self.altitude_m.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "ascent altitude must be finite",
            });
        }
        if !self.inertial_speed_m_s.is_finite() || self.inertial_speed_m_s < 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "ascent inertial speed must be finite and non-negative",
            });
        }
        if !self.flight_path_angle_rad.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "ascent flight-path angle must be finite",
            });
        }
        if !self.dynamic_pressure_pa.is_finite() || self.dynamic_pressure_pa < 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "ascent dynamic pressure must be finite and non-negative",
            });
        }
        if !self.mass_fraction.is_finite() || !(0.0..=1.0).contains(&self.mass_fraction) {
            return Err(PhysicsError::InvalidParameter {
                reason: "ascent mass fraction must be finite and in [0, 1]",
            });
        }
        Ok(())
    }
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

/// Pitch-program ascent reference.
///
/// The program linearly interpolates pitch angle from the vertical in
/// a fixed inertial x-z plane, with body `+x` aligned to the resulting
/// reference direction. Times before the first schedule entry clamp to
/// the first pitch; times after the final entry clamp to the final
/// pitch.
#[derive(Clone, Debug, PartialEq)]
pub struct PitchProgramAscentReference {
    schedule_s: Vec<f64>,
    pitch_rad: Vec<f64>,
}

impl PitchProgramAscentReference {
    /// Construct a validated pitch-program reference generator.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when the schedule and pitch arrays
    /// differ in length, contain fewer than two entries, contain
    /// non-finite values, or the schedule is not strictly increasing.
    pub fn new(schedule_s: Vec<f64>, pitch_rad: Vec<f64>) -> Result<Self, PhysicsError> {
        validate_pitch_program(&schedule_s, &pitch_rad)?;
        Ok(Self {
            schedule_s,
            pitch_rad,
        })
    }

    /// Schedule timestamps (s).
    #[must_use]
    pub fn schedule_s(&self) -> &[f64] {
        &self.schedule_s
    }

    /// Pitch values (rad), one per schedule timestamp.
    #[must_use]
    pub fn pitch_rad(&self) -> &[f64] {
        &self.pitch_rad
    }

    /// Interpolated pitch angle at `time`.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] if `time` is non-finite. Constructor
    /// validation guarantees the interpolation intervals themselves are
    /// usable.
    pub fn pitch_at(&self, time: SimTime) -> Result<f64, PhysicsError> {
        let t = time.as_seconds();
        if !t.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "ascent reference time must be finite",
            });
        }
        if t <= self.schedule_s[0] {
            return Ok(self.pitch_rad[0]);
        }
        let last = self.schedule_s.len() - 1;
        if t >= self.schedule_s[last] {
            return Ok(self.pitch_rad[last]);
        }
        for index in 0..last {
            let t0 = self.schedule_s[index];
            let t1 = self.schedule_s[index + 1];
            if t >= t0 && t <= t1 {
                let alpha = (t - t0) / (t1 - t0);
                return Ok(self.pitch_rad[index]
                    + alpha * (self.pitch_rad[index + 1] - self.pitch_rad[index]));
            }
        }
        Err(PhysicsError::OutOfEnvelope {
            reason: "ascent reference time did not fall in a pitch-program interval",
        })
    }
}

impl AscentReferenceGenerator for PitchProgramAscentReference {
    fn ascent_reference(
        &self,
        state: &AscentState,
        time: SimTime,
    ) -> Result<AscentReference, PhysicsError> {
        state.validate()?;
        let pitch = self.pitch_at(time)?;
        let forward_eci = [pitch.sin(), 0.0, pitch.cos()];
        let q_body_to_eci_xyzw = reference_quaternion_from_body_x(forward_eci)?;
        Ok(AscentReference {
            q_body_to_eci_xyzw,
            body_rate_rad_s: None,
        })
    }
}

/// Gravity-turn ascent reference.
///
/// Once the inertial speed exceeds the configured minimum, body `+x`
/// is aligned with the inertial velocity vector. The generator derives
/// an attitude reference only from the vehicle state.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GravityTurnAscentReference {
    minimum_speed_m_s: f64,
}

impl GravityTurnAscentReference {
    /// Construct a gravity-turn reference generator.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when `minimum_speed_m_s` is non-finite
    /// or negative.
    pub fn new(minimum_speed_m_s: f64) -> Result<Self, PhysicsError> {
        if !minimum_speed_m_s.is_finite() || minimum_speed_m_s < 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "gravity-turn minimum speed must be finite and non-negative",
            });
        }
        Ok(Self { minimum_speed_m_s })
    }

    /// Minimum speed before body `+x` can align with velocity (m/s).
    #[must_use]
    pub const fn minimum_speed_m_s(&self) -> f64 {
        self.minimum_speed_m_s
    }
}

impl Default for GravityTurnAscentReference {
    fn default() -> Self {
        Self {
            minimum_speed_m_s: GRAVITY_TURN_MINIMUM_SPEED_M_S,
        }
    }
}

impl AscentReferenceGenerator for GravityTurnAscentReference {
    fn ascent_reference(
        &self,
        state: &AscentState,
        _time: SimTime,
    ) -> Result<AscentReference, PhysicsError> {
        state.validate()?;
        let speed = vector_norm(state.velocity_eci_m_s);
        if speed < self.minimum_speed_m_s {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "gravity-turn reference requires non-zero inertial speed",
            });
        }
        let q_body_to_eci_xyzw = reference_quaternion_from_body_x(state.velocity_eci_m_s)?;
        Ok(AscentReference {
            q_body_to_eci_xyzw,
            body_rate_rad_s: None,
        })
    }
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

impl BallisticState {
    /// Validate the state carried into a landing-footprint
    /// prediction.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when any vector component is
    /// non-finite, the ballistic coefficient is negative /
    /// non-finite, or the timestamp is non-finite.
    pub fn validate(&self) -> Result<(), PhysicsError> {
        require_finite_vec3(
            self.position_eci_m,
            "ballistic position components must be finite",
        )?;
        require_finite_vec3(
            self.velocity_eci_m_s,
            "ballistic velocity components must be finite",
        )?;
        if !self.ballistic_coefficient_m2_kg.is_finite() || self.ballistic_coefficient_m2_kg < 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "ballistic coefficient must be finite and non-negative",
            });
        }
        if !self.time.as_seconds().is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "ballistic state time must be finite",
            });
        }
        Ok(())
    }
}

/// Optional geodetic launch origin used only to project a
/// range-relative footprint onto recovery-map coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FootprintGeodeticOrigin {
    /// Launch-site latitude in degrees.
    pub latitude_deg: f64,
    /// Launch-site longitude in degrees.
    pub longitude_deg: f64,
    /// Launch-site height above the WGS84 ellipsoid (m).
    pub height_m: f64,
}

impl FootprintGeodeticOrigin {
    /// Validate the launch-origin geodetic fields.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when latitude / longitude are outside
    /// their conventional ranges or any field is non-finite.
    pub fn validate(&self) -> Result<(), PhysicsError> {
        if !self.latitude_deg.is_finite() || !(-90.0..=90.0).contains(&self.latitude_deg) {
            return Err(PhysicsError::InvalidParameter {
                reason: "footprint origin latitude must be finite and in [-90, 90] degrees",
            });
        }
        if !self.longitude_deg.is_finite() || !(-180.0..=180.0).contains(&self.longitude_deg) {
            return Err(PhysicsError::InvalidParameter {
                reason: "footprint origin longitude must be finite and in [-180, 180] degrees",
            });
        }
        if !self.height_m.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "footprint origin height must be finite",
            });
        }
        Ok(())
    }
}

/// Declared one-sigma landing dispersion input for an offline
/// footprint report.
///
/// This is a declared analysis input, not an accuracy promise and not
/// a comparison to any desired landing location.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FootprintDispersionInput {
    /// One-sigma semi-major axis (m).
    pub one_sigma_semi_major_m: f64,
    /// One-sigma semi-minor axis (m).
    pub one_sigma_semi_minor_m: f64,
    /// Ellipse orientation in the downrange/crossrange plane (rad).
    pub orientation_rad: f64,
}

impl FootprintDispersionInput {
    /// Validate the declared dispersion ellipse.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when axes are non-positive /
    /// non-finite, the semi-major axis is smaller than the
    /// semi-minor axis, or orientation is non-finite.
    pub fn validate(&self) -> Result<(), PhysicsError> {
        require_positive_length(
            self.one_sigma_semi_major_m,
            "footprint one-sigma semi-major axis must be finite and positive",
        )?;
        require_positive_length(
            self.one_sigma_semi_minor_m,
            "footprint one-sigma semi-minor axis must be finite and positive",
        )?;
        if self.one_sigma_semi_major_m < self.one_sigma_semi_minor_m {
            return Err(PhysicsError::InvalidParameter {
                reason: "footprint semi-major axis must be >= semi-minor axis",
            });
        }
        if !self.orientation_rad.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "footprint dispersion orientation must be finite",
            });
        }
        Ok(())
    }
}

/// Environment selection for a footprint propagation (which gravity and
/// atmosphere envelopes apply, and the optional launch-site origin used
/// only to map a range-relative prediction onto recovery coordinates).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FootprintEnvironment {
    /// Cull altitude (m): propagation stops at or below this altitude
    /// (typically ground level, 0.0).
    pub cull_altitude_m: f64,
    /// Constant downward acceleration magnitude used by the
    /// constant-gravity footprint method (m/s²).
    pub gravity_m_s2: f64,
    /// Launch-origin inertial position `[x, y, z]` (m). Nominal
    /// downrange/crossrange values are reported relative to this
    /// origin in the local x/y plane.
    pub launch_origin_eci_m: [f64; 3],
    /// Optional geodetic launch origin. When absent, geodetic
    /// latitude/longitude output remains `None`; the model never
    /// fabricates geographic coordinates.
    pub geodetic_origin: Option<FootprintGeodeticOrigin>,
    /// Optional declared dispersion input. When absent, the
    /// resulting footprint has no dispersion ellipse rather than a
    /// misleading zero-width ellipse.
    pub dispersion: Option<FootprintDispersionInput>,
}

impl FootprintEnvironment {
    /// Validate the environment carried into a landing-footprint
    /// prediction.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when the cull altitude, gravity,
    /// launch origin, geodetic origin, or dispersion declaration is
    /// invalid.
    pub fn validate(&self) -> Result<(), PhysicsError> {
        if !self.cull_altitude_m.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "footprint cull altitude must be finite",
            });
        }
        if !self.gravity_m_s2.is_finite() || self.gravity_m_s2 <= MIN_FOOTPRINT_GRAVITY_M_S2 {
            return Err(PhysicsError::InvalidParameter {
                reason: "footprint gravity must be finite and strictly positive",
            });
        }
        require_finite_vec3(
            self.launch_origin_eci_m,
            "footprint launch-origin components must be finite",
        )?;
        if let Some(origin) = self.geodetic_origin {
            origin.validate()?;
        }
        if let Some(dispersion) = self.dispersion {
            dispersion.validate()?;
        }
        Ok(())
    }
}

/// Dispersion ellipse attached to a landing-footprint report.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FootprintDispersionEllipse {
    /// One-sigma semi-major axis (m).
    pub one_sigma_semi_major_m: f64,
    /// One-sigma semi-minor axis (m).
    pub one_sigma_semi_minor_m: f64,
    /// Three-sigma semi-major axis (m).
    pub three_sigma_semi_major_m: f64,
    /// Three-sigma semi-minor axis (m).
    pub three_sigma_semi_minor_m: f64,
    /// Ellipse orientation in the downrange/crossrange plane (rad).
    pub orientation_rad: f64,
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
    /// Bearing in the downrange/crossrange plane, computed as
    /// `atan2(crossrange, downrange)` (rad).
    pub bearing_rad: f64,
    /// Time from the seed state to the cull-altitude crossing (s).
    pub time_to_cull_s: f64,
    /// Optional predicted geodetic latitude in degrees, present only
    /// when the environment provides a geodetic launch origin.
    pub latitude_deg: Option<f64>,
    /// Optional predicted geodetic longitude in degrees, present only
    /// when the environment provides a geodetic launch origin.
    pub longitude_deg: Option<f64>,
    /// Optional declared dispersion ellipse.
    pub dispersion_ellipse: Option<FootprintDispersionEllipse>,
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

/// Constant-gravity closed-form footprint model.
///
/// This is the first consumed offline footprint implementation: a
/// deterministic, forward-only toy model for short-range scenarios
/// whose environment explicitly selects constant gravity. It solves
/// the vertical constant-acceleration crossing to the environment's
/// cull altitude, advances horizontal motion linearly, and reports
/// range-relative output. It does not accept a desired landing
/// location and it produces no control command.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ConstantGravityRangeSafetyFootprint;

impl RangeSafetyFootprint for ConstantGravityRangeSafetyFootprint {
    fn landing_footprint(
        &self,
        state: &BallisticState,
        env: &FootprintEnvironment,
    ) -> Result<LandingFootprint, PhysicsError> {
        state.validate()?;
        env.validate()?;
        let time_to_cull_s = constant_gravity_time_to_cull_s(state, env)?;
        let landing_xy_m = [
            state.position_eci_m[0] + state.velocity_eci_m_s[0] * time_to_cull_s,
            state.position_eci_m[1] + state.velocity_eci_m_s[1] * time_to_cull_s,
        ];
        let downrange_m = landing_xy_m[0] - env.launch_origin_eci_m[0];
        let crossrange_m = landing_xy_m[1] - env.launch_origin_eci_m[1];
        let bearing_rad = if downrange_m == 0.0 && crossrange_m == 0.0 {
            0.0
        } else {
            crossrange_m.atan2(downrange_m)
        };
        let (latitude_deg, longitude_deg) = match env.geodetic_origin {
            Some(origin) => geodetic_from_range_relative(origin, downrange_m, crossrange_m)?,
            None => (None, None),
        };
        Ok(LandingFootprint {
            downrange_m,
            crossrange_m,
            bearing_rad,
            time_to_cull_s,
            latitude_deg,
            longitude_deg,
            dispersion_ellipse: env.dispersion.map(dispersion_ellipse),
        })
    }
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

impl EntryCorridor {
    /// Validate entry-corridor limits.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when any limit is non-finite or not
    /// strictly positive, or when the flight-path-angle band is
    /// outside `(0, π/2)`.
    pub fn validate(&self) -> Result<(), PhysicsError> {
        require_positive_length(
            self.max_heat_rate_w_m2,
            "entry-corridor heat-rate limit must be finite and positive",
        )?;
        require_positive_length(
            self.max_load_factor_g,
            "entry-corridor load-factor limit must be finite and positive",
        )?;
        if !self.flight_path_angle_band_rad.is_finite()
            || self.flight_path_angle_band_rad <= MIN_ENTRY_CORRIDOR_BAND_RAD
            || self.flight_path_angle_band_rad >= core::f64::consts::FRAC_PI_2
        {
            return Err(PhysicsError::InvalidParameter {
                reason: "entry-corridor flight-path-angle band must be in (0, π/2)",
            });
        }
        Ok(())
    }
}

/// Instantaneous state sampled during entry-corridor reference
/// generation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EntryState {
    /// Geometric altitude (m).
    pub altitude_m: f64,
    /// Inertial speed magnitude (m/s).
    pub velocity_m_s: f64,
    /// Flight-path angle (rad), positive upward.
    pub flight_path_angle_rad: f64,
    /// Current stagnation-point heat rate estimate (W/m²).
    pub heat_rate_w_m2: f64,
    /// Current deceleration load factor (g).
    pub load_factor_g: f64,
}

impl EntryState {
    /// Validate the sampled entry state.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when any field is non-finite, altitude
    /// or diagnostic scalars are negative, velocity is non-positive,
    /// or flight-path angle is outside `(-π/2, π/2)`.
    pub fn validate(&self) -> Result<(), PhysicsError> {
        if !self.altitude_m.is_finite() || self.altitude_m < 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "entry altitude must be finite and non-negative",
            });
        }
        if !self.velocity_m_s.is_finite() || self.velocity_m_s <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "entry velocity must be finite and positive",
            });
        }
        if !self.flight_path_angle_rad.is_finite()
            || self.flight_path_angle_rad.abs() >= core::f64::consts::FRAC_PI_2
        {
            return Err(PhysicsError::InvalidParameter {
                reason: "entry flight-path angle must be finite and in (-π/2, π/2)",
            });
        }
        if !self.heat_rate_w_m2.is_finite() || self.heat_rate_w_m2 < 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "entry heat rate must be finite and non-negative",
            });
        }
        if !self.load_factor_g.is_finite() || self.load_factor_g < 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "entry load factor must be finite and non-negative",
            });
        }
        Ok(())
    }
}

/// Corridor-reference output for a lifting entry.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EntryCorridorReferenceOutput {
    /// Reference bank angle (rad).
    pub bank_angle_rad: f64,
    /// Positive heat-rate margin to the configured corridor limit (W/m²).
    pub heat_rate_margin_w_m2: f64,
    /// Positive load-factor margin to the configured corridor limit (g).
    pub load_factor_margin_g: f64,
    /// Positive flight-path-angle margin to the configured band (rad).
    pub flight_path_angle_margin_rad: f64,
}

/// Produces a bank-angle reference for a lifting entry from corridor
/// limits and the current entry state. Corridor-driven, never
/// target-driven.
pub trait EntryCorridorReference {
    /// Reference bank angle and corridor margins for the current
    /// entry state.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when the state lies outside the
    /// corridor's feasible band.
    fn bank_reference(
        &self,
        corridor: &EntryCorridor,
        state: &EntryState,
        time: SimTime,
    ) -> Result<EntryCorridorReferenceOutput, PhysicsError>;
}

/// Simple bounded bank-angle entry-corridor reference.
///
/// The command is proportional only to flight-path angle within the
/// declared corridor and is saturated by `max_bank_rad`. Heat-rate and
/// load-factor limits are fail-closed envelope checks, not targets.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BandLimitedEntryCorridorReference {
    nominal_bank_rad: f64,
    max_bank_rad: f64,
}

impl BandLimitedEntryCorridorReference {
    /// Construct a bounded entry-corridor reference.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when bank angles are non-finite,
    /// `max_bank_rad` is not positive, or the nominal bank exceeds
    /// the absolute bank limit.
    pub fn new(nominal_bank_rad: f64, max_bank_rad: f64) -> Result<Self, PhysicsError> {
        if !nominal_bank_rad.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "entry nominal bank angle must be finite",
            });
        }
        if !max_bank_rad.is_finite()
            || max_bank_rad <= MIN_ENTRY_BANK_LIMIT_RAD
            || max_bank_rad > core::f64::consts::PI
        {
            return Err(PhysicsError::InvalidParameter {
                reason: "entry maximum bank angle must be in (0, π]",
            });
        }
        if nominal_bank_rad.abs() > max_bank_rad {
            return Err(PhysicsError::InvalidParameter {
                reason: "entry nominal bank angle must not exceed max_bank_rad",
            });
        }
        Ok(Self {
            nominal_bank_rad,
            max_bank_rad,
        })
    }

    /// Nominal bank angle (rad).
    #[must_use]
    pub const fn nominal_bank_rad(&self) -> f64 {
        self.nominal_bank_rad
    }

    /// Maximum absolute bank angle (rad).
    #[must_use]
    pub const fn max_bank_rad(&self) -> f64 {
        self.max_bank_rad
    }
}

impl Default for BandLimitedEntryCorridorReference {
    fn default() -> Self {
        Self {
            nominal_bank_rad: 0.0,
            max_bank_rad: core::f64::consts::FRAC_PI_2,
        }
    }
}

impl EntryCorridorReference for BandLimitedEntryCorridorReference {
    fn bank_reference(
        &self,
        corridor: &EntryCorridor,
        state: &EntryState,
        time: SimTime,
    ) -> Result<EntryCorridorReferenceOutput, PhysicsError> {
        corridor.validate()?;
        state.validate()?;
        if !time.as_seconds().is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "entry reference time must be finite",
            });
        }
        let heat_rate_margin_w_m2 = corridor.max_heat_rate_w_m2 - state.heat_rate_w_m2;
        let load_factor_margin_g = corridor.max_load_factor_g - state.load_factor_g;
        let flight_path_angle_margin_rad =
            corridor.flight_path_angle_band_rad - state.flight_path_angle_rad.abs();
        if heat_rate_margin_w_m2 < 0.0 {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "entry state exceeds heat-rate corridor",
            });
        }
        if load_factor_margin_g < 0.0 {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "entry state exceeds load-factor corridor",
            });
        }
        if flight_path_angle_margin_rad < 0.0 {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "entry state exceeds flight-path-angle corridor",
            });
        }
        let normalized_gamma = state.flight_path_angle_rad / corridor.flight_path_angle_band_rad;
        let bank_angle_rad = (self.nominal_bank_rad - normalized_gamma * self.max_bank_rad)
            .clamp(-self.max_bank_rad, self.max_bank_rad);
        Ok(EntryCorridorReferenceOutput {
            bank_angle_rad,
            heat_rate_margin_w_m2,
            load_factor_margin_g,
            flight_path_angle_margin_rad,
        })
    }
}

fn require_positive_mass(value: f64, reason: &'static str) -> Result<(), PhysicsError> {
    if value.is_finite() && value > 0.0 {
        return Ok(());
    }
    Err(PhysicsError::InvalidParameter { reason })
}

fn require_positive_length(value: f64, reason: &'static str) -> Result<(), PhysicsError> {
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

fn constant_gravity_time_to_cull_s(
    state: &BallisticState,
    env: &FootprintEnvironment,
) -> Result<f64, PhysicsError> {
    let height_above_cull_m = state.position_eci_m[2] - env.cull_altitude_m;
    if !height_above_cull_m.is_finite() || height_above_cull_m <= 0.0 {
        return Err(PhysicsError::OutOfEnvelope {
            reason: "ballistic state must start above the footprint cull altitude",
        });
    }
    let vertical_velocity_m_s = state.velocity_eci_m_s[2];
    let discriminant = vertical_velocity_m_s * vertical_velocity_m_s
        + 2.0 * env.gravity_m_s2 * height_above_cull_m;
    if !discriminant.is_finite() || discriminant < 0.0 {
        return Err(PhysicsError::NonFinite {
            reason: "constant-gravity footprint time discriminant is invalid",
        });
    }
    let time_to_cull_s = (vertical_velocity_m_s + discriminant.sqrt()) / env.gravity_m_s2;
    if !time_to_cull_s.is_finite() || time_to_cull_s < 0.0 {
        return Err(PhysicsError::NonFinite {
            reason: "constant-gravity footprint crossing time is invalid",
        });
    }
    Ok(time_to_cull_s)
}

fn geodetic_from_range_relative(
    origin: FootprintGeodeticOrigin,
    downrange_m: f64,
    crossrange_m: f64,
) -> Result<(Option<f64>, Option<f64>), PhysicsError> {
    origin.validate()?;
    let latitude_rad = origin.latitude_deg.to_radians();
    let cos_latitude = latitude_rad.cos();
    if cos_latitude.abs() <= MIN_LONGITUDE_COSINE {
        return Err(PhysicsError::OutOfEnvelope {
            reason: "footprint geodetic longitude projection is singular at this latitude",
        });
    }
    let latitude_deg =
        origin.latitude_deg + crossrange_m / WGS84_A_M * 180.0 / core::f64::consts::PI;
    let longitude_deg = origin.longitude_deg
        + downrange_m / (WGS84_A_M * cos_latitude) * 180.0 / core::f64::consts::PI;
    if !latitude_deg.is_finite() || !longitude_deg.is_finite() {
        return Err(PhysicsError::NonFinite {
            reason: "footprint geodetic projection produced non-finite coordinates",
        });
    }
    Ok((
        Some(latitude_deg),
        Some(normalize_longitude_deg(longitude_deg)),
    ))
}

fn normalize_longitude_deg(longitude_deg: f64) -> f64 {
    let mut wrapped = longitude_deg;
    while wrapped > 180.0 {
        wrapped -= 360.0;
    }
    while wrapped < -180.0 {
        wrapped += 360.0;
    }
    wrapped
}

fn dispersion_ellipse(input: FootprintDispersionInput) -> FootprintDispersionEllipse {
    FootprintDispersionEllipse {
        one_sigma_semi_major_m: input.one_sigma_semi_major_m,
        one_sigma_semi_minor_m: input.one_sigma_semi_minor_m,
        three_sigma_semi_major_m: 3.0 * input.one_sigma_semi_major_m,
        three_sigma_semi_minor_m: 3.0 * input.one_sigma_semi_minor_m,
        orientation_rad: input.orientation_rad,
    }
}

fn validate_pitch_program(schedule_s: &[f64], pitch_rad: &[f64]) -> Result<(), PhysicsError> {
    if schedule_s.len() != pitch_rad.len() {
        return Err(PhysicsError::InvalidParameter {
            reason: "pitch-program schedule and pitch arrays must have equal length",
        });
    }
    if schedule_s.len() < 2 {
        return Err(PhysicsError::InvalidParameter {
            reason: "pitch-program schedule requires at least two entries",
        });
    }
    for index in 0..schedule_s.len() {
        if !schedule_s[index].is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "pitch-program schedule values must be finite",
            });
        }
        if !pitch_rad[index].is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "pitch-program pitch values must be finite",
            });
        }
        if index > 0 && schedule_s[index] <= schedule_s[index - 1] {
            return Err(PhysicsError::InvalidParameter {
                reason: "pitch-program schedule must be strictly increasing",
            });
        }
    }
    Ok(())
}

fn reference_quaternion_from_body_x(forward_eci: [f64; 3]) -> Result<[f64; 4], PhysicsError> {
    require_finite_vec3(
        forward_eci,
        "ascent reference direction components must be finite",
    )?;
    let norm = vector_norm(forward_eci);
    if norm <= MIN_DIRECTION_NORM {
        return Err(PhysicsError::OutOfEnvelope {
            reason: "ascent reference direction norm is too small",
        });
    }
    let body_x = Vector3::new(
        forward_eci[0] / norm,
        forward_eci[1] / norm,
        forward_eci[2] / norm,
    );
    let mut reference_y = Vector3::new(0.0, 1.0, 0.0);
    let projected_y = reference_y - body_x * body_x.dot(&reference_y);
    let projected_y_norm = projected_y.norm();
    let body_y = if projected_y_norm > MIN_DIRECTION_NORM {
        projected_y / projected_y_norm
    } else {
        reference_y = Vector3::new(0.0, 0.0, 1.0);
        let fallback_y = reference_y - body_x * body_x.dot(&reference_y);
        let fallback_y_norm = fallback_y.norm();
        if fallback_y_norm <= MIN_DIRECTION_NORM {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "ascent reference could not construct an orthonormal frame",
            });
        }
        fallback_y / fallback_y_norm
    };
    let body_z = body_x.cross(&body_y);
    let matrix = Matrix3::from_columns(&[body_x, body_y, body_z]);
    let rotation = Rotation3::from_matrix_unchecked(matrix);
    let q = UnitQuaternion::from_rotation_matrix(&rotation);
    let q_body_to_eci_xyzw = [q.i, q.j, q.k, q.w];
    validate_reference_quaternion(q_body_to_eci_xyzw)?;
    Ok(q_body_to_eci_xyzw)
}

fn vector_norm(value: [f64; 3]) -> f64 {
    let mut sum = 0.0_f64;
    sum += value[0] * value[0];
    sum += value[1] * value[1];
    sum += value[2] * value[2];
    sum.sqrt()
}

fn validate_reference_quaternion(q_xyzw: [f64; 4]) -> Result<(), PhysicsError> {
    if !q_xyzw.iter().all(|component| component.is_finite()) {
        return Err(PhysicsError::NonFinite {
            reason: "ascent reference quaternion components must be finite",
        });
    }
    let mut norm_sq = 0.0_f64;
    norm_sq += q_xyzw[0] * q_xyzw[0];
    norm_sq += q_xyzw[1] * q_xyzw[1];
    norm_sq += q_xyzw[2] * q_xyzw[2];
    norm_sq += q_xyzw[3] * q_xyzw[3];
    if (norm_sq - 1.0).abs() > QUATERNION_NORM_TOLERANCE {
        return Err(PhysicsError::NonFinite {
            reason: "ascent reference quaternion must be unit length",
        });
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::float_cmp, clippy::unwrap_used)]
mod tests {
    use super::{
        AscentReferenceGenerator, AscentState, BallisticState, BandLimitedEntryCorridorReference,
        ConstantGravityRangeSafetyFootprint, EntryCorridor, EntryCorridorReference, EntryState,
        FootprintDispersionInput, FootprintEnvironment, FootprintGeodeticOrigin,
        GravityTurnAscentReference, MomentumConservingStageSeparation, PitchProgramAscentReference,
        RangeSafetyFootprint, STAGE_SEPARATION_MOMENTUM_TOLERANCE_KG_M_S, StageSeparationModel,
    };
    use crate::PhysicsError;
    use nalgebra::{Quaternion, UnitQuaternion, Vector3};
    use openbmp_core::SimTime;

    fn nominal_ascent_state() -> AscentState {
        AscentState {
            position_eci_m: [0.0, 0.0, 100.0],
            velocity_eci_m_s: [10.0, 0.0, 100.0],
            altitude_m: 100.0,
            inertial_speed_m_s: 100.498_756_211_208_9,
            flight_path_angle_rad: 1.471_127_674_303_734_7,
            dynamic_pressure_pa: 0.0,
            mass_fraction: 1.0,
        }
    }

    fn nominal_footprint_env() -> FootprintEnvironment {
        FootprintEnvironment {
            cull_altitude_m: 0.0,
            gravity_m_s2: 10.0,
            launch_origin_eci_m: [0.0, 0.0, 0.0],
            geodetic_origin: None,
            dispersion: None,
        }
    }

    fn body_x_axis(q_xyzw: [f64; 4]) -> Vector3<f64> {
        let q = Quaternion::new(q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]);
        UnitQuaternion::new_normalize(q).transform_vector(&Vector3::new(1.0, 0.0, 0.0))
    }

    #[test]
    fn pitch_program_interpolates_and_clamps() {
        let program =
            PitchProgramAscentReference::new(vec![0.0, 10.0, 30.0], vec![0.0, 0.2, 0.6]).unwrap();
        assert_eq!(program.pitch_at(SimTime::from_seconds(-1.0)).unwrap(), 0.0);
        assert_eq!(program.pitch_at(SimTime::from_seconds(40.0)).unwrap(), 0.6);
        assert_eq!(program.pitch_at(SimTime::from_seconds(20.0)).unwrap(), 0.4);
    }

    #[test]
    fn pitch_program_reference_aligns_body_x_with_pitch_direction() {
        let program = PitchProgramAscentReference::new(vec![0.0, 10.0], vec![0.0, 0.4]).unwrap();
        let reference = program
            .ascent_reference(&nominal_ascent_state(), SimTime::from_seconds(5.0))
            .unwrap();
        let body_x = body_x_axis(reference.q_body_to_eci_xyzw);
        let expected_pitch = 0.2_f64;
        let expected = Vector3::new(expected_pitch.sin(), 0.0, expected_pitch.cos());
        assert!((body_x - expected).norm() < 1.0e-12);
    }

    #[test]
    fn pitch_program_rejects_non_monotonic_schedule() {
        let err = PitchProgramAscentReference::new(vec![0.0, 10.0, 10.0], vec![0.0, 0.1, 0.2])
            .unwrap_err();
        assert!(matches!(err, PhysicsError::InvalidParameter { .. }));
    }

    #[test]
    fn gravity_turn_aligns_body_x_with_inertial_velocity() {
        let reference = GravityTurnAscentReference::default()
            .ascent_reference(&nominal_ascent_state(), SimTime::from_seconds(1.0))
            .unwrap();
        let body_x = body_x_axis(reference.q_body_to_eci_xyzw);
        let expected = Vector3::new(10.0, 0.0, 100.0).normalize();
        assert!((body_x - expected).norm() < 1.0e-12);
    }

    #[test]
    fn gravity_turn_rejects_zero_velocity() {
        let mut state = nominal_ascent_state();
        state.velocity_eci_m_s = [0.0, 0.0, 0.0];
        state.inertial_speed_m_s = 0.0;
        let err = GravityTurnAscentReference::default()
            .ascent_reference(&state, SimTime::from_seconds(1.0))
            .unwrap_err();
        assert!(matches!(err, PhysicsError::OutOfEnvelope { .. }));
    }

    #[test]
    fn constant_gravity_footprint_predicts_range_relative_landing() {
        let state = BallisticState {
            position_eci_m: [0.0, 0.0, 100.0],
            velocity_eci_m_s: [20.0, -5.0, 0.0],
            ballistic_coefficient_m2_kg: 0.0,
            time: SimTime::from_seconds(12.0),
        };
        let footprint = ConstantGravityRangeSafetyFootprint
            .landing_footprint(&state, &nominal_footprint_env())
            .unwrap();
        let expected_time = (2.0_f64 * 100.0 / 10.0).sqrt();
        assert!((footprint.time_to_cull_s - expected_time).abs() < 1.0e-12);
        assert!((footprint.downrange_m - 20.0 * expected_time).abs() < 1.0e-12);
        assert!((footprint.crossrange_m + 5.0 * expected_time).abs() < 1.0e-12);
        assert!(footprint.latitude_deg.is_none());
        assert!(footprint.longitude_deg.is_none());
        assert!(footprint.dispersion_ellipse.is_none());
    }

    #[test]
    fn constant_gravity_footprint_emits_optional_geodetic_and_dispersion() {
        let state = BallisticState {
            position_eci_m: [0.0, 0.0, 100.0],
            velocity_eci_m_s: [10.0, 0.0, 0.0],
            ballistic_coefficient_m2_kg: 0.0,
            time: SimTime::from_seconds(0.0),
        };
        let env = FootprintEnvironment {
            geodetic_origin: Some(FootprintGeodeticOrigin {
                latitude_deg: 50.0,
                longitude_deg: 30.0,
                height_m: 100.0,
            }),
            dispersion: Some(FootprintDispersionInput {
                one_sigma_semi_major_m: 30.0,
                one_sigma_semi_minor_m: 10.0,
                orientation_rad: 0.25,
            }),
            ..nominal_footprint_env()
        };
        let footprint = ConstantGravityRangeSafetyFootprint
            .landing_footprint(&state, &env)
            .unwrap();
        assert_eq!(footprint.latitude_deg.unwrap(), 50.0);
        assert!(footprint.longitude_deg.unwrap() > 30.0);
        let dispersion = footprint.dispersion_ellipse.unwrap();
        assert_eq!(dispersion.one_sigma_semi_major_m, 30.0);
        assert_eq!(dispersion.one_sigma_semi_minor_m, 10.0);
        assert_eq!(dispersion.three_sigma_semi_major_m, 90.0);
        assert_eq!(dispersion.three_sigma_semi_minor_m, 30.0);
        assert_eq!(dispersion.orientation_rad, 0.25);
    }

    #[test]
    fn footprint_rejects_state_below_cull_altitude() {
        let state = BallisticState {
            position_eci_m: [0.0, 0.0, 0.0],
            velocity_eci_m_s: [0.0, 0.0, -1.0],
            ballistic_coefficient_m2_kg: 0.0,
            time: SimTime::from_seconds(0.0),
        };
        let err = ConstantGravityRangeSafetyFootprint
            .landing_footprint(&state, &nominal_footprint_env())
            .unwrap_err();
        assert!(matches!(err, PhysicsError::OutOfEnvelope { .. }));
    }

    #[test]
    fn footprint_rejects_degenerate_dispersion() {
        let env = FootprintEnvironment {
            dispersion: Some(FootprintDispersionInput {
                one_sigma_semi_major_m: 1.0,
                one_sigma_semi_minor_m: 2.0,
                orientation_rad: 0.0,
            }),
            ..nominal_footprint_env()
        };
        let state = BallisticState {
            position_eci_m: [0.0, 0.0, 100.0],
            velocity_eci_m_s: [0.0, 0.0, 0.0],
            ballistic_coefficient_m2_kg: 0.0,
            time: SimTime::from_seconds(0.0),
        };
        let err = ConstantGravityRangeSafetyFootprint
            .landing_footprint(&state, &env)
            .unwrap_err();
        assert!(matches!(err, PhysicsError::InvalidParameter { .. }));
    }

    fn nominal_entry_corridor() -> EntryCorridor {
        EntryCorridor {
            max_heat_rate_w_m2: 1.0e6,
            max_load_factor_g: 8.0,
            flight_path_angle_band_rad: 0.2,
        }
    }

    fn nominal_entry_state() -> EntryState {
        EntryState {
            altitude_m: 80_000.0,
            velocity_m_s: 7_800.0,
            flight_path_angle_rad: -0.05,
            heat_rate_w_m2: 2.5e5,
            load_factor_g: 2.0,
        }
    }

    #[test]
    fn entry_corridor_reference_reports_bank_and_margins() {
        let reference = BandLimitedEntryCorridorReference::new(0.0, 1.0).unwrap();
        let output = reference
            .bank_reference(
                &nominal_entry_corridor(),
                &nominal_entry_state(),
                SimTime::from_seconds(10.0),
            )
            .unwrap();
        assert!(output.bank_angle_rad > 0.0);
        assert_eq!(output.heat_rate_margin_w_m2, 750_000.0);
        assert_eq!(output.load_factor_margin_g, 6.0);
        assert!((output.flight_path_angle_margin_rad - 0.15).abs() < 1.0e-12);
    }

    #[test]
    fn entry_corridor_rejects_excess_heat_load_or_angle() {
        let reference = BandLimitedEntryCorridorReference::default();
        let mut state = nominal_entry_state();
        state.heat_rate_w_m2 = 2.0e6;
        let err = reference
            .bank_reference(
                &nominal_entry_corridor(),
                &state,
                SimTime::from_seconds(10.0),
            )
            .unwrap_err();
        assert!(matches!(err, PhysicsError::OutOfEnvelope { .. }));

        state = nominal_entry_state();
        state.flight_path_angle_rad = -0.5;
        let err = reference
            .bank_reference(
                &nominal_entry_corridor(),
                &state,
                SimTime::from_seconds(10.0),
            )
            .unwrap_err();
        assert!(matches!(err, PhysicsError::OutOfEnvelope { .. }));
    }

    #[test]
    fn entry_corridor_constructor_rejects_invalid_bank_limits() {
        let err = BandLimitedEntryCorridorReference::new(2.0, 1.0).unwrap_err();
        assert!(matches!(err, PhysicsError::InvalidParameter { .. }));
    }

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
