//! Flight-profile trait surfaces and consumed profile helpers.
//!
//! Trait definitions and consumed profile helpers for the multi-phase
//! flight-profile work described in
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
use openbmp_core::{Eci, Position3, SimTime};

use crate::error::PhysicsError;
use crate::frames::{
    FrameContext, LocalGeodeticOrigin, WGS84_A_M, WGS84_ECCENTRICITY_SQUARED, WGS84_FLATTENING,
};
use crate::gravity::GravityModel;

const MIN_DIRECTION_NORM: f64 = 1.0e-12;
const QUATERNION_NORM_TOLERANCE: f64 = 1.0e-9;
const MIN_FOOTPRINT_GRAVITY_M_S2: f64 = 1.0e-12;
const MIN_NUMERICAL_FOOTPRINT_STEP_S: f64 = 1.0e-6;
const DEFAULT_NUMERICAL_FOOTPRINT_STEP_S: f64 = 1.0;
const DEFAULT_NUMERICAL_FOOTPRINT_MAX_TIME_S: f64 = 86_400.0;
const DEFAULT_DRAG_WIND_FOOTPRINT_STEP_S: f64 = 0.25;
const DEFAULT_DRAG_WIND_FOOTPRINT_MAX_TIME_S: f64 = 86_400.0;
const DEFAULT_FOOTPRINT_SEA_LEVEL_DENSITY_KG_M3: f64 = 1.225;
const DEFAULT_FOOTPRINT_DENSITY_SCALE_HEIGHT_M: f64 = 7_000.0;
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
    /// Launch-origin inertial position `[x, y, z]` (m). The
    /// constant-gravity method reports the flat local x/y offset from
    /// this point; numerical Earth-gravity methods derive an
    /// east/north tangent plane from it when no geodetic origin is
    /// declared.
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
        self.validate_common()?;
        if !self.gravity_m_s2.is_finite() || self.gravity_m_s2 <= MIN_FOOTPRINT_GRAVITY_M_S2 {
            return Err(PhysicsError::InvalidParameter {
                reason: "footprint gravity must be finite and strictly positive",
            });
        }
        Ok(())
    }

    fn validate_common(&self) -> Result<(), PhysicsError> {
        if !self.cull_altitude_m.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "footprint cull altitude must be finite",
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

/// Exponential atmosphere approximation used by the offline
/// drag/wind-aware footprint Monte Carlo propagator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FootprintDragModel {
    /// Reference density at the cull-altitude surface (kg/m³).
    pub surface_density_kg_m3: f64,
    /// Exponential density scale height (m).
    pub density_scale_height_m: f64,
}

impl Default for FootprintDragModel {
    fn default() -> Self {
        Self {
            surface_density_kg_m3: DEFAULT_FOOTPRINT_SEA_LEVEL_DENSITY_KG_M3,
            density_scale_height_m: DEFAULT_FOOTPRINT_DENSITY_SCALE_HEIGHT_M,
        }
    }
}

impl FootprintDragModel {
    /// Validate the drag-density parameters.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when either scalar is non-finite, the
    /// density is negative, or the scale height is non-positive.
    pub fn validate(&self) -> Result<(), PhysicsError> {
        if !self.surface_density_kg_m3.is_finite() || self.surface_density_kg_m3 < 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "footprint drag surface density must be finite and non-negative",
            });
        }
        if !self.density_scale_height_m.is_finite() || self.density_scale_height_m <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "footprint drag density scale height must be finite and positive",
            });
        }
        Ok(())
    }

    fn density_at_altitude_m(self, altitude_m: f64) -> Result<f64, PhysicsError> {
        if !altitude_m.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "footprint drag altitude is non-finite",
            });
        }
        let clamped_altitude_m = altitude_m.max(0.0);
        let exponent = -clamped_altitude_m / self.density_scale_height_m;
        let density = self.surface_density_kg_m3 * exponent.exp();
        if !density.is_finite() || density < 0.0 {
            return Err(PhysicsError::NonFinite {
                reason: "footprint drag density calculation is invalid",
            });
        }
        Ok(density)
    }
}

/// One sampled footprint input produced by the runner-side Monte
/// Carlo sampler.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FootprintSampleInput {
    /// Zero-based sample index.
    pub sample_index: u32,
    /// Sampled burnout state.
    pub state: BallisticState,
    /// Sampled constant wind vector in the propagation frame (m/s).
    pub wind_eci_m_s: [f64; 3],
}

impl FootprintSampleInput {
    fn validate(&self) -> Result<(), PhysicsError> {
        self.state.validate()?;
        require_finite_vec3(
            self.wind_eci_m_s,
            "footprint sample wind components must be finite",
        )
    }
}

/// Monte-Carlo footprint input consumed by the physics propagator.
#[derive(Clone, Debug, PartialEq)]
pub struct FootprintMonteCarloInput {
    /// Nominal burnout state used for the result's nominal footprint.
    pub nominal_state: BallisticState,
    /// Sampled burnout states and winds.
    pub samples: Vec<FootprintSampleInput>,
    /// Requested radial-distance confidence levels in `(0, 1)`.
    pub confidence_levels: Vec<f64>,
    /// Drag-density model for the sampled propagation.
    pub drag: FootprintDragModel,
    /// Fixed integration step in seconds.
    pub step_s: f64,
    /// Maximum propagation horizon in seconds.
    pub max_time_s: f64,
}

impl FootprintMonteCarloInput {
    /// Build input with default drag/wind propagation limits.
    #[must_use]
    pub fn new(
        nominal_state: BallisticState,
        samples: Vec<FootprintSampleInput>,
        confidence_levels: Vec<f64>,
    ) -> Self {
        Self {
            nominal_state,
            samples,
            confidence_levels,
            drag: FootprintDragModel::default(),
            step_s: DEFAULT_DRAG_WIND_FOOTPRINT_STEP_S,
            max_time_s: DEFAULT_DRAG_WIND_FOOTPRINT_MAX_TIME_S,
        }
    }

    /// Validate the full Monte-Carlo input.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when the nominal state, sample list,
    /// confidence levels, drag model, or integration limits are
    /// invalid.
    pub fn validate(&self) -> Result<(), PhysicsError> {
        self.nominal_state.validate()?;
        if self.samples.is_empty() {
            return Err(PhysicsError::InvalidParameter {
                reason: "footprint Monte Carlo requires at least one sample",
            });
        }
        for sample in &self.samples {
            sample.validate()?;
        }
        if self.confidence_levels.is_empty() {
            return Err(PhysicsError::InvalidParameter {
                reason: "footprint Monte Carlo requires at least one confidence level",
            });
        }
        for level in &self.confidence_levels {
            if !level.is_finite() || *level <= 0.0 || *level >= 1.0 {
                return Err(PhysicsError::InvalidParameter {
                    reason: "footprint Monte Carlo confidence levels must be in (0, 1)",
                });
            }
        }
        self.drag.validate()?;
        validate_numerical_footprint_limits(self.step_s, self.max_time_s)
    }
}

/// Successful propagated footprint sample.
#[derive(Clone, Debug, PartialEq)]
pub struct FootprintSample {
    /// Zero-based sample index.
    pub sample_index: u32,
    /// Sampled burnout state.
    pub state: BallisticState,
    /// Sampled constant wind vector in the propagation frame (m/s).
    pub wind_eci_m_s: [f64; 3],
    /// Landing footprint for this sample.
    pub landing: LandingFootprint,
}

/// Failed propagated footprint sample.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FootprintSampleFailure {
    /// Zero-based sample index.
    pub sample_index: u32,
    /// Failure reason surfaced by the propagator.
    pub reason: String,
}

/// Radial-distance quantile from the Monte-Carlo sample cloud.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FootprintQuantile {
    /// Confidence level in `(0, 1)`.
    pub confidence_level: f64,
    /// Radial distance in the downrange/crossrange plane (m). The
    /// reference point is determined by the result field that carries
    /// this quantile.
    pub radial_distance_m: f64,
}

/// Monte-Carlo footprint result: sample cloud, summary statistics,
/// covariance ellipse, and failed samples.
#[derive(Clone, Debug, PartialEq)]
pub struct FootprintMonteCarloResult {
    /// Drag/wind-aware nominal footprint from the nominal state.
    pub nominal: LandingFootprint,
    /// Successfully propagated samples.
    pub samples: Vec<FootprintSample>,
    /// Samples that failed propagation.
    pub failures: Vec<FootprintSampleFailure>,
    /// Mean downrange distance (m) across successful samples.
    pub mean_downrange_m: f64,
    /// Mean crossrange distance (m) across successful samples.
    pub mean_crossrange_m: f64,
    /// Empirical 50% circular probable radius about the sample mean
    /// in the downrange/crossrange plane (m).
    pub cep50_m: f64,
    /// Downrange component of the sample mean offset from the nominal
    /// footprint (m).
    pub mean_offset_downrange_from_nominal_m: f64,
    /// Crossrange component of the sample mean offset from the
    /// nominal footprint (m).
    pub mean_offset_crossrange_from_nominal_m: f64,
    /// Radial miss distance from the nominal footprint to the sample
    /// mean (m). This is output-only and accepts no target input.
    pub mean_miss_distance_from_nominal_m: f64,
    /// Downrange/downrange covariance element (m²).
    pub covariance_downrange_downrange_m2: f64,
    /// Downrange/crossrange covariance element (m²).
    pub covariance_downrange_crossrange_m2: f64,
    /// Crossrange/crossrange covariance element (m²).
    pub covariance_crossrange_crossrange_m2: f64,
    /// One- and three-sigma covariance ellipse from successful samples.
    pub dispersion_ellipse: FootprintDispersionEllipse,
    /// Requested radial-distance quantiles about the sample mean.
    pub quantiles: Vec<FootprintQuantile>,
    /// Requested radial-error quantiles about the nominal footprint.
    pub nominal_radial_error_quantiles: Vec<FootprintQuantile>,
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

/// Fixed-step numerical footprint propagation under a configured
/// gravity model.
///
/// This forward-only model integrates an unpowered ballistic state
/// with deterministic RK4 until the trajectory crosses the WGS84
/// radial ellipsoid surface plus `cull_altitude_m`. It is intended for
/// long coast / descent profiles where constant gravity is not a
/// credible approximation, including J2 and zonal-only EGM2008
/// gravity. It does not accept a desired landing location and produces
/// no control command.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NumericalGravityRangeSafetyFootprint<G> {
    gravity: G,
    step_s: f64,
    max_time_s: f64,
}

impl<G> NumericalGravityRangeSafetyFootprint<G> {
    /// Construct with the default numerical footprint limits.
    ///
    /// The default step is 1 s and the default horizon is 24 h.
    #[must_use]
    pub const fn new(gravity: G) -> Self {
        Self {
            gravity,
            step_s: DEFAULT_NUMERICAL_FOOTPRINT_STEP_S,
            max_time_s: DEFAULT_NUMERICAL_FOOTPRINT_MAX_TIME_S,
        }
    }

    /// Construct with explicit fixed step and maximum propagation
    /// horizon.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] when `step_s` is not
    /// finite and at least `1e-6`, or when `max_time_s` is not finite
    /// and positive.
    pub fn with_limits(gravity: G, step_s: f64, max_time_s: f64) -> Result<Self, PhysicsError> {
        validate_numerical_footprint_limits(step_s, max_time_s)?;
        Ok(Self {
            gravity,
            step_s,
            max_time_s,
        })
    }

    /// Fixed integration step in seconds.
    #[must_use]
    pub const fn step_s(&self) -> f64 {
        self.step_s
    }

    /// Maximum propagation horizon in seconds.
    #[must_use]
    pub const fn max_time_s(&self) -> f64 {
        self.max_time_s
    }
}

impl<G: GravityModel> RangeSafetyFootprint for NumericalGravityRangeSafetyFootprint<G> {
    fn landing_footprint(
        &self,
        state: &BallisticState,
        env: &FootprintEnvironment,
    ) -> Result<LandingFootprint, PhysicsError> {
        state.validate()?;
        env.validate_common()?;
        validate_numerical_footprint_limits(self.step_s, self.max_time_s)?;

        let mut current = NumericalFootprintState {
            position_eci_m: Vector3::new(
                state.position_eci_m[0],
                state.position_eci_m[1],
                state.position_eci_m[2],
            ),
            velocity_eci_m_s: Vector3::new(
                state.velocity_eci_m_s[0],
                state.velocity_eci_m_s[1],
                state.velocity_eci_m_s[2],
            ),
            ballistic_coefficient_m2_kg: state.ballistic_coefficient_m2_kg,
            time_s: state.time.as_seconds(),
        };
        let mut current_altitude_above_cull_m =
            altitude_above_numerical_cull_m(current.position_eci_m, env.cull_altitude_m)?;
        if current_altitude_above_cull_m <= 0.0 {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "ballistic state must start above the footprint cull altitude",
            });
        }

        let mut elapsed_s = 0.0_f64;
        while elapsed_s < self.max_time_s {
            let remaining_s = self.max_time_s - elapsed_s;
            let dt_s = self.step_s.min(remaining_s);
            let next = rk4_gravity_step(&self.gravity, current, dt_s)?;
            let next_altitude_above_cull_m =
                altitude_above_numerical_cull_m(next.position_eci_m, env.cull_altitude_m)?;
            if next_altitude_above_cull_m <= 0.0 {
                let denominator = current_altitude_above_cull_m - next_altitude_above_cull_m;
                if !denominator.is_finite() || denominator <= 0.0 {
                    return Err(PhysicsError::NonFinite {
                        reason: "numerical footprint cull interpolation is invalid",
                    });
                }
                let alpha = current_altitude_above_cull_m / denominator;
                let landing_position_eci_m =
                    current.position_eci_m + (next.position_eci_m - current.position_eci_m) * alpha;
                let time_to_cull_s = elapsed_s + dt_s * alpha;
                if !time_to_cull_s.is_finite() || time_to_cull_s < 0.0 {
                    return Err(PhysicsError::NonFinite {
                        reason: "numerical footprint crossing time is invalid",
                    });
                }
                return numerical_landing_footprint_from_position(
                    state,
                    env,
                    landing_position_eci_m,
                    time_to_cull_s,
                );
            }
            current = next;
            current_altitude_above_cull_m = next_altitude_above_cull_m;
            elapsed_s += dt_s;
        }

        Err(PhysicsError::OutOfEnvelope {
            reason: "numerical footprint did not reach cull altitude before max_time_s",
        })
    }
}

/// Run drag/wind-aware Monte-Carlo footprint propagation under the
/// flat constant-gravity footprint model.
///
/// This does not replace [`ConstantGravityRangeSafetyFootprint`]; it
/// is the sampled dispersion path used when the scenario declares
/// footprint uncertainty sources.
///
/// # Errors
///
/// Returns [`PhysicsError`] when the environment or Monte-Carlo input
/// is invalid, the nominal footprint cannot be propagated, or every
/// sample fails.
pub fn constant_gravity_footprint_monte_carlo(
    env: &FootprintEnvironment,
    input: &FootprintMonteCarloInput,
) -> Result<FootprintMonteCarloResult, PhysicsError> {
    env.validate()?;
    input.validate()?;
    let nominal = drag_wind_landing_footprint_constant(
        &input.nominal_state,
        env,
        input.drag,
        [0.0, 0.0, 0.0],
        input.step_s,
        input.max_time_s,
    )?;
    run_footprint_monte_carlo(input, nominal, |sample| {
        drag_wind_landing_footprint_constant(
            &sample.state,
            env,
            input.drag,
            sample.wind_eci_m_s,
            input.step_s,
            input.max_time_s,
        )
    })
}

/// Run drag/wind-aware Monte-Carlo footprint propagation under an
/// Earth-gravity footprint model.
///
/// The gravity model is still deterministic; stochasticity is limited
/// to the already-sampled burnout state, ballistic coefficient, and
/// wind vector supplied in [`FootprintMonteCarloInput`].
///
/// # Errors
///
/// Returns [`PhysicsError`] when the environment or Monte-Carlo input
/// is invalid, the nominal footprint cannot be propagated, or every
/// sample fails.
pub fn numerical_gravity_footprint_monte_carlo<G: GravityModel>(
    gravity: &G,
    env: &FootprintEnvironment,
    input: &FootprintMonteCarloInput,
) -> Result<FootprintMonteCarloResult, PhysicsError> {
    env.validate_common()?;
    input.validate()?;
    let nominal = drag_wind_landing_footprint_numerical(
        gravity,
        &input.nominal_state,
        env,
        input.drag,
        [0.0, 0.0, 0.0],
        input.step_s,
        input.max_time_s,
    )?;
    run_footprint_monte_carlo(input, nominal, |sample| {
        drag_wind_landing_footprint_numerical(
            gravity,
            &sample.state,
            env,
            input.drag,
            sample.wind_eci_m_s,
            input.step_s,
            input.max_time_s,
        )
    })
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

/// Per-stage mass and performance properties for ideal staging
/// analysis. Stages are ordered bottom-up: index 0 lights first.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StageMassProperties {
    /// Specific impulse (s).
    pub isp_s: f64,
    /// Structural coefficient `m_struct / (m_struct + m_prop)`.
    pub structural_coefficient: f64,
    /// Optional structural mass (kg), required for forward budget
    /// mode and computed for optimal mode.
    pub structural_mass_kg: Option<f64>,
    /// Optional propellant mass (kg), required for forward budget
    /// mode and computed for optimal mode.
    pub propellant_mass_kg: Option<f64>,
}

impl StageMassProperties {
    fn validate_common(&self) -> Result<(), PhysicsError> {
        if !self.isp_s.is_finite() || self.isp_s <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "staging stage isp_s must be finite and positive",
            });
        }
        if !self.structural_coefficient.is_finite()
            || !(0.0..1.0).contains(&self.structural_coefficient)
        {
            return Err(PhysicsError::InvalidParameter {
                reason: "staging structural_coefficient must lie in (0, 1)",
            });
        }
        Ok(())
    }

    fn require_masses(&self) -> Result<(f64, f64), PhysicsError> {
        let structural = self
            .structural_mass_kg
            .ok_or(PhysicsError::InvalidParameter {
                reason: "forward staging budget requires structural_mass_kg",
            })?;
        let propellant = self
            .propellant_mass_kg
            .ok_or(PhysicsError::InvalidParameter {
                reason: "forward staging budget requires propellant_mass_kg",
            })?;
        if !structural.is_finite() || structural <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "staging structural_mass_kg must be finite and positive",
            });
        }
        if !propellant.is_finite() || propellant <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "staging propellant_mass_kg must be finite and positive",
            });
        }
        let coeff = structural / (structural + propellant);
        if (coeff - self.structural_coefficient).abs() > 1.0e-9 {
            return Err(PhysicsError::InvalidParameter {
                reason: "staging masses do not match structural_coefficient",
            });
        }
        Ok((structural, propellant))
    }
}

/// Forward staging budget / mass-optimal split input. Every field is
/// vehicle-intrinsic: there is deliberately no launch site, target,
/// range, azimuth, impact point, or accuracy field.
#[derive(Clone, Debug, PartialEq)]
pub struct StagingBudgetInput {
    /// Stages ordered bottom-up.
    pub stages: Vec<StageMassProperties>,
    /// Payload mass above the upper stage (kg).
    pub payload_mass_kg: f64,
    /// Required ideal ΔV for optimal mode. `None` selects forward
    /// budget mode from declared masses.
    pub delta_v_budget_m_s: Option<f64>,
}

impl StagingBudgetInput {
    fn validate(&self) -> Result<(), PhysicsError> {
        if self.stages.is_empty() {
            return Err(PhysicsError::InvalidParameter {
                reason: "staging analysis requires at least one stage",
            });
        }
        require_positive_mass(
            self.payload_mass_kg,
            "staging payload mass must be finite and positive",
        )?;
        for stage in &self.stages {
            stage.validate_common()?;
        }
        if let Some(delta_v) = self.delta_v_budget_m_s {
            if !delta_v.is_finite() || delta_v <= 0.0 {
                return Err(PhysicsError::InvalidParameter {
                    reason: "staging delta_v_budget_m_s must be finite and positive",
                });
            }
        }
        Ok(())
    }
}

/// Staging analysis mode used in the output report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StagingBudgetMode {
    /// Forward rocket-equation budget from declared masses.
    Budget,
    /// Mass-optimal split for a declared ideal ΔV.
    Optimal,
}

/// Per-stage staging analysis output.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StagingStageReport {
    /// Bottom-up stage index.
    pub stage_index: usize,
    /// Specific impulse (s).
    pub isp_s: f64,
    /// Effective exhaust velocity `g0 * Isp` (m/s).
    pub effective_exhaust_velocity_m_s: f64,
    /// Structural coefficient.
    pub structural_coefficient: f64,
    /// Payload ratio `m_above / (m_struct + m_prop)`.
    pub payload_ratio: f64,
    /// Stage initial/final mass ratio.
    pub mass_ratio: f64,
    /// Ideal ΔV contribution (m/s).
    pub delta_v_m_s: f64,
    /// Structural mass (kg).
    pub structural_mass_kg: f64,
    /// Propellant mass (kg).
    pub propellant_mass_kg: f64,
    /// Mass above this stage (kg).
    pub mass_above_kg: f64,
}

/// Pure output report for ideal staging analysis.
#[derive(Clone, Debug, PartialEq)]
pub struct StagingBudgetReport {
    /// Analysis mode.
    pub mode: StagingBudgetMode,
    /// Per-stage output ordered bottom-up.
    pub stages: Vec<StagingStageReport>,
    /// Total ideal ΔV (m/s).
    pub total_delta_v_m_s: f64,
    /// Payload mass (kg).
    pub payload_mass_kg: f64,
    /// Gross liftoff mass (kg).
    pub gross_liftoff_mass_kg: f64,
    /// Payload fraction `payload / GLOW`.
    pub payload_fraction: f64,
    /// Whether gravity/drag/steering losses are excluded. Always true
    /// for this model.
    pub ideal_loss_free: bool,
}

/// Offline, output-only staging budget analysis.
pub trait StagingBudgetAnalysis {
    /// Analyze a forward budget or, when `delta_v_budget_m_s` is
    /// present, solve the mass-optimal split achieving it.
    ///
    /// # Errors
    ///
    /// Fails closed on invalid coefficients/masses, infeasible ΔV,
    /// or a missing multiplier bracket.
    fn analyze(&self, input: &StagingBudgetInput) -> Result<StagingBudgetReport, PhysicsError>;
}

/// Ideal rocket-equation staging analyzer with fixed-iteration
/// bisection for optimal mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IdealStagingBudgetAnalysis {
    bisection_iterations: usize,
}

impl IdealStagingBudgetAnalysis {
    /// Construct with a fixed bisection iteration count.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when `bisection_iterations` is zero.
    pub fn new(bisection_iterations: usize) -> Result<Self, PhysicsError> {
        if bisection_iterations == 0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "staging bisection_iterations must be positive",
            });
        }
        Ok(Self {
            bisection_iterations,
        })
    }

    /// Fixed iteration count.
    #[must_use]
    pub const fn bisection_iterations(&self) -> usize {
        self.bisection_iterations
    }
}

impl Default for IdealStagingBudgetAnalysis {
    fn default() -> Self {
        Self {
            bisection_iterations: 96,
        }
    }
}

impl StagingBudgetAnalysis for IdealStagingBudgetAnalysis {
    fn analyze(&self, input: &StagingBudgetInput) -> Result<StagingBudgetReport, PhysicsError> {
        input.validate()?;
        match input.delta_v_budget_m_s {
            Some(delta_v) => self.analyze_optimal(input, delta_v),
            None => analyze_forward_staging(input),
        }
    }
}

impl IdealStagingBudgetAnalysis {
    fn analyze_optimal(
        &self,
        input: &StagingBudgetInput,
        required_delta_v_m_s: f64,
    ) -> Result<StagingBudgetReport, PhysicsError> {
        let max_delta_v: f64 = input
            .stages
            .iter()
            .map(|stage| {
                let c = crate::gravity::STANDARD_GRAVITY_M_S2 * stage.isp_s;
                c * (1.0 / stage.structural_coefficient).ln()
            })
            .sum();
        if required_delta_v_m_s >= max_delta_v {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "staging requested delta-v is infeasible for structural coefficients",
            });
        }
        let min_mu = input
            .stages
            .iter()
            .map(|stage| 1.0 / (crate::gravity::STANDARD_GRAVITY_M_S2 * stage.isp_s))
            .fold(0.0_f64, f64::max);
        let mut lo = min_mu * (1.0 + 1.0e-12);
        if lo <= min_mu {
            lo = min_mu + 1.0e-15;
        }
        let hi = 1.0_f64.max(2.0 * lo);
        if staging_constraint(input, hi)? < required_delta_v_m_s {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "staging multiplier root was not bracketed",
            });
        }
        let mut low = lo;
        let mut high = hi;
        for _ in 0..self.bisection_iterations {
            let mid = 0.5 * (low + high);
            if staging_constraint(input, mid)? < required_delta_v_m_s {
                low = mid;
            } else {
                high = mid;
            }
        }
        let mu = 0.5 * (low + high);
        let mut stage_reports = Vec::with_capacity(input.stages.len());
        let mut above_mass_kg = input.payload_mass_kg;
        let mut computed = Vec::with_capacity(input.stages.len());
        for (stage_index, stage) in input.stages.iter().enumerate().rev() {
            let c = crate::gravity::STANDARD_GRAVITY_M_S2 * stage.isp_s;
            let mass_ratio = optimal_mass_ratio(c, stage.structural_coefficient, mu)?;
            let payload_ratio =
                (1.0 - mass_ratio * stage.structural_coefficient) / (mass_ratio - 1.0);
            if !payload_ratio.is_finite() || payload_ratio <= 0.0 {
                return Err(PhysicsError::OutOfEnvelope {
                    reason: "staging optimal payload ratio is infeasible",
                });
            }
            let stage_total = above_mass_kg / payload_ratio;
            let structural_mass = stage.structural_coefficient * stage_total;
            let propellant_mass = (1.0 - stage.structural_coefficient) * stage_total;
            let delta_v = c * mass_ratio.ln();
            computed.push(StagingStageReport {
                stage_index,
                isp_s: stage.isp_s,
                effective_exhaust_velocity_m_s: c,
                structural_coefficient: stage.structural_coefficient,
                payload_ratio,
                mass_ratio,
                delta_v_m_s: delta_v,
                structural_mass_kg: structural_mass,
                propellant_mass_kg: propellant_mass,
                mass_above_kg: above_mass_kg,
            });
            above_mass_kg += stage_total;
        }
        computed.reverse();
        stage_reports.extend(computed);
        finish_staging_report(
            StagingBudgetMode::Optimal,
            stage_reports,
            input.payload_mass_kg,
        )
    }
}

fn analyze_forward_staging(
    input: &StagingBudgetInput,
) -> Result<StagingBudgetReport, PhysicsError> {
    let mut mass_above_by_stage = vec![input.payload_mass_kg; input.stages.len()];
    let mut running_above = input.payload_mass_kg;
    for (index, stage) in input.stages.iter().enumerate().rev() {
        mass_above_by_stage[index] = running_above;
        let (structural, propellant) = stage.require_masses()?;
        running_above += structural + propellant;
    }
    let mut reports = Vec::with_capacity(input.stages.len());
    for (index, stage) in input.stages.iter().enumerate() {
        let (structural, propellant) = stage.require_masses()?;
        let stage_total = structural + propellant;
        let payload_ratio = mass_above_by_stage[index] / stage_total;
        let mass_ratio = (1.0 + payload_ratio) / (stage.structural_coefficient + payload_ratio);
        if !mass_ratio.is_finite() || mass_ratio <= 1.0 {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "staging forward mass ratio must be greater than 1",
            });
        }
        let c = crate::gravity::STANDARD_GRAVITY_M_S2 * stage.isp_s;
        reports.push(StagingStageReport {
            stage_index: index,
            isp_s: stage.isp_s,
            effective_exhaust_velocity_m_s: c,
            structural_coefficient: stage.structural_coefficient,
            payload_ratio,
            mass_ratio,
            delta_v_m_s: c * mass_ratio.ln(),
            structural_mass_kg: structural,
            propellant_mass_kg: propellant,
            mass_above_kg: mass_above_by_stage[index],
        });
    }
    finish_staging_report(StagingBudgetMode::Budget, reports, input.payload_mass_kg)
}

fn finish_staging_report(
    mode: StagingBudgetMode,
    stages: Vec<StagingStageReport>,
    payload_mass_kg: f64,
) -> Result<StagingBudgetReport, PhysicsError> {
    let total_delta_v_m_s = stages.iter().map(|stage| stage.delta_v_m_s).sum();
    let stage_mass_sum: f64 = stages
        .iter()
        .map(|stage| stage.structural_mass_kg + stage.propellant_mass_kg)
        .sum();
    let gross_liftoff_mass_kg = payload_mass_kg + stage_mass_sum;
    let payload_fraction = payload_mass_kg / gross_liftoff_mass_kg;
    for value in [total_delta_v_m_s, gross_liftoff_mass_kg, payload_fraction] {
        if !value.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "staging report contains a non-finite value",
            });
        }
    }
    Ok(StagingBudgetReport {
        mode,
        stages,
        total_delta_v_m_s,
        payload_mass_kg,
        gross_liftoff_mass_kg,
        payload_fraction,
        ideal_loss_free: true,
    })
}

fn staging_constraint(input: &StagingBudgetInput, mu: f64) -> Result<f64, PhysicsError> {
    let mut sum = 0.0;
    for stage in &input.stages {
        let c = crate::gravity::STANDARD_GRAVITY_M_S2 * stage.isp_s;
        let n = optimal_mass_ratio(c, stage.structural_coefficient, mu)?;
        sum += c * n.ln();
    }
    if !sum.is_finite() {
        return Err(PhysicsError::NonFinite {
            reason: "staging multiplier constraint is non-finite",
        });
    }
    Ok(sum)
}

fn optimal_mass_ratio(c: f64, epsilon: f64, mu: f64) -> Result<f64, PhysicsError> {
    let n = (c * mu - 1.0) / (c * epsilon * mu);
    if !n.is_finite() || n <= 1.0 {
        return Err(PhysicsError::OutOfEnvelope {
            reason: "staging optimal mass ratio is outside feasible envelope",
        });
    }
    Ok(n)
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

#[derive(Clone, Copy, Debug, PartialEq)]
struct NumericalFootprintState {
    position_eci_m: Vector3<f64>,
    velocity_eci_m_s: Vector3<f64>,
    ballistic_coefficient_m2_kg: f64,
    time_s: f64,
}

fn validate_numerical_footprint_limits(step_s: f64, max_time_s: f64) -> Result<(), PhysicsError> {
    if !step_s.is_finite() || step_s < MIN_NUMERICAL_FOOTPRINT_STEP_S {
        return Err(PhysicsError::InvalidParameter {
            reason: "numerical footprint step must be finite and at least 1e-6 s",
        });
    }
    if !max_time_s.is_finite() || max_time_s <= 0.0 {
        return Err(PhysicsError::InvalidParameter {
            reason: "numerical footprint max_time_s must be finite and positive",
        });
    }
    Ok(())
}

fn rk4_gravity_step<G: GravityModel>(
    gravity: &G,
    state: NumericalFootprintState,
    dt_s: f64,
) -> Result<NumericalFootprintState, PhysicsError> {
    let (k1_r, k1_v) = numerical_footprint_derivative(gravity, state)?;
    let s2 = NumericalFootprintState {
        position_eci_m: state.position_eci_m + k1_r * (0.5 * dt_s),
        velocity_eci_m_s: state.velocity_eci_m_s + k1_v * (0.5 * dt_s),
        ballistic_coefficient_m2_kg: state.ballistic_coefficient_m2_kg,
        time_s: state.time_s + 0.5 * dt_s,
    };
    let (k2_r, k2_v) = numerical_footprint_derivative(gravity, s2)?;
    let s3 = NumericalFootprintState {
        position_eci_m: state.position_eci_m + k2_r * (0.5 * dt_s),
        velocity_eci_m_s: state.velocity_eci_m_s + k2_v * (0.5 * dt_s),
        ballistic_coefficient_m2_kg: state.ballistic_coefficient_m2_kg,
        time_s: state.time_s + 0.5 * dt_s,
    };
    let (k3_r, k3_v) = numerical_footprint_derivative(gravity, s3)?;
    let s4 = NumericalFootprintState {
        position_eci_m: state.position_eci_m + k3_r * dt_s,
        velocity_eci_m_s: state.velocity_eci_m_s + k3_v * dt_s,
        ballistic_coefficient_m2_kg: state.ballistic_coefficient_m2_kg,
        time_s: state.time_s + dt_s,
    };
    let (k4_r, k4_v) = numerical_footprint_derivative(gravity, s4)?;
    let one_sixth_dt = dt_s / 6.0;
    let position_eci_m =
        state.position_eci_m + (k1_r + 2.0 * k2_r + 2.0 * k3_r + k4_r) * one_sixth_dt;
    let velocity_eci_m_s =
        state.velocity_eci_m_s + (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v) * one_sixth_dt;
    if !position_eci_m.iter().all(|v| v.is_finite())
        || !velocity_eci_m_s.iter().all(|v| v.is_finite())
    {
        return Err(PhysicsError::NonFinite {
            reason: "numerical footprint RK4 step produced non-finite state",
        });
    }
    Ok(NumericalFootprintState {
        position_eci_m,
        velocity_eci_m_s,
        ballistic_coefficient_m2_kg: state.ballistic_coefficient_m2_kg,
        time_s: state.time_s + dt_s,
    })
}

fn numerical_footprint_derivative<G: GravityModel>(
    gravity: &G,
    state: NumericalFootprintState,
) -> Result<(Vector3<f64>, Vector3<f64>), PhysicsError> {
    if !state.time_s.is_finite() {
        return Err(PhysicsError::InvalidParameter {
            reason: "numerical footprint state time must be finite",
        });
    }
    let acceleration_eci_m_s2 = gravity.gravity_eci_m_s2(
        Position3::<Eci>::from_vector(state.position_eci_m),
        SimTime::from_seconds(state.time_s),
    )?;
    Ok((state.velocity_eci_m_s, acceleration_eci_m_s2))
}

fn run_footprint_monte_carlo<F>(
    input: &FootprintMonteCarloInput,
    nominal: LandingFootprint,
    mut propagate: F,
) -> Result<FootprintMonteCarloResult, PhysicsError>
where
    F: FnMut(&FootprintSampleInput) -> Result<LandingFootprint, PhysicsError>,
{
    let mut samples = Vec::with_capacity(input.samples.len());
    let mut failures = Vec::new();
    for sample in &input.samples {
        match propagate(sample) {
            Ok(landing) => samples.push(FootprintSample {
                sample_index: sample.sample_index,
                state: sample.state,
                wind_eci_m_s: sample.wind_eci_m_s,
                landing,
            }),
            Err(err) => failures.push(FootprintSampleFailure {
                sample_index: sample.sample_index,
                reason: err.to_string(),
            }),
        }
    }
    if samples.is_empty() {
        return Err(PhysicsError::OutOfEnvelope {
            reason: "footprint Monte Carlo produced no successful samples",
        });
    }
    let stats = footprint_sample_statistics(&nominal, &samples, &input.confidence_levels)?;
    Ok(FootprintMonteCarloResult {
        nominal,
        samples,
        failures,
        mean_downrange_m: stats.mean_downrange_m,
        mean_crossrange_m: stats.mean_crossrange_m,
        cep50_m: stats.cep50_m,
        mean_offset_downrange_from_nominal_m: stats.mean_offset_downrange_from_nominal_m,
        mean_offset_crossrange_from_nominal_m: stats.mean_offset_crossrange_from_nominal_m,
        mean_miss_distance_from_nominal_m: stats.mean_miss_distance_from_nominal_m,
        covariance_downrange_downrange_m2: stats.covariance_downrange_downrange_m2,
        covariance_downrange_crossrange_m2: stats.covariance_downrange_crossrange_m2,
        covariance_crossrange_crossrange_m2: stats.covariance_crossrange_crossrange_m2,
        dispersion_ellipse: stats.dispersion_ellipse,
        quantiles: stats.quantiles,
        nominal_radial_error_quantiles: stats.nominal_radial_error_quantiles,
    })
}

struct FootprintSampleStatistics {
    mean_downrange_m: f64,
    mean_crossrange_m: f64,
    cep50_m: f64,
    mean_offset_downrange_from_nominal_m: f64,
    mean_offset_crossrange_from_nominal_m: f64,
    mean_miss_distance_from_nominal_m: f64,
    covariance_downrange_downrange_m2: f64,
    covariance_downrange_crossrange_m2: f64,
    covariance_crossrange_crossrange_m2: f64,
    dispersion_ellipse: FootprintDispersionEllipse,
    quantiles: Vec<FootprintQuantile>,
    nominal_radial_error_quantiles: Vec<FootprintQuantile>,
}

fn footprint_sample_statistics(
    nominal: &LandingFootprint,
    samples: &[FootprintSample],
    confidence_levels: &[f64],
) -> Result<FootprintSampleStatistics, PhysicsError> {
    let n = samples.len();
    let inv_n = 1.0 / n as f64;
    let mut sum_downrange_m = 0.0;
    let mut sum_crossrange_m = 0.0;
    for sample in samples {
        sum_downrange_m += sample.landing.downrange_m;
        sum_crossrange_m += sample.landing.crossrange_m;
    }
    let mean_downrange_m = sum_downrange_m * inv_n;
    let mean_crossrange_m = sum_crossrange_m * inv_n;
    let mean_offset_downrange_from_nominal_m = mean_downrange_m - nominal.downrange_m;
    let mean_offset_crossrange_from_nominal_m = mean_crossrange_m - nominal.crossrange_m;
    let mean_miss_distance_from_nominal_m = radial_norm_m(
        mean_offset_downrange_from_nominal_m,
        mean_offset_crossrange_from_nominal_m,
    );
    if !mean_miss_distance_from_nominal_m.is_finite() {
        return Err(PhysicsError::NonFinite {
            reason: "footprint Monte Carlo nominal miss distance is non-finite",
        });
    }
    let mut c_dd = 0.0;
    let mut c_dc = 0.0;
    let mut c_cc = 0.0;
    let mut mean_distances = Vec::with_capacity(n);
    let mut nominal_distances = Vec::with_capacity(n);
    for sample in samples {
        let d_downrange_m = sample.landing.downrange_m - mean_downrange_m;
        let d_crossrange_m = sample.landing.crossrange_m - mean_crossrange_m;
        c_dd += d_downrange_m * d_downrange_m;
        c_dc += d_downrange_m * d_crossrange_m;
        c_cc += d_crossrange_m * d_crossrange_m;
        mean_distances.push(radial_norm_m(d_downrange_m, d_crossrange_m));
        nominal_distances.push(radial_norm_m(
            sample.landing.downrange_m - nominal.downrange_m,
            sample.landing.crossrange_m - nominal.crossrange_m,
        ));
    }
    let denom = if n > 1 { (n - 1) as f64 } else { 1.0 };
    c_dd /= denom;
    c_dc /= denom;
    c_cc /= denom;
    if !c_dd.is_finite() || !c_dc.is_finite() || !c_cc.is_finite() {
        return Err(PhysicsError::NonFinite {
            reason: "footprint Monte Carlo covariance is non-finite",
        });
    }
    mean_distances.sort_by(f64::total_cmp);
    nominal_distances.sort_by(f64::total_cmp);
    let cep50_m = radial_quantile_m(&mean_distances, 0.5);
    let quantiles = radial_quantiles(&mean_distances, confidence_levels);
    let nominal_radial_error_quantiles = radial_quantiles(&nominal_distances, confidence_levels);
    Ok(FootprintSampleStatistics {
        mean_downrange_m,
        mean_crossrange_m,
        cep50_m,
        mean_offset_downrange_from_nominal_m,
        mean_offset_crossrange_from_nominal_m,
        mean_miss_distance_from_nominal_m,
        covariance_downrange_downrange_m2: c_dd,
        covariance_downrange_crossrange_m2: c_dc,
        covariance_crossrange_crossrange_m2: c_cc,
        dispersion_ellipse: covariance_dispersion_ellipse(c_dd, c_dc, c_cc)?,
        quantiles,
        nominal_radial_error_quantiles,
    })
}

fn radial_norm_m(downrange_m: f64, crossrange_m: f64) -> f64 {
    (downrange_m * downrange_m + crossrange_m * crossrange_m).sqrt()
}

fn radial_quantiles(
    sorted_distances_m: &[f64],
    confidence_levels: &[f64],
) -> Vec<FootprintQuantile> {
    confidence_levels
        .iter()
        .copied()
        .map(|confidence_level| FootprintQuantile {
            confidence_level,
            radial_distance_m: radial_quantile_m(sorted_distances_m, confidence_level),
        })
        .collect()
}

fn radial_quantile_m(sorted_distances_m: &[f64], confidence_level: f64) -> f64 {
    let rank = (confidence_level * sorted_distances_m.len() as f64).ceil();
    let index = ((rank as usize).saturating_sub(1)).min(sorted_distances_m.len() - 1);
    sorted_distances_m[index]
}

fn covariance_dispersion_ellipse(
    covariance_downrange_downrange_m2: f64,
    covariance_downrange_crossrange_m2: f64,
    covariance_crossrange_crossrange_m2: f64,
) -> Result<FootprintDispersionEllipse, PhysicsError> {
    let trace = covariance_downrange_downrange_m2 + covariance_crossrange_crossrange_m2;
    let diff = covariance_downrange_downrange_m2 - covariance_crossrange_crossrange_m2;
    let root = (diff * diff + 4.0 * covariance_downrange_crossrange_m2.powi(2)).sqrt();
    let lambda_major = 0.5 * (trace + root);
    let lambda_minor = 0.5 * (trace - root);
    if !lambda_major.is_finite() || !lambda_minor.is_finite() {
        return Err(PhysicsError::NonFinite {
            reason: "footprint Monte Carlo covariance eigenvalues are non-finite",
        });
    }
    let one_sigma_semi_major_m = lambda_major.max(0.0).sqrt();
    let one_sigma_semi_minor_m = lambda_minor.max(0.0).sqrt();
    let orientation_rad = 0.5 * (2.0 * covariance_downrange_crossrange_m2).atan2(diff);
    Ok(FootprintDispersionEllipse {
        one_sigma_semi_major_m,
        one_sigma_semi_minor_m,
        three_sigma_semi_major_m: 3.0 * one_sigma_semi_major_m,
        three_sigma_semi_minor_m: 3.0 * one_sigma_semi_minor_m,
        orientation_rad,
    })
}

fn drag_wind_landing_footprint_constant(
    state: &BallisticState,
    env: &FootprintEnvironment,
    drag: FootprintDragModel,
    wind_eci_m_s: [f64; 3],
    step_s: f64,
    max_time_s: f64,
) -> Result<LandingFootprint, PhysicsError> {
    state.validate()?;
    env.validate()?;
    drag.validate()?;
    validate_numerical_footprint_limits(step_s, max_time_s)?;
    require_finite_vec3(
        wind_eci_m_s,
        "footprint drag/wind constant-gravity wind components must be finite",
    )?;

    let mut current = NumericalFootprintState {
        position_eci_m: Vector3::new(
            state.position_eci_m[0],
            state.position_eci_m[1],
            state.position_eci_m[2],
        ),
        velocity_eci_m_s: Vector3::new(
            state.velocity_eci_m_s[0],
            state.velocity_eci_m_s[1],
            state.velocity_eci_m_s[2],
        ),
        ballistic_coefficient_m2_kg: state.ballistic_coefficient_m2_kg,
        time_s: state.time.as_seconds(),
    };
    let mut current_altitude_above_cull_m = current.position_eci_m.z - env.cull_altitude_m;
    if current_altitude_above_cull_m <= 0.0 {
        return Err(PhysicsError::OutOfEnvelope {
            reason: "ballistic state must start above the footprint cull altitude",
        });
    }
    let wind = Vector3::new(wind_eci_m_s[0], wind_eci_m_s[1], wind_eci_m_s[2]);
    let mut elapsed_s = 0.0;
    while elapsed_s < max_time_s {
        let dt_s = step_s.min(max_time_s - elapsed_s);
        let next = rk4_drag_wind_step(current, dt_s, |s| {
            let altitude_m = s.position_eci_m.z - env.cull_altitude_m;
            let gravity = Vector3::new(0.0, 0.0, -env.gravity_m_s2);
            drag_wind_derivative(s, gravity, wind, drag, altitude_m)
        })?;
        let next_altitude_above_cull_m = next.position_eci_m.z - env.cull_altitude_m;
        if next_altitude_above_cull_m <= 0.0 {
            let denominator = current_altitude_above_cull_m - next_altitude_above_cull_m;
            if !denominator.is_finite() || denominator <= 0.0 {
                return Err(PhysicsError::NonFinite {
                    reason: "drag/wind footprint cull interpolation is invalid",
                });
            }
            let alpha = current_altitude_above_cull_m / denominator;
            let landing_position_eci_m =
                current.position_eci_m + (next.position_eci_m - current.position_eci_m) * alpha;
            let time_to_cull_s = elapsed_s + dt_s * alpha;
            let downrange_m = landing_position_eci_m.x - env.launch_origin_eci_m[0];
            let crossrange_m = landing_position_eci_m.y - env.launch_origin_eci_m[1];
            let bearing_rad =
                if downrange_m.abs() <= f64::EPSILON && crossrange_m.abs() <= f64::EPSILON {
                    0.0
                } else {
                    crossrange_m.atan2(downrange_m)
                };
            let (latitude_deg, longitude_deg) = match env.geodetic_origin {
                Some(origin) => geodetic_from_range_relative(origin, downrange_m, crossrange_m)?,
                None => (None, None),
            };
            return Ok(LandingFootprint {
                downrange_m,
                crossrange_m,
                bearing_rad,
                time_to_cull_s,
                latitude_deg,
                longitude_deg,
                dispersion_ellipse: env.dispersion.map(dispersion_ellipse),
            });
        }
        current = next;
        current_altitude_above_cull_m = next_altitude_above_cull_m;
        elapsed_s += dt_s;
    }
    Err(PhysicsError::OutOfEnvelope {
        reason: "drag/wind footprint did not reach cull altitude before max_time_s",
    })
}

fn drag_wind_landing_footprint_numerical<G: GravityModel>(
    gravity: &G,
    state: &BallisticState,
    env: &FootprintEnvironment,
    drag: FootprintDragModel,
    wind_eci_m_s: [f64; 3],
    step_s: f64,
    max_time_s: f64,
) -> Result<LandingFootprint, PhysicsError> {
    state.validate()?;
    env.validate_common()?;
    drag.validate()?;
    validate_numerical_footprint_limits(step_s, max_time_s)?;
    require_finite_vec3(
        wind_eci_m_s,
        "footprint drag/wind numerical wind components must be finite",
    )?;

    let mut current = NumericalFootprintState {
        position_eci_m: Vector3::new(
            state.position_eci_m[0],
            state.position_eci_m[1],
            state.position_eci_m[2],
        ),
        velocity_eci_m_s: Vector3::new(
            state.velocity_eci_m_s[0],
            state.velocity_eci_m_s[1],
            state.velocity_eci_m_s[2],
        ),
        ballistic_coefficient_m2_kg: state.ballistic_coefficient_m2_kg,
        time_s: state.time.as_seconds(),
    };
    let mut current_altitude_above_cull_m =
        altitude_above_numerical_cull_m(current.position_eci_m, env.cull_altitude_m)?;
    if current_altitude_above_cull_m <= 0.0 {
        return Err(PhysicsError::OutOfEnvelope {
            reason: "ballistic state must start above the footprint cull altitude",
        });
    }
    let wind = Vector3::new(wind_eci_m_s[0], wind_eci_m_s[1], wind_eci_m_s[2]);
    let mut elapsed_s = 0.0;
    while elapsed_s < max_time_s {
        let dt_s = step_s.min(max_time_s - elapsed_s);
        let next = rk4_drag_wind_step(current, dt_s, |s| {
            let gravity_acceleration = gravity.gravity_eci_m_s2(
                Position3::<Eci>::from_vector(s.position_eci_m),
                SimTime::from_seconds(s.time_s),
            )?;
            let altitude_m = altitude_above_numerical_cull_m(s.position_eci_m, 0.0)?;
            drag_wind_derivative(s, gravity_acceleration, wind, drag, altitude_m)
        })?;
        let next_altitude_above_cull_m =
            altitude_above_numerical_cull_m(next.position_eci_m, env.cull_altitude_m)?;
        if next_altitude_above_cull_m <= 0.0 {
            let denominator = current_altitude_above_cull_m - next_altitude_above_cull_m;
            if !denominator.is_finite() || denominator <= 0.0 {
                return Err(PhysicsError::NonFinite {
                    reason: "drag/wind numerical footprint cull interpolation is invalid",
                });
            }
            let alpha = current_altitude_above_cull_m / denominator;
            let landing_position_eci_m =
                current.position_eci_m + (next.position_eci_m - current.position_eci_m) * alpha;
            let time_to_cull_s = elapsed_s + dt_s * alpha;
            return numerical_landing_footprint_from_position(
                state,
                env,
                landing_position_eci_m,
                time_to_cull_s,
            );
        }
        current = next;
        current_altitude_above_cull_m = next_altitude_above_cull_m;
        elapsed_s += dt_s;
    }
    Err(PhysicsError::OutOfEnvelope {
        reason: "drag/wind numerical footprint did not reach cull altitude before max_time_s",
    })
}

fn rk4_drag_wind_step<F>(
    state: NumericalFootprintState,
    dt_s: f64,
    mut derivative: F,
) -> Result<NumericalFootprintState, PhysicsError>
where
    F: FnMut(NumericalFootprintState) -> Result<(Vector3<f64>, Vector3<f64>), PhysicsError>,
{
    let (k1_r, k1_v) = derivative(state)?;
    let s2 = NumericalFootprintState {
        position_eci_m: state.position_eci_m + k1_r * (0.5 * dt_s),
        velocity_eci_m_s: state.velocity_eci_m_s + k1_v * (0.5 * dt_s),
        ballistic_coefficient_m2_kg: state.ballistic_coefficient_m2_kg,
        time_s: state.time_s + 0.5 * dt_s,
    };
    let (k2_r, k2_v) = derivative(s2)?;
    let s3 = NumericalFootprintState {
        position_eci_m: state.position_eci_m + k2_r * (0.5 * dt_s),
        velocity_eci_m_s: state.velocity_eci_m_s + k2_v * (0.5 * dt_s),
        ballistic_coefficient_m2_kg: state.ballistic_coefficient_m2_kg,
        time_s: state.time_s + 0.5 * dt_s,
    };
    let (k3_r, k3_v) = derivative(s3)?;
    let s4 = NumericalFootprintState {
        position_eci_m: state.position_eci_m + k3_r * dt_s,
        velocity_eci_m_s: state.velocity_eci_m_s + k3_v * dt_s,
        ballistic_coefficient_m2_kg: state.ballistic_coefficient_m2_kg,
        time_s: state.time_s + dt_s,
    };
    let (k4_r, k4_v) = derivative(s4)?;
    let one_sixth_dt = dt_s / 6.0;
    let position_eci_m =
        state.position_eci_m + (k1_r + 2.0 * k2_r + 2.0 * k3_r + k4_r) * one_sixth_dt;
    let velocity_eci_m_s =
        state.velocity_eci_m_s + (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v) * one_sixth_dt;
    if !position_eci_m.iter().all(|v| v.is_finite())
        || !velocity_eci_m_s.iter().all(|v| v.is_finite())
    {
        return Err(PhysicsError::NonFinite {
            reason: "drag/wind footprint RK4 step produced non-finite state",
        });
    }
    Ok(NumericalFootprintState {
        position_eci_m,
        velocity_eci_m_s,
        ballistic_coefficient_m2_kg: state.ballistic_coefficient_m2_kg,
        time_s: state.time_s + dt_s,
    })
}

fn drag_wind_derivative(
    state: NumericalFootprintState,
    gravity_acceleration_eci_m_s2: Vector3<f64>,
    wind_eci_m_s: Vector3<f64>,
    drag: FootprintDragModel,
    altitude_m: f64,
) -> Result<(Vector3<f64>, Vector3<f64>), PhysicsError> {
    if !state.time_s.is_finite() {
        return Err(PhysicsError::InvalidParameter {
            reason: "drag/wind footprint state time must be finite",
        });
    }
    let density = drag.density_at_altitude_m(altitude_m)?;
    let relative_velocity_m_s = state.velocity_eci_m_s - wind_eci_m_s;
    let speed_m_s = relative_velocity_m_s.norm();
    let drag_acceleration_m_s2 = if speed_m_s <= f64::EPSILON
        || density <= f64::EPSILON
        || state.ballistic_coefficient_m2_kg <= f64::EPSILON
    {
        Vector3::zeros()
    } else {
        let scale = -0.5 * density * state.ballistic_coefficient_m2_kg * speed_m_s;
        relative_velocity_m_s * scale
    };
    let acceleration = gravity_acceleration_eci_m_s2 + drag_acceleration_m_s2;
    if !acceleration.iter().all(|v| v.is_finite()) {
        return Err(PhysicsError::NonFinite {
            reason: "drag/wind footprint acceleration is non-finite",
        });
    }
    Ok((state.velocity_eci_m_s, acceleration))
}

fn numerical_landing_footprint_from_position(
    state: &BallisticState,
    env: &FootprintEnvironment,
    landing_position_eci_m: Vector3<f64>,
    time_to_cull_s: f64,
) -> Result<LandingFootprint, PhysicsError> {
    let landing_time_s = state.time.as_seconds() + time_to_cull_s;
    let (downrange_m, crossrange_m, latitude_deg, longitude_deg) =
        if let Some(origin) = env.geodetic_origin {
            numerical_landing_from_geodetic_origin(origin, landing_position_eci_m, landing_time_s)?
        } else {
            let (downrange_m, crossrange_m) =
                numerical_range_relative_from_inertial_origin(env, landing_position_eci_m)?;
            (downrange_m, crossrange_m, None, None)
        };
    let bearing_rad = if downrange_m == 0.0 && crossrange_m == 0.0 {
        0.0
    } else {
        crossrange_m.atan2(downrange_m)
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

fn numerical_landing_from_geodetic_origin(
    origin: FootprintGeodeticOrigin,
    landing_position_eci_m: Vector3<f64>,
    landing_time_s: f64,
) -> Result<(f64, f64, Option<f64>, Option<f64>), PhysicsError> {
    if !landing_time_s.is_finite() {
        return Err(PhysicsError::NonFinite {
            reason: "numerical footprint landing time is non-finite",
        });
    }
    let local_origin = LocalGeodeticOrigin::new_degrees(
        origin.latitude_deg,
        origin.longitude_deg,
        origin.height_m,
    )?;
    let frame = FrameContext::wgs84_uniform_rotation(Some(local_origin));
    let landing_ecef = frame.eci_to_ecef_position(
        SimTime::from_seconds(landing_time_s),
        Position3::<Eci>::from_vector(landing_position_eci_m),
    );
    let landing_ned = frame.ecef_to_ned_position(landing_ecef)?;
    let crossrange_m = landing_ned.vector.x;
    let downrange_m = landing_ned.vector.y;
    let (latitude_deg, longitude_deg) = geodetic_degrees_from_wgs84_ecef(landing_ecef.vector)?;
    if !downrange_m.is_finite() || !crossrange_m.is_finite() {
        return Err(PhysicsError::NonFinite {
            reason: "numerical footprint range-relative projection is non-finite",
        });
    }
    Ok((downrange_m, crossrange_m, latitude_deg, longitude_deg))
}

fn numerical_range_relative_from_inertial_origin(
    env: &FootprintEnvironment,
    landing_position_eci_m: Vector3<f64>,
) -> Result<(f64, f64), PhysicsError> {
    let origin = Vector3::new(
        env.launch_origin_eci_m[0],
        env.launch_origin_eci_m[1],
        env.launch_origin_eci_m[2],
    );
    let origin_norm = origin.norm();
    let delta = landing_position_eci_m - origin;
    if origin_norm <= MIN_DIRECTION_NORM {
        return Ok((delta.x, delta.y));
    }
    let up = origin / origin_norm;
    let spin_axis = Vector3::new(0.0, 0.0, 1.0);
    let mut east = spin_axis.cross(&up);
    let east_norm = east.norm();
    if east_norm <= MIN_DIRECTION_NORM {
        east = Vector3::new(1.0, 0.0, 0.0).cross(&up);
    }
    let east_norm = east.norm();
    if east_norm <= MIN_DIRECTION_NORM {
        return Err(PhysicsError::OutOfEnvelope {
            reason: "numerical footprint could not construct local tangent basis",
        });
    }
    let east = east / east_norm;
    let north = up.cross(&east);
    let downrange_m = delta.dot(&east);
    let crossrange_m = delta.dot(&north);
    if !downrange_m.is_finite() || !crossrange_m.is_finite() {
        return Err(PhysicsError::NonFinite {
            reason: "numerical footprint range-relative projection is non-finite",
        });
    }
    Ok((downrange_m, crossrange_m))
}

fn altitude_above_numerical_cull_m(
    position_eci_m: Vector3<f64>,
    cull_altitude_m: f64,
) -> Result<f64, PhysicsError> {
    let radius_m = position_eci_m.norm();
    if !radius_m.is_finite() || radius_m <= MIN_DIRECTION_NORM {
        return Err(PhysicsError::OutOfEnvelope {
            reason: "numerical footprint position is too close to the Earth center",
        });
    }
    let surface_radius_m = wgs84_radial_surface_radius_m(position_eci_m / radius_m)?;
    let altitude_m = radius_m - surface_radius_m;
    let altitude_above_cull_m = altitude_m - cull_altitude_m;
    if !altitude_above_cull_m.is_finite() {
        return Err(PhysicsError::NonFinite {
            reason: "numerical footprint cull altitude calculation is non-finite",
        });
    }
    Ok(altitude_above_cull_m)
}

fn wgs84_radial_surface_radius_m(unit_direction: Vector3<f64>) -> Result<f64, PhysicsError> {
    let b_m = WGS84_A_M * (1.0 - WGS84_FLATTENING);
    let x2_y2 = unit_direction.x * unit_direction.x + unit_direction.y * unit_direction.y;
    let z2 = unit_direction.z * unit_direction.z;
    let denominator = (x2_y2 / (WGS84_A_M * WGS84_A_M) + z2 / (b_m * b_m)).sqrt();
    if !denominator.is_finite() || denominator <= 0.0 {
        return Err(PhysicsError::NonFinite {
            reason: "WGS84 radial surface radius calculation is invalid",
        });
    }
    Ok(1.0 / denominator)
}

fn geodetic_degrees_from_wgs84_ecef(
    ecef_m: Vector3<f64>,
) -> Result<(Option<f64>, Option<f64>), PhysicsError> {
    if !ecef_m.iter().all(|v| v.is_finite()) {
        return Err(PhysicsError::NonFinite {
            reason: "numerical footprint ECEF landing position is non-finite",
        });
    }
    let x = ecef_m.x;
    let y = ecef_m.y;
    let z = ecef_m.z;
    if x == 0.0 && y == 0.0 && z == 0.0 {
        return Err(PhysicsError::OutOfEnvelope {
            reason: "WGS84 geodetic conversion is undefined at the Earth center",
        });
    }
    let p = (x * x + y * y).sqrt();
    let longitude_rad = y.atan2(x);
    let a_m = WGS84_A_M;
    let e2 = WGS84_ECCENTRICITY_SQUARED;
    let b_m = a_m * (1.0 - e2).sqrt();
    let ep2 = e2 / (1.0 - e2);
    let theta = (z * a_m).atan2(p * b_m);
    let sin_theta = theta.sin();
    let cos_theta = theta.cos();
    let latitude_rad = (z + ep2 * b_m * sin_theta * sin_theta * sin_theta)
        .atan2(p - e2 * a_m * cos_theta * cos_theta * cos_theta);
    let latitude_deg = latitude_rad.to_degrees();
    let longitude_deg = normalize_longitude_deg(longitude_rad.to_degrees());
    if !latitude_deg.is_finite() || !longitude_deg.is_finite() {
        return Err(PhysicsError::NonFinite {
            reason: "WGS84 geodetic conversion produced non-finite coordinates",
        });
    }
    Ok((Some(latitude_deg), Some(longitude_deg)))
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
        FootprintMonteCarloInput, FootprintSampleInput, GravityTurnAscentReference,
        IdealStagingBudgetAnalysis, MomentumConservingStageSeparation,
        NumericalGravityRangeSafetyFootprint, PitchProgramAscentReference, RangeSafetyFootprint,
        STAGE_SEPARATION_MOMENTUM_TOLERANCE_KG_M_S, StageMassProperties, StageSeparationModel,
        StagingBudgetAnalysis, StagingBudgetInput, StagingBudgetMode,
        constant_gravity_footprint_monte_carlo,
    };
    use crate::{
        Egm2008ZonalGravity, J2Gravity, PhysicsError, WGS84_A_M, WGS84_J2, WGS84_MU_M3_S2,
        WGS84_OMEGA_RAD_S,
    };
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

    #[test]
    fn j2_numerical_footprint_propagates_to_wgs84_cull() {
        let state = BallisticState {
            position_eci_m: [WGS84_A_M + 1_000.0, 0.0, 0.0],
            velocity_eci_m_s: [0.0, 100.0, 0.0],
            ballistic_coefficient_m2_kg: 0.0,
            time: SimTime::from_seconds(0.0),
        };
        let env = FootprintEnvironment {
            cull_altitude_m: 0.0,
            gravity_m_s2: 0.0,
            launch_origin_eci_m: [WGS84_A_M, 0.0, 0.0],
            geodetic_origin: None,
            dispersion: None,
        };
        let model = NumericalGravityRangeSafetyFootprint::with_limits(
            J2Gravity::new(WGS84_MU_M3_S2, WGS84_A_M, WGS84_J2).unwrap(),
            0.25,
            120.0,
        )
        .unwrap();
        let footprint = model.landing_footprint(&state, &env).unwrap();
        assert!(footprint.time_to_cull_s > 0.0);
        assert!(footprint.downrange_m > 0.0);
        assert!(footprint.crossrange_m.abs() < 1.0e-6);
        assert!(footprint.latitude_deg.is_none());
        assert!(footprint.longitude_deg.is_none());
    }

    #[test]
    fn j2_numerical_footprint_emits_direct_geodetic_output() {
        let initial_radius_m = WGS84_A_M + 1_000.0;
        let state = BallisticState {
            position_eci_m: [initial_radius_m, 0.0, 0.0],
            velocity_eci_m_s: [0.0, WGS84_OMEGA_RAD_S * initial_radius_m + 100.0, 0.0],
            ballistic_coefficient_m2_kg: 0.0,
            time: SimTime::from_seconds(0.0),
        };
        let env = FootprintEnvironment {
            cull_altitude_m: 0.0,
            gravity_m_s2: 0.0,
            launch_origin_eci_m: [WGS84_A_M, 0.0, 0.0],
            geodetic_origin: Some(FootprintGeodeticOrigin {
                latitude_deg: 0.0,
                longitude_deg: 0.0,
                height_m: 0.0,
            }),
            dispersion: None,
        };
        let model = NumericalGravityRangeSafetyFootprint::with_limits(
            J2Gravity::new(WGS84_MU_M3_S2, WGS84_A_M, WGS84_J2).unwrap(),
            0.25,
            120.0,
        )
        .unwrap();
        let footprint = model.landing_footprint(&state, &env).unwrap();
        assert!(footprint.downrange_m > 0.0);
        assert!(footprint.crossrange_m.abs() < 1.0e-6);
        assert!(footprint.latitude_deg.unwrap().abs() < 1.0e-9);
        assert!(footprint.longitude_deg.unwrap() > 0.0);
    }

    #[test]
    fn egm2008_numerical_footprint_propagates_with_dispersion() {
        let state = BallisticState {
            position_eci_m: [WGS84_A_M + 2_000.0, 0.0, 200.0],
            velocity_eci_m_s: [0.0, 80.0, -10.0],
            ballistic_coefficient_m2_kg: 0.0,
            time: SimTime::from_seconds(5.0),
        };
        let env = FootprintEnvironment {
            cull_altitude_m: 0.0,
            gravity_m_s2: 0.0,
            launch_origin_eci_m: [WGS84_A_M, 0.0, 0.0],
            geodetic_origin: None,
            dispersion: Some(FootprintDispersionInput {
                one_sigma_semi_major_m: 40.0,
                one_sigma_semi_minor_m: 15.0,
                orientation_rad: 0.1,
            }),
        };
        let model = NumericalGravityRangeSafetyFootprint::with_limits(
            Egm2008ZonalGravity::wgs84_egm2008_zonal(),
            0.25,
            120.0,
        )
        .unwrap();
        let footprint = model.landing_footprint(&state, &env).unwrap();
        assert!(footprint.time_to_cull_s > 0.0);
        assert!(footprint.downrange_m > 0.0);
        let dispersion = footprint.dispersion_ellipse.unwrap();
        assert_eq!(dispersion.three_sigma_semi_major_m, 120.0);
        assert_eq!(dispersion.three_sigma_semi_minor_m, 45.0);
    }

    #[test]
    fn footprint_monte_carlo_zero_uncertainty_collapses_to_nominal() {
        let nominal = BallisticState {
            position_eci_m: [0.0, 0.0, 100.0],
            velocity_eci_m_s: [20.0, 0.0, -5.0],
            ballistic_coefficient_m2_kg: 0.01,
            time: SimTime::from_seconds(0.0),
        };
        let samples = (0..4)
            .map(|sample_index| FootprintSampleInput {
                sample_index,
                state: nominal,
                wind_eci_m_s: [0.0, 0.0, 0.0],
            })
            .collect();
        let input = FootprintMonteCarloInput::new(nominal, samples, vec![0.5, 0.9]);
        let result =
            constant_gravity_footprint_monte_carlo(&nominal_footprint_env(), &input).unwrap();
        for sample in &result.samples {
            assert!((sample.landing.downrange_m - result.nominal.downrange_m).abs() < 1.0e-12);
            assert!((sample.landing.crossrange_m - result.nominal.crossrange_m).abs() < 1.0e-12);
        }
        assert!(result.dispersion_ellipse.one_sigma_semi_major_m < 1.0e-12);
        assert!(result.dispersion_ellipse.one_sigma_semi_minor_m < 1.0e-12);
        assert!(result.cep50_m < 1.0e-12);
        assert!(result.mean_miss_distance_from_nominal_m < 1.0e-12);
        for quantile in &result.nominal_radial_error_quantiles {
            assert!(quantile.radial_distance_m < 1.0e-12);
        }
    }

    #[test]
    fn footprint_monte_carlo_wind_and_bc_change_spread() {
        let nominal = BallisticState {
            position_eci_m: [0.0, 0.0, 100.0],
            velocity_eci_m_s: [25.0, 2.0, -5.0],
            ballistic_coefficient_m2_kg: 0.01,
            time: SimTime::from_seconds(0.0),
        };
        let mut high_drag = nominal;
        high_drag.ballistic_coefficient_m2_kg = 0.03;
        let samples = vec![
            FootprintSampleInput {
                sample_index: 0,
                state: nominal,
                wind_eci_m_s: [0.0, 0.0, 0.0],
            },
            FootprintSampleInput {
                sample_index: 1,
                state: high_drag,
                wind_eci_m_s: [0.0, 0.0, 0.0],
            },
            FootprintSampleInput {
                sample_index: 2,
                state: nominal,
                wind_eci_m_s: [5.0, 0.0, 0.0],
            },
            FootprintSampleInput {
                sample_index: 3,
                state: nominal,
                wind_eci_m_s: [-5.0, 0.0, 0.0],
            },
        ];
        let input = FootprintMonteCarloInput::new(nominal, samples, vec![0.5, 0.9]);
        let result =
            constant_gravity_footprint_monte_carlo(&nominal_footprint_env(), &input).unwrap();
        assert_eq!(result.samples.len(), 4);
        assert!(result.dispersion_ellipse.one_sigma_semi_major_m > 0.1);
        assert!(result.covariance_downrange_downrange_m2 > 0.0);
        assert!(result.quantiles[1].radial_distance_m >= result.quantiles[0].radial_distance_m);
        assert!(result.cep50_m > 0.0);
        assert!(
            result.nominal_radial_error_quantiles[1].radial_distance_m
                >= result.nominal_radial_error_quantiles[0].radial_distance_m
        );
    }

    #[test]
    fn footprint_monte_carlo_covariance_ellipse_is_stable() {
        let nominal = BallisticState {
            position_eci_m: [0.0, 0.0, 100.0],
            velocity_eci_m_s: [25.0, 2.0, -5.0],
            ballistic_coefficient_m2_kg: 0.01,
            time: SimTime::from_seconds(0.0),
        };
        let samples = vec![
            FootprintSampleInput {
                sample_index: 0,
                state: nominal,
                wind_eci_m_s: [0.0, 0.0, 0.0],
            },
            FootprintSampleInput {
                sample_index: 1,
                state: BallisticState {
                    velocity_eci_m_s: [26.0, 2.5, -5.0],
                    ..nominal
                },
                wind_eci_m_s: [2.0, 0.0, 0.0],
            },
            FootprintSampleInput {
                sample_index: 2,
                state: BallisticState {
                    velocity_eci_m_s: [24.0, 1.0, -5.0],
                    ..nominal
                },
                wind_eci_m_s: [-2.0, 1.0, 0.0],
            },
            FootprintSampleInput {
                sample_index: 3,
                state: BallisticState {
                    velocity_eci_m_s: [25.5, 3.0, -5.0],
                    ..nominal
                },
                wind_eci_m_s: [0.0, -1.0, 0.0],
            },
        ];
        let input = FootprintMonteCarloInput::new(nominal, samples, vec![0.5, 0.9]);
        let result =
            constant_gravity_footprint_monte_carlo(&nominal_footprint_env(), &input).unwrap();
        assert!(
            (result.dispersion_ellipse.one_sigma_semi_major_m - 5.515_605_369_845_789).abs()
                < 1.0e-12
        );
        assert!(
            (result.dispersion_ellipse.one_sigma_semi_minor_m - 0.716_847_007_223_690_8).abs()
                < 1.0e-12
        );
        assert!(
            (result.dispersion_ellipse.orientation_rad - 0.288_884_977_197_166_16).abs() < 1.0e-12
        );
        assert!((result.cep50_m - result.quantiles[0].radial_distance_m).abs() < 1.0e-12);
        assert!(result.mean_miss_distance_from_nominal_m > 0.0);
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

    #[test]
    fn staging_optimal_equal_stages_split_equal_delta_v() {
        let input = StagingBudgetInput {
            stages: vec![
                StageMassProperties {
                    isp_s: 300.0,
                    structural_coefficient: 0.1,
                    structural_mass_kg: None,
                    propellant_mass_kg: None,
                },
                StageMassProperties {
                    isp_s: 300.0,
                    structural_coefficient: 0.1,
                    structural_mass_kg: None,
                    propellant_mass_kg: None,
                },
            ],
            payload_mass_kg: 100.0,
            delta_v_budget_m_s: Some(6000.0),
        };
        let report = IdealStagingBudgetAnalysis::default()
            .analyze(&input)
            .unwrap();
        assert_eq!(report.mode, StagingBudgetMode::Optimal);
        assert!((report.total_delta_v_m_s - 6000.0).abs() < 1.0e-9);
        assert!((report.stages[0].delta_v_m_s - 3000.0).abs() < 1.0e-8);
        assert!((report.stages[1].delta_v_m_s - 3000.0).abs() < 1.0e-8);
        let c = crate::gravity::STANDARD_GRAVITY_M_S2 * 300.0;
        let expected_n = (3000.0_f64 / c).exp();
        assert!((report.stages[0].mass_ratio - expected_n).abs() < 1.0e-12);
    }

    #[test]
    fn staging_forward_budget_matches_tsiolkovsky_sum() {
        let input = StagingBudgetInput {
            stages: vec![
                StageMassProperties {
                    isp_s: 280.0,
                    structural_coefficient: 0.1,
                    structural_mass_kg: Some(100.0),
                    propellant_mass_kg: Some(900.0),
                },
                StageMassProperties {
                    isp_s: 320.0,
                    structural_coefficient: 0.2,
                    structural_mass_kg: Some(20.0),
                    propellant_mass_kg: Some(80.0),
                },
            ],
            payload_mass_kg: 50.0,
            delta_v_budget_m_s: None,
        };
        let report = IdealStagingBudgetAnalysis::default()
            .analyze(&input)
            .unwrap();
        let recomputed: f64 = report
            .stages
            .iter()
            .map(|stage| stage.effective_exhaust_velocity_m_s * stage.mass_ratio.ln())
            .sum();
        assert!((report.total_delta_v_m_s - recomputed).abs() < 1.0e-12);
        assert_eq!(report.mode, StagingBudgetMode::Budget);
    }

    #[test]
    fn staging_optimal_rejects_infeasible_delta_v() {
        let input = StagingBudgetInput {
            stages: vec![StageMassProperties {
                isp_s: 250.0,
                structural_coefficient: 0.2,
                structural_mass_kg: None,
                propellant_mass_kg: None,
            }],
            payload_mass_kg: 10.0,
            delta_v_budget_m_s: Some(100_000.0),
        };
        assert!(matches!(
            IdealStagingBudgetAnalysis::default().analyze(&input),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
    }
}
