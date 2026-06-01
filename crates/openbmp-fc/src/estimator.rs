//! Estimator framework + EKF reference impl.
//!
//! Provides:
//! - `Estimator` trait — `predict(dt)`, `update(measurement)`,
//!   `state()`, `status()`.
//! - `Ekf` — 15-state error-state EKF. Process model: rigid-body
//!   propagation. Measurement updates from bus topics
//!   (`sensor.imu`, `sensor.gnss`, `sensor.magnetometer`,
//!   `sensor.barometer`, `sensor.star_tracker`). Innovation gates
//!   per measurement.
//!
//! Also provides `Mekf`, a 6-state multiplicative quaternion
//! filter for attitude-only estimation. The square-root UKF family —
//! the 15-state [`crate::sr_ukf::SquareRootUkf`] (full
//! state) and 6-state [`crate::sr_ukf::SquareRootUkfAttitude`]
//! (attitude-only) variants — supersedes an earlier classical
//! 6-state `Ukf`.
//!
//! # State vector layout (`Ekf`)
//!
//! ```text
//! x = [ position_eci(3),     // [m]      0..3
//!       velocity_eci(3),     // [m/s]    3..6
//!       attitude_error(3),   // [rad]    6..9   (axis-angle in body)
//!       gyro_bias_body(3),   // [rad/s]  9..12
//!       accel_bias_body(3) ] // [m/s^2]  12..15
//! ```
//!
//! Attitude error is multiplicative: the error-state quaternion is
//! `q_full = q_nominal ⊗ exp(error_axis_angle/2)`. After every update
//! the error is reset and folded into `q_nominal`.

use nalgebra::{Matrix3, SMatrix, SVector, UnitQuaternion, Vector3};
#[cfg(not(feature = "std"))]
use num_traits::Float;
use openbmp_core::{Eci, Position3, SimTime};
use openbmp_mission::CanonicalRegionStates;
use openbmp_physics::atmosphere;
use openbmp_physics::earth;
use openbmp_physics::gravity::{self, ConstantGravity, GravityModel};
use openbmp_physics::magnetic::{EarthDipoleField, MagneticFieldEci};
use std::boxed::Box;
use std::format;
use std::string::ToString;

use crate::bus::Bus;
use crate::error::{ControllerError, EstimatorError};
use crate::params::ParamSection;
use crate::scheduler::{Job, JobContext};
use crate::topics::{
    AttitudeEstimate, BarometerSample, EstimatorLaneSelection, EstimatorMode,
    EstimatorRegimeRegionStatePublish, EstimatorStatus, GnssSample, ImuSample, MagnetometerSample,
    PositionEstimate, StarTrackerSample,
};

/// Adapter that wraps the rich [`openbmp_physics::gravity::GravityModel`]
/// trait (frame-tagged `Position3<Eci>`, `SimTime`, `Result`) into the
/// simple `Vec3 → Vec3` shape the FC's predict / update path uses.
///
/// Gravity model failures are converted into estimator configuration
/// errors so invalid envelopes cannot silently turn into zero
/// acceleration.
struct GravityAdapter {
    inner: Box<dyn GravityModel + Send + Sync>,
}

impl std::fmt::Debug for GravityAdapter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GravityAdapter").finish_non_exhaustive()
    }
}

impl GravityAdapter {
    fn new<G: GravityModel + Send + Sync + 'static>(model: G) -> Self {
        Self {
            inner: Box::new(model),
        }
    }

    fn at(
        &self,
        position_eci_m: Vector3<f64>,
        time: SimTime,
    ) -> Result<Vector3<f64>, EstimatorError> {
        let p = Position3::<Eci>::new(position_eci_m.x, position_eci_m.y, position_eci_m.z);
        self.inner
            .gravity_eci_m_s2(p, time)
            .map_err(|err| EstimatorError::InvalidConfig {
                reason: format!("gravity model rejected query: {err}"),
            })
    }
}

// `STANDARD_GRAVITY_M_S2` is a positive, finite compile-time
// constant; `down_z` only fails on non-finite or negative magnitudes,
// neither of which apply here. The expect is unreachable.
#[allow(clippy::expect_used)]
fn default_constant_gravity_down_z() -> ConstantGravity {
    ConstantGravity::down_z(gravity::STANDARD_GRAVITY_M_S2)
        .expect("STANDARD_GRAVITY_M_S2 is positive and finite")
}

/// Estimator trait. Implementations consume sensor topics from the
/// bus and publish `AttitudeEstimate` / `PositionEstimate` /
/// `EstimatorStatus` topics, plus optional estimator-specific
/// diagnostics such as `EstimatorMode`.
pub trait Estimator {
    /// Static name; appears in dictionaries and event payloads.
    fn name(&self) -> &'static str;

    /// Propagates the filter forward by `dt` using the most recent
    /// IMU sample (or zero-update if no IMU sample is fresh).
    ///
    /// # Errors
    ///
    /// Returns [`EstimatorError::NonFiniteState`] on any non-finite
    /// numerical step.
    fn predict(&mut self, dt: f64) -> Result<(), EstimatorError>;

    /// Applies a measurement update from a typed measurement source.
    ///
    /// # Errors
    ///
    /// Returns [`EstimatorError::InnovationGateRejected`] if the
    /// innovation chi-square test fails the configured gate.
    fn update_imu(&mut self, sample: &ImuSample) -> Result<(), EstimatorError>;

    /// Applies a GNSS measurement update.
    ///
    /// # Errors
    ///
    /// See [`Estimator::update_imu`].
    fn update_gnss(&mut self, sample: &GnssSample) -> Result<(), EstimatorError>;

    /// Applies a barometer measurement update.
    ///
    /// # Errors
    ///
    /// See [`Estimator::update_imu`].
    fn update_baro(&mut self, sample: &BarometerSample) -> Result<(), EstimatorError>;

    /// Applies a magnetometer measurement update.
    ///
    /// # Errors
    ///
    /// See [`Estimator::update_imu`].
    fn update_mag(&mut self, sample: &MagnetometerSample) -> Result<(), EstimatorError>;

    /// Applies a star-tracker (full-attitude) measurement update.
    /// Default is a no-op for estimators that do not consume attitude
    /// fixes; the error-state EKF overrides it.
    ///
    /// # Errors
    ///
    /// See [`Estimator::update_imu`].
    fn update_star_tracker(&mut self, _sample: &StarTrackerSample) -> Result<(), EstimatorError> {
        Ok(())
    }

    /// Returns the current attitude estimate.
    fn attitude(&self) -> AttitudeEstimate;

    /// Returns the current position / velocity estimate.
    fn position(&self) -> PositionEstimate;

    /// Returns the current estimator-status snapshot.
    fn status(&self) -> EstimatorStatus;

    /// Returns an optional mode-diagnostic snapshot for estimators
    /// that track multiple model hypotheses.
    fn estimator_mode(&self) -> Option<EstimatorMode> {
        None
    }

    /// Returns an optional active-lane diagnostic snapshot for
    /// estimators that route multiple redundant estimator lanes.
    fn estimator_lane_selection(&self) -> Option<EstimatorLaneSelection> {
        None
    }

    /// Clears one-cycle diagnostic state before draining this tick's
    /// measurements. Persistent health, covariance, and dead-reckoning
    /// state are left unchanged.
    fn begin_tick(&mut self) {}

    /// Selects the high-dynamics process-noise schedule. Estimators
    /// that do not expose a position / velocity process-noise schedule
    /// leave this as a no-op.
    fn set_high_dynamics_process_noise(&mut self, _active: bool) {}
}

/// Configuration parameters for the EKF.
#[derive(Clone, Debug)]
pub struct EkfParams {
    /// Process-noise standard deviation on attitude rate (rad/s).
    pub sigma_w_gyro: f64,
    /// Process-noise standard deviation on accelerometer bias
    /// random walk (m/s²/√s).
    pub sigma_w_accel_bias: f64,
    /// Process-noise standard deviation on gyro bias random walk
    /// (rad/s/√s).
    pub sigma_w_gyro_bias: f64,
    /// Process-noise standard deviation on the position state
    /// (m/√s). Keeps the position covariance from collapsing so the
    /// filter stays responsive to GNSS. `0` reproduces the legacy
    /// no-process-noise behaviour.
    pub sigma_w_position_m: f64,
    /// Process-noise standard deviation on the velocity state
    /// (m/s/√s). Without it the velocity covariance collapses during
    /// high-dynamics flight and the filter stops trusting GNSS
    /// velocity, dead-reckoning (and drifting) through any subsequent
    /// low-specific-force coast. `0` reproduces the legacy behaviour.
    pub sigma_w_velocity_m_s: f64,
    /// First-order Gauss-Markov gyro-bias time constant. `∞`
    /// preserves the legacy random-walk limit.
    pub tau_gyro_bias_s: f64,
    /// First-order Gauss-Markov accelerometer-bias time constant.
    /// `∞` preserves the legacy random-walk limit.
    pub tau_accel_bias_s: f64,
    /// Measurement-noise standard deviation on each GNSS position
    /// component (m).
    pub sigma_gnss_pos_m: f64,
    /// Measurement-noise standard deviation on each GNSS velocity
    /// component (m/s).
    pub sigma_gnss_vel_m_s: f64,
    /// Measurement-noise standard deviation on barometric altitude
    /// (m).
    pub sigma_baro_alt_m: f64,
    /// Measurement-noise standard deviation on each magnetometer
    /// component (nT).
    pub sigma_mag_nt: f64,
    /// Measurement-noise standard deviation on each star-tracker
    /// attitude-error component (rad). A star tracker provides a full
    /// 3-DOF attitude fix, so it (unlike a single magnetometer vector)
    /// observes rotation about the local field direction too.
    pub sigma_star_tracker_rad: f64,
    /// Optional explicit innovation-gate threshold. Leave as `NaN`
    /// to derive per-measurement gates from
    /// [`EkfParams::innovation_false_alarm_rate`].
    pub innovation_gate: f64,
    /// False-alarm probability used to derive chi-square innovation
    /// gates by measurement dimension when `innovation_gate` is `NaN`.
    pub innovation_false_alarm_rate: f64,
    /// Dead-reckoning timeout in seconds — the time without a GNSS or
    /// star-tracker update after which the filter flags
    /// dead-reckoning.
    pub dead_reckon_timeout_s: f64,
    /// Multiplier on position/velocity process-noise covariance while
    /// the commander-published estimator regime is high dynamics.
    /// `1.0` preserves the base schedule exactly.
    pub high_dynamics_q_scale: f64,
}

impl Default for EkfParams {
    fn default() -> Self {
        Self {
            sigma_w_gyro: 0.01,
            sigma_w_accel_bias: 1.0e-4,
            sigma_w_gyro_bias: 1.0e-5,
            sigma_w_position_m: 0.0,
            sigma_w_velocity_m_s: 0.0,
            tau_gyro_bias_s: f64::INFINITY,
            tau_accel_bias_s: f64::INFINITY,
            sigma_gnss_pos_m: 5.0,
            sigma_gnss_vel_m_s: 0.5,
            sigma_baro_alt_m: 2.0,
            sigma_mag_nt: 100.0,
            sigma_star_tracker_rad: 1.0e-4,
            innovation_gate: f64::NAN,
            innovation_false_alarm_rate: 0.01,
            dead_reckon_timeout_s: 1.5,
            high_dynamics_q_scale: 1.0,
        }
    }
}

impl ParamSection for EkfParams {
    const NAME: &'static str = "estimator.ekf";
}

impl EkfParams {
    fn gate_for_dof(&self, dof: f64) -> f64 {
        innovation_gate_threshold(self.innovation_gate, self.innovation_false_alarm_rate, dof)
    }
}

/// 15-state error-state Extended Kalman Filter.
#[allow(clippy::struct_excessive_bools)] // per-sensor `last_*_updated_this_tick` flags
pub struct Ekf {
    params: EkfParams,
    /// Nominal ECI position (m).
    pos_eci: Vector3<f64>,
    /// Nominal ECI velocity (m/s).
    vel_eci: Vector3<f64>,
    /// Nominal body-to-ECI rotation.
    q_body_to_eci: UnitQuaternion<f64>,
    /// Estimated gyro bias in the body frame (rad/s).
    gyro_bias: Vector3<f64>,
    /// Estimated accel bias in the body frame (m/s²).
    accel_bias: Vector3<f64>,
    /// Body-frame angular velocity, debiased, last predict.
    omega_body: Vector3<f64>,
    /// Error-state covariance, 15×15.
    p: SMatrix<f64, 15, 15>,
    /// Last received IMU sample, if any.
    last_imu: Option<ImuSample>,
    /// Time-since-last GNSS / corrective measurement (s).
    time_since_corrective_s: f64,
    /// Latest sensor chi-square readings.
    last_chi2_imu: f64,
    last_chi2_gnss: f64,
    last_chi2_baro: f64,
    last_chi2_mag: f64,
    last_star_tracker_chi2: f64,
    last_innovation_rejected: bool,
    /// Last whitened innovation per corrective sensor.
    /// Reset every tick by `begin_tick`; set by the corresponding
    /// `update_*` method; reported by `status()` alongside the
    /// `*_updated_this_tick` flag.
    last_gnss_innovation_whitened: [f64; 6],
    last_gnss_updated_this_tick: bool,
    last_baro_innovation_whitened: f64,
    last_baro_updated_this_tick: bool,
    last_mag_innovation_whitened: [f64; 3],
    last_mag_updated_this_tick: bool,
    /// Log-determinant of the innovation covariance
    /// `S` from the most recent measurement update of the matching
    /// sensor. Computed as `2 · Σ log L_diag` from the Cholesky
    /// factorisation already done for the chi-square statistic. The
    /// IMM consumes these as the `log det S_j` term in the per-mode
    /// Gaussian-likelihood `log Λ_j = −0.5 (chi2_j + d log 2π + log
    /// det S_j)`. Reset to `f64::NAN` by `begin_tick` and reported
    /// only when the matching `*_updated_this_tick` flag is `true`.
    last_log_det_s_gnss: f64,
    last_log_det_s_baro: f64,
    last_log_det_s_mag: f64,
    /// `true` once at least one corrective measurement has been
    /// applied.
    initialized: bool,
    /// Gravity model — applied during the predict step so the EKF
    /// integrates inertial acceleration (specific force + gravity).
    gravity: GravityAdapter,
    /// Magnetic-field model — used by `update_mag` to predict the
    /// body-frame magnetic vector and form an innovation.
    mag_field: Box<dyn MagneticFieldEci>,
    /// `true` while the commander-published estimator-regime state is
    /// the canonical high-dynamics mode.
    high_dynamics_q_active: bool,
}

impl std::fmt::Debug for Ekf {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Ekf")
            .field("params", &self.params)
            .field("pos_eci", &self.pos_eci)
            .field("vel_eci", &self.vel_eci)
            .field("q_body_to_eci", &self.q_body_to_eci)
            .field("gyro_bias", &self.gyro_bias)
            .field("accel_bias", &self.accel_bias)
            .field("initialized", &self.initialized)
            .field("time_since_corrective_s", &self.time_since_corrective_s)
            .finish_non_exhaustive()
    }
}

impl Ekf {
    /// Constructs a freshly-initialised EKF anchored at the origin
    /// with identity attitude, zero biases, the default flat-Earth
    /// gravity model (`ConstantGravity::down_z(STANDARD_GRAVITY_M_S2)`), and the academic-
    /// tier dipole magnetic-field model.
    #[must_use]
    pub fn new(params: EkfParams) -> Self {
        let mut p = SMatrix::<f64, 15, 15>::zeros();
        // Generous initial covariance — refined by first update.
        for i in 0..3 {
            p[(i, i)] = 100.0; // position
            p[(i + 3, i + 3)] = 10.0; // velocity
            p[(i + 6, i + 6)] = 0.1; // attitude error
            p[(i + 9, i + 9)] = 0.001; // gyro bias
            p[(i + 12, i + 12)] = 0.01; // accel bias
        }
        Self {
            params,
            pos_eci: Vector3::zeros(),
            vel_eci: Vector3::zeros(),
            q_body_to_eci: UnitQuaternion::identity(),
            gyro_bias: Vector3::zeros(),
            accel_bias: Vector3::zeros(),
            omega_body: Vector3::zeros(),
            p,
            last_imu: None,
            time_since_corrective_s: 0.0,
            last_chi2_imu: 0.0,
            last_chi2_gnss: 0.0,
            last_chi2_baro: 0.0,
            last_chi2_mag: 0.0,
            last_star_tracker_chi2: 0.0,
            last_innovation_rejected: false,
            last_gnss_innovation_whitened: [0.0; 6],
            last_gnss_updated_this_tick: false,
            last_baro_innovation_whitened: 0.0,
            last_baro_updated_this_tick: false,
            last_mag_innovation_whitened: [0.0; 3],
            last_mag_updated_this_tick: false,
            last_log_det_s_gnss: f64::NAN,
            last_log_det_s_baro: f64::NAN,
            last_log_det_s_mag: f64::NAN,
            initialized: false,
            gravity: GravityAdapter::new(default_constant_gravity_down_z()),
            mag_field: Box::new(EarthDipoleField::default()),
            high_dynamics_q_active: false,
        }
    }

    /// Replaces the gravity model. Returns the updated `Ekf` so the
    /// builder pattern can chain onto `Ekf::new`.
    #[must_use]
    pub fn with_gravity_model<G: GravityModel + Send + Sync + 'static>(
        mut self,
        gravity: G,
    ) -> Self {
        self.gravity = GravityAdapter::new(gravity);
        self
    }

    /// Replaces the magnetic-field model. Returns the updated `Ekf`.
    #[must_use]
    pub fn with_mag_field_model<M: MagneticFieldEci + 'static>(mut self, mag: M) -> Self {
        self.mag_field = Box::new(mag);
        self
    }

    /// Seeds the filter with a known initial pose and velocity.
    pub fn seed(
        &mut self,
        pos_eci: Vector3<f64>,
        vel_eci: Vector3<f64>,
        q_body_to_eci: UnitQuaternion<f64>,
    ) {
        self.pos_eci = pos_eci;
        self.vel_eci = vel_eci;
        self.q_body_to_eci = q_body_to_eci;
        self.initialized = true;
    }

    /// Returns the largest covariance diagonal entry.
    #[must_use]
    pub fn covariance_max_diag(&self) -> f64 {
        (0..15).map(|i| self.p[(i, i)]).fold(0.0, f64::max)
    }
}

impl Estimator for Ekf {
    fn name(&self) -> &'static str {
        "estimator.ekf"
    }

    fn predict(&mut self, dt: f64) -> Result<(), EstimatorError> {
        if !dt.is_finite() || dt <= 0.0 {
            return Ok(());
        }
        let Some(imu) = self.last_imu else {
            return Ok(());
        };
        let gyro_bias_decay = gauss_markov_decay(dt, self.params.tau_gyro_bias_s);
        let accel_bias_decay = gauss_markov_decay(dt, self.params.tau_accel_bias_s);
        self.gyro_bias *= gyro_bias_decay;
        self.accel_bias *= accel_bias_decay;
        scale_covariance_state(&mut self.p, 9..12, gyro_bias_decay);
        scale_covariance_state(&mut self.p, 12..15, accel_bias_decay);

        let omega_meas = imu.gyro_rad_s - self.gyro_bias;
        let accel_meas = imu.accel_m_s2 - self.accel_bias;
        self.omega_body = omega_meas;

        // Attitude integration: q_dot = 0.5 * q ⊗ omega
        let dq = quaternion_from_omega(omega_meas, dt);
        self.q_body_to_eci *= dq;
        renormalize_quaternion(&mut self.q_body_to_eci);

        // Specific force in ECI: f_eci = R(q) · accel_meas, where
        // accel_meas is the IMU specific force in body. Inertial
        // acceleration is f_eci + g_eci, where g_eci is the
        // gravity vector returned by the configured model.
        let r_body_to_eci = self.q_body_to_eci.to_rotation_matrix();
        let f_eci = r_body_to_eci * accel_meas;
        let g_eci = self.gravity.at(self.pos_eci, SimTime::ZERO)?;
        let a_eci = f_eci + g_eci;
        self.vel_eci += a_eci * dt;
        self.pos_eci += self.vel_eci * dt;

        // Covariance propagation: Q dt added on the diagonals.
        let position_velocity_q_scale = if self.high_dynamics_q_active
            && self.params.high_dynamics_q_scale.is_finite()
            && self.params.high_dynamics_q_scale > 0.0
        {
            self.params.high_dynamics_q_scale
        } else {
            1.0
        };
        let q_diag = SVector::<f64, 15>::from_iterator((0..15).map(|i| match i {
            0..=2 => {
                self.params.sigma_w_position_m
                    * self.params.sigma_w_position_m
                    * position_velocity_q_scale
                    * dt
            }
            3..=5 => {
                self.params.sigma_w_velocity_m_s
                    * self.params.sigma_w_velocity_m_s
                    * position_velocity_q_scale
                    * dt
            }
            6..=8 => self.params.sigma_w_gyro * self.params.sigma_w_gyro * dt,
            9..=11 => gauss_markov_process_variance(
                self.params.sigma_w_gyro_bias,
                dt,
                self.params.tau_gyro_bias_s,
            ),
            12..=14 => gauss_markov_process_variance(
                self.params.sigma_w_accel_bias,
                dt,
                self.params.tau_accel_bias_s,
            ),
            _ => 0.0,
        }));
        for i in 0..15 {
            self.p[(i, i)] += q_diag[i];
        }

        self.time_since_corrective_s += dt;

        if !self.pos_eci.iter().all(|v| v.is_finite())
            || !self.vel_eci.iter().all(|v| v.is_finite())
        {
            return Err(EstimatorError::NonFiniteState { stage: "predict" });
        }
        Ok(())
    }

    fn update_imu(&mut self, sample: &ImuSample) -> Result<(), EstimatorError> {
        self.last_imu = Some(*sample);
        // The IMU is a propagation source, not a corrective update.
        // The chi-square slot is reserved for an inertial-self-test
        // (e.g. accel-magnitude vs g) that a future revision can
        // surface.
        self.last_chi2_imu = 0.0;
        Ok(())
    }

    fn update_gnss(&mut self, sample: &GnssSample) -> Result<(), EstimatorError> {
        // GNSS is fused as two INDEPENDENT, independently-gated 3-D blocks
        // — position (states 0..3) then velocity (states 3..6) —
        // processed sequentially. The GNSS measurement-noise R is
        // block-diagonal (position and velocity receiver errors are
        // independent), so sequential processing is mathematically
        // identical to the joint 6-D update when BOTH blocks pass their
        // gate; the difference is that each block is gated on its own
        // 3-DOF chi-square. A velocity-innovation spike under high thrust
        // therefore rejects ONLY the velocity block — the good position
        // fix is still applied — instead of the previous single combined
        // 6-D gate dropping the whole fix and dead-reckoning the filter
        // into divergence. (Sequential per-sensor gating is the standard
        // robust-INS/GNSS treatment.)
        self.last_gnss_updated_this_tick = true;
        let sigma_pos2 = self.params.sigma_gnss_pos_m * self.params.sigma_gnss_pos_m;
        let sigma_vel2 = self.params.sigma_gnss_vel_m_s * self.params.sigma_gnss_vel_m_s;
        let (chi2_p, ld_p, w_p, applied_p) =
            self.gnss_block_update(sample.position_eci_m, self.pos_eci, 0, sigma_pos2)?;
        // Velocity block reads the velocity AFTER the position update so
        // the two form a proper sequential (joint-equivalent) pass.
        let (chi2_v, ld_v, w_v, applied_v) =
            self.gnss_block_update(sample.velocity_eci_m_s, self.vel_eci, 3, sigma_vel2)?;

        // Combined diagnostics for the FDIR GLRT / IMM likelihood: the
        // whitened innovation is the concatenation of the two blocks and
        // the chi-square / log-det are their sums, so the invariant
        // `‖ν̃‖² = chi2` still holds (asserted by
        // `whitened_innovation_norm_squared_equals_chi2`).
        self.last_gnss_innovation_whitened[..3].copy_from_slice(&w_p);
        self.last_gnss_innovation_whitened[3..].copy_from_slice(&w_v);
        self.last_chi2_gnss = chi2_p + chi2_v;
        self.last_log_det_s_gnss = ld_p + ld_v;

        if applied_p || applied_v {
            self.time_since_corrective_s = 0.0;
            self.initialized = true;
        }
        if !applied_p || !applied_v {
            self.last_innovation_rejected = true;
            // Report the rejected block's statistic (velocity first, since
            // it is the one that spikes under high dynamics).
            let (chi2, gate) = if applied_v {
                (chi2_p, self.params.gate_for_dof(3.0))
            } else {
                (chi2_v, self.params.gate_for_dof(3.0))
            };
            return Err(EstimatorError::InnovationGateRejected {
                measurement: "gnss",
                chi2,
                gate,
            });
        }
        self.last_innovation_rejected = false;
        Ok(())
    }

    fn update_baro(&mut self, sample: &BarometerSample) -> Result<(), EstimatorError> {
        // Convert measured pressure → geopotential altitude via the
        // shared USSA76 troposphere helper, then form a 1-D position-z
        // innovation. The authority lives in `openbmp-physics`, not in
        // simulator-only `openbmp-physics`.
        let measured_alt_m =
            atmosphere::pressure_altitude_troposphere_m(sample.pressure_pa - sample.bias_pa);
        let predicted_alt_m = self.pos_eci.z;
        let innovation = measured_alt_m - predicted_alt_m;

        // 1-D measurement: H selects state element 2 (position z).
        let mut h: SMatrix<f64, 1, 15> = SMatrix::zeros();
        h[(0, 2)] = 1.0;
        let r_var =
            SMatrix::<f64, 1, 1>::new(self.params.sigma_baro_alt_m * self.params.sigma_baro_alt_m);
        let s = h * self.p * h.transpose() + r_var;
        let s_scalar = s[(0, 0)];
        if s_scalar <= 0.0 || !s_scalar.is_finite() {
            return Err(EstimatorError::InvalidConfig {
                reason: "baro innovation covariance non-positive".to_string(),
            });
        }
        let chi2 = innovation * innovation / s_scalar;
        self.last_chi2_baro = chi2;
        // Scalar whitening — ν̃ = ν / √S, so (ν̃)² = chi2.
        self.last_baro_innovation_whitened = innovation / s_scalar.sqrt();
        self.last_baro_updated_this_tick = true;
        // Scalar log det S = log s_scalar.
        self.last_log_det_s_baro = s_scalar.ln();
        let gate = self.params.gate_for_dof(1.0);
        if chi2 > gate {
            self.last_innovation_rejected = true;
            return Err(EstimatorError::InnovationGateRejected {
                measurement: "baro",
                chi2,
                gate,
            });
        }
        let k = self.p * h.transpose() / s_scalar;
        let dx: SVector<f64, 15> = k * innovation;
        self.apply_state_update(&dx);
        self.p = joseph_covariance_update(&self.p, &k, &h, &r_var);
        self.last_innovation_rejected = false;
        Ok(())
    }

    fn update_mag(&mut self, sample: &MagnetometerSample) -> Result<(), EstimatorError> {
        if !sample.healthy {
            return Ok(());
        }
        let measured = sample.field_body_nt - sample.hard_iron_body_nt;
        let r_var = SMatrix::<f64, 3, 3>::from_diagonal_element(
            self.params.sigma_mag_nt * self.params.sigma_mag_nt,
        );
        let mut final_k = SMatrix::<f64, 15, 3>::zeros();
        let mut final_h = SMatrix::<f64, 3, 15>::zeros();
        for iteration in 0..3 {
            let (innovation, h) = self.mag_innovation_and_jacobian(measured, sample.time);
            let s = h * self.p * h.transpose() + r_var;
            let Some(chol) = s.cholesky() else {
                return Err(EstimatorError::InvalidConfig {
                    reason: "mag innovation covariance non-positive-definite".to_string(),
                });
            };
            let s_inv = chol.inverse();
            let chi2 = innovation.dot(&(s_inv * innovation));
            if iteration == 0 {
                self.last_chi2_mag = chi2;
                // Whiten the iteration-0 innovation. The
                // Gauss-Newton iterations refine the state estimate but
                // not the innovation distribution; the GLRT consumes
                // the first-iteration whitened residual.
                let l = chol.l();
                let Some(whitened) = l.solve_lower_triangular(&innovation) else {
                    return Err(EstimatorError::InvalidConfig {
                        reason:
                            "mag Cholesky lower-triangular solve failed (should be unreachable)"
                                .to_string(),
                    });
                };
                for i in 0..3 {
                    self.last_mag_innovation_whitened[i] = whitened[i];
                }
                self.last_mag_updated_this_tick = true;
                // log det S = 2 · Σ log L_ii.
                let mut log_det_s = 0.0_f64;
                for i in 0..3 {
                    log_det_s += l[(i, i)].ln();
                }
                log_det_s *= 2.0;
                self.last_log_det_s_mag = log_det_s;
                let gate = self.params.gate_for_dof(3.0);
                if chi2 > gate {
                    self.last_innovation_rejected = true;
                    return Err(EstimatorError::InnovationGateRejected {
                        measurement: "mag",
                        chi2,
                        gate,
                    });
                }
            }
            let k = self.p * h.transpose() * s_inv;
            let dx: SVector<f64, 15> = k * innovation;
            let converged = dx.norm() < 1.0e-6 * self.state_norm().max(1.0);
            self.apply_state_update(&dx);
            final_k = k;
            final_h = h;
            if converged {
                break;
            }
        }
        self.p = joseph_covariance_update(&self.p, &final_k, &final_h, &r_var);
        self.last_innovation_rejected = false;
        Ok(())
    }

    #[allow(clippy::many_single_char_names)] // standard EKF naming: h, s, k.
    fn update_star_tracker(&mut self, sample: &StarTrackerSample) -> Result<(), EstimatorError> {
        if !sample.healthy {
            return Ok(());
        }
        // The star tracker delivers the full attitude q_eci_to_body. Form
        // the body-frame small-angle error between the measured and
        // nominal attitude — a direct observation of the attitude-error
        // state (indices 6..9), so H is the identity on those columns.
        // A star tracker is a 3-DOF attitude fix, so (unlike a single
        // magnetometer vector, which leaves rotation about the local field
        // unobservable) it constrains EVERY axis — this is what resolves
        // the off-pole / equatorial observability gap.
        let [mx, my, mz, mw] = sample.q_eci_to_body_xyzw;
        // q_body_to_eci(measured) = inverse(q_eci_to_body) = conjugate for
        // a unit quaternion: (x, y, z, w) -> (-x, -y, -z, w).
        let q_be_meas_xyzw = [-mx, -my, -mz, mw];
        let q_nom = self.q_body_to_eci;
        let innovation =
            quaternion_error_small_angle([q_nom.i, q_nom.j, q_nom.k, q_nom.w], q_be_meas_xyzw);
        let mut h: SMatrix<f64, 3, 15> = SMatrix::zeros();
        for i in 0..3 {
            h[(i, i + 6)] = 1.0;
        }
        let r_var = SMatrix::<f64, 3, 3>::from_diagonal_element(
            self.params.sigma_star_tracker_rad * self.params.sigma_star_tracker_rad,
        );
        let s = h * self.p * h.transpose() + r_var;
        let Some(chol) = s.cholesky() else {
            return Err(EstimatorError::InvalidConfig {
                reason: "star-tracker innovation covariance non-positive-definite".to_string(),
            });
        };
        let s_inv = chol.inverse();
        let chi2 = innovation.dot(&(s_inv * innovation));
        self.last_star_tracker_chi2 = chi2;
        let gate = self.params.gate_for_dof(3.0);
        if chi2 > gate {
            self.last_innovation_rejected = true;
            return Err(EstimatorError::InnovationGateRejected {
                measurement: "star_tracker",
                chi2,
                gate,
            });
        }
        let k = self.p * h.transpose() * s_inv;
        let dx: SVector<f64, 15> = k * innovation;
        self.apply_state_update(&dx);
        self.p = joseph_covariance_update(&self.p, &k, &h, &r_var);
        self.time_since_corrective_s = 0.0;
        self.last_innovation_rejected = false;
        self.initialized = true;
        Ok(())
    }

    fn attitude(&self) -> AttitudeEstimate {
        let q = self.q_body_to_eci.into_inner();
        AttitudeEstimate {
            time: openbmp_core::SimTime::ZERO,
            q_body_to_eci_xyzw: [q.i, q.j, q.k, q.w],
            omega_body_rad_s: self.omega_body,
            gyro_bias_body_rad_s: self.gyro_bias,
        }
    }

    fn position(&self) -> PositionEstimate {
        PositionEstimate {
            time: openbmp_core::SimTime::ZERO,
            position_eci_m: self.pos_eci,
            velocity_eci_m_s: self.vel_eci,
            accel_bias_body_m_s2: self.accel_bias,
        }
    }

    fn status(&self) -> EstimatorStatus {
        EstimatorStatus {
            time: openbmp_core::SimTime::ZERO,
            initialized: self.initialized,
            dead_reckoning: self.time_since_corrective_s > self.params.dead_reckon_timeout_s,
            imu_chi2: self.last_chi2_imu,
            gnss_chi2: self.last_chi2_gnss,
            baro_chi2: self.last_chi2_baro,
            mag_chi2: self.last_chi2_mag,
            star_tracker_chi2: self.last_star_tracker_chi2,
            innovation_rejected: self.last_innovation_rejected,
            gnss_innovation_whitened: self.last_gnss_innovation_whitened,
            gnss_updated_this_tick: self.last_gnss_updated_this_tick,
            baro_innovation_whitened: self.last_baro_innovation_whitened,
            baro_updated_this_tick: self.last_baro_updated_this_tick,
            mag_innovation_whitened: self.last_mag_innovation_whitened,
            mag_updated_this_tick: self.last_mag_updated_this_tick,
        }
    }

    fn begin_tick(&mut self) {
        self.last_chi2_imu = 0.0;
        self.last_chi2_gnss = 0.0;
        self.last_chi2_baro = 0.0;
        self.last_chi2_mag = 0.0;
        self.last_star_tracker_chi2 = 0.0;
        self.last_innovation_rejected = false;
        self.last_gnss_innovation_whitened = [0.0; 6];
        self.last_gnss_updated_this_tick = false;
        self.last_baro_innovation_whitened = 0.0;
        self.last_baro_updated_this_tick = false;
        self.last_mag_innovation_whitened = [0.0; 3];
        self.last_mag_updated_this_tick = false;
        self.last_log_det_s_gnss = f64::NAN;
        self.last_log_det_s_baro = f64::NAN;
        self.last_log_det_s_mag = f64::NAN;
    }

    fn set_high_dynamics_process_noise(&mut self, active: bool) {
        self.high_dynamics_q_active = active;
    }
}

impl Ekf {
    /// One 3-component GNSS block (position or velocity) as an
    /// independent, gated, Joseph-form update on state indices
    /// `start..start+3`. Returns `(chi2, log_det_s, whitened, applied)`;
    /// `applied` is false when the block's 3-DOF chi-square exceeds the
    /// gate (the state/covariance are then left untouched).
    #[allow(clippy::many_single_char_names)] // standard EKF naming: h, s, k, l, etc.
    fn gnss_block_update(
        &mut self,
        measurement: Vector3<f64>,
        predicted: Vector3<f64>,
        start: usize,
        var: f64,
    ) -> Result<(f64, f64, [f64; 3], bool), EstimatorError> {
        let innovation = measurement - predicted;
        let mut h: SMatrix<f64, 3, 15> = SMatrix::zeros();
        for i in 0..3 {
            h[(i, start + i)] = 1.0;
        }
        let r_var = SMatrix::<f64, 3, 3>::from_diagonal_element(var);
        let s = h * self.p * h.transpose() + r_var;
        let Some(chol) = s.cholesky() else {
            return Err(EstimatorError::InvalidConfig {
                reason: "GNSS innovation covariance non-positive-definite".to_string(),
            });
        };
        let l = chol.l();
        let Some(whitened) = l.solve_lower_triangular(&innovation) else {
            return Err(EstimatorError::InvalidConfig {
                reason: "Cholesky lower-triangular solve failed (should be unreachable)"
                    .to_string(),
            });
        };
        // log det S = 2 · Σ log L_ii (locked-order; no FMA).
        let mut log_det_s = 0.0_f64;
        for i in 0..3 {
            log_det_s += l[(i, i)].ln();
        }
        log_det_s *= 2.0;
        let s_inv = chol.inverse();
        let chi2 = innovation.dot(&(s_inv * innovation));
        let w = [whitened[0], whitened[1], whitened[2]];
        let gate = self.params.gate_for_dof(3.0);
        if chi2 > gate {
            return Ok((chi2, log_det_s, w, false));
        }
        let k = self.p * h.transpose() * s_inv;
        let dx: SVector<f64, 15> = k * innovation;
        self.apply_state_update(&dx);
        self.p = joseph_covariance_update(&self.p, &k, &h, &r_var);
        Ok((chi2, log_det_s, w, true))
    }

    fn apply_state_update(&mut self, dx: &SVector<f64, 15>) {
        self.pos_eci += Vector3::new(dx[0], dx[1], dx[2]);
        self.vel_eci += Vector3::new(dx[3], dx[4], dx[5]);
        let attitude_error = Vector3::new(dx[6], dx[7], dx[8]);
        if attitude_error.norm() > 0.0 {
            let dq = quaternion_from_axis_angle(attitude_error);
            self.q_body_to_eci *= dq;
            renormalize_quaternion(&mut self.q_body_to_eci);
        }
        self.gyro_bias += Vector3::new(dx[9], dx[10], dx[11]);
        self.accel_bias += Vector3::new(dx[12], dx[13], dx[14]);
    }

    fn mag_innovation_and_jacobian(
        &self,
        measured_body_nt: Vector3<f64>,
        time: SimTime,
    ) -> (Vector3<f64>, SMatrix<f64, 3, 15>) {
        let r_eci_to_body = self.q_body_to_eci.to_rotation_matrix().transpose();
        let predicted = r_eci_to_body.matrix() * self.mag_field.field_eci_nt(self.pos_eci, time);
        let innovation = measured_body_nt - predicted;
        let mut h: SMatrix<f64, 3, 15> = SMatrix::zeros();
        let skew = skew_symmetric(predicted);
        for r in 0..3 {
            for c in 0..3 {
                h[(r, c + 6)] = skew[(r, c)];
            }
        }
        (innovation, h)
    }

    fn state_norm(&self) -> f64 {
        stack6(self.pos_eci, self.vel_eci).norm()
            + self.gyro_bias.norm()
            + self.accel_bias.norm()
            + 1.0
    }

    /// Log-determinant of the innovation covariance `S`
    /// from the most recent measurement update of the matching
    /// sensor. `f64::NAN` when no update of that sensor occurred on
    /// the current tick.
    ///
    /// Used by [`crate::imm::ImmEstimator`] as the `log det S_j` term
    /// in the per-mode Gaussian-likelihood formula
    /// `log Λ_j = −0.5 (chi2_j + d log 2π + log det S_j)`.
    #[must_use]
    pub fn last_log_det_s_gnss(&self) -> f64 {
        self.last_log_det_s_gnss
    }

    /// See [`Ekf::last_log_det_s_gnss`].
    #[must_use]
    pub fn last_log_det_s_baro(&self) -> f64 {
        self.last_log_det_s_baro
    }

    /// See [`Ekf::last_log_det_s_gnss`].
    #[must_use]
    pub fn last_log_det_s_mag(&self) -> f64 {
        self.last_log_det_s_mag
    }

    /// Full internal-state snapshot suitable for IMM
    /// mixing. Returned in the same order as the 15-element error
    /// state: `(pos, vel, q, gyro_bias, accel_bias, P)`. The
    /// counterpart [`Ekf::set_internal_state`] writes them back; the
    /// pair round-trips bit-exactly on every component.
    #[allow(clippy::type_complexity)] // tuple shape mirrors the 15-state error-state ordering
    #[must_use]
    pub fn internal_state(
        &self,
    ) -> (
        Vector3<f64>,
        Vector3<f64>,
        UnitQuaternion<f64>,
        Vector3<f64>,
        Vector3<f64>,
        SMatrix<f64, 15, 15>,
    ) {
        (
            self.pos_eci,
            self.vel_eci,
            self.q_body_to_eci,
            self.gyro_bias,
            self.accel_bias,
            self.p,
        )
    }

    /// Write a full internal-state snapshot back into
    /// the EKF. The IMM mixing step calls this after computing the
    /// per-mode mixed prior. The quaternion is renormalised after
    /// writeback so naive linear blending of mode quaternions stays
    /// on the unit sphere.
    pub fn set_internal_state(
        &mut self,
        pos_eci: Vector3<f64>,
        vel_eci: Vector3<f64>,
        q_body_to_eci: UnitQuaternion<f64>,
        gyro_bias: Vector3<f64>,
        accel_bias: Vector3<f64>,
        p: SMatrix<f64, 15, 15>,
    ) {
        self.pos_eci = pos_eci;
        self.vel_eci = vel_eci;
        self.q_body_to_eci = q_body_to_eci;
        renormalize_quaternion(&mut self.q_body_to_eci);
        self.gyro_bias = gyro_bias;
        self.accel_bias = accel_bias;
        self.p = p;
    }
}

fn stack6(a: Vector3<f64>, b: Vector3<f64>) -> SVector<f64, 6> {
    SVector::<f64, 6>::from_iterator((0..6).map(|i| if i < 3 { a[i] } else { b[i - 3] }))
}

use openbmp_physics::kinematics::{
    quaternion_error_small_angle, quaternion_from_axis_angle, quaternion_from_omega,
    renormalize_quaternion, skew_symmetric,
};

fn joseph_covariance_update<const N: usize, const M: usize>(
    p: &SMatrix<f64, N, N>,
    k: &SMatrix<f64, N, M>,
    h: &SMatrix<f64, M, N>,
    r: &SMatrix<f64, M, M>,
) -> SMatrix<f64, N, N> {
    let i_kh = SMatrix::<f64, N, N>::identity() - k * h;
    let updated = i_kh * p * i_kh.transpose() + k * r * k.transpose();
    (updated + updated.transpose()) * 0.5
}

fn gauss_markov_decay(dt_s: f64, tau_s: f64) -> f64 {
    if tau_s.is_finite() && tau_s > 0.0 {
        (-dt_s / tau_s).exp()
    } else {
        1.0
    }
}

fn gauss_markov_process_variance(sigma: f64, dt_s: f64, tau_s: f64) -> f64 {
    if tau_s.is_finite() && tau_s > 0.0 {
        0.5 * sigma * sigma * tau_s * (1.0 - (-2.0 * dt_s / tau_s).exp())
    } else {
        sigma * sigma * dt_s
    }
}

fn scale_covariance_state<const N: usize>(
    p: &mut SMatrix<f64, N, N>,
    indices: std::ops::Range<usize>,
    scale: f64,
) {
    if (scale - 1.0).abs() <= f64::EPSILON {
        return;
    }
    for i in indices.clone() {
        for col in 0..N {
            p[(i, col)] *= scale;
        }
    }
    for row in 0..N {
        for j in indices.clone() {
            p[(row, j)] *= scale;
        }
    }
}

fn innovation_gate_threshold(configured_gate: f64, false_alarm_rate: f64, dof: f64) -> f64 {
    if configured_gate.is_finite() && configured_gate > 0.0 {
        return configured_gate;
    }
    let probability = (1.0 - false_alarm_rate).clamp(0.5, 0.999_999_999);
    openbmp_physics::statistics::chi_square_inverse_cdf_wilson_hilferty(probability, dof)
}

/// Job that runs the estimator on each scheduler tick. The job
/// drains the latest IMU/GNSS/Baro/Mag samples from the bus,
/// drives the predict/update cycle, and republishes the resulting
/// estimate topics.
#[derive(Debug)]
pub struct EstimatorJob<E: Estimator + std::fmt::Debug> {
    estimator: E,
    name: &'static str,
    last_imu_seq: u64,
    last_gnss_seq: u64,
    last_baro_seq: u64,
    last_mag_seq: u64,
    last_star_tracker_seq: u64,
    last_predict_time_s: f64,
}

impl<E: Estimator + std::fmt::Debug + 'static> EstimatorJob<E> {
    /// Constructs the job. `name` is the static job name used by the
    /// scheduler dictionary.
    #[must_use]
    pub fn new(estimator: E) -> Self {
        Self {
            estimator,
            name: "estimator.tick",
            last_imu_seq: 0,
            last_gnss_seq: 0,
            last_baro_seq: 0,
            last_mag_seq: 0,
            last_star_tracker_seq: 0,
            last_predict_time_s: 0.0,
        }
    }

    /// Borrows the underlying estimator immutably.
    pub fn estimator(&self) -> &E {
        &self.estimator
    }

    fn drain_imu(&mut self, bus: &Bus) -> Result<(), ControllerError> {
        let seq = bus.sequence::<ImuSample>()?;
        if seq.value() > self.last_imu_seq {
            if let Some((sample, _)) = bus.latest::<ImuSample>()? {
                self.estimator.update_imu(&sample)?;
            }
            self.last_imu_seq = seq.value();
        }
        Ok(())
    }

    fn drain_gnss(&mut self, bus: &Bus) -> Result<(), ControllerError> {
        let seq = bus.sequence::<GnssSample>()?;
        if seq.value() > self.last_gnss_seq {
            if let Some((sample, _)) = bus.latest::<GnssSample>()? {
                let _ = self.estimator.update_gnss(&sample);
            }
            self.last_gnss_seq = seq.value();
        }
        Ok(())
    }

    fn drain_baro(&mut self, bus: &Bus) -> Result<(), ControllerError> {
        let seq = bus.sequence::<BarometerSample>()?;
        if seq.value() > self.last_baro_seq {
            if let Some((sample, _)) = bus.latest::<BarometerSample>()? {
                let _ = self.estimator.update_baro(&sample);
            }
            self.last_baro_seq = seq.value();
        }
        Ok(())
    }

    fn drain_mag(&mut self, bus: &Bus) -> Result<(), ControllerError> {
        let seq = bus.sequence::<MagnetometerSample>()?;
        if seq.value() > self.last_mag_seq {
            if let Some((sample, _)) = bus.latest::<MagnetometerSample>()? {
                let _ = self.estimator.update_mag(&sample);
            }
            self.last_mag_seq = seq.value();
        }
        Ok(())
    }

    fn drain_star_tracker(&mut self, bus: &Bus) -> Result<(), ControllerError> {
        let seq = bus.sequence::<StarTrackerSample>()?;
        if seq.value() > self.last_star_tracker_seq {
            if let Some((sample, _)) = bus.latest::<StarTrackerSample>()? {
                let _ = self.estimator.update_star_tracker(&sample);
            }
            self.last_star_tracker_seq = seq.value();
        }
        Ok(())
    }

    fn sync_process_noise_regime(&mut self, bus: &Bus) {
        let boost_state = CanonicalRegionStates::estimator_boost_mode().value();
        let high_dynamics = bus
            .latest::<EstimatorRegimeRegionStatePublish>()
            .ok()
            .flatten()
            .is_some_and(|(state, _)| state.state_id == boost_state);
        self.estimator
            .set_high_dynamics_process_noise(high_dynamics);
    }
}

impl<E: Estimator + std::fmt::Debug + 'static> Job for EstimatorJob<E> {
    fn name(&self) -> &'static str {
        self.name
    }

    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        self.estimator.begin_tick();
        self.sync_process_noise_regime(ctx.bus);
        // Drain sensor topics (in deterministic order).
        let _ = self.drain_imu(ctx.bus);
        let _ = self.drain_baro(ctx.bus);
        let _ = self.drain_mag(ctx.bus);
        let _ = self.drain_star_tracker(ctx.bus);
        let _ = self.drain_gnss(ctx.bus);

        // Predict.
        let now_s = ctx.clock.now().as_seconds();
        let dt = if self.last_predict_time_s > 0.0 {
            (now_s - self.last_predict_time_s).max(0.0)
        } else {
            0.0
        };
        if dt > 0.0 {
            let _ = self.estimator.predict(dt);
        }
        self.last_predict_time_s = now_s;

        // Republish estimates.
        let mut attitude = self.estimator.attitude();
        attitude.time = ctx.clock.now();
        let _ = ctx.bus.publish(attitude);

        let mut position = self.estimator.position();
        position.time = ctx.clock.now();
        let _ = ctx.bus.publish(position);

        let mut status = self.estimator.status();
        status.time = ctx.clock.now();
        let _ = ctx.bus.publish(status);

        if let Some(mut mode) = self.estimator.estimator_mode() {
            mode.time = ctx.clock.now();
            let _ = ctx.bus.publish(mode);
        }

        if let Some(mut lanes) = self.estimator.estimator_lane_selection() {
            lanes.time = ctx.clock.now();
            let _ = ctx.bus.publish(lanes);
        }

        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn ekf_seed_initialises_state() {
        let mut ekf = Ekf::new(EkfParams::default());
        ekf.seed(
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::new(4.0, 5.0, 6.0),
            UnitQuaternion::identity(),
        );
        let pos = ekf.position();
        assert!((pos.position_eci_m.x - 1.0).abs() < f64::EPSILON);
        assert!((pos.velocity_eci_m_s.y - 5.0).abs() < f64::EPSILON);
    }

    #[test]
    fn ekf_high_dynamics_q_scale_multiplies_position_velocity_process_noise() {
        let params = EkfParams {
            sigma_w_position_m: 2.0,
            sigma_w_velocity_m_s: 3.0,
            high_dynamics_q_scale: 4.0,
            ..EkfParams::default()
        };
        let imu = ImuSample {
            time: SimTime::ZERO,
            gyro_rad_s: Vector3::zeros(),
            accel_m_s2: Vector3::new(0.0, 0.0, gravity::STANDARD_GRAVITY_M_S2),
            healthy: true,
        };

        let mut nominal = Ekf::new(params.clone());
        nominal.update_imu(&imu).unwrap();
        let p0_pos = nominal.p[(0, 0)];
        let p0_vel = nominal.p[(3, 3)];
        nominal.predict(1.0).unwrap();
        assert!((nominal.p[(0, 0)] - p0_pos - 4.0).abs() < 1.0e-12);
        assert!((nominal.p[(3, 3)] - p0_vel - 9.0).abs() < 1.0e-12);

        let mut boosted = Ekf::new(params);
        boosted.set_high_dynamics_process_noise(true);
        boosted.update_imu(&imu).unwrap();
        let p0_pos = boosted.p[(0, 0)];
        let p0_vel = boosted.p[(3, 3)];
        boosted.predict(1.0).unwrap();
        assert!((boosted.p[(0, 0)] - p0_pos - 16.0).abs() < 1.0e-12);
        assert!((boosted.p[(3, 3)] - p0_vel - 36.0).abs() < 1.0e-12);
    }

    #[test]
    fn estimator_job_reads_estimator_regime_topic_for_high_dynamics_schedule() {
        let bus = Bus::new();
        bus.register::<EstimatorRegimeRegionStatePublish>().unwrap();
        let mut job = EstimatorJob::new(Ekf::new(EkfParams::default()));

        job.sync_process_noise_regime(&bus);
        assert!(!job.estimator().high_dynamics_q_active);

        bus.publish(EstimatorRegimeRegionStatePublish {
            state_id: CanonicalRegionStates::estimator_boost_mode().value(),
        })
        .unwrap();
        job.sync_process_noise_regime(&bus);
        assert!(job.estimator().high_dynamics_q_active);

        bus.publish(EstimatorRegimeRegionStatePublish {
            state_id: openbmp_mission::PhaseId::from_path("mission.regions.estimator_regime.coast")
                .value(),
        })
        .unwrap();
        job.sync_process_noise_regime(&bus);
        assert!(!job.estimator().high_dynamics_q_active);
    }

    #[test]
    fn gauss_markov_noise_density_converges_to_stationary_variance() {
        let sigma = 0.2;
        let tau_s = 10.0;
        let dt_s = 0.1;
        let phi = gauss_markov_decay(dt_s, tau_s);
        let q = gauss_markov_process_variance(sigma, dt_s, tau_s);
        let mut p = 0.0;
        for _ in 0..10_000 {
            p = phi * phi * p + q;
        }
        let expected = 0.5 * sigma * sigma * tau_s;
        assert!((p - expected).abs() < 1.0e-10);
    }

    #[test]
    fn gravity_model_errors_are_not_silently_zeroed() {
        let adapter = GravityAdapter::new(openbmp_physics::gravity::PointMassGravity::wgs84());
        let err = adapter.at(Vector3::zeros(), SimTime::ZERO).unwrap_err();
        assert!(matches!(err, EstimatorError::InvalidConfig { .. }));
    }

    #[test]
    fn ekf_predict_advances_position_under_velocity() {
        // Disable gravity for this kinematics-only test.
        let mut ekf = Ekf::new(EkfParams::default())
            .with_gravity_model(ConstantGravity::new(Vector3::zeros()).unwrap());
        ekf.seed(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(10.0, 0.0, 0.0),
            UnitQuaternion::identity(),
        );
        ekf.update_imu(&ImuSample {
            time: openbmp_core::SimTime::ZERO,
            gyro_rad_s: Vector3::zeros(),
            accel_m_s2: Vector3::zeros(),
            healthy: true,
        })
        .unwrap();
        for _ in 0..100 {
            ekf.predict(0.01).unwrap();
        }
        let pos = ekf.position();
        // 100 * 0.01 s = 1 s at 10 m/s in +x.
        assert!((pos.position_eci_m.x - 10.0).abs() < 1.0);
    }

    #[test]
    fn ekf_free_fall_matches_analytic_kinematics() {
        // Free fall: IMU senses zero specific force, gravity model
        // pulls velocity down at -9.81 m/s². After 10 s, position
        // z should be ≈ -0.5 · 9.81 · 100 = −490.5 m, velocity z
        // ≈ −98.1 m/s, with the 1-cm tolerance the plan specifies.
        let mut ekf = Ekf::new(EkfParams::default());
        ekf.seed(
            Vector3::zeros(),
            Vector3::zeros(),
            UnitQuaternion::identity(),
        );
        ekf.update_imu(&ImuSample {
            time: openbmp_core::SimTime::ZERO,
            gyro_rad_s: Vector3::zeros(),
            accel_m_s2: Vector3::zeros(),
            healthy: true,
        })
        .unwrap();
        let dt = 0.001;
        let n = 10_000; // 10 s
        for _ in 0..n {
            ekf.predict(dt).unwrap();
        }
        let pos = ekf.position();
        let elapsed_s = f64::from(n) * dt;
        let expected_position_z = -0.5 * gravity::STANDARD_GRAVITY_M_S2 * elapsed_s * elapsed_s;
        let expected_velocity_z = -gravity::STANDARD_GRAVITY_M_S2 * elapsed_s;
        // Forward-Euler integration: z_n = -0.5 · g · (n · dt)² is
        // exact only as dt → 0. With dt = 1 ms, the trapezoidal
        // analytic answer is z = -0.5 · g · t²; the
        // integrated answer differs by O(dt · g · t) ≈ 4.9 cm.
        assert!(
            (pos.position_eci_m.z - expected_position_z).abs() < 0.05,
            "position z {} not within 5 cm of {expected_position_z} m",
            pos.position_eci_m.z,
        );
        assert!(
            (pos.velocity_eci_m_s.z - expected_velocity_z).abs() < 0.05,
            "velocity z {} not within 5 cm/s of {expected_velocity_z} m/s",
            pos.velocity_eci_m_s.z
        );
    }

    #[test]
    fn ekf_point_mass_gravity_pulls_toward_earth_center() {
        // With the central (point-mass) nav gravity model, free fall at
        // a position on the +x axis accelerates the velocity toward the
        // origin (−x), NOT along a fixed −z as the constant-Z model
        // would. This is the position-dependent behaviour an orbital
        // ascent needs. Magnitude matches µ/r².
        use openbmp_physics::gravity::PointMassGravity;
        let r_m = 6_371_000.0;
        let mu = openbmp_physics::frames::WGS84_MU_M3_S2;
        let mut ekf = Ekf::new(EkfParams::default()).with_gravity_model(PointMassGravity::wgs84());
        ekf.seed(
            Vector3::new(r_m, 0.0, 0.0),
            Vector3::zeros(),
            UnitQuaternion::identity(),
        );
        ekf.update_imu(&ImuSample {
            time: openbmp_core::SimTime::ZERO,
            gyro_rad_s: Vector3::zeros(),
            accel_m_s2: Vector3::zeros(),
            healthy: true,
        })
        .unwrap();
        let dt = 0.001;
        let n = 100; // 0.1 s — short, so position barely moves
        for _ in 0..n {
            ekf.predict(dt).unwrap();
        }
        let pos = ekf.position();
        let elapsed_s = f64::from(n) * dt;
        let g_expected = mu / (r_m * r_m); // ≈ 9.82 m/s² toward −x
        let expected_vx = -g_expected * elapsed_s;
        assert!(
            (pos.velocity_eci_m_s.x - expected_vx).abs() < 1.0e-3,
            "central gravity vx {} not ≈ {expected_vx} m/s (toward Earth center)",
            pos.velocity_eci_m_s.x,
        );
        // Off-axis components stay ~zero (gravity is purely radial here),
        // unlike the constant-Z model which would put it all on z.
        assert!(pos.velocity_eci_m_s.y.abs() < 1.0e-9);
        assert!(pos.velocity_eci_m_s.z.abs() < 1.0e-9);
    }

    // -----------------------------------------------------------------
    // Whitened-innovation export invariants.
    //
    // These tests pin the contract that the windowed-mean-shift GLRT
    // detector relies on: every per-sensor `update_*` method emits a
    // whitened innovation vector `ν̃ = L⁻¹ ν` whose squared L₂ norm
    // equals the chi-square statistic the estimator already publishes.
    // Bit-stable construction is asserted through `to_bits` so the
    // determinism contract carries from the EKF into the FDIR
    // detector's sliding window unchanged.
    // -----------------------------------------------------------------

    fn ekf_for_innovation_test() -> Ekf {
        let mut ekf = Ekf::new(EkfParams {
            sigma_gnss_pos_m: 5.0,
            sigma_gnss_vel_m_s: 0.5,
            sigma_baro_alt_m: 2.0,
            sigma_mag_nt: 200.0,
            // Use an explicit gate well above any realistic chi-square
            // so update_* never short-circuits before it sets the
            // whitened-innovation slot.
            innovation_gate: 1.0e6,
            ..EkfParams::default()
        });
        ekf.seed(
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(0.0, 0.0, 0.0),
            UnitQuaternion::identity(),
        );
        ekf
    }

    #[test]
    fn ekf_gnss_whitened_innovation_norm_squared_equals_chi2() {
        let mut ekf = ekf_for_innovation_test();
        ekf.begin_tick();
        ekf.update_gnss(&GnssSample {
            time: SimTime::ZERO,
            position_eci_m: Vector3::new(20.0, 10.0, 5.0),
            velocity_eci_m_s: Vector3::new(0.5, -0.5, 0.0),
            position_bias_eci_m: Vector3::zeros(),
            healthy: true,
        })
        .unwrap();
        let status = ekf.status();
        assert!(status.gnss_updated_this_tick);
        let norm_sq: f64 = status.gnss_innovation_whitened.iter().map(|v| v * v).sum();
        assert!(
            (norm_sq - status.gnss_chi2).abs() < 1.0e-9,
            "‖ν̃‖² = {norm_sq} should equal chi2 = {} for GNSS",
            status.gnss_chi2
        );
    }

    #[test]
    fn ekf_gnss_velocity_spike_rejects_velocity_keeps_position() {
        // A GNSS fix with a good position but a large velocity-component
        // outlier must apply the position correction and reject ONLY the
        // velocity block. The previous single combined 6-D gate dropped
        // the whole fix (including the good position), which under thrust
        // dead-reckoned the filter into divergence.
        let mut ekf = Ekf::new(EkfParams::default());
        ekf.seed(
            Vector3::zeros(),
            Vector3::zeros(),
            UnitQuaternion::identity(),
        );
        ekf.begin_tick();
        let res = ekf.update_gnss(&GnssSample {
            time: SimTime::ZERO,
            position_eci_m: Vector3::new(2.0, 0.0, 0.0), // small → passes 3-DOF gate
            velocity_eci_m_s: Vector3::new(20.0, 0.0, 0.0), // huge → fails velocity gate
            position_bias_eci_m: Vector3::zeros(),
            healthy: true,
        });
        // The velocity block is rejected, so the call reports a gate breach…
        assert!(
            res.is_err(),
            "a velocity-component outlier should report a gate rejection"
        );
        let est = ekf.position();
        // …but the position correction WAS applied (moved toward the fix)…
        assert!(
            est.position_eci_m.x > 1.0,
            "position should still be corrected, got x = {}",
            est.position_eci_m.x
        );
        // …and the velocity was NOT corrupted by the rejected outlier.
        assert!(
            est.velocity_eci_m_s.x.abs() < 1.0,
            "velocity outlier should be rejected, got x = {}",
            est.velocity_eci_m_s.x
        );
    }

    #[test]
    fn ekf_star_tracker_update_corrects_attitude_toward_measurement() {
        // Seeded attitude is identity. The star tracker reports the true
        // attitude q_body_to_eci = R_z(+0.01 rad) (so q_eci_to_body is its
        // inverse, R_z(-0.01)). The EKF attitude must rotate toward +z.
        let mut ekf = ekf_for_innovation_test();
        ekf.begin_tick();
        let half = 0.005_f64; // half of the 0.01 rad rotation
        ekf.update_star_tracker(&StarTrackerSample {
            time: SimTime::ZERO,
            q_eci_to_body_xyzw: [0.0, 0.0, -half.sin(), half.cos()],
            healthy: true,
        })
        .unwrap();
        let q = ekf.attitude().q_body_to_eci_xyzw; // [x, y, z, w]
        assert!(
            q[2] > 1.0e-4,
            "attitude should rotate toward the measured +z (qz>0), got qz = {}",
            q[2]
        );
        assert!(ekf.status().star_tracker_chi2 >= 0.0);
    }

    #[test]
    fn ekf_baro_whitened_innovation_squared_equals_chi2() {
        let mut ekf = ekf_for_innovation_test();
        ekf.begin_tick();
        ekf.update_baro(&BarometerSample {
            time: SimTime::ZERO,
            pressure_pa: 90_000.0, // ~1 km altitude vs. seeded z = 1 m
            bias_pa: 0.0,
            healthy: true,
        })
        .unwrap();
        let status = ekf.status();
        assert!(status.baro_updated_this_tick);
        let nu_sq = status.baro_innovation_whitened * status.baro_innovation_whitened;
        assert!(
            (nu_sq - status.baro_chi2).abs() < 1.0e-9,
            "(ν̃)² = {nu_sq} should equal chi2 = {} for baro",
            status.baro_chi2
        );
    }

    #[test]
    fn ekf_mag_whitened_innovation_norm_squared_equals_chi2() {
        let mut ekf = ekf_for_innovation_test();
        ekf.begin_tick();
        // The dipole field at the seeded position is non-zero; using a
        // mismatched measured field guarantees a non-trivial innovation.
        ekf.update_mag(&MagnetometerSample {
            time: SimTime::ZERO,
            field_body_nt: Vector3::new(20_000.0, 5_000.0, -30_000.0),
            hard_iron_body_nt: Vector3::zeros(),
            healthy: true,
        })
        .unwrap();
        let status = ekf.status();
        assert!(status.mag_updated_this_tick);
        let norm_sq: f64 = status.mag_innovation_whitened.iter().map(|v| v * v).sum();
        assert!(
            (norm_sq - status.mag_chi2).abs() < 1.0e-9,
            "‖ν̃‖² = {norm_sq} should equal chi2 = {} for mag",
            status.mag_chi2
        );
    }

    #[test]
    fn ekf_begin_tick_clears_whitened_innovation_slots() {
        let mut ekf = ekf_for_innovation_test();
        ekf.begin_tick();
        ekf.update_gnss(&GnssSample {
            time: SimTime::ZERO,
            position_eci_m: Vector3::new(20.0, 10.0, 5.0),
            velocity_eci_m_s: Vector3::new(0.5, -0.5, 0.0),
            position_bias_eci_m: Vector3::zeros(),
            healthy: true,
        })
        .unwrap();
        assert!(ekf.status().gnss_updated_this_tick);
        ekf.begin_tick();
        let status = ekf.status();
        assert!(!status.gnss_updated_this_tick);
        assert_eq!(status.gnss_innovation_whitened, [0.0; 6]);
        assert!(!status.baro_updated_this_tick);
        assert!(!status.mag_updated_this_tick);
    }

    #[test]
    fn ekf_whitened_innovation_is_bit_stable_across_two_runs() {
        let sample = GnssSample {
            time: SimTime::ZERO,
            position_eci_m: Vector3::new(2.5, -1.25, 0.75),
            velocity_eci_m_s: Vector3::new(0.1, -0.2, 0.05),
            position_bias_eci_m: Vector3::zeros(),
            healthy: true,
        };
        let mut a = ekf_for_innovation_test();
        a.begin_tick();
        a.update_gnss(&sample).unwrap();
        let mut b = ekf_for_innovation_test();
        b.begin_tick();
        b.update_gnss(&sample).unwrap();
        let s_a = a.status();
        let s_b = b.status();
        for i in 0..6 {
            assert_eq!(
                s_a.gnss_innovation_whitened[i].to_bits(),
                s_b.gnss_innovation_whitened[i].to_bits(),
                "GNSS whitened innovation component {i} not bit-stable",
            );
        }
        assert_eq!(s_a.gnss_chi2.to_bits(), s_b.gnss_chi2.to_bits());
    }

    // -----------------------------------------------------------------
    // IMM-supporting EKF surface (log_det_s, internal_state).
    //
    // The IMM (`crate::imm::ImmEstimator`) reads each mode's
    // `log_det_s_*` to form per-mode Gaussian likelihoods, and
    // round-trips state via `internal_state` / `set_internal_state`
    // for the mixing step. These tests pin those contracts.
    // -----------------------------------------------------------------

    #[test]
    fn ekf_log_det_s_gnss_matches_two_sum_log_l_diag_after_update() {
        let mut ekf = ekf_for_innovation_test();
        ekf.begin_tick();
        // Pre-update: log det should be NaN (no update this tick).
        assert!(ekf.last_log_det_s_gnss().is_nan());
        ekf.update_gnss(&GnssSample {
            time: SimTime::ZERO,
            position_eci_m: Vector3::new(20.0, 10.0, 5.0),
            velocity_eci_m_s: Vector3::new(0.5, -0.5, 0.0),
            position_bias_eci_m: Vector3::zeros(),
            healthy: true,
        })
        .unwrap();
        let log_det = ekf.last_log_det_s_gnss();
        assert!(
            log_det.is_finite(),
            "log det should be finite after a successful update; got {log_det}",
        );
        // Reconstruct expected via independent Cholesky on the H P H' + R
        // form. Innovation covariance is diagonal at this initial
        // covariance because P is initialised diagonal and H is the
        // identity on the first 6 elements; pin it analytically.
        // P_pos = 100, P_vel = 10 (per Ekf::new). R diag is
        // sigma_pos² then sigma_vel². Det = product of all 6 diagonal
        // entries; log det = sum of logs.
        let p_diag: [f64; 6] = [100.0, 100.0, 100.0, 10.0, 10.0, 10.0];
        let r_diag: [f64; 6] = [25.0, 25.0, 25.0, 0.25, 0.25, 0.25];
        let expected: f64 = p_diag
            .iter()
            .zip(r_diag.iter())
            .map(|(p, r)| (p + r).ln())
            .sum();
        assert!(
            (log_det - expected).abs() < 1.0e-9,
            "log det = {log_det} should equal Σ log(P_diag + R_diag) = {expected}",
        );
    }

    #[test]
    fn ekf_log_det_s_per_sensor_resets_in_begin_tick() {
        let mut ekf = ekf_for_innovation_test();
        ekf.begin_tick();
        ekf.update_gnss(&GnssSample {
            time: SimTime::ZERO,
            position_eci_m: Vector3::new(20.0, 10.0, 5.0),
            velocity_eci_m_s: Vector3::new(0.5, -0.5, 0.0),
            position_bias_eci_m: Vector3::zeros(),
            healthy: true,
        })
        .unwrap();
        assert!(ekf.last_log_det_s_gnss().is_finite());
        ekf.begin_tick();
        assert!(ekf.last_log_det_s_gnss().is_nan());
        assert!(ekf.last_log_det_s_baro().is_nan());
        assert!(ekf.last_log_det_s_mag().is_nan());
    }

    #[test]
    fn ekf_internal_state_set_then_get_round_trips_bit_exactly() {
        let mut ekf = Ekf::new(EkfParams::default());
        // Seed with a non-trivial state covering all 5 sub-vectors.
        let pos = Vector3::new(1.5, -2.25, 3.75);
        let vel = Vector3::new(-0.1, 0.2, 0.3);
        let q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.4);
        let gyro_bias = Vector3::new(1.0e-3, -2.0e-3, 3.0e-4);
        let accel_bias = Vector3::new(0.05, -0.02, 0.01);
        let mut p = SMatrix::<f64, 15, 15>::identity() * 0.123;
        // Make P non-diagonal to test full-matrix round-trip.
        p[(0, 1)] = 0.04;
        p[(1, 0)] = 0.04;
        ekf.set_internal_state(pos, vel, q, gyro_bias, accel_bias, p);
        let (got_pos, got_vel, got_q, got_gyro, got_accel, got_p) = ekf.internal_state();
        for i in 0..3 {
            assert_eq!(got_pos[i].to_bits(), pos[i].to_bits(), "pos[{i}]");
            assert_eq!(got_vel[i].to_bits(), vel[i].to_bits(), "vel[{i}]");
            assert_eq!(got_gyro[i].to_bits(), gyro_bias[i].to_bits(), "gyro[{i}]");
            assert_eq!(
                got_accel[i].to_bits(),
                accel_bias[i].to_bits(),
                "accel[{i}]"
            );
        }
        // Quaternion: set_internal_state renormalises, so we can only
        // bit-check the input-was-already-unit case.
        let qc = q.into_inner().coords;
        let got_qc = got_q.into_inner().coords;
        for i in 0..4 {
            assert_eq!(qc[i].to_bits(), got_qc[i].to_bits(), "q[{i}]");
        }
        for i in 0..15 {
            for j in 0..15 {
                assert_eq!(got_p[(i, j)].to_bits(), p[(i, j)].to_bits(), "P[{i},{j}]");
            }
        }
    }
}

// ---------------------------------------------------------------------
// MEKF — multiplicative EKF for quaternion attitude
// ---------------------------------------------------------------------

/// Configuration parameters for the MEKF.
#[derive(Clone, Debug)]
pub struct MekfParams {
    /// Process-noise standard deviation on attitude rate (rad/s).
    pub sigma_w_gyro: f64,
    /// Process-noise standard deviation on gyro bias random walk
    /// (rad/s/√s).
    pub sigma_w_gyro_bias: f64,
    /// First-order Gauss-Markov gyro-bias time constant. `∞`
    /// preserves the legacy random-walk limit.
    pub tau_gyro_bias_s: f64,
    /// Measurement-noise standard deviation on each magnetometer
    /// component (nT).
    pub sigma_mag_nt: f64,
    /// Measurement-noise standard deviation on star-tracker
    /// quaternion components.
    pub sigma_star_tracker: f64,
    /// Innovation-gate threshold (chi-square ratio).
    pub innovation_gate: f64,
    /// False-alarm probability used to derive chi-square innovation
    /// gates by measurement dimension when `innovation_gate` is `NaN`.
    pub innovation_false_alarm_rate: f64,
}

impl Default for MekfParams {
    fn default() -> Self {
        Self {
            sigma_w_gyro: 0.005,
            sigma_w_gyro_bias: 1e-6,
            tau_gyro_bias_s: f64::INFINITY,
            sigma_mag_nt: 50.0,
            sigma_star_tracker: 1e-4,
            innovation_gate: f64::NAN,
            innovation_false_alarm_rate: 0.01,
        }
    }
}

impl ParamSection for MekfParams {
    const NAME: &'static str = "estimator.mekf";
}

impl MekfParams {
    fn gate_for_dof(&self, dof: f64) -> f64 {
        innovation_gate_threshold(self.innovation_gate, self.innovation_false_alarm_rate, dof)
    }
}

/// Multiplicative EKF for quaternion attitude. Uses a 6-state error
/// vector (3-axis attitude error + 3-axis gyro bias). The attitude
/// quaternion is propagated in nominal and the small-angle error is
/// reset after each update.
pub struct Mekf {
    params: MekfParams,
    q_body_to_eci: UnitQuaternion<f64>,
    gyro_bias: Vector3<f64>,
    omega_body: Vector3<f64>,
    p: SMatrix<f64, 6, 6>,
    last_imu: Option<ImuSample>,
    last_chi2_mag: f64,
    last_innovation_rejected: bool,
    /// Last whitened mag innovation; reset by `begin_tick`.
    last_mag_innovation_whitened: [f64; 3],
    last_mag_updated_this_tick: bool,
    initialized: bool,
    /// Reference position (ECI) used to evaluate the magnetic-field
    /// model. The MEKF is attitude-only, so it doesn't track its own
    /// position; the embedder may seed a constant via [`Mekf::seed`].
    reference_position_eci_m: Vector3<f64>,
    /// Magnetic-field model — used by `update_mag` to predict the
    /// expected body-frame field given the current attitude.
    mag_field: Box<dyn MagneticFieldEci>,
}

impl std::fmt::Debug for Mekf {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Mekf")
            .field("params", &self.params)
            .field("q_body_to_eci", &self.q_body_to_eci)
            .field("gyro_bias", &self.gyro_bias)
            .field("initialized", &self.initialized)
            .field("last_chi2_mag", &self.last_chi2_mag)
            .finish_non_exhaustive()
    }
}

impl Mekf {
    /// Constructs a fresh MEKF with identity attitude and zero bias.
    #[must_use]
    pub fn new(params: MekfParams) -> Self {
        let mut p = SMatrix::<f64, 6, 6>::zeros();
        for i in 0..3 {
            p[(i, i)] = 0.1;
            p[(i + 3, i + 3)] = 0.001;
        }
        Self {
            params,
            q_body_to_eci: UnitQuaternion::identity(),
            gyro_bias: Vector3::zeros(),
            omega_body: Vector3::zeros(),
            p,
            last_imu: None,
            last_chi2_mag: 0.0,
            last_innovation_rejected: false,
            last_mag_innovation_whitened: [0.0; 3],
            last_mag_updated_this_tick: false,
            initialized: false,
            reference_position_eci_m: Vector3::new(earth::MEAN_RADIUS_M, 0.0, 0.0),
            mag_field: Box::new(EarthDipoleField::default()),
        }
    }

    /// Replaces the magnetic-field model.
    #[must_use]
    pub fn with_mag_field_model<M: MagneticFieldEci + 'static>(mut self, mag: M) -> Self {
        self.mag_field = Box::new(mag);
        self
    }

    /// Seeds the MEKF with a known initial attitude.
    pub fn seed(&mut self, q_body_to_eci: UnitQuaternion<f64>) {
        self.q_body_to_eci = q_body_to_eci;
        self.initialized = true;
    }

    /// Replaces the reference position used to evaluate the magnetic
    /// field model. Default: equatorial-surface (6 371 000 m, 0, 0).
    pub fn set_reference_position(&mut self, position_eci_m: Vector3<f64>) {
        self.reference_position_eci_m = position_eci_m;
    }

    /// Returns the largest covariance diagonal entry.
    #[must_use]
    pub fn covariance_max_diag(&self) -> f64 {
        (0..6).map(|i| self.p[(i, i)]).fold(0.0, f64::max)
    }
}

impl Estimator for Mekf {
    fn name(&self) -> &'static str {
        "estimator.mekf"
    }

    fn predict(&mut self, dt: f64) -> Result<(), EstimatorError> {
        if !dt.is_finite() || dt <= 0.0 {
            return Ok(());
        }
        let Some(imu) = self.last_imu else {
            return Ok(());
        };
        let gyro_bias_decay = gauss_markov_decay(dt, self.params.tau_gyro_bias_s);
        self.gyro_bias *= gyro_bias_decay;
        scale_covariance_state(&mut self.p, 3..6, gyro_bias_decay);

        let omega = imu.gyro_rad_s - self.gyro_bias;
        self.omega_body = omega;
        let dq = quaternion_from_omega(omega, dt);
        self.q_body_to_eci *= dq;
        renormalize_quaternion(&mut self.q_body_to_eci);
        // Diagonal Q dt update.
        for i in 0..3 {
            self.p[(i, i)] += self.params.sigma_w_gyro * self.params.sigma_w_gyro * dt;
            self.p[(i + 3, i + 3)] += gauss_markov_process_variance(
                self.params.sigma_w_gyro_bias,
                dt,
                self.params.tau_gyro_bias_s,
            );
        }
        Ok(())
    }

    fn update_imu(&mut self, sample: &ImuSample) -> Result<(), EstimatorError> {
        self.last_imu = Some(*sample);
        Ok(())
    }

    fn update_gnss(&mut self, _sample: &GnssSample) -> Result<(), EstimatorError> {
        // MEKF is attitude-only.
        Ok(())
    }

    fn update_baro(&mut self, _sample: &BarometerSample) -> Result<(), EstimatorError> {
        Ok(())
    }

    fn update_mag(&mut self, sample: &MagnetometerSample) -> Result<(), EstimatorError> {
        if !sample.healthy {
            return Ok(());
        }
        let measured = sample.field_body_nt - sample.hard_iron_body_nt;
        let r_var = SMatrix::<f64, 3, 3>::from_diagonal_element(
            self.params.sigma_mag_nt * self.params.sigma_mag_nt,
        );
        let mut final_k = SMatrix::<f64, 6, 3>::zeros();
        let mut final_h = SMatrix::<f64, 3, 6>::zeros();
        let mut final_attitude_error = Vector3::zeros();
        for iteration in 0..3 {
            let (innovation, h) = self.mag_innovation_and_jacobian(measured, sample.time);
            let s = h * self.p * h.transpose() + r_var;
            let Some(chol) = s.cholesky() else {
                return Err(EstimatorError::InvalidConfig {
                    reason: "MEKF mag innovation covariance non-positive-definite".to_string(),
                });
            };
            let s_inv = chol.inverse();
            let chi2 = innovation.dot(&(s_inv * innovation));
            if iteration == 0 {
                self.last_chi2_mag = chi2;
                // Whitened residual ν̃ = L⁻¹ ν, ‖ν̃‖² = chi2.
                let l = chol.l();
                let Some(whitened) = l.solve_lower_triangular(&innovation) else {
                    return Err(EstimatorError::InvalidConfig {
                        reason:
                            "mag Cholesky lower-triangular solve failed (should be unreachable)"
                                .to_string(),
                    });
                };
                for i in 0..3 {
                    self.last_mag_innovation_whitened[i] = whitened[i];
                }
                self.last_mag_updated_this_tick = true;
                let gate = self.params.gate_for_dof(3.0);
                if chi2 > gate {
                    self.last_innovation_rejected = true;
                    return Err(EstimatorError::InnovationGateRejected {
                        measurement: "mag",
                        chi2,
                        gate,
                    });
                }
            }
            let k = self.p * h.transpose() * s_inv;
            let dx: SVector<f64, 6> = k * innovation;
            let attitude_error = Vector3::new(dx[0], dx[1], dx[2]);
            self.apply_error_state_update(&dx);
            final_attitude_error = attitude_error;
            final_k = k;
            final_h = h;
            if dx.norm() < 1.0e-6 * (self.gyro_bias.norm() + 1.0) {
                break;
            }
        }
        let joseph = joseph_covariance_update(&self.p, &final_k, &final_h, &r_var);
        self.p = markley_covariance_reset(&joseph, final_attitude_error);
        self.last_innovation_rejected = false;
        self.initialized = true;
        Ok(())
    }

    fn attitude(&self) -> AttitudeEstimate {
        let q = self.q_body_to_eci.into_inner();
        AttitudeEstimate {
            time: openbmp_core::SimTime::ZERO,
            q_body_to_eci_xyzw: [q.i, q.j, q.k, q.w],
            omega_body_rad_s: self.omega_body,
            gyro_bias_body_rad_s: self.gyro_bias,
        }
    }

    fn position(&self) -> PositionEstimate {
        // MEKF is attitude-only; position passes through as zero.
        PositionEstimate {
            time: openbmp_core::SimTime::ZERO,
            position_eci_m: Vector3::zeros(),
            velocity_eci_m_s: Vector3::zeros(),
            accel_bias_body_m_s2: Vector3::zeros(),
        }
    }

    fn status(&self) -> EstimatorStatus {
        EstimatorStatus {
            time: openbmp_core::SimTime::ZERO,
            initialized: self.initialized,
            dead_reckoning: false,
            imu_chi2: 0.0,
            gnss_chi2: 0.0,
            baro_chi2: 0.0,
            mag_chi2: self.last_chi2_mag,
            star_tracker_chi2: 0.0,
            innovation_rejected: self.last_innovation_rejected,
            gnss_innovation_whitened: [0.0; 6],
            gnss_updated_this_tick: false,
            baro_innovation_whitened: 0.0,
            baro_updated_this_tick: false,
            mag_innovation_whitened: self.last_mag_innovation_whitened,
            mag_updated_this_tick: self.last_mag_updated_this_tick,
        }
    }

    fn begin_tick(&mut self) {
        self.last_chi2_mag = 0.0;
        self.last_innovation_rejected = false;
        self.last_mag_innovation_whitened = [0.0; 3];
        self.last_mag_updated_this_tick = false;
    }
}

impl Mekf {
    fn apply_error_state_update(&mut self, dx: &SVector<f64, 6>) {
        let attitude_error = Vector3::new(dx[0], dx[1], dx[2]);
        if attitude_error.norm() > 0.0 {
            let dq = quaternion_from_axis_angle(attitude_error);
            self.q_body_to_eci *= dq;
            renormalize_quaternion(&mut self.q_body_to_eci);
        }
        self.gyro_bias += Vector3::new(dx[3], dx[4], dx[5]);
    }

    fn mag_innovation_and_jacobian(
        &self,
        measured_body_nt: Vector3<f64>,
        time: SimTime,
    ) -> (Vector3<f64>, SMatrix<f64, 3, 6>) {
        let r_eci_to_body = self.q_body_to_eci.to_rotation_matrix().transpose();
        let predicted = r_eci_to_body.matrix()
            * self
                .mag_field
                .field_eci_nt(self.reference_position_eci_m, time);
        let innovation = measured_body_nt - predicted;
        let mut h: SMatrix<f64, 3, 6> = SMatrix::zeros();
        let skew = skew_symmetric(predicted);
        for r in 0..3 {
            for c in 0..3 {
                h[(r, c)] = skew[(r, c)];
            }
        }
        (innovation, h)
    }
}

fn markley_covariance_reset(
    p: &SMatrix<f64, 6, 6>,
    attitude_error: Vector3<f64>,
) -> SMatrix<f64, 6, 6> {
    let skew = skew_symmetric(attitude_error);
    let g = Matrix3::<f64>::identity() - 0.5 * skew + (skew * skew) / 12.0;
    let mut reset = SMatrix::<f64, 6, 6>::identity();
    for r in 0..3 {
        for c in 0..3 {
            reset[(r, c)] = g[(r, c)];
        }
    }
    let out = reset * p * reset.transpose();
    (out + out.transpose()) * 0.5
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod mekf_tests {
    use super::*;

    #[test]
    fn mekf_predict_advances_attitude_under_omega() {
        let mut mekf = Mekf::new(MekfParams::default());
        mekf.update_imu(&ImuSample {
            time: openbmp_core::SimTime::ZERO,
            gyro_rad_s: Vector3::new(0.0, 0.0, 1.0),
            accel_m_s2: Vector3::zeros(),
            healthy: true,
        })
        .unwrap();
        for _ in 0..100 {
            mekf.predict(0.01).unwrap();
        }
        // 1 rad over 1 s — quaternion w should have rotated.
        let att = mekf.attitude();
        assert!((att.q_body_to_eci_xyzw[3] - 1.0_f64).abs() > 0.05);
    }
}
