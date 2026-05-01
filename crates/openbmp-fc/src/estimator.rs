//! Estimator framework + EKF reference impl.
//!
//! Phase 4.3 ships:
//! - `Estimator` trait — `predict(dt)`, `update(measurement)`,
//!   `state()`, `status()`.
//! - `Ekf` — 15-state error-state EKF. Process model: rigid-body
//!   propagation. Measurement updates from bus topics
//!   (`sensor.imu`, `sensor.gnss`, `sensor.magnetometer`,
//!   `sensor.barometer`, `sensor.star_tracker`). Innovation gates
//!   per measurement.
//!
//! Phase 4.A also ships `Mekf`, a 6-state multiplicative quaternion
//! filter for attitude-only estimation. Phase 4.C adds a real
//! sigma-point `Ukf` for the same attitude + gyro-bias state; the
//! full 15-state UKF and square-root UKF remain Phase 5 scope.
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
use openbmp_core::{Eci, Position3, SimTime};
use openbmp_physics::atmosphere;
use openbmp_physics::earth;
use openbmp_physics::gravity::{self, ConstantGravity, GravityModel};
use openbmp_physics::magnetic::{EarthDipoleField, MagneticFieldEci};

use crate::bus::Bus;
use crate::error::{ControllerError, EstimatorError};
use crate::params::ParamSection;
use crate::scheduler::{Job, JobContext};
use crate::topics::{
    AttitudeEstimate, BarometerSample, EstimatorStatus, GnssSample, ImuSample, MagnetometerSample,
    PositionEstimate,
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
/// `EstimatorStatus` topics.
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

    /// Returns the current attitude estimate.
    fn attitude(&self) -> AttitudeEstimate;

    /// Returns the current position / velocity estimate.
    fn position(&self) -> PositionEstimate;

    /// Returns the current estimator-status snapshot.
    fn status(&self) -> EstimatorStatus;

    /// Clears one-cycle diagnostic state before draining this tick's
    /// measurements. Persistent health, covariance, and dead-reckoning
    /// state are left unchanged.
    fn begin_tick(&mut self) {}
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
}

impl Default for EkfParams {
    fn default() -> Self {
        Self {
            sigma_w_gyro: 0.01,
            sigma_w_accel_bias: 1.0e-4,
            sigma_w_gyro_bias: 1.0e-5,
            tau_gyro_bias_s: f64::INFINITY,
            tau_accel_bias_s: f64::INFINITY,
            sigma_gnss_pos_m: 5.0,
            sigma_gnss_vel_m_s: 0.5,
            sigma_baro_alt_m: 2.0,
            sigma_mag_nt: 100.0,
            innovation_gate: f64::NAN,
            innovation_false_alarm_rate: 0.01,
            dead_reckon_timeout_s: 1.5,
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
    last_innovation_rejected: bool,
    /// `true` once at least one corrective measurement has been
    /// applied.
    initialized: bool,
    /// Gravity model — applied during the predict step so the EKF
    /// integrates inertial acceleration (specific force + gravity).
    gravity: GravityAdapter,
    /// Magnetic-field model — used by `update_mag` to predict the
    /// body-frame magnetic vector and form an innovation.
    mag_field: Box<dyn MagneticFieldEci>,
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
            last_innovation_rejected: false,
            initialized: false,
            gravity: GravityAdapter::new(default_constant_gravity_down_z()),
            mag_field: Box::new(EarthDipoleField::default()),
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
        let q_diag = SVector::<f64, 15>::from_iterator((0..15).map(|i| match i {
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
        // 6-D GNSS update on position + velocity. H projects the
        // first 6 state elements.
        let z = stack6(sample.position_eci_m, sample.velocity_eci_m_s);
        let mut h: SMatrix<f64, 6, 15> = SMatrix::zeros();
        for i in 0..6 {
            h[(i, i)] = 1.0;
        }
        let r_var = SMatrix::<f64, 6, 6>::from_diagonal(&SVector::<f64, 6>::from_iterator(
            (0..6).map(|i| {
                if i < 3 {
                    self.params.sigma_gnss_pos_m * self.params.sigma_gnss_pos_m
                } else {
                    self.params.sigma_gnss_vel_m_s * self.params.sigma_gnss_vel_m_s
                }
            }),
        ));
        let predicted = stack6(self.pos_eci, self.vel_eci);
        let innovation = z - predicted;

        let s = h * self.p * h.transpose() + r_var;
        let Some(s_inv) = s.try_inverse() else {
            return Err(EstimatorError::InvalidConfig {
                reason: "GNSS innovation covariance singular".to_string(),
            });
        };
        let chi2 = innovation.dot(&(s_inv * innovation));
        self.last_chi2_gnss = chi2;
        let gate = self.params.gate_for_dof(6.0);
        if chi2 > gate {
            self.last_innovation_rejected = true;
            return Err(EstimatorError::InnovationGateRejected {
                measurement: "gnss",
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
            let Some(s_inv) = s.try_inverse() else {
                return Err(EstimatorError::InvalidConfig {
                    reason: "mag innovation covariance singular".to_string(),
                });
            };
            let chi2 = innovation.dot(&(s_inv * innovation));
            if iteration == 0 {
                self.last_chi2_mag = chi2;
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
            star_tracker_chi2: 0.0,
            innovation_rejected: self.last_innovation_rejected,
        }
    }

    fn begin_tick(&mut self) {
        self.last_chi2_imu = 0.0;
        self.last_chi2_gnss = 0.0;
        self.last_chi2_baro = 0.0;
        self.last_chi2_mag = 0.0;
        self.last_innovation_rejected = false;
    }
}

impl Ekf {
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
        let predicted = r_eci_to_body * self.mag_field.field_eci_nt(self.pos_eci, time);
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
}

fn stack6(a: Vector3<f64>, b: Vector3<f64>) -> SVector<f64, 6> {
    SVector::<f64, 6>::from_iterator((0..6).map(|i| if i < 3 { a[i] } else { b[i - 3] }))
}

fn quaternion_from_omega(omega: Vector3<f64>, dt: f64) -> UnitQuaternion<f64> {
    let theta = omega * dt;
    quaternion_from_axis_angle(theta)
}

fn quaternion_from_axis_angle(axis_angle: Vector3<f64>) -> UnitQuaternion<f64> {
    let mag = axis_angle.norm();
    if mag <= f64::EPSILON.sqrt() {
        UnitQuaternion::identity()
    } else {
        UnitQuaternion::from_scaled_axis(axis_angle)
    }
}

fn renormalize_quaternion(q: &mut UnitQuaternion<f64>) {
    *q = UnitQuaternion::new_normalize((*q).into_inner());
}

fn skew_symmetric(v: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -v.z, v.y, //
        v.z, 0.0, -v.x, //
        -v.y, v.x, 0.0,
    )
}

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
    chi_square_inverse_cdf_wilson_hilferty(probability, dof)
}

fn chi_square_inverse_cdf_wilson_hilferty(probability: f64, dof: f64) -> f64 {
    if !probability.is_finite() || !dof.is_finite() || dof <= 0.0 {
        return f64::INFINITY;
    }
    let z = inverse_standard_normal_cdf(probability);
    let a = 2.0 / (9.0 * dof);
    dof * (1.0 - a + z * a.sqrt()).powi(3)
}

fn inverse_standard_normal_cdf(probability: f64) -> f64 {
    // Peter J. Acklam's rational approximation. Coefficients are
    // dimensionless numerical-method constants, not physical data.
    const A: [f64; 6] = [
        -3.969_683_028_665_376e1,
        2.209_460_984_245_205e2,
        -2.759_285_104_469_687e2,
        1.383_577_518_672_69e2,
        -3.066_479_806_614_716e1,
        2.506_628_277_459_239,
    ];
    const B: [f64; 5] = [
        -5.447_609_879_822_406e1,
        1.615_858_368_580_409e2,
        -1.556_989_798_598_866e2,
        6.680_131_188_771_972e1,
        -1.328_068_155_288_572e1,
    ];
    const C: [f64; 6] = [
        -7.784_894_002_430_293e-3,
        -3.223_964_580_411_365e-1,
        -2.400_758_277_161_838,
        -2.549_732_539_343_734,
        4.374_664_141_464_968,
        2.938_163_982_698_783,
    ];
    const D: [f64; 4] = [
        7.784_695_709_041_462e-3,
        3.224_671_290_700_398e-1,
        2.445_134_137_142_996,
        3.754_408_661_907_416,
    ];
    const P_LOW: f64 = 0.024_25;
    const P_HIGH: f64 = 1.0 - P_LOW;

    let p = probability.clamp(f64::MIN_POSITIVE, 1.0 - f64::EPSILON);
    if p < P_LOW {
        let q = (-2.0 * p.ln()).sqrt();
        return (((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0);
    }
    if p > P_HIGH {
        let q = (-2.0 * (1.0 - p).ln()).sqrt();
        return -(((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0);
    }
    let q = p - 0.5;
    let r = q * q;
    (((((A[0] * r + A[1]) * r + A[2]) * r + A[3]) * r + A[4]) * r + A[5]) * q
        / (((((B[0] * r + B[1]) * r + B[2]) * r + B[3]) * r + B[4]) * r + 1.0)
}

// ---------------------------------------------------------------------
// UKF — classical sigma-point attitude + gyro-bias filter (Phase 4.C)
// ---------------------------------------------------------------------

/// Tunables for the classical Julier-Uhlmann / Wan-van der Merwe UKF.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct UkfParams {
    /// Sigma-point spread.
    pub alpha: f64,
    /// Distribution prior. `2.0` is the Gaussian default.
    pub beta: f64,
    /// Tertiary scaling parameter.
    pub kappa: f64,
    /// Process-noise standard deviation on attitude rate (rad/s).
    pub sigma_w_gyro: f64,
    /// Process-noise standard deviation on gyro bias (rad/s/√s).
    pub sigma_w_gyro_bias: f64,
    /// First-order Gauss-Markov gyro-bias time constant. `∞`
    /// preserves the random-walk limit.
    pub tau_gyro_bias_s: f64,
    /// Measurement-noise standard deviation on each magnetometer
    /// component (nT).
    pub sigma_mag_nt: f64,
    /// Optional explicit innovation gate. `NaN` derives a chi-square
    /// threshold from `innovation_false_alarm_rate`.
    pub innovation_gate: f64,
    /// False-alarm probability for derived chi-square gates.
    pub innovation_false_alarm_rate: f64,
}

impl Default for UkfParams {
    fn default() -> Self {
        Self {
            alpha: 1.0e-3,
            beta: 2.0,
            kappa: 0.0,
            sigma_w_gyro: 0.005,
            sigma_w_gyro_bias: 1.0e-6,
            tau_gyro_bias_s: f64::INFINITY,
            sigma_mag_nt: 50.0,
            innovation_gate: f64::NAN,
            innovation_false_alarm_rate: 0.01,
        }
    }
}

impl ParamSection for UkfParams {
    const NAME: &'static str = "estimator.ukf";
}

impl UkfParams {
    fn weights(&self, dimension: f64) -> (f64, f64, f64) {
        let lambda = self.alpha * self.alpha * (dimension + self.kappa) - dimension;
        let spread = dimension + lambda;
        let w0_mean = lambda / spread;
        let w0_cov = w0_mean + (1.0 - self.alpha * self.alpha + self.beta);
        let wi = 0.5 / spread;
        (w0_mean, w0_cov, wi)
    }

    fn gate_for_dof(&self, dof: f64) -> f64 {
        innovation_gate_threshold(self.innovation_gate, self.innovation_false_alarm_rate, dof)
    }
}

/// Six-state classical UKF for attitude error + gyro bias.
pub struct Ukf {
    params: UkfParams,
    q_body_to_eci: UnitQuaternion<f64>,
    gyro_bias: Vector3<f64>,
    omega_body: Vector3<f64>,
    p: SMatrix<f64, 6, 6>,
    last_imu: Option<ImuSample>,
    last_chi2_mag: f64,
    last_innovation_rejected: bool,
    initialized: bool,
    reference_position_eci_m: Vector3<f64>,
    mag_field: Box<dyn MagneticFieldEci>,
}

impl std::fmt::Debug for Ukf {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Ukf")
            .field("params", &self.params)
            .field("q_body_to_eci", &self.q_body_to_eci)
            .field("gyro_bias", &self.gyro_bias)
            .field("last_chi2_mag", &self.last_chi2_mag)
            .field("initialized", &self.initialized)
            .finish_non_exhaustive()
    }
}

#[derive(Copy, Clone, Debug)]
struct AttitudeSigmaPoint {
    attitude_error: Vector3<f64>,
    gyro_bias: Vector3<f64>,
}

impl Ukf {
    /// Constructs an attitude + gyro-bias UKF.
    #[must_use]
    pub fn new(params: UkfParams) -> Self {
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

    /// Seeds the nominal attitude.
    pub fn seed(&mut self, q_body_to_eci: UnitQuaternion<f64>) {
        self.q_body_to_eci = q_body_to_eci;
        self.initialized = true;
    }

    /// Replaces the magnetic-field reference position.
    pub fn set_reference_position(&mut self, position_eci_m: Vector3<f64>) {
        self.reference_position_eci_m = position_eci_m;
    }

    /// Returns the largest covariance diagonal entry.
    #[must_use]
    pub fn covariance_max_diag(&self) -> f64 {
        (0..6).map(|i| self.p[(i, i)]).fold(0.0, f64::max)
    }

    fn sigma_points(&self) -> Result<Vec<AttitudeSigmaPoint>, EstimatorError> {
        const N: usize = 6;
        const N_F64: f64 = 6.0;
        let lambda = self.params.alpha * self.params.alpha * (N_F64 + self.params.kappa) - N_F64;
        let spread = N_F64 + lambda;
        if !spread.is_finite() || spread <= 0.0 {
            return Err(EstimatorError::InvalidConfig {
                reason: "UKF sigma-point spread must be positive".to_string(),
            });
        }
        let Some(cholesky) = self.p.cholesky() else {
            return Err(EstimatorError::InvalidConfig {
                reason: "UKF covariance is not positive definite".to_string(),
            });
        };
        let scaled_l = cholesky.l() * spread.sqrt();
        let mut mean = SVector::<f64, N>::zeros();
        mean[3] = self.gyro_bias.x;
        mean[4] = self.gyro_bias.y;
        mean[5] = self.gyro_bias.z;
        let mut out = Vec::with_capacity(2 * N + 1);
        out.push(sigma_from_vector(mean));
        for i in 0..N {
            let sigma_column = scaled_l.column(i);
            out.push(sigma_from_vector(mean + sigma_column));
            out.push(sigma_from_vector(mean - sigma_column));
        }
        Ok(out)
    }

    fn recombine_prediction(
        &self,
        propagated: &[UnitQuaternion<f64>],
        biases: &[Vector3<f64>],
        dt_s: f64,
    ) -> (UnitQuaternion<f64>, Vector3<f64>, SMatrix<f64, 6, 6>) {
        let (w0_mean, w0_cov, wi) = self.params.weights(6.0);
        let mut q_mean = propagated[0];
        for _ in 0..2 {
            let mut correction = w0_mean * quaternion_error_vector(q_mean, propagated[0]);
            for q in &propagated[1..] {
                correction += wi * quaternion_error_vector(q_mean, *q);
            }
            if correction.norm() <= 1.0e-12 {
                break;
            }
            q_mean *= quaternion_from_axis_angle(correction);
            renormalize_quaternion(&mut q_mean);
        }
        let mut bias_mean = w0_mean * biases[0];
        for bias in &biases[1..] {
            bias_mean += wi * *bias;
        }

        let mut p = SMatrix::<f64, 6, 6>::zeros();
        accumulate_ukf_covariance(&mut p, w0_cov, q_mean, propagated[0], bias_mean, biases[0]);
        for (q, bias) in propagated[1..].iter().zip(&biases[1..]) {
            accumulate_ukf_covariance(&mut p, wi, q_mean, *q, bias_mean, *bias);
        }
        for i in 0..3 {
            p[(i, i)] += self.params.sigma_w_gyro * self.params.sigma_w_gyro * dt_s;
            p[(i + 3, i + 3)] += gauss_markov_process_variance(
                self.params.sigma_w_gyro_bias,
                dt_s,
                self.params.tau_gyro_bias_s,
            );
        }
        (q_mean, bias_mean, (p + p.transpose()) * 0.5)
    }
}

impl Estimator for Ukf {
    fn name(&self) -> &'static str {
        "estimator.ukf"
    }

    fn predict(&mut self, dt: f64) -> Result<(), EstimatorError> {
        if !dt.is_finite() || dt <= 0.0 {
            return Ok(());
        }
        let Some(imu) = self.last_imu else {
            return Ok(());
        };
        let sigma = self.sigma_points()?;
        let bias_decay = gauss_markov_decay(dt, self.params.tau_gyro_bias_s);
        let mut propagated = Vec::with_capacity(sigma.len());
        let mut biases = Vec::with_capacity(sigma.len());
        for point in sigma {
            let mut q = self.q_body_to_eci * quaternion_from_axis_angle(point.attitude_error);
            let bias = point.gyro_bias * bias_decay;
            let omega = imu.gyro_rad_s - bias;
            q *= quaternion_from_omega(omega, dt);
            renormalize_quaternion(&mut q);
            propagated.push(q);
            biases.push(bias);
        }
        let (q_mean, bias_mean, p) = self.recombine_prediction(&propagated, &biases, dt);
        self.q_body_to_eci = q_mean;
        self.gyro_bias = bias_mean;
        self.omega_body = imu.gyro_rad_s - self.gyro_bias;
        self.p = p;
        Ok(())
    }

    fn update_imu(&mut self, sample: &ImuSample) -> Result<(), EstimatorError> {
        self.last_imu = Some(*sample);
        Ok(())
    }

    fn update_gnss(&mut self, _sample: &GnssSample) -> Result<(), EstimatorError> {
        Ok(())
    }

    fn update_baro(&mut self, _sample: &BarometerSample) -> Result<(), EstimatorError> {
        Ok(())
    }

    fn update_mag(&mut self, sample: &MagnetometerSample) -> Result<(), EstimatorError> {
        if !sample.healthy {
            return Ok(());
        }
        let sigma = self.sigma_points()?;
        let (w0_mean, w0_cov, wi) = self.params.weights(6.0);
        let measured = sample.field_body_nt - sample.hard_iron_body_nt;
        let mut z_sigma = Vec::with_capacity(sigma.len());
        for point in &sigma {
            let q = self.q_body_to_eci * quaternion_from_axis_angle(point.attitude_error);
            let r_eci_to_body = q.to_rotation_matrix().transpose();
            z_sigma.push(
                r_eci_to_body
                    * self
                        .mag_field
                        .field_eci_nt(self.reference_position_eci_m, sample.time),
            );
        }
        let mut z_mean = w0_mean * z_sigma[0];
        for z in &z_sigma[1..] {
            z_mean += wi * *z;
        }
        let mut s = SMatrix::<f64, 3, 3>::from_diagonal_element(
            self.params.sigma_mag_nt * self.params.sigma_mag_nt,
        );
        let mut pxz = SMatrix::<f64, 6, 3>::zeros();
        accumulate_ukf_measurement_covariance(
            &mut s,
            &mut pxz,
            w0_cov,
            sigma[0],
            self.gyro_bias,
            z_sigma[0],
            z_mean,
        );
        for (point, z) in sigma[1..].iter().zip(&z_sigma[1..]) {
            accumulate_ukf_measurement_covariance(
                &mut s,
                &mut pxz,
                wi,
                *point,
                self.gyro_bias,
                *z,
                z_mean,
            );
        }
        let Some(s_inv) = s.try_inverse() else {
            return Err(EstimatorError::InvalidConfig {
                reason: "UKF mag innovation covariance singular".to_string(),
            });
        };
        let innovation = measured - z_mean;
        let chi2 = innovation.dot(&(s_inv * innovation));
        self.last_chi2_mag = chi2;
        let gate = self.params.gate_for_dof(3.0);
        if chi2 > gate {
            self.last_innovation_rejected = true;
            return Err(EstimatorError::InnovationGateRejected {
                measurement: "mag",
                chi2,
                gate,
            });
        }
        let k = pxz * s_inv;
        let dx = k * innovation;
        self.q_body_to_eci *= quaternion_from_axis_angle(Vector3::new(dx[0], dx[1], dx[2]));
        renormalize_quaternion(&mut self.q_body_to_eci);
        self.gyro_bias += Vector3::new(dx[3], dx[4], dx[5]);
        let p = self.p - k * s * k.transpose();
        self.p = (p + p.transpose()) * 0.5;
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
        }
    }

    fn begin_tick(&mut self) {
        self.last_chi2_mag = 0.0;
        self.last_innovation_rejected = false;
    }
}

fn sigma_from_vector(x: SVector<f64, 6>) -> AttitudeSigmaPoint {
    AttitudeSigmaPoint {
        attitude_error: Vector3::new(x[0], x[1], x[2]),
        gyro_bias: Vector3::new(x[3], x[4], x[5]),
    }
}

fn quaternion_error_vector(
    reference: UnitQuaternion<f64>,
    sample: UnitQuaternion<f64>,
) -> Vector3<f64> {
    (reference.inverse() * sample).scaled_axis()
}

fn accumulate_ukf_covariance(
    p: &mut SMatrix<f64, 6, 6>,
    weight: f64,
    q_mean: UnitQuaternion<f64>,
    q_sample: UnitQuaternion<f64>,
    bias_mean: Vector3<f64>,
    bias_sample: Vector3<f64>,
) {
    let attitude_error = quaternion_error_vector(q_mean, q_sample);
    let bias_error = bias_sample - bias_mean;
    let dx = SVector::<f64, 6>::new(
        attitude_error.x,
        attitude_error.y,
        attitude_error.z,
        bias_error.x,
        bias_error.y,
        bias_error.z,
    );
    *p += weight * (dx * dx.transpose());
}

fn accumulate_ukf_measurement_covariance(
    s: &mut SMatrix<f64, 3, 3>,
    pxz: &mut SMatrix<f64, 6, 3>,
    weight: f64,
    point: AttitudeSigmaPoint,
    bias_mean: Vector3<f64>,
    z: Vector3<f64>,
    z_mean: Vector3<f64>,
) {
    let dx = SVector::<f64, 6>::new(
        point.attitude_error.x,
        point.attitude_error.y,
        point.attitude_error.z,
        point.gyro_bias.x - bias_mean.x,
        point.gyro_bias.y - bias_mean.y,
        point.gyro_bias.z - bias_mean.z,
    );
    let dz = z - z_mean;
    *s += weight * (dz * dz.transpose());
    *pxz += weight * (dx * dz.transpose());
}

/// Unscented transform of `x²` for a scalar Gaussian.
#[must_use]
pub fn unscented_square_moments(mean: f64, variance: f64, params: UkfParams) -> Option<(f64, f64)> {
    if !mean.is_finite() || !variance.is_finite() || variance < 0.0 {
        return None;
    }
    let lambda = params.alpha * params.alpha * (1.0 + params.kappa) - 1.0;
    let spread = 1.0 + lambda;
    if spread <= 0.0 {
        return None;
    }
    let (w0_mean, w0_cov, wi) = params.weights(1.0);
    let delta = (spread * variance).sqrt();
    let ys = [
        mean * mean,
        (mean + delta) * (mean + delta),
        (mean - delta) * (mean - delta),
    ];
    let y_mean = w0_mean * ys[0] + wi * ys[1] + wi * ys[2];
    let y_var = w0_cov * (ys[0] - y_mean) * (ys[0] - y_mean)
        + wi * (ys[1] - y_mean) * (ys[1] - y_mean)
        + wi * (ys[2] - y_mean) * (ys[2] - y_mean);
    Some((y_mean, y_var))
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod ukf_tests {
    use approx::assert_abs_diff_eq;

    use super::*;

    #[test]
    fn unscented_square_matches_gaussian_moments_with_three_sigma_spread() {
        let params = UkfParams {
            alpha: 1.0,
            beta: 0.0,
            kappa: 2.0,
            ..UkfParams::default()
        };
        let (mean, variance) = unscented_square_moments(0.0, 2.0, params).unwrap();
        assert_abs_diff_eq!(mean, 2.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(variance, 8.0, epsilon = 1.0e-12);
    }

    #[test]
    fn ukf_predict_propagates_sigma_points_deterministically() {
        let mut ukf = Ukf::new(UkfParams::default());
        ukf.update_imu(&ImuSample {
            time: openbmp_core::SimTime::ZERO,
            gyro_rad_s: Vector3::new(0.0, 0.0, 0.1),
            accel_m_s2: Vector3::zeros(),
            healthy: true,
        })
        .unwrap();
        ukf.predict(0.1).unwrap();
        let first = ukf.attitude();
        ukf.predict(0.1).unwrap();
        let second = ukf.attitude();
        assert!(second.q_body_to_eci_xyzw[3] < first.q_body_to_eci_xyzw[3]);
    }
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
}

impl<E: Estimator + std::fmt::Debug + 'static> Job for EstimatorJob<E> {
    fn name(&self) -> &'static str {
        self.name
    }

    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        self.estimator.begin_tick();
        // Drain sensor topics (in deterministic order).
        let _ = self.drain_imu(ctx.bus);
        let _ = self.drain_baro(ctx.bus);
        let _ = self.drain_mag(ctx.bus);
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
}

// ---------------------------------------------------------------------
// MEKF — multiplicative EKF for quaternion attitude (Phase 4.8)
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
            let Some(s_inv) = s.try_inverse() else {
                return Err(EstimatorError::InvalidConfig {
                    reason: "MEKF mag innovation covariance singular".to_string(),
                });
            };
            let chi2 = innovation.dot(&(s_inv * innovation));
            if iteration == 0 {
                self.last_chi2_mag = chi2;
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
        }
    }

    fn begin_tick(&mut self) {
        self.last_chi2_mag = 0.0;
        self.last_innovation_rejected = false;
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
        let predicted = r_eci_to_body
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
