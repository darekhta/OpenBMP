//! Synthetic GNSS receiver.
//!
//! Per ECI axis the measurement transformation is:
//!
//! ```text
//! pos_bias = OU.step(pos_bias_prev)        (correlated bias drift)
//! pos_meas = truth_position + pos_bias + N(0, σ_pos)
//! vel_meas = truth_velocity         + N(0, σ_vel)
//! ```
//!
//! The receiver noise model follows the IS-GPS-200 user-equivalent
//! range error budget at the receiver-output level — additive
//! Gaussian noise on position and velocity plus a slowly-drifting
//! Ornstein-Uhlenbeck position bias. **No satellite geometry, no
//! pseudorange, no ionosphere, no tropospheric model.** This is a
//! receiver-output noise model; the navigation-message processing
//! is out of scope.
//!
//! # Component-id layout
//!
//! 11 reserved component slots per GNSS sensor: 3 position-noise
//! axes, 3 velocity-noise axes, 3 position bias OU drives.
//! Slots 9 and 10 are reserved for the receiver clock bias / drift
//! random-walk drives so adding the deterministic clock model does not
//! shift the original component streams.
//!
//! ```text
//! channel        component_id
//! pos_noise_x    0
//! pos_noise_y    1
//! pos_noise_z    2
//! vel_noise_x    3
//! vel_noise_y    4
//! vel_noise_z    5
//! pos_bias_ou_x  6
//! pos_bias_ou_y  7
//! pos_bias_ou_z  8
//! clock_bias_rw  9
//! clock_drift_rw 10
//! ```
//!
//! # Determinism
//!
//! Pure `f64`; locked operand order on the per-axis sum
//! `truth + bias + noise`. Per-component RNG sub-streams via
//! `DeterministicRng::for_sensor_component(seed, step, sensor_id,
//! component_id)`. Truth-bypass mode (every `σ = 0` and every OU
//! `σ = 0`) returns the truth bit-equal.

use nalgebra::Vector3;
use openbmp_core::{DeterministicRng, Duration, SensorId, SimTime, StepIndex};

use crate::error::SensorError;
use crate::noise::{BoxMullerGaussian, OrnsteinUhlenbeck};
use crate::sensor::{SensorMeasurement, SensorTruth, SyntheticSensor, require_truth_finite};

const SUB_POS_NOISE_X: u32 = 0;
const SUB_POS_NOISE_Y: u32 = 1;
const SUB_POS_NOISE_Z: u32 = 2;
const SUB_VEL_NOISE_X: u32 = 3;
const SUB_VEL_NOISE_Y: u32 = 4;
const SUB_VEL_NOISE_Z: u32 = 5;
const SUB_POS_BIAS_OU_X: u32 = 6;
const SUB_POS_BIAS_OU_Y: u32 = 7;
const SUB_POS_BIAS_OU_Z: u32 = 8;
const SUB_CLOCK_BIAS_RW: u32 = 9;
const SUB_CLOCK_DRIFT_RW: u32 = 10;
const SPEED_OF_LIGHT_M_S: f64 = 299_792_458.0;

const POS_NOISE_IDS: [u32; 3] = [SUB_POS_NOISE_X, SUB_POS_NOISE_Y, SUB_POS_NOISE_Z];
const VEL_NOISE_IDS: [u32; 3] = [SUB_VEL_NOISE_X, SUB_VEL_NOISE_Y, SUB_VEL_NOISE_Z];
const POS_BIAS_OU_IDS: [u32; 3] = [SUB_POS_BIAS_OU_X, SUB_POS_BIAS_OU_Y, SUB_POS_BIAS_OU_Z];

/// Per-axis isotropic noise budget for the synthetic GNSS receiver.
///
/// All values are in SI units (m, m/s, s, m/√s). For the OU bias
/// drift on the position channels the budget specifies the OU
/// mean-reversion rate `θ` (1/s) and the OU white-noise drive
/// strength `σ` (m/√s). The continuous-time stationary
/// distribution is `N(0, σ²/(2θ))`; users can derive a steady-
/// state std directly from those two values.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct GnssNoiseBudget {
    /// Per-axis Gaussian-noise stddev on reported ECI position (m).
    pub sigma_position_m: [f64; 3],
    /// Per-axis Gaussian-noise stddev on reported ECI velocity
    /// (m/s).
    pub sigma_velocity_m_s: [f64; 3],
    /// OU mean-reversion rate `θ` for the position-bias drift, per
    /// axis (1/s). Zero disables bias drift on that axis.
    pub position_bias_ou_theta_per_s: [f64; 3],
    /// OU white-noise drive strength `σ` for the position-bias
    /// drift, per axis (m/√s). Zero disables bias drift.
    pub position_bias_ou_sigma_m_sqrt_s: [f64; 3],
    /// Sensor sample period (s). GNSS receivers report
    /// at the kernel base tick — same convention as the IMU.
    pub dt_s: f64,
    /// Body-frame GNSS antenna phase-center offset from the vehicle
    /// reference point (m).
    pub mount_offset_body_m: Vector3<f64>,
    /// Initial receiver clock bias (s). In this receiver-output model
    /// it is projected onto the local radial direction as
    /// `c * bias`.
    pub clock_bias_s: f64,
    /// Initial receiver clock drift (s/s). In this receiver-output
    /// model it is projected onto the velocity direction as
    /// `c * drift`.
    pub clock_drift_s_per_s: f64,
    /// Clock-bias random-walk drive (s/√s).
    pub clock_bias_rw_sigma_s_sqrt_s: f64,
    /// Clock-drift random-walk drive ((s/s)/√s).
    pub clock_drift_rw_sigma_s_per_s_sqrt_s: f64,
    /// Fixed measurement latency (s). The synthetic adapter reports
    /// `truth.time - fixed_latency_s` as the capture timestamp and
    /// uses one-sample hold for positive latency.
    pub fixed_latency_s: f64,
}

impl GnssNoiseBudget {
    /// Construct after validating the parameters.
    ///
    /// # Errors
    ///
    /// Returns [`SensorError::InvalidParameter`] for negative sigma
    /// or theta, non-finite values, or non-positive `dt_s`.
    pub fn new(
        sigma_position_m: [f64; 3],
        sigma_velocity_m_s: [f64; 3],
        position_bias_ou_theta_per_s: [f64; 3],
        position_bias_ou_sigma_m_sqrt_s: [f64; 3],
        dt_s: f64,
    ) -> Result<Self, SensorError> {
        for v in sigma_position_m
            .iter()
            .chain(sigma_velocity_m_s.iter())
            .chain(position_bias_ou_theta_per_s.iter())
            .chain(position_bias_ou_sigma_m_sqrt_s.iter())
            .chain(std::iter::once(&dt_s))
        {
            if !v.is_finite() {
                return Err(SensorError::NonFinite {
                    reason: "GNSS noise budget contains a NaN or infinite value",
                });
            }
        }
        for v in sigma_position_m
            .iter()
            .chain(sigma_velocity_m_s.iter())
            .chain(position_bias_ou_theta_per_s.iter())
            .chain(position_bias_ou_sigma_m_sqrt_s.iter())
        {
            if *v < 0.0 {
                return Err(SensorError::InvalidParameter {
                    reason: "GNSS noise budget sigma / theta values must be non-negative",
                });
            }
        }
        if dt_s <= 0.0 {
            return Err(SensorError::InvalidParameter {
                reason: "GNSS dt_s must be strictly positive",
            });
        }
        Ok(Self {
            sigma_position_m,
            sigma_velocity_m_s,
            position_bias_ou_theta_per_s,
            position_bias_ou_sigma_m_sqrt_s,
            dt_s,
            mount_offset_body_m: Vector3::zeros(),
            clock_bias_s: 0.0,
            clock_drift_s_per_s: 0.0,
            clock_bias_rw_sigma_s_sqrt_s: 0.0,
            clock_drift_rw_sigma_s_per_s_sqrt_s: 0.0,
            fixed_latency_s: 0.0,
        })
    }

    /// Return a copy of this budget with deterministic receiver
    /// geometry / timing terms enabled.
    ///
    /// # Errors
    ///
    /// Returns [`SensorError`] when any deterministic term is
    /// non-finite or when latency is negative.
    pub fn with_deterministic_errors(
        mut self,
        mount_offset_body_m: Vector3<f64>,
        clock_bias_s: f64,
        clock_drift_s_per_s: f64,
        fixed_latency_s: f64,
    ) -> Result<Self, SensorError> {
        require_vector_finite(&mount_offset_body_m)?;
        for value in [clock_bias_s, clock_drift_s_per_s, fixed_latency_s] {
            if !value.is_finite() {
                return Err(SensorError::NonFinite {
                    reason: "GNSS deterministic error parameter is non-finite",
                });
            }
        }
        if fixed_latency_s < 0.0 {
            return Err(SensorError::InvalidParameter {
                reason: "GNSS fixed latency must be non-negative",
            });
        }
        self.mount_offset_body_m = mount_offset_body_m;
        self.clock_bias_s = clock_bias_s;
        self.clock_drift_s_per_s = clock_drift_s_per_s;
        self.fixed_latency_s = fixed_latency_s;
        Ok(self)
    }

    /// Return a copy of this budget with receiver clock random-walk
    /// drives enabled.
    ///
    /// # Errors
    ///
    /// Returns [`SensorError`] when a drive is non-finite or negative.
    pub fn with_clock_random_walk(
        mut self,
        clock_bias_rw_sigma_s_sqrt_s: f64,
        clock_drift_rw_sigma_s_per_s_sqrt_s: f64,
    ) -> Result<Self, SensorError> {
        for value in [
            clock_bias_rw_sigma_s_sqrt_s,
            clock_drift_rw_sigma_s_per_s_sqrt_s,
        ] {
            if !value.is_finite() {
                return Err(SensorError::NonFinite {
                    reason: "GNSS clock random-walk parameter is non-finite",
                });
            }
            if value < 0.0 {
                return Err(SensorError::InvalidParameter {
                    reason: "GNSS clock random-walk sigma must be non-negative",
                });
            }
        }
        self.clock_bias_rw_sigma_s_sqrt_s = clock_bias_rw_sigma_s_sqrt_s;
        self.clock_drift_rw_sigma_s_per_s_sqrt_s = clock_drift_rw_sigma_s_per_s_sqrt_s;
        Ok(self)
    }
}

/// Synthetic GNSS receiver.
///
/// Holds the noise budget plus per-axis OU bias state. Each axis
/// gets its own optional `OrnsteinUhlenbeck` primitive; `None`
/// disables the bias-drift channel on that axis (used when both
/// `θ` and `σ` are zero).
#[derive(Clone, Debug)]
pub struct SyntheticGnss {
    sensor_id: SensorId,
    budget: GnssNoiseBudget,
    position_bias_state_m: [f64; 3],
    position_bias_ou: [Option<OrnsteinUhlenbeck>; 3],
    clock_bias_state_s: f64,
    clock_drift_state_s_per_s: f64,
    latency_truth: Option<SensorTruth>,
    gauss: BoxMullerGaussian,
}

impl SyntheticGnss {
    /// Construct from a sensor id and a fully-validated noise
    /// budget.
    ///
    /// # Errors
    ///
    /// Returns [`SensorError::InvalidParameter`] /
    /// [`SensorError::NonFinite`] when the OU primitive rejects the
    /// supplied per-axis parameters.
    pub fn new(sensor_id: SensorId, budget: GnssNoiseBudget) -> Result<Self, SensorError> {
        let clock_bias_state_s = budget.clock_bias_s;
        let clock_drift_state_s_per_s = budget.clock_drift_s_per_s;
        let position_bias_ou = [
            Self::build_ou(
                budget.position_bias_ou_theta_per_s[0],
                budget.position_bias_ou_sigma_m_sqrt_s[0],
                budget.dt_s,
            )?,
            Self::build_ou(
                budget.position_bias_ou_theta_per_s[1],
                budget.position_bias_ou_sigma_m_sqrt_s[1],
                budget.dt_s,
            )?,
            Self::build_ou(
                budget.position_bias_ou_theta_per_s[2],
                budget.position_bias_ou_sigma_m_sqrt_s[2],
                budget.dt_s,
            )?,
        ];
        Ok(Self {
            sensor_id,
            budget,
            position_bias_state_m: [0.0; 3],
            position_bias_ou,
            clock_bias_state_s,
            clock_drift_state_s_per_s,
            latency_truth: None,
            gauss: BoxMullerGaussian::new(),
        })
    }

    fn build_ou(
        theta: f64,
        sigma: f64,
        dt_s: f64,
    ) -> Result<Option<OrnsteinUhlenbeck>, SensorError> {
        // OU disabled on this axis when both θ and σ are zero. The
        // OU primitive itself rejects θ ≤ 0; this wrapper lets the
        // budget represent "no bias drift" cleanly.
        if theta == 0.0 && sigma == 0.0 {
            return Ok(None);
        }
        OrnsteinUhlenbeck::new(theta, sigma, dt_s).map(Some)
    }

    /// Read-only access to the budget.
    #[must_use]
    pub const fn budget(&self) -> &GnssNoiseBudget {
        &self.budget
    }

    /// Current per-axis OU position-bias state (m).
    #[must_use]
    pub const fn position_bias_state_m(&self) -> &[f64; 3] {
        &self.position_bias_state_m
    }

    /// Current receiver clock bias state (s).
    #[must_use]
    pub const fn clock_bias_state_s(&self) -> f64 {
        self.clock_bias_state_s
    }

    /// Current receiver clock drift state (s/s).
    #[must_use]
    pub const fn clock_drift_state_s_per_s(&self) -> f64 {
        self.clock_drift_state_s_per_s
    }
}

impl SyntheticSensor for SyntheticGnss {
    fn sensor_id(&self) -> SensorId {
        self.sensor_id
    }

    fn measure(
        &mut self,
        truth: &SensorTruth,
        step: StepIndex,
        scenario_seed: u64,
    ) -> Result<SensorMeasurement, SensorError> {
        require_truth_finite(truth)?;
        let measurement_truth = if self.budget.fixed_latency_s > 0.0 {
            let delayed = self.latency_truth.unwrap_or(*truth);
            self.latency_truth = Some(*truth);
            delayed
        } else {
            *truth
        };

        // Snapshot OU primitives so the per-axis loop doesn't borrow
        // self mutably twice.
        let ou_primitives = self.position_bias_ou;
        let sigma_pos = self.budget.sigma_position_m;
        let sigma_vel = self.budget.sigma_velocity_m_s;
        self.step_clock(step, scenario_seed)?;
        let body_to_eci = measurement_truth.attitude_eci_to_body.inverse();
        let mount_offset_eci_m = body_to_eci * self.budget.mount_offset_body_m;
        let mount_velocity_eci_m_s = body_to_eci
            * measurement_truth
                .angular_velocity_body_rad_s
                .cross(&self.budget.mount_offset_body_m);
        let radial_direction_eci = unit_or_zero(measurement_truth.position_eci.vector);
        let velocity_direction_eci = unit_or_zero(measurement_truth.velocity_eci.vector);
        let clock_position_bias_eci_m =
            radial_direction_eci * (SPEED_OF_LIGHT_M_S * self.clock_bias_state_s);
        let clock_velocity_bias_eci_m_s =
            velocity_direction_eci * (SPEED_OF_LIGHT_M_S * self.clock_drift_state_s_per_s);
        let truth_position =
            measurement_truth.position_eci.vector + mount_offset_eci_m + clock_position_bias_eci_m;
        let truth_velocity = measurement_truth.velocity_eci.vector
            + mount_velocity_eci_m_s
            + clock_velocity_bias_eci_m_s;
        let mut reported_position = [0.0_f64; 3];
        let mut reported_velocity = [0.0_f64; 3];

        for axis in 0..3 {
            // Step the OU bias on this axis. When the axis was
            // constructed with both θ = 0 and σ = 0 the OU is
            // disabled — the bias state stays at its initial value
            // (0) and the per-component RNG slot is left untouched.
            if let Some(ou) = ou_primitives[axis].as_ref() {
                let mut rng_bias = DeterministicRng::for_sensor_component(
                    scenario_seed,
                    step,
                    self.sensor_id,
                    POS_BIAS_OU_IDS[axis],
                );
                self.position_bias_state_m[axis] =
                    ou.step(self.position_bias_state_m[axis], &mut rng_bias, &self.gauss);
            }

            // Position noise.
            let mut rng_pos = DeterministicRng::for_sensor_component(
                scenario_seed,
                step,
                self.sensor_id,
                POS_NOISE_IDS[axis],
            );
            let pos_noise = self.gauss.sample(&mut rng_pos, 0.0, sigma_pos[axis])?;
            // Locked operand order: truth + bias + noise.
            let truth_axis = truth_position[axis];
            let pos_axis = truth_axis + self.position_bias_state_m[axis] + pos_noise;
            if !pos_axis.is_finite() {
                return Err(SensorError::NonFinite {
                    reason: "GNSS reported position is non-finite",
                });
            }
            reported_position[axis] = pos_axis;

            // Velocity noise (no bias drift on velocity in 3.10).
            let mut rng_vel = DeterministicRng::for_sensor_component(
                scenario_seed,
                step,
                self.sensor_id,
                VEL_NOISE_IDS[axis],
            );
            let vel_noise = self.gauss.sample(&mut rng_vel, 0.0, sigma_vel[axis])?;
            let truth_axis_v = truth_velocity[axis];
            let vel_axis = truth_axis_v + vel_noise;
            if !vel_axis.is_finite() {
                return Err(SensorError::NonFinite {
                    reason: "GNSS reported velocity is non-finite",
                });
            }
            reported_velocity[axis] = vel_axis;
        }

        Ok(SensorMeasurement::Gnss {
            position_eci_m: Vector3::new(
                reported_position[0],
                reported_position[1],
                reported_position[2],
            ),
            velocity_eci_m_s: Vector3::new(
                reported_velocity[0],
                reported_velocity[1],
                reported_velocity[2],
            ),
            position_bias_eci_m: Vector3::new(
                self.position_bias_state_m[0] + clock_position_bias_eci_m.x,
                self.position_bias_state_m[1] + clock_position_bias_eci_m.y,
                self.position_bias_state_m[2] + clock_position_bias_eci_m.z,
            ),
        })
    }

    fn capture_time(&self, truth_time: SimTime) -> SimTime {
        if self.budget.fixed_latency_s <= 0.0 {
            return truth_time;
        }
        let delayed_s = (truth_time - Duration::from_seconds(self.budget.fixed_latency_s))
            .as_seconds()
            .max(0.0);
        SimTime::from_seconds(delayed_s)
    }
}

impl SyntheticGnss {
    fn step_clock(&mut self, step: StepIndex, scenario_seed: u64) -> Result<(), SensorError> {
        if self.budget.clock_drift_rw_sigma_s_per_s_sqrt_s > 0.0 {
            let mut rng = DeterministicRng::for_sensor_component(
                scenario_seed,
                step,
                self.sensor_id,
                SUB_CLOCK_DRIFT_RW,
            );
            let drive = self.gauss.sample(
                &mut rng,
                0.0,
                self.budget.clock_drift_rw_sigma_s_per_s_sqrt_s * self.budget.dt_s.sqrt(),
            )?;
            self.clock_drift_state_s_per_s += drive;
        }
        self.clock_bias_state_s += self.clock_drift_state_s_per_s * self.budget.dt_s;
        if self.budget.clock_bias_rw_sigma_s_sqrt_s > 0.0 {
            let mut rng = DeterministicRng::for_sensor_component(
                scenario_seed,
                step,
                self.sensor_id,
                SUB_CLOCK_BIAS_RW,
            );
            let drive = self.gauss.sample(
                &mut rng,
                0.0,
                self.budget.clock_bias_rw_sigma_s_sqrt_s * self.budget.dt_s.sqrt(),
            )?;
            self.clock_bias_state_s += drive;
        }
        if !self.clock_bias_state_s.is_finite() || !self.clock_drift_state_s_per_s.is_finite() {
            return Err(SensorError::NonFinite {
                reason: "GNSS receiver clock state is non-finite",
            });
        }
        Ok(())
    }
}

fn require_vector_finite(value: &Vector3<f64>) -> Result<(), SensorError> {
    for component in [value.x, value.y, value.z] {
        if !component.is_finite() {
            return Err(SensorError::NonFinite {
                reason: "GNSS vector parameter is non-finite",
            });
        }
    }
    Ok(())
}

fn unit_or_zero(value: Vector3<f64>) -> Vector3<f64> {
    let norm = value.norm();
    if norm <= f64::EPSILON {
        Vector3::zeros()
    } else {
        value / norm
    }
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::panic,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]
mod tests {
    use super::*;
    use openbmp_core::{Position3, SimTime, Velocity3};

    use crate::sensor::{Sensor, SyntheticSensorAdapter};

    fn truth_at(time_s: f64) -> SensorTruth {
        SensorTruth {
            position_eci: Position3::new(7_000_000.0, 0.0, 0.0),
            velocity_eci: Velocity3::new(0.0, 7_500.0, 0.0),
            attitude_eci_to_body: nalgebra::UnitQuaternion::identity(),
            angular_velocity_body_rad_s: nalgebra::Vector3::zeros(),
            angular_acceleration_body_rad_s2: nalgebra::Vector3::zeros(),
            specific_force_body_m_s2: nalgebra::Vector3::zeros(),
            static_pressure_pa: 101_325.0,
            atmosphere_density_kg_m3: 1.225,
            speed_of_sound_m_s: 340.294,
            air_relative_velocity_body_m_s: nalgebra::Vector3::zeros(),
            altitude_geometric_m: 0.0,
            magnetic_field_body_nt: nalgebra::Vector3::zeros(),
            time: SimTime::from_seconds(time_s),
        }
    }

    fn zero_budget(dt_s: f64) -> GnssNoiseBudget {
        GnssNoiseBudget::new(
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            dt_s,
        )
        .unwrap()
    }

    fn nominal_budget(dt_s: f64) -> GnssNoiseBudget {
        GnssNoiseBudget::new(
            [3.0, 3.0, 3.0],                      // 3 m position stddev (IS-GPS-200 nominal)
            [0.1, 0.1, 0.1],                      // 0.1 m/s velocity stddev
            [1.0 / 60.0, 1.0 / 60.0, 1.0 / 60.0], // 60-second OU correlation
            [0.5, 0.5, 0.5],                      // OU drive strength
            dt_s,
        )
        .unwrap()
    }

    #[test]
    fn rejects_negative_sigma_position() {
        let res = GnssNoiseBudget::new(
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            0.01,
        );
        assert!(matches!(res, Err(SensorError::InvalidParameter { .. })));
    }

    #[test]
    fn rejects_negative_sigma_velocity() {
        let res = GnssNoiseBudget::new(
            [0.0, 0.0, 0.0],
            [-0.1, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            0.01,
        );
        assert!(matches!(res, Err(SensorError::InvalidParameter { .. })));
    }

    #[test]
    fn rejects_non_positive_dt() {
        let res = GnssNoiseBudget::new([0.0; 3], [0.0; 3], [0.0; 3], [0.0; 3], 0.0);
        assert!(matches!(res, Err(SensorError::InvalidParameter { .. })));
        let res = GnssNoiseBudget::new([0.0; 3], [0.0; 3], [0.0; 3], [0.0; 3], f64::NAN);
        assert!(matches!(res, Err(SensorError::NonFinite { .. })));
    }

    #[test]
    fn truth_bypass_returns_position_bit_equal_to_truth() {
        let id = SensorId::from_path("sensors.gnss.test");
        let mut sensor = SyntheticGnss::new(id, zero_budget(0.01)).unwrap();
        let truth = truth_at(0.0);
        let m = sensor
            .measure(&truth, StepIndex::new(0), 0xC0FF_EE00)
            .unwrap();
        let SensorMeasurement::Gnss {
            position_eci_m,
            velocity_eci_m_s,
            position_bias_eci_m,
        } = m
        else {
            panic!("expected Gnss measurement");
        };
        assert_eq!(
            position_eci_m.x.to_bits(),
            truth.position_eci.vector.x.to_bits()
        );
        assert_eq!(
            position_eci_m.y.to_bits(),
            truth.position_eci.vector.y.to_bits()
        );
        assert_eq!(
            position_eci_m.z.to_bits(),
            truth.position_eci.vector.z.to_bits()
        );
        assert_eq!(
            velocity_eci_m_s.x.to_bits(),
            truth.velocity_eci.vector.x.to_bits()
        );
        assert_eq!(
            velocity_eci_m_s.y.to_bits(),
            truth.velocity_eci.vector.y.to_bits()
        );
        assert_eq!(
            velocity_eci_m_s.z.to_bits(),
            truth.velocity_eci.vector.z.to_bits()
        );
        // Bias state stays at zero with σ_OU = 0.
        assert_eq!(position_bias_eci_m.x.to_bits(), 0.0_f64.to_bits());
        assert_eq!(position_bias_eci_m.y.to_bits(), 0.0_f64.to_bits());
        assert_eq!(position_bias_eci_m.z.to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn lever_arm_applies_antenna_position_and_velocity_offsets() {
        let id = SensorId::from_path("sensors.gnss.lever");
        let budget = zero_budget(0.1)
            .with_deterministic_errors(Vector3::new(1.5, 0.0, 0.0), 0.0, 0.0, 0.0)
            .unwrap();
        let mut sensor = SyntheticGnss::new(id, budget).unwrap();
        let mut truth = truth_at(0.0);
        truth.angular_velocity_body_rad_s = Vector3::new(0.0, 0.0, 2.0);

        let m = sensor.measure(&truth, StepIndex::new(0), 0).unwrap();
        let SensorMeasurement::Gnss {
            position_eci_m,
            velocity_eci_m_s,
            ..
        } = m
        else {
            panic!("expected Gnss measurement");
        };

        assert!((position_eci_m.x - (truth.position_eci.vector.x + 1.5)).abs() < 1.0e-12);
        assert!((position_eci_m.y - truth.position_eci.vector.y).abs() < 1.0e-12);
        assert!((velocity_eci_m_s.x - truth.velocity_eci.vector.x).abs() < 1.0e-12);
        assert!((velocity_eci_m_s.y - (truth.velocity_eci.vector.y + 3.0)).abs() < 1.0e-12);
    }

    #[test]
    fn clock_bias_and_drift_project_into_receiver_output() {
        let id = SensorId::from_path("sensors.gnss.clock");
        let budget = zero_budget(0.1)
            .with_deterministic_errors(Vector3::zeros(), 1.0e-9, 2.0e-10, 0.0)
            .unwrap();
        let mut sensor = SyntheticGnss::new(id, budget).unwrap();
        let truth = truth_at(0.0);

        let m = sensor.measure(&truth, StepIndex::new(0), 0).unwrap();
        let SensorMeasurement::Gnss {
            position_eci_m,
            velocity_eci_m_s,
            position_bias_eci_m,
        } = m
        else {
            panic!("expected Gnss measurement");
        };

        let expected_bias_m = SPEED_OF_LIGHT_M_S * (1.0e-9 + 2.0e-10 * 0.1);
        let expected_drift_m_s = SPEED_OF_LIGHT_M_S * 2.0e-10;
        assert!(
            (position_eci_m.x - (truth.position_eci.vector.x + expected_bias_m)).abs() < 1.0e-9
        );
        assert!((position_bias_eci_m.x - expected_bias_m).abs() < 1.0e-9);
        assert!(
            (velocity_eci_m_s.y - (truth.velocity_eci.vector.y + expected_drift_m_s)).abs()
                < 1.0e-9
        );
    }

    #[test]
    fn fixed_latency_reports_delayed_timestamp() {
        let id = SensorId::from_path("sensors.gnss.latency");
        let budget = zero_budget(0.1)
            .with_deterministic_errors(Vector3::zeros(), 0.0, 0.0, 0.25)
            .unwrap();
        let sensor = SyntheticGnss::new(id, budget).unwrap();
        let mut adapter = SyntheticSensorAdapter::new(sensor);

        adapter.prime(truth_at(1.0), StepIndex::new(0), 0);
        let first = adapter.read().unwrap();
        assert!((first.time.as_seconds() - 0.75).abs() < 1.0e-12);

        adapter.prime(truth_at(1.1), StepIndex::new(1), 0);
        let second = adapter.read().unwrap();
        assert!((second.time.as_seconds() - 0.85).abs() < 1.0e-12);
    }

    #[test]
    fn determinism_byte_stable_across_two_runs() {
        let id = SensorId::from_path("sensors.gnss.test");
        let mut sensor_a = SyntheticGnss::new(id, nominal_budget(0.01)).unwrap();
        let mut sensor_b = SyntheticGnss::new(id, nominal_budget(0.01)).unwrap();
        for step in 0..200 {
            let truth = truth_at(0.01 * f64::from(step));
            let m_a = sensor_a
                .measure(&truth, StepIndex::new(step as u64), 0xDEAD_BEEF)
                .unwrap();
            let m_b = sensor_b
                .measure(&truth, StepIndex::new(step as u64), 0xDEAD_BEEF)
                .unwrap();
            let SensorMeasurement::Gnss {
                position_eci_m: pa,
                velocity_eci_m_s: va,
                position_bias_eci_m: bias_a,
            } = m_a
            else {
                panic!("expected Gnss");
            };
            let SensorMeasurement::Gnss {
                position_eci_m: pb,
                velocity_eci_m_s: vb,
                position_bias_eci_m: bias_b,
            } = m_b
            else {
                panic!("expected Gnss");
            };
            assert_eq!(pa.x.to_bits(), pb.x.to_bits());
            assert_eq!(pa.y.to_bits(), pb.y.to_bits());
            assert_eq!(pa.z.to_bits(), pb.z.to_bits());
            assert_eq!(va.x.to_bits(), vb.x.to_bits());
            assert_eq!(va.y.to_bits(), vb.y.to_bits());
            assert_eq!(va.z.to_bits(), vb.z.to_bits());
            assert_eq!(bias_a.x.to_bits(), bias_b.x.to_bits());
            assert_eq!(bias_a.y.to_bits(), bias_b.y.to_bits());
            assert_eq!(bias_a.z.to_bits(), bias_b.z.to_bits());
        }
    }

    #[test]
    fn position_noise_empirical_std_within_5_percent_of_budget() {
        let id = SensorId::from_path("sensors.gnss.long_run");
        let sigma_pos = 3.0;
        let budget = GnssNoiseBudget::new(
            [sigma_pos, sigma_pos, sigma_pos],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0], // no OU bias
            [0.0, 0.0, 0.0],
            0.01,
        )
        .unwrap();
        let mut sensor = SyntheticGnss::new(id, budget).unwrap();
        // Fixed-truth (no orbital motion) so the deviation reflects
        // pure noise.
        let truth = truth_at(0.0);
        let n = 10_000_u64;
        let mut sum_x = 0.0;
        let mut sum_xx = 0.0;
        for step in 0..n {
            let m = sensor
                .measure(&truth, StepIndex::new(step), 0x424E_5353)
                .unwrap();
            let SensorMeasurement::Gnss {
                position_eci_m: p, ..
            } = m
            else {
                panic!("expected Gnss");
            };
            let dx = p.x - truth.position_eci.vector.x;
            sum_x += dx;
            sum_xx += dx * dx;
        }
        let n_f = n as f64;
        let mean = sum_x / n_f;
        let variance = sum_xx / n_f - mean * mean;
        let stddev = variance.sqrt();
        let rel_err = (stddev - sigma_pos).abs() / sigma_pos;
        assert!(
            rel_err < 0.05,
            "empirical std {stddev} differs from budget {sigma_pos} by {rel_err}",
        );
        // Long-run mean should be close to zero (within 0.1 σ).
        assert!(
            mean.abs() < 0.1 * sigma_pos,
            "long-run mean {mean} too far from zero (σ = {sigma_pos})",
        );
    }

    #[test]
    fn dropping_velocity_y_sigma_does_not_shift_position_x_stream() {
        // Per-component independence: shaking the velocity-y σ
        // shouldn't change the position-x noise stream because they
        // draw from different RNG sub-streams (component_id 4 vs 0).
        let id = SensorId::from_path("sensors.gnss.indep");
        let budget_a = GnssNoiseBudget::new(
            [3.0, 3.0, 3.0],
            [0.1, 0.1, 0.1],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            0.01,
        )
        .unwrap();
        let budget_b = GnssNoiseBudget::new(
            [3.0, 3.0, 3.0],
            [0.1, 999.0, 0.1], // arbitrary, only velocity-y σ changes
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            0.01,
        )
        .unwrap();
        let mut sensor_a = SyntheticGnss::new(id, budget_a).unwrap();
        let mut sensor_b = SyntheticGnss::new(id, budget_b).unwrap();
        let truth = truth_at(0.0);
        for step in 0..50 {
            let m_a = sensor_a
                .measure(&truth, StepIndex::new(step), 0x55AA_55AA)
                .unwrap();
            let m_b = sensor_b
                .measure(&truth, StepIndex::new(step), 0x55AA_55AA)
                .unwrap();
            let SensorMeasurement::Gnss {
                position_eci_m: pa, ..
            } = m_a
            else {
                panic!("expected Gnss");
            };
            let SensorMeasurement::Gnss {
                position_eci_m: pb, ..
            } = m_b
            else {
                panic!("expected Gnss");
            };
            assert_eq!(pa.x.to_bits(), pb.x.to_bits());
        }
    }
}
