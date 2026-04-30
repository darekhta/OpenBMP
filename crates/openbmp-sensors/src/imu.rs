//! Synthetic IMU with the IEEE 952 five-component noise model.
//!
//! Per axis, the measurement transformation is:
//!
//! ```text
//! scaled    = truth · (1 + scale_factor_ppm · 1e-6)
//! bias_OU   = OU.step(bias_OU_prev)        (correlated bias drift)
//! bias_RRW  = RRW.step(bias_RRW_prev)      (random-walk bias)
//! arw_noise = N(0, arw_per_sqrt_s / sqrt(dt))   (white-noise rate)
//! noisy     = scaled + bias_OU + bias_RRW + arw_noise
//! reported  = round(noisy / lsb) · lsb     (quantisation)
//! ```
//!
//! Axes are isotropic in Phase 2 (same noise budget for x, y, z).
//! Per-axis variation is a Phase-3 schema extension.
//!
//! # Component-id layout
//!
//! 30 reserved component slots per IMU (5 components × 6 axes). Only
//! the random-noise channels actually draw from the RNG; the
//! deterministic channels (scale factor, quantisation) leave their
//! reserved slots unused so a future sub-component cannot shift any
//! existing channel's RNG stream.
//!
//! ```text
//! axis          base_offset
//! gyro_x        0  (ARW=0, OU=1, RRW=2, scale=3 [reserved], quant=4 [reserved])
//! gyro_y        5  (ARW=5, OU=6, RRW=7, scale=8 [reserved], quant=9 [reserved])
//! gyro_z       10  …
//! accel_x      15
//! accel_y      20
//! accel_z      25
//! ```

use nalgebra::Vector3;
use openbmp_core::{DeterministicRng, SensorId, StepIndex};

use crate::error::SensorError;
use crate::noise::{BoxMullerGaussian, IntegratedWhiteNoise, OrnsteinUhlenbeck};
use crate::sensor::{
    Sensor, SensorMeasurement, SensorTruth, SyntheticSensor, require_truth_finite,
};

const COMPONENTS_PER_AXIS: u32 = 5;
const SUB_ARW: u32 = 0;
const SUB_BIAS_OU: u32 = 1;
const SUB_RRW: u32 = 2;

const AXIS_BASE_GYRO_X: u32 = 0;
const AXIS_BASE_GYRO_Y: u32 = COMPONENTS_PER_AXIS;
const AXIS_BASE_GYRO_Z: u32 = 2 * COMPONENTS_PER_AXIS;
const AXIS_BASE_ACCEL_X: u32 = 3 * COMPONENTS_PER_AXIS;
const AXIS_BASE_ACCEL_Y: u32 = 4 * COMPONENTS_PER_AXIS;
const AXIS_BASE_ACCEL_Z: u32 = 5 * COMPONENTS_PER_AXIS;

const GYRO_AXIS_BASES: [u32; 3] = [AXIS_BASE_GYRO_X, AXIS_BASE_GYRO_Y, AXIS_BASE_GYRO_Z];
const ACCEL_AXIS_BASES: [u32; 3] = [AXIS_BASE_ACCEL_X, AXIS_BASE_ACCEL_Y, AXIS_BASE_ACCEL_Z];

// ---------------------------------------------------------------------
// TriaxialNoiseBudget
// ---------------------------------------------------------------------

/// Isotropic noise budget for the three axes of a single IMU triad.
///
/// All values are in SI units. Phase-3 schema extension will allow
/// per-axis variation; Phase 2 applies the same budget to x, y, z.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct TriaxialNoiseBudget {
    /// Angular random walk (gyro) or velocity random walk (accel),
    /// in canonical units / √s. Per-sample rate-noise stddev is
    /// `arw_per_sqrt_s / sqrt(dt)`.
    pub arw_per_sqrt_s: f64,
    /// OU mean-reversion rate `θ = 1 / τ_BI` for bias instability,
    /// in 1/s.
    pub bias_ou_theta: f64,
    /// OU white-noise strength `σ` for bias instability.
    pub bias_ou_sigma: f64,
    /// Random-walk drive stddev for RRW (gyro) / ARW (accel),
    /// in canonical units / √s.
    pub rrw_sigma_per_sqrt_s: f64,
    /// Deterministic scale-factor error in ppm.
    pub scale_factor_ppm: f64,
    /// Quantisation LSB in canonical units (rad/s for gyro, m/s² for
    /// accel). Set to zero to disable quantisation.
    pub quantization_lsb: f64,
}

impl TriaxialNoiseBudget {
    /// Construct after validating the parameters.
    ///
    /// # Errors
    ///
    /// Returns [`SensorError::InvalidParameter`] for negative sigma /
    /// LSB / theta values; [`SensorError::NonFinite`] for any
    /// non-finite parameter.
    pub fn new(
        arw_per_sqrt_s: f64,
        bias_ou_theta: f64,
        bias_ou_sigma: f64,
        rrw_sigma_per_sqrt_s: f64,
        scale_factor_ppm: f64,
        quantization_lsb: f64,
    ) -> Result<Self, SensorError> {
        for v in [
            arw_per_sqrt_s,
            bias_ou_theta,
            bias_ou_sigma,
            rrw_sigma_per_sqrt_s,
            scale_factor_ppm,
            quantization_lsb,
        ] {
            if !v.is_finite() {
                return Err(SensorError::NonFinite {
                    reason: "IMU triaxial noise budget contains a NaN or infinite value",
                });
            }
        }
        if arw_per_sqrt_s < 0.0
            || bias_ou_sigma < 0.0
            || rrw_sigma_per_sqrt_s < 0.0
            || quantization_lsb < 0.0
        {
            return Err(SensorError::InvalidParameter {
                reason: "IMU triaxial noise budget has a negative noise / LSB value",
            });
        }
        if bias_ou_theta <= 0.0 {
            return Err(SensorError::InvalidParameter {
                reason: "IMU bias-instability theta must be > 0",
            });
        }
        Ok(Self {
            arw_per_sqrt_s,
            bias_ou_theta,
            bias_ou_sigma,
            rrw_sigma_per_sqrt_s,
            scale_factor_ppm,
            quantization_lsb,
        })
    }
}

// ---------------------------------------------------------------------
// ImuNoiseBudget
// ---------------------------------------------------------------------

/// Full noise budget for a single IMU (gyro + accel triads + dt).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ImuNoiseBudget {
    /// Gyro triaxial budget. `arw_per_sqrt_s` units rad/√s.
    pub gyro: TriaxialNoiseBudget,
    /// Accel triaxial budget. `arw_per_sqrt_s` units (m/s²)/√s
    /// (often called VRW = velocity random walk for accels).
    pub accel: TriaxialNoiseBudget,
    /// IMU sample interval (s).
    pub dt_s: f64,
}

impl ImuNoiseBudget {
    /// Construct after validating `dt_s > 0`.
    ///
    /// # Errors
    ///
    /// Returns [`SensorError::NonFinite`] / [`SensorError::InvalidParameter`]
    /// for non-finite or non-positive `dt_s`. Triaxial budgets are
    /// validated by [`TriaxialNoiseBudget::new`] at their own
    /// construction site.
    pub fn new(
        gyro: TriaxialNoiseBudget,
        accel: TriaxialNoiseBudget,
        dt_s: f64,
    ) -> Result<Self, SensorError> {
        if !dt_s.is_finite() {
            return Err(SensorError::NonFinite {
                reason: "IMU dt is NaN or infinite",
            });
        }
        if dt_s <= 0.0 {
            return Err(SensorError::InvalidParameter {
                reason: "IMU dt must be > 0",
            });
        }
        Ok(Self { gyro, accel, dt_s })
    }
}

// ---------------------------------------------------------------------
// SyntheticImu
// ---------------------------------------------------------------------

/// Phase-2 synthetic IMU using the IEEE 952 five-component model.
#[derive(Clone, Debug)]
pub struct SyntheticImu {
    sensor_id: SensorId,
    budget: ImuNoiseBudget,
    /// Per-axis OU bias state: gyro xyz then accel xyz.
    bias_ou: [f64; 6],
    /// Per-axis RRW bias state: gyro xyz then accel xyz.
    bias_rrw: [f64; 6],
    /// Cached OU and RRW objects per triad (gyro, accel).
    gyro_ou: OrnsteinUhlenbeck,
    accel_ou: OrnsteinUhlenbeck,
    gyro_rrw: IntegratedWhiteNoise,
    accel_rrw: IntegratedWhiteNoise,
    gauss: BoxMullerGaussian,
}

impl SyntheticImu {
    /// Construct from a sensor id and a fully-validated noise budget.
    ///
    /// # Errors
    ///
    /// Returns [`SensorError::InvalidParameter`] /
    /// [`SensorError::NonFinite`] when the OU or RRW primitives
    /// reject the supplied parameters.
    pub fn new(sensor_id: SensorId, budget: ImuNoiseBudget) -> Result<Self, SensorError> {
        let gyro_ou = OrnsteinUhlenbeck::new(
            budget.gyro.bias_ou_theta,
            budget.gyro.bias_ou_sigma,
            budget.dt_s,
        )?;
        let accel_ou = OrnsteinUhlenbeck::new(
            budget.accel.bias_ou_theta,
            budget.accel.bias_ou_sigma,
            budget.dt_s,
        )?;
        let gyro_rrw = IntegratedWhiteNoise::new(budget.gyro.rrw_sigma_per_sqrt_s, budget.dt_s)?;
        let accel_rrw = IntegratedWhiteNoise::new(budget.accel.rrw_sigma_per_sqrt_s, budget.dt_s)?;
        Ok(Self {
            sensor_id,
            budget,
            bias_ou: [0.0; 6],
            bias_rrw: [0.0; 6],
            gyro_ou,
            accel_ou,
            gyro_rrw,
            accel_rrw,
            gauss: BoxMullerGaussian::new(),
        })
    }

    /// Read-only access to the budget.
    #[must_use]
    pub const fn budget(&self) -> &ImuNoiseBudget {
        &self.budget
    }

    /// Current OU bias state per axis.
    #[must_use]
    pub const fn bias_ou_state(&self) -> &[f64; 6] {
        &self.bias_ou
    }

    /// Current RRW bias state per axis.
    #[must_use]
    pub const fn bias_rrw_state(&self) -> &[f64; 6] {
        &self.bias_rrw
    }

    #[allow(clippy::too_many_arguments, clippy::similar_names)]
    fn measure_axis(
        &mut self,
        truth_value: f64,
        budget: &TriaxialNoiseBudget,
        ou: &OrnsteinUhlenbeck,
        rrw: &IntegratedWhiteNoise,
        bias_ou_idx: usize,
        bias_rrw_idx: usize,
        scenario_seed: u64,
        step: StepIndex,
        axis_base: u32,
    ) -> Result<f64, SensorError> {
        // Per-component RNG sub-streams.
        let mut rng_arw = DeterministicRng::for_sensor_component(
            scenario_seed,
            step,
            self.sensor_id,
            axis_base + SUB_ARW,
        );
        let mut rng_ou = DeterministicRng::for_sensor_component(
            scenario_seed,
            step,
            self.sensor_id,
            axis_base + SUB_BIAS_OU,
        );
        let mut rng_rrw = DeterministicRng::for_sensor_component(
            scenario_seed,
            step,
            self.sensor_id,
            axis_base + SUB_RRW,
        );

        // Step OU and RRW. These are stateful so the next call
        // continues from where this one ended.
        self.bias_ou[bias_ou_idx] = ou.step(self.bias_ou[bias_ou_idx], &mut rng_ou, &self.gauss);
        self.bias_rrw[bias_rrw_idx] =
            rrw.step(self.bias_rrw[bias_rrw_idx], &mut rng_rrw, &self.gauss);

        // ARW: per-sample rate noise stddev = arw_per_sqrt_s / sqrt(dt).
        let arw_step_stddev = budget.arw_per_sqrt_s / self.budget.dt_s.sqrt();
        let arw_noise = self.gauss.sample(&mut rng_arw, 0.0, arw_step_stddev)?;

        // Apply scale factor (deterministic), then add bias channels
        // and the white-noise rate term. Locked operand order.
        let scale = 1.0 + budget.scale_factor_ppm * 1.0e-6;
        let scaled = truth_value * scale;
        let noisy = scaled + self.bias_ou[bias_ou_idx] + self.bias_rrw[bias_rrw_idx] + arw_noise;

        // Quantisation. lsb = 0 disables.
        let quantised = if budget.quantization_lsb > 0.0 {
            (noisy / budget.quantization_lsb).round() * budget.quantization_lsb
        } else {
            noisy
        };

        if !quantised.is_finite() {
            return Err(SensorError::NonFinite {
                reason: "IMU axis measurement produced a non-finite value",
            });
        }
        Ok(quantised)
    }
}

impl Sensor for SyntheticImu {
    type Output = SensorMeasurement;
    fn sensor_id(&self) -> SensorId {
        self.sensor_id
    }
}

impl SyntheticSensor for SyntheticImu {
    fn measure(
        &mut self,
        truth: &SensorTruth,
        step: StepIndex,
        scenario_seed: u64,
    ) -> Result<SensorMeasurement, SensorError> {
        require_truth_finite(truth)?;

        // Snapshot the OU/RRW primitives so we don't borrow `self`
        // mutably twice. They're `Copy`.
        let gyro_ou = self.gyro_ou;
        let accel_ou = self.accel_ou;
        let gyro_rrw = self.gyro_rrw;
        let accel_rrw = self.accel_rrw;
        let gyro_budget = self.budget.gyro;
        let accel_budget = self.budget.accel;

        let truth_omega = truth.angular_velocity_body_rad_s;
        let truth_accel = truth.specific_force_body_m_s2;

        let gyro_x = self.measure_axis(
            truth_omega.x,
            &gyro_budget,
            &gyro_ou,
            &gyro_rrw,
            0,
            0,
            scenario_seed,
            step,
            GYRO_AXIS_BASES[0],
        )?;
        let gyro_y = self.measure_axis(
            truth_omega.y,
            &gyro_budget,
            &gyro_ou,
            &gyro_rrw,
            1,
            1,
            scenario_seed,
            step,
            GYRO_AXIS_BASES[1],
        )?;
        let gyro_z = self.measure_axis(
            truth_omega.z,
            &gyro_budget,
            &gyro_ou,
            &gyro_rrw,
            2,
            2,
            scenario_seed,
            step,
            GYRO_AXIS_BASES[2],
        )?;
        let accel_x = self.measure_axis(
            truth_accel.x,
            &accel_budget,
            &accel_ou,
            &accel_rrw,
            3,
            3,
            scenario_seed,
            step,
            ACCEL_AXIS_BASES[0],
        )?;
        let accel_y = self.measure_axis(
            truth_accel.y,
            &accel_budget,
            &accel_ou,
            &accel_rrw,
            4,
            4,
            scenario_seed,
            step,
            ACCEL_AXIS_BASES[1],
        )?;
        let accel_z = self.measure_axis(
            truth_accel.z,
            &accel_budget,
            &accel_ou,
            &accel_rrw,
            5,
            5,
            scenario_seed,
            step,
            ACCEL_AXIS_BASES[2],
        )?;

        Ok(SensorMeasurement::Imu {
            gyro_rad_s: Vector3::new(gyro_x, gyro_y, gyro_z),
            accel_m_s2: Vector3::new(accel_x, accel_y, accel_z),
        })
    }
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::panic,
    clippy::similar_names
)]
mod tests {
    use super::*;
    use nalgebra::UnitQuaternion;
    use openbmp_core::{ChannelId, Position3, SimTime, Velocity3};

    fn fixture_truth() -> SensorTruth {
        SensorTruth {
            position_eci: Position3::new(0.0, 0.0, 0.0),
            velocity_eci: Velocity3::new(0.0, 0.0, 0.0),
            attitude_eci_to_body: UnitQuaternion::identity(),
            angular_velocity_body_rad_s: Vector3::new(0.01, 0.02, 0.03),
            specific_force_body_m_s2: Vector3::new(0.5, 1.0, 9.806_65),
            static_pressure_pa: 0.0,
            altitude_geometric_m: 0.0,
            magnetic_field_body_nt: Vector3::zeros(),
            time: SimTime::ZERO,
        }
    }

    fn zero_noise_budget() -> ImuNoiseBudget {
        let triax = TriaxialNoiseBudget::new(0.0, 1.0, 0.0, 0.0, 0.0, 0.0).unwrap();
        ImuNoiseBudget::new(triax, triax, 0.01).unwrap()
    }

    fn small_noise_budget() -> ImuNoiseBudget {
        let gyro = TriaxialNoiseBudget::new(
            1.0e-4, // ARW (rad/√s) — modest
            1.0,    // bias_ou_theta (1/s)
            1.0e-4, // bias_ou_sigma
            1.0e-5, // rrw sigma
            10.0,   // scale factor 10 ppm
            0.0,    // no quantisation
        )
        .unwrap();
        let accel = TriaxialNoiseBudget::new(
            1.0e-3, // VRW (m/s² /√s)
            1.0, 1.0e-3, 1.0e-4, 10.0, 0.0,
        )
        .unwrap();
        ImuNoiseBudget::new(gyro, accel, 0.01).unwrap()
    }

    #[test]
    fn imu_with_zero_noise_returns_truth_through_scale_factor() {
        let mut imu =
            SyntheticImu::new(SensorId::from_path("sensors.imu"), zero_noise_budget()).unwrap();
        let truth = fixture_truth();
        let m = imu.measure(&truth, StepIndex::new(0), 0).unwrap();
        match m {
            SensorMeasurement::Imu {
                gyro_rad_s,
                accel_m_s2,
            } => {
                // scale_factor_ppm = 0 → multiplier = 1.0 → bit-exact echo.
                assert_eq!(gyro_rad_s, truth.angular_velocity_body_rad_s);
                assert_eq!(accel_m_s2, truth.specific_force_body_m_s2);
            }
            _ => panic!("expected Imu variant"),
        }
    }

    #[test]
    fn imu_quantisation_rounds_to_nearest_lsb() {
        let triax = TriaxialNoiseBudget::new(0.0, 1.0, 0.0, 0.0, 0.0, 0.1).unwrap();
        let budget = ImuNoiseBudget::new(triax, triax, 0.01).unwrap();
        let mut imu = SyntheticImu::new(SensorId::from_path("sensors.imu"), budget).unwrap();
        let mut truth = fixture_truth();
        // Inputs that are nearly-but-not-exactly LSB multiples.
        truth.angular_velocity_body_rad_s = Vector3::new(0.123, 0.456, 0.789);
        truth.specific_force_body_m_s2 = Vector3::new(0.05, -0.05, 0.16);
        let m = imu.measure(&truth, StepIndex::new(0), 0).unwrap();
        match m {
            SensorMeasurement::Imu {
                gyro_rad_s,
                accel_m_s2,
            } => {
                // 0.123 → 0.1, 0.456 → 0.5, 0.789 → 0.8.
                assert!((gyro_rad_s.x - 0.1).abs() < 1.0e-12);
                assert!((gyro_rad_s.y - 0.5).abs() < 1.0e-12);
                assert!((gyro_rad_s.z - 0.8).abs() < 1.0e-12);
                // 0.05 rounds to nearest even per banker's rounding;
                // f64::round rounds half away from zero, giving 0.1 / -0.1.
                assert!((accel_m_s2.x - 0.1).abs() < 1.0e-12);
                assert!((accel_m_s2.y + 0.1).abs() < 1.0e-12);
                assert!((accel_m_s2.z - 0.2).abs() < 1.0e-12);
            }
            _ => panic!("expected Imu variant"),
        }
    }

    #[test]
    fn imu_is_bit_stable_across_two_runs() {
        let mut a =
            SyntheticImu::new(SensorId::from_path("sensors.imu"), small_noise_budget()).unwrap();
        let mut b =
            SyntheticImu::new(SensorId::from_path("sensors.imu"), small_noise_budget()).unwrap();
        let truth = fixture_truth();
        for k in 0..200 {
            let ma = a.measure(&truth, StepIndex::new(k), 0x00C0_FFEE).unwrap();
            let mb = b.measure(&truth, StepIndex::new(k), 0x00C0_FFEE).unwrap();
            assert_eq!(ma, mb);
        }
    }

    /// Reordering scenario sensors must not shift any sensor's RNG
    /// stream, because `SensorId` is path-derived and sensor-component
    /// RNGs are domain-separated from telemetry-channel RNGs.
    #[test]
    fn imu_stream_is_independent_of_sibling_sensors() {
        let truth = fixture_truth();
        let imu_id = SensorId::from_path("sensors.imu");

        // Run A: build the IMU, take 50 measurements.
        let mut imu_a = SyntheticImu::new(imu_id, small_noise_budget()).unwrap();
        let mut stream_a = Vec::new();
        for k in 0..50 {
            stream_a.push(imu_a.measure(&truth, StepIndex::new(k), 7).unwrap());
        }

        // Run B: construct and measure a sibling barometer before the
        // IMU on every step. The IMU stream must be byte-identical to A
        // because each noise stream is keyed by (sensor_id, component_id),
        // not construction or measurement order.
        let mut baro = crate::barometer::SyntheticBarometer::new(
            SensorId::from_path("sensors.barometer"),
            10.0,
            1.0,
            5.0,
            0.01,
        )
        .unwrap();
        let mut imu_b = SyntheticImu::new(imu_id, small_noise_budget()).unwrap();
        let mut stream_b = Vec::new();
        for k in 0..50 {
            let step = StepIndex::new(k);
            let _ = baro.measure(&truth, step, 7).unwrap();

            // The most collision-prone payload overlap still differs
            // because sensor-component RNG seeds carry the SENS domain tag.
            let mut channel_rng =
                DeterministicRng::for_channel(7, step, ChannelId::new(imu_id.value()));
            let mut sensor_rng = DeterministicRng::for_sensor_component(7, step, imu_id, SUB_ARW);
            assert_ne!(channel_rng.next_u64(), sensor_rng.next_u64());

            stream_b.push(imu_b.measure(&truth, step, 7).unwrap());
        }

        assert_eq!(stream_a, stream_b);
    }

    #[test]
    fn triaxial_noise_budget_rejects_invalid_inputs() {
        // negative arw
        assert!(matches!(
            TriaxialNoiseBudget::new(-1.0, 1.0, 0.0, 0.0, 0.0, 0.0),
            Err(SensorError::InvalidParameter { .. })
        ));
        // zero theta
        assert!(matches!(
            TriaxialNoiseBudget::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            Err(SensorError::InvalidParameter { .. })
        ));
        // NaN
        assert!(matches!(
            TriaxialNoiseBudget::new(f64::NAN, 1.0, 0.0, 0.0, 0.0, 0.0),
            Err(SensorError::NonFinite { .. })
        ));
    }

    #[test]
    fn imu_noise_budget_rejects_non_positive_dt() {
        let triax = TriaxialNoiseBudget::new(0.0, 1.0, 0.0, 0.0, 0.0, 0.0).unwrap();
        assert!(matches!(
            ImuNoiseBudget::new(triax, triax, 0.0),
            Err(SensorError::InvalidParameter { .. })
        ));
        assert!(matches!(
            ImuNoiseBudget::new(triax, triax, f64::NAN),
            Err(SensorError::NonFinite { .. })
        ));
    }
}
