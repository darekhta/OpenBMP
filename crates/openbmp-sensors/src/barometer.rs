//! Synthetic barometric pressure sensor.
//!
//! Reports `truth_pressure + bias + Gaussian(0, σ²)` where the bias
//! evolves as an Ornstein-Uhlenbeck process (slow drift around the
//! long-run mean of zero).
//!
//! Two component IDs are used per sensor:
//! * `0` — Gaussian measurement noise.
//! * `1` — OU bias-drift noise.

use openbmp_core::{DeterministicRng, SensorId, StepIndex};

use crate::error::SensorError;
use crate::noise::{BoxMullerGaussian, OrnsteinUhlenbeck};
use crate::sensor::{SensorMeasurement, SensorTruth, SyntheticSensor, require_truth_finite};

const COMPONENT_ID_MEASUREMENT_NOISE: u32 = 0;
const COMPONENT_ID_BIAS_DRIFT: u32 = 1;

/// Synthetic barometer.
#[derive(Clone, Debug)]
pub struct SyntheticBarometer {
    sensor_id: SensorId,
    /// Per-sample Gaussian noise stddev on the reported pressure (Pa).
    measurement_stddev_pa: f64,
    /// OU process governing the slow bias drift.
    bias_drift: OrnsteinUhlenbeck,
    /// Current bias state in Pa, updated each call.
    bias_pa: f64,
    /// Box-Muller sampler shared by both noise channels.
    gauss: BoxMullerGaussian,
}

impl SyntheticBarometer {
    /// Construct from `(sensor_id, measurement_stddev_pa, bias_theta,
    /// bias_sigma, dt)`.
    ///
    /// The bias starts at zero. `bias_theta = 1 / τ_bias` is the
    /// OU mean-reversion rate; `bias_sigma` is the OU white-noise
    /// strength. Stationary bias-drift stddev is
    /// `bias_sigma / sqrt(2 · bias_theta)`.
    ///
    /// # Errors
    ///
    /// Returns [`SensorError::InvalidParameter`] when
    /// `measurement_stddev_pa < 0`. Other validation errors flow
    /// from [`OrnsteinUhlenbeck::new`].
    pub fn new(
        sensor_id: SensorId,
        measurement_stddev_pa: f64,
        bias_theta: f64,
        bias_sigma: f64,
        dt: f64,
    ) -> Result<Self, SensorError> {
        if !measurement_stddev_pa.is_finite() {
            return Err(SensorError::NonFinite {
                reason: "barometer measurement stddev is NaN or infinite",
            });
        }
        if measurement_stddev_pa < 0.0 {
            return Err(SensorError::InvalidParameter {
                reason: "barometer measurement stddev must be ≥ 0",
            });
        }
        let bias_drift = OrnsteinUhlenbeck::new(bias_theta, bias_sigma, dt)?;
        Ok(Self {
            sensor_id,
            measurement_stddev_pa,
            bias_drift,
            bias_pa: 0.0,
            gauss: BoxMullerGaussian::new(),
        })
    }

    /// Current bias state in Pa.
    #[must_use]
    pub const fn current_bias_pa(&self) -> f64 {
        self.bias_pa
    }
}

impl SyntheticSensor for SyntheticBarometer {
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

        // Per-component RNG sub-streams. Adding a sibling component
        // to the IMU does NOT shift these because each component is
        // keyed by its own (sensor_id, component_id) pair.
        let mut rng_meas = DeterministicRng::for_sensor_component(
            scenario_seed,
            step,
            self.sensor_id,
            COMPONENT_ID_MEASUREMENT_NOISE,
        );
        let mut rng_bias = DeterministicRng::for_sensor_component(
            scenario_seed,
            step,
            self.sensor_id,
            COMPONENT_ID_BIAS_DRIFT,
        );

        // Step the OU bias forward by one sample.
        self.bias_pa = self
            .bias_drift
            .step(self.bias_pa, &mut rng_bias, &self.gauss);

        // Additive Gaussian measurement noise. Locked order:
        // truth + bias + noise.
        let noise = self
            .gauss
            .sample(&mut rng_meas, 0.0, self.measurement_stddev_pa)?;
        let pressure_pa = truth.static_pressure_pa + self.bias_pa + noise;

        Ok(SensorMeasurement::Barometer {
            pressure_pa,
            bias_pa: self.bias_pa,
        })
    }
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::panic
)]
mod tests {
    use super::*;
    use nalgebra::{UnitQuaternion, Vector3};
    use openbmp_core::{Position3, SimTime, Velocity3};

    fn fixture_truth() -> SensorTruth {
        SensorTruth {
            position_eci: Position3::new(0.0, 0.0, 0.0),
            velocity_eci: Velocity3::new(0.0, 0.0, 0.0),
            attitude_eci_to_body: UnitQuaternion::identity(),
            angular_velocity_body_rad_s: Vector3::zeros(),
            specific_force_body_m_s2: Vector3::zeros(),
            static_pressure_pa: 101_325.0,
            altitude_geometric_m: 0.0,
            magnetic_field_body_nt: Vector3::zeros(),
            time: SimTime::ZERO,
        }
    }

    #[test]
    fn barometer_with_zero_noise_and_zero_bias_returns_truth_pressure_exactly() {
        let mut b = SyntheticBarometer::new(
            SensorId::from_path("sensors.barometer"),
            0.0, // no measurement noise
            1.0, // bias theta (positive but irrelevant when sigma = 0)
            0.0, // bias sigma — no drift
            0.01,
        )
        .unwrap();
        let truth = fixture_truth();
        let m = b.measure(&truth, StepIndex::new(0), 0).unwrap();
        match m {
            SensorMeasurement::Barometer {
                pressure_pa,
                bias_pa,
            } => {
                assert_eq!(pressure_pa.to_bits(), truth.static_pressure_pa.to_bits());
                assert_eq!(bias_pa.to_bits(), 0.0_f64.to_bits());
            }
            _ => panic!("expected Barometer variant"),
        }
    }

    #[test]
    fn barometer_is_bit_stable_across_two_runs() {
        let mut a = SyntheticBarometer::new(
            SensorId::from_path("sensors.barometer"),
            10.0,
            1.0,
            5.0,
            0.01,
        )
        .unwrap();
        let mut b = SyntheticBarometer::new(
            SensorId::from_path("sensors.barometer"),
            10.0,
            1.0,
            5.0,
            0.01,
        )
        .unwrap();
        let truth = fixture_truth();
        for k in 0..200 {
            let ma = a.measure(&truth, StepIndex::new(k), 0x00C0_FFEE).unwrap();
            let mb = b.measure(&truth, StepIndex::new(k), 0x00C0_FFEE).unwrap();
            assert_eq!(ma, mb);
        }
    }

    #[test]
    fn barometer_rejects_non_finite_truth() {
        let mut b = SyntheticBarometer::new(
            SensorId::from_path("sensors.barometer"),
            10.0,
            1.0,
            5.0,
            0.01,
        )
        .unwrap();
        let mut truth = fixture_truth();
        truth.altitude_geometric_m = f64::NAN;
        assert!(matches!(
            b.measure(&truth, StepIndex::new(0), 0),
            Err(SensorError::NonFinite { .. }),
        ));
    }

    #[test]
    fn barometer_constructor_rejects_negative_measurement_stddev() {
        let err = SyntheticBarometer::new(
            SensorId::from_path("sensors.barometer"),
            -1.0,
            1.0,
            5.0,
            0.01,
        )
        .unwrap_err();
        assert!(matches!(err, SensorError::InvalidParameter { .. }));
    }
}
