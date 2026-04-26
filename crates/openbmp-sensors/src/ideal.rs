//! Ideal-state sensor — bit-equal echo of the truth bag.
//!
//! Useful for analytic-toy validation and golden-output regression
//! where the noise channels are intentionally muted. The
//! `IdealStateSensor` carries a [`SensorId`] for telemetry routing
//! but never draws from the RNG.

use openbmp_core::{SensorId, StepIndex};

use crate::error::SensorError;
use crate::sensor::{SensorMeasurement, SensorTruth, SyntheticSensor, require_truth_finite};

/// Ideal sensor that returns the truth bag verbatim.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct IdealStateSensor {
    sensor_id: SensorId,
}

impl IdealStateSensor {
    /// Construct from a [`SensorId`].
    #[must_use]
    pub const fn new(sensor_id: SensorId) -> Self {
        Self { sensor_id }
    }
}

impl SyntheticSensor for IdealStateSensor {
    fn sensor_id(&self) -> SensorId {
        self.sensor_id
    }

    fn measure(
        &mut self,
        truth: &SensorTruth,
        _step: StepIndex,
        _scenario_seed: u64,
    ) -> Result<SensorMeasurement, SensorError> {
        require_truth_finite(truth)?;
        Ok(SensorMeasurement::IdealState(*truth))
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
            position_eci: Position3::new(7_000_000.0, 1_500_000.0, 800_000.0),
            velocity_eci: Velocity3::new(100.0, 200.0, 300.0),
            attitude_eci_to_body: UnitQuaternion::identity(),
            angular_velocity_body_rad_s: Vector3::new(0.01, 0.02, 0.03),
            specific_force_body_m_s2: Vector3::new(0.0, 0.0, 9.806_65),
            static_pressure_pa: 101_325.0,
            altitude_geometric_m: 0.0,
            time: SimTime::ZERO,
        }
    }

    #[test]
    fn ideal_state_sensor_round_trips_truth_bit_equal() {
        let mut s = IdealStateSensor::new(SensorId::from_path("sensors.ideal_state"));
        let truth = fixture_truth();
        let m = s.measure(&truth, StepIndex::new(0), 0).unwrap();
        match m {
            SensorMeasurement::IdealState(echoed) => {
                assert_eq!(echoed, truth);
            }
            _ => panic!("expected IdealState variant"),
        }
    }

    #[test]
    fn ideal_state_sensor_rejects_non_finite_truth() {
        let mut s = IdealStateSensor::new(SensorId::from_path("sensors.ideal_state"));
        let mut truth = fixture_truth();
        truth.static_pressure_pa = f64::NAN;
        assert!(matches!(
            s.measure(&truth, StepIndex::new(0), 0),
            Err(SensorError::NonFinite { .. }),
        ));
    }

    #[test]
    fn ideal_state_sensor_id_is_path_derived() {
        let s = IdealStateSensor::new(SensorId::from_path("sensors.ideal_state"));
        assert_eq!(s.sensor_id(), SensorId::from_path("sensors.ideal_state"));
    }
}
