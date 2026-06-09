//! Open-loop measurement-domain SIL fault injection.
//!
//! A [`MeasurementStimulus`] transforms a freshly-read [`SensorMeasurement`]
//! before it reaches the flight controller, modelling a sensor fault at the
//! FC/sensor boundary.
//!
//! Forward-only by construction: [`MeasurementStimulus::apply`] takes ONLY
//! the measurement and a deterministic RNG — never truth or any estimate —
//! so a stimulus at tick `t` is a function of `(tick, authored parameters,
//! seed)` alone. A closed-loop "drive to a condition" law that reads the
//! vehicle's state is not expressible through this type.

use nalgebra::Vector3;
use openbmp_core::DeterministicRng;

use crate::sensor::SensorMeasurement;

/// An open-loop measurement-domain fault applied at the sensor-read →
/// FC-publish boundary.
#[derive(Clone, Debug, PartialEq)]
pub enum MeasurementStimulus {
    /// Add a constant offset to the sensor's primary measured vector:
    /// accelerometer specific force for an IMU, ECI position for a GNSS
    /// receiver, body-frame field for a magnetometer, and the offset's
    /// `x` component to the scalar barometer pressure. Deterministic —
    /// draws nothing from the RNG. Star-tracker and ideal-state
    /// measurements are unaffected.
    Bias {
        /// Constant additive offset in the sensor's measurement units.
        offset: Vector3<f64>,
    },
}

impl MeasurementStimulus {
    /// Apply this fault to a freshly-read measurement in place.
    ///
    /// `rng` draws from the dedicated SIL-stimulus stream
    /// ([`DeterministicRng::for_stimulus_component`]); deterministic faults
    /// such as [`Self::Bias`] ignore it, but keeping it in the signature
    /// lets stochastic faults stay byte-stable without changing the seam.
    pub fn apply(&self, measurement: &mut SensorMeasurement, rng: &mut DeterministicRng) {
        let _ = rng;
        match self {
            Self::Bias { offset } => apply_bias(measurement, *offset),
        }
    }
}

fn apply_bias(measurement: &mut SensorMeasurement, offset: Vector3<f64>) {
    match measurement {
        SensorMeasurement::Imu { accel_m_s2, .. } => *accel_m_s2 += offset,
        SensorMeasurement::Gnss { position_eci_m, .. } => *position_eci_m += offset,
        SensorMeasurement::Magnetometer { field_body_nt, .. } => *field_body_nt += offset,
        SensorMeasurement::Barometer { pressure_pa, .. } => *pressure_pa += offset.x,
        SensorMeasurement::StarTracker { .. } | SensorMeasurement::IdealState(_) => {}
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;
    use openbmp_core::{SensorId, StepIndex};

    fn rng() -> DeterministicRng {
        DeterministicRng::for_stimulus_component(7, StepIndex::new(0), SensorId::new(1), 0)
    }

    #[test]
    fn bias_shifts_gnss_position_by_offset() {
        let mut measurement = SensorMeasurement::Gnss {
            position_eci_m: Vector3::new(100.0, 200.0, 300.0),
            velocity_eci_m_s: Vector3::new(1.0, 2.0, 3.0),
            position_bias_eci_m: Vector3::zeros(),
        };
        let stim = MeasurementStimulus::Bias {
            offset: Vector3::new(10.0, -5.0, 2.0),
        };
        stim.apply(&mut measurement, &mut rng());
        match measurement {
            SensorMeasurement::Gnss {
                position_eci_m,
                velocity_eci_m_s,
                ..
            } => {
                assert_eq!(position_eci_m, Vector3::new(110.0, 195.0, 302.0));
                // Velocity is untouched.
                assert_eq!(velocity_eci_m_s, Vector3::new(1.0, 2.0, 3.0));
            }
            other => panic!("expected GNSS, got {other:?}"),
        }
    }

    #[test]
    fn bias_shifts_imu_accel_and_barometer_pressure() {
        let mut imu = SensorMeasurement::Imu {
            gyro_rad_s: Vector3::new(0.1, 0.2, 0.3),
            accel_m_s2: Vector3::new(0.0, 0.0, -9.81),
        };
        MeasurementStimulus::Bias {
            offset: Vector3::new(0.5, 0.0, 0.0),
        }
        .apply(&mut imu, &mut rng());
        match imu {
            SensorMeasurement::Imu {
                gyro_rad_s,
                accel_m_s2,
            } => {
                assert_eq!(accel_m_s2, Vector3::new(0.5, 0.0, -9.81));
                assert_eq!(gyro_rad_s, Vector3::new(0.1, 0.2, 0.3));
            }
            other => panic!("expected IMU, got {other:?}"),
        }

        let mut baro = SensorMeasurement::Barometer {
            pressure_pa: 101_325.0,
            bias_pa: 0.0,
        };
        MeasurementStimulus::Bias {
            offset: Vector3::new(50.0, 0.0, 0.0),
        }
        .apply(&mut baro, &mut rng());
        match baro {
            SensorMeasurement::Barometer { pressure_pa, .. } => {
                assert_eq!(pressure_pa, 101_375.0);
            }
            other => panic!("expected barometer, got {other:?}"),
        }
    }
}
