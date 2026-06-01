//! Lockstep protocol validators for the generic HIL bridge.
//!
//! The simulator must not advance the plant past step `N` until the
//! external flight-controller side has consumed the sensor frame for
//! step `N` and returned either matching actuator commands or an
//! explicit acknowledgement/fault. These helpers enforce the schema
//! invariants; transports and retry policy remain downstream-owned.

use crate::error::BridgeError;
use crate::packet::{ActuatorCommandPacket, PROTOCOL_VERSION, SensorPacket, StepAckPacket};

/// Validate a peer's advertised protocol version.
///
/// # Errors
///
/// Returns [`BridgeError::ProtocolVersionMismatch`] when `peer_version`
/// differs from [`PROTOCOL_VERSION`].
pub fn validate_protocol_version(peer_version: u16) -> Result<(), BridgeError> {
    if peer_version == PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(BridgeError::ProtocolVersionMismatch {
            expected: PROTOCOL_VERSION,
            got: peer_version,
        })
    }
}

/// Validate that a command packet unblocks exactly the outstanding
/// sensor frame.
///
/// # Errors
///
/// Returns [`BridgeError::StepMismatch`] or [`BridgeError::TimeMismatch`]
/// when the command references a different simulator step.
pub fn validate_command_for_sensor(
    sensor: &SensorPacket,
    command: &ActuatorCommandPacket,
) -> Result<(), BridgeError> {
    validate_step_and_time(
        sensor.step,
        sensor.sim_time_s,
        command.step,
        command.sim_time_s,
    )
}

/// Validate that an explicit acknowledgement unblocks exactly the
/// outstanding sensor frame.
///
/// # Errors
///
/// Returns [`BridgeError::StepMismatch`] or [`BridgeError::TimeMismatch`]
/// when the acknowledgement references a different simulator step.
pub fn validate_ack_for_sensor(
    sensor: &SensorPacket,
    ack: &StepAckPacket,
) -> Result<(), BridgeError> {
    validate_step_and_time(sensor.step, sensor.sim_time_s, ack.step, ack.sim_time_s)
}

fn validate_step_and_time(
    expected_step: u64,
    expected_time_s: f64,
    got_step: u64,
    got_time_s: f64,
) -> Result<(), BridgeError> {
    if got_step != expected_step {
        return Err(BridgeError::StepMismatch {
            expected: expected_step,
            got: got_step,
        });
    }
    if got_time_s.to_bits() != expected_time_s.to_bits() {
        return Err(BridgeError::TimeMismatch {
            expected: expected_time_s,
            got: got_time_s,
        });
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::packet::{PROTOCOL_VERSION, StepAckStatus};

    fn sensor(step: u64) -> SensorPacket {
        SensorPacket {
            sim_time_s: step as f64 * 0.01,
            step,
            imu_accel_body_m_s2: [0.0, 0.0, 9.81],
            imu_gyro_body_rad_s: [0.0; 3],
            baro_altitude_m: None,
            gnss_position_eci_m: None,
            gnss_velocity_eci_m_s: None,
            mag_body_tesla: None,
        }
    }

    #[test]
    fn protocol_version_accepts_current_version() {
        validate_protocol_version(PROTOCOL_VERSION).unwrap();
    }

    #[test]
    fn protocol_version_rejects_mismatch() {
        assert!(matches!(
            validate_protocol_version(PROTOCOL_VERSION + 1),
            Err(BridgeError::ProtocolVersionMismatch { .. })
        ));
    }

    #[test]
    fn matching_command_unblocks_sensor_step() {
        let sensor = sensor(42);
        let command = ActuatorCommandPacket {
            sim_time_s: sensor.sim_time_s,
            step: sensor.step,
            effector_commands: Vec::new(),
            engine_throttles: Vec::new(),
        };
        validate_command_for_sensor(&sensor, &command).unwrap();
    }

    #[test]
    fn command_for_wrong_step_is_rejected() {
        let sensor = sensor(42);
        let command = ActuatorCommandPacket {
            sim_time_s: sensor.sim_time_s,
            step: 43,
            effector_commands: Vec::new(),
            engine_throttles: Vec::new(),
        };
        assert!(matches!(
            validate_command_for_sensor(&sensor, &command),
            Err(BridgeError::StepMismatch {
                expected: 42,
                got: 43
            })
        ));
    }

    #[test]
    fn ack_for_wrong_time_is_rejected() {
        let sensor = sensor(42);
        let ack = StepAckPacket {
            sim_time_s: sensor.sim_time_s + 0.01,
            step: sensor.step,
            status: StepAckStatus::Accepted,
        };
        assert!(matches!(
            validate_ack_for_sensor(&sensor, &ack),
            Err(BridgeError::TimeMismatch { .. })
        ));
    }
}
