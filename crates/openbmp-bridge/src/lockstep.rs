//! Lockstep protocol validators for the generic HIL bridge.
//!
//! The simulator must not advance the plant past step `N` until the
//! external flight-controller side has consumed the sensor frame for
//! step `N` and returned either matching actuator commands or an
//! explicit acknowledgement/fault. These helpers enforce the schema
//! invariants; transports and retry policy remain downstream-owned.

use crate::error::BridgeError;
use crate::packet::{ActuatorCommandPacket, PROTOCOL_VERSION, SensorPacket, StepAckPacket};
#[cfg(feature = "std")]
use crate::packet::{
    BridgeEndpointRole, BridgeFaultCode, BridgeFaultPacket, BridgeHelloPacket, BridgeMessage,
};
#[cfg(feature = "std")]
use crate::transport::Transport;

/// Response that unblocks one simulator lockstep frame.
#[cfg(feature = "std")]
#[derive(Clone, Debug, PartialEq)]
pub enum LockstepResponse {
    /// Peer returned actuator commands for the outstanding sensor frame.
    Command(ActuatorCommandPacket),
    /// Peer explicitly acknowledged the outstanding sensor frame.
    Ack(StepAckPacket),
    /// Peer reported a fail-closed fault.
    Fault(BridgeFaultPacket),
}

/// Simulator-side lockstep master over a message transport.
///
/// The master sends the protocol hello, validates the peer hello, emits
/// one [`SensorPacket`] per simulator frame, then blocks until the peer
/// returns a matching [`ActuatorCommandPacket`], matching
/// [`StepAckPacket`], or fail-closed [`BridgeFaultPacket`].
#[cfg(feature = "std")]
#[derive(Debug)]
pub struct LockstepSimMaster<T> {
    transport: T,
    max_payload_len: u32,
}

#[cfg(feature = "std")]
impl<T: Transport> LockstepSimMaster<T> {
    /// Construct a simulator-side lockstep master.
    #[must_use]
    pub const fn new(transport: T, max_payload_len: u32) -> Self {
        Self {
            transport,
            max_payload_len,
        }
    }

    /// Consume this master and return the wrapped transport.
    #[must_use]
    pub fn into_inner(self) -> T {
        self.transport
    }

    /// Exchange hello messages and validate protocol version plus peer role.
    ///
    /// # Errors
    ///
    /// Returns [`BridgeError`] if the transport fails, the peer sends a
    /// non-hello message, advertises an incompatible protocol version, or
    /// does not advertise [`BridgeEndpointRole::FlightController`].
    pub fn handshake(&mut self) -> Result<BridgeHelloPacket, BridgeError> {
        self.transport
            .send(&BridgeMessage::Hello(BridgeHelloPacket::new(
                BridgeEndpointRole::Simulator,
                self.max_payload_len,
            )))?;

        let message = self.transport.recv()?;
        let BridgeMessage::Hello(peer_hello) = message else {
            let error = BridgeError::UnexpectedMessage {
                expected: "Hello",
                got: message_kind(&message),
            };
            self.send_fault(None, BridgeFaultCode::ApplicationFault)?;
            return Err(error);
        };

        if let Err(error) = validate_protocol_version(peer_hello.protocol_version) {
            self.send_fault(None, BridgeFaultCode::ProtocolVersionMismatch)?;
            return Err(error);
        }

        if peer_hello.role != BridgeEndpointRole::FlightController {
            self.send_fault(None, BridgeFaultCode::ApplicationFault)?;
            return Err(BridgeError::RoleMismatch {
                expected: BridgeEndpointRole::FlightController,
                got: peer_hello.role,
            });
        }

        Ok(peer_hello)
    }

    /// Send one sensor frame and block until the peer returns a valid response.
    ///
    /// # Errors
    ///
    /// Returns [`BridgeError`] if transport I/O fails, the peer sends an
    /// unexpected message, or a command/ack response references a different
    /// step or simulation timestamp. On validation failures the master sends
    /// a fail-closed fault before returning the error.
    pub fn step(&mut self, sensor: &SensorPacket) -> Result<LockstepResponse, BridgeError> {
        self.transport
            .send(&BridgeMessage::Sensor(sensor.clone()))?;
        let message = self.transport.recv()?;

        match message {
            BridgeMessage::Command(command) => {
                if let Err(error) = validate_command_for_sensor(sensor, &command) {
                    self.send_fault(Some(sensor.step), fault_code_for_error(&error))?;
                    return Err(error);
                }
                Ok(LockstepResponse::Command(command))
            }
            BridgeMessage::Ack(ack) => {
                if let Err(error) = validate_ack_for_sensor(sensor, &ack) {
                    self.send_fault(Some(sensor.step), fault_code_for_error(&error))?;
                    return Err(error);
                }
                Ok(LockstepResponse::Ack(ack))
            }
            BridgeMessage::Fault(fault) => Ok(LockstepResponse::Fault(fault)),
            other => {
                let error = BridgeError::UnexpectedMessage {
                    expected: "Command | Ack | Fault",
                    got: message_kind(&other),
                };
                self.send_fault(Some(sensor.step), BridgeFaultCode::ApplicationFault)?;
                Err(error)
            }
        }
    }

    /// Send a fail-closed fault packet through the wrapped transport.
    ///
    /// # Errors
    ///
    /// Returns [`BridgeError`] if the fault packet cannot be sent.
    pub fn send_fault(
        &mut self,
        step: Option<u64>,
        code: BridgeFaultCode,
    ) -> Result<(), BridgeError> {
        self.transport
            .send(&BridgeMessage::Fault(BridgeFaultPacket { step, code }))
    }
}

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

#[cfg(feature = "std")]
fn message_kind(message: &BridgeMessage) -> &'static str {
    match message {
        BridgeMessage::Hello(_) => "Hello",
        BridgeMessage::Sensor(_) => "Sensor",
        BridgeMessage::Command(_) => "Command",
        BridgeMessage::Ack(_) => "Ack",
        BridgeMessage::Fault(_) => "Fault",
    }
}

#[cfg(feature = "std")]
fn fault_code_for_error(error: &BridgeError) -> BridgeFaultCode {
    match error {
        BridgeError::ProtocolVersionMismatch { .. } => BridgeFaultCode::ProtocolVersionMismatch,
        BridgeError::StepMismatch { .. } => BridgeFaultCode::StepMismatch,
        BridgeError::TimeMismatch { .. } => BridgeFaultCode::TimeMismatch,
        BridgeError::PayloadTooLarge { .. } => BridgeFaultCode::PayloadTooLarge,
        BridgeError::Decode(_) => BridgeFaultCode::DecodeFailed,
        BridgeError::Encode(_)
        | BridgeError::FrameTruncated { .. }
        | BridgeError::PrefixIncomplete { .. }
        | BridgeError::TransportClosed
        | BridgeError::UnexpectedMessage { .. }
        | BridgeError::RoleMismatch { .. } => BridgeFaultCode::ApplicationFault,
        BridgeError::Io(_) => BridgeFaultCode::ApplicationFault,
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use std::thread;

    use crate::packet::{BridgeMessage, PROTOCOL_VERSION, StepAckStatus};
    use crate::transport::{Transport, in_process_transport_pair};

    fn sensor(step: u64) -> SensorPacket {
        SensorPacket {
            sim_time_s: step as f64 * 0.01,
            step,
            imu_accel_body_m_s2: [0.0, 0.0, 9.81],
            imu_gyro_body_rad_s: [0.0; 3],
            baro_altitude_m: None,
            gnss_position_eci_m: None,
            gnss_velocity_eci_m_s: None,
            gnss_position_bias_eci_m: None,
            mag_body_tesla: None,
            mag_body_nt: None,
            mag_hard_iron_body_nt: None,
            baro_pressure_pa: None,
            baro_bias_pa: None,
            star_tracker_attitude_eci_to_body_xyzw: None,
            ..SensorPacket::default()
        }
    }

    fn command_for(sensor: &SensorPacket, step: u64) -> ActuatorCommandPacket {
        ActuatorCommandPacket {
            sim_time_s: sensor.sim_time_s,
            step,
            effector_commands: vec![(0, 0.5)],
            engine_throttles: vec![(0, 0.75)],
            engine_commands: vec![],
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
            engine_commands: Vec::new(),
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
            engine_commands: Vec::new(),
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

    #[test]
    fn lockstep_master_handshakes_and_accepts_command() {
        let (sim_endpoint, mut fc_endpoint) = in_process_transport_pair();
        let sensor = sensor(11);
        let expected_command = command_for(&sensor, sensor.step);
        let peer = thread::spawn({
            let expected_command = expected_command.clone();
            move || {
                assert!(matches!(
                    fc_endpoint.recv().unwrap(),
                    BridgeMessage::Hello(BridgeHelloPacket {
                        protocol_version: PROTOCOL_VERSION,
                        role: BridgeEndpointRole::Simulator,
                        ..
                    })
                ));
                fc_endpoint
                    .send(&BridgeMessage::Hello(BridgeHelloPacket::new(
                        BridgeEndpointRole::FlightController,
                        4096,
                    )))
                    .unwrap();
                assert!(matches!(
                    fc_endpoint.recv().unwrap(),
                    BridgeMessage::Sensor(frame) if frame.step == 11
                ));
                fc_endpoint
                    .send(&BridgeMessage::Command(expected_command))
                    .unwrap();
            }
        });

        let mut master = LockstepSimMaster::new(sim_endpoint, 4096);
        let hello = master.handshake().unwrap();
        assert_eq!(hello.role, BridgeEndpointRole::FlightController);
        assert_eq!(
            master.step(&sensor).unwrap(),
            LockstepResponse::Command(expected_command)
        );
        peer.join().unwrap();
    }

    #[test]
    fn lockstep_master_accepts_explicit_ack() {
        let (sim_endpoint, mut fc_endpoint) = in_process_transport_pair();
        let sensor = sensor(12);
        let ack = StepAckPacket {
            sim_time_s: sensor.sim_time_s,
            step: sensor.step,
            status: StepAckStatus::Accepted,
        };
        let peer = thread::spawn({
            let ack = ack.clone();
            move || {
                fc_endpoint.recv().unwrap();
                fc_endpoint
                    .send(&BridgeMessage::Hello(BridgeHelloPacket::new(
                        BridgeEndpointRole::FlightController,
                        4096,
                    )))
                    .unwrap();
                fc_endpoint.recv().unwrap();
                fc_endpoint.send(&BridgeMessage::Ack(ack)).unwrap();
            }
        });

        let mut master = LockstepSimMaster::new(sim_endpoint, 4096);
        master.handshake().unwrap();
        assert_eq!(master.step(&sensor).unwrap(), LockstepResponse::Ack(ack));
        peer.join().unwrap();
    }

    #[test]
    fn lockstep_master_rejects_step_mismatch_and_emits_fault() {
        let (sim_endpoint, mut fc_endpoint) = in_process_transport_pair();
        let sensor = sensor(13);
        let peer_sensor = sensor.clone();
        let peer = thread::spawn(move || {
            fc_endpoint.recv().unwrap();
            fc_endpoint
                .send(&BridgeMessage::Hello(BridgeHelloPacket::new(
                    BridgeEndpointRole::FlightController,
                    4096,
                )))
                .unwrap();
            fc_endpoint.recv().unwrap();
            fc_endpoint
                .send(&BridgeMessage::Command(command_for(
                    &peer_sensor,
                    peer_sensor.step + 1,
                )))
                .unwrap();
            fc_endpoint.recv().unwrap()
        });

        let mut master = LockstepSimMaster::new(sim_endpoint, 4096);
        master.handshake().unwrap();
        assert!(matches!(
            master.step(&sensor),
            Err(BridgeError::StepMismatch {
                expected: 13,
                got: 14
            })
        ));
        assert!(matches!(
            peer.join().unwrap(),
            BridgeMessage::Fault(BridgeFaultPacket {
                step: Some(13),
                code: BridgeFaultCode::StepMismatch
            })
        ));
    }

    #[test]
    fn lockstep_master_rejects_wrong_peer_role() {
        let (sim_endpoint, mut fc_endpoint) = in_process_transport_pair();
        let peer = thread::spawn(move || {
            fc_endpoint.recv().unwrap();
            fc_endpoint
                .send(&BridgeMessage::Hello(BridgeHelloPacket::new(
                    BridgeEndpointRole::Simulator,
                    4096,
                )))
                .unwrap();
            fc_endpoint.recv().unwrap()
        });

        let mut master = LockstepSimMaster::new(sim_endpoint, 4096);
        assert!(matches!(
            master.handshake(),
            Err(BridgeError::RoleMismatch {
                expected: BridgeEndpointRole::FlightController,
                got: BridgeEndpointRole::Simulator
            })
        ));
        assert!(matches!(
            peer.join().unwrap(),
            BridgeMessage::Fault(BridgeFaultPacket {
                step: None,
                code: BridgeFaultCode::ApplicationFault
            })
        ));
    }
}
