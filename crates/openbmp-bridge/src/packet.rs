//! Abstract HIL message schema.
//!
//! These types are deliberately transport- and hardware-agnostic.
//! Fields are plain SI scalars and normalized commands; the schema
//! does not reference any concrete sensor product, bus protocol, or
//! flight-software stack.

use serde::{Deserialize, Serialize};

/// Wire protocol version. Both ends compare this and reject a peer
/// that does not match. Bump on any breaking schema change.
pub const PROTOCOL_VERSION: u16 = 2;

/// Endpoint role advertised during the in-house lockstep handshake.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum BridgeEndpointRole {
    /// OpenBMP simulator side: emits sensor frames and waits for matching commands.
    Simulator,
    /// External flight-controller side: consumes sensor frames and emits commands.
    FlightController,
}

/// Versioned hello packet exchanged before a lockstep stream starts.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BridgeHelloPacket {
    /// Wire protocol version supported by the sender.
    pub protocol_version: u16,
    /// Sender role in the lockstep exchange.
    pub role: BridgeEndpointRole,
    /// Maximum length-prefixed payload the sender will accept, in bytes.
    pub max_payload_len: u32,
}

impl BridgeHelloPacket {
    /// Construct a hello packet for the current protocol version.
    #[must_use]
    pub const fn new(role: BridgeEndpointRole, max_payload_len: u32) -> Self {
        Self {
            protocol_version: PROTOCOL_VERSION,
            role,
            max_payload_len,
        }
    }
}

/// Simulated sensor measurements emitted by the simulator each tick.
///
/// Every optional field is `None` when the scenario does not model
/// that sensor. All quantities are in SI units in the body or ECI
/// frame as named.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SensorPacket {
    /// Simulation time of this sample (s).
    pub sim_time_s: f64,
    /// Kernel step index of this sample.
    pub step: u64,
    /// Specific force in the body frame (m/s²), as an IMU accelerometer
    /// would measure it.
    pub imu_accel_body_m_s2: [f64; 3],
    /// Angular velocity in the body frame (rad/s), as an IMU gyro would
    /// measure it.
    pub imu_gyro_body_rad_s: [f64; 3],
    /// Barometric altitude (m), when a barometer is modelled.
    pub baro_altitude_m: Option<f64>,
    /// GNSS position in the ECI frame (m), when GNSS is modelled.
    pub gnss_position_eci_m: Option<[f64; 3]>,
    /// GNSS velocity in the ECI frame (m/s), when GNSS is modelled.
    pub gnss_velocity_eci_m_s: Option<[f64; 3]>,
    /// GNSS position-bias state in the ECI frame (m), when the
    /// synthetic receiver exposes a slowly drifting bias term.
    pub gnss_position_bias_eci_m: Option<[f64; 3]>,
    /// Magnetic field in the body frame (tesla), when a magnetometer is
    /// modelled.
    pub mag_body_tesla: Option<[f64; 3]>,
    /// Magnetic field in the body frame (nT), preserving the FC bus unit
    /// without round-trip unit conversion.
    pub mag_body_nt: Option<[f64; 3]>,
    /// Magnetometer hard-iron body-frame offset (nT), when exposed by
    /// the synthetic sensor budget.
    pub mag_hard_iron_body_nt: Option<[f64; 3]>,
    /// Barometer pressure measurement (Pa), preserving the FC bus value.
    pub baro_pressure_pa: Option<f64>,
    /// Barometer bias state (Pa), preserving the FC bus value.
    pub baro_bias_pa: Option<f64>,
    /// Star-tracker attitude quaternion `[x, y, z, w]` from ECI to body.
    pub star_tracker_attitude_eci_to_body_xyzw: Option<[f64; 4]>,
}

/// Abstract normalized actuator commands accepted by the simulator
/// each tick.
///
/// Commands are addressed by the integer id the scenario assigns to
/// each effector / engine. Effector commands are normalized
/// deflections; engine throttles are in `[0, 1]`. The simulator
/// clamps both at apply time.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ActuatorCommandPacket {
    /// Simulation time the command targets (s).
    pub sim_time_s: f64,
    /// Kernel step index the command targets.
    pub step: u64,
    /// `(effector_id, normalized_command)` pairs.
    pub effector_commands: Vec<(u32, f64)>,
    /// `(engine_id, throttle_unit)` pairs, throttle in `[0, 1]`.
    pub engine_throttles: Vec<(u32, f64)>,
    /// Full per-engine commands, preserving throttle, gimbal, ignition,
    /// and shutdown fields for runner-side engine racks.
    pub engine_commands: Vec<EngineCommandPacket>,
}

/// One engine-specific command in a bridge actuator packet.
#[derive(Copy, Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct EngineCommandPacket {
    /// Engine id assigned by the scenario.
    pub engine_id: u32,
    /// Commanded throttle in `[0, 1]`.
    pub throttle_unit: f64,
    /// Commanded pitch gimbal angle (rad).
    pub gimbal_pitch_rad: f64,
    /// Commanded yaw gimbal angle (rad).
    pub gimbal_yaw_rad: f64,
    /// `true` if ignition is requested.
    pub ignite: bool,
    /// `true` if shutdown is requested.
    pub shutdown: bool,
}

/// Result of consuming one simulator sensor frame.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum StepAckStatus {
    /// The frame was consumed and matching commands are ready or intentionally empty.
    Accepted,
    /// The frame was rejected because it arrived after a newer step.
    RejectedLate,
    /// The frame was rejected because the receiver could not process it.
    RejectedFault,
}

/// Explicit acknowledgement for a lockstep sensor frame.
///
/// In the normal path an [`ActuatorCommandPacket`] with the same
/// `step` and `sim_time_s` is also an acknowledgement. This packet is
/// available for empty-command, fault, or heartbeat paths where the
/// receiver must still unblock the simulator deterministically.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct StepAckPacket {
    /// Simulation time of the sensor frame being acknowledged (s).
    pub sim_time_s: f64,
    /// Kernel step index of the sensor frame being acknowledged.
    pub step: u64,
    /// Receiver disposition for this step.
    pub status: StepAckStatus,
}

/// Generic lockstep fault code.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum BridgeFaultCode {
    /// Peer protocol version did not match [`PROTOCOL_VERSION`].
    ProtocolVersionMismatch,
    /// A response referenced a different step than the outstanding frame.
    StepMismatch,
    /// A response referenced a different simulation timestamp.
    TimeMismatch,
    /// The incoming payload exceeded the receiver's advertised limit.
    PayloadTooLarge,
    /// The incoming payload could not be decoded.
    DecodeFailed,
    /// The receiver hit an application-level fault outside the wire schema.
    ApplicationFault,
}

/// Fault packet for fail-closed lockstep streams.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BridgeFaultPacket {
    /// Step associated with the fault, when known.
    pub step: Option<u64>,
    /// Machine-readable fault class.
    pub code: BridgeFaultCode,
}

/// One typed message on the in-house HIL stream.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum BridgeMessage {
    /// Initial protocol-version and role announcement.
    Hello(BridgeHelloPacket),
    /// Simulator-to-controller sensor frame.
    Sensor(SensorPacket),
    /// Controller-to-simulator actuator command frame.
    Command(ActuatorCommandPacket),
    /// Controller-to-simulator explicit step acknowledgement.
    Ack(StepAckPacket),
    /// Either direction fail-closed fault notification.
    Fault(BridgeFaultPacket),
}
