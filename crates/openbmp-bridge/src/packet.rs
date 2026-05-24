//! Abstract HIL message schema.
//!
//! These types are deliberately transport- and hardware-agnostic.
//! Fields are plain SI scalars and normalized commands; the schema
//! does not reference any concrete sensor product, bus protocol, or
//! flight-software stack.

use serde::{Deserialize, Serialize};

/// Wire protocol version. Both ends compare this and reject a peer
/// that does not match. Bump on any breaking schema change.
pub const PROTOCOL_VERSION: u16 = 1;

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
    /// Magnetic field in the body frame (tesla), when a magnetometer is
    /// modelled.
    pub mag_body_tesla: Option<[f64; 3]>,
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
}
