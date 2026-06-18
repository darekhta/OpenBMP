//! `postcard` encode/decode plus length-prefixed stream framing.

use alloc::vec::Vec;

use serde::Serialize;
use serde::de::DeserializeOwned;

use crate::error::BridgeError;

/// Encode a message to `postcard` bytes.
///
/// # Errors
///
/// Returns [`BridgeError::Encode`] if serialization fails.
pub fn encode<T: Serialize>(message: &T) -> Result<Vec<u8>, BridgeError> {
    postcard::to_allocvec(message).map_err(BridgeError::Encode)
}

/// Encode a message to caller-owned `postcard` bytes.
///
/// Returns the number of bytes written to `output`.
///
/// # Errors
///
/// Returns [`BridgeError::Encode`] if serialization fails or `output`
/// is too small for the encoded message.
pub fn encode_into<T: Serialize>(message: &T, output: &mut [u8]) -> Result<usize, BridgeError> {
    postcard::to_slice(message, output)
        .map(|used| used.len())
        .map_err(BridgeError::Encode)
}

/// Decode a message from `postcard` bytes.
///
/// # Errors
///
/// Returns [`BridgeError::Decode`] if deserialization fails.
pub fn decode<T: DeserializeOwned>(bytes: &[u8]) -> Result<T, BridgeError> {
    postcard::from_bytes(bytes).map_err(BridgeError::Decode)
}

/// Prefix a payload with its `u32` little-endian length for stream
/// transports that need message boundaries.
///
/// HIL control messages are small (a few hundred bytes); a payload at
/// or beyond 4 GiB cannot occur for this schema, so the length is
/// clamped to `u32::MAX` rather than returning a fallible result.
#[must_use]
pub fn frame(payload: &[u8]) -> Vec<u8> {
    let len = u32::try_from(payload.len()).unwrap_or(u32::MAX);
    let mut framed = Vec::with_capacity(4 + payload.len());
    framed.extend_from_slice(&len.to_le_bytes());
    framed.extend_from_slice(payload);
    framed
}

/// Prefix a payload with its `u32` little-endian length into caller-owned
/// storage.
///
/// Returns the number of bytes written.
///
/// # Errors
///
/// Returns [`BridgeError::PayloadTooLarge`] when `payload` cannot fit in
/// the 32-bit frame-length prefix or `output` is too small for the prefix
/// plus payload.
pub fn frame_into(payload: &[u8], output: &mut [u8]) -> Result<usize, BridgeError> {
    let len = u32::try_from(payload.len()).map_err(|_| BridgeError::PayloadTooLarge {
        max: u32::MAX as usize,
        got: payload.len(),
    })?;
    let needed = 4usize
        .checked_add(payload.len())
        .ok_or(BridgeError::PayloadTooLarge {
            max: usize::MAX - 4,
            got: payload.len(),
        })?;
    if output.len() < needed {
        return Err(BridgeError::PayloadTooLarge {
            max: output.len().saturating_sub(4),
            got: payload.len(),
        });
    }
    output[..4].copy_from_slice(&len.to_le_bytes());
    output[4..needed].copy_from_slice(payload);
    Ok(needed)
}

/// Read one length-prefixed frame from the front of `buf`.
///
/// On success returns `(payload, consumed)` where `consumed` is the
/// total bytes read including the 4-byte prefix, so the caller can
/// advance its read cursor.
///
/// # Errors
///
/// Returns [`BridgeError::PrefixIncomplete`] when fewer than four
/// bytes are available, or [`BridgeError::FrameTruncated`] when the
/// declared payload is longer than the remaining buffer.
pub fn deframe(buf: &[u8]) -> Result<(&[u8], usize), BridgeError> {
    if buf.len() < 4 {
        return Err(BridgeError::PrefixIncomplete { have: buf.len() });
    }
    let len = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as usize;
    let available = buf.len() - 4;
    if available < len {
        return Err(BridgeError::FrameTruncated {
            needed: len,
            have: available,
        });
    }
    Ok((&buf[4..4 + len], 4 + len))
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::packet::{ActuatorCommandPacket, ImuIncrementPacket, SensorPacket};

    #[test]
    fn sensor_packet_round_trips() {
        let pkt = SensorPacket {
            sim_time_s: 2.5,
            step: 250,
            imu_accel_body_m_s2: [0.1, -0.2, 9.81],
            imu_gyro_body_rad_s: [0.01, 0.02, -0.03],
            imu_increments: vec![
                ImuIncrementPacket {
                    delta_theta_rad: [1.0e-4, 0.0, -1.0e-4],
                    delta_v_m_s: [0.001, 0.002, 0.003],
                    dt_s: 0.00025,
                    seq: 12,
                },
                ImuIncrementPacket {
                    delta_theta_rad: [2.0e-4, 1.0e-4, 0.0],
                    delta_v_m_s: [0.004, 0.005, 0.006],
                    dt_s: 0.00025,
                    seq: 13,
                },
            ],
            baro_altitude_m: Some(1234.5),
            gnss_position_eci_m: Some([1.0e6, 2.0e6, 3.0e6]),
            gnss_velocity_eci_m_s: None,
            gnss_position_bias_eci_m: Some([1.0, 2.0, 3.0]),
            mag_body_tesla: Some([2.1e-5, 0.0, 4.2e-5]),
            mag_body_nt: Some([21_000.0, 0.0, 42_000.0]),
            mag_hard_iron_body_nt: Some([10.0, -5.0, 2.0]),
            baro_pressure_pa: Some(89_000.0),
            baro_bias_pa: Some(12.0),
            airdata_static_pressure_pa: Some(88_500.0),
            airdata_impact_pressure_pa: Some(1_250.0),
            airdata_mach: Some(0.15),
            airdata_calibrated_airspeed_m_s: Some(51.0),
            airdata_true_airspeed_m_s: Some(52.0),
            airdata_angle_of_attack_rad: Some(0.02),
            airdata_sideslip_rad: Some(-0.01),
            airdata_pressure_altitude_m: Some(1_100.0),
            star_tracker_attitude_eci_to_body_xyzw: Some([0.0, 0.0, 0.0, 1.0]),
        };
        let bytes = encode(&pkt).unwrap();
        let back: SensorPacket = decode(&bytes).unwrap();
        assert_eq!(pkt, back);
    }

    #[test]
    fn command_packet_round_trips() {
        let cmd = ActuatorCommandPacket {
            sim_time_s: 1.0,
            step: 100,
            effector_commands: vec![(0, 0.5), (1, -0.5), (2, 0.0)],
            engine_throttles: vec![(0, 1.0), (1, 0.25)],
            engine_commands: vec![crate::packet::EngineCommandPacket {
                engine_id: 7,
                throttle_unit: 0.8,
                gimbal_pitch_rad: 0.01,
                gimbal_yaw_rad: -0.02,
                ignite: true,
                shutdown: false,
            }],
        };
        let bytes = encode(&cmd).unwrap();
        let back: ActuatorCommandPacket = decode(&bytes).unwrap();
        assert_eq!(cmd, back);
    }

    #[test]
    fn command_packet_encodes_into_caller_buffer() {
        let cmd = ActuatorCommandPacket {
            sim_time_s: 1.0,
            step: 100,
            effector_commands: vec![(0, 0.5), (1, -0.5)],
            engine_throttles: vec![(0, 1.0)],
            engine_commands: vec![crate::packet::EngineCommandPacket {
                engine_id: 7,
                throttle_unit: 0.8,
                gimbal_pitch_rad: 0.01,
                gimbal_yaw_rad: -0.02,
                ignite: true,
                shutdown: false,
            }],
        };

        let mut bytes = [0u8; 256];
        let len = encode_into(&cmd, &mut bytes).unwrap();
        let back: ActuatorCommandPacket = decode(&bytes[..len]).unwrap();

        assert_eq!(cmd, back);
    }

    #[test]
    fn encode_into_reports_small_output_buffer() {
        let cmd = ActuatorCommandPacket {
            sim_time_s: 1.0,
            step: 100,
            effector_commands: vec![(0, 0.5), (1, -0.5)],
            engine_throttles: vec![],
            engine_commands: vec![],
        };
        let mut bytes = [0u8; 1];

        assert!(matches!(
            encode_into(&cmd, &mut bytes),
            Err(BridgeError::Encode(_))
        ));
    }

    #[test]
    fn frame_deframe_recovers_payload() {
        let payload = b"hello-hil";
        let framed = frame(payload);
        let (recovered, consumed) = deframe(&framed).unwrap();
        assert_eq!(recovered, payload);
        assert_eq!(consumed, framed.len());
    }

    #[test]
    fn frame_into_recovers_payload() {
        let payload = b"hello-hil";
        let mut framed = [0u8; 32];
        let written = frame_into(payload, &mut framed).unwrap();
        let (recovered, consumed) = deframe(&framed[..written]).unwrap();

        assert_eq!(recovered, payload);
        assert_eq!(consumed, written);
    }

    #[test]
    fn frame_into_reports_small_output_buffer() {
        let mut framed = [0u8; 5];

        assert!(matches!(
            frame_into(b"payload", &mut framed),
            Err(BridgeError::PayloadTooLarge { max: 1, got: 7 })
        ));
    }

    #[test]
    fn deframe_handles_concatenated_frames() {
        let a = frame(b"first");
        let b = frame(b"second");
        let mut stream = a.clone();
        stream.extend_from_slice(&b);

        let (p1, c1) = deframe(&stream).unwrap();
        assert_eq!(p1, b"first");
        let (p2, c2) = deframe(&stream[c1..]).unwrap();
        assert_eq!(p2, b"second");
        assert_eq!(c1 + c2, stream.len());
    }

    #[test]
    fn deframe_reports_incomplete_prefix() {
        assert!(matches!(
            deframe(&[0u8, 1]),
            Err(BridgeError::PrefixIncomplete { have: 2 })
        ));
    }

    #[test]
    fn deframe_reports_truncated_payload() {
        // Prefix claims 10 bytes but only 2 follow.
        let mut buf = 10u32.to_le_bytes().to_vec();
        buf.extend_from_slice(&[1, 2]);
        assert!(matches!(
            deframe(&buf),
            Err(BridgeError::FrameTruncated {
                needed: 10,
                have: 2
            })
        ));
    }

    #[test]
    fn encoded_message_survives_framing_round_trip() {
        let cmd = ActuatorCommandPacket {
            sim_time_s: 3.0,
            step: 300,
            effector_commands: vec![(7, 0.33)],
            engine_throttles: vec![],
            engine_commands: vec![],
        };
        let framed = frame(&encode(&cmd).unwrap());
        let (payload, _) = deframe(&framed).unwrap();
        let back: ActuatorCommandPacket = decode(payload).unwrap();
        assert_eq!(cmd, back);
    }
}
