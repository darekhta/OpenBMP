//! Determinism hash helpers for runner-level gates.

use openbmp_bridge::{ActuatorCommandPacket, BridgeError, BridgeMessage, encode, frame};
use openbmp_telemetry::{TelemetryError, TelemetryTable};
use sha2::{Digest, Sha256};

/// Stable digest summary for the FC actuator command stream.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ActuatorStreamReport {
    /// Number of actuator command packets included in the stream digest.
    pub packet_count: usize,
    /// SHA-256 over length-prefixed bridge command messages.
    pub sha256_hex: String,
}

/// Compute SHA-256 over the canonical JSON telemetry representation.
///
/// The telemetry JSON writer fixes row order, channel order, object-key
/// order, and float formatting, so the digest is stable for byte-identical
/// telemetry tables.
///
/// # Errors
///
/// Returns [`TelemetryError`] if canonical telemetry serialization fails.
pub fn telemetry_sha256(table: &TelemetryTable) -> Result<[u8; 32], TelemetryError> {
    let mut bytes = Vec::new();
    table.write_json(&mut bytes)?;
    Ok(Sha256::digest(bytes).into())
}

/// Compute a lowercase hexadecimal SHA-256 digest for canonical telemetry.
///
/// # Errors
///
/// Returns [`TelemetryError`] if canonical telemetry serialization fails.
pub fn telemetry_sha256_hex(table: &TelemetryTable) -> Result<String, TelemetryError> {
    let digest = telemetry_sha256(table)?;
    Ok(hex_digest(&digest))
}

/// Compute SHA-256 over an ordered stream of bridge actuator commands.
///
/// Each command is encoded as the actual `BridgeMessage::Command` postcard
/// payload and length-prefixed with the bridge stream framing before hashing,
/// so the evidence follows the same byte representation used by socket
/// transports.
///
/// # Errors
///
/// Returns [`BridgeError::Encode`] if bridge serialization fails.
pub fn actuator_stream_sha256(packets: &[ActuatorCommandPacket]) -> Result<[u8; 32], BridgeError> {
    let mut hasher = Sha256::new();
    for packet in packets {
        let payload = encode(&BridgeMessage::Command(packet.clone()))?;
        hasher.update(frame(&payload));
    }
    Ok(hasher.finalize().into())
}

/// Compute a lowercase hexadecimal SHA-256 digest for bridge actuator commands.
///
/// # Errors
///
/// Returns [`BridgeError::Encode`] if bridge serialization fails.
pub fn actuator_stream_sha256_hex(
    packets: &[ActuatorCommandPacket],
) -> Result<String, BridgeError> {
    let digest = actuator_stream_sha256(packets)?;
    Ok(hex_digest(&digest))
}

/// Build a compact report for a non-empty actuator command stream.
///
/// # Errors
///
/// Returns [`BridgeError::Encode`] if bridge serialization fails.
pub fn actuator_stream_report(
    packets: &[ActuatorCommandPacket],
) -> Result<Option<ActuatorStreamReport>, BridgeError> {
    if packets.is_empty() {
        return Ok(None);
    }
    Ok(Some(ActuatorStreamReport {
        packet_count: packets.len(),
        sha256_hex: actuator_stream_sha256_hex(packets)?,
    }))
}

fn hex_digest(digest: &[u8; 32]) -> String {
    let mut out = String::with_capacity(digest.len() * 2);
    const HEX: &[u8; 16] = b"0123456789abcdef";
    for byte in *digest {
        out.push(char::from(HEX[usize::from(byte >> 4)]));
        out.push(char::from(HEX[usize::from(byte & 0x0f)]));
    }
    out
}
