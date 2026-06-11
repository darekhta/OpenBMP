//! Bridge codec, lockstep, and transport error type.

use thiserror::Error;

use crate::packet::BridgeEndpointRole;

/// Errors raised while encoding, decoding, framing, validating, or
/// transporting bridge messages.
#[derive(Debug, Error)]
pub enum BridgeError {
    /// `postcard` failed to serialize a message.
    #[error("bridge encode failed: {0}")]
    Encode(postcard::Error),
    /// `postcard` failed to deserialize a message.
    #[error("bridge decode failed: {0}")]
    Decode(postcard::Error),
    /// A length-prefixed frame declared a payload longer than the
    /// buffer holds (the caller should read more bytes and retry).
    #[error("bridge frame truncated: need {needed} bytes, have {have}")]
    FrameTruncated {
        /// Bytes the frame's length prefix requires.
        needed: usize,
        /// Bytes currently available after the prefix.
        have: usize,
    },
    /// A frame's length prefix could not be read (fewer than four
    /// bytes available).
    #[error("bridge frame prefix incomplete: have {have} of 4 bytes")]
    PrefixIncomplete {
        /// Bytes currently available.
        have: usize,
    },
    /// A framed bridge payload exceeded the transport's configured limit.
    #[error("bridge payload too large: max {max} bytes, got {got}")]
    PayloadTooLarge {
        /// Maximum payload bytes accepted by the transport.
        max: usize,
        /// Payload bytes declared or produced by the peer.
        got: usize,
    },
    /// The peer closed the transport before a complete message arrived.
    #[error("bridge transport closed")]
    TransportClosed,
    /// The underlying stream transport returned an I/O error.
    #[error("bridge transport I/O failed: {0}")]
    Io(#[from] std::io::Error),
    /// A peer sent a different message kind than the current lockstep
    /// phase permits.
    #[error("bridge unexpected message: expected {expected}, got {got}")]
    UnexpectedMessage {
        /// Message kind expected by the lockstep phase.
        expected: &'static str,
        /// Message kind received from the peer.
        got: &'static str,
    },
    /// A peer advertised a role that cannot satisfy this side of the
    /// lockstep exchange.
    #[error("bridge endpoint role mismatch: expected {expected:?}, got {got:?}")]
    RoleMismatch {
        /// Endpoint role expected by this side.
        expected: BridgeEndpointRole,
        /// Endpoint role advertised by the peer.
        got: BridgeEndpointRole,
    },
    /// Peer reported an incompatible wire protocol version.
    #[error("bridge protocol mismatch: expected version {expected}, got {got}")]
    ProtocolVersionMismatch {
        /// Version supported by this crate.
        expected: u16,
        /// Version reported by the peer.
        got: u16,
    },
    /// A lockstep response targeted a different kernel step.
    #[error("bridge lockstep step mismatch: expected step {expected}, got {got}")]
    StepMismatch {
        /// Step emitted by the simulator sensor frame.
        expected: u64,
        /// Step reported by the response packet.
        got: u64,
    },
    /// A lockstep response targeted a different simulation timestamp.
    #[error("bridge lockstep time mismatch: expected {expected:.9}s, got {got:.9}s")]
    TimeMismatch {
        /// Simulation timestamp emitted by the sensor frame.
        expected: f64,
        /// Simulation timestamp reported by the response packet.
        got: f64,
    },
}
