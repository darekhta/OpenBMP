//! Bridge codec error type.

use thiserror::Error;

/// Errors raised while encoding, decoding, or framing bridge messages.
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
