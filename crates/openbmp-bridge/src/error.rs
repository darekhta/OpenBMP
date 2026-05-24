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
}
