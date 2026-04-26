//! Error type for the openbmp-propulsion crate.
//!
//! Mirrors the shape of `openbmp-aero::AeroError` and
//! `openbmp-env::EnvError` so the kernel-side adapter at Phase 2.10
//! can fold motor failures into the same chain without
//! `openbmp-propulsion` depending on `openbmp-sim`.

use thiserror::Error;

/// Errors raised by motor models in this crate.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum MotorError {
    /// A query was outside the motor's declared validity envelope
    /// (other than the documented "thrust = 0 outside burn window"
    /// behaviour, which is not an error).
    #[error("motor out of envelope: {reason}")]
    OutOfEnvelope {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// A non-finite (NaN / Inf) input or output was encountered.
    #[error("motor produced or received non-finite value: {reason}")]
    NonFinite {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// An invalid motor parameter was supplied (e.g., negative
    /// propellant mass).
    #[error("invalid motor parameter: {reason}")]
    InvalidParameter {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// A motor file failed structural validation (non-monotone
    /// thrust-curve time grid, inconsistent burn duration, missing
    /// field, schema mismatch).
    #[error("malformed motor: {reason}")]
    MalformedMotor {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// A motor file could not be read from disk.
    #[error("motor file I/O error: {reason}")]
    Io {
        /// Path and underlying I/O error message.
        reason: String,
    },
}
