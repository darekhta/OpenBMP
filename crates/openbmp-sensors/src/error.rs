//! Error type for the openbmp-sensors crate.
//!
//! Mirrors the shape of `openbmp-aero::AeroError` and
//! `openbmp-propulsion::MotorError` so the kernel-side adapter at
//! Phase 2.10 can fold sensor failures into the same chain without
//! `openbmp-sensors` depending on `openbmp-sim`.

use thiserror::Error;

/// Errors raised by sensor models in this crate.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum SensorError {
    /// A non-finite (NaN / Inf) input or output was encountered.
    #[error("sensor produced or received non-finite value: {reason}")]
    NonFinite {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// An invalid sensor parameter was supplied (e.g., negative noise
    /// standard deviation, non-positive correlation time).
    #[error("invalid sensor parameter: {reason}")]
    InvalidParameter {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// A noise-budget file failed structural validation.
    #[error("malformed sensor noise budget: {reason}")]
    MalformedBudget {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// A noise-budget file could not be read from disk.
    #[error("sensor budget file I/O error: {reason}")]
    Io {
        /// Path and underlying I/O error message.
        reason: String,
    },
    /// The controller called [`crate::Sensor::read`] but no sample is
    /// available yet (typical when the controller polls faster than
    /// the sensor's native cadence, or when a synthetic sensor's
    /// runner-pushed truth port has not been primed).
    #[error("no sensor sample available")]
    NoSample,
}
