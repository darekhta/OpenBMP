//! Error type for physics-model evaluation.
//!
//! Designed to convert into the kernel's typed-error chain in
//! `openbmp-sim` / `openbmp-cli` without `openbmp-physics` depending
//! on `openbmp-sim` (the L0/L1 layering rule).

use openbmp_core::FrameError;
use thiserror::Error;

/// Errors raised by physics models in this crate.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum PhysicsError {
    /// A query was outside the model's declared validity envelope.
    #[error("physics model out of envelope: {reason}")]
    OutOfEnvelope {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// A model produced a non-finite output.
    #[error("physics model produced non-finite output: {reason}")]
    NonFinite {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// An invalid model parameter was supplied (e.g., negative `µ`).
    #[error("invalid physics model parameter: {reason}")]
    InvalidParameter {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// A frame-related operation failed.
    #[error(transparent)]
    Frame(#[from] FrameError),
}
