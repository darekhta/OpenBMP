//! Error type for the openbmp-env crate.
//!
//! Designed to convert into `openbmp_sim::ModelEvalError` via the
//! conversion below so higher-layer kernels can adapt
//! environment-model failures into the kernel's typed-error chain
//! without `openbmp-env` depending on `openbmp-sim` (the L1/L2
//! layering rule, see `docs/phase-2-plan.md § Implementation Seams`).

use openbmp_core::FrameError;
use thiserror::Error;

/// Errors raised by environment models in this crate.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum EnvError {
    /// A query was outside the model's declared validity envelope.
    #[error("environment model out of envelope: {reason}")]
    OutOfEnvelope {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// A model produced a non-finite output.
    #[error("environment model produced non-finite output: {reason}")]
    NonFinite {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// An invalid model parameter was supplied (e.g., negative `µ`).
    #[error("invalid environment model parameter: {reason}")]
    InvalidParameter {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// A frame-related operation failed.
    #[error(transparent)]
    Frame(#[from] FrameError),
}
