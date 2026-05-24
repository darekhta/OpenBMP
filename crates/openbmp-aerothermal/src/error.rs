//! Typed error surface for the aerothermal crate.

use thiserror::Error;

/// Errors raised by aerothermal model evaluation.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum AerothermalError {
    /// Out-of-envelope query (e.g. negative density, sub-sonic regime
    /// for a hypersonic-only correlation, etc.).
    #[error("aerothermal model out of envelope: {reason}")]
    OutOfEnvelope {
        /// Short reason.
        reason: &'static str,
    },
    /// Model evaluation produced a non-finite result.
    #[error("aerothermal model produced non-finite output: {reason}")]
    NonFinite {
        /// Short reason.
        reason: &'static str,
    },
    /// Invalid configuration parameter at model construction.
    #[error("aerothermal model invalid parameter: {reason}")]
    InvalidParameter {
        /// Short reason.
        reason: &'static str,
    },
}
