//! Error surface for feed-system network primitives.

use thiserror::Error;

/// Typed failures returned by feed-system validation and solves.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum FeedSystemError {
    /// A scalar input or computed state was NaN or infinite.
    #[error("non-finite feed-system value: {reason}")]
    NonFinite {
        /// Human-readable failure reason.
        reason: &'static str,
    },
    /// A finite scalar was outside the supported envelope.
    #[error("invalid feed-system parameter: {reason}")]
    InvalidParameter {
        /// Human-readable failure reason.
        reason: &'static str,
    },
    /// The algebraic network solve did not converge inside its fixed cap.
    #[error("feed-system solve did not converge: {reason}")]
    NonConverged {
        /// Human-readable failure reason.
        reason: &'static str,
    },
}
