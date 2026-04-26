//! Error type for the openbmp-aero crate.
//!
//! Mirrors the shape of `openbmp-env::EnvError` so kernel-side
//! adapters can fold environment and aerodynamics errors into the
//! same `ModelEvalError` chain at a higher layer without
//! `openbmp-aero` depending on `openbmp-sim`.

use thiserror::Error;

/// Errors raised by aerodynamics models in this crate.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum AeroError {
    /// A query was outside the deck's declared validity envelope and
    /// the deck does not opt into extrapolation.
    #[error("aero deck out of envelope: {reason}")]
    OutOfEnvelope {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// A non-finite (NaN / Inf) input or output was encountered.
    #[error("aero model produced or received non-finite value: {reason}")]
    NonFinite {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// An invalid model parameter was supplied (e.g., negative
    /// reference area).
    #[error("invalid aero model parameter: {reason}")]
    InvalidParameter {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// A deck failed structural validation (non-monotone axis,
    /// inconsistent table length, missing field, schema mismatch).
    #[error("malformed aero deck: {reason}")]
    MalformedDeck {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// A deck file could not be read from disk (path missing,
    /// permission denied, etc.). The contained `String` carries the
    /// path and the underlying I/O error message.
    #[error("aero deck I/O error: {reason}")]
    Io {
        /// Path and underlying I/O error message.
        reason: String,
    },
}
