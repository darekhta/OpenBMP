//! Error surface for thermochemistry decks.

use thiserror::Error;

/// Typed failures returned by thermochemistry deck validation and lookup.
#[derive(Debug, Error)]
pub enum ThermochemError {
    /// A scalar input, grid point, or table value was NaN or infinite.
    #[error("non-finite thermochemistry value: {reason}")]
    NonFinite {
        /// Human-readable failure reason.
        reason: &'static str,
    },
    /// A parameter was finite but outside the supported envelope.
    #[error("invalid thermochemistry parameter: {reason}")]
    InvalidParameter {
        /// Human-readable failure reason.
        reason: &'static str,
    },
    /// The deck shape or TOML schema is malformed.
    #[error("malformed thermochemistry deck: {reason}")]
    MalformedDeck {
        /// Human-readable failure reason.
        reason: &'static str,
    },
    /// A lookup was requested outside the deck envelope.
    #[error("thermochemistry query outside deck envelope: {reason}")]
    OutOfEnvelope {
        /// Human-readable failure reason.
        reason: &'static str,
    },
    /// TOML parsing failed before semantic validation.
    #[cfg(feature = "parser")]
    #[error("thermochemistry deck TOML parse failed")]
    ParseToml(#[from] toml::de::Error),
}
