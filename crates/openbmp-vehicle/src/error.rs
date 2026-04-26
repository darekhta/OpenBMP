//! Error type for the openbmp-vehicle crate.

use thiserror::Error;

/// Errors raised when constructing or composing a [`crate::Vehicle`].
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum VehicleError {
    /// A vehicle was constructed with an invalid parameter (e.g.,
    /// duplicated model name within a single force / moment list).
    #[error("invalid vehicle parameter: {reason}")]
    InvalidParameter {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// A vehicle was constructed in a structurally inconsistent way
    /// (e.g., empty model name).
    #[error("malformed vehicle: {reason}")]
    MalformedVehicle {
        /// Short human-readable reason.
        reason: &'static str,
    },
}
