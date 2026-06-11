//! Error surface for plume similarity primitives.

use thiserror::Error;

/// Typed failures returned by plume similarity validation.
#[derive(Clone, Copy, Debug, Error, Eq, PartialEq)]
pub enum PlumeError {
    /// A scalar input was NaN or infinite.
    #[error("non-finite plume value: {field}")]
    NonFinite {
        /// Invalid field name.
        field: &'static str,
    },
    /// A scalar input was finite but outside the supported envelope.
    #[error("invalid plume value: {field} must {rule}")]
    InvalidParameter {
        /// Invalid field name.
        field: &'static str,
        /// Required rule.
        rule: &'static str,
    },
}
