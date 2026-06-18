//! Error types for the forward AFTS containment monitor.

use thiserror::Error;

/// Error returned by IIP prediction, containment geometry validation, and
/// rule-table evaluation.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum AftsError {
    /// A scalar parameter was NaN, infinite, or outside its admissible range.
    #[error("invalid AFTS parameter {field}: {reason}")]
    InvalidParameter {
        /// Parameter name.
        field: &'static str,
        /// Rejection reason.
        reason: &'static str,
    },
    /// A vector expected to be finite and non-zero was invalid.
    #[error("invalid AFTS vector {field}: {reason}")]
    InvalidVector {
        /// Vector field name.
        field: &'static str,
        /// Rejection reason.
        reason: &'static str,
    },
    /// A containment polygon did not have enough valid vertices.
    #[error("invalid containment polygon {field}: {reason}")]
    InvalidPolygon {
        /// Polygon field name.
        field: &'static str,
        /// Rejection reason.
        reason: &'static str,
    },
    /// The state did not intersect the configured impact sphere over the
    /// forward propagation horizon.
    #[error("no forward impact within {max_time_s:e} s horizon")]
    NoImpactWithinHorizon {
        /// Maximum forward propagation time.
        max_time_s: f64,
    },
}

pub(crate) fn require_finite(field: &'static str, value: f64) -> Result<f64, AftsError> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(AftsError::InvalidParameter {
            field,
            reason: "must be finite",
        })
    }
}

pub(crate) fn require_positive(field: &'static str, value: f64) -> Result<f64, AftsError> {
    let value = require_finite(field, value)?;
    if value > 0.0 {
        Ok(value)
    } else {
        Err(AftsError::InvalidParameter {
            field,
            reason: "must be positive",
        })
    }
}

pub(crate) fn require_non_negative(field: &'static str, value: f64) -> Result<f64, AftsError> {
    let value = require_finite(field, value)?;
    if value >= 0.0 {
        Ok(value)
    } else {
        Err(AftsError::InvalidParameter {
            field,
            reason: "must be non-negative",
        })
    }
}
