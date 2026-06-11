//! Error types for deterministic contact primitives.

use thiserror::Error;

/// Error returned by contact-model construction and evaluation.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum ContactError {
    /// A required scalar parameter was NaN, infinite, or outside its
    /// admissible range.
    #[error("invalid contact parameter {field}: {reason}")]
    InvalidParameter {
        /// Parameter name.
        field: &'static str,
        /// Rejection reason.
        reason: &'static str,
    },
    /// A vector expected to be finite and non-zero was invalid.
    #[error("invalid contact vector {field}: {reason}")]
    InvalidVector {
        /// Vector field name.
        field: &'static str,
        /// Rejection reason.
        reason: &'static str,
    },
    /// Explicit penalty-contact sub-stepping violates the conservative
    /// stiffness-vs-step bound.
    #[error(
        "contact stability bound violated: substep {substep_s:e} s exceeds {max_substep_s:e} s for natural frequency {natural_frequency_rad_s:e} rad/s"
    )]
    StabilityBoundViolation {
        /// Contact natural frequency from `sqrt(k / m)`.
        natural_frequency_rad_s: f64,
        /// Requested sub-step size.
        substep_s: f64,
        /// Maximum permitted sub-step size.
        max_substep_s: f64,
    },
}

pub(crate) fn require_finite(field: &'static str, value: f64) -> Result<f64, ContactError> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(ContactError::InvalidParameter {
            field,
            reason: "must be finite",
        })
    }
}

pub(crate) fn require_non_negative(field: &'static str, value: f64) -> Result<f64, ContactError> {
    let value = require_finite(field, value)?;
    if value >= 0.0 {
        Ok(value)
    } else {
        Err(ContactError::InvalidParameter {
            field,
            reason: "must be non-negative",
        })
    }
}

pub(crate) fn require_positive(field: &'static str, value: f64) -> Result<f64, ContactError> {
    let value = require_finite(field, value)?;
    if value > 0.0 {
        Ok(value)
    } else {
        Err(ContactError::InvalidParameter {
            field,
            reason: "must be positive",
        })
    }
}
