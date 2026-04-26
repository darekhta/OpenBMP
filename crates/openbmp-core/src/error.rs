//! Project-wide error types for `openbmp-core`.
//!
//! Higher-level crates compose these via `#[from]` on their own error
//! types so every error keeps a structured, displayable cause.

use crate::frames::FrameId;
use thiserror::Error;

/// Aggregated error type for `openbmp-core` operations.
#[derive(Error, Debug, Clone, PartialEq)]
pub enum CoreError {
    /// A time-related operation produced an invalid value.
    #[error(transparent)]
    Time(#[from] TimeError),
    /// A frame-related operation produced an invalid value.
    #[error(transparent)]
    Frame(#[from] FrameError),
}

/// Errors produced by time primitives in [`crate::time`].
#[derive(Error, Debug, Clone, PartialEq)]
pub enum TimeError {
    /// A time or duration value was `NaN` or infinite.
    #[error("time value is not finite: {value}")]
    NotFinite {
        /// The offending raw value in seconds.
        value: f64,
    },
    /// A monotonicity invariant was violated.
    #[error("time would go backward: prior={prior_s} s, next={next_s} s")]
    NotMonotonic {
        /// The prior `SimTime` in seconds.
        prior_s: f64,
        /// The would-be-next `SimTime` in seconds.
        next_s: f64,
    },
    /// A duration was expected to be strictly positive.
    #[error("duration is not strictly positive: {seconds} s")]
    NotPositive {
        /// The offending duration in seconds.
        seconds: f64,
    },
}

/// Errors produced by frame operations in [`crate::frames`].
#[derive(Error, Debug, Clone, PartialEq)]
pub enum FrameError {
    /// A vector or quaternion contained `NaN` or infinite components.
    #[error("frame value is not finite (frame {frame:?})")]
    NotFinite {
        /// The frame in which the offending value was expressed.
        frame: FrameId,
    },
    /// A quaternion was not normalised within tolerance.
    #[error("quaternion is not normalised: |q| = {magnitude}, tolerance = {tolerance}")]
    NotNormalised {
        /// The actual magnitude.
        magnitude: f64,
        /// The acceptable tolerance.
        tolerance: f64,
    },
    /// A required transform is not implemented in the active
    /// [`crate::frames::FrameProfile`].
    #[error("transform from {from:?} to {to:?} is not available in profile {profile}")]
    TransformNotAvailable {
        /// Source frame.
        from: FrameId,
        /// Target frame.
        to: FrameId,
        /// Active profile name.
        profile: &'static str,
    },
}
