//! Error type for the openbmp-propulsion crate.
//!
//! Mirrors the shape of `openbmp-aero::AeroError` and
//! `openbmp-physics::PhysicsError` so the kernel-side adapter at Phase 2.10
//! can fold motor failures into the same chain without
//! `openbmp-propulsion` depending on `openbmp-sim`.

use thiserror::Error;

/// Errors raised by motor models in this crate.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum MotorError {
    /// A query was outside the motor's declared validity envelope
    /// (other than the documented "thrust = 0 outside burn window"
    /// behaviour, which is not an error).
    #[error("motor out of envelope: {reason}")]
    OutOfEnvelope {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// A non-finite (NaN / Inf) input or output was encountered.
    #[error("motor produced or received non-finite value: {reason}")]
    NonFinite {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// An invalid motor parameter was supplied (e.g., negative
    /// propellant mass).
    #[error("invalid motor parameter: {reason}")]
    InvalidParameter {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// A motor file failed structural validation (non-monotone
    /// thrust-curve time grid, inconsistent burn duration, missing
    /// field, schema mismatch).
    #[error("malformed motor: {reason}")]
    MalformedMotor {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// A motor file could not be read from disk.
    #[error("motor file I/O error: {reason}")]
    Io {
        /// Path and underlying I/O error message.
        reason: String,
    },
}

/// Errors raised by [`crate::engine::EngineModel`] implementations
/// and the [`crate::cluster::EngineCluster`] container.
///
/// Sibling type to [`MotorError`]: the two trait families don't
/// intersect, so they keep separate error surfaces. The Phase-3.6
/// runner-side `EngineRack` lifts these into `CliError::Engine`.
///
/// `Eq` is not derived because [`EngineError::InvalidDt`] carries
/// an `f64`; equality semantics on the variant are not meaningful
/// for that case.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum EngineError {
    /// One or more [`crate::engine::EngineLimits`] components are
    /// invalid.
    #[error("engine limits invalid: {reason}")]
    InvalidLimits {
        /// Human-readable reason.
        reason: &'static str,
    },
    /// `step()` was called with a non-positive or non-finite `dt`.
    /// Phase 3 uses fixed-step integration.
    #[error("engine step dt invalid: got {got_s} s")]
    InvalidDt {
        /// Offending `dt` in seconds.
        got_s: f64,
    },
    /// `apply_command()` received a non-finite scalar (throttle or
    /// gimbal angle).
    #[error("engine command is not finite: {reason}")]
    NonFiniteCommand {
        /// Which field tripped the check.
        reason: &'static str,
    },
    /// A fault payload is invalid (e.g. `Stuck { at_throttle }`
    /// outside `[0, 1]`, `OverThrust { factor }` non-finite).
    #[error("engine fault invalid: {reason}")]
    InvalidFault {
        /// Human-readable reason.
        reason: &'static str,
    },
    /// An invalid construction parameter (e.g. negative `isp_s`,
    /// non-finite `max_thrust_n`).
    #[error("invalid engine parameter: {reason}")]
    InvalidParameter {
        /// Short human-readable reason.
        reason: &'static str,
    },
}
