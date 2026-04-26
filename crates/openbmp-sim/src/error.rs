//! Error types for the simulation kernel.
//!
//! Errors compose: [`SimulationError`] wraps lower-level errors via
//! `#[from]` so the kernel's `step()` and `run()` can use `?` against
//! [`IntegratorError`], [`TimeError`], and [`StateError`].

use openbmp_core::{StepIndex, TimeError};
use openbmp_state::StateError;
use thiserror::Error;

/// Aggregated error type returned by [`crate::SimulationKernel`] methods.
#[derive(Debug, Error)]
pub enum SimulationError {
    /// A wrapped [`TimeError`] from `openbmp-core`.
    #[error(transparent)]
    Time(#[from] TimeError),
    /// A wrapped [`StateError`] from `openbmp-state`.
    #[error(transparent)]
    State(#[from] StateError),
    /// A wrapped [`IntegratorError`] from this crate's integrator.
    #[error(transparent)]
    Integrator(#[from] IntegratorError),
    /// The configuration passed to `SimulationKernel::new` was rejected.
    #[error("invalid simulation configuration: {reason}")]
    InvalidConfig {
        /// Human-readable reason.
        reason: String,
    },
    /// The post-step state contained `NaN` or infinite components, or
    /// failed `is_finite()` (which for `PointMassState` also requires
    /// strictly-positive mass).
    #[error("non-finite or invalid state at step {step:?}")]
    NonFiniteState {
        /// Step at which the failure was observed.
        step: StepIndex,
    },
    /// The floating-point environment was not in the strictly defined
    /// state required by the determinism contract (FTZ / DAZ flushed
    /// to zero, round-to-nearest-ties-to-even rounding mode).
    #[error(
        "floating-point environment is dirty (ftz={ftz}, daz={daz}, \
         rounding_mode={rounding_mode}); cannot guarantee bit-stable replay"
    )]
    FpEnvironmentDirty {
        /// FTZ (flush-to-zero) flag observed in MXCSR.
        ftz: bool,
        /// DAZ (denormals-are-zero) flag observed in MXCSR.
        daz: bool,
        /// MXCSR rounding-mode bits (00 = round-to-nearest-even).
        rounding_mode: u32,
    },
}

/// Errors produced by an [`crate::Integrator`] implementation.
#[derive(Debug, Clone, Copy, PartialEq, Error)]
pub enum IntegratorError {
    /// One of the RK4 stages returned a derivative containing `NaN` or
    /// infinity.
    #[error("integrator derivative is not finite (one of the RK4 stages produced NaN/Inf)")]
    NonFiniteDerivative,
    /// The integrated state contained non-finite components after
    /// summation.
    #[error("integrated state is not finite")]
    NonFiniteState,
    /// The integration step `dt` was not strictly positive and finite.
    #[error("integrator step is not strictly positive and finite: {dt_seconds} s")]
    InvalidStep {
        /// The offending `dt` in seconds.
        dt_seconds: f64,
    },
}

/// Reason a simulation run terminated.
#[derive(Debug, Clone, PartialEq)]
pub enum StopReason {
    /// Reached or exceeded the configured end time.
    EndTime {
        /// Simulation time at which the stop fired.
        reached_s: f64,
    },
    /// A user-supplied stop condition fired with the given label.
    UserRequested {
        /// Static label describing the user-defined stop trigger.
        label: &'static str,
    },
    /// The kernel observed a non-finite state and aborted.
    NonFiniteState {
        /// Step at which the failure was observed.
        step: StepIndex,
    },
    /// `StepIndex::checked_next` overflowed.
    StepOverflow {
        /// The step that could not advance.
        step: StepIndex,
    },
}

impl StopReason {
    /// Canonical short label, useful for telemetry tags and diagnostics.
    #[must_use]
    pub const fn label(&self) -> &'static str {
        match self {
            Self::EndTime { .. } => "end-time",
            Self::UserRequested { label } => label,
            Self::NonFiniteState { .. } => "non-finite-state",
            Self::StepOverflow { .. } => "step-overflow",
        }
    }
}
