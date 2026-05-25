//! Error types for the simulation kernel.
//!
//! Errors compose: [`SimulationError`] wraps lower-level errors via
//! `#[from]` so the kernel's `step()` and `run()` can use `?` against
//! [`IntegratorError`], [`TimeError`], [`StateError`], and
//! [`ModelEvalError`].

use openbmp_core::{StepIndex, TimeError};
use openbmp_mission::{MissionGraphError, PhaseId};
use openbmp_state::StateError;
use thiserror::Error;

/// Re-export of [`openbmp_models::ModelEvalError`] for back-compat.
///
/// This error lives in `openbmp-models`. The
/// `openbmp_sim::ModelEvalError` path stays valid so existing
/// imports keep compiling.
pub use openbmp_models::ModelEvalError;

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
    /// A wrapped [`ModelEvalError`] raised while validating model
    /// configuration before the integrator is entered.
    #[error(transparent)]
    ModelEval(#[from] ModelEvalError),
    /// A wrapped [`MissionGraphError`] raised while validating the
    /// scenario-declared mission graph.
    #[error(transparent)]
    MissionGraph(#[from] MissionGraphError),
    /// The configuration passed to `SimulationKernel::new` was rejected.
    #[error("invalid simulation configuration: {reason}")]
    InvalidConfig {
        /// Human-readable reason.
        reason: String,
    },
    /// The post-step state failed structural validation after the
    /// kernel assigned canonical time.
    #[error("invalid post-step state at step {step:?}: {source}")]
    InvalidPostStepState {
        /// Step at which the failure was observed.
        step: StepIndex,
        /// Underlying state validation error.
        #[source]
        source: StateError,
    },
    /// A requested rigid-body stage separation could not be applied.
    #[error("invalid rigid-body separation: {reason}")]
    InvalidRigidBodySeparation {
        /// Human-readable reason.
        reason: String,
    },
    /// The floating-point environment was not in the strictly defined
    /// state required by the determinism contract (FTZ / DAZ off,
    /// round-to-nearest-ties-to-even rounding mode).
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
#[derive(Debug, Clone, PartialEq, Error)]
pub enum IntegratorError {
    /// One of the RK4 stages returned a derivative containing `NaN` or
    /// infinity.
    #[error("integrator derivative is not finite (one of the RK4 stages produced NaN/Inf)")]
    NonFiniteDerivative,
    /// A start, intermediate, or integrated state was not valid for
    /// integration.
    #[error("integrator state is not valid for integration")]
    NonFiniteState,
    /// The integration step `dt` was not strictly positive and finite.
    #[error("integrator step is not strictly positive and finite: {dt_seconds} s")]
    InvalidStep {
        /// The offending `dt` in seconds.
        dt_seconds: f64,
    },
    /// A model returned a typed evaluation error mid-step.
    #[error(transparent)]
    ModelEval(#[from] ModelEvalError),
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
    /// A scenario-declared mission stop action fired and ended the run
    /// cleanly. Distinct from [`Self::UserRequested`] so
    /// determinism telemetry can distinguish CLI-driven stops from
    /// scenario-driven mission ends.
    MissionEnded {
        /// Active phase at the time the mission ended, if a mission
        /// graph was wired.
        phase: Option<PhaseId>,
        /// Scenario-declared label for the stop reason.
        label: String,
    },
}

impl StopReason {
    /// Canonical short label, useful for telemetry tags and diagnostics.
    ///
    /// Returns `&str` rather than `&'static str` because
    /// [`Self::MissionEnded`] carries a scenario-declared owned label.
    #[must_use]
    pub fn label(&self) -> &str {
        match self {
            Self::EndTime { .. } => "end-time",
            Self::UserRequested { label } => label,
            Self::NonFiniteState { .. } => "non-finite-state",
            Self::StepOverflow { .. } => "step-overflow",
            Self::MissionEnded { label, .. } => label,
        }
    }
}
