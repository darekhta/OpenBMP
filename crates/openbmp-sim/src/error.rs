//! Error types for the simulation kernel.
//!
//! Errors compose: [`SimulationError`] wraps lower-level errors via
//! `#[from]` so the kernel's `step()` and `run()` can use `?` against
//! [`IntegratorError`], [`TimeError`], [`StateError`], and
//! [`ModelEvalError`].

use std::borrow::Cow;

use openbmp_core::{ModelId, StepIndex, TimeError};
use openbmp_state::StateError;
use thiserror::Error;

use crate::events::{MissionGraphError, PhaseId};

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

/// Typed evaluation error returned by a fallible model
/// (`EnvironmentModel`, `ForceModel`, `MomentModel`, `MassModel`,
/// `AeroDeck`, `Motor`, etc.) when its inputs leave its validity
/// envelope.
///
/// Carries the `ModelId` of the offending model so the kernel can
/// surface `(step_index, model_id, error)` to the user without a
/// silent clamp / NaN / panic. Phase-2 models *must* return one of
/// these variants instead of a panic or `NaN`.
///
/// See `docs/phase-2-plan.md § Implementation Seams Locked Before
/// Coding` and `docs/software-architecture.md § Model Interfaces` for
/// the contract.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum ModelEvalError {
    /// The model's inputs were outside its declared validity envelope
    /// (e.g. atmosphere queried above its altitude ceiling, aero deck
    /// queried outside its (Mach, alpha, beta) grid, motor queried
    /// before ignition).
    #[error("model {model:?} out of envelope: {reason}")]
    OutOfEnvelope {
        /// Offending model's id.
        model: ModelId,
        /// Short human-readable reason.
        reason: Cow<'static, str>,
    },
    /// The model produced a non-finite output (`NaN` or `Inf`).
    #[error("model {model:?} produced non-finite output")]
    NonFinite {
        /// Offending model's id.
        model: ModelId,
    },
    /// The state passed to the model was structurally invalid for
    /// the model's purposes.
    #[error("model {model:?} received invalid state: {reason}")]
    InvalidState {
        /// Offending model's id.
        model: ModelId,
        /// Short human-readable reason.
        reason: Cow<'static, str>,
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
    /// A scenario-declared `EventAction::Stop` fired and ended the
    /// run cleanly. Distinct from [`Self::UserRequested`] so
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
