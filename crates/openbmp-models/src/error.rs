//! Model-evaluation error type.
//!
//! Phase-3.14.A: extracted from `openbmp-sim` so model trait surfaces
//! (`ForceModel`, `MomentModel`, `MassModel`, `RigidMassModel`,
//! `EnvironmentModel`) can be implemented without depending on the
//! simulator. The simulator's broader `SimulationError` family remains
//! in `openbmp-sim`.

use std::borrow::Cow;

use openbmp_core::ModelId;
use thiserror::Error;

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
/// See `docs/software-architecture.md § Model Interfaces` for the
/// contract.
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
