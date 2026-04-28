//! `openbmp-sim` — OpenBMP lockstep simulation kernel.
//!
//! Phase 2.1: a deterministic, fixed-step Runge-Kutta 4 kernel for
//! [`openbmp_state::PointMassState`] and [`openbmp_state::RigidBodyState`]
//! under user-supplied force, moment, mass, environment, and
//! stop-condition models.
//!
//! # Architecture
//!
//! * [`SimState`] — trait implemented by states an integrator can
//!   advance. Implemented for `PointMassState` and `RigidBodyState`.
//! * [`SimStateDerivative`] — trait implemented by the time-derivative
//!   of a state. Carries the canonical RK4 weighted-sum with locked
//!   evaluation order.
//! * [`Integrator`] — trait for numerical integrators. Phase 1.3 ships
//!   [`Rk4FixedStep`].
//! * [`ForceModel`], [`MomentModel`], [`MassModel`], [`RigidMassModel`],
//!   [`EnvironmentModel`] — model trait surfaces. The crate ships
//!   [`ConstantGravityForce`], [`ZeroForce`], [`ZeroMoment`],
//!   [`ConstantMass`], [`LinearBurnMass`], [`ConstantMassRigid`],
//!   [`LinearBurnMassRigid`], and [`NullEnvironment`].
//! * [`StopCondition`] — trait for halt-the-kernel predicates. Phase
//!   1.3 ships [`AlwaysContinue`], [`EndTime`], [`MaxSteps`].
//! * [`SimulationKernel`] — owns the integrator + models + stop
//!   condition + state. Drives the step loop.
//!
//! # Determinism contract
//!
//! Every layer in this crate respects the OpenBMP determinism contract:
//!
//! * No wall-clock time, no system RNG, no network, no file I/O.
//! * Single-threaded, synchronous; no `tokio` / threads / atomics.
//! * Locked weighted-sum order in the RK4 stage combination.
//! * Time advances by canonical `start + step * dt` multiplication
//!   (not accumulation) to avoid O(N · ε) drift.
//! * MXCSR (FTZ / DAZ / rounding mode) is asserted clean at kernel
//!   construction on x86_64.
//! * No `f64::mul_add` anywhere on the hot path.
//! * Tracing-emitted bytes are not part of deterministic output; the
//!   determinism CI gate verifies this.
//!
//! See `docs/software-architecture.md § Simulation Kernel` and
//! `docs/software-architecture.md § Determinism Profile`.

pub mod derivative;
pub mod error;
pub mod events;
pub mod integrator;
pub mod kernel;
pub mod models;
pub mod stop;

pub use derivative::{PointMassDerivative, RigidBodyDerivative, SimStateDerivative};
pub use error::{IntegratorError, ModelEvalError, SimulationError, StopReason};
pub use events::{
    BuiltInEventTrigger, EventAction, EventBinding, EventEvalState, EventId, EventScalars,
    EventTrigger, FiredEvent, MissionGraphError, MissionPhaseGraph, Phase, PhaseId,
    PhaseTransition,
};
pub use integrator::{Integrator, IntegratorDeterminism, Rk4FixedStep, SimState};
pub use kernel::{Phase1Kernel, RigidBodyKernel, RigidModels, SimulationConfig, SimulationKernel};
pub use models::{
    ConstantGravityForce, ConstantMass, ConstantMassRigid, EffectorActualsView, EnvironmentModel,
    EnvironmentQuery, EnvironmentSample, ForceContext, ForceModel, LinearBurnMass,
    LinearBurnMassRigid, MassModel, MassPropertiesRate, MomentContext, MomentModel,
    NullEnvironment, RigidMassModel, ZeroForce, ZeroMoment,
};
pub use stop::{AlwaysContinue, EndTime, MaxSteps, StopCondition};
