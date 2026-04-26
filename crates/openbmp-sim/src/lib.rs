//! `openbmp-sim` — OpenBMP lockstep simulation kernel.
//!
//! Phase 1.3: a deterministic, fixed-step Runge-Kutta 4 kernel for
//! [`openbmp_state::PointMassState`] under user-supplied force, mass,
//! environment, and stop-condition models. Rigid-body integration
//! (with quaternion attitude) is deferred to a follow-on sub-phase.
//!
//! # Architecture
//!
//! * [`SimState`] — trait implemented by states an integrator can
//!   advance. Phase 1.3 implements it for `PointMassState`.
//! * [`SimStateDerivative`] — trait implemented by the time-derivative
//!   of a state. Carries the canonical RK4 weighted-sum with locked
//!   evaluation order.
//! * [`Integrator`] — trait for numerical integrators. Phase 1.3 ships
//!   [`Rk4FixedStep`].
//! * [`ForceModel`], [`MassModel`], [`EnvironmentModel`] — model
//!   trait surfaces. Phase 1.3 ships [`ConstantGravityForce`],
//!   [`ZeroForce`], [`ConstantMass`], [`LinearBurnMass`], and
//!   [`NullEnvironment`].
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
//! See `docs/phase-1-plan.md § 1.3` and
//! `docs/software-architecture.md § Determinism Profile`.

pub mod derivative;
pub mod error;
pub mod integrator;
pub mod kernel;
pub mod models;
pub mod stop;

pub use derivative::{PointMassDerivative, SimStateDerivative};
pub use error::{IntegratorError, SimulationError, StopReason};
pub use integrator::{Integrator, IntegratorDeterminism, Rk4FixedStep, SimState};
pub use kernel::{SimulationConfig, SimulationKernel};
pub use models::{
    ConstantGravityForce, ConstantMass, EnvironmentModel, EnvironmentQuery, EnvironmentSample,
    ForceContext, ForceModel, LinearBurnMass, MassModel, NullEnvironment, ZeroForce,
};
pub use stop::{AlwaysContinue, EndTime, MaxSteps, StopCondition};
