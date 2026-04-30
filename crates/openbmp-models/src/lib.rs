//! `openbmp-models` — hardware-portable model trait surfaces.
//!
//! Phase-3.14.A: extracted from `openbmp-sim` so any code that
//! implements or consumes the model trait surfaces (`ForceModel`,
//! `MomentModel`, `MassModel`, `RigidMassModel`, `EnvironmentModel`)
//! does not transitively depend on the simulator's integrator,
//! kernel, scheduler, or stop conditions. Hardware adopters can
//! build a HAL on these abstractions without pulling in the
//! simulation kernel.
//!
//! # Module surface
//!
//! - [`SimState`] / [`SimStateDerivative`] — the data shape an
//!   integrator (anywhere) advances. Implemented for
//!   `openbmp_state::PointMassState` and
//!   `openbmp_state::RigidBodyState`.
//! - [`ForceModel`], [`MomentModel`], [`MassModel`],
//!   [`RigidMassModel`], [`EnvironmentModel`] — model trait surfaces.
//! - [`ForceContext`], [`MomentContext`], [`MassContext`],
//!   [`EnvironmentQuery`], [`EnvironmentSample`] — per-step context
//!   threaded into model evaluation.
//! - [`EffectorActualsView`], [`EngineSnapshotView`],
//!   [`TankSnapshotView`], [`RecoverySnapshotView`] — read-only
//!   borrow-views the kernel publishes to its rack adapters; the
//!   shapes are general-purpose enough that a HAL adopter could
//!   implement the same publication path from real hardware
//!   actuators / engines / tanks / recovery devices.
//! - [`ModelEvalError`] — typed evaluation error returned by every
//!   fallible model.
//! - Phase-1 simple impls: [`ConstantGravityForce`], [`ZeroForce`],
//!   [`ZeroMoment`], [`ConstantMass`], [`LinearBurnMass`],
//!   [`ConstantMassRigid`], [`LinearBurnMassRigid`],
//!   [`NullEnvironment`].
//!
//! `openbmp-sim` re-exports every item below for back-compatibility
//! during the Phase-3.14 transition.

pub mod derivative;
pub mod error;
pub mod models;
pub mod state;

pub use derivative::{PointMassDerivative, RigidBodyDerivative, SimStateDerivative};
pub use error::ModelEvalError;
pub use models::{
    ConstantGravityForce, ConstantMass, ConstantMassRigid, EffectorActualsView, EngineSnapshotView,
    EnvironmentModel, EnvironmentQuery, EnvironmentSample, ForceContext, ForceModel,
    LinearBurnMass, LinearBurnMassRigid, MassContext, MassModel, MassPropertiesRate, MomentContext,
    MomentModel, NullEnvironment, RecoverySnapshot, RecoverySnapshotView, RigidMassModel,
    TankSnapshot, TankSnapshotView, ZeroForce, ZeroMoment,
};
pub use state::{Integratable, SimState, VehicleState};
