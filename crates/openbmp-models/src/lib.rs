//! `openbmp-models` — hardware-portable model trait surfaces.
//!
//! Lives apart from `openbmp-sim` so any code that
//! implements or consumes the model trait surfaces (`ForceModel`,
//! `MomentModel`, `MassModel`, `RigidMassModel`, `EnvironmentModel`)
//! does not transitively depend on the simulator's integrator,
//! kernel, scheduler, or stop conditions. Hardware adopters can
//! build a HAL on these abstractions without pulling in the
//! simulation kernel.
//!
//! # Module surface
//!
//! - [`VehicleState`], [`TranslationalState`],
//!   [`RigidBodyKinematicState`], [`Integratable`], [`SimState`], and
//!   [`SimStateDerivative`] — hardware-portable state snapshot and
//!   integration traits. Implemented for `openbmp_state::PointMassState`
//!   and `openbmp_state::RigidBodyState`.
//! - [`ForceModel`], [`MomentModel`], [`MassModel`],
//!   [`RigidMassModel`], [`EnvironmentModel`] — model trait surfaces.
//! - [`ForceContext`], [`MomentContext`], [`MassContext`],
//!   [`EnvironmentQuery`], [`EnvironmentSample`] — per-step context
//!   threaded into model evaluation.
//! - [`EffectorActualsView`], [`EngineSnapshot`],
//!   [`EngineSnapshotView`],
//!   [`TankSnapshotView`], [`RecoverySnapshotView`] — read-only
//!   borrow-views the kernel publishes to its rack adapters; the
//!   shapes are general-purpose enough that a HAL adopter could
//!   implement the same publication path from real hardware
//!   actuators / engines / tanks / recovery devices.
//! - [`ModelEvalError`] — typed evaluation error returned by every
//!   fallible model.
//! - Simple impls: [`ConstantGravityForce`], [`ZeroForce`],
//!   [`ZeroMoment`], [`ConstantMass`], [`LinearBurnMass`],
//!   [`ConstantMassRigid`], [`LinearBurnMassRigid`],
//!   [`NullEnvironment`].
//!
//! `openbmp-sim` re-exports every item below for back-compatibility.

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

extern crate alloc;

pub mod derivative;
pub mod error;
pub mod models;
pub mod port;
pub mod state;

pub use derivative::{PointMassDerivative, RigidBodyDerivative, SimStateDerivative};
pub use error::ModelEvalError;
pub use models::{
    ConstantGravityForce, ConstantMass, ConstantMassRigid, EffectorActualsView, EngineSnapshot,
    EngineSnapshotView, EnvironmentModel, EnvironmentQuery, EnvironmentSample, ForceContext,
    ForceModel, LinearBurnMass, LinearBurnMassRigid, MassContext, MassModel, MassPropertiesRate,
    MomentContext, MomentModel, NullEnvironment, PhaseGatedForceModel, RecoverySnapshot,
    RecoverySnapshotView, RigidMassModel, TankSnapshot, TankSnapshotView, ZeroForce, ZeroMoment,
};
pub use port::{
    FmiClockIntervalVariability, FmiClockVariable, FmiClockedVariable, FmiScalarVariable,
    FmiVariableCausality, FmiVariableType, FmuCoSimulationBackend, FmuCoSimulationModelPort,
    FmuCoSimulationPortSpec, ModelPort, ModelPortKind, ModelPortMetadata, NativeModelPort,
};
#[cfg(feature = "std")]
pub use port::{FmuArchive, FmuArchiveError, FmuModelDescription};
pub use state::{
    Integratable, RigidBodyKinematicState, SimState, TranslationalState, VehicleState,
};
