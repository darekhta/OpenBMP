//! Model trait declarations + simple impls — re-exported from
//! `openbmp-models`.
//!
//! The model trait surface lives in `openbmp-models` so
//! model authors and HAL adopters can implement / consume them
//! without depending on the simulator. This module is now a thin
//! re-export to preserve every existing `openbmp_sim::models::*`
//! import path.

pub use openbmp_models::models::{
    ConstantGravityForce, ConstantMass, ConstantMassRigid, EffectorActualsView, EngineSnapshotView,
    EnvironmentModel, EnvironmentQuery, EnvironmentSample, ForceContext, ForceModel,
    LinearBurnMass, LinearBurnMassRigid, MassContext, MassModel, MassPropertiesRate, MomentContext,
    MomentModel, NullEnvironment, RecoverySnapshot, RecoverySnapshotView, RigidMassModel,
    TankSnapshot, TankSnapshotView, ZeroForce, ZeroMoment,
};
