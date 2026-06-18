//! `openbmp-vehicle` — OpenBMP vehicle composition.
//!
//! Contents:
//!
//! * [`vehicle::Vehicle`] trait — the architecture's
//!   force / moment / mass composition surface, parameterised over
//!   `S: SimState` so it serves both point-mass and rigid-body
//!   kernels.
//! * [`vehicle::KernelVehicle`] — the implementation that
//!   composes ordered force-model and moment-model lists and carries a
//!   single mass model.
//!   `KernelVehicle::force_n_eci` evaluates the list in declared
//!   order with locked operand sum and short-circuits on the first
//!   model error. The contract is *order matters* — floating-point
//!   summation is not associative,
//!   so reordering the list changes the byte output.
//! * [`vehicle::ForceBreakdown`] / [`vehicle::MomentBreakdown`] —
//!   per-model components plus total. The kernel-side adapter
//!   evaluates the breakdown once per step, uses its total
//!   for dynamics, and publishes `force.<name>.{x,y,z}` telemetry
//!   channels.
//! * [`vehicle::BoxedMassModel`] — convenience wrapper around
//!   `Box<dyn MassModel>` for the vehicle-owned mass model.
//! * [`error::VehicleError`].
//!
//! # Byte-stability preservation
//!
//! The analytic toy gravity path remains byte-identical when
//! the single gravity force is wrapped in a one-element `KernelVehicle`;
//! the `single_force_model_vehicle_byte_matches_raw_model` test asserts
//! the direct model result, and the kernel integration test asserts the
//! final state. The `analytic_toy` regression continues to use
//! the kernel's existing generic `F: ForceModel<PointMassState>`
//! surface, so swapping in `KernelVehicle` is an opt-in.
//!
//! # Determinism
//!
//! Pure arithmetic on `f64`; locked operand order on the summation
//! left fold; no FMA, no wall-clock, no system RNG, no network, no
//! file I/O.
//!
//! # Crate layering
//!
//! `openbmp-vehicle` is an L1 crate alongside `openbmp-sim`. Both
//! own kernel-facing trait surfaces (`ForceModel`, `MomentModel`,
//! `MassModel`, `Vehicle`); L2 crates (`openbmp-physics`,
//! `openbmp-aero`, `openbmp-propulsion`, `openbmp-sensors`) define
//! their own physics-side traits and the kernel-side adapter
//! wires the two together.

pub mod adapters;
pub mod assembly;
pub mod effector;
pub mod error;
pub mod landing_gear;
pub mod propellant_budget;
pub mod recovery;
pub mod structural;
pub mod tank;
pub mod vehicle;

pub use adapters::{
    AeroMethodForceAdapter, AeroMethodMomentAdapter, DeckDragForceAdapter, DirectTorqueBinding,
    DirectTorqueMomentAdapter, EngineClusterForceAdapter, EngineClusterMassAdapter,
    EngineClusterMomentAdapter, GravityForceAdapter, HalfSpaceContactForceAdapter,
    MotorMassAdapter, MotorThrustForceAdapter, RecoveryRackForceAdapter, RigidMotorMassAdapter,
    TankRackForceAdapter, TankRackMassAdapter, TankRackMomentAdapter,
};
pub use assembly::{
    Assembly, AssemblyBuilder, AssemblyError, Body, BodyGeometry, KernelModelBundle,
    KernelModelBundleRigid, VehicleAssembly,
};
pub use effector::{
    ControlEffector, EffectorError, EffectorFault, EffectorLimits, EffectorState, LinearActuator,
    PulsePolarity, PwpfModulator, PwpfParams, PwpfStep, RateLimitBacklashDescribingFunction,
    RcsBlowdownParams, RcsCoupledAllocation, RcsCoupledAllocator, RcsCoupledThrusterPulse,
    RcsError, RcsMinimumImpulseBit, RcsPulse, RcsPulseEffector, RcsPulseEffectorParams,
    RcsThrusterBankEffector, RcsThrusterBankEffectorParams, RcsThrusterBankPulse,
    RcsThrusterConfig, SecondOrderServo, SecondOrderServoParams,
    rate_limit_backlash_describing_function,
};
pub use error::VehicleError;
pub use landing_gear::{CrushCore, CrushCoreResponse, LandingGearError, LandingGearLeg, OleoStage};
pub use propellant_budget::{
    EnginePropellantBinding, EnginePropellantUtilization, FeedMode, PropellantBudget,
    PropellantBudgetError, PropellantBudgetReport, PropellantTankState,
};
pub use recovery::{
    DragDevice, DrogueMainRecovery, ParachuteDrag, RecoveryCommand, RecoveryError, RecoveryModel,
    RecoveryPhase,
};
pub use tank::{
    BaffleModel, BaffledPendulum, EquivalentPendulum, EquivalentSpringMass, ForceMomentBody,
    MassContribution, MovingMassModel, PropellantSpec, RigidLiquid, Tank, TankError, TankGeometry,
};
pub use vehicle::{
    BoxedMassModel, ForceBreakdown, KernelVehicle, MomentBreakdown, NamedForceModel,
    NamedMomentModel, Vehicle,
};
