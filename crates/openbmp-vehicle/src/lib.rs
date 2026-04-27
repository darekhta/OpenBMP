//! `openbmp-vehicle` — OpenBMP vehicle composition.
//!
//! Phase 2.8 ships:
//!
//! * [`vehicle::Vehicle`] trait — the architecture's long-term
//!   force / moment / mass composition surface, parameterised over
//!   `S: SimState` so it serves both point-mass and rigid-body
//!   kernels.
//! * [`vehicle::BasicVehicle`] — Phase-2 implementation that
//!   composes ordered force-model and moment-model lists and carries a
//!   single mass model.
//!   `BasicVehicle::force_n_eci` evaluates the list in declared
//!   order with locked operand sum and short-circuits on the first
//!   model error. The Phase-2 plan documents the contract
//!   *order matters* — floating-point summation is not associative,
//!   so reordering the list changes the byte output.
//! * [`vehicle::ForceBreakdown`] / [`vehicle::MomentBreakdown`] —
//!   per-model components plus total. The kernel-side adapter at
//!   Phase 2.10 evaluates the breakdown once per step, uses its total
//!   for dynamics, and publishes `force.<name>.{x,y,z}` telemetry
//!   channels.
//! * [`vehicle::BoxedMassModel`] — convenience wrapper around
//!   `Box<dyn MassModel>` for the vehicle-owned mass model. Phase 3 will
//!   add `MultiStageMass` as a richer composition.
//! * [`error::VehicleError`].
//!
//! # Phase-1 byte-stability preservation
//!
//! The Phase-1 analytic toy gravity path remains byte-identical when
//! the single gravity force is wrapped in a one-element `BasicVehicle`;
//! the `single_force_model_vehicle_byte_matches_raw_model` test asserts
//! the direct model result, and the kernel integration test asserts the
//! final state. The Phase-1 `analytic_toy` regression continues to use
//! the kernel's existing generic `F: ForceModel<PointMassState>`
//! surface, so swapping in `BasicVehicle` is a future-Phase opt-in.
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
//! `MassModel`, `Vehicle`); L2 crates (`openbmp-env`,
//! `openbmp-aero`, `openbmp-propulsion`, `openbmp-sensors`) define
//! their own physics-side traits and the kernel-side adapter at
//! Phase 2.10 wires the two together.

pub mod adapters;
pub mod assembly;
pub mod error;
pub mod vehicle;

pub use adapters::{
    AxialDragForceAdapter, GravityForceAdapter, MotorMassAdapter, MotorThrustForceAdapter,
    RigidMotorMassAdapter,
};
pub use assembly::{
    AssemblyError, BasicAssembly, BasicAssemblyBuilder, Body, BodyGeometry, KernelModelBundle,
    KernelModelBundleRigid, VehicleAssembly,
};
pub use error::VehicleError;
pub use vehicle::{
    BasicVehicle, BoxedMassModel, ForceBreakdown, MomentBreakdown, NamedForceModel,
    NamedMomentModel, Vehicle,
};
