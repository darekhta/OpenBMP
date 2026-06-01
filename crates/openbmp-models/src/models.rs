//! Model trait declarations and simple implementations.
//!
//! Conventions:
//!
//! * Every fallible-evaluation method returns `Result<_,
//!   crate::ModelEvalError>` per the architecture's locked seam.
//! * [`ForceModel`] is generic over the [`crate::SimState`] it consumes
//!   — the [`ConstantGravityForce`] / [`ZeroForce`] models impl
//!   `ForceModel<PointMassState>`, with parallel rigid-body impls.
//! * [`MomentModel`] is the parallel rigid-body trait.
//!
//! The byte-stable contract holds: each model's
//! arithmetic is unchanged, fallibility never short-circuits
//! when used through the CLI runner, and the
//! `mass_kg`/`mass_rate_kg_s` accessors keep their behaviour.
//!
//! No model in this module accesses wall-clock time, system RNG,
//! network, or the file system.

use alloc::boxed::Box;
use alloc::collections::BTreeMap;
use alloc::string::String;
use core::fmt;

use nalgebra::{Matrix3, Vector3};
use openbmp_core::{
    BodyId, Eci, EngineId, Position3, RecoveryId, SimTime, TankId, ValidationStatus,
};
use openbmp_state::{MassProperties, PointMassState};
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

use crate::error::ModelEvalError;
use crate::state::SimState;

// ---------------------------------------------------------------------
// EffectorActualsView
// ---------------------------------------------------------------------

/// Read-only view of the kernel's per-step effector-actuals snapshot.
///
/// The runner-side `EffectorRack` wires into the deck
/// lookup: before each `kernel.step()`, the runner pushes the rack's
/// `EffectorState.actual` values into a kernel-owned
/// `BTreeMap<String, f64>` keyed by deck-axis name. The kernel's
/// derive closure then exposes that snapshot to every
/// [`ForceModel::force_n_eci`] / [`MomentModel::moment_n_m_body`]
/// call inside the RK4 stages via this view.
///
/// Legacy / Schema-1 scenarios use [`EffectorActualsView::empty`],
/// which holds no map at all — every `get` returns `None`. Schema-1
/// decks ignore the view entirely, so legacy code paths are
/// byte-identical to the map-free path.
///
/// `BTreeMap` (not `HashMap`) defeats macOS `SipHash` randomisation
/// and matches the rest of the codebase's deterministic-collection
/// convention.
#[derive(Copy, Clone, Debug, Default)]
pub struct EffectorActualsView<'a> {
    inner: Option<&'a BTreeMap<String, f64>>,
}

impl<'a> EffectorActualsView<'a> {
    /// Wrap a borrowed snapshot map.
    #[must_use]
    pub const fn new(map: &'a BTreeMap<String, f64>) -> Self {
        Self { inner: Some(map) }
    }

    /// Construct an empty view. Used by every legacy caller and by
    /// kernel/model tests that don't exercise the schema-2 path.
    #[must_use]
    pub const fn empty() -> Self {
        Self { inner: None }
    }

    /// Look up a deck-axis name. Returns `None` for missing keys and
    /// for the empty view.
    #[must_use]
    pub fn get(&self, deck_axis_name: &str) -> Option<f64> {
        self.inner.and_then(|m| m.get(deck_axis_name).copied())
    }

    /// `true` if the view holds no entries (or is the empty
    /// constructor).
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.inner.is_none_or(BTreeMap::is_empty)
    }

    /// Number of entries in the view (`0` for the empty constructor).
    #[must_use]
    pub fn len(&self) -> usize {
        self.inner.map_or(0, BTreeMap::len)
    }

    /// Iterate `(deck_axis_name, actual)` pairs. Empty for the empty
    /// view. Iteration order is `BTreeMap`-deterministic.
    pub fn iter(&self) -> impl Iterator<Item = (&str, f64)> + '_ {
        self.inner
            .into_iter()
            .flat_map(|m| m.iter().map(|(k, v)| (k.as_str(), *v)))
    }
}

// ---------------------------------------------------------------------
// EngineSnapshot / EngineSnapshotView
// ---------------------------------------------------------------------

/// Kernel-facing per-step output snapshot for one engine.
///
/// This is deliberately a flat data carrier in `openbmp-models`.
/// The propulsion crate owns concrete engine lifecycle enums and
/// state machines; the kernel and model trait layer only need the
/// deterministic numeric outputs consumed by force, moment, and mass
/// adapters. Keeping this type here prevents the portable model
/// surface and the simulation kernel from depending upward on
/// `openbmp-propulsion`.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct EngineSnapshot {
    /// Body-frame thrust vector in Newtons. Gimbal already applied by
    /// the runner-side engine rack.
    pub thrust_body: Vector3<f64>,
    /// Mass-flow rate at this step, in kg/s. Non-negative for engine
    /// implementations that obey the propulsion contract.
    pub mass_flow_kg_per_s: f64,
    /// Cumulative propellant consumed since rack construction, in kg.
    pub consumed_kg: f64,
    /// Stable lifecycle-state index for diagnostics / telemetry
    /// surfaces that need it without importing a propulsion enum.
    ///
    /// The runner maps propulsion states as: `Idle = 0`,
    /// `Igniting = 1`, `Burning = 2`, `Shutdown = 3`,
    /// `Failed = 4`.
    pub lifecycle_state_index: u8,
}

impl EngineSnapshot {
    /// Construct the at-rest snapshot used by empty or unstepped
    /// engine racks.
    #[must_use]
    pub fn idle() -> Self {
        Self {
            thrust_body: Vector3::zeros(),
            mass_flow_kg_per_s: 0.0,
            consumed_kg: 0.0,
            lifecycle_state_index: 0,
        }
    }
}

/// Read-only view of the kernel's per-step engine snapshot.
///
/// The runner-side `EngineRack` wires into the cluster
/// adapters: before each `kernel.step()`, the runner pushes a
/// `BTreeMap<EngineId, EngineSnapshot>` into the kernel via
/// `set_engine_snapshot(...)`. The kernel's derive closure then
/// exposes that snapshot to every [`ForceModel::force_n_eci`] /
/// [`MomentModel::moment_n_m_body`] / `MassModel::mass_kg_at` call
/// inside the RK4 stages via this view.
///
/// Legacy / non-cluster scenarios use [`EngineSnapshotView::empty`],
/// which holds no map at all — every `get` returns `None`. The
/// kernel-side cluster adapters short-circuit on the empty view, so
/// the legacy single-motor path is byte-identical.
///
/// `BTreeMap` (not `HashMap`) defeats macOS `SipHash` randomisation
/// and matches the rest of the codebase's deterministic-collection
/// convention.
#[derive(Copy, Clone, Debug, Default)]
pub struct EngineSnapshotView<'a> {
    inner: Option<&'a BTreeMap<EngineId, EngineSnapshot>>,
}

impl<'a> EngineSnapshotView<'a> {
    /// Wrap a borrowed snapshot map.
    #[must_use]
    pub const fn new(map: &'a BTreeMap<EngineId, EngineSnapshot>) -> Self {
        Self { inner: Some(map) }
    }

    /// Construct an empty view. Used by every legacy caller and by
    /// kernel/model tests that don't exercise the cluster path.
    #[must_use]
    pub const fn empty() -> Self {
        Self { inner: None }
    }

    /// Look up an engine by id. Returns `None` for missing keys and
    /// for the empty view.
    #[must_use]
    pub fn get(&self, id: EngineId) -> Option<EngineSnapshot> {
        self.inner.and_then(|m| m.get(&id).copied())
    }

    /// `true` if the view holds no entries (or is the empty
    /// constructor).
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.inner.is_none_or(BTreeMap::is_empty)
    }

    /// Number of entries in the view (`0` for the empty constructor).
    #[must_use]
    pub fn len(&self) -> usize {
        self.inner.map_or(0, BTreeMap::len)
    }

    /// Iterate `(EngineId, EngineSnapshot)` pairs. Empty for the
    /// empty view. Iteration order is `BTreeMap`-deterministic.
    pub fn iter(&self) -> impl Iterator<Item = (EngineId, EngineSnapshot)> + '_ {
        self.inner
            .into_iter()
            .flat_map(|m| m.iter().map(|(k, v)| (*k, *v)))
    }
}

// ---------------------------------------------------------------------
// TankSnapshotView
// ---------------------------------------------------------------------

/// Per-step snapshot of one tank's moving-mass dynamics.
///
/// Flat data carrier owned by `openbmp-sim` so the kernel-side
/// adapters can read tank state without depending on
/// `openbmp-vehicle::tank`. The runner's `TankRack` packs the
/// `MovingMassModel::mass_contribution` and `reaction_body`
/// observations into this struct each kernel base tick.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct TankSnapshot {
    /// Current moving-mass `mass_kg` from
    /// `MassContribution::mass_kg`.
    pub mass_kg: f64,
    /// Mount-relative CG offset, body frame, metres.
    pub cg_offset_body_m: Vector3<f64>,
    /// Body-origin inertia delta with parallel-axis applied,
    /// kg · m².
    pub inertia_delta_body_kg_m2: Matrix3<f64>,
    /// Body-frame reaction force exerted on the parent body,
    /// Newtons.
    pub reaction_force_body_n: Vector3<f64>,
    /// Body-frame reaction moment about parent body origin,
    /// Newton-metres.
    pub reaction_moment_body_n_m: Vector3<f64>,
    /// Remaining fluid mass, kg. Reported for telemetry; the mass
    /// contributions in the kernel use `mass_kg`, which equals
    /// `fluid_remaining_kg` for the tank implementations.
    pub fluid_remaining_kg: f64,
}

/// Read-only view of the kernel's per-step tank snapshot.
///
/// The runner-side `TankRack` wires into the kernel: every
/// kernel base tick the runner advances each tank's slosh state
/// using last step's `(accel_body, omega_body)`, packs the resulting
/// `MovingMassModel` observations into a `BTreeMap<TankId,
/// TankSnapshot>`, and pushes the map via `set_tank_snapshot(...)`.
/// The kernel-side tank-rack adapters (`TankRackForceAdapter`,
/// `TankRackMassAdapter`, `TankRackMomentAdapter` in
/// `openbmp-vehicle`) read it through this view.
///
/// Legacy scenarios with no `[[vehicle.assembly.tanks]]` block use
/// [`TankSnapshotView::empty`], every `get` returns `None`, and the
/// rack adapters short-circuit on the empty view. Pre-3.7 byte
/// output is preserved.
#[derive(Copy, Clone, Debug, Default)]
pub struct TankSnapshotView<'a> {
    inner: Option<&'a BTreeMap<TankId, TankSnapshot>>,
}

impl<'a> TankSnapshotView<'a> {
    /// Wrap a borrowed snapshot map.
    #[must_use]
    pub const fn new(map: &'a BTreeMap<TankId, TankSnapshot>) -> Self {
        Self { inner: Some(map) }
    }

    /// Construct an empty view.
    #[must_use]
    pub const fn empty() -> Self {
        Self { inner: None }
    }

    /// Look up a tank by id. Returns `None` for missing keys and
    /// for the empty view.
    #[must_use]
    pub fn get(&self, id: TankId) -> Option<TankSnapshot> {
        self.inner.and_then(|m| m.get(&id).copied())
    }

    /// `true` if the view holds no entries.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.inner.is_none_or(BTreeMap::is_empty)
    }

    /// Number of entries in the view.
    #[must_use]
    pub fn len(&self) -> usize {
        self.inner.map_or(0, BTreeMap::len)
    }

    /// Iterate `(TankId, TankSnapshot)` pairs in `BTreeMap` order.
    pub fn iter(&self) -> impl Iterator<Item = (TankId, TankSnapshot)> + '_ {
        self.inner
            .into_iter()
            .flat_map(|m| m.iter().map(|(k, v)| (*k, *v)))
    }
}

// ---------------------------------------------------------------------
// RecoverySnapshotView
// ---------------------------------------------------------------------

/// Per-step snapshot of one recovery device's deployment state.
///
/// Flat data carrier owned by `openbmp-sim` so the kernel-side
/// recovery-rack adapter (`openbmp-vehicle::adapters::
/// RecoveryRackForceAdapter`) can compute drag without depending on
/// `openbmp-vehicle::recovery`. The runner's `RecoveryRack` packs the
/// `RecoveryModel::phase`, `current_c_d`, and `current_drag_area_m2`
/// observations into this struct each kernel base tick.
///
/// `phase_index` is the `openbmp_vehicle::RecoveryPhase` discriminant
/// reproduced as a `u8` (0 = Stowed, 1 = Drogue, 2 = Main) so the
/// `openbmp-sim` layer doesn't need to depend on `openbmp-vehicle`.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct RecoverySnapshot {
    /// `true` for any non-`Stowed` phase. Drag evaluation derives
    /// force from `c_d` and `drag_area_m2`; this flag exists for
    /// telemetry and diagnostics.
    pub deployed: bool,
    /// Phase discriminant (0 = Stowed, 1 = Drogue, 2 = Main). Used by
    /// telemetry channels.
    pub phase_index: u8,
    /// Current drag coefficient `C_D` at this phase. Zero in `Stowed`.
    pub c_d: f64,
    /// Current drag area `A` (m²) at this phase. Zero in `Stowed`.
    pub drag_area_m2: f64,
}

/// Read-only view of the kernel's per-step recovery snapshot.
///
/// The runner-side `RecoveryRack` wires into the kernel:
/// every kernel base tick the runner walks each recovery device's
/// state machine (advancing it on any fired deploy-recovery event),
/// packs the resulting
/// `(phase, c_d, area)` triple into a
/// `BTreeMap<RecoveryId, RecoverySnapshot>`, and pushes the map via
/// `set_recovery_snapshot(...)`. The kernel-side recovery-rack
/// adapter reads it through this view.
///
/// Legacy scenarios with no `[[vehicle.assembly.recovery]]` block use
/// [`RecoverySnapshotView::empty`]; every `get` returns `None`, and
/// the rack adapter short-circuits on the empty view. Pre-3.9 byte
/// output is preserved.
#[derive(Copy, Clone, Debug, Default)]
pub struct RecoverySnapshotView<'a> {
    inner: Option<&'a BTreeMap<RecoveryId, RecoverySnapshot>>,
}

impl<'a> RecoverySnapshotView<'a> {
    /// Wrap a borrowed snapshot map.
    #[must_use]
    pub const fn new(map: &'a BTreeMap<RecoveryId, RecoverySnapshot>) -> Self {
        Self { inner: Some(map) }
    }

    /// Construct an empty view.
    #[must_use]
    pub const fn empty() -> Self {
        Self { inner: None }
    }

    /// Look up a recovery device by id. Returns `None` for missing
    /// keys and for the empty view.
    #[must_use]
    pub fn get(&self, id: RecoveryId) -> Option<RecoverySnapshot> {
        self.inner.and_then(|m| m.get(&id).copied())
    }

    /// `true` if the view holds no entries.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.inner.is_none_or(BTreeMap::is_empty)
    }

    /// Number of entries in the view.
    #[must_use]
    pub fn len(&self) -> usize {
        self.inner.map_or(0, BTreeMap::len)
    }

    /// Iterate `(RecoveryId, RecoverySnapshot)` pairs in `BTreeMap`
    /// order.
    pub fn iter(&self) -> impl Iterator<Item = (RecoveryId, RecoverySnapshot)> + '_ {
        self.inner
            .into_iter()
            .flat_map(|m| m.iter().map(|(k, v)| (*k, *v)))
    }
}

// ---------------------------------------------------------------------
// Environment
// ---------------------------------------------------------------------

/// Query passed to an environment model when requesting a sample.
#[derive(Copy, Clone, Debug)]
pub struct EnvironmentQuery {
    /// Sub-step time at which the sample is requested.
    pub time: SimTime,
    /// Position in `Eci` at which the sample is requested.
    pub position_eci: Position3<Eci>,
}

/// One environment sample returned by an [`EnvironmentModel`].
///
/// Carries a gravity field, atmospheric density, the inertial
/// velocity of the local still atmosphere, and a wind vector populated
/// by the runner-side `WindRack` before each `kernel.step()`. The
/// default-zero velocities keep legacy toy / vacuum scenarios stable:
/// callers that do not provide rotating-atmosphere or wind data retain
/// the historical `vehicle velocity == air-relative velocity`
/// semantics.
#[derive(Copy, Clone, Debug, Default)]
pub struct EnvironmentSample {
    /// Local gravitational acceleration in `Eci`, m/s².
    pub gravity_eci_m_s2: Vector3<f64>,
    /// Local atmospheric mass density, kg/m³. Defaults to zero so
    /// environments without an atmosphere preserve vacuum semantics.
    pub atmosphere_density_kg_m3: f64,
    /// Inertial ECI velocity of a still atmosphere parcel at the
    /// query point, m/s. WGS84/IERS runner environments populate this
    /// from the ECEF↔ECI state transform; toy fixed-Earth and null
    /// environments leave it at zero.
    pub atmosphere_velocity_eci_m_s: Vector3<f64>,
    /// NED wind vector at the kernel's current step,
    /// `(north, east, down)`, m/s. Defaults to zero — the runner
    /// pushes a non-zero value via `kernel.set_wind_sample` only for
    /// scenarios that declare a non-`none` `[wind]` kind.
    pub wind_ned_m_s: Vector3<f64>,
    /// Same wind sample expressed as an ECI vector, m/s. This is the
    /// value force and heating models subtract from vehicle velocity
    /// after subtracting [`Self::atmosphere_velocity_eci_m_s`].
    pub wind_eci_m_s: Vector3<f64>,
    /// Linear map from `(north, east, down)` wind components to ECI at
    /// this sample. The kernel uses it when splicing the per-step NED
    /// wind override into a sampled environment.
    pub wind_ned_to_eci: Matrix3<f64>,
}

impl EnvironmentSample {
    /// Replace the NED wind sample and refresh its ECI representation.
    pub fn set_wind_ned_m_s(&mut self, wind_ned_m_s: Vector3<f64>) {
        self.wind_ned_m_s = wind_ned_m_s;
        self.wind_eci_m_s = self.wind_ned_to_eci * wind_ned_m_s;
    }

    /// Vehicle velocity relative to the local moving atmosphere, in
    /// ECI components.
    #[must_use]
    pub fn air_relative_velocity_eci_m_s(
        &self,
        vehicle_velocity_eci_m_s: Vector3<f64>,
    ) -> Vector3<f64> {
        vehicle_velocity_eci_m_s - self.atmosphere_velocity_eci_m_s - self.wind_eci_m_s
    }
}

/// Trait implemented by environment-providing models.
pub trait EnvironmentModel {
    /// Sample the environment at the given query.
    ///
    /// # Errors
    ///
    /// Returns a [`ModelEvalError`] when the query leaves the model's
    /// validity envelope or the model produces non-finite output.
    fn sample(&self, query: EnvironmentQuery) -> Result<EnvironmentSample, ModelEvalError>;

    /// Validation status declared by this model.
    #[must_use]
    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Experimental
    }
}

/// Null environment: returns a zero-gravity sample at every query.
#[derive(Copy, Clone, Debug, Default)]
pub struct NullEnvironment;

impl EnvironmentModel for NullEnvironment {
    fn sample(&self, _query: EnvironmentQuery) -> Result<EnvironmentSample, ModelEvalError> {
        Ok(EnvironmentSample::default())
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

// ---------------------------------------------------------------------
// Force
// ---------------------------------------------------------------------

/// Inputs passed to a [`ForceModel::force_n_eci`] call.
///
/// Generic over the [`SimState`] the model consumes. The kernel
/// pre-extracts `mass_kg` so force models do not need to know the
/// concrete state's mass-access pattern (point-mass `state.mass` vs.
/// rigid-body `state.mass_props.mass`).
#[derive(Copy, Clone, Debug)]
pub struct ForceContext<'a, S: SimState> {
    /// Current (possibly sub-step) state.
    pub state: &'a S,
    /// Environment sample evaluated at this state and time.
    pub environment: &'a EnvironmentSample,
    /// Total mass of the body at this sub-step (kg). Pre-extracted by
    /// the kernel so this trait stays state-agnostic.
    pub mass_kg: f64,
    /// Sub-step time. May be the kernel's published time
    /// (start-of-step) or one of the RK4 intermediate times.
    pub time: SimTime,
    /// Active rigid body for this model evaluation. `None` means the
    /// pre-separation composite or a legacy single-lane scenario.
    /// `Some(body)` means force models must evaluate only resources
    /// owned by that post-separation body and skip resources owned by
    /// other bodies.
    pub active_body: Option<BodyId>,
    /// Active mission phase id, when a mission graph or flight
    /// controller owns phase state. The value is the stable
    /// path-derived `PhaseId` payload from `openbmp-mission`; keeping
    /// it as a raw integer preserves this crate's dependency boundary.
    pub phase_id: Option<u64>,
    /// Read-only view of the kernel's effector-actuals
    /// snapshot, keyed by deck-axis name. Empty for legacy /
    /// Schema-1 scenarios; populated by the runner before each
    /// `kernel.step()` for Schema-2 scenarios. All four RK4 stages
    /// see the same snapshot — matches the architecture's "effectors
    /// step at the kernel base tick" cadence.
    pub effector_actuals: EffectorActualsView<'a>,
    /// Read-only view of the kernel's per-engine
    /// snapshot, keyed by `EngineId`. Empty for legacy single-motor
    /// scenarios; populated by the runner's `EngineRack` before
    /// each `kernel.step()` for cluster scenarios. All four RK4
    /// stages see the same snapshot.
    pub engine_snapshot: EngineSnapshotView<'a>,
    /// Read-only view of the kernel's per-tank snapshot,
    /// keyed by [`TankId`]. Empty for legacy scenarios with no
    /// `[[vehicle.assembly.tanks]]` block; populated by the runner's
    /// `TankRack` before each `kernel.step()`. All four RK4 stages
    /// see the same snapshot.
    pub tank_snapshot: TankSnapshotView<'a>,
    /// Read-only view of the kernel's per-recovery snapshot,
    /// keyed by [`RecoveryId`]. Empty for legacy scenarios with no
    /// `[[vehicle.assembly.recovery]]` block; populated by the
    /// runner's `RecoveryRack` before each `kernel.step()`. All four
    /// RK4 stages see the same snapshot — phase transitions only
    /// happen on the kernel base tick (driven by `MissionPhaseGraph`
    /// event firings), so per-stage drag area is constant within
    /// one main step.
    pub recovery_snapshot: RecoverySnapshotView<'a>,
}

/// Trait implemented by force-providing models.
///
/// Generic over `S: SimState` so the same model trait can serve point-
/// mass and rigid-body kernels. The simple models impl
/// `ForceModel<PointMassState>`, with parallel rigid-body impls.
///
/// The model returns total force in `Eci`, in Newtons. The integrator
/// divides by mass to get acceleration.
pub trait ForceModel<S: SimState> {
    /// Total force in `Eci`, in Newtons.
    ///
    /// # Errors
    ///
    /// Returns a [`ModelEvalError`] when the model is queried outside
    /// its validity envelope or produces non-finite output.
    fn force_n_eci(&self, ctx: ForceContext<'_, S>) -> Result<Vector3<f64>, ModelEvalError>;

    /// `true` when this force model can be safely reused for every
    /// independently propagated rigid body after a stage separation.
    ///
    /// Only state-local, body-agnostic models such as gravity or
    /// zero-force should opt in. Vehicle-owned models whose force
    /// depends on a specific engine, aero deck, tank, recovery device,
    /// or effector must stay `false` until the caller supplies an
    /// explicit per-body force stack.
    #[must_use]
    fn supports_separated_body_propagation(&self) -> bool {
        false
    }

    /// Validation status declared by this model.
    #[must_use]
    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Experimental
    }
}

/// Constant gravity. The force returned is `mass * g`, where `g` is
/// the gravitational acceleration vector configured at construction.
#[derive(Copy, Clone, Debug)]
pub struct ConstantGravityForce {
    g_eci_m_s2: Vector3<f64>,
}

impl ConstantGravityForce {
    /// Construct from an explicit ECI gravity-acceleration vector
    /// (m/s²).
    #[must_use]
    pub const fn new(g_eci_m_s2: Vector3<f64>) -> Self {
        Self { g_eci_m_s2 }
    }

    /// Convenience: gravity along the negative `Eci` Z axis with the
    /// given magnitude (m/s²).
    #[must_use]
    pub fn down_z(g_magnitude_m_s2: f64) -> Self {
        Self::new(Vector3::new(0.0, 0.0, -g_magnitude_m_s2))
    }

    /// The gravitational acceleration vector returned by this model
    /// (m/s²).
    #[must_use]
    pub const fn g_eci_m_s2(&self) -> Vector3<f64> {
        self.g_eci_m_s2
    }
}

impl ForceModel<PointMassState> for ConstantGravityForce {
    fn force_n_eci(
        &self,
        ctx: ForceContext<'_, PointMassState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        // Locked order: scalar * vector, no FMA.
        Ok(ctx.mass_kg * self.g_eci_m_s2)
    }

    fn supports_separated_body_propagation(&self) -> bool {
        true
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

/// Zero-force model. Useful for inertial-coast scenarios and tests.
#[derive(Copy, Clone, Debug, Default)]
pub struct ZeroForce;

impl<S: SimState> ForceModel<S> for ZeroForce {
    fn force_n_eci(&self, _ctx: ForceContext<'_, S>) -> Result<Vector3<f64>, ModelEvalError> {
        Ok(Vector3::zeros())
    }

    fn supports_separated_body_propagation(&self) -> bool {
        true
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

/// Phase-dispatching force model.
///
/// The kernel still stores one `F: ForceModel<S>` value, but this
/// wrapper lets that value select a phase-specific force stack from
/// [`ForceContext::phase_id`]. Each phase entry can itself be any
/// compound force model, so higher layers can install different
/// ascent, coast, or entry stacks without mutating kernel-owned model
/// state on a phase transition.
pub struct PhaseGatedForceModel<S: SimState> {
    default_model: Box<dyn ForceModel<S>>,
    phase_models: BTreeMap<u64, Box<dyn ForceModel<S>>>,
}

impl<S: SimState> fmt::Debug for PhaseGatedForceModel<S> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PhaseGatedForceModel")
            .field("default_model", &"<dyn ForceModel>")
            .field("phase_count", &self.phase_models.len())
            .finish()
    }
}

impl<S: SimState> PhaseGatedForceModel<S> {
    /// Construct from a default model and per-phase overrides keyed by
    /// the stable path-derived [`openbmp_mission::PhaseId`] payload.
    #[must_use]
    pub fn new(
        default_model: Box<dyn ForceModel<S>>,
        phase_models: BTreeMap<u64, Box<dyn ForceModel<S>>>,
    ) -> Self {
        Self {
            default_model,
            phase_models,
        }
    }

    /// Construct with no phase overrides.
    #[must_use]
    pub fn from_default(default_model: Box<dyn ForceModel<S>>) -> Self {
        Self::new(default_model, BTreeMap::new())
    }

    /// Add or replace the model used for one phase.
    ///
    /// Returns the previous phase model if one existed.
    pub fn insert_phase_model(
        &mut self,
        phase_id: u64,
        model: Box<dyn ForceModel<S>>,
    ) -> Option<Box<dyn ForceModel<S>>> {
        self.phase_models.insert(phase_id, model)
    }

    /// Number of phase-specific model entries.
    #[must_use]
    pub fn phase_count(&self) -> usize {
        self.phase_models.len()
    }

    fn active_model(&self, phase_id: Option<u64>) -> &dyn ForceModel<S> {
        phase_id
            .and_then(|phase| self.phase_models.get(&phase))
            .map_or(self.default_model.as_ref(), |model| model.as_ref())
    }
}

impl<S: SimState> ForceModel<S> for PhaseGatedForceModel<S> {
    fn force_n_eci(&self, ctx: ForceContext<'_, S>) -> Result<Vector3<f64>, ModelEvalError> {
        self.active_model(ctx.phase_id).force_n_eci(ctx)
    }

    fn supports_separated_body_propagation(&self) -> bool {
        self.default_model.supports_separated_body_propagation()
            && self
                .phase_models
                .values()
                .all(|model| model.supports_separated_body_propagation())
    }

    fn validation(&self) -> ValidationStatus {
        if self.default_model.validation() == ValidationStatus::Checked
            && self
                .phase_models
                .values()
                .all(|model| model.validation() == ValidationStatus::Checked)
        {
            ValidationStatus::Checked
        } else {
            ValidationStatus::Experimental
        }
    }
}

// ---------------------------------------------------------------------
// Moment trait surface
// ---------------------------------------------------------------------

/// Inputs passed to a [`MomentModel::moment_n_m_body`] call.
///
/// Generic over the [`SimState`] the model consumes. Body-frame
/// moments require attitude / inertia information, which is why the
/// trait is parameterised over the state type.
#[derive(Copy, Clone, Debug)]
pub struct MomentContext<'a, S: SimState> {
    /// Current (possibly sub-step) state.
    pub state: &'a S,
    /// Environment sample evaluated at this state and time.
    pub environment: &'a EnvironmentSample,
    /// Sub-step time.
    pub time: SimTime,
    /// Active rigid body for this model evaluation. `None` means the
    /// pre-separation composite or a legacy single-lane scenario.
    /// `Some(body)` means moment models must evaluate only resources
    /// owned by that post-separation body and skip resources owned by
    /// other bodies.
    pub active_body: Option<BodyId>,
    /// Active mission phase id, encoded as the stable path-derived
    /// `PhaseId` payload. `None` means no phase-gated selection is
    /// active.
    pub phase_id: Option<u64>,
    /// Read-only effector-actuals view (same shape as
    /// `ForceContext.effector_actuals`). Schema-2 moment models
    /// consume the deflection axes that perturb `CM`
    /// in the same way schema-2 force models do.
    pub effector_actuals: EffectorActualsView<'a>,
    /// Read-only engine snapshot view. The rigid-body
    /// `EngineClusterMomentAdapter` reads per-engine thrust + mount
    /// point from this view to compute the cluster moment about
    /// the body origin.
    pub engine_snapshot: EngineSnapshotView<'a>,
    /// Read-only tank snapshot view. The rigid-body
    /// `TankRackMomentAdapter` reads per-tank reaction moments from
    /// this view.
    pub tank_snapshot: TankSnapshotView<'a>,
}

/// Trait implemented by moment-providing models.
///
/// Returns total body-frame moment in `N·m`. Impls include the
/// aero moment and gravity-gradient toy.
pub trait MomentModel<S: SimState> {
    /// Total moment in `Body`, in `N·m`.
    ///
    /// # Errors
    ///
    /// Returns a [`ModelEvalError`] when the model is queried outside
    /// its validity envelope or produces non-finite output.
    fn moment_n_m_body(&self, ctx: MomentContext<'_, S>) -> Result<Vector3<f64>, ModelEvalError>;

    /// `true` when this moment model can be safely reused for every
    /// independently propagated rigid body after a stage separation.
    ///
    /// Moment sources tied to specific engines, tanks, aero surfaces,
    /// or effectors must stay `false` until the caller supplies an
    /// explicit per-body moment stack.
    #[must_use]
    fn supports_separated_body_propagation(&self) -> bool {
        false
    }

    /// Validation status declared by this model.
    #[must_use]
    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Experimental
    }
}

/// Zero-moment model. The torque-free analytic toy uses
/// this for the headline rigid-body validation case.
#[derive(Copy, Clone, Debug, Default)]
pub struct ZeroMoment;

impl<S: SimState> MomentModel<S> for ZeroMoment {
    fn moment_n_m_body(&self, _ctx: MomentContext<'_, S>) -> Result<Vector3<f64>, ModelEvalError> {
        Ok(Vector3::zeros())
    }

    fn supports_separated_body_propagation(&self) -> bool {
        true
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

// ---------------------------------------------------------------------
// Mass
// ---------------------------------------------------------------------

/// Inputs passed to a [`MassModel::mass_kg_at`] /
/// [`MassModel::mass_rate_kg_s_at`] call.
///
/// This context-carrying entry point lets the
/// `EngineClusterMassAdapter` read the kernel's per-engine
/// snapshot. Existing time-only models keep the no-op default
/// forwarding from the legacy `mass_kg(t)` / `mass_rate_kg_s(t)`
/// methods — every time-only mass model is byte-identical
/// because the default forward calls the original method.
#[derive(Copy, Clone, Debug)]
pub struct MassContext<'a> {
    /// Sub-step time. May be the kernel's published time
    /// (start-of-step) or one of the RK4 intermediate times.
    pub time: SimTime,
    /// Active rigid body for this mass-property query. `None` means
    /// the pre-separation composite or a legacy single-lane scenario.
    pub active_body: Option<BodyId>,
    /// Read-only engine snapshot view. Empty for legacy
    /// scenarios; populated by the runner before each `step()` for
    /// cluster scenarios.
    pub engine_snapshot: EngineSnapshotView<'a>,
    /// Read-only tank snapshot view. Empty for legacy
    /// scenarios; populated by the runner before each `step()` for
    /// scenarios that declare `[[vehicle.assembly.tanks]]`.
    pub tank_snapshot: TankSnapshotView<'a>,
}

/// Trait implemented by mass-property-providing models.
///
/// Surfaces mass and mass-rate (point-mass). Full
/// rigid-body mass-property models (`MassProperties` derivatives —
/// inertia tensor, CG offset, their time derivatives) accompany
/// the [`MomentModel`] impls.
pub trait MassModel {
    /// Total mass at simulation time `t` (kg).
    ///
    /// # Errors
    ///
    /// Returns a [`ModelEvalError`] when the model is queried outside
    /// its validity envelope or produces non-finite output.
    fn mass_kg(&self, t: SimTime) -> Result<f64, ModelEvalError>;

    /// Time derivative of mass at `t` (kg/s). Negative for mass loss
    /// (propellant burn). Zero for [`ConstantMass`].
    ///
    /// # Errors
    ///
    /// Returns a [`ModelEvalError`] when the model is queried outside
    /// its validity envelope or produces non-finite output.
    fn mass_rate_kg_s(&self, t: SimTime) -> Result<f64, ModelEvalError>;

    /// Context-carrying entry point. The default
    /// implementation forwards to [`Self::mass_kg`], preserving
    /// byte-identical legacy behaviour. The
    /// `EngineClusterMassAdapter` overrides this method to read
    /// the per-engine `consumed_kg` from the kernel snapshot.
    ///
    /// # Errors
    ///
    /// Forwards the [`ModelEvalError`] from the override or the
    /// default's inner [`Self::mass_kg`] call.
    fn mass_kg_at(&self, ctx: MassContext<'_>) -> Result<f64, ModelEvalError> {
        self.mass_kg(ctx.time)
    }

    /// Context-carrying entry point. The default forwards
    /// to [`Self::mass_rate_kg_s`].
    ///
    /// # Errors
    ///
    /// Forwards the [`ModelEvalError`] from the override or the
    /// default's inner [`Self::mass_rate_kg_s`] call.
    fn mass_rate_kg_s_at(&self, ctx: MassContext<'_>) -> Result<f64, ModelEvalError> {
        self.mass_rate_kg_s(ctx.time)
    }

    /// Convenience: typed mass at `t`.
    ///
    /// # Errors
    ///
    /// Forwards the [`ModelEvalError`] returned by [`Self::mass_kg`].
    fn mass(&self, t: SimTime) -> Result<Mass, ModelEvalError> {
        Ok(Mass::new::<kilogram>(self.mass_kg(t)?))
    }

    /// Validation status declared by this model.
    #[must_use]
    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Experimental
    }
}

/// Constant-mass model.
#[derive(Copy, Clone, Debug)]
pub struct ConstantMass {
    mass_kg: f64,
}

impl ConstantMass {
    /// Construct from a scalar in kilograms. The value is **not**
    /// validated here; the kernel's `new()` validates the initial
    /// state's mass.
    #[must_use]
    pub const fn new(mass_kg: f64) -> Self {
        Self { mass_kg }
    }
}

impl MassModel for ConstantMass {
    fn mass_kg(&self, _t: SimTime) -> Result<f64, ModelEvalError> {
        Ok(self.mass_kg)
    }

    fn mass_rate_kg_s(&self, _t: SimTime) -> Result<f64, ModelEvalError> {
        Ok(0.0)
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

/// Linearly-burning mass: `m(t) = m0 + rate * (t - t0)`.
///
/// Useful for exercising variable-mass integrator behaviour without
/// committing to a full propulsion model.
#[derive(Copy, Clone, Debug)]
pub struct LinearBurnMass {
    /// Start time (s).
    pub t0_s: f64,
    /// Mass at `t0` (kg).
    pub m0_kg: f64,
    /// Burn rate (kg/s); typically negative for mass loss.
    pub rate_kg_s: f64,
}

impl LinearBurnMass {
    /// Construct a linear-burn mass model.
    #[must_use]
    pub const fn new(t0_s: f64, m0_kg: f64, rate_kg_s: f64) -> Self {
        Self {
            t0_s,
            m0_kg,
            rate_kg_s,
        }
    }
}

impl MassModel for LinearBurnMass {
    fn mass_kg(&self, t: SimTime) -> Result<f64, ModelEvalError> {
        Ok(self.m0_kg + self.rate_kg_s * (t.as_seconds() - self.t0_s))
    }

    fn mass_rate_kg_s(&self, _t: SimTime) -> Result<f64, ModelEvalError> {
        Ok(self.rate_kg_s)
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

// ---------------------------------------------------------------------
// Rigid-body mass models
// ---------------------------------------------------------------------

/// Time derivative of [`MassProperties`].
///
/// Returned by [`RigidMassModel::mass_properties_rate`]. The
/// built-in models return zero centre-of-mass rate, but the field is
/// present so the rigid-body derivative already covers the
/// full mass / CG / inertia rate shape expected by motor and
/// tank models.
#[derive(Copy, Clone, Debug, Default)]
pub struct MassPropertiesRate {
    /// `dmass/dt` in kg/s.
    pub mass_rate_kg_s: f64,
    /// `d(center_of_mass_body)/dt` in m/s, body-frame components.
    pub center_of_mass_rate_body_m_s: Vector3<f64>,
    /// `dI_body/dt` in kg·m²/s.
    pub inertia_rate_body: Matrix3<f64>,
}

impl MassPropertiesRate {
    /// All-zero rate (constant mass and inertia).
    #[must_use]
    pub fn zero() -> Self {
        Self::default()
    }
}

/// Trait implemented by rigid-body mass-property-providing models.
///
/// Returns full [`MassProperties`] (mass, body-frame CG, body-frame
/// inertia tensor) plus their time derivatives. Ships
/// [`ConstantMassRigid`] and [`LinearBurnMassRigid`]; the latter
/// declares fixed inertia per the audited simplification
/// (motor inertia rate is zero unless the scenario explicitly says
/// otherwise).
pub trait RigidMassModel {
    /// Mass properties at simulation time `t`.
    ///
    /// # Errors
    ///
    /// Returns a [`ModelEvalError`] when the model is queried outside
    /// its validity envelope or produces non-finite output.
    fn mass_properties(&self, t: SimTime) -> Result<MassProperties, ModelEvalError>;

    /// Time derivative of mass properties at `t`.
    ///
    /// # Errors
    ///
    /// Returns a [`ModelEvalError`] when the model is queried outside
    /// its validity envelope or produces non-finite output.
    fn mass_properties_rate(&self, t: SimTime) -> Result<MassPropertiesRate, ModelEvalError>;

    /// Context-carrying mass-properties query. The default
    /// implementation preserves time-only legacy behaviour. Per-body
    /// models override this to select resources owned by
    /// `ctx.active_body` after separation.
    ///
    /// # Errors
    ///
    /// Forwards the [`ModelEvalError`] from the override or the
    /// default's inner [`Self::mass_properties`] call.
    fn mass_properties_at(&self, ctx: MassContext<'_>) -> Result<MassProperties, ModelEvalError> {
        self.mass_properties(ctx.time)
    }

    /// Context-carrying mass-properties-rate query. The default
    /// implementation preserves time-only legacy behaviour. Per-body
    /// models override this to select resources owned by
    /// `ctx.active_body` after separation.
    ///
    /// # Errors
    ///
    /// Forwards the [`ModelEvalError`] from the override or the
    /// default's inner [`Self::mass_properties_rate`] call.
    fn mass_properties_rate_at(
        &self,
        ctx: MassContext<'_>,
    ) -> Result<MassPropertiesRate, ModelEvalError> {
        self.mass_properties_rate(ctx.time)
    }

    /// `true` when the rigid mass model is compatible with a
    /// separated-body lane whose mass properties are held on the lane
    /// state itself.
    ///
    /// Constant mass properties opt in. Time-varying propellant,
    /// engine, or tank models must stay `false` until separated lanes
    /// own their own mass-property model.
    #[must_use]
    fn supports_separated_body_propagation(&self) -> bool {
        false
    }

    /// Validation status declared by this model.
    #[must_use]
    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Experimental
    }
}

/// Constant-mass-and-inertia rigid-body model.
///
/// Wraps a fixed [`MassProperties`] and returns it for all times.
/// Mass-properties rate is zero. Useful for torque-free precession
/// validation cases and for any rigid-body scenario whose mass
/// budget is dominated by the dry vehicle.
#[derive(Copy, Clone, Debug)]
pub struct ConstantMassRigid {
    properties: MassProperties,
}

impl ConstantMassRigid {
    /// Construct from explicit [`MassProperties`].
    #[must_use]
    pub const fn new(properties: MassProperties) -> Self {
        Self { properties }
    }
}

impl RigidMassModel for ConstantMassRigid {
    fn mass_properties(&self, _t: SimTime) -> Result<MassProperties, ModelEvalError> {
        Ok(self.properties)
    }

    fn mass_properties_rate(&self, _t: SimTime) -> Result<MassPropertiesRate, ModelEvalError> {
        Ok(MassPropertiesRate::zero())
    }

    fn supports_separated_body_propagation(&self) -> bool {
        true
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

/// Linearly-burning rigid-body mass with fixed body-frame inertia.
///
/// Audited simplification: the inertia tensor
/// stays at the constructor value while mass varies linearly. A motor
/// with a real inertia derivative is handled by
/// `EngineCluster`.
#[derive(Copy, Clone, Debug)]
pub struct LinearBurnMassRigid {
    /// Burn-start time (s).
    pub t0_s: f64,
    /// Mass at `t0` (kg).
    pub m0_kg: f64,
    /// Burn rate (kg/s); typically negative for mass loss.
    pub rate_kg_s: f64,
    /// Body-frame centre of mass (held fixed).
    pub center_of_mass_body: Position3<openbmp_core::Body>,
    /// Body-frame inertia tensor (held fixed).
    pub inertia_body: Matrix3<f64>,
}

impl LinearBurnMassRigid {
    /// Construct.
    #[must_use]
    pub const fn new(
        t0_s: f64,
        m0_kg: f64,
        rate_kg_s: f64,
        center_of_mass_body: Position3<openbmp_core::Body>,
        inertia_body: Matrix3<f64>,
    ) -> Self {
        Self {
            t0_s,
            m0_kg,
            rate_kg_s,
            center_of_mass_body,
            inertia_body,
        }
    }
}

impl RigidMassModel for LinearBurnMassRigid {
    fn mass_properties(&self, t: SimTime) -> Result<MassProperties, ModelEvalError> {
        let m_kg = self.m0_kg + self.rate_kg_s * (t.as_seconds() - self.t0_s);
        Ok(MassProperties::new(
            Mass::new::<kilogram>(m_kg),
            self.center_of_mass_body,
            self.inertia_body,
        ))
    }

    fn mass_properties_rate(&self, _t: SimTime) -> Result<MassPropertiesRate, ModelEvalError> {
        Ok(MassPropertiesRate {
            mass_rate_kg_s: self.rate_kg_s,
            center_of_mass_rate_body_m_s: Vector3::zeros(),
            inertia_rate_body: Matrix3::zeros(),
        })
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use openbmp_core::{Position3, Velocity3};

    fn sample_state() -> PointMassState {
        PointMassState::new(
            SimTime::ZERO,
            Position3::origin(),
            Velocity3::zero(),
            Mass::new::<kilogram>(2.5),
        )
    }

    #[test]
    fn constant_gravity_produces_mass_times_g() {
        let g = ConstantGravityForce::down_z(9.80665);
        let state = sample_state();
        let env = EnvironmentSample::default();
        let f = g
            .force_n_eci(ForceContext {
                state: &state,
                environment: &env,
                mass_kg: state.mass.get::<kilogram>(),
                time: SimTime::ZERO,
                active_body: None,
                phase_id: None,
                effector_actuals: EffectorActualsView::empty(),
                engine_snapshot: EngineSnapshotView::empty(),
                tank_snapshot: TankSnapshotView::empty(),
                recovery_snapshot: RecoverySnapshotView::empty(),
            })
            .expect("force eval must succeed");
        // mass=2.5, g=9.80665 → force_z = -2.5 * 9.80665 = -24.516625
        assert_abs_diff_eq!(f.x, 0.0);
        assert_abs_diff_eq!(f.y, 0.0);
        assert_abs_diff_eq!(f.z, -2.5 * 9.80665, epsilon = 1.0e-12);
    }

    #[test]
    fn zero_force_returns_zero_vector() {
        let state = sample_state();
        let env = EnvironmentSample::default();
        let f = ZeroForce
            .force_n_eci(ForceContext {
                state: &state,
                environment: &env,
                mass_kg: state.mass.get::<kilogram>(),
                time: SimTime::ZERO,
                active_body: None,
                phase_id: None,
                effector_actuals: EffectorActualsView::empty(),
                engine_snapshot: EngineSnapshotView::empty(),
                tank_snapshot: TankSnapshotView::empty(),
                recovery_snapshot: RecoverySnapshotView::empty(),
            })
            .expect("zero force eval must succeed");
        assert_abs_diff_eq!(f.norm(), 0.0);
    }

    #[test]
    fn phase_gated_force_model_dispatches_by_phase_id() {
        let state = sample_state();
        let env = EnvironmentSample::default();
        let entry_phase = 42_u64;
        let default = ConstantGravityForce::new(Vector3::new(0.0, 1.0, 0.0));
        let entry = ConstantGravityForce::new(Vector3::new(2.0, 0.0, 0.0));
        let mut phase_models: BTreeMap<u64, Box<dyn ForceModel<PointMassState>>> = BTreeMap::new();
        phase_models.insert(entry_phase, Box::new(entry));
        let model = PhaseGatedForceModel::new(Box::new(default), phase_models);

        let force_at = |phase_id| {
            model
                .force_n_eci(ForceContext {
                    state: &state,
                    environment: &env,
                    mass_kg: state.mass.get::<kilogram>(),
                    time: SimTime::ZERO,
                    active_body: None,
                    phase_id,
                    effector_actuals: EffectorActualsView::empty(),
                    engine_snapshot: EngineSnapshotView::empty(),
                    tank_snapshot: TankSnapshotView::empty(),
                    recovery_snapshot: RecoverySnapshotView::empty(),
                })
                .expect("phase-gated force eval")
        };

        assert_eq!(force_at(None), Vector3::new(0.0, 2.5, 0.0));
        assert_eq!(force_at(Some(7)), Vector3::new(0.0, 2.5, 0.0));
        assert_eq!(force_at(Some(entry_phase)), Vector3::new(5.0, 0.0, 0.0));
    }

    #[test]
    fn constant_mass_returns_constant_value() {
        let m = ConstantMass::new(1.5);
        assert_abs_diff_eq!(m.mass_kg(SimTime::ZERO).unwrap(), 1.5);
        assert_abs_diff_eq!(m.mass_kg(SimTime::from_seconds(100.0)).unwrap(), 1.5);
        assert_abs_diff_eq!(m.mass_rate_kg_s(SimTime::ZERO).unwrap(), 0.0);
    }

    #[test]
    fn linear_burn_mass_evolves_linearly() {
        let m = LinearBurnMass::new(0.0, 10.0, -0.5);
        assert_abs_diff_eq!(m.mass_kg(SimTime::ZERO).unwrap(), 10.0);
        assert_abs_diff_eq!(m.mass_kg(SimTime::from_seconds(2.0)).unwrap(), 9.0);
        assert_abs_diff_eq!(m.mass_kg(SimTime::from_seconds(20.0)).unwrap(), 0.0);
        assert_abs_diff_eq!(m.mass_rate_kg_s(SimTime::ZERO).unwrap(), -0.5);
    }

    #[test]
    fn null_environment_returns_zero_gravity() {
        let env = NullEnvironment;
        let s = env
            .sample(EnvironmentQuery {
                time: SimTime::ZERO,
                position_eci: Position3::origin(),
            })
            .expect("null env sample must succeed");
        assert_abs_diff_eq!(s.gravity_eci_m_s2.norm(), 0.0);
    }

    #[test]
    fn validation_labels_are_checked_for_core_models() {
        assert_eq!(
            <ConstantGravityForce as ForceModel<PointMassState>>::validation(
                &ConstantGravityForce::down_z(9.81)
            ),
            ValidationStatus::Checked
        );
        assert_eq!(
            <ZeroForce as ForceModel<PointMassState>>::validation(&ZeroForce),
            ValidationStatus::Checked
        );
        assert_eq!(
            ConstantMass::new(1.0).validation(),
            ValidationStatus::Checked
        );
        assert_eq!(NullEnvironment.validation(), ValidationStatus::Checked);
    }
}
