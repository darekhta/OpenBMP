//! Kernel-side adapters that wrap L2 physics models
//! (`openbmp-physics`, `openbmp-aero`, `openbmp-propulsion`) into
//! `openbmp-sim` `ForceModel` / `MassModel` impls consumable by
//! the kernel and by [`crate::KernelVehicle`].
//!
//! `openbmp-vehicle` is the architectural seam where L2 physics
//! flow into L1 kernel surfaces. Each adapter owns a generic over
//! its underlying physics trait so a deck or motor or atmosphere
//! can swap without touching the adapter.
//!
//! # Scope
//!
//! The point-mass force adapters serve the
//! sounding-rocket runner: a vertical-launch convention
//! where thrust is along ECI `+z` (no attitude). The
//! rigid-body counterparts extend them:
//!
//! - The same `GravityForceAdapter`, `MotorThrustForceAdapter`, and
//!   `DeckDragForceAdapter` structs gain `ForceModel<RigidBodyState>`
//!   impls. Gravity and aero are state-symmetric (they read
//!   `state.position` / `state.velocity`, both fields exist on both
//!   states). Motor thrust differs: the rigid impl rotates body-`+z`
//!   thrust into ECI through the attitude quaternion, lifting the
//!   point-mass-only assumption that the rocket points along ECI
//!   `+z` for the entire flight.
//! - A new `RigidMotorMassAdapter` struct implements
//!   `RigidMassModel`, wrapping a `Motor` plus the vehicle dry mass /
//!   body-frame CG offset / body-frame inertia tensor. CG and
//!   inertia are fixed (`mass_properties_rate` returns zero
//!   for those fields); the motor's mass-rate is the only non-zero
//!   contribution.

use nalgebra::Vector3;

use openbmp_aero::{AeroDeck, AeroError};
use openbmp_models::{
    ForceContext, ForceModel, MassModel, MassPropertiesRate, ModelEvalError, MomentContext,
    MomentModel, RigidMassModel,
};
use openbmp_physics::{AtmosphereModel, GravityModel};
use openbmp_propulsion::Motor;
use openbmp_state::{MassProperties, PointMassState, RigidBodyState};

use openbmp_core::{Body, BodyId, ModelId, Position3, SimTime, ValidationStatus};
use std::borrow::Cow;
use std::collections::BTreeMap;
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

fn owner_allows(
    active_body: Option<BodyId>,
    owner: Option<BodyId>,
    model_id: ModelId,
    missing_reason: &'static str,
) -> Result<bool, ModelEvalError> {
    match active_body {
        None => Ok(true),
        Some(active) => match owner {
            Some(owner) => Ok(owner == active),
            None => Err(ModelEvalError::InvalidState {
                model: model_id,
                reason: Cow::Borrowed(missing_reason),
            }),
        },
    }
}

fn mapped_owner_allows<K: Ord>(
    active_body: Option<BodyId>,
    owners: &BTreeMap<K, BodyId>,
    key: &K,
    model_id: ModelId,
    missing_reason: &'static str,
) -> Result<bool, ModelEvalError> {
    match active_body {
        None => Ok(true),
        Some(active) => owners.get(key).map_or_else(
            || {
                Err(ModelEvalError::InvalidState {
                    model: model_id,
                    reason: Cow::Borrowed(missing_reason),
                })
            },
            |owner| Ok(*owner == active),
        ),
    }
}

// ---------------------------------------------------------------------
// GravityForceAdapter
// ---------------------------------------------------------------------

/// Wraps an [`openbmp_physics::GravityModel`] as a kernel-side
/// [`ForceModel<PointMassState>`].
///
/// The kernel passes `mass_kg` through `ForceContext`, so the
/// adapter only needs to compute `force = mass · g(position, time)`
/// in ECI. The `GravityModel` itself returns acceleration; the
/// adapter handles the mass scaling.
#[derive(Copy, Clone, Debug)]
pub struct GravityForceAdapter<G> {
    model: G,
    model_id: ModelId,
}

impl<G> GravityForceAdapter<G> {
    /// Construct from a gravity model and a stable model id.
    #[must_use]
    pub const fn new(model: G, model_id: ModelId) -> Self {
        Self { model, model_id }
    }

    /// Read-only access to the wrapped gravity model.
    #[must_use]
    pub const fn model(&self) -> &G {
        &self.model
    }
}

impl<G: GravityModel> ForceModel<PointMassState> for GravityForceAdapter<G> {
    fn force_n_eci(
        &self,
        ctx: ForceContext<'_, PointMassState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        let g = self
            .model
            .gravity_eci_m_s2(ctx.state.position, ctx.time)
            .map_err(|_| ModelEvalError::OutOfEnvelope {
                model: self.model_id,
                reason: Cow::Borrowed("gravity model out of envelope"),
            })?;
        // Locked operand order: scalar · vector. No FMA.
        let f = ctx.mass_kg * g;
        if !f.x.is_finite() || !f.y.is_finite() || !f.z.is_finite() {
            return Err(ModelEvalError::NonFinite {
                model: self.model_id,
            });
        }
        Ok(f)
    }

    fn supports_separated_body_propagation(&self) -> bool {
        true
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

impl<G: GravityModel> ForceModel<RigidBodyState> for GravityForceAdapter<G> {
    fn force_n_eci(
        &self,
        ctx: ForceContext<'_, RigidBodyState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        // RigidBodyState shares `position` and the gravity-vector
        // computation with PointMassState. The kernel pre-extracts
        // mass through `ctx.mass_kg` so the adapter does not read
        // `state.mass_props` directly.
        let g = self
            .model
            .gravity_eci_m_s2(ctx.state.position, ctx.time)
            .map_err(|_| ModelEvalError::OutOfEnvelope {
                model: self.model_id,
                reason: Cow::Borrowed("gravity model out of envelope"),
            })?;
        // Locked operand order: scalar · vector. No FMA.
        let f = ctx.mass_kg * g;
        if !f.x.is_finite() || !f.y.is_finite() || !f.z.is_finite() {
            return Err(ModelEvalError::NonFinite {
                model: self.model_id,
            });
        }
        Ok(f)
    }

    fn supports_separated_body_propagation(&self) -> bool {
        true
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

// ---------------------------------------------------------------------
// MotorThrustForceAdapter
// ---------------------------------------------------------------------

/// Wraps an [`openbmp_propulsion::Motor`] as a kernel-side
/// [`ForceModel<PointMassState>`].
///
/// Vertical-launch convention: thrust is applied along ECI
/// `+z`. The rigid-body adapter rotates body-`+z` thrust into
/// ECI via the attitude quaternion.
#[derive(Copy, Clone, Debug)]
pub struct MotorThrustForceAdapter<M> {
    motor: M,
    ignition_time_s: f64,
    model_id: ModelId,
    owner: Option<BodyId>,
}

impl<M> MotorThrustForceAdapter<M> {
    /// Construct from a motor, an ignition time in seconds (relative
    /// to the kernel's epoch), and a stable model id.
    #[must_use]
    pub const fn new(motor: M, ignition_time_s: f64, model_id: ModelId) -> Self {
        Self {
            motor,
            ignition_time_s,
            model_id,
            owner: None,
        }
    }

    /// Construct with an owning body for post-separation routing.
    #[must_use]
    pub const fn new_owned(
        motor: M,
        ignition_time_s: f64,
        model_id: ModelId,
        owner: BodyId,
    ) -> Self {
        Self {
            motor,
            ignition_time_s,
            model_id,
            owner: Some(owner),
        }
    }

    /// Read-only access to the wrapped motor.
    #[must_use]
    pub const fn motor(&self) -> &M {
        &self.motor
    }

    /// Configured ignition time (s, kernel-epoch).
    #[must_use]
    pub const fn ignition_time_s(&self) -> f64 {
        self.ignition_time_s
    }
}

impl<M: Motor> ForceModel<PointMassState> for MotorThrustForceAdapter<M> {
    fn force_n_eci(
        &self,
        ctx: ForceContext<'_, PointMassState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        if !owner_allows(
            ctx.active_body,
            self.owner,
            self.model_id,
            "motor thrust adapter: mounted_to is required for separated-body propagation",
        )? {
            return Ok(Vector3::zeros());
        }
        let t_since = ctx.time.as_seconds() - self.ignition_time_s;
        let thrust_n =
            self.motor
                .thrust_n_at(t_since)
                .map_err(|_| ModelEvalError::OutOfEnvelope {
                    model: self.model_id,
                    reason: Cow::Borrowed("motor thrust query failed"),
                })?;
        if !thrust_n.is_finite() {
            return Err(ModelEvalError::NonFinite {
                model: self.model_id,
            });
        }
        Ok(Vector3::new(0.0, 0.0, thrust_n))
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

impl<M: Motor> ForceModel<RigidBodyState> for MotorThrustForceAdapter<M> {
    fn force_n_eci(
        &self,
        ctx: ForceContext<'_, RigidBodyState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        if !owner_allows(
            ctx.active_body,
            self.owner,
            self.model_id,
            "motor thrust adapter: mounted_to is required for separated-body propagation",
        )? {
            return Ok(Vector3::zeros());
        }
        let t_since = ctx.time.as_seconds() - self.ignition_time_s;
        let thrust_n =
            self.motor
                .thrust_n_at(t_since)
                .map_err(|_| ModelEvalError::OutOfEnvelope {
                    model: self.model_id,
                    reason: Cow::Borrowed("motor thrust query failed"),
                })?;
        if !thrust_n.is_finite() {
            return Err(ModelEvalError::NonFinite {
                model: self.model_id,
            });
        }
        // Body-axis thrust convention: thrust along body `+z`,
        // rotated to ECI through the attitude quaternion. For the
        // identity orientation this reduces to the point-mass impl;
        // for any tilted body the thrust vector points along the
        // rocket's body `+z`, which is the typical solid-rocket
        // convention.
        let thrust_body = Vector3::new(0.0, 0.0, thrust_n);
        let f = ctx.state.orientation.q * thrust_body;
        if !f.x.is_finite() || !f.y.is_finite() || !f.z.is_finite() {
            return Err(ModelEvalError::NonFinite {
                model: self.model_id,
            });
        }
        Ok(f)
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }

    fn supports_separated_body_propagation(&self) -> bool {
        self.owner.is_some()
    }
}

// ---------------------------------------------------------------------
// MotorMassAdapter
// ---------------------------------------------------------------------

/// Wraps an [`openbmp_propulsion::Motor`] + a dry vehicle mass as a
/// kernel-side [`MassModel`].
///
/// Total mass = `dry_vehicle_mass_kg + motor.mass_kg(t - ignition)`.
/// Total mass-rate = `motor.mass_rate_kg_s(t - ignition)` (the dry
/// vehicle is, well, dry).
#[derive(Copy, Clone, Debug)]
pub struct MotorMassAdapter<M> {
    motor: M,
    dry_vehicle_mass_kg: f64,
    ignition_time_s: f64,
    model_id: ModelId,
}

impl<M> MotorMassAdapter<M> {
    /// Construct from `(motor, dry_vehicle_mass_kg, ignition_time_s,
    /// model_id)`.
    #[must_use]
    pub const fn new(
        motor: M,
        dry_vehicle_mass_kg: f64,
        ignition_time_s: f64,
        model_id: ModelId,
    ) -> Self {
        Self {
            motor,
            dry_vehicle_mass_kg,
            ignition_time_s,
            model_id,
        }
    }

    /// Read-only access to the wrapped motor.
    #[must_use]
    pub const fn motor(&self) -> &M {
        &self.motor
    }
}

impl<M: Motor> MassModel for MotorMassAdapter<M> {
    fn mass_kg(&self, t: SimTime) -> Result<f64, ModelEvalError> {
        let t_since = t.as_seconds() - self.ignition_time_s;
        let motor_mass =
            self.motor
                .mass_kg(t_since)
                .map_err(|_| ModelEvalError::OutOfEnvelope {
                    model: self.model_id,
                    reason: Cow::Borrowed("motor mass query failed"),
                })?;
        let total = self.dry_vehicle_mass_kg + motor_mass;
        if !total.is_finite() {
            return Err(ModelEvalError::NonFinite {
                model: self.model_id,
            });
        }
        Ok(total)
    }

    fn mass_rate_kg_s(&self, t: SimTime) -> Result<f64, ModelEvalError> {
        let t_since = t.as_seconds() - self.ignition_time_s;
        let rate =
            self.motor
                .mass_rate_kg_s(t_since)
                .map_err(|_| ModelEvalError::OutOfEnvelope {
                    model: self.model_id,
                    reason: Cow::Borrowed("motor mass-rate query failed"),
                })?;
        if !rate.is_finite() {
            return Err(ModelEvalError::NonFinite {
                model: self.model_id,
            });
        }
        Ok(rate)
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

// ---------------------------------------------------------------------
// DeckDragForceAdapter
// ---------------------------------------------------------------------

/// Point-mass axial-drag adapter.
///
/// Wraps an [`openbmp_aero::AeroDeck`] + [`openbmp_physics::AtmosphereModel`]
/// as a `ForceModel<PointMassState>` that produces only axial drag
/// (`F = -CD · q · S · v_hat`) opposing the vehicle's ECI velocity.
/// Wind is assumed zero; wind-aware resolution into the body frame
/// is handled by the runner-side wind rack.
///
/// Altitude for the atmosphere lookup is taken as the ECI `z`
/// component of position. This is the vertical-launch simplification:
/// launches are from sea level along
/// `+z` so `position.vector.z` is a good proxy for geometric
/// altitude.
///
/// CD is sampled at `(mach, alpha=0, beta=0)`. Non-axial aero
/// (CN, CM in body frame) is intentionally not consumed by this
/// adapter — that is rigid-body work.
#[derive(Clone, Debug)]
pub struct DeckDragForceAdapter<Atm> {
    deck: AeroDeck,
    atmosphere: Atm,
    model_id: ModelId,
    owner: Option<BodyId>,
}

impl<Atm> DeckDragForceAdapter<Atm> {
    /// Construct from a deck, an atmosphere model, and a stable
    /// model id.
    #[must_use]
    pub const fn new(deck: AeroDeck, atmosphere: Atm, model_id: ModelId) -> Self {
        Self {
            deck,
            atmosphere,
            model_id,
            owner: None,
        }
    }

    /// Construct with an owning body for post-separation routing.
    #[must_use]
    pub const fn new_owned(
        deck: AeroDeck,
        atmosphere: Atm,
        model_id: ModelId,
        owner: BodyId,
    ) -> Self {
        Self {
            deck,
            atmosphere,
            model_id,
            owner: Some(owner),
        }
    }

    /// Read-only access to the wrapped aero deck.
    #[must_use]
    pub const fn deck(&self) -> &AeroDeck {
        &self.deck
    }

    /// Read-only access to the wrapped atmosphere model.
    #[must_use]
    pub const fn atmosphere(&self) -> &Atm {
        &self.atmosphere
    }
}

impl<Atm: AtmosphereModel> ForceModel<PointMassState> for DeckDragForceAdapter<Atm> {
    fn force_n_eci(
        &self,
        ctx: ForceContext<'_, PointMassState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        if !owner_allows(
            ctx.active_body,
            self.owner,
            self.model_id,
            "aero deck adapter: mounted_to is required for separated-body propagation",
        )? {
            return Ok(Vector3::zeros());
        }
        compute_axial_drag(
            &self.deck,
            &self.atmosphere,
            self.model_id,
            ctx.state.velocity.vector,
            ctx.state.position.vector.z,
            ctx.time,
            ctx.effector_actuals,
        )
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

impl<Atm: AtmosphereModel> ForceModel<RigidBodyState> for DeckDragForceAdapter<Atm> {
    fn force_n_eci(
        &self,
        ctx: ForceContext<'_, RigidBodyState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        if !owner_allows(
            ctx.active_body,
            self.owner,
            self.model_id,
            "aero deck adapter: mounted_to is required for separated-body propagation",
        )? {
            return Ok(Vector3::zeros());
        }
        // Axial-drag adapter is axisymmetric: drag opposes the ECI
        // velocity vector, magnitude depends only on speed and
        // altitude. RigidBodyState carries the same `velocity` and
        // `position` fields PointMassState does, so the rigid impl
        // delegates to the shared computation. Wind and aero
        // moment / sideslip are handled elsewhere.
        compute_axial_drag(
            &self.deck,
            &self.atmosphere,
            self.model_id,
            ctx.state.velocity.vector,
            ctx.state.position.vector.z,
            ctx.time,
            ctx.effector_actuals,
        )
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }

    fn supports_separated_body_propagation(&self) -> bool {
        self.owner.is_some()
    }
}

/// Shared axial-drag force computation used by both the point-mass
/// and rigid-body `DeckDragForceAdapter` impls. Operand order is
/// locked here so a future refactor can't accidentally diverge the
/// two paths.
///
/// When the loaded deck is schema-2, the helper builds a
/// `BTreeMap<&str, f64>` keyed by deck-axis name from the kernel's
/// `EffectorActualsView`, then calls `deck.lookup` with the map.
/// Schema-1 decks have `effector_axis_names() == []`, the loop is
/// zero-iteration, and the lookup ignores the empty map — the
/// schema-1 path is byte-identical to pre-3.5.
fn compute_axial_drag<Atm: AtmosphereModel>(
    deck: &AeroDeck,
    atmosphere: &Atm,
    model_id: ModelId,
    velocity_eci: Vector3<f64>,
    position_eci_z: f64,
    time: SimTime,
    effector_actuals: openbmp_models::EffectorActualsView<'_>,
) -> Result<Vector3<f64>, ModelEvalError> {
    let v = velocity_eci;
    let speed_sq = v.x * v.x + v.y * v.y + v.z * v.z;
    if speed_sq <= 0.0 {
        // Zero velocity → zero drag. No need to evaluate the
        // atmosphere or the deck.
        return Ok(Vector3::zeros());
    }
    let speed = speed_sq.sqrt();

    // Altitude proxy: ECI z (vertical-launch simplification).
    // Clamp to zero so sub-surface descent (e.g. simulation
    // tail past apogee) doesn't trip the atmosphere model's
    // sub-zero rejection — the rocket is "on the ground" at
    // z ≤ 0 and aerodynamics there is academic.
    let altitude_m = position_eci_z.max(0.0);
    let atm_sample =
        atmosphere
            .sample(altitude_m, time)
            .map_err(|_| ModelEvalError::OutOfEnvelope {
                model: model_id,
                reason: Cow::Borrowed("atmosphere out of envelope at altitude"),
            })?;

    let speed_of_sound = atm_sample.speed_of_sound_m_s;
    if speed_of_sound <= 0.0 || !speed_of_sound.is_finite() {
        return Err(ModelEvalError::OutOfEnvelope {
            model: model_id,
            reason: Cow::Borrowed("atmosphere returned non-positive speed of sound"),
        });
    }

    let mach = speed / speed_of_sound;
    // Build the deflections map keyed by deck-axis name
    // from the kernel's effector-actuals view. For schema-1 decks
    // `deck.effector_axis_names()` is empty, the loop is
    // zero-iteration, and `deflections` stays empty. For schema-2
    // decks, every declared effector axis must have a value (the
    // runner's pre-step `assert_axes_match_effectors` check at
    // construction time guarantees the rack populates each one);
    // a missing key here would surface as `AeroError::InvalidParameter`
    // from the deck.
    let mut deflections: std::collections::BTreeMap<&str, f64> = std::collections::BTreeMap::new();
    for axis_name in deck.effector_axis_names() {
        if let Some(value) = effector_actuals.get(axis_name) {
            deflections.insert(axis_name.as_str(), value);
        }
    }
    let coefficients = deck
        .lookup(mach, 0.0, 0.0, &deflections)
        .map_err(|err| map_aero_lookup_error(model_id, err))?;

    // Locked operand order: q = 0.5 · ρ · |v|².
    let q = 0.5 * atm_sample.density_kg_m3 * speed_sq;
    let drag_magnitude = coefficients.cd * q * deck.reference_area_m2();

    // Force opposes the velocity unit vector.
    let v_hat = v / speed;
    let f = -drag_magnitude * v_hat;
    if !f.x.is_finite() || !f.y.is_finite() || !f.z.is_finite() {
        return Err(ModelEvalError::NonFinite { model: model_id });
    }
    Ok(f)
}

fn map_aero_lookup_error(model_id: ModelId, err: AeroError) -> ModelEvalError {
    match err {
        AeroError::OutOfEnvelope { reason } => ModelEvalError::OutOfEnvelope {
            model: model_id,
            reason: Cow::Borrowed(reason),
        },
        AeroError::NonFinite { .. } => ModelEvalError::NonFinite { model: model_id },
        AeroError::InvalidParameter { reason } | AeroError::MalformedDeck { reason } => {
            ModelEvalError::InvalidState {
                model: model_id,
                reason: Cow::Borrowed(reason),
            }
        }
        AeroError::Io { reason } => ModelEvalError::InvalidState {
            model: model_id,
            reason: Cow::Owned(reason),
        },
    }
}

// ---------------------------------------------------------------------
// EngineCluster adapters
// ---------------------------------------------------------------------

/// Kernel-side force adapter for an engine cluster.
///
/// Reads the kernel's per-engine snapshot via
/// [`openbmp_models::EngineSnapshotView`] and sums per-engine
/// `thrust_body` in scenario-declared engine order. Operand order
/// is locked left-fold; FMA disabled.
///
/// Point-mass impl: the cluster sums full body-frame `thrust_body`
/// vectors and treats the sum as ECI directly. Point-mass kernels
/// have no orientation, so gimbal-induced lateral components map
/// to ECI x/y. The asymmetric-thrust signal (one engine off in a
/// non-symmetric cluster) appears in the lateral components.
///
/// Rigid-body impl: rotates each engine's body-frame thrust through
/// the attitude quaternion before summing, matching
/// [`MotorThrustForceAdapter`]'s rigid-body convention.
#[derive(Clone, Debug)]
pub struct EngineClusterForceAdapter {
    engine_ids: Vec<openbmp_core::EngineId>,
    engine_owners: BTreeMap<openbmp_core::EngineId, BodyId>,
    model_id: ModelId,
}

impl EngineClusterForceAdapter {
    /// Construct from a parallel `engine_ids` array (scenario-
    /// declared order) and a stable model id.
    #[must_use]
    pub fn new(engine_ids: Vec<openbmp_core::EngineId>, model_id: ModelId) -> Self {
        Self {
            engine_ids,
            engine_owners: BTreeMap::new(),
            model_id,
        }
    }

    /// Construct with per-engine owner bodies for post-separation
    /// routing.
    #[must_use]
    pub fn new_with_owners(
        engine_ids: Vec<openbmp_core::EngineId>,
        engine_owners: BTreeMap<openbmp_core::EngineId, BodyId>,
        model_id: ModelId,
    ) -> Self {
        Self {
            engine_ids,
            engine_owners,
            model_id,
        }
    }
}

impl ForceModel<PointMassState> for EngineClusterForceAdapter {
    fn force_n_eci(
        &self,
        ctx: ForceContext<'_, PointMassState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        let mut force_eci = Vector3::zeros();
        for id in &self.engine_ids {
            if !mapped_owner_allows(
                ctx.active_body,
                &self.engine_owners,
                id,
                self.model_id,
                "engine cluster force adapter: mounted_to is required for separated-body propagation",
            )? {
                continue;
            }
            let snap = ctx
                .engine_snapshot
                .get(*id)
                .ok_or(ModelEvalError::OutOfEnvelope {
                    model: self.model_id,
                    reason: Cow::Borrowed(
                        "engine cluster force adapter: snapshot missing declared engine id",
                    ),
                })?;
            // Point-mass kernel has no orientation; the cluster's
            // body-frame thrust is treated as ECI directly. This
            // maps gimbal lateral components onto ECI x/y, which is
            // how the 3.6.D exit-criterion scenario detects
            // asymmetric thrust.
            force_eci += snap.thrust_body;
        }
        if !force_eci.x.is_finite() || !force_eci.y.is_finite() || !force_eci.z.is_finite() {
            return Err(ModelEvalError::NonFinite {
                model: self.model_id,
            });
        }
        Ok(force_eci)
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

impl ForceModel<RigidBodyState> for EngineClusterForceAdapter {
    fn force_n_eci(
        &self,
        ctx: ForceContext<'_, RigidBodyState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        let mut force_eci = Vector3::zeros();
        for id in &self.engine_ids {
            if !mapped_owner_allows(
                ctx.active_body,
                &self.engine_owners,
                id,
                self.model_id,
                "engine cluster force adapter: mounted_to is required for separated-body propagation",
            )? {
                continue;
            }
            let snap = ctx
                .engine_snapshot
                .get(*id)
                .ok_or(ModelEvalError::OutOfEnvelope {
                    model: self.model_id,
                    reason: Cow::Borrowed(
                        "engine cluster force adapter: snapshot missing declared engine id",
                    ),
                })?;
            // Rotate each engine's body-frame thrust through the
            // attitude quaternion. Same locked rotation order as
            // `MotorThrustForceAdapter::force_n_eci<RigidBodyState>`.
            let f_eci = ctx.state.orientation.q * snap.thrust_body;
            force_eci += f_eci;
        }
        if !force_eci.x.is_finite() || !force_eci.y.is_finite() || !force_eci.z.is_finite() {
            return Err(ModelEvalError::NonFinite {
                model: self.model_id,
            });
        }
        Ok(force_eci)
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }

    fn supports_separated_body_propagation(&self) -> bool {
        self.engine_ids
            .iter()
            .all(|id| self.engine_owners.contains_key(id))
    }
}

/// Kernel-side moment adapter for a rigid-body engine cluster.
///
/// Reads each engine's body-frame thrust from the kernel snapshot and
/// combines it with the matching body-frame mount point. The returned
/// moment is `mount_point_body × thrust_body`, summed in
/// scenario-declared order with locked left-fold operand order.
#[derive(Clone, Debug)]
pub struct EngineClusterMomentAdapter {
    engine_ids: Vec<openbmp_core::EngineId>,
    mount_points_body: Vec<Position3<Body>>,
    engine_owners: BTreeMap<openbmp_core::EngineId, BodyId>,
    model_id: ModelId,
}

impl EngineClusterMomentAdapter {
    /// Construct from parallel `engine_ids` and `mount_points_body`
    /// arrays in scenario-declared order.
    ///
    /// # Errors
    ///
    /// Returns [`ModelEvalError::InvalidState`] when the arrays are not
    /// the same length.
    pub fn new(
        engine_ids: Vec<openbmp_core::EngineId>,
        mount_points_body: Vec<Position3<Body>>,
        model_id: ModelId,
    ) -> Result<Self, ModelEvalError> {
        if engine_ids.len() != mount_points_body.len() {
            return Err(ModelEvalError::InvalidState {
                model: model_id,
                reason: Cow::Borrowed(
                    "engine cluster moment adapter: engine_ids and mount_points_body must have the same length",
                ),
            });
        }
        Ok(Self {
            engine_ids,
            mount_points_body,
            engine_owners: BTreeMap::new(),
            model_id,
        })
    }

    /// Construct with per-engine owner bodies for post-separation
    /// routing.
    ///
    /// # Errors
    ///
    /// Returns [`ModelEvalError::InvalidState`] when the arrays are not
    /// the same length.
    pub fn new_with_owners(
        engine_ids: Vec<openbmp_core::EngineId>,
        mount_points_body: Vec<Position3<Body>>,
        engine_owners: BTreeMap<openbmp_core::EngineId, BodyId>,
        model_id: ModelId,
    ) -> Result<Self, ModelEvalError> {
        if engine_ids.len() != mount_points_body.len() {
            return Err(ModelEvalError::InvalidState {
                model: model_id,
                reason: Cow::Borrowed(
                    "engine cluster moment adapter: engine_ids and mount_points_body must have the same length",
                ),
            });
        }
        Ok(Self {
            engine_ids,
            mount_points_body,
            engine_owners,
            model_id,
        })
    }
}

impl MomentModel<RigidBodyState> for EngineClusterMomentAdapter {
    fn moment_n_m_body(
        &self,
        ctx: MomentContext<'_, RigidBodyState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        let mut total_moment_body = Vector3::zeros();
        for (id, mount) in self.engine_ids.iter().zip(self.mount_points_body.iter()) {
            if !mapped_owner_allows(
                ctx.active_body,
                &self.engine_owners,
                id,
                self.model_id,
                "engine cluster moment adapter: mounted_to is required for separated-body propagation",
            )? {
                continue;
            }
            let snap = ctx
                .engine_snapshot
                .get(*id)
                .ok_or(ModelEvalError::OutOfEnvelope {
                    model: self.model_id,
                    reason: Cow::Borrowed(
                        "engine cluster moment adapter: snapshot missing declared engine id",
                    ),
                })?;
            let moment_body = mount.vector.cross(&snap.thrust_body);
            total_moment_body += moment_body;
        }
        if !total_moment_body.x.is_finite()
            || !total_moment_body.y.is_finite()
            || !total_moment_body.z.is_finite()
        {
            return Err(ModelEvalError::NonFinite {
                model: self.model_id,
            });
        }
        Ok(total_moment_body)
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }

    fn supports_separated_body_propagation(&self) -> bool {
        self.engine_ids
            .iter()
            .all(|id| self.engine_owners.contains_key(id))
    }
}

/// Kernel-side mass adapter for an engine cluster.
///
/// Reports `mass_kg = dry_vehicle_mass_kg - Σ consumed_kg` where
/// `consumed_kg` is the cumulative propellant deficit per engine
/// from the kernel snapshot. The runner's `EngineRack` accumulates
/// the deficit forward; the kernel never re-integrates from
/// `mass_flow`, which would drift relative to the rack and break
/// determinism.
///
/// Supports point-mass only; rigid-body cluster mass relies on
/// tank-driven mass-property dynamics.
#[derive(Clone, Debug)]
pub struct EngineClusterMassAdapter {
    dry_vehicle_mass_kg: f64,
    engine_ids: Vec<openbmp_core::EngineId>,
    engine_owners: BTreeMap<openbmp_core::EngineId, BodyId>,
    model_id: ModelId,
}

impl EngineClusterMassAdapter {
    /// Construct from a dry-vehicle mass, a parallel `engine_ids`
    /// array (scenario-declared order), and a stable model id.
    #[must_use]
    pub fn new(
        dry_vehicle_mass_kg: f64,
        engine_ids: Vec<openbmp_core::EngineId>,
        model_id: ModelId,
    ) -> Self {
        Self {
            dry_vehicle_mass_kg,
            engine_ids,
            engine_owners: BTreeMap::new(),
            model_id,
        }
    }

    /// Construct with per-engine owner bodies for post-separation
    /// routing.
    #[must_use]
    pub fn new_with_owners(
        dry_vehicle_mass_kg: f64,
        engine_ids: Vec<openbmp_core::EngineId>,
        engine_owners: BTreeMap<openbmp_core::EngineId, BodyId>,
        model_id: ModelId,
    ) -> Self {
        Self {
            dry_vehicle_mass_kg,
            engine_ids,
            engine_owners,
            model_id,
        }
    }
}

impl MassModel for EngineClusterMassAdapter {
    fn mass_kg(&self, _t: SimTime) -> Result<f64, ModelEvalError> {
        // Time-only fallback (used outside the kernel hot path):
        // assume zero consumption. Production callers go through
        // `mass_kg_at(MassContext)` which reads the kernel snapshot.
        Ok(self.dry_vehicle_mass_kg)
    }

    fn mass_rate_kg_s(&self, _t: SimTime) -> Result<f64, ModelEvalError> {
        // Time-only fallback: zero rate.
        Ok(0.0)
    }

    fn mass_kg_at(&self, ctx: openbmp_models::MassContext<'_>) -> Result<f64, ModelEvalError> {
        let mut consumed = 0.0_f64;
        for id in &self.engine_ids {
            if !mapped_owner_allows(
                ctx.active_body,
                &self.engine_owners,
                id,
                self.model_id,
                "engine cluster mass adapter: mounted_to is required for separated-body propagation",
            )? {
                continue;
            }
            let snap = ctx
                .engine_snapshot
                .get(*id)
                .ok_or(ModelEvalError::OutOfEnvelope {
                    model: self.model_id,
                    reason: Cow::Borrowed(
                        "engine cluster mass adapter: snapshot missing declared engine id",
                    ),
                })?;
            consumed += snap.consumed_kg;
        }
        let mass = self.dry_vehicle_mass_kg - consumed;
        if !mass.is_finite() {
            return Err(ModelEvalError::NonFinite {
                model: self.model_id,
            });
        }
        Ok(mass)
    }

    fn mass_rate_kg_s_at(
        &self,
        ctx: openbmp_models::MassContext<'_>,
    ) -> Result<f64, ModelEvalError> {
        let mut total = 0.0_f64;
        for id in &self.engine_ids {
            if !mapped_owner_allows(
                ctx.active_body,
                &self.engine_owners,
                id,
                self.model_id,
                "engine cluster mass adapter: mounted_to is required for separated-body propagation",
            )? {
                continue;
            }
            let snap = ctx
                .engine_snapshot
                .get(*id)
                .ok_or(ModelEvalError::OutOfEnvelope {
                    model: self.model_id,
                    reason: Cow::Borrowed(
                        "engine cluster mass adapter: snapshot missing declared engine id",
                    ),
                })?;
            total += snap.mass_flow_kg_per_s;
        }
        let rate = -total;
        if !rate.is_finite() {
            return Err(ModelEvalError::NonFinite {
                model: self.model_id,
            });
        }
        Ok(rate)
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

// ---------------------------------------------------------------------
// RigidMotorMassAdapter
// ---------------------------------------------------------------------

/// Rigid-body counterpart to [`MotorMassAdapter`]: wraps a
/// [`Motor`] plus the vehicle's dry mass, body-frame CG offset, and
/// body-frame inertia tensor as a kernel-side
/// [`RigidMassModel`].
///
/// Simplification: CG and inertia are held fixed
/// (constructor values). The motor's mass-rate is the only
/// non-zero `MassPropertiesRate` component.
#[derive(Copy, Clone, Debug)]
pub struct RigidMotorMassAdapter<M> {
    motor: M,
    dry_vehicle_mass_kg: f64,
    dry_center_of_mass_body: Position3<Body>,
    dry_inertia_body: nalgebra::Matrix3<f64>,
    ignition_time_s: f64,
    model_id: ModelId,
    owner: Option<BodyId>,
}

impl<M> RigidMotorMassAdapter<M> {
    /// Construct from `(motor, dry_vehicle_mass_kg,
    /// dry_center_of_mass_body, dry_inertia_body, ignition_time_s,
    /// model_id)`. The `dry_*` fields are the vehicle's body-frame
    /// CG and inertia; the motor contributes only mass (no inertia
    /// derivative).
    #[must_use]
    pub const fn new(
        motor: M,
        dry_vehicle_mass_kg: f64,
        dry_center_of_mass_body: Position3<Body>,
        dry_inertia_body: nalgebra::Matrix3<f64>,
        ignition_time_s: f64,
        model_id: ModelId,
    ) -> Self {
        Self {
            motor,
            dry_vehicle_mass_kg,
            dry_center_of_mass_body,
            dry_inertia_body,
            ignition_time_s,
            model_id,
            owner: None,
        }
    }

    /// Construct with an owning body for post-separation mass-rate
    /// routing.
    #[must_use]
    pub const fn new_owned(
        motor: M,
        dry_vehicle_mass_kg: f64,
        dry_center_of_mass_body: Position3<Body>,
        dry_inertia_body: nalgebra::Matrix3<f64>,
        ignition_time_s: f64,
        model_id: ModelId,
        owner: BodyId,
    ) -> Self {
        Self {
            motor,
            dry_vehicle_mass_kg,
            dry_center_of_mass_body,
            dry_inertia_body,
            ignition_time_s,
            model_id,
            owner: Some(owner),
        }
    }

    /// Read-only access to the wrapped motor.
    #[must_use]
    pub const fn motor(&self) -> &M {
        &self.motor
    }
}

impl<M: Motor> RigidMassModel for RigidMotorMassAdapter<M> {
    fn mass_properties(&self, t: SimTime) -> Result<MassProperties, ModelEvalError> {
        let t_since = t.as_seconds() - self.ignition_time_s;
        let motor_mass =
            self.motor
                .mass_kg(t_since)
                .map_err(|_| ModelEvalError::OutOfEnvelope {
                    model: self.model_id,
                    reason: Cow::Borrowed("motor mass query failed"),
                })?;
        let total_mass_kg = self.dry_vehicle_mass_kg + motor_mass;
        if !total_mass_kg.is_finite() {
            return Err(ModelEvalError::NonFinite {
                model: self.model_id,
            });
        }
        Ok(MassProperties::new(
            Mass::new::<kilogram>(total_mass_kg),
            self.dry_center_of_mass_body,
            self.dry_inertia_body,
        ))
    }

    fn mass_properties_rate(&self, t: SimTime) -> Result<MassPropertiesRate, ModelEvalError> {
        let t_since = t.as_seconds() - self.ignition_time_s;
        let mass_rate_kg_s =
            self.motor
                .mass_rate_kg_s(t_since)
                .map_err(|_| ModelEvalError::OutOfEnvelope {
                    model: self.model_id,
                    reason: Cow::Borrowed("motor mass-rate query failed"),
                })?;
        if !mass_rate_kg_s.is_finite() {
            return Err(ModelEvalError::NonFinite {
                model: self.model_id,
            });
        }
        Ok(MassPropertiesRate {
            mass_rate_kg_s,
            // CG and inertia are constant. The motor
            // contributes only mass-flow.
            center_of_mass_rate_body_m_s: Vector3::zeros(),
            inertia_rate_body: nalgebra::Matrix3::zeros(),
        })
    }

    fn mass_properties_rate_at(
        &self,
        ctx: openbmp_models::MassContext<'_>,
    ) -> Result<MassPropertiesRate, ModelEvalError> {
        if !owner_allows(
            ctx.active_body,
            self.owner,
            self.model_id,
            "rigid motor mass adapter: mounted_to is required for separated-body propagation",
        )? {
            return Ok(MassPropertiesRate::zero());
        }
        self.mass_properties_rate(ctx.time)
    }

    fn supports_separated_body_propagation(&self) -> bool {
        self.owner.is_some()
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

// ---------------------------------------------------------------------
// TankRackForceAdapter
// ---------------------------------------------------------------------

/// Kernel-side force adapter for a tank rack.
///
/// Sums the body-frame reaction forces of every declared tank from
/// the kernel's `TankSnapshotView` and returns the total in `Eci`.
/// Point-mass: body-frame == ECI (no orientation), pass through.
/// Rigid-body: rotate each tank's body-frame reaction force into
/// ECI through `state.orientation`.
///
/// Operand order: scenario-declared `tank_ids` order with locked
/// left-fold summation. Empty snapshot → zero force, byte-identical
/// to pre-3.7.
#[derive(Clone, Debug)]
pub struct TankRackForceAdapter {
    tank_ids: Vec<openbmp_core::TankId>,
    tank_owners: BTreeMap<openbmp_core::TankId, BodyId>,
    model_id: ModelId,
}

impl TankRackForceAdapter {
    /// Construct from a parallel `tank_ids` array (scenario-declared
    /// order) and a stable model id.
    #[must_use]
    pub fn new(tank_ids: Vec<openbmp_core::TankId>, model_id: ModelId) -> Self {
        Self {
            tank_ids,
            tank_owners: BTreeMap::new(),
            model_id,
        }
    }

    /// Construct with per-tank owner bodies for post-separation
    /// routing.
    #[must_use]
    pub fn new_with_owners(
        tank_ids: Vec<openbmp_core::TankId>,
        tank_owners: BTreeMap<openbmp_core::TankId, BodyId>,
        model_id: ModelId,
    ) -> Self {
        Self {
            tank_ids,
            tank_owners,
            model_id,
        }
    }
}

impl ForceModel<PointMassState> for TankRackForceAdapter {
    fn force_n_eci(
        &self,
        ctx: ForceContext<'_, PointMassState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        let mut force_eci = Vector3::zeros();
        for id in &self.tank_ids {
            if !mapped_owner_allows(
                ctx.active_body,
                &self.tank_owners,
                id,
                self.model_id,
                "tank rack force adapter: mounted_to is required for separated-body propagation",
            )? {
                continue;
            }
            let snap = ctx
                .tank_snapshot
                .get(*id)
                .ok_or(ModelEvalError::OutOfEnvelope {
                    model: self.model_id,
                    reason: Cow::Borrowed(
                        "tank rack force adapter: snapshot missing declared tank id",
                    ),
                })?;
            // Point-mass kernel has no orientation; body == ECI.
            force_eci += snap.reaction_force_body_n;
        }
        if !force_eci.x.is_finite() || !force_eci.y.is_finite() || !force_eci.z.is_finite() {
            return Err(ModelEvalError::NonFinite {
                model: self.model_id,
            });
        }
        Ok(force_eci)
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

impl ForceModel<RigidBodyState> for TankRackForceAdapter {
    fn force_n_eci(
        &self,
        ctx: ForceContext<'_, RigidBodyState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        let mut force_eci = Vector3::zeros();
        for id in &self.tank_ids {
            if !mapped_owner_allows(
                ctx.active_body,
                &self.tank_owners,
                id,
                self.model_id,
                "tank rack force adapter: mounted_to is required for separated-body propagation",
            )? {
                continue;
            }
            let snap = ctx
                .tank_snapshot
                .get(*id)
                .ok_or(ModelEvalError::OutOfEnvelope {
                    model: self.model_id,
                    reason: Cow::Borrowed(
                        "tank rack force adapter: snapshot missing declared tank id",
                    ),
                })?;
            let f_eci = ctx.state.orientation.q * snap.reaction_force_body_n;
            force_eci += f_eci;
        }
        if !force_eci.x.is_finite() || !force_eci.y.is_finite() || !force_eci.z.is_finite() {
            return Err(ModelEvalError::NonFinite {
                model: self.model_id,
            });
        }
        Ok(force_eci)
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }

    fn supports_separated_body_propagation(&self) -> bool {
        self.tank_ids
            .iter()
            .all(|id| self.tank_owners.contains_key(id))
    }
}

// ---------------------------------------------------------------------
// TankRackMomentAdapter
// ---------------------------------------------------------------------

/// Kernel-side moment adapter for a tank rack (rigid-body).
///
/// Sums the body-frame reaction moments of every declared tank from
/// the kernel's `TankSnapshotView` in scenario-declared order with
/// locked left-fold operand order.
#[derive(Clone, Debug)]
pub struct TankRackMomentAdapter {
    tank_ids: Vec<openbmp_core::TankId>,
    tank_owners: BTreeMap<openbmp_core::TankId, BodyId>,
    model_id: ModelId,
}

impl TankRackMomentAdapter {
    /// Construct from a parallel `tank_ids` array (scenario-declared
    /// order) and a stable model id.
    #[must_use]
    pub fn new(tank_ids: Vec<openbmp_core::TankId>, model_id: ModelId) -> Self {
        Self {
            tank_ids,
            tank_owners: BTreeMap::new(),
            model_id,
        }
    }

    /// Construct with per-tank owner bodies for post-separation
    /// routing.
    #[must_use]
    pub fn new_with_owners(
        tank_ids: Vec<openbmp_core::TankId>,
        tank_owners: BTreeMap<openbmp_core::TankId, BodyId>,
        model_id: ModelId,
    ) -> Self {
        Self {
            tank_ids,
            tank_owners,
            model_id,
        }
    }
}

impl MomentModel<RigidBodyState> for TankRackMomentAdapter {
    fn moment_n_m_body(
        &self,
        ctx: MomentContext<'_, RigidBodyState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        let mut total = Vector3::zeros();
        for id in &self.tank_ids {
            if !mapped_owner_allows(
                ctx.active_body,
                &self.tank_owners,
                id,
                self.model_id,
                "tank rack moment adapter: mounted_to is required for separated-body propagation",
            )? {
                continue;
            }
            let snap = ctx
                .tank_snapshot
                .get(*id)
                .ok_or(ModelEvalError::OutOfEnvelope {
                    model: self.model_id,
                    reason: Cow::Borrowed(
                        "tank rack moment adapter: snapshot missing declared tank id",
                    ),
                })?;
            total += snap.reaction_moment_body_n_m;
        }
        if !total.x.is_finite() || !total.y.is_finite() || !total.z.is_finite() {
            return Err(ModelEvalError::NonFinite {
                model: self.model_id,
            });
        }
        Ok(total)
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }

    fn supports_separated_body_propagation(&self) -> bool {
        self.tank_ids
            .iter()
            .all(|id| self.tank_owners.contains_key(id))
    }
}

// ---------------------------------------------------------------------
// DirectTorqueMomentAdapter
// ---------------------------------------------------------------------

/// Kernel-side moment adapter for direct-torque
/// effectors.
///
/// Reads each declared effector's current deflection (rad) from the
/// kernel's `EffectorActualsView`, multiplies by per-effector
/// effectiveness (N·m / rad), and sums per body axis. Used in
/// closed-loop autopilot-validation scenarios where the kernel must
/// respond to autopilot torque commands without going through an aero
/// deck (e.g. `diff-flatness-figure-eight`).
///
/// The snapshot keys are the bare effector ids the runner uses to
/// register the effectors with the kernel via `set_effector_actuals`.
/// The runner is responsible for ensuring those keys reach the
/// snapshot map every tick; missing keys contribute zero torque on
/// that axis (no error — the autopilot might have configured fewer
/// than three direct-torque effectors).
#[derive(Clone, Debug)]
pub struct DirectTorqueMomentAdapter {
    /// Per-effector binding: snapshot key, body-axis index (0/1/2),
    /// effectiveness (N·m / rad). Iteration order is stable across
    /// reruns by `Vec` order, which the runner produces in
    /// scenario-declared `[[vehicle.assembly.effectors]]` order.
    bindings: Vec<DirectTorqueBinding>,
    model_id: ModelId,
}

/// One binding row inside [`DirectTorqueMomentAdapter`].
#[derive(Clone, Debug)]
pub struct DirectTorqueBinding {
    /// Snapshot map key the runner pushes the effector deflection
    /// under (typically the effector's bare scenario-text id).
    pub snapshot_key: String,
    /// Optional owner body for post-separation routing.
    pub owner: Option<BodyId>,
    /// Body-axis index in `[roll, pitch, yaw]` order.
    pub body_axis_index: usize,
    /// Per-rad torque effectiveness (N·m / rad).
    pub effectiveness_n_m_per_rad: f64,
}

impl DirectTorqueMomentAdapter {
    /// Construct from a parallel binding vector and a stable model id.
    #[must_use]
    pub fn new(bindings: Vec<DirectTorqueBinding>, model_id: ModelId) -> Self {
        Self { bindings, model_id }
    }
}

impl MomentModel<RigidBodyState> for DirectTorqueMomentAdapter {
    fn moment_n_m_body(
        &self,
        ctx: MomentContext<'_, RigidBodyState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        let mut total: Vector3<f64> = Vector3::zeros();
        for binding in &self.bindings {
            if !owner_allows(
                ctx.active_body,
                binding.owner,
                self.model_id,
                "direct torque moment adapter: mounted_to is required for separated-body propagation",
            )? {
                continue;
            }
            let deflection = ctx
                .effector_actuals
                .get(&binding.snapshot_key)
                .unwrap_or(0.0);
            let axis = binding.body_axis_index;
            // Defensive: axis is constructed from `TorqueAxis` which is
            // bounded to {0, 1, 2}; the bound is reasserted here so a
            // future binding source that does not honour it cannot
            // out-of-bounds.
            if axis >= 3 {
                return Err(ModelEvalError::InvalidState {
                    model: self.model_id,
                    reason: Cow::Borrowed(
                        "direct torque moment adapter: body_axis_index must be 0, 1, or 2",
                    ),
                });
            }
            total[axis] += deflection * binding.effectiveness_n_m_per_rad;
        }
        if !total.x.is_finite() || !total.y.is_finite() || !total.z.is_finite() {
            return Err(ModelEvalError::NonFinite {
                model: self.model_id,
            });
        }
        Ok(total)
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }

    fn supports_separated_body_propagation(&self) -> bool {
        self.bindings.iter().all(|binding| binding.owner.is_some())
    }
}

// ---------------------------------------------------------------------
// TankRackMassAdapter
// ---------------------------------------------------------------------

/// Kernel-side mass adapter for a tank rack (point-mass).
///
/// Wraps an inner [`MassModel`] and adds each declared tank's
/// `mass_kg` from the kernel snapshot on top.
///
/// **Limitation.** Tank fluid is not linked to the
/// engine cluster's propellant accounting — scenarios that declare
/// both `[[vehicle.assembly.engines]]` and a tank intended to be
/// the cluster's propellant store will double-count that propellant
/// (the cluster mass adapter already debits consumed propellant from
/// `dry_vehicle_mass_kg`, and this adapter then adds the tank's
/// `fluid_kg` on top). Sloshing scenarios use
/// non-engine setups to avoid the double-count; engine-tank coupling
/// is not modelled.
pub struct TankRackMassAdapter {
    inner: Box<dyn MassModel>,
    tank_ids: Vec<openbmp_core::TankId>,
    tank_owners: BTreeMap<openbmp_core::TankId, BodyId>,
    model_id: ModelId,
}

impl std::fmt::Debug for TankRackMassAdapter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TankRackMassAdapter")
            .field("tank_ids", &self.tank_ids)
            .field("tank_owners", &self.tank_owners)
            .field("model_id", &self.model_id)
            .field("inner", &"Box<dyn MassModel>")
            .finish()
    }
}

impl TankRackMassAdapter {
    /// Construct from an inner mass model, a parallel `tank_ids`
    /// array (scenario-declared order), and a stable model id.
    #[must_use]
    pub fn new(
        inner: Box<dyn MassModel>,
        tank_ids: Vec<openbmp_core::TankId>,
        model_id: ModelId,
    ) -> Self {
        Self {
            inner,
            tank_ids,
            tank_owners: BTreeMap::new(),
            model_id,
        }
    }

    /// Construct with per-tank owner bodies for post-separation
    /// routing.
    #[must_use]
    pub fn new_with_owners(
        inner: Box<dyn MassModel>,
        tank_ids: Vec<openbmp_core::TankId>,
        tank_owners: BTreeMap<openbmp_core::TankId, BodyId>,
        model_id: ModelId,
    ) -> Self {
        Self {
            inner,
            tank_ids,
            tank_owners,
            model_id,
        }
    }
}

impl MassModel for TankRackMassAdapter {
    fn mass_kg(&self, t: SimTime) -> Result<f64, ModelEvalError> {
        // Time-only fallback: no snapshot available, return inner's
        // value. Production callers go through `mass_kg_at`.
        self.inner.mass_kg(t)
    }

    fn mass_rate_kg_s(&self, t: SimTime) -> Result<f64, ModelEvalError> {
        self.inner.mass_rate_kg_s(t)
    }

    fn mass_kg_at(&self, ctx: openbmp_models::MassContext<'_>) -> Result<f64, ModelEvalError> {
        let mut total = self.inner.mass_kg_at(ctx)?;
        for id in &self.tank_ids {
            if !mapped_owner_allows(
                ctx.active_body,
                &self.tank_owners,
                id,
                self.model_id,
                "tank rack mass adapter: mounted_to is required for separated-body propagation",
            )? {
                continue;
            }
            let snap = ctx
                .tank_snapshot
                .get(*id)
                .ok_or(ModelEvalError::OutOfEnvelope {
                    model: self.model_id,
                    reason: Cow::Borrowed(
                        "tank rack mass adapter: snapshot missing declared tank id",
                    ),
                })?;
            total += snap.mass_kg;
        }
        if !total.is_finite() {
            return Err(ModelEvalError::NonFinite {
                model: self.model_id,
            });
        }
        Ok(total)
    }

    fn mass_rate_kg_s_at(
        &self,
        ctx: openbmp_models::MassContext<'_>,
    ) -> Result<f64, ModelEvalError> {
        // Tank mass changes only via drain. The drain rate is set
        // outside the kernel hot path (via `Tank::drain` in the
        // runner), so the per-step mass-rate as far as the kernel
        // is concerned is the inner's plus zero. Future phases
        // unify this when tank-cluster drain coupling lands.
        self.inner.mass_rate_kg_s_at(ctx)
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

// ---------------------------------------------------------------------
// RecoveryRackForceAdapter
// ---------------------------------------------------------------------

/// Kernel-side force adapter for a recovery rack
/// (parachutes / drag devices).
///
/// Sums the drag-area · drag-coefficient product across every
/// declared recovery device from the kernel's
/// [`openbmp_models::RecoverySnapshotView`], queries the atmosphere for
/// density at the body's altitude proxy (ECI z, clamped to ≥ 0 to
/// match the [`DeckDragForceAdapter`] convention), and returns
/// `F = -½ ρ |v|² · Σ(C_D · A) · v̂` in ECI per Knacke 1992 Chapter 5.
///
/// Drag is identical for point-mass and rigid-body kernels: it
/// opposes the body's ECI velocity, magnitude depends only on
/// altitude and speed, and the direction does not depend on body
/// attitude. (Long parachute risers are assumed to decouple body
/// rotation from drag direction.)
///
/// Operand order: scenario-declared `recovery_ids` order with locked
/// left-fold summation of `(c_d, drag_area)` products. Empty snapshot
/// → no recovery devices → zero force, byte-identical to pre-3.9.
/// Stowed devices contribute `c_d = drag_area = 0` per
/// [`crate::recovery::RecoveryModel::current_c_d`] /
/// [`crate::recovery::RecoveryModel::current_drag_area_m2`], so the
/// summation skips them naturally.
pub struct RecoveryRackForceAdapter<Atm> {
    recovery_ids: Vec<openbmp_core::RecoveryId>,
    recovery_owners: BTreeMap<openbmp_core::RecoveryId, BodyId>,
    atmosphere: Atm,
    model_id: ModelId,
}

impl<Atm: std::fmt::Debug> std::fmt::Debug for RecoveryRackForceAdapter<Atm> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RecoveryRackForceAdapter")
            .field("recovery_ids", &self.recovery_ids)
            .field("recovery_owners", &self.recovery_owners)
            .field("atmosphere", &self.atmosphere)
            .field("model_id", &self.model_id)
            .finish()
    }
}

impl<Atm> RecoveryRackForceAdapter<Atm> {
    /// Construct from a parallel `recovery_ids` array
    /// (scenario-declared order), an atmosphere model, and a stable
    /// model id.
    #[must_use]
    pub const fn new(
        recovery_ids: Vec<openbmp_core::RecoveryId>,
        atmosphere: Atm,
        model_id: ModelId,
    ) -> Self {
        Self {
            recovery_ids,
            recovery_owners: BTreeMap::new(),
            atmosphere,
            model_id,
        }
    }

    /// Construct with per-recovery-device owner bodies for
    /// post-separation routing.
    #[must_use]
    pub fn new_with_owners(
        recovery_ids: Vec<openbmp_core::RecoveryId>,
        recovery_owners: BTreeMap<openbmp_core::RecoveryId, BodyId>,
        atmosphere: Atm,
        model_id: ModelId,
    ) -> Self {
        Self {
            recovery_ids,
            recovery_owners,
            atmosphere,
            model_id,
        }
    }
}

impl<Atm: AtmosphereModel> ForceModel<PointMassState> for RecoveryRackForceAdapter<Atm> {
    fn force_n_eci(
        &self,
        ctx: ForceContext<'_, PointMassState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        compute_recovery_drag(
            &self.recovery_ids,
            &self.atmosphere,
            self.model_id,
            ctx.state.velocity.vector,
            ctx.state.position.vector.z,
            ctx.time,
            ctx.active_body,
            &self.recovery_owners,
            ctx.recovery_snapshot,
        )
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

impl<Atm: AtmosphereModel> ForceModel<RigidBodyState> for RecoveryRackForceAdapter<Atm> {
    fn force_n_eci(
        &self,
        ctx: ForceContext<'_, RigidBodyState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        compute_recovery_drag(
            &self.recovery_ids,
            &self.atmosphere,
            self.model_id,
            ctx.state.velocity.vector,
            ctx.state.position.vector.z,
            ctx.time,
            ctx.active_body,
            &self.recovery_owners,
            ctx.recovery_snapshot,
        )
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }

    fn supports_separated_body_propagation(&self) -> bool {
        self.recovery_ids
            .iter()
            .all(|id| self.recovery_owners.contains_key(id))
    }
}

/// Shared recovery-drag computation for point-mass and rigid-body
/// adapters. Locked operand order so future refactors can't diverge
/// the byte output of the two kernels at zero-deflection points.
#[allow(clippy::too_many_arguments)]
fn compute_recovery_drag<Atm: AtmosphereModel>(
    recovery_ids: &[openbmp_core::RecoveryId],
    atmosphere: &Atm,
    model_id: ModelId,
    velocity_eci: Vector3<f64>,
    position_eci_z: f64,
    time: SimTime,
    active_body: Option<BodyId>,
    recovery_owners: &BTreeMap<openbmp_core::RecoveryId, BodyId>,
    recovery_snapshot: openbmp_models::RecoverySnapshotView<'_>,
) -> Result<Vector3<f64>, ModelEvalError> {
    // Sum (c_d * area) across every declared device. Locked operand
    // order matches the scenario-declared `recovery_ids` Vec; same
    // left-fold convention as TankRackForceAdapter / EngineCluster.
    let mut sum_cd_area = 0.0_f64;
    for id in recovery_ids {
        if !mapped_owner_allows(
            active_body,
            recovery_owners,
            id,
            model_id,
            "recovery rack force adapter: mounted_to is required for separated-body propagation",
        )? {
            continue;
        }
        let snap = recovery_snapshot
            .get(*id)
            .ok_or(ModelEvalError::OutOfEnvelope {
                model: model_id,
                reason: Cow::Borrowed(
                    "recovery rack force adapter: snapshot missing declared recovery id",
                ),
            })?;
        // Stowed devices have `deployed = false` and zero c_d / area
        // — the multiplication contributes 0 and the summation
        // doesn't need a special case.
        sum_cd_area += snap.c_d * snap.drag_area_m2;
    }
    if sum_cd_area <= 0.0 {
        // No deployed area. Skip the atmosphere lookup so legacy
        // scenarios keep their byte output (no atmosphere queries
        // are introduced when no recovery is declared).
        return Ok(Vector3::zeros());
    }

    let v = velocity_eci;
    let speed_sq = v.x * v.x + v.y * v.y + v.z * v.z;
    if speed_sq <= 0.0 {
        return Ok(Vector3::zeros());
    }
    let speed = speed_sq.sqrt();

    // Altitude proxy: ECI z (vertical-launch simplification, matches
    // DeckDragForceAdapter). Clamp to ≥ 0 so atmosphere out-of-
    // envelope rejections don't fire on academic scenarios that
    // start sub-surface or run past surface impact.
    let altitude_m = position_eci_z.max(0.0);
    let atm_sample =
        atmosphere
            .sample(altitude_m, time)
            .map_err(|_| ModelEvalError::OutOfEnvelope {
                model: model_id,
                reason: Cow::Borrowed(
                    "recovery rack force adapter: atmosphere out of envelope at altitude",
                ),
            })?;

    // Locked operand order: q = 0.5 · ρ · |v|².
    let q = 0.5 * atm_sample.density_kg_m3 * speed_sq;
    let drag_magnitude = q * sum_cd_area;

    let v_hat = v / speed;
    let f = -drag_magnitude * v_hat;
    if !f.x.is_finite() || !f.y.is_finite() || !f.z.is_finite() {
        return Err(ModelEvalError::NonFinite { model: model_id });
    }
    Ok(f)
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    use openbmp_core::{BodyId, EngineId, Position3, SimTime, Velocity3};
    use openbmp_models::EngineSnapshot;
    use openbmp_models::EnvironmentSample;
    use openbmp_physics::{AtmosphereSample, ConstantGravity, IsothermalAtmosphere, PhysicsError};
    use openbmp_propulsion::SolidMotor;
    use proptest::prelude::*;
    use uom::si::f64::Mass;
    use uom::si::mass::kilogram;

    fn fixture_state(z_position: f64, z_velocity: f64) -> PointMassState {
        PointMassState::new(
            SimTime::ZERO,
            Position3::new(0.0, 0.0, z_position),
            Velocity3::new(0.0, 0.0, z_velocity),
            Mass::new::<kilogram>(1.0),
        )
    }

    fn ctx<'a>(
        state: &'a PointMassState,
        env: &'a EnvironmentSample,
        mass_kg: f64,
        time_s: f64,
    ) -> ForceContext<'a, PointMassState> {
        ForceContext {
            state,
            environment: env,
            mass_kg,
            time: SimTime::from_seconds(time_s),
            active_body: None,
            effector_actuals: openbmp_models::EffectorActualsView::empty(),
            engine_snapshot: openbmp_models::EngineSnapshotView::empty(),
            tank_snapshot: openbmp_models::TankSnapshotView::empty(),
            recovery_snapshot: openbmp_models::RecoverySnapshotView::empty(),
        }
    }

    fn null_env() -> EnvironmentSample {
        EnvironmentSample::default()
    }

    fn d12_textbook_motor() -> SolidMotor {
        SolidMotor::load_from_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/motors/synthetic-solid-textbook.toml"
        )))
        .expect("textbook motor must parse")
    }

    fn small_drag_deck() -> AeroDeck {
        AeroDeck::load_from_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/aero/synthetic-finned-cylinder.toml"
        )))
        .expect("aero deck must parse")
    }

    // -----------------------------------------------------------------
    // GravityForceAdapter
    // -----------------------------------------------------------------

    #[test]
    fn gravity_adapter_multiplies_acceleration_by_mass() {
        let g = ConstantGravity::down_z(9.806_65).unwrap();
        let adapter = GravityForceAdapter::new(g, ModelId::new(0));
        let state = fixture_state(0.0, 0.0);
        let env = null_env();
        let f = adapter.force_n_eci(ctx(&state, &env, 2.5, 0.0)).unwrap();
        // F = m · g = 2.5 · (0, 0, -9.80665) = (0, 0, -24.516625)
        assert_eq!(f.z.to_bits(), (-2.5 * 9.806_65_f64).to_bits());
        assert_eq!(f.x.to_bits(), 0.0_f64.to_bits());
        assert_eq!(f.y.to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn gravity_adapter_with_zero_mass_returns_zero_force() {
        let g = ConstantGravity::down_z(9.806_65).unwrap();
        let adapter = GravityForceAdapter::new(g, ModelId::new(0));
        let state = fixture_state(0.0, 0.0);
        let env = null_env();
        let f = adapter.force_n_eci(ctx(&state, &env, 0.0, 0.0)).unwrap();
        assert_eq!(f, Vector3::zeros());
    }

    // -----------------------------------------------------------------
    // MotorThrustForceAdapter
    // -----------------------------------------------------------------

    #[test]
    fn motor_thrust_adapter_returns_zero_before_ignition() {
        let motor = d12_textbook_motor();
        let adapter = MotorThrustForceAdapter::new(motor, 1.0, ModelId::new(0));
        let state = fixture_state(0.0, 0.0);
        let env = null_env();
        // Query at t = 0, ignition at t = 1 → t_since = -1 → thrust = 0.
        let f = adapter.force_n_eci(ctx(&state, &env, 1.0, 0.0)).unwrap();
        assert_eq!(f, Vector3::zeros());
    }

    #[test]
    fn motor_thrust_adapter_produces_thrust_along_eci_z_during_burn() {
        let motor = d12_textbook_motor();
        let adapter = MotorThrustForceAdapter::new(motor, 0.0, ModelId::new(0));
        let state = fixture_state(0.0, 0.0);
        let env = null_env();
        // Query at t = 2.0 s into the textbook motor's 4 s burn
        // (between 0.5 and 3.5 s flat-top thrust = 1000 N).
        let f = adapter.force_n_eci(ctx(&state, &env, 1.0, 2.0)).unwrap();
        assert_eq!(f.x.to_bits(), 0.0_f64.to_bits());
        assert_eq!(f.y.to_bits(), 0.0_f64.to_bits());
        assert_eq!(f.z.to_bits(), 1000.0_f64.to_bits());
    }

    #[test]
    fn motor_thrust_adapter_returns_zero_after_burnout() {
        let motor = d12_textbook_motor();
        let adapter = MotorThrustForceAdapter::new(motor, 0.0, ModelId::new(0));
        let state = fixture_state(0.0, 0.0);
        let env = null_env();
        // Query well past the 4 s burn.
        let f = adapter.force_n_eci(ctx(&state, &env, 1.0, 10.0)).unwrap();
        assert_eq!(f, Vector3::zeros());
    }

    // -----------------------------------------------------------------
    // MotorMassAdapter
    // -----------------------------------------------------------------

    #[test]
    fn motor_mass_adapter_combines_dry_and_motor_mass() {
        let motor = d12_textbook_motor();
        let adapter = MotorMassAdapter::new(motor, 5.0, 0.0, ModelId::new(0));
        // Pre-ignition: dry + initial motor mass = 5.0 + (0.5 + 1.0) = 6.5
        let m_pre = adapter.mass_kg(SimTime::from_seconds(-1.0)).unwrap();
        assert_eq!(m_pre, 6.5);
        // Post-burnout: dry + motor dry = 5.0 + 0.5 = 5.5
        let m_post = adapter.mass_kg(SimTime::from_seconds(10.0)).unwrap();
        assert_eq!(m_post, 5.5);
    }

    #[test]
    fn motor_mass_adapter_rate_is_non_positive_in_burn_window() {
        let motor = d12_textbook_motor();
        let adapter = MotorMassAdapter::new(motor, 5.0, 0.0, ModelId::new(0));
        // Pre and post burn: rate = 0.
        assert_eq!(
            adapter.mass_rate_kg_s(SimTime::from_seconds(-0.1)).unwrap(),
            0.0
        );
        assert_eq!(
            adapter.mass_rate_kg_s(SimTime::from_seconds(10.0)).unwrap(),
            0.0
        );
        // Mid-burn: rate ≤ 0.
        let r = adapter.mass_rate_kg_s(SimTime::from_seconds(2.0)).unwrap();
        assert!(r <= 0.0, "rate must be ≤ 0 mid-burn, got {r}");
    }

    // -----------------------------------------------------------------
    // DeckDragForceAdapter
    // -----------------------------------------------------------------

    #[test]
    fn axial_drag_adapter_zero_velocity_returns_zero_force() {
        let atm = IsothermalAtmosphere::ussa_sea_level();
        let deck = small_drag_deck();
        let adapter = DeckDragForceAdapter::new(deck, atm, ModelId::new(0));
        let state = fixture_state(0.0, 0.0);
        let env = null_env();
        let f = adapter.force_n_eci(ctx(&state, &env, 1.0, 0.0)).unwrap();
        assert_eq!(f, Vector3::zeros());
    }

    #[test]
    fn axial_drag_adapter_opposes_velocity_with_finite_force() {
        // Small upward velocity at sea level. Drag should be in -z.
        let atm = IsothermalAtmosphere::ussa_sea_level();
        let deck = small_drag_deck();
        let adapter = DeckDragForceAdapter::new(deck, atm, ModelId::new(0));
        let state = fixture_state(0.0, 50.0);
        let env = null_env();
        let f = adapter.force_n_eci(ctx(&state, &env, 1.0, 0.0)).unwrap();
        assert!(f.z < 0.0, "drag must oppose +z velocity; got z = {}", f.z);
        assert!(f.z.is_finite());
        // x and y values are zero (sign bit irrelevant since the
        // velocity is purely along z; -0.0 emerges from the negation).
        assert_eq!(f.x, 0.0);
        assert_eq!(f.y, 0.0);
    }

    #[test]
    fn axial_drag_adapter_maps_missing_schema2_deflection_to_invalid_state() {
        let deck = AeroDeck::new_n_d(
            vec![
                "mach".to_string(),
                "alpha".to_string(),
                "beta".to_string(),
                "delta_e_deg".to_string(),
            ],
            vec![vec![0.0, 1.0], vec![0.0], vec![0.0], vec![-20.0, 20.0]],
            vec![0.0; 4],
            vec![0.5; 4],
            vec![0.0; 4],
            1.0,
            1.0,
        )
        .unwrap();
        let adapter = DeckDragForceAdapter::new(
            deck,
            IsothermalAtmosphere::ussa_sea_level(),
            ModelId::new(0),
        );
        let state = fixture_state(0.0, 50.0);
        let env = null_env();
        let err = adapter
            .force_n_eci(ctx(&state, &env, 1.0, 0.0))
            .unwrap_err();
        assert!(matches!(err, ModelEvalError::InvalidState { .. }));
    }

    // =================================================================
    // Rigid-body adapter impls
    // =================================================================

    use openbmp_core::{AngularVelocity3, Position3 as Pos3, Quaternion};
    use openbmp_state::{MassProperties, RigidBodyState};

    fn rigid_state_with_orientation(
        z_position: f64,
        z_velocity: f64,
        orientation: Quaternion<openbmp_core::Body, openbmp_core::Eci>,
    ) -> RigidBodyState {
        let mass_props =
            MassProperties::with_uniform_inertia(Mass::new::<kilogram>(1.0), Pos3::origin(), 1.0);
        RigidBodyState::new(
            SimTime::ZERO,
            Position3::new(0.0, 0.0, z_position),
            Velocity3::new(0.0, 0.0, z_velocity),
            orientation,
            AngularVelocity3::new(0.0, 0.0, 0.0),
            mass_props,
        )
    }

    fn identity_body_to_eci() -> Quaternion<openbmp_core::Body, openbmp_core::Eci> {
        Quaternion::from_unit_quaternion(nalgebra::UnitQuaternion::identity())
    }

    fn rigid_ctx<'a>(
        state: &'a RigidBodyState,
        env: &'a EnvironmentSample,
        mass_kg: f64,
        time_s: f64,
    ) -> ForceContext<'a, RigidBodyState> {
        ForceContext {
            state,
            environment: env,
            mass_kg,
            time: SimTime::from_seconds(time_s),
            active_body: None,
            effector_actuals: openbmp_models::EffectorActualsView::empty(),
            engine_snapshot: openbmp_models::EngineSnapshotView::empty(),
            tank_snapshot: openbmp_models::TankSnapshotView::empty(),
            recovery_snapshot: openbmp_models::RecoverySnapshotView::empty(),
        }
    }

    fn rigid_moment_ctx<'a>(
        state: &'a RigidBodyState,
        env: &'a EnvironmentSample,
        snapshot: &'a BTreeMap<EngineId, EngineSnapshot>,
    ) -> MomentContext<'a, RigidBodyState> {
        MomentContext {
            state,
            environment: env,
            time: SimTime::ZERO,
            active_body: None,
            effector_actuals: openbmp_models::EffectorActualsView::empty(),
            engine_snapshot: openbmp_models::EngineSnapshotView::new(snapshot),
            tank_snapshot: openbmp_models::TankSnapshotView::empty(),
        }
    }

    // -----------------------------------------------------------------
    // Rigid GravityForceAdapter
    // -----------------------------------------------------------------

    #[test]
    fn rigid_gravity_force_adapter_matches_point_mass_at_identity_orientation() {
        let g = ConstantGravity::down_z(9.806_65).unwrap();
        let adapter = GravityForceAdapter::new(g, ModelId::new(0));
        let env = null_env();
        let pm_state = fixture_state(100.0, 0.0);
        let rb_state = rigid_state_with_orientation(100.0, 0.0, identity_body_to_eci());
        let pm =
            ForceModel::<PointMassState>::force_n_eci(&adapter, ctx(&pm_state, &env, 1.5, 0.0))
                .unwrap();
        let rb = ForceModel::<RigidBodyState>::force_n_eci(
            &adapter,
            rigid_ctx(&rb_state, &env, 1.5, 0.0),
        )
        .unwrap();
        // Gravity is state-symmetric — bit-equal at any orientation
        // because it doesn't read attitude.
        assert_eq!(pm.x.to_bits(), rb.x.to_bits());
        assert_eq!(pm.y.to_bits(), rb.y.to_bits());
        assert_eq!(pm.z.to_bits(), rb.z.to_bits());
    }

    // -----------------------------------------------------------------
    // Rigid MotorThrustForceAdapter
    // -----------------------------------------------------------------

    #[test]
    fn rigid_motor_thrust_at_identity_orientation_equals_point_mass_thrust() {
        let motor = d12_textbook_motor();
        let adapter = MotorThrustForceAdapter::new(motor, 0.0, ModelId::new(0));
        let env = null_env();
        let pm_state = fixture_state(0.0, 0.0);
        let rb_state = rigid_state_with_orientation(0.0, 0.0, identity_body_to_eci());
        // Mid-burn time.
        let pm =
            ForceModel::<PointMassState>::force_n_eci(&adapter, ctx(&pm_state, &env, 1.5, 1.0))
                .unwrap();
        let rb = ForceModel::<RigidBodyState>::force_n_eci(
            &adapter,
            rigid_ctx(&rb_state, &env, 1.5, 1.0),
        )
        .unwrap();
        // Identity orientation: body +z = ECI +z. Bit-equal thrust.
        assert_eq!(pm.x.to_bits(), rb.x.to_bits());
        assert_eq!(pm.y.to_bits(), rb.y.to_bits());
        assert_eq!(pm.z.to_bits(), rb.z.to_bits());
    }

    #[test]
    fn rigid_motor_thrust_rotates_body_z_through_attitude_quaternion() {
        // 90° rotation about ECI +x: body +z → ECI -y.
        let motor = d12_textbook_motor();
        let adapter = MotorThrustForceAdapter::new(motor, 0.0, ModelId::new(0));
        let env = null_env();
        // Build the rotation: rotating body +z by 90° about ECI +x
        // axis sends body +z to ECI -y. Quaternion (axis = +x, angle
        // = π/2): (cos(π/4), sin(π/4)·1, 0, 0) = (√2/2, √2/2, 0, 0).
        let half_root_two = std::f64::consts::FRAC_1_SQRT_2;
        let raw = nalgebra::Quaternion::new(half_root_two, half_root_two, 0.0, 0.0);
        let unit = nalgebra::UnitQuaternion::from_quaternion(raw);
        let orientation =
            Quaternion::<openbmp_core::Body, openbmp_core::Eci>::from_unit_quaternion(unit);
        let state = rigid_state_with_orientation(0.0, 0.0, orientation);
        let f =
            ForceModel::<RigidBodyState>::force_n_eci(&adapter, rigid_ctx(&state, &env, 1.5, 1.0))
                .unwrap();
        // Compute expected magnitude from the same motor query.
        let thrust_mag = adapter.motor().thrust_n_at(1.0).unwrap();
        // Expected: ECI = (0, -thrust, 0). Allow 1e-12 absolute
        // tolerance for the floating-point rotation.
        let tol = 1.0e-12;
        assert!(f.x.abs() < tol, "f.x = {}", f.x);
        assert!(
            (f.y + thrust_mag).abs() < tol * thrust_mag.abs().max(1.0),
            "f.y = {}, expected ≈ -{}",
            f.y,
            thrust_mag
        );
        assert!(f.z.abs() < tol, "f.z = {}", f.z);
    }

    // -----------------------------------------------------------------
    // Rigid DeckDragForceAdapter
    // -----------------------------------------------------------------

    #[test]
    fn rigid_axial_drag_matches_point_mass_at_identity_orientation() {
        let atm = IsothermalAtmosphere::ussa_sea_level();
        let deck = small_drag_deck();
        let adapter = DeckDragForceAdapter::new(deck, atm, ModelId::new(0));
        let env = null_env();
        let pm_state = fixture_state(0.0, 50.0);
        let rb_state = rigid_state_with_orientation(0.0, 50.0, identity_body_to_eci());
        let pm =
            ForceModel::<PointMassState>::force_n_eci(&adapter, ctx(&pm_state, &env, 1.0, 0.0))
                .unwrap();
        let rb = ForceModel::<RigidBodyState>::force_n_eci(
            &adapter,
            rigid_ctx(&rb_state, &env, 1.0, 0.0),
        )
        .unwrap();
        // Axial drag is axisymmetric — depends only on speed and
        // altitude, not on attitude. Bit-equal across the two paths.
        assert_eq!(pm.x.to_bits(), rb.x.to_bits());
        assert_eq!(pm.y.to_bits(), rb.y.to_bits());
        assert_eq!(pm.z.to_bits(), rb.z.to_bits());
    }

    // -----------------------------------------------------------------
    // RigidMotorMassAdapter
    // -----------------------------------------------------------------

    #[test]
    fn rigid_motor_mass_adapter_total_mass_matches_point_mass_adapter() {
        let motor = d12_textbook_motor();
        let pm = MotorMassAdapter::new(motor, 5.0, 0.0, ModelId::new(0));
        let inertia = nalgebra::Matrix3::identity();
        let rb = RigidMotorMassAdapter::new(
            d12_textbook_motor(),
            5.0,
            Pos3::origin(),
            inertia,
            0.0,
            ModelId::new(0),
        );
        // Pre-, mid-, and post-burn samples should all match.
        for t_s in [-0.5, 0.0, 1.0, 2.0, 4.0, 5.0, 6.0] {
            let pm_mass = pm.mass_kg(SimTime::from_seconds(t_s)).unwrap();
            let rb_props = rb.mass_properties(SimTime::from_seconds(t_s)).unwrap();
            assert_eq!(
                pm_mass.to_bits(),
                rb_props.mass_kg().to_bits(),
                "mass mismatch at t = {t_s}",
            );
        }
    }

    #[test]
    fn rigid_motor_mass_adapter_holds_inertia_fixed_across_burn() {
        let motor = d12_textbook_motor();
        let inertia = nalgebra::Matrix3::from_diagonal(&Vector3::new(0.5, 0.5, 0.1));
        let cg_offset = Pos3::origin();
        let rb = RigidMotorMassAdapter::new(motor, 5.0, cg_offset, inertia, 0.0, ModelId::new(0));
        // Inertia and CG are constant across burn.
        let early = rb.mass_properties(SimTime::from_seconds(0.5)).unwrap();
        let late = rb.mass_properties(SimTime::from_seconds(3.5)).unwrap();
        assert_eq!(early.center_of_mass_body, late.center_of_mass_body);
        assert_eq!(early.inertia_body, late.inertia_body);
        // Rate has zero CG / inertia derivatives.
        let rate = rb.mass_properties_rate(SimTime::from_seconds(2.0)).unwrap();
        assert_eq!(rate.center_of_mass_rate_body_m_s, Vector3::zeros());
        assert_eq!(rate.inertia_rate_body, nalgebra::Matrix3::zeros());
        // Mass-rate is non-positive in the burn window.
        assert!(rate.mass_rate_kg_s <= 0.0);
    }

    // -----------------------------------------------------------------
    // EngineClusterMomentAdapter
    // -----------------------------------------------------------------

    proptest! {
        #[test]
        fn engine_cluster_moment_equals_literal_cross_product_sum(
            entries in proptest::collection::vec(
                (
                    -10.0_f64..10.0,
                    -10.0_f64..10.0,
                    -10.0_f64..10.0,
                    -10_000.0_f64..10_000.0,
                    -10_000.0_f64..10_000.0,
                    -10_000.0_f64..10_000.0,
                ),
                2..=9,
            )
        ) {
            let mut engine_ids = Vec::with_capacity(entries.len());
            let mut mount_points_body = Vec::with_capacity(entries.len());
            let mut snapshot = BTreeMap::new();

            for (index, (mx, my, mz, tx, ty, tz)) in entries.iter().copied().enumerate() {
                let id = EngineId::from_path(&format!("test.cluster.engine_{index}"));
                engine_ids.push(id);
                mount_points_body.push(Position3::<Body>::new(mx, my, mz));
                snapshot.insert(
                    id,
                    EngineSnapshot {
                        thrust_body: Vector3::new(tx, ty, tz),
                        mass_flow_kg_per_s: 0.0,
                        consumed_kg: 0.0,
                        lifecycle_state_index: 2,
                    },
                );
            }

            let adapter = EngineClusterMomentAdapter::new(
                engine_ids,
                mount_points_body.clone(),
                ModelId::new(999),
            )
            .unwrap();
            let state = rigid_state_with_orientation(0.0, 0.0, identity_body_to_eci());
            let env = null_env();
            let got = adapter
                .moment_n_m_body(rigid_moment_ctx(&state, &env, &snapshot))
                .unwrap();

            let mut expected = Vector3::zeros();
            for (mount, (_, _, _, tx, ty, tz)) in mount_points_body
                .iter()
                .zip(entries.iter().copied())
            {
                expected += mount.vector.cross(&Vector3::new(tx, ty, tz));
            }

            prop_assert_eq!(got.x.to_bits(), expected.x.to_bits());
            prop_assert_eq!(got.y.to_bits(), expected.y.to_bits());
            prop_assert_eq!(got.z.to_bits(), expected.z.to_bits());
        }
    }

    // -----------------------------------------------------------------
    // RecoveryRackForceAdapter
    // -----------------------------------------------------------------

    use openbmp_core::RecoveryId;
    use openbmp_models::{RecoverySnapshot, RecoverySnapshotView};

    fn recovery_snapshot_map(
        entries: &[(RecoveryId, bool, f64, f64)],
    ) -> BTreeMap<RecoveryId, RecoverySnapshot> {
        entries
            .iter()
            .copied()
            .map(|(id, deployed, c_d, drag_area_m2)| {
                (
                    id,
                    RecoverySnapshot {
                        deployed,
                        phase_index: if deployed { 2 } else { 0 },
                        c_d,
                        drag_area_m2,
                    },
                )
            })
            .collect()
    }

    fn recovery_ctx<'a>(
        state: &'a PointMassState,
        env: &'a EnvironmentSample,
        time_s: f64,
        snapshot: &'a BTreeMap<RecoveryId, RecoverySnapshot>,
    ) -> ForceContext<'a, PointMassState> {
        ForceContext {
            state,
            environment: env,
            mass_kg: state.mass.get::<kilogram>(),
            time: SimTime::from_seconds(time_s),
            active_body: None,
            effector_actuals: openbmp_models::EffectorActualsView::empty(),
            engine_snapshot: openbmp_models::EngineSnapshotView::empty(),
            tank_snapshot: openbmp_models::TankSnapshotView::empty(),
            recovery_snapshot: RecoverySnapshotView::new(snapshot),
        }
    }

    #[test]
    fn recovery_rack_empty_ids_returns_zero_force() {
        let atm = IsothermalAtmosphere::ussa_sea_level();
        let adapter = RecoveryRackForceAdapter::new(vec![], atm, ModelId::new(370));
        let state = fixture_state(1000.0, -50.0);
        let env = null_env();
        let snapshot = BTreeMap::new();
        let f = adapter
            .force_n_eci(recovery_ctx(&state, &env, 0.0, &snapshot))
            .unwrap();
        assert_eq!(f.x.to_bits(), 0.0_f64.to_bits());
        assert_eq!(f.y.to_bits(), 0.0_f64.to_bits());
        assert_eq!(f.z.to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn recovery_rack_all_stowed_returns_zero_force() {
        let id_a = RecoveryId::from_path("recovery.a");
        let id_b = RecoveryId::from_path("recovery.b");
        let atm = IsothermalAtmosphere::ussa_sea_level();
        let adapter = RecoveryRackForceAdapter::new(vec![id_a, id_b], atm, ModelId::new(370));
        let state = fixture_state(1000.0, -50.0);
        let env = null_env();
        // Both stowed: deployed = false, c_d = 0, area = 0.
        let snapshot = recovery_snapshot_map(&[(id_a, false, 0.0, 0.0), (id_b, false, 0.0, 0.0)]);
        let f = adapter
            .force_n_eci(recovery_ctx(&state, &env, 0.0, &snapshot))
            .unwrap();
        assert_eq!(f.x.to_bits(), 0.0_f64.to_bits());
        assert_eq!(f.y.to_bits(), 0.0_f64.to_bits());
        assert_eq!(f.z.to_bits(), 0.0_f64.to_bits());
    }

    #[derive(Copy, Clone, Debug)]
    struct RejectingAtmosphere;

    impl AtmosphereModel for RejectingAtmosphere {
        fn sample(
            &self,
            _altitude_geometric_m: f64,
            _time: SimTime,
        ) -> Result<AtmosphereSample, PhysicsError> {
            Err(PhysicsError::OutOfEnvelope {
                reason: "test atmosphere should not be sampled",
            })
        }
    }

    #[test]
    fn recovery_rack_all_stowed_skips_atmosphere_lookup() {
        let id_a = RecoveryId::from_path("recovery.a");
        let id_b = RecoveryId::from_path("recovery.b");
        let adapter =
            RecoveryRackForceAdapter::new(vec![id_a, id_b], RejectingAtmosphere, ModelId::new(370));
        let state = fixture_state(1000.0, -50.0);
        let env = null_env();
        let snapshot = recovery_snapshot_map(&[(id_a, false, 0.0, 0.0), (id_b, false, 0.0, 0.0)]);
        let f = adapter
            .force_n_eci(recovery_ctx(&state, &env, 0.0, &snapshot))
            .unwrap();
        assert_eq!(f.x.to_bits(), 0.0_f64.to_bits());
        assert_eq!(f.y.to_bits(), 0.0_f64.to_bits());
        assert_eq!(f.z.to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn recovery_rack_zero_velocity_returns_zero_force() {
        let id_a = RecoveryId::from_path("recovery.a");
        let atm = IsothermalAtmosphere::ussa_sea_level();
        let adapter = RecoveryRackForceAdapter::new(vec![id_a], atm, ModelId::new(370));
        // velocity = 0 → drag = 0 regardless of deployment.
        let state = fixture_state(0.0, 0.0);
        let env = null_env();
        let snapshot = recovery_snapshot_map(&[(id_a, true, 1.5, 2.0)]);
        let f = adapter
            .force_n_eci(recovery_ctx(&state, &env, 0.0, &snapshot))
            .unwrap();
        assert_eq!(f.x.to_bits(), 0.0_f64.to_bits());
        assert_eq!(f.y.to_bits(), 0.0_f64.to_bits());
        assert_eq!(f.z.to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn recovery_rack_deployed_drag_opposes_velocity() {
        let id_a = RecoveryId::from_path("recovery.main");
        let atm = IsothermalAtmosphere::ussa_sea_level();
        let adapter = RecoveryRackForceAdapter::new(vec![id_a], atm, ModelId::new(370));
        // -50 m/s descent (state z velocity points down).
        let state = fixture_state(1000.0, -50.0);
        let env = null_env();
        let c_d = 1.5;
        let area = 2.0;
        let snapshot = recovery_snapshot_map(&[(id_a, true, c_d, area)]);
        let f = adapter
            .force_n_eci(recovery_ctx(&state, &env, 0.0, &snapshot))
            .unwrap();

        // Locked-order recomputation matches the impl exactly:
        // sum_cd_area = 0.0 + c_d * area
        let sum_cd_area = 0.0_f64 + c_d * area;
        let speed_sq = 0.0_f64 * 0.0 + 0.0 * 0.0 + (-50.0) * (-50.0);
        let atm_sample = atm.sample(1000.0, SimTime::ZERO).unwrap();
        let q = 0.5 * atm_sample.density_kg_m3 * speed_sq;
        let drag_magnitude = q * sum_cd_area;
        // f = -drag_magnitude * v / speed; v_hat = v / sqrt(speed_sq).
        let speed = speed_sq.sqrt();
        let expected_z = -drag_magnitude * (-50.0_f64 / speed);

        // x / y vanish by orthogonality but carry a -0.0 sign because
        // `-drag_magnitude * (0.0/speed)` evaluates to negative zero;
        // assert magnitude (numeric equality) for those.
        assert_eq!(f.x, 0.0);
        assert_eq!(f.y, 0.0);
        assert_eq!(f.z.to_bits(), expected_z.to_bits());
        assert!(expected_z > 0.0, "drag must oppose downward velocity");
    }

    #[test]
    fn recovery_rack_sums_multiple_deployed_devices_in_declared_order() {
        let id_a = RecoveryId::from_path("recovery.drogue");
        let id_b = RecoveryId::from_path("recovery.main");
        let atm = IsothermalAtmosphere::ussa_sea_level();
        let adapter = RecoveryRackForceAdapter::new(vec![id_a, id_b], atm, ModelId::new(370));
        let state = fixture_state(500.0, -40.0);
        let env = null_env();
        let snapshot = recovery_snapshot_map(&[
            (id_a, true, 1.0, 0.5), // drogue: small
            (id_b, true, 1.5, 4.0), // main: large
        ]);
        let f = adapter
            .force_n_eci(recovery_ctx(&state, &env, 0.0, &snapshot))
            .unwrap();

        // Locked-order recomputation matching the impl exactly:
        // sum_cd_area = ((0.0 + 1.0 * 0.5) + 1.5 * 4.0)
        let sum_cd_area = 0.0_f64 + 1.0 * 0.5 + 1.5 * 4.0;
        let speed_sq = 0.0_f64 * 0.0 + 0.0 * 0.0 + (-40.0) * (-40.0);
        let atm_sample = atm.sample(500.0, SimTime::ZERO).unwrap();
        let q = 0.5 * atm_sample.density_kg_m3 * speed_sq;
        let drag_magnitude = q * sum_cd_area;
        let speed = speed_sq.sqrt();
        let expected_z = -drag_magnitude * (-40.0_f64 / speed);
        assert_eq!(f.z.to_bits(), expected_z.to_bits());
    }

    #[test]
    fn recovery_rack_filters_drag_by_active_body_owner() {
        let id_upper = RecoveryId::from_path("recovery.upper");
        let id_lower = RecoveryId::from_path("recovery.lower");
        let body_upper = BodyId::from_path("vehicle.assembly.bodies.upper");
        let body_lower = BodyId::from_path("vehicle.assembly.bodies.lower");
        let owners = BTreeMap::from([(id_upper, body_upper), (id_lower, body_lower)]);
        let atm = IsothermalAtmosphere::ussa_sea_level();
        let adapter = RecoveryRackForceAdapter::new_with_owners(
            vec![id_upper, id_lower],
            owners,
            atm,
            ModelId::new(370),
        );
        let state = fixture_state(500.0, -40.0);
        let env = null_env();
        let snapshot =
            recovery_snapshot_map(&[(id_upper, true, 1.0, 0.5), (id_lower, true, 1.5, 4.0)]);

        let mut upper_ctx = recovery_ctx(&state, &env, 0.0, &snapshot);
        upper_ctx.active_body = Some(body_upper);
        let upper_force = adapter.force_n_eci(upper_ctx).unwrap();

        let mut lower_ctx = recovery_ctx(&state, &env, 0.0, &snapshot);
        lower_ctx.active_body = Some(body_lower);
        let lower_force = adapter.force_n_eci(lower_ctx).unwrap();

        let all_force = adapter
            .force_n_eci(recovery_ctx(&state, &env, 0.0, &snapshot))
            .unwrap();
        assert!(
            lower_force.z > upper_force.z * 10.0,
            "lower body should receive only its larger recovery drag"
        );
        assert!((all_force.z - (upper_force.z + lower_force.z)).abs() < 1.0e-9);
    }

    #[test]
    fn recovery_rack_missing_owner_fails_for_active_body() {
        let id_upper = RecoveryId::from_path("recovery.upper");
        let id_lower = RecoveryId::from_path("recovery.lower");
        let body_upper = BodyId::from_path("vehicle.assembly.bodies.upper");
        let body_lower = BodyId::from_path("vehicle.assembly.bodies.lower");
        let owners = BTreeMap::from([(id_upper, body_upper)]);
        let atm = IsothermalAtmosphere::ussa_sea_level();
        let adapter = RecoveryRackForceAdapter::new_with_owners(
            vec![id_upper, id_lower],
            owners,
            atm,
            ModelId::new(370),
        );
        let state = fixture_state(500.0, -40.0);
        let env = null_env();
        let snapshot =
            recovery_snapshot_map(&[(id_upper, true, 1.0, 0.5), (id_lower, true, 1.5, 4.0)]);
        let mut ctx = recovery_ctx(&state, &env, 0.0, &snapshot);
        ctx.active_body = Some(body_lower);
        let err = adapter.force_n_eci(ctx).unwrap_err();
        assert!(matches!(err, ModelEvalError::InvalidState { .. }));
    }

    #[test]
    fn recovery_rack_missing_id_in_snapshot_is_out_of_envelope() {
        let id_a = RecoveryId::from_path("recovery.a");
        let id_b = RecoveryId::from_path("recovery.b");
        let atm = IsothermalAtmosphere::ussa_sea_level();
        let adapter = RecoveryRackForceAdapter::new(vec![id_a, id_b], atm, ModelId::new(370));
        let state = fixture_state(1000.0, -50.0);
        let env = null_env();
        // Snapshot missing id_b.
        let snapshot = recovery_snapshot_map(&[(id_a, true, 1.5, 2.0)]);
        let err = adapter
            .force_n_eci(recovery_ctx(&state, &env, 0.0, &snapshot))
            .unwrap_err();
        assert!(matches!(err, ModelEvalError::OutOfEnvelope { .. }));
    }

    #[test]
    fn recovery_rack_byte_stable_across_two_evaluations() {
        let id_a = RecoveryId::from_path("recovery.main");
        let atm = IsothermalAtmosphere::ussa_sea_level();
        let adapter = RecoveryRackForceAdapter::new(vec![id_a], atm, ModelId::new(370));
        let state = fixture_state(1500.0, -42.0);
        let env = null_env();
        let snapshot = recovery_snapshot_map(&[(id_a, true, 1.6, 3.5)]);
        let f1 = adapter
            .force_n_eci(recovery_ctx(&state, &env, 0.0, &snapshot))
            .unwrap();
        let f2 = adapter
            .force_n_eci(recovery_ctx(&state, &env, 0.0, &snapshot))
            .unwrap();
        assert_eq!(f1.x.to_bits(), f2.x.to_bits());
        assert_eq!(f1.y.to_bits(), f2.y.to_bits());
        assert_eq!(f1.z.to_bits(), f2.z.to_bits());
    }
}
