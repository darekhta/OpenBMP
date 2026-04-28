//! Kernel-side adapters that wrap L2 physics models
//! (`openbmp-env`, `openbmp-aero`, `openbmp-propulsion`) into
//! `openbmp-sim` `ForceModel` / `MassModel` impls consumable by
//! the kernel and by [`crate::BasicVehicle`].
//!
//! `openbmp-vehicle` is the architectural seam where L2 physics
//! flow into L1 kernel surfaces. Each adapter owns a generic over
//! its underlying physics trait so a deck or motor or atmosphere
//! can swap without touching the adapter.
//!
//! # Scope
//!
//! Phase 2.9 shipped point-mass force adapters for the
//! Phase-2.11 sounding-rocket runner: a vertical-launch convention
//! where thrust is along ECI `+z` (no attitude). Phase 3.1 adds
//! the rigid-body counterparts:
//!
//! - The same `GravityForceAdapter`, `MotorThrustForceAdapter`, and
//!   `AxialDragForceAdapter` structs gain `ForceModel<RigidBodyState>`
//!   impls. Gravity and aero are state-symmetric (they read
//!   `state.position` / `state.velocity`, both fields exist on both
//!   states). Motor thrust differs: the rigid impl rotates body-`+z`
//!   thrust into ECI through the attitude quaternion, lifting the
//!   point-mass-only assumption that the rocket points along ECI
//!   `+z` for the entire flight.
//! - A new `RigidMotorMassAdapter` struct implements
//!   `RigidMassModel`, wrapping a `Motor` plus the vehicle dry mass /
//!   body-frame CG offset / body-frame inertia tensor. Phase-3.1
//!   uses fixed CG and inertia (`mass_properties_rate` returns zero
//!   for those fields); the motor's mass-rate is the only non-zero
//!   contribution. Phase 3.6 / 3.7 will add motor and tank inertia
//!   derivatives.

use nalgebra::Vector3;

use openbmp_aero::AeroDeck;
use openbmp_env::{AtmosphereModel, GravityModel};
use openbmp_propulsion::Motor;
use openbmp_sim::{
    ForceContext, ForceModel, MassModel, MassPropertiesRate, ModelEvalError, RigidMassModel,
};
use openbmp_state::{MassProperties, PointMassState, RigidBodyState};

use openbmp_core::{Body, ModelId, Position3, SimTime, ValidationStatus};
use std::borrow::Cow;
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

// ---------------------------------------------------------------------
// GravityForceAdapter
// ---------------------------------------------------------------------

/// Wraps an [`openbmp_env::GravityModel`] as a kernel-side
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
/// Phase-2 vertical-launch convention: thrust is applied along ECI
/// `+z`. The rigid-body adapter rotates body-`+z` thrust into
/// ECI via the attitude quaternion.
#[derive(Copy, Clone, Debug)]
pub struct MotorThrustForceAdapter<M> {
    motor: M,
    ignition_time_s: f64,
    model_id: ModelId,
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
// AxialDragForceAdapter
// ---------------------------------------------------------------------

/// Point-mass axial-drag adapter.
///
/// Wraps an [`openbmp_aero::AeroDeck`] + [`openbmp_env::AtmosphereModel`]
/// as a `ForceModel<PointMassState>` that produces only axial drag
/// (`F = -CD · q · S · v_hat`) opposing the vehicle's ECI velocity.
/// Wind is assumed zero — Phase-3 will introduce a wind-aware
/// rigid-body adapter that resolves wind into the body frame.
///
/// Altitude for the atmosphere lookup is taken as the ECI `z`
/// component of position. This is the vertical-launch simplification
/// the Phase-2.9 plan locks in: launches are from sea level along
/// `+z` so `position.vector.z` is a good proxy for geometric
/// altitude. Phase 3 will introduce a `LocalGeodeticOrigin`-aware
/// altitude resolver.
///
/// CD is sampled at `(mach, alpha=0, beta=0)`. Non-axial aero
/// (CN, CM in body frame) is intentionally not consumed by this
/// adapter — that is rigid-body work.
#[derive(Clone, Debug)]
pub struct AxialDragForceAdapter<Atm> {
    deck: AeroDeck,
    atmosphere: Atm,
    model_id: ModelId,
}

impl<Atm> AxialDragForceAdapter<Atm> {
    /// Construct from a deck, an atmosphere model, and a stable
    /// model id.
    #[must_use]
    pub const fn new(deck: AeroDeck, atmosphere: Atm, model_id: ModelId) -> Self {
        Self {
            deck,
            atmosphere,
            model_id,
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

impl<Atm: AtmosphereModel> ForceModel<PointMassState> for AxialDragForceAdapter<Atm> {
    fn force_n_eci(
        &self,
        ctx: ForceContext<'_, PointMassState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        compute_axial_drag(
            &self.deck,
            &self.atmosphere,
            self.model_id,
            ctx.state.velocity.vector,
            ctx.state.position.vector.z,
            ctx.time,
        )
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

impl<Atm: AtmosphereModel> ForceModel<RigidBodyState> for AxialDragForceAdapter<Atm> {
    fn force_n_eci(
        &self,
        ctx: ForceContext<'_, RigidBodyState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        // Axial-drag adapter is axisymmetric: drag opposes the ECI
        // velocity vector, magnitude depends only on speed and
        // altitude. RigidBodyState carries the same `velocity` and
        // `position` fields PointMassState does, so the rigid impl
        // delegates to the shared computation. Wind and aero
        // moment / sideslip are Phase-3 follow-ons (3.5/3.6/3.8).
        compute_axial_drag(
            &self.deck,
            &self.atmosphere,
            self.model_id,
            ctx.state.velocity.vector,
            ctx.state.position.vector.z,
            ctx.time,
        )
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

/// Shared axial-drag force computation used by both the point-mass
/// and rigid-body `AxialDragForceAdapter` impls. Operand order is
/// locked here so a future refactor can't accidentally diverge the
/// two paths.
fn compute_axial_drag<Atm: AtmosphereModel>(
    deck: &AeroDeck,
    atmosphere: &Atm,
    model_id: ModelId,
    velocity_eci: Vector3<f64>,
    position_eci_z: f64,
    time: SimTime,
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
    // Phase-3.5: Schema-1 decks ignore the deflections map. Schema-2
    // adapter wiring lands in 3.5.C; for now the `AxialDragForceAdapter`
    // continues to query at (mach, 0, 0) with no effector axes.
    let deflections = std::collections::BTreeMap::<&str, f64>::new();
    let coefficients =
        deck.lookup(mach, 0.0, 0.0, &deflections)
            .map_err(|_| ModelEvalError::OutOfEnvelope {
                model: model_id,
                reason: Cow::Borrowed("aero deck out of envelope at (mach, 0, 0)"),
            })?;

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

// ---------------------------------------------------------------------
// RigidMotorMassAdapter
// ---------------------------------------------------------------------

/// Rigid-body counterpart to [`MotorMassAdapter`]: wraps a
/// [`Motor`] plus the vehicle's dry mass, body-frame CG offset, and
/// body-frame inertia tensor as a kernel-side
/// [`RigidMassModel`].
///
/// Phase 3.1 simplification: CG and inertia are held fixed
/// (constructor values). The motor's mass-rate is the only
/// non-zero `MassPropertiesRate` component. Phase 3.6 / 3.7 will
/// extend this with motor-internal inertia derivatives and
/// tank-driven CG / inertia changes.
#[derive(Copy, Clone, Debug)]
pub struct RigidMotorMassAdapter<M> {
    motor: M,
    dry_vehicle_mass_kg: f64,
    dry_center_of_mass_body: Position3<Body>,
    dry_inertia_body: nalgebra::Matrix3<f64>,
    ignition_time_s: f64,
    model_id: ModelId,
}

impl<M> RigidMotorMassAdapter<M> {
    /// Construct from `(motor, dry_vehicle_mass_kg,
    /// dry_center_of_mass_body, dry_inertia_body, ignition_time_s,
    /// model_id)`. The `dry_*` fields are the vehicle's body-frame
    /// CG and inertia; the motor contributes only mass (no inertia
    /// derivative in Phase 3.1).
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
            // Phase 3.1: CG and inertia are constant. The motor
            // contributes only mass-flow; CG-shift and inertia
            // derivatives land in Phase 3.6 / 3.7.
            center_of_mass_rate_body_m_s: Vector3::zeros(),
            inertia_rate_body: nalgebra::Matrix3::zeros(),
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
    use openbmp_core::{Position3, SimTime, Velocity3};
    use openbmp_env::{ConstantGravity, IsothermalAtmosphere};
    use openbmp_propulsion::SolidMotor;
    use openbmp_sim::EnvironmentSample;
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
    // AxialDragForceAdapter
    // -----------------------------------------------------------------

    #[test]
    fn axial_drag_adapter_zero_velocity_returns_zero_force() {
        let atm = IsothermalAtmosphere::ussa_sea_level();
        let deck = small_drag_deck();
        let adapter = AxialDragForceAdapter::new(deck, atm, ModelId::new(0));
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
        let adapter = AxialDragForceAdapter::new(deck, atm, ModelId::new(0));
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

    // =================================================================
    // Phase 3.1: rigid-body adapter impls
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
    // Rigid AxialDragForceAdapter
    // -----------------------------------------------------------------

    #[test]
    fn rigid_axial_drag_matches_point_mass_at_identity_orientation() {
        let atm = IsothermalAtmosphere::ussa_sea_level();
        let deck = small_drag_deck();
        let adapter = AxialDragForceAdapter::new(deck, atm, ModelId::new(0));
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
    fn rigid_motor_mass_adapter_inertia_held_fixed_in_phase_3_1() {
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
}
