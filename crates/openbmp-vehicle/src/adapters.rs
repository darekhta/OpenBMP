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
//! # Phase 2.9 scope
//!
//! All adapters here target `PointMassState`. Rigid-body adapters
//! (alpha/beta from attitude, body-frame aero composition, motor
//! thrust along body `+x`) land in Phase 3 alongside the rigid-body
//! aero coupling. Phase 2.9 ships a vertical-launch sounding-rocket
//! validation built on these point-mass adapters.

use nalgebra::Vector3;

use openbmp_aero::AeroDeck;
use openbmp_env::{AtmosphereModel, GravityModel};
use openbmp_propulsion::Motor;
use openbmp_sim::{ForceContext, ForceModel, MassModel, ModelEvalError};
use openbmp_state::PointMassState;

use openbmp_core::{ModelId, SimTime, ValidationStatus};
use std::borrow::Cow;

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

// ---------------------------------------------------------------------
// MotorThrustForceAdapter
// ---------------------------------------------------------------------

/// Wraps an [`openbmp_propulsion::Motor`] as a kernel-side
/// [`ForceModel<PointMassState>`].
///
/// Phase-2 vertical-launch convention: thrust is applied along ECI
/// `+z`. Future rigid-body adapter rotates body-`+x` thrust into
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
        let thrust_n = self.motor.thrust_n_at(t_since).map_err(|_| {
            ModelEvalError::OutOfEnvelope {
                model: self.model_id,
                reason: Cow::Borrowed("motor thrust query failed"),
            }
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
        let motor_mass = self.motor.mass_kg(t_since).map_err(|_| {
            ModelEvalError::OutOfEnvelope {
                model: self.model_id,
                reason: Cow::Borrowed("motor mass query failed"),
            }
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
        let rate = self.motor.mass_rate_kg_s(t_since).map_err(|_| {
            ModelEvalError::OutOfEnvelope {
                model: self.model_id,
                reason: Cow::Borrowed("motor mass-rate query failed"),
            }
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
        let v = ctx.state.velocity.vector;
        let speed_sq = v.x * v.x + v.y * v.y + v.z * v.z;
        if speed_sq <= 0.0 {
            // Zero velocity → zero drag. No need to evaluate the
            // atmosphere or the deck.
            return Ok(Vector3::zeros());
        }
        let speed = speed_sq.sqrt();

        // Altitude proxy: ECI z (vertical-launch simplification).
        let altitude_m = ctx.state.position.vector.z;
        let atm_sample = self.atmosphere.sample(altitude_m, ctx.time).map_err(|_| {
            ModelEvalError::OutOfEnvelope {
                model: self.model_id,
                reason: Cow::Borrowed("atmosphere out of envelope at altitude"),
            }
        })?;

        let speed_of_sound = atm_sample.speed_of_sound_m_s;
        if speed_of_sound <= 0.0 || !speed_of_sound.is_finite() {
            return Err(ModelEvalError::OutOfEnvelope {
                model: self.model_id,
                reason: Cow::Borrowed("atmosphere returned non-positive speed of sound"),
            });
        }

        let mach = speed / speed_of_sound;
        let coefficients = self.deck.lookup(mach, 0.0, 0.0).map_err(|_| {
            ModelEvalError::OutOfEnvelope {
                model: self.model_id,
                reason: Cow::Borrowed("aero deck out of envelope at (mach, 0, 0)"),
            }
        })?;

        // Locked operand order: q = 0.5 · ρ · |v|².
        let q = 0.5 * atm_sample.density_kg_m3 * speed_sq;
        let drag_magnitude = coefficients.cd * q * self.deck.reference_area_m2();

        // Force opposes the velocity unit vector.
        let v_hat = v / speed;
        let f = -drag_magnitude * v_hat;
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
            adapter
                .mass_rate_kg_s(SimTime::from_seconds(-0.1))
                .unwrap(),
            0.0
        );
        assert_eq!(
            adapter
                .mass_rate_kg_s(SimTime::from_seconds(10.0))
                .unwrap(),
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
}
