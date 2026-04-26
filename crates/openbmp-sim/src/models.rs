//! Model trait declarations and Phase-1.3 simple implementations.
//!
//! Phase 1.3 ships:
//!
//! * [`ForceModel`] + [`ConstantGravityForce`].
//! * [`MassModel`] + [`ConstantMass`] (and [`LinearBurnMass`] for
//!   testing variable-mass integrator behaviour).
//! * [`EnvironmentModel`] + [`NullEnvironment`].
//!
//! The trait shapes intentionally take a `PointMassState`-flavoured
//! [`ForceContext`] for now. When rigid-body integration lands they
//! will either be made generic over the state type or replaced by
//! parallel `MomentModel` / rigid-body-specific contexts.
//!
//! No model in this module accesses wall-clock time, system RNG,
//! network, or the file system.

use nalgebra::Vector3;
use openbmp_core::{Eci, Position3, SimTime, ValidationStatus};
use openbmp_state::PointMassState;
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

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
/// Phase 1.3 carries only a gravity field; atmosphere, wind, and
/// magnetic field land in Phase 2.
#[derive(Copy, Clone, Debug, Default)]
pub struct EnvironmentSample {
    /// Local gravitational acceleration in `Eci`, m/s².
    pub gravity_eci_m_s2: Vector3<f64>,
}

/// Trait implemented by environment-providing models.
pub trait EnvironmentModel {
    /// Sample the environment at the given query.
    fn sample(&self, query: EnvironmentQuery) -> EnvironmentSample;

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
    fn sample(&self, _query: EnvironmentQuery) -> EnvironmentSample {
        EnvironmentSample::default()
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
/// Borrowed lifetime ties to the kernel's owned models / environment;
/// the model receives an immutable view of the current state, the
/// current environment sample, and the current sub-step time.
#[derive(Copy, Clone, Debug)]
pub struct ForceContext<'a> {
    /// Current (possibly sub-step) state.
    pub state: &'a PointMassState,
    /// Environment sample evaluated at this state and time.
    pub environment: &'a EnvironmentSample,
    /// Sub-step time. May be the kernel's published time
    /// (start-of-step) or one of the RK4 intermediate times.
    pub time: SimTime,
}

/// Trait implemented by force-providing models.
///
/// The model returns a total force in `Eci`, in Newtons. The
/// integrator divides by mass to get acceleration.
pub trait ForceModel {
    /// Total force in `Eci`, in Newtons.
    fn force_n_eci(&self, ctx: ForceContext<'_>) -> Vector3<f64>;

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

impl ForceModel for ConstantGravityForce {
    fn force_n_eci(&self, ctx: ForceContext<'_>) -> Vector3<f64> {
        let mass_kg = ctx.state.mass.get::<kilogram>();
        // Locked order: scalar * vector, no FMA.
        mass_kg * self.g_eci_m_s2
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

/// Zero-force model. Useful for inertial-coast scenarios and tests.
#[derive(Copy, Clone, Debug, Default)]
pub struct ZeroForce;

impl ForceModel for ZeroForce {
    fn force_n_eci(&self, _ctx: ForceContext<'_>) -> Vector3<f64> {
        Vector3::zeros()
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::Checked
    }
}

// ---------------------------------------------------------------------
// Mass
// ---------------------------------------------------------------------

/// Trait implemented by mass-property-providing models.
///
/// Phase 1.3 surfaces only mass and mass-rate (point-mass). Full
/// rigid-body mass-property models land with Phase 1.4 (inertia
/// tensor + center of mass evolution).
pub trait MassModel {
    /// Total mass at simulation time `t` (kg).
    fn mass_kg(&self, t: SimTime) -> f64;

    /// Time derivative of mass at `t` (kg/s). Negative for mass loss
    /// (propellant burn). Zero for [`ConstantMass`].
    fn mass_rate_kg_s(&self, t: SimTime) -> f64;

    /// Convenience: typed mass at `t`.
    #[must_use]
    fn mass(&self, t: SimTime) -> Mass {
        Mass::new::<kilogram>(self.mass_kg(t))
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
    fn mass_kg(&self, _t: SimTime) -> f64 {
        self.mass_kg
    }

    fn mass_rate_kg_s(&self, _t: SimTime) -> f64 {
        0.0
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
    fn mass_kg(&self, t: SimTime) -> f64 {
        self.m0_kg + self.rate_kg_s * (t.as_seconds() - self.t0_s)
    }

    fn mass_rate_kg_s(&self, _t: SimTime) -> f64 {
        self.rate_kg_s
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
        let f = g.force_n_eci(ForceContext {
            state: &state,
            environment: &env,
            time: SimTime::ZERO,
        });
        // mass=2.5, g=9.80665 → force_z = -2.5 * 9.80665 = -24.516625
        assert_abs_diff_eq!(f.x, 0.0);
        assert_abs_diff_eq!(f.y, 0.0);
        assert_abs_diff_eq!(f.z, -2.5 * 9.80665, epsilon = 1.0e-12);
    }

    #[test]
    fn zero_force_returns_zero_vector() {
        let state = sample_state();
        let env = EnvironmentSample::default();
        let f = ZeroForce.force_n_eci(ForceContext {
            state: &state,
            environment: &env,
            time: SimTime::ZERO,
        });
        assert_abs_diff_eq!(f.norm(), 0.0);
    }

    #[test]
    fn constant_mass_returns_constant_value() {
        let m = ConstantMass::new(1.5);
        assert_abs_diff_eq!(m.mass_kg(SimTime::ZERO), 1.5);
        assert_abs_diff_eq!(m.mass_kg(SimTime::from_seconds(100.0)), 1.5);
        assert_abs_diff_eq!(m.mass_rate_kg_s(SimTime::ZERO), 0.0);
    }

    #[test]
    fn linear_burn_mass_evolves_linearly() {
        let m = LinearBurnMass::new(0.0, 10.0, -0.5);
        assert_abs_diff_eq!(m.mass_kg(SimTime::ZERO), 10.0);
        assert_abs_diff_eq!(m.mass_kg(SimTime::from_seconds(2.0)), 9.0);
        assert_abs_diff_eq!(m.mass_kg(SimTime::from_seconds(20.0)), 0.0);
        assert_abs_diff_eq!(m.mass_rate_kg_s(SimTime::ZERO), -0.5);
    }

    #[test]
    fn null_environment_returns_zero_gravity() {
        let env = NullEnvironment;
        let s = env.sample(EnvironmentQuery {
            time: SimTime::ZERO,
            position_eci: Position3::origin(),
        });
        assert_abs_diff_eq!(s.gravity_eci_m_s2.norm(), 0.0);
    }

    #[test]
    fn validation_labels_are_checked_for_phase_1_3_models() {
        assert_eq!(
            ConstantGravityForce::down_z(9.81).validation(),
            ValidationStatus::Checked
        );
        assert_eq!(ZeroForce.validation(), ValidationStatus::Checked);
        assert_eq!(
            ConstantMass::new(1.0).validation(),
            ValidationStatus::Checked
        );
        assert_eq!(NullEnvironment.validation(), ValidationStatus::Checked);
    }
}
