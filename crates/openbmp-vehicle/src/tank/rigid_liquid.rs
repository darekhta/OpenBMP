//! [`RigidLiquid`] — Phase-3.7.A toy [`MovingMassModel`] with no
//! slosh dynamics.
//!
//! Behaviour:
//!
//! - Mass = current fluid mass; CG at the tank mount point
//!   (`cg_offset_body_m = 0`).
//! - Inertia delta about parent body origin = `m · (||r||² · I3 - r
//!   · rᵀ)` where `r = mount_point_body_m`. Standard parallel-axis
//!   for a point mass at the mount.
//! - Reaction force / moment back on the parent body = zero. The
//!   fluid is rigid relative to the body, so no slosh-driven
//!   feedback. This is the documented Phase-3.7 baseline.
//! - `drain(kg/s)` sets the next [`MovingMassModel::step`] drain
//!   rate. The step subtracts `rate · dt` from the fluid mass and
//!   clamps at zero.
//!
//! Use case: a payload bay with a fixed mass distribution, or a
//! tank where slosh is irrelevant for the scenario being simulated.
//! Not physical for sounding-rocket sloshing — use
//! [`crate::tank::equivalent_pendulum::EquivalentPendulum`]
//! (Phase 3.7.B) for that.

use nalgebra::Vector3;
use openbmp_core::Duration;

use super::{
    point_mass_inertia_about_origin, ForceMomentBody, MassContribution, MovingMassModel,
    PropellantSpec, TankError, TankGeometry,
};

/// Phase-3.7 no-slosh toy moving mass.
#[derive(Debug, Clone)]
pub struct RigidLiquid {
    fluid_kg: f64,
    mount_point_body_m: Vector3<f64>,
    pending_drain_kg_per_s: f64,
}

impl RigidLiquid {
    /// Construct a [`RigidLiquid`] sized from `geometry × density ×
    /// fill_fraction`.
    ///
    /// # Errors
    ///
    /// Returns [`TankError`] for invalid geometry / propellant /
    /// fill fraction / mount point.
    pub fn new(
        geometry: TankGeometry,
        propellant: PropellantSpec,
        fill_fraction: f64,
        mount_point_body_m: Vector3<f64>,
    ) -> Result<Self, TankError> {
        geometry.require_valid()?;
        propellant.require_valid()?;
        if !fill_fraction.is_finite() || !(0.0..=1.0).contains(&fill_fraction) {
            return Err(TankError::InvalidFillFraction { value: fill_fraction });
        }
        if !mount_point_body_m.iter().all(|c| c.is_finite()) {
            return Err(TankError::InvalidMountPoint);
        }
        let fluid_kg = geometry.volume_m3() * propellant.density_kg_m3 * fill_fraction;
        Ok(Self {
            fluid_kg,
            mount_point_body_m,
            pending_drain_kg_per_s: 0.0,
        })
    }
}

impl MovingMassModel for RigidLiquid {
    fn drain(&mut self, kg_per_s: f64) -> Result<(), TankError> {
        if !kg_per_s.is_finite() || kg_per_s < 0.0 {
            return Err(TankError::InvalidDrainRate { value: kg_per_s });
        }
        self.pending_drain_kg_per_s = kg_per_s;
        Ok(())
    }

    fn step(
        &mut self,
        accel_body_m_s2: Vector3<f64>,
        omega_body_rad_s: Vector3<f64>,
        dt: Duration,
    ) -> Result<(), TankError> {
        if !accel_body_m_s2.iter().all(|c| c.is_finite()) {
            return Err(TankError::NonFiniteStepInput {
                reason: "accel_body_m_s2 contains a non-finite component",
            });
        }
        if !omega_body_rad_s.iter().all(|c| c.is_finite()) {
            return Err(TankError::NonFiniteStepInput {
                reason: "omega_body_rad_s contains a non-finite component",
            });
        }
        let dt_s = dt.as_seconds();
        if !dt_s.is_finite() || dt_s < 0.0 {
            return Err(TankError::NonFiniteStepInput {
                reason: "dt must be finite and non-negative",
            });
        }
        let drained_kg = self.pending_drain_kg_per_s * dt_s;
        let next = self.fluid_kg - drained_kg;
        self.fluid_kg = if next > 0.0 { next } else { 0.0 };
        Ok(())
    }

    fn mass_contribution(&self) -> MassContribution {
        MassContribution {
            mass_kg: self.fluid_kg,
            cg_offset_body_m: Vector3::zeros(),
            inertia_delta_body_kg_m2: point_mass_inertia_about_origin(
                self.fluid_kg,
                self.mount_point_body_m,
            ),
        }
    }

    fn reaction_body(&self) -> ForceMomentBody {
        ForceMomentBody {
            force_body_n: Vector3::zeros(),
            moment_body_n_m: Vector3::zeros(),
        }
    }

    fn fluid_remaining_kg(&self) -> f64 {
        self.fluid_kg
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests {
    use super::*;

    fn finite_propellant() -> PropellantSpec {
        PropellantSpec {
            density_kg_m3: 1000.0,
            label: "water",
        }
    }

    fn unit_sphere() -> TankGeometry {
        TankGeometry::Sphere { radius_m: 1.0 }
    }

    #[test]
    fn initial_fluid_matches_volume_density_fill() {
        let model = RigidLiquid::new(unit_sphere(), finite_propellant(), 0.5, Vector3::zeros())
            .unwrap();
        let expected = (4.0 / 3.0) * std::f64::consts::PI * 1000.0 * 0.5;
        assert!((model.fluid_remaining_kg() - expected).abs() < 1e-9);
    }

    #[test]
    fn drain_reduces_fluid_by_rate_times_dt() {
        let mut model = RigidLiquid::new(unit_sphere(), finite_propellant(), 1.0, Vector3::zeros())
            .unwrap();
        let initial = model.fluid_remaining_kg();
        model.drain(10.0).unwrap();
        model
            .step(Vector3::zeros(), Vector3::zeros(), Duration::from_seconds(0.5))
            .unwrap();
        let expected = initial - 10.0 * 0.5;
        assert!((model.fluid_remaining_kg() - expected).abs() < 1e-9);
    }

    #[test]
    fn drain_clamps_at_zero_when_depleted() {
        let mut model = RigidLiquid::new(unit_sphere(), finite_propellant(), 0.001, Vector3::zeros())
            .unwrap();
        model.drain(1000.0).unwrap();
        model
            .step(Vector3::zeros(), Vector3::zeros(), Duration::from_seconds(1.0))
            .unwrap();
        assert!((model.fluid_remaining_kg() - 0.0).abs() < 1e-12);
    }

    #[test]
    fn drain_rejects_negative_rate() {
        let mut model = RigidLiquid::new(unit_sphere(), finite_propellant(), 1.0, Vector3::zeros())
            .unwrap();
        assert!(matches!(
            model.drain(-1.0),
            Err(TankError::InvalidDrainRate { .. })
        ));
    }

    #[test]
    fn drain_rejects_nan_rate() {
        let mut model = RigidLiquid::new(unit_sphere(), finite_propellant(), 1.0, Vector3::zeros())
            .unwrap();
        assert!(matches!(
            model.drain(f64::NAN),
            Err(TankError::InvalidDrainRate { .. })
        ));
    }

    #[test]
    fn step_rejects_non_finite_accel() {
        let mut model = RigidLiquid::new(unit_sphere(), finite_propellant(), 1.0, Vector3::zeros())
            .unwrap();
        assert!(matches!(
            model.step(
                Vector3::new(f64::NAN, 0.0, 0.0),
                Vector3::zeros(),
                Duration::from_seconds(0.001)
            ),
            Err(TankError::NonFiniteStepInput { .. })
        ));
    }

    #[test]
    fn mass_contribution_at_zero_mount_has_zero_inertia() {
        let model = RigidLiquid::new(unit_sphere(), finite_propellant(), 1.0, Vector3::zeros())
            .unwrap();
        let contribution = model.mass_contribution();
        assert!(contribution.inertia_delta_body_kg_m2.iter().all(|c| c.abs() < 1e-9));
        assert!(contribution.cg_offset_body_m.iter().all(|c| c.abs() < 1e-12));
        assert!(contribution.mass_kg > 0.0);
    }

    #[test]
    fn mass_contribution_at_offset_mount_uses_parallel_axis() {
        let mount = Vector3::new(0.0, 0.0, 2.0);
        let model = RigidLiquid::new(unit_sphere(), finite_propellant(), 1.0, mount).unwrap();
        let contribution = model.mass_contribution();
        // For a point mass at (0, 0, 2): I_xx = I_yy = m·d², I_zz = 0.
        let m = contribution.mass_kg;
        let d2 = 4.0;
        let i = contribution.inertia_delta_body_kg_m2;
        assert!((i[(0, 0)] - m * d2).abs() < 1e-9);
        assert!((i[(1, 1)] - m * d2).abs() < 1e-9);
        assert!((i[(2, 2)] - 0.0).abs() < 1e-9);
        assert!(i[(0, 1)].abs() < 1e-12);
        assert!(i[(0, 2)].abs() < 1e-12);
        assert!(i[(1, 2)].abs() < 1e-12);
    }

    #[test]
    fn reaction_force_is_always_zero() {
        let mut model = RigidLiquid::new(unit_sphere(), finite_propellant(), 1.0, Vector3::new(1.0, 2.0, 3.0))
            .unwrap();
        // Apply some accel/omega and verify reaction stays zero.
        model
            .step(
                Vector3::new(10.0, 5.0, 9.81),
                Vector3::new(0.1, 0.2, 0.3),
                Duration::from_seconds(0.001),
            )
            .unwrap();
        let r = model.reaction_body();
        assert!(r.force_body_n.iter().all(|c| c.abs() < 1e-12));
        assert!(r.moment_body_n_m.iter().all(|c| c.abs() < 1e-12));
    }

    #[test]
    fn determinism_repeated_drain_step_byte_stable() {
        // Two parallel models with identical inputs must produce
        // bit-identical fluid_remaining traces over many steps.
        let geometry = unit_sphere();
        let propellant = finite_propellant();
        let mount = Vector3::new(0.5, 0.0, 1.0);
        let mut a = RigidLiquid::new(geometry, propellant, 1.0, mount).unwrap();
        let mut b = RigidLiquid::new(geometry, propellant, 1.0, mount).unwrap();
        for step in 0_i32..1000 {
            let rate = f64::from(step).sin().abs() * 5.0;
            a.drain(rate).unwrap();
            b.drain(rate).unwrap();
            let dt = Duration::from_seconds(0.001);
            a.step(Vector3::new(0.0, 0.0, -9.81), Vector3::zeros(), dt)
                .unwrap();
            b.step(Vector3::new(0.0, 0.0, -9.81), Vector3::zeros(), dt)
                .unwrap();
            assert_eq!(a.fluid_remaining_kg().to_bits(), b.fluid_remaining_kg().to_bits());
        }
    }
}
