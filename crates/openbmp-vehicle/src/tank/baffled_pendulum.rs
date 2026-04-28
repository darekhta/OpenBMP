//! [`BaffledPendulum`] — Phase-3.7.C [`EquivalentPendulum`] with a
//! [`BaffleModel`]-supplied damping increment.
//!
//! Per Abramson SP-106 §7.4 (Eq. 7-46), tank baffles increase the
//! slosh damping ratio approximately linearly with baffle-area-to-
//! tank-cross-section ratio. Phase-3.7 ships the minimum surface:
//! the baffle's `damping_increment_zeta` is added to the bare-tank
//! `damping_ratio_zeta` at construction, producing a single
//! effective damping that the inner [`EquivalentPendulum`] uses.
//! Baffle-area integration and full Eq. 7-46 evaluation are
//! deferred — downstream users can curve-fit and ship a constant
//! `damping_increment_zeta` per baffle configuration.
//!
//! Operationally [`BaffledPendulum`] behaves identically to an
//! [`EquivalentPendulum`] with the additive damping; the wrapper
//! exists so scenarios can declare "this tank has baffles" through
//! configuration without rewriting the model selection.

use nalgebra::Vector3;
use openbmp_core::Duration;

use super::{
    BaffleModel, ForceMomentBody, MassContribution, MovingMassModel, PropellantSpec, TankError,
    TankGeometry, equivalent_pendulum::EquivalentPendulum,
};

/// Phase-3.7.C baffled equivalent-pendulum slosh model.
#[derive(Debug, Clone)]
pub struct BaffledPendulum {
    inner: EquivalentPendulum,
    baffle: BaffleModel,
}

impl BaffledPendulum {
    /// Construct a new baffled pendulum.
    ///
    /// Effective damping ratio = `base_damping_ratio_zeta +
    /// baffle.damping_increment_zeta`. Both must be non-negative.
    ///
    /// # Errors
    ///
    /// Returns [`TankError`] for invalid geometry / propellant /
    /// fill / mount / damping / baffle.
    pub fn new(
        geometry: TankGeometry,
        propellant: PropellantSpec,
        fill_fraction: f64,
        mount_point_body_m: Vector3<f64>,
        base_damping_ratio_zeta: f64,
        baffle: BaffleModel,
    ) -> Result<Self, TankError> {
        baffle.require_valid()?;
        if !base_damping_ratio_zeta.is_finite() || base_damping_ratio_zeta < 0.0 {
            return Err(TankError::InvalidBaffle {
                reason: "base_damping_ratio_zeta must be finite and non-negative",
            });
        }
        let total_zeta = base_damping_ratio_zeta + baffle.damping_increment_zeta;
        let inner = EquivalentPendulum::new(
            geometry,
            propellant,
            fill_fraction,
            mount_point_body_m,
            total_zeta,
        )?;
        Ok(Self { inner, baffle })
    }

    /// Set the initial slosh state.
    ///
    /// # Errors
    ///
    /// Returns [`TankError::NonFiniteStepInput`] for non-finite
    /// inputs.
    pub fn set_initial_slosh(
        &mut self,
        angles_rad: (f64, f64),
        rates_rad_s: (f64, f64),
    ) -> Result<(), TankError> {
        self.inner.set_initial_slosh(angles_rad, rates_rad_s)
    }

    /// Active baffle model.
    #[must_use]
    pub fn baffle(&self) -> BaffleModel {
        self.baffle
    }

    /// Current slosh angles `(θ_x, θ_y)`.
    #[must_use]
    pub fn slosh_angles_rad(&self) -> (f64, f64) {
        self.inner.slosh_angles_rad()
    }

    /// Current slosh rates `(θ̇_x, θ̇_y)`.
    #[must_use]
    pub fn slosh_rates_rad_s(&self) -> (f64, f64) {
        self.inner.slosh_rates_rad_s()
    }
}

impl MovingMassModel for BaffledPendulum {
    fn drain(&mut self, kg_per_s: f64) -> Result<(), TankError> {
        self.inner.drain(kg_per_s)
    }

    fn step(
        &mut self,
        accel_body_m_s2: Vector3<f64>,
        omega_body_rad_s: Vector3<f64>,
        dt: Duration,
    ) -> Result<(), TankError> {
        self.inner.step(accel_body_m_s2, omega_body_rad_s, dt)
    }

    fn mass_contribution(&self) -> MassContribution {
        self.inner.mass_contribution()
    }

    fn reaction_body(&self) -> ForceMomentBody {
        self.inner.reaction_body()
    }

    fn fluid_remaining_kg(&self) -> f64 {
        self.inner.fluid_remaining_kg()
    }
}

#[cfg(test)]
#[allow(
    clippy::cast_lossless,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::expect_used,
    clippy::panic,
    clippy::unwrap_used
)]
mod tests {
    use super::*;

    fn water() -> PropellantSpec {
        PropellantSpec {
            density_kg_m3: 1000.0,
            label: "water_textbook",
        }
    }

    fn cylinder_a05_h2() -> TankGeometry {
        TankGeometry::Cylinder {
            radius_m: 0.5,
            height_m: 2.0,
        }
    }

    #[test]
    fn rejects_negative_base_damping() {
        let result = BaffledPendulum::new(
            cylinder_a05_h2(),
            water(),
            1.0,
            Vector3::zeros(),
            -0.01,
            BaffleModel {
                damping_increment_zeta: 0.05,
            },
        );
        assert!(matches!(result, Err(TankError::InvalidBaffle { .. })));
    }

    #[test]
    fn rejects_negative_baffle_increment() {
        let result = BaffledPendulum::new(
            cylinder_a05_h2(),
            water(),
            1.0,
            Vector3::zeros(),
            0.0,
            BaffleModel {
                damping_increment_zeta: -0.05,
            },
        );
        assert!(matches!(result, Err(TankError::InvalidBaffle { .. })));
    }

    #[test]
    fn baffled_decays_faster_than_unbaffled() {
        let mut bare =
            EquivalentPendulum::new(cylinder_a05_h2(), water(), 1.0, Vector3::zeros(), 0.005)
                .unwrap();
        let mut baffled = BaffledPendulum::new(
            cylinder_a05_h2(),
            water(),
            1.0,
            Vector3::zeros(),
            0.005,
            BaffleModel {
                damping_increment_zeta: 0.05,
            },
        )
        .unwrap();
        bare.set_initial_slosh((0.05, 0.0), (0.0, 0.0)).unwrap();
        baffled.set_initial_slosh((0.05, 0.0), (0.0, 0.0)).unwrap();

        let g = 9.81;
        let dt = Duration::from_seconds(0.001);
        for _ in 0..20_000 {
            bare.step(Vector3::new(0.0, 0.0, g), Vector3::zeros(), dt)
                .unwrap();
            baffled
                .step(Vector3::new(0.0, 0.0, g), Vector3::zeros(), dt)
                .unwrap();
        }
        let (bare_theta, _) = bare.slosh_angles_rad();
        let (baffled_theta, _) = baffled.slosh_angles_rad();
        // The baffled magnitude should be substantially smaller
        // after equal sim time. Tighter bound is overkill.
        assert!(
            baffled_theta.abs() < bare_theta.abs() * 0.5,
            "baffled |θ| {} not less than half bare |θ| {}",
            baffled_theta.abs(),
            bare_theta.abs()
        );
    }

    #[test]
    fn zero_increment_matches_bare_pendulum() {
        // With baffle increment = 0 and same base damping, the
        // BaffledPendulum's state evolution is byte-equal to the
        // bare EquivalentPendulum.
        let mut bare =
            EquivalentPendulum::new(cylinder_a05_h2(), water(), 1.0, Vector3::zeros(), 0.005)
                .unwrap();
        let mut baffled = BaffledPendulum::new(
            cylinder_a05_h2(),
            water(),
            1.0,
            Vector3::zeros(),
            0.005,
            BaffleModel {
                damping_increment_zeta: 0.0,
            },
        )
        .unwrap();
        bare.set_initial_slosh((0.05, 0.0), (0.0, 0.0)).unwrap();
        baffled.set_initial_slosh((0.05, 0.0), (0.0, 0.0)).unwrap();

        let g = 9.81;
        let dt = Duration::from_seconds(0.001);
        for _ in 0..1000 {
            bare.step(Vector3::new(0.0, 0.0, g), Vector3::zeros(), dt)
                .unwrap();
            baffled
                .step(Vector3::new(0.0, 0.0, g), Vector3::zeros(), dt)
                .unwrap();
            let (bx, by) = bare.slosh_angles_rad();
            let (bdx, bdy) = baffled.slosh_angles_rad();
            assert_eq!(bx.to_bits(), bdx.to_bits());
            assert_eq!(by.to_bits(), bdy.to_bits());
        }
    }

    #[test]
    fn determinism_byte_stable_replay() {
        let baffle = BaffleModel {
            damping_increment_zeta: 0.03,
        };
        let mut a = BaffledPendulum::new(
            cylinder_a05_h2(),
            water(),
            0.7,
            Vector3::new(0.5, 0.0, 1.0),
            0.005,
            baffle,
        )
        .unwrap();
        let mut b = a.clone();
        a.set_initial_slosh((0.03, 0.02), (0.0, -0.01)).unwrap();
        b.set_initial_slosh((0.03, 0.02), (0.0, -0.01)).unwrap();
        let dt = Duration::from_seconds(0.001);
        for step in 0_i32..10_000 {
            let phase = f64::from(step) * 0.001;
            let accel = Vector3::new(
                0.5 * phase.sin(),
                0.3 * phase.cos(),
                9.81 + 0.2 * phase.sin(),
            );
            let drain = 0.5 * (1.0 + phase.sin().abs());
            a.drain(drain).unwrap();
            b.drain(drain).unwrap();
            a.step(accel, Vector3::zeros(), dt).unwrap();
            b.step(accel, Vector3::zeros(), dt).unwrap();
            let (a_x, a_y) = a.slosh_angles_rad();
            let (b_x, b_y) = b.slosh_angles_rad();
            assert_eq!(a_x.to_bits(), b_x.to_bits());
            assert_eq!(a_y.to_bits(), b_y.to_bits());
        }
    }
}
