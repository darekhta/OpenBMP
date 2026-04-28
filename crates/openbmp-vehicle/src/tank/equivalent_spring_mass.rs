//! [`EquivalentSpringMass`] — Phase-3.7.C alternative slosh model.
//!
//! For high-fill-fraction studies the linearised
//! [`crate::tank::equivalent_pendulum::EquivalentPendulum`] can be
//! re-cast as a translational mass-on-a-spring oscillator with the
//! same natural frequency. The slosh mass moves in a horizontal
//! plane (perpendicular to the body axial axis) under a linear
//! restoring force `F = -k · x` where `k = m_slosh · ω_n²`.
//!
//! The two models are physically equivalent in the small-angle
//! limit; this one expresses the state in linear coordinates
//! (`displacement_m`, `velocity_m_s`) rather than angular
//! coordinates, which downstream control / estimator code may find
//! more convenient when interfacing with linear models or when
//! coupling to multi-DOF structural FEM in future phases.
//!
//! Phase-3.7.C uses the same Abramson cylindrical-tank closed-form
//! `ω_n²(g_eff, h, a)` as
//! [`crate::tank::equivalent_pendulum::EquivalentPendulum`], the
//! same semi-implicit Euler integration scheme, and the same
//! locked operand order. The only structural differences are the
//! state representation and the reaction-force formula
//! (`-m · ẍ` directly, no pendulum-length scale factor).

use nalgebra::Vector3;
use openbmp_core::Duration;

use super::{
    point_mass_inertia_about_origin, ForceMomentBody, MassContribution, MovingMassModel,
    PropellantSpec, TankError, TankGeometry,
};

/// First root of `J_1'(x) = 0` (Abramson SP-106 Table 7.1).
/// Mirrors [`crate::tank::equivalent_pendulum::KSI_1`]; both models
/// share the cylindrical-tank Bessel-root form.
const KSI_1: f64 = 1.841;

/// Phase-3.7.C equivalent-spring-mass slosh model.
#[derive(Debug, Clone)]
pub struct EquivalentSpringMass {
    fluid_kg: f64,
    mount_point_body_m: Vector3<f64>,
    tank_radius_m: f64,
    propellant_density_kg_m3: f64,
    damping_ratio_zeta: f64,
    pending_drain_kg_per_s: f64,

    displacement_x_m: f64,
    velocity_x_m_s: f64,
    displacement_y_m: f64,
    velocity_y_m_s: f64,

    last_accel_x_m_s2: f64,
    last_accel_y_m_s2: f64,
}

impl EquivalentSpringMass {
    /// Construct a new equivalent spring-mass slosh model. Same
    /// validation rules as
    /// [`crate::tank::equivalent_pendulum::EquivalentPendulum::new`].
    ///
    /// # Errors
    ///
    /// Returns [`TankError`] for invalid geometry / propellant /
    /// fill / mount / damping.
    pub fn new(
        geometry: TankGeometry,
        propellant: PropellantSpec,
        fill_fraction: f64,
        mount_point_body_m: Vector3<f64>,
        damping_ratio_zeta: f64,
    ) -> Result<Self, TankError> {
        geometry.require_valid()?;
        propellant.require_valid()?;
        if !fill_fraction.is_finite() || !(0.0..=1.0).contains(&fill_fraction) {
            return Err(TankError::InvalidFillFraction { value: fill_fraction });
        }
        if !mount_point_body_m.iter().all(|c| c.is_finite()) {
            return Err(TankError::InvalidMountPoint);
        }
        if !damping_ratio_zeta.is_finite() || damping_ratio_zeta < 0.0 {
            return Err(TankError::InvalidBaffle {
                reason: "damping_ratio_zeta must be finite and non-negative",
            });
        }
        let TankGeometry::Cylinder { radius_m, height_m: _ } = geometry else {
            return Err(TankError::InvalidGeometry {
                reason: "EquivalentSpringMass requires a Cylinder geometry",
            });
        };
        let fluid_kg = geometry.volume_m3() * propellant.density_kg_m3 * fill_fraction;
        Ok(Self {
            fluid_kg,
            mount_point_body_m,
            tank_radius_m: radius_m,
            propellant_density_kg_m3: propellant.density_kg_m3,
            damping_ratio_zeta,
            pending_drain_kg_per_s: 0.0,
            displacement_x_m: 0.0,
            velocity_x_m_s: 0.0,
            displacement_y_m: 0.0,
            velocity_y_m_s: 0.0,
            last_accel_x_m_s2: 0.0,
            last_accel_y_m_s2: 0.0,
        })
    }

    /// Set the initial slosh state.
    ///
    /// `(displacement_m.0, displacement_m.1)` is `(x, y)` in metres
    /// and `(velocity_m_s.0, velocity_m_s.1)` is `(ẋ, ẏ)`.
    ///
    /// # Errors
    ///
    /// Returns [`TankError::NonFiniteStepInput`] for non-finite
    /// inputs.
    pub fn set_initial_slosh(
        &mut self,
        displacement_m: (f64, f64),
        velocity_m_s: (f64, f64),
    ) -> Result<(), TankError> {
        if !(displacement_m.0.is_finite()
            && displacement_m.1.is_finite()
            && velocity_m_s.0.is_finite()
            && velocity_m_s.1.is_finite())
        {
            return Err(TankError::NonFiniteStepInput {
                reason: "initial slosh state must be finite",
            });
        }
        self.displacement_x_m = displacement_m.0;
        self.displacement_y_m = displacement_m.1;
        self.velocity_x_m_s = velocity_m_s.0;
        self.velocity_y_m_s = velocity_m_s.1;
        Ok(())
    }

    /// Current displacement `(x, y)` in metres.
    #[must_use]
    pub fn displacement_m(&self) -> (f64, f64) {
        (self.displacement_x_m, self.displacement_y_m)
    }

    /// Current velocity `(ẋ, ẏ)` in metres per second.
    #[must_use]
    pub fn velocity_m_s(&self) -> (f64, f64) {
        (self.velocity_x_m_s, self.velocity_y_m_s)
    }

    /// Current liquid height. Same formula as
    /// [`EquivalentPendulum::fluid_height_m`].
    #[must_use]
    pub fn fluid_height_m(&self) -> f64 {
        let cross_section = std::f64::consts::PI * self.tank_radius_m * self.tank_radius_m;
        let volume = self.fluid_kg / self.propellant_density_kg_m3;
        volume / cross_section
    }

    /// Closed-form `ω_n²` at the current axial accel and fluid
    /// level. Same formula as
    /// [`crate::tank::equivalent_pendulum::EquivalentPendulum::omega_n_squared_rad2_s2`].
    #[must_use]
    pub fn omega_n_squared_rad2_s2(&self, axial_accel_m_s2: f64) -> f64 {
        let h = self.fluid_height_m();
        if axial_accel_m_s2 <= 0.0 || h <= 0.0 {
            return 0.0;
        }
        let arg = KSI_1 * h / self.tank_radius_m;
        (axial_accel_m_s2 / self.tank_radius_m) * KSI_1 * arg.tanh()
    }
}

impl MovingMassModel for EquivalentSpringMass {
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

        let axial_accel = accel_body_m_s2.z;
        let lateral_accel_x = accel_body_m_s2.x;
        let lateral_accel_y = accel_body_m_s2.y;
        let omega_n_squared = self.omega_n_squared_rad2_s2(axial_accel);
        let omega_n = omega_n_squared.sqrt();

        // x axis: locked operand order (damping, restoring, forcing,
        // ddot, velocity, position).
        let damping_x = 2.0 * self.damping_ratio_zeta * omega_n * self.velocity_x_m_s;
        let restoring_x = omega_n_squared * self.displacement_x_m;
        let forcing_x = lateral_accel_x;
        let accel_x = forcing_x - damping_x - restoring_x;
        self.last_accel_x_m_s2 = accel_x;
        self.velocity_x_m_s += dt_s * accel_x;
        self.displacement_x_m += dt_s * self.velocity_x_m_s;

        // y axis (declared after x).
        let damping_y = 2.0 * self.damping_ratio_zeta * omega_n * self.velocity_y_m_s;
        let restoring_y = omega_n_squared * self.displacement_y_m;
        let forcing_y = lateral_accel_y;
        let accel_y = forcing_y - damping_y - restoring_y;
        self.last_accel_y_m_s2 = accel_y;
        self.velocity_y_m_s += dt_s * accel_y;
        self.displacement_y_m += dt_s * self.velocity_y_m_s;

        Ok(())
    }

    fn mass_contribution(&self) -> MassContribution {
        let m = self.fluid_kg;
        let cg_offset = Vector3::new(self.displacement_x_m, self.displacement_y_m, 0.0);
        let r_total = self.mount_point_body_m + cg_offset;
        MassContribution {
            mass_kg: m,
            cg_offset_body_m: cg_offset,
            inertia_delta_body_kg_m2: point_mass_inertia_about_origin(m, r_total),
        }
    }

    fn reaction_body(&self) -> ForceMomentBody {
        let m = self.fluid_kg;
        let force = Vector3::new(-m * self.last_accel_x_m_s2, -m * self.last_accel_y_m_s2, 0.0);
        let moment = self.mount_point_body_m.cross(&force);
        ForceMomentBody {
            force_body_n: force,
            moment_body_n_m: moment,
        }
    }

    fn fluid_remaining_kg(&self) -> f64 {
        self.fluid_kg
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
    use crate::tank::equivalent_pendulum::EquivalentPendulum;

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
    fn rejects_non_cylinder_geometry() {
        let result = EquivalentSpringMass::new(
            TankGeometry::Sphere { radius_m: 1.0 },
            water(),
            1.0,
            Vector3::zeros(),
            0.0,
        );
        assert!(matches!(result, Err(TankError::InvalidGeometry { .. })));
    }

    #[test]
    fn omega_n_matches_pendulum_at_same_state() {
        let spring = EquivalentSpringMass::new(
            cylinder_a05_h2(),
            water(),
            0.7,
            Vector3::zeros(),
            0.0,
        )
        .unwrap();
        let pendulum = EquivalentPendulum::new(
            cylinder_a05_h2(),
            water(),
            0.7,
            Vector3::zeros(),
            0.0,
        )
        .unwrap();
        let axial = 9.81;
        assert_eq!(
            spring.omega_n_squared_rad2_s2(axial).to_bits(),
            pendulum.omega_n_squared_rad2_s2(axial).to_bits(),
        );
    }

    #[test]
    fn free_response_oscillation_frequency_matches_closed_form() {
        let mut spring = EquivalentSpringMass::new(
            cylinder_a05_h2(),
            water(),
            1.0,
            Vector3::zeros(),
            0.0,
        )
        .unwrap();
        spring.set_initial_slosh((0.05, 0.0), (0.0, 0.0)).unwrap();
        let g = 9.81;
        let omega_sq = spring.omega_n_squared_rad2_s2(g);
        let omega_n = omega_sq.sqrt();
        let expected_period = 2.0 * std::f64::consts::PI / omega_n;
        let dt = Duration::from_seconds(0.001);
        let dt_s = dt.as_seconds();

        let mut prev_x = 0.05;
        let mut crossings: Vec<f64> = Vec::new();
        let mut t = 0.0;
        for _ in 0..200_000 {
            spring
                .step(Vector3::new(0.0, 0.0, g), Vector3::zeros(), dt)
                .unwrap();
            t += dt_s;
            let (x, _) = spring.displacement_m();
            if (prev_x > 0.0) != (x > 0.0) && crossings.len() < 200 {
                crossings.push(t);
            }
            prev_x = x;
            if crossings.len() >= 200 {
                break;
            }
        }
        let n = crossings.len();
        let measured_half_period = (crossings[n - 1] - crossings[0]) / ((n - 1) as f64);
        let measured_period = 2.0 * measured_half_period;
        let relative_error = (measured_period - expected_period).abs() / expected_period;
        assert!(
            relative_error < 0.01,
            "period error {relative_error} > 1%; expected {expected_period}, measured {measured_period}"
        );
    }

    #[test]
    fn energy_is_conserved_within_one_percent_over_100_cycles() {
        let mut spring = EquivalentSpringMass::new(
            cylinder_a05_h2(),
            water(),
            1.0,
            Vector3::zeros(),
            0.0,
        )
        .unwrap();
        spring.set_initial_slosh((0.05, 0.0), (0.0, 0.0)).unwrap();
        let g = 9.81;
        let omega_sq = spring.omega_n_squared_rad2_s2(g);
        let initial_energy = 0.5 * omega_sq * 0.05_f64.powi(2);
        let dt = Duration::from_seconds(0.001);
        let cycles_target = 100;
        let total_steps = (((cycles_target as f64) * 2.0 * std::f64::consts::PI / omega_sq.sqrt())
            / dt.as_seconds())
        .ceil() as usize;

        let mut min_e = initial_energy;
        let mut max_e = initial_energy;
        for _ in 0..total_steps {
            spring
                .step(Vector3::new(0.0, 0.0, g), Vector3::zeros(), dt)
                .unwrap();
            let (x, _) = spring.displacement_m();
            let (v, _) = spring.velocity_m_s();
            let e = 0.5 * v.powi(2) + 0.5 * omega_sq * x.powi(2);
            if e < min_e {
                min_e = e;
            }
            if e > max_e {
                max_e = e;
            }
        }
        let drift = (max_e - min_e).abs() / initial_energy;
        assert!(drift < 0.01, "energy drift {drift} > 1%");
    }

    #[test]
    fn lateral_acceleration_excites_displacement() {
        let mut spring = EquivalentSpringMass::new(
            cylinder_a05_h2(),
            water(),
            1.0,
            Vector3::zeros(),
            0.0,
        )
        .unwrap();
        let g = 9.81;
        let dt = Duration::from_seconds(0.001);
        for _ in 0..100 {
            spring
                .step(Vector3::new(2.0, 0.0, g), Vector3::zeros(), dt)
                .unwrap();
        }
        let (x, _) = spring.displacement_m();
        assert!(x.abs() > 1e-3, "lateral kick must excite displacement; got {x}");
    }

    #[test]
    fn determinism_byte_stable_replay() {
        let mut a = EquivalentSpringMass::new(
            cylinder_a05_h2(),
            water(),
            0.7,
            Vector3::new(0.5, 0.0, 1.0),
            0.005,
        )
        .unwrap();
        let mut b = a.clone();
        a.set_initial_slosh((0.01, 0.005), (0.0, -0.001)).unwrap();
        b.set_initial_slosh((0.01, 0.005), (0.0, -0.001)).unwrap();
        let dt = Duration::from_seconds(0.001);
        for step in 0_i32..10_000 {
            let phase = f64::from(step) * 0.001;
            let accel = Vector3::new(0.5 * phase.sin(), 0.3 * phase.cos(), 9.81 + 0.2 * phase.sin());
            let drain = 0.5 * (1.0 + phase.sin().abs());
            a.drain(drain).unwrap();
            b.drain(drain).unwrap();
            a.step(accel, Vector3::zeros(), dt).unwrap();
            b.step(accel, Vector3::zeros(), dt).unwrap();
            let (a_x, a_y) = a.displacement_m();
            let (b_x, b_y) = b.displacement_m();
            assert_eq!(a_x.to_bits(), b_x.to_bits());
            assert_eq!(a_y.to_bits(), b_y.to_bits());
            assert_eq!(a.fluid_remaining_kg().to_bits(), b.fluid_remaining_kg().to_bits());
        }
    }
}
