//! [`EquivalentPendulum`] — Phase-3.7.B Abramson SP-106 equivalent
//! pendulum slosh dynamics.
//!
//! # Physics
//!
//! For a cylindrical tank with a vertical axis aligned with the
//! parent body's `+z` direction, the lowest sloshing mode is the
//! antisymmetric mode about a vertical plane through the tank
//! centre (Abramson SP-106 §7.4). The closed-form natural frequency
//! is
//!
//! ```text
//! ω_n² = (g_eff / a) · ξ_1 · tanh(ξ_1 · h / a)
//! ```
//!
//! and the equivalent pendulum length is
//!
//! ```text
//! L_pend = a / (ξ_1 · tanh(ξ_1 · h / a))
//! ```
//!
//! where `a` is the tank radius, `h` is the current liquid depth,
//! `g_eff` is the axial acceleration at the mount, and
//! `ξ_1 = 1.841` is the first root of `J_1'(x) = 0` (Abramson Table
//! 7.1, cylindrical-tank entry).
//!
//! Phase 3.7.B parameterises the pendulum by **two** independent
//! tilt angles `(θ_x, θ_y)` representing swing in the body x-z and
//! y-z planes respectively, each obeying the small-angle linearised
//! Abramson Eq 7-25:
//!
//! ```text
//! θ̈ + 2ζω_n θ̇ + ω_n² θ = a_lateral / L_pend
//! ```
//!
//! Lateral acceleration components are read directly from the
//! body-frame acceleration vector passed into [`MovingMassModel::step`].
//! The pendulum is decoupled across `(x, y)` — Phase-3.7 does not
//! model cross-axis slosh coupling (e.g. swirling).
//!
//! # Integration scheme
//!
//! Phase-3.7 uses **semi-implicit (symplectic) Euler** rather than
//! the pure-explicit form sketched in `phase-3-plan.md § 3.7`:
//! explicit Euler is unstable for an undamped harmonic oscillator
//! (energy grows by `(dt · ω)²` per step), and the property test
//! for ≤1% energy conservation over 100 oscillations cannot be met
//! at any practical `dt`. Semi-implicit Euler is bit-stable for
//! fixed `dt`, conserves a modified energy exactly, and preserves
//! the determinism contract.
//!
//! Locked operand order per axis: `damping_term`, `restoring_term`,
//! `forcing_term`, `theta_ddot = forcing - damping - restoring`;
//! velocity update `θ̇_new = θ̇_old + dt · θ̈`; position update
//! `θ_new = θ_old + dt · θ̇_new`. Axes are evaluated `x` then `y`
//! (declared order). No FMA.
//!
//! # Mass contribution
//!
//! Phase-3.7.B treats **all** of the fluid as the slosh mass (i.e.
//! `m_slosh = m_total`). Real-physics slosh has a static fraction
//! that stays at the tank base plus a slosh fraction that swings;
//! the split depends on tank geometry and fill fraction (Abramson
//! Table 7.1). Phase-3.7 ships the conservative simplification —
//! all fluid swings — and downstream extensions can split the mass
//! by replacing the moving-mass model with their own implementation.
//!
//! `cg_offset_body_m` is the swing displacement of the slosh mass
//! relative to the tank mount point (small-angle approximation):
//!
//! ```text
//! cg_offset = (L · θ_x, L · θ_y, -L)
//! ```
//!
//! `inertia_delta_body_kg_m2` is the parallel-axis term for the
//! point mass at body-frame offset `mount_point + cg_offset`.
//!
//! # Reaction force / moment
//!
//! Reaction force on the parent body is `-m · a_slosh_lateral`,
//! where `a_slosh_lateral ≈ L · θ̈_lateral` for small angles. The
//! axial reaction force is zero (Phase-3.7 simplification — full
//! pendulum centripetal feedback would also produce a small axial
//! component). Reaction moment about body origin is
//! `mount_point × reaction_force` (small-angle: ignore the
//! pendulum-length contribution to the moment arm).
//!
//! # Determinism
//!
//! - Pure `f64` arithmetic; no FMA, no system RNG, no I/O.
//! - Single sub-step (forward symplectic Euler) per main RK4 step.
//!   Higher sub-step counts are deferred to Phase-3.7.D scenario
//!   opt-in (and break bit-stability when the count changes —
//!   documented in `provenance.md`).
//! - Recomputes `ω_n²` and `L_pend` from the post-drain fluid level
//!   each step; values within one step are self-consistent.

use nalgebra::Vector3;
use openbmp_core::Duration;

use super::{
    point_mass_inertia_about_origin, ForceMomentBody, MassContribution, MovingMassModel,
    PropellantSpec, TankError, TankGeometry,
};

/// First root of `J_1'(x) = 0`, the Bessel-function eigenvalue
/// associated with the antisymmetric fundamental sloshing mode in
/// a cylindrical tank (Abramson SP-106 Table 7.1).
const KSI_1: f64 = 1.841;

/// Phase-3.7.B equivalent-pendulum slosh model.
#[derive(Debug, Clone)]
pub struct EquivalentPendulum {
    fluid_kg: f64,
    mount_point_body_m: Vector3<f64>,
    tank_radius_m: f64,
    propellant_density_kg_m3: f64,
    damping_ratio_zeta: f64,
    pending_drain_kg_per_s: f64,

    theta_x_rad: f64,
    theta_dot_x_rad_s: f64,
    theta_y_rad: f64,
    theta_dot_y_rad_s: f64,

    last_theta_ddot_x_rad_s2: f64,
    last_theta_ddot_y_rad_s2: f64,
    last_pendulum_length_m: f64,
}

impl EquivalentPendulum {
    /// Construct a new equivalent pendulum.
    ///
    /// `tank_radius_m` is the cylindrical tank's internal radius;
    /// it must equal the geometry's `radius_m`. `damping_ratio_zeta`
    /// is the bare-tank damping (typical academic value 0.005). The
    /// pendulum starts at `θ = 0`, `θ̇ = 0`.
    ///
    /// # Errors
    ///
    /// Returns [`TankError`] for invalid geometry / propellant /
    /// fill / mount, or when `damping_ratio_zeta` is non-finite or
    /// negative, or when `tank_radius_m` does not match the
    /// geometry's radius.
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
                reason: "EquivalentPendulum requires a Cylinder geometry",
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
            theta_x_rad: 0.0,
            theta_dot_x_rad_s: 0.0,
            theta_y_rad: 0.0,
            theta_dot_y_rad_s: 0.0,
            last_theta_ddot_x_rad_s2: 0.0,
            last_theta_ddot_y_rad_s2: 0.0,
            last_pendulum_length_m: radius_m,
        })
    }

    /// Set the initial slosh state. Use for perturbation studies or
    /// the Phase-3.7.B free-response unit / property tests.
    ///
    /// `(angles_rad.0, angles_rad.1)` is `(θ_x, θ_y)` and
    /// `(rates_rad_s.0, rates_rad_s.1)` is `(θ̇_x, θ̇_y)`.
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
        if !(angles_rad.0.is_finite()
            && angles_rad.1.is_finite()
            && rates_rad_s.0.is_finite()
            && rates_rad_s.1.is_finite())
        {
            return Err(TankError::NonFiniteStepInput {
                reason: "initial slosh state must be finite",
            });
        }
        self.theta_x_rad = angles_rad.0;
        self.theta_y_rad = angles_rad.1;
        self.theta_dot_x_rad_s = rates_rad_s.0;
        self.theta_dot_y_rad_s = rates_rad_s.1;
        Ok(())
    }

    /// Current slosh-tilt angles `(θ_x, θ_y)`, radians.
    #[must_use]
    pub fn slosh_angles_rad(&self) -> (f64, f64) {
        (self.theta_x_rad, self.theta_y_rad)
    }

    /// Current slosh-tilt rates `(θ̇_x, θ̇_y)`, radians per second.
    #[must_use]
    pub fn slosh_rates_rad_s(&self) -> (f64, f64) {
        (self.theta_dot_x_rad_s, self.theta_dot_y_rad_s)
    }

    /// Current liquid height in the tank, metres.
    #[must_use]
    pub fn fluid_height_m(&self) -> f64 {
        let cross_section = std::f64::consts::PI * self.tank_radius_m * self.tank_radius_m;
        let volume = self.fluid_kg / self.propellant_density_kg_m3;
        volume / cross_section
    }

    /// Closed-form `ω_n²` at the current axial acceleration and
    /// fluid level. Returns 0 when axial accel is non-positive or
    /// fluid is depleted (no restoring force in either case).
    #[must_use]
    pub fn omega_n_squared_rad2_s2(&self, axial_accel_m_s2: f64) -> f64 {
        if axial_accel_m_s2 <= 0.0 {
            return 0.0;
        }
        let h = self.fluid_height_m();
        if h <= 0.0 {
            return 0.0;
        }
        let arg = KSI_1 * h / self.tank_radius_m;
        (axial_accel_m_s2 / self.tank_radius_m) * KSI_1 * arg.tanh()
    }

    /// Closed-form pendulum length at the current fluid level.
    /// Falls back to the tank radius when fluid is depleted (matches
    /// the asymptotic formula limit and avoids division by zero).
    #[must_use]
    pub fn pendulum_length_m(&self) -> f64 {
        let h = self.fluid_height_m();
        if h <= 0.0 {
            return self.tank_radius_m;
        }
        let arg = KSI_1 * h / self.tank_radius_m;
        let tanh_val = arg.tanh();
        if tanh_val <= 0.0 {
            return self.tank_radius_m;
        }
        self.tank_radius_m / (KSI_1 * tanh_val)
    }
}

impl MovingMassModel for EquivalentPendulum {
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

        // 1. Drain.
        let drained_kg = self.pending_drain_kg_per_s * dt_s;
        let next = self.fluid_kg - drained_kg;
        self.fluid_kg = if next > 0.0 { next } else { 0.0 };

        // 2. Recompute ω_n² and L_pend at the post-drain fluid level.
        let axial_accel = accel_body_m_s2.z;
        let lateral_accel_x = accel_body_m_s2.x;
        let lateral_accel_y = accel_body_m_s2.y;
        let omega_n_squared = self.omega_n_squared_rad2_s2(axial_accel);
        let omega_n = omega_n_squared.sqrt();
        let l_pend = self.pendulum_length_m();
        self.last_pendulum_length_m = l_pend;

        // 3. θ̈ for x axis (locked operand order: damping, restoring,
        //    forcing, then theta_ddot = forcing - damping - restoring).
        let damping_x = 2.0 * self.damping_ratio_zeta * omega_n * self.theta_dot_x_rad_s;
        let restoring_x = omega_n_squared * self.theta_x_rad.sin();
        let forcing_x = lateral_accel_x / l_pend;
        let theta_ddot_x = forcing_x - damping_x - restoring_x;
        self.last_theta_ddot_x_rad_s2 = theta_ddot_x;

        // 4. Symplectic Euler x: velocity update first, then
        //    position update using the freshly-updated velocity.
        self.theta_dot_x_rad_s += dt_s * theta_ddot_x;
        self.theta_x_rad += dt_s * self.theta_dot_x_rad_s;

        // 5. Same for y axis (declared after x).
        let damping_y = 2.0 * self.damping_ratio_zeta * omega_n * self.theta_dot_y_rad_s;
        let restoring_y = omega_n_squared * self.theta_y_rad.sin();
        let forcing_y = lateral_accel_y / l_pend;
        let theta_ddot_y = forcing_y - damping_y - restoring_y;
        self.last_theta_ddot_y_rad_s2 = theta_ddot_y;

        self.theta_dot_y_rad_s += dt_s * theta_ddot_y;
        self.theta_y_rad += dt_s * self.theta_dot_y_rad_s;

        Ok(())
    }

    fn mass_contribution(&self) -> MassContribution {
        let m = self.fluid_kg;
        let l = self.last_pendulum_length_m;
        let cg_offset = Vector3::new(
            l * self.theta_x_rad.sin(),
            l * self.theta_y_rad.sin(),
            -l,
        );
        let r_total = self.mount_point_body_m + cg_offset;
        MassContribution {
            mass_kg: m,
            cg_offset_body_m: cg_offset,
            inertia_delta_body_kg_m2: point_mass_inertia_about_origin(m, r_total),
        }
    }

    fn reaction_body(&self) -> ForceMomentBody {
        let m = self.fluid_kg;
        let l = self.last_pendulum_length_m;
        let force = Vector3::new(
            -m * l * self.last_theta_ddot_x_rad_s2,
            -m * l * self.last_theta_ddot_y_rad_s2,
            0.0,
        );
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
    use proptest::prelude::*;

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
        let result = EquivalentPendulum::new(
            TankGeometry::Sphere { radius_m: 1.0 },
            water(),
            1.0,
            Vector3::zeros(),
            0.0,
        );
        assert!(matches!(result, Err(TankError::InvalidGeometry { .. })));
    }

    #[test]
    fn rejects_negative_damping() {
        let result = EquivalentPendulum::new(
            cylinder_a05_h2(),
            water(),
            1.0,
            Vector3::zeros(),
            -0.01,
        );
        assert!(matches!(result, Err(TankError::InvalidBaffle { .. })));
    }

    #[test]
    fn omega_n_squared_zero_at_zero_axial_accel() {
        let pend = EquivalentPendulum::new(
            cylinder_a05_h2(),
            water(),
            1.0,
            Vector3::zeros(),
            0.0,
        )
        .unwrap();
        assert!(pend.omega_n_squared_rad2_s2(0.0).abs() < 1e-12);
        assert!(pend.omega_n_squared_rad2_s2(-9.81).abs() < 1e-12);
    }

    #[test]
    fn omega_n_squared_matches_closed_form() {
        // a = 0.5 m, full tank h = 2 m, g = 9.81: arg = 1.841·2/0.5 = 7.364, tanh ≈ 1.0
        // ω_n² = (9.81/0.5) · 1.841 · 1.0 ≈ 36.12
        let pend = EquivalentPendulum::new(
            cylinder_a05_h2(),
            water(),
            1.0,
            Vector3::zeros(),
            0.0,
        )
        .unwrap();
        let omega_sq = pend.omega_n_squared_rad2_s2(9.81);
        let arg = KSI_1 * 2.0 / 0.5;
        let expected = (9.81 / 0.5) * KSI_1 * arg.tanh();
        assert!((omega_sq - expected).abs() < 1e-9);
    }

    #[test]
    fn pendulum_length_falls_back_when_fluid_empty() {
        let mut pend = EquivalentPendulum::new(
            cylinder_a05_h2(),
            water(),
            0.001,
            Vector3::zeros(),
            0.0,
        )
        .unwrap();
        // Drain it dry.
        pend.drain(1e6).unwrap();
        pend.step(
            Vector3::new(0.0, 0.0, 9.81),
            Vector3::zeros(),
            Duration::from_seconds(1.0),
        )
        .unwrap();
        assert!((pend.fluid_remaining_kg() - 0.0).abs() < 1e-12);
        assert!((pend.pendulum_length_m() - 0.5).abs() < 1e-12);
    }

    #[test]
    fn free_response_oscillation_frequency_matches_closed_form() {
        // Set initial angle, no damping, no lateral accel; integrate
        // and measure period via zero crossings.
        let mut pend = EquivalentPendulum::new(
            cylinder_a05_h2(),
            water(),
            1.0,
            Vector3::zeros(),
            0.0,
        )
        .unwrap();
        pend.set_initial_slosh((0.05, 0.0), (0.0, 0.0)).unwrap();
        let g = 9.81;
        let omega_sq = pend.omega_n_squared_rad2_s2(g);
        let omega_n = omega_sq.sqrt();
        let expected_period = 2.0 * std::f64::consts::PI / omega_n;
        let dt = Duration::from_seconds(0.001);
        let dt_s = dt.as_seconds();

        let mut prev_theta = 0.05;
        let mut crossings: Vec<f64> = Vec::new();
        let mut t = 0.0;
        for _ in 0..200_000 {
            pend.step(Vector3::new(0.0, 0.0, g), Vector3::zeros(), dt)
                .unwrap();
            t += dt_s;
            let (theta, _) = pend.slosh_angles_rad();
            if (prev_theta > 0.0) != (theta > 0.0) && crossings.len() < 200 {
                crossings.push(t);
            }
            prev_theta = theta;
            if crossings.len() >= 200 {
                break;
            }
        }
        // 200 crossings = 100 full oscillations. Period = 2 *
        // (crossings[-1] - crossings[0]) / (crossings.len() - 1).
        // (Each pair of crossings spans one half period.)
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
        // ½θ̇² + ½ω_n² θ² across 100 cycles.
        let mut pend = EquivalentPendulum::new(
            cylinder_a05_h2(),
            water(),
            1.0,
            Vector3::zeros(),
            0.0,
        )
        .unwrap();
        pend.set_initial_slosh((0.05, 0.0), (0.0, 0.0)).unwrap();
        let g = 9.81;
        let omega_sq = pend.omega_n_squared_rad2_s2(g);
        let initial_energy = 0.5 * omega_sq * 0.05_f64.powi(2);
        let dt = Duration::from_seconds(0.001);
        let cycles_target = 100;
        let total_steps =
            ((cycles_target as f64) * (2.0 * std::f64::consts::PI / omega_sq.sqrt()) / dt.as_seconds())
                .ceil() as usize;

        let mut min_e = initial_energy;
        let mut max_e = initial_energy;
        for _ in 0..total_steps {
            pend.step(Vector3::new(0.0, 0.0, g), Vector3::zeros(), dt)
                .unwrap();
            let (theta, _) = pend.slosh_angles_rad();
            let (theta_dot, _) = pend.slosh_rates_rad_s();
            let e = 0.5 * theta_dot.powi(2) + 0.5 * omega_sq * theta.powi(2);
            if e < min_e {
                min_e = e;
            }
            if e > max_e {
                max_e = e;
            }
        }
        let relative_drift = (max_e - min_e).abs() / initial_energy;
        assert!(
            relative_drift < 0.01,
            "energy drift {relative_drift} > 1%; min {min_e}, max {max_e}, initial {initial_energy}"
        );
    }

    proptest! {
        #![proptest_config(ProptestConfig {
            cases: 16,
            .. ProptestConfig::default()
        })]

        #[test]
        fn property_energy_bounded_for_any_small_initial_angle(
            theta0 in -0.1_f64..0.1_f64,
            theta_dot0 in -0.1_f64..0.1_f64,
        ) {
            let mut pend = EquivalentPendulum::new(
                cylinder_a05_h2(),
                water(),
                1.0,
                Vector3::zeros(),
                0.0,
            )
            .unwrap();
            pend.set_initial_slosh((theta0, 0.0), (theta_dot0, 0.0)).unwrap();
            let g = 9.81;
            let omega_sq = pend.omega_n_squared_rad2_s2(g);
            let initial_energy =
                0.5 * theta_dot0.powi(2) + 0.5 * omega_sq * theta0.powi(2);
            let dt = Duration::from_seconds(0.001);
            let cycles_target = 100;
            let total_steps = ((cycles_target as f64) * (2.0 * std::f64::consts::PI / omega_sq.sqrt())
                / dt.as_seconds()).ceil() as usize;

            let mut min_e = initial_energy;
            let mut max_e = initial_energy;
            for _ in 0..total_steps {
                pend.step(Vector3::new(0.0, 0.0, g), Vector3::zeros(), dt).unwrap();
                let (theta, _) = pend.slosh_angles_rad();
                let (theta_dot, _) = pend.slosh_rates_rad_s();
                let e = 0.5 * theta_dot.powi(2) + 0.5 * omega_sq * theta.powi(2);
                if e < min_e { min_e = e; }
                if e > max_e { max_e = e; }
            }
            let bound = (max_e - min_e).abs();
            let tolerance = 0.01_f64.max(initial_energy * 0.01);
            prop_assert!(
                bound < tolerance,
                "energy drift {bound} > tolerance {tolerance}; theta0={theta0}, theta_dot0={theta_dot0}, initial={initial_energy}"
            );
        }
    }

    #[test]
    fn damping_decays_amplitude_when_zeta_positive() {
        let mut pend = EquivalentPendulum::new(
            cylinder_a05_h2(),
            water(),
            1.0,
            Vector3::zeros(),
            0.05,
        )
        .unwrap();
        pend.set_initial_slosh((0.05, 0.0), (0.0, 0.0)).unwrap();
        let g = 9.81;
        let dt = Duration::from_seconds(0.001);
        let mut peak_amplitude_initial = 0.0_f64;
        let mut peak_amplitude_late = 0.0_f64;
        let mut prev_rate = 0.0;
        let mut zero_rate_crossings = 0;
        for _ in 0..50_000 {
            pend.step(Vector3::new(0.0, 0.0, g), Vector3::zeros(), dt)
                .unwrap();
            let (theta, _) = pend.slosh_angles_rad();
            let (theta_dot, _) = pend.slosh_rates_rad_s();
            if (prev_rate > 0.0) != (theta_dot > 0.0) {
                zero_rate_crossings += 1;
                if zero_rate_crossings <= 2 {
                    peak_amplitude_initial = peak_amplitude_initial.max(theta.abs());
                } else if zero_rate_crossings > 50 {
                    peak_amplitude_late = peak_amplitude_late.max(theta.abs());
                }
            }
            prev_rate = theta_dot;
        }
        assert!(
            peak_amplitude_late < peak_amplitude_initial * 0.5,
            "expected amplitude decay; initial peak {peak_amplitude_initial}, late peak {peak_amplitude_late}"
        );
    }

    #[test]
    fn lateral_acceleration_excites_pendulum() {
        let mut pend = EquivalentPendulum::new(
            cylinder_a05_h2(),
            water(),
            1.0,
            Vector3::zeros(),
            0.0,
        )
        .unwrap();
        let g = 9.81;
        let dt = Duration::from_seconds(0.001);
        // Apply lateral accel for a brief window then release.
        for _ in 0..100 {
            pend.step(
                Vector3::new(2.0, 0.0, g),
                Vector3::zeros(),
                dt,
            )
            .unwrap();
        }
        let (theta_after_kick, _) = pend.slosh_angles_rad();
        assert!(theta_after_kick.abs() > 1e-3, "lateral kick must excite the pendulum, got {theta_after_kick}");
    }

    #[test]
    fn determinism_byte_stable_replay_over_many_steps() {
        let mut a = EquivalentPendulum::new(
            cylinder_a05_h2(),
            water(),
            0.7,
            Vector3::new(0.5, 0.0, 1.0),
            0.005,
        )
        .unwrap();
        let mut b = a.clone();
        a.set_initial_slosh((0.03, 0.02), (0.0, -0.01)).unwrap();
        b.set_initial_slosh((0.03, 0.02), (0.0, -0.01)).unwrap();
        let dt = Duration::from_seconds(0.001);
        for step in 0_i32..10_000 {
            let phase = f64::from(step) * 0.001;
            let accel = Vector3::new(0.5 * phase.sin(), 0.3 * phase.cos(), 9.81 + 0.2 * phase.sin());
            let omega = Vector3::zeros();
            let drain = 0.5 * (1.0 + phase.sin().abs());
            a.drain(drain).unwrap();
            b.drain(drain).unwrap();
            a.step(accel, omega, dt).unwrap();
            b.step(accel, omega, dt).unwrap();
            let (a_x, a_y) = a.slosh_angles_rad();
            let (b_x, b_y) = b.slosh_angles_rad();
            assert_eq!(a_x.to_bits(), b_x.to_bits());
            assert_eq!(a_y.to_bits(), b_y.to_bits());
            assert_eq!(a.fluid_remaining_kg().to_bits(), b.fluid_remaining_kg().to_bits());
        }
    }

    #[test]
    fn reaction_force_is_zero_when_pendulum_at_rest() {
        let mut pend = EquivalentPendulum::new(
            cylinder_a05_h2(),
            water(),
            1.0,
            Vector3::zeros(),
            0.0,
        )
        .unwrap();
        pend.step(
            Vector3::new(0.0, 0.0, 9.81),
            Vector3::zeros(),
            Duration::from_seconds(0.001),
        )
        .unwrap();
        let r = pend.reaction_body();
        assert!(r.force_body_n.iter().all(|c| c.abs() < 1e-9), "{r:?}");
    }

    #[test]
    fn reaction_force_appears_when_pendulum_excited() {
        let mut pend = EquivalentPendulum::new(
            cylinder_a05_h2(),
            water(),
            1.0,
            Vector3::zeros(),
            0.0,
        )
        .unwrap();
        pend.set_initial_slosh((0.05, 0.0), (0.0, 0.0)).unwrap();
        // Step once with axial accel only — this lets the spring
        // restoring term build a non-trivial θ̈_x that backreacts.
        pend.step(
            Vector3::new(0.0, 0.0, 9.81),
            Vector3::zeros(),
            Duration::from_seconds(0.001),
        )
        .unwrap();
        let r = pend.reaction_body();
        assert!(r.force_body_n.x.abs() > 0.0, "expected non-zero reaction force x; got {r:?}");
    }
}
