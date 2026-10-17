//! [`AbramsonMultiMass`] -- mass-correct cylindrical slosh analog.
//!
//! This model implements the Dodge/Abramson fixed-mass plus N slosh-mass
//! mechanical analog for an upright cylindrical tank. Unlike
//! [`crate::tank::equivalent_pendulum::EquivalentPendulum`], it does not put
//! the entire fluid mass on the moving oscillator. Each selected lateral mode
//! receives its own translational spring-mass state, while the residual
//! non-sloshing mass is treated as rigidly attached and contributes only mass
//! properties.
//!
//! The state variables are linear displacements `(x, y)` rather than pendulum
//! angles. The spring-mass form is equivalent to the small-angle pendulum form
//! with `omega_n^2 = g_eff / L_n`, and lets each mode reuse the same
//! semi-implicit Euler arithmetic as [`crate::tank::equivalent_spring_mass`].

use nalgebra::{Matrix3, Vector3};
use openbmp_core::Duration;

use super::{
    ForceMomentBody, MassContribution, MovingMassModel, PropellantSpec, TankError, TankGeometry,
    point_mass_inertia_about_origin,
};

/// First eight roots of `J_1'(xi) = 0` used by the cylindrical lateral
/// antisymmetric slosh modes.
pub const CYLINDER_J1_DERIVATIVE_ROOTS: [f64; 8] = [
    1.841_183_781_340_659_3,
    5.331_442_773_525_032_5,
    8.536_316_366_346_286,
    11.706_004_902_592_063,
    14.863_588_633_909_034,
    18.015_527_862_681_804,
    21.164_369_859_188,
    24.311_326_857_210_776,
];

/// Maximum number of cylindrical modes the built-in Bessel-root table supports.
pub const MAX_ABRAMSON_CYLINDER_MODES: usize = CYLINDER_J1_DERIVATIVE_ROOTS.len();

/// Closed-form parameters for one cylindrical Abramson/Dodge slosh mode.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AbramsonModeParams {
    /// One-based mode number.
    pub mode: usize,
    /// Root of `J_1'(xi) = 0`.
    pub xi: f64,
    /// Slosh-mass fraction `m_n / m_liq` for this mode.
    pub mass_fraction: f64,
    /// Equivalent pendulum length `L_n`, metres.
    pub pendulum_length_m: f64,
    /// Pendulum hinge elevation measured up from the liquid bottom, metres.
    pub hinge_height_from_bottom_m: f64,
    /// Hinge elevation relative to the instantaneous liquid center of mass.
    pub hinge_height_from_fluid_cg_m: f64,
    /// Equilibrium slosh-mass elevation relative to the instantaneous liquid
    /// center of mass. This is `H_n - L_n` in the pendulum analog.
    pub slosh_mass_height_from_fluid_cg_m: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct AbramsonModeState {
    displacement_x_m: f64,
    velocity_x_m_s: f64,
    displacement_y_m: f64,
    velocity_y_m_s: f64,
    last_accel_x_m_s2: f64,
    last_accel_y_m_s2: f64,
}

/// Mass-correct fixed-mass plus N slosh-mass model for an upright cylinder.
#[derive(Debug, Clone)]
pub struct AbramsonMultiMass {
    fluid_kg: f64,
    mount_point_body_m: Vector3<f64>,
    tank_radius_m: f64,
    propellant_density_kg_m3: f64,
    damping_ratio_zeta: f64,
    pending_drain_kg_per_s: f64,
    modes: Vec<AbramsonModeState>,
}

impl AbramsonMultiMass {
    /// Construct a new mass-correct Abramson/Dodge cylindrical slosh model.
    ///
    /// # Errors
    ///
    /// Returns [`TankError`] for invalid geometry / propellant / fill / mount /
    /// damping / mode count.
    pub fn new(
        geometry: TankGeometry,
        propellant: PropellantSpec,
        fill_fraction: f64,
        mount_point_body_m: Vector3<f64>,
        damping_ratio_zeta: f64,
        mode_count: usize,
    ) -> Result<Self, TankError> {
        geometry.require_valid()?;
        propellant.require_valid()?;
        require_valid_mode_count(mode_count)?;
        if !fill_fraction.is_finite() || !(0.0..=1.0).contains(&fill_fraction) {
            return Err(TankError::InvalidFillFraction {
                value: fill_fraction,
            });
        }
        if !mount_point_body_m.iter().all(|c| c.is_finite()) {
            return Err(TankError::InvalidMountPoint);
        }
        if !damping_ratio_zeta.is_finite() || damping_ratio_zeta < 0.0 {
            return Err(TankError::InvalidBaffle {
                reason: "damping_ratio_zeta must be finite and non-negative",
            });
        }
        let TankGeometry::Cylinder {
            radius_m,
            height_m: _,
        } = geometry
        else {
            return Err(TankError::InvalidGeometry {
                reason: "AbramsonMultiMass requires a Cylinder geometry",
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
            modes: vec![AbramsonModeState::default(); mode_count],
        })
    }

    /// Number of active slosh modes.
    #[must_use]
    pub fn mode_count(&self) -> usize {
        self.modes.len()
    }

    /// Set the initial translational slosh state for one mode.
    ///
    /// `mode` is one-based. `(displacement_m.0, displacement_m.1)` is `(x, y)`
    /// in metres and `(velocity_m_s.0, velocity_m_s.1)` is `(dx/dt, dy/dt)`.
    ///
    /// # Errors
    ///
    /// Returns [`TankError`] for a mode outside the configured range or
    /// non-finite inputs.
    pub fn set_initial_mode_slosh(
        &mut self,
        mode: usize,
        displacement_m: (f64, f64),
        velocity_m_s: (f64, f64),
    ) -> Result<(), TankError> {
        if mode == 0 || mode > self.modes.len() {
            return Err(TankError::InvalidSloshModeCount {
                value: mode,
                max: self.modes.len(),
            });
        }
        if !(displacement_m.0.is_finite()
            && displacement_m.1.is_finite()
            && velocity_m_s.0.is_finite()
            && velocity_m_s.1.is_finite())
        {
            return Err(TankError::NonFiniteStepInput {
                reason: "initial slosh state must be finite",
            });
        }
        let state = &mut self.modes[mode - 1];
        state.displacement_x_m = displacement_m.0;
        state.displacement_y_m = displacement_m.1;
        state.velocity_x_m_s = velocity_m_s.0;
        state.velocity_y_m_s = velocity_m_s.1;
        Ok(())
    }

    /// Convenience setter for the first mode, matching the spring-mass scenario
    /// initial-state vocabulary.
    ///
    /// # Errors
    ///
    /// Returns [`TankError`] for non-finite inputs.
    pub fn set_initial_slosh(
        &mut self,
        displacement_m: (f64, f64),
        velocity_m_s: (f64, f64),
    ) -> Result<(), TankError> {
        self.set_initial_mode_slosh(1, displacement_m, velocity_m_s)
    }

    /// Current displacement `(x, y)` for one one-based mode.
    #[must_use]
    pub fn displacement_m(&self, mode: usize) -> Option<(f64, f64)> {
        self.modes
            .get(mode.checked_sub(1)?)
            .map(|state| (state.displacement_x_m, state.displacement_y_m))
    }

    /// Current velocity `(dx/dt, dy/dt)` for one one-based mode.
    #[must_use]
    pub fn velocity_m_s(&self, mode: usize) -> Option<(f64, f64)> {
        self.modes
            .get(mode.checked_sub(1)?)
            .map(|state| (state.velocity_x_m_s, state.velocity_y_m_s))
    }

    /// Current liquid height under the same cylinder convention used by the
    /// existing pendulum and spring-mass models.
    #[must_use]
    pub fn fluid_height_m(&self) -> f64 {
        cylinder_fluid_height_m(
            self.fluid_kg,
            self.tank_radius_m,
            self.propellant_density_kg_m3,
        )
    }

    /// Current closed-form mode parameters for every configured mode.
    #[must_use]
    pub fn mode_parameters(&self) -> Vec<AbramsonModeParams> {
        let h = self.fluid_height_m();
        if h <= 0.0 {
            return Vec::new();
        }
        (1..=self.modes.len())
            .map(|mode| cylinder_mode_parameters(self.tank_radius_m, h, mode))
            .collect()
    }

    /// Current slosh masses in kilograms, declaration/mode order.
    #[must_use]
    pub fn slosh_masses_kg(&self) -> Vec<f64> {
        self.mode_parameters()
            .iter()
            .map(|params| self.fluid_kg * params.mass_fraction)
            .collect()
    }

    /// Current rigidly attached residual mass in kilograms.
    #[must_use]
    pub fn fixed_mass_kg(&self) -> f64 {
        let slosh_kg: f64 = self.slosh_masses_kg().iter().sum();
        (self.fluid_kg - slosh_kg).max(0.0)
    }

    /// Current fixed-mass height relative to the instantaneous liquid center of
    /// mass. It is chosen by the static-center-of-mass constraint.
    #[must_use]
    pub fn fixed_mass_height_from_fluid_cg_m(&self) -> f64 {
        let params = self.mode_parameters();
        if params.is_empty() {
            return 0.0;
        }
        let mut slosh_static_moment_kg_m = 0.0;
        for params in &params {
            let mass_kg = self.fluid_kg * params.mass_fraction;
            slosh_static_moment_kg_m += mass_kg * params.slosh_mass_height_from_fluid_cg_m;
        }
        let fixed_mass_kg = (self.fluid_kg
            - params
                .iter()
                .map(|p| self.fluid_kg * p.mass_fraction)
                .sum::<f64>())
        .max(0.0);
        if fixed_mass_kg > 0.0 {
            -slosh_static_moment_kg_m / fixed_mass_kg
        } else {
            0.0
        }
    }

    /// Closed-form natural frequency squared for one mode at the current fluid
    /// level and supplied axial acceleration.
    #[must_use]
    pub fn omega_n_squared_rad2_s2(&self, mode: usize, axial_accel_m_s2: f64) -> Option<f64> {
        if axial_accel_m_s2 <= 0.0 {
            return Some(0.0);
        }
        let h = self.fluid_height_m();
        if h <= 0.0 || mode == 0 || mode > self.modes.len() {
            return None;
        }
        let params = cylinder_mode_parameters(self.tank_radius_m, h, mode);
        Some(axial_accel_m_s2 / params.pendulum_length_m)
    }

    fn reset_states(&mut self) {
        for state in &mut self.modes {
            *state = AbramsonModeState::default();
        }
    }

    fn mass_split(&self) -> (Vec<AbramsonModeParams>, Vec<f64>, f64, f64) {
        let params = self.mode_parameters();
        let mut masses = Vec::with_capacity(params.len());
        let mut slosh_mass_sum = 0.0;
        let mut slosh_static_moment_kg_m = 0.0;
        for params in &params {
            let mass_kg = self.fluid_kg * params.mass_fraction;
            masses.push(mass_kg);
            slosh_mass_sum += mass_kg;
            slosh_static_moment_kg_m += mass_kg * params.slosh_mass_height_from_fluid_cg_m;
        }
        let fixed_mass_kg = (self.fluid_kg - slosh_mass_sum).max(0.0);
        let fixed_height_m = if fixed_mass_kg > 0.0 {
            -slosh_static_moment_kg_m / fixed_mass_kg
        } else {
            0.0
        };
        (params, masses, fixed_mass_kg, fixed_height_m)
    }
}

impl MovingMassModel for AbramsonMultiMass {
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
        if self.fluid_kg == 0.0 {
            self.reset_states();
            return Ok(());
        }

        let h = self.fluid_height_m();
        if h <= 0.0 {
            self.reset_states();
            return Ok(());
        }

        let axial_accel = accel_body_m_s2.z;
        let lateral_accel_x = accel_body_m_s2.x;
        let lateral_accel_y = accel_body_m_s2.y;
        for (index, state) in self.modes.iter_mut().enumerate() {
            let params = cylinder_mode_parameters(self.tank_radius_m, h, index + 1);
            let omega_n_squared = if axial_accel > 0.0 {
                axial_accel / params.pendulum_length_m
            } else {
                0.0
            };
            let omega_n = omega_n_squared.sqrt();

            let damping_x = 2.0 * self.damping_ratio_zeta * omega_n * state.velocity_x_m_s;
            let restoring_x = omega_n_squared * state.displacement_x_m;
            let forcing_x = lateral_accel_x;
            let accel_x = forcing_x - damping_x - restoring_x;
            state.last_accel_x_m_s2 = accel_x;
            state.velocity_x_m_s += dt_s * accel_x;
            state.displacement_x_m += dt_s * state.velocity_x_m_s;

            let damping_y = 2.0 * self.damping_ratio_zeta * omega_n * state.velocity_y_m_s;
            let restoring_y = omega_n_squared * state.displacement_y_m;
            let forcing_y = lateral_accel_y;
            let accel_y = forcing_y - damping_y - restoring_y;
            state.last_accel_y_m_s2 = accel_y;
            state.velocity_y_m_s += dt_s * accel_y;
            state.displacement_y_m += dt_s * state.velocity_y_m_s;
        }

        Ok(())
    }

    fn mass_contribution(&self) -> MassContribution {
        if self.fluid_kg == 0.0 {
            return MassContribution::default();
        }

        let (params, masses, fixed_mass_kg, fixed_height_m) = self.mass_split();
        let mut weighted_cg = Vector3::zeros();
        let mut inertia = Matrix3::zeros();

        if fixed_mass_kg > 0.0 {
            let fixed_offset = Vector3::new(0.0, 0.0, fixed_height_m);
            weighted_cg += fixed_mass_kg * fixed_offset;
            inertia += point_mass_inertia_about_origin(
                fixed_mass_kg,
                self.mount_point_body_m + fixed_offset,
            );
        }

        for ((params, mass_kg), state) in params.iter().zip(masses.iter()).zip(self.modes.iter()) {
            if *mass_kg == 0.0 {
                continue;
            }
            let offset = Vector3::new(
                state.displacement_x_m,
                state.displacement_y_m,
                params.slosh_mass_height_from_fluid_cg_m,
            );
            weighted_cg += *mass_kg * offset;
            inertia += point_mass_inertia_about_origin(*mass_kg, self.mount_point_body_m + offset);
        }

        MassContribution {
            mass_kg: self.fluid_kg,
            cg_offset_body_m: weighted_cg / self.fluid_kg,
            inertia_delta_body_kg_m2: inertia,
        }
    }

    fn reaction_body(&self) -> ForceMomentBody {
        if self.fluid_kg == 0.0 {
            return ForceMomentBody::default();
        }

        let (params, masses, _, _) = self.mass_split();
        let mut force_total = Vector3::zeros();
        let mut moment_total = Vector3::zeros();
        for ((params, mass_kg), state) in params.iter().zip(masses.iter()).zip(self.modes.iter()) {
            if *mass_kg == 0.0 {
                continue;
            }
            let force = Vector3::new(
                -*mass_kg * state.last_accel_x_m_s2,
                -*mass_kg * state.last_accel_y_m_s2,
                0.0,
            );
            let offset = Vector3::new(
                state.displacement_x_m,
                state.displacement_y_m,
                params.slosh_mass_height_from_fluid_cg_m,
            );
            force_total += force;
            moment_total += (self.mount_point_body_m + offset).cross(&force);
        }

        ForceMomentBody {
            force_body_n: force_total,
            moment_body_n_m: moment_total,
        }
    }

    fn fluid_remaining_kg(&self) -> f64 {
        self.fluid_kg
    }
}

/// Closed-form cylindrical mode parameters for one one-based mode.
///
/// # Panics
///
/// Panics if `mode` is outside `1..=MAX_ABRAMSON_CYLINDER_MODES` or geometry
/// dimensions are invalid. Public constructors validate these conditions before
/// calling the helper.
#[must_use]
pub fn cylinder_mode_parameters(
    radius_m: f64,
    fluid_height_m: f64,
    mode: usize,
) -> AbramsonModeParams {
    assert!(radius_m.is_finite() && radius_m > 0.0);
    assert!(fluid_height_m.is_finite() && fluid_height_m > 0.0);
    assert!((1..=MAX_ABRAMSON_CYLINDER_MODES).contains(&mode));

    let diameter_m = 2.0 * radius_m;
    let xi = CYLINDER_J1_DERIVATIVE_ROOTS[mode - 1];
    let h_over_d = fluid_height_m / diameter_m;
    let two_xi_h_over_d = 2.0 * xi * h_over_d;
    let xi_h_over_d = xi * h_over_d;
    let mass_fraction = two_xi_h_over_d.tanh() / (xi * (xi * xi - 1.0) * h_over_d);
    let pendulum_length_m = diameter_m / (2.0 * xi * two_xi_h_over_d.tanh());
    let hinge_height_from_bottom_m = 0.5 * fluid_height_m
        - (diameter_m / (2.0 * xi)) * (xi_h_over_d.tanh() - 1.0 / two_xi_h_over_d.sinh());
    let hinge_height_from_fluid_cg_m = hinge_height_from_bottom_m - 0.5 * fluid_height_m;
    let slosh_mass_height_from_fluid_cg_m = hinge_height_from_fluid_cg_m - pendulum_length_m;

    AbramsonModeParams {
        mode,
        xi,
        mass_fraction,
        pendulum_length_m,
        hinge_height_from_bottom_m,
        hinge_height_from_fluid_cg_m,
        slosh_mass_height_from_fluid_cg_m,
    }
}

fn cylinder_fluid_height_m(fluid_kg: f64, radius_m: f64, density_kg_m3: f64) -> f64 {
    let cross_section = std::f64::consts::PI * radius_m * radius_m;
    let volume = fluid_kg / density_kg_m3;
    volume / cross_section
}

fn require_valid_mode_count(mode_count: usize) -> Result<(), TankError> {
    if !(1..=MAX_ABRAMSON_CYLINDER_MODES).contains(&mode_count) {
        return Err(TankError::InvalidSloshModeCount {
            value: mode_count,
            max: MAX_ABRAMSON_CYLINDER_MODES,
        });
    }
    Ok(())
}

#[cfg(test)]
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::expect_used,
    clippy::panic,
    clippy::unwrap_used
)]
mod tests {
    use super::*;
    use serde::Deserialize;

    const SLOSH_DECK: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/slosh/abramson-cylinder-table-v1.toml"
    ));
    const SLOSH_PROVENANCE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/slosh/provenance.md"
    ));
    const SLOSH_DECK_SHA256: &str =
        "92a9b642aada4b1d65a38e88c4cc16c09a666613adc5069c0d986acc69717cbe";
    const DODGE_SOURCE_PDF_SHA256: &str =
        "2985277d1b7a0b7ad445cc11c5bc56587a89a5b8998d5ddc549976d75c5f0d31";

    #[derive(Debug, Deserialize)]
    struct SloshDeck {
        schema: String,
        source_pdf_sha256: String,
        cases: Vec<SloshDeckCase>,
    }

    #[derive(Debug, Deserialize)]
    struct SloshDeckCase {
        h_over_d: f64,
        mode: usize,
        xi: f64,
        mass_fraction: f64,
        pendulum_length_over_d: f64,
        hinge_height_over_d: f64,
        slosh_mass_height_over_d: f64,
    }

    fn water() -> PropellantSpec {
        PropellantSpec {
            density_kg_m3: 1000.0,
            label: "water_textbook",
        }
    }

    fn cylinder_d1_h1() -> TankGeometry {
        TankGeometry::Cylinder {
            radius_m: 0.5,
            height_m: 1.0,
        }
    }

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        let error = (actual - expected).abs();
        assert!(
            error <= tolerance,
            "actual {actual} differs from expected {expected} by {error}, tolerance {tolerance}"
        );
    }

    #[test]
    fn rejects_non_cylinder_geometry() {
        let result = AbramsonMultiMass::new(
            TankGeometry::Sphere { radius_m: 1.0 },
            water(),
            1.0,
            Vector3::zeros(),
            0.0,
            1,
        );
        assert!(matches!(result, Err(TankError::InvalidGeometry { .. })));
    }

    #[test]
    fn rejects_zero_or_unsupported_mode_count() {
        let zero = AbramsonMultiMass::new(cylinder_d1_h1(), water(), 1.0, Vector3::zeros(), 0.0, 0);
        assert!(matches!(zero, Err(TankError::InvalidSloshModeCount { .. })));

        let too_many = AbramsonMultiMass::new(
            cylinder_d1_h1(),
            water(),
            1.0,
            Vector3::zeros(),
            0.0,
            MAX_ABRAMSON_CYLINDER_MODES + 1,
        );
        assert!(matches!(
            too_many,
            Err(TankError::InvalidSloshModeCount { .. })
        ));
    }

    #[test]
    fn cylindrical_parameters_match_dodge_table_3_2_values() {
        let params = [
            cylinder_mode_parameters(0.5, 1.0, 1),
            cylinder_mode_parameters(0.5, 1.0, 2),
            cylinder_mode_parameters(0.5, 1.0, 3),
        ];

        let expected_xi = [1.841, 5.329, 8.531];
        let expected_mass_fraction = [0.226_967, 0.006_839, 0.001_630];
        let expected_l_over_d = [0.271_909, 0.093_783, 0.058_573];
        let expected_hinge_over_d = [0.255_443, 0.406_226, 0.441_427];
        for (index, params) in params.iter().enumerate() {
            assert_close(params.xi, expected_xi[index], expected_xi[index] * 0.01);
            assert_close(
                params.mass_fraction,
                expected_mass_fraction[index],
                expected_mass_fraction[index] * 0.01,
            );
            assert_close(
                params.pendulum_length_m,
                expected_l_over_d[index],
                expected_l_over_d[index] * 0.01,
            );
            assert_close(
                params.hinge_height_from_bottom_m,
                expected_hinge_over_d[index],
                expected_hinge_over_d[index] * 0.01,
            );
        }
    }

    #[test]
    fn checked_in_dodge_table_matches_closed_form_helper() {
        assert!(SLOSH_PROVENANCE.contains(SLOSH_DECK_SHA256));
        assert!(SLOSH_PROVENANCE.contains(DODGE_SOURCE_PDF_SHA256));

        let deck: SloshDeck = toml::from_str(SLOSH_DECK).unwrap();
        assert_eq!(deck.schema, "openbmp.slosh.abramson-cylinder-table.v1");
        assert_eq!(deck.source_pdf_sha256, DODGE_SOURCE_PDF_SHA256);
        assert_eq!(deck.cases.len(), 12);
        for case in &deck.cases {
            let diameter_m = 1.0;
            let params = cylinder_mode_parameters(0.5 * diameter_m, case.h_over_d, case.mode);
            assert_close(params.xi, case.xi, case.xi.abs() * 0.01);
            assert_close(
                params.mass_fraction,
                case.mass_fraction,
                case.mass_fraction.abs() * 0.01,
            );
            assert_close(
                params.pendulum_length_m / diameter_m,
                case.pendulum_length_over_d,
                case.pendulum_length_over_d.abs() * 0.01,
            );
            assert_close(
                params.hinge_height_from_bottom_m / diameter_m,
                case.hinge_height_over_d,
                case.hinge_height_over_d.abs() * 0.01,
            );
            assert_close(
                params.slosh_mass_height_from_fluid_cg_m / diameter_m,
                case.slosh_mass_height_over_d,
                case.slosh_mass_height_over_d.abs() * 0.01,
            );
        }
    }

    #[test]
    fn fixed_mass_is_residual_and_static_cg_balances_mount() {
        let model =
            AbramsonMultiMass::new(cylinder_d1_h1(), water(), 1.0, Vector3::zeros(), 0.0, 3)
                .unwrap();
        let total = model.fluid_remaining_kg();
        let slosh_mass_kg: f64 = model.slosh_masses_kg().iter().sum();
        let fixed = model.fixed_mass_kg();
        assert_close(fixed + slosh_mass_kg, total, total * 1e-14);
        assert_close(fixed / total, 0.764_564, 0.764_564 * 0.01);

        let mass = model.mass_contribution();
        assert_close(mass.cg_offset_body_m.x, 0.0, 1e-14);
        assert_close(mass.cg_offset_body_m.y, 0.0, 1e-14);
        assert_close(mass.cg_offset_body_m.z, 0.0, 1e-14);
    }

    #[test]
    fn free_response_frequency_matches_closed_form_first_mode() {
        let mut model =
            AbramsonMultiMass::new(cylinder_d1_h1(), water(), 1.0, Vector3::zeros(), 0.0, 3)
                .unwrap();
        model.set_initial_slosh((0.05, 0.0), (0.0, 0.0)).unwrap();
        let g = 9.81;
        let omega_sq = model.omega_n_squared_rad2_s2(1, g).unwrap();
        let omega_n = omega_sq.sqrt();
        let expected_period = 2.0 * std::f64::consts::PI / omega_n;
        let dt = Duration::from_seconds(0.001);
        let dt_s = dt.as_seconds();

        let mut prev_x = 0.05;
        let mut crossings: Vec<f64> = Vec::new();
        let mut t = 0.0;
        for _ in 0..200_000 {
            model
                .step(Vector3::new(0.0, 0.0, g), Vector3::zeros(), dt)
                .unwrap();
            t += dt_s;
            let (x, _) = model.displacement_m(1).unwrap();
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
    fn first_mode_energy_is_conserved_within_one_percent_over_100_cycles() {
        let mut model =
            AbramsonMultiMass::new(cylinder_d1_h1(), water(), 1.0, Vector3::zeros(), 0.0, 3)
                .unwrap();
        model.set_initial_slosh((0.05, 0.0), (0.0, 0.0)).unwrap();
        let g = 9.81;
        let omega_sq = model.omega_n_squared_rad2_s2(1, g).unwrap();
        let initial_energy = 0.5 * omega_sq * 0.05_f64.powi(2);
        let dt = Duration::from_seconds(0.001);
        let cycles_target = 100;
        let total_steps = (((cycles_target as f64) * 2.0 * std::f64::consts::PI / omega_sq.sqrt())
            / dt.as_seconds())
        .ceil() as usize;

        let mut min_e = initial_energy;
        let mut max_e = initial_energy;
        for _ in 0..total_steps {
            model
                .step(Vector3::new(0.0, 0.0, g), Vector3::zeros(), dt)
                .unwrap();
            let (x, _) = model.displacement_m(1).unwrap();
            let (v, _) = model.velocity_m_s(1).unwrap();
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
    fn moving_first_mode_shifts_cg_less_than_all_fluid_mass() {
        let mut model =
            AbramsonMultiMass::new(cylinder_d1_h1(), water(), 1.0, Vector3::zeros(), 0.0, 3)
                .unwrap();
        model.set_initial_slosh((0.1, 0.0), (0.0, 0.0)).unwrap();
        let mass = model.mass_contribution();
        let first_mode_fraction = cylinder_mode_parameters(0.5, 1.0, 1).mass_fraction;
        assert_close(
            mass.cg_offset_body_m.x,
            0.1 * first_mode_fraction,
            0.1 * first_mode_fraction * 1e-12,
        );
    }

    #[test]
    fn empty_tank_resets_all_mode_states() {
        let mut model =
            AbramsonMultiMass::new(cylinder_d1_h1(), water(), 0.001, Vector3::zeros(), 0.0, 3)
                .unwrap();
        model
            .set_initial_mode_slosh(2, (0.05, -0.02), (0.1, -0.1))
            .unwrap();
        model.drain(1e6).unwrap();
        model
            .step(
                Vector3::new(3.0, -4.0, 9.81),
                Vector3::zeros(),
                Duration::from_seconds(1.0),
            )
            .unwrap();

        assert_eq!(model.fluid_remaining_kg().to_bits(), 0.0_f64.to_bits());
        for mode in 1..=3 {
            assert_eq!(
                model.displacement_m(mode).unwrap().0.to_bits(),
                0.0_f64.to_bits()
            );
            assert_eq!(
                model.displacement_m(mode).unwrap().1.to_bits(),
                0.0_f64.to_bits()
            );
            assert_eq!(
                model.velocity_m_s(mode).unwrap().0.to_bits(),
                0.0_f64.to_bits()
            );
            assert_eq!(
                model.velocity_m_s(mode).unwrap().1.to_bits(),
                0.0_f64.to_bits()
            );
        }
    }
}
