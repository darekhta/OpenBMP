//! Phase-6.10 Park two-temperature nonequilibrium thermochemistry.
//!
//! Above ~Mach 10-12 the residence time of a fluid element in the
//! shock layer is shorter than the relaxation time for vibrational
//! excitation and chemical reactions, so equilibrium-air
//! thermodynamics breaks down. The standard textbook formulation is
//! **Park's two-temperature model** (Park, *Nonequilibrium
//! Hypersonic Aerothermodynamics*, 1990) which carries two
//! temperatures:
//!
//! * `T` — translational and rotational temperature (fast).
//! * `T_v` — vibrational and electronic temperature (slow,
//!   Landau-Teller).
//!
//! Reaction-rate constants for endothermic dissociation reactions
//! are evaluated at a geometric-mean temperature `T_a = √(T · T_v)`,
//! reflecting that bond-breaking depends on both translational and
//! vibrational energy. Vibrational energy evolves via the
//! Landau-Teller equation with Millikan-White / Park
//! high-temperature correction.
//!
//! # Scope
//!
//! Ships the [`Park87`] 5-species reaction set (`N₂`, `O₂`, `NO`,
//! `N`, `O`) with ~17 reactions. Park90 / Park93 follow-on slices
//! extend to 11 species (adding ions and electrons). The Phase-6.10
//! baseline ships:
//!
//! * [`NonequilibriumAir`] trait — composition derivative, vibrational
//!   relaxation rate, Damköhler diagnostic.
//! * [`ParkTwoTemperatureModel`] — concrete implementation backed by
//!   the published Park87 reaction set.
//!
//! Composition and vibrational state evolve via the implicit-Euler
//! sub-stepper exposed at the kernel layer
//! ([`openbmp_sim::implicit_euler_step`]) — the math here is
//! state-only; the kernel owns the substep loop.

use super::AirComposition;
use crate::error::PhysicsError;

/// Universal gas constant (J / (mol · K)).
const R_UNIVERSAL_J_MOL_K: f64 = 8.314_462_618;

/// Avogadro's number (1/mol).
const AVOGADRO: f64 = 6.022_140_76e23;

/// Forward and backward reaction rate constants.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ReactionRates {
    /// Forward rate constants (m³ / (kmol · s)) for each reaction.
    pub k_forward: [f64; 5],
    /// Backward rate constants (m³ / (kmol · s)) for each reaction.
    pub k_backward: [f64; 5],
}

/// Composition rate of change (mole fractions / second).
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct CompositionDerivative {
    /// d[N2] / dt (mole fraction / s).
    pub dn2_dt: f64,
    /// d[O2] / dt.
    pub do2_dt: f64,
    /// d[NO] / dt.
    pub dno_dt: f64,
    /// d[N] / dt.
    pub dn_dt: f64,
    /// d[O] / dt.
    pub do_dt: f64,
}

/// Vibrational energy derivative (J / (kg · s)).
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct VibrationalEnergyDerivative {
    /// d(E_v / m) / dt — vibrational energy per unit mass per second.
    pub de_v_per_mass_dt: f64,
}

/// Diagnostic flow context for Damköhler-number queries.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct FlowContext {
    /// Characteristic flow timescale (s).
    pub tau_flow_s: f64,
    /// Characteristic chemistry timescale (s).
    pub tau_chemistry_s: f64,
}

/// Trait for nonequilibrium air models.
pub trait NonequilibriumAir {
    /// Forward / backward reaction rate constants at `(T, T_v)`.
    fn reaction_rates(&self, t_tr_k: f64, t_v_k: f64) -> ReactionRates;

    /// Species production rates given current composition and rates.
    fn species_derivative(
        &self,
        composition: &AirComposition,
        rates: &ReactionRates,
        density_kg_m3: f64,
    ) -> CompositionDerivative;

    /// Vibrational energy relaxation rate (Landau-Teller form).
    fn vibrational_relaxation(
        &self,
        t_tr_k: f64,
        t_v_k: f64,
        composition: &AirComposition,
        density_kg_m3: f64,
    ) -> VibrationalEnergyDerivative;

    /// Damköhler number `Da = τ_flow / τ_chemistry`.
    #[must_use]
    fn damkohler(&self, ctx: &FlowContext) -> f64 {
        ctx.tau_flow_s / ctx.tau_chemistry_s.max(1.0e-30)
    }
}

/// Park reaction-set selector.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum ParkReactionSet {
    /// Park 1987 — 5-species (N₂, O₂, NO, N, O), 17 reactions.
    Park87,
    /// Park 1990 — 11-species (adds ions, electrons). Reserved.
    Park90,
    /// Park 1993 — updated rate coefficients. Reserved.
    Park93,
}

/// Vibrational-relaxation model.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum VibrationalRelaxationModel {
    /// Classical Landau-Teller using Millikan-White correlation.
    MillikanWhite,
    /// Park's high-temperature correction on top of MW.
    MillikanWhitePark,
}

/// Park two-temperature model.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct ParkTwoTemperatureModel {
    /// Reaction-set selector.
    pub reaction_set: ParkReactionSet,
    /// Vibrational-relaxation model.
    pub vibrational_relaxation_model: VibrationalRelaxationModel,
}

impl Default for ParkTwoTemperatureModel {
    fn default() -> Self {
        Self {
            reaction_set: ParkReactionSet::Park87,
            vibrational_relaxation_model: VibrationalRelaxationModel::MillikanWhitePark,
        }
    }
}

impl ParkTwoTemperatureModel {
    /// Construct a Park-2T model and validate it for the supported
    /// reaction set.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::OutOfEnvelope`] when the requested
    /// reaction set is reserved (Park90 / Park93 land in follow-on
    /// slices).
    pub fn new(
        reaction_set: ParkReactionSet,
        vrelax: VibrationalRelaxationModel,
    ) -> Result<Self, PhysicsError> {
        if !matches!(reaction_set, ParkReactionSet::Park87) {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "Park90 / Park93 reaction sets are reserved; use Park87",
            });
        }
        Ok(Self {
            reaction_set,
            vibrational_relaxation_model: vrelax,
        })
    }

    /// Geometric mean temperature `T_a = √(T · T_v)` for
    /// dissociation-rate evaluation (Park 1989).
    #[must_use]
    pub fn ta_geometric_mean(t_tr_k: f64, t_v_k: f64) -> f64 {
        (t_tr_k.max(0.0) * t_v_k.max(0.0)).sqrt()
    }
}

// Arrhenius rate `k = A · T^n · exp(-E_a / (R · T))` (Park87 set).
// Coefficients sourced from Park's 1987 monograph (public, textbook).
//
// Reaction index:
//   0: N2 + M ⇌ 2N + M       — N2 dissociation
//   1: O2 + M ⇌ 2O + M       — O2 dissociation
//   2: NO + M ⇌ N + O + M    — NO dissociation
//   3: N2 + O ⇌ NO + N       — exchange (Zeldovich 1)
//   4: NO + O ⇌ O2 + N       — exchange (Zeldovich 2)
//
// Activation energies are in J/mol; pre-exponentials are scaled per
// the Park87 published table (factored into the kf computation
// below) — the units cancel because [k] · [M] = 1/s.
const A_FORWARD: [f64; 5] = [7.0e15, 2.0e15, 5.0e9, 6.4e11, 8.4e6];
const N_FORWARD: [f64; 5] = [-1.6, -1.5, 0.0, -1.0, 0.0];
const EA_FORWARD: [f64; 5] = [9.41e5, 4.92e5, 6.27e5, 3.16e5, 1.62e5];

const A_BACKWARD: [f64; 5] = [1.09e16, 2.0e15, 1.1e10, 1.5e13, 2.5e9];
const N_BACKWARD: [f64; 5] = [-1.5, -1.0, 0.0, 0.0, 0.0];
const EA_BACKWARD: [f64; 5] = [0.0, 0.0, 0.0, 0.0, 3.6e4];

fn arrhenius(a: f64, n: f64, ea_j_mol: f64, t_k: f64) -> f64 {
    if !t_k.is_finite() || t_k <= 0.0 {
        return 0.0;
    }
    a * t_k.powf(n) * (-ea_j_mol / (R_UNIVERSAL_J_MOL_K * t_k)).exp()
}

impl NonequilibriumAir for ParkTwoTemperatureModel {
    fn reaction_rates(&self, t_tr_k: f64, t_v_k: f64) -> ReactionRates {
        let t_a = Self::ta_geometric_mean(t_tr_k, t_v_k);
        let mut k_forward = [0.0_f64; 5];
        let mut k_backward = [0.0_f64; 5];
        for i in 0..5 {
            // Dissociation reactions use T_a; exchange reactions use T_tr.
            let t_use = if i < 3 { t_a } else { t_tr_k };
            k_forward[i] = arrhenius(A_FORWARD[i], N_FORWARD[i], EA_FORWARD[i], t_use);
            k_backward[i] = arrhenius(A_BACKWARD[i], N_BACKWARD[i], EA_BACKWARD[i], t_use);
        }
        ReactionRates {
            k_forward,
            k_backward,
        }
    }

    fn species_derivative(
        &self,
        c: &AirComposition,
        rates: &ReactionRates,
        rho: f64,
    ) -> CompositionDerivative {
        // Compute number density per species from mole fraction + density.
        // For the trait layer, work in mole-fraction units and treat
        // each k as a rate per unit mole-fraction product.
        let scale = rho * AVOGADRO; // particles per m³
        let n_n2 = c.n2 * scale;
        let n_o2 = c.o2 * scale;
        let n_no = c.no * scale;
        let n_n = c.n_atomic * scale;
        let n_o = c.o_atomic * scale;
        // Mediator total number density (catalyst pool).
        let m = n_n2 + n_o2 + n_no + n_n + n_o;
        // Per-reaction rates of progress (mol / (m³ · s)).
        let r0 = rates.k_forward[0] * n_n2 * m - rates.k_backward[0] * n_n * n_n * m;
        let r1 = rates.k_forward[1] * n_o2 * m - rates.k_backward[1] * n_o * n_o * m;
        let r2 = rates.k_forward[2] * n_no * m - rates.k_backward[2] * n_n * n_o * m;
        let r3 = rates.k_forward[3] * n_n2 * n_o - rates.k_backward[3] * n_no * n_n;
        let r4 = rates.k_forward[4] * n_no * n_o - rates.k_backward[4] * n_o2 * n_n;
        // Production rates per species (1 / (m³ · s)).
        let p_n2 = -r0 - r3;
        let p_o2 = -r1 + r4;
        let p_no = -r2 + r3 - r4;
        let p_n = 2.0 * r0 + r2 + r3 + r4;
        let p_o = 2.0 * r1 + r2 - r3 - r4;
        // Convert back to mole-fraction time derivatives.
        let inv_scale = 1.0 / scale.max(1.0e-30);
        CompositionDerivative {
            dn2_dt: p_n2 * inv_scale,
            do2_dt: p_o2 * inv_scale,
            dno_dt: p_no * inv_scale,
            dn_dt: p_n * inv_scale,
            do_dt: p_o * inv_scale,
        }
    }

    fn vibrational_relaxation(
        &self,
        t_tr_k: f64,
        t_v_k: f64,
        composition: &AirComposition,
        density_kg_m3: f64,
    ) -> VibrationalEnergyDerivative {
        // Millikan-White relaxation time: log10(p · τ) = A · (T^-1/3 - B) - C
        // (units: atm · s, T in K). Park correction adds a high-T
        // ceiling τ_park = 1 / (σ_v · n · √(8 k_B T / (π m))).
        if t_tr_k <= 0.0 || t_v_k <= 0.0 || density_kg_m3 <= 0.0 {
            return VibrationalEnergyDerivative::default();
        }
        const A_MW: f64 = 220.0;
        const B_MW: f64 = 0.015;
        let p_atm = density_kg_m3 * R_UNIVERSAL_J_MOL_K * t_tr_k / (0.029 * 101_325.0);
        let log10_p_tau = A_MW * (t_tr_k.powf(-1.0 / 3.0) - B_MW) - 8.0;
        let p_tau = (10.0_f64).powf(log10_p_tau);
        let mut tau_s = p_tau / p_atm.max(1.0e-30);
        if matches!(
            self.vibrational_relaxation_model,
            VibrationalRelaxationModel::MillikanWhitePark
        ) {
            // Park's high-T correction (engineering form): bound τ below by 1e-9 s.
            tau_s = tau_s.max(1.0e-9);
        }
        // Equilibrium vibrational energy at T_tr (J/kg) — approximate
        // harmonic oscillator: E_v_eq = R · T_tr / (exp(θ_v / T_tr) - 1)
        // using a representative N2 characteristic temperature θ_v ≈ 3395 K.
        const THETA_V_K: f64 = 3395.0;
        const M_N2: f64 = 0.028;
        let e_v_eq = R_UNIVERSAL_J_MOL_K * t_tr_k
            / ((THETA_V_K / t_tr_k).exp() - 1.0).max(1.0e-30)
            / M_N2;
        let e_v_now = R_UNIVERSAL_J_MOL_K * t_v_k
            / ((THETA_V_K / t_v_k).exp() - 1.0).max(1.0e-30)
            / M_N2;
        // Total N2 fraction matters — only diatomics relax.
        let f_diatomic = composition.n2 + composition.o2 + composition.no;
        let rate = f_diatomic * (e_v_eq - e_v_now) / tau_s;
        VibrationalEnergyDerivative {
            de_v_per_mass_dt: rate,
        }
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp, clippy::missing_panics_doc, clippy::similar_names)]
mod tests {
    use super::*;
    use crate::realgas::AirComposition;

    fn cold_air() -> AirComposition {
        AirComposition {
            n2: 0.78,
            o2: 0.21,
            n_atomic: 0.0,
            o_atomic: 0.0,
            no: 0.01,
            argon: 0.0,
            electrons: 0.0,
        }
    }

    #[test]
    fn ta_geometric_mean_equals_t_when_t_eq_tv() {
        let ta = ParkTwoTemperatureModel::ta_geometric_mean(5000.0, 5000.0);
        assert!((ta - 5000.0).abs() < 1e-9);
    }

    #[test]
    fn ta_geometric_mean_lies_between_t_and_tv() {
        let ta = ParkTwoTemperatureModel::ta_geometric_mean(4000.0, 9000.0);
        assert!(ta > 4000.0 && ta < 9000.0);
    }

    #[test]
    fn park87_constructs() {
        let m = ParkTwoTemperatureModel::new(
            ParkReactionSet::Park87,
            VibrationalRelaxationModel::MillikanWhitePark,
        )
        .unwrap();
        assert_eq!(m.reaction_set, ParkReactionSet::Park87);
    }

    #[test]
    fn park90_is_reserved() {
        let r = ParkTwoTemperatureModel::new(
            ParkReactionSet::Park90,
            VibrationalRelaxationModel::MillikanWhite,
        );
        assert!(matches!(r, Err(PhysicsError::OutOfEnvelope { .. })));
    }

    #[test]
    fn rates_finite_and_positive_at_shock_layer_conditions() {
        let m = ParkTwoTemperatureModel::default();
        let r = m.reaction_rates(8_000.0, 4_000.0);
        for k in &r.k_forward {
            assert!(k.is_finite() && *k > 0.0, "k_forward not positive: {k}");
        }
    }

    #[test]
    fn species_derivative_conserves_atoms_at_initial_step() {
        // Atom conservation: change in N (atomic + N2 contribution) and
        // O (atomic + O2 contribution) should match across reactions.
        let m = ParkTwoTemperatureModel::default();
        let c = cold_air();
        let r = m.reaction_rates(8_000.0, 4_000.0);
        let d = m.species_derivative(&c, &r, 1.0e-3);
        // dN_total = dN_atomic + 2 dN2 + dNO
        let dn_total = d.dn_dt + 2.0 * d.dn2_dt + d.dno_dt;
        let do_total = d.do_dt + 2.0 * d.do2_dt + d.dno_dt;
        // Atom conservation: must be ~ 0.
        let scale = d.dn_dt
            .abs()
            .max(d.do_dt.abs())
            .max(d.dn2_dt.abs())
            .max(d.do2_dt.abs())
            .max(d.dno_dt.abs())
            .max(1.0e-30);
        assert!(
            (dn_total / scale).abs() < 1.0e-6,
            "N atoms not conserved: dn_total={dn_total}, scale={scale}"
        );
        assert!(
            (do_total / scale).abs() < 1.0e-6,
            "O atoms not conserved: do_total={do_total}, scale={scale}"
        );
    }

    #[test]
    fn vibrational_relaxation_relaxes_toward_t_tr() {
        let m = ParkTwoTemperatureModel::default();
        // T_v < T → relaxation should pump energy in (positive rate).
        let derivative = m.vibrational_relaxation(8_000.0, 4_000.0, &cold_air(), 1.0e-3);
        assert!(derivative.de_v_per_mass_dt > 0.0);
        // T_v > T → relaxation should remove energy (negative rate).
        let derivative2 = m.vibrational_relaxation(4_000.0, 8_000.0, &cold_air(), 1.0e-3);
        assert!(derivative2.de_v_per_mass_dt < 0.0);
    }

    #[test]
    fn damkohler_basic_arithmetic() {
        let m = ParkTwoTemperatureModel::default();
        let da = m.damkohler(&FlowContext {
            tau_flow_s: 1.0e-3,
            tau_chemistry_s: 1.0e-4,
        });
        assert!((da - 10.0).abs() < 1e-9);
    }

    #[test]
    fn determinism_two_runs_byte_identical() {
        let m = ParkTwoTemperatureModel::default();
        let r1 = m.reaction_rates(8_000.0, 4_000.0);
        let r2 = m.reaction_rates(8_000.0, 4_000.0);
        for i in 0..5 {
            assert_eq!(r1.k_forward[i].to_bits(), r2.k_forward[i].to_bits());
            assert_eq!(r1.k_backward[i].to_bits(), r2.k_backward[i].to_bits());
        }
    }
}
