//! Phase-6.10 Park two-temperature nonequilibrium thermochemistry.
//!
//! The Park two-temperature formulation is the intended nonequilibrium
//! air model for OpenBMP, but the first Phase-6 implementation carried
//! a hand-entered five-reaction proxy while the design document called
//! for a published Park87 reaction set with roughly seventeen
//! reactions and species-specific Millikan-White constants. The audit
//! could not verify those coefficients against a public table in the
//! repository.
//!
//! # Honest Scope
//!
//! This module now keeps only the typed API surface and fails closed
//! for every reaction-set query. A follow-on slice must add
//! provenance-backed Park87/Park90/Park93 tables, species-pair
//! vibrational relaxation constants, and the Mach-15 shock-layer
//! public benchmark before this model can return reaction rates.

use super::AirComposition;
use crate::error::PhysicsError;

/// Forward and backward reaction rate constants.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ReactionRates {
    /// Forward rate constants for each reaction.
    pub k_forward: [f64; 5],
    /// Backward rate constants for each reaction.
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
    /// `d(E_v / m) / dt` — vibrational energy per unit mass per second.
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
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::OutOfEnvelope`] when the selected
    /// reaction set is reserved or the query is outside the model
    /// envelope.
    fn reaction_rates(&self, t_tr_k: f64, t_v_k: f64) -> Result<ReactionRates, PhysicsError>;

    /// Species production rates given current composition and rates.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::OutOfEnvelope`] when the model is
    /// reserved.
    fn species_derivative(
        &self,
        composition: &AirComposition,
        rates: &ReactionRates,
        density_kg_m3: f64,
    ) -> Result<CompositionDerivative, PhysicsError>;

    /// Vibrational energy relaxation rate (Landau-Teller form).
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::OutOfEnvelope`] when the model is
    /// reserved.
    fn vibrational_relaxation(
        &self,
        t_tr_k: f64,
        t_v_k: f64,
        composition: &AirComposition,
        density_kg_m3: f64,
    ) -> Result<VibrationalEnergyDerivative, PhysicsError>;

    /// Damköhler number `Da = τ_flow / τ_chemistry`.
    #[must_use]
    fn damkohler(&self, ctx: &FlowContext) -> f64 {
        ctx.tau_flow_s / ctx.tau_chemistry_s.max(1.0e-30)
    }
}

/// Park reaction-set selector.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum ParkReactionSet {
    /// Park 1987 — 5-species (`N2`, `O2`, `NO`, `N`, `O`). Reserved
    /// until verified public coefficients land.
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
    /// Construct a Park-2T model.
    ///
    /// # Errors
    ///
    /// Always returns [`PhysicsError::OutOfEnvelope`] until verified
    /// Park reaction tables and Millikan-White constants land.
    pub fn new(
        _reaction_set: ParkReactionSet,
        _vrelax: VibrationalRelaxationModel,
    ) -> Result<Self, PhysicsError> {
        Err(Self::deferred())
    }

    /// Geometric mean temperature `T_a = sqrt(T * T_v)` for
    /// dissociation-rate evaluation.
    #[must_use]
    pub fn ta_geometric_mean(t_tr_k: f64, t_v_k: f64) -> f64 {
        (t_tr_k.max(0.0) * t_v_k.max(0.0)).sqrt()
    }

    fn deferred() -> PhysicsError {
        PhysicsError::OutOfEnvelope {
            reason: "Park two-temperature reaction sets are deferred pending verified public coefficients",
        }
    }
}

impl NonequilibriumAir for ParkTwoTemperatureModel {
    fn reaction_rates(&self, _t_tr_k: f64, _t_v_k: f64) -> Result<ReactionRates, PhysicsError> {
        Err(Self::deferred())
    }

    fn species_derivative(
        &self,
        _composition: &AirComposition,
        _rates: &ReactionRates,
        _density_kg_m3: f64,
    ) -> Result<CompositionDerivative, PhysicsError> {
        Err(Self::deferred())
    }

    fn vibrational_relaxation(
        &self,
        _t_tr_k: f64,
        _t_v_k: f64,
        _composition: &AirComposition,
        _density_kg_m3: f64,
    ) -> Result<VibrationalEnergyDerivative, PhysicsError> {
        Err(Self::deferred())
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

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
    fn park87_construction_is_reserved() {
        let model = ParkTwoTemperatureModel::new(
            ParkReactionSet::Park87,
            VibrationalRelaxationModel::MillikanWhitePark,
        );
        assert!(matches!(model, Err(PhysicsError::OutOfEnvelope { .. })));
    }

    #[test]
    fn manual_park_model_queries_fail_closed() {
        let model = ParkTwoTemperatureModel::default();
        assert!(matches!(
            model.reaction_rates(8000.0, 4000.0),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
        let rates = ReactionRates {
            k_forward: [0.0; 5],
            k_backward: [0.0; 5],
        };
        assert!(matches!(
            model.species_derivative(&cold_air(), &rates, 1.0e-3),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            model.vibrational_relaxation(8000.0, 4000.0, &cold_air(), 1.0e-3),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn damkohler_basic_arithmetic_remains_available() {
        let model = ParkTwoTemperatureModel::default();
        let da = model.damkohler(&FlowContext {
            tau_flow_s: 1.0e-3,
            tau_chemistry_s: 1.0e-4,
        });
        assert!((da - 10.0).abs() < 1e-9);
    }
}
