//! Phase-6.2 real-gas thermodynamics.
//!
//! Equilibrium-air thermodynamics for hypersonic flow. For
//! temperatures above ~600 K the ideal-gas `γ = 1.4` assumption
//! breaks down: vibrational excitation, dissociation of O₂ and N₂,
//! and (above ~6000 K) ionisation all shift the effective specific-
//! heat ratio away from the cold-gas value.
//!
//! [`EquilibriumAir`] is the trait the rest of the simulator
//! consumes. [`TannehillEquilibriumAir`] is the in-house 5-species
//! Tannehill / Mugalev curve-fit implementation (Tannehill and
//! Mugalev, *Equilibrium Air Computations*; reproduced in Anderson,
//! *Hypersonic and High-Temperature Gas Dynamics*, 3rd ed., §16).
//!
//! # Scope
//!
//! Ships the 5-species (`N₂`, `O₂`, `N`, `O`, `Ar`) equilibrium-air
//! correlation valid roughly for `100 ≤ T ≤ 15_000 K` and
//! `0.001 ≤ p / p_0 ≤ 10` (with `p_0 = 101 325 Pa`). The 11-species
//! Mugalev correlation (which adds `NO`, `N⁺`, `O⁺`, `NO⁺`, `e⁻`,
//! `N⁺⁺`) is a follow-on slice — exposed here as the
//! [`MugalevEquilibriumAir`] placeholder.
//!
//! # Determinism
//!
//! Pure `f64` arithmetic; piecewise polynomial fits with locked
//! operand order; no FMA. The trait surface is total: callers that
//! query outside the declared validity envelope receive
//! [`PhysicsError::OutOfEnvelope`] from the wrapping
//! [`EquilibriumAir::validate`] guard.

pub mod park_2t;
pub mod tannehill;

pub use park_2t::{
    CompositionDerivative, FlowContext, NonequilibriumAir, ParkReactionSet,
    ParkTwoTemperatureModel, ReactionRates, VibrationalEnergyDerivative,
    VibrationalRelaxationModel,
};
pub use tannehill::TannehillEquilibriumAir;

use crate::error::PhysicsError;

/// Air composition in mole fractions of total number density.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct AirComposition {
    /// Diatomic nitrogen mole fraction.
    pub n2: f64,
    /// Diatomic oxygen mole fraction.
    pub o2: f64,
    /// Atomic nitrogen mole fraction (dissociation product).
    pub n_atomic: f64,
    /// Atomic oxygen mole fraction (dissociation product).
    pub o_atomic: f64,
    /// Nitric oxide mole fraction (intermediate).
    pub no: f64,
    /// Argon mole fraction.
    pub argon: f64,
    /// Electron mole fraction (ionisation, T > ~6000 K).
    pub electrons: f64,
}

impl AirComposition {
    /// Standard cold-air sea-level composition (frozen).
    #[must_use]
    pub const fn sea_level() -> Self {
        Self {
            n2: 0.78084,
            o2: 0.20946,
            n_atomic: 0.0,
            o_atomic: 0.0,
            no: 0.0,
            argon: 0.00934,
            electrons: 0.0,
        }
    }

    /// Sum of mole fractions. Equilibrium-air outputs may carry small
    /// rounding deviations from 1.0; callers can renormalise before
    /// consuming downstream.
    #[must_use]
    pub fn sum(&self) -> f64 {
        self.n2 + self.o2 + self.n_atomic + self.o_atomic + self.no + self.argon + self.electrons
    }
}

/// Effective specific heats and transport returned by
/// [`EquilibriumAir`] evaluations.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct EquilibriumAirState {
    /// Composition (mole fractions).
    pub composition: AirComposition,
    /// Effective ratio of specific heats `γ_eff = Cp / Cv`.
    pub gamma_eff: f64,
    /// Equilibrium speed of sound (m/s).
    pub speed_of_sound_m_s: f64,
    /// Mean molecular weight (kg/mol).
    pub mean_molecular_weight_kg_per_mol: f64,
}

/// Trait for equilibrium-air models.
pub trait EquilibriumAir {
    /// Composition at `(T, p)`.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::OutOfEnvelope`] when the query is
    /// outside the model's documented validity window.
    fn composition(
        &self,
        temperature_k: f64,
        pressure_pa: f64,
    ) -> Result<AirComposition, PhysicsError>;

    /// Effective ratio of specific heats at `(T, p)`.
    ///
    /// # Errors
    ///
    /// Out-of-envelope on bad queries.
    fn gamma_eff(&self, temperature_k: f64, pressure_pa: f64) -> Result<f64, PhysicsError>;

    /// Equilibrium speed of sound (m/s).
    ///
    /// # Errors
    ///
    /// Out-of-envelope on bad queries.
    fn speed_of_sound_m_s(
        &self,
        temperature_k: f64,
        pressure_pa: f64,
    ) -> Result<f64, PhysicsError>;

    /// Compute everything at once for callers that need all three.
    ///
    /// # Errors
    ///
    /// Out-of-envelope on bad queries.
    fn state(
        &self,
        temperature_k: f64,
        pressure_pa: f64,
    ) -> Result<EquilibriumAirState, PhysicsError>;
}

/// 11-species Mugalev equilibrium-air placeholder. Deferred to a
/// follow-on slice; constructing and querying returns
/// `OutOfEnvelope` with a "deferred" reason.
#[derive(Copy, Clone, Debug, Default)]
pub struct MugalevEquilibriumAir;

impl EquilibriumAir for MugalevEquilibriumAir {
    fn composition(&self, _t: f64, _p: f64) -> Result<AirComposition, PhysicsError> {
        Err(PhysicsError::OutOfEnvelope {
            reason: "MugalevEquilibriumAir (11-species) is deferred; use TannehillEquilibriumAir",
        })
    }
    fn gamma_eff(&self, _t: f64, _p: f64) -> Result<f64, PhysicsError> {
        Err(PhysicsError::OutOfEnvelope {
            reason: "MugalevEquilibriumAir (11-species) is deferred; use TannehillEquilibriumAir",
        })
    }
    fn speed_of_sound_m_s(&self, _t: f64, _p: f64) -> Result<f64, PhysicsError> {
        Err(PhysicsError::OutOfEnvelope {
            reason: "MugalevEquilibriumAir (11-species) is deferred; use TannehillEquilibriumAir",
        })
    }
    fn state(&self, _t: f64, _p: f64) -> Result<EquilibriumAirState, PhysicsError> {
        Err(PhysicsError::OutOfEnvelope {
            reason: "MugalevEquilibriumAir (11-species) is deferred; use TannehillEquilibriumAir",
        })
    }
}
