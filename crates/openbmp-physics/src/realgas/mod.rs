//! Phase-6.2 real-gas thermodynamics.
//!
//! Equilibrium-air thermodynamics for hypersonic flow. For
//! temperatures above ~600 K the ideal-gas `γ = 1.4` assumption
//! breaks down: vibrational excitation, dissociation of O₂ and N₂,
//! and (above ~6000 K) ionisation all shift the effective specific-
//! heat ratio away from the cold-gas value.
//!
//! [`EquilibriumAir`] is the trait the rest of the simulator
//! consumes. [`TannehillEquilibriumAir`] and
//! [`MugalevEquilibriumAir`] are typed-reserved placeholders until
//! verified public coefficient tables or clean-room implementations
//! land with provenance.
//!
//! # Scope
//!
//! The Phase-6 audit removed a synthesized Tannehill table that had
//! not been checked against a public source. Both equilibrium-air
//! implementations now fail closed with [`PhysicsError::OutOfEnvelope`]
//! until verified data are added in a follow-on slice.
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
    ArrheniusForwardCoefficient, CompositionDerivative, FlowContext, NonequilibriumAir,
    PARK87_FORWARD_REACTIONS_AS_PUBLISHED, ParkForwardReaction, ParkReactionSet,
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
    /// Diatomic nitrogen ion mole fraction.
    pub n2_ion: f64,
    /// Diatomic oxygen ion mole fraction.
    pub o2_ion: f64,
    /// Nitric-oxide ion mole fraction.
    pub no_ion: f64,
    /// Atomic nitrogen ion mole fraction.
    pub n_ion: f64,
    /// Atomic oxygen ion mole fraction.
    pub o_ion: f64,
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
            n2_ion: 0.0,
            o2_ion: 0.0,
            no_ion: 0.0,
            n_ion: 0.0,
            o_ion: 0.0,
            electrons: 0.0,
        }
    }

    /// Sum of mole fractions. Equilibrium-air outputs may carry small
    /// rounding deviations from 1.0; callers can renormalise before
    /// consuming downstream.
    #[must_use]
    pub fn sum(&self) -> f64 {
        self.n2
            + self.o2
            + self.n_atomic
            + self.o_atomic
            + self.no
            + self.argon
            + self.n2_ion
            + self.o2_ion
            + self.no_ion
            + self.n_ion
            + self.o_ion
            + self.electrons
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
    fn speed_of_sound_m_s(&self, temperature_k: f64, pressure_pa: f64)
    -> Result<f64, PhysicsError>;

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
/// follow-on slice; querying returns `OutOfEnvelope` with a
/// "deferred" reason.
#[derive(Copy, Clone, Debug, Default)]
pub struct MugalevEquilibriumAir;

impl EquilibriumAir for MugalevEquilibriumAir {
    fn composition(&self, _t: f64, _p: f64) -> Result<AirComposition, PhysicsError> {
        Err(PhysicsError::OutOfEnvelope {
            reason: "MugalevEquilibriumAir (11-species) is deferred pending verified public coefficients",
        })
    }
    fn gamma_eff(&self, _t: f64, _p: f64) -> Result<f64, PhysicsError> {
        Err(PhysicsError::OutOfEnvelope {
            reason: "MugalevEquilibriumAir (11-species) is deferred pending verified public coefficients",
        })
    }
    fn speed_of_sound_m_s(&self, _t: f64, _p: f64) -> Result<f64, PhysicsError> {
        Err(PhysicsError::OutOfEnvelope {
            reason: "MugalevEquilibriumAir (11-species) is deferred pending verified public coefficients",
        })
    }
    fn state(&self, _t: f64, _p: f64) -> Result<EquilibriumAirState, PhysicsError> {
        Err(PhysicsError::OutOfEnvelope {
            reason: "MugalevEquilibriumAir (11-species) is deferred pending verified public coefficients",
        })
    }
}
