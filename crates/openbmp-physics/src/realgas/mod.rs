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
    ArrheniusForwardCoefficient, CompositionDerivative, FlowContext, MillikanWhitePairCoefficient,
    NonequilibriumAir, PARK87_FORWARD_REACTIONS_AS_PUBLISHED,
    PARK93_FORWARD_REACTIONS_AS_PUBLISHED, ParkAirSpecies, ParkForwardReaction, ParkReactionSet,
    ParkTwoTemperatureModel, ReactionRates, VibrationalEnergyDerivative,
    VibrationalRelaxationModel, park87_millikan_white_pair_coefficients,
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

    /// Sum of neutral heavy-species mole fractions.
    ///
    /// Excludes ions and electrons; useful for checking that a
    /// nominally frozen / neutral composition has not accidentally
    /// populated the reserved ionisation channels.
    #[must_use]
    pub fn neutral_mole_fraction_sum(&self) -> f64 {
        self.n2 + self.o2 + self.n_atomic + self.o_atomic + self.no + self.argon
    }

    /// Sum of singly charged heavy ion mole fractions.
    #[must_use]
    pub fn ion_mole_fraction_sum(&self) -> f64 {
        self.n2_ion + self.o2_ion + self.no_ion + self.n_ion + self.o_ion
    }

    /// Electron-minus-ion mole-fraction residual.
    ///
    /// The Phase-6 ionised-air surface assumes singly charged
    /// positive ions; charge-neutral equilibrium outputs should have
    /// this residual near zero within table / solver tolerance.
    #[must_use]
    pub fn charge_neutrality_residual(&self) -> f64 {
        self.electrons - self.ion_mole_fraction_sum()
    }

    /// Validate finite, non-negative mole fractions and total
    /// normalization.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::NonFinite`] for NaN / Inf entries and
    /// [`PhysicsError::InvalidParameter`] for negative entries, a bad
    /// tolerance, or a total mole fraction outside `1 +/- tolerance`.
    pub fn validate_mole_fractions(&self, tolerance: f64) -> Result<(), PhysicsError> {
        if !tolerance.is_finite() || tolerance < 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "air-composition tolerance must be finite and non-negative",
            });
        }
        for value in [
            self.n2,
            self.o2,
            self.n_atomic,
            self.o_atomic,
            self.no,
            self.argon,
            self.n2_ion,
            self.o2_ion,
            self.no_ion,
            self.n_ion,
            self.o_ion,
            self.electrons,
        ] {
            if !value.is_finite() {
                return Err(PhysicsError::NonFinite {
                    reason: "air-composition mole fraction is NaN or Inf",
                });
            }
            if value < 0.0 {
                return Err(PhysicsError::InvalidParameter {
                    reason: "air-composition mole fractions must be non-negative",
                });
            }
        }
        if (self.sum() - 1.0).abs() > tolerance {
            return Err(PhysicsError::InvalidParameter {
                reason: "air-composition mole fractions do not sum to one",
            });
        }
        Ok(())
    }

    /// Validate charge neutrality for the ionised-air channels.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] when the tolerance
    /// is invalid or when the electron-minus-ion residual exceeds the
    /// supplied tolerance.
    pub fn validate_charge_neutrality(&self, tolerance: f64) -> Result<(), PhysicsError> {
        if !tolerance.is_finite() || tolerance < 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "charge-neutrality tolerance must be finite and non-negative",
            });
        }
        if self.charge_neutrality_residual().abs() > tolerance {
            return Err(PhysicsError::InvalidParameter {
                reason: "air-composition ion/electron fractions are not charge-neutral",
            });
        }
        Ok(())
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

impl MugalevEquilibriumAir {
    /// Smallest temperature `T` (K) intended for the reserved model.
    pub const T_MIN_K: f64 = 100.0;
    /// Largest temperature `T` (K) intended for the reserved model.
    pub const T_MAX_K: f64 = 15_000.0;
    /// Reference pressure `p_0` (Pa) used by the reserved pressure
    /// ratio coordinate.
    pub const P0_PA: f64 = 101_325.0;
    /// Smallest pressure ratio `p/p_0` intended for the reserved model.
    pub const P_RATIO_MIN: f64 = 1.0e-3;
    /// Largest pressure ratio `p/p_0` intended for the reserved model.
    pub const P_RATIO_MAX: f64 = 10.0;

    /// Validate the query envelope without evaluating the reserved
    /// 11-species correlation.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::NonFinite`] for NaN / Inf inputs,
    /// [`PhysicsError::InvalidParameter`] for non-positive pressure,
    /// and [`PhysicsError::OutOfEnvelope`] for values outside the
    /// reserved equilibrium-air table coordinates.
    pub fn validate_query(temperature_k: f64, pressure_pa: f64) -> Result<(), PhysicsError> {
        if !temperature_k.is_finite() || !pressure_pa.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "Mugalev equilibrium-air query contains NaN or Inf",
            });
        }
        if pressure_pa <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "Mugalev equilibrium-air pressure must be positive",
            });
        }
        if !(Self::T_MIN_K..=Self::T_MAX_K).contains(&temperature_k) {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "Mugalev equilibrium-air temperature outside reserved envelope",
            });
        }
        let pressure_ratio = pressure_pa / Self::P0_PA;
        if !(Self::P_RATIO_MIN..=Self::P_RATIO_MAX).contains(&pressure_ratio) {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "Mugalev equilibrium-air pressure ratio outside reserved envelope",
            });
        }
        Ok(())
    }

    fn deferred() -> PhysicsError {
        PhysicsError::OutOfEnvelope {
            reason: "MugalevEquilibriumAir (11-species) is deferred pending verified public coefficients",
        }
    }
}

impl EquilibriumAir for MugalevEquilibriumAir {
    fn composition(
        &self,
        temperature_k: f64,
        pressure_pa: f64,
    ) -> Result<AirComposition, PhysicsError> {
        Self::validate_query(temperature_k, pressure_pa)?;
        Err(Self::deferred())
    }
    fn gamma_eff(&self, temperature_k: f64, pressure_pa: f64) -> Result<f64, PhysicsError> {
        Self::validate_query(temperature_k, pressure_pa)?;
        Err(Self::deferred())
    }
    fn speed_of_sound_m_s(
        &self,
        temperature_k: f64,
        pressure_pa: f64,
    ) -> Result<f64, PhysicsError> {
        Self::validate_query(temperature_k, pressure_pa)?;
        Err(Self::deferred())
    }
    fn state(
        &self,
        temperature_k: f64,
        pressure_pa: f64,
    ) -> Result<EquilibriumAirState, PhysicsError> {
        Self::validate_query(temperature_k, pressure_pa)?;
        Err(Self::deferred())
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn sea_level_composition_validates_with_trace_gas_tolerance() {
        let c = AirComposition::sea_level();
        assert!((c.neutral_mole_fraction_sum() - c.sum()).abs() < 1.0e-15);
        c.validate_mole_fractions(5.0e-4).unwrap();
        c.validate_charge_neutrality(0.0).unwrap();
    }

    #[test]
    fn ionised_composition_checks_charge_neutrality() {
        let c = AirComposition {
            n2: 0.74,
            o2: 0.20,
            n_ion: 0.02,
            o_ion: 0.01,
            electrons: 0.03,
            ..AirComposition::default()
        };
        c.validate_mole_fractions(0.0).unwrap();
        c.validate_charge_neutrality(1.0e-15).unwrap();
        assert_eq!(c.ion_mole_fraction_sum().to_bits(), 0.03_f64.to_bits());
        assert_eq!(c.charge_neutrality_residual().to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn negative_mole_fraction_is_rejected() {
        let c = AirComposition {
            n2: 1.1,
            o2: -0.1,
            ..AirComposition::default()
        };
        assert!(matches!(
            c.validate_mole_fractions(0.0),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn non_normalized_composition_is_rejected() {
        let c = AirComposition {
            n2: 0.5,
            o2: 0.25,
            ..AirComposition::default()
        };
        assert!(matches!(
            c.validate_mole_fractions(1.0e-6),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn mugalev_validates_query_before_deferred_error() {
        let model = MugalevEquilibriumAir;
        assert!(matches!(
            model.state(3000.0, 101_325.0),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            model.gamma_eff(f64::INFINITY, 101_325.0),
            Err(PhysicsError::NonFinite { .. })
        ));
        assert!(matches!(
            model.composition(3000.0, 0.0),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            model.speed_of_sound_m_s(16_000.0, 101_325.0),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn non_neutral_ionised_composition_is_rejected() {
        let c = AirComposition {
            n2: 0.90,
            n_ion: 0.05,
            electrons: 0.04,
            o2: 0.01,
            ..AirComposition::default()
        };
        c.validate_mole_fractions(0.0).unwrap();
        assert!(matches!(
            c.validate_charge_neutrality(1.0e-6),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }
}
