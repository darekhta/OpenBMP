//! Deferred Tannehill / Mugalev 5-species equilibrium-air surface.
//!
//! This module originally carried a hand-synthesised
//! `(T, p)` grid that was shaped like the expected Tannehill /
//! Mugalev equilibrium-air trends but was not verified against a
//! public coefficient table or a citable reproduced table. That is
//! not acceptable for a model named after a published correlation.
//!
//! # Honest Scope
//!
//! [`TannehillEquilibriumAir`] is therefore a typed-reserved model:
//! the public trait surface exists, but every query fails closed with
//! [`PhysicsError::OutOfEnvelope`] until a follow-on slice imports a
//! verified public table or a clean-room implementation of the
//! published correlation with provenance. This avoids shipping a
//! plausible-looking real-gas table as research-grade data.

use super::{AirComposition, EquilibriumAir, EquilibriumAirState};
use crate::error::PhysicsError;

/// Tannehill 5-species equilibrium-air model.
///
/// Reserved pending verified public coefficients / table values.
#[derive(Copy, Clone, Debug, Default)]
pub struct TannehillEquilibriumAir;

impl TannehillEquilibriumAir {
    /// Smallest temperature `T` (K) intended for the reserved model.
    pub const T_MIN_K: f64 = 100.0;
    /// Largest temperature `T` (K) intended for the reserved model.
    pub const T_MAX_K: f64 = 15_000.0;
    /// Reference pressure `p_0` (Pa) used by the Tannehill table
    /// pressure-ratio coordinate.
    pub const P0_PA: f64 = 101_325.0;
    /// Smallest pressure ratio `p/p_0` intended for the reserved model.
    pub const P_RATIO_MIN: f64 = 1.0e-3;
    /// Largest pressure ratio `p/p_0` intended for the reserved model.
    pub const P_RATIO_MAX: f64 = 10.0;

    /// Validate the query envelope without evaluating the reserved
    /// correlation.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::NonFinite`] for NaN / Inf inputs,
    /// [`PhysicsError::InvalidParameter`] for non-positive pressure,
    /// and [`PhysicsError::OutOfEnvelope`] for values outside the
    /// intended Tannehill table coordinates.
    pub fn validate_query(temperature_k: f64, pressure_pa: f64) -> Result<(), PhysicsError> {
        if !temperature_k.is_finite() || !pressure_pa.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "Tannehill equilibrium-air query contains NaN or Inf",
            });
        }
        if pressure_pa <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "Tannehill equilibrium-air pressure must be positive",
            });
        }
        if !(Self::T_MIN_K..=Self::T_MAX_K).contains(&temperature_k) {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "Tannehill equilibrium-air temperature outside reserved envelope",
            });
        }
        let pressure_ratio = pressure_pa / Self::P0_PA;
        if !(Self::P_RATIO_MIN..=Self::P_RATIO_MAX).contains(&pressure_ratio) {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "Tannehill equilibrium-air pressure ratio outside reserved envelope",
            });
        }
        Ok(())
    }

    fn deferred() -> PhysicsError {
        PhysicsError::OutOfEnvelope {
            reason: "TannehillEquilibriumAir is deferred pending verified public table values",
        }
    }
}

impl EquilibriumAir for TannehillEquilibriumAir {
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
    fn tannehill_queries_fail_closed_until_verified_table_lands() {
        let model = TannehillEquilibriumAir;
        assert!(matches!(
            model.gamma_eff(3000.0, 101_325.0),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            model.composition(3000.0, 101_325.0),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            model.speed_of_sound_m_s(3000.0, 101_325.0),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            model.state(3000.0, 101_325.0),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn tannehill_validates_query_before_deferred_error() {
        let model = TannehillEquilibriumAir;
        assert!(matches!(
            model.gamma_eff(f64::NAN, 101_325.0),
            Err(PhysicsError::NonFinite { .. })
        ));
        assert!(matches!(
            model.gamma_eff(3000.0, -1.0),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            model.gamma_eff(99.0, 101_325.0),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            model.gamma_eff(3000.0, 101_325.0 * 11.0),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
    }
}
