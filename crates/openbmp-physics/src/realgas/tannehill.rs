//! Deferred Tannehill / Mugalev 5-species equilibrium-air surface.
//!
//! Phase 6 originally landed this module with a hand-synthesised
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
    /// Smallest pressure ratio `p/p_0` intended for the reserved model.
    pub const P_RATIO_MIN: f64 = 1.0e-3;
    /// Largest pressure ratio `p/p_0` intended for the reserved model.
    pub const P_RATIO_MAX: f64 = 10.0;

    fn deferred() -> PhysicsError {
        PhysicsError::OutOfEnvelope {
            reason: "TannehillEquilibriumAir is deferred pending verified public table values",
        }
    }
}

impl EquilibriumAir for TannehillEquilibriumAir {
    fn composition(
        &self,
        _temperature_k: f64,
        _pressure_pa: f64,
    ) -> Result<AirComposition, PhysicsError> {
        Err(Self::deferred())
    }

    fn gamma_eff(&self, _temperature_k: f64, _pressure_pa: f64) -> Result<f64, PhysicsError> {
        Err(Self::deferred())
    }

    fn speed_of_sound_m_s(
        &self,
        _temperature_k: f64,
        _pressure_pa: f64,
    ) -> Result<f64, PhysicsError> {
        Err(Self::deferred())
    }

    fn state(
        &self,
        _temperature_k: f64,
        _pressure_pa: f64,
    ) -> Result<EquilibriumAirState, PhysicsError> {
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
}
