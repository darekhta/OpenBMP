//! Isothermal atmosphere — toy.
//!
//! Stores a constant `(ρ, p, T)` and the corresponding speed of sound,
//! returning them at every query. Useful for unit-test fixtures and
//! analytic-toy scenarios that need a non-vacuum atmosphere without a
//! layered model.
//!
//! Speed of sound is supplied by the constructor (caller-controlled).
//! For physical consistency a typical caller would compute
//! `a = √(γ · R · T / M)` once with the gas constants of interest and
//! pass the result; the model itself does no checking that
//! `(ρ, p, T, a)` are mutually consistent.

use openbmp_core::SimTime;

use super::{AtmosphereModel, AtmosphereSample};
use crate::error::PhysicsError;

/// Constant-everywhere atmosphere.
///
/// The four constructor inputs are validated through
/// [`AtmosphereSample::new`] so a caller can't construct an
/// `IsothermalAtmosphere` whose sample is non-finite or negative.
#[derive(Copy, Clone, Debug)]
pub struct IsothermalAtmosphere {
    sample: AtmosphereSample,
}

impl IsothermalAtmosphere {
    /// Construct from explicit `(ρ, p, T, a)` scalars.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::NonFinite`] or
    /// [`PhysicsError::InvalidParameter`] via [`AtmosphereSample::new`].
    pub fn new(
        density_kg_m3: f64,
        pressure_pa: f64,
        temperature_k: f64,
        speed_of_sound_m_s: f64,
    ) -> Result<Self, PhysicsError> {
        Ok(Self {
            sample: AtmosphereSample::new(
                density_kg_m3,
                pressure_pa,
                temperature_k,
                speed_of_sound_m_s,
            )?,
        })
    }

    /// Sea-level US Standard 1976 reference values, useful for tests:
    /// `ρ = 1.2250 kg/m³`, `p = 101325 Pa`, `T = 288.15 K`,
    /// `a = 340.294 m/s`. The sample is **not** required to be
    /// physically self-consistent; this constructor returns the
    /// canonical sea-level numbers from NOAA-S/T 76-1562 table 1.
    ///
    /// All four values are positive and finite, so the
    /// [`AtmosphereSample::new`] validation never returns an error
    /// here. The `expect` below cannot panic on the pinned constants.
    #[must_use]
    #[allow(clippy::expect_used, clippy::missing_panics_doc)]
    pub fn ussa_sea_level() -> Self {
        Self::new(1.225_000, 101_325.0, 288.150, 340.294)
            .expect("USSA76 sea-level sample is well-formed by construction")
    }

    /// The configured sample (returned at every query).
    #[must_use]
    pub const fn sample_value(&self) -> AtmosphereSample {
        self.sample
    }
}

impl AtmosphereModel for IsothermalAtmosphere {
    fn sample(
        &self,
        _altitude_geometric_m: f64,
        _time: SimTime,
    ) -> Result<AtmosphereSample, PhysicsError> {
        Ok(self.sample)
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn returns_configured_sample_at_any_altitude() {
        let atm = IsothermalAtmosphere::new(1.225, 101_325.0, 288.15, 340.3).unwrap();
        for z in [0.0, 1_000.0, 10_000.0, 80_000.0, 1_000_000.0] {
            let s = atm.sample(z, SimTime::ZERO).unwrap();
            assert_abs_diff_eq!(s.density_kg_m3, 1.225);
            assert_abs_diff_eq!(s.pressure_pa, 101_325.0);
            assert_abs_diff_eq!(s.temperature_k, 288.15);
            assert_abs_diff_eq!(s.speed_of_sound_m_s, 340.3);
        }
    }

    #[test]
    fn returns_configured_sample_at_any_time() {
        let atm = IsothermalAtmosphere::ussa_sea_level();
        for t in [0.0, 1.0, 1_000.0, 86_400.0] {
            let s = atm.sample(0.0, SimTime::from_seconds(t)).unwrap();
            assert_abs_diff_eq!(s.density_kg_m3, 1.225);
            assert_abs_diff_eq!(s.pressure_pa, 101_325.0);
            assert_abs_diff_eq!(s.temperature_k, 288.15);
        }
    }

    #[test]
    fn rejects_non_finite_inputs() {
        assert!(IsothermalAtmosphere::new(f64::NAN, 0.0, 0.0, 0.0).is_err());
        assert!(IsothermalAtmosphere::new(0.0, f64::INFINITY, 0.0, 0.0).is_err());
        assert!(IsothermalAtmosphere::new(0.0, 0.0, f64::NAN, 0.0).is_err());
        assert!(IsothermalAtmosphere::new(0.0, 0.0, 0.0, f64::NAN).is_err());
    }

    #[test]
    fn rejects_negative_inputs() {
        assert!(IsothermalAtmosphere::new(-1.0, 0.0, 0.0, 0.0).is_err());
        assert!(IsothermalAtmosphere::new(0.0, -1.0, 0.0, 0.0).is_err());
        assert!(IsothermalAtmosphere::new(0.0, 0.0, -1.0, 0.0).is_err());
        assert!(IsothermalAtmosphere::new(0.0, 0.0, 0.0, -1.0).is_err());
    }

    #[test]
    fn accepts_zero_components_for_vacuum_case() {
        let atm = IsothermalAtmosphere::new(0.0, 0.0, 0.0, 0.0).unwrap();
        let s = atm.sample(0.0, SimTime::ZERO).unwrap();
        assert_abs_diff_eq!(s.density_kg_m3, 0.0);
    }
}
