//! Atmosphere models.
//!
//! Phase 2.3 ships:
//!
//! * [`IsothermalAtmosphere`] (sub-phase 2.3.A) — toy: returns
//!   constant `(ρ, p, T, a)` at every query. Useful for unit-test
//!   fixtures and for analytic-toy scenarios that need a non-vacuum
//!   atmosphere without a layered model.
//! * [`UsStandard1976`] (sub-phase 2.3.B) — in-house Rust port of
//!   NOAA-S/T 76-1562 / NASA-TM-X-74335. Geopotential layers 0–86 km
//!   only; 86 km+ is a Phase-6 extension.
//!
//! Determinism: pure arithmetic on `f64`; locked operand order on
//! barometric formulas; no FMA. No wall-clock time, no system RNG,
//! no network, no file I/O.
//!
//! Layering: atmosphere models depend only on `openbmp-core` (L0)
//! and `openbmp-env::error`. The `AtmosphereSample` type lives here
//! (in `openbmp-env`) rather than in `openbmp-sim` so the env crate
//! stays self-contained per the Phase-2 plan's L1/L2 layering rule.

pub mod isothermal;
pub mod us_standard_1976;

pub use isothermal::IsothermalAtmosphere;
pub use us_standard_1976::{
    ExoatmosphericPolicy, USSA76_GAMMA_AIR, USSA76_MAX_GEOMETRIC_M, USSA76_MAX_GEOPOTENTIAL_M,
    USSA76_MOLAR_MASS_AIR_KG_MOL, USSA76_REFERENCE_RADIUS_M, USSA76_UNIVERSAL_GAS_CONSTANT,
    UsStandard1976,
};

use openbmp_core::SimTime;

use crate::error::EnvError;

/// Atmospheric state at a single altitude / time.
///
/// All fields are raw `f64` in SI units, named with their unit
/// suffix (matching the project's `_m_s2` / `_kg_m3` convention from
/// gravity and the kernel boundary). The kernel side may re-wrap
/// these into typed `uom` quantities at the trait boundary; the hot
/// path stays unwrapped.
///
/// "Density" is the bulk mass density of the atmospheric mixture.
/// "Speed of sound" is the local adiabatic value
/// `a = √(γ · R · T / M)` for the constant `γ`, `R`, `M` declared by
/// the model.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct AtmosphereSample {
    /// Mass density (kg/m³).
    pub density_kg_m3: f64,
    /// Static pressure (Pa).
    pub pressure_pa: f64,
    /// Static temperature (K).
    pub temperature_k: f64,
    /// Local adiabatic speed of sound (m/s).
    pub speed_of_sound_m_s: f64,
}

impl AtmosphereSample {
    /// Construct from explicit scalar components, validating finiteness
    /// and physical-positivity invariants.
    ///
    /// # Errors
    ///
    /// Returns [`EnvError::NonFinite`] if any input is `NaN` or `Inf`.
    /// Returns [`EnvError::InvalidParameter`] if temperature, density,
    /// pressure, or speed of sound is negative.
    pub fn new(
        density_kg_m3: f64,
        pressure_pa: f64,
        temperature_k: f64,
        speed_of_sound_m_s: f64,
    ) -> Result<Self, EnvError> {
        let s = Self {
            density_kg_m3,
            pressure_pa,
            temperature_k,
            speed_of_sound_m_s,
        };
        s.require_valid()?;
        Ok(s)
    }

    /// Validate finiteness and physical-positivity invariants.
    ///
    /// # Errors
    ///
    /// Returns [`EnvError::NonFinite`] for `NaN`/`Inf` components and
    /// [`EnvError::InvalidParameter`] for negative scalars. Zero
    /// density / pressure are accepted (vacuum case).
    pub fn require_valid(&self) -> Result<(), EnvError> {
        for v in [
            self.density_kg_m3,
            self.pressure_pa,
            self.temperature_k,
            self.speed_of_sound_m_s,
        ] {
            if !v.is_finite() {
                return Err(EnvError::NonFinite {
                    reason: "atmosphere sample component is NaN or infinite",
                });
            }
            if v < 0.0 {
                return Err(EnvError::InvalidParameter {
                    reason: "atmosphere sample component must be non-negative",
                });
            }
        }
        Ok(())
    }
}

/// Trait implemented by atmosphere-providing environment models.
///
/// Models take **geometric altitude** (m above the model's reference
/// surface; for `UsStandard1976` this is the WGS84 / USSA76 reference
/// radius). Geopotential conversion happens inside the model. Time
/// is included for forward compatibility with future time-varying
/// models (none in Phase 2).
pub trait AtmosphereModel {
    /// Sample the atmosphere at a geometric altitude (m).
    ///
    /// # Errors
    ///
    /// Returns [`EnvError::OutOfEnvelope`] when the altitude is
    /// outside the model's declared validity range, or
    /// [`EnvError::NonFinite`] when an arithmetic step produces
    /// `NaN` / `Inf`.
    fn sample(
        &self,
        altitude_geometric_m: f64,
        time: SimTime,
    ) -> Result<AtmosphereSample, EnvError>;
}
