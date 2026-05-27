//! Atmosphere models.
//!
//! Provides:
//!
//! * [`IsothermalAtmosphere`] — toy: returns
//!   constant `(ρ, p, T, a)` at every query. Useful for unit-test
//!   fixtures and for analytic-toy scenarios that need a non-vacuum
//!   atmosphere without a layered model.
//! * [`UsStandard1976`] — in-house Rust port of
//!   NOAA-S/T 76-1562 / NASA-TM-X-74335. Geopotential layers 0–86 km
//!   only.
//! * [`PiecewiseExponentialAtmosphere`] — 14-layer engineering
//!   exponential atmosphere covering 0-1000 km, sourced from Vallado
//!   *Fundamentals of Astrodynamics and Applications* 4th ed. Table
//!   8-4. Honest scope: this is **not** NRLMSISE-00 — no solar-flux
//!   dependence, no per-species number densities. Captures the
//!   altitude-dominant variation that determines orbital drag for
//!   engineering analyses.
//! * [`Nrlmsise00Static`] and [`Nrlmsise00Full`] — NRLMSISE-00
//!   static-default and full-input coefficient paths covering
//!   0-1000 km.
//!
//! Determinism: pure arithmetic on `f64`; locked operand order on
//! barometric formulas; no FMA. No wall-clock time, no system RNG,
//! no network, no runtime file I/O.
//!
//! Layering: atmosphere models depend only on `openbmp-core` (L0)
//! and `openbmp-physics::error`. The `AtmosphereSample` type lives here
//! (in `openbmp-physics`) rather than in simulator crates so controller
//! and simulator consumers share one HAL-portable atmosphere surface.

pub mod isothermal;
pub mod nrlmsise00;
pub mod piecewise_exponential;
pub mod us_standard_1976;

pub use isothermal::IsothermalAtmosphere;
pub use nrlmsise00::{Nrlmsise00Full, Nrlmsise00Inputs, Nrlmsise00Outputs, Nrlmsise00Static};
pub use piecewise_exponential::{
    ExponentialLayer, PIECEWISE_EXP_MAX_GEOMETRIC_M, PiecewiseExpExoatmosphericPolicy,
    PiecewiseExponentialAtmosphere,
};
pub use us_standard_1976::{ExoatmosphericPolicy, UsStandard1976};

use openbmp_core::SimTime;

use crate::error::PhysicsError;

// ---------------------------------------------------------------------
// USSA76 defining constants (NOAA-S/T 76-1562 §1.2-§1.3)
// ---------------------------------------------------------------------

/// Standard acceleration of gravity (m/s² per geopotential metre).
pub const USSA76_G0_M_S2: f64 = crate::gravity::STANDARD_GRAVITY_M_S2;

/// USSA76 universal gas constant `R*`, in J / (kmol · K).
pub const USSA76_UNIVERSAL_GAS_CONSTANT: f64 = 8_314.32;

/// Modern CODATA ideal-gas constant in J / (mol · K), used by
/// controller-side formulas that operate in mol units.
pub const IDEAL_GAS_CONSTANT_J_MOL_K: f64 = 8.314_462_618;

/// Mean molecular weight of dry air at sea level, kg/kmol.
pub const USSA76_MOLAR_MASS_AIR_KG_KMOL: f64 = 28.9644;

/// Same value expressed in kg/mol.
pub const USSA76_MOLAR_MASS_AIR_KG_MOL: f64 = USSA76_MOLAR_MASS_AIR_KG_KMOL * 1.0e-3;

/// Ratio of specific heats for dry air (`Cp / Cv`).
pub const USSA76_GAMMA_AIR: f64 = 1.40;

/// Sea-level standard temperature (K).
pub const USSA76_SEA_LEVEL_TEMPERATURE_K: f64 = 288.15;

/// Sea-level static pressure (Pa).
pub const USSA76_SEA_LEVEL_PRESSURE_PA: f64 = 101_325.0;

/// Sea-level standard mass density of dry air (kg/m³).
///
/// Derived from the ideal-gas law at sea-level pressure and
/// temperature with the USSA76 mean molecular weight; matches the
/// canonical 1.225 kg/m³ value cited in the standard.
pub const USSA76_SEA_LEVEL_DENSITY_KG_M3: f64 = 1.225;

/// Troposphere lapse rate (K / m).
pub const USSA76_TROPOSPHERE_LAPSE_RATE_K_PER_M: f64 = -6.5e-3;

/// Tropopause geopotential altitude (m).
pub const USSA76_TROPOPAUSE_GEOPOTENTIAL_M: f64 = 11_000.0;

/// Static pressure at the 11 km tropopause (Pa), derived from the
/// locked USSA76 recurrence.
pub const USSA76_TROPOPAUSE_PRESSURE_PA: f64 = 22_632.063_973_462_91;

/// USSA76 effective Earth radius (m) used in geopotential / geometric
/// conversion.
pub const USSA76_REFERENCE_RADIUS_M: f64 = 6_356_766.0;

/// Top of the 7-layer USSA76 model in geopotential metres.
pub const USSA76_MAX_GEOPOTENTIAL_M: f64 = 84_852.0;

/// Top of the 7-layer USSA76 model in geometric metres.
pub const USSA76_MAX_GEOMETRIC_M: f64 = 86_000.0;

/// Convert geometric altitude `z` (m) to geopotential altitude `h'`
/// (m') using the USSA76 effective radius.
#[must_use]
pub fn geopotential_from_geometric(z_geometric_m: f64) -> f64 {
    let r = USSA76_REFERENCE_RADIUS_M;
    r * z_geometric_m / (r + z_geometric_m)
}

/// Convert geopotential altitude `h'` (m') to geometric altitude `z`
/// (m).
#[must_use]
pub fn geometric_from_geopotential(h_geopotential_m: f64) -> f64 {
    let r = USSA76_REFERENCE_RADIUS_M;
    r * h_geopotential_m / (r - h_geopotential_m)
}

/// Invert the USSA76 troposphere pressure relation to recover
/// geopotential altitude (m) from static pressure (Pa).
///
/// This helper is valid for the 0..11 km troposphere segment and
/// clamps lower pressures to the tropopause altitude rather than
/// extrapolating through layers the caller has not modelled.
#[must_use]
pub fn pressure_altitude_troposphere_m(pressure_pa: f64) -> f64 {
    if pressure_pa <= 0.0 || pressure_pa < USSA76_TROPOPAUSE_PRESSURE_PA {
        return USSA76_TROPOPAUSE_GEOPOTENTIAL_M;
    }
    let exponent_recip = -USSA76_UNIVERSAL_GAS_CONSTANT * USSA76_TROPOSPHERE_LAPSE_RATE_K_PER_M
        / (USSA76_G0_M_S2 * USSA76_MOLAR_MASS_AIR_KG_KMOL);
    let ratio = pressure_pa / USSA76_SEA_LEVEL_PRESSURE_PA;
    (USSA76_SEA_LEVEL_TEMPERATURE_K / USSA76_TROPOSPHERE_LAPSE_RATE_K_PER_M)
        * (ratio.powf(exponent_recip) - 1.0)
}

/// Closed-form dynamic pressure `q = ½ · ρ · v²` (Pa) for given mass
/// density (kg/m³) and total airspeed magnitude (m/s).
///
/// Both inputs must be non-negative for a physically meaningful
/// result; this helper does no clamping, mirroring the project
/// convention that math primitives stay total and let callers
/// validate inputs.
#[must_use]
pub fn dynamic_pressure_pa(density_kg_m3: f64, velocity_m_s: f64) -> f64 {
    0.5 * density_kg_m3 * velocity_m_s * velocity_m_s
}

/// Atmosphere sample in SI units.
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
    /// Returns [`PhysicsError::NonFinite`] if any input is `NaN` or `Inf`.
    /// Returns [`PhysicsError::InvalidParameter`] if temperature, density,
    /// pressure, or speed of sound is negative.
    pub fn new(
        density_kg_m3: f64,
        pressure_pa: f64,
        temperature_k: f64,
        speed_of_sound_m_s: f64,
    ) -> Result<Self, PhysicsError> {
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
    /// Returns [`PhysicsError::NonFinite`] for `NaN`/`Inf` components and
    /// [`PhysicsError::InvalidParameter`] for negative scalars. Zero
    /// density / pressure are accepted (vacuum case).
    pub fn require_valid(&self) -> Result<(), PhysicsError> {
        for v in [
            self.density_kg_m3,
            self.pressure_pa,
            self.temperature_k,
            self.speed_of_sound_m_s,
        ] {
            if !v.is_finite() {
                return Err(PhysicsError::NonFinite {
                    reason: "atmosphere sample component is NaN or infinite",
                });
            }
            if v < 0.0 {
                return Err(PhysicsError::InvalidParameter {
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
/// models (none currently).
pub trait AtmosphereModel {
    /// Sample the atmosphere at a geometric altitude (m).
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::OutOfEnvelope`] when the altitude is
    /// outside the model's declared validity range, or
    /// [`PhysicsError::NonFinite`] when an arithmetic step produces
    /// `NaN` / `Inf`.
    fn sample(
        &self,
        altitude_geometric_m: f64,
        time: SimTime,
    ) -> Result<AtmosphereSample, PhysicsError>;
}
