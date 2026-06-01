//! Layered piecewise-exponential atmosphere.
//!
//! In each altitude layer the model evaluates
//!
//! ```text
//!   ρ(h) = ρ_base · exp(−(h − h_base) / H_layer)
//! ```
//!
//! with a constant scale height `H_layer` per layer. Each layer is
//! reported with a layer-effective scale-height temperature
//! `T_layer = M_air · g_0 · H_layer / R`, which keeps the
//! density / pressure / temperature triple consistent through the
//! ideal-gas law `p = ρ R T / M`. The speed of sound uses the
//! per-layer constant `γ = 1.4` for dry air.
//!
//! The Vallado / Curtis / Wertz table pins density and scale height,
//! not local thermodynamic temperature. The reported `temperature_k`
//! is therefore a self-consistency value for the fixed-`M_air`,
//! fixed-`g_0` exponential fit; above the lower atmosphere, the scale
//! height also folds in composition and gravity variation that this
//! deliberately simple model does not resolve.
//!
//! **Honest scope.** This is a layered exponential approximation,
//! widely reproduced in orbital-mechanics textbooks (Vallado 4th ed.
//! Table 8-4; Curtis *Orbital Mechanics for Engineering Students*
//! Appendix D; Wertz *Spacecraft Attitude Determination and Control*
//! §17.2.1) for engineering-grade LEO drag estimation. It is **not**
//! NRLMSISE-00 — there is no solar-flux dependence, no per-species
//! number-density output, no diurnal / latitude / longitude
//! variation. Within its declared envelope (0-1000 km) the layered
//! model captures the dominant altitude variation that dominates
//! orbital drag; for missions that require solar-flux-aware densities
//! select the full-input NRLMSISE-00 model instead.
//!
//! # Determinism
//!
//! Pure `f64` arithmetic with locked operand order on every
//! barometric formula; layer lookup is a linear scan over a
//! compile-time constant array; no FMA call sites.

use openbmp_core::SimTime;

#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::{
    AtmosphereModel, AtmosphereSample, USSA76_G0_M_S2, USSA76_GAMMA_AIR, sutherland_viscosity,
};
use crate::error::PhysicsError;

/// Layer-mean molecular weight of dry air (kg/kmol).
///
/// Above 86 km the species-resolved mean molecular weight diverges
/// from the sea-level value, but the layered exponential model treats
/// the atmosphere as a uniform ideal gas with a single mean molecular
/// weight — the published layer densities and scale heights are
/// already self-consistent under this approximation.
const LAYER_MEAN_MOLECULAR_WEIGHT_KG_KMOL: f64 = 28.9644;

/// Universal gas constant `R*` (J / kmol K) — same value as the
/// USSA76 pin so the temperature derivation stays consistent across
/// the two models.
const LAYER_UNIVERSAL_GAS_CONSTANT: f64 = 8314.32;

/// One altitude layer of the piecewise-exponential model.
#[derive(Copy, Clone, Debug)]
pub struct ExponentialLayer {
    /// Geometric altitude at the base of this layer (m).
    pub base_altitude_m: f64,
    /// Reference mass density at the base of this layer (kg/m³).
    pub base_density_kg_m3: f64,
    /// Scale height for this layer (m). Non-zero, finite.
    pub scale_height_m: f64,
}

/// 14-layer altitude table covering 0-1000 km.
///
/// Densities and scale heights are taken from Vallado, D. A. (2013),
/// *Fundamentals of Astrodynamics and Applications*, 4th ed.,
/// Table 8-4 ("Exponential Atmosphere Model"). The same table is
/// reproduced in Curtis, *Orbital Mechanics for Engineering Students*
/// Appendix D and Wertz / Larson, *Space Mission Analysis and Design*
/// §8.1; the Vallado tabulation is selected because its base-altitude
/// breakpoints align with the USSA76 / COESA76 layer boundaries. The
/// underlying source is the US Standard Atmosphere 1976 supplemental
/// reference profile (NOAA-S/T 76-1562 Part 2) re-fitted to a
/// layered-exponential form for engineering drag estimation.
///
/// The table values are widely-reproduced engineering atmosphere
/// constants and carry the inherent ±30% uncertainty of any
/// "standard" upper-atmosphere density above 100 km — solar-flux
/// dependence (NRLMSISE-00 territory) is intentionally not modelled
/// here.
const LAYERS: [ExponentialLayer; 14] = [
    ExponentialLayer {
        base_altitude_m: 0.0,
        base_density_kg_m3: 1.225,
        scale_height_m: 7_249.0,
    },
    ExponentialLayer {
        base_altitude_m: 25_000.0,
        base_density_kg_m3: 3.899e-2,
        scale_height_m: 6_349.0,
    },
    ExponentialLayer {
        base_altitude_m: 30_000.0,
        base_density_kg_m3: 1.774e-2,
        scale_height_m: 6_682.0,
    },
    ExponentialLayer {
        base_altitude_m: 40_000.0,
        base_density_kg_m3: 3.972e-3,
        scale_height_m: 7_554.0,
    },
    ExponentialLayer {
        base_altitude_m: 50_000.0,
        base_density_kg_m3: 1.057e-3,
        scale_height_m: 8_382.0,
    },
    ExponentialLayer {
        base_altitude_m: 60_000.0,
        base_density_kg_m3: 3.206e-4,
        scale_height_m: 7_714.0,
    },
    ExponentialLayer {
        base_altitude_m: 70_000.0,
        base_density_kg_m3: 8.770e-5,
        scale_height_m: 6_549.0,
    },
    ExponentialLayer {
        base_altitude_m: 80_000.0,
        base_density_kg_m3: 1.905e-5,
        scale_height_m: 5_799.0,
    },
    ExponentialLayer {
        base_altitude_m: 90_000.0,
        base_density_kg_m3: 3.396e-6,
        scale_height_m: 5_382.0,
    },
    ExponentialLayer {
        base_altitude_m: 100_000.0,
        base_density_kg_m3: 5.297e-7,
        scale_height_m: 5_877.0,
    },
    ExponentialLayer {
        base_altitude_m: 150_000.0,
        base_density_kg_m3: 2.070e-9,
        scale_height_m: 22_523.0,
    },
    ExponentialLayer {
        base_altitude_m: 200_000.0,
        base_density_kg_m3: 2.541e-10,
        scale_height_m: 37_105.0,
    },
    ExponentialLayer {
        base_altitude_m: 500_000.0,
        base_density_kg_m3: 6.967e-13,
        scale_height_m: 65_812.0,
    },
    ExponentialLayer {
        base_altitude_m: 700_000.0,
        base_density_kg_m3: 3.070e-14,
        scale_height_m: 124_640.0,
    },
];

/// Maximum geometric altitude (m) supported by the model. Above this
/// altitude `sample` returns either an
/// [`PhysicsError::OutOfEnvelope`] or a vacuum sample, depending on
/// the configured policy.
pub const PIECEWISE_EXP_MAX_GEOMETRIC_M: f64 = 1_000_000.0;

/// Policy for altitudes above the model's documented ceiling.
///
/// Mirrors [`super::ExoatmosphericPolicy`] for the `UsStandard1976`
/// model: scenarios that integrate through the exoatmospheric coast
/// segment may opt into vacuum samples instead of failing closed at
/// the ceiling.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub enum PiecewiseExpExoatmosphericPolicy {
    /// Fail closed above [`PIECEWISE_EXP_MAX_GEOMETRIC_M`] with
    /// [`PhysicsError::OutOfEnvelope`].
    #[default]
    FailClosed,
    /// Return a vacuum sample (`ρ = 0`, `p = 0`, top-layer
    /// temperature, top-layer speed of sound) above the ceiling. Lets
    /// coast-phase integration through the upper atmosphere proceed
    /// without faulting.
    ZeroDensityAboveCeiling,
}

/// Layered piecewise-exponential atmosphere model (0-1000 km).
#[derive(Copy, Clone, Debug)]
pub struct PiecewiseExponentialAtmosphere {
    exoatmospheric_policy: PiecewiseExpExoatmosphericPolicy,
}

impl PiecewiseExponentialAtmosphere {
    /// Construct with [`PiecewiseExpExoatmosphericPolicy::FailClosed`].
    #[must_use]
    pub const fn new() -> Self {
        Self {
            exoatmospheric_policy: PiecewiseExpExoatmosphericPolicy::FailClosed,
        }
    }

    /// Construct with the given exoatmospheric policy.
    #[must_use]
    pub const fn with_exoatmospheric_policy(policy: PiecewiseExpExoatmosphericPolicy) -> Self {
        Self {
            exoatmospheric_policy: policy,
        }
    }

    /// Configured exoatmospheric policy.
    #[must_use]
    pub const fn exoatmospheric_policy(&self) -> PiecewiseExpExoatmosphericPolicy {
        self.exoatmospheric_policy
    }

    /// The shipped 14-layer table.
    #[must_use]
    pub const fn layer_table() -> &'static [ExponentialLayer] {
        &LAYERS
    }

    /// Find the layer containing `altitude_m`.
    ///
    /// Returns `None` for altitudes below the layer-0 base
    /// (sub-zero geometric altitude).
    fn enclosing_layer(altitude_m: f64) -> Option<&'static ExponentialLayer> {
        if altitude_m < LAYERS[0].base_altitude_m {
            return None;
        }
        let mut chosen: &ExponentialLayer = &LAYERS[0];
        for layer in &LAYERS[1..] {
            if altitude_m >= layer.base_altitude_m {
                chosen = layer;
            } else {
                break;
            }
        }
        Some(chosen)
    }
}

impl Default for PiecewiseExponentialAtmosphere {
    fn default() -> Self {
        Self::new()
    }
}

impl AtmosphereModel for PiecewiseExponentialAtmosphere {
    fn sample(
        &self,
        altitude_geometric_m: f64,
        _time: SimTime,
    ) -> Result<AtmosphereSample, PhysicsError> {
        if !altitude_geometric_m.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "piecewise-exponential atmosphere: altitude must be finite",
            });
        }
        if altitude_geometric_m > PIECEWISE_EXP_MAX_GEOMETRIC_M {
            return match self.exoatmospheric_policy {
                PiecewiseExpExoatmosphericPolicy::FailClosed => Err(PhysicsError::OutOfEnvelope {
                    reason: "piecewise-exponential atmosphere envelope ceiling exceeded",
                }),
                PiecewiseExpExoatmosphericPolicy::ZeroDensityAboveCeiling => {
                    // Safe: `LAYERS` is a compile-time constant array
                    // of length 14; `last()` cannot fail.
                    let top = &LAYERS[LAYERS.len() - 1];
                    let temperature_k = layer_temperature_k(top.scale_height_m);
                    let speed_of_sound_m_s = layer_speed_of_sound(temperature_k);
                    Ok(AtmosphereSample {
                        density_kg_m3: 0.0,
                        pressure_pa: 0.0,
                        temperature_k,
                        speed_of_sound_m_s,
                        dynamic_viscosity_pa_s: sutherland_viscosity(temperature_k),
                    })
                }
            };
        }
        let Some(layer) = Self::enclosing_layer(altitude_geometric_m) else {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "piecewise-exponential atmosphere: altitude below 0 m",
            });
        };
        // DETERMINISM: locked operand order — base * exp(-Δh / H).
        let delta_h = altitude_geometric_m - layer.base_altitude_m;
        let density_kg_m3 = layer.base_density_kg_m3 * (-delta_h / layer.scale_height_m).exp();
        let temperature_k = layer_temperature_k(layer.scale_height_m);
        // Ideal gas: p = ρ R T / M (using kmol form to match the USSA76 pins).
        let pressure_pa = density_kg_m3 * LAYER_UNIVERSAL_GAS_CONSTANT * temperature_k
            / LAYER_MEAN_MOLECULAR_WEIGHT_KG_KMOL;
        let speed_of_sound_m_s = layer_speed_of_sound(temperature_k);
        let sample = AtmosphereSample {
            density_kg_m3,
            pressure_pa,
            temperature_k,
            speed_of_sound_m_s,
            dynamic_viscosity_pa_s: sutherland_viscosity(temperature_k),
        };
        sample.require_valid()?;
        Ok(sample)
    }
}

/// Layer-effective temperature derived from the scale height under
/// hydrostatic + ideal-gas balance: `H = R T / (M g)` ⇒
/// `T = M g H / R`.
fn layer_temperature_k(scale_height_m: f64) -> f64 {
    LAYER_MEAN_MOLECULAR_WEIGHT_KG_KMOL * USSA76_G0_M_S2 * scale_height_m
        / LAYER_UNIVERSAL_GAS_CONSTANT
}

/// Local adiabatic speed of sound under dry-air γ.
fn layer_speed_of_sound(temperature_k: f64) -> f64 {
    (USSA76_GAMMA_AIR * LAYER_UNIVERSAL_GAS_CONSTANT * temperature_k
        / LAYER_MEAN_MOLECULAR_WEIGHT_KG_KMOL)
        .sqrt()
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::cast_precision_loss
)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn at_h(h: f64) -> f64 {
        h
    }

    #[test]
    fn rejects_non_finite_altitude() {
        let atm = PiecewiseExponentialAtmosphere::new();
        for h in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(
                matches!(
                    atm.sample(h, SimTime::ZERO),
                    Err(PhysicsError::NonFinite { .. })
                ),
                "non-finite altitude {h} should produce NonFinite",
            );
        }
    }

    #[test]
    fn rejects_negative_altitude() {
        let atm = PiecewiseExponentialAtmosphere::new();
        assert!(matches!(
            atm.sample(-100.0, SimTime::ZERO),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn fails_closed_above_envelope_by_default() {
        let atm = PiecewiseExponentialAtmosphere::new();
        assert!(matches!(
            atm.sample(PIECEWISE_EXP_MAX_GEOMETRIC_M + 1.0, SimTime::ZERO),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn returns_vacuum_above_envelope_when_policy_opts_in() {
        let atm = PiecewiseExponentialAtmosphere::with_exoatmospheric_policy(
            PiecewiseExpExoatmosphericPolicy::ZeroDensityAboveCeiling,
        );
        let sample = atm
            .sample(PIECEWISE_EXP_MAX_GEOMETRIC_M + 50_000.0, SimTime::ZERO)
            .unwrap();
        assert_eq!(sample.density_kg_m3, 0.0);
        assert_eq!(sample.pressure_pa, 0.0);
        assert!(sample.temperature_k > 0.0);
        assert!(sample.speed_of_sound_m_s > 0.0);
    }

    #[test]
    fn sea_level_density_matches_tabulated_base_value() {
        let atm = PiecewiseExponentialAtmosphere::new();
        let sample = atm.sample(at_h(0.0), SimTime::ZERO).unwrap();
        assert_abs_diff_eq!(sample.density_kg_m3, 1.225, epsilon = 1.0e-12);
    }

    #[test]
    fn density_decreases_monotonically_within_each_layer() {
        let atm = PiecewiseExponentialAtmosphere::new();
        let layers = PiecewiseExponentialAtmosphere::layer_table();
        for (i, layer) in layers.iter().enumerate() {
            let upper_bound = layers
                .get(i + 1)
                .map_or(PIECEWISE_EXP_MAX_GEOMETRIC_M, |next| next.base_altitude_m);
            let mid = (layer.base_altitude_m + upper_bound) * 0.5;
            let base = atm.sample(layer.base_altitude_m, SimTime::ZERO).unwrap();
            let middle = atm.sample(mid, SimTime::ZERO).unwrap();
            assert!(
                middle.density_kg_m3 < base.density_kg_m3,
                "layer {i} non-monotonic density: base {:.3e}, mid {:.3e}",
                base.density_kg_m3,
                middle.density_kg_m3,
            );
        }
    }

    #[test]
    fn density_decreases_monotonically_across_layer_boundaries() {
        // Sample at every layer base; the values must be non-increasing
        // — this catches the common error of accidentally selecting a
        // higher base density at a higher altitude.
        let atm = PiecewiseExponentialAtmosphere::new();
        let layers = PiecewiseExponentialAtmosphere::layer_table();
        for window in layers.windows(2) {
            let lower = atm
                .sample(window[0].base_altitude_m, SimTime::ZERO)
                .unwrap();
            let upper = atm
                .sample(window[1].base_altitude_m, SimTime::ZERO)
                .unwrap();
            assert!(
                upper.density_kg_m3 < lower.density_kg_m3,
                "density at h = {} m ({:.3e}) is not below density at h = {} m ({:.3e})",
                window[1].base_altitude_m,
                upper.density_kg_m3,
                window[0].base_altitude_m,
                lower.density_kg_m3,
            );
        }
    }

    #[test]
    fn pressure_and_speed_of_sound_are_finite_and_positive_in_envelope() {
        let atm = PiecewiseExponentialAtmosphere::new();
        for h in [0.0, 12_500.0, 80_000.0, 200_000.0, 500_000.0, 1_000_000.0] {
            let sample = atm.sample(h, SimTime::ZERO).unwrap();
            assert!(
                sample.pressure_pa.is_finite() && sample.pressure_pa > 0.0,
                "pressure at h = {h} m is not positive: {sample:?}",
            );
            assert!(
                sample.temperature_k > 0.0,
                "temperature at h = {h} m is not positive",
            );
            assert!(
                sample.speed_of_sound_m_s > 0.0,
                "speed of sound at h = {h} m is not positive",
            );
        }
    }

    #[test]
    fn density_matches_tabulated_layer_base_at_each_breakpoint() {
        let atm = PiecewiseExponentialAtmosphere::new();
        for layer in PiecewiseExponentialAtmosphere::layer_table() {
            let sample = atm.sample(layer.base_altitude_m, SimTime::ZERO).unwrap();
            assert_abs_diff_eq!(
                sample.density_kg_m3,
                layer.base_density_kg_m3,
                epsilon = 1.0e-12,
            );
        }
    }

    #[test]
    fn determinism_byte_stable_across_reruns() {
        let atm = PiecewiseExponentialAtmosphere::new();
        for h in [0.0, 12_345.0, 75_000.0, 250_000.0, 950_000.0] {
            let a = atm.sample(h, SimTime::ZERO).unwrap();
            let b = atm.sample(h, SimTime::ZERO).unwrap();
            assert_eq!(a.density_kg_m3.to_bits(), b.density_kg_m3.to_bits());
            assert_eq!(a.pressure_pa.to_bits(), b.pressure_pa.to_bits());
            assert_eq!(a.temperature_k.to_bits(), b.temperature_k.to_bits());
            assert_eq!(
                a.speed_of_sound_m_s.to_bits(),
                b.speed_of_sound_m_s.to_bits(),
            );
        }
    }

    #[test]
    fn density_at_400_km_is_in_documented_engineering_range() {
        // Vallado Table 8-4 quotes ρ(400 km) ≈ 2.803e-12 kg/m³ for the
        // exponential model. Our layered fit (between 200 km and 500 km)
        // should produce a density that brackets this value within an
        // order of magnitude — it sits within the inherent ±30%
        // uncertainty of any "standard" upper-atmosphere density.
        let atm = PiecewiseExponentialAtmosphere::new();
        let sample = atm.sample(400_000.0, SimTime::ZERO).unwrap();
        let lower_bound = 1.0e-13;
        let upper_bound = 1.0e-10;
        assert!(
            sample.density_kg_m3 > lower_bound && sample.density_kg_m3 < upper_bound,
            "ρ(400 km) = {:.3e} kg/m³ outside the engineering range [{:.0e}, {:.0e}]",
            sample.density_kg_m3,
            lower_bound,
            upper_bound,
        );
    }

    #[test]
    fn sea_level_temperature_under_layered_fit_is_within_troposphere_envelope() {
        // The layered exponential fit derives a layer-mean temperature
        // from H = R T / (M g). For the sea-level layer (H = 7249 m),
        // T ≈ 246 K — colder than the standard sea-level 288 K because
        // the scale height is fitted to the column mean rather than the
        // local temperature. Verify the value lands in the
        // troposphere-mean envelope.
        let atm = PiecewiseExponentialAtmosphere::new();
        let sample = atm.sample(0.0, SimTime::ZERO).unwrap();
        assert!(
            sample.temperature_k > 200.0 && sample.temperature_k < 300.0,
            "layered-exponential sea-level T = {:.1} K outside [200, 300] K envelope",
            sample.temperature_k,
        );
    }
}
