//! US Standard Atmosphere 1976 — geopotential layers 0–86 km.
//!
//! In-house Rust port of the model from
//! NOAA-S/T 76-1562 / NASA-TM-X-74335 (NTRS 19770009539). Covers the
//! seven lower atmospheric layers (geopotential 0 m through 84852 m,
//! corresponding to geometric 86 km). Above 86 km the model returns
//! [`PhysicsError::OutOfEnvelope`] by default; opt-in
//! [`ExoatmosphericPolicy::ZeroDensityAboveCeiling`] returns a
//! `(ρ ≈ 0, p ≈ 0, T = ceiling, a = ceiling)` sample so coast-phase
//! integration through the upper atmosphere doesn't fault.
//!
//! # Reference
//!
//! Defining constants and layer structure per NOAA-S/T 76-1562
//! §1.2-§1.3. The base-pressure column is the locked f64 recurrence
//! from the sea-level pressure through the standard barometric
//! formulas. Regression tests verify the in-source pin
//! against `data/atmosphere/us_standard_1976.toml` and check
//! per-kilometre samples against a TOML-derived reference.
//!
//! # Determinism
//!
//! Pure arithmetic on `f64`; locked operand order on every
//! barometric formula; no fused multiply-add call sites. The two layer
//! formulas (gradient `L ≠ 0` vs. isothermal `L = 0`) are split into
//! distinct branches; both produce bit-stable output on the
//! reference platform profile.
//!
//! Upper-atmosphere density is covered by the separate NRLMSISE-00
//! model. USSA76 is the academic baseline for
//! troposphere / lower-stratosphere sounding-rocket scenarios.

use openbmp_core::SimTime;

use super::{
    AtmosphereModel, AtmosphereSample, USSA76_G0_M_S2, USSA76_GAMMA_AIR, USSA76_MAX_GEOMETRIC_M,
    USSA76_MAX_GEOPOTENTIAL_M, USSA76_MOLAR_MASS_AIR_KG_KMOL, USSA76_SEA_LEVEL_PRESSURE_PA,
    USSA76_SEA_LEVEL_TEMPERATURE_K, USSA76_TROPOPAUSE_GEOPOTENTIAL_M,
    USSA76_TROPOPAUSE_PRESSURE_PA, USSA76_TROPOSPHERE_LAPSE_RATE_K_PER_M,
    USSA76_UNIVERSAL_GAS_CONSTANT, geopotential_from_geometric,
};
use crate::error::PhysicsError;

// ---------------------------------------------------------------------
// Layer table — NOAA-S/T 76-1562 table 4
// ---------------------------------------------------------------------

/// One USSA76 atmospheric layer.
#[derive(Copy, Clone, Debug)]
struct Layer {
    /// Geopotential altitude at the base of this layer (m').
    base_geopotential_m: f64,
    /// Static temperature at the base of this layer (K).
    base_temperature_k: f64,
    /// Lapse rate within this layer (K / m'). Zero for isothermal
    /// layers.
    lapse_rate_k_per_m: f64,
    /// Static pressure at the base of this layer (Pa).
    base_pressure_pa: f64,
}

/// USSA76 layer table per NOAA-S/T 76-1562 §1.2 (table 4).
///
/// The base-pressure column is derived recursively from the layer-0
/// sea-level value by chaining the barometric formulas across each
/// previous layer using the locked operand order in this module. This
/// keeps pressure continuous at layer bases and avoids mixing rounded
/// table-display values with the runtime recurrence.
const LAYERS: [Layer; 7] = [
    Layer {
        base_geopotential_m: 0.0,
        base_temperature_k: USSA76_SEA_LEVEL_TEMPERATURE_K,
        lapse_rate_k_per_m: USSA76_TROPOSPHERE_LAPSE_RATE_K_PER_M,
        base_pressure_pa: USSA76_SEA_LEVEL_PRESSURE_PA,
    },
    Layer {
        base_geopotential_m: USSA76_TROPOPAUSE_GEOPOTENTIAL_M,
        base_temperature_k: 216.65,
        lapse_rate_k_per_m: 0.0,
        base_pressure_pa: USSA76_TROPOPAUSE_PRESSURE_PA,
    },
    Layer {
        base_geopotential_m: 20_000.0,
        base_temperature_k: 216.65,
        lapse_rate_k_per_m: 1.0e-3,
        base_pressure_pa: 5_474.888_669_677_775,
    },
    Layer {
        base_geopotential_m: 32_000.0,
        base_temperature_k: 228.65,
        lapse_rate_k_per_m: 2.8e-3,
        base_pressure_pa: 868.018_684_755_228_2,
    },
    Layer {
        base_geopotential_m: 47_000.0,
        base_temperature_k: 270.65,
        lapse_rate_k_per_m: 0.0,
        base_pressure_pa: 110.906_305_554_966_11,
    },
    Layer {
        base_geopotential_m: 51_000.0,
        base_temperature_k: 270.65,
        lapse_rate_k_per_m: -2.8e-3,
        base_pressure_pa: 66.938_873_118_687_4,
    },
    Layer {
        base_geopotential_m: 71_000.0,
        base_temperature_k: 214.65,
        lapse_rate_k_per_m: -2.0e-3,
        base_pressure_pa: 3.956_420_428_040_732_7,
    },
];

// ---------------------------------------------------------------------
// Exoatmospheric policy
// ---------------------------------------------------------------------

/// What the model does for queries above its 86 km geometric ceiling.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub enum ExoatmosphericPolicy {
    /// Default. Returns [`PhysicsError::OutOfEnvelope`] when the geometric
    /// altitude exceeds [`USSA76_MAX_GEOMETRIC_M`].
    #[default]
    FailClosed,
    /// Returns a vacuum sample (`ρ = 0`, `p = 0`, `T` clamped to the
    /// ceiling temperature, `a` clamped to the ceiling speed of
    /// sound) for any altitude above the ceiling. Useful for coast
    /// phases through the upper atmosphere where the kernel needs a
    /// finite answer rather than a halt.
    ZeroDensityAboveCeiling,
}

// ---------------------------------------------------------------------
// UsStandard1976
// ---------------------------------------------------------------------

/// US Standard Atmosphere 1976 — geopotential layers 0–86 km.
#[derive(Copy, Clone, Debug, Default)]
pub struct UsStandard1976 {
    exoatmospheric_policy: ExoatmosphericPolicy,
}

impl UsStandard1976 {
    /// Default constructor — fail-closed above 86 km.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            exoatmospheric_policy: ExoatmosphericPolicy::FailClosed,
        }
    }

    /// Construct with the given exoatmospheric policy.
    #[must_use]
    pub const fn with_exoatmospheric_policy(policy: ExoatmosphericPolicy) -> Self {
        Self {
            exoatmospheric_policy: policy,
        }
    }

    /// Active policy.
    #[must_use]
    pub const fn exoatmospheric_policy(&self) -> ExoatmosphericPolicy {
        self.exoatmospheric_policy
    }

    /// Sample at a geopotential altitude (m'). The public surface
    /// takes geometric altitude through [`AtmosphereModel::sample`];
    /// this entry point is exposed for callers that already have
    /// geopotential altitude (e.g., the regression test against the
    /// data-pin reference) and the layer-by-layer pin verification.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::OutOfEnvelope`] for `h' < 0` or
    /// `h' > USSA76_MAX_GEOPOTENTIAL_M` (subject to the active
    /// exoatmospheric policy for the upper bound).
    pub fn sample_at_geopotential(
        &self,
        h_geopotential_m: f64,
    ) -> Result<AtmosphereSample, PhysicsError> {
        if !h_geopotential_m.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "geopotential altitude is NaN or infinite",
            });
        }
        if h_geopotential_m < 0.0 {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "geopotential altitude below 0 m'; USSA76 not defined for sub-surface",
            });
        }

        if h_geopotential_m > USSA76_MAX_GEOPOTENTIAL_M {
            return self.exoatmospheric_sample();
        }

        let layer = layer_for_geopotential(h_geopotential_m);
        let temperature_k = temperature_in_layer(layer, h_geopotential_m);
        let pressure_pa = pressure_in_layer(layer, h_geopotential_m, temperature_k);
        let density_kg_m3 = density_from_p_t(pressure_pa, temperature_k);
        let speed_of_sound_m_s = speed_of_sound_from_t(temperature_k);

        AtmosphereSample::new(
            density_kg_m3,
            pressure_pa,
            temperature_k,
            speed_of_sound_m_s,
        )
    }

    fn exoatmospheric_sample(self) -> Result<AtmosphereSample, PhysicsError> {
        match self.exoatmospheric_policy {
            ExoatmosphericPolicy::FailClosed => Err(PhysicsError::OutOfEnvelope {
                reason: "geometric altitude above USSA76 86 km ceiling",
            }),
            ExoatmosphericPolicy::ZeroDensityAboveCeiling => {
                // Use the ceiling-layer temperature (top of layer 6:
                // T = T_b + L · (h' - h'_b) at h' = 84852 m').
                let top_layer = LAYERS[6];
                let ceiling_temperature_k =
                    temperature_in_layer(top_layer, USSA76_MAX_GEOPOTENTIAL_M);
                let ceiling_speed_of_sound = speed_of_sound_from_t(ceiling_temperature_k);
                AtmosphereSample::new(0.0, 0.0, ceiling_temperature_k, ceiling_speed_of_sound)
            }
        }
    }
}

impl AtmosphereModel for UsStandard1976 {
    fn sample(
        &self,
        altitude_geometric_m: f64,
        _time: SimTime,
    ) -> Result<AtmosphereSample, PhysicsError> {
        if !altitude_geometric_m.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "geometric altitude is NaN or infinite",
            });
        }
        if altitude_geometric_m > USSA76_MAX_GEOMETRIC_M {
            return self.exoatmospheric_sample();
        }
        if altitude_geometric_m < 0.0 {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "geometric altitude below 0 m; USSA76 not defined for sub-surface",
            });
        }
        let h_geopotential_m =
            geopotential_from_geometric(altitude_geometric_m).min(USSA76_MAX_GEOPOTENTIAL_M);
        self.sample_at_geopotential(h_geopotential_m)
    }
}

// ---------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------

/// Pick the layer whose base geopotential bracket contains `h'`.
/// Caller has already verified `0 ≤ h' ≤ USSA76_MAX_GEOPOTENTIAL_M`.
fn layer_for_geopotential(h_geopotential_m: f64) -> Layer {
    // Linear scan (only 7 layers; unrolled by the compiler).
    let mut chosen = LAYERS[0];
    for layer in LAYERS.iter().copied() {
        if h_geopotential_m >= layer.base_geopotential_m {
            chosen = layer;
        } else {
            break;
        }
    }
    chosen
}

/// Temperature in a layer at geopotential altitude `h'`.
fn temperature_in_layer(layer: Layer, h_geopotential_m: f64) -> f64 {
    // T(h') = T_b + L · (h' - h'_b). Locked operand order.
    layer.base_temperature_k
        + layer.lapse_rate_k_per_m * (h_geopotential_m - layer.base_geopotential_m)
}

/// Pressure in a layer at geopotential altitude `h'` given the
/// temperature already evaluated there.
fn pressure_in_layer(layer: Layer, h_geopotential_m: f64, temperature_k: f64) -> f64 {
    if layer.lapse_rate_k_per_m == 0.0 {
        // Isothermal layer:
        //   p(h') = p_b · exp(-g₀' · M / (R · T_b) · (h' - h'_b))
        // Locked operand order.
        let coeff = USSA76_G0_M_S2 * USSA76_MOLAR_MASS_AIR_KG_KMOL
            / (USSA76_UNIVERSAL_GAS_CONSTANT * layer.base_temperature_k);
        let dh = h_geopotential_m - layer.base_geopotential_m;
        layer.base_pressure_pa * (-coeff * dh).exp()
    } else {
        // Gradient layer:
        //   p(h') = p_b · (T_b / T(h'))^(g₀' · M / (R · L))
        // Locked operand order.
        let exponent = USSA76_G0_M_S2 * USSA76_MOLAR_MASS_AIR_KG_KMOL
            / (USSA76_UNIVERSAL_GAS_CONSTANT * layer.lapse_rate_k_per_m);
        let temperature_ratio = layer.base_temperature_k / temperature_k;
        layer.base_pressure_pa * temperature_ratio.powf(exponent)
    }
}

/// Density via the ideal-gas law: `ρ = p · M / (R · T)`.
fn density_from_p_t(pressure_pa: f64, temperature_k: f64) -> f64 {
    // Locked order: p · M / (R · T).
    pressure_pa * USSA76_MOLAR_MASS_AIR_KG_KMOL / (USSA76_UNIVERSAL_GAS_CONSTANT * temperature_k)
}

/// Speed of sound from temperature: `a = √(γ · R · T / M)`.
fn speed_of_sound_from_t(temperature_k: f64) -> f64 {
    (USSA76_GAMMA_AIR * USSA76_UNIVERSAL_GAS_CONSTANT * temperature_k
        / USSA76_MOLAR_MASS_AIR_KG_KMOL)
        .sqrt()
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::atmosphere::geometric_from_geopotential;
    use approx::assert_abs_diff_eq;

    fn assert_relative(actual: f64, expected: f64, tol: f64, label: &str) {
        let denom = expected.abs().max(1.0e-30);
        let rel = (actual - expected).abs() / denom;
        assert!(
            rel < tol,
            "{label}: actual = {actual}, expected = {expected}, relative error = {rel}",
        );
    }

    // -----------------------------------------------------------------
    // Geopotential / geometric round-trip
    // -----------------------------------------------------------------

    #[test]
    fn geopotential_geometric_round_trip() {
        for z in [0.0, 1_000.0, 11_000.0, 50_000.0, 86_000.0] {
            let h = geopotential_from_geometric(z);
            let z_back = geometric_from_geopotential(h);
            assert_abs_diff_eq!(z_back, z, epsilon = 1.0e-6);
        }
    }

    #[test]
    fn geopotential_at_sea_level_is_zero() {
        assert_abs_diff_eq!(geopotential_from_geometric(0.0), 0.0);
    }

    #[test]
    fn geopotential_at_86km_geometric_is_close_to_84852m() {
        // USSA76 §1.2 documents 86 km geometric as the top boundary
        // and 84852 m' as the geopotential top. The closed-form
        // conversion `h' = R · z / (R + z)` with the pinned
        // `R = 6356766.0` gives `84852.046` for `z = 86000`. The
        // standard pins both endpoints separately rather than
        // deriving one from the other; the public geometric sampler
        // clamps this residual to the geopotential ceiling.
        let h = geopotential_from_geometric(USSA76_MAX_GEOMETRIC_M);
        assert_abs_diff_eq!(h, USSA76_MAX_GEOPOTENTIAL_M, epsilon = 0.1);
    }

    // -----------------------------------------------------------------
    // Sea-level reference values (NOAA-S/T 76-1562 table 1)
    // -----------------------------------------------------------------

    #[test]
    fn sea_level_temperature_and_pressure_match_published() {
        let atm = UsStandard1976::new();
        let s = atm.sample(0.0, SimTime::ZERO).unwrap();
        // Published sea-level values (USSA76 table 1):
        //   T = 288.150 K, p = 101325 Pa, ρ = 1.225 kg/m³,
        //   a = 340.294 m/s.
        assert_abs_diff_eq!(s.temperature_k, 288.150, epsilon = 1.0e-6);
        assert_abs_diff_eq!(s.pressure_pa, 101_325.0, epsilon = 1.0e-9);
        assert_abs_diff_eq!(s.density_kg_m3, 1.225, epsilon = 1.0e-6);
        assert_abs_diff_eq!(s.speed_of_sound_m_s, 340.294, epsilon = 5.0e-4);
    }

    // -----------------------------------------------------------------
    // Layer-boundary continuity (T)
    // -----------------------------------------------------------------

    /// USSA76 defines temperature as piecewise-continuous at each
    /// internal layer boundary: `T(h_b - ε) == T(h_b + ε)`. Verify
    /// to ULP at the six internal breakpoints.
    #[test]
    fn temperature_continuous_at_internal_boundaries() {
        for boundary in [11_000.0, 20_000.0, 32_000.0, 47_000.0, 51_000.0, 71_000.0] {
            let just_below =
                temperature_in_layer(layer_for_geopotential(boundary - 0.001), boundary - 0.001);
            let just_above =
                temperature_in_layer(layer_for_geopotential(boundary + 0.001), boundary + 0.001);
            // Allow a handful of ULPs for the 0.001 m offset linear-extrapolation.
            assert_abs_diff_eq!(just_below, just_above, epsilon = 1.0e-4);
        }
    }

    /// At the boundary itself (`h = h_b`), the layer-selection rule
    /// chooses the layer whose base equals the boundary, so `T` is
    /// exactly the published `T_b`.
    #[test]
    fn temperature_at_layer_base_matches_table() {
        for layer in LAYERS {
            let t = temperature_in_layer(
                layer_for_geopotential(layer.base_geopotential_m),
                layer.base_geopotential_m,
            );
            assert_abs_diff_eq!(t, layer.base_temperature_k);
        }
    }

    /// At the boundary itself the pressure equals the pinned `p_b`.
    #[test]
    fn pressure_at_layer_base_matches_table() {
        let atm = UsStandard1976::new();
        for layer in LAYERS {
            let s = atm
                .sample_at_geopotential(layer.base_geopotential_m)
                .unwrap();
            assert_relative(
                s.pressure_pa,
                layer.base_pressure_pa,
                1.0e-9,
                "layer-base pressure",
            );
        }
    }

    /// Recompute each next layer's base pressure from the previous
    /// layer using the runtime branch and locked operand order. This
    /// guards against drift between the pinned base-pressure table and
    /// the recurrence used inside each layer.
    #[test]
    fn pressure_base_table_matches_locked_recurrence_to_bits() {
        for pair in LAYERS.windows(2) {
            let layer = pair[0];
            let next = pair[1];
            let temperature_k = temperature_in_layer(layer, next.base_geopotential_m);
            let pressure_pa = pressure_in_layer(layer, next.base_geopotential_m, temperature_k);
            assert_eq!(
                pressure_pa.to_bits(),
                next.base_pressure_pa.to_bits(),
                "p_b recurrence mismatch at h_b = {} m'",
                next.base_geopotential_m,
            );
        }
    }

    // -----------------------------------------------------------------
    // Per-altitude regression (mid-layer T values, computed from the
    // layer constants by the closed-form formula)
    //
    // The "expected" values below are computed deterministically
    // from the layer table itself rather than transcribed from a
    // table 4 reference; they verify the chosen formula branch
    // (gradient vs. isothermal) produces the correct intra-layer
    // value. Layer-base values are verified separately against the
    // pinned constants.
    // -----------------------------------------------------------------

    #[test]
    fn temperature_within_layers_follows_lapse_rate() {
        let atm = UsStandard1976::new();
        // (h_geopotential, expected_T) — derived from layer table:
        //   T = T_b + L · (h - h_b)
        let cases = [
            (5_000.0, 288.15 - 6.5e-3 * 5_000.0),     // layer 0
            (15_000.0, 216.65),                       // layer 1 (isothermal)
            (25_000.0, 216.65 + 1.0e-3 * 5_000.0),    // layer 2
            (40_000.0, 228.65 + 2.8e-3 * 8_000.0),    // layer 3
            (49_000.0, 270.65),                       // layer 4 (isothermal)
            (60_000.0, 270.65 + (-2.8e-3) * 9_000.0), // layer 5
            (80_000.0, 214.65 + (-2.0e-3) * 9_000.0), // layer 6
        ];
        for (h, t_expected) in cases {
            let s = atm.sample_at_geopotential(h).unwrap();
            assert_abs_diff_eq!(s.temperature_k, t_expected, epsilon = 1.0e-9);
        }
    }

    /// At the base of each layer, the sample must match the pinned
    /// `(T_b, p_b)` constants exactly (within f64 precision).
    #[test]
    fn sample_at_layer_base_matches_pinned_constants() {
        let atm = UsStandard1976::new();
        for layer in LAYERS {
            let s = atm
                .sample_at_geopotential(layer.base_geopotential_m)
                .unwrap();
            assert_abs_diff_eq!(s.temperature_k, layer.base_temperature_k);
            assert_relative(
                s.pressure_pa,
                layer.base_pressure_pa,
                1.0e-9,
                &format!("p at h_b = {} m'", layer.base_geopotential_m),
            );
        }
    }

    // -----------------------------------------------------------------
    // Property tests
    // -----------------------------------------------------------------

    /// `ρ > 0` everywhere in `[0, 86 km]` geopotential.
    #[test]
    fn density_strictly_positive_throughout_envelope() {
        let atm = UsStandard1976::new();
        let mut h = 0.0_f64;
        while h <= USSA76_MAX_GEOPOTENTIAL_M {
            let s = atm.sample_at_geopotential(h).unwrap();
            assert!(s.density_kg_m3 > 0.0, "ρ at h = {h} = {}", s.density_kg_m3);
            h += 1_000.0;
        }
    }

    /// `T > 0`, `p > 0`, `a > 0` everywhere in `[0, 86 km]`.
    #[test]
    fn temperature_pressure_speed_of_sound_strictly_positive() {
        let atm = UsStandard1976::new();
        let mut h = 0.0_f64;
        while h <= USSA76_MAX_GEOPOTENTIAL_M {
            let s = atm.sample_at_geopotential(h).unwrap();
            assert!(s.temperature_k > 0.0, "T at h = {h}");
            assert!(s.pressure_pa > 0.0, "p at h = {h}");
            assert!(s.speed_of_sound_m_s > 0.0, "a at h = {h}");
            h += 1_000.0;
        }
    }

    #[test]
    fn pressure_monotonically_decreases_throughout_envelope() {
        let atm = UsStandard1976::new();
        let mut h = 0.0_f64;
        let mut previous_pressure_pa = atm.sample_at_geopotential(h).unwrap().pressure_pa;
        h += 100.0;
        while h <= USSA76_MAX_GEOPOTENTIAL_M {
            let pressure_pa = atm.sample_at_geopotential(h).unwrap().pressure_pa;
            assert!(
                pressure_pa < previous_pressure_pa,
                "p did not decrease from h = {} to h = {h}",
                h - 100.0,
            );
            previous_pressure_pa = pressure_pa;
            h += 100.0;
        }
    }

    // -----------------------------------------------------------------
    // Out-of-envelope behaviour
    // -----------------------------------------------------------------

    #[test]
    fn fails_closed_above_86_km_by_default() {
        let atm = UsStandard1976::new();
        let err = atm.sample(86_001.0, SimTime::ZERO).unwrap_err();
        assert!(matches!(err, PhysicsError::OutOfEnvelope { .. }));
    }

    #[test]
    fn exact_86_km_geometric_is_inside_envelope() {
        let atm = UsStandard1976::new();
        let geometric = atm.sample(USSA76_MAX_GEOMETRIC_M, SimTime::ZERO).unwrap();
        let geopotential = atm
            .sample_at_geopotential(USSA76_MAX_GEOPOTENTIAL_M)
            .unwrap();
        assert_eq!(
            geometric.density_kg_m3.to_bits(),
            geopotential.density_kg_m3.to_bits(),
        );
        assert_eq!(
            geometric.pressure_pa.to_bits(),
            geopotential.pressure_pa.to_bits(),
        );
        assert_eq!(
            geometric.temperature_k.to_bits(),
            geopotential.temperature_k.to_bits(),
        );
        assert_eq!(
            geometric.speed_of_sound_m_s.to_bits(),
            geopotential.speed_of_sound_m_s.to_bits(),
        );
    }

    #[test]
    fn fails_closed_below_zero_altitude() {
        let atm = UsStandard1976::new();
        let err = atm.sample(-1.0, SimTime::ZERO).unwrap_err();
        assert!(matches!(err, PhysicsError::OutOfEnvelope { .. }));
    }

    #[test]
    fn fails_closed_on_non_finite_altitude() {
        let atm = UsStandard1976::new();
        assert!(matches!(
            atm.sample(f64::NAN, SimTime::ZERO),
            Err(PhysicsError::NonFinite { .. })
        ));
        assert!(matches!(
            atm.sample(f64::INFINITY, SimTime::ZERO),
            Err(PhysicsError::NonFinite { .. })
        ));
    }

    #[test]
    fn exoatmospheric_policy_returns_zero_density_when_set() {
        let atm = UsStandard1976::with_exoatmospheric_policy(
            ExoatmosphericPolicy::ZeroDensityAboveCeiling,
        );
        let s = atm.sample(100_000.0, SimTime::ZERO).unwrap();
        assert_eq!(s.density_kg_m3, 0.0);
        assert_eq!(s.pressure_pa, 0.0);
        assert!(s.temperature_k > 0.0);
        assert!(s.speed_of_sound_m_s > 0.0);
    }

    #[test]
    fn exoatmospheric_policy_does_not_trigger_at_exact_86_km() {
        let atm = UsStandard1976::with_exoatmospheric_policy(
            ExoatmosphericPolicy::ZeroDensityAboveCeiling,
        );
        let s = atm.sample(USSA76_MAX_GEOMETRIC_M, SimTime::ZERO).unwrap();
        assert!(s.density_kg_m3 > 0.0);
        assert!(s.pressure_pa > 0.0);
    }

    // -----------------------------------------------------------------
    // Determinism
    // -----------------------------------------------------------------

    #[test]
    fn bit_stable_across_two_evaluations() {
        let atm = UsStandard1976::new();
        for h in [0.0, 5_000.0, 25_000.0, 60_000.0, 84_852.0] {
            let a = atm.sample_at_geopotential(h).unwrap();
            let b = atm.sample_at_geopotential(h).unwrap();
            assert_eq!(a.density_kg_m3.to_bits(), b.density_kg_m3.to_bits());
            assert_eq!(a.pressure_pa.to_bits(), b.pressure_pa.to_bits());
            assert_eq!(a.temperature_k.to_bits(), b.temperature_k.to_bits());
            assert_eq!(
                a.speed_of_sound_m_s.to_bits(),
                b.speed_of_sound_m_s.to_bits()
            );
        }
    }
}
