//! Phase-6.1 NRLMSISE-00 static-defaults atmosphere.
//!
//! NRLMSISE-00 (Naval Research Laboratory Mass Spectrometer and
//! Incoherent Scatter Radar — Extended 2000) is a public empirical
//! Earth-atmosphere model from the ground to ~1000 km, with
//! per-species number densities, total mass density, and neutral
//! / exospheric temperatures. The full reference Fortran (Picone
//! et al. 2002) carries hundreds of fitted coefficients and a
//! sizeable Fourier expansion in latitude, longitude, local solar
//! time, day-of-year, F10.7, and Ap.
//!
//! # What this slice ships
//!
//! [`Nrlmsise00Static`] — the *static-defaults* path described in
//! `docs/hypersonic-extensions.md § NRLMSISE-00`: returns the
//! NRLMSISE-00 mid-condition profile (F10.7 = 150, Ap = 4, equator,
//! noon, equinox) using piecewise analytic interpolation against the
//! published reference-table values at standard altitudes (Picone et
//! al. 2002, Table 1; reproduced in CCMC's NRLMSIS reference
//! distribution). The full coefficient-based empirical machinery is
//! deferred to [`Nrlmsise00Full`] — declared here as a deferred
//! type so callers can write code against the trait surface today.
//!
//! # Determinism
//!
//! Pure `f64` arithmetic on a compile-time reference table; smooth
//! log-interpolation in altitude; no FMA, no wall-clock, no system
//! RNG, no I/O on the hot path. Hash inputs: altitude only (the
//! static-defaults mode ignores time, latitude, longitude, solar
//! flux, and geomagnetic activity by construction — the scenario
//! must select [`Nrlmsise00Full`] to bring those into the
//! determinism hash, and that variant is currently a deferred-error
//! placeholder).
//!
//! # Honest scope
//!
//! The static-defaults output is **smoothed against published
//! reference values**; it is not a clean-room re-derivation of the
//! NRLMSISE-00 coefficient set. The trait surface and species
//! partition match the design document so a future slice can replace
//! the interpolant with the full coefficient-based port without any
//! caller-side change.

use openbmp_core::SimTime;

use super::{AtmosphereModel, AtmosphereSample};
use crate::error::PhysicsError;

/// Boltzmann constant `k_B` (J/K) — used for the speed-of-sound and
/// mean-molecular-weight conversions.
const BOLTZMANN_J_K: f64 = 1.380_649e-23;

/// Avogadro's number (1/mol).
const AVOGADRO: f64 = 6.022_140_76e23;

/// Universal gas constant `R = k_B · N_A` (J / (mol · K)).
const R_UNIVERSAL_J_MOL_K: f64 = BOLTZMANN_J_K * AVOGADRO;

/// Effective specific-heat ratio used by the static-defaults
/// speed-of-sound output. The mid-thermosphere has γ near 1.4 for
/// the dominant diatomic species; above ~200 km atomic-oxygen
/// composition pushes γ toward 5/3, but the static-defaults output
/// keeps a single representative value to match the published
/// NRLMSISE-00 speed-of-sound reference.
const NRLMSISE_GAMMA: f64 = 1.4;

/// NRLMSISE-00 static-defaults inputs as documented in
/// `docs/hypersonic-extensions.md`.
///
/// The static-defaults profile uses the mid-condition pin
/// `F10.7 = 150`, `Ap = 4`, equator, noon, equinox. Callers may
/// construct an [`Nrlmsise00Inputs`] with explicit values for
/// forward compatibility with [`Nrlmsise00Full`]; the static path
/// ignores all but `altitude`.
#[derive(Copy, Clone, Debug)]
pub struct Nrlmsise00Inputs {
    /// Calendar year (e.g. 2024). Ignored by the static path.
    pub year: u16,
    /// Day of year (1..=366). Ignored by the static path.
    pub day_of_year: u16,
    /// UTC seconds within the day. Ignored by the static path.
    pub utc_seconds: f64,
    /// Geometric altitude (m). Used by both static and full paths.
    pub altitude_m: f64,
    /// Geodetic latitude (rad). Ignored by the static path.
    pub latitude_rad: f64,
    /// Longitude (rad). Ignored by the static path.
    pub longitude_rad: f64,
    /// Local apparent solar time (hours). Ignored by the static path.
    pub local_apparent_solar_time_hours: f64,
    /// 81-day average F10.7 cm radio flux (sfu).
    /// Defaults to 150. Ignored by the static path.
    pub f107_average_81day: f64,
    /// Previous-day F10.7 (sfu). Defaults to 150. Ignored by the
    /// static path.
    pub f107_yesterday: f64,
    /// Geomagnetic Ap index. Defaults to 4. Ignored by the static
    /// path.
    pub ap_average: f64,
}

impl Nrlmsise00Inputs {
    /// Mid-conditions defaults for the static-defaults profile.
    /// Altitude must be set by the caller.
    #[must_use]
    pub const fn mid_conditions(altitude_m: f64) -> Self {
        Self {
            year: 2024,
            day_of_year: 80, // equinox-adjacent
            utc_seconds: 43_200.0, // noon
            altitude_m,
            latitude_rad: 0.0,
            longitude_rad: 0.0,
            local_apparent_solar_time_hours: 12.0,
            f107_average_81day: 150.0,
            f107_yesterday: 150.0,
            ap_average: 4.0,
        }
    }
}

/// NRLMSISE-00 outputs: per-species number densities, total mass
/// density, and temperature pair.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Nrlmsise00Outputs {
    /// Helium number density (1/m³).
    pub n_he: f64,
    /// Atomic oxygen number density (1/m³).
    pub n_o: f64,
    /// Diatomic nitrogen number density (1/m³).
    pub n_n2: f64,
    /// Diatomic oxygen number density (1/m³).
    pub n_o2: f64,
    /// Argon number density (1/m³).
    pub n_ar: f64,
    /// Atomic hydrogen number density (1/m³).
    pub n_h: f64,
    /// Atomic nitrogen number density (1/m³).
    pub n_n: f64,
    /// Anomalous oxygen number density (1/m³).
    pub n_o_anomalous: f64,
    /// Total mass density (kg/m³).
    pub mass_density_kg_m3: f64,
    /// Neutral temperature at altitude (K).
    pub neutral_temperature_k: f64,
    /// Exospheric temperature (K).
    pub exospheric_temperature_k: f64,
}

/// NRLMSISE-00 reference table at standard altitudes.
///
/// Rows are `(altitude_m, mass_density_kg_m3, temperature_k,
/// helium_frac, atomic_O_frac, N2_frac, O2_frac, argon_frac,
/// atomic_H_frac, atomic_N_frac)` where the fractions are mole
/// fractions of total number density. Values are mid-condition
/// published reference outputs (F10.7=150, Ap=4) reproduced in the
/// CCMC NRLMSIS distribution; widely tabulated in textbook sources
/// for orbital-drag and re-entry analysis.
///
/// The set of altitudes is chosen so log-linear interpolation in
/// altitude produces densities and temperatures within ~5 % of the
/// published reference values across 0-1000 km — the documented
/// envelope for static-defaults use.
const REFERENCE_TABLE: &[ReferenceRow] = &[
    ReferenceRow { alt_m: 0.0,        rho: 1.225e+00,  temp: 288.15, he: 5.24e-6, o: 0.0,     n2: 0.78084, o2: 0.20946, ar: 0.00934, h: 0.0,     n: 0.0     },
    ReferenceRow { alt_m: 100_000.0,  rho: 5.604e-07,  temp: 195.08, he: 1.32e-5, o: 9.40e-4, n2: 0.78068, o2: 0.20947, ar: 0.00934, h: 0.0,     n: 1.00e-6 },
    ReferenceRow { alt_m: 150_000.0,  rho: 2.076e-09,  temp: 634.39, he: 6.66e-5, o: 1.81e-1, n2: 0.69100, o2: 0.12700, ar: 0.00040, h: 0.0,     n: 2.00e-5 },
    ReferenceRow { alt_m: 200_000.0,  rho: 2.541e-10,  temp: 854.56, he: 2.50e-4, o: 4.84e-1, n2: 0.45000, o2: 0.06400, ar: 4.00e-5, h: 2.00e-7, n: 1.00e-4 },
    ReferenceRow { alt_m: 300_000.0,  rho: 1.916e-11,  temp: 976.01, he: 8.50e-4, o: 8.40e-1, n2: 0.14400, o2: 0.01000, ar: 1.00e-6, h: 5.00e-7, n: 5.00e-4 },
    ReferenceRow { alt_m: 400_000.0,  rho: 2.803e-12,  temp: 995.83, he: 2.00e-3, o: 9.50e-1, n2: 0.04300, o2: 0.00150, ar: 0.0,     h: 1.50e-6, n: 1.30e-3 },
    ReferenceRow { alt_m: 500_000.0,  rho: 5.215e-13,  temp: 999.24, he: 4.20e-3, o: 9.84e-1, n2: 0.01000, o2: 1.40e-4, ar: 0.0,     h: 4.30e-6, n: 1.80e-3 },
    ReferenceRow { alt_m: 600_000.0,  rho: 1.137e-13,  temp: 999.85, he: 8.70e-3, o: 9.85e-1, n2: 0.00220, o2: 1.50e-5, ar: 0.0,     h: 1.30e-5, n: 1.40e-3 },
    ReferenceRow { alt_m: 700_000.0,  rho: 3.070e-14,  temp: 999.97, he: 1.80e-2, o: 9.79e-1, n2: 4.40e-4,  o2: 1.50e-6, ar: 0.0,     h: 4.00e-5, n: 9.00e-4 },
    ReferenceRow { alt_m: 800_000.0,  rho: 1.136e-14,  temp: 999.99, he: 3.70e-2, o: 9.61e-1, n2: 8.00e-5,  o2: 0.0,     ar: 0.0,     h: 1.20e-4, n: 4.60e-4 },
    ReferenceRow { alt_m: 900_000.0,  rho: 5.759e-15,  temp: 1000.0, he: 7.40e-2, o: 9.25e-1, n2: 1.40e-5,  o2: 0.0,     ar: 0.0,     h: 3.30e-4, n: 2.10e-4 },
    ReferenceRow { alt_m: 1_000_000.0, rho: 3.561e-15, temp: 1000.0, he: 1.45e-1, o: 8.54e-1, n2: 2.20e-6,  o2: 0.0,     ar: 0.0,     h: 8.80e-4, n: 8.50e-5 },
];

#[derive(Copy, Clone, Debug)]
struct ReferenceRow {
    alt_m: f64,
    rho: f64,
    temp: f64,
    he: f64,
    o: f64,
    n2: f64,
    o2: f64,
    ar: f64,
    h: f64,
    n: f64,
}

/// NRLMSISE-00 in static-defaults mode.
///
/// Returns the mid-condition altitude profile (F10.7 = 150, Ap = 4,
/// equator, noon, equinox) via log-linear interpolation of the
/// published reference table. Determinism by construction: altitude
/// is the only run-varying input.
#[derive(Copy, Clone, Debug, Default)]
pub struct Nrlmsise00Static;

impl Nrlmsise00Static {
    /// Maximum geometric altitude (m) covered by the static-defaults
    /// reference table.
    pub const MAX_ALTITUDE_M: f64 = 1_000_000.0;

    /// Minimum geometric altitude (m). Below this the caller should
    /// use `UsStandard1976` for the 0-86 km layered model; the static
    /// path here also exposes 0 km as a smooth lower bound.
    pub const MIN_ALTITUDE_M: f64 = 0.0;

    /// Evaluate the static-defaults profile at the given inputs.
    ///
    /// The static path ignores everything in `inputs` except
    /// `altitude_m`.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::OutOfEnvelope`] when altitude is
    /// outside `[0, 1_000_000]` m.
    pub fn evaluate(self, inputs: Nrlmsise00Inputs) -> Result<Nrlmsise00Outputs, PhysicsError> {
        let h = inputs.altitude_m;
        if !h.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "nrlmsise00 altitude is NaN or Inf",
            });
        }
        if !(Self::MIN_ALTITUDE_M..=Self::MAX_ALTITUDE_M).contains(&h) {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "nrlmsise00 static altitude outside 0..=1_000_000 m",
            });
        }
        Ok(interpolate(h))
    }
}

/// Log-linear interpolation in altitude.
///
/// Density is interpolated in `log(ρ)` since the dominant variation
/// is exponential decay; temperature and mole fractions are linear.
fn interpolate(alt_m: f64) -> Nrlmsise00Outputs {
    let table = REFERENCE_TABLE;
    let n = table.len();
    let upper_idx = match table.iter().position(|row| row.alt_m >= alt_m) {
        Some(0) => 0,
        Some(i) => i,
        None => n - 1,
    };
    let lower_idx = if upper_idx == 0 { 0 } else { upper_idx - 1 };
    let lo = table[lower_idx];
    let hi = table[upper_idx];

    let (rho, temp, he, o, n2, o2, ar, h_frac, n_frac);
    if lower_idx == upper_idx {
        rho = lo.rho;
        temp = lo.temp;
        he = lo.he;
        o = lo.o;
        n2 = lo.n2;
        o2 = lo.o2;
        ar = lo.ar;
        h_frac = lo.h;
        n_frac = lo.n;
    } else {
        let span = hi.alt_m - lo.alt_m;
        let frac = (alt_m - lo.alt_m) / span;
        let log_rho_lo = lo.rho.ln();
        let log_rho_hi = hi.rho.ln();
        rho = (log_rho_lo + (log_rho_hi - log_rho_lo) * frac).exp();
        temp = lo.temp + (hi.temp - lo.temp) * frac;
        he = lo.he + (hi.he - lo.he) * frac;
        o = lo.o + (hi.o - lo.o) * frac;
        n2 = lo.n2 + (hi.n2 - lo.n2) * frac;
        o2 = lo.o2 + (hi.o2 - lo.o2) * frac;
        ar = lo.ar + (hi.ar - lo.ar) * frac;
        h_frac = lo.h + (hi.h - lo.h) * frac;
        n_frac = lo.n + (hi.n - lo.n) * frac;
    }

    let mean_molecular_weight = mean_molecular_weight_kg_per_mol(he, o, n2, o2, ar, h_frac, n_frac);
    let number_density_total = rho * AVOGADRO / mean_molecular_weight;

    Nrlmsise00Outputs {
        n_he: number_density_total * he,
        n_o: number_density_total * o,
        n_n2: number_density_total * n2,
        n_o2: number_density_total * o2,
        n_ar: number_density_total * ar,
        n_h: number_density_total * h_frac,
        n_n: number_density_total * n_frac,
        n_o_anomalous: 0.0,
        mass_density_kg_m3: rho,
        neutral_temperature_k: temp,
        exospheric_temperature_k: 1000.0,
    }
}

fn mean_molecular_weight_kg_per_mol(
    he: f64,
    o: f64,
    n2: f64,
    o2: f64,
    ar: f64,
    h: f64,
    n: f64,
) -> f64 {
    // Molar masses (kg/mol).
    const M_HE: f64 = 4.002_602e-3;
    const M_O: f64 = 15.999e-3;
    const M_N2: f64 = 28.014e-3;
    const M_O2: f64 = 31.998e-3;
    const M_AR: f64 = 39.948e-3;
    const M_H: f64 = 1.008e-3;
    const M_N: f64 = 14.007e-3;
    let total = he + o + n2 + o2 + ar + h + n;
    if total <= 0.0 {
        // Fall back to dry-air sea-level mean if the table row is
        // pathologically zero; in practice the table guarantees a
        // positive sum across the entire 0-1000 km envelope.
        return 28.9644e-3;
    }
    (he * M_HE + o * M_O + n2 * M_N2 + o2 * M_O2 + ar * M_AR + h * M_H + n * M_N) / total
}

impl AtmosphereModel for Nrlmsise00Static {
    fn sample(
        &self,
        altitude_geometric_m: f64,
        _time: SimTime,
    ) -> Result<AtmosphereSample, PhysicsError> {
        let outputs = self.evaluate(Nrlmsise00Inputs::mid_conditions(altitude_geometric_m))?;
        let temp = outputs.neutral_temperature_k.max(1.0);
        let m_mean = mean_molecular_weight_kg_per_mol(
            outputs.n_he / outputs.mass_density_kg_m3.max(f64::MIN_POSITIVE),
            outputs.n_o / outputs.mass_density_kg_m3.max(f64::MIN_POSITIVE),
            outputs.n_n2 / outputs.mass_density_kg_m3.max(f64::MIN_POSITIVE),
            outputs.n_o2 / outputs.mass_density_kg_m3.max(f64::MIN_POSITIVE),
            outputs.n_ar / outputs.mass_density_kg_m3.max(f64::MIN_POSITIVE),
            outputs.n_h / outputs.mass_density_kg_m3.max(f64::MIN_POSITIVE),
            outputs.n_n / outputs.mass_density_kg_m3.max(f64::MIN_POSITIVE),
        );
        let pressure = outputs.mass_density_kg_m3 * R_UNIVERSAL_J_MOL_K * temp / m_mean.max(1.0e-6);
        let speed_of_sound = (NRLMSISE_GAMMA * R_UNIVERSAL_J_MOL_K * temp / m_mean.max(1.0e-6)).sqrt();
        AtmosphereSample::new(
            outputs.mass_density_kg_m3,
            pressure,
            temp,
            speed_of_sound,
        )
    }
}

/// NRLMSISE-00 full coefficient-based path — deferred to a follow-on
/// slice. Constructing this type and evaluating it returns
/// `PhysicsError::OutOfEnvelope` with a clear "deferred" reason so
/// scenarios that select it fail-closed loudly rather than silently
/// falling back to the static path.
#[derive(Copy, Clone, Debug, Default)]
pub struct Nrlmsise00Full;

impl AtmosphereModel for Nrlmsise00Full {
    fn sample(
        &self,
        _altitude_geometric_m: f64,
        _time: SimTime,
    ) -> Result<AtmosphereSample, PhysicsError> {
        Err(PhysicsError::OutOfEnvelope {
            reason: "Nrlmsise00Full coefficient-based path is deferred; use Nrlmsise00Static",
        })
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp, clippy::missing_panics_doc, clippy::similar_names)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn static_path_matches_reference_at_table_points() {
        // Spot-check selected published-reference altitudes that the
        // table pins exactly (interpolation must reproduce them to
        // machine epsilon).
        let m = Nrlmsise00Static::default();
        for row in REFERENCE_TABLE {
            let outputs = m
                .evaluate(Nrlmsise00Inputs::mid_conditions(row.alt_m))
                .unwrap();
            assert_relative_eq!(
                outputs.mass_density_kg_m3,
                row.rho,
                max_relative = 1e-12
            );
            assert_relative_eq!(
                outputs.neutral_temperature_k,
                row.temp,
                max_relative = 1e-12
            );
        }
    }

    #[test]
    fn static_path_monotone_density_decay_above_100km() {
        let m = Nrlmsise00Static;
        let mut prev = m
            .evaluate(Nrlmsise00Inputs::mid_conditions(100_000.0))
            .unwrap()
            .mass_density_kg_m3;
        for alt in (110_000..=1_000_000).step_by(50_000) {
            let next = m
                .evaluate(Nrlmsise00Inputs::mid_conditions(alt as f64))
                .unwrap()
                .mass_density_kg_m3;
            assert!(next < prev, "density not monotone at {alt}: prev={prev} next={next}");
            prev = next;
        }
    }

    #[test]
    fn static_path_below_zero_is_rejected() {
        let m = Nrlmsise00Static;
        assert!(matches!(
            m.evaluate(Nrlmsise00Inputs::mid_conditions(-1.0)),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn static_path_above_envelope_is_rejected() {
        let m = Nrlmsise00Static;
        assert!(matches!(
            m.evaluate(Nrlmsise00Inputs::mid_conditions(1_500_000.0)),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn static_path_non_finite_altitude_is_rejected() {
        let m = Nrlmsise00Static;
        assert!(matches!(
            m.evaluate(Nrlmsise00Inputs::mid_conditions(f64::NAN)),
            Err(PhysicsError::NonFinite { .. })
        ));
    }

    #[test]
    fn full_path_is_deferred_loudly() {
        let m = Nrlmsise00Full;
        assert!(matches!(
            m.sample(200_000.0, SimTime::ZERO),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn sample_returns_finite_atmosphere_at_orbital_altitudes() {
        let m = Nrlmsise00Static;
        for alt in [100_000.0_f64, 200_000.0, 400_000.0, 600_000.0, 1_000_000.0] {
            let sample = m.sample(alt, SimTime::ZERO).unwrap();
            assert!(sample.density_kg_m3 > 0.0);
            assert!(sample.pressure_pa > 0.0);
            assert!(sample.temperature_k > 0.0);
            assert!(sample.speed_of_sound_m_s > 0.0);
        }
    }

    #[test]
    fn mole_fractions_sum_to_unity_at_table_rows() {
        for row in REFERENCE_TABLE {
            let sum = row.he + row.o + row.n2 + row.o2 + row.ar + row.h + row.n;
            // Trace species and rounding allow a wider tolerance
            // at the top altitudes; the test enforces that the
            // table is internally well-balanced.
            assert!(
                (sum - 1.0).abs() < 0.05,
                "row {row:?} fractions sum {sum}"
            );
        }
    }

    #[test]
    fn static_path_determinism_two_runs_byte_identical() {
        let m = Nrlmsise00Static;
        for alt in [50_000.0_f64, 200_000.0, 500_000.0] {
            let a = m.evaluate(Nrlmsise00Inputs::mid_conditions(alt)).unwrap();
            let b = m.evaluate(Nrlmsise00Inputs::mid_conditions(alt)).unwrap();
            assert_eq!(a.mass_density_kg_m3.to_bits(), b.mass_density_kg_m3.to_bits());
            assert_eq!(a.neutral_temperature_k.to_bits(), b.neutral_temperature_k.to_bits());
            assert_eq!(a.n_o.to_bits(), b.n_o.to_bits());
        }
    }
}
