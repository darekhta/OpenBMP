//! NRLMSISE-00 static-defaults atmosphere.
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
//! # What this module ships
//!
//! [`Nrlmsise00Static`] — the *static-defaults* path described in
//! `docs/hypersonic-extensions.md § NRLMSISE-00`: returns the
//! NRLMSISE-00 mid-condition profile (F10.7 = 150, Ap = 4, equator,
//! noon, equinox) using interpolation against public NRLMSISE-00
//! model outputs at standard altitudes. The audit regenerated these
//! rows from the public C model interface exposed by the
//! `nrlmsise00` Python package:
//!
//! ```text
//! gtd7(year=2024, doy=80, sec=43200, alt_km, lat=0, lon=0,
//!      lst=12, f107a=150, f107=150, ap=4)
//! ```
//!
//! [`Nrlmsise00Full`] — the coefficient path. OpenBMP ships the
//! coefficient tables directly from the NASA/CCMC `ModelWeb` archive
//! copy of `nrlmsise-00_data.c` and evaluates them with an in-repo
//! pure-Rust implementation.
//!
//! # Determinism
//!
//! Pure `f64` arithmetic on a compile-time reference table; smooth
//! log-interpolation in altitude; no FMA, no wall-clock, no system
//! RNG, no I/O on the hot path. Hash inputs: altitude only (the
//! static-defaults mode ignores time, latitude, longitude, solar
//! flux, and geomagnetic activity by construction — the scenario
//! must select [`Nrlmsise00Full`] to bring those into the
//! determinism hash.
//!
//! # Honest scope
//!
//! The static-defaults output is **smoothed against published
//! reference values**. Use [`Nrlmsise00Full`] when the query must
//! respond to date/time, geodetic position, F10.7, and Ap.

use openbmp_core::SimTime;

use super::{AtmosphereModel, AtmosphereSample};
use crate::error::PhysicsError;

#[path = "nrlmsise00_coefficients.rs"]
mod nrlmsise00_coefficients;
#[path = "nrlmsise00_model.rs"]
mod nrlmsise00_model;

/// Boltzmann constant `k_B` (J/K) — used for the speed-of-sound and
/// mean-molecular-weight conversions.
const BOLTZMANN_J_K: f64 = 1.380_649e-23;

/// Convert number density from `cm^-3` to `m^-3`.
const CM3_TO_M3: f64 = 1.0e6;

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
#[derive(Copy, Clone, Debug, PartialEq)]
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

#[derive(Copy, Clone, Debug)]
struct Nrlmsise00LowLevelInput {
    day_of_year: u32,
    ut_seconds: f64,
    altitude_km: f64,
    latitude_deg: f64,
    longitude_deg: f64,
    local_solar_time_hours: f64,
    f107_daily: f64,
    f107_avg: f64,
    ap_daily: f64,
    ap_array: [f64; 7],
}

impl Nrlmsise00Inputs {
    /// Mid-conditions defaults for the static-defaults profile.
    /// Altitude must be set by the caller.
    #[must_use]
    pub const fn mid_conditions(altitude_m: f64) -> Self {
        Self {
            year: 2024,
            day_of_year: 80,       // equinox-adjacent
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

    /// Return a copy with `altitude_m` replaced.
    #[must_use]
    pub const fn with_altitude_m(mut self, altitude_m: f64) -> Self {
        self.altitude_m = altitude_m;
        self
    }

    /// Validate the full NRLMSISE-00 input envelope used by the
    /// reserved coefficient path.
    ///
    /// The static-defaults profile intentionally ignores these fields;
    /// this method exists so scenario plumbing can validate a full
    /// MSIS query before the coefficient port lands.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::NonFinite`] for non-finite numeric
    /// fields, [`PhysicsError::InvalidParameter`] for malformed date /
    /// solar / geomagnetic inputs, and [`PhysicsError::OutOfEnvelope`]
    /// for altitude outside the documented 0-1000 km range.
    pub fn validate_full_path(&self) -> Result<(), PhysicsError> {
        if !self.altitude_m.is_finite()
            || !self.utc_seconds.is_finite()
            || !self.latitude_rad.is_finite()
            || !self.longitude_rad.is_finite()
            || !self.local_apparent_solar_time_hours.is_finite()
            || !self.f107_average_81day.is_finite()
            || !self.f107_yesterday.is_finite()
            || !self.ap_average.is_finite()
        {
            return Err(PhysicsError::NonFinite {
                reason: "nrlmsise00 full input field is NaN or Inf",
            });
        }
        if !(1..=366).contains(&self.day_of_year) {
            return Err(PhysicsError::InvalidParameter {
                reason: "nrlmsise00 day_of_year must be in 1..=366",
            });
        }
        if !(0.0..86_400.0).contains(&self.utc_seconds) {
            return Err(PhysicsError::InvalidParameter {
                reason: "nrlmsise00 utc_seconds must be in [0, 86400)",
            });
        }
        if !(0.0..24.0).contains(&self.local_apparent_solar_time_hours) {
            return Err(PhysicsError::InvalidParameter {
                reason: "nrlmsise00 local solar time must be in [0, 24)",
            });
        }
        if !(-std::f64::consts::FRAC_PI_2..=std::f64::consts::FRAC_PI_2)
            .contains(&self.latitude_rad)
        {
            return Err(PhysicsError::InvalidParameter {
                reason: "nrlmsise00 latitude must be in [-pi/2, pi/2]",
            });
        }
        if !(-std::f64::consts::PI..=std::f64::consts::PI).contains(&self.longitude_rad) {
            return Err(PhysicsError::InvalidParameter {
                reason: "nrlmsise00 longitude must be in [-pi, pi]",
            });
        }
        if self.f107_average_81day <= 0.0 || self.f107_yesterday <= 0.0 || self.ap_average < 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "nrlmsise00 F10.7 values must be positive and Ap must be non-negative",
            });
        }
        if !(Nrlmsise00Static::MIN_ALTITUDE_M..=Nrlmsise00Static::MAX_ALTITUDE_M)
            .contains(&self.altitude_m)
        {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "nrlmsise00 altitude outside 0..=1_000_000 m",
            });
        }
        Ok(())
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
/// Rows are public-model outputs for the static-defaults condition
/// declared above. Number densities are stored in `1/m^3`; mass
/// density is stored in `kg/m^3`; temperatures are Kelvin.
#[allow(clippy::unreadable_literal)]
const REFERENCE_TABLE: &[ReferenceRow] = &[
    ReferenceRow {
        alt_m: 0.000000000000e+00,
        rho: 1.183070023108e+00,
        temp: 3.004810309657e+02,
        texo: 1.027318464900e+03,
        n_he: 1.290415065750e+20,
        n_o: 0.000000000000e+00,
        n_n2: 1.922915969896e+25,
        n_o2: 5.158606672441e+24,
        n_ar: 2.300095937391e+23,
        n_h: 0.000000000000e+00,
        n_n: 0.000000000000e+00,
        n_o_anomalous: 0.000000000000e+00,
    },
    ReferenceRow {
        alt_m: 1.000000000000e+05,
        rho: 7.283486293373e-07,
        temp: 1.829433801178e+02,
        texo: 1.027318464900e+03,
        n_he: 1.509804049847e+14,
        n_o: 6.882496845920e+17,
        n_n2: 1.222722698995e+19,
        n_o2: 2.513528291891e+18,
        n_ar: 1.239085933383e+17,
        n_h: 2.204268500612e+13,
        n_n: 6.307149380445e+11,
        n_o_anomalous: 3.371056080156e-37,
    },
    ReferenceRow {
        alt_m: 1.500000000000e+05,
        rho: 2.052959861123e-09,
        temp: 7.051340462723e+02,
        texo: 1.130778214192e+03,
        n_he: 1.959738601244e+13,
        n_o: 2.089144809148e+16,
        n_n2: 3.019837735253e+16,
        n_o2: 1.697682429498e+15,
        n_ar: 5.141770572692e+13,
        n_h: 4.852705777643e+11,
        n_n: 3.168941308442e+13,
        n_o_anomalous: 1.344916740252e-15,
    },
    ReferenceRow {
        alt_m: 2.000000000000e+05,
        rho: 3.155122198534e-10,
        temp: 9.751425526154e+02,
        texo: 1.130778214192e+03,
        n_he: 1.229624110371e+13,
        n_o: 5.370039640700e+15,
        n_n2: 3.524168532585e+15,
        n_o2: 1.278960754498e+14,
        n_ar: 2.607685026575e+12,
        n_h: 1.248803862117e+11,
        n_n: 8.742553553470e+13,
        n_o_anomalous: 1.866070668449e-03,
    },
    ReferenceRow {
        alt_m: 3.000000000000e+05,
        rho: 3.329444506045e-11,
        temp: 1.109001608620e+03,
        texo: 1.130778214192e+03,
        n_he: 7.532296046657e+12,
        n_o: 9.136907009405e+14,
        n_n2: 1.752781300896e+14,
        n_o2: 4.051596593885e+12,
        n_ar: 3.707792940655e+10,
        n_h: 8.753204317884e+10,
        n_n: 2.633605948289e+13,
        n_o_anomalous: 5.059608582348e+06,
    },
    ReferenceRow {
        alt_m: 4.000000000000e+05,
        rho: 6.059650178061e-12,
        temp: 1.127547972102e+03,
        texo: 1.130778214192e+03,
        n_he: 5.113865028955e+12,
        n_o: 1.989290623180e+14,
        n_n2: 1.231794953020e+13,
        n_o2: 1.948629555681e+11,
        n_ar: 8.386188567106e+08,
        n_h: 7.868900275524e+10,
        n_n: 6.844341510438e+12,
        n_o_anomalous: 1.264445941761e+09,
    },
    ReferenceRow {
        alt_m: 5.000000000000e+05,
        rho: 1.346227709059e-12,
        temp: 1.130271537992e+03,
        texo: 1.130778214192e+03,
        n_he: 3.550928079408e+12,
        n_o: 4.641083522171e+13,
        n_n2: 9.664652715650e+11,
        n_o2: 1.063076413813e+10,
        n_ar: 2.211877242442e+07,
        n_h: 7.175050881209e+10,
        n_n: 1.909258888942e+12,
        n_o_anomalous: 4.132491481638e+09,
    },
    ReferenceRow {
        alt_m: 6.000000000000e+05,
        rho: 3.344156452382e-13,
        temp: 1.130694378795e+03,
        texo: 1.130778214192e+03,
        n_he: 2.495746686536e+12,
        n_o: 1.133168010354e+13,
        n_n2: 8.198590536651e+10,
        n_o2: 6.339773638561e+08,
        n_ar: 6.518750687443e+05,
        n_h: 6.568468274129e+10,
        n_n: 5.559811563104e+11,
        n_o_anomalous: 4.255555467085e+09,
    },
    ReferenceRow {
        alt_m: 7.000000000000e+05,
        rho: 9.264716286363e-14,
        temp: 1.130763614744e+03,
        texo: 1.130778214192e+03,
        n_he: 1.772229371917e+12,
        n_o: 2.881451870332e+12,
        n_n2: 7.465587730199e+09,
        n_o2: 4.099532242727e+07,
        n_ar: 2.125653144915e+04,
        n_h: 6.029504142101e+10,
        n_n: 1.677680201662e+11,
        n_o_anomalous: 3.241077152271e+09,
    },
    ReferenceRow {
        alt_m: 8.000000000000e+05,
        rho: 3.000378620897e-14,
        temp: 1.130775544116e+03,
        texo: 1.130778214192e+03,
        n_he: 1.270645437779e+12,
        n_o: 7.614404718195e+11,
        n_n2: 7.271273265396e+08,
        n_o2: 2.862780514460e+06,
        n_ar: 7.630622347499e+02,
        n_h: 5.548244358205e+10,
        n_n: 5.235764692515e+10,
        n_o_anomalous: 2.294402410868e+09,
    },
    ReferenceRow {
        alt_m: 9.000000000000e+05,
        rho: 1.213038523151e-14,
        temp: 1.130777702385e+03,
        texo: 1.130778214192e+03,
        n_he: 9.194458835245e+11,
        n_o: 2.087585869145e+11,
        n_n2: 7.553109378973e+07,
        n_o2: 2.151818637473e+05,
        n_ar: 3.003184697139e+01,
        n_h: 5.117179396253e+10,
        n_n: 1.687475046394e+10,
        n_o_anomalous: 1.604664647083e+09,
    },
    ReferenceRow {
        alt_m: 1.000000000000e+06,
        rho: 6.240839133382e-15,
        temp: 1.130778111567e+03,
        texo: 1.130778214192e+03,
        n_he: 6.712115540845e+11,
        n_o: 5.928947230117e+10,
        n_n2: 8.345628638197e+06,
        n_o2: 1.735690533821e+04,
        n_ar: 1.290967599736e+00,
        n_h: 4.730029858523e+10,
        n_n: 5.609238699978e+09,
        n_o_anomalous: 1.126695867996e+09,
    },
];

#[derive(Copy, Clone, Debug)]
struct ReferenceRow {
    alt_m: f64,
    rho: f64,
    temp: f64,
    texo: f64,
    n_he: f64,
    n_o: f64,
    n_n2: f64,
    n_o2: f64,
    n_ar: f64,
    n_h: f64,
    n_n: f64,
    n_o_anomalous: f64,
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
#[allow(clippy::similar_names)]
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

    let (rho, temp, texo, n_he, n_o, n_n2, n_o2, n_ar, n_h, n_n, n_o_anomalous);
    if lower_idx == upper_idx {
        rho = lo.rho;
        temp = lo.temp;
        texo = lo.texo;
        n_he = lo.n_he;
        n_o = lo.n_o;
        n_n2 = lo.n_n2;
        n_o2 = lo.n_o2;
        n_ar = lo.n_ar;
        n_h = lo.n_h;
        n_n = lo.n_n;
        n_o_anomalous = lo.n_o_anomalous;
    } else {
        let span = hi.alt_m - lo.alt_m;
        let frac = (alt_m - lo.alt_m) / span;
        rho = log_interp_nonnegative(lo.rho, hi.rho, frac);
        temp = lo.temp + (hi.temp - lo.temp) * frac;
        texo = lo.texo + (hi.texo - lo.texo) * frac;
        n_he = log_interp_nonnegative(lo.n_he, hi.n_he, frac);
        n_o = log_interp_nonnegative(lo.n_o, hi.n_o, frac);
        n_n2 = log_interp_nonnegative(lo.n_n2, hi.n_n2, frac);
        n_o2 = log_interp_nonnegative(lo.n_o2, hi.n_o2, frac);
        n_ar = log_interp_nonnegative(lo.n_ar, hi.n_ar, frac);
        n_h = log_interp_nonnegative(lo.n_h, hi.n_h, frac);
        n_n = log_interp_nonnegative(lo.n_n, hi.n_n, frac);
        n_o_anomalous = log_interp_nonnegative(lo.n_o_anomalous, hi.n_o_anomalous, frac);
    }

    Nrlmsise00Outputs {
        n_he,
        n_o,
        n_n2,
        n_o2,
        n_ar,
        n_h,
        n_n,
        n_o_anomalous,
        mass_density_kg_m3: rho,
        neutral_temperature_k: temp,
        exospheric_temperature_k: texo,
    }
}

fn log_interp_nonnegative(lo: f64, hi: f64, frac: f64) -> f64 {
    if lo > 0.0 && hi > 0.0 {
        (lo.ln() + (hi.ln() - lo.ln()) * frac).exp()
    } else {
        lo + (hi - lo) * frac
    }
}

impl AtmosphereModel for Nrlmsise00Static {
    fn sample(
        &self,
        altitude_geometric_m: f64,
        _time: SimTime,
    ) -> Result<AtmosphereSample, PhysicsError> {
        let outputs = self.evaluate(Nrlmsise00Inputs::mid_conditions(altitude_geometric_m))?;
        outputs_to_sample(outputs)
    }
}

fn outputs_to_sample(outputs: Nrlmsise00Outputs) -> Result<AtmosphereSample, PhysicsError> {
    validate_outputs(outputs)?;
    let temp = outputs.neutral_temperature_k.max(1.0);
    let neutral_number_density = outputs.n_he
        + outputs.n_o
        + outputs.n_n2
        + outputs.n_o2
        + outputs.n_ar
        + outputs.n_h
        + outputs.n_n;
    let pressure = neutral_number_density * BOLTZMANN_J_K * temp;
    let speed_of_sound =
        (NRLMSISE_GAMMA * pressure / outputs.mass_density_kg_m3.max(f64::MIN_POSITIVE)).sqrt();
    AtmosphereSample::new(outputs.mass_density_kg_m3, pressure, temp, speed_of_sound)
}

fn validate_outputs(outputs: Nrlmsise00Outputs) -> Result<(), PhysicsError> {
    let values = [
        outputs.n_he,
        outputs.n_o,
        outputs.n_n2,
        outputs.n_o2,
        outputs.n_ar,
        outputs.n_h,
        outputs.n_n,
        outputs.n_o_anomalous,
        outputs.mass_density_kg_m3,
        outputs.neutral_temperature_k,
        outputs.exospheric_temperature_k,
    ];
    if values.iter().any(|value| !value.is_finite()) {
        return Err(PhysicsError::NonFinite {
            reason: "nrlmsise00 output field is NaN or Inf",
        });
    }
    if outputs.mass_density_kg_m3 <= 0.0
        || outputs.neutral_temperature_k <= 0.0
        || outputs.exospheric_temperature_k <= 0.0
    {
        return Err(PhysicsError::InvalidParameter {
            reason: "nrlmsise00 output density and temperatures must be positive",
        });
    }
    if values[..8].iter().any(|value| *value < 0.0) {
        return Err(PhysicsError::InvalidParameter {
            reason: "nrlmsise00 species number densities must be non-negative",
        });
    }
    Ok(())
}

/// NRLMSISE-00 full coefficient-based path.
///
/// `evaluate` consumes a complete [`Nrlmsise00Inputs`] query. The
/// [`AtmosphereModel`] implementation uses `base_inputs` as a
/// scenario-declared deterministic column and replaces only altitude
/// for each sample; this preserves the existing altitude/time trait
/// while allowing scenario authors to pin date, location, F10.7, and
/// Ap for boost/re-entry drag studies.
#[derive(Copy, Clone, Debug)]
pub struct Nrlmsise00Full {
    base_inputs: Nrlmsise00Inputs,
}

impl Default for Nrlmsise00Full {
    fn default() -> Self {
        Self::mid_conditions()
    }
}

impl Nrlmsise00Full {
    /// Construct a full coefficient-path model with mid-condition
    /// defaults for all non-altitude inputs.
    #[must_use]
    pub const fn mid_conditions() -> Self {
        Self {
            base_inputs: Nrlmsise00Inputs::mid_conditions(0.0),
        }
    }

    /// Construct a full coefficient-path model from a scenario
    /// default query. The altitude in `base_inputs` is ignored by
    /// [`AtmosphereModel::sample`] and replaced by the sampled
    /// altitude.
    ///
    /// # Errors
    ///
    /// Returns the same validation errors as
    /// [`Nrlmsise00Inputs::validate_full_path`].
    pub fn new(base_inputs: Nrlmsise00Inputs) -> Result<Self, PhysicsError> {
        base_inputs.validate_full_path()?;
        Ok(Self { base_inputs })
    }

    /// Scenario/default inputs used by the trait-based sampler.
    #[must_use]
    pub const fn base_inputs(self) -> Nrlmsise00Inputs {
        self.base_inputs
    }

    /// Evaluate the full-input NRLMSISE-00 path.
    ///
    /// The low-level coefficient calculation uses OpenBMP-local
    /// static coefficient tables generated from the public
    /// NASA/CCMC archived `nrlmsise-00_data.c`. Output number
    /// densities are converted from `cm^-3` to `m^-3` to match
    /// OpenBMP's atmosphere API.
    ///
    /// # Errors
    ///
    /// Returns input-validation errors from
    /// [`Nrlmsise00Inputs::validate_full_path`] or output validation
    /// errors if the coefficient evaluator returns non-finite or
    /// physically invalid values.
    pub fn evaluate(self, inputs: Nrlmsise00Inputs) -> Result<Nrlmsise00Outputs, PhysicsError> {
        inputs.validate_full_path()?;
        let query = Nrlmsise00LowLevelInput {
            day_of_year: u32::from(inputs.day_of_year),
            ut_seconds: inputs.utc_seconds,
            altitude_km: inputs.altitude_m / 1000.0,
            latitude_deg: inputs.latitude_rad.to_degrees(),
            longitude_deg: inputs.longitude_rad.to_degrees(),
            local_solar_time_hours: inputs.local_apparent_solar_time_hours,
            f107_daily: inputs.f107_yesterday,
            f107_avg: inputs.f107_average_81day,
            ap_daily: inputs.ap_average,
            ap_array: [inputs.ap_average; 7],
        };
        let (density_cm3, exospheric_temperature_k, neutral_temperature_k) =
            nrlmsise00_model::compute(&query);
        let outputs = Nrlmsise00Outputs {
            n_he: density_cm3[0] * CM3_TO_M3,
            n_o: density_cm3[1] * CM3_TO_M3,
            n_n2: density_cm3[2] * CM3_TO_M3,
            n_o2: density_cm3[3] * CM3_TO_M3,
            n_ar: density_cm3[4] * CM3_TO_M3,
            n_h: density_cm3[6] * CM3_TO_M3,
            n_n: density_cm3[7] * CM3_TO_M3,
            n_o_anomalous: density_cm3[8] * CM3_TO_M3,
            mass_density_kg_m3: density_cm3[5] * 1000.0,
            neutral_temperature_k,
            exospheric_temperature_k,
        };
        validate_outputs(outputs)?;
        Ok(outputs)
    }
}

impl AtmosphereModel for Nrlmsise00Full {
    fn sample(
        &self,
        altitude_geometric_m: f64,
        _time: SimTime,
    ) -> Result<AtmosphereSample, PhysicsError> {
        let inputs = self.base_inputs.with_altitude_m(altitude_geometric_m);
        let outputs = self.evaluate(inputs)?;
        outputs_to_sample(outputs)
    }
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::missing_panics_doc,
    clippy::similar_names
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn static_path_matches_reference_at_table_points() {
        // Spot-check selected published-reference altitudes that the
        // table pins exactly (interpolation must reproduce them to
        // machine epsilon).
        let m = Nrlmsise00Static;
        for row in REFERENCE_TABLE {
            let outputs = m
                .evaluate(Nrlmsise00Inputs::mid_conditions(row.alt_m))
                .unwrap();
            assert_relative_eq!(outputs.mass_density_kg_m3, row.rho, max_relative = 1e-12);
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
                .evaluate(Nrlmsise00Inputs::mid_conditions(f64::from(alt)))
                .unwrap()
                .mass_density_kg_m3;
            assert!(
                next < prev,
                "density not monotone at {alt}: prev={prev} next={next}"
            );
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
    fn full_path_returns_finite_sample() {
        let m = Nrlmsise00Full::default();
        let sample = m.sample(200_000.0, SimTime::ZERO).unwrap();
        assert!(sample.density_kg_m3 > 0.0);
        assert!(sample.pressure_pa > 0.0);
        assert!(sample.temperature_k > 0.0);
        assert!(sample.speed_of_sound_m_s > 0.0);
    }

    #[test]
    fn full_inputs_validate_mid_conditions() {
        Nrlmsise00Inputs::mid_conditions(200_000.0)
            .validate_full_path()
            .unwrap();
    }

    #[test]
    fn full_inputs_reject_out_of_range_latitude() {
        let mut inputs = Nrlmsise00Inputs::mid_conditions(200_000.0);
        inputs.latitude_rad = 100.0_f64.to_radians();
        assert!(matches!(
            inputs.validate_full_path(),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn full_evaluate_rejects_bad_inputs_before_model_call() {
        let m = Nrlmsise00Full::default();
        let mut inputs = Nrlmsise00Inputs::mid_conditions(200_000.0);
        inputs.ap_average = -1.0;
        assert!(matches!(
            m.evaluate(inputs),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn full_evaluate_valid_inputs_returns_composition() {
        let m = Nrlmsise00Full::default();
        let outputs = m
            .evaluate(Nrlmsise00Inputs::mid_conditions(200_000.0))
            .unwrap();
        assert!(outputs.mass_density_kg_m3 > 0.0);
        assert!(outputs.neutral_temperature_k > 0.0);
        assert!(outputs.exospheric_temperature_k > 0.0);
        assert!(outputs.n_o > 0.0);
        assert!(outputs.n_n2 > 0.0);
    }

    #[test]
    fn full_path_matches_c_release_reference_case() {
        let m = Nrlmsise00Full::default();
        let mut inputs = Nrlmsise00Inputs::mid_conditions(400_000.0);
        inputs.day_of_year = 172;
        inputs.utc_seconds = 29_000.0;
        inputs.latitude_rad = 60.0_f64.to_radians();
        inputs.longitude_rad = (-70.0_f64).to_radians();
        inputs.local_apparent_solar_time_hours = 16.0;
        inputs.f107_average_81day = 150.0;
        inputs.f107_yesterday = 150.0;
        inputs.ap_average = 4.0;

        let outputs = m.evaluate(inputs).unwrap();
        assert_relative_eq!(outputs.n_he, 6.665_177e11, max_relative = 1e-3);
        assert_relative_eq!(outputs.n_o, 1.138_806e14, max_relative = 1e-3);
        assert_relative_eq!(outputs.n_n2, 1.998_211e13, max_relative = 1e-3);
        assert_relative_eq!(outputs.n_o2, 4.022_764e11, max_relative = 1e-3);
        assert_relative_eq!(outputs.n_ar, 3.557_465e9, max_relative = 1e-3);
        assert_relative_eq!(outputs.n_h, 3.475_312e10, max_relative = 1e-3);
        assert_relative_eq!(outputs.n_n, 4.095_913e12, max_relative = 1e-3);
        assert_relative_eq!(outputs.n_o_anomalous, 2.667_273e10, max_relative = 1e-3);
        assert_relative_eq!(
            outputs.mass_density_kg_m3,
            4.074_714e-12,
            max_relative = 1e-3
        );
        assert_relative_eq!(
            outputs.exospheric_temperature_k,
            1.250_540e3,
            max_relative = 1e-3
        );
        assert_relative_eq!(
            outputs.neutral_temperature_k,
            1.241_416e3,
            max_relative = 1e-3
        );
    }

    #[test]
    fn full_path_responds_to_solar_activity() {
        let m = Nrlmsise00Full::default();
        let mut quiet = Nrlmsise00Inputs::mid_conditions(400_000.0);
        quiet.f107_average_81day = 70.0;
        quiet.f107_yesterday = 70.0;
        quiet.ap_average = 4.0;
        let mut active = quiet;
        active.f107_average_81day = 250.0;
        active.f107_yesterday = 250.0;
        active.ap_average = 50.0;

        let quiet_rho = m.evaluate(quiet).unwrap().mass_density_kg_m3;
        let active_rho = m.evaluate(active).unwrap().mass_density_kg_m3;
        assert!(
            active_rho > quiet_rho,
            "active solar/geomagnetic conditions should increase density: quiet={quiet_rho} active={active_rho}"
        );
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
    fn species_number_densities_are_non_negative_at_table_rows() {
        for row in REFERENCE_TABLE {
            assert!(row.n_he >= 0.0);
            assert!(row.n_o >= 0.0);
            assert!(row.n_n2 >= 0.0);
            assert!(row.n_o2 >= 0.0);
            assert!(row.n_ar >= 0.0);
            assert!(row.n_h >= 0.0);
            assert!(row.n_n >= 0.0);
            assert!(row.n_o_anomalous >= 0.0);
            assert!(row.rho > 0.0);
        }
    }

    #[test]
    fn static_path_determinism_two_runs_byte_identical() {
        let m = Nrlmsise00Static;
        for alt in [50_000.0_f64, 200_000.0, 500_000.0] {
            let a = m.evaluate(Nrlmsise00Inputs::mid_conditions(alt)).unwrap();
            let b = m.evaluate(Nrlmsise00Inputs::mid_conditions(alt)).unwrap();
            assert_eq!(
                a.mass_density_kg_m3.to_bits(),
                b.mass_density_kg_m3.to_bits()
            );
            assert_eq!(
                a.neutral_temperature_k.to_bits(),
                b.neutral_temperature_k.to_bits()
            );
            assert_eq!(a.n_o.to_bits(), b.n_o.to_bits());
        }
    }
}
