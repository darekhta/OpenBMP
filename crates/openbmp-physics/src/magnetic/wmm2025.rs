//! WMM 2025 (NOAA NCEI / NGA / UK DGC, December 2024 release).
//!
//! Direct port of the NOAA reference algorithm: Gauss-normalized
//! associated Legendre recursion with Schmidt-conversion factors
//! pre-multiplied into the Gauss coefficients. Equivalent to the
//! reference C / pyGeoMag implementations.
//!
//! Coefficient table is a verbatim port of `WMM.COF` shipped at
//! `data/magnetic/WMM.COF` (SHA-256 pinned in
//! `data/magnetic/provenance.md`); the upstream
//! `WMM2025_TestValues.txt` 100-row reference set drives the
//! validation tests below.
//!
//! # Validity envelope
//!
//! WMM 2025 is authoritative for decimal years `[2025.0, 2030.0)`.
//! Outside this range the model fails closed with
//! [`PhysicsError::OutOfEnvelope`].
//!
//! # Convention
//!
//! - Input: geodetic latitude (rad, +north), longitude (rad,
//!   +east), height above the WGS-84 ellipsoid (m), and a
//!   simulation time interpreted as elapsed seconds since the
//!   scenario decimal-year epoch supplied at construction.
//! - Output: magnetic flux density vector in geodetic NED
//!   (`+north`, `+east`, `+down`), in nanotesla.
//!
//! # Determinism
//!
//! Pure `f64` arithmetic; no FMA; locked operand order on the
//! recursion and the field summation. Cross-platform last-bit
//! determinism for `sin` / `cos` / `sqrt` is the same disposition
//! as the rest of `openbmp-physics` — Linux CI gate is the only proof
//! point.

#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_wrap,
    clippy::similar_names,
    clippy::many_single_char_names,
    clippy::too_many_lines,
    clippy::neg_cmp_op_on_partial_ord,
    clippy::inconsistent_digit_grouping,
    clippy::unreadable_literal
)] // Phase-3.10: WMM-reference algorithm with locked-name `a`, `b`,
// `c`, `q`, `r`, `d`, `bt`, `bp`, `br` variables matches the
// NOAA C / pyGeoMag source for traceability.

use nalgebra::Vector3;
use openbmp_core::{Eci, FrameContext, Position3, SimTime, WGS84_A_M, WGS84_ECCENTRICITY_SQUARED};

use super::{EarthDipoleField, MagneticFieldEci, MagneticModel};
use crate::error::PhysicsError;

mod coefficients;

use coefficients::{COEFFS, EPOCH_DECIMAL_YEAR, MAX_DEGREE, VALIDITY_END_DECIMAL_YEAR};

/// Geomagnetic reference radius `a` used in WMM (m). Distinct from
/// the WGS-84 semi-major axis; this is the constant that appears in
/// the `(a/r)^{n+1}` factor of the spherical-harmonic series.
pub const GEOMAGNETIC_REFERENCE_RADIUS_M: f64 = 6_371_200.0;

/// Seconds per Julian year (365.25 days). Decimal-year deltas use
/// this conversion factor.
pub const SECONDS_PER_JULIAN_YEAR: f64 = 365.25 * 86_400.0;

/// One coefficient row from `WMM.COF`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct CofCoefficient {
    /// Degree.
    pub n: u8,
    /// Order.
    pub m: u8,
    /// `g_n^m` at epoch (nT).
    pub g_nt: f64,
    /// `h_n^m` at epoch (nT). Zero for `m = 0`.
    pub h_nt: f64,
    /// `ġ_n^m` (nT / year).
    pub g_dot_nt_per_year: f64,
    /// `ḣ_n^m` (nT / year). Zero for `m = 0`.
    pub h_dot_nt_per_year: f64,
}

/// Wmm2025 model. Holds the scenario decimal-year epoch + the
/// pre-computed Gauss-Schmidt scratch tables (`c`, `cd`, `k`,
/// `snorm`).
#[derive(Clone, Debug)]
pub struct Wmm2025 {
    start_decimal_year: f64,
    /// Effective Schmidt-multiplied `g_n^m` lookup, indexed by
    /// `[m][n]` (the WMM-reference orientation).
    c: Vec<Vec<f64>>,
    /// Effective Schmidt-multiplied `h_n^m` lookup, stored at
    /// `[n][m-1]` (the WMM-reference orientation; only `m ≥ 1`).
    /// `cd_h` mirrors for the secular variation.
    h: Vec<Vec<f64>>,
    /// Secular variation `ġ_n^m`.
    cd: Vec<Vec<f64>>,
    /// Secular variation `ḣ_n^m`.
    cd_h: Vec<Vec<f64>>,
    /// Gauss-recursion coefficient `K[m][n] = ((n-1)² - m²) /
    /// ((2n-1)(2n-3))`.
    k: Vec<Vec<f64>>,
}

impl Wmm2025 {
    /// Construct a model anchored at a given scenario decimal-year
    /// epoch.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::OutOfEnvelope`] when
    /// `start_decimal_year` is outside `[2025.0, 2030.0)`.
    pub fn new_for_decimal_year(start_decimal_year: f64) -> Result<Self, PhysicsError> {
        if !start_decimal_year.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "WMM 2025 start_decimal_year is not finite",
            });
        }
        if !(EPOCH_DECIMAL_YEAR..VALIDITY_END_DECIMAL_YEAR).contains(&start_decimal_year) {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "WMM 2025 is valid only for decimal years in [2025.0, 2030.0)",
            });
        }
        let size = MAX_DEGREE + 1;
        let mut c = vec![vec![0.0_f64; size]; size];
        let mut cd = vec![vec![0.0_f64; size]; size];
        let mut h = vec![vec![0.0_f64; size]; size];
        let mut cd_h = vec![vec![0.0_f64; size]; size];
        let mut k = vec![vec![0.0_f64; size]; size];
        let mut snorm = vec![0.0_f64; size * size];

        // Stage raw g, h coefficients.
        for coeff in COEFFS {
            let n = coeff.n as usize;
            let m = coeff.m as usize;
            c[m][n] = coeff.g_nt;
            cd[m][n] = coeff.g_dot_nt_per_year;
            if m != 0 {
                h[n][m - 1] = coeff.h_nt;
                cd_h[n][m - 1] = coeff.h_dot_nt_per_year;
            }
        }

        // Schmidt → effective conversion (per the WMM reference
        // implementation): pre-multiply g, h by the snorm factors so
        // the per-step recursion can produce un-normalized P and the
        // accumulator's product still yields Schmidt-equivalent
        // contributions. Locked operand order matches the WMM C /
        // pyGeoMag recursion.
        snorm[0] = 1.0;
        for n in 1..size {
            let n_i = n as i64;
            snorm[n] = snorm[n - 1] * ((2 * n_i - 1) as f64) / (n_i as f64);
            let mut j = 2.0;
            for m in 0..=n {
                let m_i = m as i64;
                k[m][n] = (((n_i - 1) * (n_i - 1) - m_i * m_i) as f64)
                    / ((2 * n_i - 1) as f64 * (2 * n_i - 3) as f64);
                if m > 0 {
                    let flnmj = ((n_i - m_i + 1) as f64 * j) / ((n_i + m_i) as f64);
                    snorm[n + m * size] = snorm[n + (m - 1) * size] * flnmj.sqrt();
                    j = 1.0;
                    h[n][m - 1] *= snorm[n + m * size];
                    cd_h[n][m - 1] *= snorm[n + m * size];
                }
                c[m][n] *= snorm[n + m * size];
                cd[m][n] *= snorm[n + m * size];
            }
        }
        // The reference implementations special-case k[1][1] = 0 (it
        // sits outside the recursion's valid index range and is
        // never multiplied into a real P[n-2][m] that exists, but the
        // WMM C source nulls it explicitly so we match for parity).
        k[1][1] = 0.0;
        Ok(Self {
            start_decimal_year,
            c,
            h,
            cd,
            cd_h,
            k,
        })
    }

    /// Scenario-start decimal year supplied at construction.
    #[must_use]
    pub fn start_decimal_year(&self) -> f64 {
        self.start_decimal_year
    }

    /// Read-only access to the in-source coefficient table. Used by
    /// the data-pin verification test.
    #[must_use]
    pub fn coefficients() -> &'static [CofCoefficient] {
        COEFFS
    }

    /// Evaluate the field at a geodetic point.
    ///
    /// `latitude_rad` and `longitude_rad` are WGS-84 geodetic
    /// (positive north / east). `height_m` is height above the
    /// WGS-84 ellipsoid. `time` is simulation time elapsed since
    /// scenario start.
    ///
    /// Returns the geodetic-NED magnetic flux density (nT).
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::OutOfEnvelope`] when the resolved
    /// decimal-year falls outside `[2025.0, 2030.0)` and
    /// [`PhysicsError::NonFinite`] on non-finite intermediate
    /// arithmetic.
    pub fn field_geodetic_ned_nt(
        &self,
        latitude_rad: f64,
        longitude_rad: f64,
        height_m: f64,
        time: SimTime,
    ) -> Result<Vector3<f64>, PhysicsError> {
        let elapsed_years = time.as_seconds() / SECONDS_PER_JULIAN_YEAR;
        let decimal_year = self.start_decimal_year + elapsed_years;
        if !decimal_year.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "WMM 2025 evaluation time is not finite",
            });
        }
        if !(EPOCH_DECIMAL_YEAR..VALIDITY_END_DECIMAL_YEAR).contains(&decimal_year) {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "WMM 2025 is valid only for decimal years in [2025.0, 2030.0)",
            });
        }
        if !latitude_rad.is_finite() || !longitude_rad.is_finite() || !height_m.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "WMM 2025: non-finite geodetic input",
            });
        }
        // WMM-reference geodetic → spherical conversion.
        let (sin_phi, cos_phi) = latitude_rad.sin_cos();
        let (sin_lambda, cos_lambda) = longitude_rad.sin_cos();
        let a = WGS84_A_M;
        let a2 = a * a;
        let b2 = a2 * (1.0 - WGS84_ECCENTRICITY_SQUARED);
        let c2 = a2 - b2;
        let a4 = a2 * a2;
        let b4 = b2 * b2;
        // Note: in the WMM reference algorithm `c4` is the *symbol*
        // `a⁴ − b⁴`, not `(c²)²`. The substitution simplifies the
        // `r²` formula in this block.
        let c4 = a4 - b4;
        let srlat2 = sin_phi * sin_phi;
        let crlat2 = cos_phi * cos_phi;
        let q = (a2 - c2 * srlat2).sqrt();
        let q1 = height_m * q;
        let q2_factor = (q1 + a2) / (q1 + b2);
        let q2 = q2_factor * q2_factor;
        let ct = sin_phi / (q2 * crlat2 + srlat2).sqrt();
        let st = (1.0 - ct * ct).sqrt();
        let r2 = height_m * height_m + 2.0 * q1 + (a4 - c4 * srlat2) / (q * q);
        let r = r2.sqrt();
        if !(r > 0.0) {
            return Err(PhysicsError::NonFinite {
                reason: "WMM 2025: geocentric radius is zero",
            });
        }
        let d = (a2 * crlat2 + b2 * srlat2).sqrt();
        let ca = (height_m + d) / r;
        let sa = c2 * cos_phi * sin_phi / (r * d);

        // sin(mλ) / cos(mλ) tables.
        let size = MAX_DEGREE + 1;
        let mut sp = vec![0.0_f64; size];
        let mut cp = vec![0.0_f64; size];
        sp[0] = 0.0;
        cp[0] = 1.0;
        sp[1] = sin_lambda;
        cp[1] = cos_lambda;
        for m in 2..size {
            sp[m] = sp[1] * cp[m - 1] + cp[1] * sp[m - 1];
            cp[m] = cp[1] * cp[m - 1] - sp[1] * sp[m - 1];
        }

        // Per-evaluation Legendre / derivative tables. `p` is keyed
        // `n + m * size` to match the WMM-reference flat layout; `dp`
        // is `[m][n]`.
        let mut p = vec![0.0_f64; size * size];
        let mut dp = vec![vec![0.0_f64; size]; size];
        let mut pp = vec![0.0_f64; size];
        p[0] = 1.0;
        pp[0] = 1.0;

        let dt_years = decimal_year - EPOCH_DECIMAL_YEAR;
        let aor = GEOMAGNETIC_REFERENCE_RADIUS_M / r;
        let mut ar = aor * aor;
        let (mut br, mut bt, mut bp, mut bpp) = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
        for n in 1..size {
            ar *= aor;
            for m in 0..=n {
                // Gauss-normalized Legendre recursion for P, dP/dθ.
                if n == m {
                    p[n + m * size] = st * p[(n - 1) + (m - 1) * size];
                    dp[m][n] = st * dp[m - 1][n - 1] + ct * p[(n - 1) + (m - 1) * size];
                } else if n == 1 && m == 0 {
                    p[n + m * size] = ct * p[(n - 1) + m * size];
                    dp[m][n] = ct * dp[m][n - 1] - st * p[(n - 1) + m * size];
                } else if n > 1 && m != n {
                    if m > n - 2 {
                        // Sentinel out-of-range index zeroed per the
                        // WMM reference convention.
                        p[(n - 2) + m * size] = 0.0;
                        dp[m][n - 2] = 0.0;
                    }
                    p[n + m * size] =
                        ct * p[(n - 1) + m * size] - self.k[m][n] * p[(n - 2) + m * size];
                    dp[m][n] = ct * dp[m][n - 1]
                        - st * p[(n - 1) + m * size]
                        - self.k[m][n] * dp[m][n - 2];
                }

                // Time-adjusted coefficients.
                let tcg = self.c[m][n] + dt_years * self.cd[m][n];
                let tch = if m != 0 {
                    self.h[n][m - 1] + dt_years * self.cd_h[n][m - 1]
                } else {
                    0.0
                };

                // Accumulate field components in geocentric NED.
                let par = ar * p[n + m * size];
                let (temp1, temp2) = if m == 0 {
                    (tcg * cp[m], tcg * sp[m])
                } else {
                    (tcg * cp[m] + tch * sp[m], tcg * sp[m] - tch * cp[m])
                };
                bt -= ar * temp1 * dp[m][n];
                bp += (m as f64) * temp2 * par;
                br += ((n + 1) as f64) * temp1 * par;

                // North / south geographic-pole limit for Bφ.
                if st == 0.0 && m == 1 {
                    if n == 1 {
                        pp[n] = pp[n - 1];
                    } else {
                        pp[n] = ct * pp[n - 1] - self.k[m][n] * pp[n - 2];
                    }
                    let parp = ar * pp[n];
                    bpp += (m as f64) * temp2 * parp;
                }
            }
        }

        let by = if st == 0.0 { bpp } else { bp / st };
        // Rotate (Bt, Br) from geocentric to geodetic.
        let bx = -bt * ca - br * sa;
        let bz = bt * sa - br * ca;
        if !bx.is_finite() || !by.is_finite() || !bz.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "WMM 2025: non-finite field component",
            });
        }
        Ok(Vector3::new(bx, by, bz))
    }
}

impl MagneticModel for Wmm2025 {
    fn field_ned_nt(
        &self,
        position_eci: Position3<Eci>,
        time: SimTime,
    ) -> Result<Vector3<f64>, PhysicsError> {
        // Phase-3.10: ECI → ECEF for the toy fixed-earth profile is
        // the identity. WGS-84 rotation profiles are the runner's
        // job to disambiguate at scenario load.
        let _ = FrameContext::toy_fixed_earth();
        let (lat, lon, h) = ecef_to_geodetic(position_eci.vector);
        self.field_geodetic_ned_nt(lat, lon, h, time)
    }
}

impl MagneticFieldEci for Wmm2025 {
    fn field_eci_nt(&self, position_eci_m: Vector3<f64>, time: SimTime) -> Vector3<f64> {
        let fallback = || EarthDipoleField::default().field_eci_nt(position_eci_m, time);
        let (lat, lon, h) = ecef_to_geodetic(position_eci_m);
        let Ok(ned) = self.field_geodetic_ned_nt(lat, lon, h, time) else {
            return fallback();
        };
        let eci = ned_to_fixed_earth(lat, lon, ned);
        if eci.x.is_finite() && eci.y.is_finite() && eci.z.is_finite() {
            eci
        } else {
            fallback()
        }
    }
}

// ---------------------------------------------------------------------
// ECEF → geodetic
// ---------------------------------------------------------------------

/// Closed-form WGS-84 ECEF → geodetic conversion (Bowring 1985).
/// Returns `(latitude_rad, longitude_rad, height_m)`.
pub(crate) fn ecef_to_geodetic(ecef: Vector3<f64>) -> (f64, f64, f64) {
    let x = ecef.x;
    let y = ecef.y;
    let z = ecef.z;
    let p = (x * x + y * y).sqrt();
    let lon = y.atan2(x);
    let a = WGS84_A_M;
    let e2 = WGS84_ECCENTRICITY_SQUARED;
    let b = a * (1.0 - e2).sqrt();
    let ep2 = e2 / (1.0 - e2);
    let theta = (z * a).atan2(p * b);
    let sin_theta = theta.sin();
    let cos_theta = theta.cos();
    let lat = (z + ep2 * b * sin_theta * sin_theta * sin_theta)
        .atan2(p - e2 * a * cos_theta * cos_theta * cos_theta);
    let n = a / (1.0 - e2 * lat.sin() * lat.sin()).sqrt();
    let h = if lat.cos().abs() > 1.0e-12 {
        p / lat.cos() - n
    } else {
        z.abs() - b
    };
    (lat, lon, h)
}

fn ned_to_fixed_earth(latitude_rad: f64, longitude_rad: f64, ned_nt: Vector3<f64>) -> Vector3<f64> {
    let (sin_lat, cos_lat) = latitude_rad.sin_cos();
    let (sin_lon, cos_lon) = longitude_rad.sin_cos();
    let north = Vector3::new(-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat);
    let east = Vector3::new(-sin_lon, cos_lon, 0.0);
    let down = Vector3::new(-cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat);
    ned_nt.x * north + ned_nt.y * east + ned_nt.z * down
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::panic
)]
mod tests {
    use super::*;
    use openbmp_core::SimTime;

    const WMM2025_TEST_VALUES: &str =
        include_str!("../../../../data/magnetic/WMM2025_TestValues.txt");
    const WMM_REFERENCE_TOLERANCE_NT: f64 = 5.0;
    const WMM_REFERENCE_ROW_COUNT: usize = 100;

    #[derive(Debug)]
    struct WmmReferenceRow {
        line_no: usize,
        decimal_year: f64,
        altitude_km: f64,
        latitude_deg: f64,
        longitude_deg: f64,
        x_nt: f64,
        y_nt: f64,
        z_nt: f64,
    }

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    fn parse_reference_rows() -> Vec<WmmReferenceRow> {
        let mut rows = Vec::new();
        for (line_idx, line) in WMM2025_TEST_VALUES.lines().enumerate() {
            let line_no = line_idx + 1;
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }

            let fields: Vec<&str> = line.split_whitespace().collect();
            assert_eq!(
                fields.len(),
                18,
                "WMM2025_TestValues.txt line {line_no} has unexpected field count: {line}"
            );

            let parse = |idx: usize, name: &str| -> f64 {
                fields[idx].parse::<f64>().unwrap_or_else(|err| {
                    panic!("WMM2025_TestValues.txt line {line_no}: failed to parse {name}: {err}")
                })
            };

            rows.push(WmmReferenceRow {
                line_no,
                decimal_year: parse(0, "decimal year"),
                altitude_km: parse(1, "altitude"),
                latitude_deg: parse(2, "latitude"),
                longitude_deg: parse(3, "longitude"),
                x_nt: parse(7, "X"),
                y_nt: parse(8, "Y"),
                z_nt: parse(9, "Z"),
            });
        }
        rows
    }

    #[test]
    fn coefficient_count_is_90() {
        assert_eq!(COEFFS.len(), 90);
    }

    #[test]
    fn coefficients_are_lex_sorted() {
        for w in COEFFS.windows(2) {
            assert!(
                (w[0].n, w[0].m) < (w[1].n, w[1].m),
                "WMM 2025 coefficients must be sorted by (n, m); violation at {:?} → {:?}",
                (w[0].n, w[0].m),
                (w[1].n, w[1].m),
            );
        }
    }

    #[test]
    fn h_is_zero_for_zonal_terms() {
        for c in COEFFS {
            if c.m == 0 {
                assert_eq!(c.h_nt, 0.0, "h must be 0 for m = 0 (n = {})", c.n);
                assert_eq!(c.h_dot_nt_per_year, 0.0);
            }
        }
    }

    #[test]
    fn rejects_out_of_envelope_decimal_year() {
        assert!(Wmm2025::new_for_decimal_year(2024.99).is_err());
        assert!(Wmm2025::new_for_decimal_year(2030.0).is_err());
        assert!(Wmm2025::new_for_decimal_year(f64::NAN).is_err());
        assert!(Wmm2025::new_for_decimal_year(2025.0).is_ok());
        assert!(Wmm2025::new_for_decimal_year(2027.5).is_ok());
        assert!(Wmm2025::new_for_decimal_year(2029.999).is_ok());
    }

    #[test]
    fn rejects_evaluation_past_validity_end() {
        let m = Wmm2025::new_for_decimal_year(2029.5).unwrap();
        let dt_seconds = 1.0 * SECONDS_PER_JULIAN_YEAR;
        let result = m.field_geodetic_ned_nt(0.0, 0.0, 0.0, SimTime::from_seconds(dt_seconds));
        assert!(matches!(result, Err(PhysicsError::OutOfEnvelope { .. })));
    }

    #[test]
    fn matches_all_noaa_reference_test_values() {
        let rows = parse_reference_rows();
        assert_eq!(rows.len(), WMM_REFERENCE_ROW_COUNT);

        let model = Wmm2025::new_for_decimal_year(EPOCH_DECIMAL_YEAR).unwrap();
        for row in rows {
            let elapsed_years = row.decimal_year - EPOCH_DECIMAL_YEAR;
            let v = model
                .field_geodetic_ned_nt(
                    row.latitude_deg.to_radians(),
                    row.longitude_deg.to_radians(),
                    row.altitude_km * 1_000.0,
                    SimTime::from_seconds(elapsed_years * SECONDS_PER_JULIAN_YEAR),
                )
                .unwrap();

            assert!(
                approx_eq(v.x, row.x_nt, WMM_REFERENCE_TOLERANCE_NT),
                "line {} X mismatch: got {}, expected {}, diff {}",
                row.line_no,
                v.x,
                row.x_nt,
                v.x - row.x_nt
            );
            assert!(
                approx_eq(v.y, row.y_nt, WMM_REFERENCE_TOLERANCE_NT),
                "line {} Y mismatch: got {}, expected {}, diff {}",
                row.line_no,
                v.y,
                row.y_nt,
                v.y - row.y_nt
            );
            assert!(
                approx_eq(v.z, row.z_nt, WMM_REFERENCE_TOLERANCE_NT),
                "line {} Z mismatch: got {}, expected {}, diff {}",
                row.line_no,
                v.z,
                row.z_nt,
                v.z - row.z_nt
            );
        }
    }

    #[test]
    fn determinism_byte_stable_replay() {
        let m = Wmm2025::new_for_decimal_year(2026.0).unwrap();
        let lat = 0.5;
        let lon = 1.0;
        let h = 5_000.0;
        let a = m
            .field_geodetic_ned_nt(lat, lon, h, SimTime::from_seconds(100.0))
            .unwrap();
        let b = m
            .field_geodetic_ned_nt(lat, lon, h, SimTime::from_seconds(100.0))
            .unwrap();
        assert_eq!(a.x.to_bits(), b.x.to_bits());
        assert_eq!(a.y.to_bits(), b.y.to_bits());
        assert_eq!(a.z.to_bits(), b.z.to_bits());
    }

    #[test]
    fn ecef_to_geodetic_round_trip_at_equator() {
        let p = Vector3::new(WGS84_A_M, 0.0, 0.0);
        let (lat, lon, h) = ecef_to_geodetic(p);
        assert!(approx_eq(lat, 0.0, 1.0e-12));
        assert!(approx_eq(lon, 0.0, 1.0e-12));
        assert!(approx_eq(h, 0.0, 1.0e-6));
    }

    #[test]
    fn magnetic_field_eci_rotates_ned_on_fixed_earth_profile() {
        let model = Wmm2025::new_for_decimal_year(EPOCH_DECIMAL_YEAR).unwrap();
        let position = Vector3::new(WGS84_A_M, 0.0, 0.0);
        let ned = model
            .field_geodetic_ned_nt(0.0, 0.0, 0.0, SimTime::ZERO)
            .unwrap();
        let eci = model.field_eci_nt(position, SimTime::ZERO);
        assert!(approx_eq(eci.x, -ned.z, 1.0e-9));
        assert!(approx_eq(eci.y, ned.y, 1.0e-9));
        assert!(approx_eq(eci.z, ned.x, 1.0e-9));
    }
}
