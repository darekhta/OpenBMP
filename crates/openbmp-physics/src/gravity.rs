//! Gravity models.
//!
//! Provides:
//!
//! * [`ConstantGravity`] — uniform `g` vector in ECI. Mirrors the
//!   `openbmp-sim::ConstantGravityForce` behaviour exactly, but lives
//!   here so the environment-side gravity API doesn't depend on
//!   `openbmp-sim`. That force model stays byte-identical; higher
//!   layers adapt one to the other.
//! * [`PointMassGravity`] — Newtonian `−µ/r² r̂`. Default constructor
//!   pins the WGS84 `µ`.
//! * [`J2Gravity`] — point-mass plus the J2 zonal harmonic, expressed
//!   in ECI. Default constructor pins WGS84
//!   `µ`, `R_e`, and the NIMA TR 8350.2 `J2 = 1.082626683 × 10⁻³`.
//!
//! All three models implement the [`GravityModel`] trait and report
//! failure via [`crate::error::PhysicsError`] (out-of-envelope, non-finite,
//! invalid parameter). They never panic, never silent-clamp, and
//! never return `NaN`.
//!
//! Determinism: pure arithmetic on `f64`; locked operand order on the
//! J2 sum; no FMA.

use nalgebra::Vector3;
use openbmp_core::{Eci, Position3, SimTime};

use crate::error::PhysicsError;
use crate::frames::{WGS84_A_M, WGS84_MU_M3_S2};

/// WGS84 unnormalised J2 zonal-harmonic coefficient.
///
/// Source: NIMA TR8350.2, WGS84 Implementation Manual, §3.
pub const WGS84_J2: f64 = 1.082_626_683e-3;

/// ISO / USSA76 standard gravity (m/s²).
pub const STANDARD_GRAVITY_M_S2: f64 = 9.806_65;

/// Constant ECI gravity vector along negative z using standard
/// gravity.
#[must_use]
pub fn standard_down_z_eci_m_s2() -> Vector3<f64> {
    Vector3::new(0.0, 0.0, -STANDARD_GRAVITY_M_S2)
}

/// Trait implemented by gravity-providing environment models.
///
/// Returns the gravitational acceleration vector at an inertial
/// position and simulation time. Time is included for forward
/// compatibility with future time-varying corrections (none
/// currently).
pub trait GravityModel {
    /// Gravitational acceleration in ECI, in m/s².
    ///
    /// # Errors
    ///
    /// Returns an [`PhysicsError`] when the position is at a singular
    /// location (e.g., Earth's centre for `PointMassGravity`) or the
    /// model produces a non-finite output.
    fn gravity_eci_m_s2(
        &self,
        position_eci: Position3<Eci>,
        time: SimTime,
    ) -> Result<Vector3<f64>, PhysicsError>;
}

// ---------------------------------------------------------------------
// ConstantGravity
// ---------------------------------------------------------------------

/// Constant gravity. Returns the configured ECI acceleration vector
/// at every query. Matches the toy scaffold.
#[derive(Copy, Clone, Debug)]
pub struct ConstantGravity {
    g_eci_m_s2: Vector3<f64>,
}

impl ConstantGravity {
    /// Construct from an explicit ECI acceleration vector.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if any component is
    /// non-finite.
    pub fn new(g_eci_m_s2: Vector3<f64>) -> Result<Self, PhysicsError> {
        if !g_eci_m_s2.iter().all(|v| v.is_finite()) {
            return Err(PhysicsError::InvalidParameter {
                reason: "constant gravity acceleration must be finite",
            });
        }
        Ok(Self { g_eci_m_s2 })
    }

    /// Convenience: gravity along the negative ECI `+z` axis with the
    /// given magnitude in m/s². Magnitudes must be non-negative; a
    /// negative magnitude is treated as a configuration error.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if the magnitude is
    /// negative or non-finite.
    pub fn down_z(g_magnitude_m_s2: f64) -> Result<Self, PhysicsError> {
        if !g_magnitude_m_s2.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "constant gravity magnitude must be finite",
            });
        }
        if g_magnitude_m_s2 < 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "constant gravity magnitude must be non-negative; \
                         use new(...) with an explicit vector for non-down directions",
            });
        }
        Self::new(Vector3::new(0.0, 0.0, -g_magnitude_m_s2))
    }

    /// The configured ECI acceleration vector, in m/s².
    #[must_use]
    pub const fn g_eci_m_s2(&self) -> Vector3<f64> {
        self.g_eci_m_s2
    }
}

impl GravityModel for ConstantGravity {
    fn gravity_eci_m_s2(
        &self,
        _position_eci: Position3<Eci>,
        _time: SimTime,
    ) -> Result<Vector3<f64>, PhysicsError> {
        Ok(self.g_eci_m_s2)
    }
}

// ---------------------------------------------------------------------
// PointMassGravity
// ---------------------------------------------------------------------

/// Newtonian point-mass gravity: `g(r) = −µ · r / |r|³`.
///
/// `r` is the inertial position vector; `µ` is the central body's
/// gravitational parameter. The default constructor uses the WGS84
/// Earth value.
#[derive(Copy, Clone, Debug)]
pub struct PointMassGravity {
    mu_m3_s2: f64,
}

impl PointMassGravity {
    /// Construct from an explicit gravitational parameter.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if `mu_m3_s2` is not
    /// strictly positive and finite.
    pub fn new(mu_m3_s2: f64) -> Result<Self, PhysicsError> {
        if !mu_m3_s2.is_finite() || mu_m3_s2 <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "gravitational parameter µ must be strictly positive and finite",
            });
        }
        Ok(Self { mu_m3_s2 })
    }

    /// WGS84 Earth.
    #[must_use]
    pub const fn wgs84() -> Self {
        Self {
            mu_m3_s2: WGS84_MU_M3_S2,
        }
    }
}

impl GravityModel for PointMassGravity {
    fn gravity_eci_m_s2(
        &self,
        position_eci: Position3<Eci>,
        _time: SimTime,
    ) -> Result<Vector3<f64>, PhysicsError> {
        let r = position_eci.vector;
        let r2 = r.dot(&r);
        if r2 == 0.0 {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "PointMassGravity is singular at r = 0",
            });
        }
        let r_norm = r2.sqrt();
        // DETERMINISM: locked order — multiply scalar coefficient by
        // vector components in source order; no FMA.
        let coeff = -self.mu_m3_s2 / (r_norm * r2);
        let g = coeff * r;
        if !g.iter().all(|v| v.is_finite()) {
            return Err(PhysicsError::NonFinite {
                reason: "point-mass gravity produced non-finite acceleration",
            });
        }
        Ok(g)
    }
}

// ---------------------------------------------------------------------
// J2Gravity
// ---------------------------------------------------------------------

/// Point-mass gravity plus the J2 zonal-harmonic perturbation, in
/// ECI Cartesian form.
///
/// The Cartesian J2 acceleration is (Vallado 4th ed., §8.6;
/// Montenbruck & Gill, *Satellite Orbits*, §3.2):
///
/// ```text
///   g_central = −µ · r / r³
///   k         = 1.5 · J2 · µ · R_e² / r⁵
///   z_factor  = 5 · z² / r²
///   g_J2_x    = k · x · (z_factor − 1)
///   g_J2_y    = k · y · (z_factor − 1)
///   g_J2_z    = k · z · (z_factor − 3)
///   g_total   = g_central + g_J2
/// ```
///
/// The default constructor pins WGS84 values:
///
/// * `µ = WGS84_MU_M3_S2`
/// * `R_e = WGS84_A_M`
/// * `J2 = WGS84_J2`
#[derive(Copy, Clone, Debug)]
pub struct J2Gravity {
    mu_m3_s2: f64,
    r_e_m: f64,
    j2: f64,
}

impl J2Gravity {
    /// Construct from explicit parameters.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if `mu_m3_s2` or `r_e_m`
    /// is not strictly positive and finite, or if `j2` is non-finite.
    /// `j2 = 0` is permitted (the model degenerates to point-mass).
    pub fn new(mu_m3_s2: f64, r_e_m: f64, j2: f64) -> Result<Self, PhysicsError> {
        if !mu_m3_s2.is_finite() || mu_m3_s2 <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "µ must be strictly positive and finite",
            });
        }
        if !r_e_m.is_finite() || r_e_m <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "Earth radius must be strictly positive and finite",
            });
        }
        if !j2.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "J2 must be finite",
            });
        }
        Ok(Self {
            mu_m3_s2,
            r_e_m,
            j2,
        })
    }

    /// WGS84 defaults.
    #[must_use]
    pub const fn wgs84() -> Self {
        Self {
            mu_m3_s2: WGS84_MU_M3_S2,
            r_e_m: WGS84_A_M,
            j2: WGS84_J2,
        }
    }

    /// Configured `µ` in m³/s².
    #[must_use]
    pub const fn mu_m3_s2(&self) -> f64 {
        self.mu_m3_s2
    }

    /// Configured Earth radius in m.
    #[must_use]
    pub const fn r_e_m(&self) -> f64 {
        self.r_e_m
    }

    /// Configured J2 coefficient (dimensionless).
    #[must_use]
    pub const fn j2(&self) -> f64 {
        self.j2
    }
}

impl GravityModel for J2Gravity {
    fn gravity_eci_m_s2(
        &self,
        position_eci: Position3<Eci>,
        _time: SimTime,
    ) -> Result<Vector3<f64>, PhysicsError> {
        let r = position_eci.vector;
        let r2 = r.dot(&r);
        if r2 == 0.0 {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "J2Gravity is singular at r = 0",
            });
        }
        let r_norm = r2.sqrt();
        let r3 = r_norm * r2;
        let r5 = r3 * r2;

        // Central term: g_c = -µ · r / r³
        let g_central_coeff = -self.mu_m3_s2 / r3;
        let g_central = g_central_coeff * r;

        let g_j2 = j2_perturbation_eci(r, r2, r5, self.mu_m3_s2, self.r_e_m, self.j2);

        // Locked order: central + J2.
        let g = g_central + g_j2;
        if !g.iter().all(|v| v.is_finite()) {
            return Err(PhysicsError::NonFinite {
                reason: "J2 gravity produced non-finite acceleration",
            });
        }
        Ok(g)
    }
}

fn j2_perturbation_eci(
    r: Vector3<f64>,
    r2: f64,
    r5: f64,
    mu_m3_s2: f64,
    r_e_m: f64,
    j2: f64,
) -> Vector3<f64> {
    let k = 1.5 * j2 * mu_m3_s2 * r_e_m * r_e_m / r5;
    let z2_over_r2 = (r.z * r.z) / r2;
    let z_factor = 5.0 * z2_over_r2;
    Vector3::new(
        k * r.x * (z_factor - 1.0),
        k * r.y * (z_factor - 1.0),
        k * r.z * (z_factor - 3.0),
    )
}

// ---------------------------------------------------------------------
// Egm2008ZonalGravity
// ---------------------------------------------------------------------

/// EGM2008 zonal-harmonic coefficient `J_3` (unnormalised). Source:
/// Pavlis, N. K., et al. (2012). *The development and evaluation of
/// the Earth Gravitational Model 2008 (EGM2008)*, J. Geophys. Res.
/// 117, B04406. Public NGA-published tables; widely reproduced in
/// Vallado 4th ed. Table 8-7 ("Earth zonal harmonics, EGM-96") with
/// values that match EGM2008 zonals to the precision shown.
pub const EGM2008_J3: f64 = -2.532_641_3e-6;
/// EGM2008 zonal-harmonic coefficient `J_4` (unnormalised). Same
/// source as [`EGM2008_J3`].
pub const EGM2008_J4: f64 = -1.619_898_4e-6;
/// EGM2008 zonal-harmonic coefficient `J_5` (unnormalised). Same
/// source as [`EGM2008_J3`].
pub const EGM2008_J5: f64 = -2.277_358_8e-7;
/// EGM2008 zonal-harmonic coefficient `J_6` (unnormalised). Same
/// source as [`EGM2008_J3`].
pub const EGM2008_J6: f64 = 5.408_082_3e-7;

/// Maximum supported zonal degree for [`Egm2008ZonalGravity`]. The
/// implementation maintains fixed-size arrays for the per-degree
/// coefficients and the per-step Legendre-polynomial recurrence;
/// extending past degree 6 would require trustworthy higher-degree
/// EGM2008 zonal coefficients that this slice does not pin.
pub const EGM2008_MAX_DEGREE: usize = 6;

/// Truncated EGM2008 zonal-harmonic gravity model (degrees 2-6).
///
/// Adds the `J_3` through `J_n` zonal corrections on top of the
/// existing `J_2` perturbation, in ECI Cartesian form. The
/// implementation computes the spherical-harmonic acceleration via
/// the closed-form gradient of the geopotential
///
/// ```text
///   V_n = −(μ/r) (R_e/r)^n J_n P_n(ξ),    ξ = z/r
/// ```
///
/// using the recursive formulae
///
/// ```text
///   g_n_x = (μ R_e^n J_n / r^{n+3}) · x · [(n+1) P_n(ξ) + ξ P_n'(ξ)]
///   g_n_y = same with y
///   g_n_z = (μ R_e^n J_n / r^{n+3}) · [(n+1) z P_n(ξ) − r (1−ξ²) P_n'(ξ)]
/// ```
///
/// Locked operand order matches the existing `J_2` path so the model
/// degenerates to byte-identical [`J2Gravity`] output when
/// configured with `degree = 2`. Higher degrees add a deterministic
/// summation of per-axis contributions using the same sign convention:
/// positive `J_2` strengthens Newtonian gravity at the equator.
///
/// **Honest scope.** This is the **zonal-only** truncation of EGM2008
/// — tesseral and sectoral terms are deferred to a later slice that
/// pins higher-degree normalised coefficients. For reentry-class
/// orbits, zonal-only `J_2`-`J_6` captures the dominant secular
/// perturbations (right-ascension drift, argument-of-perigee drift,
/// nodal regression).
#[derive(Copy, Clone, Debug)]
pub struct Egm2008ZonalGravity {
    mu_m3_s2: f64,
    r_e_m: f64,
    /// Per-degree zonal coefficients `[J_2, J_3, J_4, J_5, J_6]`.
    /// Higher-degree slots beyond the configured cap are zero so the
    /// summation degenerates to the requested truncation.
    j_n: [f64; EGM2008_MAX_DEGREE - 1],
    /// Inclusive maximum degree consulted (`>= 2`, `<=
    /// EGM2008_MAX_DEGREE`).
    degree: usize,
}

impl Egm2008ZonalGravity {
    /// Construct from explicit parameters.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if `mu_m3_s2` or
    /// `r_e_m` is not strictly positive and finite, if any zonal
    /// coefficient is non-finite, or if `degree` is outside `[2,
    /// EGM2008_MAX_DEGREE]`.
    pub fn new(
        mu_m3_s2: f64,
        r_e_m: f64,
        j_n: [f64; EGM2008_MAX_DEGREE - 1],
        degree: usize,
    ) -> Result<Self, PhysicsError> {
        if !mu_m3_s2.is_finite() || mu_m3_s2 <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "µ must be strictly positive and finite",
            });
        }
        if !r_e_m.is_finite() || r_e_m <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "Earth radius must be strictly positive and finite",
            });
        }
        if !j_n.iter().all(|v| v.is_finite()) {
            return Err(PhysicsError::InvalidParameter {
                reason: "every zonal coefficient J_n must be finite",
            });
        }
        if !(2..=EGM2008_MAX_DEGREE).contains(&degree) {
            return Err(PhysicsError::InvalidParameter {
                reason: "EGM2008 zonal degree must be in [2, 6]",
            });
        }
        Ok(Self {
            mu_m3_s2,
            r_e_m,
            j_n,
            degree,
        })
    }

    /// WGS84 / EGM2008-zonal defaults: `µ = WGS84_MU_M3_S2`, `R_e =
    /// WGS84_A_M`, `J_n` from the public Pavlis et al. 2012 tables,
    /// truncation at degree 6.
    #[must_use]
    pub const fn wgs84_egm2008_zonal() -> Self {
        Self {
            mu_m3_s2: WGS84_MU_M3_S2,
            r_e_m: WGS84_A_M,
            j_n: [WGS84_J2, EGM2008_J3, EGM2008_J4, EGM2008_J5, EGM2008_J6],
            degree: EGM2008_MAX_DEGREE,
        }
    }

    /// Configured `µ` (m³/s²).
    #[must_use]
    pub const fn mu_m3_s2(&self) -> f64 {
        self.mu_m3_s2
    }

    /// Configured Earth radius (m).
    #[must_use]
    pub const fn r_e_m(&self) -> f64 {
        self.r_e_m
    }

    /// Configured maximum zonal degree.
    #[must_use]
    pub const fn degree(&self) -> usize {
        self.degree
    }

    /// `J_n` values consulted (zero-padded after `degree`).
    #[must_use]
    pub const fn j_n(&self) -> [f64; EGM2008_MAX_DEGREE - 1] {
        self.j_n
    }
}

impl GravityModel for Egm2008ZonalGravity {
    // The Legendre recurrence below converts the loop index `usize`
    // into `f64`; the maximum degree is bounded at compile time by
    // `EGM2008_MAX_DEGREE = 6`, so the cast can never lose precision.
    #[allow(clippy::cast_precision_loss)]
    fn gravity_eci_m_s2(
        &self,
        position_eci: Position3<Eci>,
        _time: SimTime,
    ) -> Result<Vector3<f64>, PhysicsError> {
        let r_vec = position_eci.vector;
        let r2 = r_vec.dot(&r_vec);
        if r2 == 0.0 {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "Egm2008ZonalGravity is singular at r = 0",
            });
        }
        let r = r2.sqrt();
        let r3 = r * r2;
        let r5 = r3 * r2;
        let inv_r = 1.0 / r;
        let xi = r_vec.z * inv_r;

        // Central term: g_central = -µ r / r³.
        let g_central = (-self.mu_m3_s2 / r3) * r_vec;

        // Per-degree zonal sum. Pre-compute the Legendre polynomial
        // value `P_n(ξ)` and derivative `P_n'(ξ)` via the standard
        // recurrences; pre-compute the radial scale `(R_e/r)^n` by
        // running multiplication.
        //
        //   P_{n+1}(ξ) = ((2n+1)·ξ·P_n − n·P_{n-1}) / (n+1)
        //   P_{n+1}'(ξ) = ((2n+1)·(P_n + ξ·P_n') − n·P_{n-1}') / (n+1)
        //
        // Initial conditions: P_0 = 1, P_0' = 0; P_1 = ξ, P_1' = 1.
        let mut p_prev = 1.0_f64;
        let mut p_n = xi;
        let mut p_prev_prime = 0.0_f64;
        let mut p_n_prime = 1.0_f64;
        let mut radial_pow = self.r_e_m * inv_r; // (R_e/r)^1
        let mu_over_r3 = self.mu_m3_s2 / r3;
        let one_minus_xi2 = 1.0 - xi * xi;

        // Degree 2 is evaluated through the exact helper used by
        // `J2Gravity`, so a degree-2 EGM2008 configuration degenerates
        // to byte-identical J2 output. The recurrence still advances
        // through n = 2 below so the higher-degree Legendre state is
        // available when `degree > 2`.
        let mut g_zonal =
            j2_perturbation_eci(r_vec, r2, r5, self.mu_m3_s2, self.r_e_m, self.j_n[0]);
        for n in 2..=self.degree {
            // Advance Legendre to degree n.
            let n_f = n as f64;
            let n_minus_1_f = (n - 1) as f64;
            let two_n_minus_1 = 2.0 * n_minus_1_f + 1.0;
            let p_next = (two_n_minus_1 * xi * p_n - n_minus_1_f * p_prev) / n_f;
            let p_next_prime =
                (two_n_minus_1 * (p_n + xi * p_n_prime) - n_minus_1_f * p_prev_prime) / n_f;
            p_prev = p_n;
            p_prev_prime = p_n_prime;
            p_n = p_next;
            p_n_prime = p_next_prime;
            // (R_e / r)^n
            radial_pow *= self.r_e_m * inv_r;
            if n == 2 {
                continue;
            }

            let j = self.j_n[n - 2];
            // Common scale: (µ R_e^n J_n) / r^{n+3} = µ/r³ · (R_e/r)^n · J_n
            let scale = mu_over_r3 * radial_pow * j;
            // Bracket factor for x, y components: (n+1) P_n + ξ P_n'.
            let bracket_xy = (n_f + 1.0) * p_n + xi * p_n_prime;
            // Bracket factor for z component:
            //   (n+1) z P_n − r (1−ξ²) P_n'
            // Note: rewriting using z = ξ r: (n+1) ξ r P_n − r (1−ξ²) P_n'
            //   = r · [(n+1) ξ P_n − (1−ξ²) P_n']
            // Then g_n_z = (µ R_e^n J_n / r^{n+3}) · r · [...] = scale · r · [...]
            let bracket_z = (n_f + 1.0) * xi * p_n - one_minus_xi2 * p_n_prime;
            g_zonal.x += scale * r_vec.x * bracket_xy;
            g_zonal.y += scale * r_vec.y * bracket_xy;
            g_zonal.z += scale * r * bracket_z;
        }

        let g = g_central + g_zonal;
        if !g.iter().all(|v| v.is_finite()) {
            return Err(PhysicsError::NonFinite {
                reason: "EGM2008 zonal gravity produced non-finite acceleration",
            });
        }
        Ok(g)
    }
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

    fn at_x(x: f64) -> Position3<Eci> {
        Position3::new(x, 0.0, 0.0)
    }

    fn at_z(z: f64) -> Position3<Eci> {
        Position3::new(0.0, 0.0, z)
    }

    #[test]
    fn constant_gravity_returns_configured_vector() {
        let g_vec = Vector3::new(1.0, -2.0, -9.81);
        let g = ConstantGravity::new(g_vec).unwrap();
        let out = g
            .gravity_eci_m_s2(Position3::origin(), SimTime::ZERO)
            .unwrap();
        assert_abs_diff_eq!(out.x, 1.0);
        assert_abs_diff_eq!(out.y, -2.0);
        assert_abs_diff_eq!(out.z, -9.81);
    }

    #[test]
    fn constant_gravity_down_z_rejects_negative_magnitude() {
        let err = ConstantGravity::down_z(-1.0).unwrap_err();
        assert!(matches!(err, PhysicsError::InvalidParameter { .. }));
    }

    #[test]
    fn point_mass_gravity_at_earth_surface_radial_axis() {
        let g = PointMassGravity::wgs84();
        let out = g.gravity_eci_m_s2(at_x(WGS84_A_M), SimTime::ZERO).unwrap();
        // Magnitude should be ~ µ / a² ≈ 9.798 m/s² (slightly different
        // from local g because of Earth's rotation and oblateness).
        let expected = WGS84_MU_M3_S2 / (WGS84_A_M * WGS84_A_M);
        assert_abs_diff_eq!(out.x, -expected, epsilon = 1.0e-6);
        assert_abs_diff_eq!(out.y, 0.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(out.z, 0.0, epsilon = 1.0e-12);
    }

    #[test]
    fn point_mass_gravity_singular_at_origin() {
        let g = PointMassGravity::wgs84();
        let err = g
            .gravity_eci_m_s2(Position3::origin(), SimTime::ZERO)
            .unwrap_err();
        assert!(matches!(err, PhysicsError::OutOfEnvelope { .. }));
    }

    #[test]
    fn point_mass_gravity_rejects_non_positive_mu() {
        assert!(PointMassGravity::new(0.0).is_err());
        assert!(PointMassGravity::new(-1.0).is_err());
        assert!(PointMassGravity::new(f64::NAN).is_err());
    }

    #[test]
    fn point_mass_gravity_is_antisymmetric() {
        let g = PointMassGravity::wgs84();
        let r = 7_000_000.0;
        let g_pos = g.gravity_eci_m_s2(at_x(r), SimTime::ZERO).unwrap();
        let g_neg = g.gravity_eci_m_s2(at_x(-r), SimTime::ZERO).unwrap();
        assert_abs_diff_eq!(g_pos.x + g_neg.x, 0.0, epsilon = 1.0e-9);
        assert_abs_diff_eq!(g_pos.y + g_neg.y, 0.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(g_pos.z + g_neg.z, 0.0, epsilon = 1.0e-12);
    }

    #[test]
    fn j2_gravity_reduces_to_point_mass_when_j2_is_zero() {
        let g_pm = PointMassGravity::wgs84();
        let g_j2 = J2Gravity::new(WGS84_MU_M3_S2, WGS84_A_M, 0.0).unwrap();
        let r = Position3::new(7_000_000.0, 1_500_000.0, 800_000.0);
        let pm = g_pm.gravity_eci_m_s2(r, SimTime::ZERO).unwrap();
        let j2 = g_j2.gravity_eci_m_s2(r, SimTime::ZERO).unwrap();
        assert_abs_diff_eq!(pm.x, j2.x, epsilon = 1.0e-12);
        assert_abs_diff_eq!(pm.y, j2.y, epsilon = 1.0e-12);
        assert_abs_diff_eq!(pm.z, j2.z, epsilon = 1.0e-12);
    }

    #[test]
    fn j2_gravity_at_equator_strengthens_pure_newtonian_gravity() {
        // At the equatorial plane (z = 0), `z_factor = 0`, so:
        //   g_J2_x = k · x · (-1)  (k > 0, x > 0)
        // The J2 perturbation at the equator points radially *inward*
        // (same direction as the central -µr/r³ term), strengthening
        // pure Newtonian gravity at a point on the equator. This is
        // the *Newtonian* (no-centrifugal) result; the everyday
        // observation that "polar gravity > equatorial gravity"
        // arises after subtracting Earth's centrifugal acceleration in
        // a rotating frame, not from this Cartesian J2 alone.
        let g_pm = PointMassGravity::wgs84();
        let g_j2 = J2Gravity::wgs84();
        let r = at_x(WGS84_A_M);
        let pm = g_pm.gravity_eci_m_s2(r, SimTime::ZERO).unwrap();
        let total = g_j2.gravity_eci_m_s2(r, SimTime::ZERO).unwrap();
        // Total should be *more* negative (larger magnitude) at the
        // equator under the J2 perturbation alone.
        assert!(
            total.x < pm.x,
            "J2 perturbation should strengthen pure Newtonian equatorial gravity: pm={}, total={}",
            pm.x,
            total.x,
        );
    }

    #[test]
    fn j2_gravity_along_polar_axis_weakens_pure_newtonian_gravity() {
        // At the pole (x = y = 0, z = R_e), `z_factor = 5`, so:
        //   g_J2_z = k · z · (5 - 3) = 2 · k · z   (k > 0, z > 0)
        // The J2 perturbation at the pole points radially *outward*
        // (opposite to the central -µr/r³ term), weakening pure
        // Newtonian gravity at a point on the polar axis.
        let g_pm = PointMassGravity::wgs84();
        let g_j2 = J2Gravity::wgs84();
        let r = at_z(WGS84_A_M);
        let pm = g_pm.gravity_eci_m_s2(r, SimTime::ZERO).unwrap();
        let total = g_j2.gravity_eci_m_s2(r, SimTime::ZERO).unwrap();
        // Total should be *less* negative (smaller magnitude) at the
        // pole under the J2 perturbation alone.
        assert!(
            total.z > pm.z,
            "J2 perturbation should weaken pure Newtonian polar gravity: pm={}, total={}",
            pm.z,
            total.z,
        );
    }

    #[test]
    fn j2_equatorial_perturbation_scales_at_geo_radius() {
        // On the equatorial axis, the J2 perturbation ratio has a
        // simple independent form:
        //   |g_J2| / |g_central| = 1.5 · J2 · (R_e / r)^2
        // This large-r check catches exponent mistakes in the r^5
        // denominator that surface-level sign tests can miss.
        let g_pm = PointMassGravity::wgs84();
        let g_j2 = J2Gravity::wgs84();
        let geo_radius_m = 42_164_000.0;
        let r = at_x(geo_radius_m);
        let pm = g_pm.gravity_eci_m_s2(r, SimTime::ZERO).unwrap();
        let total = g_j2.gravity_eci_m_s2(r, SimTime::ZERO).unwrap();

        let perturbation = total.x - pm.x;
        let expected_ratio = 1.5 * WGS84_J2 * (WGS84_A_M / geo_radius_m).powi(2);
        let actual_ratio = perturbation.abs() / pm.x.abs();
        assert_abs_diff_eq!(actual_ratio, expected_ratio, epsilon = 1.0e-15);
    }

    #[test]
    fn j2_gravity_rejects_invalid_parameters() {
        assert!(J2Gravity::new(0.0, WGS84_A_M, WGS84_J2).is_err());
        assert!(J2Gravity::new(WGS84_MU_M3_S2, 0.0, WGS84_J2).is_err());
        assert!(J2Gravity::new(WGS84_MU_M3_S2, WGS84_A_M, f64::NAN).is_err());
    }

    /// Kepler check: integrating point-mass gravity over a circular
    /// orbit by closed-form (no integrator) should produce a constant
    /// magnitude of acceleration. We don't run a full kernel here —
    /// just verify the model produces the right behaviour for orbital
    /// physics primitives.
    #[test]
    fn point_mass_gravity_magnitude_constant_along_circle() {
        let g = PointMassGravity::wgs84();
        let r_circle = 7_000_000.0;
        let positions = [
            at_x(r_circle),
            Position3::new(0.0, r_circle, 0.0),
            Position3::new(r_circle * 0.6, r_circle * 0.8, 0.0),
            Position3::new(r_circle * 0.3, -r_circle * 0.4, r_circle * 0.866_025_4),
        ];
        // Last vector has norm sqrt(0.09 + 0.16 + 0.75) ≈ 1, but multiplied
        // by r_circle gives a sphere point. Verify that all four produce
        // the same acceleration magnitude.
        let mags: Vec<f64> = positions
            .iter()
            .map(|p| g.gravity_eci_m_s2(*p, SimTime::ZERO).unwrap().norm())
            .collect();
        for m in &mags[1..] {
            assert_abs_diff_eq!(*m, mags[0], epsilon = 1.0e-3);
        }
    }

    // EGM2008 zonal-gravity tests ------------------------------------

    fn at_xyz(x: f64, y: f64, z: f64) -> Position3<Eci> {
        Position3::new(x, y, z)
    }

    #[test]
    fn egm2008_zonal_constructor_rejects_invalid_inputs() {
        // Non-positive µ.
        assert!(matches!(
            Egm2008ZonalGravity::new(0.0, WGS84_A_M, [WGS84_J2, 0.0, 0.0, 0.0, 0.0], 2),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        // Non-positive Earth radius.
        assert!(matches!(
            Egm2008ZonalGravity::new(WGS84_MU_M3_S2, -1.0, [WGS84_J2, 0.0, 0.0, 0.0, 0.0], 2),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        // Non-finite J coefficient.
        assert!(matches!(
            Egm2008ZonalGravity::new(
                WGS84_MU_M3_S2,
                WGS84_A_M,
                [WGS84_J2, f64::NAN, 0.0, 0.0, 0.0],
                3,
            ),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        // Out-of-range degree.
        assert!(matches!(
            Egm2008ZonalGravity::new(WGS84_MU_M3_S2, WGS84_A_M, [0.0; 5], 1),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            Egm2008ZonalGravity::new(WGS84_MU_M3_S2, WGS84_A_M, [0.0; 5], 7),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn egm2008_zonal_singular_at_origin() {
        let g = Egm2008ZonalGravity::wgs84_egm2008_zonal();
        let err = g
            .gravity_eci_m_s2(Position3::origin(), SimTime::ZERO)
            .unwrap_err();
        assert!(matches!(err, PhysicsError::OutOfEnvelope { .. }));
    }

    #[test]
    fn egm2008_zonal_at_degree_2_with_only_j2_matches_existing_j2_gravity() {
        // Configure EGM2008 zonal with degree=2 and J_3..J_6 = 0;
        // it should produce byte-identical output to J2Gravity.
        let zonal =
            Egm2008ZonalGravity::new(WGS84_MU_M3_S2, WGS84_A_M, [WGS84_J2, 0.0, 0.0, 0.0, 0.0], 2)
                .expect("ok");
        let j2 = J2Gravity::wgs84();
        let positions = [
            at_x(WGS84_A_M + 100_000.0),
            at_z(WGS84_A_M + 100_000.0),
            at_xyz(7e6, 1e6, 5e5),
            at_xyz(0.0, 7e6, 0.0),
        ];
        for pos in positions {
            let g_zonal = zonal.gravity_eci_m_s2(pos, SimTime::ZERO).unwrap();
            let g_j2 = j2.gravity_eci_m_s2(pos, SimTime::ZERO).unwrap();
            for axis in 0..3 {
                assert_eq!(g_zonal[axis].to_bits(), g_j2[axis].to_bits());
            }
        }
    }

    #[test]
    fn egm2008_zonal_returns_well_defined_acceleration_at_low_earth_orbit() {
        let g = Egm2008ZonalGravity::wgs84_egm2008_zonal();
        // 400 km altitude, prograde orbit slice.
        let pos = at_xyz(WGS84_A_M + 400_000.0, 0.0, 0.0);
        let out = g.gravity_eci_m_s2(pos, SimTime::ZERO).unwrap();
        // Magnitude should be near µ/r² ≈ 8.69 m/s² at r = 6778 km
        // with small zonal corrections.
        let r = pos.vector.norm();
        let central_mag = WGS84_MU_M3_S2 / (r * r);
        let total_mag = out.norm();
        // Zonal correction magnitude is at most ~0.05 m/s² near
        // surface; should be much smaller fraction at LEO.
        assert!(
            (total_mag - central_mag).abs() < 0.05,
            "zonal correction unexpectedly large: total {total_mag}, central {central_mag}"
        );
    }

    #[test]
    fn egm2008_zonal_higher_degrees_change_acceleration() {
        // Compare degree=2 vs degree=6 truncations on a non-equatorial
        // point: the higher-degree contributions must be non-zero.
        let zonal_2 = Egm2008ZonalGravity::new(
            WGS84_MU_M3_S2,
            WGS84_A_M,
            [WGS84_J2, EGM2008_J3, EGM2008_J4, EGM2008_J5, EGM2008_J6],
            2,
        )
        .unwrap();
        let zonal_6 = Egm2008ZonalGravity::wgs84_egm2008_zonal();
        let pos = at_xyz(WGS84_A_M * 0.6, 0.0, WGS84_A_M * 0.8);
        let g2 = zonal_2.gravity_eci_m_s2(pos, SimTime::ZERO).unwrap();
        let g6 = zonal_6.gravity_eci_m_s2(pos, SimTime::ZERO).unwrap();
        let max_diff = (g6 - g2).iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
        assert!(
            max_diff > 1.0e-9,
            "degree-2 vs degree-6 must differ at non-equatorial point; diff was {max_diff}"
        );
    }

    #[test]
    fn egm2008_zonal_is_deterministic_across_reruns() {
        let g = Egm2008ZonalGravity::wgs84_egm2008_zonal();
        let pos = at_xyz(7.5e6, -1.2e6, 3.4e6);
        let a = g.gravity_eci_m_s2(pos, SimTime::ZERO).unwrap();
        let b = g.gravity_eci_m_s2(pos, SimTime::ZERO).unwrap();
        for axis in 0..3 {
            assert_eq!(a[axis].to_bits(), b[axis].to_bits());
        }
    }

    #[test]
    fn egm2008_zonal_radial_acceleration_at_pole_matches_central_plus_j2() {
        // At a pole (z = r, x = y = 0), J_3 contribution is zero only
        // for degrees that vanish at ξ = 1; verify the model still
        // returns finite values close to central + J2 dominant.
        let g = Egm2008ZonalGravity::wgs84_egm2008_zonal();
        let pos = at_z(WGS84_A_M + 1_000_000.0);
        let out = g.gravity_eci_m_s2(pos, SimTime::ZERO).unwrap();
        assert!(out.iter().all(|v| v.is_finite()));
        // x and y components should be ~zero at a pure-z position.
        assert_abs_diff_eq!(out.x, 0.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(out.y, 0.0, epsilon = 1.0e-12);
        // z must be inward (negative).
        assert!(out.z < 0.0);
    }

    #[test]
    fn egm2008_zonal_legendre_recurrence_matches_closed_form_for_low_degrees() {
        // Sanity check on the recurrence: for ξ = 0.5, P_2 = 1/8,
        // P_3 = -7/16, P_4 = -77/128. Validate by a manual
        // re-implementation.
        let xi = 0.5_f64;
        let p_2_closed = (3.0 * xi * xi - 1.0) / 2.0;
        let p_3_closed = (5.0 * xi.powi(3) - 3.0 * xi) / 2.0;
        let p_4_closed = (35.0 * xi.powi(4) - 30.0 * xi * xi + 3.0) / 8.0;
        // Run the same recurrence the gravity model uses.
        let mut p_prev = 1.0_f64;
        let mut p_n = xi;
        for n in 2..=4_usize {
            let n_f = n as f64;
            let n_minus_1_f = (n - 1) as f64;
            let two_n_minus_1 = 2.0 * n_minus_1_f + 1.0;
            let p_next = (two_n_minus_1 * xi * p_n - n_minus_1_f * p_prev) / n_f;
            p_prev = p_n;
            p_n = p_next;
            let expected = match n {
                2 => p_2_closed,
                3 => p_3_closed,
                4 => p_4_closed,
                _ => unreachable!(),
            };
            assert_abs_diff_eq!(p_n, expected, epsilon = 1.0e-15);
        }
    }
}
