//! Gravity models.
//!
//! Phase 2.2.B ships:
//!
//! * [`ConstantGravity`] — uniform `g` vector in ECI. Replicates the
//!   Phase-1 scaffold's behaviour exactly, but lives here so the
//!   environment-side gravity API doesn't depend on
//!   `openbmp-sim`. The Phase-1 `openbmp-sim::ConstantGravityForce`
//!   stays byte-identical; higher layers adapt one to the other.
//! * [`PointMassGravity`] — Newtonian `−µ/r² r̂`. Default constructor
//!   pins the WGS84 `µ`.
//! * [`J2Gravity`] — point-mass plus the J2 zonal harmonic, expressed
//!   in ECI. Default constructor pins WGS84
//!   `µ`, `R_e`, and the NIMA TR 8350.2 `J2 = 1.082626683 × 10⁻³`.
//!
//! All three models implement the [`GravityModel`] trait and report
//! failure via [`crate::error::EnvError`] (out-of-envelope, non-finite,
//! invalid parameter). They never panic, never silent-clamp, and
//! never return `NaN`.
//!
//! Determinism: pure arithmetic on `f64`; locked operand order on the
//! J2 sum; no FMA.

use nalgebra::Vector3;
use openbmp_core::{Eci, Position3, SimTime, WGS84_A_M, WGS84_MU_M3_S2};

use crate::error::EnvError;

/// J2 zonal-harmonic coefficient for the WGS84 reference ellipsoid.
///
/// Source: NIMA TR 8350.2 (NGA WGS84), 3rd edition (2000), table 3.5.
/// Cited live at the NGA WGS84 portal:
/// <https://earth-info.nga.mil/index.php?dir=wgs84&action=wgs84>.
///
/// `J2 = 1.082626683 × 10⁻³` (unnormalised). The corresponding
/// normalised harmonic is `C̄₂₀ = -J2 / √5`.
pub const WGS84_J2: f64 = 1.082_626_683e-3;

/// Trait implemented by gravity-providing environment models.
///
/// Returns the gravitational acceleration vector at an inertial
/// position and simulation time. Time is included for forward
/// compatibility with future time-varying corrections (none in
/// Phase 2).
pub trait GravityModel {
    /// Gravitational acceleration in ECI, in m/s².
    ///
    /// # Errors
    ///
    /// Returns an [`EnvError`] when the position is at a singular
    /// location (e.g., Earth's centre for `PointMassGravity`) or the
    /// model produces a non-finite output.
    fn gravity_eci_m_s2(
        &self,
        position_eci: Position3<Eci>,
        time: SimTime,
    ) -> Result<Vector3<f64>, EnvError>;
}

// ---------------------------------------------------------------------
// ConstantGravity
// ---------------------------------------------------------------------

/// Constant gravity. Returns the configured ECI acceleration vector
/// at every query. Matches the Phase-1 toy scaffold.
#[derive(Copy, Clone, Debug)]
pub struct ConstantGravity {
    g_eci_m_s2: Vector3<f64>,
}

impl ConstantGravity {
    /// Construct from an explicit ECI acceleration vector.
    ///
    /// # Errors
    ///
    /// Returns [`EnvError::InvalidParameter`] if any component is
    /// non-finite.
    pub fn new(g_eci_m_s2: Vector3<f64>) -> Result<Self, EnvError> {
        if !g_eci_m_s2.iter().all(|v| v.is_finite()) {
            return Err(EnvError::InvalidParameter {
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
    /// Returns [`EnvError::InvalidParameter`] if the magnitude is
    /// negative or non-finite.
    pub fn down_z(g_magnitude_m_s2: f64) -> Result<Self, EnvError> {
        if !g_magnitude_m_s2.is_finite() {
            return Err(EnvError::InvalidParameter {
                reason: "constant gravity magnitude must be finite",
            });
        }
        if g_magnitude_m_s2 < 0.0 {
            return Err(EnvError::InvalidParameter {
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
    ) -> Result<Vector3<f64>, EnvError> {
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
    /// Returns [`EnvError::InvalidParameter`] if `mu_m3_s2` is not
    /// strictly positive and finite.
    pub fn new(mu_m3_s2: f64) -> Result<Self, EnvError> {
        if !mu_m3_s2.is_finite() || mu_m3_s2 <= 0.0 {
            return Err(EnvError::InvalidParameter {
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
    ) -> Result<Vector3<f64>, EnvError> {
        let r = position_eci.vector;
        let r2 = r.dot(&r);
        if r2 == 0.0 {
            return Err(EnvError::OutOfEnvelope {
                reason: "PointMassGravity is singular at r = 0",
            });
        }
        let r_norm = r2.sqrt();
        // DETERMINISM: locked order — multiply scalar coefficient by
        // vector components in source order; no FMA.
        let coeff = -self.mu_m3_s2 / (r_norm * r2);
        let g = coeff * r;
        if !g.iter().all(|v| v.is_finite()) {
            return Err(EnvError::NonFinite {
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
    /// Returns [`EnvError::InvalidParameter`] if `mu_m3_s2` or `r_e_m`
    /// is not strictly positive and finite, or if `j2` is non-finite.
    /// `j2 = 0` is permitted (the model degenerates to point-mass).
    pub fn new(mu_m3_s2: f64, r_e_m: f64, j2: f64) -> Result<Self, EnvError> {
        if !mu_m3_s2.is_finite() || mu_m3_s2 <= 0.0 {
            return Err(EnvError::InvalidParameter {
                reason: "µ must be strictly positive and finite",
            });
        }
        if !r_e_m.is_finite() || r_e_m <= 0.0 {
            return Err(EnvError::InvalidParameter {
                reason: "Earth radius must be strictly positive and finite",
            });
        }
        if !j2.is_finite() {
            return Err(EnvError::InvalidParameter {
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
    ) -> Result<Vector3<f64>, EnvError> {
        let r = position_eci.vector;
        let r2 = r.dot(&r);
        if r2 == 0.0 {
            return Err(EnvError::OutOfEnvelope {
                reason: "J2Gravity is singular at r = 0",
            });
        }
        let r_norm = r2.sqrt();
        let r3 = r_norm * r2;
        let r5 = r3 * r2;

        // Central term: g_c = -µ · r / r³
        let g_central_coeff = -self.mu_m3_s2 / r3;
        let g_central = g_central_coeff * r;

        // J2 term:
        let k = 1.5 * self.j2 * self.mu_m3_s2 * self.r_e_m * self.r_e_m / r5;
        let z2_over_r2 = (r.z * r.z) / r2;
        let z_factor = 5.0 * z2_over_r2;
        let g_j2 = Vector3::new(
            k * r.x * (z_factor - 1.0),
            k * r.y * (z_factor - 1.0),
            k * r.z * (z_factor - 3.0),
        );

        // Locked order: central + J2.
        let g = g_central + g_j2;
        if !g.iter().all(|v| v.is_finite()) {
            return Err(EnvError::NonFinite {
                reason: "J2 gravity produced non-finite acceleration",
            });
        }
        Ok(g)
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use openbmp_core::WGS84_A_M;

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
        assert!(matches!(err, EnvError::InvalidParameter { .. }));
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
        assert!(matches!(err, EnvError::OutOfEnvelope { .. }));
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
}
