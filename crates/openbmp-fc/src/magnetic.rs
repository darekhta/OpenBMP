//! Magnetic-field models.
//!
//! Provides:
//! - [`MagneticFieldModel`] — trait abstracting the geomagnetic
//!   field vector at an inertial position.
//! - [`EarthDipoleField`] — academic-tier degree-1 truncation of the
//!   World Magnetic Model (single dipole aligned with the ECI z-axis).
//!   Marked **validated-toy**: accuracy is sufficient for
//!   academic attitude-tracking scenarios but not for real-world
//!   geomagnetic operations.
//!
//! The full WMM 2025 spherical-harmonic field with the COF dataset
//! is a Phase 4.C deferral — pulling the dataset and the truncation
//! arithmetic into `openbmp-fc` is heavyweight and is out of scope
//! for the academic-tier reference shipped here.
//!
//! # Convention
//!
//! ECI z is treated as the magnetic-dipole axis (geographic-pole
//! coincidence). For a position `p = (x, y, z)`, with `r = |p|` and
//! colatitude `θ` such that `cos(θ) = z/r`:
//!
//! ```text
//!     B_r     = -2 · B_eq · (R_e/r)^3 · cos(θ)
//!     B_θ     =     -B_eq · (R_e/r)^3 · sin(θ)
//! ```
//!
//! These are projected onto ECI axes via the local radial /
//! southward-meridional unit vectors. At the magnetic equator the
//! field points horizontally (along ECI +z toward the magnetic
//! north pole); at the pole it points vertically along ECI −z
//! (downward into the planet, matching the magnetic-dip convention).

use nalgebra::Vector3;

/// Mean Earth radius (m).
const R_EARTH_M: f64 = 6_371_000.0;

/// Magnetic-field model — abstracts the geomagnetic field vector at
/// a given inertial position.
pub trait MagneticFieldModel: std::fmt::Debug {
    /// Returns the magnetic field vector in ECI (nT).
    fn field_eci_nt(&self, position_eci_m: Vector3<f64>) -> Vector3<f64>;
}

/// Academic-tier dipole field with the dipole axis aligned with the
/// ECI z-axis. Marked `validated-toy`. Configurable equatorial-surface
/// magnitude (default 30 000 nT, the textbook value for Earth).
#[derive(Copy, Clone, Debug)]
pub struct EarthDipoleField {
    /// Equatorial-surface field magnitude (nT). Default 30 000 nT.
    pub b_eq_nt: f64,
}

impl Default for EarthDipoleField {
    fn default() -> Self {
        Self { b_eq_nt: 30_000.0 }
    }
}

impl MagneticFieldModel for EarthDipoleField {
    fn field_eci_nt(&self, position_eci_m: Vector3<f64>) -> Vector3<f64> {
        let r = position_eci_m.norm();
        if r < R_EARTH_M * 0.5 {
            // Numerically tame the inside-the-Earth singularity for
            // simulator startup transients (position seeded at origin).
            // The field below 0.5·R_e is not physically meaningful in
            // a dipole approximation; clamp to the equatorial value.
            return Vector3::new(0.0, 0.0, self.b_eq_nt);
        }
        let r_ratio_cubed = (R_EARTH_M / r).powi(3);
        let cos_theta = position_eci_m.z / r;
        let sin_theta = (1.0 - cos_theta * cos_theta).max(0.0).sqrt();

        let b_r = -2.0 * self.b_eq_nt * r_ratio_cubed * cos_theta;
        let b_theta = -self.b_eq_nt * r_ratio_cubed * sin_theta;

        // Local radial unit (outward).
        let r_hat = position_eci_m / r;
        // Local southward-meridional unit, when sin(θ) > 0. At the
        // poles (sin(θ) → 0) θ̂ is undefined, but B_θ → 0 there.
        let theta_hat = if sin_theta > 1e-9 {
            let rho =
                (position_eci_m.x * position_eci_m.x + position_eci_m.y * position_eci_m.y).sqrt();
            Vector3::new(
                position_eci_m.x * position_eci_m.z / (r * rho),
                position_eci_m.y * position_eci_m.z / (r * rho),
                -rho / r,
            )
        } else {
            Vector3::zeros()
        };

        b_r * r_hat + b_theta * theta_hat
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn dipole_at_equator_is_horizontal_toward_pole() {
        let model = EarthDipoleField::default();
        let pos = Vector3::new(R_EARTH_M, 0.0, 0.0);
        let b = model.field_eci_nt(pos);
        // At the equator the field points along +z (toward magnetic
        // north pole) and has no x/y component.
        assert!(b.x.abs() < 1.0);
        assert!(b.y.abs() < 1.0);
        assert!((b.z - 30_000.0).abs() < 1.0);
    }

    #[test]
    fn dipole_at_pole_is_radial_inward() {
        let model = EarthDipoleField::default();
        let pos = Vector3::new(0.0, 0.0, R_EARTH_M);
        let b = model.field_eci_nt(pos);
        // At the pole the field points along −z (into the planet, the
        // magnetic dip).
        assert!(b.x.abs() < 1.0);
        assert!(b.y.abs() < 1.0);
        assert!((b.z + 60_000.0).abs() < 1.0);
    }

    #[test]
    fn dipole_decreases_with_altitude() {
        let model = EarthDipoleField::default();
        let surface = model.field_eci_nt(Vector3::new(R_EARTH_M, 0.0, 0.0));
        let aloft = model.field_eci_nt(Vector3::new(R_EARTH_M * 2.0, 0.0, 0.0));
        // (R_e/(2 R_e))³ = 1/8 — magnitude should drop by ~8.
        let ratio = surface.norm() / aloft.norm();
        assert!((ratio - 8.0).abs() < 0.1);
    }
}
