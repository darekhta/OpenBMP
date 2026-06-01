//! Closed-form solutions for analytic-toy validation cases.
//!
//! Each type encapsulates the parameters of an academic textbook
//! problem and provides closed-form `position_at(t)` / `velocity_at(t)`
//! evaluations. Tests in higher-level crates can compare integrator
//! output against these references.
//!
//! No state, no IO, no RNG — pure functions only.

use nalgebra::Vector2;
use thiserror::Error;

/// Convergence tolerance used by the Kepler solver.
pub const KEPLER_TOLERANCE: f64 = 1.0e-12;

/// Specific orbital energy `ε = v²/2 - μ/r`.
///
/// This is conserved on an unforced two-body coast arc. Use as a
/// drift-bounded invariant for fixed-step integration, not as an
/// exact equality over a finite numerical trajectory.
#[must_use]
pub fn specific_orbital_energy(mu_m3_s2: f64, radius_m: f64, speed_m_s: f64) -> f64 {
    0.5 * speed_m_s * speed_m_s - mu_m3_s2 / radius_m
}

/// Vis-viva speed `v = sqrt(μ(2/r - 1/a))`.
///
/// This is an exact closed-form pin for an ideal two-body orbit.
#[must_use]
pub fn vis_viva_speed(mu_m3_s2: f64, radius_m: f64, semi_major_axis_m: f64) -> f64 {
    (mu_m3_s2 * (2.0 / radius_m - 1.0 / semi_major_axis_m)).sqrt()
}

/// Specific angular momentum `h = r × v`.
///
/// Conserved on a torque-free two-body coast arc.
#[must_use]
pub fn specific_angular_momentum(position_m: [f64; 3], velocity_m_s: [f64; 3]) -> [f64; 3] {
    [
        position_m[1] * velocity_m_s[2] - position_m[2] * velocity_m_s[1],
        position_m[2] * velocity_m_s[0] - position_m[0] * velocity_m_s[2],
        position_m[0] * velocity_m_s[1] - position_m[1] * velocity_m_s[0],
    ]
}

/// Euclidean norm of a 3-vector.
#[must_use]
pub fn norm3(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

/// Errors produced by checked Keplerian helper methods.
#[derive(Copy, Clone, Debug, Error, PartialEq)]
pub enum KeplerError {
    /// Orbital elements or query time were outside the supported
    /// elliptical two-body domain.
    #[error(
        "invalid Keplerian elements: a={semi_major_axis_m}, e={eccentricity}, \
         mu={mu_m3_s2}, t={time_s}"
    )]
    InvalidElements {
        /// Semi-major axis in metres.
        semi_major_axis_m: f64,
        /// Eccentricity.
        eccentricity: f64,
        /// Standard gravitational parameter in m^3/s^2.
        mu_m3_s2: f64,
        /// Query time in seconds.
        time_s: f64,
    },
    /// The caller supplied zero Newton iterations.
    #[error("Kepler solver requires at least one iteration")]
    NonPositiveMaxIterations,
    /// Newton's method did not meet [`KEPLER_TOLERANCE`].
    #[error(
        "Kepler solver did not converge in {max_iter} iterations; \
         residual={residual}"
    )]
    DidNotConverge {
        /// Maximum iterations attempted.
        max_iter: u32,
        /// Final residual in Kepler's equation.
        residual: f64,
    },
}

/// 1-D constant-acceleration drop.
///
/// Position: `x(t) = x0 + v0·t + ½·a·t²`
/// Velocity: `v(t) = v0 + a·t`
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ConstantAccelerationDrop {
    /// Initial position (m).
    pub initial_position_m: f64,
    /// Initial velocity (m/s).
    pub initial_velocity_m_s: f64,
    /// Constant acceleration (m/s²).
    pub acceleration_m_s2: f64,
}

impl ConstantAccelerationDrop {
    /// Position at time `t` (seconds since start).
    #[must_use]
    pub fn position_at(&self, t: f64) -> f64 {
        self.initial_position_m
            + self.initial_velocity_m_s * t
            + 0.5 * self.acceleration_m_s2 * t * t
    }

    /// Velocity at time `t` (seconds since start).
    #[must_use]
    pub fn velocity_at(&self, t: f64) -> f64 {
        self.initial_velocity_m_s + self.acceleration_m_s2 * t
    }
}

/// Simple harmonic oscillator: `ẍ + ω²·x = 0`.
///
/// Position: `x(t) = A·cos(ω·t + φ)`
/// Velocity: `v(t) = -A·ω·sin(ω·t + φ)`
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct HarmonicOscillator {
    /// Amplitude (m).
    pub amplitude_m: f64,
    /// Angular frequency (rad/s).
    pub omega_rad_s: f64,
    /// Phase offset (rad).
    pub phase_rad: f64,
}

impl HarmonicOscillator {
    /// Position at time `t`.
    #[must_use]
    pub fn position_at(&self, t: f64) -> f64 {
        self.amplitude_m * (self.omega_rad_s * t + self.phase_rad).cos()
    }

    /// Velocity at time `t`.
    #[must_use]
    pub fn velocity_at(&self, t: f64) -> f64 {
        -self.amplitude_m * self.omega_rad_s * (self.omega_rad_s * t + self.phase_rad).sin()
    }
}

/// Two-body Keplerian orbit (unperturbed point-mass gravity).
///
/// Stores the orbital elements at epoch and provides a position
/// evaluation via the standard Kepler solution. Supports only
/// circular and elliptical orbits in the orbital plane (true anomaly
/// is computed via Newton's method on Kepler's equation).
///
/// This is a research-grade analytic reference, not an operational
/// orbit-determination tool.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct TwoBodyKeplerian {
    /// Semi-major axis (m).
    pub semi_major_axis_m: f64,
    /// Eccentricity (dimensionless, `[0, 1)` for bound orbits).
    pub eccentricity: f64,
    /// Standard gravitational parameter `GM` (m³/s²) of the central
    /// body.
    pub mu_m3_s2: f64,
    /// Mean anomaly at epoch (rad).
    pub mean_anomaly_at_epoch_rad: f64,
}

impl TwoBodyKeplerian {
    /// Validate that the elements represent a finite elliptical orbit
    /// at finite query time `t`.
    ///
    /// # Errors
    ///
    /// Returns [`KeplerError::InvalidElements`] for non-finite values,
    /// non-positive `a` or `mu`, or eccentricity outside `[0, 1)`.
    pub fn require_valid_at(&self, t: f64) -> Result<(), KeplerError> {
        if self.semi_major_axis_m.is_finite()
            && self.semi_major_axis_m > 0.0
            && self.eccentricity.is_finite()
            && (0.0..1.0).contains(&self.eccentricity)
            && self.mu_m3_s2.is_finite()
            && self.mu_m3_s2 > 0.0
            && self.mean_anomaly_at_epoch_rad.is_finite()
            && t.is_finite()
        {
            Ok(())
        } else {
            Err(KeplerError::InvalidElements {
                semi_major_axis_m: self.semi_major_axis_m,
                eccentricity: self.eccentricity,
                mu_m3_s2: self.mu_m3_s2,
                time_s: t,
            })
        }
    }

    /// Mean motion `n = sqrt(mu / a^3)` in rad/s.
    #[must_use]
    pub fn mean_motion_rad_s(&self) -> f64 {
        (self.mu_m3_s2 / self.semi_major_axis_m.powi(3)).sqrt()
    }

    /// Mean anomaly at time `t` (seconds from epoch).
    #[must_use]
    pub fn mean_anomaly_at(&self, t: f64) -> f64 {
        self.mean_anomaly_at_epoch_rad + self.mean_motion_rad_s() * t
    }

    /// Eccentric anomaly via Newton's method on Kepler's equation
    /// `M = E - e·sin(E)`.
    ///
    /// This convenience method returns `NaN` when checked solving
    /// fails. Use [`TwoBodyKeplerian::eccentric_anomaly_at_checked`]
    /// when tests need diagnostics.
    #[must_use]
    pub fn eccentric_anomaly_at(&self, t: f64, max_iter: u32) -> f64 {
        self.eccentric_anomaly_at_checked(t, max_iter)
            .unwrap_or(f64::NAN)
    }

    /// Checked eccentric anomaly solver via Newton's method on
    /// Kepler's equation `M = E - e*sin(E)`.
    ///
    /// # Errors
    ///
    /// Returns [`KeplerError`] when the elements are invalid,
    /// `max_iter == 0`, or the solver does not reach
    /// [`KEPLER_TOLERANCE`].
    pub fn eccentric_anomaly_at_checked(&self, t: f64, max_iter: u32) -> Result<f64, KeplerError> {
        self.require_valid_at(t)?;
        if max_iter == 0 {
            return Err(KeplerError::NonPositiveMaxIterations);
        }

        let mean_anomaly = self
            .mean_anomaly_at(t)
            .rem_euclid(2.0 * std::f64::consts::PI);
        let mut eccentric_anomaly = if self.eccentricity < 0.8 {
            mean_anomaly
        } else {
            std::f64::consts::PI
        };
        for _ in 0..max_iter {
            let f = eccentric_anomaly - self.eccentricity * eccentric_anomaly.sin() - mean_anomaly;
            let fp = 1.0 - self.eccentricity * eccentric_anomaly.cos();
            let delta = f / fp;
            eccentric_anomaly -= delta;
            if delta.abs() < KEPLER_TOLERANCE {
                return Ok(eccentric_anomaly);
            }
        }
        let residual =
            eccentric_anomaly - self.eccentricity * eccentric_anomaly.sin() - mean_anomaly;
        Err(KeplerError::DidNotConverge { max_iter, residual })
    }

    /// In-plane radius at time `t`.
    #[must_use]
    pub fn radius_at(&self, t: f64) -> f64 {
        self.radius_at_checked(t).unwrap_or(f64::NAN)
    }

    /// Checked in-plane radius at time `t`.
    ///
    /// # Errors
    ///
    /// Returns [`KeplerError`] when the elements are invalid or the
    /// eccentric-anomaly solver does not converge.
    pub fn radius_at_checked(&self, t: f64) -> Result<f64, KeplerError> {
        let eccentric_anomaly = self.eccentric_anomaly_at_checked(t, 64)?;
        Ok(self.semi_major_axis_m * (1.0 - self.eccentricity * eccentric_anomaly.cos()))
    }

    /// Checked position in the orbital plane at time `t`.
    ///
    /// The returned vector is `(x, y)` in the perifocal plane. It is
    /// intentionally not rotated into ECI because the
    /// inclination/RAAN/argument-of-periapsis surface is not part of
    /// the fixture contract.
    ///
    /// # Errors
    ///
    /// Returns [`KeplerError`] when the elements are invalid or the
    /// eccentric-anomaly solver does not converge.
    pub fn position_in_orbital_plane_at(&self, t: f64) -> Result<Vector2<f64>, KeplerError> {
        let eccentric_anomaly = self.eccentric_anomaly_at_checked(t, 64)?;
        let one_minus_e2 = 1.0 - self.eccentricity * self.eccentricity;
        Ok(Vector2::new(
            self.semi_major_axis_m * (eccentric_anomaly.cos() - self.eccentricity),
            self.semi_major_axis_m * one_minus_e2.sqrt() * eccentric_anomaly.sin(),
        ))
    }

    /// Checked velocity in the orbital plane at time `t`.
    ///
    /// # Errors
    ///
    /// Returns [`KeplerError`] when the elements are invalid or the
    /// eccentric-anomaly solver does not converge.
    pub fn velocity_in_orbital_plane_at(&self, t: f64) -> Result<Vector2<f64>, KeplerError> {
        let eccentric_anomaly = self.eccentric_anomaly_at_checked(t, 64)?;
        let denominator = 1.0 - self.eccentricity * eccentric_anomaly.cos();
        let scale = self.mean_motion_rad_s() * self.semi_major_axis_m / denominator;
        let one_minus_e2 = 1.0 - self.eccentricity * self.eccentricity;
        Ok(Vector2::new(
            -scale * eccentric_anomaly.sin(),
            scale * one_minus_e2.sqrt() * eccentric_anomaly.cos(),
        ))
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use openbmp_physics::WGS84_MU_M3_S2;

    #[test]
    fn constant_acceleration_drop_at_t0() {
        let drop = ConstantAccelerationDrop {
            initial_position_m: 100.0,
            initial_velocity_m_s: 0.0,
            acceleration_m_s2: -9.81,
        };
        assert_abs_diff_eq!(drop.position_at(0.0), 100.0);
        assert_abs_diff_eq!(drop.velocity_at(0.0), 0.0);
    }

    #[test]
    fn constant_acceleration_drop_at_one_second() {
        let drop = ConstantAccelerationDrop {
            initial_position_m: 100.0,
            initial_velocity_m_s: 0.0,
            acceleration_m_s2: -9.81,
        };
        // x(1) = 100 + 0 - 0.5·9.81·1 = 95.095
        assert_abs_diff_eq!(drop.position_at(1.0), 95.095, epsilon = 1.0e-9);
        assert_abs_diff_eq!(drop.velocity_at(1.0), -9.81, epsilon = 1.0e-9);
    }

    #[test]
    fn harmonic_oscillator_at_t0() {
        let h = HarmonicOscillator {
            amplitude_m: 1.0,
            omega_rad_s: 2.0,
            phase_rad: 0.0,
        };
        assert_abs_diff_eq!(h.position_at(0.0), 1.0);
        assert_abs_diff_eq!(h.velocity_at(0.0), 0.0, epsilon = 1.0e-12);
    }

    #[test]
    fn harmonic_oscillator_quarter_period() {
        // ω = 2π → period T = 1 s. At t = T/4, x = 0, v = -A·ω.
        let h = HarmonicOscillator {
            amplitude_m: 2.0,
            omega_rad_s: 2.0 * std::f64::consts::PI,
            phase_rad: 0.0,
        };
        assert_abs_diff_eq!(h.position_at(0.25), 0.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(
            h.velocity_at(0.25),
            -2.0 * 2.0 * std::f64::consts::PI,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn keplerian_circular_orbit_radius() {
        let orbit = TwoBodyKeplerian {
            semi_major_axis_m: 7.0e6,
            eccentricity: 0.0,
            mu_m3_s2: 3.986e14,
            mean_anomaly_at_epoch_rad: 0.0,
        };
        // Circular orbit: radius is constant at any time.
        assert_abs_diff_eq!(orbit.radius_at(0.0), 7.0e6, epsilon = 1.0);
        assert_abs_diff_eq!(orbit.radius_at(1000.0), 7.0e6, epsilon = 1.0);
    }

    #[test]
    fn vis_viva_and_specific_energy_agree_for_circular_orbit() {
        let mu = WGS84_MU_M3_S2;
        let radius = 7.0e6;
        let speed = vis_viva_speed(mu, radius, radius);
        let energy = specific_orbital_energy(mu, radius, speed);

        assert_abs_diff_eq!(speed * speed, mu / radius, epsilon = 1.0e-6);
        assert_abs_diff_eq!(energy, -mu / (2.0 * radius), epsilon = 1.0e-6);
    }

    #[test]
    fn specific_angular_momentum_cross_product_matches_right_hand_rule() {
        let h = specific_angular_momentum([7.0e6, 0.0, 0.0], [0.0, 7.5e3, 0.0]);

        assert_abs_diff_eq!(h[0], 0.0);
        assert_abs_diff_eq!(h[1], 0.0);
        assert_abs_diff_eq!(h[2], 7.0e6 * 7.5e3);
        assert_abs_diff_eq!(norm3(h), 7.0e6 * 7.5e3);
    }

    #[test]
    fn keplerian_eccentric_orbit_radius_oscillates() {
        let orbit = TwoBodyKeplerian {
            semi_major_axis_m: 7.0e6,
            eccentricity: 0.1,
            mu_m3_s2: 3.986e14,
            mean_anomaly_at_epoch_rad: 0.0,
        };
        // r ranges from a(1-e) to a(1+e).
        let r0 = orbit.radius_at(0.0);
        // At periapsis: r = a(1-e).
        assert_abs_diff_eq!(r0, 7.0e6 * 0.9, epsilon = 1.0);
    }

    #[test]
    fn keplerian_checked_solver_rejects_invalid_elements() {
        let orbit = TwoBodyKeplerian {
            semi_major_axis_m: -7.0e6,
            eccentricity: 0.1,
            mu_m3_s2: 3.986e14,
            mean_anomaly_at_epoch_rad: 0.0,
        };
        let err = orbit.eccentric_anomaly_at_checked(0.0, 64).unwrap_err();
        assert!(matches!(err, KeplerError::InvalidElements { .. }));
    }

    #[test]
    fn keplerian_checked_solver_reports_non_convergence() {
        let orbit = TwoBodyKeplerian {
            semi_major_axis_m: 7.0e6,
            eccentricity: 0.9,
            mu_m3_s2: 3.986e14,
            mean_anomaly_at_epoch_rad: 1.0,
        };
        let err = orbit.eccentric_anomaly_at_checked(100.0, 1).unwrap_err();
        assert!(matches!(err, KeplerError::DidNotConverge { .. }));
    }

    #[test]
    fn keplerian_orbital_plane_position_velocity_are_finite() {
        let orbit = TwoBodyKeplerian {
            semi_major_axis_m: 7.0e6,
            eccentricity: 0.1,
            mu_m3_s2: 3.986e14,
            mean_anomaly_at_epoch_rad: 0.2,
        };
        let position = orbit.position_in_orbital_plane_at(10.0).unwrap();
        let velocity = orbit.velocity_in_orbital_plane_at(10.0).unwrap();
        assert!(position.iter().all(|component| component.is_finite()));
        assert!(velocity.iter().all(|component| component.is_finite()));
    }
}
