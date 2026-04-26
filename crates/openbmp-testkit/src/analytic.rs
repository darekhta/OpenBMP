//! Closed-form solutions for analytic-toy validation cases.
//!
//! Each type encapsulates the parameters of an academic textbook
//! problem and provides closed-form `position_at(t)` / `velocity_at(t)`
//! evaluations. Tests in higher-level crates can compare integrator
//! output against these references.
//!
//! No state, no IO, no RNG — pure functions only.

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
/// evaluation via the standard Kepler solution. Phase 1.6 ships only
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
    /// Mean anomaly at time `t` (seconds from epoch).
    #[must_use]
    pub fn mean_anomaly_at(&self, t: f64) -> f64 {
        let n = (self.mu_m3_s2 / self.semi_major_axis_m.powi(3)).sqrt();
        self.mean_anomaly_at_epoch_rad + n * t
    }

    /// Eccentric anomaly via Newton's method on Kepler's equation
    /// `M = E - e·sin(E)`.
    ///
    /// Converges in `max_iter` iterations to a tolerance of
    /// `1.0e-12` for `e < 1`.
    #[must_use]
    pub fn eccentric_anomaly_at(&self, t: f64, max_iter: u32) -> f64 {
        let m = self.mean_anomaly_at(t);
        let mut e = m;
        for _ in 0..max_iter {
            let f = e - self.eccentricity * e.sin() - m;
            let fp = 1.0 - self.eccentricity * e.cos();
            let delta = f / fp;
            e -= delta;
            if delta.abs() < 1.0e-12 {
                break;
            }
        }
        e
    }

    /// In-plane radius at time `t`.
    #[must_use]
    pub fn radius_at(&self, t: f64) -> f64 {
        let e_anom = self.eccentric_anomaly_at(t, 64);
        self.semi_major_axis_m * (1.0 - self.eccentricity * e_anom.cos())
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

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
}
