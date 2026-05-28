//! Deterministic celestial ephemeris helpers.
//!
//! This module intentionally does not read SPICE / JPL DE files. It
//! provides a small HAL-portable ephemeris trait and a deterministic
//! built-in Sun/Moon approximation that is sufficient to drive
//! third-body perturbation tests and high-apogee mission studies
//! without adding file I/O to `openbmp-physics`.

use nalgebra::Vector3;
use openbmp_core::SimTime;

use crate::error::PhysicsError;

/// Astronomical unit in metres (IAU 2012 exact definition).
pub const ASTRONOMICAL_UNIT_M: f64 = 149_597_870_700.0;

/// Solar gravitational parameter, m^3/s^2.
///
/// Source: IAU 2015 nominal solar GM.
pub const SUN_MU_M3_S2: f64 = 1.327_124_4e20;

/// Lunar gravitational parameter, m^3/s^2.
///
/// Source: NASA/JPL DE-series canonical lunar GM, rounded to the
/// precision needed by this deterministic low-order model.
pub const MOON_MU_M3_S2: f64 = 4.904_869_5e12;

/// J2000 epoch as a Julian Date.
pub const J2000_JULIAN_DATE: f64 = 2_451_545.0;

const DEG_TO_RAD: f64 = core::f64::consts::PI / 180.0;

/// Celestial body supported by built-in ephemeris and third-body
/// gravity models.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum CelestialBody {
    /// The Sun.
    Sun,
    /// Earth's Moon.
    Moon,
}

impl CelestialBody {
    /// Canonical scenario label.
    #[must_use]
    pub const fn as_label(self) -> &'static str {
        match self {
            Self::Sun => "sun",
            Self::Moon => "moon",
        }
    }

    /// Gravitational parameter in m^3/s^2.
    #[must_use]
    pub const fn mu_m3_s2(self) -> f64 {
        match self {
            Self::Sun => SUN_MU_M3_S2,
            Self::Moon => MOON_MU_M3_S2,
        }
    }
}

/// Position provider for named celestial bodies.
///
/// Returned vectors are Earth-centered inertial, in metres, expressed
/// in the same ECI axes used by the scenario state.
pub trait EphemerisModel {
    /// Earth-centered inertial body position in metres.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] if the requested body is outside the
    /// model envelope or the computed state is non-finite.
    fn body_position_eci_m(
        &self,
        body: CelestialBody,
        time: SimTime,
    ) -> Result<Vector3<f64>, PhysicsError>;
}

/// Low-precision deterministic Sun/Moon ephemeris.
///
/// The formulas are compact analytical approximations around J2000:
/// the Sun path follows the standard low-precision apparent solar
/// longitude / distance approximation, and the Moon path follows a
/// first-order longitude, latitude, and distance approximation. The
/// model is appropriate for deterministic perturbation studies; it is
/// not a substitute for JPL DE / SPICE navigation products.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct LowPrecisionSunMoonEphemeris {
    epoch_julian_date: f64,
}

impl LowPrecisionSunMoonEphemeris {
    /// Construct from the scenario epoch as Julian Date.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if the epoch is not
    /// finite.
    pub fn new(epoch_julian_date: f64) -> Result<Self, PhysicsError> {
        if !epoch_julian_date.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "ephemeris epoch Julian Date must be finite",
            });
        }
        Ok(Self { epoch_julian_date })
    }

    /// J2000 epoch.
    #[must_use]
    pub const fn j2000() -> Self {
        Self {
            epoch_julian_date: J2000_JULIAN_DATE,
        }
    }

    /// Configured epoch as Julian Date.
    #[must_use]
    pub const fn epoch_julian_date(&self) -> f64 {
        self.epoch_julian_date
    }

    fn days_since_j2000(self, time: SimTime) -> f64 {
        self.epoch_julian_date - J2000_JULIAN_DATE + time.as_seconds() / 86_400.0
    }
}

impl Default for LowPrecisionSunMoonEphemeris {
    fn default() -> Self {
        Self::j2000()
    }
}

impl EphemerisModel for LowPrecisionSunMoonEphemeris {
    fn body_position_eci_m(
        &self,
        body: CelestialBody,
        time: SimTime,
    ) -> Result<Vector3<f64>, PhysicsError> {
        let days = self.days_since_j2000(time);
        if !days.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "ephemeris query time produced non-finite Julian offset",
            });
        }
        let position = match body {
            CelestialBody::Sun => low_precision_sun_eci_m(days),
            CelestialBody::Moon => low_precision_moon_eci_m(days),
        };
        if !position.iter().all(|v| v.is_finite()) {
            return Err(PhysicsError::NonFinite {
                reason: "low-precision ephemeris produced non-finite position",
            });
        }
        Ok(position)
    }
}

fn wrap_degrees(degrees: f64) -> f64 {
    degrees.rem_euclid(360.0)
}

fn sin_deg(degrees: f64) -> f64 {
    (wrap_degrees(degrees) * DEG_TO_RAD).sin()
}

fn cos_deg(degrees: f64) -> f64 {
    (wrap_degrees(degrees) * DEG_TO_RAD).cos()
}

fn low_precision_sun_eci_m(days_since_j2000: f64) -> Vector3<f64> {
    let mean_longitude_deg = wrap_degrees(280.460 + 0.985_647_4 * days_since_j2000);
    let mean_anomaly_deg = wrap_degrees(357.528 + 0.985_600_3 * days_since_j2000);
    let ecliptic_longitude_deg = mean_longitude_deg
        + 1.915 * sin_deg(mean_anomaly_deg)
        + 0.020 * sin_deg(2.0 * mean_anomaly_deg);
    let obliquity_deg = 23.439 - 0.000_000_4 * days_since_j2000;
    let radius_au = 1.000_14
        - 0.016_71 * cos_deg(mean_anomaly_deg)
        - 0.000_14 * cos_deg(2.0 * mean_anomaly_deg);
    let radius_m = radius_au * ASTRONOMICAL_UNIT_M;
    let cos_lambda = cos_deg(ecliptic_longitude_deg);
    let sin_lambda = sin_deg(ecliptic_longitude_deg);
    let cos_eps = cos_deg(obliquity_deg);
    let sin_eps = sin_deg(obliquity_deg);
    Vector3::new(
        radius_m * cos_lambda,
        radius_m * cos_eps * sin_lambda,
        radius_m * sin_eps * sin_lambda,
    )
}

fn low_precision_moon_eci_m(days_since_j2000: f64) -> Vector3<f64> {
    let mean_longitude_deg = wrap_degrees(218.316 + 13.176_396 * days_since_j2000);
    let mean_anomaly_deg = wrap_degrees(134.963 + 13.064_993 * days_since_j2000);
    let argument_of_latitude_deg = wrap_degrees(93.272 + 13.229_350 * days_since_j2000);
    let ecliptic_longitude_deg = mean_longitude_deg + 6.289 * sin_deg(mean_anomaly_deg);
    let ecliptic_latitude_deg = 5.128 * sin_deg(argument_of_latitude_deg);
    let radius_m = (385_001.0 - 20_905.0 * cos_deg(mean_anomaly_deg)) * 1_000.0;
    let obliquity_deg = 23.439 - 0.000_000_4 * days_since_j2000;

    let cos_lambda = cos_deg(ecliptic_longitude_deg);
    let sin_lambda = sin_deg(ecliptic_longitude_deg);
    let cos_beta = cos_deg(ecliptic_latitude_deg);
    let sin_beta = sin_deg(ecliptic_latitude_deg);
    let cos_eps = cos_deg(obliquity_deg);
    let sin_eps = sin_deg(obliquity_deg);

    Vector3::new(
        radius_m * cos_beta * cos_lambda,
        radius_m * (cos_beta * sin_lambda * cos_eps - sin_beta * sin_eps),
        radius_m * (cos_beta * sin_lambda * sin_eps + sin_beta * cos_eps),
    )
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn low_precision_sun_distance_is_near_one_au() {
        let ephemeris = LowPrecisionSunMoonEphemeris::j2000();
        let sun = ephemeris
            .body_position_eci_m(CelestialBody::Sun, SimTime::ZERO)
            .unwrap();
        let radius_au = sun.norm() / ASTRONOMICAL_UNIT_M;
        assert!((0.98..1.02).contains(&radius_au));
    }

    #[test]
    fn low_precision_moon_distance_is_lunar_scale() {
        let ephemeris = LowPrecisionSunMoonEphemeris::j2000();
        let moon = ephemeris
            .body_position_eci_m(CelestialBody::Moon, SimTime::ZERO)
            .unwrap();
        let radius_km = moon.norm() / 1_000.0;
        assert!((350_000.0..410_000.0).contains(&radius_km));
    }
}
