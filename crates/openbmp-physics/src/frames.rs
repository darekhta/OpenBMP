//! Frame profiles, contexts, and time-aware ECI ↔ ECEF ↔ NED transforms.
//!
//! `openbmp-core::frames` owns the *type-level* frame machinery — the
//! [`Frame`](openbmp_core::Frame) trait and tag types
//! ([`Eci`], [`Ecef`],
//! [`Ned`], [`Enu`](openbmp_core::Enu),
//! [`Body`](openbmp_core::Body)), the value types
//! ([`Position3`],
//! [`Velocity3`], and friends), and
//! [`Quaternion`](openbmp_core::Quaternion).
//!
//! This module owns the *physics* on top of those types — namely the
//! WGS84 ellipsoid constants, [`LocalGeodeticOrigin`], [`FrameContext`],
//! and the time-aware [`FrameTransform`] implementations between ECI,
//! ECEF and NED.
//!
//! The split keeps `openbmp-core` free of Earth-specific physics
//! constants while letting both the simulator and the FC consume the
//! same time-aware transforms through one HAL-portable crate.
//!
//! Determinism: pure `f64` arithmetic with locked operand order; no
//! FMA, no wall-clock time, no system RNG.

use nalgebra::{Matrix3, Vector3};
#[cfg(not(feature = "std"))]
use num_traits::{Euclid, Float};
use openbmp_core::{Ecef, Eci, Frame as CoreFrame, FrameError, Ned, Position3, SimTime, Velocity3};

use crate::erfa_nut00a_data::{IAU2000A_LUNI_SOLAR_TERMS, IAU2000A_PLANETARY_TERMS};

#[cfg(not(feature = "std"))]
use alloc::vec::Vec;

// ---------------------------------------------------------------------
// FrameProfile
// ---------------------------------------------------------------------

/// Active frame profile.
///
/// The profile determines which [`FrameTransform`] implementations and
/// time-aware methods are available. Provides
/// [`FrameProfile::ToyFixedEarth`],
/// [`FrameProfile::Wgs84UniformRotation`], and
/// [`FrameProfile::IersTabulated`], and [`FrameProfile::IersCio`].
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash)]
pub enum FrameProfile {
    /// Toy: ECI and ECEF coincide; no Earth rotation; no leap seconds.
    /// Used for analytic-toy scenarios that need no Earth-fixed
    /// physics.
    #[default]
    ToyFixedEarth,
    /// WGS84 with uniform Earth rotation: ECI/ECEF differ only by a
    /// rotation about the inertial `+z` axis at the constant WGS84
    /// rate `ω_e = 7.2921151467 × 10⁻⁵ rad/s`. No EOP, no polar
    /// motion, no leap seconds.
    Wgs84UniformRotation,
    /// WGS84 with scenario-pinned Earth-orientation parameters:
    /// absolute UTC epoch, interpolated UT1-UTC, and polar motion.
    /// This remains a compact deterministic transform. It includes
    /// IAU 1976 mean precession and IAU 1980 nutation from J2000 to
    /// date, but not a full SPICE frame chain.
    IersTabulated,
    /// WGS84 with scenario-pinned EOP and caller-provided CIO `X/Y/s`
    /// samples. This uses the IAU 2000 CIO matrix product
    /// `RPOM * R3(ERA) * RC2I`, but does not compute the IAU
    /// 2006/2000A X/Y/s series internally.
    IersCio,
}

impl FrameProfile {
    /// Canonical profile name as it appears in scenario files.
    #[must_use]
    pub const fn as_label(self) -> &'static str {
        match self {
            Self::ToyFixedEarth => "toy-fixed-earth",
            Self::Wgs84UniformRotation => "wgs84-uniform-rotation",
            Self::IersTabulated => "iers-tabulated",
            Self::IersCio => "iers-cio",
        }
    }
}

// ---------------------------------------------------------------------
// Earth orientation
// ---------------------------------------------------------------------

const SECONDS_PER_DAY: f64 = 86_400.0;
const J2000_JULIAN_DATE: f64 = 2_451_545.0;
const JULIAN_CENTURY_DAYS: f64 = 36_525.0;
const JULIAN_MILLENNIUM_DAYS: f64 = 365_250.0;
const TWO_PI: f64 = 2.0 * core::f64::consts::PI;
const DEG_TO_RAD: f64 = core::f64::consts::PI / 180.0;
const FULL_CIRCLE_ARCSECONDS: f64 = 1_296_000.0;
const TENTH_MILLIARCSECOND_TO_RAD: f64 = ARCSECOND_TO_RAD / 10_000.0;
const TENTH_MICROARCSECOND_TO_RAD: f64 = ARCSECOND_TO_RAD / 10_000_000.0;
const FRAME_RATE_STEP_S: f64 = 1.0;

/// Radians per arcsecond.
pub const ARCSECOND_TO_RAD: f64 = core::f64::consts::PI / (180.0 * 3_600.0);

/// Terrestrial Time minus International Atomic Time, seconds.
pub const TT_MINUS_TAI_S: f64 = 32.184;

/// IAU Earth Rotation Angle rate, rad/s.
///
/// This is the ERA slope with respect to UT1:
/// `2π * 1.00273781191135448 / 86400`.
pub const EARTH_ROTATION_ANGLE_RATE_RAD_S: f64 = TWO_PI * 1.002_737_811_911_354_6 / SECONDS_PER_DAY;

#[derive(Copy, Clone)]
struct CioS06Term {
    nfa: [i32; 8],
    sine_arcsec: f64,
    cosine_arcsec: f64,
}

impl CioS06Term {
    const fn new(nfa: [i32; 8], sine_arcsec: f64, cosine_arcsec: f64) -> Self {
        Self {
            nfa,
            sine_arcsec,
            cosine_arcsec,
        }
    }
}

const CIO_S06_POLYNOMIAL_ARCSECONDS: [f64; 6] = [
    94.00e-6,
    3808.65e-6,
    -122.68e-6,
    -72574.11e-6,
    27.98e-6,
    15.62e-6,
];

// ERFA `s06.c` series coefficients for s+XY/2. The fundamental arguments are
// l, l', F, D, Om, LVe, LE, pA.
const CIO_S06_TERMS_0: [CioS06Term; 33] = [
    CioS06Term::new([0, 0, 0, 0, 1, 0, 0, 0], -2640.73e-6, 0.39e-6),
    CioS06Term::new([0, 0, 0, 0, 2, 0, 0, 0], -63.53e-6, 0.02e-6),
    CioS06Term::new([0, 0, 2, -2, 3, 0, 0, 0], -11.75e-6, -0.01e-6),
    CioS06Term::new([0, 0, 2, -2, 1, 0, 0, 0], -11.21e-6, -0.01e-6),
    CioS06Term::new([0, 0, 2, -2, 2, 0, 0, 0], 4.57e-6, 0.00e-6),
    CioS06Term::new([0, 0, 2, 0, 3, 0, 0, 0], -2.02e-6, 0.00e-6),
    CioS06Term::new([0, 0, 2, 0, 1, 0, 0, 0], -1.98e-6, 0.00e-6),
    CioS06Term::new([0, 0, 0, 0, 3, 0, 0, 0], 1.72e-6, 0.00e-6),
    CioS06Term::new([0, 1, 0, 0, 1, 0, 0, 0], 1.41e-6, 0.01e-6),
    CioS06Term::new([0, 1, 0, 0, -1, 0, 0, 0], 1.26e-6, 0.01e-6),
    CioS06Term::new([1, 0, 0, 0, -1, 0, 0, 0], 0.63e-6, 0.00e-6),
    CioS06Term::new([1, 0, 0, 0, 1, 0, 0, 0], 0.63e-6, 0.00e-6),
    CioS06Term::new([0, 1, 2, -2, 3, 0, 0, 0], -0.46e-6, 0.00e-6),
    CioS06Term::new([0, 1, 2, -2, 1, 0, 0, 0], -0.45e-6, 0.00e-6),
    CioS06Term::new([0, 0, 4, -4, 4, 0, 0, 0], -0.36e-6, 0.00e-6),
    CioS06Term::new([0, 0, 1, -1, 1, -8, 12, 0], 0.24e-6, 0.12e-6),
    CioS06Term::new([0, 0, 2, 0, 0, 0, 0, 0], -0.32e-6, 0.00e-6),
    CioS06Term::new([0, 0, 2, 0, 2, 0, 0, 0], -0.28e-6, 0.00e-6),
    CioS06Term::new([1, 0, 2, 0, 3, 0, 0, 0], -0.27e-6, 0.00e-6),
    CioS06Term::new([1, 0, 2, 0, 1, 0, 0, 0], -0.26e-6, 0.00e-6),
    CioS06Term::new([0, 0, 2, -2, 0, 0, 0, 0], 0.21e-6, 0.00e-6),
    CioS06Term::new([0, 1, -2, 2, -3, 0, 0, 0], -0.19e-6, 0.00e-6),
    CioS06Term::new([0, 1, -2, 2, -1, 0, 0, 0], -0.18e-6, 0.00e-6),
    CioS06Term::new([0, 0, 0, 0, 0, 8, -13, -1], 0.10e-6, -0.05e-6),
    CioS06Term::new([0, 0, 0, 2, 0, 0, 0, 0], -0.15e-6, 0.00e-6),
    CioS06Term::new([2, 0, -2, 0, -1, 0, 0, 0], 0.14e-6, 0.00e-6),
    CioS06Term::new([0, 1, 2, -2, 2, 0, 0, 0], 0.14e-6, 0.00e-6),
    CioS06Term::new([1, 0, 0, -2, 1, 0, 0, 0], -0.14e-6, 0.00e-6),
    CioS06Term::new([1, 0, 0, -2, -1, 0, 0, 0], -0.14e-6, 0.00e-6),
    CioS06Term::new([0, 0, 4, -2, 4, 0, 0, 0], -0.13e-6, 0.00e-6),
    CioS06Term::new([0, 0, 2, -2, 4, 0, 0, 0], 0.11e-6, 0.00e-6),
    CioS06Term::new([1, 0, -2, 0, -3, 0, 0, 0], -0.11e-6, 0.00e-6),
    CioS06Term::new([1, 0, -2, 0, -1, 0, 0, 0], -0.11e-6, 0.00e-6),
];

const CIO_S06_TERMS_1: [CioS06Term; 3] = [
    CioS06Term::new([0, 0, 0, 0, 2, 0, 0, 0], -0.07e-6, 3.57e-6),
    CioS06Term::new([0, 0, 0, 0, 1, 0, 0, 0], 1.73e-6, -0.03e-6),
    CioS06Term::new([0, 0, 2, -2, 3, 0, 0, 0], 0.00e-6, 0.48e-6),
];

const CIO_S06_TERMS_2: [CioS06Term; 25] = [
    CioS06Term::new([0, 0, 0, 0, 1, 0, 0, 0], 743.52e-6, -0.17e-6),
    CioS06Term::new([0, 0, 2, -2, 2, 0, 0, 0], 56.91e-6, 0.06e-6),
    CioS06Term::new([0, 0, 2, 0, 2, 0, 0, 0], 9.84e-6, -0.01e-6),
    CioS06Term::new([0, 0, 0, 0, 2, 0, 0, 0], -8.85e-6, 0.01e-6),
    CioS06Term::new([0, 1, 0, 0, 0, 0, 0, 0], -6.38e-6, -0.05e-6),
    CioS06Term::new([1, 0, 0, 0, 0, 0, 0, 0], -3.07e-6, 0.00e-6),
    CioS06Term::new([0, 1, 2, -2, 2, 0, 0, 0], 2.23e-6, 0.00e-6),
    CioS06Term::new([0, 0, 2, 0, 1, 0, 0, 0], 1.67e-6, 0.00e-6),
    CioS06Term::new([1, 0, 2, 0, 2, 0, 0, 0], 1.30e-6, 0.00e-6),
    CioS06Term::new([0, 1, -2, 2, -2, 0, 0, 0], 0.93e-6, 0.00e-6),
    CioS06Term::new([1, 0, 0, -2, 0, 0, 0, 0], 0.68e-6, 0.00e-6),
    CioS06Term::new([0, 0, 2, -2, 1, 0, 0, 0], -0.55e-6, 0.00e-6),
    CioS06Term::new([1, 0, -2, 0, -2, 0, 0, 0], 0.53e-6, 0.00e-6),
    CioS06Term::new([0, 0, 0, 2, 0, 0, 0, 0], -0.27e-6, 0.00e-6),
    CioS06Term::new([1, 0, 0, 0, 1, 0, 0, 0], -0.27e-6, 0.00e-6),
    CioS06Term::new([1, 0, -2, -2, -2, 0, 0, 0], -0.26e-6, 0.00e-6),
    CioS06Term::new([1, 0, 0, 0, -1, 0, 0, 0], -0.25e-6, 0.00e-6),
    CioS06Term::new([1, 0, 2, 0, 1, 0, 0, 0], 0.22e-6, 0.00e-6),
    CioS06Term::new([2, 0, 0, -2, 0, 0, 0, 0], -0.21e-6, 0.00e-6),
    CioS06Term::new([2, 0, -2, 0, -1, 0, 0, 0], 0.20e-6, 0.00e-6),
    CioS06Term::new([0, 0, 2, 2, 2, 0, 0, 0], 0.17e-6, 0.00e-6),
    CioS06Term::new([2, 0, 2, 0, 2, 0, 0, 0], 0.13e-6, 0.00e-6),
    CioS06Term::new([2, 0, 0, 0, 0, 0, 0, 0], -0.13e-6, 0.00e-6),
    CioS06Term::new([1, 0, 2, -2, 2, 0, 0, 0], -0.12e-6, 0.00e-6),
    CioS06Term::new([0, 0, 2, 0, 0, 0, 0, 0], -0.11e-6, 0.00e-6),
];

const CIO_S06_TERMS_3: [CioS06Term; 4] = [
    CioS06Term::new([0, 0, 0, 0, 1, 0, 0, 0], 0.30e-6, -23.42e-6),
    CioS06Term::new([0, 0, 2, -2, 2, 0, 0, 0], -0.03e-6, -1.46e-6),
    CioS06Term::new([0, 0, 2, 0, 2, 0, 0, 0], -0.01e-6, -0.25e-6),
    CioS06Term::new([0, 0, 0, 0, 2, 0, 0, 0], 0.00e-6, 0.23e-6),
];

const CIO_S06_TERMS_4: [CioS06Term; 1] = [CioS06Term::new(
    [0, 0, 0, 0, 1, 0, 0, 0],
    -0.26e-6,
    -0.01e-6,
)];

/// Fukushima-Williams precession angles, radians.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct FukushimaWilliamsAngles {
    /// `gamma_bar`, the frame-bias/precession angle `epE`.
    pub gamma_bar_rad: f64,
    /// `phi_bar`, the frame-bias/precession angle `pE`.
    pub phi_bar_rad: f64,
    /// `psi_bar`, the frame-bias/precession angle `pEP`.
    pub psi_bar_rad: f64,
    /// `epsilon_A`, mean obliquity of the ecliptic.
    pub epsilon_a_rad: f64,
}

/// CIO `X`, `Y`, `s` coordinates, radians.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct CioXysCoordinates {
    /// Celestial intermediate pole `X`, radians.
    pub x_rad: f64,
    /// Celestial intermediate pole `Y`, radians.
    pub y_rad: f64,
    /// CIO locator `s`, radians.
    pub s_rad: f64,
}

/// One scenario-relative Earth-orientation sample.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct EarthOrientationSample {
    /// Scenario-relative simulation time, seconds.
    pub time_s: f64,
    /// UT1 minus UTC, seconds.
    pub ut1_minus_utc_s: f64,
    /// Polar motion `x_p`, radians.
    pub polar_motion_x_rad: f64,
    /// Polar motion `y_p`, radians.
    pub polar_motion_y_rad: f64,
    /// Celestial intermediate pole observed offset `dX`, radians.
    ///
    /// This is currently stored and interpolated for the future CIO frame path;
    /// the existing equinox-based `IersTabulated` transform does not consume it.
    pub cip_offset_x_rad: f64,
    /// Celestial intermediate pole observed offset `dY`, radians.
    ///
    /// This is currently stored and interpolated for the future CIO frame path;
    /// the existing equinox-based `IersTabulated` transform does not consume it.
    pub cip_offset_y_rad: f64,
    /// Optional excess length of day, seconds.
    ///
    /// When both bracketing samples provide `lod_s`, the table uses it
    /// as the endpoint derivative for Hermite interpolation of
    /// UT1-UTC. Omitted values preserve the legacy linear
    /// interpolation path.
    pub lod_s: Option<f64>,
}

impl EarthOrientationSample {
    /// Construct and validate a sample.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when any field
    /// is non-finite.
    pub fn new(
        time_s: f64,
        ut1_minus_utc_s: f64,
        polar_motion_x_rad: f64,
        polar_motion_y_rad: f64,
    ) -> Result<Self, FrameError> {
        if !time_s.is_finite()
            || !ut1_minus_utc_s.is_finite()
            || !polar_motion_x_rad.is_finite()
            || !polar_motion_y_rad.is_finite()
        {
            return Err(FrameError::InvalidFrameProfileData {
                reason: "Earth-orientation samples must be finite",
            });
        }
        Ok(Self {
            time_s,
            ut1_minus_utc_s,
            polar_motion_x_rad,
            polar_motion_y_rad,
            cip_offset_x_rad: 0.0,
            cip_offset_y_rad: 0.0,
            lod_s: None,
        })
    }

    /// Return this sample with celestial intermediate pole offsets.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when either offset is
    /// non-finite.
    pub fn with_cip_offsets(
        mut self,
        cip_offset_x_rad: f64,
        cip_offset_y_rad: f64,
    ) -> Result<Self, FrameError> {
        if !cip_offset_x_rad.is_finite() || !cip_offset_y_rad.is_finite() {
            return Err(FrameError::InvalidFrameProfileData {
                reason: "Earth-orientation celestial-pole-offset samples must be finite",
            });
        }
        self.cip_offset_x_rad = cip_offset_x_rad;
        self.cip_offset_y_rad = cip_offset_y_rad;
        Ok(self)
    }

    /// Construct and validate a sample with length-of-day data.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when any field
    /// is non-finite.
    pub fn new_with_lod(
        time_s: f64,
        ut1_minus_utc_s: f64,
        polar_motion_x_rad: f64,
        polar_motion_y_rad: f64,
        lod_s: f64,
    ) -> Result<Self, FrameError> {
        let mut sample = Self::new(
            time_s,
            ut1_minus_utc_s,
            polar_motion_x_rad,
            polar_motion_y_rad,
        )?;
        if !lod_s.is_finite() {
            return Err(FrameError::InvalidFrameProfileData {
                reason: "Earth-orientation length-of-day samples must be finite",
            });
        }
        sample.lod_s = Some(lod_s);
        Ok(sample)
    }

    /// Earth rotation angle rate implied by this sample, rad/s.
    ///
    /// `lod_s` is excess length of day, so positive values slow the
    /// UT1 rotation rate relative to the nominal ERA slope.
    #[must_use]
    pub fn earth_rotation_rate_rad_s(self) -> f64 {
        self.lod_s.map_or(EARTH_ROTATION_ANGLE_RATE_RAD_S, |lod_s| {
            EARTH_ROTATION_ANGLE_RATE_RAD_S * (1.0 - lod_s / SECONDS_PER_DAY)
        })
    }
}

/// Scenario-pinned Earth-orientation parameter table.
#[derive(Clone, Debug, PartialEq)]
pub struct EarthOrientationTable {
    samples: Vec<EarthOrientationSample>,
}

impl EarthOrientationTable {
    /// Construct a table from strictly increasing samples.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when the table
    /// is empty, contains non-finite values, or sample times are not
    /// strictly increasing.
    pub fn new(samples: Vec<EarthOrientationSample>) -> Result<Self, FrameError> {
        if samples.is_empty() {
            return Err(FrameError::InvalidFrameProfileData {
                reason: "Earth-orientation table must contain at least one sample",
            });
        }
        for sample in &samples {
            sample.validate()?;
        }
        for pair in samples.windows(2) {
            if pair[0].time_s >= pair[1].time_s {
                return Err(FrameError::InvalidFrameProfileData {
                    reason: "Earth-orientation sample times must be strictly increasing",
                });
            }
        }
        Ok(Self { samples })
    }

    /// Table samples.
    #[must_use]
    pub fn samples(&self) -> &[EarthOrientationSample] {
        &self.samples
    }

    /// Whether the table covers the closed interval `[start_s, stop_s]`.
    #[must_use]
    pub fn covers_interval(&self, start_s: f64, stop_s: f64) -> bool {
        let Some(first) = self.samples.first() else {
            return false;
        };
        let Some(last) = self.samples.last() else {
            return false;
        };
        start_s.is_finite()
            && stop_s.is_finite()
            && start_s <= stop_s
            && first.time_s <= start_s
            && last.time_s >= stop_s
    }

    /// Interpolate an orientation sample at scenario-relative time.
    ///
    /// Values outside the pinned table are endpoint-held; runners are
    /// expected to validate table coverage for the configured
    /// simulation interval before constructing the context.
    #[must_use]
    pub fn sample(&self, t: SimTime) -> EarthOrientationSample {
        let time_s = t.as_seconds();
        if time_s <= self.samples[0].time_s {
            return self.samples[0];
        }
        let last_index = self.samples.len() - 1;
        if time_s >= self.samples[last_index].time_s {
            return self.samples[last_index];
        }
        let upper = self
            .samples
            .partition_point(|sample| sample.time_s < time_s);
        let lo = self.samples[upper - 1];
        let hi = self.samples[upper];
        let alpha = (time_s - lo.time_s) / (hi.time_s - lo.time_s);
        EarthOrientationSample {
            time_s,
            ut1_minus_utc_s: interpolate_ut1_minus_utc(lo, hi, alpha),
            polar_motion_x_rad: lerp(lo.polar_motion_x_rad, hi.polar_motion_x_rad, alpha),
            polar_motion_y_rad: lerp(lo.polar_motion_y_rad, hi.polar_motion_y_rad, alpha),
            cip_offset_x_rad: lerp(lo.cip_offset_x_rad, hi.cip_offset_x_rad, alpha),
            cip_offset_y_rad: lerp(lo.cip_offset_y_rad, hi.cip_offset_y_rad, alpha),
            lod_s: interpolate_optional_lod(lo.lod_s, hi.lod_s, alpha),
        }
    }
}

impl EarthOrientationSample {
    fn validate(self) -> Result<(), FrameError> {
        if !self.time_s.is_finite()
            || !self.ut1_minus_utc_s.is_finite()
            || !self.polar_motion_x_rad.is_finite()
            || !self.polar_motion_y_rad.is_finite()
            || !self.cip_offset_x_rad.is_finite()
            || !self.cip_offset_y_rad.is_finite()
            || self.lod_s.is_some_and(|lod_s| !lod_s.is_finite())
        {
            return Err(FrameError::InvalidFrameProfileData {
                reason: "Earth-orientation samples must be finite",
            });
        }
        Ok(())
    }
}

/// One caller-provided CIO `X`, `Y`, `s` sample.
///
/// This stores already-computed celestial intermediate pole coordinates and
/// CIO locator values. It does not compute the IAU 2006/2000A series.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct CioXysSample {
    /// Scenario-relative simulation time, seconds.
    pub time_s: f64,
    /// Celestial intermediate pole `X`, radians.
    pub x_rad: f64,
    /// Celestial intermediate pole `Y`, radians.
    pub y_rad: f64,
    /// CIO locator `s`, radians.
    pub s_rad: f64,
}

impl CioXysSample {
    /// Construct and validate a caller-provided CIO `X`, `Y`, `s` sample.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when any field is
    /// non-finite or when `X^2 + Y^2 >= 1`.
    pub fn new(time_s: f64, x_rad: f64, y_rad: f64, s_rad: f64) -> Result<Self, FrameError> {
        finite_cio_input(time_s)?;
        finite_cio_input(x_rad)?;
        finite_cio_input(y_rad)?;
        finite_cio_input(s_rad)?;
        validate_cio_xy(x_rad, y_rad)?;
        Ok(Self {
            time_s,
            x_rad,
            y_rad,
            s_rad,
        })
    }
}

/// Scenario-pinned table of caller-provided CIO `X`, `Y`, `s` samples.
///
/// Values outside the pinned table are endpoint-held. This table is a substrate
/// for precomputed or fixture-supplied X/Y/s data and makes no claim that the
/// IAU 2006/2000A series is implemented.
#[derive(Clone, Debug, PartialEq)]
pub struct CioXysTable {
    samples: Vec<CioXysSample>,
}

impl CioXysTable {
    /// Construct a table from strictly increasing samples.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when the table is empty,
    /// contains invalid samples, or sample times are not strictly increasing.
    pub fn new(samples: Vec<CioXysSample>) -> Result<Self, FrameError> {
        if samples.is_empty() {
            return Err(FrameError::InvalidFrameProfileData {
                reason: "CIO X/Y/s table must contain at least one sample",
            });
        }
        for sample in &samples {
            validate_cio_xys_sample(*sample)?;
        }
        for pair in samples.windows(2) {
            if pair[0].time_s >= pair[1].time_s {
                return Err(FrameError::InvalidFrameProfileData {
                    reason: "CIO X/Y/s sample times must be strictly increasing",
                });
            }
        }
        Ok(Self { samples })
    }

    /// Table samples.
    #[must_use]
    pub fn samples(&self) -> &[CioXysSample] {
        &self.samples
    }

    /// Whether the table covers the closed interval `[start_s, stop_s]`.
    #[must_use]
    pub fn covers_interval(&self, start_s: f64, stop_s: f64) -> bool {
        let Some(first) = self.samples.first() else {
            return false;
        };
        let Some(last) = self.samples.last() else {
            return false;
        };
        start_s.is_finite()
            && stop_s.is_finite()
            && start_s <= stop_s
            && first.time_s <= start_s
            && last.time_s >= stop_s
    }

    /// Interpolate a CIO `X`, `Y`, `s` sample at scenario-relative time.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when `t` is non-finite.
    pub fn sample(&self, t: SimTime) -> Result<CioXysSample, FrameError> {
        let time_s = t.as_seconds();
        finite_cio_input(time_s)?;
        if time_s <= self.samples[0].time_s {
            return Ok(self.samples[0]);
        }
        let last_index = self.samples.len() - 1;
        if time_s >= self.samples[last_index].time_s {
            return Ok(self.samples[last_index]);
        }
        let upper = self
            .samples
            .partition_point(|sample| sample.time_s < time_s);
        let lo = self.samples[upper - 1];
        let hi = self.samples[upper];
        let alpha = (time_s - lo.time_s) / (hi.time_s - lo.time_s);
        let sample = CioXysSample {
            time_s,
            x_rad: lerp(lo.x_rad, hi.x_rad, alpha),
            y_rad: lerp(lo.y_rad, hi.y_rad, alpha),
            s_rad: lerp(lo.s_rad, hi.s_rad, alpha),
        };
        validate_cio_xys_sample(sample)?;
        Ok(sample)
    }
}

/// Fully sampled CIO frame inputs at one scenario-relative time.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct CioFrameSample {
    /// Scenario-relative simulation time, seconds.
    pub time_s: f64,
    /// UTC Julian Date for the sample.
    pub utc_julian_date: f64,
    /// UT1 Julian Date for ERA.
    pub ut1_julian_date: f64,
    /// TT Julian Date for the TIO locator `s'`.
    pub tt_julian_date: f64,
    /// Caller-provided `X` plus EOP `dX`, radians.
    pub x_rad: f64,
    /// Caller-provided `Y` plus EOP `dY`, radians.
    pub y_rad: f64,
    /// Caller-provided CIO locator `s`, radians.
    pub s_rad: f64,
    /// Interpolated Earth-orientation sample used for EOP, UT1, and polar motion.
    pub earth_orientation: EarthOrientationSample,
}

#[derive(Clone, Debug, PartialEq)]
enum CioXysSource {
    Table(CioXysTable),
    Iau2006a,
}

/// CIO frame substrate backed by pinned EOP and CIO `X`, `Y`, `s`.
///
/// This composes the ERFA-pinned CIO primitives into a GCRS-to-ITRS matrix:
/// `RPOM * R3(ERA) * RC2I`. The model adds EOP `dX` and `dY` offsets to the
/// sampled or generated `X` and `Y` values, computes ERA from UT1, and
/// computes the TIO locator `s'` from TT.
#[derive(Clone, Debug, PartialEq)]
pub struct CioFrameModel {
    epoch_utc_julian_date: f64,
    tai_minus_utc_s: f64,
    earth_orientation: EarthOrientationTable,
    xys: CioXysSource,
}

impl CioFrameModel {
    /// Construct a CIO frame model from a UTC epoch, leap-second offset, EOP
    /// table, and caller-provided CIO `X`, `Y`, `s` table.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when the UTC epoch or
    /// `TAI-UTC` offset is non-finite.
    pub fn new(
        epoch_utc_julian_date: f64,
        tai_minus_utc_s: f64,
        earth_orientation: EarthOrientationTable,
        xys: CioXysTable,
    ) -> Result<Self, FrameError> {
        finite_cio_input(epoch_utc_julian_date)?;
        finite_cio_input(tai_minus_utc_s)?;
        Ok(Self {
            epoch_utc_julian_date,
            tai_minus_utc_s,
            earth_orientation,
            xys: CioXysSource::Table(xys),
        })
    }

    /// Construct a CIO frame model that generates IAU 2006/2000A `X`, `Y`, `s`
    /// internally from TT.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when the UTC epoch or
    /// `TAI-UTC` offset is non-finite.
    pub fn iau2006a(
        epoch_utc_julian_date: f64,
        tai_minus_utc_s: f64,
        earth_orientation: EarthOrientationTable,
    ) -> Result<Self, FrameError> {
        finite_cio_input(epoch_utc_julian_date)?;
        finite_cio_input(tai_minus_utc_s)?;
        Ok(Self {
            epoch_utc_julian_date,
            tai_minus_utc_s,
            earth_orientation,
            xys: CioXysSource::Iau2006a,
        })
    }

    /// UTC epoch Julian Date for scenario time `t = 0`.
    #[must_use]
    pub const fn epoch_utc_julian_date(&self) -> f64 {
        self.epoch_utc_julian_date
    }

    /// Pinned `TAI-UTC` offset, seconds.
    #[must_use]
    pub const fn tai_minus_utc_s(&self) -> f64 {
        self.tai_minus_utc_s
    }

    /// Pinned Earth-orientation table.
    #[must_use]
    pub const fn earth_orientation_table(&self) -> &EarthOrientationTable {
        &self.earth_orientation
    }

    /// Caller-provided CIO `X`, `Y`, `s` table, when this model is table-backed.
    #[must_use]
    pub const fn xys_table(&self) -> Option<&CioXysTable> {
        match &self.xys {
            CioXysSource::Table(table) => Some(table),
            CioXysSource::Iau2006a => None,
        }
    }

    /// Whether this model generates IAU 2006/2000A `X`, `Y`, `s` internally.
    #[must_use]
    pub const fn uses_generated_iau2006a_xys(&self) -> bool {
        matches!(&self.xys, CioXysSource::Iau2006a)
    }

    /// Build all sampled CIO frame inputs for a scenario-relative time.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when the time or sampled
    /// combined `X`/`Y` values are invalid.
    pub fn sample(&self, t: SimTime) -> Result<CioFrameSample, FrameError> {
        let time_s = t.as_seconds();
        finite_cio_input(time_s)?;
        let earth_orientation = self.earth_orientation.sample(t);
        earth_orientation.validate()?;
        let bridge = TimeScaleBridge::from_earth_orientation_sample(
            self.tai_minus_utc_s,
            earth_orientation,
        )?;
        let utc_julian_date = self.epoch_utc_julian_date + time_s / SECONDS_PER_DAY;
        let ut1_julian_date = bridge.utc_julian_date_to_ut1(utc_julian_date)?;
        let tt_julian_date = bridge.utc_julian_date_to_tt(utc_julian_date)?;
        let xys = match &self.xys {
            CioXysSource::Table(table) => {
                let sample = table.sample(t)?;
                CioXysCoordinates {
                    x_rad: sample.x_rad,
                    y_rad: sample.y_rad,
                    s_rad: sample.s_rad,
                }
            }
            CioXysSource::Iau2006a => cio_xys_iau2006a(tt_julian_date)?,
        };
        let x_rad = xys.x_rad + earth_orientation.cip_offset_x_rad;
        let y_rad = xys.y_rad + earth_orientation.cip_offset_y_rad;
        validate_cio_xy(x_rad, y_rad)?;
        Ok(CioFrameSample {
            time_s,
            utc_julian_date,
            ut1_julian_date,
            tt_julian_date,
            x_rad,
            y_rad,
            s_rad: xys.s_rad,
            earth_orientation,
        })
    }

    /// Compute the GCRS-to-ITRS CIO matrix at a scenario-relative time.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when any sampled input is
    /// invalid.
    pub fn gcrs_to_itrs_matrix(&self, t: SimTime) -> Result<Matrix3<f64>, FrameError> {
        let sample = self.sample(t)?;
        let rc2i = cio_celestial_to_intermediate_matrix(sample.x_rad, sample.y_rad, sample.s_rad)?;
        let era = earth_rotation_angle_iau2000(sample.ut1_julian_date)?;
        let sp = tio_locator_sp00(sample.tt_julian_date)?;
        let rpom = polar_motion_matrix_iau2000(
            sample.earth_orientation.polar_motion_x_rad,
            sample.earth_orientation.polar_motion_y_rad,
            sp,
        )?;
        cio_celestial_to_terrestrial_matrix(rc2i, era, rpom)
    }

    /// Compute the ITRS-to-GCRS CIO matrix at a scenario-relative time.
    ///
    /// # Errors
    ///
    /// Forwards validation errors from [`Self::gcrs_to_itrs_matrix`].
    pub fn itrs_to_gcrs_matrix(&self, t: SimTime) -> Result<Matrix3<f64>, FrameError> {
        Ok(self.gcrs_to_itrs_matrix(t)?.transpose())
    }
}

/// Scenario-pinned UTC/UT1/TAI/TT/TDB conversion helper.
///
/// This bridge stores the leap-second-derived `TAI-UTC` offset and the
/// EOP-derived `UT1-UTC` offset that apply at one epoch. It provides the
/// deterministic compact TT→TDB approximation already used by runner celestial
/// ingestion. It is not the full ERFA `eraDtdb` topocentric series.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct TimeScaleBridge {
    tai_minus_utc_s: f64,
    ut1_minus_utc_s: f64,
}

impl TimeScaleBridge {
    /// Construct a bridge from pinned `TAI-UTC` and `UT1-UTC` offsets.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when either offset is
    /// non-finite.
    pub fn new(tai_minus_utc_s: f64, ut1_minus_utc_s: f64) -> Result<Self, FrameError> {
        if !tai_minus_utc_s.is_finite() || !ut1_minus_utc_s.is_finite() {
            return Err(FrameError::InvalidFrameProfileData {
                reason: "time-scale bridge offsets must be finite",
            });
        }
        Ok(Self {
            tai_minus_utc_s,
            ut1_minus_utc_s,
        })
    }

    /// Construct a bridge from a leap-second offset and an EOP sample.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when the leap-second
    /// offset or sample fields are non-finite.
    pub fn from_earth_orientation_sample(
        tai_minus_utc_s: f64,
        sample: EarthOrientationSample,
    ) -> Result<Self, FrameError> {
        sample.validate()?;
        Self::new(tai_minus_utc_s, sample.ut1_minus_utc_s)
    }

    /// Pinned `TAI-UTC` offset, seconds.
    #[must_use]
    pub const fn tai_minus_utc_s(self) -> f64 {
        self.tai_minus_utc_s
    }

    /// Pinned `UT1-UTC` offset, seconds.
    #[must_use]
    pub const fn ut1_minus_utc_s(self) -> f64 {
        self.ut1_minus_utc_s
    }

    /// Convert a UTC-relative second offset to UT1-relative seconds.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when `utc_s` is
    /// non-finite.
    pub fn utc_seconds_to_ut1(self, utc_s: f64) -> Result<f64, FrameError> {
        finite_time_scale_input(utc_s)?;
        Ok(utc_s + self.ut1_minus_utc_s)
    }

    /// Convert a UTC-relative second offset to TAI-relative seconds.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when `utc_s` is
    /// non-finite.
    pub fn utc_seconds_to_tai(self, utc_s: f64) -> Result<f64, FrameError> {
        finite_time_scale_input(utc_s)?;
        Ok(utc_s + self.tai_minus_utc_s)
    }

    /// Convert a UTC-relative second offset to TT-relative seconds.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when `utc_s` is
    /// non-finite.
    pub fn utc_seconds_to_tt(self, utc_s: f64) -> Result<f64, FrameError> {
        finite_time_scale_input(utc_s)?;
        Ok(utc_s + self.tai_minus_utc_s + TT_MINUS_TAI_S)
    }

    /// Convert a UTC Julian Date to a UT1 Julian Date.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when `utc_julian_date`
    /// is non-finite.
    pub fn utc_julian_date_to_ut1(self, utc_julian_date: f64) -> Result<f64, FrameError> {
        finite_time_scale_input(utc_julian_date)?;
        Ok(utc_julian_date + self.ut1_minus_utc_s / SECONDS_PER_DAY)
    }

    /// Convert a UTC Julian Date to a TT Julian Date.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when `utc_julian_date`
    /// is non-finite.
    pub fn utc_julian_date_to_tt(self, utc_julian_date: f64) -> Result<f64, FrameError> {
        finite_time_scale_input(utc_julian_date)?;
        Ok(utc_julian_date + (self.tai_minus_utc_s + TT_MINUS_TAI_S) / SECONDS_PER_DAY)
    }

    /// Approximate `TDB-TT` at a TT Julian Date, seconds.
    ///
    /// This uses the compact two-term periodic approximation:
    /// `0.001657 sin(g) + 0.00001385 sin(2g)`, with
    /// `g = 357.53° + 0.9856003° * (JD_TT - J2000)`.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when `tt_julian_date`
    /// is non-finite.
    pub fn tdb_minus_tt_approx_s(tt_julian_date: f64) -> Result<f64, FrameError> {
        finite_time_scale_input(tt_julian_date)?;
        let days_since_j2000 = tt_julian_date - J2000_JULIAN_DATE;
        let mean_anomaly_rad = (357.53 + 0.985_600_3 * days_since_j2000) * DEG_TO_RAD;
        Ok(0.001_657 * mean_anomaly_rad.sin() + 0.000_013_85 * (2.0 * mean_anomaly_rad).sin())
    }

    /// Convert a TT Julian Date to an approximate TDB Julian Date.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when `tt_julian_date`
    /// is non-finite.
    pub fn tt_julian_date_to_tdb_approx(tt_julian_date: f64) -> Result<f64, FrameError> {
        Ok(tt_julian_date + Self::tdb_minus_tt_approx_s(tt_julian_date)? / SECONDS_PER_DAY)
    }

    /// ERFA `eraDtdb`-compatible approximation to `TDB-TT`, seconds.
    ///
    /// `tt_julian_date` is the TT Julian Date. `ut1_day_fraction` is the UT1
    /// fraction of one day. `longitude_rad` is east-positive geodetic
    /// longitude. `distance_spin_axis_km` is the observer distance from the
    /// Earth spin axis, and `distance_north_equator_km` is the distance north
    /// of the equatorial plane. Passing zero for both distances returns the
    /// geocentric Fairhead-Bretagnon term only.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when any input is
    /// non-finite.
    pub fn tdb_minus_tt_erfa_approx_s(
        tt_julian_date: f64,
        ut1_day_fraction: f64,
        longitude_rad: f64,
        distance_spin_axis_km: f64,
        distance_north_equator_km: f64,
    ) -> Result<f64, FrameError> {
        finite_time_scale_input(tt_julian_date)?;
        finite_time_scale_input(ut1_day_fraction)?;
        finite_time_scale_input(longitude_rad)?;
        finite_time_scale_input(distance_spin_axis_km)?;
        finite_time_scale_input(distance_north_equator_km)?;
        Ok(erfa_dtdb_approx_s(
            tt_julian_date,
            ut1_day_fraction,
            longitude_rad,
            distance_spin_axis_km,
            distance_north_equator_km,
        ))
    }

    /// Convert a TT Julian Date to an ERFA `eraDtdb`-compatible TDB Julian Date.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when any input is
    /// non-finite.
    pub fn tt_julian_date_to_tdb_erfa_approx(
        tt_julian_date: f64,
        ut1_day_fraction: f64,
        longitude_rad: f64,
        distance_spin_axis_km: f64,
        distance_north_equator_km: f64,
    ) -> Result<f64, FrameError> {
        Ok(tt_julian_date
            + Self::tdb_minus_tt_erfa_approx_s(
                tt_julian_date,
                ut1_day_fraction,
                longitude_rad,
                distance_spin_axis_km,
                distance_north_equator_km,
            )? / SECONDS_PER_DAY)
    }

    /// Convert a UTC Julian Date to an approximate TDB Julian Date.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when `utc_julian_date`
    /// is non-finite.
    pub fn utc_julian_date_to_tdb_approx(self, utc_julian_date: f64) -> Result<f64, FrameError> {
        Self::tt_julian_date_to_tdb_approx(self.utc_julian_date_to_tt(utc_julian_date)?)
    }

    /// Convert a UTC Julian Date to an ERFA `eraDtdb`-compatible TDB Julian Date.
    ///
    /// The bridge supplies `UT1-UTC` and `TAI-UTC`; the caller supplies
    /// topocentric observer geometry. Passing zero for both distance arguments
    /// returns the geocentric Fairhead-Bretagnon term only.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when any input is
    /// non-finite.
    pub fn utc_julian_date_to_tdb_erfa_approx(
        self,
        utc_julian_date: f64,
        longitude_rad: f64,
        distance_spin_axis_km: f64,
        distance_north_equator_km: f64,
    ) -> Result<f64, FrameError> {
        let tt_julian_date = self.utc_julian_date_to_tt(utc_julian_date)?;
        let ut1_julian_date = self.utc_julian_date_to_ut1(utc_julian_date)?;
        let ut1_day_fraction = ut1_julian_date % 1.0;
        Self::tt_julian_date_to_tdb_erfa_approx(
            tt_julian_date,
            ut1_day_fraction,
            longitude_rad,
            distance_spin_axis_km,
            distance_north_equator_km,
        )
    }
}

fn finite_time_scale_input(value: f64) -> Result<(), FrameError> {
    if !value.is_finite() {
        return Err(FrameError::InvalidFrameProfileData {
            reason: "time-scale bridge inputs must be finite",
        });
    }
    Ok(())
}

fn erfa_dtdb_approx_s(
    tt_julian_date: f64,
    ut1_day_fraction: f64,
    longitude_rad: f64,
    distance_spin_axis_km: f64,
    distance_north_equator_km: f64,
) -> f64 {
    // ERFA `eraDtdb`, Copyright (C) 2013-2023 NumFOCUS Foundation,
    // derived with permission from SOFA. This is a direct Rust transcription of
    // the Fairhead-Bretagnon polynomial summation and Moyer/Murray topocentric
    // correction structure, using the coefficient table in `erfa_dtdb_data`.
    let t = (tt_julian_date - J2000_JULIAN_DATE) / JULIAN_MILLENNIUM_DAYS;
    let tsol = (ut1_day_fraction % 1.0) * TWO_PI + longitude_rad;
    let w = t / 3_600.0;
    let elsun = ((280.466_456_83 + 1_296_027_711.034_29 * w) % 360.0) * DEG_TO_RAD;
    let emsun = ((357.529_109_18 + 1_295_965_810.481 * w) % 360.0) * DEG_TO_RAD;
    let d = ((297.850_195_47 + 16_029_616_012.090 * w) % 360.0) * DEG_TO_RAD;
    let elj = ((34.351_518_74 + 109_306_899.894_53 * w) % 360.0) * DEG_TO_RAD;
    let els = ((50.077_444_30 + 44_046_398.470_38 * w) % 360.0) * DEG_TO_RAD;

    let topocentric_s = 0.000_29e-10 * distance_spin_axis_km * (tsol + elsun - els).sin()
        + 0.001_00e-10 * distance_spin_axis_km * (tsol - 2.0 * emsun).sin()
        + 0.001_33e-10 * distance_spin_axis_km * (tsol - d).sin()
        + 0.001_33e-10 * distance_spin_axis_km * (tsol + elsun - elj).sin()
        - 0.002_29e-10 * distance_spin_axis_km * (tsol + 2.0 * elsun + emsun).sin()
        - 0.022_00e-10 * distance_north_equator_km * (elsun + emsun).cos()
        + 0.053_12e-10 * distance_spin_axis_km * (tsol - emsun).sin()
        - 0.136_77e-10 * distance_spin_axis_km * (tsol + 2.0 * elsun).sin()
        - 1.318_40e-10 * distance_north_equator_km * elsun.cos()
        + 3.176_79e-10 * distance_spin_axis_km * tsol.sin();

    let w0 = erfa_dtdb_series_sum(473, 0, t);
    let w1 = erfa_dtdb_series_sum(678, 474, t);
    let w2 = erfa_dtdb_series_sum(763, 679, t);
    let w3 = erfa_dtdb_series_sum(783, 764, t);
    let w4 = erfa_dtdb_series_sum(786, 784, t);
    let fairhead_bretagnon_s = t * (t * (t * (t * w4 + w3) + w2) + w1) + w0;
    let jpl_mass_adjustment_s = 0.000_65e-6 * (6_069.776_754 * t + 4.021_194).sin()
        + 0.000_33e-6 * (213.299_095 * t + 5.543_132).sin()
        - 0.001_96e-6 * (6_208.294_251 * t + 5.696_701).sin()
        - 0.001_73e-6 * (74.781_599 * t + 2.435_900).sin()
        + 0.036_38e-6 * t * t;

    topocentric_s + fairhead_bretagnon_s + jpl_mass_adjustment_s
}

fn erfa_dtdb_series_sum(start: usize, end: usize, t: f64) -> f64 {
    let mut sum = 0.0;
    for index in (end..=start).rev() {
        let (amplitude_s, frequency_rad_per_millennium, phase_rad) =
            crate::erfa_dtdb_data::FAIRHEAD_TERMS[index];
        sum += amplitude_s * (frequency_rad_per_millennium * t + phase_rad).sin();
    }
    sum
}

#[derive(Clone, Debug, PartialEq)]
struct IersFrameData {
    epoch_utc_julian_date: f64,
    earth_orientation: EarthOrientationTable,
}

// ---------------------------------------------------------------------
// WGS84 geodetic constants
//
// Defining-parameter values from NIMA TR 8350.2, *Department of
// Defense World Geodetic System 1984*, 3rd ed. (2000), tables 3.1
// and 3.5. Cited at the live NGA WGS84 portal:
// https://earth-info.nga.mil/index.php?dir=wgs84&action=wgs84.
//
// J2 lives in `crate::gravity`; it's a gravity-model coefficient and
// not a frame primitive. Earth rotation rate, semi-major axis,
// flattening, and gravitational parameter live here because they
// describe the rotating reference ellipsoid every frame profile in
// this crate is anchored against.
// ---------------------------------------------------------------------

/// WGS84 semi-major axis (equatorial radius), `a`, in metres.
pub const WGS84_A_M: f64 = 6_378_137.0;

/// WGS84 inverse flattening, `1/f`. Dimensionless.
pub const WGS84_INV_FLATTENING: f64 = 298.257_223_563;

/// WGS84 flattening `f = 1 / WGS84_INV_FLATTENING`. Dimensionless.
pub const WGS84_FLATTENING: f64 = 1.0 / WGS84_INV_FLATTENING;

/// WGS84 first-eccentricity squared, `e² = f (2 − f)`. Dimensionless.
pub const WGS84_ECCENTRICITY_SQUARED: f64 = WGS84_FLATTENING * (2.0 - WGS84_FLATTENING);

/// WGS84 gravitational parameter `µ = G · M`, in m³/s².
pub const WGS84_MU_M3_S2: f64 = 3.986_004_418e14;

/// WGS84 mean Earth angular rotation rate, `ω_e`, in rad/s.
pub const WGS84_OMEGA_RAD_S: f64 = 7.292_115_146_7e-5;

/// Closed-form IMU truth for a point fixed in a uniformly rotating frame.
///
/// Values are expressed in the inertial ECI axes. Callers that need a local
/// or body sensor frame should rotate these vectors into that frame after this
/// evaluation.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct StationaryRotatingFrameImuTruth {
    /// Non-gravitational specific force required to hold the point stationary
    /// in the rotating frame, expressed in ECI axes, m/s^2.
    pub specific_force_eci_m_s2: Vector3<f64>,
    /// Angular velocity sensed by an ideal gyro fixed in the rotating frame,
    /// expressed in ECI axes, rad/s.
    pub angular_velocity_eci_rad_s: Vector3<f64>,
}

/// Evaluate ideal IMU truth for a point fixed in a uniformly rotating frame.
///
/// `position_eci` is the point's inertial position, `gravity_eci_m_s2` is the
/// gravitational acceleration from the selected gravity model at that position,
/// and `angular_velocity_rad_s` is the frame spin about inertial `+z`.
///
/// The stationary point has inertial acceleration
/// `omega x (omega x r)`. An ideal accelerometer measures the non-gravity
/// support specific force `a - g`, while an ideal gyro measures the frame spin.
///
/// # Errors
///
/// Returns [`FrameError::NotFinite`] for a non-finite position and
/// [`FrameError::InvalidFrameProfileData`] for non-finite gravity, spin, or
/// output values.
pub fn stationary_rotating_frame_imu_truth_eci(
    position_eci: Position3<Eci>,
    gravity_eci_m_s2: Vector3<f64>,
    angular_velocity_rad_s: f64,
) -> Result<StationaryRotatingFrameImuTruth, FrameError> {
    let position_eci = position_eci.require_finite()?;
    if !gravity_eci_m_s2.iter().all(|value| value.is_finite()) {
        return Err(FrameError::InvalidFrameProfileData {
            reason: "stationary rotating-frame gravity vector must be finite",
        });
    }
    if !angular_velocity_rad_s.is_finite() {
        return Err(FrameError::InvalidFrameProfileData {
            reason: "stationary rotating-frame angular velocity must be finite",
        });
    }

    let omega_eci_rad_s = Vector3::new(0.0, 0.0, angular_velocity_rad_s);
    let inertial_accel_eci_m_s2 =
        omega_eci_rad_s.cross(&omega_eci_rad_s.cross(&position_eci.vector));
    let specific_force_eci_m_s2 = inertial_accel_eci_m_s2 - gravity_eci_m_s2;

    if !specific_force_eci_m_s2
        .iter()
        .all(|value| value.is_finite())
    {
        return Err(FrameError::InvalidFrameProfileData {
            reason: "stationary rotating-frame IMU truth produced non-finite output",
        });
    }

    Ok(StationaryRotatingFrameImuTruth {
        specific_force_eci_m_s2,
        angular_velocity_eci_rad_s: omega_eci_rad_s,
    })
}

// ---------------------------------------------------------------------
// LocalGeodeticOrigin
// ---------------------------------------------------------------------

/// Scenario-declared local geodetic origin used by the NED helpers.
///
/// All angles are stored in radians. Convenience constructors accept
/// degrees. Values are validated at construction:
/// `latitude_rad ∈ [-π/2, π/2]`, `longitude_rad ∈ [-π, π]`, height
/// finite. The origin is intended to be set once at scenario start
/// and treated as immutable thereafter.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct LocalGeodeticOrigin {
    /// Geodetic latitude in radians.
    pub latitude_rad: f64,
    /// Geodetic longitude in radians.
    pub longitude_rad: f64,
    /// WGS84 ellipsoidal height in metres.
    pub height_m: f64,
}

impl LocalGeodeticOrigin {
    /// Construct from radians and metres, validating the envelope.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidGeodeticCoordinate`] when latitude
    /// is outside `[-π/2, π/2]`, longitude outside `[-π, π]`, or
    /// height is `NaN` / infinite.
    pub fn new_radians(
        latitude_rad: f64,
        longitude_rad: f64,
        height_m: f64,
    ) -> Result<Self, FrameError> {
        if !latitude_rad.is_finite() || latitude_rad.abs() > core::f64::consts::FRAC_PI_2 {
            return Err(FrameError::InvalidGeodeticCoordinate {
                reason: "latitude must be finite and in [-π/2, π/2] rad",
            });
        }
        if !longitude_rad.is_finite() || longitude_rad.abs() > core::f64::consts::PI {
            return Err(FrameError::InvalidGeodeticCoordinate {
                reason: "longitude must be finite and in [-π, π] rad",
            });
        }
        if !height_m.is_finite() {
            return Err(FrameError::InvalidGeodeticCoordinate {
                reason: "height must be finite",
            });
        }
        Ok(Self {
            latitude_rad,
            longitude_rad,
            height_m,
        })
    }

    /// Construct from degrees and metres, validating the envelope.
    ///
    /// # Errors
    ///
    /// Forwards [`FrameError::InvalidGeodeticCoordinate`] from
    /// [`Self::new_radians`].
    pub fn new_degrees(
        latitude_deg: f64,
        longitude_deg: f64,
        height_m: f64,
    ) -> Result<Self, FrameError> {
        Self::new_radians(
            latitude_deg.to_radians(),
            longitude_deg.to_radians(),
            height_m,
        )
    }

    /// Convert this origin to its ECEF position via the standard
    /// WGS84 ellipsoid mapping (no iteration; closed form for the
    /// forward direction).
    #[must_use]
    pub fn to_ecef_position(self) -> Position3<Ecef> {
        let sin_lat = self.latitude_rad.sin();
        let cos_lat = self.latitude_rad.cos();
        let sin_lon = self.longitude_rad.sin();
        let cos_lon = self.longitude_rad.cos();
        let n = WGS84_A_M / (1.0 - WGS84_ECCENTRICITY_SQUARED * sin_lat * sin_lat).sqrt();
        // DETERMINISM: locked left-to-right scalar arithmetic, no FMA.
        let nh = n + self.height_m;
        let x = nh * cos_lat * cos_lon;
        let y = nh * cos_lat * sin_lon;
        let z = (n * (1.0 - WGS84_ECCENTRICITY_SQUARED) + self.height_m) * sin_lat;
        Position3::new(x, y, z)
    }
}

/// Snapshotted frame context for a scenario.
///
/// Created at scenario start and then immutable. The active profile
/// determines which transforms exist; an optional local geodetic
/// origin anchors NED-frame transforms in the
/// [`FrameProfile::Wgs84UniformRotation`] profile.
///
/// Higher-fidelity profiles may additionally carry pinned
/// Earth-orientation tables.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FrameContext {
    profile: FrameProfile,
    local_origin: Option<LocalGeodeticOrigin>,
    iers: Option<IersFrameData>,
    cio: Option<CioFrameModel>,
}

impl FrameContext {
    /// Construct a context for [`FrameProfile::ToyFixedEarth`].
    #[must_use]
    pub const fn toy_fixed_earth() -> Self {
        Self {
            profile: FrameProfile::ToyFixedEarth,
            local_origin: None,
            iers: None,
            cio: None,
        }
    }

    /// Construct a context for [`FrameProfile::Wgs84UniformRotation`].
    ///
    /// `local_origin` is optional — pass `None` for ECI / ECEF
    /// transforms only, `Some(...)` to additionally enable the NED
    /// helpers anchored at that origin.
    #[must_use]
    pub const fn wgs84_uniform_rotation(local_origin: Option<LocalGeodeticOrigin>) -> Self {
        Self {
            profile: FrameProfile::Wgs84UniformRotation,
            local_origin,
            iers: None,
            cio: None,
        }
    }

    /// Construct a context for [`FrameProfile::IersTabulated`].
    ///
    /// `epoch_utc_julian_date` is the scenario UTC epoch expressed as
    /// Julian Date; the Earth-orientation table is sampled against the
    /// same scenario-relative [`SimTime`] values the kernel advances.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when the epoch
    /// is non-finite.
    pub fn iers_tabulated(
        epoch_utc_julian_date: f64,
        local_origin: Option<LocalGeodeticOrigin>,
        earth_orientation: EarthOrientationTable,
    ) -> Result<Self, FrameError> {
        if !epoch_utc_julian_date.is_finite() {
            return Err(FrameError::InvalidFrameProfileData {
                reason: "IERS frame epoch Julian Date must be finite",
            });
        }
        Ok(Self {
            profile: FrameProfile::IersTabulated,
            local_origin,
            iers: Some(IersFrameData {
                epoch_utc_julian_date,
                earth_orientation,
            }),
            cio: None,
        })
    }

    /// Construct a context for [`FrameProfile::IersCio`].
    ///
    /// `epoch_utc_julian_date` is the scenario UTC epoch expressed as Julian
    /// Date. The Earth-orientation table and CIO X/Y/s table are sampled
    /// against scenario-relative [`SimTime`] values. The `X/Y/s` values are
    /// caller-provided; this constructor does not compute the IAU 2006/2000A
    /// series.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when the epoch,
    /// `TAI-UTC` offset, or model inputs are invalid.
    pub fn iers_cio(
        epoch_utc_julian_date: f64,
        tai_minus_utc_s: f64,
        local_origin: Option<LocalGeodeticOrigin>,
        earth_orientation: EarthOrientationTable,
        xys: CioXysTable,
    ) -> Result<Self, FrameError> {
        if !epoch_utc_julian_date.is_finite() {
            return Err(FrameError::InvalidFrameProfileData {
                reason: "IERS CIO frame epoch Julian Date must be finite",
            });
        }
        let cio = CioFrameModel::new(
            epoch_utc_julian_date,
            tai_minus_utc_s,
            earth_orientation.clone(),
            xys,
        )?;
        Ok(Self {
            profile: FrameProfile::IersCio,
            local_origin,
            iers: Some(IersFrameData {
                epoch_utc_julian_date,
                earth_orientation,
            }),
            cio: Some(cio),
        })
    }

    /// Construct a context for [`FrameProfile::IersCio`] that generates
    /// IAU 2006/2000A CIO `X`, `Y`, `s` internally from TT.
    ///
    /// `epoch_utc_julian_date` is the scenario UTC epoch expressed as Julian
    /// Date. The Earth-orientation table is sampled against scenario-relative
    /// [`SimTime`] values and supplies UT1-UTC, polar motion, and dX/dY offsets.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidFrameProfileData`] when the epoch,
    /// `TAI-UTC` offset, or model inputs are invalid.
    pub fn iers_cio_iau2006a(
        epoch_utc_julian_date: f64,
        tai_minus_utc_s: f64,
        local_origin: Option<LocalGeodeticOrigin>,
        earth_orientation: EarthOrientationTable,
    ) -> Result<Self, FrameError> {
        if !epoch_utc_julian_date.is_finite() {
            return Err(FrameError::InvalidFrameProfileData {
                reason: "IERS CIO frame epoch Julian Date must be finite",
            });
        }
        let cio = CioFrameModel::iau2006a(
            epoch_utc_julian_date,
            tai_minus_utc_s,
            earth_orientation.clone(),
        )?;
        Ok(Self {
            profile: FrameProfile::IersCio,
            local_origin,
            iers: Some(IersFrameData {
                epoch_utc_julian_date,
                earth_orientation,
            }),
            cio: Some(cio),
        })
    }

    /// The active profile.
    #[must_use]
    pub const fn profile(&self) -> FrameProfile {
        self.profile
    }

    /// The declared local geodetic origin, if any.
    #[must_use]
    pub const fn local_origin(&self) -> Option<&LocalGeodeticOrigin> {
        self.local_origin.as_ref()
    }

    /// Pinned Earth-orientation table used by IERS profiles, if any.
    #[must_use]
    pub fn earth_orientation_table(&self) -> Option<&EarthOrientationTable> {
        self.iers.as_ref().map(|iers| &iers.earth_orientation)
    }

    /// Pinned CIO frame model used by [`FrameProfile::IersCio`], if any.
    #[must_use]
    pub const fn cio_frame_model(&self) -> Option<&CioFrameModel> {
        self.cio.as_ref()
    }

    // ---------------------------------------------------------------
    // Time-aware ECI ↔ ECEF transforms
    //
    // For the WGS84 uniform-rotation profile, ECI and ECEF coincide at
    // `t = 0` and ECEF rotates eastward (about the inertial +z axis)
    // at the constant rate `WGS84_OMEGA_RAD_S`. The Earth rotation
    // angle at simulation time `t` is `θ(t) = ω_e · t`.
    //
    // The ToyFixedEarth profile returns identity for all of these
    // and the `FrameTransform<Eci, Ecef>` impl delegates here so it
    // stays time-aware for WGS84.
    // ---------------------------------------------------------------

    /// Earth rotation angle at simulation time `t`, in radians.
    /// Returns `0.0` for [`FrameProfile::ToyFixedEarth`].
    #[must_use]
    pub fn earth_rotation_angle(&self, t: SimTime) -> f64 {
        match self.profile {
            FrameProfile::ToyFixedEarth => 0.0,
            FrameProfile::Wgs84UniformRotation => WGS84_OMEGA_RAD_S * t.as_seconds(),
            FrameProfile::IersTabulated => {
                let Some(iers) = self.iers.as_ref() else {
                    return WGS84_OMEGA_RAD_S * t.as_seconds();
                };
                let eop = iers.earth_orientation.sample(t);
                let jd_ut1 = iers.epoch_utc_julian_date
                    + (t.as_seconds() + eop.ut1_minus_utc_s) / SECONDS_PER_DAY;
                earth_rotation_angle_from_ut1_julian_date(jd_ut1)
            }
            FrameProfile::IersCio => self
                .cio
                .as_ref()
                .and_then(|cio| cio.sample(t).ok())
                .and_then(|sample| earth_rotation_angle_iau2000(sample.ut1_julian_date).ok())
                .unwrap_or(f64::NAN),
        }
    }

    /// Transform an ECI position to ECEF at simulation time `t`.
    ///
    /// For `iers-tabulated`, the path is J2000 ECI -> true-of-date
    /// axes via precession and nutation -> TIRS via Earth Rotation
    /// Angle -> ECEF via polar motion. `ToyFixedEarth` profile always
    /// returns `p_eci` reinterpreted as ECEF.
    #[must_use]
    pub fn eci_to_ecef_position(&self, t: SimTime, p_eci: Position3<Eci>) -> Position3<Ecef> {
        Position3::from_vector(self.eci_to_ecef_vector(t, p_eci.vector))
    }

    /// Transform an ECEF position to ECI at simulation time `t`.
    /// Inverse of [`Self::eci_to_ecef_position`].
    #[must_use]
    pub fn ecef_to_eci_position(&self, t: SimTime, p_ecef: Position3<Ecef>) -> Position3<Eci> {
        Position3::from_vector(self.ecef_to_eci_vector(t, p_ecef.vector))
    }

    /// Transform an ECI velocity to ECEF velocity at simulation time
    /// `t` for a body at ECI position `p_eci`.
    ///
    /// `v_ecef = R_z(-θ) · (v_eci - ω × r_eci)`. Subtracts the transport
    /// rate from the rotating reference frame.
    #[must_use]
    #[allow(clippy::many_single_char_names)] // r, v, s, c are physics-conventional
    pub fn eci_to_ecef_velocity(
        &self,
        t: SimTime,
        v_eci: Velocity3<Eci>,
        p_eci: Position3<Eci>,
    ) -> Velocity3<Ecef> {
        if matches!(
            self.profile,
            FrameProfile::IersTabulated | FrameProfile::IersCio
        ) {
            let rotation = self.eci_to_ecef_rotation_matrix(t);
            let rotation_rate = self.eci_to_ecef_rotation_rate_matrix(t);
            return Velocity3::from_vector(rotation * v_eci.vector + rotation_rate * p_eci.vector);
        }
        let omega = self.angular_velocity_z();
        let r = self.eci_to_true_of_date_vector(t, p_eci.vector);
        let v = self.eci_to_true_of_date_vector(t, v_eci.vector);
        // ω × r in the true-of-date intermediate frame, where the
        // compact Earth spin axis is the frame `+z` axis.
        let cross = nalgebra::Vector3::new(-omega * r.y, omega * r.x, 0.0);
        let v_inertial_minus_transport = v - cross;
        let theta = self.earth_rotation_angle(t);
        let tirs = rotate_z(v_inertial_minus_transport, -theta);
        Velocity3::from_vector(self.tirs_to_ecef_vector(t, tirs))
    }

    /// Transform an ECEF velocity to ECI velocity at simulation time
    /// `t` for a body at ECEF position `p_ecef`.
    ///
    /// Inverse of [`Self::eci_to_ecef_velocity`].
    #[must_use]
    #[allow(clippy::many_single_char_names)] // r, v, s, c are physics-conventional
    pub fn ecef_to_eci_velocity(
        &self,
        t: SimTime,
        v_ecef: Velocity3<Ecef>,
        p_ecef: Position3<Ecef>,
    ) -> Velocity3<Eci> {
        if matches!(
            self.profile,
            FrameProfile::IersTabulated | FrameProfile::IersCio
        ) {
            let ecef_to_eci = self.ecef_to_eci_rotation_matrix(t);
            let r_eci = ecef_to_eci * p_ecef.vector;
            let eci_to_ecef_rate = self.eci_to_ecef_rotation_rate_matrix(t);
            return Velocity3::from_vector(
                ecef_to_eci * (v_ecef.vector - eci_to_ecef_rate * r_eci),
            );
        }
        // Rotate the ECEF velocity into true-of-date inertial
        // orientation, add the transport rate there, then rotate back
        // to the scenario ECI axes.
        let theta = self.earth_rotation_angle(t);
        let tirs = self.ecef_to_tirs_vector(t, v_ecef.vector);
        let v_rotated_true_of_date = rotate_z(tirs, theta);
        let r_tirs = self.ecef_to_tirs_vector(t, p_ecef.vector);
        let r_true_of_date = rotate_z(r_tirs, theta);
        let omega = self.angular_velocity_z();
        let cross = Vector3::new(-omega * r_true_of_date.y, omega * r_true_of_date.x, 0.0);
        Velocity3::from_vector(self.true_of_date_to_eci_vector(t, v_rotated_true_of_date + cross))
    }

    /// Earth angular velocity along the inertial `+z` axis (rad/s).
    /// `0.0` for `ToyFixedEarth`, `WGS84_OMEGA_RAD_S` for
    /// `Wgs84UniformRotation`.
    #[must_use]
    pub const fn angular_velocity_z(&self) -> f64 {
        match self.profile {
            FrameProfile::ToyFixedEarth => 0.0,
            FrameProfile::Wgs84UniformRotation => WGS84_OMEGA_RAD_S,
            FrameProfile::IersTabulated | FrameProfile::IersCio => EARTH_ROTATION_ANGLE_RATE_RAD_S,
        }
    }

    /// Earth angular velocity along the frame spin axis at simulation
    /// time `t`, rad/s.
    ///
    /// For [`FrameProfile::IersTabulated`], this uses the sampled
    /// length-of-day value when available. Without `lod_s`, the
    /// nominal ERA slope is returned.
    #[must_use]
    pub fn angular_velocity_z_at(&self, t: SimTime) -> f64 {
        match self.profile {
            FrameProfile::ToyFixedEarth => 0.0,
            FrameProfile::Wgs84UniformRotation => WGS84_OMEGA_RAD_S,
            FrameProfile::IersTabulated | FrameProfile::IersCio => {
                self.earth_orientation_sample(t).earth_rotation_rate_rad_s()
            }
        }
    }

    fn earth_orientation_sample(&self, t: SimTime) -> EarthOrientationSample {
        self.iers.as_ref().map_or(
            EarthOrientationSample {
                time_s: t.as_seconds(),
                ut1_minus_utc_s: 0.0,
                polar_motion_x_rad: 0.0,
                polar_motion_y_rad: 0.0,
                cip_offset_x_rad: 0.0,
                cip_offset_y_rad: 0.0,
                lod_s: None,
            },
            |iers| iers.earth_orientation.sample(t),
        )
    }

    /// Rotate an ECI vector into ECEF axes at simulation time `t`.
    ///
    /// This applies only the frame orientation, with no translational or
    /// transport-rate terms. Use it for direction-like vectors and
    /// accelerations. Use [`Self::eci_to_ecef_velocity`] for velocities.
    #[must_use]
    pub fn eci_to_ecef_vector(&self, t: SimTime, v_eci: Vector3<f64>) -> Vector3<f64> {
        if self.profile == FrameProfile::IersCio {
            return self
                .cio
                .as_ref()
                .and_then(|cio| cio.gcrs_to_itrs_matrix(t).ok())
                .map_or(Vector3::repeat(f64::NAN), |rotation| rotation * v_eci);
        }
        let theta = self.earth_rotation_angle(t);
        let true_of_date = self.eci_to_true_of_date_vector(t, v_eci);
        let tirs = rotate_z(true_of_date, -theta);
        self.tirs_to_ecef_vector(t, tirs)
    }

    /// Rotate an ECEF vector into ECI axes at simulation time `t`.
    ///
    /// This applies only the frame orientation, with no translational or
    /// transport-rate terms. Use it for direction-like vectors and
    /// accelerations. Use [`Self::ecef_to_eci_velocity`] for velocities.
    #[must_use]
    pub fn ecef_to_eci_vector(&self, t: SimTime, v_ecef: Vector3<f64>) -> Vector3<f64> {
        if self.profile == FrameProfile::IersCio {
            return self
                .cio
                .as_ref()
                .and_then(|cio| cio.itrs_to_gcrs_matrix(t).ok())
                .map_or(Vector3::repeat(f64::NAN), |rotation| rotation * v_ecef);
        }
        let theta = self.earth_rotation_angle(t);
        let tirs = self.ecef_to_tirs_vector(t, v_ecef);
        let true_of_date = rotate_z(tirs, theta);
        self.true_of_date_to_eci_vector(t, true_of_date)
    }

    fn eci_to_ecef_rotation_matrix(&self, t: SimTime) -> Matrix3<f64> {
        Matrix3::from_columns(&[
            self.eci_to_ecef_vector(t, Vector3::new(1.0, 0.0, 0.0)),
            self.eci_to_ecef_vector(t, Vector3::new(0.0, 1.0, 0.0)),
            self.eci_to_ecef_vector(t, Vector3::new(0.0, 0.0, 1.0)),
        ])
    }

    fn ecef_to_eci_rotation_matrix(&self, t: SimTime) -> Matrix3<f64> {
        Matrix3::from_columns(&[
            self.ecef_to_eci_vector(t, Vector3::new(1.0, 0.0, 0.0)),
            self.ecef_to_eci_vector(t, Vector3::new(0.0, 1.0, 0.0)),
            self.ecef_to_eci_vector(t, Vector3::new(0.0, 0.0, 1.0)),
        ])
    }

    fn eci_to_ecef_rotation_rate_matrix(&self, t: SimTime) -> Matrix3<f64> {
        self.rotation_rate_matrix(t, Self::eci_to_ecef_rotation_matrix)
    }

    fn rotation_rate_matrix(
        &self,
        t: SimTime,
        rotation: fn(&Self, SimTime) -> Matrix3<f64>,
    ) -> Matrix3<f64> {
        let time_s = t.as_seconds();
        let before = rotation(self, SimTime::from_seconds(time_s - FRAME_RATE_STEP_S));
        let after = rotation(self, SimTime::from_seconds(time_s + FRAME_RATE_STEP_S));
        (after - before) * (0.5 / FRAME_RATE_STEP_S)
    }

    fn tirs_to_ecef_vector(&self, t: SimTime, v_tirs: Vector3<f64>) -> Vector3<f64> {
        if self.profile != FrameProfile::IersTabulated {
            return v_tirs;
        }
        let eop = self.earth_orientation_sample(t);
        rotate_x(
            rotate_y(v_tirs, -eop.polar_motion_x_rad),
            -eop.polar_motion_y_rad,
        )
    }

    fn ecef_to_tirs_vector(&self, t: SimTime, v_ecef: Vector3<f64>) -> Vector3<f64> {
        if self.profile != FrameProfile::IersTabulated {
            return v_ecef;
        }
        let eop = self.earth_orientation_sample(t);
        rotate_y(
            rotate_x(v_ecef, eop.polar_motion_y_rad),
            eop.polar_motion_x_rad,
        )
    }

    fn eci_to_true_of_date_vector(&self, t: SimTime, v_eci: Vector3<f64>) -> Vector3<f64> {
        if self.profile != FrameProfile::IersTabulated {
            return v_eci;
        }
        precess_j2000_to_true_of_date_vector(self.frame_model_julian_date(t), v_eci)
    }

    fn true_of_date_to_eci_vector(&self, t: SimTime, v_tod: Vector3<f64>) -> Vector3<f64> {
        if self.profile != FrameProfile::IersTabulated {
            return v_tod;
        }
        true_of_date_to_j2000_vector(self.frame_model_julian_date(t), v_tod)
    }

    fn frame_model_julian_date(&self, t: SimTime) -> f64 {
        self.iers.as_ref().map_or(J2000_JULIAN_DATE, |iers| {
            iers.epoch_utc_julian_date + t.as_seconds() / SECONDS_PER_DAY
        })
    }

    // ---------------------------------------------------------------
    // ECEF ↔ NED helpers anchored at the local origin
    //
    // The ECEF→NED rotation matrix at geodetic latitude φ and
    // longitude λ is the standard textbook form:
    //
    //   R_ecef_to_ned =
    //     [-sin(φ)·cos(λ), -sin(φ)·sin(λ),  cos(φ)
    //      -sin(λ),         cos(λ),          0
    //      -cos(φ)·cos(λ), -cos(φ)·sin(λ), -sin(φ)]
    //
    // Position and vector helpers below.
    // ---------------------------------------------------------------

    /// Transform an ECEF *position* to NED relative to the declared
    /// local origin.
    ///
    /// `p_ned = R_ecef_to_ned · (p_ecef - origin_ecef)`.
    /// The local vertical is the WGS84 geodetic normal at the
    /// declared latitude/longitude, not the geocentric radial except
    /// at the equator and poles.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::LocalOriginRequired`] when no local
    /// origin is declared in the active context.
    pub fn ecef_to_ned_position(
        &self,
        p_ecef: Position3<Ecef>,
    ) -> Result<Position3<Ned>, FrameError> {
        let origin = self.require_local_origin()?;
        let origin_ecef = origin.to_ecef_position();
        let delta = p_ecef.vector - origin_ecef.vector;
        let (sin_lat, cos_lat) = origin.latitude_rad.sin_cos();
        let (sin_lon, cos_lon) = origin.longitude_rad.sin_cos();
        let n = -sin_lat * cos_lon * delta.x - sin_lat * sin_lon * delta.y + cos_lat * delta.z;
        let e = -sin_lon * delta.x + cos_lon * delta.y;
        let d = -cos_lat * cos_lon * delta.x - cos_lat * sin_lon * delta.y - sin_lat * delta.z;
        Ok(Position3::new(n, e, d))
    }

    /// Transform an ECEF *vector* (e.g., a velocity) to NED. Pure
    /// rotation, no origin offset.
    ///
    /// `+down` is anti-parallel to the WGS84 geodetic normal at the
    /// declared origin, not generally toward Earth's centre.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::LocalOriginRequired`] when no local
    /// origin is declared.
    pub fn ecef_to_ned_velocity(
        &self,
        v_ecef: Velocity3<Ecef>,
    ) -> Result<Velocity3<Ned>, FrameError> {
        let origin = self.require_local_origin()?;
        let v = v_ecef.vector;
        let (sin_lat, cos_lat) = origin.latitude_rad.sin_cos();
        let (sin_lon, cos_lon) = origin.longitude_rad.sin_cos();
        let n = -sin_lat * cos_lon * v.x - sin_lat * sin_lon * v.y + cos_lat * v.z;
        let e = -sin_lon * v.x + cos_lon * v.y;
        let d = -cos_lat * cos_lon * v.x - cos_lat * sin_lon * v.y - sin_lat * v.z;
        Ok(Velocity3::new(n, e, d))
    }

    /// Transform an NED *vector* (e.g., a wind velocity) to ECEF.
    /// Pure rotation; inverse of [`Self::ecef_to_ned_velocity`].
    ///
    /// `+down` is anti-parallel to the WGS84 geodetic normal at the
    /// declared origin, not generally toward Earth's centre.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::LocalOriginRequired`] when no local
    /// origin is declared.
    pub fn ned_to_ecef_velocity(
        &self,
        v_ned: Velocity3<Ned>,
    ) -> Result<Velocity3<Ecef>, FrameError> {
        let origin = self.require_local_origin()?;
        let v = v_ned.vector;
        let (sin_lat, cos_lat) = origin.latitude_rad.sin_cos();
        let (sin_lon, cos_lon) = origin.longitude_rad.sin_cos();
        // R_ned_to_ecef = R_ecef_to_ned^T.
        let x = -sin_lat * cos_lon * v.x - sin_lon * v.y - cos_lat * cos_lon * v.z;
        let y = -sin_lat * sin_lon * v.x + cos_lon * v.y - cos_lat * sin_lon * v.z;
        let z = cos_lat * v.x - sin_lat * v.z;
        Ok(Velocity3::new(x, y, z))
    }

    fn require_local_origin(&self) -> Result<&LocalGeodeticOrigin, FrameError> {
        self.local_origin
            .as_ref()
            .ok_or(FrameError::LocalOriginRequired {
                profile: self.profile.as_label(),
            })
    }
}

fn lerp(a: f64, b: f64, alpha: f64) -> f64 {
    a + alpha * (b - a)
}

fn interpolate_optional_lod(lo: Option<f64>, hi: Option<f64>, alpha: f64) -> Option<f64> {
    match (lo, hi) {
        (Some(lo), Some(hi)) => Some(lerp(lo, hi, alpha)),
        _ => None,
    }
}

fn interpolate_ut1_minus_utc(
    lo: EarthOrientationSample,
    hi: EarthOrientationSample,
    alpha: f64,
) -> f64 {
    match (lo.lod_s, hi.lod_s) {
        (Some(lo_lod_s), Some(hi_lod_s)) => {
            let interval_s = hi.time_s - lo.time_s;
            let lo_slope_s_per_s = -lo_lod_s / SECONDS_PER_DAY;
            let hi_slope_s_per_s = -hi_lod_s / SECONDS_PER_DAY;
            cubic_hermite(
                lo.ut1_minus_utc_s,
                hi.ut1_minus_utc_s,
                lo_slope_s_per_s,
                hi_slope_s_per_s,
                interval_s,
                alpha,
            )
        }
        _ => lerp(lo.ut1_minus_utc_s, hi.ut1_minus_utc_s, alpha),
    }
}

fn cubic_hermite(y0: f64, y1: f64, m0: f64, m1: f64, interval_s: f64, alpha: f64) -> f64 {
    let alpha2 = alpha * alpha;
    let alpha3 = alpha2 * alpha;
    let h00 = 2.0 * alpha3 - 3.0 * alpha2 + 1.0;
    let h10 = alpha3 - 2.0 * alpha2 + alpha;
    let h01 = -2.0 * alpha3 + 3.0 * alpha2;
    let h11 = alpha3 - alpha2;
    h00 * y0 + h10 * interval_s * m0 + h01 * y1 + h11 * interval_s * m1
}

fn earth_rotation_angle_from_ut1_julian_date(jd_ut1: f64) -> f64 {
    earth_rotation_angle_iau2000(jd_ut1).unwrap_or(f64::NAN)
}

/// Earth Rotation Angle, IAU 2000 model, from UT1 Julian Date.
///
/// This is the ERFA `eraEra00` formulation using a J2000 split to preserve
/// fractional-day precision while accepting a single Julian Date argument.
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when `ut1_julian_date` is
/// not finite.
pub fn earth_rotation_angle_iau2000(ut1_julian_date: f64) -> Result<f64, FrameError> {
    earth_rotation_angle_iau2000_parts(J2000_JULIAN_DATE, ut1_julian_date - J2000_JULIAN_DATE)
}

/// Earth Rotation Angle, IAU 2000 model, from a two-part UT1 Julian Date.
///
/// This directly mirrors ERFA `eraEra00`, preserving precision when callers
/// split a date into an epoch part and a fractional-day part.
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when either date part is
/// not finite.
pub fn earth_rotation_angle_iau2000_parts(
    ut1_date1: f64,
    ut1_date2: f64,
) -> Result<f64, FrameError> {
    finite_cio_input(ut1_date1)?;
    finite_cio_input(ut1_date2)?;
    let (d1, d2) = if ut1_date1 < ut1_date2 {
        (ut1_date1, ut1_date2)
    } else {
        (ut1_date2, ut1_date1)
    };
    let t = d1 + (d2 - J2000_JULIAN_DATE);
    let f = (d1 % 1.0) + (d2 % 1.0);
    Ok(rem_euclid_two_pi(
        TWO_PI * (f + 0.779_057_273_264_0 + 0.002_737_811_911_354_48 * t),
    ))
}

/// Approximate TIO locator `s'`, IAU 2000, from TT Julian Date.
///
/// This is the ERFA `eraSp00` secular approximation.
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when `tt_julian_date` is
/// not finite.
pub fn tio_locator_sp00(tt_julian_date: f64) -> Result<f64, FrameError> {
    tio_locator_sp00_parts(J2000_JULIAN_DATE, tt_julian_date - J2000_JULIAN_DATE)
}

/// Approximate TIO locator `s'`, IAU 2000, from a two-part TT Julian Date.
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when either date part is
/// not finite.
pub fn tio_locator_sp00_parts(tt_date1: f64, tt_date2: f64) -> Result<f64, FrameError> {
    finite_cio_input(tt_date1)?;
    finite_cio_input(tt_date2)?;
    let t = ((tt_date1 - J2000_JULIAN_DATE) + tt_date2) / JULIAN_CENTURY_DAYS;
    Ok(-47.0e-6 * t * ARCSECOND_TO_RAD)
}

/// CIO locator `s`, IAU 2006, from TT Julian Date and CIP `X`, `Y`.
///
/// This is the ERFA `eraS06` formulation for the CIO locator compatible with
/// IAU 2006/2000A precession-nutation. Callers provide the CIP coordinates;
/// this function computes the `s` series, including the final `-X*Y/2` term.
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when any input is
/// non-finite or when `X^2 + Y^2 >= 1`.
pub fn cio_locator_s06(tt_julian_date: f64, x_rad: f64, y_rad: f64) -> Result<f64, FrameError> {
    cio_locator_s06_parts(
        J2000_JULIAN_DATE,
        tt_julian_date - J2000_JULIAN_DATE,
        x_rad,
        y_rad,
    )
}

/// CIO locator `s`, IAU 2006, from a two-part TT Julian Date and CIP `X`, `Y`.
///
/// This directly mirrors ERFA `eraS06`, preserving precision when callers split
/// a date into an epoch part and a fractional-day part.
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when any input is
/// non-finite or when `X^2 + Y^2 >= 1`.
pub fn cio_locator_s06_parts(
    tt_date1: f64,
    tt_date2: f64,
    x_rad: f64,
    y_rad: f64,
) -> Result<f64, FrameError> {
    finite_cio_input(tt_date1)?;
    finite_cio_input(tt_date2)?;
    validate_cio_xy(x_rad, y_rad)?;

    let t = ((tt_date1 - J2000_JULIAN_DATE) + tt_date2) / JULIAN_CENTURY_DAYS;
    let fa = [
        erfa_fal03(t),
        erfa_falp03(t),
        erfa_faf03(t),
        erfa_fad03(t),
        erfa_faom03(t),
        erfa_fave03(t),
        erfa_fae03(t),
        erfa_fapa03(t),
    ];

    let w0 = cio_s06_accumulate(CIO_S06_POLYNOMIAL_ARCSECONDS[0], &CIO_S06_TERMS_0, fa);
    let w1 = cio_s06_accumulate(CIO_S06_POLYNOMIAL_ARCSECONDS[1], &CIO_S06_TERMS_1, fa);
    let w2 = cio_s06_accumulate(CIO_S06_POLYNOMIAL_ARCSECONDS[2], &CIO_S06_TERMS_2, fa);
    let w3 = cio_s06_accumulate(CIO_S06_POLYNOMIAL_ARCSECONDS[3], &CIO_S06_TERMS_3, fa);
    let w4 = cio_s06_accumulate(CIO_S06_POLYNOMIAL_ARCSECONDS[4], &CIO_S06_TERMS_4, fa);
    let w5 = CIO_S06_POLYNOMIAL_ARCSECONDS[5];

    Ok(
        (w0 + (w1 + (w2 + (w3 + (w4 + w5 * t) * t) * t) * t) * t) * ARCSECOND_TO_RAD
            - x_rad * y_rad / 2.0,
    )
}

/// Mean obliquity of the ecliptic, IAU 2006, from TT Julian Date.
///
/// This is the ERFA `eraObl06` polynomial.
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when `tt_julian_date` is
/// not finite.
pub fn mean_obliquity_iau2006(tt_julian_date: f64) -> Result<f64, FrameError> {
    mean_obliquity_iau2006_parts(J2000_JULIAN_DATE, tt_julian_date - J2000_JULIAN_DATE)
}

/// Mean obliquity of the ecliptic, IAU 2006, from a two-part TT Julian Date.
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when either date part is
/// not finite.
pub fn mean_obliquity_iau2006_parts(tt_date1: f64, tt_date2: f64) -> Result<f64, FrameError> {
    finite_cio_input(tt_date1)?;
    finite_cio_input(tt_date2)?;
    let t = ((tt_date1 - J2000_JULIAN_DATE) + tt_date2) / JULIAN_CENTURY_DAYS;
    Ok((84_381.406
        + t * (-46.836_769
            + t * (-0.000_183_1
                + t * (0.002_003_40 + t * (-0.000_000_576 + t * (-0.000_000_043_4))))))
        * ARCSECOND_TO_RAD)
}

/// Fukushima-Williams precession angles, IAU 2006, from TT Julian Date.
///
/// This is the ERFA `eraPfw06` formulation.
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when `tt_julian_date` is
/// not finite.
pub fn fukushima_williams_angles_iau2006(
    tt_julian_date: f64,
) -> Result<FukushimaWilliamsAngles, FrameError> {
    fukushima_williams_angles_iau2006_parts(J2000_JULIAN_DATE, tt_julian_date - J2000_JULIAN_DATE)
}

/// Fukushima-Williams precession angles, IAU 2006, from a two-part TT Julian Date.
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when either date part is
/// not finite.
pub fn fukushima_williams_angles_iau2006_parts(
    tt_date1: f64,
    tt_date2: f64,
) -> Result<FukushimaWilliamsAngles, FrameError> {
    finite_cio_input(tt_date1)?;
    finite_cio_input(tt_date2)?;
    let t = ((tt_date1 - J2000_JULIAN_DATE) + tt_date2) / JULIAN_CENTURY_DAYS;
    let gamma_bar_rad = (-0.052_928
        + t * (10.556_378
            + t * (0.493_204_4
                + t * (-0.000_312_38 + t * (-0.000_002_788 + t * 0.000_000_026_0)))))
        * ARCSECOND_TO_RAD;
    let phi_bar_rad = (84_381.412_819
        + t * (-46.811_016
            + t * (0.051_126_8
                + t * (0.000_532_89 + t * (-0.000_000_440 + t * (-0.000_000_017_6))))))
        * ARCSECOND_TO_RAD;
    let psi_bar_rad = (-0.041_775
        + t * (5_038.481_484
            + t * (1.558_417_5
                + t * (-0.000_185_22 + t * (-0.000_026_452 + t * (-0.000_000_014_8))))))
        * ARCSECOND_TO_RAD;
    Ok(FukushimaWilliamsAngles {
        gamma_bar_rad,
        phi_bar_rad,
        psi_bar_rad,
        epsilon_a_rad: mean_obliquity_iau2006_parts(tt_date1, tt_date2)?,
    })
}

/// Rotation matrix from Fukushima-Williams angles.
///
/// This is the ERFA `eraFw2m` matrix construction:
/// `R1(-eps) * R3(-psi) * R1(phi) * R3(gamma)`.
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when any angle is
/// non-finite.
pub fn fukushima_williams_matrix(
    gamma_rad: f64,
    phi_rad: f64,
    psi_rad: f64,
    epsilon_rad: f64,
) -> Result<Matrix3<f64>, FrameError> {
    finite_cio_input(gamma_rad)?;
    finite_cio_input(phi_rad)?;
    finite_cio_input(psi_rad)?;
    finite_cio_input(epsilon_rad)?;
    let matrix = erfa_rotate_z_matrix(Matrix3::identity(), gamma_rad);
    let matrix = erfa_rotate_x_matrix(matrix, phi_rad);
    let matrix = erfa_rotate_z_matrix(matrix, -psi_rad);
    Ok(erfa_rotate_x_matrix(matrix, -epsilon_rad))
}

/// Extract CIP `X`, `Y` coordinates from a bias-precession-nutation matrix.
///
/// This mirrors ERFA `eraBpn2xy`: the CIP vector is the bottom row of the
/// GCRS-to-true matrix.
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when the matrix is
/// non-finite or the extracted `X`, `Y` pair is outside the unit sphere.
pub fn cip_xy_from_bias_precession_nutation_matrix(
    rbpn: Matrix3<f64>,
) -> Result<(f64, f64), FrameError> {
    finite_matrix_input(rbpn)?;
    let x_rad = rbpn[(2, 0)];
    let y_rad = rbpn[(2, 1)];
    validate_cio_xy(x_rad, y_rad)?;
    Ok((x_rad, y_rad))
}

/// IAU 2000A nutation in longitude and obliquity from TT Julian Date.
///
/// This mirrors ERFA `eraNut00a`, including the MHB2000 luni-solar and
/// planetary nutation terms with free core nutation omitted.
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when `tt_julian_date` is
/// not finite.
pub fn nutation_angles_iau2000a(tt_julian_date: f64) -> Result<(f64, f64), FrameError> {
    nutation_angles_iau2000a_parts(J2000_JULIAN_DATE, tt_julian_date - J2000_JULIAN_DATE)
}

/// IAU 2000A nutation in longitude and obliquity from a two-part TT Julian Date.
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when either date part is
/// not finite.
pub fn nutation_angles_iau2000a_parts(
    tt_date1: f64,
    tt_date2: f64,
) -> Result<(f64, f64), FrameError> {
    finite_cio_input(tt_date1)?;
    finite_cio_input(tt_date2)?;
    let t = ((tt_date1 - J2000_JULIAN_DATE) + tt_date2) / JULIAN_CENTURY_DAYS;

    let el = erfa_fal03(t);
    let elp = (1_287_104.793_05
        + t * (129_596_581.048_1 + t * (-0.553_2 + t * (0.000_136 + t * (-0.000_011_49)))))
        % FULL_CIRCLE_ARCSECONDS
        * ARCSECOND_TO_RAD;
    let f = erfa_faf03(t);
    let d = (1_072_260.703_69
        + t * (1_602_961_601.209_0 + t * (-6.370_6 + t * (0.006_593 + t * (-0.000_031_69)))))
        % FULL_CIRCLE_ARCSECONDS
        * ARCSECOND_TO_RAD;
    let om = erfa_faom03(t);

    let mut dp = 0.0;
    let mut de = 0.0;
    for term in IAU2000A_LUNI_SOLAR_TERMS.iter().rev() {
        let arg = (f64::from(term.nl) * el
            + f64::from(term.nlp) * elp
            + f64::from(term.nf) * f
            + f64::from(term.nd) * d
            + f64::from(term.nom) * om)
            % TWO_PI;
        let sarg = arg.sin();
        let carg = arg.cos();
        dp += (term.sp + term.spt * t) * sarg + term.cp * carg;
        de += (term.ce + term.cet * t) * carg + term.se * sarg;
    }
    let dpsils = dp * TENTH_MICROARCSECOND_TO_RAD;
    let depsls = de * TENTH_MICROARCSECOND_TO_RAD;

    let al = (2.355_555_98 + 8_328.691_426_955_4 * t) % TWO_PI;
    let af = (1.627_905_234 + 8_433.466_158_131 * t) % TWO_PI;
    let ad = (5.198_466_741 + 7_771.377_146_812_1 * t) % TWO_PI;
    let aom = (2.182_439_20 - 33.757_045 * t) % TWO_PI;
    let alme = erfa_fame03(t);
    let alve = erfa_fave03(t);
    let alea = erfa_fae03(t);
    let alma = erfa_fama03(t);
    let alju = erfa_faju03(t);
    let alsa = erfa_fasa03(t);
    let alur = erfa_faur03(t);
    let alne = (5.321_159_000 + 3.812_777_400_0 * t) % TWO_PI;
    let apa = erfa_fapa03(t);

    dp = 0.0;
    de = 0.0;
    for term in IAU2000A_PLANETARY_TERMS.iter().rev() {
        let arg = (f64::from(term.nl) * al
            + f64::from(term.nf) * af
            + f64::from(term.nd) * ad
            + f64::from(term.nom) * aom
            + f64::from(term.nme) * alme
            + f64::from(term.nve) * alve
            + f64::from(term.nea) * alea
            + f64::from(term.nma) * alma
            + f64::from(term.nju) * alju
            + f64::from(term.nsa) * alsa
            + f64::from(term.nur) * alur
            + f64::from(term.nne) * alne
            + f64::from(term.npa) * apa)
            % TWO_PI;
        let sarg = arg.sin();
        let carg = arg.cos();
        dp += term.sp * sarg + term.cp * carg;
        de += term.se * sarg + term.ce * carg;
    }
    let dpsipl = dp * TENTH_MICROARCSECOND_TO_RAD;
    let depspl = de * TENTH_MICROARCSECOND_TO_RAD;

    Ok((dpsils + dpsipl, depsls + depspl))
}

/// IAU 2006/2000A nutation in longitude and obliquity from TT Julian Date.
///
/// This mirrors ERFA `eraNut06a`: IAU 2000A nutation plus P03 adjustments.
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when `tt_julian_date` is
/// not finite.
pub fn nutation_angles_iau2006a(tt_julian_date: f64) -> Result<(f64, f64), FrameError> {
    nutation_angles_iau2006a_parts(J2000_JULIAN_DATE, tt_julian_date - J2000_JULIAN_DATE)
}

/// IAU 2006/2000A nutation in longitude and obliquity from a two-part TT Julian Date.
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when either date part is
/// not finite.
pub fn nutation_angles_iau2006a_parts(
    tt_date1: f64,
    tt_date2: f64,
) -> Result<(f64, f64), FrameError> {
    finite_cio_input(tt_date1)?;
    finite_cio_input(tt_date2)?;
    let t = ((tt_date1 - J2000_JULIAN_DATE) + tt_date2) / JULIAN_CENTURY_DAYS;
    let fj2 = -2.777_4e-6 * t;
    let (dpsi, deps) = nutation_angles_iau2000a_parts(tt_date1, tt_date2)?;
    Ok((dpsi + dpsi * (0.469_7e-6 + fj2), deps + deps * fj2))
}

/// Bias-precession-nutation matrix, IAU 2006/2000A, from TT Julian Date.
///
/// This mirrors ERFA `eraPnm06a`.
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when `tt_julian_date` is
/// not finite.
pub fn bias_precession_nutation_matrix_iau2006a(
    tt_julian_date: f64,
) -> Result<Matrix3<f64>, FrameError> {
    bias_precession_nutation_matrix_iau2006a_parts(
        J2000_JULIAN_DATE,
        tt_julian_date - J2000_JULIAN_DATE,
    )
}

/// Bias-precession-nutation matrix, IAU 2006/2000A, from a two-part TT Julian Date.
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when either date part is
/// not finite.
pub fn bias_precession_nutation_matrix_iau2006a_parts(
    tt_date1: f64,
    tt_date2: f64,
) -> Result<Matrix3<f64>, FrameError> {
    let angles = fukushima_williams_angles_iau2006_parts(tt_date1, tt_date2)?;
    let (dpsi, deps) = nutation_angles_iau2006a_parts(tt_date1, tt_date2)?;
    fukushima_williams_matrix(
        angles.gamma_bar_rad,
        angles.phi_bar_rad,
        angles.psi_bar_rad + dpsi,
        angles.epsilon_a_rad + deps,
    )
}

/// IAU 2006/2000A CIO `X`, `Y`, `s` coordinates from TT Julian Date.
///
/// This mirrors ERFA `eraXys06a`.
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when `tt_julian_date` is
/// not finite.
pub fn cio_xys_iau2006a(tt_julian_date: f64) -> Result<CioXysCoordinates, FrameError> {
    cio_xys_iau2006a_parts(J2000_JULIAN_DATE, tt_julian_date - J2000_JULIAN_DATE)
}

/// IAU 2006/2000A CIO `X`, `Y`, `s` coordinates from a two-part TT Julian Date.
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when either date part is
/// not finite.
pub fn cio_xys_iau2006a_parts(
    tt_date1: f64,
    tt_date2: f64,
) -> Result<CioXysCoordinates, FrameError> {
    let rbpn = bias_precession_nutation_matrix_iau2006a_parts(tt_date1, tt_date2)?;
    let (x_rad, y_rad) = cip_xy_from_bias_precession_nutation_matrix(rbpn)?;
    let s_rad = cio_locator_s06_parts(tt_date1, tt_date2, x_rad, y_rad)?;
    Ok(CioXysCoordinates {
        x_rad,
        y_rad,
        s_rad,
    })
}

/// Celestial-to-intermediate matrix from CIP `X`, `Y` and CIO locator `s`.
///
/// This is the ERFA `eraC2ixys` matrix-construction step. It does not compute
/// the IAU 2006/2000A `X`, `Y`, `s` series; callers must provide those
/// coordinates in radians.
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when any input is non-finite
/// or when `X^2 + Y^2 >= 1`.
pub fn cio_celestial_to_intermediate_matrix(
    x_rad: f64,
    y_rad: f64,
    s_rad: f64,
) -> Result<Matrix3<f64>, FrameError> {
    finite_cio_input(x_rad)?;
    finite_cio_input(y_rad)?;
    finite_cio_input(s_rad)?;
    validate_cio_xy(x_rad, y_rad)?;
    let r2 = x_rad * x_rad + y_rad * y_rad;
    let e = if r2 > 0.0 { y_rad.atan2(x_rad) } else { 0.0 };
    let d = (r2 / (1.0 - r2)).sqrt().atan();
    let matrix = erfa_rotate_z_matrix(Matrix3::identity(), e);
    let matrix = erfa_rotate_y_matrix(matrix, d);
    Ok(erfa_rotate_z_matrix(matrix, -(e + s_rad)))
}

/// IAU 2000 polar-motion matrix from pole coordinates and TIO locator `s'`.
///
/// This is the ERFA `eraPom00` construction. The matrix maps from the
/// intermediate terrestrial frame to ITRS/TRS.
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when any input is non-finite.
pub fn polar_motion_matrix_iau2000(
    xp_rad: f64,
    yp_rad: f64,
    sp_rad: f64,
) -> Result<Matrix3<f64>, FrameError> {
    finite_cio_input(xp_rad)?;
    finite_cio_input(yp_rad)?;
    finite_cio_input(sp_rad)?;
    let matrix = erfa_rotate_z_matrix(Matrix3::identity(), sp_rad);
    let matrix = erfa_rotate_y_matrix(matrix, -xp_rad);
    Ok(erfa_rotate_x_matrix(matrix, -yp_rad))
}

/// Assemble a CIO celestial-to-terrestrial matrix from already-computed pieces.
///
/// This corresponds to the ERFA/IERS product
/// `RPOM * R3(ERA) * RC2I`. The `rc2i` argument is normally produced by
/// [`cio_celestial_to_intermediate_matrix`], `era_rad` by
/// [`earth_rotation_angle_iau2000`], and `rpom` by
/// [`polar_motion_matrix_iau2000`].
///
/// # Errors
///
/// Returns [`FrameError::InvalidFrameProfileData`] when `era_rad` or any matrix
/// element is non-finite.
pub fn cio_celestial_to_terrestrial_matrix(
    rc2i: Matrix3<f64>,
    era_rad: f64,
    rpom: Matrix3<f64>,
) -> Result<Matrix3<f64>, FrameError> {
    finite_cio_input(era_rad)?;
    finite_matrix_input(rc2i)?;
    finite_matrix_input(rpom)?;
    let rera = erfa_rotate_z_matrix(Matrix3::identity(), era_rad);
    Ok(rpom * rera * rc2i)
}

fn validate_cio_xys_sample(sample: CioXysSample) -> Result<(), FrameError> {
    finite_cio_input(sample.time_s)?;
    finite_cio_input(sample.s_rad)?;
    validate_cio_xy(sample.x_rad, sample.y_rad)
}

fn validate_cio_xy(x_rad: f64, y_rad: f64) -> Result<(), FrameError> {
    finite_cio_input(x_rad)?;
    finite_cio_input(y_rad)?;
    let r2 = x_rad * x_rad + y_rad * y_rad;
    if r2 >= 1.0 {
        return Err(FrameError::InvalidFrameProfileData {
            reason: "CIO X/Y coordinates must satisfy X^2 + Y^2 < 1",
        });
    }
    Ok(())
}

fn finite_cio_input(value: f64) -> Result<(), FrameError> {
    if !value.is_finite() {
        return Err(FrameError::InvalidFrameProfileData {
            reason: "CIO frame inputs must be finite",
        });
    }
    Ok(())
}

fn finite_matrix_input(matrix: Matrix3<f64>) -> Result<(), FrameError> {
    for value in matrix.iter() {
        finite_cio_input(*value)?;
    }
    Ok(())
}

fn cio_s06_accumulate(base_arcsec: f64, terms: &[CioS06Term], fa: [f64; 8]) -> f64 {
    let mut value = base_arcsec;
    for term in terms.iter().rev() {
        let mut angle = 0.0;
        for (coefficient, argument) in term.nfa.iter().zip(fa.iter()) {
            angle += f64::from(*coefficient) * *argument;
        }
        value += term.sine_arcsec * angle.sin() + term.cosine_arcsec * angle.cos();
    }
    value
}

fn erfa_fal03(t: f64) -> f64 {
    (485_868.249_036
        + t * (1_717_915_923.217_8 + t * (31.879_2 + t * (0.051_635 + t * (-0.000_244_70)))))
        % FULL_CIRCLE_ARCSECONDS
        * ARCSECOND_TO_RAD
}

fn erfa_falp03(t: f64) -> f64 {
    (1_287_104.793_048
        + t * (129_596_581.048_1 + t * (-0.553_2 + t * (0.000_136 + t * (-0.000_011_49)))))
        % FULL_CIRCLE_ARCSECONDS
        * ARCSECOND_TO_RAD
}

fn erfa_faf03(t: f64) -> f64 {
    (335_779.526_232
        + t * (1_739_527_262.847_8 + t * (-12.751_2 + t * (-0.001_037 + t * 0.000_004_17))))
        % FULL_CIRCLE_ARCSECONDS
        * ARCSECOND_TO_RAD
}

fn erfa_fad03(t: f64) -> f64 {
    (1_072_260.703_692
        + t * (1_602_961_601.209_0 + t * (-6.370_6 + t * (0.006_593 + t * (-0.000_031_69)))))
        % FULL_CIRCLE_ARCSECONDS
        * ARCSECOND_TO_RAD
}

fn erfa_faom03(t: f64) -> f64 {
    (450_160.398_036
        + t * (-6_962_890.543_1 + t * (7.472_2 + t * (0.007_702 + t * (-0.000_059_39)))))
        % FULL_CIRCLE_ARCSECONDS
        * ARCSECOND_TO_RAD
}

fn erfa_fave03(t: f64) -> f64 {
    (3.176_146_697 + 1_021.328_554_621_1 * t) % TWO_PI
}

fn erfa_fae03(t: f64) -> f64 {
    (1.753_470_314 + 628.307_584_999_1 * t) % TWO_PI
}

fn erfa_fapa03(t: f64) -> f64 {
    (0.024_381_750 + 0.000_005_386_91 * t) * t
}

fn erfa_fame03(t: f64) -> f64 {
    (4.402_608_842 + 2_608.790_314_157_4 * t) % TWO_PI
}

fn erfa_fama03(t: f64) -> f64 {
    (6.203_480_913 + 334.061_242_670_0 * t) % TWO_PI
}

fn erfa_faju03(t: f64) -> f64 {
    (0.599_546_497 + 52.969_096_264_1 * t) % TWO_PI
}

fn erfa_fasa03(t: f64) -> f64 {
    (0.874_016_757 + 21.329_910_496_0 * t) % TWO_PI
}

fn erfa_faur03(t: f64) -> f64 {
    (5.481_293_872 + 7.478_159_856_7 * t) % TWO_PI
}

fn julian_centuries_since_j2000(julian_date: f64) -> f64 {
    (julian_date - J2000_JULIAN_DATE) / JULIAN_CENTURY_DAYS
}

fn precession_angles_iau1976(julian_date: f64) -> (f64, f64, f64) {
    let t = julian_centuries_since_j2000(julian_date);
    let zeta = (2_306.218_1 * t + 0.301_88 * t * t + 0.017_998 * t * t * t) * ARCSECOND_TO_RAD;
    let theta = (2_004.310_9 * t - 0.426_65 * t * t - 0.041_833 * t * t * t) * ARCSECOND_TO_RAD;
    let z = (2_306.218_1 * t + 1.094_68 * t * t + 0.018_203 * t * t * t) * ARCSECOND_TO_RAD;
    (zeta, theta, z)
}

fn precess_j2000_to_mean_of_date_vector(julian_date: f64, v: Vector3<f64>) -> Vector3<f64> {
    if !julian_date.is_finite() {
        return v;
    }
    let (zeta, theta, z) = precession_angles_iau1976(julian_date);
    rotate_z(rotate_y(rotate_z(v, zeta), -theta), z)
}

/// Rotate a vector from mean equator/equinox of date to J2000.
pub(crate) fn precess_mean_of_date_to_j2000_vector(
    julian_date: f64,
    v: Vector3<f64>,
) -> Vector3<f64> {
    if !julian_date.is_finite() {
        return v;
    }
    let (zeta, theta, z) = precession_angles_iau1976(julian_date);
    rotate_z(rotate_y(rotate_z(v, -z), theta), -zeta)
}

fn precess_j2000_to_true_of_date_vector(julian_date: f64, v: Vector3<f64>) -> Vector3<f64> {
    nutate_mean_of_date_to_true_of_date_vector(
        julian_date,
        precess_j2000_to_mean_of_date_vector(julian_date, v),
    )
}

fn true_of_date_to_j2000_vector(julian_date: f64, v: Vector3<f64>) -> Vector3<f64> {
    precess_mean_of_date_to_j2000_vector(
        julian_date,
        nutate_true_of_date_to_mean_of_date_vector(julian_date, v),
    )
}

fn mean_obliquity_iau1980(julian_date: f64) -> f64 {
    let t = julian_centuries_since_j2000(julian_date);
    (84_381.448 + (-46.815_0 + (-0.000_59 + 0.001_813 * t) * t) * t) * ARCSECOND_TO_RAD
}

fn nutation_angles_iau1980(julian_date: f64) -> (f64, f64) {
    if !julian_date.is_finite() {
        return (0.0, 0.0);
    }
    let t = julian_centuries_since_j2000(julian_date);
    let (l, l_prime, f, d, omega) = nutation_fundamental_arguments_iau1980(t);
    let mut dpsi = 0.0;
    let mut deps = 0.0;
    for term in IAU_1980_NUTATION_TERMS.iter().rev() {
        let arg = f64::from(term.l) * l
            + f64::from(term.l_prime) * l_prime
            + f64::from(term.f) * f
            + f64::from(term.d) * d
            + f64::from(term.omega) * omega;
        let sine_coeff = term.dpsi_0 + term.dpsi_t * t;
        let cosine_coeff = term.deps_0 + term.deps_t * t;
        if sine_coeff != 0.0 {
            dpsi += sine_coeff * arg.sin();
        }
        if cosine_coeff != 0.0 {
            deps += cosine_coeff * arg.cos();
        }
    }
    (
        dpsi * TENTH_MILLIARCSECOND_TO_RAD,
        deps * TENTH_MILLIARCSECOND_TO_RAD,
    )
}

fn nutation_fundamental_arguments_iau1980(t: f64) -> (f64, f64, f64, f64, f64) {
    let l = normalize_angle_pm_pi(
        (485_866.733 + (715_922.633 + (31.310 + 0.064 * t) * t) * t) * ARCSECOND_TO_RAD
            + (1_325.0 * t % 1.0) * TWO_PI,
    );
    let l_prime = normalize_angle_pm_pi(
        (1_287_099.804 + (1_292_581.224 + (-0.577 - 0.012 * t) * t) * t) * ARCSECOND_TO_RAD
            + (99.0 * t % 1.0) * TWO_PI,
    );
    let f = normalize_angle_pm_pi(
        (335_778.877 + (295_263.137 + (-13.257 + 0.011 * t) * t) * t) * ARCSECOND_TO_RAD
            + (1_342.0 * t % 1.0) * TWO_PI,
    );
    let d = normalize_angle_pm_pi(
        (1_072_261.307 + (1_105_601.328 + (-6.891 + 0.019 * t) * t) * t) * ARCSECOND_TO_RAD
            + (1_236.0 * t % 1.0) * TWO_PI,
    );
    let omega = normalize_angle_pm_pi(
        (450_160.280 + (-482_890.539 + (7.455 + 0.008 * t) * t) * t) * ARCSECOND_TO_RAD
            + (-5.0 * t % 1.0) * TWO_PI,
    );
    (l, l_prime, f, d, omega)
}

fn nutate_mean_of_date_to_true_of_date_vector(julian_date: f64, v: Vector3<f64>) -> Vector3<f64> {
    if !julian_date.is_finite() {
        return v;
    }
    let (dpsi, deps) = nutation_angles_iau1980(julian_date);
    let eps = mean_obliquity_iau1980(julian_date);
    rotate_x(rotate_z(rotate_x(v, -eps), dpsi), eps + deps)
}

/// Rotate a vector from true equator/equinox of date to mean equator/equinox of date.
pub(crate) fn nutate_true_of_date_to_mean_of_date_vector(
    julian_date: f64,
    v: Vector3<f64>,
) -> Vector3<f64> {
    if !julian_date.is_finite() {
        return v;
    }
    let (dpsi, deps) = nutation_angles_iau1980(julian_date);
    let eps = mean_obliquity_iau1980(julian_date);
    rotate_x(rotate_z(rotate_x(v, -(eps + deps)), -dpsi), eps)
}

fn normalize_angle_pm_pi(theta: f64) -> f64 {
    let mut normalized = rem_euclid_two_pi(theta);
    if normalized >= core::f64::consts::PI {
        normalized -= TWO_PI;
    }
    normalized
}

fn rem_euclid_two_pi(theta: f64) -> f64 {
    #[cfg(feature = "std")]
    {
        theta.rem_euclid(TWO_PI)
    }
    #[cfg(not(feature = "std"))]
    {
        Euclid::rem_euclid(&theta, &TWO_PI)
    }
}

#[derive(Copy, Clone)]
struct NutationTerm {
    l: i32,
    l_prime: i32,
    f: i32,
    d: i32,
    omega: i32,
    dpsi_0: f64,
    dpsi_t: f64,
    deps_0: f64,
    deps_t: f64,
}

// Third-party notice: the IAU 1980 nutation terms and compact helper
// formulas below are adapted from ERFA `eraNut80` and `eraObl80`.
// ERFA is Copyright (C) 2013-2023, NumFOCUS Foundation, all rights
// reserved, and is derived with permission from the IAU SOFA library.
// This OpenBMP implementation is not SOFA software and is not endorsed
// by SOFA, the IAU, or NumFOCUS.
//
// ERFA terms: redistribution and use in source and binary forms, with
// or without modification, are permitted provided that redistributions
// of source code retain the copyright notice, conditions, and
// disclaimer; redistributions in binary form reproduce them in the
// documentation and/or other materials; and neither the name of the
// Standards Of Fundamental Astronomy Board, the International
// Astronomical Union nor the names of contributors may be used to
// endorse or promote products derived from this software without
// specific prior written permission.
//
// ERFA disclaimer: this software is provided by the copyright holders
// and contributors "as is" and any express or implied warranties,
// including, but not limited to, the implied warranties of
// merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the copyright holder or contributors be liable for
// any direct, indirect, incidental, special, exemplary, or consequential
// damages however caused and on any theory of liability, whether in
// contract, strict liability, or tort, arising in any way out of the use
// of this software, even if advised of the possibility of such damage.
const IAU_1980_NUTATION_TERMS: [NutationTerm; 106] = [
    NutationTerm {
        l: 0,
        l_prime: 0,
        f: 0,
        d: 0,
        omega: 1,
        dpsi_0: -171996.0,
        dpsi_t: -174.2,
        deps_0: 92025.0,
        deps_t: 8.9,
    },
    NutationTerm {
        l: 0,
        l_prime: 0,
        f: 0,
        d: 0,
        omega: 2,
        dpsi_0: 2062.0,
        dpsi_t: 0.2,
        deps_0: -895.0,
        deps_t: 0.5,
    },
    NutationTerm {
        l: -2,
        l_prime: 0,
        f: 2,
        d: 0,
        omega: 1,
        dpsi_0: 46.0,
        dpsi_t: 0.0,
        deps_0: -24.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 2,
        l_prime: 0,
        f: -2,
        d: 0,
        omega: 0,
        dpsi_0: 11.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: -2,
        l_prime: 0,
        f: 2,
        d: 0,
        omega: 2,
        dpsi_0: -3.0,
        dpsi_t: 0.0,
        deps_0: 1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: -1,
        f: 0,
        d: -1,
        omega: 0,
        dpsi_0: -3.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: -2,
        f: 2,
        d: -2,
        omega: 1,
        dpsi_0: -2.0,
        dpsi_t: 0.0,
        deps_0: 1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 2,
        l_prime: 0,
        f: -2,
        d: 0,
        omega: 1,
        dpsi_0: 1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 0,
        f: 2,
        d: -2,
        omega: 2,
        dpsi_0: -13187.0,
        dpsi_t: -1.6,
        deps_0: 5736.0,
        deps_t: -3.1,
    },
    NutationTerm {
        l: 0,
        l_prime: 1,
        f: 0,
        d: 0,
        omega: 0,
        dpsi_0: 1426.0,
        dpsi_t: -3.4,
        deps_0: 54.0,
        deps_t: -0.1,
    },
    NutationTerm {
        l: 0,
        l_prime: 1,
        f: 2,
        d: -2,
        omega: 2,
        dpsi_0: -517.0,
        dpsi_t: 1.2,
        deps_0: 224.0,
        deps_t: -0.6,
    },
    NutationTerm {
        l: 0,
        l_prime: -1,
        f: 2,
        d: -2,
        omega: 2,
        dpsi_0: 217.0,
        dpsi_t: -0.5,
        deps_0: -95.0,
        deps_t: 0.3,
    },
    NutationTerm {
        l: 0,
        l_prime: 0,
        f: 2,
        d: -2,
        omega: 1,
        dpsi_0: 129.0,
        dpsi_t: 0.1,
        deps_0: -70.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 2,
        l_prime: 0,
        f: 0,
        d: -2,
        omega: 0,
        dpsi_0: 48.0,
        dpsi_t: 0.0,
        deps_0: 1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 0,
        f: 2,
        d: -2,
        omega: 0,
        dpsi_0: -22.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 2,
        f: 0,
        d: 0,
        omega: 0,
        dpsi_0: 17.0,
        dpsi_t: -0.1,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 1,
        f: 0,
        d: 0,
        omega: 1,
        dpsi_0: -15.0,
        dpsi_t: 0.0,
        deps_0: 9.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 2,
        f: 2,
        d: -2,
        omega: 2,
        dpsi_0: -16.0,
        dpsi_t: 0.1,
        deps_0: 7.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: -1,
        f: 0,
        d: 0,
        omega: 1,
        dpsi_0: -12.0,
        dpsi_t: 0.0,
        deps_0: 6.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: -2,
        l_prime: 0,
        f: 0,
        d: 2,
        omega: 1,
        dpsi_0: -6.0,
        dpsi_t: 0.0,
        deps_0: 3.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: -1,
        f: 2,
        d: -2,
        omega: 1,
        dpsi_0: -5.0,
        dpsi_t: 0.0,
        deps_0: 3.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 2,
        l_prime: 0,
        f: 0,
        d: -2,
        omega: 1,
        dpsi_0: 4.0,
        dpsi_t: 0.0,
        deps_0: -2.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 1,
        f: 2,
        d: -2,
        omega: 1,
        dpsi_0: 4.0,
        dpsi_t: 0.0,
        deps_0: -2.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 0,
        f: 0,
        d: -1,
        omega: 0,
        dpsi_0: -4.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 2,
        l_prime: 1,
        f: 0,
        d: -2,
        omega: 0,
        dpsi_0: 1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 0,
        f: -2,
        d: 2,
        omega: 1,
        dpsi_0: 1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 1,
        f: -2,
        d: 2,
        omega: 0,
        dpsi_0: -1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 1,
        f: 0,
        d: 0,
        omega: 2,
        dpsi_0: 1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: -1,
        l_prime: 0,
        f: 0,
        d: 1,
        omega: 1,
        dpsi_0: 1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 1,
        f: 2,
        d: -2,
        omega: 0,
        dpsi_0: -1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 0,
        f: 2,
        d: 0,
        omega: 2,
        dpsi_0: -2274.0,
        dpsi_t: -0.2,
        deps_0: 977.0,
        deps_t: -0.5,
    },
    NutationTerm {
        l: 1,
        l_prime: 0,
        f: 0,
        d: 0,
        omega: 0,
        dpsi_0: 712.0,
        dpsi_t: 0.1,
        deps_0: -7.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 0,
        f: 2,
        d: 0,
        omega: 1,
        dpsi_0: -386.0,
        dpsi_t: -0.4,
        deps_0: 200.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 0,
        f: 2,
        d: 0,
        omega: 2,
        dpsi_0: -301.0,
        dpsi_t: 0.0,
        deps_0: 129.0,
        deps_t: -0.1,
    },
    NutationTerm {
        l: 1,
        l_prime: 0,
        f: 0,
        d: -2,
        omega: 0,
        dpsi_0: -158.0,
        dpsi_t: 0.0,
        deps_0: -1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: -1,
        l_prime: 0,
        f: 2,
        d: 0,
        omega: 2,
        dpsi_0: 123.0,
        dpsi_t: 0.0,
        deps_0: -53.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 0,
        f: 0,
        d: 2,
        omega: 0,
        dpsi_0: 63.0,
        dpsi_t: 0.0,
        deps_0: -2.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 0,
        f: 0,
        d: 0,
        omega: 1,
        dpsi_0: 63.0,
        dpsi_t: 0.1,
        deps_0: -33.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: -1,
        l_prime: 0,
        f: 0,
        d: 0,
        omega: 1,
        dpsi_0: -58.0,
        dpsi_t: -0.1,
        deps_0: 32.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: -1,
        l_prime: 0,
        f: 2,
        d: 2,
        omega: 2,
        dpsi_0: -59.0,
        dpsi_t: 0.0,
        deps_0: 26.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 0,
        f: 2,
        d: 0,
        omega: 1,
        dpsi_0: -51.0,
        dpsi_t: 0.0,
        deps_0: 27.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 0,
        f: 2,
        d: 2,
        omega: 2,
        dpsi_0: -38.0,
        dpsi_t: 0.0,
        deps_0: 16.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 2,
        l_prime: 0,
        f: 0,
        d: 0,
        omega: 0,
        dpsi_0: 29.0,
        dpsi_t: 0.0,
        deps_0: -1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 0,
        f: 2,
        d: -2,
        omega: 2,
        dpsi_0: 29.0,
        dpsi_t: 0.0,
        deps_0: -12.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 2,
        l_prime: 0,
        f: 2,
        d: 0,
        omega: 2,
        dpsi_0: -31.0,
        dpsi_t: 0.0,
        deps_0: 13.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 0,
        f: 2,
        d: 0,
        omega: 0,
        dpsi_0: 26.0,
        dpsi_t: 0.0,
        deps_0: -1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: -1,
        l_prime: 0,
        f: 2,
        d: 0,
        omega: 1,
        dpsi_0: 21.0,
        dpsi_t: 0.0,
        deps_0: -10.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: -1,
        l_prime: 0,
        f: 0,
        d: 2,
        omega: 1,
        dpsi_0: 16.0,
        dpsi_t: 0.0,
        deps_0: -8.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 0,
        f: 0,
        d: -2,
        omega: 1,
        dpsi_0: -13.0,
        dpsi_t: 0.0,
        deps_0: 7.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: -1,
        l_prime: 0,
        f: 2,
        d: 2,
        omega: 1,
        dpsi_0: -10.0,
        dpsi_t: 0.0,
        deps_0: 5.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 1,
        f: 0,
        d: -2,
        omega: 0,
        dpsi_0: -7.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 1,
        f: 2,
        d: 0,
        omega: 2,
        dpsi_0: 7.0,
        dpsi_t: 0.0,
        deps_0: -3.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: -1,
        f: 2,
        d: 0,
        omega: 2,
        dpsi_0: -7.0,
        dpsi_t: 0.0,
        deps_0: 3.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 0,
        f: 2,
        d: 2,
        omega: 2,
        dpsi_0: -8.0,
        dpsi_t: 0.0,
        deps_0: 3.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 0,
        f: 0,
        d: 2,
        omega: 0,
        dpsi_0: 6.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 2,
        l_prime: 0,
        f: 2,
        d: -2,
        omega: 2,
        dpsi_0: 6.0,
        dpsi_t: 0.0,
        deps_0: -3.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 0,
        f: 0,
        d: 2,
        omega: 1,
        dpsi_0: -6.0,
        dpsi_t: 0.0,
        deps_0: 3.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 0,
        f: 2,
        d: 2,
        omega: 1,
        dpsi_0: -7.0,
        dpsi_t: 0.0,
        deps_0: 3.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 0,
        f: 2,
        d: -2,
        omega: 1,
        dpsi_0: 6.0,
        dpsi_t: 0.0,
        deps_0: -3.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 0,
        f: 0,
        d: -2,
        omega: 1,
        dpsi_0: -5.0,
        dpsi_t: 0.0,
        deps_0: 3.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: -1,
        f: 0,
        d: 0,
        omega: 0,
        dpsi_0: 5.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 2,
        l_prime: 0,
        f: 2,
        d: 0,
        omega: 1,
        dpsi_0: -5.0,
        dpsi_t: 0.0,
        deps_0: 3.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 1,
        f: 0,
        d: -2,
        omega: 0,
        dpsi_0: -4.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 0,
        f: -2,
        d: 0,
        omega: 0,
        dpsi_0: 4.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 0,
        f: 0,
        d: 1,
        omega: 0,
        dpsi_0: -4.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 1,
        f: 0,
        d: 0,
        omega: 0,
        dpsi_0: -3.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 0,
        f: 2,
        d: 0,
        omega: 0,
        dpsi_0: 3.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: -1,
        f: 2,
        d: 0,
        omega: 2,
        dpsi_0: -3.0,
        dpsi_t: 0.0,
        deps_0: 1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: -1,
        l_prime: -1,
        f: 2,
        d: 2,
        omega: 2,
        dpsi_0: -3.0,
        dpsi_t: 0.0,
        deps_0: 1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: -2,
        l_prime: 0,
        f: 0,
        d: 0,
        omega: 1,
        dpsi_0: -2.0,
        dpsi_t: 0.0,
        deps_0: 1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 3,
        l_prime: 0,
        f: 2,
        d: 0,
        omega: 2,
        dpsi_0: -3.0,
        dpsi_t: 0.0,
        deps_0: 1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: -1,
        f: 2,
        d: 2,
        omega: 2,
        dpsi_0: -3.0,
        dpsi_t: 0.0,
        deps_0: 1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 1,
        f: 2,
        d: 0,
        omega: 2,
        dpsi_0: 2.0,
        dpsi_t: 0.0,
        deps_0: -1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: -1,
        l_prime: 0,
        f: 2,
        d: -2,
        omega: 1,
        dpsi_0: -2.0,
        dpsi_t: 0.0,
        deps_0: 1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 2,
        l_prime: 0,
        f: 0,
        d: 0,
        omega: 1,
        dpsi_0: 2.0,
        dpsi_t: 0.0,
        deps_0: -1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 0,
        f: 0,
        d: 0,
        omega: 2,
        dpsi_0: -2.0,
        dpsi_t: 0.0,
        deps_0: 1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 3,
        l_prime: 0,
        f: 0,
        d: 0,
        omega: 0,
        dpsi_0: 2.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 0,
        f: 2,
        d: 1,
        omega: 2,
        dpsi_0: 2.0,
        dpsi_t: 0.0,
        deps_0: -1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: -1,
        l_prime: 0,
        f: 0,
        d: 0,
        omega: 2,
        dpsi_0: 1.0,
        dpsi_t: 0.0,
        deps_0: -1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 0,
        f: 0,
        d: -4,
        omega: 0,
        dpsi_0: -1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: -2,
        l_prime: 0,
        f: 2,
        d: 2,
        omega: 2,
        dpsi_0: 1.0,
        dpsi_t: 0.0,
        deps_0: -1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: -1,
        l_prime: 0,
        f: 2,
        d: 4,
        omega: 2,
        dpsi_0: -2.0,
        dpsi_t: 0.0,
        deps_0: 1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 2,
        l_prime: 0,
        f: 0,
        d: -4,
        omega: 0,
        dpsi_0: -1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 1,
        f: 2,
        d: -2,
        omega: 2,
        dpsi_0: 1.0,
        dpsi_t: 0.0,
        deps_0: -1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 0,
        f: 2,
        d: 2,
        omega: 1,
        dpsi_0: -1.0,
        dpsi_t: 0.0,
        deps_0: 1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: -2,
        l_prime: 0,
        f: 2,
        d: 4,
        omega: 2,
        dpsi_0: -1.0,
        dpsi_t: 0.0,
        deps_0: 1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: -1,
        l_prime: 0,
        f: 4,
        d: 0,
        omega: 2,
        dpsi_0: 1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: -1,
        f: 0,
        d: -2,
        omega: 0,
        dpsi_0: 1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 2,
        l_prime: 0,
        f: 2,
        d: -2,
        omega: 1,
        dpsi_0: 1.0,
        dpsi_t: 0.0,
        deps_0: -1.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 2,
        l_prime: 0,
        f: 2,
        d: 2,
        omega: 2,
        dpsi_0: -1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 0,
        f: 0,
        d: 2,
        omega: 1,
        dpsi_0: -1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 0,
        f: 4,
        d: -2,
        omega: 2,
        dpsi_0: 1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 3,
        l_prime: 0,
        f: 2,
        d: -2,
        omega: 2,
        dpsi_0: 1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 0,
        f: 2,
        d: -2,
        omega: 0,
        dpsi_0: -1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 1,
        f: 2,
        d: 0,
        omega: 1,
        dpsi_0: 1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: -1,
        l_prime: -1,
        f: 0,
        d: 2,
        omega: 1,
        dpsi_0: 1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 0,
        f: -2,
        d: 0,
        omega: 1,
        dpsi_0: -1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 0,
        f: 2,
        d: -1,
        omega: 2,
        dpsi_0: -1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 1,
        f: 0,
        d: 2,
        omega: 0,
        dpsi_0: -1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 0,
        f: -2,
        d: -2,
        omega: 0,
        dpsi_0: -1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: -1,
        f: 2,
        d: 0,
        omega: 1,
        dpsi_0: -1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 1,
        f: 0,
        d: -2,
        omega: 1,
        dpsi_0: -1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 1,
        l_prime: 0,
        f: -2,
        d: 2,
        omega: 0,
        dpsi_0: -1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 2,
        l_prime: 0,
        f: 0,
        d: 2,
        omega: 0,
        dpsi_0: 1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 0,
        f: 2,
        d: 4,
        omega: 2,
        dpsi_0: -1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
    NutationTerm {
        l: 0,
        l_prime: 1,
        f: 0,
        d: 1,
        omega: 0,
        dpsi_0: 1.0,
        dpsi_t: 0.0,
        deps_0: 0.0,
        deps_t: 0.0,
    },
];

fn rotate_z(v: Vector3<f64>, theta: f64) -> Vector3<f64> {
    let (s, c) = theta.sin_cos();
    Vector3::new(c * v.x - s * v.y, s * v.x + c * v.y, v.z)
}

fn rotate_x(v: Vector3<f64>, theta: f64) -> Vector3<f64> {
    let (s, c) = theta.sin_cos();
    Vector3::new(v.x, c * v.y - s * v.z, s * v.y + c * v.z)
}

fn rotate_y(v: Vector3<f64>, theta: f64) -> Vector3<f64> {
    let (s, c) = theta.sin_cos();
    Vector3::new(c * v.x + s * v.z, v.y, -s * v.x + c * v.z)
}

fn erfa_rotate_z_matrix(mut matrix: Matrix3<f64>, psi: f64) -> Matrix3<f64> {
    let (s, c) = psi.sin_cos();
    let a00 = c * matrix[(0, 0)] + s * matrix[(1, 0)];
    let a01 = c * matrix[(0, 1)] + s * matrix[(1, 1)];
    let a02 = c * matrix[(0, 2)] + s * matrix[(1, 2)];
    let a10 = -s * matrix[(0, 0)] + c * matrix[(1, 0)];
    let a11 = -s * matrix[(0, 1)] + c * matrix[(1, 1)];
    let a12 = -s * matrix[(0, 2)] + c * matrix[(1, 2)];
    matrix[(0, 0)] = a00;
    matrix[(0, 1)] = a01;
    matrix[(0, 2)] = a02;
    matrix[(1, 0)] = a10;
    matrix[(1, 1)] = a11;
    matrix[(1, 2)] = a12;
    matrix
}

fn erfa_rotate_x_matrix(mut matrix: Matrix3<f64>, phi: f64) -> Matrix3<f64> {
    let (s, c) = phi.sin_cos();
    let a10 = c * matrix[(1, 0)] + s * matrix[(2, 0)];
    let a11 = c * matrix[(1, 1)] + s * matrix[(2, 1)];
    let a12 = c * matrix[(1, 2)] + s * matrix[(2, 2)];
    let a20 = -s * matrix[(1, 0)] + c * matrix[(2, 0)];
    let a21 = -s * matrix[(1, 1)] + c * matrix[(2, 1)];
    let a22 = -s * matrix[(1, 2)] + c * matrix[(2, 2)];
    matrix[(1, 0)] = a10;
    matrix[(1, 1)] = a11;
    matrix[(1, 2)] = a12;
    matrix[(2, 0)] = a20;
    matrix[(2, 1)] = a21;
    matrix[(2, 2)] = a22;
    matrix
}

fn erfa_rotate_y_matrix(mut matrix: Matrix3<f64>, theta: f64) -> Matrix3<f64> {
    let (s, c) = theta.sin_cos();
    let a00 = c * matrix[(0, 0)] - s * matrix[(2, 0)];
    let a01 = c * matrix[(0, 1)] - s * matrix[(2, 1)];
    let a02 = c * matrix[(0, 2)] - s * matrix[(2, 2)];
    let a20 = s * matrix[(0, 0)] + c * matrix[(2, 0)];
    let a21 = s * matrix[(0, 1)] + c * matrix[(2, 1)];
    let a22 = s * matrix[(0, 2)] + c * matrix[(2, 2)];
    matrix[(0, 0)] = a00;
    matrix[(0, 1)] = a01;
    matrix[(0, 2)] = a02;
    matrix[(2, 0)] = a20;
    matrix[(2, 1)] = a21;
    matrix[(2, 2)] = a22;
    matrix
}

/// Time-aware conversion from frame `From` to frame `To`.
///
/// Implementations are provided by [`FrameContext`] for frame pairs that
/// exist in the compiled profile surface. Unsupported static pairs have
/// no trait implementation and therefore fail at compile time. Runtime
/// profile-gated transforms should expose fallible helper
/// constructors that use [`FrameError::TransformNotAvailable`].
///
/// ECI/ECEF transforms take [`SimTime`] because all non-toy Earth-fixed
/// profiles are time-dependent. The [`FrameProfile::ToyFixedEarth`]
/// implementation ignores time and remains identity.
pub trait FrameTransform<From: CoreFrame, To: CoreFrame> {
    /// Convert a position vector from `From` to `To` at simulation time
    /// `t`.
    fn transform_position(&self, t: SimTime, p: Position3<From>) -> Position3<To>;
    /// Convert a velocity vector from `From` to `To` at the given
    /// simulation time and position. Position is needed for
    /// non-inertial frame transforms (transport rate); same-frame and
    /// toy-profile transforms ignore it.
    fn transform_velocity(
        &self,
        t: SimTime,
        v: Velocity3<From>,
        p: Position3<From>,
    ) -> Velocity3<To>;
}

// Identity transform for any same-frame pair, in any profile.
impl<F: CoreFrame> FrameTransform<F, F> for FrameContext {
    fn transform_position(&self, _t: SimTime, p: Position3<F>) -> Position3<F> {
        p
    }
    fn transform_velocity(&self, _t: SimTime, v: Velocity3<F>, _p: Position3<F>) -> Velocity3<F> {
        v
    }
}

// ECI ↔ ECEF. Toy-fixed-earth is identity for every `t`; WGS84 uniform
// rotation delegates to the time-aware helpers above.
impl FrameTransform<Eci, Ecef> for FrameContext {
    fn transform_position(&self, t: SimTime, p: Position3<Eci>) -> Position3<Ecef> {
        self.eci_to_ecef_position(t, p)
    }
    fn transform_velocity(
        &self,
        t: SimTime,
        v: Velocity3<Eci>,
        p: Position3<Eci>,
    ) -> Velocity3<Ecef> {
        self.eci_to_ecef_velocity(t, v, p)
    }
}

impl FrameTransform<Ecef, Eci> for FrameContext {
    fn transform_position(&self, t: SimTime, p: Position3<Ecef>) -> Position3<Eci> {
        self.ecef_to_eci_position(t, p)
    }
    fn transform_velocity(
        &self,
        t: SimTime,
        v: Velocity3<Ecef>,
        p: Position3<Ecef>,
    ) -> Velocity3<Eci> {
        self.ecef_to_eci_velocity(t, v, p)
    }
}

#[cfg(test)]
#[allow(clippy::excessive_precision, clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn frame_context_toy_default_profile() {
        let ctx = FrameContext::toy_fixed_earth();
        assert_eq!(ctx.profile(), FrameProfile::ToyFixedEarth);
        assert_eq!(ctx.profile().as_label(), "toy-fixed-earth");
    }

    #[test]
    fn frame_context_eci_to_ecef_identity_in_toy() {
        let ctx = FrameContext::toy_fixed_earth();
        let p_eci: Position3<Eci> = Position3::new(7000.0, 0.0, 0.0);
        let p_ecef: Position3<Ecef> = ctx.transform_position(SimTime::from_seconds(10.0), p_eci);
        assert_abs_diff_eq!(p_ecef.vector.x, 7000.0);
        assert_abs_diff_eq!(p_ecef.vector.y, 0.0);
        assert_abs_diff_eq!(p_ecef.vector.z, 0.0);
    }

    #[test]
    fn frame_context_round_trip_eci_ecef_eci() {
        let ctx = FrameContext::toy_fixed_earth();
        let p_in: Position3<Eci> = Position3::new(1.0, 2.0, 3.0);
        let t = SimTime::from_seconds(10.0);
        let p_ecef: Position3<Ecef> = ctx.transform_position(t, p_in);
        let p_back: Position3<Eci> = ctx.transform_position(t, p_ecef);
        assert_abs_diff_eq!(p_in.vector.x, p_back.vector.x);
        assert_abs_diff_eq!(p_in.vector.y, p_back.vector.y);
        assert_abs_diff_eq!(p_in.vector.z, p_back.vector.z);
    }

    // -----------------------------------------------------------------
    // WGS84 frame profile tests
    // -----------------------------------------------------------------

    mod wgs84 {
        use super::*;
        use approx::assert_abs_diff_eq;

        const KSC_LAT_DEG: f64 = 28.5729;
        const KSC_LON_DEG: f64 = -80.6490;

        fn ksc_origin() -> LocalGeodeticOrigin {
            LocalGeodeticOrigin::new_degrees(KSC_LAT_DEG, KSC_LON_DEG, 0.0)
                .expect("KSC origin must be valid")
        }

        #[test]
        fn frame_profile_label() {
            assert_eq!(
                FrameProfile::Wgs84UniformRotation.as_label(),
                "wgs84-uniform-rotation",
            );
        }

        #[test]
        fn local_geodetic_origin_rejects_out_of_envelope() {
            assert!(matches!(
                LocalGeodeticOrigin::new_degrees(91.0, 0.0, 0.0),
                Err(FrameError::InvalidGeodeticCoordinate { .. })
            ));
            assert!(matches!(
                LocalGeodeticOrigin::new_degrees(0.0, 200.0, 0.0),
                Err(FrameError::InvalidGeodeticCoordinate { .. })
            ));
            assert!(matches!(
                LocalGeodeticOrigin::new_degrees(0.0, 0.0, f64::NAN),
                Err(FrameError::InvalidGeodeticCoordinate { .. })
            ));
        }

        #[test]
        fn geodetic_to_ecef_at_equator_zero_height_matches_a() {
            let o = LocalGeodeticOrigin::new_degrees(0.0, 0.0, 0.0).unwrap();
            let p = o.to_ecef_position();
            assert_abs_diff_eq!(p.vector.x, WGS84_A_M, epsilon = 1.0e-6);
            assert_abs_diff_eq!(p.vector.y, 0.0, epsilon = 1.0e-9);
            assert_abs_diff_eq!(p.vector.z, 0.0, epsilon = 1.0e-9);
        }

        #[test]
        fn geodetic_to_ecef_at_north_pole_matches_polar_radius() {
            // Polar radius b = a · (1 - f).
            let polar_radius = WGS84_A_M * (1.0 - WGS84_FLATTENING);
            let o = LocalGeodeticOrigin::new_degrees(90.0, 0.0, 0.0).unwrap();
            let p = o.to_ecef_position();
            assert_abs_diff_eq!(p.vector.x, 0.0, epsilon = 1.0e-6);
            assert_abs_diff_eq!(p.vector.y, 0.0, epsilon = 1.0e-6);
            assert_abs_diff_eq!(p.vector.z, polar_radius, epsilon = 1.0e-6);
        }

        #[test]
        fn earth_rotation_angle_is_zero_for_toy_fixed_earth() {
            let ctx = FrameContext::toy_fixed_earth();
            assert_abs_diff_eq!(ctx.earth_rotation_angle(SimTime::from_seconds(1000.0)), 0.0);
        }

        #[test]
        fn earth_rotation_angle_scales_linearly_for_wgs84() {
            let ctx = FrameContext::wgs84_uniform_rotation(None);
            let theta = ctx.earth_rotation_angle(SimTime::from_seconds(1.0));
            assert_abs_diff_eq!(theta, WGS84_OMEGA_RAD_S, epsilon = 1.0e-15);
            let theta_3600 = ctx.earth_rotation_angle(SimTime::from_seconds(3600.0));
            assert_abs_diff_eq!(theta_3600, WGS84_OMEGA_RAD_S * 3600.0, epsilon = 1.0e-12);
        }

        #[test]
        fn stationary_rotating_frame_imu_truth_equator_matches_closed_form() {
            use crate::gravity::{GravityModel, PointMassGravity};

            let position_eci: Position3<Eci> = Position3::new(WGS84_A_M, 0.0, 0.0);
            let gravity_eci_m_s2 = PointMassGravity::wgs84()
                .gravity_eci_m_s2(position_eci, SimTime::ZERO)
                .unwrap();

            let truth = stationary_rotating_frame_imu_truth_eci(
                position_eci,
                gravity_eci_m_s2,
                WGS84_OMEGA_RAD_S,
            )
            .unwrap();

            let gravitational_magnitude = WGS84_MU_M3_S2 / (WGS84_A_M * WGS84_A_M);
            let centripetal_magnitude = WGS84_OMEGA_RAD_S * WGS84_OMEGA_RAD_S * WGS84_A_M;
            let expected_specific_force_x = gravitational_magnitude - centripetal_magnitude;

            assert_abs_diff_eq!(
                truth.specific_force_eci_m_s2.x,
                expected_specific_force_x,
                epsilon = 1.0e-12
            );
            assert_abs_diff_eq!(truth.specific_force_eci_m_s2.y, 0.0, epsilon = 1.0e-15);
            assert_abs_diff_eq!(truth.specific_force_eci_m_s2.z, 0.0, epsilon = 1.0e-15);
            assert_abs_diff_eq!(truth.angular_velocity_eci_rad_s.x, 0.0, epsilon = 0.0);
            assert_abs_diff_eq!(truth.angular_velocity_eci_rad_s.y, 0.0, epsilon = 0.0);
            assert_abs_diff_eq!(
                truth.angular_velocity_eci_rad_s.z,
                WGS84_OMEGA_RAD_S,
                epsilon = 0.0
            );
        }

        #[test]
        fn stationary_rotating_frame_gyro_projects_to_local_ned() {
            use crate::gravity::{GravityModel, PointMassGravity};

            let origin = ksc_origin();
            let position_eci = Position3::<Eci>::from_vector(origin.to_ecef_position().vector);
            let gravity_eci_m_s2 = PointMassGravity::wgs84()
                .gravity_eci_m_s2(position_eci, SimTime::ZERO)
                .unwrap();

            let truth = stationary_rotating_frame_imu_truth_eci(
                position_eci,
                gravity_eci_m_s2,
                WGS84_OMEGA_RAD_S,
            )
            .unwrap();

            let ctx = FrameContext::wgs84_uniform_rotation(Some(origin));
            let gyro_ned = ctx
                .ecef_to_ned_velocity(Velocity3::<Ecef>::from_vector(
                    truth.angular_velocity_eci_rad_s,
                ))
                .unwrap();
            let latitude_rad = KSC_LAT_DEG.to_radians();

            assert_abs_diff_eq!(
                gyro_ned.vector.x,
                WGS84_OMEGA_RAD_S * latitude_rad.cos(),
                epsilon = 1.0e-16
            );
            assert_abs_diff_eq!(gyro_ned.vector.y, 0.0, epsilon = 1.0e-20);
            assert_abs_diff_eq!(
                gyro_ned.vector.z,
                -WGS84_OMEGA_RAD_S * latitude_rad.sin(),
                epsilon = 1.0e-16
            );
        }

        #[test]
        fn stationary_rotating_frame_imu_truth_rejects_nonfinite_inputs() {
            let position_eci: Position3<Eci> = Position3::new(WGS84_A_M, 0.0, 0.0);
            let gravity_eci_m_s2 = Vector3::new(-9.0, 0.0, 0.0);

            assert!(matches!(
                stationary_rotating_frame_imu_truth_eci(
                    Position3::new(f64::NAN, 0.0, 0.0),
                    gravity_eci_m_s2,
                    WGS84_OMEGA_RAD_S,
                ),
                Err(FrameError::NotFinite { .. })
            ));
            assert!(matches!(
                stationary_rotating_frame_imu_truth_eci(
                    position_eci,
                    Vector3::new(f64::NAN, 0.0, 0.0),
                    WGS84_OMEGA_RAD_S,
                ),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                stationary_rotating_frame_imu_truth_eci(position_eci, gravity_eci_m_s2, f64::NAN,),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
        }

        #[test]
        fn eci_ecef_position_round_trip_at_arbitrary_time() {
            let ctx = FrameContext::wgs84_uniform_rotation(None);
            let t = SimTime::from_seconds(1234.5);
            let p_eci: Position3<Eci> = Position3::new(7_000_000.0, 1_500_000.0, 800_000.0);
            let p_ecef = ctx.eci_to_ecef_position(t, p_eci);
            let p_back = ctx.ecef_to_eci_position(t, p_ecef);
            assert_abs_diff_eq!(p_back.vector.x, p_eci.vector.x, epsilon = 1.0e-6);
            assert_abs_diff_eq!(p_back.vector.y, p_eci.vector.y, epsilon = 1.0e-6);
            assert_abs_diff_eq!(p_back.vector.z, p_eci.vector.z, epsilon = 1.0e-6);
        }

        #[test]
        fn frame_transform_trait_is_time_aware_for_wgs84_eci_ecef() {
            let ctx = FrameContext::wgs84_uniform_rotation(None);
            let t = SimTime::from_seconds(1234.5);
            let p_eci: Position3<Eci> = Position3::new(7_000_000.0, 1_500_000.0, 800_000.0);
            let via_trait: Position3<Ecef> = ctx.transform_position(t, p_eci);
            let via_helper = ctx.eci_to_ecef_position(t, p_eci);

            assert_abs_diff_eq!(via_trait.vector.x, via_helper.vector.x, epsilon = 0.0);
            assert_abs_diff_eq!(via_trait.vector.y, via_helper.vector.y, epsilon = 0.0);
            assert_abs_diff_eq!(via_trait.vector.z, via_helper.vector.z, epsilon = 0.0);
            assert!(
                (via_trait.vector.y - p_eci.vector.y).abs() > 1.0e-6,
                "WGS84 ECI/ECEF trait transform must not silently be identity at t != 0",
            );
        }

        #[test]
        fn eci_ecef_position_at_t_zero_is_identity_for_wgs84() {
            let ctx = FrameContext::wgs84_uniform_rotation(None);
            let p_eci: Position3<Eci> = Position3::new(7_000_000.0, 1_500_000.0, 800_000.0);
            let p_ecef = ctx.eci_to_ecef_position(SimTime::ZERO, p_eci);
            assert_abs_diff_eq!(p_ecef.vector.x, p_eci.vector.x, epsilon = 1.0e-15);
            assert_abs_diff_eq!(p_ecef.vector.y, p_eci.vector.y, epsilon = 1.0e-15);
            assert_abs_diff_eq!(p_ecef.vector.z, p_eci.vector.z, epsilon = 1.0e-15);
        }

        #[test]
        fn eci_ecef_velocity_round_trip_at_arbitrary_time() {
            let ctx = FrameContext::wgs84_uniform_rotation(None);
            let t = SimTime::from_seconds(60.0);
            let p_eci: Position3<Eci> = Position3::new(7_000_000.0, 0.0, 0.0);
            let v_eci: Velocity3<Eci> = Velocity3::new(0.0, 7_500.0, 0.0);
            let v_ecef = ctx.eci_to_ecef_velocity(t, v_eci, p_eci);
            let p_ecef = ctx.eci_to_ecef_position(t, p_eci);
            let v_back = ctx.ecef_to_eci_velocity(t, v_ecef, p_ecef);
            assert_abs_diff_eq!(v_back.vector.x, v_eci.vector.x, epsilon = 1.0e-6);
            assert_abs_diff_eq!(v_back.vector.y, v_eci.vector.y, epsilon = 1.0e-6);
            assert_abs_diff_eq!(v_back.vector.z, v_eci.vector.z, epsilon = 1.0e-6);
        }

        #[test]
        fn ned_position_at_local_origin_is_origin() {
            let origin = ksc_origin();
            let ctx = FrameContext::wgs84_uniform_rotation(Some(origin));
            let origin_ecef = origin.to_ecef_position();
            let p_ned = ctx.ecef_to_ned_position(origin_ecef).unwrap();
            assert_abs_diff_eq!(p_ned.vector.x, 0.0, epsilon = 1.0e-6);
            assert_abs_diff_eq!(p_ned.vector.y, 0.0, epsilon = 1.0e-6);
            assert_abs_diff_eq!(p_ned.vector.z, 0.0, epsilon = 1.0e-6);
        }

        #[test]
        fn ned_helpers_require_local_origin() {
            let ctx = FrameContext::wgs84_uniform_rotation(None);
            let p_ecef: Position3<Ecef> = Position3::origin();
            let result = ctx.ecef_to_ned_position(p_ecef);
            assert!(matches!(
                result,
                Err(FrameError::LocalOriginRequired { .. })
            ));
        }

        #[test]
        fn ned_velocity_round_trip_through_ecef() {
            let ctx = FrameContext::wgs84_uniform_rotation(Some(ksc_origin()));
            let v_ned: Velocity3<Ned> = Velocity3::new(10.0, 5.0, -3.0);
            let v_ecef = ctx.ned_to_ecef_velocity(v_ned).unwrap();
            let v_back = ctx.ecef_to_ned_velocity(v_ecef).unwrap();
            assert_abs_diff_eq!(v_back.vector.x, v_ned.vector.x, epsilon = 1.0e-12);
            assert_abs_diff_eq!(v_back.vector.y, v_ned.vector.y, epsilon = 1.0e-12);
            assert_abs_diff_eq!(v_back.vector.z, v_ned.vector.z, epsilon = 1.0e-12);
        }

        #[test]
        fn down_vector_at_local_origin_is_minus_geodetic_normal() {
            // The NED `+down` basis vector expressed in ECEF should be
            // anti-parallel to the outward geodetic normal at the
            // origin's latitude / longitude. (It is *not* anti-parallel
            // to the geocentric radial — the WGS84 ellipsoid's geodetic
            // vertical and geocentric radial differ at non-equatorial
            // latitudes by the geodetic-vs-geocentric latitude offset.
            // This test locks in the correct geodetic convention.)
            let origin = ksc_origin();
            let ctx = FrameContext::wgs84_uniform_rotation(Some(origin));
            let v_ned: Velocity3<Ned> = Velocity3::new(0.0, 0.0, 1.0);
            let v_ecef = ctx.ned_to_ecef_velocity(v_ned).unwrap();
            // Outward geodetic normal: (cos φ cos λ, cos φ sin λ, sin φ).
            let (sin_lat, cos_lat) = origin.latitude_rad.sin_cos();
            let (sin_lon, cos_lon) = origin.longitude_rad.sin_cos();
            let up_ecef = nalgebra::Vector3::new(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat);
            // down ≈ -up.
            let dot = v_ecef.vector.normalize().dot(&up_ecef);
            assert_abs_diff_eq!(dot, -1.0, epsilon = 1.0e-12);
        }
    }

    mod iers {
        use super::*;
        use approx::assert_abs_diff_eq;

        fn table() -> EarthOrientationTable {
            EarthOrientationTable::new(vec![
                EarthOrientationSample::new(
                    0.0,
                    0.1,
                    0.01 * ARCSECOND_TO_RAD,
                    -0.02 * ARCSECOND_TO_RAD,
                )
                .unwrap(),
                EarthOrientationSample::new(
                    10.0,
                    1.1,
                    0.03 * ARCSECOND_TO_RAD,
                    0.04 * ARCSECOND_TO_RAD,
                )
                .unwrap(),
            ])
            .unwrap()
        }

        #[test]
        fn frame_profile_label() {
            assert_eq!(FrameProfile::IersTabulated.as_label(), "iers-tabulated");
            assert_eq!(FrameProfile::IersCio.as_label(), "iers-cio");
        }

        #[test]
        fn earth_orientation_table_requires_strictly_increasing_times() {
            let err = EarthOrientationTable::new(vec![
                EarthOrientationSample::new(1.0, 0.0, 0.0, 0.0).unwrap(),
                EarthOrientationSample::new(1.0, 0.0, 0.0, 0.0).unwrap(),
            ])
            .unwrap_err();
            assert!(matches!(err, FrameError::InvalidFrameProfileData { .. }));
        }

        #[test]
        fn earth_orientation_table_interpolates_samples() {
            let sample = table().sample(SimTime::from_seconds(5.0));
            assert_abs_diff_eq!(sample.ut1_minus_utc_s, 0.6, epsilon = 1.0e-15);
            assert_eq!(sample.lod_s, None);
            assert_eq!(sample.cip_offset_x_rad.to_bits(), 0.0_f64.to_bits());
            assert_eq!(sample.cip_offset_y_rad.to_bits(), 0.0_f64.to_bits());
            assert_abs_diff_eq!(
                sample.polar_motion_x_rad,
                0.02 * ARCSECOND_TO_RAD,
                epsilon = 1.0e-20
            );
            assert_abs_diff_eq!(
                sample.polar_motion_y_rad,
                0.01 * ARCSECOND_TO_RAD,
                epsilon = 1.0e-20
            );
        }

        #[test]
        fn earth_orientation_table_interpolates_celestial_pole_offsets() {
            let table = EarthOrientationTable::new(vec![
                EarthOrientationSample::new(0.0, 0.0, 0.0, 0.0)
                    .unwrap()
                    .with_cip_offsets(0.10 * ARCSECOND_TO_RAD, -0.20 * ARCSECOND_TO_RAD)
                    .unwrap(),
                EarthOrientationSample::new(10.0, 0.0, 0.0, 0.0)
                    .unwrap()
                    .with_cip_offsets(0.30 * ARCSECOND_TO_RAD, 0.40 * ARCSECOND_TO_RAD)
                    .unwrap(),
            ])
            .unwrap();

            let sample = table.sample(SimTime::from_seconds(5.0));
            assert_abs_diff_eq!(
                sample.cip_offset_x_rad,
                0.20 * ARCSECOND_TO_RAD,
                epsilon = 1.0e-20
            );
            assert_abs_diff_eq!(
                sample.cip_offset_y_rad,
                0.10 * ARCSECOND_TO_RAD,
                epsilon = 1.0e-20
            );
        }

        fn assert_matrix_close(actual: Matrix3<f64>, expected: [[f64; 3]; 3], epsilon: f64) {
            for row in 0..3 {
                for col in 0..3 {
                    assert_abs_diff_eq!(actual[(row, col)], expected[row][col], epsilon = epsilon);
                }
            }
        }

        #[test]
        fn cio_primitives_match_erfa_reference_pins() {
            let era = earth_rotation_angle_iau2000_parts(J2000_JULIAN_DATE, -0.25).unwrap();
            assert_abs_diff_eq!(era, 3.319_864_341_135_049_47, epsilon = 1.0e-15);

            let sp = tio_locator_sp00_parts(J2000_JULIAN_DATE, 0.123_456_789).unwrap();
            assert_abs_diff_eq!(sp, -7.701_892_938_134_164_56e-16, epsilon = 1.0e-27);

            let rc2i =
                cio_celestial_to_intermediate_matrix(0.001_312_227_2, -0.000_002_928_1, -1.7e-9)
                    .unwrap();
            assert_matrix_close(
                rc2i,
                [
                    [
                        9.999_991_390_295_172_03e-1,
                        3.621_167_059_068_399_36e-9,
                        -1.312_227_199_995_022_29e-3,
                    ],
                    [
                        2.211_685_228_914_397_09e-10,
                        9.999_999_999_957_132_07e-1,
                        2.928_102_230_786_240_34e-6,
                    ],
                    [
                        1.312_227_200_000_000_08e-3,
                        -2.928_099_999_999_999_97e-6,
                        9.999_991_390_252_302_99e-1,
                    ],
                ],
                1.0e-15,
            );

            let rpom = polar_motion_matrix_iau2000(
                0.080_406 * ARCSECOND_TO_RAD,
                0.263_110 * ARCSECOND_TO_RAD,
                sp,
            )
            .unwrap();
            assert_matrix_close(
                rpom,
                [
                    [
                        9.999_999_999_999_240_61e-1,
                        -7.701_892_938_133_579_82e-16,
                        3.898_192_884_329_236_65e-7,
                    ],
                    [
                        4.980_210_526_170_005_72e-13,
                        9.999_999_999_991_864_29e-1,
                        -1.275_593_276_366_857_46e-6,
                    ],
                    [
                        -3.898_192_884_326_054_98e-7,
                        1.275_593_276_366_954_45e-6,
                        9.999_999_999_991_104_89e-1,
                    ],
                ],
                1.0e-15,
            );

            let rc2t = cio_celestial_to_terrestrial_matrix(rc2i, era, rpom).unwrap();
            assert_matrix_close(
                rc2t,
                [
                    [
                        -9.841_507_944_760_536_92e-1,
                        -1.773_289_211_424_053_38e-1,
                        1.291_301_135_665_221_15e-3,
                    ],
                    [
                        1.773_287_630_111_310_69e-1,
                        -9.841_516_416_231_509_41e-1,
                        -2.368_531_177_883_222_31e-4,
                    ],
                    [
                        1.312_837_040_341_384_82e-3,
                        -4.114_350_983_125_245_37e-6,
                        9.999_991_382_206_175_89e-1,
                    ],
                ],
                1.0e-15,
            );
        }

        #[test]
        fn cio_locator_s06_matches_erfa_reference_pins() {
            let x = 0.001_312_227_2;
            let y = -0.000_002_928_1;
            let s = cio_locator_s06_parts(J2000_JULIAN_DATE, 0.123_456_789, x, y).unwrap();
            assert_abs_diff_eq!(s, -7.836_488_806_556_619_13e-9, epsilon = 1.0e-23);

            let s = cio_locator_s06(J2000_JULIAN_DATE - 0.25, x, y).unwrap();
            assert_abs_diff_eq!(s, -7.833_537_865_332_714_09e-9, epsilon = 1.0e-23);

            let s = cio_locator_s06_parts(2_400_000.5, 59_000.25, 1.11e-3, -3.22e-6).unwrap();
            assert_abs_diff_eq!(s, -9.641_932_006_881_364_56e-9, epsilon = 1.0e-23);
        }

        #[test]
        fn fukushima_williams_iau2006_primitives_match_erfa_reference_pins() {
            let angles =
                fukushima_williams_angles_iau2006_parts(J2000_JULIAN_DATE, 0.123_456_789).unwrap();
            assert_abs_diff_eq!(
                angles.gamma_bar_rad,
                -2.564_291_976_780_594_82e-7,
                epsilon = 1.0e-21
            );
            assert_abs_diff_eq!(
                angles.phi_bar_rad,
                4.090_926_328_929_354_04e-1,
                epsilon = 1.0e-16
            );
            assert_abs_diff_eq!(
                angles.psi_bar_rad,
                -1.199_652_876_215_736_73e-7,
                epsilon = 1.0e-21
            );
            assert_abs_diff_eq!(
                angles.epsilon_a_rad,
                4.090_925_998_330_684_49e-1,
                epsilon = 1.0e-16
            );

            let matrix = fukushima_williams_matrix(
                angles.gamma_bar_rad,
                angles.phi_bar_rad,
                angles.psi_bar_rad,
                angles.epsilon_a_rad,
            )
            .unwrap();
            assert_matrix_close(
                matrix,
                [
                    [
                        9.999_999_999_999_881_21e-1,
                        -1.463_631_900_385_518_31e-7,
                        4.771_943_206_312_183_23e-8,
                    ],
                    [
                        1.463_631_884_609_539_72e-7,
                        9.999_999_999_999_886_76e-1,
                        3.305_986_427_238_804_38e-8,
                    ],
                    [
                        -4.771_943_690_186_848_25e-8,
                        -3.305_985_738_893_108_97e-8,
                        9.999_999_999_999_982_24e-1,
                    ],
                ],
                1.0e-15,
            );
            let (x, y) = cip_xy_from_bias_precession_nutation_matrix(matrix).unwrap();
            assert_abs_diff_eq!(x, -4.771_943_690_186_848_25e-8, epsilon = 1.0e-16);
            assert_abs_diff_eq!(y, -3.305_985_738_893_108_97e-8, epsilon = 1.0e-16);

            let angles = fukushima_williams_angles_iau2006_parts(2_400_000.5, 59_000.25).unwrap();
            assert_abs_diff_eq!(
                angles.gamma_bar_rad,
                1.029_000_168_533_993_02e-5,
                epsilon = 1.0e-20
            );
            assert_abs_diff_eq!(
                angles.phi_bar_rad,
                4.090_463_180_908_815_44e-1,
                epsilon = 1.0e-16
            );
            assert_abs_diff_eq!(
                angles.psi_bar_rad,
                4.986_380_615_734_828_90e-3,
                epsilon = 1.0e-18
            );
            assert_abs_diff_eq!(
                angles.epsilon_a_rad,
                4.090_462_492_407_350_16e-1,
                epsilon = 1.0e-16
            );
            let matrix = fukushima_williams_matrix(
                angles.gamma_bar_rad,
                angles.phi_bar_rad,
                angles.psi_bar_rad,
                angles.epsilon_a_rad,
            )
            .unwrap();
            assert_matrix_close(
                matrix,
                [
                    [
                        9.999_876_150_536_295_42e-1,
                        -4.564_698_135_609_840_91e-3,
                        -1.983_247_408_890_322_26e-3,
                    ],
                    [
                        4.564_698_251_919_902_32e-3,
                        9.999_895_817_006_831_94e-1,
                        -4.467_844_746_777_715_29e-6,
                    ],
                    [
                        1.983_247_141_187_783_30e-3,
                        -4.585_136_567_663_279_02e-6,
                        9.999_980_333_529_429_06e-1,
                    ],
                ],
                1.0e-15,
            );
            let (x, y) = cip_xy_from_bias_precession_nutation_matrix(matrix).unwrap();
            assert_abs_diff_eq!(x, 1.983_247_141_187_783_30e-3, epsilon = 1.0e-16);
            assert_abs_diff_eq!(y, -4.585_136_567_663_279_02e-6, epsilon = 1.0e-16);
        }

        #[test]
        fn iau2000a_nutation_primitives_match_erfa_reference_pins() {
            let (dpsi, deps) =
                nutation_angles_iau2000a_parts(J2000_JULIAN_DATE, 0.123_456_789).unwrap();
            assert_abs_diff_eq!(dpsi, -6.753_862_483_119_224_17e-5, epsilon = 1.0e-18);
            assert_abs_diff_eq!(deps, -2.798_320_418_915_259_54e-5, epsilon = 1.0e-18);

            let (dpsi, deps) =
                nutation_angles_iau2006a_parts(J2000_JULIAN_DATE, 0.123_456_789).unwrap();
            assert_abs_diff_eq!(dpsi, -6.753_865_655_345_028_15e-5, epsilon = 1.0e-18);
            assert_abs_diff_eq!(deps, -2.798_320_418_888_989_66e-5, epsilon = 1.0e-18);

            let (dpsi, deps) = nutation_angles_iau2000a_parts(2_400_000.5, 59_000.25).unwrap();
            assert_abs_diff_eq!(dpsi, -8.678_832_744_757_026_22e-5, epsilon = 1.0e-18);
            assert_abs_diff_eq!(deps, -1.419_559_879_539_360_48e-6, epsilon = 1.0e-18);

            let (dpsi, deps) = nutation_angles_iau2006a_parts(2_400_000.5, 59_000.25).unwrap();
            assert_abs_diff_eq!(dpsi, -8.678_831_900_799_636_62e-5, epsilon = 1.0e-18);
            assert_abs_diff_eq!(deps, -1.419_559_074_729_552_56e-6, epsilon = 1.0e-18);
        }

        #[test]
        fn cio_xys_iau2006a_matches_erfa_reference_pins() {
            let xys = cio_xys_iau2006a_parts(J2000_JULIAN_DATE, 0.123_456_789).unwrap();
            assert_abs_diff_eq!(xys.x_rad, -2.691_131_156_240_899_70e-5, epsilon = 1.0e-16);
            assert_abs_diff_eq!(xys.y_rad, -2.801_709_241_869_051_15e-5, epsilon = 1.0e-16);
            assert_abs_diff_eq!(xys.s_rad, -1.013_464_339_029_271_28e-8, epsilon = 1.0e-20);
            let rbpn =
                bias_precession_nutation_matrix_iau2006a_parts(J2000_JULIAN_DATE, 0.123_456_789)
                    .unwrap();
            assert_matrix_close(
                rbpn,
                [
                    [
                        9.999_999_977_270_406_24e-1,
                        6.181_914_725_004_830_80e-5,
                        2.691_304_351_431_068_63e-5,
                    ],
                    [
                        -6.181_990_122_862_773_50e-5,
                        9.999_999_976_967_177_68e-1,
                        2.801_542_872_067_017_62e-5,
                    ],
                    [
                        -2.691_131_156_240_899_70e-5,
                        -2.801_709_241_869_051_15e-5,
                        9.999_999_992_454_119_41e-1,
                    ],
                ],
                1.0e-15,
            );

            let xys = cio_xys_iau2006a_parts(2_400_000.5, 59_000.25).unwrap();
            assert_abs_diff_eq!(xys.x_rad, 1.948_722_490_818_444_65e-3, epsilon = 1.0e-16);
            assert_abs_diff_eq!(xys.y_rad, -5.848_488_218_894_286_62e-6, epsilon = 1.0e-16);
            assert_abs_diff_eq!(xys.s_rad, -5.730_491_742_158_362_81e-9, epsilon = 1.0e-20);
            let rbpn =
                bias_precession_nutation_matrix_iau2006a_parts(2_400_000.5, 59_000.25).unwrap();
            assert_matrix_close(
                rbpn,
                [
                    [
                        9.999_880_432_260_022_10e-1,
                        -4.485_070_773_392_563_39e-3,
                        -1.948_729_121_472_727_02e-3,
                    ],
                    [
                        4.485_073_654_355_566_27e-3,
                        9.999_899_420_023_947_72e-1,
                        -2.891_739_999_130_494_45e-6,
                    ],
                    [
                        1.948_722_490_818_444_65e-3,
                        -5.848_488_218_894_286_62e-6,
                        9.999_981_012_214_218_53e-1,
                    ],
                ],
                1.0e-15,
            );
        }

        #[test]
        fn cio_xys_iau2006a_matches_iers_tn36_table_reference_pin() {
            // Independent pins generated from IERS TN36 Ch.5 electronic Tables
            // 5.2a, 5.2b, and 5.2d. Those tabulated series have a
            // 0.1-microarcsecond cutoff and differ slightly from the ERFA/SOFA
            // implementation notes, so this check uses a 1-microarcsecond
            // element tolerance for X/Y and a sub-microarcsecond s tolerance.
            let xys = cio_xys_iau2006a_parts(J2000_JULIAN_DATE, 0.123_456_789).unwrap();
            assert_abs_diff_eq!(xys.x_rad, -2.691_131_097_043_647_77e-5, epsilon = 5.0e-12);
            assert_abs_diff_eq!(xys.y_rad, -2.801_709_414_731_057_97e-5, epsilon = 5.0e-12);
            assert_abs_diff_eq!(xys.s_rad, -1.013_464_340_526_123_92e-8, epsilon = 1.0e-15);

            let xys = cio_xys_iau2006a_parts(2_400_000.5, 59_000.25).unwrap();
            assert_abs_diff_eq!(xys.x_rad, 1.948_722_492_366_840_61e-3, epsilon = 5.0e-12);
            assert_abs_diff_eq!(xys.y_rad, -5.848_488_732_060_917_12e-6, epsilon = 5.0e-12);
            assert_abs_diff_eq!(xys.s_rad, -5.730_491_237_620_793_09e-9, epsilon = 1.0e-15);
        }

        #[test]
        fn cio_frame_model_can_generate_iau2006a_xys() {
            let desired_tt_julian_date = J2000_JULIAN_DATE + 0.123_456_789;
            let tai_minus_utc_s = 37.0;
            let epoch_utc_julian_date =
                desired_tt_julian_date - (tai_minus_utc_s + TT_MINUS_TAI_S) / SECONDS_PER_DAY;
            let earth_orientation = EarthOrientationTable::new(vec![
                EarthOrientationSample::new(0.0, 0.0, 0.0, 0.0).unwrap(),
            ])
            .unwrap();
            let model =
                CioFrameModel::iau2006a(epoch_utc_julian_date, tai_minus_utc_s, earth_orientation)
                    .unwrap();

            assert!(model.xys_table().is_none());
            assert!(model.uses_generated_iau2006a_xys());
            let sample = model.sample(SimTime::ZERO).unwrap();
            assert_abs_diff_eq!(
                sample.tt_julian_date,
                desired_tt_julian_date,
                epsilon = 1.0e-12
            );
            assert_abs_diff_eq!(
                sample.x_rad,
                -2.691_131_156_240_899_70e-5,
                epsilon = 1.0e-16
            );
            assert_abs_diff_eq!(
                sample.y_rad,
                -2.801_709_241_869_051_15e-5,
                epsilon = 1.0e-16
            );
            assert_abs_diff_eq!(
                sample.s_rad,
                -1.013_464_339_029_271_28e-8,
                epsilon = 1.0e-20
            );
        }

        #[test]
        fn cio_xys_table_interpolates_samples() {
            let table = CioXysTable::new(vec![
                CioXysSample::new(0.0, 1.0e-6, -2.0e-6, 3.0e-9).unwrap(),
                CioXysSample::new(10.0, 3.0e-6, 2.0e-6, 7.0e-9).unwrap(),
            ])
            .unwrap();

            let sample = table.sample(SimTime::from_seconds(5.0)).unwrap();
            assert_abs_diff_eq!(sample.time_s, 5.0, epsilon = 0.0);
            assert_abs_diff_eq!(sample.x_rad, 2.0e-6, epsilon = 1.0e-21);
            assert_abs_diff_eq!(sample.y_rad, 0.0, epsilon = 1.0e-21);
            assert_abs_diff_eq!(sample.s_rad, 5.0e-9, epsilon = 1.0e-24);
        }

        #[test]
        fn cio_frame_model_reproduces_erfa_reference_matrix_pin() {
            let desired_ut1_julian_date = J2000_JULIAN_DATE - 0.25;
            let desired_tt_julian_date = J2000_JULIAN_DATE + 0.123_456_789;
            let tai_minus_utc_s = (desired_tt_julian_date - desired_ut1_julian_date)
                * SECONDS_PER_DAY
                - TT_MINUS_TAI_S;
            let earth_orientation = EarthOrientationTable::new(vec![
                EarthOrientationSample::new(
                    0.0,
                    0.0,
                    0.080_406 * ARCSECOND_TO_RAD,
                    0.263_110 * ARCSECOND_TO_RAD,
                )
                .unwrap(),
            ])
            .unwrap();
            let xys = CioXysTable::new(vec![
                CioXysSample::new(0.0, 0.001_312_227_2, -0.000_002_928_1, -1.7e-9).unwrap(),
            ])
            .unwrap();
            let model = CioFrameModel::new(
                desired_ut1_julian_date,
                tai_minus_utc_s,
                earth_orientation,
                xys,
            )
            .unwrap();

            let actual = model
                .gcrs_to_itrs_matrix(SimTime::from_seconds(0.0))
                .unwrap();
            assert_matrix_close(
                actual,
                [
                    [
                        -9.841_507_944_760_536_92e-1,
                        -1.773_289_211_424_053_38e-1,
                        1.291_301_135_665_221_15e-3,
                    ],
                    [
                        1.773_287_630_111_310_69e-1,
                        -9.841_516_416_231_509_41e-1,
                        -2.368_531_177_883_222_31e-4,
                    ],
                    [
                        1.312_837_040_341_384_82e-3,
                        -4.114_350_983_125_245_37e-6,
                        9.999_991_382_206_175_89e-1,
                    ],
                ],
                1.0e-14,
            );

            let inverse = model
                .itrs_to_gcrs_matrix(SimTime::from_seconds(0.0))
                .unwrap();
            assert_matrix_close(
                inverse * actual,
                [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                1.0e-14,
            );
        }

        #[test]
        fn cio_frame_model_generated_xys_matches_erfa_c2tcio_grid() {
            let epoch_utc_julian_date = 2_400_000.5 + 59_000.0;
            let tai_minus_utc_s = 37.0;
            let earth_orientation = EarthOrientationTable::new(vec![
                EarthOrientationSample::new(
                    0.0,
                    -0.120,
                    0.050 * ARCSECOND_TO_RAD,
                    0.310 * ARCSECOND_TO_RAD,
                )
                .unwrap()
                .with_cip_offsets(0.000_200 * ARCSECOND_TO_RAD, -0.000_100 * ARCSECOND_TO_RAD)
                .unwrap(),
                EarthOrientationSample::new(
                    43_200.0,
                    -0.110,
                    0.060 * ARCSECOND_TO_RAD,
                    0.300 * ARCSECOND_TO_RAD,
                )
                .unwrap()
                .with_cip_offsets(0.000_210 * ARCSECOND_TO_RAD, -0.000_110 * ARCSECOND_TO_RAD)
                .unwrap(),
                EarthOrientationSample::new(
                    86_400.0,
                    -0.100,
                    0.070 * ARCSECOND_TO_RAD,
                    0.290 * ARCSECOND_TO_RAD,
                )
                .unwrap()
                .with_cip_offsets(0.000_230 * ARCSECOND_TO_RAD, -0.000_130 * ARCSECOND_TO_RAD)
                .unwrap(),
            ])
            .unwrap();
            let model =
                CioFrameModel::iau2006a(epoch_utc_julian_date, tai_minus_utc_s, earth_orientation)
                    .unwrap();

            // Generated from upstream ERFA: Xys06a, C2ixys, Era00, Sp00,
            // Pom00, and C2tcio, with dX/dY added before C2ixys.
            let expected = [
                (
                    0.0,
                    [
                        [
                            -3.633_719_946_373_014_65e-1,
                            -9.316_438_694_510_980_06e-1,
                            7.028_708_945_838_188_85e-4,
                        ],
                        [
                            9.316_420_934_663_464_10e-1,
                            -3.633_726_743_658_914_90e-1,
                            -1.819_122_267_292_602_60e-3,
                        ],
                        [
                            1.950_178_184_804_003_59e-3,
                            -6.193_975_088_610_687_63e-6,
                            9.999_980_983_815_329_74e-1,
                        ],
                    ],
                ),
                (
                    21_600.0,
                    [
                        [
                            9.331_963_062_213_082_25e-1,
                            -3.593_624_079_815_442_30e-1,
                            -1.820_380_348_272_124_16e-3,
                        ],
                        [
                            3.593_617_333_968_537_89e-1,
                            9.331_980_817_117_988_55e-1,
                            -6.963_184_812_717_321_26e-4,
                        ],
                        [
                            1.949_006_135_145_264_43e-3,
                            -4.373_202_720_227_257_70e-6,
                            9.999_981_006_761_764_49e-1,
                        ],
                    ],
                ),
                (
                    86_400.0,
                    [
                        [
                            -3.472_913_803_685_841_46e-1,
                            -9.377_570_292_744_352_72e-1,
                            6.716_904_565_096_755_08e-4,
                        ],
                        [
                            9.377_552_418_671_643_02e-1,
                            -3.472_920_298_689_935_46e-1,
                            -1.830_939_685_091_581_08e-3,
                        ],
                        [
                            1.950_249_301_957_026_91e-3,
                            -5.988_324_102_978_946_78e-6,
                            9.999_980_982_440_916_93e-1,
                        ],
                    ],
                ),
            ];

            for (time_s, expected_matrix) in expected {
                let actual = model
                    .gcrs_to_itrs_matrix(SimTime::from_seconds(time_s))
                    .unwrap();
                assert_matrix_close(actual, expected_matrix, 5.0e-14);
            }
        }

        #[test]
        fn cio_frame_model_adds_eop_cip_offsets_before_matrix_build() {
            let earth_orientation = EarthOrientationTable::new(vec![
                EarthOrientationSample::new(0.0, 0.0, 0.0, 0.0)
                    .unwrap()
                    .with_cip_offsets(1.0e-8, -2.0e-8)
                    .unwrap(),
                EarthOrientationSample::new(10.0, 0.2, 0.01 * ARCSECOND_TO_RAD, 0.0)
                    .unwrap()
                    .with_cip_offsets(3.0e-8, -4.0e-8)
                    .unwrap(),
            ])
            .unwrap();
            let xys = CioXysTable::new(vec![
                CioXysSample::new(0.0, 1.0e-3, -2.0e-6, 1.0e-9).unwrap(),
                CioXysSample::new(10.0, 1.2e-3, -4.0e-6, 5.0e-9).unwrap(),
            ])
            .unwrap();
            let model =
                CioFrameModel::new(J2000_JULIAN_DATE, 37.0, earth_orientation, xys).unwrap();

            let t = SimTime::from_seconds(5.0);
            let sample = model.sample(t).unwrap();
            assert_abs_diff_eq!(sample.x_rad, 1.1e-3 + 2.0e-8, epsilon = 1.0e-18);
            assert_abs_diff_eq!(sample.y_rad, -3.0e-6 - 3.0e-8, epsilon = 1.0e-20);
            assert_abs_diff_eq!(sample.s_rad, 3.0e-9, epsilon = 1.0e-24);
            assert_abs_diff_eq!(
                sample.earth_orientation.ut1_minus_utc_s,
                0.1,
                epsilon = 1.0e-15
            );

            let actual = model.gcrs_to_itrs_matrix(t).unwrap();
            let rc2i =
                cio_celestial_to_intermediate_matrix(sample.x_rad, sample.y_rad, sample.s_rad)
                    .unwrap();
            let era = earth_rotation_angle_iau2000(sample.ut1_julian_date).unwrap();
            let sp = tio_locator_sp00(sample.tt_julian_date).unwrap();
            let rpom = polar_motion_matrix_iau2000(
                sample.earth_orientation.polar_motion_x_rad,
                sample.earth_orientation.polar_motion_y_rad,
                sp,
            )
            .unwrap();
            let expected = cio_celestial_to_terrestrial_matrix(rc2i, era, rpom).unwrap();
            for row in 0..3 {
                for col in 0..3 {
                    assert_abs_diff_eq!(
                        actual[(row, col)],
                        expected[(row, col)],
                        epsilon = 1.0e-15
                    );
                }
            }
        }

        #[test]
        fn cio_frame_model_rejects_invalid_inputs() {
            assert!(matches!(
                CioXysSample::new(0.0, 1.0, 0.0, 0.0),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                CioXysTable::new(vec![
                    CioXysSample::new(0.0, 1.0e-6, 0.0, 0.0).unwrap(),
                    CioXysSample::new(0.0, 2.0e-6, 0.0, 0.0).unwrap(),
                ]),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));

            let earth_orientation = EarthOrientationTable::new(vec![
                EarthOrientationSample::new(0.0, 0.0, 0.0, 0.0)
                    .unwrap()
                    .with_cip_offsets(0.30, 0.0)
                    .unwrap(),
            ])
            .unwrap();
            let xys =
                CioXysTable::new(vec![CioXysSample::new(0.0, 0.75, 0.0, 0.0).unwrap()]).unwrap();
            let model =
                CioFrameModel::new(J2000_JULIAN_DATE, 37.0, earth_orientation, xys).unwrap();

            assert!(matches!(
                model.sample(SimTime::from_seconds(0.0)),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                model.sample(SimTime::from_seconds(f64::NAN)),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                CioFrameModel::new(
                    f64::NAN,
                    37.0,
                    EarthOrientationTable::new(vec![
                        EarthOrientationSample::new(0.0, 0.0, 0.0, 0.0).unwrap(),
                    ])
                    .unwrap(),
                    CioXysTable::new(vec![CioXysSample::new(0.0, 0.0, 0.0, 0.0).unwrap()]).unwrap(),
                ),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
        }

        #[test]
        fn iers_cio_context_uses_cio_model_matrix() {
            let earth_orientation = EarthOrientationTable::new(vec![
                EarthOrientationSample::new(
                    0.0,
                    0.0,
                    0.080_406 * ARCSECOND_TO_RAD,
                    0.263_110 * ARCSECOND_TO_RAD,
                )
                .unwrap(),
            ])
            .unwrap();
            let xys = CioXysTable::new(vec![
                CioXysSample::new(0.0, 0.001_312_227_2, -0.000_002_928_1, -1.7e-9).unwrap(),
            ])
            .unwrap();
            let ctx = FrameContext::iers_cio(
                J2000_JULIAN_DATE - 0.25,
                37.0,
                None,
                earth_orientation,
                xys,
            )
            .unwrap();
            assert_eq!(ctx.profile(), FrameProfile::IersCio);
            let expected = ctx
                .cio_frame_model()
                .unwrap()
                .gcrs_to_itrs_matrix(SimTime::ZERO)
                .unwrap();
            let actual = ctx.eci_to_ecef_rotation_matrix(SimTime::ZERO);
            for row in 0..3 {
                for col in 0..3 {
                    assert_abs_diff_eq!(actual[(row, col)], expected[(row, col)], epsilon = 0.0);
                }
            }
        }

        #[test]
        fn iers_cio_context_can_generate_iau2006a_xys() {
            let earth_orientation = EarthOrientationTable::new(vec![
                EarthOrientationSample::new(
                    0.0,
                    0.0,
                    0.080_406 * ARCSECOND_TO_RAD,
                    0.263_110 * ARCSECOND_TO_RAD,
                )
                .unwrap(),
            ])
            .unwrap();
            let ctx = FrameContext::iers_cio_iau2006a(
                J2000_JULIAN_DATE - 0.25,
                37.0,
                None,
                earth_orientation,
            )
            .unwrap();
            assert_eq!(ctx.profile(), FrameProfile::IersCio);
            let model = ctx.cio_frame_model().unwrap();
            assert!(model.xys_table().is_none());
            assert!(model.uses_generated_iau2006a_xys());
            let expected = model.gcrs_to_itrs_matrix(SimTime::ZERO).unwrap();
            let actual = ctx.eci_to_ecef_rotation_matrix(SimTime::ZERO);
            for row in 0..3 {
                for col in 0..3 {
                    assert_abs_diff_eq!(actual[(row, col)], expected[(row, col)], epsilon = 0.0);
                }
            }
        }

        #[test]
        fn cio_primitives_reject_invalid_inputs() {
            assert!(matches!(
                earth_rotation_angle_iau2000(f64::NAN),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                tio_locator_sp00(f64::INFINITY),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                cio_locator_s06(f64::NAN, 0.0, 0.0),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                cio_locator_s06(J2000_JULIAN_DATE, f64::NAN, 0.0),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                cio_locator_s06(J2000_JULIAN_DATE, 1.0, 0.0),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                mean_obliquity_iau2006(f64::NAN),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                fukushima_williams_angles_iau2006(f64::NEG_INFINITY),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                fukushima_williams_matrix(0.0, 0.0, f64::NAN, 0.0),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                cip_xy_from_bias_precession_nutation_matrix(Matrix3::new(
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    0.0,
                    f64::NAN,
                    0.0,
                    1.0,
                )),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                nutation_angles_iau2000a(f64::NAN),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                nutation_angles_iau2006a(f64::INFINITY),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                bias_precession_nutation_matrix_iau2006a(f64::NAN),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                cio_xys_iau2006a(f64::NEG_INFINITY),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                cio_celestial_to_intermediate_matrix(1.0, 0.0, 0.0),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                polar_motion_matrix_iau2000(0.0, f64::NAN, 0.0),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                cio_celestial_to_terrestrial_matrix(
                    Matrix3::identity(),
                    f64::NEG_INFINITY,
                    Matrix3::identity()
                ),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
        }

        #[test]
        fn earth_orientation_sample_rejects_nonfinite_celestial_pole_offsets() {
            let err = EarthOrientationSample::new(0.0, 0.0, 0.0, 0.0)
                .unwrap()
                .with_cip_offsets(f64::NAN, 0.0)
                .unwrap_err();
            assert!(matches!(err, FrameError::InvalidFrameProfileData { .. }));
        }

        #[test]
        fn time_scale_bridge_converts_utc_offsets() {
            let sample = EarthOrientationSample::new(0.0, 0.3345, 0.0, 0.0).unwrap();
            let bridge = TimeScaleBridge::from_earth_orientation_sample(37.0, sample).unwrap();

            assert_abs_diff_eq!(bridge.tai_minus_utc_s(), 37.0, epsilon = 0.0);
            assert_abs_diff_eq!(bridge.ut1_minus_utc_s(), 0.3345, epsilon = 0.0);
            assert_abs_diff_eq!(bridge.utc_seconds_to_ut1(10.0).unwrap(), 10.3345);
            assert_abs_diff_eq!(bridge.utc_seconds_to_tai(10.0).unwrap(), 47.0);
            assert_abs_diff_eq!(
                bridge.utc_seconds_to_tt(10.0).unwrap(),
                79.184,
                epsilon = 1.0e-14
            );

            let utc = J2000_JULIAN_DATE;
            assert_abs_diff_eq!(
                bridge.utc_julian_date_to_ut1(utc).unwrap(),
                utc + 0.3345 / SECONDS_PER_DAY,
                epsilon = 1.0e-15
            );
            assert_abs_diff_eq!(
                bridge.utc_julian_date_to_tt(utc).unwrap(),
                utc + (37.0 + TT_MINUS_TAI_S) / SECONDS_PER_DAY,
                epsilon = 1.0e-15
            );
        }

        #[test]
        fn time_scale_bridge_matches_existing_utc_to_tdb_pin() {
            let bridge = TimeScaleBridge::new(37.0, 0.0).unwrap();
            let utc = 2_457_754.5; // 2017-01-01T00:00:00 UTC.
            let tt = bridge.utc_julian_date_to_tt(utc).unwrap();
            let tdb = bridge.utc_julian_date_to_tdb_approx(utc).unwrap();
            let tdb_from_tt = TimeScaleBridge::tt_julian_date_to_tdb_approx(tt).unwrap();
            let offset_s = (tdb - utc) * SECONDS_PER_DAY;

            assert!((69.18..69.19).contains(&offset_s));
            assert_abs_diff_eq!(tdb, tdb_from_tt, epsilon = 1.0e-15);
            assert!(TimeScaleBridge::tdb_minus_tt_approx_s(tt).unwrap().abs() < 0.002);
        }

        #[test]
        fn time_scale_bridge_matches_erfa_dtdb_reference_pins() {
            let tt = 2_457_754.5 + 69.184 / SECONDS_PER_DAY;
            let ut1_fraction = 0.591_297_5 / SECONDS_PER_DAY;
            assert_abs_diff_eq!(
                TimeScaleBridge::tdb_minus_tt_erfa_approx_s(tt, ut1_fraction, 0.0, 0.0, 0.0)
                    .unwrap(),
                -4.949_663_477_050_876e-5,
                epsilon = 1.0e-14
            );
            assert_abs_diff_eq!(
                TimeScaleBridge::tdb_minus_tt_erfa_approx_s(
                    tt,
                    ut1_fraction,
                    -1.407_821_092,
                    3_914.0,
                    5_040.0,
                )
                .unwrap(),
                -5.091_806_148_981_582e-5,
                epsilon = 1.0e-14
            );
            assert_abs_diff_eq!(
                TimeScaleBridge::tdb_minus_tt_erfa_approx_s(
                    J2000_JULIAN_DATE,
                    0.5,
                    -1.407_821_092,
                    3_914.0,
                    5_040.0,
                )
                .unwrap(),
                -9.813_479_664_719_177e-5,
                epsilon = 1.0e-14
            );
            assert_abs_diff_eq!(
                TimeScaleBridge::tdb_minus_tt_erfa_approx_s(
                    2_450_123.7,
                    0.1,
                    1.234,
                    5_000.0,
                    -1_000.0,
                )
                .unwrap(),
                1.010_099_868_717_604e-3,
                epsilon = 1.0e-14
            );
        }

        #[test]
        fn time_scale_bridge_converts_utc_epoch_with_erfa_dtdb_reference() {
            let bridge = TimeScaleBridge::new(37.0, 0.591_297_5).unwrap();
            let utc = 2_457_754.5;
            let tdb = bridge
                .utc_julian_date_to_tdb_erfa_approx(utc, 0.0, 0.0, 0.0)
                .unwrap();
            let offset_s = (tdb - utc) * SECONDS_PER_DAY;

            assert_abs_diff_eq!(
                offset_s,
                69.184 - 4.949_663_477_050_876e-5,
                epsilon = 4.0e-5
            );
        }

        #[test]
        fn time_scale_bridge_rejects_nonfinite_inputs() {
            assert!(matches!(
                TimeScaleBridge::new(f64::NAN, 0.0),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            let bridge = TimeScaleBridge::new(37.0, 0.1).unwrap();
            assert!(matches!(
                bridge.utc_seconds_to_ut1(f64::INFINITY),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                bridge.utc_julian_date_to_tdb_approx(f64::NAN),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                TimeScaleBridge::tt_julian_date_to_tdb_approx(f64::NEG_INFINITY),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                TimeScaleBridge::tdb_minus_tt_erfa_approx_s(f64::NAN, 0.0, 0.0, 0.0, 0.0),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
            assert!(matches!(
                TimeScaleBridge::tt_julian_date_to_tdb_erfa_approx(
                    J2000_JULIAN_DATE,
                    f64::INFINITY,
                    0.0,
                    0.0,
                    0.0
                ),
                Err(FrameError::InvalidFrameProfileData { .. })
            ));
        }

        #[test]
        fn earth_orientation_table_uses_lod_for_ut1_hermite_interpolation() {
            let table = EarthOrientationTable::new(vec![
                EarthOrientationSample::new_with_lod(0.0, 0.0, 0.0, 0.0, -8_640.0).unwrap(),
                EarthOrientationSample::new_with_lod(10.0, 0.0, 0.0, 0.0, 0.0).unwrap(),
            ])
            .unwrap();
            let sample = table.sample(SimTime::from_seconds(5.0));
            assert_abs_diff_eq!(sample.ut1_minus_utc_s, 0.125, epsilon = 1.0e-15);
            assert_abs_diff_eq!(sample.lod_s.unwrap(), -4_320.0, epsilon = 1.0e-15);
        }

        #[test]
        fn iers_angular_velocity_uses_sampled_lod() {
            let table = EarthOrientationTable::new(vec![
                EarthOrientationSample::new_with_lod(0.0, 0.0, 0.0, 0.0, 1.0).unwrap(),
                EarthOrientationSample::new_with_lod(10.0, 0.0, 0.0, 0.0, 3.0).unwrap(),
            ])
            .unwrap();
            let ctx = FrameContext::iers_tabulated(2_451_545.0, None, table).unwrap();
            let expected = EARTH_ROTATION_ANGLE_RATE_RAD_S * (1.0 - 2.0 / SECONDS_PER_DAY);
            assert_abs_diff_eq!(
                ctx.angular_velocity_z_at(SimTime::from_seconds(5.0)),
                expected,
                epsilon = 1.0e-18
            );
        }

        #[test]
        fn iers_rotation_uses_interpolated_ut1() {
            let base = EarthOrientationTable::new(vec![
                EarthOrientationSample::new(0.0, 0.0, 0.0, 0.0).unwrap(),
                EarthOrientationSample::new(10.0, 0.0, 0.0, 0.0).unwrap(),
            ])
            .unwrap();
            let shifted = EarthOrientationTable::new(vec![
                EarthOrientationSample::new(0.0, 0.0, 0.0, 0.0).unwrap(),
                EarthOrientationSample::new(10.0, 1.0, 0.0, 0.0).unwrap(),
            ])
            .unwrap();
            let epoch = 2_451_545.0;
            let base_ctx = FrameContext::iers_tabulated(epoch, None, base).unwrap();
            let shifted_ctx = FrameContext::iers_tabulated(epoch, None, shifted).unwrap();
            let t = SimTime::from_seconds(5.0);
            let delta = shifted_ctx.earth_rotation_angle(t) - base_ctx.earth_rotation_angle(t);
            assert_abs_diff_eq!(
                delta,
                EARTH_ROTATION_ANGLE_RATE_RAD_S * 0.5,
                epsilon = 2.0e-9
            );
        }

        #[test]
        fn iers_position_round_trip_with_polar_motion() {
            let ctx = FrameContext::iers_tabulated(2_451_545.0, None, table()).unwrap();
            let t = SimTime::from_seconds(5.0);
            let p_eci: Position3<Eci> = Position3::new(7_000_000.0, 1_500_000.0, 800_000.0);
            let p_ecef = ctx.eci_to_ecef_position(t, p_eci);
            let p_back = ctx.ecef_to_eci_position(t, p_ecef);
            assert_abs_diff_eq!(p_back.vector.x, p_eci.vector.x, epsilon = 1.0e-6);
            assert_abs_diff_eq!(p_back.vector.y, p_eci.vector.y, epsilon = 1.0e-6);
            assert_abs_diff_eq!(p_back.vector.z, p_eci.vector.z, epsilon = 1.0e-6);
        }

        #[test]
        fn iers_precession_is_identity_at_j2000() {
            let v = Vector3::new(1.0, 2.0, 3.0);
            let precessed = precess_j2000_to_mean_of_date_vector(2_451_545.0, v);
            assert_abs_diff_eq!(precessed.x, v.x, epsilon = 1.0e-15);
            assert_abs_diff_eq!(precessed.y, v.y, epsilon = 1.0e-15);
            assert_abs_diff_eq!(precessed.z, v.z, epsilon = 1.0e-15);
        }

        #[test]
        fn iers_precession_moves_axes_over_decades() {
            let jd_2050 = 2_451_545.0 + 0.5 * 36_525.0;
            let x_axis = Vector3::new(1.0, 0.0, 0.0);
            let precessed = precess_j2000_to_mean_of_date_vector(jd_2050, x_axis);
            let (zeta, theta, z) = precession_angles_iau1976(jd_2050);
            let (sin_zeta, cos_zeta) = zeta.sin_cos();
            let (sin_theta, cos_theta) = theta.sin_cos();
            let (sin_z, cos_z) = z.sin_cos();
            let expected = Vector3::new(
                cos_z * cos_theta * cos_zeta - sin_z * sin_zeta,
                sin_z * cos_theta * cos_zeta + cos_z * sin_zeta,
                sin_theta * cos_zeta,
            );
            assert_abs_diff_eq!(precessed.norm(), 1.0, epsilon = 1.0e-15);
            assert_abs_diff_eq!(precessed.x, expected.x, epsilon = 1.0e-15);
            assert_abs_diff_eq!(precessed.y, expected.y, epsilon = 1.0e-15);
            assert_abs_diff_eq!(precessed.z, expected.z, epsilon = 1.0e-15);
            assert!(
                precessed.y > 0.0,
                "x-axis should precess toward positive RA"
            );
            assert!(
                precessed.z > 0.0,
                "x-axis should precess toward positive mean-date declination"
            );
            assert!(
                (precessed - x_axis).norm() > 0.005,
                "precession over five decades should be arcminute-scale: {precessed:?}"
            );
            let restored = precess_mean_of_date_to_j2000_vector(jd_2050, precessed);
            assert_abs_diff_eq!(restored.x, x_axis.x, epsilon = 1.0e-14);
            assert_abs_diff_eq!(restored.y, x_axis.y, epsilon = 1.0e-14);
            assert_abs_diff_eq!(restored.z, x_axis.z, epsilon = 1.0e-14);
        }

        #[test]
        fn iers_nutation_angles_match_erfa_reference() {
            let jd = 2_400_000.5 + 53_736.0;
            let (dpsi, deps) = nutation_angles_iau1980(jd);
            assert_abs_diff_eq!(dpsi, -0.9643658353226563966e-5, epsilon = 1.0e-17);
            assert_abs_diff_eq!(deps, 0.4060051006879713322e-4, epsilon = 1.0e-17);
        }

        #[test]
        fn iers_nutation_matrix_matches_erfa_reference() {
            let jd = 2_400_000.5 + 53_736.0;
            let x = nutate_mean_of_date_to_true_of_date_vector(jd, Vector3::new(1.0, 0.0, 0.0));
            let y = nutate_mean_of_date_to_true_of_date_vector(jd, Vector3::new(0.0, 1.0, 0.0));
            let z = nutate_mean_of_date_to_true_of_date_vector(jd, Vector3::new(0.0, 0.0, 1.0));
            assert_abs_diff_eq!(x.x, 0.9999999999534999268, epsilon = 1.0e-14);
            assert_abs_diff_eq!(x.y, -0.8847780042583435924e-5, epsilon = 1.0e-14);
            assert_abs_diff_eq!(x.z, -0.3836265729708478796e-5, epsilon = 1.0e-14);
            assert_abs_diff_eq!(y.x, 0.8847935789636432161e-5, epsilon = 1.0e-14);
            assert_abs_diff_eq!(y.y, 0.9999999991366569963, epsilon = 1.0e-14);
            assert_abs_diff_eq!(y.z, 0.4060049308612638555e-4, epsilon = 1.0e-14);
            assert_abs_diff_eq!(z.x, 0.3835906502164019142e-5, epsilon = 1.0e-14);
            assert_abs_diff_eq!(z.y, -0.4060052702727130809e-4, epsilon = 1.0e-14);
            assert_abs_diff_eq!(z.z, 0.9999999991684415129, epsilon = 1.0e-14);
        }

        #[test]
        fn iers_precession_nutation_matrix_matches_erfa_reference() {
            let jd = 2_400_000.5 + 50_123.999_9;
            let x = precess_j2000_to_true_of_date_vector(jd, Vector3::new(1.0, 0.0, 0.0));
            let y = precess_j2000_to_true_of_date_vector(jd, Vector3::new(0.0, 1.0, 0.0));
            let z = precess_j2000_to_true_of_date_vector(jd, Vector3::new(0.0, 0.0, 1.0));
            assert_abs_diff_eq!(x.x, 0.9999995831934611169, epsilon = 1.0e-13);
            assert_abs_diff_eq!(x.y, -0.8373804896118301316e-3, epsilon = 1.0e-13);
            assert_abs_diff_eq!(x.z, -0.3638774789072144473e-3, epsilon = 1.0e-13);
            assert_abs_diff_eq!(y.x, 0.8373654045728124011e-3, epsilon = 1.0e-13);
            assert_abs_diff_eq!(y.y, 0.9999996485439674092, epsilon = 1.0e-13);
            assert_abs_diff_eq!(y.z, -0.4160674085851722359e-4, epsilon = 1.0e-13);
            assert_abs_diff_eq!(z.x, 0.3639121916933106191e-3, epsilon = 1.0e-13);
            assert_abs_diff_eq!(z.y, 0.4130202510421549752e-4, epsilon = 1.0e-13);
            assert_abs_diff_eq!(z.z, 0.9999999329310274805, epsilon = 1.0e-13);
        }

        #[test]
        fn iers_velocity_round_trip_with_precession_and_nutation() {
            let epoch_2050 = 2_451_545.0 + 0.5 * 36_525.0;
            let ctx = FrameContext::iers_tabulated(epoch_2050, None, table()).unwrap();
            let t = SimTime::from_seconds(5.0);
            let p_eci: Position3<Eci> = Position3::new(7_000_000.0, 1_500_000.0, 800_000.0);
            let v_eci: Velocity3<Eci> = Velocity3::new(120.0, 7_400.0, -25.0);
            let p_ecef = ctx.eci_to_ecef_position(t, p_eci);
            let v_ecef = ctx.eci_to_ecef_velocity(t, v_eci, p_eci);
            let v_back = ctx.ecef_to_eci_velocity(t, v_ecef, p_ecef);
            assert_abs_diff_eq!(v_back.vector.x, v_eci.vector.x, epsilon = 1.0e-9);
            assert_abs_diff_eq!(v_back.vector.y, v_eci.vector.y, epsilon = 1.0e-9);
            assert_abs_diff_eq!(v_back.vector.z, v_eci.vector.z, epsilon = 1.0e-9);
        }

        #[test]
        fn iers_velocity_includes_celestial_frame_rate() {
            let constant_eop = EarthOrientationTable::new(vec![
                EarthOrientationSample::new(0.0, 0.0, 0.0, 0.0).unwrap(),
                EarthOrientationSample::new(10.0, 0.0, 0.0, 0.0).unwrap(),
            ])
            .unwrap();
            let epoch_2050 = 2_451_545.0 + 0.5 * 36_525.0;
            let ctx = FrameContext::iers_tabulated(epoch_2050, None, constant_eop).unwrap();
            let t = SimTime::from_seconds(5.0);
            let p_eci: Position3<Eci> = Position3::new(42_000_000.0, -7_000_000.0, 3_000_000.0);
            let transported = ctx.eci_to_ecef_velocity(t, Velocity3::new(0.0, 0.0, 0.0), p_eci);

            let before = ctx
                .eci_to_ecef_position(SimTime::from_seconds(4.0), p_eci)
                .vector;
            let after = ctx
                .eci_to_ecef_position(SimTime::from_seconds(6.0), p_eci)
                .vector;
            let numerical_derivative = (after - before) * 0.5;
            assert_abs_diff_eq!(
                transported.vector.x,
                numerical_derivative.x,
                epsilon = 1.0e-9
            );
            assert_abs_diff_eq!(
                transported.vector.y,
                numerical_derivative.y,
                epsilon = 2.0e-9
            );
            assert_abs_diff_eq!(
                transported.vector.z,
                numerical_derivative.z,
                epsilon = 1.0e-9
            );

            let theta = ctx.earth_rotation_angle(t);
            let r_true_of_date = ctx.eci_to_true_of_date_vector(t, p_eci.vector);
            let earth_spin_transport = Vector3::new(
                -EARTH_ROTATION_ANGLE_RATE_RAD_S * r_true_of_date.y,
                EARTH_ROTATION_ANGLE_RATE_RAD_S * r_true_of_date.x,
                0.0,
            );
            let earth_spin_only =
                ctx.tirs_to_ecef_vector(t, rotate_z(-earth_spin_transport, -theta));
            assert!(
                (transported.vector - earth_spin_only).norm() > 1.0e-5,
                "precession/nutation frame rate should be measurable at GEO scale"
            );
        }

        #[test]
        fn iers_table_reports_interval_coverage() {
            let table = table();
            assert!(table.covers_interval(0.0, 10.0));
            assert!(!table.covers_interval(-1.0, 10.0));
            assert!(!table.covers_interval(0.0, 11.0));
        }
    }
}
