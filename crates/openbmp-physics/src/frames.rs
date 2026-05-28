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
use openbmp_core::{Ecef, Eci, Frame as CoreFrame, FrameError, Ned, Position3, SimTime, Velocity3};

// ---------------------------------------------------------------------
// FrameProfile
// ---------------------------------------------------------------------

/// Active frame profile.
///
/// The profile determines which [`FrameTransform`] implementations and
/// time-aware methods are available. Provides
/// [`FrameProfile::ToyFixedEarth`],
/// [`FrameProfile::Wgs84UniformRotation`], and
/// [`FrameProfile::IersTabulated`].
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
}

impl FrameProfile {
    /// Canonical profile name as it appears in scenario files.
    #[must_use]
    pub const fn as_label(self) -> &'static str {
        match self {
            Self::ToyFixedEarth => "toy-fixed-earth",
            Self::Wgs84UniformRotation => "wgs84-uniform-rotation",
            Self::IersTabulated => "iers-tabulated",
        }
    }
}

// ---------------------------------------------------------------------
// Earth orientation
// ---------------------------------------------------------------------

const SECONDS_PER_DAY: f64 = 86_400.0;
const J2000_JULIAN_DATE: f64 = 2_451_545.0;
const JULIAN_CENTURY_DAYS: f64 = 36_525.0;
const TWO_PI: f64 = 2.0 * std::f64::consts::PI;
const TENTH_MILLIARCSECOND_TO_RAD: f64 = ARCSECOND_TO_RAD / 10_000.0;
const FRAME_RATE_STEP_S: f64 = 1.0;

/// Radians per arcsecond.
pub const ARCSECOND_TO_RAD: f64 = std::f64::consts::PI / (180.0 * 3_600.0);

/// IAU Earth Rotation Angle rate, rad/s.
///
/// This is the ERA slope with respect to UT1:
/// `2π * 1.00273781191135448 / 86400`.
pub const EARTH_ROTATION_ANGLE_RATE_RAD_S: f64 = TWO_PI * 1.002_737_811_911_354_6 / SECONDS_PER_DAY;

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
            lod_s: None,
        })
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
            || self.lod_s.is_some_and(|lod_s| !lod_s.is_finite())
        {
            return Err(FrameError::InvalidFrameProfileData {
                reason: "Earth-orientation samples must be finite",
            });
        }
        Ok(())
    }
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
        if !latitude_rad.is_finite() || latitude_rad.abs() > std::f64::consts::FRAC_PI_2 {
            return Err(FrameError::InvalidGeodeticCoordinate {
                reason: "latitude must be finite and in [-π/2, π/2] rad",
            });
        }
        if !longitude_rad.is_finite() || longitude_rad.abs() > std::f64::consts::PI {
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
}

impl FrameContext {
    /// Construct a context for [`FrameProfile::ToyFixedEarth`].
    #[must_use]
    pub const fn toy_fixed_earth() -> Self {
        Self {
            profile: FrameProfile::ToyFixedEarth,
            local_origin: None,
            iers: None,
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

    /// Pinned Earth-orientation table used by
    /// [`FrameProfile::IersTabulated`], if any.
    #[must_use]
    pub fn earth_orientation_table(&self) -> Option<&EarthOrientationTable> {
        self.iers.as_ref().map(|iers| &iers.earth_orientation)
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
        if self.profile == FrameProfile::IersTabulated {
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
        if self.profile == FrameProfile::IersTabulated {
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
            FrameProfile::IersTabulated => EARTH_ROTATION_ANGLE_RATE_RAD_S,
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
            FrameProfile::IersTabulated => {
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
                lod_s: None,
            },
            |iers| iers.earth_orientation.sample(t),
        )
    }

    fn eci_to_ecef_vector(&self, t: SimTime, v_eci: Vector3<f64>) -> Vector3<f64> {
        let theta = self.earth_rotation_angle(t);
        let true_of_date = self.eci_to_true_of_date_vector(t, v_eci);
        let tirs = rotate_z(true_of_date, -theta);
        self.tirs_to_ecef_vector(t, tirs)
    }

    fn ecef_to_eci_vector(&self, t: SimTime, v_ecef: Vector3<f64>) -> Vector3<f64> {
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
    let days_since_j2000 = jd_ut1 - J2000_JULIAN_DATE;
    (TWO_PI * (0.779_057_273_264_0 + 1.002_737_811_911_354_6 * days_since_j2000)).rem_euclid(TWO_PI)
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
    let mut normalized = theta.rem_euclid(TWO_PI);
    if normalized >= std::f64::consts::PI {
        normalized -= TWO_PI;
    }
    normalized
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
#[allow(clippy::expect_used, clippy::unwrap_used)]
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
            let jd = 2_400_000.5 + 50_123.9999;
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
                epsilon = 1.0e-9
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
