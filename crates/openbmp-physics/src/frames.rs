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

use openbmp_core::{Ecef, Eci, Frame as CoreFrame, FrameError, Ned, Position3, SimTime, Velocity3};

// ---------------------------------------------------------------------
// FrameProfile
// ---------------------------------------------------------------------

/// Active frame profile.
///
/// The profile determines which [`FrameTransform`] implementations and
/// time-aware methods are available. Provides
/// [`FrameProfile::ToyFixedEarth`] and
/// [`FrameProfile::Wgs84UniformRotation`].
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
    /// motion, no leap seconds. IERS-tabulated and SPICE-reference
    /// profiles are out of scope.
    Wgs84UniformRotation,
}

impl FrameProfile {
    /// Canonical profile name as it appears in scenario files.
    #[must_use]
    pub const fn as_label(self) -> &'static str {
        match self {
            Self::ToyFixedEarth => "toy-fixed-earth",
            Self::Wgs84UniformRotation => "wgs84-uniform-rotation",
        }
    }
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
}

impl FrameContext {
    /// Construct a context for [`FrameProfile::ToyFixedEarth`].
    #[must_use]
    pub const fn toy_fixed_earth() -> Self {
        Self {
            profile: FrameProfile::ToyFixedEarth,
            local_origin: None,
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
        }
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
        }
    }

    /// Transform an ECI position to ECEF at simulation time `t`.
    ///
    /// `R_z(-θ) · p_eci`. `ToyFixedEarth` profile always returns
    /// `p_eci` reinterpreted as ECEF.
    #[must_use]
    pub fn eci_to_ecef_position(&self, t: SimTime, p_eci: Position3<Eci>) -> Position3<Ecef> {
        let theta = self.earth_rotation_angle(t);
        let (s, c) = (theta.sin(), theta.cos());
        // Rotate by -θ about z: x' = c x + s y; y' = -s x + c y; z' = z.
        let v = p_eci.vector;
        Position3::new(c * v.x + s * v.y, -s * v.x + c * v.y, v.z)
    }

    /// Transform an ECEF position to ECI at simulation time `t`.
    /// Inverse of [`Self::eci_to_ecef_position`].
    #[must_use]
    pub fn ecef_to_eci_position(&self, t: SimTime, p_ecef: Position3<Ecef>) -> Position3<Eci> {
        let theta = self.earth_rotation_angle(t);
        let (s, c) = (theta.sin(), theta.cos());
        // Rotate by +θ about z: x' = c x - s y; y' = s x + c y; z' = z.
        let v = p_ecef.vector;
        Position3::new(c * v.x - s * v.y, s * v.x + c * v.y, v.z)
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
        let omega = self.angular_velocity_z();
        let r = p_eci.vector;
        // ω × r where ω = (0, 0, ω_e):
        // (ω × r)_x = -ω_e · y; (ω × r)_y = +ω_e · x; (ω × r)_z = 0.
        let cross = nalgebra::Vector3::new(-omega * r.y, omega * r.x, 0.0);
        let v_inertial_minus_transport = v_eci.vector - cross;
        let theta = self.earth_rotation_angle(t);
        let (s, c) = (theta.sin(), theta.cos());
        let v = v_inertial_minus_transport;
        Velocity3::new(c * v.x + s * v.y, -s * v.x + c * v.y, v.z)
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
        // Rotate the ECEF velocity into ECI orientation:
        let theta = self.earth_rotation_angle(t);
        let (s, c) = (theta.sin(), theta.cos());
        let v = v_ecef.vector;
        let v_rotated = nalgebra::Vector3::new(c * v.x - s * v.y, s * v.x + c * v.y, v.z);
        // Add the transport rate ω × r_eci, where r_eci is the ECI
        // position derived from p_ecef. We compute it inline.
        let r = p_ecef.vector;
        let r_eci = nalgebra::Vector3::new(c * r.x - s * r.y, s * r.x + c * r.y, r.z);
        let omega = self.angular_velocity_z();
        let cross = nalgebra::Vector3::new(-omega * r_eci.y, omega * r_eci.x, 0.0);
        Velocity3::from_vector(v_rotated + cross)
    }

    /// Earth angular velocity along the inertial `+z` axis (rad/s).
    /// `0.0` for `ToyFixedEarth`, `WGS84_OMEGA_RAD_S` for
    /// `Wgs84UniformRotation`.
    #[must_use]
    pub const fn angular_velocity_z(&self) -> f64 {
        match self.profile {
            FrameProfile::ToyFixedEarth => 0.0,
            FrameProfile::Wgs84UniformRotation => WGS84_OMEGA_RAD_S,
        }
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
}
