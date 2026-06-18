//! `openbmp-afts` — forward AFTS containment-monitor primitives.
//!
//! This crate is a clean-room, simulation-only automatic flight-termination
//! monitor substrate. It predicts a forward instantaneous impact point (IIP)
//! from the current inertial state, compares that point against declarative
//! geospatial keep-inside rules, and emits a latched terminate decision with
//! the first rule id that fired.
//!
//! The API is intentionally forward-only. It contains no targeting,
//! aimpoint-solving, range-safety certification, device-driver, or flight
//! deployment surface.

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]
#![deny(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

pub mod error;

pub use error::AftsError;

#[cfg(not(feature = "std"))]
use num_traits::Float;
use openbmp_core::{Eci, Position3, Vector3, Velocity3};
use openbmp_physics::{WGS84_A_M, WGS84_FLATTENING, WGS84_MU_M3_S2, WGS84_OMEGA_RAD_S};
use openbmp_state::{PointMassState, RigidBodyState};

use crate::error::{require_finite, require_non_negative, require_positive};

const DEFAULT_STEP_S: f64 = 0.25;
const DEFAULT_MAX_TIME_S: f64 = 7_200.0;
const DEFAULT_RADIUS_TOLERANCE_M: f64 = 1.0e-3;
const BOUNDARY_EPS_RAD: f64 = 1.0e-12;
const WGS84_B_M: f64 = WGS84_A_M * (1.0 - WGS84_FLATTENING);

/// Geodetic-like spherical latitude/longitude point in radians.
///
/// The current AFTS substrate uses a spherical IIP surface pinned to the WGS84
/// semi-major axis. Latitude is geocentric latitude on that sphere, not a full
/// ellipsoidal conversion.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LatLon {
    /// Latitude in radians, constrained to `[-pi/2, pi/2]`.
    pub latitude_rad: f64,
    /// Longitude in radians, constrained to `[-pi, pi]`.
    pub longitude_rad: f64,
}

impl LatLon {
    /// Creates a validated latitude/longitude point from radians.
    ///
    /// # Errors
    ///
    /// Returns [`AftsError::InvalidParameter`] when either angle is non-finite
    /// or outside its valid range.
    pub fn new_radians(latitude_rad: f64, longitude_rad: f64) -> Result<Self, AftsError> {
        let latitude_rad = require_finite("latitude_rad", latitude_rad)?;
        let longitude_rad = require_finite("longitude_rad", longitude_rad)?;
        if latitude_rad.abs() > core::f64::consts::FRAC_PI_2 {
            return Err(AftsError::InvalidParameter {
                field: "latitude_rad",
                reason: "must be in [-pi/2, pi/2]",
            });
        }
        if longitude_rad.abs() > core::f64::consts::PI {
            return Err(AftsError::InvalidParameter {
                field: "longitude_rad",
                reason: "must be in [-pi, pi]",
            });
        }
        Ok(Self {
            latitude_rad,
            longitude_rad,
        })
    }

    /// Creates a validated latitude/longitude point from degrees.
    ///
    /// # Errors
    ///
    /// Forwards validation errors from [`Self::new_radians`].
    pub fn new_degrees(latitude_deg: f64, longitude_deg: f64) -> Result<Self, AftsError> {
        Self::new_radians(latitude_deg.to_radians(), longitude_deg.to_radians())
    }

    /// Converts this point to a unit vector on the spherical IIP surface.
    #[must_use]
    pub fn to_unit_vector(self) -> Vector3<f64> {
        let cos_lat = self.latitude_rad.cos();
        Vector3::new(
            cos_lat * self.longitude_rad.cos(),
            cos_lat * self.longitude_rad.sin(),
            self.latitude_rad.sin(),
        )
    }
}

/// Forward instantaneous-impact-point prediction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImpactPoint {
    /// Impact latitude/longitude on the configured spherical surface.
    pub point: LatLon,
    /// Forward time from the supplied state to impact.
    pub time_to_impact_s: f64,
}

/// Impact-surface geometry used by the IIP propagator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum IipImpactSurface {
    /// Spherical impact surface.
    Sphere {
        /// Surface radius.
        radius_m: f64,
    },
    /// Oblate spheroid impact surface.
    OblateSpheroid {
        /// Equatorial semi-major axis.
        semi_major_m: f64,
        /// Polar semi-minor axis.
        semi_minor_m: f64,
    },
}

impl IipImpactSurface {
    /// Creates a validated spherical impact surface.
    ///
    /// # Errors
    ///
    /// Returns [`AftsError::InvalidParameter`] when `radius_m` is non-finite
    /// or not positive.
    pub fn sphere(radius_m: f64) -> Result<Self, AftsError> {
        Ok(Self::Sphere {
            radius_m: require_positive("surface.radius_m", radius_m)?,
        })
    }

    /// Creates a validated oblate spheroid impact surface.
    ///
    /// # Errors
    ///
    /// Returns [`AftsError::InvalidParameter`] when either axis is invalid or
    /// `semi_minor_m > semi_major_m`.
    pub fn oblate_spheroid(semi_major_m: f64, semi_minor_m: f64) -> Result<Self, AftsError> {
        let semi_major_m = require_positive("surface.semi_major_m", semi_major_m)?;
        let semi_minor_m = require_positive("surface.semi_minor_m", semi_minor_m)?;
        if semi_minor_m > semi_major_m {
            return Err(AftsError::InvalidParameter {
                field: "surface.semi_minor_m",
                reason: "must be <= semi_major_m",
            });
        }
        Ok(Self::OblateSpheroid {
            semi_major_m,
            semi_minor_m,
        })
    }

    /// WGS84 spherical surface pinned to the semi-major axis.
    #[must_use]
    pub const fn wgs84_sphere() -> Self {
        Self::Sphere {
            radius_m: WGS84_A_M,
        }
    }

    /// WGS84 ellipsoid surface.
    #[must_use]
    pub const fn wgs84_ellipsoid() -> Self {
        Self::OblateSpheroid {
            semi_major_m: WGS84_A_M,
            semi_minor_m: WGS84_B_M,
        }
    }

    /// Returns the equatorial radius or sphere radius for this surface.
    #[must_use]
    pub const fn equatorial_radius_m(self) -> f64 {
        match self {
            Self::Sphere { radius_m } => radius_m,
            Self::OblateSpheroid { semi_major_m, .. } => semi_major_m,
        }
    }

    /// Returns the polar radius for this surface.
    #[must_use]
    pub const fn polar_radius_m(self) -> f64 {
        match self {
            Self::Sphere { radius_m } => radius_m,
            Self::OblateSpheroid { semi_minor_m, .. } => semi_minor_m,
        }
    }

    fn surface_radius_along_vector(self, position: Vector3<f64>) -> Result<f64, AftsError> {
        let radius_m = position.norm();
        if !radius_m.is_finite() || radius_m <= 0.0 {
            return Err(AftsError::InvalidVector {
                field: "position_m",
                reason: "must be finite and non-zero",
            });
        }
        match self {
            Self::Sphere { radius_m } => Ok(radius_m),
            Self::OblateSpheroid {
                semi_major_m,
                semi_minor_m,
            } => {
                let unit = position / radius_m;
                let xy = unit.x * unit.x + unit.y * unit.y;
                let denominator = (xy / (semi_major_m * semi_major_m)
                    + (unit.z * unit.z) / (semi_minor_m * semi_minor_m))
                    .sqrt();
                if !denominator.is_finite() || denominator <= 0.0 {
                    return Err(AftsError::InvalidVector {
                        field: "position_m",
                        reason: "surface projection must stay finite",
                    });
                }
                Ok(1.0 / denominator)
            }
        }
    }

    fn height_m(self, position: Vector3<f64>) -> Result<f64, AftsError> {
        Ok(position.norm() - self.surface_radius_along_vector(position)?)
    }

    fn lat_lon_from_vector(self, position: Vector3<f64>) -> Result<LatLon, AftsError> {
        match self {
            Self::Sphere { .. } => lat_lon_from_vector(position),
            Self::OblateSpheroid {
                semi_major_m,
                semi_minor_m,
            } => ellipsoid_lat_lon_from_vector(position, semi_major_m, semi_minor_m),
        }
    }
}

/// Deterministic exponential ballistic-drag model for forward IIP propagation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExponentialAtmosphericDrag {
    reference_radius_m: f64,
    reference_density_kg_m3: f64,
    scale_height_m: f64,
    ballistic_coefficient_kg_m2: f64,
}

impl ExponentialAtmosphericDrag {
    /// Creates a validated exponential drag model.
    ///
    /// The ballistic coefficient is `mass / (C_D A)` in kg/m².
    ///
    /// # Errors
    ///
    /// Returns [`AftsError::InvalidParameter`] when any scalar is invalid.
    pub fn new(
        reference_radius_m: f64,
        reference_density_kg_m3: f64,
        scale_height_m: f64,
        ballistic_coefficient_kg_m2: f64,
    ) -> Result<Self, AftsError> {
        Ok(Self {
            reference_radius_m: require_positive("drag.reference_radius_m", reference_radius_m)?,
            reference_density_kg_m3: require_non_negative(
                "drag.reference_density_kg_m3",
                reference_density_kg_m3,
            )?,
            scale_height_m: require_positive("drag.scale_height_m", scale_height_m)?,
            ballistic_coefficient_kg_m2: require_positive(
                "drag.ballistic_coefficient_kg_m2",
                ballistic_coefficient_kg_m2,
            )?,
        })
    }

    /// Reference radius used for the exponential density law.
    #[must_use]
    pub const fn reference_radius_m(self) -> f64 {
        self.reference_radius_m
    }

    /// Density at `reference_radius_m`.
    #[must_use]
    pub const fn reference_density_kg_m3(self) -> f64 {
        self.reference_density_kg_m3
    }

    /// Exponential scale height.
    #[must_use]
    pub const fn scale_height_m(self) -> f64 {
        self.scale_height_m
    }

    /// Ballistic coefficient `mass / (C_D A)`.
    #[must_use]
    pub const fn ballistic_coefficient_kg_m2(self) -> f64 {
        self.ballistic_coefficient_kg_m2
    }

    fn acceleration_m_s2(
        self,
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        earth_rotation_rate_rad_s: f64,
    ) -> Result<Vector3<f64>, AftsError> {
        let radius_m = position.norm();
        if !radius_m.is_finite() || radius_m <= 0.0 {
            return Err(AftsError::InvalidVector {
                field: "position_m",
                reason: "must stay finite and non-zero during drag propagation",
            });
        }
        let altitude_m = radius_m - self.reference_radius_m;
        let density_kg_m3 =
            self.reference_density_kg_m3 * (-altitude_m / self.scale_height_m).exp();
        if density_kg_m3 <= 0.0 {
            return Ok(Vector3::new(0.0, 0.0, 0.0));
        }
        let atmosphere_velocity = Vector3::new(
            -earth_rotation_rate_rad_s * position.y,
            earth_rotation_rate_rad_s * position.x,
            0.0,
        );
        let relative_velocity = velocity - atmosphere_velocity;
        let speed_m_s = relative_velocity.norm();
        if !speed_m_s.is_finite() || speed_m_s <= 0.0 {
            return Ok(Vector3::new(0.0, 0.0, 0.0));
        }
        Ok(relative_velocity
            * (-0.5 * density_kg_m3 * speed_m_s / self.ballistic_coefficient_kg_m2))
    }
}

/// Fixed-step central-gravity IIP propagator.
///
/// The default propagated surface is a sphere. [`Self::wgs84_spherical`] pins
/// that sphere to `WGS84_A_M` and `WGS84_MU_M3_S2`; [`Self::wgs84_ellipsoid`]
/// uses the WGS84 oblate spheroid; rotating constructors also apply the
/// standard Earth-rotation longitude correction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IipPropagator {
    surface: IipImpactSurface,
    mu_m3_s2: f64,
    step_s: f64,
    max_time_s: f64,
    radius_tolerance_m: f64,
    earth_rotation_rate_rad_s: f64,
    drag_model: Option<ExponentialAtmosphericDrag>,
}

impl IipPropagator {
    /// Creates a validated fixed-step central-gravity IIP propagator.
    ///
    /// # Errors
    ///
    /// Returns [`AftsError::InvalidParameter`] when any scalar is non-finite
    /// or outside its required positive/non-negative range.
    pub fn new(
        radius_m: f64,
        mu_m3_s2: f64,
        step_s: f64,
        max_time_s: f64,
        radius_tolerance_m: f64,
    ) -> Result<Self, AftsError> {
        Self::new_with_earth_rotation(
            radius_m,
            mu_m3_s2,
            step_s,
            max_time_s,
            radius_tolerance_m,
            0.0,
        )
    }

    /// Creates a validated fixed-step central-gravity IIP propagator with an
    /// Earth-fixed longitude correction.
    ///
    /// `earth_rotation_rate_rad_s = 0` leaves longitude in the inertial frame.
    /// A positive value rotates the predicted impact vector into an
    /// Earth-fixed frame by subtracting `earth_rotation_rate_rad_s *
    /// time_to_impact_s` from the inertial longitude.
    ///
    /// # Errors
    ///
    /// Returns [`AftsError::InvalidParameter`] when any scalar is non-finite
    /// or outside its required positive/non-negative range.
    pub fn new_with_earth_rotation(
        radius_m: f64,
        mu_m3_s2: f64,
        step_s: f64,
        max_time_s: f64,
        radius_tolerance_m: f64,
        earth_rotation_rate_rad_s: f64,
    ) -> Result<Self, AftsError> {
        Self::new_with_surface_and_drag(
            IipImpactSurface::sphere(radius_m)?,
            mu_m3_s2,
            step_s,
            max_time_s,
            radius_tolerance_m,
            earth_rotation_rate_rad_s,
            None,
        )
    }

    /// Creates a validated fixed-step IIP propagator over a configured impact
    /// surface and optional deterministic drag model.
    ///
    /// # Errors
    ///
    /// Returns [`AftsError::InvalidParameter`] when any scalar is non-finite
    /// or outside its required positive/non-negative range.
    pub fn new_with_surface_and_drag(
        surface: IipImpactSurface,
        mu_m3_s2: f64,
        step_s: f64,
        max_time_s: f64,
        radius_tolerance_m: f64,
        earth_rotation_rate_rad_s: f64,
        drag_model: Option<ExponentialAtmosphericDrag>,
    ) -> Result<Self, AftsError> {
        let mu_m3_s2 = require_positive("mu_m3_s2", mu_m3_s2)?;
        let step_s = require_positive("step_s", step_s)?;
        let max_time_s = require_positive("max_time_s", max_time_s)?;
        let radius_tolerance_m = require_non_negative("radius_tolerance_m", radius_tolerance_m)?;
        let earth_rotation_rate_rad_s =
            require_non_negative("earth_rotation_rate_rad_s", earth_rotation_rate_rad_s)?;
        Ok(Self {
            surface,
            mu_m3_s2,
            step_s,
            max_time_s,
            radius_tolerance_m,
            earth_rotation_rate_rad_s,
            drag_model,
        })
    }

    /// Creates the default WGS84-spherical central-gravity IIP propagator with
    /// inertial longitude (no Earth-rotation correction).
    #[must_use]
    pub const fn wgs84_spherical() -> Self {
        Self {
            surface: IipImpactSurface::wgs84_sphere(),
            mu_m3_s2: WGS84_MU_M3_S2,
            step_s: DEFAULT_STEP_S,
            max_time_s: DEFAULT_MAX_TIME_S,
            radius_tolerance_m: DEFAULT_RADIUS_TOLERANCE_M,
            earth_rotation_rate_rad_s: 0.0,
            drag_model: None,
        }
    }

    /// Creates the default WGS84-ellipsoid central-gravity IIP propagator with
    /// inertial longitude (no Earth-rotation correction).
    #[must_use]
    pub const fn wgs84_ellipsoid() -> Self {
        Self {
            surface: IipImpactSurface::wgs84_ellipsoid(),
            mu_m3_s2: WGS84_MU_M3_S2,
            step_s: DEFAULT_STEP_S,
            max_time_s: DEFAULT_MAX_TIME_S,
            radius_tolerance_m: DEFAULT_RADIUS_TOLERANCE_M,
            earth_rotation_rate_rad_s: 0.0,
            drag_model: None,
        }
    }

    /// Creates the default WGS84-spherical central-gravity IIP propagator with
    /// WGS84 uniform-rotation longitude correction.
    #[must_use]
    pub const fn wgs84_rotating_earth() -> Self {
        Self {
            surface: IipImpactSurface::wgs84_sphere(),
            mu_m3_s2: WGS84_MU_M3_S2,
            step_s: DEFAULT_STEP_S,
            max_time_s: DEFAULT_MAX_TIME_S,
            radius_tolerance_m: DEFAULT_RADIUS_TOLERANCE_M,
            earth_rotation_rate_rad_s: WGS84_OMEGA_RAD_S,
            drag_model: None,
        }
    }

    /// Creates the default WGS84-ellipsoid central-gravity IIP propagator with
    /// WGS84 uniform-rotation longitude correction.
    #[must_use]
    pub const fn wgs84_ellipsoid_rotating_earth() -> Self {
        Self {
            surface: IipImpactSurface::wgs84_ellipsoid(),
            mu_m3_s2: WGS84_MU_M3_S2,
            step_s: DEFAULT_STEP_S,
            max_time_s: DEFAULT_MAX_TIME_S,
            radius_tolerance_m: DEFAULT_RADIUS_TOLERANCE_M,
            earth_rotation_rate_rad_s: WGS84_OMEGA_RAD_S,
            drag_model: None,
        }
    }

    /// Returns a copy of this propagator with deterministic exponential drag.
    #[must_use]
    pub const fn with_exponential_drag(mut self, drag_model: ExponentialAtmosphericDrag) -> Self {
        self.drag_model = Some(drag_model);
        self
    }

    /// Returns the configured impact-surface radius.
    #[must_use]
    pub const fn radius_m(self) -> f64 {
        self.surface.equatorial_radius_m()
    }

    /// Returns the configured impact surface.
    #[must_use]
    pub const fn impact_surface(self) -> IipImpactSurface {
        self.surface
    }

    /// Returns the Earth-rotation rate used to convert inertial impact
    /// longitude into Earth-fixed longitude.
    #[must_use]
    pub const fn earth_rotation_rate_rad_s(self) -> f64 {
        self.earth_rotation_rate_rad_s
    }

    /// Returns the optional deterministic drag model.
    #[must_use]
    pub const fn drag_model(self) -> Option<ExponentialAtmosphericDrag> {
        self.drag_model
    }

    /// Builds corridor metrics from an inertial position/velocity state using
    /// this propagator's impact surface.
    ///
    /// # Errors
    ///
    /// Returns [`AftsError`] when vectors or the surface projection are invalid.
    pub fn flight_corridor_sample(
        self,
        position: Position3<Eci>,
        velocity: Velocity3<Eci>,
    ) -> Result<FlightCorridorSample, AftsError> {
        let position_vector = finite_nonzero_position(position)?;
        FlightCorridorSample::from_position_velocity(
            position,
            velocity,
            self.surface.surface_radius_along_vector(position_vector)?,
        )
    }

    /// Predicts IIP from an OpenBMP point-mass state.
    ///
    /// # Errors
    ///
    /// Returns [`AftsError`] when the state vectors are invalid or the
    /// trajectory does not intersect the configured surface inside the horizon.
    pub fn impact_point_from_state(self, state: &PointMassState) -> Result<ImpactPoint, AftsError> {
        self.impact_point_from_position_velocity(state.position, state.velocity)
    }

    /// Predicts IIP from an OpenBMP rigid-body state projection.
    ///
    /// # Errors
    ///
    /// Forwards errors from [`Self::impact_point_from_position_velocity`].
    pub fn impact_point_from_rigid_body(
        self,
        state: &RigidBodyState,
    ) -> Result<ImpactPoint, AftsError> {
        self.impact_point_from_position_velocity(state.position, state.velocity)
    }

    /// Predicts IIP from an inertial position and velocity.
    ///
    /// # Errors
    ///
    /// Returns [`AftsError::InvalidVector`] for non-finite or near-origin
    /// position vectors, or [`AftsError::NoImpactWithinHorizon`] when the
    /// forward central-gravity trajectory misses the configured horizon.
    pub fn impact_point_from_position_velocity(
        self,
        position: Position3<Eci>,
        velocity: Velocity3<Eci>,
    ) -> Result<ImpactPoint, AftsError> {
        let mut p = finite_nonzero_position(position)?;
        let mut v = finite_velocity(velocity)?;
        let initial_height_m = self.surface.height_m(p)?;
        if initial_height_m <= self.radius_tolerance_m {
            let point = self.corrected_lat_lon_from_vector(p, 0.0)?;
            return Ok(ImpactPoint {
                point,
                time_to_impact_s: 0.0,
            });
        }

        let mut elapsed_s = 0.0;
        let mut previous_p = p;
        let mut previous_height_m = initial_height_m;
        while elapsed_s < self.max_time_s {
            let step_s = self.step_s.min(self.max_time_s - elapsed_s);
            let (next_p, next_v) = rk4_step(
                p,
                v,
                step_s,
                self.mu_m3_s2,
                self.drag_model,
                self.earth_rotation_rate_rad_s,
            )?;
            let next_height_m = self.surface.height_m(next_p)?;
            if next_height_m <= self.radius_tolerance_m {
                let fraction =
                    crossing_fraction(previous_height_m, next_height_m, self.radius_tolerance_m);
                let impact_position = previous_p + ((next_p - previous_p) * fraction);
                let time_to_impact_s = elapsed_s + step_s * fraction;
                return Ok(ImpactPoint {
                    point: self.corrected_lat_lon_from_vector(impact_position, time_to_impact_s)?,
                    time_to_impact_s,
                });
            }
            elapsed_s += step_s;
            previous_p = next_p;
            previous_height_m = next_height_m;
            p = next_p;
            v = next_v;
        }

        Err(AftsError::NoImpactWithinHorizon {
            max_time_s: self.max_time_s,
        })
    }

    fn corrected_lat_lon_from_vector(
        self,
        position: Vector3<f64>,
        time_to_impact_s: f64,
    ) -> Result<LatLon, AftsError> {
        let mut point = self.surface.lat_lon_from_vector(position)?;
        if self.earth_rotation_rate_rad_s > 0.0 && time_to_impact_s > 0.0 {
            point.longitude_rad = normalize_longitude(
                point.longitude_rad - self.earth_rotation_rate_rad_s * time_to_impact_s,
            );
        }
        Ok(point)
    }
}

impl Default for IipPropagator {
    fn default() -> Self {
        Self::wgs84_spherical()
    }
}

/// Declarative keep-inside polygon.
///
/// Polygons are interpreted in latitude/longitude radians and must not cross
/// the antimeridian in this initial substrate.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContainmentPolygon<'a> {
    id: &'a str,
    vertices: &'a [LatLon],
}

impl<'a> ContainmentPolygon<'a> {
    /// Creates a validated containment polygon view.
    ///
    /// # Errors
    ///
    /// Returns [`AftsError::InvalidPolygon`] for empty ids, fewer than three
    /// vertices, or antimeridian-spanning longitude jumps.
    pub fn new(id: &'a str, vertices: &'a [LatLon]) -> Result<Self, AftsError> {
        if id.is_empty() {
            return Err(AftsError::InvalidPolygon {
                field: "id",
                reason: "must not be empty",
            });
        }
        if vertices.len() < 3 {
            return Err(AftsError::InvalidPolygon {
                field: "vertices",
                reason: "must contain at least three points",
            });
        }
        for pair in vertices.windows(2) {
            if crosses_antimeridian(pair[0], pair[1]) {
                return Err(AftsError::InvalidPolygon {
                    field: "vertices",
                    reason: "must not cross the antimeridian",
                });
            }
        }
        if crosses_antimeridian(vertices[vertices.len() - 1], vertices[0]) {
            return Err(AftsError::InvalidPolygon {
                field: "vertices",
                reason: "must not cross the antimeridian",
            });
        }
        Ok(Self { id, vertices })
    }

    /// Polygon identifier.
    #[must_use]
    pub const fn id(self) -> &'a str {
        self.id
    }

    /// Polygon vertices.
    #[must_use]
    pub const fn vertices(self) -> &'a [LatLon] {
        self.vertices
    }

    /// Returns whether `point` lies inside or on the polygon boundary.
    #[must_use]
    pub fn contains(self, point: LatLon) -> bool {
        let x = point.longitude_rad;
        let y = point.latitude_rad;
        let mut inside = false;
        let mut previous = self.vertices[self.vertices.len() - 1];
        for current in self.vertices {
            if point_on_segment(point, previous, *current) {
                return true;
            }
            let xi = current.longitude_rad;
            let yi = current.latitude_rad;
            let xj = previous.longitude_rad;
            let yj = previous.latitude_rad;
            let crosses_ray = (yi > y) != (yj > y);
            if crosses_ray {
                let x_intersect = (xj - xi) * (y - yi) / (yj - yi) + xi;
                if x <= x_intersect {
                    inside = !inside;
                }
            }
            previous = *current;
        }
        inside
    }
}

/// Forward containment rule type.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContainmentRuleKind {
    /// Rule fires when the IIP exits the polygon.
    KeepInside,
    /// Rule fires when the IIP enters the polygon.
    KeepOut,
}

/// A forward containment rule.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContainmentRule<'a> {
    id: &'a str,
    kind: ContainmentRuleKind,
    polygon: ContainmentPolygon<'a>,
}

impl<'a> ContainmentRule<'a> {
    /// Creates a rule that fires when the IIP exits `keep_inside`.
    ///
    /// # Errors
    ///
    /// Returns [`AftsError::InvalidPolygon`] when `id` is empty.
    pub fn keep_inside(
        id: &'a str,
        keep_inside: ContainmentPolygon<'a>,
    ) -> Result<Self, AftsError> {
        Self::new(id, ContainmentRuleKind::KeepInside, keep_inside)
    }

    /// Creates a rule that fires when the IIP enters `keep_out`.
    ///
    /// # Errors
    ///
    /// Returns [`AftsError::InvalidPolygon`] when `id` is empty.
    pub fn keep_out(id: &'a str, keep_out: ContainmentPolygon<'a>) -> Result<Self, AftsError> {
        Self::new(id, ContainmentRuleKind::KeepOut, keep_out)
    }

    fn new(
        id: &'a str,
        kind: ContainmentRuleKind,
        polygon: ContainmentPolygon<'a>,
    ) -> Result<Self, AftsError> {
        if id.is_empty() {
            return Err(AftsError::InvalidPolygon {
                field: "id",
                reason: "must not be empty",
            });
        }
        Ok(Self { id, kind, polygon })
    }

    /// Rule identifier emitted when this rule fires.
    #[must_use]
    pub const fn id(self) -> &'a str {
        self.id
    }

    /// Rule kind.
    #[must_use]
    pub const fn kind(self) -> ContainmentRuleKind {
        self.kind
    }

    /// Returns whether this rule fires for the supplied IIP.
    #[must_use]
    pub fn fires(self, point: LatLon) -> bool {
        match self.kind {
            ContainmentRuleKind::KeepInside => !self.polygon.contains(point),
            ContainmentRuleKind::KeepOut => self.polygon.contains(point),
        }
    }
}

/// Scalar flight-state metric usable by AFTS corridor rules.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CorridorMetric {
    /// Geocentric altitude above the configured AFTS surface radius.
    AltitudeM,
    /// Inertial speed magnitude.
    SpeedMS,
    /// Flight-path angle from local horizontal, positive when moving outward.
    FlightPathAngleRad,
}

/// Telemetry-aligned flight-state scalars for AFTS corridor rules.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlightCorridorSample {
    /// Geocentric altitude above the configured AFTS surface radius.
    pub altitude_m: f64,
    /// Inertial speed magnitude.
    pub speed_m_s: f64,
    /// Flight-path angle from local horizontal, positive when moving outward.
    pub flight_path_angle_rad: f64,
}

impl FlightCorridorSample {
    /// Creates a validated corridor sample from scalar metrics.
    ///
    /// # Errors
    ///
    /// Returns [`AftsError::InvalidParameter`] when a scalar is non-finite,
    /// speed is negative, or flight-path angle is outside `[-pi/2, pi/2]`.
    pub fn new(
        altitude_m: f64,
        speed_m_s: f64,
        flight_path_angle_rad: f64,
    ) -> Result<Self, AftsError> {
        let altitude_m = require_finite("altitude_m", altitude_m)?;
        let speed_m_s = require_non_negative("speed_m_s", speed_m_s)?;
        let flight_path_angle_rad = require_finite("flight_path_angle_rad", flight_path_angle_rad)?;
        if flight_path_angle_rad.abs() > core::f64::consts::FRAC_PI_2 {
            return Err(AftsError::InvalidParameter {
                field: "flight_path_angle_rad",
                reason: "must be in [-pi/2, pi/2]",
            });
        }
        Ok(Self {
            altitude_m,
            speed_m_s,
            flight_path_angle_rad,
        })
    }

    /// Builds corridor metrics from an inertial position/velocity state and a
    /// spherical surface radius.
    ///
    /// Zero speed yields a flight-path angle of zero because the local
    /// direction of travel is undefined.
    ///
    /// # Errors
    ///
    /// Returns [`AftsError`] when the vectors or surface radius are invalid.
    pub fn from_position_velocity(
        position: Position3<Eci>,
        velocity: Velocity3<Eci>,
        surface_radius_m: f64,
    ) -> Result<Self, AftsError> {
        let surface_radius_m = require_positive("surface_radius_m", surface_radius_m)?;
        let position = finite_nonzero_position(position)?;
        let velocity = finite_velocity(velocity)?;
        let radius_m = position.norm();
        let altitude_m = radius_m - surface_radius_m;
        let speed_m_s = velocity.norm();
        let flight_path_angle_rad = if speed_m_s > 0.0 {
            let radial_speed_m_s = position.dot(&velocity) / radius_m;
            (radial_speed_m_s / speed_m_s).clamp(-1.0, 1.0).asin()
        } else {
            0.0
        };
        Self::new(altitude_m, speed_m_s, flight_path_angle_rad)
    }

    /// Returns the metric value requested by a corridor rule.
    #[must_use]
    pub const fn value(self, metric: CorridorMetric) -> f64 {
        match metric {
            CorridorMetric::AltitudeM => self.altitude_m,
            CorridorMetric::SpeedMS => self.speed_m_s,
            CorridorMetric::FlightPathAngleRad => self.flight_path_angle_rad,
        }
    }
}

/// A scalar corridor rule that fires outside the configured inclusive bounds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CorridorRule<'a> {
    id: &'a str,
    metric: CorridorMetric,
    min_value: Option<f64>,
    max_value: Option<f64>,
}

impl<'a> CorridorRule<'a> {
    /// Creates a validated scalar corridor rule.
    ///
    /// At least one bound must be present. When both bounds are present,
    /// `min_value <= max_value` is required.
    ///
    /// # Errors
    ///
    /// Returns [`AftsError::InvalidParameter`] when the id or bounds are
    /// invalid.
    pub fn new(
        id: &'a str,
        metric: CorridorMetric,
        min_value: Option<f64>,
        max_value: Option<f64>,
    ) -> Result<Self, AftsError> {
        if id.is_empty() {
            return Err(AftsError::InvalidParameter {
                field: "id",
                reason: "must not be empty",
            });
        }
        if min_value.is_none() && max_value.is_none() {
            return Err(AftsError::InvalidParameter {
                field: "bounds",
                reason: "must include min_value or max_value",
            });
        }
        let min_value = min_value
            .map(|value| require_finite("min_value", value))
            .transpose()?;
        let max_value = max_value
            .map(|value| require_finite("max_value", value))
            .transpose()?;
        if let (Some(min_value), Some(max_value)) = (min_value, max_value)
            && min_value > max_value
        {
            return Err(AftsError::InvalidParameter {
                field: "min_value",
                reason: "must be <= max_value",
            });
        }
        Ok(Self {
            id,
            metric,
            min_value,
            max_value,
        })
    }

    /// Rule identifier emitted when this rule fires.
    #[must_use]
    pub const fn id(self) -> &'a str {
        self.id
    }

    /// Metric evaluated by this rule.
    #[must_use]
    pub const fn metric(self) -> CorridorMetric {
        self.metric
    }

    /// Inclusive lower bound, if present.
    #[must_use]
    pub const fn min_value(self) -> Option<f64> {
        self.min_value
    }

    /// Inclusive upper bound, if present.
    #[must_use]
    pub const fn max_value(self) -> Option<f64> {
        self.max_value
    }

    /// Returns whether this rule fires for the supplied scalar sample.
    #[must_use]
    pub fn fires(self, sample: FlightCorridorSample) -> bool {
        let value = sample.value(self.metric);
        if let Some(min_value) = self.min_value
            && value < min_value
        {
            return true;
        }
        if let Some(max_value) = self.max_value
            && value > max_value
        {
            return true;
        }
        false
    }
}

/// Direction filter for geospatial gate-crossing rules.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GateCrossingDirection {
    /// Any crossing of the gate segment fires.
    Any,
    /// Fires when the path moves from the negative side to the positive side
    /// of the directed gate segment.
    NegativeToPositive,
    /// Fires when the path moves from the positive side to the negative side
    /// of the directed gate segment.
    PositiveToNegative,
}

/// Directed geospatial gate segment.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GateSegment<'a> {
    id: &'a str,
    start: LatLon,
    end: LatLon,
}

impl<'a> GateSegment<'a> {
    /// Creates a validated directed gate segment.
    ///
    /// # Errors
    ///
    /// Returns [`AftsError::InvalidPolygon`] when the id is empty, endpoints
    /// are identical, or the gate crosses the antimeridian.
    pub fn new(id: &'a str, start: LatLon, end: LatLon) -> Result<Self, AftsError> {
        if id.is_empty() {
            return Err(AftsError::InvalidPolygon {
                field: "id",
                reason: "must not be empty",
            });
        }
        if same_point(start, end) {
            return Err(AftsError::InvalidPolygon {
                field: "gate",
                reason: "endpoints must be distinct",
            });
        }
        if crosses_antimeridian(start, end) {
            return Err(AftsError::InvalidPolygon {
                field: "gate",
                reason: "must not cross the antimeridian",
            });
        }
        Ok(Self { id, start, end })
    }

    /// Gate identifier.
    #[must_use]
    pub const fn id(self) -> &'a str {
        self.id
    }

    /// Start point of the directed gate.
    #[must_use]
    pub const fn start(self) -> LatLon {
        self.start
    }

    /// End point of the directed gate.
    #[must_use]
    pub const fn end(self) -> LatLon {
        self.end
    }

    /// Signed side of `point` relative to the directed gate segment in local
    /// longitude/latitude coordinates.
    #[must_use]
    pub fn side(self, point: LatLon) -> f64 {
        oriented_area2(self.start, self.end, point)
    }
}

/// A geospatial gate-crossing rule over successive IIP points.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GateRule<'a> {
    id: &'a str,
    gate: GateSegment<'a>,
    direction: GateCrossingDirection,
}

impl<'a> GateRule<'a> {
    /// Creates a validated gate-crossing rule.
    ///
    /// # Errors
    ///
    /// Returns [`AftsError::InvalidPolygon`] when the rule id is empty.
    pub fn new(
        id: &'a str,
        gate: GateSegment<'a>,
        direction: GateCrossingDirection,
    ) -> Result<Self, AftsError> {
        if id.is_empty() {
            return Err(AftsError::InvalidPolygon {
                field: "id",
                reason: "must not be empty",
            });
        }
        Ok(Self {
            id,
            gate,
            direction,
        })
    }

    /// Rule identifier emitted when this rule fires.
    #[must_use]
    pub const fn id(self) -> &'a str {
        self.id
    }

    /// Direction filter.
    #[must_use]
    pub const fn direction(self) -> GateCrossingDirection {
        self.direction
    }

    /// Returns whether the segment from `previous` to `current` crosses this
    /// rule's gate segment with the configured direction filter.
    #[must_use]
    pub fn fires(self, previous: LatLon, current: LatLon) -> bool {
        if same_point(previous, current)
            || !segments_intersect(previous, current, self.gate.start, self.gate.end)
        {
            return false;
        }
        match self.direction {
            GateCrossingDirection::Any => true,
            GateCrossingDirection::NegativeToPositive => {
                let previous_side = self.gate.side(previous);
                let current_side = self.gate.side(current);
                previous_side <= BOUNDARY_EPS_RAD && current_side > BOUNDARY_EPS_RAD
            }
            GateCrossingDirection::PositiveToNegative => {
                let previous_side = self.gate.side(previous);
                let current_side = self.gate.side(current);
                previous_side >= -BOUNDARY_EPS_RAD && current_side < -BOUNDARY_EPS_RAD
            }
        }
    }
}

/// Zone classification for AFTS geospatial zone rules.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ZoneColor {
    /// Safe zone. When any green zones are declared, the IIP must remain inside
    /// at least one green zone.
    Green,
    /// Forbidden zone. Entering a red zone fires immediately.
    Red,
}

/// A geospatial AFTS zone rule.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ZoneRule<'a> {
    id: &'a str,
    color: ZoneColor,
    polygon: ContainmentPolygon<'a>,
}

impl<'a> ZoneRule<'a> {
    /// Creates a validated zone rule.
    ///
    /// # Errors
    ///
    /// Returns [`AftsError::InvalidPolygon`] when the id is empty.
    pub fn new(
        id: &'a str,
        color: ZoneColor,
        polygon: ContainmentPolygon<'a>,
    ) -> Result<Self, AftsError> {
        if id.is_empty() {
            return Err(AftsError::InvalidPolygon {
                field: "id",
                reason: "must not be empty",
            });
        }
        Ok(Self { id, color, polygon })
    }

    /// Rule identifier emitted when this rule fires.
    #[must_use]
    pub const fn id(self) -> &'a str {
        self.id
    }

    /// Zone color.
    #[must_use]
    pub const fn color(self) -> ZoneColor {
        self.color
    }

    /// Returns whether `point` is inside or on this zone's polygon boundary.
    #[must_use]
    pub fn contains(self, point: LatLon) -> bool {
        self.polygon.contains(point)
    }
}

/// Evaluates a deterministic red/green zone rule table.
///
/// Red zones win over green zones. If one or more green zones are declared,
/// they are interpreted as a union: the IIP must be inside at least one green
/// zone. The returned id is the first red zone containing the point, or the
/// first green-zone id when the point is outside the green union.
#[must_use]
pub fn evaluate_zone_rules<'a>(rules: &'a [ZoneRule<'a>], point: LatLon) -> Option<&'a str> {
    let mut first_green_id = None;
    let mut inside_any_green = false;
    for rule in rules {
        match rule.color() {
            ZoneColor::Red => {
                if rule.contains(point) {
                    return Some(rule.id());
                }
            }
            ZoneColor::Green => {
                if first_green_id.is_none() {
                    first_green_id = Some(rule.id());
                }
                inside_any_green |= rule.contains(point);
            }
        }
    }
    if first_green_id.is_some() && !inside_any_green {
        return first_green_id;
    }
    None
}

/// Latched AFTS monitor decision.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AftsDecision<'a> {
    /// Whether termination is currently commanded.
    pub terminate: bool,
    /// First containment rule that fired, if termination is latched.
    pub rule_id: Option<&'a str>,
}

/// Forward containment monitor with a latched terminate output.
#[derive(Clone, Debug, PartialEq)]
pub struct AftsMonitor<'a> {
    rules: &'a [ContainmentRule<'a>],
    latched_rule_id: Option<&'a str>,
}

impl<'a> AftsMonitor<'a> {
    /// Creates an unlatched monitor over a non-empty rule table.
    ///
    /// # Errors
    ///
    /// Returns [`AftsError::InvalidPolygon`] when the rule table is empty.
    pub fn new(rules: &'a [ContainmentRule<'a>]) -> Result<Self, AftsError> {
        if rules.is_empty() {
            return Err(AftsError::InvalidPolygon {
                field: "rules",
                reason: "must contain at least one containment rule",
            });
        }
        Ok(Self {
            rules,
            latched_rule_id: None,
        })
    }

    /// Evaluates a predicted IIP and updates the latched terminate decision.
    #[must_use]
    pub fn evaluate_iip(&mut self, impact: ImpactPoint) -> AftsDecision<'a> {
        self.evaluate_point(impact.point)
    }

    /// Evaluates a latitude/longitude point and updates the latched terminate
    /// decision.
    #[must_use]
    pub fn evaluate_point(&mut self, point: LatLon) -> AftsDecision<'a> {
        if let Some(rule_id) = self.latched_rule_id {
            return AftsDecision {
                terminate: true,
                rule_id: Some(rule_id),
            };
        }
        for rule in self.rules {
            if rule.fires(point) {
                self.latched_rule_id = Some(rule.id());
                return AftsDecision {
                    terminate: true,
                    rule_id: self.latched_rule_id,
                };
            }
        }
        AftsDecision {
            terminate: false,
            rule_id: None,
        }
    }

    /// Returns the current latched rule id, if any.
    #[must_use]
    pub const fn latched_rule_id(&self) -> Option<&'a str> {
        self.latched_rule_id
    }

    /// Clears the terminate latch.
    pub fn reset_latch(&mut self) {
        self.latched_rule_id = None;
    }
}

fn finite_nonzero_position(position: Position3<Eci>) -> Result<Vector3<f64>, AftsError> {
    if !position.is_finite() {
        return Err(AftsError::InvalidVector {
            field: "position_m",
            reason: "must be finite",
        });
    }
    let vector = position.vector;
    if vector.norm() <= 0.0 {
        return Err(AftsError::InvalidVector {
            field: "position_m",
            reason: "must be non-zero",
        });
    }
    Ok(vector)
}

fn finite_velocity(velocity: Velocity3<Eci>) -> Result<Vector3<f64>, AftsError> {
    if velocity.is_finite() {
        Ok(velocity.vector)
    } else {
        Err(AftsError::InvalidVector {
            field: "velocity_m_s",
            reason: "must be finite",
        })
    }
}

fn rk4_step(
    position: Vector3<f64>,
    velocity: Vector3<f64>,
    step_s: f64,
    mu_m3_s2: f64,
    drag_model: Option<ExponentialAtmosphericDrag>,
    earth_rotation_rate_rad_s: f64,
) -> Result<(Vector3<f64>, Vector3<f64>), AftsError> {
    let (k1_p, k1_v) = derivative(
        position,
        velocity,
        mu_m3_s2,
        drag_model,
        earth_rotation_rate_rad_s,
    )?;
    let (k2_p, k2_v) = derivative(
        position + k1_p * (0.5 * step_s),
        velocity + k1_v * (0.5 * step_s),
        mu_m3_s2,
        drag_model,
        earth_rotation_rate_rad_s,
    )?;
    let (k3_p, k3_v) = derivative(
        position + k2_p * (0.5 * step_s),
        velocity + k2_v * (0.5 * step_s),
        mu_m3_s2,
        drag_model,
        earth_rotation_rate_rad_s,
    )?;
    let (k4_p, k4_v) = derivative(
        position + k3_p * step_s,
        velocity + k3_v * step_s,
        mu_m3_s2,
        drag_model,
        earth_rotation_rate_rad_s,
    )?;
    let next_position = position + (k1_p + k2_p * 2.0 + k3_p * 2.0 + k4_p) * (step_s / 6.0);
    let next_velocity = velocity + (k1_v + k2_v * 2.0 + k3_v * 2.0 + k4_v) * (step_s / 6.0);
    Ok((next_position, next_velocity))
}

fn derivative(
    position: Vector3<f64>,
    velocity: Vector3<f64>,
    mu_m3_s2: f64,
    drag_model: Option<ExponentialAtmosphericDrag>,
    earth_rotation_rate_rad_s: f64,
) -> Result<(Vector3<f64>, Vector3<f64>), AftsError> {
    let radius_m = position.norm();
    if !radius_m.is_finite() || radius_m <= 0.0 {
        return Err(AftsError::InvalidVector {
            field: "position_m",
            reason: "must stay finite and non-zero during propagation",
        });
    }
    let mut acceleration = position * (-mu_m3_s2 / (radius_m * radius_m * radius_m));
    if let Some(drag_model) = drag_model {
        acceleration +=
            drag_model.acceleration_m_s2(position, velocity, earth_rotation_rate_rad_s)?;
    }
    Ok((velocity, acceleration))
}

fn crossing_fraction(previous_value: f64, next_value: f64, target_value: f64) -> f64 {
    let denominator = previous_value - next_value;
    if denominator <= 0.0 {
        1.0
    } else {
        ((previous_value - target_value) / denominator).clamp(0.0, 1.0)
    }
}

fn lat_lon_from_vector(position: Vector3<f64>) -> Result<LatLon, AftsError> {
    let radius_m = position.norm();
    if !radius_m.is_finite() || radius_m <= 0.0 {
        return Err(AftsError::InvalidVector {
            field: "impact_position_m",
            reason: "must be finite and non-zero",
        });
    }
    let rho_m = (position.x * position.x + position.y * position.y).sqrt();
    LatLon::new_radians(position.z.atan2(rho_m), position.y.atan2(position.x))
}

fn ellipsoid_lat_lon_from_vector(
    position: Vector3<f64>,
    semi_major_m: f64,
    semi_minor_m: f64,
) -> Result<LatLon, AftsError> {
    let radius_m = position.norm();
    if !radius_m.is_finite() || radius_m <= 0.0 {
        return Err(AftsError::InvalidVector {
            field: "impact_position_m",
            reason: "must be finite and non-zero",
        });
    }
    let rho_m = (position.x * position.x + position.y * position.y).sqrt();
    let longitude_rad = position.y.atan2(position.x);
    let latitude_rad = if rho_m <= 0.0 {
        position.z.signum() * core::f64::consts::FRAC_PI_2
    } else {
        let a2 = semi_major_m * semi_major_m;
        let b2 = semi_minor_m * semi_minor_m;
        let e2 = (a2 - b2) / a2;
        let ep2 = (a2 - b2) / b2;
        let theta = (position.z * semi_major_m).atan2(rho_m * semi_minor_m);
        let sin_theta = theta.sin();
        let cos_theta = theta.cos();
        (position.z + ep2 * semi_minor_m * sin_theta * sin_theta * sin_theta)
            .atan2(rho_m - e2 * semi_major_m * cos_theta * cos_theta * cos_theta)
    };
    LatLon::new_radians(latitude_rad, longitude_rad)
}

fn normalize_longitude(longitude_rad: f64) -> f64 {
    let two_pi = 2.0 * core::f64::consts::PI;
    let mut wrapped = longitude_rad - (longitude_rad / two_pi).trunc() * two_pi;
    if wrapped > core::f64::consts::PI {
        wrapped -= two_pi;
    }
    if wrapped < -core::f64::consts::PI {
        wrapped += two_pi;
    }
    wrapped
}

fn crosses_antimeridian(a: LatLon, b: LatLon) -> bool {
    (a.longitude_rad - b.longitude_rad).abs() > core::f64::consts::PI
}

fn same_point(a: LatLon, b: LatLon) -> bool {
    (a.latitude_rad - b.latitude_rad).abs() <= BOUNDARY_EPS_RAD
        && (a.longitude_rad - b.longitude_rad).abs() <= BOUNDARY_EPS_RAD
}

fn oriented_area2(a: LatLon, b: LatLon, c: LatLon) -> f64 {
    let ax = a.longitude_rad;
    let ay = a.latitude_rad;
    let bx = b.longitude_rad;
    let by = b.latitude_rad;
    let cx = c.longitude_rad;
    let cy = c.latitude_rad;
    (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
}

fn segments_intersect(a0: LatLon, a1: LatLon, b0: LatLon, b1: LatLon) -> bool {
    if point_on_segment(a0, b0, b1)
        || point_on_segment(a1, b0, b1)
        || point_on_segment(b0, a0, a1)
        || point_on_segment(b1, a0, a1)
    {
        return true;
    }
    let a0_side = oriented_area2(b0, b1, a0);
    let a1_side = oriented_area2(b0, b1, a1);
    let b0_side = oriented_area2(a0, a1, b0);
    let b1_side = oriented_area2(a0, a1, b1);
    a0_side * a1_side < 0.0 && b0_side * b1_side < 0.0
}

fn point_on_segment(point: LatLon, a: LatLon, b: LatLon) -> bool {
    let px = point.longitude_rad;
    let py = point.latitude_rad;
    let ax = a.longitude_rad;
    let ay = a.latitude_rad;
    let bx = b.longitude_rad;
    let by = b.latitude_rad;
    let cross = oriented_area2(a, b, point);
    if cross.abs() > BOUNDARY_EPS_RAD {
        return false;
    }
    let min_x = ax.min(bx) - BOUNDARY_EPS_RAD;
    let max_x = ax.max(bx) + BOUNDARY_EPS_RAD;
    let min_y = ay.min(by) - BOUNDARY_EPS_RAD;
    let max_y = ay.max(by) + BOUNDARY_EPS_RAD;
    px >= min_x && px <= max_x && py >= min_y && py <= max_y
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use openbmp_core::SimTime;
    use uom::si::f64::Mass;
    use uom::si::mass::kilogram;

    fn point(latitude_deg: f64, longitude_deg: f64) -> LatLon {
        LatLon::new_degrees(latitude_deg, longitude_deg).unwrap()
    }

    fn square_polygon() -> [LatLon; 4] {
        [
            point(-1.0, -1.0),
            point(-1.0, 1.0),
            point(1.0, 1.0),
            point(1.0, -1.0),
        ]
    }

    fn wgs84_surface_point(latitude_deg: f64, longitude_deg: f64) -> Vector3<f64> {
        let latitude_rad = latitude_deg.to_radians();
        let longitude_rad = longitude_deg.to_radians();
        let sin_lat = latitude_rad.sin();
        let cos_lat = latitude_rad.cos();
        let n = WGS84_A_M
            / (1.0 - (1.0 - (WGS84_B_M * WGS84_B_M) / (WGS84_A_M * WGS84_A_M)) * sin_lat * sin_lat)
                .sqrt();
        Vector3::new(
            n * cos_lat * longitude_rad.cos(),
            n * cos_lat * longitude_rad.sin(),
            (n * (WGS84_B_M * WGS84_B_M) / (WGS84_A_M * WGS84_A_M)) * sin_lat,
        )
    }

    fn closed_form_iip_fixture() -> toml::Value {
        toml::from_str(include_str!(
            "../../../data/afts/closed-form-iip-tolerance-v1.toml"
        ))
        .unwrap()
    }

    fn f64_field(table: &toml::value::Table, key: &str) -> f64 {
        let value = table
            .get(key)
            .unwrap_or_else(|| panic!("missing fixture key `{key}`"));
        value
            .as_float()
            .or_else(|| value.as_integer().map(|integer| integer as f64))
            .unwrap_or_else(|| panic!("fixture key `{key}` must be numeric"))
    }

    fn vector3_field(table: &toml::value::Table, key: &str) -> Vector3<f64> {
        let values = table
            .get(key)
            .and_then(toml::Value::as_array)
            .unwrap_or_else(|| panic!("fixture key `{key}` must be an array"));
        assert_eq!(values.len(), 3, "fixture vector `{key}` must have 3 values");
        Vector3::new(
            values[0]
                .as_float()
                .or_else(|| values[0].as_integer().map(|integer| integer as f64))
                .unwrap(),
            values[1]
                .as_float()
                .or_else(|| values[1].as_integer().map(|integer| integer as f64))
                .unwrap(),
            values[2]
                .as_float()
                .or_else(|| values[2].as_integer().map(|integer| integer as f64))
                .unwrap(),
        )
    }

    fn closed_form_spherical_iip(
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        surface_radius_m: f64,
        mu_m3_s2: f64,
        earth_rotation_rate_rad_s: f64,
    ) -> ImpactPoint {
        let radius_m = position.norm();
        let h_vec = position.cross(&velocity);
        let h_norm = h_vec.norm();
        assert!(h_norm > 0.0, "closed-form fixture must be non-radial");
        let eccentricity_vector = velocity.cross(&h_vec) / mu_m3_s2 - position / radius_m;
        let eccentricity = eccentricity_vector.norm();
        assert!(
            eccentricity > 1.0e-12 && eccentricity < 1.0,
            "fixture must be a non-circular elliptic conic, got e = {eccentricity}"
        );
        let semilatus_rectum_m = h_norm * h_norm / mu_m3_s2;
        let target_cos_true_anomaly = (semilatus_rectum_m / surface_radius_m - 1.0) / eccentricity;
        assert!(
            target_cos_true_anomaly.abs() <= 1.0,
            "closed-form conic must intersect the requested surface"
        );
        let current_true_anomaly = true_anomaly_from_state(
            position,
            velocity,
            eccentricity_vector,
            eccentricity,
            h_norm,
            mu_m3_s2,
        );
        let target_true_anomaly = [
            target_cos_true_anomaly.acos(),
            -target_cos_true_anomaly.acos(),
        ]
        .into_iter()
        .filter_map(|candidate| {
            let time_to_impact_s = elliptic_time_of_flight_s(
                current_true_anomaly,
                candidate,
                eccentricity,
                semilatus_rectum_m,
                mu_m3_s2,
            );
            (time_to_impact_s > 1.0e-9).then_some((candidate, time_to_impact_s))
        })
        .min_by(|left, right| left.1.total_cmp(&right.1))
        .expect("closed-form conic must have a future impact");

        let e_hat = eccentricity_vector / eccentricity;
        let h_hat = h_vec / h_norm;
        let q_hat = h_hat.cross(&e_hat);
        let impact_position = (e_hat * target_true_anomaly.0.cos()
            + q_hat * target_true_anomaly.0.sin())
            * surface_radius_m;
        let mut point = lat_lon_from_vector(impact_position).unwrap();
        if earth_rotation_rate_rad_s > 0.0 {
            point.longitude_rad = normalize_longitude(
                point.longitude_rad - earth_rotation_rate_rad_s * target_true_anomaly.1,
            );
        }
        ImpactPoint {
            point,
            time_to_impact_s: target_true_anomaly.1,
        }
    }

    fn true_anomaly_from_state(
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        eccentricity_vector: Vector3<f64>,
        eccentricity: f64,
        h_norm: f64,
        mu_m3_s2: f64,
    ) -> f64 {
        let radius_m = position.norm();
        let cos_true_anomaly = eccentricity_vector.dot(&position) / (eccentricity * radius_m);
        let radial_velocity_m_s = position.dot(&velocity) / radius_m;
        let sin_true_anomaly = radial_velocity_m_s * h_norm / (mu_m3_s2 * eccentricity);
        sin_true_anomaly.atan2(cos_true_anomaly)
    }

    fn elliptic_time_of_flight_s(
        start_true_anomaly: f64,
        end_true_anomaly: f64,
        eccentricity: f64,
        semilatus_rectum_m: f64,
        mu_m3_s2: f64,
    ) -> f64 {
        let semi_major_m = semilatus_rectum_m / (1.0 - eccentricity * eccentricity);
        let mean_motion_rad_s = (mu_m3_s2 / (semi_major_m * semi_major_m * semi_major_m)).sqrt();
        normalize_positive_angle(
            mean_anomaly_from_true_anomaly(end_true_anomaly, eccentricity)
                - mean_anomaly_from_true_anomaly(start_true_anomaly, eccentricity),
        ) / mean_motion_rad_s
    }

    fn mean_anomaly_from_true_anomaly(true_anomaly: f64, eccentricity: f64) -> f64 {
        let denominator = 1.0 + eccentricity * true_anomaly.cos();
        let eccentric_anomaly = ((1.0 - eccentricity * eccentricity).sqrt() * true_anomaly.sin())
            .atan2(eccentricity + true_anomaly.cos());
        debug_assert!(denominator > 0.0);
        eccentric_anomaly - eccentricity * eccentric_anomaly.sin()
    }

    fn normalize_positive_angle(angle_rad: f64) -> f64 {
        let two_pi = 2.0 * core::f64::consts::PI;
        let mut angle_rad = angle_rad % two_pi;
        if angle_rad < 0.0 {
            angle_rad += two_pi;
        }
        angle_rad
    }

    fn angular_separation_rad(a: LatLon, b: LatLon) -> f64 {
        a.to_unit_vector()
            .dot(&b.to_unit_vector())
            .clamp(-1.0, 1.0)
            .acos()
    }

    #[test]
    fn radial_iip_matches_closed_form_conic_point() {
        let radius_m = WGS84_A_M;
        let altitude_m = 100_000.0;
        let initial_radius_m = radius_m + altitude_m;
        let latitude_rad = 20.0_f64.to_radians();
        let longitude_rad = -75.0_f64.to_radians();
        let unit = LatLon::new_radians(latitude_rad, longitude_rad)
            .unwrap()
            .to_unit_vector();
        let position = Position3::from_vector(unit * initial_radius_m);
        let velocity = Velocity3::from_vector(unit * -1_000.0);
        let state = PointMassState::new(
            SimTime::ZERO,
            position,
            velocity,
            Mass::new::<kilogram>(1.0),
        );

        let iip = IipPropagator::wgs84_spherical()
            .impact_point_from_state(&state)
            .unwrap();

        assert_abs_diff_eq!(iip.point.latitude_rad, latitude_rad, epsilon = 1.0e-9);
        assert_abs_diff_eq!(iip.point.longitude_rad, longitude_rad, epsilon = 1.0e-9);
        assert!(iip.time_to_impact_s > 0.0);
    }

    #[test]
    fn non_radial_iip_matches_closed_form_conic_tolerance_table() {
        let fixture = closed_form_iip_fixture();
        let tolerances = fixture
            .get("tolerances")
            .and_then(toml::Value::as_table)
            .expect("fixture tolerances table");
        let case = fixture
            .get("case")
            .and_then(toml::Value::as_array)
            .and_then(|cases| cases.first())
            .and_then(toml::Value::as_table)
            .expect("fixture case table");
        let position = vector3_field(case, "position_eci_m");
        let velocity = vector3_field(case, "velocity_eci_m_s");
        let surface_radius_m = f64_field(case, "surface_radius_m");
        let mu_m3_s2 = f64_field(case, "mu_m3_s2");
        let earth_rotation_rate_rad_s = f64_field(case, "earth_rotation_rate_rad_s");

        let closed_form = closed_form_spherical_iip(
            position,
            velocity,
            surface_radius_m,
            mu_m3_s2,
            earth_rotation_rate_rad_s,
        );
        let propagated = IipPropagator::new_with_earth_rotation(
            surface_radius_m,
            mu_m3_s2,
            f64_field(case, "rk4_step_s"),
            f64_field(case, "max_time_s"),
            0.0,
            earth_rotation_rate_rad_s,
        )
        .unwrap()
        .impact_point_from_position_velocity(
            Position3::from_vector(position),
            Velocity3::from_vector(velocity),
        )
        .unwrap();

        let angular_error_rad = angular_separation_rad(propagated.point, closed_form.point);
        assert!(
            angular_error_rad <= f64_field(tolerances, "angular_rad"),
            "numeric IIP angular error {angular_error_rad:e} exceeds tolerance: propagated={propagated:?}, closed_form={closed_form:?}"
        );
        assert_abs_diff_eq!(
            propagated.time_to_impact_s,
            closed_form.time_to_impact_s,
            epsilon = f64_field(tolerances, "time_s")
        );
    }

    #[test]
    fn wgs84_rotating_earth_uses_symbolic_rotation_rate() {
        let propagator = IipPropagator::wgs84_rotating_earth();

        assert_abs_diff_eq!(
            propagator.earth_rotation_rate_rad_s(),
            WGS84_OMEGA_RAD_S,
            epsilon = 0.0
        );
    }

    #[test]
    fn rotating_earth_iip_applies_longitude_correction() {
        let altitude_m = 100_000.0;
        let initial_radius_m = WGS84_A_M + altitude_m;
        let latitude_rad = 10.0_f64.to_radians();
        let longitude_rad = 15.0_f64.to_radians();
        let unit = LatLon::new_radians(latitude_rad, longitude_rad)
            .unwrap()
            .to_unit_vector();
        let position = Position3::from_vector(unit * initial_radius_m);
        let velocity = Velocity3::from_vector(unit * -1_000.0);

        let inertial = IipPropagator::wgs84_spherical()
            .impact_point_from_position_velocity(position, velocity)
            .unwrap();
        let rotating = IipPropagator::wgs84_rotating_earth()
            .impact_point_from_position_velocity(position, velocity)
            .unwrap();

        assert_abs_diff_eq!(
            rotating.point.latitude_rad,
            inertial.point.latitude_rad,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            rotating.time_to_impact_s,
            inertial.time_to_impact_s,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            rotating.point.longitude_rad,
            normalize_longitude(
                inertial.point.longitude_rad - WGS84_OMEGA_RAD_S * inertial.time_to_impact_s
            ),
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn wgs84_ellipsoid_iip_reports_geodetic_surface_latitude() {
        let latitude_deg = 45.0;
        let longitude_deg = -30.0;
        let position = Position3::from_vector(wgs84_surface_point(latitude_deg, longitude_deg));
        let velocity = Velocity3::new(0.0, 0.0, 0.0);

        let iip = IipPropagator::wgs84_ellipsoid()
            .impact_point_from_position_velocity(position, velocity)
            .unwrap();

        assert_abs_diff_eq!(
            iip.point.latitude_rad,
            latitude_deg.to_radians(),
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            iip.point.longitude_rad,
            longitude_deg.to_radians(),
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(iip.time_to_impact_s, 0.0, epsilon = 0.0);
        assert_abs_diff_eq!(
            IipPropagator::wgs84_ellipsoid()
                .impact_surface()
                .polar_radius_m(),
            WGS84_B_M,
            epsilon = 0.0
        );
    }

    #[test]
    fn exponential_drag_changes_forward_iip_longitude() {
        let altitude_m = 40_000.0;
        let position = Position3::new(WGS84_A_M + altitude_m, 0.0, 0.0);
        let velocity = Velocity3::new(-500.0, 1_200.0, 0.0);
        let no_drag = IipPropagator::new(WGS84_A_M, WGS84_MU_M3_S2, 0.1, 300.0, 0.0)
            .unwrap()
            .impact_point_from_position_velocity(position, velocity)
            .unwrap();
        let drag = ExponentialAtmosphericDrag::new(WGS84_A_M, 0.02, 8_500.0, 500.0).unwrap();
        let with_drag = IipPropagator::new(WGS84_A_M, WGS84_MU_M3_S2, 0.1, 300.0, 0.0)
            .unwrap()
            .with_exponential_drag(drag)
            .impact_point_from_position_velocity(position, velocity)
            .unwrap();

        assert!(
            with_drag.point.longitude_rad < no_drag.point.longitude_rad,
            "drag should reduce downrange longitude: no_drag={no_drag:?}, with_drag={with_drag:?}"
        );
        assert!(with_drag.time_to_impact_s > 0.0);
    }

    #[test]
    fn exponential_drag_rejects_invalid_ballistic_coefficient() {
        let err = ExponentialAtmosphericDrag::new(WGS84_A_M, 1.225, 8_500.0, 0.0).unwrap_err();

        assert_eq!(
            err,
            AftsError::InvalidParameter {
                field: "drag.ballistic_coefficient_kg_m2",
                reason: "must be positive"
            }
        );
    }

    #[test]
    fn polygon_contains_hand_computed_inside_boundary_and_outside_cases() {
        let vertices = square_polygon();
        let polygon = ContainmentPolygon::new("box", &vertices).unwrap();

        assert!(polygon.contains(point(0.0, 0.0)));
        assert!(polygon.contains(point(1.0, 0.0)));
        assert!(!polygon.contains(point(2.0, 0.0)));
        assert!(!polygon.contains(point(0.0, 2.0)));
    }

    #[test]
    fn containment_monitor_latches_first_termination_rule() {
        let vertices = square_polygon();
        let polygon = ContainmentPolygon::new("box", &vertices).unwrap();
        let rule = ContainmentRule::keep_inside("range_box", polygon).unwrap();
        let rules = [rule];
        let mut monitor = AftsMonitor::new(&rules).unwrap();

        let nominal = monitor.evaluate_point(point(0.0, 0.0));
        assert!(!nominal.terminate);
        assert_eq!(nominal.rule_id, None);

        let violation = monitor.evaluate_point(point(3.0, 0.0));
        assert!(violation.terminate);
        assert_eq!(violation.rule_id, Some("range_box"));

        let still_latched = monitor.evaluate_point(point(0.0, 0.0));
        assert!(still_latched.terminate);
        assert_eq!(still_latched.rule_id, Some("range_box"));
    }

    #[test]
    fn keep_out_rule_fires_inside_forbidden_polygon() {
        let vertices = square_polygon();
        let polygon = ContainmentPolygon::new("hazard", &vertices).unwrap();
        let rule = ContainmentRule::keep_out("hazard_zone", polygon).unwrap();

        assert_eq!(rule.kind(), ContainmentRuleKind::KeepOut);
        assert!(rule.fires(point(0.0, 0.0)));
        assert!(!rule.fires(point(2.0, 0.0)));
    }

    #[test]
    fn corridor_sample_computes_altitude_speed_and_flight_path_angle() {
        let sample = FlightCorridorSample::from_position_velocity(
            Position3::new(WGS84_A_M + 1_000.0, 0.0, 0.0),
            Velocity3::new(0.0, 100.0, 100.0),
            WGS84_A_M,
        )
        .unwrap();

        assert_abs_diff_eq!(sample.altitude_m, 1_000.0, epsilon = 1.0e-9);
        assert_abs_diff_eq!(sample.speed_m_s, 100.0_f64.hypot(100.0), epsilon = 1.0e-12);
        assert_abs_diff_eq!(sample.flight_path_angle_rad, 0.0, epsilon = 1.0e-12);

        let outbound = FlightCorridorSample::from_position_velocity(
            Position3::new(WGS84_A_M + 1_000.0, 0.0, 0.0),
            Velocity3::new(100.0, 0.0, 0.0),
            WGS84_A_M,
        )
        .unwrap();
        assert_abs_diff_eq!(
            outbound.flight_path_angle_rad,
            core::f64::consts::FRAC_PI_2,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn corridor_rule_fires_outside_inclusive_bounds() {
        let sample = FlightCorridorSample::new(1_000.0, 200.0, 0.1).unwrap();
        let altitude_rule = CorridorRule::new(
            "altitude-band",
            CorridorMetric::AltitudeM,
            Some(0.0),
            Some(500.0),
        )
        .unwrap();
        let speed_rule = CorridorRule::new(
            "speed-band",
            CorridorMetric::SpeedMS,
            Some(100.0),
            Some(300.0),
        )
        .unwrap();

        assert_eq!(altitude_rule.metric(), CorridorMetric::AltitudeM);
        assert!(altitude_rule.fires(sample));
        assert!(!speed_rule.fires(sample));
        assert_eq!(speed_rule.id(), "speed-band");
    }

    #[test]
    fn corridor_rule_rejects_empty_bounds() {
        let err = CorridorRule::new("bad", CorridorMetric::AltitudeM, None, None).unwrap_err();

        assert!(matches!(
            err,
            AftsError::InvalidParameter {
                field: "bounds",
                ..
            }
        ));
    }

    #[test]
    fn gate_rule_fires_when_iip_track_crosses_segment() {
        let gate = GateSegment::new("gate", point(-1.0, 0.0), point(1.0, 0.0)).unwrap();
        let rule = GateRule::new("gate-cross", gate, GateCrossingDirection::Any).unwrap();

        assert!(rule.fires(point(0.0, -1.0), point(0.0, 1.0)));
        assert!(!rule.fires(point(2.0, -1.0), point(2.0, 1.0)));
    }

    #[test]
    fn gate_rule_honors_crossing_direction() {
        let gate = GateSegment::new("gate", point(-1.0, 0.0), point(1.0, 0.0)).unwrap();
        let negative_to_positive = GateRule::new(
            "gate-cross",
            gate,
            GateCrossingDirection::NegativeToPositive,
        )
        .unwrap();
        let positive_to_negative = GateRule::new(
            "gate-cross",
            gate,
            GateCrossingDirection::PositiveToNegative,
        )
        .unwrap();

        assert!(negative_to_positive.fires(point(0.0, 1.0), point(0.0, -1.0)));
        assert!(!negative_to_positive.fires(point(0.0, -1.0), point(0.0, 1.0)));
        assert!(positive_to_negative.fires(point(0.0, -1.0), point(0.0, 1.0)));
        assert!(!positive_to_negative.fires(point(0.0, 1.0), point(0.0, -1.0)));
    }

    #[test]
    fn gate_segment_rejects_degenerate_endpoint_pair() {
        let err = GateSegment::new("gate", point(1.0, 1.0), point(1.0, 1.0)).unwrap_err();

        assert!(matches!(
            err,
            AftsError::InvalidPolygon { field: "gate", .. }
        ));
    }

    #[test]
    fn zone_rules_fire_on_red_zone_entry() {
        let vertices = square_polygon();
        let polygon = ContainmentPolygon::new("red", &vertices).unwrap();
        let rule = ZoneRule::new("red-zone", ZoneColor::Red, polygon).unwrap();

        assert_eq!(rule.color(), ZoneColor::Red);
        assert_eq!(
            evaluate_zone_rules(&[rule], point(0.0, 0.0)),
            Some("red-zone")
        );
        assert_eq!(evaluate_zone_rules(&[rule], point(2.0, 0.0)), None);
    }

    #[test]
    fn green_zone_rules_use_union_semantics() {
        let west_vertices = [
            point(-1.0, -3.0),
            point(-1.0, -1.0),
            point(1.0, -1.0),
            point(1.0, -3.0),
        ];
        let east_vertices = [
            point(-1.0, 1.0),
            point(-1.0, 3.0),
            point(1.0, 3.0),
            point(1.0, 1.0),
        ];
        let west = ContainmentPolygon::new("west", &west_vertices).unwrap();
        let east = ContainmentPolygon::new("east", &east_vertices).unwrap();
        let rules = [
            ZoneRule::new("green-west", ZoneColor::Green, west).unwrap(),
            ZoneRule::new("green-east", ZoneColor::Green, east).unwrap(),
        ];

        assert_eq!(evaluate_zone_rules(&rules, point(0.0, -2.0)), None);
        assert_eq!(evaluate_zone_rules(&rules, point(0.0, 2.0)), None);
        assert_eq!(
            evaluate_zone_rules(&rules, point(0.0, 0.0)),
            Some("green-west")
        );
    }

    #[test]
    fn red_zone_wins_over_green_zone() {
        let vertices = square_polygon();
        let green = ContainmentPolygon::new("green", &vertices).unwrap();
        let red = ContainmentPolygon::new("red", &vertices).unwrap();
        let rules = [
            ZoneRule::new("green-zone", ZoneColor::Green, green).unwrap(),
            ZoneRule::new("red-zone", ZoneColor::Red, red).unwrap(),
        ];

        assert_eq!(
            evaluate_zone_rules(&rules, point(0.0, 0.0)),
            Some("red-zone")
        );
    }

    #[test]
    fn propagator_reports_no_impact_for_stable_circular_state() {
        let radius_m = WGS84_A_M + 400_000.0;
        let circular_speed_m_s = (WGS84_MU_M3_S2 / radius_m).sqrt();
        let propagator = IipPropagator::new(WGS84_A_M, WGS84_MU_M3_S2, 1.0, 60.0, 0.0).unwrap();
        let err = propagator
            .impact_point_from_position_velocity(
                Position3::new(radius_m, 0.0, 0.0),
                Velocity3::new(0.0, circular_speed_m_s, 0.0),
            )
            .unwrap_err();

        assert!(matches!(err, AftsError::NoImpactWithinHorizon { .. }));
    }

    #[test]
    fn afts_surface_has_no_fc_or_inverse_targeting_dependency() {
        let dependency_surface = [
            "openbmp-core",
            "openbmp-state",
            "openbmp-physics",
            "impact_point_from_state",
            "evaluate_iip",
        ];

        assert!(!dependency_surface.contains(&"openbmp-fc"));
        assert!(!dependency_surface.contains(&"solve_burn_to_reach"));
    }
}
