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
//! * [`TesseralGravity`] — the first static non-zonal harmonic surface,
//!   currently degree 2 / order 2, using Cartesian solid-harmonic
//!   polynomials for deterministic tesseral and sectoral acceleration.
//!
//! All models implement the [`GravityModel`] trait and report
//! failure via [`crate::error::PhysicsError`] (out-of-envelope, non-finite,
//! invalid parameter). They never panic, never silent-clamp, and
//! never return `NaN`.
//!
//! Determinism: pure arithmetic on `f64`; locked operand order on the
//! J2 sum; no FMA.

use nalgebra::Vector3;
#[cfg(not(feature = "std"))]
use num_traits::Float;
use openbmp_core::{Eci, Position3, SimTime, Velocity3};

#[cfg(feature = "std")]
use crate::ephemeris::{ASTRONOMICAL_UNIT_M, CelestialBody, EphemerisModel};
use crate::error::PhysicsError;
use crate::frames::{WGS84_A_M, WGS84_MU_M3_S2};

/// WGS84 unnormalised J2 zonal-harmonic coefficient.
///
/// Source: NIMA TR8350.2, WGS84 Implementation Manual, §3.
pub const WGS84_J2: f64 = 1.082_626_683e-3;

/// ISO / USSA76 standard gravity (m/s²).
pub const STANDARD_GRAVITY_M_S2: f64 = 9.806_65;

/// Speed of light in vacuum, m/s (SI exact).
pub const SPEED_OF_LIGHT_M_S: f64 = 299_792_458.0;

/// Solar radiation pressure at 1 astronomical unit, in N/m².
///
/// This is the standard cannonball-SRP engineering constant used by
/// Montenbruck-Gill style force models.
pub const SOLAR_RADIATION_PRESSURE_1_AU_N_M2: f64 = 4.56e-6;

/// IAU 2015 nominal solar radius, in metres.
pub const IAU_NOMINAL_SOLAR_RADIUS_M: f64 = 695_700_000.0;

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
// ThirdBodyGravity
// ---------------------------------------------------------------------

/// One perturbing third body.
#[cfg(feature = "std")]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ThirdBody {
    body: CelestialBody,
    mu_m3_s2: f64,
}

#[cfg(feature = "std")]
impl ThirdBody {
    /// Construct from a built-in celestial body and its canonical
    /// gravitational parameter.
    #[must_use]
    pub const fn canonical(body: CelestialBody) -> Self {
        Self {
            body,
            mu_m3_s2: body.mu_m3_s2(),
        }
    }

    /// Construct from an explicit gravitational parameter.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if `mu_m3_s2` is
    /// not strictly positive and finite.
    pub fn new(body: CelestialBody, mu_m3_s2: f64) -> Result<Self, PhysicsError> {
        if !mu_m3_s2.is_finite() || mu_m3_s2 <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "third-body gravitational parameter must be strictly positive and finite",
            });
        }
        Ok(Self { body, mu_m3_s2 })
    }

    /// Celestial body identifier.
    #[must_use]
    pub const fn body(&self) -> CelestialBody {
        self.body
    }

    /// Gravitational parameter in m^3/s^2.
    #[must_use]
    pub const fn mu_m3_s2(&self) -> f64 {
        self.mu_m3_s2
    }
}

/// Central gravity plus third-body point-mass perturbations.
///
/// The perturbing acceleration is evaluated in the central-body frame:
///
/// ```text
/// q   = r · (r - 2r_b) / |r_b|²
/// f(q)= q(3 + 3q + q²) / (1 + (1+q)^(3/2))
/// a_3 = -μ_b / |r_b - r|³ · (r + f(q) r_b)
/// ```
///
/// where `r` is the vehicle position relative to Earth and `r_b` is
/// the perturbing body's Earth-centered inertial position from the configured
/// ephemeris model. This is Battin's cancellation-free form of the ordinary
/// third-body difference.
#[derive(Clone, Debug)]
#[cfg(feature = "std")]
pub struct ThirdBodyGravity<G, E> {
    central: G,
    ephemeris: E,
    bodies: Vec<ThirdBody>,
}

#[cfg(feature = "std")]
impl<G, E> ThirdBodyGravity<G, E> {
    /// Construct a third-body gravity model.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if no perturbing
    /// bodies are provided.
    pub fn new(
        central: G,
        ephemeris: E,
        bodies: impl Into<Vec<ThirdBody>>,
    ) -> Result<Self, PhysicsError> {
        let bodies = bodies.into();
        if bodies.is_empty() {
            return Err(PhysicsError::InvalidParameter {
                reason: "third-body gravity requires at least one perturbing body",
            });
        }
        Ok(Self {
            central,
            ephemeris,
            bodies,
        })
    }

    /// Wrapped central gravity model.
    #[must_use]
    pub const fn central(&self) -> &G {
        &self.central
    }

    /// Wrapped ephemeris model.
    #[must_use]
    pub const fn ephemeris(&self) -> &E {
        &self.ephemeris
    }

    /// Perturbing bodies in deterministic evaluation order.
    #[must_use]
    pub fn bodies(&self) -> &[ThirdBody] {
        &self.bodies
    }
}

#[cfg(feature = "std")]
impl<G: GravityModel, E: EphemerisModel> GravityModel for ThirdBodyGravity<G, E> {
    fn gravity_eci_m_s2(
        &self,
        position_eci: Position3<Eci>,
        time: SimTime,
    ) -> Result<Vector3<f64>, PhysicsError> {
        let mut acceleration = self.central.gravity_eci_m_s2(position_eci, time)?;
        for body in &self.bodies {
            let body_position = self.ephemeris.body_position_eci_m(body.body, time)?;
            acceleration +=
                third_body_perturbation(position_eci.vector, body_position, body.mu_m3_s2)?;
        }
        if !acceleration.iter().all(|v| v.is_finite()) {
            return Err(PhysicsError::NonFinite {
                reason: "third-body gravity produced non-finite acceleration",
            });
        }
        Ok(acceleration)
    }
}

#[cfg(feature = "std")]
fn third_body_perturbation(
    vehicle_position: Vector3<f64>,
    body_position: Vector3<f64>,
    mu_m3_s2: f64,
) -> Result<Vector3<f64>, PhysicsError> {
    let relative = body_position - vehicle_position;
    let relative_r2 = relative.dot(&relative);
    let body_r2 = body_position.dot(&body_position);
    if relative_r2 == 0.0 || body_r2 == 0.0 {
        return Err(PhysicsError::OutOfEnvelope {
            reason: "third-body perturbation is singular at coincident body positions",
        });
    }
    let relative_r = relative_r2.sqrt();
    let q = vehicle_position.dot(&(vehicle_position - 2.0 * body_position)) / body_r2;
    let one_plus_q = relative_r2 / body_r2;
    let one_plus_q_3_over_2 = one_plus_q * one_plus_q.sqrt();
    let f_q = q * (3.0 + 3.0 * q + q * q) / (1.0 + one_plus_q_3_over_2);
    let perturbation =
        (-mu_m3_s2 / (relative_r * relative_r2)) * (vehicle_position + f_q * body_position);
    if !perturbation.iter().all(|v| v.is_finite()) {
        return Err(PhysicsError::NonFinite {
            reason: "third-body perturbation produced non-finite acceleration",
        });
    }
    Ok(perturbation)
}

// ---------------------------------------------------------------------
// SolarRadiationPressure
// ---------------------------------------------------------------------

/// Cannonball solar-radiation-pressure perturbation with conical Earth shadow.
///
/// The model uses a scalar reflectivity coefficient `C_r`, area-to-mass ratio,
/// inverse-square solar pressure scaling from 1 AU, and a conical
/// umbra/penumbra shadow factor from the apparent overlap of the solar and
/// Earth disks. It returns SRP acceleration only; central gravity remains an
/// explicit separate model so existing force stacks stay byte-identical until
/// this model is deliberately composed.
#[derive(Clone, Debug)]
#[cfg(feature = "std")]
pub struct SolarRadiationPressure<E> {
    ephemeris: E,
    area_m2: f64,
    mass_kg: f64,
    coefficient_reflectivity: f64,
    solar_pressure_1_au_n_m2: f64,
    occulting_radius_m: f64,
    solar_radius_m: f64,
}

#[cfg(feature = "std")]
impl<E> SolarRadiationPressure<E> {
    /// Construct a cannonball SRP model using OpenBMP's default solar and
    /// Earth constants.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if area, mass, or
    /// reflectivity are not strictly positive and finite.
    pub fn new(
        ephemeris: E,
        area_m2: f64,
        mass_kg: f64,
        coefficient_reflectivity: f64,
    ) -> Result<Self, PhysicsError> {
        Self::with_constants(
            ephemeris,
            area_m2,
            mass_kg,
            coefficient_reflectivity,
            SOLAR_RADIATION_PRESSURE_1_AU_N_M2,
            WGS84_A_M,
            IAU_NOMINAL_SOLAR_RADIUS_M,
        )
    }

    /// Construct a cannonball SRP model with explicit solar-pressure, occulting
    /// body, and solar-radius constants.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if any scalar parameter is
    /// not strictly positive and finite.
    #[allow(clippy::too_many_arguments)]
    pub fn with_constants(
        ephemeris: E,
        area_m2: f64,
        mass_kg: f64,
        coefficient_reflectivity: f64,
        solar_pressure_1_au_n_m2: f64,
        occulting_radius_m: f64,
        solar_radius_m: f64,
    ) -> Result<Self, PhysicsError> {
        if !area_m2.is_finite() || area_m2 <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "SRP cannonball area must be strictly positive and finite",
            });
        }
        if !mass_kg.is_finite() || mass_kg <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "SRP cannonball mass must be strictly positive and finite",
            });
        }
        if !coefficient_reflectivity.is_finite() || coefficient_reflectivity <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "SRP reflectivity coefficient must be strictly positive and finite",
            });
        }
        if !solar_pressure_1_au_n_m2.is_finite() || solar_pressure_1_au_n_m2 <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "SRP solar pressure must be strictly positive and finite",
            });
        }
        if !occulting_radius_m.is_finite() || occulting_radius_m <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "SRP occulting radius must be strictly positive and finite",
            });
        }
        if !solar_radius_m.is_finite() || solar_radius_m <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "SRP solar radius must be strictly positive and finite",
            });
        }
        Ok(Self {
            ephemeris,
            area_m2,
            mass_kg,
            coefficient_reflectivity,
            solar_pressure_1_au_n_m2,
            occulting_radius_m,
            solar_radius_m,
        })
    }

    /// Wrapped ephemeris model.
    #[must_use]
    pub const fn ephemeris(&self) -> &E {
        &self.ephemeris
    }

    /// Cannonball cross-sectional area in m².
    #[must_use]
    pub const fn area_m2(&self) -> f64 {
        self.area_m2
    }

    /// Spacecraft mass in kg.
    #[must_use]
    pub const fn mass_kg(&self) -> f64 {
        self.mass_kg
    }

    /// Cannonball reflectivity coefficient `C_r`.
    #[must_use]
    pub const fn coefficient_reflectivity(&self) -> f64 {
        self.coefficient_reflectivity
    }

    /// Cross-sectional area-to-mass ratio in m²/kg.
    #[must_use]
    pub fn area_to_mass_m2_kg(&self) -> f64 {
        self.area_m2 / self.mass_kg
    }

    /// Solar pressure at 1 AU in N/m².
    #[must_use]
    pub const fn solar_pressure_1_au_n_m2(&self) -> f64 {
        self.solar_pressure_1_au_n_m2
    }

    /// Radius of the occulting body in metres.
    #[must_use]
    pub const fn occulting_radius_m(&self) -> f64 {
        self.occulting_radius_m
    }

    /// Solar radius in metres.
    #[must_use]
    pub const fn solar_radius_m(&self) -> f64 {
        self.solar_radius_m
    }

    /// Evaluate only the conical-shadow sunlight fraction `ν`.
    ///
    /// `ν = 1` means full sunlight, `ν = 0` means umbra, and intermediate
    /// values are penumbra from exact circular-disk overlap on the apparent
    /// sky.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] if the ephemeris query fails, the spacecraft is
    /// inside the occulting body, or the geometry is singular/non-finite.
    pub fn shadow_factor(
        &self,
        position_eci: Position3<Eci>,
        time: SimTime,
    ) -> Result<f64, PhysicsError>
    where
        E: EphemerisModel,
    {
        let sun_position = self
            .ephemeris
            .body_position_eci_m(CelestialBody::Sun, time)?;
        conical_shadow_factor(
            position_eci.vector,
            sun_position,
            self.occulting_radius_m,
            self.solar_radius_m,
        )
    }

    /// Solar-radiation-pressure acceleration in ECI, in m/s².
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] if the ephemeris query fails or if the SRP
    /// geometry is singular/out of envelope.
    pub fn acceleration_eci_m_s2(
        &self,
        position_eci: Position3<Eci>,
        time: SimTime,
    ) -> Result<Vector3<f64>, PhysicsError>
    where
        E: EphemerisModel,
    {
        let sun_position = self
            .ephemeris
            .body_position_eci_m(CelestialBody::Sun, time)?;
        solar_radiation_pressure_perturbation(
            position_eci.vector,
            sun_position,
            self.area_m2,
            self.mass_kg,
            self.coefficient_reflectivity,
            self.solar_pressure_1_au_n_m2,
            self.occulting_radius_m,
            self.solar_radius_m,
        )
    }
}

#[cfg(feature = "std")]
impl<E: EphemerisModel> GravityModel for SolarRadiationPressure<E> {
    fn gravity_eci_m_s2(
        &self,
        position_eci: Position3<Eci>,
        time: SimTime,
    ) -> Result<Vector3<f64>, PhysicsError> {
        self.acceleration_eci_m_s2(position_eci, time)
    }
}

#[cfg(feature = "std")]
#[allow(clippy::too_many_arguments)]
fn solar_radiation_pressure_perturbation(
    vehicle_position: Vector3<f64>,
    sun_position: Vector3<f64>,
    area_m2: f64,
    mass_kg: f64,
    coefficient_reflectivity: f64,
    solar_pressure_1_au_n_m2: f64,
    occulting_radius_m: f64,
    solar_radius_m: f64,
) -> Result<Vector3<f64>, PhysicsError> {
    let sun_to_vehicle = vehicle_position - sun_position;
    let sun_to_vehicle_r2 = sun_to_vehicle.dot(&sun_to_vehicle);
    if sun_to_vehicle_r2 == 0.0 {
        return Err(PhysicsError::OutOfEnvelope {
            reason: "SRP is singular at the solar centre",
        });
    }
    let sun_to_vehicle_r = sun_to_vehicle_r2.sqrt();
    let shadow_factor = conical_shadow_factor(
        vehicle_position,
        sun_position,
        occulting_radius_m,
        solar_radius_m,
    )?;
    let pressure_scale =
        solar_pressure_1_au_n_m2 * (ASTRONOMICAL_UNIT_M * ASTRONOMICAL_UNIT_M) / sun_to_vehicle_r2;
    let acceleration = shadow_factor
        * pressure_scale
        * coefficient_reflectivity
        * (area_m2 / mass_kg)
        * (sun_to_vehicle / sun_to_vehicle_r);
    if !acceleration.iter().all(|v| v.is_finite()) {
        return Err(PhysicsError::NonFinite {
            reason: "SRP produced non-finite acceleration",
        });
    }
    Ok(acceleration)
}

#[cfg(feature = "std")]
fn conical_shadow_factor(
    vehicle_position: Vector3<f64>,
    sun_position: Vector3<f64>,
    occulting_radius_m: f64,
    solar_radius_m: f64,
) -> Result<f64, PhysicsError> {
    if !vehicle_position.iter().all(|v| v.is_finite())
        || !sun_position.iter().all(|v| v.is_finite())
    {
        return Err(PhysicsError::InvalidParameter {
            reason: "SRP shadow geometry vectors must be finite",
        });
    }
    let earth_to_vehicle_r2 = vehicle_position.dot(&vehicle_position);
    if earth_to_vehicle_r2 == 0.0 {
        return Err(PhysicsError::OutOfEnvelope {
            reason: "SRP shadow is singular at the occulting-body centre",
        });
    }
    let sun_from_vehicle = sun_position - vehicle_position;
    let sun_from_vehicle_r2 = sun_from_vehicle.dot(&sun_from_vehicle);
    if sun_from_vehicle_r2 == 0.0 {
        return Err(PhysicsError::OutOfEnvelope {
            reason: "SRP shadow is singular at the solar centre",
        });
    }
    let earth_from_vehicle = -vehicle_position;
    let earth_from_vehicle_r = earth_to_vehicle_r2.sqrt();
    let sun_from_vehicle_r = sun_from_vehicle_r2.sqrt();
    if earth_from_vehicle_r <= occulting_radius_m {
        return Err(PhysicsError::OutOfEnvelope {
            reason: "SRP shadow is undefined inside the occulting body",
        });
    }
    if sun_from_vehicle_r <= solar_radius_m {
        return Err(PhysicsError::OutOfEnvelope {
            reason: "SRP shadow is undefined inside the solar body",
        });
    }
    let occulting_angular_radius = (occulting_radius_m / earth_from_vehicle_r).asin();
    let solar_angular_radius = (solar_radius_m / sun_from_vehicle_r).asin();
    let cos_separation = clamp_unit(
        sun_from_vehicle.dot(&earth_from_vehicle) / (sun_from_vehicle_r * earth_from_vehicle_r),
    );
    let separation = cos_separation.acos();
    let shadow_factor =
        solar_disk_visible_fraction(solar_angular_radius, occulting_angular_radius, separation);
    if !shadow_factor.is_finite() {
        return Err(PhysicsError::NonFinite {
            reason: "SRP shadow factor produced non-finite output",
        });
    }
    Ok(shadow_factor)
}

#[cfg(feature = "std")]
fn solar_disk_visible_fraction(
    solar_angular_radius: f64,
    occulting_angular_radius: f64,
    center_separation_angle: f64,
) -> f64 {
    let sun_r = solar_angular_radius;
    let occ_r = occulting_angular_radius;
    let separation = center_separation_angle;
    if separation >= sun_r + occ_r {
        return 1.0;
    }
    if separation <= (occ_r - sun_r).abs() {
        if occ_r >= sun_r {
            return 0.0;
        }
        return clamp_unit_interval(1.0 - (occ_r * occ_r) / (sun_r * sun_r));
    }

    let sun_r2 = sun_r * sun_r;
    let occ_r2 = occ_r * occ_r;
    let separation2 = separation * separation;
    let sun_segment =
        sun_r2 * clamp_unit((separation2 + sun_r2 - occ_r2) / (2.0 * separation * sun_r)).acos();
    let occ_segment =
        occ_r2 * clamp_unit((separation2 + occ_r2 - sun_r2) / (2.0 * separation * occ_r)).acos();
    let triangle = 0.5
        * ((-separation + sun_r + occ_r)
            * (separation + sun_r - occ_r)
            * (separation - sun_r + occ_r)
            * (separation + sun_r + occ_r))
            .max(0.0)
            .sqrt();
    let overlap_area = sun_segment + occ_segment - triangle;
    clamp_unit_interval(1.0 - overlap_area / (core::f64::consts::PI * sun_r2))
}

#[cfg(feature = "std")]
fn clamp_unit(value: f64) -> f64 {
    value.clamp(-1.0, 1.0)
}

#[cfg(feature = "std")]
fn clamp_unit_interval(value: f64) -> f64 {
    value.clamp(0.0, 1.0)
}

// ---------------------------------------------------------------------
// RelativisticCorrection
// ---------------------------------------------------------------------

/// First post-Newtonian Schwarzschild correction for a central body.
///
/// The correction uses `β = γ = 1` and the named speed of light `c`:
///
/// ```text
/// a_rel = μ/(c²r³) · [ (4μ/r − v²) r + 4(r·v)v ]
/// ```
///
/// It intentionally excludes Lense-Thirring and de Sitter terms; those remain
/// outside the WP-08.2 parity ceiling.
#[derive(Copy, Clone, Debug)]
pub struct RelativisticCorrection {
    mu_m3_s2: f64,
    speed_of_light_m_s: f64,
}

impl RelativisticCorrection {
    /// Construct a Schwarzschild correction with explicit central-body `µ` and
    /// speed of light.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if either scalar is not
    /// strictly positive and finite.
    pub fn new(mu_m3_s2: f64, speed_of_light_m_s: f64) -> Result<Self, PhysicsError> {
        if !mu_m3_s2.is_finite() || mu_m3_s2 <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "relativistic correction µ must be strictly positive and finite",
            });
        }
        if !speed_of_light_m_s.is_finite() || speed_of_light_m_s <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "relativistic correction c must be strictly positive and finite",
            });
        }
        Ok(Self {
            mu_m3_s2,
            speed_of_light_m_s,
        })
    }

    /// WGS84 Earth Schwarzschild correction using the exact SI speed of light.
    #[must_use]
    pub const fn wgs84_schwarzschild() -> Self {
        Self {
            mu_m3_s2: WGS84_MU_M3_S2,
            speed_of_light_m_s: SPEED_OF_LIGHT_M_S,
        }
    }

    /// Configured gravitational parameter in m³/s².
    #[must_use]
    pub const fn mu_m3_s2(&self) -> f64 {
        self.mu_m3_s2
    }

    /// Configured speed of light in m/s.
    #[must_use]
    pub const fn speed_of_light_m_s(&self) -> f64 {
        self.speed_of_light_m_s
    }

    /// Schwarzschild correction acceleration in ECI, in m/s².
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] if the position is singular, the velocity is
    /// non-finite, or the resulting correction is non-finite.
    pub fn acceleration_eci_m_s2(
        &self,
        position_eci: Position3<Eci>,
        velocity_eci_m_s: Velocity3<Eci>,
    ) -> Result<Vector3<f64>, PhysicsError> {
        schwarzschild_perturbation(
            position_eci.vector,
            velocity_eci_m_s.vector,
            self.mu_m3_s2,
            self.speed_of_light_m_s,
        )
    }
}

fn schwarzschild_perturbation(
    position_eci_m: Vector3<f64>,
    velocity_eci_m_s: Vector3<f64>,
    mu_m3_s2: f64,
    speed_of_light_m_s: f64,
) -> Result<Vector3<f64>, PhysicsError> {
    let r2 = position_eci_m.dot(&position_eci_m);
    if r2 == 0.0 {
        return Err(PhysicsError::OutOfEnvelope {
            reason: "Schwarzschild correction is singular at r = 0",
        });
    }
    if !position_eci_m.iter().all(|v| v.is_finite())
        || !velocity_eci_m_s.iter().all(|v| v.is_finite())
    {
        return Err(PhysicsError::InvalidParameter {
            reason: "Schwarzschild state vectors must be finite",
        });
    }
    let r = r2.sqrt();
    let v2 = velocity_eci_m_s.dot(&velocity_eci_m_s);
    let radial_rate_term = position_eci_m.dot(&velocity_eci_m_s);
    let c2 = speed_of_light_m_s * speed_of_light_m_s;
    let coefficient = mu_m3_s2 / (c2 * r2 * r);
    let acceleration = coefficient
        * (((4.0 * mu_m3_s2 / r) - v2) * position_eci_m
            + 4.0 * radial_rate_term * velocity_eci_m_s);
    if !acceleration.iter().all(|v| v.is_finite()) {
        return Err(PhysicsError::NonFinite {
            reason: "Schwarzschild correction produced non-finite acceleration",
        });
    }
    Ok(acceleration)
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
// TesseralGravity
// ---------------------------------------------------------------------

/// Maximum harmonic degree implemented by the current [`TesseralGravity`]
/// evaluator.
///
/// This is an intentionally narrow WP-08.1 starter slice. The full parity
/// target is the Pines/Gottlieb runtime-selectable EGM2008 kernel; this
/// constant keeps the public surface fail-closed until that kernel lands.
pub const TESSERAL_GRAVITY_MAX_DEGREE: usize = 2;

/// Maximum harmonic order implemented by the current [`TesseralGravity`]
/// evaluator.
pub const TESSERAL_GRAVITY_MAX_ORDER: usize = 2;

/// Permanent-tide convention associated with a harmonic coefficient block.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum TideSystem {
    /// Tide-free field.
    TideFree,
    /// Zero-tide field.
    ZeroTide,
    /// Mean-tide field.
    MeanTide,
}

impl TideSystem {
    /// Parse a canonical tide-system tag used by gravity coefficient blocks.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] for unknown tags. The parser
    /// is intentionally narrow so data-ingestion call sites fail closed instead
    /// of silently accepting alternate spellings.
    pub fn from_tag(tag: &str) -> Result<Self, PhysicsError> {
        match tag {
            "tide_free" => Ok(Self::TideFree),
            "zero_tide" => Ok(Self::ZeroTide),
            "mean_tide" => Ok(Self::MeanTide),
            _ => Err(PhysicsError::InvalidParameter {
                reason: "unknown gravity coefficient tide_system tag",
            }),
        }
    }

    /// Canonical tide-system tag.
    #[must_use]
    pub const fn tag(&self) -> &'static str {
        match self {
            Self::TideFree => "tide_free",
            Self::ZeroTide => "zero_tide",
            Self::MeanTide => "mean_tide",
        }
    }
}

/// Degree-2 fully-normalized harmonic coefficients for [`TesseralGravity`].
///
/// This is the ingestion-facing low-degree coefficient block used before the
/// full high-degree Pines/Gottlieb kernel lands. It accepts the common
/// fully-normalized `Cbar/Sbar` convention and converts to the current
/// degree-2 unnormalized evaluator deterministically.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct NormalizedDegreeTwoTesseralCoefficients {
    cbar20: f64,
    cbar21: f64,
    sbar21: f64,
    cbar22: f64,
    sbar22: f64,
    tide_system: TideSystem,
}

impl NormalizedDegreeTwoTesseralCoefficients {
    /// Construct finite degree-2 fully-normalized coefficients.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if any coefficient is
    /// non-finite.
    pub fn new(
        cbar20: f64,
        cbar21: f64,
        sbar21: f64,
        cbar22: f64,
        sbar22: f64,
        tide_system: TideSystem,
    ) -> Result<Self, PhysicsError> {
        let coefficients = [cbar20, cbar21, sbar21, cbar22, sbar22];
        if !coefficients.iter().all(|value| value.is_finite()) {
            return Err(PhysicsError::InvalidParameter {
                reason: "degree-2 normalized tesseral coefficients must be finite",
            });
        }
        Ok(Self {
            cbar20,
            cbar21,
            sbar21,
            cbar22,
            sbar22,
            tide_system,
        })
    }

    /// Fully-normalized `Cbar20`.
    #[must_use]
    pub const fn cbar20(&self) -> f64 {
        self.cbar20
    }

    /// Fully-normalized `Cbar21`.
    #[must_use]
    pub const fn cbar21(&self) -> f64 {
        self.cbar21
    }

    /// Fully-normalized `Sbar21`.
    #[must_use]
    pub const fn sbar21(&self) -> f64 {
        self.sbar21
    }

    /// Fully-normalized `Cbar22`.
    #[must_use]
    pub const fn cbar22(&self) -> f64 {
        self.cbar22
    }

    /// Fully-normalized `Sbar22`.
    #[must_use]
    pub const fn sbar22(&self) -> f64 {
        self.sbar22
    }

    /// Permanent-tide convention declared for the coefficients.
    #[must_use]
    pub const fn tide_system(&self) -> TideSystem {
        self.tide_system
    }

    /// Convert to the unnormalized degree-2 coefficient convention used by the
    /// current [`TesseralGravity`] evaluator.
    ///
    /// The factors are the real fully-normalized associated-Legendre factors
    /// for `(n, m) = (2, 0), (2, 1), (2, 2)`: `sqrt(5)`, `sqrt(5/3)`, and
    /// `sqrt(5/12)`.
    pub fn to_unnormalized(self) -> Result<DegreeTwoTesseralCoefficients, PhysicsError> {
        DegreeTwoTesseralCoefficients::new(
            self.cbar20 * 5.0_f64.sqrt(),
            self.cbar21 * (5.0_f64 / 3.0).sqrt(),
            self.sbar21 * (5.0_f64 / 3.0).sqrt(),
            self.cbar22 * (5.0_f64 / 12.0).sqrt(),
            self.sbar22 * (5.0_f64 / 12.0).sqrt(),
            self.tide_system,
        )
    }
}

/// Degree-2 unnormalised harmonic coefficients for [`TesseralGravity`].
///
/// Coefficients follow the low-degree geopotential convention
///
/// ```text
/// U = µ/r · [1 + (R/r)^2 · (C20 P20 + P21(C21 cosλ + S21 sinλ)
///                         + P22(C22 cos2λ + S22 sin2λ))]
/// ```
///
/// where `C20 = -J2` reproduces the existing [`J2Gravity`] sign convention.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct DegreeTwoTesseralCoefficients {
    c20: f64,
    c21: f64,
    s21: f64,
    c22: f64,
    s22: f64,
    tide_system: TideSystem,
}

impl DegreeTwoTesseralCoefficients {
    /// Construct finite degree-2 coefficients.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if any coefficient is
    /// non-finite.
    pub fn new(
        c20: f64,
        c21: f64,
        s21: f64,
        c22: f64,
        s22: f64,
        tide_system: TideSystem,
    ) -> Result<Self, PhysicsError> {
        let coefficients = [c20, c21, s21, c22, s22];
        if !coefficients.iter().all(|value| value.is_finite()) {
            return Err(PhysicsError::InvalidParameter {
                reason: "degree-2 tesseral coefficients must be finite",
            });
        }
        Ok(Self {
            c20,
            c21,
            s21,
            c22,
            s22,
            tide_system,
        })
    }

    /// Coefficients that reproduce [`J2Gravity`] for the supplied positive
    /// `J2` value.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if `j2` is non-finite.
    pub fn from_j2(j2: f64, tide_system: TideSystem) -> Result<Self, PhysicsError> {
        if !j2.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "J2 must be finite",
            });
        }
        Self::new(-j2, 0.0, 0.0, 0.0, 0.0, tide_system)
    }

    /// Convert a fully-normalized degree-2 coefficient block into the
    /// unnormalized convention used by this evaluator.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if the normalized block
    /// contains non-finite coefficients.
    pub fn from_fully_normalized(
        normalized: NormalizedDegreeTwoTesseralCoefficients,
    ) -> Result<Self, PhysicsError> {
        normalized.to_unnormalized()
    }

    /// All harmonic coefficients set to zero.
    #[must_use]
    pub const fn zero(tide_system: TideSystem) -> Self {
        Self {
            c20: 0.0,
            c21: 0.0,
            s21: 0.0,
            c22: 0.0,
            s22: 0.0,
            tide_system,
        }
    }

    /// Zonal `C20` coefficient.
    #[must_use]
    pub const fn c20(&self) -> f64 {
        self.c20
    }

    /// Tesseral `C21` coefficient.
    #[must_use]
    pub const fn c21(&self) -> f64 {
        self.c21
    }

    /// Tesseral `S21` coefficient.
    #[must_use]
    pub const fn s21(&self) -> f64 {
        self.s21
    }

    /// Sectoral `C22` coefficient.
    #[must_use]
    pub const fn c22(&self) -> f64 {
        self.c22
    }

    /// Sectoral `S22` coefficient.
    #[must_use]
    pub const fn s22(&self) -> f64 {
        self.s22
    }

    /// Permanent-tide convention declared for the coefficients.
    #[must_use]
    pub const fn tide_system(&self) -> TideSystem {
        self.tide_system
    }
}

/// Static degree-2 tesseral gravity evaluator.
///
/// This model is off by default and does not ingest EGM2008 yet. It is the
/// first WP-08.1 substrate: a fail-closed non-zonal force surface that proves
/// the public API, degree/order validation, J2 byte-regression path, and
/// singularity-free Cartesian degree-2 tesseral/sectoral acceleration. Full
/// Pines/Gottlieb high-degree synthesis remains future work.
#[derive(Copy, Clone, Debug)]
pub struct TesseralGravity {
    mu_m3_s2: f64,
    r_e_m: f64,
    coefficients: DegreeTwoTesseralCoefficients,
    degree: usize,
    order: usize,
}

impl TesseralGravity {
    /// Construct a degree-2/order-2-or-lower tesseral gravity model.
    ///
    /// `degree = 0` is permitted and evaluates point-mass gravity only.
    /// `degree = 2` evaluates the configured degree-2 coefficients up to
    /// `order`. Degree 1 and degrees above [`TESSERAL_GRAVITY_MAX_DEGREE`] are
    /// rejected until the full harmonic kernel lands.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] for invalid gravity
    /// constants, unsupported degree/order, or non-finite coefficients.
    pub fn new(
        mu_m3_s2: f64,
        r_e_m: f64,
        coefficients: DegreeTwoTesseralCoefficients,
        degree: usize,
        order: usize,
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
        if degree == 1 || degree > TESSERAL_GRAVITY_MAX_DEGREE {
            return Err(PhysicsError::InvalidParameter {
                reason: "tesseral gravity currently supports degree 0 or 2 only",
            });
        }
        if order > degree || order > TESSERAL_GRAVITY_MAX_ORDER {
            return Err(PhysicsError::InvalidParameter {
                reason: "tesseral gravity order must be <= degree and <= 2",
            });
        }
        Ok(Self {
            mu_m3_s2,
            r_e_m,
            coefficients,
            degree,
            order,
        })
    }

    /// WGS84 degree-2/order-0 model that is byte-identical to
    /// [`J2Gravity::wgs84`] for the same query point.
    #[must_use]
    pub const fn wgs84_j2() -> Self {
        Self {
            mu_m3_s2: WGS84_MU_M3_S2,
            r_e_m: WGS84_A_M,
            coefficients: DegreeTwoTesseralCoefficients {
                c20: -WGS84_J2,
                c21: 0.0,
                s21: 0.0,
                c22: 0.0,
                s22: 0.0,
                tide_system: TideSystem::TideFree,
            },
            degree: 2,
            order: 0,
        }
    }

    /// Construct from a fully-normalized degree-2 coefficient block.
    ///
    /// This is the data-ingestion bridge for the current low-degree tesseral
    /// substrate. It does not lift the degree/order ceiling; high-degree
    /// EGM2008 synthesis remains guarded behind the future Pines/Gottlieb
    /// kernel.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] for invalid constants,
    /// unsupported degree/order, or non-finite coefficients.
    pub fn from_normalized_degree_two(
        mu_m3_s2: f64,
        r_e_m: f64,
        normalized: NormalizedDegreeTwoTesseralCoefficients,
        degree: usize,
        order: usize,
    ) -> Result<Self, PhysicsError> {
        Self::new(
            mu_m3_s2,
            r_e_m,
            normalized.to_unnormalized()?,
            degree,
            order,
        )
    }

    /// Configured `µ` (m³/s²).
    #[must_use]
    pub const fn mu_m3_s2(&self) -> f64 {
        self.mu_m3_s2
    }

    /// Configured reference radius (m).
    #[must_use]
    pub const fn r_e_m(&self) -> f64 {
        self.r_e_m
    }

    /// Degree-2 coefficients.
    #[must_use]
    pub const fn coefficients(&self) -> DegreeTwoTesseralCoefficients {
        self.coefficients
    }

    /// Configured maximum degree.
    #[must_use]
    pub const fn degree(&self) -> usize {
        self.degree
    }

    /// Configured maximum order.
    #[must_use]
    pub const fn order(&self) -> usize {
        self.order
    }
}

impl GravityModel for TesseralGravity {
    fn gravity_eci_m_s2(
        &self,
        position_eci: Position3<Eci>,
        _time: SimTime,
    ) -> Result<Vector3<f64>, PhysicsError> {
        let r = position_eci.vector;
        let r2 = r.dot(&r);
        if r2 == 0.0 {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "TesseralGravity is singular at r = 0",
            });
        }
        let r_norm = r2.sqrt();
        let r3 = r_norm * r2;
        let r5 = r3 * r2;
        let g_central = (-self.mu_m3_s2 / r3) * r;
        if self.degree == 0 {
            return Ok(g_central);
        }
        let g_degree_two = if self.order == 0 {
            // This preserves the existing J2 implementation exactly for the
            // degree-2/order-0 regression path.
            j2_perturbation_eci(r, r2, r5, self.mu_m3_s2, self.r_e_m, -self.coefficients.c20)
        } else {
            degree_two_tesseral_perturbation_eci(
                r,
                r2,
                r_norm,
                self.mu_m3_s2,
                self.r_e_m,
                self.coefficients,
                self.order,
            )
        };
        let g = g_central + g_degree_two;
        if !g.iter().all(|value| value.is_finite()) {
            return Err(PhysicsError::NonFinite {
                reason: "tesseral gravity produced non-finite acceleration",
            });
        }
        Ok(g)
    }
}

fn degree_two_tesseral_perturbation_eci(
    r: Vector3<f64>,
    r2: f64,
    r_norm: f64,
    mu_m3_s2: f64,
    r_e_m: f64,
    coefficients: DegreeTwoTesseralCoefficients,
    order: usize,
) -> Vector3<f64> {
    let x = r.x;
    let y = r.y;
    let z = r.z;
    let mut n = 0.5 * coefficients.c20 * (2.0 * z * z - x * x - y * y);
    let mut dn_dx = -coefficients.c20 * x;
    let mut dn_dy = -coefficients.c20 * y;
    let mut dn_dz = 2.0 * coefficients.c20 * z;

    if order >= 1 {
        n += 3.0 * z * (coefficients.c21 * x + coefficients.s21 * y);
        dn_dx += 3.0 * coefficients.c21 * z;
        dn_dy += 3.0 * coefficients.s21 * z;
        dn_dz += 3.0 * (coefficients.c21 * x + coefficients.s21 * y);
    }
    if order >= 2 {
        n += 3.0 * coefficients.c22 * (x * x - y * y) + 6.0 * coefficients.s22 * x * y;
        dn_dx += 6.0 * coefficients.c22 * x + 6.0 * coefficients.s22 * y;
        dn_dy += -6.0 * coefficients.c22 * y + 6.0 * coefficients.s22 * x;
    }

    let r5 = r2 * r2 * r_norm;
    let r7 = r5 * r2;
    let scale = mu_m3_s2 * r_e_m * r_e_m;
    Vector3::new(
        scale * (dn_dx / r5 - 5.0 * n * x / r7),
        scale * (dn_dy / r5 - 5.0 * n * y / r7),
        scale * (dn_dz / r5 - 5.0 * n * z / r7),
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
    use crate::ephemeris::{CelestialBody, LowPrecisionSunMoonEphemeris};
    use approx::assert_abs_diff_eq;

    fn at_x(x: f64) -> Position3<Eci> {
        Position3::new(x, 0.0, 0.0)
    }

    fn at_z(z: f64) -> Position3<Eci> {
        Position3::new(0.0, 0.0, z)
    }

    fn load_wgs84_normalized_degree_two_fixture() -> NormalizedDegreeTwoTesseralCoefficients {
        let fixture = include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/gravity/wgs84-degree2-normalized-v1.toml"
        ));
        let value: toml::Value = toml::from_str(fixture).expect("gravity coefficient TOML parses");
        let table = value.as_table().expect("gravity coefficient TOML table");
        assert_eq!(
            table
                .get("dataset_id")
                .and_then(toml::Value::as_str)
                .expect("dataset_id"),
            "openbmp.wgs84.gravity.degree2-normalized.v1"
        );
        assert_eq!(
            table
                .get("normalization")
                .and_then(toml::Value::as_str)
                .expect("normalization"),
            "fully_normalized"
        );
        let tide_system = TideSystem::from_tag(
            table
                .get("tide_system")
                .and_then(toml::Value::as_str)
                .expect("tide_system"),
        )
        .expect("known tide system");
        let coefficients = table
            .get("degree_2")
            .and_then(toml::Value::as_table)
            .expect("degree_2 table");
        let scalar = |key: &str| -> f64 {
            coefficients
                .get(key)
                .and_then(toml::Value::as_float)
                .expect(key)
        };
        NormalizedDegreeTwoTesseralCoefficients::new(
            scalar("cbar20"),
            scalar("cbar21"),
            scalar("sbar21"),
            scalar("cbar22"),
            scalar("sbar22"),
            tide_system,
        )
        .expect("finite normalized degree-2 coefficients")
    }

    #[derive(Copy, Clone, Debug)]
    struct FixedEphemeris {
        position: Vector3<f64>,
    }

    impl EphemerisModel for FixedEphemeris {
        fn body_position_eci_m(
            &self,
            _body: CelestialBody,
            _time: SimTime,
        ) -> Result<Vector3<f64>, PhysicsError> {
            Ok(self.position)
        }
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
    fn third_body_perturbation_is_zero_at_central_origin() {
        let gravity = ThirdBodyGravity::new(
            ConstantGravity::new(Vector3::zeros()).unwrap(),
            FixedEphemeris {
                position: Vector3::new(384_400_000.0, 0.0, 0.0),
            },
            vec![ThirdBody::canonical(CelestialBody::Moon)],
        )
        .unwrap();
        let out = gravity
            .gravity_eci_m_s2(Position3::origin(), SimTime::ZERO)
            .unwrap();
        assert_abs_diff_eq!(out.x, 0.0, epsilon = 1.0e-20);
        assert_abs_diff_eq!(out.y, 0.0, epsilon = 1.0e-20);
        assert_abs_diff_eq!(out.z, 0.0, epsilon = 1.0e-20);
    }

    #[test]
    fn third_body_perturbation_is_measurable_at_high_apogee() {
        let gravity = ThirdBodyGravity::new(
            ConstantGravity::new(Vector3::zeros()).unwrap(),
            LowPrecisionSunMoonEphemeris::j2000(),
            vec![ThirdBody::canonical(CelestialBody::Moon)],
        )
        .unwrap();
        let out = gravity
            .gravity_eci_m_s2(at_x(100_000_000.0), SimTime::ZERO)
            .unwrap();
        assert!(out.norm() > 1.0e-6);
    }

    #[test]
    fn third_body_battin_matches_naive_difference_in_well_conditioned_case() {
        let vehicle_position = Vector3::new(7_000_000.0, 2_000_000.0, 1_000_000.0);
        let body_position = Vector3::new(384_400_000.0, -12_000_000.0, 3_000_000.0);
        let mu_m3_s2 = CelestialBody::Moon.mu_m3_s2();

        let battin = third_body_perturbation(vehicle_position, body_position, mu_m3_s2).unwrap();
        let naive = naive_third_body_perturbation(vehicle_position, body_position, mu_m3_s2);

        for axis in 0..3 {
            assert_abs_diff_eq!(battin[axis], naive[axis], epsilon = 1.0e-18);
        }
    }

    #[test]
    fn third_body_battin_preserves_small_component_lost_by_naive_difference() {
        let vehicle_position = Vector3::new(0.0, 1.0, 0.0);
        let body_position = Vector3::new(1.0e16, 0.0, 0.0);
        let mu_m3_s2 = 1.0;

        let battin = third_body_perturbation(vehicle_position, body_position, mu_m3_s2).unwrap();
        let naive = naive_third_body_perturbation(vehicle_position, body_position, mu_m3_s2);

        assert_eq!(naive.x.to_bits(), 0.0_f64.to_bits());
        assert!(battin.x.is_finite());
        assert!(battin.x < 0.0);
        assert_abs_diff_eq!(battin.x, -1.5e-64, epsilon = 1.0e-76);
        assert_abs_diff_eq!(battin.y, naive.y, epsilon = 1.0e-62);
    }

    #[test]
    fn srp_returns_cannonball_acceleration_in_full_sunlight() {
        let sun_position = Vector3::new(ASTRONOMICAL_UNIT_M, 0.0, 0.0);
        let srp = SolarRadiationPressure::new(
            FixedEphemeris {
                position: sun_position,
            },
            20.0,
            1_000.0,
            1.2,
        )
        .unwrap();
        let vehicle_position = Position3::new(0.0, 7_000_000.0, 0.0);

        let shadow = srp.shadow_factor(vehicle_position, SimTime::ZERO).unwrap();
        let acceleration = srp
            .acceleration_eci_m_s2(vehicle_position, SimTime::ZERO)
            .unwrap();

        let sun_to_vehicle = vehicle_position.vector - sun_position;
        let expected_magnitude = SOLAR_RADIATION_PRESSURE_1_AU_N_M2
            * (ASTRONOMICAL_UNIT_M * ASTRONOMICAL_UNIT_M)
            / sun_to_vehicle.dot(&sun_to_vehicle)
            * 1.2
            * (20.0 / 1_000.0);

        assert_abs_diff_eq!(shadow, 1.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(acceleration.norm(), expected_magnitude, epsilon = 1.0e-15);
        assert!(acceleration.x < 0.0);
        assert!(acceleration.y > 0.0);
    }

    #[test]
    fn srp_conical_shadow_covers_umbra_penumbra_and_full_sun() {
        let sun_position = Vector3::new(ASTRONOMICAL_UNIT_M, 0.0, 0.0);
        let srp = SolarRadiationPressure::new(
            FixedEphemeris {
                position: sun_position,
            },
            12.0,
            600.0,
            1.3,
        )
        .unwrap();
        let umbra = srp
            .shadow_factor(Position3::new(-42_000_000.0, 0.0, 0.0), SimTime::ZERO)
            .unwrap();
        let penumbra_plus = srp
            .shadow_factor(
                Position3::new(-42_000_000.0, 6_450_000.0, 0.0),
                SimTime::ZERO,
            )
            .unwrap();
        let penumbra_minus = srp
            .shadow_factor(
                Position3::new(-42_000_000.0, -6_450_000.0, 0.0),
                SimTime::ZERO,
            )
            .unwrap();
        let full_sun = srp
            .shadow_factor(
                Position3::new(-42_000_000.0, 7_000_000.0, 0.0),
                SimTime::ZERO,
            )
            .unwrap();

        assert_abs_diff_eq!(umbra, 0.0, epsilon = 1.0e-15);
        assert!(penumbra_plus > 0.0 && penumbra_plus < 1.0);
        assert_abs_diff_eq!(penumbra_plus, penumbra_minus, epsilon = 1.0e-15);
        assert_abs_diff_eq!(full_sun, 1.0, epsilon = 1.0e-15);
    }

    #[test]
    fn solar_disk_visible_fraction_matches_equal_disk_segment_area() {
        let visible = solar_disk_visible_fraction(1.0, 1.0, 1.0);
        let overlap = 2.0 * (core::f64::consts::PI / 3.0) - (3.0_f64.sqrt() * 0.5);
        let expected = 1.0 - overlap / core::f64::consts::PI;

        assert_abs_diff_eq!(visible, expected, epsilon = 1.0e-15);
    }

    #[test]
    fn srp_rejects_spacecraft_inside_occulting_body() {
        let sun_position = Vector3::new(ASTRONOMICAL_UNIT_M, 0.0, 0.0);
        let srp = SolarRadiationPressure::new(
            FixedEphemeris {
                position: sun_position,
            },
            10.0,
            500.0,
            1.0,
        )
        .unwrap();

        let err = srp
            .acceleration_eci_m_s2(Position3::new(WGS84_A_M - 1.0, 0.0, 0.0), SimTime::ZERO)
            .unwrap_err();

        assert!(matches!(err, PhysicsError::OutOfEnvelope { .. }));
    }

    #[test]
    fn schwarzschild_correction_matches_circular_orbit_closed_form() {
        let r = 26_560_000.0;
        let circular_speed = (WGS84_MU_M3_S2 / r).sqrt();
        let correction = RelativisticCorrection::wgs84_schwarzschild();

        let acceleration = correction
            .acceleration_eci_m_s2(
                Position3::new(r, 0.0, 0.0),
                Velocity3::new(0.0, circular_speed, 0.0),
            )
            .unwrap();
        let expected_magnitude =
            3.0 * WGS84_MU_M3_S2 * WGS84_MU_M3_S2 / (SPEED_OF_LIGHT_M_S.powi(2) * r.powi(3));

        assert!(acceleration.x > 0.0);
        assert_abs_diff_eq!(acceleration.y, 0.0, epsilon = 1.0e-20);
        assert_abs_diff_eq!(acceleration.z, 0.0, epsilon = 1.0e-20);
        assert_abs_diff_eq!(acceleration.norm(), expected_magnitude, epsilon = 1.0e-22);
        assert!(acceleration.norm() > 2.7e-10 && acceleration.norm() < 2.9e-10);
    }

    #[test]
    fn schwarzschild_correction_rejects_singular_or_nonfinite_state() {
        let correction = RelativisticCorrection::wgs84_schwarzschild();

        let singular = correction
            .acceleration_eci_m_s2(Position3::origin(), Velocity3::zero())
            .unwrap_err();
        let nonfinite = correction
            .acceleration_eci_m_s2(
                Position3::new(7_000_000.0, 0.0, 0.0),
                Velocity3::new(0.0, f64::NAN, 0.0),
            )
            .unwrap_err();

        assert!(matches!(singular, PhysicsError::OutOfEnvelope { .. }));
        assert!(matches!(nonfinite, PhysicsError::InvalidParameter { .. }));
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

    #[test]
    fn tesseral_gravity_degree_two_zonal_matches_existing_j2_byte_for_byte() {
        let tesseral = TesseralGravity::wgs84_j2();
        let j2 = J2Gravity::wgs84();
        let positions = [
            at_x(WGS84_A_M + 100_000.0),
            at_z(WGS84_A_M + 100_000.0),
            Position3::new(7_000_000.0, 1_000_000.0, 500_000.0),
            Position3::new(0.0, 7_000_000.0, 0.0),
        ];

        for position in positions {
            let tesseral_accel = tesseral.gravity_eci_m_s2(position, SimTime::ZERO).unwrap();
            let j2_accel = j2.gravity_eci_m_s2(position, SimTime::ZERO).unwrap();
            for axis in 0..3 {
                assert_eq!(tesseral_accel[axis].to_bits(), j2_accel[axis].to_bits());
            }
        }
    }

    #[test]
    fn tesseral_gravity_loads_normalized_degree_two_pin() {
        let normalized = load_wgs84_normalized_degree_two_fixture();
        assert_eq!(normalized.tide_system().tag(), "tide_free");

        let coefficients = DegreeTwoTesseralCoefficients::from_fully_normalized(normalized)
            .expect("normalized WGS84 degree-2 coefficients convert");
        assert_abs_diff_eq!(coefficients.c20(), -WGS84_J2, epsilon = 1.0e-18);
        assert_eq!(coefficients.c21().to_bits(), 0.0_f64.to_bits());
        assert_eq!(coefficients.s21().to_bits(), 0.0_f64.to_bits());
        assert_eq!(coefficients.c22().to_bits(), 0.0_f64.to_bits());
        assert_eq!(coefficients.s22().to_bits(), 0.0_f64.to_bits());

        let tesseral = TesseralGravity::from_normalized_degree_two(
            WGS84_MU_M3_S2,
            WGS84_A_M,
            normalized,
            2,
            0,
        )
        .expect("normalized coefficient block builds tesseral gravity");
        let j2 = J2Gravity::wgs84();
        let position = Position3::new(7_200_000.0, -1_300_000.0, 900_000.0);
        let from_file = tesseral.gravity_eci_m_s2(position, SimTime::ZERO).unwrap();
        let direct_j2 = j2.gravity_eci_m_s2(position, SimTime::ZERO).unwrap();
        for axis in 0..3 {
            assert_abs_diff_eq!(from_file[axis], direct_j2[axis], epsilon = 1.0e-17);
        }
    }

    #[test]
    fn tesseral_gravity_zero_degree_reduces_to_point_mass() {
        let tesseral = TesseralGravity::new(
            WGS84_MU_M3_S2,
            WGS84_A_M,
            DegreeTwoTesseralCoefficients::zero(TideSystem::TideFree),
            0,
            0,
        )
        .unwrap();
        let point_mass = PointMassGravity::wgs84();
        let position = Position3::new(7_200_000.0, -900_000.0, 300_000.0);

        let tesseral_accel = tesseral.gravity_eci_m_s2(position, SimTime::ZERO).unwrap();
        let point_mass_accel = point_mass
            .gravity_eci_m_s2(position, SimTime::ZERO)
            .unwrap();

        for axis in 0..3 {
            assert_eq!(
                tesseral_accel[axis].to_bits(),
                point_mass_accel[axis].to_bits()
            );
        }
    }

    #[test]
    fn tesseral_gravity_sectoral_term_changes_longitude_acceleration() {
        let coefficients =
            DegreeTwoTesseralCoefficients::new(0.0, 0.0, 0.0, 1.0e-6, 0.0, TideSystem::TideFree)
                .unwrap();
        let tesseral = TesseralGravity::new(WGS84_MU_M3_S2, WGS84_A_M, coefficients, 2, 2).unwrap();
        let point_mass = PointMassGravity::wgs84();
        let position = Position3::new(WGS84_A_M + 400_000.0, 250_000.0, 0.0);

        let tesseral_accel = tesseral.gravity_eci_m_s2(position, SimTime::ZERO).unwrap();
        let point_mass_accel = point_mass
            .gravity_eci_m_s2(position, SimTime::ZERO)
            .unwrap();
        let perturbation = tesseral_accel - point_mass_accel;

        assert!(perturbation.norm() > 1.0e-5);
        assert!(perturbation.x.is_finite());
        assert!(perturbation.y.is_finite());
        assert_abs_diff_eq!(perturbation.z, 0.0, epsilon = 1.0e-14);
    }

    #[test]
    fn tesseral_gravity_tesseral_terms_are_finite_near_pole() {
        let coefficients = DegreeTwoTesseralCoefficients::new(
            -WGS84_J2,
            2.0e-7,
            -3.0e-7,
            1.0e-7,
            -2.0e-7,
            TideSystem::TideFree,
        )
        .unwrap();
        let tesseral = TesseralGravity::new(WGS84_MU_M3_S2, WGS84_A_M, coefficients, 2, 2).unwrap();
        let position = Position3::new(1.0, -2.0, WGS84_A_M + 500_000.0);

        let acceleration = tesseral.gravity_eci_m_s2(position, SimTime::ZERO).unwrap();

        assert!(acceleration.iter().all(|value| value.is_finite()));
        assert!(acceleration.z < 0.0);
    }

    #[test]
    fn tesseral_gravity_rejects_unsupported_degree_order_and_nonfinite_coefficients() {
        assert!(DegreeTwoTesseralCoefficients::from_j2(f64::NAN, TideSystem::TideFree).is_err());
        assert!(
            NormalizedDegreeTwoTesseralCoefficients::new(
                f64::NAN,
                0.0,
                0.0,
                0.0,
                0.0,
                TideSystem::TideFree
            )
            .is_err()
        );
        assert!(TideSystem::from_tag("solid_tide").is_err());
        let coefficients = DegreeTwoTesseralCoefficients::zero(TideSystem::TideFree);

        assert!(matches!(
            TesseralGravity::new(WGS84_MU_M3_S2, WGS84_A_M, coefficients, 1, 0),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            TesseralGravity::new(WGS84_MU_M3_S2, WGS84_A_M, coefficients, 3, 0),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            TesseralGravity::new(WGS84_MU_M3_S2, WGS84_A_M, coefficients, 2, 3),
            Err(PhysicsError::InvalidParameter { .. })
        ));
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

    fn naive_third_body_perturbation(
        vehicle_position: Vector3<f64>,
        body_position: Vector3<f64>,
        mu_m3_s2: f64,
    ) -> Vector3<f64> {
        let relative = body_position - vehicle_position;
        let relative_r2 = relative.dot(&relative);
        let body_r2 = body_position.dot(&body_position);
        let relative_r = relative_r2.sqrt();
        let body_r = body_r2.sqrt();
        mu_m3_s2 * (relative / (relative_r * relative_r2) - body_position / (body_r * body_r2))
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
