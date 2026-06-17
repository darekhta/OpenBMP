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
//! * [`FiniteDifferencePinesGravity`] — a transitional normalized Pines
//!   model that composes point mass with the bounded finite-difference
//!   harmonic-correction oracle.
//! * [`HarmonicSynthesisPlan`] / [`HarmonicSynthesisTier`] — checked
//!   runtime high-degree truncation requests for future EGM2008 tiers.
//! * [`GottliebPotentialSum`] — a scalar-potential recomposition oracle
//!   for cross-checking the Pines coefficient sum.
//!
//! All models implement the [`GravityModel`] trait and report
//! failure via [`crate::error::PhysicsError`] (out-of-envelope, non-finite,
//! invalid parameter). They never panic, never silent-clamp, and
//! never return `NaN`.
//!
//! Determinism: pure arithmetic on `f64`; locked operand order on the
//! J2 sum; no FMA.

#[cfg(not(feature = "std"))]
use alloc::vec::Vec;
use nalgebra::{Matrix3, Vector3};
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

fn point_mass_acceleration_gradient_s2(
    mu_m3_s2: f64,
    position_eci_m: Vector3<f64>,
) -> Result<Matrix3<f64>, PhysicsError> {
    if !mu_m3_s2.is_finite() || mu_m3_s2 <= 0.0 {
        return Err(PhysicsError::InvalidParameter {
            reason: "point-mass gradient µ must be strictly positive and finite",
        });
    }
    let r2 = position_eci_m.dot(&position_eci_m);
    if r2 == 0.0 {
        return Err(PhysicsError::OutOfEnvelope {
            reason: "point-mass gradient is singular at r = 0",
        });
    }
    let r = r2.sqrt();
    let r3 = r * r2;
    let r5 = r3 * r2;
    let gradient = (3.0 * mu_m3_s2 / r5) * (position_eci_m * position_eci_m.transpose())
        - (mu_m3_s2 / r3) * Matrix3::identity();
    if !gradient.iter().all(|value| value.is_finite()) {
        return Err(PhysicsError::NonFinite {
            reason: "point-mass gradient produced non-finite output",
        });
    }
    Ok(gradient)
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

const FULLY_NORMALIZED_SCALE_MAX_DEGREE: usize = 4096;

/// Maximum order accepted by [`HarmonicLongitudeTrigonometry`].
///
/// The bound matches the checked normalization helper envelope and keeps
/// synthesis scratch allocation explicit until the full high-degree kernel
/// chooses a runtime policy.
pub const HARMONIC_LONGITUDE_MAX_ORDER: usize = 4096;

/// Maximum degree accepted by [`PinesLegendreTable`].
///
/// This comfortably covers the planned EGM2008 70/120/360 runtime tiers while
/// keeping scratch allocation bounded before full synthesis lands.
pub const PINES_LEGENDRE_MAX_DEGREE: usize = 720;

/// First runtime-selectable high-degree EGM2008 tier planned for WP-08.1.
pub const HARMONIC_SYNTHESIS_EGM2008_DEGREE_70: usize = 70;

/// Intermediate runtime-selectable high-degree EGM2008 tier planned for WP-08.1.
pub const HARMONIC_SYNTHESIS_EGM2008_DEGREE_120: usize = 120;

/// Full in-repo runtime-selectable EGM2008 tier planned for WP-08.1.
pub const HARMONIC_SYNTHESIS_EGM2008_DEGREE_360: usize = 360;

/// Scale a real fully-normalized harmonic coefficient into the unnormalized
/// associated-Legendre convention used by the current low-degree evaluators.
///
/// The scale is
/// `sqrt((2 - delta_0m) * (2n + 1) * (n - m)! / (n + m)!)`, so
/// `C_nm = Cbar_nm * scale(n, m)` and `S_nm = Sbar_nm * scale(n, m)`.
///
/// # Errors
///
/// Returns [`PhysicsError::InvalidParameter`] if `order > degree` or if the
/// degree exceeds the bounded high-degree envelope used by this checked helper.
/// Returns [`PhysicsError::NonFinite`] if the finite recurrence underflows or
/// overflows into a non-finite scale.
pub fn fully_normalized_to_unnormalized_scale(
    degree: usize,
    order: usize,
) -> Result<f64, PhysicsError> {
    if order > degree {
        return Err(PhysicsError::InvalidParameter {
            reason: "normalization scale order must be <= degree",
        });
    }
    if degree > FULLY_NORMALIZED_SCALE_MAX_DEGREE {
        return Err(PhysicsError::InvalidParameter {
            reason: "normalization scale degree exceeds checked high-degree envelope",
        });
    }
    let degree_u32 = u32::try_from(degree).map_err(|_| PhysicsError::InvalidParameter {
        reason: "normalization scale degree does not fit checked integer range",
    })?;
    let order_u32 = u32::try_from(order).map_err(|_| PhysicsError::InvalidParameter {
        reason: "normalization scale order does not fit checked integer range",
    })?;
    let two_n_plus_one = degree_u32
        .checked_mul(2)
        .and_then(|value| value.checked_add(1))
        .ok_or(PhysicsError::InvalidParameter {
            reason: "normalization scale degree factor overflowed",
        })?;
    let mut scale_squared = f64::from(two_n_plus_one);
    if order != 0 {
        scale_squared *= 2.0;
    }
    let first_denominator = degree_u32 - order_u32 + 1;
    let last_denominator = degree_u32 + order_u32;
    for denominator in first_denominator..=last_denominator {
        scale_squared /= f64::from(denominator);
    }
    let scale = scale_squared.sqrt();
    if !scale.is_finite() || scale <= 0.0 {
        return Err(PhysicsError::NonFinite {
            reason: "normalization scale produced non-finite output",
        });
    }
    Ok(scale)
}

/// Deterministic `cos(mλ)` / `sin(mλ)` table for harmonic synthesis.
///
/// Real spherical-harmonic gravity uses longitude terms in the form
/// `Cbar_nm cos(mλ) + Sbar_nm sin(mλ)`. This table computes the complete
/// order range `0..=max_order` once using a locked complex-multiply recurrence:
///
/// ```text
/// cos(mλ) = cos(λ) cos((m-1)λ) - sin(λ) sin((m-1)λ)
/// sin(mλ) = sin(λ) cos((m-1)λ) + cos(λ) sin((m-1)λ)
/// ```
///
/// It is a small shared substrate for the future Pines/Gottlieb kernels; it
/// does not evaluate a gravity field by itself.
#[derive(Clone, Debug, PartialEq)]
pub struct HarmonicLongitudeTrigonometry {
    longitude_rad: f64,
    cosine_by_order: Vec<f64>,
    sine_by_order: Vec<f64>,
}

impl HarmonicLongitudeTrigonometry {
    /// Build a longitude trigonometry table through `max_order`.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if the longitude is
    /// non-finite, the order exceeds [`HARMONIC_LONGITUDE_MAX_ORDER`], or the
    /// table length overflows. Returns [`PhysicsError::NonFinite`] if the
    /// recurrence produces a non-finite value.
    pub fn new(longitude_rad: f64, max_order: usize) -> Result<Self, PhysicsError> {
        if !longitude_rad.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "harmonic longitude must be finite",
            });
        }
        if max_order > HARMONIC_LONGITUDE_MAX_ORDER {
            return Err(PhysicsError::InvalidParameter {
                reason: "harmonic longitude order exceeds checked high-order envelope",
            });
        }
        let len = max_order
            .checked_add(1)
            .ok_or(PhysicsError::InvalidParameter {
                reason: "harmonic longitude table length overflowed",
            })?;
        let mut cosine_by_order = Vec::with_capacity(len);
        let mut sine_by_order = Vec::with_capacity(len);
        cosine_by_order.push(1.0);
        sine_by_order.push(0.0);

        if max_order > 0 {
            let (sin_lambda, cos_lambda) = longitude_rad.sin_cos();
            cosine_by_order.push(cos_lambda);
            sine_by_order.push(sin_lambda);
            for order in 2..=max_order {
                let previous_cosine = cosine_by_order[order - 1];
                let previous_sine = sine_by_order[order - 1];
                let cosine = cos_lambda * previous_cosine - sin_lambda * previous_sine;
                let sine = sin_lambda * previous_cosine + cos_lambda * previous_sine;
                if !cosine.is_finite() || !sine.is_finite() {
                    return Err(PhysicsError::NonFinite {
                        reason: "harmonic longitude recurrence produced non-finite output",
                    });
                }
                cosine_by_order.push(cosine);
                sine_by_order.push(sine);
            }
        }

        Ok(Self {
            longitude_rad,
            cosine_by_order,
            sine_by_order,
        })
    }

    /// Input longitude `λ`, in radians.
    #[must_use]
    pub const fn longitude_rad(&self) -> f64 {
        self.longitude_rad
    }

    /// Maximum order available in this table.
    #[must_use]
    pub fn max_order(&self) -> usize {
        self.cosine_by_order.len() - 1
    }

    /// Return `(cos(mλ), sin(mλ))` for `order = m`.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if `order` is outside the
    /// table envelope.
    pub fn harmonic(&self, order: usize) -> Result<(f64, f64), PhysicsError> {
        let cosine = self
            .cosine_by_order
            .get(order)
            .ok_or(PhysicsError::InvalidParameter {
                reason: "harmonic longitude lookup outside table envelope",
            })?;
        let sine = self
            .sine_by_order
            .get(order)
            .ok_or(PhysicsError::InvalidParameter {
                reason: "harmonic longitude lookup outside table envelope",
            })?;
        Ok((*cosine, *sine))
    }

    /// Return `cos(mλ)` for `order = m`.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if `order` is outside the
    /// table envelope.
    pub fn cosine(&self, order: usize) -> Result<f64, PhysicsError> {
        Ok(self.harmonic(order)?.0)
    }

    /// Return `sin(mλ)` for `order = m`.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if `order` is outside the
    /// table envelope.
    pub fn sine(&self, order: usize) -> Result<f64, PhysicsError> {
        Ok(self.harmonic(order)?.1)
    }
}

/// Singularity-free Pines longitude polynomials from direction cosines.
///
/// The Pines formulation uses `s = x/r` and `t = y/r` instead of geodetic or
/// spherical longitude. This table stores the real and imaginary components of
/// `(s + i t)^m`, with `m = 0..=max_order`, using the locked recurrence:
///
/// ```text
/// r_m = s r_{m-1} - t i_{m-1}
/// i_m = s i_{m-1} + t r_{m-1}
/// ```
///
/// At the pole (`s = t = 0`), all positive-order terms are exactly zero rather
/// than singular.
#[derive(Clone, Debug, PartialEq)]
pub struct PinesLongitudePolynomials {
    s: f64,
    t: f64,
    real_by_order: Vec<f64>,
    imaginary_by_order: Vec<f64>,
}

impl PinesLongitudePolynomials {
    /// Build Pines longitude polynomials through `max_order`.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if either direction cosine is
    /// non-finite, if `s² + t²` is outside the unit sphere beyond round-off
    /// tolerance, if `max_order` exceeds [`HARMONIC_LONGITUDE_MAX_ORDER`], or if
    /// the table length overflows. Returns [`PhysicsError::NonFinite`] if the
    /// recurrence produces a non-finite value.
    pub fn new(s: f64, t: f64, max_order: usize) -> Result<Self, PhysicsError> {
        if !s.is_finite() || !t.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "Pines longitude direction cosines must be finite",
            });
        }
        let horizontal_norm2 = s * s + t * t;
        if horizontal_norm2 > 1.0 + 16.0 * f64::EPSILON {
            return Err(PhysicsError::InvalidParameter {
                reason: "Pines longitude direction cosines must satisfy s^2 + t^2 <= 1",
            });
        }
        if max_order > HARMONIC_LONGITUDE_MAX_ORDER {
            return Err(PhysicsError::InvalidParameter {
                reason: "Pines longitude order exceeds checked high-order envelope",
            });
        }
        let len = max_order
            .checked_add(1)
            .ok_or(PhysicsError::InvalidParameter {
                reason: "Pines longitude table length overflowed",
            })?;
        let mut real_by_order = Vec::with_capacity(len);
        let mut imaginary_by_order = Vec::with_capacity(len);
        real_by_order.push(1.0);
        imaginary_by_order.push(0.0);

        for order in 1..=max_order {
            let previous_real = real_by_order[order - 1];
            let previous_imaginary = imaginary_by_order[order - 1];
            let real = s * previous_real - t * previous_imaginary;
            let imaginary = s * previous_imaginary + t * previous_real;
            if !real.is_finite() || !imaginary.is_finite() {
                return Err(PhysicsError::NonFinite {
                    reason: "Pines longitude recurrence produced non-finite output",
                });
            }
            real_by_order.push(real);
            imaginary_by_order.push(imaginary);
        }

        Ok(Self {
            s,
            t,
            real_by_order,
            imaginary_by_order,
        })
    }

    /// Direction cosine `s = x/r`.
    #[must_use]
    pub const fn s(&self) -> f64 {
        self.s
    }

    /// Direction cosine `t = y/r`.
    #[must_use]
    pub const fn t(&self) -> f64 {
        self.t
    }

    /// Maximum order available in this table.
    #[must_use]
    pub fn max_order(&self) -> usize {
        self.real_by_order.len() - 1
    }

    /// Return `(r_m, i_m) = (Re((s + i t)^m), Im((s + i t)^m))`.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if `order` is outside the
    /// table envelope.
    pub fn polynomial(&self, order: usize) -> Result<(f64, f64), PhysicsError> {
        let real = self
            .real_by_order
            .get(order)
            .ok_or(PhysicsError::InvalidParameter {
                reason: "Pines longitude lookup outside table envelope",
            })?;
        let imaginary =
            self.imaginary_by_order
                .get(order)
                .ok_or(PhysicsError::InvalidParameter {
                    reason: "Pines longitude lookup outside table envelope",
                })?;
        Ok((*real, *imaginary))
    }

    /// Return `r_m = Re((s + i t)^m)`.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if `order` is outside the
    /// table envelope.
    pub fn real(&self, order: usize) -> Result<f64, PhysicsError> {
        Ok(self.polynomial(order)?.0)
    }

    /// Return `i_m = Im((s + i t)^m)`.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if `order` is outside the
    /// table envelope.
    pub fn imaginary(&self, order: usize) -> Result<f64, PhysicsError> {
        Ok(self.polynomial(order)?.1)
    }
}

/// Body-fixed geometry for normalized Pines harmonic synthesis.
///
/// The position is expressed in the body-fixed frame associated with the
/// harmonic coefficients. The cached direction cosines feed the
/// singularity-free Pines Legendre and longitude recurrences; the cached
/// radius ratio feeds the radial attenuation `(R/r)^n`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PinesSynthesisPoint {
    radius_m: f64,
    reference_radius_m: f64,
    reference_radius_over_radius: f64,
    s: f64,
    t: f64,
    u: f64,
}

impl PinesSynthesisPoint {
    /// Build a synthesis point from a body-fixed Cartesian position.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if any input is non-finite or
    /// if `reference_radius_m` is not strictly positive. Returns
    /// [`PhysicsError::OutOfEnvelope`] at the central-body singularity.
    pub fn new(
        position_body_fixed_m: Vector3<f64>,
        reference_radius_m: f64,
    ) -> Result<Self, PhysicsError> {
        if !position_body_fixed_m.iter().all(|value| value.is_finite()) {
            return Err(PhysicsError::InvalidParameter {
                reason: "Pines synthesis position must be finite",
            });
        }
        if !reference_radius_m.is_finite() || reference_radius_m <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "Pines synthesis reference radius must be strictly positive and finite",
            });
        }
        let radius2 = position_body_fixed_m.dot(&position_body_fixed_m);
        if radius2 == 0.0 {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "Pines synthesis is singular at r = 0",
            });
        }
        let radius_m = radius2.sqrt();
        let inv_radius = 1.0 / radius_m;
        let reference_radius_over_radius = reference_radius_m * inv_radius;
        if !reference_radius_over_radius.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "Pines synthesis radial ratio produced non-finite output",
            });
        }
        Ok(Self {
            radius_m,
            reference_radius_m,
            reference_radius_over_radius,
            s: position_body_fixed_m.x * inv_radius,
            t: position_body_fixed_m.y * inv_radius,
            u: position_body_fixed_m.z * inv_radius,
        })
    }

    /// Radius `r`, in metres.
    #[must_use]
    pub const fn radius_m(&self) -> f64 {
        self.radius_m
    }

    /// Harmonic reference radius `R`, in metres.
    #[must_use]
    pub const fn reference_radius_m(&self) -> f64 {
        self.reference_radius_m
    }

    /// Radial attenuation base `R/r`.
    #[must_use]
    pub const fn reference_radius_over_radius(&self) -> f64 {
        self.reference_radius_over_radius
    }

    /// Direction cosine `s = x/r`.
    #[must_use]
    pub const fn s(&self) -> f64 {
        self.s
    }

    /// Direction cosine `t = y/r`.
    #[must_use]
    pub const fn t(&self) -> f64 {
        self.t
    }

    /// Direction cosine `u = z/r`.
    #[must_use]
    pub const fn u(&self) -> f64 {
        self.u
    }
}

/// Checked harmonic degree/order truncation request.
///
/// The type validates only the mathematical truncation shape (`order <= degree`)
/// and optional data-envelope bounds. It does not imply that a particular
/// evaluator supports the requested degree; model constructors still enforce
/// their own implementation limits.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct HarmonicTruncation {
    degree: usize,
    order: usize,
}

impl HarmonicTruncation {
    /// Construct a truncation request.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] when `order > degree`.
    pub fn new(degree: usize, order: usize) -> Result<Self, PhysicsError> {
        if order > degree {
            return Err(PhysicsError::InvalidParameter {
                reason: "harmonic truncation order must be <= degree",
            });
        }
        Ok(Self { degree, order })
    }

    /// Construct a truncation request constrained by a declared data envelope.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] when `order > degree`, the
    /// envelope itself is invalid, or the request exceeds `max_degree` /
    /// `max_order`.
    pub fn within_envelope(
        degree: usize,
        order: usize,
        max_degree: usize,
        max_order: usize,
    ) -> Result<Self, PhysicsError> {
        let truncation = Self::new(degree, order)?;
        if max_order > max_degree {
            return Err(PhysicsError::InvalidParameter {
                reason: "harmonic truncation envelope max_order must be <= max_degree",
            });
        }
        if degree > max_degree || order > max_order {
            return Err(PhysicsError::InvalidParameter {
                reason: "harmonic truncation outside declared envelope",
            });
        }
        Ok(truncation)
    }

    /// Construct a truncation request constrained by a normalized coefficient
    /// field.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] when the request is not a
    /// valid truncation or exceeds the field envelope.
    pub fn for_normalized_field(
        field: &NormalizedHarmonicField,
        degree: usize,
        order: usize,
    ) -> Result<Self, PhysicsError> {
        Self::within_envelope(degree, order, field.max_degree(), field.max_order())
    }

    /// Inclusive maximum harmonic degree.
    #[must_use]
    pub const fn degree(&self) -> usize {
        self.degree
    }

    /// Inclusive maximum harmonic order.
    #[must_use]
    pub const fn order(&self) -> usize {
        self.order
    }
}

/// Runtime-selectable high-degree harmonic synthesis tier.
///
/// These named tiers mirror the WP-08.1 EGM2008 rollout plan. Selecting a tier
/// does not imply that EGM2008 coefficients are present; callers must resolve a
/// tier against a concrete [`NormalizedHarmonicField`] through
/// [`HarmonicSynthesisPlan`] so missing data envelopes fail closed.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum HarmonicSynthesisTier {
    /// EGM2008 degree/order 70 truncation tier.
    Egm2008Degree70,
    /// EGM2008 degree/order 120 truncation tier.
    Egm2008Degree120,
    /// EGM2008 degree/order 360 truncation tier.
    Egm2008Degree360,
}

impl HarmonicSynthesisTier {
    /// Inclusive maximum harmonic degree for this tier.
    #[must_use]
    pub const fn degree(&self) -> usize {
        match self {
            Self::Egm2008Degree70 => HARMONIC_SYNTHESIS_EGM2008_DEGREE_70,
            Self::Egm2008Degree120 => HARMONIC_SYNTHESIS_EGM2008_DEGREE_120,
            Self::Egm2008Degree360 => HARMONIC_SYNTHESIS_EGM2008_DEGREE_360,
        }
    }

    /// Inclusive maximum harmonic order for this tier.
    #[must_use]
    pub const fn order(&self) -> usize {
        self.degree()
    }

    /// Degree/order truncation requested by this tier.
    pub fn truncation(&self) -> Result<HarmonicTruncation, PhysicsError> {
        HarmonicTruncation::new(self.degree(), self.order())
    }
}

/// Checked harmonic synthesis plan resolved against a coefficient field.
///
/// The plan is a small guardrail before the full analytic high-degree Pines /
/// Gottlieb kernels land: it proves that a runtime request fits the loaded
/// coefficient envelope and the bounded scratch tables used by the current
/// Pines scalar and finite-difference paths.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct HarmonicSynthesisPlan {
    truncation: HarmonicTruncation,
    tier: Option<HarmonicSynthesisTier>,
}

impl HarmonicSynthesisPlan {
    /// Resolve a custom degree/order truncation against a normalized field and
    /// the bounded Pines synthesis scratch envelopes.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if the truncation exceeds
    /// the field envelope, the Pines Legendre degree cap, or the longitude
    /// recurrence order cap.
    pub fn for_normalized_field(
        field: &NormalizedHarmonicField,
        truncation: HarmonicTruncation,
    ) -> Result<Self, PhysicsError> {
        let truncation = checked_pines_synthesis_truncation(field, truncation)?;
        Ok(Self {
            truncation,
            tier: None,
        })
    }

    /// Resolve a named EGM2008 tier against a normalized field and the bounded
    /// Pines synthesis scratch envelopes.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if the loaded field does not
    /// cover the requested tier or if the tier exceeds the checked scratch
    /// limits.
    pub fn egm2008_tier_for_normalized_field(
        field: &NormalizedHarmonicField,
        tier: HarmonicSynthesisTier,
    ) -> Result<Self, PhysicsError> {
        let truncation = checked_pines_synthesis_truncation(field, tier.truncation()?)?;
        Ok(Self {
            truncation,
            tier: Some(tier),
        })
    }

    /// Resolved truncation.
    #[must_use]
    pub const fn truncation(&self) -> HarmonicTruncation {
        self.truncation
    }

    /// Named tier, if this plan was resolved from one.
    #[must_use]
    pub const fn tier(&self) -> Option<HarmonicSynthesisTier> {
        self.tier
    }

    /// Inclusive maximum harmonic degree.
    #[must_use]
    pub const fn degree(&self) -> usize {
        self.truncation.degree()
    }

    /// Inclusive maximum harmonic order.
    #[must_use]
    pub const fn order(&self) -> usize {
        self.truncation.order()
    }
}

/// Holmes-Featherstone normalized Legendre table for the Pines kernel.
///
/// The table stores the derived Pines `A[n,m](u)` functions for
/// `0 <= n <= max_degree` and `0 <= m <= min(n, max_order)`, where `u = z/r`
/// is the Cartesian direction cosine. It follows the recurrence documented in
/// `docs/parity/08-environment-gravity-and-frames.md` and is intended as a
/// reusable substrate for the future singularity-free harmonic synthesis
/// kernel. It does not assemble gravity acceleration by itself.
#[derive(Clone, Debug, PartialEq)]
pub struct PinesLegendreTable {
    u: f64,
    max_degree: usize,
    max_order: usize,
    values: Vec<f64>,
}

impl PinesLegendreTable {
    /// Build a normalized Pines `A[n,m](u)` table.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if `u` is non-finite or
    /// outside `[-1, 1]`, if `max_order > max_degree`, or if `max_degree`
    /// exceeds [`PINES_LEGENDRE_MAX_DEGREE`]. Returns
    /// [`PhysicsError::NonFinite`] if the recurrence produces a non-finite
    /// value.
    pub fn new(u: f64, max_degree: usize, max_order: usize) -> Result<Self, PhysicsError> {
        if !u.is_finite() || !(-1.0..=1.0).contains(&u) {
            return Err(PhysicsError::InvalidParameter {
                reason: "Pines Legendre direction cosine must be finite and in [-1, 1]",
            });
        }
        if max_degree > PINES_LEGENDRE_MAX_DEGREE {
            return Err(PhysicsError::InvalidParameter {
                reason: "Pines Legendre degree exceeds checked high-degree envelope",
            });
        }
        HarmonicTruncation::new(max_degree, max_order)?;
        let storage_len = normalized_harmonic_storage_len(max_degree, max_order).ok_or(
            PhysicsError::InvalidParameter {
                reason: "Pines Legendre table storage size overflowed",
            },
        )?;
        let mut values = Vec::with_capacity(storage_len);
        values.resize(storage_len, 0.0);

        set_pines_legendre_value(&mut values, max_order, 0, 0, 1.0)?;
        if max_degree >= 1 {
            let sqrt_3 = 3.0_f64.sqrt();
            set_pines_legendre_value(&mut values, max_order, 1, 0, u * sqrt_3)?;
            if max_order >= 1 {
                set_pines_legendre_value(&mut values, max_order, 1, 1, sqrt_3)?;
            }
        }

        for order in 2..=max_degree.min(max_order) {
            let order_f64 = harmonic_index_to_f64(order)?;
            let factor = ((2.0 * order_f64 + 1.0) / (2.0 * order_f64)).sqrt();
            let previous =
                pines_legendre_value_from_slice(&values, max_order, order - 1, order - 1)?;
            set_pines_legendre_value(&mut values, max_order, order, order, factor * previous)?;
        }

        for order in 0..=max_order {
            let start_degree = if order == 0 { 2 } else { order + 1 };
            if start_degree > max_degree {
                continue;
            }
            for degree in start_degree..=max_degree {
                let g = pines_legendre_g_factor(degree, order)?;
                let previous =
                    pines_legendre_value_from_slice(&values, max_order, degree - 1, order)?;
                let h_previous = if degree == order + 1 {
                    0.0
                } else {
                    pines_legendre_h_factor(degree, order)?
                        * pines_legendre_value_from_slice(&values, max_order, degree - 2, order)?
                };
                let value = u * g * previous - h_previous;
                if !value.is_finite() {
                    return Err(PhysicsError::NonFinite {
                        reason: "Pines Legendre recurrence produced non-finite output",
                    });
                }
                set_pines_legendre_value(&mut values, max_order, degree, order, value)?;
            }
        }

        Ok(Self {
            u,
            max_degree,
            max_order,
            values,
        })
    }

    /// Direction cosine `u = z/r`.
    #[must_use]
    pub const fn u(&self) -> f64 {
        self.u
    }

    /// Maximum tabulated harmonic degree.
    #[must_use]
    pub const fn max_degree(&self) -> usize {
        self.max_degree
    }

    /// Maximum tabulated harmonic order.
    #[must_use]
    pub const fn max_order(&self) -> usize {
        self.max_order
    }

    /// Number of packed table slots.
    #[must_use]
    pub fn storage_len(&self) -> usize {
        self.values.len()
    }

    /// Return `A[n,m](u)` for a valid degree/order pair.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if the request is outside the
    /// table envelope or has `order > degree`.
    pub fn value(&self, degree: usize, order: usize) -> Result<f64, PhysicsError> {
        HarmonicTruncation::within_envelope(degree, order, self.max_degree, self.max_order)?;
        pines_legendre_value_from_slice(&self.values, self.max_order, degree, order)
    }
}

fn set_pines_legendre_value(
    values: &mut [f64],
    max_order: usize,
    degree: usize,
    order: usize,
    value: f64,
) -> Result<(), PhysicsError> {
    let index = normalized_harmonic_index(max_order, degree, order).ok_or(
        PhysicsError::InvalidParameter {
            reason: "Pines Legendre table index outside storage envelope",
        },
    )?;
    let slot = values
        .get_mut(index)
        .ok_or(PhysicsError::InvalidParameter {
            reason: "Pines Legendre table index exceeded storage length",
        })?;
    *slot = value;
    Ok(())
}

fn pines_legendre_value_from_slice(
    values: &[f64],
    max_order: usize,
    degree: usize,
    order: usize,
) -> Result<f64, PhysicsError> {
    let index = normalized_harmonic_index(max_order, degree, order).ok_or(
        PhysicsError::InvalidParameter {
            reason: "Pines Legendre table lookup outside storage envelope",
        },
    )?;
    values
        .get(index)
        .copied()
        .ok_or(PhysicsError::InvalidParameter {
            reason: "Pines Legendre table lookup exceeded storage length",
        })
}

fn pines_legendre_g_factor(degree: usize, order: usize) -> Result<f64, PhysicsError> {
    let n = harmonic_index_to_f64(degree)?;
    let m = harmonic_index_to_f64(order)?;
    Ok(((2.0 * n + 1.0) * (2.0 * n - 1.0) / ((n - m) * (n + m))).sqrt())
}

fn pines_legendre_h_factor(degree: usize, order: usize) -> Result<f64, PhysicsError> {
    let n = harmonic_index_to_f64(degree)?;
    let m = harmonic_index_to_f64(order)?;
    Ok(
        ((2.0 * n + 1.0) * (n + m - 1.0) * (n - m - 1.0) / ((2.0 * n - 3.0) * (n + m) * (n - m)))
            .sqrt(),
    )
}

fn harmonic_index_to_f64(value: usize) -> Result<f64, PhysicsError> {
    let value_u32 = u32::try_from(value).map_err(|_| PhysicsError::InvalidParameter {
        reason: "harmonic index does not fit checked integer range",
    })?;
    Ok(f64::from(value_u32))
}

/// Deterministic normalized Pines scalar-potential correction sum.
///
/// This is the dimensionless coefficient sum
/// `Σ (R/r)^n A_nm(u) (Cbar_nm r_m + Sbar_nm i_m)` over a validated
/// truncation. Multiplying by `µ/r` yields the corresponding scalar potential
/// correction. This is kernel substrate only; acceleration and gradients remain
/// separate synthesis work.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PinesPotentialSum {
    truncation: HarmonicTruncation,
    dimensionless_correction: f64,
    term_count: usize,
}

impl PinesPotentialSum {
    /// Truncation used to produce this sum.
    #[must_use]
    pub const fn truncation(&self) -> HarmonicTruncation {
        self.truncation
    }

    /// Dimensionless normalized scalar-potential correction.
    #[must_use]
    pub const fn dimensionless_correction(&self) -> f64 {
        self.dimensionless_correction
    }

    /// Number of deterministic `(n, m)` slots visited by the summation.
    #[must_use]
    pub const fn term_count(&self) -> usize {
        self.term_count
    }
}

/// Deterministic normalized Gottlieb-style scalar-potential correction sum.
///
/// This recomposes the same fully-normalized coefficient field through ordinary
/// longitude trigonometry and an explicit horizontal-power term
/// `cos(phi)^m`, rather than through Pines direction-cosine longitude
/// polynomials. It is a scalar-potential cross-check substrate only; the full
/// normalized Gottlieb acceleration-gradient oracle remains separate work.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct GottliebPotentialSum {
    truncation: HarmonicTruncation,
    dimensionless_correction: f64,
    term_count: usize,
}

impl GottliebPotentialSum {
    /// Truncation used to produce this sum.
    #[must_use]
    pub const fn truncation(&self) -> HarmonicTruncation {
        self.truncation
    }

    /// Dimensionless normalized scalar-potential correction.
    #[must_use]
    pub const fn dimensionless_correction(&self) -> f64 {
        self.dimensionless_correction
    }

    /// Number of deterministic `(n, m)` slots visited by the summation.
    #[must_use]
    pub const fn term_count(&self) -> usize {
        self.term_count
    }
}

/// One fully-normalized spherical-harmonic coefficient pair.
///
/// `degree` and `order` identify `(n, m)`. The cosine coefficient is `Cbar_nm`;
/// the sine coefficient is `Sbar_nm`. Zonal `m = 0` entries must carry
/// `Sbar = 0` because there is no sine term for order zero.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct NormalizedHarmonicCoefficient {
    degree: usize,
    order: usize,
    cbar: f64,
    sbar: f64,
}

impl NormalizedHarmonicCoefficient {
    /// Construct a finite fully-normalized coefficient pair.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if `order > degree`, either
    /// coefficient is non-finite, or an order-zero sine coefficient is nonzero.
    pub fn new(degree: usize, order: usize, cbar: f64, sbar: f64) -> Result<Self, PhysicsError> {
        let truncation = HarmonicTruncation::new(degree, order)?;
        if !cbar.is_finite() || !sbar.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "normalized harmonic coefficients must be finite",
            });
        }
        if order == 0 && sbar != 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "order-zero harmonic sine coefficient must be zero",
            });
        }
        Ok(Self {
            degree: truncation.degree,
            order: truncation.order,
            cbar,
            sbar,
        })
    }

    /// Harmonic degree `n`.
    #[must_use]
    pub const fn degree(&self) -> usize {
        self.degree
    }

    /// Harmonic order `m`.
    #[must_use]
    pub const fn order(&self) -> usize {
        self.order
    }

    /// Fully-normalized cosine coefficient `Cbar_nm`.
    #[must_use]
    pub const fn cbar(&self) -> f64 {
        self.cbar
    }

    /// Fully-normalized sine coefficient `Sbar_nm`.
    #[must_use]
    pub const fn sbar(&self) -> f64 {
        self.sbar
    }
}

/// Fully-normalized spherical-harmonic coefficient field.
///
/// The field stores `Cbar/Sbar` pairs in deterministic packed `(n, m)` row
/// order for `0 <= n <= max_degree` and `0 <= m <= min(n, max_order)`. Missing
/// entries inside the declared envelope default to zero. Duplicate,
/// out-of-range, and non-finite coefficients are rejected at construction so
/// ingestion paths fail closed before force-model evaluation.
#[derive(Clone, Debug, PartialEq)]
pub struct NormalizedHarmonicField {
    max_degree: usize,
    max_order: usize,
    tide_system: TideSystem,
    coefficients: Vec<(f64, f64)>,
    coefficient_count: usize,
}

/// Iterator over every valid normalized harmonic coefficient slot in row order.
///
/// The iterator yields `(degree, order)` pairs with degree increasing first and
/// order increasing inside each degree, constrained to
/// `0 <= order <= min(degree, max_order)`. Missing coefficients inside the
/// field envelope are yielded as zero-valued [`NormalizedHarmonicCoefficient`]
/// entries so synthesis kernels can consume a complete deterministic stream.
#[derive(Clone, Debug)]
pub struct NormalizedHarmonicFieldIter<'a> {
    field: &'a NormalizedHarmonicField,
    degree: usize,
    order: usize,
    remaining: usize,
}

/// Fully-normalized ICGEM `.gfc` coefficients plus source constants.
///
/// ICGEM/NGA-style gravity files carry both normalized coefficient rows and the
/// source `µ`, reference radius, and declared maximum degree needed to build a
/// deterministic runtime force model. This parsed result keeps that metadata
/// attached to the truncated [`NormalizedHarmonicField`] instead of requiring
/// callers to duplicate constants out-of-band.
#[cfg(feature = "std")]
#[derive(Clone, Debug, PartialEq)]
pub struct IcgemGfcNormalizedField {
    field: NormalizedHarmonicField,
    gravity_constant_m3_s2: f64,
    reference_radius_m: f64,
    source_max_degree: usize,
}

#[cfg(feature = "std")]
impl IcgemGfcNormalizedField {
    /// Construct an ICGEM parsed field result.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if source constants are not
    /// strictly positive and finite or if the field envelope exceeds the source
    /// degree declared by the file header.
    pub fn new(
        field: NormalizedHarmonicField,
        gravity_constant_m3_s2: f64,
        reference_radius_m: f64,
        source_max_degree: usize,
    ) -> Result<Self, PhysicsError> {
        if !gravity_constant_m3_s2.is_finite() || gravity_constant_m3_s2 <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "ICGEM GFC earth_gravity_constant must be strictly positive and finite",
            });
        }
        if !reference_radius_m.is_finite() || reference_radius_m <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "ICGEM GFC radius must be strictly positive and finite",
            });
        }
        if field.max_degree() > source_max_degree {
            return Err(PhysicsError::InvalidParameter {
                reason: "ICGEM GFC requested max_degree exceeds source max_degree",
            });
        }
        Ok(Self {
            field,
            gravity_constant_m3_s2,
            reference_radius_m,
            source_max_degree,
        })
    }

    /// Truncated normalized harmonic field.
    #[must_use]
    pub const fn field(&self) -> &NormalizedHarmonicField {
        &self.field
    }

    /// Consume the parsed result and return its truncated field.
    #[must_use]
    pub fn into_field(self) -> NormalizedHarmonicField {
        self.field
    }

    /// Source gravitational parameter in m^3/s^2.
    #[must_use]
    pub const fn gravity_constant_m3_s2(&self) -> f64 {
        self.gravity_constant_m3_s2
    }

    /// Source harmonic reference radius in metres.
    #[must_use]
    pub const fn reference_radius_m(&self) -> f64 {
        self.reference_radius_m
    }

    /// Maximum harmonic degree declared by the source file header.
    #[must_use]
    pub const fn source_max_degree(&self) -> usize {
        self.source_max_degree
    }
}

impl Iterator for NormalizedHarmonicFieldIter<'_> {
    type Item = NormalizedHarmonicCoefficient;

    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining == 0 || self.degree > self.field.max_degree {
            return None;
        }
        let degree = self.degree;
        let order = self.order;
        let (cbar, sbar) = self.field.coefficient(degree, order).ok()?;
        self.remaining -= 1;
        if order < degree.min(self.field.max_order) {
            self.order += 1;
        } else {
            self.degree += 1;
            self.order = 0;
        }
        Some(NormalizedHarmonicCoefficient {
            degree,
            order,
            cbar,
            sbar,
        })
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.remaining, Some(self.remaining))
    }
}

impl ExactSizeIterator for NormalizedHarmonicFieldIter<'_> {}

impl NormalizedHarmonicField {
    /// Construct a fully-normalized coefficient field.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if `max_order > max_degree`,
    /// the packed storage size overflows `usize`, any coefficient falls outside
    /// the declared degree/order envelope, or a duplicate `(degree, order)` pair
    /// is supplied.
    pub fn new(
        max_degree: usize,
        max_order: usize,
        tide_system: TideSystem,
        coefficients: impl IntoIterator<Item = NormalizedHarmonicCoefficient>,
    ) -> Result<Self, PhysicsError> {
        if max_order > max_degree {
            return Err(PhysicsError::InvalidParameter {
                reason: "normalized harmonic max_order must be <= max_degree",
            });
        }
        let storage_len = normalized_harmonic_storage_len(max_degree, max_order).ok_or(
            PhysicsError::InvalidParameter {
                reason: "normalized harmonic field storage size overflowed",
            },
        )?;
        let mut values = Vec::with_capacity(storage_len);
        values.resize(storage_len, (0.0, 0.0));
        let mut seen = Vec::with_capacity(storage_len);
        seen.resize(storage_len, false);

        let mut coefficient_count = 0;
        for coefficient in coefficients {
            if coefficient.degree > max_degree || coefficient.order > max_order {
                return Err(PhysicsError::InvalidParameter {
                    reason: "normalized harmonic coefficient outside declared field envelope",
                });
            }
            let index = normalized_harmonic_index(max_order, coefficient.degree, coefficient.order)
                .ok_or(PhysicsError::InvalidParameter {
                    reason: "normalized harmonic coefficient order must be <= degree",
                })?;
            if seen[index] {
                return Err(PhysicsError::InvalidParameter {
                    reason: "duplicate normalized harmonic coefficient",
                });
            }
            seen[index] = true;
            values[index] = (coefficient.cbar, coefficient.sbar);
            coefficient_count += 1;
        }

        Ok(Self {
            max_degree,
            max_order,
            tide_system,
            coefficients: values,
            coefficient_count,
        })
    }

    /// Construct a fully-normalized coefficient field from an OpenBMP TOML
    /// fixture.
    ///
    /// This parser is available only with the `std` feature so the HAL/no-std
    /// physics core does not carry a TOML dependency. It accepts the current
    /// provenance-pinned fixture schemas:
    ///
    /// * `openbmp.gravity.normalized-degree2.v1`
    /// * `openbmp.gravity.normalized-zonal.v1`
    /// * `openbmp.gravity.normalized-field.v1`
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] when the TOML cannot be
    /// parsed, required metadata is missing, the normalization is not
    /// `fully_normalized`, the schema is unsupported, or any coefficient fails
    /// the same validation as [`Self::new`].
    #[cfg(feature = "std")]
    pub fn from_normalized_toml_str(input: &str) -> Result<Self, PhysicsError> {
        let value =
            toml::from_str::<toml::Value>(input).map_err(|_| PhysicsError::InvalidParameter {
                reason: "normalized harmonic TOML must parse",
            })?;
        let table = value.as_table().ok_or(PhysicsError::InvalidParameter {
            reason: "normalized harmonic TOML root must be a table",
        })?;
        let _dataset_id = normalized_harmonic_toml_str(table, "dataset_id")?;
        let schema_version = normalized_harmonic_toml_str(table, "schema_version")?;
        let normalization = normalized_harmonic_toml_str(table, "normalization")?;
        if normalization != "fully_normalized" {
            return Err(PhysicsError::InvalidParameter {
                reason: "normalized harmonic TOML normalization must be fully_normalized",
            });
        }
        let tide_system =
            TideSystem::from_tag(normalized_harmonic_toml_str(table, "tide_system")?)?;

        match schema_version {
            "openbmp.gravity.normalized-degree2.v1" => {
                Self::from_normalized_degree2_toml_table(table, tide_system)
            }
            "openbmp.gravity.normalized-zonal.v1" => {
                Self::from_normalized_general_toml_table(table, tide_system, true)
            }
            "openbmp.gravity.normalized-field.v1" => {
                Self::from_normalized_general_toml_table(table, tide_system, false)
            }
            _ => Err(PhysicsError::InvalidParameter {
                reason: "unsupported normalized harmonic TOML schema_version",
            }),
        }
    }

    /// Construct a fully-normalized coefficient field from an ICGEM `.gfc`
    /// coefficient block.
    ///
    /// This parser covers the static `gfc n m Cbar Sbar ...` line shape used
    /// by ICGEM/NGA-style gravity model files. It requires an
    /// `end_of_head` marker, a `norm fully_normalized` header, and a supported
    /// `tide_system` tag. Coefficients outside `max_degree` / `max_order` are
    /// ignored after their degree/order shape is validated, so callers can
    /// ingest a large source file into a deterministic runtime truncation.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] when required metadata is
    /// missing, the normalization is unsupported, a coefficient line is
    /// malformed, or any retained coefficient fails the same validation as
    /// [`Self::new`].
    #[cfg(feature = "std")]
    pub fn from_icgem_gfc_str(
        input: &str,
        max_degree: usize,
        max_order: usize,
    ) -> Result<Self, PhysicsError> {
        Ok(Self::from_icgem_gfc_str_with_metadata(input, max_degree, max_order)?.into_field())
    }

    /// Construct a fully-normalized coefficient field plus ICGEM source metadata.
    ///
    /// This validates and returns the `earth_gravity_constant`, `radius`, and
    /// source `max_degree` headers alongside the requested field truncation. The
    /// requested `max_degree` must not exceed the source header degree.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] when required metadata is
    /// missing, source constants are invalid, the requested degree exceeds the
    /// source degree, the normalization is unsupported, a coefficient line is
    /// malformed, or any retained coefficient fails the same validation as
    /// [`Self::new`].
    #[cfg(feature = "std")]
    pub fn from_icgem_gfc_str_with_metadata(
        input: &str,
        max_degree: usize,
        max_order: usize,
    ) -> Result<IcgemGfcNormalizedField, PhysicsError> {
        let mut norm_is_fully_normalized = false;
        let mut tide_system = None;
        let mut gravity_constant_m3_s2 = None;
        let mut reference_radius_m = None;
        let mut source_max_degree = None;
        let mut in_header = true;
        let mut saw_end_of_head = false;
        let mut coefficients = Vec::new();

        for raw_line in input.lines() {
            let line = raw_line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let mut parts = line.split_whitespace();
            let Some(kind) = parts.next() else {
                continue;
            };

            if in_header {
                match kind {
                    "end_of_head" => {
                        in_header = false;
                        saw_end_of_head = true;
                    }
                    "norm" => {
                        let norm = parts.next().ok_or(PhysicsError::InvalidParameter {
                            reason: "ICGEM GFC norm header must include a value",
                        })?;
                        if norm != "fully_normalized" {
                            return Err(PhysicsError::InvalidParameter {
                                reason: "ICGEM GFC norm must be fully_normalized",
                            });
                        }
                        norm_is_fully_normalized = true;
                    }
                    "earth_gravity_constant" => {
                        let value = parse_icgem_f64(
                            parts.next(),
                            "ICGEM GFC earth_gravity_constant header must parse",
                        )?;
                        if value <= 0.0 {
                            return Err(PhysicsError::InvalidParameter {
                                reason: "ICGEM GFC earth_gravity_constant must be strictly positive and finite",
                            });
                        }
                        gravity_constant_m3_s2 = Some(value);
                    }
                    "radius" => {
                        let value =
                            parse_icgem_f64(parts.next(), "ICGEM GFC radius header must parse")?;
                        if value <= 0.0 {
                            return Err(PhysicsError::InvalidParameter {
                                reason: "ICGEM GFC radius must be strictly positive and finite",
                            });
                        }
                        reference_radius_m = Some(value);
                    }
                    "max_degree" => {
                        source_max_degree = Some(parse_icgem_usize(
                            parts.next(),
                            "ICGEM GFC max_degree header must parse",
                        )?);
                    }
                    "tide_system" => {
                        let tag = parts.next().ok_or(PhysicsError::InvalidParameter {
                            reason: "ICGEM GFC tide_system header must include a value",
                        })?;
                        tide_system = Some(TideSystem::from_tag(tag)?);
                    }
                    _ => {}
                }
                continue;
            }

            if kind != "gfc" {
                return Err(PhysicsError::InvalidParameter {
                    reason: "ICGEM GFC static parser accepts only gfc coefficient lines",
                });
            }
            let degree = parse_icgem_usize(parts.next(), "ICGEM GFC degree must parse")?;
            let order = parse_icgem_usize(parts.next(), "ICGEM GFC order must parse")?;
            if order > degree {
                return Err(PhysicsError::InvalidParameter {
                    reason: "ICGEM GFC coefficient order must be <= degree",
                });
            }
            if let Some(source_max_degree) = source_max_degree
                && degree > source_max_degree
            {
                return Err(PhysicsError::InvalidParameter {
                    reason: "ICGEM GFC coefficient degree exceeds source max_degree",
                });
            }
            let cbar = parse_icgem_f64(parts.next(), "ICGEM GFC Cbar must parse")?;
            let sbar = parse_icgem_f64(parts.next(), "ICGEM GFC Sbar must parse")?;
            if degree <= max_degree && order <= max_order {
                coefficients.push(NormalizedHarmonicCoefficient::new(
                    degree, order, cbar, sbar,
                )?);
            }
        }

        if !saw_end_of_head {
            return Err(PhysicsError::InvalidParameter {
                reason: "ICGEM GFC input must contain end_of_head",
            });
        }
        if !norm_is_fully_normalized {
            return Err(PhysicsError::InvalidParameter {
                reason: "ICGEM GFC input must declare norm fully_normalized",
            });
        }
        let tide_system = tide_system.ok_or(PhysicsError::InvalidParameter {
            reason: "ICGEM GFC input must declare tide_system",
        })?;
        let gravity_constant_m3_s2 =
            gravity_constant_m3_s2.ok_or(PhysicsError::InvalidParameter {
                reason: "ICGEM GFC input must declare earth_gravity_constant",
            })?;
        let reference_radius_m = reference_radius_m.ok_or(PhysicsError::InvalidParameter {
            reason: "ICGEM GFC input must declare radius",
        })?;
        let source_max_degree = source_max_degree.ok_or(PhysicsError::InvalidParameter {
            reason: "ICGEM GFC input must declare max_degree",
        })?;
        let field = Self::new(max_degree, max_order, tide_system, coefficients)?;
        IcgemGfcNormalizedField::new(
            field,
            gravity_constant_m3_s2,
            reference_radius_m,
            source_max_degree,
        )
    }

    /// Declared maximum harmonic degree.
    #[must_use]
    pub const fn max_degree(&self) -> usize {
        self.max_degree
    }

    /// Declared maximum harmonic order.
    #[must_use]
    pub const fn max_order(&self) -> usize {
        self.max_order
    }

    /// Permanent-tide convention declared for the field.
    #[must_use]
    pub const fn tide_system(&self) -> TideSystem {
        self.tide_system
    }

    /// Number of explicitly supplied coefficient pairs.
    #[must_use]
    pub const fn coefficient_count(&self) -> usize {
        self.coefficient_count
    }

    /// Number of deterministic packed coefficient slots.
    #[must_use]
    pub fn storage_len(&self) -> usize {
        self.coefficients.len()
    }

    /// Iterate every deterministic packed coefficient slot.
    ///
    /// Missing entries inside the declared envelope are yielded as zero-valued
    /// coefficients. This is the stream shape expected by future harmonic
    /// synthesis kernels.
    #[must_use]
    pub fn coefficients(&self) -> NormalizedHarmonicFieldIter<'_> {
        NormalizedHarmonicFieldIter {
            field: self,
            degree: 0,
            order: 0,
            remaining: self.coefficients.len(),
        }
    }

    /// Return a copy with the central `(0, 0)` coefficient set to zero.
    ///
    /// Full gravity coefficient files commonly carry `Cbar00 = 1` to encode
    /// the central `µ/r` potential term. Models that already add point-mass
    /// gravity must evaluate only the harmonic correction field, so this helper
    /// makes the central-term stripping explicit and deterministic before
    /// constructing those models.
    #[must_use]
    pub fn without_central_term(&self) -> Self {
        let mut field = self.clone();
        if let Some((cbar00, sbar00)) = field.coefficients.first_mut() {
            let had_nonzero_central_term = *cbar00 != 0.0 || *sbar00 != 0.0;
            *cbar00 = 0.0;
            *sbar00 = 0.0;
            if had_nonzero_central_term {
                field.coefficient_count = field.coefficient_count.saturating_sub(1);
            }
        }
        field
    }

    /// Return a fully-normalized `Cbar/Sbar` pair for `(degree, order)`.
    ///
    /// Missing entries inside the declared envelope return `(0, 0)`.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if the requested pair is
    /// outside the declared field envelope or has `order > degree`.
    pub fn coefficient(&self, degree: usize, order: usize) -> Result<(f64, f64), PhysicsError> {
        HarmonicTruncation::for_normalized_field(self, degree, order)?;
        let index = normalized_harmonic_index(self.max_order, degree, order).ok_or(
            PhysicsError::InvalidParameter {
                reason: "normalized harmonic coefficient lookup order must be <= degree",
            },
        )?;
        Ok(self.coefficients[index])
    }

    /// Extract the degree-2 coefficients used by the current tesseral evaluator.
    ///
    /// Missing `m = 1` or `m = 2` entries inside a lower-order field are treated
    /// as zeros; this lets an order-0 field feed the byte-identical J2 path.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if the field does not declare
    /// at least degree 2.
    pub fn normalized_degree_two_tesseral_coefficients(
        &self,
    ) -> Result<NormalizedDegreeTwoTesseralCoefficients, PhysicsError> {
        if self.max_degree < 2 {
            return Err(PhysicsError::InvalidParameter {
                reason: "normalized harmonic field does not include degree 2",
            });
        }
        let (cbar20, _) = self.coefficient(2, 0)?;
        let (cbar21, sbar21) = self.coefficient_or_default(2, 1);
        let (cbar22, sbar22) = self.coefficient_or_default(2, 2);
        NormalizedDegreeTwoTesseralCoefficients::new(
            cbar20,
            cbar21,
            sbar21,
            cbar22,
            sbar22,
            self.tide_system,
        )
    }

    /// Evaluate the normalized Pines scalar-potential correction sum.
    ///
    /// The position must be in the body-fixed frame associated with this
    /// coefficient field. Missing in-envelope coefficients participate as
    /// exact zeros, preserving deterministic slot order for future acceleration
    /// and gradient synthesis.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] if the truncation exceeds the field envelope,
    /// if the synthesis geometry is invalid, or if any recurrence/sum produces
    /// a non-finite value.
    pub fn pines_dimensionless_potential_sum(
        &self,
        position_body_fixed_m: Vector3<f64>,
        reference_radius_m: f64,
        truncation: HarmonicTruncation,
    ) -> Result<PinesPotentialSum, PhysicsError> {
        let truncation = checked_pines_synthesis_truncation(self, truncation)?;
        let point = PinesSynthesisPoint::new(position_body_fixed_m, reference_radius_m)?;
        let legendre = PinesLegendreTable::new(point.u(), truncation.degree(), truncation.order())?;
        let longitude = PinesLongitudePolynomials::new(point.s(), point.t(), truncation.order())?;

        let mut radial_power = 1.0_f64;
        let mut sum = 0.0_f64;
        let mut term_count = 0_usize;
        for degree in 0..=truncation.degree() {
            if degree > 0 {
                radial_power *= point.reference_radius_over_radius();
                if !radial_power.is_finite() {
                    return Err(PhysicsError::NonFinite {
                        reason: "Pines potential radial power produced non-finite output",
                    });
                }
            }
            for order in 0..=degree.min(truncation.order()) {
                let (cbar, sbar) = self.coefficient(degree, order)?;
                let a_nm = legendre.value(degree, order)?;
                let (real, imaginary) = longitude.polynomial(order)?;
                sum += radial_power * a_nm * (cbar * real + sbar * imaginary);
                if !sum.is_finite() {
                    return Err(PhysicsError::NonFinite {
                        reason: "Pines potential sum produced non-finite output",
                    });
                }
                term_count += 1;
            }
        }

        Ok(PinesPotentialSum {
            truncation,
            dimensionless_correction: sum,
            term_count,
        })
    }

    /// Evaluate the normalized Gottlieb-style scalar-potential correction sum.
    ///
    /// The position must be in the body-fixed frame associated with this
    /// coefficient field. This is a scalar-potential recomposition oracle for
    /// the Pines path: it uses `cos(mλ)` / `sin(mλ)` longitude terms and an
    /// explicit `(sqrt(x² + y²) / r)^m` factor instead of Pines
    /// direction-cosine longitude polynomials.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] if the truncation exceeds the field envelope,
    /// if the synthesis geometry is invalid, or if any recurrence/sum produces
    /// a non-finite value.
    pub fn gottlieb_dimensionless_potential_sum(
        &self,
        position_body_fixed_m: Vector3<f64>,
        reference_radius_m: f64,
        truncation: HarmonicTruncation,
    ) -> Result<GottliebPotentialSum, PhysicsError> {
        let truncation = checked_pines_synthesis_truncation(self, truncation)?;
        let point = PinesSynthesisPoint::new(position_body_fixed_m, reference_radius_m)?;
        let legendre = PinesLegendreTable::new(point.u(), truncation.degree(), truncation.order())?;
        let longitude_rad = point.t().atan2(point.s());
        let longitude = HarmonicLongitudeTrigonometry::new(longitude_rad, truncation.order())?;
        let horizontal_norm = (point.s() * point.s() + point.t() * point.t()).sqrt();

        let mut radial_power = 1.0_f64;
        let mut sum = 0.0_f64;
        let mut term_count = 0_usize;
        for degree in 0..=truncation.degree() {
            if degree > 0 {
                radial_power *= point.reference_radius_over_radius();
                if !radial_power.is_finite() {
                    return Err(PhysicsError::NonFinite {
                        reason: "Gottlieb potential radial power produced non-finite output",
                    });
                }
            }
            let mut horizontal_power = 1.0_f64;
            for order in 0..=degree.min(truncation.order()) {
                if order > 0 {
                    horizontal_power *= horizontal_norm;
                    if !horizontal_power.is_finite() {
                        return Err(PhysicsError::NonFinite {
                            reason: "Gottlieb potential horizontal power produced non-finite output",
                        });
                    }
                }
                let (cbar, sbar) = self.coefficient(degree, order)?;
                let a_nm = legendre.value(degree, order)?;
                let (cos_m_lambda, sin_m_lambda) = longitude.harmonic(order)?;
                sum += radial_power
                    * horizontal_power
                    * a_nm
                    * (cbar * cos_m_lambda + sbar * sin_m_lambda);
                if !sum.is_finite() {
                    return Err(PhysicsError::NonFinite {
                        reason: "Gottlieb potential sum produced non-finite output",
                    });
                }
                term_count += 1;
            }
        }

        Ok(GottliebPotentialSum {
            truncation,
            dimensionless_correction: sum,
            term_count,
        })
    }

    /// Evaluate harmonic-correction acceleration from the normalized
    /// Gottlieb-style scalar-potential sum by deterministic symmetric finite
    /// differences.
    ///
    /// This mirrors
    /// [`Self::pines_potential_correction_acceleration_finite_difference_m_s2`]
    /// but differentiates
    /// [`Self::gottlieb_dimensionless_potential_sum`]. It is an acceleration
    /// cross-check path, not the final analytic normalized Gottlieb gradient
    /// kernel.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if `µ` or `step_m` is not
    /// strictly positive and finite, or if the wrapped scalar-potential
    /// evaluation rejects the geometry/truncation. Returns
    /// [`PhysicsError::NonFinite`] if the finite-difference result is
    /// non-finite.
    pub fn gottlieb_potential_correction_acceleration_finite_difference_m_s2(
        &self,
        mu_m3_s2: f64,
        position_body_fixed_m: Vector3<f64>,
        reference_radius_m: f64,
        truncation: HarmonicTruncation,
        step_m: f64,
    ) -> Result<Vector3<f64>, PhysicsError> {
        if !mu_m3_s2.is_finite() || mu_m3_s2 <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "Gottlieb finite-difference µ must be strictly positive and finite",
            });
        }
        if !step_m.is_finite() || step_m <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "Gottlieb finite-difference step must be strictly positive and finite",
            });
        }
        PinesSynthesisPoint::new(position_body_fixed_m, reference_radius_m)?;

        let offset_x = Vector3::new(step_m, 0.0, 0.0);
        let offset_y = Vector3::new(0.0, step_m, 0.0);
        let offset_z = Vector3::new(0.0, 0.0, step_m);
        let acceleration = Vector3::new(
            (self.gottlieb_potential_correction_m2_s2(
                mu_m3_s2,
                position_body_fixed_m + offset_x,
                reference_radius_m,
                truncation,
            )? - self.gottlieb_potential_correction_m2_s2(
                mu_m3_s2,
                position_body_fixed_m - offset_x,
                reference_radius_m,
                truncation,
            )?) / (2.0 * step_m),
            (self.gottlieb_potential_correction_m2_s2(
                mu_m3_s2,
                position_body_fixed_m + offset_y,
                reference_radius_m,
                truncation,
            )? - self.gottlieb_potential_correction_m2_s2(
                mu_m3_s2,
                position_body_fixed_m - offset_y,
                reference_radius_m,
                truncation,
            )?) / (2.0 * step_m),
            (self.gottlieb_potential_correction_m2_s2(
                mu_m3_s2,
                position_body_fixed_m + offset_z,
                reference_radius_m,
                truncation,
            )? - self.gottlieb_potential_correction_m2_s2(
                mu_m3_s2,
                position_body_fixed_m - offset_z,
                reference_radius_m,
                truncation,
            )?) / (2.0 * step_m),
        );
        if !acceleration.iter().all(|value| value.is_finite()) {
            return Err(PhysicsError::NonFinite {
                reason: "Gottlieb finite-difference acceleration produced non-finite output",
            });
        }
        Ok(acceleration)
    }

    /// Evaluate harmonic-correction acceleration from the normalized Pines
    /// scalar-potential sum by deterministic symmetric finite differences.
    ///
    /// This returns the gradient of `µ/r * pines_dimensionless_potential_sum`
    /// for the supplied field/truncation. It is intended as a bounded
    /// cross-check and transition path for the full analytic Pines/Gottlieb
    /// acceleration kernel, not as the final high-degree production evaluator.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if `µ` or `step_m` is not
    /// strictly positive and finite, or if the wrapped scalar-potential
    /// evaluation rejects the geometry/truncation. Returns
    /// [`PhysicsError::NonFinite`] if the finite-difference result is
    /// non-finite.
    pub fn pines_potential_correction_acceleration_finite_difference_m_s2(
        &self,
        mu_m3_s2: f64,
        position_body_fixed_m: Vector3<f64>,
        reference_radius_m: f64,
        truncation: HarmonicTruncation,
        step_m: f64,
    ) -> Result<Vector3<f64>, PhysicsError> {
        if !mu_m3_s2.is_finite() || mu_m3_s2 <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "Pines finite-difference µ must be strictly positive and finite",
            });
        }
        if !step_m.is_finite() || step_m <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "Pines finite-difference step must be strictly positive and finite",
            });
        }
        PinesSynthesisPoint::new(position_body_fixed_m, reference_radius_m)?;

        let offset_x = Vector3::new(step_m, 0.0, 0.0);
        let offset_y = Vector3::new(0.0, step_m, 0.0);
        let offset_z = Vector3::new(0.0, 0.0, step_m);
        let acceleration = Vector3::new(
            (self.pines_potential_correction_m2_s2(
                mu_m3_s2,
                position_body_fixed_m + offset_x,
                reference_radius_m,
                truncation,
            )? - self.pines_potential_correction_m2_s2(
                mu_m3_s2,
                position_body_fixed_m - offset_x,
                reference_radius_m,
                truncation,
            )?) / (2.0 * step_m),
            (self.pines_potential_correction_m2_s2(
                mu_m3_s2,
                position_body_fixed_m + offset_y,
                reference_radius_m,
                truncation,
            )? - self.pines_potential_correction_m2_s2(
                mu_m3_s2,
                position_body_fixed_m - offset_y,
                reference_radius_m,
                truncation,
            )?) / (2.0 * step_m),
            (self.pines_potential_correction_m2_s2(
                mu_m3_s2,
                position_body_fixed_m + offset_z,
                reference_radius_m,
                truncation,
            )? - self.pines_potential_correction_m2_s2(
                mu_m3_s2,
                position_body_fixed_m - offset_z,
                reference_radius_m,
                truncation,
            )?) / (2.0 * step_m),
        );
        if !acceleration.iter().all(|value| value.is_finite()) {
            return Err(PhysicsError::NonFinite {
                reason: "Pines finite-difference acceleration produced non-finite output",
            });
        }
        Ok(acceleration)
    }

    /// Evaluate the harmonic-correction acceleration gradient from the
    /// normalized Gottlieb-style finite-difference acceleration oracle.
    ///
    /// The returned matrix uses the convention `gradient[(i, j)] = da_i / dr_j`
    /// in `s^-2`, with columns representing derivatives with respect to `x`,
    /// `y`, and `z`. This is a deterministic finite-difference oracle, not the
    /// final analytic normalized Gottlieb gradient kernel.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if either finite-difference
    /// step is not strictly positive and finite, or if the wrapped acceleration
    /// evaluation rejects the model inputs. Returns [`PhysicsError::NonFinite`]
    /// if the finite-difference matrix contains non-finite values.
    pub fn gottlieb_potential_correction_acceleration_gradient_finite_difference_s2(
        &self,
        mu_m3_s2: f64,
        position_body_fixed_m: Vector3<f64>,
        reference_radius_m: f64,
        truncation: HarmonicTruncation,
        acceleration_step_m: f64,
        gradient_step_m: f64,
    ) -> Result<Matrix3<f64>, PhysicsError> {
        PinesSynthesisPoint::new(position_body_fixed_m, reference_radius_m)?;
        finite_difference_acceleration_gradient_s2(
            position_body_fixed_m,
            gradient_step_m,
            |position| {
                self.gottlieb_potential_correction_acceleration_finite_difference_m_s2(
                    mu_m3_s2,
                    position,
                    reference_radius_m,
                    truncation,
                    acceleration_step_m,
                )
            },
            "Gottlieb finite-difference gradient step must be strictly positive and finite",
            "Gottlieb finite-difference acceleration gradient produced non-finite output",
        )
    }

    /// Evaluate the harmonic-correction acceleration gradient from the
    /// normalized Pines finite-difference acceleration oracle.
    ///
    /// The returned matrix uses the convention `gradient[(i, j)] = da_i / dr_j`
    /// in `s^-2`, with columns representing derivatives with respect to `x`,
    /// `y`, and `z`. This is a deterministic finite-difference oracle, not the
    /// final analytic high-degree Pines/Gottlieb gradient kernel.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if either finite-difference
    /// step is not strictly positive and finite, or if the wrapped acceleration
    /// evaluation rejects the model inputs. Returns [`PhysicsError::NonFinite`]
    /// if the finite-difference matrix contains non-finite values.
    pub fn pines_potential_correction_acceleration_gradient_finite_difference_s2(
        &self,
        mu_m3_s2: f64,
        position_body_fixed_m: Vector3<f64>,
        reference_radius_m: f64,
        truncation: HarmonicTruncation,
        acceleration_step_m: f64,
        gradient_step_m: f64,
    ) -> Result<Matrix3<f64>, PhysicsError> {
        PinesSynthesisPoint::new(position_body_fixed_m, reference_radius_m)?;
        finite_difference_acceleration_gradient_s2(
            position_body_fixed_m,
            gradient_step_m,
            |position| {
                self.pines_potential_correction_acceleration_finite_difference_m_s2(
                    mu_m3_s2,
                    position,
                    reference_radius_m,
                    truncation,
                    acceleration_step_m,
                )
            },
            "Pines finite-difference gradient step must be strictly positive and finite",
            "Pines finite-difference acceleration gradient produced non-finite output",
        )
    }

    fn coefficient_or_default(&self, degree: usize, order: usize) -> (f64, f64) {
        if degree > self.max_degree || order > self.max_order || order > degree {
            return (0.0, 0.0);
        }
        self.coefficient(degree, order).unwrap_or((0.0, 0.0))
    }

    fn pines_potential_correction_m2_s2(
        &self,
        mu_m3_s2: f64,
        position_body_fixed_m: Vector3<f64>,
        reference_radius_m: f64,
        truncation: HarmonicTruncation,
    ) -> Result<f64, PhysicsError> {
        let point = PinesSynthesisPoint::new(position_body_fixed_m, reference_radius_m)?;
        let sum = self.pines_dimensionless_potential_sum(
            position_body_fixed_m,
            reference_radius_m,
            truncation,
        )?;
        let potential = mu_m3_s2 / point.radius_m() * sum.dimensionless_correction();
        if !potential.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "Pines scalar-potential correction produced non-finite output",
            });
        }
        Ok(potential)
    }

    fn gottlieb_potential_correction_m2_s2(
        &self,
        mu_m3_s2: f64,
        position_body_fixed_m: Vector3<f64>,
        reference_radius_m: f64,
        truncation: HarmonicTruncation,
    ) -> Result<f64, PhysicsError> {
        let point = PinesSynthesisPoint::new(position_body_fixed_m, reference_radius_m)?;
        let sum = self.gottlieb_dimensionless_potential_sum(
            position_body_fixed_m,
            reference_radius_m,
            truncation,
        )?;
        let potential = mu_m3_s2 / point.radius_m() * sum.dimensionless_correction();
        if !potential.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "Gottlieb scalar-potential correction produced non-finite output",
            });
        }
        Ok(potential)
    }

    #[cfg(feature = "std")]
    fn from_normalized_degree2_toml_table(
        table: &toml::value::Table,
        tide_system: TideSystem,
    ) -> Result<Self, PhysicsError> {
        let degree_2 = table
            .get("degree_2")
            .and_then(toml::Value::as_table)
            .ok_or(PhysicsError::InvalidParameter {
                reason: "normalized degree-2 TOML must contain a degree_2 table",
            })?;
        Self::new(
            2,
            2,
            tide_system,
            [
                NormalizedHarmonicCoefficient::new(
                    2,
                    0,
                    normalized_harmonic_toml_f64(degree_2, "cbar20")?,
                    0.0,
                )?,
                NormalizedHarmonicCoefficient::new(
                    2,
                    1,
                    normalized_harmonic_toml_f64(degree_2, "cbar21")?,
                    normalized_harmonic_toml_f64(degree_2, "sbar21")?,
                )?,
                NormalizedHarmonicCoefficient::new(
                    2,
                    2,
                    normalized_harmonic_toml_f64(degree_2, "cbar22")?,
                    normalized_harmonic_toml_f64(degree_2, "sbar22")?,
                )?,
            ],
        )
    }

    #[cfg(feature = "std")]
    fn from_normalized_general_toml_table(
        table: &toml::value::Table,
        tide_system: TideSystem,
        require_zonal: bool,
    ) -> Result<Self, PhysicsError> {
        let max_degree = normalized_harmonic_toml_usize(table, "max_degree")?;
        let max_order = normalized_harmonic_toml_usize(table, "max_order")?;
        if require_zonal && max_order != 0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "normalized zonal TOML max_order must be zero",
            });
        }
        let coefficients = table
            .get("coefficient")
            .and_then(toml::Value::as_array)
            .ok_or(PhysicsError::InvalidParameter {
                reason: "normalized harmonic TOML must contain coefficient entries",
            })?
            .iter()
            .map(|entry| {
                let entry = entry.as_table().ok_or(PhysicsError::InvalidParameter {
                    reason: "normalized harmonic coefficient entry must be a table",
                })?;
                let degree = normalized_harmonic_toml_usize(entry, "degree")?;
                let order = normalized_harmonic_toml_usize(entry, "order")?;
                if require_zonal && order != 0 {
                    return Err(PhysicsError::InvalidParameter {
                        reason: "normalized zonal TOML coefficient order must be zero",
                    });
                }
                NormalizedHarmonicCoefficient::new(
                    degree,
                    order,
                    normalized_harmonic_toml_f64(entry, "cbar")?,
                    normalized_harmonic_toml_f64(entry, "sbar")?,
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        Self::new(max_degree, max_order, tide_system, coefficients)
    }
}

#[cfg(feature = "std")]
fn normalized_harmonic_toml_str<'a>(
    table: &'a toml::value::Table,
    key: &'static str,
) -> Result<&'a str, PhysicsError> {
    table
        .get(key)
        .and_then(toml::Value::as_str)
        .ok_or(PhysicsError::InvalidParameter {
            reason: "normalized harmonic TOML string field is missing or invalid",
        })
}

#[cfg(feature = "std")]
fn normalized_harmonic_toml_usize(
    table: &toml::value::Table,
    key: &'static str,
) -> Result<usize, PhysicsError> {
    let value =
        table
            .get(key)
            .and_then(toml::Value::as_integer)
            .ok_or(PhysicsError::InvalidParameter {
                reason: "normalized harmonic TOML integer field is missing or invalid",
            })?;
    usize::try_from(value).map_err(|_| PhysicsError::InvalidParameter {
        reason: "normalized harmonic TOML integer field must be non-negative",
    })
}

#[cfg(feature = "std")]
fn normalized_harmonic_toml_f64(
    table: &toml::value::Table,
    key: &'static str,
) -> Result<f64, PhysicsError> {
    let value =
        table
            .get(key)
            .and_then(toml::Value::as_float)
            .ok_or(PhysicsError::InvalidParameter {
                reason: "normalized harmonic TOML float field is missing or invalid",
            })?;
    if !value.is_finite() {
        return Err(PhysicsError::InvalidParameter {
            reason: "normalized harmonic TOML float field must be finite",
        });
    }
    Ok(value)
}

#[cfg(feature = "std")]
fn parse_icgem_usize(value: Option<&str>, reason: &'static str) -> Result<usize, PhysicsError> {
    value
        .ok_or(PhysicsError::InvalidParameter { reason })?
        .parse::<usize>()
        .map_err(|_| PhysicsError::InvalidParameter { reason })
}

#[cfg(feature = "std")]
fn parse_icgem_f64(value: Option<&str>, reason: &'static str) -> Result<f64, PhysicsError> {
    let value = value.ok_or(PhysicsError::InvalidParameter { reason })?;
    let parsed = if value.contains('D') || value.contains('d') {
        value
            .replace(['D', 'd'], "E")
            .parse::<f64>()
            .map_err(|_| PhysicsError::InvalidParameter { reason })?
    } else {
        value
            .parse::<f64>()
            .map_err(|_| PhysicsError::InvalidParameter { reason })?
    };
    if !parsed.is_finite() {
        return Err(PhysicsError::InvalidParameter {
            reason: "ICGEM GFC coefficient value must be finite",
        });
    }
    Ok(parsed)
}

fn finite_difference_acceleration_gradient_s2(
    position_m: Vector3<f64>,
    gradient_step_m: f64,
    acceleration_at: impl Fn(Vector3<f64>) -> Result<Vector3<f64>, PhysicsError>,
    invalid_step_reason: &'static str,
    non_finite_reason: &'static str,
) -> Result<Matrix3<f64>, PhysicsError> {
    if !gradient_step_m.is_finite() || gradient_step_m <= 0.0 {
        return Err(PhysicsError::InvalidParameter {
            reason: invalid_step_reason,
        });
    }
    let offset_x = Vector3::new(gradient_step_m, 0.0, 0.0);
    let offset_y = Vector3::new(0.0, gradient_step_m, 0.0);
    let offset_z = Vector3::new(0.0, 0.0, gradient_step_m);
    let inv_delta = 1.0 / (2.0 * gradient_step_m);
    let da_dx = (acceleration_at(position_m + offset_x)? - acceleration_at(position_m - offset_x)?)
        * inv_delta;
    let da_dy = (acceleration_at(position_m + offset_y)? - acceleration_at(position_m - offset_y)?)
        * inv_delta;
    let da_dz = (acceleration_at(position_m + offset_z)? - acceleration_at(position_m - offset_z)?)
        * inv_delta;
    let gradient = Matrix3::from_columns(&[da_dx, da_dy, da_dz]);
    if !gradient.iter().all(|value| value.is_finite()) {
        return Err(PhysicsError::NonFinite {
            reason: non_finite_reason,
        });
    }
    Ok(gradient)
}

fn normalized_harmonic_storage_len(max_degree: usize, max_order: usize) -> Option<usize> {
    if max_degree <= max_order {
        checked_triangular_count(max_degree.checked_add(1)?)
    } else {
        let prefix = checked_triangular_count(max_order.checked_add(1)?)?;
        let tail_rows = max_degree.checked_sub(max_order)?;
        let row_width = max_order.checked_add(1)?;
        prefix.checked_add(tail_rows.checked_mul(row_width)?)
    }
}

fn normalized_harmonic_index(max_order: usize, degree: usize, order: usize) -> Option<usize> {
    if order > degree || order > max_order {
        return None;
    }
    let row_start = if degree <= max_order {
        checked_triangular_count(degree)?
    } else {
        let prefix = checked_triangular_count(max_order.checked_add(1)?)?;
        let tail_rows_before = degree.checked_sub(max_order)?.checked_sub(1)?;
        let row_width = max_order.checked_add(1)?;
        prefix.checked_add(tail_rows_before.checked_mul(row_width)?)?
    };
    row_start.checked_add(order)
}

fn checked_triangular_count(row_count: usize) -> Option<usize> {
    row_count
        .checked_mul(row_count.checked_add(1)?)?
        .checked_div(2)
}

fn checked_pines_synthesis_truncation(
    field: &NormalizedHarmonicField,
    truncation: HarmonicTruncation,
) -> Result<HarmonicTruncation, PhysicsError> {
    let truncation =
        HarmonicTruncation::for_normalized_field(field, truncation.degree(), truncation.order())?;
    if truncation.degree() > PINES_LEGENDRE_MAX_DEGREE {
        return Err(PhysicsError::InvalidParameter {
            reason: "Pines synthesis truncation degree exceeds checked Legendre envelope",
        });
    }
    if truncation.order() > HARMONIC_LONGITUDE_MAX_ORDER {
        return Err(PhysicsError::InvalidParameter {
            reason: "Pines synthesis truncation order exceeds checked longitude envelope",
        });
    }
    Ok(truncation)
}

// ---------------------------------------------------------------------
// FiniteDifferencePinesGravity
// ---------------------------------------------------------------------

/// Static normalized Pines gravity via symmetric finite differences.
///
/// This model evaluates point-mass gravity plus the harmonic correction from
/// [`NormalizedHarmonicField::pines_potential_correction_acceleration_finite_difference_m_s2`].
/// It is a bounded transition surface for WP-08.1 validation: useful for
/// plumbing normalized fields through the public [`GravityModel`] trait and
/// cross-checking low-degree terms, but not the final analytic high-degree
/// Pines/Gottlieb production evaluator.
///
/// The input ECI vector is currently treated as body-fixed with axes aligned to
/// ECI. A later force-stack slice will insert the Earth-orientation frame
/// transform before this model is used for rotating-body production runs.
#[derive(Clone, Debug, PartialEq)]
pub struct FiniteDifferencePinesGravity {
    mu_m3_s2: f64,
    reference_radius_m: f64,
    field: NormalizedHarmonicField,
    truncation: HarmonicTruncation,
    finite_difference_step_m: f64,
}

impl FiniteDifferencePinesGravity {
    /// Construct a finite-difference normalized Pines gravity model.
    ///
    /// `field` is interpreted as a harmonic correction field; `Cbar00/Sbar00`
    /// must therefore be exactly zero so the configured point-mass `µ` is not
    /// double-counted.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if gravity constants or the
    /// finite-difference step are invalid, if the truncation exceeds either the
    /// field envelope or the bounded Pines scratch envelopes, or if the field
    /// carries a nonzero degree-0/order-0 coefficient.
    pub fn new(
        mu_m3_s2: f64,
        reference_radius_m: f64,
        field: NormalizedHarmonicField,
        truncation: HarmonicTruncation,
        finite_difference_step_m: f64,
    ) -> Result<Self, PhysicsError> {
        if !mu_m3_s2.is_finite() || mu_m3_s2 <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "Pines gravity µ must be strictly positive and finite",
            });
        }
        if !reference_radius_m.is_finite() || reference_radius_m <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "Pines gravity reference radius must be strictly positive and finite",
            });
        }
        if !finite_difference_step_m.is_finite() || finite_difference_step_m <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "Pines gravity finite-difference step must be strictly positive and finite",
            });
        }
        let truncation =
            HarmonicSynthesisPlan::for_normalized_field(&field, truncation)?.truncation();
        let (cbar00, sbar00) = field.coefficient(0, 0)?;
        if cbar00 != 0.0 || sbar00 != 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "Pines gravity correction field must not include a central C00 term",
            });
        }
        Ok(Self {
            mu_m3_s2,
            reference_radius_m,
            field,
            truncation,
            finite_difference_step_m,
        })
    }

    /// Construct from a full normalized gravity field that may include Cbar00.
    ///
    /// This is the ingestion-facing constructor for ICGEM/NGA-style gravity
    /// fields that include the central `Cbar00 = 1` term. The central slot is
    /// explicitly stripped before delegating to [`Self::new`], preserving this
    /// model's point-mass-plus-harmonic-correction decomposition.
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`Self::new`] after central-term stripping.
    pub fn new_from_full_normalized_field(
        mu_m3_s2: f64,
        reference_radius_m: f64,
        field: NormalizedHarmonicField,
        truncation: HarmonicTruncation,
        finite_difference_step_m: f64,
    ) -> Result<Self, PhysicsError> {
        Self::new(
            mu_m3_s2,
            reference_radius_m,
            field.without_central_term(),
            truncation,
            finite_difference_step_m,
        )
    }

    /// Construct directly from an ICGEM `.gfc` coefficient block.
    ///
    /// The parser validates `norm fully_normalized`, `tide_system`,
    /// `earth_gravity_constant`, `radius`, and the source `max_degree` header,
    /// truncates static `gfc` rows to the requested degree/order envelope, strips
    /// `Cbar00`, and then builds the finite-difference Pines transition model
    /// with the source constants from the file.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if the GFC input is malformed,
    /// if the requested truncation exceeds the source/field/scratch envelopes, or
    /// if model constants and finite-difference settings are invalid.
    #[cfg(feature = "std")]
    pub fn new_from_icgem_gfc_str(
        input: &str,
        max_degree: usize,
        max_order: usize,
        finite_difference_step_m: f64,
    ) -> Result<Self, PhysicsError> {
        let parsed = NormalizedHarmonicField::from_icgem_gfc_str_with_metadata(
            input, max_degree, max_order,
        )?;
        let mu_m3_s2 = parsed.gravity_constant_m3_s2();
        let reference_radius_m = parsed.reference_radius_m();
        Self::new_from_full_normalized_field(
            mu_m3_s2,
            reference_radius_m,
            parsed.into_field(),
            HarmonicTruncation::new(max_degree, max_order)?,
            finite_difference_step_m,
        )
    }

    /// Construct from a pre-resolved harmonic synthesis plan.
    ///
    /// The plan is revalidated against `field` so callers cannot accidentally
    /// reuse a plan resolved for a different coefficient envelope.
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`Self::new`].
    pub fn new_with_synthesis_plan(
        mu_m3_s2: f64,
        reference_radius_m: f64,
        field: NormalizedHarmonicField,
        plan: HarmonicSynthesisPlan,
        finite_difference_step_m: f64,
    ) -> Result<Self, PhysicsError> {
        Self::new(
            mu_m3_s2,
            reference_radius_m,
            field,
            plan.truncation(),
            finite_difference_step_m,
        )
    }

    /// Construct from a named EGM2008 high-degree runtime tier.
    ///
    /// This validates the requested tier against the supplied field envelope.
    /// It still uses the finite-difference Pines transition path; it does not
    /// claim the final analytic EGM2008 kernel is complete.
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`Self::new`] plus an error when `field`
    /// does not cover `tier`.
    pub fn new_with_synthesis_tier(
        mu_m3_s2: f64,
        reference_radius_m: f64,
        field: NormalizedHarmonicField,
        tier: HarmonicSynthesisTier,
        finite_difference_step_m: f64,
    ) -> Result<Self, PhysicsError> {
        let plan = HarmonicSynthesisPlan::egm2008_tier_for_normalized_field(&field, tier)?;
        Self::new_with_synthesis_plan(
            mu_m3_s2,
            reference_radius_m,
            field,
            plan,
            finite_difference_step_m,
        )
    }

    /// Configured gravitational parameter in m^3/s^2.
    #[must_use]
    pub const fn mu_m3_s2(&self) -> f64 {
        self.mu_m3_s2
    }

    /// Configured harmonic reference radius in metres.
    #[must_use]
    pub const fn reference_radius_m(&self) -> f64 {
        self.reference_radius_m
    }

    /// Owned normalized harmonic correction field.
    #[must_use]
    pub const fn field(&self) -> &NormalizedHarmonicField {
        &self.field
    }

    /// Configured harmonic truncation.
    #[must_use]
    pub const fn truncation(&self) -> HarmonicTruncation {
        self.truncation
    }

    /// Symmetric finite-difference step in metres.
    #[must_use]
    pub const fn finite_difference_step_m(&self) -> f64 {
        self.finite_difference_step_m
    }

    /// Full acceleration-gradient matrix for the current transition model.
    ///
    /// The returned matrix uses `gradient[(i, j)] = da_i / dr_j` in `s^-2`.
    /// It is the closed-form point-mass tensor plus the finite-difference Pines
    /// harmonic-correction tensor. As with [`Self::gravity_eci_m_s2`], the input
    /// ECI axes are currently treated as body-fixed aligned.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] if point-mass or harmonic-correction gradient
    /// evaluation rejects the position, model constants, truncation, or
    /// finite-difference steps.
    pub fn acceleration_gradient_eci_s2(
        &self,
        position_eci: Position3<Eci>,
        gradient_step_m: f64,
    ) -> Result<Matrix3<f64>, PhysicsError> {
        let position = position_eci.vector;
        let point_mass_gradient = point_mass_acceleration_gradient_s2(self.mu_m3_s2, position)?;
        let correction_gradient = self
            .field
            .pines_potential_correction_acceleration_gradient_finite_difference_s2(
                self.mu_m3_s2,
                position,
                self.reference_radius_m,
                self.truncation,
                self.finite_difference_step_m,
                gradient_step_m,
            )?;
        let gradient = point_mass_gradient + correction_gradient;
        if !gradient.iter().all(|value| value.is_finite()) {
            return Err(PhysicsError::NonFinite {
                reason: "Pines gravity acceleration gradient produced non-finite output",
            });
        }
        Ok(gradient)
    }
}

impl GravityModel for FiniteDifferencePinesGravity {
    fn gravity_eci_m_s2(
        &self,
        position_eci: Position3<Eci>,
        _time: SimTime,
    ) -> Result<Vector3<f64>, PhysicsError> {
        let r = position_eci.vector;
        PinesSynthesisPoint::new(r, self.reference_radius_m)?;
        let r2 = r.dot(&r);
        let r_norm = r2.sqrt();
        let r3 = r_norm * r2;
        let central = (-self.mu_m3_s2 / r3) * r;
        let correction = self
            .field
            .pines_potential_correction_acceleration_finite_difference_m_s2(
                self.mu_m3_s2,
                r,
                self.reference_radius_m,
                self.truncation,
                self.finite_difference_step_m,
            )?;
        let acceleration = central + correction;
        if !acceleration.iter().all(|value| value.is_finite()) {
            return Err(PhysicsError::NonFinite {
                reason: "finite-difference Pines gravity produced non-finite acceleration",
            });
        }
        Ok(acceleration)
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
            self.cbar20 * fully_normalized_to_unnormalized_scale(2, 0)?,
            self.cbar21 * fully_normalized_to_unnormalized_scale(2, 1)?,
            self.sbar21 * fully_normalized_to_unnormalized_scale(2, 1)?,
            self.cbar22 * fully_normalized_to_unnormalized_scale(2, 2)?,
            self.sbar22 * fully_normalized_to_unnormalized_scale(2, 2)?,
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

    /// Construct from a fully-normalized harmonic field.
    ///
    /// This bridges the reusable coefficient substrate into the current
    /// degree-2 evaluator. It remains fail-closed: the requested degree/order
    /// must be supported both by the field and by [`TesseralGravity::new`].
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] for invalid constants,
    /// requested degree/order outside the field envelope, unsupported
    /// [`TesseralGravity`] degree/order, or non-finite coefficients.
    pub fn from_normalized_field(
        mu_m3_s2: f64,
        r_e_m: f64,
        field: &NormalizedHarmonicField,
        degree: usize,
        order: usize,
    ) -> Result<Self, PhysicsError> {
        let truncation = HarmonicTruncation::for_normalized_field(field, degree, order)?;
        let coefficients = if degree == 0 {
            DegreeTwoTesseralCoefficients::zero(field.tide_system())
        } else {
            field
                .normalized_degree_two_tesseral_coefficients()?
                .to_unnormalized()?
        };
        Self::new(
            mu_m3_s2,
            r_e_m,
            coefficients,
            truncation.degree(),
            truncation.order(),
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

    /// Construct the zonal `J_2..J_n` truncation from a fully-normalized
    /// harmonic coefficient field.
    ///
    /// Fully-normalized zonal coefficients convert through
    /// `J_n = -Cbar_n0 * sqrt(2n + 1)`, matching the degree-2 convention used
    /// by [`NormalizedDegreeTwoTesseralCoefficients`]. Missing in-envelope
    /// zonal entries default to zero through [`NormalizedHarmonicField`].
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] for invalid gravity
    /// constants, requested degree outside `[2, 6]`, requested degree outside
    /// the field envelope, or non-finite converted coefficients.
    pub fn from_normalized_field(
        mu_m3_s2: f64,
        r_e_m: f64,
        field: &NormalizedHarmonicField,
        degree: usize,
    ) -> Result<Self, PhysicsError> {
        let truncation = HarmonicTruncation::for_normalized_field(field, degree, 0)?;
        let mut j_n = [0.0; EGM2008_MAX_DEGREE - 1];
        for n in 2..=truncation.degree() {
            let (cbar_n0, _) = field.coefficient(n, 0)?;
            j_n[n - 2] = if cbar_n0 == 0.0 {
                0.0
            } else {
                -cbar_n0 * fully_normalized_zonal_scale(n)?
            };
        }
        Self::new(mu_m3_s2, r_e_m, j_n, truncation.degree())
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

fn fully_normalized_zonal_scale(degree: usize) -> Result<f64, PhysicsError> {
    fully_normalized_to_unnormalized_scale(degree, 0)
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

    fn load_wgs84_normalized_harmonic_field_fixture() -> NormalizedHarmonicField {
        let fixture = include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/gravity/wgs84-degree2-normalized-v1.toml"
        ));
        NormalizedHarmonicField::from_normalized_toml_str(fixture)
            .expect("finite normalized harmonic field")
    }

    fn normalized_field_from_degree_two(
        coefficients: DegreeTwoTesseralCoefficients,
    ) -> NormalizedHarmonicField {
        NormalizedHarmonicField::new(
            2,
            2,
            coefficients.tide_system(),
            [
                NormalizedHarmonicCoefficient::new(
                    2,
                    0,
                    coefficients.c20() / fully_normalized_to_unnormalized_scale(2, 0).unwrap(),
                    0.0,
                )
                .unwrap(),
                NormalizedHarmonicCoefficient::new(
                    2,
                    1,
                    coefficients.c21() / fully_normalized_to_unnormalized_scale(2, 1).unwrap(),
                    coefficients.s21() / fully_normalized_to_unnormalized_scale(2, 1).unwrap(),
                )
                .unwrap(),
                NormalizedHarmonicCoefficient::new(
                    2,
                    2,
                    coefficients.c22() / fully_normalized_to_unnormalized_scale(2, 2).unwrap(),
                    coefficients.s22() / fully_normalized_to_unnormalized_scale(2, 2).unwrap(),
                )
                .unwrap(),
            ],
        )
        .expect("finite normalized degree-2 harmonic field")
    }

    fn load_egm2008_normalized_zonal_degree6_fixture() -> NormalizedHarmonicField {
        let fixture = include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/gravity/egm2008-zonal-degree6-normalized-v1.toml"
        ));
        NormalizedHarmonicField::from_normalized_toml_str(fixture)
            .expect("finite normalized EGM2008 zonal field")
    }

    fn load_synthetic_normalized_degree4_field_fixture() -> NormalizedHarmonicField {
        let fixture = include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/gravity/synthetic-degree4-normalized-field-v1.toml"
        ));
        NormalizedHarmonicField::from_normalized_toml_str(fixture)
            .expect("finite synthetic normalized field")
    }

    fn synthetic_icgem_degree4_fixture() -> &'static str {
        include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/gravity/synthetic-degree4-normalized-icgem-v1.gfc"
        ))
    }

    fn load_synthetic_icgem_degree4_field_fixture(
        max_degree: usize,
        max_order: usize,
    ) -> NormalizedHarmonicField {
        NormalizedHarmonicField::from_icgem_gfc_str(
            synthetic_icgem_degree4_fixture(),
            max_degree,
            max_order,
        )
        .expect("finite synthetic ICGEM normalized field")
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
    fn tesseral_gravity_loads_normalized_harmonic_field_pin() {
        let field = load_wgs84_normalized_harmonic_field_fixture();
        assert_eq!(field.max_degree(), 2);
        assert_eq!(field.max_order(), 2);
        assert_eq!(field.tide_system(), TideSystem::TideFree);
        assert_eq!(field.coefficient_count(), 3);
        assert_eq!(field.storage_len(), 6);

        let normalized = field
            .normalized_degree_two_tesseral_coefficients()
            .expect("degree-2 field extracts");
        let coefficients = normalized
            .to_unnormalized()
            .expect("degree-2 field converts");
        assert_abs_diff_eq!(coefficients.c20(), -WGS84_J2, epsilon = 1.0e-18);

        let tesseral =
            TesseralGravity::from_normalized_field(WGS84_MU_M3_S2, WGS84_A_M, &field, 2, 0)
                .expect("normalized field builds tesseral gravity");
        let j2 = J2Gravity::wgs84();
        let position = Position3::new(7_200_000.0, -1_300_000.0, 900_000.0);
        let from_field = tesseral.gravity_eci_m_s2(position, SimTime::ZERO).unwrap();
        let direct_j2 = j2.gravity_eci_m_s2(position, SimTime::ZERO).unwrap();
        for axis in 0..3 {
            assert_abs_diff_eq!(from_field[axis], direct_j2[axis], epsilon = 1.0e-17);
        }
    }

    #[test]
    fn tesseral_normalized_harmonic_field_parses_egm2008_zonal_toml_pin() {
        let field = load_egm2008_normalized_zonal_degree6_fixture();

        assert_eq!(field.max_degree(), 6);
        assert_eq!(field.max_order(), 0);
        assert_eq!(field.tide_system(), TideSystem::TideFree);
        assert_eq!(field.coefficient_count(), 5);
        assert_eq!(field.storage_len(), 7);
        assert_abs_diff_eq!(
            field.coefficient(2, 0).unwrap().0,
            -WGS84_J2 / 5.0_f64.sqrt(),
            epsilon = 1.0e-20
        );
        assert_eq!(
            field.coefficient(2, 0).unwrap().1.to_bits(),
            0.0_f64.to_bits()
        );
        assert_eq!(field.coefficient(1, 0).unwrap(), (0.0, 0.0));
    }

    #[test]
    fn tesseral_normalized_harmonic_field_parses_synthetic_general_toml_pin() {
        let field = load_synthetic_normalized_degree4_field_fixture();

        assert_eq!(field.max_degree(), 4);
        assert_eq!(field.max_order(), 3);
        assert_eq!(field.tide_system(), TideSystem::TideFree);
        assert_eq!(field.coefficient_count(), 8);
        assert_eq!(field.storage_len(), 14);
        assert_eq!(field.coefficient(3, 2).unwrap(), (0.0, 0.0));
        assert_eq!(field.coefficient(4, 1).unwrap(), (0.0, 0.0));
        assert_eq!(field.coefficient(4, 3).unwrap(), (7.0e-9, -5.0e-9));
        assert!(matches!(
            field.coefficient(4, 4),
            Err(PhysicsError::InvalidParameter { .. })
        ));

        let plan = HarmonicSynthesisPlan::for_normalized_field(
            &field,
            HarmonicTruncation::new(4, 3).unwrap(),
        )
        .unwrap();
        assert_eq!(plan.truncation(), HarmonicTruncation::new(4, 3).unwrap());
    }

    #[test]
    fn tesseral_normalized_harmonic_field_parses_synthetic_icgem_gfc_pin() {
        let toml_field = load_synthetic_normalized_degree4_field_fixture();
        let gfc_field = load_synthetic_icgem_degree4_field_fixture(4, 3);

        assert_eq!(gfc_field.max_degree(), 4);
        assert_eq!(gfc_field.max_order(), 3);
        assert_eq!(gfc_field.tide_system(), TideSystem::TideFree);
        assert_eq!(gfc_field.coefficient_count(), 9);
        assert_eq!(gfc_field.storage_len(), 14);
        assert_eq!(gfc_field.coefficient(0, 0).unwrap(), (1.0, 0.0));
        assert_eq!(gfc_field.coefficient(4, 3).unwrap(), (7.0e-9, -5.0e-9));
        assert!(matches!(
            gfc_field.coefficient(4, 4),
            Err(PhysicsError::InvalidParameter { .. })
        ));

        for degree in 2..=4 {
            for order in 0..=degree.min(3) {
                assert_eq!(
                    gfc_field.coefficient(degree, order).unwrap(),
                    toml_field.coefficient(degree, order).unwrap()
                );
            }
        }

        let correction_field = gfc_field.without_central_term();
        assert_eq!(correction_field.coefficient_count(), 8);
        assert_eq!(correction_field.coefficient(0, 0).unwrap(), (0.0, 0.0));
        assert_eq!(
            correction_field.coefficient(4, 3).unwrap(),
            toml_field.coefficient(4, 3).unwrap()
        );

        let truncated = load_synthetic_icgem_degree4_field_fixture(3, 1);
        assert_eq!(truncated.max_degree(), 3);
        assert_eq!(truncated.max_order(), 1);
        assert_eq!(truncated.coefficient_count(), 5);
        assert_eq!(truncated.storage_len(), 7);
        assert_eq!(truncated.coefficient(3, 1).unwrap(), (-7.0e-7, 4.0e-7));
        assert!(matches!(
            truncated.coefficient(3, 3),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn tesseral_normalized_harmonic_field_parses_icgem_gfc_metadata_pin() {
        let parsed = NormalizedHarmonicField::from_icgem_gfc_str_with_metadata(
            synthetic_icgem_degree4_fixture(),
            4,
            3,
        )
        .unwrap();

        assert_eq!(
            parsed.gravity_constant_m3_s2().to_bits(),
            WGS84_MU_M3_S2.to_bits()
        );
        assert_eq!(parsed.reference_radius_m().to_bits(), WGS84_A_M.to_bits());
        assert_eq!(parsed.source_max_degree(), 4);
        assert_eq!(parsed.field().max_degree(), 4);
        assert_eq!(parsed.field().max_order(), 3);
        assert_eq!(parsed.field().tide_system(), TideSystem::TideFree);
        assert_eq!(parsed.field().coefficient(0, 0).unwrap(), (1.0, 0.0));

        let field = parsed.into_field();
        assert_eq!(field.coefficient_count(), 9);
        assert_eq!(field.coefficient(4, 3).unwrap(), (7.0e-9, -5.0e-9));
    }

    #[test]
    fn tesseral_normalized_harmonic_field_icgem_parser_rejects_bad_metadata() {
        let fixture = synthetic_icgem_degree4_fixture();

        let bad_norm = fixture.replace("norm fully_normalized", "norm unnormalized");
        assert!(matches!(
            NormalizedHarmonicField::from_icgem_gfc_str(&bad_norm, 4, 3),
            Err(PhysicsError::InvalidParameter { .. })
        ));

        let missing_tide = fixture.replace("tide_system tide_free\n", "");
        assert!(matches!(
            NormalizedHarmonicField::from_icgem_gfc_str(&missing_tide, 4, 3),
            Err(PhysicsError::InvalidParameter { .. })
        ));

        let missing_end = fixture.replace("end_of_head\n", "");
        assert!(matches!(
            NormalizedHarmonicField::from_icgem_gfc_str(&missing_end, 4, 3),
            Err(PhysicsError::InvalidParameter { .. })
        ));

        let dynamic_line = fixture.replace(
            "gfc 2 0 -4.8000000000000000D-04",
            "gfct 2 0 -4.8000000000000000D-04",
        );
        assert!(matches!(
            NormalizedHarmonicField::from_icgem_gfc_str(&dynamic_line, 4, 3),
            Err(PhysicsError::InvalidParameter { .. })
        ));

        let malformed_value =
            fixture.replace("gfc 2 1 1.7000000000000000D-06", "gfc 2 1 not-a-float");
        assert!(matches!(
            NormalizedHarmonicField::from_icgem_gfc_str(&malformed_value, 4, 3),
            Err(PhysicsError::InvalidParameter { .. })
        ));

        let missing_mu = fixture.replace("earth_gravity_constant 3.9860044180000000D+14\n", "");
        assert!(matches!(
            NormalizedHarmonicField::from_icgem_gfc_str_with_metadata(&missing_mu, 4, 3),
            Err(PhysicsError::InvalidParameter { .. })
        ));

        let bad_radius = fixture.replace(
            "radius 6.3781370000000000D+06",
            "radius -6.3781370000000000D+06",
        );
        assert!(matches!(
            NormalizedHarmonicField::from_icgem_gfc_str_with_metadata(&bad_radius, 4, 3),
            Err(PhysicsError::InvalidParameter { .. })
        ));

        let undersized_source = fixture.replace("max_degree 4", "max_degree 3");
        assert!(matches!(
            NormalizedHarmonicField::from_icgem_gfc_str_with_metadata(&undersized_source, 4, 3),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn tesseral_normalized_harmonic_field_toml_parser_rejects_bad_metadata() {
        let fixture = include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/gravity/egm2008-zonal-degree6-normalized-v1.toml"
        ));

        assert!(matches!(
            NormalizedHarmonicField::from_normalized_toml_str("not valid = ["),
            Err(PhysicsError::InvalidParameter { .. })
        ));

        let bad_normalization = fixture.replace("fully_normalized", "unnormalized");
        assert!(matches!(
            NormalizedHarmonicField::from_normalized_toml_str(&bad_normalization),
            Err(PhysicsError::InvalidParameter { .. })
        ));

        let bad_schema = fixture.replace(
            "openbmp.gravity.normalized-zonal.v1",
            "openbmp.gravity.unknown.v1",
        );
        assert!(matches!(
            NormalizedHarmonicField::from_normalized_toml_str(&bad_schema),
            Err(PhysicsError::InvalidParameter { .. })
        ));

        let bad_zonal_order = fixture.replace("max_order      = 0", "max_order      = 1");
        assert!(matches!(
            NormalizedHarmonicField::from_normalized_toml_str(&bad_zonal_order),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn tesseral_normalized_harmonic_field_defaults_missing_coefficients_to_zero() {
        let field = NormalizedHarmonicField::new(
            4,
            2,
            TideSystem::ZeroTide,
            [
                NormalizedHarmonicCoefficient::new(4, 2, 4.2e-9, -7.0e-10).unwrap(),
                NormalizedHarmonicCoefficient::new(2, 0, -4.0e-4, 0.0).unwrap(),
            ],
        )
        .unwrap();

        assert_eq!(field.coefficient_count(), 2);
        assert_eq!(field.storage_len(), 12);
        assert_eq!(field.coefficient(0, 0).unwrap(), (0.0, 0.0));
        assert_eq!(field.coefficient(2, 1).unwrap(), (0.0, 0.0));
        assert_eq!(field.coefficient(4, 2).unwrap(), (4.2e-9, -7.0e-10));
        assert!(matches!(
            field.coefficient(4, 3),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            field.coefficient(1, 2),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn tesseral_normalized_harmonic_field_iterates_slots_in_row_order() {
        let field = NormalizedHarmonicField::new(
            3,
            2,
            TideSystem::MeanTide,
            [
                NormalizedHarmonicCoefficient::new(2, 0, -4.0e-4, 0.0).unwrap(),
                NormalizedHarmonicCoefficient::new(3, 2, 4.2e-9, -7.0e-10).unwrap(),
            ],
        )
        .unwrap();

        let slots = field
            .coefficients()
            .map(|coefficient| {
                (
                    coefficient.degree(),
                    coefficient.order(),
                    coefficient.cbar(),
                    coefficient.sbar(),
                )
            })
            .collect::<Vec<_>>();

        assert_eq!(
            slots
                .iter()
                .map(|(degree, order, _, _)| (*degree, *order))
                .collect::<Vec<_>>(),
            vec![
                (0, 0),
                (1, 0),
                (1, 1),
                (2, 0),
                (2, 1),
                (2, 2),
                (3, 0),
                (3, 1),
                (3, 2),
            ]
        );
        assert_eq!(slots.len(), field.storage_len());
        assert_eq!(slots[3], (2, 0, -4.0e-4, 0.0));
        assert_eq!(slots[8], (3, 2, 4.2e-9, -7.0e-10));
        assert_eq!(slots[4], (2, 1, 0.0, 0.0));
        let mut iter = field.coefficients();
        assert_eq!(iter.len(), field.storage_len());
        assert!(iter.next().is_some());
        assert_eq!(iter.len(), field.storage_len() - 1);
    }

    #[test]
    fn tesseral_fully_normalized_scale_matches_degree_two_and_zonal_factors() {
        assert_abs_diff_eq!(
            fully_normalized_to_unnormalized_scale(2, 0).unwrap(),
            5.0_f64.sqrt(),
            epsilon = 1.0e-15
        );
        assert_abs_diff_eq!(
            fully_normalized_to_unnormalized_scale(2, 1).unwrap(),
            (5.0_f64 / 3.0).sqrt(),
            epsilon = 1.0e-15
        );
        assert_abs_diff_eq!(
            fully_normalized_to_unnormalized_scale(2, 2).unwrap(),
            (5.0_f64 / 12.0).sqrt(),
            epsilon = 1.0e-15
        );
        assert_abs_diff_eq!(
            fully_normalized_to_unnormalized_scale(6, 0).unwrap(),
            13.0_f64.sqrt(),
            epsilon = 1.0e-15
        );
        assert!(matches!(
            fully_normalized_to_unnormalized_scale(2, 3),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            fully_normalized_to_unnormalized_scale(FULLY_NORMALIZED_SCALE_MAX_DEGREE + 1, 0),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn tesseral_longitude_trigonometry_matches_direct_trig() {
        let longitude_rad = 0.73_f64;
        let table = HarmonicLongitudeTrigonometry::new(longitude_rad, 12).unwrap();

        assert_eq!(table.longitude_rad().to_bits(), longitude_rad.to_bits());
        assert_eq!(table.max_order(), 12);
        assert_eq!(table.harmonic(0).unwrap(), (1.0, 0.0));

        for order in 1..=12 {
            let order_u32 = u32::try_from(order).unwrap();
            let (expected_sine, expected_cosine) = (f64::from(order_u32) * longitude_rad).sin_cos();
            assert_abs_diff_eq!(
                table.cosine(order).unwrap(),
                expected_cosine,
                epsilon = 1.0e-14
            );
            assert_abs_diff_eq!(table.sine(order).unwrap(), expected_sine, epsilon = 1.0e-14);
            assert_abs_diff_eq!(
                table.harmonic(order).unwrap().0,
                expected_cosine,
                epsilon = 1.0e-14
            );
            assert_abs_diff_eq!(
                table.harmonic(order).unwrap().1,
                expected_sine,
                epsilon = 1.0e-14
            );
        }
    }

    #[test]
    fn tesseral_longitude_trigonometry_rejects_invalid_inputs() {
        assert!(matches!(
            HarmonicLongitudeTrigonometry::new(f64::NAN, 0),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            HarmonicLongitudeTrigonometry::new(0.0, HARMONIC_LONGITUDE_MAX_ORDER + 1),
            Err(PhysicsError::InvalidParameter { .. })
        ));

        let table = HarmonicLongitudeTrigonometry::new(-1.2, 3).unwrap();
        assert!(matches!(
            table.harmonic(4),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            table.cosine(4),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            table.sine(4),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn tesseral_pines_longitude_polynomials_match_complex_powers() {
        let table = PinesLongitudePolynomials::new(0.6, 0.8, 4).unwrap();

        assert_eq!(table.s().to_bits(), 0.6_f64.to_bits());
        assert_eq!(table.t().to_bits(), 0.8_f64.to_bits());
        assert_eq!(table.max_order(), 4);
        assert_eq!(table.polynomial(0).unwrap(), (1.0, 0.0));
        assert_eq!(table.polynomial(1).unwrap(), (0.6, 0.8));
        assert_abs_diff_eq!(table.real(2).unwrap(), -0.28, epsilon = 1.0e-15);
        assert_abs_diff_eq!(table.imaginary(2).unwrap(), 0.96, epsilon = 1.0e-15);

        for order in 0..=4 {
            let (real, imaginary) = table.polynomial(order).unwrap();
            assert_abs_diff_eq!(real * real + imaginary * imaginary, 1.0, epsilon = 1.0e-15);
        }
    }

    #[test]
    fn tesseral_pines_longitude_polynomials_are_finite_at_pole() {
        let table = PinesLongitudePolynomials::new(0.0, 0.0, 4).unwrap();

        assert_eq!(table.polynomial(0).unwrap(), (1.0, 0.0));
        for order in 1..=4 {
            assert_eq!(table.polynomial(order).unwrap(), (0.0, 0.0));
        }
    }

    #[test]
    fn tesseral_pines_longitude_polynomials_reject_invalid_inputs() {
        assert!(matches!(
            PinesLongitudePolynomials::new(f64::NAN, 0.0, 0),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            PinesLongitudePolynomials::new(1.0, 1.0, 0),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            PinesLongitudePolynomials::new(0.0, 0.0, HARMONIC_LONGITUDE_MAX_ORDER + 1),
            Err(PhysicsError::InvalidParameter { .. })
        ));

        let table = PinesLongitudePolynomials::new(0.3, -0.4, 2).unwrap();
        assert!(matches!(
            table.polynomial(3),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            table.real(3),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            table.imaginary(3),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn tesseral_harmonic_truncation_validates_degree_order_envelopes() {
        let truncation = HarmonicTruncation::new(12, 5).unwrap();
        assert_eq!(truncation.degree(), 12);
        assert_eq!(truncation.order(), 5);

        let bounded = HarmonicTruncation::within_envelope(4, 2, 6, 3).unwrap();
        assert_eq!(bounded, HarmonicTruncation::new(4, 2).unwrap());

        let field = NormalizedHarmonicField::new(3, 1, TideSystem::TideFree, []).unwrap();
        let field_truncation = HarmonicTruncation::for_normalized_field(&field, 3, 1).unwrap();
        assert_eq!(field_truncation.degree(), 3);
        assert_eq!(field_truncation.order(), 1);

        assert!(matches!(
            HarmonicTruncation::new(2, 3),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            HarmonicTruncation::within_envelope(7, 0, 6, 3),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            HarmonicTruncation::within_envelope(4, 4, 6, 3),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            HarmonicTruncation::within_envelope(0, 0, 0, 1),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            HarmonicTruncation::for_normalized_field(&field, 3, 2),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn tesseral_harmonic_synthesis_plan_validates_runtime_tiers() {
        let tier = HarmonicSynthesisTier::Egm2008Degree70;
        assert_eq!(tier.degree(), HARMONIC_SYNTHESIS_EGM2008_DEGREE_70);
        assert_eq!(tier.order(), HARMONIC_SYNTHESIS_EGM2008_DEGREE_70);
        assert_eq!(
            tier.truncation().unwrap(),
            HarmonicTruncation::new(
                HARMONIC_SYNTHESIS_EGM2008_DEGREE_70,
                HARMONIC_SYNTHESIS_EGM2008_DEGREE_70
            )
            .unwrap()
        );

        let field_70 = NormalizedHarmonicField::new(
            HARMONIC_SYNTHESIS_EGM2008_DEGREE_70,
            HARMONIC_SYNTHESIS_EGM2008_DEGREE_70,
            TideSystem::TideFree,
            [],
        )
        .unwrap();
        let plan =
            HarmonicSynthesisPlan::egm2008_tier_for_normalized_field(&field_70, tier).unwrap();
        assert_eq!(plan.tier(), Some(tier));
        assert_eq!(plan.degree(), HARMONIC_SYNTHESIS_EGM2008_DEGREE_70);
        assert_eq!(plan.order(), HARMONIC_SYNTHESIS_EGM2008_DEGREE_70);

        let custom = HarmonicSynthesisPlan::for_normalized_field(
            &field_70,
            HarmonicTruncation::new(12, 4).unwrap(),
        )
        .unwrap();
        assert_eq!(custom.tier(), None);
        assert_eq!(custom.truncation(), HarmonicTruncation::new(12, 4).unwrap());

        assert!(matches!(
            HarmonicSynthesisPlan::egm2008_tier_for_normalized_field(
                &field_70,
                HarmonicSynthesisTier::Egm2008Degree120
            ),
            Err(PhysicsError::InvalidParameter { .. })
        ));

        let field_360 = NormalizedHarmonicField::new(
            HARMONIC_SYNTHESIS_EGM2008_DEGREE_360,
            HARMONIC_SYNTHESIS_EGM2008_DEGREE_360,
            TideSystem::TideFree,
            [],
        )
        .unwrap();
        let plan_360 = HarmonicSynthesisPlan::egm2008_tier_for_normalized_field(
            &field_360,
            HarmonicSynthesisTier::Egm2008Degree360,
        )
        .unwrap();
        assert_eq!(
            plan_360.truncation(),
            HarmonicTruncation::new(
                HARMONIC_SYNTHESIS_EGM2008_DEGREE_360,
                HARMONIC_SYNTHESIS_EGM2008_DEGREE_360
            )
            .unwrap()
        );

        let over_degree_field = NormalizedHarmonicField::new(
            PINES_LEGENDRE_MAX_DEGREE + 1,
            0,
            TideSystem::TideFree,
            [],
        )
        .unwrap();
        assert!(matches!(
            HarmonicSynthesisPlan::for_normalized_field(
                &over_degree_field,
                HarmonicTruncation::new(PINES_LEGENDRE_MAX_DEGREE + 1, 0).unwrap()
            ),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn tesseral_pines_legendre_table_matches_low_degree_closed_form() {
        let u = 0.37_f64;
        let table = PinesLegendreTable::new(u, 4, 4).unwrap();

        assert_eq!(table.u().to_bits(), u.to_bits());
        assert_eq!(table.max_degree(), 4);
        assert_eq!(table.max_order(), 4);
        assert_eq!(table.storage_len(), 15);

        assert_abs_diff_eq!(table.value(0, 0).unwrap(), 1.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(
            table.value(1, 0).unwrap(),
            3.0_f64.sqrt() * u,
            epsilon = 1.0e-15
        );
        assert_abs_diff_eq!(
            table.value(1, 1).unwrap(),
            3.0_f64.sqrt(),
            epsilon = 1.0e-15
        );
        assert_abs_diff_eq!(
            table.value(2, 0).unwrap(),
            0.5 * 5.0_f64.sqrt() * (3.0 * u * u - 1.0),
            epsilon = 1.0e-15
        );
        assert_abs_diff_eq!(
            table.value(2, 1).unwrap(),
            15.0_f64.sqrt() * u,
            epsilon = 1.0e-15
        );
        assert_abs_diff_eq!(
            table.value(2, 2).unwrap(),
            0.5 * 15.0_f64.sqrt(),
            epsilon = 1.0e-15
        );
        assert!(matches!(
            table.value(3, 4),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn tesseral_pines_legendre_table_rejects_invalid_inputs() {
        assert!(matches!(
            PinesLegendreTable::new(f64::NAN, 0, 0),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            PinesLegendreTable::new(1.01, 0, 0),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            PinesLegendreTable::new(0.0, 2, 3),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            PinesLegendreTable::new(0.0, PINES_LEGENDRE_MAX_DEGREE + 1, 0),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn tesseral_pines_potential_sum_matches_degree_two_cartesian_terms() {
        let c20 = -1.2e-3;
        let c21 = 2.0e-6;
        let s21 = -3.0e-6;
        let c22 = 4.0e-6;
        let s22 = -5.0e-6;
        let field = NormalizedHarmonicField::new(
            2,
            2,
            TideSystem::TideFree,
            [
                NormalizedHarmonicCoefficient::new(
                    2,
                    0,
                    c20 / fully_normalized_to_unnormalized_scale(2, 0).unwrap(),
                    0.0,
                )
                .unwrap(),
                NormalizedHarmonicCoefficient::new(
                    2,
                    1,
                    c21 / fully_normalized_to_unnormalized_scale(2, 1).unwrap(),
                    s21 / fully_normalized_to_unnormalized_scale(2, 1).unwrap(),
                )
                .unwrap(),
                NormalizedHarmonicCoefficient::new(
                    2,
                    2,
                    c22 / fully_normalized_to_unnormalized_scale(2, 2).unwrap(),
                    s22 / fully_normalized_to_unnormalized_scale(2, 2).unwrap(),
                )
                .unwrap(),
            ],
        )
        .unwrap();
        let position: Vector3<f64> = Vector3::new(7_100_000.0, -800_000.0, 1_200_000.0);
        let r2 = position.dot(&position);
        let r = r2.sqrt();
        let x = position.x;
        let y = position.y;
        let z = position.z;
        let numerator = 0.5 * c20 * (2.0 * z * z - x * x - y * y)
            + 3.0 * z * (c21 * x + s21 * y)
            + 3.0 * c22 * (x * x - y * y)
            + 6.0 * s22 * x * y;
        let expected = (WGS84_A_M / r).powi(2) * numerator / r2;

        let sum = field
            .pines_dimensionless_potential_sum(
                position,
                WGS84_A_M,
                HarmonicTruncation::new(2, 2).unwrap(),
            )
            .unwrap();

        assert_eq!(sum.truncation(), HarmonicTruncation::new(2, 2).unwrap());
        assert_eq!(sum.term_count(), 6);
        assert_abs_diff_eq!(sum.dimensionless_correction(), expected, epsilon = 1.0e-18);
    }

    #[test]
    fn tesseral_pines_potential_sum_rejects_invalid_geometry_or_truncation() {
        let field = NormalizedHarmonicField::new(
            2,
            0,
            TideSystem::TideFree,
            [NormalizedHarmonicCoefficient::new(2, 0, -WGS84_J2 / 5.0_f64.sqrt(), 0.0).unwrap()],
        )
        .unwrap();
        let truncation = HarmonicTruncation::new(2, 0).unwrap();

        assert!(matches!(
            PinesSynthesisPoint::new(Vector3::new(0.0, 0.0, 0.0), WGS84_A_M),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            PinesSynthesisPoint::new(Vector3::new(f64::NAN, 0.0, 0.0), WGS84_A_M),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            field.pines_dimensionless_potential_sum(
                Vector3::new(7_000_000.0, 0.0, 0.0),
                0.0,
                truncation
            ),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            field.pines_dimensionless_potential_sum(
                Vector3::new(7_000_000.0, 0.0, 0.0),
                WGS84_A_M,
                HarmonicTruncation::new(2, 1).unwrap()
            ),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn tesseral_gottlieb_potential_sum_matches_pines_scalar_sum() {
        let truncation = HarmonicTruncation::new(4, 3).unwrap();
        let positions = [
            Vector3::new(7_100_000.0, -800_000.0, 1_200_000.0),
            Vector3::new(WGS84_A_M + 300_000.0, 0.0, 0.0),
            Vector3::new(1.0, -2.0, WGS84_A_M + 500_000.0),
        ];

        for field in [
            load_synthetic_normalized_degree4_field_fixture(),
            load_synthetic_icgem_degree4_field_fixture(4, 3),
        ] {
            for position in positions {
                let pines = field
                    .pines_dimensionless_potential_sum(position, WGS84_A_M, truncation)
                    .unwrap();
                let gottlieb = field
                    .gottlieb_dimensionless_potential_sum(position, WGS84_A_M, truncation)
                    .unwrap();

                assert_eq!(gottlieb.truncation(), pines.truncation());
                assert_eq!(gottlieb.term_count(), pines.term_count());
                assert_eq!(gottlieb.term_count(), 14);
                assert_abs_diff_eq!(
                    gottlieb.dimensionless_correction(),
                    pines.dimensionless_correction(),
                    epsilon = 1.0e-15
                );
            }
        }
    }

    #[test]
    fn tesseral_gottlieb_potential_sum_rejects_invalid_geometry_or_truncation() {
        let field = NormalizedHarmonicField::new(
            2,
            0,
            TideSystem::TideFree,
            [NormalizedHarmonicCoefficient::new(2, 0, -WGS84_J2 / 5.0_f64.sqrt(), 0.0).unwrap()],
        )
        .unwrap();
        let truncation = HarmonicTruncation::new(2, 0).unwrap();

        assert!(matches!(
            field.gottlieb_dimensionless_potential_sum(
                Vector3::new(0.0, 0.0, 0.0),
                WGS84_A_M,
                truncation
            ),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            field.gottlieb_dimensionless_potential_sum(
                Vector3::new(f64::NAN, 0.0, 0.0),
                WGS84_A_M,
                truncation
            ),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            field.gottlieb_dimensionless_potential_sum(
                Vector3::new(7_000_000.0, 0.0, 0.0),
                0.0,
                truncation
            ),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            field.gottlieb_dimensionless_potential_sum(
                Vector3::new(7_000_000.0, 0.0, 0.0),
                WGS84_A_M,
                HarmonicTruncation::new(2, 1).unwrap()
            ),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn tesseral_gottlieb_finite_difference_acceleration_matches_degree_two_analytic_terms() {
        let coefficients = DegreeTwoTesseralCoefficients::new(
            -1.2e-3,
            2.0e-6,
            -3.0e-6,
            4.0e-6,
            -5.0e-6,
            TideSystem::TideFree,
        )
        .unwrap();
        let field = normalized_field_from_degree_two(coefficients);
        let position: Vector3<f64> = Vector3::new(7_100_000.0, -800_000.0, 1_200_000.0);
        let r2 = position.dot(&position);
        let r_norm = r2.sqrt();
        let expected = degree_two_tesseral_perturbation_eci(
            position,
            r2,
            r_norm,
            WGS84_MU_M3_S2,
            WGS84_A_M,
            coefficients,
            2,
        );

        let acceleration = field
            .gottlieb_potential_correction_acceleration_finite_difference_m_s2(
                WGS84_MU_M3_S2,
                position,
                WGS84_A_M,
                HarmonicTruncation::new(2, 2).unwrap(),
                10.0,
            )
            .unwrap();

        for axis in 0..3 {
            assert_abs_diff_eq!(acceleration[axis], expected[axis], epsilon = 5.0e-9);
        }
    }

    #[test]
    fn tesseral_gottlieb_finite_difference_acceleration_rejects_invalid_inputs() {
        let field = NormalizedHarmonicField::new(
            2,
            0,
            TideSystem::TideFree,
            [NormalizedHarmonicCoefficient::new(2, 0, -WGS84_J2 / 5.0_f64.sqrt(), 0.0).unwrap()],
        )
        .unwrap();
        let position = Vector3::new(7_000_000.0, 0.0, 0.0);
        let truncation = HarmonicTruncation::new(2, 0).unwrap();

        assert!(matches!(
            field.gottlieb_potential_correction_acceleration_finite_difference_m_s2(
                0.0, position, WGS84_A_M, truncation, 10.0
            ),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            field.gottlieb_potential_correction_acceleration_finite_difference_m_s2(
                WGS84_MU_M3_S2,
                position,
                WGS84_A_M,
                truncation,
                0.0
            ),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            field.gottlieb_potential_correction_acceleration_finite_difference_m_s2(
                WGS84_MU_M3_S2,
                Vector3::new(0.0, 0.0, 0.0),
                WGS84_A_M,
                truncation,
                10.0
            ),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            field.gottlieb_potential_correction_acceleration_finite_difference_m_s2(
                WGS84_MU_M3_S2,
                position,
                WGS84_A_M,
                HarmonicTruncation::new(2, 1).unwrap(),
                10.0
            ),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn tesseral_pines_finite_difference_acceleration_matches_degree_two_analytic_terms() {
        let coefficients = DegreeTwoTesseralCoefficients::new(
            -1.2e-3,
            2.0e-6,
            -3.0e-6,
            4.0e-6,
            -5.0e-6,
            TideSystem::TideFree,
        )
        .unwrap();
        let field = NormalizedHarmonicField::new(
            2,
            2,
            TideSystem::TideFree,
            [
                NormalizedHarmonicCoefficient::new(
                    2,
                    0,
                    coefficients.c20() / fully_normalized_to_unnormalized_scale(2, 0).unwrap(),
                    0.0,
                )
                .unwrap(),
                NormalizedHarmonicCoefficient::new(
                    2,
                    1,
                    coefficients.c21() / fully_normalized_to_unnormalized_scale(2, 1).unwrap(),
                    coefficients.s21() / fully_normalized_to_unnormalized_scale(2, 1).unwrap(),
                )
                .unwrap(),
                NormalizedHarmonicCoefficient::new(
                    2,
                    2,
                    coefficients.c22() / fully_normalized_to_unnormalized_scale(2, 2).unwrap(),
                    coefficients.s22() / fully_normalized_to_unnormalized_scale(2, 2).unwrap(),
                )
                .unwrap(),
            ],
        )
        .unwrap();
        let position: Vector3<f64> = Vector3::new(7_100_000.0, -800_000.0, 1_200_000.0);
        let r2 = position.dot(&position);
        let r_norm = r2.sqrt();
        let expected = degree_two_tesseral_perturbation_eci(
            position,
            r2,
            r_norm,
            WGS84_MU_M3_S2,
            WGS84_A_M,
            coefficients,
            2,
        );

        let acceleration = field
            .pines_potential_correction_acceleration_finite_difference_m_s2(
                WGS84_MU_M3_S2,
                position,
                WGS84_A_M,
                HarmonicTruncation::new(2, 2).unwrap(),
                10.0,
            )
            .unwrap();

        for axis in 0..3 {
            assert_abs_diff_eq!(acceleration[axis], expected[axis], epsilon = 5.0e-9);
        }
    }

    #[test]
    fn tesseral_pines_finite_difference_acceleration_rejects_invalid_inputs() {
        let field = NormalizedHarmonicField::new(
            2,
            0,
            TideSystem::TideFree,
            [NormalizedHarmonicCoefficient::new(2, 0, -WGS84_J2 / 5.0_f64.sqrt(), 0.0).unwrap()],
        )
        .unwrap();
        let position = Vector3::new(7_000_000.0, 0.0, 0.0);
        let truncation = HarmonicTruncation::new(2, 0).unwrap();

        assert!(matches!(
            field.pines_potential_correction_acceleration_finite_difference_m_s2(
                0.0, position, WGS84_A_M, truncation, 10.0
            ),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            field.pines_potential_correction_acceleration_finite_difference_m_s2(
                WGS84_MU_M3_S2,
                position,
                WGS84_A_M,
                truncation,
                0.0
            ),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            field.pines_potential_correction_acceleration_finite_difference_m_s2(
                WGS84_MU_M3_S2,
                Vector3::new(0.0, 0.0, 0.0),
                WGS84_A_M,
                truncation,
                10.0
            ),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            field.pines_potential_correction_acceleration_finite_difference_m_s2(
                WGS84_MU_M3_S2,
                position,
                WGS84_A_M,
                HarmonicTruncation::new(2, 1).unwrap(),
                10.0
            ),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn tesseral_finite_difference_acceleration_gradient_matches_degree_two_analytic_terms() {
        let coefficients = DegreeTwoTesseralCoefficients::new(
            -1.2e-3,
            2.0e-6,
            -3.0e-6,
            4.0e-6,
            -5.0e-6,
            TideSystem::TideFree,
        )
        .unwrap();
        let field = normalized_field_from_degree_two(coefficients);
        let position: Vector3<f64> = Vector3::new(7_100_000.0, -800_000.0, 1_200_000.0);
        let truncation = HarmonicTruncation::new(2, 2).unwrap();
        let gradient_step_m = 100.0;
        let expected = finite_difference_acceleration_gradient_s2(
            position,
            gradient_step_m,
            |candidate| {
                let r2 = candidate.dot(&candidate);
                let r_norm = r2.sqrt();
                Ok(degree_two_tesseral_perturbation_eci(
                    candidate,
                    r2,
                    r_norm,
                    WGS84_MU_M3_S2,
                    WGS84_A_M,
                    coefficients,
                    2,
                ))
            },
            "test gradient step must be finite",
            "test acceleration gradient must be finite",
        )
        .unwrap();

        let pines = field
            .pines_potential_correction_acceleration_gradient_finite_difference_s2(
                WGS84_MU_M3_S2,
                position,
                WGS84_A_M,
                truncation,
                10.0,
                gradient_step_m,
            )
            .unwrap();
        let gottlieb = field
            .gottlieb_potential_correction_acceleration_gradient_finite_difference_s2(
                WGS84_MU_M3_S2,
                position,
                WGS84_A_M,
                truncation,
                10.0,
                gradient_step_m,
            )
            .unwrap();

        for row in 0..3 {
            for col in 0..3 {
                assert_abs_diff_eq!(pines[(row, col)], expected[(row, col)], epsilon = 1.0e-14);
                assert_abs_diff_eq!(
                    gottlieb[(row, col)],
                    expected[(row, col)],
                    epsilon = 1.0e-14
                );
            }
        }
    }

    #[test]
    fn tesseral_pines_and_gottlieb_finite_difference_acceleration_gradients_match() {
        let field = load_synthetic_normalized_degree4_field_fixture();
        let position = Vector3::new(7_100_000.0, -800_000.0, 1_200_000.0);
        let truncation = HarmonicTruncation::new(4, 3).unwrap();

        let pines = field
            .pines_potential_correction_acceleration_gradient_finite_difference_s2(
                WGS84_MU_M3_S2,
                position,
                WGS84_A_M,
                truncation,
                10.0,
                100.0,
            )
            .unwrap();
        let gottlieb = field
            .gottlieb_potential_correction_acceleration_gradient_finite_difference_s2(
                WGS84_MU_M3_S2,
                position,
                WGS84_A_M,
                truncation,
                10.0,
                100.0,
            )
            .unwrap();

        for row in 0..3 {
            for col in 0..3 {
                assert!(pines[(row, col)].is_finite());
                assert_abs_diff_eq!(pines[(row, col)], gottlieb[(row, col)], epsilon = 2.0e-15);
            }
        }
    }

    #[test]
    fn tesseral_finite_difference_acceleration_gradient_rejects_invalid_inputs() {
        let field = NormalizedHarmonicField::new(
            2,
            0,
            TideSystem::TideFree,
            [NormalizedHarmonicCoefficient::new(2, 0, -WGS84_J2 / 5.0_f64.sqrt(), 0.0).unwrap()],
        )
        .unwrap();
        let position = Vector3::new(7_000_000.0, 0.0, 0.0);
        let truncation = HarmonicTruncation::new(2, 0).unwrap();

        assert!(matches!(
            field.pines_potential_correction_acceleration_gradient_finite_difference_s2(
                WGS84_MU_M3_S2,
                position,
                WGS84_A_M,
                truncation,
                0.0,
                100.0
            ),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            field.pines_potential_correction_acceleration_gradient_finite_difference_s2(
                WGS84_MU_M3_S2,
                position,
                WGS84_A_M,
                truncation,
                10.0,
                0.0
            ),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            field.gottlieb_potential_correction_acceleration_gradient_finite_difference_s2(
                WGS84_MU_M3_S2,
                Vector3::new(0.0, 0.0, 0.0),
                WGS84_A_M,
                truncation,
                10.0,
                100.0
            ),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            field.gottlieb_potential_correction_acceleration_gradient_finite_difference_s2(
                WGS84_MU_M3_S2,
                position,
                WGS84_A_M,
                HarmonicTruncation::new(2, 1).unwrap(),
                10.0,
                100.0
            ),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn finite_difference_pines_gravity_matches_degree_two_tesseral_model() {
        let coefficients = DegreeTwoTesseralCoefficients::new(
            -1.2e-3,
            2.0e-6,
            -3.0e-6,
            4.0e-6,
            -5.0e-6,
            TideSystem::TideFree,
        )
        .unwrap();
        let field = normalized_field_from_degree_two(coefficients);
        let pines = FiniteDifferencePinesGravity::new(
            WGS84_MU_M3_S2,
            WGS84_A_M,
            field,
            HarmonicTruncation::new(2, 2).unwrap(),
            10.0,
        )
        .unwrap();
        let tesseral = TesseralGravity::new(WGS84_MU_M3_S2, WGS84_A_M, coefficients, 2, 2).unwrap();
        let position = Position3::new(7_100_000.0, -800_000.0, 1_200_000.0);

        let pines_acceleration = pines.gravity_eci_m_s2(position, SimTime::ZERO).unwrap();
        let tesseral_acceleration = tesseral.gravity_eci_m_s2(position, SimTime::ZERO).unwrap();

        for axis in 0..3 {
            assert_abs_diff_eq!(
                pines_acceleration[axis],
                tesseral_acceleration[axis],
                epsilon = 5.0e-9
            );
        }
    }

    #[test]
    fn finite_difference_pines_gravity_zero_field_reduces_to_point_mass() {
        let field = NormalizedHarmonicField::new(0, 0, TideSystem::TideFree, []).unwrap();
        let pines = FiniteDifferencePinesGravity::new(
            WGS84_MU_M3_S2,
            WGS84_A_M,
            field,
            HarmonicTruncation::new(0, 0).unwrap(),
            10.0,
        )
        .unwrap();
        let point_mass = PointMassGravity::wgs84();
        let position = Position3::new(7_200_000.0, -900_000.0, 300_000.0);

        let pines_acceleration = pines.gravity_eci_m_s2(position, SimTime::ZERO).unwrap();
        let point_mass_acceleration = point_mass
            .gravity_eci_m_s2(position, SimTime::ZERO)
            .unwrap();

        for axis in 0..3 {
            assert_eq!(
                pines_acceleration[axis].to_bits(),
                point_mass_acceleration[axis].to_bits()
            );
        }
    }

    #[test]
    fn finite_difference_pines_gravity_zero_field_gradient_reduces_to_point_mass() {
        let field = NormalizedHarmonicField::new(0, 0, TideSystem::TideFree, []).unwrap();
        let pines = FiniteDifferencePinesGravity::new(
            WGS84_MU_M3_S2,
            WGS84_A_M,
            field,
            HarmonicTruncation::new(0, 0).unwrap(),
            10.0,
        )
        .unwrap();
        let position = Position3::new(7_200_000.0, -900_000.0, 300_000.0);

        let gradient = pines.acceleration_gradient_eci_s2(position, 100.0).unwrap();
        let expected =
            point_mass_acceleration_gradient_s2(WGS84_MU_M3_S2, position.vector).unwrap();

        for row in 0..3 {
            for col in 0..3 {
                assert_eq!(
                    gradient[(row, col)].to_bits(),
                    expected[(row, col)].to_bits()
                );
            }
        }
    }

    #[test]
    fn finite_difference_pines_gravity_accepts_full_icgem_field_after_central_strip() {
        let full_field = load_synthetic_icgem_degree4_field_fixture(4, 3);
        let correction_field = load_synthetic_normalized_degree4_field_fixture();
        let truncation = HarmonicTruncation::new(4, 3).unwrap();
        assert!(matches!(
            FiniteDifferencePinesGravity::new(
                WGS84_MU_M3_S2,
                WGS84_A_M,
                full_field.clone(),
                truncation,
                10.0
            ),
            Err(PhysicsError::InvalidParameter { .. })
        ));

        let from_full = FiniteDifferencePinesGravity::new_from_full_normalized_field(
            WGS84_MU_M3_S2,
            WGS84_A_M,
            full_field,
            truncation,
            10.0,
        )
        .unwrap();
        let from_correction = FiniteDifferencePinesGravity::new(
            WGS84_MU_M3_S2,
            WGS84_A_M,
            correction_field,
            truncation,
            10.0,
        )
        .unwrap();
        assert_eq!(from_full.field().coefficient(0, 0).unwrap(), (0.0, 0.0));
        let position = Position3::new(7_100_000.0, -800_000.0, 1_200_000.0);
        let from_full_acceleration = from_full.gravity_eci_m_s2(position, SimTime::ZERO).unwrap();
        let from_correction_acceleration = from_correction
            .gravity_eci_m_s2(position, SimTime::ZERO)
            .unwrap();

        for axis in 0..3 {
            assert_eq!(
                from_full_acceleration[axis].to_bits(),
                from_correction_acceleration[axis].to_bits()
            );
        }
    }

    #[test]
    fn finite_difference_pines_gravity_builds_from_icgem_gfc_metadata() {
        let from_gfc = FiniteDifferencePinesGravity::new_from_icgem_gfc_str(
            synthetic_icgem_degree4_fixture(),
            4,
            3,
            10.0,
        )
        .unwrap();
        let from_full = FiniteDifferencePinesGravity::new_from_full_normalized_field(
            WGS84_MU_M3_S2,
            WGS84_A_M,
            load_synthetic_icgem_degree4_field_fixture(4, 3),
            HarmonicTruncation::new(4, 3).unwrap(),
            10.0,
        )
        .unwrap();

        assert_eq!(from_gfc.mu_m3_s2().to_bits(), WGS84_MU_M3_S2.to_bits());
        assert_eq!(from_gfc.reference_radius_m().to_bits(), WGS84_A_M.to_bits());
        assert_eq!(
            from_gfc.truncation(),
            HarmonicTruncation::new(4, 3).unwrap()
        );
        assert_eq!(from_gfc.field().coefficient(0, 0).unwrap(), (0.0, 0.0));
        assert_eq!(from_gfc.field().coefficient_count(), 8);

        let position = Position3::new(7_100_000.0, -800_000.0, 1_200_000.0);
        let from_gfc_acceleration = from_gfc.gravity_eci_m_s2(position, SimTime::ZERO).unwrap();
        let from_full_acceleration = from_full.gravity_eci_m_s2(position, SimTime::ZERO).unwrap();

        for axis in 0..3 {
            assert_eq!(
                from_gfc_acceleration[axis].to_bits(),
                from_full_acceleration[axis].to_bits()
            );
        }
    }

    #[test]
    fn finite_difference_pines_gravity_accepts_resolved_runtime_tier() {
        let field_70 = NormalizedHarmonicField::new(
            HARMONIC_SYNTHESIS_EGM2008_DEGREE_70,
            HARMONIC_SYNTHESIS_EGM2008_DEGREE_70,
            TideSystem::TideFree,
            [],
        )
        .unwrap();
        let tier_model = FiniteDifferencePinesGravity::new_with_synthesis_tier(
            WGS84_MU_M3_S2,
            WGS84_A_M,
            field_70.clone(),
            HarmonicSynthesisTier::Egm2008Degree70,
            10.0,
        )
        .unwrap();
        assert_eq!(
            tier_model.truncation(),
            HarmonicTruncation::new(
                HARMONIC_SYNTHESIS_EGM2008_DEGREE_70,
                HARMONIC_SYNTHESIS_EGM2008_DEGREE_70
            )
            .unwrap()
        );

        let custom_plan = HarmonicSynthesisPlan::for_normalized_field(
            &field_70,
            HarmonicTruncation::new(12, 4).unwrap(),
        )
        .unwrap();
        let custom_model = FiniteDifferencePinesGravity::new_with_synthesis_plan(
            WGS84_MU_M3_S2,
            WGS84_A_M,
            field_70.clone(),
            custom_plan,
            10.0,
        )
        .unwrap();
        assert_eq!(custom_model.truncation(), custom_plan.truncation());

        assert!(matches!(
            FiniteDifferencePinesGravity::new_with_synthesis_tier(
                WGS84_MU_M3_S2,
                WGS84_A_M,
                field_70,
                HarmonicSynthesisTier::Egm2008Degree120,
                10.0
            ),
            Err(PhysicsError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn finite_difference_pines_gravity_rejects_invalid_inputs() {
        let field = NormalizedHarmonicField::new(
            2,
            0,
            TideSystem::TideFree,
            [NormalizedHarmonicCoefficient::new(2, 0, -WGS84_J2 / 5.0_f64.sqrt(), 0.0).unwrap()],
        )
        .unwrap();
        let truncation = HarmonicTruncation::new(2, 0).unwrap();

        assert!(matches!(
            FiniteDifferencePinesGravity::new(0.0, WGS84_A_M, field.clone(), truncation, 10.0),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            FiniteDifferencePinesGravity::new(WGS84_MU_M3_S2, 0.0, field.clone(), truncation, 10.0),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            FiniteDifferencePinesGravity::new(
                WGS84_MU_M3_S2,
                WGS84_A_M,
                field.clone(),
                truncation,
                0.0
            ),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            FiniteDifferencePinesGravity::new(
                WGS84_MU_M3_S2,
                WGS84_A_M,
                field.clone(),
                HarmonicTruncation::new(2, 1).unwrap(),
                10.0
            ),
            Err(PhysicsError::InvalidParameter { .. })
        ));

        let central_field = NormalizedHarmonicField::new(
            0,
            0,
            TideSystem::TideFree,
            [NormalizedHarmonicCoefficient::new(0, 0, 1.0, 0.0).unwrap()],
        )
        .unwrap();
        assert!(matches!(
            FiniteDifferencePinesGravity::new(
                WGS84_MU_M3_S2,
                WGS84_A_M,
                central_field,
                HarmonicTruncation::new(0, 0).unwrap(),
                10.0
            ),
            Err(PhysicsError::InvalidParameter { .. })
        ));

        let model =
            FiniteDifferencePinesGravity::new(WGS84_MU_M3_S2, WGS84_A_M, field, truncation, 10.0)
                .unwrap();
        assert!(matches!(
            model.gravity_eci_m_s2(Position3::origin(), SimTime::ZERO),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn tesseral_normalized_harmonic_field_rejects_invalid_or_duplicate_coefficients() {
        assert!(NormalizedHarmonicCoefficient::new(2, 3, 1.0, 0.0).is_err());
        assert!(NormalizedHarmonicCoefficient::new(2, 0, 1.0, 1.0e-12).is_err());
        assert!(NormalizedHarmonicCoefficient::new(2, 1, f64::NAN, 0.0).is_err());
        assert!(NormalizedHarmonicField::new(2, 3, TideSystem::TideFree, []).is_err());

        let duplicate = [
            NormalizedHarmonicCoefficient::new(2, 0, -4.0e-4, 0.0).unwrap(),
            NormalizedHarmonicCoefficient::new(2, 0, -5.0e-4, 0.0).unwrap(),
        ];
        assert!(matches!(
            NormalizedHarmonicField::new(2, 2, TideSystem::TideFree, duplicate),
            Err(PhysicsError::InvalidParameter { .. })
        ));

        let out_of_range = [NormalizedHarmonicCoefficient::new(3, 0, -1.0e-6, 0.0).unwrap()];
        assert!(matches!(
            NormalizedHarmonicField::new(2, 2, TideSystem::TideFree, out_of_range),
            Err(PhysicsError::InvalidParameter { .. })
        ));
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
    fn egm2008_zonal_loads_normalized_degree6_pin() {
        let field = load_egm2008_normalized_zonal_degree6_fixture();
        assert_eq!(field.max_degree(), 6);
        assert_eq!(field.max_order(), 0);
        assert_eq!(field.tide_system(), TideSystem::TideFree);
        assert_eq!(field.coefficient_count(), 5);

        let from_field =
            Egm2008ZonalGravity::from_normalized_field(WGS84_MU_M3_S2, WGS84_A_M, &field, 6)
                .unwrap();
        let direct = Egm2008ZonalGravity::wgs84_egm2008_zonal();

        assert_eq!(from_field.degree(), direct.degree());
        let from_field_j = from_field.j_n();
        let direct_j = direct.j_n();
        for degree in 2..=6 {
            assert_abs_diff_eq!(
                from_field_j[degree - 2],
                direct_j[degree - 2],
                epsilon = 1.0e-21
            );
        }

        let positions = [
            at_xyz(7_200_000.0, -1_300_000.0, 900_000.0),
            at_xyz(WGS84_A_M * 0.6, 0.0, WGS84_A_M * 0.8),
        ];
        for position in positions {
            let from_field_accel = from_field
                .gravity_eci_m_s2(position, SimTime::ZERO)
                .unwrap();
            let direct_accel = direct.gravity_eci_m_s2(position, SimTime::ZERO).unwrap();
            for axis in 0..3 {
                assert_abs_diff_eq!(
                    from_field_accel[axis],
                    direct_accel[axis],
                    epsilon = 1.0e-15
                );
            }
        }
    }

    #[test]
    fn egm2008_zonal_from_normalized_field_defaults_missing_zonals_to_zero() {
        let field = NormalizedHarmonicField::new(
            6,
            0,
            TideSystem::TideFree,
            [NormalizedHarmonicCoefficient::new(2, 0, -WGS84_J2 / 5.0_f64.sqrt(), 0.0).unwrap()],
        )
        .unwrap();
        let zonal =
            Egm2008ZonalGravity::from_normalized_field(WGS84_MU_M3_S2, WGS84_A_M, &field, 4)
                .unwrap();

        let j_n = zonal.j_n();
        assert_abs_diff_eq!(j_n[0], WGS84_J2, epsilon = 1.0e-21);
        assert_eq!(j_n[1].to_bits(), 0.0_f64.to_bits());
        assert_eq!(j_n[2].to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn egm2008_zonal_from_normalized_field_rejects_bad_degree_envelope() {
        let field = NormalizedHarmonicField::new(
            2,
            0,
            TideSystem::TideFree,
            [NormalizedHarmonicCoefficient::new(2, 0, -WGS84_J2 / 5.0_f64.sqrt(), 0.0).unwrap()],
        )
        .unwrap();

        assert!(matches!(
            Egm2008ZonalGravity::from_normalized_field(WGS84_MU_M3_S2, WGS84_A_M, &field, 3),
            Err(PhysicsError::InvalidParameter { .. })
        ));
        assert!(matches!(
            Egm2008ZonalGravity::from_normalized_field(WGS84_MU_M3_S2, WGS84_A_M, &field, 1),
            Err(PhysicsError::InvalidParameter { .. })
        ));
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
