//! Deterministic contact dynamics primitives.
//!
//! `openbmp-contact` is the L2 contact-mechanics substrate. It owns
//! fail-closed, allocation-free primitives for half-space gap evaluation,
//! compliant normal forces, regularized Coulomb friction, explicit
//! sub-step stability checks, scalar stops/backlash/latches, and
//! contact-energy accounting. It does not depend on `openbmp-sim`,
//! `openbmp-runner`, or `openbmp-fc`; higher layers opt in by adapting these
//! primitives into force accumulators.
//!
//! The initial tier intentionally covers point/sphere contact against a
//! plane plus scalar mechanism constraints. Runner scenario wiring, gear-leg
//! assemblies, terrain decks, and implicit contact solvers are later work
//! packages.

#![forbid(unsafe_code)]
#![deny(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

pub mod backlash;
pub mod error;
pub mod latch;
pub mod stop;

pub use backlash::{BacklashFlank, BacklashGap, BacklashResponse};
pub use error::ContactError;
pub use latch::{LatchState, LatchTransition, LatchWindow, MonotoneLatch};
pub use stop::{ScalarStop, ScalarStopResponse, StopSide};

use error::{require_finite, require_non_negative, require_positive};

const DEFAULT_STABILITY_SAFETY_FRACTION: f64 = 0.628_318_530_717_958_6;

/// Three-component vector used by the contact substrate.
///
/// The crate uses fixed arrays instead of a linear-algebra dependency so the
/// core force laws remain lightweight and easy to adapt into multiple higher
/// layers.
pub type ContactVector3 = [f64; 3];

/// Infinite contact plane represented as `dot(position, normal) - offset = 0`.
///
/// Positive gap is the half-space in the direction of `normal`. For the
/// default ground plane `z = 0`, the normal is `[0, 0, 1]` and points upward.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HalfSpace {
    normal: ContactVector3,
    offset_m: f64,
}

impl HalfSpace {
    /// Creates a half-space from a finite, non-zero normal and plane offset.
    ///
    /// The normal is normalized during construction.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidVector`] if the normal is not finite or
    /// has zero length, and [`ContactError::InvalidParameter`] if the offset is
    /// not finite.
    pub fn new(normal: ContactVector3, offset_m: f64) -> Result<Self, ContactError> {
        let normal = unit_vector("normal", normal)?;
        let offset_m = require_finite("offset_m", offset_m)?;
        Ok(Self { normal, offset_m })
    }

    /// Returns the canonical fixed ground plane `z = 0`.
    #[must_use]
    pub const fn ground_z0() -> Self {
        Self {
            normal: [0.0, 0.0, 1.0],
            offset_m: 0.0,
        }
    }

    /// Returns the normalized half-space normal.
    #[must_use]
    pub const fn normal(self) -> ContactVector3 {
        self.normal
    }

    /// Returns the plane offset in meters.
    #[must_use]
    pub const fn offset_m(self) -> f64 {
        self.offset_m
    }
}

/// Contact geometry attached to a body-fixed point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ContactGeometry {
    /// The contact point itself touches the half-space.
    Point,
    /// A sphere touches when its surface reaches the half-space.
    Sphere {
        /// Sphere radius in meters.
        radius_m: f64,
    },
}

impl ContactGeometry {
    /// Creates a spherical contact geometry.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] if `radius_m` is not finite
    /// and non-negative.
    pub fn sphere(radius_m: f64) -> Result<Self, ContactError> {
        Ok(Self::Sphere {
            radius_m: require_non_negative("radius_m", radius_m)?,
        })
    }

    fn radius_m(self) -> Result<f64, ContactError> {
        match self {
            Self::Point => Ok(0.0),
            Self::Sphere { radius_m } => require_non_negative("radius_m", radius_m),
        }
    }
}

/// Signed gap and relative velocity at a contact point.
///
/// `gap_m < 0` means penetration. `normal_velocity_m_s > 0` means the bodies
/// are separating along the half-space normal.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactKinematics {
    /// Signed normal gap in meters.
    pub gap_m: f64,
    /// Relative normal velocity in meters per second.
    pub normal_velocity_m_s: f64,
    /// Tangential relative velocity in meters per second.
    pub tangential_velocity_m_s: ContactVector3,
}

impl ContactKinematics {
    /// Creates a kinematic contact state.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] or
    /// [`ContactError::InvalidVector`] when any component is non-finite.
    pub fn new(
        gap_m: f64,
        normal_velocity_m_s: f64,
        tangential_velocity_m_s: ContactVector3,
    ) -> Result<Self, ContactError> {
        Ok(Self {
            gap_m: require_finite("gap_m", gap_m)?,
            normal_velocity_m_s: require_finite("normal_velocity_m_s", normal_velocity_m_s)?,
            tangential_velocity_m_s: finite_vector(
                "tangential_velocity_m_s",
                tangential_velocity_m_s,
            )?,
        })
    }

    /// Returns normal penetration depth in meters.
    #[must_use]
    pub fn penetration_m(self) -> f64 {
        if self.gap_m < 0.0 { -self.gap_m } else { 0.0 }
    }

    /// Returns normal penetration rate in meters per second.
    ///
    /// Positive values mean the penetration depth is increasing.
    #[must_use]
    pub fn penetration_rate_m_s(self) -> f64 {
        -self.normal_velocity_m_s
    }

    /// Returns the tangential speed magnitude.
    #[must_use]
    pub fn tangential_speed_m_s(self) -> f64 {
        norm(self.tangential_velocity_m_s)
    }
}

/// Kinematics for a contact geometry against a half-space.
///
/// # Errors
///
/// Returns an error when the geometry radius or supplied vectors are invalid.
pub fn half_space_kinematics(
    half_space: HalfSpace,
    geometry: ContactGeometry,
    contact_center_position_m: ContactVector3,
    contact_center_velocity_m_s: ContactVector3,
) -> Result<ContactKinematics, ContactError> {
    let position = finite_vector("contact_center_position_m", contact_center_position_m)?;
    let velocity = finite_vector("contact_center_velocity_m_s", contact_center_velocity_m_s)?;
    let radius_m = geometry.radius_m()?;
    let normal = half_space.normal();
    let normal_velocity_m_s = dot(velocity, normal);
    let normal_velocity = scale(normal, normal_velocity_m_s);
    let tangential_velocity = sub(velocity, normal_velocity);
    ContactKinematics::new(
        dot(position, normal) - half_space.offset_m() - radius_m,
        normal_velocity_m_s,
        tangential_velocity,
    )
}

/// Result of a normal-force evaluation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NormalResponse {
    /// Non-negative normal force in newtons.
    pub normal_force_n: f64,
    /// Elastic potential energy stored in the normal law.
    pub elastic_energy_j: f64,
    /// Power dissipated by the normal damping law.
    pub damping_power_w: f64,
}

/// Kelvin-Voigt linear spring-damper normal contact.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KelvinVoigtNormal {
    stiffness_n_m: f64,
    damping_n_s_m: f64,
}

impl KelvinVoigtNormal {
    /// Creates a Kelvin-Voigt normal law.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] if stiffness is not positive
    /// or damping is negative/non-finite.
    pub fn new(stiffness_n_m: f64, damping_n_s_m: f64) -> Result<Self, ContactError> {
        Ok(Self {
            stiffness_n_m: require_positive("stiffness_n_m", stiffness_n_m)?,
            damping_n_s_m: require_non_negative("damping_n_s_m", damping_n_s_m)?,
        })
    }

    /// Evaluates the normal response for the supplied kinematics.
    ///
    /// The damping contribution is `-c * gap_dot`; forces are clamped to zero
    /// if the point is separated or if separating velocity exceeds the elastic
    /// term.
    #[must_use]
    pub fn evaluate(self, kin: ContactKinematics) -> NormalResponse {
        let x = kin.penetration_m();
        if x <= 0.0 {
            return no_normal_response();
        }
        let elastic_force_n = self.stiffness_n_m * x;
        let damping_force_n = -self.damping_n_s_m * kin.normal_velocity_m_s;
        let normal_force_n = (elastic_force_n + damping_force_n).max(0.0);
        let elastic_energy_j = 0.5 * self.stiffness_n_m * x * x;
        let damping_power_w = (self.damping_n_s_m * kin.penetration_rate_m_s().powi(2)).max(0.0);
        NormalResponse {
            normal_force_n,
            elastic_energy_j,
            damping_power_w,
        }
    }

    /// Returns the spring stiffness in newtons per meter.
    #[must_use]
    pub const fn stiffness_n_m(self) -> f64 {
        self.stiffness_n_m
    }
}

/// Undamped Hertzian `F = k x^(3/2)` normal contact.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HertzNormal {
    stiffness_n_m_3_2: f64,
}

impl HertzNormal {
    /// Creates an undamped Hertzian normal law.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] if stiffness is not positive.
    pub fn new(stiffness_n_m_3_2: f64) -> Result<Self, ContactError> {
        Ok(Self {
            stiffness_n_m_3_2: require_positive("stiffness_n_m_3_2", stiffness_n_m_3_2)?,
        })
    }

    /// Evaluates the normal response for the supplied kinematics.
    #[must_use]
    pub fn evaluate(self, kin: ContactKinematics) -> NormalResponse {
        let x = kin.penetration_m();
        if x <= 0.0 {
            return no_normal_response();
        }
        let x_3_2 = x * x.sqrt();
        NormalResponse {
            normal_force_n: self.stiffness_n_m_3_2 * x_3_2,
            elastic_energy_j: 0.4 * self.stiffness_n_m_3_2 * x_3_2 * x,
            damping_power_w: 0.0,
        }
    }
}

/// Hunt-Crossley nonlinear contact with restitution-derived damping factor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HuntCrossleyNormal {
    stiffness_n_m_3_2: f64,
    damping_factor_s_m: f64,
}

impl HuntCrossleyNormal {
    /// Creates a Hunt-Crossley law from impact restitution and reference speed.
    ///
    /// The damping factor follows the common first-order relation
    /// `alpha = 3(1 - e) / (2 v_ref)`.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] when stiffness or reference
    /// speed are not positive, or when restitution is outside `[0, 1]`.
    pub fn from_restitution(
        stiffness_n_m_3_2: f64,
        restitution: f64,
        reference_impact_speed_m_s: f64,
    ) -> Result<Self, ContactError> {
        let stiffness_n_m_3_2 = require_positive("stiffness_n_m_3_2", stiffness_n_m_3_2)?;
        let restitution = require_finite("restitution", restitution)?;
        if !(0.0..=1.0).contains(&restitution) {
            return Err(ContactError::InvalidParameter {
                field: "restitution",
                reason: "must be in [0, 1]",
            });
        }
        let reference_impact_speed_m_s =
            require_positive("reference_impact_speed_m_s", reference_impact_speed_m_s)?;
        Ok(Self {
            stiffness_n_m_3_2,
            damping_factor_s_m: 1.5 * (1.0 - restitution) / reference_impact_speed_m_s,
        })
    }

    /// Creates a Hunt-Crossley law from an explicit damping factor.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] when stiffness is not
    /// positive or damping factor is negative/non-finite.
    pub fn new(stiffness_n_m_3_2: f64, damping_factor_s_m: f64) -> Result<Self, ContactError> {
        Ok(Self {
            stiffness_n_m_3_2: require_positive("stiffness_n_m_3_2", stiffness_n_m_3_2)?,
            damping_factor_s_m: require_non_negative("damping_factor_s_m", damping_factor_s_m)?,
        })
    }

    /// Evaluates the normal response for the supplied kinematics.
    #[must_use]
    pub fn evaluate(self, kin: ContactKinematics) -> NormalResponse {
        let x = kin.penetration_m();
        if x <= 0.0 {
            return no_normal_response();
        }
        let x_3_2 = x * x.sqrt();
        let penetration_rate_m_s = kin.penetration_rate_m_s();
        let elastic_force_n = self.stiffness_n_m_3_2 * x_3_2;
        let multiplier = (1.0 + self.damping_factor_s_m * penetration_rate_m_s).max(0.0);
        let normal_force_n = elastic_force_n * multiplier;
        let damping_power_w =
            (normal_force_n - elastic_force_n).max(0.0) * penetration_rate_m_s.max(0.0);
        NormalResponse {
            normal_force_n,
            elastic_energy_j: 0.4 * self.stiffness_n_m_3_2 * x_3_2 * x,
            damping_power_w,
        }
    }

    /// Returns the restitution-derived damping factor in seconds per meter.
    #[must_use]
    pub const fn damping_factor_s_m(self) -> f64 {
        self.damping_factor_s_m
    }
}

/// Supported normal-contact force laws.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum NormalLaw {
    /// Linear spring-damper normal contact.
    KelvinVoigt(KelvinVoigtNormal),
    /// Undamped Hertzian normal contact.
    Hertz(HertzNormal),
    /// Hunt-Crossley nonlinear normal contact.
    HuntCrossley(HuntCrossleyNormal),
}

impl NormalLaw {
    /// Evaluates the selected normal law.
    #[must_use]
    pub fn evaluate(self, kin: ContactKinematics) -> NormalResponse {
        match self {
            Self::KelvinVoigt(model) => model.evaluate(kin),
            Self::Hertz(model) => model.evaluate(kin),
            Self::HuntCrossley(model) => model.evaluate(kin),
        }
    }
}

/// Tanh-regularized Coulomb friction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RegularizedCoulombFriction {
    coefficient: f64,
    regularization_speed_m_s: f64,
}

impl RegularizedCoulombFriction {
    /// Creates a friction model with kinetic coefficient and smoothing speed.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] if the coefficient is
    /// negative/non-finite or if the regularization speed is not positive.
    pub fn new(coefficient: f64, regularization_speed_m_s: f64) -> Result<Self, ContactError> {
        Ok(Self {
            coefficient: require_non_negative("coefficient", coefficient)?,
            regularization_speed_m_s: require_positive(
                "regularization_speed_m_s",
                regularization_speed_m_s,
            )?,
        })
    }

    /// Returns the vector friction force opposing tangential motion.
    #[must_use]
    pub fn force_n(self, kin: ContactKinematics, normal_force_n: f64) -> ContactVector3 {
        if normal_force_n <= 0.0 || self.coefficient == 0.0 {
            return [0.0, 0.0, 0.0];
        }
        let speed = kin.tangential_speed_m_s();
        if speed <= 0.0 {
            return [0.0, 0.0, 0.0];
        }
        let magnitude_n =
            self.coefficient * normal_force_n * (speed / self.regularization_speed_m_s).tanh();
        scale(kin.tangential_velocity_m_s, -magnitude_n / speed)
    }

    /// Returns kinetic coefficient.
    #[must_use]
    pub const fn coefficient(self) -> f64 {
        self.coefficient
    }
}

/// Contact evaluation output for one point/sphere against one half-space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactForce {
    /// Whether the normal law observed penetration.
    pub in_contact: bool,
    /// Non-negative normal force in newtons.
    pub normal_force_n: f64,
    /// Tangential friction force in newtons.
    pub tangential_force_n: ContactVector3,
    /// Unit normal direction of the contact plane.
    pub normal_direction: ContactVector3,
    /// Elastic energy stored in the normal law.
    pub elastic_energy_j: f64,
    /// Instantaneous damping power in watts.
    pub damping_power_w: f64,
}

impl ContactForce {
    /// Returns the total force vector in newtons.
    #[must_use]
    pub fn total_force_n(self) -> ContactVector3 {
        add(
            scale(self.normal_direction, self.normal_force_n),
            self.tangential_force_n,
        )
    }
}

/// Pairing of a local contact geometry and force laws.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactPair {
    geometry: ContactGeometry,
    normal_law: NormalLaw,
    friction: RegularizedCoulombFriction,
}

impl ContactPair {
    /// Creates a contact pair.
    #[must_use]
    pub const fn new(
        geometry: ContactGeometry,
        normal_law: NormalLaw,
        friction: RegularizedCoulombFriction,
    ) -> Self {
        Self {
            geometry,
            normal_law,
            friction,
        }
    }

    /// Evaluates this contact pair against a half-space.
    ///
    /// # Errors
    ///
    /// Returns an error when kinematics construction fails, for example from a
    /// non-finite position or velocity component.
    pub fn evaluate_half_space(
        self,
        half_space: HalfSpace,
        contact_center_position_m: ContactVector3,
        contact_center_velocity_m_s: ContactVector3,
    ) -> Result<ContactForce, ContactError> {
        let kin = half_space_kinematics(
            half_space,
            self.geometry,
            contact_center_position_m,
            contact_center_velocity_m_s,
        )?;
        let normal = self.normal_law.evaluate(kin);
        Ok(ContactForce {
            in_contact: normal.normal_force_n > 0.0,
            normal_force_n: normal.normal_force_n,
            tangential_force_n: self.friction.force_n(kin, normal.normal_force_n),
            normal_direction: half_space.normal(),
            elastic_energy_j: normal.elastic_energy_j,
            damping_power_w: normal.damping_power_w,
        })
    }
}

/// Explicit penalty-contact sub-step stability configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactStabilityConfig {
    stiffness_n_m: f64,
    effective_mass_kg: f64,
    major_step_s: f64,
    substeps: u32,
    safety_fraction: f64,
}

impl ContactStabilityConfig {
    /// Creates a stability-check configuration with the default safety factor.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] for non-positive stiffness,
    /// effective mass, step size, or sub-step count.
    pub fn new(
        stiffness_n_m: f64,
        effective_mass_kg: f64,
        major_step_s: f64,
        substeps: u32,
    ) -> Result<Self, ContactError> {
        Self::with_safety_fraction(
            stiffness_n_m,
            effective_mass_kg,
            major_step_s,
            substeps,
            DEFAULT_STABILITY_SAFETY_FRACTION,
        )
    }

    /// Creates a stability-check configuration with an explicit safety factor.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] for invalid values.
    pub fn with_safety_fraction(
        stiffness_n_m: f64,
        effective_mass_kg: f64,
        major_step_s: f64,
        substeps: u32,
        safety_fraction: f64,
    ) -> Result<Self, ContactError> {
        if substeps == 0 {
            return Err(ContactError::InvalidParameter {
                field: "substeps",
                reason: "must be positive",
            });
        }
        Ok(Self {
            stiffness_n_m: require_positive("stiffness_n_m", stiffness_n_m)?,
            effective_mass_kg: require_positive("effective_mass_kg", effective_mass_kg)?,
            major_step_s: require_positive("major_step_s", major_step_s)?,
            substeps,
            safety_fraction: require_positive("safety_fraction", safety_fraction)?,
        })
    }

    /// Returns the natural frequency `sqrt(k / m)`.
    #[must_use]
    pub fn natural_frequency_rad_s(self) -> f64 {
        (self.stiffness_n_m / self.effective_mass_kg).sqrt()
    }

    /// Returns the explicit contact sub-step size.
    #[must_use]
    pub fn substep_s(self) -> f64 {
        self.major_step_s / f64::from(self.substeps)
    }

    /// Returns the maximum permitted sub-step.
    #[must_use]
    pub fn max_substep_s(self) -> f64 {
        self.safety_fraction / self.natural_frequency_rad_s()
    }

    /// Fails closed if the requested sub-step violates the contact bound.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::StabilityBoundViolation`] when
    /// `major_step_s / substeps > safety_fraction / sqrt(k / m)`. The default
    /// safety fraction is `0.1 * 2*pi`, matching the parity-program WP-14.1
    /// fixed-step contact margin.
    pub fn validate(self) -> Result<(), ContactError> {
        let natural_frequency_rad_s = self.natural_frequency_rad_s();
        let substep_s = self.substep_s();
        let max_substep_s = self.max_substep_s();
        if substep_s <= max_substep_s {
            Ok(())
        } else {
            Err(ContactError::StabilityBoundViolation {
                natural_frequency_rad_s,
                substep_s,
                max_substep_s,
            })
        }
    }
}

/// Contact energy balance for one step or run segment.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactEnergyAudit {
    /// Mechanical energy before the segment.
    pub initial_mechanical_energy_j: f64,
    /// Mechanical energy after the segment.
    pub final_mechanical_energy_j: f64,
    /// External work applied during the segment.
    pub external_work_j: f64,
    /// Non-negative energy dissipated by contact damping/friction.
    pub dissipated_energy_j: f64,
}

impl ContactEnergyAudit {
    /// Creates an energy-audit record.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] if an energy component is
    /// not finite or if dissipated energy is negative.
    pub fn new(
        initial_mechanical_energy_j: f64,
        final_mechanical_energy_j: f64,
        external_work_j: f64,
        dissipated_energy_j: f64,
    ) -> Result<Self, ContactError> {
        Ok(Self {
            initial_mechanical_energy_j: require_finite(
                "initial_mechanical_energy_j",
                initial_mechanical_energy_j,
            )?,
            final_mechanical_energy_j: require_finite(
                "final_mechanical_energy_j",
                final_mechanical_energy_j,
            )?,
            external_work_j: require_finite("external_work_j", external_work_j)?,
            dissipated_energy_j: require_non_negative("dissipated_energy_j", dissipated_energy_j)?,
        })
    }

    /// Returns the signed closure error:
    /// `final + dissipated - initial - external_work`.
    #[must_use]
    pub fn closure_error_j(self) -> f64 {
        ((self.final_mechanical_energy_j + self.dissipated_energy_j)
            - self.initial_mechanical_energy_j)
            - self.external_work_j
    }

    /// Returns absolute closure error normalized by the largest energy scale.
    #[must_use]
    pub fn relative_closure_error(self) -> f64 {
        let scale_j = self
            .initial_mechanical_energy_j
            .abs()
            .max(self.final_mechanical_energy_j.abs())
            .max(self.external_work_j.abs())
            .max(self.dissipated_energy_j.abs())
            .max(1.0);
        self.closure_error_j().abs() / scale_j
    }

    /// Returns `true` if the absolute and relative tolerances are both met.
    #[must_use]
    pub fn closes_within(self, absolute_tol_j: f64, relative_tol: f64) -> bool {
        self.closure_error_j().abs() <= absolute_tol_j
            || self.relative_closure_error() <= relative_tol
    }
}

fn no_normal_response() -> NormalResponse {
    NormalResponse {
        normal_force_n: 0.0,
        elastic_energy_j: 0.0,
        damping_power_w: 0.0,
    }
}

fn finite_vector(
    field: &'static str,
    vector: ContactVector3,
) -> Result<ContactVector3, ContactError> {
    if vector.iter().all(|component| component.is_finite()) {
        Ok(vector)
    } else {
        Err(ContactError::InvalidVector {
            field,
            reason: "all components must be finite",
        })
    }
}

fn unit_vector(
    field: &'static str,
    vector: ContactVector3,
) -> Result<ContactVector3, ContactError> {
    let vector = finite_vector(field, vector)?;
    let magnitude = norm(vector);
    if magnitude > 0.0 {
        Ok(scale(vector, 1.0 / magnitude))
    } else {
        Err(ContactError::InvalidVector {
            field,
            reason: "must be non-zero",
        })
    }
}

fn dot(a: ContactVector3, b: ContactVector3) -> f64 {
    (a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2])
}

fn norm(a: ContactVector3) -> f64 {
    dot(a, a).sqrt()
}

fn scale(a: ContactVector3, scalar: f64) -> ContactVector3 {
    [a[0] * scalar, a[1] * scalar, a[2] * scalar]
}

fn add(a: ContactVector3, b: ContactVector3) -> ContactVector3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub(a: ContactVector3, b: ContactVector3) -> ContactVector3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

#[cfg(test)]
mod tests {
    use approx::{assert_abs_diff_eq, assert_relative_eq};

    use super::*;

    #[test]
    fn half_space_sphere_kinematics_subtracts_radius_and_velocity_normal() {
        let geometry = ContactGeometry::sphere(0.25).unwrap();
        let kin = half_space_kinematics(
            HalfSpace::ground_z0(),
            geometry,
            [1.0, 2.0, 0.20],
            [3.0, 4.0, -5.0],
        )
        .unwrap();
        assert_abs_diff_eq!(kin.gap_m, -0.05, epsilon = 1.0e-15);
        assert_abs_diff_eq!(kin.normal_velocity_m_s, -5.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(kin.tangential_velocity_m_s[0], 3.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(kin.tangential_velocity_m_s[1], 4.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(kin.tangential_velocity_m_s[2], 0.0, epsilon = 1.0e-15);
    }

    #[test]
    fn kelvin_voigt_static_penetration_balances_weight() {
        let mass_kg = 12.0;
        let gravity_m_s2 = 9.80665;
        let stiffness_n_m = 48_000.0;
        let penetration_m = mass_kg * gravity_m_s2 / stiffness_n_m;
        let model = KelvinVoigtNormal::new(stiffness_n_m, 0.0).unwrap();
        let kin = ContactKinematics::new(-penetration_m, 0.0, [0.0, 0.0, 0.0]).unwrap();
        let response = model.evaluate(kin);
        assert_relative_eq!(
            response.normal_force_n,
            mass_kg * gravity_m_s2,
            epsilon = 1.0e-12,
            max_relative = 1.0e-12
        );
    }

    #[test]
    fn hertz_elastic_energy_matches_integral() {
        let model = HertzNormal::new(5_000.0).unwrap();
        let kin = ContactKinematics::new(-0.04, 0.0, [0.0, 0.0, 0.0]).unwrap();
        let response = model.evaluate(kin);
        let expected_force = 5_000.0 * 0.04_f64.powf(1.5);
        let expected_energy = 0.4 * expected_force * 0.04;
        assert_relative_eq!(
            response.normal_force_n,
            expected_force,
            max_relative = 1.0e-14
        );
        assert_relative_eq!(
            response.elastic_energy_j,
            expected_energy,
            max_relative = 1.0e-14
        );
    }

    #[test]
    fn hunt_crossley_restitution_sets_damping_factor() {
        let model = HuntCrossleyNormal::from_restitution(10_000.0, 0.8, 2.0).unwrap();
        assert_abs_diff_eq!(model.damping_factor_s_m(), 0.15, epsilon = 1.0e-15);
        let kin = ContactKinematics::new(-0.01, -2.0, [0.0, 0.0, 0.0]).unwrap();
        let response = model.evaluate(kin);
        let elastic_force = 10_000.0 * 0.01_f64.powf(1.5);
        assert_relative_eq!(
            response.normal_force_n,
            elastic_force * 1.3,
            max_relative = 1.0e-14
        );
    }

    #[test]
    fn regularized_coulomb_sliding_block_decelerates_at_mu_g() {
        let mass_kg = 2.0;
        let gravity_m_s2 = 9.80665;
        let mu = 0.37;
        let friction = RegularizedCoulombFriction::new(mu, 1.0e-9).unwrap();
        let kin = ContactKinematics::new(-0.01, 0.0, [10.0, 0.0, 0.0]).unwrap();
        let force = friction.force_n(kin, mass_kg * gravity_m_s2);
        assert_abs_diff_eq!(force[0] / mass_kg, -mu * gravity_m_s2, epsilon = 1.0e-12);
        assert_abs_diff_eq!(force[1], 0.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(force[2], 0.0, epsilon = 1.0e-15);
    }

    #[test]
    fn contact_pair_returns_total_force_in_locked_components() {
        let pair = ContactPair::new(
            ContactGeometry::Point,
            NormalLaw::KelvinVoigt(KelvinVoigtNormal::new(1_000.0, 0.0).unwrap()),
            RegularizedCoulombFriction::new(0.5, 1.0e-9).unwrap(),
        );
        let force = pair
            .evaluate_half_space(HalfSpace::ground_z0(), [0.0, 0.0, -0.01], [2.0, 0.0, 0.0])
            .unwrap();
        let total = force.total_force_n();
        assert_abs_diff_eq!(force.normal_force_n, 10.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(total[0], -5.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(total[1], 0.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(total[2], 10.0, epsilon = 1.0e-15);
    }

    #[test]
    fn stability_bound_fails_closed_for_too_large_substep() {
        let config = ContactStabilityConfig::new(10_000.0, 1.0, 0.01, 1).unwrap();
        let err = config.validate().unwrap_err();
        match err {
            ContactError::StabilityBoundViolation {
                natural_frequency_rad_s,
                substep_s,
                max_substep_s,
            } => {
                assert_abs_diff_eq!(natural_frequency_rad_s, 100.0, epsilon = 1.0e-15);
                assert_abs_diff_eq!(substep_s, 0.01, epsilon = 1.0e-15);
                assert_abs_diff_eq!(max_substep_s, 0.006_283_185_307_179_586, epsilon = 1.0e-15);
            }
            _ => unreachable!("wrong error"),
        }
    }

    #[test]
    fn stability_bound_accepts_substepped_contact() {
        let config = ContactStabilityConfig::new(10_000.0, 1.0, 0.01, 8).unwrap();
        assert!(config.validate().is_ok());
    }

    #[test]
    fn energy_audit_closes_with_dissipation() {
        let audit = ContactEnergyAudit::new(10.0, 8.5, 0.0, 1.5).unwrap();
        assert_abs_diff_eq!(audit.closure_error_j(), 0.0, epsilon = 1.0e-15);
        assert!(audit.closes_within(1.0e-12, 1.0e-12));
    }
}
