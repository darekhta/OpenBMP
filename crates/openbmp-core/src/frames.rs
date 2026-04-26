//! Coordinate frames and frame-tagged vectors.
//!
//! Frame tags are zero-sized types ([`Eci`], [`Ecef`], [`Ned`], [`Enu`],
//! [`Body`]) implementing the [`Frame`] trait. Vector-like types are
//! parameterised by frame: a [`Position3<Eci>`] is a different type
//! from a [`Position3<Ecef>`] and the compiler refuses to mix them.
//!
//! Conversions between frames are explicit through the
//! [`FrameTransform`] trait. The active [`FrameContext`] determines
//! which transforms exist; Phase-1 ships only the
//! [`FrameProfile::ToyFixedEarth`] profile, in which `ECI` and `ECEF`
//! coincide and local-frame transforms are identity for matching
//! frames.
//!
//! No type in this module accesses wall-clock time, system RNG, or any
//! external Earth-orientation data. Higher-fidelity frame profiles in
//! later phases plug in via the same trait surface.

use std::marker::PhantomData;
use std::ops::{Add, Mul, Neg, Sub};

use nalgebra::{UnitQuaternion, Vector3};

use crate::error::FrameError;

// ---------------------------------------------------------------------
// Frame trait + tag types
// ---------------------------------------------------------------------

/// Compile-time tag for a coordinate frame.
pub trait Frame: 'static + Copy + Send + Sync {
    /// Runtime identifier for this frame.
    const ID: FrameId;
}

/// Runtime identifier for a coordinate frame.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub enum FrameId {
    /// Earth-Centered Inertial.
    Eci,
    /// Earth-Centered, Earth-Fixed.
    Ecef,
    /// Local North-East-Down.
    Ned,
    /// Local East-North-Up.
    Enu,
    /// Vehicle body frame.
    Body,
}

impl FrameId {
    /// Returns the canonical short label.
    #[must_use]
    pub const fn as_label(self) -> &'static str {
        match self {
            Self::Eci => "ECI",
            Self::Ecef => "ECEF",
            Self::Ned => "NED",
            Self::Enu => "ENU",
            Self::Body => "Body",
        }
    }
}

/// Earth-Centered Inertial frame tag. OpenBMP's canonical propagation
/// frame.
#[derive(Copy, Clone, Debug, Default)]
pub struct Eci;
impl Frame for Eci {
    const ID: FrameId = FrameId::Eci;
}

/// Earth-Centered, Earth-Fixed frame tag. Rotates with Earth in
/// higher-fidelity profiles; identity to ECI in
/// [`FrameProfile::ToyFixedEarth`].
#[derive(Copy, Clone, Debug, Default)]
pub struct Ecef;
impl Frame for Ecef {
    const ID: FrameId = FrameId::Ecef;
}

/// Local North-East-Down navigation frame tag.
#[derive(Copy, Clone, Debug, Default)]
pub struct Ned;
impl Frame for Ned {
    const ID: FrameId = FrameId::Ned;
}

/// Local East-North-Up navigation frame tag.
#[derive(Copy, Clone, Debug, Default)]
pub struct Enu;
impl Frame for Enu {
    const ID: FrameId = FrameId::Enu;
}

/// Vehicle body frame tag.
#[derive(Copy, Clone, Debug, Default)]
pub struct Body;
impl Frame for Body {
    const ID: FrameId = FrameId::Body;
}

// ---------------------------------------------------------------------
// Position3 / Velocity3 / AngularVelocity3
// ---------------------------------------------------------------------

/// Position vector in frame `F`, stored as `Vector3<f64>` interpreted
/// as metres.
#[derive(Copy, Clone, Debug)]
pub struct Position3<F: Frame> {
    /// Underlying vector (metres).
    pub vector: Vector3<f64>,
    _phantom: PhantomData<F>,
}

impl<F: Frame> Position3<F> {
    /// Construct from XYZ components in metres.
    #[must_use]
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            vector: Vector3::new(x, y, z),
            _phantom: PhantomData,
        }
    }

    /// Construct from a raw vector.
    #[must_use]
    pub const fn from_vector(vector: Vector3<f64>) -> Self {
        Self {
            vector,
            _phantom: PhantomData,
        }
    }

    /// Origin (zero vector).
    #[must_use]
    pub fn origin() -> Self {
        Self::from_vector(Vector3::zeros())
    }

    /// Returns the Euclidean norm in metres.
    #[must_use]
    pub fn norm(self) -> f64 {
        self.vector.norm()
    }

    /// Returns `true` if every component is finite.
    #[must_use]
    pub fn is_finite(self) -> bool {
        self.vector.iter().all(|v| v.is_finite())
    }
}

impl<F: Frame> Default for Position3<F> {
    fn default() -> Self {
        Self::origin()
    }
}

impl<F: Frame> PartialEq for Position3<F> {
    fn eq(&self, other: &Self) -> bool {
        self.vector == other.vector
    }
}

impl<F: Frame> Sub for Position3<F> {
    type Output = Vector3<f64>;
    fn sub(self, rhs: Self) -> Vector3<f64> {
        self.vector - rhs.vector
    }
}

/// Velocity vector in frame `F`, stored as `Vector3<f64>` interpreted
/// as metres per second.
#[derive(Copy, Clone, Debug)]
pub struct Velocity3<F: Frame> {
    /// Underlying vector (m/s).
    pub vector: Vector3<f64>,
    _phantom: PhantomData<F>,
}

impl<F: Frame> Velocity3<F> {
    /// Construct from XYZ components in m/s.
    #[must_use]
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            vector: Vector3::new(x, y, z),
            _phantom: PhantomData,
        }
    }

    /// Construct from a raw vector.
    #[must_use]
    pub const fn from_vector(vector: Vector3<f64>) -> Self {
        Self {
            vector,
            _phantom: PhantomData,
        }
    }

    /// Zero velocity.
    #[must_use]
    pub fn zero() -> Self {
        Self::from_vector(Vector3::zeros())
    }

    /// Returns the Euclidean norm in m/s.
    #[must_use]
    pub fn norm(self) -> f64 {
        self.vector.norm()
    }

    /// Returns `true` if every component is finite.
    #[must_use]
    pub fn is_finite(self) -> bool {
        self.vector.iter().all(|v| v.is_finite())
    }
}

impl<F: Frame> Default for Velocity3<F> {
    fn default() -> Self {
        Self::zero()
    }
}

impl<F: Frame> PartialEq for Velocity3<F> {
    fn eq(&self, other: &Self) -> bool {
        self.vector == other.vector
    }
}

impl<F: Frame> Add for Velocity3<F> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::from_vector(self.vector + rhs.vector)
    }
}

impl<F: Frame> Sub for Velocity3<F> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::from_vector(self.vector - rhs.vector)
    }
}

impl<F: Frame> Neg for Velocity3<F> {
    type Output = Self;
    fn neg(self) -> Self {
        Self::from_vector(-self.vector)
    }
}

impl<F: Frame> Mul<f64> for Velocity3<F> {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self::from_vector(self.vector * rhs)
    }
}

/// Angular velocity in frame `F`, stored as `Vector3<f64>` interpreted
/// as radians per second.
#[derive(Copy, Clone, Debug)]
pub struct AngularVelocity3<F: Frame> {
    /// Underlying vector (rad/s).
    pub vector: Vector3<f64>,
    _phantom: PhantomData<F>,
}

impl<F: Frame> AngularVelocity3<F> {
    /// Construct from XYZ components in rad/s.
    #[must_use]
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            vector: Vector3::new(x, y, z),
            _phantom: PhantomData,
        }
    }

    /// Construct from a raw vector.
    #[must_use]
    pub const fn from_vector(vector: Vector3<f64>) -> Self {
        Self {
            vector,
            _phantom: PhantomData,
        }
    }

    /// Zero angular velocity.
    #[must_use]
    pub fn zero() -> Self {
        Self::from_vector(Vector3::zeros())
    }

    /// Returns the Euclidean norm in rad/s.
    #[must_use]
    pub fn norm(self) -> f64 {
        self.vector.norm()
    }

    /// Returns `true` if every component is finite.
    #[must_use]
    pub fn is_finite(self) -> bool {
        self.vector.iter().all(|v| v.is_finite())
    }
}

impl<F: Frame> Default for AngularVelocity3<F> {
    fn default() -> Self {
        Self::zero()
    }
}

impl<F: Frame> PartialEq for AngularVelocity3<F> {
    fn eq(&self, other: &Self) -> bool {
        self.vector == other.vector
    }
}

// ---------------------------------------------------------------------
// Quaternion<From, To>
// ---------------------------------------------------------------------

/// A unit quaternion encoding the passive rotation that takes a vector
/// expressed in `From`-frame components to its expression in `To`-frame
/// components.
#[derive(Copy, Clone, Debug)]
pub struct Quaternion<From: Frame, To: Frame> {
    /// Underlying nalgebra unit quaternion.
    pub q: UnitQuaternion<f64>,
    _phantom: PhantomData<(From, To)>,
}

impl<From: Frame, To: Frame> Quaternion<From, To> {
    /// Wrap an existing nalgebra unit quaternion.
    #[must_use]
    pub const fn from_unit_quaternion(q: UnitQuaternion<f64>) -> Self {
        Self {
            q,
            _phantom: PhantomData,
        }
    }

    /// Inverse rotation, returning a quaternion that takes vectors back
    /// from `To` to `From`.
    #[must_use]
    pub fn inverse(self) -> Quaternion<To, From> {
        Quaternion::from_unit_quaternion(self.q.inverse())
    }

    /// Apply this rotation to a [`Position3`].
    #[must_use]
    pub fn rotate_position(self, p: Position3<From>) -> Position3<To> {
        Position3::from_vector(self.q * p.vector)
    }

    /// Apply this rotation to a [`Velocity3`].
    #[must_use]
    pub fn rotate_velocity(self, v: Velocity3<From>) -> Velocity3<To> {
        Velocity3::from_vector(self.q * v.vector)
    }

    /// Apply this rotation to an [`AngularVelocity3`].
    #[must_use]
    pub fn rotate_angular_velocity(self, w: AngularVelocity3<From>) -> AngularVelocity3<To> {
        AngularVelocity3::from_vector(self.q * w.vector)
    }

    /// Returns `true` if the underlying quaternion is normalised within
    /// `tolerance` of unit magnitude.
    #[must_use]
    pub fn is_normalised(self, tolerance: f64) -> bool {
        (self.q.into_inner().norm() - 1.0).abs() <= tolerance
    }

    /// Returns the magnitude of the underlying quaternion (1.0 within
    /// rounding for a properly normalised value).
    #[must_use]
    pub fn magnitude(self) -> f64 {
        self.q.into_inner().norm()
    }

    /// Validate normalisation within tolerance.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::NotNormalised`] if the quaternion is not
    /// within tolerance of unit magnitude.
    pub fn require_normalised(self, tolerance: f64) -> Result<Self, FrameError> {
        let magnitude = self.magnitude();
        if (magnitude - 1.0).abs() > tolerance {
            return Err(FrameError::NotNormalised {
                magnitude,
                tolerance,
            });
        }
        Ok(self)
    }
}

impl<F: Frame> Quaternion<F, F> {
    /// Identity rotation (no-op transform within a single frame).
    #[must_use]
    pub fn identity() -> Self {
        Self::from_unit_quaternion(UnitQuaternion::identity())
    }
}

impl<A: Frame, B: Frame, C: Frame> Mul<Quaternion<B, C>> for Quaternion<A, B> {
    type Output = Quaternion<A, C>;

    /// Compose two rotations. `Q_AB * Q_BC` produces the rotation that
    /// takes a vector in `A` directly to its expression in `C`.
    fn mul(self, rhs: Quaternion<B, C>) -> Quaternion<A, C> {
        // v_C = q_BC * v_B = q_BC * (q_AB * v_A) = (q_BC * q_AB) * v_A.
        Quaternion::from_unit_quaternion(rhs.q * self.q)
    }
}

// ---------------------------------------------------------------------
// FrameContext / FrameProfile / FrameTransform
// ---------------------------------------------------------------------

/// Active frame profile.
///
/// The profile determines which [`FrameTransform`] implementations are
/// available and what they compute. Phase-1 ships only
/// [`FrameProfile::ToyFixedEarth`].
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash)]
pub enum FrameProfile {
    /// Toy: ECI and ECEF coincide; no Earth rotation; no leap seconds.
    /// Used for analytic-toy scenarios that need no Earth-fixed
    /// physics.
    #[default]
    ToyFixedEarth,
}

impl FrameProfile {
    /// Canonical profile name as it appears in scenario files.
    #[must_use]
    pub const fn as_label(self) -> &'static str {
        match self {
            Self::ToyFixedEarth => "toy-fixed-earth",
        }
    }
}

/// Snapshotted frame context for a scenario.
///
/// Created at scenario start and then immutable. Higher-fidelity
/// profiles will carry pinned Earth-orientation tables; the toy
/// profile carries only its discriminant.
#[derive(Clone, Debug, Default)]
pub struct FrameContext {
    profile: FrameProfile,
}

impl FrameContext {
    /// Construct a context for [`FrameProfile::ToyFixedEarth`].
    #[must_use]
    pub const fn toy_fixed_earth() -> Self {
        Self {
            profile: FrameProfile::ToyFixedEarth,
        }
    }

    /// The active profile.
    #[must_use]
    pub const fn profile(&self) -> FrameProfile {
        self.profile
    }
}

/// Conversion from frame `From` to frame `To`.
///
/// Implementations are provided by [`FrameContext`] for the frame
/// pairs available in the active [`FrameProfile`]. Frame pairs that
/// are not available return [`FrameError::TransformNotAvailable`] from
/// any methods that would attempt them.
pub trait FrameTransform<From: Frame, To: Frame> {
    /// Convert a position vector from `From` to `To`.
    fn transform_position(&self, p: Position3<From>) -> Position3<To>;
    /// Convert a velocity vector from `From` to `To` at the given
    /// position. Position is needed for non-inertial frame transforms
    /// (Coriolis, transport rate); the toy profile ignores it.
    fn transform_velocity(&self, v: Velocity3<From>, p: Position3<From>) -> Velocity3<To>;
}

// Identity transform for any same-frame pair, in any profile.
impl<F: Frame> FrameTransform<F, F> for FrameContext {
    fn transform_position(&self, p: Position3<F>) -> Position3<F> {
        p
    }
    fn transform_velocity(&self, v: Velocity3<F>, _p: Position3<F>) -> Velocity3<F> {
        v
    }
}

// Toy-fixed-earth: ECI ↔ ECEF identity (Earth not rotating).
impl FrameTransform<Eci, Ecef> for FrameContext {
    fn transform_position(&self, p: Position3<Eci>) -> Position3<Ecef> {
        Position3::from_vector(p.vector)
    }
    fn transform_velocity(&self, v: Velocity3<Eci>, _p: Position3<Eci>) -> Velocity3<Ecef> {
        Velocity3::from_vector(v.vector)
    }
}

impl FrameTransform<Ecef, Eci> for FrameContext {
    fn transform_position(&self, p: Position3<Ecef>) -> Position3<Eci> {
        Position3::from_vector(p.vector)
    }
    fn transform_velocity(&self, v: Velocity3<Ecef>, _p: Position3<Ecef>) -> Velocity3<Eci> {
        Velocity3::from_vector(v.vector)
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use proptest::prelude::*;

    #[test]
    fn frame_id_labels() {
        assert_eq!(FrameId::Eci.as_label(), "ECI");
        assert_eq!(FrameId::Ecef.as_label(), "ECEF");
        assert_eq!(FrameId::Ned.as_label(), "NED");
        assert_eq!(FrameId::Enu.as_label(), "ENU");
        assert_eq!(FrameId::Body.as_label(), "Body");
    }

    #[test]
    fn frame_trait_constants() {
        assert_eq!(<Eci as Frame>::ID, FrameId::Eci);
        assert_eq!(<Ecef as Frame>::ID, FrameId::Ecef);
        assert_eq!(<Body as Frame>::ID, FrameId::Body);
    }

    #[test]
    fn position_origin_is_zero() {
        let p: Position3<Eci> = Position3::origin();
        assert_abs_diff_eq!(p.norm(), 0.0);
    }

    #[test]
    fn position_norm() {
        let p: Position3<Eci> = Position3::new(3.0, 4.0, 0.0);
        assert_abs_diff_eq!(p.norm(), 5.0);
    }

    #[test]
    fn position_finite_classifier() {
        let p: Position3<Eci> = Position3::new(1.0, 2.0, 3.0);
        assert!(p.is_finite());
        let bad: Position3<Eci> = Position3::new(f64::NAN, 0.0, 0.0);
        assert!(!bad.is_finite());
    }

    #[test]
    fn velocity_addition() {
        let a: Velocity3<Eci> = Velocity3::new(1.0, 2.0, 3.0);
        let b: Velocity3<Eci> = Velocity3::new(4.0, 5.0, 6.0);
        let sum = a + b;
        assert_abs_diff_eq!(sum.vector.x, 5.0);
        assert_abs_diff_eq!(sum.vector.y, 7.0);
        assert_abs_diff_eq!(sum.vector.z, 9.0);
    }

    #[test]
    fn velocity_scalar_multiply() {
        let v: Velocity3<Eci> = Velocity3::new(1.0, 2.0, 3.0);
        let scaled = v * 2.5;
        assert_abs_diff_eq!(scaled.vector.x, 2.5);
        assert_abs_diff_eq!(scaled.vector.y, 5.0);
        assert_abs_diff_eq!(scaled.vector.z, 7.5);
    }

    #[test]
    fn quaternion_identity_is_no_op() {
        let q: Quaternion<Eci, Eci> = Quaternion::identity();
        let p: Position3<Eci> = Position3::new(1.0, 2.0, 3.0);
        let r = q.rotate_position(p);
        assert_abs_diff_eq!(r.vector.x, 1.0);
        assert_abs_diff_eq!(r.vector.y, 2.0);
        assert_abs_diff_eq!(r.vector.z, 3.0);
    }

    #[test]
    fn quaternion_identity_is_normalised() {
        let q: Quaternion<Eci, Eci> = Quaternion::identity();
        assert!(q.is_normalised(1.0e-12));
        assert_abs_diff_eq!(q.magnitude(), 1.0, epsilon = 1.0e-12);
    }

    #[test]
    fn quaternion_inverse_round_trip() {
        let axis = nalgebra::Unit::new_normalize(nalgebra::Vector3::new(0.0, 0.0, 1.0));
        let qz = UnitQuaternion::from_axis_angle(&axis, std::f64::consts::FRAC_PI_2);
        let q: Quaternion<Eci, Body> = Quaternion::from_unit_quaternion(qz);
        let p: Position3<Eci> = Position3::new(1.0, 0.0, 0.0);
        let p_body = q.rotate_position(p);
        let q_inv: Quaternion<Body, Eci> = q.inverse();
        let p_back = q_inv.rotate_position(p_body);
        assert_abs_diff_eq!(p_back.vector.x, 1.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(p_back.vector.y, 0.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(p_back.vector.z, 0.0, epsilon = 1.0e-12);
    }

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
        let p_ecef: Position3<Ecef> = ctx.transform_position(p_eci);
        assert_abs_diff_eq!(p_ecef.vector.x, 7000.0);
        assert_abs_diff_eq!(p_ecef.vector.y, 0.0);
        assert_abs_diff_eq!(p_ecef.vector.z, 0.0);
    }

    #[test]
    fn frame_context_round_trip_eci_ecef_eci() {
        let ctx = FrameContext::toy_fixed_earth();
        let p_in: Position3<Eci> = Position3::new(1.0, 2.0, 3.0);
        let p_ecef: Position3<Ecef> = ctx.transform_position(p_in);
        let p_back: Position3<Eci> = ctx.transform_position(p_ecef);
        assert_abs_diff_eq!(p_in.vector.x, p_back.vector.x);
        assert_abs_diff_eq!(p_in.vector.y, p_back.vector.y);
        assert_abs_diff_eq!(p_in.vector.z, p_back.vector.z);
    }

    proptest! {
        #[test]
        fn property_position_finite_iff_components_finite(
            x in -1e9_f64..1e9,
            y in -1e9_f64..1e9,
            z in -1e9_f64..1e9,
        ) {
            let p: Position3<Eci> = Position3::new(x, y, z);
            prop_assert!(p.is_finite());
        }

        #[test]
        fn property_velocity_neg_round_trip(
            x in -1e6_f64..1e6,
            y in -1e6_f64..1e6,
            z in -1e6_f64..1e6,
        ) {
            let v: Velocity3<Eci> = Velocity3::new(x, y, z);
            let back = -(-v);
            prop_assert!((back.vector.x - x).abs() < 1.0e-12);
            prop_assert!((back.vector.y - y).abs() < 1.0e-12);
            prop_assert!((back.vector.z - z).abs() < 1.0e-12);
        }

        #[test]
        fn property_quaternion_inverse_round_trip(
            ax in -1.0_f64..1.0,
            ay in -1.0_f64..1.0,
            az in -1.0_f64..1.0,
            angle in -3.0_f64..3.0,
        ) {
            // Skip degenerate axes.
            let mag = (ax * ax + ay * ay + az * az).sqrt();
            prop_assume!(mag > 1.0e-3);
            let axis = nalgebra::Unit::new_normalize(nalgebra::Vector3::new(ax, ay, az));
            let q_inner = UnitQuaternion::from_axis_angle(&axis, angle);
            let q: Quaternion<Eci, Body> = Quaternion::from_unit_quaternion(q_inner);
            let p: Position3<Eci> = Position3::new(1.0, 2.0, 3.0);
            let rotated = q.rotate_position(p);
            let back = q.inverse().rotate_position(rotated);
            prop_assert!((back.vector.x - 1.0).abs() < 1.0e-9);
            prop_assert!((back.vector.y - 2.0).abs() < 1.0e-9);
            prop_assert!((back.vector.z - 3.0).abs() < 1.0e-9);
        }
    }
}
