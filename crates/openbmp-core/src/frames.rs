//! Coordinate frames and frame-tagged vectors.
//!
//! Frame tags are zero-sized types ([`Eci`], [`Ecef`], [`Ned`], [`Enu`],
//! [`Body`]) implementing the [`Frame`] trait. Vector-like types are
//! parameterised by frame: a [`Position3<Eci>`] is a different type
//! from a [`Position3<Ecef>`] and the compiler refuses to mix them.
//!
//! Conversions between frames are explicit through time-aware
//! [`FrameTransform`] implementations or the profile-specific helper
//! methods on [`FrameContext`]. The active [`FrameContext`] determines
//! which transforms exist; [`FrameProfile::ToyFixedEarth`] keeps `ECI`
//! and `ECEF` coincident, while
//! [`FrameProfile::Wgs84UniformRotation`] rotates `ECEF` relative to
//! `ECI` as a pure function of [`SimTime`].
//!
//! No type in this module accesses wall-clock time, system RNG, or any
//! external Earth-orientation data. Higher-fidelity frame profiles in
//! later phases plug in via the same trait surface.
//!
//! ```compile_fail
//! use openbmp_core::{Ecef, Eci, Position3};
//!
//! let inertial: Position3<Eci> = Position3::new(1.0, 0.0, 0.0);
//! let fixed: Position3<Ecef> = Position3::new(1.0, 0.0, 0.0);
//! let _ = inertial - fixed; // frame mismatch: does not compile
//! ```

use std::marker::PhantomData;
use std::ops::{Add, Mul, Neg, Sub};

use nalgebra::{Quaternion as NalgebraQuaternion, UnitQuaternion, Vector3};

use crate::error::FrameError;
use crate::time::{Duration, SimTime};

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
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash)]
pub struct Eci;
impl Frame for Eci {
    const ID: FrameId = FrameId::Eci;
}

/// Earth-Centered, Earth-Fixed frame tag. Rotates with Earth in
/// higher-fidelity profiles; identity to ECI in
/// [`FrameProfile::ToyFixedEarth`].
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash)]
pub struct Ecef;
impl Frame for Ecef {
    const ID: FrameId = FrameId::Ecef;
}

/// Local North-East-Down navigation frame tag.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash)]
pub struct Ned;
impl Frame for Ned {
    const ID: FrameId = FrameId::Ned;
}

/// Local East-North-Up navigation frame tag.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash)]
pub struct Enu;
impl Frame for Enu {
    const ID: FrameId = FrameId::Enu;
}

/// Vehicle body frame tag.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash)]
pub struct Body;
impl Frame for Body {
    const ID: FrameId = FrameId::Body;
}

// ---------------------------------------------------------------------
// Position3 / Displacement3 / Velocity3 / Acceleration3 / AngularVelocity3
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

    /// Validate that every component is finite.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::NotFinite`] if any component is `NaN` or
    /// infinite.
    pub fn require_finite(self) -> Result<Self, FrameError> {
        if self.is_finite() {
            Ok(self)
        } else {
            Err(FrameError::NotFinite { frame: F::ID })
        }
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

impl<F: Frame> Add<Displacement3<F>> for Position3<F> {
    type Output = Self;
    fn add(self, rhs: Displacement3<F>) -> Self {
        Self::from_vector(self.vector + rhs.vector)
    }
}

impl<F: Frame> Sub<Displacement3<F>> for Position3<F> {
    type Output = Self;
    fn sub(self, rhs: Displacement3<F>) -> Self {
        Self::from_vector(self.vector - rhs.vector)
    }
}

impl<F: Frame> Sub for Position3<F> {
    type Output = Displacement3<F>;
    fn sub(self, rhs: Self) -> Displacement3<F> {
        Displacement3::from_vector(self.vector - rhs.vector)
    }
}

/// Displacement vector in frame `F`, stored as `Vector3<f64>`
/// interpreted as metres.
#[derive(Copy, Clone, Debug)]
pub struct Displacement3<F: Frame> {
    /// Underlying vector (metres).
    pub vector: Vector3<f64>,
    _phantom: PhantomData<F>,
}

impl<F: Frame> Displacement3<F> {
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

    /// Zero displacement.
    #[must_use]
    pub fn zero() -> Self {
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

    /// Validate that every component is finite.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::NotFinite`] if any component is `NaN` or
    /// infinite.
    pub fn require_finite(self) -> Result<Self, FrameError> {
        if self.is_finite() {
            Ok(self)
        } else {
            Err(FrameError::NotFinite { frame: F::ID })
        }
    }
}

impl<F: Frame> Default for Displacement3<F> {
    fn default() -> Self {
        Self::zero()
    }
}

impl<F: Frame> PartialEq for Displacement3<F> {
    fn eq(&self, other: &Self) -> bool {
        self.vector == other.vector
    }
}

impl<F: Frame> Add for Displacement3<F> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::from_vector(self.vector + rhs.vector)
    }
}

impl<F: Frame> Sub for Displacement3<F> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::from_vector(self.vector - rhs.vector)
    }
}

impl<F: Frame> Neg for Displacement3<F> {
    type Output = Self;
    fn neg(self) -> Self {
        Self::from_vector(-self.vector)
    }
}

impl<F: Frame> Mul<f64> for Displacement3<F> {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self::from_vector(self.vector * rhs)
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

    /// Validate that every component is finite.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::NotFinite`] if any component is `NaN` or
    /// infinite.
    pub fn require_finite(self) -> Result<Self, FrameError> {
        if self.is_finite() {
            Ok(self)
        } else {
            Err(FrameError::NotFinite { frame: F::ID })
        }
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

impl<F: Frame> Add<VelocityDelta3<F>> for Velocity3<F> {
    type Output = Self;
    fn add(self, rhs: VelocityDelta3<F>) -> Self {
        Self::from_vector(self.vector + rhs.vector)
    }
}

impl<F: Frame> Sub<VelocityDelta3<F>> for Velocity3<F> {
    type Output = Self;
    fn sub(self, rhs: VelocityDelta3<F>) -> Self {
        Self::from_vector(self.vector - rhs.vector)
    }
}

impl<F: Frame> Sub for Velocity3<F> {
    type Output = VelocityDelta3<F>;
    fn sub(self, rhs: Self) -> VelocityDelta3<F> {
        VelocityDelta3::from_vector(self.vector - rhs.vector)
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

impl<F: Frame> Mul<Duration> for Velocity3<F> {
    type Output = Displacement3<F>;
    fn mul(self, rhs: Duration) -> Displacement3<F> {
        Displacement3::from_vector(self.vector * rhs.as_seconds())
    }
}

impl<F: Frame> Mul<Velocity3<F>> for Duration {
    type Output = Displacement3<F>;
    fn mul(self, rhs: Velocity3<F>) -> Displacement3<F> {
        rhs * self
    }
}

/// Acceleration vector in frame `F`, stored as `Vector3<f64>`
/// interpreted as metres per second squared.
#[derive(Copy, Clone, Debug)]
pub struct Acceleration3<F: Frame> {
    /// Underlying vector (m/s^2).
    pub vector: Vector3<f64>,
    _phantom: PhantomData<F>,
}

impl<F: Frame> Acceleration3<F> {
    /// Construct from XYZ components in m/s^2.
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

    /// Zero acceleration.
    #[must_use]
    pub fn zero() -> Self {
        Self::from_vector(Vector3::zeros())
    }

    /// Returns the Euclidean norm in m/s^2.
    #[must_use]
    pub fn norm(self) -> f64 {
        self.vector.norm()
    }

    /// Returns `true` if every component is finite.
    #[must_use]
    pub fn is_finite(self) -> bool {
        self.vector.iter().all(|v| v.is_finite())
    }

    /// Validate that every component is finite.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::NotFinite`] if any component is `NaN` or
    /// infinite.
    pub fn require_finite(self) -> Result<Self, FrameError> {
        if self.is_finite() {
            Ok(self)
        } else {
            Err(FrameError::NotFinite { frame: F::ID })
        }
    }
}

impl<F: Frame> Default for Acceleration3<F> {
    fn default() -> Self {
        Self::zero()
    }
}

impl<F: Frame> PartialEq for Acceleration3<F> {
    fn eq(&self, other: &Self) -> bool {
        self.vector == other.vector
    }
}

impl<F: Frame> Add for Acceleration3<F> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::from_vector(self.vector + rhs.vector)
    }
}

impl<F: Frame> Sub for Acceleration3<F> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::from_vector(self.vector - rhs.vector)
    }
}

impl<F: Frame> Neg for Acceleration3<F> {
    type Output = Self;
    fn neg(self) -> Self {
        Self::from_vector(-self.vector)
    }
}

impl<F: Frame> Mul<f64> for Acceleration3<F> {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self::from_vector(self.vector * rhs)
    }
}

impl<F: Frame> Mul<Duration> for Acceleration3<F> {
    type Output = VelocityDelta3<F>;
    fn mul(self, rhs: Duration) -> VelocityDelta3<F> {
        VelocityDelta3::from_vector(self.vector * rhs.as_seconds())
    }
}

impl<F: Frame> Mul<Acceleration3<F>> for Duration {
    type Output = VelocityDelta3<F>;
    fn mul(self, rhs: Acceleration3<F>) -> VelocityDelta3<F> {
        rhs * self
    }
}

/// Velocity increment produced by integrating acceleration over time,
/// stored as `Vector3<f64>` interpreted as metres per second.
///
/// This type exists to keep `Velocity3 + (Acceleration3 * Duration)`
/// typed without reusing a raw vector.
#[derive(Copy, Clone, Debug)]
pub struct VelocityDelta3<F: Frame> {
    /// Underlying vector (m/s).
    pub vector: Vector3<f64>,
    _phantom: PhantomData<F>,
}

impl<F: Frame> VelocityDelta3<F> {
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

    /// Zero velocity increment.
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

    /// Validate that every component is finite.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::NotFinite`] if any component is `NaN` or
    /// infinite.
    pub fn require_finite(self) -> Result<Self, FrameError> {
        if self.is_finite() {
            Ok(self)
        } else {
            Err(FrameError::NotFinite { frame: F::ID })
        }
    }
}

impl<F: Frame> Default for VelocityDelta3<F> {
    fn default() -> Self {
        Self::zero()
    }
}

impl<F: Frame> PartialEq for VelocityDelta3<F> {
    fn eq(&self, other: &Self) -> bool {
        self.vector == other.vector
    }
}

impl<F: Frame> Add for VelocityDelta3<F> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::from_vector(self.vector + rhs.vector)
    }
}

impl<F: Frame> Sub for VelocityDelta3<F> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::from_vector(self.vector - rhs.vector)
    }
}

impl<F: Frame> Neg for VelocityDelta3<F> {
    type Output = Self;
    fn neg(self) -> Self {
        Self::from_vector(-self.vector)
    }
}

impl<F: Frame> Mul<f64> for VelocityDelta3<F> {
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

    /// Validate that every component is finite.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::NotFinite`] if any component is `NaN` or
    /// infinite.
    pub fn require_finite(self) -> Result<Self, FrameError> {
        if self.is_finite() {
            Ok(self)
        } else {
            Err(FrameError::NotFinite { frame: F::ID })
        }
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

impl<F: Frame> Add for AngularVelocity3<F> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::from_vector(self.vector + rhs.vector)
    }
}

impl<F: Frame> Sub for AngularVelocity3<F> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::from_vector(self.vector - rhs.vector)
    }
}

impl<F: Frame> Neg for AngularVelocity3<F> {
    type Output = Self;
    fn neg(self) -> Self {
        Self::from_vector(-self.vector)
    }
}

impl<F: Frame> Mul<f64> for AngularVelocity3<F> {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self::from_vector(self.vector * rhs)
    }
}

// ---------------------------------------------------------------------
// Quaternion<From, To>
// ---------------------------------------------------------------------

/// A unit quaternion that maps a vector expressed in `From`-frame
/// components to its expression in `To`-frame components.
///
/// The implementation follows nalgebra's `q * v` convention. OpenBMP
/// avoids the overloaded active/passive vocabulary in this API; the
/// type parameters state the component mapping directly.
#[derive(Copy, Clone, Debug)]
pub struct Quaternion<From: Frame, To: Frame> {
    /// Underlying nalgebra unit quaternion.
    pub q: UnitQuaternion<f64>,
    _phantom: PhantomData<(From, To)>,
}

fn is_valid_unit_tolerance(tolerance: f64) -> bool {
    tolerance.is_finite() && (0.0..1.0).contains(&tolerance)
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

    /// Construct from raw `(w, x, y, z)` components after validating
    /// finite values and near-unit magnitude.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::InvalidTolerance`] when `tolerance` is not
    /// finite or not in `[0, 1)`, [`FrameError::NotFinite`] when any
    /// component is `NaN` or infinite, and
    /// [`FrameError::NotNormalised`] when the raw quaternion magnitude
    /// differs from 1.0 by more than `tolerance`.
    pub fn from_wxyz_checked(
        w: f64,
        x: f64,
        y: f64,
        z: f64,
        tolerance: f64,
    ) -> Result<Self, FrameError> {
        if !is_valid_unit_tolerance(tolerance) {
            return Err(FrameError::InvalidTolerance { tolerance });
        }
        let raw = NalgebraQuaternion::new(w, x, y, z);
        if !raw.coords.iter().all(|component| component.is_finite()) {
            return Err(FrameError::NotFinite { frame: From::ID });
        }
        let magnitude = raw.norm();
        if (magnitude - 1.0).abs() > tolerance {
            return Err(FrameError::NotNormalised {
                magnitude,
                tolerance,
            });
        }
        Ok(Self::from_unit_quaternion(UnitQuaternion::from_quaternion(
            raw,
        )))
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

    /// Returns `true` if all underlying quaternion components are
    /// finite.
    #[must_use]
    pub fn is_finite(self) -> bool {
        self.q
            .into_inner()
            .coords
            .iter()
            .all(|component| component.is_finite())
    }

    /// Validate that every underlying quaternion component is finite.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError::NotFinite`] if any component is `NaN` or
    /// infinite.
    pub fn require_finite(self) -> Result<Self, FrameError> {
        if self.is_finite() {
            Ok(self)
        } else {
            Err(FrameError::NotFinite { frame: From::ID })
        }
    }

    /// Returns `true` if the underlying quaternion is normalised within
    /// `tolerance` of unit magnitude.
    #[must_use]
    pub fn is_normalised(self, tolerance: f64) -> bool {
        let magnitude = self.q.into_inner().norm();
        is_valid_unit_tolerance(tolerance)
            && magnitude.is_finite()
            && (magnitude - 1.0).abs() <= tolerance
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
    /// Returns [`FrameError::InvalidTolerance`] if `tolerance` is not
    /// finite or not in `[0, 1)`; returns
    /// [`FrameError::NotNormalised`] if the quaternion is not within
    /// tolerance of unit magnitude.
    pub fn require_normalised(self, tolerance: f64) -> Result<Self, FrameError> {
        if !is_valid_unit_tolerance(tolerance) {
            return Err(FrameError::InvalidTolerance { tolerance });
        }
        self.require_finite()?;
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
/// The profile determines which [`FrameTransform`] implementations and
/// time-aware methods are available. Phase-1 ships
/// [`FrameProfile::ToyFixedEarth`]; Phase-2.2 adds
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
    /// motion, no leap seconds. Phase-2.2 ships this profile;
    /// IERS-tabulated and SPICE-reference profiles are deferred.
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
// WGS84 geodetic constants (Phase 2.2)
//
// Defining-parameter values from NIMA TR 8350.2, *Department of
// Defense World Geodetic System 1984*, 3rd ed. (2000), tables 3.1
// and 3.5. Cited at the live NGA WGS84 portal:
// https://earth-info.nga.mil/index.php?dir=wgs84&action=wgs84.
//
// J2 lives in `openbmp-physics::gravity`; it's a gravity-model coefficient
// and not a frame primitive. Earth rotation rate, semi-major axis,
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

/// Scenario-declared local geodetic origin used by Phase-2 NED helpers.
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
/// Higher-fidelity profiles in later phases will additionally carry
/// pinned Earth-orientation tables.
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
    // Time-aware ECI ↔ ECEF transforms (Phase 2.2)
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
/// profile-gated transforms in later phases should expose fallible helper
/// constructors that use [`FrameError::TransformNotAvailable`].
///
/// ECI/ECEF transforms take [`SimTime`] because all non-toy Earth-fixed
/// profiles are time-dependent. The [`FrameProfile::ToyFixedEarth`]
/// implementation ignores time and remains identity.
pub trait FrameTransform<From: Frame, To: Frame> {
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
impl<F: Frame> FrameTransform<F, F> for FrameContext {
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
    fn position_difference_is_typed_displacement() {
        let a: Position3<Eci> = Position3::new(5.0, 7.0, 11.0);
        let b: Position3<Eci> = Position3::new(2.0, 3.0, 5.0);
        let d: Displacement3<Eci> = a - b;
        assert_abs_diff_eq!(d.vector.x, 3.0);
        assert_abs_diff_eq!(d.vector.y, 4.0);
        assert_abs_diff_eq!(d.vector.z, 6.0);
        assert_eq!(b + d, a);
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
    fn velocity_difference_is_typed_delta() {
        let v1: Velocity3<Eci> = Velocity3::new(10.0, 2.0, -4.0);
        let v0: Velocity3<Eci> = Velocity3::new(3.0, -1.0, -6.0);
        let dv: VelocityDelta3<Eci> = v1 - v0;
        assert_abs_diff_eq!(dv.vector.x, 7.0);
        assert_abs_diff_eq!(dv.vector.y, 3.0);
        assert_abs_diff_eq!(dv.vector.z, 2.0);
        assert_eq!(v0 + dv, v1);
    }

    #[test]
    fn velocity_times_duration_is_displacement() {
        let v: Velocity3<Eci> = Velocity3::new(10.0, -2.0, 0.5);
        let d: Displacement3<Eci> = v * Duration::from_seconds(2.0);
        assert_abs_diff_eq!(d.vector.x, 20.0);
        assert_abs_diff_eq!(d.vector.y, -4.0);
        assert_abs_diff_eq!(d.vector.z, 1.0);
    }

    #[test]
    fn acceleration_times_duration_updates_velocity() {
        let v0: Velocity3<Eci> = Velocity3::new(1.0, 2.0, 3.0);
        let a: Acceleration3<Eci> = Acceleration3::new(0.5, -1.0, 2.0);
        let v1 = v0 + a * Duration::from_seconds(4.0);
        assert_abs_diff_eq!(v1.vector.x, 3.0);
        assert_abs_diff_eq!(v1.vector.y, -2.0);
        assert_abs_diff_eq!(v1.vector.z, 11.0);
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
    fn quaternion_composition_matches_sequential_rotation() {
        let z_axis = nalgebra::Unit::new_normalize(nalgebra::Vector3::new(0.0, 0.0, 1.0));
        let x_axis = nalgebra::Unit::new_normalize(nalgebra::Vector3::new(1.0, 0.0, 0.0));
        let inertial_to_fixed: Quaternion<Eci, Ecef> =
            Quaternion::from_unit_quaternion(UnitQuaternion::from_axis_angle(&z_axis, 0.25));
        let fixed_to_body: Quaternion<Ecef, Body> =
            Quaternion::from_unit_quaternion(UnitQuaternion::from_axis_angle(&x_axis, -0.5));
        let inertial_to_body: Quaternion<Eci, Body> = inertial_to_fixed * fixed_to_body;
        let p: Position3<Eci> = Position3::new(3.0, 5.0, 7.0);
        let sequential = fixed_to_body.rotate_position(inertial_to_fixed.rotate_position(p));
        let composed = inertial_to_body.rotate_position(p);
        assert_abs_diff_eq!(composed.vector.x, sequential.vector.x, epsilon = 1.0e-12);
        assert_abs_diff_eq!(composed.vector.y, sequential.vector.y, epsilon = 1.0e-12);
        assert_abs_diff_eq!(composed.vector.z, sequential.vector.z, epsilon = 1.0e-12);
    }

    #[test]
    fn quaternion_from_wxyz_checked_rejects_bad_inputs() {
        let good: Result<Quaternion<Eci, Body>, FrameError> =
            Quaternion::from_wxyz_checked(1.0, 0.0, 0.0, 0.0, 1.0e-12);
        assert!(good.is_ok());

        let nonfinite: Result<Quaternion<Eci, Body>, FrameError> =
            Quaternion::from_wxyz_checked(f64::NAN, 0.0, 0.0, 0.0, 1.0e-12);
        assert!(matches!(nonfinite, Err(FrameError::NotFinite { .. })));

        let not_normalised: Result<Quaternion<Eci, Body>, FrameError> =
            Quaternion::from_wxyz_checked(2.0, 0.0, 0.0, 0.0, 1.0e-12);
        assert!(matches!(
            not_normalised,
            Err(FrameError::NotNormalised { .. })
        ));

        let bad_tolerance: Result<Quaternion<Eci, Body>, FrameError> =
            Quaternion::from_wxyz_checked(1.0, 0.0, 0.0, 0.0, -1.0);
        assert!(matches!(
            bad_tolerance,
            Err(FrameError::InvalidTolerance { .. })
        ));

        let meaningless_tolerance: Result<Quaternion<Eci, Body>, FrameError> =
            Quaternion::from_wxyz_checked(0.0, 0.0, 0.0, 0.0, 1.0);
        assert!(matches!(
            meaningless_tolerance,
            Err(FrameError::InvalidTolerance { .. })
        ));
    }

    #[test]
    fn quaternion_require_normalised_rejects_nonfinite_components() {
        let q: Quaternion<Eci, Body> = Quaternion::from_unit_quaternion(
            UnitQuaternion::new_unchecked(nalgebra::Quaternion::new(f64::NAN, 0.0, 0.0, 0.0)),
        );
        assert!(!q.is_finite());
        assert!(matches!(
            q.require_normalised(1.0e-12),
            Err(FrameError::NotFinite { .. })
        ));
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

    // -----------------------------------------------------------------
    // Phase 2.2.A — WGS84 frame profile tests
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
            assert!(LocalGeodeticOrigin::new_degrees(91.0, 0.0, 0.0).is_err());
            assert!(LocalGeodeticOrigin::new_degrees(0.0, 200.0, 0.0).is_err());
            assert!(LocalGeodeticOrigin::new_degrees(0.0, 0.0, f64::NAN).is_err());
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
