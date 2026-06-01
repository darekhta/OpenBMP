//! Coordinate frames and frame-tagged vectors.
//!
//! Frame tags are zero-sized types ([`Eci`], [`Ecef`], [`Ned`], [`Enu`],
//! [`Body`]) implementing the [`Frame`] trait. Vector-like types are
//! parameterised by frame: a [`Position3<Eci>`] is a different type
//! from a [`Position3<Ecef>`] and the compiler refuses to mix them.
//!
//! This module owns the *type-level* frame machinery — frame trait
//! and tag types, value types, [`Quaternion`] rotation, and
//! [`FrameError`] — without depending on any
//! Earth-specific physics constants. Time-aware transforms between
//! ECI, ECEF, and NED, the WGS84 ellipsoid constants,
//! `LocalGeodeticOrigin`, `FrameContext`, and the `FrameTransform`
//! trait + impls live in `openbmp-physics::frames`.
//!
//! No type in this module accesses wall-clock time, system RNG, or any
//! external Earth-orientation data.
//!
//! ```compile_fail
//! use openbmp_core::{Ecef, Eci, Position3};
//!
//! let inertial: Position3<Eci> = Position3::new(1.0, 0.0, 0.0);
//! let fixed: Position3<Ecef> = Position3::new(1.0, 0.0, 0.0);
//! let _ = inertial - fixed; // frame mismatch: does not compile
//! ```

use core::marker::PhantomData;
use core::ops::{Add, Mul, Neg, Sub};

use nalgebra::{Quaternion as NalgebraQuaternion, UnitQuaternion, Vector3};

use crate::error::FrameError;
use crate::time::Duration;

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
/// higher-fidelity profiles; identity to ECI in toy fixed-earth
/// profiles.
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
