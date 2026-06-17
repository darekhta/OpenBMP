//! `openbmp-multibody` — spatial-vector multibody dynamics substrate.
//!
//! This crate is the L2 home for the flexible/articulated multibody program.
//! It deliberately depends only on lower L0/L1 crates plus the hardware-portable
//! `openbmp-models` trait surface and `nalgebra`: no simulator, runner,
//! scenario, or flight-controller dependency is allowed here.
//! The first slice provides the deterministic spatial algebra, rigid spatial
//! inertia construction, joint subspace vocabulary, and topological tree shape
//! that the later ABA/RNEA/CRBA implementation consumes.
//!
//! Spatial vectors use Featherstone's angular-over-linear convention:
//! motion `v = [omega; v_linear]`, force `f = [moment; force_linear]`.

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

extern crate alloc;

use core::ops::{Add, Mul};

use alloc::vec;
use alloc::vec::Vec;

use nalgebra::{Matrix3, Quaternion as NalgebraQuaternion, SMatrix, SVector, Vector3};
use openbmp_core::{BodyId, SimTime};
use openbmp_models::{Integratable, SimStateDerivative, VehicleState};
use openbmp_state::MassProperties;
use thiserror::Error;

/// Six-component spatial vector storage.
pub type SpatialVector = SVector<f64, 6>;

/// Six-by-six spatial matrix storage.
pub type SpatialMatrix = SMatrix<f64, 6, 6>;

/// Spatial motion vector `v = [omega; v_linear]`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SpatialMotion {
    vector: SpatialVector,
}

impl SpatialMotion {
    /// Construct from angular and linear components.
    #[must_use]
    pub fn new(angular_rad_s: Vector3<f64>, linear_m_s: Vector3<f64>) -> Self {
        let mut vector = SpatialVector::zeros();
        vector.fixed_rows_mut::<3>(0).copy_from(&angular_rad_s);
        vector.fixed_rows_mut::<3>(3).copy_from(&linear_m_s);
        Self { vector }
    }

    /// Construct from raw angular-over-linear storage.
    #[must_use]
    pub const fn from_vector(vector: SpatialVector) -> Self {
        Self { vector }
    }

    /// Zero spatial motion.
    #[must_use]
    pub fn zero() -> Self {
        Self {
            vector: SpatialVector::zeros(),
        }
    }

    /// Returns the raw angular-over-linear vector.
    #[must_use]
    pub const fn vector(&self) -> &SpatialVector {
        &self.vector
    }

    /// Angular component.
    #[must_use]
    pub fn angular_rad_s(&self) -> Vector3<f64> {
        self.vector.fixed_rows::<3>(0).into_owned()
    }

    /// Linear component.
    #[must_use]
    pub fn linear_m_s(&self) -> Vector3<f64> {
        self.vector.fixed_rows::<3>(3).into_owned()
    }

    /// Motion cross-product matrix `v x`.
    #[must_use]
    pub fn crossm(&self) -> SpatialMatrix {
        let omega_cross = skew(self.angular_rad_s());
        let linear_cross = skew(self.linear_m_s());
        let mut out = SpatialMatrix::zeros();
        out.fixed_view_mut::<3, 3>(0, 0).copy_from(&omega_cross);
        out.fixed_view_mut::<3, 3>(3, 0).copy_from(&linear_cross);
        out.fixed_view_mut::<3, 3>(3, 3).copy_from(&omega_cross);
        out
    }

    /// Force cross-product matrix `v x* = -(v x)^T`.
    #[must_use]
    pub fn crossf(&self) -> SpatialMatrix {
        -self.crossm().transpose()
    }

    /// Returns `true` if every component is finite.
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.vector.iter().all(|v| v.is_finite())
    }
}

/// Spatial force vector `f = [moment; force_linear]`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SpatialForce {
    vector: SpatialVector,
}

impl SpatialForce {
    /// Construct from moment and linear force components.
    #[must_use]
    pub fn new(moment_n_m: Vector3<f64>, force_n: Vector3<f64>) -> Self {
        let mut vector = SpatialVector::zeros();
        vector.fixed_rows_mut::<3>(0).copy_from(&moment_n_m);
        vector.fixed_rows_mut::<3>(3).copy_from(&force_n);
        Self { vector }
    }

    /// Construct from raw moment-over-force storage.
    #[must_use]
    pub const fn from_vector(vector: SpatialVector) -> Self {
        Self { vector }
    }

    /// Zero spatial force.
    #[must_use]
    pub fn zero() -> Self {
        Self {
            vector: SpatialVector::zeros(),
        }
    }

    /// Returns the raw moment-over-force vector.
    #[must_use]
    pub const fn vector(&self) -> &SpatialVector {
        &self.vector
    }

    /// Moment component.
    #[must_use]
    pub fn moment_n_m(&self) -> Vector3<f64> {
        self.vector.fixed_rows::<3>(0).into_owned()
    }

    /// Linear force component.
    #[must_use]
    pub fn force_n(&self) -> Vector3<f64> {
        self.vector.fixed_rows::<3>(3).into_owned()
    }

    /// Returns `true` if every component is finite.
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.vector.iter().all(|v| v.is_finite())
    }
}

/// Spatial rigid-body inertia about a body frame.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SpatialInertia {
    matrix: SpatialMatrix,
}

impl SpatialInertia {
    /// Construct from a validated rigid-body mass-property set.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if the mass properties fail their existing
    /// state-layer validity checks or the resulting spatial inertia is
    /// non-finite.
    pub fn from_mass_properties(mass_properties: &MassProperties) -> Result<Self, MultibodyError> {
        mass_properties.require_valid(1.0e-12)?;
        let mass_kg = mass_properties.mass_kg();
        let com = mass_properties.center_of_mass_body.vector;
        let com_cross = skew(com);
        let top_left = mass_properties.inertia_body - mass_kg * com_cross * com_cross;
        let top_right = mass_kg * com_cross;
        let bottom_left = -mass_kg * com_cross;
        let bottom_right = mass_kg * Matrix3::identity();
        let mut matrix = SpatialMatrix::zeros();
        matrix.fixed_view_mut::<3, 3>(0, 0).copy_from(&top_left);
        matrix.fixed_view_mut::<3, 3>(0, 3).copy_from(&top_right);
        matrix.fixed_view_mut::<3, 3>(3, 0).copy_from(&bottom_left);
        matrix.fixed_view_mut::<3, 3>(3, 3).copy_from(&bottom_right);
        if !matrix.iter().all(|v| v.is_finite()) {
            return Err(MultibodyError::NonFinite {
                reason: "spatial inertia matrix contains non-finite components",
            });
        }
        Ok(Self { matrix })
    }

    /// Construct directly from a finite 6x6 matrix.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError::NonFinite`] if any element is non-finite.
    pub fn from_matrix(matrix: SpatialMatrix) -> Result<Self, MultibodyError> {
        if !matrix.iter().all(|v| v.is_finite()) {
            return Err(MultibodyError::NonFinite {
                reason: "spatial inertia matrix contains non-finite components",
            });
        }
        Ok(Self { matrix })
    }

    /// Returns the 6x6 spatial inertia matrix.
    #[must_use]
    pub const fn matrix(&self) -> &SpatialMatrix {
        &self.matrix
    }

    /// Applies the spatial inertia to a motion vector.
    #[must_use]
    pub fn multiply_motion(&self, motion: SpatialMotion) -> SpatialForce {
        SpatialForce::from_vector(self.matrix * motion.vector)
    }
}

/// Pluecker coordinate transform from a parent frame to a child frame.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PluckerTransform {
    rot_child_from_parent: Matrix3<f64>,
    translation_parent_m: Vector3<f64>,
}

impl PluckerTransform {
    /// Identity transform.
    #[must_use]
    pub fn identity() -> Self {
        Self {
            rot_child_from_parent: Matrix3::identity(),
            translation_parent_m: Vector3::zeros(),
        }
    }

    /// Construct from child-from-parent rotation and parent-to-child
    /// translation expressed in the parent frame.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError::NonFinite`] if any component is non-finite.
    pub fn new(
        rot_child_from_parent: Matrix3<f64>,
        translation_parent_m: Vector3<f64>,
    ) -> Result<Self, MultibodyError> {
        if !rot_child_from_parent.iter().all(|v| v.is_finite())
            || !translation_parent_m.iter().all(|v| v.is_finite())
        {
            return Err(MultibodyError::NonFinite {
                reason: "Pluecker transform components must be finite",
            });
        }
        Ok(Self {
            rot_child_from_parent,
            translation_parent_m,
        })
    }

    /// Child-from-parent rotation matrix.
    #[must_use]
    pub const fn rot_child_from_parent(&self) -> &Matrix3<f64> {
        &self.rot_child_from_parent
    }

    /// Parent-to-child translation expressed in the parent frame, metres.
    #[must_use]
    pub const fn translation_parent_m(&self) -> &Vector3<f64> {
        &self.translation_parent_m
    }

    /// Parent-to-child spatial motion transform matrix.
    #[must_use]
    pub fn motion_matrix_parent_to_child(&self) -> SpatialMatrix {
        let mut out = SpatialMatrix::zeros();
        let skew_translation = skew(self.translation_parent_m);
        out.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&self.rot_child_from_parent);
        out.fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&(-self.rot_child_from_parent * skew_translation));
        out.fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&self.rot_child_from_parent);
        out
    }

    /// Transform a spatial motion from parent-frame coordinates to child-frame
    /// coordinates.
    #[must_use]
    pub fn transform_motion_parent_to_child(&self, motion_parent: SpatialMotion) -> SpatialMotion {
        SpatialMotion::from_vector(self.motion_matrix_parent_to_child() * motion_parent.vector)
    }

    /// Transform a child-frame spatial force into parent-frame coordinates
    /// using the power-dual transform.
    #[must_use]
    pub fn transform_force_child_to_parent(&self, force_child: SpatialForce) -> SpatialForce {
        SpatialForce::from_vector(
            self.motion_matrix_parent_to_child().transpose() * force_child.vector,
        )
    }

    /// Transform a child-frame spatial inertia into parent-frame coordinates.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError::NonFinite`] if the transformed matrix contains
    /// non-finite components.
    pub fn transform_inertia_child_to_parent(
        &self,
        inertia_child: SpatialInertia,
    ) -> Result<SpatialInertia, MultibodyError> {
        let x = self.motion_matrix_parent_to_child();
        SpatialInertia::from_matrix(x.transpose() * inertia_child.matrix * x)
    }

    /// Compose two parent-to-child transforms.
    ///
    /// `self` maps frame `A` to frame `B`; `next` maps frame `B` to frame `C`.
    /// The returned transform maps `A` to `C`.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError::NonFinite`] if the composed transform contains
    /// non-finite components.
    pub fn then(&self, next: &Self) -> Result<Self, MultibodyError> {
        let rot_child_from_parent = next.rot_child_from_parent * self.rot_child_from_parent;
        let translation_parent_m = self.translation_parent_m
            + self.rot_child_from_parent.transpose() * next.translation_parent_m;
        Self::new(rot_child_from_parent, translation_parent_m)
    }
}

/// Joint type connecting a tree body to its parent.
#[derive(Clone, Debug, PartialEq)]
pub enum Joint {
    /// Six-degree-of-freedom free-flyer root.
    FreeFlyer,
    /// One-degree-of-freedom revolute joint with a body-frame unit axis.
    Revolute {
        /// Joint axis expressed in the child body frame.
        axis_body_unit: Vector3<f64>,
    },
    /// One-degree-of-freedom prismatic joint with a body-frame unit axis.
    Prismatic {
        /// Joint axis expressed in the child body frame.
        axis_body_unit: Vector3<f64>,
    },
    /// Welded fixed joint.
    Welded {
        /// Whether this welded joint has been released for a later separation
        /// transition. The first slice does not perform release dynamics.
        released: bool,
    },
    /// Three-degree-of-freedom spherical joint.
    Spherical,
}

impl Joint {
    /// Construct a normalized revolute joint.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError::InvalidParameter`] if the axis is zero or
    /// non-finite.
    pub fn revolute(axis_body: Vector3<f64>) -> Result<Self, MultibodyError> {
        Ok(Self::Revolute {
            axis_body_unit: unit_axis(axis_body, "revolute joint axis")?,
        })
    }

    /// Construct a normalized prismatic joint.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError::InvalidParameter`] if the axis is zero or
    /// non-finite.
    pub fn prismatic(axis_body: Vector3<f64>) -> Result<Self, MultibodyError> {
        Ok(Self::Prismatic {
            axis_body_unit: unit_axis(axis_body, "prismatic joint axis")?,
        })
    }

    /// Number of generalized coordinates for this joint.
    #[must_use]
    pub const fn n_q(&self) -> usize {
        match self {
            Self::FreeFlyer => 7,
            Self::Revolute { .. } | Self::Prismatic { .. } => 1,
            Self::Welded { .. } => 0,
            Self::Spherical => 4,
        }
    }

    /// Number of generalized velocity coordinates for this joint.
    #[must_use]
    pub const fn n_qd(&self) -> usize {
        match self {
            Self::FreeFlyer => 6,
            Self::Revolute { .. } | Self::Prismatic { .. } => 1,
            Self::Welded { .. } => 0,
            Self::Spherical => 3,
        }
    }

    /// Motion subspace columns in locked order.
    #[must_use]
    pub fn motion_subspace(&self) -> Vec<SpatialMotion> {
        match self {
            Self::FreeFlyer => {
                let mut columns = Vec::with_capacity(6);
                for axis in 0..3 {
                    let mut angular = Vector3::zeros();
                    angular[axis] = 1.0;
                    columns.push(SpatialMotion::new(angular, Vector3::zeros()));
                }
                for axis in 0..3 {
                    let mut linear = Vector3::zeros();
                    linear[axis] = 1.0;
                    columns.push(SpatialMotion::new(Vector3::zeros(), linear));
                }
                columns
            }
            Self::Revolute { axis_body_unit } => {
                vec![SpatialMotion::new(*axis_body_unit, Vector3::zeros())]
            }
            Self::Prismatic { axis_body_unit } => {
                vec![SpatialMotion::new(Vector3::zeros(), *axis_body_unit)]
            }
            Self::Welded { .. } => Vec::new(),
            Self::Spherical => {
                let mut columns = Vec::with_capacity(3);
                for axis in 0..3 {
                    let mut angular = Vector3::zeros();
                    angular[axis] = 1.0;
                    columns.push(SpatialMotion::new(angular, Vector3::zeros()));
                }
                columns
            }
        }
    }

    /// Configuration-dependent parent-to-child transform contributed by this
    /// joint.
    ///
    /// Coordinate order is:
    ///
    /// * free-flyer: `[qw, qx, qy, qz, px, py, pz]`
    /// * revolute: `[theta_rad]`
    /// * prismatic: `[displacement_m]`
    /// * welded: `[]`
    /// * spherical: `[qw, qx, qy, qz]`
    ///
    /// Quaternion coordinates are normalized during evaluation so small
    /// integration drift does not change the transform scale.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] when the coordinate slice length is wrong,
    /// any coordinate is non-finite, or a quaternion has zero norm.
    pub fn joint_transform(&self, q: &[f64]) -> Result<PluckerTransform, MultibodyError> {
        if q.len() != self.n_q() {
            return Err(MultibodyError::GeneralizedVectorDimensionMismatch {
                vector: "joint q",
                expected: self.n_q(),
                actual: q.len(),
            });
        }
        if !q.iter().all(|v| v.is_finite()) {
            return Err(MultibodyError::NonFinite {
                reason: "joint coordinates contain non-finite components",
            });
        }
        match self {
            Self::FreeFlyer => {
                let rot_child_from_parent =
                    rotation_parent_from_child_from_quaternion(&q[0..4])?.transpose();
                PluckerTransform::new(rot_child_from_parent, Vector3::new(q[4], q[5], q[6]))
            }
            Self::Revolute { axis_body_unit } => PluckerTransform::new(
                rotation_about_unit_axis(*axis_body_unit, q[0]),
                Vector3::zeros(),
            ),
            Self::Prismatic { axis_body_unit } => {
                PluckerTransform::new(Matrix3::identity(), *axis_body_unit * q[0])
            }
            Self::Welded { .. } => Ok(PluckerTransform::identity()),
            Self::Spherical => PluckerTransform::new(
                rotation_parent_from_child_from_quaternion(q)?.transpose(),
                Vector3::zeros(),
            ),
        }
    }
}

/// Index of a body in a topologically ordered multibody tree.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct BodyIndex(usize);

impl BodyIndex {
    /// Construct from a raw zero-based index.
    #[must_use]
    pub const fn new(index: usize) -> Self {
        Self(index)
    }

    /// Returns the raw zero-based index.
    #[must_use]
    pub const fn index(self) -> usize {
        self.0
    }
}

/// Body specification supplied to [`MultibodyTree::new`].
#[derive(Clone, Debug, PartialEq)]
pub struct TreeBodySpec {
    /// Stable body identifier.
    pub id: BodyId,
    /// Parent body index. `None` marks the root.
    pub parent: Option<BodyIndex>,
    /// Joint connecting this body to its parent.
    pub joint: Joint,
    /// Rigid spatial inertia.
    pub inertia: SpatialInertia,
    /// Fixed parent-to-body tree transform.
    pub parent_to_body: PluckerTransform,
}

/// Body stored inside a validated [`MultibodyTree`].
#[derive(Clone, Debug, PartialEq)]
pub struct TreeBody {
    /// Stable body identifier.
    pub id: BodyId,
    /// Parent body index. `None` marks the root.
    pub parent: Option<BodyIndex>,
    /// Joint connecting this body to its parent.
    pub joint: Joint,
    /// Rigid spatial inertia.
    pub inertia: SpatialInertia,
    /// Fixed parent-to-body tree transform.
    pub parent_to_body: PluckerTransform,
    /// Offset into the generalized coordinate vector.
    pub q_offset: usize,
    /// Offset into the generalized velocity vector.
    pub qd_offset: usize,
}

/// Dense joint-space inertia matrix.
///
/// Stored row-major in a deterministic `Vec<f64>` so the crate remains usable
/// without `std`.
#[derive(Clone, Debug, PartialEq)]
pub struct JointSpaceInertia {
    dimension: usize,
    values_row_major: Vec<f64>,
}

impl JointSpaceInertia {
    /// Construct from row-major values.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if the value count is not `dimension^2` or
    /// any element is non-finite.
    pub fn new(dimension: usize, values_row_major: Vec<f64>) -> Result<Self, MultibodyError> {
        if values_row_major.len() != dimension * dimension {
            return Err(MultibodyError::GeneralizedVectorDimensionMismatch {
                vector: "joint-space inertia values",
                expected: dimension * dimension,
                actual: values_row_major.len(),
            });
        }
        if !values_row_major.iter().all(|v| v.is_finite()) {
            return Err(MultibodyError::NonFinite {
                reason: "joint-space inertia matrix contains non-finite components",
            });
        }
        Ok(Self {
            dimension,
            values_row_major,
        })
    }

    /// Zero matrix of dimension `dimension`.
    #[must_use]
    pub fn zeros(dimension: usize) -> Self {
        Self {
            dimension,
            values_row_major: vec![0.0; dimension * dimension],
        }
    }

    /// Matrix dimension.
    #[must_use]
    pub const fn dimension(&self) -> usize {
        self.dimension
    }

    /// Row-major matrix values.
    #[must_use]
    pub fn values_row_major(&self) -> &[f64] {
        &self.values_row_major
    }

    /// Return one matrix element, or `None` when indices are outside the
    /// matrix.
    #[must_use]
    pub fn at(&self, row: usize, col: usize) -> Option<f64> {
        if row >= self.dimension || col >= self.dimension {
            return None;
        }
        Some(self.values_row_major[row * self.dimension + col])
    }

    fn set_symmetric(&mut self, row: usize, col: usize, value: f64) {
        self.values_row_major[row * self.dimension + col] = value;
        self.values_row_major[col * self.dimension + row] = value;
    }
}

/// Generalized-coordinate multibody state derivative.
///
/// `q_dot` has one entry for every generalized coordinate in
/// [`MultibodyState::q`]. `qd_dot` has one entry for every generalized
/// velocity in [`MultibodyState::qd`]. For quaternion-coordinate joints,
/// `q_dot` is the already-lifted coordinate derivative; use
/// [`MultibodyTree::coordinate_derivative_from_velocity`] when converting a
/// [`MultibodyState`] velocity vector into this representation.
#[derive(Clone, Debug, PartialEq)]
pub struct MultibodyDerivative {
    q_dot: Vec<f64>,
    qd_dot: Vec<f64>,
}

impl MultibodyDerivative {
    /// Construct from raw generalized-coordinate and velocity derivatives.
    #[must_use]
    pub fn new(q_dot: Vec<f64>, qd_dot: Vec<f64>) -> Self {
        Self { q_dot, qd_dot }
    }

    /// All-zero derivative with explicit dimensions.
    #[must_use]
    pub fn zero(n_q: usize, n_qd: usize) -> Self {
        Self {
            q_dot: vec![0.0; n_q],
            qd_dot: vec![0.0; n_qd],
        }
    }

    /// Generalized-coordinate derivatives.
    #[must_use]
    pub fn q_dot(&self) -> &[f64] {
        &self.q_dot
    }

    /// Generalized-velocity derivatives.
    #[must_use]
    pub fn qd_dot(&self) -> &[f64] {
        &self.qd_dot
    }

    /// Returns `true` if every derivative component is finite.
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.q_dot.iter().all(|v| v.is_finite()) && self.qd_dot.iter().all(|v| v.is_finite())
    }

    /// Total scalar dimension.
    #[must_use]
    pub fn dimension(&self) -> usize {
        self.q_dot.len() + self.qd_dot.len()
    }

    /// Locked-order L2 norm over `q_dot` followed by `qd_dot`.
    #[must_use]
    pub fn l2_norm(&self) -> f64 {
        let mut sum = 0.0;
        for value in &self.q_dot {
            sum += value * value;
        }
        for value in &self.qd_dot {
            sum += value * value;
        }
        <f64 as nalgebra::ComplexField>::sqrt(sum)
    }

    /// Componentwise derivative sum.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if the two derivatives have different
    /// dimensions or the result contains non-finite components.
    pub fn checked_add(&self, rhs: &Self) -> Result<Self, MultibodyError> {
        if self.q_dot.len() != rhs.q_dot.len() {
            return Err(MultibodyError::GeneralizedVectorDimensionMismatch {
                vector: "q_dot",
                expected: self.q_dot.len(),
                actual: rhs.q_dot.len(),
            });
        }
        if self.qd_dot.len() != rhs.qd_dot.len() {
            return Err(MultibodyError::GeneralizedVectorDimensionMismatch {
                vector: "qd_dot",
                expected: self.qd_dot.len(),
                actual: rhs.qd_dot.len(),
            });
        }
        let out = Self {
            q_dot: self
                .q_dot
                .iter()
                .zip(rhs.q_dot.iter())
                .map(|(lhs, rhs)| lhs + rhs)
                .collect(),
            qd_dot: self
                .qd_dot
                .iter()
                .zip(rhs.qd_dot.iter())
                .map(|(lhs, rhs)| lhs + rhs)
                .collect(),
        };
        if !out.is_finite() {
            return Err(MultibodyError::NonFinite {
                reason: "multibody derivative sum contains non-finite components",
            });
        }
        Ok(out)
    }

    /// Componentwise scalar multiplication.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if `scale` or any output component is
    /// non-finite.
    pub fn scaled(&self, scale: f64) -> Result<Self, MultibodyError> {
        if !scale.is_finite() {
            return Err(MultibodyError::NonFinite {
                reason: "multibody derivative scale is non-finite",
            });
        }
        let out = Self {
            q_dot: self.q_dot.iter().map(|value| value * scale).collect(),
            qd_dot: self.qd_dot.iter().map(|value| value * scale).collect(),
        };
        if !out.is_finite() {
            return Err(MultibodyError::NonFinite {
                reason: "scaled multibody derivative contains non-finite components",
            });
        }
        Ok(out)
    }
}

impl Add for MultibodyDerivative {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        assert_eq!(
            self.q_dot.len(),
            rhs.q_dot.len(),
            "multibody q_dot dimensions must match for derivative addition"
        );
        assert_eq!(
            self.qd_dot.len(),
            rhs.qd_dot.len(),
            "multibody qd_dot dimensions must match for derivative addition"
        );
        Self {
            q_dot: self
                .q_dot
                .into_iter()
                .zip(rhs.q_dot)
                .map(|(lhs, rhs)| lhs + rhs)
                .collect(),
            qd_dot: self
                .qd_dot
                .into_iter()
                .zip(rhs.qd_dot)
                .map(|(lhs, rhs)| lhs + rhs)
                .collect(),
        }
    }
}

impl Mul<f64> for MultibodyDerivative {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self {
        Self {
            q_dot: self.q_dot.into_iter().map(|value| value * rhs).collect(),
            qd_dot: self.qd_dot.into_iter().map(|value| value * rhs).collect(),
        }
    }
}

impl SimStateDerivative for MultibodyDerivative {
    fn is_finite(&self) -> bool {
        Self::is_finite(self)
    }

    fn l2_norm(&self) -> f64 {
        Self::l2_norm(self)
    }

    fn dimension(&self) -> usize {
        Self::dimension(self)
    }
}

/// Topologically ordered multibody tree.
#[derive(Clone, Debug, PartialEq)]
pub struct MultibodyTree {
    bodies: Vec<TreeBody>,
    n_q: usize,
    n_qd: usize,
}

impl MultibodyTree {
    /// Construct a tree from topologically ordered body specs.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if the tree is empty, has anything other
    /// than one root, has a non-free-flyer root, or references a parent that is
    /// not earlier in the deterministic body order.
    pub fn new(specs: Vec<TreeBodySpec>) -> Result<Self, MultibodyError> {
        if specs.is_empty() {
            return Err(MultibodyError::InvalidTopology {
                reason: "multibody tree requires at least one body",
            });
        }
        let root_count = specs.iter().filter(|body| body.parent.is_none()).count();
        if root_count != 1 {
            return Err(MultibodyError::InvalidTopology {
                reason: "multibody tree requires exactly one root",
            });
        }

        let mut bodies = Vec::with_capacity(specs.len());
        let mut n_q = 0usize;
        let mut n_qd = 0usize;
        for (index, spec) in specs.into_iter().enumerate() {
            match spec.parent {
                None => {
                    if !matches!(spec.joint, Joint::FreeFlyer) {
                        return Err(MultibodyError::InvalidTopology {
                            reason: "root body must use a free-flyer joint",
                        });
                    }
                }
                Some(parent) if parent.index() >= index => {
                    return Err(MultibodyError::InvalidTopology {
                        reason: "body parent index must precede child index",
                    });
                }
                Some(_) => {}
            }
            let q_offset = n_q;
            let qd_offset = n_qd;
            n_q += spec.joint.n_q();
            n_qd += spec.joint.n_qd();
            bodies.push(TreeBody {
                id: spec.id,
                parent: spec.parent,
                joint: spec.joint,
                inertia: spec.inertia,
                parent_to_body: spec.parent_to_body,
                q_offset,
                qd_offset,
            });
        }
        Ok(Self { bodies, n_q, n_qd })
    }

    /// Bodies in deterministic topological order.
    #[must_use]
    pub fn bodies(&self) -> &[TreeBody] {
        &self.bodies
    }

    /// Number of generalized coordinates.
    #[must_use]
    pub const fn n_q(&self) -> usize {
        self.n_q
    }

    /// Number of generalized velocity coordinates.
    #[must_use]
    pub const fn n_qd(&self) -> usize {
        self.n_qd
    }

    /// Validate that a state vector matches this tree's dimensions and
    /// contains only finite values.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] when the generalized coordinate dimensions
    /// or numeric components are invalid.
    pub fn validate_state(&self, state: &MultibodyState) -> Result<(), MultibodyError> {
        if state.q.len() != self.n_q || state.qd.len() != self.n_qd {
            return Err(MultibodyError::DimensionMismatch {
                expected_q: self.n_q,
                actual_q: state.q.len(),
                expected_qd: self.n_qd,
                actual_qd: state.qd.len(),
            });
        }
        if !state.is_finite() {
            return Err(MultibodyError::NonFinite {
                reason: "multibody state contains non-finite components",
            });
        }
        Ok(())
    }

    /// Validate that a derivative vector matches this tree's dimensions and
    /// contains only finite values.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if dimensions or numeric components are
    /// invalid.
    pub fn validate_derivative(
        &self,
        derivative: &MultibodyDerivative,
    ) -> Result<(), MultibodyError> {
        if derivative.q_dot.len() != self.n_q {
            return Err(MultibodyError::GeneralizedVectorDimensionMismatch {
                vector: "q_dot",
                expected: self.n_q,
                actual: derivative.q_dot.len(),
            });
        }
        if derivative.qd_dot.len() != self.n_qd {
            return Err(MultibodyError::GeneralizedVectorDimensionMismatch {
                vector: "qd_dot",
                expected: self.n_qd,
                actual: derivative.qd_dot.len(),
            });
        }
        if !derivative.is_finite() {
            return Err(MultibodyError::NonFinite {
                reason: "multibody derivative contains non-finite components",
            });
        }
        Ok(())
    }

    /// Build a simulator-adapter state from raw generalized vectors.
    ///
    /// The returned [`MultibodySimState`] carries the tree-derived
    /// quaternion-coordinate offsets needed by
    /// [`openbmp_models::Integratable::project`], while preserving the raw
    /// generalized state shape for dynamics calls.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if the raw state dimensions, numeric
    /// components, or quaternion coordinate slices are invalid for this tree.
    pub fn sim_state(
        &self,
        time: SimTime,
        q: Vec<f64>,
        qd: Vec<f64>,
    ) -> Result<MultibodySimState, MultibodyError> {
        self.sim_state_from_state(MultibodyState::new(time, q, qd))
    }

    /// Attach this tree's projection metadata to an existing multibody state.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if the state does not match this tree's
    /// dimensions or contains invalid quaternion coordinate slices.
    pub fn sim_state_from_state(
        &self,
        state: MultibodyState,
    ) -> Result<MultibodySimState, MultibodyError> {
        self.validate_state(&state)?;
        MultibodySimState::new(state, self.quaternion_coordinate_offsets())
    }

    /// Lift generalized velocities into generalized-coordinate derivatives.
    ///
    /// Scalar revolute/prismatic joints map `qd` directly into `q_dot`.
    /// Free-flyer and spherical quaternion coordinates use the same
    /// body-frame quaternion convention as the rigid-body state model,
    /// `q_dot = 0.5 * q ⊗ [0, omega_body]`. The quaternion-rate lift uses the
    /// raw finite, nonzero RK substep quaternion and leaves normalization to
    /// projection after the weighted sum. Free-flyer quaternions store the
    /// parent-from-child rotation, so translation rates are converted from
    /// body-frame linear velocity to parent-frame position derivatives with the
    /// current normalized quaternion.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if the state dimensions/numeric components
    /// are invalid or a quaternion coordinate slice has zero norm.
    pub fn coordinate_derivative_from_velocity(
        &self,
        state: &MultibodyState,
    ) -> Result<Vec<f64>, MultibodyError> {
        self.validate_state(state)?;
        let mut q_dot = Vec::with_capacity(self.n_q);
        for body in &self.bodies {
            let q_start = body.q_offset;
            let qd_start = body.qd_offset;
            match body.joint {
                Joint::FreeFlyer => {
                    let q = &state.q[q_start..q_start + 7];
                    let qd = &state.qd[qd_start..qd_start + 6];
                    q_dot.extend_from_slice(&quaternion_derivative_from_body_rate(
                        &q[0..4],
                        Vector3::new(qd[0], qd[1], qd[2]),
                    )?);
                    let rot_parent_from_child =
                        rotation_parent_from_child_from_quaternion(&q[0..4])?;
                    let linear_body = Vector3::new(qd[3], qd[4], qd[5]);
                    let translation_dot_parent = rot_parent_from_child * linear_body;
                    q_dot.extend_from_slice(translation_dot_parent.as_slice());
                }
                Joint::Revolute { .. } | Joint::Prismatic { .. } => {
                    q_dot.push(state.qd[qd_start]);
                }
                Joint::Welded { .. } => {}
                Joint::Spherical => {
                    q_dot.extend_from_slice(&quaternion_derivative_from_body_rate(
                        &state.q[q_start..q_start + 4],
                        Vector3::new(
                            state.qd[qd_start],
                            state.qd[qd_start + 1],
                            state.qd[qd_start + 2],
                        ),
                    )?);
                }
            }
        }
        if !q_dot.iter().all(|value| value.is_finite()) {
            return Err(MultibodyError::NonFinite {
                reason: "coordinate derivative contains non-finite components",
            });
        }
        Ok(q_dot)
    }

    /// Build a complete multibody state derivative from state and
    /// generalized accelerations.
    ///
    /// This is the adapter-facing bridge between forward dynamics, which
    /// returns `qdd`, and generalized-state integration, which advances
    /// `q_dot`/`qd_dot`.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if the state, acceleration vector, or lifted
    /// derivative is dimensionally invalid or non-finite.
    pub fn derivative_from_state_and_acceleration(
        &self,
        state: &MultibodyState,
        qdd: &[f64],
    ) -> Result<MultibodyDerivative, MultibodyError> {
        self.validate_generalized_vector("generalized acceleration", qdd)?;
        let derivative = MultibodyDerivative::new(
            self.coordinate_derivative_from_velocity(state)?,
            qdd.to_vec(),
        );
        self.validate_derivative(&derivative)?;
        Ok(derivative)
    }

    /// Build a complete simulator derivative by evaluating ABA forward
    /// dynamics and lifting the resulting generalized accelerations.
    ///
    /// This is the kernel-facing bridge from generalized loads to the
    /// `openbmp-models` derivative shape. The inputs are the same locked-order
    /// force inputs accepted by [`Self::forward_dynamics_aba_at_state`], and the
    /// returned derivative pairs the ABA `qdd` with the state-dependent
    /// kinematic lift from [`Self::coordinate_derivative_from_velocity`].
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if forward dynamics or derivative lifting
    /// rejects dimensions, non-finite inputs, singular articulated blocks, or
    /// invalid quaternion coordinates.
    pub fn derivative_from_forward_dynamics(
        &self,
        state: &MultibodyState,
        generalized_forces: &[f64],
        root_parent_acceleration: SpatialMotion,
        external_forces_body: &[SpatialForce],
    ) -> Result<MultibodyDerivative, MultibodyError> {
        let qdd = self.forward_dynamics_aba_at_state(
            state,
            generalized_forces,
            root_parent_acceleration,
            external_forces_body,
        )?;
        self.derivative_from_state_and_acceleration(state, &qdd)
    }

    /// Map root free-flyer body loads into locked-order generalized forces.
    ///
    /// The root free-flyer generalized-force order follows
    /// [`Joint::motion_subspace`]: body-frame moment `(x, y, z)` followed by
    /// body-frame force `(x, y, z)`. This helper accepts the force in the root's
    /// virtual-parent frame so a runner can pass an inertial force from the
    /// existing force-model surface, then rotates it into the root body frame.
    /// Non-root generalized-force entries are filled with zero; later joint
    /// actuators can add to the returned vector in the same locked order.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if the state is invalid, either load vector is
    /// non-finite, or the root quaternion is invalid.
    pub fn root_free_flyer_generalized_forces_from_loads(
        &self,
        state: &MultibodyState,
        moment_body_n_m: Vector3<f64>,
        force_parent_n: Vector3<f64>,
    ) -> Result<Vec<f64>, MultibodyError> {
        self.validate_state(state)?;
        if !moment_body_n_m.iter().all(|value| value.is_finite())
            || !force_parent_n.iter().all(|value| value.is_finite())
        {
            return Err(MultibodyError::NonFinite {
                reason: "root free-flyer loads contain non-finite components",
            });
        }

        let root = &self.bodies[0];
        debug_assert!(matches!(root.joint, Joint::FreeFlyer));
        let q_start = root.q_offset;
        let rot_parent_from_body =
            rotation_parent_from_child_from_quaternion(&state.q[q_start..q_start + 4])?;
        let force_body_n = rot_parent_from_body.transpose() * force_parent_n;
        let mut generalized_forces = vec![0.0; self.n_qd];
        generalized_forces[root.qd_offset] = moment_body_n_m.x;
        generalized_forces[root.qd_offset + 1] = moment_body_n_m.y;
        generalized_forces[root.qd_offset + 2] = moment_body_n_m.z;
        generalized_forces[root.qd_offset + 3] = force_body_n.x;
        generalized_forces[root.qd_offset + 4] = force_body_n.y;
        generalized_forces[root.qd_offset + 5] = force_body_n.z;
        Ok(generalized_forces)
    }

    /// Build a complete derivative from root free-flyer force/moment loads.
    ///
    /// This combines [`Self::root_free_flyer_generalized_forces_from_loads`] and
    /// [`Self::derivative_from_forward_dynamics`] for the single-body bridge the
    /// runner will use first, while still accepting the root parent acceleration
    /// and per-body external spatial-force inputs used by the full ABA path.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if load mapping, ABA forward dynamics, or
    /// derivative lifting rejects the supplied inputs.
    pub fn derivative_from_root_free_flyer_loads(
        &self,
        state: &MultibodyState,
        moment_body_n_m: Vector3<f64>,
        force_parent_n: Vector3<f64>,
        root_parent_acceleration: SpatialMotion,
        external_forces_body: &[SpatialForce],
    ) -> Result<MultibodyDerivative, MultibodyError> {
        let generalized_forces = self.root_free_flyer_generalized_forces_from_loads(
            state,
            moment_body_n_m,
            force_parent_n,
        )?;
        self.derivative_from_forward_dynamics(
            state,
            &generalized_forces,
            root_parent_acceleration,
            external_forces_body,
        )
    }

    /// Deterministic componentwise state advance followed by quaternion
    /// projection.
    ///
    /// This mirrors the integrator-side `advance_by` shape while preserving a
    /// tree-owned projection path for callers that keep raw [`MultibodyState`]
    /// values instead of the [`MultibodySimState`] adapter. The tree knows which
    /// coordinate slices are free-flyer or spherical quaternions.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if state/derivative dimensions are invalid,
    /// `h_seconds` is non-finite, quaternion projection fails, or the advanced
    /// state contains non-finite values.
    pub fn advance_state_by(
        &self,
        state: &MultibodyState,
        h_seconds: f64,
        derivative: &MultibodyDerivative,
    ) -> Result<MultibodyState, MultibodyError> {
        self.validate_state(state)?;
        self.validate_derivative(derivative)?;
        if !h_seconds.is_finite() {
            return Err(MultibodyError::NonFinite {
                reason: "integration step is non-finite",
            });
        }

        let mut q = Vec::with_capacity(self.n_q);
        for index in 0..self.n_q {
            q.push(state.q[index] + h_seconds * derivative.q_dot[index]);
        }
        let mut qd = Vec::with_capacity(self.n_qd);
        for index in 0..self.n_qd {
            qd.push(state.qd[index] + h_seconds * derivative.qd_dot[index]);
        }
        let mut advanced = MultibodyState::new(
            SimTime::from_seconds(state.time.as_seconds() + h_seconds),
            q,
            qd,
        );
        self.project_state(&mut advanced)?;
        self.validate_state(&advanced)?;
        Ok(advanced)
    }

    /// Project quaternion coordinate slices in-place.
    ///
    /// Free-flyer and spherical joints carry quaternion coordinates. This
    /// normalizes each such slice without changing translation, scalar joint,
    /// or generalized-velocity coordinates.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if the state dimensions are invalid, any
    /// component is non-finite, or a quaternion slice has zero norm.
    pub fn project_state(&self, state: &mut MultibodyState) -> Result<(), MultibodyError> {
        self.validate_state(state)?;
        for body in &self.bodies {
            match body.joint {
                Joint::FreeFlyer | Joint::Spherical => {
                    normalize_quaternion_slice(&mut state.q[body.q_offset..body.q_offset + 4])?;
                }
                Joint::Revolute { .. } | Joint::Prismatic { .. } | Joint::Welded { .. } => {}
            }
        }
        Ok(())
    }

    /// Locked-order scalar size of a multibody state.
    ///
    /// Components are summed as all `q` entries followed by all `qd` entries,
    /// matching [`MultibodyDerivative::l2_norm`] and
    /// [`Self::weighted_error_norm`].
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if the state is dimensionally invalid or
    /// non-finite.
    pub fn scalar_state_size(&self, state: &MultibodyState) -> Result<f64, MultibodyError> {
        self.validate_state(state)?;
        let mut sum = 0.0;
        for value in &state.q {
            sum += value * value;
        }
        for value in &state.qd {
            sum += value * value;
        }
        Ok(<f64 as nalgebra::ComplexField>::sqrt(sum))
    }

    /// Per-component scaled RMS error norm for adaptive integrator adapters.
    ///
    /// This follows the same shape as `openbmp-models::Integratable`, walking
    /// `q` entries then `qd` entries in a locked order:
    ///
    /// `sqrt((1 / N) * sum((h * error_deriv_i / (atol + rtol * max(|prev_i|,
    /// end_i|)))^2))`
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if either state, the derivative, step, or
    /// tolerance inputs are invalid.
    pub fn weighted_error_norm(
        &self,
        end_state: &MultibodyState,
        prev_state: &MultibodyState,
        error_derivative: &MultibodyDerivative,
        h_seconds: f64,
        atol: f64,
        rtol: f64,
    ) -> Result<f64, MultibodyError> {
        self.validate_state(end_state)?;
        self.validate_state(prev_state)?;
        self.validate_derivative(error_derivative)?;
        if !h_seconds.is_finite() || !atol.is_finite() || !rtol.is_finite() {
            return Err(MultibodyError::NonFinite {
                reason: "weighted error norm inputs contain non-finite components",
            });
        }
        if atol <= 0.0 || rtol < 0.0 {
            return Err(MultibodyError::InvalidParameter {
                reason: "weighted error norm tolerances must satisfy atol > 0 and rtol >= 0",
            });
        }

        let mut sum = 0.0;
        for index in 0..self.n_q {
            sum += weighted_error_term(
                prev_state.q[index],
                end_state.q[index],
                error_derivative.q_dot[index],
                h_seconds,
                atol,
                rtol,
            );
        }
        for index in 0..self.n_qd {
            sum += weighted_error_term(
                prev_state.qd[index],
                end_state.qd[index],
                error_derivative.qd_dot[index],
                h_seconds,
                atol,
                rtol,
            );
        }
        let dimension = (self.n_q + self.n_qd) as f64;
        Ok(<f64 as nalgebra::ComplexField>::sqrt(sum / dimension))
    }

    /// Effective parent-to-child transform for a body at the supplied
    /// generalized state.
    ///
    /// This composes the body-fixed tree transform with the joint's
    /// configuration-dependent transform.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if `body_index` is outside the tree, the
    /// state has invalid dimensions/non-finite values, or the joint transform
    /// rejects its coordinate slice.
    pub fn body_transform_parent_to_child_at_state(
        &self,
        body_index: BodyIndex,
        state: &MultibodyState,
    ) -> Result<PluckerTransform, MultibodyError> {
        self.validate_state(state)?;
        let body = self
            .bodies
            .get(body_index.index())
            .ok_or(MultibodyError::InvalidBodyIndex {
                index: body_index.index(),
                body_count: self.bodies.len(),
            })?;
        let q_start = body.q_offset;
        let q_end = q_start + body.joint.n_q();
        let joint_transform = body.joint.joint_transform(&state.q[q_start..q_end])?;
        body.parent_to_body.then(&joint_transform)
    }

    /// Release a welded subtree into an independent free-flyer tree.
    ///
    /// The selected body must be a non-root `Welded { released: false }` body.
    /// The returned tree preserves the selected subtree's deterministic order,
    /// converts the selected body into a free-flyer root, and copies descendant
    /// joint coordinates/velocities unchanged. The returned state seeds the new
    /// free-flyer root from the selected body's parent-frame pose and body-frame
    /// spatial velocity at the release instant.
    ///
    /// This is the state handoff substrate for momentum-continuing separation.
    /// Runner-side propagation replacement remains a separate integration step.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if the state is invalid, the body index is out
    /// of range, the selected body is the root, the selected joint is not an
    /// unreleased welded joint, or the remapped subtree/state fails validation.
    pub fn release_welded_subtree_as_free_flyer(
        &self,
        state: &MultibodyState,
        release_body: BodyIndex,
    ) -> Result<(Self, MultibodyState), MultibodyError> {
        self.validate_state(state)?;
        let release_index = release_body.index();
        let release = self
            .bodies
            .get(release_index)
            .ok_or(MultibodyError::InvalidBodyIndex {
                index: release_index,
                body_count: self.bodies.len(),
            })?;
        if release.parent.is_none() {
            return Err(MultibodyError::InvalidTopology {
                reason: "released welded subtree root must not be the existing root",
            });
        }
        match release.joint {
            Joint::Welded { released: false } => {}
            Joint::Welded { released: true } => {
                return Err(MultibodyError::InvalidParameter {
                    reason: "welded joint is already marked released",
                });
            }
            _ => {
                return Err(MultibodyError::InvalidParameter {
                    reason: "released subtree root must use an unreleased welded joint",
                });
            }
        }

        let transforms = self.body_transforms_parent_to_child_at_state(state)?;
        let root_transforms = self.body_transforms_root_parent_to_child(&transforms)?;
        let velocities = self.body_spatial_velocities_with_transforms(state, &transforms);

        let mut index_map = vec![None; self.bodies.len()];
        let mut specs = Vec::new();
        for old_index in release_index..self.bodies.len() {
            if !self.is_descendant_or_self(old_index, release_index) {
                continue;
            }
            let old = &self.bodies[old_index];
            let new_index = BodyIndex::new(specs.len());
            index_map[old_index] = Some(new_index);
            let (parent, joint, parent_to_body) = if old_index == release_index {
                (None, Joint::FreeFlyer, PluckerTransform::identity())
            } else {
                let old_parent = old.parent.ok_or(MultibodyError::InvalidTopology {
                    reason: "released subtree descendant unexpectedly missing parent",
                })?;
                let new_parent =
                    index_map[old_parent.index()].ok_or(MultibodyError::InvalidTopology {
                        reason: "released subtree descendant parent was not remapped",
                    })?;
                (Some(new_parent), old.joint.clone(), old.parent_to_body)
            };
            specs.push(TreeBodySpec {
                id: old.id,
                parent,
                joint,
                inertia: old.inertia,
                parent_to_body,
            });
        }

        let released_tree = Self::new(specs)?;
        let released_transform = root_transforms[release_index];
        let rot_parent_from_child = released_transform.rot_child_from_parent.transpose();
        let quaternion_parent_from_child =
            nalgebra::UnitQuaternion::from_matrix(&rot_parent_from_child).into_inner();
        let translation_parent_m = released_transform.translation_parent_m;
        let mut q = vec![
            quaternion_parent_from_child.w,
            quaternion_parent_from_child.i,
            quaternion_parent_from_child.j,
            quaternion_parent_from_child.k,
            translation_parent_m.x,
            translation_parent_m.y,
            translation_parent_m.z,
        ];
        let mut qd = velocities[release_index].as_slice().to_vec();

        for (old_index, remapped) in index_map
            .iter()
            .enumerate()
            .skip(release_index + 1)
            .take(self.bodies.len() - release_index - 1)
        {
            if remapped.is_none() {
                continue;
            }
            let old = &self.bodies[old_index];
            q.extend_from_slice(&state.q[old.q_offset..old.q_offset + old.joint.n_q()]);
            qd.extend_from_slice(&state.qd[old.qd_offset..old.qd_offset + old.joint.n_qd()]);
        }

        let released_state = MultibodyState::new(state.time, q, qd);
        released_tree.validate_state(&released_state)?;
        Ok((released_tree, released_state))
    }

    /// Generalized momentum `p = H(q) qd` in locked generalized-velocity order.
    ///
    /// This is the deterministic momentum conjugate to [`MultibodyState::qd`].
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if the state is invalid, if CRBA rejects the
    /// state-dependent transforms, or if the resulting momentum is non-finite.
    pub fn generalized_momentum_at_state(
        &self,
        state: &MultibodyState,
    ) -> Result<Vec<f64>, MultibodyError> {
        self.validate_state(state)?;
        let inertia = self.joint_space_inertia_crba_at_state(state)?;
        let mut momentum = vec![0.0; self.n_qd];
        for (row, value) in momentum.iter_mut().enumerate() {
            for col in 0..self.n_qd {
                *value += inertia.values_row_major()[row * self.n_qd + col] * state.qd[col];
            }
        }
        if !momentum.iter().all(|value| value.is_finite()) {
            return Err(MultibodyError::NonFinite {
                reason: "generalized momentum contains non-finite components",
            });
        }
        Ok(momentum)
    }

    /// Kinetic energy in joules from the state-dependent joint-space inertia.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if generalized momentum evaluation fails or
    /// the resulting scalar is non-finite.
    pub fn kinetic_energy_at_state(&self, state: &MultibodyState) -> Result<f64, MultibodyError> {
        let momentum = self.generalized_momentum_at_state(state)?;
        let energy = 0.5
            * state
                .qd
                .iter()
                .zip(momentum.iter())
                .map(|(velocity, conjugate)| velocity * conjugate)
                .sum::<f64>();
        if !energy.is_finite() {
            return Err(MultibodyError::NonFinite {
                reason: "kinetic energy is non-finite",
            });
        }
        Ok(energy)
    }

    /// Composite-rigid-body joint-space inertia at a generalized state.
    ///
    /// This applies q-dependent joint transforms but still omits velocity bias,
    /// gravity, external forces, floating-base factorization, and ABA. It is a
    /// state-dependent CRBA substrate, not a full WP-01.1 completion claim.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] when state validation or a joint transform
    /// fails.
    pub fn joint_space_inertia_crba_at_state(
        &self,
        state: &MultibodyState,
    ) -> Result<JointSpaceInertia, MultibodyError> {
        let transforms = self.body_transforms_parent_to_child_at_state(state)?;
        self.joint_space_inertia_crba_with_transforms(&transforms)
    }

    /// Composite-rigid-body joint-space inertia for the current fixed tree
    /// transforms and q-independent joint subspaces.
    ///
    /// This is the first CRBA substrate used by WP-01.1 self-consistency tests.
    /// It does not yet apply q-dependent joint transforms, velocity terms, or
    /// floating-base factorization; those remain part of the full ABA/RNEA/CRBA
    /// work package.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if an intermediate transformed inertia is
    /// non-finite.
    pub fn joint_space_inertia_crba_fixed_transforms(
        &self,
    ) -> Result<JointSpaceInertia, MultibodyError> {
        let transforms: Vec<PluckerTransform> =
            self.bodies.iter().map(|body| body.parent_to_body).collect();
        self.joint_space_inertia_crba_with_transforms(&transforms)
    }

    fn joint_space_inertia_crba_with_transforms(
        &self,
        transforms_parent_to_child: &[PluckerTransform],
    ) -> Result<JointSpaceInertia, MultibodyError> {
        let mut composite_inertia: Vec<SpatialMatrix> = self
            .bodies
            .iter()
            .map(|body| *body.inertia.matrix())
            .collect();

        for child_index in (0..self.bodies.len()).rev() {
            if let Some(parent) = self.bodies[child_index].parent {
                let x = transforms_parent_to_child[child_index].motion_matrix_parent_to_child();
                let transformed = x.transpose() * composite_inertia[child_index] * x;
                composite_inertia[parent.index()] += transformed;
            }
        }

        let subspaces: Vec<Vec<SpatialMotion>> = self
            .bodies
            .iter()
            .map(|body| body.joint.motion_subspace())
            .collect();
        let mut h = JointSpaceInertia::zeros(self.n_qd);

        for body_index in 0..self.bodies.len() {
            let body = &self.bodies[body_index];
            for (local_col, column_motion) in subspaces[body_index].iter().enumerate() {
                let col = body.qd_offset + local_col;
                let mut force = composite_inertia[body_index] * column_motion.vector();
                let mut ancestor_index = body_index;
                loop {
                    let ancestor = &self.bodies[ancestor_index];
                    for (local_row, row_motion) in subspaces[ancestor_index].iter().enumerate() {
                        let row = ancestor.qd_offset + local_row;
                        h.set_symmetric(row, col, row_motion.vector().dot(&force));
                    }
                    if let Some(parent) = ancestor.parent {
                        let x = transforms_parent_to_child[ancestor_index]
                            .motion_matrix_parent_to_child();
                        force = x.transpose() * force;
                        ancestor_index = parent.index();
                    } else {
                        break;
                    }
                }
            }
        }

        JointSpaceInertia::new(self.n_qd, h.values_row_major)
    }

    /// Recursive Newton-Euler inverse dynamics at a generalized state.
    ///
    /// This uses q-dependent joint transforms, generalized velocities for the
    /// velocity-bias terms, a virtual-root parent acceleration seed, and
    /// per-body external spatial forces. The root acceleration seed is
    /// expressed in the root body's virtual parent frame; a gravity term can be
    /// supplied as `-g` in that frame and will be transformed through the root
    /// transform. External forces are expressed in each body's frame and are
    /// subtracted from the inertial force balance.
    ///
    /// This is a deterministic RNEA substrate for WP-01.1. Simulator adapter
    /// wiring, scenario opt-in, and external oracle validation remain separate
    /// work.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] when state validation, qdd validation, base
    /// acceleration validation, external-force validation, or a joint transform
    /// fails.
    pub fn inverse_dynamics_rnea_at_state(
        &self,
        state: &MultibodyState,
        qdd: &[f64],
        root_parent_acceleration: SpatialMotion,
        external_forces_body: &[SpatialForce],
    ) -> Result<Vec<f64>, MultibodyError> {
        let transforms = self.body_transforms_parent_to_child_at_state(state)?;
        self.inverse_dynamics_rnea_with_terms(
            &state.qd,
            qdd,
            &transforms,
            root_parent_acceleration,
            external_forces_body,
        )
    }

    /// Dense forward dynamics at a generalized state.
    ///
    /// This solves `H(q) qdd = tau - C(q, qd, a_root, f_ext)` using the
    /// state-dependent CRBA inertia matrix and the biased/forced RNEA path.
    /// It is deterministic and remains useful as an independent cross-check
    /// for the O(n) articulated-body path.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] when state validation, generalized-force
    /// validation, bias-force evaluation, CRBA evaluation, or the dense linear
    /// solve fails.
    pub fn forward_dynamics_dense_at_state(
        &self,
        state: &MultibodyState,
        generalized_forces: &[f64],
        root_parent_acceleration: SpatialMotion,
        external_forces_body: &[SpatialForce],
    ) -> Result<Vec<f64>, MultibodyError> {
        self.validate_generalized_vector("generalized forces", generalized_forces)?;
        let h = self.joint_space_inertia_crba_at_state(state)?;
        let zero_qdd = vec![0.0; self.n_qd];
        let bias = self.inverse_dynamics_rnea_at_state(
            state,
            &zero_qdd,
            root_parent_acceleration,
            external_forces_body,
        )?;
        let rhs: Vec<f64> = generalized_forces
            .iter()
            .zip(bias.iter())
            .map(|(force, bias_force)| force - bias_force)
            .collect();
        solve_dense_linear_system(h.dimension(), h.values_row_major(), &rhs)
    }

    /// Articulated-body forward dynamics at a generalized state.
    ///
    /// This is the `O(n)` Featherstone ABA path over the same q-dependent
    /// transforms, velocity-bias terms, root parent acceleration seed, and
    /// per-body external spatial forces as [`Self::inverse_dynamics_rnea_at_state`].
    /// Multi-DoF joints, including the free-flyer root, are solved with a
    /// fixed-order symmetric LDLT factorization of each local joint-space
    /// articulated inertia block.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] when state validation, generalized-force
    /// validation, external-force validation, joint-transform evaluation, or a
    /// local articulated inertia factorization fails.
    pub fn forward_dynamics_aba_at_state(
        &self,
        state: &MultibodyState,
        generalized_forces: &[f64],
        root_parent_acceleration: SpatialMotion,
        external_forces_body: &[SpatialForce],
    ) -> Result<Vec<f64>, MultibodyError> {
        self.validate_generalized_vector("generalized forces", generalized_forces)?;
        let transforms = self.body_transforms_parent_to_child_at_state(state)?;
        if !root_parent_acceleration.is_finite() {
            return Err(MultibodyError::NonFinite {
                reason: "root parent acceleration contains non-finite components",
            });
        }
        if external_forces_body.len() != self.bodies.len() {
            return Err(MultibodyError::GeneralizedVectorDimensionMismatch {
                vector: "external forces",
                expected: self.bodies.len(),
                actual: external_forces_body.len(),
            });
        }
        if !external_forces_body.iter().all(SpatialForce::is_finite) {
            return Err(MultibodyError::NonFinite {
                reason: "external forces contain non-finite components",
            });
        }

        let subspaces: Vec<Vec<SpatialMotion>> = self
            .bodies
            .iter()
            .map(|body| body.joint.motion_subspace())
            .collect();
        let mut velocities = vec![SpatialVector::zeros(); self.bodies.len()];
        let mut coriolis = vec![SpatialVector::zeros(); self.bodies.len()];
        let mut articulated_inertias: Vec<SpatialMatrix> = self
            .bodies
            .iter()
            .map(|body| *body.inertia.matrix())
            .collect();
        let mut articulated_biases = vec![SpatialVector::zeros(); self.bodies.len()];
        let mut joint_u_columns: Vec<Vec<SpatialVector>> = vec![Vec::new(); self.bodies.len()];
        let mut joint_d_matrices: Vec<Vec<f64>> = vec![Vec::new(); self.bodies.len()];
        let mut joint_u_vectors: Vec<Vec<f64>> = vec![Vec::new(); self.bodies.len()];

        for body_index in 0..self.bodies.len() {
            let body = &self.bodies[body_index];
            let x = transforms[body_index].motion_matrix_parent_to_child();
            let parent_velocity = if let Some(parent) = body.parent {
                x * velocities[parent.index()]
            } else {
                SpatialVector::zeros()
            };
            let joint_velocity = joint_motion(&subspaces[body_index], &state.qd[body.qd_offset..]);
            let velocity = parent_velocity + joint_velocity;
            velocities[body_index] = velocity;
            coriolis[body_index] = SpatialMotion::from_vector(velocity).crossm() * joint_velocity;
            articulated_biases[body_index] = SpatialMotion::from_vector(velocity).crossf()
                * (body.inertia.matrix() * velocity)
                - external_forces_body[body_index].vector();
        }

        for body_index in (0..self.bodies.len()).rev() {
            let body = &self.bodies[body_index];
            let subspace = &subspaces[body_index];
            let joint_dof = subspace.len();
            let mut u_columns = Vec::with_capacity(joint_dof);
            for motion in subspace {
                u_columns.push(articulated_inertias[body_index] * motion.vector());
            }
            let d_matrix = joint_space_block(subspace, &u_columns);
            let mut u_vector = Vec::with_capacity(joint_dof);
            for (local, motion) in subspace.iter().enumerate() {
                u_vector.push(
                    generalized_forces[body.qd_offset + local]
                        - motion.vector().dot(&articulated_biases[body_index]),
                );
            }

            let d_inv_u = solve_symmetric_ldlt(joint_dof, &d_matrix, &u_vector)?;
            let d_inverse = invert_symmetric_ldlt(joint_dof, &d_matrix)?;
            let projected_inertia = spatial_projected_inertia(&u_columns, &d_inverse);
            let reduced_inertia = articulated_inertias[body_index] - projected_inertia;
            let reduced_bias = articulated_biases[body_index]
                + reduced_inertia * coriolis[body_index]
                + spatial_weighted_sum(&u_columns, &d_inv_u);

            joint_u_columns[body_index] = u_columns;
            joint_d_matrices[body_index] = d_matrix;
            joint_u_vectors[body_index] = u_vector;
            articulated_inertias[body_index] = reduced_inertia;
            articulated_biases[body_index] = reduced_bias;

            if let Some(parent) = body.parent {
                let x = transforms[body_index].motion_matrix_parent_to_child();
                articulated_inertias[parent.index()] += x.transpose() * reduced_inertia * x;
                articulated_biases[parent.index()] += x.transpose() * reduced_bias;
            }
        }

        let mut accelerations = vec![SpatialVector::zeros(); self.bodies.len()];
        let mut qdd = vec![0.0; self.n_qd];
        for body_index in 0..self.bodies.len() {
            let body = &self.bodies[body_index];
            let x = transforms[body_index].motion_matrix_parent_to_child();
            let parent_acceleration = if let Some(parent) = body.parent {
                x * accelerations[parent.index()]
            } else {
                x * root_parent_acceleration.vector()
            };
            let mut acceleration = parent_acceleration + coriolis[body_index];
            let joint_dof = subspaces[body_index].len();
            if joint_dof > 0 {
                let mut rhs = joint_u_vectors[body_index].clone();
                for (local, u_column) in joint_u_columns[body_index].iter().enumerate() {
                    rhs[local] -= u_column.dot(&acceleration);
                }
                let joint_qdd =
                    solve_symmetric_ldlt(joint_dof, &joint_d_matrices[body_index], &rhs)?;
                for (local, value) in joint_qdd.iter().enumerate() {
                    qdd[body.qd_offset + local] = *value;
                    acceleration += subspaces[body_index][local].vector() * *value;
                }
            }
            accelerations[body_index] = acceleration;
        }
        self.validate_generalized_vector("ABA generalized accelerations", &qdd)?;
        Ok(qdd)
    }

    /// Zero-velocity Recursive Newton-Euler inverse dynamics at a generalized
    /// state.
    ///
    /// This uses q-dependent joint transforms, but intentionally omits velocity
    /// bias, base acceleration, and external forces. Given `qdd`, it computes
    /// `tau = H(q)*qdd` for the state-dependent CRBA substrate.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] when state validation, qdd validation, or a
    /// joint transform fails.
    pub fn inverse_dynamics_rnea_at_state_no_bias(
        &self,
        state: &MultibodyState,
        qdd: &[f64],
    ) -> Result<Vec<f64>, MultibodyError> {
        let transforms = self.body_transforms_parent_to_child_at_state(state)?;
        let zero_qd = vec![0.0; self.n_qd];
        let zero_external_forces = vec![SpatialForce::zero(); self.bodies.len()];
        self.inverse_dynamics_rnea_with_terms(
            &zero_qd,
            qdd,
            &transforms,
            SpatialMotion::zero(),
            &zero_external_forces,
        )
    }

    /// Zero-velocity Recursive Newton-Euler inverse dynamics over fixed tree
    /// transforms.
    ///
    /// Given generalized acceleration `qdd`, this computes `tau = H*qdd` for
    /// the same fixed-transform, q-independent-subspace assumptions as
    /// [`Self::joint_space_inertia_crba_fixed_transforms`]. It is intentionally
    /// a substrate for the CRBA-column self-consistency gate, not yet the full
    /// WP-01.1 RNEA with velocity bias, gravity, external forces, or
    /// q-dependent joint transforms.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if `qdd` has the wrong length or contains a
    /// non-finite value.
    pub fn inverse_dynamics_rnea_fixed_transforms(
        &self,
        qdd: &[f64],
    ) -> Result<Vec<f64>, MultibodyError> {
        let transforms: Vec<PluckerTransform> =
            self.bodies.iter().map(|body| body.parent_to_body).collect();
        let zero_qd = vec![0.0; self.n_qd];
        let zero_external_forces = vec![SpatialForce::zero(); self.bodies.len()];
        self.inverse_dynamics_rnea_with_terms(
            &zero_qd,
            qdd,
            &transforms,
            SpatialMotion::zero(),
            &zero_external_forces,
        )
    }

    fn inverse_dynamics_rnea_with_terms(
        &self,
        qd: &[f64],
        qdd: &[f64],
        transforms_parent_to_child: &[PluckerTransform],
        root_parent_acceleration: SpatialMotion,
        external_forces_body: &[SpatialForce],
    ) -> Result<Vec<f64>, MultibodyError> {
        if qd.len() != self.n_qd {
            return Err(MultibodyError::GeneralizedVectorDimensionMismatch {
                vector: "qd",
                expected: self.n_qd,
                actual: qd.len(),
            });
        }
        if qdd.len() != self.n_qd {
            return Err(MultibodyError::GeneralizedVectorDimensionMismatch {
                vector: "qdd",
                expected: self.n_qd,
                actual: qdd.len(),
            });
        }
        if transforms_parent_to_child.len() != self.bodies.len() {
            return Err(MultibodyError::GeneralizedVectorDimensionMismatch {
                vector: "body transforms",
                expected: self.bodies.len(),
                actual: transforms_parent_to_child.len(),
            });
        }
        if external_forces_body.len() != self.bodies.len() {
            return Err(MultibodyError::GeneralizedVectorDimensionMismatch {
                vector: "external forces",
                expected: self.bodies.len(),
                actual: external_forces_body.len(),
            });
        }
        if !qd.iter().all(|v| v.is_finite()) {
            return Err(MultibodyError::NonFinite {
                reason: "generalized velocity contains non-finite components",
            });
        }
        if !qdd.iter().all(|v| v.is_finite()) {
            return Err(MultibodyError::NonFinite {
                reason: "generalized acceleration contains non-finite components",
            });
        }
        if !root_parent_acceleration.is_finite() {
            return Err(MultibodyError::NonFinite {
                reason: "root parent acceleration contains non-finite components",
            });
        }
        if !external_forces_body.iter().all(SpatialForce::is_finite) {
            return Err(MultibodyError::NonFinite {
                reason: "external forces contain non-finite components",
            });
        }

        let subspaces: Vec<Vec<SpatialMotion>> = self
            .bodies
            .iter()
            .map(|body| body.joint.motion_subspace())
            .collect();
        let mut velocities = vec![SpatialVector::zeros(); self.bodies.len()];
        let mut accelerations = vec![SpatialVector::zeros(); self.bodies.len()];
        let mut forces = vec![SpatialVector::zeros(); self.bodies.len()];

        for body_index in 0..self.bodies.len() {
            let body = &self.bodies[body_index];
            let x = transforms_parent_to_child[body_index].motion_matrix_parent_to_child();
            let parent_velocity = if let Some(parent) = body.parent {
                x * velocities[parent.index()]
            } else {
                SpatialVector::zeros()
            };
            let mut joint_velocity = SpatialVector::zeros();
            let mut joint_acceleration = SpatialVector::zeros();
            for (local, motion) in subspaces[body_index].iter().enumerate() {
                let generalized_index = body.qd_offset + local;
                joint_velocity += motion.vector() * qd[generalized_index];
                joint_acceleration += motion.vector() * qdd[generalized_index];
            }

            let velocity = parent_velocity + joint_velocity;
            let parent_acceleration = if let Some(parent) = body.parent {
                x * accelerations[parent.index()]
            } else {
                x * root_parent_acceleration.vector()
            };
            let acceleration = parent_acceleration
                + joint_acceleration
                + SpatialMotion::from_vector(velocity).crossm() * joint_velocity;
            let inertia_times_velocity = body.inertia.matrix() * velocity;
            let bias_force = SpatialMotion::from_vector(velocity).crossf() * inertia_times_velocity;

            velocities[body_index] = velocity;
            accelerations[body_index] = acceleration;
            forces[body_index] = body.inertia.matrix() * acceleration + bias_force
                - external_forces_body[body_index].vector();
        }

        let mut tau = vec![0.0; self.n_qd];
        for body_index in (0..self.bodies.len()).rev() {
            let body = &self.bodies[body_index];
            for (local, motion) in subspaces[body_index].iter().enumerate() {
                tau[body.qd_offset + local] = motion.vector().dot(&forces[body_index]);
            }
            if let Some(parent) = body.parent {
                let parent_force = transforms_parent_to_child[body_index]
                    .motion_matrix_parent_to_child()
                    .transpose()
                    * forces[body_index];
                forces[parent.index()] += parent_force;
            }
        }
        Ok(tau)
    }

    fn validate_generalized_vector(
        &self,
        vector: &'static str,
        values: &[f64],
    ) -> Result<(), MultibodyError> {
        if values.len() != self.n_qd {
            return Err(MultibodyError::GeneralizedVectorDimensionMismatch {
                vector,
                expected: self.n_qd,
                actual: values.len(),
            });
        }
        if !values.iter().all(|v| v.is_finite()) {
            return Err(MultibodyError::NonFinite { reason: vector });
        }
        Ok(())
    }

    fn body_transforms_root_parent_to_child(
        &self,
        transforms_parent_to_child: &[PluckerTransform],
    ) -> Result<Vec<PluckerTransform>, MultibodyError> {
        let mut root_transforms = vec![PluckerTransform::identity(); self.bodies.len()];
        for body_index in 0..self.bodies.len() {
            root_transforms[body_index] = if let Some(parent) = self.bodies[body_index].parent {
                root_transforms[parent.index()].then(&transforms_parent_to_child[body_index])?
            } else {
                transforms_parent_to_child[body_index]
            };
        }
        Ok(root_transforms)
    }

    fn body_spatial_velocities_with_transforms(
        &self,
        state: &MultibodyState,
        transforms_parent_to_child: &[PluckerTransform],
    ) -> Vec<SpatialVector> {
        let subspaces: Vec<Vec<SpatialMotion>> = self
            .bodies
            .iter()
            .map(|body| body.joint.motion_subspace())
            .collect();
        let mut velocities = vec![SpatialVector::zeros(); self.bodies.len()];
        for body_index in 0..self.bodies.len() {
            let body = &self.bodies[body_index];
            let x = transforms_parent_to_child[body_index].motion_matrix_parent_to_child();
            let parent_velocity = if let Some(parent) = body.parent {
                x * velocities[parent.index()]
            } else {
                SpatialVector::zeros()
            };
            velocities[body_index] =
                parent_velocity + joint_motion(&subspaces[body_index], &state.qd[body.qd_offset..]);
        }
        velocities
    }

    fn is_descendant_or_self(&self, body_index: usize, ancestor_index: usize) -> bool {
        let mut cursor = Some(BodyIndex::new(body_index));
        while let Some(index) = cursor {
            if index.index() == ancestor_index {
                return true;
            }
            cursor = self.bodies[index.index()].parent;
        }
        false
    }

    fn body_transforms_parent_to_child_at_state(
        &self,
        state: &MultibodyState,
    ) -> Result<Vec<PluckerTransform>, MultibodyError> {
        self.validate_state(state)?;
        self.bodies
            .iter()
            .map(|body| {
                let q_start = body.q_offset;
                let q_end = q_start + body.joint.n_q();
                let joint_transform = body.joint.joint_transform(&state.q[q_start..q_end])?;
                body.parent_to_body.then(&joint_transform)
            })
            .collect()
    }

    fn quaternion_coordinate_offsets(&self) -> Vec<usize> {
        self.bodies
            .iter()
            .filter_map(|body| match body.joint {
                Joint::FreeFlyer | Joint::Spherical => Some(body.q_offset),
                Joint::Revolute { .. } | Joint::Prismatic { .. } | Joint::Welded { .. } => None,
            })
            .collect()
    }
}

/// Generalized-coordinate multibody state.
#[derive(Clone, Debug, PartialEq)]
pub struct MultibodyState {
    /// Monotonic simulation time.
    pub time: SimTime,
    /// Generalized coordinates.
    pub q: Vec<f64>,
    /// Generalized velocities.
    pub qd: Vec<f64>,
}

impl MultibodyState {
    /// Construct from raw generalized-coordinate vectors.
    #[must_use]
    pub fn new(time: SimTime, q: Vec<f64>, qd: Vec<f64>) -> Self {
        Self { time, q, qd }
    }

    /// Returns `true` if time, coordinates, and velocities are finite.
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.time.as_seconds().is_finite()
            && self.q.iter().all(|v| v.is_finite())
            && self.qd.iter().all(|v| v.is_finite())
    }
}

/// Topology-aware simulator adapter for a multibody generalized state.
///
/// [`MultibodyState`] stores only raw generalized coordinates and velocities.
/// The simulator integration trait also needs a state-local `project()` hook,
/// so this adapter carries the tree-derived quaternion offsets needed to
/// renormalize free-flyer and spherical coordinate slices after each
/// integration step.
#[derive(Clone, Debug, PartialEq)]
pub struct MultibodySimState {
    state: MultibodyState,
    quaternion_offsets: Vec<usize>,
}

impl MultibodySimState {
    /// Construct from a raw state plus validated quaternion offsets.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError`] if the state is non-finite, has no scalar
    /// components, or any quaternion offset is outside the coordinate vector,
    /// duplicated, non-finite, or zero-norm.
    pub fn new(
        state: MultibodyState,
        quaternion_offsets: Vec<usize>,
    ) -> Result<Self, MultibodyError> {
        validate_sim_state_shape(&state, &quaternion_offsets)?;
        Ok(Self {
            state,
            quaternion_offsets,
        })
    }

    /// Underlying raw generalized-coordinate state.
    #[must_use]
    pub const fn state(&self) -> &MultibodyState {
        &self.state
    }

    /// Consume the adapter and return the raw generalized-coordinate state.
    #[must_use]
    pub fn into_state(self) -> MultibodyState {
        self.state
    }

    /// Generalized coordinates.
    #[must_use]
    pub fn q(&self) -> &[f64] {
        &self.state.q
    }

    /// Generalized velocities.
    #[must_use]
    pub fn qd(&self) -> &[f64] {
        &self.state.qd
    }

    /// Quaternion coordinate offsets carried for integration projection.
    #[must_use]
    pub fn quaternion_offsets(&self) -> &[usize] {
        &self.quaternion_offsets
    }
}

impl VehicleState for MultibodySimState {
    fn time(&self) -> SimTime {
        self.state.time
    }

    fn is_finite(&self) -> bool {
        self.state.is_finite()
    }

    fn with_time(mut self, t: SimTime) -> Self {
        self.state.time = t;
        self
    }
}

impl Integratable for MultibodySimState {
    type Derivative = MultibodyDerivative;

    fn is_valid_for_integration(&self) -> bool {
        validate_sim_state_shape(&self.state, &self.quaternion_offsets).is_ok()
    }

    fn advance_by(&self, h_seconds: f64, derivative: &Self::Derivative) -> Self {
        assert!(
            h_seconds.is_finite(),
            "multibody integration step must be finite"
        );
        assert_eq!(
            self.state.q.len(),
            derivative.q_dot.len(),
            "multibody q and q_dot dimensions must match for state advance"
        );
        assert_eq!(
            self.state.qd.len(),
            derivative.qd_dot.len(),
            "multibody qd and qd_dot dimensions must match for state advance"
        );
        let mut q = Vec::with_capacity(self.state.q.len());
        for index in 0..self.state.q.len() {
            q.push(self.state.q[index] + h_seconds * derivative.q_dot[index]);
        }
        let mut qd = Vec::with_capacity(self.state.qd.len());
        for index in 0..self.state.qd.len() {
            qd.push(self.state.qd[index] + h_seconds * derivative.qd_dot[index]);
        }
        Self {
            state: MultibodyState::new(
                SimTime::from_seconds(self.state.time.as_seconds() + h_seconds),
                q,
                qd,
            ),
            quaternion_offsets: self.quaternion_offsets.clone(),
        }
    }

    fn project(&mut self) {
        project_quaternion_offsets(&mut self.state.q, &self.quaternion_offsets);
    }

    fn scalar_state_size(&self) -> f64 {
        let mut sum = 0.0;
        for value in &self.state.q {
            sum += value * value;
        }
        for value in &self.state.qd {
            sum += value * value;
        }
        <f64 as nalgebra::ComplexField>::sqrt(sum)
    }

    fn weighted_error_norm(
        &self,
        prev_state: &Self,
        error_deriv: &Self::Derivative,
        h: f64,
        atol: f64,
        rtol: f64,
    ) -> f64 {
        assert_eq!(
            self.state.q.len(),
            prev_state.state.q.len(),
            "multibody weighted-error q dimensions must match"
        );
        assert_eq!(
            self.state.qd.len(),
            prev_state.state.qd.len(),
            "multibody weighted-error qd dimensions must match"
        );
        assert_eq!(
            self.state.q.len(),
            error_deriv.q_dot.len(),
            "multibody weighted-error q/q_dot dimensions must match"
        );
        assert_eq!(
            self.state.qd.len(),
            error_deriv.qd_dot.len(),
            "multibody weighted-error qd/qd_dot dimensions must match"
        );
        let dimension = self.state.q.len() + self.state.qd.len();
        if dimension == 0 {
            return f64::INFINITY;
        }
        let mut sum = 0.0;
        for index in 0..self.state.q.len() {
            sum += weighted_error_term(
                prev_state.state.q[index],
                self.state.q[index],
                error_deriv.q_dot[index],
                h,
                atol,
                rtol,
            );
        }
        for index in 0..self.state.qd.len() {
            sum += weighted_error_term(
                prev_state.state.qd[index],
                self.state.qd[index],
                error_deriv.qd_dot[index],
                h,
                atol,
                rtol,
            );
        }
        <f64 as nalgebra::ComplexField>::sqrt(sum / dimension as f64)
    }
}

/// Errors emitted by the multibody substrate.
#[derive(Debug, Error, PartialEq)]
pub enum MultibodyError {
    /// Invalid tree topology.
    #[error("invalid multibody topology: {reason}")]
    InvalidTopology {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// Invalid numeric parameter.
    #[error("invalid multibody parameter: {reason}")]
    InvalidParameter {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// Non-finite input or result.
    #[error("multibody value is non-finite: {reason}")]
    NonFinite {
        /// Short human-readable reason.
        reason: &'static str,
    },
    /// Generalized-coordinate dimension mismatch.
    #[error(
        "multibody state dimension mismatch: expected q={expected_q}, qd={expected_qd}; got q={actual_q}, qd={actual_qd}"
    )]
    DimensionMismatch {
        /// Expected generalized-coordinate length.
        expected_q: usize,
        /// Actual generalized-coordinate length.
        actual_q: usize,
        /// Expected generalized-velocity length.
        expected_qd: usize,
        /// Actual generalized-velocity length.
        actual_qd: usize,
    },
    /// Generalized vector length mismatch.
    #[error("multibody vector dimension mismatch for {vector}: expected {expected}, got {actual}")]
    GeneralizedVectorDimensionMismatch {
        /// Vector label.
        vector: &'static str,
        /// Expected length.
        expected: usize,
        /// Actual length.
        actual: usize,
    },
    /// Body index outside the tree.
    #[error("invalid multibody body index {index}; body count is {body_count}")]
    InvalidBodyIndex {
        /// Requested body index.
        index: usize,
        /// Number of bodies in the tree.
        body_count: usize,
    },
    /// Dense linear solve failed because the system is singular or ill
    /// conditioned at the selected pivot.
    #[error("singular multibody linear system at pivot {pivot}")]
    SingularSystem {
        /// Zero-based pivot index where factorization failed.
        pivot: usize,
    },
    /// Lower state-layer mass-property validation failed.
    #[error(transparent)]
    State(#[from] openbmp_state::StateError),
}

fn skew(v: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

fn unit_axis(axis: Vector3<f64>, reason: &'static str) -> Result<Vector3<f64>, MultibodyError> {
    if !axis.iter().all(|v| v.is_finite()) {
        return Err(MultibodyError::InvalidParameter { reason });
    }
    let norm = axis.norm();
    if norm == 0.0 {
        return Err(MultibodyError::InvalidParameter { reason });
    }
    Ok(axis / norm)
}

fn rotation_about_unit_axis(axis: Vector3<f64>, angle_rad: f64) -> Matrix3<f64> {
    let (sin_angle, cos_angle) = <f64 as nalgebra::ComplexField>::sin_cos(angle_rad);
    let one_minus_cos = 1.0 - cos_angle;
    let x = axis.x;
    let y = axis.y;
    let z = axis.z;
    Matrix3::new(
        cos_angle + x * x * one_minus_cos,
        x * y * one_minus_cos - z * sin_angle,
        x * z * one_minus_cos + y * sin_angle,
        y * x * one_minus_cos + z * sin_angle,
        cos_angle + y * y * one_minus_cos,
        y * z * one_minus_cos - x * sin_angle,
        z * x * one_minus_cos - y * sin_angle,
        z * y * one_minus_cos + x * sin_angle,
        cos_angle + z * z * one_minus_cos,
    )
}

fn rotation_parent_from_child_from_quaternion(q: &[f64]) -> Result<Matrix3<f64>, MultibodyError> {
    if q.len() != 4 {
        return Err(MultibodyError::GeneralizedVectorDimensionMismatch {
            vector: "quaternion",
            expected: 4,
            actual: q.len(),
        });
    }
    if !q.iter().all(|v| v.is_finite()) {
        return Err(MultibodyError::NonFinite {
            reason: "quaternion contains non-finite components",
        });
    }
    let norm2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if norm2 == 0.0 {
        return Err(MultibodyError::InvalidParameter {
            reason: "quaternion norm must be non-zero",
        });
    }
    let inv_norm = 1.0 / <f64 as nalgebra::ComplexField>::sqrt(norm2);
    let w = q[0] * inv_norm;
    let x = q[1] * inv_norm;
    let y = q[2] * inv_norm;
    let z = q[3] * inv_norm;
    Ok(Matrix3::new(
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y - w * z),
        2.0 * (x * z + w * y),
        2.0 * (x * y + w * z),
        1.0 - 2.0 * (x * x + z * z),
        2.0 * (y * z - w * x),
        2.0 * (x * z - w * y),
        2.0 * (y * z + w * x),
        1.0 - 2.0 * (x * x + y * y),
    ))
}

fn quaternion_derivative_from_body_rate(
    q: &[f64],
    omega_body_rad_s: Vector3<f64>,
) -> Result<[f64; 4], MultibodyError> {
    if q.len() != 4 {
        return Err(MultibodyError::GeneralizedVectorDimensionMismatch {
            vector: "quaternion",
            expected: 4,
            actual: q.len(),
        });
    }
    if !q.iter().all(|value| value.is_finite())
        || !omega_body_rad_s.iter().all(|value| value.is_finite())
    {
        return Err(MultibodyError::NonFinite {
            reason: "quaternion kinematic inputs contain non-finite components",
        });
    }
    let norm2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if norm2 == 0.0 {
        return Err(MultibodyError::InvalidParameter {
            reason: "quaternion norm must be non-zero",
        });
    }
    let q_dot = NalgebraQuaternion::new(q[0], q[1], q[2], q[3])
        * NalgebraQuaternion::new(
            0.0,
            omega_body_rad_s.x,
            omega_body_rad_s.y,
            omega_body_rad_s.z,
        )
        * 0.5;
    Ok([q_dot.w, q_dot.i, q_dot.j, q_dot.k])
}

fn normalize_quaternion_slice(q: &mut [f64]) -> Result<(), MultibodyError> {
    if q.len() != 4 {
        return Err(MultibodyError::GeneralizedVectorDimensionMismatch {
            vector: "quaternion",
            expected: 4,
            actual: q.len(),
        });
    }
    if !q.iter().all(|v| v.is_finite()) {
        return Err(MultibodyError::NonFinite {
            reason: "quaternion contains non-finite components",
        });
    }
    let norm2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if norm2 == 0.0 {
        return Err(MultibodyError::InvalidParameter {
            reason: "quaternion norm must be non-zero",
        });
    }
    let inv_norm = 1.0 / <f64 as nalgebra::ComplexField>::sqrt(norm2);
    for value in q.iter_mut() {
        *value *= inv_norm;
    }
    Ok(())
}

fn validate_sim_state_shape(
    state: &MultibodyState,
    quaternion_offsets: &[usize],
) -> Result<(), MultibodyError> {
    if state.q.is_empty() && state.qd.is_empty() {
        return Err(MultibodyError::InvalidParameter {
            reason: "multibody simulator state must contain at least one scalar component",
        });
    }
    if !state.is_finite() {
        return Err(MultibodyError::NonFinite {
            reason: "multibody simulator state contains non-finite components",
        });
    }
    for (index, offset) in quaternion_offsets.iter().enumerate() {
        for previous in &quaternion_offsets[0..index] {
            if previous == offset {
                return Err(MultibodyError::InvalidParameter {
                    reason: "multibody simulator quaternion offsets must be unique",
                });
            }
        }
        if offset.saturating_add(4) > state.q.len() {
            return Err(MultibodyError::GeneralizedVectorDimensionMismatch {
                vector: "quaternion offset",
                expected: state.q.len(),
                actual: offset.saturating_add(4),
            });
        }
        let q = &state.q[*offset..*offset + 4];
        if !q.iter().all(|value| value.is_finite()) {
            return Err(MultibodyError::NonFinite {
                reason: "quaternion contains non-finite components",
            });
        }
        let norm2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
        if norm2 == 0.0 {
            return Err(MultibodyError::InvalidParameter {
                reason: "quaternion norm must be non-zero",
            });
        }
    }
    Ok(())
}

fn project_quaternion_offsets(q: &mut [f64], quaternion_offsets: &[usize]) {
    for offset in quaternion_offsets {
        if offset.saturating_add(4) > q.len() {
            continue;
        }
        let slice = &mut q[*offset..*offset + 4];
        let norm2 =
            slice[0] * slice[0] + slice[1] * slice[1] + slice[2] * slice[2] + slice[3] * slice[3];
        if norm2 > 0.0 && norm2.is_finite() {
            let inv_norm = 1.0 / <f64 as nalgebra::ComplexField>::sqrt(norm2);
            for value in slice {
                *value *= inv_norm;
            }
        }
    }
}

fn weighted_error_term(
    previous: f64,
    current: f64,
    error_derivative: f64,
    h_seconds: f64,
    atol: f64,
    rtol: f64,
) -> f64 {
    let scale = atol + rtol * abs_finite(previous).max(abs_finite(current));
    let scaled = h_seconds * error_derivative / scale;
    scaled * scaled
}

fn joint_motion(subspace: &[SpatialMotion], qd_slice: &[f64]) -> SpatialVector {
    let mut out = SpatialVector::zeros();
    for (local, motion) in subspace.iter().enumerate() {
        out += motion.vector() * qd_slice[local];
    }
    out
}

fn joint_space_block(subspace: &[SpatialMotion], u_columns: &[SpatialVector]) -> Vec<f64> {
    let joint_dof = subspace.len();
    let mut out = vec![0.0; joint_dof * joint_dof];
    for row in 0..joint_dof {
        for col in 0..joint_dof {
            out[row * joint_dof + col] = subspace[row].vector().dot(&u_columns[col]);
        }
    }
    out
}

fn spatial_weighted_sum(columns: &[SpatialVector], weights: &[f64]) -> SpatialVector {
    let mut out = SpatialVector::zeros();
    for (column, weight) in columns.iter().zip(weights.iter()) {
        out += column * *weight;
    }
    out
}

fn spatial_projected_inertia(u_columns: &[SpatialVector], d_inverse: &[f64]) -> SpatialMatrix {
    let joint_dof = u_columns.len();
    let mut out = SpatialMatrix::zeros();
    for row in 0..6 {
        for col in 0..6 {
            let mut value = 0.0;
            for left in 0..joint_dof {
                for right in 0..joint_dof {
                    value += u_columns[left][row]
                        * d_inverse[left * joint_dof + right]
                        * u_columns[right][col];
                }
            }
            out[(row, col)] = value;
        }
    }
    out
}

fn invert_symmetric_ldlt(
    dimension: usize,
    matrix_row_major: &[f64],
) -> Result<Vec<f64>, MultibodyError> {
    let mut inverse = vec![0.0; dimension * dimension];
    for col in 0..dimension {
        let mut unit = vec![0.0; dimension];
        unit[col] = 1.0;
        let solved = solve_symmetric_ldlt(dimension, matrix_row_major, &unit)?;
        for row in 0..dimension {
            inverse[row * dimension + col] = solved[row];
        }
    }
    Ok(inverse)
}

fn solve_symmetric_ldlt(
    dimension: usize,
    matrix_row_major: &[f64],
    rhs: &[f64],
) -> Result<Vec<f64>, MultibodyError> {
    if matrix_row_major.len() != dimension * dimension {
        return Err(MultibodyError::GeneralizedVectorDimensionMismatch {
            vector: "symmetric LDLT matrix",
            expected: dimension * dimension,
            actual: matrix_row_major.len(),
        });
    }
    if rhs.len() != dimension {
        return Err(MultibodyError::GeneralizedVectorDimensionMismatch {
            vector: "symmetric LDLT rhs",
            expected: dimension,
            actual: rhs.len(),
        });
    }
    if !matrix_row_major.iter().all(|v| v.is_finite()) || !rhs.iter().all(|v| v.is_finite()) {
        return Err(MultibodyError::NonFinite {
            reason: "symmetric LDLT system contains non-finite components",
        });
    }

    let mut lower = vec![0.0; dimension * dimension];
    let mut diagonal = vec![0.0; dimension];
    for row in 0..dimension {
        for col in 0..row {
            let mut value = matrix_row_major[row * dimension + col];
            for k in 0..col {
                value -= lower[row * dimension + k] * diagonal[k] * lower[col * dimension + k];
            }
            if diagonal[col] <= 1.0e-12 {
                return Err(MultibodyError::SingularSystem { pivot: col });
            }
            lower[row * dimension + col] = value / diagonal[col];
        }
        let mut diag = matrix_row_major[row * dimension + row];
        for k in 0..row {
            let l = lower[row * dimension + k];
            diag -= l * l * diagonal[k];
        }
        if diag <= 1.0e-12 {
            return Err(MultibodyError::SingularSystem { pivot: row });
        }
        lower[row * dimension + row] = 1.0;
        diagonal[row] = diag;
    }

    let mut y = vec![0.0; dimension];
    for row in 0..dimension {
        let mut value = rhs[row];
        for col in 0..row {
            value -= lower[row * dimension + col] * y[col];
        }
        y[row] = value;
    }
    let mut z = vec![0.0; dimension];
    for row in 0..dimension {
        z[row] = y[row] / diagonal[row];
    }
    let mut x = vec![0.0; dimension];
    for row in (0..dimension).rev() {
        let mut value = z[row];
        for col in (row + 1)..dimension {
            value -= lower[col * dimension + row] * x[col];
        }
        x[row] = value;
    }
    if !x.iter().all(|v| v.is_finite()) {
        return Err(MultibodyError::NonFinite {
            reason: "symmetric LDLT solve produced non-finite components",
        });
    }
    Ok(x)
}

fn solve_dense_linear_system(
    dimension: usize,
    matrix_row_major: &[f64],
    rhs: &[f64],
) -> Result<Vec<f64>, MultibodyError> {
    if matrix_row_major.len() != dimension * dimension {
        return Err(MultibodyError::GeneralizedVectorDimensionMismatch {
            vector: "dense system matrix",
            expected: dimension * dimension,
            actual: matrix_row_major.len(),
        });
    }
    if rhs.len() != dimension {
        return Err(MultibodyError::GeneralizedVectorDimensionMismatch {
            vector: "dense system rhs",
            expected: dimension,
            actual: rhs.len(),
        });
    }
    if !matrix_row_major.iter().all(|v| v.is_finite()) || !rhs.iter().all(|v| v.is_finite()) {
        return Err(MultibodyError::NonFinite {
            reason: "dense linear system contains non-finite components",
        });
    }

    let mut a = matrix_row_major.to_vec();
    let mut b = rhs.to_vec();
    for pivot in 0..dimension {
        let mut pivot_row = pivot;
        let mut pivot_abs = abs_finite(a[pivot * dimension + pivot]);
        for row in (pivot + 1)..dimension {
            let candidate = abs_finite(a[row * dimension + pivot]);
            if candidate > pivot_abs {
                pivot_abs = candidate;
                pivot_row = row;
            }
        }
        if pivot_abs <= 1.0e-12 {
            return Err(MultibodyError::SingularSystem { pivot });
        }
        if pivot_row != pivot {
            for col in pivot..dimension {
                a.swap(pivot * dimension + col, pivot_row * dimension + col);
            }
            b.swap(pivot, pivot_row);
        }

        let pivot_value = a[pivot * dimension + pivot];
        for row in (pivot + 1)..dimension {
            let factor = a[row * dimension + pivot] / pivot_value;
            a[row * dimension + pivot] = 0.0;
            for col in (pivot + 1)..dimension {
                a[row * dimension + col] -= factor * a[pivot * dimension + col];
            }
            b[row] -= factor * b[pivot];
        }
    }

    let mut out = vec![0.0; dimension];
    for row in (0..dimension).rev() {
        let mut value = b[row];
        for col in (row + 1)..dimension {
            value -= a[row * dimension + col] * out[col];
        }
        let pivot_value = a[row * dimension + row];
        if abs_finite(pivot_value) <= 1.0e-12 {
            return Err(MultibodyError::SingularSystem { pivot: row });
        }
        out[row] = value / pivot_value;
    }
    if !out.iter().all(|v| v.is_finite()) {
        return Err(MultibodyError::NonFinite {
            reason: "dense linear solve produced non-finite components",
        });
    }
    Ok(out)
}

fn abs_finite(value: f64) -> f64 {
    if value < 0.0 { -value } else { value }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use nalgebra::UnitQuaternion;
    use openbmp_core::{Body, Position3};
    use openbmp_models::{RigidBodyDerivative, SimState};
    use serde::Deserialize;
    use uom::si::f64::Mass;
    use uom::si::mass::kilogram;

    #[derive(Debug, Deserialize)]
    struct AbaToleranceFixture {
        q: Vec<f64>,
        qd: Vec<f64>,
        generalized_forces: Vec<f64>,
        root_acceleration: SpatialMotionFixture,
        external_forces: Vec<SpatialForceFixture>,
        tolerances: AbaToleranceThresholds,
    }

    #[derive(Debug, Deserialize)]
    struct SpatialV2CompatibleOracleFixture {
        source: SpatialV2CompatibleOracleSource,
        q: Vec<f64>,
        qd: Vec<f64>,
        generalized_forces: Vec<f64>,
        root_acceleration: SpatialMotionFixture,
        external_forces: Vec<SpatialForceFixture>,
        expected_qdd: Vec<f64>,
        tolerances: SpatialV2CompatibleOracleThresholds,
    }

    #[derive(Debug, Deserialize)]
    struct SpatialV2CompatibleOracleSource {
        solver: String,
        export_format: String,
        independence: String,
        notes: String,
    }

    #[derive(Debug, Deserialize)]
    struct SpatialMotionFixture {
        angular: [f64; 3],
        linear: [f64; 3],
    }

    #[derive(Debug, Deserialize)]
    struct SpatialForceFixture {
        moment: [f64; 3],
        force: [f64; 3],
    }

    #[derive(Debug, Deserialize)]
    struct AbaToleranceThresholds {
        aba_dense_max_abs: f64,
        rnea_round_trip_max_abs: f64,
    }

    #[derive(Debug, Deserialize)]
    struct SpatialV2CompatibleOracleThresholds {
        qdd_max_abs: f64,
    }

    #[derive(Debug, Deserialize)]
    struct DoublePendulumEnergyMomentumFixture {
        link_lengths_m: [f64; 2],
        masses_kg: [f64; 2],
        q: [f64; 2],
        qd: [f64; 2],
        expected_kinetic_energy_j: f64,
        expected_revolute_generalized_momentum: [f64; 2],
        tolerances: DoublePendulumEnergyMomentumThresholds,
    }

    #[derive(Debug, Deserialize)]
    struct DoublePendulumEnergyMomentumThresholds {
        kinetic_energy_abs: f64,
        generalized_momentum_max_abs: f64,
    }

    fn load_aba_tolerance_fixture() -> AbaToleranceFixture {
        toml::from_str(include_str!(
            "../tests/expected/aba-chain-tolerance-v1.toml"
        ))
        .expect("ABA tolerance fixture parses")
    }

    fn load_spatial_v2_compatible_oracle_fixture() -> SpatialV2CompatibleOracleFixture {
        toml::from_str(include_str!(
            "../tests/expected/spatial-v2-compatible-aba-oracle-chain-v1.toml"
        ))
        .expect("Spatial_v2-compatible ABA oracle fixture parses")
    }

    fn load_double_pendulum_energy_momentum_fixture() -> DoublePendulumEnergyMomentumFixture {
        toml::from_str(include_str!(
            "../tests/expected/double-pendulum-energy-momentum-v1.toml"
        ))
        .expect("double-pendulum energy/momentum fixture parses")
    }

    fn motion_from_fixture(fixture: &SpatialMotionFixture) -> SpatialMotion {
        SpatialMotion::new(
            Vector3::from(fixture.angular),
            Vector3::from(fixture.linear),
        )
    }

    fn force_from_fixture(fixture: &SpatialForceFixture) -> SpatialForce {
        SpatialForce::new(Vector3::from(fixture.moment), Vector3::from(fixture.force))
    }

    fn assert_max_abs_diff(actual: &[f64], expected: &[f64], tolerance: f64, label: &str) {
        assert_eq!(
            actual.len(),
            expected.len(),
            "{label} vector length mismatch"
        );
        let mut max_abs = 0.0_f64;
        for (actual_value, expected_value) in actual.iter().zip(expected.iter()) {
            max_abs = max_abs.max((actual_value - expected_value).abs());
        }
        assert!(
            max_abs <= tolerance,
            "{label} max_abs={max_abs:.17e} tolerance={tolerance:.17e}"
        );
    }

    fn mass_properties() -> MassProperties {
        MassProperties::with_diagonal_inertia(
            Mass::new::<kilogram>(10.0),
            Position3::<Body>::new(0.2, -0.3, 0.4),
            3.0,
            4.0,
            5.0,
        )
    }

    fn inertia() -> SpatialInertia {
        SpatialInertia::from_mass_properties(&mass_properties()).unwrap()
    }

    fn point_mass_spatial_inertia(mass_kg: f64, com_body_m: Vector3<f64>) -> SpatialInertia {
        let com_cross = skew(com_body_m);
        let mut matrix = SpatialMatrix::zeros();
        matrix
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&(-mass_kg * com_cross * com_cross));
        matrix
            .fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&(mass_kg * com_cross));
        matrix
            .fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&(-mass_kg * com_cross));
        matrix
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(mass_kg * Matrix3::identity()));
        SpatialInertia::from_matrix(matrix).unwrap()
    }

    fn double_pendulum_fixture_tree(
        fixture: &DoublePendulumEnergyMomentumFixture,
    ) -> MultibodyTree {
        let root = TreeBodySpec {
            id: BodyId::new(1),
            parent: None,
            joint: Joint::FreeFlyer,
            inertia: SpatialInertia::from_matrix(SpatialMatrix::zeros()).unwrap(),
            parent_to_body: PluckerTransform::identity(),
        };
        let link_1 = TreeBodySpec {
            id: BodyId::new(2),
            parent: Some(BodyIndex::new(0)),
            joint: Joint::revolute(Vector3::new(0.0, 0.0, 1.0)).unwrap(),
            inertia: point_mass_spatial_inertia(
                fixture.masses_kg[0],
                Vector3::new(fixture.link_lengths_m[0], 0.0, 0.0),
            ),
            parent_to_body: PluckerTransform::identity(),
        };
        let link_2 = TreeBodySpec {
            id: BodyId::new(3),
            parent: Some(BodyIndex::new(1)),
            joint: Joint::revolute(Vector3::new(0.0, 0.0, 1.0)).unwrap(),
            inertia: point_mass_spatial_inertia(
                fixture.masses_kg[1],
                Vector3::new(fixture.link_lengths_m[1], 0.0, 0.0),
            ),
            parent_to_body: PluckerTransform::new(
                Matrix3::identity(),
                Vector3::new(fixture.link_lengths_m[0], 0.0, 0.0),
            )
            .unwrap(),
        };
        MultibodyTree::new(vec![root, link_1, link_2]).unwrap()
    }

    fn sample_tree() -> MultibodyTree {
        let root = TreeBodySpec {
            id: BodyId::new(1),
            parent: None,
            joint: Joint::FreeFlyer,
            inertia: inertia(),
            parent_to_body: PluckerTransform::identity(),
        };
        let hinge = TreeBodySpec {
            id: BodyId::new(2),
            parent: Some(BodyIndex::new(0)),
            joint: Joint::revolute(Vector3::new(0.0, 1.0, 0.0)).unwrap(),
            inertia: inertia(),
            parent_to_body: PluckerTransform::new(Matrix3::identity(), Vector3::new(1.0, 0.0, 0.0))
                .unwrap(),
        };
        let slider = TreeBodySpec {
            id: BodyId::new(3),
            parent: Some(BodyIndex::new(1)),
            joint: Joint::prismatic(Vector3::new(0.0, 0.0, 1.0)).unwrap(),
            inertia: inertia(),
            parent_to_body: PluckerTransform::new(Matrix3::identity(), Vector3::new(0.0, 0.5, 0.0))
                .unwrap(),
        };
        MultibodyTree::new(vec![root, hinge, slider]).unwrap()
    }

    #[test]
    fn spatial_motion_cross_matrix_matches_vector_formula() {
        let a = SpatialMotion::new(Vector3::new(1.0, -2.0, 3.0), Vector3::new(4.0, 5.0, -6.0));
        let b = SpatialMotion::new(Vector3::new(-0.5, 2.5, 1.5), Vector3::new(3.0, -1.0, 2.0));

        let out = a.crossm() * b.vector();
        let expected_angular = a.angular_rad_s().cross(&b.angular_rad_s());
        let expected_linear =
            a.linear_m_s().cross(&b.angular_rad_s()) + a.angular_rad_s().cross(&b.linear_m_s());

        for axis in 0..3 {
            assert_abs_diff_eq!(out[axis], expected_angular[axis], epsilon = 1.0e-15);
            assert_abs_diff_eq!(out[axis + 3], expected_linear[axis], epsilon = 1.0e-15);
        }
    }

    #[test]
    fn spatial_force_cross_matrix_is_motion_dual() {
        let motion = SpatialMotion::new(Vector3::new(0.2, -0.7, 1.1), Vector3::new(2.0, -3.0, 4.0));

        assert_eq!(motion.crossf(), -motion.crossm().transpose());
    }

    #[test]
    fn spatial_inertia_from_mass_properties_uses_parallel_axis_blocks() {
        let props = mass_properties();
        let spatial = SpatialInertia::from_mass_properties(&props).unwrap();
        let c = props.center_of_mass_body.vector;
        let c_cross = skew(c);
        let expected_top_left = props.inertia_body - props.mass_kg() * c_cross * c_cross;

        for row in 0..3 {
            for col in 0..3 {
                assert_abs_diff_eq!(
                    spatial.matrix()[(row, col)],
                    expected_top_left[(row, col)],
                    epsilon = 1.0e-14
                );
                assert_abs_diff_eq!(
                    spatial.matrix()[(row, col + 3)],
                    props.mass_kg() * c_cross[(row, col)],
                    epsilon = 1.0e-14
                );
                assert_abs_diff_eq!(
                    spatial.matrix()[(row + 3, col)],
                    -props.mass_kg() * c_cross[(row, col)],
                    epsilon = 1.0e-14
                );
            }
            assert_abs_diff_eq!(spatial.matrix()[(row + 3, row + 3)], props.mass_kg());
        }
        for row in 0..6 {
            for col in 0..6 {
                assert_abs_diff_eq!(
                    spatial.matrix()[(row, col)],
                    spatial.matrix()[(col, row)],
                    epsilon = 1.0e-14
                );
            }
        }
    }

    #[test]
    fn plucker_transform_preserves_motion_force_power_pairing() {
        let rot = UnitQuaternion::from_euler_angles(0.2, -0.1, 0.3)
            .to_rotation_matrix()
            .into_inner();
        let transform = PluckerTransform::new(rot, Vector3::new(1.0, -2.0, 0.5)).unwrap();
        let parent_motion =
            SpatialMotion::new(Vector3::new(0.3, -0.4, 0.7), Vector3::new(8.0, -1.0, 2.0));
        let child_force =
            SpatialForce::new(Vector3::new(1.2, 3.4, -5.6), Vector3::new(7.8, -9.0, 1.1));

        let child_motion = transform.transform_motion_parent_to_child(parent_motion);
        let parent_force = transform.transform_force_child_to_parent(child_force);
        let child_power = child_motion.vector().dot(child_force.vector());
        let parent_power = parent_motion.vector().dot(parent_force.vector());

        assert_abs_diff_eq!(child_power, parent_power, epsilon = 1.0e-13);
    }

    #[test]
    fn plucker_transform_then_composes_parent_to_child_frames() {
        let parent_to_mid = PluckerTransform::new(
            UnitQuaternion::from_euler_angles(0.0, 0.0, core::f64::consts::FRAC_PI_2)
                .to_rotation_matrix()
                .into_inner(),
            Vector3::new(1.0, 2.0, 3.0),
        )
        .unwrap();
        let mid_to_child =
            PluckerTransform::new(Matrix3::identity(), Vector3::new(4.0, 0.0, -1.0)).unwrap();

        let composed = parent_to_mid.then(&mid_to_child).unwrap();

        assert_abs_diff_eq!(composed.translation_parent_m().x, 1.0, epsilon = 1.0e-14);
        assert_abs_diff_eq!(composed.translation_parent_m().y, -2.0, epsilon = 1.0e-14);
        assert_abs_diff_eq!(composed.translation_parent_m().z, 2.0, epsilon = 1.0e-14);
        assert_abs_diff_eq!(
            composed.motion_matrix_parent_to_child(),
            mid_to_child.motion_matrix_parent_to_child()
                * parent_to_mid.motion_matrix_parent_to_child(),
            epsilon = 1.0e-14
        );
    }

    #[test]
    fn joint_motion_subspaces_use_locked_axis_order() {
        let free = Joint::FreeFlyer.motion_subspace();
        let revolute = Joint::revolute(Vector3::new(0.0, 0.0, 2.0))
            .unwrap()
            .motion_subspace();
        let prismatic = Joint::prismatic(Vector3::new(3.0, 0.0, 0.0))
            .unwrap()
            .motion_subspace();

        assert_eq!(free.len(), 6);
        assert_eq!(free[0].angular_rad_s(), Vector3::new(1.0, 0.0, 0.0));
        assert_eq!(free[3].linear_m_s(), Vector3::new(1.0, 0.0, 0.0));
        assert_eq!(revolute[0].angular_rad_s(), Vector3::new(0.0, 0.0, 1.0));
        assert_eq!(revolute[0].linear_m_s(), Vector3::zeros());
        assert_eq!(prismatic[0].angular_rad_s(), Vector3::zeros());
        assert_eq!(prismatic[0].linear_m_s(), Vector3::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn joint_transforms_apply_configuration_coordinates() {
        let revolute = Joint::revolute(Vector3::new(0.0, 0.0, 2.0)).unwrap();
        let prismatic = Joint::prismatic(Vector3::new(3.0, 0.0, 0.0)).unwrap();

        let revolute_transform = revolute
            .joint_transform(&[core::f64::consts::FRAC_PI_2])
            .unwrap();
        let prismatic_transform = prismatic.joint_transform(&[2.5]).unwrap();

        assert_abs_diff_eq!(
            revolute_transform.rot_child_from_parent(),
            &Matrix3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            epsilon = 1.0e-14
        );
        assert_eq!(revolute_transform.translation_parent_m(), &Vector3::zeros());
        assert_eq!(
            prismatic_transform.rot_child_from_parent(),
            &Matrix3::identity()
        );
        assert_abs_diff_eq!(
            prismatic_transform.translation_parent_m(),
            &Vector3::new(2.5, 0.0, 0.0),
            epsilon = 1.0e-14
        );
    }

    #[test]
    fn quaternion_joint_transforms_normalize_and_reject_zero_norm() {
        let free = Joint::FreeFlyer;
        let spherical = Joint::Spherical;

        let free_transform = free
            .joint_transform(&[2.0, 0.0, 0.0, 0.0, 1.0, -2.0, 3.0])
            .unwrap();
        let spherical_transform = spherical.joint_transform(&[2.0, 0.0, 0.0, 0.0]).unwrap();
        let zero_quaternion_err = spherical
            .joint_transform(&[0.0, 0.0, 0.0, 0.0])
            .unwrap_err();

        assert_eq!(free_transform.rot_child_from_parent(), &Matrix3::identity());
        assert_eq!(
            free_transform.translation_parent_m(),
            &Vector3::new(1.0, -2.0, 3.0)
        );
        assert_eq!(
            spherical_transform.rot_child_from_parent(),
            &Matrix3::identity()
        );
        assert!(matches!(
            zero_quaternion_err,
            MultibodyError::InvalidParameter { .. }
        ));
    }

    #[test]
    fn multibody_tree_assigns_offsets_and_validates_state_dimensions() {
        let root = TreeBodySpec {
            id: BodyId::new(1),
            parent: None,
            joint: Joint::FreeFlyer,
            inertia: inertia(),
            parent_to_body: PluckerTransform::identity(),
        };
        let hinge = TreeBodySpec {
            id: BodyId::new(2),
            parent: Some(BodyIndex::new(0)),
            joint: Joint::revolute(Vector3::new(0.0, 1.0, 0.0)).unwrap(),
            inertia: inertia(),
            parent_to_body: PluckerTransform::identity(),
        };
        let welded = TreeBodySpec {
            id: BodyId::new(3),
            parent: Some(BodyIndex::new(1)),
            joint: Joint::Welded { released: false },
            inertia: inertia(),
            parent_to_body: PluckerTransform::identity(),
        };

        let tree = MultibodyTree::new(vec![root, hinge, welded]).unwrap();
        let good_state =
            MultibodyState::new(SimTime::ZERO, vec![0.0; tree.n_q()], vec![0.0; tree.n_qd()]);
        let bad_state = MultibodyState::new(
            SimTime::ZERO,
            vec![0.0; tree.n_q() - 1],
            vec![0.0; tree.n_qd()],
        );

        assert_eq!(tree.n_q(), 8);
        assert_eq!(tree.n_qd(), 7);
        assert_eq!(tree.bodies()[0].q_offset, 0);
        assert_eq!(tree.bodies()[1].q_offset, 7);
        assert_eq!(tree.bodies()[1].qd_offset, 6);
        assert!(tree.validate_state(&good_state).is_ok());
        assert!(matches!(
            tree.validate_state(&bad_state),
            Err(MultibodyError::DimensionMismatch { .. })
        ));
    }

    #[test]
    fn release_welded_subtree_as_free_flyer_preserves_pose_velocity_and_descendants() {
        let root = TreeBodySpec {
            id: BodyId::new(1),
            parent: None,
            joint: Joint::FreeFlyer,
            inertia: inertia(),
            parent_to_body: PluckerTransform::identity(),
        };
        let booster = TreeBodySpec {
            id: BodyId::new(2),
            parent: Some(BodyIndex::new(0)),
            joint: Joint::Welded { released: false },
            inertia: inertia(),
            parent_to_body: PluckerTransform::new(Matrix3::identity(), Vector3::new(1.0, 0.0, 0.0))
                .unwrap(),
        };
        let hinge = TreeBodySpec {
            id: BodyId::new(3),
            parent: Some(BodyIndex::new(1)),
            joint: Joint::revolute(Vector3::new(0.0, 1.0, 0.0)).unwrap(),
            inertia: inertia(),
            parent_to_body: PluckerTransform::identity(),
        };
        let sibling = TreeBodySpec {
            id: BodyId::new(4),
            parent: Some(BodyIndex::new(0)),
            joint: Joint::prismatic(Vector3::new(0.0, 0.0, 1.0)).unwrap(),
            inertia: inertia(),
            parent_to_body: PluckerTransform::identity(),
        };
        let tree = MultibodyTree::new(vec![root, booster, hinge, sibling]).unwrap();
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![1.0, 0.0, 0.0, 0.0, 10.0, 20.0, 30.0, 0.4, -2.0],
            vec![0.1, 0.2, 0.3, 1.0, 2.0, 3.0, 0.7, -0.5],
        );

        let (released_tree, released_state) = tree
            .release_welded_subtree_as_free_flyer(&state, BodyIndex::new(1))
            .unwrap();

        assert_eq!(released_tree.bodies().len(), 2);
        assert_eq!(released_tree.bodies()[0].id, BodyId::new(2));
        assert_eq!(released_tree.bodies()[1].id, BodyId::new(3));
        assert!(matches!(released_tree.bodies()[0].joint, Joint::FreeFlyer));
        assert_eq!(released_tree.bodies()[1].parent, Some(BodyIndex::new(0)));
        assert_eq!(released_tree.n_q(), 8);
        assert_eq!(released_tree.n_qd(), 7);
        assert_abs_diff_eq!(released_state.q[0], 1.0, epsilon = 1.0e-14);
        assert_abs_diff_eq!(released_state.q[1], 0.0, epsilon = 1.0e-14);
        assert_abs_diff_eq!(released_state.q[2], 0.0, epsilon = 1.0e-14);
        assert_abs_diff_eq!(released_state.q[3], 0.0, epsilon = 1.0e-14);
        assert_abs_diff_eq!(released_state.q[4], 11.0, epsilon = 1.0e-14);
        assert_abs_diff_eq!(released_state.q[5], 20.0, epsilon = 1.0e-14);
        assert_abs_diff_eq!(released_state.q[6], 30.0, epsilon = 1.0e-14);
        assert_abs_diff_eq!(released_state.q[7], 0.4, epsilon = 1.0e-14);
        assert_abs_diff_eq!(released_state.qd[0], 0.1, epsilon = 1.0e-14);
        assert_abs_diff_eq!(released_state.qd[1], 0.2, epsilon = 1.0e-14);
        assert_abs_diff_eq!(released_state.qd[2], 0.3, epsilon = 1.0e-14);
        assert_abs_diff_eq!(released_state.qd[3], 1.0, epsilon = 1.0e-14);
        assert_abs_diff_eq!(released_state.qd[4], 2.3, epsilon = 1.0e-14);
        assert_abs_diff_eq!(released_state.qd[5], 2.8, epsilon = 1.0e-14);
        assert_abs_diff_eq!(released_state.qd[6], 0.7, epsilon = 1.0e-14);
    }

    #[test]
    fn release_welded_subtree_as_free_flyer_rejects_root_and_non_welded_body() {
        let tree = sample_tree();
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![1.0, 0.0, 0.0, 0.0, 0.2, -0.1, 0.3, 0.4, 0.15],
            vec![0.0; tree.n_qd()],
        );

        let root_err = tree
            .release_welded_subtree_as_free_flyer(&state, BodyIndex::new(0))
            .unwrap_err();
        let non_welded_err = tree
            .release_welded_subtree_as_free_flyer(&state, BodyIndex::new(1))
            .unwrap_err();

        assert!(matches!(root_err, MultibodyError::InvalidTopology { .. }));
        assert!(matches!(
            non_welded_err,
            MultibodyError::InvalidParameter { .. }
        ));
    }

    #[test]
    fn body_transform_at_state_composes_fixed_and_joint_transforms() {
        let root = TreeBodySpec {
            id: BodyId::new(1),
            parent: None,
            joint: Joint::FreeFlyer,
            inertia: inertia(),
            parent_to_body: PluckerTransform::identity(),
        };
        let slider = TreeBodySpec {
            id: BodyId::new(2),
            parent: Some(BodyIndex::new(0)),
            joint: Joint::prismatic(Vector3::new(1.0, 0.0, 0.0)).unwrap(),
            inertia: inertia(),
            parent_to_body: PluckerTransform::new(
                UnitQuaternion::from_euler_angles(0.0, 0.0, core::f64::consts::FRAC_PI_2)
                    .to_rotation_matrix()
                    .into_inner(),
                Vector3::new(1.0, 2.0, 3.0),
            )
            .unwrap(),
        };
        let tree = MultibodyTree::new(vec![root, slider]).unwrap();
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0],
            vec![0.0; tree.n_qd()],
        );

        let transform = tree
            .body_transform_parent_to_child_at_state(BodyIndex::new(1), &state)
            .unwrap();

        assert_abs_diff_eq!(transform.translation_parent_m().x, 1.0, epsilon = 1.0e-14);
        assert_abs_diff_eq!(transform.translation_parent_m().y, -2.0, epsilon = 1.0e-14);
        assert_abs_diff_eq!(transform.translation_parent_m().z, 3.0, epsilon = 1.0e-14);
    }

    #[test]
    fn multibody_tree_rejects_invalid_topology() {
        let child_before_parent = TreeBodySpec {
            id: BodyId::new(2),
            parent: Some(BodyIndex::new(1)),
            joint: Joint::revolute(Vector3::new(0.0, 1.0, 0.0)).unwrap(),
            inertia: inertia(),
            parent_to_body: PluckerTransform::identity(),
        };
        let root = TreeBodySpec {
            id: BodyId::new(1),
            parent: None,
            joint: Joint::FreeFlyer,
            inertia: inertia(),
            parent_to_body: PluckerTransform::identity(),
        };

        assert!(matches!(
            MultibodyTree::new(vec![child_before_parent, root]),
            Err(MultibodyError::InvalidTopology { .. })
        ));
    }

    #[test]
    fn state_dependent_transforms_reject_bad_body_index_or_quaternion() {
        let tree = sample_tree();
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.1],
            vec![0.0; tree.n_qd()],
        );
        let invalid_body = tree
            .body_transform_parent_to_child_at_state(BodyIndex::new(99), &state)
            .unwrap_err();
        let bad_quaternion_state = MultibodyState::new(
            SimTime::ZERO,
            vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.1],
            vec![0.0; tree.n_qd()],
        );
        let bad_quaternion = tree
            .joint_space_inertia_crba_at_state(&bad_quaternion_state)
            .unwrap_err();

        assert!(matches!(
            invalid_body,
            MultibodyError::InvalidBodyIndex {
                index: 99,
                body_count: 3
            }
        ));
        assert!(matches!(
            bad_quaternion,
            MultibodyError::InvalidParameter { .. }
        ));
    }

    #[test]
    fn multibody_derivative_checked_arithmetic_and_norm_are_locked() {
        let derivative = MultibodyDerivative::new(vec![1.0, -2.0, 3.0], vec![4.0, -5.0, 6.0, -7.0]);
        let scaled = derivative.scaled(0.5).unwrap();
        let summed = derivative.checked_add(&scaled).unwrap();
        let mismatch = derivative
            .checked_add(&MultibodyDerivative::new(vec![1.0], vec![2.0]))
            .unwrap_err();

        assert_eq!(derivative.dimension(), 7);
        assert_abs_diff_eq!(
            derivative.l2_norm(),
            <f64 as nalgebra::ComplexField>::sqrt(1.0 + 4.0 + 9.0 + 16.0 + 25.0 + 36.0 + 49.0),
            epsilon = 1.0e-15
        );
        assert_eq!(scaled.q_dot(), &[0.5, -1.0, 1.5]);
        assert_eq!(scaled.qd_dot(), &[2.0, -2.5, 3.0, -3.5]);
        assert_eq!(summed.q_dot(), &[1.5, -3.0, 4.5]);
        assert_eq!(summed.qd_dot(), &[6.0, -7.5, 9.0, -10.5]);
        assert!(matches!(
            mismatch,
            MultibodyError::GeneralizedVectorDimensionMismatch {
                vector: "q_dot",
                ..
            }
        ));
    }

    #[test]
    fn advance_state_by_integrates_components_and_projects_quaternions() {
        let tree = sample_tree();
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.2],
            vec![1.0, -1.0, 2.0, -2.0, 3.0, -3.0, 4.0, -4.0],
        );
        let derivative = MultibodyDerivative::new(
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 0.4, 0.5],
            vec![0.2, 0.4, -0.6, -0.8, 1.0, -1.2, 1.4, -1.6],
        );

        let advanced = tree.advance_state_by(&state, 0.5, &derivative).unwrap();

        assert_abs_diff_eq!(advanced.time.as_seconds(), 0.5, epsilon = 0.0);
        assert_eq!(&advanced.q[0..4], &[1.0, 0.0, 0.0, 0.0]);
        assert_abs_diff_eq!(advanced.q[4], 0.5, epsilon = 1.0e-15);
        assert_abs_diff_eq!(advanced.q[5], 1.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(advanced.q[6], 1.5, epsilon = 1.0e-15);
        assert_abs_diff_eq!(advanced.q[7], 0.3, epsilon = 1.0e-15);
        assert_abs_diff_eq!(advanced.q[8], 0.45, epsilon = 1.0e-15);
        assert_abs_diff_eq!(advanced.qd[0], 1.1, epsilon = 1.0e-15);
        assert_abs_diff_eq!(advanced.qd[1], -0.8, epsilon = 1.0e-15);
        assert_abs_diff_eq!(advanced.qd[7], -4.8, epsilon = 1.0e-15);
    }

    #[test]
    fn project_state_normalizes_free_flyer_and_spherical_quaternions() {
        let root = TreeBodySpec {
            id: BodyId::new(1),
            parent: None,
            joint: Joint::FreeFlyer,
            inertia: inertia(),
            parent_to_body: PluckerTransform::identity(),
        };
        let spherical = TreeBodySpec {
            id: BodyId::new(2),
            parent: Some(BodyIndex::new(0)),
            joint: Joint::Spherical,
            inertia: inertia(),
            parent_to_body: PluckerTransform::identity(),
        };
        let tree = MultibodyTree::new(vec![root, spherical]).unwrap();
        let mut state = MultibodyState::new(
            SimTime::ZERO,
            vec![2.0, 0.0, 0.0, 0.0, 7.0, 8.0, 9.0, 0.0, 0.0, 3.0, 4.0],
            vec![0.0; tree.n_qd()],
        );

        tree.project_state(&mut state).unwrap();

        assert_eq!(&state.q[0..4], &[1.0, 0.0, 0.0, 0.0]);
        assert_eq!(&state.q[4..7], &[7.0, 8.0, 9.0]);
        assert_abs_diff_eq!(state.q[7], 0.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(state.q[8], 0.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(state.q[9], 0.6, epsilon = 1.0e-15);
        assert_abs_diff_eq!(state.q[10], 0.8, epsilon = 1.0e-15);
    }

    #[test]
    fn coordinate_derivative_lifts_free_flyer_and_scalar_velocities() {
        let tree = sample_tree();
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![1.0, 0.0, 0.0, 0.0, 10.0, 20.0, 30.0, 0.25, 0.5],
            vec![0.0, 0.0, 2.0, 1.0, 2.0, 3.0, 4.0, -5.0],
        );

        let q_dot = tree.coordinate_derivative_from_velocity(&state).unwrap();

        assert_eq!(q_dot.len(), tree.n_q());
        assert_abs_diff_eq!(q_dot[0], 0.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(q_dot[1], 0.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(q_dot[2], 0.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(q_dot[3], 1.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(q_dot[4], 1.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(q_dot[5], 2.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(q_dot[6], 3.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(q_dot[7], 4.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(q_dot[8], -5.0, epsilon = 1.0e-15);
    }

    #[test]
    fn coordinate_derivative_uses_raw_free_flyer_quaternion_for_rate() {
        let tree = sample_tree();
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![2.0, 0.0, 0.0, 0.0, 10.0, 20.0, 30.0, 0.25, 0.5],
            vec![0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 4.0, -5.0],
        );

        let q_dot = tree.coordinate_derivative_from_velocity(&state).unwrap();

        assert_abs_diff_eq!(q_dot[0], 0.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(q_dot[1], 0.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(q_dot[2], 0.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(q_dot[3], 2.0, epsilon = 1.0e-15);
    }

    #[test]
    fn coordinate_derivative_rotates_free_flyer_body_velocity_to_parent() {
        let tree = sample_tree();
        let inv_sqrt_2 = 1.0 / <f64 as nalgebra::ComplexField>::sqrt(2.0);
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![
                inv_sqrt_2, 0.0, 0.0, inv_sqrt_2, 10.0, 20.0, 30.0, 0.25, 0.5,
            ],
            vec![0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 4.0, -5.0],
        );

        let q_dot = tree.coordinate_derivative_from_velocity(&state).unwrap();

        assert_abs_diff_eq!(q_dot[4], -2.0, epsilon = 1.0e-14);
        assert_abs_diff_eq!(q_dot[5], 1.0, epsilon = 1.0e-14);
        assert_abs_diff_eq!(q_dot[6], 3.0, epsilon = 1.0e-14);
        assert_abs_diff_eq!(q_dot[7], 4.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(q_dot[8], -5.0, epsilon = 1.0e-15);
    }

    #[test]
    fn coordinate_derivative_lifts_spherical_quaternion_rate() {
        let root = TreeBodySpec {
            id: BodyId::new(1),
            parent: None,
            joint: Joint::FreeFlyer,
            inertia: inertia(),
            parent_to_body: PluckerTransform::identity(),
        };
        let spherical = TreeBodySpec {
            id: BodyId::new(2),
            parent: Some(BodyIndex::new(0)),
            joint: Joint::Spherical,
            inertia: inertia(),
            parent_to_body: PluckerTransform::identity(),
        };
        let tree = MultibodyTree::new(vec![root, spherical]).unwrap();
        let inv_sqrt_2 = 1.0 / <f64 as nalgebra::ComplexField>::sqrt(2.0);
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![
                1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, inv_sqrt_2, inv_sqrt_2, 0.0, 0.0,
            ],
            vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0],
        );

        let q_dot = tree.coordinate_derivative_from_velocity(&state).unwrap();
        let expected = quaternion_derivative_from_body_rate(
            &[inv_sqrt_2, inv_sqrt_2, 0.0, 0.0],
            Vector3::new(1.0, 2.0, 3.0),
        )
        .unwrap();

        assert_eq!(q_dot.len(), tree.n_q());
        for axis in 0..4 {
            assert_abs_diff_eq!(q_dot[7 + axis], expected[axis], epsilon = 1.0e-15);
        }
    }

    #[test]
    fn derivative_from_state_and_acceleration_pairs_lifted_q_dot_with_qdd() {
        let tree = sample_tree();
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![1.0, 0.0, 0.0, 0.0, 0.2, -0.1, 0.3, 0.4, 0.15],
            vec![0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7, -0.8],
        );
        let qdd = vec![0.2, -0.1, 0.3, -0.4, 0.5, -0.6, 0.7, -0.8];

        let derivative = tree
            .derivative_from_state_and_acceleration(&state, &qdd)
            .unwrap();
        let q_dot = tree.coordinate_derivative_from_velocity(&state).unwrap();
        let short_qdd = tree
            .derivative_from_state_and_acceleration(&state, &qdd[0..qdd.len() - 1])
            .unwrap_err();

        assert_eq!(derivative.q_dot(), q_dot.as_slice());
        assert_eq!(derivative.qd_dot(), qdd.as_slice());
        assert!(matches!(
            short_qdd,
            MultibodyError::GeneralizedVectorDimensionMismatch {
                vector: "generalized acceleration",
                ..
            }
        ));
    }

    fn assert_sim_state_trait<S: SimState>(_state: &S) {}

    #[test]
    fn multibody_sim_state_implements_model_traits_and_projects_quaternions() {
        let root = TreeBodySpec {
            id: BodyId::new(1),
            parent: None,
            joint: Joint::FreeFlyer,
            inertia: inertia(),
            parent_to_body: PluckerTransform::identity(),
        };
        let spherical = TreeBodySpec {
            id: BodyId::new(2),
            parent: Some(BodyIndex::new(0)),
            joint: Joint::Spherical,
            inertia: inertia(),
            parent_to_body: PluckerTransform::identity(),
        };
        let tree = MultibodyTree::new(vec![root, spherical]).unwrap();
        let sim_state = tree
            .sim_state(
                SimTime::ZERO,
                vec![2.0, 0.0, 0.0, 0.0, 10.0, 20.0, 30.0, 0.0, 3.0, 4.0, 0.0],
                vec![0.0; tree.n_qd()],
            )
            .unwrap();
        let derivative = MultibodyDerivative::new(
            vec![0.5, 0.1, -0.2, 0.3, 1.0, 2.0, 3.0, 0.4, -0.5, 0.6, -0.7],
            vec![0.2, -0.1, 0.3, -0.4, 0.5, -0.6, 0.7, -0.8, 0.9],
        );

        assert_sim_state_trait(&sim_state);
        assert_eq!(sim_state.quaternion_offsets(), &[0, 7]);
        assert_eq!(derivative.dimension(), tree.n_q() + tree.n_qd());
        assert!(derivative.is_finite());

        let mut advanced = sim_state.advance_by(0.5, &derivative);
        advanced.project();
        let retimed = advanced.clone().with_time(SimTime::from_seconds(9.0));
        let norm = advanced.weighted_error_norm(&sim_state, &derivative, 0.5, 1.0e-6, 1.0e-3);

        assert_abs_diff_eq!(advanced.time().as_seconds(), 0.5, epsilon = 0.0);
        assert_abs_diff_eq!(retimed.time().as_seconds(), 9.0, epsilon = 0.0);
        assert!(advanced.is_valid_for_integration());
        assert!(norm.is_finite());
        assert!(norm > 0.0);

        let root_q_norm = advanced.q()[0..4]
            .iter()
            .map(|value| value * value)
            .sum::<f64>();
        let spherical_q_norm = advanced.q()[7..11]
            .iter()
            .map(|value| value * value)
            .sum::<f64>();
        assert_abs_diff_eq!(root_q_norm, 1.0, epsilon = 1.0e-14);
        assert_abs_diff_eq!(spherical_q_norm, 1.0, epsilon = 1.0e-14);
    }

    #[test]
    fn multibody_sim_state_rejects_bad_projection_metadata() {
        let state = MultibodyState::new(SimTime::ZERO, vec![1.0, 0.0, 0.0, 0.0], Vec::new());
        let duplicate = MultibodySimState::new(state.clone(), vec![0, 0]).unwrap_err();
        let out_of_range = MultibodySimState::new(state.clone(), vec![1]).unwrap_err();
        let zero_norm = MultibodySimState::new(
            MultibodyState::new(SimTime::ZERO, vec![0.0, 0.0, 0.0, 0.0], Vec::new()),
            vec![0],
        )
        .unwrap_err();

        assert!(matches!(duplicate, MultibodyError::InvalidParameter { .. }));
        assert!(matches!(
            out_of_range,
            MultibodyError::GeneralizedVectorDimensionMismatch {
                vector: "quaternion offset",
                ..
            }
        ));
        assert!(matches!(zero_norm, MultibodyError::InvalidParameter { .. }));
    }

    #[test]
    fn scalar_state_size_and_weighted_error_norm_use_locked_order() {
        let tree = sample_tree();
        let prev = MultibodyState::new(
            SimTime::ZERO,
            vec![1.0, 0.0, 0.0, 0.0, 0.2, -0.1, 0.3, 0.4, 0.15],
            vec![0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7, -0.8],
        );
        let end = MultibodyState::new(
            SimTime::from_seconds(0.25),
            vec![1.0, 0.0, 0.0, 0.0, 0.25, -0.05, 0.35, 0.45, 0.2],
            vec![0.2, -0.1, 0.4, -0.3, 0.6, -0.5, 0.8, -0.7],
        );
        let error = MultibodyDerivative::new(
            vec![0.01, -0.02, 0.03, -0.04, 0.05, -0.06, 0.07, -0.08, 0.09],
            vec![0.11, -0.12, 0.13, -0.14, 0.15, -0.16, 0.17, -0.18],
        );

        let state_size = tree.scalar_state_size(&prev).unwrap();
        let norm = tree
            .weighted_error_norm(&end, &prev, &error, 0.25, 1.0e-6, 1.0e-3)
            .unwrap();

        let mut expected_size_sum = 0.0;
        for value in &prev.q {
            expected_size_sum += value * value;
        }
        for value in &prev.qd {
            expected_size_sum += value * value;
        }
        let mut expected_norm_sum = 0.0;
        for index in 0..tree.n_q() {
            expected_norm_sum += weighted_error_term(
                prev.q[index],
                end.q[index],
                error.q_dot()[index],
                0.25,
                1.0e-6,
                1.0e-3,
            );
        }
        for index in 0..tree.n_qd() {
            expected_norm_sum += weighted_error_term(
                prev.qd[index],
                end.qd[index],
                error.qd_dot()[index],
                0.25,
                1.0e-6,
                1.0e-3,
            );
        }
        let expected_norm = <f64 as nalgebra::ComplexField>::sqrt(expected_norm_sum / 17.0);

        assert_abs_diff_eq!(
            state_size,
            <f64 as nalgebra::ComplexField>::sqrt(expected_size_sum),
            epsilon = 1.0e-15
        );
        assert_abs_diff_eq!(norm, expected_norm, epsilon = 1.0e-15);
        assert!(matches!(
            tree.weighted_error_norm(&end, &prev, &error, 0.25, 0.0, 1.0e-3),
            Err(MultibodyError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn crba_fixed_transforms_single_root_matches_spatial_inertia() {
        let root_inertia = inertia();
        let tree = MultibodyTree::new(vec![TreeBodySpec {
            id: BodyId::new(1),
            parent: None,
            joint: Joint::FreeFlyer,
            inertia: root_inertia,
            parent_to_body: PluckerTransform::identity(),
        }])
        .unwrap();

        let h = tree.joint_space_inertia_crba_fixed_transforms().unwrap();

        assert_eq!(h.dimension(), 6);
        for row in 0..6 {
            for col in 0..6 {
                assert_abs_diff_eq!(
                    h.at(row, col).unwrap(),
                    root_inertia.matrix()[(row, col)],
                    epsilon = 1.0e-14
                );
            }
        }
    }

    #[test]
    fn crba_fixed_transforms_matches_rnea_columns() {
        let tree = sample_tree();
        let h = tree.joint_space_inertia_crba_fixed_transforms().unwrap();

        for col in 0..tree.n_qd() {
            let mut qdd = vec![0.0; tree.n_qd()];
            qdd[col] = 1.0;
            let tau = tree.inverse_dynamics_rnea_fixed_transforms(&qdd).unwrap();
            for (row, tau_row) in tau.iter().enumerate() {
                assert_abs_diff_eq!(*tau_row, h.at(row, col).unwrap(), epsilon = 1.0e-13);
            }
        }

        for row in 0..h.dimension() {
            assert!(h.at(row, row).unwrap() > 0.0);
            for col in 0..h.dimension() {
                assert_abs_diff_eq!(
                    h.at(row, col).unwrap(),
                    h.at(col, row).unwrap(),
                    epsilon = 1.0e-14
                );
            }
        }
    }

    #[test]
    fn crba_at_state_matches_rnea_no_bias_columns() {
        let tree = sample_tree();
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![1.0, 0.0, 0.0, 0.0, 0.2, -0.1, 0.3, 0.4, 0.15],
            vec![0.0; tree.n_qd()],
        );
        let h = tree.joint_space_inertia_crba_at_state(&state).unwrap();

        for col in 0..tree.n_qd() {
            let mut qdd = vec![0.0; tree.n_qd()];
            qdd[col] = 1.0;
            let tau = tree
                .inverse_dynamics_rnea_at_state_no_bias(&state, &qdd)
                .unwrap();
            for (row, tau_row) in tau.iter().enumerate() {
                assert_abs_diff_eq!(*tau_row, h.at(row, col).unwrap(), epsilon = 1.0e-13);
            }
        }

        for row in 0..h.dimension() {
            assert!(h.at(row, row).unwrap() > 0.0);
            for col in 0..h.dimension() {
                assert_abs_diff_eq!(
                    h.at(row, col).unwrap(),
                    h.at(col, row).unwrap(),
                    epsilon = 1.0e-14
                );
            }
        }
    }

    #[test]
    fn rnea_at_state_matches_no_bias_when_velocity_and_forces_are_zero() {
        let tree = sample_tree();
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![1.0, 0.0, 0.0, 0.0, 0.2, -0.1, 0.3, 0.4, 0.15],
            vec![0.0; tree.n_qd()],
        );
        let qdd = vec![0.2, -0.1, 0.3, -0.4, 0.5, -0.6, 0.7, -0.8];
        let external_forces = vec![SpatialForce::zero(); tree.bodies().len()];

        let full = tree
            .inverse_dynamics_rnea_at_state(&state, &qdd, SpatialMotion::zero(), &external_forces)
            .unwrap();
        let no_bias = tree
            .inverse_dynamics_rnea_at_state_no_bias(&state, &qdd)
            .unwrap();

        assert_eq!(full.len(), no_bias.len());
        for (actual, expected) in full.iter().zip(no_bias.iter()) {
            assert_abs_diff_eq!(*actual, *expected, epsilon = 1.0e-13);
        }
    }

    #[test]
    fn rnea_at_state_includes_free_flyer_velocity_bias() {
        let root_inertia = inertia();
        let tree = MultibodyTree::new(vec![TreeBodySpec {
            id: BodyId::new(1),
            parent: None,
            joint: Joint::FreeFlyer,
            inertia: root_inertia,
            parent_to_body: PluckerTransform::identity(),
        }])
        .unwrap();
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            vec![0.3, -0.4, 0.5, 1.2, -0.7, 0.9],
        );
        let qdd = vec![0.0; tree.n_qd()];
        let external_forces = vec![SpatialForce::zero(); tree.bodies().len()];

        let tau = tree
            .inverse_dynamics_rnea_at_state(&state, &qdd, SpatialMotion::zero(), &external_forces)
            .unwrap();
        let velocity = SpatialMotion::from_vector(SpatialVector::from_column_slice(&state.qd));
        let expected = velocity.crossf() * (root_inertia.matrix() * velocity.vector());

        for axis in 0..tree.n_qd() {
            assert_abs_diff_eq!(tau[axis], expected[axis], epsilon = 1.0e-13);
        }
    }

    #[test]
    fn rnea_at_state_applies_root_acceleration_and_external_forces() {
        let root_inertia = inertia();
        let tree = MultibodyTree::new(vec![TreeBodySpec {
            id: BodyId::new(1),
            parent: None,
            joint: Joint::FreeFlyer,
            inertia: root_inertia,
            parent_to_body: PluckerTransform::identity(),
        }])
        .unwrap();
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            vec![0.0; tree.n_qd()],
        );
        let qdd = vec![0.0; tree.n_qd()];
        let root_acceleration = SpatialMotion::new(Vector3::zeros(), Vector3::new(0.0, 0.0, -9.81));
        let expected_force = root_inertia.matrix() * root_acceleration.vector();
        let no_external = vec![SpatialForce::zero(); tree.bodies().len()];
        let canceling_external = vec![SpatialForce::from_vector(expected_force)];

        let tau_gravity = tree
            .inverse_dynamics_rnea_at_state(&state, &qdd, root_acceleration, &no_external)
            .unwrap();
        let tau_cancelled = tree
            .inverse_dynamics_rnea_at_state(&state, &qdd, root_acceleration, &canceling_external)
            .unwrap();

        for axis in 0..tree.n_qd() {
            assert_abs_diff_eq!(tau_gravity[axis], expected_force[axis], epsilon = 1.0e-13);
            assert_abs_diff_eq!(tau_cancelled[axis], 0.0, epsilon = 1.0e-13);
        }
    }

    #[test]
    fn rnea_at_state_rejects_bad_base_acceleration_or_external_forces() {
        let tree = sample_tree();
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![1.0, 0.0, 0.0, 0.0, 0.2, -0.1, 0.3, 0.4, 0.15],
            vec![0.0; tree.n_qd()],
        );
        let qdd = vec![0.0; tree.n_qd()];
        let short_external = vec![SpatialForce::zero(); tree.bodies().len() - 1];
        let mut nonfinite_external = vec![SpatialForce::zero(); tree.bodies().len()];
        nonfinite_external[0] =
            SpatialForce::new(Vector3::new(f64::NAN, 0.0, 0.0), Vector3::zeros());

        let short_err = tree
            .inverse_dynamics_rnea_at_state(&state, &qdd, SpatialMotion::zero(), &short_external)
            .unwrap_err();
        let external_err = tree
            .inverse_dynamics_rnea_at_state(
                &state,
                &qdd,
                SpatialMotion::zero(),
                &nonfinite_external,
            )
            .unwrap_err();
        let base_err = tree
            .inverse_dynamics_rnea_at_state(
                &state,
                &qdd,
                SpatialMotion::new(Vector3::new(f64::NAN, 0.0, 0.0), Vector3::zeros()),
                &vec![SpatialForce::zero(); tree.bodies().len()],
            )
            .unwrap_err();

        assert!(matches!(
            short_err,
            MultibodyError::GeneralizedVectorDimensionMismatch {
                vector: "external forces",
                ..
            }
        ));
        assert!(matches!(external_err, MultibodyError::NonFinite { .. }));
        assert!(matches!(base_err, MultibodyError::NonFinite { .. }));
    }

    #[test]
    fn forward_dynamics_dense_round_trips_biased_rnea() {
        let tree = sample_tree();
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![1.0, 0.0, 0.0, 0.0, 0.2, -0.1, 0.3, 0.4, 0.15],
            vec![0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7, -0.8],
        );
        let expected_qdd = vec![0.25, -0.15, 0.35, -0.45, 0.55, -0.65, 0.75, -0.85];
        let root_acceleration = SpatialMotion::new(Vector3::zeros(), Vector3::new(0.0, 0.0, -9.81));
        let external_forces = vec![
            SpatialForce::zero(),
            SpatialForce::new(Vector3::new(0.1, -0.2, 0.3), Vector3::new(0.4, -0.5, 0.6)),
            SpatialForce::new(Vector3::new(-0.3, 0.2, -0.1), Vector3::new(-0.6, 0.5, -0.4)),
        ];
        let tau = tree
            .inverse_dynamics_rnea_at_state(
                &state,
                &expected_qdd,
                root_acceleration,
                &external_forces,
            )
            .unwrap();

        let actual_qdd = tree
            .forward_dynamics_dense_at_state(&state, &tau, root_acceleration, &external_forces)
            .unwrap();

        for (actual, expected) in actual_qdd.iter().zip(expected_qdd.iter()) {
            assert_abs_diff_eq!(*actual, *expected, epsilon = 1.0e-11);
        }
    }

    #[test]
    fn forward_dynamics_aba_matches_dense_bridge() {
        let tree = sample_tree();
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![1.0, 0.0, 0.0, 0.0, 0.2, -0.1, 0.3, 0.4, 0.15],
            vec![0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7, -0.8],
        );
        let generalized_forces = vec![1.0, -0.5, 0.25, -0.75, 0.6, -0.4, 0.9, -0.2];
        let root_acceleration = SpatialMotion::new(Vector3::zeros(), Vector3::new(0.0, 0.0, -9.81));
        let external_forces = vec![
            SpatialForce::zero(),
            SpatialForce::new(Vector3::new(0.2, -0.1, 0.05), Vector3::new(0.3, -0.2, 0.1)),
            SpatialForce::new(
                Vector3::new(-0.05, 0.08, -0.03),
                Vector3::new(-0.4, 0.2, -0.1),
            ),
        ];

        let dense = tree
            .forward_dynamics_dense_at_state(
                &state,
                &generalized_forces,
                root_acceleration,
                &external_forces,
            )
            .unwrap();
        let aba = tree
            .forward_dynamics_aba_at_state(
                &state,
                &generalized_forces,
                root_acceleration,
                &external_forces,
            )
            .unwrap();
        let tau_round_trip = tree
            .inverse_dynamics_rnea_at_state(&state, &aba, root_acceleration, &external_forces)
            .unwrap();

        for (actual, expected) in aba.iter().zip(dense.iter()) {
            assert_abs_diff_eq!(*actual, *expected, epsilon = 1.0e-10);
        }
        for (actual, expected) in tau_round_trip.iter().zip(generalized_forces.iter()) {
            assert_abs_diff_eq!(*actual, *expected, epsilon = 1.0e-10);
        }
    }

    #[test]
    fn forward_dynamics_aba_matches_checked_tolerance_table() {
        let fixture = load_aba_tolerance_fixture();
        let tree = sample_tree();
        assert_eq!(fixture.q.len(), tree.n_q());
        assert_eq!(fixture.qd.len(), tree.n_qd());
        assert_eq!(fixture.generalized_forces.len(), tree.n_qd());
        assert_eq!(fixture.external_forces.len(), tree.bodies().len());

        let state = MultibodyState::new(SimTime::ZERO, fixture.q, fixture.qd);
        let root_acceleration = motion_from_fixture(&fixture.root_acceleration);
        let external_forces: Vec<SpatialForce> = fixture
            .external_forces
            .iter()
            .map(force_from_fixture)
            .collect();

        let dense = tree
            .forward_dynamics_dense_at_state(
                &state,
                &fixture.generalized_forces,
                root_acceleration,
                &external_forces,
            )
            .unwrap();
        let aba = tree
            .forward_dynamics_aba_at_state(
                &state,
                &fixture.generalized_forces,
                root_acceleration,
                &external_forces,
            )
            .unwrap();
        let tau_round_trip = tree
            .inverse_dynamics_rnea_at_state(&state, &aba, root_acceleration, &external_forces)
            .unwrap();

        assert_max_abs_diff(
            &aba,
            &dense,
            fixture.tolerances.aba_dense_max_abs,
            "ABA versus dense",
        );
        assert_max_abs_diff(
            &tau_round_trip,
            &fixture.generalized_forces,
            fixture.tolerances.rnea_round_trip_max_abs,
            "RNEA round trip",
        );
    }

    #[test]
    fn forward_dynamics_aba_matches_spatial_v2_compatible_oracle_fixture() {
        let fixture = load_spatial_v2_compatible_oracle_fixture();
        let tree = sample_tree();
        assert_eq!(fixture.source.export_format, "spatial-v2-aba-qdd-v1");
        assert_eq!(fixture.source.independence, "self-consistency");
        assert_eq!(fixture.source.solver, "openbmp-multibody");
        assert!(
            fixture
                .source
                .notes
                .contains("not an independent Featherstone Spatial_v2 run")
        );
        assert_eq!(fixture.q.len(), tree.n_q());
        assert_eq!(fixture.qd.len(), tree.n_qd());
        assert_eq!(fixture.generalized_forces.len(), tree.n_qd());
        assert_eq!(fixture.expected_qdd.len(), tree.n_qd());
        assert_eq!(fixture.external_forces.len(), tree.bodies().len());

        let state = MultibodyState::new(SimTime::ZERO, fixture.q, fixture.qd);
        let root_acceleration = motion_from_fixture(&fixture.root_acceleration);
        let external_forces: Vec<SpatialForce> = fixture
            .external_forces
            .iter()
            .map(force_from_fixture)
            .collect();

        let aba = tree
            .forward_dynamics_aba_at_state(
                &state,
                &fixture.generalized_forces,
                root_acceleration,
                &external_forces,
            )
            .unwrap();

        assert_max_abs_diff(
            &aba,
            &fixture.expected_qdd,
            fixture.tolerances.qdd_max_abs,
            "Spatial_v2-compatible ABA oracle qdd",
        );
    }

    #[test]
    fn energy_and_generalized_momentum_match_double_pendulum_tolerance_table() {
        let fixture = load_double_pendulum_energy_momentum_fixture();
        let tree = double_pendulum_fixture_tree(&fixture);
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                fixture.q[0],
                fixture.q[1],
            ],
            vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, fixture.qd[0], fixture.qd[1]],
        );

        let kinetic_energy = tree.kinetic_energy_at_state(&state).unwrap();
        let generalized_momentum = tree.generalized_momentum_at_state(&state).unwrap();

        assert_abs_diff_eq!(
            kinetic_energy,
            fixture.expected_kinetic_energy_j,
            epsilon = fixture.tolerances.kinetic_energy_abs
        );
        assert_max_abs_diff(
            &generalized_momentum[6..8],
            &fixture.expected_revolute_generalized_momentum,
            fixture.tolerances.generalized_momentum_max_abs,
            "double-pendulum revolute generalized momentum",
        );
    }

    #[test]
    fn articulated_gimbal_joint_inertia_couples_root_and_engine_axis() {
        let airframe_properties = MassProperties::with_diagonal_inertia(
            Mass::new::<kilogram>(1800.0),
            Position3::<Body>::new(0.0, 0.0, 0.0),
            900.0,
            1300.0,
            850.0,
        );
        let engine_properties = MassProperties::with_diagonal_inertia(
            Mass::new::<kilogram>(115.0),
            Position3::<Body>::new(0.18, 0.0, -0.42),
            9.0,
            17.0,
            14.0,
        );
        let tree = MultibodyTree::new(vec![
            TreeBodySpec {
                id: BodyId::new(1),
                parent: None,
                joint: Joint::FreeFlyer,
                inertia: SpatialInertia::from_mass_properties(&airframe_properties).unwrap(),
                parent_to_body: PluckerTransform::identity(),
            },
            TreeBodySpec {
                id: BodyId::new(2),
                parent: Some(BodyIndex::new(0)),
                joint: Joint::revolute(Vector3::new(0.0, 1.0, 0.0)).unwrap(),
                inertia: SpatialInertia::from_mass_properties(&engine_properties).unwrap(),
                parent_to_body: PluckerTransform::new(
                    Matrix3::identity(),
                    Vector3::new(0.72, 0.0, -3.4),
                )
                .unwrap(),
            },
        ])
        .unwrap();
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.18],
            vec![0.0; tree.n_qd()],
        );

        let h = tree.joint_space_inertia_crba_at_state(&state).unwrap();
        let gimbal_col = 6;
        let root_rotational_coupling = (0..3)
            .map(|row| h.at(row, gimbal_col).unwrap().abs())
            .fold(0.0_f64, f64::max);
        let root_translational_coupling = (3..6)
            .map(|row| h.at(row, gimbal_col).unwrap().abs())
            .fold(0.0_f64, f64::max);

        assert!(
            root_rotational_coupling > 1.0,
            "gimbal inertia column should couple into root angular slots: {root_rotational_coupling:.17e}"
        );
        assert!(
            root_translational_coupling > 1.0,
            "offset gimbal inertia column should couple into root linear slots: {root_translational_coupling:.17e}"
        );
        for root_row in 0..6 {
            assert_abs_diff_eq!(
                h.at(root_row, gimbal_col).unwrap(),
                h.at(gimbal_col, root_row).unwrap(),
                epsilon = 1.0e-12
            );
        }

        let mut generalized_forces = vec![0.0; tree.n_qd()];
        generalized_forces[gimbal_col] = 240.0;
        let external_forces = vec![SpatialForce::zero(); tree.bodies().len()];

        let dense = tree
            .forward_dynamics_dense_at_state(
                &state,
                &generalized_forces,
                SpatialMotion::zero(),
                &external_forces,
            )
            .unwrap();
        let aba = tree
            .forward_dynamics_aba_at_state(
                &state,
                &generalized_forces,
                SpatialMotion::zero(),
                &external_forces,
            )
            .unwrap();
        let tau_round_trip = tree
            .inverse_dynamics_rnea_at_state(&state, &aba, SpatialMotion::zero(), &external_forces)
            .unwrap();
        let root_reaction_qdd = aba[0..6]
            .iter()
            .map(|value| value.abs())
            .fold(0.0_f64, f64::max);

        assert!(
            root_reaction_qdd > 1.0e-4,
            "actuated gimbal should drive an equal-and-opposite root reaction: {root_reaction_qdd:.17e}"
        );
        assert!(
            aba[gimbal_col].abs() > 1.0e-4,
            "actuated gimbal axis should accelerate: {:.17e}",
            aba[gimbal_col]
        );
        for (actual, expected) in aba.iter().zip(dense.iter()) {
            assert_abs_diff_eq!(*actual, *expected, epsilon = 1.0e-10);
        }
        for (actual, expected) in tau_round_trip.iter().zip(generalized_forces.iter()) {
            assert_abs_diff_eq!(*actual, *expected, epsilon = 1.0e-10);
        }
    }

    #[test]
    fn free_flyer_derivative_matches_rigid_body_no_offset_equations() {
        let mass_properties = MassProperties::with_uniform_inertia(
            Mass::new::<kilogram>(2.0),
            Position3::<Body>::new(0.0, 0.0, 0.0),
            2.0,
        );
        let tree = MultibodyTree::new(vec![TreeBodySpec {
            id: BodyId::new(1),
            parent: None,
            joint: Joint::FreeFlyer,
            inertia: SpatialInertia::from_mass_properties(&mass_properties).unwrap(),
            parent_to_body: PluckerTransform::identity(),
        }])
        .unwrap();
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![1.0, 0.0, 0.0, 0.0, 100.0, -200.0, 300.0],
            vec![2.0, 4.0, 6.0, 0.0, 0.0, 0.0],
        );
        let derivative = tree
            .derivative_from_root_free_flyer_loads(
                &state,
                Vector3::new(4.0, 8.0, 12.0),
                Vector3::new(14.0, 16.0, 18.0),
                SpatialMotion::zero(),
                &[SpatialForce::zero()],
            )
            .unwrap();
        let expected_rigid = RigidBodyDerivative::new(
            Vector3::zeros(),
            Vector3::new(7.0, 8.0, 9.0),
            NalgebraQuaternion::new(0.0, 1.0, 2.0, 3.0),
            Vector3::new(2.0, 4.0, 6.0),
            0.0,
            Vector3::zeros(),
            Matrix3::zeros(),
        );

        assert_eq!(
            derivative.q_dot()[0].to_bits(),
            expected_rigid.quaternion_rate.w.to_bits()
        );
        assert_eq!(
            derivative.q_dot()[1].to_bits(),
            expected_rigid.quaternion_rate.i.to_bits()
        );
        assert_eq!(
            derivative.q_dot()[2].to_bits(),
            expected_rigid.quaternion_rate.j.to_bits()
        );
        assert_eq!(
            derivative.q_dot()[3].to_bits(),
            expected_rigid.quaternion_rate.k.to_bits()
        );
        for axis in 0..3 {
            assert_eq!(
                derivative.q_dot()[4 + axis].to_bits(),
                expected_rigid.velocity_m_s_eci[axis].to_bits()
            );
            assert_eq!(
                derivative.qd_dot()[axis].to_bits(),
                expected_rigid.angular_acceleration_rad_s2_body[axis].to_bits()
            );
            assert_eq!(
                derivative.qd_dot()[3 + axis].to_bits(),
                expected_rigid.acceleration_m_s2_eci[axis].to_bits()
            );
        }
    }

    #[test]
    fn root_free_flyer_load_adapter_rotates_parent_force_and_zeros_other_joints() {
        let spherical = TreeBodySpec {
            id: BodyId::new(2),
            parent: Some(BodyIndex::new(0)),
            joint: Joint::Spherical,
            inertia: inertia(),
            parent_to_body: PluckerTransform::identity(),
        };
        let tree = MultibodyTree::new(vec![
            TreeBodySpec {
                id: BodyId::new(1),
                parent: None,
                joint: Joint::FreeFlyer,
                inertia: inertia(),
                parent_to_body: PluckerTransform::identity(),
            },
            spherical,
        ])
        .unwrap();
        let inv_sqrt_2 = 1.0 / <f64 as nalgebra::ComplexField>::sqrt(2.0);
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![
                inv_sqrt_2, 0.0, 0.0, inv_sqrt_2, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            ],
            vec![0.0; tree.n_qd()],
        );

        let generalized_forces = tree
            .root_free_flyer_generalized_forces_from_loads(
                &state,
                Vector3::new(1.0, 2.0, 3.0),
                Vector3::new(4.0, 5.0, 6.0),
            )
            .unwrap();

        assert_eq!(&generalized_forces[0..3], &[1.0, 2.0, 3.0]);
        assert_abs_diff_eq!(generalized_forces[3], 5.0, epsilon = 1.0e-14);
        assert_abs_diff_eq!(generalized_forces[4], -4.0, epsilon = 1.0e-14);
        assert_abs_diff_eq!(generalized_forces[5], 6.0, epsilon = 1.0e-14);
        assert_eq!(&generalized_forces[6..], &[0.0, 0.0, 0.0]);
    }

    #[test]
    fn forward_dynamics_aba_rejects_bad_inputs_and_singular_blocks() {
        let tree = sample_tree();
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![1.0, 0.0, 0.0, 0.0, 0.2, -0.1, 0.3, 0.4, 0.15],
            vec![0.0; tree.n_qd()],
        );
        let external_forces = vec![SpatialForce::zero(); tree.bodies().len()];
        let short_force = tree
            .forward_dynamics_aba_at_state(
                &state,
                &vec![0.0; tree.n_qd() - 1],
                SpatialMotion::zero(),
                &external_forces,
            )
            .unwrap_err();
        let short_external = tree
            .forward_dynamics_aba_at_state(
                &state,
                &vec![0.0; tree.n_qd()],
                SpatialMotion::zero(),
                &external_forces[0..external_forces.len() - 1],
            )
            .unwrap_err();

        let singular_tree = MultibodyTree::new(vec![TreeBodySpec {
            id: BodyId::new(1),
            parent: None,
            joint: Joint::FreeFlyer,
            inertia: SpatialInertia::from_matrix(SpatialMatrix::zeros()).unwrap(),
            parent_to_body: PluckerTransform::identity(),
        }])
        .unwrap();
        let singular_state = MultibodyState::new(
            SimTime::ZERO,
            vec![1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            vec![0.0; singular_tree.n_qd()],
        );
        let singular = singular_tree
            .forward_dynamics_aba_at_state(
                &singular_state,
                &vec![0.0; singular_tree.n_qd()],
                SpatialMotion::zero(),
                &[SpatialForce::zero()],
            )
            .unwrap_err();

        assert!(matches!(
            short_force,
            MultibodyError::GeneralizedVectorDimensionMismatch {
                vector: "generalized forces",
                ..
            }
        ));
        assert!(matches!(
            short_external,
            MultibodyError::GeneralizedVectorDimensionMismatch {
                vector: "external forces",
                ..
            }
        ));
        assert!(matches!(singular, MultibodyError::SingularSystem { .. }));
    }

    #[test]
    fn forward_dynamics_dense_rejects_bad_generalized_forces() {
        let tree = sample_tree();
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![1.0, 0.0, 0.0, 0.0, 0.2, -0.1, 0.3, 0.4, 0.15],
            vec![0.0; tree.n_qd()],
        );
        let external_forces = vec![SpatialForce::zero(); tree.bodies().len()];
        let short = tree
            .forward_dynamics_dense_at_state(
                &state,
                &vec![0.0; tree.n_qd() - 1],
                SpatialMotion::zero(),
                &external_forces,
            )
            .unwrap_err();
        let mut nonfinite = vec![0.0; tree.n_qd()];
        nonfinite[0] = f64::NAN;
        let nonfinite_err = tree
            .forward_dynamics_dense_at_state(
                &state,
                &nonfinite,
                SpatialMotion::zero(),
                &external_forces,
            )
            .unwrap_err();

        assert!(matches!(
            short,
            MultibodyError::GeneralizedVectorDimensionMismatch {
                vector: "generalized forces",
                ..
            }
        ));
        assert!(matches!(nonfinite_err, MultibodyError::NonFinite { .. }));
    }

    #[test]
    fn forward_dynamics_dense_rejects_singular_inertia() {
        let tree = MultibodyTree::new(vec![TreeBodySpec {
            id: BodyId::new(1),
            parent: None,
            joint: Joint::FreeFlyer,
            inertia: SpatialInertia::from_matrix(SpatialMatrix::zeros()).unwrap(),
            parent_to_body: PluckerTransform::identity(),
        }])
        .unwrap();
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            vec![0.0; tree.n_qd()],
        );
        let err = tree
            .forward_dynamics_dense_at_state(
                &state,
                &vec![0.0; tree.n_qd()],
                SpatialMotion::zero(),
                &[SpatialForce::zero()],
            )
            .unwrap_err();

        assert!(matches!(err, MultibodyError::SingularSystem { .. }));
    }

    #[test]
    fn rnea_fixed_transforms_rejects_bad_generalized_acceleration() {
        let tree = sample_tree();

        let short = tree
            .inverse_dynamics_rnea_fixed_transforms(&vec![0.0; tree.n_qd() - 1])
            .unwrap_err();
        let mut nonfinite = vec![0.0; tree.n_qd()];
        nonfinite[0] = f64::NAN;
        let nonfinite_err = tree
            .inverse_dynamics_rnea_fixed_transforms(&nonfinite)
            .unwrap_err();

        assert!(matches!(
            short,
            MultibodyError::GeneralizedVectorDimensionMismatch { .. }
        ));
        assert!(matches!(nonfinite_err, MultibodyError::NonFinite { .. }));
    }
}
