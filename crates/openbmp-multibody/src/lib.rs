//! `openbmp-multibody` — spatial-vector multibody dynamics substrate.
//!
//! This crate is the L2 home for the flexible/articulated multibody program.
//! It deliberately depends only on lower L0/L1 crates plus `nalgebra`: no
//! simulator, runner, scenario, or flight-controller dependency is allowed here.
//! The first slice provides the deterministic spatial algebra, rigid spatial
//! inertia construction, joint subspace vocabulary, and topological tree shape
//! that the later ABA/RNEA/CRBA implementation consumes.
//!
//! Spatial vectors use Featherstone's angular-over-linear convention:
//! motion `v = [omega; v_linear]`, force `f = [moment; force_linear]`.

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

extern crate alloc;

use alloc::vec;
use alloc::vec::Vec;

use nalgebra::{Matrix3, SMatrix, SVector, Vector3};
use openbmp_core::{BodyId, SimTime};
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

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use nalgebra::UnitQuaternion;
    use openbmp_core::{Body, Position3};
    use uom::si::f64::Mass;
    use uom::si::mass::kilogram;

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
}
