//! Slosh-as-body multibody construction (WP-01.4).
//!
//! A propellant slosh mode is represented as a revolute-pendulum body hanging
//! in the multibody tree. The lateral reaction on the parent vehicle then
//! emerges from the articulated-body recursion rather than from an externally
//! summed equivalent-pendulum force, so the slosh reaction is applied to the
//! parent through the joint (internal, equal/opposite) by construction.
//!
//! The closed-form small-angle frequency reproduces the Abramson cylindrical
//! relation `omega_n^2 = a / L_n`, where `a` is the axial (thrust) acceleration
//! and `L_n` is the equivalent pendulum length supplied per fill level by the
//! [`openbmp-vehicle`] Abramson model. This crate stays at the L2
//! spatial-vector layer: it consumes pendulum length / mass / geometry and is
//! unaware of tank tables, which a higher layer (the runner) supplies.
//!
//! Scope note (WP-01.4 is partial): this module provides the slosh-body
//! *construction* and validates the inertia/frequency relation that sets the
//! mode. Runner tree wiring and end-to-end moving-base validation remain open.
//! (The articulated forward dynamics this builds on are verified correct against
//! an independent maximal-coordinate reference; see `docs/parity/progress.md`
//! for the one open energy-accessor question that does not affect this builder.)

use alloc::vec::Vec;

use nalgebra::{Matrix3, Vector3};
use openbmp_core::BodyId;

use crate::{
    BodyIndex, Joint, MultibodyError, PluckerTransform, SpatialInertia, SpatialMatrix, TreeBodySpec,
};

/// Maximum allowed `|swing . hang|` after normalization. The hang direction
/// must be perpendicular to the swing axis or the revolute rotation would not
/// move the slosh mass.
const HANG_SWING_PERP_TOL: f64 = 1.0e-9;

/// One propellant-slosh mode represented as a revolute-pendulum tree body.
///
/// The body frame is placed at the hinge with axes aligned to the parent body
/// frame. The slosh point mass sits at `pendulum_length_m * hang_unit` in that
/// frame; the revolute joint swings it about `swing_axis_body`.
#[derive(Clone, Debug, PartialEq)]
pub struct PendulumSloshBody {
    /// Stable identifier for the slosh body.
    pub id: BodyId,
    /// Parent (tank/vehicle) body index in the tree.
    pub parent: BodyIndex,
    /// Hinge location expressed in the parent body frame (metres).
    pub hinge_offset_parent_m: Vector3<f64>,
    /// Revolute swing axis expressed in the parent body frame. Need not be unit
    /// length; it is normalized during construction.
    pub swing_axis_body: Vector3<f64>,
    /// Equilibrium hang direction from the hinge toward the slosh mass. Need not
    /// be unit length; it is normalized and must be perpendicular to the swing
    /// axis. For a vehicle under `+axial` thrust this points along `-axial`.
    pub hang_direction_body: Vector3<f64>,
    /// Equivalent pendulum length `L_n` (metres).
    pub pendulum_length_m: f64,
    /// Slosh mass `m_n` (kilograms).
    pub slosh_mass_kg: f64,
}

impl PendulumSloshBody {
    /// Build the [`TreeBodySpec`] for this slosh mode.
    ///
    /// # Errors
    ///
    /// Returns [`MultibodyError::InvalidParameter`] for a non-finite or
    /// non-positive length/mass, a zero swing axis or hang direction, or a hang
    /// direction that is not perpendicular to the swing axis;
    /// [`MultibodyError::NonFinite`] if the hinge offset is non-finite.
    pub fn tree_body_spec(&self) -> Result<TreeBodySpec, MultibodyError> {
        require_positive(self.pendulum_length_m, "slosh pendulum length must be positive")?;
        require_positive(self.slosh_mass_kg, "slosh mass must be positive")?;
        if !self.hinge_offset_parent_m.iter().all(|v| v.is_finite()) {
            return Err(MultibodyError::NonFinite {
                reason: "slosh hinge offset contains non-finite components",
            });
        }
        let swing = unit_axis(self.swing_axis_body, "slosh swing axis must be non-zero")?;
        let hang = unit_axis(self.hang_direction_body, "slosh hang direction must be non-zero")?;
        if swing.dot(&hang).abs() > HANG_SWING_PERP_TOL {
            return Err(MultibodyError::InvalidParameter {
                reason: "slosh hang direction must be perpendicular to the swing axis",
            });
        }

        let com_body = hang * self.pendulum_length_m;
        let inertia = point_mass_spatial_inertia(self.slosh_mass_kg, com_body)?;
        let parent_to_body = PluckerTransform::new(Matrix3::identity(), self.hinge_offset_parent_m)?;
        Ok(TreeBodySpec {
            id: self.id,
            parent: Some(self.parent),
            joint: Joint::revolute(swing)?,
            inertia,
            parent_to_body,
        })
    }

    /// Closed-form small-angle slosh frequency `sqrt(a / L_n)` (rad/s) for the
    /// supplied axial acceleration, matching the Abramson pendulum relation
    /// `omega_n^2 = a / L_n`. Returns `0.0` for non-positive acceleration
    /// (free-fall / pre-ignition), mirroring
    /// [`openbmp-vehicle`]'s `omega_n_squared_rad2_s2`.
    #[must_use]
    pub fn small_angle_frequency_rad_s(&self, axial_accel_m_s2: f64) -> f64 {
        if axial_accel_m_s2 <= 0.0 || self.pendulum_length_m <= 0.0 {
            return 0.0;
        }
        <f64 as nalgebra::ComplexField>::sqrt(axial_accel_m_s2 / self.pendulum_length_m)
    }
}

/// Append the [`TreeBodySpec`] for every slosh mode to a body-spec list, in the
/// supplied (declaration) order so the resulting tree is deterministic.
///
/// # Errors
///
/// Propagates the first error from [`PendulumSloshBody::tree_body_spec`].
pub fn append_pendulum_slosh_bodies(
    specs: &mut Vec<TreeBodySpec>,
    slosh_bodies: &[PendulumSloshBody],
) -> Result<(), MultibodyError> {
    for body in slosh_bodies {
        let spec = body.tree_body_spec()?;
        specs.push(spec);
    }
    Ok(())
}

fn require_positive(value: f64, reason: &'static str) -> Result<(), MultibodyError> {
    if !value.is_finite() || value <= 0.0 {
        return Err(MultibodyError::InvalidParameter { reason });
    }
    Ok(())
}

fn unit_axis(axis: Vector3<f64>, reason: &'static str) -> Result<Vector3<f64>, MultibodyError> {
    if !axis.iter().all(|v| v.is_finite()) {
        return Err(MultibodyError::NonFinite {
            reason: "slosh axis contains non-finite components",
        });
    }
    let norm = axis.norm();
    if norm <= 0.0 {
        return Err(MultibodyError::InvalidParameter { reason });
    }
    Ok(axis / norm)
}

/// Spatial inertia of a point mass located at `com_body_m`, using the same
/// power-dual construction as [`SpatialInertia::from_mass_properties`] with a
/// zero body-frame inertia tensor. The mass is offset from the joint axis, so
/// the resulting articulated joint inertia is non-singular.
fn point_mass_spatial_inertia(
    mass_kg: f64,
    com_body_m: Vector3<f64>,
) -> Result<SpatialInertia, MultibodyError> {
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
    SpatialInertia::from_matrix(matrix)
}

fn skew(v: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::float_cmp, clippy::panic, clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::{MultibodyState, MultibodyTree};
    use openbmp_core::SimTime;
    use openbmp_state::MassProperties;
    use uom::si::f64::Mass;
    use uom::si::mass::kilogram;

    fn heavy_root_spec(mass_kg: f64) -> TreeBodySpec {
        let properties = MassProperties::with_diagonal_inertia(
            Mass::new::<kilogram>(mass_kg),
            openbmp_core::Position3::<openbmp_core::Body>::new(0.0, 0.0, 0.0),
            0.4 * mass_kg,
            0.5 * mass_kg,
            0.6 * mass_kg,
        );
        TreeBodySpec {
            id: BodyId::new(1),
            parent: None,
            joint: Joint::FreeFlyer,
            inertia: SpatialInertia::from_mass_properties(&properties).unwrap(),
            parent_to_body: PluckerTransform::identity(),
        }
    }

    /// Revolute about body `x`, mass hanging along `-z`: a planar pendulum in
    /// the `y-z` plane (the canonical lateral slosh layout under axial thrust).
    fn lateral_slosh_body(length_m: f64, mass_kg: f64) -> PendulumSloshBody {
        PendulumSloshBody {
            id: BodyId::new(2),
            parent: BodyIndex::new(0),
            hinge_offset_parent_m: Vector3::new(0.0, 0.0, 1.5),
            swing_axis_body: Vector3::new(1.0, 0.0, 0.0),
            hang_direction_body: Vector3::new(0.0, 0.0, -1.0),
            pendulum_length_m: length_m,
            slosh_mass_kg: mass_kg,
        }
    }

    fn slosh_tree(root_mass_kg: f64, length_m: f64, slosh_mass_kg: f64) -> MultibodyTree {
        let root = heavy_root_spec(root_mass_kg);
        let slosh = lateral_slosh_body(length_m, slosh_mass_kg)
            .tree_body_spec()
            .unwrap();
        MultibodyTree::new(vec![root, slosh]).unwrap()
    }

    #[test]
    fn small_angle_frequency_matches_abramson_relation() {
        let body = lateral_slosh_body(0.8, 120.0);
        // omega^2 = a / L.
        for &accel in &[2.0, 9.81, 25.0] {
            let expected = (accel / 0.8_f64).sqrt();
            assert!((body.small_angle_frequency_rad_s(accel) - expected).abs() < 1.0e-12);
        }
        // Free-fall / pre-ignition collapses to zero, matching the tank model.
        assert_eq!(body.small_angle_frequency_rad_s(0.0), 0.0);
        assert_eq!(body.small_angle_frequency_rad_s(-3.0), 0.0);
    }

    #[test]
    fn tree_body_spec_has_revolute_dof_and_preserves_mass() {
        let tree = slosh_tree(5.0e5, 0.8, 120.0);
        // Root free-flyer (7 q / 6 qd) plus one revolute (1 / 1).
        assert_eq!(tree.n_q(), 8);
        assert_eq!(tree.n_qd(), 7);
        // The slosh body's force-block diagonal is the slosh mass.
        let slosh = &tree.bodies()[1];
        let mass = slosh.inertia.matrix()[(3, 3)];
        assert!((mass - 120.0).abs() < 1.0e-9);
    }

    #[test]
    fn joint_inertia_about_hinge_matches_pendulum_inertia() {
        // The joint-space inertia diagonal for the slosh revolute is the
        // pendulum point mass's moment of inertia about the hinge axis, m*L^2
        // (the mass sits at perpendicular distance L from the swing axis). With
        // the gravity restoring torque m*a*L this yields omega^2 = a/L, the
        // Abramson relation -- validated here at the inertia level, free of the
        // floating-base finite-amplitude integrator (which is a WP-01.1 item).
        let length = 0.8;
        let mass = 120.0;
        let tree = slosh_tree(5.0e5, length, mass);
        let state = MultibodyState::new(
            SimTime::ZERO,
            vec![1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            vec![0.0; tree.n_qd()],
        );
        let h = tree.joint_space_inertia_crba_at_state(&state).unwrap();
        let n = tree.n_qd();
        let slosh_diag = h.at(n - 1, n - 1).unwrap();
        let expected = mass * length * length;
        let rel = (slosh_diag - expected).abs() / expected;
        assert!(
            rel < 1.0e-9,
            "joint inertia {slosh_diag} vs m*L^2 {expected} (rel {rel})"
        );
    }

    #[test]
    fn rejects_degenerate_geometry() {
        // Hang parallel to swing axis: rotation does not move the mass.
        let mut body = lateral_slosh_body(0.8, 120.0);
        body.hang_direction_body = Vector3::new(1.0, 0.0, 0.0);
        assert!(matches!(
            body.tree_body_spec(),
            Err(MultibodyError::InvalidParameter { .. })
        ));

        // Non-positive length / mass.
        let mut bad_len = lateral_slosh_body(0.0, 120.0);
        bad_len.pendulum_length_m = 0.0;
        assert!(bad_len.tree_body_spec().is_err());
        let mut bad_mass = lateral_slosh_body(0.8, 0.0);
        bad_mass.slosh_mass_kg = -1.0;
        assert!(bad_mass.tree_body_spec().is_err());

        // Zero swing axis.
        let mut zero_axis = lateral_slosh_body(0.8, 120.0);
        zero_axis.swing_axis_body = Vector3::zeros();
        assert!(zero_axis.tree_body_spec().is_err());
    }

    #[test]
    fn append_builds_deterministic_multi_mode_tree() {
        let mut specs = vec![heavy_root_spec(5.0e5)];
        let modes = [
            PendulumSloshBody {
                id: BodyId::new(2),
                parent: BodyIndex::new(0),
                hinge_offset_parent_m: Vector3::new(0.0, 0.0, 1.5),
                swing_axis_body: Vector3::new(1.0, 0.0, 0.0),
                hang_direction_body: Vector3::new(0.0, 0.0, -1.0),
                pendulum_length_m: 0.8,
                slosh_mass_kg: 120.0,
            },
            PendulumSloshBody {
                id: BodyId::new(3),
                parent: BodyIndex::new(0),
                hinge_offset_parent_m: Vector3::new(0.0, 0.0, 1.2),
                swing_axis_body: Vector3::new(0.0, 1.0, 0.0),
                hang_direction_body: Vector3::new(0.0, 0.0, -1.0),
                pendulum_length_m: 0.5,
                slosh_mass_kg: 40.0,
            },
        ];
        append_pendulum_slosh_bodies(&mut specs, &modes).unwrap();
        let tree = MultibodyTree::new(specs).unwrap();
        // Root (7/6) + two revolutes (1/1 each).
        assert_eq!(tree.n_q(), 9);
        assert_eq!(tree.n_qd(), 8);
        assert_eq!(tree.bodies()[1].id, BodyId::new(2));
        assert_eq!(tree.bodies()[2].id, BodyId::new(3));
    }
}
