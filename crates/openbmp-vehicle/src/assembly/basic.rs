//! Concrete [`BasicAssembly`] — flat-tree [`crate::assembly::VehicleAssembly`] impl.
//!
//! Phase-3.3 ships the single-body and multi-body summation cases;
//! propulsion / effectors / tanks / sensors are reserved slots
//! materialising in 3.4 / 3.6 / 3.7 / 3.10.

use nalgebra::{Matrix3, Vector3};
use openbmp_core::{Body as BodyFrame, BodyId, Position3, SimTime, VehicleId};
use openbmp_state::MassProperties;
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

use crate::assembly::body::Body;
use crate::assembly::{AssemblyError, VehicleAssembly};
use crate::error::VehicleError;

/// Flat-tree [`VehicleAssembly`] implementation. Holds the bodies in
/// scenario-declared order; future phases extend with effectors /
/// engines / tanks / sensors.
#[derive(Clone, Debug)]
pub struct BasicAssembly {
    id: VehicleId,
    bodies: Vec<Body>,
}

impl BasicAssembly {
    /// Begin building a new assembly. Use [`BasicAssemblyBuilder::add_body`]
    /// to append bodies in scenario-declared order.
    #[must_use]
    pub fn builder(id: VehicleId) -> BasicAssemblyBuilder {
        BasicAssemblyBuilder::new(id)
    }

    /// Construct a single-body assembly directly from the legacy
    /// flat `[vehicle]` block (used by the resolver when no
    /// `[vehicle.assembly]` block is declared).
    ///
    /// `inertia_body_kg_m2` is `None` for point-mass scenarios; in
    /// that case a unit identity tensor is used as a structural
    /// placeholder. For rigid-body scenarios the inertia must be
    /// provided.
    ///
    /// # Errors
    ///
    /// Returns [`AssemblyError`] for invalid inputs.
    pub fn single_body_legacy(
        vehicle_id: VehicleId,
        body_id: BodyId,
        dry_mass_kg: f64,
        inertia_body_kg_m2: Option<Matrix3<f64>>,
        reference_geometry: crate::assembly::BodyGeometry,
    ) -> Result<Self, AssemblyError> {
        let inertia = inertia_body_kg_m2
            .unwrap_or_else(|| Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, 1.0)));
        let body = Body::new(
            body_id,
            reference_geometry,
            dry_mass_kg,
            Position3::origin(),
            inertia,
        )?;
        Self::builder(vehicle_id).add_body(body)?.build()
    }
}

impl VehicleAssembly for BasicAssembly {
    fn id(&self) -> VehicleId {
        self.id
    }

    fn bodies(&self) -> &[Body] {
        &self.bodies
    }

    fn mass_properties(&self, _t: SimTime) -> Result<MassProperties, VehicleError> {
        // Phase-3.3: dry mass-properties only (no propellant burn at
        // the body level — that lives on the motor adapter wired
        // separately by the resolver). Single-body fast path matches
        // the legacy direct-mass-model behaviour.
        if self.bodies.len() == 1 {
            return Ok(single_body_mass_properties(&self.bodies[0]));
        }
        Ok(sum_bodies_mass_properties(&self.bodies))
    }
}

/// Single-body mass properties: just lift the body's dry components
/// into [`MassProperties`].
fn single_body_mass_properties(body: &Body) -> MassProperties {
    MassProperties::new(
        Mass::new::<kilogram>(body.dry_mass_kg()),
        *body.dry_cg_body(),
        *body.dry_inertia_body(),
    )
}

/// Multi-body mass properties: sum masses, compute the mass-weighted
/// CG, then transport each body's inertia tensor to the assembly CG
/// via the parallel-axis theorem and sum.
///
/// Iteration is in scenario-declared order with a locked left-fold
/// summation; reordering `[[vehicle.assembly.bodies]]` changes byte
/// output by design.
fn sum_bodies_mass_properties(bodies: &[Body]) -> MassProperties {
    let mut total_mass = 0.0_f64;
    let mut weighted_cg = Vector3::<f64>::zeros();
    for body in bodies {
        let m = body.dry_mass_kg();
        total_mass += m;
        weighted_cg += body.dry_cg_body().vector * m;
    }
    let assembly_cg_vec = weighted_cg / total_mass;
    let mut assembly_inertia = Matrix3::<f64>::zeros();
    for body in bodies {
        let m = body.dry_mass_kg();
        let r = body.dry_cg_body().vector - assembly_cg_vec;
        let r_outer = r * r.transpose();
        let r_dot_r = r.dot(&r);
        let identity = Matrix3::<f64>::identity();
        // Parallel-axis transport: I_assembly += I_body + m * (|r|² · I − r·rᵀ)
        let parallel_axis = identity * r_dot_r - r_outer;
        assembly_inertia += body.dry_inertia_body() + parallel_axis * m;
    }
    MassProperties::new(
        Mass::new::<kilogram>(total_mass),
        Position3::<BodyFrame>::new(assembly_cg_vec.x, assembly_cg_vec.y, assembly_cg_vec.z),
        assembly_inertia,
    )
}

/// Builder for [`BasicAssembly`].
#[derive(Debug)]
pub struct BasicAssemblyBuilder {
    id: VehicleId,
    bodies: Vec<Body>,
}

impl BasicAssemblyBuilder {
    fn new(id: VehicleId) -> Self {
        Self {
            id,
            bodies: Vec::new(),
        }
    }

    /// Append a body. Bodies are stored in insertion order.
    ///
    /// # Errors
    ///
    /// Returns [`AssemblyError::DuplicateBody`] when a body with the
    /// same id is already present.
    pub fn add_body(mut self, body: Body) -> Result<Self, AssemblyError> {
        let id = body.id();
        if self.bodies.iter().any(|existing| existing.id() == id) {
            return Err(AssemblyError::DuplicateBody { id });
        }
        self.bodies.push(body);
        Ok(self)
    }

    /// Finalise into [`BasicAssembly`].
    ///
    /// # Errors
    ///
    /// Returns [`AssemblyError::EmptyBodies`] when no bodies were
    /// added.
    pub fn build(self) -> Result<BasicAssembly, AssemblyError> {
        if self.bodies.is_empty() {
            return Err(AssemblyError::EmptyBodies);
        }
        Ok(BasicAssembly {
            id: self.id,
            bodies: self.bodies,
        })
    }
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::float_cmp,
    clippy::panic,
    clippy::unwrap_used
)]
mod tests {
    use super::*;
    use crate::assembly::body::BodyGeometry;
    use approx::assert_abs_diff_eq;

    fn unit_inertia() -> Matrix3<f64> {
        Matrix3::from_diagonal(&Vector3::new(0.01, 0.01, 0.001))
    }

    fn make_body(id: &str, mass_kg: f64, cg: Position3<BodyFrame>) -> Body {
        Body::new(
            BodyId::from_path(id),
            BodyGeometry::Cylinder {
                length_m: 1.0,
                diameter_m: 0.1,
            },
            mass_kg,
            cg,
            unit_inertia(),
        )
        .expect("ok body")
    }

    #[test]
    fn empty_assembly_rejected() {
        let err = BasicAssembly::builder(VehicleId::from_path("v"))
            .build()
            .unwrap_err();
        assert!(matches!(err, AssemblyError::EmptyBodies));
    }

    #[test]
    fn duplicate_body_id_rejected() {
        let err = BasicAssembly::builder(VehicleId::from_path("v"))
            .add_body(make_body("b", 1.0, Position3::origin()))
            .unwrap()
            .add_body(make_body("b", 2.0, Position3::origin()))
            .unwrap_err();
        assert!(matches!(err, AssemblyError::DuplicateBody { .. }));
    }

    #[test]
    fn single_body_assembly_lifts_mass_properties() {
        let assembly = BasicAssembly::builder(VehicleId::from_path("v"))
            .add_body(make_body("only", 0.080, Position3::origin()))
            .unwrap()
            .build()
            .unwrap();
        let props = assembly.mass_properties(SimTime::ZERO).unwrap();
        assert_abs_diff_eq!(props.mass.get::<kilogram>(), 0.080);
    }

    #[test]
    fn two_body_assembly_sums_masses() {
        let assembly = BasicAssembly::builder(VehicleId::from_path("v"))
            .add_body(make_body("main", 0.080, Position3::origin()))
            .unwrap()
            .add_body(make_body("fairing", 0.005, Position3::new(0.0, 0.0, 0.6)))
            .unwrap()
            .build()
            .unwrap();
        let props = assembly.mass_properties(SimTime::ZERO).unwrap();
        assert_abs_diff_eq!(props.mass.get::<kilogram>(), 0.085);
        // Mass-weighted CG: (0.080·0 + 0.005·0.6) / 0.085 ≈ 0.0353 m
        let expected_cg_z = (0.080 * 0.0 + 0.005 * 0.6) / 0.085;
        assert_abs_diff_eq!(
            props.center_of_mass_body.vector.z,
            expected_cg_z,
            epsilon = 1e-12
        );
    }

    #[test]
    fn body_declaration_order_changes_summed_cg_byte_output() {
        let body_main = make_body("main", 0.080, Position3::origin());
        let body_fairing = make_body("fairing", 0.005, Position3::new(0.0, 0.0, 0.6));
        let forward = BasicAssembly::builder(VehicleId::from_path("v"))
            .add_body(body_main.clone())
            .unwrap()
            .add_body(body_fairing.clone())
            .unwrap()
            .build()
            .unwrap();
        let reversed = BasicAssembly::builder(VehicleId::from_path("v"))
            .add_body(body_fairing)
            .unwrap()
            .add_body(body_main)
            .unwrap()
            .build()
            .unwrap();
        let f_props = forward.mass_properties(SimTime::ZERO).unwrap();
        let r_props = reversed.mass_properties(SimTime::ZERO).unwrap();
        // Mathematically equal; bit-pattern may differ due to float
        // sum order. We assert the bit pattern matches when there
        // are only two bodies (commutative for two terms with the
        // same denominator).
        assert_eq!(
            f_props.mass.get::<kilogram>().to_bits(),
            r_props.mass.get::<kilogram>().to_bits()
        );
    }

    #[test]
    fn legacy_single_body_constructor_round_trips() {
        let assembly = BasicAssembly::single_body_legacy(
            VehicleId::from_path("v"),
            BodyId::from_path("vehicle"),
            0.080,
            Some(unit_inertia()),
            BodyGeometry::Cylinder {
                length_m: 0.56,
                diameter_m: 0.029,
            },
        )
        .unwrap();
        assert_eq!(assembly.bodies().len(), 1);
        let props = assembly.mass_properties(SimTime::ZERO).unwrap();
        assert_abs_diff_eq!(props.mass.get::<kilogram>(), 0.080);
    }

    #[test]
    fn legacy_single_body_uses_identity_inertia_when_unset() {
        // Point-mass scenarios pass `None`; the placeholder identity
        // tensor is structurally valid (positive diagonal, symmetric)
        // so the kernel's mass-property check passes.
        let assembly = BasicAssembly::single_body_legacy(
            VehicleId::from_path("v"),
            BodyId::from_path("vehicle"),
            1.0,
            None,
            BodyGeometry::Reference {
                length_m: 1.0,
                area_m2: 1.0,
            },
        )
        .unwrap();
        let props = assembly.mass_properties(SimTime::ZERO).unwrap();
        for diag in 0..3 {
            assert!(props.inertia_body[(diag, diag)] > 0.0);
        }
    }
}
