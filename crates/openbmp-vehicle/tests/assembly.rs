//! Phase-3.3.A integration tests for the [`VehicleAssembly`] surface.
//!
//! Covers:
//! - Single-body assembly mass-property lift.
//! - Multi-body summation: total mass, mass-weighted CG.
//! - Build-twice determinism: re-constructing the same assembly
//!   produces bit-identical mass-property output.
//! - Body-id collision rejection.

#![allow(
    clippy::expect_used,
    clippy::float_cmp,
    clippy::panic,
    clippy::unwrap_used
)]

use approx::assert_abs_diff_eq;
use nalgebra::{Matrix3, Vector3};
use openbmp_core::{Body as BodyFrame, BodyId, Position3, SimTime, VehicleId};
use openbmp_vehicle::{Assembly, AssemblyError, Body, BodyGeometry, VehicleAssembly};
use uom::si::mass::kilogram;

fn rod_inertia(length_m: f64, mass_kg: f64) -> Matrix3<f64> {
    let i_transverse = mass_kg * length_m * length_m / 12.0;
    Matrix3::from_diagonal(&Vector3::new(i_transverse, i_transverse, 1.0e-6))
}

fn make_body(id: &str, mass_kg: f64, cg: Position3<BodyFrame>, length_m: f64) -> Body {
    Body::new(
        BodyId::from_path(id),
        BodyGeometry::Cylinder {
            length_m,
            diameter_m: 0.029,
        },
        mass_kg,
        cg,
        rod_inertia(length_m, mass_kg),
    )
    .expect("ok body")
}

#[test]
fn single_body_assembly_lifts_mass() {
    let assembly = Assembly::builder(VehicleId::from_path("niskanen-rocket"))
        .add_body(make_body(
            "vehicle.assembly.bodies.main",
            0.080,
            Position3::origin(),
            0.56,
        ))
        .unwrap()
        .build()
        .unwrap();
    let props = assembly.mass_properties(SimTime::ZERO).unwrap();
    assert_abs_diff_eq!(props.mass.get::<kilogram>(), 0.080);
}

#[test]
fn two_body_assembly_sums_mass_and_weights_cg() {
    let assembly = Assembly::builder(VehicleId::from_path("two-body-fairing"))
        .add_body(make_body(
            "vehicle.assembly.bodies.main",
            0.080,
            Position3::origin(),
            0.56,
        ))
        .unwrap()
        .add_body(make_body(
            "vehicle.assembly.bodies.fairing",
            0.005,
            Position3::new(0.0, 0.0, 0.6),
            0.05,
        ))
        .unwrap()
        .build()
        .unwrap();
    let props = assembly.mass_properties(SimTime::ZERO).unwrap();
    assert_abs_diff_eq!(props.mass.get::<kilogram>(), 0.085);
    let expected_cg_z = (0.080 * 0.0 + 0.005 * 0.6) / 0.085;
    assert_abs_diff_eq!(
        props.center_of_mass_body.vector.z,
        expected_cg_z,
        epsilon = 1.0e-12
    );
}

#[test]
fn build_twice_produces_bit_identical_mass_properties() {
    fn build() -> Assembly {
        Assembly::builder(VehicleId::from_path("v"))
            .add_body(make_body("main", 0.080, Position3::origin(), 0.56))
            .unwrap()
            .add_body(make_body(
                "fairing",
                0.005,
                Position3::new(0.0, 0.0, 0.6),
                0.05,
            ))
            .unwrap()
            .build()
            .unwrap()
    }
    let a = build();
    let b = build();
    let a_props = a.mass_properties(SimTime::ZERO).unwrap();
    let b_props = b.mass_properties(SimTime::ZERO).unwrap();
    assert_eq!(
        a_props.mass.get::<kilogram>().to_bits(),
        b_props.mass.get::<kilogram>().to_bits(),
    );
    for axis in 0..3 {
        assert_eq!(
            a_props.center_of_mass_body.vector[axis].to_bits(),
            b_props.center_of_mass_body.vector[axis].to_bits(),
        );
    }
    for row in 0..3 {
        for col in 0..3 {
            assert_eq!(
                a_props.inertia_body[(row, col)].to_bits(),
                b_props.inertia_body[(row, col)].to_bits(),
            );
        }
    }
}

#[test]
fn duplicate_body_id_rejected() {
    let err = Assembly::builder(VehicleId::from_path("v"))
        .add_body(make_body("main", 0.080, Position3::origin(), 0.56))
        .unwrap()
        .add_body(make_body("main", 0.005, Position3::origin(), 0.05))
        .unwrap_err();
    assert!(matches!(err, AssemblyError::DuplicateBody { .. }));
}

#[test]
fn empty_assembly_rejected() {
    let err = Assembly::builder(VehicleId::from_path("v"))
        .build()
        .unwrap_err();
    assert!(matches!(err, AssemblyError::EmptyBodies));
}
