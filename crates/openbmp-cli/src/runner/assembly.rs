//! Phase-3.3 scenario → [`openbmp_vehicle::BasicAssembly`] resolver.
//!
//! Bridges a parsed [`openbmp_scenario::ScenarioDocument`] to the
//! `openbmp-vehicle` assembly tree. Two paths:
//!
//! - **`[vehicle.assembly]` declared** — build a [`BasicAssembly`]
//!   from the declared bodies.
//! - **No assembly declared (legacy)** — synthesise a single-body
//!   `BasicAssembly` from `vehicle.mass_kg` and (if present)
//!   `inertia_tensor_body_kg_m2`. The single-body fast path
//!   preserves byte-identical kernel construction for every
//!   Phase-2.10 / 3.1 scenario.
//!
//! Phase-3.3 leaves the runner's existing `build_vehicle` /
//! `build_mass_model` paths in place — the resolver's output is
//! validated but does not yet drive kernel construction. The actual
//! multi-body mass-property propagation lands with the multi-body
//! scenario fixture in 3.3.D.

use nalgebra::Matrix3;
use openbmp_core::{BodyId, VehicleId};
use openbmp_scenario::{BodyGeometryConfig, ScenarioDocument};
use openbmp_vehicle::{BasicAssembly, Body, BodyGeometry};

use crate::error::CliError;

/// Build the runtime [`BasicAssembly`] from the parsed scenario.
///
/// When the scenario declares `[vehicle.assembly]`, the resolver
/// uses it directly. Otherwise, it synthesises a one-body assembly
/// from the legacy flat `[vehicle]` block.
///
/// # Errors
///
/// Returns [`CliError::Scenario`] when the assembly fails
/// `BasicAssembly` construction (invalid geometry, asymmetric
/// inertia, duplicate body id, etc.).
pub fn synthesize_assembly(document: &ScenarioDocument) -> Result<BasicAssembly, CliError> {
    let vehicle_id = scenario_vehicle_id(document);
    if let Some(assembly) = &document.vehicle.assembly {
        let mut builder = BasicAssembly::builder(vehicle_id);
        for (index, config) in assembly.bodies.iter().enumerate() {
            let body_id =
                BodyId::from_path(&format!("vehicle.assembly.bodies.{id}", id = config.id));
            let geometry = body_geometry_from_config(&config.geometry);
            let cg_body = openbmp_core::Position3::new(
                config.dry_cg_body_m[0],
                config.dry_cg_body_m[1],
                config.dry_cg_body_m[2],
            );
            let inertia = config
                .dry_inertia_body_kg_m2
                .map_or_else(default_inertia, matrix_from_rows);
            let body = Body::new(body_id, geometry, config.dry_mass_kg, cg_body, inertia).map_err(
                |err| {
                    CliError::Scenario(openbmp_scenario::ScenarioError::InvalidNumber {
                        field: format!("vehicle.assembly.bodies[{index}]"),
                        value: f64::NAN,
                        rule: leak_static_str(format!("{err}")),
                    })
                },
            )?;
            builder = builder.add_body(body).map_err(|err| {
                CliError::Scenario(openbmp_scenario::ScenarioError::InvalidNumber {
                    field: format!("vehicle.assembly.bodies[{index}]"),
                    value: f64::NAN,
                    rule: leak_static_str(format!("{err}")),
                })
            })?;
        }
        builder.build().map_err(|err| {
            CliError::Scenario(openbmp_scenario::ScenarioError::InvalidNumber {
                field: "vehicle.assembly".to_owned(),
                value: f64::NAN,
                rule: leak_static_str(format!("{err}")),
            })
        })
    } else {
        synthesize_legacy_single_body(document, vehicle_id)
    }
}

fn scenario_vehicle_id(document: &ScenarioDocument) -> VehicleId {
    if let Some(assembly) = &document.vehicle.assembly
        && let Some(id) = &assembly.id
    {
        return VehicleId::from_path(&format!("vehicle.assembly.{id}"));
    }
    VehicleId::from_path(&format!("scenario.{}", document.meta.name))
}

fn synthesize_legacy_single_body(
    document: &ScenarioDocument,
    vehicle_id: VehicleId,
) -> Result<BasicAssembly, CliError> {
    let body_id = BodyId::from_path("vehicle.body");
    // Reference geometry: pull from aero deck length / area_m2 if
    // present, else use a unit reference. Phase-3.3 doesn't use the
    // geometry on the kernel hot path, so the placeholder is safe.
    let geometry = BodyGeometry::Reference {
        length_m: 1.0,
        area_m2: 1.0,
    };
    let inertia = document
        .vehicle
        .inertia_tensor_body_kg_m2
        .map_or_else(default_inertia, matrix_from_rows);
    BasicAssembly::single_body_legacy(
        vehicle_id,
        body_id,
        document.vehicle.mass_kg,
        Some(inertia),
        geometry,
    )
    .map_err(|err| {
        CliError::Scenario(openbmp_scenario::ScenarioError::InvalidNumber {
            field: "vehicle (legacy synthesis)".to_owned(),
            value: f64::NAN,
            rule: leak_static_str(format!("{err}")),
        })
    })
}

fn body_geometry_from_config(config: &BodyGeometryConfig) -> BodyGeometry {
    match config {
        BodyGeometryConfig::Cylinder {
            length_m,
            diameter_m,
        } => BodyGeometry::Cylinder {
            length_m: *length_m,
            diameter_m: *diameter_m,
        },
        BodyGeometryConfig::Cone {
            length_m,
            base_diameter_m,
        } => BodyGeometry::Cone {
            length_m: *length_m,
            base_diameter_m: *base_diameter_m,
        },
        BodyGeometryConfig::Reference { length_m, area_m2 } => BodyGeometry::Reference {
            length_m: *length_m,
            area_m2: *area_m2,
        },
    }
}

fn matrix_from_rows(rows: [[f64; 3]; 3]) -> Matrix3<f64> {
    Matrix3::new(
        rows[0][0], rows[0][1], rows[0][2], rows[1][0], rows[1][1], rows[1][2], rows[2][0],
        rows[2][1], rows[2][2],
    )
}

fn default_inertia() -> Matrix3<f64> {
    Matrix3::from_diagonal(&nalgebra::Vector3::new(1.0, 1.0, 1.0))
}

/// `ScenarioError::InvalidNumber.rule` is `&'static str`; assembly
/// errors carry runtime strings. Leak them so the static-str
/// requirement is satisfied without changing the error shape. Used
/// only on the parse-error path where an extra allocation is
/// negligible.
fn leak_static_str(s: String) -> &'static str {
    Box::leak(s.into_boxed_str())
}
