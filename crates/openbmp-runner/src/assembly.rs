//! Phase-3.3 scenario → [`openbmp_vehicle::Assembly`] resolver.
//!
//! Bridges a parsed [`openbmp_scenario::ScenarioDocument`] to the
//! `openbmp-vehicle` assembly tree.
//!
//! Schema v2 requires `[vehicle.assembly]`; this resolver builds an
//! [`Assembly`] from the declared bodies and supplies the dry mass
//! properties used during kernel construction. Propulsion, effectors,
//! tanks, and recovery devices are assembled by their runner-side
//! racks from the child blocks declared under the same assembly tree.

use nalgebra::Matrix3;
use openbmp_core::{BodyId, SimTime, VehicleId};
use openbmp_scenario::{BodyGeometryConfig, ScenarioDocument};
use openbmp_state::MassProperties;
use openbmp_vehicle::{Assembly, Body, BodyGeometry, VehicleAssembly};

use crate::error::RunnerError;

/// Build the runtime [`Assembly`] from the parsed scenario.
///
/// Build the runtime [`Assembly`] from the parsed
/// `[vehicle.assembly]` declaration.
///
/// # Errors
///
/// Returns [`RunnerError::Scenario`] when the assembly fails
/// `Assembly` construction (invalid geometry, asymmetric
/// inertia, duplicate body id, etc.).
pub fn synthesize_assembly(document: &ScenarioDocument) -> Result<Assembly, RunnerError> {
    let vehicle_id = scenario_vehicle_id(document);
    let assembly = &document.vehicle.assembly;
    let mut builder = Assembly::builder(vehicle_id);
    for (index, config) in assembly.bodies.iter().enumerate() {
        let body_id = BodyId::from_path(&format!("vehicle.assembly.bodies.{id}", id = config.id));
        let geometry = body_geometry_from_config(&config.geometry);
        let cg_body = openbmp_core::Position3::new(
            config.dry_cg_body_m[0],
            config.dry_cg_body_m[1],
            config.dry_cg_body_m[2],
        );
        let inertia = config
            .dry_inertia_body_kg_m2
            .map_or_else(default_inertia, matrix_from_rows);
        let body =
            Body::new(body_id, geometry, config.dry_mass_kg, cg_body, inertia).map_err(|err| {
                RunnerError::Assembly {
                    field: format!("vehicle.assembly.bodies[{index}]"),
                    reason: err.to_string(),
                }
            })?;
        builder = builder.add_body(body).map_err(|err| RunnerError::Assembly {
            field: format!("vehicle.assembly.bodies[{index}]"),
            reason: err.to_string(),
        })?;
    }
    builder.build().map_err(|err| RunnerError::Assembly {
        field: "vehicle.assembly".to_owned(),
        reason: err.to_string(),
    })
}

/// Return the assembly's dry mass properties at `time`.
///
/// Phase-3.3 assemblies are dry/static, but threading the time through
/// this helper keeps the runner shape aligned with later engine/tank
/// mass-property models.
///
/// # Errors
///
/// Returns [`RunnerError::Assembly`] if the resolved assembly cannot
/// produce dry mass properties.
pub fn dry_mass_properties_at(
    assembly: &Assembly,
    time: SimTime,
    field: &str,
) -> Result<MassProperties, RunnerError> {
    assembly
        .mass_properties(time)
        .map_err(|err| RunnerError::Assembly {
            field: field.to_owned(),
            reason: err.to_string(),
        })
}

/// Return the assembly dry mass in kilograms at `time`.
///
/// # Errors
///
/// Returns [`RunnerError::Assembly`] if the assembly is structurally
/// empty. Normal scenario resolution rejects that earlier.
pub fn dry_mass_kg_at(assembly: &Assembly, _time: SimTime, field: &str) -> Result<f64, RunnerError> {
    let bodies = assembly.bodies();
    if bodies.is_empty() {
        return Err(RunnerError::Assembly {
            field: field.to_owned(),
            reason: "assembly must contain at least one body".to_owned(),
        });
    }
    if bodies.len() == 1 {
        return Ok(bodies[0].dry_mass_kg());
    }
    let mut total_mass_kg = 0.0_f64;
    for body in bodies {
        total_mass_kg += body.dry_mass_kg();
    }
    Ok(total_mass_kg)
}

fn scenario_vehicle_id(document: &ScenarioDocument) -> VehicleId {
    if let Some(id) = &document.vehicle.assembly.id {
        return VehicleId::from_path(&format!("vehicle.assembly.{id}"));
    }
    VehicleId::from_path(&format!("scenario.{}", document.meta.name))
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
