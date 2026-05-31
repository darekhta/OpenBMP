//! Regression coverage for atmosphere telemetry sampling in geocentric
//! coordinates.

use openbmp_telemetry::TelemetryValue;

fn atmosphere_density_column(outcome: &openbmp_runner::RunOutcome) -> Vec<f64> {
    let channel = outcome
        .table
        .schema()
        .channels()
        .iter()
        .find(|channel| channel.name == "atmosphere.density_kg_m3")
        .expect("atmosphere density channel must exist")
        .id;

    outcome
        .table
        .rows()
        .iter()
        .map(|row| match row.get(channel) {
            Some(TelemetryValue::Float64(value)) => *value,
            other => panic!("unexpected atmosphere density value: {other:?}"),
        })
        .collect()
}

fn run_scenario(toml: &str) -> openbmp_runner::RunOutcome {
    let scenario = openbmp_scenario::Scenario::from_toml_str(toml).expect("scenario parses");
    openbmp_runner::run(&scenario).expect("scenario runs")
}

#[test]
fn point_mass_atmosphere_telemetry_uses_geocentric_altitude() {
    let outcome = run_scenario(
        r#"
openbmp.scenario = 3

[meta]
name = "point-mass-geocentric-atmosphere-telemetry"
description = "Synthetic geocentric atmosphere telemetry regression."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.1
dt_s = 0.1
seed = 12

[vehicle]
kind = "point_mass"
initial_position_eci_m = [6471000.0, 0.0, 1.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "point-mass-geocentric-atmosphere-telemetry"

[[vehicle.assembly.bodies]]
id = "mass"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 0.0
atmosphere = "piecewise_exponential"
wind = "none"

[atmosphere]
kind = "piecewise_exponential"

[forces]
models = ["gravity"]

[telemetry]
output.csv = "out/point-mass-geocentric-atmosphere-telemetry.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#,
    );

    let densities = atmosphere_density_column(&outcome);
    assert!(
        densities[0] < 1.0e-5,
        "geocentric position at 100 km altitude must not sample near-sea-level density: {:?}",
        densities
    );
}

#[test]
fn rigid_body_atmosphere_telemetry_uses_geocentric_altitude() {
    let outcome = run_scenario(
        r#"
openbmp.scenario = 3

[meta]
name = "rigid-body-geocentric-atmosphere-telemetry"
description = "Synthetic rigid-body geocentric atmosphere telemetry regression."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.1
dt_s = 0.1
seed = 13

[vehicle]
kind = "rigid_body"
initial_position_eci_m = [6471000.0, 0.0, 1.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]
initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
initial_angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "rigid-body-geocentric-atmosphere-telemetry"

[[vehicle.assembly.bodies]]
id = "body"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 0.0
atmosphere = "piecewise_exponential"
wind = "none"

[atmosphere]
kind = "piecewise_exponential"

[forces]
models = ["gravity"]

[telemetry]
output.csv = "out/rigid-body-geocentric-atmosphere-telemetry.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#,
    );

    let densities = atmosphere_density_column(&outcome);
    assert!(
        densities[0] < 1.0e-5,
        "geocentric position at 100 km altitude must not sample near-sea-level density: {:?}",
        densities
    );
}
