//! Regression coverage for atmosphere telemetry sampling in geocentric
//! coordinates.

#![allow(clippy::expect_used, clippy::panic)]

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
    let outcome = run_scenario(include_str!(
        "fixtures/point-mass-geocentric-atmosphere-telemetry.toml"
    ));

    let densities = atmosphere_density_column(&outcome);
    assert!(
        densities[0] < 1.0e-5,
        "geocentric position at 100 km altitude must not sample near-sea-level density: {:?}",
        densities
    );
}

#[test]
fn rigid_body_atmosphere_telemetry_uses_geocentric_altitude() {
    let outcome = run_scenario(include_str!(
        "fixtures/rigid-body-geocentric-atmosphere-telemetry.toml"
    ));

    let densities = atmosphere_density_column(&outcome);
    assert!(
        densities[0] < 1.0e-5,
        "geocentric position at 100 km altitude must not sample near-sea-level density: {:?}",
        densities
    );
}

#[test]
fn point_mass_atmosphere_telemetry_uses_declared_launch_radius() {
    let outcome = run_scenario(include_str!(
        "fixtures/point-mass-rounded-radius-atmosphere-telemetry.toml"
    ));

    let densities = atmosphere_density_column(&outcome);
    assert!(
        densities[0] > 1.2,
        "initial rounded-radius launch point should sample sea-level density: {:?}",
        densities
    );
    assert!(
        densities[1] < 1.1,
        "1 km above a rounded 6370 km launch radius must not still sample sea-level density: {:?}",
        densities
    );
}
