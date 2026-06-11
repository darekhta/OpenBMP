//! Regression coverage for scenario-wired ambient pressure thrust.

#![allow(clippy::expect_used, clippy::panic)]

use openbmp_telemetry::TelemetryValue;

fn f64_column(outcome: &openbmp_runner::RunOutcome, name: &str) -> Vec<f64> {
    let channel = outcome
        .table
        .schema()
        .channels()
        .iter()
        .find(|channel| channel.name == name)
        .unwrap_or_else(|| panic!("telemetry channel `{name}` must exist"))
        .id;

    outcome
        .table
        .rows()
        .iter()
        .map(|row| match row.get(channel) {
            Some(TelemetryValue::Float64(value)) => *value,
            other => panic!("unexpected `{name}` telemetry value: {other:?}"),
        })
        .collect()
}

fn run_scenario(toml: &str) -> openbmp_runner::RunOutcome {
    let scenario = openbmp_scenario::Scenario::from_toml_str(toml).expect("scenario parses");
    openbmp_runner::run(&scenario).expect("scenario runs")
}

fn max_positive_thrust_n(outcome: &openbmp_runner::RunOutcome) -> f64 {
    f64_column(outcome, "force.thrust.z_n")
        .into_iter()
        .filter(|value| *value > 0.0)
        .fold(0.0, f64::max)
}

#[test]
fn pressure_thrust_uses_runtime_atmosphere_pressure() {
    let pressure_corrected = run_scenario(include_str!("fixtures/point-mass-pressure-thrust.toml"));
    let constant_nozzle = run_scenario(
        &include_str!("fixtures/point-mass-pressure-thrust.toml").replace(
            r#"ambient_pressure_correction = "pressure_thrust""#,
            r#"ambient_pressure_correction = "constant""#,
        ),
    );

    let pressure_corrected_thrust = max_positive_thrust_n(&pressure_corrected);
    let constant_thrust = max_positive_thrust_n(&constant_nozzle);

    assert!(
        pressure_corrected_thrust > 0.0,
        "pressure-thrust scenario should burn: {pressure_corrected_thrust}"
    );
    assert!(
        pressure_corrected_thrust < constant_thrust,
        "sea-level ambient pressure should reduce this over-expanded synthetic nozzle: \
         pressure-corrected={pressure_corrected_thrust}, constant={constant_thrust}"
    );

    let pressures = f64_column(&pressure_corrected, "atmosphere.pressure_pa");
    assert!(
        pressures.iter().any(|pressure| *pressure > 80_000.0),
        "pressure-thrust runner fixture must sample dense low-altitude pressure: {pressures:?}"
    );
}

#[test]
fn nozzle_separation_clips_overexpanded_pressure_penalty() {
    let pressure_corrected = run_scenario(include_str!("fixtures/point-mass-pressure-thrust.toml"));
    let separated = run_scenario(
        &include_str!("fixtures/point-mass-pressure-thrust.toml").replace(
            r#"ambient_pressure_correction = "pressure_thrust""#,
            r#"ambient_pressure_correction = "pressure_thrust"
separation = "summerfield""#,
        ),
    );

    let pressure_corrected_thrust = max_positive_thrust_n(&pressure_corrected);
    let separated_thrust = max_positive_thrust_n(&separated);

    assert!(
        separated_thrust > pressure_corrected_thrust,
        "separation should clip the sea-level overexpansion penalty: \
         separated={separated_thrust}, pressure-corrected={pressure_corrected_thrust}"
    );
}

#[test]
fn inline_grain_transient_mode_runs_through_runner() {
    let quasi_static = run_scenario(include_str!("fixtures/point-mass-pressure-thrust.toml"));
    let transient = run_scenario(
        &include_str!("fixtures/point-mass-pressure-thrust.toml").replace(
            r#"geometry = "end_burner""#,
            r#"geometry = "end_burner"
mode = "transient""#,
        ),
    );

    let quasi_static_thrust = max_positive_thrust_n(&quasi_static);
    let transient_thrust = max_positive_thrust_n(&transient);

    assert!(transient_thrust > 0.0);
    assert_ne!(quasi_static_thrust.to_bits(), transient_thrust.to_bits());
}

#[test]
fn inline_grain_thermochem_deck_overrides_propellant_constants() {
    let baseline = run_scenario(include_str!("fixtures/point-mass-pressure-thrust.toml"));
    let thermochem = run_scenario(
        &include_str!("fixtures/point-mass-pressure-thrust.toml").replace(
            "[propulsion.nozzle]",
            "[propulsion.thermochem]\n\
             file = \"tests/fixtures/thermochem/synthetic-grain.toml\"\n\
             file_sha256 = \"01984e1e1122f7ceba2859a6c7a2b3771b88e1ece9fbf9e8a75afc7ae7142f64\"\n\
             chamber_pressure_pa = 2000000.0\n\
             mixture_ratio = 2.5\n\
             \n\
             [propulsion.nozzle]",
        ),
    );

    let baseline_thrust = max_positive_thrust_n(&baseline);
    let thermochem_thrust = max_positive_thrust_n(&thermochem);

    assert!(thermochem_thrust > 0.0);
    assert_ne!(baseline_thrust.to_bits(), thermochem_thrust.to_bits());
}
