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

fn has_channel(outcome: &openbmp_runner::RunOutcome, name: &str) -> bool {
    outcome
        .table
        .schema()
        .channels()
        .iter()
        .any(|channel| channel.name == name)
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
fn plume_telemetry_is_opt_in_and_uses_live_nozzle_state() {
    let baseline = run_scenario(include_str!("fixtures/point-mass-pressure-thrust.toml"));
    assert!(!has_channel(&baseline, "plume.nozzle_pressure_ratio"));

    let plume_enabled = run_scenario(
        &include_str!("fixtures/point-mass-pressure-thrust.toml").replace(
            "[forces]",
            "[aero]\n\
             \n\
             [aero.plume]\n\
             engine_count = 1\n\
             reference_area_m2 = 1.0\n\
             exit_area_total_m2 = 0.002\n\
             base_area_m2 = 1.0\n\
             center_spacing_m = 0.0\n\
             merge_evaluation_distance_m = 0.0\n\
             pifs_onset_angle_rad = 0.05\n\
             \n\
             [forces]",
        ),
    );

    let nozzle_pressure_ratio = f64_column(&plume_enabled, "plume.nozzle_pressure_ratio");
    assert!(
        nozzle_pressure_ratio.iter().any(|value| *value > 1.0),
        "live plume telemetry should report powered nozzle pressure ratio: {nozzle_pressure_ratio:?}"
    );
    let thrust_coefficient = f64_column(&plume_enabled, "plume.thrust_coefficient");
    assert!(
        thrust_coefficient.iter().any(|value| *value > 0.0),
        "live plume telemetry should report powered thrust coefficient: {thrust_coefficient:?}"
    );
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
             file_sha256 = \"fa3ff43bc91d04317d44a141cf5ff28680e4e0b35d89ff3c91d92ee60f1a842b\"\n\
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
