//! SIL stimulation coverage for scheduled propulsion faults.

#![allow(clippy::expect_used)]

use std::fs;
use std::path::Path;

use openbmp_sil::{
    FaultInjection, FaultTarget, MissionPackage, ParameterOverride, SilStimulation, read_signal,
};

const PHALCON_TVC_PROBE: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../scenarios/phalcon9/phalcon9-tvc-probe.toml"
);

const RIGID_ENGINE_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "sil-scheduled-engine-fault"
description = "Synthetic SIL scheduled propulsion fault regression."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.6
dt_s = 0.1
seed = 29

[vehicle]
kind = "rigid_body"
initial_position_eci_m = [0.0, 0.0, 10.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]
initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
initial_angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "sil-scheduled-engine-fault"

[[vehicle.assembly.bodies]]
id = "core"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 100.0
dry_cg_body_m = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2 = [[10.0, 0.0, 0.0], [0.0, 10.0, 0.0], [0.0, 0.0, 10.0]]

[[vehicle.assembly.engines]]
id = "main"
mounted_to = "core"
kind = { kind = "liquid_engine" }
mount_point_body_m = [0.0, 0.0, 0.0]
limits = { max_thrust_n = 1000.0, isp_s = 250.0, ignition_transient_s = 0.0, shutdown_transient_s = 0.0, max_gimbal_rad = 0.0 }

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 0.0
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity", "thrust"]

[mission]
initial_phase = "flight"

[[mission.phases]]
id = "flight"
label = "flight"

[scenario_script]

[[scenario_script.events]]
id = "ignite"
trigger = { kind = "at_time", time_s = 0.1 }
action = { kind = "engine_command", id = "main", command = { throttle_unit = 1.0, gimbal_pitch_rad = 0.0, gimbal_yaw_rad = 0.0, ignite = true, shutdown = false } }
once = true

[telemetry]
output.csv = "out/sil-scheduled-engine-fault.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

fn write_package(dir: &Path) -> (MissionPackage, std::path::PathBuf) {
    let scenario_path = dir.join("scenario.toml");
    fs::write(&scenario_path, RIGID_ENGINE_SCENARIO).expect("write scenario");
    let manifest_path = dir.join("mission-package.toml");
    fs::write(
        &manifest_path,
        "[package]\nid = \"test.propulsion-stimulus\"\nversion = \"0.0.1\"\n\n[files]\nscenario = \"scenario.toml\"\n",
    )
    .expect("write manifest");
    (
        MissionPackage::load(&manifest_path).expect("load package"),
        scenario_path,
    )
}

fn load_real_scenario_package(dir: &Path, scenario_path: &str) -> MissionPackage {
    let manifest_path = dir.join("mission-package.toml");
    fs::write(
        &manifest_path,
        format!(
            "[package]\nid = \"test.propulsion-response\"\nversion = \"0.0.1\"\n\n[files]\nscenario = \"{scenario_path}\"\n",
        ),
    )
    .expect("write manifest");
    MissionPackage::load(&manifest_path).expect("load package")
}

fn toml_value(src: &str, key: &str) -> toml::Value {
    let value: toml::Value = toml::from_str(src).expect("parse TOML value");
    value.get(key).expect("TOML key must exist").clone()
}

fn last_float(report: &openbmp_sil::SilRunReport, channel: &str) -> f64 {
    read_signal(report, channel, 8)
        .expect("read signal")
        .last
        .expect("channel must have a last sample")
        .value
        .parse::<f64>()
        .expect("channel last sample must be f64")
}

fn max_float(report: &openbmp_sil::SilRunReport, channel: &str) -> f64 {
    read_signal(report, channel, 8)
        .expect("read signal")
        .max
        .expect("channel must have a numeric max")
        .parse::<f64>()
        .expect("channel max must be f64")
}

#[test]
fn scheduled_propulsion_fault_stimulation_changes_engine_telemetry() {
    let dir = tempfile::tempdir().expect("tempdir");
    let (package, scenario_path) = write_package(dir.path());

    let nominal = package.run_case(None).expect("nominal run");
    let stimulus = SilStimulation {
        faults: vec![FaultInjection {
            target: FaultTarget::ScheduledEngine {
                rule_id: "sil-main-hardoff".to_owned(),
                engine_id: "main".to_owned(),
                start_step: 4,
            },
            fault: toml_value(r#"fault = { kind = "hard_off" }"#, "fault"),
        }],
        ..SilStimulation::default()
    };
    let faulted = package
        .run_case_with_stimulation(None, &stimulus)
        .expect("faulted run");

    let nominal_last = last_float(&nominal, "engine.main.thrust_n");
    let faulted_last = last_float(&faulted, "engine.main.thrust_n");
    let faulted_peak = max_float(&faulted, "engine.main.thrust_n");

    assert!(
        nominal_last > 900.0,
        "nominal engine should still be burning at end, got {nominal_last}"
    );
    assert!(
        faulted_peak > 900.0,
        "faulted run should burn before the scheduled hard-off, got peak {faulted_peak}"
    );
    assert!(
        faulted_last < 1.0,
        "scheduled hard-off should remove terminal thrust, got {faulted_last}"
    );
    assert!(
        faulted
            .evidence
            .stimuli
            .iter()
            .any(|record| record.kind == "fault_injection"
                && record.target == "propulsion.faults.rules.sil-main-hardoff"),
        "scheduled propulsion fault must be recorded as SIL evidence"
    );
    assert!(
        !fs::read_to_string(scenario_path)
            .expect("read original scenario")
            .contains("[propulsion.faults]"),
        "SIL stimulation must not mutate the package scenario file"
    );
}

#[test]
fn observed_stimulated_tvc_engine_fault_records_ekf_response() {
    let dir = tempfile::tempdir().expect("tempdir");
    let package = load_real_scenario_package(dir.path(), PHALCON_TVC_PROBE);
    let short_nominal = SilStimulation {
        parameter_overrides: vec![ParameterOverride {
            path: "time.stop_s".to_owned(),
            value: toml::Value::Float(3.0),
        }],
        ..SilStimulation::default()
    };
    let faulted_stimulus = SilStimulation {
        faults: vec![FaultInjection {
            target: FaultTarget::ScheduledEngine {
                rule_id: "sil-eng0-hardoff".to_owned(),
                engine_id: "eng_0".to_owned(),
                start_step: 150,
            },
            fault: toml_value(r#"fault = { kind = "hard_off" }"#, "fault"),
        }],
        parameter_overrides: short_nominal.parameter_overrides.clone(),
        ..SilStimulation::default()
    };

    let nominal = package
        .run_case_observed_with_stimulation(None, 20, &short_nominal)
        .expect("nominal observed short TVC run");
    let faulted = package
        .run_case_observed_with_stimulation(None, 20, &faulted_stimulus)
        .expect("faulted observed short TVC run");

    let nominal_terminal_thrust = last_float(&nominal, "engine.eng_0.thrust_n");
    let faulted_terminal_thrust = last_float(&faulted, "engine.eng_0.thrust_n");
    assert!(
        nominal_terminal_thrust > 100_000.0,
        "nominal eng_0 should still be burning, got {nominal_terminal_thrust}"
    );
    assert!(
        faulted_terminal_thrust < 1.0,
        "scheduled hard-off should remove eng_0 terminal thrust, got {faulted_terminal_thrust}"
    );

    let nominal_velocity_z = last_float(&nominal, "velocity_z_m_s");
    let faulted_velocity_z = last_float(&faulted, "velocity_z_m_s");
    assert!(
        nominal_velocity_z > faulted_velocity_z + 2.0,
        "engine-out response should reduce terminal vertical velocity: nominal {nominal_velocity_z}, faulted {faulted_velocity_z}"
    );

    let observations = faulted
        .evidence
        .observations
        .as_ref()
        .expect("faulted observed run must attach EKF evidence");
    assert!(
        observations.summary.ticks > 0,
        "EKF observation summary must cover runner ticks"
    );
    assert!(
        observations.summary.max_pos_err_m.is_some(),
        "closed-loop TVC run must publish translational EKF residuals"
    );
    assert!(
        faulted
            .evidence
            .stimuli
            .iter()
            .any(|record| record.target == "propulsion.faults.rules.sil-eng0-hardoff"),
        "fault rule must be recorded as SIL stimulation evidence"
    );
}
