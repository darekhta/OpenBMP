//! Regression coverage for exported flight-controller reference telemetry.

use openbmp_core::ChannelId;
use openbmp_telemetry::TelemetryValue;

fn channel_id(outcome: &openbmp_runner::RunOutcome, name: &str) -> ChannelId {
    outcome
        .table
        .schema()
        .channels()
        .iter()
        .find(|channel| channel.name == name)
        .unwrap_or_else(|| panic!("channel `{name}` must exist"))
        .id
}

#[test]
fn fc_scenarios_export_guidance_reference_quaternion() {
    let base_toml = r#"
openbmp.scenario = 3

[meta]
name = "fc-reference-telemetry"
description = "Synthetic FC reference telemetry regression."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.05
dt_s = 0.01
seed = 42

[vehicle]
kind = "rigid_body"
initial_position_eci_m = [0.0, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]
initial_quaternion_body_to_eci_xyzw = [0.0, 0.0, 0.0, 1.0]
initial_angular_velocity_body_rad_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "fc-reference-telemetry"

[[vehicle.assembly.bodies]]
id = "main"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_inertia_body_kg_m2 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[[vehicle.assembly.effectors]]
id = "attitude"
kind = { kind = "direct_torque", axis = "pitch", effectiveness_n_m_per_rad = 1.0 }
limits = { min = -0.35, max = 0.35, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }
initial_position = 0.0
unit = "rad"

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 0.0
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity"]

[telemetry]
output.csv = "out/fc-reference-telemetry.csv"

[validation]
require_finite_state = true
require_monotonic_time = true

[mission]
initial_phase = "pad"

[[mission.phases]]
id = "pad"
label = "pad"

[[mission.phases]]
id = "powered_ascent"
label = "powered ascent"
allowed_effectors = ["attitude"]

[[mission.events]]
id = "liftoff"
trigger = { kind = "at_time", time_s = 0.0 }
action = { kind = "enter_phase", phase = "powered_ascent" }

[[mission.transitions]]
from = "pad"
to = "powered_ascent"
event = "liftoff"

[fc]
estimator = "ekf"
autopilot = "three_loop"
guidance = "attitude_hold"
reference_q_xyzw = [0.0, 9.98334166468281548e-2, 0.0, 9.95004165278025821e-1]
base_rate_hz = 1000
frame_budget_us = 2000

[fc.ekf]

[fc.health]
imu_stale_after_s = 0.05
gnss_stale_after_s = 0.5
baro_stale_after_s = 10.0
mag_stale_after_s = 0.2
overrun_burst_count = 5

[fc.gain_schedule."mission.phases.powered_ascent"]
rate_kp = [0.5, 0.5, 0.5]
rate_kd = [0.05, 0.05, 0.05]
attitude_kp = [2.0, 2.0, 1.0]
elevator_limit_rad = 0.35
aileron_limit_rad = 0.35
rudder_limit_rad = 0.35
throttle_baseline = 0.0
"#;
    let scenario = openbmp_scenario::Scenario::from_toml_str(base_toml).expect("scenario parses");

    let outcome = openbmp_runner::run(&scenario).expect("scenario runs");

    let valid = channel_id(&outcome, "guidance.reference.valid");
    let q_x = channel_id(&outcome, "guidance.reference.q_x");
    let q_y = channel_id(&outcome, "guidance.reference.q_y");
    let q_z = channel_id(&outcome, "guidance.reference.q_z");
    let q_w = channel_id(&outcome, "guidance.reference.q_w");

    let valid_quaternion_rows = outcome
        .table
        .rows()
        .iter()
        .filter(|row| matches!(row.get(valid), Some(TelemetryValue::Bool(true))))
        .map(|row| {
            [q_x, q_y, q_z, q_w].map(|channel| match row.get(channel) {
                Some(TelemetryValue::Float64(value)) => *value,
                other => panic!("unexpected reference quaternion value: {other:?}"),
            })
        })
        .collect::<Vec<_>>();

    assert!(
        !valid_quaternion_rows.is_empty(),
        "FC guidance should publish at least one valid reference row"
    );
    for q in valid_quaternion_rows {
        let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
        assert!(
            (norm - 1.0).abs() < 1.0e-12,
            "reference quaternion must stay unit length, got {q:?} with norm {norm}"
        );
    }

    assert!(
        outcome
            .table
            .schema()
            .channels()
            .iter()
            .all(|channel| channel.name != "guidance.cutoff.time_to_go_s"),
        "attitude-hold guidance should not export ascent cutoff telemetry"
    );

    let ascent_reference_toml = base_toml.replace(
        "guidance = \"attitude_hold\"\nreference_q_xyzw = [0.0, 9.98334166468281548e-2, 0.0, 9.95004165278025821e-1]",
        "guidance = \"ascent_reference\"",
    )
    .replace(
        "[fc.ekf]",
        "[fc.ascent_reference]\nmethod = \"pitch_program\"\nschedule_s = [0.0, 1.0]\npitch_rad = [0.0, 0.0]\n\n[fc.ekf]",
    );
    let ascent_reference = openbmp_scenario::Scenario::from_toml_str(&ascent_reference_toml)
        .expect("ascent-reference scenario parses");
    let ascent_outcome =
        openbmp_runner::run(&ascent_reference).expect("ascent-reference scenario runs");
    let _cutoff_valid = channel_id(&ascent_outcome, "guidance.cutoff.valid");
    let _cutoff_time = channel_id(&ascent_outcome, "guidance.cutoff.time_to_go_s");
}
