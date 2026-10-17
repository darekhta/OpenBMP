//! Runner-level regression coverage for opt-in structural reaction feedback.

#![allow(clippy::expect_used, clippy::panic)]

use std::path::Path;

use openbmp_scenario::Scenario;
use openbmp_sim::StopReason;
use openbmp_telemetry::TelemetryValue;

const TVC_PROBE_PATH: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../scenarios/phalcon9/phalcon9-tvc-probe.toml"
);

const STRUCTURAL_BENDING_OPT_IN: &str = r#"

[vehicle.bending]
frequency_hz = 4.0
damping_ratio = 0.04
modal_mass_kg = 5000.0
slope_at_engine = 0.02
slope_at_gyro = 0.005
"#;

const STRUCTURAL_BENDING_STRONG_REACTION_OPT_IN: &str = r#"

[vehicle.bending]
frequency_hz = 4.0
damping_ratio = 0.04
modal_mass_kg = 5000.0
slope_at_engine = 0.5
slope_at_gyro = 0.005
"#;

const STRUCTURAL_REACTION_OPT_IN: &str = r#"

[vehicle.reaction]
enabled = true
"#;

#[cfg(feature = "indi")]
const ENGINE_OUT_OPT_IN: &str = r#"

[propulsion.faults]

[[propulsion.faults.rules]]
id = "eng-0-hard-off"
engine_id = "eng_0"
start_step = 200
fault = { kind = "hard_off" }
"#;

#[cfg(feature = "indi")]
const INDI_RATE_LOOP_OPT_IN: &str = r#"
rate_loop_kind = "indi"

[fc.autopilot_params.indi]
inertia_per_axis_kg_m2 = [1.0e7, 1.0e7, 2.0e5]
control_effectiveness_per_axis = [9.2232e7, 9.2232e7, 1.0]
filter_cutoff_rad_s = 25.0
filter_kind = "first_order_low_pass"
attitude_to_omega_dot_gain = [2.0, 2.0, 1.0]
"#;

#[cfg(feature = "indi")]
const TVC_GIMBAL_EFFECTIVENESS_PER_ENGINE_RAD_S2: f64 = -9.2232 / 9.0;

#[cfg(feature = "indi")]
fn dense_inca_tvc_gimbal_allocation_opt_in() -> String {
    let mut toml = String::from(
        r#"

[fc.autopilot_allocation]
kind = "dense_inca"
gamma = 1000000.0
virtual_weights = [1.0, 1.0, 1.0]
"#,
    );
    for engine_index in 0..9 {
        toml.push_str(&format!(
            r#"

[[fc.autopilot_allocation.dense_effectors]]
id = "eng_{engine_index}.gimbal_yaw"
effectiveness = [{TVC_GIMBAL_EFFECTIVENESS_PER_ENGINE_RAD_S2}, 0.0, 0.0]

[[fc.autopilot_allocation.dense_effectors]]
id = "eng_{engine_index}.gimbal_pitch"
effectiveness = [0.0, {TVC_GIMBAL_EFFECTIVENESS_PER_ENGINE_RAD_S2}, 0.0]
"#
        ));
    }
    toml
}

fn load_tvc_probe_with_bending_block(bending_block: &str, reaction_enabled: bool) -> Scenario {
    let mut toml = std::fs::read_to_string(TVC_PROBE_PATH).expect("read TVC probe scenario");
    toml.push_str(bending_block);
    if reaction_enabled {
        toml.push_str(STRUCTURAL_REACTION_OPT_IN);
    }
    let source_dir = Path::new(TVC_PROBE_PATH).parent().map(Path::to_path_buf);
    Scenario::from_toml_str_with_source_dir(&toml, source_dir).expect("scenario parses")
}

fn load_tvc_probe_with_structural_bending(reaction_enabled: bool) -> Scenario {
    load_tvc_probe_with_bending_block(STRUCTURAL_BENDING_OPT_IN, reaction_enabled)
}

fn load_tvc_probe_with_structural_reaction() -> Scenario {
    load_tvc_probe_with_structural_bending(true)
}

fn assert_reaches_end_time(outcome: &openbmp_runner::RunOutcome) {
    assert!(matches!(outcome.stop_reason, StopReason::EndTime { .. }));
    assert!((outcome.final_time_s - 40.0).abs() <= 1.0e-9);
}

#[cfg(feature = "indi")]
fn assert_reaches_expected_end_time(outcome: &openbmp_runner::RunOutcome, expected_s: f64) {
    assert!(matches!(outcome.stop_reason, StopReason::EndTime { .. }));
    assert!((outcome.final_time_s - expected_s).abs() <= 1.0e-9);
}

fn f64_column(outcome: &openbmp_runner::RunOutcome, name: &str) -> Vec<f64> {
    let id = outcome
        .table
        .schema()
        .channels()
        .iter()
        .find(|channel| channel.name == name)
        .unwrap_or_else(|| panic!("channel `{name}` must exist"))
        .id;
    outcome
        .table
        .rows()
        .iter()
        .map(|row| match row.get(id) {
            Some(TelemetryValue::Float64(value)) => *value,
            other => panic!("unexpected value in {name}: {other:?}"),
        })
        .collect()
}

#[cfg(feature = "indi")]
fn i64_column(outcome: &openbmp_runner::RunOutcome, name: &str) -> Vec<i64> {
    let id = outcome
        .table
        .schema()
        .channels()
        .iter()
        .find(|channel| channel.name == name)
        .unwrap_or_else(|| panic!("channel `{name}` must exist"))
        .id;
    outcome
        .table
        .rows()
        .iter()
        .map(|row| match row.get(id) {
            Some(TelemetryValue::Int64(value)) => *value,
            other => panic!("unexpected value in {name}: {other:?}"),
        })
        .collect()
}

fn max_abs_difference(lhs: &[f64], rhs: &[f64]) -> f64 {
    assert_eq!(lhs.len(), rhs.len());
    lhs.iter()
        .zip(rhs)
        .map(|(a, b)| (a - b).abs())
        .fold(0.0, f64::max)
}

#[cfg(feature = "indi")]
fn rate_history_delta_rms(
    lhs: &openbmp_runner::RunOutcome,
    rhs: &openbmp_runner::RunOutcome,
) -> f64 {
    let mut sum_sq = 0.0;
    let mut samples = 0_usize;
    for column in [
        "angular_velocity.x_rad_s",
        "angular_velocity.y_rad_s",
        "angular_velocity.z_rad_s",
    ] {
        let lhs_values = f64_column(lhs, column);
        let rhs_values = f64_column(rhs, column);
        assert_eq!(lhs_values.len(), rhs_values.len());
        for (lhs_value, rhs_value) in lhs_values.iter().zip(&rhs_values) {
            let delta = lhs_value - rhs_value;
            sum_sq += delta * delta;
            samples += 1;
        }
    }
    assert!(samples > 0);
    (sum_sq / samples as f64).sqrt()
}

#[cfg(feature = "indi")]
fn shortened_tvc_probe_with_rate_loop(
    bending_block: &str,
    reaction_enabled: bool,
    use_indi: bool,
    use_dense_inca_tvc: bool,
) -> Scenario {
    let mut toml = std::fs::read_to_string(TVC_PROBE_PATH).expect("read TVC probe scenario");
    toml = toml.replace("stop_s  = 40.0", "stop_s  = 20.0");
    if use_indi {
        toml = toml.replace(
            "trajectory_kind       = \"pid\"\n",
            &format!("trajectory_kind       = \"pid\"\n{INDI_RATE_LOOP_OPT_IN}"),
        );
    }
    if use_dense_inca_tvc {
        toml.push_str(&dense_inca_tvc_gimbal_allocation_opt_in());
    }
    toml.push_str(bending_block);
    if reaction_enabled {
        toml.push_str(STRUCTURAL_REACTION_OPT_IN);
    }
    let source_dir = Path::new(TVC_PROBE_PATH).parent().map(Path::to_path_buf);
    Scenario::from_toml_str_with_source_dir(&toml, source_dir).expect("scenario parses")
}

#[cfg(feature = "indi")]
fn shortened_dense_inca_tvc_probe_with_engine_out() -> Scenario {
    let mut toml = std::fs::read_to_string(TVC_PROBE_PATH).expect("read TVC probe scenario");
    toml = toml.replace("stop_s  = 40.0", "stop_s  = 5.0");
    toml = toml.replace(
        "trajectory_kind       = \"pid\"\n",
        &format!("trajectory_kind       = \"pid\"\n{INDI_RATE_LOOP_OPT_IN}"),
    );
    toml.push_str(&dense_inca_tvc_gimbal_allocation_opt_in());
    toml.push_str(ENGINE_OUT_OPT_IN);
    let source_dir = Path::new(TVC_PROBE_PATH).parent().map(Path::to_path_buf);
    Scenario::from_toml_str_with_source_dir(&toml, source_dir).expect("scenario parses")
}

#[test]
fn structural_reaction_opt_in_tvc_probe_remains_bounded() {
    let scenario = load_tvc_probe_with_structural_reaction();
    let outcome = openbmp_runner::run(&scenario).expect("TVC probe with structural reaction runs");
    assert_reaches_end_time(&outcome);

    let wx = f64_column(&outcome, "angular_velocity.x_rad_s");
    let wy = f64_column(&outcome, "angular_velocity.y_rad_s");
    let wz = f64_column(&outcome, "angular_velocity.z_rad_s");
    let qx = f64_column(&outcome, "attitude.q_x");
    let qy = f64_column(&outcome, "attitude.q_y");
    let qz = f64_column(&outcome, "attitude.q_z");
    let qw = f64_column(&outcome, "attitude.q_w");

    let mut max_rate_rad_s = 0.0_f64;
    let mut max_quaternion_norm_error = 0.0_f64;
    for ((((((wx, wy), wz), qx), qy), qz), qw) in wx
        .iter()
        .zip(&wy)
        .zip(&wz)
        .zip(&qx)
        .zip(&qy)
        .zip(&qz)
        .zip(&qw)
    {
        let rate = (*wx * *wx + *wy * *wy + *wz * *wz).sqrt();
        assert!(rate.is_finite(), "angular-rate magnitude must stay finite");
        max_rate_rad_s = max_rate_rad_s.max(rate);
        let norm_sq = *qx * *qx + *qy * *qy + *qz * *qz + *qw * *qw;
        assert!(norm_sq.is_finite(), "quaternion norm must stay finite");
        max_quaternion_norm_error = max_quaternion_norm_error.max((norm_sq - 1.0).abs());
    }

    assert!(
        max_rate_rad_s < 2.0,
        "structural reaction injected unbounded attitude rate: {max_rate_rad_s:.17e} rad/s"
    );
    assert!(
        max_quaternion_norm_error < 1.0e-9,
        "structural reaction run lost quaternion normalization: {max_quaternion_norm_error:.17e}"
    );
}

#[test]
fn structural_reaction_opt_in_perturbs_tvc_probe_attitude_history() {
    let baseline = openbmp_runner::run(&load_tvc_probe_with_bending_block(
        STRUCTURAL_BENDING_STRONG_REACTION_OPT_IN,
        false,
    ))
    .expect("TVC probe with bending and no reaction runs");
    let reaction = openbmp_runner::run(&load_tvc_probe_with_bending_block(
        STRUCTURAL_BENDING_STRONG_REACTION_OPT_IN,
        true,
    ))
    .expect("TVC probe with bending reaction runs");
    assert_reaches_end_time(&baseline);
    assert_reaches_end_time(&reaction);

    let mut max_attitude_rate_delta = 0.0_f64;
    for column in [
        "angular_velocity.x_rad_s",
        "angular_velocity.y_rad_s",
        "angular_velocity.z_rad_s",
    ] {
        max_attitude_rate_delta = max_attitude_rate_delta.max(max_abs_difference(
            &f64_column(&baseline, column),
            &f64_column(&reaction, column),
        ));
    }

    assert!(max_attitude_rate_delta.is_finite());
    assert!(
        max_attitude_rate_delta > 1.0e-10,
        "reaction opt-in should create a nonzero TVC attitude-rate history delta; got {max_attitude_rate_delta:.17e}"
    );
}

#[cfg(feature = "indi")]
#[test]
fn scalar_indi_rate_loop_reduces_live_structural_reaction_residual() {
    let pid_baseline = openbmp_runner::run(&shortened_tvc_probe_with_rate_loop(
        STRUCTURAL_BENDING_STRONG_REACTION_OPT_IN,
        false,
        false,
        false,
    ))
    .expect("PID TVC probe without reaction runs");
    let pid_reaction = openbmp_runner::run(&shortened_tvc_probe_with_rate_loop(
        STRUCTURAL_BENDING_STRONG_REACTION_OPT_IN,
        true,
        false,
        false,
    ))
    .expect("PID TVC probe with reaction runs");
    let indi_baseline = openbmp_runner::run(&shortened_tvc_probe_with_rate_loop(
        STRUCTURAL_BENDING_STRONG_REACTION_OPT_IN,
        false,
        true,
        false,
    ))
    .expect("INDI TVC probe without reaction runs");
    let indi_reaction = openbmp_runner::run(&shortened_tvc_probe_with_rate_loop(
        STRUCTURAL_BENDING_STRONG_REACTION_OPT_IN,
        true,
        true,
        false,
    ))
    .expect("INDI TVC probe with reaction runs");

    for outcome in [&pid_baseline, &pid_reaction, &indi_baseline, &indi_reaction] {
        assert_reaches_expected_end_time(outcome, 20.0);
    }

    let pid_residual_rms = rate_history_delta_rms(&pid_baseline, &pid_reaction);
    let indi_residual_rms = rate_history_delta_rms(&indi_baseline, &indi_reaction);
    assert!(pid_residual_rms.is_finite());
    assert!(indi_residual_rms.is_finite());
    assert!(
        indi_residual_rms < 0.75 * pid_residual_rms,
        "INDI should reduce live structural-reaction body-rate residual versus PID: \
         pid={pid_residual_rms:.17e}, indi={indi_residual_rms:.17e}"
    );
}

#[cfg(feature = "indi")]
#[test]
fn dense_inca_runtime_reduces_live_structural_reaction_residual() {
    let pid_baseline = openbmp_runner::run(&shortened_tvc_probe_with_rate_loop(
        STRUCTURAL_BENDING_STRONG_REACTION_OPT_IN,
        false,
        false,
        false,
    ))
    .expect("PID TVC probe without reaction runs");
    let pid_reaction = openbmp_runner::run(&shortened_tvc_probe_with_rate_loop(
        STRUCTURAL_BENDING_STRONG_REACTION_OPT_IN,
        true,
        false,
        false,
    ))
    .expect("PID TVC probe with reaction runs");
    let dense_inca_baseline = openbmp_runner::run(&shortened_tvc_probe_with_rate_loop(
        STRUCTURAL_BENDING_STRONG_REACTION_OPT_IN,
        false,
        true,
        true,
    ))
    .expect("dense-INCA TVC probe without reaction runs");
    let dense_inca_reaction = openbmp_runner::run(&shortened_tvc_probe_with_rate_loop(
        STRUCTURAL_BENDING_STRONG_REACTION_OPT_IN,
        true,
        true,
        true,
    ))
    .expect("dense-INCA TVC probe with reaction runs");

    for outcome in [
        &pid_baseline,
        &pid_reaction,
        &dense_inca_baseline,
        &dense_inca_reaction,
    ] {
        assert_reaches_expected_end_time(outcome, 20.0);
    }

    let pid_residual_rms = rate_history_delta_rms(&pid_baseline, &pid_reaction);
    let dense_inca_residual_rms =
        rate_history_delta_rms(&dense_inca_baseline, &dense_inca_reaction);
    assert!(pid_residual_rms.is_finite());
    assert!(dense_inca_residual_rms.is_finite());

    assert!(
        dense_inca_residual_rms < 0.75 * pid_residual_rms,
        "dense-INCA physical TVC-gimbal allocation should reduce live structural-reaction \
         body-rate residual versus PID: pid={pid_residual_rms:.17e}, \
         dense_inca={dense_inca_residual_rms:.17e}"
    );
}

#[cfg(feature = "indi")]
#[test]
fn dense_inca_physical_tvc_survives_scripted_engine_out() {
    let outcome = openbmp_runner::run(&shortened_dense_inca_tvc_probe_with_engine_out())
        .expect("dense-INCA TVC probe with engine-out runs");
    assert_reaches_expected_end_time(&outcome, 5.0);

    let failed_state_seen = i64_column(&outcome, "engine.eng_0.state_index").contains(&4);
    assert!(
        failed_state_seen,
        "engine-out regression should record eng_0 reaching Failed state"
    );
    let actuator_stream = outcome
        .actuator_stream
        .as_ref()
        .expect("FC actuator stream evidence should be present");
    assert!(
        actuator_stream.packet_count > 0,
        "engine-out dense-INCA run should emit actuator command packets"
    );
}
