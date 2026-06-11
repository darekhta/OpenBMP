//! Runner-level FC transport tests.

#![allow(clippy::expect_used, clippy::panic)]

use std::path::Path;

use openbmp_scenario::Scenario;

const SCENARIO_PATH: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../scenarios/closed-loop-attitude-hold/scenario.toml"
);

fn remove_section(toml: &str, section: &str) -> String {
    let header = format!("\n[{section}]\n");
    let start = toml.find(&header).expect("section exists");
    let body_start = start + header.len();
    let end = toml[body_start..]
        .find("\n[")
        .map_or(toml.len(), |offset| body_start + offset);
    format!("{}{}", &toml[..start], &toml[end..])
}

fn load(extra: &str) -> Scenario {
    let mut toml = std::fs::read_to_string(SCENARIO_PATH).expect("read scenario");
    toml = toml.replace("openbmp.scenario = 2", "openbmp.scenario = 3");
    toml.push_str(extra);
    let source_dir = Path::new(SCENARIO_PATH).parent().map(Path::to_path_buf);
    Scenario::from_toml_str_with_source_dir(&toml, source_dir).expect("scenario parses")
}

fn load_without_section(section: &str, extra: &str) -> Scenario {
    let mut toml = std::fs::read_to_string(SCENARIO_PATH).expect("read scenario");
    toml = toml.replace("openbmp.scenario = 2", "openbmp.scenario = 3");
    toml = remove_section(&toml, section);
    toml.push_str(extra);
    let source_dir = Path::new(SCENARIO_PATH).parent().map(Path::to_path_buf);
    Scenario::from_toml_str_with_source_dir(&toml, source_dir).expect("scenario parses")
}

#[test]
fn in_process_fc_transport_preserves_telemetry_bytes() {
    let baseline = openbmp_runner::run(&load("")).expect("baseline run");
    let transported = openbmp_runner::run(&load(
        r#"

[fc.transport]
mode = "in_process"
max_payload_len = 4096
"#,
    ))
    .expect("transported run");

    assert_eq!(
        baseline.table, transported.table,
        "[fc.transport] in_process must preserve canonical telemetry bytes"
    );
    assert_eq!(
        openbmp_runner::determinism::telemetry_sha256_hex(&baseline.table)
            .expect("baseline telemetry hash"),
        openbmp_runner::determinism::telemetry_sha256_hex(&transported.table)
            .expect("transport telemetry hash"),
        "[fc.transport] in_process must preserve the canonical telemetry SHA-256 gate"
    );
    assert_eq!(
        baseline.actuator_stream, transported.actuator_stream,
        "[fc.transport] in_process must preserve the actuator command stream digest"
    );
    assert!(
        baseline
            .actuator_stream
            .as_ref()
            .is_some_and(|report| report.packet_count > 0),
        "closed-loop baseline should produce actuator stream evidence"
    );
}

#[test]
fn tcp_loopback_fc_transport_preserves_telemetry_bytes() {
    let baseline = openbmp_runner::run(&load("")).expect("baseline run");
    let transported = openbmp_runner::run(&load(
        r#"

[fc.transport]
mode = "tcp_loopback"
max_payload_len = 4096
"#,
    ))
    .expect("transported run");

    assert_eq!(
        baseline.table, transported.table,
        "[fc.transport] tcp_loopback must preserve canonical telemetry bytes"
    );
    assert_eq!(
        openbmp_runner::determinism::telemetry_sha256_hex(&baseline.table)
            .expect("baseline telemetry hash"),
        openbmp_runner::determinism::telemetry_sha256_hex(&transported.table)
            .expect("transport telemetry hash"),
        "[fc.transport] tcp_loopback must preserve the canonical telemetry SHA-256 gate"
    );
    assert_eq!(
        baseline.actuator_stream, transported.actuator_stream,
        "[fc.transport] tcp_loopback must preserve the actuator command stream digest"
    );
    assert!(
        transported
            .actuator_stream
            .as_ref()
            .is_some_and(|report| report.packet_count > 0),
        "tcp_loopback run should produce actuator stream evidence"
    );
}

#[cfg(unix)]
#[test]
fn unix_loopback_fc_transport_preserves_telemetry_bytes() {
    let baseline = openbmp_runner::run(&load("")).expect("baseline run");
    let transported = openbmp_runner::run(&load(
        r#"

[fc.transport]
mode = "unix_loopback"
max_payload_len = 4096
"#,
    ))
    .expect("transported run");

    assert_eq!(
        baseline.table, transported.table,
        "[fc.transport] unix_loopback must preserve canonical telemetry bytes"
    );
    assert_eq!(
        openbmp_runner::determinism::telemetry_sha256_hex(&baseline.table)
            .expect("baseline telemetry hash"),
        openbmp_runner::determinism::telemetry_sha256_hex(&transported.table)
            .expect("transport telemetry hash"),
        "[fc.transport] unix_loopback must preserve the canonical telemetry SHA-256 gate"
    );
    assert_eq!(
        baseline.actuator_stream, transported.actuator_stream,
        "[fc.transport] unix_loopback must preserve the actuator command stream digest"
    );
    assert!(
        transported
            .actuator_stream
            .as_ref()
            .is_some_and(|report| report.packet_count > 0),
        "unix_loopback run should produce actuator stream evidence"
    );
}

#[test]
fn external_process_fc_transport_runs_stdio_peer() {
    let peer = env!("CARGO_BIN_EXE_openbmp-fc-stdio-echo");
    let extra = format!(
        r#"

[scenario_director]
mission_authority = "kernel"

[fc.transport]
mode = "external_process"
command = {peer:?}
max_payload_len = 4096
"#
    );
    let first = openbmp_runner::run(&load(&extra)).expect("first external-process run");
    let second = openbmp_runner::run(&load(&extra)).expect("second external-process run");

    assert_eq!(
        first.table, second.table,
        "[fc.transport] external_process must be deterministic across child-process runs"
    );
    assert_eq!(
        first.actuator_stream, second.actuator_stream,
        "[fc.transport] external_process must preserve deterministic actuator stream evidence"
    );
    assert!(
        first
            .actuator_stream
            .as_ref()
            .is_some_and(|report| report.packet_count > 0),
        "external_process run should produce actuator stream evidence"
    );
}

#[test]
fn fc_transport_rejects_missing_imu_sensor() {
    let scenario = load_without_section(
        "sensors.imu",
        r#"

[fc.transport]
mode = "in_process"
"#,
    );
    let err = openbmp_runner::run(&scenario).unwrap_err();

    assert!(
        err.to_string().contains("requires an IMU sensor"),
        "unexpected error: {err}"
    );
}

#[test]
fn fc_transport_protocol_version_mismatch_fails_closed() {
    let scenario = load(
        r#"

[fc.transport]
mode = "in_process"
peer_protocol_version = 0
"#,
    );
    let err = openbmp_runner::run(&scenario).unwrap_err();

    assert!(
        err.to_string().contains("bridge protocol mismatch"),
        "unexpected error: {err}"
    );
    assert!(err.to_string().contains("got 0"), "unexpected error: {err}");
}

#[test]
fn fc_transport_sensor_drop_fault_fails_closed() {
    let scenario = load(
        r#"

[fc.transport]
mode = "in_process"

[[fc.transport_faults.packet_rules]]
id         = "drop-first-sensor"
start_step = 0
end_step   = 0
direction  = "sensor"
transform  = { kind = "drop" }
"#,
    );
    let err = openbmp_runner::run(&scenario).unwrap_err();

    assert!(
        err.to_string().contains("drop-first-sensor"),
        "unexpected error: {err}"
    );
    assert!(
        err.to_string().contains("sensor frame step 0 dropped"),
        "unexpected error: {err}"
    );
}

#[test]
fn fc_transport_command_duplicate_fault_fails_closed() {
    let scenario = load(
        r#"

[fc.transport]
mode = "in_process"

[[fc.transport_faults.packet_rules]]
id         = "duplicate-first-command"
start_step = 0
end_step   = 0
direction  = "command"
transform  = { kind = "duplicate" }
"#,
    );
    let err = openbmp_runner::run(&scenario).unwrap_err();

    assert!(
        err.to_string().contains("duplicate-first-command"),
        "unexpected error: {err}"
    );
    assert!(
        err.to_string().contains("command frame step 0 duplicated"),
        "unexpected error: {err}"
    );
}

#[test]
fn fc_transport_sensor_delay_fault_fails_closed() {
    let scenario = load(
        r#"

[fc.transport]
mode = "in_process"

[[fc.transport_faults.packet_rules]]
id         = "delay-first-sensor"
start_step = 0
end_step   = 0
direction  = "sensor"
transform  = { kind = "delay", steps = 2 }
"#,
    );
    let err = openbmp_runner::run(&scenario).unwrap_err();

    assert!(
        err.to_string().contains("delay-first-sensor"),
        "unexpected error: {err}"
    );
    assert!(
        err.to_string()
            .contains("sensor frame step 0 delayed by 2 step(s)"),
        "unexpected error: {err}"
    );
}

#[test]
fn fc_transport_command_bit_flip_fault_fails_closed() {
    let scenario = load(
        r#"

[fc.transport]
mode = "in_process"

[[fc.transport_faults.packet_rules]]
id         = "bit-flip-first-command"
start_step = 0
end_step   = 0
direction  = "command"
transform  = { kind = "bit_flip", mask = 5 }
"#,
    );
    let err = openbmp_runner::run(&scenario).unwrap_err();

    assert!(
        err.to_string().contains("bit-flip-first-command"),
        "unexpected error: {err}"
    );
    assert!(
        err.to_string()
            .contains("command frame step 0 corrupted by bit-flip mask 0x05"),
        "unexpected error: {err}"
    );
}

#[test]
fn fc_transport_command_step_offset_fault_fails_closed() {
    let scenario = load(
        r#"

[fc.transport]
mode = "in_process"

[[fc.transport_faults.packet_rules]]
id         = "offset-first-command-step"
start_step = 0
end_step   = 0
direction  = "command"
transform  = { kind = "step_offset", offset = 1 }
"#,
    );
    let err = openbmp_runner::run(&scenario).unwrap_err();

    assert!(
        err.to_string().contains("bridge lockstep step mismatch"),
        "unexpected error: {err}"
    );
    assert!(
        err.to_string().contains("expected step 0, got 1"),
        "unexpected error: {err}"
    );
}

#[test]
fn fc_transport_command_time_offset_fault_fails_closed() {
    let scenario = load(
        r#"

[fc.transport]
mode = "in_process"

[[fc.transport_faults.packet_rules]]
id         = "offset-first-command-time"
start_step = 0
end_step   = 0
direction  = "command"
transform  = { kind = "time_offset", offset_s = 0.001 }
"#,
    );
    let err = openbmp_runner::run(&scenario).unwrap_err();

    assert!(
        err.to_string().contains("bridge lockstep time mismatch"),
        "unexpected error: {err}"
    );
}
