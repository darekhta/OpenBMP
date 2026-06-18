//! Runner-level FC transport tests.

#![allow(clippy::expect_used, clippy::panic)]

use std::path::Path;

use openbmp_scenario::Scenario;

const SCENARIO_PATH: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../scenarios/closed-loop-attitude-hold/scenario.toml"
);

const VISIBLE_HIGH_MARGIN_COMM: &str = r#"

[comm]

[[comm.sites]]
id = "equator-zero"
latitude_deg = 0.0
longitude_deg = 0.0
altitude_m = 0.0
min_elevation_deg = 0.0

[[comm.antennas]]
id = "s-band-omni"
gain_deck = "../../data/comm/antenna-link-budget-v1.toml"
body_mask_deck = "../../data/comm/antenna-link-budget-v1.toml"

[[comm.links]]
id = "s-band-equator"
site_id = "equator-zero"
antenna_id = "s-band-omni"
eirp_dbw = -33.0
receiver_g_over_t_db_k = 0.0
frequency_hz = 2.0e9
bit_rate_bps = 1000.0
required_eb_n0_db = 3.0
atmospheric_loss_db = 1.0
rain_loss_db = 0.5
pointing_loss_db = 0.25
polarization_loss_db = 0.1
implementation_loss_db = 0.0
fer_curve_deck = "../../data/comm/link-budget-fer-v1.toml"
packet_processing_delay_s = 0.005
packet_error_action = { kind = "drop" }
"#;

const HANDOVER_COMM: &str = r#"

[comm]
bridge_link_selection = "best_margin"

[[comm.sites]]
id = "opposite-side"
latitude_deg = 0.0
longitude_deg = 180.0
altitude_m = 0.0
min_elevation_deg = 0.0

[[comm.sites]]
id = "equator-zero"
latitude_deg = 0.0
longitude_deg = 0.0
altitude_m = 0.0
min_elevation_deg = 0.0

[[comm.antennas]]
id = "s-band-omni"
gain_deck = "../../data/comm/antenna-link-budget-v1.toml"
body_mask_deck = "../../data/comm/antenna-link-budget-v1.toml"

[[comm.links]]
id = "blocked-first"
site_id = "opposite-side"
antenna_id = "s-band-omni"
eirp_dbw = -33.0
receiver_g_over_t_db_k = 0.0
frequency_hz = 2.0e9
bit_rate_bps = 1000.0
required_eb_n0_db = 3.0
atmospheric_loss_db = 1.0
rain_loss_db = 0.5
pointing_loss_db = 0.25
polarization_loss_db = 0.1
implementation_loss_db = 0.0
fer_curve_deck = "../../data/comm/link-budget-fer-v1.toml"
packet_processing_delay_s = 0.0
packet_error_action = { kind = "drop" }

[[comm.links]]
id = "healthy-second"
site_id = "equator-zero"
antenna_id = "s-band-omni"
eirp_dbw = -33.0
receiver_g_over_t_db_k = 0.0
frequency_hz = 2.0e9
bit_rate_bps = 1000.0
required_eb_n0_db = 3.0
atmospheric_loss_db = 1.0
rain_loss_db = 0.5
pointing_loss_db = 0.25
polarization_loss_db = 0.1
implementation_loss_db = 0.0
fer_curve_deck = "../../data/comm/link-budget-fer-v1.toml"
packet_processing_delay_s = 0.0
packet_error_action = { kind = "drop" }
"#;

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

fn load_visible_comm_with_processing_delay(delay_s: f64, extra: &str) -> Scenario {
    let mut toml = std::fs::read_to_string(SCENARIO_PATH).expect("read scenario");
    toml = toml.replace("openbmp.scenario = 2", "openbmp.scenario = 3");
    toml = toml.replace(
        "initial_position_eci_m   = [0.0, 0.0, 0.0]",
        "initial_position_eci_m   = [6379137.0, 0.0, 0.0]",
    );
    let comm = VISIBLE_HIGH_MARGIN_COMM.replace(
        "packet_processing_delay_s = 0.005",
        &format!("packet_processing_delay_s = {delay_s:.17e}"),
    );
    toml.push_str(&comm);
    toml.push_str(extra);
    let source_dir = Path::new(SCENARIO_PATH).parent().map(Path::to_path_buf);
    Scenario::from_toml_str_with_source_dir(&toml, source_dir).expect("scenario parses")
}

fn load_visible_comm(extra: &str) -> Scenario {
    load_visible_comm_with_processing_delay(0.005, extra)
}

fn load_handover_comm(selection: &str, extra: &str) -> Scenario {
    let mut toml = std::fs::read_to_string(SCENARIO_PATH).expect("read scenario");
    toml = toml.replace("openbmp.scenario = 2", "openbmp.scenario = 3");
    toml = toml.replace(
        "initial_position_eci_m   = [0.0, 0.0, 0.0]",
        "initial_position_eci_m   = [6379137.0, 0.0, 0.0]",
    );
    let comm = HANDOVER_COMM.replace(
        "bridge_link_selection = \"best_margin\"",
        &format!("bridge_link_selection = {selection:?}"),
    );
    toml.push_str(&comm);
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

#[test]
fn fc_transport_comm_processing_delay_queues_actuator_commands() {
    let direct = openbmp_runner::run(&load_visible_comm("")).expect("direct comm run");
    let transported = openbmp_runner::run(&load_visible_comm(
        r#"

[fc.transport]
mode = "in_process"
max_payload_len = 4096
"#,
    ))
    .expect("transported delayed comm run");

    let direct_stream = direct
        .actuator_stream
        .as_ref()
        .expect("direct actuator stream");
    let transported_stream = transported
        .actuator_stream
        .as_ref()
        .expect("transport actuator stream");
    assert!(
        transported_stream.packet_count + 8 <= direct_stream.packet_count,
        "two-way comm delay should leave the final command frames queued: direct={}, transported={}",
        direct_stream.packet_count,
        transported_stream.packet_count
    );
    assert_ne!(
        direct_stream.sha256_hex, transported_stream.sha256_hex,
        "delayed transport command stream should not hash like immediate direct commands"
    );

    let comm = transported.comm.as_ref().expect("transported comm report");
    let link = comm.links.first().expect("comm link report");
    assert_eq!(
        link.packet_processing_delay_s.to_bits(),
        0.005_f64.to_bits()
    );
    let timeout = link.data_loss_timeout.as_ref();
    assert!(
        timeout.is_none(),
        "test link does not declare timeout latch"
    );
}

#[test]
fn fc_transport_zero_step_comm_link_preserves_direct_bridge_bytes() {
    let direct = openbmp_runner::run(&load_visible_comm_with_processing_delay(0.0, ""))
        .expect("direct comm run");
    let transported = openbmp_runner::run(&load_visible_comm_with_processing_delay(
        0.0,
        r#"

[fc.transport]
mode = "in_process"
max_payload_len = 4096
"#,
    ))
    .expect("transported zero-step comm run");

    assert_eq!(
        direct.table, transported.table,
        "zero-step delivered comm link should preserve telemetry bytes"
    );
    assert_eq!(
        openbmp_runner::determinism::telemetry_sha256_hex(&direct.table)
            .expect("direct telemetry hash"),
        openbmp_runner::determinism::telemetry_sha256_hex(&transported.table)
            .expect("transport telemetry hash"),
        "zero-step delivered comm link should preserve telemetry digest"
    );
    assert_eq!(
        direct.actuator_stream, transported.actuator_stream,
        "zero-step delivered comm link should preserve actuator stream evidence"
    );

    let comm = transported.comm.as_ref().expect("transported comm report");
    let link = comm.links.first().expect("comm link report");
    assert_eq!(link.packet_processing_delay_s.to_bits(), 0.0_f64.to_bits());
}

#[test]
fn fc_transport_best_margin_comm_link_handover_uses_visible_second_link() {
    let transported = openbmp_runner::run(&load_handover_comm(
        "best_margin",
        r#"

[fc.transport]
mode = "in_process"
max_payload_len = 4096
"#,
    ))
    .expect("best-margin handover run");

    assert!(
        transported
            .actuator_stream
            .as_ref()
            .is_some_and(|report| report.packet_count > 0),
        "best-margin selected visible second link should deliver transport commands"
    );
    let comm = transported.comm.as_ref().expect("comm report");
    assert_eq!(comm.links.len(), 2);
    assert_eq!(comm.links[0].link_id, "blocked-first");
    assert_eq!(comm.links[1].link_id, "healthy-second");
}

#[test]
fn fc_transport_first_declared_comm_link_fails_closed_on_blocked_first_link() {
    let err = openbmp_runner::run(&load_handover_comm(
        "first_declared",
        r#"

[fc.transport]
mode = "in_process"
max_payload_len = 4096
"#,
    ))
    .unwrap_err();

    assert!(
        err.to_string()
            .contains("comm link `blocked-first` dropped fc.transport sensor frame step 0"),
        "unexpected error: {err}"
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
