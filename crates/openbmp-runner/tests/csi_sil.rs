//! Runner-level CSI SIL evidence for structural pickup through the FC bridge.

#![allow(clippy::expect_used, clippy::panic)]

use std::path::Path;

use openbmp_core::StepIndex;
use openbmp_runner::sil::{FcObservation, SilMonitor};
use openbmp_sensors::SensorTruth;
use openbmp_sim::StopReason;
use openbmp_telemetry::TelemetryValue;

const TVC_PROBE_PATH: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../scenarios/phalcon9/phalcon9-tvc-probe.toml"
);

const CSI_SIL_BLOCK: &str = r#"

[vehicle.bending]
frequency_hz = 4.0
damping_ratio = 0.04
modal_mass_kg = 5000.0
slope_at_engine = 0.02
slope_at_gyro = 0.001

[csi]
frequency_grid_rad_s = [1.0, 25.132741228718345, 60.0]

[csi.criteria]
min_gain_margin_db = 0.0
min_phase_margin_deg = 0.0

[csi.modal_assembly]
sensor = "gyro_rate"
loop_gain = 1.0

[[csi.modal_assembly.frozen_points]]
id = "powered_ascent"
time_s = 0.0

[csi.filter_insertion]
mode_index = 0
zero_damping_ratio = 0.02
pole_damping_ratio = 0.20
"#;

#[derive(Default)]
struct CountingMonitor {
    ticks: usize,
    saw_attitude: bool,
}

impl SilMonitor for CountingMonitor {
    fn observe(&mut self, _step: StepIndex, _truth: &SensorTruth, observation: &FcObservation) {
        self.ticks += 1;
        self.saw_attitude |= observation.attitude.is_some();
    }
}

fn load_tvc_probe_with_csi() -> openbmp_scenario::Scenario {
    let mut toml = std::fs::read_to_string(TVC_PROBE_PATH).expect("read TVC probe scenario");
    toml.push_str(CSI_SIL_BLOCK);
    let source_dir = Path::new(TVC_PROBE_PATH).parent().map(Path::to_path_buf);
    openbmp_scenario::Scenario::from_toml_str_with_source_dir(&toml, source_dir)
        .expect("scenario parses")
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

#[test]
fn csi_filter_insertion_tvc_probe_remains_bounded() {
    let scenario = load_tvc_probe_with_csi();
    let mut monitor = CountingMonitor::default();
    let outcome = openbmp_runner::run_with_monitor(&scenario, &mut monitor)
        .expect("TVC probe with CSI filter insertion runs");
    assert!(matches!(outcome.stop_reason, StopReason::EndTime { .. }));
    assert!((outcome.final_time_s - 40.0).abs() <= 1.0e-9);
    assert!(monitor.ticks > 0, "SIL monitor must observe FC ticks");
    assert!(
        monitor.saw_attitude,
        "SIL monitor must observe FC attitude estimates"
    );

    let csi = outcome.csi.as_ref().expect("CSI report is emitted");
    assert_eq!(csi.trajectory.point_count, 1);
    assert!(csi.gyro_notch.is_some(), "CSI notch must be inserted");
    let actuator_stream = outcome
        .actuator_stream
        .as_ref()
        .expect("FC actuator command stream evidence is emitted");
    assert!(
        actuator_stream.packet_count > 0,
        "FC must emit actuator command packets"
    );

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
        "CSI filter insertion run produced unbounded attitude rate: \
         {max_rate_rad_s:.17e} rad/s"
    );
    assert!(
        max_quaternion_norm_error < 1.0e-9,
        "CSI filter insertion run lost quaternion normalization: \
         {max_quaternion_norm_error:.17e}"
    );
}
