//! End-to-end test: L1 adaptive augmentation rejects a
//! matched roll-axis actuator disturbance better than PID alone.
//!
//! Two sibling scenarios:
//!   - `scenarios/diff-flatness-figure-eight-l1` — PID rate loop +
//!     `[fc.autopilot_params.l1_adaptive]` block + `ReducedRate {
//!     factor = 0.7 }` fault on the roll-torque effector.
//!   - `scenarios/diff-flatness-figure-eight-baseline` — same vehicle,
//!     same trajectory, same fault, but no L1 block (PID-only rate
//!     loop).
//!
//! The test runs both scenarios, computes the maximum body-frame
//! angular-velocity magnitude `|ω|` for each, and asserts the L1 run's
//! peak is at most half the PID baseline's peak — i.e. L1 has
//! materially tighter closed-loop tracking under the same disturbance.
//!
//! Also confirms each scenario is byte-stable across reruns, since
//! the L1 channel state evolution must be as deterministic as the
//! polynomial-coefficient pipeline it augments.
//!
//! Requires the `l1-adaptive` Cargo feature to be on (otherwise the
//! L1 block parses fine but the runner has no augmentation to install
//! and the comparison degenerates).

#![cfg(feature = "l1-adaptive")]
#![allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]

use std::fs;
use std::path::{Path, PathBuf};

use arrow::array::Float64Array;
use assert_cmd::assert::OutputAssertExt;
use assert_cmd::cargo::CommandCargoExt;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use openbmp_fc::trajectory::{
    MinimumSnapTrajectory, MinimumSnapWaypoint, YawProfile, flat_output_attitude_reference,
};
use openbmp_testkit::tolerance::ToleranceTable;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use tempfile::{Builder, TempDir};

struct RunOutput {
    parquet: PathBuf,
    kernel_steps: u32,
}

fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .expect("workspace root must exist")
}

fn openbmp() -> std::process::Command {
    std::process::Command::cargo_bin("openbmp").expect("binary built")
}

fn tolerance_table_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/expected/diff-flatness-figure-eight-l1.toml")
}

fn tempdir(label: &str) -> TempDir {
    Builder::new()
        .prefix(&format!("openbmp-{label}-"))
        .tempdir()
        .expect("tempdir")
}

fn parse_kernel_steps(stdout: &str) -> u32 {
    let mut previous: Option<&str> = None;
    for token in stdout.split_whitespace() {
        if token.starts_with("steps") {
            return previous
                .expect("step count before steps token")
                .parse()
                .expect("kernel step count");
        }
        previous = Some(token);
    }
    panic!("stdout did not contain kernel step count: {stdout}");
}

fn run_to_parquet(scenario_dir: &str, label: &str) -> RunOutput {
    let scenario = workspace_root().join(format!("scenarios/{scenario_dir}/scenario.toml"));
    let temp = tempdir(label);
    let parquet = temp.path().join(format!("{scenario_dir}.parquet"));
    let mut cmd = openbmp();
    cmd.arg("run")
        .arg(&scenario)
        .arg("--output-parquet")
        .arg(&parquet);
    let assert = cmd.assert().success();
    let stdout = String::from_utf8_lossy(&assert.get_output().stdout).to_string();
    assert!(
        stdout.starts_with("openbmp run: ok"),
        "stdout was: {stdout}"
    );
    let kernel_steps = parse_kernel_steps(&stdout);
    assert!(parquet.exists(), "parquet must be written");
    let copy = std::env::temp_dir().join(format!("openbmp-{label}.parquet"));
    let _ = fs::remove_file(&copy);
    fs::copy(&parquet, &copy).expect("copy parquet");
    RunOutput {
        parquet: copy,
        kernel_steps,
    }
}

fn read_f64_column(parquet: &Path, column_name: &str) -> Vec<f64> {
    let file = fs::File::open(parquet).expect("open parquet");
    let builder = ParquetRecordBatchReaderBuilder::try_new(file).expect("parquet builder");
    let schema = builder.schema().clone();
    let col = schema
        .index_of(column_name)
        .unwrap_or_else(|_| panic!("{column_name} column present in parquet schema"));
    let reader = builder.build().expect("parquet reader");
    let mut out: Vec<f64> = Vec::new();
    for batch in reader {
        let batch = batch.expect("read batch");
        let arr = batch
            .column(col)
            .as_any()
            .downcast_ref::<Float64Array>()
            .unwrap_or_else(|| panic!("{column_name} is float64"));
        for i in 0..arr.len() {
            out.push(arr.value(i));
        }
    }
    out
}

fn max_angular_velocity_magnitude_rad_s(parquet: &Path) -> f64 {
    let wx = read_f64_column(parquet, "angular_velocity.x_rad_s");
    let wy = read_f64_column(parquet, "angular_velocity.y_rad_s");
    let wz = read_f64_column(parquet, "angular_velocity.z_rad_s");
    let mut max = 0.0_f64;
    for ((x, y), z) in wx.iter().zip(wy.iter()).zip(wz.iter()) {
        let magnitude = (x * x + y * y + z * z).sqrt();
        if magnitude > max {
            max = magnitude;
        }
    }
    max
}

fn max_quaternion_norm_error(parquet: &Path) -> f64 {
    let qx = read_f64_column(parquet, "attitude.q_x");
    let qy = read_f64_column(parquet, "attitude.q_y");
    let qz = read_f64_column(parquet, "attitude.q_z");
    let qw = read_f64_column(parquet, "attitude.q_w");
    let mut max = 0.0_f64;
    for (((x, y), z), w) in qx.iter().zip(qy.iter()).zip(qz.iter()).zip(qw.iter()) {
        let err = (x * x + y * y + z * z + w * w).sqrt() - 1.0;
        max = max.max(err.abs());
    }
    max
}

fn trajectory_reference_for(scenario_dir: &str) -> (MinimumSnapTrajectory, f64) {
    let scenario_path = workspace_root().join(format!("scenarios/{scenario_dir}/scenario.toml"));
    let scenario = openbmp_scenario::Scenario::from_file(&scenario_path)
        .expect("scenario parses for trajectory reference");
    let trajectory = scenario
        .document
        .fc
        .as_ref()
        .and_then(|fc| fc.trajectory.as_ref())
        .expect("scenario has fc.trajectory");
    let waypoints = trajectory
        .waypoints
        .iter()
        .map(|w| MinimumSnapWaypoint {
            position_eci_m: Vector3::new(
                w.position_eci_m[0],
                w.position_eci_m[1],
                w.position_eci_m[2],
            ),
            time_s: w.time_s,
        })
        .collect();
    (
        MinimumSnapTrajectory::new(waypoints).expect("minimum-snap reference builds"),
        trajectory.yaw_rad.unwrap_or(0.0),
    )
}

fn max_attitude_tracking_error_rad(parquet: &Path, scenario_dir: &str) -> f64 {
    let (trajectory, yaw_rad) = trajectory_reference_for(scenario_dir);
    let time = read_f64_column(parquet, "time_s");
    let qx = read_f64_column(parquet, "attitude.q_x");
    let qy = read_f64_column(parquet, "attitude.q_y");
    let qz = read_f64_column(parquet, "attitude.q_z");
    let qw = read_f64_column(parquet, "attitude.q_w");
    let mut max = 0.0_f64;
    for ((((t, x), y), z), w) in time
        .iter()
        .zip(qx.iter())
        .zip(qy.iter())
        .zip(qz.iter())
        .zip(qw.iter())
    {
        let flat = trajectory.evaluate(*t);
        let Some(reference) = flat_output_attitude_reference(
            &flat,
            YawProfile {
                yaw_rad,
                ..YawProfile::default()
            },
        ) else {
            continue;
        };
        let measured = UnitQuaternion::new_normalize(Quaternion::new(*w, *x, *y, *z));
        let error = reference.q_body_to_eci.inverse() * measured;
        max = max.max(error.angle().abs());
    }
    max
}

#[test]
fn diff_flatness_l1_outperforms_pid_under_roll_axis_fault() {
    let table = ToleranceTable::from_path(tolerance_table_path()).expect("tolerance table");

    let baseline = run_to_parquet(
        "diff-flatness-figure-eight-baseline",
        "l1-robustness-baseline",
    );
    let l1 = run_to_parquet("diff-flatness-figure-eight-l1", "l1-robustness-l1");

    table
        .check_metric("kernel_steps_baseline", f64::from(baseline.kernel_steps))
        .expect("baseline kernel_steps within tolerance");
    table
        .check_metric("kernel_steps_l1", f64::from(l1.kernel_steps))
        .expect("l1 kernel_steps within tolerance");

    let omega_baseline = max_angular_velocity_magnitude_rad_s(&baseline.parquet);
    let omega_l1 = max_angular_velocity_magnitude_rad_s(&l1.parquet);

    table
        .check_metric(
            "max_angular_velocity_magnitude_baseline_rad_s",
            omega_baseline,
        )
        .unwrap_or_else(|err| {
            panic!("baseline max|omega| = {omega_baseline} outside tolerance: {err}")
        });
    table
        .check_metric("max_angular_velocity_magnitude_l1_rad_s", omega_l1)
        .unwrap_or_else(|err| panic!("l1 max|omega| = {omega_l1} outside tolerance: {err}"));

    let q_err_baseline = max_quaternion_norm_error(&baseline.parquet);
    let q_err_l1 = max_quaternion_norm_error(&l1.parquet);
    table
        .check_metric("max_quaternion_norm_error_baseline", q_err_baseline)
        .unwrap_or_else(|err| panic!("baseline qerr = {q_err_baseline} outside tolerance: {err}"));
    table
        .check_metric("max_quaternion_norm_error_l1", q_err_l1)
        .unwrap_or_else(|err| panic!("l1 qerr = {q_err_l1} outside tolerance: {err}"));

    let attitude_err_baseline =
        max_attitude_tracking_error_rad(&baseline.parquet, "diff-flatness-figure-eight-baseline");
    let attitude_err_l1 =
        max_attitude_tracking_error_rad(&l1.parquet, "diff-flatness-figure-eight-l1");
    table
        .check_metric(
            "max_attitude_tracking_error_baseline_rad",
            attitude_err_baseline,
        )
        .unwrap_or_else(|err| {
            panic!("baseline attitude error = {attitude_err_baseline} outside tolerance: {err}")
        });
    table
        .check_metric("max_attitude_tracking_error_l1_rad", attitude_err_l1)
        .unwrap_or_else(|err| {
            panic!("l1 attitude error = {attitude_err_l1} outside tolerance: {err}")
        });

    // The disturbance is a matched roll-axis ReducedRate fault. The
    // baseline must actually be perturbed (otherwise the comparison
    // is meaningless), and the L1 augmentation must materially tighten
    // the closed-loop response without degrading attitude tracking.
    assert!(
        omega_baseline > 1.0e-2,
        "baseline must be visibly perturbed by the fault; max|omega| was {omega_baseline}"
    );
    let ratio = omega_l1 / omega_baseline;
    assert!(
        ratio <= 0.5,
        "L1 max|omega| ({omega_l1:.6}) must be ≤ 0.5 × baseline max|omega| ({omega_baseline:.6}); \
         ratio was {ratio:.3}"
    );
    assert!(
        attitude_err_l1 <= attitude_err_baseline,
        "L1 attitude tracking error ({attitude_err_l1:.6} rad) must not exceed baseline \
         ({attitude_err_baseline:.6} rad)"
    );

    let _ = fs::remove_file(&baseline.parquet);
    let _ = fs::remove_file(&l1.parquet);
}

#[test]
fn diff_flatness_figure_eight_l1_is_byte_stable_across_reruns() {
    let a = run_to_parquet("diff-flatness-figure-eight-l1", "l1-rerun-a");
    let b = run_to_parquet("diff-flatness-figure-eight-l1", "l1-rerun-b");
    let bytes_a = fs::read(&a.parquet).expect("read a");
    let bytes_b = fs::read(&b.parquet).expect("read b");
    assert_eq!(
        bytes_a, bytes_b,
        "two reruns of diff-flatness-figure-eight-l1 must produce byte-identical Parquet"
    );
    let _ = fs::remove_file(&a.parquet);
    let _ = fs::remove_file(&b.parquet);
}

#[test]
fn diff_flatness_figure_eight_baseline_is_byte_stable_across_reruns() {
    let a = run_to_parquet("diff-flatness-figure-eight-baseline", "baseline-rerun-a");
    let b = run_to_parquet("diff-flatness-figure-eight-baseline", "baseline-rerun-b");
    let bytes_a = fs::read(&a.parquet).expect("read a");
    let bytes_b = fs::read(&b.parquet).expect("read b");
    assert_eq!(
        bytes_a, bytes_b,
        "two reruns of diff-flatness-figure-eight-baseline must produce byte-identical Parquet"
    );
    let _ = fs::remove_file(&a.parquet);
    let _ = fs::remove_file(&b.parquet);
}
