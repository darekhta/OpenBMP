//! Phase-5.A.5 end-to-end test: prioritised redistributed
//! allocator splits the autopilot's roll demand evenly across two
//! `direct_torque` effectors, with byte-stable Parquet across
//! reruns.
//!
//! `scenarios/diff-flatness-figure-eight-allocator/scenario.toml`
//! is a sibling of `scenarios/diff-flatness-figure-eight`; the only
//! differences are the over-actuated roll axis (two `roll-torque-*`
//! effectors at ±0.2 N·m each) and the
//! `[fc.autopilot_allocation]` block selecting
//! `prioritised_redistributed` with explicit `axis_priority =
//! ["roll", "pitch", "yaw"]`.
//!
//! Asserts:
//!
//! - 8000 kernel steps with end-time stop;
//! - rigid-body quaternion stays normalised;
//! - max body-frame angular-velocity magnitude bounded under
//!   tolerance;
//! - both roll-torque effectors receive bit-identical commands at
//!   every tick (equal-capacity proportional split → exact equality
//!   on the reference platform);
//! - two reruns produce byte-identical Parquet.

#![allow(
    clippy::expect_used,
    clippy::float_cmp,
    clippy::panic,
    clippy::unwrap_used
)]

use std::fs;
use std::path::{Path, PathBuf};

use arrow::array::Float64Array;
use assert_cmd::assert::OutputAssertExt;
use assert_cmd::cargo::CommandCargoExt;
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
        .join("tests/expected/diff-flatness-figure-eight-allocator.toml")
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

fn run_to_parquet(label: &str) -> RunOutput {
    let scenario =
        workspace_root().join("scenarios/diff-flatness-figure-eight-allocator/scenario.toml");
    let temp = tempdir(label);
    let parquet = temp
        .path()
        .join("diff-flatness-figure-eight-allocator.parquet");
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
        let m = (x * x + y * y + z * z).sqrt();
        if m > max {
            max = m;
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

fn max_pairwise_diff(parquet: &Path, column_a: &str, column_b: &str) -> f64 {
    let va = read_f64_column(parquet, column_a);
    let vb = read_f64_column(parquet, column_b);
    assert_eq!(
        va.len(),
        vb.len(),
        "column lengths differ for {column_a} vs {column_b}"
    );
    let mut max = 0.0_f64;
    for (a, b) in va.iter().zip(&vb) {
        max = max.max((a - b).abs());
    }
    max
}

#[test]
fn diff_flatness_allocator_runs_to_completion() {
    let run = run_to_parquet("allocator-runs");
    let table = ToleranceTable::from_path(tolerance_table_path()).expect("tolerance table");
    table
        .check_metric("kernel_steps", f64::from(run.kernel_steps))
        .expect("kernel_steps within tolerance");
    let max_omega = max_angular_velocity_magnitude_rad_s(&run.parquet);
    table
        .check_metric("max_angular_velocity_magnitude_rad_s", max_omega)
        .unwrap_or_else(|err| {
            panic!("max_angular_velocity_magnitude_rad_s = {max_omega} outside tolerance: {err}")
        });
    assert!(
        max_omega > 1.0e-6,
        "allocator scenario must rotate the rigid body; max angular velocity was {max_omega}"
    );
    let max_q_norm_error = max_quaternion_norm_error(&run.parquet);
    table
        .check_metric("max_quaternion_norm_error", max_q_norm_error)
        .unwrap_or_else(|err| {
            panic!("max_quaternion_norm_error = {max_q_norm_error} outside tolerance: {err}")
        });
    let _ = fs::remove_file(&run.parquet);
}

#[test]
fn diff_flatness_allocator_splits_roll_demand_bit_identically() {
    // Equal-capacity roll-axis effectors must receive bit-identical
    // commands every tick. This is the headline allocator
    // behaviour — exact proportional split with no floating-point
    // drift between the two effectors.
    let run = run_to_parquet("allocator-split");
    let max_diff = max_pairwise_diff(
        &run.parquet,
        "effector.roll-torque-a.actual",
        "effector.roll-torque-b.actual",
    );
    assert_eq!(
        max_diff, 0.0,
        "roll-torque-a and roll-torque-b must be bit-identical under equal-capacity \
         proportional split; max |diff| was {max_diff:.6e}"
    );
    let _ = fs::remove_file(&run.parquet);
}

#[test]
fn diff_flatness_allocator_is_byte_stable_across_reruns() {
    let a = run_to_parquet("allocator-rerun-a");
    let b = run_to_parquet("allocator-rerun-b");
    let bytes_a = fs::read(&a.parquet).expect("read a");
    let bytes_b = fs::read(&b.parquet).expect("read b");
    assert_eq!(
        bytes_a, bytes_b,
        "two reruns of diff-flatness-figure-eight-allocator must produce byte-identical \
         Parquet (the allocator is deterministic; the closed-loop pipeline preserves that)"
    );
    let _ = fs::remove_file(&a.parquet);
    let _ = fs::remove_file(&b.parquet);
}
