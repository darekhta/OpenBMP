//! End-to-end test: per-axis LQR rate loop tracks the
//! diff-flatness figure-eight without runaway and stays byte-stable
//! across reruns.
//!
//! `scenarios/diff-flatness-figure-eight-lqr/scenario.toml` is a
//! sibling of `scenarios/diff-flatness-figure-eight`; the only
//! difference is `rate_loop_kind = "lqr"` plus the
//! `[fc.autopilot_params.lqr]` cost-weight block. The runner solves
//! the per-axis DARE at scenario load using the diagonal inertia of
//! this single-body assembly (1.0 kg·m² on each axis here) and the loop step
//! `time.dt_s = 0.001 s`.
//!
//! Asserts: scenario completes 8000 kernel steps with end-time stop
//! reason; rigid-body quaternion stays normalised after projection;
//! body-frame angular velocity remains bounded; two reruns produce
//! byte-identical Parquet (DARE solver is deterministic).
//!
//! Closed-loop comparison against the PID baseline + L1 sibling
//! lives in the controller comparison harness.

#![cfg(feature = "lqr")]
#![allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]

use std::fs;
use std::io::Write as _;
use std::path::{Path, PathBuf};

use arrow::array::Float64Array;
use assert_cmd::assert::OutputAssertExt;
use assert_cmd::cargo::CommandCargoExt;
use openbmp_testkit::tolerance::ToleranceTable;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use tempfile::{Builder, NamedTempFile, TempDir};

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
        .join("tests/expected/diff-flatness-figure-eight-lqr.toml")
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
    let scenario = workspace_root().join("scenarios/diff-flatness-figure-eight-lqr/scenario.toml");
    let temp = tempdir(label);
    let parquet = temp.path().join("diff-flatness-figure-eight-lqr.parquet");
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

fn scenario_with_second_body() -> NamedTempFile {
    let scenario = workspace_root().join("scenarios/diff-flatness-figure-eight-lqr/scenario.toml");
    let text = fs::read_to_string(&scenario).expect("read LQR scenario");
    let inserted = text.replace(
        "[[vehicle.assembly.effectors]]\nid               = \"roll-torque\"",
        "[[vehicle.assembly.bodies]]\n\
id          = \"secondary\"\n\
geometry    = { kind = \"reference\", length_m = 1.0, area_m2 = 1.0 }\n\
dry_mass_kg = 1.0\n\
dry_inertia_body_kg_m2 = [\n\
  [1.0, 0.0, 0.0],\n\
  [0.0, 1.0, 0.0],\n\
  [0.0, 0.0, 1.0],\n\
]\n\n\
[[vehicle.assembly.effectors]]\n\
id               = \"roll-torque\"",
    );
    assert_ne!(text, inserted, "fixture insertion needle must match");
    let mut file = NamedTempFile::new_in(scenario.parent().expect("scenario parent"))
        .expect("temp scenario in LQR directory");
    file.write_all(inserted.as_bytes())
        .expect("write temp LQR scenario");
    file
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

#[test]
fn diff_flatness_lqr_runs_to_completion() {
    let run = run_to_parquet("lqr-runs");
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
        "LQR figure-eight must rotate the rigid body; max angular velocity was {max_omega}"
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
fn diff_flatness_lqr_is_byte_stable_across_reruns() {
    let a = run_to_parquet("lqr-rerun-a");
    let b = run_to_parquet("lqr-rerun-b");
    let bytes_a = fs::read(&a.parquet).expect("read a");
    let bytes_b = fs::read(&b.parquet).expect("read b");
    assert_eq!(
        bytes_a, bytes_b,
        "two reruns of diff-flatness-figure-eight-lqr must produce byte-identical Parquet \
         (DARE solver is deterministic and the closed-loop pipeline preserves that)"
    );
    let _ = fs::remove_file(&a.parquet);
    let _ = fs::remove_file(&b.parquet);
}

#[test]
fn diff_flatness_lqr_rejects_multi_body_inertia_precondition() {
    let scenario = scenario_with_second_body();
    let temp = tempdir("lqr-multi-body");
    let parquet = temp.path().join("lqr-multi-body.parquet");
    let mut cmd = openbmp();
    cmd.arg("run")
        .arg(scenario.path())
        .arg("--output-parquet")
        .arg(&parquet);
    cmd.assert().failure().stderr(predicates::str::contains(
        "rate_loop_kind = \"lqr\" requires exactly one [[vehicle.assembly.bodies]] entry",
    ));
}
