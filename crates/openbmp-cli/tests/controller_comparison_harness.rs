//! Phase-5.A.3.D controller comparison harness.
//!
//! Runs four sibling scenarios that share the same vehicle, the same
//! Mellinger-Kumar minimum-snap figure-eight reference, and the same
//! `EffectorFault::ReducedRate { factor = 0.7 }` matched
//! roll-axis disturbance. The only difference between them is the
//! rate-loop kind:
//!
//! - `diff-flatness-figure-eight-baseline` — PID rate loop (Phase-4
//!   default, no augmentation)
//! - `diff-flatness-figure-eight-l1` — PID rate loop + Cao-Hovakimyan
//!   L1 adaptive augmentation (Phase 5.A.2.C)
//! - `diff-flatness-figure-eight-lqr-fault` — per-axis LQR rate loop
//!   (Phase 5.A.3.B)
//! - `diff-flatness-figure-eight-indi-fault` — per-axis INDI rate
//!   loop (Phase 5.A.3.C)
//!
//! For each scenario the harness reads the post-liftoff window of the
//! emitted Parquet and computes:
//!
//! - max body-frame angular-velocity magnitude `|ω|` (rad/s);
//! - RMS body-frame angular-velocity magnitude (rad/s) — proxy for
//!   how tightly the rate loop tracks the trajectory's commanded
//!   body rates;
//! - per-axis peak commanded torque `|τ_x|`, `|τ_y|`, `|τ_z|` (N·m,
//!   read from the `direct_torque` effector telemetry);
//! - saturation fraction — fraction of post-liftoff per-axis samples
//!   at which the actuator hit the ±0.35 N·m limit.
//!
//! The harness formats these as a markdown table and asserts byte
//! equality against a committed snapshot fixture
//! `tests/expected/controller-comparison.md`. To regenerate the
//! fixture (e.g. after re-tuning a rate-loop), run with
//! `UPDATE_EXPECT=1 cargo test -p openbmp-cli --features
//! l1-adaptive,lqr,indi --test controller_comparison_harness`.
//!
//! All four scenarios are individually byte-stable across reruns
//! (asserted by their per-scenario e2e tests); the harness does not
//! re-run that gate. The metric computation here is pure scalar
//! arithmetic, so the markdown rendering is bit-stable too — any
//! drift is a real underlying behaviour change to flag.

#![cfg(all(feature = "l1-adaptive", feature = "lqr", feature = "indi"))]
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::expect_used,
    clippy::needless_range_loop,
    clippy::panic,
    clippy::unwrap_used
)]

use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

use arrow::array::Float64Array;
use assert_cmd::assert::OutputAssertExt;
use assert_cmd::cargo::CommandCargoExt;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use tempfile::{Builder, TempDir};

const SCENARIO_STOP_S: f64 = 8.0;
const LIFTOFF_TIME_S: f64 = 0.05;
const SATURATION_THRESHOLD_NM: f64 = 0.349_999;

/// One row in the comparison table.
#[derive(Clone, Debug)]
struct ScenarioMetrics {
    label: &'static str,
    max_omega_rad_s: f64,
    rms_omega_rad_s: f64,
    peak_torque_per_axis_n_m: [f64; 3],
    saturation_fraction: f64,
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

fn tempdir(label: &str) -> TempDir {
    Builder::new()
        .prefix(&format!("openbmp-{label}-"))
        .tempdir()
        .expect("tempdir")
}

fn run_scenario(scenario_dir: &str, label: &str) -> PathBuf {
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
        "{scenario_dir} run failed; stdout was: {stdout}"
    );
    let copy = std::env::temp_dir().join(format!("openbmp-comparison-{label}.parquet"));
    let _ = fs::remove_file(&copy);
    fs::copy(&parquet, &copy).expect("copy parquet");
    copy
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

fn compute_metrics(label: &'static str, parquet: &Path) -> ScenarioMetrics {
    let wx = read_f64_column(parquet, "angular_velocity.x_rad_s");
    let wy = read_f64_column(parquet, "angular_velocity.y_rad_s");
    let wz = read_f64_column(parquet, "angular_velocity.z_rad_s");
    let total_samples = wx.len();
    assert!(total_samples > 0, "{label}: parquet was empty");
    let liftoff_idx = post_liftoff_start(total_samples);

    let mut max_omega = 0.0_f64;
    let mut sum_sq = 0.0_f64;
    let mut omega_count = 0_usize;
    for i in liftoff_idx..total_samples {
        let m = (wx[i] * wx[i] + wy[i] * wy[i] + wz[i] * wz[i]).sqrt();
        if m > max_omega {
            max_omega = m;
        }
        sum_sq += m * m;
        omega_count += 1;
    }
    let rms_omega = (sum_sq / omega_count as f64).sqrt();

    let mut peak_torque_per_axis_n_m = [0.0_f64; 3];
    let mut saturation_count = 0_usize;
    let axis_columns = [
        ("effector.roll-torque.actual", 0_usize),
        ("effector.pitch-torque.actual", 1_usize),
        ("effector.yaw-torque.actual", 2_usize),
    ];
    for (column, axis) in axis_columns {
        let vals = read_f64_column(parquet, column);
        for i in liftoff_idx..vals.len() {
            let v = vals[i].abs();
            if v > peak_torque_per_axis_n_m[axis] {
                peak_torque_per_axis_n_m[axis] = v;
            }
            if v >= SATURATION_THRESHOLD_NM {
                saturation_count += 1;
            }
        }
    }
    let saturation_fraction =
        saturation_count as f64 / (omega_count as f64 * axis_columns.len() as f64);

    ScenarioMetrics {
        label,
        max_omega_rad_s: max_omega,
        rms_omega_rad_s: rms_omega,
        peak_torque_per_axis_n_m,
        saturation_fraction,
    }
}

fn post_liftoff_start(total_samples: usize) -> usize {
    let liftoff_fraction = LIFTOFF_TIME_S / SCENARIO_STOP_S;
    let raw = liftoff_fraction * total_samples as f64;
    raw as usize
}

fn render_markdown(rows: &[ScenarioMetrics]) -> String {
    let mut out = String::new();
    let _ = writeln!(
        &mut out,
        "# Controller comparison — figure-eight under matched roll-axis ReducedRate fault"
    );
    let _ = writeln!(&mut out);
    let _ = writeln!(
        &mut out,
        "Common scenario family: `scenarios/diff-flatness-figure-eight-*`. Common disturbance:"
    );
    let _ = writeln!(
        &mut out,
        "`EffectorFault::ReducedRate {{ factor = 0.7 }}` on the roll-torque effector."
    );
    let _ = writeln!(
        &mut out,
        "Metrics computed over the post-liftoff window (`t > {LIFTOFF_TIME_S} s`) of the"
    );
    let _ = writeln!(
        &mut out,
        "8 s scenario. Saturation fraction is the fraction of per-axis samples at which"
    );
    let _ = writeln!(
        &mut out,
        "any direct-torque effector hit the ±0.35 N·m rail."
    );
    let _ = writeln!(&mut out);
    let _ = writeln!(
        &mut out,
        "| Rate loop | max \\|ω\\| (rad/s) | RMS \\|ω\\| (rad/s) | Peak τx (N·m) | Peak τy (N·m) | Peak τz (N·m) | Saturation fraction |"
    );
    let _ = writeln!(&mut out, "|---|---|---|---|---|---|---|");
    for row in rows {
        let _ = writeln!(
            &mut out,
            "| {} | {:.4} | {:.4} | {:.4} | {:.4} | {:.4} | {:.4} |",
            row.label,
            row.max_omega_rad_s,
            row.rms_omega_rad_s,
            row.peak_torque_per_axis_n_m[0],
            row.peak_torque_per_axis_n_m[1],
            row.peak_torque_per_axis_n_m[2],
            row.saturation_fraction,
        );
    }
    out
}

fn fixture_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/expected/controller-comparison.md")
}

#[test]
fn controller_comparison_matches_fixture() {
    let scenarios = [
        (
            "diff-flatness-figure-eight-baseline",
            "PID baseline",
            "baseline",
        ),
        ("diff-flatness-figure-eight-l1", "PID + L1", "l1"),
        ("diff-flatness-figure-eight-lqr-fault", "LQR", "lqr"),
        ("diff-flatness-figure-eight-indi-fault", "INDI", "indi"),
    ];
    let mut rows = Vec::new();
    for (scenario_dir, label, run_label) in scenarios {
        let parquet = run_scenario(scenario_dir, run_label);
        rows.push(compute_metrics(label, &parquet));
        let _ = fs::remove_file(&parquet);
    }
    let rendered = render_markdown(&rows);

    if std::env::var("UPDATE_EXPECT").is_ok() {
        fs::write(fixture_path(), &rendered).expect("update fixture");
        return;
    }
    let expected = fs::read_to_string(fixture_path()).unwrap_or_else(|_| {
        panic!(
            "expected fixture {} missing — run with UPDATE_EXPECT=1 to generate it",
            fixture_path().display()
        )
    });
    assert_eq!(
        rendered, expected,
        "controller-comparison fixture drift detected. Re-run with UPDATE_EXPECT=1 to update."
    );

    // Sanity assertions independent of the fixture, so the harness
    // catches a pathological regression even if the fixture has
    // drifted into a wrong-but-frozen state.
    let pid_max_omega = rows
        .iter()
        .find(|r| r.label == "PID baseline")
        .expect("PID baseline row")
        .max_omega_rad_s;
    let l1_max_omega = rows
        .iter()
        .find(|r| r.label == "PID + L1")
        .expect("PID + L1 row")
        .max_omega_rad_s;
    assert!(
        l1_max_omega < 0.5 * pid_max_omega,
        "L1 must beat PID baseline by ≥ 2× under the same fault; \
         L1 max|ω| = {l1_max_omega:.4}, PID = {pid_max_omega:.4}"
    );
    for row in &rows {
        assert!(
            row.max_omega_rad_s.is_finite() && row.max_omega_rad_s < 5.0,
            "{}: max|ω| = {} not bounded under 5 rad/s",
            row.label,
            row.max_omega_rad_s
        );
        assert!(
            (0.0..=1.0).contains(&row.saturation_fraction),
            "{}: saturation_fraction = {} outside [0, 1]",
            row.label,
            row.saturation_fraction
        );
    }
}
