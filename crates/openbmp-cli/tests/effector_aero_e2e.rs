//! Phase-3.5.D end-to-end test: schema-2 aero deck consumes
//! `EffectorState.actual` from the runner-side rack.
//!
//! Asserts the §3.5 exit criterion: the synthetic single-elevon
//! schema-2 scenario
//! `scenarios/effector-elevon-aero/single-elevon-aero-deflected.toml`
//! runs via `openbmp run`, the kernel completes with
//! `StopReason::EndTime`, and the per-model aero-force telemetry
//! at `t = 2.5 s` differs between a zero-deflection baseline run
//! and a deflected run (the elevon at +20°) by more than the 5%
//! relative tolerance the deck's CD perturbation guarantees.
//!
//! The baseline scenario is generated at test time from the
//! shipped scenario by replacing the elevon command schedule's
//! `after = 20.0` with `after = 0.0` and rewriting the
//! `[telemetry].output.parquet` path so the two runs do not
//! collide.

#![allow(clippy::expect_used, clippy::float_cmp, clippy::panic)]

use std::fs;
use std::path::{Path, PathBuf};

use arrow::array::Float64Array;
use assert_cmd::assert::OutputAssertExt;
use assert_cmd::cargo::CommandCargoExt;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use tempfile::{Builder, TempDir};

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

/// Read all rows of a single `f64` Parquet column into a `Vec<f64>`.
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

fn read_time_column(parquet: &Path) -> Vec<f64> {
    read_f64_column(parquet, "time_s")
}

/// Run the scenario at `scenario_path` and return the resulting
/// Parquet path. The Parquet lives in `tempdir`; the caller owns
/// it.
fn run_scenario(scenario_path: &Path, parquet_path: &Path) {
    let mut cmd = openbmp();
    cmd.arg("run")
        .arg(scenario_path)
        .arg("--output-parquet")
        .arg(parquet_path);
    let assert = cmd.assert().success();
    let stdout = String::from_utf8_lossy(&assert.get_output().stdout).to_string();
    assert!(
        stdout.starts_with("openbmp run: ok"),
        "stdout was: {stdout}"
    );
    assert!(
        parquet_path.exists(),
        "parquet file should have been written"
    );
}

/// Schema-2 deflected scenario lives in the workspace and points at
/// the schema-2 deck via a relative path. The baseline scenario is
/// generated at test time by string-replacing the elevon's `after`
/// value to `0.0`. The test temp dir holds the rewritten scenario
/// AND its parquet output, sitting in the same workspace location
/// pattern so the relative deck path still resolves.
fn write_baseline_scenario(temp_dir: &Path) -> PathBuf {
    let original =
        workspace_root().join("scenarios/effector-elevon-aero/single-elevon-aero-deflected.toml");
    let original_text = fs::read_to_string(&original).expect("read shipped scenario");
    // Rewrite the elevon step to hold the elevon at 0° throughout.
    let rewritten = original_text.replace(
        "command_schedule = { kind = \"step_at\", time_s = 1.0, before = 0.0, after = 20.0 }",
        "command_schedule = { kind = \"step_at\", time_s = 1.0, before = 0.0, after = 0.0 }",
    );
    assert_ne!(
        rewritten, original_text,
        "fixture shape changed: baseline rewriter did not find the elevon step",
    );
    // The deck path in the scenario is relative ("../../data/aero/...").
    // Place the rewritten scenario at a sibling location to the
    // original so the relative path still resolves.
    let scenarios_dir = temp_dir.join("scenarios").join("effector-elevon-aero");
    fs::create_dir_all(&scenarios_dir).expect("create temp scenarios dir");
    // Symlink (or copy) `data/` under the temp root so the relative
    // path works.
    let temp_data = temp_dir.join("data");
    if !temp_data.exists() {
        let real_data = workspace_root().join("data");
        #[cfg(unix)]
        std::os::unix::fs::symlink(&real_data, &temp_data).expect("symlink data dir");
        #[cfg(not(unix))]
        copy_dir_all(&real_data, &temp_data).expect("copy data dir");
    }
    let baseline_path = scenarios_dir.join("single-elevon-aero-baseline.toml");
    fs::write(&baseline_path, rewritten).expect("write baseline scenario");
    baseline_path
}

#[cfg(not(unix))]
fn copy_dir_all(src: &Path, dst: &Path) -> std::io::Result<()> {
    fs::create_dir_all(dst)?;
    for entry in fs::read_dir(src)? {
        let entry = entry?;
        let ty = entry.file_type()?;
        if ty.is_dir() {
            copy_dir_all(&entry.path(), &dst.join(entry.file_name()))?;
        } else {
            fs::copy(entry.path(), dst.join(entry.file_name()))?;
        }
    }
    Ok(())
}

#[test]
fn single_elevon_aero_scenario_runs_to_completion() {
    let scenario =
        workspace_root().join("scenarios/effector-elevon-aero/single-elevon-aero-deflected.toml");
    let temp = tempdir("effector-elevon-aero-runs");
    let parquet = temp.path().join("deflected.parquet");
    run_scenario(&scenario, &parquet);

    // The schema-2 effector channel must exist.
    let file = fs::File::open(&parquet).expect("open parquet");
    let builder = ParquetRecordBatchReaderBuilder::try_new(file).expect("parquet builder");
    let schema = builder.schema().clone();
    let _ = schema
        .index_of("effector.delta_e.actual")
        .expect("effector.delta_e.actual column present");

    let times = read_time_column(&parquet);
    let actuals = read_f64_column(&parquet, "effector.delta_e.actual");
    assert!(times.len() > 1);

    // Step 0: deflection equals the configured initial position (0°).
    assert!(actuals[0].abs() < 1.0e-12);

    // After the step + latency + a few first-order time constants,
    // the actual must have moved toward the commanded 20°.
    let post_idx = times
        .iter()
        .position(|&t| t >= 1.5)
        .expect("scenario must include t >= 1.5");
    assert!(
        actuals[post_idx] > 5.0,
        "after step + lag, actual should be well past 0° at t = {}, got {}",
        times[post_idx],
        actuals[post_idx]
    );
}

#[test]
fn schema2_aero_force_differs_between_deflected_and_baseline_runs() {
    let temp = tempdir("effector-elevon-aero-compare");

    // Run the deflected (shipped) scenario.
    let deflected_scenario =
        workspace_root().join("scenarios/effector-elevon-aero/single-elevon-aero-deflected.toml");
    let deflected_parquet = temp.path().join("deflected.parquet");
    run_scenario(&deflected_scenario, &deflected_parquet);

    // Generate the baseline scenario (elevon held at 0°) and run it.
    let baseline_scenario = write_baseline_scenario(temp.path());
    let baseline_parquet = temp.path().join("baseline.parquet");
    run_scenario(&baseline_scenario, &baseline_parquet);

    // Compare aero-force magnitude at t = 2.5 s. The deflected run
    // sees CD ≈ 0.65 vs the baseline's 0.45 (deck's
    // 0.010·|δ_e_deg| perturbation), so the aero force should
    // differ by ≥5% even after accounting for shared velocity /
    // altitude transients.
    let deflected_times = read_time_column(&deflected_parquet);
    let baseline_times = read_time_column(&baseline_parquet);
    assert_eq!(
        deflected_times, baseline_times,
        "two scenario runs must share the same time grid"
    );

    let deflected_z = read_f64_column(&deflected_parquet, "force.aero.z_n");
    let baseline_z = read_f64_column(&baseline_parquet, "force.aero.z_n");
    assert_eq!(deflected_z.len(), baseline_z.len());

    // Index at t = 2.5 s.
    let cmp_idx = deflected_times
        .iter()
        .position(|&t| t >= 2.5)
        .expect("scenario must include t >= 2.5");
    let f_deflected = deflected_z[cmp_idx];
    let f_baseline = baseline_z[cmp_idx];

    // Both should be finite and (typically) negative (drag opposing
    // ECI +z velocity during ascent / descent).
    assert!(
        f_deflected.is_finite() && f_baseline.is_finite(),
        "aero forces must be finite at t=2.5s; got deflected={f_deflected}, baseline={f_baseline}",
    );

    // Relative difference: at least 5% — the deck's CD perturbation
    // alone gives ~44% at full deflection. We require ≥5% to allow
    // the velocity / altitude transients to dampen the signal in the
    // mid-flight envelope.
    let denom = f_baseline.abs().max(f_deflected.abs()).max(1.0e-9);
    let rel_diff = (f_deflected - f_baseline).abs() / denom;
    assert!(
        rel_diff >= 0.05,
        "aero force at t=2.5s should differ by ≥5% between deflected and baseline runs; \
         got deflected = {f_deflected} N, baseline = {f_baseline} N, rel_diff = {rel_diff}",
    );

    // Sanity: the deflected run's elevon really did deflect.
    let deflected_actuals = read_f64_column(&deflected_parquet, "effector.delta_e.actual");
    let baseline_actuals = read_f64_column(&baseline_parquet, "effector.delta_e.actual");
    assert!(
        deflected_actuals[cmp_idx] > 5.0,
        "deflected run must have a non-trivial deflection at t=2.5s; got {}",
        deflected_actuals[cmp_idx]
    );
    assert!(
        baseline_actuals[cmp_idx].abs() < 1.0e-9,
        "baseline run must hold the elevon at 0°; got {}",
        baseline_actuals[cmp_idx]
    );
}

#[test]
fn schema2_run_byte_stable_across_two_invocations() {
    let scenario =
        workspace_root().join("scenarios/effector-elevon-aero/single-elevon-aero-deflected.toml");
    let temp = tempdir("effector-elevon-aero-bytestable");
    let a = temp.path().join("a.parquet");
    let b = temp.path().join("b.parquet");
    run_scenario(&scenario, &a);
    run_scenario(&scenario, &b);
    let bytes_a = fs::read(&a).expect("read a");
    let bytes_b = fs::read(&b).expect("read b");
    assert_eq!(
        bytes_a, bytes_b,
        "two runs of the same schema-2 scenario must produce byte-identical Parquet"
    );
}

#[test]
fn schema2_baseline_run_byte_stable_across_two_invocations() {
    let temp = tempdir("effector-elevon-aero-baseline-bytestable");
    let scenario = write_baseline_scenario(temp.path());
    let a = temp.path().join("baseline-a.parquet");
    let b = temp.path().join("baseline-b.parquet");
    run_scenario(&scenario, &a);
    run_scenario(&scenario, &b);
    let bytes_a = fs::read(&a).expect("read baseline a");
    let bytes_b = fs::read(&b).expect("read baseline b");
    assert_eq!(
        bytes_a, bytes_b,
        "two baseline schema-2 runs must produce byte-identical Parquet"
    );
}
