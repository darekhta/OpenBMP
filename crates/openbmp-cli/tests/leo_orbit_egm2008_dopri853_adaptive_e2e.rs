//! Phase-5.D.6 end-to-end test: 1 kg point mass in a 400 km / 30°
//! inclined LEO orbit under EGM2008 zonal-harmonic gravity (degrees
//! 2-6), integrated with the **Dormand-Prince 8(5,3) (DOP853)**
//! adaptive embedded RK pair under an I-controller and the SciPy /
//! Hairer combined err5/err3 stabilised error norm.
//!
//! Asserts:
//!
//! 1. The scenario runs to completion via `openbmp_cli::commands::run`
//!    with `stop_label = "end-time"` after 5556 outer kernel steps —
//!    the outer kernel cadence is `dt_s = 1.0` seconds, the same as
//!    the §5.D.4 DOPRI5(4) demo. The DOP853 integrator sub-steps
//!    internally as needed to satisfy `(rtol = 1e-12, atol = 1e-15)`
//!    but the kernel's outer step count is fixed by the time grid.
//! 2. The final ECI radius stays within ±5 km of the initial
//!    `r = 6_778_000 m` — same envelope as the DOPRI5(4) demo,
//!    demonstrating that the 8th-order DOP853 integrator delivers
//!    comparable accuracy on this benign orbit.
//! 3. Two reruns on the same platform produce byte-identical Parquet
//!    — `Dopri853Adaptive` is `StateStable`, which is
//!    within-platform bit-stable.
//!
//! The math-side correctness of `Dopri853Adaptive` (tableau row-sums,
//! 8th-order convergence on a polynomial, err5/err3 norm formula)
//! is covered by the unit tests in `crates/openbmp-sim/src/integrator.rs`.
//! The runner-side dispatch (`(adaptive-explicit, dopri853,
//! state-stable)` triple selection) is covered by the unit tests in
//! `crates/openbmp-cli/src/runner/integrator.rs`.

#![allow(
    clippy::expect_used,
    clippy::float_cmp,
    clippy::panic,
    clippy::struct_field_names,
    clippy::doc_markdown
)]

use std::fs;
use std::path::{Path, PathBuf};

use openbmp_cli::commands::run;
use tempfile::{Builder, TempDir};

const INITIAL_RADIUS_M: f64 = 6_778_000.0;

fn manifest_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn workspace_root() -> PathBuf {
    manifest_dir()
        .parent()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .expect("workspace root")
}

fn scenario_path() -> PathBuf {
    workspace_root().join("scenarios/leo-orbit-egm2008-dopri853-adaptive/scenario.toml")
}

fn toml_literal_path(path: &Path) -> String {
    let path = path.to_string_lossy();
    assert!(
        !path.contains('\''),
        "test temp paths must be representable as TOML literal strings: {path}",
    );
    format!("'{path}'")
}

fn stage_scenario(temp_dir: &Path, label: &str) -> PathBuf {
    let original = fs::read_to_string(scenario_path()).expect("read canonical scenario");
    let parquet = temp_dir.join(format!("{label}.parquet"));
    let rewritten = original.replace(
        "output.parquet = \"out/leo-orbit-egm2008-dopri853-adaptive.parquet\"",
        &format!("output.parquet = {}", toml_literal_path(&parquet)),
    );
    let staged = temp_dir.join(format!("{label}-scenario.toml"));
    fs::write(&staged, rewritten).expect("write staged scenario");
    staged
}

fn tempdir_for(label: &str) -> TempDir {
    Builder::new()
        .prefix(&format!("{label}-"))
        .tempdir()
        .expect("tempdir")
}

#[derive(Debug)]
struct FinalEciState {
    position_x_m: f64,
    position_y_m: f64,
    position_z_m: f64,
}

fn read_final_eci_position(parquet_path: &Path) -> FinalEciState {
    use arrow::array::Float64Array;
    use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;

    let file = fs::File::open(parquet_path).expect("open parquet");
    let builder = ParquetRecordBatchReaderBuilder::try_new(file).expect("parquet builder");
    let schema = builder.schema().clone();
    let px = schema
        .index_of("position_x_m")
        .expect("position_x_m column");
    let py = schema
        .index_of("position_y_m")
        .expect("position_y_m column");
    let pz = schema
        .index_of("position_z_m")
        .expect("position_z_m column");

    let reader = builder.build().expect("parquet reader");
    let mut last = FinalEciState {
        position_x_m: f64::NAN,
        position_y_m: f64::NAN,
        position_z_m: f64::NAN,
    };
    for batch in reader {
        let batch = batch.expect("read batch");
        let arr_x = batch
            .column(px)
            .as_any()
            .downcast_ref::<Float64Array>()
            .expect("position_x is float64");
        let arr_y = batch
            .column(py)
            .as_any()
            .downcast_ref::<Float64Array>()
            .expect("position_y is float64");
        let arr_z = batch
            .column(pz)
            .as_any()
            .downcast_ref::<Float64Array>()
            .expect("position_z is float64");
        if batch.num_rows() > 0 {
            last.position_x_m = arr_x.value(batch.num_rows() - 1);
            last.position_y_m = arr_y.value(batch.num_rows() - 1);
            last.position_z_m = arr_z.value(batch.num_rows() - 1);
        }
    }
    last
}

#[test]
fn leo_orbit_egm2008_dopri853_adaptive_runs_to_completion_within_radius_envelope() {
    let temp = tempdir_for("openbmp_dopri853_run");
    let staged = stage_scenario(temp.path(), "dopri853_run");

    let report = run::run(&staged).expect("scenario run must succeed");

    assert_eq!(report.stop_label, "end-time");
    assert_eq!(
        report.final_step, 5556,
        "outer kernel step count is set by [time] dt_s + stop_s, \
         independent of DOP853's internal sub-stepping"
    );

    let parquet_path = report
        .written
        .iter()
        .find(|p| p.extension().and_then(|e| e.to_str()) == Some("parquet"))
        .expect("parquet output path must be in report");

    let final_state = read_final_eci_position(parquet_path);
    let r_final = (final_state.position_x_m.powi(2)
        + final_state.position_y_m.powi(2)
        + final_state.position_z_m.powi(2))
    .sqrt();

    let radial_error = (r_final - INITIAL_RADIUS_M).abs();
    assert!(
        radial_error < 5_000.0,
        "final radius {r_final:.1} m drifted {radial_error:.1} m from initial \
         {INITIAL_RADIUS_M:.1} m (expected <5 km radial bound under EGM2008 zonal \
         + Dopri853Adaptive at rtol = 1e-12, atol = 1e-15)",
    );
}

#[test]
fn leo_orbit_egm2008_dopri853_adaptive_is_byte_stable_across_two_runs_within_platform() {
    let temp = tempdir_for("openbmp_dopri853_bytestable");

    let staged_a = stage_scenario(temp.path(), "dopri853_run_a");
    let staged_b = stage_scenario(temp.path(), "dopri853_run_b");

    let report_a = run::run(&staged_a).expect("run a");
    let report_b = run::run(&staged_b).expect("run b");

    assert_eq!(report_a.final_step, report_b.final_step);
    assert_eq!(
        report_a.final_time_s.to_bits(),
        report_b.final_time_s.to_bits()
    );

    let parquet_a = report_a
        .written
        .iter()
        .find(|p| p.extension().and_then(|e| e.to_str()) == Some("parquet"))
        .expect("parquet output a");
    let parquet_b = report_b
        .written
        .iter()
        .find(|p| p.extension().and_then(|e| e.to_str()) == Some("parquet"))
        .expect("parquet output b");
    let bytes_a = fs::read(parquet_a).expect("read a");
    let bytes_b = fs::read(parquet_b).expect("read b");

    assert_eq!(
        bytes_a, bytes_b,
        "two reruns of leo-orbit-egm2008-dopri853-adaptive on the same platform must \
         produce byte-identical Parquet — Dopri853Adaptive is StateStable, which is \
         within-platform bit-stable",
    );
}
