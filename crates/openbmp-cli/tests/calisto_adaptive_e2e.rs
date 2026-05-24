//! End-to-end test: RocketPy Calisto sounding-rocket
//! validation scenario integrated with the Dormand-Prince 5(4)
//! adaptive embedded RK pair under a Gustafsson PI step controller
//! on the **rigid-body** runner.
//!
//! Asserts the exit criteria for the rigid-body adaptive
//! dispatch path:
//!
//! 1. The scenario runs to completion via `openbmp run` with
//!    `stop = end-time` after 180 000 outer kernel steps — the
//!    outer kernel cadence is `dt_s = 0.001` seconds, the same as
//!    the fixed-RK4 cross-tool baseline. The adaptive
//!    integrator sub-steps internally as needed to satisfy
//!    `(rtol = 1e-7, atol = 1e-9)` but the kernel's outer step
//!    count is fixed by the time grid.
//! 2. The vehicle reaches an apogee within ±2 % of the RocketPy
//!    cross-tool baseline (3 349 m AGL) — the same envelope as the
//!    fixed-RK4 e2e gate. This demonstrates that the
//!    adaptive integrator on the rigid-body path delivers
//!    comparable accuracy at the documented tolerances; if the
//!    per-component HNW error norm or PI controller had a wiring
//!    bug, the apogee would drift outside the envelope.
//! 3. The drogue + main parachute mission graph still fires —
//!    `at_apogee` and `at_altitude_descending` triggers integrate
//!    correctly with the adaptive sub-stepping (the kernel still
//!    polls them at the outer cadence).
//! 4. Two reruns on the same platform produce byte-identical
//!    Parquet — `Dopri54Adaptive` is `StateStable`, which is
//!    within-platform bit-stable: the PI step-size search is
//!    deterministic and `last_h_s` / `last_err_prev` start from
//!    `None` on each fresh integrator instance.
//!
//! The math-side correctness of `Dopri54Adaptive` (5th-order
//! convergence, tableau row-sums, PI-factor monotonicity) and the
//! per-component `weighted_error_norm` (RMS form, 26-component
//! pairing, bit-stability) are covered by unit tests in
//! `crates/openbmp-sim/src/integrator.rs` and
//! `crates/openbmp-models/src/state.rs`. The runner-side dispatch
//! (rejecting `dopri853`, `rkf78`, etc.) is covered by the unit
//! tests in `crates/openbmp-cli/src/runner/integrator.rs`.

#![allow(clippy::expect_used, clippy::panic, clippy::float_cmp)]

use std::fs;
use std::path::{Path, PathBuf};

use arrow::array::{BooleanArray, Float64Array, Int64Array};
use assert_cmd::assert::OutputAssertExt;
use assert_cmd::cargo::CommandCargoExt;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use tempfile::{Builder, TempDir};

const ROCKETPY_APOGEE_AGL_M: f64 = 3349.0;
const APOGEE_TOLERANCE_FRAC: f64 = 0.02;
const SPACEPORT_ALTITUDE_OFFSET_M: f64 = 1400.0;
const MAIN_DEPLOY_ALTITUDE_MSL_M: f64 = 1867.0;

#[derive(Debug)]
struct CalistoTelemetry {
    position_z_m: Vec<f64>,
    recovery_deployed: Vec<bool>,
    recovery_phase_index: Vec<i64>,
    recovery_drag_area_m2: Vec<f64>,
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

fn read_calisto_telemetry(parquet_path: &Path) -> CalistoTelemetry {
    let file = fs::File::open(parquet_path).expect("open parquet");
    let builder = ParquetRecordBatchReaderBuilder::try_new(file).expect("parquet reader builder");
    let reader = builder.build().expect("parquet reader build");
    let mut out = CalistoTelemetry {
        position_z_m: Vec::new(),
        recovery_deployed: Vec::new(),
        recovery_phase_index: Vec::new(),
        recovery_drag_area_m2: Vec::new(),
    };
    for batch in reader {
        let batch = batch.expect("batch read");
        let schema = batch.schema();
        let z_index = schema
            .index_of("position_z_m")
            .expect("position_z_m column");
        let deployed_index = schema
            .index_of("recovery.calisto_chute.deployed")
            .expect("recovery deployed column");
        let phase_index = schema
            .index_of("recovery.calisto_chute.phase_index")
            .expect("recovery phase column");
        let drag_area_index = schema
            .index_of("recovery.calisto_chute.drag_area_m2")
            .expect("recovery drag area column");
        let z_array = batch
            .column(z_index)
            .as_any()
            .downcast_ref::<Float64Array>()
            .expect("position_z_m as f64");
        let deployed_array = batch
            .column(deployed_index)
            .as_any()
            .downcast_ref::<BooleanArray>()
            .expect("recovery deployed as bool");
        let phase_array = batch
            .column(phase_index)
            .as_any()
            .downcast_ref::<Int64Array>()
            .expect("recovery phase as i64");
        let drag_area_array = batch
            .column(drag_area_index)
            .as_any()
            .downcast_ref::<Float64Array>()
            .expect("recovery drag area as f64");
        for i in 0..z_array.len() {
            out.position_z_m.push(z_array.value(i));
            out.recovery_deployed.push(deployed_array.value(i));
            out.recovery_phase_index.push(phase_array.value(i));
            out.recovery_drag_area_m2.push(drag_area_array.value(i));
        }
    }
    out
}

#[test]
fn calisto_adaptive_apogee_recovery_and_byte_stability() {
    let scenario = workspace_root().join("scenarios/calisto-adaptive/scenario.toml");
    let temp = tempdir("calisto-adaptive-e2e");
    let parquet_a = temp.path().join("calisto-adaptive-a.parquet");
    let parquet_b = temp.path().join("calisto-adaptive-b.parquet");

    for parquet in [&parquet_a, &parquet_b] {
        let mut cmd = openbmp();
        cmd.arg("run")
            .arg(&scenario)
            .arg("--output-parquet")
            .arg(parquet);
        cmd.assert().success();
    }

    let telemetry = read_calisto_telemetry(&parquet_a);
    assert!(
        !telemetry.position_z_m.is_empty(),
        "parquet produced no rows"
    );

    let (apogee_index, apogee_scenario_m) = telemetry
        .position_z_m
        .iter()
        .copied()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.total_cmp(b))
        .expect("apogee row");
    assert!(
        apogee_scenario_m.is_finite(),
        "apogee must be finite, got {apogee_scenario_m}"
    );

    let apogee_agl_m = apogee_scenario_m - SPACEPORT_ALTITUDE_OFFSET_M;
    let envelope_m = ROCKETPY_APOGEE_AGL_M * APOGEE_TOLERANCE_FRAC;
    let lower = ROCKETPY_APOGEE_AGL_M - envelope_m;
    let upper = ROCKETPY_APOGEE_AGL_M + envelope_m;

    assert!(
        apogee_agl_m >= lower && apogee_agl_m <= upper,
        "Calisto-adaptive apogee AGL {apogee_agl_m:.3} m outside {APOGEE_TOLERANCE_FRAC} envelope \
         around RocketPy reference {ROCKETPY_APOGEE_AGL_M} m (allowed [{lower:.3}, {upper:.3}] m). \
         Adaptive integrator on rigid-body runner must reach the same envelope as fixed-RK4.",
    );

    let first_drogue = telemetry
        .recovery_phase_index
        .iter()
        .position(|&phase| phase == 1)
        .expect("drogue phase should be reached under adaptive integrator");
    assert!(
        first_drogue > apogee_index,
        "drogue deployment should be recorded after the apogee row"
    );
    assert!(telemetry.recovery_deployed[first_drogue]);

    let first_main = telemetry
        .recovery_phase_index
        .iter()
        .position(|&phase| phase == 2)
        .expect("main phase should be reached under adaptive integrator");
    assert!(
        first_main > first_drogue,
        "main deployment should occur after drogue deployment"
    );
    assert!(
        telemetry.position_z_m[first_main] <= MAIN_DEPLOY_ALTITUDE_MSL_M,
        "main deployment row altitude {} m should be at or below {MAIN_DEPLOY_ALTITUDE_MSL_M} m",
        telemetry.position_z_m[first_main],
    );
    assert!(telemetry.recovery_deployed[first_main]);

    let bytes_a = fs::read(&parquet_a).expect("read parquet a");
    let bytes_b = fs::read(&parquet_b).expect("read parquet b");
    assert_eq!(
        bytes_a, bytes_b,
        "calisto-adaptive must produce byte-identical Parquet across two reruns on the same \
         platform — Dopri54Adaptive is StateStable, which is within-platform bit-stable: the \
         PI step-size search is deterministic and last_h_s / last_err_prev start from None \
         on each run",
    );
}
