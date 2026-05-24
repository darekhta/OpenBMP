//! End-to-end test: `openbmp run` on the RocketPy
//! Calisto cross-tool validation scenario.
//!
//! Asserts:
//!  - `openbmp run scenarios/sounding-rocket/calisto/rocketpy-calisto.toml`
//!    succeeds and writes a Parquet file.
//!  - The vehicle reaches an apogee within the audited
//!    cross-tool envelope around RocketPy's published 3 349 m AGL.
//!    The audit found that the earlier 5 % fallback was masking a
//!    motor-mass mismatch, not RK4 truncation error. With the
//!    RocketPy `SolidMotor` mass profile, the remaining residual is
//!    pinned to a tighter 2 % envelope.
//!  - The RocketPy-mass motor profile is actually wired through the
//!    runner: ignition mass and burnout mass match the RocketPy
//!    dry/grain-geometry values.
//!  - The recovery path deploys the drogue at apogee and reaches the
//!    main-chute phase on descent.
//!  - Two reruns of the same scenario produce byte-identical Parquet
//!    output (determinism gate end-to-end across the full
//!    rigid-body hot path: scenario parse → motor + drag deck load →
//!    aero + thrust force adapters → `DrogueMainRecovery` state machine
//!    → RK4 integrator → Parquet sink).

#![allow(clippy::expect_used, clippy::panic, clippy::float_cmp)]

use std::fs;
use std::path::{Path, PathBuf};

use arrow::array::{BooleanArray, Float64Array, Int64Array};
use assert_cmd::assert::OutputAssertExt;
use assert_cmd::cargo::CommandCargoExt;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use tempfile::{Builder, TempDir};

/// RocketPy's published Calisto apogee, AGL, in metres.
/// Source: RocketPy Calisto example (Souza et al. 2022).
const ROCKETPY_APOGEE_AGL_M: f64 = 3349.0;

/// Audited cross-tool envelope after fixing the RocketPy
/// motor-mass profile. The original 1 % stretch remains a follow-up;
/// 2 % is tight enough to catch the pre-audit manufacturer-mass
/// regression while leaving room for the documented atmosphere / rail
/// simplifications.
const APOGEE_TOLERANCE_FRAC: f64 = 0.02;

/// The Calisto scenario offsets initial altitude by Spaceport
/// America's geodetic elevation (1 400 m) so the USSA76 atmosphere
/// sees the right altitude profile. Apogee AGL = max(z) − offset.
const SPACEPORT_ALTITUDE_OFFSET_M: f64 = 1400.0;

/// RocketPy Calisto `SolidMotor` mass profile:
/// dry mass = 1.815 kg, propellant mass from grain geometry.
const ROCKETPY_MOTOR_WET_MASS_KG: f64 = 4.770_911_961_392_022;
const ROCKETPY_MOTOR_DRY_MASS_KG: f64 = 1.815;
const CALISTO_DRY_AIRFRAME_MASS_KG: f64 = 14.426;
const MAIN_DEPLOY_ALTITUDE_MSL_M: f64 = 1867.0;

#[derive(Debug)]
struct CalistoTelemetry {
    time_s: Vec<f64>,
    position_z_m: Vec<f64>,
    mass_kg: Vec<f64>,
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
        time_s: Vec::new(),
        position_z_m: Vec::new(),
        mass_kg: Vec::new(),
        recovery_deployed: Vec::new(),
        recovery_phase_index: Vec::new(),
        recovery_drag_area_m2: Vec::new(),
    };
    for batch in reader {
        let batch = batch.expect("batch read");
        let schema = batch.schema();
        let time_index = schema.index_of("time_s").expect("time_s column");
        let z_index = schema
            .index_of("position_z_m")
            .expect("position_z_m column");
        let mass_index = schema.index_of("mass_kg").expect("mass_kg column");
        let deployed_index = schema
            .index_of("recovery.calisto_chute.deployed")
            .expect("recovery deployed column");
        let phase_index = schema
            .index_of("recovery.calisto_chute.phase_index")
            .expect("recovery phase column");
        let drag_area_index = schema
            .index_of("recovery.calisto_chute.drag_area_m2")
            .expect("recovery drag area column");
        let time_array = f64_column(&batch, time_index, "time_s");
        let z_array = f64_column(&batch, z_index, "position_z_m");
        let mass_array = f64_column(&batch, mass_index, "mass_kg");
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
        let drag_area_array = f64_column(
            &batch,
            drag_area_index,
            "recovery.calisto_chute.drag_area_m2",
        );
        for i in 0..z_array.len() {
            out.time_s.push(time_array.value(i));
            out.position_z_m.push(z_array.value(i));
            out.mass_kg.push(mass_array.value(i));
            out.recovery_deployed.push(deployed_array.value(i));
            out.recovery_phase_index.push(phase_array.value(i));
            out.recovery_drag_area_m2.push(drag_area_array.value(i));
        }
    }
    out
}

#[test]
fn calisto_apogee_recovery_and_byte_stability() {
    let scenario = workspace_root().join("scenarios/sounding-rocket/calisto/rocketpy-calisto.toml");
    let temp = tempdir("calisto-e2e");
    let parquet_a = temp.path().join("calisto-a.parquet");
    let parquet_b = temp.path().join("calisto-b.parquet");

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
        "Calisto apogee AGL {apogee_agl_m:.3} m outside {APOGEE_TOLERANCE_FRAC} envelope around \
         RocketPy reference {ROCKETPY_APOGEE_AGL_M} m (allowed [{lower:.3}, {upper:.3}] m)",
    );

    assert_close(
        telemetry.mass_kg[0],
        CALISTO_DRY_AIRFRAME_MASS_KG + ROCKETPY_MOTOR_WET_MASS_KG,
        1.0e-12,
        "initial mass",
    );
    let burnout_index = nearest_time_index(&telemetry.time_s, 3.9);
    assert_close(
        telemetry.mass_kg[burnout_index],
        CALISTO_DRY_AIRFRAME_MASS_KG + ROCKETPY_MOTOR_DRY_MASS_KG,
        1.0e-11,
        "burnout mass",
    );

    let first_drogue = telemetry
        .recovery_phase_index
        .iter()
        .position(|&phase| phase == 1)
        .expect("drogue phase should be reached");
    assert!(
        first_drogue > apogee_index,
        "drogue deployment should be recorded after the apogee row"
    );
    assert!(telemetry.recovery_deployed[first_drogue]);
    assert_close(
        telemetry.recovery_drag_area_m2[first_drogue],
        1.0,
        0.0,
        "drogue drag area",
    );

    let first_main = telemetry
        .recovery_phase_index
        .iter()
        .position(|&phase| phase == 2)
        .expect("main phase should be reached");
    assert!(
        first_main > first_drogue,
        "main deployment should occur after drogue deployment"
    );
    assert!(
        telemetry.position_z_m[first_main] <= MAIN_DEPLOY_ALTITUDE_MSL_M,
        "main deployment row altitude {} m should be at or below {MAIN_DEPLOY_ALTITUDE_MSL_M} m",
        telemetry.position_z_m[first_main]
    );
    assert!(telemetry.recovery_deployed[first_main]);
    assert_close(
        telemetry.recovery_drag_area_m2[first_main],
        10.0,
        0.0,
        "main drag area",
    );

    let bytes_a = fs::read(&parquet_a).expect("read parquet a");
    let bytes_b = fs::read(&parquet_b).expect("read parquet b");
    assert_eq!(
        bytes_a, bytes_b,
        "Calisto scenario changed Parquet bytes across reruns",
    );
}

fn f64_column<'a>(
    batch: &'a arrow::record_batch::RecordBatch,
    index: usize,
    name: &str,
) -> &'a Float64Array {
    batch
        .column(index)
        .as_any()
        .downcast_ref::<Float64Array>()
        .unwrap_or_else(|| panic!("{name} as f64"))
}

fn nearest_time_index(times: &[f64], target_s: f64) -> usize {
    times
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| ((**a - target_s).abs()).total_cmp(&((**b - target_s).abs())))
        .map(|(i, _)| i)
        .expect("time samples")
}

fn assert_close(actual: f64, expected: f64, epsilon: f64, label: &str) {
    assert!(
        (actual - expected).abs() <= epsilon,
        "{label}: actual {actual} differs from expected {expected} by more than {epsilon}",
    );
}
