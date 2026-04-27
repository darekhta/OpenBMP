//! Phase-2.11.A end-to-end test: `openbmp run` on the canonical
//! Niskanen 2009 Chapter-6 sounding-rocket scenario.
//!
//! Asserts:
//! - `openbmp run scenarios/sounding-rocket/niskanen-2009-chapter6.toml`
//!   succeeds and writes a Parquet file.
//! - The maximum altitude (apogee) in the trace falls within ±5% of the
//!   Niskanen 151.5 m experimental C6 apogee.
//!
//! Force ordering note: the canonical scenario declares
//! `forces.models = ["gravity", "thrust", "aero"]`. The Phase-2.9
//! sounding-rocket integration test in
//! `crates/openbmp-vehicle/tests/sounding_rocket.rs` wires
//! `gravity, drag, thrust` for historical reasons. The two paths
//! produce different float bits at the LSB level (RK4 sums in
//! declared order), but both pass the same ±5% physical envelope.
//! This test is the *runner-side* baseline for byte-stable replay.

#![allow(clippy::expect_used, clippy::panic, clippy::float_cmp)]

use std::fs;
use std::path::{Path, PathBuf};

use assert_cmd::assert::OutputAssertExt;
use assert_cmd::cargo::CommandCargoExt;
use tempfile::{Builder, NamedTempFile, TempDir};

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

fn with_reversed_mission_phase_blocks(toml: &str) -> String {
    let phase_start = toml
        .find("[[mission.phases]]")
        .expect("mission phase block start");
    let event_start = toml
        .find("[[mission.events]]")
        .expect("mission event block start");
    let phase_region = &toml[phase_start..event_start];
    let mut phase_blocks: Vec<String> = phase_region
        .split("[[mission.phases]]")
        .skip(1)
        .map(|block| format!("[[mission.phases]]{block}"))
        .collect();
    phase_blocks.reverse();
    format!(
        "{}{}{}",
        &toml[..phase_start],
        phase_blocks.concat(),
        &toml[event_start..]
    )
}

/// Niskanen 2009 Chapter-6 Table 6.1 experimental C6 apogee (m).
const NISKANEN_C6_EXPERIMENTAL_APOGEE_M: f64 = 151.5;

/// Project-declared tolerance on the apogee comparison: ±5%
/// against the published Niskanen Chapter-6 C6 experimental value.
const NISKANEN_C6_RELATIVE_TOLERANCE: f64 = 0.05;

#[test]
fn run_on_niskanen_scenario_apogee_within_tolerance() {
    let scenario = workspace_root().join("scenarios/sounding-rocket/niskanen-2009-chapter6.toml");
    let temp = tempdir("niskanen-e2e");
    let parquet = temp.path().join("niskanen-out.parquet");

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
    assert!(parquet.exists(), "parquet file should have been written");

    let apogee_m = read_max_altitude(&parquet);
    let lower = NISKANEN_C6_EXPERIMENTAL_APOGEE_M * (1.0 - NISKANEN_C6_RELATIVE_TOLERANCE);
    let upper = NISKANEN_C6_EXPERIMENTAL_APOGEE_M * (1.0 + NISKANEN_C6_RELATIVE_TOLERANCE);
    assert!(
        (lower..=upper).contains(&apogee_m),
        "apogee {apogee_m:.2} m outside ±5% of Niskanen C6 \
         experimental {NISKANEN_C6_EXPERIMENTAL_APOGEE_M} m \
         (envelope [{lower:.2}, {upper:.2}])",
    );
}

#[test]
fn run_byte_stable_across_two_invocations() {
    // Determinism gate: the same scenario invoked twice produces
    // byte-identical Parquet on the reference platform. This is the
    // CLI-side mirror of the kernel byte-stability gate.
    let scenario = workspace_root().join("scenarios/sounding-rocket/niskanen-2009-chapter6.toml");
    let temp_a = tempdir("niskanen-stable-a");
    let temp_b = tempdir("niskanen-stable-b");
    let parquet_a = temp_a.path().join("a.parquet");
    let parquet_b = temp_b.path().join("b.parquet");

    for parquet in [&parquet_a, &parquet_b] {
        let mut cmd = openbmp();
        cmd.arg("run")
            .arg(&scenario)
            .arg("--output-parquet")
            .arg(parquet);
        cmd.assert().success();
    }

    let bytes_a = fs::read(&parquet_a).expect("read parquet a");
    let bytes_b = fs::read(&parquet_b).expect("read parquet b");
    assert_eq!(
        bytes_a.len(),
        bytes_b.len(),
        "Niskanen Parquet outputs differ in length: {} vs {}",
        bytes_a.len(),
        bytes_b.len(),
    );
    assert!(
        bytes_a == bytes_b,
        "Niskanen Parquet outputs differ at the byte level — runner is not deterministic",
    );
}

#[test]
fn rigid_run_byte_stable_across_two_invocations() {
    // Phase-3.1 determinism gate: the same rigid-body scenario
    // invoked twice produces byte-identical Parquet on the reference
    // platform. This is the CLI-side mirror of the rigid kernel
    // byte-stability gate.
    let scenario =
        workspace_root().join("scenarios/sounding-rocket/niskanen-2009-chapter6-rigid.toml");
    let temp_a = tempdir("niskanen-rigid-stable-a");
    let temp_b = tempdir("niskanen-rigid-stable-b");
    let parquet_a = temp_a.path().join("a.parquet");
    let parquet_b = temp_b.path().join("b.parquet");

    for parquet in [&parquet_a, &parquet_b] {
        let mut cmd = openbmp();
        cmd.arg("run")
            .arg(&scenario)
            .arg("--output-parquet")
            .arg(parquet);
        cmd.assert().success();
    }

    let bytes_a = fs::read(&parquet_a).expect("read rigid parquet a");
    let bytes_b = fs::read(&parquet_b).expect("read rigid parquet b");
    assert_eq!(
        bytes_a.len(),
        bytes_b.len(),
        "Rigid Niskanen Parquet outputs differ in length: {} vs {}",
        bytes_a.len(),
        bytes_b.len(),
    );
    assert!(
        bytes_a == bytes_b,
        "Rigid Niskanen Parquet outputs differ at the byte level - runner is not deterministic",
    );
}

/// Phase-3.1: the rigid-body Niskanen scenario produces an apogee
/// physically equivalent to the point-mass case. With identity
/// initial orientation, zero angular velocity, `ZeroMoment`, and
/// axisymmetric drag, the rigid-body kernel sees no body-frame
/// torque and the trajectory matches the point-mass Niskanen
/// trajectory within IEEE 754 reduction-order noise. We assert
/// the same ±5% physical envelope as the point-mass test.
#[test]
fn run_on_niskanen_rigid_apogee_within_tolerance() {
    let scenario =
        workspace_root().join("scenarios/sounding-rocket/niskanen-2009-chapter6-rigid.toml");
    let temp = tempdir("niskanen-rigid-e2e");
    let parquet = temp.path().join("niskanen-rigid-out.parquet");

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
    assert!(parquet.exists(), "parquet file should have been written");

    let apogee_m = read_max_altitude(&parquet);
    let lower = NISKANEN_C6_EXPERIMENTAL_APOGEE_M * (1.0 - NISKANEN_C6_RELATIVE_TOLERANCE);
    let upper = NISKANEN_C6_EXPERIMENTAL_APOGEE_M * (1.0 + NISKANEN_C6_RELATIVE_TOLERANCE);
    assert!(
        (lower..=upper).contains(&apogee_m),
        "rigid-body apogee {apogee_m:.2} m outside ±5% of Niskanen C6 \
         experimental {NISKANEN_C6_EXPERIMENTAL_APOGEE_M} m \
         (envelope [{lower:.2}, {upper:.2}])",
    );
}

/// Cross-path consistency check: rigid-body Niskanen and point-mass
/// Niskanen apogees agree within 1% relative. Floating-point
/// reduction order differs (the kernel's RK4 chains through
/// quaternion-renormalisation in the rigid path), so bit-equality
/// is not claimed — the assertion is physical equivalence.
#[test]
fn niskanen_rigid_and_point_mass_apogees_agree_within_one_percent() {
    let pm_scenario =
        workspace_root().join("scenarios/sounding-rocket/niskanen-2009-chapter6.toml");
    let rb_scenario =
        workspace_root().join("scenarios/sounding-rocket/niskanen-2009-chapter6-rigid.toml");
    let temp = tempdir("niskanen-cross-path");
    let pm_parquet = temp.path().join("pm.parquet");
    let rb_parquet = temp.path().join("rb.parquet");

    for (scenario, parquet) in [(&pm_scenario, &pm_parquet), (&rb_scenario, &rb_parquet)] {
        let mut cmd = openbmp();
        cmd.arg("run")
            .arg(scenario)
            .arg("--output-parquet")
            .arg(parquet);
        cmd.assert().success();
    }

    let pm_apogee = read_max_altitude(&pm_parquet);
    let rb_apogee = read_max_altitude(&rb_parquet);
    let denom = pm_apogee.abs().max(1.0);
    let rel_err = (pm_apogee - rb_apogee).abs() / denom;
    assert!(
        rel_err < 0.01,
        "rigid-body apogee {rb_apogee:.6} m and point-mass apogee {pm_apogee:.6} m \
         differ by {rel_err:.6} relative; expected < 1%",
    );
}

fn read_max_altitude(parquet_path: &Path) -> f64 {
    use arrow::array::Float64Array;
    use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;

    let file = fs::File::open(parquet_path).expect("open parquet");
    let builder = ParquetRecordBatchReaderBuilder::try_new(file).expect("parquet builder");
    let schema = builder.schema().clone();
    let position_z_col = schema
        .index_of("position_z_m")
        .expect("position_z_m column present");

    let reader = builder.build().expect("parquet reader");
    let mut max_z = f64::NEG_INFINITY;
    for batch in reader {
        let batch = batch.expect("read batch");
        let position_z = batch
            .column(position_z_col)
            .as_any()
            .downcast_ref::<Float64Array>()
            .expect("position_z is float64");
        for i in 0..batch.num_rows() {
            let value = position_z.value(i);
            if value > max_z {
                max_z = value;
            }
        }
    }
    max_z
}

#[test]
fn niskanen_parquet_carries_atmosphere_force_breakdown_and_sha256_metadata() {
    use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;

    let scenario = workspace_root().join("scenarios/sounding-rocket/niskanen-2009-chapter6.toml");
    let temp = tempdir("niskanen-telemetry");
    let parquet = temp.path().join("niskanen.parquet");

    let mut cmd = openbmp();
    cmd.arg("run")
        .arg(&scenario)
        .arg("--output-parquet")
        .arg(&parquet);
    cmd.assert().success();

    let file = fs::File::open(&parquet).expect("open parquet");
    let builder = ParquetRecordBatchReaderBuilder::try_new(file).expect("parquet builder");
    let schema = builder.schema().clone();

    // Atmosphere sample channels (USSA76 declared via `[atmosphere]`).
    for name in [
        "atmosphere.density_kg_m3",
        "atmosphere.pressure_pa",
        "atmosphere.temperature_k",
        "atmosphere.speed_of_sound_m_s",
    ] {
        assert!(
            schema.index_of(name).is_ok(),
            "atmosphere channel {name} missing; columns = {:?}",
            schema.fields().iter().map(|f| f.name()).collect::<Vec<_>>(),
        );
    }

    // Per-model force breakdown channels for `gravity`, `thrust`,
    // `aero` (the canonical scenario's declared force order).
    for name in [
        "force.gravity.x_n",
        "force.gravity.y_n",
        "force.gravity.z_n",
        "force.thrust.x_n",
        "force.thrust.y_n",
        "force.thrust.z_n",
        "force.aero.x_n",
        "force.aero.y_n",
        "force.aero.z_n",
    ] {
        assert!(
            schema.index_of(name).is_ok(),
            "force-breakdown channel {name} missing",
        );
    }

    // SHA-256 metadata for each scenario-referenced external file.
    let metadata = schema.metadata();
    let aero_pin = metadata
        .get("openbmp.scenario_files.aero.deck")
        .expect("aero.deck digest metadata present");
    assert_eq!(
        aero_pin, "cd862c2af98a1f28dc86c6e754d311c7a724081ca91b80704ad89b2ec4cb5c27",
        "aero.deck digest in Parquet header must match the scenario pin",
    );
    let motor_pin = metadata
        .get("openbmp.scenario_files.propulsion.motor.file")
        .expect("propulsion.motor.file digest metadata present");
    assert_eq!(
        motor_pin, "da8272d3a7a135046c614e51b279971d37cac376f7aaaffdedc3ccc14d50ad4e",
        "propulsion.motor.file digest in Parquet header must match the scenario pin",
    );
}

#[test]
fn niskanen_parquet_force_breakdown_components_are_finite_and_nonzero() {
    // Per-row telemetry contract: every published per-model force
    // component is finite, and the Niskanen thrust/aero components are
    // not identically zero. The openbmp-vehicle unit tests cover the
    // BasicVehicle declared-order sum; this e2e test verifies the CLI
    // actually publishes the component channels.
    use arrow::array::Float64Array;
    use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;

    let scenario = workspace_root().join("scenarios/sounding-rocket/niskanen-2009-chapter6.toml");
    let temp = tempdir("niskanen-breakdown-sum");
    let parquet = temp.path().join("niskanen.parquet");
    let mut cmd = openbmp();
    cmd.arg("run")
        .arg(&scenario)
        .arg("--output-parquet")
        .arg(&parquet);
    cmd.assert().success();

    let file = fs::File::open(&parquet).expect("open parquet");
    let builder = ParquetRecordBatchReaderBuilder::try_new(file).expect("parquet builder");
    let schema = builder.schema().clone();

    let cols: Vec<usize> = [
        "force.gravity.x_n",
        "force.gravity.y_n",
        "force.gravity.z_n",
        "force.thrust.x_n",
        "force.thrust.y_n",
        "force.thrust.z_n",
        "force.aero.x_n",
        "force.aero.y_n",
        "force.aero.z_n",
    ]
    .iter()
    .map(|n| schema.index_of(n).expect("force channel"))
    .collect();

    let reader = builder.build().expect("parquet reader");
    let mut saw_nonzero_thrust = false;
    let mut saw_nonzero_aero = false;
    for batch in reader {
        let batch = batch.expect("read batch");
        let arrays: Vec<&Float64Array> = cols
            .iter()
            .map(|&i| {
                batch
                    .column(i)
                    .as_any()
                    .downcast_ref::<Float64Array>()
                    .expect("float64 force column")
            })
            .collect();
        for row in 0..batch.num_rows() {
            for arr in &arrays {
                let value = arr.value(row);
                assert!(
                    value.is_finite(),
                    "force-breakdown component must be finite, got {value} at row {row}",
                );
            }
            let sum_x = arrays[0].value(row) + arrays[3].value(row) + arrays[6].value(row);
            let sum_y = arrays[1].value(row) + arrays[4].value(row) + arrays[7].value(row);
            let sum_z = arrays[2].value(row) + arrays[5].value(row) + arrays[8].value(row);
            assert!(
                sum_x.is_finite() && sum_y.is_finite() && sum_z.is_finite(),
                "declared-order force-component sum must be finite at row {row}",
            );
            let thrust_z = arrays[5].value(row);
            if thrust_z.abs() > 0.0 {
                saw_nonzero_thrust = true;
            }
            let aero_z = arrays[8].value(row);
            if aero_z.abs() > 0.0 {
                saw_nonzero_aero = true;
            }
        }
    }
    assert!(
        saw_nonzero_thrust,
        "thrust force must be non-zero during burn"
    );
    assert!(saw_nonzero_aero, "aero force must be non-zero at speed");
}

/// Phase-3.2: the mission-block variant of the Niskanen scenario
/// declares an `at_apogee` event with `emit_telemetry_marker {
/// tag = "at_apogee_marker" }`. The runner must:
/// 1. Allocate a `bool` channel `mission.marker.at_apogee_marker`.
/// 2. Write `true` exactly once on the apogee step.
/// 3. Write `false` on every other step.
/// 4. Produce an apogee within ±5% of 151.5 m (same physics as the
///    canonical point-mass scenario).
#[test]
fn niskanen_with_mission_emits_apogee_marker() {
    use arrow::array::{BooleanArray, Float64Array};
    use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;

    let scenario =
        workspace_root().join("scenarios/sounding-rocket/niskanen-2009-chapter6-with-mission.toml");
    let temp = tempdir("niskanen-mission");
    let parquet = temp.path().join("niskanen-mission.parquet");

    let mut cmd = openbmp();
    cmd.arg("run")
        .arg(&scenario)
        .arg("--output-parquet")
        .arg(&parquet);
    cmd.assert().success();
    assert!(parquet.exists(), "parquet file must be written");

    // Apogee envelope: same gate as the point-mass scenario.
    let apogee_m = read_max_altitude(&parquet);
    let lower = NISKANEN_C6_EXPERIMENTAL_APOGEE_M * (1.0 - NISKANEN_C6_RELATIVE_TOLERANCE);
    let upper = NISKANEN_C6_EXPERIMENTAL_APOGEE_M * (1.0 + NISKANEN_C6_RELATIVE_TOLERANCE);
    assert!(
        (lower..=upper).contains(&apogee_m),
        "apogee {apogee_m:.2} m outside ±5% envelope [{lower:.2}, {upper:.2}]",
    );

    // Marker channel: walk the parquet, count `true` rows and
    // record the row index where the marker fires.
    let file = fs::File::open(&parquet).expect("open parquet");
    let builder = ParquetRecordBatchReaderBuilder::try_new(file).expect("parquet builder");
    let schema = builder.schema().clone();
    let marker_col = schema
        .index_of("mission.marker.at_apogee_marker")
        .expect("mission.marker.at_apogee_marker column present");
    let position_z_col = schema
        .index_of("position_z_m")
        .expect("position_z_m column present");

    let reader = builder.build().expect("parquet reader");
    let mut total_rows = 0_usize;
    let mut total_fires = 0_usize;
    let mut fired_at_altitude_m = f64::NAN;
    let mut max_alt_at_any_row = f64::NEG_INFINITY;
    for batch in reader {
        let batch = batch.expect("read batch");
        let marker = batch
            .column(marker_col)
            .as_any()
            .downcast_ref::<BooleanArray>()
            .expect("marker column is bool");
        let position_z = batch
            .column(position_z_col)
            .as_any()
            .downcast_ref::<Float64Array>()
            .expect("position_z is float64");
        for i in 0..batch.num_rows() {
            total_rows += 1;
            let alt = position_z.value(i);
            if alt > max_alt_at_any_row {
                max_alt_at_any_row = alt;
            }
            if marker.value(i) {
                total_fires += 1;
                fired_at_altitude_m = alt;
            }
        }
    }
    assert!(
        total_rows > 100,
        "scenario must produce a meaningful trace, got {total_rows} rows",
    );
    assert_eq!(
        total_fires, 1,
        "at_apogee_marker must fire exactly once (got {total_fires})",
    );
    // The fire altitude is the post-flip value, not the apogee
    // altitude itself; under fixed-step integration it sits within
    // a few m below the recorded apogee.
    assert!(
        fired_at_altitude_m.is_finite(),
        "marker fire altitude must be finite",
    );
    let altitude_gap_m = max_alt_at_any_row - fired_at_altitude_m;
    assert!(
        (0.0..=5.0).contains(&altitude_gap_m),
        "apogee marker should fire within 5 m of recorded apogee, gap = {altitude_gap_m:.3} m",
    );
}

#[test]
fn mission_phase_declaration_order_does_not_change_parquet_bytes() {
    let scenario_dir = workspace_root().join("scenarios/sounding-rocket");
    let scenario = scenario_dir.join("niskanen-2009-chapter6-with-mission.toml");
    let original = fs::read_to_string(&scenario).expect("read mission scenario");
    let reordered = with_reversed_mission_phase_blocks(&original);
    let variant = NamedTempFile::new_in(&scenario_dir).expect("temp scenario in scenario dir");
    fs::write(variant.path(), reordered).expect("write reordered scenario");

    let temp = tempdir("niskanen-mission-order");
    let parquet_a = temp.path().join("original.parquet");
    let parquet_b = temp.path().join("reordered.parquet");

    let runs: [(&Path, &Path); 2] = [
        (scenario.as_path(), parquet_a.as_path()),
        (variant.path(), parquet_b.as_path()),
    ];
    for (scenario_path, parquet) in runs {
        let mut cmd = openbmp();
        cmd.arg("run")
            .arg(scenario_path)
            .arg("--output-parquet")
            .arg(parquet);
        cmd.assert().success();
    }

    let bytes_a = fs::read(&parquet_a).expect("read original parquet");
    let bytes_b = fs::read(&parquet_b).expect("read reordered parquet");
    assert_eq!(
        bytes_a, bytes_b,
        "reordering mission phase declarations changed Parquet bytes",
    );
}
