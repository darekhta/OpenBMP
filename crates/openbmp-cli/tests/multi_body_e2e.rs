//! End-to-end test: multi-body scenario loads and
//! propagates through the kernel.
//!
//! Asserts: a scenario with two bodies
//! (`scenarios/multi-body/two-body-fairing.toml`) loads via
//! `openbmp run`, parses the assembly tree, and produces a Parquet
//! whose initial-row mass equals the sum of body dry masses
//! (0.085 kg).

#![allow(clippy::expect_used, clippy::float_cmp, clippy::panic)]

use std::fs;
use std::path::{Path, PathBuf};

use arrow::array::{BooleanArray, Float64Array};
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

fn read_f64_column(parquet: &Path, column_name: &str) -> Vec<f64> {
    let file = fs::File::open(parquet).expect("open parquet");
    let builder = ParquetRecordBatchReaderBuilder::try_new(file).expect("parquet builder");
    let schema = builder.schema().clone();
    let col = schema
        .index_of(column_name)
        .unwrap_or_else(|_| panic!("{column_name} column present in parquet schema"));
    let reader = builder.build().expect("parquet reader");
    let mut out = Vec::new();
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

fn read_bool_column(parquet: &Path, column_name: &str) -> Vec<bool> {
    let file = fs::File::open(parquet).expect("open parquet");
    let builder = ParquetRecordBatchReaderBuilder::try_new(file).expect("parquet builder");
    let schema = builder.schema().clone();
    let col = schema
        .index_of(column_name)
        .unwrap_or_else(|_| panic!("{column_name} column present in parquet schema"));
    let reader = builder.build().expect("parquet reader");
    let mut out = Vec::new();
    for batch in reader {
        let batch = batch.expect("read batch");
        let arr = batch
            .column(col)
            .as_any()
            .downcast_ref::<BooleanArray>()
            .unwrap_or_else(|| panic!("{column_name} is bool"));
        for i in 0..arr.len() {
            out.push(arr.value(i));
        }
    }
    out
}

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

#[test]
fn two_body_fairing_scenario_runs_to_completion() {
    let scenario = workspace_root().join("scenarios/multi-body/two-body-fairing.toml");
    let temp = tempdir("multi-body-e2e");
    let parquet = temp.path().join("two-body-fairing.parquet");

    run_scenario(&scenario, &parquet);

    // Validate initial-row mass = sum of body dry masses (0.080 + 0.005).
    let file = fs::File::open(&parquet).expect("open parquet");
    let builder = ParquetRecordBatchReaderBuilder::try_new(file).expect("parquet builder");
    let schema = builder.schema().clone();
    let mass_col = schema.index_of("mass_kg").expect("mass_kg column present");

    let reader = builder.build().expect("parquet reader");
    let mut first_mass: Option<f64> = None;
    for batch in reader {
        let batch = batch.expect("read batch");
        if first_mass.is_none() && batch.num_rows() > 0 {
            let mass = batch
                .column(mass_col)
                .as_any()
                .downcast_ref::<Float64Array>()
                .expect("mass_kg is float64");
            first_mass = Some(mass.value(0));
            break;
        }
    }
    let initial_mass = first_mass.expect("at least one row");
    assert!(
        (initial_mass - 0.085).abs() < 1.0e-9,
        "initial mass should equal the assembly dry-mass sum (0.085 kg), got {initial_mass}",
    );
}

#[test]
fn owned_engine_stage_separation_routes_thrust_to_continuing_body_only() {
    let temp = tempdir("owned-engine-stage-separation");
    let scenario = temp.path().join("owned-engine-stage-separation.toml");
    let parquet = temp.path().join("owned-engine-stage-separation.parquet");
    fs::write(
        &scenario,
        r#"
openbmp.scenario = 3

[meta]
name        = "owned-engine-stage-separation"
description = "Synthetic multi-body ownership e2e."
validation  = "experimental"

[time]
start_s = 0.0
stop_s  = 0.6
dt_s    = 0.05
seed    = 0x5152

[vehicle]
kind                                  = "rigid_body"
initial_position_eci_m                = [0.0, 0.0, 0.0]
initial_velocity_eci_m_s              = [0.0, 0.0, 0.0]
initial_quaternion_body_to_eci_xyzw   = [0.0, 0.0, 0.0, 1.0]
initial_angular_velocity_body_rad_s   = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "owned-engine-stage-separation"

[[vehicle.assembly.bodies]]
id                          = "upper"
geometry                    = { kind = "cylinder", length_m = 1.0, diameter_m = 0.1 }
dry_mass_kg                 = 4.0
dry_cg_body_m               = [0.0, 0.0, 0.0]
dry_inertia_body_kg_m2      = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

[[vehicle.assembly.bodies]]
id                          = "lower"
geometry                    = { kind = "cylinder", length_m = 1.0, diameter_m = 0.1 }
dry_mass_kg                 = 1.0
dry_cg_body_m               = [0.0, 0.0, -1.0]
dry_inertia_body_kg_m2      = [[0.25, 0.0, 0.0], [0.0, 0.25, 0.0], [0.0, 0.0, 0.25]]

[[vehicle.assembly.engines]]
id                 = "engine_a"
mounted_to         = "upper"
kind               = { kind = "liquid_engine" }
mount_point_body_m = [0.0, 0.0, 0.0]
limits             = { max_thrust_n = 100.0, isp_s = 250.0, ignition_transient_s = 0.0, shutdown_transient_s = 0.0, max_gimbal_rad = 0.0 }

[environment]
frame_profile = "toy-fixed-earth"
gravity       = "constant"
gravity_m_s2  = 0.0
atmosphere    = "none"
wind          = "none"

[forces]
models = ["gravity", "thrust"]

[telemetry]
output.parquet = "out/owned-engine-stage-separation.parquet"

[validation]
require_finite_state   = true
require_monotonic_time = true

[mission]
initial_phase = "boost"

[[mission.states]]
id    = "boost"
label = "boost"

[[mission.events]]
id      = "ignite"
trigger = { kind = "at_time", time_s = 0.05 }
action  = { kind = "engine_command", id = "engine_a", command = { throttle_unit = 1.0, gimbal_pitch_rad = 0.0, gimbal_yaw_rad = 0.0, ignite = true, shutdown = false } }
once    = true

[[mission.events]]
id      = "stage_separation"
trigger = { kind = "at_time", time_s = 0.2 }
action  = { kind = "jettison_stage", body = "lower" }
once    = true

[multi_body]

[[multi_body.separation]]
event_id          = "stage_separation"
upper_body_id     = "upper"
lower_body_id     = "lower"
conserve_momentum = true
"#,
    )
    .expect("write scenario");

    run_scenario(&scenario, &parquet);

    let separated = read_bool_column(&parquet, "body.lower.separated");
    assert!(
        separated.iter().any(|value| *value),
        "lower body must become separated in telemetry"
    );
    let primary_vz = read_f64_column(&parquet, "velocity_z_m_s");
    let lower_vz = read_f64_column(&parquet, "body.lower.velocity_z_m_s");
    let thrust_z = read_f64_column(&parquet, "force.thrust.z_n");
    let final_primary_vz = *primary_vz.last().expect("primary velocity rows");
    let final_lower_vz = *lower_vz.last().expect("lower velocity rows");
    assert!(
        final_primary_vz > final_lower_vz + 1.0,
        "owned upper engine should accelerate continuing body after separation; primary={final_primary_vz}, lower={final_lower_vz}"
    );
    assert!(
        thrust_z.iter().any(|value| *value > 50.0),
        "primary thrust telemetry should show the owned upper engine firing"
    );
}
