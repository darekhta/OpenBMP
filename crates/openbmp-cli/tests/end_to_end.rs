//! End-to-end golden test for the constant-acceleration drop.
//!
//! Runs the canonical scenario through `openbmp_cli::commands::run::run`
//! twice and asserts:
//!
//! 1. **Tolerance compliance** of the final time, position, and velocity
//!    against the committed `expected/constant-acceleration-drop.toml`.
//! 2. **Same-machine byte stability** of the Parquet archive across two
//!    independent runs (byte-stable claim, also promoted to a committed
//!    cross-machine reference golden).
//!
//! The CSV and Parquet outputs are written under a per-test temp dir
//! and never committed.

#![allow(
    clippy::expect_used,
    clippy::float_cmp,
    clippy::panic,
    clippy::similar_names
)]

use std::fs;
use std::path::{Path, PathBuf};

use openbmp_cli::{CliError, commands::run};
use openbmp_rundata::{RunIndex, verdict::VerdictReport};
use openbmp_testkit::tolerance::ToleranceTable;
use sha2::{Digest, Sha256};
use tempfile::{Builder, TempDir};

const STRUCTURAL_ACOUSTICS_SCENARIO_TEMPLATE: &str =
    include_str!("fixtures/structural-acoustics-rundata.toml");

fn manifest_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn workspace_root() -> PathBuf {
    // crates/openbmp-cli -> workspace
    manifest_dir()
        .parent()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .expect("workspace root must exist relative to CARGO_MANIFEST_DIR")
}

fn scenario_path() -> PathBuf {
    workspace_root().join("scenarios/analytic-toy/constant-acceleration-drop.toml")
}

fn tolerance_table_path() -> PathBuf {
    manifest_dir().join("tests/expected/constant-acceleration-drop.toml")
}

fn toml_literal_path(path: &Path) -> String {
    let path = path.to_string_lossy();
    assert!(
        !path.contains('\''),
        "test temp paths must be representable as TOML literal strings: {path}",
    );
    format!("'{path}'")
}

/// Copy the canonical scenario into a temp dir and rewrite the
/// telemetry output paths to point under that temp dir, so test runs
/// don't litter `out/` next to the committed scenario.
fn stage_scenario(temp_dir: &Path, label: &str) -> PathBuf {
    let original = fs::read_to_string(scenario_path()).expect("read canonical scenario");
    let parquet = temp_dir.join(format!("{label}.parquet"));
    let csv = temp_dir.join(format!("{label}.csv"));
    let rewritten = original
        .replace(
            "output.csv     = \"out/constant-acceleration-drop.csv\"",
            &format!("output.csv     = {}", toml_literal_path(&csv)),
        )
        .replace(
            "output.parquet = \"out/constant-acceleration-drop.parquet\"",
            &format!("output.parquet = {}", toml_literal_path(&parquet)),
        );
    let staged = temp_dir.join(format!("{label}-scenario.toml"));
    fs::write(&staged, rewritten).expect("write staged scenario");
    staged
}

fn stage_acoustics_scenario(temp_dir: &Path, label: &str) -> PathBuf {
    let staged = stage_scenario(temp_dir, label);
    let original = fs::read_to_string(&staged).expect("read staged scenario");
    let workspace = workspace_root();
    let ascent_deck = workspace.join("data/acoustics/ascent-acoustic-env-v1.toml");
    let fairing_deck = workspace.join("data/acoustics/fairing-internal-v1.toml");
    let response_deck = workspace.join("data/acoustics/vibroacoustic-transfer-v1.toml");
    let ascent_deck_sha256 = sha256_hex(&fs::read(&ascent_deck).expect("read ascent deck"));
    let fairing_deck_sha256 = sha256_hex(&fs::read(&fairing_deck).expect("read fairing deck"));
    let response_deck_sha256 = sha256_hex(&fs::read(&response_deck).expect("read response deck"));
    let acoustics = format!(
        r#"[acoustics.ascent]
station_id = "station-a"
deck_file = {}
deck_file_sha256 = "{ascent_deck_sha256}"
class_id = "attached_turbulent"

[[acoustics.ascent.samples]]
sample_id = "max-q"
dynamic_pressure_pa = 50000.0
mach = 0.8

[[acoustics.ascent.samples]]
sample_id = "late-max-q"
dynamic_pressure_pa = 200000.0
mach = 0.8

[acoustics.fairing_internal]
station_id = "payload-a"
deck_file = {}
deck_file_sha256 = "{fairing_deck_sha256}"
class_id = "synthetic_light_fairing"
fill_factor = 1.0

[acoustics.response]
station_id = "payload-a"
source = "fairing_internal"
deck_file = {}
deck_file_sha256 = "{response_deck_sha256}"
class_id = "synthetic_panel_light"
surface_mass_kg_m2 = 12.0
area_m2 = 3.0
mode_shape_factor = 0.5
"#,
        toml_literal_path(&ascent_deck),
        toml_literal_path(&fairing_deck),
        toml_literal_path(&response_deck),
    );
    let rewritten = original
        .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
        .replace("[telemetry]", &format!("{acoustics}\n[telemetry]"));
    fs::write(&staged, rewritten).expect("write staged acoustics scenario");
    staged
}

fn stage_structural_acoustics_scenario(temp_dir: &Path, label: &str) -> PathBuf {
    let workspace = workspace_root();
    let ascent_deck = workspace.join("data/acoustics/ascent-acoustic-env-v1.toml");
    let fairing_deck = workspace.join("data/acoustics/fairing-internal-v1.toml");
    let response_deck = workspace.join("data/acoustics/vibroacoustic-transfer-v1.toml");
    let ascent_deck_sha256 = sha256_hex(&fs::read(&ascent_deck).expect("read ascent deck"));
    let fairing_deck_sha256 = sha256_hex(&fs::read(&fairing_deck).expect("read fairing deck"));
    let response_deck_sha256 = sha256_hex(&fs::read(&response_deck).expect("read response deck"));
    let csv = temp_dir.join(format!("{label}.csv"));
    let scenario = STRUCTURAL_ACOUSTICS_SCENARIO_TEMPLATE
        .replace("@ASCENT_DECK@", &toml_literal_path(&ascent_deck))
        .replace("@ASCENT_DECK_SHA256@", &ascent_deck_sha256)
        .replace("@FAIRING_DECK@", &toml_literal_path(&fairing_deck))
        .replace("@FAIRING_DECK_SHA256@", &fairing_deck_sha256)
        .replace("@RESPONSE_DECK@", &toml_literal_path(&response_deck))
        .replace("@RESPONSE_DECK_SHA256@", &response_deck_sha256)
        .replace("@OUTPUT_CSV@", &toml_literal_path(&csv));
    let staged = temp_dir.join(format!("{label}-scenario.toml"));
    fs::write(&staged, scenario).expect("write staged structural acoustics scenario");
    staged
}

#[test]
fn constant_acceleration_drop_meets_tolerance_table() {
    let temp = tempdir_for("openbmp_e2e_tolerance");
    let staged = stage_scenario(temp.path(), "tolerance");

    let report = run::run(&staged).expect("scenario run must succeed");

    assert_eq!(report.stop_label, "end-time");
    assert_eq!(report.final_step, 1000);

    // The final state values are recovered from the in-memory table by
    // re-loading the freshly-written Parquet — easier than threading
    // them out of `RunReport`, and exercises the writer.
    let parquet_path = report
        .written
        .iter()
        .find(|p| p.extension().and_then(|e| e.to_str()) == Some("parquet"))
        .expect("parquet output path must be in report");

    let final_state = read_final_state(parquet_path);

    let table =
        ToleranceTable::from_path(tolerance_table_path()).expect("tolerance table must parse");
    table
        .check_metric("final_time_s", report.final_time_s)
        .expect("final_time_s within tolerance");
    table
        .check_metric("final_position_z_m", final_state.position_z_m)
        .expect("final_position_z_m within tolerance");
    table
        .check_metric("final_velocity_z_m_s", final_state.velocity_z_m_s)
        .expect("final_velocity_z_m_s within tolerance");
}

#[test]
fn constant_acceleration_drop_is_byte_stable_across_two_runs() {
    let temp = tempdir_for("openbmp_e2e_bytestable");

    let staged_a = stage_scenario(temp.path(), "run_a");
    let staged_b = stage_scenario(temp.path(), "run_b");

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
        "Parquet bytes must be identical across runs"
    );
}

#[test]
fn output_overrides_replace_declared_paths_and_add_missing_kinds() {
    let temp = tempdir_for("openbmp_e2e_output_overrides");
    let override_dir = temp.path().join("override");

    let staged = stage_scenario(temp.path(), "declared");
    let declared_csv = temp.path().join("declared.csv");
    let declared_parquet = temp.path().join("declared.parquet");
    let override_csv = override_dir.join("drop.csv");
    let override_json = override_dir.join("drop.json");
    let override_parquet = override_dir.join("drop.parquet");

    let overrides = run::OutputOverrides::new(
        Some(override_csv.clone()),
        Some(override_json.clone()),
        Some(override_parquet.clone()),
    );
    let report = run::run_with_overrides(&staged, &overrides).expect("run with overrides");

    assert!(override_csv.exists(), "CSV override should be written");
    assert!(override_json.exists(), "JSON override should be written");
    assert!(
        override_parquet.exists(),
        "Parquet override should be written"
    );
    assert!(
        !declared_csv.exists(),
        "declared CSV path should not be written when overridden"
    );
    assert!(
        !declared_parquet.exists(),
        "declared Parquet path should not be written when overridden"
    );
    assert_eq!(
        report.written,
        vec![override_csv, override_json, override_parquet]
    );
}

#[test]
fn run_data_indexes_are_emitted_for_real_run() {
    let temp = tempdir_for("openbmp_e2e_rundata");
    let staged = stage_scenario(temp.path(), "rundata");
    let index_jsonl = temp.path().join("run-index.jsonl");
    let index_parquet = temp.path().join("run-index.parquet");
    let options = run::RunDataOptions::new(
        Some(index_jsonl.clone()),
        Some(index_parquet.clone()),
        "openbmp-e2e-code".to_owned(),
        None,
        None,
        None,
    );

    let report = run::run_with_options(&staged, &run::OutputOverrides::default(), &options)
        .expect("run with run-data outputs");

    let record_report = report
        .run_record
        .as_ref()
        .expect("run record report should be present");
    assert!(index_jsonl.exists(), "JSONL run index should be written");
    assert!(
        index_parquet.exists(),
        "Parquet run index mirror should be written"
    );
    assert!(report.written.contains(&index_jsonl));
    assert!(report.written.contains(&index_parquet));

    let jsonl = fs::read(&index_jsonl).expect("read JSONL run index");
    assert_eq!(sha256_hex(&jsonl), record_report.record_sha256_hex);
    let index = RunIndex::from_jsonl_bytes(&jsonl).expect("parse JSONL run index");
    assert_eq!(index.records().len(), 1);

    let record = &index.records()[0];
    assert_eq!(record.run_id, record_report.run_id);
    assert_eq!(record.code_id, "openbmp-e2e-code");
    assert!(!record.channel_digests.is_empty());
    assert!(
        record
            .artifacts
            .iter()
            .any(|artifact| artifact.role == "scenario")
    );
    assert!(
        record
            .artifacts
            .iter()
            .any(|artifact| artifact.role == "telemetry_csv")
    );
    assert!(
        record
            .artifacts
            .iter()
            .any(|artifact| artifact.role == "telemetry_parquet")
    );

    let parquet = fs::read(&index_parquet).expect("read Parquet run index");
    assert_eq!(
        parquet,
        index
            .to_parquet_bytes()
            .expect("deterministic Parquet mirror")
    );
}

#[test]
fn run_data_indexes_acoustics_p95_50_artifacts() {
    let temp = tempdir_for("openbmp_e2e_rundata_acoustics");
    let staged = stage_acoustics_scenario(temp.path(), "acoustics-rundata");
    let index_jsonl = temp.path().join("run-index.jsonl");
    let options = run::RunDataOptions::new(
        Some(index_jsonl.clone()),
        None,
        "openbmp-e2e-code".to_owned(),
        None,
        None,
        None,
    );

    let report = run::run_with_options(&staged, &run::OutputOverrides::default(), &options)
        .expect("run acoustics scenario with run-data outputs");

    assert!(index_jsonl.exists(), "JSONL run index should be written");
    assert!(report.written.contains(&index_jsonl));
    let jsonl = fs::read(&index_jsonl).expect("read JSONL run index");
    let index = RunIndex::from_jsonl_bytes(&jsonl).expect("parse JSONL run index");
    let record = &index.records()[0];
    let expected_roles = [
        "acoustics_ascent_p95_50_environment_csv",
        "acoustics_ascent_p95_50_wilks_sizing_csv",
        "acoustics_fairing_internal_p95_50_environment_csv",
        "acoustics_fairing_internal_p95_50_sample_trace_csv",
        "acoustics_fairing_internal_p95_50_wilks_sizing_csv",
        "acoustics_response_loads_handoff_csv",
        "acoustics_response_p95_50_acceleration_psd_csv",
        "acoustics_response_p95_50_modal_force_psd_csv",
        "acoustics_response_p95_50_sample_trace_csv",
        "acoustics_response_p95_50_wilks_sizing_csv",
    ];
    assert_eq!(
        record
            .artifacts
            .iter()
            .filter(|artifact| artifact.role.starts_with("acoustics_"))
            .count(),
        expected_roles.len()
    );

    for role in expected_roles {
        let artifact = record
            .artifacts
            .iter()
            .find(|artifact| artifact.role == role)
            .unwrap_or_else(|| panic!("missing artifact role {role}"));
        let path = PathBuf::from(&artifact.path);
        assert!(
            path.starts_with(temp.path().join("artifacts")),
            "artifact should be stored under the run-data artifact root"
        );
        assert!(path.exists(), "artifact file should exist for role {role}");
        assert!(report.written.contains(&path));
        let bytes = fs::read(&path).expect("read acoustics run-data artifact");
        assert_eq!(sha256_hex(&bytes), artifact.sha256_hex);
        let text = std::str::from_utf8(&bytes).expect("acoustics artifact should be UTF-8 CSV");
        assert!(text.lines().count() >= 2);
        if role == "acoustics_fairing_internal_p95_50_sample_trace_csv" {
            assert!(text.contains("fill_factor"));
            assert!(text.contains("synthetic_light_fairing"));
        }
        if role == "acoustics_response_p95_50_sample_trace_csv" {
            assert!(text.contains("surface_mass_kg_m2"));
            assert!(text.contains("synthetic_panel_light"));
        }
    }
}

#[test]
fn run_data_indexes_structural_acoustics_artifact() {
    let temp = tempdir_for("openbmp_e2e_rundata_structural_acoustics");
    let staged = stage_structural_acoustics_scenario(temp.path(), "structural-acoustics-rundata");
    let index_jsonl = temp.path().join("run-index.jsonl");
    let options = run::RunDataOptions::new(
        Some(index_jsonl.clone()),
        None,
        "openbmp-e2e-code".to_owned(),
        None,
        None,
        None,
    );

    let report = run::run_with_options(&staged, &run::OutputOverrides::default(), &options)
        .expect("run structural acoustics scenario with run-data outputs");

    assert!(index_jsonl.exists(), "JSONL run index should be written");
    assert!(report.written.contains(&index_jsonl));
    let jsonl = fs::read(&index_jsonl).expect("read JSONL run index");
    let index = RunIndex::from_jsonl_bytes(&jsonl).expect("parse JSONL run index");
    let record = &index.records()[0];
    for (role, expected_header) in [
        (
            "structural_acoustics_first_bending_response_csv",
            "modal_acceleration_psd_m2_s4_per_hz",
        ),
        (
            "structural_acoustics_first_bending_line_load_recovery_csv",
            "band_equivalent_reaction_moment_mean_square_n2_m2",
        ),
    ] {
        let artifact = record
            .artifacts
            .iter()
            .find(|artifact| artifact.role == role)
            .unwrap_or_else(|| panic!("{role} artifact role should be indexed"));
        let path = PathBuf::from(&artifact.path);
        assert!(
            path.starts_with(temp.path().join("artifacts")),
            "artifact should be stored under the run-data artifact root"
        );
        assert!(path.exists(), "structural acoustics artifact should exist");
        assert!(report.written.contains(&path));
        let bytes = fs::read(&path).expect("read structural acoustics run-data artifact");
        assert_eq!(sha256_hex(&bytes), artifact.sha256_hex);
        let csv =
            std::str::from_utf8(&bytes).expect("structural acoustics artifact should be UTF-8 CSV");
        assert!(csv.contains(expected_header));
        assert!(csv.lines().count() >= 2);
    }
}

#[test]
fn verdict_report_is_emitted_for_real_run() {
    let temp = tempdir_for("openbmp_e2e_verdict");
    let staged = stage_scenario(temp.path(), "verdict");
    let rules = temp.path().join("verdict-rules.toml");
    let verdict_json = temp.path().join("verdict-report.json");
    fs::write(
        &rules,
        r#"
            [[rules]]
            kind = "limit"
            id = "position-z-max"
            channel = "position_z_m"
            metric = "max"
            max = 1000.0

            [[rules]]
            kind = "monotonic"
            id = "velocity-z-decreasing"
            channel = "velocity_z_m_s"
            direction = "decreasing"
        "#,
    )
    .expect("write verdict rules");
    let options = run::RunDataOptions::new(
        None,
        None,
        "openbmp-e2e-code".to_owned(),
        Some(rules),
        Some(verdict_json.clone()),
        None,
    );

    let report = run::run_with_options(&staged, &run::OutputOverrides::default(), &options)
        .expect("run with verdict report");

    let verdict = report
        .verdict_report
        .as_ref()
        .expect("verdict report summary should be present");
    assert!(verdict.passed);
    assert_eq!(verdict.verdict_count, 2);
    assert!(verdict_json.exists(), "verdict JSON should be written");
    assert!(report.written.contains(&verdict_json));

    let json = fs::read(&verdict_json).expect("read verdict report");
    let parsed: VerdictReport = serde_json::from_slice(&json).expect("parse verdict report");
    assert!(parsed.passed());
    assert_eq!(parsed.verdicts.len(), 2);
    assert_eq!(json, parsed.to_json_bytes().expect("stable verdict bytes"));
}

#[test]
fn verdict_flags_fail_closed_when_output_is_missing() {
    let temp = tempdir_for("openbmp_e2e_verdict_flags");
    let rules = temp.path().join("rules.toml");
    fs::write(
        &rules,
        "[[rules]]\nkind = 'limit'\nid = 'x'\nchannel = 'x'\nmetric = 'max'\nmax = 1.0\n",
    )
    .expect("write rules");
    let options = run::RunDataOptions::new(
        None,
        None,
        "openbmp-e2e-code".to_owned(),
        Some(rules),
        None,
        None,
    );

    let err = run::run_with_options(
        &temp.path().join("missing-scenario.toml"),
        &run::OutputOverrides::default(),
        &options,
    )
    .unwrap_err();
    assert!(matches!(err, CliError::RunDataConfig { .. }));
}

// ---------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------

#[derive(Debug)]
struct FinalState {
    position_z_m: f64,
    velocity_z_m_s: f64,
}

fn read_final_state(parquet_path: &Path) -> FinalState {
    use arrow::array::Float64Array;
    use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;

    let file = fs::File::open(parquet_path).expect("open parquet");
    let builder = ParquetRecordBatchReaderBuilder::try_new(file).expect("parquet builder");
    let schema = builder.schema().clone();
    let position_z_col = schema
        .index_of("position_z_m")
        .expect("position_z_m column present");
    let velocity_z_col = schema
        .index_of("velocity_z_m_s")
        .expect("velocity_z_m_s column present");

    let reader = builder.build().expect("parquet reader");
    let mut last_pz = f64::NAN;
    let mut last_vz = f64::NAN;
    for batch in reader {
        let batch = batch.expect("read batch");
        let position_z = batch
            .column(position_z_col)
            .as_any()
            .downcast_ref::<Float64Array>()
            .expect("position_z is float64");
        let velocity_z = batch
            .column(velocity_z_col)
            .as_any()
            .downcast_ref::<Float64Array>()
            .expect("velocity_z is float64");
        if batch.num_rows() > 0 {
            last_pz = position_z.value(batch.num_rows() - 1);
            last_vz = velocity_z.value(batch.num_rows() - 1);
        }
    }
    FinalState {
        position_z_m: last_pz,
        velocity_z_m_s: last_vz,
    }
}

fn tempdir_for(label: &str) -> TempDir {
    Builder::new()
        .prefix(&format!("{label}-"))
        .tempdir()
        .expect("tempdir must construct")
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(digest.len() * 2);
    for byte in digest {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}
