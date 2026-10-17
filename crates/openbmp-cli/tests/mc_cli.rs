//! CLI-facing tests for Monte Carlo utility commands.

#![allow(clippy::expect_used)]

use std::fs;
use std::path::{Path, PathBuf};

use openbmp_cli::commands::mc;
use openbmp_rundata::campaign::{CampaignManifest, CampaignOverlayMode};
use openbmp_uq::{
    CorrelatedErrorBudget, CorrelationMatrix, CredibilityFactor, CredibilityLevel,
    CredibilityRecord, UncertaintyClass, UncertaintySource,
};
use sha2::{Digest, Sha256};
use tempfile::Builder;

const MC_ENGINE_SCENARIO: &str = include_str!("fixtures/mc-scheduled-engine-fault.toml");
const MC_ACOUSTICS_ASCENT_SCENARIO_TEMPLATE: &str =
    include_str!("fixtures/mc-acoustics-ascent-campaign.toml");
const MC_ACOUSTICS_RESPONSE_SCENARIO_TEMPLATE: &str =
    include_str!("fixtures/mc-acoustics-response-campaign.toml");
const THERMOCHEM_ENGINE_DECK: &str =
    include_str!("../../openbmp-runner/tests/fixtures/thermochem/synthetic-grain.toml");
const AERO_SIGMA_DECK_TOML: &str = include_str!("fixtures/mc-aero-sigma-deck.toml");

fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn toml_literal_path(path: &Path) -> String {
    let path = path.to_string_lossy();
    assert!(
        !path.contains('\''),
        "test temp paths must be representable as TOML literal strings: {path}",
    );
    format!("'{path}'")
}

fn sha256_hex(bytes: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    let digest = hasher.finalize();
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn write_acoustics_ascent_base_scenario(path: &Path) {
    let deck = workspace_root().join("data/acoustics/ascent-acoustic-env-v1.toml");
    let deck_bytes = fs::read(&deck).expect("read ascent deck");
    let deck_sha256 = sha256_hex(&deck_bytes);
    let scenario = MC_ACOUSTICS_ASCENT_SCENARIO_TEMPLATE
        .replace("@DECK_FILE@", &toml_literal_path(&deck))
        .replace("@DECK_SHA256@", &deck_sha256);
    fs::write(path, scenario).expect("write acoustics ascent base scenario");
}

fn write_acoustics_response_campaign_base_scenario(path: &Path) {
    let workspace = workspace_root();
    let ascent_deck = workspace.join("data/acoustics/ascent-acoustic-env-v1.toml");
    let fairing_deck = workspace.join("data/acoustics/fairing-internal-v1.toml");
    let response_deck = workspace.join("data/acoustics/vibroacoustic-transfer-v1.toml");
    let ascent_deck_sha256 = sha256_hex(&fs::read(&ascent_deck).expect("read ascent deck"));
    let fairing_deck_sha256 = sha256_hex(&fs::read(&fairing_deck).expect("read fairing deck"));
    let response_deck_sha256 = sha256_hex(&fs::read(&response_deck).expect("read response deck"));
    let scenario = MC_ACOUSTICS_RESPONSE_SCENARIO_TEMPLATE
        .replace("@ASCENT_DECK@", &toml_literal_path(&ascent_deck))
        .replace("@ASCENT_DECK_SHA256@", &ascent_deck_sha256)
        .replace("@FAIRING_DECK@", &toml_literal_path(&fairing_deck))
        .replace("@FAIRING_DECK_SHA256@", &fairing_deck_sha256)
        .replace("@RESPONSE_DECK@", &toml_literal_path(&response_deck))
        .replace("@RESPONSE_DECK_SHA256@", &response_deck_sha256);
    fs::write(path, scenario).expect("write acoustics response campaign base scenario");
}

fn aero_deck_draw_overlay_options<'a>(
    samples: u64,
    first_sample_index: u64,
    dimension_id_base: u32,
    mach_correlation_length: Option<f64>,
    overlay_prefix: &'a str,
    output_dir: &'a Path,
    manifest_toml: &'a Path,
) -> mc::McAeroDeckDrawOverlaysOptions<'a> {
    mc::McAeroDeckDrawOverlaysOptions {
        samples,
        campaign_seed: 0x00A3_E0DB,
        first_sample_index,
        dimension_id_base,
        mach_correlation_length,
        alpha_correlation_length_deg: None,
        beta_correlation_length_deg: None,
        overlay_prefix,
        output_dir,
        manifest_toml,
    }
}

fn write_aero_deck_draw_base_scenario(path: &Path) {
    let scenario = include_str!("fixtures/mc-aero-deck-draw-campaign.toml");
    fs::write(path, scenario).expect("write aero deck draw base scenario");
}

fn aero_sigma_deck_with_correlation_toml() -> String {
    let mut deck = AERO_SIGMA_DECK_TOML.to_owned();
    deck.push_str(
        r#"
[uncertainty.correlation]
mach_length = 1.5
"#,
    );
    deck
}

fn aero_grid_sigma_deck_with_correlation_toml() -> &'static str {
    include_str!("fixtures/mc-aero-grid-sigma-deck.toml")
}

fn aero_six_sigma_deck_with_correlation_toml() -> String {
    let mut deck = AERO_SIGMA_DECK_TOML.to_owned();
    deck.push_str(
        r#"
[six_coefficients.cx]
data = [0.11, 0.12, 0.13]

[six_coefficients.cy]
data = [0.21, 0.22, 0.23]

[six_coefficients.cz]
data = [0.31, 0.32, 0.33]

[six_coefficients.cl]
data = [0.01, 0.02, 0.03]

[six_coefficients.cm]
data = [0.03, 0.04, 0.05]

[six_coefficients.cn]
data = [0.05, 0.06, 0.07]

[uncertainty.six_coefficients.cx]
data = [0.05, 0.05, 0.05]

[uncertainty.six_coefficients.cy]
data = [0.05, 0.05, 0.05]

[uncertainty.six_coefficients.cz]
data = [0.05, 0.05, 0.05]

[uncertainty.six_coefficients.cl]
data = [0.05, 0.05, 0.05]

[uncertainty.six_coefficients.cm]
data = [0.05, 0.05, 0.05]

[uncertainty.six_coefficients.cn]
data = [0.05, 0.05, 0.05]

[uncertainty.correlation]
mach_length = 1.5
"#,
    );
    deck
}

fn append_aero_deck_draw_upstream_uq(path: &Path) {
    let mut scenario = fs::read_to_string(path).expect("read aero deck draw scenario");
    scenario.push_str(
        r#"

[aero.uq]
deck_id = "synthetic_sigma_deck.v1"
coefficient_id = "cn"
coefficient_lower = 0.07
coefficient_nominal = 0.10
coefficient_upper = 0.12
credibility_level = 2
evidence = "crates/openbmp-cli/tests/mc_cli.rs#AERO_SIGMA_DECK_TOML"
"#,
    );
    fs::write(path, scenario).expect("write aero deck draw scenario with upstream uq");
}

#[test]
fn mc_summarize_reads_scalar_samples_and_success_interval() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let path = temp.path().join("samples.csv");
    fs::write(
        &path,
        "sample_index,perigee_km,success\n10,200.0,true\n12,220.0,true\n15,180.0,false\n",
    )
    .expect("write samples");

    let report = mc::summarize(&path, "perigee_km", Some("success"), 0.95).expect("summarize");
    assert_eq!(report.samples, 3);
    assert_eq!(report.mean.to_bits(), 200.0_f64.to_bits());
    assert!(report.standard_error.is_some());
    let success = report.success.expect("success summary");
    assert_eq!(success.trials, 3);
    assert_eq!(success.successes, 2);
    assert!(success.lower < success.fraction);
    assert!(success.upper > success.fraction);
}

#[test]
fn mc_summarize_rejects_missing_value_column() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let path = temp.path().join("samples.csv");
    fs::write(&path, "sample_index,value\n0,1.0\n").expect("write samples");

    let err = mc::summarize(&path, "missing", None, 0.95).expect_err("missing column");
    assert!(err.to_string().contains("missing column `missing`"));
}

#[test]
fn mc_acoustics_ascent_trace_materializes_valid_toml_rows() {
    let temp = Builder::new()
        .prefix("openbmp_mc_acoustics")
        .tempdir()
        .expect("tempdir");
    let output_a = temp.path().join("trace-a.toml");
    let output_b = temp.path().join("trace-b.toml");
    let options = |output_toml| mc::McAcousticsAscentTraceOptions {
        samples: 3,
        campaign_seed: 0xAC05_7105,
        design: mc::McAcousticsAscentTraceDesign::LatinHypercube,
        dynamic_pressure_min_pa: 50_000.0,
        dynamic_pressure_max_pa: 200_000.0,
        mach_min: 0.5,
        mach_max: 0.95,
        sample_prefix: "ascent-mc",
        class_id: Some("attached_turbulent"),
        output_toml,
    };

    let report = mc::write_acoustics_ascent_trace(options(&output_a)).expect("write trace");
    mc::write_acoustics_ascent_trace(options(&output_b)).expect("write repeat trace");

    assert_eq!(report.samples, 3);
    assert_eq!(report.output_toml, output_a);
    let text_a = fs::read_to_string(&output_a).expect("read trace a");
    let text_b = fs::read_to_string(&output_b).expect("read trace b");
    assert_eq!(text_a, text_b);

    let parsed: toml::Value = toml::from_str(&text_a).expect("trace TOML parses");
    let samples = parsed
        .get("acoustics")
        .and_then(|value| value.get("ascent"))
        .and_then(|value| value.get("samples"))
        .and_then(toml::Value::as_array)
        .expect("acoustics ascent samples array");
    assert_eq!(samples.len(), 3);
    for (index, sample) in samples.iter().enumerate() {
        assert_eq!(
            sample
                .get("sample_id")
                .and_then(toml::Value::as_str)
                .expect("sample id"),
            format!("ascent-mc-{index:06}")
        );
        assert_eq!(
            sample.get("class_id").and_then(toml::Value::as_str),
            Some("attached_turbulent")
        );
        let q = sample
            .get("dynamic_pressure_pa")
            .and_then(toml::Value::as_float)
            .expect("dynamic pressure");
        let mach = sample
            .get("mach")
            .and_then(toml::Value::as_float)
            .expect("mach");
        assert!((50_000.0..200_000.0).contains(&q));
        assert!((0.5..0.95).contains(&mach));
    }
}

#[test]
fn mc_acoustics_ascent_trace_rejects_single_sample() {
    let temp = Builder::new()
        .prefix("openbmp_mc_acoustics_bad")
        .tempdir()
        .expect("tempdir");
    let output = temp.path().join("trace.toml");
    let err = mc::write_acoustics_ascent_trace(mc::McAcousticsAscentTraceOptions {
        samples: 1,
        campaign_seed: 1,
        design: mc::McAcousticsAscentTraceDesign::Sobol,
        dynamic_pressure_min_pa: 50_000.0,
        dynamic_pressure_max_pa: 200_000.0,
        mach_min: 0.5,
        mach_max: 0.95,
        sample_prefix: "ascent-mc",
        class_id: None,
        output_toml: &output,
    })
    .expect_err("single sample rejected");

    assert!(err.to_string().contains("requires at least two samples"));
}

#[test]
fn mc_acoustics_response_samples_materializes_valid_toml_rows() {
    let temp = Builder::new()
        .prefix("openbmp_mc_acoustics_response")
        .tempdir()
        .expect("tempdir");
    let output_a = temp.path().join("response-a.toml");
    let output_b = temp.path().join("response-b.toml");
    let options = |output_toml| mc::McAcousticsResponseSamplesOptions {
        samples: 3,
        campaign_seed: 0xAC05_2206,
        design: mc::McAcousticsAscentTraceDesign::OwenSobol,
        surface_mass_min_kg_m2: 12.0,
        surface_mass_max_kg_m2: 24.0,
        area_min_m2: 2.0,
        area_max_m2: 5.0,
        mode_shape_min: -0.25,
        mode_shape_max: 0.75,
        sample_prefix: "panel-mc",
        class_id: Some("synthetic_panel_heavy"),
        output_toml,
    };

    let report =
        mc::write_acoustics_response_samples(options(&output_a)).expect("write response samples");
    mc::write_acoustics_response_samples(options(&output_b))
        .expect("write repeat response samples");

    assert_eq!(report.samples, 3);
    assert_eq!(report.output_toml, output_a);
    let text_a = fs::read_to_string(&output_a).expect("read response samples a");
    let text_b = fs::read_to_string(&output_b).expect("read response samples b");
    assert_eq!(text_a, text_b);

    let parsed: toml::Value = toml::from_str(&text_a).expect("response sample TOML parses");
    let samples = parsed
        .get("acoustics")
        .and_then(|value| value.get("response"))
        .and_then(|value| value.get("samples"))
        .and_then(toml::Value::as_array)
        .expect("acoustics response samples array");
    assert_eq!(samples.len(), 3);
    for (index, sample) in samples.iter().enumerate() {
        assert_eq!(
            sample
                .get("sample_id")
                .and_then(toml::Value::as_str)
                .expect("sample id"),
            format!("panel-mc-{index:06}")
        );
        assert_eq!(
            sample.get("class_id").and_then(toml::Value::as_str),
            Some("synthetic_panel_heavy")
        );
        let surface_mass = sample
            .get("surface_mass_kg_m2")
            .and_then(toml::Value::as_float)
            .expect("surface mass");
        let area = sample
            .get("area_m2")
            .and_then(toml::Value::as_float)
            .expect("area");
        let mode_shape = sample
            .get("mode_shape_factor")
            .and_then(toml::Value::as_float)
            .expect("mode shape");
        assert!((12.0..24.0).contains(&surface_mass));
        assert!((2.0..5.0).contains(&area));
        assert!((-0.25..0.75).contains(&mode_shape));
    }
}

#[test]
fn mc_acoustics_response_samples_rejects_single_sample() {
    let temp = Builder::new()
        .prefix("openbmp_mc_acoustics_response_bad")
        .tempdir()
        .expect("tempdir");
    let output = temp.path().join("response.toml");
    let err = mc::write_acoustics_response_samples(mc::McAcousticsResponseSamplesOptions {
        samples: 1,
        campaign_seed: 1,
        design: mc::McAcousticsAscentTraceDesign::Sobol,
        surface_mass_min_kg_m2: 12.0,
        surface_mass_max_kg_m2: 24.0,
        area_min_m2: 2.0,
        area_max_m2: 5.0,
        mode_shape_min: 0.25,
        mode_shape_max: 0.75,
        sample_prefix: "panel-mc",
        class_id: None,
        output_toml: &output,
    })
    .expect_err("single response sample rejected");

    assert!(err.to_string().contains("requires at least two samples"));
}

#[test]
fn mc_aero_deck_draw_overlays_materializes_campaign_fragment() {
    let temp = Builder::new()
        .prefix("openbmp_mc_aero_deck_draw")
        .tempdir()
        .expect("tempdir");
    let output_a = temp.path().join("overlays-a");
    let output_b = temp.path().join("overlays-b");
    let manifest_a = output_a.join("manifest-fragment.toml");
    let manifest_b = output_b.join("manifest-fragment.toml");

    let mut options =
        aero_deck_draw_overlay_options(3, 10, 120, Some(1.5), "draw", &output_a, &manifest_a);
    options.alpha_correlation_length_deg = Some(4.0);
    options.beta_correlation_length_deg = Some(3.0);
    let report = mc::write_aero_deck_draw_overlays(options).expect("write aero draw overlays");
    let mut repeat_options =
        aero_deck_draw_overlay_options(3, 10, 120, Some(1.5), "draw", &output_b, &manifest_b);
    repeat_options.alpha_correlation_length_deg = Some(4.0);
    repeat_options.beta_correlation_length_deg = Some(3.0);
    mc::write_aero_deck_draw_overlays(repeat_options).expect("write repeat aero draw overlays");

    assert_eq!(report.samples, 3);
    assert_eq!(report.campaign_seed, 0x00A3_E0DB);
    assert_eq!(report.first_sample_index, 10);
    assert_eq!(report.overlay_paths.len(), 3);
    let manifest_text = fs::read_to_string(&manifest_a).expect("read manifest fragment");
    let repeat_manifest_text =
        fs::read_to_string(&manifest_b).expect("read repeat manifest fragment");
    assert_eq!(manifest_text, repeat_manifest_text);
    assert!(manifest_text.contains("overlays = ["));
    assert!(manifest_text.contains("mode = \"merge\""));
    assert!(manifest_text.contains("path = \"draw-000010.toml\""));

    let full_manifest = format!(
        "campaign_id = \"aero-draw\"\nseeds = [0]\n\n{manifest_text}\n[[scenarios]]\nid = \"base\"\npath = \"base.toml\"\n"
    );
    let parsed_manifest =
        CampaignManifest::from_toml_str(&full_manifest).expect("campaign manifest parses");
    assert_eq!(
        parsed_manifest.overlays,
        vec![
            "draw-000010".to_owned(),
            "draw-000011".to_owned(),
            "draw-000012".to_owned()
        ]
    );
    assert_eq!(parsed_manifest.overlay_files.len(), 3);
    assert_eq!(
        parsed_manifest.overlay_files[0].mode,
        CampaignOverlayMode::Merge
    );

    for (offset, overlay_path) in report.overlay_paths.iter().enumerate() {
        let sample_index = 10 + offset as u64;
        let repeat_path = output_b.join(format!("draw-{sample_index:06}.toml"));
        let overlay_text = fs::read_to_string(overlay_path).expect("read overlay");
        let repeat_text = fs::read_to_string(repeat_path).expect("read repeat overlay");
        assert_eq!(overlay_text, repeat_text);
        let parsed: toml::Value = toml::from_str(&overlay_text).expect("overlay TOML parses");
        let deck_draw = parsed
            .get("aero")
            .and_then(|value| value.get("deck_draw"))
            .expect("aero deck_draw table");
        assert_eq!(
            deck_draw
                .get("campaign_seed")
                .and_then(toml::Value::as_integer),
            Some(0x00A3_E0DB)
        );
        assert_eq!(
            deck_draw
                .get("sample_index")
                .and_then(toml::Value::as_integer),
            Some(i64::try_from(sample_index).expect("sample index fits i64"))
        );
        assert_eq!(
            deck_draw
                .get("dimension_id_base")
                .and_then(toml::Value::as_integer),
            Some(120)
        );
        assert_eq!(
            deck_draw
                .get("mach_correlation_length")
                .and_then(toml::Value::as_float)
                .expect("mach correlation length")
                .to_bits(),
            1.5_f64.to_bits()
        );
        assert_eq!(
            deck_draw
                .get("alpha_correlation_length_deg")
                .and_then(toml::Value::as_float)
                .expect("alpha correlation length")
                .to_bits(),
            4.0_f64.to_bits()
        );
        assert_eq!(
            deck_draw
                .get("beta_correlation_length_deg")
                .and_then(toml::Value::as_float)
                .expect("beta correlation length")
                .to_bits(),
            3.0_f64.to_bits()
        );
    }
}

#[test]
fn mc_aero_deck_draw_overlays_can_omit_mach_correlation_length() {
    let temp = Builder::new()
        .prefix("openbmp_mc_aero_deck_draw_default")
        .tempdir()
        .expect("tempdir");
    let output_dir = temp.path().join("overlays");
    let manifest_toml = output_dir.join("manifest-fragment.toml");

    let report = mc::write_aero_deck_draw_overlays(aero_deck_draw_overlay_options(
        1,
        4,
        120,
        None,
        "draw",
        &output_dir,
        &manifest_toml,
    ))
    .expect("write aero draw overlay without explicit Mach correlation length");

    let overlay_text = fs::read_to_string(&report.overlay_paths[0]).expect("read overlay");
    assert!(overlay_text.contains("[aero.deck_draw]"));
    assert!(overlay_text.contains("sample_index = 4"));
    assert!(!overlay_text.contains("mach_correlation_length"));
    let parsed: toml::Value = toml::from_str(&overlay_text).expect("overlay TOML parses");
    let deck_draw = parsed
        .get("aero")
        .and_then(|value| value.get("deck_draw"))
        .expect("aero deck_draw table");
    assert!(deck_draw.get("mach_correlation_length").is_none());
}

#[test]
fn mc_aero_deck_draw_overlays_rejects_invalid_schedule() {
    let temp = Builder::new()
        .prefix("openbmp_mc_aero_deck_draw_bad")
        .tempdir()
        .expect("tempdir");
    let output_dir = temp.path().join("overlays");
    let manifest_toml = output_dir.join("manifest-fragment.toml");

    let no_samples = mc::write_aero_deck_draw_overlays(aero_deck_draw_overlay_options(
        0,
        0,
        0,
        Some(1.0),
        "draw",
        &output_dir,
        &manifest_toml,
    ))
    .expect_err("zero samples rejected");
    assert!(no_samples.to_string().contains("at least one sample"));

    let overflowing_index = mc::write_aero_deck_draw_overlays(aero_deck_draw_overlay_options(
        2,
        u64::MAX,
        0,
        Some(1.0),
        "draw",
        &output_dir,
        &manifest_toml,
    ))
    .expect_err("overflowing sample index rejected");
    assert!(overflowing_index.to_string().contains("overflows u64"));

    let overflowing_dimension = mc::write_aero_deck_draw_overlays(aero_deck_draw_overlay_options(
        1,
        0,
        u32::MAX - 1,
        Some(1.0),
        "draw",
        &output_dir,
        &manifest_toml,
    ))
    .expect_err("overflowing dimension id rejected");
    assert!(
        overflowing_dimension
            .to_string()
            .contains("CN, CD, and CM RNG dimensions")
    );

    let bad_mach_length = mc::write_aero_deck_draw_overlays(aero_deck_draw_overlay_options(
        1,
        0,
        0,
        Some(0.0),
        "draw",
        &output_dir,
        &manifest_toml,
    ))
    .expect_err("bad Mach correlation length rejected");
    assert!(bad_mach_length.to_string().contains("finite and positive"));

    let mut bad_alpha_options =
        aero_deck_draw_overlay_options(1, 0, 0, Some(1.0), "draw", &output_dir, &manifest_toml);
    bad_alpha_options.alpha_correlation_length_deg = Some(0.0);
    let bad_alpha_length = mc::write_aero_deck_draw_overlays(bad_alpha_options)
        .expect_err("bad alpha correlation length rejected");
    assert!(
        bad_alpha_length
            .to_string()
            .contains("alpha_correlation_length_deg")
    );

    let bad_prefix = mc::write_aero_deck_draw_overlays(aero_deck_draw_overlay_options(
        1,
        0,
        0,
        Some(1.0),
        "../draw",
        &output_dir,
        &manifest_toml,
    ))
    .expect_err("bad prefix rejected");
    assert!(bad_prefix.to_string().contains("overlay_prefix"));
}

#[test]
fn mc_aero_deck_draw_campaign_executes_generated_overlays() {
    let temp = Builder::new()
        .prefix("openbmp_mc_aero_deck_draw_campaign")
        .tempdir()
        .expect("tempdir");
    let scenario = temp.path().join("scenario.toml");
    let deck = temp.path().join("deck.toml");
    let output_dir = temp.path().join("campaign-artifacts");
    write_aero_deck_draw_base_scenario(&scenario);
    fs::write(&deck, AERO_SIGMA_DECK_TOML).expect("write sigma deck");

    let report = mc::run_aero_deck_draw_campaign(mc::McAeroDeckDrawCampaignOptions {
        scenario_path: &scenario,
        samples: 2,
        campaign_seed: 0x00A3_E0DB,
        first_sample_index: 4,
        dimension_id_base: 120,
        mach_correlation_length: Some(1.5),
        alpha_correlation_length_deg: None,
        beta_correlation_length_deg: None,
        overlay_prefix: "draw",
        campaign_id: "aero-draw-campaign",
        scenario_id: "base",
        run_seed: 7,
        output_dir: &output_dir,
        run_code_id: "test-aero-draw",
        code_ordinal: 3,
        write_index_parquet: false,
        workers: 1,
        credibility: None,
    })
    .expect("run aero deck draw campaign");

    assert_eq!(report.samples, 2);
    assert_eq!(report.overlays.overlay_paths.len(), 2);
    assert!(report.campaign_manifest_toml.exists());
    assert_eq!(report.campaign.run_count, 2);
    assert_eq!(report.campaign.campaign_id, "aero-draw-campaign");
    assert!(report.campaign.run_index_jsonl.exists());
    assert!(report.campaign.summary_json.exists());

    let manifest =
        fs::read_to_string(&report.campaign_manifest_toml).expect("read generated manifest");
    assert!(manifest.contains("campaign_id = \"aero-draw-campaign\""));
    assert!(manifest.contains("seeds = [7]"));
    assert!(manifest.contains("path = \"overlays/draw-000004.toml\""));
    assert!(manifest.contains("mode = \"merge\""));

    let mut observed_sample_indices = Vec::new();
    for run in &report.campaign.runs {
        let scenario_artifact = report
            .campaign_output_dir
            .join("runs")
            .join(&run.run_key)
            .join("scenario.toml");
        let scenario_text = fs::read_to_string(&scenario_artifact).expect("read run scenario");
        assert!(scenario_text.contains("[aero.deck_draw]"));
        assert!(scenario_text.contains("campaign_seed = 10739931"));
        if scenario_text.contains("sample_index = 4") {
            observed_sample_indices.push(4);
        }
        if scenario_text.contains("sample_index = 5") {
            observed_sample_indices.push(5);
        }
    }
    observed_sample_indices.sort_unstable();
    assert_eq!(observed_sample_indices, vec![4, 5]);
}

#[test]
fn mc_aero_deck_draw_campaign_uses_deck_correlation_default() {
    let temp = Builder::new()
        .prefix("openbmp_mc_aero_deck_draw_campaign_default")
        .tempdir()
        .expect("tempdir");
    let scenario = temp.path().join("scenario.toml");
    let deck = temp.path().join("deck.toml");
    let output_dir = temp.path().join("campaign-artifacts");
    write_aero_deck_draw_base_scenario(&scenario);
    fs::write(&deck, aero_sigma_deck_with_correlation_toml()).expect("write sigma deck");

    let report = mc::run_aero_deck_draw_campaign(mc::McAeroDeckDrawCampaignOptions {
        scenario_path: &scenario,
        samples: 1,
        campaign_seed: 0x00A3_E0DB,
        first_sample_index: 4,
        dimension_id_base: 120,
        mach_correlation_length: None,
        alpha_correlation_length_deg: None,
        beta_correlation_length_deg: None,
        overlay_prefix: "draw",
        campaign_id: "aero-draw-campaign-default",
        scenario_id: "base",
        run_seed: 7,
        output_dir: &output_dir,
        run_code_id: "test-aero-draw-default",
        code_ordinal: 3,
        write_index_parquet: false,
        workers: 1,
        credibility: None,
    })
    .expect("run aero deck draw campaign with deck-declared Mach correlation");

    assert_eq!(report.samples, 1);
    assert_eq!(report.campaign.run_count, 1);
    let overlay_text = fs::read_to_string(&report.overlays.overlay_paths[0]).expect("read overlay");
    assert!(!overlay_text.contains("mach_correlation_length"));
    let scenario_artifact = report
        .campaign_output_dir
        .join("runs")
        .join(&report.campaign.runs[0].run_key)
        .join("scenario.toml");
    let scenario_text = fs::read_to_string(&scenario_artifact).expect("read run scenario");
    assert!(scenario_text.contains("[aero.deck_draw]"));
    assert!(!scenario_text.contains("mach_correlation_length"));
    assert!(report.campaign.summary_json.exists());
}

#[test]
fn mc_aero_deck_draw_campaign_executes_alpha_beta_grid_covariance() {
    let temp = Builder::new()
        .prefix("openbmp_mc_aero_deck_draw_campaign_grid")
        .tempdir()
        .expect("tempdir");
    let scenario = temp.path().join("scenario.toml");
    let deck = temp.path().join("deck.toml");
    let output_dir = temp.path().join("campaign-artifacts");
    write_aero_deck_draw_base_scenario(&scenario);
    fs::write(&deck, aero_grid_sigma_deck_with_correlation_toml()).expect("write grid sigma deck");

    let report = mc::run_aero_deck_draw_campaign(mc::McAeroDeckDrawCampaignOptions {
        scenario_path: &scenario,
        samples: 2,
        campaign_seed: 0x00A3_E0DB,
        first_sample_index: 14,
        dimension_id_base: 120,
        mach_correlation_length: None,
        alpha_correlation_length_deg: Some(4.0),
        beta_correlation_length_deg: Some(3.0),
        overlay_prefix: "draw",
        campaign_id: "aero-draw-campaign-grid",
        scenario_id: "base",
        run_seed: 7,
        output_dir: &output_dir,
        run_code_id: "test-aero-draw-grid",
        code_ordinal: 3,
        write_index_parquet: false,
        workers: 1,
        credibility: None,
    })
    .expect("run aero deck draw campaign with alpha/beta grid covariance");

    assert_eq!(report.samples, 2);
    assert_eq!(report.campaign.run_count, 2);
    assert!(report.campaign.summary_json.exists());
    for overlay_path in &report.overlays.overlay_paths {
        let overlay_text = fs::read_to_string(overlay_path).expect("read overlay");
        assert!(!overlay_text.contains("mach_correlation_length"));
        assert!(overlay_text.contains("alpha_correlation_length_deg = 4.0"));
        assert!(overlay_text.contains("beta_correlation_length_deg = 3.0"));
    }

    let mut observed_sample_indices = Vec::new();
    for run in &report.campaign.runs {
        let scenario_artifact = report
            .campaign_output_dir
            .join("runs")
            .join(&run.run_key)
            .join("scenario.toml");
        let scenario_text = fs::read_to_string(&scenario_artifact).expect("read run scenario");
        assert!(scenario_text.contains("[aero.deck_draw]"));
        assert!(!scenario_text.contains("mach_correlation_length"));
        assert!(scenario_text.contains("alpha_correlation_length_deg = 4.0"));
        assert!(scenario_text.contains("beta_correlation_length_deg = 3.0"));
        if scenario_text.contains("sample_index = 14") {
            observed_sample_indices.push(14);
        }
        if scenario_text.contains("sample_index = 15") {
            observed_sample_indices.push(15);
        }
    }
    observed_sample_indices.sort_unstable();
    assert_eq!(observed_sample_indices, vec![14, 15]);
}

#[test]
fn mc_aero_deck_draw_campaign_preflights_six_sigma_dimension_capacity() {
    let temp = Builder::new()
        .prefix("openbmp_mc_aero_deck_draw_campaign_six_dim")
        .tempdir()
        .expect("tempdir");
    let scenario = temp.path().join("scenario.toml");
    let deck = temp.path().join("deck.toml");
    let output_dir = temp.path().join("campaign-artifacts");
    write_aero_deck_draw_base_scenario(&scenario);
    fs::write(&deck, aero_six_sigma_deck_with_correlation_toml()).expect("write six sigma deck");

    let err = mc::run_aero_deck_draw_campaign(mc::McAeroDeckDrawCampaignOptions {
        scenario_path: &scenario,
        samples: 1,
        campaign_seed: 0x00A3_E0DB,
        first_sample_index: 4,
        dimension_id_base: u32::MAX - 7,
        mach_correlation_length: None,
        alpha_correlation_length_deg: None,
        beta_correlation_length_deg: None,
        overlay_prefix: "draw",
        campaign_id: "aero-draw-campaign-six-dim",
        scenario_id: "base",
        run_seed: 7,
        output_dir: &output_dir,
        run_code_id: "test-aero-draw-six-dim",
        code_ordinal: 3,
        write_index_parquet: false,
        workers: 1,
        credibility: None,
    })
    .expect_err("six sigma deck requires nine RNG dimensions");

    assert!(err.to_string().contains("9 aero deck-draw RNG dimensions"));
    assert!(!output_dir.join("campaign.toml").exists());
}

#[test]
fn mc_aero_deck_draw_campaign_attaches_uq_credibility_report() {
    let temp = Builder::new()
        .prefix("openbmp_mc_aero_deck_draw_campaign_uq")
        .tempdir()
        .expect("tempdir");
    let scenario = temp.path().join("scenario.toml");
    let deck = temp.path().join("deck.toml");
    let uq = temp.path().join("uq").join("budget.toml");
    let report_md = temp.path().join("reports").join("credibility.md");
    let output_dir = temp.path().join("campaign-artifacts");
    write_aero_deck_draw_base_scenario(&scenario);
    append_aero_deck_draw_upstream_uq(&scenario);
    fs::write(&deck, AERO_SIGMA_DECK_TOML).expect("write sigma deck");
    fs::create_dir_all(uq.parent().expect("uq parent")).expect("uq dir");
    fs::write(&uq, uq_budget_toml(2)).expect("write uq");

    let report = mc::run_aero_deck_draw_campaign(mc::McAeroDeckDrawCampaignOptions {
        scenario_path: &scenario,
        samples: 1,
        campaign_seed: 0x00A3_E0DB,
        first_sample_index: 4,
        dimension_id_base: 120,
        mach_correlation_length: Some(1.5),
        alpha_correlation_length_deg: None,
        beta_correlation_length_deg: None,
        overlay_prefix: "draw",
        campaign_id: "aero-draw-campaign-uq",
        scenario_id: "base",
        run_seed: 7,
        output_dir: &output_dir,
        run_code_id: "test-aero-draw-uq",
        code_ordinal: 3,
        write_index_parquet: false,
        workers: 1,
        credibility: Some(mc::McCredibilityOptions {
            uq_toml: &uq,
            floor: CredibilityLevel::L2,
            report_md: Some(&report_md),
        }),
    })
    .expect("run aero deck draw campaign with credibility");

    let credibility = report.credibility.expect("credibility report");
    assert_eq!(credibility.binding_level, CredibilityLevel::L2);
    assert!(credibility.accepted);
    assert!((credibility.aggregate_one_sigma - 25.0009_f64.sqrt()).abs() < 1.0e-12);
    assert_eq!(
        report.credibility_report_md.as_deref(),
        Some(report_md.as_path())
    );
    assert!(report.campaign.summary_json.exists());
    let markdown = fs::read_to_string(&report_md).expect("read report");
    assert!(markdown.contains("Monte Carlo credibility report"));
    assert!(markdown.contains("Binding credibility:** L2"));
    assert!(markdown.contains("03.aero.synthetic_sigma_deck.v1.cn"));
}

#[test]
fn mc_aero_deck_draw_campaign_refuses_uq_below_credibility_floor() {
    let temp = Builder::new()
        .prefix("openbmp_mc_aero_deck_draw_campaign_uq_bad")
        .tempdir()
        .expect("tempdir");
    let scenario = temp.path().join("scenario.toml");
    let deck = temp.path().join("deck.toml");
    let uq = temp.path().join("uq").join("budget.toml");
    let report_md = temp.path().join("reports").join("credibility.md");
    let output_dir = temp.path().join("campaign-artifacts");
    write_aero_deck_draw_base_scenario(&scenario);
    fs::write(&deck, AERO_SIGMA_DECK_TOML).expect("write sigma deck");
    fs::create_dir_all(uq.parent().expect("uq parent")).expect("uq dir");
    fs::write(&uq, uq_budget_toml(2)).expect("write uq");

    let err = mc::run_aero_deck_draw_campaign(mc::McAeroDeckDrawCampaignOptions {
        scenario_path: &scenario,
        samples: 1,
        campaign_seed: 0x00A3_E0DB,
        first_sample_index: 4,
        dimension_id_base: 120,
        mach_correlation_length: Some(1.5),
        alpha_correlation_length_deg: None,
        beta_correlation_length_deg: None,
        overlay_prefix: "draw",
        campaign_id: "aero-draw-campaign-uq-bad",
        scenario_id: "base",
        run_seed: 7,
        output_dir: &output_dir,
        run_code_id: "test-aero-draw-uq-bad",
        code_ordinal: 3,
        write_index_parquet: false,
        workers: 1,
        credibility: Some(mc::McCredibilityOptions {
            uq_toml: &uq,
            floor: CredibilityLevel::L3,
            report_md: Some(&report_md),
        }),
    })
    .expect_err("credibility floor should fail");

    assert!(err.to_string().contains("below requested floor L3"));
    assert!(report_md.exists());
    assert!(
        !output_dir
            .join("campaign-run")
            .join("summary.json")
            .exists()
    );
}

#[test]
fn mc_acoustics_ascent_campaign_runs_generated_trace() {
    let temp = Builder::new()
        .prefix("openbmp_mc_acoustics_campaign")
        .tempdir()
        .expect("tempdir");
    let scenario = temp.path().join("scenario.toml");
    let output_csv = temp.path().join("ascent-campaign.csv");
    let output_scenario = temp.path().join("generated-scenario.toml");
    write_acoustics_ascent_base_scenario(&scenario);

    let report = mc::run_acoustics_ascent_campaign(mc::McAcousticsAscentCampaignOptions {
        scenario_path: &scenario,
        samples: 3,
        campaign_seed: 0xAC05_CA11,
        design: mc::McAcousticsAscentTraceDesign::OwenSobol,
        dynamic_pressure_min_pa: 50_000.0,
        dynamic_pressure_max_pa: 200_000.0,
        mach_min: 0.5,
        mach_max: 0.95,
        sample_prefix: "campaign",
        class_id: Some("attached_turbulent"),
        output_csv: &output_csv,
        output_artifact_dir: None,
        output_scenario_toml: Some(&output_scenario),
        checkpoint_json: None,
        max_new_samples: None,
        workers: 1,
        toolchain_profile: "test",
    })
    .expect("run acoustics ascent campaign");

    assert_eq!(report.samples, 3);
    assert_eq!(report.completed_samples, 3);
    assert!(report.complete);
    assert_eq!(report.checkpoint_json, None);
    assert_eq!(report.output_csv, output_csv);
    assert_eq!(report.output_artifact_dir, None);
    assert!(report.artifact_paths.is_empty());
    assert_eq!(report.artifact_manifest_json, None);
    assert_eq!(report.output_scenario_toml, Some(output_scenario.clone()));
    assert!(report.p95_50_oaspl_db.is_some());
    assert!(report.p95_50_wilks_required_sample_count.is_some());
    assert_eq!(report.p95_50_wilks_sample_count_sufficient, Some(false));
    assert!(report.peak_sample_oaspl_db.is_finite());

    let generated = fs::read_to_string(&output_scenario).expect("read generated scenario");
    assert!(generated.contains("[[acoustics.ascent.samples]]"));
    assert!(generated.contains("campaign-000000"));

    let csv = fs::read_to_string(&output_csv).expect("read ascent campaign CSV");
    assert!(csv.contains("sample_index,sample_id,class_id,dynamic_pressure_pa,mach,oaspl_db"));
    assert_eq!(csv.lines().count(), 4);
    assert!(csv.contains("campaign-000002"));
}

#[test]
fn mc_acoustics_ascent_campaign_reports_downstream_p95_50() {
    let temp = Builder::new()
        .prefix("openbmp_mc_acoustics_downstream_campaign")
        .tempdir()
        .expect("tempdir");
    let scenario = temp.path().join("scenario.toml");
    let output_csv = temp.path().join("ascent-campaign.csv");
    let artifact_dir = temp.path().join("campaign-artifacts");
    write_acoustics_response_campaign_base_scenario(&scenario);

    let report = mc::run_acoustics_ascent_campaign(mc::McAcousticsAscentCampaignOptions {
        scenario_path: &scenario,
        samples: 3,
        campaign_seed: 0xAC05_CA12,
        design: mc::McAcousticsAscentTraceDesign::OwenSobol,
        dynamic_pressure_min_pa: 50_000.0,
        dynamic_pressure_max_pa: 200_000.0,
        mach_min: 0.5,
        mach_max: 0.95,
        sample_prefix: "campaign",
        class_id: Some("attached_turbulent"),
        output_csv: &output_csv,
        output_artifact_dir: Some(&artifact_dir),
        output_scenario_toml: None,
        checkpoint_json: None,
        max_new_samples: None,
        workers: 1,
        toolchain_profile: "test",
    })
    .expect("run acoustics ascent campaign with downstream response");

    assert_eq!(report.samples, 3);
    assert!(report.complete);
    assert_eq!(report.output_artifact_dir, Some(artifact_dir.clone()));
    assert_eq!(
        report.artifact_manifest_json,
        Some(artifact_dir.join("manifest.json"))
    );
    assert!(report.p95_50_oaspl_db.is_some());
    assert!(report.fairing_internal_p95_50_oaspl_db.is_some());
    assert!(
        report
            .fairing_internal_p95_50_wilks_required_sample_count
            .is_some()
    );
    assert_eq!(
        report.fairing_internal_p95_50_wilks_sample_count_sufficient,
        Some(false)
    );
    let response_acceleration = report
        .response_p95_50_total_mean_square_acceleration_m2_s4
        .expect("response P95/50 acceleration evidence");
    let response_force = report
        .response_p95_50_total_mean_square_force_n2
        .expect("response P95/50 force evidence");
    assert!(response_acceleration.is_finite() && response_acceleration > 0.0);
    assert!(response_force.is_finite() && response_force > 0.0);
    assert!(report.response_p95_50_wilks_required_sample_count.is_some());
    assert_eq!(
        report.response_p95_50_wilks_sample_count_sufficient,
        Some(false)
    );

    let csv = fs::read_to_string(&output_csv).expect("read ascent campaign CSV");
    assert_eq!(csv.lines().count(), 4);

    let expected_artifacts = [
        PathBuf::from("ascent/p95_50_environment.csv"),
        PathBuf::from("ascent/p95_50_wilks_sizing.csv"),
        PathBuf::from("fairing_internal/p95_50_environment.csv"),
        PathBuf::from("fairing_internal/p95_50_sample_trace.csv"),
        PathBuf::from("fairing_internal/p95_50_wilks_sizing.csv"),
        PathBuf::from("response/loads_handoff.csv"),
        PathBuf::from("response/p95_50_acceleration_psd.csv"),
        PathBuf::from("response/p95_50_modal_force_psd.csv"),
        PathBuf::from("response/p95_50_sample_trace.csv"),
        PathBuf::from("response/p95_50_wilks_sizing.csv"),
    ];
    let expected_paths = expected_artifacts
        .iter()
        .map(|path| artifact_dir.join(path))
        .collect::<Vec<_>>();
    assert_eq!(report.artifact_paths, expected_paths);
    for (relative_path, expected_header) in [
        ("ascent/p95_50_environment.csv", "p95_50_spl_db"),
        ("fairing_internal/p95_50_environment.csv", "p95_50_spl_db"),
        ("fairing_internal/p95_50_sample_trace.csv", "fill_factor"),
        ("response/loads_handoff.csv", "band_mean_square_force_n2"),
        (
            "response/p95_50_acceleration_psd.csv",
            "p95_50_acceleration_psd_m2_s4_per_hz",
        ),
        (
            "response/p95_50_modal_force_psd.csv",
            "p95_50_modal_force_psd_n2_per_hz",
        ),
        ("response/p95_50_sample_trace.csv", "surface_mass_kg_m2"),
    ] {
        let path = artifact_dir.join(relative_path);
        let artifact_csv = fs::read_to_string(&path).expect("read campaign artifact");
        assert!(
            artifact_csv.contains(expected_header),
            "{relative_path} should contain {expected_header}"
        );
        assert!(artifact_csv.lines().count() >= 2);
    }
    let manifest =
        fs::read_to_string(artifact_dir.join("manifest.json")).expect("read artifact manifest");
    assert!(manifest.contains("openbmp.acoustics_ascent_campaign_artifacts.v1"));
    assert!(manifest.contains("acoustics_response_p95_50_acceleration_psd_csv"));
    assert!(manifest.contains("response/p95_50_acceleration_psd.csv"));
    assert!(manifest.contains("sha256_hex"));
    assert_eq!(manifest.as_bytes().last(), Some(&b'\n'));
    let manifest_json: serde_json::Value =
        serde_json::from_str(&manifest).expect("parse artifact manifest");
    let entries = manifest_json["artifacts"]
        .as_array()
        .expect("artifact manifest entries");
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
    assert_eq!(entries.len(), expected_artifacts.len());
    let expected_entries = entries.iter().zip(&expected_artifacts).zip(expected_roles);
    for ((entry, relative_path), role) in expected_entries {
        let expected_path = relative_path.to_string_lossy().replace('\\', "/");
        assert_eq!(entry["path"].as_str(), Some(expected_path.as_str()));
        assert_eq!(entry["role"].as_str(), Some(role));
        let bytes = fs::read(artifact_dir.join(relative_path)).expect("read manifest artifact");
        let digest = sha256_hex(&bytes);
        assert_eq!(entry["sha256_hex"].as_str(), Some(digest.as_str()));
    }
}

#[test]
fn mc_acoustics_response_campaign_requires_response_and_reports_artifacts() {
    let temp = Builder::new()
        .prefix("openbmp_mc_acoustics_response_campaign")
        .tempdir()
        .expect("tempdir");
    let scenario = temp.path().join("scenario.toml");
    let output_csv = temp.path().join("response-campaign.csv");
    let output_scenario = temp.path().join("response-generated.toml");
    let artifact_dir = temp.path().join("response-campaign-artifacts");
    write_acoustics_response_campaign_base_scenario(&scenario);

    let report = mc::run_acoustics_response_campaign(mc::McAcousticsResponseCampaignOptions {
        scenario_path: &scenario,
        samples: 3,
        campaign_seed: 0xAC05_CA13,
        design: mc::McAcousticsAscentTraceDesign::OwenSobol,
        dynamic_pressure_min_pa: 50_000.0,
        dynamic_pressure_max_pa: 200_000.0,
        mach_min: 0.5,
        mach_max: 0.95,
        sample_prefix: "response-campaign",
        class_id: Some("attached_turbulent"),
        response_samples: Some(mc::McAcousticsResponseCampaignSamplesOptions {
            surface_mass_min_kg_m2: 12.0,
            surface_mass_max_kg_m2: 24.0,
            area_min_m2: 2.0,
            area_max_m2: 5.0,
            mode_shape_min: 0.25,
            mode_shape_max: 0.75,
            sample_prefix: "response-panel",
            class_id: Some("synthetic_panel_heavy"),
        }),
        output_csv: &output_csv,
        output_artifact_dir: Some(&artifact_dir),
        output_scenario_toml: Some(&output_scenario),
        checkpoint_json: None,
        max_new_samples: None,
        workers: 1,
        toolchain_profile: "test-response",
    })
    .expect("run acoustics response campaign");

    assert!(report.complete);
    assert_eq!(report.samples, 3);
    assert_eq!(report.output_csv, output_csv);
    assert!(
        report
            .response_p95_50_total_mean_square_acceleration_m2_s4
            .is_some()
    );
    assert!(report.response_p95_50_total_mean_square_force_n2.is_some());
    assert!(report.response_p95_50_wilks_required_sample_count.is_some());
    assert!(
        report
            .artifact_paths
            .contains(&artifact_dir.join("response/p95_50_acceleration_psd.csv"))
    );
    assert!(
        report
            .artifact_paths
            .contains(&artifact_dir.join("response/p95_50_modal_force_psd.csv"))
    );
    assert!(
        report
            .artifact_paths
            .contains(&artifact_dir.join("response/p95_50_sample_trace.csv"))
    );
    let csv = fs::read_to_string(&output_csv).expect("read response campaign CSV");
    assert!(csv.contains("sample_index,sample_id,class_id,dynamic_pressure_pa,mach,oaspl_db"));
    let generated = fs::read_to_string(&output_scenario).expect("read generated response scenario");
    assert!(generated.contains("[[acoustics.response.samples]]"));
    assert!(generated.contains("response-panel-000000"));
    assert!(generated.contains("synthetic_panel_heavy"));
    let response_trace = fs::read_to_string(artifact_dir.join("response/p95_50_sample_trace.csv"))
        .expect("read response sample trace artifact");
    assert!(response_trace.contains("response-panel-000000"));
    assert!(response_trace.contains("synthetic_panel_heavy"));
    let manifest =
        fs::read_to_string(artifact_dir.join("manifest.json")).expect("read artifact manifest");
    assert!(manifest.contains("openbmp.acoustics_ascent_campaign_artifacts.v1"));
    assert!(manifest.contains("acoustics_response_p95_50_modal_force_psd_csv"));
    assert!(manifest.contains("acoustics_response_p95_50_sample_trace_csv"));

    let missing_response_scenario = temp.path().join("ascent-only.toml");
    let missing_response_csv = temp.path().join("missing-response.csv");
    write_acoustics_ascent_base_scenario(&missing_response_scenario);
    let err = mc::run_acoustics_response_campaign(mc::McAcousticsResponseCampaignOptions {
        scenario_path: &missing_response_scenario,
        samples: 3,
        campaign_seed: 0xAC05_CA13,
        design: mc::McAcousticsAscentTraceDesign::OwenSobol,
        dynamic_pressure_min_pa: 50_000.0,
        dynamic_pressure_max_pa: 200_000.0,
        mach_min: 0.5,
        mach_max: 0.95,
        sample_prefix: "response-campaign",
        class_id: Some("attached_turbulent"),
        response_samples: None,
        output_csv: &missing_response_csv,
        output_artifact_dir: None,
        output_scenario_toml: None,
        checkpoint_json: None,
        max_new_samples: None,
        workers: 1,
        toolchain_profile: "test-response",
    })
    .expect_err("response campaign requires [acoustics.response]");
    assert!(
        err.to_string()
            .contains("requires base scenario [acoustics.response]")
    );

    let existing_response_samples_scenario = temp.path().join("existing-response-samples.toml");
    let existing_response_samples_csv = temp.path().join("existing-response-samples.csv");
    write_acoustics_response_campaign_base_scenario(&existing_response_samples_scenario);
    let mut existing_response_samples =
        fs::read_to_string(&existing_response_samples_scenario).expect("read response scenario");
    existing_response_samples.push_str(
        r#"
[[acoustics.response.samples]]
sample_id = "existing-response"
surface_mass_kg_m2 = 12.0
area_m2 = 2.0
mode_shape_factor = 0.5
"#,
    );
    fs::write(
        &existing_response_samples_scenario,
        existing_response_samples,
    )
    .expect("write response scenario with existing samples");
    let err = mc::run_acoustics_response_campaign(mc::McAcousticsResponseCampaignOptions {
        scenario_path: &existing_response_samples_scenario,
        samples: 3,
        campaign_seed: 0xAC05_CA13,
        design: mc::McAcousticsAscentTraceDesign::OwenSobol,
        dynamic_pressure_min_pa: 50_000.0,
        dynamic_pressure_max_pa: 200_000.0,
        mach_min: 0.5,
        mach_max: 0.95,
        sample_prefix: "response-campaign",
        class_id: Some("attached_turbulent"),
        response_samples: Some(mc::McAcousticsResponseCampaignSamplesOptions {
            surface_mass_min_kg_m2: 12.0,
            surface_mass_max_kg_m2: 24.0,
            area_min_m2: 2.0,
            area_max_m2: 5.0,
            mode_shape_min: 0.25,
            mode_shape_max: 0.75,
            sample_prefix: "response-panel",
            class_id: None,
        }),
        output_csv: &existing_response_samples_csv,
        output_artifact_dir: None,
        output_scenario_toml: None,
        checkpoint_json: None,
        max_new_samples: None,
        workers: 1,
        toolchain_profile: "test-response",
    })
    .expect_err("generated response samples reject existing response samples");
    assert!(
        err.to_string()
            .contains("must not already declare [[acoustics.response.samples]]")
    );
}

#[test]
fn mc_acoustics_ascent_campaign_rejects_existing_trace_samples() {
    let temp = Builder::new()
        .prefix("openbmp_mc_acoustics_campaign_bad")
        .tempdir()
        .expect("tempdir");
    let scenario = temp.path().join("scenario.toml");
    let output_csv = temp.path().join("ascent-campaign.csv");
    write_acoustics_ascent_base_scenario(&scenario);
    let mut text = fs::read_to_string(&scenario).expect("read base scenario");
    text.push_str(
        "\n[[acoustics.ascent.samples]]\nsample_id = \"existing\"\ndynamic_pressure_pa = 50000.0\nmach = 0.8\n",
    );
    fs::write(&scenario, text).expect("write scenario with existing trace");

    let err = mc::run_acoustics_ascent_campaign(mc::McAcousticsAscentCampaignOptions {
        scenario_path: &scenario,
        samples: 2,
        campaign_seed: 1,
        design: mc::McAcousticsAscentTraceDesign::LatinHypercube,
        dynamic_pressure_min_pa: 50_000.0,
        dynamic_pressure_max_pa: 200_000.0,
        mach_min: 0.5,
        mach_max: 0.95,
        sample_prefix: "campaign",
        class_id: None,
        output_csv: &output_csv,
        output_artifact_dir: None,
        output_scenario_toml: None,
        checkpoint_json: None,
        max_new_samples: None,
        workers: 1,
        toolchain_profile: "test",
    })
    .expect_err("existing trace should reject");

    assert!(
        err.to_string()
            .contains("must not already declare [[acoustics.ascent.samples]]")
    );
}

#[test]
fn mc_acoustics_ascent_campaign_checkpoint_resumes_to_final_outputs() {
    let temp = Builder::new()
        .prefix("openbmp_mc_acoustics_campaign_checkpoint")
        .tempdir()
        .expect("tempdir");
    let scenario = temp.path().join("scenario.toml");
    let checkpoint = temp.path().join("campaign.checkpoint.json");
    let output_csv = temp.path().join("checkpointed-ascent-campaign.csv");
    let output_scenario = temp.path().join("checkpointed-generated-scenario.toml");
    let one_shot_csv = temp.path().join("one-shot-ascent-campaign.csv");
    write_acoustics_ascent_base_scenario(&scenario);

    let partial = mc::run_acoustics_ascent_campaign(mc::McAcousticsAscentCampaignOptions {
        scenario_path: &scenario,
        samples: 3,
        campaign_seed: 0xAC05_CA11,
        design: mc::McAcousticsAscentTraceDesign::OwenSobol,
        dynamic_pressure_min_pa: 50_000.0,
        dynamic_pressure_max_pa: 200_000.0,
        mach_min: 0.5,
        mach_max: 0.95,
        sample_prefix: "campaign",
        class_id: Some("attached_turbulent"),
        output_csv: &output_csv,
        output_artifact_dir: None,
        output_scenario_toml: Some(&output_scenario),
        checkpoint_json: Some(&checkpoint),
        max_new_samples: Some(1),
        workers: 2,
        toolchain_profile: "test-aarch64-clean",
    })
    .expect("partial checkpointed acoustics ascent campaign");

    assert_eq!(partial.completed_samples, 1);
    assert!(!partial.complete);
    assert_eq!(partial.checkpoint_json, Some(checkpoint.clone()));
    assert!(checkpoint.exists());
    assert!(!output_csv.exists());
    assert!(!output_scenario.exists());
    assert!(partial.artifact_paths.is_empty());
    assert_eq!(partial.artifact_manifest_json, None);

    let complete = mc::run_acoustics_ascent_campaign(mc::McAcousticsAscentCampaignOptions {
        scenario_path: &scenario,
        samples: 3,
        campaign_seed: 0xAC05_CA11,
        design: mc::McAcousticsAscentTraceDesign::OwenSobol,
        dynamic_pressure_min_pa: 50_000.0,
        dynamic_pressure_max_pa: 200_000.0,
        mach_min: 0.5,
        mach_max: 0.95,
        sample_prefix: "campaign",
        class_id: Some("attached_turbulent"),
        output_csv: &output_csv,
        output_artifact_dir: None,
        output_scenario_toml: Some(&output_scenario),
        checkpoint_json: Some(&checkpoint),
        max_new_samples: Some(3),
        workers: 2,
        toolchain_profile: "test-aarch64-clean",
    })
    .expect("complete checkpointed acoustics ascent campaign");

    assert_eq!(complete.completed_samples, 3);
    assert!(complete.complete);
    assert_eq!(complete.checkpoint_json, Some(checkpoint));
    assert!(complete.p95_50_oaspl_db.is_some());
    assert_eq!(complete.p95_50_wilks_sample_count_sufficient, Some(false));
    let checkpointed_csv = fs::read_to_string(&output_csv).expect("read checkpointed CSV");
    assert_eq!(checkpointed_csv.lines().count(), 4);
    assert!(checkpointed_csv.contains("campaign-000002"));
    assert!(output_scenario.exists());

    let one_shot = mc::run_acoustics_ascent_campaign(mc::McAcousticsAscentCampaignOptions {
        scenario_path: &scenario,
        samples: 3,
        campaign_seed: 0xAC05_CA11,
        design: mc::McAcousticsAscentTraceDesign::OwenSobol,
        dynamic_pressure_min_pa: 50_000.0,
        dynamic_pressure_max_pa: 200_000.0,
        mach_min: 0.5,
        mach_max: 0.95,
        sample_prefix: "campaign",
        class_id: Some("attached_turbulent"),
        output_csv: &one_shot_csv,
        output_artifact_dir: None,
        output_scenario_toml: None,
        checkpoint_json: None,
        max_new_samples: None,
        workers: 1,
        toolchain_profile: "test",
    })
    .expect("one-shot acoustics ascent campaign");
    assert!(one_shot.complete);
    assert_eq!(
        checkpointed_csv,
        fs::read_to_string(&one_shot_csv).expect("read one-shot CSV")
    );
}

#[test]
fn mc_propulsion_fault_campaign_runs_sampled_overlays() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let scenario = temp.path().join("scenario.toml");
    let library = temp.path().join("fault-library.toml");
    let output = temp.path().join("samples").join("propulsion-faults.csv");
    fs::write(&scenario, MC_ENGINE_SCENARIO).expect("write scenario");
    fs::write(
        &library,
        r#"
[[faults]]
id = "mc-hardoff"
engine_id = "main"
start_step = 4
probability = 1.0
fault = { kind = "hard_off" }
"#,
    )
    .expect("write library");

    let report = mc::run_propulsion_fault_campaign(mc::McPropulsionFaultCampaignOptions {
        scenario_path: &scenario,
        library_toml: &library,
        samples: 2,
        campaign_seed: 0xF00D,
        dimension_id: 0,
        metric_channel: "engine.main.thrust_n",
        success_min: None,
        success_max: Some(1.0),
        output_csv: &output,
        confidence: 0.95,
        credibility: None,
    })
    .expect("run campaign");

    assert_eq!(report.samples, 2);
    assert_eq!(report.activated_samples, 2);
    assert!(report.summary.mean < 1.0);
    let success = report.summary.success.expect("success summary");
    assert_eq!(success.trials, 2);
    assert_eq!(success.successes, 2);

    let csv = fs::read_to_string(&output).expect("read campaign csv");
    assert!(csv.starts_with("sample_index,fault_ids,activated_fault_count,value,success\n"));
    assert!(csv.contains("0,mc-hardoff,1,"));
    assert!(csv.contains("1,mc-hardoff,1,"));
    assert!(csv.lines().skip(1).all(|line| line.ends_with(",true")));
}

#[test]
fn mc_propulsion_fault_campaign_consumes_runner_upstream_uq() {
    let temp = Builder::new()
        .prefix("openbmp_mc_runner_uq")
        .tempdir()
        .expect("tempdir");
    let scenario = temp.path().join("scenario.toml");
    let thermochem = temp.path().join("thermochem.toml");
    let library = temp.path().join("fault-library.toml");
    let output = temp.path().join("samples").join("propulsion-faults.csv");
    fs::write(&thermochem, THERMOCHEM_ENGINE_DECK).expect("write thermochem");
    fs::write(&scenario, thermochemical_mc_scenario()).expect("write scenario");
    fs::write(
        &library,
        r#"
[[faults]]
id = "mc-hardoff"
engine_id = "main"
start_step = 4
probability = 1.0
fault = { kind = "hard_off" }
"#,
    )
    .expect("write library");

    let report = mc::run_propulsion_fault_campaign(mc::McPropulsionFaultCampaignOptions {
        scenario_path: &scenario,
        library_toml: &library,
        samples: 1,
        campaign_seed: 0xF00D,
        dimension_id: 0,
        metric_channel: "engine.main.thrust_n",
        success_min: None,
        success_max: Some(1.0),
        output_csv: &output,
        confidence: 0.95,
        credibility: None,
    })
    .expect("run campaign");

    let credibility = report
        .credibility
        .expect("runner upstream UQ should attach credibility");
    assert!(credibility.accepted);
    assert_eq!(credibility.binding_level, CredibilityLevel::L1);
    assert!((credibility.aggregate_one_sigma - 0.02).abs() < 1.0e-15);
    assert_eq!(credibility.aleatory_one_sigma.to_bits(), 0.0_f64.to_bits());
    assert!((credibility.epistemic_one_sigma - 0.02).abs() < 1.0e-15);
    assert!(
        credibility
            .markdown
            .contains("05.propulsion.thermochem.fa3ff43bc91d.c_star_efficiency")
    );
    assert!(output.exists());
}

#[test]
fn mc_campaign_credibility_inputs_emit_upstream_only_report() {
    let upstream = CorrelatedErrorBudget {
        sources: vec![
            uq_source(
                "03.aero.deck.cn_band",
                3.0,
                UncertaintyClass::Epistemic,
                CredibilityLevel::L1,
            ),
            uq_source(
                "05.propulsion.thermochem.c_star_efficiency",
                4.0,
                UncertaintyClass::Epistemic,
                CredibilityLevel::L1,
            ),
        ],
        correlation: Some(
            CorrelationMatrix::new(2, vec![1.0, 0.5, 0.5, 1.0]).expect("correlation"),
        ),
    };

    let (report, report_path) = mc::evaluate_campaign_credibility_inputs(None, &upstream)
        .expect("evaluate upstream")
        .expect("upstream report");

    assert_eq!(report_path, None);
    assert_eq!(report.floor, CredibilityLevel::L0);
    assert_eq!(report.binding_level, CredibilityLevel::L1);
    assert!(report.accepted);
    assert!((report.aggregate_one_sigma - 37.0_f64.sqrt()).abs() < 1.0e-12);
    assert_eq!(report.aleatory_one_sigma.to_bits(), 0.0_f64.to_bits());
    assert!((report.epistemic_one_sigma - 37.0_f64.sqrt()).abs() < 1.0e-12);
    assert!(
        report
            .markdown
            .contains("05.propulsion.thermochem.c_star_efficiency")
    );
}

#[test]
fn mc_campaign_credibility_inputs_merge_sidecar_and_upstream_sources() {
    let temp = Builder::new()
        .prefix("openbmp_mc_upstream_uq")
        .tempdir()
        .expect("tempdir");
    let uq = temp.path().join("uq.toml");
    let report_md = temp.path().join("reports").join("credibility.md");
    fs::write(&uq, uq_budget_toml(2)).expect("write uq");
    let upstream = CorrelatedErrorBudget {
        sources: vec![uq_source(
            "04.aerothermal.q_conv_w_m2_band",
            12.0,
            UncertaintyClass::Epistemic,
            CredibilityLevel::L1,
        )],
        correlation: None,
    };

    let (report, report_path) = mc::evaluate_campaign_credibility_inputs(
        Some(mc::McCredibilityOptions {
            uq_toml: &uq,
            floor: CredibilityLevel::L1,
            report_md: Some(&report_md),
        }),
        &upstream,
    )
    .expect("evaluate merged")
    .expect("merged report");

    assert_eq!(report_path.as_deref(), Some(report_md.as_path()));
    assert_eq!(report.binding_level, CredibilityLevel::L1);
    assert!(report.accepted);
    assert!((report.aggregate_one_sigma - 13.0).abs() < 1.0e-12);
    assert!((report.aleatory_one_sigma - 3.0).abs() < 1.0e-12);
    assert!((report.epistemic_one_sigma - 160.0_f64.sqrt()).abs() < 1.0e-12);
    let markdown = fs::read_to_string(&report_md).expect("read report");
    assert!(markdown.contains("04.aerothermal.q_conv_w_m2_band"));
}

#[test]
fn mc_campaign_credibility_contract_merges_sidecar_and_upstream_sources() {
    let temp = Builder::new()
        .prefix("openbmp_mc_upstream_uq_contract")
        .tempdir()
        .expect("tempdir");
    let uq = temp.path().join("uq.toml");
    fs::write(&uq, uq_budget_toml(2)).expect("write uq");
    let upstream = CorrelatedErrorBudget {
        sources: vec![uq_source(
            "03.aero.deck.cl_band",
            5.0,
            UncertaintyClass::Epistemic,
            CredibilityLevel::L1,
        )],
        correlation: None,
    };

    let (report, report_path) =
        mc::evaluate_campaign_credibility_contract(mc::CampaignCredibilityInputs::new(
            Some(mc::McCredibilityOptions {
                uq_toml: &uq,
                floor: CredibilityLevel::L1,
                report_md: None,
            }),
            &upstream,
        ))
        .expect("evaluate contract")
        .expect("contract report");

    assert_eq!(report_path, None);
    assert_eq!(report.binding_level, CredibilityLevel::L1);
    assert!(report.accepted);
    assert!((report.aggregate_one_sigma - 50.0_f64.sqrt()).abs() < 1.0e-12);
    assert!((report.aleatory_one_sigma - 3.0).abs() < 1.0e-12);
    assert!((report.epistemic_one_sigma - 41.0_f64.sqrt()).abs() < 1.0e-12);
    assert!(report.markdown.contains("03.aero.deck.cl_band"));
}

#[test]
fn mc_propulsion_fault_campaign_attaches_uq_credibility_report() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let scenario = temp.path().join("scenario.toml");
    let library = temp.path().join("fault-library.toml");
    let output = temp.path().join("samples").join("propulsion-faults.csv");
    let uq = temp.path().join("uq.toml");
    let report_md = temp.path().join("reports").join("credibility.md");
    fs::write(&scenario, MC_ENGINE_SCENARIO).expect("write scenario");
    fs::write(
        &library,
        r#"
[[faults]]
id = "mc-hardoff"
engine_id = "main"
start_step = 4
probability = 1.0
fault = { kind = "hard_off" }
"#,
    )
    .expect("write library");
    fs::write(&uq, uq_budget_toml(2)).expect("write uq");

    let report = mc::run_propulsion_fault_campaign(mc::McPropulsionFaultCampaignOptions {
        scenario_path: &scenario,
        library_toml: &library,
        samples: 1,
        campaign_seed: 0xF00D,
        dimension_id: 0,
        metric_channel: "engine.main.thrust_n",
        success_min: None,
        success_max: Some(1.0),
        output_csv: &output,
        confidence: 0.95,
        credibility: Some(mc::McCredibilityOptions {
            uq_toml: &uq,
            floor: CredibilityLevel::L2,
            report_md: Some(&report_md),
        }),
    })
    .expect("run campaign");

    let credibility = report.credibility.expect("credibility report");
    assert_eq!(credibility.binding_level, CredibilityLevel::L2);
    assert!(credibility.accepted);
    assert_eq!(
        report.credibility_report_md.as_deref(),
        Some(report_md.as_path())
    );
    assert!(output.exists());
    let markdown = fs::read_to_string(&report_md).expect("read markdown");
    assert!(markdown.contains("# Monte Carlo credibility report"));
    assert!(markdown.contains("Binding credibility:** L2"));
}

#[test]
fn mc_propulsion_fault_campaign_refuses_uq_below_credibility_floor() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let scenario = temp.path().join("scenario.toml");
    let library = temp.path().join("fault-library.toml");
    let output = temp.path().join("samples").join("propulsion-faults.csv");
    let uq = temp.path().join("uq.toml");
    let report_md = temp.path().join("reports").join("credibility.md");
    fs::write(&scenario, MC_ENGINE_SCENARIO).expect("write scenario");
    fs::write(
        &library,
        r#"
[[faults]]
id = "mc-hardoff"
engine_id = "main"
start_step = 4
probability = 1.0
fault = { kind = "hard_off" }
"#,
    )
    .expect("write library");
    fs::write(&uq, uq_budget_toml(2)).expect("write uq");

    let err = mc::run_propulsion_fault_campaign(mc::McPropulsionFaultCampaignOptions {
        scenario_path: &scenario,
        library_toml: &library,
        samples: 1,
        campaign_seed: 0xF00D,
        dimension_id: 0,
        metric_channel: "engine.main.thrust_n",
        success_min: None,
        success_max: Some(1.0),
        output_csv: &output,
        confidence: 0.95,
        credibility: Some(mc::McCredibilityOptions {
            uq_toml: &uq,
            floor: CredibilityLevel::L3,
            report_md: Some(&report_md),
        }),
    })
    .expect_err("floor should fail");

    assert!(err.to_string().contains("below requested floor L3"));
    assert!(report_md.exists());
    assert!(!output.exists());
}

#[test]
fn mc_propulsion_fault_campaign_uses_library_declared_uq_report() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let scenario = temp.path().join("scenario.toml");
    let library_dir = temp.path().join("faults");
    let library = library_dir.join("fault-library.toml");
    let output = temp.path().join("samples").join("propulsion-faults.csv");
    let uq = temp.path().join("uq").join("budget.toml");
    let report_md = temp.path().join("reports").join("credibility.md");
    fs::create_dir_all(&library_dir).expect("library dir");
    fs::create_dir_all(uq.parent().expect("uq parent")).expect("uq dir");
    fs::write(&scenario, MC_ENGINE_SCENARIO).expect("write scenario");
    fs::write(&uq, uq_budget_toml(2)).expect("write uq");
    fs::write(
        &library,
        r#"
[uq]
budget_toml = "../uq/budget.toml"
credibility_floor = "l2"
report_md = "../reports/credibility.md"

[[faults]]
id = "mc-hardoff"
engine_id = "main"
start_step = 4
probability = 1.0
fault = { kind = "hard_off" }
"#,
    )
    .expect("write library");

    let report = mc::run_propulsion_fault_campaign(mc::McPropulsionFaultCampaignOptions {
        scenario_path: &scenario,
        library_toml: &library,
        samples: 1,
        campaign_seed: 0xF00D,
        dimension_id: 0,
        metric_channel: "engine.main.thrust_n",
        success_min: None,
        success_max: Some(1.0),
        output_csv: &output,
        confidence: 0.95,
        credibility: None,
    })
    .expect("run campaign");

    let credibility = report.credibility.expect("credibility report");
    assert_eq!(credibility.binding_level, CredibilityLevel::L2);
    assert!(credibility.accepted);
    let expected_report_md = library_dir.join("../reports/credibility.md");
    assert_eq!(
        report.credibility_report_md.as_deref(),
        Some(expected_report_md.as_path())
    );
    assert!(output.exists());
    let markdown = fs::read_to_string(&report_md).expect("read markdown");
    assert!(markdown.contains("Binding credibility:** L2"));
}

#[test]
fn mc_propulsion_fault_campaign_refuses_library_uq_below_floor() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let scenario = temp.path().join("scenario.toml");
    let library_dir = temp.path().join("faults");
    let library = library_dir.join("fault-library.toml");
    let output = temp.path().join("samples").join("propulsion-faults.csv");
    let uq = temp.path().join("uq").join("budget.toml");
    let report_md = temp.path().join("reports").join("credibility.md");
    fs::create_dir_all(&library_dir).expect("library dir");
    fs::create_dir_all(uq.parent().expect("uq parent")).expect("uq dir");
    fs::write(&scenario, MC_ENGINE_SCENARIO).expect("write scenario");
    fs::write(&uq, uq_budget_toml(2)).expect("write uq");
    fs::write(
        &library,
        r#"
[uq]
budget_toml = "../uq/budget.toml"
credibility_floor = "l3"
report_md = "../reports/credibility.md"

[[faults]]
id = "mc-hardoff"
engine_id = "main"
start_step = 4
probability = 1.0
fault = { kind = "hard_off" }
"#,
    )
    .expect("write library");

    let err = mc::run_propulsion_fault_campaign(mc::McPropulsionFaultCampaignOptions {
        scenario_path: &scenario,
        library_toml: &library,
        samples: 1,
        campaign_seed: 0xF00D,
        dimension_id: 0,
        metric_channel: "engine.main.thrust_n",
        success_min: None,
        success_max: Some(1.0),
        output_csv: &output,
        confidence: 0.95,
        credibility: None,
    })
    .expect_err("floor should fail");

    assert!(err.to_string().contains("below requested floor L3"));
    assert!(report_md.exists());
    assert!(!output.exists());
}

#[test]
fn mc_summarize_attaches_uq_credibility_report_and_floor() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let samples = temp.path().join("samples.csv");
    let uq = temp.path().join("uq.toml");
    let report_md = temp.path().join("reports").join("credibility.md");
    fs::write(
        &samples,
        "sample_index,value,success\n0,3.0,true\n1,5.0,true\n",
    )
    .expect("write samples");
    fs::write(&uq, uq_budget_toml(2)).expect("write uq");

    let report = mc::summarize_with_credibility(
        &samples,
        "value",
        Some("success"),
        0.95,
        Some(mc::McCredibilityOptions {
            uq_toml: &uq,
            floor: CredibilityLevel::L2,
            report_md: Some(&report_md),
        }),
    )
    .expect("summarize with credibility");

    assert_eq!(report.summary.samples, 2);
    let credibility = report.credibility.expect("credibility report");
    assert!(credibility.accepted);
    assert_eq!(credibility.binding_level, CredibilityLevel::L2);
    assert!((credibility.aggregate_one_sigma - 5.0).abs() < 1.0e-12);
    assert!((credibility.aleatory_one_sigma - 3.0).abs() < 1.0e-12);
    assert!((credibility.epistemic_one_sigma - 4.0).abs() < 1.0e-12);
    let markdown = fs::read_to_string(&report_md).expect("read report");
    assert!(markdown.contains("Monte Carlo credibility report"));
    assert!(markdown.contains("accepted = true"));
}

#[test]
fn mc_summarize_refuses_uq_below_credibility_floor() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let samples = temp.path().join("samples.csv");
    let uq = temp.path().join("uq.toml");
    let report_md = temp.path().join("credibility.md");
    fs::write(&samples, "sample_index,value\n0,3.0\n1,5.0\n").expect("write samples");
    fs::write(&uq, uq_budget_toml(2)).expect("write uq");

    let err = mc::summarize_with_credibility(
        &samples,
        "value",
        None,
        0.95,
        Some(mc::McCredibilityOptions {
            uq_toml: &uq,
            floor: CredibilityLevel::L3,
            report_md: Some(&report_md),
        }),
    )
    .expect_err("credibility floor must fail closed");

    assert!(
        err.to_string()
            .contains("credibility binding level L2 is below requested floor L3")
    );
    let markdown = fs::read_to_string(&report_md).expect("read report");
    assert!(markdown.contains("accepted = false"));
}

#[test]
fn mc_nested_summarize_reports_pbox_variance_split_and_writes_csv() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let samples = temp.path().join("nested.csv");
    let pbox = temp.path().join("out").join("pbox.csv");
    fs::write(
        &samples,
        "epistemic_index,value\n0,-3.0\n0,1.0\n1,-1.0\n1,3.0\n",
    )
    .expect("write nested samples");

    let report = mc::summarize_nested(&samples, "epistemic_index", "value", 1.0, 0.5, Some(&pbox))
        .expect("nested summarize");

    assert_eq!(report.analysis.epistemic_samples, 2);
    assert_eq!(report.analysis.min_aleatory_samples, 2);
    assert_eq!(report.analysis.max_aleatory_samples, 2);
    assert!((report.analysis.variance_split.aleatory - 4.0).abs() < 1.0e-12);
    assert!((report.analysis.variance_split.epistemic - 1.0).abs() < 1.0e-12);
    assert_eq!(report.analysis.lower_bound_probability, Some(0.5));
    assert_eq!(report.analysis.requirement_passed, Some(true));
    assert_eq!(report.pbox_csv.as_deref(), Some(pbox.as_path()));
    let pbox_text = fs::read_to_string(&pbox).expect("read pbox");
    assert!(pbox_text.starts_with("support,lower_cdf,upper_cdf\n"));
    assert!(
        pbox_text.contains("1.00000000000000000e0,5.00000000000000000e-1,1.00000000000000000e0")
    );
}

#[test]
fn mc_nested_summarize_requirement_uses_lower_pbox_bound() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let samples = temp.path().join("nested.csv");
    fs::write(
        &samples,
        "epistemic_index,value\n0,0.0\n0,0.0\n1,10.0\n1,10.0\n",
    )
    .expect("write nested samples");

    let report = mc::summarize_nested(&samples, "epistemic_index", "value", 0.0, 0.5, None)
        .expect("nested summarize");

    assert_eq!(report.analysis.lower_bound_probability, Some(0.0));
    assert_eq!(report.analysis.requirement_passed, Some(false));
}

#[test]
fn mc_resume_scalar_checkpoint_records_chunks_and_final_summary() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let samples = temp.path().join("samples.csv");
    let checkpoint = temp.path().join("checkpoint.json");
    fs::write(
        &samples,
        "sample_index,value,success\n0,1.0,true\n1,2.0,true\n2,3.0,false\n3,4.0,true\n",
    )
    .expect("write samples");

    let partial = mc::resume_scalar_checkpoint(
        &samples,
        &checkpoint,
        "value",
        Some("success"),
        4,
        0xC0FFEE,
        0,
        "scenario-a",
        "x86_64-clean",
        2,
        2,
        0.95,
    )
    .expect("partial resume");
    assert_eq!(partial.completed, 2);
    assert!(!partial.is_complete());

    let complete = mc::resume_scalar_checkpoint(
        &samples,
        &checkpoint,
        "value",
        Some("success"),
        4,
        0xC0FFEE,
        0,
        "scenario-a",
        "x86_64-clean",
        4,
        4,
        0.95,
    )
    .expect("complete resume");
    assert_eq!(complete.completed, 4);
    let summary = complete.summary.expect("final summary");
    assert_eq!(summary.samples, 4);
    assert_eq!(summary.mean.to_bits(), 2.5_f64.to_bits());
    let success = summary.success.expect("success summary");
    assert_eq!(success.trials, 4);
    assert_eq!(success.successes, 3);
}

#[test]
fn mc_resume_scalar_checkpoint_rejects_scenario_mismatch() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let samples = temp.path().join("samples.csv");
    let checkpoint = temp.path().join("checkpoint.json");
    fs::write(&samples, "sample_index,value\n0,1.0\n1,2.0\n").expect("write samples");

    mc::resume_scalar_checkpoint(
        &samples,
        &checkpoint,
        "value",
        None,
        2,
        0xC0FFEE,
        0,
        "scenario-a",
        "x86_64-clean",
        1,
        1,
        0.95,
    )
    .expect("create checkpoint");
    let err = mc::resume_scalar_checkpoint(
        &samples,
        &checkpoint,
        "value",
        None,
        2,
        0xC0FFEE,
        0,
        "scenario-b",
        "x86_64-clean",
        1,
        1,
        0.95,
    )
    .expect_err("scenario mismatch");
    assert!(
        err.to_string()
            .contains("checkpoint metadata mismatch in scenario_sha256")
    );
}

#[test]
fn mc_wilks_reports_two_sided_sample_size() {
    let report =
        mc::wilks_sample_size(mc::WilksSide::TwoSided, 0.99865, 0.90).expect("wilks sample size");
    assert_eq!(report.samples, 2880);
    assert!(report.attained_confidence >= report.confidence);
    assert_eq!(report.side.label(), "two-sided");
}

#[test]
fn mc_wilks_rejects_invalid_probability() {
    let err =
        mc::wilks_sample_size(mc::WilksSide::OneSided, 1.0, 0.90).expect_err("invalid coverage");
    assert!(
        err.to_string()
            .contains("coverage is outside the Monte Carlo statistics envelope")
    );
}

#[test]
fn mc_lhs_writes_unit_cube_design_csv() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let path = temp.path().join("lhs.csv");
    let report = mc::write_latin_hypercube(5, 2, 0x5EED, &path).expect("write lhs");
    assert_eq!(report.samples, 5);
    assert_eq!(report.dimensions, 2);

    let mut reader = csv::Reader::from_path(&path).expect("read lhs");
    let headers = reader.headers().expect("headers").clone();
    assert_eq!(&headers[0], "sample_index");
    assert_eq!(&headers[1], "u0");
    assert_eq!(&headers[2], "u1");

    let mut strata = [vec![false; 5], vec![false; 5]];
    for (row_index, row) in reader.records().enumerate() {
        let row = row.expect("row");
        assert_eq!(
            row.get(0).expect("sample_index"),
            row_index.to_string().as_str()
        );
        for (dimension, stratum_seen) in strata.iter_mut().enumerate() {
            let value = row
                .get(dimension + 1)
                .expect("value")
                .parse::<f64>()
                .expect("float");
            assert!((0.0..1.0).contains(&value));
            let stratum = (value * 5.0).floor() as usize;
            assert!(!stratum_seen[stratum]);
            stratum_seen[stratum] = true;
        }
    }
    assert!(strata.into_iter().flatten().all(|seen| seen));
}

#[test]
fn mc_sobol_writes_reference_design_csv() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let path = temp.path().join("sobol.csv");
    let report = mc::write_sobol(4, 2, None, &path).expect("write sobol");
    assert_eq!(report.samples, 4);
    assert_eq!(report.dimensions, 2);
    assert_eq!(report.scramble_seed, None);

    let (_, columns) = read_design_columns(&path);
    assert_eq!(
        columns[0]
            .iter()
            .map(|value| value.to_bits())
            .collect::<Vec<_>>(),
        [0.0_f64, 0.5, 0.75, 0.25]
            .into_iter()
            .map(f64::to_bits)
            .collect::<Vec<_>>()
    );
    assert_eq!(
        columns[1]
            .iter()
            .map(|value| value.to_bits())
            .collect::<Vec<_>>(),
        [0.0_f64, 0.5, 0.25, 0.75]
            .into_iter()
            .map(f64::to_bits)
            .collect::<Vec<_>>()
    );
}

#[test]
fn mc_sobol_writes_scrambled_design_csv() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let base_path = temp.path().join("sobol.csv");
    let scrambled_path = temp.path().join("sobol-scrambled.csv");
    mc::write_sobol(8, 2, None, &base_path).expect("write sobol");
    let report = mc::write_sobol(8, 2, Some(0x0E11), &scrambled_path).expect("write scrambled");
    assert_eq!(report.scramble_seed, Some(0x0E11));
    let (_, base_columns) = read_design_columns(&base_path);
    let (_, scrambled_columns) = read_design_columns(&scrambled_path);
    assert_ne!(base_columns, scrambled_columns);
    assert!(
        scrambled_columns
            .iter()
            .flatten()
            .all(|value| (0.0..1.0).contains(value))
    );
}

#[test]
fn mc_iman_conover_writes_correlated_design_csv() {
    let temp = Builder::new()
        .prefix("openbmp_mc")
        .tempdir()
        .expect("tempdir");
    let design_path = temp.path().join("lhs.csv");
    let correlation_path = temp.path().join("correlation.csv");
    let output_path = temp.path().join("correlated.csv");
    mc::write_latin_hypercube(128, 2, 0x1C0E, &design_path).expect("write lhs");
    fs::write(&correlation_path, "1.0,0.7\n0.7,1.0\n").expect("write correlation");

    let report =
        mc::write_iman_conover(&design_path, &correlation_path, &output_path).expect("correlate");
    assert_eq!(report.samples, 128);
    assert_eq!(report.dimensions, 2);

    let (input_headers, input_columns) = read_design_columns(&design_path);
    let (output_headers, output_columns) = read_design_columns(&output_path);
    assert_eq!(input_headers, output_headers);
    assert_eq!(
        sorted_bits(&input_columns[0]),
        sorted_bits(&output_columns[0])
    );
    assert_eq!(
        sorted_bits(&input_columns[1]),
        sorted_bits(&output_columns[1])
    );
    let correlation = pearson(&output_columns[0], &output_columns[1]);
    assert!(
        correlation > 0.6,
        "expected positive induced correlation, got {correlation}"
    );
}

fn uq_budget_toml(level: u8) -> String {
    let factors = [
        "verification",
        "validation",
        "input_pedigree",
        "results_uncertainty",
        "results_robustness",
        "use_history",
        "ms_management",
        "people_qualification",
    ];
    let mut text = String::new();
    for (id, one_sigma, class) in [("wind", 3.0, "aleatory"), ("aero", 4.0, "epistemic")] {
        text.push_str("[[sources]]\n");
        text.push_str(&format!("id = \"{id}\"\n"));
        text.push_str(&format!("one_sigma = {one_sigma}\n"));
        text.push_str(&format!("class = \"{class}\"\n"));
        text.push_str("justification = \"synthetic test budget\"\n");
        for factor in factors {
            text.push_str(&format!(
                "\n[sources.credibility.{factor}]\nlevel = {level}\nevidence = \"V-UQ-TEST\"\n"
            ));
        }
        text.push('\n');
    }
    text
}

fn uq_source(
    source_id: &str,
    one_sigma: f64,
    class: UncertaintyClass,
    level: CredibilityLevel,
) -> UncertaintySource {
    UncertaintySource {
        source_id: source_id.to_owned(),
        one_sigma,
        class,
        credibility: credibility_record(level),
        justification: "synthetic upstream deck band".to_owned(),
    }
}

fn credibility_record(level: CredibilityLevel) -> CredibilityRecord {
    let mut record = CredibilityRecord::new();
    for factor in CredibilityFactor::ALL {
        record = record.with_score(factor, level, format!("V-{factor:?}"));
    }
    record
}

fn thermochemical_mc_scenario() -> String {
    MC_ENGINE_SCENARIO
        .replace(
            "limits = { max_thrust_n = 1000.0, isp_s = 250.0, ignition_transient_s = 0.0, shutdown_transient_s = 0.0, max_gimbal_rad = 0.0 }\n",
            "limits = { max_thrust_n = 1000.0, isp_s = 250.0, ignition_transient_s = 0.0, shutdown_transient_s = 0.0, max_gimbal_rad = 0.0 }\n\
             \n\
             [vehicle.assembly.engines.thermochemical_performance]\n\
             throat_area_m2 = 0.02\n\
             exit_area_m2 = 0.24\n\
             ambient_pressure_pa = 101325.0\n\
             separation = \"off\"\n",
        )
        .replace(
            "[environment]\n",
            "[propulsion.thermochem]\n\
             file = \"thermochem.toml\"\n\
             chamber_pressure_pa = 2000000.0\n\
             mixture_ratio = 2.5\n\
             \n\
             [environment]\n",
        )
}

fn read_design_columns(path: &std::path::Path) -> (Vec<String>, Vec<Vec<f64>>) {
    let mut reader = csv::Reader::from_path(path).expect("read design");
    let headers = reader
        .headers()
        .expect("headers")
        .iter()
        .map(str::to_owned)
        .collect::<Vec<_>>();
    let mut columns = vec![Vec::new(); headers.len() - 1];
    for row in reader.records() {
        let row = row.expect("row");
        for (dimension, column) in columns.iter_mut().enumerate() {
            column.push(
                row.get(dimension + 1)
                    .expect("value")
                    .parse::<f64>()
                    .expect("float"),
            );
        }
    }
    (headers, columns)
}

fn sorted_bits(values: &[f64]) -> Vec<u64> {
    let mut values = values.to_vec();
    values.sort_by(f64::total_cmp);
    values.into_iter().map(f64::to_bits).collect()
}

fn pearson(left: &[f64], right: &[f64]) -> f64 {
    let left_mean = left.iter().sum::<f64>() / left.len() as f64;
    let right_mean = right.iter().sum::<f64>() / right.len() as f64;
    let mut covariance = 0.0;
    let mut left_m2 = 0.0;
    let mut right_m2 = 0.0;
    for (left, right) in left.iter().zip(right.iter()) {
        let dl = left - left_mean;
        let dr = right - right_mean;
        covariance += dl * dr;
        left_m2 += dl * dl;
        right_m2 += dr * dr;
    }
    covariance / (left_m2 * right_m2).sqrt()
}
