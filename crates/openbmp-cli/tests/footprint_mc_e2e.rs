//! End-to-end test for offline footprint Monte-Carlo reporting.

#![allow(clippy::expect_used, clippy::panic)]

use std::fs;
use std::path::{Path, PathBuf};

use openbmp_cli::commands::{footprint_mc, mc};
use openbmp_uq::CredibilityLevel;
use tempfile::Builder;

fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .expect("workspace root must exist relative to CARGO_MANIFEST_DIR")
}

fn fixture_path() -> PathBuf {
    workspace_root().join("crates/openbmp-scenario/tests/fixtures/coast-footprint-mc-valid.toml")
}

fn toml_literal_path(path: &Path) -> String {
    let path = path.to_string_lossy();
    assert!(
        !path.contains('\''),
        "test temp paths must be representable as TOML literal strings: {path}",
    );
    format!("'{path}'")
}

#[test]
fn footprint_mc_writes_declared_outputs() {
    let temp = Builder::new()
        .prefix("openbmp_footprint_mc")
        .tempdir()
        .expect("tempdir");
    let csv = temp.path().join("samples.csv");
    let parquet = temp.path().join("samples.parquet");
    let summary = temp.path().join("summary.toml");

    let original = fs::read_to_string(fixture_path()).expect("read fixture");
    let rewritten = original
        .replace(
            r#"samples_csv = "out/coast-footprint-mc-samples.csv""#,
            &format!("samples_csv = {}", toml_literal_path(&csv)),
        )
        .replace(
            r#"samples_parquet = "out/coast-footprint-mc-samples.parquet""#,
            &format!("samples_parquet = {}", toml_literal_path(&parquet)),
        )
        .replace(
            r#"summary_toml = "out/coast-footprint-mc-summary.toml""#,
            &format!("summary_toml = {}", toml_literal_path(&summary)),
        );
    let scenario = temp.path().join("scenario.toml");
    fs::write(&scenario, rewritten).expect("write staged scenario");

    let report =
        footprint_mc::run(&scenario, None, None, "host-clean", None).expect("footprint mc run");
    assert_eq!(report.samples_requested, 32);
    assert_eq!(report.samples_completed, 32);
    assert_eq!(report.samples_succeeded, 32);
    assert_eq!(report.samples_failed, 0);
    assert!(report.complete);
    assert_eq!(report.written.len(), 3);
    let csv_text = fs::read_to_string(&csv).expect("read csv");
    assert!(csv_text.contains("radial_offset_from_nominal_m"));
    assert!(!csv_text.contains("position_x_eci_m"));
    assert!(!csv_text.contains("velocity_x_eci_m_s"));
    assert!(!csv_text.contains("ballistic_coefficient_m2_kg"));
    assert!(!csv_text.contains("wind_x_m_s"));
    assert!(parquet.metadata().expect("parquet metadata").len() > 0);
    let summary_text = fs::read_to_string(summary).expect("read summary");
    assert!(summary_text.contains("[dispersion_statistics]"));
    assert!(summary_text.contains("radial_dispersion_p50_m"));
    assert!(summary_text.contains("[dispersion_ellipse]"));
    assert!(summary_text.contains("[[quantiles]]"));
    assert!(summary_text.contains("[[nominal_radial_offset_quantiles]]"));
}

#[test]
fn footprint_mc_nested_writes_pbox_csv_and_summary() {
    let temp = Builder::new()
        .prefix("openbmp_footprint_nested_mc")
        .tempdir()
        .expect("tempdir");
    let csv = temp.path().join("samples.csv");
    let parquet = temp.path().join("samples.parquet");
    let summary = temp.path().join("summary.toml");
    let pbox = temp.path().join("pbox.csv");

    let original = fs::read_to_string(fixture_path()).expect("read fixture");
    let rewritten = original
        .replace(
            r#"samples_csv = "out/coast-footprint-mc-samples.csv""#,
            &format!("samples_csv = {}", toml_literal_path(&csv)),
        )
        .replace(
            r#"samples_parquet = "out/coast-footprint-mc-samples.parquet""#,
            &format!("samples_parquet = {}", toml_literal_path(&parquet)),
        )
        .replace(
            r#"summary_toml = "out/coast-footprint-mc-summary.toml""#,
            &format!("summary_toml = {}", toml_literal_path(&summary)),
        )
        .replace(
            "[landing_footprint.monte_carlo.wind]\nkind = \"constant\"",
            "[landing_footprint.monte_carlo.wind]\nuncertainty_class = \"epistemic\"\nkind = \"constant\"",
        )
        + &format!(
            "\n[landing_footprint.monte_carlo.nested]\n\
             epistemic_samples = 4\n\
             aleatory_samples = 8\n\
             metric = \"radial_offset_from_nominal_m\"\n\
             threshold = 1.0e9\n\
             minimum_probability = 1.0\n\
             pbox_csv = {}\n",
            toml_literal_path(&pbox),
        );
    let scenario = temp.path().join("scenario.toml");
    fs::write(&scenario, rewritten).expect("write staged scenario");

    let report =
        footprint_mc::run(&scenario, None, None, "host-clean", None).expect("footprint mc run");
    assert_eq!(report.samples_requested, 32);
    assert_eq!(report.samples_succeeded, 32);
    assert!(report.complete);
    assert!(report.written.contains(&pbox));

    let pbox_text = fs::read_to_string(&pbox).expect("read pbox csv");
    assert!(pbox_text.starts_with("support,lower_cdf,upper_cdf\n"));
    assert!(pbox_text.lines().count() > 1);
    let summary_text = fs::read_to_string(summary).expect("read summary");
    assert!(summary_text.contains("[nested_uq]"));
    assert!(summary_text.contains("requirement_passed = true"));
}

#[test]
fn footprint_mc_checkpoint_resume_matches_one_shot_outputs() {
    let one_shot = Builder::new()
        .prefix("openbmp_footprint_mc_one_shot")
        .tempdir()
        .expect("tempdir");
    let resumed = Builder::new()
        .prefix("openbmp_footprint_mc_resume")
        .tempdir()
        .expect("tempdir");

    let one_shot_csv = one_shot.path().join("samples.csv");
    let one_shot_parquet = one_shot.path().join("samples.parquet");
    let one_shot_summary = one_shot.path().join("summary.toml");
    let one_shot_scenario = stage_scenario(
        one_shot.path(),
        &one_shot_csv,
        &one_shot_parquet,
        &one_shot_summary,
    );
    let one_shot_report =
        footprint_mc::run(&one_shot_scenario, None, None, "host-clean", None).expect("one shot");
    assert!(one_shot_report.complete);

    let resumed_csv = resumed.path().join("samples.csv");
    let resumed_parquet = resumed.path().join("samples.parquet");
    let resumed_summary = resumed.path().join("summary.toml");
    let resumed_checkpoint = resumed.path().join("checkpoint.json");
    let resumed_scenario = stage_scenario(
        resumed.path(),
        &resumed_csv,
        &resumed_parquet,
        &resumed_summary,
    );

    let partial = footprint_mc::run(
        &resumed_scenario,
        Some(&resumed_checkpoint),
        Some(11),
        "host-clean",
        None,
    )
    .expect("partial resume");
    assert!(!partial.complete);
    assert_eq!(partial.samples_completed, 11);
    assert_eq!(partial.written, vec![resumed_checkpoint.clone()]);
    assert!(!resumed_csv.exists());
    assert!(!resumed_summary.exists());

    let complete = footprint_mc::run(
        &resumed_scenario,
        Some(&resumed_checkpoint),
        Some(64),
        "host-clean",
        None,
    )
    .expect("complete resume");
    assert!(complete.complete);
    assert_eq!(complete.samples_completed, 32);
    assert_eq!(
        fs::read_to_string(&resumed_csv).expect("read resumed csv"),
        fs::read_to_string(&one_shot_csv).expect("read one-shot csv"),
    );
    assert_eq!(
        fs::read_to_string(&resumed_summary).expect("read resumed summary"),
        fs::read_to_string(&one_shot_summary).expect("read one-shot summary"),
    );
}

#[test]
fn footprint_mc_attaches_uq_credibility_report_and_floor() {
    let temp = Builder::new()
        .prefix("openbmp_footprint_mc_uq")
        .tempdir()
        .expect("tempdir");
    let csv = temp.path().join("samples.csv");
    let parquet = temp.path().join("samples.parquet");
    let summary = temp.path().join("summary.toml");
    let uq = temp.path().join("uq.toml");
    let report_md = temp.path().join("reports").join("credibility.md");
    fs::write(&uq, uq_budget_toml(2)).expect("write uq");
    let scenario = stage_scenario(temp.path(), &csv, &parquet, &summary);

    let report = footprint_mc::run(
        &scenario,
        None,
        None,
        "host-clean",
        Some(mc::McCredibilityOptions {
            uq_toml: &uq,
            floor: CredibilityLevel::L2,
            report_md: Some(&report_md),
        }),
    )
    .expect("footprint mc with uq");

    let credibility = report.credibility.expect("credibility report");
    assert_eq!(credibility.binding_level, CredibilityLevel::L2);
    assert!(credibility.accepted);
    assert_eq!(
        report.credibility_report_md.as_deref(),
        Some(report_md.as_path())
    );
    assert!(report.written.contains(&report_md));
    let markdown = fs::read_to_string(&report_md).expect("read markdown");
    assert!(markdown.contains("# Monte Carlo credibility report"));
    assert!(markdown.contains("Binding credibility:** L2"));
}

#[test]
fn footprint_mc_uses_scenario_declared_uq_credibility_report() {
    let temp = Builder::new()
        .prefix("openbmp_footprint_mc_manifest_uq")
        .tempdir()
        .expect("tempdir");
    let csv = temp.path().join("samples.csv");
    let parquet = temp.path().join("samples.parquet");
    let summary = temp.path().join("summary.toml");
    let uq = temp.path().join("uq").join("budget.toml");
    let report_md = temp.path().join("reports").join("credibility.md");
    fs::create_dir_all(uq.parent().expect("uq parent")).expect("uq dir");
    fs::write(&uq, uq_budget_toml(2)).expect("write uq");
    let scenario = stage_scenario(temp.path(), &csv, &parquet, &summary);
    append_uq_config(&scenario, &uq, CredibilityLevel::L2, Some(&report_md));

    let report = footprint_mc::run(&scenario, None, None, "host-clean", None)
        .expect("footprint mc with scenario uq");

    let credibility = report.credibility.expect("credibility report");
    assert_eq!(credibility.binding_level, CredibilityLevel::L2);
    assert!(credibility.accepted);
    assert_eq!(
        report.credibility_report_md.as_deref(),
        Some(report_md.as_path())
    );
    assert!(report.written.contains(&report_md));
    assert!(csv.exists());
    assert!(summary.exists());
    let markdown = fs::read_to_string(&report_md).expect("read markdown");
    assert!(markdown.contains("Binding credibility:** L2"));
}

#[test]
fn footprint_mc_refuses_uq_below_credibility_floor() {
    let temp = Builder::new()
        .prefix("openbmp_footprint_mc_uq_floor")
        .tempdir()
        .expect("tempdir");
    let csv = temp.path().join("samples.csv");
    let parquet = temp.path().join("samples.parquet");
    let summary = temp.path().join("summary.toml");
    let uq = temp.path().join("uq.toml");
    let report_md = temp.path().join("reports").join("credibility.md");
    fs::write(&uq, uq_budget_toml(2)).expect("write uq");
    let scenario = stage_scenario(temp.path(), &csv, &parquet, &summary);

    let err = footprint_mc::run(
        &scenario,
        None,
        None,
        "host-clean",
        Some(mc::McCredibilityOptions {
            uq_toml: &uq,
            floor: CredibilityLevel::L3,
            report_md: Some(&report_md),
        }),
    )
    .expect_err("floor should fail");

    assert!(err.to_string().contains("below requested floor L3"));
    assert!(report_md.exists());
    assert!(!csv.exists());
    assert!(!summary.exists());
}

#[test]
fn footprint_mc_refuses_scenario_uq_below_credibility_floor() {
    let temp = Builder::new()
        .prefix("openbmp_footprint_mc_manifest_uq_floor")
        .tempdir()
        .expect("tempdir");
    let csv = temp.path().join("samples.csv");
    let parquet = temp.path().join("samples.parquet");
    let summary = temp.path().join("summary.toml");
    let uq = temp.path().join("uq").join("budget.toml");
    let report_md = temp.path().join("reports").join("credibility.md");
    fs::create_dir_all(uq.parent().expect("uq parent")).expect("uq dir");
    fs::write(&uq, uq_budget_toml(2)).expect("write uq");
    let scenario = stage_scenario(temp.path(), &csv, &parquet, &summary);
    append_uq_config(&scenario, &uq, CredibilityLevel::L3, Some(&report_md));

    let err = footprint_mc::run(&scenario, None, None, "host-clean", None)
        .expect_err("floor should fail");

    assert!(err.to_string().contains("below requested floor L3"));
    assert!(report_md.exists());
    assert!(!csv.exists());
    assert!(!summary.exists());
}

fn stage_scenario(dir: &Path, csv: &Path, parquet: &Path, summary: &Path) -> PathBuf {
    let original = fs::read_to_string(fixture_path()).expect("read fixture");
    let rewritten = rewrite_output_paths(&original, csv, parquet, summary);
    let scenario = dir.join("scenario.toml");
    fs::write(&scenario, rewritten).expect("write staged scenario");
    scenario
}

fn rewrite_output_paths(original: &str, csv: &Path, parquet: &Path, summary: &Path) -> String {
    original
        .replace(
            r#"samples_csv = "out/coast-footprint-mc-samples.csv""#,
            &format!("samples_csv = {}", toml_literal_path(csv)),
        )
        .replace(
            r#"samples_parquet = "out/coast-footprint-mc-samples.parquet""#,
            &format!("samples_parquet = {}", toml_literal_path(parquet)),
        )
        .replace(
            r#"summary_toml = "out/coast-footprint-mc-summary.toml""#,
            &format!("summary_toml = {}", toml_literal_path(summary)),
        )
}

fn append_uq_config(scenario: &Path, uq: &Path, floor: CredibilityLevel, report_md: Option<&Path>) {
    let mut text = fs::read_to_string(scenario).expect("read staged scenario");
    text.push_str("\n[landing_footprint.monte_carlo.uq]\n");
    text.push_str(&format!("budget_toml = {}\n", toml_literal_path(uq)));
    text.push_str(&format!("credibility_floor = \"{}\"\n", floor.as_label()));
    if let Some(report_md) = report_md {
        text.push_str(&format!("report_md = {}\n", toml_literal_path(report_md)));
    }
    fs::write(scenario, text).expect("write scenario uq");
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
