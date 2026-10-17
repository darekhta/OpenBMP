//! CLI smoke tests for `openbmp campaign`.

#![allow(clippy::unwrap_used)]

use std::fs;

use assert_cmd::Command;
use openbmp_cli::commands::campaign::CampaignRunCliReport;
use openbmp_rundata::campaign::{CampaignPlan, CampaignShard};
use predicates::prelude::*;
use tempfile::Builder;

const MANIFEST: &str = r#"
campaign_id = "cli-e2e"
seeds = [3, 1]
overlays = ["nominal", "windy"]

[[scenarios]]
id = "drop"
path = "scenarios/drop.toml"
"#;

const VERDICT_RULES: &str = r#"
[[rules]]
kind = "limit"
id = "position-final"
channel = "position_z_m"
metric = "final"
min = 500.0
max = 520.0
"#;

const FAILING_VERDICT_RULES: &str = r#"
[[rules]]
kind = "limit"
id = "position-final"
channel = "position_z_m"
metric = "final"
min = 900.0
"#;

const TREND_SPECS: &str = r#"
[[metrics]]
id = "drop-position-final"
family_id = "drop"
channel = "position_z_m"
metric = "final"
fail_on_drift = true

[metrics.detector]
kind = "page_hinkley"
warmup = 1
delta = 0.0
lambda = 1000.0
alpha = 1.0
direction = "upward"
"#;

#[test]
fn campaign_plan_cli_writes_plan_and_shards() {
    let temp = Builder::new()
        .prefix("openbmp_campaign_cli")
        .tempdir()
        .unwrap();
    let manifest = temp.path().join("campaign.toml");
    let plan_json = temp.path().join("planned/plan.json");
    let shards_json = temp.path().join("planned/shards.json");
    fs::write(&manifest, MANIFEST).unwrap();

    let mut cmd = Command::cargo_bin("openbmp").unwrap();
    cmd.arg("campaign")
        .arg("plan")
        .arg(&manifest)
        .arg("--output-plan-json")
        .arg(&plan_json)
        .arg("--output-shards-json")
        .arg(&shards_json)
        .arg("--workers")
        .arg("2")
        .assert()
        .success()
        .stdout(predicate::str::contains("openbmp campaign plan: ok"))
        .stdout(predicate::str::contains("runs=4"));

    let plan = CampaignPlan::from_json_bytes(&fs::read(plan_json).unwrap()).unwrap();
    let shards: Vec<CampaignShard> =
        serde_json::from_slice(&fs::read(shards_json).unwrap()).unwrap();
    let mut planned = plan
        .runs
        .iter()
        .map(|run| run.run_key.clone())
        .collect::<Vec<_>>();
    let mut sharded = shards
        .into_iter()
        .flat_map(|shard| shard.runs)
        .map(|run| run.run_key)
        .collect::<Vec<_>>();
    planned.sort();
    sharded.sort();
    assert_eq!(planned, sharded);
}

#[test]
fn campaign_run_cli_executes_manifest() {
    let temp = Builder::new()
        .prefix("openbmp_campaign_run_cli")
        .tempdir()
        .unwrap();
    let workspace = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .unwrap();
    let scenario = workspace.join("scenarios/analytic-toy/constant-acceleration-drop.toml");
    let verdict = temp.path().join("verdict.toml");
    let trend = temp.path().join("trend.toml");
    let manifest = temp.path().join("campaign.toml");
    let output = temp.path().join("out");
    fs::write(&verdict, VERDICT_RULES).unwrap();
    fs::write(&trend, TREND_SPECS).unwrap();
    fs::write(
        &manifest,
        format!(
            "campaign_id = \"cli-run-e2e\"\nseeds = [1]\nverdict_rules = '{}'\ntrend_specs = '{}'\n\n[[scenarios]]\nid = \"drop\"\npath = '{}'\n",
            verdict.display(),
            trend.display(),
            scenario.display(),
        ),
    )
    .unwrap();

    let mut cmd = Command::cargo_bin("openbmp").unwrap();
    cmd.arg("campaign")
        .arg("run")
        .arg(&manifest)
        .arg("--output-dir")
        .arg(&output)
        .arg("--run-code-id")
        .arg("campaign-cli-test")
        .arg("--code-ordinal")
        .arg("3")
        .arg("--write-index-parquet")
        .arg("--workers")
        .arg("2")
        .assert()
        .success()
        .stdout(predicate::str::contains("openbmp campaign run: ok"))
        .stdout(predicate::str::contains("runs=1"))
        .stdout(predicate::str::contains("verdict summary: status=Pass"))
        .stdout(predicate::str::contains("trend summary:"));

    assert!(output.join("summary.json").exists());
    assert!(output.join("run-index.jsonl").exists());
    assert!(output.join("run-index.parquet").exists());
    assert!(output.join("trend-store.json").exists());
    assert!(output.join("drift-report.json").exists());
    assert!(output.join("trend-scoreboard.md").exists());
}

#[test]
fn campaign_run_cli_fail_on_ci_gates_exits_after_writing_summary() {
    let temp = Builder::new()
        .prefix("openbmp_campaign_run_cli_ci_fail")
        .tempdir()
        .unwrap();
    let workspace = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .unwrap();
    let scenario = workspace.join("scenarios/analytic-toy/constant-acceleration-drop.toml");
    let verdict = temp.path().join("verdict.toml");
    let manifest = temp.path().join("campaign.toml");
    let output = temp.path().join("out");
    fs::write(&verdict, FAILING_VERDICT_RULES).unwrap();
    fs::write(
        &manifest,
        format!(
            "campaign_id = \"cli-run-ci-fail\"\nseeds = [1]\nverdict_rules = '{}'\n\n[[scenarios]]\nid = \"drop\"\npath = '{}'\n",
            verdict.display(),
            scenario.display(),
        ),
    )
    .unwrap();

    let mut cmd = Command::cargo_bin("openbmp").unwrap();
    cmd.arg("campaign")
        .arg("run")
        .arg(&manifest)
        .arg("--output-dir")
        .arg(&output)
        .arg("--workers")
        .arg("2")
        .arg("--fail-on-ci-gates")
        .assert()
        .failure()
        .stderr(predicate::str::contains("campaign CI gates failed"));

    let summary_json = output.join("summary.json");
    assert!(summary_json.exists());
    let summary: CampaignRunCliReport =
        serde_json::from_slice(&fs::read(summary_json).unwrap()).unwrap();
    assert_eq!(summary.ci_failures.len(), 1);
    assert!(summary.ci_failures[0].contains("campaign verdict summary failed"));
}
