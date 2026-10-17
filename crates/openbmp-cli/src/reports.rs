//! Console presentation for package and campaign command reports.

use openbmp_cli::commands::campaign::{CampaignPlanCliReport, CampaignRunCliReport};
use openbmp_sil::{MaterializeSidecarsReport, PackageCheckReport};

pub(super) fn package_check(report: &PackageCheckReport) {
    println!(
        "openbmp package check: ok — {} {}",
        report.package_id, report.package_version,
    );
    println!("  scenario {}", report.scenario_path.display());
    for (key, digest) in &report.hashes {
        println!("  sha256 {key} {digest}");
    }
}

pub(super) fn package_sidecars(report: &MaterializeSidecarsReport) {
    println!(
        "openbmp package materialize-sidecars: ok — {} {}",
        report.package_id, report.package_version,
    );
    for sidecar in &report.sidecars {
        println!(
            "  wrote {} {} sha256 {}",
            sidecar.field,
            sidecar.path.display(),
            sidecar.sha256,
        );
    }
}

pub(super) fn campaign_plan(report: &CampaignPlanCliReport) {
    println!(
        "openbmp campaign plan: ok - campaign={}, runs={}, wrote {}",
        report.campaign_id,
        report.run_count,
        report.plan_json.display(),
    );
    if let Some(path) = &report.shards_json {
        let worker_count = report.worker_count.unwrap_or(0);
        println!("  wrote {} shard(s) to {}", worker_count, path.display());
    }
}

pub(super) fn campaign_run(report: &CampaignRunCliReport) {
    println!(
        "openbmp campaign run: ok - campaign={}, runs={}, wrote {}",
        report.campaign_id,
        report.run_count,
        report.summary_json.display(),
    );
    println!("  run index {}", report.run_index_jsonl.display());
    if let Some(path) = &report.run_index_parquet {
        println!("  run index parquet {}", path.display());
    }
    if let Some(summary) = &report.verdict_summary {
        println!(
            "  verdict summary: status={:?}, reports={}, failed={}",
            summary.status, summary.report_count, summary.failed_report_count,
        );
    }
    if let Some(trend) = &report.trend_summary {
        println!(
            "  trend summary: store={}, drift={}, scoreboard={}, ci_failures={}",
            trend.trend_store_json.display(),
            trend.drift_report_json.display(),
            trend.scoreboard_md.display(),
            trend.ci_failures.len(),
        );
    }
    if !report.ci_failures.is_empty() {
        println!("  campaign ci failures: {}", report.ci_failures.len());
    }
}
