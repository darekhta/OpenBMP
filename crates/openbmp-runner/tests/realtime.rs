//! Regression coverage for opt-in host realtime pacing.

#![allow(clippy::expect_used, clippy::panic)]

use openbmp_scenario::Scenario;

const BASE_SCENARIO: &str = include_str!("fixtures/realtime-runner-regression.toml");

#[test]
fn realtime_pacing_modes_preserve_telemetry_bytes() {
    let baseline = run_csv(BASE_SCENARIO);
    let free_run = run_csv(&format!(
        "{BASE_SCENARIO}\n[realtime]\nmode = \"free_run\"\njitter_budget_s = 0.001\n"
    ));
    let paced = run_csv(&format!(
        "{BASE_SCENARIO}\n[realtime]\nmode = \"paced\"\ntarget_rtf = 1000.0\njitter_budget_s = 0.001\n"
    ));
    let real_time = run_csv(&format!(
        "{BASE_SCENARIO}\n[realtime]\nmode = \"real_time\"\njitter_budget_s = 0.001\n"
    ));

    assert_eq!(baseline, free_run);
    assert_eq!(baseline, paced);
    assert_eq!(baseline, real_time);
}

#[test]
fn realtime_run_reports_jitter_and_overruns_outside_telemetry() {
    let scenario = Scenario::from_toml_str(&format!(
        "{BASE_SCENARIO}\n[realtime]\nmode = \"paced\"\ntarget_rtf = 1000.0\njitter_budget_s = 0.001\n"
    ))
    .expect("scenario parses");
    let outcome = openbmp_runner::run(&scenario).expect("scenario runs");
    let realtime = outcome.realtime.expect("realtime report");
    assert_eq!(realtime.mode, "paced");
    assert_eq!(
        realtime.target_rtf.map(f64::to_bits),
        Some(1000.0_f64.to_bits())
    );
    assert_eq!(realtime.frame_count, outcome.final_step);
    assert!(realtime.jitter.is_some());
    assert!(realtime.frame_execution.is_some());
    assert!(realtime.schedulability.is_none());
}

#[test]
fn realtime_run_reports_frame_execution_budget_evidence() {
    let scenario = Scenario::from_toml_str(&format!(
        "{BASE_SCENARIO}\n[realtime]\nmode = \"paced\"\ntarget_rtf = 1.0\njitter_budget_s = 0.001\n"
    ))
    .expect("scenario parses");
    let outcome = openbmp_runner::run(&scenario).expect("scenario runs");
    let execution = outcome
        .realtime
        .expect("realtime report")
        .frame_execution
        .expect("frame execution report");
    assert_eq!(
        execution.count,
        usize::try_from(outcome.final_step).expect("step count fits usize")
    );
    assert_eq!(execution.wall_budget_ns, Some(1_000_000));
    assert!(execution.max_ns >= execution.p50_ns);
    assert!(execution.max_ns >= execution.p99_ns);
    assert!(execution.max_ns >= execution.p999_ns);
    assert!(execution.over_budget_count <= outcome.final_step);
}

#[test]
fn realtime_schedulability_tasks_reported_by_runner() {
    let scenario = Scenario::from_toml_str(&format!(
        "{BASE_SCENARIO}\n[realtime]\nmode = \"paced\"\ntarget_rtf = 1000.0\njitter_budget_s = 0.001\n\n[[realtime.task]]\nlabel = \"estimator\"\nperiod_s = 0.01\nwcet_s = 0.001\n\n[[realtime.task]]\nlabel = \"autopilot\"\nperiod_s = 0.02\nwcet_s = 0.002\n"
    ))
    .expect("scenario parses");
    let outcome = openbmp_runner::run(&scenario).expect("scenario runs");
    let schedulability = outcome
        .realtime
        .expect("realtime report")
        .schedulability
        .expect("schedulability report");
    assert_eq!(schedulability.task_count, 2);
    assert!(schedulability.schedulable);
    assert!((schedulability.utilization - 0.2).abs() < f64::EPSILON);
    assert!(schedulability.liu_layland_bound > schedulability.utilization);
}

fn run_csv(toml: &str) -> Vec<u8> {
    let scenario = Scenario::from_toml_str(toml).expect("scenario parses");
    let outcome = openbmp_runner::run(&scenario).expect("scenario runs");
    let mut csv = Vec::new();
    outcome
        .table
        .write_csv(&mut csv)
        .expect("telemetry writes to CSV");
    csv
}
