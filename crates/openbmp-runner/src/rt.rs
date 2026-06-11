//! Runner adapter for the host-side `openbmp-rt` frame pacer.

use std::time::{Duration as StdDuration, Instant};

use openbmp_rt::{
    FramePacer, JitterSummary, PaceMode, RtConfig, SchedulabilityReport, SchedulabilityTask,
    StdClock, schedulability_advisory,
};
use openbmp_scenario::{RealtimeModeConfig, ScenarioDocument};

use crate::error::RunnerError;

/// Summary of host realtime pacing for a completed run.
#[derive(Clone, Debug, PartialEq)]
pub struct RealtimeRunReport {
    /// Pacing mode label.
    pub mode: &'static str,
    /// Target real-time factor when `mode = "paced"`.
    pub target_rtf: Option<f64>,
    /// Number of paced simulation frames.
    pub frame_count: u64,
    /// Number of frames whose late jitter exceeded the budget.
    pub overrun_count: u64,
    /// Last observed real-time factor.
    pub real_time_factor: Option<f64>,
    /// Jitter percentile summary, if at least one frame was released.
    pub jitter: Option<RealtimeJitterReport>,
    /// Host-side execution time summary for deterministic frame work.
    pub frame_execution: Option<RealtimeFrameExecutionReport>,
    /// Optional rate-monotonic schedulability advisory for declared tasks.
    pub schedulability: Option<RealtimeSchedulabilityReport>,
}

/// Signed jitter percentile summary for a completed realtime run.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RealtimeJitterReport {
    /// Number of jitter samples.
    pub count: usize,
    /// Signed nearest-rank p50 jitter in nanoseconds.
    pub p50_ns: i64,
    /// Signed nearest-rank p99 jitter in nanoseconds.
    pub p99_ns: i64,
    /// Signed nearest-rank p99.9 jitter in nanoseconds.
    pub p999_ns: i64,
    /// Most-negative signed jitter sample.
    pub min_ns: i64,
    /// Most-positive signed jitter sample.
    pub max_ns: i64,
    /// Largest absolute jitter magnitude in nanoseconds.
    pub max_abs_ns: u64,
}

impl From<JitterSummary> for RealtimeJitterReport {
    fn from(summary: JitterSummary) -> Self {
        Self {
            count: summary.count,
            p50_ns: summary.p50_ns,
            p99_ns: summary.p99_ns,
            p999_ns: summary.p999_ns,
            min_ns: summary.min_ns,
            max_ns: summary.max_ns,
            max_abs_ns: summary.max_abs_ns,
        }
    }
}

/// Host-side execution-time summary for deterministic frame work.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RealtimeFrameExecutionReport {
    /// Number of measured frames.
    pub count: usize,
    /// Optional wall-frame budget in nanoseconds.
    pub wall_budget_ns: Option<u64>,
    /// Number of measured frames whose execution exceeded `wall_budget_ns`.
    pub over_budget_count: u64,
    /// Nearest-rank p50 execution time in nanoseconds.
    pub p50_ns: u64,
    /// Nearest-rank p99 execution time in nanoseconds.
    pub p99_ns: u64,
    /// Nearest-rank p99.9 execution time in nanoseconds.
    pub p999_ns: u64,
    /// Maximum measured execution time in nanoseconds.
    pub max_ns: u64,
}

/// Liu-Layland rate-monotonic schedulability advisory for declared tasks.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RealtimeSchedulabilityReport {
    /// Number of periodic tasks analyzed.
    pub task_count: usize,
    /// Sum of `wcet / period` across tasks.
    pub utilization: f64,
    /// Liu-Layland sufficient utilization bound.
    pub liu_layland_bound: f64,
    /// Whether utilization is within the sufficient bound.
    pub schedulable: bool,
}

impl From<SchedulabilityReport> for RealtimeSchedulabilityReport {
    fn from(report: SchedulabilityReport) -> Self {
        Self {
            task_count: report.task_count,
            utilization: report.utilization,
            liu_layland_bound: report.liu_layland_bound,
            schedulable: report.schedulable,
        }
    }
}

#[derive(Debug)]
pub(crate) struct RunnerRealtimePacer {
    mode: &'static str,
    target_rtf: Option<f64>,
    pacer: Option<FramePacer<StdClock>>,
    frame_execution_start: Option<Instant>,
    frame_execution_samples_ns: Vec<u64>,
    wall_budget_ns: Option<u64>,
    over_budget_count: u64,
    schedulability: Option<RealtimeSchedulabilityReport>,
}

impl RunnerRealtimePacer {
    pub(crate) fn from_document(document: &ScenarioDocument) -> Result<Self, RunnerError> {
        let Some(config) = document.realtime.as_ref() else {
            return Ok(Self {
                mode: "disabled",
                target_rtf: None,
                pacer: None,
                frame_execution_start: None,
                frame_execution_samples_ns: Vec::new(),
                wall_budget_ns: None,
                over_budget_count: 0,
                schedulability: None,
            });
        };

        let frame_period = positive_duration("time.dt_s", document.time.dt_s)?;
        let jitter_budget = positive_duration("realtime.jitter_budget_s", config.jitter_budget_s)?;
        let (mode, target_rtf, label) = match config.mode {
            RealtimeModeConfig::FreeRun => (PaceMode::FreeRun, None, "free_run"),
            RealtimeModeConfig::RealTime => (PaceMode::RealTime, None, "real_time"),
            RealtimeModeConfig::Paced => {
                let Some(target_rtf) = config.target_rtf else {
                    return Err(realtime_error(
                        "realtime.target_rtf",
                        "mode = \"paced\" requires target_rtf",
                    ));
                };
                (PaceMode::Paced { target_rtf }, Some(target_rtf), "paced")
            }
        };
        let rt_config = RtConfig::new(frame_period, jitter_budget, mode)
            .map_err(|err| realtime_error("realtime", &err.to_string()))?;
        let wall_budget_ns = wall_frame_budget_ns(rt_config);
        let schedulability = schedulability_report(config)?;

        Ok(Self {
            mode: label,
            target_rtf,
            pacer: Some(FramePacer::new(StdClock::new(), rt_config)),
            frame_execution_start: None,
            frame_execution_samples_ns: Vec::new(),
            wall_budget_ns,
            over_budget_count: 0,
            schedulability,
        })
    }

    pub(crate) fn wait_next_frame(&mut self) {
        if let Some(pacer) = &mut self.pacer {
            let _timing = pacer.wait_next_frame();
        }
    }

    pub(crate) fn begin_frame_execution(&mut self) {
        if self.pacer.is_some() {
            self.frame_execution_start = Some(Instant::now());
        }
    }

    pub(crate) fn finish_frame_execution(&mut self) {
        if self.pacer.is_none() {
            return;
        }
        let Some(start) = self.frame_execution_start.take() else {
            return;
        };
        let elapsed_ns = duration_ns_saturating(start.elapsed());
        if let Some(wall_budget_ns) = self.wall_budget_ns
            && elapsed_ns > wall_budget_ns
        {
            self.over_budget_count = self.over_budget_count.saturating_add(1);
        }
        self.frame_execution_samples_ns.push(elapsed_ns);
    }

    pub(crate) fn finish(&self) -> Option<RealtimeRunReport> {
        let pacer = self.pacer.as_ref()?;
        Some(RealtimeRunReport {
            mode: self.mode,
            target_rtf: self.target_rtf,
            frame_count: pacer.released_frames(),
            overrun_count: pacer.overrun_count(),
            real_time_factor: pacer.real_time_factor(),
            jitter: pacer.jitter_histogram().summary().map(Into::into),
            frame_execution: frame_execution_report(
                &self.frame_execution_samples_ns,
                self.wall_budget_ns,
                self.over_budget_count,
            ),
            schedulability: self.schedulability,
        })
    }
}

fn wall_frame_budget_ns(config: RtConfig) -> Option<u64> {
    match config.mode() {
        PaceMode::FreeRun => None,
        PaceMode::RealTime => Some(config.frame_period_ns()),
        PaceMode::Paced { target_rtf } => {
            let scaled = (config.frame_period_ns() as f64 / target_rtf).ceil();
            if scaled <= 1.0 {
                Some(1)
            } else if scaled >= u64::MAX as f64 {
                Some(u64::MAX)
            } else {
                Some(scaled as u64)
            }
        }
    }
}

fn frame_execution_report(
    samples_ns: &[u64],
    wall_budget_ns: Option<u64>,
    over_budget_count: u64,
) -> Option<RealtimeFrameExecutionReport> {
    if samples_ns.is_empty() {
        return None;
    }
    let mut sorted = samples_ns.to_vec();
    sorted.sort_unstable();
    Some(RealtimeFrameExecutionReport {
        count: sorted.len(),
        wall_budget_ns,
        over_budget_count,
        p50_ns: percentile_nearest_rank_u64(&sorted, 50, 100),
        p99_ns: percentile_nearest_rank_u64(&sorted, 99, 100),
        p999_ns: percentile_nearest_rank_u64(&sorted, 999, 1000),
        max_ns: sorted[sorted.len() - 1],
    })
}

fn schedulability_report(
    config: &openbmp_scenario::RealtimeConfig,
) -> Result<Option<RealtimeSchedulabilityReport>, RunnerError> {
    if config.tasks.is_empty() {
        return Ok(None);
    }
    let mut tasks = Vec::with_capacity(config.tasks.len());
    for (index, task) in config.tasks.iter().enumerate() {
        let period = positive_duration(&format!("realtime.task[{index}].period_s"), task.period_s)?;
        let wcet = positive_duration(&format!("realtime.task[{index}].wcet_s"), task.wcet_s)?;
        let rt_task = SchedulabilityTask::new(period, wcet)
            .map_err(|err| realtime_error("realtime.task", &err.to_string()))?;
        tasks.push(rt_task);
    }
    schedulability_advisory(&tasks)
        .map(Into::into)
        .map(Some)
        .map_err(|err| realtime_error("realtime.task", &err.to_string()))
}

fn positive_duration(field: &str, seconds: f64) -> Result<StdDuration, RunnerError> {
    if !seconds.is_finite() || seconds <= 0.0 {
        return Err(realtime_error(
            field,
            "duration must be positive and finite",
        ));
    }
    Ok(StdDuration::from_secs_f64(seconds))
}

fn duration_ns_saturating(duration: StdDuration) -> u64 {
    u64::try_from(duration.as_nanos()).unwrap_or(u64::MAX)
}

fn percentile_nearest_rank_u64(sorted: &[u64], numerator: usize, denominator: usize) -> u64 {
    let len = sorted.len();
    let rank = len.saturating_mul(numerator).div_ceil(denominator);
    let index = rank.saturating_sub(1).min(len - 1);
    sorted[index]
}

fn realtime_error(field: &str, reason: &str) -> RunnerError {
    RunnerError::Realtime {
        field: field.to_owned(),
        reason: reason.to_owned(),
    }
}
