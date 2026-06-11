//! Host-side soft real-time frame pacing and jitter accounting.
//!
//! `openbmp-rt` is an L7 orchestration crate. It paces fixed-step host
//! loops, records release jitter, counts budget overruns, and reports a
//! real-time factor. It does not own simulation state, guidance logic,
//! actuator commands, or flight-controller scheduling, so changing the
//! pacing mode can affect when a frame is released but not what the
//! deterministic simulation computes.
//!
//! The crate depends only on `openbmp-core` and `std`. The flight
//! controller and HAL crates must not depend on this crate.

#![deny(unsafe_code)]

use std::error::Error;
use std::fmt;
use std::time::{Duration as StdDuration, Instant};

use openbmp_core::Duration as SimDuration;

const NANOS_PER_SECOND: i128 = 1_000_000_000;

/// Clock source used by [`FramePacer`].
///
/// Production code normally uses [`StdClock`]. Tests can inject a manual
/// clock to make pacing behavior deterministic and fast.
pub trait Clock: fmt::Debug {
    /// Returns nanoseconds elapsed since the clock's monotonic origin.
    fn now_ns(&self) -> i128;

    /// Blocks or advances the clock until `target_ns`.
    fn sleep_until_ns(&mut self, target_ns: i128);
}

/// Monotonic wall clock backed by [`Instant`].
#[derive(Clone, Debug)]
pub struct StdClock {
    origin: Instant,
}

impl StdClock {
    /// Creates a wall clock whose elapsed time starts at construction.
    #[must_use]
    pub fn new() -> Self {
        Self {
            origin: Instant::now(),
        }
    }
}

impl Default for StdClock {
    fn default() -> Self {
        Self::new()
    }
}

impl Clock for StdClock {
    fn now_ns(&self) -> i128 {
        let elapsed = self.origin.elapsed().as_nanos();
        i128::try_from(elapsed).unwrap_or(i128::MAX)
    }

    fn sleep_until_ns(&mut self, target_ns: i128) {
        let now_ns = self.now_ns();
        if target_ns <= now_ns {
            return;
        }
        std::thread::sleep(std_duration_from_ns(target_ns - now_ns));
    }
}

/// Pacing mode for a fixed-step host loop.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PaceMode {
    /// Do not sleep. Frame timing is observed only.
    FreeRun,
    /// Pace one simulation second per wall-clock second.
    RealTime,
    /// Pace to a requested simulation-seconds-per-wall-second factor.
    Paced {
        /// Target real-time factor. Values greater than one are
        /// faster-than-real-time.
        target_rtf: f64,
    },
}

/// Configuration for a [`FramePacer`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RtConfig {
    frame_period_ns: u64,
    jitter_budget_ns: u64,
    mode: PaceMode,
}

impl RtConfig {
    /// Creates a config from `std` durations.
    ///
    /// # Errors
    ///
    /// Returns [`RtError::NonPositiveFramePeriod`] if the frame period is
    /// zero, [`RtError::DurationTooLarge`] if a duration does not fit the
    /// internal nanosecond representation, and
    /// [`RtError::InvalidTargetRealTimeFactor`] for invalid paced modes.
    pub fn new(
        frame_period: StdDuration,
        jitter_budget: StdDuration,
        mode: PaceMode,
    ) -> Result<Self, RtError> {
        let frame_period_ns =
            u64::try_from(frame_period.as_nanos()).map_err(|_| RtError::DurationTooLarge {
                field: "frame_period",
            })?;
        let jitter_budget_ns =
            u64::try_from(jitter_budget.as_nanos()).map_err(|_| RtError::DurationTooLarge {
                field: "jitter_budget",
            })?;
        Self::from_nanos(frame_period_ns, jitter_budget_ns, mode)
    }

    /// Creates a config directly from nanoseconds.
    ///
    /// # Errors
    ///
    /// Returns [`RtError::NonPositiveFramePeriod`] if `frame_period_ns` is
    /// zero and [`RtError::InvalidTargetRealTimeFactor`] for invalid paced
    /// modes.
    pub fn from_nanos(
        frame_period_ns: u64,
        jitter_budget_ns: u64,
        mode: PaceMode,
    ) -> Result<Self, RtError> {
        if frame_period_ns == 0 {
            return Err(RtError::NonPositiveFramePeriod);
        }
        validate_mode(mode)?;
        Ok(Self {
            frame_period_ns,
            jitter_budget_ns,
            mode,
        })
    }

    /// Returns the configured minor-frame period in nanoseconds.
    #[must_use]
    pub const fn frame_period_ns(self) -> u64 {
        self.frame_period_ns
    }

    /// Returns the configured jitter budget in nanoseconds.
    #[must_use]
    pub const fn jitter_budget_ns(self) -> u64 {
        self.jitter_budget_ns
    }

    /// Returns the configured pacing mode.
    #[must_use]
    pub const fn mode(self) -> PaceMode {
        self.mode
    }

    /// Returns the minor-frame period on the OpenBMP simulation-time axis.
    #[must_use]
    pub fn frame_period(self) -> SimDuration {
        SimDuration::from_seconds(self.frame_period_ns as f64 / NANOS_PER_SECOND as f64)
    }

    /// Returns the jitter budget on the OpenBMP duration type.
    #[must_use]
    pub fn jitter_budget(self) -> SimDuration {
        SimDuration::from_seconds(self.jitter_budget_ns as f64 / NANOS_PER_SECOND as f64)
    }
}

/// Error returned by real-time pacing helpers.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RtError {
    /// The configured frame period was zero.
    NonPositiveFramePeriod,
    /// A `std` duration could not fit in the nanosecond representation.
    DurationTooLarge {
        /// Field that could not be converted.
        field: &'static str,
    },
    /// The paced real-time factor was zero, negative, NaN, or infinite.
    InvalidTargetRealTimeFactor {
        /// Rejected target real-time factor.
        target_rtf: f64,
    },
    /// Schedulability analysis was requested without any tasks.
    EmptyTaskSet,
    /// A schedulability task had an invalid period.
    InvalidTaskPeriod {
        /// Index in the input task list.
        index: usize,
    },
}

impl fmt::Display for RtError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonPositiveFramePeriod => write!(f, "frame period must be positive"),
            Self::DurationTooLarge { field } => write!(f, "{field} duration is too large"),
            Self::InvalidTargetRealTimeFactor { target_rtf } => {
                write!(f, "invalid target real-time factor {target_rtf}")
            }
            Self::EmptyTaskSet => write!(f, "schedulability task set is empty"),
            Self::InvalidTaskPeriod { index } => {
                write!(f, "schedulability task {index} has a zero period")
            }
        }
    }
}

impl Error for RtError {}

/// Timing observation for one released frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FrameTiming {
    /// One-based frame index. The first wait releases frame 1 at `t0 + h`.
    pub frame_index: u64,
    /// Scheduled release time relative to pacer start, in nanoseconds.
    pub scheduled_elapsed_ns: i128,
    /// Observed release time relative to pacer start, in nanoseconds.
    pub release_elapsed_ns: i128,
    /// Signed release jitter in nanoseconds: `release - scheduled`.
    pub jitter_ns: i64,
    /// Whether `jitter_ns` exceeded the configured positive jitter budget.
    pub overrun: bool,
    /// Real-time factor after this release, if wall elapsed time is positive.
    pub real_time_factor: Option<f64>,
}

/// Fixed-step frame pacer with jitter accounting.
#[derive(Debug)]
pub struct FramePacer<C: Clock> {
    clock: C,
    config: RtConfig,
    start_ns: i128,
    released_frames: u64,
    last_release_elapsed_ns: Option<i128>,
    jitter: JitterHistogram,
    overrun_count: u64,
}

impl<C: Clock> FramePacer<C> {
    /// Creates a pacer using the clock's current time as `t0`.
    #[must_use]
    pub fn new(clock: C, config: RtConfig) -> Self {
        let start_ns = clock.now_ns();
        Self {
            clock,
            config,
            start_ns,
            released_frames: 0,
            last_release_elapsed_ns: None,
            jitter: JitterHistogram::default(),
            overrun_count: 0,
        }
    }

    /// Creates a pacer with pre-allocated jitter sample capacity.
    #[must_use]
    pub fn with_jitter_capacity(clock: C, config: RtConfig, capacity: usize) -> Self {
        let start_ns = clock.now_ns();
        Self {
            clock,
            config,
            start_ns,
            released_frames: 0,
            last_release_elapsed_ns: None,
            jitter: JitterHistogram::with_capacity(capacity),
            overrun_count: 0,
        }
    }

    /// Returns the immutable clock reference.
    #[must_use]
    pub const fn clock(&self) -> &C {
        &self.clock
    }

    /// Returns the mutable clock reference.
    pub fn clock_mut(&mut self) -> &mut C {
        &mut self.clock
    }

    /// Returns the pacer configuration.
    #[must_use]
    pub const fn config(&self) -> RtConfig {
        self.config
    }

    /// Returns the number of frames released so far.
    #[must_use]
    pub const fn released_frames(&self) -> u64 {
        self.released_frames
    }

    /// Waits for the next absolute minor-frame boundary and records timing.
    ///
    /// Targets are absolute: frame `k` is scheduled at `t0 + k * h_c` for
    /// [`PaceMode::RealTime`], and at `t0 + k * h_c / target_rtf` for
    /// [`PaceMode::Paced`]. [`PaceMode::FreeRun`] skips sleeping but records
    /// the same ideal target for observability.
    pub fn wait_next_frame(&mut self) -> FrameTiming {
        let frame_index = self.released_frames.saturating_add(1);
        let scheduled_elapsed_ns = self.scheduled_elapsed_ns(frame_index);
        let target_ns = self.start_ns.saturating_add(scheduled_elapsed_ns);

        if !matches!(self.config.mode, PaceMode::FreeRun) {
            self.clock.sleep_until_ns(target_ns);
        }

        let release_ns = self.clock.now_ns();
        let release_elapsed_ns = release_ns.saturating_sub(self.start_ns);
        let jitter_ns = clamp_i128_to_i64(release_ns.saturating_sub(target_ns));
        let overrun = jitter_ns > u64_to_i64_saturating(self.config.jitter_budget_ns);
        if overrun {
            self.overrun_count = self.overrun_count.saturating_add(1);
        }

        self.released_frames = frame_index;
        self.last_release_elapsed_ns = Some(release_elapsed_ns);
        self.jitter.record(jitter_ns);

        FrameTiming {
            frame_index,
            scheduled_elapsed_ns,
            release_elapsed_ns,
            jitter_ns,
            overrun,
            real_time_factor: self.real_time_factor(),
        }
    }

    /// Returns the observed jitter histogram.
    #[must_use]
    pub const fn jitter_histogram(&self) -> &JitterHistogram {
        &self.jitter
    }

    /// Returns the count of releases later than the jitter budget.
    #[must_use]
    pub const fn overrun_count(&self) -> u64 {
        self.overrun_count
    }

    /// Returns the latest observed real-time factor.
    #[must_use]
    pub fn real_time_factor(&self) -> Option<f64> {
        let wall_ns = self.last_release_elapsed_ns?;
        if wall_ns <= 0 {
            return None;
        }
        let sim_ns = self
            .released_frames
            .saturating_mul(self.config.frame_period_ns);
        Some(sim_ns as f64 / wall_ns as f64)
    }

    fn scheduled_elapsed_ns(&self, frame_index: u64) -> i128 {
        let sim_elapsed_ns =
            i128::from(self.config.frame_period_ns).saturating_mul(i128::from(frame_index));
        match self.config.mode {
            PaceMode::FreeRun | PaceMode::RealTime => sim_elapsed_ns,
            PaceMode::Paced { target_rtf } => {
                let scaled = (sim_elapsed_ns as f64 / target_rtf).round();
                if scaled >= i128::MAX as f64 {
                    i128::MAX
                } else if scaled <= i128::MIN as f64 {
                    i128::MIN
                } else {
                    scaled as i128
                }
            }
        }
    }
}

/// Signed jitter sample store and percentile helper.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct JitterHistogram {
    samples_ns: Vec<i64>,
}

impl JitterHistogram {
    /// Creates an empty histogram.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            samples_ns: Vec::new(),
        }
    }

    /// Creates an empty histogram with capacity for expected samples.
    #[must_use]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            samples_ns: Vec::with_capacity(capacity),
        }
    }

    /// Records one signed jitter sample in nanoseconds.
    pub fn record(&mut self, jitter_ns: i64) {
        self.samples_ns.push(jitter_ns);
    }

    /// Returns the number of recorded samples.
    #[must_use]
    pub fn len(&self) -> usize {
        self.samples_ns.len()
    }

    /// Returns `true` when no samples have been recorded.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.samples_ns.is_empty()
    }

    /// Returns the raw signed jitter samples in insertion order.
    #[must_use]
    pub fn samples_ns(&self) -> &[i64] {
        &self.samples_ns
    }

    /// Returns signed percentile and max-jitter summary statistics.
    #[must_use]
    pub fn summary(&self) -> Option<JitterSummary> {
        if self.samples_ns.is_empty() {
            return None;
        }
        let mut sorted = self.samples_ns.clone();
        sorted.sort_unstable();
        let p50_ns = percentile_nearest_rank(&sorted, 50, 100);
        let p99_ns = percentile_nearest_rank(&sorted, 99, 100);
        let p999_ns = percentile_nearest_rank(&sorted, 999, 1000);
        let min_ns = sorted[0];
        let max_ns = sorted[sorted.len() - 1];
        let max_abs_ns = sorted
            .iter()
            .map(|sample| sample.unsigned_abs())
            .max()
            .unwrap_or(0);
        Some(JitterSummary {
            count: sorted.len(),
            p50_ns,
            p99_ns,
            p999_ns,
            min_ns,
            max_ns,
            max_abs_ns,
        })
    }
}

/// Percentile and max-jitter summary.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct JitterSummary {
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

/// One periodic task for Liu-Layland rate-monotonic analysis.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SchedulabilityTask {
    /// Task period in nanoseconds.
    pub period_ns: u64,
    /// Worst-case execution time in nanoseconds.
    pub wcet_ns: u64,
}

impl SchedulabilityTask {
    /// Creates a task from nanosecond values.
    #[must_use]
    pub const fn from_nanos(period_ns: u64, wcet_ns: u64) -> Self {
        Self { period_ns, wcet_ns }
    }

    /// Creates a task from `std` durations.
    ///
    /// # Errors
    ///
    /// Returns [`RtError::DurationTooLarge`] if either duration does not
    /// fit the internal nanosecond representation.
    pub fn new(period: StdDuration, wcet: StdDuration) -> Result<Self, RtError> {
        let period_ns =
            u64::try_from(period.as_nanos()).map_err(|_| RtError::DurationTooLarge {
                field: "task_period",
            })?;
        let wcet_ns = u64::try_from(wcet.as_nanos())
            .map_err(|_| RtError::DurationTooLarge { field: "task_wcet" })?;
        Ok(Self { period_ns, wcet_ns })
    }
}

/// Liu-Layland rate-monotonic schedulability report.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SchedulabilityReport {
    /// Number of periodic tasks analyzed.
    pub task_count: usize,
    /// Sum of `wcet / period` across tasks.
    pub utilization: f64,
    /// Liu-Layland sufficient utilization bound.
    pub liu_layland_bound: f64,
    /// Whether the utilization is within the sufficient bound.
    pub schedulable: bool,
}

/// Returns the Liu-Layland utilization bound for `task_count` tasks.
#[must_use]
pub fn liu_layland_bound(task_count: usize) -> Option<f64> {
    if task_count == 0 {
        return None;
    }
    let n = task_count as f64;
    Some(n * (2.0_f64.powf(1.0 / n) - 1.0))
}

/// Computes a rate-monotonic schedulability advisory for periodic tasks.
///
/// This is a sufficient analytic advisory, not a hard real-time proof.
///
/// # Errors
///
/// Returns [`RtError::EmptyTaskSet`] for an empty task set or
/// [`RtError::InvalidTaskPeriod`] for a task with a zero period.
pub fn schedulability_advisory(
    tasks: &[SchedulabilityTask],
) -> Result<SchedulabilityReport, RtError> {
    if tasks.is_empty() {
        return Err(RtError::EmptyTaskSet);
    }
    let mut utilization = 0.0;
    for (index, task) in tasks.iter().enumerate() {
        if task.period_ns == 0 {
            return Err(RtError::InvalidTaskPeriod { index });
        }
        utilization += task.wcet_ns as f64 / task.period_ns as f64;
    }
    let bound = liu_layland_bound(tasks.len()).ok_or(RtError::EmptyTaskSet)?;
    Ok(SchedulabilityReport {
        task_count: tasks.len(),
        utilization,
        liu_layland_bound: bound,
        schedulable: utilization <= bound,
    })
}

fn validate_mode(mode: PaceMode) -> Result<(), RtError> {
    match mode {
        PaceMode::FreeRun | PaceMode::RealTime => Ok(()),
        PaceMode::Paced { target_rtf } if target_rtf.is_finite() && target_rtf > 0.0 => Ok(()),
        PaceMode::Paced { target_rtf } => Err(RtError::InvalidTargetRealTimeFactor { target_rtf }),
    }
}

fn percentile_nearest_rank(sorted: &[i64], numerator: usize, denominator: usize) -> i64 {
    let len = sorted.len();
    let rank = len.saturating_mul(numerator).div_ceil(denominator);
    let index = rank.saturating_sub(1).min(len - 1);
    sorted[index]
}

fn clamp_i128_to_i64(value: i128) -> i64 {
    if value > i128::from(i64::MAX) {
        i64::MAX
    } else if value < i128::from(i64::MIN) {
        i64::MIN
    } else {
        value as i64
    }
}

fn u64_to_i64_saturating(value: u64) -> i64 {
    i64::try_from(value).unwrap_or(i64::MAX)
}

fn std_duration_from_ns(ns: i128) -> StdDuration {
    if ns <= 0 {
        return StdDuration::ZERO;
    }
    let seconds = ns / NANOS_PER_SECOND;
    let subsecond_ns = (ns % NANOS_PER_SECOND) as u32;
    if seconds > i128::from(u64::MAX) {
        StdDuration::new(u64::MAX, 999_999_999)
    } else {
        StdDuration::new(seconds as u64, subsecond_ns)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Clone, Debug)]
    struct ManualClock {
        now_ns: i128,
    }

    impl ManualClock {
        const fn new(now_ns: i128) -> Self {
            Self { now_ns }
        }

        fn advance_ns(&mut self, delta_ns: i128) {
            self.now_ns = self.now_ns.saturating_add(delta_ns);
        }
    }

    impl Clock for ManualClock {
        fn now_ns(&self) -> i128 {
            self.now_ns
        }

        fn sleep_until_ns(&mut self, target_ns: i128) {
            if target_ns > self.now_ns {
                self.now_ns = target_ns;
            }
        }
    }

    #[test]
    fn pacer_modes_do_not_change_deterministic_state() -> Result<(), RtError> {
        fn run(mode: PaceMode) -> Result<Vec<u64>, RtError> {
            let config = RtConfig::from_nanos(10_000_000, 1_000_000, mode)?;
            let mut pacer = FramePacer::with_jitter_capacity(ManualClock::new(0), config, 8);
            let mut state = 0x0123_4567_89ab_cdef_u64;
            let mut outputs = Vec::with_capacity(8);
            for step in 0..8_u64 {
                let _timing = pacer.wait_next_frame();
                state = state
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407)
                    .wrapping_add(step);
                outputs.push(state);
                pacer.clock_mut().advance_ns(1_000_000);
            }
            Ok(outputs)
        }

        let free = run(PaceMode::FreeRun)?;
        let realtime = run(PaceMode::RealTime)?;
        let paced = run(PaceMode::Paced { target_rtf: 2.0 })?;
        assert_eq!(free, realtime);
        assert_eq!(free, paced);
        Ok(())
    }

    #[test]
    fn pacer_records_jitter_and_overruns() -> Result<(), RtError> {
        let config = RtConfig::from_nanos(10_000_000, 1_000_000, PaceMode::RealTime)?;
        let mut pacer = FramePacer::new(ManualClock::new(0), config);

        let first = pacer.wait_next_frame();
        assert_eq!(first.frame_index, 1);
        assert_eq!(first.jitter_ns, 0);
        assert!(!first.overrun);

        pacer.clock_mut().advance_ns(13_000_001);
        let second = pacer.wait_next_frame();
        assert_eq!(second.frame_index, 2);
        assert_eq!(second.jitter_ns, 3_000_001);
        assert!(second.overrun);
        assert_eq!(pacer.overrun_count(), 1);

        if let Some(summary) = pacer.jitter_histogram().summary() {
            assert_eq!(summary.count, 2);
            assert_eq!(summary.p50_ns, 0);
            assert_eq!(summary.p99_ns, 3_000_001);
            assert_eq!(summary.p999_ns, 3_000_001);
            assert_eq!(summary.max_ns, 3_000_001);
        } else {
            assert_eq!(pacer.jitter_histogram().len(), usize::MAX);
        }
        Ok(())
    }

    #[test]
    fn jitter_histogram_reports_signed_percentiles() {
        let mut histogram = JitterHistogram::new();
        for sample in [-5, 0, 2, 7, 10] {
            histogram.record(sample);
        }

        if let Some(summary) = histogram.summary() {
            assert_eq!(summary.count, 5);
            assert_eq!(summary.p50_ns, 2);
            assert_eq!(summary.p99_ns, 10);
            assert_eq!(summary.p999_ns, 10);
            assert_eq!(summary.min_ns, -5);
            assert_eq!(summary.max_ns, 10);
            assert_eq!(summary.max_abs_ns, 10);
        } else {
            assert_eq!(histogram.len(), usize::MAX);
        }
    }

    #[test]
    fn schedulability_advisory_matches_hand_checked_sets() -> Result<(), RtError> {
        let tasks = [
            SchedulabilityTask::from_nanos(10_000_000, 3_000_000),
            SchedulabilityTask::from_nanos(20_000_000, 4_000_000),
        ];
        let report = schedulability_advisory(&tasks)?;
        assert_eq!(report.task_count, 2);
        assert!((report.utilization - 0.5).abs() < 1.0e-12);
        assert!((report.liu_layland_bound - 0.828_427_124_746_190_3).abs() < 1.0e-12);
        assert!(report.schedulable);

        let overloaded = [
            SchedulabilityTask::from_nanos(10_000_000, 8_000_000),
            SchedulabilityTask::from_nanos(20_000_000, 8_000_000),
        ];
        let report = schedulability_advisory(&overloaded)?;
        assert!(!report.schedulable);
        Ok(())
    }

    #[test]
    fn invalid_config_rejects_bad_inputs() {
        assert_eq!(
            RtConfig::from_nanos(0, 1, PaceMode::RealTime),
            Err(RtError::NonPositiveFramePeriod)
        );
        assert!(matches!(
            RtConfig::from_nanos(1, 1, PaceMode::Paced { target_rtf: 0.0 }),
            Err(RtError::InvalidTargetRealTimeFactor { .. })
        ));
    }

    #[test]
    fn openbmp_fc_manifest_does_not_depend_on_openbmp_rt() {
        let fc_manifest = include_str!("../../openbmp-fc/Cargo.toml");
        assert!(!fc_manifest.contains("openbmp-rt"));
    }
}
