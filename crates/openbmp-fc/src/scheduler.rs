//! Cyclic scheduler with declared-budget enforcement.
//!
//! Adapted from ArduPilot's `AP_Scheduler`: each registered job
//! declares `{period_ticks, budget_us, priority}`. The dispatcher
//! evaluates all jobs once per tick, runs the due ones in priority
//! order, and refuses to start a job whose budget will not fit before
//! the end of the frame. A skipped job emits an
//! [`OverrunEvent`] so the health monitor / commander can observe
//! sustained overruns.
//!
//! # Determinism
//!
//! - Job evaluation order is `(priority, registration_index)` —
//!   stable across runs. Two jobs with the same priority always
//!   evaluate in registration order.
//! - Period checks are integer modulo: a job with `period_ticks = 5`
//!   runs on ticks `0, 5, 10, 15, ...`.
//! - The scheduler does not allocate after registration; the per-tick
//!   loop reuses fixed-size storage.
//!
//! # Budget semantics
//!
//! Budgets are **declared**, not measured wall-time. The scheduler
//! sums declared budgets in priority order and refuses to start a job
//! whose budget would exceed the remaining frame budget. The runner
//! is free to additionally measure wall-time and feed that back via
//! [`Scheduler::report_actual_us`] for further FDIR diagnostics; the
//! kernel's lockstep contract still bans `std::time` inside the
//! controller crate.

use std::boxed::Box;
use std::vec::Vec;

use crate::bus::{Bus, Sequence, Topic};
use crate::clock::Clock;
use crate::error::{ControllerError, SchedulerError};
use crate::stable_map::StableIndexMap;
use crate::topics::topic_index;
use openbmp_msgs::TopicId;

/// Job priority — lower runs first.
pub type Priority = u8;

/// Maximum number of jobs accepted by the fixed dispatch table.
pub const MAX_SCHEDULED_JOBS: usize = 64;

/// Number of recent host timing samples retained per job.
pub const MAX_JOB_TIMING_SAMPLES: usize = 128;

/// Safety cap for static frame-packing enumeration.
pub const MAX_FRAME_PACKING_HYPERPERIOD_TICKS: u64 = 100_000;

/// Trigger condition for a registered job.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Trigger {
    /// Run every `period_ticks` ticks. `0` is rejected at registration.
    Periodic {
        /// Period in ticks.
        period_ticks: u64,
    },
    /// Run when the topic identified by `topic_id` has a sequence
    /// strictly greater than the value the job last consumed.
    /// Subscribers track their own `last_seen` sequence inside their
    /// state.
    TopicUpdated {
        /// Dense topic table id (`Topic::INDEX`).
        topic_id: TopicId,
        /// Canonical topic name (`Topic::NAME`).
        topic_name: &'static str,
    },
}

/// Per-tick context handed to a [`Job::run`] invocation.
pub struct JobContext<'a> {
    /// The internal bus.
    pub bus: &'a Bus,
    /// The injected clock.
    pub clock: &'a dyn Clock,
}

impl std::fmt::Debug for JobContext<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("JobContext")
            .field("now", &self.clock.now())
            .field("tick", &self.clock.tick())
            .finish()
    }
}

/// Trait implemented by every module that participates in cyclic
/// dispatch.
pub trait Job {
    /// Static name; appears in dictionaries and overrun events.
    fn name(&self) -> &'static str;

    /// Runs one job tick. The job should keep its run-time below its
    /// declared budget; if it persistently exceeds the budget the
    /// dispatcher will skip it on subsequent frames and emit
    /// [`OverrunEvent`]s.
    ///
    /// # Errors
    ///
    /// Any error returned aborts the frame and is propagated to the
    /// caller of [`Scheduler::dispatch`].
    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError>;
}

/// Optional runner-supplied timing hook.
///
/// The default implementation simply invokes [`Job::run`]. Host
/// runners can implement this trait with `std::time::Instant` without
/// importing wall-clock APIs into `openbmp-fc`.
pub trait JobTimingObserver {
    /// Runs a job and optionally returns measured wall-clock duration
    /// in microseconds.
    ///
    /// # Errors
    ///
    /// Propagates the job error or any observer-side error.
    fn run_job(
        &mut self,
        job_name: &'static str,
        declared_budget_us: u64,
        job: &mut dyn Job,
        ctx: &JobContext<'_>,
    ) -> Result<Option<u64>, ControllerError>;
}

/// Observer used by normal lockstep dispatch: no host timing.
#[derive(Copy, Clone, Debug, Default)]
pub struct NoJobTimingObserver;

impl JobTimingObserver for NoJobTimingObserver {
    fn run_job(
        &mut self,
        _job_name: &'static str,
        _declared_budget_us: u64,
        job: &mut dyn Job,
        ctx: &JobContext<'_>,
    ) -> Result<Option<u64>, ControllerError> {
        job.run(ctx)?;
        Ok(None)
    }
}

/// Event published on `scheduler.overrun` when a job is skipped
/// because the remaining frame budget cannot accommodate it.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct OverrunEvent {
    /// Job name as registered.
    pub job_name: &'static str,
    /// Tick on which the overrun was observed.
    pub tick: u64,
    /// Remaining frame budget when the job was evaluated.
    pub remaining_budget_us: u64,
    /// Job's declared budget that did not fit.
    pub declared_budget_us: u64,
}

impl Topic for OverrunEvent {
    const NAME: &'static str = "scheduler.overrun";
    const INDEX: usize = topic_index::SCHEDULER_OVERRUN;
}

/// Event published on `scheduler.deadline_slip` when a measured job
/// execution time exceeds that job's declared budget.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct DeadlineSlipEvent {
    /// Job name as registered.
    pub job_name: &'static str,
    /// Measured execution time in microseconds.
    pub actual_us: u64,
    /// Job's declared budget in microseconds.
    pub declared_budget_us: u64,
    /// Number of measurements recorded for this job after this
    /// sample was ingested.
    pub sample_count: u64,
}

impl Topic for DeadlineSlipEvent {
    const NAME: &'static str = "scheduler.deadline_slip";
    const INDEX: usize = topic_index::SCHEDULER_DEADLINE_SLIP;
}

/// Per-job timing budget report produced by
/// [`Scheduler::report_actual_us`].
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct TimingBudgetReport {
    /// Job name as registered.
    pub job_name: &'static str,
    /// Declared budget in microseconds.
    pub declared_budget_us: u64,
    /// Most recent measured execution time in microseconds.
    pub latest_us: u64,
    /// Maximum measured execution time in microseconds.
    pub max_us: u64,
    /// Median measured execution time in microseconds.
    pub p50_us: u64,
    /// 99th percentile measured execution time in microseconds.
    pub p99_us: u64,
    /// Number of samples recorded.
    pub sample_count: u64,
}

impl Topic for TimingBudgetReport {
    const NAME: &'static str = "scheduler.timing_budget_report";
    const INDEX: usize = topic_index::SCHEDULER_TIMING_BUDGET_REPORT;
}

#[derive(Clone, Debug)]
struct TimingSamples {
    retained: [u64; MAX_JOB_TIMING_SAMPLES],
    len: usize,
    total_count: u64,
}

impl Default for TimingSamples {
    fn default() -> Self {
        Self {
            retained: [0; MAX_JOB_TIMING_SAMPLES],
            len: 0,
            total_count: 0,
        }
    }
}

impl TimingSamples {
    fn push(&mut self, sample_us: u64) {
        if self.len < MAX_JOB_TIMING_SAMPLES {
            self.retained[self.len] = sample_us;
            self.len += 1;
        } else {
            self.retained.copy_within(1..MAX_JOB_TIMING_SAMPLES, 0);
            self.retained[MAX_JOB_TIMING_SAMPLES - 1] = sample_us;
        }
        self.total_count = self.total_count.saturating_add(1);
    }

    fn max_us(&self) -> u64 {
        self.retained[..self.len].iter().copied().max().unwrap_or(0)
    }

    fn percentile_us(&self, quantile: f64) -> u64 {
        if self.len == 0 {
            return 0;
        }
        let mut sorted = [0_u64; MAX_JOB_TIMING_SAMPLES];
        sorted[..self.len].copy_from_slice(&self.retained[..self.len]);
        sorted[..self.len].sort_unstable();
        let last = self.len.saturating_sub(1);
        let rank = (quantile.clamp(0.0, 1.0) * last as f64).ceil() as usize;
        sorted[rank.min(last)]
    }
}

struct ScheduledJob {
    job: Box<dyn Job>,
    trigger: Trigger,
    budget_us: u64,
    priority: Priority,
    /// Last bus sequence the job consumed (only meaningful for
    /// [`Trigger::TopicUpdated`]).
    last_seen: Sequence,
    /// Total tick count this job has run since registration (for
    /// telemetry / debugging).
    run_count: u64,
    /// Total tick count this job has been skipped for budget reasons.
    overrun_count: u64,
    /// Host-side measured execution times reported for this job.
    actual_samples_us: TimingSamples,
}

/// Descriptor exposed via [`Scheduler::jobs`] for dictionary generation
/// and CI introspection.
#[derive(Copy, Clone, Debug)]
pub struct JobInfo {
    /// Job name as registered.
    pub name: &'static str,
    /// Trigger condition.
    pub trigger: Trigger,
    /// Declared budget (microseconds).
    pub budget_us: u64,
    /// Job priority.
    pub priority: Priority,
    /// Cumulative successful runs.
    pub run_count: u64,
    /// Cumulative overrun-induced skips.
    pub overrun_count: u64,
}

/// Static declared-budget feasibility report over one schedule
/// hyperperiod.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct FramePackingReport {
    /// Least common multiple of periodic job periods. Topic-driven
    /// jobs are conservatively treated as due on every frame.
    pub hyperperiod_ticks: u64,
    /// Tick inside the hyperperiod with the largest declared demand.
    pub worst_frame_tick: u64,
    /// Declared budget demand on the worst frame.
    pub worst_frame_budget_us: u64,
    /// Scheduler frame budget.
    pub frame_budget_us: u64,
    /// `true` when every frame in the hyperperiod fits.
    pub feasible: bool,
    /// `true` when the full hyperperiod was enumerated. If false,
    /// `feasible` is also false because the schedule was too large to
    /// prove with this host-side check.
    pub complete: bool,
}

/// Cyclic scheduler.
pub struct Scheduler {
    /// Declared frame period in microseconds; the sum of declared
    /// budgets is gated against this on every dispatch.
    frame_budget_us: u64,
    /// Jobs registered in declaration order. Dispatch order is
    /// stable-sorted by priority in [`Scheduler::dispatch_order`] at
    /// registration time.
    jobs: StableIndexMap<&'static str, ScheduledJob>,
    /// Cached `(priority, registration_order)` dispatch order. This
    /// avoids allocating and sorting on the hot dispatch path.
    dispatch_order: [usize; MAX_SCHEDULED_JOBS],
    dispatch_order_len: usize,
}

impl Default for Scheduler {
    fn default() -> Self {
        Self::new(0)
    }
}

impl std::fmt::Debug for Scheduler {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Scheduler")
            .field("frame_budget_us", &self.frame_budget_us)
            .field("jobs", &self.jobs())
            .field("dispatch_order", &self.dispatch_order())
            .finish()
    }
}

impl Scheduler {
    /// Constructs a scheduler with the given frame-budget envelope.
    /// The default is `1_000_000` µs (1 s) which is a placeholder
    /// suitable only for tests; real scenarios pick a value derived
    /// from the simulator's `dt` (e.g. `1_000_000 / base_hz` µs).
    #[must_use]
    pub fn new(frame_budget_us: u64) -> Self {
        Self {
            frame_budget_us,
            jobs: StableIndexMap::default(),
            dispatch_order: [0; MAX_SCHEDULED_JOBS],
            dispatch_order_len: 0,
        }
    }

    /// Returns the configured frame budget in microseconds.
    #[must_use]
    pub fn frame_budget_us(&self) -> u64 {
        self.frame_budget_us
    }

    /// Registers a periodic job.
    ///
    /// # Errors
    ///
    /// Returns [`SchedulerError::NonPositivePeriod`] if `period_ticks`
    /// is `0`, [`SchedulerError::NonPositiveBudget`] if `budget_us`
    /// is `0`, and [`SchedulerError::DuplicateJob`] if a job with the
    /// same name is already registered.
    pub fn register_periodic(
        &mut self,
        period_ticks: u64,
        budget_us: u64,
        priority: Priority,
        job: Box<dyn Job>,
    ) -> Result<(), SchedulerError> {
        let name = job.name();
        if period_ticks == 0 {
            return Err(SchedulerError::NonPositivePeriod {
                job_name: name,
                period_ticks,
            });
        }
        self.register_inner(Trigger::Periodic { period_ticks }, budget_us, priority, job)
    }

    /// Registers a topic-driven job.
    ///
    /// # Errors
    ///
    /// Returns [`SchedulerError::NonPositiveBudget`] if `budget_us` is
    /// `0`, and [`SchedulerError::DuplicateJob`] if a job with the
    /// same name is already registered.
    pub fn register_topic_driven<T: Topic>(
        &mut self,
        budget_us: u64,
        priority: Priority,
        job: Box<dyn Job>,
    ) -> Result<(), SchedulerError> {
        self.register_inner(
            Trigger::TopicUpdated {
                topic_id: TopicId::of::<T>(),
                topic_name: T::NAME,
            },
            budget_us,
            priority,
            job,
        )
    }

    fn register_inner(
        &mut self,
        trigger: Trigger,
        budget_us: u64,
        priority: Priority,
        job: Box<dyn Job>,
    ) -> Result<(), SchedulerError> {
        let name = job.name();
        if budget_us == 0 {
            return Err(SchedulerError::NonPositiveBudget {
                job_name: name,
                budget_us,
            });
        }
        if self.jobs.contains_key(name) {
            return Err(SchedulerError::DuplicateJob { job_name: name });
        }
        if self.jobs.len() >= MAX_SCHEDULED_JOBS {
            return Err(SchedulerError::TooManyJobs {
                max_jobs: MAX_SCHEDULED_JOBS,
            });
        }
        let registration_index = self.jobs.len();
        self.jobs.insert(
            name,
            ScheduledJob {
                job,
                trigger,
                budget_us,
                priority,
                last_seen: Sequence::ZERO,
                run_count: 0,
                overrun_count: 0,
                actual_samples_us: TimingSamples::default(),
            },
        );
        self.dispatch_order[self.dispatch_order_len] = registration_index;
        self.dispatch_order_len += 1;
        let jobs = &self.jobs;
        let dispatch_order = &mut self.dispatch_order[..self.dispatch_order_len];
        dispatch_order.sort_by_key(|idx| {
            jobs.get_index(*idx)
                .map_or((Priority::MAX, usize::MAX), |(_, j)| (j.priority, *idx))
        });
        Ok(())
    }

    /// Dispatches a single tick. Evaluates every registered job in
    /// `(priority, registration_order)`; runs the due ones whose
    /// budget fits; emits [`OverrunEvent`]s for those that do not.
    ///
    /// `tick` is the kernel's monotonic tick counter; it is only used
    /// for [`Trigger::Periodic`] modulo checks. The dispatcher does
    /// not advance any internal counter.
    ///
    /// # Errors
    ///
    /// Propagates any error returned by [`Job::run`].
    pub fn dispatch(
        &mut self,
        tick: u64,
        bus: &Bus,
        clock: &dyn Clock,
    ) -> Result<DispatchSummary, ControllerError> {
        let mut observer = NoJobTimingObserver;
        self.dispatch_with_timing_observer(tick, bus, clock, &mut observer)
    }

    /// Dispatches a single tick with a runner-supplied timing
    /// observer. This is the hook used by host SIL runners to measure
    /// per-job wall time while keeping wall-clock APIs out of
    /// `openbmp-fc`.
    ///
    /// # Errors
    ///
    /// Propagates any error returned by [`Job::run`] or by publishing
    /// timing / overrun topics.
    pub fn dispatch_with_timing_observer<O: JobTimingObserver>(
        &mut self,
        tick: u64,
        bus: &Bus,
        clock: &dyn Clock,
        observer: &mut O,
    ) -> Result<DispatchSummary, ControllerError> {
        let mut remaining_budget = self.frame_budget_us;
        let mut summary = DispatchSummary::default();

        for order_idx in 0..self.dispatch_order_len {
            let idx = self.dispatch_order[order_idx];
            let Some((_, scheduled)) = self.jobs.get_index(idx) else {
                continue;
            };
            let due = trigger_due(&scheduled.trigger, tick, scheduled.last_seen, bus);
            if !due {
                continue;
            }
            let scheduled_budget_us = scheduled.budget_us;

            if scheduled_budget_us > remaining_budget {
                let job_name = scheduled.job.name();
                let event = OverrunEvent {
                    job_name,
                    tick,
                    remaining_budget_us: remaining_budget,
                    declared_budget_us: scheduled_budget_us,
                };
                bus.publish(event)?;
                if let Some((_, scheduled_mut)) = self.jobs.get_index_mut(idx) {
                    scheduled_mut.overrun_count = scheduled_mut.overrun_count.saturating_add(1);
                }
                summary.overrun_count = summary.overrun_count.saturating_add(1);
                continue;
            }

            // Run the job.
            let ctx = JobContext { bus, clock };
            let (consumed, actual_us) = {
                let Some((_, scheduled_mut)) = self.jobs.get_index_mut(idx) else {
                    continue;
                };
                let job_name = scheduled_mut.job.name();
                let declared_budget_us = scheduled_mut.budget_us;
                let actual_us = observer.run_job(
                    job_name,
                    declared_budget_us,
                    scheduled_mut.job.as_mut(),
                    &ctx,
                )?;
                scheduled_mut.run_count = scheduled_mut.run_count.saturating_add(1);
                if let Trigger::TopicUpdated { topic_id, .. } = scheduled_mut.trigger {
                    let seq = bus_sequence_by_id(bus, topic_id);
                    scheduled_mut.last_seen = seq;
                }
                (scheduled_mut.budget_us, actual_us)
            };
            if let Some(actual_us) = actual_us {
                self.record_actual_us_by_index(idx, actual_us, bus)?;
            }
            remaining_budget = remaining_budget.saturating_sub(consumed);
            summary.run_count = summary.run_count.saturating_add(1);
        }

        summary.remaining_budget_us = remaining_budget;
        Ok(summary)
    }

    fn dispatch_order(&self) -> &[usize] {
        &self.dispatch_order[..self.dispatch_order_len]
    }

    /// Returns descriptors for every registered job in registration
    /// order.
    #[must_use]
    pub fn jobs(&self) -> Vec<JobInfo> {
        self.jobs
            .values()
            .map(|j| JobInfo {
                name: j.job.name(),
                trigger: j.trigger,
                budget_us: j.budget_us,
                priority: j.priority,
                run_count: j.run_count,
                overrun_count: j.overrun_count,
            })
            .collect()
    }

    /// Computes a static declared-budget frame-packing report over
    /// one hyperperiod. Topic-driven jobs are conservatively included
    /// in every frame because their publish cadence is external to
    /// the scheduler.
    #[must_use]
    pub fn frame_packing_report(&self) -> FramePackingReport {
        let mut complete = true;
        let mut hyperperiod_ticks = 1_u64;
        for period_ticks in self.jobs.values().filter_map(|job| match job.trigger {
            Trigger::Periodic { period_ticks } => Some(period_ticks.max(1)),
            Trigger::TopicUpdated { .. } => None,
        }) {
            let next = saturating_lcm(hyperperiod_ticks, period_ticks);
            if next > MAX_FRAME_PACKING_HYPERPERIOD_TICKS {
                hyperperiod_ticks = MAX_FRAME_PACKING_HYPERPERIOD_TICKS;
                complete = false;
                break;
            }
            hyperperiod_ticks = next;
        }

        let mut worst_frame_tick = 0_u64;
        let mut worst_frame_budget_us = 0_u64;
        for tick in 0..hyperperiod_ticks {
            let frame_budget = self
                .jobs
                .values()
                .filter(|job| match job.trigger {
                    Trigger::Periodic { period_ticks } => tick.is_multiple_of(period_ticks.max(1)),
                    Trigger::TopicUpdated { .. } => true,
                })
                .fold(0_u64, |sum, job| sum.saturating_add(job.budget_us));
            if frame_budget > worst_frame_budget_us {
                worst_frame_budget_us = frame_budget;
                worst_frame_tick = tick;
            }
        }

        FramePackingReport {
            hyperperiod_ticks,
            worst_frame_tick,
            worst_frame_budget_us,
            frame_budget_us: self.frame_budget_us,
            feasible: complete && worst_frame_budget_us <= self.frame_budget_us,
            complete,
        }
    }

    /// Validates that the declared schedule fits in every frame of
    /// the computed hyperperiod.
    ///
    /// # Errors
    ///
    /// Returns [`SchedulerError::FrameBudgetInfeasible`] if the
    /// complete hyperperiod cannot be enumerated or any frame exceeds
    /// `frame_budget_us`.
    pub fn validate_frame_packing(&self) -> Result<FramePackingReport, SchedulerError> {
        let report = self.frame_packing_report();
        if report.feasible {
            return Ok(report);
        }
        Err(SchedulerError::FrameBudgetInfeasible {
            hyperperiod_ticks: report.hyperperiod_ticks,
            worst_frame_tick: report.worst_frame_tick,
            worst_frame_budget_us: report.worst_frame_budget_us,
            frame_budget_us: report.frame_budget_us,
        })
    }

    /// Records a runner-side wall-time measurement of how long a job
    /// took on the host.
    ///
    /// The controller crate still does not read wall-clock time; the
    /// runner or board shell owns measurement and feeds the result in
    /// here. The scheduler stores a per-job timing sample set,
    /// publishes a [`TimingBudgetReport`], and emits
    /// [`DeadlineSlipEvent`] when `actual_us > budget_us`.
    ///
    /// # Errors
    ///
    /// Returns [`ControllerError`] when publishing either timing topic
    /// fails. Unknown job names are ignored so instrumentation can be
    /// wired opportunistically around partial schedules.
    pub fn report_actual_us(
        &mut self,
        job_name: &str,
        actual_us: u64,
        bus: &Bus,
    ) -> Result<(), ControllerError> {
        let Some(idx) = self.jobs.get_index_of(job_name) else {
            return Ok(());
        };
        self.record_actual_us_by_index(idx, actual_us, bus)
    }

    fn record_actual_us_by_index(
        &mut self,
        idx: usize,
        actual_us: u64,
        bus: &Bus,
    ) -> Result<(), ControllerError> {
        let Some((_, scheduled)) = self.jobs.get_index_mut(idx) else {
            return Ok(());
        };
        scheduled.actual_samples_us.push(actual_us);
        let sample_count = scheduled.actual_samples_us.total_count;
        let max_us = scheduled.actual_samples_us.max_us();
        let p50_us = scheduled.actual_samples_us.percentile_us(0.50);
        let p99_us = scheduled.actual_samples_us.percentile_us(0.99);
        let report = TimingBudgetReport {
            job_name: scheduled.job.name(),
            declared_budget_us: scheduled.budget_us,
            latest_us: actual_us,
            max_us,
            p50_us,
            p99_us,
            sample_count,
        };
        bus.publish(report)?;
        if actual_us > scheduled.budget_us {
            bus.publish(DeadlineSlipEvent {
                job_name: scheduled.job.name(),
                actual_us,
                declared_budget_us: scheduled.budget_us,
                sample_count,
            })?;
        }
        Ok(())
    }
}

#[derive(Copy, Clone, Debug, Default)]
/// Per-tick dispatch summary.
pub struct DispatchSummary {
    /// Number of jobs that ran successfully.
    pub run_count: u64,
    /// Number of jobs that were skipped because their declared budget
    /// did not fit in the remaining frame budget.
    pub overrun_count: u64,
    /// Frame budget remaining at end of dispatch (microseconds).
    pub remaining_budget_us: u64,
}

/// Decides whether a job is due this tick.
fn trigger_due(trigger: &Trigger, tick: u64, last_seen: Sequence, bus: &Bus) -> bool {
    match trigger {
        Trigger::Periodic { period_ticks } => {
            // Defensive: register_periodic rejected period == 0 already.
            if *period_ticks == 0 {
                return false;
            }
            tick.is_multiple_of(*period_ticks)
        }
        Trigger::TopicUpdated { topic_id, .. } => {
            let current = bus_sequence_by_id(bus, *topic_id);
            current.value() > last_seen.value()
        }
    }
}

/// Looks up a topic's sequence counter by dense topic id. Returns
/// `Sequence::ZERO` for unknown topics — the scheduler is permissive
/// here so a topic-driven job can be registered before its producer
/// has registered the topic; the job simply never fires until the
/// producer publishes the first value.
fn bus_sequence_by_id(bus: &Bus, topic_id: TopicId) -> Sequence {
    bus.sequence_by_id(topic_id)
}

fn saturating_lcm(a: u64, b: u64) -> u64 {
    if a == 0 || b == 0 {
        return 0;
    }
    a.saturating_div(gcd(a, b)).saturating_mul(b)
}

fn gcd(mut a: u64, mut b: u64) -> u64 {
    while b != 0 {
        let r = a % b;
        a = b;
        b = r;
    }
    a
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::items_after_statements
)]
mod tests {
    use std::cell::Cell;
    use std::rc::Rc;

    use openbmp_core::{SimTime, StepIndex};

    use super::*;
    use crate::bus::Bus;
    use crate::clock::SimulatedClock;

    #[derive(Clone, Debug, PartialEq, Eq)]
    struct Beat {
        n: u32,
    }

    impl Topic for Beat {
        const NAME: &'static str = "test.beat";
    }

    struct CounterJob {
        name: &'static str,
        counter: Rc<Cell<u32>>,
    }

    impl Job for CounterJob {
        fn name(&self) -> &'static str {
            self.name
        }
        fn run(&mut self, _ctx: &JobContext<'_>) -> Result<(), ControllerError> {
            self.counter.set(self.counter.get() + 1);
            Ok(())
        }
    }

    struct FixedTimingObserver {
        actual_us: u64,
    }

    impl JobTimingObserver for FixedTimingObserver {
        fn run_job(
            &mut self,
            _job_name: &'static str,
            _declared_budget_us: u64,
            job: &mut dyn Job,
            ctx: &JobContext<'_>,
        ) -> Result<Option<u64>, ControllerError> {
            job.run(ctx)?;
            Ok(Some(self.actual_us))
        }
    }

    #[test]
    fn periodic_job_runs_on_each_period() {
        let bus = Bus::new();
        bus.register::<OverrunEvent>().unwrap();
        let clock = SimulatedClock::new();
        let counter = Rc::new(Cell::new(0u32));
        let mut sched = Scheduler::new(1_000_000);
        sched
            .register_periodic(
                5,
                100,
                10,
                Box::new(CounterJob {
                    name: "counter",
                    counter: counter.clone(),
                }),
            )
            .unwrap();

        for tick in 0..100u64 {
            clock.set(
                SimTime::from_seconds(tick as f64 * 0.001),
                StepIndex::new(tick),
            );
            sched.dispatch(tick, &bus, &clock).unwrap();
        }
        // Expected: tick 0, 5, 10, ..., 95 → 20 fires.
        assert_eq!(counter.get(), 20);
    }

    #[test]
    fn priority_orders_dispatch() {
        let bus = Bus::new();
        bus.register::<OverrunEvent>().unwrap();
        let clock = SimulatedClock::new();

        // Two jobs: the lower-priority one observes the higher-priority's side effect.
        struct ObservingJob {
            name: &'static str,
            log: Rc<std::cell::RefCell<Vec<&'static str>>>,
        }
        impl Job for ObservingJob {
            fn name(&self) -> &'static str {
                self.name
            }
            fn run(&mut self, _ctx: &JobContext<'_>) -> Result<(), ControllerError> {
                self.log.borrow_mut().push(self.name);
                Ok(())
            }
        }

        let log = Rc::new(std::cell::RefCell::new(Vec::new()));
        let mut sched = Scheduler::new(1_000_000);
        sched
            .register_periodic(
                1,
                100,
                20,
                Box::new(ObservingJob {
                    name: "low",
                    log: log.clone(),
                }),
            )
            .unwrap();
        sched
            .register_periodic(
                1,
                100,
                10,
                Box::new(ObservingJob {
                    name: "high",
                    log: log.clone(),
                }),
            )
            .unwrap();

        sched.dispatch(0, &bus, &clock).unwrap();
        let log = log.borrow();
        assert_eq!(log.as_slice(), &["high", "low"]);
    }

    #[test]
    fn report_actual_us_publishes_timing_report_and_deadline_slip() {
        let bus = Bus::new();
        bus.register::<TimingBudgetReport>().unwrap();
        bus.register::<DeadlineSlipEvent>().unwrap();
        let counter = Rc::new(Cell::new(0u32));
        let mut sched = Scheduler::new(1_000);
        sched
            .register_periodic(
                1,
                100,
                10,
                Box::new(CounterJob {
                    name: "timed",
                    counter,
                }),
            )
            .unwrap();

        sched.report_actual_us("timed", 75, &bus).unwrap();
        sched.report_actual_us("timed", 125, &bus).unwrap();

        let (report, _) = bus.latest::<TimingBudgetReport>().unwrap().unwrap();
        assert_eq!(report.job_name, "timed");
        assert_eq!(report.declared_budget_us, 100);
        assert_eq!(report.latest_us, 125);
        assert_eq!(report.max_us, 125);
        assert_eq!(report.sample_count, 2);
        let (slip, _) = bus.latest::<DeadlineSlipEvent>().unwrap().unwrap();
        assert_eq!(slip.job_name, "timed");
        assert_eq!(slip.actual_us, 125);
        assert_eq!(slip.declared_budget_us, 100);
    }

    #[test]
    fn dispatch_timing_observer_feeds_timing_topics() {
        let bus = Bus::new();
        bus.register::<OverrunEvent>().unwrap();
        bus.register::<TimingBudgetReport>().unwrap();
        bus.register::<DeadlineSlipEvent>().unwrap();
        let clock = SimulatedClock::new();
        let counter = Rc::new(Cell::new(0u32));
        let mut sched = Scheduler::new(1_000);
        sched
            .register_periodic(
                1,
                100,
                10,
                Box::new(CounterJob {
                    name: "observed",
                    counter: counter.clone(),
                }),
            )
            .unwrap();
        let mut observer = FixedTimingObserver { actual_us: 125 };

        let summary = sched
            .dispatch_with_timing_observer(0, &bus, &clock, &mut observer)
            .unwrap();

        assert_eq!(summary.run_count, 1);
        assert_eq!(counter.get(), 1);
        let (report, _) = bus.latest::<TimingBudgetReport>().unwrap().unwrap();
        assert_eq!(report.job_name, "observed");
        assert_eq!(report.latest_us, 125);
        let (slip, _) = bus.latest::<DeadlineSlipEvent>().unwrap().unwrap();
        assert_eq!(slip.job_name, "observed");
    }

    #[test]
    fn frame_packing_report_proves_worst_hyperperiod_frame() {
        let counter = Rc::new(Cell::new(0u32));
        let mut sched = Scheduler::new(120);
        sched
            .register_periodic(
                1,
                40,
                10,
                Box::new(CounterJob {
                    name: "fast",
                    counter: counter.clone(),
                }),
            )
            .unwrap();
        sched
            .register_periodic(
                2,
                70,
                20,
                Box::new(CounterJob {
                    name: "slow",
                    counter: counter.clone(),
                }),
            )
            .unwrap();
        sched
            .register_topic_driven::<Beat>(
                10,
                30,
                Box::new(CounterJob {
                    name: "topic",
                    counter,
                }),
            )
            .unwrap();

        let report = sched.frame_packing_report();

        assert_eq!(report.hyperperiod_ticks, 2);
        assert_eq!(report.worst_frame_tick, 0);
        assert_eq!(report.worst_frame_budget_us, 120);
        assert!(report.feasible);
        assert!(report.complete);
    }

    #[test]
    fn frame_packing_validation_rejects_infeasible_schedule() {
        let counter = Rc::new(Cell::new(0u32));
        let mut sched = Scheduler::new(99);
        sched
            .register_periodic(
                1,
                40,
                10,
                Box::new(CounterJob {
                    name: "fast",
                    counter: counter.clone(),
                }),
            )
            .unwrap();
        sched
            .register_periodic(
                2,
                60,
                20,
                Box::new(CounterJob {
                    name: "slow",
                    counter,
                }),
            )
            .unwrap();

        let err = sched.validate_frame_packing().unwrap_err();

        assert!(matches!(
            err,
            SchedulerError::FrameBudgetInfeasible {
                worst_frame_budget_us: 100,
                frame_budget_us: 99,
                ..
            }
        ));
    }

    #[test]
    fn budget_overrun_emits_event_and_skips_job() {
        let bus = Bus::new();
        bus.register::<OverrunEvent>().unwrap();
        let clock = SimulatedClock::new();
        let counter = Rc::new(Cell::new(0u32));

        // Frame budget too small for the job.
        let mut sched = Scheduler::new(50);
        sched
            .register_periodic(
                1,
                100,
                10,
                Box::new(CounterJob {
                    name: "fat-job",
                    counter: counter.clone(),
                }),
            )
            .unwrap();

        let summary = sched.dispatch(0, &bus, &clock).unwrap();
        assert_eq!(counter.get(), 0);
        assert_eq!(summary.run_count, 0);
        assert_eq!(summary.overrun_count, 1);

        let (event, _) = bus.latest::<OverrunEvent>().unwrap().unwrap();
        assert_eq!(event.job_name, "fat-job");
        assert_eq!(event.declared_budget_us, 100);
        assert_eq!(event.remaining_budget_us, 50);
    }

    #[test]
    fn duplicate_job_rejected() {
        let mut sched = Scheduler::new(1_000_000);
        let counter = Rc::new(Cell::new(0u32));
        sched
            .register_periodic(
                1,
                10,
                10,
                Box::new(CounterJob {
                    name: "j",
                    counter: counter.clone(),
                }),
            )
            .unwrap();
        let err = sched
            .register_periodic(1, 10, 10, Box::new(CounterJob { name: "j", counter }))
            .unwrap_err();
        assert!(matches!(err, SchedulerError::DuplicateJob { .. }));
    }

    #[test]
    fn topic_driven_fires_only_when_topic_updates() {
        let bus = Bus::new();
        bus.register::<Beat>().unwrap();
        bus.register::<OverrunEvent>().unwrap();
        let clock = SimulatedClock::new();
        let counter = Rc::new(Cell::new(0u32));

        let mut sched = Scheduler::new(1_000_000);
        sched
            .register_topic_driven::<Beat>(
                100,
                10,
                Box::new(CounterJob {
                    name: "consumer",
                    counter: counter.clone(),
                }),
            )
            .unwrap();

        // No publish: never fires.
        sched.dispatch(0, &bus, &clock).unwrap();
        sched.dispatch(1, &bus, &clock).unwrap();
        assert_eq!(counter.get(), 0);

        // Publish once: fires once.
        bus.publish(Beat { n: 1 }).unwrap();
        sched.dispatch(2, &bus, &clock).unwrap();
        assert_eq!(counter.get(), 1);

        // Same publish: does not refire.
        sched.dispatch(3, &bus, &clock).unwrap();
        assert_eq!(counter.get(), 1);

        // Re-publish: fires again.
        bus.publish(Beat { n: 2 }).unwrap();
        sched.dispatch(4, &bus, &clock).unwrap();
        assert_eq!(counter.get(), 2);
    }
}
