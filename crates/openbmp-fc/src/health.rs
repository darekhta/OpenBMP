//! Health & arming monitor.
//!
//! Subscribes every sensor / estimator / scheduler topic the
//! controller publishes and aggregates them into a single
//! [`FailsafeFlags`] topic that the commander treats as a hard
//! arming-block.
//!
//! Provides dead-reckoning detection, sensor-divergence flagging,
//! and scheduler overrun aggregation.

use crate::error::ControllerError;
use crate::params::ParamSection;
use crate::scheduler::{DeadlineSlipEvent, Job, JobContext, OverrunEvent};
use crate::topics::{
    ActuatorStatus, BarometerSample, EstimatorStatus, FailsafeFlags, GnssSample, ImuSample,
    MagnetometerSample, StorageStatus, WatchdogStatus,
};

/// Health-monitor configuration.
#[derive(Clone, Debug)]
pub struct HealthParams {
    /// Maximum tolerable interval (s) between IMU samples before
    /// flagging the IMU unhealthy.
    pub imu_stale_after_s: f64,
    /// Maximum tolerable interval (s) between GNSS samples before
    /// flagging GNSS unhealthy.
    pub gnss_stale_after_s: f64,
    /// Maximum tolerable interval (s) between baro samples before
    /// flagging the baro unhealthy.
    pub baro_stale_after_s: f64,
    /// Maximum tolerable interval (s) between mag samples before
    /// flagging the mag unhealthy.
    pub mag_stale_after_s: f64,
    /// Number of consecutive scheduler overruns before flagging the
    /// scheduler unhealthy.
    pub overrun_burst_count: u32,
    /// Maximum tolerable interval (s) between actuator status
    /// publishes after the first status has been observed.
    pub actuator_stale_after_s: f64,
    /// Maximum tolerable interval (s) between watchdog status
    /// publishes after the first status has been observed.
    pub watchdog_stale_after_s: f64,
    /// Maximum tolerable interval (s) between storage status
    /// publishes after the first status has been observed.
    pub storage_stale_after_s: f64,
}

impl Default for HealthParams {
    fn default() -> Self {
        Self {
            imu_stale_after_s: 0.05,
            gnss_stale_after_s: 0.5,
            baro_stale_after_s: 0.2,
            mag_stale_after_s: 0.2,
            overrun_burst_count: 5,
            actuator_stale_after_s: 0.2,
            watchdog_stale_after_s: 1.0,
            storage_stale_after_s: 10.0,
        }
    }
}

impl ParamSection for HealthParams {
    const NAME: &'static str = "health";
}

/// Per-topic staleness tracker. Stores the last observed bus sequence
/// number and the clock time at which that sequence was first seen by
/// the health monitor. Stale = `clock.now()` − `last_advance_time_s`
/// exceeding the topic's threshold.
#[derive(Copy, Clone, Debug, Default)]
struct TopicStaleness {
    last_seq: u64,
    last_advance_time_s: f64,
}

impl TopicStaleness {
    fn observe(&mut self, current_seq: u64, now_s: f64) {
        if current_seq > self.last_seq {
            self.last_seq = current_seq;
            self.last_advance_time_s = now_s;
        }
    }

    fn is_stale(&self, now_s: f64, threshold_s: f64) -> bool {
        // Before the first publish, last_seq is 0 and
        // last_advance_time_s is 0; treat as fresh until clock advances
        // past threshold.
        if self.last_seq == 0 {
            return now_s > threshold_s;
        }
        (now_s - self.last_advance_time_s) > threshold_s
    }

    fn is_stale_after_first_publish(&self, now_s: f64, threshold_s: f64) -> bool {
        self.last_seq > 0 && self.is_stale(now_s, threshold_s)
    }
}

/// Health monitor job.
#[derive(Debug)]
pub struct HealthMonitor {
    name: &'static str,
    params: HealthParams,
    imu: TopicStaleness,
    gnss: TopicStaleness,
    baro: TopicStaleness,
    mag: TopicStaleness,
    actuator: TopicStaleness,
    watchdog: TopicStaleness,
    storage: TopicStaleness,
    consecutive_overruns: u32,
    consecutive_deadline_slips: u32,
    last_overrun_seq: u64,
    last_deadline_slip_seq: u64,
}

impl HealthMonitor {
    /// Constructs a health monitor with the given parameters.
    #[must_use]
    pub fn new(params: HealthParams) -> Self {
        Self {
            name: "health.tick",
            params,
            imu: TopicStaleness::default(),
            gnss: TopicStaleness::default(),
            baro: TopicStaleness::default(),
            mag: TopicStaleness::default(),
            actuator: TopicStaleness::default(),
            watchdog: TopicStaleness::default(),
            storage: TopicStaleness::default(),
            consecutive_overruns: 0,
            consecutive_deadline_slips: 0,
            last_overrun_seq: 0,
            last_deadline_slip_seq: 0,
        }
    }
}

impl Job for HealthMonitor {
    fn name(&self) -> &'static str {
        self.name
    }

    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        let now = ctx.clock.now().as_seconds();
        let mut flags = FailsafeFlags::default();

        if let Ok(seq) = ctx.bus.sequence::<ImuSample>() {
            self.imu.observe(seq.value(), now);
        }
        if let Ok(seq) = ctx.bus.sequence::<GnssSample>() {
            self.gnss.observe(seq.value(), now);
        }
        if let Ok(seq) = ctx.bus.sequence::<BarometerSample>() {
            self.baro.observe(seq.value(), now);
        }
        if let Ok(seq) = ctx.bus.sequence::<MagnetometerSample>() {
            self.mag.observe(seq.value(), now);
        }
        if let Ok(seq) = ctx.bus.sequence::<ActuatorStatus>() {
            self.actuator.observe(seq.value(), now);
        }
        if let Ok(seq) = ctx.bus.sequence::<WatchdogStatus>() {
            self.watchdog.observe(seq.value(), now);
        }
        if let Ok(seq) = ctx.bus.sequence::<StorageStatus>() {
            self.storage.observe(seq.value(), now);
        }

        flags.imu_unhealthy = self.imu.is_stale(now, self.params.imu_stale_after_s);
        flags.gnss_unhealthy = self.gnss.is_stale(now, self.params.gnss_stale_after_s);
        flags.baro_unhealthy = self.baro.is_stale(now, self.params.baro_stale_after_s);
        flags.mag_unhealthy = self.mag.is_stale(now, self.params.mag_stale_after_s);
        flags.actuator_unhealthy = self
            .actuator
            .is_stale_after_first_publish(now, self.params.actuator_stale_after_s);
        flags.watchdog_unhealthy = self
            .watchdog
            .is_stale_after_first_publish(now, self.params.watchdog_stale_after_s);
        flags.storage_unhealthy = self
            .storage
            .is_stale_after_first_publish(now, self.params.storage_stale_after_s);

        if let Ok(Some((sample, _))) = ctx.bus.latest::<ImuSample>() {
            flags.imu_unhealthy |= !sample.healthy;
        }
        if let Ok(Some((sample, _))) = ctx.bus.latest::<GnssSample>() {
            flags.gnss_unhealthy |= !sample.healthy;
        }
        if let Ok(Some((sample, _))) = ctx.bus.latest::<BarometerSample>() {
            flags.baro_unhealthy |= !sample.healthy;
        }
        if let Ok(Some((sample, _))) = ctx.bus.latest::<MagnetometerSample>() {
            flags.mag_unhealthy |= !sample.healthy;
        }
        if let Ok(Some((status, _))) = ctx.bus.latest::<ActuatorStatus>() {
            flags.actuator_unhealthy |= !status.healthy;
        }
        if let Ok(Some((status, _))) = ctx.bus.latest::<WatchdogStatus>() {
            flags.watchdog_unhealthy |= !status.healthy;
        }
        if let Ok(Some((status, _))) = ctx.bus.latest::<StorageStatus>() {
            flags.storage_unhealthy |= !status.healthy;
        }

        if let Ok(Some((est, _))) = ctx.bus.latest::<EstimatorStatus>() {
            flags.estimator_dead_reckoning = est.dead_reckoning;
        }

        if let Ok(seq) = ctx.bus.sequence::<OverrunEvent>() {
            if seq.value() > self.last_overrun_seq {
                self.consecutive_overruns = self.consecutive_overruns.saturating_add(1);
                self.last_overrun_seq = seq.value();
            } else {
                self.consecutive_overruns = 0;
            }
        }
        flags.scheduler_overrun = self.consecutive_overruns >= self.params.overrun_burst_count;
        if let Ok(seq) = ctx.bus.sequence::<DeadlineSlipEvent>() {
            if seq.value() > self.last_deadline_slip_seq {
                self.consecutive_deadline_slips = self.consecutive_deadline_slips.saturating_add(1);
                self.last_deadline_slip_seq = seq.value();
            } else {
                self.consecutive_deadline_slips = 0;
            }
        }
        flags.deadline_slip = self.consecutive_deadline_slips >= self.params.overrun_burst_count;

        let _ = ctx.bus.publish(flags);
        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use openbmp_core::{SimTime, StepIndex};

    use super::*;
    use crate::bus::Bus;
    use crate::clock::{Clock as _, SimulatedClock};
    use crate::scheduler::JobContext;

    #[test]
    fn imu_stale_after_threshold_when_no_publishes() {
        let bus = Bus::new();
        bus.register::<ImuSample>().unwrap();
        bus.register::<GnssSample>().unwrap();
        bus.register::<BarometerSample>().unwrap();
        bus.register::<MagnetometerSample>().unwrap();
        bus.register::<EstimatorStatus>().unwrap();
        bus.register::<OverrunEvent>().unwrap();
        bus.register::<FailsafeFlags>().unwrap();
        let clock = SimulatedClock::new();

        let mut h = HealthMonitor::new(HealthParams {
            imu_stale_after_s: 0.05,
            ..HealthParams::default()
        });

        // First tick at t=0: not yet stale (threshold is 0.05).
        clock.set(SimTime::ZERO, StepIndex::new(0));
        h.run(&JobContext {
            bus: &bus,
            clock: &clock,
        })
        .unwrap();
        let (flags, _) = bus.latest::<FailsafeFlags>().unwrap().unwrap();
        assert!(!flags.imu_unhealthy);

        // Tick at t=0.1 with no IMU publishes → stale.
        clock.set(SimTime::from_seconds(0.1), StepIndex::new(1));
        h.run(&JobContext {
            bus: &bus,
            clock: &clock,
        })
        .unwrap();
        let (flags, _) = bus.latest::<FailsafeFlags>().unwrap().unwrap();
        assert!(flags.imu_unhealthy);
    }

    #[test]
    fn imu_publishes_keep_health_fresh() {
        use nalgebra::Vector3;

        let bus = Bus::new();
        bus.register::<ImuSample>().unwrap();
        bus.register::<GnssSample>().unwrap();
        bus.register::<BarometerSample>().unwrap();
        bus.register::<MagnetometerSample>().unwrap();
        bus.register::<EstimatorStatus>().unwrap();
        bus.register::<OverrunEvent>().unwrap();
        bus.register::<FailsafeFlags>().unwrap();
        let clock = SimulatedClock::new();
        let mut h = HealthMonitor::new(HealthParams::default());

        for k in 0..200u64 {
            clock.set(
                SimTime::from_seconds(f64::from(u32::try_from(k).unwrap()) * 0.001),
                StepIndex::new(k),
            );
            // 1 kHz IMU publishes — well within the 50 ms staleness
            // threshold.
            bus.publish(ImuSample {
                time: clock.now(),
                gyro_rad_s: Vector3::zeros(),
                accel_m_s2: Vector3::new(0.0, 0.0, openbmp_physics::gravity::STANDARD_GRAVITY_M_S2),
                healthy: true,
            })
            .unwrap();
            h.run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();
        }
        let (flags, _) = bus.latest::<FailsafeFlags>().unwrap().unwrap();
        assert!(!flags.imu_unhealthy);
    }

    #[test]
    fn unhealthy_sample_flags_sensor_fault_without_waiting_for_stale_timeout() {
        use nalgebra::Vector3;

        let bus = Bus::new();
        bus.register::<ImuSample>().unwrap();
        bus.register::<GnssSample>().unwrap();
        bus.register::<BarometerSample>().unwrap();
        bus.register::<MagnetometerSample>().unwrap();
        bus.register::<EstimatorStatus>().unwrap();
        bus.register::<OverrunEvent>().unwrap();
        bus.register::<FailsafeFlags>().unwrap();
        let clock = SimulatedClock::new();
        clock.set(SimTime::ZERO, StepIndex::new(0));
        bus.publish(ImuSample {
            time: SimTime::ZERO,
            gyro_rad_s: Vector3::zeros(),
            accel_m_s2: Vector3::zeros(),
            healthy: false,
        })
        .unwrap();

        let mut h = HealthMonitor::new(HealthParams {
            imu_stale_after_s: 10.0,
            ..HealthParams::default()
        });
        h.run(&JobContext {
            bus: &bus,
            clock: &clock,
        })
        .unwrap();

        let (flags, _) = bus.latest::<FailsafeFlags>().unwrap().unwrap();
        assert!(flags.imu_unhealthy);
    }

    #[test]
    fn scheduler_overrun_burst_flags_failsafe() {
        let bus = Bus::new();
        bus.register::<ImuSample>().unwrap();
        bus.register::<GnssSample>().unwrap();
        bus.register::<BarometerSample>().unwrap();
        bus.register::<MagnetometerSample>().unwrap();
        bus.register::<EstimatorStatus>().unwrap();
        bus.register::<OverrunEvent>().unwrap();
        bus.register::<FailsafeFlags>().unwrap();
        let clock = SimulatedClock::new();
        let mut h = HealthMonitor::new(HealthParams {
            imu_stale_after_s: 10.0,
            gnss_stale_after_s: 10.0,
            baro_stale_after_s: 10.0,
            mag_stale_after_s: 10.0,
            overrun_burst_count: 2,
            ..HealthParams::default()
        });

        for tick in 0..2_u64 {
            clock.set(
                SimTime::from_seconds(tick as f64 * 0.001),
                StepIndex::new(tick),
            );
            bus.publish(OverrunEvent {
                job_name: "guidance.tick",
                tick,
                remaining_budget_us: 25,
                declared_budget_us: 100,
            })
            .unwrap();
            h.run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();
        }

        let (flags, _) = bus.latest::<FailsafeFlags>().unwrap().unwrap();
        assert!(flags.scheduler_overrun);

        clock.set(SimTime::from_seconds(0.003), StepIndex::new(3));
        h.run(&JobContext {
            bus: &bus,
            clock: &clock,
        })
        .unwrap();
        let (flags, _) = bus.latest::<FailsafeFlags>().unwrap().unwrap();
        assert!(!flags.scheduler_overrun);
    }

    #[test]
    fn deadline_slip_burst_flags_failsafe() {
        let bus = Bus::new();
        bus.register::<DeadlineSlipEvent>().unwrap();
        bus.register::<FailsafeFlags>().unwrap();
        let clock = SimulatedClock::new();
        let mut h = HealthMonitor::new(HealthParams {
            imu_stale_after_s: 10.0,
            gnss_stale_after_s: 10.0,
            baro_stale_after_s: 10.0,
            mag_stale_after_s: 10.0,
            overrun_burst_count: 2,
            ..HealthParams::default()
        });

        for tick in 0..2_u64 {
            clock.set(
                SimTime::from_seconds(tick as f64 * 0.001),
                StepIndex::new(tick),
            );
            bus.publish(DeadlineSlipEvent {
                job_name: "guidance.tick",
                actual_us: 250,
                declared_budget_us: 100,
                sample_count: tick + 1,
            })
            .unwrap();
            h.run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();
        }

        let (flags, _) = bus.latest::<FailsafeFlags>().unwrap().unwrap();
        assert!(flags.deadline_slip);
    }

    #[test]
    fn unhealthy_non_sensor_status_flags_failsafe() {
        let bus = Bus::new();
        bus.register::<ActuatorStatus>().unwrap();
        bus.register::<WatchdogStatus>().unwrap();
        bus.register::<StorageStatus>().unwrap();
        bus.register::<FailsafeFlags>().unwrap();
        let clock = SimulatedClock::new();
        clock.set(SimTime::ZERO, StepIndex::ZERO);
        bus.publish(ActuatorStatus {
            time: SimTime::ZERO,
            healthy: false,
        })
        .unwrap();
        bus.publish(WatchdogStatus {
            time: SimTime::ZERO,
            healthy: false,
        })
        .unwrap();
        bus.publish(StorageStatus {
            time: SimTime::ZERO,
            healthy: false,
        })
        .unwrap();

        let mut h = HealthMonitor::new(HealthParams::default());
        h.run(&JobContext {
            bus: &bus,
            clock: &clock,
        })
        .unwrap();

        let (flags, _) = bus.latest::<FailsafeFlags>().unwrap().unwrap();
        assert!(flags.actuator_unhealthy);
        assert!(flags.watchdog_unhealthy);
        assert!(flags.storage_unhealthy);
    }
}
