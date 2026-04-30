//! Fault detection / isolation / recovery (FDIR).
//!
//! Phase 4.6: residual-based detectors layered on the
//! `EstimatorStatus` innovation gates and the scheduler's
//! overrun events. When a detector trips, the FDIR module publishes
//! `FdirStatus` and the commander treats the trip as a hard
//! arming-block.

use crate::error::ControllerError;
use crate::params::ParamSection;
use crate::scheduler::{Job, JobContext};
use crate::topics::{EstimatorStatus, FailsafeFlags, FdirStatus};

/// FDIR parameters.
#[derive(Clone, Debug)]
pub struct FdirParams {
    /// Innovation chi-square threshold for the estimator-divergence
    /// detector.
    pub innovation_threshold: f64,
    /// Number of consecutive innovation breaches before the detector
    /// trips.
    pub innovation_burst_count: u32,
    /// Number of failsafe-flag bursts before the FDIR trip latches.
    pub failsafe_burst_count: u32,
}

impl Default for FdirParams {
    fn default() -> Self {
        Self {
            innovation_threshold: 25.0,
            innovation_burst_count: 5,
            failsafe_burst_count: 5,
        }
    }
}

impl ParamSection for FdirParams {
    const NAME: &'static str = "fdir";
}

/// FDIR job.
#[derive(Debug)]
pub struct FdirJob {
    name: &'static str,
    params: FdirParams,
    innovation_burst: u32,
    failsafe_burst: u32,
    triggered_mask: u64,
    ticks_since_trip: u64,
}

impl FdirJob {
    /// Constructs the FDIR job with the given parameters.
    #[must_use]
    pub fn new(params: FdirParams) -> Self {
        Self {
            name: "fdir.tick",
            params,
            innovation_burst: 0,
            failsafe_burst: 0,
            triggered_mask: 0,
            ticks_since_trip: 0,
        }
    }
}

impl Job for FdirJob {
    fn name(&self) -> &'static str {
        self.name
    }

    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        if let Ok(Some((est, _))) = ctx.bus.latest::<EstimatorStatus>() {
            let max_chi2 = est
                .gnss_chi2
                .max(est.baro_chi2)
                .max(est.mag_chi2)
                .max(est.imu_chi2);
            if max_chi2 > self.params.innovation_threshold {
                self.innovation_burst = self.innovation_burst.saturating_add(1);
            } else {
                self.innovation_burst = 0;
            }
            if self.innovation_burst >= self.params.innovation_burst_count {
                self.triggered_mask |= 1 << 0;
            }
        }

        if let Ok(Some((flags, _))) = ctx.bus.latest::<FailsafeFlags>() {
            if flags.any() {
                self.failsafe_burst = self.failsafe_burst.saturating_add(1);
            } else {
                self.failsafe_burst = 0;
            }
            if self.failsafe_burst >= self.params.failsafe_burst_count {
                self.triggered_mask |= 1 << 1;
            }
        }

        let triggered = self.triggered_mask != 0;
        if triggered {
            self.ticks_since_trip = self.ticks_since_trip.saturating_add(1);
        } else {
            self.ticks_since_trip = 0;
        }

        let status = FdirStatus {
            triggered,
            tripped_mask: self.triggered_mask,
            ticks_since_trip: self.ticks_since_trip,
        };
        let _ = ctx.bus.publish(status);
        Ok(())
    }
}
