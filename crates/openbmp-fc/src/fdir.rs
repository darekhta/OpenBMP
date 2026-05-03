//! Fault detection / isolation / recovery (FDIR).
//!
//! Phase 4.6: residual-based detectors layered on the
//! `EstimatorStatus` innovation gates and the scheduler's
//! overrun events. When a detector trips, the FDIR module publishes
//! `FdirStatus` and the commander treats the trip as a hard
//! arming-block.

use crate::error::ControllerError;
use crate::glrt::{StepOutcome, WindowedMeanShiftGlrt};
use crate::params::ParamSection;
use crate::scheduler::{Job, JobContext};
use crate::topics::{
    ActuatorCommand, AutopilotStatus, EstimatorStatus, FailsafeFlags, FdirGlrtDiagnostic,
    FdirStatus,
};

/// Fault-tree bit: IMU lane or innovation fault.
pub const FDIR_BIT_IMU: u64 = 1 << 0;
/// Fault-tree bit: barometer lane or innovation fault.
pub const FDIR_BIT_BARO: u64 = 1 << 1;
/// Fault-tree bit: GNSS lane or innovation fault.
pub const FDIR_BIT_GNSS: u64 = 1 << 2;
/// Fault-tree bit: magnetometer lane or innovation fault.
pub const FDIR_BIT_MAG: u64 = 1 << 3;
/// Fault-tree bit: scheduler budget overrun.
pub const FDIR_BIT_SCHEDULER_OVERRUN: u64 = 1 << 4;
/// Fault-tree bit: estimator dead-reckoning.
pub const FDIR_BIT_ESTIMATOR_DEAD_RECKONING: u64 = 1 << 5;
/// Fault-tree bit: autopilot saturation.
pub const FDIR_BIT_AUTOPILOT_SATURATION: u64 = 1 << 6;
/// Fault-tree bit: differential-flatness reference suppression.
pub const FDIR_BIT_AUTOPILOT_REFERENCE_SUPPRESSED: u64 = 1 << 7;

/// Detector family.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub enum DetectorKind {
    /// Consecutive-breach detector.
    #[default]
    BurstCounter,
    /// Single-sample Gaussian-innovation GLRT detector. The
    /// chi-square innovation statistic published by the estimator is
    /// the unconstrained mean-shift GLRT statistic, so this variant
    /// thresholds it directly.
    SingleSampleGlrt,
    /// Cumulative-sum detector.
    Cusum,
    /// Phase-5.B.4 — Willsky 1976 windowed-mean-shift GLRT, vector-form,
    /// running on the per-sensor whitened-innovation streams that the
    /// estimator now publishes on
    /// [`crate::topics::EstimatorStatus`]. See [`crate::glrt`] for the
    /// algorithm, threshold derivation, and determinism contract.
    WindowedMeanShiftGlrt,
}

/// FDIR parameters.
#[derive(Clone, Debug)]
pub struct FdirParams {
    /// Detector family.
    pub detector_kind: DetectorKind,
    /// Innovation chi-square threshold for the estimator-divergence
    /// detector.
    pub innovation_threshold: f64,
    /// Number of consecutive innovation breaches before the detector
    /// trips.
    pub innovation_burst_count: u32,
    /// Number of failsafe-flag bursts before the FDIR trip latches.
    pub failsafe_burst_count: u32,
    /// CUSUM drift term subtracted from the innovation statistic.
    pub cusum_drift: f64,
    /// CUSUM trip threshold.
    pub cusum_threshold: f64,
    /// Phase-5.B.4 — number of past samples retained by the
    /// windowed-mean-shift GLRT. Only read when
    /// `detector_kind = WindowedMeanShiftGlrt`.
    pub glrt_window_samples: u32,
    /// Phase-5.B.4 — desired family-wise false-alarm rate (`α`) over
    /// the GLRT window. Bonferroni-corrected internally per candidate
    /// jump time. Only read when
    /// `detector_kind = WindowedMeanShiftGlrt`.
    pub glrt_false_alarm_rate: f64,
}

impl Default for FdirParams {
    fn default() -> Self {
        Self {
            detector_kind: DetectorKind::BurstCounter,
            innovation_threshold: 25.0,
            innovation_burst_count: 5,
            failsafe_burst_count: 5,
            cusum_drift: 1.0,
            cusum_threshold: 25.0,
            glrt_window_samples: 32,
            glrt_false_alarm_rate: 0.001,
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
    cusum_score: f64,
    /// Phase-5.B.4 — per-sensor windowed GLRT detectors. Lazily
    /// constructed when `detector_kind = WindowedMeanShiftGlrt`; left
    /// `None` for the legacy detector kinds so existing scenarios stay
    /// byte-stable.
    glrt_gnss: Option<WindowedMeanShiftGlrt<6>>,
    glrt_baro: Option<WindowedMeanShiftGlrt<1>>,
    glrt_mag: Option<WindowedMeanShiftGlrt<3>>,
    /// Step counter incremented each time the FDIR job runs. Feeds
    /// the GLRT detectors' jump-time estimates.
    glrt_step: u64,
}

impl FdirJob {
    /// Constructs the FDIR job with the given parameters.
    ///
    /// # Panics
    ///
    /// Panics when `detector_kind = WindowedMeanShiftGlrt` is selected
    /// with `glrt_window_samples = 0` or
    /// `glrt_false_alarm_rate ∉ (0, 1)`. Callers are expected to
    /// validate these at scenario-load time
    /// ([`crate::params::FdirParams`] is populated from
    /// `[fc.fdir.detector]` whose own validator rejects invalid
    /// values).
    // The three `expect()` calls below are documented as the panic
    // contract; the `[fc.fdir.detector]` validator at scenario-load
    // time guarantees the GLRT params are valid before reaching here.
    #[allow(clippy::expect_used)]
    #[must_use]
    pub fn new(params: FdirParams) -> Self {
        let mut job = Self {
            name: "fdir.tick",
            params,
            innovation_burst: 0,
            failsafe_burst: 0,
            triggered_mask: 0,
            ticks_since_trip: 0,
            cusum_score: 0.0,
            glrt_gnss: None,
            glrt_baro: None,
            glrt_mag: None,
            glrt_step: 0,
        };
        if matches!(
            job.params.detector_kind,
            DetectorKind::WindowedMeanShiftGlrt
        ) {
            let w = job.params.glrt_window_samples as usize;
            let alpha = job.params.glrt_false_alarm_rate;
            job.glrt_gnss = Some(
                WindowedMeanShiftGlrt::<6>::new(w, alpha)
                    .expect("scenario validator must guarantee valid GLRT params"),
            );
            job.glrt_baro = Some(
                WindowedMeanShiftGlrt::<1>::new(w, alpha)
                    .expect("scenario validator must guarantee valid GLRT params"),
            );
            job.glrt_mag = Some(
                WindowedMeanShiftGlrt::<3>::new(w, alpha)
                    .expect("scenario validator must guarantee valid GLRT params"),
            );
        }
        job
    }

    fn estimator_mask(&self, est: EstimatorStatus) -> (u64, f64, u64) {
        let mut mask = 0_u64;
        if est.imu_chi2 > self.params.innovation_threshold {
            mask |= FDIR_BIT_IMU;
        }
        if est.baro_chi2 > self.params.innovation_threshold {
            mask |= FDIR_BIT_BARO;
        }
        if est.gnss_chi2 > self.params.innovation_threshold {
            mask |= FDIR_BIT_GNSS;
        }
        if est.mag_chi2 > self.params.innovation_threshold {
            mask |= FDIR_BIT_MAG;
        }
        if est.dead_reckoning {
            mask |= FDIR_BIT_ESTIMATOR_DEAD_RECKONING;
        }
        let mut max_chi2 = est.imu_chi2;
        let mut max_mask = FDIR_BIT_IMU;
        for (chi2, bit) in [
            (est.baro_chi2, FDIR_BIT_BARO),
            (est.gnss_chi2, FDIR_BIT_GNSS),
            (est.mag_chi2, FDIR_BIT_MAG),
        ] {
            if chi2 > max_chi2 {
                max_chi2 = chi2;
                max_mask = bit;
            }
        }
        if max_chi2 <= 0.0 {
            max_mask = 0;
        }
        (mask, max_chi2, max_mask)
    }

    fn failsafe_mask(flags: FailsafeFlags) -> u64 {
        let mut mask = 0_u64;
        if flags.imu_unhealthy {
            mask |= FDIR_BIT_IMU;
        }
        if flags.baro_unhealthy {
            mask |= FDIR_BIT_BARO;
        }
        if flags.gnss_unhealthy {
            mask |= FDIR_BIT_GNSS;
        }
        if flags.mag_unhealthy {
            mask |= FDIR_BIT_MAG;
        }
        if flags.scheduler_overrun {
            mask |= FDIR_BIT_SCHEDULER_OVERRUN;
        }
        if flags.estimator_dead_reckoning {
            mask |= FDIR_BIT_ESTIMATOR_DEAD_RECKONING;
        }
        mask
    }
}

impl Job for FdirJob {
    fn name(&self) -> &'static str {
        self.name
    }

    #[allow(clippy::too_many_lines)] // Phase-5.B.4: GLRT dispatch arm grew the function
    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        self.glrt_step = self.glrt_step.saturating_add(1);
        let mut current_mask = 0_u64;
        let mut max_chi2 = 0.0_f64;
        let mut max_chi2_mask = 0_u64;
        let mut latest_estimator: Option<EstimatorStatus> = None;
        if let Ok(Some((est, _))) = ctx.bus.latest::<EstimatorStatus>() {
            latest_estimator = Some(est);
            let (mask, statistic, statistic_mask) = self.estimator_mask(est);
            current_mask |= mask;
            max_chi2 = statistic;
            max_chi2_mask = statistic_mask;
        }
        if let Ok(Some((flags, _))) = ctx.bus.latest::<FailsafeFlags>() {
            current_mask |= Self::failsafe_mask(flags);
        }
        if let Ok(Some((actuator, _))) = ctx.bus.latest::<ActuatorCommand>()
            && actuator.saturated
        {
            current_mask |= FDIR_BIT_AUTOPILOT_SATURATION;
        }
        if let Ok(Some((autopilot, _))) = ctx.bus.latest::<AutopilotStatus>()
            && autopilot.differential_flatness_reference_suppressed
        {
            current_mask |= FDIR_BIT_AUTOPILOT_REFERENCE_SUPPRESSED;
        }

        match self.params.detector_kind {
            DetectorKind::BurstCounter => {
                if max_chi2 > self.params.innovation_threshold {
                    self.innovation_burst = self.innovation_burst.saturating_add(1);
                } else {
                    self.innovation_burst = 0;
                }
                if current_mask != 0 {
                    self.failsafe_burst = self.failsafe_burst.saturating_add(1);
                } else {
                    self.failsafe_burst = 0;
                }
                if self.innovation_burst >= self.params.innovation_burst_count
                    || self.failsafe_burst >= self.params.failsafe_burst_count
                {
                    self.triggered_mask |= current_mask;
                }
            }
            DetectorKind::SingleSampleGlrt => {
                let fault_mask = single_sample_glrt_fault_mask(
                    current_mask,
                    max_chi2,
                    max_chi2_mask,
                    self.params.innovation_threshold,
                );
                if fault_mask != 0 {
                    self.triggered_mask |= fault_mask;
                }
            }
            DetectorKind::Cusum => {
                self.cusum_score = (self.cusum_score + max_chi2 - self.params.cusum_drift).max(0.0);
                if current_mask != 0 || self.cusum_score > self.params.cusum_threshold {
                    self.triggered_mask |= current_mask | max_chi2_mask;
                }
            }
            DetectorKind::WindowedMeanShiftGlrt => {
                if let Some(est) = latest_estimator {
                    let glrt_step = self.glrt_step;
                    let mut glrt_mask = 0_u64;
                    let mut diagnostic: Option<FdirGlrtDiagnostic> = None;
                    if let Some(det) = self.glrt_gnss.as_mut()
                        && est.gnss_updated_this_tick
                        && let StepOutcome::Tripped {
                            estimated_jump_step,
                            statistic,
                            threshold,
                        } = det.step(est.gnss_innovation_whitened, glrt_step)
                    {
                        glrt_mask |= FDIR_BIT_GNSS;
                        diagnostic = Some(FdirGlrtDiagnostic {
                            sensor_mask: FDIR_BIT_GNSS,
                            estimated_jump_step,
                            statistic,
                            threshold,
                        });
                    }
                    if let Some(det) = self.glrt_baro.as_mut()
                        && est.baro_updated_this_tick
                        && let StepOutcome::Tripped {
                            estimated_jump_step,
                            statistic,
                            threshold,
                        } = det.step([est.baro_innovation_whitened], glrt_step)
                    {
                        glrt_mask |= FDIR_BIT_BARO;
                        // Latest sensor wins on the diagnostic slot.
                        diagnostic = Some(FdirGlrtDiagnostic {
                            sensor_mask: FDIR_BIT_BARO,
                            estimated_jump_step,
                            statistic,
                            threshold,
                        });
                    }
                    if let Some(det) = self.glrt_mag.as_mut()
                        && est.mag_updated_this_tick
                        && let StepOutcome::Tripped {
                            estimated_jump_step,
                            statistic,
                            threshold,
                        } = det.step(est.mag_innovation_whitened, glrt_step)
                    {
                        glrt_mask |= FDIR_BIT_MAG;
                        diagnostic = Some(FdirGlrtDiagnostic {
                            sensor_mask: FDIR_BIT_MAG,
                            estimated_jump_step,
                            statistic,
                            threshold,
                        });
                    }
                    if glrt_mask != 0 {
                        self.triggered_mask |= current_mask | glrt_mask;
                    }
                    if let Some(diag) = diagnostic {
                        let _ = ctx.bus.publish(diag);
                    }
                }
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

fn single_sample_glrt_fault_mask(
    current_mask: u64,
    innovation_chi2: f64,
    innovation_mask: u64,
    innovation_threshold: f64,
) -> u64 {
    // For a zero-mean Gaussian innovation with covariance S, the
    // unconstrained mean-shift GLRT has 2 log Lambda = r' S^-1 r,
    // i.e. the same chi-square statistic published by the estimator.
    // We threshold a single sample of that statistic; the windowed
    // mean-shift estimator from Willsky 1976 is Phase-5 work.
    let statistic = innovation_chi2.max(0.0);
    let innovation_fault = if statistic > innovation_threshold {
        innovation_mask
    } else {
        0
    };
    current_mask | innovation_fault
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use openbmp_core::{SimTime, StepIndex};

    use super::*;
    use crate::bus::Bus;
    use crate::clock::FixedClock;
    use crate::scheduler::{Job, JobContext};

    fn bus_with_fdir_topics() -> Bus {
        let bus = Bus::new();
        bus.register::<EstimatorStatus>().unwrap();
        bus.register::<FailsafeFlags>().unwrap();
        bus.register::<ActuatorCommand>().unwrap();
        bus.register::<AutopilotStatus>().unwrap();
        bus.register::<FdirStatus>().unwrap();
        bus
    }

    fn run_once(job: &mut FdirJob, bus: &Bus) -> FdirStatus {
        let clock = FixedClock::new(SimTime::from_seconds(0.001), StepIndex::new(1));
        job.run(&JobContext { bus, clock: &clock }).unwrap();
        bus.latest::<FdirStatus>().unwrap().unwrap().0
    }

    #[test]
    fn single_sample_glrt_latches_explicit_fault_bits() {
        let bus = bus_with_fdir_topics();
        bus.publish(EstimatorStatus {
            gnss_chi2: 30.0,
            dead_reckoning: true,
            ..EstimatorStatus::default()
        })
        .unwrap();
        bus.publish(ActuatorCommand {
            saturated: true,
            ..ActuatorCommand::default()
        })
        .unwrap();
        let mut job = FdirJob::new(FdirParams {
            detector_kind: DetectorKind::SingleSampleGlrt,
            innovation_threshold: 25.0,
            ..FdirParams::default()
        });

        let status = run_once(&mut job, &bus);

        assert!(status.triggered);
        assert_ne!(status.tripped_mask & FDIR_BIT_GNSS, 0);
        assert_ne!(status.tripped_mask & FDIR_BIT_ESTIMATOR_DEAD_RECKONING, 0);
        assert_ne!(status.tripped_mask & FDIR_BIT_AUTOPILOT_SATURATION, 0);
    }

    #[test]
    fn single_sample_glrt_latches_autopilot_reference_suppression() {
        let bus = bus_with_fdir_topics();
        bus.publish(AutopilotStatus {
            differential_flatness_active: true,
            differential_flatness_reference_suppressed: true,
        })
        .unwrap();
        let mut job = FdirJob::new(FdirParams {
            detector_kind: DetectorKind::SingleSampleGlrt,
            ..FdirParams::default()
        });

        let status = run_once(&mut job, &bus);

        assert!(status.triggered);
        assert_ne!(
            status.tripped_mask & FDIR_BIT_AUTOPILOT_REFERENCE_SUPPRESSED,
            0
        );
    }

    #[test]
    fn single_sample_glrt_assigns_max_innovation_source_bit() {
        let mask = single_sample_glrt_fault_mask(0, 30.0, FDIR_BIT_BARO, 25.0);
        assert_eq!(mask, FDIR_BIT_BARO);
    }

    #[test]
    fn cusum_accumulates_subthreshold_innovation_and_assigns_source_bit() {
        let bus = bus_with_fdir_topics();
        bus.publish(EstimatorStatus {
            mag_chi2: 4.0,
            ..EstimatorStatus::default()
        })
        .unwrap();
        let mut job = FdirJob::new(FdirParams {
            detector_kind: DetectorKind::Cusum,
            innovation_threshold: 25.0,
            cusum_drift: 1.0,
            cusum_threshold: 5.0,
            ..FdirParams::default()
        });

        let first = run_once(&mut job, &bus);
        let second = run_once(&mut job, &bus);

        assert!(!first.triggered);
        assert!(second.triggered);
        assert_ne!(second.tripped_mask & FDIR_BIT_MAG, 0);
    }
}
