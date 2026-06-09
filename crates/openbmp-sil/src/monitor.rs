//! Estimate-vs-truth SIL monitor.
//!
//! This is the observe half of a real SIL bench. As the flight controller
//! runs in the loop, [`EstimateVsTruthMonitor`] compares what the FC
//! *estimates* (its published nav solution, estimator health, and FDIR
//! status) against the simulated *truth* the runner already computes, and
//! records the **residuals** — never the raw truth trajectory.
//!
//! Forward-only by construction: only estimate-vs-truth error *norms* and
//! vehicle-intrinsic / FDIR flags are representable in
//! [`SilObservationSample`]; there is no field for a truth ECI series,
//! aimpoint, impact point, downrange, or miss distance, so the evidence
//! cannot feed an offline targeting fit.

use nalgebra::{Quaternion, UnitQuaternion};
use openbmp_core::StepIndex;
use openbmp_runner::sil::{FcObservation, SilMonitor};
use openbmp_sensors::SensorTruth;
use serde::{Deserialize, Serialize};

/// One decimated per-tick record of the flight controller's estimate-vs-truth
/// residuals and intrinsic health, for human review and downstream checks.
///
/// Every accuracy field is `Option` because the corresponding estimate may
/// not have been published yet; [`Self::estimator_active_mode`] is
/// additionally `None` for a single-lane estimator that does not run the IMM
/// mode filter.
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct SilObservationSample {
    /// Kernel step index of this observation.
    pub step: u64,
    /// Simulation time (s) of this observation.
    pub time_s: f64,
    /// `‖position_estimate − position_truth‖` (m, ECI).
    pub pos_err_m: Option<f64>,
    /// `‖velocity_estimate − velocity_truth‖` (m/s, ECI).
    pub vel_err_m_s: Option<f64>,
    /// Geodesic attitude error angle (rad) between the estimated and true
    /// body→ECI rotation.
    pub att_err_rad: Option<f64>,
    /// GNSS normalised estimation error squared `‖ν̃‖²` from the
    /// Cholesky-whitened innovation, when a GNSS update was evaluated this
    /// tick; `None` otherwise.
    pub nees: Option<f64>,
    /// Estimator dead-reckoning flag (no corrective fix within timeout).
    pub dead_reckoning: bool,
    /// Estimator innovation-gate rejection flag for this tick.
    pub innovation_rejected: bool,
    /// Most recent GNSS innovation chi-square ratio.
    pub gnss_chi2: f64,
    /// Most recent baro innovation chi-square ratio.
    pub baro_chi2: f64,
    /// Most recent magnetometer innovation chi-square ratio.
    pub mag_chi2: f64,
    /// FDIR triggered flag.
    pub fdir_triggered: bool,
    /// FDIR tripped-detector bitmask.
    pub fdir_tripped_mask: u64,
    /// IMM active-mode index; `None` for a single-lane estimator.
    pub estimator_active_mode: Option<u8>,
}

/// Full-fidelity aggregate over **every** tick (not decimated), so accuracy
/// bounds and FDIR-response checks stay honest regardless of the sample
/// decimation factor.
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct SilObservationSummary {
    /// Number of observed ticks.
    pub ticks: u64,
    /// Maximum position residual (m) over the run.
    pub max_pos_err_m: Option<f64>,
    /// Maximum velocity residual (m/s) over the run.
    pub max_vel_err_m_s: Option<f64>,
    /// Maximum attitude error (rad) over the run.
    pub max_att_err_rad: Option<f64>,
    /// Maximum GNSS NEES `‖ν̃‖²` over the run.
    pub max_nees: Option<f64>,
    /// Count of ticks the estimator was dead-reckoning.
    pub dead_reckoning_ticks: u64,
    /// Count of ticks an innovation was gate-rejected.
    pub innovation_rejected_ticks: u64,
    /// Count of ticks FDIR was triggered.
    pub fdir_trip_ticks: u64,
    /// Union of all FDIR tripped-detector masks over the run.
    pub fdir_tripped_mask_union: u64,
    /// First step at which FDIR triggered, if ever.
    pub first_fdir_trip_step: Option<u64>,
}

/// Estimate-vs-truth observation evidence: a full-fidelity summary plus a
/// decimated per-tick sample series.
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct SilObservationReport {
    /// Sample decimation factor: every `decimation`-th tick is recorded in
    /// [`Self::samples`]. `1` records every tick.
    pub decimation: u64,
    /// Full-fidelity aggregate over every tick.
    pub summary: SilObservationSummary,
    /// Decimated per-tick residual/flag samples.
    pub samples: Vec<SilObservationSample>,
}

/// A [`SilMonitor`] that records estimate-vs-truth residuals each tick.
#[derive(Clone, Debug)]
pub struct EstimateVsTruthMonitor {
    decimation: u64,
    tick: u64,
    summary: SilObservationSummary,
    samples: Vec<SilObservationSample>,
}

impl EstimateVsTruthMonitor {
    /// Create a monitor that records every `decimation`-th tick in the
    /// sample series (the full-fidelity summary always covers every tick).
    /// A `decimation` of `0` is treated as `1` (record every tick).
    #[must_use]
    pub fn new(decimation: u64) -> Self {
        Self {
            decimation: decimation.max(1),
            tick: 0,
            summary: SilObservationSummary::default(),
            samples: Vec::new(),
        }
    }

    /// Borrow the decimated sample series.
    #[must_use]
    pub fn samples(&self) -> &[SilObservationSample] {
        &self.samples
    }

    /// Borrow the full-fidelity summary.
    #[must_use]
    pub const fn summary(&self) -> &SilObservationSummary {
        &self.summary
    }

    /// Consume the monitor into a serialisable observation report.
    #[must_use]
    pub fn into_report(self) -> SilObservationReport {
        SilObservationReport {
            decimation: self.decimation,
            summary: self.summary,
            samples: self.samples,
        }
    }
}

/// Geodesic attitude error angle (rad) between an estimated `body→ECI`
/// rotation (as an `[x, y, z, w]` quaternion) and a truth `eci→body`
/// rotation.
///
/// The estimate is `q_est` (`body→ECI`) and the true `body→ECI` rotation is
/// `truth_eci_to_body⁻¹`, so the error rotation
/// `q_err = q_est · (truth_eci_to_body⁻¹)⁻¹ = q_est · truth_eci_to_body`
/// is the identity (angle `0`) when the estimate matches truth. Composing
/// same-direction rotations is essential: mixing a `body→ECI` estimate with
/// an `eci→body` truth would not vanish at perfect tracking.
fn attitude_error_rad(
    q_body_to_eci_xyzw: [f64; 4],
    truth_eci_to_body: &UnitQuaternion<f64>,
) -> f64 {
    let [x, y, z, w] = q_body_to_eci_xyzw;
    let q_est = UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z));
    (q_est * truth_eci_to_body).angle()
}

/// Update `slot` to the larger of itself and `value` (treating `None` as
/// "no value yet").
fn accumulate_max(slot: &mut Option<f64>, value: f64) {
    *slot = Some(slot.map_or(value, |current| current.max(value)));
}

impl SilMonitor for EstimateVsTruthMonitor {
    fn observe(&mut self, step: StepIndex, truth: &SensorTruth, observation: &FcObservation) {
        let step_value = step.value();

        // ---- Residuals (estimate − truth). Truth is consumed into scalar
        // norms and never stored. ----
        let pos_err_m = observation
            .position
            .map(|p| (p.position_eci_m - truth.position_eci.vector).norm());
        let vel_err_m_s = observation
            .position
            .map(|p| (p.velocity_eci_m_s - truth.velocity_eci.vector).norm());
        let att_err_rad = observation
            .attitude
            .map(|a| attitude_error_rad(a.q_body_to_eci_xyzw, &truth.attitude_eci_to_body));

        // ---- Estimator health ----
        let (nees, dead_reckoning, innovation_rejected, gnss_chi2, baro_chi2, mag_chi2) =
            observation.estimator.map_or(
                (None, false, false, 0.0, 0.0, 0.0),
                |estimator| {
                    let nees = estimator.gnss_updated_this_tick.then(|| {
                        estimator
                            .gnss_innovation_whitened
                            .iter()
                            .map(|component| component * component)
                            .sum::<f64>()
                    });
                    (
                        nees,
                        estimator.dead_reckoning,
                        estimator.innovation_rejected,
                        estimator.gnss_chi2,
                        estimator.baro_chi2,
                        estimator.mag_chi2,
                    )
                },
            );

        // ---- FDIR / mode ----
        let (fdir_triggered, fdir_tripped_mask) = observation
            .fdir
            .map_or((false, 0), |fdir| (fdir.triggered, fdir.tripped_mask));
        let estimator_active_mode = observation.mode.map(|mode| mode.active_mode);

        // ---- Full-fidelity summary (every tick) ----
        self.summary.ticks += 1;
        if let Some(value) = pos_err_m {
            accumulate_max(&mut self.summary.max_pos_err_m, value);
        }
        if let Some(value) = vel_err_m_s {
            accumulate_max(&mut self.summary.max_vel_err_m_s, value);
        }
        if let Some(value) = att_err_rad {
            accumulate_max(&mut self.summary.max_att_err_rad, value);
        }
        if let Some(value) = nees {
            accumulate_max(&mut self.summary.max_nees, value);
        }
        if dead_reckoning {
            self.summary.dead_reckoning_ticks += 1;
        }
        if innovation_rejected {
            self.summary.innovation_rejected_ticks += 1;
        }
        if fdir_triggered {
            self.summary.fdir_trip_ticks += 1;
            self.summary.fdir_tripped_mask_union |= fdir_tripped_mask;
            if self.summary.first_fdir_trip_step.is_none() {
                self.summary.first_fdir_trip_step = Some(step_value);
            }
        }

        // ---- Decimated sample series (every Nth tick) ----
        let record = self.tick.is_multiple_of(self.decimation);
        self.tick += 1;
        if record {
            self.samples.push(SilObservationSample {
                step: step_value,
                time_s: truth.time.as_seconds(),
                pos_err_m,
                vel_err_m_s,
                att_err_rad,
                nees,
                dead_reckoning,
                innovation_rejected,
                gnss_chi2,
                baro_chi2,
                mag_chi2,
                fdir_triggered,
                fdir_tripped_mask,
                estimator_active_mode,
            });
        }
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    #[test]
    fn attitude_error_is_zero_for_matching_estimate() {
        // Truth eci->body is some non-trivial rotation; the estimate's
        // body->eci is its inverse, i.e. a perfect estimate. The error
        // must be exactly 0.
        let truth_eci_to_body =
            UnitQuaternion::from_euler_angles(0.3, -0.7, 1.1);
        let truth_body_to_eci = truth_eci_to_body.inverse();
        let q = truth_body_to_eci.into_inner();
        let xyzw = [q.i, q.j, q.k, q.w];

        let err = attitude_error_rad(xyzw, &truth_eci_to_body);
        assert!(
            err.abs() < 1.0e-12,
            "matching estimate must yield zero attitude error, got {err}"
        );
    }

    #[test]
    fn attitude_error_recovers_known_angle() {
        // Truth identity (eci==body); estimate body->eci is a 0.2 rad yaw.
        // The geodesic error must be 0.2 rad.
        let truth_eci_to_body = UnitQuaternion::identity();
        let est_body_to_eci =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.2);
        let q = est_body_to_eci.into_inner();
        let err = attitude_error_rad([q.i, q.j, q.k, q.w], &truth_eci_to_body);
        assert!(
            (err - 0.2).abs() < 1.0e-9,
            "expected 0.2 rad attitude error, got {err}"
        );
    }

    #[test]
    fn decimation_records_every_nth_tick_but_summary_covers_all() {
        let mut monitor = EstimateVsTruthMonitor::new(3);
        let truth = SensorTruth {
            position_eci: openbmp_core::Position3::new(1.0, 2.0, 3.0),
            velocity_eci: openbmp_core::Velocity3::new(0.0, 0.0, 0.0),
            attitude_eci_to_body: UnitQuaternion::identity(),
            angular_velocity_body_rad_s: Vector3::zeros(),
            angular_acceleration_body_rad_s2: Vector3::zeros(),
            specific_force_body_m_s2: Vector3::zeros(),
            static_pressure_pa: 0.0,
            altitude_geometric_m: 0.0,
            magnetic_field_body_nt: Vector3::zeros(),
            time: openbmp_core::SimTime::from_seconds(0.0),
        };
        let observation = FcObservation::default();
        for step in 0..10u64 {
            monitor.observe(StepIndex::new(step), &truth, &observation);
        }
        // Summary covers every tick; samples are decimated by 3 (0,3,6,9).
        assert_eq!(monitor.summary().ticks, 10);
        assert_eq!(monitor.samples().len(), 4);
        let report = monitor.into_report();
        assert_eq!(report.decimation, 3);
    }
}
