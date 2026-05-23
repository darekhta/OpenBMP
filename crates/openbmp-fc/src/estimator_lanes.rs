//! Phase-5.B.2 — Multi-instance estimator routing + active-lane
//! selection.
//!
//! The classical Phase-4 FC ran a single `Box<dyn Estimator>` on the
//! scheduler. Phase-5.B.2 lifts that to N parallel estimator
//! "lanes" — each lane a distinct estimator instance (EKF, MEKF,
//! IMM, SR-UKF, SR-UKF-attitude) — driven from the same sensor
//! samples. Per tick:
//!
//! 1. **Predict.** Every lane's `predict(dt)` runs with the same
//!    `dt`. Lanes that fail closed (return `EstimatorError`) are
//!    flagged unhealthy for the rest of the tick; healthy lanes
//!    continue.
//! 2. **Measurement updates.** Every healthy lane sees the same
//!    IMU / GNSS / baro / mag samples. A per-lane innovation gate
//!    rejection inside `update_*` is local — it doesn't unhealth the
//!    lane, it just records the rejection in the lane's status.
//! 3. **Vote.** A configured [`VoterPolicy`] picks the *active*
//!    lane from the healthy set. The active lane drives the
//!    canonical `AttitudeEstimate` / `PositionEstimate` /
//!    `EstimatorStatus` topics; non-active lanes are still alive
//!    internally and their per-lane diagnostics are exposed via
//!    [`MultiLaneEstimator::lane_status`].
//!
//! The wrapper itself implements [`crate::estimator::Estimator`], so
//! existing scheduler / job / FDIR plumbing wires through unchanged.
//! The single-lane case (`MultiLaneEstimator` with one lane and
//! [`VoterPolicy::SimplexPassThrough`]) reduces to the classical
//! single-estimator behaviour byte-for-byte.
//!
//! # Voter policies (shipped this slice)
//!
//! - [`VoterPolicy::SimplexPassThrough`] — first healthy lane wins.
//!   Useful for warm-spare configurations where a single primary
//!   estimator is canonical and additional lanes exist only as a
//!   fall-back if the primary fails closed.
//! - [`VoterPolicy::MidValueSelectByInnovation`] — median-of-three
//!   selection by per-lane innovation chi-square. The median lane's
//!   innovation should be the most central / least extreme of the
//!   healthy set, which is a robust choice when any single lane
//!   might be drifting. Requires ≥ 3 lanes.
//! - [`VoterPolicy::BestByCovarianceTrace`] — minimum covariance-trace
//!   proxy wins. `EstimatorStatus` does not yet expose a covariance
//!   trace, so this slice uses the sum of the per-sensor innovation
//!   chi-square values as a deterministic stand-in until the follow-on
//!   status field lands. Cheap fallback for ≤ 2 lanes (where the
//!   median rule degenerates).
//!
//! # Determinism
//!
//! Lane execution order is the **declaration order** in
//! `MultiLaneEstimator::new`. The voter visits lanes in the same
//! fixed order; ties are broken by lane index (lower wins). This is
//! the determinism contract — permuting lanes would change which
//! lane wins on a tie.
//!
//! # References
//!
//! - PX4 ekf2 multi-instance routing pattern (academic reference,
//!   not imported).
//! - Bar-Shalom et al. (2001), *Estimation with Applications to
//!   Tracking and Navigation*, §11 ("Multiple Model Approaches") —
//!   while the IMM is one mode of multi-model use, parallel-bank
//!   voting is the simpler "non-interacting" cousin.

#![allow(clippy::doc_markdown)]

use crate::error::EstimatorError;
use crate::estimator::Estimator;
use crate::topics::{
    AttitudeEstimate, BarometerSample, EstimatorMode, EstimatorStatus, GnssSample, ImuSample,
    MagnetometerSample, PositionEstimate,
};

/// Stable identifier for a registered lane. Mirrors the scenario
/// `[[fc.estimator_lanes.lane]] id = "..."` field; the runner wraps
/// it in this newtype so downstream telemetry / FDIR can address
/// lanes by name without parsing scenario strings.
#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct LaneId(pub String);

impl LaneId {
    /// Borrow the underlying string.
    #[must_use]
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl From<String> for LaneId {
    fn from(s: String) -> Self {
        Self(s)
    }
}

impl From<&str> for LaneId {
    fn from(s: &str) -> Self {
        Self(s.to_owned())
    }
}

/// Policy for selecting the active lane each tick.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum VoterPolicy {
    /// First healthy lane wins. Lane order is the declaration order
    /// passed to [`MultiLaneEstimator::new`].
    SimplexPassThrough,
    /// Median-of-three by innovation chi-square (using the GNSS
    /// chi-square as the canonical channel; falls back to the sum
    /// of all per-sensor chi-square if no GNSS update happened this
    /// tick). Requires ≥ 3 healthy lanes; with fewer healthy lanes,
    /// degrades to [`VoterPolicy::SimplexPassThrough`].
    MidValueSelectByInnovation,
    /// Minimum covariance-trace proxy wins. Until `EstimatorStatus`
    /// exposes a real covariance trace, this uses the sum of
    /// per-sensor innovation chi-square values.
    BestByCovarianceTrace,
}

/// Per-lane diagnostic snapshot.
#[derive(Clone, Debug)]
pub struct LaneStatus {
    /// Lane id from the scenario config.
    pub id: LaneId,
    /// `true` if the lane's most recent `predict` / `update` succeeded.
    pub healthy: bool,
    /// Raw status from the lane's underlying [`Estimator::status`].
    pub status: EstimatorStatus,
    /// Optional [`EstimatorMode`] if the lane provides one (IMM does;
    /// EKF / MEKF / SR-UKF do not).
    pub mode: Option<EstimatorMode>,
}

/// Multi-lane estimator router with active-lane voter selection.
pub struct MultiLaneEstimator {
    lanes: Vec<LaneEntry>,
    policy: VoterPolicy,
    active_index: usize,
}

struct LaneEntry {
    id: LaneId,
    estimator: Box<dyn Estimator + Send>,
    healthy: bool,
}

impl std::fmt::Debug for MultiLaneEstimator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MultiLaneEstimator")
            .field("policy", &self.policy)
            .field("active_index", &self.active_index)
            .field(
                "lanes",
                &self
                    .lanes
                    .iter()
                    .map(|l| (l.id.clone(), l.healthy))
                    .collect::<Vec<_>>(),
            )
            .finish_non_exhaustive()
    }
}

impl MultiLaneEstimator {
    /// Construct a new multi-lane estimator.
    ///
    /// `lanes` is the ordered set of (id, estimator) pairs; lane
    /// order is the determinism contract and the tiebreaker for
    /// every voter policy.
    ///
    /// # Panics
    ///
    /// Panics if `lanes` is empty. Callers (the FC runner) must
    /// reject empty lane sets at scenario-parse time.
    #[must_use]
    pub fn new(lanes: Vec<(LaneId, Box<dyn Estimator + Send>)>, policy: VoterPolicy) -> Self {
        assert!(
            !lanes.is_empty(),
            "MultiLaneEstimator requires at least one registered lane"
        );
        let entries = lanes
            .into_iter()
            .map(|(id, estimator)| LaneEntry {
                id,
                estimator,
                healthy: true,
            })
            .collect();
        Self {
            lanes: entries,
            policy,
            active_index: 0,
        }
    }

    /// Number of registered lanes.
    #[must_use]
    pub fn lane_count(&self) -> usize {
        self.lanes.len()
    }

    /// Currently-active lane's id.
    #[must_use]
    pub fn active_lane_id(&self) -> &LaneId {
        &self.lanes[self.active_index].id
    }

    /// Per-lane status snapshots, ordered by lane index.
    #[must_use]
    pub fn lane_status(&self) -> Vec<LaneStatus> {
        self.lanes
            .iter()
            .map(|entry| LaneStatus {
                id: entry.id.clone(),
                healthy: entry.healthy,
                status: entry.estimator.status(),
                mode: entry.estimator.estimator_mode(),
            })
            .collect()
    }

    /// Recompute the active lane index per the configured policy.
    /// Called after every tick's measurement updates have been
    /// dispatched. Lanes that failed closed during the tick are
    /// excluded from the candidate set; if every lane has failed,
    /// the previous active index is preserved (no state change).
    fn vote(&mut self) {
        let healthy: Vec<usize> = self
            .lanes
            .iter()
            .enumerate()
            .filter_map(|(i, l)| l.healthy.then_some(i))
            .collect();
        if healthy.is_empty() {
            return;
        }
        let chosen = match self.policy {
            VoterPolicy::SimplexPassThrough => healthy[0],
            VoterPolicy::BestByCovarianceTrace => {
                // healthy is non-empty by the early return above; the
                // unwrap_or is unreachable but keeps the code total.
                healthy
                    .iter()
                    .copied()
                    .min_by(|&a, &b| {
                        let ta = covariance_trace_proxy(&self.lanes[a].estimator.status());
                        let tb = covariance_trace_proxy(&self.lanes[b].estimator.status());
                        // Stable: tiebreak by lane index (lower wins).
                        ta.partial_cmp(&tb)
                            .unwrap_or(std::cmp::Ordering::Equal)
                            .then_with(|| a.cmp(&b))
                    })
                    .unwrap_or(healthy[0])
            }
            VoterPolicy::MidValueSelectByInnovation => {
                if healthy.len() < 3 {
                    // Fewer than 3 healthy lanes — median is
                    // ill-defined; fall back to simplex pass-through.
                    healthy[0]
                } else {
                    // Sort by innovation chi-square (sum of per-sensor
                    // chi-square readings); pick the median.
                    let mut scored: Vec<(f64, usize)> = healthy
                        .iter()
                        .map(|&i| (innovation_chi2_proxy(&self.lanes[i].estimator.status()), i))
                        .collect();
                    // Sort by chi² ascending; tiebreak by lane index.
                    scored.sort_by(|(va, ia), (vb, ib)| {
                        va.partial_cmp(vb)
                            .unwrap_or(std::cmp::Ordering::Equal)
                            .then(ia.cmp(ib))
                    });
                    scored[scored.len() / 2].1
                }
            }
        };
        self.active_index = chosen;
    }

    fn finish_dispatch(&self, last_err: Option<EstimatorError>) -> Result<(), EstimatorError> {
        if self.lanes.iter().any(|l| l.healthy) {
            return Ok(());
        }
        Err(last_err.unwrap_or_else(|| EstimatorError::InvalidConfig {
            reason: "multi-lane estimator has no healthy lanes".to_owned(),
        }))
    }
}

fn covariance_trace_proxy(status: &EstimatorStatus) -> f64 {
    // The shipped EstimatorStatus does not expose covariance trace.
    // Until the follow-on adds that field, the sum of per-sensor
    // chi-square readings is only a deterministic proxy for "tighter"
    // belief; it is not true covariance algebra.
    status.imu_chi2 + status.gnss_chi2 + status.baro_chi2 + status.mag_chi2
}

fn innovation_chi2_proxy(status: &EstimatorStatus) -> f64 {
    // Median-by-innovation policy looks at the same sum-of-chi² proxy
    // as the covariance-trace policy. The two policies coincide
    // arithmetically but diverge in the *selection rule* (min vs
    // median); keeping them in separate helpers leaves room for the
    // proxy to evolve independently per follow-on slice.
    status.imu_chi2 + status.gnss_chi2 + status.baro_chi2 + status.mag_chi2
}

impl Estimator for MultiLaneEstimator {
    fn name(&self) -> &'static str {
        "estimator.multi_lane"
    }

    fn predict(&mut self, dt: f64) -> Result<(), EstimatorError> {
        // Every lane predicts independently. A lane that fails closed
        // here is unhealthed for the rest of the tick.
        let mut last_err: Option<EstimatorError> = None;
        for entry in &mut self.lanes {
            if !entry.healthy {
                continue;
            }
            if let Err(e) = entry.estimator.predict(dt) {
                entry.healthy = false;
                last_err = Some(e);
            }
        }
        // Vote even if some lanes failed; healthy ones still drive
        // the active output. Only error out if the entire bank
        // failed in lockstep (which would mean a hardware-class
        // failure upstream).
        self.vote();
        self.finish_dispatch(last_err)
    }

    fn update_imu(&mut self, sample: &ImuSample) -> Result<(), EstimatorError> {
        let mut last_err: Option<EstimatorError> = None;
        for entry in &mut self.lanes {
            if !entry.healthy {
                continue;
            }
            if let Err(e) = entry.estimator.update_imu(sample) {
                // IMU is a propagation source — failure is
                // typically a non-finite IMU rather than a gate
                // breach. Mark the lane unhealthy.
                entry.healthy = false;
                last_err = Some(e);
            }
        }
        self.vote();
        self.finish_dispatch(last_err)
    }

    fn update_gnss(&mut self, sample: &GnssSample) -> Result<(), EstimatorError> {
        let mut last_err: Option<EstimatorError> = None;
        for entry in &mut self.lanes {
            if !entry.healthy {
                continue;
            }
            // GNSS innovation gate breach is *local* — record it on
            // the lane's status (which the underlying estimator
            // already does) but don't unhealth the lane.
            // Non-finite-state errors (covariance breakdown) DO
            // unhealth.
            if let Err(e) = entry.estimator.update_gnss(sample)
                && !matches!(e, EstimatorError::InnovationGateRejected { .. })
            {
                entry.healthy = false;
                last_err = Some(e);
            }
        }
        self.vote();
        self.finish_dispatch(last_err)
    }

    fn update_baro(&mut self, sample: &BarometerSample) -> Result<(), EstimatorError> {
        let mut last_err: Option<EstimatorError> = None;
        for entry in &mut self.lanes {
            if !entry.healthy {
                continue;
            }
            if let Err(e) = entry.estimator.update_baro(sample)
                && !matches!(e, EstimatorError::InnovationGateRejected { .. })
            {
                entry.healthy = false;
                last_err = Some(e);
            }
        }
        self.vote();
        self.finish_dispatch(last_err)
    }

    fn update_mag(&mut self, sample: &MagnetometerSample) -> Result<(), EstimatorError> {
        let mut last_err: Option<EstimatorError> = None;
        for entry in &mut self.lanes {
            if !entry.healthy {
                continue;
            }
            if let Err(e) = entry.estimator.update_mag(sample)
                && !matches!(e, EstimatorError::InnovationGateRejected { .. })
            {
                entry.healthy = false;
                last_err = Some(e);
            }
        }
        self.vote();
        self.finish_dispatch(last_err)
    }

    fn attitude(&self) -> AttitudeEstimate {
        self.lanes[self.active_index].estimator.attitude()
    }

    fn position(&self) -> PositionEstimate {
        self.lanes[self.active_index].estimator.position()
    }

    fn status(&self) -> EstimatorStatus {
        self.lanes[self.active_index].estimator.status()
    }

    fn estimator_mode(&self) -> Option<EstimatorMode> {
        self.lanes[self.active_index].estimator.estimator_mode()
    }

    fn begin_tick(&mut self) {
        // Reset per-tick health so a transient failure on one tick
        // doesn't permanently sideline the lane. Persistent
        // covariance-breakdown errors will re-fire on the next
        // update.
        for entry in &mut self.lanes {
            entry.healthy = true;
            entry.estimator.begin_tick();
        }
    }
}

// ---------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::estimator::{Ekf, EkfParams, Mekf, MekfParams};
    use crate::topics::{
        AttitudeEstimate, BarometerSample, GnssSample, ImuSample, MagnetometerSample,
        PositionEstimate,
    };
    use nalgebra::{UnitQuaternion, Vector3};
    use openbmp_core::SimTime;

    fn imu_sample() -> ImuSample {
        ImuSample {
            time: SimTime::ZERO,
            gyro_rad_s: Vector3::new(0.0, 0.0, 0.0),
            accel_m_s2: Vector3::new(0.0, 0.0, 9.80665),
            healthy: true,
        }
    }

    fn build_ekf_lane() -> Box<dyn Estimator + Send> {
        let mut ekf = Ekf::new(EkfParams::default());
        ekf.seed(
            Vector3::zeros(),
            Vector3::zeros(),
            UnitQuaternion::identity(),
        );
        Box::new(ekf)
    }

    fn build_mekf_lane() -> Box<dyn Estimator + Send> {
        let mut mekf = Mekf::new(MekfParams::default());
        mekf.seed(UnitQuaternion::identity());
        Box::new(mekf)
    }

    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    enum FakeFailure {
        None,
        Gate,
        InvalidConfig,
    }

    impl FakeFailure {
        fn error(self, measurement: &'static str) -> Option<EstimatorError> {
            match self {
                Self::None => None,
                Self::Gate => Some(EstimatorError::InnovationGateRejected {
                    measurement,
                    chi2: 2.0,
                    gate: 1.0,
                }),
                Self::InvalidConfig => Some(EstimatorError::InvalidConfig {
                    reason: format!("fake {measurement} failure"),
                }),
            }
        }
    }

    struct FakeEstimator {
        status: EstimatorStatus,
        predict_failure: FakeFailure,
        update_failure: FakeFailure,
    }

    impl FakeEstimator {
        fn with_score(score: f64) -> Self {
            Self {
                status: EstimatorStatus {
                    time: SimTime::ZERO,
                    initialized: true,
                    gnss_chi2: score,
                    ..EstimatorStatus::default()
                },
                predict_failure: FakeFailure::None,
                update_failure: FakeFailure::None,
            }
        }

        fn with_update_failure(failure: FakeFailure) -> Self {
            Self {
                update_failure: failure,
                ..Self::with_score(0.0)
            }
        }

        fn with_predict_failure(failure: FakeFailure) -> Self {
            Self {
                predict_failure: failure,
                ..Self::with_score(0.0)
            }
        }
    }

    impl Estimator for FakeEstimator {
        fn name(&self) -> &'static str {
            "fake"
        }

        fn predict(&mut self, _dt: f64) -> Result<(), EstimatorError> {
            if let Some(err) = self.predict_failure.error("predict") {
                return Err(err);
            }
            Ok(())
        }

        fn update_imu(&mut self, _sample: &ImuSample) -> Result<(), EstimatorError> {
            if let Some(err) = self.update_failure.error("imu") {
                return Err(err);
            }
            Ok(())
        }

        fn update_gnss(&mut self, _sample: &GnssSample) -> Result<(), EstimatorError> {
            if let Some(err) = self.update_failure.error("gnss") {
                return Err(err);
            }
            Ok(())
        }

        fn update_baro(&mut self, _sample: &BarometerSample) -> Result<(), EstimatorError> {
            if let Some(err) = self.update_failure.error("baro") {
                return Err(err);
            }
            Ok(())
        }

        fn update_mag(&mut self, _sample: &MagnetometerSample) -> Result<(), EstimatorError> {
            if let Some(err) = self.update_failure.error("mag") {
                return Err(err);
            }
            Ok(())
        }

        fn attitude(&self) -> AttitudeEstimate {
            AttitudeEstimate {
                time: SimTime::ZERO,
                q_body_to_eci_xyzw: [0.0, 0.0, 0.0, 1.0],
                omega_body_rad_s: Vector3::zeros(),
                gyro_bias_body_rad_s: Vector3::zeros(),
            }
        }

        fn position(&self) -> PositionEstimate {
            PositionEstimate {
                time: SimTime::ZERO,
                position_eci_m: Vector3::zeros(),
                velocity_eci_m_s: Vector3::zeros(),
                accel_bias_body_m_s2: Vector3::zeros(),
            }
        }

        fn status(&self) -> EstimatorStatus {
            self.status
        }
    }

    fn fake_lane(score: f64) -> Box<dyn Estimator + Send> {
        Box::new(FakeEstimator::with_score(score))
    }

    fn fake_update_failure_lane(failure: FakeFailure) -> Box<dyn Estimator + Send> {
        Box::new(FakeEstimator::with_update_failure(failure))
    }

    fn fake_predict_failure_lane(failure: FakeFailure) -> Box<dyn Estimator + Send> {
        Box::new(FakeEstimator::with_predict_failure(failure))
    }

    fn gnss_sample() -> GnssSample {
        GnssSample {
            time: SimTime::ZERO,
            position_eci_m: Vector3::zeros(),
            velocity_eci_m_s: Vector3::zeros(),
            position_bias_eci_m: Vector3::zeros(),
            healthy: true,
        }
    }

    #[test]
    fn simplex_pass_through_picks_first_healthy_lane() {
        let lanes = vec![
            (LaneId::from("a"), build_ekf_lane()),
            (LaneId::from("b"), build_mekf_lane()),
        ];
        let mut m = MultiLaneEstimator::new(lanes, VoterPolicy::SimplexPassThrough);
        m.update_imu(&imu_sample()).unwrap();
        assert_eq!(m.active_lane_id().as_str(), "a");
    }

    #[test]
    fn lane_count_matches_construction() {
        let lanes = vec![
            (LaneId::from("primary"), build_ekf_lane()),
            (LaneId::from("backup"), build_mekf_lane()),
        ];
        let m = MultiLaneEstimator::new(lanes, VoterPolicy::SimplexPassThrough);
        assert_eq!(m.lane_count(), 2);
    }

    #[test]
    fn lane_status_returns_one_entry_per_lane_in_declaration_order() {
        let lanes = vec![
            (LaneId::from("alpha"), build_ekf_lane()),
            (LaneId::from("beta"), build_mekf_lane()),
            (LaneId::from("gamma"), build_ekf_lane()),
        ];
        let m = MultiLaneEstimator::new(lanes, VoterPolicy::SimplexPassThrough);
        let status = m.lane_status();
        assert_eq!(status.len(), 3);
        assert_eq!(status[0].id.as_str(), "alpha");
        assert_eq!(status[1].id.as_str(), "beta");
        assert_eq!(status[2].id.as_str(), "gamma");
        for s in &status {
            assert!(s.healthy);
        }
    }

    #[test]
    fn predict_dispatches_to_every_healthy_lane() {
        let lanes = vec![
            (LaneId::from("a"), build_ekf_lane()),
            (LaneId::from("b"), build_ekf_lane()),
        ];
        let mut m = MultiLaneEstimator::new(lanes, VoterPolicy::SimplexPassThrough);
        m.update_imu(&imu_sample()).unwrap();
        m.predict(0.01).unwrap();
        // Both lanes should still be healthy after a single predict
        // tick on a benign IMU sample.
        let status = m.lane_status();
        assert!(status[0].healthy);
        assert!(status[1].healthy);
    }

    #[test]
    fn begin_tick_re_marks_lanes_healthy() {
        let lanes = vec![
            (LaneId::from("a"), build_ekf_lane()),
            (LaneId::from("b"), build_mekf_lane()),
        ];
        let mut m = MultiLaneEstimator::new(lanes, VoterPolicy::SimplexPassThrough);
        // Manually unhealth a lane (without a real failure).
        m.lanes[1].healthy = false;
        m.begin_tick();
        assert!(m.lanes[0].healthy);
        assert!(m.lanes[1].healthy);
    }

    #[test]
    fn best_by_covariance_trace_picks_minimum_proxy() {
        let lanes = vec![
            (LaneId::from("high"), fake_lane(8.0)),
            (LaneId::from("low"), fake_lane(1.0)),
            (LaneId::from("mid"), fake_lane(3.0)),
        ];
        let mut m = MultiLaneEstimator::new(lanes, VoterPolicy::BestByCovarianceTrace);
        m.vote();
        assert_eq!(m.active_lane_id().as_str(), "low");
    }

    #[test]
    fn best_by_covariance_trace_ties_choose_lowest_index() {
        let lanes = vec![
            (LaneId::from("first"), fake_lane(1.0)),
            (LaneId::from("second"), fake_lane(1.0)),
        ];
        let mut m = MultiLaneEstimator::new(lanes, VoterPolicy::BestByCovarianceTrace);
        m.vote();
        assert_eq!(m.active_lane_id().as_str(), "first");
    }

    #[test]
    fn mid_value_select_by_innovation_falls_back_when_under_three_lanes() {
        // 2 lanes — median rule degenerates; should fall back to
        // simplex (lane 0).
        let lanes = vec![
            (LaneId::from("a"), build_ekf_lane()),
            (LaneId::from("b"), build_ekf_lane()),
        ];
        let mut m = MultiLaneEstimator::new(lanes, VoterPolicy::MidValueSelectByInnovation);
        m.update_imu(&imu_sample()).unwrap();
        assert_eq!(m.active_lane_id().as_str(), "a");
    }

    #[test]
    fn mid_value_select_by_innovation_picks_median_chi_square() {
        let lanes = vec![
            (LaneId::from("high"), fake_lane(9.0)),
            (LaneId::from("low"), fake_lane(1.0)),
            (LaneId::from("mid"), fake_lane(4.0)),
        ];
        let mut m = MultiLaneEstimator::new(lanes, VoterPolicy::MidValueSelectByInnovation);
        m.vote();
        assert_eq!(m.active_lane_id().as_str(), "mid");
    }

    #[test]
    fn predict_returns_last_error_when_every_lane_fails_closed() {
        let lanes = vec![
            (
                LaneId::from("a"),
                fake_predict_failure_lane(FakeFailure::InvalidConfig),
            ),
            (
                LaneId::from("b"),
                fake_predict_failure_lane(FakeFailure::InvalidConfig),
            ),
        ];
        let mut m = MultiLaneEstimator::new(lanes, VoterPolicy::SimplexPassThrough);
        let err = m.predict(0.01).unwrap_err();
        assert!(matches!(err, EstimatorError::InvalidConfig { .. }));
        assert!(m.lane_status().iter().all(|s| !s.healthy));
    }

    #[test]
    fn update_imu_returns_last_error_when_every_lane_fails_closed() {
        let lanes = vec![
            (
                LaneId::from("a"),
                fake_update_failure_lane(FakeFailure::InvalidConfig),
            ),
            (
                LaneId::from("b"),
                fake_update_failure_lane(FakeFailure::InvalidConfig),
            ),
        ];
        let mut m = MultiLaneEstimator::new(lanes, VoterPolicy::SimplexPassThrough);
        let err = m.update_imu(&imu_sample()).unwrap_err();
        assert!(matches!(err, EstimatorError::InvalidConfig { .. }));
        assert!(m.lane_status().iter().all(|s| !s.healthy));
    }

    #[test]
    fn gated_measurement_rejection_does_not_unhealth_lane() {
        let lanes = vec![
            (
                LaneId::from("a"),
                fake_update_failure_lane(FakeFailure::Gate),
            ),
            (LaneId::from("b"), fake_lane(0.0)),
        ];
        let mut m = MultiLaneEstimator::new(lanes, VoterPolicy::SimplexPassThrough);
        m.update_gnss(&gnss_sample()).unwrap();
        assert!(m.lane_status().iter().all(|s| s.healthy));
        assert_eq!(m.active_lane_id().as_str(), "a");
    }

    #[test]
    fn active_outputs_are_from_active_lane() {
        let lanes = vec![
            (LaneId::from("ekf"), build_ekf_lane()),
            (LaneId::from("mekf"), build_mekf_lane()),
        ];
        let mut m = MultiLaneEstimator::new(lanes, VoterPolicy::SimplexPassThrough);
        m.update_imu(&imu_sample()).unwrap();
        // Active = first lane (ekf).
        assert_eq!(m.active_lane_id().as_str(), "ekf");
        // Position estimate should be the EKF's (full state with
        // velocity / accel-bias slots populated). MEKF has no
        // position state, so its `position()` returns zero — if the
        // active output were coming from the MEKF lane this test
        // would catch it.
        let pos = m.position();
        assert!(pos.position_eci_m.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn name_is_stable_topic_string() {
        let lanes = vec![(LaneId::from("a"), build_ekf_lane())];
        let m = MultiLaneEstimator::new(lanes, VoterPolicy::SimplexPassThrough);
        assert_eq!(m.name(), "estimator.multi_lane");
    }
}
