# Phase 5 Plan Skeleton

Phase 5 starts from the Phase 4.C implementation pass and focuses on
higher-fidelity environment models, estimator lanes, and external
trajectory validation.

P0:
- NRLMSISE-00 upper-atmosphere density model.
- Multi-instance estimator routing with active-lane selection over the existing voter surface.

P1:
- EGM2008 spherical-harmonic gravity beyond WGS84-J2.
- Square-root UKF (Merwe & Wan 2001) for attitude and translational states.
- Observer-form anti-windup comparison against the Phase 4.C back-calculation baseline.
- **Mellinger & Kumar 2011 minimum-snap flat-output trajectory tracker** —
  the Phase 4.C `TrajectoryKind::FlatnessInspired` ships a
  PD-on-position-error → attitude-reference shape (no higher-derivative
  feed-forward). Phase 5 lands the polynomial-trajectory generator
  with snap / jerk / acceleration / velocity / position feed-forward
  terms and the matching attitude-rate / angular-acceleration
  references derived analytically from the flat outputs.
- **Cao & Hovakimyan 2010 L1 adaptive controller** — the Phase 4.C
  `L1InspiredParams` / `L1InspiredChannel` ship a scalar
  projection + first-order low-pass correction (no reference model,
  no state predictor, no cancellation of matched uncertainty
  through the predictor's tracking error). Phase 5 lands the full
  reference-model / state-predictor / piecewise-constant-adaptation
  / low-pass-filter architecture with bandwidth-projection bounds.
- **Willsky 1976 windowed-mean-shift GLRT** — the Phase 4.C
  `DetectorKind::SingleSampleGlrt` thresholds the unconstrained
  mean-shift GLRT statistic (which equals chi-square innovation) at
  a single sample. Phase 5 lands the windowed GLRT with on-line
  mean-shift estimation over a moving window and per-fault
  hypothesis selection.

P2:
- External trajectory cross-validation against published PX4 ekf2 / ArduPilot NavEKF3 datasets.
- Real ULog binary replay support.
- **Reproduce specific Bar-Shalom / Li / Kirubarajan textbook
  examples** — the Phase 4.C `tests/local_kalman_algebra.rs`
  exercises 1-D scalar Kalman algebra identities with arbitrary
  numbers; the published examples (e.g. §5.4 consistency, §5.5
  innovation analysis) with their original parameter sets are
  Phase 5.
- **Full closed-loop pipeline long-duration soak** — the Phase 4.C
  `tests/long_duration_stability.rs` exercises the EKF alone with
  synthetic measurement structs. The full sensor-ingest /
  scheduler / autopilot / mixer / kernel-bridge pipeline soak is
  Phase 5.

Closure note: remove or archive the Phase 4 planning documents once Phase 5 has a full implementation plan.
