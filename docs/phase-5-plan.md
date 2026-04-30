# Phase 5 Plan Skeleton

Phase 5 starts from the Phase 4.C implementation pass and focuses on
higher-fidelity environment models, estimator lanes, and external
trajectory validation.

P0:
- WMM 2025 spherical-harmonic geomagnetic model with COF dataset provenance.
- NRLMSISE-00 upper-atmosphere density model.
- Multi-instance estimator routing with active-lane selection over the existing voter surface.

P1:
- EGM2008 spherical-harmonic gravity beyond WGS84-J2.
- Square-root UKF (Merwe & Wan 2001) for attitude and translational states.
- Observer-form anti-windup comparison against the Phase 4.C back-calculation baseline.

P2:
- External trajectory cross-validation against published PX4 ekf2 / ArduPilot NavEKF3 datasets.
- Real ULog binary replay support.

Closure note: remove or archive the Phase 4 planning documents once Phase 5 has a full implementation plan.
