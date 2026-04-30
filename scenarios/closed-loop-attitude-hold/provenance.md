# Provenance — `scenarios/closed-loop-attitude-hold/`

Canonical OpenBMP provenance record for the Phase-4.B closed-loop
attitude-hold scenario shipped under
`scenarios/closed-loop-attitude-hold/`.

## `scenarios/closed-loop-attitude-hold/scenario.toml`

```yaml
dataset_id:       openbmp.scenario.closed_loop_attitude_hold.v1
files:
  - scenarios/closed-loop-attitude-hold/scenario.toml
  - scenarios/closed-loop-attitude-hold/expected.toml
source_class:     synthetic-openbmp
source_title:     >-
  Phase-4.B closed-loop attitude-hold scenario. Demonstrates the
  declarative `[fc]` block driving the full FC pipeline (EKF +
  three-loop autopilot + mixer + health/FDIR) against a constant
  identity-quaternion attitude reference.
source_authors:   OpenBMP (Dmitri Arekhta) for the Phase-4.B FC closure
source_id:        Synthetic OpenBMP Phase-4.B FC fixture
source_url:       —
publication_date: 2026-04-30
methodology_reference: >-
  `docs/phase-4b-plan.md § F3` documents the closed-loop scenario
  contract; `crates/openbmp-fc/tests/closed_loop.rs` exercises the
  same pipeline at the integration-test layer.
```

## Closed-loop attitude-hold validation — provenance

## Status

`experimental`. Synthetic data and academic-tier algorithm tunings.
Not validated against any published flight-test record.

## Validation tier

`validated-toy` for the algorithm-internal correctness checks
(determinism, byte-stable replay) that the
`closed_loop_pipeline_runs_deterministically` integration test in
`crates/openbmp-fc/tests/closed_loop.rs` runs on the same pipeline
this scenario describes.

`experimental` for the scenario-driven kernel run, since the
kernel↔FC bridge is reserved for a follow-up phase. Until that
bridge lands, this scenario is exercised by parser-side validation
only (`cargo test -p openbmp-scenario`).

## Synthetic data declaration

Every input in this scenario is synthetic:

- Body mass = 1.0 kg, reference area = 1.0 m². No real vehicle.
- Initial state = origin, zero velocity, identity attitude.
- IMU samples are injected with constant 9.81 m/s² accel along ECI
  +z and zero gyro rates — the controller's gravity model cancels the
  +z accel and the body sits motionless in the pipeline test.
- GNSS samples = origin position with zero velocity.
- Barometer samples = 101 325 Pa (sea-level standard).
- Magnetometer samples = 30 000 nT along ECI +z (equatorial-surface
  dipole).
- EKF / autopilot / health / FDIR parameters are textbook
  academic defaults (Stevens & Lewis 2015, Bar-Shalom-Li-Kirubarajan
  §5.4).
- Mission phase graph: pad → ascent at t = 0.1 s.

No external published reference informs any number in this file.

## Algorithms

- EKF: 15-state error-state with the constant flat-Earth gravity
  model and the Phase-4.B `EarthDipoleField` magnetic-field model
  (degree-1 truncation of WMM).
- Three-loop autopilot: rate / attitude / trajectory PID with
  anti-windup back-calculation.
- Mixer: phase-gated actuator authority via `PhaseAuthorityTable`.
- Health monitor: bus-sequence-based staleness detection.
- FDIR: burst-counter detector publishing `fdir.status`.

## Phase 4.C deferrals

This scenario does NOT exercise:

- A real UKF (Phase 4.C deferral).
- A real MPC backed by a vetted convex-QP solver (Phase 4.C).
- LCvxLD / SCvx powered-descent guidance (Phase 4.C).
- Full WMM 2025 spherical-harmonic field with the COF dataset
  (Phase 4.C).

## Sources

- Stevens, B. L. & Lewis, F. L. (2015). *Aircraft Control and
  Simulation* (3rd ed.). Wiley.
- Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2001). *Estimation
  with Applications to Tracking and Navigation*. Wiley.
- US Standard Atmosphere 1976. NOAA / NASA / USAF.
- Earth-dipole magnetic-field truncation: textbook spherical-
  harmonic degree-1 truncation of WMM.

## Pinned files

This scenario references no external data files; all parameters are
inline.
