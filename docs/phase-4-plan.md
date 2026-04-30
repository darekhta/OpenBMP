# OpenBMP Phase 4 Plan

> **Status:** Draft authored after Phase 3.14 closure. The Phase-3
> closure (`docs/phase-3-plan.md`, removed in commit `556c887`) is
> the precedent for this document's shape.

## Goal

Phase 3 lifted the simulator to a modular composable rocket and
landed the headline RocketPy Calisto cross-tool validation case.
**Phase 4 ships a virtual flight controller framework**: estimator,
autopilot, mission state machine, FDIR, and academic guidance laws.

The Phase-3.14 HAL-readiness sweep restructured the project so the
controller-side abstractions live in hardware-portable crates
(`openbmp-models`, `openbmp-mission`, the `Sensor` trait,
`ControlEffector`). Phase 4 builds on those abstractions — and the
load-bearing architectural contract for Phase 4 is:

> **`openbmp-fc` depends only on hardware-portable crates. It never
> imports `openbmp-sim`.**

A downstream adopter who layers a HAL on top of the same
abstractions can run the same controller binary against real
sensors and effectors. The OpenBMP repository itself still ships
only the simulator and only validates against academic /
public-benchmark scenarios.

## Scope Summary

### What Phase 4 ships

- **Estimator framework** in `openbmp-fc/estimator`: trait surface
  + reference impls. EKF (15-state error-state attitude / position /
  velocity / accel-bias / gyro-bias), MEKF (multiplicative quaternion
  attitude), UKF (unscented; sigma-point sampling). Each impl
  consumes `Sensor`-typed measurements, produces a typed estimate
  state.
- **Three-loop autopilot** in `openbmp-fc/autopilot`: rate, attitude,
  trajectory. Gain-scheduled by mission phase. Outputs
  `EffectorCommand` / `EngineCommand` values for the
  `ControlEffector` / `EngineModel` consumers. Anti-windup,
  saturation handling, deadband-aware integral terms.
- **Mission state machine** in `openbmp-fc/mission`: builds on the
  Phase-3.2 `MissionPhaseGraph` (in `openbmp-mission`) with
  controller-aware phase transitions (e.g., gain-schedule swap at
  burn-end, trim-target swap at separation event). Pre-launch /
  ascent / coast / apogee / descent / recovery academic phases.
- **Academic guidance laws** in `openbmp-fc/guidance`: attitude
  tracking against scripted reference state, scripted-waypoint
  navigation in inertial space, simple terminal-state regulation.
  No targeting, no real-world location guidance, no terminal-homing
  modes — the safety-boundaries `Reject` list still applies.
- **Powered-descent guidance scaffold** in `openbmp-fc/landing`:
  lossless-convexification soft-landing (LCvxLD, Acikmese & Ploen
  2007) and SCvx successive-convexification variant for academic
  powered-descent studies (e.g., reusable-vehicle return-to-pad
  textbook problem). Scenario-defined landing site, not a
  real-world target.
- **Linear MPC framework** in `openbmp-fc/mpc`: convex-QP solver
  (in-house Rust or vetted permissive-licence crate), reference
  attitude-tracking + trim-hold MPC controllers. Reuses the same
  effector / engine models the autopilot consumes.
- **FDIR framework** in `openbmp-fc/fdir`: scenario-injected fault
  models that exercise the Phase-3 effector and engine fault modes.
  Detection (residual-based), isolation (fault tree), recovery
  (autopilot mode switch, effector reconfiguration). Validation:
  the controller maintains stability through scenario-injected
  faults at known fault-coverage thresholds.
- **Public-benchmark validation case**: a Calisto-style or
  Niskanen-style closed-loop scenario where the autopilot tracks a
  scenario-declared reference profile within an academic tolerance.
  Cross-tool comparison with a publicly-published controller
  reference (RocketPy with attitude controller, OpenRocket
  controller plug-in, or an academic publication's reference
  controller).
- **Property tests, fuzz tests, microbenchmarks** across the new
  surface.

### What Phase 4 explicitly does NOT ship

| Deferred | Phase | Reason |
|---|---|---|
| Real device drivers / HAL impls | never (downstream-only) | Project boundary; downstream adopters ship those in their own repositories |
| Real-time scheduling guarantees, hard deadline assertions | never | Project boundary; adopters' qualification posture |
| Multi-body / staging promoted to first-class | 5 | Phase-3 staging hack is sufficient for Phase 4 closed-loop validation |
| Adaptive integrators (DOPRI5/8, RKF78) in the controller's plant model | 5 | RK4 fixed-step is sufficient for academic controller validation |
| Multi-rate scheduling (`RatePlan`) | 5 | Single-rate kernel + controller is sufficient for academic Phase-4 cases |
| HIL socket bridge for closed-loop | 5 | Off-by-default tooling; downstream user concern |
| Hypersonic guidance | 6 | Earth-atmosphere only; sub-Mach 5 |

## Architectural Contract

### Crate-level dependency rules

**`openbmp-fc` may import only from:**
- `openbmp-core` — math, units, frames, time, deterministic RNG,
  typed ids.
- `openbmp-state` — state types (`PointMassState`, `RigidBodyState`,
  `MassProperties`).
- `openbmp-models` — model trait surfaces (`ForceModel`,
  `MomentModel`, `MassModel`, `RigidMassModel`, `EnvironmentModel`,
  `VehicleState` data shape, `Integratable` integration extension,
  `SimStateDerivative` with primitive `Add` + `Mul<f64>` ops,
  `ModelEvalError`). Phase-3.15.E split the legacy `SimState` into
  `VehicleState + Integratable`; the `SimState` marker trait
  remains as a back-compat alias. RK4-specific weighted-sum logic
  lives integrator-side as `openbmp_sim::rk4_weighted_sum`, not on
  the derivative trait.
- `openbmp-mission` — mission graph, event triggers, phase
  transitions. Phase-3.15.C decoupled this from
  `openbmp-propulsion`; `EventAction::EngineCommand` carries
  scalar fields (throttle, gimbal, ignite, shutdown) rather than a
  typed `openbmp_propulsion::EngineCommand`. The runner's
  `EngineRack::apply_commands` constructs the typed propulsion
  command at apply time.
- `openbmp-sensors` — `Sensor` trait + `Sensor::read()` ingestion
  contract + `Timestamped<T>` wrapper + `SensorMeasurement` /
  `SensorTruth` value shapes. Phase-3.15.B added `read()` so
  controllers consume measurements via
  `dyn Sensor<Output = SensorMeasurement>` without ever touching
  `SensorTruth`. The `SyntheticSensor` trait stays Phase-3-side
  (gated by the `synthetic` feature); the runner wraps each
  synthetic with `SyntheticSensorAdapter` to expose it through
  `Sensor`.
- `openbmp-vehicle` — `ControlEffector` trait + `EngineModel` /
  `EngineCommand` types. The controller composes effector / engine
  commands; it does not run the actuator dynamics itself.
- `openbmp-aero` — aero deck types when the controller carries an
  internal plant model.
- `openbmp-propulsion` — engine + motor types when the controller
  carries an internal plant model.

**`openbmp-fc` MUST NOT import:**
- `openbmp-sim` — the simulator. The controller's plant model
  imports model traits from `openbmp-models`, not the kernel from
  `openbmp-sim`.
- `openbmp-cli` — the runner. The controller is consumed by the
  runner, not the other way around.
- `openbmp-scenario` — TOML parser. The controller takes typed
  config structs; the runner / scenario adapter parses TOML and
  hands the structs over.
- `openbmp-telemetry` — telemetry archive format. The runner emits
  controller-state telemetry by calling controller getters; the
  controller does not emit Parquet directly.

The rule is enforced by review and by
`openbmp-testkit`'s flight-controller dependency tripwire.

### Hardware-portability check

At every Phase-4 sub-phase exit, run:
```
cargo build -p openbmp-fc --no-default-features
```
This must compile down to the controller binary against
`openbmp-sensors`'s `Sensor` trait + the model-side abstractions
without pulling in the synthetic noise machinery. The same controller
binary, linked against a downstream HAL crate that implements
`Sensor` from real-hardware reads, is the design contract.

## Sub-phase Plan

Order is dependency-respecting.

### 4.1 — Estimator framework

**Scope.** `Estimator` trait + EKF reference impl. The controller's
state estimation entry point.

**Tasks.**
- `crates/openbmp-fc/src/estimator/mod.rs`: `Estimator` trait —
  `predict(state, dt)`, `update(measurement)`, `state()`.
- `crates/openbmp-fc/src/estimator/ekf.rs`: 15-state error-state EKF.
  Process model: `RigidBodyState` propagation. Measurement updates
  from `Sensor` measurements (IMU, GNSS, magnetometer, star tracker).
- Property tests: residual whitening (innovation sequence is
  zero-mean), filter consistency (NEES / NIS chi-square), and
  bit-stable replay.

**Exit criteria.** EKF tracks a scripted-trajectory reference within
academic tolerance. Sensor swap (IMU only vs IMU + GNSS) shows
expected accuracy improvement.

**Effort.** Medium (~1.5 weeks).

### 4.2 — MEKF + UKF

**Scope.** Two more estimator impls. MEKF for quaternion attitude
with multiplicative error; UKF for non-Gaussian / non-linear
measurement models.

**Effort.** Medium (~1.5 weeks).

### 4.3 — Three-loop autopilot

**Scope.** Rate / attitude / trajectory loops with gain scheduling
keyed on mission phase. Anti-windup, saturation, deadband-aware
integration.

**Tasks.**
- Rate loop: `gyro → torque command → effector deflection`.
- Attitude loop: `attitude_estimate → rate_command`.
- Trajectory loop: `position_estimate → attitude_command`.
- Gain table per `PhaseId`. Gains hot-swap on phase transition.

**Exit criteria.** Autopilot tracks a scripted attitude reference
within academic tolerance through mission-phase transitions.

**Effort.** Large (~2.0 weeks).

### 4.4 — Mission state machine integration

**Scope.** Build on the Phase-3.2 `MissionPhaseGraph` with
controller-aware transitions. Gain-schedule swap on burn-end;
trim-target swap on separation; controller mode swap on apogee.

**Effort.** Medium (~1.0 weeks).

### 4.5 — Academic guidance laws

**Scope.** Attitude tracking against scripted reference; scripted-
waypoint navigation in inertial space; simple terminal-state
regulation. Safety-boundary discipline strictly enforced.

**Effort.** Medium (~1.0 weeks).

### 4.6 — Powered-descent guidance scaffold

**Scope.** LCvxLD + SCvx solvers for academic powered-descent
studies. Scenario-defined landing site.

**Effort.** Large (~2.5 weeks; convex-QP solver is the long pole).

### 4.7 — Linear MPC framework

**Scope.** Convex-QP-based MPC for attitude tracking and trim hold.
In-house QP solver or vetted permissive-licence crate.

**Effort.** Medium (~1.5 weeks).

### 4.8 — FDIR framework

**Scope.** Detection (residual-based), isolation (fault tree),
recovery (autopilot mode switch). Validates against scenario-
injected effector / engine faults from Phase 3.4 / 3.6.

**Effort.** Medium (~1.0 weeks).

### 4.9 — Public-benchmark closed-loop validation case

**Scope.** A Calisto-class or Niskanen-class closed-loop scenario
that exercises the entire controller stack (estimator + autopilot
+ mission FSM + FDIR) against a scripted reference. Cross-tool
comparison with a publicly-published controller reference.

**Exit criteria.** Closed-loop trajectory matches the published
reference within an academic tolerance documented in the scenario's
`provenance.md`.

**Effort.** Medium (~1.5 weeks; long pole is finding a public
reference with sufficiently documented controller parameters).

### 4.10 — Phase 4 closure

**Scope.** Documentation cleanup + Phase-4 plan removal, mirroring
the Phase-1 / Phase-2 / Phase-3 closure pattern.

**Tasks.**
- `docs/design-concept.md § Phase Roadmap` — Phase 4 in past tense
  with the actual deliverables.
- `docs/README.md` — drop the link to `phase-4-plan.md`.
- `crates/openbmp-fc/README.md` — refresh status from "Phase 4
  stub" to "Phase 4 implemented".
- `git rm docs/phase-4-plan.md`.

**Effort.** Small (~0.3 weeks).

## Risk Register

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| `openbmp-fc` accidentally imports `openbmp-sim` | Medium | High | The sub-phase exit checklist runs `cargo build -p openbmp-fc --no-default-features`; `openbmp-testkit`'s dependency tripwire flags any forbidden controller edge in `crates/openbmp-fc/Cargo.toml`. |
| EKF / MEKF / UKF accumulate non-bit-stable state across reruns | Medium | High | Determinism tests on the filter equivalent to the kernel-level twin-run gate. Locked operand order on every `predict`/`update` math op. |
| Powered-descent QP solver pulls a heavy dependency | Medium | Medium | Vet the QP solver against the project's `cargo deny` policy before importing. Permissive license, no system-time dependency, no allocation on the solve loop. In-house implementation if no acceptable solver exists. |
| Closed-loop validation case picks a reference with insufficient controller documentation | Medium | Medium | Pre-pick the reference (RocketPy attitude controller is a known candidate) before the sub-phase begins; spike a quick comparison run during 4.1. |
| Controller correctness depends on the kernel's exact RK4 stage order | Low | High | The controller's plant model uses the same `SimState` / `SimStateDerivative` as the kernel; any RK4 variation only affects the *kernel-driven* truth, not the controller's predictions. The estimator's process model integrates separately and has its own determinism guarantees. |
| FDIR injection patterns leak into non-fault scenarios | Low | Medium | FDIR is opt-in via scenario `[faults]` block; the controller's fault flag is `false` when no fault is declared, and the recovery path is unreachable. |
| Phase 4 motivates someone to add real-RTOS targets | Low | High | Safety-boundaries `Reject` list still applies. Real RTOS ports, real bus protocols, real device drivers do not land in this repository regardless of how convincing the use case sounds; downstream HAL adopters host those in their own repositories. |

## Acceptance Gate for Phase 4 Closure

Phase 4 closes when **all** of the following are true:

1. Every sub-phase exit criterion is met.
2. `cargo fmt --all -- --check` clean.
3. `cargo clippy --workspace --all-targets --all-features -- -D
   warnings` clean.
4. `cargo test --workspace --all-features` green.
5. `cargo build -p openbmp-fc --no-default-features` compiles
   without any synthetic-feature crates active. The HAL-portability
   contract holds.
6. `crates/openbmp-fc/Cargo.toml` carries no `openbmp-sim`,
   `openbmp-cli`, `openbmp-scenario`, `openbmp-telemetry`,
   `openbmp-bridge`, or `openbmp-aerothermal` dependency; the
   `openbmp-testkit` dependency tripwire enforces this in CI.
7. The Phase-1 `constant-acceleration-drop`, Phase-2
   `niskanen-2009-chapter6`, and Phase-3 `rocketpy-calisto`
   scenarios continue to produce byte-identical Parquet across two
   reruns.
8. The Phase-4 closed-loop validation scenario passes its
   academic-tolerance gate.
9. The CI determinism gate runs all four scenarios twice on
   `x86_64-unknown-linux-gnu`.
10. Every new dataset file (controller gains, reference profiles,
    QP-solver test fixtures) has a sibling `provenance.md` and
    `openbmp check-provenance` passes.
11. `docs/design-concept.md § Phase Roadmap` lists Phase 4 in past
    tense with the actual deliverables.
12. `docs/phase-4-plan.md` is removed and `docs/README.md` no
    longer links to it.

## Suggested Calendar

| # | Sub-phase | Weeks |
|---|---|---|
| 4.1 | Estimator framework + EKF | 1.5 |
| 4.2 | MEKF + UKF | 1.5 |
| 4.3 | Three-loop autopilot | 2.0 |
| 4.4 | Mission state machine integration | 1.0 |
| 4.5 | Academic guidance laws | 1.0 |
| 4.6 | Powered-descent guidance | 2.5 |
| 4.7 | Linear MPC | 1.5 |
| 4.8 | FDIR | 1.0 |
| 4.9 | Closed-loop validation | 1.5 |
| 4.10 | Phase 4 closure | 0.3 |
| **Total** | | **13.8** |

With a ~30 % slip buffer for audit follow-ups, that puts a realistic
Phase-4 closure at **~5 calendar months** of focused work.
