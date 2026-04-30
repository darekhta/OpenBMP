# OpenBMP Phase 4 Plan

> **Status:** Draft revision after Phase 3.15 closure. Replaces the
> earlier "virtual flight controller framework" draft. Mirrors the
> Phase-3 closure pattern in shape; reframes the deliverable as
> **an autopilot binary with the architecture and discipline of a
> deployable flight controller, validated in SITL only**.

## Phase 4.A + Phase 4.B + Phase 4.C Audit State (2026-04-30)

Phase 4.A landed the architectural skeleton: bus, scheduler,
commander, parameter registry, table registry, dictionary, voter,
sensor-ingest, EKF (15-state error-state) with GNSS update, MEKF
(attitude-only), three-loop autopilot, guidance, mixer,
HealthMonitor, FdirJob, replay, and the `FlightController` façade.

Phase 4.B deleted over-claiming stubs (`mpc.rs`, `landing.rs`, and
the mean-only `Ukf`), added barometer / magnetometer updates,
gravity and magnetic model seams, trajectory-loop plumbing,
bus-sequence staleness, gain schedule / phase authority / voter /
FDIR runtime wiring, the `[fc]` parser, `FcRunner`, and the
closed-loop-attitude-hold fixture.

The Phase 4 audit corrected overstatements in the landed claim. The
post-audit state is:

- EKF / MEKF covariance updates use Joseph form, innovation gates are
  derived per measurement dimension from a stated false-alarm rate
  unless a legacy explicit gate is supplied, and quaternion propagation
  explicitly renormalizes.
- `openbmp-physics` now owns the **complete** HAL-portable physics
  stack: gravity (`ConstantGravity`, `PointMassGravity`, `J2Gravity`),
  atmosphere (`IsothermalAtmosphere`, full 7-layer `UsStandard1976`),
  magnetic (`EarthDipoleField`, full WMM 2025 `Wmm2025`), wind
  (`NoWind`, `ConstantWind`, `LayeredWind`, `GustWind`), `PhysicsError`,
  and the trait surfaces. The simulator-side `openbmp-env` crate was
  retired (folded into `openbmp-physics` per
  `docs/physics-consolidation-plan.md`); both `openbmp-fc` and the
  kernel-side adapters in `openbmp-vehicle` / `openbmp-cli` consume
  physics directly. The FC's parallel `GravityModel` /
  `MagneticFieldModel` traits and the `EarthDipoleField` /
  `ConstantGravityZ` / `Wgs84J2GravityModel` impls were deleted in
  the consolidation; the FC's estimators now use
  `openbmp_physics::gravity::GravityModel` and
  `openbmp_physics::magnetic::MagneticFieldEci` directly.
- `EstimatorStatus::innovation_rejected` is no longer hard-coded
  false; transient estimator diagnostics reset at the start of each
  estimator scheduler tick.
- `ActuatorCommand.saturated` reports trajectory-loop, attitude-loop,
  and rate-loop saturation.
- Voted IMU ingest publishes per-lane `sensor.status`, and an
  integration test exercises `SyntheticSensorAdapter -> VotedImuIngest
  -> bus`.
- The mixer can publish `EffectorCommandSet` keyed by `EffectorId`
  through an explicit `ActuatorChannelMap`; `FcRunner` derives phase
  authority from mission graph allowed effectors / engines and `[fc]`
  overrides.
- `[fc]` now carries `frame_budget_us`; guidance/health/FDIR periods
  derive from `base_rate_hz`.

The Phase 4.C audit execution added the kernel↔FC bridge and the
first SOTA-adjacent algorithm pass:

- `[fc]` scenarios now drive the `phase2_*.rs` kernel runners through
  a lockstep bridge that primes synthetic sensors, steps the
  `FlightController`, and applies FC effector / engine command sets to
  the simulator racks. When `[fc]` is absent, the bridge is a no-op.
- EKF / MEKF now include Gauss-Markov bias dynamics, iterated
  magnetometer updates, Markley-style MEKF covariance reset, and
  WGS84-J2 gravity through the HAL-portable `openbmp-physics` crate.
- A real 6-state sigma-point UKF exists for attitude + gyro-bias
  validation; full 15-state and square-root UKF variants are Phase 5.
- Autopilot SOTA hooks include gyro notch filters,
  differential-flatness attitude-reference generation, and a
  feature-gated L1 adaptive rate-loop augmentation.
- FDIR supports burst-counter, GLRT, and CUSUM detector families with
  explicit tripped-mask bits; sensor lane status now covers IMU,
  barometer, GNSS, and magnetometer ingest.
- Clarabel v0.9 is vetted in `docs/clarabel-vetting.md` and is gated
  behind the `mpc` feature for QP / SOCP primitives.

The audit still does **not** claim full algorithm closure for every
research item. Remaining Phase 5 work includes full WMM 2025,
NRLMSISE-00, multi-instance estimator routing, full 15-state /
square-root UKF, and full MPC / LCvxLD trajectory reproduction
against published references.

## Goal

Phase 3 delivered a modular composable rocket and the headline RocketPy
Calisto cross-tool validation case. Phase 3.14/3.15 restructured the
project so the controller-side abstractions (`Sensor`,
`ControlEffector`, `MissionPhaseGraph`, model traits) live in
hardware-portable crates, and the `openbmp-fc` dependency cone is
verified in CI.

**Phase 4 makes `openbmp-fc` a flight-controller binary** with
deployable-grade architecture: lockstep clock, internal pub/sub bus,
cyclic scheduler with budget enforcement, commander as the single
state-machine owner, parameter registry, voter, health/arming gate,
and phase-gated actuator authority — wired around the academic
algorithms (EKF / MEKF / three-loop autopilot / mission FSM /
guidance / FDIR / optionally MPC). The same binary, linked against
a downstream HAL crate, is intended to be deployable; the OpenBMP
repository itself **ships only simulator-local validation** and
**makes no compliance claim** under IEC 61508, ISO 26262, DO-178C,
or equivalent regimes.

This is the load-bearing reframe vs. the previous draft:

> Phase 4 is **not** an algorithm bundle. It is an autopilot
> architecture into which academic algorithms are slotted. The
> scaffolding (bus, scheduler, commander, params, voter, health) is
> what makes the binary HAL-portable; the algorithms are what make it
> useful.

The architectural contract from the previous draft is preserved
verbatim:

> **`openbmp-fc` depends only on hardware-portable crates. It never
> imports `openbmp-sim`.**

## Scope Summary

### What Phase 4 ships

**Autopilot architecture (new in this revision):**

- **Internal pub/sub bus** in `openbmp-fc/bus`: typed-topic registry,
  single-writer many-reader cells with sequence counter, no blocking,
  no allocation on the hot path. Topics: `imu_sample`, `baro_sample`,
  `gnss_sample`, `mag_sample`, `attitude_estimate`,
  `position_estimate`, `estimator_status`, `vehicle_status`,
  `failsafe_flags`, `actuator_cmd`, `engine_cmd`. Pattern adapted from
  PX4 uORB.
- **Cyclic scheduler** in `openbmp-fc/scheduler`: rate-group dispatch
  with `{period, budget, priority}` per job. Refuses to start a job
  whose budget will not fit before the next frame; emits an
  `overrun_event` topic on slip. Pattern adapted from
  ArduPilot `AP_Scheduler`.
- **Lockstep clock contract**: `openbmp-fc` exposes `Clock::now()` as
  the only time source; `std::time::*` is forbidden in the crate (lint-
  enforced at the workspace level, on top of the existing kernel ban).
  The simulator's runner injects `now()` from `SimTime`. A downstream
  HAL injects `now()` from a hardware monotonic.
- **Parameter registry** in `openbmp-fc/params`: typed, named,
  default-valued parameters loaded at boot. Defined in TOML; codegen
  produces a flat `Params` struct the modules consume. Hot-tunable via
  the bus's `parameter_update` topic. Persisted to telemetry as a JSON
  dump alongside the run. Pattern adapted from PX4 + F Prime.
- **Tables** in `openbmp-fc/tables`: validated-then-activated typed
  blobs (gain schedules, mass/CG/inertia sweep curves). Two-buffer
  swap. Pattern adapted from NASA cFE TBL.
- **Commander** in `openbmp-fc/commander`: the **only** module that
  owns flight phase. Inputs are bus topics; outputs are
  `vehicle_status` (current phase) and `arming_state`. Phase
  transitions are gated by per-phase guards
  (e.g., "Boost requires armed + thrust > threshold + estimator
  healthy"). Builds on the Phase-3.2 `MissionPhaseGraph`.
- **Health & arming** in `openbmp-fc/health`: subscribes to every
  sensor / estimator / commander topic; publishes `failsafe_flags`
  (innovation gate breach, dead-reckoning, sensor divergence,
  scheduler overrun, parameter-bound violation). Commander reads those
  as hard inputs to its transitions.
- **Sensor voter** in `openbmp-fc/voter`: trait for N-of-M voting on
  redundant sensors. Phase 4 ships the simplex (pass-through) and
  triplex mid-value-select impls; OpenBMP scenarios still typically
  use one IMU per sensor channel, but the seam exists so triplex is a
  configuration, not a code change.
- **Phase-gated actuator authority** in `openbmp-fc/mixer`: actuator
  command writes are zero-mixed unless the commander phase + the
  mission graph's `allowed_effectors` mask permits the channel. This
  prevents a controller bug in one phase from actuating in another.
- **Event / telemetry / log distinction**: three streams. *Events* are
  severity-tagged sparse human-readable lines (F Prime EVS shape).
  *Telemetry channels* are current-value periodic (F Prime TlmChan
  shape). *Logs* are high-rate bus-topic dumps for replay
  (PX4 ULog shape). The simulator runner consumes all three; a
  downstream HAL adopter picks per their bandwidth posture.

**Academic algorithms (preserved from previous draft, slotted into the
architecture above):**

- **Estimator framework** in `openbmp-fc/estimator`: trait surface +
  reference impls. EKF (15-state error-state position / velocity /
  attitude / accel-bias / gyro-bias), MEKF (multiplicative quaternion
  attitude), and Phase 4.C 6-state sigma-point UKF for attitude +
  gyro-bias validation — see § 4.8 / 4.11.
  Each consumes `Sensor::read()`
  measurements via the bus, publishes `estimator_status` (innovation
  ratios, dead-reckoning flag, lane health) so the commander can react
  without reaching into filter internals.
- **Three-loop autopilot** in `openbmp-fc/autopilot`: rate / attitude /
  trajectory loops, gain-scheduled by mission phase, anti-windup,
  saturation, deadband-aware integration. Consumes `attitude_estimate`
  / `position_estimate` from the bus, publishes `actuator_cmd` /
  `engine_cmd`. Stevens & Lewis 2015 academic formulation.
- **Mission FSM integration** in `openbmp-fc/mission`: builds on the
  Phase-3.2 `MissionPhaseGraph` with controller-aware transitions
  (gain-schedule swap on burn-end, trim-target swap on separation,
  controller mode swap on apogee). Owned by the commander.
- **Academic guidance laws** in `openbmp-fc/guidance`: attitude
  tracking against scripted reference state, scripted-waypoint
  navigation in inertial space, simple terminal-state regulation.
  Safety-boundary `Reject` list still applies categorically.
- **FDIR framework** in `openbmp-fc/fdir`: residual-based detection
  (built on innovation gates exposed by the estimator), fault-tree
  isolation, recovery via commander mode swap and effector
  reconfiguration. Validates against scenario-injected effector /
  engine fault modes from Phase 3.4 / 3.6.
- **Public-benchmark closed-loop validation case** in `scenarios/`: a
  Calisto-style or Niskanen-style closed-loop scenario where the full
  autopilot binary tracks a scenario-declared reference profile within
  an academic tolerance documented in the scenario's `provenance.md`.
  This is the proof point that the architecture works as a real
  autopilot.
- **Log-replay tooling** in `openbmp-fc/replay` + a CLI subcommand:
  state-stable replay (default) and bit-stable replay (stretch). Reads
  a recorded ULog-shaped log, re-injects sensor-stream topics into the
  bus, asserts actuator-stream equivalence within tolerance.

### What Phase 4 explicitly does NOT ship

| Deferred | Phase | Reason |
|---|---|---|
| Real device drivers / HAL impls | never (downstream-only) | Project boundary; downstream adopters ship those in their own repositories |
| Real-time scheduling guarantees, hard deadline assertions on real hardware | never | Project boundary; adopters' qualification posture |
| Linear MPC framework + powered-descent guidance (LCvxLD / SCvx) | 4 stretch (4.11) | Gated on QP-solver vetting; can land if 4.1–4.10 closes early; otherwise Phase 5 |
| Multi-body / staging promoted to first-class | 5 | Phase-3 staging hack is sufficient for Phase 4 closed-loop validation |
| Adaptive integrators (DOPRI5/8, RKF78) in the controller's plant model | 5 | RK4 fixed-step is sufficient for academic controller validation |
| Kernel-level multi-rate scheduling (`RatePlan`) | 5 | Single-rate kernel sufficient for Phase 4; the autopilot's *internal* scheduling is multi-rate and lands in 4.1 |
| Triplex / quadruplex sensor lanes wired to real redundancy | 5 | Voter trait lands in 4.2; real lane wiring is downstream-side |
| HIL socket bridge for closed-loop | 5 | Off-by-default tooling; downstream user concern |
| Hypersonic guidance | 6 | Earth-atmosphere only; sub-Mach 5 |
| `openbmp-fc` as a deployable flight stack | never | Repository does not validate or claim deployability |

## Architectural Contract

### Crate-level dependency rules (preserved from previous draft)

**`openbmp-fc` may import only from:**
- `openbmp-core` — math, units, frames, time, deterministic RNG, typed
  ids, **`Clock` trait** (new in 4.1; see below).
- `openbmp-state` — state types (`PointMassState`, `RigidBodyState`,
  `MassProperties`).
- `openbmp-models` — model trait surfaces (`ForceModel`, `MomentModel`,
  `MassModel`, `RigidMassModel`, `EnvironmentModel`, `VehicleState`,
  `TranslationalState` / `RigidBodyKinematicState`, `Integratable`,
  `SimStateDerivative`, `ModelEvalError`).
- `openbmp-mission` — mission graph, event triggers, phase transitions.
- `openbmp-sensors` — `Sensor` trait + `Sensor::read()` + `Timestamped`
  + measurement / truth value shapes. Default-features-off in
  `openbmp-fc`'s own dev profile so `synthetic` does not leak.
- `openbmp-vehicle` — `ControlEffector` trait + `EngineModel` /
  `EngineCommand` types.
- `openbmp-aero`, `openbmp-propulsion` — deck / engine types when the
  controller carries an internal plant model (estimator process model,
  MPC plant).

**`openbmp-fc` MUST NOT import:**
- `openbmp-sim`, `openbmp-cli`, `openbmp-scenario`, `openbmp-telemetry`,
  `openbmp-bridge`, `openbmp-aerothermal`, `openbmp-env`.

The rule is enforced by `openbmp-testkit`'s flight-controller
dependency tripwire (Phase 3.15.A hardening), which resolves
workspace aliases and `dep:<name>` feature activations so renamed
forbidden edges fail in CI.

### Hardware-portability check (preserved)

At every Phase-4 sub-phase exit:
```
cargo build -p openbmp-fc --no-default-features
```
This must produce a controller binary against
`openbmp-sensors`'s `Sensor` trait + the model-side abstractions
without pulling in `synthetic`. The same binary linked against a
downstream HAL crate is the design contract.

### Lockstep clock contract (new in this revision)

`openbmp-fc/src/clock.rs` exports a `Clock` trait:
```rust
pub trait Clock {
    fn now(&self) -> SimTime;
    fn tick(&self) -> StepIndex;
}
```
`openbmp-fc` modules read time only via an injected `&dyn Clock`.
**`std::time::SystemTime`, `std::time::Instant`, and any wall-clock
source are banned in `openbmp-fc`** — enforced by a workspace-level
clippy lint (`disallowed_methods` for `Instant::now`, `SystemTime::now`)
and a tripwire in `openbmp-testkit` that greps the FC source tree.

This bans drift between SITL replay and HAL deployment, and keeps the
binary's notion of "now" consistent with the simulator's `SimTime` or
the HAL's hardware monotonic. Pattern from PX4 lockstep + ArduPilot
SITL.

### SITL discipline (new in this revision)

`openbmp-fc` source must contain no `cfg!(simulator)`,
`cfg!(test)`-gated production logic, or `if running_in_sim` branches.
The simulator drives behaviour entirely through the bus topics it
publishes (sensor samples, parameter updates, command-channel inputs)
and the bus topics it subscribes to (actuator commands, telemetry,
events). A simple lint or grep tripwire catches accidental
introductions.

## Sub-phase Plan

Order is dependency-respecting. **Sub-phase 4.1 lands the autopilot
binary skeleton before any algorithm. Algorithms in 4.3–4.9 plug into
the skeleton.**

### 4.1 — Autopilot binary skeleton

**Scope.** The architectural scaffolding that makes everything else
possible.

**Tasks.**
- `openbmp-fc/src/clock.rs`: `Clock` trait + `SimulatedClock` impl
  (constructor takes a callback that returns `SimTime`). Workspace
  lints + `openbmp-testkit` tripwire wired in.
- `openbmp-fc/src/bus/`: typed-topic registry. Topics defined as Rust
  types deriving a `Topic` trait via a small derive macro (in-crate,
  no external dep). Single-writer many-reader cells with sequence
  counter. Compile-time-checked subscriber registration.
- `openbmp-fc/src/scheduler/`: rate-group dispatcher. Each registered
  job carries `{topic_or_period, budget_us, priority}`. Refuses to
  start an over-budget job; emits `overrun_event`.
- `openbmp-fc/src/params/`: parameter file format (TOML), codegen for
  typed `Params` struct, runtime override via `parameter_update` bus
  topic, dump-to-JSON sidecar.
- `openbmp-fc/src/tables/`: validated-then-activated typed table
  format. Two-buffer atomic swap.
- `openbmp-fc/src/dictionary/`: build-time codegen of a JSON dictionary
  describing every command, event, channel, and parameter the binary
  exposes. Shipped alongside release artifacts so external consumers
  can parse logs.
- `openbmp-fc/src/lib.rs`: module exports + `FlightController` struct
  that owns clock, bus, scheduler, params, tables, dictionary.

**Property tests.** Bus single-writer-many-reader determinism, rate-
group integer-divisor dispatch, parameter-update round-trip, table
validate-then-activate atomicity, scheduler overrun detection, clock-
trait tripwire on `std::time` reintroduction.

**Exit criteria.** A "no-op autopilot" binary compiles, runs in a
synthetic 100-tick scenario, publishes a heartbeat event on every
tick, dumps params + dictionary to disk, and produces byte-identical
output across reruns. `cargo build -p openbmp-fc --no-default-features`
succeeds.

**Effort.** Large (~2.0 weeks).

### 4.2 — Sensor ingest path + voter

**Scope.** Move sensor data from `Sensor::read()` to bus topics, with
a voter in front so triplex is a configuration not a refactor.

**Tasks.**
- `openbmp-fc/src/voter/`: `Voter<T>` trait with `vote(samples: &[T])
  -> VotedReading<T>`. Reference impls: `PassThroughVoter` (simplex),
  `MidValueSelectVoter` (triplex), `WeightedMeanVoter` (weighted).
- `openbmp-fc/src/sensor_ingest/`: per-sensor-kind ingest jobs that
  call `Sensor::read()` at the sensor's natural cadence, hand the
  result through the voter, publish on the bus
  (`imu_sample`, `baro_sample`, etc.).
- Per-sensor `sensor_status` topic with health, latency, and
  innovation-gate-future-use flags.
- Property tests: voter ordering invariance, NoSample propagation,
  bus publish + sequence-counter correctness across thousands of
  ticks.

**Exit criteria.** Sensor truth flowing through `SyntheticSensorAdapter`
+ `Sensor::read()` lands as bus messages with correct timestamps and
sequence counters, voted through pass-through, ready for an estimator
to subscribe.

**Effort.** Medium (~1.0 weeks).

### 4.3 — Estimator framework + EKF

**Scope.** First estimator algorithm slotted into the bus.

**Tasks.**
- `openbmp-fc/src/estimator/mod.rs`: `Estimator` trait — `predict(dt)`,
  `update(measurement)`, `state()`, `status() -> EstimatorStatus`.
  `EstimatorStatus` carries innovation test ratios, dead-reckoning
  flag, lane health.
- `openbmp-fc/src/estimator/ekf.rs`: 15-state error-state EKF.
  Process model: `RigidBodyState` propagation. Measurement updates from
  bus topics (`imu_sample`, `gnss_sample`, `mag_sample`, `baro_sample`).
  Innovation gates configurable per measurement. Publishes
  `attitude_estimate`, `position_estimate`, `estimator_status`.
- Bind estimator state readers to `TranslationalState` /
  `RigidBodyKinematicState`; use concrete `RigidBodyState` only where
  the full layout is part of the algorithm.
- Property tests: residual whitening, NEES / NIS chi-square consistency,
  bit-stable replay across reruns.

**Exit criteria.** EKF tracks a scripted-trajectory reference within
academic tolerance. Sensor swap (IMU only vs IMU + GNSS) shows
expected accuracy improvement. `estimator_status` flags dead-reckoning
within one update of GNSS dropout.

**Effort.** Large (~2.0 weeks).

### 4.4 — Commander + mission FSM integration

**Scope.** Single state-machine owner. Arming chain. Phase-gated
actuator mask.

**Tasks.**
- `openbmp-fc/src/commander/`: `Commander` job that owns the active
  `PhaseId`. Reads the Phase-3.2 `MissionPhaseGraph` (passed in via
  config), evaluates `BuiltInEventTrigger` against bus topics each
  tick, applies `EventBinding::action` (publishes `vehicle_status`
  with new phase, arms / disarms, etc.). Gated transitions enforce
  per-phase preconditions.
- `openbmp-fc/src/commander/arming.rs`: arming chain. Subscribes to
  `failsafe_flags`; refuses to publish "Armed" when any
  arm-blocking flag is set.
- `openbmp-fc/src/mixer/`: actuator mixer that zero-mixes
  `actuator_cmd` writes from any source unless commander phase + the
  mission graph's `allowed_effectors` permits the channel. Same for
  `engine_cmd` against `allowed_engines`.

**Exit criteria.** A scripted scenario walks through PreLaunch → Armed
→ Boost → Coast → Apogee → Descent → Recovery → Landed under
commander control. An actuator command issued during the wrong phase
is silently zero-mixed and a warning event fires.

**Effort.** Medium (~1.0 weeks).

### 4.5 — Three-loop autopilot

**Scope.** Rate / attitude / trajectory loops, gain-scheduled by
commander-published phase. Anti-windup, saturation, deadband-aware
integration. Stevens & Lewis 2015 formulation.

**Tasks.**
- Rate loop: `gyro → torque command → effector deflection`.
- Attitude loop: `attitude_estimate → rate_command`.
- Trajectory loop: `position_estimate → attitude_command`.
- Gain table per `PhaseId` loaded from `tables/`. Gains hot-swap on
  phase transition.
- Saturation reporting via `actuator_status` topic; integrators freeze
  on saturation.

**Exit criteria.** Autopilot tracks a scripted attitude reference
within academic tolerance through mission-phase transitions. Hot-swap
on burn-end is bumpless within an academic tolerance.

**Effort.** Large (~2.0 weeks).

### 4.6 — Health & arming + FDIR

**Scope.** Failsafe-flag publisher + fault tree + recovery via
commander mode swap. Combines what the previous draft had as separate
"FDIR" and what was missing entirely ("Health").

**Tasks.**
- `openbmp-fc/src/health/`: subscribes every topic the rest of the
  binary publishes; publishes `failsafe_flags` (an aggregated bitfield)
  and per-source `health_status` events. Innovation-gate breaches,
  dead-reckoning detection, sensor divergence, scheduler overrun,
  parameter-bound violations all surface here.
- `openbmp-fc/src/fdir/`: residual-based detection (subscribed to
  `estimator_status`), fault-tree isolation (effector / engine fault
  modes from Phase 3.4 / 3.6), recovery via commander mode swap +
  effector reconfiguration via the mixer.
- Property tests: scenario-injected fault triggers expected
  `failsafe_flags`; commander transitions to defined safe-state; no
  actuator commands escape the mixer during the fault window.

**Exit criteria.** Each Phase-3.4 effector fault and Phase-3.6 engine
fault, when injected, produces detected → isolated → recovered within
an academic latency budget documented per fault. Controller maintains
stability through the recovery in the closed-loop validation scenario.

**Effort.** Medium (~1.5 weeks).

### 4.7 — Academic guidance laws

**Scope.** Attitude tracking, scripted reference state, scenario-
waypoint navigation, simple terminal-state regulation.
Safety-boundary `Reject` list applies categorically.

**Effort.** Medium (~1.0 weeks).

### 4.8 — MEKF + UKF

**Scope.** MEKF for quaternion attitude with multiplicative error.
Slots into the bus alongside the EKF; estimator selection is a
parameter.

The Phase 4.C audit pass restored UKF only as a real sigma-point
filter: a 6-state attitude + gyro-bias Julier-Uhlmann implementation
that propagates every sigma point and recombines quaternion attitude
through an iterative mean. The old mean-only scaffold remains deleted.
The full 15-state and square-root UKF variants are Phase 5 scope.

**Effort.** Medium (~1.0 weeks; was 1.5 weeks with UKF in scope).

### 4.9 — Closed-loop validation case + log-replay tooling

**Scope.** The proof point.

**Tasks.**
- `scenarios/<name>-closed-loop/`: a Calisto-class or Niskanen-class
  scenario with the full autopilot binary tracking a scripted reference
  profile. `provenance.md` documents the academic tolerance envelope
  and the public reference (RocketPy attitude controller, OpenRocket
  controller plug-in, or an academic publication's reference
  controller).
- `openbmp-fc/src/replay/`: log-replay reader that consumes a
  ULog-shaped log file, re-injects sensor-stream topics into the bus,
  asserts actuator-stream equivalence within tolerance.
- `openbmp run --replay <log>` CLI subcommand (CLI-side only —
  `openbmp-fc` exposes the library API; the runner wires it).
- State-stable replay is the merge gate; bit-stable replay is a
  stretch goal that ships if no non-determinism remains.

**Exit criteria.** Closed-loop trajectory matches the public reference
within the documented academic tolerance. Log-replay produces
identical actuator stream within state-stable tolerance. Replay tool
catches a deliberately introduced determinism regression in CI.

**Effort.** Large (~2.0 weeks; long pole is finding a public reference
with sufficiently documented controller parameters).

### 4.10 — Phase 4 closure

**Scope.** Documentation cleanup + Phase-4 plan removal, mirroring the
Phase-1 / 2 / 3 closure pattern.

**Tasks.**
- `docs/design-concept.md § Phase Roadmap` — Phase 4 in past tense
  with the actual deliverables.
- `docs/software-architecture.md § Flight Controller` —
  rewrite as "Flight Controller" with the architecture documented and
  the simulator-local validation status preserved.
- `docs/README.md` — drop the link to `phase-4-plan.md`.
- `crates/openbmp-fc/README.md` — refresh from "Phase 4 stub" to
  "Phase 4 implemented", documenting the bus / scheduler /
  commander / params / health / voter / mixer / estimator / autopilot
  / mission / guidance / fdir / replay module structure.
- `git rm docs/phase-4-plan.md`.

**Effort.** Small (~0.3 weeks).

### 4.11 — Solver-backed MPC / powered-descent primitives

**Status.** Partially implemented behind feature gates. Phase 4.A initially shipped scaffold modules
(`mpc.rs` with `LqrAttitudeMpc`, `landing.rs` with `Lcvxld` / `Scvx`)
that wore the algorithm names but were single-step PD / constant-gain
LQR controllers under the hood. Phase 4.B deleted them rather than
leave the misrepresenting names in the public API.

**Phase 4.C state.** Clarabel v0.9 is the vetted permissive-licence
conic solver (`docs/clarabel-vetting.md`). The `mpc` feature exposes
deterministic QP / SOCP primitives and smoke tests. Full
receding-horizon MPC and LCvxLD / SCvx trajectory reproduction remain
Phase 5 research scope.

**Tasks.**
- QP / SOCP solver vetting: candidate crates evaluated against
  permissive license, no system-time dep, no allocation on solve loop,
  audit status. In-house implementation if no acceptable solver exists.
- `openbmp-fc/src/mpc/`: convex-QP-based MPC for attitude tracking
  and trim hold. Reuses the same effector / engine plant the autopilot
  consumes. Slots into the bus via `actuator_cmd` topic.
- `openbmp-fc/src/landing/`: lossless-convexification soft-landing
  (LCvxLD, Acikmese & Ploen 2007) and SCvx successive-convexification
  variant for academic powered-descent studies.
  Scenario-defined landing site, not a real-world target.

**Effort.** Large (~4.0 weeks combined; QP-solver vetting is the long
pole).

## Risk Register

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| `openbmp-fc` accidentally imports `openbmp-sim` | Medium | High | Sub-phase exit checklist runs `cargo build -p openbmp-fc --no-default-features`; `openbmp-testkit` dependency tripwire flags forbidden controller edges. |
| `openbmp-fc` accidentally calls `std::time::Instant` / `SystemTime` | Medium | High | Workspace clippy `disallowed_methods` lint + `openbmp-testkit` source-grep tripwire. The lockstep contract is brittle; a single accidental call in a deeply-nested utility breaks SITL replay. |
| Bus-topic registry races between modules under future multi-rate kernel | Low | High | Single-threaded discipline preserved; the bus is single-writer-many-reader by construction; topic ordering deterministic via priority + registration order. |
| Cyclic scheduler over-budget jobs slip silently | Medium | Medium | Scheduler emits `overrun_event` on every slip; commander treats sustained overruns as a failsafe condition. |
| Commander state machine deadlocks (no transition fires for an expected condition) | Medium | High | Property tests on the `MissionPhaseGraph` reachability invariants; CI exercises every phase transition path in the closed-loop validation scenario. |
| Voter pattern feels "over-engineered for one IMU" and is removed | Low | High | The trait is ~30 lines; cost is negligible and the seam is the difference between Phase-5 triplex being a config change vs. a refactor. Document the rationale prominently in `voter/mod.rs`. |
| Parameter registry codegen complexity blows the budget | Medium | Medium | Start with hand-written `Params` struct + TOML-deserialise; defer macro/codegen to Phase 5 if it's not critical. |
| Log-replay determinism guarantee is weaker than expected (state-stable but not bit-stable) | Medium | Medium | State-stable is the merge gate; bit-stable is a stretch goal with a documented reason if missed. PX4 community has the same posture. |
| EKF / MEKF / UKF accumulate non-bit-stable state across reruns | Medium | High | Determinism tests on each filter equivalent to the kernel-level twin-run gate. Locked operand order on every `predict`/`update` op. |
| Closed-loop validation case picks a reference with insufficient controller documentation | Medium | Medium | Pre-pick the reference (RocketPy attitude controller is a known candidate) before 4.9 starts; spike a quick comparison run during 4.5. |
| QP solver vetting blocks 4.11 indefinitely | Medium | Low | 4.11 is explicitly stretch; deferring to Phase 5 is acceptable. Gate the decision early. |
| Phase 4 motivates someone to add real-RTOS targets | Low | High | Safety-boundaries `Reject` list still applies. Real RTOS ports, real bus protocols, real device drivers do not land in this repository regardless of how convincing the use case sounds. The framing means deployable-grade *architecture*, not deployable *qualification*. |
| Reframe causes safety-boundary drift (architecture-grade autopilot → deployable autopilot) | Medium | High | The safety-boundary doc (`docs/safety-boundaries.md`) is unchanged. Every release artifact carries the non-suitability disclaimer. The `crates/openbmp-fc/README.md` and `docs/software-architecture.md` continue to label the binary "simulator-local" and "not validated for deployment". |

## Acceptance Gate for Phase 4 Closure

Phase 4 closes when **all** of the following are true:

1. Every sub-phase 4.1–4.10 exit criterion is met. (4.11 is stretch
   and not gating.)
2. `cargo fmt --all -- --check` clean.
3. `cargo clippy --workspace --all-targets --all-features -- -D
   warnings` clean.
4. `cargo test --workspace --all-features` green.
5. `cargo build -p openbmp-fc --no-default-features` compiles without
   any synthetic-feature crates active. The HAL-portability contract
   holds.
6. `crates/openbmp-fc/Cargo.toml` carries no `openbmp-sim`,
   `openbmp-cli`, `openbmp-scenario`, `openbmp-telemetry`,
   `openbmp-bridge`, `openbmp-aerothermal`, or `openbmp-env`
   dependency; the `openbmp-testkit` dependency tripwire enforces this.
7. The lockstep clock contract holds: `std::time::*` source-grep
   tripwire passes; the workspace `disallowed_methods` clippy lint
   covers `Instant::now` and `SystemTime::now` in `openbmp-fc`.
8. Cyclic scheduler overrun-catch test fires in CI under a deliberate
   over-budget job injection.
9. The Phase-1 `constant-acceleration-drop`, Phase-2
   `niskanen-2009-chapter6`, and Phase-3 `rocketpy-calisto` scenarios
   continue to produce byte-identical Parquet across two reruns.
10. The Phase-4 closed-loop validation scenario passes its
    academic-tolerance gate.
11. The Phase-4 log-replay determinism gate runs the closed-loop
    scenario, replays the recorded log, and asserts state-stable
    actuator-stream equivalence. (Bit-stable is a stretch goal.)
12. The CI determinism gate runs all four scenarios twice on
    `x86_64-unknown-linux-gnu`.
13. Every new dataset file (controller gains, reference profiles,
    parameter defaults, table fixtures) has a sibling `provenance.md`
    and `openbmp check-provenance` passes.
14. The build emits a `dictionary.json` artifact alongside the binary,
    listing every command, event, telemetry channel, and parameter
    the autopilot exposes; CI asserts a non-empty dictionary.
15. `docs/design-concept.md § Phase Roadmap` lists Phase 4 in past
    tense with the actual deliverables.
16. `docs/phase-4-plan.md` is removed and `docs/README.md` no longer
    links to it.

## Suggested Calendar

| # | Sub-phase | Weeks |
|---|---|---|
| 4.1 | Autopilot binary skeleton | 2.0 |
| 4.2 | Sensor ingest + voter | 1.0 |
| 4.3 | Estimator framework + EKF | 2.0 |
| 4.4 | Commander + mission FSM | 1.0 |
| 4.5 | Three-loop autopilot | 2.0 |
| 4.6 | Health & arming + FDIR | 1.5 |
| 4.7 | Academic guidance laws | 1.0 |
| 4.8 | MEKF + UKF | 1.5 |
| 4.9 | Closed-loop validation + replay | 2.0 |
| 4.10 | Phase 4 closure | 0.3 |
| **Subtotal (gating)** | | **14.3** |
| 4.11 | (Stretch) MPC + powered-descent | 4.0 |

With a ~30 % slip buffer for audit follow-ups, that puts a realistic
Phase-4 closure at **~5–5.5 calendar months** of focused work for the
gating sub-phases (4.1–4.10), and **~7 calendar months** if 4.11
lands.

## Changes from the previous draft

For maintainers reviewing the diff against the earlier "virtual flight
controller framework" draft:

1. **Goal reframed.** From "virtual flight controller framework" to
   "autopilot architecture with deployable-grade discipline,
   validated in SITL only". Safety boundary unchanged; what changed
   is the *engineering posture* on the architecture itself.
2. **New sub-phase 4.1 — Autopilot binary skeleton.** Bus,
   scheduler, clock contract, parameters, tables, dictionary. Lands
   before any algorithm.
3. **New sub-phase 4.2 — Sensor ingest + voter.** Voter trait exists
   even with simplex sensors; future triplex is a config change.
4. **New sub-phase 4.4 — Commander.** Single state-machine owner,
   arming chain, phase-gated actuator mixer. Combines what was the
   previous draft's 4.4 (Mission FSM) with the missing Commander
   abstraction.
5. **Combined 4.6 — Health & arming + FDIR.** Failsafe flags +
   residual-based detection + commander-mode-swap recovery.
6. **Combined 4.9 — Closed-loop validation + log-replay.** State-
   stable replay is the merge gate; bit-stable replay is a stretch.
7. **MPC + powered-descent demoted to stretch 4.11.** Both depend on
   QP-solver vetting that is high-risk and outside the architecture
   work; gating Phase 4 closure on them is too risky.
8. **New architectural contract — lockstep clock.** `std::time::*`
   banned in `openbmp-fc`; `Clock` trait is the only time source.
9. **New architectural contract — SITL discipline.** No
   `cfg!(simulator)` in `openbmp-fc`. Sim drives behaviour entirely
   through bus topics.
10. **New acceptance gates.** Cyclic-scheduler overrun-catch test;
    log-replay determinism gate; dictionary-artifact build check.
11. **Risk register expanded.** Lockstep contract violation,
    scheduler overrun, commander deadlock, voter removal, replay
    determinism downgrade, reframe-induced safety-boundary drift —
    all documented with mitigations.
12. **Calendar grew from 13.8 to 14.3 gating weeks.** The skeleton
    sub-phase adds ~2 weeks; consolidation of FDIR with health and
    of replay with closed-loop validation recovers ~1.5 weeks; net
    +0.5 weeks. The 4.11 stretch adds another 4 if pursued.
