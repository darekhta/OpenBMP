# openbmp-fc

L4 flight controller.

**Status:** Implemented — simulator-local validation only.

## Purpose

Simulator-local autopilot composed of an internal pub/sub bus, a
cyclic scheduler with budget enforcement, a commander as the single
state-machine owner, a parameter registry, a sensor voter, a health /
arming gate, a phase-gated actuator mixer, and the academic
algorithms (estimator / autopilot / mission FSM / guidance / FDIR).
Outputs are abstract semantic commands and optional `EffectorId` keyed
command sets consumed by simulator-local actuator models or the
optional generic socket bridge. **No real hardware protocols, no
targeting, no terminal-homing.**

The architecture is hardware-portable: the same binary, linked
against a downstream HAL crate that implements the `Sensor` /
`ControlEffector` / `EngineModel` traits from real hardware reads,
runs the same control flow on real hardware. The OpenBMP repository
itself ships only simulator-local validation and makes no compliance
claim under any safety-critical regime.

## Module map

```
openbmp-fc/
├── clock          # lockstep clock contract (std::time banned in this crate)
├── bus            # typed pub/sub topic registry (uORB-shaped)
├── scheduler      # cyclic dispatcher, budget enforcement, overrun events
├── params         # typed parameter sections
├── tables         # validated-then-activated typed tables
├── dictionary     # build-time JSON dictionary of topics/params/tables/jobs
├── topics         # canonical bus topics (sensor / estimator / commander / ...)
├── voter          # N-of-M voter (simplex / triplex / weighted / covariance)
├── sensor_ingest  # per-sensor-kind ingest jobs + lane status
├── estimator      # EKF/MEKF/UKF with Joseph updates + false-alarm gates;
│                  # gravity / magnetic models consumed from openbmp-physics
├── commander      # mission FSM + arming chain
├── autopilot      # three-loop autopilot, gain-scheduled
├── filters        # deterministic biquad / notch filters
├── mixer          # phase-gated actuator authority
├── health         # failsafe-flag publisher (bus-sequence staleness)
├── fdir           # residual-based detection (publishes status)
├── guidance       # attitude-hold + waypoint-track guidance
├── replay         # in-memory bus recorder/replayer for state-stable replay
├── mpc            # feature-gated Clarabel QP primitives
├── landing        # feature-gated Clarabel SOCP primitives
├── ud             # feature-gated UD covariance factor helper
├── l1_adaptive    # feature-gated L1 rate-loop augmentation
└── controller     # FlightController façade (composes everything)
```

### Implementation State

Implemented in this crate today:

- **Kernel↔FC runner bridge** — `[fc]` scenarios build `FcRunner`,
  prime synthetic sensors, step the controller lockstep from the
  point-mass or rigid-body runner, and feed effector / engine command
  sets back into the simulator racks.
- **Estimator upgrades** — EKF / MEKF Joseph updates, Markley-style
  MEKF covariance reset, Gauss-Markov bias dynamics, iterated
  magnetometer update, WGS84-J2 gravity via `openbmp-physics`, and a
  real 6-state sigma-point UKF for attitude + gyro bias.
- **Autopilot upgrades** — optional gyro notch filters,
  minimum-snap differential-flatness attitude-reference generation, and
  feature-gated Cao-Hovakimyan L1 adaptive rate-loop augmentation.
- **FDIR / voter upgrades** — burst-counter / single-sample GLRT /
  CUSUM detector
  families with explicit fault bits, per-kind sensor lane status, and
  covariance-weighted scalar voting.
- **Solver-backed primitives** — Clarabel v0.9 QP / SOCP smoke-tested
  behind the `mpc` feature. The vetting record is
  `docs/clarabel-vetting.md`.

Explicitly deferred to downstream work:

- NRLMSISE-00 upper atmosphere.
- Multi-instance estimator routing with active-lane selection.
- Full 15-state / square-root UKF.
- Full receding-horizon MPC and LCvxLD / SCvx trajectory reproduction
  against published powered-descent references.

Note: full WMM 2025 (`openbmp_physics::magnetic::Wmm2025`) is
available in the workspace via `openbmp-physics`; the FC's degree-1
`EarthDipoleField` placeholder is superseded for FC scenarios that
set `mag_field = "wmm_2025"` in `[fc.ekf]` or `[fc.mekf]`.

## Inputs and Outputs

Inputs: bus topics — `sensor.imu`, `sensor.barometer`, `sensor.gnss`,
`sensor.magnetometer`, `sensor.star_tracker`, plus
`sensor.status` from voted ingest jobs and `guidance.reference` from a
guidance job.

Outputs: bus topics — `autopilot.actuator_cmd`,
`actuator.effector_cmds`, `autopilot.engine_cmd` (after the mixer's
phase-gated republish), `commander.vehicle_status`,
`health.failsafe_flags`, `estimator.attitude`, `estimator.position`,
`estimator.status`, `fdir.status`.

The simulator's runner bridge subscribes to `actuator.effector_cmds`
and `actuator.engine_cmds`, then converts those into the
`ControlEffector::step` / `EngineModel::apply_command` calls used by
the kernel racks. When `[fc]` is absent, the bridge is a no-op.

## Units and Frames

Estimator state in `ECI` for position / velocity, body-frame for
attitude. Commands are typed angular deflections (rad) and unit
throttles in `[0, 1]`.

## Lockstep clock contract

`openbmp-fc` modules read time only via the injected `Clock` trait.
`std::time::Instant::now()` and `std::time::SystemTime::now()` are
banned crate-wide; the simulator's runner injects `Clock` from
`SimTime`, a downstream HAL injects from a hardware monotonic.

## Assumptions

All controller code is simulator-local. Gains, references, and
guidance modes are scenario-defined academic data, not operational
tuning or deployment logic.

## Validity Range

Each estimator, autopilot, and guidance mode declares its own rate,
state, sensor, and reference validity ranges. Unsupported phases or
references fail closed.

## Determinism

The bus, scheduler, parameters, and tables are single-threaded by
design; iteration order is registration order. The estimator's
predict/update math uses locked operand order. No wall-clock, no
system RNG, no allocation on the hot path.

## Validation

`experimental`. Validation uses analytic attitude/rate
tracking, synthetic-sensor scenarios, full bus-history determinism,
long-duration no-NaN / bounded-covariance checks, and textbook
Kalman-filter examples. External flight-stack trajectory
cross-validation is downstream work.

## Data Provenance

Any shipped gain sets or noise models are synthetic or textbook
examples with provenance. No real fielded controller gains ship in
this crate.

## Safety Boundary

Guidance laws shipped: `AttitudeHoldGuidance`, `WaypointGuidance`
(scenario-defined inertial waypoints).

**Explicitly NOT shipped:** proportional navigation, augmented PN,
sliding-mode terminal-homing, target-tracking guidance, intercept
geometry, terminal-mode autopilots (BTT/STT), TERCOM, DSMAC,
scene-matching. See `docs/safety-boundaries.md`.

## References

- PX4 Autopilot — uORB messaging pattern (re-implemented in Rust).
- ArduPilot — `AP_Scheduler` cyclic dispatcher pattern; `AP_HAL`
  separation between board-portable and board-specific code.
- NASA cFE — TBL validated-then-activated table pattern.
- F Prime (NASA JPL) — Commands / Events / Channels / Parameters
  vocabulary.
- Stevens & Lewis 2015. *Aircraft Control and Simulation.* Wiley.
- NaveGo (Rodríguez et al.); NorthStarUAS `insgnss_tools`; INSTINCT
  (U. Stuttgart) — 15-state error-state INS+GNSS Kalman formulation.
