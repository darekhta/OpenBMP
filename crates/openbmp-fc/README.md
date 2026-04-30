# openbmp-fc

L4 flight controller.

**Status:** Phase 4 — in progress.

## Purpose

Simulator-local autopilot composed of an internal pub/sub bus, a
cyclic scheduler with budget enforcement, a commander as the single
state-machine owner, a parameter registry, a sensor voter, a health /
arming gate, a phase-gated actuator mixer, and the academic
algorithms (estimator / autopilot / mission FSM / guidance / FDIR).
Outputs are abstract normalized commands consumed by simulator-local
actuator models or the optional generic socket bridge. **No real
hardware protocols, no targeting, no terminal-homing.**

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
├── voter          # N-of-M sensor voter (simplex / triplex / weighted-mean)
├── sensor_ingest  # per-sensor-kind ingest jobs that publish to the bus
├── estimator      # Estimator trait + EKF (15-state error-state) + MEKF
├── magnetic       # MagneticFieldModel trait + EarthDipoleField (academic)
├── commander      # mission FSM + arming chain
├── autopilot      # three-loop autopilot, gain-scheduled
├── mixer          # phase-gated actuator authority
├── health         # failsafe-flag publisher (bus-sequence staleness)
├── fdir           # residual-based detection (publishes status)
├── guidance       # attitude-hold + waypoint-track guidance
├── replay         # in-memory bus recorder/replayer for state-stable replay
└── controller     # FlightController façade (composes everything)
```

### Phase 4.C deferrals

Not shipped in this crate today (gated on a vetted permissive-licence
solver passing `cargo deny` review):

- **UKF** — a real sigma-point unscented Kalman filter (the prior
  `Ukf` scaffold propagated only the mean and inflated the diagonal —
  that is an EKF, so it was deleted rather than left to mislead).
- **MPC** — a real receding-horizon convex-QP-driven controller. The
  `LqrAttitudeMpc` scaffold was constant-gain LQR; deleted.
- **LCvxLD / SCvx powered descent** — real lossless-convexification
  / successive-convexification soft-landing. The `Lcvxld` / `Scvx`
  scaffolds were single-step PD controllers; deleted.
- **Full WMM 2025** — 12-degree spherical-harmonic geomagnetic field
  with the COF dataset. The shipped `EarthDipoleField` is the
  degree-1 truncation, marked `validated-toy`.

## Inputs and Outputs

Inputs: bus topics — `sensor.imu`, `sensor.barometer`, `sensor.gnss`,
`sensor.magnetometer`, `sensor.star_tracker`, plus
`guidance.reference` from a guidance job.

Outputs: bus topics — `autopilot.actuator_cmd`,
`autopilot.engine_cmd` (after the mixer's phase-gated republish),
`commander.vehicle_status`, `health.failsafe_flags`,
`estimator.attitude`, `estimator.position`, `estimator.status`,
`fdir.status`.

The simulator's runner subscribes to `autopilot.actuator_cmd` /
`autopilot.engine_cmd` and converts those into the
`ControlEffector::step` / `EngineModel::apply_command` calls the
kernel uses.

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

`experimental`. Phase 4 validation uses analytic attitude/rate
tracking and synthetic-sensor scenarios; the closed-loop validation
case (Phase 4.9) compares against a public reference within an
academic tolerance.

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
