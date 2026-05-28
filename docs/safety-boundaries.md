# OpenBMP Safety Boundaries

OpenBMP is an academic, simulation-only research platform for rigid-body
dynamics with a primary application to rocket / launch-vehicle / sounding-rocket
simulation. This checklist defines what maintainers should accept or reject
during design, review, and testing.

These boundaries are a **product requirement**, not a licensing note. The
codebase should make unsafe use difficult by avoiding hardware-facing
interfaces *in this repository*, by keeping every flight-controller API
the project itself ships consumable by simulator-local models, and by
never shipping real fielded-vehicle parameter sets.

The architecture is framed so the controller-side trait
surfaces (`Sensor`, `ControlEffector`, mission graph, model traits) are
re-implementable against real hardware via a downstream HAL. The
*project* still ships only the simulator and only validates against
academic / public-benchmark scenarios; *adopters* who write their own
HAL accept their own qualification and deployment posture. The list
below describes what the OpenBMP repository accepts or rejects — not
what downstream HAL adopters may build under their own qualification
posture in their own repositories.

## Project Definition

OpenBMP means **Open Body Motion Platform** in this repository.

The project may simulate generic virtual rigid bodies and rocket-class
vehicles with synthetic or public/educational parameters. It must not
become a weapon design tool, a deployable guidance stack, **or** a
distribution channel for restricted vehicle data. The OpenBMP
repository itself ships no hardware abstraction layer and no real
device drivers; downstream adopters who add a HAL on top of the
controller-side trait surfaces do so in their own repositories under
their own qualification posture.

## Accept

Accept contributions that are limited to:

### Dynamics and physics
- 3-DOF and 6-DOF rigid-body state propagation.
- Numerical integrators (RK4, DOPRI5/8, etc.) with documented order, step,
  stability, and reproducibility properties.
- Solver-profile infrastructure for research simulations, including explicit
  adaptive, implicit source-term, and fixed sub-step methods, when all profiles
  remain simulator-local and non-deployable.
- Coordinate frame transformations (ECI, ECEF, NED, ENU, body) implemented as
  type-tagged operations.
- Atmosphere abstractions, including in-house implementations of public
  models (e.g., US Standard Atmosphere 1976 and NRLMSISE-00, with public
  coefficients only).
- Gravity abstractions, including constant, point-mass, J2, and truncated
  spherical-harmonic expansions using public coefficients only (e.g., low-order
  EGM2008, with provenance).
- Wind models (constant, layered, gust) using documented academic profiles.
- Magnetic-field models when relevant, using public coefficients.
- Mass-property evolution (constant, linear-burn, table-driven from synthetic
  motor data).
- Aerodynamic decks expressed as tabular `(Mach, alpha, beta) -> coefficient`
  with synthetic, textbook, or public/educational parameters and explicit
  provenance.
- Propulsion models (solid, liquid, hybrid, cold-gas) with synthetic, textbook,
  or public/educational thrust curves and explicit provenance.

### Flight software (simulator-local only)
- Synthetic sensor models (IMU, barometer, GNSS, magnetometer, etc.) that
  generate simulated measurements from simulated truth, with documented
  academic noise/bias models. **No device drivers.**
- A virtual flight controller framework: sensor fusion (EKF, UKF, MEKF),
  attitude estimation, state estimation, autopilot loops, and **academic**
  guidance laws limited to attitude tracking, trajectory regulation against
  scripted reference states, and simple navigation between scenario waypoints.
- A launch-sequence / mission-phase state machine with academic phases
  (pre-launch, ascent, coast, apogee, descent, recovery) used **only** to
  drive the simulator's internal phase logic.
- A fault-detection-isolation-recovery (FDIR) framework that detects and
  responds to **simulated** faults injected by the scenario, used to study
  controller resilience.

### Test infrastructure
- Software-in-the-loop (SIL) tests where the virtual flight controller, models,
  and sensors run in-process with the kernel.
- Hardware-in-the-loop (HIL) **patterns** in the form of an optional socket
  bridge between the simulator and an external client, using an in-house
  wire format. The bridge is **a generic lab-HIL adapter owned by the
  downstream user**: OpenBMP ships only the simulator side of the socket
  and the abstract message schema. The bridge ships with **no real device
  drivers, no real bus protocols, no MAVLink/CAN/I2C/SPI integrations, no
  real flight-computer firmware, and no real sensor-bus glue**. It sends
  simulated sensor packets out and accepts abstract normalized command
  packets in. Any concrete protocol adapter that talks to physical
  hardware is the downstream user's responsibility under their own
  export-control and qualification posture, and lives in their own
  repository — not in OpenBMP.
- Deterministic test harnesses, golden telemetry, scenario fuzzing, property
  tests, snapshot tests, analytic-toy validation, and microbenchmarks.
- Read-only telemetry visualization and local playback tools that consume
  OpenBMP archives but do not send commands to a running simulator.

### Documentation
- Documentation of assumptions, units, frames, validity ranges, limitations,
  and validation status next to every model.
- Validation labels: `experimental`, `checked`, `validated-toy`, `research`.
- Provenance notes for any imported public data (atmosphere, gravity, motor
  database).

## Reject

Reject contributions that add or request:

### Targeting and operational behaviour
- Targeting, intercept, terminal guidance to specific real-world locations,
  route planning to specific targets, payload delivery, target-objective
  optimization, seeker integration, terminal-homing logic, or any guidance law
  whose stated purpose is to strike a real-world point.
- Any code or documentation labelled "Target", "Seeker", "Warhead", "Strike",
  "Interceptor", "Kill", "Threat", or equivalent operational terminology.
- Mission-planning tools for operational launches, weapon-employment scripts,
  field procedures, or launch-site procedures.
- Counter-defense, counter-radar, electronic-warfare, jamming, decoy, or
  detection-evasion logic.
- Terrain-contour-matching navigation libraries (TERCOM, DSMAC,
  scene-matching against real elevation data, optical-feature-based
  terminal homing). The relevant academic literature is operationally
  framed and OpenBMP explicitly does not implement, ship reference data
  for, or validate against any such algorithm.
- Terminal-homing aerodynamic / sensor / autopilot configurations:
  bank-to-turn (BTT), skid-to-turn (STT), or any explicitly-named
  "terminal-mode" autopilot variant whose stated purpose is end-game
  maneuvering.

> **Flight profiles — accepted boundary.** The multi-phase flight-profile work
> (see [`flight-profiles-architecture.md`](flight-profiles-architecture.md)) is
> deliberately on the accepted side of this line and reinforces it. Ascent and
> entry references are *reference-trajectory generation* (an inertial cutoff
> state or an entry-corridor limit band), never guidance to a location. The
> range-safety landing footprint is a *forward, offline* prediction of where an
> unpowered body comes down, in range-relative coordinates, for recovery and
> range-safety planning — it accepts no desired landing location, no aimpoint,
> and emits no steering command. Offline Monte-Carlo accuracy diagnostics are
> nominal-referenced sample statistics, never target-scored objectives. The
> offline staging analysis is likewise vehicle-intrinsic: it optimizes only an
> ideal ΔV / mass budget over `Isp`, structural coefficient, and payload mass,
> never range or trajectory to a location, and it emits report metadata rather
> than commands. The
> bank-angle modulation in a lifting entry is corridor-driven (heat-rate /
> load-factor limits), not a terminal-mode end-game variant. New aimpoint /
> miss-distance / terminal-guidance /
> bank-to-turn / skid-to-turn / intercept terms are added to the parse-time
> lint. See [`profile-vocabulary-and-guardrails.md`](profile-vocabulary-and-guardrails.md).

### Real fielded-vehicle data
- Aerodynamic decks for any operational missile or operational launch vehicle
  that are not openly published.
- Motor / engine performance data for any operational system that is not
  openly published.
- Mass properties, sensor parameters, or controller gains lifted from any
  fielded system.
- ITAR-restricted, MTCR Category I, EAR-restricted, or Wassenaar dual-use
  technical data.
- Performance envelopes, accuracy claims (CEP, miss distance), or operational
  range/payload claims for real systems.
- News-reported, journalist-reported, defence-industry-reported, or
  government-deftech-cluster-reported parameter sets for any current or
  recent operational vehicle programme, regardless of source country.
  Non-public reporting is rejected on provenance grounds; public reporting
  that mixes textbook physics with real-vehicle parameters is rejected on
  the parameter set, not the physics. Use textbook or synthetic data only.

### Hardware and deployment (project-side rejection list)

These rejections apply to **this repository**. Downstream adopters who
build a HAL against the controller-side trait surfaces (`Sensor`,
`ControlEffector`, mission graph, model traits) ship those integrations
in their own repositories under their own export-control and
qualification posture; OpenBMP itself does not host them.

- Real device drivers, board support packages, real bus protocols
  (CAN, MAVLink, DDS, MIL-STD-1553, etc.), real RTOS ports, real
  flight-computer firmware, or shipped embedded binaries — none of
  these land in the OpenBMP repository.
- Real-time scheduling guarantees, hard deadline assertions,
  deployable executive code, or anything that suggests *the OpenBMP
  binary itself* is suitable for flight on a real vehicle.
- Mission-control, command-and-control, or live-operations user
  interfaces that can command a real system or imply operational
  readiness from the OpenBMP binary. Read-only telemetry viewers are
  acceptable only as offline analysis tools.
- Claims that the OpenBMP-shipped binary or the OpenBMP-shipped
  validation evidence is suitable for operational flight, weapon
  development, field deployment, or live-fire testing.

## Review Questions

Before accepting a feature, answer:

- Can this run without any physical hardware?
- Does it avoid real fielded-vehicle parameters and operational performance
  claims?
- Within this repository, is every controller output consumed only by
  simulator-local models, or by the optional generic socket bridge in
  test scenarios? (Downstream HAL adopters wire to real hardware in
  their own repositories; that integration is out of scope for this
  question.)
- Does it avoid targeting, terminal homing to real-world locations, and
  payload-delivery behaviour?
- Is the feature useful for academic simulation even if all real-world
  vehicle data is removed?
- Are assumptions, units, frames, noise models, and validation status
  documented?
- Does the feature introduce hard real-time guarantees, real-bus
  protocols, or device-driver code into *this repository*? (If yes,
  reject — those belong in downstream HAL adopters' repositories.)

If any answer is "no" for the first six, or "yes" for the last, the feature
is outside the project boundary.

## Naming Rules

> **Cross-reference.** The mission-state vocabulary canon —
> the authoritative list of academic state names and the operational /
> engagement-derived terms that are rejected — is in
> [`mission-states-vocabulary.md`](mission-states-vocabulary.md). The
> rules below state the general workspace-wide naming posture; the
> mission-FSM vocabulary table is the load-bearing per-state canon.

Use names that reinforce simulation-only scope:

- Prefer: `RigidBody`, `Vehicle`, `Atmosphere`, `Gravity`, `Wind`, `Motor`,
  `AeroDeck`, `SyntheticSensor`, `Estimator`, `Autopilot` (in the
  `openbmp-fc` crate only, where the term is explicitly virtual), `Phase`,
  `MissionStateMachine`, `Scenario`, `Telemetry`, `Validation`, `FaultModel`.
- Avoid: `Target`, `Seeker`, `Warhead`, `Strike`, `Interceptor`, `Kill`,
  `Threat`, `Launch` (as a verb implying real launch), `Engagement`,
  `WeaponSystem`.
- The term `FlightController` is acceptable inside the `openbmp-fc` crate when
  the doc-comment makes it explicit that the implementation is a
  **simulator-local virtual flight controller, not deployable flight
  software**.

## Hypersonic Extensions

Hypersonic flight (Mach > 5) is a legitimate academic research domain
(NASA X-43 / X-51 publications, ESA IXV documentation, Apollo-class re-entry
data, Anderson "Hypersonic and High-Temperature Gas Dynamics", Vinh
"Hypersonic and Planetary Entry Flight Mechanics", Park "Nonequilibrium
Hypersonic Aerothermodynamics"). OpenBMP supports hypersonic extensions
under additional rules layered on the above accept/reject lists.

### Accept (hypersonic)

- Aerodynamic decks and methods extending to hypersonic Mach numbers using
  textbook or synthetic data.
- Modified-Newtonian, tangent-cone, tangent-wedge, and other local-inclination
  aerodynamic methods (textbook hypersonic aerodynamics).
- Hypersonic similarity scaling (`K = M·θ` etc.) for academic studies.
- Hypersonic solver profiles beyond RK4: high-order explicit trajectory
  integrators, dense-output event localization, implicit / IMEX source-term
  sub-steppers, deterministic coupling telemetry, and fail-closed convergence
  policies.
- High-altitude atmosphere models implemented in-house from public
  coefficients (NRLMSISE-00, JB-2008, MSIS-86), with provenance.
- Equilibrium and frozen real-gas thermodynamics from public correlations
  (e.g., Tannehill, Mugalev, Hansen) — academic / textbook use only.
- Park two-temperature **nonequilibrium** thermochemistry using public
  reaction sets (Park'87, Park'90, Park'93) and public Millikan-White /
  Park vibrational relaxation correlations. Reaction-rate tables ship with
  citation to the published source and a `provenance.md` entry.
- Generic textbook **surface ablation** toy with steady-state and charring
  variants. Materials shipped are parametric textbook archetypes
  (`generic-charring-1`, `graphite-toy`, etc.) drawn from open published
  academic ranges. Surface energy balance, blowing correction, recession
  integration, optional mass-loss coupling to vehicle mass model.
- Stagnation-point heating correlations: Fay-Riddell, Sutton-Graves,
  Tauber-Sutton — all unclassified textbook.
- Distributed surface heating from engineering correlations (reference
  enthalpy, Eckert, Spalding-Chi).
- Convective and radiative heating partition for academic study.
- Boundary-layer-state models (laminar, transitional, turbulent) with public
  correlations and transition criteria (e^N, Reθ/M_e).
- Continuum-to-rarefied bridging via Knudsen-number-based bridge functions.
- Free-molecular-flow aero coefficients for very-high-altitude regimes.
- Offline reference packages generated by public or downstream-owned CFD,
  DSMC, radiation, thermochemistry, thermal-response, or trajectory tools,
  provided they are used for academic validation / deck ingestion, include
  provenance and uncertainty metadata, and do not import restricted or
  operational vehicle data.
- Re-entry trajectory studies and academic skip-glide reference profiles
  driven by **scenario-defined waypoints in inertial space, not real-world
  locations**.
- Public re-entry validation cases: Allen-Eggers ballistic entry, Vinh
  lifting entry, Apollo-class blunt-cone re-entry (using publicly documented
  geometry and initial conditions).
- 1-D thermal-conduction "toy" surface-temperature evolution with textbook
  material properties.

### Reject (hypersonic)

- Real fielded **Hypersonic Glide Vehicle** (HGV) parameter sets, including
  but not limited to Avangard, DF-17 / DF-ZF, AGM-183 ARRW, HGB, common-HGV
  programs.
- Real fielded **Maneuvering Re-entry Vehicle** (MaRV) parameter sets.
- Real fielded hypersonic cruise weapon parameter sets, including but not
  limited to X-51 Waverider operational data, Tsirkon, Kinzhal, BrahMos-II.
- Plasma-sheath modeling for the purpose of analyzing radio-blackout
  exploitation, communication-window engineering, or radar cross-section
  reduction during re-entry.
- Re-entry maneuvering for impact dispersion, terminal evasion, or
  defense-penetration optimization.
- Decoy modeling, penetration aids, jammer integration, electronic-warfare
  payload modeling, or counter-defense logic at any phase of hypersonic
  flight.
- Targeting in any phase of hypersonic flight, including boost, midcourse,
  glide, or terminal phase.
- Skip-glide trajectories whose stated objective is to reach a real-world
  target location.
- Operational re-entry corridor planning for any specific weapon system.
- Real fielded thermal-protection-system material parameter sets. Named
  rejections include but are not limited to PICA, AVCOAT, RCC, SLA-561V,
  FRSI, AFRSI, LI-900, MA-25S, and any other operational TPS material.
  Generic textbook ablators are accepted; operational data is rejected.
- Operational tunings of Park-2T or any nonequilibrium reaction set
  calibrated to a specific fielded vehicle programme. Public textbook
  reaction sets are accepted; operational tunings are rejected.
- Specific TPS-design optimisation tools or sizing logic for any real
  vehicle programme.
- Automatic vehicle, TPS, trajectory, or control optimization for a specific
  real operational hypersonic vehicle.
- Coupled ablation-plasma chemistry modeling targeted at radio-blackout
  exploitation, RCS reduction, or any defense-penetration purpose.

### Naming additions for hypersonic

- Prefer: `RealGasState`, `NonequilibriumState`, `KnudsenRegime`,
  `ModifiedNewtonian`, `TangentCone`, `FayRiddell`, `SuttonGraves`,
  `BoundaryLayerState`, `EntryInterface`, `LiftingEntry`, `ReentryProfile`,
  `StagnationHeating`, `SurfaceHeating`, `ParkTwoTemperatureModel`,
  `MillikanWhite`, `SteadyStateAblator`, `CharringAblator`, `ToyAblator`,
  `RecessionRate`, `BlowingCorrection`.
- Avoid: `Glide` (without "Academic" qualifier), `Penetrate`, `Maneuvering`
  (in entry context, prefer `LiftingEntry`), `BlackoutEvasion`, `Decoy`,
  `PenAid`, `Endgame`, `TerminalDescent` (prefer the academic
  `DescentPhase` from the mission FSM), `TPS_<vehicle-name>`,
  `OperationalAblator`.

## Dependency Rule

OpenBMP may study public architecture patterns from aerospace tools (NASA cFS,
PX4, ArduPilot, F Prime, RocketPy, OpenRocket, NASA GMAT, etc.), but it must
not import controlled data, operational model parameters, or hardware
interfaces from any external project. All ported patterns must be reimplemented
in-house in Rust with explicit documentation of source patterns and applied
modifications.

For hypersonic data specifically: NRLMSISE-00, JB-2008, MSIS family, EGM
gravity coefficients, public real-gas reaction-rate datasets, and academic
re-entry reference cases are public and acceptable with provenance. ITAR-,
EAR-, MTCR-, or Wassenaar-restricted hypersonic technical data are
**rejected categorically** regardless of source.

News-reported or industry-reported information about specific real
vehicles or programmes — including corruption-investigation reporting,
production-rate reporting, government deftech-cluster announcements, and
recruiter / hiring-channel reporting — is not a permitted source of
OpenBMP technical content. Open academic publications, public NASA / ESA
/ NACA technical reports, civilian standards documents, and peer-reviewed
journal articles are the permitted reference frame.

## Public-Data Provenance

Any public dataset or coefficient ingested into the repository (atmosphere
models, gravity coefficients, motor curves, textbook aero examples) must
include a `provenance.md` entry naming the source, license, retrieval date, and
verification method. Provenance entries that fail review (export-controlled,
unclear license, unverifiable source) result in removal of the dataset.
The required record, source classes, and machine-check expectations are defined
in [data-provenance.md](data-provenance.md).

Accepted source classes are public standards, public academic references,
textbook-derived examples, synthetic OpenBMP data, and converted public data
with preserved provenance. Rejected source classes include unknown-source data,
operational reporting, restricted data, proprietary data, and any source whose
technical content is tied to a real fielded system outside the project scope.

## Non-Compliance Statement

OpenBMP is not validated under any of the following safety-critical
standards: IEC 61508 (functional safety, SIL 1–4), ISO 26262 (automotive
functional safety, ASIL A–D), DO-178C / DO-254 (airborne software /
hardware certification, DAL A–E), or any equivalent civilian or military
certification regime. OpenBMP is also not subject to MIL-STD-810
environmental qualification, MIL-STD-461 / MIL-STD-464 electromagnetic
qualification, or any production-quality assurance standard.

The simulator may be useful for teaching the concepts behind these
standards (hazard analysis, safety lifecycles, deterministic replay,
structured testing, requirement traceability), but its outputs are not a
substitute for any qualified flight-software stack. Downstream consumers
who integrate OpenBMP components into stacks subject to such regimes are
responsible for their own qualification work; OpenBMP itself makes no
compliance claims.

## Disclaimers

Every release artifact and the top-level README must include a
non-suitability statement:

> OpenBMP is an academic simulation platform. It is not validated for
> operational flight, not suitable for hardware deployment, and not a
> substitute for any qualified flight-software stack. No compliance
> claims are made under IEC 61508, ISO 26262, DO-178C, or equivalent
> regimes.
