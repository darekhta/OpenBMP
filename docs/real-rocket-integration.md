# OpenBMP Real-Rocket Integration Cookbook

This document walks downstream consumers through the pattern for plugging
real-rocket data and custom models into OpenBMP. It uses a wholly
fictional reusable launch vehicle, **ARV-Reference** (Academic Reusable
Vehicle, Reference Configuration), as the worked example so the integration
pattern is demonstrable without naming or referencing any specific operator,
manufacturer, engine, or fielded vehicle.

A brief final section sketches what changes for a public-data integration
of an *externally observable* vehicle class — see § "Public-data integration
for observed vehicles" for that note.

## What OpenBMP is, and what it is not

OpenBMP is a **trait scaffold + deterministic kernel + tooling**. It ships:

- Trait surfaces for every physics layer (force, mass, environment,
  vehicle assembly, propulsion, effectors, sensors, FC, stop conditions,
  events, integrators).
- Reference implementations that use only **synthetic, public-standard,
  or textbook** data with full provenance.
- A bit-stable simulation kernel + telemetry archive.
- CI gates, validation framework, and a credibility-product workflow.

OpenBMP does **not** ship:

- Real fielded-vehicle parameter sets (aero decks, motor maps, sensor
  noise budgets, controller gains).
- Real device drivers (no MAVLink, CAN, MIL-STD-1553, board support
  packages).
- Real-time deployment or operational command links.
- Hardware-specific HIL adapters (the `openbmp-bridge` ships only a
  generic in-house socket transport — see § "Optional: generic lab
  HIL bridge").

The boundary is explicit: **OpenBMP's safety stance applies to the
OpenBMP repository**. Downstream consumers integrating real data,
custom models, or lab hardware do that in their own repositories under
their own provenance, export-control, and qualification posture. See
[`safety-boundaries.md`](safety-boundaries.md) and
[`software-architecture.md` § Extensibility for Downstream
Integration](software-architecture.md#extensibility-for-downstream-integration).

## The integration model

```text
┌──────────────────────────────────────────────────────────────────┐
│  OpenBMP (this repository)                                       │
│  ──────────────────────                                          │
│  • Trait surfaces                                                │
│  • Synthetic / textbook reference impls                          │
│  • Deterministic kernel + integrators                            │
│  • Telemetry + scenario format + CLI                             │
│  • CI gates, validation framework, credibility workflow          │
└──────────────────────────────────────────────────────────────────┘
                          ↑   depends on, via crates.io / git
                          │
┌──────────────────────────────────────────────────────────────────┐
│  Your downstream repository                                      │
│  ──────────────────────────                                      │
│  • Custom trait impls (your aero, propulsion, effectors,         │
│    sensors, autopilot, mission state machine)                    │
│  • Real-data packages (with credibility metadata, see below)     │
│  • Optional: lab HIL adapter for your test rig                   │
│  • Validation harness against your references                    │
│  • Your provenance + export-control + qualification posture      │
└──────────────────────────────────────────────────────────────────┘
```

OpenBMP defines the contract; you provide the content.

## The `VehicleAssembly` tree

For anything beyond a single point-mass or single rigid body, OpenBMP
exposes a **`VehicleAssembly`** abstraction — a tree of components that
together describe a vehicle. The tree's branches are trait-bounded so
each layer is independently swappable.

```text
VehicleAssembly
├── Bodies         — one or more rigid bodies (e.g. booster, upper stage,
│                    fairing, payload). Each body has its own state
│                    propagated by the kernel.
├── Propulsion     — engine clusters (one logical cluster per powered
│                    body). Each engine has throttle, gimbal, ignition
│                    state, mounting geometry, and limits.
├── Effectors      — control effectors (flaps, fins, gimbals, RCS thrusters)
│                    with rate limits, saturation, deadband, latency, and
│                    failure modes.
├── Tanks          — propellant containers. Optionally coupled to
│                    MovingMassModel implementations (pendulum / modal /
│                    lumped slosh) that perturb mass-properties as the
│                    vehicle accelerates.
├── Sensors        — synthetic sensors per body. Real sensor noise budgets
│                    come from the user's data package.
└── MassProperties — computed from constituent parts each step:
                     base body mass + tank contents (with moving-mass
                     perturbation) + payload + propellant burn evolution.
```

The `VehicleAssembly` is the central abstraction the kernel queries each
step to learn:

1. *Which bodies exist right now?* (Pre-stage-separation: one assembly with
   booster + upper-stage. Post-separation: two independent assemblies.)
2. *What forces and moments do the propulsion + effectors apply?*
3. *What is the current mass-properties of each body?* (After this step's
   propellant burn, after this step's slosh perturbation.)
4. *What sensor measurements are available?*

Trait sketch (illustrative; full definitions live in
[`software-architecture.md`](software-architecture.md) once Phase 3 begins):

```rust
pub trait VehicleAssembly {
    type Body: SimState;

    fn bodies(&self) -> &[Self::Body];
    fn propulsion(&self, body_id: BodyId) -> Option<&dyn EngineCluster>;
    fn effectors(&self, body_id: BodyId) -> &[Box<dyn ControlEffector>];
    fn tanks(&self, body_id: BodyId) -> &[Box<dyn TankModel>];
    fn sensors(&self, body_id: BodyId) -> &[Box<dyn SyntheticSensor>];
    fn mass_properties(&self, body_id: BodyId, t: SimTime) -> MassProperties;

    /// Apply a deterministic event (ignition, MECO, separation, ...).
    fn handle_event(&mut self, event: &VehicleEvent) -> Result<(), VehicleError>;
}
```

## The event / phase model

Real launch + reusable-recovery scenarios are **event-driven**: ignition
sequences, throttle profile segments, MECO, hot-stage / cold-stage
separation, second-stage engine relight, entry interface crossing, flap
mode transitions, flip / pitch-around, landing-burn ignition, landing-burn
cutoff, touchdown.

OpenBMP's kernel treats these as a **first-class deterministic event
timeline**. Events are scheduled either:

- **Time-triggered** (T+12.7 s: launch-table clamp release; T+158 s:
  MECO; T+162 s: separation), or
- **State-triggered** (when `altitude > 80 km`: entry interface event;
  when `velocity_z < 50 m/s`: landing-burn ignition).

The event timeline is **part of the scenario file** and is validated for
ordering, monotonicity, and safety-naming at parse time.

```toml
[[events]]
trigger = "time"
at_s = 12.7
event = "engine-cluster-ignition"
target = "booster.main-cluster"
parameters = { engines = "all", throttle = 0.65 }

[[events]]
trigger = "state"
condition = "altitude_geodetic_m > 80000.0"
event = "phase-transition"
parameters = { from = "ascent", to = "entry-interface" }
```

Multi-body staging is a special event with momentum-balance bookkeeping:

```toml
[[events]]
trigger = "time"
at_s = 162.0
event = "stage-separation"
parameters = { separating = "upper-stage", separating_from = "booster",
               impulse_n_s = 50000.0, impulse_axis_body = [0, 0, 1] }
```

Post-separation, the kernel propagates two independent
`VehicleAssembly` trees with their own state, propulsion, effectors,
mass-properties, and sensors.

## The real-data package — a credibility contract

The single biggest difference between an academic loader and a serious
simulation input pipeline is **what metadata the data carries**. Every
real-data package consumed by an OpenBMP downstream integration must
declare the following, in a sidecar manifest:

```yaml
# File: data/aero/arv_reference_v1.openbmp-package.yaml
package_id: my-org.arv-reference.aero-deck.v1
package_kind: aero-deck

# UNITS — every numeric column declares units explicitly
units:
  mach: dimensionless
  alpha: rad
  beta: rad
  delta_flap_left: rad
  delta_flap_right: rad
  cn: dimensionless
  cm: dimensionless
  cd: dimensionless

# FRAMES — every vector / coefficient declares frame + origin
frames:
  coefficient_frame: body
  reference_origin: nose-tip            # documented body-frame origin
  body_axes: aerospace-standard          # x out the nose, y right, z down

# REFERENCE GEOMETRY — what the dimensionless coefficients are normalised to
reference_geometry:
  length_m: 0.5
  area_m2: 0.196
  span_m: 0.85

# INTERPOLATION POLICY
interpolation:
  method: trilinear                      # | nearest | spline | gp_surrogate
  out_of_grid: fail-closed               # | clamp | extrapolate-warned

# EXTRAPOLATION BEHAVIOUR (only when out_of_grid != fail-closed)
extrapolation:
  policy: clamp-to-nearest
  bounds_relaxation_factor: 1.1

# VALIDITY ENVELOPE — outside these bounds, the package self-reports invalid
validity_envelope:
  mach: [0.0, 25.0]
  alpha_deg: [-15.0, 90.0]
  beta_deg: [-10.0, 10.0]
  delta_flap_deg: [-30.0, 30.0]

# PROVENANCE — required per docs/data-provenance.md
provenance:
  source_class: external-generated       # see data-provenance.md taxonomy
  generator_id: cfd-toolkit-vX.Y
  generator_inputs_hash: blake3-7f3e...
  retrieved_utc: 2026-04-01
  source_hash: blake3-9b21...
  reviewer: dr-jane-smith
  reviewer_decision: accepted

# UNCERTAINTY — quantitative, machine-readable
uncertainty:
  cn_two_sigma: 0.05                     # uniform; or per-cell grid
  cm_two_sigma: 0.03
  uncertainty_method: cfd-grid-convergence-ras-2-sigma

# VERSIONING
version: 1
package_hash: blake3-c4e7...             # full-package content hash
schema_version: 1                        # OpenBMP package-format schema

# CROSS-REFERENCES (optional)
related_packages:
  - my-org.arv-reference.mass-properties.v1
  - my-org.arv-reference.engine-map.v1

# CREDIBILITY METADATA (optional but recommended)
# Per NASA-STD-7009B credibility factors (0-4 scale per factor)
credibility:
  verification: 3                        # code + algorithm verification
  validation: 2                          # comparison vs. reference data
  input_pedigree: 3                      # data quality + provenance
  results_uncertainty: 3                 # quantified uncertainty
  results_robustness: 2                  # sensitivity to inputs
  use_history: 1                         # how widely / how long used
  ms_management: 3                       # configuration management
  people_qualifications: 4               # subject-matter-expert review
```

The OpenBMP scenario parser **rejects** any data file referenced from
`scenarios/` without a parseable manifest, and the runtime **refuses to
extrapolate beyond `validity_envelope`** unless the package opts in
explicitly.

This is the credibility contract. It is the difference between "we
loaded a CSV" and "we are simulating with a known-quality dataset whose
limits we respect."

## `ControlEffector`: actuator-level modeling

Control surfaces (flaps, fins, RCS thrusters) and engine gimbals share a
common pattern: **command in, state out, with limits and faults**. They
are modeled separately from the aero deck because the aero deck answers
"given these effector positions, what are the coefficients?" while the
effector models answer "given this command and the previous state, what
position can I actually be in?"

```rust
pub trait ControlEffector {
    type Command;
    type State;

    /// Step the effector forward by `dt` given the current command.
    /// Returns the achievable state respecting rate, saturation, deadband,
    /// latency, and any active fault mode.
    fn step(&mut self, command: Self::Command, dt: Duration) -> Self::State;

    /// Maximum rate of change of the effector state (e.g. rad/s).
    fn rate_limit(&self) -> f64;

    /// Hard saturation limits (e.g. (-30°, +30°) for a flap).
    fn saturation(&self) -> (f64, f64);

    /// Deadband around zero where small commands produce no motion.
    fn deadband(&self) -> f64;

    /// Command-to-state actuation latency.
    fn latency(&self) -> Duration;

    /// Active fault mode, if any (jam, stuck, runaway, drift).
    fn fault_mode(&self) -> Option<EffectorFault>;
}
```

The same trait covers:

- Aerodynamic flaps (e.g. body flaps on a reusable upper stage)
- Grid fins on a returning booster
- Engine TVC gimbals
- Reaction control system (RCS) thrusters
- Parachute / drogue deployment actuators

A `VehicleAssembly`'s effector list aggregates whatever the vehicle
needs, with each effector having its own rate, saturation, latency, and
fault model — which the autopilot must respect.

## `TankModel` + slosh as moving-mass

Slosh dynamics matter for any vehicle with a significant liquid
propellant inventory and aggressive attitude maneuvers — and they are
modelled as **moving-mass perturbations to the vehicle's mass
properties**, not as a one-off slosh-specific hack.

```rust
pub trait TankModel {
    /// Current propellant mass.
    fn propellant_mass_kg(&self, t: SimTime) -> f64;

    /// Mass-rate (negative for outflow during burn).
    fn mass_rate_kg_s(&self, t: SimTime) -> f64;

    /// Optional slosh dynamics — None for "frozen propellant"
    /// approximation (default for low-fidelity studies).
    fn slosh(&self) -> Option<&dyn MovingMassModel>;
}

pub trait MovingMassModel {
    /// Step the moving mass forward given the body-frame acceleration
    /// the tank is experiencing.
    fn step(&mut self, body_accel: Vector3<f64>, dt: Duration);

    /// Current centre-of-mass offset in the tank frame (perturbation
    /// from the geometric centre).
    fn com_offset_tank(&self) -> Vector3<f64>;

    /// Inertia tensor perturbation in the tank frame.
    fn inertia_perturbation_tank(&self) -> Matrix3<f64>;

    /// Validity envelope (frequencies, amplitudes, accelerations).
    fn validity_envelope(&self) -> SloshValidityEnvelope;
}
```

OpenBMP ships generic `MovingMassModel` reference implementations:

- `FrozenPropellant` — no slosh, useful as a baseline
- `EquivalentPendulumSlosh` — Abramson-style classical pendulum analogue
  (NASA SP-106, public)
- `EquivalentSpringMassDamperSlosh` — for axisymmetric tanks under
  near-constant acceleration
- `ModalSlosh` — modal-decomposition of CFD-generated mode shapes (user
  supplies the mode shapes via a real-data package)

Downstream consumers can implement their own (e.g., reduced-order models
trained from CFD) without modifying OpenBMP itself. The trait is the
contract; the data is yours.

## Multi-rate scheduling

Real flight software runs subsystems at different rates: sensors at
hundreds of Hz, attitude control at tens of Hz, position control at
single-digit Hz, telemetry at a few Hz. OpenBMP's kernel supports this
via **rate groups**, integer divisors of a base micro-tick:

```toml
[scheduling]
base_tick_hz = 1000     # 1 kHz micro-tick — also the integrator step

[[scheduling.rate_groups]]
name = "imu-sample"
hz = 1000               # every micro-tick

[[scheduling.rate_groups]]
name = "attitude-control"
hz = 200                # every 5 micro-ticks

[[scheduling.rate_groups]]
name = "position-control"
hz = 50                 # every 20 micro-ticks

[[scheduling.rate_groups]]
name = "telemetry"
hz = 10                 # every 100 micro-ticks
```

Rate groups fire deterministically: `time mod period == 0` triggers the
group. Iteration order within a tick is by `(priority, registration)`,
both fixed at scenario start. This preserves bit-stable replay regardless
of which rate groups fire on a given tick.

## Working example: ARV-Reference

A wholly fictional reusable launch vehicle used as the integration
pattern's worked example. Two stages, methane / oxygen propellant
(unspecified ratio), four body-fixed flaps on the upper stage, four grid
fins on the booster.

### Project layout

```text
arv-reference-sim/                       ← your repository
├── Cargo.toml
├── README.md
├── src/
│   ├── lib.rs
│   ├── arv_assembly.rs                  ← impl VehicleAssembly
│   ├── arv_engine_cluster.rs            ← impl EngineCluster
│   ├── arv_flap.rs                      ← impl ControlEffector for body flaps
│   ├── arv_grid_fin.rs                  ← impl ControlEffector for grid fins
│   ├── arv_tank.rs                      ← impl TankModel + slosh
│   ├── arv_aero.rs                      ← deck loader
│   ├── arv_autopilot.rs                 ← impl Autopilot
│   ├── arv_mission_fsm.rs               ← impl MissionStateMachine
│   └── main.rs                          ← scenario runner
├── data/
│   ├── aero/
│   │   ├── arv_reference_v1.parquet
│   │   └── arv_reference_v1.openbmp-package.yaml
│   ├── propulsion/
│   │   ├── arv_engine_v1.parquet
│   │   └── arv_engine_v1.openbmp-package.yaml
│   ├── mass_properties/
│   │   ├── arv_booster_v1.parquet
│   │   └── arv_booster_v1.openbmp-package.yaml
│   └── slosh/
│       └── arv_tank_modes_v1.openbmp-package.yaml
├── scenarios/
│   ├── nominal_ascent.toml
│   ├── booster_return.toml
│   ├── upper_stage_entry.toml
│   └── engine_out_at_max_q.toml
└── tests/
    ├── trajectory_validation.rs         ← cross-validation against your refs
    └── credibility_audit.rs             ← runs the credibility scoring
```

### `Cargo.toml`

```toml
[package]
name = "arv-reference-sim"
version = "0.1.0"
edition = "2024"
license = "MIT OR Apache-2.0"

[dependencies]
openbmp-core       = "0.x"
openbmp-state      = "0.x"
openbmp-sim        = "0.x"
openbmp-aero       = "0.x"           # Phase 2+
openbmp-propulsion = "0.x"           # Phase 2+
openbmp-vehicle    = "0.x"           # Phase 3 (VehicleAssembly)
openbmp-sensors    = "0.x"           # Phase 2+
openbmp-fc         = "0.x"           # Phase 4
openbmp-scenario   = "0.x"           # Phase 1.5
openbmp-telemetry  = "0.x"           # Phase 1.4
openbmp-cli        = "0.x"           # Phase 1.7

# YOUR DATA STAYS IN YOUR REPO. OpenBMP never pulls it in.
```

### A minimal scenario file

```toml
openbmp.scenario = 1

[meta]
name = "arv-reference-nominal-ascent"
description = "Two-stage methalox academic reference vehicle, nominal ascent."
validation = "research"

[time]
start_s = 0.0
stop_s = 600.0
dt_s = 0.001
seed = 42

[scheduling]
base_tick_hz = 1000

[[scheduling.rate_groups]]
name = "imu"
hz = 1000

[[scheduling.rate_groups]]
name = "attitude-control"
hz = 200

[[scheduling.rate_groups]]
name = "position-control"
hz = 50

[[scheduling.rate_groups]]
name = "telemetry"
hz = 10

[vehicle.assembly]
kind = "arv-reference"                           # your custom assembly
configuration = "nominal-ascent"

[vehicle.aero]
deck = "data/aero/arv_reference_v1.parquet"
package = "data/aero/arv_reference_v1.openbmp-package.yaml"

[vehicle.propulsion.booster]
engine_cluster = "data/propulsion/arv_engine_v1.parquet"
package = "data/propulsion/arv_engine_v1.openbmp-package.yaml"
count = 9
mounting = "ring-9"

[vehicle.propulsion.upper-stage]
engine_cluster = "data/propulsion/arv_engine_v1.parquet"
package = "data/propulsion/arv_engine_v1.openbmp-package.yaml"
count = 1
mounting = "central"

# ... effectors, tanks, sensors, fc all reference your data + impls

[[events]]
trigger = "time"
at_s = 0.0
event = "engine-cluster-ignition"
target = "booster.main-cluster"
parameters = { engines = "all", throttle = 0.65 }

[[events]]
trigger = "state"
condition = "booster.fuel_mass_kg < 5000.0"
event = "stage-meco"
target = "booster.main-cluster"

[[events]]
trigger = "time-after"
of = "stage-meco"
delay_s = 4.0
event = "stage-separation"
parameters = { separating = "upper-stage", separating_from = "booster" }

[telemetry]
output.parquet = "out/nominal_ascent.parquet"
sample_rate_hz = 10

[validation]
require_finite_state = true
require_normalized_quaternions = true
require_positive_mass = true
```

### What you implement, what OpenBMP provides

You implement (in your repo):

- `VehicleAssembly` for ARV-Reference (booster, upper-stage, mass props,
  staging logic).
- `EngineCluster` reading your ENG-format propulsion package.
- `ControlEffector` for body flaps and grid fins.
- `TankModel` with your slosh model choice.
- `Autopilot` with your gain schedule.
- `MissionStateMachine` for your event sequencing.
- Validation tests against your reference (CFD, ground-test data,
  earlier flights, etc.).

OpenBMP provides:

- Deterministic kernel + RK4 / DOPRI integrators
- Event timeline + multi-body staging machinery
- Telemetry archive
- Scenario parser + safety-name lint
- Aerodynamic deck format + interpolation
- Tracing + tooling
- Cross-validation harness against in-tree analytic-toy + public-benchmark
  cases
- The CI gates (deny / vet / determinism / golden / typos / etc.)

## Cross-validation methodology

OpenBMP's validation framework follows two existing standards:

1. **NASA-STD-7009B (March 2024)** — *Standard for Models and
   Simulations*. Defines uniform practices for M&S with explicit
   acceptance criteria and a credibility-products framework. Eight
   credibility factors scored 0–4; OpenBMP's `credibility` block in the
   real-data package format maps onto these directly.
   <https://standards.nasa.gov/standard/NASA/NASA-STD-7009>
2. **AIAA G-077-1998** — *Guide for the Verification and Validation of
   Computational Fluid Dynamics Simulations*. Specifically scoped to
   CFD V&V terminology and uncertainty methodology; OpenBMP uses it for
   aero-deck and aerothermal-coefficient validation discussions.
   <https://netforum.aiaa.org/eweb/DynamicPage.aspx?WebCode=ProdDetailAdd&ivd_prc_prd_key=BA93ABAF-C987-4FCD-9471-C61E95860FBB>

Cross-validation tactics for downstream integrations:

| Reference | Use it to validate |
|---|---|
| **GMAT** (NASA, open) | Orbital propagation segments of your trajectory |
| **POST2** (NASA, open) | Atmospheric ascent trajectory |
| **RocketPy** (MIT) | Sounding-rocket-class trajectory and apogee |
| **JSBSim** (LGPL) | Atmospheric flight dynamics, control-surface decks |
| **Your wind-tunnel data** | Aero deck cells in the tested envelope |
| **Your ground-test stand data** | Engine performance, mass-flow, gimbal response |
| **Your earlier flight telemetry** (if available) | End-to-end trajectory, attitude, fuel margins |
| **Public-domain reference flights** | Apollo entry, Stardust SRC, IXV (publicly disclosed) |

Each cross-validation case ships its own `expected.toml` tolerance
table per [`verification.md`](verification.md), with
`absolute_tolerance` and `relative_tolerance` bounds derived from the
reference's known accuracy.

## Optional: generic lab HIL bridge

For HIL-style integration with a lab test rig (engine controller test
stand, sensor breadboard, GNC processor evaluation board, etc.), the
optional `openbmp-bridge` crate provides a **generic in-house socket
transport**:

```text
[Your lab test rig]  ←─ your protocol ──→  [your-bridge-adapter]
                                                   │  postcard / TCP
                                                   ↓
                                          [openbmp-bridge]
                                                   ↑
                                                   │  in-process traits
                                                   ↓
                                            [OpenBMP kernel]
```

The OpenBMP-side half (`openbmp-bridge`) is generic and ships
`postcard`-encoded `BridgeSensorPacket` and `BridgeCommandPacket` types.
**You write the lab-side adapter** that translates your specific test
rig's protocol to the in-house wire format. That adapter lives in your
repository, with your protocol stack, your timing posture, your
export-control compliance. OpenBMP does not ship adapters for any
specific bus, controller, vendor, or vehicle.

## Public-data integration for observed vehicles

Some downstream consumers will want to model an **externally observed**
vehicle from publicly available data only. The integration pattern is
the same as for ARV-Reference; what changes is the **data sourcing and
credibility scoring**:

- Use only **publicly disclosed regulatory documents** (FAA EISs / EAs,
  ICAO filings, environmental impact statements), **public press
  releases**, **academic estimation papers**, and **community
  photogrammetry / observation efforts**.
- Apply rigorous credibility scoring per NASA-STD-7009B credibility
  factors. Public-data packages typically score lower on `input_pedigree`
  and `results_robustness`, and that lower score must be visible in
  the package metadata.
- Cite the source for every coefficient and parameter in the
  `provenance` block.
- Validate against publicly observable trajectory data (from launch
  webcasts and regulatory filings) — note explicitly which flight tests
  the data was calibrated against.

A representative published example of this pattern in 2026 academic
literature is the *CEAS Space Journal* analysis using only public flight-
test data from a high-cadence reusable-launch programme to calibrate
mass / engine / aero estimates and validate predictions. Downstream
consumers replicating this pattern with OpenBMP get the kernel +
infrastructure for free; they own the data-quality story.

OpenBMP itself never ships parameter sets for any specific real vehicle,
operator, or programme. The framework remains EAR99-compatible and
strictly civilian-academic; downstream consumers' own integrations are
their own responsibility.

## Validation-against-public-references playbook

Recommended workflow for any downstream integration:

1. **Build it.** Write your trait impls, package your data with full
   credibility metadata, write your scenario.
2. **Run it.** `openbmp run my_scenario.toml --output out/run.parquet`.
3. **Diff it.** Use OpenBMP's golden-test workflow against committed
   reference outputs to confirm bit-stability across reruns.
4. **Cross-validate it.** Run the same scenario through GMAT / POST2 /
   RocketPy as appropriate; compare key metrics.
5. **Score it.** Fill out the credibility block; submit for peer review
   if publishing.
6. **Document its limits.** Validity envelope, known unmodelled physics,
   uncertainty bounds — all in the `expected.toml` and the package
   manifests.
7. **Cite it.** When publishing results, cite OpenBMP's commit hash,
   your data package hashes, and your reference comparison codes.

## Anti-patterns

Things downstream consumers should NOT do:

- Embed real fielded-vehicle parameters in upstream PRs to OpenBMP.
- Distribute aero decks or motor maps without provenance.
- Use OpenBMP as if it were certified flight software (it is not).
- Connect the optional bridge to operational command links for fielded
  hardware.
- Use OpenBMP-derived analyses as the sole basis for safety-of-flight
  decisions on real vehicles without independent qualification.
- Frame OpenBMP as endorsing or validating any specific operator or
  programme.

## References

- [`software-architecture.md`](software-architecture.md) — full trait
  specifications.
- [`safety-boundaries.md`](safety-boundaries.md) — what stays out of the
  OpenBMP repository.
- [`data-provenance.md`](data-provenance.md) — required provenance
  records (aligned with the package format above).
- [`verification.md`](verification.md) — V&V framework + tolerance
  tables.
- NASA-STD-7009B (March 2024). *Standard for Models and Simulations*.
  <https://standards.nasa.gov/standard/NASA/NASA-STD-7009>
- AIAA G-077-1998. *Guide for the Verification and Validation of
  Computational Fluid Dynamics Simulations*.
  <https://netforum.aiaa.org/eweb/DynamicPage.aspx?WebCode=ProdDetailAdd&ivd_prc_prd_key=BA93ABAF-C987-4FCD-9471-C61E95860FBB>
