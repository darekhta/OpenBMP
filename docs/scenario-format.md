# OpenBMP Scenario Format

OpenBMP scenarios are TOML-shaped, versioned configuration files. They define
models, initial state, deterministic schedule, telemetry outputs, and
validation rules for one simulation run.

The schema header is `openbmp.scenario = 1`. Phase 2 added optional blocks
(`[aero]`, `[propulsion]`, `[wind]`, `[atmosphere]`, `[frames.local_origin]`,
`[sensors]`) without bumping the schema version, so all Phase-1 scenarios
continue to parse byte-identically. Phase-2 specifics live in the
[Phase-2 Extensions](#phase-2-extensions) section at the bottom.

This document is the format contract. It intentionally favors strict, verbose
fields over compact syntax.

## Format Rules

- The root key `openbmp.scenario = 1` is mandatory.
- Unknown top-level tables and unknown fields are parse errors.
- All dimensional fields include units in the field name.
- All vector fields include frame names in the field name or table schema.
- Every model selected by name must resolve through the model registry.
- Every referenced file is resolved relative to the scenario file path.
- Every referenced file that comes from `data/` requires provenance.
- The parser fails closed on invalid values, NaN, infinity, negative mass,
  invalid rates, out-of-range enum values, and unsupported schema versions.

## Required Top-Level Tables

| Table | Required | Purpose |
|---|---|---|
| `[meta]` | yes | Name, description, validation label |
| `[time]` | yes | Start, stop, step, seed |
| `[vehicle]` | yes | Vehicle kind and initial state |
| `[environment]` | yes | Gravity, atmosphere, wind, magnetic models |
| `[forces]` | yes | Force and moment model ordering |
| `[telemetry]` | yes | Output files and schema options |
| `[validation]` | yes | Runtime validation rules |
| `[epoch]` | no | Absolute time metadata |
| `[frames]` | no | Frame profile and local origins |
| `[aero]` | no | External aerodynamic deck reference (Phase 2.10) |
| `[propulsion]` | no | Motor reference; ignition time (Phase 2.10) |
| `[wind]` | no | Structured wind block; overrides `environment.wind` (Phase 2.10) |
| `[atmosphere]` | no | Structured atmosphere block; overrides `environment.atmosphere` (Phase 2.10) |
| `[solver]` | no | Integrator / solver profile; required for Phase-6 hypersonic scenarios |
| `[data_packages]` | no | External real-data or high-fidelity reference package sidecars |
| `[sensors]` | no | Synthetic sensor models |
| `[fc]` | no | Simulator-local virtual flight controller |
| `[faults]` | no | Scenario-injected fault models |
| `[batch]` | no | Batch or Monte Carlo sweep metadata |

## Minimal Example

```toml
openbmp.scenario = 1

[meta]
name = "constant-acceleration-drop"
description = "Analytic toy scenario for vertical acceleration."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 10.0
dt_s = 0.01
seed = 42

[vehicle]
kind = "point_mass"
mass_kg = 1.0
initial_position_eci_m = [0.0, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 9.80665
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity"]

[telemetry]
output.csv = "out/constant-acceleration-drop.csv"
output.parquet = "out/constant-acceleration-drop.parquet"

[validation]
require_finite_state = true
require_monotonic_time = true
```

In Phase 1, `environment.gravity_m_s2` is a non-negative magnitude. The
analytic-toy runner applies it along the toy `-z` direction; explicit gravity
vectors are a later scenario-format extension.

Vehicle mass semantics depend on propulsion declaration in schema version 1.
Without `[propulsion.motor]`, `vehicle.mass_kg` is the total point mass. With a
motor block, `vehicle.mass_kg` is the dry airframe mass excluding the motor; the
Phase-2.11 runner adds the motor's time-varying mass from the pinned motor file.

## Metadata

`[meta]` fields:

| Field | Type | Required |
|---|---|---|
| `name` | string | yes |
| `description` | string | yes |
| `validation` | enum | yes |
| `provenance` | string | no |

Allowed `validation` values are `experimental`, `checked`,
`validated-toy`, and `research`.

## Time

`[time]` fields:

| Field | Type | Required | Rule |
|---|---|---|---|
| `start_s` | float | yes | finite |
| `stop_s` | float | yes | `stop_s > start_s` |
| `dt_s` | float | yes | positive finite |
| `seed` | integer | yes | unsigned 64-bit in memory; TOML integer literals are limited to the non-negative `i64` range until string seed parsing lands |

`dt_s` is the base kernel step. Multi-rate schedules are integer divisors of
the base step and must be declared under subsystem-specific `rate_hz` fields.

Phase-1 TOML seed literals should stay in `0..=i64::MAX`. The scenario model
stores seeds as `u64`, but TOML integer syntax itself cannot represent values
above signed 64-bit range portably.

## Solver Profile

`[solver]` is optional for Phase 1 and defaults to fixed-step RK4. Phase-6
hypersonic scenarios must declare it explicitly because solver choice,
tolerances, dense-output event policy, and source-term sub-stepping are part of
the validation claim.

```toml
[solver]
profile = "fixed-step-explicit"
# Allowed: fixed-step-explicit, adaptive-explicit, implicit-source-term,
# partitioned-hypersonic.
trajectory_method = "rk4"           # rk4 | dopri54 | dopri853 | rkf78
determinism = "bit-stable"          # bit-stable | state-stable

[solver.adaptive]
rtol = 1.0e-9
atol = 1.0e-12
min_dt_s = 1.0e-5
max_dt_s = 0.1
dense_output = true

[solver.source_terms]
chemistry_method = "implicit-euler" # implicit-euler | rosenbrock-wanner | bdf
chemistry_substeps = 16
material_method = "implicit-euler"
material_substeps = 8
nonlinear_tolerance = 1.0e-10
nonlinear_max_iter = 12
```

The parser rejects adaptive or implicit profiles without explicit tolerances.
For `bit-stable` scenarios, adaptive fields must be absent unless the method is
used only to generate a non-golden reference run.

## Data Packages

External data packages are declared by sidecar path. The loader validates hashes,
source class, units, frames, envelopes, uncertainty, credibility metadata, and
high-fidelity solver assumptions before the scenario starts.

```toml
[data_packages]
aero = "data/arv-reference/aero/data-package.yaml"
thermal_response = "data/external_refs/thermal_response/package.yaml"
trajectory_reference = "data/external_refs/trajectory/post2-like-reference.yaml"
```

Packages that include `downstream-private` source data must live outside the
OpenBMP repository and are the downstream user's compliance responsibility.

## Epoch and Frames

Absolute epoch data is optional. If present, it follows
[frames-time.md](frames-time.md).

```toml
[epoch]
scale = "UTC"
iso8601 = "2026-01-01T00:00:00Z"
leap_second_table = "data/time/leap_seconds_2026a.toml"

[frames]
profile = "wgs84-uniform-rotation"
```

Accepted frame profiles are `toy-fixed-earth`, `wgs84-uniform-rotation`,
`iers-tabulated`, and validation-only `spice-reference`.

## Model Ordering

The force model list is ordered and deterministic:

```toml
[forces]
models = ["gravity", "aero", "thrust"]
```

The kernel evaluates and sums force and moment providers in declared order.
Changing the order is a scenario change and can change golden telemetry.

## Safety-Limited Names

Scenario fields and model names must use academic vocabulary. The parser and
CI lint should reject names containing forbidden operational terms listed in
[safety-boundaries.md](safety-boundaries.md), including `target`, `seeker`,
`warhead`, `strike`, `interceptor`, `kill`, `threat`, `engagement`, and
terminal-homing equivalents.

Location-like fields are allowed only when their role is unambiguous:

- Accepted: `local_origin`, `entry_interface`, `recovery_area_toy`.
- Rejected: `target_location`, `impact_point`, `terminal_waypoint`,
  `strike_coordinate`.

## Batch Runs

Batch metadata is optional and never changes the meaning of a single scenario.

```toml
[batch]
manifest = "out/batch-manifest.toml"
run_id = "mc-000042"
worker_index = 7
worker_count = 32
```

A batch runner records:

- Base scenario hash.
- Parameter sweep definition.
- Seed allocation policy.
- Worker index and count.
- Output archive paths.
- Output hashes.
- Failed-run diagnostics.

Batch runs must not mutate the base scenario in place.

## Scenario Lint

Phase 1 should provide `openbmp check scenario.toml`. The check should:

- Parse with unknown-field rejection.
- Resolve all file references.
- Verify provenance exists for referenced data files.
- Check units and frame suffixes.
- Check safety-limited names.
- Check telemetry channel names and referenced model IDs for safety-limited
  vocabulary.
- Verify deterministic seed and schedule fields.
- Report all diagnostics before returning failure when possible.

The same check runs in CI for every committed scenario.

## Phase 2 Extensions

Phase 2.10 added scenario-format extensions for sounding-rocket scenarios.
Every new block is optional; existing Phase-1 scenarios parse unchanged.
The canonical worked example is
[`scenarios/sounding-rocket/niskanen-2009-chapter6.toml`](../scenarios/sounding-rocket/niskanen-2009-chapter6.toml).

### Rigid-body initial state

When `[vehicle].kind = "rigid_body"`, three additional fields are required:

```toml
[vehicle]
kind                                 = "rigid_body"
mass_kg                              = 1.5
initial_position_eci_m               = [0.0, 0.0, 0.0]
initial_velocity_eci_m_s             = [0.0, 0.0, 0.0]
initial_quaternion_body_to_eci_xyzw  = [0.0, 0.0, 0.0, 1.0]
initial_angular_velocity_body_rad_s  = [0.0, 0.0, 0.0]
inertia_tensor_body_kg_m2            = [
  [0.10, 0.0,  0.0],
  [0.0,  0.10, 0.0],
  [0.0,  0.0,  0.01],
]
```

The quaternion is the body-to-ECI rotation in `[x, y, z, w]` order;
the parser checks unit-norm to 1e-9. The inertia tensor is declared in
body axes as a 3x3 kg m^2 matrix; the parser requires finite symmetric
entries with strictly positive diagonal moments, and rigid-kernel
construction applies the full physical validity checks. The
rigid-body-only fields are rejected when `kind = "point_mass"`.

### Gravity coefficients

`[environment].gravity` selects one of `constant`, `point_mass`, or `j2`,
each with its own required-coefficient set:

| `gravity` | Required | Rejected |
|---|---|---|
| `"constant"` | `gravity_m_s2` | `mu_m3_s2`, `r_e_m`, `j2` |
| `"point_mass"` | `mu_m3_s2` | `gravity_m_s2`, `r_e_m`, `j2` |
| `"j2"` | `mu_m3_s2`, `r_e_m` | `gravity_m_s2` |

For `gravity = "j2"` the dimensionless `j2` coefficient defaults to the
WGS84 value (1.082626683 × 10⁻³) when omitted.

### Frames local origin

When `[frames]` is present, an optional local origin anchors NED wind,
altitude, and the WGS84 launch initialisation:

```toml
[frames]
profile = "wgs84-uniform-rotation"

[frames.local_origin]
latitude_deg  = 60.18
longitude_deg = 24.83
height_m      = 0.0
source        = "Helsinki proxy launch site (Niskanen 2009 Chapter 6)."
```

Latitude is in `[-90, 90]`, longitude in `[-180, 180]`. The `source`
field is a free-form provenance string for the declared origin and is
not optional. If both `[environment].frame_profile` and
`[frames].profile` are declared, they must match.

### Aero deck reference

```toml
[aero]
deck         = "../../data/aero/synthetic-niskanen-ch6-rocket.toml"
deck_sha256  = "cd862c2af98a1f28dc86c6e754d311c7a724081ca91b80704ad89b2ec4cb5c27"
```

`deck` is resolved relative to the scenario file directory. The optional
`deck_sha256` field pins the file's SHA-256 digest; mismatches fail
closed.

Two deck formats are supported. **Schema 1** is the Phase-2.5 three-axis
`(mach, alpha, beta) → (CN, CD, CM)` deck documented in
[`software-architecture.md § Deck Format`](software-architecture.md#deck-format-in-house-toml).
**Schema 2** (Phase 3.5) extends Schema 1 with optional control-effector
axes; see [Schema-2 aero decks](#schema-2-aero-decks-phase-35) below.
The `[aero]` block stays the same in both cases — it's just a reference
plus an optional digest pin. The schema discriminator lives inside the
deck file itself (`openbmp.aero_deck = 1` or `= 2`).

### Motor reference

```toml
[propulsion.motor]
file         = "../../data/motors/estes-c6-eng-derived.toml"
ignite_at_s  = 0.0
variant      = "solid"
file_sha256  = "da8272d3a7a135046c614e51b279971d37cac376f7aaaffdedc3ccc14d50ad4e"
```

`variant` is optional; when present, it must resolve to a registered
motor variant. Phase 2.11 wires only the `solid` motor variant.
`ignite_at_s` is the time since scenario start when the motor begins
burning; finite-required, no positivity rule (negative values are an
explicit pre-roll convention).

### Wind block

`[wind]` is the structured wind selection. When present, its `kind`
must agree with the flat `environment.wind` (unless the latter is
`"none"`, in which case the structured block wins):

```toml
[environment]
wind = "constant"

[wind]
kind         = "constant"
wind_ned_m_s = [3.0, 0.0, 0.0]
```

`wind_ned_m_s` is required when `kind = "constant"`, rejected
otherwise. Selecting `environment.wind = "constant"` therefore requires
the structured `[wind]` block.

### Atmosphere block

Symmetric to `[wind]`:

```toml
[environment]
atmosphere = "us_standard_1976"

[atmosphere]
kind = "us_standard_1976"
```

When `kind = "isothermal"`, the block requires `density_kg_m3`,
`pressure_pa`, and `temperature_k`, so selecting
`environment.atmosphere = "isothermal"` requires the structured
`[atmosphere]` block. For `us_standard_1976` no additional state is
required (the model is parameterless).

### Sensors

`[sensors.<name>]` declares one synthetic sensor per entry. The
`<name>` becomes the sensor's stable identifier; reordering entries
does not shift RNG streams (per Phase 2.7).

```toml
[sensors.imu]
kind         = "imu"
file         = "../../data/sensors/imu-tactical.toml"
file_sha256  = "537d60b57650f3d7569b338b6de2de97119b4401f2f244d173330be7aa37454e"

[sensors.barometer]
kind = "barometer"
file = "../../data/sensors/baro-consumer.toml"

[sensors.truth]
kind = "ideal_state"
```

`kind = "ideal_state"` carries no noise budget and rejects `file`;
`kind = "imu" | "barometer"` requires `file`.

### Force model registry (Phase 2)

Phase-2 force-model names accepted in `[forces].models`:

| Name | Source | Notes |
|---|---|---|
| `gravity` | `openbmp-env` | Constant, point-mass, or J2 (selected by `[environment].gravity`) |
| `aero` | `openbmp-aero` | Requires `[aero]` block |
| `thrust` | `openbmp-propulsion` | Requires `[propulsion.motor]` block |

The list is ordered and deterministic; reordering changes telemetry
bytes.

### Hash pinning

Every external file referenced by the scenario can carry an optional
`*_sha256` companion field (`aero.deck_sha256`,
`propulsion.motor.file_sha256`, `sensors.<name>.file_sha256`). When
present, the parser computes the file's SHA-256 digest at load time
and fails closed on mismatch. When absent, the digest is still computed
and surfaced by `openbmp check`; recording those digests in telemetry
headers is runner-side Phase-2.11 work.

`openbmp check` surfaces resolved digests in its output:

```
openbmp check: ok — niskanen-2009-chapter6 (Checked)
  telemetry.output.parquet -> .../out/niskanen-2009-chapter6.parquet
  aero.deck -> .../data/aero/...toml (sha256:cd862c2af98a1f28dc86c6e754d311c7a724081ca91b80704ad89b2ec4cb5c27)
  propulsion.motor.file -> .../data/motors/estes-c6-eng-derived.toml (sha256:da8272d3a7a135046c614e51b279971d37cac376f7aaaffdedc3ccc14d50ad4e)
```

The digest is the SHA-256 of the file bytes encoded as 64 lower-case
hex characters; pin strings are normalised before comparison so
upper-case input still verifies.

## Phase 3 Extensions

Phase 3.2 adds the optional declarative `[mission]` block. When
declared, the runner builds an event-driven mission graph from the
scenario; legacy scenarios without `[mission]` parse and run
identically to pre-3.2.

### Mission block

The `[mission]` block declares phases, events, and transitions. The
canonical example mirrors the
[`niskanen-2009-chapter6-with-mission.toml`](../scenarios/sounding-rocket/niskanen-2009-chapter6-with-mission.toml)
scenario:

```toml
[mission]
initial_phase = "ascent"

[[mission.phases]]
id    = "ascent"
label = "powered + coast ascent"

[[mission.phases]]
id    = "descent"
label = "post-apogee descent"

[[mission.events]]
id      = "at_apogee_marker"
trigger = { kind = "at_apogee" }
action  = { kind = "emit_telemetry_marker", tag = "at_apogee_marker" }
once    = true            # default; may be omitted

[[mission.transitions]]
from  = "ascent"
to    = "descent"
event = "at_apogee_marker"
```

#### Trigger vocabulary

Triggers are crossing detectors: each fires on the step where the
monitored value transitions across the trigger threshold, never
re-firing while the value remains on the same side. All triggers
return `false` on step 0 because no previous-step snapshot exists.

| `trigger.kind` | Required fields | Semantics |
|---|---|---|
| `at_time` | `time_s: f64` | Fires when post-step time crosses `time_s`. |
| `at_altitude_ascending` | `altitude_m: f64` | Fires when altitude crosses up through `altitude_m` (previous below, current at or above). |
| `at_altitude_descending` | `altitude_m: f64` | Fires when altitude crosses down through `altitude_m`. |
| `at_apogee` | — | Fires when vertical velocity flips from `> 0` to `<= 0`. |
| `at_mass_fraction` | `remaining: f64` (in `[0, 1]`) | Fires when mass fraction (current / initial) drops to or below `remaining`. |
| `at_dynamic_pressure` | `pressure_pa: f64`, `falling: bool` | Deferred to Phase 3.5, when atmosphere is wired into event evaluation. Phase 3.4 rejects this trigger at parse time with `UnsupportedTriggerKind`. |

The `kind = "scripted"` trigger is rejected at parse time with a
typed deferral error: scripted triggers are deferred to a later
Phase-3 sub-phase. Phase 3.4 ships `ControlEffector` itself; the
deterministic per-effector `command_schedule` (see
[Control effectors](#control-effectors-phase-34) below) covers the
common scripted-command case without a separate trigger surface.

#### Action vocabulary

| `action.kind` | Required fields | Semantics |
|---|---|---|
| `enter_phase` | `phase: string` (declared phase id) | Sets the active mission phase; visible via runner-side telemetry / diagnostics. |
| `emit_telemetry_marker` | `tag: string` (snake_case) | Allocates a `bool` telemetry channel `mission.marker.<tag>`; runner writes `true` on every step the event fires, `false` on every other step. Channel allocation is alphabetical by tag for declaration-order independence. |
| `stop` | `label: string` | Halts the run with `StopReason::MissionEnded { label }`. Distinct from `EndTime` so determinism telemetry can distinguish CLI-driven stops from scenario-driven mission ends. |
| `effector_override` | `id: string` (declared effector id), `command: f64` (finite) | One-shot command override for the named effector on the next runner step. Resolves the declared id against the runner's effector rack via FNV-1a-64 of `vehicle.assembly.effectors.<id>`. Unknown ids are rejected by `openbmp check`. The kernel records the action; the runner drains it from the per-step fired-event queue and applies it on the next rack tick before the kernel step. Override wins over any declared `command_schedule` for that rack tick only. |
| `engine_command` | `id: string` (declared engine id), `command: { throttle_unit: f64 ∈ [0,1], gimbal_pitch_rad: f64, gimbal_yaw_rad: f64, ignite: bool, shutdown: bool }` | Per-engine command targeting a declared `[[vehicle.assembly.engines]]` by id. Resolves the declared id via FNV-1a-64 of `vehicle.assembly.engines.<id>`. Unknown ids are rejected by `openbmp check`. Kernel records; runner-side `EngineRack` drains and applies on the next rack tick before the kernel step. `ignite=true` is honoured only from `Idle`; `shutdown=true` only from `Igniting` / `Burning`. Throttle / gimbal values are clamped to engine limits at apply time. |

The two remaining reserved actions `separation` / `deploy_recovery`
are still rejected at parse time with typed deferral errors
pointing at Phase 3.6 / 3.9. Phase 3.4 wires `effector_override`,
Phase 3.6 wires `engine_command`; the rest follow when their
assembly children land.

#### `once` semantics

`once = true` (default) makes the binding fire at most once per
simulation run. `once = false` allows the binding to re-fire on
every crossing — useful when a trajectory may legitimately cross
the same threshold more than once (e.g. multi-stage altitude
descent).

#### Determinism

Phase ids and event ids are FNV-1a-64 hashes of the canonical
scenario paths `mission.phases.<id>` and `mission.events.<id>`.
Reordering `[[mission.phases]]`, `[[mission.events]]`, or
`[[mission.transitions]]` blocks in the TOML produces an identical
graph, identical bindings, and identical Parquet bytes. The
phase graph is canonicalised by `(longest-path-depth-from-initial,
PhaseId.value())`; transitions by `(from-depth, from-id, to-depth,
to-id, EventId.value())`. Cycles, unreachable phases, unknown id
references, and duplicate phase ids are rejected at scenario load
time with `ScenarioError::MissionGraph`.

#### Validation invariants

The parser enforces:

- `mission.initial_phase` references a declared phase id.
- All phase ids are unique and non-empty.
- All event ids are unique.
- All `transitions[i].{from, to}` reference declared phases.
- All `transitions[i].event` references a declared event.
- Each `(from, event)` transition pair has at most one target phase.
- Numeric trigger fields are finite; `remaining` is in `[0, 1]`.
- The graph is acyclic and every phase is reachable from
  `initial_phase`.

### Vehicle assembly (Phase 3.3)

When `[vehicle.assembly]` is declared, the scenario describes the
vehicle as a tree of bodies. Phase-3.3 supports single-body and
multi-body assemblies; future phases will add propulsion / effectors
/ tanks / sensors as child blocks of the assembly. The flat
`[vehicle].mass_kg` field stays required and must equal the sum of
declared body dry masses (consistency check: max(1e-12 absolute,
1e-9 relative tolerance)).

The canonical multi-body example mirrors
[`scenarios/multi-body/two-body-fairing.toml`](../scenarios/multi-body/two-body-fairing.toml):

```toml
[vehicle]
kind                     = "point_mass"
mass_kg                  = 0.085   # = sum of body dry masses
initial_position_eci_m   = [0.0, 0.0, 100.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "two-body-fairing"

[[vehicle.assembly.bodies]]
id            = "main"
geometry      = { kind = "cylinder", length_m = 0.56, diameter_m = 0.029 }
dry_mass_kg   = 0.080
dry_cg_body_m = [0.0, 0.0, 0.0]

[[vehicle.assembly.bodies]]
id            = "fairing"
geometry      = { kind = "cone", length_m = 0.05, base_diameter_m = 0.029 }
dry_mass_kg   = 0.005
dry_cg_body_m = [0.0, 0.0, 0.6]
```

#### Geometry vocabulary

`geometry.kind` is one of `cylinder` / `cone` / `reference`:

| `kind` | Required fields | Reference area |
|---|---|---|
| `cylinder` | `length_m`, `diameter_m` | π·(diameter_m / 2)² |
| `cone` | `length_m`, `base_diameter_m` | π·(base_diameter_m / 2)² |
| `reference` | `length_m`, `area_m2` | declared `area_m2` directly |

The `reference` variant is the escape hatch for non-axisymmetric or
pre-computed geometry, mirroring the aero-deck reference.

#### Reserved future-phase children

`[[vehicle.assembly.effectors]]` ships in Phase 3.4 — see
[Control effectors](#control-effectors-phase-34) below.
`[[vehicle.assembly.engines]]` ships in Phase 3.6 — see
[Engine clusters](#engine-clusters-phase-36) below. The remaining
child blocks are reserved and **rejected at parse time** with a
typed `UnsupportedAssemblyChild` error:

- `[[vehicle.assembly.tanks]]` → Phase 3.7 (`Tank` + slosh).

#### Determinism

Body ids are FNV-1a-64 hashes of the canonical scenario body path
(`vehicle.assembly.bodies.<id>`). Reordering `[[vehicle.assembly.bodies]]`
does not shift any body's id. The multi-body summation in
`BasicAssembly::mass_properties` is a left fold in scenario-declared
order with locked operand order; the same assembly built from
different declaration orders produces the same body-id set but may
produce slightly different summed bytes (matches the existing
`[forces].models` declared-order convention).

#### Validation invariants

Enforced at scenario-parse time:

- `bodies` must be non-empty.
- All body ids are unique.
- `[vehicle].mass_kg` equals `sum(bodies[*].dry_mass_kg)` within
  max(1e-12 absolute, 1e-9 relative tolerance).
- Per-body: `dry_mass_kg` finite + positive, `dry_cg_body_m`
  components finite, geometry components finite + positive.
- Inertia tensor (when present): finite, symmetric within 1e-9,
  positive diagonal.
- Rigid-body scenarios: every body declares
  `dry_inertia_body_kg_m2`, and the flat
  `vehicle.inertia_tensor_body_kg_m2` matches the assembled dry
  inertia tensor within the same consistency tolerance.
- The two remaining reserved future-phase child blocks (`engines`,
  `tanks`) must be empty. `effectors` ships in Phase 3.4 and is
  validated by the rules in [Control effectors](#control-effectors-phase-34).

#### Phase-3.3 limitations

- The runner consumes the assembly's dry mass properties for kernel
  mass construction. Force / moment construction still uses the
  existing per-runner paths until the engines / tanks assembly
  children land in later Phase-3 sub-phases.
- Point-mass propagation only uses the assembled dry mass; body CG and
  inertia affect rigid-body mass properties, not point-mass dynamics.

### Control effectors (Phase 3.4)

`[[vehicle.assembly.effectors]]` declares one or more
`ControlEffector` instances mounted on the assembly. Phase 3.4 ships
the effector state machine, a deterministic `command_schedule`, the
runner-side `EffectorRack`, and per-effector deflection telemetry.
The effector deflection is observable in the Parquet via
`effector.<id>.actual` but is **not yet consumed by force / moment
evaluation** — the aero deck stays schema-1 in 3.4. Schema-2 deck
consumption arrives in Phase 3.5.

The canonical example mirrors
[`scenarios/effector-elevon/single-elevon-elevator-step.toml`](../scenarios/effector-elevon/single-elevon-elevator-step.toml):

```toml
[[vehicle.assembly.effectors]]
id               = "delta_e"
kind             = { kind = "linear_actuator", tau_s = 0.05 }
limits           = { min = -0.349, max = 0.349, max_rate_per_s = 5.236, deadband = 0.0, latency_s = 0.020 }
initial_position = 0.0
unit             = "rad"
command_schedule = { kind = "step_at", time_s = 0.5, before = 0.0, after = 0.087 }
```

#### Effector kinds

`kind.kind` is tagged on the inner `kind` field. Phase 3.4 ships
one variant; future phases will add nonlinear / multi-axis / smart
actuators.

| `kind.kind` | Required fields | Semantics |
|---|---|---|
| `linear_actuator` | `tau_s: f64` (optional, default `0.0`) | First-order lag with rate clamp + saturation + deadband + pure-delay buffer. `tau_s = 0` collapses to a rate-clamped tracker (no lag). The latency must be an integer multiple of the scenario's `time.dt_s` (sub-`dt` latency is rejected at construction). |

#### Limits vocabulary

`limits` is a flat table with five required fields:

| Field | Units | Constraint |
|---|---|---|
| `min` | rad (or whatever the actuator drives) | finite, `min < max` |
| `max` | same | finite |
| `max_rate_per_s` | unit / s | finite, strictly positive |
| `deadband` | unit | finite, `>= 0`, `<= (max - min)` |
| `latency_s` | s | finite, `>= 0`, integer multiple of `time.dt_s` |

`initial_position` (optional, default `0.0`) must lie in
`[min, max]`. The rack pre-fills the actuator's pure-delay buffer
with `initial_position` so step 0 reflects the at-rest state.

`unit` is optional telemetry metadata for the scalar effector axis.
When omitted, the runner records unit `"1"`; angular surfaces should
declare `unit = "rad"`.

#### Command schedules

`command_schedule` is optional. When omitted, the effector holds
`initial_position` indefinitely. When present, it carries one of
three deterministic shapes:

| `command_schedule.kind` | Required fields | Semantics |
|---|---|---|
| `constant` | `value: f64` | Commands `value` at every step. |
| `step_at` | `time_s`, `before`, `after` (all `f64`) | Commands `before` while `t < time_s`; `after` thereafter. |
| `linear_ramp` | `start_time_s`, `end_time_s`, `start`, `end` (all `f64`) | Holds `start` while `t <= start_time_s`; ramps linearly to `end` over `[start_time_s, end_time_s]`; holds `end` thereafter. |

A scenario-declared `command_schedule` is the deterministic baseline.
A mission `effector_override` action (see
[Action vocabulary](#action-vocabulary)) wins over the schedule for
the next step only.

#### Faults

`fault` is optional and load-time only in Phase 3.4: a fault
declared in the scenario is injected at construction and persists
for the run. Run-time fault injection (mid-mission failures) is
deferred to a later sub-phase.

| `fault.kind` | Required fields | Semantics |
|---|---|---|
| `jam` | `at: f64` (in `[min, max]`) | Effector freezes at `at`; commands ignored. |
| `runaway` | `rate_per_s: f64` (finite) | Actual position drifts at the constant signed rate, clamped to `[min, max]`. |
| `reduced_rate` | `factor: f64` (in `[0, 1]`) | Effective `max_rate_per_s` is scaled by `factor`. Saturation, deadband, and lag remain in effect. |
| `hardover` | `to: f64` (in `[min, max]`) | Actual position slews at `max_rate_per_s` toward `to` regardless of command. |

#### Telemetry

For each declared effector, the runner allocates one telemetry
channel `effector.<id>.actual` (`f64`, using the effector's declared
`unit`, or `"1"` when omitted). The channels are allocated in scenario-declared order,
**after** the per-model force breakdown and **before** any mission
markers — that ordering is the determinism contract for channel
ids.

#### Determinism

Effector ids are FNV-1a-64 hashes of the canonical scenario path
`vehicle.assembly.effectors.<id>`. Reordering
`[[vehicle.assembly.effectors]]` blocks does not shift any
effector's id. The rack iterates effectors in scenario-declared
order; per-step command resolution is `override > schedule > hold`.
Override events are stored in a `BTreeMap<EffectorId, f64>` so
iteration order is locked across platforms (defeats macOS
`SipHash` randomisation). Empty rack → byte-identical legacy
scenarios: every per-step rack operation is gated on
`!rack.is_empty()`.

#### Validation invariants

Enforced at scenario-parse time:

- `id` non-empty; effector ids unique within the assembly.
- `kind.kind` is a wired variant (`linear_actuator`).
- `limits.{min, max, max_rate_per_s, deadband, latency_s}` finite;
  `min < max`; `max_rate_per_s > 0`; `deadband >= 0` and
  `deadband <= (max - min)`; `latency_s >= 0`.
- `initial_position`, when present, finite and in `[min, max]`.
- `command_schedule` finite numeric fields; `linear_ramp` requires
  `start_time_s < end_time_s`.
- `unit`, when present, non-empty.
- `fault` when present: `jam.at` and `hardover.to` in `[min, max]`;
  `runaway.rate_per_s` finite; `reduced_rate.factor` in `[0, 1]`.
- `linear_actuator.tau_s` (when present) finite and `>= 0`.

`openbmp check` rejects any latency that is not an integer multiple of
the scenario `time.dt_s` (the fixed-step pure-delay buffer cannot
represent sub-`dt` latency).

#### Phase-3.4 limitations (now lifted by Phase 3.5)

- ~~The aero deck stays schema-1; the effector deflection is
  observable in the Parquet but is **not** consumed by force /
  moment evaluation. Schema-2 deck consumption arrives in Phase 3.5.~~
  Phase 3.5 ships [Schema-2 aero decks](#schema-2-aero-decks-phase-35);
  effector `actual` deflections now flow through the kernel into
  the deck lookup.
- Faults are load-time only; run-time fault injection is deferred.
- Only the `linear_actuator` kind ships. Nonlinear / multi-axis /
  smart-actuator variants follow in later sub-phases.

### Schema-2 aero decks (Phase 3.5)

Phase 3.5 extends the aero deck format with optional **control-effector
axes**. A schema-2 deck is identified by `openbmp.aero_deck = 2` in the
deck file and adds an `[axis_order]` block declaring the locked axis
ordering, plus per-effector grid axes (e.g. `delta_e_deg = [-20, 0,
20]`). At runtime, the runner-side `EffectorRack` snapshot flows
through the kernel's `EffectorActualsView` into the deck's multilinear
lookup, so the rate-limited / saturated `EffectorState.actual` from
Phase 3.4 actually modulates `(CN, CD, CM)`.

Schema-1 decks continue to load and produce bit-identical lookups; the
schema discriminator is purely additive.

#### Wire format

```toml
openbmp.aero_deck = 2

reference.area_m2  = 1.0
reference.length_m = 1.0
provenance         = "..."
validation         = "experimental"

[grid]
mach        = [0.0, 0.5, 1.0]
alpha_deg   = [-5.0, 0.0, 5.0]
beta_deg    = [0.0]
delta_e_deg = [-20.0, 0.0, 20.0]   # effector axis (Phase 3.5)

[axis_order]
order = ["mach", "alpha", "beta", "delta_e_deg"]

[coefficients.cn]
data = [/* row-major over the 4-axis cartesian product */]

[coefficients.cd]
data = [/* ... */]

[coefficients.cm]
data = [/* ... */]

[interpolation]
method        = "multilinear"   # required; only method wired in 3.5
extrapolation = "error"         # required; control surfaces saturate
                                # via the ControlEffector layer
```

#### Validation rules

- `axis_order.order` must start with `["mach", "alpha", "beta"]` (the
  three base axes are always present and always first in the locked
  reduction order).
- Effector axes (`axis_order.order[3..]`) must each end with `_deg` or
  `_rad`. The runner uses this suffix to verify the matching scenario
  effector's `unit` field at runner build time (see Unit-matching
  contract below).
- Effector axis names must be unique after stripping `_deg` / `_rad`;
  a deck cannot declare both `delta_e_deg` and `delta_e_rad`.
- Up to 3 effector axes (6 axes total) are supported in Phase 3.5.
  Larger decks are deferred to a later sub-phase.
- The `[grid]` table must declare exactly the axes in `axis_order` —
  extra keys are rejected, missing keys are rejected. The three base
  axes use the existing schema-1 grid keys (`mach`, `alpha_deg`,
  `beta_deg`); effector axes use their `axis_order` name verbatim.
- `interpolation.method` must be `"multilinear"`. Other methods are
  reserved for hypersonic Phase-6 work.
- `interpolation.extrapolation` must be `"error"` (fail-closed).
  Schema-2 decks do not expose `clamp` — control surfaces saturate
  via the `ControlEffector` rate-limit / position-limit layer, not
  via the deck's extrapolation policy.
- Coefficient table lengths equal the cartesian product of axis
  lengths.

#### Unit-matching contract

When the runner loads a scenario with a schema-2 deck, it pairs each
declared effector axis with a scenario effector by name and asserts
the unit suffix matches:

| Deck axis name | Required scenario `unit` |
|---|---|
| `delta_e_deg` (or `delta_a_deg`, `delta_r_deg`, …) | `"deg"` |
| `delta_e_rad` | `"rad"` |
| any other custom axis name with `_<unit>` suffix | the suffix string |

The matcher strips the unit suffix from the deck axis name to find the
scenario effector by `id`; therefore deck axis order is independent of
the scenario effector declaration order. Mismatches (duplicate
stripped deck axis, no matching effector, mismatched unit) fail closed
at runner build time with `CliError::AeroEffectorMismatch`, before any
kernel step is taken.
Schema-1 decks and decks without effector axes skip this check
entirely.

#### Interpolation

Multilinear over N axes (3 ≤ N ≤ 6). The reduction collapses the
**innermost axis first** (the last axis in `axis_order`) and walks
outward; this preserves bit-identical f64 outputs at N = 3 against the
Phase-2.5 trilinear path. The locked operand order is part of the
byte-stability contract and is asserted by a 1024-case property test
in `crates/openbmp-aero/src/deck.rs`.

#### Determinism

Schema-1 fixtures are bit-identical under the schema-2 loader. Every
existing scenario (D12, Niskanen point-mass, Niskanen rigid, Phase-1
analytic-toy, Phase-3.4 effector e2e) continues to produce
byte-identical Parquet under Phase 3.5. The kernel-owned
`effector_actuals: BTreeMap<String, f64>` snapshot is empty by
default; the runner only populates it when at least one schema-2 deck
axis matches a scenario effector. Empty snapshot → empty
`EffectorActualsView` → schema-1 lookups ignore the view → byte-stable
legacy path.

The canonical schema-2 example ships at
[`scenarios/effector-elevon-aero/single-elevon-aero-deflected.toml`](../scenarios/effector-elevon-aero/single-elevon-aero-deflected.toml)
and the deck at
[`data/aero/synthetic-elevon-1d.toml`](../data/aero/synthetic-elevon-1d.toml).

### Engine clusters (Phase 3.6)

`[[vehicle.assembly.engines]]` declares one or more `EngineModel`
instances mounted on the assembly. Phase 3.6 ships the
`LiquidEngine` reference impl plus the runner-side `EngineRack`
and the kernel-side cluster adapters (force, mass). Engines
respond to per-engine `engine_command` mission events; per-engine
`command_schedule` (effector-style scripted commands) is **not**
in scope for 3.6 — scripted command sequences flow through the
`mission.events[*]` timeline.

A scenario uses **either** the legacy `[propulsion.motor]` block
(single solid motor) **or** `[[vehicle.assembly.engines]]`
(multi-engine liquid cluster) — never both. Co-declaration is
rejected at parse time with `ScenarioError::AmbiguousPropulsion`.

The canonical example mirrors
[`scenarios/multi-engine-octaweb/four-engine-shutdown.toml`](../scenarios/multi-engine-octaweb/four-engine-shutdown.toml):

```toml
[vehicle.assembly]
id             = "octaweb-four-engine"
cluster_layout = "octaweb"

[[vehicle.assembly.engines]]
id                  = "engine_a"
kind                = { kind = "liquid_engine" }
mount_point_body_m  = [0.0, 0.0, 0.0]
limits              = { max_thrust_n = 5000.0, isp_s = 250.0,
                        ignition_transient_s = 0.1,
                        shutdown_transient_s = 0.1,
                        max_gimbal_rad = 0.087 }
```

#### Engine kinds

`kind.kind` is tagged on the inner `kind` field. Phase 3.6 ships
one variant; later phases add hybrid / cold-gas / chamber-pressure
models.

| `kind.kind` | Required fields | Semantics |
|---|---|---|
| `liquid_engine` | (none) | Linear ignition transient (0 → commanded thrust over `ignition_transient_s`), constant-throttle burn, linear shutdown transient (current → 0 over `shutdown_transient_s`). Mass flow `mdot = thrust / (g0 · isp_s)` with `g0 = 9.80665`. Gimbal applied as locked-order pitch-around-body-y then yaw-around-body-x rotation of nominal body-`+z` thrust. |

#### Limits vocabulary

`limits` is a flat table with five required fields:

| Field | Units | Constraint |
|---|---|---|
| `max_thrust_n` | N | finite, strictly positive |
| `isp_s` | s | finite, strictly positive |
| `ignition_transient_s` | s | finite, `>= 0` (`0.0` → instantaneous ignition) |
| `shutdown_transient_s` | s | finite, `>= 0` (`0.0` → instantaneous shutdown) |
| `max_gimbal_rad` | rad | finite, `>= 0` (`0.0` → fixed-axis engine) |

#### Mount geometry

`mount_point_body_m: [x, y, z]` — body-frame mount position in
metres. Used by the rigid-body kernel's cluster moment adapter
(`mount × thrust_body`, summed in scenario-declared order).
Point-mass scenarios store the mount points but don't use them.

#### Cluster layout

Optional `cluster_layout` on `[vehicle.assembly]`: one of
`axial | ring | octaweb | custom` (default: `custom`). The tag
informs telemetry and docs; Phase 3.6 has no behavioural use for
it. Future sub-phases may use the layout to drive symmetry-aware
fault scenarios or controller-side allocation tables.

#### Faults

`fault` is optional and load-time only in Phase 3.6: a fault
declared in the scenario is injected at construction and persists
for the run. Run-time fault injection is deferred to a later
sub-phase (mirrors Phase-3.4 effector faults).

| `fault.kind` | Required fields | Semantics |
|---|---|---|
| `stuck` | `at_throttle: f64` (in `[0, 1]`) | Throttle stuck at `at_throttle`; engine ignores command throttle but still honours ignite / shutdown lifecycle. |
| `hard_off` | — | Engine commanded off and never restarts. Sets state to `Failed` on first step. |
| `over_thrust` | `factor: f64` (finite, `>= 0`) | Thrust scaled by `factor`. `>= 1` → over-thrust; `< 1` → under-thrust. |
| `gimbal_locked` | `pitch_rad: f64`, `yaw_rad: f64` (each in `±max_gimbal_rad`) | Gimbal frozen at the given angles regardless of command. |

#### Lifecycle and command resolution

Engine state machine: `Idle → Igniting → Burning → Shutdown`.
`Shutdown` and `Failed` are terminal — engines do not re-ignite.

Per-step command resolution: `engine_command` event firing →
runner drains and applies → engine latches `(throttle, gimbal)`
and processes `(ignite, shutdown)` lifecycle flags. Multiple
events targeting the same engine in one step are rejected by the
runner; declare a single `engine_command` per engine per step.

#### Determinism

Engine ids are FNV-1a-64 hashes of the canonical scenario path
`vehicle.assembly.engines.<id>`. Reordering
`[[vehicle.assembly.engines]]` blocks does not shift any engine's
id. The runner-side `EngineRack` iterates engines in
scenario-declared order; the kernel-side
`EngineClusterForceAdapter` sums per-engine thrust contributions
in the same order with locked left-fold operand order. The
kernel-pushed snapshot map is `BTreeMap<EngineId, EngineSnapshot>`
(not `HashMap`) — deterministic iteration on macOS `SipHash`
builds.

Single-motor and no-propulsion scenarios short-circuit every
rack-related operation on `engine_rack.is_empty()`; the kernel's
`engine_snapshot` field stays at the empty `BTreeMap` set in
`new()`, the cluster adapters are never instantiated, and legacy
scenarios produce byte-identical Parquet to pre-3.6.

#### Validation invariants

Enforced at scenario-parse time:

- `engines` non-empty when declared (zero-engine cluster is
  rejected).
- All engine ids unique within the assembly.
- `limits.{max_thrust_n, isp_s}` finite + strictly positive.
- `limits.{ignition_transient_s, shutdown_transient_s, max_gimbal_rad}`
  finite + non-negative.
- `mount_point_body_m` finite components.
- `kind.kind` is a wired variant (`liquid_engine`).
- `fault` when present: `stuck.at_throttle` in `[0, 1]`;
  `over_thrust.factor` non-negative; `gimbal_locked.{pitch,yaw}_rad`
  in `±max_gimbal_rad`.
- Cross-validate `mission.events[*].action.id` (when action kind
  is `engine_command`) against declared engine ids.
- Reject engine clusters that omit `thrust` from `forces.models`;
  declared engines must be dynamically active, not mass-only.
- Reject scenarios that declare both `[propulsion.motor]` and
  `[[vehicle.assembly.engines]]` (`AmbiguousPropulsion`).
- `cluster_layout`, when present, is one of
  `axial | ring | octaweb | custom`.
- `engine_command.command.throttle_unit` finite and in `[0, 1]`;
  gimbal angles finite (runtime engine clamps to its
  `max_gimbal_rad`).

#### Phase-3.6 limitations

- Rigid-body cluster mass-properties (with inertia tensor
  evolution as propellant is consumed) are deferred to Phase 3.7
  alongside tank-driven dynamics. Rigid scenarios with engine
  clusters use `ConstantMassRigid` for kernel mass-properties;
  the cluster's force and moment adapters still consume the
  per-engine snapshot normally.
- Faults are load-time only.
- Only the `liquid_engine` kind ships.
- Per-engine `command_schedule` (effector-style declarative
  scripts) is out of scope for 3.6; engines drive only via
  `mission.events[*].action.engine_command`.

The canonical Phase-3.6 example ships at
[`scenarios/multi-engine-octaweb/four-engine-shutdown.toml`](../scenarios/multi-engine-octaweb/four-engine-shutdown.toml).
