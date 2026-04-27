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
closed. The deck format itself is the Phase-2.5 schema-1 deck
documented in [`software-architecture.md § Deck Format`](software-architecture.md#deck-format-in-house-toml).

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
| `at_dynamic_pressure` | `pressure_pa: f64`, `falling: bool` | Deferred to Phase 3.4, when atmosphere is wired into event evaluation. Phase 3.2 rejects this trigger at parse time with `UnsupportedTriggerKind`. |

The `kind = "scripted"` trigger is rejected at parse time with a
typed deferral error: scripted triggers ship in Phase 3.4 alongside
`ControlEffector`.

#### Action vocabulary

| `action.kind` | Required fields | Semantics |
|---|---|---|
| `enter_phase` | `phase: string` (declared phase id) | Sets the active mission phase; visible via runner-side telemetry / diagnostics. |
| `emit_telemetry_marker` | `tag: string` (snake_case) | Allocates a `bool` telemetry channel `mission.marker.<tag>`; runner writes `true` on every step the event fires, `false` on every other step. Channel allocation is alphabetical by tag for declaration-order independence. |
| `stop` | `label: string` | Halts the run with `StopReason::MissionEnded { label }`. Distinct from `EndTime` so determinism telemetry can distinguish CLI-driven stops from scenario-driven mission ends. |

The four reserved actions `engine_command` / `effector_override` /
`separation` / `deploy_recovery` are rejected at parse time with
typed deferral errors pointing at Phase 3.6 / 3.4 / 3.6 / 3.9.

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
declared body dry masses (consistency check, 1e-9 tolerance).

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

The following child blocks are reserved and **rejected at parse time
in Phase 3.3** with a typed `UnsupportedAssemblyChild` error:

- `[[vehicle.assembly.effectors]]` → Phase 3.4 (`ControlEffector`).
- `[[vehicle.assembly.engines]]` → Phase 3.6 (`EngineCluster`).
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
  1e-9.
- Per-body: `dry_mass_kg` finite + positive, `dry_cg_body_m`
  components finite, geometry components finite + positive.
- Inertia tensor (when present): finite, symmetric within 1e-9,
  positive diagonal.
- The four reserved future-phase child blocks (`effectors`,
  `engines`, `tanks`) must be empty.

#### Phase-3.3 limitations

- The kernel still drives off the flat `[vehicle].mass_kg` field.
  The assembly tree is *advisory* in 3.3 — the resolver builds and
  validates a `BasicAssembly` from the scenario, but kernel
  construction continues through the existing per-runner
  `build_vehicle` / `build_mass_model` paths. Phase-3.4+ will land
  the kernel-side mass-model resolver as the assembly tree gains
  real propulsion / effector / tank content.
- Rigid-body multi-body scenarios with non-trivial body inertia
  tensors are deferred to Phase-3.4+ alongside the
  ControlEffector / engine / tank tree extensions.
