# OpenBMP Scenario Format

OpenBMP scenarios are TOML-shaped, versioned configuration files. They define
models, initial state, deterministic schedule, telemetry outputs, and
validation rules for one simulation run.

The schema header is `openbmp.scenario = 2`. Phase-3.13 retired the v1
flat-vehicle shape; every scenario now carries a mandatory
`[vehicle.assembly]` block and per-body dry mass / inertia live on
`[[vehicle.assembly.bodies]]`. v1 scenarios fail closed at the header
check. See [Migrating v1 scenarios to v2](#migrating-v1-scenarios-to-v2)
for the mechanical rewrite. Phase-2 specifics live in the
[Phase-2 Extensions](#phase-2-extensions) section at the bottom.

This document is the format contract. It intentionally favors strict, verbose
fields over compact syntax.

## Format Rules

- The root key `openbmp.scenario = 2` is mandatory.
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
| `[forces]` | no | Optional force and moment model ordering override; omitted scenarios derive `["gravity", "thrust"?, "aero"?]` from the assembly and model blocks |
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
openbmp.scenario = 2

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
initial_position_eci_m = [0.0, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "constant-acceleration-drop"

[[vehicle.assembly.bodies]]
id = "main"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0

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

In schema version 2, the assembly is the single dry-mass source of truth.
Point-mass scenarios sum `[[vehicle.assembly.bodies]].dry_mass_kg` for the
initial dry vehicle mass. With `[propulsion.motor]`, the runner adds the
motor's time-varying mass from the pinned motor file; with
`[[vehicle.assembly.engines]]`, the engine-cluster rack supplies the thrust and
mass-flow path.

## Migrating v1 Scenarios To v2

The v1 to v2 rewrite is mechanical:

- Change the header to `openbmp.scenario = 2`.
- Remove top-level vehicle dry-mass and inertia fields from `[vehicle]`.
- Add `[vehicle.assembly]` and at least one `[[vehicle.assembly.bodies]]`
  entry.
- Move the old dry mass into `dry_mass_kg` on the body. For rigid-body
  scenarios, move the old inertia matrix into
  `dry_inertia_body_kg_m2` on each body that contributes inertia.
- Keep `[forces]` only when the scenario needs a non-default order or wants to
  disable a derived force. When omitted, the loader derives `gravity`, then
  `thrust` if propulsion or assembly engines are declared, then `aero` if an
  aero deck is declared.

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
Those blocks remain optional in schema v2 unless selected models require
their matching structured table.
The canonical worked example is
[`scenarios/sounding-rocket/niskanen-2009-chapter6.toml`](../scenarios/sounding-rocket/niskanen-2009-chapter6.toml).

### Rigid-body initial state

When `[vehicle].kind = "rigid_body"`, three additional fields are required:

```toml
[vehicle]
kind                                 = "rigid_body"
initial_position_eci_m               = [0.0, 0.0, 0.0]
initial_velocity_eci_m_s             = [0.0, 0.0, 0.0]
initial_quaternion_body_to_eci_xyzw  = [0.0, 0.0, 0.0, 1.0]
initial_angular_velocity_body_rad_s  = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "rigid-body-example"

[[vehicle.assembly.bodies]]
id = "main"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.5
dry_inertia_body_kg_m2 = [
  [0.10, 0.0,  0.0],
  [0.0,  0.10, 0.0],
  [0.0,  0.0,  0.01],
]
```

The quaternion is the body-to-ECI rotation in `[x, y, z, w]` order;
the parser checks unit-norm to 1e-9. Per-body inertia tensors are
declared in body axes as 3x3 kg m^2 matrices; the parser requires
finite symmetric entries with strictly positive diagonal moments, and
rigid-kernel construction applies the full physical validity checks.
The rigid-body-only initial-state fields are rejected when
`kind = "point_mass"`.

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
otherwise. Phase 3.8 also accepts `kind = "layered"` with a `layers`
table and `kind = "gust"` with Dryden `intensity_m_s`,
`length_scale_m`, and `airspeed_m_s` parameters. Selecting any
non-`none` `environment.wind` therefore requires the structured
`[wind]` block; leaving `environment.wind = "none"` lets a structured
`[wind]` block opt into the active model.

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

[sensors.gnss]
kind = "gnss"
file = "../../data/sensors/gnss-textbook.toml"

[sensors.mag]
kind = "magnetometer"
file = "../../data/sensors/magnetometer-textbook.toml"

[sensors.star]
kind = "star_tracker"
file = "../../data/sensors/star-tracker-textbook.toml"
```

`kind = "ideal_state"` carries no noise budget and rejects `file`;
all other kinds require `file`. The Phase-2.7 set is `imu` /
`barometer`; the Phase-3.10 additions are `gnss` (IS-GPS-200
receiver-output noise: per-axis Gaussian on position + velocity
plus an OU position-bias drift), `magnetometer` (per-axis
Gaussian on the body-frame WMM 2025 truth field plus constant
3×3 soft-iron and 3-vector hard-iron biases), and `star_tracker`
(small-angle Gaussian rotation-vector perturbation per axis).

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
| `deploy_recovery` | `id: string` (declared recovery id), `command: "deploy" \| "deploy_drogue" \| "deploy_main" \| "stow"` | Recovery-device command targeting a declared `[[vehicle.assembly.recovery]]` by id. Resolves via FNV-1a-64 of `vehicle.assembly.recovery.<id>`. Unknown ids and kind-incompatible commands are rejected by `openbmp check`; runner-side `RecoveryRack` drains accepted firings on the next rack tick before the kernel step. |

The remaining reserved action `separation` is still rejected at parse
time with a typed deferral error pointing at Phase 3.6 / 3.7.
Phase 3.4 wires `effector_override`, Phase 3.6 wires
`engine_command`, and Phase 3.9 wires `deploy_recovery`.

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
vehicle as a tree of bodies. Schema v2 requires this block for every
scenario. Phase-3.3 introduced single-body and multi-body assemblies;
later Phase-3 sub-phases added effectors, engine clusters, tanks, and
recovery devices as child blocks of the assembly. Sensors remain
outside the assembly for now. The assembly bodies are the dry mass and
dry inertia source of truth.

The canonical multi-body example mirrors
[`scenarios/multi-body/two-body-fairing.toml`](../scenarios/multi-body/two-body-fairing.toml):

```toml
[vehicle]
kind                     = "point_mass"
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

#### Assembly children

`[[vehicle.assembly.effectors]]` shipped in Phase 3.4 — see
[Control effectors](#control-effectors-phase-34) below.
`[[vehicle.assembly.engines]]` shipped in Phase 3.6 — see
[Engine clusters](#engine-clusters-phase-36) below.
`[[vehicle.assembly.tanks]]` shipped in Phase 3.7 — see
[Tanks and slosh](#tanks-and-slosh-phase-37) below.
`[[vehicle.assembly.recovery]]` shipped in Phase 3.9 — see
[Recovery and descent](#recovery-and-descent-phase-39) below.

#### Determinism

Body ids are FNV-1a-64 hashes of the canonical scenario body path
(`vehicle.assembly.bodies.<id>`). Reordering `[[vehicle.assembly.bodies]]`
does not shift any body's id. The multi-body summation in
`Assembly::mass_properties` is a left fold in scenario-declared
order with locked operand order; the same assembly built from
different declaration orders produces the same body-id set but may
produce slightly different summed bytes (matches the existing
`[forces].models` declared-order convention).

#### Validation invariants

Enforced at scenario-parse time:

- `bodies` must be non-empty.
- All body ids are unique.
- Per-body: `dry_mass_kg` finite + positive, `dry_cg_body_m`
  components finite, geometry components finite + positive.
- Inertia tensor (when present): finite, symmetric within 1e-9,
  positive diagonal.
- Rigid-body scenarios: every body declares
  `dry_inertia_body_kg_m2`.
- Assembly child blocks are validated by their phase sections:
  `effectors` (Phase 3.4), `engines` (Phase 3.6), `tanks`
  (Phase 3.7), and `recovery` (Phase 3.9). Unknown child blocks
  remain parse errors.

#### Phase-3.3 limitations

- The runner consumes the assembly's dry mass properties for kernel
  mass construction. Force / moment construction still uses
  runner-side adapters until the remaining single-motor adapter path
  is retired behind the engine-cluster infrastructure.
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

Schema-1 aero-deck fixtures are bit-identical under the schema-2 deck
loader. The migrated schema-v2 scenarios continue to produce
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
single-motor scenarios preserve the same hot path.

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

### Tanks and slosh (Phase 3.7)

Phase 3.7 lands the `[[vehicle.assembly.tanks]]` block and the
runner-side tank rack that drives moving-mass dynamics through the
kernel snapshot path (mirroring the Phase-3.6 engine-cluster
pattern). Each declared tank produces a body-frame reaction force +
moment on the parent body, contributes its `mass_kg` to the vehicle
total mass, and tracks fluid-mass evolution under a scenario-
declared drain rate.

```toml
[[vehicle.assembly.tanks]]
id                       = "fuel_tank"
mounted_to               = "main"
mount_point_body_m       = [0.0, 0.0, 1.5]
geometry                 = { kind = "cylinder", radius_m = 0.5, height_m = 1.5 }
propellant               = { density_kg_m3 = 1000.0, label = "water_textbook" }
initial_fill_fraction    = 0.7
moving_mass              = { kind = "equivalent_pendulum", damping_ratio_zeta = 0.005 }
drain_rate_kg_per_s      = 0.0
initial_slosh            = { angles_rad = [0.05, 0.0], rates_rad_s = [0.0, 0.0] }
```

#### Required fields

- `id` — `snake_case` scenario-text identifier; must be unique
  across `[[vehicle.assembly.tanks]]`.
- `mounted_to` — id of the parent body in
  `[[vehicle.assembly.bodies]]`. References to undeclared bodies
  fail at parse with `UnknownBodyReference`.
- `mount_point_body_m` — `[x, y, z]` body-frame mount point, m.
- `geometry` — `kind`-tagged enum: `cylinder` (`radius_m`,
  `height_m`), `sphere` (`radius_m`), or `ellipsoid_textbook`
  (`a_m`, `b_m`, `c_m`).
- `propellant` — `{ density_kg_m3, label }`. Phase-3 ships textbook
  density only; fielded propellant data is rejected per
  `safety-boundaries.md`.
- `initial_fill_fraction` — `[0, 1]`. Initial fluid mass is
  `geometry.volume × density × fill`.
- `moving_mass` — `kind`-tagged enum:
  - `rigid_liquid` — no slosh (Phase 3.7.A toy);
  - `equivalent_pendulum { damping_ratio_zeta }` — Abramson
    cylindrical-tank antisymmetric fundamental mode (Phase 3.7.B);
  - `equivalent_spring_mass { damping_ratio_zeta }` — translational
    alternative;
  - `baffled_pendulum { base_damping_ratio_zeta }` — pendulum +
    `baffle_model.damping_increment_zeta`.

#### Optional fields

- `baffle_model = { damping_increment_zeta }` — additive damping
  increment consumed only by `baffled_pendulum`. Phase-3.7
  minimum: a scalar increment per Abramson Eq 7-46 simplified.
- `drain_rate_kg_per_s` — Phase-3.7 ships drain decoupled from
  engine clusters. Defaults to `0.0` (no drain). Future phase ties
  this to the engine-cluster total mdot.
- `initial_slosh = { angles_rad: [θ_x, θ_y], rates_rad_s: [θ̇_x,
  θ̇_y] }` — initial slosh perturbation for `equivalent_pendulum`
  and `baffled_pendulum`.
- `initial_slosh = { displacement_body_m: [x, y], velocity_body_m_s:
  [ẋ, ẏ] }` — initial slosh perturbation for
  `equivalent_spring_mass`.

#### Cross-block validation

- Non-`rigid_liquid` `moving_mass` on a non-cylindrical geometry is
  rejected (Abramson cylindrical-tank closed forms apply only to
  cylinders).
- Non-`rigid_liquid` `moving_mass` in a `vehicle.kind = "point_mass"`
  scenario is rejected — slosh dynamics in a point-mass kernel are
  degenerate (no body-frame orientation, lateral reaction force has
  no rotational coupling). The validator fails loud rather than
  silently producing a no-op tank.
- `initial_slosh` is rejected for `rigid_liquid`; `baffle_model` is
  rejected unless `moving_mass.kind = "baffled_pendulum"`.
- `drain_rate_kg_per_s × time.dt_s` must not exceed the initial tank
  fluid mass; Phase 3.7 rejects single-step emptying rather than
  silently clamping away the moving-mass dynamics.

#### Determinism

- Tank ids are FNV-1a-64 of `vehicle.assembly.tanks.<id>`; the
  runner-side rack stores them in a `BTreeMap<TankId, Tank>` for
  deterministic iteration on macOS `SipHash` builds.
- Slosh integration is semi-implicit (symplectic) Euler with
  locked operand order — single sub-step per kernel base tick.
  Phase 3.7 deviated from the original "forward Euler" wording:
  pure explicit Euler is unstable for an undamped harmonic
  oscillator and cannot meet the 1 % energy-conservation gate
  over 100 oscillations at any practical `dt`. The semi-implicit
  variant has the same operation count, the same locked operand
  order, and the same bit-stable replay properties; the per-sub-
  phase commit history records the rationale.
- The slosh state advances using **prior step's** `(accel_body,
  omega_body)` — the documented one-step lag that breaks the
  circular dependency between the tank's reaction force and the
  kernel's per-step force evaluation. The first step uses zeros.

#### Phase-3.7 limitations

- Engine-cluster ↔ tank drain coupling is **not** wired. Tanks
  declare a constant `drain_rate_kg_per_s` (default 0); engines
  continue to track their own propellant accounting from Phase 3.6.
  Future phase unifies the two.
- The `TankRackMassAdapter` adds each tank's `mass_kg` to the
  vehicle total. For a tank intended as the cluster's propellant
  store this overcounts the propellant (the cluster's
  `EngineClusterMassAdapter` already debits consumed propellant
  from the dry mass). The Phase-3.7.E exit-criterion scenario
  sizes the tank at about 5 % of vehicle dry mass to keep the
  overcount small; future phase unifies the accounting.
- Rigid-body cluster mass-properties (with inertia tensor
  evolution from per-engine `consumed_kg`) are still deferred —
  rigid scenarios with engine clusters use `ConstantMassRigid` for
  kernel mass-properties (Phase-3.6 deferral). The
  `TankSnapshot.inertia_delta_body_kg_m2` is published in the
  snapshot but is not consumed by the rigid mass-properties model in
  Phase 3.7.
- Slosh telemetry channels (`tank.<id>.slosh_angle_rad`,
  `tank.<id>.fluid_kg`, etc.) are deferred to a Phase-3.X
  follow-on; the Phase-3.7.E e2e test verifies determinism via
  full-Parquet byte equality rather than per-tank channels.

The canonical Phase-3.7 example ships at
[`scenarios/sloshing-tank/sloshing-tank.toml`](../scenarios/sloshing-tank/sloshing-tank.toml).

### Recovery and descent (Phase 3.9)

Phase 3.9 lands `[[vehicle.assembly.recovery]]` devices and the
runner-side recovery rack. Recovery is force-only in this phase: the
kernel applies drag at the body center of gravity, with no recovery
mass contribution and no recovery moment.

```toml
[[vehicle.assembly.recovery]]
id   = "dual_chute"
kind = { kind = "drogue_main", drogue_c_d = 1.0, drogue_area_m2 = 0.5, main_c_d = 1.5, main_area_m2 = 4.0 }

[[mission.events]]
id      = "evt_apogee"
trigger = { kind = "at_apogee" }
action  = { kind = "deploy_recovery", id = "dual_chute", command = "deploy_drogue" }
```

#### Recovery kinds

`kind` is a tagged enum:

| `kind.kind` | Required fields | Commands |
|---|---|---|
| `parachute_drag` | `c_d`, `area_inflated_m2` | `deploy` |
| `drogue_main` | `drogue_c_d`, `drogue_area_m2`, `main_c_d`, `main_area_m2` | `deploy_drogue`, `deploy_main` |
| `drag_device` | `c_d`, `area_deployed_m2` | `deploy`, `stow` |

All drag coefficients and areas must be finite and strictly positive.
The runner publishes zero drag while a device is stowed.

#### Event compatibility

`deploy_recovery` actions target one declared recovery id. Scenario
validation rejects unknown ids and rejects commands incompatible with
the target kind. The accepted matrix is:

| Recovery kind | `deploy` | `deploy_drogue` | `deploy_main` | `stow` |
|---|---|---|---|---|
| `parachute_drag` | accepted | rejected | rejected | rejected |
| `drogue_main` | rejected | accepted | accepted | rejected |
| `drag_device` | accepted | rejected | rejected | accepted |

Multiple recovery commands for the same device in one kernel step are
rejected by the runner; use separate event firings for ordered
transitions such as deploy then stow.

#### Telemetry

Each declared recovery device allocates three channels in
scenario-declared order, after effector channels and before mission
markers:

| Channel | Type | Units | Notes |
|---|---|---|---|
| `recovery.<id>.deployed` | bool | bool | `true` for any non-stowed phase |
| `recovery.<id>.phase_index` | int64 | 1 | `0 = Stowed`, `1 = Drogue`, `2 = Main` |
| `recovery.<id>.drag_area_m2` | float64 | m^2 | Current effective drag area |

The recovery force adapter sums deployed `C_D A` terms in
scenario-declared recovery order and short-circuits before atmosphere
sampling when the sum is zero, preserving legacy byte-stability for
scenarios with no recovery devices.

#### Phase-3.9 limitations

- Drag opposes ECI velocity, matching the existing axial-drag adapter.
  Wind-relative parachute drag is deferred.
- Canopy inflation transients are deferred; all Phase-3.9 recovery
  state changes are instantaneous at the rack tick.
- Recovery devices do not contribute mass or moments.

The canonical Phase-3.9 example ships at
[`scenarios/parachute-recovery/parachute-descent.toml`](../scenarios/parachute-recovery/parachute-descent.toml).
