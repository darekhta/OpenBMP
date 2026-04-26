# OpenBMP Scenario Format

OpenBMP scenarios are TOML-shaped, versioned configuration files. They define
models, initial state, deterministic schedule, telemetry outputs, and
validation rules for one simulation run.

This document is the Phase-1 format contract. It intentionally favors strict,
verbose fields over compact syntax.

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
| `seed` | integer | yes | unsigned 64-bit |

`dt_s` is the base kernel step. Multi-rate schedules are integer divisors of
the base step and must be declared under subsystem-specific `rate_hz` fields.

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
