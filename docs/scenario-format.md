# OpenBMP Scenario Format

OpenBMP scenarios are TOML-shaped, versioned configuration files. They define
models, initial state, deterministic schedule, telemetry outputs, and
validation rules for one simulation run.

The schema header is `openbmp.scenario = 2` or `openbmp.scenario = 3`.
The v1 flat-vehicle shape is retired; every supported scenario
now carries a mandatory `[vehicle.assembly]` block and per-body dry
mass / inertia live on `[[vehicle.assembly.bodies]]`. v1 scenarios fail
closed at the header check. See
[Migrating v1 scenarios to v2](#migrating-v1-scenarios-to-v2) for the
mechanical rewrite. Sounding-rocket specifics live in the
[Sounding-Rocket Extensions](#sounding-rocket-extensions) section at the bottom.

This document is the format contract. It intentionally favors strict, verbose
fields over compact syntax.

## Format Rules

- The root key `openbmp.scenario` is mandatory and must be `2` or `3`.
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
| `[aero]` | no | Aerodynamic coefficient source: external deck, inline buildup, or live method |
| `[aerothermal]` | no | Live entry heating / thermal-toy / ablation diagnostics and optional rigid-body mass feedback |
| `[propulsion]` | no | Motor reference; ignition time |
| `[wind]` | no | Structured wind block; overrides `environment.wind` |
| `[atmosphere]` | no | Structured atmosphere block; overrides `environment.atmosphere` |
| `[solver]` | no | Integrator / solver profile; required for hypersonic scenarios |
| `[data_packages]` | no | External real-data or high-fidelity reference package sidecars |
| `[sensors]` | no | Synthetic sensor models |
| `[fc]` | no | Simulator-local flight controller |
| `[faults]` | no | Scenario-injected fault models |
| `[batch]` | no | Batch or Monte Carlo sweep metadata |
| `[landing_footprint]` | no | Schema-v3 offline range-safety footprint post-processing |
| `[staging_analysis]` | no | Schema-v3 offline ideal ΔV budget / mass-optimal staging analysis |
| `[entry_profile]` | no | Schema-v3 descent / entry handoff and entry diagnostics |

## Flight Controller Block

`[fc]` is simulator-local. It configures a controller that is
safe-boundary constrained and does not imply hardware support.

```toml
[fc]
estimator        = "ekf"          # "ekf" | "mekf"
autopilot        = "three_loop"
guidance         = "attitude_hold" # "attitude_hold" | "waypoint"
reference_q_xyzw = [0.0, 0.0, 0.0, 1.0]
base_rate_hz     = 1000
frame_budget_us  = 2000

[fc.ekf]
mag_field                  = "wmm_2025" # "earth_dipole" | "wmm_2025"
mag_epoch_decimal_year     = 2025.0
sigma_w_gyro               = 0.01
sigma_w_accel_bias         = 1.0e-4
sigma_w_gyro_bias          = 1.0e-5
tau_gyro_bias_s            = 100.0
tau_accel_bias_s           = 100.0
sigma_gnss_pos_m           = 5.0
sigma_gnss_vel_m_s         = 0.5
sigma_baro_alt_m           = 2.0
sigma_mag_nt               = 100.0
innovation_false_alarm_rate = 0.01
dead_reckon_timeout_s      = 1.5

[fc.autopilot_params]
anti_windup             = { kind = "back_calculation", gain = 1.0 }
rate_deadband_rad_s     = 0.001
trajectory_loop_enabled = false
trajectory_kind         = "pid" # "pid" | "minimum_snap" (Mellinger-Kumar 2011 differential-flatness tracker; requires `[fc.trajectory]` under v3)

[fc.health]
imu_stale_after_s   = 0.05
gnss_stale_after_s  = 0.5
baro_stale_after_s  = 10.0
mag_stale_after_s   = 0.2
overrun_burst_count = 5

[fc.fdir]
detector_kind          = "burst_counter" # "burst_counter" | "single_sample_glrt" | "cusum"
innovation_threshold   = 25.0
innovation_burst_count = 5
failsafe_burst_count   = 5

[fc.gain_schedule."mission.phases.ascent"]
rate_kp            = [0.5, 0.5, 0.5]
rate_kd            = [0.05, 0.05, 0.05]
attitude_kp        = [2.0, 2.0, 1.0]
elevator_limit_rad = 0.35
aileron_limit_rad  = 0.35
rudder_limit_rad   = 0.35
throttle_baseline  = 0.0

[fc.actuator_channels]
elevator = "delta_e"
aileron  = "delta_a"
rudder   = "delta_r"

[fc.phase_authority."mission.phases.ascent"]
autopilot_allowed = true
engines_allowed   = true
```

`frame_budget_us` is required when `[fc]` is present. Legacy
`innovation_gate` remains accepted for explicit chi-square thresholds,
but new scenarios should prefer `innovation_false_alarm_rate` so the
controller derives the correct threshold for 1-D, 3-D, and 6-D
measurements. `fc.actuator_channels` maps semantic controller channels
to `[[vehicle.assembly.effectors]]` ids; bare ids are resolved as
`vehicle.assembly.effectors.<id>`.

`[fc.health]` and at least one `[fc.gain_schedule.<phase>]` entry are
required for every FC-enabled scenario. There is no hidden
health-threshold or academic-gain fall-through in the scenario schema;
the scenario provenance record must explain those tunings. `[fc.fdir]`
is optional and defaults to the controller's burst-counter parameters
when omitted. `tau_gyro_bias_s` / `tau_accel_bias_s` are optional;
omitting them preserves random-walk bias dynamics (`tau = infinity`).

Schema v3 also supports powered-ascent reference generation:

```toml
[fc]
guidance = "ascent_reference"

[fc.ascent_reference]
method     = "pitch_program" # or "gravity_turn"
schedule_s = [0.0, 10.0, 30.0]
pitch_rad  = [0.0, 0.05, 0.20]

[fc.gain_schedule."mission.phases.powered_ascent"]
# normal three-loop gains for this phase
```

`guidance = "ascent_reference"` requires `vehicle.kind = "rigid_body"`, a
declared `powered_ascent` mission phase/state, and a matching gain-schedule
entry. `pitch_program` requires equal-length `schedule_s` / `pitch_rad` arrays
with strictly increasing times. `gravity_turn` accepts no schedule fields and
aligns body `+x` with inertial velocity once motion is established.
`explicit_reference` is still reserved and fails closed.

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

`environment.gravity_m_s2` is a non-negative magnitude. The
analytic-toy runner applies it along the toy `-z` direction; explicit gravity
vectors are a separate scenario-format extension.

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
  `[aero]` coefficient source is declared.

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
Runner simulations also install an automatic sea-level ground-impact
stop condition. `stop_s` remains the upper time bound; a descending
trajectory that crosses `position.z <= 0` stops earlier with a
`ground-impact` stop reason, even without a manual mission event.

TOML seed literals should stay in `0..=i64::MAX`. The scenario model
stores seeds as `u64`, but TOML integer syntax itself cannot represent values
above signed 64-bit range portably.

## Solver Profile

`[solver]` is optional and defaults to fixed-step RK4. Hypersonic
scenarios must declare it explicitly because solver choice,
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
leap_second_table_sha256 = "<64 hex chars>"
eop = "data/earth_orientation/example-eop.toml"
eop_sha256 = "<64 hex chars>"

[frames]
profile = "iers-tabulated"
```

Accepted frame profiles are `toy-fixed-earth`, `wgs84-uniform-rotation`,
`iers-tabulated`, and validation-only `spice-reference`.
`iers-tabulated` requires `[epoch]`, `epoch.scale = "UTC"`, and
`epoch.eop`; the EOP file is loaded through the same SHA-256-pinned
resolved-file path used for aero decks and motor curves.

The EOP table format currently consumed by the runner is deterministic
TOML with scenario-relative sample times:

```toml
format = "openbmp-eop-v1"

[[samples]]
time_s = 0.0
ut1_minus_utc_s = 0.102
x_pole_arcsec = 0.045
y_pole_arcsec = 0.312

[[samples]]
time_s = 60.0
ut1_minus_utc_s = 0.103
x_pole_arcsec = 0.045
y_pole_arcsec = 0.312
```

Samples must be strictly time-ordered and cover `[time.start_s,
time.stop_s]`. OpenBMP linearly interpolates UT1-UTC and polar motion,
applies compact IAU 1976 mean precession and IAU 1980 nutation from J2000
to date, uses the scenario UTC epoch to compute IAU Earth Rotation Angle,
and applies a compact polar-motion rotation in the ECI/ECEF transform. It
does not yet implement SPICE frame chains, IAU 2006/2000A CIO-based
transforms, or precession/nutation rate terms in velocity transport.

When `epoch.leap_second_table` is declared, the runner loads it through
the same resolved-file path and optional SHA-256 pin as other external
inputs. The field accepts either the deterministic OpenBMP TOML format
or a NAIF `KPL/LSK` leap-second text kernel containing
`DELTET/DELTA_AT` entries. The OpenBMP TOML format is:

```toml
format = "openbmp-leap-seconds-v1"

[[entries]]
effective_utc = "2015-07-01T00:00:00Z"
tai_minus_utc_s = 36

[[entries]]
effective_utc = "2017-01-01T00:00:00Z"
tai_minus_utc_s = 37
```

Entries must be strictly time-ordered. UTC ephemeris epochs use the
latest entry at or before `epoch.iso8601` to convert UTC -> TT -> TDB;
TT epochs are converted to TDB with the runner's compact deterministic
periodic correction. SPK ephemeris epochs may use `TDB`, `TT`, or `UTC`,
but `UTC` requires `epoch.leap_second_table` pointing at either an
OpenBMP TOML leap-second table or a NAIF LSK file such as
`naif0012.tls`.

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

The `openbmp check scenario.toml` command performs the following checks:

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

## Sounding-Rocket Extensions

These scenario-format extensions support sounding-rocket scenarios.
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

`[environment].gravity` selects one of `constant`, `point_mass`, `j2`,
`egm2008`, or `third_body`, each with its own required-coefficient set:

| `gravity` | Required | Rejected |
|---|---|---|
| `"constant"` | `gravity_m_s2` | `mu_m3_s2`, `r_e_m`, `j2`, `gravity_base`, `third_bodies`, `ephemeris`, `ephemeris_file`, `ephemeris_files` |
| `"point_mass"` | `mu_m3_s2` | `gravity_m_s2`, `r_e_m`, `j2`, `gravity_base`, `third_bodies`, `ephemeris`, `ephemeris_file`, `ephemeris_files` |
| `"j2"` | `mu_m3_s2`, `r_e_m` | `gravity_m_s2` |
| `"egm2008"` | — pinned WGS84 / EGM2008 zonal constants | `gravity_m_s2`, `mu_m3_s2`, `r_e_m`, `j2`, `gravity_base`, `third_bodies`, `ephemeris`, `ephemeris_file`, `ephemeris_files` |
| `"third_body"` | `gravity_base`, `third_bodies`; plus the selected base coefficients | `gravity_m_s2` |

For `gravity = "j2"` the dimensionless `j2` coefficient defaults to the
WGS84 value (1.082626683 × 10⁻³) when omitted.

`gravity = "third_body"` wraps a central Earth gravity model and adds
Sun/Moon point-mass perturbations using
`a_3 = mu_b * ((r_b - r) / |r_b - r|^3 - r_b / |r_b|^3)`.
`gravity_base` is one of `point_mass`, `j2`, or `egm2008`. The
`third_bodies` list accepts `"sun"` and/or `"moon"` and must be unique.
`ephemeris = "low_precision_sun_moon"` selects the built-in
deterministic analytical ephemeris; if `[epoch]` is omitted the
ephemeris starts at J2000, otherwise the runner parses
`epoch.iso8601` with `epoch.scale` `UTC`, `TT`, or `TDB`.
`ephemeris = "spk"` selects pinned binary SPK/BSP kernels supplied by
either singular `ephemeris_file` plus optional
`ephemeris_file_sha256`, or ordered `ephemeris_files` plus optional
`ephemeris_files_sha256`. The plural pin list, when present, must
match the file list length. Later files take precedence over earlier
files for overlapping SPK segments. The SPK reader evaluates seconds
past J2000 on the ephemeris-time/TDB axis, so `[epoch].scale` may be
`"TDB"`, `"TT"`, or `"UTC"`; `"UTC"` requires
`epoch.leap_second_table` as either OpenBMP TOML or a NAIF LSK text
kernel.

```toml
[epoch]
scale = "UTC"
iso8601 = "2000-01-01T12:00:00Z"

[environment]
frame_profile = "wgs84-uniform-rotation"
gravity       = "third_body"
gravity_base  = "egm2008"
third_bodies  = ["sun", "moon"]
ephemeris     = "low_precision_sun_moon"
atmosphere    = "none"
wind          = "none"
```

```toml
[epoch]
scale = "UTC"
iso8601 = "2017-01-01T00:00:00Z"
leap_second_table = "data/time/naif0012.tls"
leap_second_table_sha256 = "<64 hex chars>"

[environment]
frame_profile = "wgs84-uniform-rotation"
gravity       = "third_body"
gravity_base  = "egm2008"
third_bodies  = ["sun", "moon"]
ephemeris     = "spk"
ephemeris_files = [
  "data/ephemeris/de440s.bsp",
  "data/ephemeris/mission-overlay.bsp",
]
ephemeris_files_sha256 = [
  "<64 hex chars>",
  "<64 hex chars>",
]
atmosphere    = "none"
wind          = "none"
```

The SPK reader supports geometric Sun/Moon states from binary DAF/SPK
kernels with type 2 or type 3 Chebyshev segments, type 8/9 Lagrange
state segments, type 12/13 Hermite state segments, type 18 ESOC/DDID
subtype 0/1 packets, and type 20 Chebyshev velocity-only segments in
the J2000 frame. It also accepts the built-in SPICE `ECLIPJ2000`
inertial frame and rotates those segment states into OpenBMP's J2000
ECI chain. Type 2 velocities are derived from the Chebyshev position
derivative; type 3 velocities come from the segment velocity
coefficients; type 8/9 states are interpolated from equal/unequal-time
discrete position/velocity records; type 12/13 states use
equal/unequal-time Hermite interpolation of position and velocity
records; type 18 subtype 0 interpolates ESOC/DDID position and velocity
packets separately from their derivative fields, while subtype 1 uses
Lagrange interpolation of position/velocity packets; type 20 integrates
Chebyshev velocity polynomials from each record midpoint position
constant. It follows SPK segment priority inside each file and
preserves load-order precedence across a file list, so later files can
override earlier overlapping segments. It combines target/center chains
such as Solar-System-Barycenter -> Earth-Moon Barycenter -> Earth/Moon.
It does not yet implement light-time, stellar aberration, generic text
kernels beyond NAIF LSK leap-second files, non-J2000 frame transforms
beyond built-in `ECLIPJ2000`, or the remaining SPK segment types.

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

### Aerodynamic coefficient source

```toml
[aero]
deck         = "../../data/aero/synthetic-niskanen-ch6-rocket.toml"
deck_sha256  = "cd862c2af98a1f28dc86c6e754d311c7a724081ca91b80704ad89b2ec4cb5c27"
```

`[aero]` declares a coefficient source or a live method:

- `deck = "..."`: an external TOML deck resolved relative to the
  scenario file directory. Optional `deck_sha256` pins the file's
  SHA-256 digest; mismatches fail closed.
- `[aero.buildup]`: an inline launch-vehicle continuum drag buildup
  that bakes a deck at load time from vehicle geometry and the declared
  reference atmosphere.
- `[aero.method]`: a runtime method selector. `kind = "deck"` is the
  default and uses `deck` or `[aero.buildup]`; hypersonic live methods
  (`modified_newtonian`, `tangent_cone`, `tangent_wedge`,
  `free_molecular`) do not use an external deck and run directly in the
  force / moment stack. `kind = "hybrid"` blends continuum and
  free-molecular methods by runtime Knudsen number; its continuum
  low-Mach method may use `kind = "deck"` to reuse the same `deck` or
  `[aero.buildup]` source.

Declaring both `deck` and `[aero.buildup]` fails with
`ScenarioError::AmbiguousAero`.

The shipped decks in `data/aero/` are launch-vehicle fixtures whose
Mach coverage tops out in the low-supersonic range. Do not use a
deck-only method for Mach-20 re-entry unless the deck explicitly covers
that envelope with a documented extrapolation policy. Re-entry scenarios
should use `kind = "hybrid"` so a low-Mach deck can hand off to a
hypersonic continuum method and then to `free_molecular` as Knudsen
number rises.

Runtime hypersonic method example:

```toml
[aero]

[aero.method]
kind = "modified_newtonian"

[aero.method.modified_newtonian]
cp_max = 2.0
reference_area_m2 = 1.0
reference_length_m = 1.0
```

Hybrid Knudsen-bridge example:

```toml
[aero]

[aero.buildup]
# ... geometry-driven deck source used by the low-Mach continuum branch ...

[aero.method]
kind = "hybrid"

[aero.method.hybrid]
reference_length_m = 0.2
mach_handoff = 4.0

[aero.method.hybrid.continuum_low_mach]
kind = "deck"

[aero.method.hybrid.continuum_high_mach]
kind = "modified_newtonian"

[aero.method.hybrid.continuum_high_mach.modified_newtonian]
cp_max = 2.0
reference_area_m2 = 0.0314
reference_length_m = 0.2

[aero.method.hybrid.free_molecular]
reference_area_m2 = 0.0314
accommodation_normal = 1.0
accommodation_tangential = 1.0

[aero.method.hybrid.bridge]
kind = "linear" # cheng | erfc | linear
kn_lo = 0.01
kn_hi = 10.0
```

Rigid-body runs wire the live aero method into both force and
body-frame moment evaluation; point-mass runs consume only force.

### Live aerothermal coupling

`[aerothermal]` runs inside the runner loop when declared. The driver
samples the runtime atmosphere at the current state, builds an
`AerothermalContext`, evaluates Sutton-Graves or Fay-Riddell stagnation
heating, and emits `aerothermal.*` telemetry. Optional
`[aerothermal.thermal_toy]` and `[aerothermal.ablation]` sub-blocks
advance one step per kernel tick. `feedback = "mass"` is accepted only
for `rigid_body` vehicles and feeds the ablation gas mass rate into the
rigid mass model.

```toml
[aerothermal]
stagnation_kind = "sutton_graves" # or "fay_riddell"
nose_radius_m = 0.5
wall_temperature_k = 1500.0
wall_catalysis = "fully_catalytic" # or "non_catalytic"

[aerothermal.ablation]
virgin_material = "textbook_pica_like"
char_material = "textbook_char"
thickness_m = 0.05
n_nodes = 7
pyrolysis_enthalpy_j_kg = 2.4e6
gas_yield_fraction = 0.6
feedback = "mass" # "none" by default
```

Telemetry channels include `aerothermal.q_conv_w_m2`,
`aerothermal.q_rad_w_m2`, `aerothermal.h_aw_j_kg`,
`aerothermal.recovery_temperature_k`,
`aerothermal.stagnation_temperature_k`, `aerothermal.knudsen`,
`aerothermal.wall_temperature_k`,
`aerothermal.backwall_temperature_k`,
`aerothermal.recession_depth_m`,
`aerothermal.gas_mdot_kg_m2_s`, and
`mass.aerothermal_mass_loss_kg_s`.

Two deck formats are supported. **Schema 1** is the three-axis
`(mach, alpha, beta) → (CN, CD, CM)` deck documented in
[`software-architecture.md § Deck Format`](software-architecture.md#deck-format-in-house-toml).
**Schema 2** extends Schema 1 with optional control-effector
axes; see [Schema-2 aero decks](#schema-2-aero-decks) below.
For external decks, the schema discriminator lives inside the deck file
itself (`openbmp.aero_deck = 1` or `= 2`).
When `[multi_body]` is declared and `forces.models` includes `"aero"`,
`aero.mounted_to = "<body id>"` is required so post-separation lanes
can route aero forces to only the body that still carries the aero
surface model.

Inline buildup form:

```toml
[aero]

[aero.buildup]
body_diameter_m = 0.197
body_length_m = 2.34
surface_roughness_m = 6.0e-5
reference_area_m2 = 0.0305
reference_length_m = 0.197
center_of_gravity_from_nose_m = 1.1 # optional; defaults to body midpoint
mach_grid = { min = 0.0, max = 8.0, steps = 161 }
alpha_grid_deg = { min = 0.0, max = 8.0, steps = 9 }
reference_altitude_m = 0.0

[aero.buildup.nose]
shape = "ogive" # conical | ogive | von_karman | hemispherical
fineness = 3.5

[aero.buildup.afterbody] # optional boattail or flare
exit_diameter_m = 0.140
length_m = 0.180

[aero.buildup.fins] # optional
count = 4
root_chord_m = 0.30
tip_chord_m = 0.12
span_m = 0.16
thickness_ratio = 0.06
sweep_rad = 0.52
```

The buildup uses Sutherland viscosity from the sampled reference
temperature to compute bake-time Reynolds number, includes power-off
base drag and boattail/flare effects, and produces a fixed
`(mach, alpha, beta=0) -> (CN, CD, CM)` deck. Point-mass vehicles
consume the baked deck axially at `alpha = beta = 0`; rigid-body
vehicles resolve velocity into the body frame and query the alpha/beta
axes before rotating the reduced force back to ECI.

### Motor reference

```toml
[propulsion.motor]
file         = "../../data/motors/estes-c6-eng-derived.toml"
ignite_at_s  = 0.0
variant      = "solid"
mounted_to   = "upper"
file_sha256  = "da8272d3a7a135046c614e51b279971d37cac376f7aaaffdedc3ccc14d50ad4e"
```

`variant` is optional; when present, it must resolve to a registered
motor variant. Only the `solid` motor variant is wired.
`ignite_at_s` is the time since scenario start when the motor begins
burning; finite-required, no positivity rule (negative values are an
explicit pre-roll convention).
When `[multi_body]` is declared, `[propulsion.motor].mounted_to` is
required and must reference a declared body. The continuing or
separated lane that owns the motor receives its thrust and mass-rate;
other lanes skip it.

Instead of `file`, `[propulsion.motor]` may declare an inline
`[propulsion.motor.grain]` producer. `file` and `grain` are mutually
exclusive (`AmbiguousPropulsion`). The grain path is a forward internal
ballistics model: it produces the same validated `SolidMotor` thrust curve
the file path consumes.

```toml
[propulsion.motor]
variant     = "solid"
ignite_at_s = 0.0
mounted_to  = "upper"

[propulsion.motor.grain]
geometry              = "bates"       # end_burner | bates | tabulated
segments              = 4
outer_radius_m        = 0.025
core_radius_m         = 0.010
segment_length_m      = 0.090
throat_radius_m       = 0.0085
expansion_ratio       = 8.0
dry_mass_kg           = 0.0
provenance            = "synthetic/textbook inline grain regression"

[propulsion.motor.grain.propellant]
label          = "synthetic_textbook"
density_kg_m3  = 1841.0
burn_rate_a    = 0.00882
burn_rate_n    = 0.319
c_star_m_s     = 912.0
gamma          = 1.131
web_steps      = 200
```

`end_burner` uses `cross_section_area_m2` and `length_m`; `bates` uses
`segments`, `outer_radius_m`, `core_radius_m`, and `segment_length_m`;
`tabulated` uses `points = [[web_m, burn_area_m2], ...]` plus
`propellant_volume_m3`. All parameters are synthetic/textbook/public and are
rejected if non-finite or outside the quasi-steady envelope.

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
otherwise. The parser also accepts `kind = "layered"` with a `layers`
table, `kind = "gust"` with Dryden `intensity_m_s`,
`length_scale_m`, and `airspeed_m_s` parameters, and `kind = "hwm14"`
with optional HWM14 scalar inputs:

```toml
[wind]
kind = "hwm14"
year = 1995
day_of_year = 150
utc_s = 43200.0
latitude_deg = -45.0
longitude_deg = -85.0
ap_current_3h = 80.0
```

The `hwm14` runtime evaluates the public HWM14 quiet-time model plus
DWM07 disturbance winds from bundled data files. Omitted HWM14 fields
use the public `checkhwm14` height-profile case shown above. Use
`ap_current_3h = -1.0` for quiet-time winds only. Selecting any
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
required (the model is parameterless). Runner-side `us_standard_1976`
sampling preserves the 0-86 km model in-envelope and returns a
zero-density vacuum sample above the ceiling so high-apogee coast or
entry trajectories do not fault at the first exoatmospheric aero
sample. Use `nrlmsise00` or `nrlmsis2_compat` when upper-atmosphere
density above 86 km is physically relevant rather than negligible.

### Sensors

`[sensors.<name>]` declares one synthetic sensor per entry. The
`<name>` becomes the sensor's stable identifier; reordering entries
does not shift RNG streams.

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
all other kinds require `file`. The base set is `imu` /
`barometer`; the further kinds are `gnss` (IS-GPS-200
receiver-output noise: per-axis Gaussian on position + velocity
plus an OU position-bias drift), `magnetometer` (per-axis
Gaussian on the body-frame WMM 2025 truth field plus constant
3×3 soft-iron and 3-vector hard-iron biases), and `star_tracker`
(small-angle Gaussian rotation-vector perturbation per axis).

### Force model registry

Force-model names accepted in `[forces].models`:

| Name | Source | Notes |
|---|---|---|
| `gravity` | `openbmp-physics` | Constant, point-mass, or J2 (selected by `[environment].gravity`) |
| `aero` | `openbmp-aero` | Requires `[aero]` block with `deck` or `buildup` |
| `thrust` | `openbmp-propulsion` | Requires `[propulsion.motor]` block |

The list is ordered and deterministic; reordering changes telemetry
bytes.

Schema v3 also accepts phase-gated overrides:

```toml
[forces]
models = ["gravity"] # default stack

[[forces.phase_override]]
phase = "entry_interface"
models = ["gravity", "aero", "aerothermal_diagnostics"]
```

The override phase must match a declared mission phase/state. The
runner builds the union of default and override force models, but in
phases without an override only `[forces].models` plus runtime internal
models such as recovery drag are active. When overrides are present,
telemetry includes `forces.active_models` so the selected stack is
observable after the run.

### Hash pinning

Every external file referenced by the scenario can carry an optional
`*_sha256` companion field (`aero.deck_sha256`,
`propulsion.motor.file_sha256`, `sensors.<name>.file_sha256`,
`epoch.eop_sha256`, `environment.ephemeris_file_sha256`, or ordered
`environment.ephemeris_files_sha256`). When present, the parser
computes the file's SHA-256 digest at load time and fails closed on
mismatch. The plural ephemeris pin list must have the same length as
`environment.ephemeris_files`. When absent, the digest is still
computed and surfaced by `openbmp check`; recording those digests in
telemetry headers is handled runner-side.

`openbmp check` surfaces resolved digests in its output:

```
openbmp check: ok — niskanen-2009-chapter6 (Checked)
  telemetry.output.parquet -> .../out/niskanen-2009-chapter6.parquet
  aero.deck -> .../data/aero/...toml (sha256:cd862c2af98a1f28dc86c6e754d311c7a724081ca91b80704ad89b2ec4cb5c27)
  propulsion.motor.file -> .../data/motors/estes-c6-eng-derived.toml (sha256:da8272d3a7a135046c614e51b279971d37cac376f7aaaffdedc3ccc14d50ad4e)
  environment.ephemeris_file -> .../data/ephemeris/de440s.bsp (sha256:...)
  environment.ephemeris_files[1] -> .../data/ephemeris/mission-overlay.bsp (sha256:...)
```

The digest is the SHA-256 of the file bytes encoded as 64 lower-case
hex characters; pin strings are normalised before comparison so
upper-case input still verifies.

## Mission and Assembly Extensions

The optional declarative `[mission]` block, when
declared, drives the runner to build an event-driven mission graph from the
scenario; scenarios without `[mission]` parse and run
identically without one.

### Mission block

> **Superseded.** The flat-DAG `[mission]` block
> documented here is the v3 contract. The scenario
> format v4 adds hierarchical `[[mission.states]]` blocks,
> `[[mission.regions]]` orthogonal-region declarations, and a
> `[mission.scope]` human-readable scenario classifier. The
> authoritative format
> reference is [`mission-graph-architecture.md`](mission-graph-architecture.md);
> the canonical state vocabulary is in
> [`mission-states-vocabulary.md`](mission-states-vocabulary.md).
> The v3 syntax below remains accepted; the shipped
> `niskanen-2009-chapter6-with-mission.toml` scenario exercises the
> v4 `states` / `regions` / `scope` path.

The v3 `[mission]` block declares phases, events, and transitions. The
v4 canonical example mirrors the
[`niskanen-2009-chapter6-with-mission.toml`](../scenarios/sounding-rocket/niskanen-2009-chapter6-with-mission.toml)
scenario:

```toml
[mission]
initial_phase = "ascent"
test_only_state_override = false

[mission.scope]
kind = "sounding_rocket"

[[mission.states]]
id    = "ascent"
label = "powered + coast ascent"

[[mission.states]]
id    = "descent"
label = "post-apogee descent"

[[mission.regions]]
id            = "health"
initial_state = "nominal"

[[mission.regions.states]]
id    = "nominal"
label = "all systems nominal"

[[mission.regions.states]]
id    = "abort_requested"
label = "safe-state request latched"

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
| `at_velocity` | `velocity_m_s: f64`, optional `falling: bool = false` | Fires when speed magnitude crosses `velocity_m_s`; `falling = false` selects the rising edge and `falling = true` selects the falling edge. |
| `at_dynamic_pressure` | `pressure_pa: f64 >= 0`, `falling: bool` | Fires when dynamic pressure crosses `pressure_pa` in the direction set by `falling`. Runner event evaluation computes `q = 0.5 * rho * |v|^2` from the selected runtime atmosphere; use `us_standard_1976`, `piecewise_exponential`, `nrlmsise00`, or `nrlmsis2_compat`. |
| `at_relative_distance` | `body: string`, `distance_m: f64 > 0`, optional `reference_body: string`, optional `falling: bool = false` | Rigid-body `[multi_body]` only. Fires when the range between `body` and `reference_body` crosses `distance_m`; if `reference_body` is omitted, the current primary lane is used. The trigger is false until the named bodies are active propagated lanes. |

The `kind = "scripted"` trigger is rejected at parse time with a
typed deferral error: scripted triggers are not supported. The
deterministic per-effector `command_schedule` (see
[Control effectors](#control-effectors) below) covers the
common scripted-command case without a separate trigger surface.

Example post-deployment clearance marker:

```toml
[[mission.events]]
id      = "rv1_clear"
trigger = { kind = "at_relative_distance", body = "rv1", distance_m = 25.0 }
action  = { kind = "emit_telemetry_marker", tag = "rv1_clear" }
once    = true
```

#### Action vocabulary

| `action.kind` | Required fields | Semantics |
|---|---|---|
| `enter_phase` | `phase: string` (declared phase id) | Sets the active mission phase; visible via runner-side telemetry / diagnostics. |
| `emit_telemetry_marker` | `tag: string` (snake_case) | Allocates a `bool` telemetry channel `mission.marker.<tag>`; runner writes `true` on every step the event fires, `false` on every other step. Channel allocation is alphabetical by tag for declaration-order independence. |
| `stop` | `label: string` | Halts the run with `StopReason::MissionEnded { label }`. Distinct from `EndTime` so determinism telemetry can distinguish CLI-driven stops from scenario-driven mission ends. |
| `effector_override` | `id: string` (declared effector id), `command: f64` (finite) | One-shot command override for the named effector on the next runner step. Resolves the declared id against the runner's effector rack via FNV-1a-64 of `vehicle.assembly.effectors.<id>`. Unknown ids are rejected by `openbmp check`. The kernel records the action; the runner drains it from the per-step fired-event queue and applies it on the next rack tick before the kernel step. Override wins over any declared `command_schedule` for that rack tick only. |
| `engine_command` | `id: string` (declared engine id), `command: { throttle_unit: f64 ∈ [0,1], gimbal_pitch_rad: f64, gimbal_yaw_rad: f64, ignite: bool, shutdown: bool }` | Per-engine command targeting a declared `[[vehicle.assembly.engines]]` by id. Resolves the declared id via FNV-1a-64 of `vehicle.assembly.engines.<id>`. Unknown ids are rejected by `openbmp check`. Kernel records; runner-side `EngineRack` drains and applies on the next rack tick before the kernel step. `ignite=true` is honoured only from `Idle`; `shutdown=true` only from `Igniting` / `Burning`. Throttle / gimbal values are clamped to engine limits at apply time. |
| `jettison_stage` | `body: string` (declared body id) | Stage-separation command targeting a declared `[[vehicle.assembly.bodies]]` by id. Requires `vehicle.kind = "rigid_body"`, exactly one matching `[[multi_body.separation]]`, no duplicate jettison of the same body, and momentum conservation when `conserve_momentum = true`. The runner executes this for fixed-step RK4 rigid-body profiles. Gravity-only profiles remain valid; aero, thrust, tanks, recovery, and effectors are allowed only when each resource declares an explicit owning body. |
| `jettison_bodies` | `bodies: [string, ...]` (declared body ids) | Batch stage-separation command. Every listed body is partitioned from the same pre-separation rigid-body state and appended as an independent lane on the same event tick. Each body requires a matching `[[multi_body.separation]]` with the same `event_id`; duplicate body ids are rejected. This is the coordinated deployment path for a bus releasing multiple RV-like bodies. |
| `deploy_recovery` | `id: string` (declared recovery id), `command: "deploy" \| "deploy_drogue" \| "deploy_main" \| "stow"` | Recovery-device command targeting a declared `[[vehicle.assembly.recovery]]` by id. Resolves via FNV-1a-64 of `vehicle.assembly.recovery.<id>`. Unknown ids and kind-incompatible commands are rejected by `openbmp check`; runner-side `RecoveryRack` drains accepted firings on the next rack tick before the kernel step. |

The reserved action `separation` is rejected at parse time with a typed
deferral error because it does not identify the departing body. The
`effector_override`, `engine_command`, `jettison_stage`,
`jettison_bodies`, and `deploy_recovery` actions are wired end-to-end
inside their documented validation envelopes.

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

### Mission block v4

The v4 mission block adds hierarchical state
declarations, orthogonal-region declarations, and the scenario-scope
classifier. v3 scenarios continue to parse byte-identically because
every v4 field is optional and the v3 → v4 lifting pass promotes
`[[mission.phases]]` to flat (depth-1) `[[mission.states]]` while
preserving every FNV-1a-64 id.

#### Hierarchical states (`[[mission.states]]`)

```toml
[[mission.states]]
id     = "in_flight"
label  = "in-flight composite"
# `parent` omitted = top-level state.

[[mission.states]]
id     = "in_flight.boost.first_stage_burn"
parent = "in_flight"
label  = "first stage burning"
allowed_effectors = ["pitch", "yaw"]
allowed_engines   = ["s1.merlin1"]

# Action arrays are HAL-portable: `enter_state`,
# `emit_telemetry_marker`, `stop`. Scenario-script actions (engine /
# effector / separation / recovery) belong in `[[mission.events]]`
# bindings, not in state action lists.
[[mission.states.on_entry]]
kind = "emit_telemetry_marker"
tag  = "first_stage_ignite"

[[mission.states.on_exit]]
kind = "emit_telemetry_marker"
tag  = "first_stage_meco"
```

Validation invariants (in addition to v3 rules):

- Every `parent` references a declared state.
- Parent relation is acyclic (no state is its own ancestor).
- Flat sibling top-level states are valid; graph reachability is
  checked by `[[mission.transitions]]`, not by the parent relation.
- Canonical-form sort: states by `(depth-from-root,
  parent-StateId.value(), StateId.value())`; transitions extend the
  five-tuple with the ancestor-LCA depth.

#### Orthogonal regions (`[[mission.regions]]`)

```toml
[[mission.regions]]
id            = "health"
initial_state = "nominal"

[[mission.regions.states]]
id    = "nominal"
label = "all systems nominal"

[[mission.regions.states]]
id    = "degraded"
label = "non-fatal degradation"
```

When `[[mission.regions]]` is omitted, the runner supplies the four
canonical regions (`mission`, `health`, `comms`,
`estimator_regime`). The commander keeps `mission` synced to the
active state and derives `safe_state_requested` from `health`.
`CrossRegionGuard` exists in `openbmp-mission` as a portable
primitive, but scenario-level `guard` / `priority` fields are not part
of the parser surface.

#### Scenario scope tag

```toml
[mission.scope]
kind = "sounding_rocket"   # one of: sounding_rocket,
                           # propulsive_landing, orbital_insertion,
                           # re_entry, closed_loop_test
```

Used only for human-readable telemetry breadcrumbs; does not gate
behaviour.

#### Test-only override channel

```toml
[mission]
test_only_state_override = true   # default false
```

When `true`, the simulator may write the
`commander.scenario_state_override` topic to force the commander into
a specific state for validation. HAL builds of `openbmp-fc`
(`--features hal --no-default-features`) compile out this topic
entirely; setting the flag in a HAL deployment is a load-time error.

#### Migrating v3 → v4

v3 scenarios load against the v4 parser byte-identically. The
implicit lifting pass:

1. Reads `[[mission.phases]]` into a flat `[[mission.states]]` view
   (every state has `parent = None`, depth = 0).
2. Auto-declares the four canonical regions if `[[mission.regions]]`
   is omitted.
3. Leaves `scope` unset and `test_only_state_override` `false`.

The scenario format v4 contract is finalised; the hypersonic extensions
may add scope variants for hypersonic profiles.

### Vehicle assembly

When `[vehicle.assembly]` is declared, the scenario describes the
vehicle as a tree of bodies. Schema v2 requires this block for every
scenario. The assembly supports single-body and multi-body shapes, with
effectors, engine clusters, tanks, and
recovery devices as child blocks of the assembly. Sensors remain
outside the assembly. The assembly bodies are the dry mass and
dry inertia source of truth.

The canonical multi-body example mirrors
[`scenarios/multi-body/two-body-fairing.toml`](../scenarios/multi-body/two-body-fairing.toml):

The coordinated deployment example
[`scenarios/multi-body/bus-rv-deployment.toml`](../scenarios/multi-body/bus-rv-deployment.toml)
uses `jettison_bodies` to release two rigid lanes from the same
pre-separation bus state and emits a relative-distance clearance
marker after deployment.

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

`[[vehicle.assembly.effectors]]` — see
[Control effectors](#control-effectors) below.
`[[vehicle.assembly.engines]]` — see
[Engine clusters](#engine-clusters) below.
`[[vehicle.assembly.tanks]]` — see
[Tanks and slosh](#tanks-and-slosh) below.
`[[vehicle.assembly.recovery]]` — see
[Recovery and descent](#recovery-and-descent) below.

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
- Assembly child blocks are validated by their respective sections:
  `effectors`, `engines`, `tanks`, and `recovery`. Unknown child blocks
  remain parse errors.

#### Assembly limitations

- The runner consumes the assembly's dry mass properties for kernel
  mass construction. Force / moment construction still uses
  runner-side adapters until the remaining single-motor adapter path
  is retired behind the engine-cluster infrastructure.
- Point-mass propagation only uses the assembled dry mass; body CG and
  inertia affect rigid-body mass properties, not point-mass dynamics.

### Control effectors

`[[vehicle.assembly.effectors]]` declares one or more
`ControlEffector` instances mounted on the assembly. This provides
the effector state machine, a deterministic `command_schedule`, the
runner-side `EffectorRack`, and per-effector deflection telemetry.
The effector deflection is observable in the Parquet via
`effector.<id>.actual`. With a schema-1 aero deck the deflection is
**not consumed by force / moment evaluation**; schema-2 deck
consumption couples it in.

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

`kind.kind` is tagged on the inner `kind` field. One variant is
currently wired; nonlinear / multi-axis / smart actuators are possible
extensions.

| `kind.kind` | Required fields | Semantics |
|---|---|---|
| `linear_actuator` | `tau_s: f64` (optional, default `0.0`) | First-order lag with rate clamp + saturation + deadband + pure-delay buffer. `tau_s = 0` collapses to a rate-clamped tracker (no lag). The latency must be an integer multiple of the scenario's `time.dt_s` (sub-`dt` latency is rejected at construction). |

#### Limits vocabulary

`limits` is a flat table with five required fields and three optional throttle
dynamics fields:

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

`fault` is optional and load-time only: a fault
declared in the scenario is injected at construction and persists
for the run. Run-time fault injection (mid-mission failures) is
not supported.

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
- `mounted_to`, when present, must reference a declared body. It is
  required for every effector when `[multi_body]` is declared so
  post-separation aero-axis and direct-torque ownership is explicit.
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

#### Limitations

- With a schema-1 aero deck the effector deflection is observable in
  the Parquet but is **not** consumed by force / moment evaluation.
  [Schema-2 aero decks](#schema-2-aero-decks) couple it in:
  effector `actual` deflections flow through the kernel into
  the deck lookup.
- Faults are load-time only; run-time fault injection is not supported.
- Only the `linear_actuator` kind ships. Nonlinear / multi-axis /
  smart-actuator variants are possible extensions.

### Schema-2 aero decks

Schema-2 extends the aero deck format with optional **control-effector
axes**. A schema-2 deck is identified by `openbmp.aero_deck = 2` in the
deck file and adds an `[axis_order]` block declaring the locked axis
ordering, plus per-effector grid axes (e.g. `delta_e_deg = [-20, 0,
20]`). At runtime, the runner-side `EffectorRack` snapshot flows
through the kernel's `EffectorActualsView` into the deck's multilinear
lookup, so the rate-limited / saturated `EffectorState.actual`
actually modulates `(CN, CD, CM)`.

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
delta_e_deg = [-20.0, 0.0, 20.0]   # effector axis

[axis_order]
order = ["mach", "alpha", "beta", "delta_e_deg"]

[coefficients.cn]
data = [/* row-major over the 4-axis cartesian product */]

[coefficients.cd]
data = [/* ... */]

[coefficients.cm]
data = [/* ... */]

[interpolation]
method        = "multilinear"   # required; only method wired
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
- Up to 3 effector axes (6 axes total) are supported.
  Larger decks are not yet supported.
- The `[grid]` table must declare exactly the axes in `axis_order` —
  extra keys are rejected, missing keys are rejected. The three base
  axes use the existing schema-1 grid keys (`mach`, `alpha_deg`,
  `beta_deg`); effector axes use their `axis_order` name verbatim.
- `interpolation.method` must be `"multilinear"`. Other methods are
  reserved for hypersonic work.
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
at runner build time with `RunnerError::AeroEffectorMismatch`, before
any kernel step is taken.
Schema-1 decks and decks without effector axes skip this check
entirely.

#### Interpolation

Multilinear over N axes (3 ≤ N ≤ 6). The reduction collapses the
**innermost axis first** (the last axis in `axis_order`) and walks
outward; this preserves bit-identical f64 outputs at N = 3 against the
trilinear path. The locked operand order is part of the
byte-stability contract and is asserted by a 1024-case property test
in `crates/openbmp-aero/src/deck.rs`.

#### Determinism

Schema-1 aero-deck fixtures are bit-identical under the schema-2 deck
loader. The migrated schema-v2 scenarios continue to produce
byte-identical Parquet. The kernel-owned
`effector_actuals: BTreeMap<String, f64>` snapshot is empty by
default; the runner only populates it when at least one schema-2 deck
axis matches a scenario effector. Empty snapshot → empty
`EffectorActualsView` → schema-1 lookups ignore the view → byte-stable
legacy path.

The canonical schema-2 example ships at
[`scenarios/effector-elevon-aero/single-elevon-aero-deflected.toml`](../scenarios/effector-elevon-aero/single-elevon-aero-deflected.toml)
and the deck at
[`data/aero/synthetic-elevon-1d.toml`](../data/aero/synthetic-elevon-1d.toml).

### Engine clusters

`[[vehicle.assembly.engines]]` declares one or more `EngineModel`
instances mounted on the assembly. This provides the
`LiquidEngine` reference impl plus the runner-side `EngineRack`
and the kernel-side cluster adapters (force, mass). Engines
respond to per-engine `engine_command` mission events; per-engine
`command_schedule` (effector-style scripted commands) is **not**
in scope — scripted command sequences flow through the
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

`kind.kind` is tagged on the inner `kind` field. One variant is
currently wired; hybrid / cold-gas / chamber-pressure models are
possible extensions.

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
| `throttle_slew_per_s` | 1/s | optional; non-negative or `+inf`; default `+inf` preserves instantaneous throttle |
| `min_throttle_unit` | — | optional; finite in `[0, 1]`; non-zero commands below the floor clamp up |
| `isp_throttle_falloff` | — | optional; finite in `[0, 1)`; linear low-throttle Isp derate |

#### Propellant budget and feed coupling

An engine may declare a `propellant` sub-table that binds its mass flow to one
or two tanks. The runner uses a one-step lag: the current engine snapshot
sets next-step tank drain and depletion shutdown, preserving deterministic
force/tank ordering.

```toml
[[vehicle.assembly.engines]]
id                 = "main"
mounted_to         = "stage1"
kind               = { kind = "liquid_engine" }
mount_point_body_m = [0.0, 0.0, 0.0]
limits             = { max_thrust_n = 5000.0, isp_s = 300.0,
                       ignition_transient_s = 0.2,
                       shutdown_transient_s = 0.2,
                       max_gimbal_rad = 0.087,
                       throttle_slew_per_s = 2.0,
                       min_throttle_unit = 0.4,
                       isp_throttle_falloff = 0.05 }
propellant         = { oxidizer_fuel_ratio = 2.3,
                       fuel_tank = "tank_fuel",
                       oxidizer_tank = "tank_ox",
                       feed = "blowdown",
                       residual_reserve_kg = 0.5 }
```

`oxidizer_fuel_ratio = 0.0` is monopropellant and rejects
`oxidizer_tank`; positive O/F requires `oxidizer_tank`. Bound tanks must exist
and be mounted to the same body as the engine. `feed = "regulated"` is the
default. `feed = "blowdown"` requires each bound tank to declare
`ullage = { initial_pressure_pa = ..., gas_gamma = ... }`; delivered thrust
and Isp scale with the isentropic ullage pressure ratio.

#### Mount geometry

`mount_point_body_m: [x, y, z]` — body-frame mount position in
metres. Used by the rigid-body kernel's cluster moment adapter
(`mount × thrust_body`, summed in scenario-declared order).
Point-mass scenarios store the mount points but don't use them.

#### Cluster layout

Optional `cluster_layout` on `[vehicle.assembly]`: one of
`axial | ring | octaweb | custom` (default: `custom`). The tag
informs telemetry and docs; it has no behavioural use
today. The layout may later drive symmetry-aware
fault scenarios or controller-side allocation tables.

#### Faults

`fault` is optional and load-time only: a fault
declared in the scenario is injected at construction and persists
for the run. Run-time fault injection is not supported
(mirrors the effector faults).

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
- `mounted_to`, when present, must reference a declared body. It is
  required for every engine when `[multi_body]` is declared so
  thrust, moment, and consumed-mass routing is per body after
  separation.
- `limits.{max_thrust_n, isp_s}` finite + strictly positive.
- `limits.{ignition_transient_s, shutdown_transient_s, max_gimbal_rad}`
  finite + non-negative.
- `limits.throttle_slew_per_s` non-negative or `+inf`;
  `min_throttle_unit` in `[0, 1]`; `isp_throttle_falloff` in `[0, 1)`.
- Engine `propellant` bindings require declared tanks on the same owner body;
  positive O/F requires `oxidizer_tank`; monopropellant rejects it; blowdown
  requires tank `ullage`.
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

#### Limitations

- Rigid-body cluster mass-properties debit each engine's
  `consumed_kg` from the body that owns that engine. Inertia
  evolution from engine propellant geometry is not modelled; tanks
  carry their own moving-mass inertia contribution.
- Faults are load-time only.
- Only the `liquid_engine` kind ships.
- Per-engine `command_schedule` (effector-style declarative
  scripts) is out of scope; engines drive only via
  `mission.events[*].action.engine_command`.

The canonical example ships at
[`scenarios/multi-engine-octaweb/four-engine-shutdown.toml`](../scenarios/multi-engine-octaweb/four-engine-shutdown.toml).

### Tanks and slosh

The `[[vehicle.assembly.tanks]]` block and the
runner-side tank rack drive moving-mass dynamics through the
kernel snapshot path (mirroring the engine-cluster
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
- `propellant` — `{ density_kg_m3, label }`. Textbook
  density only; fielded propellant data is rejected per
  `safety-boundaries.md`.
- `initial_fill_fraction` — `[0, 1]`. Initial fluid mass is
  `geometry.volume × density × fill`.
- `moving_mass` — `kind`-tagged enum:
  - `rigid_liquid` — no slosh (rigid-liquid toy);
  - `equivalent_pendulum { damping_ratio_zeta }` — Abramson
    cylindrical-tank antisymmetric fundamental mode;
  - `equivalent_spring_mass { damping_ratio_zeta }` — translational
    alternative;
  - `baffled_pendulum { base_damping_ratio_zeta }` — pendulum +
    `baffle_model.damping_increment_zeta`.

#### Optional fields

- `baffle_model = { damping_increment_zeta }` — additive damping
  increment consumed only by `baffled_pendulum`: a scalar increment
  per Abramson Eq 7-46 simplified.
- `ullage = { initial_pressure_pa, gas_gamma }` — pressurant state required
  when an engine bound to this tank selects `feed = "blowdown"`.
- `drain_rate_kg_per_s` — drain is decoupled from
  engine propellant budgets. Defaults to `0.0` (no scenario drain); engine
  coupling adds its own deterministic one-step-lag drain rate.
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
- `ullage` requires `initial_fill_fraction < 1.0`; blowdown engine bindings
  reject tanks without `ullage`.
- `drain_rate_kg_per_s × time.dt_s` must not exceed the initial tank
  fluid mass; single-step emptying is rejected rather than
  silently clamping away the moving-mass dynamics.

#### Determinism

- Tank ids are FNV-1a-64 of `vehicle.assembly.tanks.<id>`; the
  runner-side rack stores them in a `BTreeMap<TankId, Tank>` for
  deterministic iteration on macOS `SipHash` builds.
- Slosh integration is semi-implicit (symplectic) Euler with
  locked operand order — single sub-step per kernel base tick.
  Semi-implicit Euler is used rather than forward Euler:
  pure explicit Euler is unstable for an undamped harmonic
  oscillator and cannot meet the 1 % energy-conservation gate
  over 100 oscillations at any practical `dt`. The semi-implicit
  variant has the same operation count, the same locked operand
  order, and the same bit-stable replay properties.
- The slosh state advances using **prior step's** `(accel_body,
  omega_body)` — the documented one-step lag that breaks the
  circular dependency between the tank's reaction force and the
  kernel's per-step force evaluation. The first step uses zeros.

#### Limitations

- Engine-cluster ↔ tank drain coupling is **not** wired. Tanks
  declare a constant `drain_rate_kg_per_s` (default 0); engines
  track their own propellant accounting. A future revision could
  unify the two.
- The `TankRackMassAdapter` adds each tank's `mass_kg` to the
  vehicle total. For a tank intended as the cluster's propellant
  store this overcounts the propellant (the cluster's
  `EngineClusterMassAdapter` already debits consumed propellant
  from the dry mass). The exit-criterion scenario
  sizes the tank at about 5 % of vehicle dry mass to keep the
  overcount small.
- Rigid-body mass-properties consume each tank snapshot for the body
  named by `mounted_to`: `mass_kg` contributes to that lane's mass,
  `cg_offset_body_m` is applied from the tank mount point, and
  `inertia_delta_body_kg_m2` is added to the lane inertia tensor.
- Slosh telemetry channels (`tank.<id>.slosh_angle_rad`,
  `tank.<id>.fluid_kg`, etc.) are not separately exposed;
  the e2e test verifies determinism via
  full-Parquet byte equality rather than per-tank channels.

The canonical example ships at
[`scenarios/sloshing-tank/sloshing-tank.toml`](../scenarios/sloshing-tank/sloshing-tank.toml).

### Entry profile

`[entry_profile]` is a schema-v3 block for descent / entry profile
handoff validation and entry diagnostics. It consumes the existing
mission graph, Allen-Eggers ballistic-entry closed forms, and Vinh
lifting-entry equations. It accepts no target, aimpoint, or desired
landing coordinate.

```toml
[entry_profile]
mode = "ballistic"                # "ballistic" | "lifting"
entry_interface_altitude_m = 122000.0
final_descent_altitude_m = 5000.0

# Optional; defaults shown.
surface_density_kg_m3 = 1.225
scale_height_m = 7000.0
nose_radius_m = 1.0       # optional; enables stagnation heating diagnostics
```

`mode = "ballistic"` enables the runner-side
`entry_profile_for_sample` report using Allen-Eggers peak-deceleration
diagnostics. When `nose_radius_m` is present, the same report also includes
Sutton-Graves / Allen-Eggers peak convective heat flux, peak-heating altitude,
and convective heat load. `mode = "lifting"` additionally requires
`vehicle.kind = "rigid_body"`, a `lifting_entry` mission phase,
`lift_to_drag_ratio`, and `[entry_profile.corridor]`:

```toml
[entry_profile.corridor]
max_heat_rate_w_m2 = 1000000.0
max_load_factor_g = 8.0
flight_path_angle_band_rad = 0.2
nominal_bank_rad = 0.0
max_bank_rad = 1.2
```

Validation requires an atmospheric aero force path (`[aero]` and
`"aero"` in `forces.models`), mission phases named `entry_interface`,
`final_descent`, and `recovery`, and an
`at_altitude_descending` event that enters `entry_interface` at
`entry_interface_altitude_m`. If `final_descent_altitude_m` is set, a
matching descent-altitude handoff to `final_descent` is required.
Entry interfaces above the USSA76 86 km ceiling require
`environment.atmosphere = "piecewise_exponential"`.

### Landing footprint

`[landing_footprint]` is a schema-v3 offline post-processing block for
range-safety / recovery analysis. It is not connected to the flight
controller and accepts no desired landing coordinate.

```toml
[landing_footprint]
method = "egm2008"
cull_altitude_m = 0.0
include_geodetic = false

[landing_footprint.dispersion]
one_sigma_semi_major_m = 25.0
one_sigma_semi_minor_m = 10.0
orientation_rad = 0.0

[landing_footprint.monte_carlo]
samples = 1024
seed = 1234
confidence_levels = [0.5, 0.9, 0.99]

[landing_footprint.monte_carlo.output]
samples_csv = "out/footprint-mc-samples.csv"
samples_parquet = "out/footprint-mc-samples.parquet"
summary_toml = "out/footprint-mc-summary.toml"

[landing_footprint.monte_carlo.wind]
kind = "constant"
sigma_ned_m_s = [2.0, 1.0, 0.0]
speed_scale_sigma = 0.0

[landing_footprint.monte_carlo.ballistic_coefficient]
nominal_m2_kg = 0.01
sigma_m2_kg = 0.002
distribution = "normal"
min_m2_kg = 0.0
max_m2_kg = 0.05

[landing_footprint.monte_carlo.burnout_state]
position_sigma_eci_m = [1.0, 1.0, 0.5]
velocity_sigma_eci_m_s = [0.5, 0.5, 0.2]
time_sigma_s = 0.01
```

Consumed methods are:

| Method | Required `environment.gravity` | Propagation |
|---|---|---|
| `constant_gravity` | `constant` | Closed-form flat constant-gravity crossing. |
| `j2` | `j2` | Fixed-step RK4 propagation under the scenario J2 gravity parameters. |
| `egm2008` | `egm2008` | Fixed-step RK4 propagation under the pinned zonal-only EGM2008 degree-2 through degree-6 model. |

Every method requires a mission phase named `coast` or
`ballistic_descent`. `include_geodetic = true` additionally requires
`[frames.local_origin]`; otherwise the offline report contains only
range-relative `downrange_m` / `crossrange_m` output. The optional
dispersion block is a declared ellipse source. If absent, no dispersion
ellipse is reported. Numerical Earth-gravity methods stop at the WGS84
radial ellipsoid surface plus `cull_altitude_m`.

`[landing_footprint.monte_carlo]` enables offline sampled dispersion. It
requires at least one declared uncertainty source under
`wind`, `ballistic_coefficient`, or `burnout_state`; a Monte-Carlo block
with no source fails closed. The runner samples deterministic streams
from the Monte-Carlo seed (or `[time].seed` when omitted) plus the sample
index, propagates each sample with drag and wind using the sampled
ballistic coefficient, and writes the declared CSV/Parquet sample cloud
plus TOML summary. `wind.kind` accepts `constant`, `layered`, `hwm14`, or
`ensemble`; the current offline propagator consumes the sampled local-NED
perturbation as the constant wind vector for that footprint sample.

The Monte-Carlo summary includes an output-only `[accuracy]` block:
`cep50_m` is the empirical 50% circular radius about the successful sample
mean, while `mean_miss_distance_from_nominal_m` is the radial offset from
the nominal forward footprint to that sample mean. The sample cloud also
contains per-sample `offset_*_from_nominal_m`,
`miss_distance_from_nominal_m`, and `radial_distance_from_mean_m`
channels. `[[quantiles]]` are mean-centered radial-distance quantiles;
`[[nominal_miss_distance_quantiles]]` are radial-error quantiles about the
nominal forward footprint. These diagnostics do not add a target, aimpoint,
or desired landing coordinate.

As everywhere else in the profile work, fields naming a desired landing
location, aimpoint, miss distance, or equivalent targeting concept are
rejected by the lint before deserialization.

### Staging analysis

`[staging_analysis]` is a schema-v3 offline post-processing block. It runs the
ideal loss-free rocket equation and writes compact report metadata under
`openbmp.staging_analysis.*` in the telemetry schema. It is not a guidance
input and cannot express range, launch site, target, azimuth, impact point, or
accuracy.

```toml
[staging_analysis]
mode               = "optimal"  # budget | optimal
delta_v_budget_m_s = 9400.0     # required for optimal, rejected for budget
payload_mass_kg    = 250.0

[[staging_analysis.stages]]     # bottom-up
isp_s                  = 280.0
structural_coefficient = 0.08

[[staging_analysis.stages]]
isp_s                  = 320.0
structural_coefficient = 0.10
```

`mode = "budget"` omits `delta_v_budget_m_s` and requires each stage to also
declare `structural_mass_kg` and `propellant_mass_kg`. `mode = "optimal"`
rejects per-stage masses and computes the mass-optimal split for the declared
ideal ΔV. `structural_coefficient` must lie in `(0, 1)`, and all masses and
`isp_s` values must be finite and positive.

### Recovery and descent

`[[vehicle.assembly.recovery]]` declares recovery devices driven by the
runner-side recovery rack. Recovery is force-only: the
kernel applies drag at the body center of gravity, with no recovery
mass contribution and no recovery moment.

```toml
[[vehicle.assembly.recovery]]
id         = "dual_chute"
mounted_to = "lower"
kind       = { kind = "drogue_main", drogue_c_d = 1.0, drogue_area_m2 = 0.5, main_c_d = 1.5, main_area_m2 = 4.0 }

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
When `[multi_body]` is declared, every recovery device must set
`mounted_to` to a declared body. After separation, only that body's
lane receives the recovery drag.

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

#### Limitations

- Drag opposes ECI velocity, matching the existing axial-drag adapter.
  Wind-relative parachute drag is not modelled.
- Canopy inflation transients are not modelled; recovery
  state changes are instantaneous at the rack tick.
- Recovery devices do not contribute mass or moments.

The canonical example ships at
[`scenarios/parachute-recovery/parachute-descent.toml`](../scenarios/parachute-recovery/parachute-descent.toml).

## Advanced-Profile Extensions

Schema version **v3** adds opt-in scenario blocks for advanced
controller and propagation profiles. The parser accepts v2 and v3
headers; v2 scenarios continue to parse byte-identically and require no
change. Each new block parses with `serde(deny_unknown_fields)`.

A v3 scenario header looks like this:

```toml
openbmp.scenario = 3
```

A v2 scenario that declares any v3-only block fails closed at validate
time with a `SchemaVersionFieldReserved` diagnostic naming the field
and the required version. A v3 scenario that declares a block whose
runtime consumer is not present fails
closed with an `ElementNotYetSupported` diagnostic. This avoids silent
no-ops and keeps the schema honest about what runtime
behaviour it can deliver.

### v2 → v3 migration

- Bump `openbmp.scenario = 2` to `openbmp.scenario = 3`.
- Existing fields parse identically. Existing scenarios are
  byte-stable across the bump.
- New v3 blocks below are opt-in. Under v2, declaring them fails closed
  with `SchemaVersionFieldReserved` instead of being ignored.

### v3-only top-level blocks

#### `[schedule]` — multi-rate scheduling

```toml
[schedule]
base_hz = 1000

[[schedule.group]]
label   = "env"
hz      = 100
members = ["atmosphere", "gravity", "wind"]

[[schedule.group]]
label   = "fc"
hz      = 50
members = ["estimator", "autopilot", "fdir", "mission_fsm"]
```

Field-name discipline: every group's `hz` must divide `base_hz`
exactly (`base_hz % hz == 0`). The loader rejects non-integer divisors
at scenario load time, not at runtime. Groups must have unique labels
and non-empty member lists. The `[schedule]` block bypasses the
workspace-level unit-suffix lint because `ScheduleConfig::validate`
covers the field-internal invariants. The runtime consumer
resolves the rate plan once at scenario start and the
kernel walks the same fixed list every tick.

#### `[multi_body]` — multi-body simultaneous propagation

```toml
[multi_body]

[[multi_body.separation]]
event_id              = "fairing_separation"
upper_body_id         = "main"
lower_body_id         = "lower_stage"
upper_delta_v_body_m_s = [0.0, 0.0, 0.5]
lower_delta_v_body_m_s = [0.0, 0.0, -0.5]
conserve_momentum     = true
```

Each `[[multi_body.separation]]` entry binds to a mission event by id
and declares the two `vehicle.assembly.bodies[*].id` values that
continue propagating after the event. Optional impulsive delta-V
fields apply at the separation moment. `conserve_momentum` defaults
to `true`; the loader verifies
`m_u·Δv_u + m_l·Δv_l ≈ 0` to a documented tolerance.

The runtime consumer supports fixed-step RK4 rigid-body profiles.
Post-separation force-stack ownership is explicit:

- `[aero].mounted_to` is required when `"aero"` is in
  `forces.models`.
- `[propulsion.motor].mounted_to` is required when a motor is
  declared.
- `vehicle.assembly.engines[*].mounted_to` is required when engines
  are declared.
- `vehicle.assembly.effectors[*].mounted_to` is required when
  effectors are declared.
- `vehicle.assembly.recovery[*].mounted_to` is required when recovery
  devices are declared.
- `vehicle.assembly.tanks[*].mounted_to` is always required.

After separation, each lane evaluates only the aero, thrust, engine,
tank, recovery, effector, snapshot, and mass-property resources owned
by that lane's active body. Ambiguous ownership fails at scenario load
or at the model capability gate; the composite force stack is not
silently reused for detached bodies. Telemetry includes the continuing
primary lane in the existing state channels and each departing body in
`body.<lower_body_id>.*` channels, including
`body.<lower_body_id>.separated`. Relative range triggers can observe
these lanes with `trigger.kind = "at_relative_distance"` after the
separation has occurred.

### v3-only `[fc]` sub-blocks

#### `[fc.estimator_lanes]` — multi-instance estimator routing

```toml
[fc.estimator_lanes]
voter = "best_by_covariance_trace"

[[fc.estimator_lanes.lane]]
id        = "primary"
estimator = "ekf"

[[fc.estimator_lanes.lane]]
id        = "spare"
estimator = "mekf"
```

`voter` is one of `simplex_pass_through`, `mid_value_select_by_innovation`,
or `best_by_covariance_trace`. Lane ids must be unique. The
consumer wires parallel filter instances to the controller's pub/sub
bus and selects the active lane each tick.

#### `[fc.autopilot_allocation]` — control allocation

```toml
[fc.autopilot_allocation]
kind          = "prioritised_redistributed"
axis_priority = ["roll", "yaw", "pitch"]
```

`kind` is one of `pseudo_inverse` (Stevens & Lewis 2015 §3.5) or
`prioritised_redistributed` (Härkegård 2002). Only
`prioritised_redistributed` is consumed; `pseudo_inverse` remains parseable but the
runner fails closed because the general `G_eff` path is not wired. `axis_priority`
lists body-frame axes in highest-first order and must contain exactly
`"roll"`, `"pitch"`, and `"yaw"` once each.

The allocator derives capacity only from `direct_torque`
effectors with exact symmetric limits (`max == -min`). Phase authority is
applied before the proportional split, so disallowed effectors receive
zero commands and do not contribute capacity.

#### `[fc.fdir.detector]` — FDIR detector tuning

```toml
[fc.fdir.detector]
kind             = "windowed_glrt"
window_samples   = 32
parity_threshold = 25.0
```

The `detector_kind` field on `[fc.fdir]` drives
the burst / single-sample-GLRT / CUSUM detectors; this
sub-block adds tuning data for the windowed-mean-shift GLRT
(Willsky 1976) and Patton-Frank parity-space residual generator.

#### `[fc.trajectory]` — minimum-snap trajectory waypoints

```toml
[fc.autopilot_params]
trajectory_loop_enabled = true
trajectory_kind         = "minimum_snap"

[fc.trajectory]
kind    = "minimum_snap"
yaw_rad = 0.0

[[fc.trajectory.waypoint]]
position_eci_m = [0.0, 0.0, 100.0]
time_s         = 0.0

[[fc.trajectory.waypoint]]
position_eci_m = [10.0, 5.0, 100.0]
time_s         = 2.0
```

`[fc.trajectory]` is v3-only and consumed by the
Mellinger-Kumar minimum-snap differential-flatness tracker. It parses
with `serde(deny_unknown_fields)`. `kind = "minimum_snap"` is the only
supported value. `yaw_rad` is optional and defaults to
`0.0`; time-varying yaw splines are not supported. The waypoint list is
declared as `[[fc.trajectory.waypoint]]` entries, each with finite
`position_eci_m = [x, y, z]` and finite, strictly increasing `time_s`.
At least two waypoints are required. Adjacent waypoint times must be
between `0.001 s` and `600 s`, inclusive, to keep the deterministic KKT
solve inside its documented conditioning envelope.

The block is cross-validated with
`fc.autopilot_params.trajectory_kind`: selecting `"minimum_snap"`
requires `[fc.trajectory]`, and declaring `[fc.trajectory]` requires
`trajectory_kind = "minimum_snap"`.

#### `[fc.autopilot_params.rate_loop_kind]` and INDI

```toml
[fc.autopilot_params]
rate_loop_kind = "indi" # "pid" | "lqr" | "indi"; field is v3-only

[fc.autopilot_params.indi]
inertia_per_axis_kg_m2         = [1.0, 1.0, 1.0]
control_effectiveness_per_axis = [1.0, 1.0, 1.0]
filter_cutoff_rad_s            = 50.0
filter_kind                    = "second_order_butterworth" # default; or "first_order_low_pass"
attitude_to_omega_dot_gain     = [10.0, 10.0, 5.0]
```

`rate_loop_kind` is a v3-only field. Omitted
`rate_loop_kind` keeps the PID rate loop; declaring
`rate_loop_kind = "lqr"` requires `[fc.autopilot_params.lqr]`, and
declaring `rate_loop_kind = "indi"` requires
`[fc.autopilot_params.indi]`. The parser also rejects either
parameter block when the matching `rate_loop_kind` value is absent.

INDI is gated by the `openbmp-cli/indi` Cargo feature. The runner
fails closed unless the scenario assembly contains exactly one body
with diagonal `dry_inertia_body_kg_m2`; the per-axis increment model
does not claim coupled multi-body or non-diagonal-inertia support.
The INDI block declares the controller's working inertia estimate,
per-axis control effectiveness, one synchronized low-pass cutoff and
filter kind shared by both the measured-rate and prior-command
filters, and the outer-loop attitude-to-angular-acceleration P gain.
`filter_cutoff_rad_s` must be positive and strictly below
`π / time.dt_s`. Combining `rate_loop_kind = "indi"` with
`[fc.autopilot_params.l1_adaptive]` is rejected at scenario load;
that composition is deferred until the filter-interaction behaviour
is characterised.

#### `[fc.autopilot_params.attitude_loop_kind]` and MPC

```toml
[fc.autopilot_params]
attitude_loop_kind = "mpc" # "pid" | "mpc"; field is v3-only

[fc.autopilot_params.attitude_mpc]
horizon_n        = 20
q_x              = [100.0, 100.0, 50.0]
r_u              = [0.1, 0.1, 0.1]
terminal_p       = [1000.0, 1000.0, 500.0]
rate_limit_rad_s = [3.0, 3.0, 3.0]
```

`attitude_loop_kind` is a v3-only field. Omitted
`attitude_loop_kind` keeps the PID attitude loop. Declaring
`attitude_loop_kind = "mpc"` requires
`[fc.autopilot_params.attitude_mpc]`, and declaring the MPC parameter
block without the `"mpc"` selector is rejected. The MPC path is gated by
the `openbmp-cli/mpc` Cargo feature.

The MPC is a command-level attitude-error controller. Its QP
uses the small-angle dynamics `x[k+1] = x[k] - dt*u[k]`, where `u` is the
commanded body rate. It does not model downstream PID/LQR/INDI rate-loop
lag, actuator saturation, or future reference-attitude motion across the
horizon. `attitude_loop_kind = "mpc"` is otherwise independent of
`rate_loop_kind`; existing rate-loop composition rules still apply, such
as the `indi` + `l1_adaptive` rejection above.

### v3-only kind values

#### `gravity = "egm2008"`

```toml
[environment]
gravity = "egm2008"
```

Selects the EGM2008 **zonal-only** gravity model,
`openbmp_physics::Egm2008ZonalGravity`, truncated to degrees 2 through
6. The model pins WGS84 `µ`, WGS84 `R_e`, and the public `J_2..J_6`
zonal coefficients in source; there are no per-scenario `degree`,
`order`, or `coefficients_path` overrides in the shipped surface. v2
scenarios that name `egm2008` fail closed with a schema-version
diagnostic.

Tesseral / sectoral terms, Cunningham recursion, full coefficient-file
loading, and scenario-selectable degree/order are deferred to a future
gravity slice.

#### `atmosphere = "piecewise_exponential"`

```toml
[environment]
atmosphere = "piecewise_exponential"

[atmosphere]
kind = "piecewise_exponential"
```

Selects the layered exponential atmosphere,
`openbmp_physics::PiecewiseExponentialAtmosphere`, with the fixed
14-layer Vallado Table 8-4 density / scale-height fit covering
0-1000 km. The structured `[atmosphere]` block is optional when the
legacy `environment.atmosphere` selector names the same model; if both
are present they must agree. v2 scenarios that name
`piecewise_exponential` fail closed with a schema-version diagnostic.
The model reports a scale-height-effective `temperature_k` for
ideal-gas self-consistency; it is not a source-tabulated thermospheric
temperature product.

#### `atmosphere = "nrlmsise00"`

```toml
[environment]
atmosphere = "nrlmsise00"

[atmosphere]
kind = "nrlmsise00"
year = 2024
day_of_year = 80
utc_s = 43200.0
latitude_deg = 0.0
longitude_deg = 0.0
local_apparent_solar_time_h = 12.0
f107_average_81day_sfu = 150.0
f107_yesterday_sfu = 150.0
ap_average = 4.0
```

Selects the NRLMSISE-00 empirical atmosphere model, supplied by the
hypersonic extensions. It is a v3-only atmosphere kind and is wired to
the in-repository coefficient evaluator for the 0-1000 km model
envelope.

The structured `[atmosphere]` fields are optional; omitted values use
the static mid-condition defaults shown above. If the legacy
`environment.atmosphere` selector and the structured block are both
present, both must name `nrlmsise00`. The runner stores these scalar
environment inputs as deterministic scenario parameters and varies only
altitude through the existing atmosphere sampling trait.

#### `atmosphere = "nrlmsis2_compat"`

```toml
[environment]
atmosphere = "nrlmsis2_compat"

[atmosphere]
kind = "nrlmsis2_compat"
year = 2024
day_of_year = 80
utc_s = 43200.0
latitude_deg = 0.0
longitude_deg = 0.0
local_apparent_solar_time_h = 12.0
f107_average_81day_sfu = 150.0
f107_yesterday_sfu = 150.0
ap_average = 4.0
```

Selects OpenBMP's NRLMSIS 2.x compatibility atmosphere profile. It is
a v3-only atmosphere kind wired to the in-repository NRLMSISE-00
coefficient evaluator, with a bounded upper-atmosphere correction and a
nitric-oxide number-density proxy. It does not vendor or claim to be
the official NRLMSIS 2.0 / 2.1 coefficient package.

The structured `[atmosphere]` fields use the same MSIS-family inputs as
`nrlmsise00`. If the legacy `environment.atmosphere` selector and the
structured block are both present, both must name `nrlmsis2_compat`.

### Hard guardrails

- v3 introduces no field that advances proportional navigation,
  terminal homing, real-world targeting, real device drivers, or
  real bus protocols. Every consumer honours
  [docs/safety-boundaries.md](safety-boundaries.md).
- The determinism CI gate continues to assert byte-identical Parquet
  for the existing v2 scenario set across this schema bump.
