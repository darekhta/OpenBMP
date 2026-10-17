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
| `[contact]` | no | Schema-v3 opt-in compliant half-space contact force |
| `[telemetry]` | yes | Output files and schema options |
| `[comm]` | no | Schema-v3 ground-site geometry for telemetry/RF visibility studies |
| `[acoustics]` | no | Schema-v3 liftoff, IOP, ascent/fairing, and response analysis reports with gated summary telemetry |
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
| `[monte_carlo]` | no | Schema-v3 synthetic rare-event Monte Carlo manifest |
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
high_dynamics_q_scale      = 4.0 # optional; > 0, default 1.0

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

[fc.scheduler]
guidance_rate_hz   = 100
health_rate_hz     = 100
fdir_rate_hz       = 100
guidance_budget_us = 100

[fc.fdir]
detector_kind          = "burst_counter" # "burst_counter" | "single_sample_glrt" | "cusum"
innovation_threshold   = 25.0
innovation_burst_count = 5
failsafe_burst_count   = 5

[fc.fdir.redlines]
body_rate_rad_s = 4.0

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
`high_dynamics_q_scale` is optional and defaults to `1.0`; when the
commander publishes `mission.regions.estimator_regime.boost_mode`, the
EKF multiplies position/velocity process-noise covariance by this
scale. Use `set_region_state` actions in the mission HSM to declare
boost/coast regime changes.

`[fc.scheduler]` is optional. Missing fields preserve the reference
topology: estimator, commander, autopilot, and mixer run at
`base_rate_hz`; guidance, health, and FDIR run at 100 Hz. Declared
rates must be in `1..=base_rate_hz`, and declared budgets must be
positive microsecond values. The scheduler converts rates to integer
base-rate periods with ceil division, so a requested 50 Hz task under
`base_rate_hz = 1000` runs every 20 ticks. `instrument_timing = true`
enables host-side per-job wall-clock timing in the runner; measured
microseconds are reported through `scheduler.timing_budget_report` and
deadline slips through `scheduler.deadline_slip`. The wall-clock call
remains outside `openbmp-fc`.

For flight builds, the validated declarative tables are loaded as a
compact I-load payload, not as TOML. `openbmp-hal` defines the `OBIL`
envelope around that payload: envelope version, payload schema
version, payload length, and CRC-32. Flight loaders must reject a
stale schema or CRC mismatch before deserializing the payload, and may
select a separately stored fallback I-load if the primary image fails
validation.

Schema v3 also supports powered-ascent reference generation:

```toml
[fc]
guidance = "ascent_reference"

[fc.ascent_reference]
method     = "pitch_program" # or "gravity_turn"
schedule_s = [0.0, 10.0, 30.0]
pitch_rad  = [0.0, 0.05, 0.20]
yaw_rad    = [0.0, 0.00, 0.01] # optional for pitch_program

[fc.gain_schedule."mission.phases.powered_ascent"]
# normal three-loop gains for this phase
```

`guidance = "ascent_reference"` requires `vehicle.kind = "rigid_body"`, a
declared `powered_ascent` mission phase/state, and a matching gain-schedule
entry. `pitch_program` requires equal-length `schedule_s` / `pitch_rad` arrays
with strictly increasing times; optional `yaw_rad` must use the same length.
`gravity_turn` accepts no schedule fields and aligns body `+x` with inertial
velocity once motion is established.
`explicit_reference` is still reserved and fails closed.

Schema v3 can also replay an optimized offline reference profile through the FC
guidance boundary:

```toml
[fc]
guidance = "optimized_reference"

[fc.optimized_reference]
active_phase_paths = ["mission.phases.powered_ascent"]
radial_direction_eci = [1.0, 0.0, 0.0]
tangential_direction_eci = [0.0, 1.0, 0.0]
reference_q_xyzw = [0.0, 0.0, 0.0, 1.0]

[[fc.optimized_reference.samples]]
time_s = 0.0
radius_m = 6_371_000.0
speed_m_s = 0.0
flight_path_angle_rad = 0.0

[[fc.optimized_reference.samples]]
time_s = 1.0
radius_m = 6_371_100.0
speed_m_s = 100.0
flight_path_angle_rad = 0.05
```

The block requires at least two strictly increasing samples with positive
radius, non-negative speed, and `|flight_path_angle_rad| < pi/2`. Instead of
inline samples, the same block may name a postcard-encoded trajopt I-load file:

```toml
[fc.optimized_reference]
iload_file = "optimized-reference.iload"
iload_file_sha256 = "<optional sha256 hex pin>"
active_phase_paths = ["mission.phases.powered_ascent"]
```

Inline `samples` and `iload_file` are mutually exclusive. The runner maps either
source into the FC-native optimized-reference guidance job; the FC crate still
does not depend on `openbmp-trajopt`.

Schema v3 can also declare a provenance-pinned imported robust-controller
I-load source:

```toml
[fc.robust_controller]
iload_file = "robust-controller.toml"
iload_file_sha256 = "<optional sha256 hex pin>"
```

The scenario loader resolves `iload_file` relative to the scenario directory and
verifies `iload_file_sha256` when present. The runner materializes the resolved
file into an active FC robust-controller table at boot. The I-load file uses a
single `[robust_controller]` envelope with sorted nodes:

```toml
[robust_controller]
controller_id = "robust-local-v1"
source_id = "hinf-reduced-order-local"
source_sha256_hex = "<source artifact sha256>"
variable = "dynamic_pressure_pa"
interp = "linear"
states = 1
measurements = 1
controls = 1

[[robust_controller.nodes]]
node_id = "qbar-low"
schedule_value = 10000.0
flight_condition_tag = "mach_qbar_mass"
states = 1
measurements = 1
controls = 1
a_row_major = [-0.20]
b_row_major = [1.0]
c_row_major = [0.8]
d_row_major = [0.0]

[[robust_controller.nodes]]
node_id = "qbar-high"
schedule_value = 25000.0
flight_condition_tag = "mach_qbar_mass"
states = 1
measurements = 1
controls = 1
a_row_major = [-0.35]
b_row_major = [1.2]
c_row_major = [0.7]
d_row_major = [0.05]
```

This loads and validates the table; closed-loop measurement mapping and command
mixing are separate GNC integration work.

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
initial_position_eci_m = [0.0, 0.0, 1000.0]
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
For `environment.gravity = "constant"` scenarios, runner simulations
also install an automatic sea-level ground-impact stop condition.

### Realtime Pacing

`[realtime]` is optional and v3-only. When absent, the runner does not read the
wall clock. When present, the runner paces each kernel frame through
`openbmp-rt` while keeping timing observations outside telemetry:

```toml
[realtime]
mode = "paced"          # "free_run" | "real_time" | "paced"
target_rtf = 1000.0     # required only for mode = "paced"
jitter_budget_s = 0.001

[[realtime.task]]
label = "estimator"
period_s = 0.01
wcet_s = 0.001
```

The minor-frame period is always `[time].dt_s`. `mode = "free_run"` records
jitter without sleeping; `mode = "real_time"` targets one simulation second per
wall-clock second; `mode = "paced"` targets `target_rtf` simulation seconds per
wall-clock second. Realtime jitter, overruns, host frame-execution percentiles,
and frame-budget overrun counts are reported by the runner and CLI, not written
into canonical telemetry. Optional `[[realtime.task]]` entries declare periodic
task budgets for the runner's Liu-Layland rate-monotonic schedulability
advisory; the advisory is reported outside canonical telemetry with the timing
record.

### Communications Geometry

`[comm]` is optional and schema-v3. It declares ground-site geometry for the
WP-20.1 communications visibility/pass-table substrate. The parser validates
site ids, geodetic degrees, minimum elevation, and optional azimuth-binned
terrain masks. Runner paths sample primary-body truth states through the
selected frame profile and expose deterministic pass-table CSV bytes through
`RunOutcome.comm`. They also publish per-site `comm.<site>.slant_range_m`,
`.range_rate_m_s`, `.elevation_rad`, `.azimuth_rad`, `.mask_elevation_rad`,
and `.visible` telemetry channels. `[[comm.antennas]]` may declare
WP-20.2 gain/body-mask deck files with optional SHA-256 pins; these decks
are resolved, hashed, parsed from resolved bytes, validated by the runner,
summarized in `RunOutcome.comm`, and applied to link budgets by rotating the
vehicle-to-site line of sight into the body frame. Body-mask decks may carry
comm-native `[body_mask].triangles`, or may use the existing
`openbmp.panel_mesh_aero = 1` panel-mesh deck shape with a `[body_mask]`
antenna position and sampled directions; the runner derives the mask by direct
ray casts over a deterministic watertight-mesh BVH and rejects open or
non-manifold panel meshes fail-closed. `[[comm.links]]` declares WP-20.3 free-space
link-budget constants plus pinned FER, atmospheric-loss, rain-loss, and
optional electron-density plasma-blackout sources; runner paths resolve the
decks or consume the declared live aerothermal proxy, compute one `LinkState`
per declared link/site sample, and publish
`comm.link.<link>.free_space_loss_db`, `.c_n0_dbhz`, `.eb_n0_db`,
`.margin_db`, `.fer`, `.visible`, `.blackout`, `.antenna_gain_dbi`, and
`.body_blocked` telemetry channels. Links may also declare a packet-effect
processing delay, errored-frame action, and optional packet loss-rate gate; the
runner validates and reports that model and fails closed if a declared loss-rate
gate falls outside its exact binomial interval. Links may additionally declare a
downlink `data_loss_timeout_s`; the runner latches and reports the first
continuous data-loss gap exceeding that timeout. When `[fc.transport]` is
active, the runner applies the selected comm link's packet model to FC bridge
sensor and command frames in fail-closed mode for drop or bit-flip effects and
queues delivered frames by step-quantized link latency. `[comm]` can select the
bridge link by declaration order or by current best visible margin, with an
optional dB hysteresis deadband for best-margin handover, or by declared
pass-plan windows. `[comm.tracking]` can opt into vehicle-bound ground
tracking observables for reconstruction; the runner emits visible-site
range/range-rate/azimuth/elevation measurements in
`RunOutcome.comm.tracking_observables` with declared latency, bias, and
quantization plus optional deterministic white noise and first-order
Gauss-Markov bias states from a pinned instrument deck, exports the same
measurement stream as deterministic `RunOutcome.comm.tracking_observables_csv`
bytes, and reports the instrument evidence in `RunOutcome.comm.tracking`.

```toml
[comm]
bridge_link_selection = "first_declared" # optional; "best_margin" or "declared_plan"
bridge_link_hysteresis_db = 0.0          # optional, best_margin deadband

[comm.tracking]                         # optional
instrument_deck = "data/comm/tracking-instrument-v1.toml" # optional
instrument_deck_sha256 = "<64 hex chars>"                  # optional
latency_s = 0.25
range_bias_m = 2.0
range_rate_bias_m_s = -0.1
azimuth_bias_deg = 0.05
elevation_bias_deg = -0.02
range_quantization_m = 1.0
range_rate_quantization_m_s = 0.1
angle_quantization_deg = 0.01

[[comm.sites]]
id = "ksc"
latitude_deg = 28.572872
longitude_deg = -80.648981
altitude_m = 3.0
min_elevation_deg = 5.0
terrain_mask = [
  { azimuth_deg = 0.0, min_elevation_deg = 6.0 },
  { azimuth_deg = 180.0, min_elevation_deg = 8.0 },
]

[[comm.antennas]]
id = "s-band-low-gain"
gain_deck = "data/comm/antenna-body-mask-v1.toml"
gain_deck_sha256 = "<64 hex chars>"       # optional
body_mask_deck = "data/comm/body-mask.toml"
body_mask_deck_sha256 = "<64 hex chars>"  # optional

[[comm.links]]
id = "s-band-ksc"
site_id = "ksc"
antenna_id = "s-band-low-gain"
eirp_dbw = -40.0
receiver_g_over_t_db_k = 0.0
frequency_hz = 2.0e9
bit_rate_bps = 1000.0
required_eb_n0_db = 3.0
atmospheric_loss_db = 1.0
atmospheric_loss_deck = "data/comm/attenuation-loss-v1.toml"
atmospheric_loss_deck_sha256 = "<64 hex chars>"  # optional
rain_loss_db = 0.5
rain_loss_deck = "data/comm/attenuation-loss-v1.toml"
rain_loss_deck_sha256 = "<64 hex chars>"         # optional
blackout_electron_density_source = "deck"        # optional: deck or live_aerothermal
blackout_electron_density_deck = "data/comm/plasma-blackout-v1.toml" # optional
blackout_electron_density_deck_sha256 = "<64 hex chars>"             # optional
blackout_ramp_width_hz = 1.0e8        # optional, default 0.0
blackout_max_attenuation_db = 40.0    # optional, default 60.0
pointing_loss_db = 0.25
polarization_loss_db = 0.1
implementation_loss_db = 0.0
fer_curve_deck = "data/comm/link-budget-fer-v1.toml"
fer_curve_deck_sha256 = "<64 hex chars>"  # optional
packet_processing_delay_s = 0.02          # optional, default 0.0
packet_error_action = { kind = "drop" }   # optional; or { kind = "bit_flip", mask = 165 }
packet_loss_rate_gate = { packet_count = 2048, alpha = 0.001 } # optional
data_loss_timeout_s = 1.5                 # optional downlink continuity latch

[[comm.bridge_pass_plan]]
link_id = "s-band-ksc"
start_s = 0.0
end_s = 120.0
```

Tracking instrument decks use a `[tracking_instrument]` table:

```toml
[tracking_instrument]
range_noise_stddev_m = 0.5
range_rate_noise_stddev_m_s = 0.02
azimuth_noise_stddev_deg = 0.001
elevation_noise_stddev_deg = 0.0015
range_gauss_markov_stddev_m = 2.0
range_gauss_markov_tau_s = 10.0
range_rate_gauss_markov_stddev_m_s = 0.2
range_rate_gauss_markov_tau_s = 20.0
azimuth_gauss_markov_stddev_deg = 0.002
azimuth_gauss_markov_tau_s = 30.0
elevation_gauss_markov_stddev_deg = 0.003
elevation_gauss_markov_tau_s = 40.0
```

`latitude_deg` must be in `[-90, 90]`, `longitude_deg` in `[-180, 180]`,
and elevation fields in `[-90, 90]`. Terrain-mask `azimuth_deg` values are
accepted in `[0, 360]` and are interpreted as degrees from north toward east.
Link `site_id` and `antenna_id` must reference declared comm site and antenna
ids. Frequency and bit rate must be positive; loss fields must be finite and
non-negative. Loss decks are optional and additive to the constant loss
fields. If a `*_loss_deck_sha256` pin is present, the matching deck path must
also be present. `blackout_electron_density_source` defaults to `deck`. In
deck mode, if `blackout_electron_density_deck` is present, the runner expects
a TOML `[plasma_blackout]` table with strictly increasing
`[[plasma_blackout.samples]]` entries containing finite non-negative `time_s`
and `electron_number_density_m3`. The density profile is linearly interpolated
inside the deck time range and clamped outside it. In `live_aerothermal` mode,
the link must not declare a deck and the scenario must declare
`[aerothermal.electron_density_proxy]`; the runner uses the current live
`aerothermal.electron_number_density_m3` sample. In both modes attenuation is
added to the vehicle-originating hop using `f_p = 8.98 sqrt(n_e)`, with
`blackout_max_attenuation_db` applied at/below plasma frequency and a linear
ramp over `blackout_ramp_width_hz` above it. `blackout_*_sha256` pins require
the matching deck path, and blackout ramp/attenuation values must be
non-negative. `packet_processing_delay_s` must be finite and non-negative.
`bridge_link_selection` defaults to `first_declared`; `best_margin` selects the
visible, non-blackout declared link with the highest current margin and falls
back to the first declared link when no link is usable so the bridge still fails
closed. `declared_plan` selects the active `[[comm.bridge_pass_plan]]` window by
latest observation time; plan gaps keep comm effects active and drop bridge
packets fail-closed. `bridge_link_hysteresis_db` must be finite and
non-negative; when it is positive and `best_margin` is active, the previously
selected visible non-blackout link is retained until the best candidate exceeds
its margin by more than the declared deadband. Each pass-plan `link_id` must
reference a declared link, `start_s` must be finite and non-negative, `end_s`
must be greater than `start_s`, and windows must not overlap.
`bridge_link_selection = "declared_plan"` requires at least one pass-plan
window. `[comm.tracking]` is optional; when present, `latency_s`,
`range_quantization_m`, `range_rate_quantization_m_s`, and
`angle_quantization_deg` must be finite and non-negative, while bias fields
must be finite. If `instrument_deck` is present, the runner expects a TOML
`[tracking_instrument]` table with finite non-negative white-noise standard
deviations plus finite non-negative first-order Gauss-Markov stationary
standard deviations and correlation times for range, range-rate, azimuth, and
elevation. A non-zero Gauss-Markov standard deviation requires a positive
`*_tau_s`; zero standard deviation disables that component. The matching
`instrument_deck_sha256` pin requires the deck path. Tracking observables are
emitted only for visible ground-site samples and are vehicle-bound because
their inputs come from the runner's primary-body site observations; each record
includes the applied white-noise values and the current Gauss-Markov bias
state. The CSV measurement stream uses a fixed header, RFC-4180 escaping for
site ids, and `"{:.17e}"` formatting for floating-point fields.
`[[comm.relays]]` declares GEO relay nodes with `id`, `longitude_deg`,
optional `altitude_m`, optional `min_elevation_deg`, relay-to-ground budget
constants, and a relay downlink `fer_curve_deck`; optional
`[[comm.relays.availability_windows]]` entries with `start_s`/`end_s` gate
relay availability by time and must not overlap. `[[comm.links]].relay_id`
references one of those relay ids and makes the runner compose the
vehicle-to-relay and relay-to-ground budgets into one two-hop `LinkState` using
bottleneck margin and combined independent-hop FER. The
`packet_error_action.kind` value `"bit_flip"` requires a non-zero `mask`;
omitted packet settings default to zero processing delay and `drop` on errored
FER draws. `packet_loss_rate_gate.packet_count` must be in `[1, 1000000]`, and
`packet_loss_rate_gate.alpha` must be in `(0, 1)`. When present, the runner
evaluates both downlink and uplink deterministic packet streams against the
latest computed `LinkState` sample and records the gate evidence in
`RunOutcome.comm.links[*].packet_loss_rate_gate`. `data_loss_timeout_s` must be
positive when present; the runner records delivered/lost downlink sample counts,
the first loss, first timeout trigger, maximum continuous loss gap, and a
monitor-side `hold`/`terminate` verdict in
`RunOutcome.comm.links[*].data_loss_timeout`. Contiguous sampled blackout
windows are summarized in `RunOutcome.comm.blackout_windows` with link id,
step/time span, sample count, and visible-sample count. Bridge packet-effect selections
are summarized in `RunOutcome.comm.bridge_selections` as contiguous link or
scheduled-gap intervals with step/time spans, sample counts, and sensor/command
drop or bit-flip counts. Link/gap transitions are summarized in
`RunOutcome.comm.bridge_handovers`. Visible/non-blackout link passes are
summarized in `RunOutcome.comm.link_passes` with sampled step/time spans,
sampled and interpolated durations, sample count, min/max/mean margin, sampled
and interpolated margin profiles, and bridge packet drop or bit-flip counts
attributed to each pass. Relay declarations consumed by a link, including
declared availability windows, are summarized in `RunOutcome.comm.links[*].relay`.

### AFTS Containment Rules

`[afts]` is optional and v3-only. It declares forward containment-monitor rule
tables for `openbmp-afts`. The parser validates the table, and the runner
samples telemetry-aligned truth states into an optional `RunOutcome.afts`
sidecar report. This observer does not alter the simulator's kernel
`StopReason`.

```toml
[afts]
enabled = true

[afts.propagator]
surface = "wgs84_ellipsoid" # "wgs84_sphere" | "wgs84_ellipsoid"; default "wgs84_sphere"
earth_rotation = "wgs84_uniform" # "frame_profile" | "disabled" | "wgs84_uniform"; default "frame_profile"
step_s = 0.25               # optional positive RK4 step override
max_time_s = 7200.0         # optional positive forward horizon override
radius_tolerance_m = 0.001  # optional non-negative impact-surface tolerance

[afts.propagator.drag]
reference_density_kg_m3 = 0.01
scale_height_m = 8500.0
ballistic_coefficient_kg_m2 = 250.0

[[afts.keep_inside]]
id = "range_box"
evidence = "data/afts/range-box/provenance.md"
evidence_file = "data/afts/range-box/provenance.md"
evidence_file_sha256 = "<64 hex chars>"
vertices = [
  { latitude_deg = -1.0, longitude_deg = -1.0 },
  { latitude_deg = -1.0, longitude_deg =  1.0 },
  { latitude_deg =  1.0, longitude_deg =  1.0 },
  { latitude_deg =  1.0, longitude_deg = -1.0 },
]

[[afts.keep_out]]
id = "hazard_zone"
evidence = "data/afts/hazard-zone/provenance.md"
vertices = [
  { latitude_deg = -0.2, longitude_deg = -0.2 },
  { latitude_deg = -0.2, longitude_deg =  0.2 },
  { latitude_deg =  0.2, longitude_deg =  0.2 },
  { latitude_deg =  0.2, longitude_deg = -0.2 },
]

[[afts.corridor]]
id = "entry_speed_band"
evidence = "data/afts/entry-speed-band/provenance.md"
metric = "speed_m_s"     # "altitude_m" | "speed_m_s" | "flight_path_angle_rad"
min_speed_m_s = 0.0      # optional inclusive lower bound for speed_m_s
max_speed_m_s = 2200.0   # optional inclusive upper bound for speed_m_s

[[afts.gate]]
id = "iip_zero_longitude_gate"
evidence = "data/afts/iip-gate/provenance.md"
start = { latitude_deg = -1.0, longitude_deg = 0.0 }
end = { latitude_deg = 1.0, longitude_deg = 0.0 }
direction = "any"        # "any" | "negative_to_positive" | "positive_to_negative"

[[afts.zone]]
id = "population_red_zone"
evidence = "data/afts/population-red-zone/provenance.md"
color = "red"            # "green" | "red"
vertices = [
  { latitude_deg = -0.5, longitude_deg = -0.5 },
  { latitude_deg = -0.5, longitude_deg =  0.5 },
  { latitude_deg =  0.5, longitude_deg =  0.5 },
  { latitude_deg =  0.5, longitude_deg = -0.5 },
]
```

`[afts.propagator]` is optional. When omitted, the monitor preserves the
original WGS84 semi-major-axis spherical surface and uses `environment.frame_profile`
to decide whether to apply WGS84 uniform-rotation longitude correction. `surface =
"wgs84_ellipsoid"` switches IIP surface detection and reported geodetic latitude
to the WGS84 oblate ellipsoid. `[afts.propagator.drag]` is optional and adds a
deterministic exponential ballistic-drag perturbation to the forward IIP
integration; omit the table for vacuum central gravity.

When `enabled = true`, at least one `[[afts.keep_inside]]` or
`[[afts.keep_out]]` or `[[afts.corridor]]` or `[[afts.gate]]` or
`[[afts.zone]]` rule is required. Keep-inside rules fire when the predicted IIP
exits the polygon; keep-out rules fire when the predicted IIP enters the
polygon; corridor rules fire when the sampled truth state leaves an inclusive
scalar bound; gate rules fire when successive predicted IIP samples cross the
declared gate segment. Red zones fire when the predicted IIP enters the
polygon; green zones form a safe-zone union, so when any green zones are
declared the IIP must stay inside at least one green zone. Corridor
`metric` is one of `"altitude_m"`, `"speed_m_s"`, or
`"flight_path_angle_rad"` and must declare at least one matching unit-suffixed
bound: `min_altitude_m`/`max_altitude_m`, `min_speed_m_s`/`max_speed_m_s`, or
`min_flight_path_angle_rad`/`max_flight_path_angle_rad`. Gate `direction`
defaults to `"any"` and can restrict crossings to `"negative_to_positive"` or
`"positive_to_negative"` relative to the directed `start` -> `end` segment.
Rule ids and evidence strings must be nonempty, rule ids must be unique across
all AFTS rule lists, each polygon must have at least three vertices, latitudes
must be in `[-90, 90]` degrees, longitudes in `[-180, 180]` degrees, and
polygon/gate segments must not cross the antimeridian. Red zones are evaluated
before the green-zone union. `evidence` remains a human-readable provenance
label; `evidence_file` is optional, but when present it is resolved and SHA-256
pinned through the standard scenario file resolver. The monitor is
forward-only: the schema does not carry an aimpoint, target coordinate, or
inverse burn-to-reach surface.

`stop_s` remains the upper time bound; a descending local-altitude
trajectory that crosses `position.z <= 0` stops earlier with a
`ground-impact` stop reason, even without a manual mission event.
Orbital gravity profiles do not install this stop because ECI
`position.z` is not a ground-altitude proxy.

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
dense_output = true               # adaptive dopri54 | dopri853

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
`solver.adaptive.dense_output = true` is state-stable and opt-in: the runner
accepts it for `adaptive-explicit` + `dopri54` or `dopri853`, where
`openbmp-sim` provides method-specific dense segments for event localization and
output resampling (`Dopri54Adaptive::advance_with_dense_output` quartic DOPRI5,
`Dopri853Adaptive::advance_with_dense_output` order-7 DOP853). The runner
rejects dense output for fixed-step profiles and for adaptive methods without a
wired interpolant.

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
cio_xys = "data/earth_orientation/example-cio-xys.toml"
cio_xys_sha256 = "<64 hex chars>"

[frames]
profile = "iers-cio"
```

Accepted frame profiles are `toy-fixed-earth`, `wgs84-uniform-rotation`,
`iers-tabulated`, `iers-cio`, and validation-only `spice-reference`.
`iers-tabulated` requires `[epoch]`, `epoch.scale = "UTC"`, and `epoch.eop`.
`iers-cio` additionally requires a leap-second source via
`epoch.leap_second_table` or a NAIF LSK in `environment.ephemeris_meta_kernel`.
It generates IAU 2006/2000A CIO X/Y/s internally by default; if
`epoch.cio_xys` is supplied, that pinned table is used instead. These files are
loaded through the same SHA-256-pinned resolved-file path used for aero decks
and motor curves.

The EOP table format currently consumed by the runner is deterministic
TOML with scenario-relative sample times:

```toml
format = "openbmp-eop-v1"

[[samples]]
time_s = 0.0
ut1_minus_utc_s = 0.102
x_pole_arcsec = 0.045
y_pole_arcsec = 0.312
cip_offset_x_arcsec = 0.001
cip_offset_y_arcsec = -0.002
lod_s = 0.001

[[samples]]
time_s = 60.0
ut1_minus_utc_s = 0.103
x_pole_arcsec = 0.045
y_pole_arcsec = 0.312
cip_offset_x_arcsec = 0.001
cip_offset_y_arcsec = -0.002
lod_s = 0.001
```

Samples must be strictly time-ordered and cover `[time.start_s,
time.stop_s]`. OpenBMP linearly interpolates polar motion and the optional
`cip_offset_x_arcsec` / `cip_offset_y_arcsec` celestial-pole-offset pair.
UT1-UTC uses linear interpolation unless both bracketing samples include
optional `lod_s`; with `lod_s`, it uses length-of-day-constrained Hermite
interpolation and reports the corresponding sampled Earth spin rate. EOP rows
may omit both CIP-offset fields for legacy equinox-frame scenarios, but a row
that declares only one of the two offset fields is rejected.
The checked `data/eop/finals2000a-2017-001-004-openbmp-eop-v1.toml` fixture is
a provenance-pinned real-data example clipped from IERS/USNO `finals2000A.data`.
The runner also accepts raw fixed-width `finals2000A.data` rows directly in
`epoch.eop`; for that path it converts row MJD UTC to scenario-relative
`time_s` using `epoch.iso8601`, uses Bulletin B final columns when present, and
falls back to Bulletin A columns otherwise. `data/eop/finals2000a-2017-001-004.raw`
is the matching raw fixture.
The runner also accepts raw IERS EOP 14 C04 IAU2000A text with the published
header and daily rows. For that path it consumes MJD, x/y pole, UT1-UTC, LOD,
and dX/dY in the published C04 units and converts row MJD UTC to
scenario-relative `time_s`. `data/eop/eopc04-14-iau2000a-2017-001-004.raw`
is the checked C04 raw fixture.
The transform applies compact IAU 1976 mean precession and IAU 1980
nutation from J2000 to date, uses the scenario UTC epoch to compute IAU
Earth Rotation Angle, and applies a compact polar-motion rotation in the
ECI/ECEF transform.
Velocity transforms include the finite-difference rate of the full
J2000-to-ECEF orientation chain.

`iers-cio` uses the CIO product `RPOM * R3(ERA) * RC2I`. By default the runner
generates ERFA-compatible IAU 2006/2000A X/Y/s from TT. Scenarios may instead
provide deterministic TOML X/Y/s tables:

```toml
format = "openbmp-cio-xys-v1"

[[samples]]
time_s = 0.0
x_rad = 0.0010
y_rad = -0.000002
s_rad = 0.000000001

[[samples]]
time_s = 60.0
x_rad = 0.0011
y_rad = -0.000003
s_rad = 0.000000002
```

Samples must be strictly time-ordered and cover `[time.start_s, time.stop_s]`.
OpenBMP linearly interpolates X, Y, and s, adds EOP dX/dY to X/Y before
constructing `RC2I`, derives UT1 and TT through the pinned EOP and leap-second
tables, and applies polar motion. Generated mode computes X/Y/s from TT, still
adds EOP dX/dY offsets to X/Y, and keeps the same ERA, TIO, and polar-motion
construction. This profile does not implement SPICE frame chains.

When `epoch.leap_second_table` is declared, the runner loads it through
the same resolved-file path and optional SHA-256 pin as other external
inputs. The field accepts either the deterministic OpenBMP TOML format
or a NAIF `KPL/LSK` leap-second text kernel containing
`DELTET/DELTA_AT` entries. For `environment.ephemeris = "spk"` with an
`ephemeris_meta_kernel`, a NAIF LSK referenced by `KERNELS_TO_LOAD` can
also supply UTC -> TDB conversion when `epoch.leap_second_table` is
omitted. An explicit `epoch.leap_second_table` takes precedence. The
OpenBMP TOML format is:

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
TT epochs are converted to TDB with the shared `TimeScaleBridge` ERFA
`eraDtdb`-compatible helper. If `[frames.local_origin]` is present, its
WGS84 geodetic origin supplies the topocentric dtdb observer geometry;
otherwise conversion uses geocentric observer geometry. SPK ephemeris epochs
may use `TDB`, `TT`, or `UTC`; `UTC` requires either `epoch.leap_second_table` pointing at an
OpenBMP TOML leap-second table / NAIF LSK file such as `naif0012.tls`,
or a NAIF LSK in the selected SPK meta-kernel.

## Model Ordering

The force model list is ordered and deterministic:

```toml
[forces]
models = ["gravity", "aero", "thrust"]
```

The kernel evaluates and sums force and moment providers in declared order.
Changing the order is a scenario change and can change golden telemetry.

## Names, Units, and Frames

Scenario fields and model names should be specific enough for reviewers and
tools to understand their role. The parser enforces dimensional unit suffixes
and vector-frame suffixes, but it does not reject names based on a policy
vocabulary list.

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

### Structural modes and reaction

Rigid-body scenarios may declare legacy first-bending parameters with
`[vehicle.bending]`. Schema v3 scenarios may instead declare
declaration-ordered lateral modes under `[vehicle.modes]`; the two blocks are
mutually exclusive.

```toml
[vehicle.modes]

[[vehicle.modes.lateral]]
id = "first"
frequency_hz = 4.0
damping_ratio = 0.04
modal_mass_kg = 5000.0
slope_at_engine = 0.02
slope_at_gyro = 0.005
shape_at_accelerometer = 0.0

[vehicle.reaction]
enabled = true
```

Each `[[vehicle.modes.lateral]]` entry requires a unique non-empty `id`,
positive `frequency_hz` and `modal_mass_kg`, non-negative `damping_ratio`, and
finite mode-shape values. `[vehicle.reaction]` is v3-only and requires either
`[vehicle.bending]` or `[vehicle.modes]`. When omitted, structural gyro and
accelerometer pickup can still be active, but the mode-summed reaction torque
fed back into rigid-body dynamics remains exactly zero. Set
`[vehicle.reaction] enabled = true` to opt into the T1 lever-reaction feedback
path.

Schema v3 `[vehicle.modes]` can alternatively load a provenance-pinned external
modal asset. Asset-backed modes are mutually exclusive with inline
`[[vehicle.modes.lateral]]` rows. The first supported asset format is CSV v1
with the row schema marker `openbmp.modal_asset.csv.v1`; the scenario resolver
verifies the optional file hash before the runner maps the rows into the same
structural modal rack.

```toml
[vehicle.modes]

[vehicle.modes.asset]
format = "csv_v1"
file = "data/modal/synthetic-sol103-modal-asset-v1.csv"
file_sha256 = "6d3f3c621a9340d509cca35f111826ad06cf45bc2ded92027a98b4af3c5f3171"
frequency_tolerance_fraction = 0.01
mac_min = 0.9
```

Asset ingestion fails closed on schema/version mismatch, invalid numeric rows,
duplicate mode ids, frequency-reference error above
`frequency_tolerance_fraction`, or source-shape MAC below `mac_min`.

Schema v3 scenarios with `[vehicle.bending]` or `[vehicle.modes]` can also
byte-lock the structural transient integrator. Omitting `[vehicle.transient]`
keeps the legacy symplectic-Euler modal step. Declaring the block selects the
native constant-step `openbmp-structdyn` integrator; `newmark_beta` requires
explicit `beta` and `gamma`, while `generalized_alpha` requires explicit
`rho_inf` and derives its `(alpha_m, alpha_f, beta, gamma)` set.

```toml
[vehicle.transient]
scheme = "newmark_beta" # legacy_symplectic_euler | newmark_beta | generalized_alpha
beta = 0.25
gamma = 0.5
```

```toml
[vehicle.transient]
scheme = "generalized_alpha"
rho_inf = 0.5
```

Schema v3 scenarios can also opt into structural line-load recovery. The block
is off by default, requires `[vehicle.bending]` or `[vehicle.modes]`, and emits
one telemetry channel per declared station/output as
`structural.load.<station>.<output>`. Modal coefficient vectors use the locked
order `[mode0.q_x, mode0.q_y, mode1.q_x, mode1.q_y, ...]`.
`mode_acceleration` defaults to true and includes the residual correction.

```toml
[loads.recovery]
mode_acceleration = true

[[loads.recovery.stations]]
id = "interstage"
axial_m = 12.0

[[loads.recovery.stations.outputs]]
id = "shear_x" # axial_force, shear_x, shear_y, bending_x, bending_y, torsion
displacement_coefficients = [0.0, 0.0]
acceleration_coefficients = [10.0, 5.0]
mode_acceleration_coefficients = [1.5, -0.5] # optional; empty means zeros
```

Schema v3 scenarios can also declare structural event-force envelopes under
`[loads.events]`. This requires a structural mode block, evaluates declared
`step`, `ramp_release`, `rectangular_impulse`, `half_sine_impulse`,
`dryden_gust`, or `von_karman_gust` event forces, projects `Phi^T F`, stores
max-absolute modal force envelopes plus CSV bytes in `RunOutcome`, and injects
the same projected event forces into the live structural rack modal state. Rows
may also declare a closed `flight_event_kind` (`liftoff_release`,
`max_q_gust`, `engine_ignition`, `stage_separation`, or `meco`), which is
validated against compatible profile kinds and copied into the structural event
report/CSV. Gust profiles use the declared force vector as the peak max-Q load
and require `gust_length_scale_s`; `dryden_gust` uses an exponential envelope,
while `von_karman_gust` uses a normalized
`(1 + (t/L)^2)^(-5/6)` roll-off. Legacy modal stepping also supports optional
positive `modal_stiffness_scale_1` multipliers for declared event stiffness
changes; explicit Newmark/generalized-alpha transients apply the same
multipliers through a per-step variable-stiffness modal system.

```toml
[loads.events]
sample_times_s = [0.0, 0.5, 1.0]

[[loads.events.forces]]
id = "liftoff_release"
flight_event_kind = "liftoff_release" # optional: liftoff_release | max_q_gust | engine_ignition | stage_separation | meco
kind = "ramp_release" # step | ramp_release | rectangular_impulse | half_sine_impulse | dryden_gust | von_karman_gust
start_s = 0.0
duration_s = 1.0 # required except for kind = "step"
# gust_length_scale_s = 0.25 # required only for gust kinds
# modal_stiffness_scale_1 = [0.75, 1.25] # optional modal-coordinate multipliers
physical_force_n = [8.0, -4.0]
mode_shape_rows_1 = [[1.0, -1.0], [0.25, 2.0]]
```

Schema v3 scenarios with `[vehicle.bending]` or `[vehicle.modes]` can also
declare a control-structure-interaction margin sweep under `[csi]`. The block
is a typed front door for the FC-portable `openbmp-fc::csi` helpers: it
declares the sampled frequency grid, required gain/phase margins, and
exactly one frozen-time plant source. Authors can enter open-loop plants using
continuous-time numerator/denominator coefficients in ascending powers of `s`,
or ask the runner to assemble gyro-rate / accelerometer modal plants from the
active structural modes via `[csi.modal_assembly]`. The rigid-body runner
evaluates the sweep into `RunOutcome.csi` margin summary and CSV evidence.
`[csi.filter_insertion]` can also derive a three-axis FC gyro notch from a
selected structural modal frequency and insert it into the autopilot filter
table. The current time-domain CSI boundedness evidence is a synthetic
runner-level SIL check, not cFS/HIL qualification or flight-test validation.

```toml
[csi]
frequency_grid_rad_s = [1.0, 4.0, 16.0]

[csi.criteria]
min_gain_margin_db = 6.0
min_phase_margin_deg = 30.0

[[csi.frozen_plants]]
id = "liftoff"
time_s = 0.0
numerator_coefficients = [1.0]
denominator_coefficients = [1.0, 0.6, 0.08]

[[csi.frozen_plants]]
id = "max_q"
time_s = 40.0
numerator_coefficients = [1.2, 0.01]
denominator_coefficients = [1.0, 0.5, 0.06]
```

Modal assembly uses `slope_at_engine` as the modal input coupling, then
`slope_at_gyro` for `sensor = "gyro_rate"` or `shape_at_accelerometer` for
`sensor = "accelerometer"`.

```toml
[csi]
frequency_grid_rad_s = [1.0, 4.0, 16.0]

[csi.criteria]
min_gain_margin_db = 6.0
min_phase_margin_deg = 30.0

[csi.modal_assembly]
sensor = "gyro_rate"
loop_gain = 2.5

[[csi.modal_assembly.frozen_points]]
id = "liftoff"
time_s = 0.0

[[csi.modal_assembly.frozen_points]]
id = "max_q"
time_s = 40.0
loop_gain_scale = 1.25
```

CSI-designed filter insertion uses the same FC autopilot notch table as
`[fc.autopilot_params.gyro_notch]`, so the two declarations are mutually
exclusive.

```toml
[csi.filter_insertion]
mode_index = 0
zero_damping_ratio = 0.02
pole_damping_ratio = 0.20
```

### Landing gear

Schema v3 rigid-body scenarios may opt into a massless landing-gear rack with
`[vehicle.landing_gear]`. When `[forces]` is omitted, the parser derives the
`landing_gear` force model automatically. When `[forces]` is explicit, it must
include `"landing_gear"` whenever this block is present. The block is rejected
for schema v2 and for point-mass vehicles.

```toml
[vehicle.landing_gear]
data_file = "../../data/landing_gear/synthetic-four-leg-oleo-crush.toml"
data_file_sha256 = "efef5b573d83551fecd724383c9420de712bd62665e5ff989642916bbf2a2d5b"
ground_altitude_m = 0.0

[[vehicle.landing_gear.legs]]
id = "front_left"
mounted_to = "core"
attach_body_m = [1.0, 1.0, 0.0]
strut_axis_body = [0.0, 0.0, -1.0]
free_length_m = 1.0
footpad = "sphere" # "point" | "sphere"
footpad_radius_m = 0.12
footpad_friction_coefficient = 0.0
footpad_friction_regularization_speed_m_s = 0.001
oleo = { p0_pa = 150000.0, v0_m3 = 0.06, gamma_unit = 1.25, orifice_c_n_s2_m2 = 800000.0, stroke_max_m = 0.55, piston_area_m2 = 0.015 }
crush = { f_crush_n = 40000.0, stroke_max_m = 0.25, k_elastic_n_m = 400000.0 }
prismatic_dof = { stroke_max_m = 0.80, initial_stroke_m = 0.0, initial_compression_rate_m_s = 0.0, mechanism_id = "front_left_stop" }

[[contact.mechanism]]
id = "front_left_stop"
coordinate = "front_left"
kind = "stop"
side = "lower"
limit_rad = 0.2
stiffness_n_m = 1000.0
```

Each leg id must be unique, `mounted_to` must name a declared
`[[vehicle.assembly.bodies]]` id, `strut_axis_body` must be nonzero, and
`free_length_m` must be positive. Sphere footpads require
`footpad_radius_m`; point footpads reject it. Footpad friction defaults to
zero and, when supplied, uses the same tanh-regularized Coulomb convention as
`[contact]`: the coefficient must be non-negative and
`footpad_friction_regularization_speed_m_s` must be positive. At least one of
`oleo` or `crush` is required per leg. Oleo pressure, volume, exponent
(`gamma_unit`),
stroke, and piston area must be positive; damping may be zero; and
`piston_area_m2 * stroke_max_m` must stay below `v0_m3`. Crush force,
stroke, and elastic stiffness must be positive.

`prismatic_dof` is optional. When present it declares the tree-facing
compression coordinate contract for the leg without changing the current
massless landing-gear force model. `stroke_max_m` must be positive and must not
exceed `free_length_m`; `initial_stroke_m` defaults to zero and must lie inside
`[0, stroke_max_m]`; `initial_compression_rate_m_s` defaults to zero and must
be finite. The runner materializes the current prismatic coordinate from the
live gear stroke as `displacement = free_length_m - stroke_m` and
`displacement_rate = -compression_rate_m_s`. `mechanism_id` is optional; when
present it must reference a declared `[[contact.mechanism]]` whose `coordinate`
equals the landing-gear leg id. The runner evaluates that scalar mechanism on
the prismatic displacement coordinate and adds only the positive compressive
part of the generalized force to the external-normal landing-gear load.

The rigid-body runner publishes `force.landing_gear.{x,y,z}_n` plus per-leg
`landing_gear.<id>.stroke_m`, `.gap_m`, `.compression_rate_m_s`, `.force_n`,
`.crushed_m`, and `.in_contact`. Referenced gear data files are resolved
relative to the scenario file and recorded in telemetry metadata under
`openbmp.scenario_files.vehicle.landing_gear.data_file`. The runner also
publishes `.prismatic_displacement_m`,
`.prismatic_displacement_rate_m_s`, and
`.prismatic_footpad_body_{x,y,z}_m` only for legs declaring
`prismatic_dof`. The runner also
emits `RunOutcome.landing_gear` outside canonical telemetry bytes with
NoContact/Rest/Unsettled classification, final per-leg samples, load/stroke
maxima, per-pad tangential speed/friction evidence, and a deterministic gear
energy audit. The helper
`openbmp_runner::landing_gear::recover_section_loads_body_x` recovers
structural body-x section loads from those final per-leg samples for line-load
consumers.

### Gravity coefficients

`[environment].gravity` selects one of `constant`, `point_mass`, `j2`,
`egm2008`, `tesseral`, or `third_body`, each with its own
required-coefficient set:

| `gravity` | Required | Rejected |
|---|---|---|
| `"constant"` | `gravity_m_s2` | `mu_m3_s2`, `r_e_m`, `j2`, `gravity_base`, `third_bodies`, `ephemeris`, `ephemeris_file`, `ephemeris_files`, `ephemeris_meta_kernel` |
| `"point_mass"` | `mu_m3_s2` | `gravity_m_s2`, `r_e_m`, `j2`, `gravity_base`, `third_bodies`, `ephemeris`, `ephemeris_file`, `ephemeris_files`, `ephemeris_meta_kernel` |
| `"j2"` | `mu_m3_s2`, `r_e_m` | `gravity_m_s2` |
| `"egm2008"` | — pinned WGS84 / EGM2008 zonal constants; or `egm2008_coefficients_file` plus either `egm2008_degree`/`egm2008_order` or `egm2008_tier` for opt-in coefficient-file mode | `gravity_m_s2`, `mu_m3_s2`, `r_e_m`, `j2`, `gravity_base`, `third_bodies`, `ephemeris`, `ephemeris_file`, `ephemeris_files`, `ephemeris_meta_kernel` |
| `"tesseral"` | `mu_m3_s2`, `r_e_m`, `tesseral_degree`, `tesseral_order`, `tesseral_c20`, `tesseral_c21`, `tesseral_s21`, `tesseral_c22`, `tesseral_s22` | `gravity_m_s2`, `j2`, `gravity_base`, `third_bodies`, `ephemeris`, `ephemeris_file`, `ephemeris_files`, `ephemeris_meta_kernel` |
| `"third_body"` | `gravity_base`, `third_bodies`; plus the selected base coefficients | `gravity_m_s2` |

For `gravity = "j2"` the dimensionless `j2` coefficient defaults to the
WGS84 value (1.082626683 × 10⁻³) when omitted.

`gravity = "third_body"` wraps a central Earth gravity model and adds
Sun/Moon point-mass perturbations using
`a_3 = mu_b * ((r_b - r) / |r_b - r|^3 - r_b / |r_b|^3)`.
`gravity_base` is one of `point_mass`, `j2`, `egm2008`, or
`tesseral`. The
`third_bodies` list accepts `"sun"` and/or `"moon"` and must be unique.
`ephemeris = "low_precision_sun_moon"` selects the built-in
deterministic analytical ephemeris; if `[epoch]` is omitted the
ephemeris starts at J2000, otherwise the runner parses
`epoch.iso8601` with `epoch.scale` `UTC`, `TT`, or `TDB`.
`ephemeris = "spk"` selects pinned binary SPK/BSP kernels supplied by
exactly one of:

- Singular `ephemeris_file` plus optional `ephemeris_file_sha256`.
- Ordered `ephemeris_files` plus optional `ephemeris_files_sha256`.
- A NAIF `KPL/MK` `ephemeris_meta_kernel` plus optional
  `ephemeris_meta_kernel_sha256` and ordered
  `ephemeris_meta_kernel_files_sha256` pins for the expanded
  `KERNELS_TO_LOAD` list.

The plural pin lists, when present, must match their file-list length.
Meta-kernel path symbols `PATH_SYMBOLS` / `PATH_VALUES` and `+`
continuations are supported; relative referenced paths resolve against
the meta-kernel's parent directory. The runner consumes binary SPK
entries from the expanded list and records every referenced file for
provenance. A NAIF LSK referenced by the meta-kernel can supply UTC
time conversion if `epoch.leap_second_table` is omitted; when several
referenced LSK files are present, the last one in `KERNELS_TO_LOAD`
order is used. Meta-kernel referenced text kernels can also define fixed
mission frames with `TKFRAME_*_SPEC` values `MATRIX`, `ANGLES`, or
`QUATERNION`; matrices use NAIF's column-major kernel order and the
resolved transform is applied as `V_relative = M * V_tkframe`. Later SPK
files take precedence over earlier files for overlapping SPK segments.
The SPK reader evaluates seconds past J2000 on the ephemeris-time/TDB
axis, so `[epoch].scale` may be `"TDB"`, `"TT"`, or `"UTC"`; `"UTC"`
requires either `epoch.leap_second_table` or a NAIF LSK in the selected
meta-kernel.

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

```toml
[environment]
frame_profile = "wgs84-uniform-rotation"
gravity       = "third_body"
gravity_base  = "egm2008"
third_bodies  = ["sun", "moon"]
ephemeris     = "spk"
ephemeris_meta_kernel = "data/ephemeris/mission.tm"
ephemeris_meta_kernel_sha256 = "<64 hex chars>"
ephemeris_meta_kernel_files_sha256 = [
  "<64 hex chars>", # first KERNELS_TO_LOAD entry
  "<64 hex chars>", # second KERNELS_TO_LOAD entry
]
atmosphere    = "none"
wind          = "none"
```

The SPK reader supports geometric Sun/Moon states from binary DAF/SPK
kernels with type 1 modified-difference arrays, type 2 or type 3
Chebyshev segments, type 5 two-body discrete-state segments, type 8/9
Lagrange state segments, type 10 TLE/SGP4 segments, type 12/13 Hermite
state segments, type 14 generic Chebyshev position/velocity segments,
type 15 precessing-conic segments, type 17 equinoctial-element segments,
type 18 ESOC/DDID
subtype 0/1 packets, type 19 ESOC/DDID piecewise mini-segments, type 20
Chebyshev velocity-only segments, and type 21 extended
modified-difference arrays in NAIF's built-in SPICE inertial frame IDs
1-21 or in fixed `TKFRAME_*_SPEC` frames loaded from the selected
meta-kernel, rotating those segment states into OpenBMP's J2000 ECI
chain. Fixed TK frames may use `MATRIX`, `ANGLES`, or `QUATERNION`
specs, and may be relative to a supported built-in inertial frame or to
another fixed TK frame declared in the same text kernel. Type 10 remains
J2000-only because its evaluator already rotates TEME states into J2000.
Type 1 evaluates fixed-dimension
modified difference lines; type 2 velocities are derived from the
Chebyshev position derivative; type 3 velocities come from the segment
velocity coefficients; type 5 propagates bracketing discrete states with
two-body motion and blends the propagated states with NAIF's cosine
weighting; type 8/9 states are interpolated from
equal/unequal-time discrete position/velocity records; type 12/13
states use equal/unequal-time Hermite interpolation of position and
velocity records; type 14 evaluates fixed-packet generic-segment
Chebyshev position and velocity records with explicit epoch indexing;
type 15 propagates from periapsis and applies optional J2 node/apsis
precession flags; type 17 evaluates equinoctial elements, their constant
rates, and the record's reference-plane pole transform;
type 18 subtype 0 interpolates ESOC/DDID position and velocity packets
separately from their derivative fields, while subtype 1 uses Lagrange
interpolation of position/velocity packets; type 19 selects the
appropriate interpolation interval, honors the segment's boundary-choice
flag, and evaluates subtype 0/1/2 mini-segments; type 20 integrates
Chebyshev velocity polynomials from each record midpoint position
constant; type 10 evaluates fixed-packet TLE elements with SGP4,
applies NAIF's cosine blend between bracketing element sets, and rotates
TEME states into J2000; type 21 evaluates variable-dimension extended
modified difference lines. It follows SPK segment priority inside each file and
preserves load-order precedence across a file list, so later files can
override earlier overlapping segments. It combines target/center chains
such as Solar-System-Barycenter -> Earth-Moon Barycenter -> Earth/Moon.
Geometric states are the default and remain the input to force models;
the SPK API also exposes reception-side `LT`, `LT+S`, `CN`, and
`CN+S` observer corrections plus transmission-side `XLT`, `XLT+S`,
`XCN`, and `XCN+S` pointing corrections. These are Newtonian
light-time / stellar-aberration corrections like SPICE's documented
`ABCORR` flags; higher-order relativistic effects remain outside
scope. It does not yet implement generic text kernels beyond NAIF LSK
leap-second files and fixed TK frames, dynamic frames, or a full
FK/PCK/CK frame-kernel chain.

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

Schema v3 accepts an opt-in deterministic deck draw under `[aero.deck_draw]`
for deck-backed or buildup-backed coefficient sources that carry per-node
sigma tables. The runner applies the draw once at deck-load time; scenarios
without this block preserve nominal deck behavior.

```toml
[aero.deck_draw]
campaign_seed = 42
sample_index = 7
dimension_id_base = 200
mach_correlation_length = 1.5 # optional when the deck declares [uncertainty.correlation].mach_length
alpha_correlation_length_deg = 4.0 # optional
beta_correlation_length_deg = 3.0  # optional
```

The draw uses three deterministic coefficient streams beginning at
`dimension_id_base` for `CN`, `CD`, and `CM`. If the deck also carries
`[uncertainty.six_coefficients.*]`, the same draw uses six additional streams
at `dimension_id_base + 3` through `dimension_id_base + 8` for
`CX/CY/CZ/Cl/Cm/Cn` and fails closed if the dimension range overflows.
Deck-declared `[uncertainty.correlation].coefficient` and
`[uncertainty.correlation].six_coefficient` matrices mix the reduced or
six-channel residual streams before per-node sigma scaling.
`mach_correlation_length` is in Mach-axis units; when omitted, the runner uses
the loaded deck's `[uncertainty.correlation].mach_length` metadata and fails
closed if neither source is available. `alpha_correlation_length_deg` and
`beta_correlation_length_deg` are optional degree-valued covariance lengths;
when omitted, the draw keeps the backward-compatible same-Mach residual sharing
across the omitted axis. This is a single-sample runner hook; `openbmp mc
aero-deck-draw-overlays` can materialize per-sample campaign overlay files and
a manifest fragment for sample-index scheduling, while `openbmp mc
aero-deck-draw-campaign` generates and executes a native campaign over those
draws. Both campaign commands accept `--alpha-correlation-length-deg` and
`--beta-correlation-length-deg`; the campaign runner also accepts `--uq-toml`,
`--credibility-floor`, and `--credibility-report-md` for sidecar UQ credibility
gating/reporting.

Schema v3 also accepts an offline aero-database run-matrix asset under
`[aero.run_matrix]`. The file must be an `openbmp.aerodb_run_matrix = 1` TOML
document; the scenario layer resolves and pins it, and the runner exposes a
file-fed helper that parses it through the condition-only `openbmp-aerodb`
schema. This declaration is not a flight-runtime aerodynamic coefficient source
and does not launch an external solver. A scenario that uses `[aero]` only for
this offline asset should keep `"aero"` out of `[forces].models`.

```toml
[aero.run_matrix]
file = "data/aero/synthetic-run-matrix.toml"
file_sha256 = "..." # optional
```

Schema v3 also accepts an opt-in plume telemetry block under `[aero]`.
This block is not an aerodynamic coefficient source; it may be declared
without `deck`, `[aero.buildup]`, or `[aero.method]` only when `aero` is
not in the active force model list. An `[aero]` block without
`[aero.plume]` does not allocate plume telemetry channels and preserves the
same canonical telemetry bytes as omitting `[aero]`.

```toml
[aero]

[aero.plume]
engine_count = 1
reference_area_m2 = 1.0
exit_area_total_m2 = 0.002
base_area_m2 = 1.0
center_spacing_m = 0.0
merge_evaluation_distance_m = 0.0
pifs_onset_angle_rad = 0.05
```

For point-mass solid-motor scenarios with a runner-sampled atmosphere,
`[aero.plume]` emits `plume.nozzle_pressure_ratio`,
`plume.exit_pressure_ratio`, `plume.thrust_coefficient`,
`plume.momentum_flux_ratio`, `plume.initial_turn_angle_rad`,
`plume.merge_distance_m`, `plume.cluster_merged`, and
`plume.pifs_onset`. `plume.momentum_flux_ratio` uses total nozzle thrust
over `rho_inf * V_inf^2 * reference_area_m2`, with dynamic pressure supplying
one half of `rho_inf * V_inf^2`. The point-mass path derives single-engine
plume geometry from the solid motor: the runtime geometry uses
`engine_count = 1`, the motor nozzle exit area, and zero center spacing even
though the static geometry fields remain required for schema compatibility.
Rigid-body liquid-engine scenarios may emit the same telemetry when every
declared liquid engine has
`[vehicle.assembly.engines.thermochemical_performance]`; the runner uses
the thermochemical nozzle metadata plus live engine mass-flow snapshots.
For that rigid-body liquid path, active engine count, summed exit area, and
center-to-center spacing are derived from active engine snapshots and
body-frame engine mount points even though the schema still requires the
static plume geometry fields for compatibility with the point-mass path.
Inactive or singular rows, such as a stationary pre-burn row with zero
dynamic pressure, are emitted as zero/false.

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
`AerothermalContext`, evaluates Sutton-Graves or Fay-Riddell convective
stagnation heating, optionally adds Tauber-Sutton radiative heating, and emits
`aerothermal.*` telemetry. Optional
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

[aerothermal.radiative]
kind = "tauber_sutton_earth_air"

[aerothermal.electron_density_proxy]
source_id = "synthetic-entry-density.v1"
evidence = "docs/parity/20-telemetry-rf-links-and-ground-network.md#wp-205"
activation_temperature_k = 1500.0
reference_temperature_k = 3000.0
reference_electron_number_density_m3 = 5.0e16
max_electron_number_density_m3 = 1.0e18

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
`mass.aerothermal_mass_loss_kg_s`. When
`[aerothermal.electron_density_proxy]` is declared, telemetry also includes
`aerothermal.electron_number_density_m3`; this is a declared proxy for comm
blackout coupling, not a real-gas chemistry validation.

### Free-field DSM acoustics

`[acoustics.dsm]` is an opt-in schema-v3 analysis block. It does not affect
forces, mass, or controller inputs, and scenarios that omit `[acoustics]`
allocate no `acoustics.*` telemetry channels. The runner maps the declared
free-field single-engine or identical-engine cluster DSM inputs into
`openbmp-acoustics`, emits an
out-of-band `RunOutcome.acoustics` report with a station spectrum CSV and
OASPL-vs-height CSV, and adds summary telemetry channels:
`acoustics.dsm.acoustic_power_w`, `acoustics.dsm.sound_power_level_db`,
`acoustics.dsm.station_oaspl_db`, and `acoustics.dsm.source_count`. This
surface can use inline source-line weights, a SHA-pinned Schema-1 `deck_file`
containing DSM source allocation and directivity rows, or a SHA-pinned
SP-8072 Figure 12 `source_curve_file` that derives normalized source
allocation from a declared `exit_mach`, or a SHA-pinned SP-8072 Figure 14
`apparent_source_curve_file` that places one apparent source per band from
Strouhal number. Inline, deck-backed, and Figure 12 distributed-source inputs
can also pair with a SHA-pinned SP-8072 Figure 13
`axial_spectrum_curve_file`, which applies source-position-dependent
source-by-band weights. Inline, Figure 12, and Figure 14 source inputs can also
pair with a SHA-pinned SP-8072 Figure 10 `directivity_curve_file`, which derives
per-band DSM directivity rows from declared exit diameter and velocity. Any
source/directivity choice can also use a SHA-pinned SP-8072 Figure 5
`spectrum_curve_file`, which derives normalized one-third-octave band-power
fractions from declared exit diameter and velocity. The shipped SP-8072 curve
decks and runner curve loaders validate their manifest id, source PDF hash, and
figure id against the pinned NASA SP-8072 source manifest after scenario
SHA-pin verification, before they are treated as public-source evidence.
Propagation uses spherical
spreading plus either the scalar `absorption_db_per_m` expanded uniformly over
the active band grid or an optional SHA-pinned `absorption_deck_file` with
per-band `dB/m` coefficients. Absorption decks may expose named atmosphere
profiles with representative temperature/pressure/humidity metadata; declare
`absorption_profile_id` to select one, otherwise the deck's default profile is
used. Optional `pad_path_length_extra_m` adds a declared
duct/trench/deflector path-length term before spherical spreading and
absorption. Optional `pad_deflection_origin_axial_m` plus
`pad_deflection_angle_deg` maps source-line coordinates through a declared
single-bend plume axis before range/directivity evaluation. A SHA-pinned
`suppression_deck_file` plus `suppression_class` can apply a per-band negative
transfer for synthetic pad water-suppression evidence. Optional
`cluster_engine_count` treats the
declared exit condition as per engine and applies exact incoherent
identical-engine acoustic-power scaling. Optional
`[[acoustics.dsm.cluster_engines]]` entries instead declare lateral engine
offsets; the runner energy-adds each engine's station spectrum after applying
its geometric receiver range/directivity, with `receiver_azimuth_deg`
selecting the receiver bearing in the cluster plane. The cluster geometry path
can optionally apply a scalar `cluster_correlation_coefficient` cross-pressure
term and a broadband `cluster_shielding_loss_db` after summation, or a
SHA-pinned `cluster_interaction_deck_file` plus `cluster_interaction_class`
for synthetic per-band correlation and shielding. The scalar and deck-backed
cluster interaction paths are mutually exclusive. The runner does not yet model
calibrated plume interaction or cluster-specific directivity. The Figure 5,
Figure 10, Figure 12, Figure 13, and Figure 14 paths are coarse public curve
fixtures for parser/integration evidence; the absorption, atmosphere-profile,
suppression, and cluster-interaction deck fixtures are synthetic. Production
SP-8072 calibration, production/public atmospheric attenuation, pad calibration, and
worked-example validation remain separate work.

`[acoustics.iop]` is an opt-in schema-v3 ignition-overpressure analysis block.
It uses source strength from either explicit `gas_generation_rate_n_per_s` plus
`ramp_duration_s` or a declared liquid-engine `source_engine_id` whose
start-rate proxy is derived from `limits.max_thrust_n /
limits.ignition_transient_s`. For source engines with
`[vehicle.assembly.engines.thermochemical_performance]`, the runner instead
uses the same thermochemistry-resolved nominal `mdot*c*` source strength
divided by `limits.ignition_transient_s`, while also reporting the
thermochemistry-derived max thrust. Duct/trench residence time comes from
declared volume, exit area, and discharge velocity or from a SHA-pinned
synthetic `duct_deck_file` plus `duct_class`; propagation uses inverse-square
receiver decay, optional pressure-amplitude suppression loss, optional
SHA-pinned synthetic `suppression_deck_file` plus
`suppression_class`, and lower/upper dB uncertainty bounds. The scalar
`suppression_loss_db` and deck-selected suppression loss are additive. When
`source_engine_id` is used, the report also records the first declared
`[[scenario_script.events]]` `at_time` `engine_command` that targets the
source engine with `ignite = true`, when such an event exists; during
point-mass and rigid-body runs it also records the first runtime-observed
fired source-engine ignite event id, step, and time. Point-mass and rigid-body
source-engine IOP reports also carry a deterministic runtime source-history CSV
with step, time, source strength, finite-difference source rate, thrust, and
mass-flow columns; for thermochemical source engines the runtime source
strength uses `mass_flow_kg_per_s * effective_cstar_m_s`, otherwise it uses
thrust magnitude. When the observed finite-difference source-rate peak is
positive, the report also carries runtime-reconstructed nominal, lower, and
upper IOP peak pressures plus duration, evaluated with the observed peak source
rate and the same duct, receiver, suppression, uncertainty, and ramp-duration
terms as the declared envelope. When present, the runner emits
`RunOutcome.acoustics.iop` plus
gated telemetry channels
`acoustics.iop.peak_overpressure_pa`,
`acoustics.iop.lower_peak_overpressure_pa`,
`acoustics.iop.upper_peak_overpressure_pa`, and
`acoustics.iop.duration_s`. This remains a declared engineering envelope;
calibrated `d(mdot*c*)/dt` reconstruction, runtime pressure-history
calibration of the reconstructed command-response history, calibrated
duct/trench decks, suppression data calibration, and published IOP trend
validation are separate work.

`[acoustics.ascent]` is an opt-in schema-v3 ascent acoustic environment
analysis block. It consumes a SHA-pinned Schema-1 ascent acoustic class deck,
defaults to explicit `class_id` selection, or can use
`class_selection = "mach_gate"` to select the unique deck class whose Mach gate
contains each single or trace sample. It evaluates either a declared single
`dynamic_pressure_pa`/`mach` sample or `[[acoustics.ascent.samples]]` trace
rows, and emits
`RunOutcome.acoustics.ascent` with deck SHA-256, dataset id, source class,
peak-sample metadata, envelope OASPL, uncertainty, deterministic
one-third-octave envelope spectrum CSV bytes, and deterministic sample trace
CSV bytes. The runner also gates summary telemetry channels
`acoustics.ascent.oaspl_db` and `acoustics.ascent.uncertainty_db`. This path is
synthetic class-deck evidence only. When at least two trace rows are declared,
the ascent report also carries P95/50 OASPL, tolerance factor, and deterministic
environment-table CSV bytes derived from the trace sample spectra. Full
geometry-driven `03` regime/zone selection, published payload-user-guide or MPE
curves, and flight correlation are separate work.

`[acoustics.fairing_internal]` is an opt-in schema-v3 fairing internal acoustic
transfer block. It requires `[acoustics.ascent]` in the same scenario, consumes
that external ascent envelope spectrum, applies a SHA-pinned Schema-1 fairing
insertion-loss deck plus dimensionless `fill_factor`, and emits
`RunOutcome.acoustics.fairing_internal` with deck SHA-256, dataset id, source
class, selected class, fill factor, fill-factor gain, OASPL, uncertainty, and
deterministic one-third-octave spectrum CSV bytes. The runner also gates
`acoustics.fairing_internal.oaspl_db`,
`acoustics.fairing_internal.uncertainty_db`, and
`acoustics.fairing_internal.fill_factor_gain_db`. This path is synthetic
transfer-deck evidence only. When the source ascent block has at least two
trace samples, the fairing report also carries internal P95/50 OASPL,
tolerance factor, deterministic environment-table CSV bytes, and deterministic
fairing P95/50 sample-trace CSV bytes from the transferred trace sample
spectra. Optional `[[acoustics.fairing_internal.samples]]` rows can provide
per-ascent-sample fairing transfer inputs for those P95/50 calculations: each
row may declare `sample_id`, may override `class_id`, and must declare
`fill_factor`. When present, the row count must match the retained ascent
sample spectra. The top-level fairing fields still drive the envelope fairing
response and existing single-response reports. Published payload-user-guide/MPE
curves, payload-zone spatial maps, and flight correlation are separate work.

`[acoustics.response]` is an opt-in schema-v3 vibroacoustic response transfer
block. It requires `source = "ascent"` with `[acoustics.ascent]`, or
`source = "fairing_internal"` with `[acoustics.fairing_internal]`, consumes the
selected one-third-octave SPL spectrum, applies a SHA-pinned Schema-1 response
transfer deck plus `surface_mass_kg_m2`, `area_m2`, and dimensionless
`mode_shape_factor`, and emits `RunOutcome.acoustics.response` with deck
SHA-256, dataset id, source class, selected class, surface-mass gain,
modal-force participation, integrated acceleration mean-square, integrated
modal-force mean-square, and deterministic acceleration/modal-force PSD CSV
bytes. The runner also gates
`acoustics.response.total_mean_square_acceleration_m2_s4`,
`acoustics.response.total_mean_square_force_n2`, and
`acoustics.response.uncertainty_db`. This path is synthetic transfer-deck
evidence only. The response report also carries deterministic structural loads
handoff CSV bytes with modal-force PSD, per-band mean-square force, signed
modal participation, and total mean-square force. In rigid-body scenarios that
also declare `[vehicle.bending]`, the runner consumes the typed acoustic
modal-force PSD through the structural rack and records
`RunOutcome.structural_acoustics` as an out-of-band first-bending random
response summary with per-band displacement/acceleration CSV bytes plus a
first-increment engine-station equivalent reaction-moment line-load recovery
CSV derived from the same first bending mode slope and modal acceleration PSD.
When the selected source has at least two retained sample spectra, the response report
also carries a P95/50 tolerance factor, integrated P95/50 mean-square
acceleration and modal-force values, and deterministic P95/50
acceleration/modal-force PSD CSV bytes. Optional
`[[acoustics.response.samples]]` rows can provide per-source-sample response
inputs for those P95/50 calculations: each row may declare `sample_id`, may
override `class_id`, and must declare `surface_mass_kg_m2`, `area_m2`, and
`mode_shape_factor`. When present, the row count must match the retained source
sample spectra, and the response report emits a deterministic P95/50 sample
trace CSV recording the paired response inputs and per-sample reductions. The
top-level response fields still drive the envelope response and existing
single-response reports. The ascent, fairing-internal, and response P95/50
report families also carry one-sided Wilks sizing evidence: required sample
count, achieved confidence, adequacy, and deterministic sizing CSV bytes. When
`openbmp run` is invoked with run-data output options, nonempty
ascent/fairing/response P95/50 CSV payloads, fairing and response P95/50 sample
traces, the response loads handoff CSV, the structural-acoustics first-bending
response CSV, and the structural-acoustics first-bending line-load recovery CSV
are written under the run-data artifact root and indexed in the `RunRecord`
with role labels, paths, and SHA-256 pins.
`openbmp mc acoustics-ascent-trace` can materialize
deterministic `[[acoustics.ascent.samples]]` TOML snippets from
LHS/Sobol/Owen-Sobol designs over declared dynamic-pressure and Mach ranges,
and `openbmp mc acoustics-response-samples` can materialize deterministic
`[[acoustics.response.samples]]` snippets over declared surface-mass, area, and
mode-shape ranges for response-side P95/50 input variation. `openbmp mc
acoustics-ascent-campaign` can inject generated ascent samples into a base
scenario, run the normal runner, write the evaluated ascent trace CSV, report
ascent P95/50/Wilks evidence, and surface downstream fairing-internal and
response P95/50/Wilks summary values when the base scenario declares those
blocks. `openbmp mc acoustics-response-campaign` is a response-focused wrapper
over the same generated ascent-trace path: it requires the base scenario to
declare `[acoustics.response]`, can optionally append generated
`[[acoustics.response.samples]]` rows from response-side range flags, preserves
the same checkpoint/artifact behavior, and reports the response P95/50
acceleration, modal-force, and Wilks evidence on completion. With
`--output-artifact-dir`, completed campaigns also write nonempty evaluated
campaign CSV artifacts such as ascent/fairing P95/50 environment tables,
response loads handoff and P95/50 PSD tables, response P95/50 sample traces,
and structural-acoustic CSVs when
the generated run emits them. The artifact directory also receives
`manifest.json` with each artifact role, relative path, and SHA-256 pin.
Generated fairing/response design-variable samplers, full run-data campaign
aggregation, calibrated SEA-lite/Franken transfer evidence, and broader
structural `02` stack integration are separate work.

```toml
[acoustics.iop]
station_id = "station-a"
gas_generation_rate_n_per_s = 2400000.0
ramp_duration_s = 0.25
# Optional liquid-engine source coupling. When present, omit
# gas_generation_rate_n_per_s and ramp_duration_s.
# source_engine_id = "main"
duct_volume_m3 = 120.0
duct_exit_area_m2 = 6.0
duct_discharge_velocity_m_s = 40.0
duct_geometry_gain = 1.5
# Optional synthetic duct/trench deck path. When present, omit the four inline
# duct_* scalars above and declare duct_class.
# duct_deck_file = "data/acoustics/iop-duct-v1.toml"
# duct_deck_file_sha256 = "7711a6c472fc5343fda1452626ad7ff41323c8510b6d82d8de6d7b707e807c7e"
# duct_class = "long_trench"
# Optional synthetic pressure-amplitude suppression deck. The selected class
# loss adds to suppression_loss_db.
# suppression_deck_file = "data/acoustics/iop-suppression-v1.toml"
# suppression_deck_file_sha256 = "641894d3cd5ac3dbc8529219b47e39deb1c0974121724fb5023f879f98e9ecc7"
# suppression_class = "water_deluge_nominal"
receiver_distance_m = 30.0
suppression_loss_db = 3.0
uncertainty_minus_db = 2.0
uncertainty_plus_db = 4.0
```

```toml
[acoustics.ascent]
station_id = "station-a"
deck_file = "data/acoustics/ascent-acoustic-env-v1.toml"
deck_file_sha256 = "995283f2f4d9c1134f5fc4cb0e6e857c320137e9544ffc1308f6f0933e83e87b"
class_id = "attached_turbulent"
dynamic_pressure_pa = 50000.0
mach = 0.8

# Optional automatic class mode. When set, omit class_id here and in sample
# rows; the runner selects the unique deck class whose Mach gate contains each
# sample.
# class_selection = "mach_gate"
#
# Optional trace mode. When rows are declared, dynamic_pressure_pa and mach
# above may be omitted; class_id remains the default class for rows without an
# override. The runner envelopes the sample spectra in band order.
[[acoustics.ascent.samples]]
sample_id = "max-q"
dynamic_pressure_pa = 50000.0
mach = 0.8

[[acoustics.ascent.samples]]
sample_id = "late-max-q"
dynamic_pressure_pa = 200000.0
mach = 0.8

[acoustics.fairing_internal]
station_id = "payload-a"
deck_file = "data/acoustics/fairing-internal-v1.toml"
deck_file_sha256 = "fbb2dcc426b9d37a93781ef3621dba5a3c7eebc2c56d01f7fcd78e50e0558500"
class_id = "synthetic_light_fairing"
fill_factor = 1.0

# Optional fairing P95/50 input rows. When declared, the number of rows must
# match the retained ascent sample spectra.
# [[acoustics.fairing_internal.samples]]
# sample_id = "fairing-open"
# fill_factor = 0.25
#
# [[acoustics.fairing_internal.samples]]
# sample_id = "fairing-packed"
# class_id = "synthetic_heavy_fairing"
# fill_factor = 0.75

[acoustics.response]
station_id = "payload-a"
source = "fairing_internal"
deck_file = "data/acoustics/vibroacoustic-transfer-v1.toml"
deck_file_sha256 = "9f791c04c67cc1f8ba3d98d25e541ca33cc044c2ff8cfe61f033d9d66ef37be5"
class_id = "synthetic_panel_light"
surface_mass_kg_m2 = 12.0
area_m2 = 3.0
mode_shape_factor = 0.5

# Optional response P95/50 input rows. When declared, the number of rows must
# match the retained source sample spectra.
# [[acoustics.response.samples]]
# sample_id = "panel-light"
# surface_mass_kg_m2 = 12.0
# area_m2 = 3.0
# mode_shape_factor = 0.5
#
# [[acoustics.response.samples]]
# sample_id = "panel-heavy"
# class_id = "synthetic_panel_heavy"
# surface_mass_kg_m2 = 24.0
# area_m2 = 4.0
# mode_shape_factor = 0.75
```

```toml
[acoustics.dsm]
station_id = "station-a"
# Optional deck path. When present, omit [[acoustics.dsm.source_line]].
# deck_file = "data/acoustics/dsm-source-directivity-v1.toml"
# deck_file_sha256 = "169a6c35e513aafd8f111e6ba7e0b8d707cd0f1c8b1904eabc056d5d7536fe00"
# Optional SP-8072 Figure 12 source curve. When present, omit deck_file and
# [[acoustics.dsm.source_line]], and declare exit_mach.
# source_curve_file = "data/acoustics/sp8072-figure12-source-power-v1.toml"
# source_curve_file_sha256 = "d199bebbb4bcd217f03f7a8ad717f875aa33419f7dbb2532e3b04649fb848b62"
# exit_mach = 1.0
# Optional SP-8072 Figure 14 apparent-source curve. When present, omit
# deck_file, source_curve_file, and [[acoustics.dsm.source_line]].
# apparent_source_curve_file = "data/acoustics/sp8072-figure14-apparent-source-v1.toml"
# apparent_source_curve_file_sha256 = "5c768b2ce628c07ad2cc8e2a1a6df99e893318ce79757c0675e77bc79f1b6000"
# Optional SP-8072 Figure 13 axial spectrum curve. May pair with inline
# [[acoustics.dsm.source_line]], source_curve_file, or deck_file declarations;
# omit when apparent_source_curve_file is present.
# axial_spectrum_curve_file = "data/acoustics/sp8072-figure13-axial-spectrum-v1.toml"
# axial_spectrum_curve_file_sha256 = "41d90c99e53afe77d1d4503d7bf17df627341d4ac26323ec2a32f601faa318da"
# Optional SP-8072 Figure 10 directivity curve. May pair with inline
# [[acoustics.dsm.source_line]], source_curve_file, or
# apparent_source_curve_file; omit when deck_file is present.
# directivity_curve_file = "data/acoustics/sp8072-figure10-directivity-v1.toml"
# directivity_curve_file_sha256 = "b374017177dae2cb5028ae7d096d253e9fd300665d05f33dd82d1ce2aaad7894"
# Optional SP-8072 Figure 5 spectrum curve. May pair with inline source,
# source_curve_file, or deck_file declarations.
# spectrum_curve_file = "data/acoustics/sp8072-figure5-spectrum-v1.toml"
# spectrum_curve_file_sha256 = "0d5c9ed47b94e835562f13e71d3ace4207b5a56b0991bfdbf0b9ebed11bb8b19"
# Optional per-band air-absorption deck. When present, it overrides the scalar
# absorption_db_per_m fallback.
# absorption_deck_file = "data/acoustics/dsm-air-absorption-v1.toml"
# absorption_deck_file_sha256 = "cda1136c9a9f935ac576fbe06671f00d8e0d1ed56bbb5d9c2cd42dc34847af48"
# Optional named profile from the absorption deck.
# absorption_profile_id = "humid_pad_303k_70rh"
# Optional per-band pad suppression deck. When present, select one class.
# suppression_deck_file = "data/acoustics/pad-suppression-v1.toml"
# suppression_deck_file_sha256 = "4888aeb8dde08333638d7d1127d864ea544a1a2ab78e4eabd95ad3d0b42cf932"
# suppression_class = "water_nominal"
receiver_axial_m = 5.0
receiver_radial_m = 30.0
receiver_azimuth_deg = 0.0
mass_flow_kg_s = 10.0
# Optional identical-engine cluster multiplier; exit fields are per engine.
# cluster_engine_count = 4
# Optional cluster interaction correction. Requires cluster_engine_count or
# [[acoustics.dsm.cluster_engines]].
# cluster_correlation_coefficient = 0.0
# cluster_shielding_loss_db = 0.0
# Optional per-band cluster interaction deck. Use this instead of scalar
# cluster_correlation_coefficient/cluster_shielding_loss_db.
# cluster_interaction_deck_file = "data/acoustics/pad-cluster-interaction-v1.toml"
# cluster_interaction_deck_file_sha256 = "301aaf420d9231834a89962184b8e57e9a3117a704f556f61a4eb1ab0c836b37"
# cluster_interaction_class = "coherent_shielded"
exit_velocity_m_s = 2000.0
exit_diameter_m = 1.0
acoustic_efficiency = 0.01
peak_strouhal = 0.2
width_decades = 0.5
ambient_impedance_pa_s_m = 408.0
absorption_db_per_m = 0.001
# Optional duct/trench/deflector path-length term.
# pad_path_length_extra_m = 5.0
# Optional single-bend plume-axis deflection toward the receiver radial plane.
# pad_deflection_origin_axial_m = 0.0
# pad_deflection_angle_deg = 90.0

# Optional geometric cluster. Use this instead of cluster_engine_count.
# [[acoustics.dsm.cluster_engines]]
# id = "center"
# offset_x_m = 0.0
# offset_y_m = 0.0
#
# [[acoustics.dsm.cluster_engines]]
# id = "near"
# offset_x_m = 10.0
# offset_y_m = 0.0

[[acoustics.dsm.source_line]]
axial_m = 0.0
weight = 1.0

[[acoustics.dsm.source_line]]
axial_m = 10.0
weight = 2.0

[[acoustics.dsm.liftoff_steps]]
time_s = 0.0
vehicle_height_m = 0.0

[[acoustics.dsm.liftoff_steps]]
time_s = 1.0
vehicle_height_m = 50.0
```

Two deck formats are supported. **Schema 1** is the three-axis
`(mach, alpha, beta) → (CN, CD, CM)` deck documented in
[`software-architecture.md § Deck Format`](software-architecture.md#deck-format-in-house-toml).
**Schema 2** extends Schema 1 with optional control-effector
axes; see [Schema-2 aero decks](#schema-2-aero-decks) below.
Both deck schemas may also carry optional row-major
`[dynamic_derivatives.<moment>_<rate>]` tables for non-dimensional rate
derivative lookup, where `<moment>` is `cl`, `cm`, or `cn` and `<rate>` is
`p`, `q`, or `r`. Missing tables default to zero, so legacy
`cm_q`/`cn_r`/`cl_p` decks remain valid. Rigid-body `DeckLookup` moment
evaluation consumes them when angular-rate state is available; point-mass/no-rate
evaluations keep zero-rate behavior.
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
datcom = true # optional; default false
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

[aero.buildup.protuberances] # optional; requires datcom = true
count = 2
frontal_area_m2 = 1.2e-4
drag_coefficient = 1.15
interference_factor = 1.0 # optional; default 1.0
```

The buildup uses Sutherland viscosity from the sampled reference
temperature to compute bake-time Reynolds number, includes power-off
base drag and boattail/flare effects, and produces a fixed
`(mach, alpha, beta=0) -> (CN, CD, CM)` deck. Point-mass vehicles
consume the baked deck axially at `alpha = beta = 0`; rigid-body
vehicles resolve velocity into the body frame and query the alpha/beta
axes before rotating the reduced force back to ECI.
When `datcom = true`, the bake uses the opt-in `DatcomComponentBuildup`
path with DATCOM-style area-rule forebody wave drag, fin/body carryover,
slender-body crossflow, optional equivalent protuberance drag, and bounded
principal `Cm_q`/`Cn_r`/`Cl_p` dynamic-derivative tables consumed by the
rigid-body deck moment path. The deck format can also carry the full
`Cl/Cm/Cn` by `p/q/r` derivative matrix for external calibrated decks. The
default `datcom = false` path preserves the legacy linear buildup and rejects
`[aero.buildup.protuberances]`.

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
mode                  = "quasi_static" # quasi_static | transient
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

`mode` defaults to `quasi_static`, the algebraic equilibrium `pc(Kn)` march.
`transient` emits the same `SolidMotor` surface through an explicit
lumped-volume chamber-pressure integration with deterministic ignition
transient and the reduced erosive-burn-rate hook available in the propulsion
API.

`end_burner` uses `cross_section_area_m2` and `length_m`; `bates` uses
`segments`, `outer_radius_m`, `core_radius_m`, and `segment_length_m`;
`tabulated` uses `points = [[web_m, burn_area_m2], ...]` plus
`propellant_volume_m3`. All parameters are synthetic/textbook/public and are
rejected if non-finite or outside the selected regression envelope.

Inline grain motors, feed-network engines, and liquid-engine
`thermochemical_performance` opt-ins may also declare a pinned Schema-1
thermochemistry deck. The runner resolves the deck through the same
referenced-file SHA-256 path as aero and motor files, samples it at the
declared nominal chamber pressure and mixture ratio, and uses the returned
state for reduced propulsion constants. Inline grains consume `c_star_m_s`
and `gamma`; feed networks consume `c_star_m_s`, chamber temperature, and
the molecular-weight-derived gas constant; liquid-engine performance consumes
ideal `c_star_m_s`, optional per-state `c_star_efficiency = { min, nominal,
max }`, and `gamma` plus per-engine nozzle geometry to derive max thrust,
nominal choked mass flow, and the effective-`Isp` envelope. When the block is omitted, the inline
`[propulsion.motor.grain.propellant]`, `[[propulsion.feed_network]]`, and
constant-`Isp` liquid-engine limits are preserved.

```toml
[propulsion.thermochem]
file = "../../data/thermochem/synthetic/schema-1.toml"
file_sha256 = "<64 hex chars>"
chamber_pressure_pa = 2000000.0
mixture_ratio = 2.5

# inside each [[state]] row of the referenced deck:
# c_star_efficiency = { min = 0.96, nominal = 0.98, max = 1.0 }
```

`[propulsion.nozzle]` is an optional runtime override for the single
solid-motor path:

```toml
[propulsion.nozzle]
ambient_pressure_correction = "pressure_thrust" # constant | pressure_thrust
```

`constant` preserves the motor thrust curve exactly. `pressure_thrust`
treats the stored curve as momentum thrust and adds `(p_e - p_a) A_e`
from the runner's sampled atmosphere pressure. Inline grain motors carry
the required throat area and gas `gamma`; motor-file users must provide
`geometry.throat_area_m2` and `geometry.gamma` before opting in.

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

### Day-of-launch wind block

`[day_of_launch.wind]` is a v3-only offline declaration for measured,
spliced, or synthetic day-of-launch wind inputs. It does not change the
default runner wind model by itself. `openbmp trajopt dol-redesign` consumes
validated `kind = "spliced"` declarations to produce deterministic DOL
I-load updates. The `trajopt dol-verify-run` and `trajopt dol-verify-sil`
commands can bind those updated I-load bytes to verifier telemetry/report-card
artifacts, and `openbmp-trajopt::dol` includes a synthetic RSS persistence
knockdown substrate, a synthetic geometric launch-COLA screening substrate,
and a `trajopt dol-commit-readiness` file-fed CLI latch over those reports;
operational LCC/scenario integration remains separate DOL work.

```toml
openbmp.scenario = 3

[day_of_launch.wind]
kind = "spliced" # measured | spliced | synthetic_family

[day_of_launch.wind.qc]
min_altitude_m = 0.0
max_altitude_m = 20_000.0
max_gap_m = 2_000.0
max_shear_per_s = 0.02
max_speed_m_s = 100.0
min_quality_unit = 0.8
max_staleness_s = 18_000.0

[day_of_launch.wind.filter]
cutoff_wavelength_m = 2_000.0

[day_of_launch.wind.splice]
lower_profile_id = "balloon-1"
upper_profile_id = "profiler-1"
overlap_lower_altitude_m = 1_000.0
overlap_upper_altitude_m = 2_000.0

[[day_of_launch.wind.profiles]]
id = "balloon-1"
source = "balloon"
released_at_s = -18_000.0
provenance_id = "synthetic-balloon"
provenance_uri = "data/wind/synthetic-balloon-family-v1.toml"
samples = [
  { altitude_m = 0.0, wind_ned_m_s = [5.0, 0.0, 0.0], quality_unit = 1.0 },
  { altitude_m = 1_000.0, wind_ned_m_s = [7.0, 1.0, 0.0], quality_unit = 1.0 },
  { altitude_m = 2_000.0, wind_ned_m_s = [9.0, 1.5, 0.0], quality_unit = 1.0 },
]
```

`kind = "measured"` requires one or more `[[day_of_launch.wind.profiles]]`
rows and rejects `splice` and `synthetic_family`. `kind = "spliced"`
requires at least two profiles plus `[day_of_launch.wind.splice]`; splice
profile ids must reference declared profile rows and the overlap band must be
strictly increasing. `kind = "synthetic_family"` requires
`[day_of_launch.wind.synthetic_family]` and rejects inline profiles:

```toml
[day_of_launch.wind]
kind = "synthetic_family"

[day_of_launch.wind.synthetic_family]
seed = 12345
member_count = 16
released_at_s = -18_000.0
altitude_min_m = 0.0
altitude_max_m = 20_000.0
altitude_step_m = 500.0
base_wind_ned_m_s = [4.0, -2.0, 0.0]
shear_ned_per_s = [0.0012, 0.0004, 0.0]
amplitude_ned_m_s = [6.0, 3.0, 0.4]
```

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
specific_force_source = "finite_difference"

[sensors.imu.high_rate]
sub_samples = 4
delta_theta_lsb_rad = 1.0e-6
delta_v_lsb_m_s = 1.0e-5

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

[sensors.airdata]
kind = "airdata"
file = "../../data/sensors/airdata-textbook.toml"
```

`kind = "ideal_state"` carries no noise budget and rejects `file`;
all other kinds require `file`. The base set is `imu` /
`barometer`; the further kinds are `gnss` (IS-GPS-200
receiver-output noise: per-axis Gaussian on position + velocity
plus an OU position-bias drift), `magnetometer` (per-axis
Gaussian on the body-frame WMM 2025 truth field plus constant
3×3 soft-iron and 3-vector hard-iron biases), `star_tracker`
(small-angle Gaussian rotation-vector perturbation per axis), and
`airdata` (compressible Pitot-static impact pressure, calibrated
airspeed, pressure altitude, and alpha/beta vanes from runner-supplied
air-relative truth).

For IMUs, `specific_force_source` is optional and defaults to
`"finite_difference"` for byte continuity. Schema v3 also accepts
`"force_accumulator"`, which makes the FC bridge fill IMU truth from
the runner's non-gravity force accumulator and rigid-body moment
accumulator, and `"stationary_rotating_frame"`, a validation source
that fills body-frame accelerometer and gyro truth from the closed-form
stationary rotating-frame oracle. Non-IMU sensors reject
`specific_force_source`.

`[sensors.<imu>.high_rate]` is optional and IMU-only. It enables
deterministic high-rate strapdown increment-window generation inside
the synthetic IMU while preserving the existing `sensor.imu` averaged
sample output. The direct FC bus path publishes those windows as
`sensor.imu_increments`, and bridge/PIL packet transport carries them in
protocol v4 `imu_increments` frames. `sub_samples` is a count per IMU sample
and must be positive; `delta_theta_lsb_rad` and `delta_v_lsb_m_s` are
non-negative quantization LSBs, with `0` disabling the corresponding
quantization channel. Non-IMU sensors reject `high_rate`.

### Force model registry

Force-model names accepted in `[forces].models`:

| Name | Source | Notes |
|---|---|---|
| `gravity` | `openbmp-physics` | Constant, point-mass, or J2 (selected by `[environment].gravity`) |
| `aero` | `openbmp-aero` | Requires `[aero]` block with `deck` or `buildup` |
| `thrust` | `openbmp-propulsion` | Requires `[propulsion.motor]` block |
| `contact` | `openbmp-contact` | Requires schema-v3 `[contact]`; disables the legacy terminal ground-impact stop |

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

### Contact block

`[contact]` is schema-v3 only and opts a scenario into compliant
half-space contact. The initial runner wiring evaluates a point or
sphere attached to the vehicle state position against the fixed plane
`z = ground_altitude_m`. When `[forces]` is omitted, declaring
`[contact]` derives `contact` into the force stack after `gravity`.
When `[forces]` is explicit, `models` must include `"contact"`.

```toml
[forces]
models = ["gravity", "contact"]

[contact]
kind = "half_space"
ground_altitude_m = 0.0
geometry = "sphere"        # "point" | "sphere"
radius_m = 0.25            # required only for sphere
normal_law = "kelvin_voigt" # "kelvin_voigt" | "hertz" | "hunt_crossley"
stiffness_n_m = 1000.0
damping_n_s_m = 0.0
friction_law = "regularized_coulomb" # default; or "anchored_stiction"
friction_coefficient = 0.1
friction_regularization_speed_m_s = 0.01
effective_mass_kg = 1.0
substeps = 32

[[contact.mechanism]]
id = "deploy-lower-stop"
coordinate = "gear.deploy_angle_rad"
kind = "stop"       # "stop" | "backlash" | "latch"
side = "lower"     # stop only: "lower" | "upper"
limit_rad = 0.0
stiffness_n_m = 1000.0
damping_n_s_m = 2.0

[[contact.mechanism]]
id = "drive-backlash"
coordinate = "gear.drive_angle_rad"
kind = "backlash"
center_rad = 0.0
dead_zone_width_rad = 0.05
stiffness_n_m = 500.0

[[contact.mechanism]]
id = "lock-latch"
coordinate = "gear.lock_angle_rad"
kind = "latch"
center_rad = 1.0
half_width_rad = 0.01
```

For `normal_law = "hertz"`, use `stiffness_n_m_3_2` plus
`stability_stiffness_n_m`. For `normal_law = "hunt_crossley"`, use
`stiffness_n_m_3_2`, `stability_stiffness_n_m`, and either
`damping_factor_s_m` or both `restitution` and
`reference_impact_speed_m_s`. Scenario validation fails closed when
`dt_s / substeps` violates the contact stability bound derived from
`stability_stiffness_n_m` or Kelvin-Voigt `stiffness_n_m` and
`effective_mass_kg`. The runner also uses `substeps` operationally: when
`[contact]` is present, the simulation kernel advances at
`time.dt_s / contact.substeps`, so contact force, diagnostics, and
`RunOutcome.contact` samples are produced at fixed substep boundaries.
For `friction_law = "anchored_stiction"`, replace
`friction_coefficient` and `friction_regularization_speed_m_s` with
`static_friction_coefficient`, `kinetic_friction_coefficient`,
`tangential_stiffness_n_m`, optional `tangential_damping_n_s_m`, and
`restick_speed_m_s`. The runner routes this through a time-gated anchor state
so repeated RK-stage force evaluations at the same timestamp do not advance
the tangential anchor twice.

`[[contact.mechanism]]` entries are schema-level scalar mechanism fixtures for
angular stops, backlash, and engage-once latches. They are validated and traced
under `[contact]`, but they are not yet force-coupled into doc-`01` tree joints
or CFE loop constraints.

Contact remains off by default. With `[contact]` present, the point-mass
and rigid-body runners disable the default `GroundImpact` stop and
publish the standard per-force telemetry channels
`force.contact.{x,y,z}_n` plus scalar diagnostics
`contact.gap_m`, `contact.penetration_m`,
`contact.normal_velocity_m_s`, and `contact.normal_force_n`. Library callers
also receive `RunOutcome.contact`, an out-of-band run report with the final
diagnostics, max penetration, max normal force, sample count, and a closed
`NoContact` / `Rest` / `Unsettled` classification. The report also carries a
run-level contact energy audit: initial/final elastic energy, contact work on
the vehicle, damping/friction dissipation, signed closure error, and relative
closure error. The report is not part of canonical telemetry bytes.

### Hash pinning

Every external file referenced by the scenario can carry an optional
`*_sha256` companion field (`aero.deck_sha256`,
`propulsion.motor.file_sha256`, `sensors.<name>.file_sha256`,
`epoch.eop_sha256`, `afts.keep_inside[*].evidence_file_sha256`,
`environment.egm2008_coefficients_file_sha256`,
`environment.ephemeris_file_sha256`, or ordered
`environment.ephemeris_files_sha256`,
`environment.ephemeris_meta_kernel_sha256`, and
`environment.ephemeris_meta_kernel_files_sha256`). When present, the
parser computes the file's SHA-256 digest at load time and fails closed
on mismatch. The plural ephemeris pin lists must have the same length as
their resolved file lists. When absent, the digest is still computed and
surfaced by `openbmp check`; recording those digests in telemetry
headers is handled runner-side.

`openbmp check` surfaces resolved digests in its output:

```
openbmp check: ok — niskanen-2009-chapter6 (Checked)
  telemetry.output.parquet -> .../out/niskanen-2009-chapter6.parquet
  aero.deck -> .../data/aero/...toml (sha256:cd862c2af98a1f28dc86c6e754d311c7a724081ca91b80704ad89b2ec4cb5c27)
  propulsion.motor.file -> .../data/motors/estes-c6-eng-derived.toml (sha256:da8272d3a7a135046c614e51b279971d37cac376f7aaaffdedc3ccc14d50ad4e)
  environment.egm2008_coefficients_file -> .../data/gravity/egm2008-degree10-normalized-icgem-v1.gfc (sha256:...)
  environment.ephemeris_file -> .../data/ephemeris/de440s.bsp (sha256:...)
  environment.ephemeris_files[1] -> .../data/ephemeris/mission-overlay.bsp (sha256:...)
  environment.ephemeris_meta_kernel -> .../data/ephemeris/mission.tm (sha256:...)
  environment.ephemeris_meta_kernel.files[0] -> .../data/ephemeris/de440s.bsp (sha256:...)
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
| `at_body_altitude_descending` | `body: string`, `altitude_m: f64` | Rigid-body `[multi_body]` only. Fires when the named body lane crosses down through `altitude_m`. The trigger is false until the named body is an active propagated lane. |
| `at_apogee` | — | Fires when vertical velocity flips from `> 0` to `<= 0`. |
| `at_mass_fraction` | `remaining: f64` (in `[0, 1]`) | Fires when mass fraction (current / initial) drops to or below `remaining`. |
| `at_velocity` | `velocity_m_s: f64`, optional `falling: bool = false` | Fires when speed magnitude crosses `velocity_m_s`; `falling = false` selects the rising edge and `falling = true` selects the falling edge. |
| `at_dynamic_pressure` | `pressure_pa: f64 >= 0`, `falling: bool` | Fires when dynamic pressure crosses `pressure_pa` in the direction set by `falling`. Runner event evaluation computes `q = 0.5 * rho * |v|^2` from the selected runtime atmosphere; use `us_standard_1976`, `piecewise_exponential`, `nrlmsise00`, or `nrlmsis2_compat`. |
| `at_relative_distance` | `body: string`, `distance_m: f64 > 0`, optional `reference_body: string`, optional `falling: bool = false` | Rigid-body `[multi_body]` only. Fires when the range between `body` and `reference_body` crosses `distance_m`; if `reference_body` is omitted, the current primary lane is used. The trigger is false until the named bodies are active propagated lanes. |
| `at_relative_speed` | `body: string`, `speed_m_s: f64 > 0`, optional `reference_body: string`, optional `falling: bool = false` | Rigid-body `[multi_body]` only. Fires when the relative speed between `body` and `reference_body` crosses `speed_m_s`; if `reference_body` is omitted, the current primary lane is used. The trigger is false until the named bodies are active propagated lanes. |

Flight-controller-owned missions reject `at_relative_distance`,
`at_relative_speed`, and `at_body_altitude_descending` at scenario validation
until onboard relative/body-lane observables are wired. Those triggers depend
on simulator truth lane maps and are allowed only for kernel-authority /
scenario-director evaluation today.

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

#### Mission action vocabulary

| `action.kind` | Required fields | Semantics |
|---|---|---|
| `enter_phase` | `phase: string` (declared phase id) | Sets the active mission phase; visible via runner-side telemetry / diagnostics. |
| `emit_telemetry_marker` | `tag: string` (snake_case) | Allocates a `bool` telemetry channel `mission.marker.<tag>`; runner writes `true` on every step the event fires, `false` on every other step. Channel allocation is alphabetical by tag for declaration-order independence. |
| `set_region_state` | `region: string`, `state: string` | HAL-portable region-state update. Bare `region = "estimator_regime"` resolves to `mission.regions.estimator_regime`; bare `state = "boost_mode"` resolves under that region. The commander publishes the updated per-region topic; the EKF uses `estimator_regime.boost_mode` to apply `fc.ekf.high_dynamics_q_scale` to position/velocity process noise. |
| `stop` | `label: string` | Halts the run with `StopReason::MissionEnded { label }`. Distinct from `EndTime` so determinism telemetry can distinguish CLI-driven stops from scenario-driven mission ends. |

`[[mission.events]]` rejects simulator-owned actions. If a mission event
declares `engine_command`, `effector_override`, `jettison_stage`,
`jettison_bodies`, `deploy_recovery`, or forced `separation`, `openbmp check`
fails with a message directing the action to `[[scenario_script.events]]`.

### Scenario script block

The optional v3 `[scenario_script]` block contains simulator-owned test
stimulus. It deliberately reuses the same trigger vocabulary as mission
events, but its actions are not HAL-portable mission data and are not linked
into `openbmp-mission`.

```toml
[[scenario_script.events]]
id      = "ignition_cmd"
trigger = { kind = "at_time", time_s = 0.05 }
action  = { kind = "engine_command", id = "engine_a",
            command = { throttle_unit = 1.0,
                        gimbal_pitch_rad = 0.0,
                        gimbal_yaw_rad = 0.0,
                        ignite = true,
                        shutdown = false } }
```

#### Scenario script action vocabulary

| `action.kind` | Required fields | Semantics |
|---|---|---|
| `effector_override` | `id: string` (declared effector id), `command: f64` (finite) | One-shot command override for the named effector on the next runner step. Resolves the declared id against the runner's effector rack via FNV-1a-64 of `vehicle.assembly.effectors.<id>`. Unknown ids are rejected by `openbmp check`. The kernel records the action; the runner drains it from the per-step fired-event queue and applies it on the next rack tick before the kernel step. Override wins over any declared `command_schedule` for that rack tick only. |
| `engine_command` | `id: string` (declared engine id), `command: { throttle_unit: f64 ∈ [0,1], gimbal_pitch_rad: f64, gimbal_yaw_rad: f64, ignite: bool, shutdown: bool }` | Per-engine command addressing a declared `[[vehicle.assembly.engines]]` by id. Resolves the declared id via FNV-1a-64 of `vehicle.assembly.engines.<id>`. Unknown ids are rejected by `openbmp check`. Kernel records; runner-side `EngineRack` drains and applies on the next rack tick before the kernel step. `ignite=true` is honoured only from `Idle`; `shutdown=true` only from `Igniting` / `Burning`. Throttle / gimbal values are clamped to engine limits at apply time. |
| `jettison_stage` | `body: string` (declared body id) | Stage-separation command addressing a declared `[[vehicle.assembly.bodies]]` by id. Requires `vehicle.kind = "rigid_body"`, exactly one matching `[[multi_body.separation]]`, no duplicate jettison of the same body, and momentum conservation when `conserve_momentum = true`. The runner executes this for fixed-step RK4 rigid-body profiles. Gravity-only profiles remain valid; aero, thrust, tanks, recovery, and effectors are allowed only when each resource declares an explicit owning body. |
| `jettison_bodies` | `bodies: [string, ...]` (declared body ids) | Batch stage-separation command. Every listed body is partitioned from the same pre-separation rigid-body state and appended as an independent lane on the same event tick. Each body requires a matching `[[multi_body.separation]]` with the same `event_id`; duplicate body ids are rejected. This is the coordinated deployment path for a bus releasing multiple RV-like bodies. |
| `deploy_recovery` | `id: string` (declared recovery id), `command: "deploy" \| "deploy_drogue" \| "deploy_main" \| "stow"` | Recovery-device command addressing a declared `[[vehicle.assembly.recovery]]` by id. Resolves via FNV-1a-64 of `vehicle.assembly.recovery.<id>`. Unknown ids and kind-incompatible commands are rejected by `openbmp check`; runner-side `RecoveryRack` drains accepted firings on the next rack tick before the kernel step. |

The reserved action `separation` is rejected at parse time with a typed
deferral error because it does not identify the departing body. The
`effector_override`, `engine_command`, `jettison_stage`,
`jettison_bodies`, and `deploy_recovery` actions are wired end-to-end
inside their documented validation envelopes. HAL-portable mission actions
such as `emit_telemetry_marker`, `set_region_state`, and `stop` are rejected
under `[[scenario_script.events]]`.

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
# `set_region_state`, `emit_telemetry_marker`, `stop`.
# Scenario-script actions (engine / effector / separation / recovery)
# belong in `[[mission.events]]` bindings, not in state action lists.
[[mission.states.on_entry]]
kind = "emit_telemetry_marker"
tag  = "first_stage_ignite"

[[mission.states.on_entry]]
kind   = "set_region_state"
region = "estimator_regime"
state  = "boost_mode"

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
(`--features "std hal" --no-default-features`) compile out this topic
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

The initial-formation example
[`scenarios/multi-body/initial-formation.toml`](../scenarios/multi-body/initial-formation.toml)
uses `[[multi_body.initial_lane]]` to start with two active rigid-body
lanes and emits a relative-distance marker without a separation event.

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

`kind.kind` is tagged on the inner `kind` field. Three variants are
currently wired; nonlinear / multi-axis / smart actuators are possible
extensions.

| `kind.kind` | Required fields | Semantics |
|---|---|---|
| `linear_actuator` | `tau_s: f64` (optional, default `0.0`) | First-order lag with rate clamp + saturation + deadband + pure-delay buffer. `tau_s = 0` collapses to a rate-clamped tracker (no lag). The latency must be an integer multiple of the scenario's `time.dt_s` (sub-`dt` latency is rejected at construction). |
| `second_order_servo` | `natural_frequency_rad_s: f64`, `damping_ratio: f64`, `max_accel_per_s2: f64`, `backlash_half_width: f64` | Finite-bandwidth second-order servo with command-side rate and acceleration limiting, backlash dead zone, saturation, pure-delay buffer, and the canonical effector fault modes. Valid anywhere `linear_actuator` is valid. |
| `direct_torque` | `axis: "roll" \| "pitch" \| "yaw"`, `effectiveness_n_m_per_rad: f64`, optional `rcs` table | v3 rigid-body effector interpreted as a body-axis torque command by the moment-model layer. By default it uses the same continuous actuator limits and command schedules as `linear_actuator`; with `rcs`, the runner quantizes the command through the legacy scalar MIB/PWPF pulse effector, an independent body-frame thruster bank, or a grouped coupled-bank allocator before the direct-torque moment adapter consumes the average output. |

`direct_torque.rcs` is optional. The legacy scalar shorthand remains valid:
`rcs = { minimum_impulse_n_s, nominal_thrust_n, pwpf = { ... } }`. The scalar
form may also declare `mode = "scalar"` explicitly. An independent physical
bank declares `mode = "bank"` plus `thrusters = [{ ... }]`; each thruster
carries `id`, `position_body_m`, `direction_body`, `minimum_impulse_n_s`,
`nominal_thrust_n`, and optional `blowdown = { initial_pressure_pa,
minimum_pressure_pa, usable_impulse_n_s, pressure_exponent = 1.0 }` and
`feed = { tank, specific_impulse_s, pressure_exponent = 1.0 }`. The bank
allocator uses deterministic declared order, computes body-axis torque from
`position_body_m x direction_body`, applies per-thruster MIB quantization,
optional thruster-local blowdown pressure scaling, and optional live tank-feed
pressure scaling, and reports equivalent direct-torque command units so
`effectiveness_n_m_per_rad` semantics stay unchanged. A
coupled physical bank declares `mode = "coupled_bank"`, `group = "<id>"`, and
the same `thrusters` list on exactly one roll, one pitch, and one yaw
`direct_torque` effector. The runner groups those effectors, allocates the
three-axis requested body torque impulse once through the shared physical bank,
then publishes equivalent per-axis direct-torque command units through the
ordinary effector snapshot path. Optional `pwpf` on scalar, bank, or
coupled-bank members carries `gain`, `time_constant_s`, `on_threshold`, and
`off_threshold` with `0 <= off_threshold < on_threshold`; coupled-bank PWPF
gates each member axis before the shared vector allocation.

`feed` is available only on physical RCS thrusters (`mode = "bank"` or
`mode = "coupled_bank"`). The named `tank` must be declared under
`[[vehicle.assembly.tanks]]`, must carry an `ullage` block, and must share the
effector body when the effector has `mounted_to`. Each step, the runner samples
the tank's isentropic ullage pressure scale, raises it by
`pressure_exponent`, multiplies that thruster's nominal thrust by the result,
and drains propellant from the tank as
`abs(actual_impulse_n_s) / (specific_impulse_s * g0)`.

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
four deterministic shapes:

| `command_schedule.kind` | Required fields | Semantics |
|---|---|---|
| `constant` | `value: f64` | Commands `value` at every step. |
| `step_at` | `time_s`, `before`, `after` (all `f64`) | Commands `before` while `t < time_s`; `after` thereafter. |
| `linear_ramp` | `start_time_s`, `end_time_s`, `start`, `end` (all `f64`) | Holds `start` while `t <= start_time_s`; ramps linearly to `end` over `[start_time_s, end_time_s]`; holds `end` thereafter. |
| `piecewise_linear` | `points = [{ time_s, value }, ...]` | Holds the first point before its time, linearly interpolates between strictly time-ordered adjacent points, and holds the final point thereafter. |

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
| `oscillatory` | `amplitude: f64` (non-negative), `frequency_hz: f64` (> 0), `phase_rad: f64` | Adds a deterministic sinusoidal offset to the incoming command before normal actuator dynamics, RCS quantization, and saturation. |

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
- `kind.kind` is a wired variant (`linear_actuator`, `second_order_servo`, or
	  v3 `direct_torque`); `direct_torque.rcs`, when present, is scalar
	  (`minimum_impulse_n_s` and `nominal_thrust_n` positive), bank mode
	  (non-empty `thrusters`, finite non-zero `direction_body`, positive MIB and
	  nominal thrust, body-axis authority matching command limits, optional
	  positive blowdown pressures/usable impulse/exponent, and optional
	  `feed` whose tank exists, declares ullage, and has positive Isp/exponent),
	  or coupled-bank mode
	  (non-empty `group`, identical thruster bank on exactly one roll, pitch, and
	  yaw member, shared mounted body, and signed authority for every member axis).
  Scalar, bank, and coupled-bank `pwpf` blocks require positive `gain`,
  `time_constant_s`, and `on_threshold` plus `0 <= off_threshold <
  on_threshold`.
- `limits.{min, max, max_rate_per_s, deadband, latency_s}` finite;
  `min < max`; `max_rate_per_s > 0`; `deadband >= 0` and
  `deadband <= (max - min)`; `latency_s >= 0`.
- `initial_position`, when present, finite and in `[min, max]`.
- `command_schedule` finite numeric fields; `linear_ramp` requires
  `start_time_s < end_time_s`; `piecewise_linear.points` must be
  non-empty with strictly increasing `time_s` entries.
- `unit`, when present, non-empty.
- `fault` when present: `jam.at` and `hardover.to` in `[min, max]`;
  `runaway.rate_per_s` finite; `reduced_rate.factor` in `[0, 1]`;
  `oscillatory.amplitude >= 0`, `oscillatory.frequency_hz > 0`, and finite
  `oscillatory.phase_rad`.
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
respond to per-engine `engine_command` scenario-script events; per-engine
`command_schedule` (effector-style scripted commands) is **not**
in scope — scripted command sequences flow through the
`scenario_script.events[*]` timeline.

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

Schema v3 liquid engines can opt into thermochemistry-derived performance by
adding a subtable to the engine. This requires `[propulsion.thermochem]`.
Only `limits.max_thrust_n` and `limits.isp_s` are replaced at runner
construction. The runner uses the deck state's nominal `c_star_efficiency`
for deterministic `Isp` and preserves the min/max mass-flow and `Isp` envelope
in the derived performance result; lifecycle, gimbal, throttle,
propellant-budget, and fault settings remain unchanged.

```toml
[vehicle.assembly.engines.thermochemical_performance]
throat_area_m2      = 0.02
exit_area_m2        = 0.24
ambient_pressure_pa = 101325.0
separation          = "off" # off | summerfield | schmucker
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

#### TVC actuator dynamics

Schema v3 engines may opt into the second-order TVC actuator plant with a
`tvc_actuator` inline table. The position limit remains
`limits.max_gimbal_rad`; the actuator block supplies bandwidth, rate, and
reflected-inertia parameters. If omitted, the engine uses the legacy gimbal
slew latch.

```toml
[[vehicle.assembly.engines]]
id                 = "main"
kind               = { kind = "liquid_engine" }
mount_point_body_m = [0.0, 0.0, 0.0]
limits             = { max_thrust_n = 5000.0, isp_s = 250.0,
                       ignition_transient_s = 0.1,
                       shutdown_transient_s = 0.1,
                       max_gimbal_rad = 0.087 }
tvc_actuator       = { natural_frequency_rad_s = 20.0,
                       damping_ratio = 0.5,
                       max_rate_rad_s = 1.0,
                       gimbal_inertia_kg_m2 = 2.0 }
```

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

An engine with a propellant budget may opt into a reduced feed-network
override through `[[propulsion.feed_network]]`. The first wired kind is
`tank_valve_chamber`, a single-fluid tank-valve-chamber equilibrium
solve from `openbmp-feedsystem`. The runner samples it each rack tick
with the declared fixed valve opening and maps the solved chamber
pressure to the existing engine feed scale as
`chamber_pressure_pa / reference_chamber_pressure_pa`. Existing
regulated/blowdown depletion handling still owns tank drain and
shutdown; depletion keeps the feed scale at zero.

```toml
[[propulsion.feed_network]]
kind = "tank_valve_chamber"
engine_id = "main"
tank_pressure_pa = 4000000.0
propellant_density_kg_m3 = 810.0
valve_area_m2 = 0.00008
valve_discharge_coefficient = 0.72
throat_area_m2 = 0.00012
c_star_m_s = 1600.0
reference_chamber_pressure_pa = 4000000.0
valve_open_fraction = 1.0
```

`engine_id` must name a declared `[[vehicle.assembly.engines]]` entry
that also declares a `propellant` sub-table. Multiple feed networks may
be declared, but each engine id may appear at most once.

For a stateful two-propellant transient chamber, use
`kind = "transient_dual_valve_chamber"`. The runner stores the chamber
pressure state in the feed-network rack, advances it once per simulation
tick with the scenario `time.dt_s`, and maps the updated pressure to the
same feed scale `chamber_pressure_pa / reference_chamber_pressure_pa`.
The oxidizer and fuel valve openings are fixed commands in this first
scenario-facing tier; controller wiring remains separate.

```toml
[[propulsion.feed_network]]
kind = "transient_dual_valve_chamber"
engine_id = "main"
oxidizer_tank_pressure_pa = 4000000.0
fuel_tank_pressure_pa = 3500000.0
oxidizer_density_kg_m3 = 810.0
fuel_density_kg_m3 = 720.0
oxidizer_valve_area_m2 = 0.00008
fuel_valve_area_m2 = 0.00004
oxidizer_valve_discharge_coefficient = 0.72
fuel_valve_discharge_coefficient = 0.68
chamber_volume_m3 = 0.08
gas_temperature_k = 3400.0
gas_constant_j_per_kg_k = 360.0
throat_area_m2 = 0.00012
c_star_m_s = 1600.0
initial_chamber_pressure_pa = 500000.0
reference_chamber_pressure_pa = 4000000.0
oxidizer_open_fraction = 1.0
fuel_open_fraction = 0.8
```

Add `controller = { ... }` to the transient block to close a deterministic
pressure/MR loop over the two valve commands. The pressure loop moves both
valves together; the mixture loop biases oxidizer and fuel in opposite
directions. Valve bounds and slew limits are enforced by `openbmp-feedsystem`.

```toml
controller = {
  target_chamber_pressure_pa = 3000000.0,
  target_mixture_ratio = 2.0,
  pressure_proportional_gain_per_pa = 0.0000001,
  pressure_integral_gain_per_pa_s = 0.0,
  pressure_integral_limit_pa_s = 10000000.0,
  mixture_proportional_gain = 0.1,
  mixture_integral_gain_per_s = 0.0,
  mixture_integral_limit_s = 10.0,
  min_open_fraction = 0.0,
  max_open_fraction = 1.0,
  max_open_fraction_slew_per_s = 10.0,
}
```

Add `oxidizer_pump = { ... }` and/or `fuel_pump = { ... }` to the transient
block to solve a generic normalized turbopump map before the run starts. The
runner adds the solved pressure rise to that leg's declared tank pressure; the
pump NPSH gate can derate the pressure rise through
`cavitation_head_multiplier`.

```toml
oxidizer_pump = {
  design_volumetric_flow_m3_per_s = 0.05,
  design_pressure_rise_pa = 6000000.0,
  design_shaft_speed_rad_per_s = 3000.0,
  fluid_density_kg_m3 = 810.0,
  design_efficiency = 0.70,
  required_npsh_m = 20.0,
  specific_speed = 0.8,
  head_coefficients = [1.2, -0.2, 0.0],
  efficiency_coefficients = [0.8, 0.4, -0.2],
  cavitation_head_multiplier = 0.25,
  operating_volumetric_flow_m3_per_s = 0.05,
  operating_shaft_speed_rad_per_s = 3000.0,
  suction_pressure_pa = 1000000.0,
  vapor_pressure_pa = 30000.0,
}
```

Add `oxidizer_line = { ... }` and/or `fuel_line = { ... }` to the
transient block to couple a frictionless fixed-grid MOC feed line into that
leg. The runner advances one line step per rack tick and applies the
downstream pressure perturbation around the leg's tank-plus-pump feed
pressure before the valve/chamber step. The line grid must be Courant-exact
with the scenario time step:
`time.dt_s = length_m / segment_count / wave_speed_m_s`.

```toml
oxidizer_line = {
  length_m = 40.0,
  wave_speed_m_s = 1000.0,
  density_kg_m3 = 810.0,
  cross_section_area_m2 = 0.01,
  segment_count = 4,
  initial_head_m = 100.0,
  initial_velocity_m_s = 2.0,
  upstream_head_m = 100.0,
  downstream_velocity_m_s = 0.0,
}
```

Standalone line topologies can be declared with `[[propulsion.lines]]`. These
lines use the same fixed-grid MOC primitive and may be referenced by transient
feed-network legs through `oxidizer_line_id = "..."` and/or
`fuel_line_id = "..."`. The runner builds one named topology, requires every
declared line to share the scenario Courant step
`time.dt_s = length_m / segment_count / wave_speed_m_s`, advances the topology
once per feed-network rack tick, and applies each referenced line's downstream
pressure perturbation around that leg's tank-plus-pump feed pressure before the
valve/chamber step. Line ids must be unique, and a leg may declare either an
inline `*_line = { ... }` block or a `*_line_id`, not both.

```toml
[[propulsion.lines]]
id = "oxidizer-main"
length_m = 40.0
wave_speed_m_s = 1000.0
density_kg_m3 = 810.0
cross_section_area_m2 = 0.01
segment_count = 4
initial_head_m = 100.0
initial_velocity_m_s = 2.0
upstream = { kind = "head", head_m = 100.0 }
downstream = { kind = "accumulator",
               reference_head_m = 100.0,
               stored_volume_delta_m3 = 0.0,
               compliance_m3_per_m = 0.00001 }
```

`upstream.kind` may be `head`, `velocity`, or `gas_source`. `downstream.kind`
may be `velocity`, `head`, `accumulator`, or `gas_accumulator`; linear
accumulator compliance must be positive, and gas source/accumulator boundaries
require positive `gas_reference_volume_m3`, `gas_precharge_pressure_pa`,
`polytropic_exponent`, and remaining gas volume.

Standalone line endpoints may be joined with zero-storage
`[[propulsion.line_junctions]]` declarations. Each junction connects at least
two unique line endpoints, all referenced line ids must exist, and an endpoint
may belong to only one junction. Connected endpoints solve a common junction
head with zero net signed flow; the per-line `upstream` / `downstream` boundary
fields remain the terminal defaults for endpoints not connected to a junction.

```toml
[[propulsion.line_junctions]]
id = "tee"
connections = [
  { line_id = "inlet-main", endpoint = "downstream" },
  { line_id = "branch-a", endpoint = "upstream" },
  { line_id = "branch-b", endpoint = "upstream" },
]
```

```toml
[[propulsion.feed_network]]
kind = "transient_dual_valve_chamber"
engine_id = "main"
# ... tank, valve, chamber, and command fields as above ...
oxidizer_line_id = "oxidizer-main"
```

`[propulsion.pogo]` runs the reduced feed-half POGO stability primitive at
runner startup. The block is optional and has no effect when omitted. It
couples one longitudinal structural mode to a first-order feed response,
attenuates the open-loop gain with accumulator compliance, and can fail the
run at build time when `require_stable = true` and the reduced verdict is
unstable. Configured runs also populate `RunOutcome.pogo` with the reduced
feed snapshot, a closed-loop eigenvalue/RHP report, a structural
sampled-Nyquist margin report, an optional pump-dispersion worst-corner report,
an optional lumped feedline-coupled eigen report, and deterministic
structural-margin CSV bytes. The sampled structural Nyquist clearance
threshold defaults to `0.1`; set
`require_structural_nyquist_clearance = true` to reject a run when the sampled
loop is at or below that declared clearance.

```toml
[propulsion.pogo]
mode_natural_frequency_rad_s = 60.0
mode_damping_ratio = 0.04
open_loop_gain_rad2_s2 = 500.0
feed_time_constant_s = 0.02
mass_flow_gain_time_s = 0.004
cavitation_compliance_m3_per_pa = 0.000000001
accumulator_compliance_m3_per_pa = 0.0
structural_nyquist_clearance_1 = 0.1
require_structural_nyquist_clearance = false
require_stable = true
```

Optional pump Cb/Mb dispersion boxes declare finite min/max bounds around the
nominal `cavitation_compliance_m3_per_pa` and `mass_flow_gain_time_s` scalars.
The block must carry a provenance pointer to a repository `provenance.md`
record; the runner carries that pointer into `RunOutcome.pogo`.

```toml
[propulsion.pogo.dispersion_box]
provenance = "data/feed_system/provenance.md#openbmp.feed_system.generic_pogo_stability.v1"
cavitation_compliance_min_m3_per_pa = 0.0000000005
cavitation_compliance_max_m3_per_pa = 0.000000004
mass_flow_gain_time_min_s = 0.002
mass_flow_gain_time_max_s = 0.006
```

Optional feedline adapters reference standalone `[[propulsion.lines]]` rows by
id and map their length, wave speed, density, and cross-section into
`RunOutcome.pogo.feedline_coupled`.

```toml
[[propulsion.lines]]
id = "oxidizer-main"
length_m = 100.0
wave_speed_m_s = 1000.0
density_kg_m3 = 1000.0
cross_section_area_m2 = 0.01
segment_count = 1
initial_head_m = 100.0
initial_velocity_m_s = 2.0
upstream = { kind = "head", head_m = 100.0 }
downstream = { kind = "velocity", velocity_m_s = 0.0 }

[propulsion.pogo.feedline]
line_ids = ["oxidizer-main"]
feedline_damping_ratio_1 = 0.02
```

Rigid-body scenarios with a `[vehicle.bending]` first-mode block can use that
structural modal data directly:

```toml
[propulsion.pogo]
mode_source = "vehicle_bending"
open_loop_gain_rad2_s2 = 500.0
feed_time_constant_s = 0.02
mass_flow_gain_time_s = 0.004
cavitation_compliance_m3_per_pa = 0.000000001
accumulator_compliance_m3_per_pa = 0.0
structural_nyquist_clearance_1 = 0.1
require_structural_nyquist_clearance = false
require_stable = true
```

When `mode_source = "vehicle_bending"`, omit
`mode_natural_frequency_rad_s` and `mode_damping_ratio`; the runner derives
them from `[vehicle.bending].frequency_hz` and
`[vehicle.bending].damping_ratio` and fails closed if no bending block is
declared.

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

Per-engine `fault` is optional and load-time only: a fault declared on
`[[vehicle.assembly.engines]]` is injected at construction and persists for the
run. Use `[propulsion.faults]` for deterministic mid-run engine fault
injection.

| `fault.kind` | Required fields | Semantics |
|---|---|---|
| `stuck` | `at_throttle: f64` (in `[0, 1]`) | Throttle stuck at `at_throttle`; engine ignores command throttle but still honours ignite / shutdown lifecycle. |
| `hard_off` | — | Engine commanded off and never restarts. Sets state to `Failed` on first step. |
| `over_thrust` | `factor: f64` (finite, `>= 0`) | Thrust scaled by `factor`. `>= 1` → over-thrust; `< 1` → under-thrust. |
| `hard_start_overpressure` | `factor: f64` (finite, `>= 1`), `duration_s: f64` (positive) | Ignition transient over-pressure. Scales thrust only while the engine is igniting and its in-state timer is within `duration_s`. |
| `cavitation_thrust_loss` | `factor: f64` (finite, in `[0, 1]`) | Pump-cavitation thrust loss. Scales thrust once injected by a cavitation trigger or explicit rule. |
| `gimbal_locked` | `pitch_rad: f64`, `yaw_rad: f64` (each in `±max_gimbal_rad`) | Gimbal frozen at the given angles regardless of command. |

Scheduled propulsion faults are declared under `[propulsion.faults]`. Each
`[[propulsion.faults.rules]]` rule injects one engine fault before the engine
rack steps at `start_step`; the injected fault then persists according to the
engine fault semantics. Rule ids must be unique, and `engine_id` must reference
a declared engine. Native SIL package runs can append the same rule shape in
memory with `FaultTarget::ScheduledEngine`; the package scenario file is not
modified, and the normal scenario parser still validates the resulting rule.

```toml
[propulsion.faults]

[[propulsion.faults.rules]]
id = "main-engine-out"
engine_id = "main"
start_step = 250
fault = { kind = "hard_off" }
```

Cavitation-triggered propulsion faults are also declared under
`[propulsion.faults]`. Each `[[propulsion.faults.cavitation_rules]]` rule
injects one engine fault the first time a configured transient feed-network
turbopump reports `NPSH_available < NPSH_required`. `leg` is
`oxidizer | fuel | any`; scenario validation requires a matching
`[[propulsion.feed_network]]` entry with the selected pump leg.

```toml
[propulsion.faults]

[[propulsion.faults.cavitation_rules]]
id = "main-ox-pump-cavitation"
engine_id = "main"
leg = "oxidizer"
fault = { kind = "cavitation_thrust_loss", factor = 0.45 }
```

Mixture-ratio runaway faults are feed-network faults, not engine faults:
`[[propulsion.faults.mixture_ratio_runaway_rules]]` targets a
`transient_dual_valve_chamber` feed network and starts a deterministic valve
command drift at `start_step`. Positive oxidizer and negative fuel rates drive
oxidizer-rich excursions; the opposite drives fuel-rich excursions. Commands are
clamped to `[0, 1]` after drift.

```toml
[propulsion.faults]

[[propulsion.faults.mixture_ratio_runaway_rules]]
id = "main-mr-runaway"
engine_id = "main"
start_step = 320
oxidizer_open_fraction_rate_per_s = 0.4
fuel_open_fraction_rate_per_s = -0.2
```

#### Lifecycle and command resolution

Engine state machine: `Idle → Igniting → Burning → Shutdown`.
`Shutdown` and `Failed` are terminal — engines do not re-ignite.

Per-step command resolution: `scenario_script.events[*].engine_command`
event firing →
runner drains and applies → engine latches `(throttle, gimbal)`
and processes `(ignite, shutdown)` lifecycle flags. Multiple
events addressing the same engine in one step are rejected by the
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
  `over_thrust.factor` non-negative; `hard_start_overpressure.factor`
  at least `1`; `hard_start_overpressure.duration_s` positive;
  `cavitation_thrust_loss.factor` in `[0, 1]`;
  `gimbal_locked.{pitch,yaw}_rad` in `±max_gimbal_rad`.
- `[propulsion.faults].mixture_ratio_runaway_rules` requires finite drift
  rates, at least one non-zero rate, and a matching
  `transient_dual_valve_chamber` feed network.
- Cross-validate `scenario_script.events[*].action.id` (when action kind
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
- Per-engine `fault` declarations are load-time only. Mid-run engine faults use
  `[propulsion.faults]` scheduled or cavitation-triggered rules.
- Only the `liquid_engine` kind ships.
- Per-engine `command_schedule` (effector-style declarative
  scripts) is out of scope; engines drive only via
  `scenario_script.events[*].action.engine_command`.

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
moving_mass              = { kind = "equivalent_pendulum", damping_ratio_zeta = 0.005, freefall_restoring = { kind = "capillary_surface_wave", surface_tension_n_m = 0.072, damping_ratio_zeta = 1.0 } }
ullage                   = { initial_pressure_pa = 2000000.0, gas_gamma = 1.4 }
pressurization           = { initial_ullage_temperature_k = 300.0, pressurant_inlet_pressure_pa = 5000000.0, pressurant_inlet_temperature_k = 320.0, pressurant_gas_constant_j_per_kg_k = 287.0, pressurant_heat_capacity_ratio = 1.4, pressurant_flow_area_m2 = 1.0e-4, pressurant_discharge_coefficient = 0.8, ullage_heat_transfer_w_per_k = 10.0, tank_wall_temperature_k = 315.0, tank_cross_section_area_m2 = 1.0, gravity_m_s2 = 9.80665 }
drain_rate_kg_per_s      = 0.0
safe_dump_rate_kg_per_s  = 0.0
safe_dump_orifice        = { discharge_coefficient = 0.8, area_m2 = 1.0e-5, backpressure_ramp = { start_time_s = 0.0, end_time_s = 1.0, start_pressure_pa = 101325.0, end_pressure_pa = 150000.0 } }
safe_dump_vent           = { discharge_coefficient = 0.8, area_m2 = 1.0e-3, gas_temperature_k = 300.0, gas_specific_gas_constant_j_kg_k = 287.0, thermal_model = "adiabatic_wall_relaxation", wall_temperature_k = 320.0, wall_thermal_relaxation_tau_s = 0.5, backpressure_source = "ambient_atmosphere", backpressure_pa = 101325.0 }
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
- `propellant` — `{ density_kg_m3, label }`. Document the source and
  applicability of non-synthetic density data in the scenario provenance.
- `initial_fill_fraction` — `[0, 1]`. Initial fluid mass is
  `geometry.volume × density × fill`.
- `moving_mass` — `kind`-tagged enum:
  - `rigid_liquid` — no slosh (rigid-liquid toy);
  - `equivalent_pendulum { damping_ratio_zeta, freefall_restoring? }` —
    Abramson cylindrical-tank antisymmetric fundamental mode. Optional
    `freefall_restoring = { kind = "capillary_surface_wave",
    surface_tension_n_m, damping_ratio_zeta }` adds the low-g capillary
    term `ω_cap² = (σ / ρ) · k³ · tanh(kh)`, where `k = 1.841 / radius`;
  - `equivalent_spring_mass { damping_ratio_zeta }` — translational
    alternative;
  - `abramson_multi_mass { damping_ratio_zeta, mode_count? }` —
    mass-correct Abramson/Dodge cylindrical fixed-mass plus N slosh-mass
    analog. `mode_count` defaults to `3` and is bounded to `1..=8`;
  - `baffled_pendulum { base_damping_ratio_zeta }` — pendulum +
    `baffle_model.damping_increment_zeta`.

#### Optional fields

- `baffle_model = { damping_increment_zeta }` — additive damping
  increment consumed only by `baffled_pendulum`: a scalar increment
  per Abramson Eq 7-46 simplified.
- `ullage = { initial_pressure_pa, gas_gamma }` — pressurant state required
  when an engine bound to this tank selects `feed = "blowdown"`.
- `pressurization = { initial_ullage_temperature_k,
  pressurant_inlet_pressure_pa, pressurant_inlet_temperature_k,
  pressurant_gas_constant_j_per_kg_k, pressurant_heat_capacity_ratio,
  pressurant_flow_area_m2, pressurant_discharge_coefficient,
  ullage_heat_transfer_w_per_k, tank_wall_temperature_k,
  tank_cross_section_area_m2, gravity_m_s2 }` — opt-in finite-volume
  ideal-gas pressurant hardware substrate. It requires `ullage`, uses the
  existing `openbmp-feedsystem` lumped pressurization model, advances with the
  current tank outflow volume rate, and supplies a previous-step/current-rack
  ullage pressure override to pressure-coupled `safe_dump_orifice` flow when
  SAFE dump is active. An active SAFE command with
  `pressurization_enabled = false` latches this path disabled. It cannot be
  co-declared with `safe_dump_vent` because both models own ullage gas state.
  This is a
  research/toy substrate; it is not a production cryogenic phase-change,
  regulator/bottle certification, or flight-calibrated pressurant-hardware
  model.
- `drain_rate_kg_per_s` — drain is decoupled from
  engine propellant budgets. Defaults to `0.0` (no scenario drain); engine
  coupling adds its own deterministic one-step-lag drain rate.
- `safe_dump_rate_kg_per_s` — separate SAFE dump path, consumed only when
  the flight controller's propellant-safing command requests dump. Defaults
  to `0.0` and does not reopen ordinary drain paths.
- `safe_dump_orifice = { discharge_coefficient, area_m2, backpressure_pa?,
  backpressure_source?, backpressure_ramp? }` — pressure-coupled SAFE dump path,
  consumed only when the flight controller's propellant-safing command requests
  dump. It requires `ullage`, computes the live pressure from the isentropic
  ullage volume ratio, applies an incompressible orifice law, and adds to
  `safe_dump_rate_kg_per_s`. `backpressure_pa` defaults to `0.0`.
  `backpressure_source` defaults to `"configured"`; `"ambient_atmosphere"` uses
  the runner environment sample's current atmosphere pressure. Optional
  `backpressure_ramp = { start_time_s, end_time_s, start_pressure_pa,
  end_pressure_pa }` overrides `backpressure_pa` at runtime using deterministic
  clamped linear interpolation over scenario elapsed time and is mutually
  exclusive with `backpressure_source = "ambient_atmosphere"`.
- `safe_dump_vent = { discharge_coefficient, area_m2, gas_temperature_k,
  gas_specific_gas_constant_j_kg_k, thermal_model?, backpressure_pa?,
  backpressure_source?, wall_temperature_k?, wall_thermal_relaxation_tau_s?,
  backpressure_ramp? }` — choked ullage-gas vent path, consumed only when the
  flight controller's propellant-safing command requests dump. It requires
  `ullage`, computes the initial gas mass from the ideal-gas relation, applies
  deterministic choked/subcritical orifice mass flow, and reduces the pressure
  available to `safe_dump_orifice` on later steps. `thermal_model` defaults to
  `"fixed_temperature"`; `"adiabatic_ullage"` updates gas temperature and
  pressure with an isentropic ullage-gas relation as gas mass and ullage volume
  change; `"adiabatic_wall_relaxation"` applies that post-vent adiabatic update
  and then relaxes the gas temperature toward `wall_temperature_k` over
  `wall_thermal_relaxation_tau_s`. `backpressure_pa` defaults to `0.0`, and
  `backpressure_source` has the same `"configured"`/`"ambient_atmosphere"`
  semantics as the orifice field. Optional `backpressure_ramp` has the same
  shape and runtime interpolation semantics as the orifice ramp and is mutually
  exclusive with `backpressure_source = "ambient_atmosphere"`.
- `initial_slosh = { angles_rad: [θ_x, θ_y], rates_rad_s: [θ̇_x,
  θ̇_y] }` — initial slosh perturbation for `equivalent_pendulum`
  and `baffled_pendulum`.
- `initial_slosh = { displacement_body_m: [x, y], velocity_body_m_s:
  [ẋ, ẏ] }` — initial slosh perturbation for
  `equivalent_spring_mass` and the first mode of `abramson_multi_mass`.

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
- `pressurization` requires `ullage`, positive finite pressure/temperature/gas
  constants/flow area/cross-section/gravity fields, heat-capacity ratio greater
  than `1`, `pressurant_discharge_coefficient` in `(0, 1]`, and non-negative
  `ullage_heat_transfer_w_per_k`. It is rejected when `safe_dump_vent` is also
  declared.
- `safe_dump_orifice` requires `ullage`, `discharge_coefficient` in `(0, 1]`,
  positive `area_m2`, non-negative `backpressure_pa`, and any
  `backpressure_ramp` to have non-negative pressures, non-negative
  `start_time_s`, and `end_time_s > start_time_s`. `backpressure_source =
  "ambient_atmosphere"` is rejected when `backpressure_ramp` is declared.
- `safe_dump_vent` requires `ullage`, `discharge_coefficient` in `(0, 1]`,
  positive `area_m2`, positive `gas_temperature_k`, positive
  `gas_specific_gas_constant_j_kg_k`, non-negative `backpressure_pa`, and an
  initial vent flow that cannot empty the ullage gas in one `time.dt_s` step.
  `thermal_model` must be `fixed_temperature`, `adiabatic_ullage`, or
  `adiabatic_wall_relaxation`; the wall-relaxed model also requires positive
  finite `wall_temperature_k` and `wall_thermal_relaxation_tau_s`, while those
  wall fields are rejected for the other models. Any `backpressure_ramp` obeys
  the same validation as the orifice ramp and is rejected with
  `backpressure_source = "ambient_atmosphere"`.
- `drain_rate_kg_per_s × time.dt_s` and
  the initial combined SAFE dump rate (`safe_dump_rate_kg_per_s` plus
  `safe_dump_orifice` at initial ullage pressure) multiplied by `time.dt_s`
  must not exceed the initial tank fluid mass; single-step emptying is rejected
  rather than silently clamping away the moving-mass dynamics.

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
- The slosh state advances using the **prior step's**
  `(specific_force_body, omega_body)` — the documented one-step lag
  that breaks the circular dependency between the tank's reaction
  force and the kernel's per-step force evaluation. The first step
  uses zeros.

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
lifting-entry equations.

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
recovery and dispersion analysis. It is not connected to the flight
controller.

```toml
[landing_footprint]
method = "egm2008"
cull_altitude_m = 0.0
geodetic_output = "range_relative_only"

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

[landing_footprint.monte_carlo.uq]
budget_toml = "uq/footprint-budget.toml"
credibility_floor = "l2"
report_md = "out/footprint-mc-credibility.md"

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
speed_sigma_m_s = 0.5
time_sigma_s = 0.01
```

Consumed methods are:

| Method | Required `environment.gravity` | Propagation |
|---|---|---|
| `constant_gravity` | `constant` | Closed-form flat constant-gravity crossing. |
| `j2` | `j2` | Fixed-step RK4 propagation under the scenario J2 gravity parameters. |
| `egm2008` | `egm2008` | Fixed-step RK4 propagation under the pinned zonal-only EGM2008 degree-2 through degree-6 model. |

Every method requires a mission phase named `coast` or
`ballistic_descent`. `geodetic_output = "reviewed_recovery_coordinates"`
additionally requires `[frames.local_origin]`; the default
`"range_relative_only"` emits only range-relative `downrange_m` /
`crossrange_m` output. The optional dispersion block is a declared
ellipse source. If absent, no dispersion ellipse is reported. Numerical
Earth-gravity methods stop at the WGS84 radial ellipsoid surface plus
`cull_altitude_m`.

`[landing_footprint.monte_carlo]` enables offline sampled dispersion. It
requires at least one declared uncertainty source under
`wind`, `ballistic_coefficient`, or `burnout_state`; a Monte-Carlo block
with no source fails closed. The runner samples deterministic streams
from the Monte-Carlo seed (or `[time].seed` when omitted) plus the sample
index, propagates each sample with drag and wind using the sampled
ballistic coefficient, and writes the declared CSV/Parquet sample cloud
plus TOML summary. Burnout-state velocity uncertainty is deliberately
limited to scalar `speed_sigma_m_s`, sampled along the nominal velocity
direction, so the scenario cannot express per-axis launch-direction
dispersion. `wind.kind` accepts `constant`, `layered`, `hwm14`, or
`ensemble`; the current offline propagator consumes the sampled local-NED
perturbation as the constant wind vector for that footprint sample.

`[landing_footprint.monte_carlo.uq]` is optional. When present,
`budget_toml` points to an `openbmp-uq` TOML budget with source-tagged
one-sigma terms and NASA-STD-7009B credibility factors;
`credibility_floor` accepts `l0` through `l4` (or `0` through `4`) and
defaults to `l0`; `report_md` optionally writes the deterministic Markdown
credibility report. `openbmp footprint-mc` fail-closes before accepting final
sample/summary evidence when the budget's binding credibility level is below
the configured floor. CLI `--uq-toml` options can still be used as a sidecar
override for ad hoc runs.

The Monte-Carlo summary includes an output-only `[dispersion_statistics]` block:
`radial_dispersion_p50_m` is the empirical 50% radial dispersion about the successful sample
mean, while `mean_radial_offset_from_nominal_m` is the radial offset from
the nominal forward footprint to that sample mean. The persisted sample cloud
contains per-sample landing and radial-offset diagnostics only: it deliberately
does not write the sampled burnout state, ballistic coefficient, or wind vector
next to each landing point. `[[quantiles]]` are mean-centered
radial-distance quantiles;
`[[nominal_radial_offset_quantiles]]` are radial-error quantiles about the
nominal forward footprint. These diagnostics are written after propagation and
do not change simulator commands.

### Synthetic rare-event Monte Carlo

`[monte_carlo]` is a schema-v3 manifest for synthetic rare-event estimator
studies in `openbmp-mc`. It is consumed by `openbmp mc rare-event`; the block is
not a vehicle risk statement, is not bound to a named vehicle or mission, and
cannot declare a ground target or aimpoint.

```toml
[monte_carlo]
seed = 777

[monte_carlo.limit_state]
kind = "synthetic_linear"
label = "synthetic-limit-state"
beta = 3.0
dimension = 4

[monte_carlo.subset_simulation]
samples_per_level = 4096
conditional_probability = 0.1
max_levels = 6
proposal_sigma = 0.8
dimension_id = 19

[monte_carlo.cross_entropy]
samples = 4096
elite_fraction = 0.1
iterations = 5
smoothing = 0.8
min_std_dev = 0.2
dimension_id = 23
```

The required `[monte_carlo.limit_state]` block accepts only
`kind = "synthetic_linear"` and `label = "synthetic-limit-state"`.
`beta` is the reliability index for the analytic linear standard-normal
limit state, and `dimension` is the standard-normal input dimension.
At least one estimator block,
`[monte_carlo.subset_simulation]` or `[monte_carlo.cross_entropy]`, must be
present. Sample counts must be at least two, probabilities and smoothing
factors must be strictly between zero and one, and sigma/std-dev controls must
be positive.

The pre-serde consumer-agreement lint rejects target-like vocabulary under
`[monte_carlo.limit_state]`, including ground-aimpoint, aimpoint, target, CEP,
impact, and miss-distance terms. Neutral uses of those words outside this
limit-state block remain governed by the normal typed schema.

Run the manifest with:

```bash
openbmp mc rare-event scenario.toml --output-toml out/rare-event-report.toml
```

The optional TOML evidence file records each estimator report, subset-simulation
threshold levels, and cross-entropy adaptation rows with deterministic numeric
formatting.

### Staging analysis

`[staging_analysis]` is a schema-v3 offline post-processing block. It runs the
ideal loss-free rocket equation and writes compact report metadata under
`openbmp.staging_analysis.*` in the telemetry schema. It is not consumed by the
flight controller.

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
runner-side recovery rack. The baseline recovery path applies drag at the body
center of gravity with no recovery moment; inflating parachutes can also opt
into scalar apparent mass as an added mass/rate contribution.

```toml
[[vehicle.assembly.recovery]]
id         = "dual_chute"
mounted_to = "lower"
kind       = { kind = "drogue_main", drogue_c_d = 1.0, drogue_area_m2 = 0.5, main_c_d = 1.5, main_area_m2 = 4.0 }
drogue_inflation = { fill_time_s = 0.5, opening_force_coeff_c_x = 1.1, growth = "polynomial", exponent_j = 1.0 }
main_inflation   = { fill_time_s = 1.5, opening_force_coeff_c_x = 1.3, growth = "polynomial", exponent_j = 2.0, apparent_mass_coeff_k_a = 0.65 }
sequence = { snatch_delta_v_m_s = 12.0, snatch_effective_mass_kg = 220.0, pack_drag_load_n = 350.0, mortar_pack_mass_kg = 8.0, mortar_pack_velocity_body_m_s = [0.0, 0.0, 18.0] }
risers = [{ id = "main_riser", unstretched_length_m = 10.0, axial_rigidity_n = 20000.0, damping_n_s_m = 50.0, vehicle_attach_body_m = [0.0, 0.5, -1.0], canopy_attach_body_m = [0.0, 0.0, 0.0] }]

[[vehicle.assembly.recovery]]
id         = "main_chute"
mounted_to = "lower"
kind       = { kind = "parachute_drag", c_d = 1.5, area_inflated_m2 = 8.0 }
inflation  = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = "polynomial", exponent_j = 2.0, apparent_mass_coeff_k_a = 0.65 }

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

Schema v3 single-stage `parachute_drag` devices can opt into an `inflation`
inline table. Two-stage `drogue_main` devices can opt into
`drogue_inflation` and/or `main_inflation` tables; the active stage selects its
own fill clock and optional load/apparent-mass parameters. The fill duration is
either declared directly with `fill_time_s` or derived from Knacke-style
line-stretch inputs:
`fill_constant_n`, `nominal_diameter_m`, `line_stretch_speed_m_s`, and optional
`fill_exponent_k` (default `1.0`). The explicit and derived fill-time forms are
mutually exclusive. `growth` is `"polynomial"` or `"exponential"`,
`exponent_j` controls polynomial `tau^j` growth, `alpha` controls exponential
growth, and `initial_drag_area_fraction` defaults to `0.0`. The runner starts
the fill clock when `deploy` fires and publishes the effective ramped area
through `recovery.<id>.drag_area_m2`; scenarios without an inflation table keep
the legacy instantaneous area swap. Inflating parachutes also publish an
infinite-mass opening-load diagnostic from `opening_force_coeff_c_x`. Optional
`apparent_mass_coeff_k_a` on a single-stage or active staged profile enables
scalar apparent mass
`m_a = k_a * rho * (pi/6) * D_p^3`; point-mass and rigid-body mass paths add
`m_a`, and the recovery force adapter applies `-(dm_a/dt) * v_air`. Here
`v_air` is the shared environment air-relative velocity
`vehicle_velocity_eci_m_s - atmosphere_velocity_eci_m_s - wind_eci_m_s`, so
declared `[wind]` affects recovery drag without a recovery-specific
`velocity_source` field.
For polynomial growth, `inflation.pflanz`, `drogue_inflation.pflanz`, or
`main_inflation.pflanz` optionally adds Pflanz `X1(A, j)` finite-mass peak-load
reduction either inline:
`{ ballistic_parameter_a = [...], exponent_j = [...], reduction_factors = [...] }`,
or from a resolved SHA-pinned TOML deck:
`{ file = "data/recovery/pflanz-x1-synthetic-v1.toml", file_sha256 = "2d679136f9f123eced753fbbbb3f6b67432be2530462b5ab2366cb239737546c" }`.
The file and inline forms are mutually exclusive.

Schema v3 parachute recovery rows with declared `risers` may also declare a
deployment `sequence` diagnostic block:

```toml
sequence = { snatch_delta_v_m_s = 12.0, snatch_effective_mass_kg = 220.0, pack_drag_load_n = 350.0, mortar_pack_mass_kg = 8.0, mortar_pack_velocity_body_m_s = [0.0, 0.0, 18.0] }
```

`snatch_delta_v_m_s` is the non-negative relative velocity term `delta_v` at
line stretch, `snatch_effective_mass_kg` is the positive effective decelerated
mass `m_e`, and `pack_drag_load_n` is an optional non-negative pack-drag term
that defaults to zero. The runner derives the equivalent line stiffness from
the declared risers as `sum(EA/L0)` and publishes the Knacke-style estimate
`F_snatch = delta_v * sqrt(k_line * m_e) + F_drag,pack` after the recovery
device is deployed.

If `mortar_pack_mass_kg` and `mortar_pack_velocity_body_m_s` are declared
together, the runner treats the pack velocity as the pack ejection velocity in
body axes and applies the equal-and-opposite vehicle reaction impulse
`-m_pack * v_pack` as a constant force over the next kernel tick after an
accepted deployment command. The direct recovery telemetry records the impulse
and equivalent force in body axes, and the force breakdown records the same
load as `force.recovery_sequence_force.*`. The two mortar fields must be
declared as a pair. This sequence surface still does not advance bag-strip
kinematics or apply the snatch load itself to the vehicle force stack.

Schema v3 parachute recovery rows can also declare `risers`, an array of
slack-taut Kelvin-Voigt line elements reserved for snatch, attach-load, and
future two-body coupling. Each riser requires a local `id`,
`unstretched_length_m` (`L0`), effective `axial_rigidity_n` (`EA`),
non-negative `damping_n_s_m`, `vehicle_attach_body_m`, and
`canopy_attach_body_m`. A riser may also declare
`canopy_velocity_body_m_s = [vx, vy, vz]` to advance that canopy/pack endpoint
at constant body-frame velocity while the recovery device is deployed; omitted
velocity defaults to zero. If `canopy_mass_kg` is present, that same velocity is
the initial endpoint velocity and the runner force-integrates the canopy/pack
endpoint from the line force after slack-taut engagement; `canopy_mass_kg` must
be positive. A recovery row may instead declare a shared translated canopy/pack
state with
`riser_canopy = { initial_position_body_m = [x, y, z], initial_velocity_body_m_s = [vx, vy, vz], mass_kg = m }`.
When `riser_canopy` is present, each `canopy_attach_body_m` is a fixed offset
from the shared canopy origin, all declared risers feed their summed line force
into the one shared mass, and per-riser `canopy_velocity_body_m_s` /
`canopy_mass_kg` fields are rejected. The shared `mass_kg` must be positive;
the shared velocity defaults to zero when omitted. The vehicle-layer element evaluates
`T = max(0, EA/L0 * delta + c * delta_dot)` with exactly zero force while
`length < L0`, exposes a fixed-tick slack-to-taut crossing locator for linearly
moving endpoints, and exposes the attach-point moment helper `r x F`. The runner
evaluates declared risers in body coordinates while the recovery device is
deployed, advances prescribed-velocity endpoints on each rack step when no mass
is declared, force-integrates mass-declared endpoints from the line force, can
force-integrate all risers through one shared translated canopy state, and
publishes aggregate tension, force, moment, and elastic-energy diagnostics; the
rigid-body recovery-riser moment adapter consumes the
aggregate force and moment snapshot, shifts the moment to the live CG, and
contributes the result as the `recovery_riser` moment component. The
recovery-riser force adapter also consumes the aggregate body-frame force as a separate
`recovery_riser_force` force component: point-mass scenarios treat the body
frame as aligned with ECI, while rigid-body scenarios rotate it through the
live attitude. For live runner snapshots with declared risers, the rack also
adds the device's aerodynamic `C_D A` drag load into the riser force/moment
totals at the declared attach geometry and marks the snapshot so
`recovery_drag` skips that same scalar drag term. For multiple declared risers,
the drag handoff is distributed across per-riser attach points by declared-line
tension; if all declared risers are slack, the fallback is an equal split in
declaration order.

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

When a row declares `inflation`, `drogue_inflation`, or `main_inflation`, the
runner allocates these additional channels for that device only. For
`drogue_main`, the values describe the currently active inflating stage.

| Channel | Type | Units | Notes |
|---|---|---|---|
| `recovery.<id>.inflation_progress` | float64 | 1 | Fill progress, clamped to `[0, 1]` |
| `recovery.<id>.inflation_complete` | bool | bool | `true` after the declared fill time has elapsed |
| `recovery.<id>.opening_dynamic_pressure_pa` | float64 | Pa | Dynamic pressure used by the infinite-mass load diagnostic |
| `recovery.<id>.opening_load_n` | float64 | N | Current infinite-mass opening-load diagnostic |
| `recovery.<id>.opening_peak_load_n` | float64 | N | Full-area infinite-mass peak opening-load diagnostic |

When any declared inflation table carries `apparent_mass_coeff_k_a`, the runner
allocates these additional channels for that device:

| Channel | Type | Units | Notes |
|---|---|---|---|
| `recovery.<id>.apparent_mass_kg` | float64 | kg | Scalar canopy apparent mass |
| `recovery.<id>.apparent_mass_rate_kg_s` | float64 | kg/s | Scalar apparent-mass growth rate |

When any declared inflation table carries `pflanz`, the runner allocates these
additional diagnostic channels for that device:

| Channel | Type | Units | Notes |
|---|---|---|---|
| `recovery.<id>.opening_ballistic_parameter_a` | float64 | 1 | Runtime Pflanz ballistic parameter |
| `recovery.<id>.opening_pflanz_reduction_factor_x1` | float64 | 1 | Interpolated finite-mass reduction factor |
| `recovery.<id>.opening_finite_mass_peak_load_n` | float64 | N | Full-area finite-mass peak opening-load diagnostic |

When a recovery row declares `sequence`, the runner allocates these additional
diagnostic channels for that device:

| Channel | Type | Units | Notes |
|---|---|---|---|
| `recovery.<id>.snatch_delta_v_m_s` | float64 | m/s | Declared relative velocity term |
| `recovery.<id>.snatch_stiffness_n_m` | float64 | N/m | Equivalent riser stiffness `sum(EA/L0)` |
| `recovery.<id>.snatch_effective_mass_kg` | float64 | kg | Declared effective decelerated mass |
| `recovery.<id>.snatch_pack_drag_load_n` | float64 | N | Declared pack-drag load term |
| `recovery.<id>.snatch_peak_load_n` | float64 | N | `delta_v * sqrt(k_line * m_e) + F_drag,pack` |

When the `sequence` block also declares mortar pack mass and velocity, the
runner allocates these one-tick impulse channels:

| Channel | Type | Units | Notes |
|---|---|---|---|
| `recovery.<id>.mortar_impulse_body_x_n_s` | float64 | N*s | Vehicle reaction impulse, body X |
| `recovery.<id>.mortar_impulse_body_y_n_s` | float64 | N*s | Vehicle reaction impulse, body Y |
| `recovery.<id>.mortar_impulse_body_z_n_s` | float64 | N*s | Vehicle reaction impulse, body Z |
| `recovery.<id>.mortar_force_body_x_n` | float64 | N | One-tick equivalent force, body X |
| `recovery.<id>.mortar_force_body_y_n` | float64 | N | One-tick equivalent force, body Y |
| `recovery.<id>.mortar_force_body_z_n` | float64 | N | One-tick equivalent force, body Z |

When a recovery row declares one or more `risers`, the runner allocates these
aggregate diagnostic channels for that device:

| Channel | Type | Units | Notes |
|---|---|---|---|
| `recovery.<id>.riser_tension_sum_n` | float64 | N | Sum of declared riser tensions |
| `recovery.<id>.riser_tension_max_n` | float64 | N | Maximum declared riser tension |
| `recovery.<id>.riser_force_body_x_n` | float64 | N | Aggregate anchor force, body X |
| `recovery.<id>.riser_force_body_y_n` | float64 | N | Aggregate anchor force, body Y |
| `recovery.<id>.riser_force_body_z_n` | float64 | N | Aggregate anchor force, body Z |
| `recovery.<id>.riser_moment_body_x_n_m` | float64 | N*m | Aggregate attach-point moment, body X |
| `recovery.<id>.riser_moment_body_y_n_m` | float64 | N*m | Aggregate attach-point moment, body Y |
| `recovery.<id>.riser_moment_body_z_n_m` | float64 | N*m | Aggregate attach-point moment, body Z |
| `recovery.<id>.riser_elastic_energy_j` | float64 | J | Sum of elastic potential energy in declared risers |

The force breakdown also allocates `force.recovery_riser_force.x_n`,
`.y_n`, and `.z_n` when any recovery row declares risers. It allocates
`force.recovery_sequence_force.x_n`, `.y_n`, and `.z_n` when any sequence block
declares mortar pack mass and velocity.

The recovery force adapter sums deployed `C_D A` terms and apparent-mass-rate
terms in scenario-declared recovery order. It skips a device's scalar `C_D A`
drag term when that snapshot has transmitted the same load through declared
risers, avoiding double-counting between `recovery_drag` and
`recovery_riser_force`. It short-circuits before atmosphere sampling when both
sums are zero; if only apparent-mass rate is active, it applies the
`-(dm_a/dt) * v_air` term without a density query.

#### Limitations

- Drag opposes air-relative velocity expressed in ECI components, matching
  the existing axial-drag adapter; wind-relative recovery drag comes from the
  shared environment sample, not a recovery-specific switch.
- Production or digitized real Pflanz table calibration and real-table
  integrated Pflanz validation remain future recovery-parity work.
- `sequence` currently covers the snatch-load estimate and a discrete
  one-tick mortar reaction impulse. Bag-strip/line-stretch state,
  force-applied snatch transients, reefing/disreef stages, and cutter-time
  dispersions remain future recovery-parity work.
- Declared risers are validated, publish runner diagnostics, and contribute a
  separate aggregate line-force component plus a rigid-body moment component;
  for live snapshots they partially replace the scalar aerodynamic `C_D A`
  drag term with a line-transmitted attach load. This is still not a full
  two-body canopy/vehicle model: the handoff uses declared static line shares,
  and the runner's current shared canopy path is a translated point-mass state,
  not a 6-DOF canopy body in the multibody tree or a two-body solve. Scalar
  apparent mass contributes only through the opt-in mass and force-rate paths
  above.

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
primary_body_id = "bus"

[[multi_body.initial_lane]]
body_id                         = "observer"
position_eci_m                  = [25.0, 0.0, 1000.0]
velocity_eci_m_s                = [0.0, 1.0, 0.0]
quaternion_body_to_eci_xyzw     = [0.0, 0.0, 0.0, 1.0]
angular_velocity_body_rad_s     = [0.0, 0.0, 0.0]

[[multi_body.separation]]
event_id              = "fairing_separation"
upper_body_id         = "bus"
lower_body_id         = "lower_stage"
lower_weld_translation_upper_body_m = [0.0, 0.0, -2.0]
lower_weld_quaternion_upper_to_lower_xyzw = [0.0, 0.0, 0.0, 1.0]
upper_delta_v_body_m_s = [0.0, 0.0, 0.5]
lower_delta_v_body_m_s = [0.0, 0.0, -0.5]
conserve_momentum     = true

[[multi_body.welded_body]]
parent_body_id = "lower_stage"
child_body_id = "payload"
child_weld_translation_parent_body_m = [0.0, 0.0, 1.5]
child_weld_quaternion_parent_to_child_xyzw = [0.0, 0.0, 0.0, 1.0]
```

`primary_body_id` and `[[multi_body.initial_lane]]` declare rigid-body
lanes that are active from simulation start. The main vehicle
`[vehicle]` initial position, velocity, attitude, and angular rate
belong to `primary_body_id`; each initial lane supplies its own
inertial state and body-to-ECI attitude. The runner computes each
lane's mass properties from its owned assembly body and routes
per-body resources exactly as it does after separation. Initial-lane
telemetry uses the same `body.<body_id>.*` channel family as detached
bodies; `body.<body_id>.separated` is `true` from the first row for
these lanes.

`propagation_authority` defaults to `"rigid_kernel"`. Setting it to
`"primary_root_free_flyer"` is a guarded root-only handoff mode: it
requires `primary_body_id`, rejects `[[multi_body.initial_lane]]`,
`[[multi_body.separation]]`, `[[multi_body.landing_controller]]`, and
`[[multi_body.gimbal_joint]]`, and after each fixed-step RK4 tick replaces the
primary rigid state with the root-free-flyer multibody forecast recorded for
that same tick. Setting it to `"separated_root_free_flyer"` is a separated-lane
handoff mode: it requires at least one `[[multi_body.initial_lane]]` or
`[[multi_body.separation]]`, rejects `[[multi_body.gimbal_joint]]`, and after
each fixed-step RK4 tick replaces already-active separated lanes with their
same-tick root-free-flyer multibody forecasts. Lanes created by a jettison on
the current tick are mirrored for the next tick; they are not retroactively
rewritten before a pre-step forecast exists. Setting it to
`"welded_release_jettison"` is a guarded jettison handoff mode: it
requires `primary_body_id` and at least one `[[multi_body.separation]]`, rejects
`[[multi_body.initial_lane]]` and `[[multi_body.gimbal_joint]]`, computes the
departing lower body or bodies by releasing configured welded upper/lower
multibody pairs, including declared dry welded descendants, with
`release_welded_subtree_as_free_flyer()`, and installs the continuing and
departing same-time rigid states through the explicit separation-state kernel
API. This mode supports `jettison_stage` and `jettison_bodies`, and applies the
declared welded translation/orientation, dry descendant mass-property
aggregation, separation delta-v, angular-rate tip-off, lower attitude offset,
and mass-property COM-offset semantics after the welded release. Setting it to
`"articulated_gimbal_root"` is a primary-body articulated-gimbal root-state
handoff mode: it requires `primary_body_id` and one or more
`[[multi_body.gimbal_joint]]` declarations whose `body_id` matches
`primary_body_id`, rejects initial lanes, separation declarations, and landing
controllers, and after each fixed-step RK4 tick replaces the primary rigid
state with the projected root state from the articulated gimbal forecast. For
multiple gimbals, the runner builds one coupled free-flyer tree with all
declared primary-body gimbal children and uses that coupled root forecast for
the handoff. Once the first authoritative root handoff has occurred, the next
pre-step articulated seed carries the prior forecast's internal joint
coordinates and rates forward while refreshing the root slots from the
authoritative rigid state. CFE-constrained root handoff remains limited to the
single-gimbal path.

Each `[[multi_body.separation]]` entry binds to a mission event by id
and declares the two `vehicle.assembly.bodies[*].id` values that
continue propagating after the event.
`lower_weld_translation_upper_body_m` and
`lower_weld_quaternion_upper_to_lower_xyzw` optionally declare the fixed welded
pose of the departing body relative to the continuing upper body before release;
they default to zero translation and identity orientation. Optional
`[[multi_body.welded_body]]` entries declare dry welded descendants under a
departing lower body with `parent_body_id`, `child_body_id`,
`child_weld_translation_parent_body_m`, and
`child_weld_quaternion_parent_to_child_xyzw`; the runner preserves their dry
mass in the departing lane and rejects descendants that own active runtime
resources. Optional impulsive delta-V fields apply at the separation moment.
`conserve_momentum` defaults to `true`; the loader verifies
`m_u·Δv_u + m_l·Δv_l ≈ 0` to a documented tolerance, where `m_l` includes
declared dry welded descendants of the departing lower body.

The runtime consumer supports fixed-step RK4 rigid-body profiles.
Initial and post-separation force-stack ownership is explicit:

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

Each lane evaluates only the aero, thrust, engine, tank, recovery,
effector, snapshot, and mass-property resources owned by that lane's
active body. Ambiguous ownership fails at scenario load or at the
model capability gate; the composite force stack is not silently reused
for an active body lane. Telemetry includes the continuing primary lane
in the existing state channels and each non-primary lane in
`body.<body_id>.*` channels, including `body.<body_id>.separated`.
Relative range and speed triggers can observe initial or detached
lanes with `trigger.kind = "at_relative_distance"` or
`trigger.kind = "at_relative_speed"` once the referenced bodies are
active. They remain simulator/kernel-authority triggers; `[fc]`
missions reject them until FC relative navigation exists.

Separated lanes may also declare a simulator-side attitude target that
drives direct-torque effectors owned by that lane:

```toml
[[multi_body.attitude_target]]
body_id = "lower_stage"
start_time_s = 147.0
end_time_s = 214.0
body_axis_body = [0.0, 0.0, 1.0]
roll_effector = "lower-roll-trim"
pitch_effector = "lower-pitch-trim"
yaw_effector = "lower-yaw-trim"
kp = 0.8
kd = 2.0
max_command = 1.0
target = { kind = "surface_relative_axes", radial = 0.0, downrange = -1.0, crossrange = 0.0, downrange_axis_eci = [0.0, 1.0, 0.0] }
```

`body_id` must name an assembly body that can become an active
separated lane. Each listed effector must be `kind = "direct_torque"`,
mounted to that same body, and match the declared roll / pitch / yaw
axis. `body_axis_body` defaults to body +Z. The target provider is
either `kind = "eci_vector"` with a non-zero `vector_eci`, or
`kind = "surface_relative_axes"` with non-zero radial / downrange /
crossrange components and a `downrange_axis_eci` that is not parallel
to local radial. This controller is a simulator diagnostic for
post-separation attitude dynamics; it is not recovered flight data.

Separated lanes may also declare a simulator-side terminal landing
controller that commands one engine owned by that lane:

```toml
[[multi_body.landing_controller]]
body_id                   = "lower_stage"
engine_id                 = "landing"
start_altitude_m          = 10000.0
target_altitude_m         = 0.0
target_vertical_speed_m_s = -5.0
target_position_eci_m     = [6370000.0, 0.0, 0.0] # optional
lateral_kp_s2             = 0.00002
lateral_kd_s              = 0.05
max_lateral_accel_m_s2    = 3.0
gravity_margin_m_s2       = 9.80665
min_throttle_unit         = 0.0
max_throttle_unit         = 1.0
```

The controller becomes active at or below `start_altitude_m`, estimates the
radial braking acceleration needed to meet `target_vertical_speed_m_s` at
`target_altitude_m`, adds `gravity_margin_m_s2`, clamps the result to the
declared throttle bounds, and emits one-tick engine commands through the
normal engine rack. If `target_position_eci_m` is present, the controller also
adds tangent-plane lateral feedback (`lateral_kp_s2`, `lateral_kd_s`,
`max_lateral_accel_m_s2`) and emits gimbal commands toward the combined
vertical/lateral acceleration vector. `body_id` must name a body that can
become an active separated lane. `engine_id` must name an engine whose
`vehicle.assembly.engines[*].mounted_to` owner is the same body. The
controller is a deterministic scenario-director aid for SIL evidence; it is
not flight software and does not represent a fielded propulsive-landing
algorithm.

Scenarios may also declare articulated engine gimbal joints for the
runner-side multibody tree materializer:

```toml
[[multi_body.gimbal_joint]]
body_id                    = "core"
engine_id                  = "center"
axis_body                  = [0.0, 1.0, 0.0]
secondary_axis_body        = [1.0, 0.0, 0.0] # optional two-axis gimbal frame
pivot_body_m               = [0.0, 0.0, -16.0]
engine_mass_kg             = 115.0
engine_cg_body_m           = [0.0, 0.0, -0.4]
engine_inertia_body_kg_m2  = [[9.0, 0.0, 0.0], [0.0, 17.0, 0.0], [0.0, 0.0, 14.0]]
thrust_application_body_m  = [0.2, 0.0, -0.4]
neutral_thrust_body        = [0.0, 0.0, 1.0]
mechanism_id               = "center-pitch-stop" # optional [contact] scalar mechanism
mechanism_axis             = "primary"           # optional: primary | secondary
initial_angle_rad          = 0.0
initial_rate_rad_s         = 0.0
```

`body_id` must name a declared assembly body, and `engine_id` must name an
engine whose `vehicle.assembly.engines[*].mounted_to` owner is that body. The
axis must be finite and non-zero; the pivot is expressed in the parent body
frame; `engine_mass_kg`, `engine_cg_body_m`, and
`engine_inertia_body_kg_m2` supply the inertial engine body because the
existing thrust engine entry has no dry-mass contract.
`thrust_application_body_m` is optional, defaults to the gimbal-frame origin,
and gives the child-frame point where the runner applies the live
`EngineSnapshot::thrust_body` vector when evaluating the articulated shadow.
`neutral_thrust_body` is optional, defaults to child-frame `+z`, must be
finite, non-zero, and not parallel to `axis_body`, and is used to derive the
one-axis revolute coordinate from a nonzero live thrust vector. The stored
coordinate follows the multibody child-from-parent sign convention, so it may
have the opposite sign of a propulsion pitch/yaw command depending on the
chosen axis.
`mechanism_id` is optional. When present, it must name a
`[[contact.mechanism]]` entry whose `coordinate` equals this gimbal
`engine_id`; the rigid runner evaluates that scalar stop/backlash/latch
mechanism against the selected gimbal coordinate and rate, then injects the
resulting generalized force into that articulated joint. `mechanism_axis`
defaults to `primary`; `secondary` is valid only when `mechanism_id` is present
and the gimbal declares `secondary_axis_body`. In single-gimbal
`articulated_gimbal_root` authority mode, the same mechanism force participates
in the authoritative root forecast and already-engaged monotone latch state is
carried with the articulated internal state. When a later delivered live gimbal
deflection changes after an authoritative handoff, the runner refreshes the
articulated coordinate before mechanism evaluation, using delivered TVC
gimbal angle/rate snapshots directly for standard-axis gimbals, live thrust
direction for powered nonstandard-axis layouts, and a snapshot-derived unit
thrust direction for zero-thrust nonstandard-axis layouts. A latch centered on
that live deflection can therefore engage even without thrust. Kernel-facing TVC
load-torque snapshots are projected onto the declared gimbal axes as generalized
joint force during articulated derivative/forecast evaluation, and TVC
inertial-reaction snapshots are applied as a root-body external moment. Validated
full tail-wags-dog benchmark closure and coupled-acceleration term closure remain
future work.
`secondary_axis_body` is optional. When present, the parser treats the
declaration as a two-axis articulated gimbal shadow: `axis_body`,
`secondary_axis_body`, and `neutral_thrust_body` must be mutually orthogonal,
and `secondary_axis_body` must align with
`axis_body x neutral_thrust_body`. The runner materializes a massless
intermediate gimbal frame for the secondary axis and stores joint coordinates
in secondary-axis then primary-axis order.
The runner materializes each declaration during rigid-body session preparation
as a root free-flyer plus one or two revolute articulated bodies, validates its
CRBA inertia matrix, and refreshes a non-authoritative ABA derivative from the
live engine snapshot before each rigid-body tick. It also stores a
non-authoritative one-step RK4 forecast over the articulated gimbal tree under
the held live engine snapshot and projects the forecast root slots back into a
rigid-state shape for future propagation handoff checks. The current rigid
force/moment path remains authoritative until the WP-01.2 propagation handoff
lands.

Scenarios may declare row-space CFE loop-constraint fixtures for the
multibody/contact assembly boundary:

```toml
[[multi_body.cfe_loop]]
loop_id = "gear-side-brace-x"
position_residual = 0.0
velocity_residual_per_s = 0.0
velocity_bias_acceleration_per_s2 = 0.0 # optional

[[multi_body.cfe_loop.jacobian_entry]]
generalized_velocity_index = 6
jacobian_value = 1.0

[[multi_body.cfe_loop.jacobian_entry]]
generalized_velocity_index = 7
jacobian_value = -0.25
```

Each `[[multi_body.cfe_loop]]` entry declares one scalar loop row. `loop_id`
must be unique. `position_residual`, `velocity_residual_per_s`, and
`velocity_bias_acceleration_per_s2` are row-native residual and bias terms;
their physical units depend on the concrete loop coordinate model. The
`[[multi_body.cfe_loop.jacobian_entry]]` rows form a sparse Jacobian row by
generalized-velocity column index. Column indices must be unique within one loop
row, each `jacobian_value` must be finite, and the assembled row must have
non-zero finite norm. The rigid runner currently consumes these rows only for a
single scenario-declared articulated gimbal shadow: it validates each column
against that tree's generalized-velocity dimension, assembles
`G*qdd_free + Gdot*qdot` and `G*H(q)^-1*G^T`, solves the Baumgarte CFE system,
and injects the projected generalized force into the non-authoritative shadow
derivative and forecast. Rigid-body telemetry includes one channel triplet per
declared loop:
`multi_body.cfe_loop.<loop_id>.lagrange_multiplier`,
`multi_body.cfe_loop.<loop_id>.stabilized_acceleration_per_s2`, and
`multi_body.cfe_loop.<loop_id>.acceleration_residual_per_s2`, with `loop_id`
sanitized by the normal telemetry suffix rules. If
`[multi_body].propagation_authority = "articulated_gimbal_root"`, the same
CFE-constrained forecast is the guarded single-gimbal root handoff source.
Concrete scenario-declared vehicle loop-closure wiring beyond the current
articulated-gimbal shadow path, broader tree-joint authority, full
rigid-kernel replacement, and published CFE/ADAMS benchmark scenarios remain
future work.

For the currently supported articulated-gimbal CFE path, scenarios may also
declare a concrete coordinate lock instead of a precomputed row-space fixture:

```toml
[[multi_body.cfe_gimbal_lock]]
loop_id = "main-engine-primary-lock"
engine_id = "main_engine"
axis = "primary"                 # optional, defaults to "primary"; "secondary" requires a two-axis gimbal
locked_angle_rad = 0.0
velocity_bias_acceleration_per_s2 = 0.0 # optional
```

Each `[[multi_body.cfe_gimbal_lock]]` entry references one
`[[multi_body.gimbal_joint]]` by `engine_id`. The parser validates finite lock
targets and velocity-bias accelerations, a unique `loop_id` across row-space
and concrete CFE declarations, a known articulated gimbal engine, and secondary
axis use only on two-axis gimbals. During runner evaluation the lock materializes
to one CFE row from the live articulated state: position residual
`q_axis - locked_angle_rad`, velocity residual `qd_axis`, and a unit Jacobian
entry in the selected generalized-velocity slot. It uses the same solve,
projection, telemetry, and guarded `articulated_gimbal_root` handoff path as
`[[multi_body.cfe_loop]]`.

Two-axis articulated gimbal shadows may also declare a concrete planar
four-bar or side-brace row:

```toml
[[multi_body.cfe_planar_four_bar]]
loop_id = "main-engine-four-bar"
engine_id = "main_engine"
input_axis = "secondary" # optional default
output_axis = "primary"  # optional default
ground_pivot_a_m = [0.0, 0.0]
ground_pivot_d_m = [1.0, 0.0]
input_link_m = 0.5
coupler_link_m = 1.0
output_link_m = 0.5
velocity_bias_acceleration_per_s2 = 0.0 # optional
```

Each `[[multi_body.cfe_planar_four_bar]]` entry references one two-axis
`[[multi_body.gimbal_joint]]` by `engine_id`. `loop_id` shares the same CFE
namespace and telemetry suffix checks as row-space loops and gimbal locks.
`input_axis` and `output_axis` must be distinct, and the fixed pivots plus link
lengths must pass the `openbmp-contact::PlanarFourBarLoop` reachability check.
During runner evaluation the helper materializes the row from the live selected
gimbal coordinates and rates: position residual is the moving-endpoint distance
minus `coupler_link_m`, velocity residual is `J * qd`, and the two-column
Jacobian is projected into the selected generalized-velocity slots. The row
then uses the same solve, projection, telemetry, and guarded
`articulated_gimbal_root` path as `[[multi_body.cfe_loop]]`. This is current
articulated-gimbal-shadow reachability for the four-bar row model; prismatic
landing-gear DOFs and general vehicle tree-joint authority remain future work.

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
or `best_by_covariance_trace`. The covariance-trace voter selects the healthy
lane with the smallest exposed position/velocity/attitude covariance diagonal
trace; if a lane exposes no covariance diagonal block, it falls back to the
innovation chi-square proxy. Lane ids must be unique. The consumer wires
parallel filter instances to the controller's pub/sub bus and selects the
active lane each tick. Multi-lane estimators publish `estimator.lane_selection`;
FDIR latches
`FDIR_BIT_ESTIMATOR_LANE_FAILOVER` when the active lane moves away from
lane 0 or when all lanes fail.

#### `[fc.autopilot_allocation]` — control allocation

```toml
[fc.autopilot_allocation]
kind          = "prioritised_redistributed"
axis_priority = ["roll", "yaw", "pitch"]
```

`kind` is one of `pseudo_inverse` (Stevens & Lewis 2015 §3.5),
`prioritised_redistributed` (Härkegård 2002), `dense_wls`, or `dense_inca`.
The first two are consumed for the current `direct_torque` effector surface.
`pseudo_inverse` uses
a bounded weighted pseudo-inverse per body axis. `axis_priority` applies to
`prioritised_redistributed`, lists body-frame axes in highest-first order, and
must contain exactly `"roll"`, `"pitch"`, and `"yaw"` once each.

The allocator derives capacity only from `direct_torque`
effectors with exact symmetric limits (`max == -min`). Phase authority is
applied before the proportional split, so disallowed effectors receive
zero commands and do not contribute capacity.

`dense_wls` declares explicit dense effectivity columns in fixed `[roll, pitch,
yaw]` row order and can use asymmetric bounds/rate limits. `dense_inca` uses the
same `dense_effectors` block, but consumes `autopilot.pseudo_control_cmd`
desired/measured angular-acceleration pseudo-control from the INDI rate loop and
therefore requires the `indi` Cargo feature in the runner. Each `id` must name a
declared `vehicle.assembly.effectors` entry or a physical engine-gimbal alias of
the form `<engine_id>.gimbal_pitch` / `<engine_id>.gimbal_yaw`. When `min`,
`max`, or `rate_limit` is omitted for a declared effector, the runner uses that
effector's existing `limits.min`, `limits.max`, and `limits.max_rate_per_s`.
When those fields are omitted for an engine-gimbal alias, the runner uses
`-limits.max_gimbal_rad`, `limits.max_gimbal_rad`, and
`limits.gimbal_slew_rad_per_s` from the referenced engine. `gamma` defaults to
`1.0e6`, `virtual_weights` defaults to `[1, 1, 1]`, `effort_weight` defaults to
`1`, and `preferred` defaults to `0`.

```toml
[fc.autopilot_allocation]
kind = "dense_wls"
gamma = 1000000.0
virtual_weights = [1.0, 1.0, 1.0]

[[fc.autopilot_allocation.dense_effectors]]
id = "left-tvc"
effectiveness = [0.4, 1.0, 0.2]
min = -0.35
max = 0.35
rate_limit = 20.0

[[fc.autopilot_allocation.dense_effectors]]
id = "right-tvc"
effectiveness = [-0.4, 1.0, -0.2]
effort_weight = 1.2

[[fc.autopilot_allocation.dense_effectors]]
id = "main_engine.gimbal_pitch"
effectiveness = [0.0, -1.0, 0.0]
```

#### `[[fc.engine_out_trim]]` — scalar engine-out trim biases

```toml
[[fc.engine_out_trim]]
failed_engine = "center"

[[fc.engine_out_trim.commands]]
engine = "left"
gimbal_pitch_bias_rad = 0.005
gimbal_yaw_bias_rad = -0.010

[[fc.engine_out_trim.commands]]
engine = "right"
gimbal_pitch_bias_rad = -0.010
gimbal_yaw_bias_rad = 0.015
```

`[[fc.engine_out_trim]]` is v3-only and applies to the scalar `EngineDemand`
fan-out path after engine availability has removed failed engines. Each row
names the `failed_engine` that activates it and one or more surviving-engine
`commands`. When the failed engine is unavailable, the mixer keeps the
engine-out throttle redistribution from the scalar fan-out path and adds each
declared `gimbal_pitch_bias_rad` / `gimbal_yaw_bias_rad` to the matching
surviving engine command. Failed engines still receive zero throttle, zero
gimbal, and shutdown. Trim is not applied to shutdown commands.

All referenced engines must be declared in `[[vehicle.assembly.engines]]`.
Rows must have unique `failed_engine` values; each row must contain at least
one command; command engines must be unique within a row and cannot target the
failed engine. This is the manual calibrated table path; it cannot be declared
with `[fc.engine_out_trim_auto]`.

#### `[fc.engine_out_trim_auto]` — geometry-derived scalar engine-out trim biases

```toml
[fc.engine_out_trim_auto]
max_bias_rad = 0.05 # optional
```

`[fc.engine_out_trim_auto]` is v3-only and derives an `EngineOutTrimTable` for
the scalar `EngineDemand` fan-out path from `[[vehicle.assembly.engines]]`
geometry. It cannot be declared with manual `[[fc.engine_out_trim]]` rows. When
enabled, the runner computes one row per single failed engine using declared
engine `mount_point_body_m`, `limits.max_thrust_n`, and `limits.max_gimbal_rad`.
The derivation compares the nominal axial-thrust moment with the moment after
scalar survivor throttle retrim, then solves a regularized small-angle TVC
least-squares system for survivor pitch/yaw trim biases. Biases are clamped by
each engine's `limits.max_gimbal_rad` and by `max_bias_rad` when provided.

This is a first-order geometry-derived scalar fan-out substrate. It does not
estimate flight-calibrated integrator bias, live mass properties, thrust
calibration, a guidance target, or closed-loop engine-out recoverability.

#### `[fc.transport]` — opt-in FC bridge transport

```toml
[fc.transport]
mode = "in_process" # "in_process" | "tcp_loopback" | "unix_loopback" | "external_process"
command = "path/to/fc-peer" # required only for mode = "external_process"
args = ["--bridge-stdio"] # optional, external_process only
working_dir = "." # optional, external_process only
max_payload_len = 4096 # optional, bytes
peer_protocol_version = 3 # optional; normally omit
```

`[fc.transport]` is v3-only. When absent, `[fc]` uses the legacy direct
in-process bridge and existing telemetry is unchanged. `mode = "in_process"`
routes the FC boundary through `openbmp-bridge` `BridgeMessage` packets over an
in-process endpoint pair; `mode = "tcp_loopback"` uses the same local simulator
and controller endpoints over a framed TCP loopback stream; `mode = "unix_loopback"`
uses a temporary Unix-domain socket on Unix platforms. `mode = "external_process"`
spawns `command` with `args`, couples the bridge over child stdin/stdout, and
kills the child if it is still running when the session drops. All transport
modes validate the bridge hello before exchanging sensor and actuator frames.

The current runner wiring is deliberately lossless: opt-in transport scenarios
must include an IMU and may also carry GNSS position/velocity/bias, barometer
pressure/bias, Pitot-static air-data, magnetometer nT/hard-iron state, and
star-tracker attitude through the bridge packet. Effector commands and full engine throttle, gimbal,
ignition, and shutdown commands are carried in the actuator packet.
The runner reports an actuator command-stream SHA-256 digest outside canonical
telemetry so direct, in-process, and TCP-loopback paths can be compared without
changing telemetry bytes. `max_payload_len`, when present, must be greater than
zero. `peer_protocol_version` is a negative-test hook for protocol-mismatch
scenarios; omit it for normal runs.

#### `[fc.transport_faults]` — bridge-level signal transforms

```toml
[fc.transport]
mode = "in_process"

[[fc.transport_faults.rules]]
id         = "imu-x-bias"
start_step = 100
end_step   = 250 # optional, inclusive
signal     = { kind = "imu_accel_body_mps2", axis = "x" }
transform  = { kind = "additive_bias", offset = 0.25 }

[[fc.transport_faults.rules]]
id         = "engine-throttle-limit"
start_step = 150
signal     = { kind = "engine_throttle", engine_id = 0 }
transform  = { kind = "saturate", min = 0.0, max = 0.5 }

[[fc.transport_faults.rules]]
id         = "baro-pressure-drift"
start_step = 200
signal     = { kind = "baro_pressure_pa" }
transform  = { kind = "drift", rate_per_s = 2.0, reference_time_s = 0.0 }

[[fc.transport_faults.rules]]
id         = "gyro-noise-burst"
start_step = 250
end_step   = 300
signal     = { kind = "imu_gyro_body_rad_s", axis = "z" }
transform  = { kind = "noise_burst", amplitude = 0.01, seed = 42 }

[[fc.transport_faults.rules]]
id         = "imu-increment-dtheta-y-bias"
start_step = 260
signal     = { kind = "imu_increment_delta_theta_rad", sample_index = 1, axis = "y" }
transform  = { kind = "additive_bias", offset = 1.0e-6 }

[[fc.transport_faults.packet_rules]]
id         = "drop-first-sensor-frame"
start_step = 0
end_step   = 0
direction  = "sensor"
transform  = { kind = "drop" }

[[fc.transport_faults.packet_rules]]
id         = "duplicate-first-command-frame"
start_step = 0
direction  = "command"
transform  = { kind = "duplicate" }

[[fc.transport_faults.packet_rules]]
id         = "delay-sensor-frame"
start_step = 12
direction  = "sensor"
transform  = { kind = "delay", steps = 2 }

[[fc.transport_faults.packet_rules]]
id         = "bit-flip-command-frame"
start_step = 15
direction  = "command"
transform  = { kind = "bit_flip", mask = 5 }

[[fc.transport_faults.packet_rules]]
id         = "skew-command-step"
start_step = 20
direction  = "command"
transform  = { kind = "step_offset", offset = 1 }

[[fc.transport_faults.packet_rules]]
id         = "skew-command-time"
start_step = 25
direction  = "command"
transform  = { kind = "time_offset", offset_s = 0.001 }
```

`[fc.transport_faults]` is schema-v3 and requires `[fc.transport]`. Rules are
evaluated in declaration order while `start_step <= step <= end_step` (or with
no upper bound when `end_step` is omitted). The runner translates these rules
to `openbmp-bridge` scalar transforms at the FC transport boundary: sensor
rules mutate `SensorPacket` fields before the controller consumes them, and
command rules mutate `ActuatorCommandPacket` fields before the simulator
applies them.

High-rate IMU increment signals use zero-based `sample_index` inside the
bridge packet's `imu_increments` window. Supported increment signals are
`imu_increment_delta_theta_rad`, `imu_increment_delta_v_m_s`, and
`imu_increment_dt_s`; absent sample indices are skipped the same way absent
optional sensor channels are skipped.

Supported transforms are `additive_bias`, `scale`, `stuck`, `saturate`,
`reverse_sign`, deterministic scalar `quantize`, deterministic time-ramp
`drift`, and deterministic bounded `noise_burst`. Numeric transform parameters
must be finite; `saturate` also requires `min <= max`, `quantize` requires a
positive `quantum`, and `noise_burst` requires a positive `amplitude`.
`noise_burst` draws from a rule-local deterministic bridge-fault RNG stream
keyed by `seed`, step, signal, and rule id, so adding a sibling fault does not
shift its draws. Supported vector axes are `x`, `y`, and `z`; star-tracker
quaternion signals also accept `w`.

Packet-level `packet_rules` currently support `direction = "sensor"` or
`"command"` with `drop`, `duplicate`, `delay`, `bit_flip`, `step_offset`, or
`time_offset` transforms. Delay transforms require a positive `steps` count,
bit-flip transforms require a non-zero `mask`, `step_offset` requires a
non-zero signed `offset`, and `time_offset` requires a finite non-zero
`offset_s`. The current runner treats configured packet drop, duplicate, delay,
or bit-flip dispositions as fail-closed lockstep behavior and rejects command
step/time skew through the bridge lockstep validator.

These are abstract message-level signal transforms. They do not model any
specific sensor product, actuator hardware, electrical fault, or bus protocol.

#### `[fc.fdir.detector]` — FDIR detector tuning

```toml
[fc.fdir.detector]
kind             = "windowed_mean_shift_glrt"
window_samples   = 32
parity_threshold = 25.0
```

The `detector_kind` field on `[fc.fdir]` drives
the burst / single-sample-GLRT / CUSUM detectors; this
sub-block adds tuning data for the windowed-mean-shift GLRT
(Willsky 1976) and Patton-Frank parity-space residual generator.

#### `[fc.fdir.redlines]` — FDIR watchpoints

```toml
[fc.fdir.redlines]
body_rate_rad_s = 4.0
```

Redlines are v3-only FDIR I-load watchpoints. `body_rate_rad_s`
sets a body-rate magnitude limit; exceeding it causes the FDIR job to
latch the body-rate redline fault bit. Omit the block to disable these
watchpoints.

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

#### `[fc.autopilot_params.load_relief]`

```toml
[fc.autopilot_params.load_relief]
a0 = 1.0
a1 = 0.2
g_alpha_max = 0.8
g_acc = 0.01
q_alpha_limit_pa_rad = 1200.0

[[fc.autopilot_params.load_relief.blend_schedule]]
schedule_value = 0.0
blend = 0.0

[[fc.autopilot_params.load_relief.blend_schedule]]
schedule_value = 20.0
blend = 1.0
```

`[fc.autopilot_params.load_relief]` is v3-only. The runner maps it
to `AutopilotParams::load_relief`, which is off by default. When
enabled, the three-loop autopilot estimates alpha/beta from
`PositionEstimate`, `EnvironmentEstimate`, and the optional
`estimator.wind` topic published by the FC bridge, clamps alpha and
beta by `q_alpha_limit_pa_rad`, and adds the resulting pitch/yaw
correction before the attitude loop.

`blend_schedule` is declared as
`[[fc.autopilot_params.load_relief.blend_schedule]]` entries. Each node
has a finite `schedule_value` and a `blend` in `[0, 1]`; schedule values
must be strictly ascending. The schema intentionally has no
target/aimpoint/range fields, and unknown fields are rejected at load
time.

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
zonal coefficients in source. v2 scenarios that name `egm2008` fail
closed with a schema-version diagnostic.

Schema v3 can opt into the current coefficient-file transition path:

```toml
[environment]
frame_profile = "wgs84-uniform-rotation" # or another rotating Earth frame
gravity = "egm2008"
egm2008_coefficients_file = "data/gravity/egm2008-degree10-normalized-icgem-v1.gfc"
egm2008_coefficients_file_sha256 = "<64 hex chars>" # optional
egm2008_degree = 10
egm2008_order = 10
# Or use one named tier instead of egm2008_degree / egm2008_order:
# egm2008_tier = "degree70" # "degree120" or "degree360"
egm2008_finite_difference_step_m = 10.0 # optional; defaults to 10 m
```

The runner resolves and optionally SHA-pins the ICGEM `.gfc` text,
parses the source `earth_gravity_constant` and `radius`, strips the
central `Cbar00` term, builds
`openbmp_physics::FiniteDifferencePinesGravity`, and wraps it in
`EarthFixedGravity` so the body-fixed harmonic field is evaluated through
the selected frame. The current checked-in runtime fixture is the
provenance-pinned clipped EGM2008 degree-10/order-10 block. Named
`egm2008_tier` requests fail closed unless the source file declares enough
degree/order coverage. The full degree-70/120/360 analytic Pines/Gottlieb
kernel and NGA HARMONIC_SYNTH
acceptance tables remain deferred.

#### `gravity = "tesseral"`

```toml
[environment]
frame_profile = "wgs84-uniform-rotation" # or "iers-cio" with [epoch] inputs
gravity = "tesseral"
mu_m3_s2 = 3.986004418e14
r_e_m = 6378137.0
tesseral_degree = 2
tesseral_order = 2
tesseral_c20 = -1.082626683e-3
tesseral_c21 = 0.0
tesseral_s21 = 0.0
tesseral_c22 = 1.0e-7
tesseral_s22 = 0.0
tesseral_tide_system = "tide_free" # optional; defaults to tide_free
```

Selects the current bounded degree-2 Earth-fixed tesseral/sectoral
gravity path. The runner builds `openbmp_physics::TesseralGravity`,
wraps it in `EarthFixedGravity`, and evaluates it through the scenario
`FrameContext`. The current evaluator accepts only `tesseral_degree = 0`
or `2`, and `tesseral_order <= degree <= 2`; higher-degree Pines/Gottlieb
runtime synthesis remains deferred.

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

### Schema Scope

- v3 keeps concrete real-device integrations out of the upstream scenario
  schema. Real bus protocols and device drivers remain downstream-owned
  integration work.
- The determinism CI gate continues to assert byte-identical Parquet
  for the existing v2 scenario set across this schema bump.
