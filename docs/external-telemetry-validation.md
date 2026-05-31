# External Telemetry Validation

OpenBMP can compare a scenario run against local telemetry CSV or JSON that
is kept outside the repository. This is intended as a quarantined diagnostic
workflow: use external observations to expose simulator, environment,
controller, or event-model bugs, then fix the underlying code and keep the
committed scenarios synthetic.

This workflow must not be used to commit recovered operational telemetry,
fielded vehicle parameter sets, fitted aero decks, fitted controller gains,
or mission constants derived from a real flight into OpenBMP.

## Command

```bash
openbmp compare-telemetry \
  scenarios/phalcon9/phalcon9-orbit.toml \
  ../openbmp-private-data/reference-flight.csv \
  --mapping ../openbmp-private-data/reference-flight-map.toml \
  --report-json ../openbmp-private-data/reference-flight-report.json
```

The reference file is read locally. OpenBMP does not copy it, hash it into
scenario provenance, or write it to a project output directory. The mapping
file may live outside the repository alongside the reference data. The
optional `--report-json` output is a local machine-readable comparison
summary for batch diagnostics, including `passed`, `failure_summary`, and
per-metric error fields. It is written before the command returns a failed
comparison status. JSON
references may be an object of column arrays:

```json
{
  "time": [0.0, 1.0],
  "altitude": [0.0, 0.002],
  "velocity": [0.0, 2.832]
}
```

or an array of row objects:

```json
[
  { "time": 0.0, "altitude": 0.0, "velocity": 0.0 },
  { "time": 1.0, "altitude": 0.002, "velocity": 2.832 }
]
```

JSONL / NDJSON references use the same row-object shape, one object per line:

```jsonl
{ "time": 0.0, "altitude": 0.0, "velocity": 0.0 }
{ "time": 1.0, "altitude": 0.002, "velocity": 2.832 }
```

## Mapping Format

```toml
[reference]
time_column = "time_s"

[comparison]
time_tolerance_s = 0.25

[[metrics]]
id = "altitude"
reference_column = "altitude_km"
reference_scale = 1000.0
tolerance_abs = 15000.0
tolerance_rel = 0.05
actual = { kind = "altitude_from_position", radius_m = 6371000.0 }

[[metrics]]
id = "speed"
reference_column = "speed_m_s"
tolerance_abs = 250.0
tolerance_rel = 0.03
actual = { kind = "surface_relative_speed" }

[[metrics]]
id = "thrust_norm"
reference_column = "thrust_kn"
reference_scale = 1000.0
tolerance_abs = 500000.0
tolerance_rel = 0.10
actual = { kind = "norm3", x = "force.thrust.x_n", y = "force.thrust.y_n", z = "force.thrust.z_n" }
```

Supported `actual.kind` values:

| Kind | Purpose |
|---|---|
| `channel` | Directly compare one OpenBMP float telemetry channel: `actual = { kind = "channel", name = "..." }`. |
| `altitude_from_position` | Compare `sqrt(x^2+y^2+z^2) - radius_m`; defaults to `position_x_m`, `position_y_m`, `position_z_m`. |
| `speed_from_velocity` | Compare `sqrt(vx^2+vy^2+vz^2)`; defaults to `velocity_x_m_s`, `velocity_y_m_s`, `velocity_z_m_s`. |
| `surface_relative_speed` | Compare `|v_eci - omega x r_eci|` for Earth-rotating webcast velocity; defaults to WGS84 `omega_rad_s = 7.2921151467e-5` and the standard position/velocity channels. |
| `surface_relative_axis_velocity` | Compare signed `(v_eci - omega x r_eci) dot axis_eci`, with `axis_eci = [x, y, z]`. |
| `surface_relative_local_axis_velocity` | Compare signed surface-relative velocity along `axis_eci` after projecting that axis into the local horizontal plane for geocentric states. |
| `surface_relative_horizontal_speed` | Compare local-horizontal speed from `v_eci - omega x r_eci`; useful when recovered telemetry reports the horizontal component magnitude rather than a fixed inertial-axis component. |
| `surface_relative_radial_velocity` | Compare signed radial velocity from `v_eci - omega x r_eci`; geocentric states use `r_hat`, local-frame states use `+z`. |
| `norm3` | Compare a generic vector norm from three named float telemetry channels. |

Each metric uses the envelope
`max(tolerance_abs, tolerance_rel * max(abs(reference), relative_floor))`.
`relative_floor` defaults to `1.0`.

## How To Use The Result

Good uses:

- Identify inconsistent gravity, atmosphere, aerodynamic, propulsion,
  event-timing, staging, mass-accounting, estimator, or autopilot behavior.
- Add first-principles tests after finding a bug: conservation checks,
  analytic toy cases, model cross-checks, or synthetic regression tests.
- Compare multiple synthetic scenarios against the same broad observed
  envelope to find algorithm sensitivity.

Bad uses:

- Fitting scenario constants until the external curve matches.
- Importing the reference data, mapping, or derived fitted parameters into
  the repository.
- Treating one real flight as a golden trajectory for a committed in-tree
  scenario.

The intended loop is: compare externally, diagnose a physics or controller
defect, fix the defect in general code, then validate committed synthetic
scenarios without depending on the external data.
