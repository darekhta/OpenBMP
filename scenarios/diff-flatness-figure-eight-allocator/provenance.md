# Diff-flatness figure-eight allocator scenario provenance

## Scenario

`scenarios/diff-flatness-figure-eight-allocator/scenario.toml`

## Source class

Synthetic. Identical to `scenarios/diff-flatness-figure-eight`
modulo two changes:

1. The roll axis is split across two `direct_torque` effectors
   `roll-torque-a` and `roll-torque-b`, each carrying a ±0.2 N·m
   symmetric box bound. Combined roll capacity is ±0.4 N·m, just
   above the autopilot's 0.35 N·m peak demand. Pitch and yaw axes
   keep their single-effector ±0.35 N·m surfaces.
2. `[fc.autopilot_allocation]` selects the
   `prioritised_redistributed` allocator with explicit
   `axis_priority = ["roll", "pitch", "yaw"]`. The Phase-5.A.5
   runner walks the `[[vehicle.assembly.effectors]]` list, derives
   the per-axis effector groups for `direct_torque` effectors, and
   installs the allocator on the mixer. The autopilot's
   `aileron_rad` demand is interpreted as roll-axis torque and
   distributed across both `roll-torque-*` effectors proportional
   to their per-effector capacity (50/50 here, since both carry
   equal limits).

The waypoint sequence, vehicle inertia, mission graph, MEKF, and
per-phase gain schedule are unchanged from the parent scenario.
The legacy `[fc.actuator_channels]` mapping is retained for the
`ActuatorCommand` semantic-topic publish path. The allocator consumes
the raw roll / pitch / yaw demand for the per-effector
`EffectorCommandSet`; phase authority is applied before the capacity
split, so disallowed effectors receive explicit zero commands and do not
contribute authority.

## License / restrictions

Synthetic OpenBMP data. No real fielded-vehicle parameters,
no real-world locations, no ITAR/EAR/MTCR/Wassenaar content.

## References

- Mellinger, D. and Kumar, V. *Minimum snap trajectory generation
  and control for quadrotors*. IEEE ICRA 2011, pp. 2520–2525.
- Härkegård, O. (2002). *Efficient Active Set Algorithms for
  Solving Constrained Least Squares Problems in Aircraft Control
  Allocation*. IEEE CDC 2002.
- Bordignon, K. A. and Durham, W. C. (1995). *Closed-Form
  Solutions to Constrained Control Allocation Problem*. JGCD
  18(5):1000–1007 — original "redistributed pseudoinverse"
  formulation; Phase 5.A.5 ships the simpler single-axis case.

## Validation status

`experimental`. Validates parser-side and runs end-to-end
deterministically. The Phase-5.A.5 e2e test
(`crates/openbmp-cli/tests/diff_flatness_allocator_e2e.rs`)
asserts:

- the scenario completes 8000 kernel steps with end-time stop;
- the rigid-body quaternion stays normalised after projection;
- both roll-torque effectors receive bit-identical commands at
  every tick (the proportional-split formula for equal-capacity
  effectors gives exact equality on the reference platform);
- two reruns produce byte-identical Parquet (the allocator is
  deterministic; combined with the existing rigid-body byte
  stability this propagates through the closed-loop pipeline).

## Safety boundary

`docs/safety-boundaries.md` accept list — academic guidance laws
(attitude tracking, scenario-defined waypoint navigation in
inertial space). The waypoints are scenario-internal points in
inertial space, not real-world locations or targets. No
proportional-navigation or terminal-homing logic. The redundant
roll effectors are a synthetic over-actuation demo, not a model of
a fielded effector configuration.
