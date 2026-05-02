# Diff-flatness figure-eight PID-baseline scenario provenance

## Scenario

`scenarios/diff-flatness-figure-eight-baseline/scenario.toml`

## Source class

Synthetic. Identical to `scenarios/diff-flatness-figure-eight-l1`
modulo one field: the `[fc.autopilot_params.l1_adaptive]` block is
absent. This makes the rate loop run as a pure PID under the same
`ReducedRate { factor = 0.7 }` actuator disturbance on the roll-torque
effector. The pair (this scenario + the L1 sibling) is the closed-loop
controlled experiment used by
`crates/openbmp-cli/tests/diff_flatness_l1_robustness_e2e.rs` to
isolate the L1 contribution. The baseline and L1 sibling intentionally
share `time.seed = 0x4c31_4146_3168_466c` so deterministic synthetic
sensor noise does not become a comparison variable.

## License / restrictions

Synthetic OpenBMP data. No real fielded-vehicle parameters,
no real-world locations, no ITAR/EAR/MTCR/Wassenaar content.

## Validation status

`experimental`. Validates parser-side and runs end-to-end
deterministically; serves as the PID-only baseline against which the
L1 sibling is compared.

## Safety boundary

`docs/safety-boundaries.md` accept list — academic guidance laws
(attitude tracking, scenario-defined waypoint navigation in inertial
space). The waypoints are scenario-internal points in inertial space,
not real-world locations or targets. No proportional-navigation or
terminal-homing logic. The fault injection is a synthetic actuator
disturbance, not a model of any real fielded effector.
