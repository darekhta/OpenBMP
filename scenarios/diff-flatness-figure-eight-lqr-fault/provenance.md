# Diff-flatness figure-eight LQR-fault scenario provenance

## Scenario

`scenarios/diff-flatness-figure-eight-lqr-fault/scenario.toml`

## Source class

Synthetic. Identical to
`scenarios/diff-flatness-figure-eight-lqr` modulo a single field on
the roll-torque effector:

```
fault = { kind = "reduced_rate", factor = 0.7 }
```

This is the same matched-uncertainty disturbance shipped on the
`diff-flatness-figure-eight-baseline` and
`diff-flatness-figure-eight-l1` siblings, so the four fault-bearing
scenarios (PID baseline, PID + L1, LQR, INDI) face the same plant
disturbance and the controller comparison harness in
`crates/openbmp-cli/tests/controller_comparison_harness.rs` can
make an apples-to-apples comparison.

The waypoint sequence, vehicle, mission graph, MEKF, per-phase
gain schedule, and LQR cost weights are unchanged from the parent
`diff-flatness-figure-eight-lqr` scenario. The deterministic seed is
intentionally aligned with the PID-baseline, PID + L1, and INDI
fault-bearing siblings so the comparison harness sees the same
synthetic-sensor stream across all four rows.

## License / restrictions

Synthetic OpenBMP data. No real fielded-vehicle parameters,
no real-world locations, no ITAR/EAR/MTCR/Wassenaar content.

## References

- Mellinger, D. and Kumar, V. *Minimum snap trajectory generation and
  control for quadrotors*. IEEE ICRA 2011, pp. 2520–2525.
- Anderson, B.D.O. and Moore, J.B. *Optimal Control: Linear Quadratic
  Methods*. Prentice-Hall, 1990.

## Validation status

`experimental`. Validates parser-side and runs end-to-end
deterministically; consumed by the controller
comparison harness alongside the PID baseline, PID + L1, and INDI
fault-bearing siblings.

## Safety boundary

`docs/safety-boundaries.md` accept list — academic guidance laws
(attitude tracking, scenario-defined waypoint navigation in inertial
space). The waypoints are scenario-internal points in inertial space,
not real-world locations or targets. No proportional-navigation or
terminal-homing logic. The fault injection is a synthetic actuator
disturbance, not a model of any real fielded effector.
