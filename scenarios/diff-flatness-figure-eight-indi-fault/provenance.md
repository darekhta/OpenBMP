# Diff-flatness figure-eight INDI-fault scenario provenance

## Scenario

`scenarios/diff-flatness-figure-eight-indi-fault/scenario.toml`

## Source class

Synthetic. Identical to
`scenarios/diff-flatness-figure-eight-indi` modulo a single field on
the roll-torque effector:

```
fault = { kind = "reduced_rate", factor = 0.7 }
```

This is the same matched-uncertainty disturbance shipped on the
Phase-5.A.2.D `diff-flatness-figure-eight-baseline` and
`diff-flatness-figure-eight-l1` siblings, so the four fault-bearing
scenarios (PID baseline, PID + L1, LQR, INDI) face the same plant
disturbance and the controller comparison harness in
`crates/openbmp-cli/tests/controller_comparison_harness.rs` can
make an apples-to-apples comparison.

The waypoint sequence, vehicle, mission graph, MEKF, per-phase
gain schedule, and INDI parameters (inertia, control effectiveness,
filter cutoff, attitude gain) are unchanged from the parent
`diff-flatness-figure-eight-indi` scenario. The configured INDI
inertia matches the truth-side body inertia (1.0 kg·m² per axis),
so this scenario does not stress INDI's hallmark
parameter-mismatch robustness; that demonstration is deferred to
a future scenario.

## License / restrictions

Synthetic OpenBMP data. No real fielded-vehicle parameters,
no real-world locations, no ITAR/EAR/MTCR/Wassenaar content.

## References

- Mellinger, D. and Kumar, V. *Minimum snap trajectory generation and
  control for quadrotors*. IEEE ICRA 2011, pp. 2520–2525.
- Smeur, E. J. J., Chu, Q., and de Croon, G. C. H. E. (2016).
  *Adaptive Incremental Nonlinear Dynamic Inversion for Attitude
  Control of Micro Air Vehicles.* JGCD 39(3):450–461.

## Validation status

`experimental`. Validates parser-side and runs end-to-end
deterministically; consumed by the Phase-5.A.3.D controller
comparison harness alongside the PID baseline, PID + L1, and LQR
fault-bearing siblings.

## Safety boundary

`docs/safety-boundaries.md` accept list — academic guidance laws
(attitude tracking, scenario-defined waypoint navigation in inertial
space). The waypoints are scenario-internal points in inertial space,
not real-world locations or targets. No proportional-navigation or
terminal-homing logic. The fault injection is a synthetic actuator
disturbance, not a model of any real fielded effector.
