# Diff-flatness figure-eight L1 scenario provenance

## Scenario

`scenarios/diff-flatness-figure-eight-l1/scenario.toml`

## Source class

Synthetic. Identical to `scenarios/diff-flatness-figure-eight` modulo:

1. An `EffectorFault::ReducedRate { factor = 0.7 }` mounted on the
   roll-torque effector — simulating a 30 % slew-rate loss on that
   axis as a matched, axis-local actuator disturbance.
2. A populated `[fc.autopilot_params.l1_adaptive]` block, which
   activates the per-axis Cao-Hovakimyan 2010 L1 adaptive augmentation
   on the rate loop.

The waypoint sequence, vehicle, mission graph, and gain schedule are
unchanged from the parent scenario.

## License / restrictions

Synthetic OpenBMP data. No real fielded-vehicle parameters,
no real-world locations, no ITAR/EAR/MTCR/Wassenaar content.

## References

- Mellinger, D. and Kumar, V. *Minimum snap trajectory generation and
  control for quadrotors*. IEEE ICRA 2011, pp. 2520–2525.
- Cao, C. and Hovakimyan, N. *L1 Adaptive Control Theory: Guaranteed
  Robustness with Fast Adaptation*. SIAM Advances in Design and
  Control, 2010.

## Validation status

`experimental`. Validates parser-side and runs end-to-end
deterministically. Closed-loop attitude-tracking improvement against
the baseline-PID sibling is asserted by
`crates/openbmp-cli/tests/diff_flatness_l1_robustness_e2e.rs`.

## Safety boundary

`docs/safety-boundaries.md` accept list — academic guidance laws
(attitude tracking, scenario-defined waypoint navigation in inertial
space). The waypoints are scenario-internal points in inertial space,
not real-world locations or targets. No proportional-navigation or
terminal-homing logic. The fault injection is a synthetic actuator
disturbance, not a model of any real fielded effector.
