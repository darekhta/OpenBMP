# Diff-flatness figure-eight scenario provenance

## Scenario

`scenarios/diff-flatness-figure-eight/scenario.toml`

## Source class

Synthetic. The waypoint sequence is hand-chosen for an academic
figure-eight (lemniscate-shaped) trajectory in the ECI XY plane at
constant altitude `z = 100 m`. Five waypoints over 8 seconds:

| time (s) | position ECI (m)        |
|----------|-------------------------|
| 0.0      | (  0.0,  0.0, 100.0)    |
| 2.0      | ( 10.0,  5.0, 100.0)    |
| 4.0      | ( 20.0,  0.0, 100.0)    |
| 6.0      | ( 10.0, -5.0, 100.0)    |
| 8.0      | (  0.0,  0.0, 100.0)    |

The trajectory itself (polynomial coefficients) is computed at
scenario load by the Mellinger & Kumar 2011 minimum-snap solver in
`crates/openbmp-fc/src/trajectory.rs`. No external dataset is
shipped — the waypoints are inline in the scenario file.

## License / restrictions

Synthetic OpenBMP data. No real fielded-vehicle parameters,
no real-world locations, no ITAR/EAR/MTCR/Wassenaar content.

## Reference

Mellinger, D. and Kumar, V. *Minimum snap trajectory generation and
control for quadrotors*. IEEE ICRA 2011, pp. 2520–2525.

## Validation status

`experimental`. The scenario validates parser-side and runs end-to-end
deterministically; tighter closed-loop attitude tracking tolerance
comes from the L1-adaptive and observer-form anti-windup
sibling scenarios. The trajectory generator's polynomial
coefficients are bit-stable across reruns on the reference platform
profile, validated by `crates/openbmp-fc/src/trajectory.rs` unit tests
and the `diff_flatness_e2e.rs` end-to-end test.

## Safety boundary

`docs/safety-boundaries.md` accept list — academic guidance laws
(attitude tracking, scenario-defined waypoint navigation in inertial
space). The waypoints are scenario-internal points in inertial space,
not real-world locations or targets. No proportional-navigation or
terminal-homing logic.
