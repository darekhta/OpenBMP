# Diff-flatness figure-eight INDI scenario provenance

## Scenario

`scenarios/diff-flatness-figure-eight-indi/scenario.toml`

## Source class

Synthetic. Identical to `scenarios/diff-flatness-figure-eight`
modulo two fields under `[fc.autopilot_params]`:

1. `rate_loop_kind = "indi"` swaps the rate loop from PID to the
   per-axis Incremental Nonlinear Dynamic Inversion shipped in
   Phase 5.A.3.C.
2. `[fc.autopilot_params.indi]` declares INDI's working estimate of
   body-axis inertia, per-axis control effectiveness, the
   synchronised filter cutoff applied identically to `ω` and the
   prior actuator command, and the outer-loop attitude-to-
   angular-acceleration P-gain.

The waypoint sequence, vehicle, mission graph, and per-phase gain
schedule are unchanged from the parent scenario. INDI is single-body
and depends on diagonal body-axis inertia; the unit identity inertia
matrix declared on `vehicle.assembly.bodies[0]` satisfies both
preconditions.

The configured INDI inertia matches the truth-side body inertia in
this baseline demonstration scenario. A future Phase 5.A.3.D
comparison harness includes a deliberate-mismatch variant that
showcases INDI's hallmark robustness when the parameter inertia is
overestimated by ~30 %.

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
deterministically. Closed-loop tracking quality is asserted by
`crates/openbmp-cli/tests/diff_flatness_indi_e2e.rs`.

## Safety boundary

`docs/safety-boundaries.md` accept list — academic guidance laws
(attitude tracking, scenario-defined waypoint navigation in inertial
space). The waypoints are scenario-internal points in inertial space,
not real-world locations or targets. No proportional-navigation or
terminal-homing logic.
