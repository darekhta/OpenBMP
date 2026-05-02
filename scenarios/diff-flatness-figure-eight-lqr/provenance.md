# Diff-flatness figure-eight LQR scenario provenance

## Scenario

`scenarios/diff-flatness-figure-eight-lqr/scenario.toml`

## Source class

Synthetic. Identical to `scenarios/diff-flatness-figure-eight`
modulo two fields under `[fc.autopilot_params]`:

1. `rate_loop_kind = "lqr"` swaps the rate loop from PID to the
   per-axis LQR shipped in Phase 5.A.3.B.
2. `[fc.autopilot_params.lqr]` declares per-axis cost weights
   (`q_omega`, `q_int`, `r`).

The waypoint sequence, vehicle, mission graph, MEKF, and per-phase
gain schedule are unchanged from the parent scenario. The
gain-schedule rate-PID gains are still parsed (the autopilot keeps
them in case future scenarios mix LQR and PID per phase) but are
not consulted while `rate_loop_kind = "lqr"` is active.

## License / restrictions

Synthetic OpenBMP data. No real fielded-vehicle parameters,
no real-world locations, no ITAR/EAR/MTCR/Wassenaar content.

## References

- Mellinger, D. and Kumar, V. *Minimum snap trajectory generation and
  control for quadrotors*. IEEE ICRA 2011, pp. 2520–2525.
- Anderson, B.D.O. and Moore, J.B. *Optimal Control: Linear Quadratic
  Methods*. Prentice-Hall, 1990.
- Lewis, F.L., Vrabie, D., and Syrmos, V.L. *Optimal Control*, 3rd ed.
  Wiley, 2012.

## Validation status

`experimental`. Validates parser-side and runs end-to-end
deterministically. Closed-loop tracking quality is asserted by
`crates/openbmp-cli/tests/diff_flatness_lqr_e2e.rs`.

## Safety boundary

`docs/safety-boundaries.md` accept list — academic guidance laws
(attitude tracking, scenario-defined waypoint navigation in inertial
space). The waypoints are scenario-internal points in inertial space,
not real-world locations or targets. No proportional-navigation or
terminal-homing logic.
