# Diff-flatness figure-eight MPC scenario provenance

## Scenario

`scenarios/diff-flatness-figure-eight-mpc/scenario.toml`

## Source class

Synthetic. Identical to `scenarios/diff-flatness-figure-eight`
modulo two fields under `[fc.autopilot_params]`:

1. `attitude_loop_kind = "mpc"` swaps the per-axis PID attitude
   loop for the receding-horizon attitude MPC.
2. `[fc.autopilot_params.attitude_mpc]` declares the horizon length,
   per-axis stage / terminal cost weights, and the box rate-command
   bound the MPC enforces over every horizon step.

The waypoint sequence, vehicle, mission graph, MEKF, per-phase gain
schedule, and PID rate-loop gains are unchanged from the parent
scenario. The MPC replaces ONLY the attitude loop and outputs the
optimal first-step rate command consumed by the unchanged rate loop.
The `[fc.autopilot_params.attitude_mpc]` weights are scenario-demo
weights, not fielded-vehicle defaults: the 20 ms horizon is intentionally
short for the offline figure-eight regression, and the QP models the
commanded-rate integrator rather than the downstream rate-loop / actuator
dynamics.

The scenario keeps a unique deterministic seed
`0x4d50_4331_3168_466c` (`"MPC11hFl"`) because it is not part of the
reduced-rate fault comparison family. The comparison
siblings share `0x4c31_4146_3168_466c`; this MPC case is a standalone
solver-path demonstration.

## License / restrictions

Synthetic OpenBMP data. No real fielded-vehicle parameters,
no real-world locations, no ITAR/EAR/MTCR/Wassenaar content.

## References

- Mellinger, D. and Kumar, V. *Minimum snap trajectory generation and
  control for quadrotors*. IEEE ICRA 2011, pp. 2520–2525.
- Maciejowski, J.M. *Predictive Control with Constraints*. Prentice
  Hall, 2002 — receding-horizon QP formulation; condensed cost
  derivation in Ch. 3.
- Goodfellow, Y. and the Clarabel authors. *Clarabel: an
  interior-point solver for conic programs in Rust*. SIAM Journal on
  Optimization (in press); see github.com/oxfordcontrol/Clarabel.

## Validation status

`experimental`. Validates parser-side and runs end-to-end
deterministically. Closed-loop tracking is asserted by
`crates/openbmp-cli/tests/diff_flatness_mpc_e2e.rs` (run-to-completion
plus byte-stable Parquet across reruns — Clarabel is configured with
the deterministic settings shared by the
`solve_attitude_box_qp` path, so the QP solution is bit-identical between
runs on the reference platform). The validation does not claim real-time
solver budget compliance or a production cascaded-loop plant model.

## Safety boundary

`docs/safety-boundaries.md` accept list — academic guidance laws
(attitude tracking, scenario-defined waypoint navigation in inertial
space). The waypoints are scenario-internal points in inertial space,
not real-world locations or targets. No proportional-navigation or
terminal-homing logic.
