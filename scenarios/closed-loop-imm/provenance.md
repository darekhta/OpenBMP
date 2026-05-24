# Closed-loop IMM demo provenance

## Scenario

`scenarios/closed-loop-imm/scenario.toml`

## Source class

Synthetic. The scenario is the end-to-end demo for the
Bar-Shalom IMM (Interacting Multiple Model) estimator
(`openbmp_fc::imm::ImmEstimator`).

The vehicle, sensor noise budgets, mission graph, autopilot pipeline,
gain schedule, and phase-authority table are identical to
`scenarios/closed-loop-attitude-hold/scenario.toml`. The estimator
selection is the only change: `[fc].estimator = "imm"` plus a new
v3-only `[fc.imm]` block declaring a 2-mode bank with a Markov
transition matrix and per-mode EKF tuning overrides.

`FcEstimatorKind` covers `{Ekf, Mekf, Imm}`. The runner builds the IMM bank
from the per-mode `[[fc.imm.mode]]` overrides on top of the base
`[fc.ekf]` parameters, runs the bank's two EKFs in parallel, mixes
their priors via the transition matrix at the start of each tick,
and fuses their outputs by the posterior mode probabilities.

Under the nominal closed-loop trajectory the active mode should
remain at index 0 (the nominal-tuning mode); the maneuver mode
(index 1, 10× process noise on gyro and accel-bias channels) should
remain at low probability throughout the 1 s run. The math-side
unit tests in `crates/openbmp-fc/src/imm.rs` cover Markov-transition
prediction and likelihood-driven probability evolution; this
scenario covers the runtime wiring + byte-stability of the runtime
path.

## License / restrictions

Synthetic OpenBMP data. No real fielded-vehicle parameters, no
real-world locations, no ITAR / EAR / MTCR / Wassenaar content.

## References

- Bar-Shalom, Y., Kirubarajan, T., and Li, X. R. (2001).
  *Estimation with Applications to Tracking and Navigation*,
  Wiley §11.6 (IMM derivation and worked examples) — primary
  mathematical source.
- Blom, H. A. P. and Bar-Shalom, Y. (1988). *The interacting
  multiple model algorithm for systems with Markovian switching
  coefficients*, IEEE Transactions on Automatic Control 33(8),
  780-783 — foundational paper.

## Validation status

`experimental`. Validates parser-side and runs end-to-end
deterministically. The e2e test
(`crates/openbmp-cli/tests/closed_loop_imm_e2e.rs`) asserts:

- the scenario completes 1000 RK4 steps with end-time stop;
- two reruns produce byte-identical Parquet (the IMM bank is
  deterministic: pure `f64` arithmetic, locked operand order on
  per-mode mixing and likelihood updates, no FMA, no system RNG).

Math-side coverage (`crates/openbmp-fc/src/imm.rs`):

- 13 unit tests covering constructor validation (mode count,
  transition-matrix row sums, initial-probability sums),
  probability-simplex invariant after measurement updates, fused
  position is the weighted mean of mode positions, byte-stable
  determinism across two IMM instances fed the same measurement
  sequence, log-sum-exp numerical-stability with NEG_INFINITY
  entries, EstimatorMode topic zero-pads unused slots, mode
  probabilities respect prediction-only Markov transitions,
  gate-rejected measurements record current likelihoods,
  antipodal-quaternion fusion falls back to the active mode, the
  estimator job publishes EstimatorMode, and mode probability
  evolves under synthetic likelihood separation.

EKF-side IMM-supporting surface
(`crates/openbmp-fc/src/estimator.rs`):

- 3 invariant tests covering `log det S = 2 · Σ log L_diag`
  reconstruction, per-sensor `last_log_det_s_*` reset in
  `begin_tick`, and `internal_state` ↔ `set_internal_state`
  bit-exact round-trip on every component.

Scenario-validator coverage:

- 5 tests in `crates/openbmp-scenario/src/scenario.rs` covering
  v3 happy-path acceptance, v2 schema rejection,
  transition-matrix-row-sum rejection, initial-probability sum
  rejection, and mode-count vs matrix-size mismatch rejection.

## Safety boundary

`docs/safety-boundaries.md` accept list — academic estimator
verification scenario. The IMM is used strictly for
self-state-estimation under regime change (the textbook IMM use
case); the project-side scope guardrail in `docs/safety-boundaries.md`
explicitly forbids any multi-target tracking extension.
No guidance / navigation / control logic beyond the existing
closed-loop attitude-hold pipeline; no target geometry; no
real-world locations.
