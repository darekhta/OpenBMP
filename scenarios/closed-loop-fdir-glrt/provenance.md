# Closed-loop FDIR-GLRT demo provenance

## Scenario

`scenarios/closed-loop-fdir-glrt/scenario.toml`

## Source class

Synthetic. The scenario is the Phase-5.B.4 end-to-end demo for the
new Willsky 1976 windowed-mean-shift GLRT detector
(`openbmp_fc::glrt::WindowedMeanShiftGlrt`,
`openbmp_fc::fdir::DetectorKind::WindowedMeanShiftGlrt`).

The vehicle, sensor noise budgets, mission graph, FC pipeline, gain
schedule, and phase-authority table are identical to
`scenarios/closed-loop-attitude-hold/scenario.toml`. The only change
is the addition of the v3-only `[fc.fdir.detector]` sub-block that
selects the windowed-mean-shift GLRT detector with a 32-sample
window and α = 0.001 family-wise false-alarm rate.

The Phase-5.B.4 commit promoted `[fc.fdir.detector].kind` from a
free-form string (parser-only in Phase 5.0) to a typed enum
`FcFdirDetectorKindV5::WindowedMeanShiftGlrt`. The runner builds
three per-sensor detectors (GNSS dim 6, baro dim 1, mag dim 3) from
the whitened-innovation streams the EKF now exports on
`EstimatorStatus`.

Under the nominal closed-loop trajectory the innovations stay
well-behaved and the windowed test should not trip in the 1 s run.
The math-side unit tests in
`crates/openbmp-fc/src/glrt.rs` cover trip semantics under
synthetic step injections; this scenario covers the wiring +
byte-stability of the runtime path.

## License / restrictions

Synthetic OpenBMP data. No real fielded-vehicle parameters, no
real-world locations, no ITAR / EAR / MTCR / Wassenaar content.

## References

- Willsky, A. S. and Jones, H. L. (1976). *A generalized likelihood
  ratio approach to the detection and estimation of jumps in linear
  systems*. IEEE Transactions on Automatic Control 21(1), 108-112.
  doi:10.1109/TAC.1976.1101146 — primary mathematical source.
- Hamilton, J. D. (1994). *Time Series Analysis*, §9.4 (Princeton
  University Press) — textbook reformulation of the GLRT for
  Gaussian innovation sequences.

## Validation status

`experimental`. Validates parser-side and runs end-to-end
deterministically. The Phase-5.B.4 e2e test
(`crates/openbmp-cli/tests/closed_loop_fdir_glrt_e2e.rs`) asserts:

- the scenario completes 1000 RK4 steps with end-time stop;
- two reruns produce byte-identical Parquet (the new detector and
  the EKF whitened-innovation export both honour the project's
  determinism contract).

Trip and no-false-trip semantics are covered at the math layer by
deterministic unit tests in `crates/openbmp-fc/src/glrt.rs`; this
scenario is the runtime wiring and byte-stability check.

Math-side coverage (`crates/openbmp-fc/src/glrt.rs`):

- 9 unit tests covering constructor validation, single-sample
  collapse to χ²(d) test, pure-H₀ no-trip over 200 samples, 4σ
  step-injection trips within window resolution, byte-stable
  determinism across two detector instances fed the same stream,
  reset clears state, and a scalar-D=1 specialisation.

EKF-side invariant coverage (`crates/openbmp-fc/src/estimator.rs`):

- 5 invariant tests covering `‖ν̃‖² = chi2` for GNSS / baro / mag
  whitened innovations, `begin_tick` clears the slots, and
  bit-stable whitening across two EKF instances fed the same
  measurement.

## Safety boundary

`docs/safety-boundaries.md` accept list — academic FDIR-detector
verification scenario. No guidance / navigation / control logic
beyond the existing closed-loop attitude-hold pipeline; no target
geometry; no real-world locations.
