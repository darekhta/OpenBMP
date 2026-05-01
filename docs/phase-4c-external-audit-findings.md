# Phase 4.C external audit findings

Auditor: Codex
Date: 2026-04-30
Commit: cd47cf20dc35c2cfc40a8bae6257d553d3ce0f8c (`cd47cf2`)
Toolchain: `rustc 1.95.0 (59807616e 2026-04-14)`; MSRV gate also run with `rustc 1.93.1 (01f6ddf75 2026-02-11)`
Platform: Darwin darekhta-mini 25.3.0 arm64

## Executive summary

The mechanical gate claims mostly reproduce: formatting, clippy, workspace build/test, FC no-default build/test, feature builds, MSRV build, machete, openbmp-env removal, lockstep lint, and scenario byte-stability all pass. The literal test-count claim is correct: `1063` passed, `0` failed, `5` ignored. The main audit failures are capability and math-truthfulness claims: WMM 2025 is in `openbmp-physics`, but the FC bridge and scenario config still hard-code/use `EarthDipoleField` and provide no `wmm_2025` opt-in; the GLRT detector is a threshold latch, not a GLR statistic; the Gauss-Markov covariance update does not implement the documented `q * tau / 2` steady-state variance when the parameter is a noise density. Several tests also validate local algebra rather than the advertised production workflow.

## Verified claims

| Claim | Status | Evidence |
|---|---|---|
| `cargo fmt` clean | confirmed | `cargo +1.95 fmt --all -- --check`, exit 0 |
| `cargo clippy` clean | confirmed | `cargo +1.95 clippy --workspace --all-targets --all-features --locked -- -D warnings`, exit 0 |
| Workspace all-features build | confirmed | `cargo +1.95 build --workspace --all-features --locked`, exit 0 |
| Workspace all-features tests | confirmed | `cargo +1.95 test --workspace --all-features --locked`, exit 0 |
| `1 063 tests passing` | confirmed | `cargo test --workspace --all-features ...`: `passed: 1063 failed: 0 ignored: 5` |
| HAL portability | confirmed | `cargo +1.95 build -p openbmp-fc --no-default-features --locked` and `cargo +1.95 test -p openbmp-fc --no-default-features --locked`, both exit 0 |
| FC feature builds | confirmed | `mpc`, `square-root-ekf`, `l1-adaptive`, and combined `mpc square-root-ekf l1-adaptive serde` with `--no-default-features` all build |
| Supply-chain gates | confirmed with warnings | `cargo deny check` exits 0 but emits unmatched-license and duplicate-crate warnings; `cargo machete` exits 0 clean |
| MSRV gate | confirmed | `cargo +1.93 build --workspace --all-features --locked`, exit 0 |
| `openbmp-env` retired | confirmed | `cargo tree -p openbmp-env` exits 101 with `package ID specification openbmp-env did not match any packages`; no `openbmp_env::` or `openbmp-env` TOML refs found |
| Lockstep-clock tripwire | confirmed with scope caveat | `cargo +1.95 test -p openbmp-testkit --lib --locked fc_lints`, exit 0; manual `rg` found no wall-clock APIs in FC, FC tests, physics, or bridge |
| Byte-stable scenarios | confirmed | All 12 scenario TOMLs under `scenarios/` were run twice with `--output-parquet`; `openbmp diff` and `cmp -s` passed for all 24 Parquet outputs |
| Joseph covariance form | confirmed | `crates/openbmp-fc/src/estimator.rs:671` implements `(I-KH)P(I-KH)^T + KRK^T` |
| WMM evaluator against NOAA values | confirmed in physics | `cargo +1.95 test -p openbmp-physics --lib --locked magnetic::wmm2025::tests::matches_all_noaa_reference_test_values`, exit 0 |

## Unverified or false claims

| Claim | Severity | What I found | Evidence |
|---|---|---|---|
| "WMM 2025 is in-tree, FC dipole placeholder superseded for scenarios that opt in." | high | `Wmm2025` is in-tree, but the FC bridge hard-codes `EarthDipoleField`, `FcConfig` has no magnetic-field selector, and `environment.magnetic` accepts only `"none"`. No FC or runner code path references `Wmm2025`. | `crates/openbmp-cli/src/runner/fc_bridge.rs:37`, `:78`; `crates/openbmp-scenario/src/document.rs:714`; `rg Wmm2025 crates/openbmp-fc crates/openbmp-cli/src/runner scenarios` returns only an FC lib doc mention |
| "WMM 2025 was previously on [Phase 5] list and is now removed." | medium | `docs/phase-5-plan.md` still lists WMM 2025 as P0. | `docs/phase-5-plan.md:7` |
| Physics consolidation DoD item 18 exports `validity` | medium | `openbmp-physics` exports gravity/atmosphere/magnetic/wind/error/earth, but no `validity` module exists. | `crates/openbmp-physics/src/lib.rs:44`; `ls crates/openbmp-physics/src` |
| Physics consolidation DoD item 21 closed-loop scenario runs with `[fc.estimator.mag_field = "wmm_2025"]` | high | The closed-loop scenario explicitly documents `EarthDipoleField` and has no such field; the scenario schema would reject unknown FC fields. | `scenarios/closed-loop-attitude-hold/scenario.toml:8`; `crates/openbmp-scenario/src/document.rs:3120` |
| "Gauss-Markov bias dynamics with steady-state variance `sigma^2 = q * tau / 2`." | medium | The covariance recurrence uses `Qd = sigma^2 * (1-exp(-2dt/tau))`, after scaling `P` by `exp(-2dt/tau)`, so `P_inf = sigma^2`. For a continuous noise density `q`, the exact OU discretization is `Qd = q*tau/2*(1-exp(-2dt/tau))`. | `crates/openbmp-fc/src/estimator.rs:690`; parameter docs use `/sqrt(s)` units at `crates/openbmp-fc/src/estimator.rs:156` |
| "GLRT ... detector families." | high | `DetectorKind::Glrt` only trips when an existing fault mask is nonzero or max chi-square exceeds a threshold. It does not compute a generalized likelihood-ratio statistic over hypotheses/windows. | `crates/openbmp-fc/src/fdir.rs:205` |
| "L1 adaptive augmentation ... Cao & Hovakimyan 2010." | medium | The module is a scalar disturbance estimate with projection and first-order low-pass correction. I found no reference model/state predictor structure, and scenario config has no L1 selector. | `crates/openbmp-fc/src/l1_adaptive.rs:61`; `crates/openbmp-scenario/src/document.rs:3302` |
| "Differential-flatness trajectory tracking." | medium | The selectable path computes a PD desired acceleration from position/velocity error and converts that acceleration to attitude. It is not a full flat-output trajectory tracker using planned higher derivatives. | `crates/openbmp-fc/src/autopilot.rs:377`; `crates/openbmp-fc/src/autopilot.rs:463` |
| "cargo deny ... all clean." | low | The command exits 0, but it is not warning-clean. | `cargo deny check`: unmatched license allowances and duplicate crate warnings |

## Algorithm soundness

### Joseph covariance update

- Reference: Joseph stabilized covariance update; Bierman 1977 square-root filtering literature uses this as the PSD-preserving covariance form.
- Implementation: `crates/openbmp-fc/src/estimator.rs:671`
- Math derivation: For linearized measurement `z = Hx + v`, `S = HPH^T + R`, `K = PH^T S^-1`. The Joseph posterior covariance expands the covariance of `(I-KH)e - Kv` as `(I-KH)P(I-KH)^T + KRK^T`, preserving symmetry/positive semidefiniteness better than `(I-KH)P`.
- Verdict: matches.
- Evidence: code computes `i_kh * p * i_kh.transpose() + k * r * k.transpose()` and symmetrizes.

### Markley MEKF reset

- Reference: Markley 2003, multiplicative attitude-error reset, second-order covariance reset matrix.
- Implementation: `crates/openbmp-fc/src/estimator.rs:1801`
- Math derivation: After injecting the estimated small attitude error into the nominal quaternion, the remaining local error coordinates change. Markley's second-order reset Jacobian is `G = I - 1/2[alpha x] + 1/12[alpha x]^2`; the covariance reset is `G P G^T`, extended block-diagonally for non-attitude error states.
- Verdict: matches with caveat.
- Evidence: code constructs `g = I - 0.5*skew + skew*skew/12` and applies it to the 3 attitude rows/cols. Caveat: the iterated mag update applies state corrections inside the loop and only resets covariance with the final iteration's attitude error; if multiple nontrivial iterations occur, this is not the same as applying/resetting each iteration.

### Gauss-Markov bias covariance

- Reference: first-order Ornstein-Uhlenbeck/Gauss-Markov process `db = -b/tau dt + sqrt(q) dW`.
- Implementation: `crates/openbmp-fc/src/estimator.rs:682`, `:690`
- Math derivation: The exact discrete transition is `phi = exp(-dt/tau)`, `P_{k+1}=phi^2 P_k + q*tau/2*(1-phi^2)`, so the stationary variance is `q*tau/2`. The implementation scales `P` by `phi^2` and adds `sigma^2*(1-phi^2)`, giving stationary variance `sigma^2`.
- Verdict: does not match the stated `q*tau/2` contract unless `sigma` is intentionally a stationary standard deviation, which conflicts with the `/sqrt(s)` parameter docs.
- Evidence: `gauss_markov_process_variance` lacks the `tau/2` factor.

### WMM 2025 evaluator

- Reference: NOAA/NCEI WMM2025 coefficient and 100-row test-value set.
- Implementation: `crates/openbmp-physics/src/magnetic/wmm2025.rs:423`
- Math derivation: The WMM evaluator should parse geodetic latitude/longitude/altitude and decimal year, apply secular variation to the Gauss coefficients, evaluate the spherical harmonic, and compare NED X/Y/Z against the published reference set within the WMM's declared significant-figure tolerance.
- Verdict: physics evaluator matches reference values; FC/bridge integration does not consume it.
- Evidence: `matches_all_noaa_reference_test_values` parses `data/magnetic/WMM2025_TestValues.txt` and checks 100 rows at 5 nT tolerance; targeted cargo test passes.

### FDIR GLRT/CUSUM

- Reference: GLRT computes a likelihood-ratio statistic between fault/no-fault hypotheses; CUSUM accumulates shifted residual statistics over time.
- Implementation: `crates/openbmp-fc/src/fdir.rs:187`
- Math derivation: A GLRT should form a statistic from residual likelihoods/covariance under competing hypotheses. A CUSUM detector updates `S_k = max(0, S_{k-1} + statistic - drift)` and trips when `S_k` crosses a threshold.
- Verdict: CUSUM shape is present; GLRT does not match.
- Evidence: the GLRT branch only ORs `current_mask` when a threshold is exceeded. The CUSUM branch accumulates `max_chi2 - drift`.

## Risk findings

- Severity: high
- Description: `GravityAdapter` silently clamps any gravity-model error to zero. Point-mass and J2 gravity return `OutOfEnvelope` at `r = 0` and `NonFinite` on overflow/nonfinite output; under the adapter the FC receives a finite zero-gravity vector and no bus topic/log/status records the clamp.
- Reproduction steps: inspect `crates/openbmp-fc/src/estimator.rs:71`, `crates/openbmp-physics/src/gravity.rs:189`, and `crates/openbmp-physics/src/gravity.rs:308`.
- Recommendation: expose an estimator status/fault bit or make adapter errors observable while keeping the hot path total.

- Severity: medium
- Description: The WMM and J2 capability surface exists programmatically but is not scenario-driven through the FC runner. This is the practical reason the WMM opt-in and closed-loop DoD claims fail.
- Reproduction steps: `rg -n "Wmm2025|wmm_2025" crates/openbmp-fc crates/openbmp-cli/src/runner scenarios`; inspect `FcConfig`.
- Recommendation: add explicit scenario fields for FC gravity/magnetic models or downgrade docs to "available as library APIs only."

- Severity: low
- Description: The lockstep tripwire does not scan `openbmp-physics`, although the commit message says the tripwire passes for both FC and physics. Manual `rg` found physics clean.
- Reproduction steps: inspect `crates/openbmp-testkit/src/fc_lints.rs:15`; run the manual wall-clock `rg` over physics.
- Recommendation: add a physics scan test or narrow the claim.

- Severity: low
- Description: The audit handover's scenario command uses `--output <dir>`, but the actual CLI exposes `--output-csv`, `--output-json`, and `--output-parquet`.
- Reproduction steps: `target/release/openbmp run scenarios/analytic-toy/constant-acceleration-drop.toml --output /tmp/audit` exits 2; `target/release/openbmp run --help`.
- Recommendation: update the handover command.

## Tests of suspect rigor

- File: `crates/openbmp-fc/tests/textbook_examples.rs:16`
- What the test claims to verify: Bar-Shalom/Li/Kirubarajan textbook consistency examples.
- What it actually verifies: local scalar Kalman algebra with arbitrary numbers; the whitened innovation test divides a square root by itself.
- Recommendation: cite a concrete published example number/table or rename as local algebra smoke tests.

- File: `crates/openbmp-fc/tests/long_duration_stability.rs:13`
- What the test claims to verify: 60-second estimator stability.
- What it actually verifies: direct calls into `Ekf` using synthetic measurement structs. It does not exercise sensor ingest, bus scheduling, autopilot, mixer, or bridge commands.
- Recommendation: keep this as an EKF stability test, and add a separate full-pipeline long-duration test if that claim is needed.

- File: `crates/openbmp-physics/tests/wmm_data_pin.rs:1`
- What the test claims to verify: WMM data pinning.
- What it actually verifies: `WMM.COF` SHA-256 and coefficient table round-trip, not NOAA test vectors. The NOAA vector comparison is real but lives in `crates/openbmp-physics/src/magnetic/wmm2025.rs:526`.
- Recommendation: update references to the real NOAA-vector test location.

- File: `crates/openbmp-fc/src/fdir.rs:205`
- What the tests claim to verify: `glrt_latches_explicit_fault_bits`.
- What it actually verifies: a threshold/fault-bit latch under a `Glrt` enum variant, not a GLRT statistic.
- Recommendation: rename the detector or implement a real likelihood-ratio statistic.

## Architectural observations

HAL portability is intact by cargo build/test and dependency tree: `openbmp-fc` depends on `openbmp-core`, `openbmp-physics`, `openbmp-mission`, `openbmp-sensors`, math/serde/error crates, and optional Clarabel, not simulator/tooling crates. The physics consolidation removed the old crate and duplicate FC traits. The bridge's three load-bearing translations are mostly correct: specific force subtracts gravity before IMU truth, magnetic field is rotated ECI-to-body, and barometer truth uses kernel position rather than FC estimate. The barometer path clamps negative `z` altitude to zero; that may be intentional for the toy frame, but it should be documented if below-ground trajectories are expected.

The FC runner is still missing the model-selection plumbing implied by the Phase 4.C docs. `FcRunner::new` constructs default `Ekf`/`Mekf` instances and never calls `with_gravity_model` or `with_mag_field_model`; `FcBridge` owns `EarthDipoleField` directly. This makes the consolidated physics library useful, but not yet the scenario-selectable FC capability described by the commit.

## Open questions for the project owner

- Should WMM 2025 be considered Phase 4 complete only as a physics crate model, or must the FC scenario path actually support `wmm_2025`?
- Should `GravityAdapter` remain fail-silent, or should estimator status/FDIR expose adapter clamps?
- Should chi-square gates use an exact inverse CDF instead of Wilson-Hilferty approximation? For FAR 0.01 and 3 DOF, code returns about 11.369 rather than the exact 11.345.
- Should GLRT/L1/differential-flatness names be downgraded to match the shipped simplified algorithms, or should the implementations be expanded to match the cited references?
