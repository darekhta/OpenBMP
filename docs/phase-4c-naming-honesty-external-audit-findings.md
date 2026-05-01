# Phase 4.C naming-honesty external audit findings

Auditor: Codex
Date: 2026-05-01
Audited commit: db0ece7
Toolchain: rustc 1.95.0 (59807616e 2026-04-14)

## Executive summary

The `db0ece7` rename is behaviorally pure for the three shipped
surfaces: autopilot trajectory selection, FDIR detector selection, and
the feature-gated L1-inspired rate-loop augmentation. Scenario outputs
are byte-stable against `db0ece7~1` across the closed-loop, Calisto,
and analytic-drop scenarios, and the all-features workspace test count
is exactly 1082 passed tests. The new names are honest against the
implementations: `SingleSampleGlrt` thresholds a single chi-square
innovation sample, and `FlatnessInspired` uses PD acceleration plus
thrust-axis attitude assignment, not a full flat-output tracker. I
found no high-severity behavioral issue. I found two fixable honesty
issues: the scenario enum "round-trip" claim was false at `db0ece7`
because the enums did not implement `Serialize`, and old scenario
strings lingered in historical handover docs.

## Verified claims

| Claim | Status | Evidence |
|---|---|---|
| No old enum/type/scenario name lingers | Verified for production crates and scenarios; false for historical handover docs at `db0ece7`, fixed after audit | `rg -n "DifferentialFlatness\|::Glrt\\b\|L1AdaptiveParams\|L1AdaptiveChannel" crates --glob '*.rs'` returned no matches. `rg -n "\"differential_flatness\"\|\"glrt\"" crates docs scenarios` found `docs/phase-4c-audit.md:227,245` at `db0ece7`; the broader handover regex also found `docs/phase-4c-physics-followup-external-audit.md:296`. Current matches are confined to audit findings docs plus allowed `crate::l1_adaptive::L1Inspired*` paths. |
| Scenario string mapping is correct | Inbound mapping verified at `db0ece7`; full round-trip false until remediation | Throwaway test under `target/audit-scratch/` parsed `flatness_inspired` and `single_sample_glrt`, rejected old/hyphen spellings, and passed. The same test with `Serialize` derives failed at `db0ece7` with E0277 because `FcTrajectoryKind` and `FcFdirDetectorKind` did not implement `Serialize`. Current code derives `Serialize` and has a tracked regression test. |
| Byte-stable across closed-loop / Calisto / drop scenarios | Verified | Isolated worktrees at `db0ece7` and `db0ece7~1`; `diff -r /tmp/audit-naming-old /tmp/audit-naming-new` was silent. New SHAs: `scenario.parquet` = `78722b2f2bf62241a9ecce8ba1667d0ec0c637eb5a078b36571a0cad9901e158`; `rocketpy-calisto.parquet` = `37fac1281cf8a370e127003152ffd7b790803a7328c02210b6329a006937bceb`; `constant-acceleration-drop.parquet` = `fdb52f47b236b77669db53f6e7d47fc3cd47e05881cbe2ef71949d7427fe36fb`. |
| Test count is exactly 1082 | Verified | Exact `db0ece7` isolated worktree: `passed: 1082 failed: 0 ignored: 5`. |
| Dropped test was a tautology | Verified | Recovered `whitened_innovation_identity_is_unitless_smoke` from `db0ece7~1`; it computes `sqrt(prior_p + r) / sqrt(prior_p + r)` with finite positive `prior_p = 2`, `r = 3`, then asserts it is 1. It tested no edge case or system behavior. |
| `SingleSampleGlrt` name accurately describes thresholding the chi-square innovation | Verified | `single_sample_glrt_fault_mask` clamps the supplied innovation chi-square to nonnegative, compares one sample to `innovation_threshold`, and ORs the source bit into the current fault mask. For Gaussian innovation `r ~ N(mu, S)`, testing `mu = 0` against unconstrained `mu` yields maximized likelihood at `mu_hat = r`; twice the log likelihood ratio reduces to `r^T S^-1 r`, the chi-square innovation statistic. This is the Bar-Shalom/Li/Kirubarajan innovation-consistency statistic and the one-sample specialization of the Willsky 1976 mean-shift GLRT setup, not the windowed detector. |
| `FlatnessInspired` name accurately describes PD-on-position + attitude assignment without higher-derivative feedforward | Verified | At `db0ece7`, `flatness_pd_accel` computes `kp*(r_ref-r) + kd*(v_ref-v) - standard_down_z_eci_m_s2()`, i.e. PD desired acceleration plus hover-trim gravity feed-forward. `flatness_inspired_attitude_reference` builds body z from the desired-acceleration direction and body x/y from the yaw reference; no jerk, snap, attitude-rate, or angular-acceleration feed-forward appears. This is the attitude-assignment piece of Mellinger-Kumar, not the full tracker. |
| Pre-existing housekeeping diffs are unrelated to the rename | Verified for content; provenance not independently provable | Diffs in `dictionary.rs`, `error.rs`, `mixer.rs`, `scheduler.rs`, `topics.rs`, `voter.rs`, and `phase-4c-audit.md` are doc-link or typo changes only. `git log --all --oneline -- <files>` shows commits, not pre-commit worktree provenance. |
| HAL portability preserved | Verified | Exact `db0ece7`: `cargo +1.95 build -p openbmp-fc --no-default-features --locked` and `cargo +1.95 test -p openbmp-fc --no-default-features --locked` passed. Feature-gated `mpc,square-root-ekf,l1-adaptive` build/test also passed. |
| Rustdoc, deny, machete gates are clean | Verified with warnings noted | `RUSTDOCFLAGS="-D warnings" cargo +1.95 doc -p openbmp-fc -p openbmp-scenario --no-deps --locked` passed. `cargo machete` found no unused dependencies. `cargo deny check` exited 0 with pre-existing warnings for unmatched license allowances and duplicate lockfile entries. |

## Unverified or false claims

| Claim | Severity | What I found | Evidence |
|---|---|---|---|
| Scenario variants should serialize and deserialize as the new strings | Medium | At `db0ece7`, they deserialize but do not serialize because the scenario enums derive `Deserialize` only. | Throwaway `Serialize` wrapper failed with `FcTrajectoryKind: serde::Serialize is not satisfied` and the same error for `FcFdirDetectorKind`. |
| Old names occur only inside prior audit findings docs | Low | At `db0ece7`, `docs/phase-4c-audit.md` still listed `"differential_flatness"` and `"glrt"` in the Phase 4.C handover text, and `docs/phase-4c-physics-followup-external-audit.md` still used `differential_flatness` in a grep instruction. These were not consumers or fixtures, but they violated the mechanical old-name expectation. | `docs/phase-4c-audit.md:227,245`; `docs/phase-4c-physics-followup-external-audit.md:296`. |
| Housekeeping was already in the working tree before the rename pass | Low | The diffs are unrelated to behavior, but Git history alone cannot prove they were pre-existing unstaged changes. | `git log --all --oneline -- crates/openbmp-fc/src/dictionary.rs crates/openbmp-fc/src/mixer.rs crates/openbmp-fc/src/topics.rs` only shows commits, not the pre-commit worktree. |

## Architectural observations

The `l1_adaptive.rs` file name, `crate::l1_adaptive::*` module path,
and `l1-adaptive` feature flag mismatch with `L1Inspired*` type names
is defensible. The module doc explicitly says the feature flag is kept
for downstream-API stability while the Rust types use honest
`L1Inspired*` naming. Rustdoc with `-D warnings` passed for
`openbmp-fc` and `openbmp-scenario`, so the mismatch is documented and
does not leave broken intra-doc links.

The Phase 4.C and Phase 5 plans are paired correctly:

| Phase 4.C shipped surface | Phase 5 cited-reference item |
|---|---|
| `TrajectoryKind::FlatnessInspired` | Mellinger & Kumar 2011 full minimum-snap flat-output tracker |
| `L1InspiredParams` / `L1InspiredChannel` | Cao & Hovakimyan 2010 full L1 reference-model / state-predictor architecture |
| `DetectorKind::SingleSampleGlrt` | Willsky 1976 windowed mean-shift GLRT |
| `tests/local_kalman_algebra.rs` | Bar-Shalom / Li / Kirubarajan textbook examples with original parameter sets |
| `tests/long_duration_stability.rs` | Full closed-loop pipeline long-duration soak |

No Phase 5 cited-reference item lacked a corresponding Phase 4.C
inspired or smoke-test surface, and no Phase 4.C inspired surface
lacked its Phase 5 full-SOTA target.

## Risk findings

### R1 - Medium - Scenario enum round-trip claim false at `db0ece7`

Description: `FcTrajectoryKind` and `FcFdirDetectorKind` used
`#[serde(rename_all = "snake_case")]` but derived only `Deserialize`.
The new strings parsed correctly, and old strings were rejected, but a
true TOML round-trip could not compile.

Reproduction:

```bash
cargo test --manifest-path target/audit-scratch/Cargo.toml --offline
```

With wrappers deriving `Serialize`, this failed at `db0ece7` with
E0277 for both enums. Current remediation derives `Serialize` on both
enums and adds a tracked `openbmp-scenario` regression test.

### R2 - Low - Historical handover docs still used old scenario strings

Description: `docs/phase-4c-audit.md` listed the pre-rename scenario
strings in D10/D11, and
`docs/phase-4c-physics-followup-external-audit.md` retained the old
trajectory spelling in a grep instruction. These were not executable
and did not affect scenario parsing, but they failed the old-name sweep
outside the prior findings docs.

Reproduction:

```bash
rg -n "\"differential_flatness\"|\"glrt\"" crates docs scenarios
rg -n "DifferentialFlatness|differential_flatness|::Glrt\\b|\"glrt\"|L1AdaptiveParams|L1AdaptiveChannel|l1_adaptive\\s*:" crates docs scenarios --glob '*.rs' --glob '*.md' --glob '*.toml'
```

Current remediation updates the handover text to
`flatness_inspired` / `single_sample_glrt`, updates the follow-up audit
grep instruction, and describes the full SOTA work as Phase 5.

## Verification commands

```bash
rustc --version
git diff db0ece7~1..db0ece7 --stat
git diff --name-status db0ece7~1..db0ece7
rg -n "DifferentialFlatness|::Glrt\\b|L1AdaptiveParams|L1AdaptiveChannel" crates --glob '*.rs'
rg -n "\"differential_flatness\"|\"glrt\"" crates docs scenarios
rg -n "FlatnessInspired|SingleSampleGlrt|L1InspiredParams|L1InspiredChannel" crates --glob '*.rs'
rg -n "\"flatness_inspired\"|\"single_sample_glrt\"" crates docs scenarios
cargo test --manifest-path target/audit-scratch/Cargo.toml --offline
git show db0ece7~1:crates/openbmp-fc/tests/textbook_examples.rs
git diff --word-diff=plain db0ece7~1..db0ece7 -- crates/openbmp-fc/src/autopilot.rs
git diff --word-diff=plain db0ece7~1..db0ece7 -- crates/openbmp-fc/src/fdir.rs
git diff --word-diff=plain db0ece7~1..db0ece7 -- crates/openbmp-fc/src/l1_adaptive.rs
git diff db0ece7~1..db0ece7 -- crates/openbmp-fc/src/dictionary.rs crates/openbmp-fc/src/error.rs crates/openbmp-fc/src/mixer.rs crates/openbmp-fc/src/scheduler.rs crates/openbmp-fc/src/topics.rs crates/openbmp-fc/src/voter.rs docs/phase-4c-audit.md
git log --all --oneline -- crates/openbmp-fc/src/dictionary.rs crates/openbmp-fc/src/mixer.rs crates/openbmp-fc/src/topics.rs
cargo +1.95 fmt --all -- --check
cargo +1.95 clippy --workspace --all-targets --all-features --locked -- -D warnings
cargo +1.95 build --workspace --all-targets --all-features --locked
cargo +1.95 test --workspace --all-features --locked
cargo +1.95 test --workspace --all-features --locked > target/audit-scratch/workspace-test-count.log 2>&1
rg "test result:" target/audit-scratch/workspace-test-count.log | awk '{ p+=$4; f+=$6; i+=$8 } END { print "passed:", p, "failed:", f, "ignored:", i }'
cargo +1.95 build -p openbmp-fc --no-default-features --locked
cargo +1.95 test -p openbmp-fc --no-default-features --locked
cargo +1.95 build -p openbmp-fc --features mpc,square-root-ekf,l1-adaptive --locked
cargo +1.95 test -p openbmp-fc --features mpc,square-root-ekf,l1-adaptive --locked
cargo deny check
cargo machete
RUSTDOCFLAGS="-D warnings" cargo +1.95 doc -p openbmp-fc -p openbmp-scenario --no-deps --locked
```

Byte-stability commands were run from detached worktrees:

```bash
git worktree add --detach /tmp/openbmp-audit-naming-new db0ece7
git worktree add --detach /tmp/openbmp-audit-naming-old db0ece7~1
CARGO_TARGET_DIR=/tmp/audit-naming-target-new cargo build --release --bin openbmp --locked --manifest-path /tmp/openbmp-audit-naming-new/Cargo.toml
CARGO_TARGET_DIR=/tmp/audit-naming-target-old cargo build --release --bin openbmp --locked --manifest-path /tmp/openbmp-audit-naming-old/Cargo.toml
for s in scenarios/closed-loop-attitude-hold/scenario.toml \
         scenarios/sounding-rocket/calisto/rocketpy-calisto.toml \
         scenarios/analytic-toy/constant-acceleration-drop.toml; do
  base=$(basename "$s" .toml)
  /tmp/audit-naming-target-new/release/openbmp run "/tmp/openbmp-audit-naming-new/$s" --output-parquet "/tmp/audit-naming-new/${base}.parquet"
  /tmp/audit-naming-target-old/release/openbmp run "/tmp/openbmp-audit-naming-old/$s" --output-parquet "/tmp/audit-naming-old/${base}.parquet"
done
diff -r /tmp/audit-naming-old /tmp/audit-naming-new
shasum -a 256 /tmp/audit-naming-new/*.parquet
```

## Sign-off

At exact `db0ece7`, the rename is behaviorally fit to keep: no
function-body behavior changed beyond identifier substitution, no
scenario bytes changed, and no production consumer kept the old names.
The commit did need follow-up for the false TOML round-trip claim and
the stale old strings in historical handover docs. Those issues are
addressed in the current code by deriving `Serialize` for the two
scenario enums, adding a regression test for the new strings and
old-spelling rejection, and updating the stale handover text before
the Phase 4 docs were removed from the working tree.
