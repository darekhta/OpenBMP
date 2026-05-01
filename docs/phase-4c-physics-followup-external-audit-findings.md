# Phase 4.C Physics Follow-Up External Audit Findings

Audited working tree: changes after commit `3640023`.

Original audit mode: read-only, no network. I inspected code, docs, CI
configuration, and ran deterministic local gates. After the project owner
authorized fixes, this document was updated with the resolution status below.

## Executive Verdict

Resolved after follow-up fixes.

The physics extractions are real: dynamic pressure, kinematics, and innovation
gate statistics now live in `openbmp-physics`, FC delegates to them, and I did
not find duplicate helper implementations left in FC. The release CLI scenarios
requested by the audit were byte-stable across repeated runs.

The original audit found four issues. They are now addressed: the handover doc
no longer trips the benchmark-literal scanner, WGS84 const assertions run in
normal `openbmp-physics` builds, stale WGS84 docs were rewritten, and the
differential-flatness gravity sign has a focused regression test. The full
workspace test count is now `1083 passed, 0 failed, 5 ignored`.

## Claim-By-Claim Verification

| Claim | Verdict | Evidence |
| --- | --- | --- |
| Dynamic pressure moved to `openbmp-physics::atmosphere::dynamic_pressure_pa`; output byte-stable. | Verified. | `openbmp-fc/src/commander.rs` calls the physics helper with `USSA76_SEA_LEVEL_DENSITY_KG_M3`. The helper is the same `0.5 * density * velocity * velocity` operand order. Three release scenario replay pairs compared identical via `openbmp diff`. |
| `USSA76_SEA_LEVEL_DENSITY_KG_M3 = 1.225` is canonical. | Verified. | The constant is defined in `crates/openbmp-physics/src/atmosphere/mod.rs`. A sea-level USSA calculation from standard pressure, molar mass, gas constant, and temperature gives `1.224999155887712`, so the checked-in value is the canonical rounded density. |
| Kinematics primitives moved from FC to physics; no duplicates remain. | Verified. | `rg` finds `quaternion_from_axis_angle`, `quaternion_from_omega`, `renormalize_quaternion`, and `skew_symmetric` only in `crates/openbmp-physics/src/kinematics.rs` plus its tests. FC imports the physics functions. |
| `flatness_pd_accel` sign fix is real and correct. | Verified and now tested. | The code subtracts `standard_down_z_eci_m_s2()`, which is the correct way to add upward gravity compensation when standard gravity is negative Z. `autopilot::tests::flatness_pd_accel_adds_upward_gravity_compensation_at_trim` covers the zero-error trim case and requires upward gravity compensation. |
| Chi-square inverse CDF and Acklam inverse normal moved to physics; FC delegates. | Verified. | `innovation_gate_threshold` delegates to `openbmp_physics::statistics::chi_square_inverse_cdf_wilson_hilferty`; helper definitions only exist in `openbmp-physics/src/statistics.rs`. The spot tests cover known 3-DOF and 6-DOF values, invalid inputs, median, one-sigma, and symmetry. |
| WGS84 dual residence is guarded by `static_assertions`; check cfg/test-only and CI. | Verified after fix. | The constants are present in core and physics and the const assertions compare them in normal builds. `static_assertions` is now a regular `openbmp-physics` dependency, and `cargo +1.95 check -p openbmp-physics --locked` evaluates the guard. |
| Outstanding follow-up docs are complete. | Verified after fix. | The top-level follow-up section identifies the direct `FrameContext` consumer surface, the `openbmp-physics` README no longer claims WGS84 is re-exported from core, and `D-PC-9` now describes the interim dual residence plus final physics ownership target. |
| Test count grew from 1063 to 1083. | Verified after fix. | The exact workspace test run reported `1083 passed, 0 failed, 5 ignored`. The total includes the added differential-flatness regression. |

## Findings Table

| ID | Severity | Area | Status | Resolution |
| --- | --- | --- | --- | --- |
| F-1 | Medium | Test gate / audit artifact | Resolved | Rewrote the audit handover to cite WGS84 symbols instead of exact benchmark literals; the inline benchmark-constant tripwire now passes. |
| F-2 | Medium | WGS84 dual-residence guard | Resolved | Moved `static_assertions` to regular `openbmp-physics` dependencies and placed the const assertions in normal module scope. |
| F-3 | Low | Documentation consistency | Resolved | Updated the physics README and rewrote `D-PC-9` around interim dual residence and the final physics ownership target. |
| F-4 | Medium | Test coverage | Resolved | Added `flatness_pd_accel_adds_upward_gravity_compensation_at_trim` in `openbmp-fc::autopilot` tests. |

## Applied Fixes

1. Rewrote the exact WGS84 benchmark numeric literals in
   `docs/phase-4c-physics-followup-external-audit.md` by symbol name rather
   than adding an audit handover doc to the source-of-truth allow-list.

2. Strengthened the WGS84 dual-residence tripwire so the const assertions
   compile in normal `openbmp-physics` builds.

3. Cleaned up the stale WGS84 documentation in the `openbmp-physics` README
   module map and `D-PC-9` in `docs/physics-consolidation-plan.md`.

4. Added a focused regression test for the differential-flatness sign behavior.

## Test And Command Log

Passed:

- `cargo +1.95 fmt --all -- --check`
- `cargo +1.95 clippy --workspace --all-targets --all-features --locked -- -D warnings`
- `cargo +1.95 build --workspace --all-features --locked`
- `cargo +1.95 build -p openbmp-fc --no-default-features --locked`
- `cargo +1.95 test -p openbmp-fc --no-default-features --locked`
- `cargo +1.95 build -p openbmp-physics --no-default-features --locked`
- `cargo +1.95 test -p openbmp-physics --no-default-features --locked`
- `cargo +1.95 check -p openbmp-physics --locked`
- `cargo +1.95 check -p openbmp-physics --tests --locked`
- `cargo deny check` exited 0 with existing warning output for unmatched license allowances and duplicate dependency versions.
- `cargo machete`
- `cargo +1.95 build --release --bin openbmp --locked`
- `cargo +1.95 test -p openbmp-testkit --test inline_data_tripwire --locked no_inline_benchmark_constants_in_source`
- `cargo +1.95 test -p openbmp-fc --lib --locked flatness_pd_accel_adds_upward_gravity_compensation_at_trim`
- `cargo +1.95 test --workspace --all-features --locked`

Original audit failure, now fixed:

- `cargo +1.95 test --workspace --all-features --locked`
  - Failed in `openbmp-testkit` integration test
    `no_inline_benchmark_constants_in_source`.
  - The reported violations are exact WGS84 benchmark literals in the audit
    handover document.

Original partial result, now fixed:

- Exact test-count pipeline:
  `cargo test --workspace --all-features 2>&1 | grep "test result:" | awk ...`
  returned `907 passed, 1 failed, 3 ignored` before Cargo stopped. This does
  not verify the claimed total.

Post-fix test count:

- `cargo +1.95 test --workspace --all-features --locked` reported
  `1083 passed, 0 failed, 5 ignored`.

Release byte-stability checks:

- `scenarios/sounding-rocket/calisto/rocketpy-calisto.toml`
  - `openbmp diff: identical (180002 rows, 32 columns matched)`
  - SHA-256 of first output:
    `37fac1281cf8a370e127003152ffd7b790803a7328c02210b6329a006937bceb`
- `scenarios/closed-loop-attitude-hold/scenario.toml`
  - `openbmp diff: identical (1002 rows, 9 columns matched)`
  - SHA-256 of first output:
    `78722b2f2bf62241a9ecce8ba1667d0ec0c637eb5a078b36571a0cad9901e158`
- `scenarios/analytic-toy/constant-acceleration-drop.toml`
  - `openbmp diff: identical (1002 rows, 9 columns matched)`
  - SHA-256 of first output:
    `fdb52f47b236b77669db53f6e7d47fc3cd47e05881cbe2ef71949d7427fe36fb`

## Appendix: Exact Commands

Duplicate helper search:

```bash
rg -n "fn (skew_symmetric|quaternion_from_omega|quaternion_from_axis_angle|renormalize_quaternion|chi_square_inverse_cdf|inverse_standard_normal_cdf)" crates --glob '*.rs'
```

Differential-flatness coverage search:

```bash
rg -n "trajectory_kind\s*=\s*\"differential_flatness\"|DifferentialFlatness|differential_flatness" scenarios crates docs -S
```

Frame consumer search:

```bash
rg -n "FrameContext|LocalGeodeticOrigin" crates --glob '*.rs'
```

WGS84 dual-residence search:

```bash
rg -n "WGS84_A_M|WGS84_MU_M3_S2" crates/openbmp-core/src/frames.rs crates/openbmp-physics/src/gravity.rs crates/openbmp-testkit/tests/inline_data_tripwire.rs
```

FC dependency surface:

```bash
cargo tree -p openbmp-fc --all-features --locked
```

CI coverage spot-check:

```bash
rg -n "clippy --workspace|nextest run|cargo test -p openbmp-fc|cargo test -p openbmp-testkit|cargo test --doc" .github/workflows/ci.yml
```

Release scenario replay command shape:

```bash
cargo +1.95 build --release --bin openbmp --locked

base1=/tmp/openbmp-phase-4c-physics-followup-audit-a
base2=/tmp/openbmp-phase-4c-physics-followup-audit-b
rm -rf "$base1" "$base2"
mkdir -p "$base1" "$base2"

target/release/openbmp run scenarios/sounding-rocket/calisto/rocketpy-calisto.toml --output-parquet "$base1/calisto.parquet"
target/release/openbmp run scenarios/sounding-rocket/calisto/rocketpy-calisto.toml --output-parquet "$base2/calisto.parquet"
cmp "$base1/calisto.parquet" "$base2/calisto.parquet"
target/release/openbmp diff "$base1/calisto.parquet" "$base2/calisto.parquet"

target/release/openbmp run scenarios/closed-loop-attitude-hold/scenario.toml --output-parquet "$base1/closed-loop-attitude-hold.parquet"
target/release/openbmp run scenarios/closed-loop-attitude-hold/scenario.toml --output-parquet "$base2/closed-loop-attitude-hold.parquet"
cmp "$base1/closed-loop-attitude-hold.parquet" "$base2/closed-loop-attitude-hold.parquet"
target/release/openbmp diff "$base1/closed-loop-attitude-hold.parquet" "$base2/closed-loop-attitude-hold.parquet"

target/release/openbmp run scenarios/analytic-toy/constant-acceleration-drop.toml --output-parquet "$base1/constant-acceleration-drop.parquet"
target/release/openbmp run scenarios/analytic-toy/constant-acceleration-drop.toml --output-parquet "$base2/constant-acceleration-drop.parquet"
cmp "$base1/constant-acceleration-drop.parquet" "$base2/constant-acceleration-drop.parquet"
target/release/openbmp diff "$base1/constant-acceleration-drop.parquet" "$base2/constant-acceleration-drop.parquet"
```
