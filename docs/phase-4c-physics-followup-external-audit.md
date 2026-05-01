# Phase 4.C physics-followup external audit handover

> **Audience.** An external auditor. Independent of the prior author.
> No prior context with this codebase. Mandated to **verify**, not
> rubber-stamp.
>
> **Scope.** This audit covers the **uncommitted working tree** that
> follows commit `3640023` ("Phase 4.C external-audit fixes: WMM 2025
> wiring, GLRT/GM correctness, no silent gravity clamp"). The diff is
> small (10 modified files, 2 new files, ~150 net inserted lines) but
> touches load-bearing physics. The scope is narrower than the prior
> external audit; the bar is the same.
>
> **Stance.** Skeptical. The prior author wrote both the
> implementation AND the framing prose that justifies it. The
> external audit's job is to find where the framing lies, where the
> sign is wrong, where the test rubber-stamps, and where the
> "documented" trade-off is just an unexecuted shortcut.
>
> **Read-only.** Authorized to read every file, run any deterministic
> command (cargo build / test / clippy / fmt / deny / machete), and
> write a findings report. **Not** authorized to modify code, push
> commits, or rerun the prior author's loop. If a finding requires
> code changes, it goes in the report; the project owner decides what
> to do.

## What you're auditing

A **second-pass scan of `openbmp-fc`** for inline physics that
should live in `openbmp-physics`, and the partial consolidation that
followed. The motivating user instruction was: "physics primitives
should live in openbmp-physics."

Concretely, the working tree:

1. **Adds `openbmp-physics::statistics`** — a new module with
   `chi_square_inverse_cdf_wilson_hilferty` and
   `inverse_standard_normal_cdf` (the latter being Acklam's
   rational approximation). The functions, constants, and tests
   were extracted from `openbmp-fc::estimator` where they had
   lived since the prior audit landed FAR-derived innovation
   gates.

2. **Adds `openbmp-physics::kinematics`** — a new module with four
   rigid-body math primitives: `quaternion_from_axis_angle`,
   `quaternion_from_omega`, `renormalize_quaternion`, and
   `skew_symmetric`. They were inline in
   `openbmp-fc::estimator` and used by the EKF and MEKF predict
   / update paths. Six new tests are included.

3. **Extends `openbmp-physics::atmosphere`** with
   `dynamic_pressure_pa(density, velocity)` and a
   `USSA76_SEA_LEVEL_DENSITY_KG_M3 = 1.225` constant. Both replace
   an inline `0.5 * 1.225 * v_z²` formula in
   `openbmp-fc::commander`.

4. **Fixes a sign error** in
   `openbmp-fc::autopilot::flatness_pd_accel`. The function used
   `- Vector3::new(0, 0, STANDARD_GRAVITY_M_S2)` for gravity
   feedforward, which the prior author claims was the wrong sign.
   The fix replaces the inline vector with
   `- standard_down_z_eci_m_s2()`. **The prior author's claim is
   that this both consolidates and fixes a real bug.** That claim
   is one of your main verification targets.

5. **Documents WGS84 dual residence**. `WGS84_A_M` and
   `WGS84_MU_M3_S2` are inline in **both** `openbmp-core::frames`
   and `openbmp-physics::gravity`; inspect those source files for
   the exact benchmark literals. The prior author's first attempt
   was to dedupe by re-exporting from core. The user pushed back
   ("physics primitives should live in openbmp-physics"). The author
   then considered the proper full consolidation (move
   `FrameContext` + `LocalGeodeticOrigin` + WGS84 from core to
   `physics::frames`), declined to execute it mid-turn, and instead
   landed:

    - inline definitions in **both** locations,
    - a `static_assertions::const_assert!` tripwire in
      `physics::gravity::wgs84_dual_residence_tripwire` that
      verifies the values match,
    - a documented "Outstanding follow-up" section in
      `docs/physics-consolidation-plan.md` and the physics README.

   The dual residence is the prior author's **acknowledged
   architectural debt**, not a clean state. Your audit decides
   whether the tripwire + documentation are sufficient guard rails
   or whether the half-consolidated state is itself a finding.

6. **Cosmetic test-fixture cleanup** in
   `openbmp-fc::health.rs::tests` (a magic `9.81` is replaced
   with `STANDARD_GRAVITY_M_S2`).

## Key claims to verify

The prior author makes specific claims. Each is a hypothesis to
falsify:

1. **"`commander.rs` dynamic-pressure formula was inline; now uses
   `physics::atmosphere::dynamic_pressure_pa`."** Check that the
   commander's call site produces byte-identical output for the
   pre-existing input range. The formula is `q = 0.5 · ρ · v²`;
   if the operand order changed, byte-stable scenarios diff. Run
   the regression scenarios.

2. **"`USSA76_SEA_LEVEL_DENSITY_KG_M3 = 1.225` is the canonical
   USSA76 sea-level density."** Verify against the standard.
   The exact USSA76 sea-level density is computed as `p / (R · T)`
   with the model's mean molecular weight: that gives
   1.224999... kg/m³, conventionally rounded to 1.225. Confirm
   the prior author didn't import a less-precise rounded value or
   silently use 1.2 / 1.23 / 1.25.

3. **"Four kinematics primitives moved from FC to physics."**
   Verify by `grep`:
   - `crates/openbmp-fc/src/estimator.rs` should NOT contain `fn
     skew_symmetric`, `fn quaternion_from_omega`, `fn
     quaternion_from_axis_angle`, `fn renormalize_quaternion`.
   - `crates/openbmp-physics/src/kinematics.rs` should be the
     single workspace home for them.
   - All callers should `use openbmp_physics::kinematics::...` (or
     prefix-qualified equivalent).
   No duplication across the workspace.

4. **"Sign fix in `flatness_pd_accel` is a real bug fix, not just a
   consolidation."** Audit:
   - Convention: `gravity_eci = (0, 0, -STANDARD_GRAVITY_M_S2)`,
     i.e. ECI z is up and gravity points down. Confirm by reading
     `openbmp-physics::gravity::standard_down_z_eci_m_s2` —
     should return `Vector3::new(0, 0, -STANDARD_GRAVITY_M_S2)`.
   - Specific force = inertial_accel − gravity_eci. For PD
     trajectory tracking with desired inertial acceleration =
     `pd_correction`, specific-force feedforward =
     `pd_correction − gravity_eci = pd_correction + (0, 0, +g)`.
   - Old code: `pd_correction − Vector3::new(0, 0, +g) =
     pd_correction + (0, 0, −g)`. Specific force pointing **down**
     by g — that's free-fall, not hover-trim.
   - New code: `pd_correction − standard_down_z_eci_m_s2() =
     pd_correction − (0, 0, −g) = pd_correction + (0, 0, +g)`.
     Specific force pointing **up** by g — correct.
   - **Severity calibration:** the prior author claims this is
     "high severity, also a bug" but also notes the path is
     unexercised (`closed-loop-attitude-hold/scenario.toml` uses
     `trajectory_kind = "pid"`, no test exercises differential-
     flatness). So the bug ships, but no scenario triggers it. The
     correctness claim and the severity claim should both stand
     under your scrutiny.

5. **"Chi-square inverse CDF moved cleanly; FC delegates to
   physics."** Verify:
   - `crates/openbmp-fc/src/estimator.rs` no longer contains the
     ~70 lines of Acklam coefficients or the Wilson-Hilferty
     formula.
   - `crates/openbmp-physics/src/statistics.rs` has them.
   - The FC's `innovation_gate_threshold` calls
     `openbmp_physics::statistics::chi_square_inverse_cdf_wilson_hilferty`.
   - All Acklam coefficients are bit-identical to the previous FC
     definition; if any drifted, byte-stable behavior changes.
   - The new tests in `physics::statistics`:
     - `chi_square_inverse_known_3dof_99pct` — claims tabulated
       value 11.345 with epsilon 0.05. Confirm 11.345 is the
       canonical tabulated value (one published reference is
       11.3449); 0.05 epsilon is loose enough to mask sub-percent
       errors. Tighter epsilon is preferable.
     - `chi_square_inverse_known_6dof_99pct` — same shape.
     - `standard_normal_inverse_at_median_is_zero` — Z(0.5) = 0.

6. **"WGS84 dual residence is guarded by `static_assertions`
   tripwire."** Verify:
   - `crates/openbmp-physics/src/gravity.rs` has a
     `wgs84_dual_residence_tripwire` test module with
     `static_assertions::const_assert!(WGS84_A_M ==
     openbmp_core::WGS84_A_M)` and the same for `WGS84_MU_M3_S2`.
   - The assertion fires at compile time, not at run time —
     verify by deliberately changing one constant in a scratch
     branch and confirming `cargo build` fails. (This is a
     `target/audit-scratch/` exercise; do NOT modify the working
     tree.)
   - The inline-data tripwire's allow-list correctly lists both
     `crates/openbmp-physics/src/gravity.rs` and
     `crates/openbmp-core/src/frames.rs` for the literals.

7. **"Outstanding follow-up is documented and tractable."** Read
   `docs/physics-consolidation-plan.md § Outstanding follow-up` and
   `crates/openbmp-physics/README.md § Outstanding follow-up`.
   Verify:
   - Both docs name the same follow-up scope.
   - The consumer surface listed (physics::magnetic::wmm2025,
     physics::wind, openbmp-cli::runner::wind,
     openbmp-cli::runner::phase2_*) is **complete** — no other
     crate transitively touches `FrameContext` methods. `grep -rln
     "FrameContext\|LocalGeodeticOrigin" crates/` should return
     only the listed consumers + core itself.
   - The "we will execute this in a separate commit gated on user
     authorisation" framing is honest, not a way to indefinitely
     defer.

8. **"Test count is now 1083."** Run
   `cargo test --workspace --all-features` yourself, sum the test
   counts, confirm. The expected sources of new/follow-up tests:
   - `physics::kinematics::tests` (7 tests)
   - `physics::statistics::tests` (6 tests)
   - `fc::autopilot::tests::flatness_pd_accel_adds_upward_gravity_compensation_at_trim`
   - existing tests unchanged
   If the count is materially different, the prior author
   miscounted or hidden test moved.

## Audit dimensions

### A. Honesty — does the diff match the framing?

Open `git diff HEAD`. For each modified file:
- Is the diff what the framing prose claims it is?
- Are there incidental changes the framing didn't mention?
- Does the dual-residence tripwire actually compile-time-check, or
  is it a runtime check that could pass with diverged values for
  most inputs?

### B. Soundness — is the math correct?

Pick three:
- **Wilson-Hilferty chi-square inverse**: derive the formula from
  the cited paper (Wilson & Hilferty 1931). Confirm operand order
  and the cube exponent.
- **Acklam inverse normal CDF**: spot-check 3-5 coefficients
  against the published Acklam table. Coefficients are
  numerically sensitive — a single sign flip breaks the
  approximation in one tail.
- **Skew-symmetric matrix**: verify
  `skew(v) · w = v × w` for a few non-trivial `(v, w)` pairs.
  The convention has two valid sign conventions; verify the
  workspace consistently uses the one shipped here.
- **Dynamic pressure**: `q = 0.5 · ρ · v²` is textbook. Verify the
  helper computes this and not `0.5 · ρ · v · v` (different
  operand order) which could differ at the bit level under FMA.

### C. Architecture — does the consolidation hold?

- Does any consumer of the moved primitives still inline its own
  copy? `grep` for the specific function names.
- Does `openbmp-fc` still depend transitively on
  `openbmp-physics` only (HAL portability)?
- Does `cargo build -p openbmp-fc --no-default-features` still
  succeed after the moves?
- Does the `static_assertions` tripwire compile when
  `cargo check -p openbmp-physics` is invoked, or only when the
  test target is built? If the tripwire is `#[cfg(test)]`, it
  doesn't fire in a normal build — verify the user's intended
  guard rail actually fires when expected.

### D. Determinism — did the moves change byte-stable scenarios?

The prior consolidations preserved byte-stability because operand
order was preserved. This consolidation makes function-body moves
(same operand order). But the dynamic-pressure helper changes the
expression: from `0.5 * 1.225 * vertical_velocity_m_s *
vertical_velocity_m_s` to
`dynamic_pressure_pa(USSA76_SEA_LEVEL_DENSITY_KG_M3,
vertical_velocity_m_s)` which expands to
`0.5 * density_kg_m3 * velocity_m_s * velocity_m_s`.

Bit-identity check:
- `0.5 * 1.225 * v * v` vs `0.5 * 1.225 * v * v` — same expression
  if the constant binds to `density_kg_m3 = 1.225`. Verify the
  `dynamic_pressure_pa` function body matches the inline form
  exactly.
- Run the byte-stable Phase-1/2/3 scenarios twice and diff
  Parquet. Repeat for `closed-loop-attitude-hold`.

### E. Test truthfulness

- `physics::kinematics::tests`: do any of the 6 tests reduce to
  trivial assertions (e.g. `is_finite`)? Tight tolerances or
  loose? `quaternion_from_axis_angle_pi_around_z_flips_x` should
  show `(1, 0, 0) → (-1, 0, 0)` — verify the test makes that
  exact assertion.
- `physics::statistics::tests`: epsilon 0.05 on tabulated chi^2
  values — is this loose enough to mask a real drift? Compare
  against the actual tabulated precision: chi²(0.99, 3) =
  11.34487, chi²(0.99, 6) = 16.81189. Wilson-Hilferty at 3 DOF
  has ~0.2% error, ~0.05% at 6 DOF. So the epsilon is calibrated
  to the WH approximation's known accuracy, not arbitrary.
- `physics::statistics::standard_normal_inverse_one_sigma_known_value`:
  epsilon 1.0e-6 on Acklam — is this realistic? Acklam claims
  4.5e-9 maximum error. The 1.0e-6 tolerance is loose. Could be
  tightened to 1.0e-7 without false failures.

### F. Risk surface

- The dual residence of WGS84 constants is acknowledged as debt.
  What's the realistic likelihood that someone updates one
  without the other? The `static_assertions` catches it at
  compile time — but only when `cargo check -p openbmp-physics
  --tests` runs. Does CI run that target?
- The sign fix in `flatness_pd_accel` is a behavior change. The
  prior author claims no scenario exercises it. Verify by `grep`
  for `differential_flatness` / `trajectory_kind`. If a test
  scenario uses it, byte-stability breaks.
- The chi-square inverse path goes through the same Wilson-
  Hilferty code as before, just relocated. But the FC's call site
  changed from a free-function call to an
  `openbmp_physics::statistics::...` qualified path. Confirm no
  shadow-namespacing issue (e.g. accidentally calling a different
  function with the same name).

## Specific code locations

These are the high-yield places to investigate:

- `crates/openbmp-physics/src/kinematics.rs` (new) —
  4 functions + 6 tests + the small-rotation cutoff constant.
- `crates/openbmp-physics/src/statistics.rs` (new) —
  Acklam coefficients + Wilson-Hilferty + tests.
- `crates/openbmp-physics/src/atmosphere/mod.rs` —
  `USSA76_SEA_LEVEL_DENSITY_KG_M3` constant addition,
  `dynamic_pressure_pa` helper.
- `crates/openbmp-physics/src/gravity.rs` —
  the `wgs84_dual_residence_tripwire` test module.
- `crates/openbmp-physics/src/lib.rs` —
  the `pub use` lines for new modules and types.
- `crates/openbmp-fc/src/estimator.rs` —
  imports and removed code.
- `crates/openbmp-fc/src/commander.rs` (lines ~140–155) —
  dynamic-pressure call site.
- `crates/openbmp-fc/src/autopilot.rs` (lines ~463–478) —
  the sign-fix site (`flatness_pd_accel`).
- `crates/openbmp-fc/src/health.rs` (line 232) —
  the test-fixture cosmetic.
- `crates/openbmp-testkit/tests/inline_data_tripwire.rs` —
  the WGS84 needle allow-list updates.
- `docs/physics-consolidation-plan.md § Outstanding follow-up` —
  the FrameContext + WGS84 follow-up framing.
- `crates/openbmp-physics/README.md § Outstanding follow-up` —
  the same in the README.

## Reproduction commands

```bash
# Mechanical gates (run all of them)
cargo +1.95 fmt --all -- --check
cargo +1.95 clippy --workspace --all-targets --all-features --locked -- -D warnings
cargo +1.95 build --workspace --all-features --locked
cargo +1.95 test --workspace --all-features --locked
cargo +1.95 build -p openbmp-fc --no-default-features --locked
cargo +1.95 test -p openbmp-fc --no-default-features --locked
cargo +1.95 build -p openbmp-physics --no-default-features --locked
cargo +1.95 test -p openbmp-physics --no-default-features --locked
cargo deny check
cargo machete

# Test-count audit
cargo test --workspace --all-features 2>&1 | grep "test result:" | \
  awk '{ p+=$4; f+=$6 } END { print "passed:", p, "failed:", f }'
# Expected: 1083 passed, 0 failed. Anything else is a finding.

# Byte-stable scenario regression
mkdir -p /tmp/audit-physics-followup-{1,2}
for scenario in scenarios/sounding-rocket/calisto/rocketpy-calisto.toml \
                scenarios/closed-loop-attitude-hold/scenario.toml \
                scenarios/analytic-toy/constant-acceleration-drop.toml; do
  cargo run --bin openbmp --release -- run "$scenario" \
    --output /tmp/audit-physics-followup-1 2>/dev/null || continue
  cargo run --bin openbmp --release -- run "$scenario" \
    --output /tmp/audit-physics-followup-2 2>/dev/null || continue
  # diff the produced Parquet files; report any non-zero diff
  for f in /tmp/audit-physics-followup-1/*.parquet; do
    base=$(basename "$f")
    if ! cmp "$f" "/tmp/audit-physics-followup-2/$base" >/dev/null 2>&1; then
      echo "BYTE-STABLE DIFF: $scenario / $base"
    fi
  done
done

# Verify FC no longer carries kinematics or chi-square inline
grep -n "fn skew_symmetric\|fn quaternion_from_omega\|fn quaternion_from_axis_angle\|fn renormalize_quaternion\|fn chi_square_inverse_cdf\|fn inverse_standard_normal_cdf" \
  crates/openbmp-fc/src/*.rs
# Expected: no matches. Any match is a regression.

# Verify single workspace home for moved primitives
grep -rln "fn skew_symmetric\|fn quaternion_from_omega" --include='*.rs' crates/
# Expected: only crates/openbmp-physics/src/kinematics.rs.

# Static_assertions tripwire fires at compile time
cargo check -p openbmp-physics --tests 2>&1 | head -10
# Expected: clean build. Now temporarily edit
# crates/openbmp-physics/src/gravity.rs to set
# WGS84_A_M by a tiny amount and rerun. Expected: const-eval
# error pointing at the const_assert. Revert before continuing.
# (This requires a working-tree mod — skip this exercise unless
# explicitly authorized; if skipped, document the unverified
# assumption in your findings.)

# WGS84 duplication is the only acknowledged debt
grep -n "WGS84_A_M\b" crates/openbmp-core/src/frames.rs crates/openbmp-physics/src/gravity.rs
grep -n "WGS84_MU_M3_S2\b" crates/openbmp-core/src/frames.rs crates/openbmp-physics/src/gravity.rs
# Expected: each constant defined exactly twice (once in core, once in physics).
# Each definition must equal the other.
```

## Deliverables

`docs/phase-4c-physics-followup-external-audit-findings.md` (you
write it). Sections:

```markdown
# Phase 4.C physics-followup external audit findings

Auditor: <your name>
Date: <ISO 8601>
Working tree at: <git rev-parse HEAD> + uncommitted diff
Toolchain: <rustc --version>

## Executive summary

3-5 sentences. Are the prior author's claims about this round
(chi-square move, kinematics consolidation, dynamic-pressure
helper, sign fix, dual-residence tripwire, outstanding follow-up
framing) load-bearing-true?

## Verified claims

| Claim | Status | Evidence |
|---|---|---|
| Test count is 1083 | ... | output |
| HAL portability preserved | ... | ... |
| Sign fix in flatness_pd_accel is mathematically correct | ... | derivation |
| Chi-square inverse Wilson-Hilferty matches the cited paper | ... | derivation |
| ... | ... | ... |

## Unverified or false claims

| Claim | Severity | What I found | Evidence |
|---|---|---|---|

## Algorithm soundness deep-dives

For each algorithm audited:
- Reference
- Implementation file:line
- Math derivation (one paragraph)
- Verdict
- Evidence

## Architectural observations

The dual-residence of WGS84 constants is the single largest
architectural finding to evaluate:
- Is the static_assertions tripwire load-bearing?
- Is the documentation of the follow-up sufficient, or is it a
  shortcut that should be flagged?

## Risk findings

For each risk: severity, description, reproduction.

## Tests of suspect rigor

For each rubber-stamp test found: file:line, what it claims, what
it verifies.

## Sign-off

Whether the working tree is fit for the user to commit, or whether
findings need addressing first.
```

## Authorization scope

You may:
- Read every file in the repo.
- Run any `cargo` command (build, test, clippy, fmt, deny,
  machete, run, doc, tree).
- Run any deterministic shell command for inspection (grep, find,
  diff, sha256sum, cmp).
- Write your findings to
  `docs/phase-4c-physics-followup-external-audit-findings.md`.
- Write throwaway test files (call them
  `target/audit-scratch/*.rs`) to verify algorithm claims.

You may NOT:
- Modify production code, scenarios, data files, or the prior
  author's documentation.
- Stage or commit changes (other than the findings doc you write).
- Push, force-push, branch-delete, rebase, or reset.
- Run anything that requires network access (no `curl`,
  `cargo install`, `cargo update`, `git fetch`).
- Modify dependencies or `Cargo.lock`.
- Disable any test, lint, or feature gate.

If verification requires code modification, file the finding and
stop. Don't fix.

## Non-goals

- Style preferences. Workspace lints already enforce.
- Speculative future-phase work. Phase 5 is its own scope.
- Re-running the prior external audit
  (`docs/phase-4c-external-audit.md`). That work is done; you're
  auditing a follow-up.
- Litigating the user's pushback against the re-export approach.
  The user said "physics primitives should live in
  openbmp-physics," the prior author chose dual-residence-with-
  tripwire as the pragmatic compromise, and the full
  consolidation is documented as outstanding. Your job is to
  decide whether the compromise + the documented follow-up are
  load-bearing under the user's stated principle, or whether the
  half-state is itself a finding.

## Tone expectations

- **Don't soften findings.** "The sign fix changes behavior on the
  differential-flatness path" is a finding. "Maybe consider
  adding a test" is not.
- **Don't speculate without evidence.** "The chi-square move
  might affect determinism" is not a finding. "Running scenario
  X twice produces a 4-byte Parquet diff at row 12,341, column 7,
  caused by operand order in dynamic_pressure_pa" is.
- **Don't accept "but the tests pass" as defense.** The prior
  audit pattern caught real bugs precisely because the tests
  were rubber-stamping. Verify what each test actually checks.
- **Cite references.** Every algorithmic claim against the
  implementation should cite a paper, textbook, or primary
  source.
- **Be specific about severity.** `high`: a load-bearing claim is
  false (e.g. tests don't pass, sign fix is itself wrong, byte-
  stability broke). `medium`: a claim is overstated or misleading
  (e.g. Acklam test tolerance is loose enough to mask a real
  drift). `low`: a claim is technically correct but inelegant or
  contains stale framing prose.

## A specific challenge — the dual residence

The prior author landed an unusual architectural state: WGS84
constants inline in two crates simultaneously, guarded by a
compile-time tripwire. The prior author's defense:

> The proper consolidation requires moving `FrameContext` +
> `LocalGeodeticOrigin` + WGS84 from `openbmp-core` to
> `openbmp-physics::frames`. That's a workspace-shape change
> comparable to the `openbmp-env` retirement. Doing it mid-turn
> without explicit user authorisation would have been overreach.
> The dual residence with `static_assertions` is the documented
> intermediate state; the full consolidation is tracked.

Test that defense adversarially:

1. **Is the workspace-shape change actually larger than what's
   already been done in this Phase 4.C round?** Count the
   consumer surface (files that import `FrameContext` or
   `LocalGeodeticOrigin`). Compare to the Phase 4.C consolidation
   that retired `openbmp-env` (which moved ~3 500 lines and
   touched dozens of files). If the surface is small, "needs
   user authorisation" is a punt.

2. **Does the static_assertions tripwire actually fire when
   expected?** It's `#[cfg(test)]`-gated, meaning it only fires
   under `cargo test` or `cargo check --tests`, not under plain
   `cargo build`. Confirm CI runs at least one of those. If a
   downstream consumer builds without `--tests`, the tripwire is
   silent and drift can ship.

3. **Is the documentation of the outstanding follow-up
   load-bearing, or is it the kind of "we'll get to it" framing
   that gets quietly forgotten?** Verify the
   "Outstanding follow-up" section names a concrete consumer
   surface, a definition of done, and a tractable scope. If it's
   vague, the documentation is shelf-decoration.

4. **Could the prior author have done the full consolidation in
   the working tree without breaking byte-stability?** The
   `FrameContext` methods consume `WGS84_A_M`,
   `WGS84_OMEGA_RAD_S`, etc. Moving them with the constants
   would preserve operand order. The prior author's "workspace-
   shape change" framing implies risk; assess whether the risk is
   real or rhetorical.

This isn't a witch-hunt; it's a stress-test of the prior author's
reasoning. Sometimes the dual residence is the right intermediate
state. Sometimes it's a shortcut. The audit's job is to tell the
difference.

## Sign-off

The audit is complete when **all** of:

1. Every claim in the "Key claims to verify" table has a
   verified-or-refuted entry in the findings doc.
2. The sign-fix correctness in `flatness_pd_accel` has been
   independently derived from convention.
3. The Wilson-Hilferty and Acklam algorithms have been spot-
   checked against their cited papers.
4. The byte-stable scenarios have been re-run and any diff has
   been reported with file:row:column precision.
5. The HAL-portability claim has been verified by running the
   no-default-features build.
6. The dual-residence architectural debt has an opinionated
   verdict: acceptable as an intermediate state, or itself a
   finding.
7. The findings doc is self-contained and reproduces every
   verification command.

Project owner reads the findings and decides what to do (commit
as-is, fix findings first, or hold the working tree pending
clarification).
