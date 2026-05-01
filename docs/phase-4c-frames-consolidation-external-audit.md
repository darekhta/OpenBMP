# Phase 4.C frames + WGS84 consolidation external audit handover

> **Audience.** An external auditor. Independent of the prior author.
> No prior context with this codebase. Mandated to **verify**, not
> rubber-stamp.
>
> **Scope.** This audit covers commit `71edfdf` ("Phase 4.C: move
> FrameContext + WGS84 from openbmp-core to openbmp-physics::frames").
> The commit is small in net lines (847 inserted, 822 deleted across
> 18 files; one new file) but reshapes a workspace boundary that had
> been load-bearing since Phase 1. The prior physics-followup audit
> (`docs/phase-4c-physics-followup-external-audit.md` and findings)
> identified the dual-residence-with-tripwire state as acknowledged
> architectural debt and named the consolidation in this commit as
> the principled fix. Your audit decides whether the fix actually
> lands what the prior audit asked for, and whether anything broke
> on the way.
>
> **Stance.** Skeptical. The prior author is also the author who
> previously introduced the dual residence, then defended it as
> "needs user authorisation," then executed the consolidation in a
> single commit when the user said go. The framing prose may be
> doing more work than the code.
>
> **Read-only.** Authorized to read every file, run any deterministic
> command (cargo build / test / clippy / fmt / deny / machete), and
> write a findings report. **Not** authorized to modify code, push
> commits, or rerun the prior author's loop. If a finding requires
> code changes, it goes in the report; the project owner decides what
> to do.

## What the commit claims

1. **Moves WGS84 constants** out of `openbmp-core::frames` and into
   `openbmp-physics::frames`. The six constants are `WGS84_A_M`,
   `WGS84_INV_FLATTENING`, `WGS84_FLATTENING`,
   `WGS84_ECCENTRICITY_SQUARED`, `WGS84_MU_M3_S2`,
   `WGS84_OMEGA_RAD_S`. Values are unchanged.

2. **Moves `LocalGeodeticOrigin`, `FrameProfile`, `FrameContext`,
   the `FrameTransform` trait, and all `FrameTransform` impls** from
   `openbmp-core::frames` to `openbmp-physics::frames`. Methods on
   `FrameContext` (`earth_rotation_angle`, `eci_to_ecef_position`,
   `ecef_to_eci_position`, `eci_to_ecef_velocity`,
   `ecef_to_eci_velocity`, `angular_velocity_z`,
   `ecef_to_ned_position`, `ecef_to_ned_velocity`,
   `ned_to_ecef_velocity`) are unchanged in body.

3. **Drops the dual-residence guard.** The previous state declared
   `WGS84_A_M` and `WGS84_MU_M3_S2` inline in **both**
   `openbmp-core::frames` and `openbmp-physics::gravity`, with a
   `static_assertions::const_assert!` pair at module scope in
   `openbmp-physics::gravity` proving the values matched.
   `openbmp-physics::gravity` now imports the constants from
   `crate::frames`; the `const_assert!` lines are gone.

4. **Trims `openbmp-core::frames` to type-level machinery only.**
   What stays: `Frame` trait, frame tag types (`Eci`, `Ecef`, `Ned`,
   `Enu`, `Body`), `FrameId`, value types (`Position3<F>`,
   `Velocity3<F>`, `Acceleration3<F>`, `Displacement3<F>`,
   `VelocityDelta3<F>`, `AngularVelocity3<F>`),
   `Quaternion<From, To>` rotation, and `FrameError` (which is in
   `core::error` but referenced by `core::frames::Quaternion`).
   What leaves: every Earth-specific physics constant, every
   time-aware transform, the `FrameProfile` enum, the
   `FrameTransform` trait and impls, and the WGS84 + FrameContext
   tests.

5. **Migrates consumers.** Files updated to import the moved items
   from `openbmp_physics::frames` (or `openbmp_physics`):
   - `crates/openbmp-physics/src/magnetic/wmm2025.rs`
   - `crates/openbmp-physics/src/magnetic/mod.rs` (doc reference)
   - `crates/openbmp-physics/src/wind/constant.rs`
   - `crates/openbmp-physics/src/wind/gust.rs`
   - `crates/openbmp-physics/src/wind/layered.rs`
   - `crates/openbmp-physics/src/wind/mod.rs`
   - `crates/openbmp-physics/tests/regression.rs`
   - `crates/openbmp-cli/src/runner/wind.rs`
   - `crates/openbmp-cli/src/runner/phase2_point_mass.rs`
   - `crates/openbmp-cli/src/runner/phase2_rigid_body.rs`

6. **Updates the WGS84 inline-data tripwire allow-list** in
   `crates/openbmp-testkit/tests/inline_data_tripwire.rs` so the
   WGS84 GM and equatorial-radius needles are allow-listed only at
   `crates/openbmp-physics/src/frames.rs` (not at
   `crates/openbmp-physics/src/gravity.rs` or
   `crates/openbmp-core/src/frames.rs`).

7. **Updates documentation.** Both
   `crates/openbmp-physics/README.md` and
   `docs/physics-consolidation-plan.md` reflect the completed
   consolidation; the plan's "Outstanding follow-up" section is
   marked completed with date 2026-04-30.

## Key claims to verify

The prior author makes specific claims. Each is a hypothesis to
falsify:

1. **"All six WGS84 constants live in exactly one place."**
   `grep` for each constant name across `crates/`. Each should
   appear once as a `pub const` definition (in
   `crates/openbmp-physics/src/frames.rs`) and zero times as an
   inline definition elsewhere. Imports / references are fine; a
   second `pub const` with the same name is the failure.

2. **"WGS84 numeric values are bit-identical to the prior commit."**
   `git show 71edfdf~1:crates/openbmp-core/src/frames.rs | grep
   WGS84_` and compare every literal to the new
   `openbmp-physics/src/frames.rs` definition. Any digit drift is a
   high-severity finding because `frame_transform_trait_is_time_aware_for_wgs84_eci_ecef`
   and the byte-stable scenario regressions silently consume these.

3. **"FrameContext method bodies are unchanged."** For each method
   listed in claim #2 above, diff the pre-commit body in
   `core/frames.rs` against the post-commit body in
   `physics/frames.rs`. Operand order, sign convention, and helper
   composition must be byte-identical or downstream byte-stable
   scenarios diff. Pay special attention to
   `eci_to_ecef_velocity` and `ecef_to_eci_velocity` (transport-
   rate cross product), and `ecef_to_ned_position` /
   `_to_ecef_velocity` / `_to_ned_velocity` (geodetic-normal
   rotation matrix).

4. **"Tests moved together with the code."** Before commit,
   `core/frames.rs` carried (a) value-type / Quaternion tests and
   (b) `FrameContext` + WGS84 tests. After commit, only (a) is in
   core; (b) is in `physics/frames.rs`. Verify by counting `#[test]`
   functions in each file pre/post and matching them by name. Any
   test that was deleted rather than moved is a finding (not
   automatically a high-severity one — sometimes deletion is right,
   but it should be deliberate, and the commit message doesn't list
   any test deletions).

5. **"No consumer outside the listed set transitively touches the
   moved items."** Run:
   ```bash
   grep -rln "FrameContext\|LocalGeodeticOrigin\|FrameProfile\|FrameTransform\|WGS84_A_M\|WGS84_INV_FLATTENING\|WGS84_FLATTENING\|WGS84_ECCENTRICITY_SQUARED\|WGS84_MU_M3_S2\|WGS84_OMEGA_RAD_S" \
     crates/ --include='*.rs' | sort
   ```
   The expected set is:
   - `crates/openbmp-cli/src/runner/{phase2_point_mass,phase2_rigid_body,wind}.rs`
   - `crates/openbmp-core/src/{error,frames,lib}.rs` (only `FrameError` mentions for `error.rs`; only doc refs / type re-exports for the others; **no Earth-specific physics**)
   - `crates/openbmp-physics/src/{frames,gravity,lib}.rs`
   - `crates/openbmp-physics/src/magnetic/{mod,wmm2025}.rs`
   - `crates/openbmp-physics/src/wind/{constant,gust,layered,mod}.rs`
   - `crates/openbmp-physics/tests/regression.rs`
   - `crates/openbmp-vehicle/src/adapters.rs` (one stale doc reference; verify it's just a doc, not a use site that would mean the migration missed a consumer)

   Any file in `state`, `sensors`, `mission`, `scenario`,
   `propulsion`, `models`, `sim`, `fc`, `aero`, `aerothermal`,
   `telemetry`, or `testkit` (other than the tripwire allow-list)
   that names one of these symbols is a missed consumer.

6. **"`openbmp-core::error::FrameError` still works for code paths
   that need it."** `FrameError::LocalOriginRequired` and
   `FrameError::TransformNotAvailable` carry `profile: &'static
   str`, which the prior implementation populated from
   `FrameProfile::as_label()`. After the move, `FrameProfile` is in
   `openbmp-physics`, but `FrameError` is still in `openbmp-core`.
   Verify the variants still exist with the same fields, and
   verify `FrameContext::require_local_origin` (now in
   `physics/frames.rs`) still constructs them with a non-empty
   profile label.

7. **"`static_assertions` dep is still used somewhere."** The
   `const_assert!` for WGS84 is gone; if `static_assertions` was
   used only there, the dep is now dead. Run `cargo machete` and /
   or `grep -rn "static_assertions" crates/openbmp-physics/`.
   `wind/gust.rs` should still use it (`assert_impl_all`,
   `assert_not_impl_any`); if so, the dep remains live. If those
   uses are gone too, the dep is dead and should be removed.

8. **"`openbmp-core` lib doc is up to date."** Read
   `crates/openbmp-core/src/lib.rs` lines 1–55. The crate-level doc
   now states that frame transforms live in `openbmp-physics::frames`.
   Verify the documentation does not still claim
   `FrameContext` / `FrameTransform` are part of `openbmp-core`,
   and that the public re-exports from `frames::` no longer name
   the moved items.

9. **"Tripwire allow-list is correctly tightened."** Open
   `crates/openbmp-testkit/tests/inline_data_tripwire.rs`. The
   WGS84 GM, WGS84 J2, and WGS84 equatorial-radius needles each have
   a tightened allow-list:
   - WGS84 GM needle: expected at
     `data/gravity/wgs84-j2.toml`,
     `docs/data-provenance.md`,
     `crates/openbmp-physics/src/frames.rs`,
     `crates/openbmp-testkit/tests/inline_data_tripwire.rs`.
     Must NOT include `crates/openbmp-core/src/frames.rs` or
     `crates/openbmp-physics/src/gravity.rs`.
   - WGS84 equatorial-radius needle: same shape.
   - WGS84 J2 unnormalised needle: unchanged — J2 lives
     only in physics::gravity.

   Run `cargo test -p openbmp-testkit --test inline_data_tripwire`
   and confirm pass. Then run a `grep -rn "3\.986004418\|6378137\.0"
   crates/ docs/ data/` to enumerate every occurrence and
   cross-check it against the allow-list. Anything in source
   outside the allow-list is a high-severity finding (the test
   should have caught it; if it didn't, there's a tripwire bug).

10. **"Documentation matches reality."** Read both
    `crates/openbmp-physics/README.md` (Layering with `openbmp-core`
    section) and
    `docs/physics-consolidation-plan.md` (Frames + WGS84 constants
    consolidation section). Verify:
    - The "completed 2026-04-30" framing accurately describes the
      working tree at this commit.
    - The set of moved items in the README matches the set in
      `physics/frames.rs`.
    - No section still says "outstanding," "deferred," "follow-up,"
      or "to be done" with respect to this consolidation.
    - The README's "Module map" section is consistent with the
      crate's actual `pub mod` lines in `lib.rs`.

## Audit dimensions

### A. Honesty — does the diff match the framing?

`git show 71edfdf` and read the body of every modified file. For
each:
- Is the diff what the commit message claims it is?
- Are there incidental changes the commit didn't mention (renamed
  imports, drive-by clippy fixes, formatting)?
- Did any test get weaker (epsilon loosened, assertion deleted) on
  the way through the move?
- Are imports tidy? Did the move leave behind unused imports
  anywhere (compiler warnings turn into hard errors under `-D
  warnings`, so the build catches it; but it's worth checking
  `clippy --workspace --all-targets -- -D warnings` exit code)?

### B. Soundness — is the math still correct?

The math in this commit is supposed to be a pure relocation. Pick
three:

- **WGS84 GM and a**: derive `WGS84_A_M` and `WGS84_MU_M3_S2` from
  NIMA TR 8350.2 (the citation the prior author claims). Confirm the
  constants in `physics/frames.rs` match bit-for-bit.

- **First-eccentricity squared**: `physics/frames.rs` defines
  `WGS84_ECCENTRICITY_SQUARED = WGS84_FLATTENING * (2.0 -
  WGS84_FLATTENING)`. Verify this is the canonical form (some
  textbooks write `f * (2 - f)` directly; others factor as
  `2*f - f*f` which differs in operand order and could affect
  bit-stability if anything depends on this constant at hot-path
  scale). Compare to the pre-commit form.

- **`eci_to_ecef_velocity` transport-rate cross product**: derive
  `v_ecef = R_z(-θ) · (v_eci - ω × r_eci)` for `ω = (0, 0, ω_e)`
  and `r_eci = (x, y, z)`. The expected cross product is
  `ω × r = (-ω_e · y, ω_e · x, 0)`. Confirm the implementation
  computes `Vector3::new(-omega * r.y, omega * r.x, 0.0)` and not
  the negation. Sign error here causes silent transport-rate
  inversion.

- **ECEF → NED rotation matrix**: confirm the pre-commit and
  post-commit rotation matrices for `ecef_to_ned_position` are
  identical and match the textbook form
  `R_ecef_to_ned = [[-sφ·cλ, -sφ·sλ, +cφ], [-sλ, +cλ, 0],
  [-cφ·cλ, -cφ·sλ, -sφ]]` where φ = lat, λ = lon.

### C. Architecture — does the consolidation hold?

- Does `openbmp-core` now have **zero** physics constants? Search
  `crates/openbmp-core/src/` for `WGS84_`, `µ`, `mu`, `omega`,
  `g`, `J2`, etc. The remnant `MEAN_RADIUS_M` should not be in
  core (it's in `openbmp-physics::earth`). Other constants in core
  like `std::f64::consts::PI` are fine.

- Does `openbmp-fc` still build with `--no-default-features` after
  the move? The FC doesn't import `FrameContext` directly (per the
  prior physics-followup audit findings), so it should be
  unaffected. Verify:
  ```bash
  cargo build -p openbmp-fc --no-default-features --locked
  cargo test  -p openbmp-fc --no-default-features --locked
  ```

- Does `openbmp-physics::frames` correctly avoid leaking
  `FrameError`'s LocalOriginRequired path through a non-physics
  consumer? `LocalOriginRequired { profile: &'static str }` is the
  variant whose only caller is `FrameContext::require_local_origin`
  in `physics/frames.rs`. If `FrameError` lives in `openbmp-core`,
  the variant is technically declared in core but only ever
  constructed in physics. That's fine architecturally as long as
  the variant's documentation accurately reflects where it's
  raised. Read `crates/openbmp-core/src/error.rs` lines around
  `LocalOriginRequired` and verify the doc still references
  `FrameProfile` and `FrameContext` correctly given they now live
  outside core. If the doc points at `crate::frames::FrameContext`
  and that path no longer exists, it's a stale doc finding.

- Re-export hygiene: `openbmp-physics::lib.rs` should re-export the
  new public surface. Verify each of the moved items is named in
  exactly one `pub use frames::{ ... }` block, with no
  duplicates (e.g. `pub use frames::FrameContext;` listed twice).

### D. Determinism — did byte-stable scenarios survive the move?

The consolidation changed nothing about operand order; that's the
prior author's claim. Test it:
- Run the byte-stable scenarios twice with the new binary,
  hash-compare. Then check out the prior commit (`git checkout
  71edfdf~1`), run the same scenarios, hash-compare against the
  new tree. **Any non-zero diff is a high-severity finding** —
  the consolidation was supposed to be a pure relocation.

```bash
mkdir -p /tmp/audit-frames-{old,new}
git checkout 71edfdf
cargo run --bin openbmp --release -- run \
  scenarios/sounding-rocket/calisto/rocketpy-calisto.toml \
  --output-parquet /tmp/audit-frames-new/rocketpy-calisto.parquet
cargo run --bin openbmp --release -- run \
  scenarios/closed-loop-attitude-hold/scenario.toml \
  --output-parquet /tmp/audit-frames-new/closed-loop-attitude-hold.parquet
cargo run --bin openbmp --release -- run \
  scenarios/analytic-toy/constant-acceleration-drop.toml \
  --output-csv /tmp/audit-frames-new/constant-acceleration-drop.csv \
  --output-parquet /tmp/audit-frames-new/constant-acceleration-drop.parquet

git checkout 71edfdf~1
cargo run --bin openbmp --release -- run \
  scenarios/sounding-rocket/calisto/rocketpy-calisto.toml \
  --output-parquet /tmp/audit-frames-old/rocketpy-calisto.parquet
cargo run --bin openbmp --release -- run \
  scenarios/closed-loop-attitude-hold/scenario.toml \
  --output-parquet /tmp/audit-frames-old/closed-loop-attitude-hold.parquet
cargo run --bin openbmp --release -- run \
  scenarios/analytic-toy/constant-acceleration-drop.toml \
  --output-csv /tmp/audit-frames-old/constant-acceleration-drop.csv \
  --output-parquet /tmp/audit-frames-old/constant-acceleration-drop.parquet

git checkout 71edfdf  # leave tree at audited commit
diff -r /tmp/audit-frames-old /tmp/audit-frames-new
```

Any byte difference points at the diff. Don't accept "it's just
metadata, ignore it" — verify every difference and rule out the
moved code as a cause.

### E. Test truthfulness

Read the moved test bodies in `physics/frames.rs`. Each test
should have at least one assertion that would fail if the
implementation were wrong:

- `frame_context_eci_to_ecef_identity_in_toy` — asserts (7000,
  0, 0) → (7000, 0, 0). A pure-identity test: would catch a
  swapped `x` / `y` bug; would NOT catch the WGS84 case.
- `eci_ecef_position_round_trip_at_arbitrary_time` — round-trip
  to within 1.0e-6. Tight enough to catch real precision
  drift.
- `frame_transform_trait_is_time_aware_for_wgs84_eci_ecef` — has
  the load-bearing assertion `(via_trait.vector.y -
  p_eci.vector.y).abs() > 1.0e-6` which guards against a silent
  identity regression. Verify the assertion still has a non-zero
  threshold (a `> 0.0` would have been a tighter sentinel; `>
  1.0e-6` is appropriate given Earth-rotation rate at t=1234.5s).
- `down_vector_at_local_origin_is_minus_geodetic_normal` — locks
  in the geodetic-vs-geocentric convention. This test was the
  whole reason the WGS84 work didn't silently regress. Confirm
  `epsilon = 1.0e-12` is preserved.
- `local_geodetic_origin_rejects_out_of_envelope` — input
  rejection. Confirm all three assert variants still match
  `Err(FrameError::InvalidGeodeticCoordinate { .. })` not just
  `is_err()`. (`is_err()` is the rubber-stamp anti-pattern; the
  prior author's pattern is to match the specific variant.)

### F. Risk surface

- **Re-export drift.** `openbmp-physics::lib.rs` re-exports the
  moved items. If a consumer used to import `FrameContext` from
  `openbmp_core::FrameContext`, it now imports from
  `openbmp_physics::FrameContext` (or `openbmp_physics::frames::FrameContext`).
  If any consumer was missed, it would fail to compile. The full
  workspace build passing is necessary but not sufficient — verify
  the build was actually run (`cargo build --workspace --all-targets
  --all-features`) and not just `cargo build -p openbmp-physics`.

- **Doc-only references.** `crates/openbmp-vehicle/src/adapters.rs`
  has a `LocalGeodeticOrigin` mention in a doc comment.
  Confirm it's a doc reference, not an actual import (which would
  mean a missed consumer). Verify the doc reference still resolves
  if intra-doc-links is run.

- **`use openbmp_core::FrameContext` lurkers.** Run:
  ```bash
  grep -rn "openbmp_core::FrameContext\|openbmp_core::WGS84_\|openbmp_core::LocalGeodeticOrigin\|openbmp_core::FrameProfile\|openbmp_core::FrameTransform" crates/
  ```
  Expected: zero matches. Anything is a missed consumer that's
  somehow compiling because of unrelated re-exports — chase it to
  ground.

## Specific code locations

These are the high-yield places to investigate:

- `crates/openbmp-physics/src/frames.rs` (NEW, 580 lines) — the
  whole new module, including the moved tests.
- `crates/openbmp-physics/src/gravity.rs` lines 24–34 — imports
  from `crate::frames`; verify the `static_assertions::const_assert!`
  pair is **gone**, not commented out.
- `crates/openbmp-physics/src/lib.rs` — public surface (the new
  `pub mod frames;` plus the `pub use frames::{ ... }` re-export).
- `crates/openbmp-core/src/frames.rs` — slimmed (1869 → 1181
  lines). Verify the removed sections match what the commit
  claims.
- `crates/openbmp-core/src/lib.rs` — re-exports trimmed.
- `crates/openbmp-physics/src/magnetic/wmm2025.rs` line ~50–53 —
  imports refactored.
- `crates/openbmp-physics/src/wind/{constant,gust,layered,mod}.rs`
  — imports refactored.
- `crates/openbmp-cli/src/runner/{wind,phase2_point_mass,phase2_rigid_body}.rs`
  — type-prefix refactored.
- `crates/openbmp-physics/tests/regression.rs` — WGS84 constant
  imports moved from `openbmp_core` to `openbmp_physics`.
- `crates/openbmp-testkit/tests/inline_data_tripwire.rs` lines
  105–147 — allow-list tightened.
- `crates/openbmp-physics/README.md` § Module map and § Layering
  with `openbmp-core` — narrative updated.
- `docs/physics-consolidation-plan.md` § Frames + WGS84 constants
  consolidation (completed 2026-04-30) — narrative updated.

## Reproduction commands

```bash
# Mechanical gates (run all of them)
cargo +1.95 fmt --all -- --check
cargo +1.95 clippy --workspace --all-targets --all-features --locked -- -D warnings
cargo +1.95 build --workspace --all-targets --all-features --locked
cargo +1.95 test --workspace --all-features --locked
cargo +1.95 build -p openbmp-fc       --no-default-features --locked
cargo +1.95 test  -p openbmp-fc       --no-default-features --locked
cargo +1.95 build -p openbmp-physics  --no-default-features --locked
cargo +1.95 test  -p openbmp-physics  --no-default-features --locked
cargo deny check
cargo machete

# Test-count audit (expect 1079; the prior physics-followup audit
# was on 1083; FrameContext + WGS84 tests moved across crate
# boundaries but a small number may have been deduplicated. Any
# loss > 4 deserves a written justification.)
cargo test --workspace --all-features 2>&1 | grep "test result:" | \
  awk '{ p+=$4; f+=$6 } END { print "passed:", p, "failed:", f }'

# Single-source-of-truth verification
for sym in WGS84_A_M WGS84_INV_FLATTENING WGS84_FLATTENING \
           WGS84_ECCENTRICITY_SQUARED WGS84_MU_M3_S2 WGS84_OMEGA_RAD_S \
           LocalGeodeticOrigin FrameProfile FrameContext FrameTransform; do
  echo "=== $sym ==="
  grep -rn "pub const $sym\|pub struct $sym\|pub enum $sym\|pub trait $sym\b\|pub fn $sym" \
    crates/ --include='*.rs'
done
# Each item must show exactly one defining occurrence.

# Consumer-surface enumeration
grep -rln "FrameContext\|LocalGeodeticOrigin\|FrameProfile\|FrameTransform\|WGS84_A_M\|WGS84_INV_FLATTENING\|WGS84_FLATTENING\|WGS84_ECCENTRICITY_SQUARED\|WGS84_MU_M3_S2\|WGS84_OMEGA_RAD_S" \
  crates/ --include='*.rs' | sort

# Stale openbmp_core paths
grep -rn "openbmp_core::FrameContext\|openbmp_core::LocalGeodeticOrigin\|openbmp_core::FrameProfile\|openbmp_core::FrameTransform\|openbmp_core::WGS84_" \
  crates/ --include='*.rs'
# Expected: zero matches.

# Tripwire enforcement
cargo test -p openbmp-testkit --test inline_data_tripwire 2>&1 | tail -5

# Inline-needle enumeration
grep -rn "3\.986004418\|6378137\.0\|1\.082626683" \
  crates/ docs/ data/ scenarios/ 2>/dev/null
# Cross-check every result against the allow-list in
# crates/openbmp-testkit/tests/inline_data_tripwire.rs.

# Byte-stable scenario regression (see § D)
mkdir -p /tmp/audit-frames-{old,new}
git checkout 71edfdf
cargo run --bin openbmp --release -- run \
  scenarios/sounding-rocket/calisto/rocketpy-calisto.toml \
  --output-parquet /tmp/audit-frames-new/rocketpy-calisto.parquet
cargo run --bin openbmp --release -- run \
  scenarios/closed-loop-attitude-hold/scenario.toml \
  --output-parquet /tmp/audit-frames-new/closed-loop-attitude-hold.parquet
cargo run --bin openbmp --release -- run \
  scenarios/analytic-toy/constant-acceleration-drop.toml \
  --output-csv /tmp/audit-frames-new/constant-acceleration-drop.csv \
  --output-parquet /tmp/audit-frames-new/constant-acceleration-drop.parquet
git checkout 71edfdf~1
cargo run --bin openbmp --release -- run \
  scenarios/sounding-rocket/calisto/rocketpy-calisto.toml \
  --output-parquet /tmp/audit-frames-old/rocketpy-calisto.parquet
cargo run --bin openbmp --release -- run \
  scenarios/closed-loop-attitude-hold/scenario.toml \
  --output-parquet /tmp/audit-frames-old/closed-loop-attitude-hold.parquet
cargo run --bin openbmp --release -- run \
  scenarios/analytic-toy/constant-acceleration-drop.toml \
  --output-csv /tmp/audit-frames-old/constant-acceleration-drop.csv \
  --output-parquet /tmp/audit-frames-old/constant-acceleration-drop.parquet
git checkout 71edfdf
diff -r /tmp/audit-frames-old /tmp/audit-frames-new
# Expected: no diff. Anything is a finding.
```

## Deliverables

`docs/phase-4c-frames-consolidation-external-audit-findings.md`
(you write it). Sections:

```markdown
# Phase 4.C frames + WGS84 consolidation external audit findings

Auditor: <your name>
Date: <ISO 8601>
Working tree at: 71edfdf
Toolchain: <rustc --version>

## Executive summary

3-5 sentences. Did the consolidation land cleanly? Are the
prior author's claims load-bearing-true? Any
high-severity findings?

## Verified claims

| Claim | Status | Evidence |
|---|---|---|
| Six WGS84 constants live in exactly one place | ... | grep output |
| WGS84 numeric values bit-identical to prior commit | ... | git diff output |
| FrameContext method bodies unchanged | ... | git show 71edfdf -- crates/openbmp-physics/src/frames.rs |
| All moved tests preserved | ... | test count + per-name verification |
| No consumer outside the listed set touches the moved items | ... | grep output |
| Byte-stable scenarios identical pre/post | ... | diff output |
| HAL portability preserved (FC --no-default-features) | ... | build log |
| ... | ... | ... |

## Unverified or false claims

| Claim | Severity | What I found | Evidence |
|---|---|---|---|

## Architectural observations

The dual-residence-with-tripwire is gone. Did the consolidation
actually leave the workspace in the principled state the prior
audits demanded, or did it just relocate the debt? Look for:
- Are there *other* physics primitives still in openbmp-core that
  follow the same pattern (existed because of historical reasons,
  should now live in physics)?
- Is the new `openbmp-physics::frames` module well-shaped, or is
  it a flat dump of everything that left core?

## Risk findings

For each risk: severity, description, reproduction.

## Tests of suspect rigor

For each rubber-stamp test found in the moved suite: file:line,
what it claims, what it verifies.

## Sign-off

Whether the working tree at 71edfdf is fit to keep, or whether
findings need addressing.
```

## Authorization scope

You may:
- Read every file in the repo.
- Run any `cargo` command (build, test, clippy, fmt, deny,
  machete, run, doc, tree).
- Run any deterministic shell command for inspection (grep, find,
  diff, sha256sum, cmp, git log / show / diff / checkout).
- Check out historical commits (`git checkout 71edfdf~1` ↔ `git
  checkout 71edfdf`) for byte-stable comparisons. Restore the
  audited HEAD when done.
- Write your findings to
  `docs/phase-4c-frames-consolidation-external-audit-findings.md`.
- Write throwaway files to `target/audit-scratch/` to verify
  algorithm claims.

You may NOT:
- Modify production code, scenarios, data files, or the prior
  author's documentation (other than the findings doc).
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
- Re-architecting `openbmp-physics::frames` even further (e.g.
  splitting into sub-modules). That's a separate streamline review
  scope.
- Re-running the prior physics-followup audit
  (`docs/phase-4c-physics-followup-external-audit.md`). That work
  is done and its findings were addressed in commit `bb9da62`.
- Litigating the prior author's previous "needs user authorisation"
  framing. The user authorised; the work landed. Your job is to
  decide whether what landed is correct.

## Tone expectations

- **Don't soften findings.** "Byte-stable scenario X drifted by 4
  bytes at row 12,341" is a finding. "Tests pass and that's
  reassuring" is not.
- **Don't accept "but it compiles" as defense.** A workspace can
  compile with stale doc references, with re-export shadowing, or
  with missed consumers that happened to be downstream of an
  unrelated re-export.
- **Cite references for math claims.** WGS84 values trace to NIMA
  TR 8350.2; cite section + table for any numeric verification.
- **Be specific about severity.**
  - `high`: a load-bearing claim is false (e.g. byte-stability
    broke; a constant changed value; a method body changed).
  - `medium`: a claim is overstated or misleading (e.g. doc still
    refers to the old location; tripwire allow-list missed an
    occurrence; a stale `use` lurks behind an unrelated re-export).
  - `low`: cosmetic or aesthetic (e.g. README phrasing is awkward;
    a comment is stale but harmless).

## Sign-off

The audit is complete when **all** of:

1. Every claim in "Key claims to verify" has a verified-or-refuted
   entry in the findings doc.
2. The byte-stable scenarios have been re-run pre/post and any
   diff has been reported with file:row:column precision.
3. The single-source-of-truth invariants (one definition per
   constant / type) have been verified by exhaustive grep, not
   just spot-check.
4. The doc updates (README + plan) have been read end-to-end and
   any stale framing flagged.
5. The HAL-portability claim has been verified by running
   `cargo build -p openbmp-fc --no-default-features --locked`.
6. The findings doc is self-contained and reproduces every
   verification command.

Project owner reads the findings and decides what to do (commit
the findings doc as-is, fix findings first, or revert the
consolidation if a high-severity finding lands).
