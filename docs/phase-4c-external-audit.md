# Phase 4.C external audit handover

> **Audience.** An external auditor. Independent of the prior author.
> No prior context with this codebase. Mandated to **verify**, not
> rubber-stamp.
>
> **Stance.** Skeptical. The prior author wrote both the
> implementation and the prior internal audit, then closed their own
> findings. The external audit's job is to find what *that* loop
> missed — claims overstated, tests that look thorough but rubber-
> stamp, math that compiles but doesn't match the cited references.
>
> **Read-only by default.** Authorized to read every file, run any
> deterministic command (cargo build / test / clippy / fmt / deny /
> machete, scenario reruns, byte-stable Parquet diffs), and write a
> findings report. **Not** authorized to modify code, push commits,
> or rerun the prior audit's loop. If a finding requires code
> changes, it goes in the report; the project owner decides what to
> do.

## What you're auditing

Three Phase-4 commits on `main`:

```
cd47cf2  Phase 4.C: audit fixes + physics consolidation, retire openbmp-env
3474b2f  Phase 4.A–B: openbmp-fc autopilot framework + close-out hardening
```

Plus the planning chain that produced them:

- `docs/phase-4-plan.md` — original plan + landed sections
- `docs/phase-4b-plan.md` — close-out punchlist
- `docs/phase-4-audit.md` — internal audit prompt (the prior author wrote it)
- `docs/phase-4-audit-findings.md` — internal audit findings (same author)
- `docs/phase-4c-plan.md` — Phase 4.C punchlist
- `docs/phase-4c-audit.md` — internal 4.C implementation prompt
- `docs/physics-consolidation-plan.md` — env → physics retirement plan
- `docs/clarabel-vetting.md` — solver vetting record
- `docs/phase-5-plan.md` — what was deferred forward

The audit author and the implementation author are the same person.
That's the conflict of interest you're checking.

## Project context (5-paragraph version)

OpenBMP is a Rust-first, simulation-only, academic research
platform for rocket-class flight simulation. It's deliberately
non-deployable: no real hardware integrations, no targeting, no
operational claims under any safety regime (IEC 61508, DO-178C,
ISO 26262). It's a Cargo workspace with ~17 crates, single binary
(`openbmp`), reproducible Parquet telemetry as the artifact.

Phase 4 promised an autopilot binary with deployable-grade
architecture: lockstep clock, internal pub/sub bus, cyclic
scheduler with budget enforcement, commander as the single
state-machine owner, parameter registry, voter, health/arming
gate, and phase-gated actuator authority — wired around academic
algorithms (EKF / MEKF / three-loop autopilot / mission FSM /
guidance / FDIR). The architectural contract: `openbmp-fc` must be
HAL-portable (depends only on hardware-portable crates, never on
the simulator).

Phase 4.A landed the architectural skeleton (commit `3474b2f`).
Phase 4.B closed gaps in 4.A: deleted misrepresenting stubs (an
LQR-shaped "MPC", PD-shaped "LCvxLD", mean-only "UKF"),
implemented missing math (real EKF baro/mag updates, gravity model
plumbed in, trajectory loop, sequence-based health staleness),
wired pre-existing types (gain schedule, phase authority, voter,
FDIR), added quality gates and property tests, integrated with the
scenario format. Phase 4.C (commit `cd47cf2`) executed the
internal audit's findings and consolidated all physics into a
single `openbmp-physics` crate, retiring the former `openbmp-env`.

The consolidation is the most architecturally invasive piece. It
moved ~3 500 lines (full WMM 2025, full 7-layer USSA76, J2 gravity,
wind family) from `openbmp-env` into `openbmp-physics`, deleted
the FC's parallel-physics surface, migrated every consumer
(`openbmp-cli`, `openbmp-vehicle`, `openbmp-aero`, `openbmp-propulsion`),
and removed the env crate from the workspace. `EnvError` was
renamed to `PhysicsError` workspace-wide.

The commit message claims: 1 063 tests passing, HAL portability
preserved (`cargo build -p openbmp-fc --no-default-features`
clean), `cargo tree -p openbmp-env` returns "package not found",
Phase-1/2/3 byte-stable scenarios still match, lockstep-clock
tripwire passes, all quality gates clean.

**These claims are what you're verifying.**

## Verification commands (run all of them, don't trust prior output)

Set MSRV first: `rustup install 1.93 1.95` if not present.

```bash
# Stable verification
cargo +1.95 fmt --all -- --check
cargo +1.95 clippy --workspace --all-targets --all-features --locked -- -D warnings
cargo +1.95 build --workspace --all-features --locked
cargo +1.95 test --workspace --all-features --locked

# HAL portability (the load-bearing architectural claim)
cargo +1.95 build -p openbmp-fc --no-default-features --locked
cargo +1.95 test -p openbmp-fc --no-default-features --locked

# Feature combinations the FC actually exposes
cargo +1.95 build -p openbmp-fc --features mpc --no-default-features --locked
cargo +1.95 build -p openbmp-fc --features square-root-ekf --no-default-features --locked
cargo +1.95 build -p openbmp-fc --features l1-adaptive --no-default-features --locked

# Supply chain
cargo deny check
cargo machete

# MSRV gate
cargo +1.93 build --workspace --all-features --locked

# Byte-stable scenario regression — the load-bearing determinism claim.
# Run twice; diff Parquet by row.
mkdir -p /tmp/audit-run-{1,2}
cargo run --bin openbmp --release -- run scenarios/sounding-rocket/calisto/rocketpy-calisto.toml \
  --output /tmp/audit-run-1
cargo run --bin openbmp --release -- run scenarios/sounding-rocket/calisto/rocketpy-calisto.toml \
  --output /tmp/audit-run-2
cargo run --bin openbmp --release -- diff /tmp/audit-run-1/*.parquet /tmp/audit-run-2/*.parquet

# Repeat for every scenario under scenarios/. Especially the closed-loop one.

# The "openbmp-env is gone" claim
cargo tree -p openbmp-env  # MUST fail with "package not found"
grep -rn "openbmp_env::" --include='*.rs' crates/ | grep -v ":#"  # MUST be empty
grep -rn "openbmp-env" --include='*.toml' . | grep -v "/target/"  # MUST be empty (besides historical docs)

# Lockstep-clock tripwire
cargo +1.95 test -p openbmp-testkit --lib --locked fc_lints

# Test count audit
cargo test --workspace --all-features 2>&1 | grep "test result:" | \
  awk '{ p+=$4; f+=$6 } END { print "passed:", p, "failed:", f }'
# Verify the prior author's claim of 1 063.
```

If any of these fails, the corresponding claim in commit `cd47cf2`'s
message is wrong. File a high-severity finding.

## Audit dimensions

### A. Honesty — does the prior author's commit message match the code?

The commit message for `cd47cf2` makes specific claims. Each is a
hypothesis to falsify:

1. "1 063 tests passing." — count them yourself. If they're 1 062 or
   1 064, file the discrepancy. Look for `#[ignore]` annotations,
   `feature = "..."` gates that hide tests, or empty test bodies that
   pass trivially.
2. "HAL portability preserved." — run the no-default-features build.
   If it fails, the FC is no longer HAL-portable.
3. "WMM 2025 is in-tree, FC dipole placeholder superseded for
   scenarios that opt in." — find the test that proves the FC actually
   uses `Wmm2025` rather than `EarthDipoleField`. The prior audit's
   internal test surface is at
   `crates/openbmp-fc/tests/`. Read every file and confirm at least
   one test exercises `Wmm2025` against a NOAA reference vector
   through the FC's bus. If not, the claim is overstated.
4. "Byte-stable Phase-1/2/3 scenarios still match." — actually re-run
   the regression scenarios twice and `diff` Parquet. If they don't
   match byte-for-byte, the trait migration broke determinism. The
   prior author admitted this risk in
   `docs/physics-consolidation-plan.md` Risk #1; the regression gate
   is the only proof.
5. "Joseph form on the EKF / MEKF covariance update." — open
   `crates/openbmp-fc/src/estimator.rs`, find the covariance-update
   sites. Verify the form is `(I − K·H) · P · (I − K·H)ᵀ + K · R · Kᵀ`
   (Joseph), not `(I − K·H) · P` (the inferior symmetric form). If
   the variable name says "joseph" but the math is the inferior
   form, file a high-severity finding.
6. "Markley-form MEKF reset preserves second-order accuracy." — find
   the reset code. Confirm it includes the Markley 2003 §III.B
   eq (33) covariance correction, not just the first-order
   `q_nominal ⊗ exp(δθ/2)` reset. The prior internal audit prompt
   explicitly called out this risk; verify it was actually closed.
7. "Gauss-Markov bias dynamics with steady-state variance
   `σ² = q · τ / 2`." — pick the time constant from a test, run the
   filter for 10× the time constant, measure the bias variance.
   Confirm the closed-form steady-state matches.
8. "Iterated EKF mag update converges within `max_iterations`." —
   read the code. Confirm the iteration count + convergence
   criterion are real, not just a `for _ in 0..3 { ... }` that
   runs the body 3 times regardless.
9. "Real 6-state sigma-point UKF, not the deleted scaffold." —
   confirm the sigma points are generated by the Wan & van der
   Merwe formulation (alpha, beta, kappa parameters) and that
   every sigma is propagated, not just the mean. If the UKF
   propagates the mean and inflates the diagonal, it's the
   deleted scaffold under a new name.
10. "Real biquad notch filter for the rate loop, off by default."
    — open `crates/openbmp-fc/src/filters.rs`. Verify the biquad
    is the standard Direct-Form-II Transposed form. Run a chirp
    test (you'll need to write one, or find one) and confirm
    magnitude attenuation at the notch frequency matches the
    declared depth_db within rounding.
11. "L1 adaptive augmentation behind a feature flag." — open
    `crates/openbmp-fc/src/l1_adaptive.rs`. Confirm the
    formulation matches Cao & Hovakimyan 2010, not a hand-wave
    that calls itself L1.
12. "Differential-flatness trajectory tracking, alternative to
    PID, selectable via scenario." — confirm the analytic
    attitude reference is computed from position derivatives
    (Mellinger & Kumar 2011), not just a per-axis PID with a
    different name.
13. "GLRT and CUSUM detectors with explicit fault bits." — verify
    the detectors actually compute the GLR statistic and CUSUM
    score, not just a rebadged burst counter.
14. "`cargo tree -p openbmp-env` returns 'package not found'." —
    verify. If the crate is still in the tree, the consolidation
    is incomplete.

For each claim, the finding format is:
- The exact quoted phrase from the commit message.
- The file:line where you tried to verify.
- What you found.
- Severity (`high` for false claims, `medium` for overstated,
  `low` for technically-correct-but-misleading).

### B. Soundness — does the math match the cited references?

Pick three to five algorithms and audit deeply. Suggested high-yield
targets:

1. **Joseph form covariance update.** Bierman 1977. Verify
   numerical stability: P stays positive-definite under random
   measurement noise. Write a 1 000-iteration property test that
   asserts the smallest eigenvalue of P stays positive. (You're
   authorized to write tests as part of the audit; they don't ship.)
2. **WMM 2025 spherical-harmonic evaluator.** Compare against the
   100-row NOAA reference test set. The prior author claims this is
   tested at `crates/openbmp-physics/tests/wmm_data_pin.rs`. Confirm
   the test actually compares against the published values, not
   against itself.
3. **J2 gravity at the equator and pole.** Vallado §8.6. The closed-
   form ratio `|g_J2| / |g_central| = 1.5 · J2 · (R_e / r)²` at the
   equator is independent. Verify the FC and the kernel both
   reproduce it to 1e-15 (not 1e-3).
4. **USSA76 7-layer model.** Compare per-kilometre against the
   1976 NOAA-S/T table. The prior author claims this is in
   `crates/openbmp-physics/tests/regression.rs`. Confirm.
5. **Markley reset second-order accuracy.** Markley 2003 §III.B
   eq (33). Derive the expected attitude-error growth rate and
   confirm the implementation matches.
6. **Innovation chi-square gate derivation from FAR.** The prior
   author claims gates are derived from a stated false-alarm rate
   per measurement DOF. Verify: for `false_alarm_rate = 0.01` and
   3 DOF (mag), the gate should be `chi2_inv(0.99, 3) ≈ 11.345`.
   Find the gate computation; confirm it's not just `9.0` again.

For each algorithm, write the math derivation in your findings
report (one paragraph), state the implementation file:line, and
state whether they match.

### C. Architecture — does the structure work as advertised?

1. **HAL portability.** `openbmp-fc/Cargo.toml` should not directly
   or transitively depend on `openbmp-sim`, `openbmp-cli`,
   `openbmp-scenario`, `openbmp-telemetry`, `openbmp-bridge`, or
   `openbmp-aerothermal`. Run `cargo tree -p openbmp-fc --all-features`
   and check.
2. **Lockstep clock contract.** No `std::time::Instant::now()`,
   `std::time::SystemTime::now()`, `std::time::Instant`, or
   `std::time::SystemTime` references inside `crates/openbmp-fc/src/`,
   `crates/openbmp-fc/tests/`, `crates/openbmp-physics/src/`, or
   `crates/openbmp-cli/src/runner/fc_bridge.rs`. The prior author
   claims a tripwire enforces this; verify the tripwire's regex
   actually catches all forms (e.g. fully-qualified paths, type
   aliases that shadow the names).
3. **Kernel↔FC bridge integration.** `[fc]` scenarios should drive
   `phase2_*.rs` runners. Find the closed-loop scenario at
   `scenarios/closed-loop-attitude-hold/`. Run it and confirm a
   Parquet output is produced. Confirm two reruns produce
   byte-identical output. The internal audit's claim is that this
   path actually works; verify by running.
4. **Trait surface duplication.** Search for parallel definitions:
   `grep -rn "pub trait GravityModel\|pub trait MagneticFieldModel\|pub trait AtmosphereModel" crates/ --include='*.rs'`.
   The expected result is exactly one definition per trait, all in
   `openbmp-physics`. If `openbmp-fc` still has its own traits, the
   consolidation is incomplete (the commit message says they were
   deleted; verify).
5. **`GravityAdapter` clamp behavior.** The prior author defended
   the deviation from D-PC-1 by adding a `GravityAdapter` that
   silently clamps `Err(_)` to `Vector3::zeros()`. This is a silent
   failure mode. Audit:
   - Under what input conditions does the underlying `GravityModel`
     return `Err`? (Read the env-side code paths now in physics.)
   - For each, would silent zero-clamp produce a wrong-but-finite
     trajectory the FC would propagate as if normal?
   - Is there a topic / log channel that records the silent
     clamping? (The prior author claimed the FC's hot path stays
     total; that doesn't mean failures are observable.)
6. **Sensor truth → measurement → bus → estimator chain.** The
   bridge owns sensor adapters. Read `fc_bridge.rs`. Confirm:
   - The kernel's specific force is gravity-subtracted before
     being passed to the IMU sensor. (The prior internal audit
     called this out; verify it was implemented correctly.)
   - The mag sample is rotated from ECI to body via the kernel's
     attitude before being published.
   - The barometer altitude uses the kernel's geometric altitude,
     not the FC's estimated altitude.
   These are the bridge's three load-bearing translations. If any
   of them is wrong, the FC sees a corrupt measurement stream.
7. **Determinism of the new physics surface.** The trait migration
   added `Position3<Eci>` construction (`Position3::new(x, y, z)`
   from the FC's `Vector3<f64>`) at every gravity call site.
   Verify: does this introduce any `f64` operand-order change vs
   the original env path? If yes, the byte-stable Phase-1/2/3
   scenarios will diff. The regression gate (verification command
   above) is the proof.

### D. Test truthfulness — are the tests actually testing what they claim?

The prior author shipped many new tests. For each, ask:

1. Does the test name match what it asserts?
2. Does the test exercise the production code path, or does it
   construct a fixture inline that bypasses the path under test?
3. Are the assertions tight (e.g. `assert!((x - 9.81).abs() < 1e-3`)
   or loose (e.g. `assert!(x.is_finite())`)?
4. Does the test exercise a single thing or a workflow? Workflow
   tests can pass even when the thing under test is broken, if
   the rest of the workflow happens to compensate.

Specific tests to read with skepticism:

- `crates/openbmp-fc/tests/textbook_examples.rs` — claims to
  reproduce Bar-Shalom-Li-Kirubarajan textbook examples. Open the
  textbook (§5.4 / §5.5), pick one example, and confirm the test
  inputs and outputs match the textbook to the published precision.
- `crates/openbmp-fc/tests/long_duration_stability.rs` — claims
  60-second stability. Confirm the test runs the *full* pipeline
  (sensor ingest → estimator → autopilot → mixer → bus
  republish), not just the integrator.
- `crates/openbmp-fc/tests/sensor_ingest.rs` — claims
  SyntheticSensorAdapter → VotedImuIngest → bus path. Confirm
  the test actually constructs a SyntheticSensorAdapter, not a
  bypass that publishes ImuSample directly.
- `crates/openbmp-physics/tests/wmm_data_pin.rs` — claims to
  validate against the NOAA reference values. Confirm the test
  loads the reference values (not hand-derived ones).
- `crates/openbmp-physics/tests/regression.rs` — claims
  per-kilometre regression. Confirm the reference values are
  pinned to a citation, not "whatever the model produced last
  time".

If a test rubber-stamps, file it as a finding. Severity depends on
what the test is supposed to be load-bearing for.

### E. Risk surface — what's the consolidation's failure-mode spectrum?

1. **Silent clamp on gravity Err.** Already discussed. What's the
   blast radius?
2. **Geodetic conversion now lives inside physics models.** Did
   the migration preserve the operand order from the env-side
   code? If a model's internal ECI→geodetic conversion uses
   different floating-point ordering than the prior env-side
   conversion, byte-stable scenarios diff.
3. **PhysicsError variants vs. EnvError variants.** Did the
   rename also change semantics? E.g. does `PhysicsError::OutOfEnvelope`
   surface in places where `EnvError::OutOfEnvelope` previously
   wouldn't, or vice versa?
4. **Feature interaction.** The FC has features `mpc`,
   `square-root-ekf`, `l1-adaptive`, `synthetic`, plus
   `default = ["sim-default"]`. Try every reasonable combination.
   Each must build green. The prior author tested some; you test
   the rest.
5. **`openbmp-physics` as a workspace centerpiece.** The crate is
   now ~3 500 lines with `synthetic` default-on. Hardware adopters
   on the FC side disable `synthetic`; verify
   `cargo build -p openbmp-physics --no-default-features --locked`
   is clean and excludes `GustWind`.
6. **Stale documentation.** The `physics-consolidation-plan.md`
   has a 21-item Definition of Done. Cross-check every item
   against the working tree. Any item marked "done in plan" but
   not actually delivered is a finding.

### F. Completeness — what's labeled "Phase 5 deferral" that should have been done?

The prior author shipped a long list under Phase 5 deferrals. Some
are truly out-of-scope (NRLMSISE-00 — fine). Others might be
self-justifying — labeled "deferred" because the prior author
didn't get to them, not because they're genuinely separate scope.

Audit `docs/phase-5-plan.md` and `docs/phase-4-plan.md`'s deferral
list. For each item, assess: is this a Phase 5 thing in spirit, or
a "we ran out of time" thing dressed as architecture?

Specific items to scrutinize:

- **Multi-instance estimator.** The prior author defends keeping
  this deferred because "the trait surface is shipped." But the
  trait surface alone doesn't help downstream HAL adopters who
  need to *select* between competing lanes. Is the deferral
  defensible, or is it a punt?
- **Full 15-state UKF.** The prior author shipped the 6-state
  attitude UKF and deferred 15-state. Reasonable on size grounds,
  but verify the 6-state UKF is actually useful for the FC's
  scenarios (not just an unused option).
- **Square-root UKF.** Deferred. The prior author shipped UD
  factorization for the EKF only. Is there a numerical-stability
  argument for keeping classical UKF, or is the square-root just
  not done?
- **Anti-windup observer form.** The prior author punted with
  "ships only if it wins on a representative scenario." Did anyone
  run that comparison? If not, the punt is unverified.
- **Cross-tool comparison against PX4 ekf2 / ArduPilot NavEKF3.**
  The prior author replaced this with Bar-Shalom textbook examples.
  Is the textbook a reasonable substitute, or did the prior author
  swap rigor for convenience?

## Specific code locations to investigate

These are the high-yield places to find issues:

- `crates/openbmp-fc/src/estimator.rs` ~50–100 — `GravityAdapter`,
  the silent-clamp wrapper. Audit blast radius.
- `crates/openbmp-fc/src/estimator.rs` (Joseph update sites) —
  verify it's actually Joseph form.
- `crates/openbmp-fc/src/estimator.rs` (Markley reset site) —
  verify second-order correction.
- `crates/openbmp-fc/src/estimator.rs` (UKF sigma generation) —
  verify Wan & van der Merwe formulation.
- `crates/openbmp-fc/src/ud.rs` — verify Bierman 1977 UD
  factorization.
- `crates/openbmp-fc/src/filters.rs` — verify biquad coefficients.
- `crates/openbmp-fc/src/l1_adaptive.rs` — verify Cao & Hovakimyan
  2010 formulation.
- `crates/openbmp-fc/src/mpc.rs`, `crates/openbmp-fc/src/landing.rs` —
  Clarabel-backed solvers. Audit the cost/constraint formulation
  against the cited references (Acikmese & Ploen 2007 for LCvxLD).
  Verify `cargo deny` actually accepts the Clarabel version
  pinned in `docs/clarabel-vetting.md`.
- `crates/openbmp-cli/src/runner/fc_bridge.rs` — sensor truth
  translation. Especially `point_mass_truth` and `rigid_body_truth`.
  Verify gravity-subtraction on specific force.
- `crates/openbmp-physics/src/magnetic/wmm2025.rs` ~588 — the WMM
  test references `WGS84_A_M`. Confirm the value is pinned to
  NIMA TR8350.2 and matches `data/magnetic/provenance.md`.
- `crates/openbmp-testkit/src/fc_lints.rs` — the lockstep tripwire.
  Verify the regex catches the patterns it claims.
- `crates/openbmp-testkit/tests/inline_data_tripwire.rs` — verify
  the allow-list points at the post-consolidation file paths.

## Reproduction notes

If you find behavior that diverges from the commit message, capture:

1. The exact `cargo` command you ran.
2. The exit code.
3. The relevant stdout / stderr (last 50 lines).
4. The `git rev-parse HEAD` at audit time (should be `cd47cf2`).
5. Toolchain (`rustc --version`).
6. Platform (`uname -a`).

A finding without reproduction is a recommendation; with
reproduction it's a regression.

## Deliverable format

`docs/phase-4c-external-audit-findings.md` (you write it). Sections:

```markdown
# Phase 4.C external audit findings

Auditor: <your name>
Date: <ISO 8601>
Commit: cd47cf2 (verify with git log)
Toolchain: <rustc --version>

## Executive summary

3-5 sentences. What's the overall assessment? Is the prior author's
"all gates clean" claim accurate? Where are the load-bearing
problems?

## Verified claims

| Claim | Status | Evidence |
|---|---|---|
| 1 063 tests passing | confirmed | output of `cargo test --workspace --all-features` |
| HAL portability | confirmed | `cargo build -p openbmp-fc --no-default-features` |
| ... | ... | ... |

## Unverified or false claims

| Claim | Severity | What I found | Evidence |
|---|---|---|---|
| ... | ... | ... | ... |

## Algorithm soundness

For each algorithm audited deeply:

### <algorithm name>

- Reference: <citation>
- Implementation: <file:line>
- Math derivation: <one paragraph>
- Verdict: matches / matches with caveats / doesn't match
- Evidence: <test command + output, or cited line numbers>

## Risk findings

For each risk identified:

- Severity (`high` / `medium` / `low`)
- Description
- Reproduction steps
- Recommendation

## Tests of suspect rigor

For each rubber-stamp test found:

- File:line
- What the test claims to verify
- What it actually verifies
- Recommendation

## Architectural observations

Any structural issues that aren't strictly bugs.

## Open questions for the project owner

Things that aren't audit findings but warrant a decision.
```

## Authorization scope

You may:
- Read every file in the repo.
- Run any `cargo` command (build, test, clippy, fmt, deny, machete,
  run, doc, tree).
- Run any deterministic shell command for inspection (grep, find,
  diff, sha256sum).
- Write your findings to `docs/phase-4c-external-audit-findings.md`.
- Write throwaway test files (call them
  `target/audit-scratch/*.rs`) to verify algorithm claims.

You may not:
- Modify production code, scenarios, data files, or the prior
  author's documentation.
- Stage or commit changes to the project (other than the findings
  doc).
- Push, force-push, branch-delete, or rebase.
- Run anything that requires network access (no `curl`,
  `cargo install`, `cargo update`, `git fetch`).
- Modify dependencies or Cargo.lock.
- Disable any test, lint, or feature gate.

If you reach a point where verification requires code modification,
file the finding and stop. Don't fix.

## Non-goals

- Style preferences. The prior author and you may disagree about
  naming conventions; that's not a finding.
- Speculative future-phase work. Phase 5 is its own scope.
- Alternative architectural choices. If the prior author picked X
  over Y for documented reasons, "Y would have been better" is not
  a finding unless X is broken.
- Re-running the prior author's internal audit. That work is done.
  You're auditing it, not redoing it.
- Code style / clippy pedantic. Workspace lints already enforce
  this.

## Tone expectations

This audit is adversarial by design, not by attitude. The prior
author is a colleague, not an adversary. But the codebase claims
SOTA quality, and you are paid to find where it isn't. Specifics:

- **Don't soften findings.** "The Joseph form is actually the
  symmetric form" is a finding. "Maybe consider revisiting the
  covariance update" is not.
- **Don't speculate without evidence.** "The UKF might not be
  correct" is not a finding. "The UKF generates 6 sigma points
  but Wan-van der Merwe specifies 2n+1=13 for n=6, see line 412"
  is.
- **Don't accept "but it works" as defense.** Working scenarios
  are necessary, not sufficient. The prior author's tests pass;
  you're auditing whether they pass for the right reason.
- **Cite references.** Every algorithmic claim against the prior
  author's implementation should cite a paper, textbook, or
  primary source.
- **Be specific about severity.** `high`: a load-bearing claim is
  false (e.g. tests don't actually pass, HAL portability is
  broken). `medium`: a claim is overstated or misleading
  (e.g. tests pass but don't test what they claim).
  `low`: a claim is technically correct but inelegant.

## A specific challenge

The prior author claims the deviation from D-PC-1 (keeping the
rich `Position3<Eci> + SimTime + Result<_, PhysicsError>` trait
shape rather than the simpler `Vec3 + SimTime` shape) is
"functionally equivalent to D-PC-1's intent (FC consumes physics,
no parallel trait surface) with significantly less migration risk."

Test that claim adversarially:

1. The `GravityAdapter` introduces a silent error-clamp. Find every
   `Err(_)` path in the gravity model implementations. For each,
   describe what the FC would observe under the silent clamp.
2. The migration risk argument was about preserving byte-stable
   scenarios. Did the prior author actually run the byte-stable
   regression on the post-migration tree? Find the evidence (the
   commit message claims it; the audit needs proof).
3. Could a hypothetical D-PC-1 implementation have preserved
   byte-stability with a more disciplined migration? If yes, the
   "less migration risk" claim was a convenience, not a necessity.

This isn't a witch-hunt; it's a stress-test of the prior author's
reasoning. Sometimes the deviation was right. Sometimes it was a
shortcut. The audit's job is to tell the difference.

## Sign-off

The audit is complete when:

1. Every claim in commit `cd47cf2`'s message has a verified-or-
   refuted entry in the findings table.
2. At least three algorithms have been audited deeply against
   their cited references.
3. The byte-stable scenarios have actually been re-run and the
   diffs reported.
4. The architectural HAL-portability claim has been verified by
   running the no-default-features build.
5. The risk surface section enumerates at least the silent-clamp
   risk, the operand-order risk on the trait migration, and any
   others you find.
6. The findings document is self-contained and reproduces every
   verification command.

Project owner reads the findings and decides what to do. The
external audit's role ends with the report.
