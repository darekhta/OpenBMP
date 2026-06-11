# Parity Program — Agent Execution Playbook

**Status:** `experimental` (process doc; ships no code).
**Audience:** the LLM agent (or engineer) implementing the parity work packages.
**Prerequisite reading:** `00-overview.md` (parity definition, invariants, DAG).

This document is the operating manual. It defines how to turn a work package
(`WP-NN.t`) from a design doc into a merged, gate-green PR without weakening any
OpenBMP invariant. The per-dimension docs (`01`–`12`, `14`–`25`) say *what* to
build; this says *how to ship it*.

---

## 1. The prime directives

1. **One work package, one PR** (or a tight set), each independently shippable.
   Never a big-bang merge. The fidelity tiers exist so value lands incrementally.
2. **Green on the full gate set (§2) before merge.** No exceptions, no
   "fix it in the next PR" for an invariant.
3. **Byte-stable-by-default.** A new capability is off until a scenario opts in.
   If a golden archive changes and the PR did not intend a behavioral change,
   the PR is wrong — find the nondeterminism, do not re-bless the golden.
4. **Honesty over optimism.** State the validation label you actually earned and
   the parity ceiling you did not cross. Do not label a textbook model
   `research`; do not imply flight readiness; lead with blockers when blocked
   (`00` §1, `docs/safety-boundaries.md`). A partially-finished WP is reported as
   partial, never as done.
5. **Forward-only, locks tighten with capability** (`00` §6). Adding a guidance
   or boundary-condition surface means adding the tripwire/compile-fail test that
   proves it cannot express a ground-aimpoint solution.

---

## 2. The gate set (must pass before merge)

Run from the workspace root. These mirror `.github/workflows/ci.yml` and
`docs/verification.md`. The agent runs them locally and does not merge until all
are green.

| Gate | Command | Enforces |
|---|---|---|
| Build | `cargo build --workspace --all-targets` | compiles, no warnings-as-errors tripped |
| Format | `cargo fmt --all -- --check` | style |
| Lints | `cargo clippy --workspace --all-targets -- -D warnings` | workspace lints incl. `missing_docs`, `unsafe_code`, `float_cmp`, `unused_must_use` (deny); `unwrap/expect/panic` (warn outside tests) |
| Unit/property/doc | `cargo test --workspace` | behavior + invariants + the tripwire tests below |
| Determinism | run canonical scenarios twice, `openbmp diff` byte-identical (the CI determinism lane) | byte-stable output on the reference profile |
| Provenance | `openbmp check-provenance <touched scenarios/data>` | every `data/`+`scenarios/` TOML has a sibling `provenance.md` listing it + SHA pin |
| Supply chain | `cargo deny check` · `cargo audit` · `cargo machete` | dependency policy, advisories, unused deps |
| Traceability | `python3 scripts/check_requirements_traceability.py` | no orphan requirements / verification items / stale anchors |

**Tripwire tests that must stay green** (subset of `cargo test`, called out
because they are easy to trip and load-bearing):

- `crates/openbmp-testkit/tests/fc_dependency_tripwire.rs` — FC depends only on
  hardware-portable crates.
- `crates/openbmp-testkit/src/fc_lints.rs` (its `#[cfg(test)]` tests) —
  lockstep-clock + no-hot-path-allocation.
- `crates/openbmp-testkit/tests/inline_data_tripwire.rs` — four-pillar
  provenance + no inline data/benchmark constants.
- `crates/openbmp-testkit/tests/ballistic_state_compile_fail.rs` — forward-only
  footprint-seed provenance (compile-fail).

If a WP legitimately introduces a new source-of-truth file for a real constant,
the fix is to add that path to the tripwire allow-list **in the same PR** — a
reviewable act — not to delete the check.

---

## 3. Per-PR invariant checklist

Copy into the PR description and answer every item (this is the
`docs/verification.md` review checklist hardened into a gate):

- [ ] Runs without physical hardware; new hardware concerns stay behind
      `openbmp-hal`/`openbmp-bridge` abstractions or are marked downstream-owned.
- [ ] New data is synthetic / textbook / openly-published, with `provenance.md`,
      license status, and SHA-256 pin. No fielded-vehicle parameter sets.
- [ ] No literal WGS84 GM⊕ / J₂ / R⊕ numerals outside their allow-listed
      source-of-truth files (refer symbolically).
- [ ] New model advances state with locked operand order, no `mul_add`, no
      wall-clock, no system RNG (`DeterministicRng` only), no unordered iteration.
- [ ] New capability is config-gated and **off by default**; existing goldens
      byte-identical.
- [ ] Units, frames, validity envelope documented; validation label declared and
      justified; tolerance table added for any numeric claim.
- [ ] Failure modes fail-closed (`Result`, not `panic`, on scenario-reachable
      paths).
- [ ] At least one test exercises the model **through a scenario**, not only in
      isolation.
- [ ] `requirements.toml` updated with verification evidence; traceability green.
- [ ] FC portability lock intact (no new `openbmp-fc` edge to a forbidden crate).
- [ ] Dual-use: change is forward-only; if it touches guidance/optimization/AFTS,
      the terminal-condition lock is preserved and a proving test was added.
- [ ] Parity ceiling for this increment stated in the PR (what is *not* validated).

---

## 4. Design-doc template (what the per-dimension docs contain, and how to read them)

Every per-dimension doc is structured identically so the agent can navigate
fast:

1. **Parity target & ceiling** — the achievable capability end-state and the
   honest validation ceiling + open substitute.
2. **Current state in source** — verified file paths and maturity (the baseline
   to regress against).
3. **Target architecture** — new/changed crates; Rust trait surfaces and data
   structures; the math (named formulations, equations, references); the
   **fidelity tiers** `T0…Tn`.
4. **Invariant preservation** — how determinism, FC-portability, byte-stable
   default, provenance, and labels are kept for this dimension.
5. **V&V plan** — the verification cases (MMS, analytic, code-to-code, public
   benchmark), tolerance tables, and which label each tier earns.
   (Dual-use considerations live in the per-WP `dual_use_note` fields, which
   remain mandatory.)
7. **Dependencies** — on other parity docs / WPs.
8. **Open-source leverage** — tools/datasets with licenses and use mode
   (port / couple / ingest).
9. **Work-package backlog** — the executable list (§5).
10. **References.**

---

## 5. Work-package backlog schema

Each doc ends with a backlog. Every entry is a self-contained, reviewable unit
with this shape. The WP heading carries the authoritative title; the remaining
fields are mandatory and are checked by `python3 scripts/check_parity_docs.py`:

```
WP-<doc>.<tier>[-<letter>] — <title>
                    e.g. WP-05.1 — Add pressure-thrust + altitude term
  goal:             one-paragraph outcome and why it matters for parity
  fidelity_tier:    Tn (from the doc's ladder)
  depends_on:       [other WP ids]  (must be merged first)
  new_crates:       [] or proposed crate + layer (review boundary before building)
  touched:          crates/files expected to change
  approach:         the concrete method/formulation to implement (point at the
                    doc section with the equations)
  acceptance:       bullet list of TESTABLE criteria, e.g.
                      - new model off by default; canonical goldens byte-identical
                      - <analytic/MMS case> matches to <tolerance> in a tolerance table
                      - <code-to-code/benchmark> case added with provenance
                      - all §2 gates green
  validation_label: target label this WP earns (experimental|checked|validated-toy|research)
  dual_use_note:    forward-only consideration, or "far from line"
  est_effort:       rough size (days/weeks) from the research ladder
  parity_ceiling:   what this WP does NOT validate
```

The agent executes backlog entries in `depends_on` order, one PR each. When a WP
proposes a `new_crate`, the first commit is the crate skeleton + its placement
justification (layer, dependency edges, why it does not break the DAG or the FC
lock), reviewed before the implementation lands.

---

## 6. The solver-consumer boundary rule (operational)

When a fidelity tier calls for high-fidelity physics that production tools get
from a heavyweight solver (RANS/DSMC/FEM/radiation/material-response), the agent
**does not** write that solver. Instead, the WP delivers:

1. an **in-repo reduced/inviscid/correlation tier** that runs deterministically
   with no external dependency and supports code-verification (MMS / analytic);
2. an **ingestion path**: a parser + schema for the external solver's output,
   landing in `data/<thing>/` with `provenance.md`, SHA pin, and a validation
   label;
3. a **UQ wrapper**: per-entry bias/random margins on the ingested data, flowing
   into the database object and Monte Carlo;
4. a **coupling adapter**: the runtime consumes the ingested deck through the
   existing model trait surface; and
5. a **code-to-code validation case**: the in-repo tier vs the external solver on
   a shared open benchmark, with a tolerance table and a documented "what is not
   validated" note (`docs/verification.md` Hypersonic V&V & UQ ladder).

This keeps OpenBMP self-contained and deterministic while making external
high-fidelity data first-class and trustworthy — the production posture without
the production solver-maintenance burden.

---

## 7. Recommended first PRs (cheap, isolated, high-value)

Start here — each is small, unblocks others, and proves the workflow end-to-end:

1. **`WP-05.1` propulsion pressure-thrust + altitude** — consume the stored
   `exit_area`, add `F = ṁ·Vₑ + (pₑ−pₐ)·Aₑ` with `pₐ(h)` from the atmosphere
   crate, split sea-level/vacuum Isp. Fixes a known 5–15% vacuum-thrust error;
   one crate; immediate `validated-toy` win.
2. **`WP-12.0`/`WP-11.0` deterministic MC crate** — replace the ad-hoc N=16
   orbit Monte Carlo with a reusable `openbmp-mc` on `DeterministicRng` +
   Welford statistics + Clopper-Pearson intervals. Substrate for everything in
   Phase A.
3. **`WP-08.1` full tesseral gravity** — Pines kernel + normalized Gottlieb
   recursion, runtime-selectable degree/order, EGM2008 coefficients ingested
   with provenance. Improves every trajectory; strong `research`-label benchmark
   path (cross-check the two recursions against each other).
4. **`WP-01.1` spatial-vector rigid tree** — `SpatialInertia`, Plücker
   transforms, `Joint` enum, ABA forward dynamics; regress byte-for-byte against
   the current single-body kernel for the no-joint case. The multibody substrate
   the whole flex/coupled-control program needs.
5. **`WP-07.0` wire the differential corrector** — give the existing unwired
   `openbmp-trajopt` corrector a real offline driver/CLI subcommand against the
   forward propagator. Turns dead code into a tested capability with no new math.

Each of these is isolated, respects every invariant, and demonstrates the
gate-green PR loop before the heavier physics tracks begin.

---

## 8. Definition of done (work-package level)

A WP is done when: it implements its tier's named formulation; it is
byte-deterministic and off-by-default with goldens unchanged; every §2 gate and
§3 checklist item is satisfied; it earns its declared validation label with a
tolerance table and validity envelope; its uncertainty is accounted where the
dimension requires it; its parity ceiling is stated; and requirements
traceability + provenance are complete. Anything less is reported as
in-progress, not done.

---

*Companion documents:* `00-overview.md` (constitution) · `01`–`12`, `14`–`25`
(per-dimension design).
