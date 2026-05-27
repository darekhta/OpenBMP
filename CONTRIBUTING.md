# Contributing to OpenBMP

Thanks for your interest in OpenBMP. This document describes how to
contribute and the safety-boundary review every change goes through.

## Project posture

OpenBMP is an academic, simulation-only research platform with explicit
scope limits. Contributions are welcome — within the boundaries set in
[`docs/safety-boundaries.md`](docs/safety-boundaries.md). Anything
outside those boundaries is rejected on safety grounds, regardless of
technical merit.

All contributors are also expected to follow
[`CODE_OF_CONDUCT.md`](CODE_OF_CONDUCT.md), the
[`ACCEPTABLE-USE.md`](ACCEPTABLE-USE.md) policy, and the export-control
posture in [`EXPORT-CONTROL.md`](EXPORT-CONTROL.md).

## Workflow

1. Open or pick up an issue describing the change. For non-trivial
   changes, propose the design in the issue first.
2. Fork the repository and create a topic branch.
3. Implement, test, and document the change. See § Code, § Tests, and §
   Documentation below.
4. Run the full local check sequence (§ Local Verification).
5. Open a pull request. The PR description must explicitly answer the
   safety-boundary review questions (§ Safety Review).
6. CI runs the full gate suite. Reviewers add comments; address them
   and push fixes. We prefer many small commits over a single large
   commit during review.
7. A maintainer merges once CI is green and review is complete.

## Code

- Rust **1.95** stable, **Edition 2024**, MSRV pinned at `1.93`.
- All public items have `///` doc comments. `missing_docs` is
  `deny`-level.
- `unsafe_code` is `deny` workspace-wide. If you genuinely need it,
  opt the affected crate out with a documented justification.
- Use `thiserror` for library error types; `anyhow` only at binary
  boundaries.
- The deterministic kernel (`openbmp-core`, `openbmp-sim`, all model
  crates) must not access wall-clock time, system RNG, network, threads,
  or unordered iteration. The Banned Patterns list in
  [`docs/software-architecture.md`](docs/software-architecture.md) is
  authoritative.
- `tokio` is permitted **only** in `openbmp-bridge`. Other crates must
  not depend on it (directly or transitively in a way that activates a
  multi-thread runtime).

## Tests

Every change ships its own tests. The test classes per
[`docs/verification.md`](docs/verification.md) are:

- Unit tests (`cargo nextest`) — small deterministic behavior.
- Property tests (`proptest`) — invariants, fail-closed behavior.
- Snapshot tests (`insta`) — structured diagnostic surfaces.
- Golden scenario tests — byte-stable telemetry for canonical
  scenarios. Updates require explicit review.
- Analytic-toy validation — closed-form physics checks.
- Public-benchmark validation — academic reference cases.
- Doc tests — every public example compiles and runs.

For new physics models, follow the model README template in
[`docs/modeling-guide.md`](docs/modeling-guide.md) and document
**purpose, inputs/outputs, units/frames, validity range, assumptions,
determinism, validation status, data provenance, and safety boundary**.

## Documentation

A change is not complete without documentation. Whenever any of the
following shifts, update the relevant doc in `docs/`:

- API surface, trait shapes, or workspace layout.
- Safety boundary, accept/reject rules.
- Frame, time, units, or telemetry conventions.
- Data sources or provenance requirements.
- Determinism profile or CI-gate behavior.

The [`docs/`](docs/) set is the source of truth.

## Safety Review

Every PR must answer these questions in the PR description (per
[`docs/safety-boundaries.md`](docs/safety-boundaries.md) Review
Questions):

1. Can this run without any physical hardware?
2. Does it avoid real fielded-vehicle parameters and operational
   performance claims?
3. Is every controller output consumed only by simulator-local models,
   or by the optional generic socket bridge in test scenarios?
4. Does it avoid targeting, terminal homing to real-world locations,
   and payload-delivery behaviour?
5. Is the feature useful for academic simulation even if all real-world
   vehicle data is removed?
6. Are assumptions, units, frames, noise models, and validation status
   documented?
7. Does the feature introduce hard real-time guarantees, real-bus
   protocols, or device-driver code? **(If yes, the contribution is
   rejected.)**

If the answer to any of 1–6 is "no," the contribution is outside the
project boundary. PR authors must reframe the contribution or accept
that it will be declined on safety grounds, not on quality grounds.

## Dual-Use Review Gate

Changes that touch guidance, the landing / footprint / dispersion surface,
entry, the estimator lanes, MPC, or scenario **input types** are
"near-the-line" and carry an extra gate on top of the Safety Review above
(see [`docs/dual-use-assessment.md`](docs/dual-use-assessment.md)):

- The change must be **forward-only**: it answers *given vehicle and
  trajectory, what happens?* — never *given a place to reach, what to do?*
- It must add **no input** that names or accepts a desired location, target,
  aimpoint, real-world waypoint, or miss-distance, and **no** accuracy / CEP
  metric scored against a target.
- The PR must declare the **enforcement tier** that binds it. A near-the-line
  capability lands only when a **Tier-1 (architectural)** constraint binds it —
  an input type in which the operational objective is *unconstructible*.
  Documentation and policy are layered on top, never in place of it.

The forward-not-inverse checklist is built into the
[pull request template](.github/pull_request_template.md).

## Naming Discipline

Avoid operational vocabulary in code, scenario fields, telemetry
channels, and documentation. The parser-level lint will reject names
matching the operational reject list (see
[`docs/safety-boundaries.md`](docs/safety-boundaries.md) Naming Rules
and [`docs/scenario-format.md`](docs/scenario-format.md)
Safety-Limited Names). Prefer the academic vocabulary documented in
[`docs/glossary.md`](docs/glossary.md).

## Local Verification

Before pushing:

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --all-features --locked -- -D warnings
cargo nextest run --workspace --all-features --locked
cargo deny check
cargo audit
cargo machete
cargo hack check --workspace --feature-powerset --locked
typos
```

Or, equivalently, the CI aliases:

```bash
cargo ci-fmt
cargo ci-clippy
cargo ci-test
cargo ci-deny
cargo ci-audit
cargo ci-hack
```

## Data Contributions

Data files (`data/`, `scenarios/`, `tests/validation/`) require a
provenance record per [`docs/data-provenance.md`](docs/data-provenance.md).
A data PR without a provenance record will not pass CI.

## License

Contributions are dual-licensed under Apache-2.0 OR MIT (see
[`README.md`](README.md) § License). By submitting a contribution, you
agree that your contribution is licensed under the same terms.
