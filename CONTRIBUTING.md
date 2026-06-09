# Contributing to OpenBMP

Thanks for your interest in OpenBMP. This document describes how to
contribute and the scope review every change goes through.

## Project posture

OpenBMP is an academic, simulation-only research platform with explicit
scope limits. Contributions are welcome — within the boundaries set in
[`docs/safety-boundaries.md`](docs/safety-boundaries.md).

All contributors are also expected to follow
[`CODE_OF_CONDUCT.md`](CODE_OF_CONDUCT.md).

## Workflow

1. Open or pick up an issue describing the change. For non-trivial
   changes, propose the design in the issue first.
2. Fork the repository and create a topic branch.
3. Implement, test, and document the change. See § Code, § Tests, and §
   Documentation below.
4. Run the full local check sequence (§ Local Verification).
5. Open a pull request. The PR description must explicitly answer the
   scope and validation review questions (§ Scope Review).
6. CI runs the full gate suite. Reviewers add comments; address them
   and push fixes. We prefer many small commits over a single large
   commit during review.
7. A maintainer merges once CI is green and review is complete.

## Protected Branch Gates

The `main` branch must be protected in GitHub repository settings. Pull
requests may merge only after the required CI status checks below pass
and after code-owner review from `.github/CODEOWNERS`; project-scope
and validation checks are not optional even for maintainer or
solo-author changes.

Required status checks from `CI`:

- `rustfmt`
- `clippy`
- `build`
- `test`
- `coverage`
- `doc-tests`
- `cargo-deny`
- `cargo-audit`
- `cargo-machete`
- `cargo-hack feature powerset`
- `msrv`
- `typos`
- `rustdoc`
- `HAL portability`
- `FC dependency graph tripwire`
- `lockstep-clock tripwire`
- `requirements traceability`
- `fc HAL build-time gate`
- `determinism-gate`

Changes that alter these job names, remove one of the checks, or make a
required check advisory-only must be reviewed as a governance change and
reflected in this list in the same PR.

## Code

- Rust **1.95** stable, **Edition 2024**, MSRV pinned at `1.95`.
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
determinism, validation status, data provenance, and scope boundary**.

## Documentation

A change is not complete without documentation. Whenever any of the
following shifts, update the relevant doc in `docs/`:

- API surface, trait shapes, or workspace layout.
- Project scope, validation boundaries, or data-provenance rules.
- Frame, time, units, or telemetry conventions.
- Data sources or provenance requirements.
- Determinism profile or CI-gate behavior.

The [`docs/`](docs/) set is the source of truth.

## Scope Review

Every PR must answer these questions in the PR description (per
[`docs/safety-boundaries.md`](docs/safety-boundaries.md)):

1. Can this run without any physical hardware?
2. Are data sources synthetic, public, or user-supplied with provenance?
3. Does the change avoid claims of operational flight readiness or
   certification?
4. Are assumptions, units, frames, noise models, and validation status
   documented?
5. Does the feature introduce real-bus protocols, device-driver code,
   or hard real-time guarantees? If yes, document the boundary clearly.

## Naming Discipline

Use clear domain names in code, scenario fields, telemetry channels, and
documentation. Scenario parser lints enforce unit suffixes and vector-frame
suffixes; vocabulary choices are handled through ordinary review and
documentation clarity.

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
