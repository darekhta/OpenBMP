# OpenBMP Documentation

**OpenBMP** — **Open Body Motion Platform** — is a Rust-first,
simulation-only research platform for rigid-body flight dynamics, focused on
rocket-class ascent, launch vehicles, propulsive landers, and lifting
re-entry.

> OpenBMP is an academic simulation platform. It is not validated for
> operational flight, not suitable for hardware deployment, not a weapon
> system, and not a substitute for any qualified flight-software stack.

This directory is the authoritative source for the project's scope,
architecture, safety posture, and data policy. Start with the design concept
and the architecture; reach for the others as needed.

## Orientation

- [Design Concept](design-concept.md) — purpose, principles, vehicle classes,
  and capabilities.
- [Software Architecture](software-architecture.md) — layered crate design,
  the simulation kernel, integrators, frames, models, the virtual flight
  controller, telemetry, and the testing taxonomy.
- [Roadmap](roadmap.md) — what is shipped and validated, shipped at research
  grade, and deferred.

## Contracts and policy

- [Safety Boundaries](safety-boundaries.md) — accept / reject rules, review
  checklist, naming rules, dependency rules, provenance requirements.
- [Dual-Use Assessment](dual-use-assessment.md) — the dual-use threat model:
  the forward-not-inverse rule, the enforcement tiers, the residual surface,
  and per-capability rationale.
- [Verification](verification.md) — validation labels, golden telemetry,
  tolerance tables, fuzzing, and public-benchmark rules.
- [Data Provenance](data-provenance.md) — required source records, source
  classes, transformation rules, machine checks, and the inline-data
  tripwires that fail the build on benchmark-data smuggling.
- [Supply Chain](supply-chain.md) — Rust dependency policy, release SBOM,
  dependency checks, and build-provenance expectations.
- [Modeling Guide](modeling-guide.md) — the model-author contract:
  documentation template, validation evidence, and safety posture.
- [Acceptable-Use Policy](../ACCEPTABLE-USE.md) and
  [Export-Control & Dual-Use Notice](../EXPORT-CONTROL.md) — intended use,
  out-of-scope use, export posture, and user responsibility (repository root).

## Reference

- [Scenario Format](scenario-format.md) — the TOML scenario contract: required
  tables, parsing rules, safety-limited names, and batch metadata.
- [Frames and Time](frames-time.md) — frame profiles, WGS84 / ECI / ECEF /
  local conventions, epoch metadata, and leap-second / EOP handling.
- [Mission Graph Architecture](mission-graph-architecture.md) — the
  hierarchical mission state machine, orthogonal regions, the single source of
  truth, the `openbmp-mission` / `openbmp-scenario-script` split, and the HAL
  adopter contract.
- [Mission States Vocabulary](mission-states-vocabulary.md) — canonical
  state-name vocabulary with academic citations, and the rejected
  operational-vocabulary table.
- [Hypersonic & Re-entry](hypersonic-extensions.md) — high-altitude
  atmosphere, real-gas thermodynamics, hypersonic aerodynamic methods,
  aerothermal heat transfer, boundary-layer state, continuum-to-rarefied
  bridging, and the re-entry trajectory tools.
- [Real-Rocket Integration](real-rocket-integration.md) — how a downstream
  adopter assembles a rocket-class vehicle on top of OpenBMP, using the
  fictional `ARV-Reference` worked example: the vehicle-assembly tree, engine
  cluster, control effectors, tank dynamics, the event timeline, the real-data
  credibility contract, the cross-validation playbook, and the generic lab-HIL
  adapter pattern.
- [Glossary](glossary.md) — shared vocabulary for frames, time, determinism,
  validation labels, and safety terms.

## Flight profiles

Design specs for the multi-phase ascent → coast → apogee → descent → entry
flight profile. These are forward-physics trajectory mechanics under the
platform's existing non-weapon guardrails; the capabilities ship as
fail-closed schema and trait stubs until each lands with validation evidence.

- [Flight Profiles Architecture](flight-profiles-architecture.md) — umbrella:
  the phase canon, the academic vocabulary, the safety posture, and how the
  companions fit together.
- [Staging and Separation](staging-and-separation.md) — executing multi-body
  stage separation: jettison, separation impulse, simultaneous spent-stage
  propagation.
- [Ascent Guidance](ascent-guidance.md) — gravity-turn / pitch-program /
  explicit reference-trajectory generation for powered ascent.
- [Ballistic Coast and Apogee](ballistic-coast-and-apogee.md) — exo-atmospheric
  coast, apogee detection, and the range-safety landing footprint.
- [Descent and Entry Profiles](descent-and-entry-profiles.md) — wiring
  Allen-Eggers / Vinh into live entry phases and recovery.
- [Profile Vocabulary and Guardrails](profile-vocabulary-and-guardrails.md) —
  accepted / rejected vocabulary and the fail-closed validation that keeps
  profiles non-weaponizable.
