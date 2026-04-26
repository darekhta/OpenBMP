# OpenBMP Documentation

**OpenBMP** = **Open Body Motion Platform**: a Rust-first, simulation-only,
academic research platform for rigid-body dynamics with a primary focus on
rocket-class and launch-vehicle-class flight simulation.

> OpenBMP is an academic simulation platform. It is not validated for
> operational flight, not suitable for hardware deployment, not a weapon
> system, and not a substitute for any qualified flight-software stack.

## Start Here

- [Design Concept](design-concept.md) — purpose, principles, vehicle classes,
  MVP, phased roadmap.
- [Software Architecture](software-architecture.md) — crate selection,
  layered design, simulation kernel, integrators, frames, models, virtual
  flight controller, telemetry, scenario format, testing taxonomy.
- [Safety Boundaries](safety-boundaries.md) — accept / reject rules,
  review checklist, naming rules, dependency rules, provenance requirements.
- [Hypersonic Extensions](hypersonic-extensions.md) — Phase-6 research
  extensions: high-altitude atmosphere, real-gas thermodynamics, hypersonic
  aerodynamic methods, aerothermal heat transfer, boundary-layer state,
  continuum-to-rarefied bridging, re-entry trajectory infrastructure, and
  the hypersonic validation suite.
- [Scenario Format](scenario-format.md) — Phase-1 scenario DSL contract,
  required tables, parsing rules, safety-limited names, and batch metadata.
- [Verification](verification.md) — validation labels, golden telemetry,
  tolerance tables, fuzzing, public-benchmark rules, and review checklist.
- [Frames and Time](frames-time.md) — frame profiles, WGS84/ECI/ECEF/local
  frame conventions, epoch metadata, leap-second and EOP handling.
- [Data Provenance](data-provenance.md) — required source records, source
  classes, transformation rules, validation status, and machine checks.
- [Supply Chain](supply-chain.md) — Rust dependency policy, release SBOM,
  audit/vet checks, and build-provenance expectations.
- [Modeling Guide](modeling-guide.md) — stable placeholder for model author
  contracts, documentation template, validation evidence, and safety posture.
- [Glossary](glossary.md) — shared vocabulary for frames, time, determinism,
  validation labels, safety terms, and hypersonic terms.
- [Phase 1 Plan](phase-1-plan.md) — executable plan for the first
  implementation phase: workspace scaffolding, sub-phases with exit
  criteria, version-pinned dependencies (April 2026 ecosystem state), CI
  gates, and Phase-2 hand-off.
- [Real-Rocket Integration Cookbook](real-rocket-integration.md) — how a
  downstream user assembles a rocket-class vehicle on top of OpenBMP using
  the fictional `ARV-Reference` worked example: `VehicleAssembly` tree,
  `EngineCluster`, `ControlEffector`, tank/slosh moving-mass dynamics,
  event/phase timeline, multi-rate scheduling, the real-data package
  credibility contract, the cross-validation playbook, and the generic
  lab-HIL adapter pattern. Public-data integration (e.g., a peer-reviewed
  Starship-class study) is treated as a downstream-user example only.

The doc set above is the authoritative source for the project's scope,
architecture, safety posture, and data policy.
