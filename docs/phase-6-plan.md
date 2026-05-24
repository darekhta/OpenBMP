# Phase 6 Plan

Phase 6 picks up from the closed Phase 5 and Phase 5.X work and targets
the **hypersonic / re-entry research extensions** described in
[`docs/hypersonic-extensions.md`](hypersonic-extensions.md): high-altitude
atmosphere, real-gas thermodynamics, hypersonic aero methods, aerothermal
heat transfer, boundary-layer state, continuum-to-rarefied bridging,
re-entry trajectory infrastructure, validation suite, 1-D thermal toy,
Park 2T nonequilibrium thermochemistry, generic ablation toy, external
reference packages, and UQ / credibility reporting. The table below is
the source of truth for what actually shipped, what was downgraded, and
what is deferred after audit.

The hypersonic-extensions document remains authoritative for the
mathematical scope, validity envelopes, safety-boundary posture, and
roadmap rationale. This plan tracks the **delivery status** of each
sub-phase landing on `main`.

## Scope and anti-scope

**In scope (Phase 6):**

- Solver profile typing, implicit source-term sub-stepper primitive,
  runner dispatch for source-term profiles, and deterministic
  telemetry metadata for the selected source-term controls. The
  profile wrapper delegates trajectory integration today; coupled
  chemistry / material source adapters land with verified models.
- NRLMSISE-00 static-defaults atmosphere (Earth, F10.7 = 150, Ap = 4
  mid conditions, altitude as the only run-varying input). Full
  empirical coefficient-based path declared as
  `Nrlmsise00Full` and reserved.
- `EquilibriumAir` trait surface with `TannehillEquilibriumAir` and
  Mugalev 11-species typed-reserved. The audit removed the
  unverified synthesized Tannehill table.
- Modified Newtonian, tangent-cone, tangent-wedge, and basic
  mesh-panel local-inclination hypersonic aero methods.
- Knudsen number, Cheng / erfc / linear bridge functions,
  Schaaf-Chambré free-molecular aero, hybrid dispatch.
- Stagnation heating: Sutton-Graves verified engineering
  simplification; Fay-Riddell cold-gas scaffold; Tauber-Sutton
  radiative model typed-reserved pending published coefficients.
- Boundary-layer state, three transition models (empirical, e^N
  placeholder, Re_θ / M_e), reference-enthalpy distributed heating.
- 1-D thermal-conduction toy with Fourier-stability fail-closed.
- Park 2T nonequilibrium thermochemistry API surface with Park87 /
  Park90 / Park93 typed-reserved. The audit removed the unverified
  five-reaction proxy and species-independent Millikan-White constants.
- Generic ablation toy: steady-state, charring archetypes, blowing
  correction, surface recession integration, and a deterministic
  energy-limited 1-D pyrolysis-front toy.
- Allen-Eggers analytic-toy ballistic-entry profile and Vinh
  lifting-entry equations as reference propagators.
- External reference package surface (CFD, DSMC, radiation,
  thermal-response, trajectory, thermochemistry) with strict
  envelope and provenance gates.
- UQ / credibility reporting: per-model uncertainty contributions,
  root-sum-square aggregation, NASA-STD-7009B-style status labels,
  Markdown report renderer.

**Explicitly out of scope (deferred / rejected):**

- Operational hypersonic-weapon parameter sets, terminal-evasion
  logic, penetration aids, targeting at any phase of hypersonic
  flight (categorical reject per
  [`docs/safety-boundaries.md`](safety-boundaries.md)).
- Real fielded TPS material parameters (PICA, AVCOAT, RCC, SLA-561V,
  FRSI, AFRSI, LI-900, MA-25S); only generic textbook archetypes
  ship.
- DSMC and CFD as in-repo solvers; OpenBMP can consume their offline
  outputs through the external-reference-package surface.
- Non-Earth atmospheres (Mars, Titan, Venus, Jupiter); framework
  could host them later but no planetary data ships.
- Park 1990 / Park 1993 reaction sets, full 11-species air,
  NRLMSISE-00 full coefficient-based path, Mugalev 11-species
  equilibrium air, e^N transition full integration, full Taylor-
  Maccoll cone shock — all carry typed-reserved variants and land
  in follow-on slices.

## Delivery status

Sub-phase status as of the latest commit on `main`.

| Sub-phase | Status | Notes |
|---|---|---|
| 6.0 — Hypersonic solver profile + implicit Euler sub-stepper | shipped (runner-dispatched scaffold) | `openbmp_sim::SolverProfile`, `implicit_euler_step`. `openbmp-cli` now accepts source-term profiles with explicit sub-step controls, wraps the selected trajectory integrator, and records solver metadata in telemetry. Coupled source-term state adapters remain follow-on work. |
| 6.1 — NRLMSISE-00 static-defaults | shipped (static interpolant) | `openbmp_physics::Nrlmsise00Static` table regenerated from public NRLMSISE-00 `gtd7` outputs for F10.7 = 150, Ap = 4, equator, noon, equinox. Full path declared `Nrlmsise00Full` (reserved). |
| 6.2 — Tannehill 5-species equilibrium air | deferred | `openbmp_physics::TannehillEquilibriumAir` now fails closed pending verified public table values. Mugalev 11-species reserved. |
| 6.3 — Hypersonic aero methods | shipped (checked approximations) | `ModifiedNewtonian`, modified-Newtonian `TangentCone`, `TangentWedge`, `LocalInclinationPanels`, `hypersonic_similarity_parameter`. Full Taylor-Maccoll validation and mesh-deck ingestion deferred. |
| 6.4 — Stagnation heating | partially shipped | `SuttonGraves` shipped; `FayRiddell` is a cold-gas checked scaffold; `TauberSuttonRadiative` fails closed pending published coefficients. |
| 6.5 — Boundary layer + distributed heating | shipped | `BoundaryLayerState`, `EmpiricalTransition`, `EnTransition`, `ReThetaTransition`, `ReferenceEnthalpyHeating`. |
| 6.6 — Knudsen bridging + free-molecular aero | shipped | `mean_free_path_m`, `knudsen_number`, `ChengBridge`, `ErfcBridge`, `LinearKnudsenBridge`, `FreeMolecularAero`, `HybridAeroMethod`. |
| 6.7 — Trajectory infrastructure | shipped | `EntryInterfaceBuilder`, `AllenEggers`, `Vinh`. |
| 6.8 — Validation suite | shipped (reduced analytic-toy battery) | `crates/openbmp-physics/tests/hypersonic_validation.rs`. Tannehill, Park-2T, Tauber-Sutton, Apollo, and Stardust public-benchmark cases reserved for follow-on slices. |
| 6.9 — 1-D thermal-conduction toy | shipped | `openbmp_aerothermal::OneDThermalToy`. |
| 6.10 — Park 2T nonequilibrium thermochemistry | deferred | `openbmp_physics::ParkTwoTemperatureModel` API surface exists, but Park87 / Park90 / Park93 all fail closed pending verified reaction tables and species-specific relaxation constants. |
| 6.11 — Generic ablation toy | shipped (checked toy) | `openbmp_aerothermal::{SteadyStateAblator, CharringAblator, DepthResolvedCharringAblator, BlowingCorrelation}`. Recession consumes caller-supplied heat flux; the depth-resolved model is energy-limited and generic, not a fielded TPS surrogate. |
| 6.12 — External reference packages | shipped | `openbmp_physics::ExternalReferencePackage` with provenance + envelope checks. |
| 6.13 — UQ + credibility reporting | shipped | `openbmp_physics::{ErrorBudget, UncertaintyContribution, ValidationStatus}`. |

## Success criteria

Phase 6 closes when:

1. Every sub-phase above has merged with its declared exit
   criterion met and its tests in CI.
2. The Phase-6 analytic-toy validation battery
   (`hypersonic_validation`) passes byte-stably on
   `x86_64-unknown-linux-gnu` across re-runs.
3. Every external reference dataset carries a sibling provenance
   record with the SHA-256 pin enforced by
   `openbmp_physics::ExternalReferencePackage::validate`.
4. No safety-boundary violation enters the repository: no
   operational hypersonic-weapon parameter sets, no fielded TPS
   material data, no terminal-evasion logic, no targeting at any
   phase of hypersonic flight.
5. The non-suitability disclaimer remains on every release artifact.

## Deferred to follow-on slices

- **Apollo / Stardust public-benchmark trajectories** — the analytic-
  toy battery is in place; the public-benchmark cross-validation
  needs the kernel-side rigid-body scenario wiring and the
  re-entry trajectory plumbing into `openbmp-cli`.
- **Coupled SolverProfile source adapters** — source-term profiles now
  dispatch through the runner and record telemetry metadata, but no
  coupled chemistry / material state adapter is wired until verified
  source models exist.
- **Tannehill 5-species equilibrium-air table** — deferred until a
  verified public table or clean-room implementation of the published
  correlation lands with provenance.
- **Real-gas-coupled Fay-Riddell** — the Phase-6.4 shipped form uses
  cold-gas post-shock conditions. The real-gas path requires a
  verified equilibrium-air implementation before wiring edge-state
  computation.
- **Tauber-Sutton radiative heating** — typed-reserved until the
  published Earth-entry piecewise-polynomial coefficients are imported
  with provenance.
- **Park 1987 / 1990 / 1993 reaction sets** — typed-reserved until
  verified public reaction tables, species-specific Millikan-White
  constants, and the Mach-15 shock-layer benchmark land.
- **`Nrlmsise00Full` coefficient-based path** — typed-reserved; the
  static-defaults profile covers the immediate hypersonic-scenario
  use cases.
- **Mugalev 11-species equilibrium air** — reserved.
- **Depth-resolved charring validation benchmark** — the energy-
  limited sharp-front toy ships; conduction-coupled char growth and
  public benchmark calibration remain deferred.
- **`LocalInclinationPanels` deck ingestion and validation** — the
  in-memory mesh-panel method with back-face shadowing ships; external
  mesh deck parsing and non-axisymmetric benchmark validation remain
  deferred.

These follow-on slices are tracked here so a future sub-phase can
pick them up without re-deriving the scope conversation.
