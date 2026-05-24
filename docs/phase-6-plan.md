# Phase 6 Plan

Phase 6 picks up from the closed Phase 5 and Phase 5.X work and ships
the **hypersonic / re-entry research extensions** described in
[`docs/hypersonic-extensions.md`](hypersonic-extensions.md): high-altitude
atmosphere, real-gas thermodynamics, hypersonic aero methods, aerothermal
heat transfer, boundary-layer state, continuum-to-rarefied bridging,
re-entry trajectory infrastructure, validation suite, 1-D thermal toy,
Park 2T nonequilibrium thermochemistry, generic ablation toy, external
reference packages, and UQ / credibility reporting.

The hypersonic-extensions document remains authoritative for the
mathematical scope, validity envelopes, safety-boundary posture, and
roadmap rationale. This plan tracks the **delivery status** of each
sub-phase landing on `main`.

## Scope and anti-scope

**In scope (Phase 6):**

- Solver profile typing and implicit source-term sub-stepper for
  stiff chemistry and thermal-response coupling.
- NRLMSISE-00 static-defaults atmosphere (Earth, F10.7 = 150, Ap = 4
  mid conditions, altitude as the only run-varying input). Full
  empirical coefficient-based path declared as
  `Nrlmsise00Full` and reserved.
- Tannehill 5-species equilibrium air `γ_eff(T, p)` and speed of
  sound; Mugalev 11-species reserved.
- Modified Newtonian, tangent-cone, tangent-wedge hypersonic aero
  methods.
- Knudsen number, Cheng / erfc / linear bridge functions,
  Schaaf-Chambré free-molecular aero, hybrid dispatch.
- Stagnation heating: Fay-Riddell (1958), Sutton-Graves,
  Tauber-Sutton radiative engineering form.
- Boundary-layer state, three transition models (empirical, e^N
  placeholder, Re_θ / M_e), reference-enthalpy distributed heating.
- 1-D thermal-conduction toy with Fourier-stability fail-closed.
- Park 2T nonequilibrium thermochemistry with Park87 reaction set,
  Millikan-White / Park vibrational relaxation, geometric-mean
  temperature for dissociation rates.
- Generic ablation toy: steady-state and charring archetypes with
  blowing correction, surface recession integration.
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
| 6.0 — Hypersonic solver profile + implicit Euler sub-stepper | shipped | `openbmp_sim::SolverProfile`, `implicit_euler_step`. |
| 6.1 — NRLMSISE-00 static-defaults | shipped | `openbmp_physics::Nrlmsise00Static`. Full path declared `Nrlmsise00Full` (reserved). |
| 6.2 — Tannehill 5-species equilibrium air | shipped | `openbmp_physics::TannehillEquilibriumAir`. Mugalev 11-species reserved. |
| 6.3 — Hypersonic aero methods | shipped | `ModifiedNewtonian`, `TangentCone`, `TangentWedge`, `hypersonic_similarity_parameter`. |
| 6.4 — Stagnation heating | shipped | `openbmp_aerothermal::{FayRiddell, SuttonGraves, TauberSuttonRadiative}`. |
| 6.5 — Boundary layer + distributed heating | shipped | `BoundaryLayerState`, `EmpiricalTransition`, `EnTransition`, `ReThetaTransition`, `ReferenceEnthalpyHeating`. |
| 6.6 — Knudsen bridging + free-molecular aero | shipped | `mean_free_path_m`, `knudsen_number`, `ChengBridge`, `ErfcBridge`, `LinearKnudsenBridge`, `FreeMolecularAero`, `HybridAeroMethod`. |
| 6.7 — Trajectory infrastructure | shipped | `EntryInterfaceBuilder`, `AllenEggers`, `Vinh`. |
| 6.8 — Validation suite | shipped (analytic-toy battery) | `crates/openbmp-physics/tests/hypersonic_validation.rs`. Apollo / Stardust public-benchmark cases reserved for the kernel-side e2e slice. |
| 6.9 — 1-D thermal-conduction toy | shipped | `openbmp_aerothermal::OneDThermalToy`. |
| 6.10 — Park 2T nonequilibrium thermochemistry | shipped (Park87 baseline) | `openbmp_physics::ParkTwoTemperatureModel`. Park90 / Park93 reserved. |
| 6.11 — Generic ablation toy | shipped | `openbmp_aerothermal::{SteadyStateAblator, CharringAblator, BlowingCorrelation}`. Depth-resolved pyrolysis-front reserved. |
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
- **Real-gas-coupled Fay-Riddell** — the Phase-6.4 shipped form uses
  cold-gas post-shock conditions. The real-gas path requires
  wiring the `TannehillEquilibriumAir` outputs into the Fay-Riddell
  edge-state computation.
- **Park 1990 / Park 1993** reaction sets (11-species, ions/electrons).
  Typed-reserved variants reject at scenario load until the
  follow-on lands.
- **`Nrlmsise00Full` coefficient-based path** — typed-reserved; the
  static-defaults profile covers the immediate hypersonic-scenario
  use cases.
- **Mugalev 11-species equilibrium air** — reserved.
- **Depth-resolved charring-ablator pyrolysis-front** — the trait-
  level constant-progress proxy ships; the depth-resolved model
  lands when a charring-scenario benchmark calls for it.
- **`LocalInclinationPanels` mesh-based aero** — the Phase-6.3 single-
  station representative-panel approximation ships; mesh integration
  with shadowing lands when a non-axisymmetric scenario requires it.

These follow-on slices are tracked here so a future sub-phase can
pick them up without re-deriving the scope conversation.
