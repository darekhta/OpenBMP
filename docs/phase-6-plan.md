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

## Closeout decision

Phase 6 is closed on `main` as **complete with honest deferrals**. In
this context, "complete" means the shipped Phase-6 surfaces compile,
test, validate their inputs, and fail closed wherever the audit could
not verify public coefficients or benchmark histories. It does **not**
mean that every originally proposed high-fidelity model is executable.

The audit deliberately chose deferral over reconstruction for
Tannehill / Mugalev equilibrium air, automatic real-gas Fay-Riddell
edge-state generation, Tauber-Sutton radiative heating, executable
Park-2T source terms, the full NRLMSISE-00 coefficient path, and full
Apollo / Stardust trajectory-history reproduction. Those are follow-on
sub-phases, not blockers for closing Phase 6, because their current
repository state is typed-reserved and fail-closed rather than
plausible-looking guessed physics.

Future work that makes any deferred item executable must land as a
separate provenance-reviewed PR with source acquisition, transformation
steps, validation tolerances, and safety-boundary review documented
before the model graduates beyond `experimental` / deferred status.

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
  Mugalev 11-species typed-reserved. The shared composition type now
  carries neutral, ion, and electron mole fractions, but the audit
  removed the unverified synthesized Tannehill table.
- Modified Newtonian, tangent-cone, tangent-wedge, and basic
  mesh-panel local-inclination hypersonic aero methods.
- Knudsen number, Cheng / erfc / linear bridge functions,
  Schaaf-Chambré free-molecular aero, hybrid dispatch.
- Stagnation heating: Sutton-Graves verified engineering
  simplification; Fay-Riddell cold-gas scaffold plus a
  caller-supplied real-gas edge-state assembly path; Tauber-Sutton
  radiative model typed-reserved pending published coefficients.
- Boundary-layer state, three transition models (empirical, e^N
  placeholder, Re_θ / M_e), reference-enthalpy distributed heating.
- 1-D thermal-conduction toy with Fourier-stability fail-closed.
- Park 2T nonequilibrium thermochemistry API surface with Park87 / Park93
  forward Arrhenius coefficients pinned as-published for the neutral
  five-species subset plus source-derived species-pair Millikan-White
  coefficients for `N2`, `O2`, and `NO` oscillators. The live Park87 /
  Park90 / Park93 source-term models remain typed-reserved. The audit
  removed the unverified five-reaction proxy and scalar relaxation
  constants.
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
| 6.0 — Hypersonic solver profile + implicit Euler sub-stepper | shipped (sim-owned dispatch scaffold) | `openbmp_sim::{SolverProfile, ProfiledIntegrator, SimulationConfig::from_trajectory_profile}`, `implicit_euler_step`. `openbmp-cli` translates scenario solver blocks into the sim-owned profile-aware integrator, and the core kernel config can now be built directly from a trajectory `SolverProfile` with fixed-step `dt` agreement enforced. Source-term profiles still carry explicit sub-step controls and telemetry metadata. Coupled source-term state adapters remain follow-on work. |
| 6.1 — NRLMSISE-00 static-defaults | shipped (static interpolant) | `openbmp_physics::Nrlmsise00Static` table regenerated from public NRLMSISE-00 `gtd7` outputs for F10.7 = 150, Ap = 4, equator, noon, equinox. Full path declared `Nrlmsise00Full` (reserved) with full-input validation but no coefficient evaluator. |
| 6.2 — Tannehill 5-species equilibrium air | deferred | `openbmp_physics::TannehillEquilibriumAir` now validates the intended `(T, p/p0)` query envelope before failing closed pending verified public table values. `AirComposition` can represent and validate the reserved 11-species ion/electron surface, and `MugalevEquilibriumAir` validates the same reserved query envelope before returning its deferred error. |
| 6.3 — Hypersonic aero methods | shipped (checked approximations) | `ModifiedNewtonian`, modified-Newtonian `TangentCone`, `TangentWedge`, `LocalInclinationPanels`, strict panel-mesh TOML ingestion, `hypersonic_similarity_parameter`. Model parameters are checked for finite, non-negative areas / coefficients and physical cone/wedge angle bounds before force assembly. Mesh panels skip away-facing triangles when `shadowing = true`; when explicitly disabled, panels are evaluated two-sided by flipping away-facing normals. Full Taylor-Maccoll validation and non-axisymmetric public benchmark validation remain deferred. |
| 6.4 — Stagnation heating | partially shipped | `SuttonGraves` shipped with checked model/context parameters; `FayRiddell` has a cold-gas checked scaffold and a caller-supplied `FayRiddellEdgeState` path for real-gas edge/wall properties; automatic real-gas shock-layer edge-state generation remains deferred until verified equilibrium-air data land. `TauberSuttonRadiative` validates the shared stagnation context, then fails closed pending published coefficients. |
| 6.5 — Boundary layer + distributed heating | shipped | `BoundaryLayerState`, `EmpiricalTransition`, `EnTransition`, `ReThetaTransition`, `ReferenceEnthalpyHeating` with checked context fields, Reynolds number, and transitional intermittency before heat-flux assembly. |
| 6.6 — Knudsen bridging + free-molecular aero | shipped | `mean_free_path_m`, `knudsen_number`, `ChengBridge`, `ErfcBridge`, `LinearKnudsenBridge`, `FreeMolecularAero`, `HybridAeroMethod`. Invalid mean-free-path / Knudsen inputs and non-finite bridge inputs now fail closed to the free-molecular limit instead of silently selecting the continuum branch. |
| 6.7 — Trajectory infrastructure | shipped | `EntryInterfaceBuilder`, `AllenEggers`, `Vinh`. Entry-interface scalar inputs now reject non-finite flight-path and heading angles before any state construction. |
| 6.8 — Validation suite | shipped (public anchors + reduced analytic-toy battery) | `crates/openbmp-physics/tests/hypersonic_validation.rs`. Apollo 4 and Stardust public entry-interface anchors, the Apollo 4 sparse entry-event timeline, NASA/TP-2006-213486 table-13 heating benchmarks, and Stardust table-19/20 TRAJ input/output maxima are pinned in SI units, represented by a canonical SHA-256-pinned `ExternalReferencePackage`, and self-validate metadata, event ordering, peak quantities, heat loads, and radiative-fraction bounds; full model-to-trajectory-history and Tauber-Sutton radiative comparisons remain reserved with Tannehill and Park-2T. |
| 6.9 — 1-D thermal-conduction toy | shipped | `openbmp_aerothermal::OneDThermalToy` with checked material emissivity, backwall conditions, and revalidated mutable temperature-state shape before each step. |
| 6.10 — Park 2T nonequilibrium thermochemistry | deferred (SHA-256-pinned reference package) | `openbmp_physics::ParkTwoTemperatureModel` API surface exists, and the reserved `ReactionRates` container no longer hard-codes the rejected five-reaction proxy or accepts unpinned / malformed rate arrays. Park87 and Park93 expose the 17 neutral-subset forward Arrhenius coefficients from Zhang et al. (2022), Table 2, as published-unit reference data with a narrow SI evaluator for the table's stated units. The Millikan-White reference surface derives species-pair coefficients for `N2`, `O2`, and `NO` oscillators from molecular weights and vibrational characteristic temperatures. The Park reference payload is represented by a canonical SHA-256-pinned `ExternalReferencePackage`; the live source model still fails closed pending backward/equilibrium constants, the Park high-temperature relaxation limiter, and Mach-15 benchmark validation. Park90 still fails closed pending a verified public table. |
| 6.11 — Generic ablation toy | shipped (checked toy) | `openbmp_aerothermal::{SteadyStateAblator, CharringAblator, DepthResolvedCharringAblator, BlowingCorrelation}`. Recession consumes caller-supplied heat flux; toy materials, blowing inputs, and charring progress now fail closed on malformed values. The depth-resolved model is energy-limited, reports per-area pyrolyzed mass plus absorbed/pyrolysis/unused energy, and remains generic rather than a fielded TPS surrogate. |
| 6.12 — External reference packages | shipped | `openbmp_physics::ExternalReferencePackage` with trimmed provenance, payload-hash, and non-negative Mach/altitude envelope checks. Payload I/O remains at the consuming deck/model boundary. |
| 6.13 — UQ + credibility reporting | shipped | `openbmp_physics::{ErrorBudget, UncertaintyContribution, ValidationStatus}` with finite aggregate RSS checks. |

## Success criteria

Phase 6 closes when:

1. Every sub-phase above has merged with its declared exit
   criterion met and its tests in CI.
2. The Phase-6 analytic-toy validation battery
   (`hypersonic_validation`) passes byte-stably on
   `x86_64-unknown-linux-gnu` across re-runs.
3. Every external reference dataset that exists as a repository data
   file carries a sibling provenance record with the SHA-256 pin
   enforced by `openbmp_physics::ExternalReferencePackage::validate`.
   In-code scalar anchors and canonical reference-payload strings are
   allowed only under the narrow exception documented in
   [`docs/data-provenance.md`](data-provenance.md) and must be listed
   in the provenance-policy deviation log below.
4. No safety-boundary violation enters the repository: no
   operational hypersonic-weapon parameter sets, no fielded TPS
   material data, no terminal-evasion logic, no targeting at any
   phase of hypersonic flight.
5. The non-suitability disclaimer remains on every release artifact.

The audit closeout satisfies these criteria by shipping only checked /
tested code paths and explicitly reserving unverified physics behind
typed errors.

## Documented provenance-policy deviations

The default provenance policy prefers standalone files under `data/`,
`scenarios/`, or `tests/validation/` with sibling `provenance.md`
records. Phase 6 accepts the following scoped deviations because the
items are small public scalar anchors or canonical payload strings
whose reviewability is better served by keeping the exact values next
to the tests that pin them:

- **Apollo / Stardust scalar entry anchors in Rust constants** —
  Apollo 4 entry-interface values, sparse event times, Apollo CM
  table-13 heating values, Stardust table-13 heating values, and
  Stardust table-19/20 TRAJ scalar maxima are stored in
  `crates/openbmp-physics/src/reentry.rs` and exercised by
  `crates/openbmp-physics/tests/hypersonic_validation.rs`. The values
  carry public NASA source URLs in rustdoc / test context and are
  represented by a canonical SHA-256-pinned
  `ExternalReferencePackage`. No full trajectory-history data file is
  shipped, so a sibling `provenance.md` would be a weaker review
  artifact than the exact constants plus hash-pin test.
- **Park 2T public reference payload in Rust constants** — the Park87 /
  Park93 forward Arrhenius rows, species molar masses, vibrational
  temperatures, and Millikan-White formula are represented by
  `PARK_2T_REFERENCE_PAYLOAD_V1` and
  `PARK_2T_REFERENCE_PAYLOAD_SHA256_HEX` in
  `crates/openbmp-physics/src/realgas/park_2t.rs`. The payload is a
  canonical text string, validated by a SHA-256 test, and wrapped in an
  `ExternalReferencePackage`. This is a reference-data pin, not an
  executable Park source-term model.
- **NRLMSISE-00 static-defaults reference table in Rust constants** —
  `Nrlmsise00Static` stores the static mid-condition interpolation
  table in `crates/openbmp-physics/src/atmosphere/nrlmsise00.rs` with
  the public `gtd7(...)` regeneration inputs documented in module
  rustdoc. This remains a static interpolant, not the full coefficient
  model. If the table is expanded, regenerated by a tool, or promoted
  to a selectable external dataset, it must move to `data/atmosphere/`
  with a normal provenance record.

These deviations do not permit hidden coefficient imports, OCR-based
transcription without independent checks, guessed tables, third-party
model vendoring without license review, or any operational vehicle /
TPS / targeting data. Any future promotion of these constants into
runtime data files must remove the deviation entry and add normal
`provenance.md` coverage.

## Deferred to follow-on slices

- **Apollo / Stardust public-benchmark trajectories** — table-13
  heating / heat-load benchmark values, sparse public
  entry-interface anchors, the Apollo 4 sparse entry-event timeline,
  and the Stardust table-19/20 TRAJ input/output maxima are pinned in
  SI units, represented by a canonical SHA-256-pinned
  `ExternalReferencePackage`, and self-validate positive finite
  metadata, event ordering, peak quantities, heat loads, and
  radiative-fraction bounds. Full model-to-trajectory-history
  cross-validation still needs kernel-side rigid-body scenario
  wiring, radiative-heating coefficients, and re-entry trajectory
  plumbing into `openbmp-cli`.
- **Coupled SolverProfile source adapters** — source-term profiles now
  dispatch through `openbmp_sim::ProfiledIntegrator` and record
  telemetry metadata, but no coupled chemistry / material state
  adapter is wired until verified source models exist.
- **Tannehill 5-species equilibrium-air table** — deferred until a
  verified public table or clean-room implementation of the published
  correlation lands with provenance.
- **Automatic real-gas Fay-Riddell edge-state generation** — the
  Phase-6.4 trait implementation uses cold-gas post-shock conditions,
  and the new `FayRiddellEdgeState` path accepts caller-supplied
  real-gas edge/wall properties. In-crate edge-state computation still
  requires a verified equilibrium-air implementation before wiring the
  shock-layer solve.
- **Tauber-Sutton radiative heating** — typed-reserved until the
  published Earth-entry piecewise-polynomial coefficients are imported
  with provenance.
- **Park 1987 / 1990 / 1993 reaction sets** — Park87 / Park93 forward
  Arrhenius coefficients are pinned as published-unit reference data
  for the neutral five-species subset, and the neutral species-pair
  Millikan-White coefficients are derived from public molecular
  weights plus `N2` / `O2` / `NO` vibrational temperatures. The
  reference payload is wrapped in a canonical SHA-256-pinned
  `ExternalReferencePackage` so consumers can validate provenance and
  byte identity without treating the source-term model as executable.
  The live source-term model remains typed-reserved until backward /
  equilibrium constants, the Park high-temperature relaxation limiter,
  and the Mach-15 shock-layer benchmark land.
  Park90 remains fully reserved pending a verified public table.
- **`Nrlmsise00Full` coefficient-based path** — typed-reserved with
  full-input validation; the static-defaults profile covers the
  immediate hypersonic-scenario use cases.
- **Mugalev 11-species equilibrium air** — reserved.
- **Depth-resolved charring validation benchmark** — the energy-
  limited sharp-front toy ships; conduction-coupled char growth and
  public benchmark calibration remain deferred.
- **`LocalInclinationPanels` public benchmark validation** — the
  in-memory mesh-panel method with back-face shadowing ships, and a
  strict `openbmp.panel_mesh_aero = 1` TOML parser now constructs the
  same method through the existing `PanelMesh::new` validation path.
  Non-axisymmetric public benchmark validation remains deferred.

These follow-on slices are tracked here so a future sub-phase can
pick them up without re-deriving the scope conversation.

## External-source audit notes

The audit searched for public, citable coefficient sources before
leaving any model reserved:

- **Apollo / Stardust entry anchors.** NASA's Apollo 4 mission page
  (`https://www.nasa.gov/mission/apollo-4/`) publishes the Apollo 4
  entry altitude, flight-path angle, and velocity. NASA TN D-5399
  (`https://ntrs.nasa.gov/citations/19690029435`) publishes Apollo 4
  entry-control event timing, including entry interface, peak-g
  events, FINAL ENTRY near maximum skip altitude, parachute
  deployment, and landing; those sparse event times are now pinned and
  validated for ordering. NASA/TP-2006-213486
  (`https://ntrs.nasa.gov/citations/20060053240`) publishes the
  Stardust table-13 ballistic parameter, velocity, flight-path angle,
  peak heat flux, and total heat load. The same report's tables 19
  and 20 publish a TRAJ input deck summary plus peak convective /
  radiative / total heat flux, stagnation pressure, dynamic pressure,
  deceleration, and heat-load outputs; those scalar maxima are now
  pinned in SI units. The report still does not provide a complete
  machine-readable trajectory history suitable for model-to-history
  comparison, so full trajectory reproduction remains deferred.
- **Tannehill / Mugalev equilibrium air.** NASA CR-2470
  (`https://ntrs.nasa.gov/citations/19740026586`) and the later
  simplified curve-fit report (`https://ntrs.nasa.gov/citations/19870019018`)
  are public and describe equilibrium-air thermodynamic curve fits, but
  the available scans / text extraction were not clean enough to
  transcribe coefficients safely, and they do not provide a verified
  5-species mole-fraction table compatible with OpenBMP's
  `AirComposition` surface. The model remains fail-closed.
- **Tauber-Sutton radiative heating.** NASA/TP-2006-213486
  (`https://ntrs.nasa.gov/citations/20060053240`) cites the
  Tauber-Sutton relation and states the `q_rad = C r_N^A rho^B f(V)`
  form, but does not reproduce the Earth-entry coefficient / `f(V)`
  table. The AIAA source paper is public bibliographically but not
  available through NTRS, so the implementation remains typed-reserved.
- **Park 2T reaction rates.** NASA/TP-20230015593
  (`https://ntrs.nasa.gov/citations/20230015593`) confirms the Park-
  adapted 5-species, 17-reaction structure and Arrhenius form, but
  references Park's coefficients rather than reproducing the tables.
  The Park 1985 NTRS record (`https://ntrs.nasa.gov/citations/19990067211`)
  has no downloadable public PDF. Zhang et al. (2022), Table 2
  (`https://link.springer.com/article/10.1186/s42774-022-00125-x/tables/2`)
  publicly reproduces the Park1987 and Park1993 forward Arrhenius
  coefficients for the neutral five-species subset, so those rows are pinned
  verbatim as published-unit reference data with a converted
  `m^3 mol^-1 s^-1` evaluator for the table's stated units. That table
  does not provide the backward-rate / equilibrium-constant path or
  species-pair vibrational relaxation constants, so the live Park
  source model remains fail-closed. The audit also uses the public
  Millikan-White correlation as reproduced in NASA report
  `https://ntrs.nasa.gov/citations/19820011246` and the five-species
  vibrational temperatures published in OSTI report
  `https://www.osti.gov/servlets/purl/1650141` to derive species-pair
  `N2` / `O2` / `NO` relaxation coefficients. Those public rows and
  formulas are wrapped in a canonical SHA-256-pinned
  `ExternalReferencePackage` without restoring the rejected scalar
  constant.
- **NRLMSISE-00 full path.** The `nrlmsise00` Python package documents
  the wrapper as GPLv2 while its bundled C source is public domain
  (`COPYING.NRLMSISE-00`). A Rust coefficient path is feasible, but the
  public-domain C port is several thousand lines of fitted coefficients
  and recurrence code; importing it requires a dedicated clean-room port
  and validation slice rather than a quick audit patch.
