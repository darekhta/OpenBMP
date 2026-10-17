# Aerodynamic Database Generation & CFD Coupling

**Status:** `experimental` (design intent; ships no code by itself).
**Audience:** the engineer or LLM agent implementing the parity work packages.
**One-line summary:** Bring OpenBMP's aerodynamics from a single representative
Modified-Newtonian panel + DATCOM-lite drag buildup up to production *method*
parity — a watertight-triangulation local-inclination panel solver, a
DATCOM-level component buildup, a vortex-lattice fin module, a CAPE-style
run-matrix orchestrator that ingests Cart3D/SU2/OpenFOAM/SPARTA results, and an
uncertainty-propagating aero-database object — while honoring the
solver-consumer posture (`00-overview.md` §1.1): OpenBMP ships the inviscid /
engineering tiers and the ingestion+UQ+coupling layer, never a from-scratch
production RANS/DSMC solver.

> Read `00-overview.md` (constitution, invariants, DAG) and
> `13-agent-execution-playbook.md` (gate set, WP schema) first. This document
> assumes both.

---

## 1. Parity target & ceiling

### 1.1 Target (capability / method parity, within posture)

The reference organizations (SpaceX, NASA, JPL, Rocket Lab) build a launch
vehicle aerodynamic *databook*: a coefficient database `C(M, α, β, …)` covering
the full ascent + entry envelope, assembled from a layered method stack —
engineering buildup (Missile-DATCOM class) at low cost, automated cut-cell
Euler (Cart3D/pyCart class) for the bulk of the matrix, viscous RANS
(FUN3D/SU2/OpenFOAM) where separation and base flow matter, DSMC (SPARTA) above
continuum breakdown — every database entry carrying a **per-node uncertainty
model** (wind-tunnel + CFD-numerical + model-form, RSS-combined to 3σ) that
flows into Monte-Carlo dispersion and loads. The databook produces not just
integrated `CA/CN/Cm` but **derived databases**: axial line loads, surface
pressure maps, hinge/protuberance/base increments, and stage-separation
proximity interference.

OpenBMP's achievable end-state (audit verdict: **approaching**, method):

| Capability | Reference practice | OpenBMP target |
|---|---|---|
| Hypersonic surface pressure | CBAERO / S/HABP / Mark IV arbitrary-body local inclination | **T1** watertight-tri panel solver + BVH shadowing + line loads + CG moments — in Rust |
| Subsonic→supersonic deck | Missile DATCOM | **T2** Allen-Perkins crossflow + carryover K-factors + area-rule wave drag + VLM fins — in Rust |
| Per-entry uncertainty | NASA SLS/Orion aero databook UQ (RSS WT+CFD+model-form) | **T3** RSS error budget + GCI helper + correlated draw into MC — in Rust |
| Nonlinear inviscid loads | Cart3D / SU2-Euler automated matrix | **T4** CAPE-style orchestrator + ingestion + GCI — *couple*, don't port |
| Viscous / rarefied loads | RANS (SA/SST) + DSMC | **T5** SU2/OpenFOAM/SPARTA ingestion + Knudsen-bridge calibration — *couple* |
| Stage-separation | multi-body interference databook | **T3/T4** parametric proximity-increment deck + dual-body superposition |

The deliverable is `openbmp-aerodb` (L6): an offline run-matrix orchestrator +
ingestion + UQ-carrying database object that *bakes* `AeroDeck` artifacts the
existing L2 `openbmp-aero` runtime consumes unchanged through the
`AeroMethod` / `DeckLookup` seam.

### 1.2 Parity ceiling (honest)

What **cannot** be matched in an open academic repo, with no hedging:

1. **Flight-correlated databooks.** Reference decks are anchored to flight
   reconstruction (vehicle telemetry) and proprietary wind-tunnel campaigns
   (transonic buffet, plume-induced separation, hot-plume base recirculation)
   at facility-calibrated accuracy. OpenBMP has no flight heritage and no WT
   access. Its `σ_WT` term is a *placeholder for a campaign it will never run*.
2. **Plume / base aerothermal coupling.** Real base drag and base heating
   depend on multi-species reacting exhaust-plume interaction (CEA/Cantera +
   reacting CFD + proprietary nozzle data); the open path only approximates.
   Cross-reference `05-propulsion-high-fidelity.md` (plume) and
   `04-aerothermal-realgas-and-tps.md` (base heating).
3. **Certification-grade UQ.** A flight-cert databook's uncertainty is
   *validated against flight*. OpenBMP's RSS margins are credible-but-uncalibrated;
   the `σ_modelform` term is honestly **inflated** in lieu of flight correlation.
4. **Turnkey high-fidelity DB generation.** Cart3D, FUN3D, CBAERO, and Missile
   DATCOM are export-sensitive and non-redistributable. OpenBMP can never *ship*
   high-fidelity DB generation — only optional couplings the user provisions
   under their own access.

**Credible open substitute.** Anchor every accuracy claim to a *published*
benchmark (ONERA M6, HB-2, AGARD-B, NASA TMR); use SU2 (LGPL) + OpenFOAM (GPL)
+ SPARTA (GPL) as the open high-fidelity stack the *user* runs; bound numerical
error with MMS + Roache GCI; obtain code-to-code agreement (OpenBMP panel/buildup
vs SU2-Euler vs Cart3D-where-available); inflate `σ_modelform` honestly; and
cross-check overall dispersion against the project's existing scraped-telemetry
external reference (LOCAL / casual only, never a flight input — see
`docs/external-telemetry-validation.md`). **State plainly: without flight
heritage the deck is a physics-faithful research deck, not a certified databook.**
The highest label any tier earns is `research`; most earn `validated-toy`.

---

## 2. Current state in source

Verified against the actual tree (`crates/openbmp-aero/`, ~5,600 LOC). The
crate is L2; it depends only on third-party math/serde primitives and **not**
on `openbmp-sim` (`crates/openbmp-aero/src/lib.rs:46-51`).

### 2.1 Hypersonic engineering methods — `src/hypersonic.rs` (896 LOC)

- `ModifiedNewtonian` (`hypersonic.rs:269-382`): a **single representative
  panel** approximation — an axisymmetric blunt body of nose radius +
  reference area, with `Cp = Cp_max·cos²(θ)` (local-normal convention). The
  body-frame force/moment is the stagnation-panel proxy:
  `F = (-Cp_max cos²α · q·S, 0, -Cp_max sin²α · q·S)`, `M_y = normal·L/2`. This
  is the principal gap: **1 panel, not a mesh**.
- `cp_max_perfect_gas(mach, gamma)` (`hypersonic.rs:299-315`): the
  Rayleigh-pitot normal-shock stagnation `Cp_max`; high-Mach γ=1.4 limit
  `≈ 1.8392` (`CP_MAX_PERFECT_GAS_INFINITE_MACH`, `:289`).
- `TangentCone` (`:389-474`) and `TangentWedge` (`:480-542`): modified-Newtonian
  tangent-cone (`Cp = Cp_max·sin²θ_c`) and 2-D Newtonian wedge
  (`Cp = 2sin²θ_w`). **Full Taylor-Maccoll integration is explicitly deferred**
  (`:11-12`, `:407-414`); both collapse to single-station axial drag.
- `PanelMesh` (`:67-148`): a validated triangulation (finite vertices,
  in-range indices, non-degenerate area) with deterministic two-manifold
  watertight validation and `PanelMeshBvh` median-split AABB ray queries.
- `LocalInclinationPanels` (`:167-253`): the mesh integrator. Iterates
  triangles, `cos θ = n̂·û_upstream`; the legacy `solve()` / `AeroMethod` path
  keeps sign-test shadowing, while opt-in `solve_with_bvh_shadowing()` builds a
  watertight `PanelMeshBvh` and skips otherwise facing panels whose centroid ray
  intersects another panel. Modified-Newtonian `Cp` only (`PanelMethod` enum has
  one variant, `:151-155`). `REQ-AERO-002` adds `PanelSolution` reporting with
  caller-supplied CG moment arms and optional axial station-bin force/moment
  loads, synthetic closed-form sphere `CD = Cp_max/2` and tangent-cone
  sharp-cone `CA = Cp_max sin²θ_c` tolerance-table regressions, plus synthetic
  nested-shell BVH occlusion evidence and a named wedge-cavity exact occlusion
  set fixture. The parser layer now has strict ASCII STL string/file and binary
  STL byte/file ingest for `PanelMesh`. `REQ-AERO-003` adds off-default
  `PanelSkinFriction` with
  reference-temperature laminar/turbulent `Cf`, tangential panel drag, and
  optional `[skin_friction]` panel-deck parsing; HB-2 high-Mach Figure 12b
  forebody `CA` evidence now covers the inviscid and skin-friction-enabled
  T1 paths, while calibrated CFD-output traction ingest remains later work.

### 2.2 Component buildup — `src/buildup.rs` (918 LOC)

- `ComponentBuildup` + `DragBuildupModel` trait (`buildup.rs:178-200`):
  forward coefficient producer → `AeroDeck` baker. Covers **CD only with a
  small-angle drag polar** plus a **linear `CN_α` from nose + fins**:
  skin friction (compressible turbulent + laminar/transition, `:257-300`),
  forebody wave drag (`forebody_pressure_wave_drag`, `:683-715`, a Mach-1.2
  blended supersonic nose term — *not* area-rule Sears-Haack), power-off base
  drag (`base_drag_coefficient`, `0.12+0.13M²` sub / `0.25/M` super, `:232`),
  boattail/flare afterbody, fin friction/pressure/normal-force-slope
  (`fin_normal_force_slope`, `:780`, a finite-wing slope × crude interference
  `1 + d/(2s+d)`, `:792`), and a flat 1% fin interference drag (`:314-316`).
  `REQ-AERO-033` adds an opt-in `DatcomComponentBuildup` selected by
  `[aero.buildup] datcom = true`; it keeps the legacy validation/deck path,
  replaces the legacy forebody wave term with a bounded Sears-Haack/area-rule
  style term in the opt-in zero-lift path, and replaces the legacy single
  fin-interference multiplier with bounded fin-on-body and body-on-fin
  carryover factors before adding a deterministic Allen-Perkins-style
  slender-body crossflow `CN` increment. Optional
  `[aero.buildup.protuberances]` adds a bounded equivalent-axisymmetric
  protuberance CD increment only when DATCOM mode is enabled. DATCOM-baked
  decks also carry optional principal `Cm_q`/`Cn_r`/`Cl_p` dynamic derivative
  tables with bounded damping signs; the deck schema can carry the full
  `Cl/Cm/Cn` by `p/q/r` derivative matrix. `DeckLookup` consumes those tables
  as body-rate moment increments when the rigid-body moment adapter supplies
  angular rates. The `datcom = false` path remains byte-compatible.
- **Missing vs full DATCOM:** NACA Report 1048 quantitative crossflow closure,
  quantitatively validated slender-body fin-body/body-fin carryover
  `K_W(B)`/`K_B(W)`, calibrated area-rule wave-drag closure, protuberance
  calibration, calibrated AGARD-B dynamic derivative closure beyond the
  bounded principal-term substrate, and quantitative cross-coupled derivative
  closure. `CM` is still a static normal-force moment sum before optional rate
  damping is applied at lookup time.

### 2.3 Deck, method seam, rarefied — `src/deck.rs` / `method.rs` / `knudsen.rs`

- `AeroDeck` (`deck.rs:127`): N-D (3≤N≤6) tabulated deck on
  `axis_order = ["mach","alpha","beta", …effectors]`, row-major flat tables,
  **locked-order innermost-first multilinear reduction** bit-identical to the
  Schema-1 trilinear at N=3 (Demmel & Nguyen 2020; FMA disabled; `deck.rs:40-69`).
  Always returns reduced `(CN, CD, CM)` and can now optionally carry explicit
  body-axis six-channel `(CX, CY, CZ, Cl, Cm, Cn)` tables. There are still
  **no surface pressures or line loads** in the runtime deck itself.
  `REQ-AERO-001` adds optional deck-native row-major
  one-sigma tables for the same reduced `(CN, CD, CM)` channels, with
  `AeroDeck::lookup_sigma` using the same locked-order interpolation stencil;
  it also adds strict deck-file metadata syntax for an exponential Mach
  uncertainty-correlation length. `REQ-AERO-005` adds a reduced-deck RSS/GCI
  assembler that populates those tables offline from per-node uncertainty
  sources. `REQ-AERO-006` adds an offline deterministic grid-correlated draw
  for reduced decks with sigma tables, and `REQ-AERO-007` wires an opt-in
  schema-v3 `[aero.deck_draw]` hook so the runner can consume one deterministic
  drawn deck sample at deck-load time, including optional alpha/beta covariance
  controls and deck-declared covariance metadata when the scenario omits an
  explicit Mach correlation length. `REQ-AERO-008`
  adds CLI campaign overlay scheduling plus a generated native campaign
  executor for deterministic per-sample deck-draw
  overlays, including sidecar-plus-base-scenario upstream aero UQ credibility
  reporting. `REQ-AERO-009` adds optional row-major six-channel tables,
  `AeroDeck::lookup_six`, optional six-channel sigma storage/lookup, strict
  `[six_coefficients.*]` and `[uncertainty.six_coefficients.*]` parsing, and
  an aerodb baker for force/moment sweeps that preserves reduced tables while
  adding explicit body-axis coefficients. `REQ-AERO-011` adds the matching
  six-channel RSS/GCI sigma assembler in `openbmp-aerodb`, with strict
  row-major TOML/CSV ingestion for per-node `CX/CY/CZ/Cl/Cm/Cn`
  fine/medium/coarse studies.
  `ExtrapolationPolicy::{FailClosed(default), Clamp}`; linear extrapolation
  deliberately absent (`:104-115`).
- `AeroMethod`/`DeckLookup` (`method.rs:91-198`): the pluggable seam. Body
  mapping uses explicit six-channel tables directly when present:
  `F = q·S·(CX,CY,CZ)`, `M = q·S·L·(Cl,Cm,Cn)`. Reduced decks keep the legacy
  fallback `F = q·S·(-CD,0,-CN)`, `M = q·S·L·(0,CM,0)`, with locked operand
  order.
  The runtime consumes *any* `AeroMethod` here — **this is the coupling point
  the new database object targets**.
- `knudsen.rs` (746 LOC): `FreeMolecularAero` (Schaaf-Chambré high-speed-ratio
  limit, `:254-320`), `GasRegime`, `HybridAeroMethod` (`:324`) dispatching
  continuum-low / continuum-high / free-molecular by Mach+Kn, and bridge
  functions (`ChengBridge`, `ErfcBridge`, `LinearKnudsenBridge`,
  `LinearMachBridge`). The **transitional regime is interpolated by a bridge,
  not physics** — the SPARTA-calibration target.

### 2.4 Parser, ingestion, UQ

- `parser.rs` (1029 LOC): strict TOML, `serde(deny_unknown_fields)`. Markers
  `openbmp.aero_deck = {1|2}` (coefficient decks) and
  `openbmp.panel_mesh_aero = 1` (panel mesh). Schema-1 and schema-2 coefficient
  decks may now include strict optional `[uncertainty]` tables for `cn`, `cd`,
  and `cm`; the parser rejects missing-channel, length-mismatched, non-finite,
  or negative sigma data fail-closed. `PanelMesh::load_ascii_stl_from_str`,
  `PanelMesh::load_ascii_stl`, `PanelMesh::load_binary_stl_from_bytes`, and
  `PanelMesh::load_binary_stl` provide strict ASCII STL string/file and binary
  STL byte/file ingest. The
  panel deck accepts optional `[skin_friction]` reference-temperature settings.
  Schema-1 and schema-2 coefficient decks may also include optional
  `[six_coefficients.*]` tables and optional
  `[uncertainty.six_coefficients.*]` tables for nominal runtime six-channel
  lookup. `draw_correlated_aero_deck` and the runner `[aero.deck_draw]` hook can
  perturb those six-channel sigma tables with separate deterministic
  Mach-correlated streams and optional deck-declared six-channel
  coefficient-correlation mixing.
  `openbmp-aerodb` now has a strict condition-only run-matrix TOML schema for
  offline reduced-deck baking, plus a schema-v3 `[aero.run_matrix]` asset
  declaration and runner handoff that resolves, pins, and parses that file
  without making it a flight-runtime force source. It can also bake a reduced
  `AeroDeck` from a strict normalized external-solver coefficient CSV that
  echoes each run-matrix condition and carries per-row solver/case/provenance
  fields, bake an explicit six-channel body-axis deck from a strict normalized
  external-solver `CX/CY/CZ/Cl/Cm/Cn` coefficient CSV, from a strict normalized
  surface-Cp CSV over `PanelMesh` triangles with optional full body
  force/moment coefficient and axial-bin load reports, from recorded SU2
  `history.csv` coefficient files carrying final
  `CFx`/`CFz`/`CMy` or zero-sideslip `CD`/`CL`/`CMy` rows, and from recorded
  SU2 surface CSV vertex `Pressure_Coefficient` files mapped onto `PanelMesh`,
  from recorded Cart3D ASCII `.triq` files whose first per-vertex scalar is
  treated as Cp after geometry validation against `PanelMesh`,
  from recorded OpenFOAM sampled-surface raw Cp files mapped onto `PanelMesh`,
  from recorded OpenFOAM scalar field dictionaries whose named boundary patch
  Cp `value` entries are mapped in panel order onto `PanelMesh`,
  plus recorded OpenFOAM scalar/vector field dictionaries such as `nut` and
  `U` assembled into validated field sets from declared case-root file paths
  or discovered selected time directories, with recorded `polyMesh/boundary`
  patch metadata plus points/faces/owner/neighbour connectivity validation,
  derived face/cell geometry reconstruction, point/face/cell zone topology
  validation, and `turbulenceProperties` field-set plus calibrated coefficient
  semantics,
  from recorded SPARTA surface-force tables mapped by surface id onto
  `PanelMesh` panels and integrated as centroid force/moment loads,
  and from recorded OpenFOAM `forceCoeffs` tables carrying final zero-sideslip
  `Cd`/`Cl`/`CmPitch` rows. A strict recorded-solver case manifest can align
  each run-matrix node to a solver artifact path and verify supplied artifact
  bytes against its SHA-256 pin. A strict recorded-solver command plan can also
  align each run-matrix node to direct program/argv/env/working-directory
  intent plus expected output artifact pins, then project those outputs into
  the existing manifest verifier. The command-plan executor runs those direct
  argv/env commands without a shell, captures stdout/stderr digests, enforces
  per-case timeouts, and verifies declared output artifact SHA pins.
  `execute_with_root_retrying` adds bounded retry execution with explicit retry
  classes and per-attempt failure evidence while keeping unsafe paths and other
  static validation failures non-retryable.
  `render_cluster_schedule` renders deterministic Slurm `sbatch` array scripts
  from command plans, including MPI ranks per case, CPU/task requests, quoted
  argv/env, log paths, submit-command metadata, and a script SHA-256 digest,
  without submitting jobs.
  Rendered schedules can also be written and submitted through a direct command
  vector, parse Slurm job ids from submit output, and query scheduler state
  output into coarse pending/running/completing/terminal status evidence.
  `wait_for_completion_with_root` polls scheduler state with bounded attempts
  until successful completion or fail-closed terminal/exhausted states, and
  `collect_expected_outputs_with_root` verifies planned output artifacts under
  the solver root against their SHA-256 pins.
  `Su2EulerCaseTemplate` renders deterministic SU2-Euler configs for each
  run-matrix node, records config SHA-256 evidence, carries direct command
  metadata, and can materialize only those generated configs under an existing
  root. `REQ-AERO-032` adds a site-specific remote artifact staging hook that
  runs a caller-supplied direct transfer command under the solver root, captures
  stdout/stderr digest evidence, and then SHA-verifies every planned output
  artifact.
- UQ now exists in `openbmp-uq` (L1, doc 11), with source-tagged upstream
  margin helpers and scenario/runner hooks for declared `[aero.uq]` coefficient
  bands. `REQ-AERO-005` adds `GridConvergence` plus the `openbmp-aerodb`
  reduced-deck RSS sigma assembler, and `REQ-AERO-006` adds a deterministic
  Mach-correlated reduced-deck draw substrate. `REQ-AERO-007` adds
  schema-v3 `[aero.deck_draw]` plus runner deck-load consumption for a declared
  sample. `REQ-AERO-008` adds `openbmp mc aero-deck-draw-overlays`, which
  materializes per-sample `[aero.deck_draw]` merge overlays plus a
  campaign-manifest fragment, and `openbmp mc aero-deck-draw-campaign`, which
  generates a full campaign manifest and executes it through the native
  campaign runner with optional UQ sidecar credibility gating/reporting and
  base-scenario `[aero.uq]` upstream-budget merge; both CLI paths can omit
  `mach_correlation_length` so deck-declared covariance metadata supplies it at
  runtime. `REQ-AERO-011` adds strict
  six-channel GCI sigma ingestion for decks that already carry nominal
  six-coefficient tables, and `REQ-AERO-012` adds strict normalized
  six-channel solver coefficient CSV deck baking, and `REQ-AERO-013` adds
  six-channel coefficient-correlation metadata plus draw mixing, and
  `REQ-AERO-016` adds deterministic SU2-Euler case config generation. The
  `REQ-AERO-017` adds bounded retry execution for direct command plans, and
  `REQ-AERO-018` adds deterministic Slurm array-script rendering for MPI-rank
  case scheduling. `REQ-AERO-019` adds direct Slurm submit/state-query evidence.
  `REQ-AERO-020` adds bounded scheduler completion polling and SHA-verified
  expected-output collection under the solver root. `REQ-AERO-021` adds
  recorded OpenFOAM scalar boundary-field Cp parsing plus deck baking.
  `REQ-AERO-022` adds recorded OpenFOAM scalar/vector dictionary parsing and
  field-set validation. `REQ-AERO-023` adds declared OpenFOAM case-root field
  loading for recorded scalar/vector dictionaries. `REQ-AERO-024` adds
  deterministic discovery of supported scalar/vector field dictionaries in one
  selected time directory. `REQ-AERO-025` adds recorded `polyMesh/boundary`
  patch metadata parsing and field-patch validation. `REQ-AERO-026` adds
  recorded `polyMesh` points/faces/owner/neighbour connectivity validation.
  `REQ-AERO-027` adds common OpenFOAM `turbulenceProperties` model/field-set
  semantic validation. `REQ-AERO-028` adds recorded `polyMesh`
  point/face/cell zone topology validation. `REQ-AERO-029` adds recorded
  `polyMesh` face-normal/cell-volume reconstruction. `REQ-AERO-030` adds
  recorded calibrated turbulence-coefficient validation. `REQ-AERO-031` adds
  calibrated runtime six-coefficient validation. `REQ-AERO-032` adds
  site-specific remote artifact staging through direct transfer commands and
  SHA-verified expected-output collection. The doc-03 gap is narrower but still
  real: broader solver orchestration, field-output parsing, and full
  uncertainty database assembly remain open.
- Existing ingestion precedent: `data/aero/calisto-drag.{toml,csv}` with a
  SHA-256 pin verified in `tests/calisto_deck_pin.rs` against the upstream
  RocketPy CSV — the pattern every CFD ingestion WP follows.

### 2.5 Validation baseline (`docs/verification.md`)

The "Continuum Aerodynamics Buildup" section verifies skin friction vs
Blasius/Schlichting, base drag vs the reference formula, boattail reduction,
the drag polar, and bit-identical deck baking. The "Hypersonic V&V and UQ
Ladder" defines the 5-layer evidence chain this doc's tiers earn against.
**ONERA M6, AGARD-B, and high-fidelity GCI solver cases remain open beyond the
current HB-2 Figure 12b panel evidence.**

---

## 3. Target architecture

### 3.0 New crate: `openbmp-aerodb` (L6)

**Placement justification.** The database-assembly concern is *offline
orchestration*: generate a run matrix, drive (or ingest) external solvers,
assemble + UQ-wrap a deck, emit an `AeroDeck` artifact. That is L6 (the
"model registry / validation rules" layer in `docs/software-architecture.md`),
**above** the L2 `openbmp-aero` runtime it produces decks *for*, and it never
sits on the kernel hot path. Dependency edges: `openbmp-aerodb` →
`openbmp-aero` (L2, for `AeroDeck`/`PanelMesh`/`AeroMethod`) + `openbmp-uq`
(L1, doc 11, for `ErrorBudget`) + serde/`rayon`/`nalgebra`. It must **not** be
depended on by `openbmp-fc`, `openbmp-sim`, or anything ≤ L5 — verified by the
existing acyclic-layer lint. The in-Rust *solvers* (panel/VLM/buildup) stay in
L2 `openbmp-aero` (so the runtime can call them live if desired and so they
carry no orchestration deps); `openbmp-aerodb` *drives* them over a run matrix
and *bakes the result*. The new tiers extend the existing L2 crate
(`openbmp-aero`) for physics and add the L6 crate for orchestration/UQ/ingestion.

```text
openbmp-aero    (L2, EXISTING)  + watertight mesh + BVH + skin friction + line loads (T1)
                                + DATCOM buildup + VLM fins (T2)
                                + 6-coeff deck + per-node σ fields (T3 schema)
openbmp-uq      (L1, NEW doc 11) ErrorBudget/RSS/GCI consumed here
openbmp-aerodb  (L6, NEW THIS DOC)
  ├── runmatrix/     RunMatrix DOE over (M,α,β,Re,Kn) — flight-condition only
  ├── ingest/        StlMesh, Cart3DTri, SU2History, OpenFOAMForce, SpartaSurf parsers
  ├── orchestrate/   SolverDriver trait, subprocess runner, manifest+SHA pin
  ├── assemble/      DeckBuilder: nodes → AeroDeck + LineLoadDeck + σ tables
  ├── uncertainty/   per-node RSS + GCI + correlated multi-Mach draw
  └── derived/       LineLoadDeck, SurfacePressureDeck, HingeMomentDeck,
                     ProtuberanceDeck, BaseDeck, StageSepDeck
```

### 3.1 Rust trait surfaces & data structures

#### Panel method, generalized (T1, in `openbmp-aero`)

```rust
/// Per-panel pressure model (extends the current 1-variant enum).
pub enum PanelMethod {
    ModifiedNewtonian,                 // Cp = Cp_max cos²θ      (windward)
    TangentCone,                       // local cone half-angle  (sharp forebody)
    TangentWedge,                      // 2-D oblique-shock
    NewtonianShadowZero,               // classical, Cp_max = 2
}

/// Watertight triangulation with a build-time spatial index for ray-cast
/// shadowing. `closure_tol` rejects non-manifold meshes fail-closed.
pub struct ShadowingMesh {
    mesh: PanelMesh,                   // existing validated tri soup
    bvh: Bvh,                          // median-split AABB tree, byte-stable build
    stations: Vec<f64>,               // axial line-load bin edges (body +x̂)
}

impl ShadowingMesh {
    pub fn from_mesh(mesh: PanelMesh, n_stations: usize) -> Result<Self, AeroError>;
    /// True if the panel centroid is occluded along -û_upstream by any other
    /// panel (concave bodies). Convex bodies fall back to the sign test.
    fn occluded(&self, centroid: Vector3<f64>, upstream: Vector3<f64>) -> bool;
}

/// Output of one panel sweep at a single (M,α,β) node.
pub struct PanelSolution {
    pub coeffs: SixCoeff,              // CX,CY,CZ,Cl,Cm,Cn about a named CG
    pub line_loads: Vec<AxialLineLoad>,// per-station CN'(x), CA'(x)  (1/m)
    pub cp: Vec<f64>,                 // per-panel Cp (surface-pressure deck)
}

pub struct SixCoeff { pub cx:f64, pub cy:f64, pub cz:f64,
                      pub cl:f64, pub cm:f64, pub cn:f64 }

pub struct AxialLineLoad { pub x_m:f64, pub dcn_dx:f64, pub dca_dx:f64 }
```

The panel integrator gains a **reference-temperature skin-friction** axial term
(Eckert/Meador-Smart `T*`), a **CG-referenced** moment (`(rᵢ − r_cg) × F`), and
**axial binning** of each panel's contribution into `stations` to produce line
loads — the inputs `02-structural-dynamics-loads-slosh-pogo.md` needs for CLA.

#### Vortex-lattice fin module (T2, in `openbmp-aero`)

```rust
pub struct VortexLattice {
    panels: Vec<HorseshoePanel>,       // control point, bound vortex, normal
    mach: f64,                         // Prandtl-Glauert / Goethert scaling
}

impl VortexLattice {
    /// Assemble and solve A Γ = -V∞·n̂ (dense LU via faer/nalgebra), then
    /// Kutta-Joukowski section lift and Trefftz-plane induced drag.
    pub fn solve(&self) -> Result<VlmSolution, AeroError>;
    /// Hinge moment = ∮ ΔCp aft of the hinge line for a deflected surface.
    pub fn hinge_moment(&self, hinge: HingeLine) -> f64;
}
```

#### Database object, run matrix, ingestion (T3/T4, in `openbmp-aerodb`)

```rust
/// A flight-condition-only DOE node. The type CANNOT name an impact point.
/// (Dual-use lock — see §6; mirrors the trajopt terminal-condition lock.)
#[non_exhaustive]
pub struct RunNode {
    pub mach: f64, pub alpha_deg: f64, pub beta_deg: f64,
    pub reynolds: f64, pub knudsen: f64, pub altitude_m: f64,
    // NO range, NO aimpoint, NO target — enforced by a compile-fail test.
}

pub struct RunMatrix { pub nodes: Vec<RunNode>, pub seed: DeterministicSeed }

/// One ingested external-solver result, provenance-pinned.
pub struct SolverResult {
    pub node: RunNode,
    pub coeffs: SixCoeff,
    pub cp: Option<Vec<(PanelId, f64)>>,
    pub provenance: SolverProvenance,  // tool, version, sha256, grids, label
    pub gci: Option<GridConvergence>,  // 3-grid Roache GCI per coefficient
}

/// Adapter from any solver's output files to SolverResult.
pub trait SolverIngest {
    fn parse(&self, files: &IngestPaths) -> Result<SolverResult, AeroError>;
    fn tool(&self) -> SolverTool;      // Cart3D | Su2 | OpenFoam | Sparta | Stl
}

/// The uncertainty-carrying database object — the central new artifact.
pub struct AeroDatabase {
    pub deck: AeroDeck,                // nominal (the existing runtime type)
    pub sigma: SigmaTable,             // per (node, coefficient) 1σ RSS budget
    pub method_map: MethodMap,         // per region: which tier produced it
    pub derived: DerivedDecks,         // line loads, base, hinge, protuberance, sep
}

pub struct SigmaTable { /* row-major σ aligned to deck axes & coeffs */ }

impl AeroDatabase {
    /// Domain-separated correlated draw for Monte Carlo (NOT white noise):
    /// a Gauss-Markov kernel over Mach so adjacent rows move together.
    pub fn draw(&self, rng: &mut DeterministicRng) -> AeroDeck;
}
```

#### GCI helper (T3, lives with `openbmp-uq`, consumed by `openbmp-aerodb`)

```rust
pub struct GridConvergence { pub p_observed: f64, pub gci_fine: f64, pub asymptotic: bool }
impl GridConvergence {
    /// Roache GCI from three monotone grids f1(fine),f2,f3(coarse), ratio r.
    pub fn from_three_grids(f1:f64,f2:f64,f3:f64,r:f64) -> Result<Self,UqError>;
}
```

### 3.2 The math (named formulations & references)

**(M1) Local-inclination panel pressure** (Anderson 2019 ch.3; Gentry Mark IV
AFFDL-TR-73-159). For panel `i`, outward unit normal `n̂_i`, freestream unit
`v̂_∞(α,β)` in body frame:

```text
sinθ_i = max(0, -n̂_i · v̂_∞)            (impact angle; θ=0 ⇒ shadow ⇒ Cp=0)
Modified Newtonian (windward):  Cp_i = Cp_max · sin²θ_i
Cp_max = (p_t2/p_∞ − 1) / (½ γ M²)
p_t2/p_∞ = [ (γ+1)²M² / (4γM² − 2(γ−1)) ]^(γ/(γ−1)) · [ (1 − γ + 2γM²)/(γ+1) ]
```

(Rayleigh-pitot; identical to the relation already in `hypersonic.rs:299-315`.)
Force and moment about an arbitrary CG:

```text
F_body = q_∞ · Σ_i Cp_i (−n̂_i) A_i
M_body = Σ_i (r_i − r_cg) × ( q_∞ Cp_i (−n̂_i) A_i )
```

Shadowing: convex ⇒ sign test on `sinθ`; concave ⇒ BVH ray-cast of the panel
centroid against the mesh along `−v̂_∞`. **O(N_panels) per node, embarrassingly
parallel over the matrix (rayon).**

**(M2) Reference-temperature skin friction** (Eckert; Meador-Smart). Tangential
axial force recovers viscous drag the inviscid panel misses:

```text
T*/T_e = 1 + 0.032 M_e² + 0.58 (T_w/T_e − 1)
Cf = 0.455 / [ log10(Re_x*)^2.58 · (1 + 0.144 M_e²)^0.65 ]   (compressible turbulent)
F_axial,visc = q_∞ Σ_i Cf,i (v̂_∞ · t̂_i) A_i
```

**(M3) Allen-Perkins nonlinear body crossflow** (NACA Report 1048; Nielsen):

```text
CN_body = (k2−k1)(A_b/A_ref) sin(2α)cos(α/2) + η·Cdc·(A_planform/A_ref) sin²α
```

`η ≈ 0.6–0.8` (fineness-dependent crossflow proportionality), `Cdc` crossflow
drag coefficient. This is the principal nonlinear `CN` the current linear
buildup lacks.

**(M4) Wave drag — area rule / Sears-Haack lower bound** (Hoerner; Ashley-Landahl):

```text
D_wave = (9π/2)(Vol / L²)² · q_∞      (Sears-Haack minimum)
```

with the transonic-area-rule cross-sectional `S(x)` from geometry, replacing the
crude Mach-1.2-blended nose term in `buildup.rs:683`.

**(M5) DATCOM fin lifting-surface slope & carryover** (Blake DATCOM 2014 manual
AD1000581):

```text
CN_α,fin = 2π·AR / ( 2 + √( AR²β'²/κ² (1 + tan²Λ/β'²) + 4 ) ),  β' = √|1−M²|
Fin-body interference: K_W(B), K_B(W) slender-body carryover from (r/s).
```

**(M6) Vortex-lattice** (Katz & Plotkin ch.12-13; AVL). Flow tangency
`Σ_j A_ij Γ_j = −V_∞·n̂_i`, `A_ij` = Biot-Savart horseshoe influence; solve dense
`A Γ = b`; Kutta-Joukowski `dL = ρ V_∞ Γ dy`; Prandtl-Glauert `x → βx` subsonic;
hinge moment `= ∫ ΔCp` aft of the hinge line.

**(M7) Per-entry RSS uncertainty** (Roache GCI; ASME V&V 20-2009; Hemsch-Walker):

```text
σ_C = √( σ_WT² + σ_CFD² + σ_modelform² + σ_extrap² )
GCI_fine = Fs·|(f_fine − f_coarse)/f_fine| / (r^p − 1),  Fs=1.25 (3 grids)
p = observed order from (f1,f2,f3);  σ_CFD ← GCI_fine
σ_modelform ← |C_Cart3D − C_RANS| at overlap (fidelity-spread proxy, inflated)
Correlated draw: C(M_k) = C_nom(M_k) + L·z,  L = chol(Σ_Mach), z ~ N(0,I)
```

**(M8) Stage-separation interference** (Pamadi Ares I; Murman-Aftosmis-Nemec
AIAA 2004-5076). Parametric grid over relative geometry
`(Δx, Δy, Δz, Δθ, Δψ, M, α)`; per-body increments
`ΔC_body2(Δx,…) = C_proximity − C_isolated`; runtime superposes on each body's
isolated deck and integrates dual-body 6-DOF until interference → 0.

### 3.3 Fidelity tiers (`T0…T5`)

| Tier | Method | External solver? | Label ceiling |
|---|---|---|---|
| **T0** (current) | 1-panel Modified-Newtonian + tangent-cone/wedge + free-molecular + Knudsen bridge + DATCOM-lite drag buildup | none | `validated-toy` |
| **T1** | Watertight-tri local-inclination panels: STL/tri ingest, BVH shadow ray-cast, per-method Cp, reference-T skin friction, CG moments, axial line loads; rayon run-matrix → hypersonic deck | none (pure Rust) | `research` (HB-2, sphere-cone analytic) |
| **T2** | DATCOM-level buildup (Allen-Perkins crossflow, carryover K-factors, area-rule wave drag, protuberance, dynamic derivatives) + Rust VLM fin/grid-fin/hinge module; 0.1<M<8 deck | none (pure Rust) | `research` (NACA 1048, AGARD-B, AVL c2c) |
| **T3** | Per-node RSS UQ (GCI for `σ_CFD`, fidelity-spread for `σ_modelform`) + correlated draw → existing MC; stage-separation interference deck | none if σ_modelform honestly inflated | `validated-toy`→`research` |
| **T4** | Out-of-process Euler coupling: CAPE-style orchestrator ingests Cart3D/SU2-Euler; parse forces/Cp; assemble line loads + GCI; replaces σ_modelform with measured nonlinear loads | **couple** (SU2 LGPL default; Cart3D where Govt-released) | `research` (c2c + benchmark) |
| **T5** | Viscous (SU2 SA/SST or OpenFOAM) base/separated/buffet loads + SPARTA DSMC above continuum breakdown calibrating the Knudsen bridge; full multi-fidelity tagged DB | **couple** (SU2/OpenFOAM/SPARTA) | `research` (ONERA M6, TMR, c2c) |

Each tier is independently shippable (WP-03.t below). T1–T3 are pure-Rust and
self-contained; T4–T5 are *couplings* per the solver-consumer rule, never ports.

---

## 4. Invariant preservation

Concretely, for this dimension (`00-overview.md` §3):

1. **Byte-determinism.** Panel integration and VLM assembly use locked operand
   order (the established `deck.rs` reduction pattern), **no `mul_add`**, no
   wall-clock, no system RNG. The BVH build is **byte-stable**: median split on
   a fixed centroid sort key with index tie-break (no `f64`-keyed `HashMap`
   iteration, no pointer-address order). Run-matrix sweep results are merged in
   **node-index order**, not thread-completion order (rayon `collect` into an
   index-keyed `Vec`, then ordered reduce — same discipline doc 11/12 require
   for MC). The dense VLM LU uses a fixed pivot strategy; the order is documented
   and pinned by a byte-diff test.
2. **Byte-stable-by-default.** Every new model is **off until a scenario opts
   in**, following the `[vehicle.bending]` precedent. T1 is gated by
   `[aero.panel_method] enabled = true` (or `kind = "local_inclination_mesh"`);
   T2 by `[aero.buildup] datcom = true`; T3 σ by `[aero.uncertainty]`; T4/T5
   ingestion by an explicit `[aero.deck] source = "…"` pointing at a pinned
   data package. Existing goldens (Calisto, Phalcon-9, hypersonic regression)
   stay byte-identical: the `AeroDatabase.draw()` at `σ=0` or "no UQ block"
   returns the nominal `AeroDeck` bit-for-bit (property test).
3. **FC hardware-portability lock.** `openbmp-aerodb` is L6 and is **forbidden**
   from the `openbmp-fc` dependency closure by construction (`fc` depends only
   on hardware-portable crates; `fc_dependency_tripwire.rs`). The FC never sees
   a CFD subprocess, binary STL/CAD pipeline, or rayon — it consumes only the
   baked `AeroDeck` through the existing sensor/aero seam. No new `openbmp-fc`
   edge.
4. **Four-pillar provenance.** Every ingested CFD/STL artifact lands in
   `data/aero/<thing>/` with a sibling `provenance.md` (tool, version, SHA-256
   pin, grids, license status, validation label), verified fail-closed at load
   (`openbmp check-provenance`), exactly like `data/aero/calisto-drag.*`. No
   inline TOML fixtures in `*.rs` (`inline_data_tripwire.rs`). **No literal
   WGS84 `GM_earth`/`J2`/`R_earth` numerals** appear in any deck, schema, or
   doc — atmosphere/Reynolds inputs reference them symbolically through the
   environment crate.
5. **Validation labels.** Each tier declares exactly one label
   (`experimental`→`checked`→`validated-toy`→`research`) with a tolerance-table
   TOML for every numeric claim. The `method_map` tags each DB region with the
   producing tier *and* its label, and `SigmaTable` carries `σ` so the
   credibility record (NASA-STD-7009B-shaped, via `openbmp-uq`) is complete. No
   artifact ever claims `flight-qualified`/`certified`/`mission-ready`.
6. **Workspace lints & fail-closed.** `missing_docs`/`unsafe_code`/`float_cmp`
   stay `deny`; ingestion and lookup return `Result`, never panic on a
   scenario-reachable path; out-of-envelope queries fail closed
   (`ExtrapolationPolicy::FailClosed`).

---

## 5. V&V plan

The evidence chain is `docs/verification.md`'s 5-layer Hypersonic V&V & UQ
ladder. Tolerance tables ship as TOML next to each case.

### 5.1 Code verification (MMS / analytic)

| Case | Method | Tier | Tolerance | Label earned |
|---|---|---|---|---|
| Sphere Newtonian `CD` | closed-form `CD = Cp_max/2` (integrated) | T1 | rel ≤ 1e-3 | `checked` |
| Sharp cone tangent-cone `CA` | closed-form Newtonian/tangent-cone | T1 | rel ≤ 1e-3 | `checked` |
| Flat plate Cf vs Blasius/Schlichting | (already in repo) | T1/T2 | rel ≤ 1e-3 | `checked` |
| Panel-integrator unit (square plate) | `F = −Cp·q·A` exact (extends `hypersonic.rs:810`) | T1 | bit-exact | `checked` |
| GCI observed-order on a manufactured monotone triple | Roache GCI recovers `p` | T3 | abs `p` ≤ 0.05 | `checked` |
| BVH shadow correctness on a concave wedge-cavity | ray-cast vs hand-computed occlusion set | T1 | exact set | `checked` |

### 5.2 Model verification & public benchmark

| Case | Reference | Tier | Tolerance (target) | Label earned |
|---|---|---|---|---|
| **HB-2** forebody `CA` & static stability | AEDC-VKF/ONERA inter-facility, M 1.5–20 | T1 | `CA` within ±10% (engineering) | `research` |
| **Sphere-cone** integrated `CA` | Newtonian analytic + published | T1 | ±5% | `research` |
| **AGARD-B** supersonic stability derivatives | inter-facility dataset | T2 | derivative sign + ±15% | `research` |
| **NACA Report 1048** crossflow `CN` | Allen-Perkins bodies of revolution | T2 | ±10% on `CN(α)` | `research` |
| **AVL** lift slope, `Cm`, hinge moment | identical geometry, code-to-code | T2 | `CLα` within ±3% | `validated-toy` (c2c) |
| **ONERA M6** surface Cp (7 stations) | AGARD AR-138 Test 2308, TMR grids | T5 | Cp within published band | `research` |
| **NASA TMR** flat-plate / bump SA-SST | code-to-code vs CFL3D/FUN3D | T5 | machine-zero verification | `research` |
| **Apollo AS-202 / Stardust** entry | NTRS 20060053240 (already pinned) | T1/T3 | bracket table-20 peak/load | `research` |

### 5.3 Code-to-code (solver-consumer)

For T4/T5 the in-repo tier is validated **against the external solver on a
shared open benchmark** (`13` §6.5): OpenBMP panel/buildup `CA/CN/Cm` vs
SU2-Euler vs Cart3D-where-available on the same geometry, with a GCI'd grid
study and a documented "what is not validated" note. Code-to-code agreement is
*evidence*, not validation — it must be paired with one of the §5.2 public
benchmarks and a UQ story before any `research` promotion (`verification.md`
§"Hypersonic V&V and UQ Ladder", para. 263).

### 5.4 UQ / credibility

Every validation case carries an `ErrorBudget` (numerical tolerance via GCI,
model-form via fidelity spread, data pedigree, interpolation/extrapolation,
atmosphere variability) rendered into the NASA-STD-7009B-shaped credibility
record. The byte-diff determinism gate extends to the *correlated-draw
ensemble* (doc 11): the same seed → byte-identical drawn decks.

---

## 7. Dependencies on other parity docs

- **`00-overview.md`** — parity definition, invariants, the L6 placement of
  `openbmp-aerodb` and the solver-consumer posture this doc obeys.
- **`13-agent-execution-playbook.md`** — gate set, WP schema, the §6
  solver-consumer boundary rule (reduced tier + ingestion + UQ + coupling +
  code-to-code) that T4/T5 follow exactly.
- **`11-monte-carlo-uq-and-validation.md`** — promotes `uq.rs` → `openbmp-uq`
  (L1); the `ErrorBudget`/GCI helper lives there. The reduced-deck draw helper
  lives in `openbmp-aerodb`, the declared single-sample runner hook lives in
  `openbmp-runner`, and per-node σ tables still need broader MC campaign
  orchestration (`openbmp-mc`, L7). **Hard dependency for T3.**
- **`02-structural-dynamics-loads-slosh-pogo.md`** — consumes the T1/T4 **line
  loads** and surface-pressure decks for coupled loads analysis (CLA) and
  buffet forcing. The `AxialLineLoad` station binning is designed for it.
- **`04-aerothermal-realgas-and-tps.md`** — shares the `PanelMesh`/edge-state
  surface; the panel solver's per-station edge pressures feed distributed
  heating; the base/plume heating ceiling is shared (§1.2 item 2).
- **`05-propulsion-high-fidelity.md`** — plume model; base-drag/base-pressure
  coupling at the boattail is the joint open-problem boundary.
- **`08-environment-gravity-and-frames.md`** — supplies atmosphere ρ, T, μ,
  speed-of-sound, mean-free-path for the Reynolds/Knudsen run-matrix axes
  (referenced symbolically; never inline WGS84 numerals).
- **`06-gnc-coupled-mimo-and-control.md`** — consumes the VLM hinge-moment and
  grid-fin authority maps (T2) and the 6-coefficient deck for load-relief.
- **`12-determinism-realtime-and-compute.md`** — the rayon run-matrix and
  ordered-merge reduction follow its deterministic-parallel discipline.

---

## 8. Open-source leverage

| Tool | License | Mode | Use |
|---|---|---|---|
| **SU2** (SU2 Foundation) | LGPL 2.1 | **couple** (subprocess + parse) | Open default for Euler & RANS (SA/SST) deck generation, T4/T5. Parse force/breakdown + surface-Cp into decks/line loads. Subprocess only; do **not** vendor source. |
| **OpenFOAM** | GPL v3 | **couple** (process boundary only) | Alternative RANS/compressible (`rhoCentralFoam`/`rhoSimpleFoam`) for base/separated flow; ingest `postProcessing` force coefficients. **Never link/embed** — copyleft. |
| **SPARTA** (Sandia) | GPL | **couple** (MPI subprocess) | DSMC rarefied/transitional above continuum breakdown, T5; ingest sampled surface tractions to build the high-Kn end and calibrate the Knudsen bridge. |
| **CAPE / pyCart / pyFun** (NASA) | NOSA 1.3 | **ingest** (reference design) | Reference architecture for OpenBMP's Rust run-matrix orchestrator (case gen, job control, force/line-load extraction, DB assembly). Port the *structure*, not the code. |
| **Cart3D** (NASA Ames) | US-Govt-purpose (not OSS) | **couple** (where Govt-released) | Premier automated cut-cell Euler DB generator (T4) + multi-body stage-sep cases. Parse-only, access-gated; **never a default dependency or redistributed**. |
| **FUN3D** (NASA Langley) | NASA usage agmt (ITAR/EAR) | **couple** (where licensed) | Production unstructured RANS option; treat like Cart3D — optional, access-gated, parse-only. **Never vendor.** |
| **CBAERO** (NASA Ames) | US academic/Govt (not OSS) | **ingest** (methodology) | Spec for the Tier-1 arbitrary-body local-inclination + engineering aeroheating method. Use tool directly only under its release. |
| **Missile DATCOM** (AFRL) | export-controlled (manual readable) | **ingest** (formulations) | The 2014 manual (AD1000581) documents Allen-Perkins crossflow, carryover K-factors, base/wave-drag for Tier-2 native Rust. **Do not bundle the Fortran.** |
| **OpenVSP + VSPAERO** (NASA) | NOSA | **ingest** (geometry + cross-check) | Parametric watertight surfaces → STL/tri for the T1 mesh; independent VLM/panel cross-check for T2 fins. |
| **AVL** (Drela & Youngren, MIT) | free academic (not OSI) | **ingest** (validation ref) | Canonical VLM reference to validate the T2 Rust vortex-lattice (`CLα`, `Cm`, hinge moments) on identical geometry. |
| **NASA TMR** (tmbwg.github.io) | US-Govt public | **ingest** (V&V data) | Authoritative SA/SST definitions, verification grids, reference CFD (flat plate, bump, MMS, ONERA M6). Freely usable; cite. |
| **OSQP** (optionally IPOPT) | Apache-2.0 (OSQP) | **link** (orchestration only) | Run-matrix DOE / active-learning case placement / surrogate fitting. OSQP preferred (permissive). Not aero physics. |

**Process-boundary rule:** GPL solvers (OpenFOAM, SPARTA) are coupled *strictly*
as binaries we run and whose output files we read — no linking, no FFI, no
vendoring — to keep OpenBMP's Apache-2.0/MIT licensing clean.

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, gate-green per `13` §2. Effort is a
rough size from the research ladder.

---

**WP-03.1 — Generalize the local-inclination panel method to a watertight mesh**
- **goal:** Replace the single-representative-panel `ModifiedNewtonian` path
  with a full watertight-triangulation solver producing whole-body
  `Cp(α,β)`, six-coefficient loads about an arbitrary CG, and per-station axial
  line loads — the first step-change beyond T0 and the substrate every later
  tier integrates. Pure Rust, no external solver.
- **implementation_status:** implemented and traced by `REQ-AERO-002` /
  `V-AERO-002`. `PanelMesh` has watertight two-manifold validation and
  `PanelMeshBvh` ray-query infrastructure from related static-mask work.
  `LocalInclinationPanels` exposes `PanelSolution`, CG-referenced panel
  moments, and axial station-bin integrated force/moment loads while preserving
  the legacy `AeroMethod` origin-referenced output, plus a synthetic
  closed-form sphere `CD = Cp_max/2` tolerance fixture and a synthetic
  tangent-cone sharp-cone `CA = Cp_max sin²θ_c` tolerance fixture. It also adds
  opt-in BVH centroid-ray aerodynamic shadowing for watertight meshes with
  synthetic nested-shell occlusion evidence and the named wedge-cavity exact-set
  fixture. The public-domain `AEDC-TDR-64-137 / AD602769` HB-2 fixture
  digitizes three high-Mach Figure 12b forebody axial-force points and checks
  the generated local-inclination HB-2 forebody mesh within ±10% for Mach
  6.06, 7.10, and 8.07. It also adds strict ASCII STL string/file plus binary
  STL byte/file ingest for `PanelMesh`. Skin friction is tracked separately by
  WP-03.2; six-coefficient deck storage exists via `REQ-AERO-009`.
- **fidelity_tier:** T1
- **depends_on:** []
- **new_crates:** none (extends `openbmp-aero`, L2)
- **touched:** `crates/openbmp-aero/src/hypersonic.rs` (PanelMethod enum,
  ShadowingMesh, PanelSolution), `src/lib.rs` (re-exports),
  `tests/`, `data/aero/<mesh>/` + `provenance.md`
- **approach:** §3.2 (M1) per-panel Newtonian/tangent-cone/wedge Cp with
  `sinθ = max(0, −n̂·v̂_∞)`; BVH (median-split AABB, byte-stable build) for
  concave shadow ray-casts (sign test for convex); CG-referenced moment
  `(rᵢ−r_cg)×F`; axial binning into `stations` for `AxialLineLoad`. Strict
  ASCII/binary STL/tri ingest via `PanelMesh::load_ascii_stl(_from_str)` and
  `PanelMesh::load_binary_stl(_from_bytes)`; broader
  CFD-output ingest remains later work. rayon sweep over a `RunMatrix` (added
  here as a thin local type, promoted to aerodb in WP-03.4).
- **acceptance:**
  - new path off by default; Calisto/Phalcon-9/hypersonic goldens byte-identical
  - square-plate unit test stays bit-exact; sphere `CD` matches closed-form
    `Cp_max/2` to rel ≤ 1e-3 (tolerance-table TOML)
  - sharp-cone `CA` matches tangent-cone analytic to rel ≤ 1e-3
  - concave wedge-cavity BVH occlusion set matches the hand-computed set exactly
  - HB-2 forebody `CA` within ±10% with a documented validity envelope
  - two-run byte-diff identical on the reference profile; all §2 gates green
- **validation_label:** `research` (HB-2 + analytic) for the hypersonic CA case;
  `checked` for the unit/MMS cases
- **dual_use_note:** far from line; `RunNode` is condition-only — add the
  compile-fail test in WP-03.4 (the first WP introducing the shared matrix type)
- **est_effort:** 2–4 weeks
- **parity_ceiling:** inviscid only (no boundary layer this WP); shadowing
  validated on synthetic concave bodies, not flight geometry; HB-2 correlation
  is limited to digitized high-Mach forebody `CA` from Figure 12b, not base drag,
  low-Mach separation, viscous increments, or calibrated six-output WT closure.

---

**WP-03.2 — Reference-temperature skin friction on the panel solver**
- **goal:** Recover the viscous axial force the inviscid panel misses, so the
  T1 deck's total `CA` is credible across the supersonic/hypersonic envelope.
- **implementation_status:** implemented and traced by `REQ-AERO-003` /
  `V-AERO-003`. `PanelSkinFriction` adds opt-in reference-temperature skin
  friction through `LocalInclinationPanels::with_skin_friction`, tangential drag
  accumulation into total force, CG moments, and axial line-load bins, plus
  optional `[skin_friction]` panel-deck parsing. Formula tests cover the
  laminar and compressible turbulent coefficient paths, and a flow-aligned
  plate proves the default inviscid result is unchanged until skin friction is
  enabled. A synthetic sharp-cone smooth-wall trend fixture pins positive
  viscous axial increment, lower increment at higher Reynolds length, and small
  high-Re correction behavior. The public-domain `AEDC-TDR-64-137 / AD602769`
  HB-2 skin-friction fixture digitizes three high-Mach Figure 12b total
  forebody axial-force points, maps the report's Reynolds-number axis to
  centerbody diameter, enables smooth-wall reference-temperature skin friction,
  and checks the total forebody `CA` within ±10% while proving a positive
  viscous increment over the inviscid panel result.
- **fidelity_tier:** T1
- **depends_on:** [WP-03.1]
- **new_crates:** none
- **touched:** `crates/openbmp-aero/src/hypersonic.rs`, tests
- **approach:** §3.2 (M2) Eckert/Meador-Smart `T*` reference state; compressible
  turbulent + laminar/transition `Cf`; tangential `F_axial,visc = q Σ Cf,i
  (v̂_∞·t̂_i) A_i`. Reuse the existing `buildup.rs` Blasius/Schlichting forms.
- **acceptance:**
  - off by default; goldens byte-identical
  - flat-plate `Cf` matches Blasius (laminar) and White/Schlichting (turbulent)
    to rel ≤ 1e-3 (tolerance table)
  - synthetic sharp-cone smooth-wall trend evidence shows positive viscous
    increment, monotone reduction with Reynolds length, and small high-Re
    correction
  - public HB-2 Figure 12b high-Mach total forebody `CA` remains within ±10%
    with smooth-wall diameter-Reynolds skin friction enabled and shows positive
    viscous increment over inviscid panels
  - all §2 gates green
- **validation_label:** `checked` (analytic Cf); `research` (HB-2 high-Mach
  total `CA` envelope); `validated-toy` (synthetic combined CA trend)
- **dual_use_note:** far from line
- **est_effort:** 1 week
- **parity_ceiling:** flat-plate reference-enthalpy approximation; no transition
  prediction beyond a fixed `Re_crit`; no separation, base drag, roughness
  calibration, isolated boundary-layer measurement closure, or low-Mach HB-2
  separated-flow closure.

---

**WP-03.3 — Close ComponentBuildup to DATCOM level**
- **goal:** Bring the 0.1<M<8 deck from "drag + linear CN" to DATCOM-class:
  Allen-Perkins nonlinear crossflow `CN`, slender-body carryover K-factors,
  area-rule wave drag, protuberance drag, and dynamic derivatives.
- **implementation_status:** partial and traced by `REQ-AERO-033` /
  `V-AERO-033`. `DatcomComponentBuildup` is an opt-in wrapper over the legacy
  component buildup. It preserves the validation and deck-baking path, replaces
  the forebody wave term with a bounded Sears-Haack/area-rule-style term only
  when opted in, then adds bounded fin-on-body/body-on-fin carryover factors in
  the opt-in fin normal-force slope path plus a bounded slender-body crossflow
  normal-force increment that uses projected body planform area, projected
  centroid, and NACA Report 1048 body-I/body-II finite-length crossflow
  drag-ratio interpolation, plus a NACA equation-15-style incremental
  foredrag CD term; both are zero at zero angle of attack, and crossflow is
  odd in angle of attack. Optional
  `[aero.buildup.protuberances]` adds a bounded equivalent protuberance CD
  increment only when `datcom = true`. `AeroDeck` and the TOML
  parser accept optional row-major `[dynamic_derivatives.<moment>_<rate>]`
  tables for every `Cl/Cm/Cn` by `p/q/r` moment-rate derivative; missing tables
  default to zero, so legacy principal-only decks remain valid. The DATCOM
  baker emits bounded principal damping tables while the legacy baker leaves
  them absent. `AeroContext` carries airspeed and body angular rates;
  `DeckLookup` applies the full `qSL*D*(pL/2V, qL/2V, rL/2V)` moment-rate
  matrix and the rigid-body `AeroMethodMomentAdapter` supplies those rates
  while point-mass/no-rate contexts stay at zero. The parser-gated
  `DatcomBuildupValidationDeck` fixture harness now loads provenance-bearing
  TOML cases, evaluates the public `DragBuildupModel` surface, checks static
  coefficients plus full dynamic-derivative expectations against
  absolute/relative tolerances, and emits deterministic validation reports.
  A separate public-domain NACA Report 1048 equation fixture checks the
  Allen-Perkins crossflow `sin^2(alpha)*cos(alpha)` shape, projected-planform
  area scaling, and projected-planform center of pressure through the public
  crossflow increment seam without claiming digitized plot closure. Additional
  public NACA Report 1048 body-I/body-II fixtures check the report geometries
  at Mach 1.98 across four angles each. Public NACA Report 1048 body-I/body-II
  equation-15 force/moment fixtures now check normal force and bow
  pitching-moment coefficients at the same Mach/angle rows with an explicit 7%
  report-equation tolerance, and matching body-I/body-II incremental-foredrag
  fixtures check equation-15 delta-CD at the same rows; scanned plot residuals
  remain open. A second public NACA Report 1048 appendix fixture checks the body-I/body-II
  finite-length crossflow drag correction ratios and derived low-Mach
  `Cd_alpha=90` values without retuning the runtime model from two report
  points. A synthetic Sears-Haack/area-rule wave-drag fixture checks the
  subsonic, transonic-blend, and supersonic-decay branches of the opt-in
  wave-drag term without claiming calibrated wave-drag closure. Synthetic DATCOM
  fin/body carryover and
  equivalent-protuberance fixtures now check the radius/span factor branch and
  Mach-factor drag branch without claiming Missile-DATCOM or wind-tunnel
  calibration.
  Schema-v3 `[aero.buildup] datcom = true` selects it in the runner while the
  absent flag preserves legacy buildup output. Focused
  tests cover opt-in behavior, odd/nonlinear projected-planform and
  finite-length-corrected crossflow, bounded carryover factors replacing the
  legacy interference path only when opted in, reduced
  Von Karman supersonic zero-alpha CD in the DATCOM area-rule path, declared
  protuberance CD increments with unchanged zero-alpha CN/CM, DATCOM-only
  dynamic derivative deck tables with damping signs, full matrix parser/lookup
  support, locked-stencil runtime rate-moment application including
  cross-coupling terms, rigid-body angular-rate adapter propagation,
  provenance-pinned synthetic DATCOM-style validation fixture ingestion and
  36 scalar coefficient/derivative checks, public NACA Report 1048
  equation-level crossflow fixture checks, public NACA Report 1048 body-I/body-II
  geometry crossflow checks, public NACA Report 1048 equation-15 force/moment
  fixture checks, public NACA Report 1048 equation-15 incremental-foredrag
  fixture checks, public NACA Report 1048 appendix finite-length crossflow
  drag-ratio checks, Sears-Haack/area-rule wave-drag branch fixture checks,
  fin/body carryover factor fixture checks, and
  equivalent-protuberance drag fixture checks,
  subsonic zero-alpha bit identity while the area-rule/protuberance branches
  are inactive, deterministic deck baking, schema-1/schema-2 derivative-table
  parsing, scenario parsing/rejection for the protuberance block, and runner
  deck-load selection. This does not yet satisfy NACA 1048 scanned
  force/moment plot residual closure, AGARD-B stability derivative validation,
  quantitatively validated DATCOM carryover K-factors, quantitatively
  calibrated area-rule wave drag, quantitatively calibrated protuberance
  increments, or quantitatively calibrated dynamic derivative closure.
- **fidelity_tier:** T2
- **depends_on:** [WP-03.1]
- **new_crates:** none
- **touched:** `crates/openbmp-aero/src/buildup.rs`, deck schema (6-coeff +
  derivatives), tests, `data/aero/<datcom-cases>/`,
  `data/aero/naca-report-1048-body-i-crossflow-v1.toml`,
  `data/aero/naca-report-1048-body-ii-crossflow-v1.toml`,
  `data/aero/naca-report-1048-body-i-force-moment-v1.toml`,
  `data/aero/naca-report-1048-body-ii-force-moment-v1.toml`,
  `data/aero/naca-report-1048-body-i-incremental-foredrag-v1.toml`,
  `data/aero/naca-report-1048-body-ii-incremental-foredrag-v1.toml`,
  `data/aero/naca-report-1048-crossflow-drag-ratio-v1.toml` + provenance
- **approach:** §3.2 (M3) Allen-Perkins crossflow; (M4) Sears-Haack/area-rule
  wave drag replacing `forebody_pressure_wave_drag`; (M5) DATCOM fin slope +
  `K_W(B)`/`K_B(W)` carryover replacing the crude `1+d/(2s+d)` interference;
  protuberance increments; principal dynamic derivatives (`Cm_q`, `Cn_r`,
  `Cl_p`) via the documented DATCOM correlations, with the deck format ready
  for full cross-coupled derivative tables from external calibrated sources.
  Clamp/blend transonic regime carefully.
- **acceptance:**
  - off by default (`[aero.buildup] datcom = true`); legacy buildup goldens
    byte-identical when the flag is absent
  - crossflow `CN(α)` matches NACA Report 1048 bodies to ±10% (tolerance table)
  - AGARD-B supersonic stability derivatives match sign and ±15%
  - bit-identical deck baking over a fixed grid (extends the existing test)
  - all §2 gates green
- **validation_label:** `research` (NACA 1048 + AGARD-B)
- **dual_use_note:** far from line; vehicle-intrinsic coefficients only
- **est_effort:** 4–6 weeks
- **parity_ceiling:** semi-empirical regime-blending heuristics are not flight-
  calibrated; transonic blend is the weakest region; no live aeroelastic coupling.
- **requirements:** [REQ-AERO-033]

---

**WP-03.4 — `openbmp-aerodb` skeleton: RunMatrix + DeckBuilder + dual-use lock**
- **goal:** Stand up the L6 orchestration crate with the condition-only
  `RunMatrix`/`RunNode` DOE, the `DeckBuilder` that assembles in-Rust sweep
  results into an `AeroDeck`, and the **compile-fail dual-use lock** proving the
  matrix cannot express a ground aimpoint.
- **implementation_status:** implemented. `REQ-AERO-004` / `V-AERO-004` adds
  the `openbmp-aerodb` crate with
  condition-only `RunNode`, deterministic `RunMatrixAxes` / `RunMatrix`, a
  reusable `lint_condition_only_run_matrix_toml` guard for target / aimpoint /
  range key vocabulary, strict `openbmp.aerodb_run_matrix = 1` TOML parsing, and
  a `DeckBuilder` that evaluates an in-Rust `AeroMethod` sweep with rayon and
  explicitly merges by node index into the current reduced `AeroDeck`
  `(CN, CD, CM)` shape. The crate has no runner/scenario/FC/tooling dependency.
  `openbmp-testkit` adds a compile-fail no-ground-aimpoint tripwire, and the FC
  dependency tripwire now forbids `openbmp-aerodb`. Schema-v3 scenarios can
  declare `[aero.run_matrix]` with file/file_sha256, the scenario resolver
  includes that asset as `aero.run_matrix.file`, and the runner exposes
  `load_declared_aero_run_matrix` to parse it through `RunMatrix::load_toml_str`
  without turning it into a runtime aero force source.
  `DeckBuilder::bake_solver_coefficient_csv_deck` accepts a strict normalized
  external-solver coefficient CSV, validates row count, sequential node index,
  exact condition echo, finite `CN/CD/CM`, and nonblank solver/case/provenance,
  then bakes a reduced `AeroDeck` in run-matrix row-major order.
  `DeckBuilder::integrate_surface_cp_csv_loads` accepts a strict normalized
  surface-Cp CSV over an existing `PanelMesh`, validates node/panel row-major
  order, exact condition echo, finite `Cp`, nonblank solver/case/provenance,
  body-frame CG, and optional axial station bins, then integrates
  `F_i = -Cp_i q A_i n_i` into body-frame force/moment coefficients,
  `CN/CD/CM`, and axial load bins. `DeckBuilder::bake_surface_cp_csv_deck`
  delegates to that report path and projects the same results into the current
  reduced deck shape.
  `DeckBuilder::bake_su2_history_csv_deck` adds a solver-specific coefficient
  adapter by parsing one recorded SU2 `history.csv` per node,
  selecting the final coefficient row, preferring body-force `CFx/CFz/CMy`, and
  allowing `CD/CL/CMy` only when beta is zero.
  `DeckBuilder::integrate_su2_surface_cp_csv_loads` and
  `bake_su2_surface_cp_csv_deck` add a solver-native surface-pressure adapter
  for recorded SU2 surface CSV files by mapping finite vertex coordinates and
  `Pressure_Coefficient` values onto `PanelMesh`, averaging triangle Cp, and
  reusing the surface-Cp load/deck path.
  `DeckBuilder::integrate_cart3d_triq_surface_cp_loads` and
  `bake_cart3d_triq_surface_cp_deck` add a recorded Cart3D ASCII `.triq`
  adapter by validating TRI/Q vertices/faces against `PanelMesh`, using the
  first per-vertex scalar as Cp, averaging triangle Cp, and reusing the same
  load/deck path.
  `DeckBuilder::integrate_openfoam_surface_cp_loads` and
  `bake_openfoam_surface_cp_deck` add a recorded OpenFOAM sampled-surface Cp
  adapter by parsing whitespace-delimited `x y z Cp` rows, mapping them to
  `PanelMesh` vertices, averaging triangle Cp, and reusing the same load/deck
  path.
  `OpenFoamScalarField::parse_str`,
  `DeckBuilder::integrate_openfoam_boundary_cp_field_loads`, and
  `bake_openfoam_boundary_cp_field_deck` add a recorded OpenFOAM scalar field
  dictionary path: `FoamFile`, `dimensions`, `internalField`, and
  `boundaryField` scalar entries are parsed, a named boundary patch `value` is
  interpreted as uniform or one Cp value per panel, and the panel-order loads
  flow through the same load-report and reduced-deck path.
  `OpenFoamVectorField::parse_str` and `OpenFoamRecordedCaseFields` add
  recorded scalar/vector OpenFOAM field evidence for dictionaries such as `U`,
  `p`, `nut`, `k`, or `omega`, validating `FoamFile.object` names, dimensions,
  internal fields, boundary values, duplicate names, and malformed vector lists
  without claiming full case-directory traversal or turbulence-model semantics.
  `OpenFoamCaseFieldFile` and `OpenFoamRecordedCaseFields::from_case_root`
  load declared scalar/vector field dictionaries from relative paths under an
  existing recorded case root, reject unsafe traversal or unreadable files, and
  then apply the same field-set validation.
  `OpenFoamRecordedCaseFields::discover_time_directory` scans one selected
  recorded time directory in filename order, classifies supported scalar/vector
  field files by `FoamFile.class`, requires `FoamFile.object` to match the
  filename, and rejects unsupported classes or empty discoveries fail-closed.
  `OpenFoamPolyMeshBoundary` parses recorded `constant/polyMesh/boundary`
  metadata, validates patch `type`/`nFaces`/`startFace` entries and
  non-overlapping boundary-face ranges, and checks that discovered field patches
  are declared by the boundary metadata.
  `OpenFoamPolyMeshConnectivity` parses recorded `points`, `faces`, `owner`,
  and `neighbour` files, validates face point indices and owner/neighbour
  counts, and checks boundary face ranges against the parsed face list and
  internal-face prefix.
  `OpenFoamTurbulenceProperties` parses recorded `constant/turbulenceProperties`
  metadata and validates common active RAS/LES models against required recorded
  `U` and turbulence scalar fields.
  `DeckBuilder::integrate_sparta_surface_force_loads` and
  `bake_sparta_surface_force_deck` add a recorded SPARTA surface-force adapter
  by parsing whitespace-delimited surface-id plus `fx/fy/fz` rows, accepting
  zero- or one-based ids, requiring exact panel coverage with finite forces,
  and integrating centroid force/moment load reports before projecting into the
  reduced deck path.
  `DeckBuilder::bake_openfoam_force_coeffs_deck` consumes one recorded OpenFOAM
  `forceCoeffs` table per node, requires a `Cd/Cl/CmPitch` header, selects the
  final data row, and rotates wind-axis coefficients only for zero-sideslip
  nodes. `REQ-AERO-009` adds `DeckBuilder::bake_panel_six_coefficient_deck` for
  in-Rust force/moment sweeps, producing an `AeroDeck` with both reduced
  `CN/CD/CM` and explicit body-axis `CX/CY/CZ/Cl/Cm/Cn` tables. `REQ-AERO-012`
  adds `DeckBuilder::bake_solver_six_coefficient_csv_deck`, consuming strict
  normalized recorded solver `CX/CY/CZ/Cl/Cm/Cn` CSV rows and deriving reduced
  compatibility tables. External solver orchestration and full uncertainty
  database assembly remain open.
  `SolverCaseManifest::load_toml_str` adds a strict manifest for recorded
  out-of-process solver artifacts: one row-major case per run-matrix node,
  exact condition echo, solver/case/provenance text, artifact kind/path, and a
  SHA-256 pin. `verify_artifact_sha256_with_bytes` checks caller-supplied
  artifact bytes against those pins. It is parse/validation metadata only and
  does not launch or schedule solver processes. `REQ-AERO-014` adds
  `SolverCommandPlan::load_toml_str` for the first command-plan substrate:
  strict `openbmp.aerodb_solver_command_plan = 1` TOML validates one direct
  program/argv/env/working-directory/timeout plan per run-matrix node, stores
  expected output artifact pins, and projects those outputs into the existing
  manifest shape without executing a subprocess. `REQ-AERO-015` adds direct
  no-shell execution for those command plans: `execute_with_root` resolves case
  paths under a caller root, applies argv/env, captures stdout/stderr digest
  evidence, enforces timeouts, rejects failed processes, and verifies declared
  output artifact SHA pins. `REQ-AERO-016` adds the first WP-03.8
  solver-specific case generator: `Su2EulerCaseTemplate::render_cases` emits
  deterministic row-major SU2-Euler configs with condition comments, protected
  SU2 freestream/reference/output keys, config SHA evidence, and direct command
  metadata, while `write_configs_to_root` materializes only those configs under
  an existing root. `REQ-AERO-017` adds bounded retry execution through
  `SolverCommandRetryPolicy` and `execute_with_root_retrying`, recording
  failed attempt fields before the successful execution and keeping unsafe path
  failures non-retryable. `REQ-AERO-018` adds deterministic Slurm array-script
  rendering through `SolverClusterScheduleTemplate` and
  `render_cluster_schedule`, producing SHA-pinned `sbatch` scripts with
  per-case MPI ranks, CPU/task requests, quoted argv/env, and submit-command
  metadata. `REQ-AERO-019` adds `SolverClusterSchedule::submit_with_root` and
  `query_job_state_with_root`, which write/submit rendered scripts through
  direct command vectors, parse Slurm job ids, and classify scheduler state
  output with stdout/stderr digest evidence. `REQ-AERO-020` adds
  `wait_for_completion_with_root` and
  `SolverCommandPlan::collect_expected_outputs_with_root`, polling bounded
  scheduler state transitions to successful completion and verifying planned
  output artifact SHA pins under the solver root. `REQ-AERO-021` adds recorded
  OpenFOAM scalar boundary-field parsing and panel-order Cp deck baking, with
  `open_foam_boundary_cp_field` artifact labeling. `REQ-AERO-022` adds recorded
  OpenFOAM scalar/vector dictionary parsing and field-set validation, with
  `open_foam_scalar_field` and `open_foam_vector_field` artifact labels.
  `REQ-AERO-023` adds declared OpenFOAM case-root field loading for recorded
  scalar/vector dictionaries. `REQ-AERO-024` adds deterministic selected
  time-directory field discovery for supported scalar/vector dictionaries.
  `REQ-AERO-025` adds recorded `polyMesh/boundary` metadata parsing and field
  patch validation. `REQ-AERO-026` adds recorded `polyMesh`
  points/faces/owner/neighbour connectivity validation. `REQ-AERO-027` adds
  common `turbulenceProperties` model/field-set semantic validation.
  `REQ-AERO-028` adds recorded `polyMesh` point/face/cell zone topology
  validation. `REQ-AERO-029` adds recorded `polyMesh` face/cell geometry
  reconstruction. `REQ-AERO-030` adds recorded calibrated
  turbulence-coefficient validation. `REQ-AERO-031` adds calibrated runtime
  six-coefficient validation. `REQ-AERO-032` adds direct-command
  site-specific remote artifact staging with stdout/stderr digest evidence and
  SHA-verified planned output collection. This
  closes the WP-03.4 skeleton/orchestration acceptance:
  crate/layering, FC lock, condition-only compile-fail guard, run-matrix lint,
  ordered panel-sweep deck baking, and gates are covered. Built-in remote
  backends and full uncertainty database assembly remain later-tier work and
  are not claimed by this WP.
- **fidelity_tier:** T1 (orchestration substrate)
- **depends_on:** [WP-03.1]
- **new_crates:** `openbmp-aerodb` (L6) — *first commit is the skeleton +
  placement justification per `13` §5*: edges to `openbmp-aero` (L2) only;
  forbidden from the `openbmp-fc` closure; reviewed before implementation lands
- **touched:** new crate; `crates/openbmp-testkit/tests/` (compile-fail +
  scenario-lint guardrail), workspace `Cargo.toml`
- **approach:** §3.0/§3.1 `RunMatrix`, `DeckBuilder`, ordered (node-index) merge
  of the rayon sweep; §6 condition-only lock — `#[non_exhaustive] RunNode` with
  no aimpoint field + a compile-fail test (mirror
  `ballistic_state_compile_fail.rs`) + a scenario-lint that fails closed on a
  run-matrix TOML referencing target/aimpoint/range.
- **acceptance:**
  - crate compiles, layer lint green, FC tripwire green (no new fc edge)
  - compile-fail test proves no ground-aimpoint construction path exists
  - scenario-lint rejects a fixture TOML naming a target field, fail-closed
  - a deck baked from the WP-03.1 panel sweep round-trips byte-identical to a
    direct in-loop bake (no orchestration drift)
  - all §2 gates green
- **validation_label:** `checked`
- **dual_use_note:** **this WP installs the tightened lock** (§6 item 1)
- **est_effort:** 2 weeks
- **parity_ceiling:** orchestration only; no external solver yet; DB carries no σ.

---

**WP-03.5 — Vortex-lattice fin / grid-fin / hinge-moment module**
- **goal:** Add attached-flow fin loads, control-surface effectiveness, and
  hinge moments below the transonic drag rise — the gap between whole-body
  DATCOM coefficients and full Euler — feeding GNC load-relief and hinge-moment
  actuation.
- **implementation_status:** partial and traced by `REQ-AERO-034` /
  `V-AERO-034`. `openbmp-aero::vlm` now exposes the first attached-flow
  fin lifting-surface substrate: `VlmFinGeometry`, `VlmFinFlow`,
  `VlmFinSurface`, and `VlmFinSolution`; a first deterministic grid-fin
  authority substrate: `VlmGridFinGeometry`, `VlmGridFinFlow`,
  `VlmGridFinSurface`, and `VlmGridFinSolution`; plus `VlmHorseshoeLattice`,
  `VlmHorseshoePanel`, and `VlmHorseshoeSolution` for a deterministic
  horseshoe-vortex influence-assembly path. The slice validates trapezoidal fin
  geometry, grid-fin projected area/cell/open-area/web geometry, and
  subsonic/attached-flow limits, computes deterministic finite-wing lift-curve
  slope from the NASA SP-405 rectangular-planform formula family, lift,
  induced drag, pitching moment, and hinge-moment trend using fixed spanwise
  strip quadrature, ships a synthetic fixture that checks
  lift/hinge values and sign/monotonicity with control deflection, plus an
  analytic rectangular AR=4 finite-wing fixture that checks lift-curve slope,
  `CL`, induced drag, and aerodynamic-center location, plus a public NASA
  SP-405 Figure 5 rectangular-wing lift-curve-slope fixture for AR=4, evaluates
  grid-fin normal force, blockage drag, pitch moment, hinge moment, and
  effective open-area trends, assembles Biot-Savart horseshoe-vortex panel
  influences in locked panel order, solves the dense system with deterministic
  pivoting, checks flow-tangency residual plus hinge trends, and exposes
  deterministic central-difference horseshoe derivative channels for CL-alpha,
  Cm-alpha, hinge-alpha, control lift, and control hinge comparisons. A
  parser-gated derivative-comparison deck/report harness now loads tolerance
  rows for those channels and emits deterministic pass/fail reports, ready for
  future AVL/NACA tables. The grid-fin and horseshoe coefficient paths apply
  bounded subsonic compressibility
  scaling, with normal-force-derived or CL/Cm/hinge coefficients scaling as
  `1/beta` and horseshoe induced drag as `1/beta^2` in focused evidence. A
  2-by-2 horseshoe fixture pins the assembled matrix/RHS f64 bit pattern for
  operand-order drift checks. A public NACA Report 921 method fixture pins the
  quarter-chord bound-vortex and three-quarter-chord control-point convention
  on a straight-quarter-chord rectangular lattice. This is not yet calibrated
  grid-fin authority, AVL code-to-code tolerance table, public NACA tabular
  lift-curve validation, or production dense-LU/backend closure.
- **fidelity_tier:** T2
- **depends_on:** [WP-03.3]
- **new_crates:** none (lives in `openbmp-aero`, L2; driven by `openbmp-aerodb`)
- **touched:** `crates/openbmp-aero/src/` (new `vlm` module), tests,
  `data/aero/vlm-fin-lift-hinge-trend-v1.toml`,
  `data/aero/vlm-finite-wing-analytic-v1.toml`,
  `data/aero/nasa-sp405-rectangular-wing-lift-slope-v1.toml`,
  `data/aero/naca-report-921-vlm-method-v1.toml`,
  `data/aero/vlm-horseshoe-derivative-comparison-v1.toml` + provenance;
  future AVL cases remain open
- **approach:** §3.2 (M6) horseshoe VLM, Biot-Savart influence assembly, dense
  LU (`faer`/`nalgebra`) with a fixed documented pivot order, Kutta-Joukowski
  section lift, Trefftz induced drag, Prandtl-Glauert/Goethert scaling, hinge
  moment `∫ΔCp` aft of the hinge line. Validate against AVL on identical
  geometry.
- **acceptance:**
  - off by default; goldens byte-identical
  - `CLα`, `Cm` match AVL on the same geometry to ±3% (code-to-code,
    tolerance table)
  - hinge moment monotonic and signed correctly with deflection
  - dense-LU operand order pinned by a byte-diff test
  - all §2 gates green
- **validation_label:** `checked` / `validated-toy` for the analytic and
  synthetic in-tree fixtures plus public NASA SP-405 formula evidence; AVL
  code-to-code and public NACA tabular
  lift-curve closure remain required for higher-confidence labels.
- **dual_use_note:** far from line; grid-fin authority is vehicle-intrinsic
- **est_effort:** 3–5 weeks
- **parity_ceiling:** linear-potential attached flow only; no transonic shock,
  no separation; supersonic kernel is the fiddly part — validate carefully.

---

**WP-03.6 — Per-node RSS uncertainty + GCI + correlated draw**
- **goal:** Make every deck node carry a `σ_C = √(σ_WT² + σ_CFD² + σ_modelform²
  + σ_extrap²)` budget and a *correlated* (not white-noise) Monte-Carlo draw,
  so OpenBMP's existing trajectory MC runs honest aero dispersion. Highest
  value-per-effort after T1.
- **fidelity_tier:** T3
- **depends_on:** [WP-03.4, WP-11.0 (openbmp-uq promotion)]
- **new_crates:** none (consumes `openbmp-uq` L1; lives in `openbmp-aerodb` +
  the deck schema in `openbmp-aero`)
- **touched:** `crates/openbmp-aero/src/deck.rs` (SigmaTable schema fields),
  `crates/openbmp-aerodb/src/uncertainty/`, `openbmp-uq` (GCI helper), tests
- **approach:** §3.2 (M7) RSS; Roache GCI for `σ_CFD` (`GridConvergence::
  from_three_grids`); fidelity-overlap spread for `σ_modelform` (honestly
  inflated when no CFD present); Gauss-Markov / Cholesky-correlated draw over
  Mach via `DeterministicRng` with a domain-separated `(seed,step,id,component)`
  tuple. σ defaults to 0 / absent ⇒ `draw()` returns the nominal deck bit-for-bit.
- **implementation_status:** implemented. `REQ-AERO-001` provides optional row-major one-sigma
  `CN/CD/CM` tables on `AeroDeck` plus strict `[uncertainty.correlation]`
  deck-file metadata for an exponential Mach covariance length. `REQ-AERO-005`
  adds the bounded RSS/GCI substrate: `openbmp-uq::GridConvergence` computes
  observed order plus fine-grid relative/absolute GCI for monotone three-grid
  data, and
  `openbmp-aerodb::{CoefficientSigmaBudget, ReducedCoefficientSigmaBudget,
  attach_reduced_sigma_tables, attach_reduced_sigma_tables_from_gci_toml,
  attach_reduced_sigma_tables_from_gci_csv}`
  validates per-node WT/CFD/model-form/extrapolation one-sigma sources,
  optionally replaces `σ_CFD` with the absolute fine-grid GCI, ingests strict
  row-major external GCI sigma TOML/CSV, RSS-combines them, and attaches
  reduced-deck sigma tables. `REQ-AERO-006` adds `CorrelatedDeckDrawSettings` and
  `draw_reduced_correlated_deck`, using `DeterministicRng::for_mc_sample` and a
  Cholesky factor of an exponential Mach covariance to draw independent
  CN/CD/CM residual fields, defaulting to shared same-Mach alpha/beta residuals
  while optional alpha/beta correlation lengths activate separable grid
  covariance over the exact deck node order; perturbations are scaled by the
  per-node sigma. If the deck declares a reduced `CN/CD/CM`
  coefficient-correlation matrix, the draw applies it through a second
  fixed-size Cholesky mixing step. Sigma-absent decks return an exact clone.
  `REQ-AERO-010` adds the six-channel extension:
  nominal six tables are preserved during reduced draws, decks with
  `[uncertainty.six_coefficients.*]` get correlated `CX/CY/CZ/Cl/Cm/Cn`
  residual fields, independent unless deck-declared six-channel
  coefficient-correlation metadata is present, and the runner deck-load hook
  uses the generalized `draw_correlated_aero_deck` helper. `REQ-AERO-011` adds
  six-channel RSS/GCI sigma ingestion: strict
  `openbmp.aerodb_six_gci_sigma = 1` TOML and
  `openbmp.aerodb_six_gci_sigma.csv.v1` CSV rows map per-node
  `CX/CY/CZ/Cl/Cm/Cn` fine/medium/coarse studies into RSS-combined sigma
  tables for nominal six-coefficient decks. `REQ-AERO-013` adds optional
  deck-declared six-channel coefficient-correlation metadata and applies its
  Cholesky factor to the six residual streams before per-node sigma scaling.
  `REQ-AERO-007` adds schema-v3
  `[aero.deck_draw]` with campaign seed, sample index, RNG dimension base, and
  optional Mach/alpha/beta correlation lengths, validates them fail-closed, and
  has the runner apply the draw after external deck load or `[aero.buildup]`
  bake before `DeckLookup` / hybrid deck consumption; when the scenario omits
  the Mach correlation length, the runner resolves it from deck-declared
  `[uncertainty.correlation]` metadata. `REQ-AERO-008` adds `openbmp mc
  aero-deck-draw-overlays`, which writes deterministic per-sample merge overlay
  TOML files including optional alpha/beta covariance controls plus a
  campaign-manifest fragment, and `openbmp mc
  aero-deck-draw-campaign`, which emits a full generated campaign manifest and
  runs it through the native campaign executor with optional `--uq-toml`
  sidecar credibility gating and Markdown report output; the campaign path
  also gathers base-scenario `[aero.uq]` through
  `openbmp_runner::aero::upstream_uq_budget` and evaluates sidecar-plus-upstream
  credibility with the shared `CampaignCredibilityInputs` contract. Both paths
  can omit `mach_correlation_length` when a deck-declared covariance length will
  be used at runtime, and validate sample count, sample-index overflow,
  explicit Mach/alpha/beta correlation lengths, dimension-base overflow,
  deck-required RNG dimension capacity for six-sigma decks, and overlay-prefix
  path safety. The campaign evidence includes a two-sample synthetic grid-sigma
  deck run with generated alpha/beta covariance overlays, deck-declared Mach
  covariance defaulting, materialized run scenarios, and ordinary campaign
  summary artifacts. Full external-solver orchestration and solver-specific CFD
  field-output parsing remain later WP-03.8 scope.
- **acceptance:**
  - off by default; σ-absent decks draw the nominal `AeroDeck` byte-identical
    (property test); existing MC goldens unchanged
  - GCI recovers a manufactured observed order `p` to abs ≤ 0.05
  - same seed ⇒ byte-identical drawn ensemble (determinism gate extended to the
    ensemble, per doc 11)
  - a scenario runs the existing trajectory MC with aero dispersion enabled and
    reports the `ErrorBudget` credibility record
  - all §2 gates green
- **validation_label:** `validated-toy` (RSS + correlated draw); `research` for
  the GCI verification case
- **dual_use_note:** far from line; σ tables are condition-indexed
- **est_effort:** 1–2 weeks
- **parity_ceiling:** `σ_WT` is a placeholder (no facility data); `σ_modelform`
  is *inflated*, not flight-calibrated — credible-but-uncalibrated UQ.

---

**WP-03.7 — Stage-separation / proximity interference deck**
- **goal:** Make OpenBMP's existing staging events carry real aerodynamic
  proximity loads (shock impingement, blockage) instead of instantaneous mass
  drops, by tabulating 6-DOF interference increments over relative geometry and
  superposing them on each body's isolated deck during dual-body integration.
- **fidelity_tier:** T3 (consumer side; CFD case matrix is WP-03.8's job)
- **depends_on:** [WP-03.1, WP-03.4]
- **new_crates:** none (`openbmp-aerodb` `derived/` + a runtime adapter in the
  runner that owns staging)
- **touched:** `crates/openbmp-aerodb/src/derived/stagesep.rs`, the runner's
  staging path, tests, `data/aero/<sep-cases>/` + provenance
- **approach:** §3.2 (M8) parametric grid over `(Δx,Δy,Δz,Δθ,Δψ,M,α)`;
  per-body increment `ΔC_body2 = C_proximity − C_isolated`; sparse/structured
  sampling (5–6 D); runtime multi-D interp + superposition + dual-body 6-DOF
  until interference → 0. Increments may be in-Rust panel-derived (T3) or
  ingested Euler (T4).
- **acceptance:**
  - off by default; staging goldens byte-identical until a scenario opts in
  - interference → 0 as `Δx → ∞` (limit test)
  - a two-stage scenario shows physically reasonable, bounded separation loads;
    dual-body 6-DOF conserves total momentum to documented tolerance
  - all §2 gates green
- **validation_label:** `validated-toy` (in-Rust increments); `research` if
  paired with ingested Ares-I-class published methodology comparison
- **dual_use_note:** dual-use-neutral (body-on-body interference, not dispense);
  condition-parameterized
- **est_effort:** 3–4 weeks
- **parity_ceiling:** high-D table needs structured sampling; plume-free
  aerodynamic interference only (no hot-plume coupling — §1.2 item 2).

---

**WP-03.8 — CAPE-style Euler coupling: ingest Cart3D / SU2-Euler**
- **goal:** Replace the `σ_modelform` placeholder with *measured* nonlinear
  inviscid loads by building the out-of-process run-matrix orchestrator that
  drives (or ingests) Cart3D/SU2-Euler and assembles decks + line loads + GCI.
  Couple, do **not** port (`00` §1.1, `13` §6).
- **fidelity_tier:** T4
- **depends_on:** [WP-03.4, WP-03.6]
- **new_crates:** none (`openbmp-aerodb` `ingest/` + `orchestrate/`)
- **touched:** `crates/openbmp-aerodb/src/{ingest,orchestrate}/`, tests,
  `data/aero/cfd/<case>/` + `provenance.md` (tool, version, SHA, grids, label)
- **approach:** `SolverIngest` adapters (`Cart3DTri`, `Su2History`) parse
  forces/breakdown + surface-Cp; `SolverDriver` subprocess runner (gated, the
  *user* provisions the solver); integrate Cp over each body's panels for line
  loads; 3-grid Roache GCI per coefficient → `σ_CFD`. SU2 (LGPL) is the open
  default; Cart3D only where the user holds a Govt-purpose release.
- **acceptance:**
  - ingestion is offline/optional; no solver is a build or default dependency
  - parsing a pinned SU2-Euler fixture round-trips `CA/CN/Cm` + Cp to bit
    precision against a recorded golden; SHA-256 provenance verified fail-closed
  - in-repo panel `CA/CN/Cm` agrees with SU2-Euler on a shared open benchmark
    geometry within a documented band (code-to-code, tolerance table) and the
    "what is not validated" note is written
  - GCI study produces an asymptotic-range observed order; σ_CFD populated
  - all §2 gates green
- **validation_label:** `research` (code-to-code + a §5.2 public benchmark + UQ)
- **dual_use_note:** general-purpose CFD; guardrail in the consumption layer
  (condition-indexed decks); controlled solvers never vendored/redistributed
- **est_effort:** 4–8 weeks (orchestrator; zero for the solver)
- **parity_ceiling:** inviscid Euler — no boundary layer, no base/separation
  physics (that is WP-03.9); no flight correlation.

---

**WP-03.9 — Viscous + rarefied coupling: RANS (SU2/OpenFOAM) + SPARTA DSMC**
- **goal:** Add viscous-corrected base/separated/buffet loads (RANS) and
  physically-correct transitional aero (DSMC) that replaces the interpolated
  Knudsen bridge with sampled SPARTA surface tractions — closing the
  multi-fidelity DB (DSMC → bridge → RANS/Euler → DATCOM/VLM), each region
  tagged with method + uncertainty.
- **fidelity_tier:** T5
- **depends_on:** [WP-03.8]
- **new_crates:** none (`openbmp-aerodb` `ingest/` adapters)
- **touched:** `crates/openbmp-aerodb/src/ingest/{openfoam,su2_rans,sparta}.rs`,
  `crates/openbmp-aero/src/knudsen.rs` (bridge calibration), tests,
  `data/aero/{rans,dsmc}/<case>/` + provenance
- **approach:** `Su2History`(SA/SST)/`OpenFOAMForce` ingestion for viscous
  forces/Cp/line loads; `SpartaSurf` ingestion of sampled surface
  pressure/shear/heat-flux above continuum breakdown; calibrate/replace
  `LinearKnudsenBridge` against the DSMC transitional points. Process-boundary
  coupling only for the GPL solvers (run binary, read files).
- **acceptance:**
  - ingestion offline/optional; GPL solvers never linked/vendored (process
    boundary only); provenance + SHA verified fail-closed
  - ONERA M6 surface Cp (7 stations) ingested and within the published band;
    NASA TMR SA/SST flat-plate/bump verification reproduced as code-to-code
  - SPARTA-calibrated bridge matches sampled transitional `CD` better than the
    pre-calibration linear bridge on a documented case (tolerance table)
  - method_map tags each DB region with tier + label; all §2 gates green
- **validation_label:** `research` (ONERA M6 + TMR + code-to-code + UQ)
- **dual_use_note:** general-purpose solvers; consumption-layer guardrail;
  never vendor/redistribute
- **est_effort:** 6–10 weeks (ingestion + calibration; zero for solvers)
- **parity_ceiling:** turbulence-model validity is the external solver's
  burden; base/plume reacting-flow coupling remains out of reach (§1.2 item 2);
  no flight correlation — research deck, not certified databook.

---

## 10. References

**Local-inclination / hypersonic panel methods**
- Anderson, *Hypersonic and High-Temperature Gas Dynamics*, 3rd ed., AIAA 2019,
  ch.3 (Newtonian) & ch.14–16.
- Bertin, *Hypersonic Aerothermodynamics*, AIAA 1994.
- Gentry, Smyth, Oliver, "The Mark IV Supersonic-Hypersonic Arbitrary-Body
  Program," AFFDL-TR-73-159, 1973.
- Kinney, "Aerodynamic Shape Optimization of Hypersonic Vehicles" (CBAERO),
  AIAA 2006; NASA Software ARC-15819-1.

**Component buildup (DATCOM-class)**
- Blake, "Missile DATCOM User's Manual — 2014 Revision," AFRL-RQ-WP-TR-2014-0281
  (manual publicly readable: DTIC AD1000581).
- Allen & Perkins, "A Study of Effects of Viscosity on Flow over Slender
  Inclined Bodies of Revolution," NACA Report 1048, 1951.
- Hoerner, *Fluid-Dynamic Drag* (1965) & *Fluid-Dynamic Lift* (1985).
- Nielsen, *Missile Aerodynamics*, AIAA 1988.

**Vortex-lattice / panel (subsonic & linearized-supersonic)**
- Katz & Plotkin, *Low-Speed Aerodynamics*, 2nd ed., Cambridge 2001, ch.12–13.
- Drela & Youngren, AVL 3.36 (Athena Vortex Lattice), MIT.
- NASA SP-405, *Vortex-Lattice Utilization*, 1976.
- Bertin & Cummings, *Aerodynamics for Engineers*, 6th ed.
- NASA OpenVSP / VSPAERO.

**Cartesian cut-cell Euler & automated DB generation**
- Aftosmis, Berger, Melton, "Robust and Efficient Cartesian Mesh Generation for
  Component-Based Geometry," AIAA J. 36(6), 1998.
- Nemec & Aftosmis, "Adjoint Error Estimation and Adaptive Refinement for
  Embedded-Boundary Cartesian Meshes," AIAA 2007-4187.
- Murman, Aftosmis, Nemec, "Automated Parameter Studies Using a Cartesian
  Method," AIAA 2004-5076.
- Cart3D, NASA Ames (US-Govt-purpose release).

**RANS / turbulence models**
- Spalart & Allmaras, "A One-Equation Turbulence Model for Aerodynamic Flows,"
  AIAA 92-0439 / *Rech. Aérosp.* 1994.
- Menter, "Two-Equation Eddy-Viscosity Turbulence Models for Engineering
  Applications," AIAA J. 32(8), 1994.
- Economon et al., "SU2: An Open-Source Suite for Multiphysics Simulation and
  Design," AIAA J. 54(3), 2016 (LGPL 2.1).
- NASA Turbulence Modeling Resource (TMR), tmbwg.github.io/turbmodels.

**DSMC / rarefied**
- Bird, *Molecular Gas Dynamics and the Direct Simulation of Gas Flows*, Oxford
  1994 (NTC, VHS).
- Plimpton & Gallis, "SPARTA Direct Simulation Monte Carlo Simulator," Sandia
  (GPL).
- Boyd & Schwartzentruber, *Nonequilibrium Gas Dynamics and Molecular
  Simulation*, Cambridge 2017.
- Schaaf & Chambré, free-molecular aerodynamics (already in `knudsen.rs`).

**Uncertainty quantification**
- Roache, *Verification and Validation in Computational Science and
  Engineering*, Hermosa 1998 (GCI).
- ASME V&V 20-2009, "Standard for Verification and Validation in CFD and Heat
  Transfer."
- Hemsch & Walker, "Statistical Analysis of Aerodynamic Database Uncertainty"
  (NASA approach).
- AIAA G-077-1998, "Guide for the Verification and Validation of CFD
  Simulations."

**Stage separation**
- Pamadi et al., "Aerodynamic Analyses and Database Development for Ares I Stage
  Separation," NASA/AIAA.
- Tartabini et al., "Ares I-X Stage Separation," NASA Langley.

**V&V benchmarks**
- Schmitt & Charpin, ONERA M6 wing, AGARD AR-138 (1979), Test 2308.
- HB-1 / HB-2 hypersonic standard ballistic models (AEDC-VKF / ONERA / NASA Ames
  inter-facility correlation).
- AGARD-B standard model (supersonic stability-derivative inter-facility
  dataset).
- Apollo AS-202 / Stardust SRC entry (NASA NTRS 20060053240; already pinned in
  OpenBMP).

**OpenBMP anchors**
- `docs/verification.md` (Continuum Aerodynamics Buildup; Hypersonic V&V & UQ
  Ladder; NASA-STD-7009B & AIAA G-077 vocabulary).
- `docs/hypersonic-extensions.md` (AeroMethod family, Knudsen bridging,
  free-molecular).
- `docs/software-architecture.md` (deck format, layer map, determinism rules).
- `docs/external-telemetry-validation.md` (LOCAL-only dispersion cross-check).
- `docs/safety-boundaries.md` (governance for couplings and controlled tools).
