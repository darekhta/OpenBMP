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
- `PanelMesh` (`:67-148`): a validated watertight-ish triangulation
  (finite vertices, in-range indices, non-degenerate area). **It does not
  verify closure (no manifold/watertight check) and has no BVH/spatial index.**
- `LocalInclinationPanels` (`:167-253`): the only mesh integrator. Iterates
  triangles, `cos θ = n̂·û_upstream`; **convex sign-test shadowing only**
  (`:224` `if self.shadowing && cos_theta <= 0.0 { continue }`) — no ray-cast,
  so concave bodies self-shadow incorrectly. Modified-Newtonian `Cp` only
  (`PanelMethod` enum has one variant, `:151-155`). **No skin friction**, no
  per-station line-load binning, moment is about the body origin only
  (`:233-235`, centroid × panel force), not an arbitrary CG.

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
- **Missing vs DATCOM:** Allen-Perkins nonlinear crossflow `CN`,
  slender-body fin-body/body-fin carryover `K_W(B)`/`K_B(W)`, area-rule wave
  drag, protuberance drag, and **all dynamic derivatives** (`Cm_q`, `Cn_r`,
  `Cl_p`, etc.). `CM` is a static nose+fin moment sum (`:339-353`).

### 2.3 Deck, method seam, rarefied — `src/deck.rs` / `method.rs` / `knudsen.rs`

- `AeroDeck` (`deck.rs:127`): N-D (3≤N≤6) tabulated deck on
  `axis_order = ["mach","alpha","beta", …effectors]`, row-major flat tables,
  **locked-order innermost-first multilinear reduction** bit-identical to the
  Schema-1 trilinear at N=3 (Demmel & Nguyen 2020; FMA disabled; `deck.rs:40-69`).
  Returns reduced `(CN, CD, CM)` only — **no CY/CZ/Cl/Cn, no surface pressures,
  no line loads, no per-entry σ**. `ExtrapolationPolicy::{FailClosed(default),
  Clamp}`; linear extrapolation deliberately absent (`:104-115`).
- `AeroMethod`/`DeckLookup` (`method.rs:91-198`): the pluggable seam. Body
  mapping `F = q·S·(-CD,0,-CN)`, `M = q·S·L·(0,CM,0)`, locked operand order.
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
  `openbmp.panel_mesh_aero = 1` (panel mesh). **No STL ingest, no CFD-output
  parser, no run-matrix schema, no σ fields.**
- UQ exists but in the wrong crate for aero: `openbmp-physics/src/uq.rs`
  (`ErrorBudget`, `UncertaintyContribution`, `aggregate_one_sigma() =
  sqrt(Σσᵢ²)+bias`, `render_markdown`). `00-overview.md` already plans to
  promote it to `openbmp-uq` (L1, doc 11). **There is no GCI helper and no
  per-deck-node σ table anywhere.**
- Existing ingestion precedent: `data/aero/calisto-drag.{toml,csv}` with a
  SHA-256 pin verified in `tests/calisto_deck_pin.rs` against the upstream
  RocketPy CSV — the pattern every CFD ingestion WP follows.

### 2.5 Validation baseline (`docs/verification.md`)

The "Continuum Aerodynamics Buildup" section verifies skin friction vs
Blasius/Schlichting, base drag vs the reference formula, boattail reduction,
the drag polar, and bit-identical deck baking. The "Hypersonic V&V and UQ
Ladder" defines the 5-layer evidence chain this doc's tiers earn against. **No
HB-2, ONERA M6, AGARD-B, or GCI case exists yet.**

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
   a CFD subprocess, an STL parser, or rayon — it consumes only the baked
   `AeroDeck` through the existing sensor/aero seam. No new `openbmp-fc` edge.
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
  (L1); the `ErrorBudget`/GCI helper and the correlated-draw consumer live
  there. The per-node σ tables feed the MC campaign orchestrator (`openbmp-mc`,
  L7). **Hard dependency for T3.**
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
- **fidelity_tier:** T1
- **depends_on:** []
- **new_crates:** none (extends `openbmp-aero`, L2)
- **touched:** `crates/openbmp-aero/src/hypersonic.rs` (PanelMethod enum,
  ShadowingMesh, PanelSolution), `src/lib.rs` (re-exports),
  `tests/`, `data/aero/<mesh>/` + `provenance.md`
- **approach:** §3.2 (M1) per-panel Newtonian/tangent-cone/wedge Cp with
  `sinθ = max(0, −n̂·v̂_∞)`; BVH (median-split AABB, byte-stable build) for
  concave shadow ray-casts (sign test for convex); CG-referenced moment
  `(rᵢ−r_cg)×F`; axial binning into `stations` for `AxialLineLoad`. STL/tri
  ingest via a new strict parser (extend `panel_mesh_aero` marker). rayon sweep
  over a `RunMatrix` (added here as a thin local type, promoted to aerodb in
  WP-03.4).
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
  validated on synthetic concave bodies, not flight geometry; no WT correlation.

---

**WP-03.2 — Reference-temperature skin friction on the panel solver**
- **goal:** Recover the viscous axial force the inviscid panel misses, so the
  T1 deck's total `CA` is credible across the supersonic/hypersonic envelope.
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
  - viscous + inviscid `CA` on a sharp cone brackets the published HB-2 viscous
    correction trend; envelope documented
  - all §2 gates green
- **validation_label:** `checked` (analytic Cf); `validated-toy` (combined CA trend)
- **dual_use_note:** far from line
- **est_effort:** 1 week
- **parity_ceiling:** flat-plate reference-enthalpy approximation; no transition
  prediction beyond a fixed `Re_crit`; no separation.

---

**WP-03.3 — Close ComponentBuildup to DATCOM level**
- **goal:** Bring the 0.1<M<8 deck from "drag + linear CN" to DATCOM-class:
  Allen-Perkins nonlinear crossflow `CN`, slender-body carryover K-factors,
  area-rule wave drag, protuberance drag, and dynamic derivatives.
- **fidelity_tier:** T2
- **depends_on:** [WP-03.1]
- **new_crates:** none
- **touched:** `crates/openbmp-aero/src/buildup.rs`, deck schema (6-coeff +
  derivatives), tests, `data/aero/<datcom-cases>/` + provenance
- **approach:** §3.2 (M3) Allen-Perkins crossflow; (M4) Sears-Haack/area-rule
  wave drag replacing `forebody_pressure_wave_drag`; (M5) DATCOM fin slope +
  `K_W(B)`/`K_B(W)` carryover replacing the crude `1+d/(2s+d)` interference;
  protuberance increments; dynamic derivatives (`Cm_q`, `Cn_r`, `Cl_p`) via the
  documented DATCOM correlations. Clamp/blend transonic regime carefully.
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

---

**WP-03.4 — `openbmp-aerodb` skeleton: RunMatrix + DeckBuilder + dual-use lock**
- **goal:** Stand up the L6 orchestration crate with the condition-only
  `RunMatrix`/`RunNode` DOE, the `DeckBuilder` that assembles in-Rust sweep
  results into an `AeroDeck`, and the **compile-fail dual-use lock** proving the
  matrix cannot express a ground aimpoint.
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
- **fidelity_tier:** T2
- **depends_on:** [WP-03.3]
- **new_crates:** none (lives in `openbmp-aero`, L2; driven by `openbmp-aerodb`)
- **touched:** `crates/openbmp-aero/src/` (new `vlm` module), tests,
  `data/aero/<avl-cases>/` + provenance
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
- **validation_label:** `validated-toy` (AVL code-to-code; `research` once paired
  with a public NACA lift-curve benchmark)
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
