# Aerothermodynamics, Real-Gas Chemistry & TPS

**Status:** `experimental` (design intent; this document ships no code).
**Audience:** the GNC + Rust engineer or LLM agent implementing the parity work
packages for the aerothermal / real-gas / thermal-protection dimension.
**One-line summary:** lift OpenBMP's stagnation-heating scaffolds and ablation
toy to a verified engineering-correlation aerothermal deck — Tauber-Sutton
radiative + Srinivasan-Tannehill equilibrium-air edge state + Eckert/Van Driest
distributed deck + a 1-D implicit FIAT/CHAR-class material-response solver on
**generic TACOT/textbook materials only** — validated against open flight
benchmarks (Stardust, FIRE II) and open-code oracles (CEA, FIAT/PATO), with
Park two-temperature nonequilibrium held at the `research` ceiling.

> Read `00-overview.md` (parity definition, invariants, solver-consumer posture,
> DAG) and `13-agent-execution-playbook.md` (the template this doc follows, the
> WP backlog schema, the gate set) before this document. This doc pairs tightly
> with `03-aerodynamics-database-and-cfd-coupling.md`: the aero deck supplies the
> edge flow conditions (M, α, β, line loads, pressure distribution) that the
> distributed heating deck integrates over, and both consume the same
> environment (`08-environment-gravity-and-frames.md`) and feed the same UQ /
> Monte Carlo machinery (`11-monte-carlo-uq-and-validation.md`).

---

## 1. Parity target & ceiling

### 1.1 Target capability (within posture)

The achievable end-state — **capability/method parity, not validation parity**
(`00` §1) — is a deterministic, in-repo aerothermal pipeline that:

1. computes **total stagnation-point heating** `q_w = q_conv + q_rad` with a
   real-gas convective term (Fay-Riddell fed by an equilibrium-air edge state)
   and a published-coefficient radiative term (Tauber-Sutton), not a tuned
   power law;
2. produces a **distributed body heating deck** `q_w(s)` over arc length with
   Eckert reference-enthalpy laminar heating and Van Driest II turbulent
   heating, a verified compressible skin-friction transform, the
   cone↔flat-plate Mangler correction, and engineering boundary-layer
   transition (Re_θ/M_e + PANT/Reda) switching laminar→turbulent;
3. supplies a **real-gas equilibrium-air edge state** from Srinivasan-Tannehill
   curve fits inline (deterministic, no iteration), with NASA CEA / Cantera
   used **offline** to generate and validate the fitted tables;
4. resolves **in-depth charring-ablator material response** with a 1-D implicit
   conduction + multi-component Arrhenius decomposition + Darcy pyrolysis-gas +
   B′-table-coupled surface energy balance solver, replacing the current
   sharp-front energy-limited toy;
5. closes the **TPS-sizing loop** — given a trajectory heat-flux history, find
   the minimum virgin thickness keeping a bondline node below an allowable
   temperature, with documented margin multipliers; and
6. carries **per-quantity uncertainty** (heating-augmentation factor, recession
   margin, material-property dispersion, model-form spread) into the Monte
   Carlo / loads pipeline, reported in a NASA-STD-7009B-shaped credibility
   record.

Park two-temperature **nonequilibrium** chemistry (5-/11-species, two-temperature
finite-rate, electron-number-density / radio-blackout) is the research frontier:
a 0-D/1-D post-shock relaxation ODE, **never** multidimensional nonequilibrium
Navier-Stokes — that is coupled externally (SU2-NEMO / SPARTA), not ported.

### 1.2 Parity ceiling (honest)

OpenBMP cannot reach SpaceX/NASA/JPL **validation parity** on three axes; the
credible open substitute for each is code-to-code + open-flight-data validation,
**not** certification:

1. **Proprietary / export-controlled material property cards.** Real PICA,
   AVCOAT, SLA-561V, and carbon-phenolic decomposition kinetics, B′ tables, and
   tested heats of ablation are ITAR/proprietary. OpenBMP **cannot and will not**
   reproduce a specific flight heat shield's margins. *Substitute:* TACOT
   (Theoretical Ablative Composite for Open Testing) and generic textbook
   archetypes only, validated by FIAT/CHAR/PATO code-to-code on those same open
   materials. The existing lock (`ablation.rs`: "No real fielded TPS material
   parameters ship") is preserved and hardened.
2. **High-fidelity CFD-aero database + arc-jet/shock-tunnel test campaigns.**
   Production programs anchor engineering correlations to thousands of
   DPLR/LAURA/US3D solutions and to facility tests (NASA Ames arc jets, CUBRC
   LENS); there is no open equivalent dataset and no test rig. *Substitute:*
   couple to open CFD (SU2-NEMO), DSMC (SPARTA), and the few open flight cases
   (FIRE II, RAM-C II, Stardust); **quantify** uncertainty rather than claim
   accuracy.
3. **Flight heritage & certification.** Margin policies, reconstructed flight
   thermocouple data, and qualification are program-internal. *Substitute:*
   published flight reconstructions (FIRE II, Stardust, Apollo 4) for external
   validity, MMS for code verification, explicit UQ.

**Net:** OpenBMP can credibly reach *"verified engineering-correlation
aerothermal + 1-D implicit material response, validated against open flight
benchmarks and open-code oracles."* That is genuinely useful for trajectory and
TPS trade studies. It is **not** flight-certification-grade heat-shield sizing
for a specific real vehicle, and this document does not imply otherwise
(`docs/standards-posture.md`, `docs/safety-boundaries.md`). Two items have
explicitly **uncertain convergence** and stay at `research`: the Park
two-temperature source model (data-hungry, stiff) and the absolute level of
radiative heating above ~12 km/s (large code-to-code spread even among
production tools on FIRE II).

---

## 2. Current state in source

Verified by reading the cited files (this is the baseline to regress against;
all existing behavior must remain byte-identical until a scenario opts in).

### 2.1 `crates/openbmp-aerothermal/`

| File | What ships today | Maturity |
|---|---|---|
| `src/stagnation.rs` | `SuttonGraves` (`q = K·√(ρ/R_n)·V³`, `K_earth = 1.7415e-4`, hot-wall correction `max(0,1−h_w/h_aw)`); `allen_eggers_heating` closed-form peak/load over an exponential atmosphere; `FayRiddell` with a `HeatTransferModel` cold-gas trait path (perfect-gas γ=1.4 Rankine-Hugoniot post-shock edge, Sutherland μ) **and** a `stagnation_from_edge_state(&FayRiddellEdgeState, WallCatalysis)` real-gas coupling seam; `with_neutral_composition_enthalpy` h_D helper; `TauberSuttonRadiative` that **refuses** (`OutOfEnvelope`, "deferred pending published coefficients"). | `SuttonGraves` validated vs Stardust (table 20) — `validated-toy`. Fay-Riddell trait path is a cold-gas **scaffold**; the edge-state seam is real-gas-ready but unfed. Tauber-Sutton is a typed-reserved stub. |
| `src/boundary_layer.rs` | `BoundaryLayer` trait + `BoundaryLayerState{Laminar,Transitional,Turbulent}`; transition models `EmpiricalTransition` (Re_x_crit), `ReThetaTransition` (Re_θ/M_e), `EnTransition` (e^N placeholder → falls back to empirical); `ReferenceEnthalpyHeating` with Eckert h* and laminar `Cf=0.664/√Re`, turbulent `Cf=0.0592·Re^−0.2`, Reynolds-analogy St. | `experimental`. Cold-gas h* (`c_p·T`), **no** Van Driest II implicit transform, **no** Mangler cone correction, transition constants are defaults without provenance, no flat-plate/cone benchmark. |
| `src/ablation.rs` | `ToyAblator` (graphite-toy, generic-charring-1 — generic textbook only); `BlowingCorrelation{FixedLambda,Lees,SpaldingChi}` (constant λ); `SteadyStateAblator` (SEB with re-radiation, no conduction, `ṁ=net/h_v`); `CharringAblator` (constant-progress proxy → delegates to steady-state); `DepthResolvedCharringAblator` — **energy-limited sharp-front toy**: `v_f = q_net/(ρ_v·h_pyro)`, projected onto fixed depth nodes. | `experimental`. The sharp-front toy is **not** a conservation-law solver: no in-depth conduction, no Arrhenius decomposition kinetics, no Darcy gas momentum, no B′-table SEB coupling. This is the largest gap vs. production TPS sizing. |
| `src/thermal_toy.rs` | `OneDThermalToy` — explicit-FTCS 1-D conduction with `ToyMaterial`, `BackwallCondition{Adiabatic,PrescribedTemperature,Convective}`, `max_stable_dt_s` CFL guard. | `validated-toy` (explicit-FD; the low-fidelity rung for the sizing loop). |
| `Cargo.toml` | Depends only on `openbmp-physics` + `thiserror`. L2 placement, no up-layer edges. | Clean. |

### 2.2 `crates/openbmp-physics/src/realgas/`

This is the real-gas backbone the heating models couple to — already scaffolded:

| Item | State |
|---|---|
| `realgas/mod.rs` — `EquilibriumAir` trait (`composition`, `gamma_eff`, `speed_of_sound_m_s`, `state` → `EquilibriumAirState`), `AirComposition` (11 species + electrons, `validate_mole_fractions`, `validate_charge_neutrality`, `neutral_formation_enthalpy_j_kg`, `neutral_mean_molar_mass_kg_mol`); pinned JANAF/NIST formation enthalpies for N, O, NO. | Trait + composition algebra **shipped**; `neutral_formation_enthalpy_j_kg` already feeds Fay-Riddell's h_D helper. |
| `TannehillEquilibriumAir` (`realgas/tannehill.rs`) | **Typed-reserved**: refuses (`OutOfEnvelope`, "deferred pending verified public table values"). An earlier synthesized table was removed in audit. This is the seam the Srinivasan-Tannehill curve fits land in. |
| `MugalevEquilibriumAir` (11-species) | Typed-reserved; `validate_query` envelope present, evaluation deferred. |
| `ParkTwoTemperatureModel` / `NonequilibriumAir` (`realgas/park_2t.rs`) | Typed API surface; **fails closed** for every reaction-set query. Pins the **public forward** Park87/Park93 neutral-subset Arrhenius rows (from Zhang et al. 2022, Table 2) and Millikan-White molecular constants in `PARK_2T_REFERENCE_PAYLOAD_V1` with a SHA-256 pin and `EnvelopeBounds`/`ProvenanceBlock` provenance. Backward/equilibrium-constant path, Park90, the high-T relaxation limiter, and the species-pair vibrational constants remain deferred. |
| `external_reference.rs` — `ExternalReferencePackage{kind, envelope, provenance}`, `ProvenanceBlock{content_hash_sha256_hex}`, `ReferencePackageKind`, `EnvelopeBounds`, `query_within_envelope`, `validate_payload_hash_hex`. | The **ingestion + provenance + envelope** primitive this doc reuses for CEA/FIAT/PATO/NEQAIR decks. Do not reinvent it. |

### 2.3 Public benchmarks already pinned

`crates/openbmp-physics/src/reentry.rs` ships Stardust SRC tables 13/19/20
(entry interface, trajectory input/output, peak convective heating, heat load)
with provenance; `stagnation.rs` tests already bracket the Stardust table-20
peak flux/altitude/load via Sutton-Graves + Allen-Eggers. These are the existing
public-flight anchors to extend.

### 2.4 Design context

`docs/hypersonic-extensions.md` defines the extension points (Earth-atmosphere
only; `openbmp-aerothermal` owns heating/BL/thermal-response/ablation; offline
high-fidelity packages are consumed, not solved). `docs/verification.md`
§"Hypersonic V&V and UQ Ladder" defines the five-layer evidence ladder and the
`research` promotion bar (≥1 code-verification case + ≥1 public reference +
declared envelope + UQ story). This doc's V&V plan (§5) instantiates that ladder.

---

## 3. Target architecture

### 3.1 Crate map & placement

No new crate is strictly required for the heating decks: the work lands in the
existing L2 `openbmp-aerothermal` (heating, BL, material response) and L2
`openbmp-physics::realgas` (equilibrium-air fits, Park 2T). One **new crate is
proposed** to keep the equilibrium-air + thermochemistry curve-fit machinery and
its CEA/Cantera-ingested tables from bloating `openbmp-physics`, matching the
`00` §4 plan:

```text
openbmp-thermochem   L2   equilibrium-air curve fits (Srinivasan-Tannehill) +
                          CEA/Cantera deck ingestion + B'-table objects
                          (shared with doc 05 propellant thermochemistry)
```

- **Placement justification.** `openbmp-thermochem` sits at L2 alongside
  `openbmp-physics`/`openbmp-aero`/`openbmp-propulsion`. It depends **down** on
  `openbmp-core` (RNG/units) and `openbmp-physics` (`EquilibriumAir` trait,
  `AirComposition`, `external_reference`). `openbmp-aerothermal` depends on it.
  It introduces **no** up-layer edge and is forbidden from any edge to
  `openbmp-fc` (the FC portability lock already forbids `openbmp-aerothermal`;
  `openbmp-thermochem` inherits that posture — heating/thermochem never enters
  the flight-software boundary). If the agent finds the curve-fit module is
  small enough, it may instead land it as `openbmp-physics::realgas::tannehill`
  fully realized; the crate-vs-module boundary is decided in `WP-04.2-a` before
  implementation.

Touched existing crates: `openbmp-aerothermal` (all four source files),
`openbmp-physics::realgas` (Tannehill fits, Park 2T backward rates),
`openbmp-runner` (deck wiring, telemetry channels), `openbmp-uq` /
`openbmp-mc` (heating-augmentation + recession-margin draws — doc 11),
`openbmp-aerodb` (consumes the same run-matrix; doc 03).

### 3.2 Trait surfaces (Rust)

The unifying principle: **OpenBMP owns the engineering-correlation tiers and the
trait surfaces; external high-fidelity physics enters as provenance-pinned,
envelope-bounded, UQ-tagged data through those same traits** (`00` §1.1, `13`
§6). The existing `HeatTransferModel`, `BoundaryLayer`, `EquilibriumAir`,
`AblationModel`, and `NonequilibriumAir` traits are kept; the new surfaces below
extend them.

#### 3.2.1 Radiative stagnation heating (Tauber-Sutton)

Replaces the refusing `TauberSuttonRadiative` stub. The `f(V)` velocity function
is a **tabulated** ingest, never a power law.

```rust
/// Published Tauber-Sutton coefficients + tabulated velocity function,
/// ingested with provenance. Earth-air and (reserved) Mars-CO2 sets.
pub struct TauberSuttonRadiative {
    /// Provenance-pinned coefficient + f(V) table package.
    package: TauberSuttonPackage,
    /// Planetary body selector (Earth shipped; Mars reserved).
    body: RadiativeBody,
}

/// Provenance-pinned Tauber-Sutton data (C, a-form, b, tabulated f(V)).
pub struct TauberSuttonPackage {
    /// q_rad = C · R_n^a · rho_inf^b · f(V).  `a` is itself a function of
    /// (V, rho_inf) per the 1991 paper; preserve that, do NOT collapse to
    /// a constant exponent (the subtlety an honest port must keep).
    c_si: f64,
    b_exponent: f64,
    /// Monotone-clamped 1-D table of f over velocity (m/s); interpolated.
    f_of_v_table: VelocityTable,
    /// Radius-exponent law a(V, rho) per Tauber-Sutton (1991), Eq. set.
    radius_exponent: RadiusExponentLaw,
    /// SHA-256-pinned source record (data/aerothermal/provenance.md).
    provenance: ProvenanceBlock,
    /// Validity window: e.g. ~3..16 km/s Earth; refuse outside.
    envelope: EnvelopeBounds,
}

impl HeatTransferModel for TauberSuttonRadiative {
    /// Returns q_rad in StagnationHeating::q_rad_w_m2; q_conv = 0 here
    /// (the caller sums Fay-Riddell q_conv + Tauber-Sutton q_rad).
    fn stagnation(&self, ctx: &AerothermalContext)
        -> Result<StagnationHeating, AerothermalError>;
}

/// Total stagnation heating: q_conv (Fay-Riddell, real-gas edge) + q_rad.
pub struct TotalStagnationHeating<'a, C: HeatTransferModel, R: HeatTransferModel> {
    pub convective: &'a C,
    pub radiative: &'a R,
}
```

#### 3.2.2 Equilibrium-air edge-state assembler

Bridges the (already shipped) `EquilibriumAir` trait to the (already shipped)
`FayRiddellEdgeState` seam, computing the post-shock + boundary-layer-edge real
gas state. This is the wire that turns the cold-gas Fay-Riddell into a real-gas
one without changing `FayRiddell`'s public API.

```rust
/// Builds a real-gas Fay-Riddell edge state from freestream + an
/// EquilibriumAir model. Solves the equilibrium normal-shock jump
/// (Rankine-Hugoniot with real-gas h(T,p)) then the stagnation
/// post-shock isentropic compression to the BL edge.
pub struct EquilibriumEdgeAssembler<'a, E: openbmp_physics::EquilibriumAir> {
    pub air: &'a E,
    /// Wall-property model: rho_w, mu_w at (T_w, p_e) from the same air.
    pub wall: WallThermoModel,
    /// Transport (mu_e) source: Gupta-Yos / Blottner inline fit, or
    /// ingested table.
    pub transport: TransportModel,
}

impl<'a, E: openbmp_physics::EquilibriumAir> EquilibriumEdgeAssembler<'a, E> {
    /// Solve the equilibrium normal shock + stagnation compression and
    /// emit the edge state Fay-Riddell consumes. Fails closed when the
    /// air model is out of envelope (the assembler never silently falls
    /// back to perfect gas).
    pub fn edge_state(&self, ctx: &AerothermalContext)
        -> Result<FayRiddellEdgeState, AerothermalError>;
}
```

#### 3.2.3 Distributed heating deck (Eckert + Van Driest II + Mangler)

Extends `ReferenceEnthalpyHeating` to a verified deck producer over a body
station list.

```rust
/// Van Driest II compressible turbulent skin-friction transform.
/// Implicit: Newton-solve C_f from the transcendental relation
///   0.242/(sqrt(C_f)·A)·[asin((2A^2−B)/Q)+asin(B/Q)]
///     = 0.41 + log10(Re_x·C_f) − omega·log10(T_w/T_e)
/// with F_c = (h_aw/h_e − 1)/(asin(alpha)+asin(beta))^2.
pub struct VanDriestII {
    pub omega: f64,        // viscosity-temperature exponent
    pub max_newton_iters: u32,
    pub newton_tol: f64,
}

impl VanDriestII {
    /// Deterministic Newton iteration with a fixed iteration cap and a
    /// locked operand order; returns OutOfEnvelope if it does not
    /// converge within the cap (never panics, never NaNs).
    pub fn cf_incompressible_equivalent(&self, re_x: f64, t_ratio: f64)
        -> Result<f64, AerothermalError>;
}

/// A body discretized into arc-length stations producing q_w(s).
pub struct DistributedHeatingDeck {
    /// Ordered body stations (arc length from stagnation, local R, etc.).
    pub stations: Vec<BodyStation>,
    /// Laminar method (Eckert reference enthalpy).
    pub laminar: ReferenceEnthalpyHeating,
    /// Turbulent method (Van Driest II reference-enthalpy heating).
    pub turbulent: VanDriestIIHeating,
    /// Transition model selecting laminar/transitional/turbulent per node.
    pub transition: TransitionSelector,
    /// Cone↔flat-plate Mangler correction toggle per station.
    pub body_class: BodyClass,
}

impl DistributedHeatingDeck {
    /// Evaluate q_w at every station for a real-gas edge state, blending
    /// laminar/turbulent by intermittency, applying Mangler for conical
    /// stations. Deterministic; stations iterated in declared order.
    pub fn evaluate(&self, edge: &FayRiddellEdgeState, ctx: &AerothermalContext)
        -> Result<Vec<SurfaceHeating>, AerothermalError>;
}

/// Engineering transition: smooth-wall Re_theta/M_e + rough-wall
/// PANT/Reda, with provenance-pinned threshold constants.
pub struct TransitionSelector {
    pub smooth: ReThetaTransition,                 // existing
    pub roughness: Option<PantRedaRoughness>,      // new, provenance-pinned
}
```

#### 3.2.4 In-depth charring material response (FIAT/CHAR-class)

The headline upgrade. Replaces `DepthResolvedCharringAblator`'s sharp-front
energy balance with a 1-D implicit conservation-law solver on a moving
(recession-tracking) grid. Trait-level so the explicit toy stays as the
low-fidelity rung.

```rust
/// Generic charring-ablator material (TACOT / textbook archetypes ONLY).
/// Multi-component decomposing resin+filler; temperature-dependent
/// virgin/char properties; B'-table surface chemistry. No real fielded
/// TPS cards (the existing ablation.rs lock is preserved and hardened).
pub struct CharringMaterial {
    pub name: &'static str,          // "tacot-open", "generic-cp-1"
    pub components: Vec<DecompositionComponent>,  // Arrhenius resin/filler
    pub virgin: TempDependentProps,  // k(T), cp(T), rho_v
    pub char: TempDependentProps,    // k(T), cp(T), rho_c
    pub permeability: Permeability,  // Darcy K(T, porosity)
    pub emissivity: TempDependentScalar,
    pub bprime: BPrimeTable,         // B'_c = f(B'_g, p_w, T_w), CEA-ingested
}

/// One Arrhenius decomposition reaction (resin or filler component):
///   drho_i/dt = -A_i (rho_i − rho_c_i) · ((rho_i−rho_c_i)/(rho_v_i−rho_c_i))^psi_i
///                · exp(−E_i/(R T))
pub struct DecompositionComponent {
    pub a_pre: f64, pub e_act_j_mol: f64, pub psi: f64,
    pub rho_virgin: f64, pub rho_char: f64, pub gamma_weight: f64,
}

/// 1-D implicit charring-ablator response solver.
pub struct InDepthMaterialResponse {
    pub material: CharringMaterial,
    pub mesh: MovingMesh1D,          // recession-tracking control volumes
    pub solver: ImplicitNewton,      // backward-Euler + Newton linearization
}

impl InDepthMaterialResponse {
    /// Advance one step under a surface heating BC. Solves, implicitly,
    /// the coupled nonlinear system:
    ///   energy:   rho cp dT/dt = d/dx(k dT/dx) + (h_g − h_s) dmdot_g/dx
    ///                            + h_pyro drho/dt
    ///   solid:    drho/dt = sum_i gamma_i drho_i/dt   (Arrhenius)
    ///   gas:      d(mdot_g)/dx = −drho/dt;  Darcy v_g = −(K/mu) dp/dx
    ///   surface:  rho_e u_e C_H (h_r − h_w) + mdot_c h_c + mdot_g h_g
    ///             − mdot_w h_w − eps sigma T_w^4 − q_cond_in = 0
    ///   (mdot_c from the B'-table: B'_c = f(B'_g, p_w, T_w))
    /// Newton iteration with locked operand order; fails closed on
    /// non-convergence or non-physical state (never panics).
    pub fn step(&mut self, dt_s: f64, bc: &SurfaceHeatBC)
        -> Result<MaterialResponseUpdate, AerothermalError>;

    /// Recession-tracking surface position and char-depth diagnostics.
    pub fn state(&self) -> MaterialResponseState;
}

/// B'-table surface mass balance (equilibrium surface composition from
/// CEA Gibbs minimization, ingested offline). Generic-material only.
pub struct BPrimeTable {
    package: ExternalReferencePackage,   // provenance + envelope (reused)
    grid: BPrimeGrid,                    // (B'_g, p_w, T_w) -> B'_c
}
```

#### 3.2.5 TPS sizing loop

```rust
/// Engineering TPS sizing: bisection on virgin thickness L until the
/// peak bondline temperature over the trajectory + soak-back equals the
/// allowable. Forward-only: sizes for a BONDLINE-TEMPERATURE constraint
/// on a vehicle-intrinsic trajectory — NEVER for terminal accuracy /
/// survivability against a ground target (dual-use lock, see §6).
pub struct TpsSizer {
    pub material: CharringMaterial,
    pub allowable_bondline_k: f64,
    pub margins: SizingMargins,          // heating factor, recession margin
}

impl TpsSizer {
    /// Given a trajectory heating history q(t) (from the stagnation +
    /// distributed decks), return required virgin thickness, total
    /// recession, char depth, peak bondline T and its timing.
    pub fn size(&self, heating_history: &HeatingHistory)
        -> Result<TpsSizingResult, AerothermalError>;
}
```

#### 3.2.6 Park two-temperature nonequilibrium (research tier)

The existing `NonequilibriumAir` / `ParkTwoTemperatureModel` API is kept. The WP
adds the **backward/equilibrium-constant path**, the Millikan-White + Park high-T
relaxation limiter, and a **0-D/1-D post-shock relaxation ODE driver** behind a
stiff implicit integrator (`diffsol` BDF, pure Rust, evaluated first; SUNDIALS
CVODE FFI as fallback). It emits species production (incl. electrons → blackout)
and a nonequilibrium edge state — it does **not** become a Navier-Stokes solver.

```rust
/// 0-D/1-D post-shock relaxation driver: integrates the Park 2T source
/// terms along the stagnation streamline from the (frozen) post-shock
/// state to the BL edge. Stiff implicit ODE; deterministic fixed-tol.
pub struct PostShockRelaxation<M: NonequilibriumAir> {
    pub model: M,
    pub integrator: StiffOdeProfile,     // BDF; declared in scenario hash
}
```

### 3.3 The math (named formulations + references)

All equations cite primary sources; coefficient transcription is provenance-gated
(§4). `GM_earth`, `J2`, `R_earth` appear only symbolically anywhere in this doc.

**(M1) Sutton-Graves engineering convective (shipped; extend K per gas).**
`q_w = K·√(ρ_∞/R_n)·V_∞³`, `K_earth = 1.7415e-4` SI. Hot-wall factor
`max(0, 1−h_w/h_aw)`. Allen-Eggers closed-form peak/load over an exponential
atmosphere (already in source). Extension: expose `K` per gas and a
Detra-Kemp-Riddell cross-check variant. *Sutton & Graves, NASA TR R-376, 1971;
Allen & Eggers, NACA TR 1381, 1958; Detra-Kemp-Riddell, Jet Propulsion 27, 1957.*

**(M2) Fay-Riddell real-gas convective stagnation.** Axisymmetric:
`q_w = 0.763·Pr^−0.6·(ρ_e μ_e)^0.4·(ρ_w μ_w)^0.1·(du_e/dx)^0.5·(h_aw−h_w)·[1+(Le^a−1)(h_D/h_aw)]`;
2-D constant 0.57. Lewis exponent `a = 0.52` (equilibrium BL / fully catalytic),
`0.63` (frozen / non-catalytic), interpolated for partial catalysis. Stagnation
velocity gradient (modified-Newtonian): `du_e/dx|_stag = (1/R_n)·√(2(p_e−p_∞)/ρ_e)`.
**Verification action:** the shipped code uses the leading constant `0.94` and
the `(ρ_w μ_w)^0.1(ρ_e μ_e)^0.4` ordering; the canonical axisymmetric value is
`0.763·Pr^−0.6`. `WP-04.2-c` **must** verify the constant/exponent placement
against the primary 1958 paper and pin the chosen form with a tolerance table —
do not silently keep `0.94`. Real-gas coupling replaces the perfect-gas
Rankine-Hugoniot edge (§3.2.2). *Fay & Riddell, J. Aero. Sci. 25(2), 1958;
Anderson, Hypersonic and High-Temperature Gas Dynamics, 2nd ed., 2006, Ch. 6.*

**(M3) Tauber-Sutton equilibrium radiative stagnation.**
`q_rad = C·R_n^a·ρ_∞^b·f(V)`, with `a = a(V, ρ_∞)` (radius exponent itself
velocity/density-dependent — preserve this) and `f(V)` a **tabulated** velocity
function (~3 km/s where f≈0 to ~16 km/s), interpolated, not power-law-fitted.
Earth-air and a reserved Mars CO2/N2 set. Total `q_w,stag = q_conv + q_rad`.
*Tauber & Sutton, J. Spacecraft & Rockets 28(1), 1991, doi:10.2514/3.26206.*

**(M4) Eckert reference-enthalpy distributed heating (laminar).**
`h* = 0.5(h_e + h_w) + 0.22(h_aw − h_e)`; evaluate `ρ*, μ*` at `h*` and `p_e`.
Laminar `C_f* = 0.664/√(Re_x*)`, `St* = C_f*/2·Pr*^−2/3` (Reynolds analogy),
`q_w = St*·ρ_e u_e·(h_aw − h_w)`. Recovery `r = Pr^1/2` (laminar); `h_aw = h_e +
r u_e²/2`. *Eckert, WADC TR 59-624, 1960.*

**(M5) Van Driest II turbulent skin-friction transform (implicit).**
`0.242/(√C_f·A)·[asin((2A²−B)/Q) + asin(B/Q)] = 0.41 + log10(Re_x C_f) −
ω·log10(T_w/T_e)`, with `F_c = (h_aw/h_e − 1)/(asin α + asin β)²`. Newton-iterate
`C_f` deterministically with a fixed cap; turbulent recovery `r = Pr^1/3`.
*Van Driest, JAS 18, 1951 & Aero. Eng. Rev. 15, 1956; Hopkins & Inouye, AIAA J.
9, 1971.*

**(M6) Mangler cone↔flat-plate transformation.** Laminar cone heating is `√3`
higher than the flat plate at the same length (multiply laminar `Re` by 3).
Applied per conical station. *Anderson Ch. 6; Bertin, Hypersonic
Aerothermodynamics, AIAA 1994.*

**(M7) Boundary-layer transition (engineering).** Smooth-wall onset when
`Re_θ/M_e` exceeds a body-class threshold (cones ~200–350). Rough-wall PANT:
`Re_θ(T_e/T_w)(k/θ) ≈ const` (~215). Reda ballistic-range roughness Reynolds
`Re_kk = ρ_k u_k k/μ_k` with effectively-smooth / transitional / fully-rough
regimes. Per-station laminar→transitional ramp→turbulent switch. e^N stays
typed-reserved (out of engineering scope). *Reda, JSR 39(2), 2002; Anderson
PANT, SAMSO-TR, 1975.*

**(M8) Srinivasan-Tannehill equilibrium-air curve fits (real-gas backbone).**
Grabau-type piecewise polynomial fits: `T = T(ρ,e)`, `p = p(ρ,e)`, `a = a(ρ,e)`
in `log(ρ/ρ_0)` and an internal-energy variable, valid to ~25,000 K / 100 atm;
companion `h(p,s)` fits. Deterministic, no iteration — the inline live-kernel
tier. CEA Gibbs minimization (`G = Σ n_j(μ_j° + RT ln(n_j P/n))` under elemental
mass balance via Lagrange + Newton) is used **offline** to generate/validate the
tables. *Srinivasan, Tannehill & Weilmuenster, NASA RP-1181, 1987; Gordon &
McBride (CEA), NASA RP-1311, 1994/96; McBride et al., NASA TP-2002-211556.*

**(M9) Charring-ablator material response (FIAT/CHAR governing equations).**
1-D moving-coordinate coupled PDEs (§3.2.4): solid energy with conduction +
pyrolysis-gas convection + decomposition enthalpy; multi-component Arrhenius
decomposition `drho_i/dt = −A_i(ρ_i−ρ_c_i)((ρ_i−ρ_c_i)/(ρ_v_i−ρ_c_i))^ψ_i
exp(−E_i/RT)`; pyrolysis-gas mass conservation with Darcy percolation
`v_g = −(K/μ)dp/dx`; B′-coupled surface energy/mass balance. Finite-volume,
fully implicit (backward Euler), Newton-linearized each step. *Chen & Milos
(FIAT), JSR 36(3), 1999; Amar et al. (CHAR), AIAA 2016-3385; Moyer & Rindal (CMA),
Aerotherm 1968; Lachaud & Mansour (PATO), JTHT 28(2), 2014.*

**(M10) TPS sizing / transient SEB.** Couple `q(t)` to the §M9 solver as a
time-varying BC; bisection/secant on virgin thickness `L` until peak bondline
`T` over trajectory + soak-back equals the allowable; documented margin
multipliers; output virgin thickness, recession, char depth, peak bondline T and
timing. *Milos & Chen, JSR 46(6), 2009; Laub & Venkatapathy, IPPW 2003.*

**(M11) Park two-temperature nonequilibrium (research).** Two energy equations
(total; vibrational-electronic). Forward rates `k_f = A T_a^n exp(−E_a/(k T_a))`,
controlling temperature `T_a = √(T·T_v)` for dissociation (Park geometric-mean).
Park-93 / Gupta-90 reaction sets (5-species neutral; 11-species ionized).
Backward rates from `K_eq(T)`. Landau-Teller vibrational relaxation with
Millikan-White `τ` + Park high-T correction; CVDV/Marrone-Treanor coupling.
Species continuity `dρ_s/dt = ω_s`. 0-D/1-D stagnation-streamline ODE, **not**
NS. *Park, Nonequilibrium Hypersonic Aerothermodynamics, Wiley 1990; Park, JTHT
7(3), 1993; Gupta et al., NASA RP-1232, 1990; Millikan & White, JCP 39, 1963.*

### 3.4 Fidelity tiers (T0…T4)

| Tier | Capability | In-repo / coupled | Label achievable |
|---|---|---|---|
| **T0 (shipped)** | Sutton-Graves stagnation + Allen-Eggers heat load (Stardust-validated); cold-gas Fay-Riddell scaffold w/ perfect-gas R-H edge; reference-enthalpy distributed (cold-gas, no Van Driest II); sharp-front charring toy; explicit-FD 1-D conduction; honest refusing stubs (Tauber-Sutton, Tannehill, Park). | in-repo | `validated-toy` (S-G); `experimental` (rest) |
| **T1 (fast wins, weeks)** | Tauber-Sutton `f(V)` table + C/a/b with provenance → radiative + total `q = q_conv + q_rad` above ~9 km/s; Re_θ/M_e + PANT/Reda transition switching the distributed model; Sutton-Graves K-per-gas + Detra-Kemp-Riddell cross-check. All algebraic, provenance-gated. | in-repo | `checked` → `research` (Tauber-Sutton vs Apollo 4 / FIRE II) |
| **T2 (real-gas edge, 1–2 mo)** | Srinivasan-Tannehill equilibrium-air curve fits (`T,p,a,h` from `ρ,e`) as a deterministic module; `EquilibriumEdgeAssembler` populates `FayRiddellEdgeState` and h_D → genuine real-gas catalytic-wall stagnation heating; Fay-Riddell constant/exponent verified vs the 1958 paper. Validate vs CEA. | in-repo (fits) + CEA ingest (offline) | `validated-toy` → `research` |
| **T3 (distributed deck + implicit material response, 2–3 mo)** | Validated Eckert + Van Driest II laminar/turbulent distributed deck with Mangler → body heating deck; 1-D implicit FIAT/CHAR-class charring solver (conduction + multi-component Arrhenius + Darcy gas + B′-coupled SEB) on TACOT/generic materials; TPS-sizing loop (bondline-temperature thickness search). | in-repo (solver) + CEA B′-table + FIAT/PATO oracle (offline) | `validated-toy` → `research` |
| **T4 (nonequilibrium, research, multi-month)** | 0-D/1-D post-shock relaxation w/ Park-93 two-temperature finite-rate chemistry (5- then 11-species), electron-density / blackout; stiff implicit ODE. Beyond this, **couple** SU2-NEMO / SPARTA, do not port. | in-repo (0-D/1-D) + SU2-NEMO/SPARTA (coupled oracle) | `research` (uncertain convergence; capped) |

---

## 4. Invariant preservation

Concretely for this dimension (every WP keeps all of `00` §3):

- **Byte-determinism.** All new arithmetic uses locked operand order, **no
  `f64::mul_add`**, no wall-clock, no system RNG, no unordered iteration. Table
  interpolation (Tauber-Sutton `f(V)`, Srinivasan-Tannehill, B′) uses a fixed
  bracket-search + linear/monotone-clamped blend with a documented operand
  order. The Van Driest II and material-response **Newton iterations** are
  deterministic: fixed iteration cap, fixed tolerance, fixed initial guess, no
  data-dependent early-exit that could reorder operations across platforms;
  non-convergence is a fail-closed `OutOfEnvelope`, never a panic or a NaN. Body
  stations and depth nodes iterate in declared `Vec` order. Verified by the CI
  byte-diff gate on the canonical scenario set and per-module
  `determinism_two_runs_byte_identical` tests (the existing pattern in
  `stagnation.rs` / `ablation.rs`).

- **Byte-stable-by-default.** Every new model is **off** until a scenario opts in
  via an explicit config block (e.g. `[vehicle.aerothermal.radiative]`,
  `[vehicle.tps.material_response]`, `[vehicle.aerothermal.realgas_edge]`,
  `[vehicle.aerothermal.nonequilibrium]`), exactly like `[vehicle.bending]`.
  Existing scenarios and golden archives remain byte-identical: the cold-gas
  Fay-Riddell trait path, the sharp-front toy, and the refusing stubs stay the
  defaults until a scenario selects the new tier. Tauber-Sutton **continues to
  refuse** until its `f(V)` package is both present and selected.

- **FC hardware-portability lock.** `openbmp-aerothermal` is already on the
  FC-forbidden list (`00` §3.3); the new `openbmp-thermochem` inherits that — no
  heating/thermochem/material-response code ever enters `openbmp-fc`. Heating and
  TPS state reach any GNC/sequencer logic only as **telemetry / sensor channels**
  through the existing FC-bridge boundary, never by importing the simulator. The
  `fc_dependency_tripwire.rs` test stays green; `WP-04.2-a` adds
  `openbmp-thermochem` to the forbidden-edge set in the same PR that creates it.

- **Four-pillar provenance.** Every ingested coefficient set — Tauber-Sutton
  `C/a/b/f(V)`, Srinivasan-Tannehill polynomial coefficients, Park backward-rate
  / Millikan-White constants, B′ tables, FIAT/PATO/NEQAIR/CEA code-to-code
  outputs, TACOT material card — lands under `data/aerothermal/` (or
  `data/thermochem/`) with a sibling `provenance.md` listing it and a SHA-256
  content pin verified fail-closed at load, reusing the existing
  `ExternalReferencePackage` / `ProvenanceBlock` machinery already used for the
  Park reference payload. **No inline TOML/data constants in `*.rs`** beyond
  allow-listed source-of-truth files; the `inline_data_tripwire.rs` test stays
  green (new source-of-truth paths added to its allow-list in the same PR). The
  audit precedent is explicit: the tuned Tauber-Sutton power law and the
  synthesized Tannehill table were **removed** rather than shipped unverified —
  this dimension re-introduces them only as cited, hash-pinned tables. **No
  literal `GM_earth` / `J2` / `R_earth` numerals** anywhere (referred to
  symbolically; the doc tripwire stays green).

- **Validation labels.** Each tier declares exactly one of `experimental` →
  `checked` → `validated-toy` → `research` (`docs/verification.md`), justified by
  the §5 evidence, with a tolerance-table TOML for every numeric claim (the
  `[[metric]]` schema with `expected`/`absolute_tolerance`/`relative_tolerance`,
  per `crates/openbmp-testkit/tests/fixtures/tolerance-sample.toml`). **No
  artifact claims `flight-qualified`/`certified`/`operational`/`mission-ready`.**
  The Park 2T model and the absolute radiative level above ~12 km/s are capped at
  `research` with an explicit uncertain-convergence note.

- **Generic-material lock (this dimension's special invariant).** Only TACOT and
  textbook archetypes ship; any request for a real fielded/export-controlled card
  (PICA/AVCOAT/carbon-phenolic kinetics) routes to refusal. `WP-04.3-c` adds a
  compile-fail/tripwire test proving a real-material card cannot be constructed as
  a bare literal (mirroring `ballistic_state_compile_fail.rs`), hardening the
  existing `ablation.rs` "No real fielded TPS material parameters ship" rule.

---

## 5. V&V plan

Instantiates the `docs/verification.md` Hypersonic V&V & UQ ladder (code
verification → model verification → code-to-code → public flight → UQ). A model
reaches `research` only with ≥1 analytic/MMS case + ≥1 public reference +
declared envelope + UQ story.

### 5.1 Verification & validation cases by tier

| Tier / model | Code verification (MMS/analytic) | Model verification | Code-to-code | Public flight | Label |
|---|---|---|---|---|---|
| T1 Tauber-Sutton radiative | f(V) table monotone-interp unit checks; q_rad → 0 below the table floor velocity | published Tauber-Sutton worked examples | NASA equilibrium radiative tables (Johnston/Mazaheri) | Apollo 4 (~11 km/s peak), FIRE II radiative | `research` |
| T1 transition (Re_θ/M_e, PANT/Reda) | threshold/blend boolean logic; laminar↔turbulent continuity | Bertin/Anderson cone thresholds | — | sharp-cone heating jump location | `checked`→`validated-toy` |
| T2 Srinivasan-Tannehill equil-air | exact perfect-gas limit recovery as T→low; round-trip (ρ,e)↔(T,p) | — | **CEA / Cantera** over a (ρ,e) grid: T,p,a,γ_eff | — | `validated-toy`→`research` |
| T2 real-gas Fay-Riddell | reduces byte-for-byte to cold-gas path when fed the perfect-gas edge (regression) | constant/exponent vs 1958 paper | DPLR Stardust reconstruction (Olynick/Chen 1999) | Stardust SRC peak convective | `research` |
| T3 Eckert + Van Driest II deck | MMS on the BL ODE; Van Driest II Newton convergence-order | Van Driest theory curves | DTIC ADA496574 correlation compendium; SU2-Euler/RANS deck | flat-plate / sharp-cone (Holden/CUBRC) | `validated-toy`→`research` |
| T3 implicit charring solver | **MMS** on the 1-D conduction+decomposition PDE (order-of-accuracy); energy-conservation balance; explicit-toy agreement in the inert limit | analytic semi-infinite conduction | **FIAT v3 / CHAR / PATO** on TACOT (NASA/CR-2015-218960) | — | `validated-toy`→`research` |
| T3 TPS sizing loop | bisection monotonicity / convergence; soak-back energy balance | — | FIAT sizing on TACOT | — | `validated-toy` |
| T4 Park 2T nonequilibrium | exact scalar stiff-ODE relaxation; equilibrium-limit recovery (k_f/k_b → K_eq) | Park-93 published rate plots | DPLR/LAURA/US3D FIRE II; SPARTA (rarefied) | **FIRE II** (convective+radiative), **RAM-C II** (electron density) | `research` (capped) |

### 5.2 Tolerance tables (target acceptance bands)

Shipped as `[[metric]]` TOML alongside each WP. Bands reflect the **honest**
spread of the open benchmarks (e.g. FIRE II radiative is known to spread widely
even among production codes — the band must not pretend otherwise).

| Case | Metric | Expected source | Target tolerance | Notes |
|---|---|---|---|---|
| Srinivasan-Tannehill vs CEA | `T(ρ,e)`, `p(ρ,e)`, `a(ρ,e)` | CEA grid | ≤ 2–3% over 1k–15k K, ≤5% to 25k K | the fit's published accuracy class |
| Real-gas Fay-Riddell vs Stardust | peak `q_conv` | Stardust table | within ~15–20% | engineering correlation vs DPLR |
| Tauber-Sutton vs FIRE II | peak `q_rad` | FIRE II published | within ~30–40% (wide on purpose) | radiative spread is large; state it |
| Van Driest II flat-plate | turbulent `C_f` | Van Driest theory | within ~5% | implicit Newton band |
| Charring solver vs FIAT/PATO (TACOT) | bondline `T(t)`, recession | FIAT/PATO | within ~10–15% on bondline T | generic-material code-to-code |
| Park 2T relaxation | equilibrium-limit composition | analytic K_eq | machine-zero in the limit; envelope only otherwise | capped `research` |

### 5.3 UQ / 7009B

Each deck node and material output carries an error budget (numerical tolerance,
model-form, data pedigree, interp/extrap, atmosphere variability) feeding a
**heating-augmentation factor** and **recession margin** draw into the Monte
Carlo (doc 11), reported in a NASA-STD-7009B-shaped credibility record (the same
eight-factor structure used elsewhere). Model-form uncertainty between tiers is
estimated honestly from the overlap spread (e.g. |real-gas − cold-gas Fay-Riddell|,
|Sutton-Graves − Fay-Riddell|, |engineering − SU2-NEMO|) and **inflated**, not
hidden, where no flight data anchors it.

---

## 7. Dependencies on other parity docs

- **`03-aerodynamics-database-and-cfd-coupling.md`** — *tight pair.* The aero
  deck supplies edge flow conditions (M, α, β, local pressure / inclination,
  line loads, body stations) that the distributed heating deck (§3.2.3)
  integrates over; the panel-method surface triangulation and BVH (doc 03 T1)
  are the geometry source for per-station heating. Both consume the same
  run-matrix orchestrator (`openbmp-aerodb`) and the same SU2/Cart3D/SPARTA
  coupling. Stage-separation proximity (doc 03 T3) implies local heating changes.
- **`08-environment-gravity-and-frames.md`** — freestream `ρ_∞, p_∞, T_∞, a_∞`
  and the perturbed/high-altitude atmosphere; entry-trajectory propagation.
- **`05-propulsion-high-fidelity.md`** — shares `openbmp-thermochem` (equilibrium
  / CEA-class thermochemistry); plume/base aerothermal coupling is a shared
  parity-ceiling item (proprietary reacting-plume data).
- **`11-monte-carlo-uq-and-validation.md`** — `openbmp-uq` / `openbmp-mc` carry
  the heating-augmentation / recession-margin / material-property dispersions and
  the 7009B credibility record; MMS infrastructure for the material-response PDE.
- **`12-determinism-realtime-and-compute.md`** — deterministic Newton iterations,
  stiff-ODE profile in the scenario hash, byte-stable table interpolation.
- **`01-flexible-multibody-dynamics.md`** / **`02-structural-dynamics-loads-slosh-pogo.md`**
  — TPS mass / recession feeds vehicle mass properties; heating-induced material
  property change is downstream of structural state (loose coupling only).
- **`06-gnc-coupled-mimo-and-control.md`** / **`09-sensors-navigation-and-actuators.md`**
  — heating / TPS state reaches GNC only as telemetry across the FC boundary
  (the portability lock), never as a simulator import.

---

## 8. Open-source leverage

Use mode per `00` §1.1 (port / couple / ingest). **Couple/ingest** for anything
heavyweight; **port (data)** only public-domain coefficient sets.

| Tool | License | Mode | Use |
|---|---|---|---|
| **NASA CEA / CEARUN** | US-Gov work (freely distributed via NASA Glenn) | **INGEST (offline)** | Generate equilibrium-air thermo+composition tables and B′ surface-equilibrium tables to (a) fit/validate the Srinivasan-Tannehill Rust module and (b) cross-check Fay-Riddell real-gas edge states. Not runtime-coupled. Document provenance; verify redistribution terms before vendoring any generated table. |
| **Cantera** | BSD-3-Clause (permissive) | **INGEST / cross-check** (optional FFI offline) | Thermochemistry, finite-rate kinetics, NASA-9 polynomial evaluation to validate the equilibrium-air fits and the Park-93 rates. Safe to link/vendor with attribution. |
| **Srinivasan-Tannehill (RP-1181) / Tannehill (CR-2470) coefficients** | US-Gov / public domain | **PORT (data)** | Transcribe the Grabau-type polynomial coefficients into the provenance-tracked Rust equilibrium-air module — ideal for the "generic + cited" discipline. |
| **Tauber-Sutton (JSR 1991) C/a/b + f(V) table** | published paper (transcribe values, cite) | **PORT (data)** | Lift the typed-reserved radiative stub. Table interp, not power-law fit. |
| **Mutation++ (VKI)** | LGPL | **COUPLE / REFERENCE** (FFI to separate process preferred) | Multicomponent thermo/transport/chemistry (incl. ionized) and B′ table generation — the de-facto edge-state + SEB reference. Prefer coupling over porting (static linking / code-lifting triggers LGPL). |
| **FIAT v3 / CHAR** | NASA software-usage agreement (access-gated) | **COUPLE / CODE-TO-CODE** | High-fidelity material-response oracle for the T3 charring solver, on TACOT/generic materials. Where the user holds access. |
| **PATO (OpenFOAM-based, NASA Ames)** | GPL (copyleft) | **CODE-TO-CODE oracle ONLY** | Open reference for charring-ablator response. **Do NOT link/port** PATO/OpenFOAM into OpenBMP; run separately, compare outputs (keeps OpenBMP clean). |
| **SU2 / SU2-NEMO** | LGPL-2.1 | **COUPLE / CODE-TO-CODE** (subprocess) | Nonequilibrium hypersonic forebody flowfields + heating decks for code-to-code validation; the credible substitute for in-house high-fidelity CFD (parity ceiling). FFI/separate-process, not a hard dependency. |
| **SPARTA (Sandia DSMC)** | GPL | **CODE-TO-CODE oracle ONLY** (subprocess) | Rarefied / high-Knudsen regime to bound where continuum BL correlations are valid (RAM-C II altitude end). |
| **NEQAIR / public equilibrium radiative tables** | NASA (access/public tables) | **INGEST / cross-check** | Cross-check Tauber-Sutton above ~9 km/s. |
| **diffsol / ode_solvers (Rust)** | MIT / Apache-2.0 | **PORT/LINK** | Pure-Rust stiff (BDF) integrator for the T4 Park post-shock relaxation ODE; evaluate first. |
| **SUNDIALS CVODE** | BSD-3-Clause | **COUPLE (FFI)** fallback | If `diffsol` stiffness handling proves insufficient. |
| **TACOT property set** | open / non-export-controlled | **PORT (data)** | The generic carbon-phenolic surrogate defined for open material-response code comparison — OpenBMP's charring material card. |

Controlled tools (FIAT, FUN3D, DATCOM-adjacent, Cart3D, CBAERO) are **never
vendored or redistributed**; coupling is the user's responsibility under their
own access, which also keeps OpenBMP clean of controlled material.

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, green on the full `13` §2 gate set,
each shipping at a declared tier and earning a declared label. Backlog schema per
`13` §5.

---

**WP-04.1-a — Import Tauber-Sutton radiative coefficients + f(V) table**
- **title:** Lift the radiative-heating stub with provenance-pinned coefficients.
- **goal:** Replace the refusing `TauberSuttonRadiative` with a real model
  emitting `q_rad = C·R_n^a·ρ_∞^b·f(V)` from a published, hash-pinned Earth-air
  coefficient set + tabulated `f(V)`, enabling total `q = q_conv + q_rad` above
  ~9 km/s. Closes the single largest "honest refusal" in stagnation heating.
- **fidelity_tier:** T1
- **depends_on:** []
- **new_crates:** none.
- **touched:** `crates/openbmp-aerothermal/src/stagnation.rs`;
  `data/aerothermal/tauber_sutton_earth/` + `provenance.md`;
  `inline_data_tripwire.rs` allow-list.
- **approach:** §3.2.1, §M3. Tabulated `f(V)` with monotone-clamped interp;
  preserve `a = a(V, ρ)`; refuse outside the velocity envelope. Reuse
  `ExternalReferencePackage`/`ProvenanceBlock` (SHA-256). No power-law fit.
- **acceptance:**
  - off by default; canonical goldens byte-identical.
  - `f(V)` interpolation unit checks (monotone, `f→0` at floor velocity) in a
    tolerance table; refuses below floor and above ceiling velocity.
  - Apollo-4 (~11 km/s) and FIRE II peak `q_rad` code-to-code within the §5.2
    band (wide, honestly stated) with provenance.
  - all §2 gates green.
- **validation_label:** `research` (with explicit radiative-spread uncertainty).
- **dual_use_note:** far from line; forward heating only.
- **est_effort:** 3–5 days.
- **parity_ceiling:** does NOT validate absolute radiative level (large
  code-to-code spread even among production tools); not a coupled
  radiation-transport solver.

**WP-04.1-b — Engineering transition + Sutton-Graves K-per-gas**
- **title:** Wire Re_θ/M_e + PANT/Reda transition and multi-K Sutton-Graves.
- **goal:** Make the distributed model switch laminar→transitional→turbulent
  from a credible, cited criterion (largest TPS-sizing uncertainty), and add a
  Detra-Kemp-Riddell Sutton-Graves cross-check + per-gas K.
- **fidelity_tier:** T1
- **depends_on:** []
- **new_crates:** none.
- **touched:** `boundary_layer.rs`, `stagnation.rs`;
  `data/aerothermal/transition/` + `provenance.md`.
- **approach:** §M7 (Re_θ/M_e thresholds + PANT `≈215` + Reda `Re_kk` regimes;
  per-station blend), §M1 (K-per-gas, DKR variant). e^N stays typed-reserved.
- **acceptance:**
  - off by default; goldens byte-identical.
  - threshold/blend boolean + continuity tests; transition location matches a
    sharp-cone reference within tolerance table.
  - all gates green.
- **validation_label:** `checked` → `validated-toy`.
- **dual_use_note:** far from line.
- **est_effort:** ~1 week.
- **parity_ceiling:** constants are body/facility-dependent; not e^N stability.

**WP-04.2-a — Propose & skeleton `openbmp-thermochem` (L2)**
- **title:** Create the equilibrium-air / thermochemistry crate skeleton.
- **goal:** Establish the L2 home for the Srinivasan-Tannehill fits + CEA/Cantera
  ingestion + B′ tables (shared with doc 05), with a reviewed dependency boundary.
- **fidelity_tier:** T2
- **depends_on:** []
- **new_crates:** `openbmp-thermochem` L2 (depends down on `openbmp-core`,
  `openbmp-physics`; forbidden edge to `openbmp-fc`).
- **touched:** new crate skeleton; workspace `Cargo.toml`;
  `fc_dependency_tripwire.rs` forbidden-edge set.
- **approach:** §3.1. First commit is the skeleton + placement justification
  (layer, edges, why it does not break the DAG or FC lock). May instead fully
  realize `openbmp-physics::realgas::tannehill` if the reviewer prefers a module
  over a crate — decide here.
- **acceptance:**
  - DAG acyclic; FC tripwire green with the new crate on the forbidden list.
  - empty crate builds, lints clean, documented.
  - all gates green.
- **validation_label:** `experimental` (skeleton).
- **dual_use_note:** far from line.
- **est_effort:** 1–2 days.
- **parity_ceiling:** no physics yet.

**WP-04.2-b — Srinivasan-Tannehill equilibrium-air curve fits**
- **title:** Implement the inline equilibrium-air `T,p,a,h(ρ,e)` curve fits.
- **goal:** Provide the deterministic real-gas backbone (no iteration) that
  populates the Fay-Riddell edge state — the gating prerequisite for genuine
  real-gas stagnation heating and B′ ablation.
- **fidelity_tier:** T2
- **depends_on:** [WP-04.2-a]
- **new_crates:** none (lands in `openbmp-thermochem` / `realgas::tannehill`).
- **touched:** `openbmp-thermochem` (or `realgas/tannehill.rs`);
  `data/thermochem/srinivasan_tannehill/` + `provenance.md`; allow-list.
- **approach:** §M8. Transcribe the RP-1181/CR-2470 Grabau-type coefficients with
  provenance; deterministic piecewise-polynomial evaluation, locked operand
  order, no FMA. Lift the `TannehillEquilibriumAir` refusal.
- **acceptance:**
  - perfect-gas limit recovered as T→low; `(ρ,e)↔(T,p)` round-trip within tol.
  - **CEA / Cantera code-to-code** over a `(ρ,e)` grid (T, p, a, γ_eff) within
    §5.2 band, tolerance table + provenance.
  - off by default; goldens byte-identical.
  - all gates green.
- **validation_label:** `validated-toy` → `research`.
- **dual_use_note:** far from line.
- **est_effort:** 2–4 weeks.
- **parity_ceiling:** fit accuracy class; not a runtime Gibbs minimizer (that
  stays offline/CEA).

**WP-04.2-c — Real-gas Fay-Riddell edge-state assembler + constant verification**
- **title:** Feed `FayRiddellEdgeState` from equilibrium air; verify the 1958
  constant/exponents.
- **goal:** Turn the cold-gas Fay-Riddell into a genuine real-gas
  catalytic-wall stagnation model via `EquilibriumEdgeAssembler`, and resolve the
  `0.94` vs `0.763·Pr^−0.6` constant question against the primary paper.
- **fidelity_tier:** T2
- **depends_on:** [WP-04.2-b]
- **new_crates:** none.
- **touched:** `stagnation.rs` (assembler), `openbmp-thermochem` (wall/transport).
- **approach:** §3.2.2, §M2. Equilibrium normal-shock jump + stagnation
  compression → `FayRiddellEdgeState`; Gupta-Yos/Blottner μ_e. **Verify the
  leading constant + exponent ordering vs Fay & Riddell 1958** and pin the
  chosen form (do not silently keep `0.94`).
- **acceptance:**
  - reduces **byte-for-byte** to the existing cold-gas trait path when fed the
    perfect-gas edge (regression test).
  - Stardust SRC peak `q_conv` within §5.2 band; DPLR reconstruction code-to-code.
  - assembler **fails closed** out of envelope (never silent perfect-gas fallback).
  - off by default; goldens byte-identical; all gates green.
- **validation_label:** `research`.
- **dual_use_note:** far from line.
- **est_effort:** 1–2 weeks.
- **parity_ceiling:** engineering correlation, not coupled shock-layer CFD.

**WP-04.3-a — Eckert + Van Driest II distributed heating deck**
- **title:** Promote distributed heating to a verified laminar+turbulent deck.
- **goal:** Produce a body heating deck `q_w(s)` with Eckert laminar + Van Driest
  II turbulent + Mangler cone correction — the heating away from the nose that
  TPS sizing needs.
- **fidelity_tier:** T3
- **depends_on:** [WP-04.2-c, WP-04.1-b]
- **new_crates:** none.
- **touched:** `boundary_layer.rs`; `data/aerothermal/heating_deck/` + provenance.
- **approach:** §3.2.3, §M4–M6. Implicit Van Driest II Newton (fixed cap,
  fail-closed); Mangler per conical station; blend by intermittency; real-gas h*
  from the equilibrium edge.
- **acceptance:**
  - MMS on the BL relation; Van Driest II convergence-order test.
  - flat-plate / sharp-cone `C_f` and `q_w` within §5.2 band (Van Driest theory +
    DTIC ADA496574; SU2 code-to-code where available); tolerance tables.
  - off by default; goldens byte-identical; all gates green.
- **validation_label:** `validated-toy` → `research`.
- **dual_use_note:** far from line.
- **est_effort:** 2–3 weeks.
- **parity_ceiling:** correlation deck, not RANS; transition constants uncertain.

**WP-04.3-b — 1-D implicit charring-ablator material response (FIAT/CHAR-class)**
- **title:** Replace the sharp-front toy with a conservation-law solver.
- **goal:** The headline TPS upgrade: in-depth conduction + multi-component
  Arrhenius decomposition + Darcy pyrolysis gas + B′-coupled SEB on
  recession-tracking nodes, on TACOT/generic materials only.
- **fidelity_tier:** T3
- **depends_on:** [WP-04.2-b]  (B′ tables need the equilibrium/CEA path)
- **new_crates:** none (lands in `openbmp-aerothermal::ablation` +
  `openbmp-thermochem` B′ tables).
- **touched:** `ablation.rs`; `data/aerothermal/tacot/` + `provenance.md`;
  `data/thermochem/bprime/` + `provenance.md`; allow-list.
- **approach:** §3.2.4, §M9. Finite-volume, backward-Euler, Newton-linearized
  each step with locked operand order; B′-table SEB (CEA-ingested, generic
  material). Keep the explicit sharp-front toy as the low-fidelity rung.
- **acceptance:**
  - **MMS** on the conduction+decomposition PDE (order-of-accuracy); energy
    balance closes; reduces to the explicit toy in the inert limit.
  - **FIAT/CHAR/PATO** code-to-code on TACOT (NASA/CR-2015-218960): bondline
    `T(t)` and recession within §5.2 band; tolerance tables + provenance.
  - off by default; goldens byte-identical; all gates green.
- **validation_label:** `validated-toy` → `research`.
- **dual_use_note:** generic-materials-only lock preserved (see WP-04.3-c).
- **est_effort:** 2–3 months.
- **parity_ceiling:** generic materials only; no real PICA/AVCOAT; 1-D (2-D/3-D
  via external PATO oracle).

**WP-04.3-c — Generic-material lock + TPS sizing loop**
- **title:** Close the bondline-temperature thickness search; harden the
  material-card lock.
- **goal:** Deliver the actual TPS-sizing deliverable (min virgin thickness for a
  bondline limit) and add the compile-fail/tripwire proving no real fielded card
  can be constructed and no terminal-accuracy/survivability objective can be
  expressed.
- **fidelity_tier:** T3
- **depends_on:** [WP-04.3-b]
- **new_crates:** none.
- **touched:** `ablation.rs` / new `tps_sizing` module;
  `crates/openbmp-testkit/tests/tps_material_card_compile_fail.rs` (new).
- **approach:** §3.2.5, §M10. Bisection/secant on thickness; soak-back window;
  documented margin multipliers. Tripwire mirrors
  `ballistic_state_compile_fail.rs`: real-material card unconstructible as a bare
  literal; sizing objective vocabulary cannot reference target/aimpoint/CEP.
- **acceptance:**
  - sizing convergence/monotonicity tests; FIAT-sizing code-to-code on TACOT.
  - **compile-fail test** green (real card + ground-aimpoint objective both
    rejected at compile time).
  - off by default; goldens byte-identical; all gates green.
- **validation_label:** `validated-toy`.
- **dual_use_note:** **lock-tightening WP** — adds the proving test (`00` §6).
- **est_effort:** 1–2 weeks.
- **parity_ceiling:** margin policy is generic, not a certified margin book.

**WP-04.3-d — Heating/recession UQ into Monte Carlo (7009B)**
- **title:** Per-quantity heating-augmentation + recession-margin dispersions.
- **goal:** Make the aerothermal deck Monte-Carlo-ready with honest model-form
  inflation and a 7009B-shaped credibility record.
- **fidelity_tier:** T3
- **depends_on:** [WP-04.3-a, WP-04.3-b]
- **new_crates:** none (uses `openbmp-uq`/`openbmp-mc` from doc 11).
- **touched:** deck schema; `openbmp-uq`; runner MC hooks.
- **approach:** §5.3. Error budget per node/output; correlated draw (not white
  noise) via `DeterministicRng`; model-form from tier-overlap spread; 7009B
  record.
- **acceptance:**
  - deterministic ensemble: byte-diff gate extends to ensemble statistics.
  - model-form term documented and inflated where unanchored.
  - all gates green.
- **validation_label:** `validated-toy`.
- **dual_use_note:** far from line.
- **est_effort:** 1–2 weeks.
- **parity_ceiling:** credible-but-uncalibrated margins (no flight correlation).

**WP-04.4-a — Park 2T backward rates + relaxation limiter (complete the source model)**
- **title:** Make `ParkTwoTemperatureModel` executable with provenance.
- **goal:** Add the backward/equilibrium-constant path, Millikan-White + Park
  high-T relaxation limiter, and species-pair vibrational constants so the
  two-temperature source model returns rates — the prerequisite for the
  post-shock relaxation driver.
- **fidelity_tier:** T4
- **depends_on:** [WP-04.2-b]
- **new_crates:** none (`openbmp-physics::realgas::park_2t`).
- **touched:** `park_2t.rs`; `data/thermochem/park_backward/` + provenance;
  reference-payload SHA pin.
- **approach:** §M11. `K_eq(T)` backward rates; geometric-mean `T_a`; complete
  the typed surface; keep failing closed until the public benchmark lands.
- **acceptance:**
  - equilibrium-limit recovery (`k_f/k_b → K_eq`) machine-zero.
  - off by default; goldens byte-identical; all gates green.
- **validation_label:** `research` (capped, uncertain convergence).
- **dual_use_note:** far from line.
- **est_effort:** 3–4 weeks.
- **parity_ceiling:** 0-D source terms only; no flowfield yet.

**WP-04.4-b — Post-shock relaxation ODE + FIRE II / RAM-C II (research)**
- **title:** 0-D/1-D stagnation-streamline nonequilibrium driver.
- **goal:** Integrate the Park 2T source along the stagnation streamline (stiff
  implicit ODE) to a nonequilibrium edge state and electron density; validate
  against FIRE II (heating) and RAM-C II (electron density / blackout).
- **fidelity_tier:** T4
- **depends_on:** [WP-04.4-a]
- **new_crates:** none.
- **touched:** `realgas/park_2t.rs` (driver); ODE-integrator dep
  (`diffsol`); `data/aerothermal/fire_ii/`, `data/aerothermal/ram_c_ii/` +
  provenance.
- **approach:** §3.2.6, §M11. `diffsol` BDF first; SUNDIALS FFI fallback;
  deterministic fixed-tol; profile in the scenario hash. Beyond 0-D/1-D, couple
  SU2-NEMO / SPARTA (oracle, not ported).
- **acceptance:**
  - exact scalar stiff-ODE relaxation verification; equilibrium-limit recovery.
  - FIRE II convective + radiative and RAM-C II electron-density code-to-code
    within (wide, honest) §5.2 envelope; provenance + UQ story.
  - off by default; goldens byte-identical; all gates green.
- **validation_label:** `research` (capped; uncertain convergence stated).
- **dual_use_note:** far from line.
- **est_effort:** 1–2 months.
- **parity_ceiling:** engineering 0-D/1-D, **not** nonequilibrium Navier-Stokes;
  multidimensional NEQ stays external (SU2-NEMO).

---

## 10. References

**Convective heating.**
- Fay, J.A. & Riddell, F.R., "Theory of Stagnation Point Heat Transfer in
  Dissociated Air," *J. Aeronautical Sciences* 25(2), 1958, pp. 73–85, 121.
- Sutton, K. & Graves, R.A., "A General Stagnation-Point Convective-Heating
  Equation for Arbitrary Gas Mixtures," NASA TR R-376, 1971.
- Allen, H.J. & Eggers, A.J., "...Aerodynamic Heating of Ballistic Missiles...,"
  NACA TR 1381, 1958.
- Detra, R.W., Kemp, N.H., Riddell, F.R., "Addendum to Heat Transfer to Satellite
  Vehicles Reentering the Atmosphere," *Jet Propulsion* 27, 1957.
- Eckert, E.R.G., "Survey of Boundary Layer Heat Transfer at High Velocities and
  High Temperatures," WADC TR 59-624, 1960.
- Van Driest, E.R., "Turbulent Boundary Layer in Compressible Fluids," *JAS* 18,
  1951; "The Problem of Aerodynamic Heating," *Aero. Eng. Review* 15, 1956.
- Hopkins & Inouye, "An Evaluation of Theories for Predicting Turbulent Skin
  Friction and Heat Transfer...," *AIAA J.* 9, 1971.
- Anderson, J.D., *Hypersonic and High-Temperature Gas Dynamics*, 2nd ed., AIAA
  Education Series, 2006 (Ch. 6, 16–18).
- Bertin, J.J., *Hypersonic Aerothermodynamics*, AIAA, 1994.

**Radiative heating.**
- Tauber, M.E. & Sutton, K., "Stagnation-Point Radiative Heating Relations for
  Earth and Mars Entries," *J. Spacecraft & Rockets* 28(1), 1991, pp. 40–42,
  doi:10.2514/3.26206.

**Transition.**
- Reda, D.C., "Review and Synthesis of Roughness-Dominated Transition
  Correlations for Reentry Applications," *J. Spacecraft & Rockets* 39(2), 2002.
- Anderson, A.D. (PANT), "Passive Nosetip Technology (PANT) Program," SAMSO-TR,
  1975.

**Equilibrium-air thermodynamics.**
- Srinivasan, S., Tannehill, J.C., Weilmuenster, K.J., "Simplified Curve Fits for
  the Thermodynamic Properties of Equilibrium Air," NASA RP-1181, 1987.
- Tannehill & Mugge, "Improved Curve Fits for the Thermodynamic Properties of
  Equilibrium Air," NASA CR-2470, 1974.
- Gordon, S. & McBride, B.J., "Computer Program for Calculation of Complex
  Chemical Equilibrium Compositions and Applications," NASA RP-1311, 1994/96.
- McBride, Zehe, Gordon, "NASA Glenn Coefficients for Calculating Thermodynamic
  Properties of Individual Species," NASA TP-2002-211556.

**Nonequilibrium chemistry.**
- Park, C., *Nonequilibrium Hypersonic Aerothermodynamics*, Wiley, 1990.
- Park, C., "Review of Chemical-Kinetic Problems of Future NASA Missions, I:
  Earth Entries," *J. Thermophysics & Heat Transfer* 7(3), 1993, pp. 385–398.
- Gupta, R.N., Yos, J.M., Thompson, R.A., Lee, K.P., "A Review of Reaction Rates
  and Thermodynamic and Transport Properties for an 11-Species Air Model...,"
  NASA RP-1232, 1990.
- Millikan & White, "Systematics of Vibrational Relaxation," *J. Chem. Phys.* 39,
  1963; Marrone & Treanor, *Phys. Fluids* 6, 1963.
- Zhang et al., "A review of the mathematical modeling of equilibrium and
  nonequilibrium hypersonic flows," *Adv. Aerodynamics*, 2022 (Table 2 — pinned
  Park87/Park93 forward rows in `park_2t.rs`).

**Material response & TPS.**
- Chen, Y.-K. & Milos, F.S., "Ablation and Thermal Response Program for
  Spacecraft Heatshield Analysis (FIAT)," *J. Spacecraft & Rockets* 36(3), 1999.
- Amar, A.J. et al., "Overview of the CHarring Ablator Response (CHAR) Code,"
  AIAA 2016-3385.
- Moyer & Rindal, "CMA (Charring Material Ablation)," Aerotherm, 1968.
- Lachaud, J. & Mansour, N.N., "Porous-Material Analysis Toolbox Based on
  OpenFOAM and Applications (PATO)," *J. Thermophysics & Heat Transfer* 28(2),
  2014.
- Milos, F.S. & Chen, Y.-K., "Two-Dimensional Ablation, Thermal Response, and
  Sizing Program for Pyrolyzing Ablators," *J. Spacecraft & Rockets* 46(6), 2009.
- Laub, B. & Venkatapathy, E., "Thermal Protection System Technology and Facility
  Needs for Demanding Future Planetary Missions," IPPW, 2003.
- TACOT (Theoretical Ablative Composite for Open Testing) — open material-response
  comparison surrogate (PATO/FIAT/CHAR).

**V&V & UQ.**
- Roache, P.J., *Verification and Validation in Computational Science and
  Engineering*, Hermosa, 1998 (GCI).
- ASME V&V 20-2009; AIAA G-077-1998.
- NASA-STD-7009B (March 2024) — Standard for Models and Simulations.
- FIRE II (1965), RAM-C II (1970), Apollo 4 / AS-501, Stardust SRC public
  reconstructions (Olynick/Chen, *J. Spacecraft* 1999; Trumble et al.).

**OpenBMP upstream anchors.**
- `docs/parity/00-overview.md`, `docs/parity/13-agent-execution-playbook.md`,
  `docs/parity/03-aerodynamics-database-and-cfd-coupling.md`,
  `docs/hypersonic-extensions.md`, `docs/verification.md`,
  `docs/data-provenance.md`, `docs/safety-boundaries.md`,
  `docs/standards-posture.md`.
