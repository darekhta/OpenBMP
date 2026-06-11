# Structural Dynamics, Loads, Slosh & POGO

**Status:** `experimental` (design intent; no code shipped by this document).
**Audience:** the engineer or LLM agent who will implement the work packages.
**One-line summary:** bring OpenBMP from a single hand-tuned bending mode and an
all-fluid slosh pendulum to a multi-mode modal block, mass-correct slosh analogs,
ingested reduced FEM models with line-load recovery (CLA/LTM/OTM), and a
Rubin/Oppenheim POGO + control-structure-interaction (CSI) margin layer — all
forward-only, byte-deterministic, and honest about the GVT/flight-correlation
ceiling.

> Read `00-overview.md` (parity definition, invariants, DAG) and
> `13-agent-execution-playbook.md` (template, backlog schema, gate set) before
> this document. This doc follows the `13` §4 template section-for-section and
> ends with a machine-readable `WP-02.t` backlog.

---

## 1. Parity target & ceiling

### 1.1 Target capability

OpenBMP today carries **one** hand-entered first lateral bending mode
(`crates/openbmp-vehicle/src/structural.rs`) coupled to the rate gyro, and an
**all-fluid** equivalent-pendulum slosh model
(`crates/openbmp-vehicle/src/tank/equivalent_pendulum.rs`). Production
launch-vehicle structural/loads practice at the reference organizations is a
qualitatively larger thing: a FEM-derived **reduced modal model** (Craig-Bampton
component-mode synthesis) carrying tens to hundreds of modes, a **coupled loads
analysis** (CLA) that integrates the modal model through a library of flight
events and recovers physical **line loads** (axial/shear/bending vs station) via
load/output transformation matrices (LTM/OTM), a **mass-correct slosh** model
(Abramson/Dodge fixed mass + N slosh masses), **POGO** longitudinal closed-loop
stability (Rubin/Oppenheim, NASA SP-8055), and **CSI** bending gain/phase-margin
design with notch filters.

The parity target — *capability/method/architecture parity within posture*
(`00` §1) — is **approaching** (the `00` gap-ledger verdict for this dimension):

- a **multi-mode lateral modal block** generalizing the single `BendingMode`,
  with per-mode `(ω_j, ζ_j, m_j, slope_engine_j, slope_gyro_j)` and the same
  symplectic, locked-operand-order integration;
- a **mass-correct slosh analog** implementing the Abramson/Dodge fixed-mass +
  N-slosh-mass split, fixing the rigid-inertia error of the current all-fluid
  pendulum;
- an **ingestion path** for an external reduced modal model
  (Nastran/CalculiX OP2/OP4/CSV — frequencies, mode shapes at instrumented
  stations, modal masses) into the modal block, with provenance + UQ;
- **CLA load recovery** (LTM/OTM with the mode-acceleration correction)
  producing station line-load time histories and per-event envelopes from a
  **flight-event forcing library** (liftoff twang, max-Q gust, ignition,
  separation, MECO);
- a native **reduced FEM + Craig-Bampton** path (beam/shell FEM + sparse
  shift-invert eigensolve + CB reduction + generalized-α modal transient) for
  self-contained CLA;
- a **POGO** layer (1-D feedline transmission lines + pump cavitation
  compliance/mass-flow-gain + closed-loop complex-eigenvalue/Nyquist stability +
  accumulator detuning) and a **CSI margin** tool (frequency-domain gain/phase
  margins over the LTV trajectory + notch/lead-lag design).

The FFRF flexible-body *dynamics* substrate (a stage as a deformable body with
rigid-flex inertia coupling in the spatial-vector tree) is owned by
`01-flexible-multibody-dynamics.md`. **This document owns the modal *content*,
the loads/CLA chain, slosh analogs, and the POGO/CSI stability analyses** that
consume that substrate (or, at lower tiers, run as standalone modal blocks
behind the existing rigid-body kernel). The two docs share the Craig-Bampton
reduction and the ingested modal asset; the boundary is stated in §7.

### 1.2 The parity ceiling (honest)

Four things **cannot** be matched in an open repo without proprietary data or a
hardware program. Each is paired with the credible open substitute:

1. **Flight-correlated as-built modal models.** Production sims use
   GVT-correlated, FEM-updated models of the *actual* vehicle — test-anchored
   frequencies, joint stiffnesses, damping — derived from ground vibration test
   (modal survey) campaigns OpenBMP will never run. **Open substitute:** validate
   the *method* (CB reduction, MAC/cross-orthogonality, FE-updating, transient
   integration) against published modal benchmarks (GARTEUR SM-AG19 / DLR AIRMOD)
   and against CalculiX code-to-code; state plainly that the numbers are
   benchmark/synthetic, not a specific vehicle. The "GVT correlation method" is
   validated against *published modal data*, **not** a real vehicle.

2. **POGO pump cavitation compliance `C_b` and mass-flow-gain factor `M_b`.**
   The two parameters that actually decide POGO stability come from proprietary
   pump test rigs / multiphase CFD. **Open substitute:** published Titan/Saturn
   parameter ranges (SP-8055, Rubin 1966/1993) entered as documented scenario
   inputs with an explicit dispersion box; optional offline VOF (OpenFOAM/
   Basilisk) to bound them; validate the *unstable-frequency band and the
   accumulator detuning trend*, never a specific margin number. POGO is therefore
   capped at a **method-level** claim.

3. **Flight loads validation.** Matching measured liftoff/max-Q/separation line
   loads requires flight telemetry and certified loads databases that are
   proprietary. **Open substitute:** cross-validate trends/magnitudes against
   published flight-loads case studies and (LOCAL/casual only) the scraped
   Falcon-9 webcast telemetry as an external-validity gross-error check
   (`docs/external-telemetry-validation.md`) — never a flight input, never a
   tuning target.

4. **Nonlinear / large-amplitude / low-g slosh.** The linear mechanical analog
   is structurally incapable of swirl, rotary slosh, and low-g reorientation.
   **Open substitute:** calibrate analog coefficients from offline VOF and
   *state the linear-regime limitation* — exactly as the current freefall
   capillary term already does.

**Ceiling one-liner:** OpenBMP can reach demonstrably-correct, benchmark-verified
*method* fidelity for structural dynamics / loads / slosh / POGO, but **cannot**
claim flight-certified loads parity or a vehicle-specific POGO margin; the
GVT-correlation, line-load, and cavitation-parameter validations are against
published/synthetic data only.

---

## 2. Current state in source

Verified by reading the files below (not summaries).

| Concern | File(s) | Maturity |
|---|---|---|
| Single first bending mode | `crates/openbmp-vehicle/src/structural.rs` | One SDOF per transverse axis (`q_x`, `q_y`); symplectic-Euler, locked operand order, no FMA; gyro slope-rate pickup; **lever-scaled** reaction moment `-m_q·slope_engine·q̈`. Explicitly first-increment (module doc: "the full distributed-load coupling is a follow-up"). |
| Runner-side bending rack | `crates/openbmp-runner/src/structural.rs` | `StructuralRack::{Inactive,Active}`; built from `[vehicle.bending]`; one-step-lag body-accel driver; exposes `gyro_pickup_rad_s()` (wired to FC bridge) and `reaction_moment_body_n_m()` (exposed but the rack doc says "wired into the dynamics in a follow-up"). `Inactive` ⇒ byte-identical to a rigid vehicle. |
| Scenario block | `crates/openbmp-scenario/src/document.rs` (`BendingConfig`, ~L2288) | `[vehicle.bending]`: `frequency_hz`, `damping_ratio`, `modal_mass_kg`, `slope_at_engine`, `slope_at_gyro`; `deny_unknown_fields`; fail-closed validation. Single mode only. |
| FC-side gyro pickup add | `crates/openbmp-runner/src/fc_bridge.rs` (~L503–514) | The bridge adds `gyro_pickup_rad_s` onto `state.angular_velocity.vector` before forming the sensed body rate the FC reads — the existing flex→sensor boundary. |
| Bending notch filter | `crates/openbmp-fc/src/filters.rs` | `Biquad` (Direct-Form-II Transposed): `notch`, `lowpass_first_order`, `butterworth_lowpass_second_order`, `magnitude_at_hz`. Pure arithmetic, no time source (lockstep-clock safe). `NotchConfig{center_hz,bandwidth_hz,depth_db}`. Consumed by `[fc.autopilot_params].gyro_notch` (`crates/openbmp-fc/src/autopilot.rs`). |
| Slosh: all-fluid pendulum | `crates/openbmp-vehicle/src/tank/equivalent_pendulum.rs` | Abramson SP-106 cylindrical first mode: `ω_n² = (g_eff/a)·ξ₁·tanh(ξ₁h/a)`, `L = a/(ξ₁·tanh(...))`, `ξ₁=1.8411837813406593`; two tilt angles; small-angle linearized; freefall capillary term; semi-implicit Euler. **Treats all fluid as slosh mass** (`m_slosh=m_total`) — module doc admits this mis-states the rigid mass/inertia split (Abramson Table 7.1). |
| Slosh variants | `crates/openbmp-vehicle/src/tank/{baffled_pendulum,equivalent_spring_mass,rigid_liquid}.rs`, `mod.rs` | `MovingMassModel` trait (`drain/step/mass_contribution/reaction_body/fluid_remaining_kg`); `BaffleModel` (scalar `damping_increment_zeta`, Abramson §7.4 linear-in-area-ratio approximation, full Eq. 7-46 deferred); `Tank`, `TankGeometry{Cylinder,Sphere,EllipsoidTextbook}`, `PropellantSpec`; `point_mass_inertia_about_origin` helper with locked operand order. |
| Frontier note | `docs/launch-vehicle-fidelity-frontier.md` | Records the remaining gaps and the integration seams (the reaction-moment wiring, slosh-coupled closure, POGO) the WPs below pick up. |

**Net baseline to regress against:** a Tier-0 capability — one decoupled bending
SDOF wired to the gyro + lever-scaled reaction, one all-fluid pendulum (+ baffled
/ spring-mass / rigid variants), symplectic-Euler, byte-deterministic,
off-by-default. No POGO, no FEM-derived modes, no line-load recovery, no slosh
mass split, no CSI margin tooling.

---

## 3. Target architecture

### 3.1 New / changed crates

```text
CHANGED:
  openbmp-vehicle   L2  + ModalBlock (N-mode lateral+longitudinal), corrected
                        slosh analog (Abramson fixed-mass + N slosh masses),
                        load-recovery (LTM/OTM) types
  openbmp-runner    L7  + ModalRack (generalizes StructuralRack), CLA event
                        forcing library, line-load telemetry channels
  openbmp-fc        L4  + CSI margin helpers stay FC-portable: notch/lead-lag
                        design + Bode/Nyquist evaluation over a frozen-time
                        linearization (NO simulator dep)  ← portability lock
  openbmp-scenario  L?  + [vehicle.modes], [vehicle.slosh.*] mass-split fields,
                        [loads.events], [pogo], [csi] blocks (deny_unknown_fields)
  openbmp-multibody L2  shares the Craig-Bampton reduction with doc 01 (FFRF)

NEW (proposed; placement justified in the introducing WP):
  openbmp-structdyn L2  modal transient (Newmark/generalized-α), CB reduction,
                        beam/shell FEM, LTM/OTM recovery, POGO linear-systems
                        (transmission-line + pump linearization + complex-eig)
                        — a pure-Rust deterministic numerics crate, no up-layer
                        edges, depends only on openbmp-core + linalg (faer/nalgebra)
  openbmp-modelio   L1  ingestion: OP2/OP4/CSV modal-model + OTM parser → a
                        provenance-pinned ModalAsset (shared by doc 01 ingest)
```

> The `openbmp-structdyn` boundary is proposed in `WP-02.3-a` (CB + transient)
> and reviewed before the FEM lands; the `openbmp-modelio` boundary in
> `WP-02.2-a` (ingest). Neither may gain an up-layer or FC edge (the FC CSI
> helpers live in `openbmp-fc` and consume only plain coefficient arrays).

### 3.2 Rust trait & data-structure surfaces

The single `BendingMode` is generalized to a **modal block** while preserving its
determinism contract (symplectic, locked operand order, no FMA). The existing
`BendingMode` becomes the `N=1` special case (and the existing scenario stays
byte-identical because `[vehicle.bending]` keeps mapping to a one-mode block —
see §4).

```rust
// openbmp-vehicle::structural — multi-mode lateral (Tier 1)

/// One reduced structural mode's modal parameters (lateral or longitudinal).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ModeParams {
    pub omega_rad_s: f64,     // ω_j  (> 0)
    pub zeta: f64,            // ζ_j  (≥ 0)
    pub modal_mass_kg: f64,   // m_j  (> 0, mass-normalized or generalized)
    pub slope_at_engine: f64, // φ'_j(x_engine) — lateral forcing lever
    pub slope_at_gyro: f64,   // φ'_j(x_gyro)   — rate-gyro pickup (slope)
    pub shape_at_accel: f64,  // φ_j(x_accel)   — accelerometer pickup (displacement)
}

/// N-mode lateral modal block. Per axis (x,y) carries N independent SDOF
/// coordinates integrated with the same symplectic, locked-operand-order
/// scheme as the legacy single mode.
#[derive(Clone, Debug, PartialEq)]
pub struct ModalBlock {
    modes: Vec<ModeParams>,        // declaration order is the operand order
    q_x: Vec<f64>, q_dot_x: Vec<f64>, last_q_ddot_x: Vec<f64>,
    q_y: Vec<f64>, q_dot_y: Vec<f64>, last_q_ddot_y: Vec<f64>,
}

impl ModalBlock {
    pub fn new(modes: Vec<ModeParams>) -> Result<Self, ModalError>;
    /// Advance all modes one sub-step under body-frame lateral specific force.
    /// Iterates modes in declaration order; per-mode q̈ = forcing − damping −
    /// restoring; θ̇ then θ update (semi-implicit). No FMA.
    pub fn step(&mut self, accel_body_m_s2: Vector3<f64>, dt: Duration)
        -> Result<(), ModalError>;
    /// Σ_j slope_gyro_j · q̇_j mapped to body pitch/yaw (sum in mode order).
    pub fn gyro_pickup_rad_s(&self) -> Vector3<f64>;
    /// Σ_j shape_accel_j · q̈_j mapped to body axes (accelerometer pickup).
    pub fn accel_pickup_m_s2(&self) -> Vector3<f64>;
    /// Σ_j −m_j · slope_engine_j · q̈_j (lever reaction, mode-summed).
    pub fn reaction_moment_body_n_m(&self) -> Vector3<f64>;
    pub fn modal_state(&self) -> &[ModalCoord]; // for load recovery + telemetry
}
```

```rust
// openbmp-structdyn::recovery — line-load recovery (Tier 2/3, LTM/OTM)

/// A precomputed Data/Load Recovery Matrix mapping modal coords + modal
/// accelerations to physical engineering responses (forces/moments/accels).
#[derive(Clone, Debug)]
pub struct RecoveryMatrix {
    a_disp: DMatrix<f64>,  // A_d : y = A_d q + A_a q̈
    a_accel: DMatrix<f64>, // A_a
    /// Optional mode-acceleration residual K⁻¹ term restoring truncated-mode
    /// quasi-static content (Flanigan / Cornwell-Craig-Johnson).
    mode_accel_residual: Option<ModeAccelResidual>,
    output_labels: Vec<OutputId>, // station + quantity (axial/shear/Mbend/accel)
}

impl RecoveryMatrix {
    /// y(t) = A_d q(t) + A_a q̈(t) [+ mode-acceleration correction].
    /// Locked operand order: dense GEMV in row order, residual added last.
    pub fn recover(&self, q: &[f64], q_ddot: &[f64]) -> Vec<f64>;
}

/// Per-station net section line load (D'Alembert free-body cut).
#[derive(Clone, Copy, Debug)]
pub struct SectionLoad {
    pub station_m: f64,
    pub axial_n: f64,
    pub shear_n: f64,
    pub bending_moment_n_m: f64,
}
```

```rust
// openbmp-structdyn::transient — CLA modal transient (Tier 3)

/// Implicit modal transient integrator. Newmark-β (β=1/4,γ=1/2) or
/// generalized-α (Chung–Hulbert, ρ_∞ documented for bit-reproducibility).
pub struct ModalTransient {
    m: DMatrix<f64>, c: DMatrix<f64>, k: DMatrix<f64>, // reduced (diagonal in modal block)
    dt_s: f64,
    k_eff_factored: CholeskyFactored,                  // precomputed (constant dt)
    scheme: TransientScheme,                           // Newmark | GeneralizedAlpha{rho_inf}
}

impl ModalTransient {
    /// One implicit step: solve K_eff q_{n+1} = RHS(f_{n+1}, q_n, q̇_n, q̈_n).
    pub fn step(&mut self, f_next: &[f64]) -> Result<&[f64], TransientError>;
}

/// CLA flight-event forcing library (modal forcing f(t) = Φᵀ F_distributed).
pub trait EventForcing: Send + Sync {
    /// Modal forcing vector at time t for this event.
    fn modal_force(&self, t_s: f64, ctx: &ForcingContext) -> Vec<f64>;
    fn label(&self) -> EventId; // Liftoff | MaxQGust | Ignition | Separation | MECO
}
```

```rust
// openbmp-structdyn::cb — Craig–Bampton reduction (Tier 3)
pub struct CraigBampton {
    /// Static constraint modes Φ_ic = −K_ii⁻¹ K_ib (one per boundary DOF).
    /// Fixed-interface normal modes Φ_ik (lowest k of K_ii φ = M_ii φ Λ).
}
impl CraigBampton {
    /// T = [[I,0],[Φ_ic, Φ_ik]]; M̂ = TᵀMT, K̂ = TᵀKT.
    pub fn reduce(m: &SparseMatrix, k: &SparseMatrix, boundary: &[Dof], n_modes: usize)
        -> Result<ReducedModel, CbError>;
}
```

```rust
// openbmp-structdyn::pogo — POGO longitudinal stability (Tier 4)

/// Pump linearization — the two dominant POGO parameters.
/// Entered as scenario inputs with a documented dispersion box (never fitted).
#[derive(Clone, Copy, Debug)]
pub struct PumpCavitation {
    pub cavitation_compliance_m3_pa: f64, // C_b = −dV_cav/dp_inlet
    pub mass_flow_gain_kg_s_pa: f64,      // M_b = −dṁ/dp_inlet
    pub pressure_rise_gain: f64,          // pump head sensitivity
}

/// 1-D feedline as a transmission line (lumped L-C-R or distributed cosh/sinh).
#[derive(Clone, Copy, Debug)]
pub struct FeedlineSegment {
    pub inertance_kg_m4: f64,   // L = ρℓ/A
    pub compliance_m3_pa: f64,  // C = V/(ρc²)
    pub resistance: f64,        // R
    pub length_m: f64, pub area_m2: f64, pub wave_speed_m_s: f64, // c (distributed form)
}

/// Closed-loop POGO model: longitudinal modal A_s ⊕ feed system ⊕ pump ⊕ engine.
pub struct PogoLoop { /* assembled impedance / state-space */ }
impl PogoLoop {
    /// Complex eigenvalues of the closed-loop matrix; instability = any λ in RHP.
    pub fn closed_loop_eigenvalues(&self) -> Vec<Complex<f64>>;
    /// Open-loop gain L(jω) for a Nyquist / gain-phase-margin assessment.
    pub fn open_loop_response(&self, omega_rad_s: f64) -> Complex<f64>;
    /// Accumulator detuning: add C_acc at the pump inlet, recompute margin.
    pub fn with_accumulator(&self, compliance_m3_pa: f64) -> Self;
}
```

```rust
// openbmp-fc::csi — control-structure-interaction margins (FC-PORTABLE)
// Consumes only plain coefficient arrays (frozen-time linearized plant);
// NO openbmp-sim / runner / scenario edge — FC portability lock preserved.

/// Frozen-time linearized open-loop transfer L(jω) = C(jωI − A)⁻¹B over the
/// rigid + flex + actuator + sensor + filter chain at one flight point.
pub struct FrozenPlant { /* A,B,C,D dense */ }
impl FrozenPlant {
    pub fn open_loop_response(&self, omega_rad_s: f64) -> Complex<f64>;
    /// (gain_margin_db, phase_margin_deg) from the L(jω) sweep.
    pub fn stability_margins(&self) -> StabilityMargins;
}
/// Bending notch H(s)=(s²+2ζ_z ω s+ω²)/(s²+2ζ_p ω s+ω²), ζ_z<ζ_p (reuses Biquad).
pub fn design_bending_notch(omega_rad_s: f64, zeta_zero: f64, zeta_pole: f64,
                            sample_rate_hz: f64) -> Result<Biquad, &'static str>;
```

The slosh correction reuses the existing `MovingMassModel` trait. A new
implementation `AbramsonMultiMass` (or a corrected `EquivalentPendulum` mode)
carries the fixed mass + N slosh masses; the trait surface
(`step/mass_contribution/reaction_body/drain/fluid_remaining_kg`) is unchanged,
so the runner-side tank rack consumes it with no plumbing change.

```rust
// openbmp-vehicle::tank — mass-correct slosh analog (Tier 1)

/// Abramson/Dodge fixed mass + N slosh masses (cylinder; sphere/ellipsoid via
/// Dodge coefficient tables). Fixes the all-fluid rigid-inertia error.
#[derive(Clone, Debug)]
pub struct AbramsonMultiMass {
    fixed_mass_kg: f64, fixed_height_m: f64,           // m_0, non-sloshing
    slosh: Vec<SloshMass>,                             // m_n on SDOF springs/pendulums
    geometry: TankGeometry, propellant: PropellantSpec, fill_fraction: f64,
}
struct SloshMass { m_kg: f64, omega_rad_s: f64, zeta: f64, attach_height_m: f64,
                   theta_x: f64, theta_y: f64, /* + rates, last accel */ }
impl MovingMassModel for AbramsonMultiMass { /* … */ }
```

### 3.3 The math (named formulations)

**(M1) Craig-Bampton fixed-interface CMS reduction.** Partition the FEM
mass/stiffness into boundary (`b`) and interior (`i`) DOF. The transformation is
`x = T q_cb`, `T = [[I, 0], [Φ_ic, Φ_ik]]`, where the **static constraint modes**
are `Φ_ic = −K_ii⁻¹ K_ib` (one per boundary DOF) and the **fixed-interface normal
modes** `Φ_ik` are the lowest `k` eigenvectors of the generalized problem
`K_ii Φ = M_ii Φ Λ` (boundary fixed). Reduced matrices `M̂ = Tᵀ M T`,
`K̂ = Tᵀ K T`; the modal block of `K̂` is `diag(ω_k²)`; modal damping
`Ĉ = diag(2 ζ_j ω_j)` (0.5–3% structural). Assembly enforces interface DOF
compatibility (primal/shared boundary DOF). *(Craig & Bampton 1968;
Craig & Kurdila 2006, ch. 19.)*

**(M2) Modal transient (CLA).** Integrate `M q̈ + C q̇ + K q = f(t)` on the
reduced system. Implicit unconditionally-stable schemes: **Newmark-β**
(`β=1/4, γ=1/2`, average acceleration) with effective stiffness
`K_eff = K + (γ/(β·dt)) C + (1/(β·dt²)) M`, solving
`K_eff q_{n+1} = f_{n+1} + M[(1/(β·dt²))q_n + (1/(β·dt))q̇_n + (1/(2β)−1)q̈_n] + C[…]`
then updating `q̇, q̈`; or **generalized-α** (Chung-Hulbert) with controllable
high-frequency numerical dissipation `ρ_∞ ∈ [0,1]` to kill spurious
truncated-mode content. For a **diagonal** modal system the per-mode SDOF reduces
to the exact-exponential / semi-implicit propagation OpenBMP already uses — the
Tier-1 modal block is exactly the current `BendingMode` scheme replicated per
mode; the Tier-3 implicit branch is required only when the reduced `M̂`/`Ĉ`
couple modes. The generalized-α `ρ_∞` is fixed and documented for
bit-reproducibility. *(Newmark 1959; Chung & Hulbert 1993; Wijker 2008.)*

**(M3) Load recovery (LTM/OTM, mode-acceleration).**
`y(t) = A_d q(t) + A_a q̈(t)` with precomputed Data/Load Recovery Matrices.
**Mode-displacement** route: physical DOF `x = Φ q`, element forces
`F_e = K_e x_e`, net section load by summing internal forces across a cut. The
**mode-acceleration correction** restores the quasi-static contribution of
truncated modes, `x ≈ K⁻¹(f − M·Φ q̈_modal)`, materially improving stress/
line-load accuracy for a truncated basis (a one-line addition, in from the
start). **Free-body / interface-force** route: at a station cut, the net line
load = D'Alembert sum over nodes outboard of the cut of inertial + applied loads,
`line_load(station,t) = S·(M_full(Φ q̈) + applied(t))`, with `S` a precomputed
per-station summation matrix. *(NASA-HDBK-7005; Flanigan; Cornwell-Craig-Johnson
1983; Wijker 2008.)*

**(M4) Mass-correct slosh analog (Abramson/Dodge).** Cylindrical tank, mode `n`:
`ω_n² = (g_eff/a)·ξ_n·tanh(ξ_n·h/a)` where `ξ_n` is the n-th root of `J_1'(x)=0`
(`ξ_1=1.8412`, `ξ_2=5.3314`, `ξ_3=8.5363`). Slosh-mass fraction
`m_n/m_f = tanh(ξ_n h/a) / [(ξ_n h/a)(ξ_n² − 1)]`; **fixed mass**
`m_0 = m_f − Σ_n m_n`. Hinge/attach heights and the fixed-mass height from the
Abramson closed-form expressions in `h/a`. Each slosh mass obeys an SDOF
`ẍ_n + 2ζ_n ω_n ẋ_n + ω_n² x_n = −a_lateral`, reaction `−m_n ẍ_n` applied at the
analog attach height; the fixed mass `m_0` contributes only rigid inertia/force.
Baffle damping from Miles' ring-baffle correlation (Abramson §5/§7.4); bare-wall
`ζ ~ (ν/(ω a²))^{1/2}`. Spherical/ellipsoidal tanks use Dodge's tabulated
coefficients. **The win-per-LOC is fixing `m_0` vs `m_n`** — the current
all-fluid pendulum mis-states the rigid inertia. *(Abramson SP-106 1966; Dodge
SwRI 2000; Bauer TR R-187 1964; Ibrahim 2005.)*

**(M5) POGO closed-loop stability (Rubin / Oppenheim / SP-8055).** Build a linear
feedback model around a flight condition. **Structure:** the reduced *axial*
modal subset (longitudinal CB modes), state-space `A_s` with thrust input at the
engine station and pressure/acceleration outputs at the tank bottom & pump inlet.
**Feed system** as 1-D fluid transmission lines: per segment inertance
`L = ρℓ/A`, compliance `C = V/(ρc²)`; line transfer
`[[cosh(γℓ), Z_c sinh(γℓ)], [sinh(γℓ)/Z_c, cosh(γℓ)]]`, `γ = s/c`,
`Z_c = ρc/A` (lumped L-C-R for short lines). **Pump:** cavitation compliance
`C_b = −dV_cav/dp_inlet`, mass-flow-gain `M_b = −dṁ/dp_inlet`, plus pressure-rise
gain. **Engine:** thrust = `k_thrust·` chamber-pressure response to inlet flow.
Close the loop as `Z(s) X(s) = 0` and test instability by (a) complex eigenvalues
of the closed-loop matrix crossing into the RHP, or (b) Nyquist of the open-loop
pump-structure-feedline gain. **Accumulator:** add a gas/spring compliance
`C_acc` at the pump inlet — it detunes the feedline resonance away from the
structural longitudinal mode, raising margin. SP-8055 mandates ≥ +6 dB gain /
adequate phase margin across the dispersion box. *(Rubin SP-8055 1970; Rubin
1966; Oppenheim & Rubin 1993; Brennen 1994.)* **This is the consumer side of the
feed-system feedback model in `05-propulsion-high-fidelity.md` — see §7.**

**(M6) Control-structure interaction (CSI).** Augment the rigid-body plant with
the modal states. Sensor outputs: `gyro_rate = ω_rigid + Σ_j φ'_j(x_gyro) q̇_j`;
`accel = a_rigid + Σ_j φ_j(x_accel) q̈_j`. Actuator (TVC/gimbal) enters modal
forcing `f_j = φ_j(x_engine)·T_gimbal`. Form open-loop
`L(s) = C(sI−A)⁻¹B` over rigid + flex + actuator + sensor + filter. Notch design
`H_notch(s) = (s² + 2ζ_z ω_n s + ω_n²)/(s² + 2ζ_p ω_n s + ω_n²)` (`ζ_z < ζ_p`)
for **gain stabilization**; lead/lag for **phase stabilization** of the first
mode. Compute gain/phase margins from the Bode/Nyquist of `L(s)`; the classic
design requirement is **≥ 6 dB gain margin, ≥ 30° phase margin per mode**,
evaluated across the trajectory (frozen-time / LTV gain scheduling at a sequence
of flight points). The time-domain check runs the augmented modal model through
the existing flight controller in the loop — which the FC-bridge SIL already
enables for one mode. *(Greensite 1967-70; Wie 2008 ch.7; Orr et al. SLS bending
filter, NTRS.)*

### 3.4 Fidelity tiers `T0…T4`

- **T0 (current).** Single first-bending SDOF wired to gyro pickup + lever-scaled
  reaction; single all-fluid equivalent pendulum (+ baffled / spring-mass /
  rigid variants); symplectic-Euler, byte-deterministic, off-by-default. No
  POGO, no FEM-derived modes, no load recovery, no slosh mass split.

- **T1 — Multi-mode lateral + corrected slosh analog.** Replace the single
  bending mode with an N-mode `ModalBlock` (still hand/spec-entered
  `ω_j, ζ_j, m_j`, mode-shape slopes; same per-mode symplectic scheme). Replace
  "all fluid swings" with the Abramson fixed-mass + N-slosh-mass split so rigid
  inertia is correct, and wire the reaction moment into the dynamics (closing the
  `StructuralRack` "follow-up"). **Earns:** `validated-toy` (Abramson closed-form
  mass fractions + period/energy property tests).

- **T2 — Ingest an external reduced modal model + line-load recovery.** Read a
  Nastran/CalculiX OP4/OP2 or CSV export (frequencies, mode shapes at
  instrumented stations, modal mass) into the modal block; add mode-acceleration
  LTM/OTM recovery producing station line loads (axial/shear/bending vs station).
  The sim consumes a real FEM without building one. **Earns:** `research`
  (code-to-code vs the source FEM's SOL103/SOL112).

- **T3 — Native reduced FEM + Craig-Bampton + transient CLA event library.** A
  Rust beam/shell FEM, sparse shift-invert Lanczos eigensolve, CB reduction,
  generalized-α modal transient, and a forcing-event library (liftoff twang,
  max-Q gust, ignition, separation, MECO). Self-contained CLA producing line-load
  envelopes per event. **Earns:** `research` (published modal benchmarks GARTEUR
  SM-AG19 / AIRMOD + MMS + code-to-code vs CalculiX).

- **T4 — POGO longitudinal stability + CSI margin analysis.** 1-D feedline
  transmission lines + pump cavitation-compliance/mass-flow-gain; close the
  structural-propulsion loop; complex-eigenvalue / Nyquist stability with
  accumulator sizing; frequency-domain bending gain/phase-margin tool over the
  LTV trajectory + notch design. Highest fidelity, highest data dependence.
  **Earns:** `research` at the **method level only** (Titan/Saturn published POGO
  cases + SP-8055 margin criteria, qualitative); per the ceiling, *no
  vehicle-specific margin claim*.

---

## 4. Invariant preservation

Concretely, for this dimension (`00` §3, `13` §3):

1. **Byte-determinism.** Every modal/slosh integrator advances state by
   locked-operand-order weighted sums (per-mode `q̈ = forcing − damping −
   restoring`; `q̇` then `q` update), **never** `f64::mul_add`, never wall-clock,
   never RNG, never unordered iteration. The `ModalBlock` iterates modes in
   **declaration order** (the operand order is the `Vec<ModeParams>` order); the
   gyro/accel/reaction sums accumulate in that same order. Dense linear algebra
   in `openbmp-structdyn` (CB reductions `M̂ = TᵀMT`, `K_eff` factorization,
   recovery GEMV, POGO/CSI complex-eigen) uses faer/nalgebra **without** FMA and
   with a fixed reduction order; the generalized-α `ρ_∞` and the Newmark
   `(β,γ)` are pinned per scenario (changing them changes bytes and is a
   documented per-scenario lock, exactly like `slosh_substeps`). MC dispersions
   of slosh/modal/pump parameters derive from
   `openbmp-core::DeterministicRng` with a domain-separated
   `(seed, step, id, component)` tuple.

2. **Byte-stable-by-default.** Every new capability is **off by default** and
   gated by an explicit scenario block, following the `[vehicle.bending]`
   precedent: `[vehicle.modes]` (multi-mode), `[vehicle.slosh.mass_split]`
   (Abramson split), `[loads.events]` (CLA), `[pogo]`, `[csi]`. **The legacy
   `[vehicle.bending]` keeps mapping to a one-mode `ModalBlock`** with identical
   arithmetic, so all existing scenarios and golden archives stay byte-identical
   until a scenario opts into the new blocks. The corrected slosh model is a *new*
   `MovingMassModel` selected by config; the existing `EquivalentPendulum` is
   untouched, so its goldens are preserved.

3. **FC hardware-portability lock.** `openbmp-fc` gains only the **CSI margin
   helpers** (`FrozenPlant`, `design_bending_notch`) which consume plain
   coefficient arrays (a frozen-time linearized plant + filter coefficients) and
   depend only on hardware-portable crates. **No** edge to `openbmp-sim`,
   `openbmp-runner`, `openbmp-scenario`, `openbmp-telemetry`, `openbmp-bridge`,
   or the new `openbmp-structdyn`. The notch reuses the existing `filters::Biquad`
   (already lockstep-clock safe, no time source). The simulator-side modal/POGO
   state reaches the FC only through the existing FC-bridge / sensor boundary
   (`fc_bridge.rs` adds `gyro_pickup_rad_s` onto the sensed body rate; the
   accelerometer pickup joins it the same way). The `fc_dependency_tripwire.rs`
   stays green.

4. **Lockstep-clock & no-hot-path-allocation.** The CSI helpers in `openbmp-fc`
   read no clock; the margin sweep is a pure function of frozen coefficients. The
   modal/slosh racks pre-size their `Vec`s at build time (mode count is known from
   the scenario), so the per-tick `step` path does not allocate.

5. **Four-pillar provenance.** Ingested modal models (OP2/OP4/CSV), POGO
   pump-parameter dispersion boxes, and benchmark modal data land under
   `data/<thing>/` with a sibling `provenance.md` (SHA-256 pinned, fail-closed at
   load). The new source-of-truth files (e.g. the benchmark modal frequencies for
   GARTEUR/AIRMOD, the Abramson `ξ_n` roots if promoted out of code) are added to
   the `inline_data_tripwire.rs` allow-list **in the same PR**. No inline TOML in
   `*.rs`. **Authoring lock:** this dimension never needs the WGS84 numerals;
   gravity enters only symbolically as `g_eff` / `GM_earth` from the environment
   crate (`08-environment-gravity-and-frames.md`).

6. **Synthetic / public-data-only.** All modal/slosh/POGO parameters checked in
   are synthetic, textbook-derived, or openly published (Abramson/Dodge tables,
   GARTEUR/AIRMOD, SP-8055 ranges) with provenance + license status. No
   fielded-vehicle modal model, no proprietary pump map, no certified loads
   database. Falcon-9 telemetry cross-checks stay LOCAL and uncommitted.

7. **Forward-only mechanical locks.** All of CLA/POGO/CSI is forward transient/
   eigen analysis of the vehicle's own response. Any *objective* added to these
   models (e.g. minimize bending load at max-Q, maximize POGO margin, minimize
   slosh-induced attitude error) must be **vehicle-intrinsic** (loads, margins,
   modal damping) with **no** range/CEP/ground-aimpoint field — mirroring the
   propulsion-models "vehicle-intrinsic Δv objective" precedent. Load-recovery
   outputs are scoped to **structural quantities** (station line loads, interface
   forces, margins) — never payload-effect or terminal-accuracy quantities. See
   §6.

8. **Validation labels.** Each tier declares exactly one label (§3.4): T1
   `validated-toy`; T2/T3 `research`; T4 `research` *method-level only*. Every
   numeric claim carries a tolerance-table TOML. No artifact claims
   `flight-qualified`/`certified`.

9. **Workspace lints & fail-closed.** `missing_docs`/`unsafe_code`/`float_cmp`/
   `unused_must_use` stay `deny`; the modal/slosh/POGO constructors return
   `Result` (reject non-finite/out-of-range params) on every scenario-reachable
   path; the scenario blocks use `deny_unknown_fields`.

10. **Requirements traceability.** Each WP adds `requirements.toml` entries with
    verification evidence; `check_requirements_traceability.py` stays green.

---

## 5. V&V plan

Methods follow the five-layer ladder (`docs/verification.md`): MMS code
verification → model verification vs closed-form/correlations → code-to-code vs
open solvers → public benchmarks → UQ reporting. Tolerances below are the
per-tier acceptance bars; each becomes a tolerance-table TOML.

| # | Case | Method | Tier | Tolerance | Label earned |
|---|---|---|---|---|---|
| V1 | Abramson cylindrical `ω_n`, `m_n/m_f`, `m_0`, attach heights vs fill | model-vs-closed-form | T1 | `ω_n`, fractions within **1%** of SP-106/Dodge tables | `validated-toy` |
| V2 | Slosh period + ≤1% energy conservation over 100 oscillations | analytic property test | T1 | energy drift **< 1%**, period within **0.5%** | `validated-toy` |
| V3 | Multi-mode block reduces to legacy single-mode (N=1) byte-for-byte | regression | T1 | **byte-identical** golden | `validated-toy` |
| V4 | Momentum conservation of corrected slosh across coast (vs RCS-arrested 4.8→<1 rad/s record) | analytic + cross-check | T1 | linear+angular momentum continuity to **1e-9** | `validated-toy` |
| V5 | MMS for the modal transient: impose `q_j(t)=sin(ωt)`, recover analytic forcing | MMS | T3 | converges at integration order; residual **< 1e-10** | `research` |
| V6 | Mode-acceleration line-load recovery vs full-FEM static recovery | model-vs-analytic | T2/T3 | line load within **2%** with correction; document error without | `research` |
| V7 | Uniform cantilever beam mode frequencies vs Blevins (`β_n L` roots, tip-mass) | analytic | T3 | first 5 frequencies within **1%** | `research` |
| V8 | CB-reduced frequencies + mode shapes vs CalculiX `*FREQUENCY`/`*SUBSTRUCTURE` on a shared FEM | code-to-code | T3 | **MAC > 0.9**, frequency error **< a few %** | `research` |
| V9 | GARTEUR SM-AG19 / DLR AIRMOD published modal data (MAC / cross-orthogonality) | public benchmark | T3 | MAC > 0.9 vs published; document FE-updating | `research` |
| V10 | POGO unstable-frequency band + accumulator detuning vs SP-8055 Titan/Saturn | public benchmark (qualitative) | T4 | unstable band brackets the published frequency; accumulator raises margin (trend) | `research` (method-level) |
| V11 | CSI: ≥ 6 dB gain / ≥ 30° phase margin per mode over an LTV trajectory; notch restores margin | analytic + SIL | T4 | margins computed; notch lifts a deliberately-unstable mode above 0 dB | `research` (method-level) |
| V12 | POGO/feed transmission-line + pump linearization MMS | MMS | T4 | manufactured transient recovered; residual **< 1e-10** | `research` |

**Honest notes.** V8/V9 validate the *method* (CB algebra, eigensolver,
correlation metrics) against published/synthetic FEMs — **not** a real vehicle's
GVT-correlated model (ceiling item 1). V10/V11 are explicitly *qualitative /
method-level* because the cavitation parameters `C_b`/`M_b` are not openly
available (ceiling item 2); the tolerance is on the *band and trend*, never a
margin number. V6 line loads are validated against analytic/FEM recovery, **not**
flight loads (ceiling item 3). The corrected slosh is linear-regime only;
large-amplitude/swirl/low-g is out of envelope and stated (ceiling item 4).

---

## 7. Dependencies on other parity docs

- **`01-flexible-multibody-dynamics.md` (FFRF + Craig-Bampton, shared asset).**
  The flexible-body *dynamics* substrate — a stage as an FFRF deformable body in
  the spatial-vector tree, with rigid-flex inertia coupling — is doc 01's. **This
  doc owns the modal content, loads/CLA chain, slosh analogs, and POGO/CSI
  stability.** The two share the **Craig-Bampton reduction** and the **ingested
  modal asset** (`openbmp-modelio::ModalAsset`). Boundary: doc 01 integrates the
  reduced modal ODE inside the tree (`M̂, K̂, Φ` as a vehicle asset); doc 02
  produces the line loads, runs the standalone CLA transient, and does the
  POGO/CSI analyses on the modal subset. At T1/T2 the modal block runs behind the
  existing rigid-body kernel (no FFRF needed); T3+ may consume doc 01's FFRF body
  when present. **`WP-02.3` (native CB) coordinates with `WP-01.5` (FFRF
  ingest)** so the CB reducer is built once.

- **`05-propulsion-high-fidelity.md` (POGO feed side).** POGO is a true capstone
  closing a structure↔propulsion loop. Doc 05 owns the **feed-system transient
  physics** (GFSSP-style fluid network, turbopump maps/affinity/NPSH, MOC
  water-hammer, cavitation linearization) — the `G_feed(s)` half of the loop.
  Doc 02 owns the **structural longitudinal modal half** and the **closed-loop
  stability assessment** (`det(I − G_feed·G_struct) = 0`, eigen/Nyquist,
  accumulator detuning, SP-8055 margins). `WP-02.4` (POGO) **depends on**
  doc 05's feed-system transfer model and the longitudinal CB modes from
  `WP-02.3`. The pump `(C_b, M_b)` dispersion box is the shared data bottleneck
  (ceiling item 2).

- **`06-gnc-coupled-mimo-and-control.md` (CSI / load-relief).** The CSI margin
  tooling and the load-relief autopilot co-design live at the doc-02↔doc-06 seam.
  Doc 02 provides the augmented flex plant and the margin evaluation; doc 06 owns
  the controller (LQR/LQG, load-relief, allocation). The notch/lead-lag design is
  shared (the FC-portable `csi` helpers).

- **`08-environment-gravity-and-frames.md`.** Slosh `g_eff` and the axial
  acceleration field come symbolically from the environment/atmosphere crate
  (never literal WGS84 numerals).

- **`11-monte-carlo-uq-and-validation.md`.** Per-entry bias/random margins on
  modal frequencies, slosh coefficients, and POGO pump parameters flow into the
  MC campaign and the 7009B credibility record (the dispersion box).

- **`12-determinism-realtime-and-compute.md`.** The dense linear algebra
  (CB reductions, `K_eff` factorization, complex-eigen) must honor the
  deterministic-f64 / fixed-reduction-order contract.

- **`03`/`04`** (aero DB) supply the distributed aero load that the CLA
  max-Q-gust event maps to modal forcing (`Φᵀ F`).

**Phase placement (`00` §5):** Phase B `02 T1–T3` (after `01` FFRF substrate);
Phase D `02 T4` POGO/CSI joins `05 T5`.

---

## 8. Open-source leverage

Use mode per the solver-consumer boundary (`00` §1.1, `13` §6): **port** =
re-implement the algorithm in deterministic Rust; **couple** = FFI/link;
**ingest** = run as an external offline tool and read its output files.

| Tool | License | Use mode | What |
|---|---|---|---|
| **CalculiX (ccx/cgx)** | GPL-2.0-or-later | **ingest** (offline) | Build the LV FEM, run `*FREQUENCY` + `*SUBSTRUCTURE GENERATE` (CB superelement); ingest modal frequencies, mode shapes at instrumented nodes, reduced M/K. **Do NOT link** ccx (GPL); file-level data ingest is clean. The T2 modal-asset source and the T3 code-to-code oracle (V8). |
| **faer-rs** | MIT OR Apache-2.0 | **port/use** (pure Rust) | Dense + sparse linear algebra for CB reductions (`M̂=TᵀMT`), the `K_eff` Newmark factorization, and the small closed-loop complex eigenproblem in POGO/CSI. Pure-Rust preserves the deterministic-f64 contract. |
| **ARPACK-NG** | BSD-3-Clause | **couple** (FFI, optional) | Lighter-weight sparse generalized eigensolve `K_ii Φ = M_ii Φ Λ` (implicitly-restarted Lanczos) for the CB fixed-interface modes; pair with a sparse LU/Cholesky for shift-invert. Use only if faer's sparse eig is insufficient at scale. |
| **SLEPc/PETSc** | BSD-2-Clause | **couple** (FFI, optional) | Industrial sparse eigensolver for very large FEMs; heavy MPI footprint is the cost, not the license. Out-of-scope unless T3 grows to 10⁵-10⁶ DOF (a LV tree is small). |
| **MFEM** | BSD-3-Clause | **port-reference** | Reference for assembling sparse M,K for beam/shell elements (bilinear-form patterns). A pure-Rust beam/shell FEM is lighter for OpenBMP's needs; read MFEM, don't link. |
| **pyNastran / pyYeti** | BSD-3-Clause / BSD-2-Clause | **port + ingest tooling** | Parse Nastran OP2/OP4 (modes, OTM, DMIG); pyYeti implements CB assembly, mode-acceleration recovery, and CLA workflows — excellent porting references and the offline ingest pipeline producing the CSV/JSON OpenBMP reads at T2. |
| **OpenFOAM (interFoam VOF) / Basilisk** | GPL-3.0 | **ingest** (offline) | Nonlinear/large-amplitude slosh (swirl, rotary, low-g reorientation) the linear analog cannot capture; run VOF to **calibrate** the analog's effective frequency/damping/mass coefficients; ingest fitted coefficients, never couple live. External tool only (do not link). |
| **cFS (NASA core Flight System)** | Apache-2.0 | **couple** (optional, SIL) | Run real FSW against OpenBMP's physics+sensor sim so the bending/POGO/CSI loops are exercised by actual flight software (the SIL North Star). |

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, green on the full `13` §2 gate set,
each shipping at a declared tier and earning a declared label. `WP-02.t[-letter]`.

---

### WP-02.1-a — Multi-mode lateral modal block

- **title:** Generalize `BendingMode` to an N-mode `ModalBlock`.
- **goal:** Replace the single hand-tuned bending mode with an N-mode modal block
  so the autopilot interacts with multiple bending frequencies and the modal
  basis can later be FEM-derived — the substrate for everything downstream. Why
  it matters for parity: production sims carry tens of modes; one mode is the
  major gap.
- **fidelity_tier:** T1
- **depends_on:** []
- **new_crates:** none (extends `openbmp-vehicle::structural`).
- **touched:** `crates/openbmp-vehicle/src/structural.rs`,
  `crates/openbmp-runner/src/structural.rs`,
  `crates/openbmp-scenario/src/document.rs` (`[vehicle.modes]`).
- **approach:** §3.2 `ModalBlock` + §3.3 (M2 diagonal SDOF branch). Per-mode
  `q̈ = forcing − damping − restoring`, semi-implicit Euler, modes iterated in
  declaration order, mode-summed gyro/accel/reaction. `[vehicle.bending]` maps to
  a one-mode block with identical arithmetic.
- **acceptance:**
  - new `[vehicle.modes]` off by default; **legacy `[vehicle.bending]` golden
    byte-identical** (V3).
  - N-mode block with N=1 byte-matches the legacy single-mode arithmetic.
  - per-axis property test: each mode rings at its `ω_j`, decays with `ζ_j`.
  - accelerometer pickup (`accel_pickup_m_s2`) added and wired through the bridge
    alongside the gyro pickup.
  - all §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line (forward modal physics).
- **est_effort:** ~1 week.
- **parity_ceiling:** modal parameters are hand-entered, not FEM-derived; no GVT
  correlation.

### WP-02.1-b — Wire the modal reaction moment into the dynamics

- **title:** Close the `StructuralRack` reaction-moment follow-up.
- **goal:** The runner exposes `reaction_moment_body_n_m()` but the doc says it is
  "wired into the dynamics in a follow-up". Apply the mode-summed reaction moment
  to the rigid body so flex reacts back on attitude (the missing half of the
  coupling).
- **fidelity_tier:** T1
- **depends_on:** [WP-02.1-a]
- **new_crates:** none.
- **touched:** `crates/openbmp-runner/src/structural.rs`, the kernel
  moment-accumulation path.
- **approach:** §3.3 (M3 lever route, mode-summed); add `−m_j·slope_engine_j·q̈_j`
  to the body moment, one-step-lag consistent with the existing driver.
- **acceptance:**
  - reaction off by default (zero when rack `Inactive` ⇒ byte-identical rigid
    run); on only when `[vehicle.modes]`/`[vehicle.bending]` present and a new
    `reaction.enabled` flag set (preserve existing bending goldens until opt-in).
  - momentum/energy bounded over a full ascent (no spurious injection).
  - all §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** ~3-5 days.
- **parity_ceiling:** lever reaction only; full distributed-load coupling is the
  FFRF path (doc 01).

### WP-02.1-c — Mass-correct slosh analog (Abramson fixed mass + N slosh masses)

- **title:** Add `AbramsonMultiMass` slosh with the fixed/slosh mass split.
- **goal:** The current pendulum treats **all** fluid as slosh mass, mis-stating
  rigid inertia. Implement the Abramson/Dodge fixed-mass `m_0` + N slosh-mass
  `m_n` split (cylinder closed-form; sphere/ellipsoid via Dodge tables) so CG and
  inertia are correct — the highest correctness-per-LOC win in the cluster.
- **fidelity_tier:** T1
- **depends_on:** []
- **new_crates:** none (new `MovingMassModel` impl in `openbmp-vehicle::tank`).
- **touched:** `crates/openbmp-vehicle/src/tank/` (+ `mod.rs` re-export),
  `crates/openbmp-scenario/src/document.rs` (`[vehicle.slosh.mass_split]`),
  `data/slosh/` (Dodge coefficient tables + `provenance.md`).
- **approach:** §3.3 (M4). `ξ_n` roots of `J_1'`, `m_n/m_f` and `m_0`, attach
  heights; N SDOF reactions at attach heights; fixed mass = rigid inertia only.
  Reuse the existing per-axis symplectic integrator and
  `point_mass_inertia_about_origin` helper.
- **acceptance:**
  - new model off by default; existing `EquivalentPendulum` goldens unchanged.
  - `ω_n`, `m_n/m_f`, `m_0`, attach heights within **1%** of SP-106/Dodge tables
    (V1, tolerance table).
  - slosh period + energy property test passes (V2); Dodge tables ingested with
    `provenance.md` + SHA pin.
  - all §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** ~1-1.5 weeks.
- **parity_ceiling:** linear-regime only; no swirl/rotary/low-g; coefficients are
  textbook, not tank-test-correlated.

### WP-02.2-a — Ingest an external reduced modal model

- **title:** OP2/OP4/CSV modal-model ingestion → provenance-pinned `ModalAsset`.
- **goal:** Read a Nastran/CalculiX modal export (frequencies, mode shapes at
  instrumented stations, modal mass) into the `ModalBlock` so the sim consumes a
  real FEM without building one — the production method-of-record.
- **fidelity_tier:** T2
- **depends_on:** [WP-02.1-a]
- **new_crates:** **propose `openbmp-modelio` (L1)** — ingestion + schema +
  provenance; no up-layer edge; shared with doc 01's FFRF ingest. First commit is
  the crate skeleton + placement justification (review before implementation).
- **touched:** new `openbmp-modelio`, `crates/openbmp-runner/src/structural.rs`
  (consume the asset), `data/modal/` + `provenance.md`.
- **approach:** §3.1; port the OP2/OP4 parse from pyNastran/pyYeti patterns
  (BSD); CSV path as the dependency-free default. Map mode shapes at instrumented
  stations to `slope_at_*`/`shape_at_accel`.
- **acceptance:**
  - ingested asset off by default; SHA-pinned `provenance.md`; fail-closed on
    schema/version mismatch.
  - code-to-code: ingested frequencies match the source FEM's SOL103 to **< 1%**;
    MAC of ingested mode shapes vs source **> 0.9** (V8, tolerance table).
  - allow-list updated for the new data path in the same PR.
  - all §2 gates green.
- **validation_label:** `research`
- **dual_use_note:** far from line; modal data is structural, not targeting.
- **est_effort:** ~2 weeks.
- **parity_ceiling:** the ingested FEM is benchmark/synthetic, not a GVT-correlated
  as-built vehicle.

### WP-02.2-b — Line-load recovery (LTM/OTM + mode-acceleration)

- **title:** Add station line-load recovery with the mode-acceleration correction.
- **goal:** Recover physical engineering quantities — axial/shear/bending vs
  station, interface forces — from the modal solution (the CLA "load recovery"
  step). The current lever-scaled reaction is the only physical-quantity output
  today; LTM/OTM is the rigorous generalization that sizes structure.
- **fidelity_tier:** T2
- **depends_on:** [WP-02.2-a]
- **new_crates:** **propose `openbmp-structdyn` (L2)** skeleton (recovery only;
  CB/transient land later) — pure-Rust numerics, depends only on core + linalg.
- **touched:** new `openbmp-structdyn::recovery`,
  `crates/openbmp-runner/` (line-load telemetry channels),
  `crates/openbmp-scenario/` (`[loads.recovery]` stations).
- **approach:** §3.3 (M3). `y = A_d q + A_a q̈` + the mode-acceleration residual;
  free-body D'Alembert station cut `S·(M(Φ q̈) + applied)`. Ingest OTM when
  present, else build `S` from the ingested mode shapes.
- **acceptance:**
  - recovery off by default; deterministic dense GEMV (locked order).
  - line load within **2%** of full-FEM static recovery **with** the
    mode-acceleration correction; documented error **without** it (V6).
  - line-load telemetry channels emitted; tolerance table added.
  - `SectionLoad`/`OutputId` vocabulary closed to structural quantities (dual-use).
  - all §2 gates green.
- **validation_label:** `research`
- **dual_use_note:** load-recovery outputs are structural only (no
  payload-effect/terminal quantities).
- **est_effort:** ~1.5-2 weeks.
- **parity_ceiling:** validated vs analytic/FEM recovery, **not** flight loads.

### WP-02.3-a — Craig-Bampton reduction + native modal transient

- **title:** Native CB reduction + Newmark/generalized-α modal transient.
- **goal:** A self-contained reduction (CB) + implicit transient so OpenBMP runs
  CLA without an external FEM at runtime, and so the FFRF asset (doc 01) can be
  built in-repo. The transient is the core CLA engine.
- **fidelity_tier:** T3
- **depends_on:** [WP-02.2-b]
- **new_crates:** extends `openbmp-structdyn` (`cb`, `transient`); coordinate the
  CB reducer with `WP-01.5` (build once).
- **touched:** `openbmp-structdyn::{cb,transient}`.
- **approach:** §3.3 (M1, M2). CB transform `T`, reduced `M̂=TᵀMT`, `K̂=TᵀKT`;
  faer sparse shift-invert for `Φ_ik`; Newmark-β default, generalized-α optional
  (`ρ_∞` pinned + documented). Precompute the `K_eff` Cholesky for constant dt.
- **acceptance:**
  - MMS: manufactured `q_j(t)` recovered at integration order; residual
    **< 1e-10** (V5/V12).
  - CB-reduced frequencies/shapes vs CalculiX `*FREQUENCY`/`*SUBSTRUCTURE`:
    **MAC > 0.9**, frequency error **< a few %** (V8).
  - `ρ_∞`, `(β,γ)` pinned; changing them is a documented per-scenario byte lock.
  - all §2 gates green.
- **validation_label:** `research`
- **dual_use_note:** far from line.
- **est_effort:** ~3-5 weeks.
- **parity_ceiling:** method-correct reduction; not GVT-correlated.

### WP-02.3-b — Beam/shell FEM + CLA flight-event forcing library

- **title:** Pure-Rust beam/shell FEM + liftoff/max-Q/ignition/sep/MECO forcing.
- **goal:** Assemble the FEM the CB reduction consumes and the event library that
  drives the CLA transient (per-event line-load envelopes), making OpenBMP a
  self-contained CLA producer.
- **fidelity_tier:** T3
- **depends_on:** [WP-02.3-a]
- **new_crates:** extends `openbmp-structdyn` (`fem`, `events`).
- **touched:** `openbmp-structdyn::{fem,events}`,
  `crates/openbmp-runner/` (event wiring), `crates/openbmp-scenario/`
  (`[loads.events]`).
- **approach:** §3.2 `EventForcing`; beam/shell element M,K (MFEM patterns);
  forcing `f(t)=Φᵀ F`: liftoff = ramp-release of hold-down preload (step), max-Q
  gust = von Kármán/Dryden wind into the aero load distribution (from doc 03),
  ignition/sep/MECO = impulse + transient stiffness change.
- **acceptance:**
  - uniform-cantilever beam frequencies vs Blevins within **1%** (V7).
  - GARTEUR SM-AG19 / AIRMOD published modal data: MAC > 0.9 (V9), FE-updating
    documented.
  - per-event line-load envelope emitted; events off by default; fail-closed on
    unknown event type.
  - all §2 gates green.
- **validation_label:** `research`
- **dual_use_note:** events are vehicle-intrinsic forcing, no targeting.
- **est_effort:** ~3-4 weeks.
- **parity_ceiling:** benchmark-correlated, not flight-loads-correlated.

### WP-02.4-a — POGO longitudinal closed-loop stability

- **title:** Feedline transmission-line + pump linearization + closed-loop
  eigen/Nyquist + accumulator detuning.
- **goal:** Close the structure↔propulsion longitudinal loop (Rubin/Oppenheim,
  SP-8055) and assess stability + accumulator sizing — the integrating capstone,
  entirely absent today.
- **fidelity_tier:** T4
- **depends_on:** [WP-02.3-a, **WP-05.* feed-system transfer model** (doc 05)]
- **new_crates:** extends `openbmp-structdyn::pogo`.
- **touched:** `crates/openbmp-structdyn/src/pogo.rs`, the modal-asset and
  feed-system transfer adapters, `crates/openbmp-scenario/src/document.rs`
  (`[vehicle.pogo]`), and `tests/expected/` tolerance tables.
- **approach:** §3.3 (M5). Longitudinal CB modal subset (`A_s`); feedline
  transmission matrix; pump `(C_b, M_b)` from a documented dispersion box (never
  fitted); close `Z(s)X(s)=0`; complex-eig RHP test + Nyquist; accumulator `C_acc`
  detuning. Consume doc 05's `G_feed(s)`.
- **acceptance:**
  - POGO/feed MMS: manufactured transient recovered; residual **< 1e-10** (V12).
  - unstable-frequency band brackets the SP-8055 Titan/Saturn published band;
    accumulator raises margin (trend) — **qualitative** (V10).
  - pump parameters entered as a dispersion box with `provenance.md`; **no
    vehicle-specific margin claimed** in any artifact.
  - all §2 gates green.
- **validation_label:** `research` (method-level only)
- **dual_use_note:** self-referential vehicle dynamics; pump gains/accumulator are
  stability inputs, no targeting surface.
- **est_effort:** ~4-8 weeks (hardest item).
- **parity_ceiling:** `C_b`/`M_b` are not openly available ⇒ margin is parametric,
  not a specific vehicle's.

### WP-02.4-b — CSI bending gain/phase-margin tool + notch design

- **title:** FC-portable frozen-time margin analysis + notch/lead-lag design.
- **goal:** Generalize the single-mode gyro-notch wiring to the full modal set
  with proper sensor mode-shape coefficients and frequency-domain margin analysis
  over the LTV trajectory — production CSI practice.
- **fidelity_tier:** T4
- **depends_on:** [WP-02.1-a]
- **new_crates:** none; adds `openbmp-fc::csi` (FC-portable, plain-coefficient
  inputs only).
- **touched:** `crates/openbmp-fc/src/csi.rs`,
  `crates/openbmp-fc/src/filters.rs` (reuse `Biquad`),
  `crates/openbmp-fc/src/autopilot.rs` (`gyro_notch` design hook),
  `crates/openbmp-scenario/` (`[csi]`).
- **approach:** §3.3 (M6). Augment the rigid plant with modal states; `L(jω)`
  sweep; `(gain_margin_db, phase_margin_deg)`; notch
  `H(s)=(s²+2ζ_z ω s+ω²)/(s²+2ζ_p ω s+ω²)`; lead/lag for the first mode; LTV
  frozen-time sweep.
- **acceptance:**
  - **FC portability lock intact** — `csi` consumes only coefficient arrays; no
    edge to sim/runner/scenario/structdyn; `fc_dependency_tripwire.rs` green.
  - ≥ 6 dB gain / ≥ 30° phase margin computed per mode over an LTV trajectory; a
    deliberately-unstable mode is lifted above 0 dB by the designed notch (V11).
  - time-domain SIL check: augmented modal model stays bounded through the FC.
  - all §2 gates green.
- **validation_label:** `research` (method-level)
- **dual_use_note:** stability-design tooling, benign; couples to the vehicle's
  own modes.
- **est_effort:** ~2-3 weeks.
- **parity_ceiling:** margins on a benchmark/synthetic modal model, not a
  GVT-correlated vehicle.

---

## 10. References

**Modal models & CMS.** Craig, R.R. & Bampton, M.C.C., "Coupling of Substructures
for Dynamic Analyses", *AIAA J.* 6(7):1313-1319, 1968. — Craig & Kurdila,
*Fundamentals of Structural Dynamics*, 2nd ed., Wiley 2006 (CMS ch. 19). —
NASA-STD-5002 / NASA-HDBK-7005, *Dynamic Environmental Criteria* (CLA &
substructuring; LTM definitions). — MSC/Nastran SOL103 + DMAP superelement
OUTPUT4 export.

**Transient integration / CLA.** Newmark, N.M., "A Method of Computation for
Structural Dynamics", *J. Eng. Mech. Div. ASCE* 85(EM3):67-94, 1959. — Chung &
Hulbert, "A Time Integration Algorithm … Generalized-α Method", *J. Appl. Mech.*
60:371, 1993. — Wijker, J., *Spacecraft Structures*, Springer 2008 (CLA
transient; mode-acceleration vs mode-displacement). — ECSS-E-ST-32-11C; ECSS-E-HB-
32-26A (mechanical loads analysis handbook).

**Load recovery.** Flanigan, C.C., "Implementation of the Mode Acceleration Data
Recovery Method", MSC Users Conf. — Cornwell, Craig & Johnson, "On the
Application of the Mode-Acceleration Method", *Earthquake Eng. & Struct. Dyn.*,
1983.

**Slosh.** Abramson, H.N. (ed.), *The Dynamic Behavior of Liquids in Moving
Containers*, NASA SP-106, 1966 (Ch. 5 baffle damping/Miles, Ch. 6 mechanical
analogies). — Dodge, F.T., *The New Dynamic Behavior of Liquids in Moving
Containers*, SwRI, 2000. — Bauer, H.F., NASA TR R-187, 1964. — Ibrahim, R.A.,
*Liquid Sloshing Dynamics*, Cambridge Univ. Press, 2005.

**POGO.** Rubin, S., "Prevention of Coupled Structure-Propulsion Instability
(POGO)", NASA SP-8055, 1970. — Rubin, S., "Longitudinal Instability of Liquid
Rockets Due to Propulsion Feedback (POGO)", *J. Spacecraft & Rockets*
3(8):1188-1195, 1966. — Oppenheim & Rubin, "Advanced Pogo Stability Analysis for
Liquid Rockets", *J. Spacecraft & Rockets* 30(3), 1993. — Brennen, C.E.,
*Hydrodynamics of Pumps*, Cambridge Univ. Press, 1994.

**CSI / bending stabilization.** Greensite, A.L., *Analysis and Design of Space
Vehicle Flight Control Systems* (NASA CR-820 series), 1967-70. — Hoelker, "Theory
of Artificial Stabilization of Missiles and Space Vehicles", NASA TR R-200,
1961. — Wie, B., *Space Vehicle Dynamics and Control*, 2nd ed., AIAA, 2008
(ch. 7). — Orr, J.S. et al., SLS flight control / bending filter (AAS/AIAA GNC,
NTRS).

**Benchmarks & verification.** GARTEUR SM-AG19 WG reports; DLR AIRMOD
stochastic-updating dataset. — Blevins, *Formulas for Natural Frequency and Mode
Shape*. — Method of Manufactured Solutions (modal transient + load recovery).

**OpenBMP anchors.** `docs/verification.md`, `docs/safety-boundaries.md`,
`docs/launch-vehicle-fidelity-frontier.md`,
`docs/external-telemetry-validation.md`; `00-overview.md` (constitution),
`13-agent-execution-playbook.md` (playbook); companions
`01-flexible-multibody-dynamics.md`, `05-propulsion-high-fidelity.md`,
`06-gnc-coupled-mimo-and-control.md`, `08-environment-gravity-and-frames.md`,
`11-monte-carlo-uq-and-validation.md`, `12-determinism-realtime-and-compute.md`.
