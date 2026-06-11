# Parachute, Decelerator & Recovery Systems

**Status:** `experimental` (design intent; this document ships no code). Date: undefined.
**Audience:** the engineer or LLM agent implementing the parity work packages.
**One-line summary:** lift recovery devices from instantaneous force-only drag
swaps to CPAS/DSS-class system simulation — parametric `C_D S(t)` inflation with
infinite/finite-mass opening loads and apparent mass, mortar/bag-strip/
line-stretch deployment sequencing with snatch loads, reefing/disreef staging,
slack-taut riser line elements, per-canopy cluster modeling with lead-lag
dispersions, two-body capsule-chute dynamics with pendulum-mode GNC mitigation —
then close the recovery lifecycle with water-impact load surrogates, sea-state
marine-recovery statistics, and mid-air-retrieval engagement, all dispersed
through deterministic Monte Carlo and anchored to reconstructed open drop-test
data.

> Read `00-overview.md` (parity definition, invariants, DAG) and
> `13-agent-execution-playbook.md` (the §4 template this doc follows and the §5
> work-package schema) before this document. This is dimension `16`; the early
> tiers are self-contained extensions of the existing recovery rack, the
> two-body tier rides the `01` multibody substrate, and the lifecycle tiers
> consume the `08` environment and `11` Monte Carlo machinery.

---

## 1. Parity target & ceiling

### 1.1 Target capability

The reference programs simulate recovery as a *system*, not a drag number. NASA
JSC's Capsule Parachute Assembly System (CPAS) program and the Orion Descent
Simulation hierarchy model each canopy in a cluster individually — per-chute
inflation timing, reefing stages, load asymmetry, wake effects — inside
dispersed Monte Carlo campaigns whose parameters were tuned against dozens of
instrumented drop tests (NTRS 20130010409, 20110011397, 20220003982). JPL's MSL
entry-descent-landing simulation carried a parachute model in POST2 with
apparent (added) mass and inflation transients (NTRS 20130012762). Orion's
"pendulum problem" — a cluster-instability-driven swing under two mains
discovered in drop testing — was characterized in simulation and mitigated
through GN&C changes (NTRS 20170003948). The lifecycle closes with water-impact
loads (Orion LS-DYNA models validated at the Langley Hydro Impact Basin, NTRS
20190000444) and recovery operations — Rocket Lab flies drogue/main marine
recovery for Electron boosters and demonstrated helicopter mid-air retrieval
(nasaspaceflight.com, May 2022; spaceflightnow.com, 2022-05-03).

OpenBMP's target (audit verdict: *force-only → approaching*, method parity) is
that **architecture**:

- **Inflation is a transient**, not a step: parametric `C_D S(t)` growth over a
  fill time, with the infinite-mass (load-dominated) and finite-mass
  (deceleration-relieved) regimes distinguished, and apparent-mass effects in
  the descent equation of motion.
- **Deployment is a load-generating event chain**: mortar fire (reaction
  impulse on the vehicle), bag strip, line stretch, snatch load, then
  inflation; reefing stages with dispersed cutter times.
- **Risers are slack-taut structural elements**, not a rigid assumption:
  tension-only spring-damper lines that transmit snatch and opening loads and
  enable two-body dynamics.
- **Clusters are per-canopy**: independent inflation lead-lag, load share
  asymmetry, cluster drag efficiency, forebody wake deficit.
- **The capsule-chute pair is a coupled two-body system** exhibiting the
  canopy-instability-driven pendulum mode, with forward-only GNC mitigation
  under canopy.
- **The lifecycle ends in loads and operations**: water-impact peak
  acceleration vs impact velocity/attitude/wave slope, sea-state-conditioned
  marine-recovery success statistics, and mid-air-retrieval engagement
  feasibility — all fed by the dispersed descent.
- **Everything is dispersed**: fill constants, opening-force coefficients,
  reefing ratios, cutter times, line stiffness, cluster lag, winds, and sea
  state flow through the deterministic Monte Carlo substrate with honest
  interval statistics.

### 1.2 The honest parity ceiling

Validation parity is structurally out of reach; each item below names the
proprietary/test-bound thing and the credible open substitute.

1. **Flight/drop-test-anchored canopy parameter sets.** CPAS's dispersions
   (fill constants, `C_X`, asymmetry factors, wake fractions) were calibrated
   against an instrumented drop-test campaign whose raw database is not public.
   *Open substitute:* parameter ranges from Knacke's *Parachute Recovery
   Systems Design Manual* and the published CPAS/MSL methodology papers, plus
   digitized trajectory/load reconstructions from the openly published NTRS
   papers as `research`-label anchors. The *methodology* (per-canopy dispersion
   architecture) is fully reachable; the *numbers* are textbook-class.
2. **Fluid-structure-interaction inflation physics.** Production inflation
   loads at the high-fidelity end come from coupled FSI/ALE simulation
   (LS-DYNA-class) and wind-tunnel/flight test. Per the solver-consumer posture
   (`00-overview.md` §1.1) OpenBMP does **not** write an FSI solver; the in-repo
   tier is the Knacke/Pflanz engineering method, and any higher-fidelity
   inflation or water-impact data enters as a provenance-pinned, UQ-wrapped
   ingested deck.
3. **Water-impact structural validation.** Orion's LS-DYNA splashdown models
   were validated against Langley Hydro Impact Basin swing-drop tests (NTRS
   20190000444); no open equivalent test series exists for an arbitrary
   vehicle. *Open substitute:* a von Kármán/Wagner momentum-theory tier
   verified against its closed form, an ingested response-surface surrogate
   path for externally produced (e.g. SPH/VOF) results, and the published
   Orion-class trends as a method anchor — never a structural-qualification
   claim.
4. **Recovery-operations reality.** Sea-state limits, ship/helicopter
   performance, and crew procedures are operator data. Electron's MAR attempts
   are documented only in press-level detail. *Open substitute:* a documented,
   scenario-declared success-criteria table (sink rate, hang angle, significant
   wave height, corridor containment) with the statistics machinery validated,
   and the criteria values labeled `experimental` user inputs.

**No artifact in this dimension may claim drop-test-validated loads or
qualification status.** Tiers earn `validated-toy` on closed-form/MMS/
code-to-code evidence and `research` only where a published reconstruction or
public benchmark anchors them (§5). The ceiling statement is repeated per-WP.

---

## 2. Current state in source

The recovery stack exists end-to-end as a *force-only, instantaneous-deploy*
chain — the architecture (trait, rack, kernel snapshot view, event wiring,
e2e determinism gate) is solid and is the regress-against baseline; the physics
above "constant `C_D·A` after a step change" is absent. Verified against
source:

### 2.1 What already exists (the regress-against baseline)

- **Recovery device models (three).**
  `crates/openbmp-vehicle/src/recovery/parachute_drag.rs` (single-stage),
  `drogue_main.rs` (two-stage `Stowed → Drogue → Main`), and `drag_device.rs`
  (cyclable airbrake) implement the `RecoveryModel` trait
  (`recovery/mod.rs:159-196`): stable `RecoveryId`, `RecoveryPhase`, current
  `(C_D, A)` pair, typed fail-closed command application, and a `step(dt)`
  that is **deliberately a no-op** — the trait doc reserves it "for future
  phases that may add inflation transients or canopy-area blends"
  (`recovery/mod.rs:153-158`). This is the exact seam Tier 1 fills.
- **Force application.** `crates/openbmp-vehicle/src/adapters.rs:2136-2169`
  (`RecoveryRackForceAdapter`) evaluates
  `F = −½ ρ |v|² Σ(C_D·A) · v̂` per Knacke 1992 Ch. 5, applied **at the CG with
  zero moment** ("long risers are assumed to decouple body rotation from drag
  direction", `recovery/mod.rs:24-32`) and against **ECI velocity, ignoring
  wind** — the module doc states the academic formulation explicitly. Both
  assumptions are honest, documented, and are dismantled by Tiers 1/3/4.
- **Kernel snapshot plumbing.** `crates/openbmp-models/src/models.rs:312-400`
  (`RecoverySnapshot`, `RecoverySnapshotView`) carries
  `(deployed, phase_index, c_d, drag_area_m2)` into the kernel per base tick;
  the runner-side `RecoveryRack` (`crates/openbmp-runner/src/recovery.rs`)
  owns a `BTreeMap<RecoveryId, Box<dyn RecoveryModel>>` (deterministic
  iteration), drains `ScenarioScriptAction::DeployRecovery` firings, rejects
  duplicate per-step commands, and short-circuits to byte-identical output
  when empty.
- **Mission-event wiring.** `crates/openbmp-mission/src/events.rs` provides
  crossing-detector triggers and the `deploy_recovery` action;
  `docs/descent-and-entry-profiles.md` defines the
  `final_descent → recovery → post_flight` vocabulary and validates
  drogue-before-main ordering. Scenario schema:
  `[[vehicle.assembly.recovery]]` with `kind = parachute_drag | drogue_main |
  drag_device` (`docs/scenario-format.md`).
- **Canonical scenario + determinism gate.**
  `scenarios/parachute-recovery/parachute-descent.toml` with
  `crates/openbmp-cli/tests/parachute_recovery_e2e.rs`: full
  parse → rack → event → snapshot → RK4 force path, byte-identical Parquet
  across reruns. Any new capability must keep this golden byte-identical until
  a scenario opts in.
- **Ground impact = a stop, not a load.** `crates/openbmp-sim/src/kernel.rs`
  (`separated_rigid_body_has_impacted_ground`, kernel.rs:104-120) retires a
  lane at a geocentric ground radius. There is no touchdown/splashdown load
  model, no surface distinction (land vs water), no sea state.
- **Descent dispersion machinery exists for *footprints*, not for *chutes*.**
  `crates/openbmp-runner/src/footprint.rs` + `openbmp-physics/src/profile.rs`
  run offline landing-footprint Monte Carlo from provenance-seeded
  `BallisticState`s (the forward-only lock,
  `crates/openbmp-testkit/tests/ballistic_state_compile_fail.rs`); the
  deterministic campaign substrate `crates/openbmp-mc` (LHS, Welford,
  Clopper-Pearson, Wilks) is live. Neither currently disperses any recovery
  parameter — `C_D S` is a scalar with no uncertainty object.
- **The code-to-code oracle is already in the repo.** The RocketPy Calisto
  reference vehicle is ingested with provenance
  (`scenarios/sounding-rocket/calisto/rocketpy-calisto.toml`,
  `data/aero/calisto-drag.toml`, `data/motors/rocketpy-calisto-m1670.toml`)
  for ascent validation; RocketPy's parachute descent (a `C_D S` + trigger-lag
  model) is the natural descent-phase cross-check and is not yet exercised.
- **Wind models exist but recovery ignores them.**
  `crates/openbmp-physics/src/wind/` ships constant/layered/HWM14/Dryden-gust
  winds with a `WIND` RNG domain; the recovery drag adapter does not consume
  wind-relative velocity, so descent *drift* — the first-order driver of
  splashdown/landing dispersion — is structurally absent from the chute phase.

### 2.2 Maturity summary

| Sub-dimension | Current | Target tier |
|---|---|---|
| Canopy drag | constant `C_D S` per phase, instantaneous swap | T1 `C_D S(t)` inflation |
| Opening loads | none (no load channel) | T1 infinite/finite-mass (Pflanz) |
| Apparent mass | none | T1 scalar / T4 tensor |
| Wind-relative descent drift | none (ECI-velocity drag) | T1 |
| Deployment sequence / snatch | none (deploy = step change) | T2 mortar/bag-strip/line-stretch |
| Reefing / disreef | none | T2 staged `C_D S` + cutters |
| Riser / line elements | rigid zero-moment assumption | T2 slack-taut spring-damper |
| Clusters | single equivalent device | T3 per-canopy lead-lag + asymmetry |
| Forebody wake | none | T3 dynamic-pressure deficit |
| Capsule-chute two-body / pendulum | none | T4 (rides `01` multibody) |
| GNC under canopy | none (descent uncontrolled) | T4 forward-only damping |
| Water-impact loads | ground-radius stop only | T5 momentum tier + ingested surrogate |
| Sea state / marine recovery stats | none | T5 spectrum + success MC |
| Mid-air retrieval | none | T6 engagement + post-capture pendulum |
| Recovery-parameter dispersion | none | T3/T5 via `openbmp-mc` |

---

## 3. Target architecture

### 3.1 Crate map and placement

**No new crate is required.** Every concern lands in an existing crate at its
existing layer; the two-body tier consumes the `openbmp-multibody` crate that
doc `01` already proposes, and the campaign tiers consume the existing
`openbmp-mc`. Justification against the `00-overview.md` §4 map: recovery
devices are vehicle-intrinsic hardware (sibling of tanks/engines/effectors →
`openbmp-vehicle`); waves are environment (sibling of atmosphere/wind →
`openbmp-physics`); campaign orchestration is L7 (`openbmp-runner` +
`openbmp-mc`). Adding an `openbmp-recovery` crate would duplicate the
vehicle-layer boundary for no dependency benefit; if `openbmp-vehicle` ever
splits for size, that is a mechanical refactor reviewed on its own.

```text
openbmp-vehicle   L2  + inflation models, deployment-sequence state machine,
                       reefing stages, riser line elements, cluster device,
                       opening/snatch-load channels (extend src/recovery/*)
openbmp-physics   L2  + sea-state spectrum (Pierson-Moskowitz/JONSWAP) +
                       deterministic wave realization; von Kármán/Wagner
                       water-impact momentum tier (extend profile/environment)
openbmp-models    L1  + extended RecoverySnapshot (per-canopy, line tensions,
                       load channels); splashdown-surrogate deck trait surface
openbmp-runner    L7  + rack wiring for sequences/clusters; touchdown/splashdown
                       event + load evaluation; recovery-ops MC campaign hooks
openbmp-mc        L7  (consumer) recovery-parameter dispersions, success stats
openbmp-multibody L2  (from doc 01) canopy-as-body + riser constraint for T4
openbmp-fc        L4  (consumer only) under-canopy rate-damping mode through the
                       existing sensor/effector boundary (PORTABILITY LOCK)
```

Ingested data packages: `data/recovery/` (canopy parameter tables, digitized
drop-test reconstructions, splashdown response-surface decks), each with a
sibling `provenance.md` + SHA-256 pin per the four-pillar contract.

### 3.2 Parametric inflation + opening loads (T1)

**Fill time and drag-area growth** (Knacke Ch. 5). From line stretch at
velocity `v_s`, the canopy fills over

```
t_f = n · D₀ / v_s^k                                              (3.2.1)
```

with `D₀` the nominal canopy diameter, `n` the canopy fill constant
(type-dependent, dispersed), and `k` an empirical exponent (default 1;
0.85-0.9 variants documented per canopy class). Drag-area growth over
`τ = (t − t_ls)/t_f ∈ [0, 1]`:

```
(C_D S)(τ) = (C_D S)_init + [(C_D S)_full − (C_D S)_init] · ξ(τ)   (3.2.2)
ξ(τ) = τ^j   (polynomial, j ≈ 2–3)  |  ξ(τ) = (e^{ατ} − 1)/(e^{α} − 1)
```

with the growth-shape choice and exponent scenario-declared and dispersed.

**Opening-load regimes.** The *infinite-mass* condition (vehicle heavy /
dynamic pressure nearly constant over the fill — drogues, high-q reefed
stages) gives the peak directly:

```
F_max = (C_D S)_full · q_s · C_X                                   (3.2.3)
```

with `q_s = ½ ρ v_s²` at line stretch and `C_X` the opening-force coefficient
(canopy-type table, dispersed). The *finite-mass* condition (mains; the system
decelerates significantly during inflation) uses the Pflanz method: the
dimensionless ballistic parameter

```
A = 2 m / [ρ · (C_D S)_full · v_s · t_f]                           (3.2.4)
F_max = (C_D S)_full · q_s · C_X · X₁(A, j)                        (3.2.5)
```

where `X₁ ∈ (0, 1]` is the force-reduction factor (tabulated vs `A` and the
growth exponent `j`; `X₁ → 1` as `A → ∞` recovers 3.2.3 — the V&V limit
check). The integrated alternative — advancing 3.2.2 inside the descent EOM
and *reading* the peak from the trajectory — is the primary path; Pflanz is
the closed-form cross-check, exactly the dual POST2-class tools use.

**Apparent (added) mass** (the MSL POST2 model element, NTRS 20130012762).
During and after inflation the canopy accelerates entrained and surrounding
air. Scalar T1 form along the descent axis:

```
m_a(τ) = k_a · ρ · (π/6) · D_p(τ)³                                 (3.2.6)
(m_v + m_c + m_a) · dv/dt = ΣF − (dm_a/dt) · v                     (3.2.7)
```

with `k_a` the added-mass coefficient (canopy-type table) and `D_p` the
instantaneous projected diameter implied by `(C_D S)(τ)`. Implementation: the
recovery device publishes `m_a` and `dm_a/dt` channels; the opted-in kernel
treats `m_a` as additional vehicle mass and `−(dm_a/dt)·v` as an additional
force — reusing the existing time-varying-mass machinery, never an implicit
acceleration solve. The `dm_a/dt` term is what makes finite-mass opening loads
and post-disreef rebound honest.

**Wind-relative drag** (same WP): the recovery force adapter gains an opt-in
airspeed mode `v_rel = v − v_wind` using the existing `WindModel` surface —
the aero-deck convention alignment that doc `03` establishes — so descent
drift exists at all.

**Surface (extend `openbmp-vehicle/src/recovery/`):**

```rust
/// Scenario-declared inflation law attached to one canopy stage.
pub struct InflationProfile {
    pub fill_constant_n: f64,          // eq 3.2.1
    pub fill_exponent_k: f64,
    pub growth: GrowthShape,           // Polynomial { j } | Exponential { alpha }
    pub opening_force_coeff_c_x: f64,  // eq 3.2.3
    pub added_mass_coeff_k_a: f64,     // eq 3.2.6
}

/// Per-tick output of an inflating stage (pushed via RecoverySnapshot).
pub struct CanopyStateSample {
    pub c_d_s_m2: f64,
    pub apparent_mass_kg: f64,
    pub apparent_mass_rate_kg_s: f64,
    pub load_n: f64,                   // axial canopy load this tick
}
```

`RecoveryModel::step` stops being a no-op for opted-in devices: it advances
`τ` with locked operand order at the kernel base tick; `(C_D S)` is held
zero-order over the tick (the same first-order hold the rack architecture
already implies), documented as `O(h)` in the fill-time resolution.

### 3.3 Deployment sequence, snatch, reefing (T2)

**Event chain as a typed sub-state machine inside the device** (no new
mission-event surface; the existing `deploy_*` command starts the chain):

```
Stowed → MortarFired → BagStrip → LineStretch → Inflating(stage 0)
       → Reefed(stage s) → Disreefing → FullOpen
```

- **Mortar fire:** the pack (mass `m_pack`) is ejected at muzzle velocity
  `v_m`; the vehicle receives the reaction impulse `J = m_pack · v_m` over the
  mortar stroke (a short force pulse, scenario-declared duration) — a real
  load case the GNC also sees.
- **Bag strip / line stretch:** the pack decelerates relative to the vehicle
  under its own drag; line stretch is declared when the relative separation
  reaches the unstretched line length `L₀` (T2 kinematic model: 1-DOF relative
  coordinate along the wake axis; T4 replaces it with the two-body solution).
- **Snatch load:** at line stretch the relative velocity `Δv` is arrested by
  the elastic line. Closed-form (impulse-energy) anchor:

```
F_snatch ≈ Δv · √(k_line · m_e) + F_drag,pack                      (3.3.1)
```

  with `k_line = (EA)_eff / L₀` and `m_e` the effective decelerated mass
  (canopy + bag + entrained air). In simulation the spring-damper riser element
  (§3.4) produces the transient directly; 3.3.1 is the tolerance-table check.
- **Reefing / disreef:** each stage `s` declares a reefed drag-area fraction
  `(C_D S)_s / (C_D S)_full`, its own inflation profile, and a cutter time
  `t_cut,s` (dispersed, per-cutter, independent draws — the CPAS practice that
  makes cluster disreef asymmetric). Disreef re-enters `Inflating(s+1)` with
  `v_s` re-sampled from the current state, so each stage has its own
  finite-mass peak.

### 3.4 Slack-taut riser line element (T2)

Tension-only Kelvin-Voigt element between attach points `p_a` (vehicle) and
`p_b` (canopy/pack), unit vector `û`, stretch `δ = ‖p_b − p_a‖ − L₀`:

```
T = max(0, (EA/L₀) · δ + c_line · δ̇)   applied as ±T·û            (3.4.1)
```

with `T ≡ 0` when slack (δ ≤ 0) — the discontinuity is event-located at the
tick boundary (deterministic, no adaptive stepping). At T2 the element acts
between the vehicle body and the 1-DOF pack/canopy coordinate (snatch loads,
riser load channels, attach-point moments via `r_attach × T·û` — retiring the
zero-moment assumption for opted-in scenarios). At T4 the same element couples
two full bodies in the `01` tree. Multiple risers per canopy and per-leg
asymmetry come free from the per-element formulation.

### 3.5 Cluster modeling (T3)

The CPAS/DSS discriminator: model *each canopy in the cluster*, not an
equivalent single chute (NTRS 20130010409).

- **Per-canopy state:** every member runs its own §3.2/§3.3 state machine with
  independently dispersed fill constant, `C_X`, reefing ratio, and cutter
  times — producing the observed **lead-lag**: one canopy opens first, takes
  the load peak, and is unloaded when the laggards inflate.
- **Load asymmetry / share:** per-canopy load `F_i` follows from its own
  `(C_D S)_i(t)` and the shared dynamic pressure; the snapshot publishes the
  per-canopy share `F_i / ΣF` so the loads consumer (`02`) sees the asymmetric
  attach-point case, and a dispersed asymmetry factor scales the lead canopy
  per the CPAS dispersion practice.
- **Cluster efficiency:** mutual aerodynamic interference reduces total drag:
  `(C_D S)_cluster = η_N · Σ (C_D S)_i` with `η_N ≤ 1` a documented
  cluster-efficiency table vs canopy count (Knacke), dispersed.
- **Forebody wake deficit:** the canopy flies in the capsule's wake; effective
  dynamic pressure `q_eff = (1 − w(x/D)) · q_∞` with `w` a documented wake
  fraction vs trailing distance (parameter table at T3; a `03` aero-deck entry
  when distributed data exists).
- **Geometry:** per-canopy attach points + mean riser angles give a net force
  direction and moment; with §3.4 elements each member's riser carries its own
  tension.

### 3.6 Two-body capsule-chute dynamics + pendulum + GNC mitigation (T4)

**Canopy as a body** in the `01` spatial-vector tree: a 6-DOF body with canopy
structural mass plus an **apparent-mass tensor** (distinct axial/normal added
mass and added moment of inertia — the standard parachute 6-DOF practice; the
scalar 3.2.6 is its trace-level reduction), connected to the capsule by §3.4
riser elements (slack-taut ⇒ no holonomic constraint; the tree sees two
free-flyer bodies exchanging line forces, which the `01` architecture already
permits as external spatial forces).

**Canopy aerodynamics:** axial `C_T(α_t)` and normal `C_N(α_t)` coefficient
tables vs total angle of attack. The pendulum driver is a canopy that is
**statically unstable near α_t = 0** (`dC_N/dα_t` destabilizing below a trim
angle): the canopy seeks its stable trim off-axis and drags the system into a
coning/pendulum limit cycle — the Orion two-main phenomenon (NTRS
20170003948). The two-body system reproduces the mode class: swing frequency
near the compound-pendulum estimate `ω ≈ √(g/L_eff)` (V&V anchor), amplitude
set by the instability and damping.

**GNC mitigation under canopy (forward-only):** a `final_descent`/under-canopy
mode in the existing gain schedule consuming estimated body rates through the
unchanged FC boundary and commanding RCS rate damping (the `09` MIB/PWPF
effectors) to limit swing amplitude — objective vocabulary strictly
`attitude/rate/swing-energy`, vehicle-intrinsic, exactly the class of
mitigation the Orion GN&C study evaluated. No new FC dependency; the FC sees
only sensor topics and effector commands.

### 3.7 Water impact, sea state, marine recovery (T5)

**In-repo momentum-theory tier (von Kármán 1929 / Wagner 1932).** For a body
of mass `m` and waterline geometry `c(z)` (instantaneous wetted radius vs
penetration `z`), vertical impact at `v₀` conserves momentum into the growing
water added mass `m_w(z) = k_w · ρ_w · c(z)³` (flat-plate added-mass form;
Wagner wetting correction as a documented option):

```
v(z) = v₀ · m / (m + m_w(z))                                       (3.7.1)
a(z) = v(z)² · (dm_w/dz) · m / (m + m_w(z))²                       (3.7.2)
```

closed-form peak acceleration for sphere/cone/wedge primitives (deadrise
wedge: the classic `cot β` scaling) — machine-precision verifiable. Effective
impact conditions come from the descent state: vertical/horizontal velocity,
hang angle under canopy, plus **wave slope and orbital velocity** rotating and
offsetting the relative-impact vector.

**Ingested surrogate (solver-consumer posture, `13` §6).** A provenance-pinned
response surface

```
a_peak, p_peak = S(v_v, v_h, θ_hang, φ_wave_slope)  ± (bias, σ)    (3.7.3)
```

produced externally (SPH/VOF-class solvers; Orion's LS-DYNA practice, NTRS
20190000444, is the method anchor) and ingested under `data/recovery/` with
per-entry UQ margins flowing into Monte Carlo. The in-repo tier (3.7.1-3.7.2)
is the code-verification floor and the fallback when no deck is supplied.

**Sea state.** `openbmp-physics` gains a wave model: Pierson-Moskowitz /
JONSWAP spectrum `S_η(ω)` parameterized by significant wave height `H_s` and
peak period `T_p`, realized as a deterministic finite cosine sum with
`DeterministicRng`-drawn phases (a new `WAVE` domain tag). At splash time and
point it yields surface elevation, slope, and orbital velocity — the inputs to
3.7.3.

**Marine-recovery statistics.** A recovery-ops campaign (via `openbmp-mc`)
disperses the descent (§3.8) × sea state, evaluates a scenario-declared
success-criteria table (max sink rate at splash, max hang angle, max `H_s`,
splash-point containment within the declared recovery-zone radius — the
permitted *site-radius* vocabulary), and reports success probability with
Clopper-Pearson intervals and Wilks-bounded load quantiles. Criteria values
are user inputs labeled `experimental`; the statistics machinery is what gets
validated.

**Mid-air retrieval (T6).** Three forward pieces, anchored to the public
Electron MAR sequence (drogue, main, helicopter hook engagement of the drogue
line near 2 km, post-catch carry or release — nasaspaceflight.com May 2022;
spaceflightnow.com 2022-05-03):

1. **Engagement corridor:** forward-propagate the dispersed descent through
   the capture-altitude band → a time-tagged corridor tube (position/velocity
   quantiles) published as a *feasibility volume for the cooperative recovery
   aircraft*. It reuses the provenance-seeded descent-state machinery
   (`footprint.rs` pattern) — never a guidance input to the vehicle.
2. **Hook-line capture:** geometric capture window (engagement-line length,
   closing-velocity bounds, lateral-offset bound) → per-sample capture
   success; capture probability across the dispersed corridor.
3. **Post-capture two-body pendulum:** at hook closure the load path transfers
   to the capture line — a §3.4 element from the (kinematically scripted)
   carrier point to the vehicle; the swing transient and peak hook load
   `T_peak` (impulse at engagement + pendulum oscillation) are the outputs.

### 3.8 Fidelity tiers

| Tier | Scope | Earns |
|---|---|---|
| **T0** (current) | instantaneous `C_D S` swap, ECI-velocity drag at CG, zero moment, ground-radius stop | `checked` |
| **T1** | `C_D S(t)` inflation (3.2.1-3.2.2), infinite/finite-mass opening loads (3.2.3-3.2.5), scalar apparent mass (3.2.6-3.2.7), wind-relative drag option, load telemetry channels | `validated-toy` |
| **T2** | deployment-sequence chain + mortar reaction + snatch (3.3.1), slack-taut riser elements (3.4.1) + attach-point moments, reefing/disreef staging with dispersed cutters | `validated-toy` |
| **T3** | per-canopy clusters (lead-lag, load share, efficiency, wake deficit), dispersed-parameter descent MC, drop-test-reconstruction validation case | `validated-toy` → `research` (CPAS-methodology anchor) |
| **T4** | two-body capsule-chute on the `01` tree, apparent-mass tensor, canopy-instability pendulum, forward-only GNC damping under canopy | `validated-toy` → `research` (Orion pendulum mode-class) |
| **T5** | von Kármán/Wagner impact tier + ingested splashdown surrogate + UQ, sea-state spectrum, marine-recovery success statistics | `validated-toy`; `research` only on the method anchor |
| **T6** | mid-air-retrieval engagement corridor + hook capture + post-capture pendulum | `validated-toy` (dynamics); `experimental` (ops statistics) |

Each tier is independently shippable; T1/T2 are useful with no Monte Carlo,
T3/T5 statistics ride `openbmp-mc` without T4, and T4 requires only the `01`
substrate plus T2 elements.

---

## 4. Invariant preservation

- **Byte-determinism.** All new dynamics advance with locked operand order at
  the kernel base tick — pure `f64`, no `f64::mul_add`, no wall-clock, no
  system RNG, no unordered iteration (clusters iterate the existing
  `BTreeMap`). New randomness (fill constants, `C_X`, cutter times, cluster
  lag/asymmetry, wave phases, surrogate noise) draws only from
  `DeterministicRng` with domain-separated `(seed, step, recovery_id,
  component)` tuples — a new `RCVY` component family alongside the existing
  `WIND` tag, with reserved component slots so adding a dispersion never
  shifts an existing stream. The slack-taut switch (3.4.1) and the sub-state
  transitions are tick-boundary events, never adaptive-step located.
- **Byte-stable-by-default.** Every capability is off until a scenario opts
  in: `[vehicle.assembly.recovery.<id>.inflation]`, `.sequence`, `.reefing`,
  `.risers`, `.cluster`, `[recovery.splashdown]`, `[environment.sea_state]`,
  `[recovery.ops_mc]`, `[recovery.mid_air_retrieval]`. The shipped devices
  keep their no-op `step()` and constant `(C_D, A)` unless the block is
  present; `scenarios/parachute-recovery/` and every other golden stay
  byte-identical. The wind-relative drag mode is an explicit
  `velocity_source = "eci" (default) | "wind_relative"` switch.
- **FC hardware-portability lock.** All recovery dynamics are sim-side
  (`openbmp-vehicle`/`-physics`/`-models`/`-runner`). The only FC-side change
  (T4 under-canopy damping mode) consumes estimated rates through the existing
  sensor boundary and commands existing effectors; `openbmp-fc` gains **no**
  edge to a forbidden crate; `fc_dependency_tripwire.rs` stays green. The MAR
  corridor and recovery statistics are L7 post-processing, invisible to the FC.
- **Lockstep-clock & no-hot-path-allocation.** Recovery devices keep
  fixed-size state; per-canopy cluster storage is sized at rack build; the
  per-tick snapshot path does not allocate beyond the existing map reuse
  pattern. No `Instant::now`/`SystemTime::now` anywhere; `fc_lints.rs` stays
  green.
- **Four-pillar provenance.** Canopy parameter tables, digitized drop-test
  reconstructions, splashdown surrogate decks, and sea-state defaults land
  under `data/recovery/` (and `data/environment/` for wave spectra constants)
  with sibling `provenance.md`, SHA-256 pin, license status (NTRS material is
  openly published US-government work — recorded explicitly), and a validation
  label. No inline fixtures in `*.rs`; tolerance tables under
  `tests/expected/`. No literal `GM_earth`/`J2`/`R_earth` numerals — symbolic
  only (the descent and footprint paths already follow this).
  `inline_data_tripwire.rs` and `openbmp check-provenance` stay green;
  new source-of-truth paths join the allow-list in the same PR.
- **Forward-only locks.** Descent-dispersion and MAR-corridor propagation
  construct ballistic/descent states only through the existing
  provenance-seeded path; `ballistic_state_compile_fail.rs` stays green and is
  **extended** in WP-16.11 to cover any new corridor-state constructor (locks
  tighten with capability, `00-overview.md` §6).
- **Validation labels.** Per the §3.8 table; the success-criteria values and
  MAR ops statistics are `experimental` user inputs; nothing claims
  `flight-qualified`/`certified`/`operational`, consistent with
  `docs/standards-posture.md` and `docs/safety-boundaries.md`.
- **Requirements traceability.** Every WP adds `requirements.toml` entries
  with verification evidence; `scripts/check_requirements_traceability.py`
  stays green.

---

## 5. V&V plan

The five-layer ladder (`docs/verification.md`) applies: MMS code-verification
→ model verification vs correlations → code-to-code vs open solvers → public
benchmarks → UQ reporting. Tolerance tables ship as TOML under
`tests/expected/`.

### 5.1 Analytic / closed-form / MMS (machine-precision class)

| Case | Setup | Tolerance | Tier / label |
|---|---|---|---|
| Terminal velocity | steady descent, constant `C_D S` ⇒ `v_t = √(2mg/(ρ C_D S))` | rel `< 1e-12` | T1 `validated-toy` |
| Infinite-mass opening load | constant-q fill ⇒ `F_max = (C_D S) q_s C_X` exactly (3.2.3) | rel `< 1e-12` | T1 `validated-toy` |
| Pflanz limit | `X₁(A) → 1` as `A → ∞`; integrated-EOM peak vs Pflanz table across `A` | limit exact; table within `< 5%` (method-consistent) | T1 `validated-toy` |
| MMS on inflation ODE | manufactured `(C_D S)(t)` source term through the descent EOM | observed RK4 order ≥ 3.9 | T1 `validated-toy` |
| Apparent-mass momentum check | impulsive `dm_a/dt` pulse; total momentum bookkeeping (3.2.7) | rel `< 1e-10` | T1 `validated-toy` |
| Snatch closed form | spring-damper line vs impulse-energy `Δv√(k m_e)` (3.3.1), zero damping | peak within `< 2%` (finite-tick) | T2 `validated-toy` |
| Elastic riser bounce | undamped slack-taut element, free bounce: energy conservation | drift `< 1e-9` rel per bounce | T2 `validated-toy` |
| Mortar reaction impulse | `∫F dt = m_pack v_m` exact | rel `< 1e-12` | T2 `validated-toy` |
| Cluster bookkeeping | N identical synchronized canopies ≡ single canopy at `η_N·N·(C_D S)` | byte-consistent force sum | T3 `validated-toy` |
| Two-body pendulum frequency | small-swing capsule-chute vs compound-pendulum `ω ≈ √(g/L_eff)` | period `< 1%` | T4 `validated-toy` |
| von Kármán impact | sphere/wedge closed-form peak accel (3.7.1-3.7.2) | rel `< 1e-9` | T5 `validated-toy` |
| Wave-spectrum moments | realized variance vs `∫S_η dω`; `H_s` recovery | `< 1%` (N-component sum) | T5 `validated-toy` |
| Capture-window geometry | hand-computed engagement offsets/closing speeds | exact boolean table | T6 `validated-toy` |

### 5.2 Code-to-code (open solvers)

| Case | Oracle | License posture |
|---|---|---|
| Calisto descent under drogue+main: descent rate profile, drift with declared wind, touchdown velocity, descent time | **RocketPy** (MIT) on the already-ingested Calisto reference (`scenarios/sounding-rocket/calisto/`) | run externally; compare within `< 2%` on terminal-descent quantities (model-class identical: `C_D S` + trigger lag) |
| Independent descent/touchdown cross-check | **OpenRocket** (GPLv3) | **external process only**, never linked/vendored |
| Splashdown surrogate production path | SPH/VOF open solvers (e.g. DualSPHysics, OpenFOAM) | external producers; OpenBMP ingests the deck with provenance + UQ (`13` §6); GPL codes never linked |

### 5.3 Public-benchmark / method anchors (`research`)

| Case | Benchmark | Honest scope |
|---|---|---|
| Dispersed descent + per-canopy cluster methodology | CPAS/Orion published reconstructions and dispersion methodology (NTRS 20130010409, 20110011397, 20220003982) | digitized published traces (provenance-pinned) bracketed by OpenBMP's dispersed envelope; *methodology* parity, not parameter identity |
| Inflation + apparent-mass model form | MSL POST2 parachute model (NTRS 20130012762) | model-form equivalence (same terms present, same regime behavior); no MSL parameter claim |
| Pendulum mode class | Orion pendulum characterization (NTRS 20170003948) | reproduce the *mode class* (two-body swing period scale, instability-driven growth, damping response) — never the Orion-specific amplitudes |
| Water-impact method | Orion LS-DYNA / Langley Hydro Impact Basin program (NTRS 20190000444) | method anchor for the surrogate pipeline shape (peak-g vs velocity/attitude/wave trends); no structural validation claim |
| MAR sequence realism | Electron recovery reporting (nasaspaceflight.com 2022-05; spaceflightnow.com 2022-05-03) | order-of-magnitude scenario anchor (altitude bands, sink rates, sequence) — press-level, labeled accordingly |

### 5.4 UQ reporting

Every dispersed parameter carries a documented distribution + source; peak
loads and success probabilities report Wilks-bounded quantiles and
Clopper-Pearson intervals through `openbmp-mc`; the ingested surrogate carries
per-entry bias/random margins into the campaign; the credibility record
follows the NASA-STD-7009B accounting that doc `11` owns. A tier earns its
§3.8 label **only** with its evidence row in place; absent an anchor it caps
at `validated-toy` and the PR says so.

---

## 7. Dependencies on other parity docs

- **`01-flexible-multibody-dynamics.md`** — the T4 two-body capsule-chute
  system rides the spatial-vector tree (canopy as a free-flyer body; riser
  elements as external spatial-force pairs). *Hard dependency for WP-16.7
  only;* T1-T3 and T5-T6 do not need it.
- **`02-structural-dynamics-loads-slosh-pogo.md`** — consumer of the new load
  channels (opening/snatch/riser/hook/water-impact peaks) as CLA-style load
  events; the line-element formulation is kept consistent with `02`'s
  structural-element conventions.
- **`03-aerodynamics-database-and-cfd-coupling.md`** — airspeed/wind-relative
  convention for the drag adapters; canopy `C_T/C_N(α_t)` tables and the
  forebody wake-deficit entry live in the `03` deck format when distributed
  data exists.
- **`06-gnc-coupled-mimo-and-control.md`** — the under-canopy damping mode
  joins the gain-schedule architecture; swing-energy/rate objectives only.
- **`08-environment-gravity-and-frames.md`** — atmosphere + GRAM-style
  dispersions and the wind models that drive descent drift; the sea-state
  model is a sibling environment block and follows `08`'s dispersion
  conventions.
- **`09-sensors-navigation-and-actuators.md`** — baro/altimeter-triggered
  deployment through the estimator (not truth), RCS MIB/PWPF effectors for
  T4 mitigation, measurement time-tags for deploy-decision realism.
- **`11-monte-carlo-uq-and-validation.md`** — dispersion objects, Wilks/
  Clopper-Pearson statistics, 7009B credibility records, and the
  reconstruction-validation pattern for the drop-test anchor case.
- **`12-determinism-realtime-and-compute.md`** — the recovery-ops campaigns
  run on the deterministic parallel-MC substrate; byte-stable ensemble
  statistics.

---

## 8. Open-source leverage

| Tool / dataset | License | Use mode | What |
|---|---|---|---|
| **RocketPy** | MIT | **code-to-code / port** | parachute descent (`C_D S` + trigger lag) on the already-ingested Calisto reference; descent-rate/drift/touchdown cross-check |
| **OpenRocket** | GPLv3 | **external process only** | independent descent/touchdown comparison — never linked/vendored |
| **NASA NTRS papers** (CPAS 20130010409 / 20110011397 / 20220003982; MSL POST2 20130012762; Orion pendulum 20170003948; Orion water impact 20190000444) | openly published (US Gov) | **ingest (digitized) / method anchor** | dispersion methodology, model forms, reconstructed traces with provenance |
| **Knacke, *Parachute Recovery Systems Design Manual*** + **Ewing/Bixby/Knacke, *Recovery Systems Design Guide* (AFFDL-TR-78-151)** | openly available (US Gov-funded manuals) | **port (equations) / parameter tables** | fill constants, `C_X`, `X₁` curves, cluster efficiency, canopy-type tables |
| **DualSPHysics / SPlisHSPlasH / OpenFOAM (interFoam)** | LGPL / MIT / GPLv3 | **external producers** | optional high-fidelity water-impact decks ingested per the solver-consumer rule; GPL/LGPL codes never linked into the workspace |
| **Pierson-Moskowitz (1964) / JONSWAP (Hasselmann 1973) spectra** | published literature | **port** | sea-state spectrum + standard parameterizations |
| **NASA `42`** | NOSA | **reference** | sanity reference for two-body/tether-class dynamics setup conventions |

License hygiene: everything ported/vendored is permissive or openly published
government work; GPL tools are external-comparison or external-producer only.
`cargo deny` enforces this.

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, green on the `13` §2 gate set.

### WP-16.1 — Parametric CdS(t) inflation + opening loads

- **title:** Replace instantaneous drag-area swap with fill-time inflation and infinite/finite-mass opening loads.
- **goal:** Give `RecoveryModel::step` its reserved purpose: advance `(C_D S)(τ)`
  per §3.2 (eqs 3.2.1-3.2.5), publish per-tick canopy load, and verify the
  integrated peak against the Pflanz closed form. This is the single highest
  parity-per-effort item: it turns "a drag number" into "a load case" and is
  the foundation every later tier reads.
- **fidelity_tier:** T1
- **depends_on:** []
- **new_crates:** []
- **touched:** `crates/openbmp-vehicle/src/recovery/` (inflation module +
  `InflationProfile`), `crates/openbmp-models/src/models.rs`
  (`RecoverySnapshot` load channel), `crates/openbmp-runner/src/recovery.rs`,
  `crates/openbmp-scenario` (`[…recovery.<id>.inflation]` block),
  `crates/openbmp-telemetry` (load channels).
- **approach:** §3.2; integrated-EOM peak primary, Pflanz (3.2.4-3.2.5) as the
  cross-check table; growth shape and constants scenario-declared.
- **acceptance:**
  - inflation off by default; `scenarios/parachute-recovery/` golden byte-identical.
  - infinite-mass peak matches 3.2.3 to rel `< 1e-12`; Pflanz limit `X₁ → 1` exact; integrated vs Pflanz `< 5%` across the `A` sweep (tolerance table).
  - MMS on the inflation-coupled descent EOM: observed order ≥ 3.9.
  - an opt-in scenario exercises deploy → fill → steady descent end-to-end; all `13` §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line (self-deceleration physics).
- **est_effort:** 2-3 weeks.
- **parity_ceiling:** engineering-method loads only; no FSI inflation physics, no drop-test-calibrated constants.

### WP-16.2 — Apparent mass + wind-relative recovery drag

- **goal:** Add the scalar apparent-mass term (3.2.6-3.2.7) via the existing
  time-varying-mass machinery plus the `−(dm_a/dt)v` force, and give the
  recovery force adapter an opt-in wind-relative airspeed mode so descent
  drift exists. Together these make finite-mass opening behavior and
  splash-point dispersion physically meaningful.
- **fidelity_tier:** T1
- **depends_on:** [WP-16.1]
- **new_crates:** []
- **touched:** `crates/openbmp-vehicle/src/recovery/`,
  `crates/openbmp-vehicle/src/adapters.rs` (`RecoveryRackForceAdapter`
  airspeed mode), kernel mass-channel wiring in `crates/openbmp-sim`,
  scenario schema (`velocity_source`, `apparent_mass = true`).
- **approach:** §3.2 (3.2.6-3.2.7); `m_a`/`dm_a/dt` published in the snapshot;
  wind sampled from the existing `WindModel` surface per the `03` convention.
- **acceptance:**
  - both features off by default; all goldens byte-identical.
  - apparent-mass momentum bookkeeping rel `< 1e-10` under an impulsive `dm_a/dt` pulse.
  - declared constant wind produces the closed-form steady drift velocity to rel `< 1e-9`.
  - all `13` §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 1-2 weeks.
- **parity_ceiling:** scalar added mass only (tensor at T4); `k_a` from textbook tables, not test.

### WP-16.3 — Slack-taut riser line element

- **goal:** A tension-only Kelvin-Voigt line element (3.4.1) with attach-point
  geometry, producing riser tension channels and attach-point moments —
  retiring the zero-moment assumption for opted-in scenarios and providing the
  structural element T2 sequencing and T4 two-body coupling both reuse.
- **fidelity_tier:** T2
- **depends_on:** [WP-16.1]
- **new_crates:** []
- **touched:** `crates/openbmp-vehicle/src/recovery/` (line-element module),
  `crates/openbmp-vehicle/src/adapters.rs` (moment contribution),
  `crates/openbmp-models` (tension channels), scenario `[…recovery.<id>.risers]`.
- **approach:** §3.4; slack-taut switch at tick boundaries; per-leg elements
  with declared `EA`, `L₀`, damping; consistent with `02` element conventions.
- **acceptance:**
  - off by default; goldens byte-identical.
  - undamped bounce conserves energy to rel `< 1e-9` per bounce; slack phase transmits exactly zero force (bit test).
  - attach-point moment matches `r × T·û` hand computation to rel `< 1e-12`.
  - all `13` §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 2 weeks.
- **parity_ceiling:** lumped line (no distributed cable modes); stiffness/damping are declared, not material-test derived.

### WP-16.4 — Deployment sequence, snatch loads, reefing/disreef staging

- **goal:** The §3.3 event chain inside the device: mortar reaction impulse,
  1-DOF bag-strip/line-stretch kinematics, snatch transient through the
  WP-16.3 element (anchored by 3.3.1), and reefed stages with independently
  dispersed cutter times — the full staged-load timeline a CPAS-class sim
  produces.
- **fidelity_tier:** T2
- **depends_on:** [WP-16.1, WP-16.3]
- **new_crates:** []
- **touched:** `crates/openbmp-vehicle/src/recovery/` (sequence state machine,
  reefing stages), `crates/openbmp-runner/src/recovery.rs`, scenario
  `[…recovery.<id>.sequence]` + `[…reefing]`, `crates/openbmp-core` (RCVY RNG
  component slots).
- **approach:** §3.3; per-stage inflation profiles re-enter §3.2 with
  re-sampled `v_s`; cutter-time draws via `DeterministicRng` reserved slots.
- **acceptance:**
  - off by default; goldens byte-identical; empty dispersion set ⇒ byte-identical reruns.
  - mortar impulse `∫F dt = m_pack v_m` to rel `< 1e-12`; snatch peak within `< 2%` of 3.3.1.
  - staged scenario shows the canonical load timeline (snatch < reefed peak < disreef peak ordering configurable and asserted).
  - all `13` §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 3 weeks.
- **parity_ceiling:** kinematic 1-DOF deployment (no line-sail/bag dynamics); cutter dispersions are declared distributions.

### WP-16.5 — Per-canopy cluster modeling

- **goal:** N-canopy clusters with independent §3.2/§3.3 state machines,
  lead-lag fill dispersions, load-share asymmetry, cluster efficiency `η_N`,
  forebody wake deficit, and per-canopy attach geometry — the CPAS
  discriminator (NTRS 20130010409) and the input the pendulum and loads work
  needs.
- **fidelity_tier:** T3
- **depends_on:** [WP-16.4]
- **new_crates:** []
- **touched:** `crates/openbmp-vehicle/src/recovery/` (cluster device),
  `crates/openbmp-models` (per-canopy snapshot entries),
  `crates/openbmp-runner/src/recovery.rs`, scenario `[…recovery.<id>.cluster]`,
  `data/recovery/` (η_N + wake tables with provenance).
- **approach:** §3.5; per-member RNG slots keyed by member index; force/moment
  summed in fixed member order.
- **acceptance:**
  - off by default; goldens byte-identical.
  - N synchronized identical members reproduce the single-canopy force scaled by `η_N·N` bit-consistently.
  - dispersed members show lead-lag with the lead canopy's share exceeding `1/N` during the lag window (asserted statistically over a fixed-seed set).
  - all `13` §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 3 weeks.
- **parity_ceiling:** interference via η_N/wake tables, not canopy-resolved aerodynamics; dispersion magnitudes are textbook/published-method values, not CPAS-calibrated.

### WP-16.6 — Dispersed descent Monte Carlo + drop-test reconstruction anchor

- **goal:** Wire recovery parameters (fill constant, `C_X`, reefing ratio,
  cutter times, line stiffness, cluster lag) into `openbmp-mc` campaigns with
  the `08` wind/atmosphere dispersions; ship the `research`-label anchor case:
  OpenBMP's dispersed envelope bracketing digitized, provenance-pinned
  CPAS-published drop-test reconstructions, plus the RocketPy Calisto descent
  code-to-code case.
- **fidelity_tier:** T3
- **depends_on:** [WP-16.5]
- **new_crates:** []
- **touched:** `crates/openbmp-runner` (campaign hooks),
  `crates/openbmp-mc` (consumer wiring), `data/recovery/` (digitized
  reconstructions + provenance), `scenarios/` (dispersed descent campaign),
  `tests/expected/` tolerance tables.
- **approach:** §3.8 + `11`'s dispersion-object and reconstruction patterns;
  Wilks-bounded peak-load quantiles; Clopper-Pearson on event probabilities.
- **acceptance:**
  - campaign block off by default; single-run goldens byte-identical; campaign reruns byte-identical (parallel MC per `12`).
  - RocketPy Calisto descent: terminal descent rate, drift, touchdown velocity within `< 2%` (tolerance table, provenance-pinned oracle output).
  - reconstruction case: published trace inside the dispersed envelope at the declared quantile; documented as methodology parity, not parameter identity.
  - all `13` §2 gates green.
- **validation_label:** `research` (anchor case); campaign machinery `validated-toy`
- **dual_use_note:** descent dispersion reuses the provenance-seeded state lock; no aimpoint surface.
- **est_effort:** 3-4 weeks.
- **parity_ceiling:** brackets *published* reconstructions only; no claim of CPAS-database-equivalent calibration.

### WP-16.7 — Two-body capsule-chute dynamics + pendulum mode

- **goal:** Canopy as a 6-DOF body on the `01` tree with an apparent-mass
  tensor and `C_T/C_N(α_t)` aerodynamics statically unstable near zero total
  α, coupled to the capsule by WP-16.3 riser elements; reproduce the
  canopy-instability-driven pendulum mode class (NTRS 20170003948) with the
  compound-pendulum frequency anchor.
- **fidelity_tier:** T4
- **depends_on:** [WP-16.3, WP-16.5, WP-01.1]
- **new_crates:** [] (consumes `openbmp-multibody` from doc 01)
- **touched:** `crates/openbmp-vehicle/src/recovery/` (canopy body
  definition), multibody wiring in `crates/openbmp-runner`, scenario
  `[…recovery.<id>.two_body]`, `data/recovery/` (canopy coefficient tables).
- **approach:** §3.6; riser forces as external spatial forces on both bodies;
  apparent-mass tensor per the parachute 6-DOF practice (MSL model form,
  NTRS 20130012762).
- **acceptance:**
  - off by default; goldens byte-identical.
  - small-swing period matches `2π/√(g/L_eff)` within `< 1%`.
  - statically unstable canopy table ⇒ growing swing; stable table ⇒ decay (mode-class assertions with fixed seeds).
  - momentum/energy bookkeeping across the two-body + line system to rel `< 1e-8` over a coast window.
  - all `13` §2 gates green.
- **validation_label:** `validated-toy` → `research` (Orion mode-class anchor)
- **dual_use_note:** far from line.
- **est_effort:** 4-6 weeks.
- **parity_ceiling:** mode-class reproduction only; no Orion-specific amplitude/parameter claim; canopy aero from tables, not FSI.

### WP-16.8 — Forward-only GNC mitigation under canopy

- **goal:** An under-canopy rate-damping mode in the existing gain schedule:
  estimated body rates in (through the unchanged FC boundary), RCS damping
  commands out, demonstrably reducing WP-16.7 swing amplitude — the
  Orion-class GN&C mitigation question answered forward-only.
- **fidelity_tier:** T4
- **depends_on:** [WP-16.7]
- **new_crates:** []
- **touched:** `crates/openbmp-fc` (gain-schedule entry; NO new forbidden
  edge), scenario gain-schedule block, demo scenario.
- **approach:** §3.6; objective vocabulary attitude/rate/swing-energy only;
  effectors are the `09` RCS models.
- **acceptance:**
  - off by default; goldens byte-identical.
  - `fc_dependency_tripwire.rs` and `fc_lints.rs` green (portability + clock locks intact).
  - with mitigation on, steady-state swing amplitude reduced by a declared factor vs WP-16.7 baseline at fixed seed (tolerance table).
  - estimator-driven (not truth-driven): deploy/damping decisions read estimated state only, asserted by construction.
  - all `13` §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** vehicle-intrinsic objectives only; no terminal-condition surface added.
- **est_effort:** 2-3 weeks.
- **parity_ceiling:** demonstrates the mitigation mechanism; no claim of a flight-tuned controller.

### WP-16.9 — Water-impact loads: momentum tier + ingested surrogate

- **goal:** Touchdown stops becoming load cases: the von Kármán/Wagner
  momentum tier (3.7.1-3.7.2) for sphere/cone/wedge primitives in-repo, plus
  the solver-consumer ingestion path for an externally produced splashdown
  response surface (3.7.3) with per-entry UQ margins — the Orion LS-DYNA
  pipeline shape (NTRS 20190000444) without the proprietary test base.
- **fidelity_tier:** T5
- **depends_on:** [WP-16.2]
- **new_crates:** []
- **touched:** `crates/openbmp-physics` (momentum-impact module),
  `crates/openbmp-models` (surrogate deck trait), `crates/openbmp-runner`
  (touchdown event + load evaluation), `data/recovery/` (surrogate deck schema
  + example synthetic deck with provenance), scenario `[recovery.splashdown]`.
- **approach:** §3.7; impact state assembled from descent state + (optional)
  wave kinematics; surrogate consumed per `13` §6 (ingestion + provenance +
  UQ + coupling + code-to-code note).
- **acceptance:**
  - off by default; goldens byte-identical (ground-radius stop unchanged unless opted in).
  - sphere/wedge closed-form peak acceleration to rel `< 1e-9`.
  - surrogate round-trip: synthetic deck in ⇒ interpolated `a_peak` ± margins out, fail-closed on out-of-envelope queries.
  - all `13` §2 gates green.
- **validation_label:** `validated-toy` (momentum tier); surrogate path `experimental` until a real deck is ingested
- **dual_use_note:** self-loads only; far from line.
- **est_effort:** 3 weeks.
- **parity_ceiling:** no structural response (loads in, not stress out); no test-validated deck shipped — the pipeline is the deliverable.

### WP-16.10 — Sea state + marine-recovery success statistics

- **goal:** Pierson-Moskowitz/JONSWAP wave model with deterministic
  realization (elevation/slope/orbital velocity at splash), feeding WP-16.9
  impact conditions; a recovery-ops campaign over descent × sea state
  evaluating a declared success-criteria table with honest interval
  statistics — the marine-recovery analysis Electron-class operations imply.
- **fidelity_tier:** T5
- **depends_on:** [WP-16.6, WP-16.9]
- **new_crates:** []
- **touched:** `crates/openbmp-physics` (wave module + `WAVE` RNG domain),
  `crates/openbmp-runner` + `crates/openbmp-mc` (ops campaign),
  scenario `[environment.sea_state]` + `[recovery.ops_mc]`,
  `tests/expected/` tolerance tables.
- **approach:** §3.7; finite cosine-sum realization, spectrum moments
  verified; success criteria use site-radius containment vocabulary only.
- **acceptance:**
  - off by default; goldens byte-identical; campaign reruns byte-identical.
  - realized `H_s` and spectral variance within `< 1%` of the analytic spectrum integral.
  - success probability reported with Clopper-Pearson intervals; load quantiles Wilks-bounded; criteria values labeled `experimental` inputs.
  - all `13` §2 gates green.
- **validation_label:** `validated-toy` (machinery); criteria values `experimental`
- **dual_use_note:** containment-vocabulary statistics; far from line.
- **est_effort:** 3 weeks.
- **parity_ceiling:** open-ocean spectra only (no directional spreading validation, no operator sea-state limits); success criteria are user-declared, not operationally derived.

### WP-16.11 — Mid-air-retrieval engagement + post-capture pendulum

- **goal:** The Electron-style MAR sequence forward-only: dispersed descent
  corridor through the capture band (provenance-seeded propagation), geometric
  hook-line capture window with capture probability, and the post-capture
  two-body pendulum transient with peak hook load — closing the recovery
  lifecycle.
- **fidelity_tier:** T6
- **depends_on:** [WP-16.6, WP-16.3]
- **new_crates:** []
- **touched:** `crates/openbmp-runner` (corridor post-processing, capture
  evaluation), `crates/openbmp-vehicle/src/recovery/` (capture-line element
  reuse), scenario `[recovery.mid_air_retrieval]`,
  `crates/openbmp-testkit/tests/ballistic_state_compile_fail.rs` (extended).
- **approach:** §3.7 item list; corridor output is a closed struct of
  time-tagged position/velocity quantiles; carrier point kinematically
  scripted; post-capture swing via the WP-16.3 element.
- **acceptance:**
  - off by default; goldens byte-identical.
  - capture-window boolean table matches hand-computed geometry exactly.
  - post-capture pendulum period within `< 1%` of the analytic estimate; peak hook load matches the impulse+swing closed form within `< 2%`.
  - `ballistic_state_compile_fail.rs` extended to the corridor-state constructor and green — the corridor cannot be built from bare literals and carries no target/aimpoint field (compile-fail proof).
  - all `13` §2 gates green.
- **validation_label:** `validated-toy` (dynamics); engagement statistics `experimental`
- **dual_use_note:** cooperative-recovery feasibility volume; forward-only lock extended (tightened) in this WP.
- **est_effort:** 3-4 weeks.
- **parity_ceiling:** press-level sequence anchor only; no helicopter performance model, no operational capture-rate claim.

---

## 10. References

1. Knacke, T. W., *Parachute Recovery Systems Design Manual*, Para Publishing,
   1992 (NWC TP 6575) — fill time, opening-force coefficients, Pflanz method,
   cluster efficiency. (Already the cited basis of the shipped drag model,
   `crates/openbmp-vehicle/src/recovery/mod.rs`.)
2. Ewing, E. G., Bixby, H. W., Knacke, T. W., *Recovery Systems Design Guide*,
   AFFDL-TR-78-151, 1978 — canopy-type parameter tables, snatch-load methods.
3. NTRS 20130010409 — CPAS engineering-development-unit analysis: per-chute
   cluster modeling and dispersion methodology (Orion CPAS program).
4. NTRS 20110011397 — CPAS simulation hierarchy and drop-test reconstruction
   practice.
5. NTRS 20220003982 — Orion Descent Simulation System-class modeling overview.
6. NTRS 20130012762 — MSL parachute models in POST2, including apparent-mass
   formulation (JPL).
7. NTRS 20170003948 — Orion main-cluster pendulum characterization and GN&C
   mitigation study.
8. NTRS 20190000444 — Orion water-impact LS-DYNA modeling validated at the
   NASA Langley Hydro Impact Basin.
9. von Kármán, T., "The Impact on Seaplane Floats During Landing," NACA
   TN-321, 1929; Wagner, H., ZAMM 12, 1932 — water-entry momentum theory.
10. Pierson, W. J., Moskowitz, L., JGR 69(24), 1964; Hasselmann, K. et al.,
    JONSWAP, Deutsches Hydrographisches Institut, 1973 — sea-state spectra.
11. nasaspaceflight.com, May 2022 — Electron "There And Back Again" booster
    recovery: drogue/main descent and helicopter mid-air-retrieval attempt.
12. spaceflightnow.com, 2022-05-03 — Electron helicopter catch-and-release
    reporting (capture-band altitudes, sink rates, post-catch release to
    marine recovery).
13. Ceotto, G. et al., "RocketPy: Six Degree-of-Freedom Rocket Trajectory
    Simulator," *Journal of Aerospace Engineering* 34(6), 2021 (MIT license)
    — the descent-phase code-to-code oracle on the ingested Calisto reference.
14. `docs/verification.md` — five-layer V&V ladder and label definitions;
    `docs/descent-and-entry-profiles.md` — descent/recovery phase vocabulary;
    `docs/scenario-format.md` — recovery scenario schema;
    `docs/standards-posture.md`, `docs/safety-boundaries.md` — permanent
    non-claims.

---

*Companion documents:* `00-overview.md` (constitution) ·
`13-agent-execution-playbook.md` (gates, template, WP schema) · `01`, `02`,
`03`, `06`, `08`, `09`, `11`, `12` (dependencies, §7).
