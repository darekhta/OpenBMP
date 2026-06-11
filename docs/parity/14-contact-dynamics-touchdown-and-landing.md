# Contact Dynamics, Touchdown & Landing

**Status:** `experimental` (design intent; this document ships no code).
**Audience:** the engineer or LLM agent implementing the parity work packages.
**One-line summary:** add a deterministic unilateral-contact and constraint
layer on the doc-`01` Featherstone tree — compliant/penalty contact with
Coulomb friction and stiction, joint stops, backlash, latches, and closed
kinematic loops via the Constraint-Force-Equation approach — then build the
landing application on it: ground plane, leg-strut/crush-core elasto-plastic
gear, footpad friction and soil compliance, engine-cutoff-height logic,
touchdown-state-dispersed tipover/gear-load Monte Carlo, and stochastic
sea-state droneship deck motion scoped to a declared cooperative platform.

> Read `00-overview.md` (parity definition, invariants, DAG) and
> `13-agent-execution-playbook.md` (the §4 template this doc follows and the §5
> work-package schema) before this document. This is dimension `14`, an
> extension of the original `01`–`12` gap ledger. The contact substrate (§3.2)
> is the shared dependency that the forthcoming RPOD/docking and
> stage-separation/fairing-contact design docs in this series will consume;
> plume ground-effect forces are consumed *from*
> `15-plume-environments-and-supersonic-retropropulsion.md`.
> **Boundary rule: `01-flexible-multibody-dynamics.md` stays a tree with
> welded→free release; this document owns everything that touches.**
> Registering this file in `scripts/check_parity_docs.py` (`EXPECTED_FILES`)
> lands with the series-index PR that admits the 14+ docs to the validated set.

---

## 1. Parity target & ceiling

### 1.1 Target capability

The reference organizations treat touchdown as a *dynamics regime*, not a stop
condition. JPL's DARTS/DSENDS runs O(N) recursive multibody dynamics with
non-smooth contact/collision and time-varying topology for EDL simulation
(dartslab.jpl.nasa.gov; IEEE Aerospace, document 1035313). NASA's POST2 carries
Constraint-Force-Equation (CFE) joint models — spherical/revolute/translational
joints and loop closures imposed as constraint forces between separately
integrated vehicles — validated code-to-code against ADAMS (NTRS 20150007891).
NASA landing programs run touchdown-dynamics Monte Carlo over dispersed
touchdown states (Viking, NTRS 19750023452) and modern Simscape-multibody
lander models with footpad contact, soil compliance, and tipover boundaries
(NTRS 20230003031). SpaceX lands boosters inside published 15 m (droneship) /
30 m (land) dispersion budgets and describes touchdown as the hard part of
propulsive landing (Blackmore, NAE *The Bridge* / Frontiers of Engineering,
2016); its simulation job postings list multi-body physics as a core competency
(greenhouse.io job 8487534002). Shipboard-landing programs model deck motion as
wave-spectrum-driven stochastic 6-DOF platform kinematics with deck-relative
touchdown criteria (JGCD, doi 10.2514/1.G009512).

OpenBMP's audit verdict for this dimension is **absent** (touchdown is
currently a `GroundImpact` stop condition, §2). The target closes three gap
rows:

1. **Contact and constraint multibody dynamics** (discipline),
2. **Touchdown, landing-gear, and moving-platform landing dynamics**
   (discipline),
3. **Propulsive-landing terminal phase: touchdown contact, leg/crush-core,
   tipover MC, moving-deck** (lifecycle).

The capability target, stated honestly in two halves:

- **At-parity (method) with the POST2/ADAMS-penalty/Simscape class of landing
  contact practice:** compliant (penalty) unilateral contact with
  Hertz/Hunt-Crossley normal laws and Coulomb friction with true stiction;
  joint stops, backlash, and latches as scalar penalty primitives; closed
  kinematic loops via the CFE approach with Baumgarte stabilization on the
  doc-`01` spatial-vector tree; elasto-plastic leg/crush-core gear; soil
  compliance; touchdown-dispersed tipover/gear-load Monte Carlo; stochastic
  sea-state deck motion. These are the formulations production landing sims
  actually use, and they are pure simulation with no proprietary-data
  dependency (`00` §1.1), so the in-repo tier can go deep.
- **Explicitly NOT targeted: DARTS-class non-smooth (measure-differential-
  inclusion / event-driven impulsive) contact with time-varying topology.**
  Penalty + CFE keeps fixed topology and fixed-step determinism — contact
  on/off is a force-law regime change, not a topology event — which is exactly
  what OpenBMP's byte-determinism invariant wants. An optional implicit
  cone-complementarity tier on the already-vendored Clarabel solver (T6, §3.8)
  is the held-open path toward the stiff/impulsive end; the full non-smooth
  machinery is a documented non-goal, not a deferred promise.

### 1.2 The honest parity ceiling

Validation parity is structurally unreachable; the open substitute for each
item is the deliverable:

1. **ADAMS itself is proprietary.** Direct ADAMS co-simulation requires a
   license. *Open substitute:* code-to-code against open multibody engines
   (Project Chrono, Drake; MBDyn as an external process, reusing the doc-`01`
   oracle), plus the **published** CFE-vs-ADAMS comparison cases in NTRS
   20150007891 as a public benchmark for the constraint layer.
2. **Flight landing-gear parameters are proprietary.** Falcon-class leg
   geometry, honeycomb crush curves, and oleo orifice maps are not public.
   *Open substitute:* Apollo LM gear architecture and crush-core behavior from
   NASA TN D-6850, Viking gear/touchdown parameters from NTRS 19750023452, and
   generic aluminum-honeycomb plateau correlations from open literature — all
   ingested with provenance as *synthetic-representative* vehicles, never a
   fielded parameter set (`00` §3.6).
3. **Real droneship deck-motion logs and site sea states are proprietary.**
   *Open substitute:* standard wave spectra (Pierson-Moskowitz, JONSWAP) through
   published barge/ship response-amplitude-operator (RAO) methods; the deck
   model is validated as a *method* (spectral moments, stationarity, RAO
   transfer) — never as a specific ship at a specific time.
4. **Soil/regolith bearing data per landing site is not openly characterized.**
   *Open substitute:* Bekker pressure-sinkage terramechanics with published
   parameter ranges (Bekker; Wong) and NASA lunar-soil-simulant data.
5. **Tipover probabilities cannot be flight-validated.** SpaceX landing
   outcomes are external observations; no open program publishes its touchdown
   MC against recovered-hardware statistics. *Open substitute:* replicate the
   Viking Monte Carlo *methodology* (NTRS 19750023452), compare tipover
   *boundary trends* to NTRS 20230003031, and use the published 15 m/30 m
   dispersion budgets (Blackmore 2016) only as an order-of-magnitude
   touchdown-state envelope. **A computed tipover probability is a
   method-validated statistic about a synthetic vehicle, never a claim about a
   real one.** Tiers earn `validated-toy` on closed-form/analytic evidence and
   `research` only where a public benchmark (published CFE/ADAMS cases, Viking
   MC, NTRS 20230003031, Housner rocking-block closed forms) anchors them.

---

## 2. Current state in source

Verified against the working tree. The honest summary: **touchdown remains a
stop condition by default, landing guidance primitives exist, footprint Monte
Carlo exists, the `openbmp-contact` substrate exists, and schema-v3 scenarios
can now opt into a half-space `contact` force with scalar diagnostics,
classification, and run-level energy audit; gear-leg contact and
leg/survival-class touchdown outcomes remain open.**

### 2.1 What already exists (the regress-against baseline)

- **Ground impact as termination.** `crates/openbmp-sim/src/stop.rs:124`
  defines `GroundImpact { ground_altitude_m }`: the kernel *stops* at an
  altitude threshold. `crates/openbmp-runner/src/rigid_body.rs:588`
  (`automatic_ground_impact`) arms it by default at sea level;
  `infer_near_surface_geocentric_radius_m` (rigid_body.rs:596) infers a
  near-surface ground radius for separated lanes, and ground-crossing separated
  lanes are marked non-propagating (asserted at rigid_body.rs:4286). Nothing
  models what happens at or after contact.
- **Initial contact substrate.** `crates/openbmp-contact/src/lib.rs` now ships
  the dependency-light L2 primitives for half-space point/sphere kinematics,
  Kelvin-Voigt, Hertz, and Hunt-Crossley normal laws, regularized Coulomb
  friction, explicit penalty-contact stability checks, and contact energy-audit
  closure. It is the crate-boundary and force-law substrate WP-14.1 requires
  for runner integration.
- **Scenario-reachable half-space contact force.** Schema-v3 `[contact]`
  declares point/sphere contact against `z = ground_altitude_m`, normal-law
  parameters, friction smoothing, effective mass, and fixed substeps.
  Scenario load derives/requires the `contact` force-model entry and fails
  closed on the same penalty-contact stability bound as `openbmp-contact`.
  The point-mass and rigid-body runners wire the contact force through the
  existing force accumulator and standard `force.contact.{x,y,z}_n` telemetry,
  publish scalar `contact.gap_m`, `contact.penetration_m`,
  `contact.normal_velocity_m_s`, and `contact.normal_force_n` diagnostics, and
  emit a `RunOutcome.contact` report with `NoContact`, `Rest`, or `Unsettled`
  endpoint classification plus contact elastic-energy/work/dissipation
  closure audit while disabling the legacy terminal `GroundImpact` stop for
  that opt-in scenario. The runner divides the kernel step size by
  `contact.substeps`, so contact evaluation and diagnostics advance on the
  declared fixed substep lattice.
- **A runner-side terminal landing throttle controller.**
  `crates/openbmp-runner/src/separated_landing.rs` is a deterministic
  *scenario-director* controller (its own doc comment: "not flight software…
  does not claim propulsive-landing fidelity"). It closes a throttle loop on
  altitude and radial velocity, optionally adds a lateral PD toward
  `target_position_eci_m`, and issues an engine **shutdown at
  `target_altitude_m`** (separated_landing.rs:154–155) — the seed of the
  engine-cutoff-height logic, but it consumes *truth* state directly and runs
  outside the FC/sensor boundary. This doc's WP-14.5 moves cutoff logic behind
  the boundary.
- **FC-side landing guidance primitive.** `crates/openbmp-fc/src/landing.rs:41`
  (`solve_accel_norm_epigraph`) is the Clarabel SOCP cone primitive that doc
  `07`'s LCvx powered-descent-to-state tier (WP-07.4) builds on. Guidance to a
  touchdown *state* exists as a solver surface; the plant it lands on does not.
- **Footprint Monte Carlo (the descent-sim feeder).**
  `crates/openbmp-runner/src/footprint.rs:479`
  (`landing_footprint_monte_carlo_for_initial_state`, with a checkpointed
  variant at :495) runs the `[landing_footprint.monte_carlo]` dispersion
  campaign for separated bodies, driven by the `openbmp footprint-mc` CLI
  (`crates/openbmp-cli/src/commands/footprint_mc.rs`). Ballistic/footprint
  states are provenance-seeded and locked by
  `crates/openbmp-testkit/tests/ballistic_state_compile_fail.rs`. This is the
  "existing descent sim" whose terminal states feed the touchdown MC (§3.6).
- **Deterministic MC substrate.** `crates/openbmp-mc/src/lib.rs` ships the L7
  campaign crate (deterministic sampling, Welford reducers, Clopper-Pearson
  intervals, `rayon` fan-out) from docs `11`/`12` — the orchestrator the
  tipover campaign consumes.
- **Mission vocabulary already names the phase.**
  `docs/mission-states-vocabulary.md` defines `post_flight.touchdown`
  ("transient state at ground contact") and the Phalcon-9 boostback e2e test
  (`crates/openbmp-cli/tests/phalcon9_orbit_boostback_e2e.rs:239–281`) drives a
  booster lane through `mission.regions.booster.landing_burn` →
  `touchdown_or_safed` — keyed on altitude, with no contact dynamics behind it.
- **Descent decelerators.** `crates/openbmp-vehicle/src/recovery/` ships
  parachute/drogue/airbrake drag devices (Knacke formulation). Descent is
  modeled; arrival is not.
- **Actuator backlash exists; joint backlash does not.**
  `crates/openbmp-vehicle/src/effector/mod.rs` + `linear.rs` carry a 1-DOF
  actuator deadband and fault modes, and doc `09` §3.8 adds
  backlash-on-reversal *inside the servo*. That is a command-path
  nonlinearity, not a mechanical joint-clearance element; this doc owns the
  latter (§3.3).
- **The multibody substrate is designed but not merged.** Doc `01` proposes
  `openbmp-multibody` (L2): spatial-vector tree, ABA/RNEA/CRBA, welded→free
  release, **fixed topology, tree only — explicitly no contact and no closed
  loops**. The crate does not yet exist under `crates/`. Tiers T1/T2 here
  deliberately ride the *existing* single-rigid-body kernel so contact value
  ships before doc `01` lands; T3 is the tree integration.

### 2.2 Maturity summary

| Sub-dimension | Current | Target tier |
|---|---|---|
| Ground contact | `GroundImpact` default; schema-v3 `[contact]` half-space point/sphere force wired through runner with scalar diagnostics and endpoint report | T1 gear/leg contact + friction + leg/survival outcome classification |
| Stiction / rest | none | T2 anchored stiction + rest detection |
| Joint stops / backlash / latches | actuator-path deadband only | T1/T2 scalar penalty primitives |
| Closed kinematic loops | none (doc `01` is tree-only) | T3 CFE + Baumgarte on the tree |
| Landing gear | none (parachutes only) | T2 oleo + crush-core legs / T3 gear DOFs |
| Engine-cutoff-height | runner-side truth-fed director (`separated_landing.rs`) | T2 FC-side cutoff FSM on sensed state |
| Soil / terrain compliance | none | T2/T3 Bekker + slope/DEM pairing |
| Tipover / gear-load MC | footprint MC of ballistic impact only | T4 touchdown-dispersed campaign |
| Moving-deck landing | none | T5 sea-state cooperative deck |
| Implicit/stiff contact | none | T6 (optional) Clarabel cone-complementarity |

---

## 3. Target architecture

### 3.1 Crate map and placement

One new crate; everything else extends existing crates along the layering in
`00` §4:

```text
NEW (proposed; boundary reviewed in WP-14.1 before building):
  openbmp-contact   L2  unilateral contact pairs (gap functions, normal laws,
                        friction/stiction), scalar stop/backlash/latch
                        primitives, CFE loop-constraint solver, contact-stiffness
                        sub-stepping, energy audit. Depends only on
                        openbmp-core/-state/-models (+ openbmp-multibody once
                        doc 01 merges). No up-layer edge; never a dependency of
                        openbmp-fc.

EXISTING (touched):
  openbmp-vehicle   L2  + landing_gear/ module: LandingGearLeg (oleo stage,
                        crush core, footpad), gear rack composition (mirrors
                        the recovery/ pattern)
  openbmp-physics   L2  + sea-state platform kinematics (wave-spectrum
                        synthesis + RAO → prescribed 6-DOF deck pose)
  openbmp-sensors   L3  + weight-on-leg contact discrete + deck-relative
                        altimeter variant (through the doc-09 surface)
  openbmp-fc        L4  + terminal cutoff FSM consuming sensed altitude + WoL
                        discretes (PORTABILITY LOCK: no new forbidden edge)
  openbmp-scenario  —   + [contact], [vehicle.landing_gear],
                        [environment.sea_state_platform], [touchdown.monte_carlo]
                        blocks (all off by default)
  openbmp-runner    L7  + contact-force accumulation wiring, touchdown outcome
                        classification, touchdown-MC campaign driver
  openbmp-cli       —   + `openbmp touchdown-mc` subcommand (parallels
                        `footprint-mc`)
  openbmp-mc        L7  (consumer) campaign sampling/reducers for the tipover MC
```

**Why a new crate.** No existing crate fits: doc `01` scopes
`openbmp-multibody` to the smooth tree on purpose (its V&V ladder is
ABA↔RNEA/energy/momentum; contact would pollute it with set-valued force laws
and geometry queries); `openbmp-vehicle` is vehicle composition, not generic
mechanics; `openbmp-physics` is environment. Contact is a distinct
mathematical object (unilateral, dissipative, regime-switching) with its own
verification ladder, consumed by at least three future docs (RPOD docking,
separation/fairing recontact, this one) — a textbook L2 boundary. The
forthcoming RPOD and separation-contact docs depend on `openbmp-contact`, not
on this doc's landing application.

### 3.2 The contact substrate: compliant unilateral contact (T1/T2)

**Gap function.** Each `ContactPair` is a body-fixed contact geometry (point or
sphere footpad at `r_b` in body axes) against a declared surface (ground
half-space, sloped/DEM terrain patch, or moving deck plane). The signed gap
`g(q)` and its rate `ġ` are evaluated in the surface frame `(t̂₁, t̂₂, n̂)`; for
the deck, the surface frame moves with the prescribed platform pose (§3.5) and
`ġ` is the *relative* normal rate.

**Normal force (penetration δ = max(0, −g)):**

```
Kelvin-Voigt (T1):    F_n = max(0, k_n δ + c_n δ̇)                       (3.2.1)
Hertz (sphere-plane): k = (4/3) E* √R,  F_n = k δ^{3/2}                  (3.2.2)
Hunt-Crossley (T1):   F_n = k δ^n (1 + (3/2) α δ̇),  α ≈ (1 − e)/v_imp   (3.2.3)
```

Hunt-Crossley is the default: damping scales with penetration, so the force is
continuous at contact onset and the model never pulls (no tension), unlike the
clamped Kelvin-Voigt whose release artifact is documented and kept only as the
cheap tier. `e` is the target restitution at reference impact speed `v_imp` —
the closed-form e-vs-α relation is a Layer-1 verification case (§5.1).

**Friction.** Two laws, both deterministic:

```
RegularizedCoulomb (T1):  F_t = −μ F_n tanh(‖v_t‖ / v_ε) v̂_t            (3.2.4)
AnchoredStiction  (T2):   stick:  F_t = −k_t (p − p_a) − c_t v_t,
                                   valid while ‖F_t‖ ≤ μ_s F_n
                          slip:   F_t = −μ_k F_n v̂_t;  anchor p_a reset   (3.2.5)
                          re-stick when ‖v_t‖ < v_stick  (Karnopp window)
```

(3.2.4) is simple and smooth but creeps at rest (documented limitation —
unusable for tipover statics). (3.2.5) is the production answer: a tangential
spring-damper anchored at the stick point `p_a`, breaking to kinetic sliding
when the friction cone is exceeded, re-anchoring inside a Karnopp velocity
window. **Stick/slip transitions are evaluated only at sub-step boundaries
with explicit hysteresis (μ_s > μ_k, v_stick band) — no iterative event
search — so the state machine is byte-deterministic by construction.**

**Stiffness vs the fixed-step integrator.** Penalty contact introduces a
contact frequency `ω_c = √(k_eff/m_eff)`. The kernel stays fixed-step RK4
(`12` invariants); contact runs at a **fixed, scenario-declared integer
sub-step count** `N` per kernel step (`h_sub = h/N`), and scenario load
**fails closed** if the declared `(k, m, h, N)` violates the documented
stability margin `h_sub ≤ 0.1 · 2π/ω_c`. No adaptive sub-stepping, ever.

**Energy audit (calculate-and-isolate).** Every contact element integrates its
dissipated work; the runner closes the budget `ΔE_kinetic + ΔE_potential =
W_contact_stored + W_dissipated` to tolerance each run. This is the cheapest
detector of a wrong sign, a leaking clamp, or an unstable sub-step.

**Trait surface (in `openbmp-contact`):**

```rust
/// One unilateral contact pair: body-fixed geometry vs a declared surface.
/// Pairs are registered at build time and iterated in registration order.
pub struct ContactPair {
    pub body: BodyId,
    pub attach_body_m: Vector3<f64>,   // footpad center, body frame
    pub geometry: ContactGeometry,     // Point | Sphere { radius_m }
    pub surface: SurfaceId,            // GroundPlane | Terrain | Deck(platform)
    pub normal_law: NormalLaw,         // KelvinVoigt | HuntCrossley
    pub friction: FrictionLaw,         // RegularizedCoulomb | AnchoredStiction
}

/// Per-sub-step evaluation; fixed registration order; no allocation.
pub trait ContactForceModel {
    fn evaluate(&mut self, kin: &ContactKinematics, h_sub_s: f64)
        -> Result<ContactForce, ContactError>;
}

pub struct ContactForce {
    pub force_surface_n: Vector3<f64>, // (t1, t2, n) components
    pub in_contact: bool,              // the weight-on-leg discrete source
    pub sticking: bool,
    pub dissipated_j: f64,             // energy-audit contribution
}
```

Contact forces/moments enter the dynamics through the **existing
force-accumulator surface** (the same surface docs `05`/`09` use) — which means
that once WP-09.1's force-based specific-force truth lands, the simulated IMU
*automatically senses touchdown loads correctly*, with no extra plumbing.

### 3.3 Scalar constraint primitives: stops, backlash, latches (T1/T2)

Defined on any scalar coordinate (a joint angle once the tree lands; an
effector or strut coordinate today):

```
JointStop:  τ = −k_s (q − q_max)₊ − c_s q̇ · step(q − q_max)   (one-sided, both ends)
Backlash:   two-sided gap 2b between drive and driven coordinates; zero torque
            inside the gap, penalty spring-damper at either flank
Latch:      monotone engage — a conditional weld that arms once inside an
            (alignment, rate) capture window and NEVER releases within the run
```

The latch is deliberately latched-once (mirroring the AFTS terminate latch in
doc `10` §3.7): gear-lock and docking soft-capture are engage-only events;
release is a *separation* concern owned by doc `01`'s welded→free joint and the
forthcoming separation-contact doc. Joint-backlash here is the mechanical
clearance element; the doc-`09` servo backlash stays a command-path model —
the two compose, they do not duplicate.

### 3.4 Closed kinematic loops: the Constraint Force Equation (T3)

The doc-`01` tree stays a tree. Loop closures (four-bar gear linkages,
telescoping strut + side-brace, latched interfaces) are imposed as **constraint
forces** appended to the ABA-evaluated tree dynamics — the POST2 CFE
methodology (NTRS 20150007891), which has the salient property that it couples
*separately integrated* bodies through computed joint constraint forces. That
matches OpenBMP's separated-lane runner architecture exactly.

```
Φ(q) = 0,    G = ∂Φ/∂q
[ H   Gᵀ ] [ q̈ ]   [ τ − C(q, q̇) ]
[ G   0  ] [ −λ ] = [ −Ġq̇ − 2αΦ̇ − β²Φ ]        (Baumgarte-stabilized index-1)
```

Solved each step by dense LDLᵀ with **fixed pivot order** (n is tens of DOF;
determinism beats asymptotics, same call as doc `01` §3). Baumgarte gains
`(α, β)` are scenario-declared with documented drift bounds; the constraint
residual `‖Φ‖` is telemetered and verified against the Baumgarte prediction
(§5.1). `H` comes from doc `01`'s CRBA; CFE is therefore a strict consumer of
WP-01.1/01.2.

### 3.5 The landing application (T2–T5)

**Ground plane and terrain.** The default surface is the local-horizontal
plane at the scenario's declared landing-site radius (the same *site
radius/state* vocabulary the forward-only lock already permits — `00` §3.7);
options add a declared slope or a DEM tile reusing doc `09`'s
`data/terrain/` ingestion (WP-09.7). Site constants are symbolic — never
literal Earth-radius numerals (`00` §3.5).

**Soil compliance.** Footpad-on-soil replaces the rigid half-space with Bekker
pressure-sinkage bearing:

```
p = (k_c / b + k_φ) z^n_soil          (Bekker; b = footpad width)        (3.5.1)
```

integrated over the pad area into a nonlinear normal compliance, with the
friction cone unchanged and absorbed energy recorded. Parameters come from
published terramechanics ranges with provenance.

**Leg/strut and crush core.** Each leg is, at T2, a *massless strut force
element* between a body hardpoint and its footpad contact (the
production-typical economy for tipover MC); at T3 a prismatic joint in the tree
with the CFE-closed side-brace linkage:

```rust
pub struct LandingGearLeg {
    pub attach_body_m: Vector3<f64>,
    pub strut_axis_body: Vector3<f64>,
    pub oleo: Option<OleoStage>,    // polytropic gas spring + v²-orifice damping
    pub crush: Option<CrushCore>,   // elasto-plastic honeycomb stage
    pub footpad: ContactPair,
}
pub struct OleoStage  { p0_pa: f64, v0_m3: f64, gamma: f64,
                        orifice_c: f64, stroke_max_m: f64 }
pub struct CrushCore  { f_crush_n: f64, stroke_max_m: f64, k_elastic: f64,
                        crushed_m: f64 /* MONOTONE state: never decreases */ }
```

The crush core is the elasto-plastic element: elastic up to the plateau force
`F_crush`, then constant-force crush with the permanent-deformation coordinate
`crushed_m` advancing monotonically (irreversible by construction — unloading
is elastic from the crushed length). The 1-D energy identity `stroke =
E_absorbed / F_crush` is the Layer-2 drop-test verification case (§5.2).
Gear-load *line loads* into the airframe are recovered through doc `02`'s
LTM/OTM machinery, and "touchdown" joins doc `02`'s CLA flight-event forcing
library as a load event.

**Engine-cutoff-height logic.** The terminal-phase FSM formalizes — and, for
opting scenarios, supersedes — the runner-side director in
`separated_landing.rs`:

```rust
pub enum TerminalCutoffTrigger {
    AltitudeAgl { h_cut_m: f64 },          // sensed AGL altitude threshold
    WeightOnLeg { n_of: u8, of: u8 },      // m-of-n leg contact discretes
    AltitudeOrWol { h_cut_m: f64, n_of: u8, of: u8 },
}
```

The FC-side FSM consumes the doc-`09` altimeter and the new **weight-on-leg
discrete** (a contact-sourced boolean published through `openbmp-sensors` with
the standard latency/time-tag treatment of doc `09` §3.9) — never truth state.
This is the realism fix the current director lacks: cutoff decisions move
behind the FC/sensor boundary onto estimated/sensed state.

**Touchdown outcome classification (closed enum):**

```rust
/// CLOSED vehicle-survival outcome vocabulary. There is deliberately no
/// impact-point, miss-distance, range, or aimpoint field (mirrors SilCheck).
pub enum TouchdownOutcome {
    Rest          { max_gear_force_n: f64, max_stroke_m: f64, settle_time_s: f64 },
    Tipover       { time_s: f64 },
    SlideOff      { time_s: f64 },        // left the pad/deck keep-in polygon
    StrokeExceeded{ leg: LegId },
    LoadExceeded  { leg: LegId, force_n: f64 },
    NoContact,
}
```

Rest detection: kinetic energy below a declared floor with all pads sticking
for a declared hold time — evaluated at step boundaries, deterministic.

### 3.6 Touchdown-dispersed tipover / gear-load Monte Carlo (T4)

The Viking pattern (NTRS 19750023452), modernized onto `openbmp-mc`:

- **Touchdown-state source — fed by the existing descent sim.** Either (a)
  terminal states of forward closed-loop descent runs (the Phalcon-9 boostback
  lane; doc `07` LCvx + doc `06` GNC), or (b) a declared touchdown-state
  dispersion block (position offset within the site dispersion radius,
  vertical/lateral velocity, tilt and tilt-rate, mass properties) sampled with
  `DeterministicRng::for_mc_sample` (WP-12.0-c). **Dispersed touchdown states
  follow the same provenance-seed discipline as footprint states**: the sampler
  type is not constructible from bare literals downstream — the
  `ballistic_state_compile_fail.rs` pattern extends to the new constructor
  (§4, §6).
- **Dispersed parameters:** μ_s/μ_k, crush plateau, oleo pressure, soil
  parameters, slope, CG offset, deck phase seed (T5) — each with per-entry
  bias/random margins flowing from the doc-`11` error budget.
- **Per-sample run:** terminal descent (optional) + contact sim →
  `TouchdownOutcome`; campaign statistics with Clopper-Pearson intervals on
  tipover probability, Wilks-sized gear-load percentiles, and order-independent
  Welford merges (docs `11`/`12`). Checkpoint/resume mirrors
  `footprint-mc` (WP-12.2-b).
- **Cheap pre-filter, honest truth:** the static stability margin (CG inside
  the footpad support polygon, energy criterion) classifies the bulk; the
  dynamic contact sim is the truth model for samples near the boundary. Both
  results are recorded so the pre-filter's misclassification rate is itself a
  reported statistic.
- The published 15 m/30 m dispersion budgets (Blackmore 2016) size the
  *input* envelope sanity check only — they validate nothing about our outputs.

### 3.7 Sea-state droneship deck (T5) — declared cooperative platform

Deck motion is a **prescribed stochastic 6-DOF pose** synthesized from a wave
spectrum through platform RAOs — standard shipboard-landing practice (JGCD doi
10.2514/1.G009512):

```
S_PM(ω)  = (A/ω⁵) exp(−B/ω⁴)                  (Pierson-Moskowitz; Hs, Tp)
S_J(ω)   = S_PM(ω) · γ^r                      (JONSWAP peak enhancement)
x_j(t)   = Σ_i √(2 S_η(ω_i) |RAO_j(ω_i)|² Δω) · cos(ω_i t + φ_ij + ψ_j(ω_i))
```

per deck DOF `j` (heave/surge/sway/roll/pitch/yaw), with `N` fixed frequency
bins, RAO magnitude/phase `(|RAO_j|, ψ_j)` from an ingested generic-barge RAO
table, and phases `φ_ij` drawn **once at scenario load** from
`DeterministicRng` with the domain tuple `(seed, platform_id, dof, bin)` —
a frozen phase table summed in fixed bin order every step (no per-step RNG, no
accumulation-order ambiguity). Variance closes against the spectral moment
`m₀ = ∫S dω = Hs²/16` as a Layer-1 case.

```rust
/// CLOSED platform vocabulary — the cooperative-platform lock (§6).
pub enum LandingPlatform {
    GroundPlane    { /* site radius, symbolic */ },
    SlopedTerrain  { dem: TerrainAssetId, slope_deg: f64 },
    CooperativeDeck{ spectrum: WaveSpectrum, rao: RaoAssetId,
                     heading_rad: f64, keep_in: DeckPolygon },
}
pub enum WaveSpectrum {
    PiersonMoskowitz { hs_m: f64, tp_s: f64 },
    Jonswap          { hs_m: f64, tp_s: f64, gamma: f64 },
}
// Deliberately ABSENT: any waypoint/trajectory schedule, any ephemeris or
// pose-time-series input, any constructor accepting a free platform track.
// Deck motion is a stationary stochastic process declared by
// (spectrum, RAO, heading) at the landing site — nothing else is expressible.
```

**Deck-relative touchdown criteria:** contact kinematics are evaluated in the
deck frame (relative normal velocity, relative tilt vs deck normal, footpad
positions vs the deck **keep-in** polygon — a safety keep-in region, the same
posture as AFTS containment, the antithesis of an aimpoint). Guidance-facing
exposure is *only* a deck-relative state through the existing doc-`07`
descent-to-**state** vocabulary plus a deck-relative altimeter/cooperative-
beacon measurement in `openbmp-sensors` — see the lock in §6.

**Plume ground-effect** force/moment increments near the surface are consumed
from `15-plume-environments-and-supersonic-retropropulsion.md` through the
existing force-accumulator surface; this doc owns only mechanical contact.

### 3.8 Fidelity tiers

| Tier | Scope | Earns |
|---|---|---|
| **T0** (current) | `GroundImpact` stop; runner-side truth-fed cutoff director; footprint MC of ballistic impact | (baseline, `checked`) |
| **T1** | `openbmp-contact` substrate on the existing rigid body: half-space ground, Kelvin-Voigt/Hertz/Hunt-Crossley normal, regularized Coulomb, scalar stop/backlash/latch primitives, fixed sub-stepping + fail-closed stiffness check, energy audit | `validated-toy` |
| **T2** | anchored stiction + rest detection; massless-strut legs (oleo + crush core + footpad); weight-on-leg discrete + FC-side cutoff FSM; Bekker soil + slope terrain | `validated-toy` |
| **T3** | contact + CFE loop constraints on the doc-`01` tree (prismatic gear DOFs, four-bar side-brace, latches); published-ADAMS/CFE + Chrono/Drake code-to-code | `research` where the public benchmark anchors; else `validated-toy` |
| **T4** | touchdown-dispersed tipover-probability / gear-load Monte Carlo on `openbmp-mc`, fed by the descent sim; Viking-methodology replication | `research` (method) |
| **T5** | sea-state cooperative deck: spectrum + RAO synthesis, moving-deck contact, deck-relative criteria, deck-phase MC dimension | `validated-toy` (spectral closed forms); `research` only with a published shipboard benchmark case |
| **T6** (optional) | implicit cone-complementarity contact step on vendored Clarabel (deterministic conic solve per sub-step) for stiff stacks; code-to-code vs the penalty tier | `experimental` → `validated-toy` |

Each tier is independently shippable; T1/T2 do not wait for doc `01`.

---

## 4. Invariant preservation

- **Byte-determinism.** All contact arithmetic is pure `f64` with locked
  operand order, no `f64::mul_add`, no wall-clock, no unordered iteration:
  contact pairs evaluate in registration order; stick/slip and latch
  transitions occur only at sub-step boundaries with explicit hysteresis; the
  CFE LDLᵀ uses a fixed pivot order; sub-stepping is a fixed declared integer
  (adaptive stepping is refused); the deck phase table is sampled once at
  scenario load from domain-separated `DeterministicRng` streams
  (`(seed, platform_id, dof, bin)`) and summed in fixed bin order. MC sampling
  uses `for_mc_sample` (doc `12`) with order-independent reducers. The
  cross-libm caveat for `tanh`/`sqrt`-bearing paths matches doc `09` §4; the
  byte-diff gate runs on the reference profile.
- **Byte-stable-by-default.** Without a `[contact]` block, behavior is
  unchanged: `GroundImpact` still terminates the run and every golden archive
  stays byte-identical. `[contact]`, `[vehicle.landing_gear]`,
  `[environment.sea_state_platform]`, and `[touchdown.monte_carlo]` are each
  independent opt-ins; the FC cutoff FSM is armed by its own scenario block and
  the legacy `separated_landing.rs` director remains the default until a
  scenario migrates (the doc-`09` `specific_force_source` migration pattern).
  New telemetry channels (per-leg force/stroke/gap, constraint residual,
  energy audit) appear only when their feature block is on.
- **FC hardware-portability lock.** Contact is plant-side truth.
  `openbmp-fc` gains the cutoff FSM only, consuming sensed altitude and the
  weight-on-leg discrete through the existing HAL/sensor boundary — **no
  `openbmp-fc` dependency on `openbmp-contact`, ever**, and no new edge to any
  forbidden crate. `fc_dependency_tripwire.rs` stays green. The WoL discrete is
  generated sim-side in `openbmp-sensors` from `ContactForce::in_contact`,
  with doc-`09` latency/time-tag semantics.
- **Lockstep-clock & no-hot-path-allocation.** Contact pair tables, leg racks,
  and the deck phase table are fixed-capacity, built at scenario load; the
  per-sub-step path performs no allocation. Time enters only through the
  injected `Clock`. `fc_lints.rs` stays green.
- **Four-pillar provenance.** Gear parameter sets (Apollo/Viking-derived
  synthetic vehicles), honeycomb crush correlations, Bekker soil ranges, RAO
  tables, and wave-spectrum fixtures land under `data/` with sibling
  `provenance.md`, SHA-256 pins, and license notes; tolerance tables live under
  `tests/expected/`; no inline fixtures in `*.rs`; no literal Earth-constant
  numerals (site radius and gravity referenced symbolically per `00` §3.5).
  `inline_data_tripwire.rs` and `openbmp check-provenance` stay green; new
  source-of-truth files are allow-listed in the same PR.
- **Validation labels.** Per the §3.8 table, justified by §5 evidence; a
  tipover probability ships with its Clopper-Pearson interval, Wilks sizing,
  and a NASA-STD-7009B-shaped credibility record (doc `11`), and is labeled as
  a statement about a synthetic vehicle. Nothing claims
  `flight-qualified`/`certified`/`operational`.
- **Forward-only locks tighten.** Landing-to-state is already permitted; this
  doc adds two locks rather than relaxing any: (1) dispersed touchdown-state
  constructors require provenance seeds (the `ballistic_state_compile_fail.rs`
  pattern extended to the touchdown sampler); (2) the **cooperative-platform
  compile-fail tripwire** (§6) proves `LandingPlatform` cannot carry a free
  trajectory/track and that deck state reaches guidance only through the
  closed descent-to-state vocabulary. The `TouchdownOutcome` enum is closed
  with no range/miss-distance/aimpoint variant.
- **Requirements traceability.** Every WP adds `requirements.toml` entries
  with verification evidence; `scripts/check_requirements_traceability.py`
  stays green.

---

## 5. V&V plan

The five-layer ladder of `docs/verification.md` applies: code verification
(MMS/analytic) → model verification vs correlations → code-to-code vs open
solvers → public benchmarks → UQ reporting. Tolerance tables ship as
`expected.toml` per case.

### 5.1 Analytic / closed-form (code verification)

| Case | Setup | Tolerance | Tier |
|---|---|---|---|
| Static rest penetration | mass on Kelvin-Voigt plane: δ = mg/k | `< 1e-12` rel | T1 |
| Elastic Hertz bounce | undamped sphere drop: energy conserved across bounce | `< 1e-6` rel (at declared sub-step) | T1 |
| Hunt-Crossley restitution | measured e vs the α(e, v_imp) closed-form relation | `< 2%` | T1 |
| Incline stiction threshold | block sticks for tan θ < μ_s, slides at a = g(sin θ − μ_k cos θ) | threshold exact; accel `< 1e-9` | T1/T2 |
| Sliding deceleration | block on plane decelerates at μ_k g | `< 1e-9` | T1 |
| Stick-slip oscillator | mass-spring-belt vs Karnopp reference solution | cycle amplitude/period `< 2%` | T2 |
| Joint stop / backlash | pendulum-on-stop restitution; dead-zone width | width exact; restitution `< 2%` | T1 |
| CFE four-bar | constrained kinematics vs closed-form loop solution | `< 1e-9 m`; `‖Φ‖` within Baumgarte bound | T3 |
| MMS on constrained EOM | manufactured `q(t)` with forcing through the CFE system | observed order ≥ design order | T3 |
| Energy audit closure | every contact scenario: ΔE vs stored + dissipated work | `< 1e-6` rel | T1–T5 |
| Rocking block | Housner slender-block: static tip threshold tan θ = b/h; rocking frequency p² = 3g/(4R) | threshold exact; period `< 2%` | T2/T4 |
| Wave-spectrum moments | synthesized deck heave variance vs m₀ = Hs²/16; mean zero-crossing period vs √(m₀/m₂) | `< 1%` (long window, declared N) | T5 |

### 5.2 Model verification / behavioral

| Case | Tolerance | Tier |
|---|---|---|
| 1-D gear drop test: crush stroke = E_absorbed/F_crush; peak load = plateau | `< 1%` | T2 |
| Oleo stage: polytropic spring curve + v² damping vs closed form | `< 1%` | T2 |
| Bekker sinkage vs published pressure-sinkage curves (declared parameter set) | `< 5%` | T2 |
| Quasi-static tipover boundary vs CG-in-support-polygon prediction | boundary within one MC cell | T4 |
| Deck-relative contact: zero relative-velocity touchdown on a heaving deck produces the same gear response as a fixed-plane touchdown | `< 1e-9` | T5 |

### 5.3 Code-to-code (open solvers)

| Case | Oracle | License posture |
|---|---|---|
| Sphere drop / stacked-friction / incline suite | **Project Chrono** (SMC penalty path) | BSD-3; external reference runs, results ingested with provenance |
| Compliant point contact + stiction on identical scenario | **Drake** (compliant point contact / TAMSI-SAP) | BSD-3; external reference |
| Tree + loop-constraint gear deployment | **MBDyn** (doc `01`'s oracle) | GPL — **external process only**, never linked |
| Soft-contact qualitative cross-check | **MuJoCo** | Apache-2.0; documented model-form differences (its friction is solver-regularized) |
| Penalty tier vs implicit cone-complementarity tier (T6) | internal cross-tier | self code-to-code |

### 5.4 Public benchmarks (`research` anchors)

| Case | Benchmark | Note |
|---|---|---|
| CFE constraint layer | published CFE-vs-ADAMS comparison cases, NTRS 20150007891 | reproduce the published joint/loop cases; compare to the published curves, not to a licensed ADAMS run |
| Touchdown MC methodology | Viking lander touchdown-dynamics Monte Carlo, NTRS 19750023452 | replicate the dispersion-in/stroke-load-out methodology on the synthetic vehicle; trend agreement, not vehicle numbers |
| Tipover boundary | NASA lunar-lander Simscape contact + soil + tipover, NTRS 20230003031 | compare tipover-boundary *shape* vs (velocity, slope, μ); order-of-magnitude/trend target |
| Gear architecture envelope | Apollo LM landing-gear experience, NASA TN D-6850 | qualitative envelope for the synthetic gear (stroke/load character) |
| Touchdown-state envelope sanity | 15 m/30 m dispersion budgets (Blackmore 2016) | sizes MC *inputs* only; explicitly not output validation |

Each tier earns its §3.8 label only with the row's evidence in a tolerance
table; absent the oracle, the tier caps at `validated-toy` and the PR says so.

### 5.5 UQ reporting

Per-parameter margins (μ, k_n, F_crush, soil n/k_φ, slope, Hs/Tp) flow into the
doc-`11` error budget; tipover probability and gear-load percentiles carry
Clopper-Pearson / Wilks statements and a 7009B credibility record. The
documented validity envelope excludes hypervelocity impact, fluid-structure
deck slam, and gear thermal effects.

---

## 7. Dependencies on other parity docs

- **`01-flexible-multibody-dynamics.md`** — the substrate boundary: doc `01`
  owns the smooth tree (ABA/RNEA/CRBA, welded→free release, FFRF flex); this
  doc owns everything unilateral. T3 (tree contact + CFE) hard-depends on
  WP-01.1/WP-01.2; T1/T2 deliberately ride the existing rigid-body kernel so
  contact ships in Phase B without waiting. Gear joints become `Joint` entries
  in doc `01`'s tree; the CFE consumes its CRBA mass matrix.
- **`02-structural-dynamics-loads-slosh-pogo.md`** — touchdown joins the CLA
  flight-event forcing library; gear loads recover into line loads through
  LTM/OTM; the crush-core hysteresis follows doc `02`'s transient-integration
  conventions (Newmark/gen-α compatibility where the modal rack is on).
- **`07-trajectory-optimization-and-mission-design.md`** — supplies the
  dispersed terminal states (LCvx/SCvx powered-descent-to-state) the touchdown
  MC consumes; its terminal-condition vocabulary lock is the only path by which
  deck-relative state reaches guidance (§6). No flow in the other direction.
- **`06-gnc-coupled-mimo-and-control.md`** — flies the closed-loop terminal
  phase whose endpoint states feed T4; the cutoff FSM is a small FC consumer
  alongside doc `06`'s stack.
- **`09-sensors-navigation-and-actuators.md`** — the altimeter/velocimeter
  (WP-09.7) and the force-based specific-force truth (WP-09.1, which makes the
  IMU sense gear loads correctly); this doc adds the weight-on-leg discrete
  through doc `09`'s sensor surface with its latency/time-tag semantics.
  Joint backlash (here) vs servo backlash (doc `09` §3.8) are distinct,
  composing models.
- **`08-environment-gravity-and-frames.md`** — site-frame/geodetic machinery
  for the local-horizontal ground plane and slope definitions; symbolic
  constants.
- **`11-monte-carlo-uq-and-validation.md`** — `openbmp-mc` sampling, Wilks
  sizing, Clopper-Pearson intervals, 7009B credibility records for the tipover
  campaign; per-entry parameter margins.
- **`12-determinism-realtime-and-compute.md`** — deterministic parallel
  campaign substrate, `for_mc_sample` domains, order-independent reducers,
  checkpoint/resume; the fixed-sub-step contract is this doc's instance of doc
  `12`'s fixed-step rule.
- **`10-flight-software-in-the-loop-xil.md`** — the cutoff FSM runs behind the
  SIL transport like any FC logic; XIL fault injection exercises stuck/dropped
  weight-on-leg discretes (a new entry in the doc-`10` fault taxonomy).
- **`15-plume-environments-and-supersonic-retropropulsion.md`** — supplies
  plume ground-effect force/moment increments near the surface, consumed here
  through the force-accumulator surface (§3.7).
- **Forthcoming siblings (consumers of §3.2):** the RPOD/docking doc
  (soft-capture compliance, latches, contact during berthing) and the
  stage-separation/fairing-contact doc (recontact/clearance during separation)
  consume `openbmp-contact` directly. Cross-references will bind to numbers
  when those docs land in the series index.

**Ordering.** T1/T2 are Phase-B-independent tracks (no doc-01 dependency); T3
follows doc `01` Phase A/B; T4 follows the Phase-A MC substrate; T5 is
independent after T2.

---

## 8. Open-source leverage

| Tool | License | Use mode | What |
|---|---|---|---|
| **Project Chrono** | BSD-3 | **reference / code-to-code** | penalty (SMC) contact + friction oracle for the §5.3 suite; its contact-parameter conventions inform the scenario schema. External runs; results ingested with provenance. |
| **Drake** | BSD-3 | **reference / algorithm source** | compliant point contact + TAMSI/SAP convex solvers; SAP's convex formulation is the design reference for the optional T6 cone-complementarity tier (re-derived on vendored Clarabel, not linked). |
| **MuJoCo** | Apache-2.0 | **reference** | soft-contact qualitative cross-check with documented model-form differences. |
| **MBDyn** | GPL | **external process only** | already doc `01`'s code-to-code oracle; reused for the gear-linkage loop cases. Never linked or vendored. |
| **parry** (dimforge) | Apache-2.0 | **port / couple** | signed-distance and shape-query primitives for gap functions if/when geometry grows past point/sphere/plane; port the narrow queries needed rather than taking the full dependency on the hot path. |
| **Clarabel.rs** | Apache-2.0 (already vendored) | **couple** | the T6 implicit contact tier's conic solver, reusing the deterministic settings surface from doc `07`. |
| **JPL DARTS/DSENDS** | not open source | **published concepts only** | method anchor for the discipline (recursive dynamics + non-smooth contact for EDL); architecture concepts from publications only — no software access assumed. |
| **NASA POST2 CFE** | not distributed openly | **published methodology + benchmark data** | the CFE formulation and its published ADAMS comparisons (NTRS 20150007891) — clean-room implementation from the paper. |
| **Simscape lunar-lander models** | MathWorks-licensed | **published results only** | NTRS 20230003031 figures/trends as the tipover-boundary benchmark; no MathWorks code reuse. |
| **Wave-spectrum / RAO literature** | textbook/public | **ingest** | Pierson-Moskowitz, JONSWAP closed forms; published generic-barge RAO tables ingested into `data/` with provenance. |
| **USGS/SRTM/PDS DEMs** | public domain | **ingest** | terrain tiles, shared with doc `09` WP-09.7. |

License hygiene per `00` §3.6 and `cargo deny`: everything linked or ported is
permissive; GPL tools remain external comparison processes.

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, green on the `13` §2 gate set.
WP-14.1 carries the crate-boundary review; cross-doc dependencies use the
sibling notation (`WP-NN.t`, doc `NN`).

### WP-14.1 — `openbmp-contact` substrate: ground plane + compliant normal + regularized Coulomb

- **title:** New L2 contact crate with half-space contact, Hertz/Hunt-Crossley/Kelvin-Voigt normal laws, regularized Coulomb friction, fixed sub-stepping, energy audit.
- **implementation_status:** partial substrate plus first runner integration
  implemented and traced by `REQ-CONTACT-001` / `V-CONTACT-001` and
  `REQ-CONTACT-002` / `V-CONTACT-002`: `openbmp-contact` exists as an L2 crate
  with point/sphere half-space kinematics, Kelvin-Voigt, Hertz, and
  Hunt-Crossley normal laws, regularized Coulomb friction, the fixed-step
  stability bound, and energy-audit closure tests. Schema-v3 `[contact]` is now
  scenario-reachable as a point/sphere half-space force in the point-mass and
  rigid-body runners with standard force telemetry, contact diagnostics
  telemetry, load-time stability checks, and a `RunOutcome.contact` report that
  classifies half-space runs as `NoContact`, `Rest`, or `Unsettled` and reports
  a deterministic elastic-energy/contact-work/dissipation closure audit.
  `contact.substeps` now drives the runner kernel step size as well as the
  load-time stability bound. Gear-leg assemblies remain a future slice before
  the contact/landing tier is complete.
- **goal:** The sim stops ending at the ground. A `ContactPair` registry on the
  existing rigid-body kernel evaluates gap/normal/friction forces into the
  force accumulator at a fixed sub-step rate, with a fail-closed
  stiffness-vs-step check at scenario load and a per-run energy-audit closure.
  Foundation for every other WP and for the forthcoming RPOD/separation docs.
- **fidelity_tier:** T1
- **depends_on:** []
- **new_crates:** **`openbmp-contact` (L2)** — first commit is the skeleton +
  placement justification (§3.1): depends only on `openbmp-core`/`-state`/
  `-models`; no up-layer edge; never a dependency of `openbmp-fc`.
- **touched:** `crates/openbmp-contact/**(new)`, `crates/openbmp-runner`
  (force-accumulator wiring, `[contact]` block), `crates/openbmp-scenario`
  (schema + fail-closed stability lint), telemetry channels (gated).
- **approach:** §3.2 (eqs 3.2.1–3.2.4). Registration-order evaluation, fixed
  integer sub-steps, locked operand order, no allocation per sub-step.
  `GroundImpact` remains the default; `[contact]` disables the terminal
  ground-impact stop and routes the compliant force through the force stack.
  Outcome classification (`NoContact`/`Rest`/`Unsettled`) and substep kernel
  integration are wired for the half-space tier.
- **acceptance:**
  - `[contact]` off by default; canonical goldens byte-identical
  - static penetration `mg/k` to `< 1e-12` rel; undamped Hertz bounce conserves
    energy to `< 1e-6` rel; Hunt-Crossley restitution matches the closed form
    to `< 2%` (tolerance tables)
  - sliding block decelerates at `μ_k g` to `< 1e-9`; energy audit closes
  - scenario load fails closed when `(k, m, h, N)` violates the stability bound
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line — generic forward mechanics.
- **est_effort:** 3–4 weeks
- **parity_ceiling:** rigid half-space, point/sphere pads only; no real
  material-pair contact parameters validated.

### WP-14.2 — Scalar stop / backlash / latch primitives

- **title:** One-sided joint stops, two-sided backlash gap, monotone latch elements on scalar coordinates.
- **implementation_status:** partial crate-level primitives implemented and
  traced by `REQ-CONTACT-003` / `V-CONTACT-003`: `openbmp-contact` exposes
  `ScalarStop`, `BacklashGap`, and `MonotoneLatch` with finite-input
  validation, unilateral no-tension clamping, exact configured backlash
  dead-zone width reporting, a closed-form Kelvin-Voigt restitution helper and
  numerical restitution test, and an engage-once latch property test. Runner
  scenario fixtures and tree-joint wiring remain later mechanism slices.
- **goal:** The constraint-primitive vocabulary (§3.3) every articulated
  mechanism needs — gear locks, deploy stops, clearance — defined on scalar
  coordinates now (effector/strut), wired to tree joints in WP-14.6. The latch
  is engage-once by construction (release belongs to doc `01` separation).
- **fidelity_tier:** T1
- **depends_on:** [WP-14.1]
- **new_crates:** none (extend `openbmp-contact`).
- **touched:** `crates/openbmp-contact/src/{stop.rs,backlash.rs,latch.rs}(new)`,
  unit + scenario fixtures.
- **approach:** §3.3. Penalty stop with damping gated on penetration; dead-zone
  flank springs; latch capture window evaluated at step boundaries, monotone
  engaged flag.
- **acceptance:**
  - off by default; goldens byte-identical
  - backlash dead-zone width exact; stop restitution matches closed form `< 2%`
  - latch engages exactly once and never releases within a run (property test)
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 1–2 weeks
- **parity_ceiling:** no real mechanism clearance/preload data validated.

### WP-14.3 — Anchored stiction + rest detection + Housner rocking case

- **title:** True stick/slip friction with stick-point anchoring, Karnopp window, deterministic rest detection.
- **implementation_status:** implemented and traced by `REQ-CONTACT-004` /
  `V-CONTACT-004` plus scenario/runner integration traced by
  `REQ-CONTACT-005` / `V-CONTACT-005`: `openbmp-contact` exposes
  `AnchoredStictionFriction` with deterministic anchor state, static-cone
  breakaway, kinetic sliding, a Karnopp restick window, and tangential
  elastic-energy plus dissipation reporting; incline stiction/sliding
  closed-form helpers; a drive-spring stick-slip oscillator fixture matched
  against an independent ideal Karnopp reference; a consecutive-hold
  `RestDetector`; and `HousnerRockingBlock` threshold/frequency/period
  anchors. Schema-v3 `[contact]` can select
  `friction_law = "anchored_stiction"`, and point-mass/rigid-body contact
  force adapters route it through a time-gated anchor state. Runner contact
  diagnostics carry tangential speed and sticking state, include tangential
  anchor energy in the contact energy audit, and classify rest through a
  kinetic-energy floor plus consecutive sticking hold.
- **goal:** Replace creep-prone regularized friction for statics: tangential
  anchor spring-damper, cone-break to kinetic slip, re-stick window, and a
  rest detector (energy floor + all-pads-sticking hold) — the prerequisites
  for any tipover statement. Adds the Housner rocking-block analytic anchor.
- **fidelity_tier:** T2
- **depends_on:** [WP-14.1]
- **new_crates:** none.
- **touched:** `crates/openbmp-contact/src/stiction.rs`, outcome
  classification in `openbmp-runner`.
- **approach:** §3.2 (eq 3.2.5). Transitions at sub-step boundaries with
  μ_s/μ_k + velocity-window hysteresis; anchor state in the contact state
  vector; no event iteration.
- **acceptance:**
  - off by default; goldens byte-identical
  - incline stiction threshold exact; stick-slip oscillator matches the Karnopp
    reference to `< 2%`
  - rocking-block static tip threshold `tan θ = b/h` exact; rocking period to
    `< 2%`
  - a block at rest on an incline shows zero anchor drift over 10⁴ steps
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** no measured friction-pair (pad/deck/soil) coefficients
  validated.

### WP-14.4 — Landing-gear legs: oleo + crush core + footpads (massless struts)

- **title:** Per-leg strut force elements with elasto-plastic crush cores and footpad contact on the rigid body.
- **implementation_status:** partial implementation traced by
  `REQ-CONTACT-006` / `V-CONTACT-006` and `REQ-CONTACT-007` /
  `V-CONTACT-007`: `openbmp-vehicle` now exposes `LandingGearLeg`,
  `OleoStage`, and `CrushCore` primitives. The leg validates body hardpoints,
  normalizes strut axes, and carries optional footpad geometry; the oleo stage
  implements a polytropic gas curve, quadratic compression damping, and
  closed-form stored-energy reporting; the crush core exposes an irreversible
  absorbed-energy update whose crushed coordinate is monotone and follows the
  plateau identity `stroke = energy / force`, with tests for the oleo
  closed-form curve/integral and crush monotonicity. Schema-v3 rigid-body
  scenarios can declare `[vehicle.landing_gear]`; the runner builds a
  deterministic landing-gear rack, routes it through force and moment adapters,
  disables automatic ground-impact termination for gear scenarios, emits
  `force.landing_gear.{x,y,z}_n` plus per-leg load/stroke/gap/compression/crush
  telemetry, records pinned synthetic gear data SHA metadata, and verifies the
  synthetic 3-D four-leg drop fixture. `RunOutcome.landing_gear` now reports
  NoContact/Rest/Unsettled classification plus a deterministic energy audit;
  the synthetic 3-D drop reaches Rest with all four legs loaded and <1% audit
  residual. Remaining WP-14.4 work: full per-pad `ContactPair` coupling and
  independent line-load recovery evidence.
- **goal:** The T2 landing vehicle: N-leg gear rack (mirroring the `recovery/`
  rack pattern) with polytropic oleo stage, monotone-coordinate crush core,
  and per-pad `ContactPair`s; 1-D and 3-D drop tests; gear loads telemetered
  for doc `02` line-load recovery. Synthetic gear parameters derive from
  Apollo/Viking publications with provenance.
- **fidelity_tier:** T2
- **depends_on:** [WP-14.1, WP-14.3]
- **new_crates:** none (extend `openbmp-vehicle` with `landing_gear/`).
- **touched:** `crates/openbmp-vehicle/src/landing_gear/**(new)`,
  `crates/openbmp-runner` (gear rack wiring), `data/landing_gear/**(new)` +
  `provenance.md`, scenario `[vehicle.landing_gear]`.
- **approach:** §3.5 (`LandingGearLeg`, `OleoStage`, `CrushCore`). Crush
  coordinate strictly monotone; unloading elastic from crushed length.
- **acceptance:**
  - off by default; goldens byte-identical
  - 1-D drop: stroke = `E/F_crush` to `< 1%`; oleo curve matches closed form
    to `< 1%`
  - 3-D four-leg drop reaches `Rest` with energy audit closed; per-leg loads
    telemetered
  - crush coordinate proven monotone (property test)
  - gear data carries `provenance.md` + SHA pin; all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** vehicle-intrinsic; far from line.
- **est_effort:** 3–4 weeks
- **parity_ceiling:** synthetic-representative gear only; no flight gear
  parameters, no drop-test hardware correlation.

### WP-14.5 — Weight-on-leg discrete + FC-side engine-cutoff FSM

- **title:** Contact discretes through the sensor boundary and a portable terminal cutoff FSM.
- **goal:** Move engine-cutoff-height logic behind the FC/sensor boundary: a
  weight-on-leg discrete generated from `ContactForce::in_contact` in
  `openbmp-sensors` (doc-`09` latency/time-tag semantics), and an
  `openbmp-fc` FSM triggering cutoff on sensed AGL altitude and/or m-of-n WoL
  (§3.5). Supersedes the truth-fed `separated_landing.rs` director for opting
  scenarios; the director remains default for golden continuity.
- **fidelity_tier:** T2
- **depends_on:** [WP-14.4, `WP-09.7` (altimeter, doc 09)]
- **new_crates:** none.
- **touched:** `crates/openbmp-sensors/src/contact_discrete.rs(new)`,
  `crates/openbmp-fc/src/terminal_cutoff.rs(new)`,
  `crates/openbmp-runner/src/fc_bridge.rs` (publication wiring), scenario
  `[fc.terminal_cutoff]`.
- **approach:** §3.5 (`TerminalCutoffTrigger`). FC consumes only sensed
  quantities; XIL fault taxonomy gains stuck/dropped WoL (doc `10`).
- **acceptance:**
  - off by default; goldens byte-identical (director path unchanged)
  - cutoff fires within one FC tick of the trigger condition on sensed state;
    a stuck-WoL fault injected through the doc-`10` seam produces the
    documented fallback (altitude trigger)
  - FC portability lock intact (no `openbmp-fc` edge to `openbmp-contact`);
    `fc_dependency_tripwire.rs` green
  - end-to-end scenario: boostback lane lands on gear with FC-side cutoff
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** forward-only; cutoff consumes sensed altitude/contact,
  never a target field.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** no real avionics WoL switch characteristics (bounce,
  debounce timing) validated.

### WP-14.6 — Contact + CFE loop constraints on the doc-01 tree

- **title:** Tree-integrated contact, prismatic gear DOFs, four-bar side-brace via Constraint Force Equation.
- **goal:** The T3 discipline core: contact pairs and stop/backlash/latch
  primitives attach to `openbmp-multibody` tree coordinates; loop closures
  (telescoping strut + side-brace, four-bar linkage) solve through the
  Baumgarte-stabilized CFE system (§3.4) with fixed-pivot LDLᵀ. Validated
  against the published CFE/ADAMS cases and the Chrono/Drake suite.
- **fidelity_tier:** T3
- **depends_on:** [WP-14.1, WP-14.2, WP-14.4, `WP-01.1`/`WP-01.2` (tree + joints, doc 01)]
- **new_crates:** none (extend `openbmp-contact`; consumes `openbmp-multibody`).
- **touched:** `crates/openbmp-contact/src/cfe.rs(new)`, gear-as-joints in
  `openbmp-vehicle/src/landing_gear/`, scenario schema.
- **approach:** §3.4. CFE consumes the doc-`01` CRBA mass matrix; constraint
  residual telemetered; gear deployment exercises stop + latch + loop together.
- **acceptance:**
  - off by default; goldens byte-identical
  - four-bar kinematics to `< 1e-9 m`; `‖Φ‖` within the Baumgarte bound;
    MMS on the constrained EOM shows design order
  - published CFE/ADAMS comparison case reproduced within its tolerance table
    (provenance-pinned digitized reference data)
  - Chrono/Drake code-to-code suite within model tolerance
  - all `13` §2 gates green
- **validation_label:** `research` (published-benchmark anchored); `validated-toy` for surfaces without an oracle
- **dual_use_note:** far from line.
- **est_effort:** 5–8 weeks
- **parity_ceiling:** no licensed-ADAMS co-simulation; DARTS-class non-smooth/
  impulsive contact remains a non-goal.

### WP-14.7 — Soil compliance (Bekker) + slope/DEM terrain pairing

- **title:** Terramechanics footpad bearing and sloped/DEM terrain surfaces.
- **goal:** Replace the rigid plane with Bekker pressure-sinkage bearing under
  footpads (eq 3.5.1), add declared-slope and DEM-tile surfaces (reusing the
  doc-`09` terrain ingestion), and record absorbed soil energy — the
  rough-terrain dimension of the tipover MC.
- **fidelity_tier:** T2/T3
- **depends_on:** [WP-14.4, `WP-09.7` (DEM ingestion, doc 09)]
- **new_crates:** none.
- **touched:** `crates/openbmp-contact/src/soil.rs(new)`, `data/soil/**(new)` +
  `provenance.md`, scenario `[contact.surface]`.
- **approach:** §3.5. Published Bekker parameter ranges with provenance; the
  friction cone is unchanged on soil.
- **acceptance:**
  - off by default; goldens byte-identical
  - sinkage vs published pressure-sinkage curve `< 5%`; energy audit closes
    including soil work
  - sloped-plane stiction threshold consistent with WP-14.3 closed form
  - soil data provenance-pinned; all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** no site-specific soil characterization; simulant-range
  parameters only.

### WP-14.8 — Touchdown Monte Carlo: tipover probability + gear-load percentiles

- **title:** Touchdown-state-dispersed campaign on `openbmp-mc` with `openbmp touchdown-mc`.
- **goal:** The Viking-methodology campaign (§3.6): dispersed touchdown states
  (from forward descent terminal states or a provenance-seeded dispersion
  block) and dispersed parameters drive per-sample contact sims to
  `TouchdownOutcome`; report tipover probability with Clopper-Pearson
  intervals, Wilks-sized gear-load percentiles, static-prefilter
  misclassification rate, and a 7009B credibility record; checkpoint/resume
  parity with `footprint-mc`.
- **fidelity_tier:** T4
- **depends_on:** [WP-14.4, WP-14.3, `WP-11.0`/`WP-11.1` (MC substrate + DoE, doc 11), `WP-12.0-c` (sample domains, doc 12)]
- **new_crates:** none (consume `openbmp-mc`).
- **touched:** `crates/openbmp-runner/src/touchdown_mc.rs(new)`,
  `crates/openbmp-cli/src/commands/touchdown_mc.rs(new)`, scenario
  `[touchdown.monte_carlo]`,
  `crates/openbmp-testkit/tests/ballistic_state_compile_fail.rs` (extend to
  the touchdown sampler).
- **approach:** §3.6. Per-sample seeds via `for_mc_sample`; order-independent
  reducers; the 15 m/30 m published budgets size the input envelope check only.
- **acceptance:**
  - off by default; goldens byte-identical
  - campaign is byte-deterministic across reruns and across worker counts
    (doc-`12` gate); checkpoint/resume verified
  - quasi-static limit reproduces the CG-in-support-polygon tipover boundary;
    Viking-methodology case replicates published trend behavior (tolerance
    table; trend-level claim documented)
  - tipover probability ships with Clopper-Pearson interval + Wilks sizing +
    7009B record
  - touchdown-state sampler not constructible from bare literals downstream
    (compile-fail extension green)
  - all `13` §2 gates green
- **validation_label:** `research` (method-level; Viking/NTRS-20230003031 anchored)
- **dual_use_note:** survival statistics only; closed outcome enum; no
  accuracy-vs-target metric exists; provenance-seed lock extended.
- **est_effort:** 4–6 weeks
- **parity_ceiling:** probabilities are statements about a synthetic vehicle
  and method; no flight-outcome or drop-test statistical validation.

### WP-14.9 — Sea-state cooperative deck platform + deck-relative touchdown (the lock-carrying WP)

- **title:** Wave-spectrum 6-DOF deck motion, moving-deck contact, deck-relative criteria, cooperative-platform compile-fail lock.
- **goal:** The droneship dimension (§3.7): Pierson-Moskowitz/JONSWAP spectra
  through ingested RAO tables synthesize a deterministic prescribed deck pose;
  contact pairs evaluate in the deck frame; outcomes add deck keep-in
  (`SlideOff`); the touchdown MC gains the deck-phase dimension. Carries the
  **new dual-use tripwire** proving the platform vocabulary cannot express a
  free-moving target.
- **fidelity_tier:** T5
- **depends_on:** [WP-14.4, WP-14.8]
- **new_crates:** none (deck kinematics in `openbmp-physics`; pairing in
  `openbmp-contact`).
- **touched:** `crates/openbmp-physics/src/seastate.rs(new)`,
  `crates/openbmp-contact/src/deck.rs(new)`,
  `crates/openbmp-testkit/tests/cooperative_platform_compile_fail.rs(new)`,
  `data/rao/**(new)` + `provenance.md`, scenario
  `[environment.sea_state_platform]`.
- **approach:** §3.7. Frozen phase table at scenario load
  (`(seed, platform_id, dof, bin)` domains), fixed bin-order summation;
  deck-relative gap/rate; deck-relative altimeter + cooperative-beacon
  measurement through the doc-`09` surface; guidance exposure only via the
  doc-`07` descent-to-state vocabulary.
- **acceptance:**
  - off by default; goldens byte-identical
  - heave variance matches `m₀ = Hs²/16` to `< 1%`; zero-crossing period
    matches `√(m₀/m₂)`; synthesis byte-deterministic across reruns
  - zero-relative-velocity deck touchdown reproduces the fixed-plane gear
    response to `< 1e-9`
  - **compile-fail lock green:** `LandingPlatform` rejects any pose
    time-series/waypoint/ephemeris construction; deck state is not consumable
    as an optimizer boundary condition outside the closed vocabulary
  - RAO/spectrum data provenance-pinned; all `13` §2 gates green
- **validation_label:** `validated-toy` (spectral closed forms); `research` only if a published shipboard-landing benchmark case is reproduced
- **dual_use_note:** the line item (§6 item 2): cooperative declared platform,
  stationary-spectrum-only motion vocabulary, keep-in (not aimpoint) polygon,
  new compile-fail tripwire — locks tighten.
- **est_effort:** 4–6 weeks
- **parity_ceiling:** generic-barge RAOs and standard spectra; no real ship,
  sea state, or landing-time deck-motion data validated.

### WP-14.10 — Optional capstone: implicit cone-complementarity contact tier on Clarabel

- **title:** Deterministic convex implicit contact step (SAP-class) as a stiff-contact alternative, code-to-code vs the penalty tier.
- **goal:** For stiff stacks (hard deck, low-stroke gear) where penalty
  sub-stepping gets expensive: a per-sub-step convex contact solve
  (friction-cone complementarity in the SAP/convex formulation) on the
  already-vendored Clarabel with the doc-`07` deterministic settings. Optional;
  ships only if the penalty tier's measured sub-step burden justifies it.
- **fidelity_tier:** T6
- **depends_on:** [WP-14.6]
- **new_crates:** none (extend `openbmp-contact`; Clarabel already vendored).
- **touched:** `crates/openbmp-contact/src/implicit.rs(new)`, scenario
  `[contact.solver]`.
- **approach:** §3.8/§8 (Drake-SAP as the published design reference,
  re-derived; fixed iteration caps and deterministic solver settings; refuse
  convergence-dependent early exit on determinism-labelled runs).
- **acceptance:**
  - off by default; goldens byte-identical
  - implicit vs penalty tier agree on the §5.1 suite within documented model
    tolerance (internal code-to-code tolerance table)
  - byte-deterministic across reruns at fixed iteration caps
  - all `13` §2 gates green
- **validation_label:** `experimental` → `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 4–6 weeks
- **parity_ceiling:** not a non-smooth/impulsive (DARTS-class) solver; no
  claim beyond the shared validity envelope of the penalty tier.

---

## 10. References

**Contact mechanics & multibody**
- R. Featherstone, *Rigid Body Dynamics Algorithms*, Springer, 2008.
- K. H. Hunt & F. R. E. Crossley, "Coefficient of Restitution Interpreted as
  Damping in Vibroimpact," *J. Applied Mechanics* 42(2), 1975.
- D. W. Marhefka & D. E. Orin, "A Compliant Contact Model with Nonlinear
  Damping for Simulation of Robotic Systems," *IEEE Trans. SMC-A* 29(6), 1999.
- G. Gilardi & I. Sharf, "Literature Survey of Contact Dynamics Modelling,"
  *Mechanism and Machine Theory* 37(10), 2002.
- D. Karnopp, "Computer Simulation of Stick-Slip Friction in Mechanical
  Dynamic Systems," *J. Dyn. Sys., Meas., Control* 107(1), 1985; C. Canudas de
  Wit et al., "A New Model for Control of Systems with Friction" (LuGre),
  *IEEE TAC* 40(3), 1995.
- J. Baumgarte, "Stabilization of Constraints and Integrals of Motion in
  Dynamical Systems," *Comp. Methods in Applied Mechanics & Eng.* 1, 1972.
- M. Anitescu & F. Potra, "Formulating Dynamic Multi-Rigid-Body Contact
  Problems with Friction as Solvable LCPs," *Nonlinear Dynamics* 14, 1997 (the
  non-smooth path *not* chosen; context for §1.1/T6).
- A. M. Castro et al., "An Unconstrained Convex Formulation of Compliant
  Contact" (SAP), *IEEE T-RO* 38(2), 2022 (Drake; T6 design reference).

**Production-practice anchors (the external evidence)**
- JPL DARTS Lab — DARTS/DSENDS: O(N) recursive multibody dynamics with
  non-smooth contact/collision and time-varying topology for EDL.
  <https://dartslab.jpl.nasa.gov>; J. Balaram et al., DSENDS, IEEE Aerospace
  (IEEE document 1035313).
- NASA POST2 Constraint-Force-Equation joint/loop models validated against
  ADAMS. NASA NTRS 20150007891.
- NASA Viking lander touchdown-dynamics Monte Carlo. NASA NTRS 19750023452.
- NASA lunar-lander Simscape contact + soil + tipover models. NASA NTRS
  20230003031.
- W. F. Rogers, "Apollo Experience Report — Lunar Module Landing Gear
  Subsystem," NASA TN D-6850, 1972.
- L. Blackmore, "Autonomous Precision Landing of Space Rockets," *The Bridge*
  (NAE Frontiers of Engineering), 46(4), 2016 — 15 m droneship / 30 m land
  dispersion budgets; "touchdown is hard."
- Shipboard-landing deck-motion methodology, *J. Guidance, Control, and
  Dynamics*, doi 10.2514/1.G009512.
- SpaceX simulation engineering posting (multi-body physics as core
  competency), greenhouse.io job 8487534002.

**Touchdown stability, terramechanics, sea state**
- G. W. Housner, "The Behavior of Inverted Pendulum Structures During
  Earthquakes," *Bull. Seismological Soc. of America* 53(2), 1963 (rocking
  block).
- M. G. Bekker, *Introduction to Terrain-Vehicle Systems*, U. Michigan Press,
  1969; J. Y. Wong, *Theory of Ground Vehicles*, 4th ed., Wiley, 2008.
- W. J. Pierson & L. Moskowitz, "A Proposed Spectral Form for Fully Developed
  Wind Seas," *J. Geophysical Research* 69(24), 1964; K. Hasselmann et al.,
  JONSWAP, *Deutsche Hydrographische Zeitschrift* Suppl. A8(12), 1973.
- O. M. Faltinsen, *Sea Loads on Ships and Offshore Structures*, Cambridge,
  1990 (RAO methods).

**Open-source tools:** Project Chrono (BSD-3), Drake (BSD-3), MuJoCo
(Apache-2.0), MBDyn (GPL — external only), parry (Apache-2.0), Clarabel
(Apache-2.0, vendored).

**Upstream OpenBMP anchors:** `docs/verification.md`,
`docs/standards-posture.md`, `docs/safety-boundaries.md`,
`docs/data-provenance.md`, `docs/mission-states-vocabulary.md`,
`docs/descent-and-entry-profiles.md`, `00-overview.md`,
`13-agent-execution-playbook.md`.

---

*Companion documents:* `00-overview.md` (constitution) ·
`01-flexible-multibody-dynamics.md` (substrate; hard dependency for T3) ·
`02`, `06`, `07`, `08`, `09`, `10`, `11`, `12` (couplings) · `15` (plume
ground-effect supplier) · `13` (execution playbook) · forthcoming RPOD/docking
and separation/fairing-contact docs (consumers of the §3.2 substrate).
