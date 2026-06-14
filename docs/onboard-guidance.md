# OpenBMP Onboard Guidance: Executive, Entry, Descent, and Contingency

**Status:** experimental (design intent; this document ships no code).

**Audience:** the engineer or LLM agent implementing the onboard-guidance work
packages, and reviewers checking the forward-only locks.

This document designs the **onboard guidance capabilities that no existing
design covers**: a guidance executive with solver-health fallback ladders,
numerical predictor-corrector entry guidance, an onboard powered-descent
executor (ignition timing + explicit terminal law + guidance thrust authority),
ascent contingency reconvergence, RCS pulse modulation, and the hardening of
the already-implemented PEG ascent family. It is a companion to
[`ascent-guidance.md`](ascent-guidance.md),
[`descent-and-entry-profiles.md`](descent-and-entry-profiles.md),
[`flight-profiles-architecture.md`](flight-profiles-architecture.md), and the
parity series ([`parity/06`](parity/06-gnc-coupled-mimo-and-control.md),
[`parity/07`](parity/07-trajectory-optimization-and-mission-design.md)).

**The one-sentence architecture:** every law in this document is an
**explicit (closed-form or fixed-iteration) method** that generates a
*reference* the existing three-loop autopilot tracks — there is **no new
in-loop optimizer**; numerical optimization stays offline per the `parity/07`
placement lock, and crosses the FC boundary only as a pre-validated I-load.

---

## 1. Scope and non-overlap

### 1.1 What this document designs (nothing else covers these)

| # | Capability | Why it is uncovered |
|---|---|---|
| A | **Guidance executive** — guidance-method moding, solver-health monitors, fallback ladders, bumpless reference handoff | `parity/06` §3.9 designs the *vehicle-level* abort-to-SAFE ladder (WP-06.4-c); the *guidance-method* lattice and multi-law fallback tree are explicitly listed there as future work |
| B | **PEG hardening + reconciliation** — convergence guards, long-burn corrector stabilization, multi-stage thrust integrals | `PegAscentReference` is implemented (`openbmp-physics/src/profile.rs:700`) but `ascent-guidance.md` still lists the explicit family as *reserved*; no design doc specifies the implemented algorithm or its hardening path |
| C | **Ascent contingency guidance** — PEG reconvergence under thrust deficit; selection among pre-validated alternate orbital conditions | WP-06.4-c covers *allocation-level* engine-out (B-column drop) and a target-free SAFE ladder; guidance-level continuation to a still-reachable orbital condition is not designed anywhere |
| D | **Entry guidance law** — numerical predictor-corrector (FNPEG-class) bank-magnitude + bank-reversal logic, corridor-enforcing | `descent-and-entry-profiles.md` ships the open-loop `EntryCorridorReference`; the predictor-corrector is named as future work and never specified; `parity/07` T5 is the *offline* entry optimizer |
| E | **Powered-descent executor** — burn-ignition timing, explicit terminal-descent law (Apollo/APDG-class), guidance thrust authority | WP-07.4/07.5 design *offline* LCvx/SCvx synthesis; WP-07.6 wires the I-load as a *tracked attitude reference*; nobody designs the onboard dispersion-absorbing law, ignition trigger, or a thrust-magnitude command path (guidance has no throttle lever today) |
| F | **RCS pulse modulation + jet selection** — PWPF / phase-plane deadband logic, minimum-impulse accounting | `parity/06` §3.2 scopes RCS jets as allocator columns and explicitly leaves pulse modulation out ("control-input quantization, out of GNC scope"); no other doc owns it |

### 1.2 What this document does NOT design (covered elsewhere — do not duplicate)

- Offline trajectory optimization, all tiers: corrector, shooting, collocation,
  pseudospectral, **LCvx**, **SCvx**, PMP witness — `parity/07` WP-07.0…07.6.
- I-load production, validation, and the FC-side tracked-reference wiring —
  WP-07.6 (this document *consumes* that boundary).
- Control allocation (dense-B WLS), engine-out **allocation** reconfiguration,
  abort-to-SAFE ladder — WP-06.1, WP-06.4-c.
- `q·α` load relief, drift-/load-minimum blending — WP-06.3-a (the descent and
  ascent laws here compose with it; they do not re-implement it).
- Bending/slosh structural filters — WP-06.2-c. TVC actuator plant, tail-wags-
  dog, INDI/INCA — WP-06.3-b.
- Navigation upgrades (tight GNSS/INS, MEKF reset) — WP-06.4-a/b.
- Hardware/system-level redundancy (triplex strings, actuator-side voting) —
  out of project scope by owner decision; the platform's redundancy is
  analytic (estimator lanes, sensor voters).

### 1.3 Vocabulary

Per [`mission-states-vocabulary.md`](mission-states-vocabulary.md) and the
parity invariants: all methods here are **forward** methods — they generate a
reference from the current estimated state toward a **declared flight,
orbital, or recovery state** drawn from the closed `TerminalCondition`-style
vocabulary. No method in this document accepts, stores, or computes a surface
coordinate (§6).

---

## 2. Current state in source (verified anchors)

| Item | Location | State |
|---|---|---|
| `AscentReferenceGenerator` + `AscentState`/`AscentReference` | `openbmp-physics/src/profile.rs:152` | Consumed API; `AscentState` already carries sensed `thrust_accel_m_s2`, `dynamic_pressure_pa`, `mass_fraction` |
| `PitchProgramAscentReference`, `GravityTurnAscentReference` | `profile.rs:182, 274` | Implemented + tested |
| `ClosedLoopInsertionAscentReference` | `profile.rs:352` | Implemented; pitch regulation to an inertial cutoff condition |
| **`PegAscentReference`** | `profile.rs:700` | **Implemented**; linear-tangent explicit guidance with `time_to_go_s()` (`profile.rs:837`) |
| `SequencedAscentReference` | `profile.rs:1056` | Implemented; vertical → pitch-kick → gravity-turn → PEG composite |
| `AscentReferenceGuidance` job; `GuidanceCutoff` publishing | `openbmp-fc/src/guidance.rs:258, 345` | Phase-gated; commander consumes `time_to_go_s` for cutoff events (`commander.rs:191`) |
| `EntryCorridorReference` (band-limited, open-loop) | `openbmp-physics/src/profile.rs` | Implemented per `descent-and-entry-profiles.md` |
| Vinh 6-state lifting entry, Allen-Eggers | `openbmp-physics/src/reentry.rs` | Implemented, benchmarked (Apollo 4, Stardust) |
| Attitude MPC + `deterministic_settings()` | `openbmp-fc/src/mpc.rs:50, 311` | Implemented (feature `mpc`) |
| SOCP epigraph primitive | `openbmp-fc/src/landing.rs:41` | Smoke-level cone primitive (reserved for WP-07.4-a sharing) |
| Min-snap + flat-output attitude construction | `openbmp-fc/src/trajectory.rs:635` | Implemented; the thrust-axis-from-specific-force construction is reused by §4.5 |
| Throttle path | `autopilot.rs` (`EngineDemand`, `throttle_baseline`, max-Q relief) | Throttle is autopilot-only; **guidance has no thrust-magnitude authority** (gap closed in §5.2) |
| I-loads | `openbmp-trajopt/src/iload.rs:135` | Versioned postcard payload: gain tables + reference profile; FC validates envelope CRC |
| FDIR bits 0–14; detector family | `openbmp-fc/src/fdir.rs:19` | Next free bit: 15 |
| Scheduler: declared budgets, overrun events, `JobTimingObserver` | `openbmp-fc/src/scheduler.rs` | Budget-policed cyclic dispatch |

**Documentation drift (fix as WP-OG.0):** `ascent-guidance.md` §"Method 3"
still marks the explicit family *reserved* and lists only two shipped methods;
the code ships five generators including PEG. The doc must be reconciled before
new guidance design lands on top.

---

## 3. Architecture

### 3.1 The generator-family pattern, extended

The platform's one guidance pattern is sound and is kept: a **generator**
(pure, deterministic, `openbmp-physics::profile`) consumes a snapshot of the
estimated state and produces a **reference**; an FC **job**
(`openbmp-fc/src/guidance.rs`) samples the bus, calls the generator, and
publishes topics; the **autopilot** tracks the reference; the **commander**
sequences phases on declared events. This document adds two sibling families
and one supervisor:

```
                         ┌──────────────────────────────┐
                         │      GuidanceExecutive (§4.1) │
                         │  mode lattice · health · fall │
                         └──────┬───────────────────────┘
            selects + monitors  │
   ┌────────────────┬───────────┼────────────────┬──────────────┐
   ▼                ▼           ▼                ▼              ▼
AscentReference  EntryReference  DescentReference  (coast: existing
(5 impls, §4.2/3) (NPC, §4.4)    (executor, §4.5)   attitude-hold)
   │                │                │
   └── ReferenceState (attitude + rate) ── existing three-loop autopilot
                    │                │
                    └── ThrustReference (§5.2, NEW) ── autopilot throttle loop
                                     │
                              GuidanceHealth (§5.1, NEW) ── executive + FDIR
```

New traits mirror `AscentReferenceGenerator` exactly (same crate, same
determinism contract, same fail-closed constructor validation):

```rust
/// Generates a lifting-entry bank/attitude reference (openbmp-physics::profile).
/// The output is a reference the autopilot tracks; corridor limits bound it.
pub trait EntryReferenceGenerator {
    fn entry_reference(
        &mut self,
        state: &EntryGuidanceState,
        time: SimTime,
    ) -> Result<EntryReference, PhysicsError>;
    /// Predictor diagnostics from the most recent call (residual, iterations,
    /// converged). Read by the FC job to publish GuidanceHealth.
    fn diagnostics(&self) -> GuidanceDiagnostics;
}

/// Generates a powered-descent attitude + thrust-acceleration reference.
pub trait DescentReferenceGenerator {
    fn descent_reference(
        &mut self,
        state: &DescentGuidanceState,
        time: SimTime,
    ) -> Result<DescentReference, PhysicsError>;
    fn time_to_go_s(&self) -> Option<f64>;
    fn diagnostics(&self) -> GuidanceDiagnostics;
}
```

`EntryReference` carries `{ bank_angle_rad, q_body_to_eci_xyzw, body_rate_rad_s:
Option<[f64;3]> }` (the quaternion is constructed from the trimmed-attitude +
bank rotation so the existing autopilot tracks it unchanged). `DescentReference`
carries `{ q_body_to_eci_xyzw, body_rate_rad_s: Option<[f64;3]>,
thrust_accel_m_s2: f64 }`.

**Mutability note.** Unlike `AscentReferenceGenerator`, these traits take
`&mut self`: predictor-corrector laws carry warm-start state (previous
solution, previous bank sign). Warm-start state is part of the deterministic
state (reset on phase entry; serialized in golden tests).

### 3.2 Onboard model set (FC-resident, I-load-delivered)

A predictor-corrector integrates the dynamics *inside the FC*. It must not
touch kernel truth models (FC boundary), so it carries its own simplified
models, delivered as data through the I-load and frozen at scenario load:

```rust
/// The FC-resident models a predictor integrates. Deliberately simpler
/// than kernel truth: model mismatch is the realism the NPC must absorb.
pub struct OnboardModelSet {
    /// Piecewise-exponential density: (h_base_m, rho_base_kg_m3, scale_h_m) knots.
    pub atmosphere: OnboardAtmosphere,
    /// Trimmed L/D and C_D·A/m vs Mach knots (lifting entry) or constant
    /// ballistic coefficient (ballistic fallback).
    pub trim_aero: OnboardTrimAero,
    /// Inverse-square gravity with optional J2 (matches estimator adapters).
    pub gravity: OnboardGravity,
    /// Propulsion: Isp(throttle) knots, T_min/T_max, mass model.
    pub propulsion: OnboardPropulsion,
}
```

Fail-closed validation at load: monotone knots, positive densities/areas,
`T_min < T_max`, validity ranges declared and checked against the scenario's
flight envelope (the same scenario-consumer-agreement rule used elsewhere).
Provenance: the I-load `SynthesisMetadata` records which truth models the
onboard set was derived from, so model-mismatch studies are reproducible.

### 3.3 Rate architecture and budgets

Guidance runs on a **major cycle**, tracking on the minor cycle — the
Shuttle-heritage split, already natural in the scheduler:

| Job | Rate (typ.) | Declared budget (typ.) | Notes |
|---|---|---|---|
| Autopilot (three-loop) | 100–1000 Hz | existing | unchanged |
| RCS modulator (§4.6) | autopilot rate | 10 µs | runs inside/just after autopilot priority |
| Descent executor (§4.5) | 10–50 Hz | 50 µs | closed-form law; cheap |
| PEG / ascent (§4.2) | 0.5–2 Hz | 100 µs | matches Shuttle PEG major cycle (~2 s) |
| Entry NPC (§4.4) | 0.5–1 Hz | 500 µs | dominated by fixed-step predictor |
| Guidance executive (§4.1) | 10 Hz | 20 µs | mode logic only |

Between major cycles the autopilot tracks the latest published reference
(latest-wins topic semantics, already the platform behavior). Budgets are
declared per scenario as today; the runner's `JobTimingObserver` measures
actuals, and sustained breach trips `FDIR_BIT_DEADLINE_SLIP` (existing). All
iteration counts below are **capped and deterministic**, so the worst case is
the declared case.

---

## 4. Capability designs

### 4.1 Guidance executive

**Problem.** Five-plus guidance methods now coexist (ascent family, entry NPC,
descent executor, attitude-hold, waypoint). Today the runner wires one job per
phase and the FSM gates them; nothing supervises *method health* inside a
phase, no fallback exists when a law degrades (NPC non-convergence, PEG
divergence, reference NaN), and reference handoff at phase boundaries is
unmanaged (step changes hit the attitude loop unshaped).

**Design.** A single `GuidanceExecutive` job (openbmp-fc) that owns, per
mission phase, an ordered **fallback ladder** of generators and a **handoff
shaper**. It is the only publisher of `ReferenceState`/`ThrustReference` when
enabled (scenario-selectable; the legacy direct-publish path remains the
default until WP-OG.1 lands).

**Mode lattice.** Per phase, the scenario declares a primary law and its
ladder, e.g.:

```toml
[fc.guidance_executive]
enabled = true

[[fc.guidance_executive.phase]]
phase = "lifting_entry"
primary  = "entry_npc"            # §4.4
fallback = ["entry_corridor"]     # existing band-limited reference
# last resort is always the target-free safe reference:
# attitude-hold at trim, declared per phase
safe_reference_q_xyzw = [0.0, 0.0, 0.0, 1.0]

[[fc.guidance_executive.phase]]
phase = "powered_ascent_upper"
primary  = "peg"
fallback = ["pitch_program"]      # I-load reference profile tracking
```

**Demotion rules (per tick, deterministic):**

1. A generator returning `Err` or a non-finite reference is demoted
   immediately (one strike — the autopilot already defends, but the executive
   must not keep calling a faulted law).
2. A generator reporting `converged = false` for `n_confirm` consecutive major
   cycles (default 3) is demoted. Single-cycle non-convergence holds the
   previous reference (predictor-correctors routinely skip a cycle).
3. Demotion is **monotonic within a phase** (no automatic re-promotion;
   re-promotion only via phase transition). This mirrors the WP-06.4-c ladder
   discipline and keeps the transition graph verifiable.
4. Exhausting the ladder publishes the phase's **safe reference**
   (attitude-hold at the declared trim, thrust reference = baseline) and
   raises `FDIR_BIT_GUIDANCE_FALLBACK_EXHAUSTED`; the commander's existing
   FDIR→arming/abort plumbing (and the WP-06.4-c SAFE ladder, when it lands)
   takes it from there. **The executive itself never commands aborts and
   carries no targets** — it selects among scenario-declared reference
   sources, in declared order.

**Handoff shaping.** On any source change (phase transition or demotion), the
executive blends the published reference over a declared window:
`q_pub = slerp(q_prev_law, q_new_law, s(t))`, `s` the smoothstep over
`handoff_blend_s` (default 1.0 s; 0 disables), rates blended linearly, thrust
reference rate-limited by `thrust_ref_slew_per_s`. This is the guidance-level
analogue of WP-06.2-b bumpless gain transfer and composes with it.

**Health publication.** Every major cycle the executive publishes
`GuidanceHealth` (§5.1) with the active law, ladder position, diagnostics of
the active law, and demotion counters — the SIL monitor and FDIR detectors
consume it.

**New FDIR bits (§5.4):** `GUIDANCE_NOT_CONVERGED` (15, detector: GLRT/burst
on the health topic), `GUIDANCE_FALLBACK_ACTIVE` (16),
`GUIDANCE_FALLBACK_EXHAUSTED` (17).

---

### 4.2 PEG consolidation and hardening (ascent)

**Problem.** `PegAscentReference` is implemented and flying in scenarios, but:
(a) `ascent-guidance.md` still calls the family reserved — the algorithm has no
design-of-record; (b) the implementation has not been hardened against the
known PEG failure modes (long-burn corrector instability, staging transients,
small-`t_go` singularity); (c) there is no optimality evidence.

**Design of record (documents the shipped algorithm).** PEG, public family
(Jaggers 1977; McHenry et al. 1979): linear-tangent steering
`λ(t) = unit(λ_v + λ̇ (t − t_λ))`, with velocity-to-go
`v_go = v_target − v − v_grav` and the thrust integrals over remaining burn
time computed from the propulsion model:

```
L = ∫ a_T dt        (Δv magnitude to t_go)         J = ∫ a_T t dt
S = ∫∫ a_T dt²      (position gain)                Q = ∫∫ a_T t dt²
```

closed-form per constant-thrust stage segment (`a_T = a_0/(1 − t/τ)`,
`τ = m/ṁ`), summed across declared stages; predictor: `r_go` from `S, Q` and
the gravity integrals over a conic/Keplerian arc; corrector: update `v_go`
from the cutoff-condition residual; cutoff: `t_go` from `L = |v_go|` — already
the source of `time_to_go_s()` → `GuidanceCutoff`.

**Hardening tiers (the new work):**

- **H1 — convergence guards.** Bound the corrector update
  (`‖Δv_go‖ ≤ k·‖v_go‖`, default k=0.1/cycle); freeze steering (hold last
  λ, λ̇) when `t_go < t_freeze` (default 4 s, Shuttle-heritage) to avoid the
  small-`t_go` singularity; reject and hold on non-finite intermediates;
  report `converged`/residual via `GuidanceDiagnostics`.
- **H2 — long-burn stabilization (SLS-class mods).** Scale `v_go` in the
  corrector and limit the steering tangent, per the public SLS PEG-enhancement
  papers (NTRS 20180002035; Mahajan AAS 25-844): both are one-line guarded
  modifications with scenario-tunable constants; A/B test on a long
  single-burn upper-stage case.
- **H3 — multi-stage integral exactness.** Thrust-integral summation across a
  declared stage table (coast gaps included), replacing any single-stage
  approximation; staging-transient test: corrector residual decays within 2
  major cycles of separation.
- **H4 — optimality evidence.** Cross-check converged PEG trajectories against
  the offline T2 collocation optimum and the WP-07.6 PMP witness on a vacuum
  ascent: PEG's linear-tangent law is the optimal-control form for vacuum
  constant-thrust ascent (bilinear-tangent degenerate case), so the payload
  penalty vs the optimizer must be small and *measured* (tolerance table;
  target < 0.5% propellant on the benchmark case).

**Doc reconciliation (WP-OG.0).** Update `ascent-guidance.md` Method 3 status,
add `ClosedLoopInsertionAscentReference`, `PegAscentReference`,
`SequencedAscentReference` to its implementation summary, and link here for
the hardening design.

---

### 4.3 Ascent contingency: thrust-deficit reconvergence + alternate-condition ladder

**Problem.** On a confirmed thrust deficit (engine-out, stuck throttle), the
allocation layer re-distributes moments (WP-06.4-c) — but guidance keeps
steering toward a cutoff condition that may no longer be reachable. The flown
practice (public record: Saturn V SA-502/Apollo 13 continuations; Falcon 9
CRS-1/Starlink-19 engine-out continuations) is **guidance reconvergence**: the
explicit law re-solves with degraded propulsion, and if the nominal condition
is unreachable, flight continues to a *pre-planned alternate orbital
condition*.

**Design.**

1. **Propulsion-state feedback into PEG (continuous, no moding).** PEG's
   thrust integrals already consume sensed `thrust_accel_m_s2`; the hardening
   in §4.2 makes the stage table live: on `FDIR`-confirmed effector loss
   (consume the WP-06.4-c reconfiguration topic) the stage table's `a_0, τ`
   are rescaled by the surviving-engine count/throttle ceiling, and PEG
   reconverges naturally — same equations, degraded coefficients. Burn time
   extends; `time_to_go_s` grows; cutoff stays condition-triggered. **This is
   the whole nominal path** — explicitly *not* a new solver.
2. **Reachability check (per major cycle, deterministic).** After
   reconvergence, compare PEG's predicted Δv-to-go `L(t_go)` against
   remaining-propellant Δv capacity (from the mass model + Isp knots) with a
   declared reserve margin: `L ≤ Δv_remaining − Δv_reserve`. Hysteresis: the
   condition must fail `n_confirm` consecutive cycles (default 5) to declare
   the active target unreachable.
3. **Alternate-condition ladder (pre-validated, offline-synthesized).** The
   scenario/I-load declares an *ordered* list of alternate cutoff conditions,
   each an entry of the **closed `TerminalCondition` orbital vocabulary only**
   (e.g. lower circular orbit; elliptical with declared perigee floor — the
   perigee floor is itself validated at load: alternates below the declared
   safe-perigee floor are rejected fail-closed). Each alternate was validated
   offline by the doc-07 pipeline (corrector/collocation run proving
   feasibility envelopes) and carries the standard audit metadata. Onboard
   logic on unreachable: walk the ladder in order, retarget PEG to the first
   alternate whose reachability check passes, raise
   `FDIR_BIT_GUIDANCE_TARGET_RETARGETED` (18), publish the new target class in
   `GuidanceHealth`. Walking past the ladder end (nothing reachable) →
   executive fallback exhaustion (§4.1) → the **target-free** WP-06.4-c SAFE
   ladder. **The SAFE machine itself remains target-free per WP-06.4-c; the
   ladder here is nominal-guidance moding among declared orbital conditions,
   never part of the abort path, and never a surface condition.**

**Compile-fail tripwire (new):** the alternate-ladder type is
`Vec<AlternateCutoff>` where `AlternateCutoff` wraps only the orbital-element
variants of `TerminalCondition`; constructing it from `RendezvousState` or any
non-orbital variant fails to compile (proving test in `openbmp-testkit`,
extending `ballistic_state_compile_fail.rs` conventions).

**V&V:** scripted thrust-deficit injection (doc-10 SIL): (a) deficit small →
nominal condition reached, longer burn (compare insertion elements vs
no-fault golden within tolerance); (b) deficit large → ladder retarget, flight
continues to alternate, elements match the alternate's offline-validated
envelope; (c) deficit catastrophic → ladder exhausted → safe reference +
FDIR bits, **no divergent steering**. Monte Carlo over failure time × deficit
magnitude (doc-11 harness) maps the reconvergence envelope; the envelope plot
is the WP deliverable.

---

### 4.4 Entry guidance: fixed-iteration numerical predictor-corrector (FNPEG-class)

**Problem.** Lifting entry today tracks an open-loop corridor reference: no
dispersion absorption, no terminal-state regulation. The flown state of the
art (Orion PredGuid lineage; Lu's FNPEG as the published reference method) is
a numerical predictor-corrector commanding bank magnitude, with bank-sign
reversals managing the out-of-plane component.

**Forward-only formulation.** The standard FNPEG corrector nulls a
*range-to-site* residual. This platform's vocabulary has no range-to-site; the
equivalent **inertial-state formulation** is used instead — identical
machinery, residual expressed against the declared terminal state:

- **Terminal condition:** `EntryTerminalState { radius_m, speed_m_s,
  flight_path_angle_rad }` + a declared **descent plane** `plane_normal_eci`
  (unit vector, provenance-tokened, synthesized offline like every recovery
  condition). The predictor stops at the terminal **energy**
  `e_f = μ/r_f − v_f²/2`; the corrector nulls the **in-plane arc residual**
  `Δs = s_pred(e_f) − s_declared` where `s` is the great-circle arc subtended
  *in the declared inertial plane* between the current position and the
  declared terminal inertial position of the recovery state (the
  `RendezvousState`-class vocabulary already allowed for rendezvous targets).
  No geodetic field exists anywhere in the type (compile-fail tripwire).

**Predictor.** Integrate the 3-DoF Vinh equations over normalized energy
`e` (monotone during entry — removes time as the independent variable, the
standard FNPEG trick), with the **onboard model set** (§3.2), the current
estimated state as the initial condition, and the parameterized bank profile

```
σ(e) = σ_0 + (e − e_0)/(e_f − e_0) · (σ_f − σ_0)        (linear in energy)
```

(`σ_f` declared, default 60°; `σ_0` is the single unknown). Integration:
fixed-grid RK4, `N_pred` = 250 steps (declared; deterministic), running in the
guidance major cycle's declared budget. Quasi-equilibrium-glide soft
constraint per Lu: when the predicted flight-path-angle rate exceeds the QEG
band, the effective vertical-lift command is biased toward the glide solution
(damps phugoid skipping; pure data-flow, no iteration).

**Corrector.** Newton on the scalar residual `Δs(σ_0)`:
`σ_0 ← σ_0 − Δs / (∂Δs/∂σ_0)`, sensitivity by one-sided finite difference
(one extra predictor run), **iteration cap 3 per major cycle**, warm-started
from the previous cycle's solution (first cycle: from the I-load reference
profile). Convergence = `|Δs| < tol_s` (declared, default 2 km equivalent
arc). Non-converged cycles hold the previous profile and report
`converged = false` (executive demotion logic, §4.1, after `n_confirm`).

**Corridor enforcement (primary, vehicle-intrinsic).** Before publication the
bank command is clipped to the **corridor band** from the existing
`EntryCorridorReference` limits: heat-rate, load-factor, and dynamic-pressure
ceilings map to a minimum vertical-lift fraction → maximum bank magnitude
`σ_max(e)`; equilibrium-glide floor maps to `σ_min(e)`. Predicted-breach
lookahead: if the predictor's trajectory breaches a corridor limit, the
profile is re-corrected against the *active-constraint* arc first (lift-up
priority). Corridor limits dominate terminal-state regulation by
construction — identical posture to `descent-and-entry-profiles.md` ("tracks
heat-rate and load-factor limits, not a place").

**Lateral logic.** Bank sign from the out-of-plane velocity component
`v_oop = v · plane_normal_eci`: reverse sign when `|v_oop|` exceeds a
velocity-scheduled deadband (declared knots), with a minimum interval between
reversals (default 30 s) and a rate-limited roll-through (the autopilot's
existing rate limits shape the reversal). This is corridor-of-the-plane
steering — inertial-plane geometry, no ground track anywhere.

**Reference construction.** The published `ReferenceState` quaternion is the
trimmed attitude (trim α from the onboard aero table at current Mach) rolled
about the velocity vector by the commanded bank — the same construction
pattern as `GravityTurnAscentReference` (velocity-aligned frame) composed with
a roll; body-rate feed-forward from the bank-rate limit.

**Fallbacks (executive ladder):** NPC → band-limited corridor reference
(existing, open-loop) → phase trim attitude-hold. Ballistic vehicles never
enter this law (existing fail-closed rule: `lifting_entry` requires
lift-capable aero).

**V&V:**
- *Verification:* predictor self-consistency (predictor flown open-loop in the
  kernel with matched models reproduces its own prediction to integration
  tolerance — catches frame/sign errors); corrector quadratic-convergence test
  on the smooth case; determinism goldens (bit-stable two-run).
- *Validation:* published FNPEG verification cases (Lu 2014; the
  CAV-H/low-lifting cases from the predictor-corrector literature) re-expressed
  against terminal energy-state; Apollo-4-class entry (already a repo
  benchmark) flown closed-loop with dispersed initial states: terminal-state
  capture within declared tolerance, corridor never breached. Cross-check vs
  offline SCvx (WP-07.5) entry trajectories where both converge: NPC terminal
  error vs optimal reference quantified.
- *Robustness:* doc-11 Monte Carlo over density bias (±30%), L/D bias (±15%),
  entry-state dispersion: capture-rate + corridor-breach-rate plots are the
  deliverable. `validation_label: research` where published cases anchor;
  `validated-toy` for the in-repo benchmarks.

---

### 4.5 Powered-descent executor: ignition timing + explicit terminal law + thrust authority

**Problem.** WP-07.4/07.6 produce an optimal descent I-load and track it as an
attitude reference. Three onboard pieces are missing: (a) **when to ignite**
(the I-load assumes its ignition state; dispersions move it), (b) **dispersion
absorption** during the burn (pure attitude tracking of a stale optimal
trajectory diverges — the flown practice is an explicit polynomial law that
re-solves in closed form every cycle), (c) **thrust-magnitude authority** for
guidance (today only the autopilot owns throttle).

**Design — three sub-laws, all closed-form (no onboard optimization):**

**(a) Ignition trigger.** During the pre-burn descent phase, each executor
cycle evaluates a 1-D stopping predictor from the current estimated state:
integrate (fixed-grid RK4, `N_ign` = 100 steps) altitude/vertical-speed under
`T_eff = k_ign · T_max` (declared effective-thrust fraction, default 0.9,
absorbing dispersion margin) + onboard gravity/drag; ignition when predicted
stopping altitude first meets the declared handover altitude band. Publishes
`time_to_go_s` to ignition (the commander fires the ignition event via the
existing `GuidanceCutoff`/event mechanism — same pattern as ascent cutoff,
opposite sign). Deterministic; hysteresis via `n_confirm` (default 2 cycles).

**(b) Terminal law — Augmented Apollo powered-descent guidance (APDG).** The
explicit law is Lu's augmented form of the Apollo/Klumpp polynomial guidance
(closed-form acceleration command toward the declared touchdown state):

```
a_cmd(t) = k_r · (r_f − r)/t_go² + k_v · (v_f − v)/t_go + a_f_ff − g(r)
```

with Apollo gains `(k_r, k_v) = (12, 6)` reducing to Klumpp's quartic for the
classic case and Lu-2019's augmentation pinning the final thrust direction
`a_f_ff` (vertical at touchdown); `t_go` from the smallest positive real root
of the standard quartic in `t_go` (Lu 2019 eq. set), solved by 8 fixed
Newton iterations from the warm-started previous root (deterministic; bounded
below by `t_go_min` to avoid the terminal singularity, freezing attitude and
handing the final meters to the constant-velocity sub-phase). Phase structure
(declared in the mission graph, consistent with `final_descent` vocabulary):

```
braking (APDG, a_cmd ≤ a_max·margin) → approach (APDG, glide-slope-respecting
target offset from I-load) → terminal (constant-descent-rate v_f_decl,
vertical attitude, t_go frozen) → touchdown detection (existing recovery path)
```

The I-load optimal trajectory (WP-07.4) seeds the *declared handover states*
between sub-phases and the feed-forward profile; APDG absorbs the dispersion
between the real state and that reference. Where no I-load is declared, APDG
flies the whole descent from the ignition state (fallback mode); the
suboptimality vs LCvx is measured, not hidden (V&V below).

**Attitude + thrust split.** `a_cmd` maps to: thrust direction
`û = unit(a_cmd)` → reference quaternion via the existing flat-output
thrust-axis construction (`trajectory.rs:635` precedent); thrust magnitude
`‖a_cmd‖` → `ThrustReference.thrust_accel_m_s2` (§5.2). Tilt limit: `û` is
clipped to the declared cone about local vertical before quaternion
construction (glide-slope/tilt vocabulary from WP-07.4 carries over).

**(c) Fallback ladder (executive):** APDG → constant-deceleration law
(`a_cmd = (v²/2Δh + g) · û_vertical`, the 1-D braking law, target-free) →
safe reference (thrust at baseline, attitude-hold vertical). Each step is
strictly simpler and strictly more conservative.

**Forward-only note.** The touchdown condition is the **descent-to-STATE
vocabulary of WP-07.4** (radius/altitude + velocity + attitude class), reusing
its compile-fail tripwire; the executor adds no new target type. Recovery-site
semantics stay where they already live (offline synthesis + provenance).

**V&V:**
- *Verification:* APDG closed-form invariants (command continuity across
  cycles; `t_go` root bracketing; terminal singularity guard); determinism
  goldens; ignition-trigger monotonicity (no chattering under noise — test
  with dispersed estimator input).
- *Validation:* Apollo LM descent published case (Klumpp 1974 P64/P65
  profiles) reproduced within tolerance — `validation_label: research`;
  cross-check vs LCvx optimum (WP-07.4) on the Mars-case benchmark: APDG
  propellant penalty quantified (literature expectation: a few percent —
  ship the measured number in the tolerance table).
- *Closed-loop:* phalcon9-class booster descent scenario (vacuum first, then
  aero-coupled), Monte Carlo over ignition-state dispersion: touchdown-state
  capture rate, propellant margin distribution, fallback-activation rate.

---

### 4.6 RCS pulse modulation and jet selection

**Problem.** RCS effectors today are `direct_torque` proportional devices —
physically they are on/off thrusters with a minimum impulse bit (MIB). The
slosh-arrest coast scenarios fly with an unrealistically continuous actuator.
No doc owns the quantization layer (`parity/06` §3.2 scopes jets as allocator
columns and stops).

**Design — a modulation stage between allocator output and effector command**
(openbmp-fc, new `modulation.rs`), two scenario-selectable modulators per
RCS group:

**(a) PWPF (pulse-width pulse-frequency).** Per-axis (or per-allocated-jet
channel): first-order lag filter `(K_m, τ_m)` driving a Schmitt trigger
`(U_on, U_off)`, output ∈ {0, 1} × jet thrust. Discretized at the autopilot
rate (Tustin, the WP-06.2-c discretization conventions). Default parameters
from the published stability/accuracy maps (Wie's *Space Vehicle Dynamics and
Control* §7; Anthony et al.): `K_m=4.5, τ_m=0.15 s, U_on=0.45, U_off=0.15`,
all scenario-tunable with fail-closed range validation (`0 < U_off < U_on`,
`τ_m > 2·dt`).

**(b) Phase-plane deadband controller (coast attitude-hold).** Classic
two-line switching logic per axis on `(θ_err, ω_err)`: fire against the
boundary when outside the deadband `±θ_db` with rate hysteresis `±ω_db`,
coast inside; produces the standard limit cycle with period/amplitude
predictable from MIB and inertia (the analytic V&V anchor). Used for
long-coast propellant-efficient hold; PWPF for active maneuvering — the
phase declares which.

**MIB + accounting.** Each RCS group declares
`{ thrust_n, min_on_time_s, min_off_time_s }`; the modulator enforces both
(a commanded pulse shorter than `min_on_time_s` either rounds up or drops,
declared policy), and publishes a new `RcsAccounting` topic:
`{ pulse_count: u32, on_time_s: f64, impulse_ns: f64 }` per group —
propellant budgeting joins the doc-05 mass model and the SIL monitor.

**Jet selection.** With WP-06.1 (dense B) the modulator consumes per-jet
allocations directly. Until then, the per-axis allocator output feeds per-axis
modulators mapped to the existing direct-torque racks — a degenerate but
honest configuration (documented limitation).

**V&V:** analytic limit-cycle check (period/amplitude vs closed-form deadband
prediction within 5%); PWPF describing-function operating point inside the
published stable region for the default set; slosh-arrest scenario re-flown
with modulated jets — arrest still succeeds with realistic MIB (the test that
makes the existing slosh result honest); pulse-count/propellant regression in
the golden set. `validation_label: validated-toy` (analytic anchors).

---

## 5. Cross-cutting plumbing

### 5.1 New bus topics

```rust
/// Guidance thrust-magnitude reference (specific force, mass-free).
pub struct ThrustReference {
    pub time: SimTime,
    pub thrust_accel_m_s2: f64,   // commanded ‖T‖/m
    pub valid: bool,              // false → autopilot baseline behavior
}

/// Guidance health, published by the executive each major cycle.
pub struct GuidanceHealth {
    pub time: SimTime,
    pub active_law: u8,           // dictionary-coded law id
    pub ladder_index: u8,         // 0 = primary
    pub converged: bool,
    pub iterations: u8,
    pub residual_norm: f64,
    pub retargeted: bool,         // §4.3 ladder advanced this flight
}

/// RCS pulse accounting per group (§4.6).
pub struct RcsAccounting { /* pulse_count, on_time_s, impulse_ns */ }
```

### 5.2 Guidance thrust authority (closing the throttle gap)

Today `EngineDemand.throttle_unit` is computed by the autopilot from
`throttle_baseline` and max-Q relief. Extension (autopilot-side, small):

```
throttle_cmd = clamp(  T_request / T_max_available(m̂),
                       throttle_min, throttle_baseline )
T_request    = m̂ · ThrustReference.thrust_accel_m_s2      (when valid)
```

with `m̂` from the FC mass model (I-load mass schedule + sensed-accel
correction — the same estimate PEG's integrals use), an inner trim loop on
sensed specific force (`AscentState.thrust_accel_m_s2` precedent: integrate
`k_T · (a_sensed − a_cmd)` into a bounded throttle trim, deterministic
anti-windup per the existing autopilot conventions), and **max-Q relief
retained as a protective ceiling** (relief throttle wins when lower:
`min(throttle_cmd, throttle_q_relief)`). When `ThrustReference.valid` is
false (every current scenario), behavior is bit-identical to today — goldens
unchanged, off by default.

### 5.3 Scenario schema (all fail-closed, schema-v3 conventions)

- `FcGuidanceKind` grows `EntryReference` and `DescentReference`; both ship
  first as **reserved** parsing stubs failing closed with
  `ScenarioError::ElementNotYetSupported` (the `ascent-guidance.md`
  precedent), consumed when their WPs land.
- New blocks: `[fc.entry_guidance]` (method = `"npc" | "corridor"`, corridor
  limits reused from the existing entry-corridor validation, NPC knobs:
  `n_pred`, `iter_cap`, `tol_arc_m`, `sigma_f_rad`, reversal deadband knots),
  `[fc.descent_guidance]` (`method = "apdg" | "const_decel"`, gains, `t_go_min`,
  tilt cone, ignition `k_ign`, handover bands), `[fc.guidance_executive]`
  (§4.1), `[fc.rcs_modulation]` (§4.6), `[fc.contingency]`
  (alternate-cutoff ladder, reserve Δv, confirmation counts).
- Consumer-agreement checks: entry NPC requires lift-capable aero + declared
  `lifting_entry` phase + corridor block; descent executor requires a
  throttleable engine (`T_min < T_max`) + declared descent phases + onboard
  model set; executive requires every named law to be configured for its
  phase (the scenario-consumer-agreement rule — a ladder naming an
  unconfigured law is a load-time error, not a runtime surprise).

### 5.4 FDIR additions

| Bit | Name | Detector |
|---|---|---|
| 15 | `GUIDANCE_NOT_CONVERGED` | burst counter on `GuidanceHealth.converged` |
| 16 | `GUIDANCE_FALLBACK_ACTIVE` | level on `ladder_index > 0` |
| 17 | `GUIDANCE_FALLBACK_EXHAUSTED` | level, latched |
| 18 | `GUIDANCE_TARGET_RETARGETED` | level, latched (§4.3) |

All four follow the existing `DetectorKind` + `FdirParams` extension pattern
(`fdir.rs:19` conventions); commander arming/abort consumption unchanged.

### 5.5 Determinism contract

Every law here is fixed-grid, fixed-iteration-cap, pre-allocated,
wall-clock-free, and warm-start-stateful only through explicitly reset state:

- Predictor grids (`N_pred`, `N_ign`) and iteration caps are scenario
  constants; early exit on convergence uses exact float comparisons of
  deterministic quantities (bit-stable).
- No new heap allocation on the tick path (workspaces sized at construction —
  `fc_lints` no-hot-path-allocation conventions).
- No Clarabel and no `openbmp-trajopt` on any new FC path
  (`fc_dependency_tripwire` unchanged; the `mpc` feature's existing surface is
  untouched).
- Golden additions: one scenario per capability (entry NPC, descent executor,
  executive fallback drill, RCS modulated coast, contingency retarget) joins
  the byte-determinism CI set, off-by-default for existing scenarios (all
  current goldens byte-identical — every capability is opt-in).

---

## 6. Forward-only frame (inherited locks + new tripwires)

Inherited, unchanged: the closed `TerminalCondition` vocabulary (no
lat/lon/range/aimpoint fields anywhere — `profile.rs` lock); optimizer
placement offline with I-load boundary (`parity/07` §6); target-free abort
ladder (WP-06.4-c); provenance-tokened recovery states; per-WP dual-use notes.

New surfaces and their proofs:

1. **Entry terminal type** (`EntryTerminalState` + `plane_normal_eci`):
   inertial-state and inertial-plane fields only; compile-fail tripwire proves
   no geodetic/range field exists (extends the WP-07.4 tripwire family).
2. **Alternate-cutoff ladder** (§4.3): type-restricted to orbital-element
   conditions; compile-fail tripwire; ladder entries carry offline-validation
   audit metadata; the abort path remains target-free.
3. **Descent touchdown condition**: reuses the WP-07.4 descent-to-STATE type
   and its tripwire verbatim; this document adds no target type.
4. **Executive**: selects among *declared* reference sources in *declared*
   order; it computes no targets; demotion is monotonic (verifiable graph).
5. The NPC and APDG are reference generators in the exact sense of
   `ascent-guidance.md`: "the output is a reference the three-loop autopilot
   tracks; it is not a guidance solution to any location" — corridor and
   vehicle-intrinsic constraints dominate; terminal regulation is to declared
   recovery states that enter the system only through the provenance-tokened
   offline pipeline.

What stays refused, restated: ground-aimpoint fire-control vocabulary,
surface-coordinate targets in any onboard type, optimize-to-impact objectives,
and any in-loop optimizer. (Governed by `docs/safety-boundaries.md` and
`docs/standards-posture.md`.)

---

## 7. V&V summary and labels

| Capability | Verification anchors | Validation anchors | Label target |
|---|---|---|---|
| Executive (§4.1) | transition-graph exhaustive test; handoff continuity bound (`‖Δq‖ < ε` at switch); determinism goldens | SIL fault-injection drill matrix (solver fault, NaN, non-convergence) | `validated-toy` |
| PEG hardening (§4.2) | corrector contraction test; staging-transient decay; small-`t_go` guard | optimality gap vs T2 + PMP witness (< 0.5% propellant, tolerance table); SLS-mod A/B on long burn | `research` |
| Contingency (§4.3) | reachability-check unit cases; ladder-walk determinism | MC reconvergence envelope (failure time × deficit); insertion elements vs offline feasibility envelopes | `validated-toy` → `research` |
| Entry NPC (§4.4) | predictor self-consistency; corrector quadratic convergence; corridor-clip precedence | Lu/FNPEG published cases; Apollo-4-class dispersed entry capture; cross-check vs WP-07.5 SCvx | `research` |
| Descent executor (§4.5) | `t_go` root bracketing; ignition monotonicity; terminal-singularity guard | Klumpp/Apollo LM case; APDG-vs-LCvx (WP-07.4) penalty measurement; touchdown-capture MC | `research` |
| RCS modulation (§4.6) | analytic limit-cycle period/amplitude (5%); PWPF stable-region check | slosh-arrest re-flight with MIB; propellant accounting regression | `validated-toy` |

Each WP ships its own tolerance-table TOML (per-case `expected/abs_tol/rel_tol`,
provenance-pinned case parameters), per repo convention.

---

## 8. Dependencies

| On | What is consumed |
|---|---|
| `parity/07` WP-07.4 / WP-07.6 | descent I-load reference + descent-to-STATE type + audit metadata (§4.5); offline cross-check oracles (§4.2 H4, §4.4, §4.5) |
| `parity/06` WP-06.4-c | reconfiguration topic + SAFE ladder handoff (§4.1, §4.3); lands independently — §4.3 step 1 degrades gracefully to sensed-thrust-only feedback if 06.4-c is absent |
| `parity/06` WP-06.1 | per-jet allocation for §4.6 jet selection (degenerate per-axis mode until then) |
| `parity/03`/`04` | truth aero/aerothermal the onboard model set is *derived from* (never linked) |
| `parity/09` | sensed inputs (specific force, estimated state) — all already on the bus |
| `parity/10` (SIL) / `11` (MC) / `12` (determinism) | fault-injection harness, dispersion campaigns, byte-determinism gate |
| `flight-profiles-architecture.md` | phase vocabulary (`powered_ascent`, `lifting_entry`, `final_descent`) and the reserved `select_guidance_profile` action (the executive is its concrete owner) |

Ordering: WP-OG.0/1/2 have no hard upstream dependency and can start now;
WP-OG.4/5 want WP-07.4 only for their *cross-check oracles* (the laws
themselves are explicit and self-contained).

---

## 9. Work-package backlog

### WP-OG.0 — Reconcile ascent-guidance.md + design-of-record for shipped PEG

- **goal:** `ascent-guidance.md` reflects the five shipped generators; §4.2's
  design-of-record (equations as implemented) reviewed against
  `profile.rs:700–1050` and committed as the documented algorithm.
- **fidelity_tier:** T0 (documentation/reconciliation)
- **depends_on:** []
- **new_crates:** none
- **touched:** `docs/ascent-guidance.md`, `docs/onboard-guidance.md` (§4.2
  equation audit), no code.
- **acceptance:** doc states match `profile.rs` reality (reviewer-checked
  against the listed line anchors); parity checker + doc links green.
- **validation_label:** `checked`
- **dual_use_note:** far from line (documentation).
- **est_effort:** 1–2 days
- **parity_ceiling:** documents; validates nothing new.

### WP-OG.1 — Guidance executive + GuidanceHealth + FDIR bits 15–17

- **goal:** §4.1 executive job, ladder config, handoff shaping, health topic,
  three FDIR bits; legacy direct-publish path preserved as default.
- **fidelity_tier:** T1
- **depends_on:** [WP-OG.0]
- **new_crates:** none (`openbmp-fc/src/guidance_executive.rs`)
- **touched:** `openbmp-fc/src/{guidance_executive.rs,topics.rs,fdir.rs}`,
  `openbmp-scenario` (`[fc.guidance_executive]`), `openbmp-runner/src/fc.rs`.
- **acceptance:** off by default; goldens byte-identical; transition-graph
  test enumerates every (phase × ladder × fault) cell; SIL fault-injection
  drill (scripted generator fault → demotion → safe reference, FDIR bits
  observed); handoff continuity bound met; deterministic two-run.
- **validation_label:** `validated-toy`
- **dual_use_note:** selects among declared reference sources only; computes
  no targets; demotion monotonic. Far from line.
- **est_effort:** ~1.5–2 wk
- **parity_ceiling:** method-level moding; not a certified FDIR/abort system.

### WP-OG.2 — PEG hardening H1–H3

- **goal:** convergence guards, SLS-class corrector stabilization, exact
  multi-stage thrust integrals; diagnostics surfaced via `GuidanceDiagnostics`.
- **fidelity_tier:** T1
- **depends_on:** [WP-OG.0]
- **new_crates:** none
- **touched:** `openbmp-physics/src/profile.rs` (PEG internals),
  `openbmp-fc/src/guidance.rs` (diagnostics plumb-through).
- **acceptance:** goldens byte-identical for default constants; guard cases
  (long burn, staging transient, `t_go → 0`) each have a regression test;
  A/B long-burn case shows the documented stabilization; corrector
  contraction property test.
- **validation_label:** `validated-toy`
- **dual_use_note:** same closed cutoff vocabulary; no new target surface.
- **est_effort:** ~1–2 wk
- **parity_ceiling:** optimality not yet claimed (that is WP-OG.3).

### WP-OG.3 — PEG optimality evidence (H4)

- **goal:** measured optimality gap vs T2 collocation + PMP witness on the
  vacuum-ascent benchmark; tolerance table ships the number.
- **fidelity_tier:** T2
- **depends_on:** [WP-OG.2, WP-07.2, WP-07.6]
- **acceptance:** gap < 0.5% propellant on the benchmark or the measured gap
  documented with cause; CI regression on the tolerance table.
- **validation_label:** `research`
- **dual_use_note:** far from line (evidence only).
- **est_effort:** ~1 wk
- **parity_ceiling:** benchmark-scoped; no flight claim.

### WP-OG.4 — Entry NPC (fixed-iteration predictor-corrector)

- **goal:** §4.4 in full: onboard model set, energy-domain predictor, capped
  Newton corrector, corridor clipping, lateral reversal logic, executive
  integration, schema + tripwire.
- **fidelity_tier:** T2
- **depends_on:** [WP-OG.1; oracle-only: WP-07.5]
- **new_crates:** none (`openbmp-physics/src/profile.rs` generator +
  `openbmp-fc` job wiring)
- **touched:** `openbmp-physics/src/{profile.rs,reentry.rs}`, `openbmp-fc`,
  `openbmp-scenario`, `openbmp-testkit` (compile-fail tripwire),
  `openbmp-trajopt/src/iload.rs` (onboard-model + entry-target sections).
- **acceptance:** off by default; goldens byte-identical; predictor
  self-consistency; published-case capture within tolerance table;
  dispersed-entry MC capture/corridor plots emitted; corridor-clip precedence
  test (corridor beats terminal regulation); **compile-fail tripwire: entry
  target type carries no geodetic/range field**; deterministic two-run.
- **validation_label:** `research` (published anchors), `validated-toy`
  (in-repo cases)
- **dual_use_note:** **the sensitive WP of this doc alongside OG.5** —
  corridor-driven bank, terminal regulation to provenance-tokened inertial
  recovery states only; inertial-plane lateral logic; proving tripwire added.
- **est_effort:** ~3–4 wk
- **parity_ceiling:** 3-DoF NPC with onboard models; no aero-coupled 6-DoF
  claim (offline SCvx territory); not flight-validated.

### WP-OG.5 — Descent executor (ignition + APDG + thrust authority)

- **goal:** §4.5 + §5.2 in full: ignition predictor, APDG with sub-phases,
  `ThrustReference` + autopilot throttle tracking (max-Q ceiling retained),
  fallback ladder, schema.
- **fidelity_tier:** T2
- **depends_on:** [WP-OG.1; oracle/seed: WP-07.4, WP-07.6]
- **touched:** `openbmp-physics/src/profile.rs` (descent generator),
  `openbmp-fc/src/{guidance.rs,autopilot.rs,topics.rs}`, `openbmp-scenario`,
  `openbmp-testkit`.
- **acceptance:** off by default; goldens byte-identical (ThrustReference
  invalid ⇒ bit-identical autopilot behavior, tested); Klumpp/Apollo case in
  tolerance table; APDG-vs-LCvx penalty measured and shipped; ignition
  monotonicity under estimator noise; touchdown-capture MC; tilt-cone clip
  test; reuses the WP-07.4 descent-to-STATE tripwire (extended to the executor
  surface); deterministic two-run.
- **validation_label:** `research`
- **dual_use_note:** descent-to-STATE vocabulary verbatim from WP-07.4; no new
  target type; thrust authority is magnitude-only with protective ceilings.
- **est_effort:** ~3–4 wk
- **parity_ceiling:** explicit-law executor; the optimal synthesis stays
  offline; no WCET/flight claim.

### WP-OG.6 — Ascent contingency (reconvergence + alternate ladder)

- **goal:** §4.3 in full: live stage-table feedback, reachability check,
  type-locked alternate ladder, FDIR bit 18, MC envelope deliverable.
- **fidelity_tier:** T2
- **depends_on:** [WP-OG.2; degrades gracefully without WP-06.4-c]
- **touched:** `openbmp-physics/src/profile.rs`, `openbmp-fc`,
  `openbmp-scenario`, `openbmp-testkit` (ladder compile-fail tripwire),
  `openbmp-trajopt` (offline alternate-validation driver + I-load section).
- **acceptance:** off by default; goldens byte-identical; the three scripted
  deficit cases (§4.3 V&V) pass; ladder walk deterministic + hysteresis
  verified; **compile-fail tripwire: ladder accepts orbital-element conditions
  only**; alternates below the declared perigee floor rejected at load; MC
  envelope plot emitted; SAFE handoff drill (ladder exhausted → target-free
  safe reference + FDIR).
- **validation_label:** `validated-toy` → `research` (envelope vs offline
  feasibility)
- **dual_use_note:** alternates are orbital conditions, pre-validated offline,
  audit-recorded; the abort path remains target-free (WP-06.4-c lock
  untouched); proving tripwire added.
- **est_effort:** ~2–3 wk
- **parity_ceiling:** guidance-level continuation; no propellant-system
  reconfiguration (doc 05), no certified abort claim.

### WP-OG.7 — RCS pulse modulation + accounting

- **goal:** §4.6 in full: PWPF + phase-plane modulators, MIB enforcement,
  `RcsAccounting`, schema; slosh-arrest scenario re-validated with MIB.
- **fidelity_tier:** T1
- **depends_on:** [none hard; WP-06.1 upgrades jet selection later]
- **touched:** `openbmp-fc/src/{modulation.rs,topics.rs}`, `openbmp-scenario`,
  the slosh-arrest scenario (new modulated variant).
- **acceptance:** off by default; goldens byte-identical; analytic limit-cycle
  within 5%; PWPF defaults inside the published stable region (cited check);
  modulated slosh-arrest succeeds; pulse/propellant accounting in telemetry +
  regression; deterministic two-run.
- **validation_label:** `validated-toy`
- **dual_use_note:** actuation quantization; far from line.
- **est_effort:** ~1.5–2 wk
- **parity_ceiling:** modulation logic; thruster plume/impingement physics
  out of scope (docs 05/15).

**Suggested order:** OG.0 → OG.1 → OG.2 → {OG.7, OG.4, OG.5 in parallel} →
OG.6 → OG.3 (after WP-07.2/07.6 land their oracles).

---

## 10. References

**Ascent (PEG family)**
- R. F. Jaggers, "An Explicit Solution to the Exoatmospheric Powered Flight
  Guidance and Trajectory Optimization Problem for Rocket Propelled Vehicles,"
  AIAA 1977-1051; and NASA NTRS 19760020204 (thrust-integral derivation).
- R. L. McHenry, T. J. Brand, A. D. Long, B. F. Cockrell, J. R. Thibodeau,
  "Space Shuttle Ascent Guidance, Navigation, and Control," *J. Astronautical
  Sciences* 27(1), 1979.
- NASA NTRS 20180002035, "Powered Explicit Guidance Modifications and
  Enhancements for Space Launch System Block-1 and Block-1B Vehicles," 2018.
- A. Mahajan et al., "Enhancements to Space Shuttle Powered Explicit Guidance,"
  AAS 25-844 (NTRS 20250011251), 2025 — corrector v_go scaling, steering
  tangent limiting, long-burn stability.

**Entry (predictor-corrector)**
- P. Lu, "Entry Guidance: A Unified Method," *JGCD* 37(3), 2014 — FNPEG.
- P. Lu, "Predictor-Corrector Entry Guidance for Low-Lifting Vehicles,"
  *JGCD* 31(4), 2008.
- C. W. Brunner, P. Lu, "Comparison of Fully Numerical Predictor-Corrector and
  Apollo Skip Entry Guidance," (verification cases for the FNPEG family).
- Orion PredGuid lineage: Putnam, Bairstow et al., "Orion Reentry Guidance
  with Extended Range Capability Using PredGuid," AIAA GNC 2008 — the flown
  NPC precedent.
- K. Tracy, Z. Manchester, "CPEG: A Convex Predictor-corrector Entry
  Guidance Algorithm," IEEE Aerospace 2022 — the convex sibling (offline/
  future-tier context only).
- Harpold & Graves, "Shuttle Entry Guidance," *J. Astronautical Sciences*
  27(3), 1979 — drag-reference heritage for the corridor fallback.

**Powered descent (explicit laws; offline optimization is parity/07)**
- A. R. Klumpp, "Apollo Lunar Descent Guidance," *Automatica* 10(2), 1974.
- G. W. Cherry, "A General, Explicit, Optimizing Guidance Law for
  Rocket-Propelled Spaceflight," AIAA 1964-638 — E-guidance.
- P. Lu, "Augmented Apollo Powered Descent Guidance," *JGCD* 42(3), 2019 —
  APDG, the terminal law of §4.5.
- C. N. D'Souza, "An Optimal Guidance Law for Planetary Landing," AIAA
  1997-3709.
- (Offline cross-check oracles: Açıkmeşe & Ploen 2007; Açıkmeşe, Carson,
  Blackmore 2013; Szmuk et al. — cited in `parity/07` §10.)

**RCS modulation**
- B. Wie, *Space Vehicle Dynamics and Control*, 2nd ed., AIAA, 2008 — §7
  phase-plane logic, PWPF describing-function stability maps.
- T. C. Anthony, B. Wie, S. Carroll, "Pulse-Modulated Control Synthesis for a
  Flexible Spacecraft," *JGCD* 13(6), 1990 — PWPF parameter selection.

**Executive / flight practice context**
- NASA, *Space Shuttle GN&C Workbook* (public releases) — major/minor cycle
  guidance architecture heritage.
- Open implementations studied (study only, never vendored): KSP
  kOS/MechJeb PEG ports, FlightGear Shuttle PEG notes — useful for
  convergence-guard folklore, not authoritative.

**Cross-references (this repo):** `ascent-guidance.md`,
`descent-and-entry-profiles.md`, `flight-profiles-architecture.md`,
`ballistic-coast-and-apogee.md`, `parity/06-gnc-coupled-mimo-and-control.md`,
`parity/07-trajectory-optimization-and-mission-design.md`,
`parity/10/11/12/13`, `safety-boundaries.md`, `standards-posture.md`.
**In-repo anchors:**
`crates/openbmp-physics/src/profile.rs` (generator families),
`crates/openbmp-fc/src/{guidance.rs,autopilot.rs,fdir.rs,scheduler.rs,mpc.rs,landing.rs,trajectory.rs}`,
`crates/openbmp-trajopt/src/iload.rs`.
