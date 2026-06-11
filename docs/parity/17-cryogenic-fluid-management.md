# Cryogenic Fluid Management & Propellant Thermodynamics

**Status:** `experimental` (design intent; no code shipped by this document).
**Audience:** the engineer or LLM agent implementing the parity work packages.
**Prerequisite reading:** `00-overview.md` (parity definition, invariants),
`13-agent-execution-playbook.md` (gate set, WP schema),
`05-propulsion-high-fidelity.md` (feed-system architecture this document
extends).

> One-line scope: replace the quasi-static isentropic ullage blowdown with a
> real tank thermodynamic model — two-zone ullage/liquid energy balance,
> self-pressurization and boiloff, pressurant injection (helium and
> autogenous), vent/relief/TVS pressure control, chilldown correlations, and
> the propellant loading/conditioning timeline — deterministic, off by
> default, and validated against published NASA tank-test and flight-stage
> data.

Document `05` deliberately deferred this dimension: its ceiling list names
"cryogenic propellant thermochemistry specifics — real boiloff/chilldown
deferred". Documents `18` (ground operations) and `19` (day-of-launch) consume
the loading timeline built here; document `05` consumes the tank outlet state
(pressure, temperature, quality) as the upstream boundary of its feed network.
This document closes the deferral.

---

## 1. Parity target & ceiling

### 1.1 Reference practice

Cryogenic fluid management (CFM) is a first-class simulation discipline at
every reference organization, distinct from feed-system transients:

- **NASA MSFC — TankSIM** models cryogenic tank pressure control for
  long-duration storage: self-pressurization, boiloff, ullage venting, axial
  jet mixing, thermodynamic vent systems (TVS), and wall condensation
  (NTRS 20170005533).
- **NASA MSFC — GFSSP** is applied beyond feed lines to tank
  self-pressurization with conjugate heat transfer; its user manual publishes
  worked example problems that function as open benchmarks.
- **SINDA/FLUINT + Thermal Desktop** (commercial) is NASA's workhorse for
  stratified-tank, zero-boiloff, and TVS modeling (NTRS 20190031763,
  NTRS 20150002709).
- **SpaceX** flies densified propellants (subcooled LOX near 66 K, chilled
  RP-1) with a late-load "load-and-go" countdown in which LOX replenish
  continues until minutes before launch — a propellant-conditioning timeline
  that is itself a simulated, NASA-reviewed process (load-and-go crew
  certification; the AMOS-6 investigation reproduced a COPV failure *entirely
  through helium loading conditions*, demonstrating validated models of
  cryo-conditioning physics).
- **Blue Origin** staffs a standing fluid-systems organization covering cryo
  storage, feed, and pressurization networks.
- **Flight experiments** — AS-203 (S-IVB liquid-hydrogen orbital
  self-pressurization experiment) and the MSFC Multipurpose Hydrogen Test Bed
  (MHTB) — published the test data that anchors every modern CFM code.

### 1.2 Parity target (capability, within posture)

An in-repo, deterministic, lumped-parameter CFM tier:

1. **Propellant property layer** — provenance-pinned thermophysical property
   decks (saturation curves, liquid/vapor density, enthalpy, specific heats,
   surface tension, viscosity, conductivity) for LOX, LN2, LCH4, LH2, and
   RP-1, with a closed `PropellantEos` trait and fail-closed validity ranges.
2. **Two-zone tank thermodynamics** — separate ullage-gas and liquid control
   volumes with a saturated-interface model, lumped wall nodes, declared heat
   leaks (MLI/foam conduction correlations), evaporation/condensation mass
   transfer, self-pressurization, and boiloff.
3. **Pressure control** — pressurant injection from a helium source (COPV
   blowdown thermodynamics) or autogenous bleed (interface to `05`),
   pre-pressurization transients, vent/relief valves with hysteresis bands,
   and a TVS (Joule-Thomson expansion + heat exchanger + mixing-jet
   efficiency).
4. **Stratification tier** — N-layer one-dimensional thermal stratification in
   the liquid and ullage with buoyancy-stable layering and jet/slosh-driven
   mixing hooks.
5. **Chilldown & loading** — engineering boiling-regime correlations for line
   and engine chilldown, tank fill/replenish/drainback timeline simulation,
   and densified-propellant conditioning (subcooled targets, recirculation),
   coupled to the `18` countdown sequencer.

All of it byte-deterministic, off by default, uncertainty-tagged through `11`,
and validated against the open data named in §5.

### 1.3 Parity ceiling (honest boundary)

- **No proprietary tank thermal maps.** Real vehicles correlate tank models
  against instrumented tanking tests; those datasets are proprietary. Open
  substitute: MHTB, K-site, and AS-203 published data; the model earns
  `research` only against those.
- **No microgravity phase management.** Zero-g propellant position, screen
  channel liquid-acquisition devices, cryocooler zero-boiloff loops, and
  settled-vs-unsettled transition physics are out of scope; the model assumes
  settled propellant under a declared acceleration floor and **fails closed**
  (validity error) below it. Long-coast CFM beyond TVS-with-settling is not
  claimed.
- **No CFD-grade mixing.** Destratification and jet mixing use engineering
  efficiency coefficients; high-fidelity mixing fields enter only as ingested,
  provenance-pinned decks per the solver-consumer rule (`13` §6).
- **No flammability/safety certification.** Geyser, ignition-hazard, and
  COPV qualification analyses are named as reference practice, not designed.
- **Property accuracy ceiling.** In-repo property decks are curve fits to
  published data validated code-to-code against CoolProp; they are not a
  REFPROP-grade reference implementation.

---

## 2. Current state in source

- `crates/openbmp-vehicle/src/propellant_budget.rs` — quasi-static isentropic
  ullage blowdown: pressure follows an isentropic expansion law as liquid
  drains; no energy equation, no heat leak, no boiloff, no pressurant
  injection. This is the baseline the new model must regress against in the
  off-by-default case.
- `crates/openbmp-vehicle/src/tank/` — tank geometry, propellant mass
  accounting, and the slosh mechanical analog (doc `02` extends); no thermal
  state.
- `crates/openbmp-runner/src/tanks.rs` — runner wiring of tank mass depletion
  into vehicle mass properties; the seam where thermodynamic tank state will
  be advanced.
- `crates/openbmp-propulsion/src/engine.rs`, `cluster.rs` — engine interfaces
  that will supply autogenous bleed conditions and consume tank outlet state
  once doc `05` T4 (feed network) lands.
- `crates/openbmp-physics/src/atmosphere/` — ambient conditions for external
  heat-leak boundary terms.
- No property layer, no ullage energy equation, no vent/relief/TVS, no
  chilldown, no loading timeline exists anywhere in the workspace.

---

## 3. Target architecture

### 3.1 Crate boundary

Default placement: a `cryo` module inside `openbmp-feedsystem` (the L2 crate
doc `05` proposes), since tank thermodynamics and feed transients share the
fluid-property layer and couple at the tank outlet. If `05`'s crate review
prefers a separate boundary, `openbmp-cryo` (L2) is the fallback, depending
only on `openbmp-core`/`-state`/`-models`. The property layer (§3.2) lands in
whichever crate ships first and is re-exported; it must never duplicate.
Decision is taken in WP-17.1's skeleton review per `13` §5. Nothing here is
reachable from `openbmp-fc` (portability lock).

### 3.2 Propellant property layer

```rust
/// Closed thermophysical property surface for a stored propellant.
pub trait PropellantEos {
    fn saturation_pressure(&self, t_k: f64) -> Result<f64, CryoError>;
    fn saturation_temperature(&self, p_pa: f64) -> Result<f64, CryoError>;
    fn liquid_density(&self, t_k: f64, p_pa: f64) -> Result<f64, CryoError>;
    fn vapor_density(&self, t_k: f64, p_pa: f64) -> Result<f64, CryoError>;
    fn liquid_enthalpy(&self, t_k: f64, p_pa: f64) -> Result<f64, CryoError>;
    fn vapor_enthalpy(&self, t_k: f64, p_pa: f64) -> Result<f64, CryoError>;
    fn heat_of_vaporization(&self, t_k: f64) -> Result<f64, CryoError>;
    fn validity(&self) -> PropertyEnvelope;
}
```

Implementation: per-fluid coefficient decks (Antoine/Wagner-form saturation
curves, polynomial/corresponding-states fits for caloric properties) stored
under `data/` with `provenance.md` and SHA pins, sourced from openly published
tabulations (NIST WebBook–derived public tables, published cryogenic
handbooks). Every call range-checks against `PropertyEnvelope` and fails
closed outside it. Pressurant gases (He, GH2, GO2, N2) get a real-gas-lite
deck (compressibility-corrected ideal gas with published Z-tables where it
matters, i.e. helium at COPV pressures).

### 3.3 Two-zone tank model (T1–T2)

State per tank: ullage mass and internal energy `(m_u, U_u)`, liquid mass and
internal energy `(m_l, U_l)`, lumped wall-node temperatures `T_w[i]`, plus the
existing geometry. Governing ODEs (locked operand order, fixed-step
sub-cycling like the contact sub-stepper in `14`):

```
dm_u/dt = ṁ_evap − ṁ_cond + ṁ_press − ṁ_vent
dm_l/dt = −ṁ_evap + ṁ_cond − ṁ_out − ṁ_bleed
dU_u/dt = Q̇_wall,u + Q̇_int,u + ṁ_press·h_press − ṁ_vent·h_u
          + ṁ_evap·h_v(T_int) − ṁ_cond·h_v(T_int) − p·dV_u/dt
dU_l/dt = Q̇_wall,l − Q̇_int,l − ṁ_out·h_l − ṁ_evap·h_v(T_int)
          + ṁ_cond·h_l(T_int) − p·dV_l/dt
```

- **Interface model:** a saturated film at `T_int = T_sat(p)`. Interfacial
  heat flows `Q̇_int` use Nusselt-correlation film coefficients on each side;
  net evaporation/condensation closes the interface energy balance
  (`ṁ_evap−ṁ_cond = (Q̇_l→int − Q̇_int→u)/h_fg`), the standard
  engineering-tier closure used by TankSIM-class codes.
- **Wall & heat leak:** lumped wall nodes with declared external boundary —
  ambient convection/radiation pre-launch (from `08` atmosphere at pad
  altitude), declared ascent heating flux hook (optional input from `04`'s
  distributed deck), and an insulation conductance from a layered MLI/foam
  correlation deck (Lockheed-type layered-MLI effective conductivity as a
  provenance-pinned correlation with declared coefficients).
- **Ullage volume kinematics** follow liquid drain (`dV_u/dt = ṁ_out/ρ_l + …`)
  with the existing tank geometry.
- **Pressure** from ullage state: `p = p(m_u, U_u, V_u)` through the gas deck
  (mixture of propellant vapor + pressurant, Dalton partial-pressure mix at
  the engineering tier).

Boiloff is then an output (vented mass at steady heat leak), not an input
coefficient — closing a fidelity gap the audit attributes to every
time-driven-thrust-curve-era assumption.

### 3.4 Pressure control (T2)

- **Pressurant source:** COPV/bottle blowdown — `(m_b, U_b)` ODE with
  real-gas-lite helium, declared bottle volume/initial state, an orifice/
  regulator flow model (choked/subsonic mass flow with locked branch
  selection), and injected enthalpy `h_press` at a declared diffuser
  temperature efficiency. **Collapse factor emerges** from the energy
  equation (cold-ullage injection condenses and cools), rather than being a
  hand-tuned multiplier; the classical collapse-factor number is recovered as
  a *diagnostic* output for comparison with published values.
- **Autogenous pressurization:** bleed state (ṁ, h) supplied by the `05`
  engine interface (heat-exchanger outlet); a declared-schedule stub allows
  running before `05` T4 lands.
- **Pre-press transient:** scripted pre-pressurization to relief band before
  engine start; couples to `18`'s countdown sequencer events.
- **Vent/relief:** valve models with open/close hysteresis bands, choked vent
  flow, deterministic latching; ground vent vs flight relief distinguished by
  configuration.
- **TVS (T3):** Joule-Thomson expansion of a liquid tap to a low-pressure
  two-phase coolant loop, heat exchanger against the bulk liquid, and a
  mixing-jet destratification efficiency `η_jet` (declared, uncertainty-
  tagged) — the TankSIM-class architecture.

### 3.5 Stratification tier (T3)

N-layer 1-D model: liquid column divided into buoyancy-ordered layers, each
with `(m_i, U_i)`; inter-layer conduction + a mixing operator driven by jet
momentum (TVS/recirculation) and a slosh-disturbance hook (doc `02`'s analog
energy as a destratification source, coefficient-tagged). Ullage similarly
2–4 layers. Self-pressurization rate at MHTB heat-leak conditions is the
discriminating validation case between two-zone and stratified tiers (the
two-zone model under-predicts; the stratified model approaches the published
pressure-rise curves — both results are reported honestly with tolerance
tables).

### 3.6 Chilldown & loading (T4)

- **Line/engine chilldown:** quasi-steady marching model over declared line
  segments with boiling-regime selection — film (Bromley-type), transition,
  and nucleate (Rohsenow-type) correlations with locked regime-switch
  hysteresis; outputs chilldown propellant consumption and time-to-quality
  gates. Coefficients are published-correlation decks with provenance; the
  regime map is reported with the validity envelope.
- **Tank loading timeline:** fill (slow-fill/fast-fill rates with flash
  evaporation at warm-tank contact), replenish/topping against boiloff,
  drainback, and stop-flow states, exposed as a `[tanks.loading]` scenario
  block consumed by `18`'s countdown sequencer (hold/recycle re-enters
  replenish; scrub enters drainback). Consumables bookkeeping (pressurant
  budget, vented mass) feeds `18`'s recycle-capability evaluation.
- **Densified conditioning:** subcooled target temperature schedules with a
  recirculation-loop heat balance; the deliverable is the conditioning
  *timeline* (temperature vs time vs flow), not a facility design.

### 3.7 Scenario schema

`[vehicle.tanks.<name>.cryo]` opt-in block: fluid id, property deck path,
initial state (fill fraction, ullage pressure/temperature), wall/insulation
deck, heat-leak boundary, pressurant source, control bands, TVS config,
loading profile reference. `deny_unknown_fields`, fail-closed validity at
load (settled-acceleration floor, property envelopes, stability of the
sub-step). Off by default: without the block, `propellant_budget.rs` behavior
is byte-identical.

### 3.8 Fidelity tiers

- **T0 (current):** isentropic ullage blowdown; no thermal state.
- **T1:** property layer + two-zone tank, self-pressurization, boiloff, vent —
  `validated-toy` (analytic/MMS) → `research` against MHTB trends.
- **T2:** helium pressurant + COPV source + pre-press + relief + autogenous
  interface — `validated-toy`; collapse-factor diagnostic vs published ranges.
- **T3:** N-layer stratification + TVS + slosh/jet mixing hooks — `research`
  against MHTB/AS-203 pressure-rise curves.
- **T4:** chilldown correlations + loading/conditioning timeline coupled to
  `18` — `checked`/`validated-toy` (correlation-tier honesty).
- **T5:** ingested high-fidelity mixing/boiling decks (CFD-produced,
  provenance-pinned, UQ-tagged) per the solver-consumer rule — `research`.

---

## 4. Invariant preservation

- **Determinism:** fixed-step sub-cycled ODE advance with locked operand
  order; regime switches (boiling map, valve hysteresis, choked/subsonic)
  latch at sub-step boundaries only; no iteration-to-convergence on the hot
  path without a fixed iteration count and deterministic seed values.
- **Byte-stable default:** everything behind `[vehicle.tanks.*.cryo]`; the
  canonical scenario set and goldens are untouched.
- **FC portability:** tank thermodynamics is plant-side (L2); the flight
  computer sees only transduced channels (tank pressure/temperature sensors
  via the `09` sensor pattern).
- **Provenance:** property decks, MLI correlation coefficients, and every
  validation dataset (MHTB/AS-203 digitizations) land under `data/` with
  `provenance.md` + SHA pins; the inline-data tripwire extends to property
  coefficients (no bare saturation-curve literals in `.rs`).
- **Fail-closed:** out-of-envelope property calls, unsettled acceleration,
  sub-step stability violations, and unknown schema fields are `Err`, never
  silent clamps.
- **Labels:** each tier earns the label in §3.8; no claim ever exceeds
  `research`.

---

## 5. V&V plan

| Case | Type | Tier | Tolerance target |
|---|---|---|---|
| Property decks vs CoolProp tabulations (saturation p/T, ρ, h for LOX/LH2/LCH4/LN2) | code-to-code | T1 | < 0.5% over declared envelope (table) |
| Closed rigid tank, adiabatic, known heat input: analytic two-zone energy balance | analytic/MMS | T1 | < 1e-9 rel (conservation audit) |
| Mass/energy closure audit every run (Σ in − Σ out − Δstorage) | property | T1+ | < 1e-10 rel |
| Self-pressurization rate, MHTB LH2 published heat-leak cases | public benchmark | T1/T3 | two-zone: trend + sign; stratified: < 20% on dp/dt (honest band, table) |
| AS-203 S-IVB orbital self-pressurization (published flight curves) | public benchmark | T3 | qualitative envelope + documented residuals |
| Collapse factor vs published helium-injection ranges | literature check | T2 | within published band, reported not tuned |
| GFSSP user-manual tank example problems | code-to-code | T2 | < 10% state trajectories (table) |
| Chilldown correlation vs published line-chilldown test data | public benchmark | T4 | regime-correct + < 30% time-to-chill (correlation-tier honesty) |
| Loading timeline: replenish balances declared boiloff at steady state | analytic | T4 | < 1e-6 rel |

Every numeric claim carries a tolerance-table TOML; "what is not validated"
(microgravity, proprietary tank maps) is restated in each table per
`docs/verification.md`.

---

## 7. Dependencies on other parity docs

- `05` — shares the fluid-property layer and the crate boundary; supplies
  autogenous bleed state; consumes tank outlet conditions (NPSH margin chain
  closes when both land). POGO (`05` T5) gains a physically grounded ullage
  compliance term from T2.
- `18` — consumes the loading/conditioning timeline and consumables
  bookkeeping; supplies countdown events (pre-press, topping stop, T-0).
- `02` — slosh analog energy as a destratification disturbance hook.
- `04` — optional ascent external-heating boundary for wall nodes.
- `08` — ambient atmosphere for pad-side boundary conditions.
- `11` — uncertainty tags on heat leak, mixing efficiencies, correlation
  coefficients; dispersion campaigns over fill/conditioning states.
- `12` — sub-cycled ODE determinism contract; campaign-scale runs.

---

## 8. Open-source leverage

| Tool/data | Use mode | License/status |
|---|---|---|
| CoolProp | code-to-code property oracle (external process; never linked) | MIT |
| NIST WebBook tabulations | property-deck source data (digitized, pinned) | public data |
| Cantera | gas-phase property cross-check for pressurant mixes | BSD-3 |
| GFSSP user-manual examples | published benchmark problems | NASA-published manual |
| TankSIM / MHTB / AS-203 NTRS papers | method + validation curves (digitized, pinned) | public NTRS |
| REFPROP | named commercial reference (not used in CI) | commercial |
| SINDA/FLUINT | named reference practice only | commercial |

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, green on the `13` §2 gate set.

### WP-17.1 — Propellant property layer + crate boundary

- **title:** `PropellantEos` trait, provenance-pinned property decks (LOX/LN2/LCH4/LH2/RP-1 + He/GH2/GO2/N2), fail-closed envelopes, CoolProp code-to-code.
- **goal:** The shared thermophysical foundation for this document and doc
  `05`'s thermochemistry/feed work, with the crate-boundary decision
  (module-in-`openbmp-feedsystem` vs `openbmp-cryo`) reviewed and recorded.
- **fidelity_tier:** T1
- **depends_on:** []
- **new_crates:** boundary review per §3.1 — first commit is the skeleton +
  placement justification; depends only on `openbmp-core`/`-state`/`-models`.
- **touched:** new crate/module, `data/` property decks + `provenance.md`,
  tolerance tables.
- **approach:** §3.2. Wagner/Antoine-form saturation fits + caloric
  polynomials from published tabulations; range-checked; deterministic.
- **acceptance:**
  - property decks match CoolProp tabulations to < 0.5% over declared
    envelopes (tolerance table per fluid)
  - out-of-envelope calls return `Err` (property tests)
  - no property literal appears outside the deck files (tripwire extended)
  - all `13` §2 gates green
- **validation_label:** `checked`
- **dual_use_note:** far from line — thermophysical constants.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** curve-fit tier, not a REFPROP-grade reference EOS.

### WP-17.2 — Two-zone tank: self-pressurization, boiloff, vent

- **title:** Ullage/liquid two-zone energy balance with saturated interface, lumped wall nodes, MLI heat-leak deck, vent/relief valves.
- **goal:** The sim stops pretending ullage is isentropic. Tank pressure,
  temperature, boiloff, and vented mass become physical outputs of an energy
  balance, replacing the T0 blowdown when a scenario opts in.
- **fidelity_tier:** T1
- **depends_on:** [WP-17.1]
- **new_crates:** none (extend WP-17.1's home).
- **touched:** cryo module, `crates/openbmp-runner/src/tanks.rs` wiring,
  `crates/openbmp-scenario` schema (`[vehicle.tanks.*.cryo]`), gated
  telemetry channels.
- **approach:** §3.3 ODE set; fixed sub-stepping; interface closure via
  film-coefficient Nusselt correlations; Lockheed-form MLI conductance deck.
- **acceptance:**
  - off by default; canonical goldens byte-identical
  - adiabatic closed-tank analytic case conserves mass/energy to < 1e-9 rel
  - per-run closure audit < 1e-10 rel; vent hysteresis latches at sub-step
    boundaries (property test)
  - MHTB-trend case documented with honest residuals (two-zone under-predicts
    dp/dt; stated in the tolerance table)
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 3–4 weeks
- **parity_ceiling:** settled propellant only; two-zone bulk states; no
  stratification yet.

### WP-17.3 — Pressurant systems: COPV source, pre-press, helium injection

- **title:** Bottle blowdown thermodynamics, regulator/orifice flow, cold-ullage injection with emergent collapse factor, pre-press sequencing hooks.
- **goal:** Pressure-fed and pump-fed ullage maintenance becomes physical:
  injected pressurant mass/enthalpy interacts with the two-zone balance, and
  the classical collapse factor is recovered as a diagnostic, not a knob.
- **fidelity_tier:** T2
- **depends_on:** [WP-17.2]
- **new_crates:** none.
- **touched:** cryo module (source + valve models), scenario schema, telemetry.
- **approach:** §3.4. Choked/subsonic orifice with locked branch selection;
  real-gas-lite helium deck; declared diffuser efficiency.
- **acceptance:**
  - off by default; goldens byte-identical
  - bottle blowdown matches the closed-form adiabatic/isothermal bounds
    (< 1% at the bounds; trajectory between them)
  - collapse-factor diagnostic falls within the published range for the
    benchmark configuration (reported, not tuned)
  - GFSSP manual example case matches to < 10% (tolerance table)
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** no COPV structural/thermal qualification physics; no
  proprietary regulator maps.

### WP-17.4 — Autogenous bleed interface to doc 05

- **title:** Engine heat-exchanger bleed state (ṁ, h) as the pressurant source, with a declared-schedule stub usable before 05 T4.
- **goal:** Methalox/hydrolox vehicles pressurize the way real ones do; the
  `05` feed network and this tank model exchange boundary conditions through
  one reviewed interface.
- **fidelity_tier:** T2
- **depends_on:** [WP-17.3]
- **new_crates:** none.
- **touched:** cryo module, `crates/openbmp-propulsion/src/engine.rs`
  interface surface (with doc `05`), scenario schema.
- **approach:** §3.4 autogenous path; trait-level boundary so either side can
  ship first; stub = declared (ṁ, T) schedule with provenance.
- **acceptance:**
  - off by default; goldens byte-identical
  - energy bookkeeping closes across the interface (audit < 1e-10 rel)
  - runs with stub and with `05` T4 (when present) produce identical results
    for matched boundary histories (seam test)
  - all `13` §2 gates green
- **validation_label:** `experimental` → `validated-toy` once 05 T4 exists
- **dual_use_note:** far from line.
- **est_effort:** 1–2 weeks
- **parity_ceiling:** heat-exchanger internals belong to doc `05`; here only
  the boundary state.

### WP-17.5 — Stratification + TVS

- **title:** N-layer 1-D thermal stratification (liquid + ullage), TVS with J-T expansion and mixing-jet efficiency, slosh-mixing hook.
- **goal:** Self-pressurization rates approach published test behavior, and
  long-coast pressure control (vent vs TVS trade) becomes simulable —
  the discriminating CFM capability.
- **fidelity_tier:** T3
- **depends_on:** [WP-17.2]
- **new_crates:** none.
- **touched:** cryo module (layer state, mixing operator, TVS), scenario
  schema, telemetry.
- **approach:** §3.5; buoyancy-ordered layers, deterministic mixing operator,
  declared η_jet with UQ tag; `02` slosh-energy destratification hook behind
  its own flag.
- **acceptance:**
  - off by default; goldens byte-identical
  - layer model reduces to two-zone when N=1 byte-identically (regression)
  - MHTB LH2 pressure-rise curve matched to < 20% dp/dt (tolerance table,
    honest residual discussion)
  - AS-203 qualitative envelope case documented with provenance
  - TVS duty-cycle case: bulk saturation pressure held within band; energy
    audit closes
  - all `13` §2 gates green
- **validation_label:** `research`
- **dual_use_note:** far from line.
- **est_effort:** 4–5 weeks
- **parity_ceiling:** 1-D engineering stratification; no CFD mixing fields
  (ingestion path is WP-17.8); settled propellant only.

### WP-17.6 — Chilldown correlations

- **title:** Line/engine chilldown marching model with film/transition/nucleate boiling correlation decks and locked regime hysteresis.
- **goal:** Chilldown propellant consumption and time-to-quality gates become
  computable countdown quantities instead of declared constants.
- **fidelity_tier:** T4
- **depends_on:** [WP-17.1]
- **new_crates:** none.
- **touched:** cryo module (chilldown), `data/` correlation decks, scenario
  schema.
- **approach:** §3.6; quasi-steady segment marching; published boiling
  correlations as pinned decks; regime map reported with validity envelope.
- **acceptance:**
  - off by default; goldens byte-identical
  - regime selection matches the published boiling map on the test matrix
    (property test)
  - published line-chilldown case: regime-correct, time-to-chill within 30%
    (correlation-tier tolerance table)
  - all `13` §2 gates green
- **validation_label:** `checked` → `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** correlation tier; no two-fluid transient CFD; no
  facility-specific calibration.

### WP-17.7 — Loading & conditioning timeline (couples doc 18)

- **title:** Fill/replenish/drainback/stop-flow tank loading states, densified-propellant conditioning schedule, consumables bookkeeping, countdown coupling.
- **goal:** The countdown's propellant story — load-and-go late replenish,
  topping against boiloff, recycle drainback, subcooled conditioning — runs
  inside the sim and feeds `18`'s sequencer and recycle logic.
- **fidelity_tier:** T4
- **depends_on:** [WP-17.2, WP-17.6]
- **new_crates:** none.
- **touched:** cryo module (loading FSM), scenario schema
  (`[tanks.loading]`), `18` sequencer interface, telemetry.
- **approach:** §3.6; loading states as a deterministic FSM advanced by the
  countdown sequencer; flash evaporation at warm-wall contact via the energy
  balance; conditioning as subcooled-target schedules.
- **acceptance:**
  - off by default; goldens byte-identical
  - steady replenish balances computed boiloff to < 1e-6 rel (analytic)
  - hold→recycle→resume sequence is deterministic and consumables-conserving
    (scenario test with `18` stub events)
  - densified conditioning reaches declared subcool target within the
    declared timeline on the reference case
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 3 weeks
- **parity_ceiling:** no facility hydraulics (pumps/valves of the ground
  plant are `18`'s declared models); no geyser/hazard analyses.

### WP-17.8 — Validation pack + UQ wiring + ingestion path

- **title:** MHTB/AS-203 digitized benchmark assets, uncertainty tags on heat leak/mixing/correlations into 11, ingested high-fidelity deck path.
- **goal:** The dimension earns its labels: public-benchmark anchoring,
  dispersion-ready uncertainty, and the solver-consumer ingestion path for
  externally produced mixing/boiling decks.
- **fidelity_tier:** T5
- **depends_on:** [WP-17.5, WP-17.6]
- **new_crates:** none.
- **touched:** `data/` benchmark assets + `provenance.md`, UQ tags, ingestion
  parser + schema, tolerance tables.
- **approach:** §5 table; `13` §6 ingestion pattern (parser + provenance +
  UQ wrapper + coupling adapter + code-to-code case).
- **acceptance:**
  - benchmark assets provenance-pinned with licenses recorded
  - per-coefficient bias/random margins flow into `11` campaigns
    (dispersion smoke case)
  - ingested-deck path round-trips with SHA verification and fails closed on
    mismatch
  - all `13` §2 gates green
- **validation_label:** `research`
- **dual_use_note:** far from line; published-data discipline only (no
  fielded-vehicle tank data).
- **est_effort:** 2–3 weeks
- **parity_ceiling:** validation parity (instrumented tanking tests of a real
  vehicle) remains structurally out of reach.

---

## 10. References

- TankSIM: NTRS 20170005533 — "TankSIM: A Cryogenic Tank Performance
  Prediction Program" (MSFC).
- GFSSP user manual + example problems (NASA MSFC, published).
- SINDA/FLUINT cryogenic applications: NTRS 20190031763, NTRS 20150002709.
- AS-203 liquid-hydrogen orbital experiment flight results (NTRS, Saturn
  program reports).
- MHTB (Multipurpose Hydrogen Test Bed) self-pressurization and TVS test
  reports (NASA MSFC, NTRS series).
- Lockheed layered-MLI effective-conductivity correlation (published
  cryogenic engineering literature).
- Barron, R. F., *Cryogenic Systems* (property and boiling correlation
  background).
- NIST Chemistry WebBook thermophysical tabulations (public data source for
  property decks).
- Load-and-go practice and densified propellants: NASA Commercial Crew
  program public reporting; SpaceX AMOS-6 anomaly update (public statement on
  helium-loading reproduction).
- Rohsenow/Bromley boiling correlations (standard heat-transfer literature).
