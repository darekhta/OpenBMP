# Electrical Power Systems & Avionics Emulation

**Status:** `experimental` (design intent; no code shipped by this document).
**Audience:** the engineer or LLM agent implementing the parity work packages.
**Prerequisite reading:** `00-overview.md`, `13-agent-execution-playbook.md`,
`10-flight-software-in-the-loop-xil.md` (the SIL boundary this document
deepens), `docs/HAL.md` (the adopter contract the FC side must respect).

> One-line scope: give the vehicle an electrical dimension — a DC power
> network with battery packs (OCV/SoC equivalent-circuit models),
> physics-coupled loads (electric pump power from `05`, actuator transients
> from `09`), battery hot-swap/jettison as guidance-relevant flight events,
> power telemetry and FC-side power FDIR (load shedding, brownout-reset
> semantics), behavioral data-bus profiles on the existing transport seam,
> and an N-string redundancy bench ("cut the strings") with a declarative
> flatsat manifest.

Electrical power is flight-critical physics at the reference orgs: Electron
flies battery-powered pump-fed engines whose **high-voltage battery
hot-swap and jettison** is a scripted flight event that directly shapes
vehicle performance; SLS avionics are tested in a SIL where flight computers
run against emulated buses and peripherals (ARTEMIS); SpaceX's "table
rocket" lays out every flight controller and randomly kills computers
mid-simulated-flight. The workspace has no EPS model and doc `10` explicitly
ships "no real drivers / bus protocols" — this document adds the electrical
*behavioral* layer that fits the posture.

---

## 1. Parity target & ceiling

### 1.1 Reference practice

- **Rocket Lab** — Electron's Rutherford engines are electric-pump-fed; the
  Payload User Guide documents two high-voltage batteries powering second-
  stage pumps until depletion, a third taking over, and the depleted pair
  being **jettisoned to increase performance** (~T+6:50 on webcasts).
  Battery capability advances were "primarily" how Electron's payload
  capacity grew. ODySSy (Rocket Lab's spacecraft digital-twin stack) ships
  "Power Models" as a first-class category.
- **NASA MSFC — SLS SIL/ARTEMIS** — flight-identical core-stage avionics
  driven by a real-time environment with emulated buses and peripheral
  emulators (booster/engine/Orion/launch-control), MAESTRO as automated test
  conductor.
- **SpaceX** — the 2013 flight-software AMA describes the all-controllers
  "table": commodity hardware emulating every controller/processor, full
  simulated flights on the bench, and "cutting the strings" — randomly
  killing a flight computer mid-sim against the triple-redundant voting
  architecture.
- **Northrop/industry** — "flatsat" HITL racks where software models can
  replace hardware components and vice-versa; NOS3 is the open smallsat
  analog (cFS + simulated avionics + ground software).

### 1.2 Parity target (capability, within posture)

1. **EPS network** — a DC electrical network solved deterministically per
   step: sources (battery packs with open-circuit-voltage vs
   state-of-charge, internal resistance vs SoC/temperature), distribution
   (buses, switches, pyro circuits), loads (declared profiles +
   physics-coupled draws), bus voltage/current states, undervoltage events.
2. **Physics-coupled loads** — electric-pump power computed from the `05`
   feed state (`P = ṁ·Δp/(ρ·η)`), TVC/actuator transient draws from `09`
   actuator models, avionics base loads — closing the loop where battery
   sag is *caused by* flight dynamics.
3. **Staged-power events** — battery hot-swap and jettison as discrete
   events coupling the EPS (source switchover) and the vehicle tree (mass/
   CG drop via the existing staging machinery) — the Electron-class pattern.
4. **Power telemetry + FC power FDIR** — voltage/current/SoC transducers as
   `09`-pattern sensors; FC-side load-shedding tables, undervoltage
   thresholds, and watchdog brownout-reset semantics (a computer that loses
   power mid-step restarts through its declared boot path) — all
   hardware-portable.
5. **Behavioral bus profiles** — bandwidth/latency/jitter/error-injection
   profiles and static schedule tables on the existing `openbmp-bridge`
   `Transport` seam — bus *behavior* without bus *protocols* (consistent
   with `docs/standards-posture.md` and doc `10`'s ceiling).
6. **N-string redundancy bench** — kill/restart of FC instances mid-run
   over the SIL lifecycle (the "cut the strings" test), voter-mask
   observability, divergence detection between lanes, and a declarative
   **flatsat manifest** binding sim channels to node endpoints and fault
   profiles.

### 1.3 Parity ceiling (honest boundary)

- **No electrochemistry beyond equivalent circuits.** Cell models are
  OCV(SoC) + R_int(SoC, T) with declared capacity/temperature derating
  decks from *published* cell data; no electrochemical-impedance or aging
  models, no proprietary pack data.
- **No bus protocol conformance.** MIL-STD-1553/SpaceWire/CAN-FD/TTE signal
  integrity, arbitration, and conformance remain out (doc `10` ceiling);
  this document models capacity/timing/error *behavior* only.
- **No electrical fault physics.** Arcs, shorts, and wiring-harness thermal
  behavior are not modeled; electrical faults are state-level (source loss,
  switch failure, bus undervolt) injected at declared seams.
- **No real avionics hardware data.** Real controller power draws, boot
  times, and redundancy implementations are proprietary; values are
  declared scenario inputs with provenance.
- **PIL/processor emulation stays doc `10`'s** (Renode-class); this document
  feeds it power/bus context but does not move that boundary.

---

## 2. Current state in source

- No EPS model anywhere: no battery, bus, load, or power state exists.
- `crates/openbmp-bridge/src/{transport.rs,fault.rs,packet.rs,zoh.rs}` —
  the transport + scripted fault seam (doc `10`); profiles in §3.6 attach
  here.
- `crates/openbmp-fc/src/{voter.rs,fdir.rs,health.rs,scheduler.rs,
  commander.rs}` — TMR voting, FDIR, health monitoring: the consumers of
  power-fault effects and the redundancy bench's observable surface.
- `crates/openbmp-hal/src/lib.rs` — the HAL trait contract (clocks, sensors,
  actuators, storage, watchdog service) the brownout-reset semantics must
  respect.
- `crates/openbmp-propulsion/src/engine.rs` — engine interface; no pump
  power hook yet (the `05` feed network is designed, not built — a declared
  pump-power stub bridges until it lands).
- `crates/openbmp-sensors/src/` — the sensor/error-model pattern power
  transducers reuse.
- `crates/openbmp-runner/src/{fc_bridge.rs,sil.rs}` + `openbmp-sil` — SIL
  wiring and lifecycle the N-string bench extends.
- `scenarios/multi-engine-octaweb` — the cluster scenario family where
  pump-power coupling becomes materially interesting.

---

## 3. Target architecture

### 3.1 Crate boundary

`openbmp-eps` (L2): the electrical network is vehicle plant physics
(consumes propulsion/actuator states, produces bus states and events),
sitting beside `openbmp-propulsion`/`openbmp-vehicle`. FC-side additions
(power FDIR, load-shed tables, brownout semantics) land in `openbmp-fc`
behind the portability lock — the FC sees transduced channels, never the
network solver. Bus profiles extend `openbmp-bridge`. The flatsat manifest
and bench live with the SIL machinery (`openbmp-sil`/runner).

### 3.2 EPS network model (T1)

```rust
pub struct EpsNetwork {
    pub nodes: Vec<BusNode>,           // declared topology, registration-ordered
    pub sources: Vec<SourceElement>,   // battery packs, ground/umbilical source
    pub loads: Vec<LoadElement>,       // declared + coupled draws
    pub switches: Vec<SwitchElement>,  // commanded/scheduled, latched states
}
```

- **Solve:** per major step, a deterministic DC nodal solve (small dense
  system, fixed ordering, direct solve — no iterative tolerance on the hot
  path; piecewise source characteristics are linearized about the latched
  operating branch and branch transitions latch at step boundaries).
- **Battery pack:** series/parallel cell arrangement; SoC ODE
  (`dSoC/dt = −I/(C_eff(T))`), OCV(SoC) deck, R_int(SoC, T) deck, declared
  thermal state (coupled later to `17`-class thermal nodes if configured);
  voltage `V = OCV − I·R_int`. Decks digitized from published cell data
  with provenance.
- **Ground source:** umbilical supply until the `18` T-0 handover event
  (internal-power transfer = source switch, observable as the classic
  countdown milestone).
- **Outputs:** per-bus voltage/current, per-source SoC/current, energy
  bookkeeping audit (∫P_source = ∫P_load + losses + Δstored).

### 3.3 Physics-coupled loads (T2)

- **Electric pump power** (the Electron-class coupling):
  `P_pump = ṁ·Δp/(ρ·η_pump·η_motor·η_inverter)` from the `05` feed state
  (declared-stub Δp schedule until `05` T4 lands). Thrust now *requires*
  electrical power: battery sag → bus undervolt → pump derate is a
  physical causal chain (derate map declared; fail-closed below floor).
- **Actuator draws:** TVC/fin actuator electrical transients from `09`
  actuator states (declared electrical conversion per actuator class);
  RCS valve solenoid pulses (per-pulse charge draw).
- **Avionics loads:** declared per-node base + mode-dependent loads (flight
  computers, radios tied to `20` transmit states).

### 3.4 Staged-power events (T2)

Hot-swap: a declared event (mission-FSM or sequencer-driven) latching a
source switchover (depleting packs offline, fresh pack online) with a
declared transfer transient; jettison: the existing staging/jettison
machinery (doc `01` separation vocabulary; `crates/openbmp-vehicle`
assembly tree) drops pack mass/CG while the EPS removes the source — one
event, two coupled effects, byte-deterministic. The performance effect
(mass drop vs remaining energy) becomes a first-class trade visible to
`07`-class staging analyses (vehicle-intrinsic objective, consistent with
the existing staging-analysis scoping).

### 3.5 Power telemetry + FC power FDIR (T3)

- **Transducers:** bus voltage/current and pack SoC/temperature sensors via
  the `09` pattern (error decks, sample rates, time tags) — the FC sees
  channels, not truth.
- **FC-side (portability-locked):** load-shed tables (declared sheddable
  loads by priority; commanded through the existing command path),
  undervoltage detection with persistence windows, and **brownout-reset
  semantics**: a node whose supply drops below its declared hold-up
  restarts through the HAL watchdog/boot path (state loss per the declared
  retention contract, re-entry through FDIR recovery — doc `06`'s
  abort-to-SAFE vocabulary unchanged). This makes power faults *mean*
  something to the flight software, which is the entire point of the
  reference-org benches.

### 3.6 Behavioral bus profiles (T4)

`TransportProfile` decorating any existing `Transport`: rate limit
(bytes/step), fixed + load-dependent latency, jitter from a
`DeterministicRng` domain, error injection (drop/corrupt with declared
rates), and optional static schedule tables (per-topic slot assignments —
TTE-*shaped* timing behavior, no protocol claim). Profiles compose with the
`10` fault ports (scripted faults still win). Acceptance includes the
degenerate profile (infinite rate, zero latency) being byte-identical to no
profile — the seam-stability proof.

### 3.7 N-string bench + flatsat manifest (T4)

- **String kill/restart:** the SIL lifecycle (doc `10` FSM) gains
  kill/restart verbs per FC node; a killed node's outputs vanish from the
  bus (voter sees it); restart re-enters through the boot path with
  declared boot time. "Cut the strings" = scripted or `DeterministicRng`-
  scheduled kills during flight scenarios.
- **Observability:** voter masks, lane divergence metrics (estimate deltas
  between strings), reconfiguration events — telemetered and judged by
  `21` verdict rules.
- **Flatsat manifest:** one TOML document declaring nodes (FC instances +
  their transports + power nodes + fault profiles) — the bench-as-data
  pattern (MAESTRO-class conduction comes from `18`'s rehearsal harness +
  `21` manifests; this document owns the binding).

### 3.8 Fidelity tiers

- **T0 (current):** no electrical dimension.
- **T1:** EPS network + batteries + declared loads + energy audit —
  `validated-toy`.
- **T2:** physics-coupled draws + hot-swap/jettison events —
  `validated-toy`.
- **T3:** power transducers + FC load-shed/brownout-reset — `validated-toy`.
- **T4:** bus profiles + N-string bench + flatsat manifest —
  `validated-toy`.

---

## 4. Invariant preservation

- **Determinism:** fixed-order nodal solve; branch latching at step
  boundaries; all stochastic elements (jitter, scheduled kills) on
  domain-separated `DeterministicRng` streams; SIL determinism hash extends
  over power/bus states.
- **Byte-stable default:** behind `[vehicle.eps]` / `[sil.flatsat]`;
  goldens untouched; degenerate-profile identity proof (§3.6).
- **FC portability:** `openbmp-fc` gains tables/thresholds/semantics only —
  no edge to `openbmp-eps`/`-bridge`/simulator crates; the existing
  dependency tripwire covers it; brownout-reset uses only HAL-contract
  surfaces.
- **Provenance:** cell decks, derate maps, and load tables under `data/`
  with `provenance.md` + SHA pins; no fielded-vehicle electrical data.
- **Fail-closed:** topology errors (floating nodes, conflicting sources),
  deck-envelope violations, and unknown manifest fields are load errors;
  pump operation below voltage floor is a fault event, not a silent clamp.
- **Labels:** per §3.8.

---

## 5. V&V plan

| Case | Type | Tier | Tolerance/criterion |
|---|---|---|---|
| Nodal solve vs hand-computed resistive networks | analytic | T1 | < 1e-12 rel |
| Battery discharge vs published cell curve deck | code-to-code (digitized data) | T1 | < 2% voltage over declared SoC range (table) |
| Energy bookkeeping audit every run | property | T1+ | closure < 1e-10 rel |
| Pump power chain: thrust ↔ electrical power consistency | internal cross-check (with `05` stub) | T2 | `P_elec·η = ṁΔp/ρ` exact at the interface |
| Hot-swap continuity: bus voltage within declared envelope through transfer | scenario | T2 | envelope respected; event deterministic |
| Jettison mass/CG effect matches staging machinery | regression vs `01` path | T2 | byte-identical to equivalent declared jettison |
| Undervolt → load-shed → recovery sequence | scenario (FC-in-loop) | T3 | declared shed order; deterministic timeline |
| Brownout reset: node restart through HAL boot path with declared state loss | SIL scenario | T3 | voter mask reflects outage; recovery per FDIR; deterministic |
| Degenerate bus profile identity | golden | T4 | byte-identical to no-profile run |
| Loss/latency statistics vs profile decks | statistical (`11` sizing) | T4 | within Clopper-Pearson bounds |
| String-kill campaign: 2-of-3 survival, dual-kill abort-to-SAFE | MC scenario | T4 | verdicts match declared redundancy logic; deterministic across workers |

---

## 7. Dependencies on other parity docs

- `05` — feed-state Δp/ṁ for pump power (declared stub until T4 lands);
  engine-out interplay with power loss cases.
- `09` — actuator electrical transients; transducer error-model pattern;
  multi-string sensor voting context.
- `10` — Transport seam for profiles; SIL lifecycle for kill/restart; PIL
  boundary unchanged; determinism-hash extension.
- `18` — umbilical/internal-power T-0 handover event; battery state in
  recycle feasibility.
- `01` — jettison through the separation vocabulary; pack mass in the
  assembly tree.
- `06` — FDIR/abort-to-SAFE consumes power-fault context; engine-out +
  power-out combined cases.
- `11`/`12` — string-kill campaign statistics; deterministic parallel
  execution.
- `20` — radio transmit loads; link loss vs power loss disambiguation in
  fault campaigns.
- `21` — bench verdicts, divergence metrics, campaign manifests.

---

## 8. Open-source leverage

| Tool/data | Use mode | License/status |
|---|---|---|
| Published Li-ion/Li-poly cell characterization data | OCV/R_int deck sources (digitized, pinned) | public literature |
| NOS3 | reference architecture for node/ground binding patterns | NASA open source |
| Electron Payload User Guide | staged-power event structure anchor (public document) | public |
| SLS SIL/ARTEMIS + MAESTRO public articles | bench/conductor architecture anchors | public |
| 2013 SpaceX flight-software AMA (archived) | "table"/string-kill practice anchor | public web |
| Shepherd/Thevenin battery-model literature | equivalent-circuit method reference | published literature |
| cFS (via doc `10` T6) | flight-software consumer for bench scenarios | NASA open source |

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, green on the `13` §2 gate set.

### WP-23.1 — `openbmp-eps` skeleton: network, batteries, declared loads

- **title:** L2 EPS crate with deterministic DC nodal solve, battery pack equivalent-circuit models (OCV/SoC/R_int decks), switches, ground source, energy audit.
- **goal:** The vehicle gains an electrical state: buses with voltages,
  packs with state of charge, and an energy ledger that must close —
  the substrate for every coupling that follows.
- **fidelity_tier:** T1
- **depends_on:** []
- **new_crates:** **`openbmp-eps` (L2)** — skeleton + placement
  justification first (§3.1); never an `openbmp-fc` dependency.
- **touched:** `crates/openbmp-eps/**(new)`, `crates/openbmp-runner` wiring,
  `crates/openbmp-scenario` (`[vehicle.eps]`), `data/` cell decks +
  `provenance.md`, gated telemetry.
- **approach:** §3.2; fixed-order direct solve; branch latching at step
  boundaries; SoC ODE on the kernel step.
- **acceptance:**
  - `[vehicle.eps]` off by default; canonical goldens byte-identical
  - resistive-network analytic cases < 1e-12 rel
  - discharge curve vs digitized cell deck < 2% (tolerance table)
  - per-run energy audit closes < 1e-10 rel
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line — vehicle-intrinsic plant.
- **est_effort:** 3 weeks
- **parity_ceiling:** equivalent-circuit tier; no aging/abuse physics; no
  proprietary pack data.

### WP-23.2 — Physics-coupled loads (pump power, actuators, avionics)

- **title:** Electric-pump draw from feed state (with declared stub), TVC/RCS actuator transients, mode-dependent avionics loads; undervolt derate fail-closed.
- **goal:** Electrical power becomes causal: throttle demands watts, watts
  sag volts, and a depleted bus physically derates thrust — the
  Electron-class coupling that makes EPS a flight-dynamics dimension.
- **fidelity_tier:** T2
- **depends_on:** [WP-23.1; cross-doc: WP-05.4-a preferred, declared-stub
  otherwise; `09` actuator states]
- **new_crates:** none.
- **touched:** `crates/openbmp-eps` (coupled loads),
  `crates/openbmp-propulsion/src/engine.rs` interface hook, scenario schema.
- **approach:** §3.3; efficiency-chain power computation; declared derate
  map with hard floor fault.
- **acceptance:**
  - off by default; goldens byte-identical
  - interface identity `P_elec·η = ṁΔp/ρ` exact (cross-check test)
  - below-floor operation raises the declared fault event (fail-closed,
    never silent clamp)
  - stub-vs-`05` equivalence for matched boundary histories (seam test)
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** declared efficiency chains; no motor/inverter
  electromagnetics.

### WP-23.3 — Battery hot-swap + jettison events

- **title:** Source-switchover transient + pack jettison through the existing separation/staging machinery; energy-vs-mass trade observability.
- **goal:** The staged-power flight event, end to end: swap keeps the pumps
  alive, jettison drops the mass, and both effects land deterministically
  in one event — reproducing the documented Electron pattern.
- **fidelity_tier:** T2
- **depends_on:** [WP-23.2; cross-doc: `01` separation vocabulary (or the
  existing jettison path in `crates/openbmp-vehicle` pre-`01`)]
- **new_crates:** none.
- **touched:** `crates/openbmp-eps` (events), `crates/openbmp-vehicle`
  assembly hooks, mission-event bindings, scenario schema.
- **approach:** §3.4; one declared event, two coupled effects; transfer
  transient envelope declared.
- **acceptance:**
  - off by default; goldens byte-identical
  - bus voltage stays within the declared envelope through swap (scenario)
  - jettison mass/CG effect byte-identical to an equivalent declared
    jettison without EPS (regression)
  - staging-analysis output reflects the energy-vs-mass trade only through
    existing vehicle-intrinsic objectives
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line — trade feeds vehicle-intrinsic staging
  vocabulary only.
- **est_effort:** 2 weeks
- **parity_ceiling:** declared transfer transients; no real switchgear
  characterization.

### WP-23.4 — Power transducers + FC load-shed/brownout FDIR

- **title:** Voltage/current/SoC sensors (09 pattern), FC-side undervolt detection, priority load-shed tables, watchdog brownout-reset semantics via the HAL contract.
- **goal:** Power faults become something the flight software experiences
  and survives: detection through real (modeled) sensors, shedding through
  the command path, and reset through the boot path — hardware-portable
  throughout.
- **fidelity_tier:** T3
- **depends_on:** [WP-23.1]
- **new_crates:** none.
- **touched:** `crates/openbmp-sensors` (power transducers),
  `crates/openbmp-fc` (power FDIR: tables, thresholds, shed/reset
  semantics — portability lock), `crates/openbmp-hal` boot/watchdog surface
  use, scenario schema.
- **approach:** §3.5; persistence-windowed detection; monotone shed order;
  reset = declared state-retention contract + FDIR re-entry.
- **acceptance:**
  - off by default; goldens byte-identical
  - shed sequence follows the declared priority table exactly (property)
  - brownout-reset SIL case: outage visible in voter mask, recovery per
    declared FDIR path, fully deterministic
  - `fc_dependency_tripwire` green (no new FC edges)
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** protective logic; abort vocabulary unchanged
  (shed/reset/reconfigure only).
- **est_effort:** 2–3 weeks
- **parity_ceiling:** declared hold-up/boot parameters; no real avionics
  power characterization.

### WP-23.5 — Behavioral bus profiles on the transport seam

- **title:** `TransportProfile` (rate/latency/jitter/error/schedule tables) decorating existing transports, composing with `10` fault ports; degenerate-identity proof.
- **goal:** Bus capacity and timing become testable constraints — TTE-shaped
  scheduling behavior and error climates without claiming any protocol —
  deepening SIL realism exactly where doc `10` drew its ceiling.
- **fidelity_tier:** T4
- **depends_on:** [WP-23.1 optional; cross-doc: WP-10.1 (transports),
  WP-10.2 (fault ports)]
- **new_crates:** none (extends `openbmp-bridge`).
- **touched:** `crates/openbmp-bridge/src/{transport.rs,fault.rs}` (profile
  decorator), scenario schema, SIL determinism hash coverage.
- **approach:** §3.6; deterministic jitter domains; scripted faults
  compose-over profiles.
- **acceptance:**
  - off by default; goldens byte-identical
  - degenerate profile run byte-identical to no-profile run (golden)
  - loss/latency statistics within Clopper-Pearson bounds of decks
  - schedule-table slotting deterministic across runs/worker counts
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line; no protocol conformance claimed
  (standards posture).
- **est_effort:** 2 weeks
- **parity_ceiling:** behavioral only — no 1553/SpaceWire/CAN protocol or
  electrical layer.

### WP-23.6 — N-string bench: kill/restart, divergence observability, flatsat manifest

- **title:** SIL lifecycle kill/restart verbs per FC node, voter-mask/lane-divergence telemetry, scripted and seeded string-kill campaigns, declarative flatsat manifest.
- **goal:** The "cut the strings" capability: redundancy claims become
  campaign results — 2-of-3 survival, dual-fault abort-to-SAFE — produced
  deterministically from one bench-as-data manifest.
- **fidelity_tier:** T4
- **depends_on:** [WP-23.4, WP-23.5; cross-doc: WP-10.1/WP-10.2 (SIL
  lifecycle + ports)]
- **new_crates:** none (extends `openbmp-sil` + runner wiring).
- **touched:** `crates/openbmp-sil` (bench module),
  `crates/openbmp-runner/src/sil.rs`, manifest schema, verdict integration
  with `21`, fixtures.
- **approach:** §3.7; kills via lifecycle FSM (power-caused kills route
  through WP-23.4 semantics; direct kills remain for isolation); divergence
  metrics on lane estimates; manifest validated fail-closed.
- **acceptance:**
  - off by default; goldens byte-identical
  - single-kill case: voter mask updates, mission continues (scenario)
  - dual-kill case: declared abort-to-SAFE verdict (scenario)
  - seeded kill campaign deterministic across worker counts; verdicts
    judged via `21` rules
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line — resilience testing of own stack.
- **est_effort:** 3 weeks
- **parity_ceiling:** simulated nodes (process-level); real-hardware
  benches remain downstream-owned per `docs/HAL.md`.

---

## 10. References

- Rocket Lab Electron Payload User Guide 8.0 — high-voltage battery
  hot-swap/jettison sequence (public document).
- SLS Systems Integration Lab / ARTEMIS / MAESTRO public articles (NASA
  MSFC).
- SpaceX flight-software AMA (2013, archived compilations) — table tests,
  string-killing, triple-redundancy practice.
- NOS3 documentation (NASA IV&V) — open flatsat architecture.
- Shepherd, C. M., "Design of Primary and Secondary Cells" (equivalent-
  circuit battery modeling lineage); standard Thevenin-model literature.
- Published lithium cell characterization datasets (OCV/R_int vs SoC/T)
  used for decks (cited per-deck in `provenance.md`).
- Doc `10` references for the XIL ladder this document deepens.
