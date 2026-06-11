# Telemetry, RF Links, Ground Stations & Tracking

**Status:** `experimental` (design intent; no code shipped by this document).
**Audience:** the engineer or LLM agent implementing the parity work packages.
**Prerequisite reading:** `00-overview.md`, `13-agent-execution-playbook.md`,
`10-flight-software-in-the-loop-xil.md` (the transport/fault seam this
document drives), `docs/standards-posture.md` (no conformance claims).

> One-line scope: make the radio link a simulated physical system — station
> geometry and visibility, antenna-pattern decks with body masking, link
> budgets down to frame-error rate, link-state-driven telemetry effects at
> the existing bridge fault seam, entry plasma blackout, a multi-station
> network with handover, ground tracking observables (range/Doppler/angles)
> for reconstruction, and frame/dictionary-shaped export adapters for
> external ground tools.

Today telemetry is perfect: every channel reaches the recorder with no
geometry, no link, no dropout physics. Reference practice treats the link as
a system with its own commit criteria — Rocket Lab's "It's a Test" abort was
a telemetry ground-equipment fault tripping a data-loss timeout, and
telemetry continuity is a documented hold/terminate criterion. Doc `09`
models GNSS reception for navigation; nothing models the vehicle's own
downlink/uplink.

---

## 1. Parity target & ceiling

### 1.1 Reference practice

- **NASA** — SCaN Link Tool (RF/optical link-budget analysis with ITU-R
  atmospheric augmentation, NTRS 20190032147); CLASS (TDRSS user-link
  performance prediction); DSN scheduling automation (JPL SSS); entry
  radio-blackout analysis from plasma flowfields (survey NTRS 20100008938);
  CCSDS framing across the fleet; Open MCT for telemetry display.
- **Rocket Lab** — owned downrange tracking site (Chatham Islands) relaying
  vehicle telemetry in real time; KSAT as sole ground-network provider for
  TT&C with automated pass scheduling; published S-band TM and GPS
  frequencies in the Payload User Guide; telemetry continuity as a hard
  commit/termination criterion (documented 2021 abort).
- **SpaceX** — all test/flight telemetry lands in a unified store with
  automated checks (practice documented by ex-team engineers); mission
  control takes over post-liftoff; multi-vehicle Starlink telemetry at
  fleet scale.
- **Cross-industry** — telemetry/data-platform engineering is a staffed
  specialty (Relativity); ground-station scheduling and link engineering are
  standing functions everywhere.

### 1.2 Parity target (capability, within posture)

1. **Geometry & visibility** — ground-site models (location, elevation mask,
   terrain mask deck), slant range, elevation/azimuth, Doppler, rise/set
   events, pass tables.
2. **Antenna & link budget** — vehicle/ground antenna gain decks (including
   body masking computed from the `03` panel mesh), EIRP → path loss →
   atmospheric losses (simplified ITU-R-shaped gas/rain attenuation decks)
   → pointing/polarization losses → C/N0 → Eb/N0 → frame-error rate through
   provenance-pinned coded-performance curves.
3. **Link-state-driven telemetry effects** — deterministic dropout/latency/
   corruption injection at the existing `openbmp-bridge` fault seam, driven
   by computed link state instead of hand-scripted faults; a
   telemetry-continuity criterion for `18`'s LCC and the AFTS-adjacent
   data-loss-timeout pattern (monitor-side, doc `10`).
4. **Entry blackout** — plasma-attenuation windows derived from the `04`
   edge-state electron-density estimate vs link frequency, validated against
   the published RAM-C reconstructions.
5. **Network & handover** — multi-station chains with declared schedules or
   deterministic greedy handover, relay (TDRSS-style geometry) tier,
   per-pass link reports.
6. **Tracking observables** — ground radar/ranging/angle measurement models
   (bias/noise decks, light-time optional) emitted as measurement streams
   for `11`'s BET and doc `24`'s reconstruction.
7. **Export adapters** — frame-shaped telemetry stream (CCSDS-like AOS/Space
   Packet shape) and channel-dictionary export (XTCE-shaped) so external
   ground tools (Yamcs/OpenMCT-class, via `21` shims) can consume OpenBMP
   runs; explicitly **non-conformant** naming per `docs/standards-posture.md`.

### 1.3 Parity ceiling (honest boundary)

- **No waveform/modem fidelity.** No modulation, coding, synchronization, or
  RF signal processing is implemented; the link is an engineering budget
  with published coded-performance curves consumed as data. GNU-Radio-class
  signal work is out of scope.
- **No standards conformance.** Exports are "CCSDS-shaped"/"XTCE-shaped" for
  interoperability testing, with no conformance claim (the established
  standards-posture pattern).
- **No real station calibration.** Site G/T, real antenna patterns, and
  network performance data are operator-proprietary; decks are synthetic or
  published-textbook values with provenance.
- **No operational scheduling product.** Pass scheduling here is
  deterministic and declarative; real network scheduling (DSN-class
  negotiation) is named, not designed.
- **Blackout ceiling.** Electron-density estimates come from `04`'s
  equilibrium-air tier (with its stated ceiling); blackout windows earn
  `research` only against the RAM-C public case.

---

## 2. Current state in source

- `crates/openbmp-telemetry/src/` (`channel.rs`, `schema.rs`, writers) —
  channelized recording to CSV/JSON/Parquet; no link between recorder and
  any physical path.
- `crates/openbmp-bridge/src/{transport.rs,fault.rs,zoh.rs,packet.rs}` —
  the SIL transport and fault-injection seam (drop/delay/corrupt exists as
  *scripted* stimulus per doc `10`); this document makes the stimulus
  *physical* (computed from link state).
- `crates/openbmp-sensors/src/gnss.rs` — navigation-side RF reception
  (doc `09` upgrades); the only RF-adjacent model in the workspace.
- `crates/openbmp-physics/src/{frames.rs,ephemeris.rs}` + `08` machinery —
  geometry substrate for site/vehicle vectors.
- `crates/openbmp-runner/src/fc_bridge.rs` — where uplink (command/I-load)
  and downlink (telemetry) cross the FC boundary; the natural place link
  effects bind.
- No site model, no link budget, no pass logic, no tracking observables, no
  blackout, no frame/dictionary export.

---

## 3. Target architecture

### 3.1 Crate boundary

`openbmp-comm` (L3, beside `openbmp-sensors`): link physics is a
measurement-like model layer — it reads truth state and environment and
produces link states and observables; it must never be imported by
`openbmp-fc` (the FC sees only the *effects*: missing/late packets, a
link-quality channel if a radio "sensor" is declared). Export adapters land
in `openbmp-telemetry` (host-side, feature-gated).

### 3.2 Sites, geometry & visibility (T1)

```rust
pub struct GroundSite {
    pub id: SiteId,
    pub geodetic: GeodeticPosition,      // via 08 frames; symbolic constants
    pub min_elevation_rad: f64,
    pub terrain_mask: Option<MaskDeck>,  // azimuth-binned elevation floor
}
```

Per step: site→vehicle vector in the local frame (existing `08`
transforms), elevation/azimuth/slant range, range rate (Doppler), visibility
boolean against mask, rise/set event extraction (fixed-step sign-change
detection, consistent with kernel event handling). Pass tables (AOS/LOS,
max elevation) are derived artifacts. Light-time correction is an optional
flag, off on the bit-stable path.

### 3.3 Antennas & link budget (T2)

- **Antenna decks:** gain vs (body-frame az/el) tables for vehicle antennas;
  gain vs boresight angle for ground apertures; provenance-pinned. **Body
  masking:** static occlusion table precomputed from the `03` watertight
  panel mesh (BVH ray casts from antenna location over the sphere), pinned
  as a deck — attitude then indexes the mask at runtime (deterministic,
  allocation-free).
- **Budget chain (per link, per step):**

```
C/N0 = EIRP + G_rx/T − L_fs − L_atm − L_rain − L_point − L_pol − k
L_fs = 20·log10(4π d / λ)
Eb/N0 = C/N0 − 10·log10(R_b)
FER   = curve_deck(code_id, Eb/N0)
```

  Atmospheric/rain losses from simplified ITU-R-shaped decks (elevation- and
  frequency-binned tables with provenance; not the full ITU-R recommendation
  chain). Coded performance (`FER(Eb/N0)`) from published curves for the
  declared code family, consumed as pinned data.
- **Link state:** `LinkState { c_n0_dbhz, eb_n0_db, fer, margin_db, visible,
  blackout }` per (vehicle antenna, site, band) — a telemetry channel set
  and the driver for §3.4.

### 3.4 Link-driven telemetry effects (T2)

The bridge fault seam gains a *source*: instead of (only) scripted faults, a
`LinkChannelModel` maps `LinkState` to deterministic packet effects — frame
loss via FER (per-packet Bernoulli draws on a `DeterministicRng` stream
domain keyed by `(seed, step, link, packet)`), latency from geometry +
declared processing delays, and corruption flags. Uplink (commands/I-loads)
and downlink (telemetry) are both subject. The **telemetry-continuity
criterion** (data-loss timeout) becomes computable: `18`'s LCC consumes it
pre-launch; in flight it feeds the `10` monitor side (a containment-monitor
input, never guidance).

### 3.5 Entry blackout (T3)

From `04`'s equilibrium-air edge state, an electron-density estimate along
the trajectory gives plasma frequency `f_p ≈ 8.98·√n_e` (Hz, n_e in m⁻³);
attenuation ramps as link frequency approaches `f_p` (engineering
attenuation model with declared shape), producing blackout entry/exit times
per band. Validation: RAM-C II published blackout windows by frequency
(the canonical open case, shared with `04`'s V&V ladder).

### 3.6 Network & handover (T4)

Multi-station chains with: declared schedules (TOML pass plans) or a
deterministic greedy scheduler (highest-margin visible site, hysteresis to
prevent flapping); handover events; relay tier (declared GEO relay node:
vehicle→relay→ground two-hop budget); per-pass link reports (duration,
margin profile, dropped-frame count). Downrange-chain configuration
reproduces the launch-ops pattern (pad site → downrange site → relay).

### 3.7 Tracking observables (T4)

Ground-based measurement models for reconstruction (not navigation):
range (transponder/echo), range-rate (Doppler), and angles (az/el) with
bias/noise/quantization decks per site instrument, time-tagged with declared
latencies, emitted as measurement streams alongside telemetry. Consumers:
`11`'s BET (RTS/batch GN already designed) and `24`'s reconciliation. The
models reuse the `09` measurement-error vocabulary (Gauss-Markov biases,
white noise, quantization) pointed at ground instruments.

### 3.8 Export adapters (T5)

- **Frame-shaped stream:** map the channel table into fixed-layout binary
  frames (CCSDS-AOS-like: sync, virtual-channel id, sequence count, payload
  of packed channels) — enough structure for external ground software to
  ingest; explicitly labeled non-conformant.
- **Dictionary export:** channel metadata (name, unit, type, calibration
  identity) to an XTCE-shaped XML/JSON document, so Yamcs/OpenMCT-class
  tools (shims in `21`) can auto-configure.
- Both are host-side, feature-gated, deterministic, and round-trip-tested
  against the native Parquet truth.

### 3.9 Fidelity tiers

- **T0 (current):** perfect telemetry; scripted faults only.
- **T1:** sites + visibility + passes — `validated-toy`.
- **T2:** antenna decks + budget + FER + link-driven effects + continuity
  criterion — `validated-toy`.
- **T3:** blackout vs RAM-C — `research`.
- **T4:** network/handover/relay + tracking observables — `validated-toy`.
- **T5:** frame/dictionary export + external-tool round trip — `checked`.

---

## 4. Invariant preservation

- **Determinism:** all draws on domain-separated `DeterministicRng` streams;
  geometry pure; greedy scheduler with hysteresis is a deterministic
  function of state; mask decks precomputed and pinned.
- **Byte-stable default:** behind `[comm]`; without it, telemetry remains
  perfect and goldens are untouched. Link effects on the bridge only apply
  in scenarios that opt in.
- **FC portability:** `openbmp-fc` never imports `openbmp-comm`; effects
  arrive as packet timing/loss through the existing bridge; link quality, if
  the vehicle "knows" it, is a declared sensor channel per `09` patterns.
- **Provenance:** antenna decks, attenuation decks, coded-performance
  curves, and mask decks under `data/` with `provenance.md` + SHA pins; no
  operator-proprietary patterns.
- **Fail-closed:** unknown bands/codes/sites, deck-envelope violations, and
  schema drift are load errors.
- **Labels:** per §3.9; blackout claims bounded by `04`'s ceiling.

---

## 5. V&V plan

| Case | Type | Tier | Tolerance/criterion |
|---|---|---|---|
| Visibility/passes vs external propagation oracle (skyfield/sgp4-class, external process) | code-to-code | T1 | rise/set < 1 s on the LEO reference case |
| Slant range/Doppler closed forms (circular orbit overflight) | analytic | T1 | < 1e-9 rel |
| Free-space loss + budget chain | analytic | T2 | < 1e-12 rel (pure arithmetic) |
| FER curve interpolation monotonicity + envelope | property | T2 | exact (monotone, clamped never silently) |
| Body-mask deck vs direct BVH ray cast (sampled attitudes) | internal code-to-code | T2 | mask agreement on all sampled rays |
| Link-driven dropout statistics vs FER | statistical (`11` sizing) | T2 | observed loss rate within Clopper-Pearson bounds of FER |
| Continuity criterion latching (data-loss timeout) | property/scenario | T2 | exact semantics; deterministic abort path with `18` |
| RAM-C II blackout windows by frequency | public benchmark | T3 | entry/exit within documented tolerance band (table; honest residuals) |
| Handover determinism + no-flapping hysteresis | property | T4 | identical pass plans across runs/thread counts |
| Tracking observables: bias/noise recovery by `11` batch GN on synthetic truth | self-consistency | T4 | estimates within CRLB-consistent bounds (table) |
| Frame/dictionary export round trip vs native Parquet | golden | T5 | lossless for declared channels; byte-stable artifacts |


---

## 7. Dependencies on other parity docs

- `08` — frames, geodetic machinery, atmosphere for attenuation context,
  ephemeris/epochs for pass timing.
- `03` — watertight panel mesh + BVH for antenna body-masking decks.
- `04` — edge-state electron density for blackout; shared RAM-C anchor.
- `09` — measurement-error vocabulary reused for ground instruments; GNSS
  stays doc `09`'s (reception for navigation), this doc owns vehicle
  TT&C links.
- `10` — bridge transport/fault seam the link effects drive; monitor-side
  data-loss-timeout pattern; SIL packets subject to link state.
- `11` — statistical sizing of dropout tests; BET consumes tracking
  observables.
- `18` — telemetry-continuity LCC criterion; umbilical→RF routing handover
  at T-0.
- `21` — pass/link reports as run records; Yamcs/OpenMCT shims consume the
  §3.8 exports.
- `24` — reconciliation consumes tracking observables and link metadata.
- `12` — determinism substrate for per-packet draws under parallel MC.

---

## 8. Open-source leverage

| Tool/data | Use mode | License/status |
|---|---|---|
| skyfield + sgp4 | external visibility/pass oracle (code-to-code, never linked) | MIT |
| ITU-R P-series structure | shape reference for attenuation decks (tables digitized from published examples, pinned) | published recommendations |
| Published coded-performance curves (CCSDS code family literature) | FER decks with provenance | public literature |
| RAM-C II reconstruction reports | blackout benchmark (shared with `04`) | public NTRS |
| Yamcs | external consumer for dictionary/frame exports (shims in `21`) | AGPL-3.0 (external process only) |
| NASA Open MCT | external display consumer (shims in `21`) | Apache-2.0 |
| OpenC3 COSMOS | named alternative consumer | AGPL-3.0 core (external) |
| SCaN Link Tool / CLASS papers | method anchors for budget structure | public NTRS |

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, green on the `13` §2 gate set.

### WP-20.1 — `openbmp-comm` skeleton: sites, visibility, passes

- **title:** L3 comm crate with ground-site schema, elevation/terrain masks, slant range/Doppler, rise/set events, pass tables.
- **goal:** The geometric substrate every link computation needs, validated
  against an external propagation oracle, with pass tables as deterministic
  artifacts.
- **fidelity_tier:** T1
- **depends_on:** []
- **new_crates:** **`openbmp-comm` (L3)** — skeleton + placement
  justification first (§3.1); never an `openbmp-fc` dependency.
- **touched:** `crates/openbmp-comm/**(new)`, `crates/openbmp-scenario`
  (`[comm.sites]`), gated telemetry channels.
- **approach:** §3.2 on `08` transforms; fixed-step event extraction; mask
  decks pinned.
- **acceptance:**
  - `[comm]` off by default; canonical goldens byte-identical
  - analytic overflight range/Doppler < 1e-9 rel; rise/set vs external
    oracle < 1 s (tolerance table)
  - pass-table artifact byte-stable
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line — geometry of own downlink.
- **est_effort:** 2 weeks
- **parity_ceiling:** point sites + mask decks; no real site surveys.

### WP-20.2 — Antenna decks + body masking from the panel mesh

- **title:** Vehicle/ground antenna gain decks and precomputed BVH occlusion masks indexed by attitude at runtime.
- **goal:** Antenna placement and vehicle attitude start mattering: link
  geometry inherits real body shadowing from the `03` mesh instead of
  assuming isotropic coverage.
- **fidelity_tier:** T2
- **depends_on:** [WP-20.1; cross-doc: WP-03.1 (watertight mesh + BVH)]
- **new_crates:** none.
- **touched:** `crates/openbmp-comm` (antenna module + mask precompute),
  `data/` antenna/mask decks + `provenance.md`.
- **approach:** §3.3; offline ray-cast table over the sphere, pinned; runtime
  table lookup, allocation-free.
- **acceptance:**
  - off by default; goldens byte-identical
  - mask deck matches direct ray casts on sampled attitudes (internal
    code-to-code, exact)
  - deck regeneration is deterministic (same mesh hash → same deck hash)
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 2 weeks
- **parity_ceiling:** static masks (no deployable-geometry changes mid-run
  beyond declared configuration switches); synthetic patterns.

### WP-20.3 — Link budget + coded-performance FER

- **title:** Full budget chain (EIRP→C/N0→Eb/N0→FER) with attenuation and coded-performance decks; `LinkState` channels.
- **goal:** Margin becomes a number with provenance: every link, every step,
  a budget and a frame-error rate from pinned decks.
- **fidelity_tier:** T2
- **depends_on:** [WP-20.2]
- **new_crates:** none.
- **touched:** `crates/openbmp-comm` (budget module), `data/` attenuation +
  FER decks + `provenance.md`, telemetry channels.
- **approach:** §3.3 chain; locked operand order; deck interpolation with
  monotonicity property tests.
- **acceptance:**
  - off by default; goldens byte-identical
  - budget arithmetic matches hand-computed analytic case < 1e-12 rel
  - FER deck interpolation monotone; out-of-envelope fails closed
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 2 weeks
- **parity_ceiling:** engineering budget; no waveform/modem physics.

### WP-20.4 — Link-driven telemetry effects + continuity criterion

- **title:** `LinkChannelModel` driving deterministic packet loss/latency/corruption at the bridge fault seam from computed `LinkState`; data-loss-timeout criterion for `18`/`10`.
- **goal:** Telemetry stops being free: dropouts and latency follow physics,
  uplink and downlink both, and the continuity commit/terminate criterion of
  reference practice becomes computable and testable.
- **fidelity_tier:** T2
- **depends_on:** [WP-20.3; cross-doc: WP-10.2 (fault ports)]
- **new_crates:** none.
- **touched:** `crates/openbmp-comm` (channel model),
  `crates/openbmp-bridge/src/fault.rs` seam (link-sourced stimulus),
  `crates/openbmp-runner/src/fc_bridge.rs` wiring, criterion shared with
  `18`.
- **approach:** §3.4; per-packet Bernoulli on domain-keyed streams; declared
  processing delays; monotone timeout latching.
- **acceptance:**
  - off by default; goldens byte-identical
  - observed loss rate within Clopper-Pearson bounds of commanded FER
    (statistical test, `11` sizing)
  - continuity-timeout scenario produces the declared hold/terminate verdict
    deterministically (with `18` stub)
  - SIL determinism hash unchanged for zero-effect link configurations
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** protective criterion; monitor-side only —
  `openbmp-fc` cannot import the link model.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** packet-level effects; no bit-level channel modeling.

### WP-20.5 — Entry plasma blackout

- **title:** Electron-density-driven attenuation windows per band from the `04` edge state, validated against RAM-C II.
- **goal:** The classic entry-comm phenomenon with the classic open anchor:
  blackout entry/exit per frequency from the trajectory itself.
- **fidelity_tier:** T3
- **depends_on:** [WP-20.3; cross-doc: WP-04.2-b (equilibrium-air edge
  state)]
- **new_crates:** none.
- **touched:** `crates/openbmp-comm` (blackout module), validation assets
  under `data/` + `provenance.md`.
- **approach:** §3.5; plasma frequency from n_e; declared attenuation ramp;
  windows as events.
- **acceptance:**
  - off by default; goldens byte-identical
  - RAM-C II window comparison documented with tolerance table and honest
    residuals (frequency ordering must be correct; timing within the
    declared band)
  - all `13` §2 gates green
- **validation_label:** `research`
- **dual_use_note:** far from line.
- **est_effort:** 2 weeks
- **parity_ceiling:** bounded by `04`'s equilibrium-air ceiling; no coupled
  EM/flowfield solution.

### WP-20.6 — Station network, handover & relay

- **title:** Multi-station chains with declared or deterministic-greedy pass plans, handover events, GEO-relay two-hop budgets, per-pass reports.
- **goal:** The downrange-chain and network dimension: who is listening when,
  with what margin, and what got lost in handover.
- **fidelity_tier:** T4
- **depends_on:** [WP-20.4]
- **new_crates:** none.
- **touched:** `crates/openbmp-comm` (network module), scenario schema,
  pass-report artifact (shared shape with `21`).
- **approach:** §3.6; greedy-with-hysteresis or declared plans; relay as a
  composed two-hop budget.
- **acceptance:**
  - off by default; goldens byte-identical
  - pass plans deterministic across runs/thread counts; hysteresis prevents
    flapping on the constructed boundary case (property)
  - relay budget equals composed single-hop budgets (analytic)
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 2 weeks
- **parity_ceiling:** declarative scheduling; no DSN-class negotiation.

### WP-20.7 — Ground tracking observables for reconstruction

- **title:** Range/range-rate/angle measurement models with instrument error decks, time tags, and measurement-stream emission for BET/reconciliation.
- **goal:** The external-tracking leg of every reference BET: independent
  observables of the vehicle's own flight, with errors honest enough to
  exercise `11`'s estimators and `24`'s reconciliation.
- **fidelity_tier:** T4
- **depends_on:** [WP-20.1; cross-doc: `09` error vocabulary, WP-11.6
  (BET consumers)]
- **new_crates:** none.
- **touched:** `crates/openbmp-comm` (tracking module), measurement-stream
  schema, `data/` instrument decks + `provenance.md`.
- **approach:** §3.7; Gauss-Markov biases + white noise + quantization;
  vehicle-state-id-bound observables (no arbitrary-point variant).
- **acceptance:**
  - off by default; goldens byte-identical
  - `11` batch GN on synthetic truth recovers injected biases within
    CRLB-consistent bounds (tolerance table)
  - observable API is vehicle-bound by type (no arbitrary-point variant) —
    asserted by a compile-visible test
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** measurement generation for self-reconstruction only;
  vehicle-bound API by type.
- **est_effort:** 2 weeks
- **parity_ceiling:** synthetic instrument decks; no real range-asset
  characteristics.

### WP-20.8 — Frame + dictionary export adapters

- **title:** CCSDS-shaped frame stream and XTCE-shaped channel dictionary export (host-side, feature-gated), round-trip-tested against native Parquet.
- **goal:** OpenBMP runs become consumable by the open ground-software
  ecosystem (Yamcs/OpenMCT-class via `21` shims) without any conformance
  claim — the interoperability layer reference practice assumes.
- **fidelity_tier:** T5
- **depends_on:** [WP-20.4]
- **new_crates:** none (lands in `openbmp-telemetry`, feature-gated).
- **touched:** `crates/openbmp-telemetry` (export module), schema docs,
  round-trip tests.
- **approach:** §3.8; fixed frame layout; dictionary generated from the
  channel schema; deterministic bytes.
- **acceptance:**
  - feature off by default; goldens byte-identical
  - export → re-import round trip lossless for declared channels (golden)
  - artifacts byte-stable; explicitly labeled non-conformant in docs and
    artifact headers
  - all `13` §2 gates green
- **validation_label:** `checked`
- **dual_use_note:** far from line — recorded sim data under existing
  provenance rules.
- **est_effort:** 2 weeks
- **parity_ceiling:** shaped-not-conformant; no certified ground-segment
  interface.

---

## 10. References

- SCaN Link Tool (NTRS 20190032147); CLASS TDRSS link analysis
  (NTRS 19850008643).
- Entry radio-blackout survey (NTRS 20100008938); RAM-C II flight
  reconstruction reports (NTRS series; shared anchor with doc `04`).
- Rocket Lab Electron Payload User Guide 8.0 (TM frequencies, downrange
  architecture); Rocket Lab post-flight analysis statement on the 2021
  telemetry-timeout abort (public).
- CCSDS blue books (TM/AOS/Space Packet, XTCE) — shape references only, per
  `docs/standards-posture.md`.
- JPL DSN scheduling automation (public AI-group pages) — named practice.
- Friis transmission and standard link-budget formulations (textbook).
- skyfield/sgp4 documentation (external oracle tooling).
