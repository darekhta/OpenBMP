# Parity Program — Implementation Progress Ledger

**Status:** `experimental` (tracking document; ships no code).
**Snapshot:** 2026-06-11, current branch state. Verified by code inspection
against each design doc's §9 acceptance criteria and `requirements.toml`
traceability (164 requirement ids).

**Marking criteria (the don't-overstate rule applies):**

- `implemented` — code + tests landed and the WP's acceptance bullets are
  satisfied on inspection. Final authority remains the `13` §2 gate set in
  CI; this ledger records evidence, it does not replace gates.
- `partial` — material elements landed; the entry names what is missing.
- `in progress` — uncommitted working-tree work.
- `not started` — no trace in the workspace.

A tier is only as done as its weakest required WP. Update this ledger in the
same PR as any WP merge.

---

## 1. Roll-up

Across the series: **207 work packages** (108 in `01`–`12`, 99 in `14`–`25`).
As of this snapshot: **20 implemented · 17 partial · 0 in progress ·
170 not started.**

| Doc | Dimension | Position on its tier ladder | WPs impl/partial/total | Next unblocked WP |
|---|---|---|---|---|
| 01 | Multibody dynamics | T0 baseline | 0/0/6 | WP-01.1 |
| 02 | Structural, loads, slosh, POGO | T0 (+ feed-side POGO prerequisite now exists via 05) | 0/0/9 | WP-02.1-a |
| 03 | Aero database & CFD | T0 baseline | 0/0/9 | WP-03.1 |
| 04 | Aerothermal, real-gas, TPS | T0 (+ shared `openbmp-thermochem` scaffold) | 0/0/11 | WP-04.1-a |
| 05 | Propulsion high-fidelity | **T1+T2 implemented**; T3 partial; T4/T5 wired-substrate partial | 3/6/9 | finish WP-05.3 (deck consumption) |
| 06 | Coupled MIMO GNC | T0 (+ lane-voting diagnostic improvement) | 0/0/10 | WP-06.1 (after 01) |
| 07 | Trajectory optimization | T0; corrector exists, unwired | 0/0/8 | WP-07.0 |
| 08 | Environment, gravity, frames | T0 baseline (zonal degree-6 cap) | 0/0/7 | WP-08.1 |
| 09 | Sensors, nav, actuators | T0 (specific force still finite-difference) | 0/0/12 | WP-09.1 |
| 10 | Flight SW in the loop (XIL) | **T1–T3 implemented**; T4 partial | 3/2/8 | finish WP-10.4, or WP-10.6 in parallel |
| 11 | Monte Carlo, UQ, validation | **T0–T1 implemented**; T2–T4 partial | 3/5/9 | WP-11.7 |
| 12 | Determinism, real-time, compute | **substantially implemented**; Linux aarch64 bit-stable CI lane present; GPU offload boundary pending | 9/0/10 | WP-12.4-b |
| 14 | Contact, touchdown, landing | WP-14.1 crate + runner force/diagnostic path present; WP-14.2 crate primitives landed; WP-14.3 implemented; WP-14.4 massless oleo/crush gear with per-pad ContactPair footpads implemented | 2/2/10 | WP-14.5 after WP-09.7, or WP-14.7 after terrain substrate |
| 15 | Plume & SRP | not started (05 nozzle state now available) | 0/0/12 | WP-15.1 |
| 16 | Parachute & recovery | T0 baseline (`recovery/` rack) | 0/0/11 | WP-16.1 |
| 17 | Cryogenic fluid management | not started | 0/0/8 | WP-17.1 |
| 18 | Ground segment & countdown | not started | 0/0/8 | WP-18.1 |
| 19 | Day-of-launch winds & commit | not started (WP-19.2 blocked on WP-07.0) | 0/0/6 | WP-19.1 |
| 20 | Telemetry, RF links, network | dictionary export exists; physics not started | 0/1/8 | WP-20.1 |
| 21 | Run-data, regression, visualization | not started | 0/0/8 | WP-21.1 |
| 22 | Acoustics & overpressure | not started | 0/0/7 | WP-22.1 |
| 23 | EPS & avionics emulation | not started | 0/0/6 | WP-23.1 |
| 24 | Postflight reconstruction & sys-ID | not started (`compare-telemetry` seed exists) | 0/0/7 | WP-24.1 |
| 25 | RPOD & docking | not started (vocabulary + multi-body + contact prerequisites exist) | 0/0/8 | WP-25.1 |

**Phase view (`00` §5).** Phase A is the critical path: of its five tracks,
`12` (MC/determinism substrate) and `11` T0–T2 are substantially done and
`05` T1 is done, but `01` (multibody tree) and `08` (tesseral gravity) have
not started — they gate `02`, `06`, and most of Phase B/C. Phase D started
early on its independent track (`10` T1–T3 done). The extension rounds are
design-complete, implementation-untouched except `14` (substrate) and the
`20` dictionary-export head start.

---

## 2. Per-dimension detail

### 01 — Flexible & articulated multibody dynamics

Baseline confirmed: single rigid body in `crates/openbmp-sim/src/kernel.rs`,
separation as mass-property partition, SDOF bending
(`crates/openbmp-vehicle/src/structural.rs`), equivalent-pendulum slosh
(`crates/openbmp-vehicle/src/tank/`). No spatial-vector/ABA code anywhere;
`openbmp-multibody` does not exist. WP-01.1 … WP-01.6: **not started**.

### 02 — Structural dynamics, loads, slosh & POGO

WP-02.1-a … WP-02.3-b: **not started** (still one SDOF mode per axis;
all-fluid pendulum slosh; no modal ingestion, no CLA).
WP-02.4-a/-b (POGO/CSI, structural half): **not started** — but the feed
half this capstone joins (doc 05 WP-05.5-a) landed its substrate today
(`crates/openbmp-feedsystem/src/pogo.rs`, `crates/openbmp-runner/src/pogo.rs`
with `PogoStabilityVerdict`, scenario `propulsion.pogo` block), so the
cross-doc prerequisite is no longer hypothetical. The structural
longitudinal modal model remains the missing half.

### 03 — Aerodynamic database & CFD coupling

Baseline confirmed: `crates/openbmp-aero` T0 (single-panel modified
Newtonian in `hypersonic.rs`, DATCOM-lite buildup in `buildup.rs`,
locked-order `AeroDeck` interpolation in `deck.rs`); no per-entry σ, no
run-matrix, no `openbmp-aerodb`. WP-03.1 … WP-03.9: **not started**.

### 04 — Aerothermal, real-gas & TPS

Baseline confirmed plus one shared scaffold: `openbmp-thermochem` landed
(deck + parser on a (log pc, MR) grid — combustion-deck shape for doc 05;
no equilibrium-air closure). Heating stack unchanged: Sutton-Graves
(`validated-toy`), cold-gas Fay-Riddell scaffold, Tauber-Sutton and
Tannehill typed-reserved (refusing), toy ablators, Park payload pinned.
WP-04.1-a … WP-04.4-b: **not started**.

### 05 — Propulsion high-fidelity

| WP | Status | Evidence / missing |
|---|---|---|
| WP-05.1 pressure-thrust + altitude | **implemented** | `NozzlePerformance` in `crates/openbmp-propulsion/src/motor.rs`; scenario `ambient_pressure_correction = "pressure_thrust"`; `crates/openbmp-runner/tests/pressure_thrust.rs` + fixture; REQ-PROP-001/V-PROP-001 |
| WP-05.2-a nozzle separation clipping | **implemented** | `NozzleSeparationCriterion::{Summerfield,Schmucker}`; scenario opt-in; test in `pressure_thrust.rs`; REQ-PROP-002 |
| WP-05.2-b transient solid ballistics | **implemented** | `TransientChamber` pc(t) ODE + erosive hook in `grain.rs`; `[propulsion.motor.grain] mode = "transient"`; test; REQ-PROP-003 |
| WP-05.3 thermochem deck ingestion | **partial** | crate + parser + scenario block + SHA pin + fixtures + inline grain and reduced feed-network runtime consumption landed; missing: CEARUN/Cantera tolerance tables and full liquid-engine performance coupling |
| WP-05.4-a feed network + transient chamber | **partial** | `openbmp-feedsystem` (graph/network/line/chamber/control/transient) + `crates/openbmp-runner/src/feed_network.rs` + scenario `propulsion.feed_networks` validation; missing: dedicated acceptance tests + tolerance tables |
| WP-05.4-b turbopump map + NPSH | **partial** | `pump.rs` (`Turbopump`, normalized map, design point) + scenario pump blocks + runner pressure/cavitation coupling + synthetic provenance-backed tolerance table; missing: public real-pump calibration |
| WP-05.4-c MOC line transients | **partial** | `line.rs` (`MocLine`) + scenario line blocks + runner pressure perturbation coupling + synthetic provenance-backed Joukowsky tolerance table; missing: richer boundary library, standalone line topology, public benchmark tables |
| WP-05.5-a POGO feed half | **partial** | `pogo.rs` (feedsystem + runner) + `propulsion.pogo` schema + synthetic provenance-backed stability tolerance table; missing: structural modal-data consumption and public Saturn V/Titan case-history evidence |
| WP-05.5-b engine-out & fault library | **partial** | `crates/openbmp-sil/tests/propulsion_stimulus.rs`; `openbmp mc` propulsion-fault campaign path with UQ flags; full doc fault set unverified |

### 06 — Coupled MIMO GNC

Baseline confirmed: per-axis allocator/controllers, 15-state error-state
EKF, GLRT/IMM/voter present. Today's `a903c50` added covariance-trace lane
voting in `estimator_lanes.rs` (+ SR-UKF touch-up) — a diagnostics
improvement, not a backlog WP. WP-06.1 … WP-06.4-d: **not started**
(dense-B allocation, n×n DARE, gain scheduling, load-relief, tight nav,
engine-out reconfiguration all absent).

### 07 — Trajectory optimization & mission design

`corrector.rs` (Gauss-Newton/LM shooting) and the locked
`TerminalCondition` vocabulary exist and the lock **holds** (no
range/aimpoint variants; `RendezvousState` present). There is **no
trajopt CLI subcommand** — the corrector remains unwired.
WP-07.0 … WP-07.6: **not started** (WP-07.0 is the gate).

### 08 — Environment, gravity & frames

Baseline confirmed: `Egm2008ZonalGravity` is zonal-only, hard-capped at
degree 6 (`gravity.rs`); IAU 1976/1980 equinox frames (no CIO path);
NRLMSISE-00/HWM14 means only (no perturbed-atmosphere decorator); WMM2025
(no IGRF-14, no gradient); SPK DAF parser present; no tides/SRP/Battin/
relativity. WP-08.1 … WP-08.7: **not started**.

### 09 — Sensors, navigation & actuators

Baseline confirmed: specific-force truth is still finite-difference
(`crates/openbmp-runner/src/fc_bridge.rs`, velocity delta minus gravity);
GNSS is a position oracle; actuators first-order.
WP-09.1 … WP-09.12: **not started**.

### 10 — Flight-software-in-the-loop (XIL)

| WP | Status | Evidence / missing |
|---|---|---|
| WP-10.1 SIL over transport | **implemented** | `Transport` + in-process/stream/child-stdio backends, `LockstepSimMaster`, SHA-256 determinism gate, ZOH tolerance table, 12 tests in `crates/openbmp-runner/tests/fc_transport.rs` |
| WP-10.2 fault library + XIL ports | **implemented** | `fault.rs` (13 scalar + 5 bus transforms), `MaPort`/`EesPort`, lifecycle FSM, `[fc.transport_faults]`, empty-schedule byte-identity, `crates/openbmp-sil/src/xil.rs` evidence adapters |
| WP-10.3 soft-real-time bench | **implemented** | `openbmp-rt` pacer (FreeRun/RealTime/Paced, jitter histogram p50/p99/p99.9, overrun count), runner `rt.rs`, `[realtime]` schema, byte-identity across modes (`tests/realtime.rs`) |
| WP-10.4 FMI 3.0 importer | **partial** | typed instance access, modelDescription parsing, single+multi masters, 5 pinned Reference FMUs + FMPy cross-check scripts; missing: clocked event scheduling, predictor/corrector co-sim loop |
| WP-10.5 FMI 3.0 exporter | **partial** | `export.rs` + `export_abi.rs` (C ABI, XSD validation, point-mass FMU + trace tolerance table); missing: full plant mapping to value references |
| WP-10.6 AFTS containment monitor | **not started** | no `openbmp-afts`, no IIP propagator, no rule table |
| WP-10.7 PIL on emulated ISA | **not started** | no factored `fc_step` entry, no Renode coupling |
| WP-10.8 cFS in the loop | **not started** | no cFS wiring |

### 11 — Monte Carlo, UQ & validation

| WP | Status | Evidence / missing |
|---|---|---|
| WP-11.0 MC substrate | **implemented** | `openbmp-mc` (Welford, Clopper-Pearson, samples table + convergence trace), `DeterministicRng::for_mc_sample` domain |
| WP-11.1 DoE + sizing | **implemented** | LHS, native Sobol (pinned Joe-Kuo direction numbers under `data/sobol/` + provenance), Owen scrambling, Iman-Conover, Wilks sizing, convergence gate + R-hat, `openbmp mc` subcommands + `tests/mc_cli.rs` |
| WP-11.2 credibility + error budget | **partial** | `openbmp-uq` complete (correlated error budget, aleatory/epistemic classes, 8-factor credibility record, min-is-binding, floor flags wired into `mc summarize`/footprint/fault campaigns); missing: auto-emission by future drivers |
| WP-11.3 nested aleatory/epistemic | **partial** | probability-box + variance split, nested footprint execution, `mc nested-summarize`; missing: generalized nested orchestration beyond landing-footprint |
| WP-11.4 rare events | **implemented** | subset simulation (Au-Beck) + cross-entropy IS on sealed synthetic limit states; compile-fail tripwire `crates/openbmp-testkit/tests/limit_state_no_aimpoint_compile_fail.rs` + UI test |
| WP-11.5 MMS + order verification | **partial** | manufactured ODE + observed-order/Richardson/GCI (`openbmp verify-order`, testkit verification module); campaign-scale integration pending |
| WP-11.6 BET reconstruction | **partial** | RTS smoother + batch Gauss-Newton + NEES/NIS in `crates/openbmp-testkit/src/reconstruction.rs`; `openbmp reconstruct` CLI; pseudo-flight campaign + LOCAL workflow documentation pending |
| WP-11.7 cross-discipline UQ wiring | **not started** | upstream per-entry margins (03/04/05 decks) not yet flowing |
| WP-11.8 campaign determinism/real-time | **partial** | delivered through doc 12's reducers/checkpointing; ensemble byte-diff gate extension pending |

### 12 — Determinism, real-time & compute

| WP | Status | Evidence / missing |
|---|---|---|
| WP-12.0-a check-provenance CI | **implemented** | CI job runs `openbmp check-provenance` over `data/` + `scenarios/` |
| WP-12.0-b FP guard portable | **implemented** | `crates/openbmp-core/src/fp.rs` (x86_64 MXCSR + aarch64 FPCR), FMA ban in `.cargo/config.toml` |
| WP-12.0-c `for_mc_sample` | **implemented** | domain-tagged campaign stream in `rng.rs` |
| WP-12.1 order-independent reducer | **implemented** | Welford merge tree in `openbmp-mc`; serial fold == merge tree tests |
| WP-12.2-a parallel fan-out | **implemented** | worker-count-invariant campaigns (tests at 1/2/4/8 workers) |
| WP-12.2-b checkpoint/resume | **implemented** | `FileCheckpointStore`, resume byte-identity, footprint `--checkpoint-json` |
| WP-12.3-a frame pacer | **implemented** | `openbmp-rt` (shared with doc 10) |
| WP-12.3-b aarch64 CI lane | **implemented** | native `ubuntu-24.04-arm` `determinism-gate (aarch64)` downloads the same-run x86_64 reference artifact and byte-diffs the fixed-step canonical scenario outputs; traced by REQ-DET-004/V-DET-004 |
| WP-12.4-a dense output | **implemented** | `advance_with_dense_output` for Dopri54/853, state-stable opt-in, off the bit-stable path |
| WP-12.4-b GPU offload boundary | **not started** | no ingested-deck GPU pathway |

### 14 — Contact dynamics, touchdown & landing

WP-14.1: **partial** — `openbmp-contact` crate landed (`e4a1a4d`:
half-space geometry, Kelvin-Voigt/Hertz/Hunt-Crossley normal laws,
regularized Coulomb, fixed sub-step stability bound, energy audit;
REQ-CONTACT-001). Scenario `[contact]` schema (`ContactConfig`) and runner
force-adapter wiring are present (`crates/openbmp-runner/src/contact.rs` plus
point-mass/rigid-body/vehicle edits), with contact diagnostics telemetry,
`RunOutcome.contact` endpoint classification, run-level energy audit, and
substep-driven kernel step sizing, plus point-mass golden-stability proof.
Missing for full acceptance: gear-leg assemblies.
WP-14.2: **partial** — `openbmp-contact` now exposes one-sided scalar
stops, two-sided backlash gaps, and monotone latch primitives with validation,
unilateral force clamping, backlash dead-zone width evidence, Kelvin-Voigt
closed-form stop restitution evidence, and engage-once latch property tests
(`REQ-CONTACT-003`). Scenario/joint fixtures remain future work with the
articulated mechanism wiring.
WP-14.3: **implemented** — `openbmp-contact` now exposes anchored stick/slip
friction with static cone breakaway, kinetic sliding, Karnopp restick window,
deterministic anchor state updates, tangential anchor elastic-energy reporting,
incline stiction/sliding closed-form helpers, a drive-spring stick-slip
oscillator fixture matched against an independent ideal Karnopp reference, a
consecutive-hold rest detector, and Housner rocking-block threshold and period
anchors (`REQ-CONTACT-004`). Schema-v3 `[contact]` can now select
`friction_law = "anchored_stiction"` and the runner contact adapters route it
through time-gated anchor state; contact diagnostics now carry tangential
speed/sticking state and runner rest classification uses a kinetic-energy floor
plus consecutive sticking hold (`REQ-CONTACT-005`).
WP-14.4: **implemented** — `openbmp-vehicle` now exposes validated
`LandingGearLeg`, `OleoStage`, and `CrushCore` primitives with oleo
polytropic-force/energy tests and irreversible crush-core plateau-stroke plus
monotonicity tests (`REQ-CONTACT-006`). Schema-v3 rigid-body scenarios can
now declare `[vehicle.landing_gear]`; the runner wires the rack as force and
moment adapters, emits `force.landing_gear.{x,y,z}_n` plus per-leg load,
stroke, gap, crush, and contact telemetry, records a pinned synthetic gear data
SHA, and verifies the checked-in 3-D four-leg drop fixture
(`REQ-CONTACT-007`). `RunOutcome.landing_gear` now classifies the synthetic
drop as `Rest` under a consecutive quiet-speed hold and carries a deterministic
energy audit that closes to <1% on the fixture; the runner can recover body-x
section loads from final per-leg samples and the fixture cross-checks mid-body
shear/bending against independently read final front-leg telemetry. Each
runtime leg now owns a per-pad `ContactPair` in external-normal mode, so the
contact substrate owns pad geometry, gap/rate evaluation, opt-in regularized
friction, tangential-speed evidence, and friction-force contribution while the
oleo/crush strut supplies the normal load.
WP-14.5 … WP-14.10: **not started**.

### 15 — Plume environments & SRP

**Not started** (no plume code). Note: its primary dependency — doc 05
nozzle/chamber state — now exists, so WP-15.1 is genuinely unblocked.

### 16 — Parachute, decelerator & recovery

T0 baseline confirmed (`crates/openbmp-vehicle/src/recovery/` drag-swap
models; `scenarios/parachute-recovery`). WP-16.1 … WP-16.11: **not
started**.

### 17 — Cryogenic fluid management

**Not started.** No property layer, no tank energy balance. (The
`openbmp-feedsystem` crate now exists as the design's default home for the
cryo module.)

### 18 — Ground segment, countdown & launch release

**Not started.** No `openbmp-ground`, no LCC engine, no GSE/T-0 models.

### 19 — Day-of-launch winds & commit operations

**Not started.** No measured-wind ingestion or redesign driver. WP-19.2
remains blocked on WP-07.0 (wired corrector); WP-19.1 is unblocked.

### 20 — Telemetry, RF links & ground network

WP-20.8 dictionary export: **partial** — a channel-dictionary export with
JSON and XTCE-shaped formats already ships (`openbmp dict`,
`crates/openbmp-cli/src/commands/dict.rs`); the CCSDS-shaped frame stream
and round-trip tests from the design are absent.
WP-20.1 … WP-20.7: **not started** (no `openbmp-comm`, no link physics).

### 21 — Run-data, regression & visualization

**Not started.** `openbmp mc` emits samples tables/convergence traces (doc
11 surface), but no run-record index, verdict rules, trends, booklets, or
CZML/glTF export exist.

### 22 — Acoustics, vibroacoustics & overpressure

**Not started.** No spectral types or DSM/IOP code.

### 23 — Electrical power & avionics emulation

**Not started.** No `openbmp-eps`, no battery/bus-profile/string-kill code.
(The bridge fault library from doc 10 is the substrate WP-23.5 will
decorate.)

### 24 — Postflight reconstruction & model correlation

**Not started.** The pre-existing `openbmp compare-telemetry` LOCAL
comparison is a seed for WP-24.1, and doc 11's RTS/GN/NEES-NIS substrate
landed — but no parameter registry, output-error estimator, identifiability
analysis, or governance pipeline exists.

### 25 — Rendezvous, proximity operations & docking

**Not started.** Prerequisites in place: multi-body propagation,
`RendezvousState` in the locked vocabulary, and the `openbmp-contact`
substrate. No LVLH/CW/TH machinery, rel-nav, prox-ops guidance, safety
verifier, or capture logic.

---

## 3. Cross-cutting locks & gates status

- **Terminal-condition vocabulary lock holds:** `TerminalCondition` carries
  orbital/flight-condition variants only; no range or aimpoint field
  exists.
- **New tripwire added with capability** (per `00` invariant 7): sealed
  `LimitState` + `limit_state_no_aimpoint_compile_fail.rs` UI test landed
  alongside the rare-event machinery.
- **Determinism toolchain hardened:** FP-environment guard (x86_64 +
  aarch64 code paths), FMA contraction ban, provenance check in CI; the
  aarch64 CI determinism lane is the one outstanding piece (WP-12.3-b).
- **Traceability:** 164 requirement ids in `requirements.toml`, including
  the new REQ-PROP-001…019, REQ-MC, and REQ-CONTACT families.
- **Byte-stable-by-default pattern observed** in everything that landed:
  pressure-thrust, transient grain, transports, realtime, contact, and
  fault schedules are all scenario-gated opt-ins.

---

## 4. Recommended next moves (from this snapshot)

1. **Close the open partials before opening new fronts:** WP-05.3 runtime
   deck consumption; WP-14.1 gear-leg closure; WP-12.4-b GPU boundary.
2. **Start the two unstarted Phase-A gates:** WP-01.1 (spatial-vector tree)
   and WP-08.1 (tesseral gravity) — they block most of Phase B/C (02, 06,
   07 closed-loop quality).
3. **WP-07.0 remains the cheapest unblocked capability win** (wire the
   existing corrector to a CLI driver) and unblocks WP-19.2 later.
4. WP-15.1 (plume state) is newly unblocked by 05's chamber/nozzle state.
