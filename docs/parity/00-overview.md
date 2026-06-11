# OpenBMP Parity Program — Overview & Design Constitution

**Status:** `experimental` (design intent; no code shipped by this document).
**Audience:** the engineer or LLM agent who will implement the work packages.
**Scope:** a forward-only, multi-quarter program to bring OpenBMP to **capability
parity** with the launch-vehicle / GNC simulators used at SpaceX, Rocket Lab,
NASA, and JPL — *within OpenBMP's deliberate posture* (open, simulation-only,
deterministic, solver-consuming, hardware-boundary-abstract).

This is the index and the constitution. Each numbered companion document
(`01`–`12` and the extension rounds `14`–`25`) carries the deep technical
design for one capability dimension and a machine-readable work-package
backlog. Document `13` is the execution playbook — the rules, gates, and
conventions the implementing agent must obey on every PR.

> Read this document, then `13-agent-execution-playbook.md`, before opening any
> design doc. The per-dimension docs assume the parity definition, the
> invariants, and the backlog schema defined here.

---

## 1. What "parity" means — and what it cannot mean

A capability audit (recorded separately) compared OpenBMP against documented
practice at the reference organizations across eleven dimensions. The result:
one dimension *approaching* parity (determinism/provenance), seven *partial*,
three *major-gap* (aerodynamics, propulsion, structural dynamics). **Zero at
full parity.** This program closes that, but only against an honestly-scoped
target. We separate two kinds of parity:

| | **Capability / method / architecture parity** | **Validation parity** |
|---|---|---|
| **Definition** | Same formulations, same algorithm classes, same V&V methodology, same fidelity *architecture* as the reference tools | The model's numbers are trusted to fly a real vehicle: flight heritage, proprietary wind-tunnel/CFD/test data, qualification, certification |
| **Reachable in an open repo?** | **Yes** — this is the program's target | **No** — structurally impossible without a vehicle program |
| **Evidence** | The existing five-layer V&V ladder (`docs/verification.md`): MMS code-verification → model-verification vs CEA/correlations → code-to-code vs open solvers → public flight benchmarks → UQ reporting; plus NASA-STD-7009B credibility accounting and open-telemetry cross-validation | Wind-tunnel campaigns, GVT/modal surveys, hot-fire test stands, flight reconstruction against the vehicle's own telemetry, range-safety sign-off |

**Every design document states its own parity ceiling explicitly** — the
specific thing that cannot be matched without proprietary data or flight
heritage, and the credible *open substitute*. This is not hedging; it is the
honest engineering boundary, and stating it is a hard requirement (see the
don't-overstate rule in `13`). The deliverable of this program is a simulator
whose *methods and architecture* are indistinguishable from production practice
and whose *credibility* is rigorously accounted — not a claim of flight
readiness, which `docs/standards-posture.md` and `docs/safety-boundaries.md`
permanently disclaim.

### 1.1 The solver-consumer posture (do not violate)

OpenBMP's existing V&V ladder is explicit: for the physics-heavy disciplines
(CFD, DSMC, radiation, material response, FEM), OpenBMP **consumes external
solver results as provenance-pinned data packages; it does not ship the
solvers.** The parity target for those dimensions is therefore *not* "write a
RANS/FEM solver solo." It is the production-grade layer that makes externally
produced high-fidelity data trustworthy and usable end-to-end:

1. an **in-repo low/mid-fidelity tier** (engineering correlations + inviscid /
   reduced-order methods) sufficient for self-contained, deterministic runs and
   code-verification;
2. a **run-matrix orchestration + ingestion + provenance** pipeline for external
   high-fidelity data (Cart3D/SU2/OpenFOAM/SPARTA/CalculiX/CEA/FIAT-class);
3. **uncertainty-propagating database objects** carrying per-entry bias/random
   margins into Monte Carlo and loads; and
4. **code-to-code + public-benchmark validation** anchoring the in-repo tiers.

Disciplines that are *pure simulation with no proprietary-data dependency* — the
multibody/flexible EOM, the GNC stack, trajectory optimization, the environment
models, the propulsion *transient* physics, the Monte Carlo/UQ machinery, the
XIL infrastructure — can reach high in-repo fidelity directly, and those docs
target that.

---

## 2. The gap ledger

Each row maps an audited dimension to its design document and its parity target.
"Target verdict" is the achievable end-state under §1 (capability parity within
posture), not validation parity.

| # | Dimension | Audit verdict | Design doc | Target (capability) |
|---|---|---|---|---|
| 01 | Flexible & articulated multibody EOM | partial (EOM) / major-gap (struct) | `01-flexible-multibody-dynamics.md` | **at-parity** (method): spatial-vector tree (ABA) + FFRF flex |
| 02 | Structural dynamics, loads, slosh, POGO | major-gap | `02-structural-dynamics-loads-slosh-pogo.md` | **approaching**: multi-mode + ingested modal model + CLA events + POGO |
| 03 | Aerodynamics & CFD database | major-gap | `03-aerodynamics-database-and-cfd-coupling.md` | **approaching** (method): inviscid tier + DB/UQ/run-matrix + verified ingestion |
| 04 | Aerothermal, real-gas, TPS | major-gap (within aero audit) | `04-aerothermal-realgas-and-tps.md` | **approaching** (method): correlations + equilibrium-air + ingested material response |
| 05 | Propulsion high-fidelity | major-gap | `05-propulsion-high-fidelity.md` | **at-parity** (method): transient engine + thermochem + feed/turbopump + POGO |
| 06 | Coupled MIMO GNC, allocation, FDIR | partial | `06-gnc-coupled-mimo-and-control.md` | **at-parity** (method): full-B allocation, coupled LQR/LQG, load-relief |
| 07 | Trajectory optimization & mission design | partial | `07-trajectory-optimization-and-mission-design.md` | **at-parity** (method): collocation/pseudospectral + LCvx/SCvx, wired closed-loop |
| 08 | Gravity, frames, atmosphere, space env | partial | `08-environment-gravity-and-frames.md` | **at-parity**: full tesseral gravity, IAU2006/2000A, GRAM-style dispersion |
| 09 | Sensors, navigation, actuators | partial | `09-sensors-navigation-and-actuators.md` | **approaching**: force-based IMU, pseudorange GNSS, actuator dynamics, RAIM |
| 10 | Flight-software-in-the-loop (XIL) | partial | `10-flight-software-in-the-loop-xil.md` | **approaching** (architecture): SIL-over-transport, PIL on emulator, FMI3, cFS |
| 11 | Monte Carlo, UQ, V&V, validation | partial | `11-monte-carlo-uq-and-validation.md` | **approaching**: LHS/Sobol/Wilks, subset-sim, 7009B, MMS, reconstruction |
| 12 | Determinism, real-time, compute | approaching | `12-determinism-realtime-and-compute.md` | **at-parity**: deterministic parallel MC, soft-real-time pacing, checkpointing |
| 14 | Contact dynamics, touchdown & landing | absent (extension) | `14-contact-dynamics-touchdown-and-landing.md` | **approaching** (method): compliant contact + CFE + gear + sea-state deck |
| 15 | Plume environments & supersonic retropropulsion | absent (deferred by 03/04/05) | `15-plume-environments-and-supersonic-retropropulsion.md` | **approaching** (method): live plume state + power-on base + SRP decks + impingement |
| 16 | Parachute, decelerator & recovery | partial (extension) | `16-parachute-decelerator-and-recovery-systems.md` | **approaching** (method): inflation/deployment/clusters + two-body + water impact |
| 17 | Cryogenic fluid management & tank thermo | absent (deferred by 05) | `17-cryogenic-fluid-management.md` | **approaching** (method): two-zone/stratified tank thermo + pressurant + chilldown + loading |
| 18 | Ground segment, countdown & launch release | partial (mission FSM only) | `18-ground-segment-countdown-and-launch-operations.md` | **at-parity** (architecture): sequencer + LCC engine + GSE/T-0 + clearance MC + rehearsals |
| 19 | Day-of-launch winds & commit operations | absent (deferred by 11) | `19-day-of-launch-winds-and-commit-operations.md` | **approaching** (method): DOLILU-class ingest → redesign → screen → commit |
| 20 | Telemetry, RF links & ground network | absent | `20-telemetry-rf-links-and-ground-network.md` | **approaching** (method): link budget → FER → channel effects; blackout; tracking observables |
| 21 | Run-data, regression & visualization | absent | `21-run-data-regression-and-visualization.md` | **at-parity** (architecture): run records + verdicts + trends + booklets + CZML/glTF export |
| 22 | Acoustics, vibroacoustics & overpressure | absent (deferred by 02/05/15) | `22-acoustics-vibroacoustics-and-overpressure.md` | **approaching** (method): SP-8072 DSM + IOP + ascent FPL + P95/50 environments |
| 23 | Electrical power & avionics emulation | absent | `23-electrical-power-and-avionics-emulation.md` | **approaching** (architecture): EPS network + coupled draws + bus profiles + N-string bench |
| 24 | Postflight reconstruction & model correlation | partial (BET in 11) | `24-postflight-reconstruction-and-model-correlation.md` | **approaching** (method): output-error sys-ID + CRLB + governed deck increments |
| 25 | Rendezvous, proximity operations & docking | absent (promised by 14) | `25-rendezvous-proximity-operations-and-docking.md` | **approaching** (method): CW/TH + prox-ops GNC + passive safety + capture via contact |

Rows `01`–`12` are the original eleven-dimension audit (aerodynamics and
aerothermal share one audited dimension). Rows `14`–`16` are the first
extension round: consumer applications the original audit folded into other
dimensions. Rows `17`–`25` are the second extension round (June 2026): a
cross-organization survey of publicly documented SpaceX / Rocket Lab /
NASA-JPL simulation practice identified nine operations, data-infrastructure,
and subsystem dimensions the original audit did not enumerate — each either
explicitly deferred by an earlier design doc or absent from the workspace.
Deliberate non-goals recorded during that survey: quantitative range-safety
risk tooling (casualty-expectation analysis) stays out — the AFTS forward
containment monitor and the provenance-locked footprint remain the entire
range-safety surface (invariant 7); constellation/fleet operations and
mission-control operator-training products are out of scope for a
launch-vehicle simulator; bus-protocol conformance and waveform-level RF stay
behind `docs/standards-posture.md`.

Per-work-package implementation status against this ledger is tracked in
`progress.md` (snapshot-dated; updated in the same PR as any WP merge).

---

## 3. Non-negotiable invariants

These are the properties that make OpenBMP *OpenBMP*. Every work package
preserves all of them. Each is enforced by a CI gate or tripwire test the agent
must keep green (see `13` for the exact commands). **A PR that weakens any
invariant is rejected regardless of the capability it adds.**

1. **Byte-determinism.** Same inputs → byte-identical Parquet/CSV on the
   reference profile. New models advance state by locked-operand-order weighted
   sums, never `f64::mul_add`, never wall-clock, never system RNG, never
   unordered iteration. Determinism is verified by the CI byte-diff gate on the
   canonical scenario set. New randomness derives from
   `openbmp-core::DeterministicRng` with a domain-separated `(seed, step, id,
   component)` tuple.

2. **Byte-stable-by-default.** Every new model/capability is **off by default**
   and gated by an explicit scenario config block, so all existing scenarios and
   golden archives remain byte-identical until a scenario opts in. This is the
   established pattern (e.g. `[vehicle.bending]`); follow it without exception.

3. **FC hardware-portability lock.** `openbmp-fc` depends **only** on
   hardware-portable crates. It must never gain a dependency on `openbmp-sim`,
   `openbmp-cli`, `openbmp-runner`, `openbmp-scenario`, `openbmp-telemetry`,
   `openbmp-bridge`, or `openbmp-aerothermal`. Enforced by
   `crates/openbmp-testkit/tests/fc_dependency_tripwire.rs` (catches renamed,
   workspace-aliased, target-conditional, and `dep:`-feature edges). New GNC code
   that needs simulator data receives it through the existing FC-bridge /
   sensor-actuator boundary — never by importing the simulator.

4. **Lockstep-clock & no-hot-path-allocation contract.** `openbmp-fc` and
   `openbmp-runner/src/fc_bridge.rs` read time only through the injected `Clock`
   trait; `std::time::Instant::now`/`SystemTime::now` are banned. Scheduler
   dispatch uses cached registration-time order; per-tick paths do not allocate
   or clone the binding/topic tables. Enforced by
   `crates/openbmp-testkit/src/fc_lints.rs`.

5. **Four-pillar provenance.** No inline TOML scenario/data fixtures in `*.rs`;
   real benchmark constants live only in their declared source-of-truth file
   (allow-listed); test TOML lives under `tests/fixtures|expected/`, `data/`, or
   `scenarios/`; every `data/` and `scenarios/` TOML has a sibling
   `provenance.md` that lists it, with a SHA-256 content pin verified
   fail-closed at scenario load. Enforced by
   `crates/openbmp-testkit/tests/inline_data_tripwire.rs` and `openbmp
   check-provenance`. **Authoring corollary:** do not write the literal WGS84
   numerals for GM⊕, J₂, or R⊕ anywhere outside their allow-listed
   source-of-truth files — refer to them symbolically. This applies to these
   design docs too.

6. **Synthetic / public-data-only.** All checked-in data is synthetic,
   textbook-derived, or openly published with provenance and license status.
   Proprietary, opaque, or fielded-vehicle parameter sets stay out of the
   repository; user-supplied comparison data stays local and is documented as a
   workflow, never committed (see `docs/external-telemetry-validation.md`).

7. **Forward-only mechanical locks.** OpenBMP simulates *forward* (state →
   forward dynamics → telemetry) and optimizes only to **flight/orbital
   conditions** and **powered-descent-to-state** (a site *radius/state*, not a
   ground aimpoint). Ground-aimpoint fire-control is refused. Footprint/ballistic
   states require provenance seeds and are not constructible downstream as bare
   literals (`crates/openbmp-testkit/tests/ballistic_state_compile_fail.rs`).
   **As capability grows — especially in trajectory optimization and GNC — the
   locks tighten, never loosen** (the terminal-condition vocabulary lock; new
   compile-fail/tripwire tests for any new guidance or boundary-condition
   surface).

8. **Validation labels.** Every new model, dataset, scenario, and validation
   case declares exactly one of `experimental` → `checked` → `validated-toy` →
   `research` (`docs/verification.md`), justified by the required evidence, and
   carries a tolerance-table TOML where it makes a numeric claim. No artifact
   ever claims `flight-qualified`/`certified`/`operational`/`mission-ready`.

9. **Workspace lints & docs.** `missing_docs`, `unsafe_code`,
   `unused_must_use`, `float_cmp` stay `deny`; `unwrap`/`expect`/`panic` stay
   `warn` outside tests. Public items are documented. Failure modes are
   fail-closed (`Result`, not `panic`, on any path reachable from a scenario).

10. **Requirements traceability.** New capability adds machine-readable entries
    to `requirements.toml` with verification evidence; `scripts/
    check_requirements_traceability.py` rejects orphans and stale anchors.

---

## 4. Architecture & crate map

OpenBMP is a layered workspace; lower layers never depend on higher ones, and
the controller/physics layers carry no simulator dependency. The program adds
the following crates (each finalized in its design doc). Layer assignments
preserve the acyclic dependency rule.

```text
EXISTING (touched):
  openbmp-core        L0  + extend DeterministicRng stream domains (MC)
  openbmp-state       L1  + multibody generalized-coordinate state
  openbmp-models      L1  + new force/moment/flex/thermo trait surfaces
  openbmp-physics     L2  + tesseral gravity, CIO frames, tides, perturbed atmosphere
  openbmp-vehicle     L2  + FFRF flex body composition, multi-mode structural rack
  openbmp-aero        L2  + inviscid panel tier, area-rule wave drag, aero-DB + UQ
  openbmp-aerothermal L2  + Tauber-Sutton, equilibrium-air edge state, distributed deck
  openbmp-propulsion  L2  + pressure-thrust, transient ballistics, thermochem deck
  openbmp-sensors     L3  + force-based IMU, pseudorange GNSS, actuator dynamics
  openbmp-fc          L4  + full-B allocation, coupled LQR/LQG, load-relief, tight nav  (portability lock!)
  openbmp-trajopt     —   + multiple-shooting, collocation, pseudospectral, LCvx/SCvx, offline driver
  openbmp-bridge      L7  + Transport trait, fault-injection ports, lockstep master
  openbmp-fmi         —   + FMI 3.0 co-simulation importer master
  openbmp-runner      L7  + multibody/flex/CFD-deck wiring; MC campaign hooks

NEW (proposed; final placement per doc):
  openbmp-multibody   L2  spatial-vector tree (ABA/RNEA/CRBA) + FFRF flex          (doc 01/02)
  openbmp-thermochem  L2  equilibrium-air curve fits + CEA/Cantera deck ingestion  (doc 04/05)
  openbmp-feedsystem  L2  GFSSP-style fluid-network + turbopump + MOC transients   (doc 05)
  openbmp-uq          L1  first-class error-budget / 7009B credibility (promote uq.rs) (doc 11)
  openbmp-mc          L7  Monte Carlo campaign orchestrator (LHS/Sobol/Wilks/subset-sim) (doc 11)
  openbmp-aerodb      L6  aero-database object + run-matrix orchestrator + ingestion (doc 03)
  openbmp-rt          L7  soft-real-time frame pacing + jitter accounting (HIL bench)  (doc 10/12)
  openbmp-structdyn   L2  CB reduction + modal transient (Newmark/gen-α) + FEM + LTM/OTM + POGO (doc 02)
  openbmp-modelio     L1  OP2/OP4/CSV modal-model + OTM ingestion → provenance-pinned ModalAsset (doc 01/02)
  openbmp-convex      L2  shared deterministic conic-solve settings + CSC builders (FC-portable) (doc 07)
  openbmp-harmonics   L1  singularity-free spherical-harmonic synthesis kernel (Pines/Gottlieb)  (doc 08)
  openbmp-visionnav   L3  pinhole + PnP + DEM-relative measurement (behind a `vision` feature)   (doc 09)
  openbmp-afts        L2  forward IIP geo-containment MONITOR (no_std-capable; never targeting)   (doc 10)
  openbmp-contact     L2  unilateral contact: compliant normal/friction + stops/latches + gear/deck (doc 14)
  openbmp-plume       L2  live plume state + power-on base/SRP/impingement engineering tiers       (doc 15)
  openbmp-comm        L3  link geometry/budget/blackout + station network + tracking observables   (doc 20)
  openbmp-acoustics   L2  SP-8072 DSM liftoff acoustics + IOP + vibroacoustic environments         (doc 22)
  openbmp-eps         L2  DC power network + battery packs + staged-power events                   (doc 23)
  openbmp-sysid       L6  output-error estimation + identifiability + reconciliation governance    (doc 24)
  openbmp-ground      L7  countdown sequencer + LCC engine + GSE/T-0 + clearance screening         (doc 18)
  openbmp-rundata     L7  run records + verdict rules + trends + report booklets + 3D export       (doc 21)
```

> This map is the authoritative crate inventory; the entries below
> `openbmp-rt` are additional narrowly-scoped crates proposed by individual
> design docs. Some are marked *optional/conditional* in their doc (e.g.
> `openbmp-convex`, `openbmp-visionnav`) and may instead land as a private module
> if the boundary review prefers it. Of the extension-round docs: `17`'s cryo
> module defaults into `openbmp-feedsystem` and `19`'s day-of-launch driver
> into `openbmp-trajopt` (own crates only if review prefers); docs `16` and
> `25` propose no new crates.

> The agent proposes the crate boundary in the WP that introduces it and gets it
> reviewed before building; do not silently fold a new concern into an existing
> crate if it would create an up-layer dependency or break the FC portability
> lock.

---

## 5. Dependency DAG & phased roadmap

Work packages have real ordering constraints. The program runs as four phases
with **parallelizable tracks inside each phase**. Tier numbers (`Tn`) below refer
to the fidelity ladders defined in each doc; early tiers ship verifiable value
without waiting for the whole dimension.

```
Phase A — Foundation (unblocks everything; mostly parallel)
  ├─ 12 T0–T2  deterministic parallel MC substrate + real-time pacing
  ├─ 08 T1–T3  tesseral gravity + third-body/SRP + CIO frames        (improves all propagation)
  ├─ 01 T1–T2  spatial-vector rigid tree + separation-as-joint-release (multibody substrate)
  ├─ 11 T0–T2  MC crate + DoE/Wilks + 7009B credibility rewire (uq.rs → openbmp-uq)
  └─ 05 T1     propulsion pressure-thrust / altitude (cheap, high-value, isolated)

Phase B — Physics fidelity (depends on A's substrate; parallel tracks)
  ├─ 02 T1–T3  multi-mode structural + ingested modal model + CLA   (needs 01 FFRF)
  ├─ 03 T1–T4  inviscid aero tier + DATCOM-level buildup + aero-DB/UQ + Euler ingestion
  ├─ 04 T1–T3  radiative heating + equilibrium-air + distributed deck (pairs with 03)
  ├─ 05 T2–T4  transient ballistics + thermochem + feed/turbopump
  └─ 09 T1–T2  force-based IMU + actuator dynamics + pseudorange GNSS (needs 05 forces)

Phase C — GNC & optimization (depends on coupled plant from A/B)
  ├─ 06 T1–T4  full-B allocation → coupled LQR/LQG → load-relief → tight nav/FDIR
  └─ 07 T0–T6  wire corrector → multiple-shooting → collocation/pseudospectral → LCvx/SCvx → closed-loop

Phase D — Integration & credibility (depends on mature models + FC boundary)
  ├─ 10 T1–T6  SIL-over-transport → fault injection → soft-real-time → FMI3 → PIL(emulator) → cFS
  ├─ 11 T3–T5  subset-simulation rare-event + MMS/reconstruction + campaign-scale
  ├─ 05 T5     POGO coupling + fault library (needs 02 longitudinal mode + 05 feed)
  └─ 02 T4     POGO/CSI margin analysis (joins 05 T5)

Extension rounds (placements; tiers shippable independently)
  ├─ into B:   15 T1–T3 (needs 05 nozzle state) · 14 T1–T2 (T3+ needs 01)
  │            16 T1–T3 (T4 needs 01) · 17 T1–T3 (pairs with 05) · 22 T1–T2 (needs 15)
  ├─ into D:   23 T1–T4 (with 10) · 20 T1–T4 (with 10/11) · 25 T1–T6 (needs 07/09/14)
  └─ Phase E — Operations & data (consumes A–D substrates)
               18 T1–T4 (with 17/23/20) · 19 T1–T5 (needs 07 + 02/03)
               21 T1–T5 (with 11/12) · 24 T1–T5 (needs 11/21)
```

**Critical-path notes.** (a) `01` (multibody substrate) gates `02` flex and the
coupled plant `06` needs; start it early. (b) `08` tesseral gravity and `12` MC
substrate are pure wins that improve everything — Phase A. (c) POGO (`05 T5` +
`02 T4`) is a true capstone: it closes a propulsion-structure feedback loop and
must come last. (d) `10` XIL is largely independent of physics fidelity and can
progress on its own track once the FC boundary is stable.

---

## 7. How to use this series

1. **Read order:** this doc → `13-agent-execution-playbook.md` → the target
   design doc.
2. **Each design doc follows the same template** (defined in `13` §4): parity
   target & ceiling; current state in source; target architecture (traits,
   structures, math, fidelity tiers); invariant-preservation notes; V&V plan;
   dual-use note; dependencies; open-source leverage; and a **machine-readable
   work-package backlog** (`WP-NN.t`) the agent executes one PR at a time.
3. **One work package ≈ one PR**, green on the full gate set, shipping at a
   declared fidelity tier and earning a declared validation label.
4. **When a tier needs external high-fidelity data** (CFD/FE/DSMC/material
   response), the agent builds ingestion + provenance + UQ + coupling + an
   in-repo reduced tier + a code-to-code validation case — never a from-scratch
   production solver (§1.1).

---

## 8. Risk register (honest)

| Risk | Where | Disposition |
|---|---|---|
| Slosh-coupled orbit closure may not converge | 01/02/06 | Already diagnosed (post-separation upper-stage tumble, `docs/launch-vehicle-fidelity-frontier.md`); treat as a *control co-design* WP with explicitly uncertain convergence, not a config knob. Ship the multibody/flex substrate regardless of whether this specific closure lands. |
| POGO stability numbers need proprietary pump cavitation/compliance data | 05/02 | Build the Rubin/Oppenheim loop and validate the *method* (MMS + published stability trends); never present a specific vehicle's margin. |
| Nonequilibrium chemistry & material response are research-grade and data-hungry | 04 | Cap at `research` label with code-to-code (FIAT/PATO/NEQAIR) + public-flight (Stardust/Apollo) validation; equilibrium-air tier is the dependable workhorse. |
| Determinism under parallel MC and real-time pacing | 11/12 | Hard requirement, not aspiration: order-independent Welford-merge reductions, per-run seeded streams; the byte-diff gate extends to ensemble statistics. |
| New capability drifts toward the dual-use line | 07/06/10 | §6 escalation: locks tighten with capability; reviewer checklist in `13`. |
| Scope/solo-maintainer overload | all | Tiers are independently shippable; Phase A delivers value before Phase D starts; nothing requires a big-bang merge. |

---

## 9. Definition of done (program level)

The program is "at capability parity within posture" when, for each dimension:

- the in-repo model implements the production-class formulation named in its
  doc, at the target tier;
- it is byte-deterministic and byte-stable-by-default, with all invariant gates
  green;
- it earns at least `validated-toy`, and `research` where a public benchmark
  exists, with a tolerance table and an explicit validity envelope;
- its uncertainty is accounted (per-entry margins flowing into Monte Carlo) and
  reported in a NASA-STD-7009B-shaped credibility record;
- its parity ceiling is documented, and the open substitute validation is in
  place; and
- requirements traceability and provenance are complete.

It is **not** done by claiming flight readiness — that remains permanently out
of scope by design.

---

*Companion documents:* `01`–`12`, `14`–`25` (per-dimension design) · `13`
(execution playbook). *Upstream anchors:* `docs/verification.md`,
`docs/standards-posture.md`, `docs/safety-boundaries.md`,
`docs/data-provenance.md`, `docs/software-architecture.md`,
`docs/launch-vehicle-fidelity-frontier.md`.
