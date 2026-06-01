# OpenBMP — Open GNC Platform Design

> **Status:** design proposal (2026-06-01). **Audience:** the OpenBMP author, future contributors, and flight-software engineers evaluating a fork.
>
> **How to read this document.** It describes the *target* architecture for evolving OpenBMP from a single-author SIL simulator into a best-in-class, flight-portable open GNC stack — built the way SpaceX/Rocket Lab/NASA build flight software (one source across a MIL→SIL→PIL→HIL ladder), whose GNC core a fork can lift onto a real flight computer, with developer tooling that serves real-vehicle teams, and with forward trajectory work kept first-class. **Maturity is mixed and called out as such:** the clean nav→guidance→control SIL core *exists today*; the single-evaluator mission sequencer, the `no_std` flight-core carve-out, the HAL/bus extraction, PIL/HIL enablement, the dispersion/conformance tooling, and the forward-only trajectory subsystem are *proposed*. Where a rung or capability is aspirational, the text says so — per the project's "honesty over completeness" rule.
>
> **Dual-use boundary (non-negotiable).** OpenBMP is forward-only by construction: no input type can express a target, and the forward propagator is offline/tooling-tier and architecturally barred from any flight loop. This boundary is *strengthened*, never relaxed, by every milestone here. See §4 and the review-mandated hardening commitments.



## Executive summary

OpenBMP is a 19-crate, edition-2024, `std`-based 6-DOF SIL simulator whose central asset is already in place: a clean, one-directional flight-software pipeline — truth → sensor → EKF/MEKF → guidance on estimated state → autopilot → mixer — with the `Clock` and bus seams that mature stacks (cFS, F´, PX4, ArduPilot) use to make sim-vs-flight a build selection rather than a code path. The initial productization slice has landed: FC-owned mission scenarios are single-evaluated by the commander on estimated state, truth-echo sensors are rejected for FC scenarios, propellant mass fraction is published to and consumed by the commander, scheduler rates/budgets are declarative I-loads, scheduler overruns flow into health/FDIR, the HAL/static-bus/I-load/recorder surface exists, and `openbmp-fc` now passes an alloc-aware `thumbv7em-none-eabihf` no-default-features target check. The remaining portability path is concrete: the host FC bus and registries still carry `Box<dyn Any>`/`TypeId`, dynamic dispatch, and unbounded collections, so the next flight-core hardening step is de-heaping and bounding the hot path rather than merely compiling. The plan still sequences **correctness before portability before hardware** across independently-shippable, V&V-gated milestones; the honest scope is that OpenBMP is a MIL/SIL platform plus a reusable plant and scenario harness that *enables and scaffolds* PIL and HIL — it does not perform them, and its standards posture is alignment, not certification. The forward-only dual-use boundary is non-negotiable and structural throughout: no input type can express a target, and several milestones strengthen that enforcement rather than relax it.

## Goals & non-goals

**Goals (the owner's, restated):**
1. **Best-in-class open GNC stack** — Navigation/Guidance/Control/Sequencing as independently-testable modules behind a sensor boundary, with first-class fault management.
2. **Industry-standard verification** — the SpaceX/Rocket Lab way: the *same source* run across a MIL→SIL→PIL→HIL ladder, with massive automated simulation, CI, golden tests, and Monte-Carlo — flight-grade rigor, honestly scoped.
3. **Flight-portable** — the identical GNC source compiles into the simulator and into a real flight computer in a fork; sim-vs-flight is a Cargo build/link selection behind a clean HAL, with a `no_std`-capable flight core separable from the simulator.
4. **A genuine dev/V&V toolchain** — scenario authoring, dispersion campaigns, external-telemetry validation, and a fork-runnable conformance suite that serves real-vehicle developers.
5. **Trajectory work as a first-class capability** — phased forward propagation, ballistic coast, and Monte-Carlo dispersion, kept forward-only.

**Non-goals (architecturally enforced where marked):**
- **No target / aimpoint / impact-point input** — no input struct carries a desired lat/lon, ECEF, range, or bearing. *(Tier-1: type absence.)*
- **No inverse targeting / launch-parameter solve** — there is no entry point that takes a target and returns launch parameters; the only direction is `scenario → trajectory`. *(Tier-1.)*
- **No guidance-to-target, terminal homing, fire-control, range/firing tables, or miss-distance/CEP-against-aimpoint** — outputs are range-relative or about the sample mean, never error-against-target. *(Tier-1 + Tier-2 lint.)*
- **No optimize-to-reach-a-condition** — only the forward-simulation half of POST2/GMAT/ASTOS is built; the targeting/optimization half is categorically refused.
- **No in-loop trajectory prediction** — the propagator is offline/post-processing only, never reachable from `openbmp-fc` or a bus topic.
- **Not DO-178C / NASA-class certification** — OpenBMP frames itself as *qualification-ready architecture, not qualified*; certification and export footing are re-earned by the integrating fork (the cFS precedent).
- **Not bit-exact cross-platform** — bit-stability is claimed only on the x86_64 reference platform; the cross-platform and host→target contract is tolerance-bounded state-equality.
- **Not fault-tolerant avionics, and not real-time-proven** — OpenBMP is redundancy-management-*ready*; Byzantine/cross-strapped HW fault tolerance and timing/ISR/EMI/WCET are hardware-bench (HIL-rung) concerns a fork owns.

## Guiding principles

- **Flight-core / simulator separation is the load-bearing invariant.** Four strata (L1 shared contracts, L2 flight-core, L3 simulator/plant, L4 tooling) with one hard rule: no L2 crate may reach an L3/L4 crate in the dependency graph. The fork supplies only a board crate and a `flight-main`; everything in L1+L2 is unmodified upstream source.
- **Decisions emerge from flight software on estimated state.** The mission FSM is single-evaluated, behind the sensor boundary, on the EKF estimate. The kernel keeps only plant-side guards (ground impact, NaN, sim-end fences). Event fire-time shifts under SIL are *signal*, not regression.
- **Declarative I-loads, not config files.** The TOML mission graph, `EventBinding` thresholds, and gain/redline tables are the flight-software initialization-load tables; they stay declarative and are flashed as a compact serialized blob — no TOML parser on the flight target.
- **Determinism is a contract, two-tiered.** Bit-stable on the reference platform (deterministic-seed goldens, exact); tolerance-bounded state-stability everywhere else and across the host→target boundary (Monte-Carlo goldens, banded). A shared pinned `libm` keeps the sim a valid cross-check of the flight core.
- **Forward-only by construction.** Weaponization is *unconstructible* (a type error), not merely discouraged; lint and docs raise activation energy but the absence of fields is the wall. Every near-the-line capability lands only behind a Tier-1 binding, and the CI dependency-graph assertion doubles as a dual-use control.
- **Fork-and-deploy.** Two topologies — `sim_topology(HostBus, SyntheticSensors)` and `flight_topology(StaticBus, BoardDrivers)` — differ only in injected backend. A fork adds a board crate implementing the HAL traits and a runner; no GNC code is written.
- **Honesty over completeness.** Never mark a rung or milestone "done" while a blocker stands; MIL=yes, SIL=yes-but-incomplete, PIL=scaffolded, HIL=scaffolded — and say which is which.

## Glossary

- **MIL / SIL / PIL / HIL** — the verification ladder. **MIL** (model-in-the-loop): the GNC model against the plant on host — N/A by design here, since the Rust algorithm *is* the model. **SIL** (software-in-the-loop): the flight-software jobs run on host, closed-loop, on FC-estimated state. **PIL** (processor-in-the-loop): the same source cross-compiled to the target CPU, plant still on host. **HIL** (hardware-in-the-loop): real avionics running the same binary in real time, with the simulated plant feeding sensors and absorbing actuator commands.
- **Flight-core** — the GNC source a fork lifts: navigation/EKF, guidance, control/autopilot, mission sequencing, FDIR (`openbmp-fc` + the `no_std`-clean math subset). `no_std`-capable, `forbid(unsafe_code)`, no heap on the hot path.
- **Plant** — the host-only physics model the flight-core is tested against (the integrator/kernel + vehicle + environment); never linked into a flight binary.
- **HAL / OSAL / PSP** — Hardware Abstraction Layer. cFS deliberately splits it on two axes: **OSAL** ("which OS" — runtime/time, e.g. the `Clock` trait) vs **PSP** ("which board" — peripherals/drivers, e.g. `Imu`/`Gnss`/`TvcActuator`). A fork implements these traits against real silicon.
- **I-load** — initialization-load: the declarative tables (mission graph, event thresholds, gains, redlines) a real flight computer ships in flash; validated on host and flashed as a compact serialized blob.
- **EventScalars / `fired()`** — the shared, portable mission interpreter: the commander builds `EventScalars` from the estimate, and `BuiltInEventTrigger::fired` evaluates declarative triggers against them. The reframe is that only *who builds the scalars and from what state* changes, not the interpreter.
- **FDIR** — Fault Detection, Isolation, and Recovery: the fault-management meta-loop (watchpoint/actionpoint, heartbeat/health, lane voting, safe-state response).
- **Lockstep** — the sim clock advances only when the integrator produces the next state *and* the FC has consumed the prior sensor frame; what makes EKF/scheduler behavior deterministic and replayable across host SIL and a HIL bench.
- **Tier-1 / Tier-2 / Tier-3** — the dual-use locks. **Tier-1**: type-level — no input type can express a target. **Tier-2**: the fail-closed `FORBIDDEN_SAFETY_TERMS` lexical lint, run before deserialization. **Tier-3**: the data-provenance gate — no real fielded-vehicle/motor/TPS/operational data — which is also the de-facto export-control control.
- **Bus / topic** — the typed, anonymous, latest-value pub/sub coupling between components (cFS Software Bus / PX4 uORB / F´ ports analog); `HostBus` (dynamic, sim) and `StaticBus<N>` (compile-time topic enum, zero-heap, flight) implement one `Bus` trait.
- **Rate group** — a deterministic cyclic-executive cadence: one timebase divided into multiple rates via declared `period_ticks` / task-rate I-loads (control high-rate, nav mid, guidance/FDIR low), expressed as topology, not hardcoded loop order.

## Consolidated risk register

| Risk | Type | Severity (likelihood / impact) | Mitigation |
|---|---|---|---|
| Alloc-free carve-out (M6) balloons: `Box<dyn Any>`/`TypeId`/`RefCell` in `bus.rs`/`scheduler.rs`/`params.rs` is a deep refactor | Technical | High / High | Stage per the crate-map ladder: target-compile first (now green), then a const-generic/enum topic table, then bound collections (`heapless`/arena). Ship as sub-PRs each behind a `cargo check --target` gate. Accept embedded-Linux (`std`-on-board) as interim if bare-metal stalls. |
| Cross-platform determinism breaks (ARM NEON / libm / FMA ≠ x86) | Technical | High / Med | Adopt tolerance-bounded as the explicit contract; pin one shared `libm` on host and target; keep the bit-exact gate scoped to the x86 reference platform only. |
| Sequencer migration (S1–S4) silently changes golden behavior | Technical | Med / High | M0 first — promote *all* golden baselines into the merge gate so every S-step is provably behavior-preserving; treat any golden delta as a reviewed behavioral change, not noise. |
| Estimated-scalar gaps (M4) lack a real observable (e.g. relative-nav) | Technical | Med / Med | Fail-closed where no observable exists; never fabricate a scalar. Scope relative-nav as its own sub-task; MECO-on-depletion reuses existing `consumed_kg`. |
| HAL trait surface insufficient for real sensor protocols (CAN/SPI/I2C, buffered/timestamped fusion, GNSS-sync, "healthy" semantics) | Technical | Med / Med | Validate the surface with a reference board-crate implementation before freezing; iterate trait granularity; reuse the proven `Sensor`/`Clock` seeds. |
| Fixed-capacity mission interpreter truncates real scenarios (ArrayVec MAX bounds vs RAM) | Technical | Med / Med | Measure scenario complexity against realistic embedded RAM (64–512 KB) before committing bounds; keep load-time graph validation on host (L4), per-tick interpreter bounded (L1). |
| HIL/real-time fidelity assumed proven by host-passing | Scope | Med / Med | Explicit doc disclaimer: timing/ISR/EMI/WCET are HIL-rung concerns OpenBMP cannot verify; label PIL/HIL "scaffolded," DO-178C "architectural readiness, not certified." |
| Solo-author scope creep across 11 milestones; over-claiming "done" | Scope | High / Med | Each milestone independently shippable with its own V&V gate; never mark done while a blocker stands; honest rung/standards status table published. |
| FDIR corridor evaluator (M10) drifts toward AFTS / aimpoint | **Dual-use** | Low / Critical | Land only behind Tier-1 binding: vehicle-state + environmental-fence inputs only, safe-state/terminate output only; extend `FORBIDDEN_SAFETY_TERMS`; CI assertion it is unreachable from guidance; document in the residual-surface table. |
| Forward-trajectory subsystem (M9) tempts "just add the optimizer" (POST2/GMAT/ASTOS targeting half) | **Dual-use / Scope** | Med / Critical | Forward half only; prominent NON-GOALS subsection; Tier-1 lock (no target field in `BallisticState`); range-relative / about-the-mean output only; per-PR merge checklist. |
| Board/backend fork seam re-imports prohibited capability | **Dual-use** | Low / Critical | Extend the M2 CI dependency-graph assertion to prove no prohibited-capability crate is reachable from any board target; HAL traits accept no aimpoint/coordinate/range; governed by `docs/dual-use-assessment.md`. |
| User imports controlled real ballistic/aero data, producing an operationally valid prediction | **Dual-use / Export** | Low / High | The architecture cannot prevent this; the Tier-3 provenance gate (SHA-256 content pins, no real fielded data) is the control, tied to `EXPORT-CONTROL.md`; adopter responsibility documented. |
| `IdealStateSensor` truth-echo used as a SIL escape hatch, defeating the sensor boundary | Technical / Dual-use | Med / Med | CI lint that no `[fc]`-authority scenario uses `IdealStateSensor`; no truth-echo path reachable in a flight build. |


---

# 1. Target Architecture & Flight Portability


> **Goal (owner's #3).** The *same* GNC source — navigation/EKF, guidance, control/autopilot, mission sequencing, FDIR — compiles into the simulator and into a real flight computer in a fork. Sim-vs-flight becomes a **Cargo build/link selection, not a code path**, exactly as ArduPilot (`waf --board sitl` vs `--board <stm32>`), PX4 (SITL vs NuttX target), cFS (OSAL+PSP per target), and F´ (CMake toolchain) all do. The application/GNC source is byte-identical across targets in every one of those stacks; OpenBMP should match that property.

This section assumes the SIL reference pattern already in the tree is *correct and not to be refactored*: truth → sensor (`fc_bridge.rs:207-244`) → EKF/MEKF (`estimator.rs`) → guidance on `PositionEstimate` → autopilot. Flight portability is about **where that pipeline can be compiled and what it links against**, not about changing its data flow.

### 1. The four strata and how today's 19 crates map onto them

The verification ladder (`vnv-standards` dossier) and every mature stack converge on one structural rule: **dependency arrows point down only, and FLIGHT-CORE never names SIMULATOR or TOOLING.** Today this is materially true at the dependency level (`openbmp-fc` does not depend on `openbmp-sim`/`openbmp-runner` and now has a `thumbv7em-none-eabihf` no-default-features check), but the flight-controller path remains `alloc`-aware and still carries `Box<dyn Any>`/`TypeId` registries, so the remaining bare-metal work is de-heaping and bounding the hot path rather than merely compiling.

| Stratum | Contract | Constraints | What lives here |
|---|---|---|---|
| **L1 SHARED contracts** | types every layer agrees on | `no_std`-first, `alloc`-free, deterministic | `openbmp-core`, `openbmp-state`, `openbmp-models` (trait surfaces), `openbmp-mission` (event/HSM types + interpreter), a *new* `openbmp-hal` |
| **L2 FLIGHT-CORE** | the GNC code a fork lifts | `#![no_std]`-capable, `#![forbid(unsafe_code)]`, no heap on hot path, all environment access via L1 traits | `openbmp-fc` (after remediation), the no_std-clean subset of `openbmp-physics`/`openbmp-propulsion`/`openbmp-aero` math |
| **L3 SIMULATOR / PLANT** | the host-only physics + virtual board | `std` OK, heap OK, **never** linked into a flight binary | `openbmp-sim` (kernel/integrator), `openbmp-runner` + `fc_bridge.rs`, `openbmp-vehicle`, `openbmp-aerothermal`, host parts of `openbmp-sensors`/`openbmp-propulsion`/`openbmp-aero` |
| **L4 TOOLING** | dev/V&V/ground, never flight | full `std`, filesystem, serde, clap, parquet | `openbmp-cli`, `openbmp-scenario`, `openbmp-scenario-script`, `openbmp-telemetry`, `openbmp-bridge`, `openbmp-testkit` |

This is cFS Apps/cFE/OSAL/PSP and the F´ component/Os/Drv split rendered as a Cargo workspace. The single load-bearing invariant — enforced in CI (§5) — is: **no L2 crate may have a path to an L3 or L4 crate in the dependency graph.**

#### Before / after crate table

| Crate | Today | Target stratum | Change required |
|---|---|---|---|
| `openbmp-core` | std (docstring promises no-alloc, unenforced) | **L1** | gate `std`; `#![cfg_attr(not(feature="std"), no_std)]`; pin `libm` for transcendentals |
| `openbmp-state` | std via nalgebra | **L1** | nalgebra `default-features=false`; no_std build target |
| `openbmp-models` | std | **L1** | trait surfaces only; no_std |
| `openbmp-mission` | `BTreeMap`/`BTreeSet`/`Vec`/`VecDeque` | **L1** (interpreter) / **L4** (graph *validation*) | split: keep `EventScalars`+`fired()` interpreter in L1 (`heapless`/arena); move Tarjan/reachability *load-time* validation to L4 |
| `openbmp-fc` | `no_std` + `alloc` target check; `Box<dyn Any>`, `RefCell`, `IndexMap`, `String` remain | **L2** | compiles bare target with defaults off; next job is static bus / bounded collections; `hal` feature already exists |
| `openbmp-sensors` | `Sensor` trait + `synthetic` feature + `std::fs` parser | **trait → L1, synthetic → L3** | `Sensor` trait is already portable; keep parser/noise behind `synthetic` |
| `openbmp-physics` | std math + profile types | **split L2/L3** | EOM/gravity/propagator math → no_std L2; deck parsers → L3 |
| `openbmp-propulsion` | parser + dynamic `Vec<Box<dyn EngineModel>>` cluster | **split L2/L3** | motor/engine math now compiles `no_std` with `alloc`; parser stays host; replace cluster `Vec<Box<dyn>>` with enum/fixed array before claiming alloc-free bare-metal |
| `openbmp-aero` | parser + tabulated/hypersonic aero math | **split L2/L3** | deck lookup, buildup, hypersonic, and Knudsen math now compile `no_std` with `alloc`; parser stays host-only |
| `openbmp-aerothermal` | thin model wrapper | **L3** (entry-only) | stays host until entry-phase WCET is measured |
| `openbmp-sim` | deterministic kernel, no fs/threads | **L3** | this is the *plant*; stays host even though it's no_std-clean today |
| `openbmp-vehicle` | composition | **L3** | host assembly |
| `openbmp-runner` / `fc_bridge.rs` | sim glue | **L3** | this is the SITL backend (the ArduPilot `SITL_State` analog) — intentionally not portable |
| `openbmp-scenario(-script)` | `std::fs` TOML | **L4** | host-only; emits a compact serialized I-load for flight |
| `openbmp-telemetry` | CSV/JSON/Parquet | **L4** | ground/dev only |
| `openbmp-cli`, `openbmp-bridge`, `openbmp-testkit` | clap / postcard / proptest | **L4** | host-only |
| **`openbmp-hal`** *(new)* | trait set (§2) | **L1** | embedded-hal-style contract crate |
| **`openbmp-fc-sim`** *(new)* | sim impls of HAL + bus | **L3** | extracted from `fc_bridge.rs` so `openbmp-fc` stops needing `synthetic` |

### 2. The HAL boundary (`openbmp-hal`)

Mirror cFS's deliberate **two-axis split** (OSAL = "which OS", PSP = "which board") and ArduPilot's `AP_HAL` interface list. Do **not** merge the axes: a fork must be able to add a board without touching the runtime, and run the runtime on a desktop with no board.

- **Axis A — runtime/time (OSAL-equivalent):** `Clock` (the one already abstracted in `clock.rs` — a `SimulatedClock` advances in lockstep with the integrator, a `MonotonicClock` reads a hardware timer; *no `std::time` in L1/L2*, already lint-forbidden). This is the PX4 lockstep / ArduPilot `SITL_State` insight: in sim, **the integrator drives time**, which is what makes EKF/scheduler runs deterministic and replayable.
- **Axis B — board/peripherals (PSP+driver-equivalent):** `Imu`, `Gnss`, `Magnetometer`, `Barometer`, `StarTracker` (one `read() -> Result<Measurement>` each — the existing `Sensor` trait in `openbmp-sensors/src/sensor.rs:171` is already this shape and is the seed); `TvcActuator`, `RcsValve`, `ThrottleCommand` on the output side; `Storage` (flight-recorder / I-load load) and `Watchdog`/health.

```
trait Clock        { fn now(&self) -> SimTime; }                 // sim: integrator-driven; flight: HW timer
trait Imu          { fn read(&mut self) -> Result<ImuSample>; }  // sim: SyntheticSensorAdapter; flight: SPI/CAN driver
trait TvcActuator  { fn command(&mut self, c: TvcCmd) -> Result<()>; }
trait Storage      { fn load_iload(&self) -> Result<&[u8]>; fn append_record(&mut self, r: &[u8]) -> Result<()>; }
```

**The fork's whole job is to implement these traits against real silicon.** The sim implements the identical traits against the kernel (`fc_bridge.rs` already constructs `SensorTruth` per tick and applies commands to effectors — that *is* the sim-side HAL impl, it just needs to live behind the trait in `openbmp-fc-sim` rather than being baked into the runner). Decide the generics-vs-dyn boundary deliberately (per `fsw-cfs` rec): **monomorphized generics at the hot inner loop** (zero-cost, no_std-friendly, embedded-hal style) and **`dyn` only at the topology seam** where binary-size/flexibility wins.

### 3. The message bus as an abstract contract

The bus is the cFS Software Bus / PX4 uORB / F´ typed-ports analog: components couple **by message type, not by call graph**, so a sim sensor source and a real driver are interchangeable publishers and every stage is independently mockable and recordable (your `compare-telemetry` harness becomes just another subscriber). OpenBMP already has a typed pub/sub bus in `openbmp-fc/src/bus.rs` with the right *semantics* (anonymous, latest-value single-slot, timestamped topics) — but the *implementation* is the portability blocker.

The fix is to make the bus a **trait the flight core depends on abstractly**, with two impls:

| | Host impl (sim) | Flight impl |
|---|---|---|
| storage | today's `RefCell<Box<dyn Any>>` keyed by `TypeId` (`bus.rs:35,97-126`) is fine | static, compile-time topic table; no heap, no `TypeId` |
| dispatch | dynamic `Rc<TopicCell<T>>` lookup | const-generic / enum-indexed topic array |
| allocation | boot-time | **zero** after `init()` |

Concretely: define `trait Bus { fn publish<T: Topic>(&self, v: T); fn read<T: Topic>(&self) -> Option<T>; }`, keep the current dynamic impl as `HostBus` (L3), and add a `StaticBus<const N: usize>` generated from a compile-time enum of all flight topics (no `Box<dyn Any>`, no `Rc`). Borrow uORB's concrete choices verbatim: every message carries a timestamp (already true), latest-value single-slot for sensor/setpoint topics, multi-instance topics for redundant sensors (feeds the ArduPilot-style lane voting in the GNC dossier).

### 4. App/component decomposition (cFS-apps / F´-style)

The current jobs (estimator, commander, guidance, autopilot, mixer, health, FDIR) registered in `openbmp-runner/src/fc.rs:116-309` are a component set: they communicate only over the bus and are scheduled by a **rate-group driver that divides one timebase** (F´ `RateGroupDriver`/`ActiveRateGroup`; cFS `SCH`). The runner now expresses cadences and declared budgets as config/topology, and the scheduler publishes overrun events that the health/FDIR path can observe. Hardware WCET measurement remains a PIL/HIL addition.

| Component | Role | Pattern source | OpenBMP today |
|---|---|---|---|
| Navigation | EKF/MEKF state estimate | PX4 EKF2 / ArduPilot EKF3 lanes | `estimator.rs`, `estimator_lanes.rs` |
| Guidance | inertial/orbital references (PEG, gravity-turn) — **forward-only, never a ground location** | NASA PEG/IGM (`gnc-architecture`) | `guidance.rs`, `profile.rs` |
| Control | autopilot → actuator commands | classical DAP | `autopilot.rs`, mixer |
| Mission | declarative HSM + `EventScalars`/`fired()` interpreter | cFS SC / PX4 commander | `commander.rs`, `openbmp-mission/src/events.rs` |
| FDIR | watchpoint/actionpoint + heartbeat + safe-state | cFS LC/HS + AFTS (forward-only corridor) | `health`, FDIR jobs |

The **topology/wiring module** is the structural guarantee that makes the core liftable: exactly two topologies that differ *only* in which backend they inject — `sim_topology(HostBus, SyntheticSensors)` and `flight_topology(StaticBus, BoardDrivers)`. Everything above the injection point is byte-identical, shared code. Keep the TOML mission graph + `EventBinding` thresholds + `hsm.rs` transition table as the **declarative flight I-load tables** (the cFE TBL-service analog); flash a compact serialized form (postcard/bincode), do not parse TOML on the flight target.

### 5. Portability-blocker remediations (ordered, cheapest-first)

These come straight from the `crate-map` and `fsw-boundary` dossiers and are verified in-tree (`bus.rs:35,97-126` heap dispatch; `openbmp-core/src/lib.rs:10-15` no-alloc docstring with *no* `#![no_std]`; `panic = "abort"` already set in the release profile — one prerequisite already met).

1. **Establish the no_std floor (L1).** Add `#![cfg_attr(not(feature="std"), no_std)]` to `openbmp-core`/`-state`/`-models`; flip nalgebra/uom/rand/serde to `default-features = false` and add a `std` feature that is **off in the core's default set** (prevent std creep). Add a CI target `cargo check -p openbmp-core --no-default-features --target thumbv7em-none-eabihf`. *Cost: low–medium; the foundation is mostly fixed-size math already.*
2. **Pin shared math.** Use the `libm` crate for `sin/cos/exp/sqrt` in **both** the no_std core and the std sim build so sim and flight compute identical transcendentals — this turns the sim into a tolerance-bounded cross-check of the flight core. State the contract explicitly: **identical behavior = tolerance-bounded, not bit-exact** (your `compare_telemetry` tooling already implies this). Bit-exactness is only claimed for x86_64; ARM NEON/scalar will differ in the last ULPs.
3. **De-heap the bus (L2).** Replace `RefCell<Box<dyn Any>>` + `TypeId` with a `trait Bus` + a compile-time topic enum + `StaticBus<N>`. Keep `HostBus` for sim. *This is the single biggest remaining unblock for an alloc-free `openbmp-fc` flight path.*
4. **De-heap the registries.** `params.rs`/`tables.rs` `Box<dyn Any>` and `scheduler.rs` `Box<dyn Job>` are **boot-time only, zero per-frame alloc** (per `fsw-boundary`), so they're acceptable on a Linux flight board immediately; for bare-metal, move to fixed-slot arenas / const-generic registration.
5. **Fix the dynamic-dispatch cluster points.** `openbmp-propulsion/src/cluster.rs` `Vec<Box<dyn EngineModel>>` → sum-type enum or fixed `[Engine; N]`; same for any `Vec<Box<dyn>>` in aero buildup.
6. **Split the mission crate.** Graph validation (Tarjan SCC, reachability — `hsm.rs`) runs once at load → L4 host; the per-tick `EventScalars`/`fired()` interpreter (`events.rs:206+`) → L1 with `heapless`/arena-backed state, fixed-capacity `ArrayVec` for bindings/lanes.
7. **Extract the sim HAL impl.** Pull `SensorTruth` construction + `SyntheticSensorAdapter` + effector application out of `fc_bridge.rs` into `openbmp-fc-sim` (L3), so `openbmp-fc` no longer needs the `synthetic` feature and depends only on `openbmp-hal` traits.
8. **Engineer panics out of L2.** Build with `panic = "abort"` (done), put the `#[panic_handler]` in the firmware binary (not the library), and treat `unwrap`/`expect`/unchecked-index/unchecked-arith in L2 as flight defects (the workspace already lints `unwrap_used`/`expect_used`/`panic` — tighten to `deny` for L2). Aspire to a `panic-never`-style core.

**Honest scope.** A *Linux-on-a-real-board* first milestone (the ArduPilot `HAL_Linux` / PX4-on-POSIX path) reaches "real hardware" with the **least no_std rework** and is the right first liftability target; bare-metal RTOS (RTIC, no heap, Hubris-style static allocation) is a later milestone. Develop on stable Rust now and treat **Ferrocene** as a drop-in qualified toolchain later (it tracks pinned upstream rustc; same source compiles) — note honestly that Ferrocene covers ISO 26262 / IEC 61508 / IEC 62304 today but **there is no turnkey DO-178C Rust certificate**, and DO-178C qualification is re-earned per integrating program (the cFS precedent: open release ≠ transferable certification). Frame OpenBMP as **qualification-ready architecture, not qualified**.

### 6. Target dependency diagram

```
                         L4 TOOLING (host only, never flight)
   openbmp-cli ── openbmp-runner ── openbmp-scenario(-script) ── openbmp-telemetry ── openbmp-bridge ── openbmp-testkit
        │              │                                                                       
        │              ▼                                                                       
        │   ┌───────────────────── L3 SIMULATOR / PLANT (host) ─────────────────────┐         
        │   │  openbmp-sim (kernel/integrator)   openbmp-vehicle   openbmp-aerothermal│         
        │   │  openbmp-fc-sim  (HostBus + sim HAL impls; was fc_bridge.rs)            │         
        │   │  host parts of: openbmp-aero / -propulsion / -sensors(synthetic)        │         
        │   └─────────────────────────────────┬──────────────────────────────────────┘         
        │                                      │   injects backend
        │   ╔══════════════════════════════════▼══════════════════════════════════╗            
        └──▶║                L2 FLIGHT-CORE  (no_std-capable, alloc-aware)          ║            
            ║   openbmp-fc  (nav · guidance · control · mission · FDIR · StaticBus) ║            
            ║   no_std math subset of: openbmp-physics / -propulsion / -aero        ║            
            ╚══════════════════════════════════┬══════════════════════════════════╝            
                                               │ depends on traits/types only
            ┌──────────────────────────────────▼──────────────────────────────────┐            
            │  L1 SHARED CONTRACTS (no_std-first)                                   │            
            │  openbmp-hal (Clock·Imu·Gnss·Mag·Baro·Tvc·Rcs·Storage·Watchdog·Bus)   │            
            │  openbmp-core   openbmp-state   openbmp-models   openbmp-mission(intp)│            
            └──────────────────────────────────────────────────────────────────────┘            

   A FORK ADDS, in its own repo:
            ┌──────────────────────────────────────────────────────────────────────┐
            │  openbmp-board-<x>  (impl L1 HAL traits against real IMU/GNSS/TVC/...)  │
            │  flight-main  (RTIC/bare loop: build flight_topology, call fc.step())   │
            └──────────────────────────────────────────────────────────────────────┘
```

Dependency arrows point **down only**. The fork supplies just the two boxes at the bottom; everything in L1+L2 is the unmodified upstream source.

### 7. How a fork deploys to real hardware

1. **Add a board crate** `openbmp-board-<x>` that `impl`s the `openbmp-hal` traits: `Clock` against the MCU monotonic timer, `Imu`/`Gnss`/`Magnetometer` against the real SPI/I2C/CAN drivers, `TvcActuator`/`RcsValve` against PWM/valve outputs, `Storage` against flash/FRAM, `Watchdog` against the hardware WDT. *No GNC code is written.*
2. **Write `flight-main`** (the runner-equivalent the fork owns): construct the `FlightController`, build `flight_topology(StaticBus, board_drivers)`, register the same component jobs, and in a real-time loop (RTIC task or bare NVIC) call `fc.step()` and push `fc.latest_actuator_command()` to the effectors. The fork replaces `openbmp-runner`; **`openbmp-fc` is unchanged.**
3. **Flash the I-load**, not the parser: run `openbmp-cli`/`openbmp-scenario` on the host to validate and serialize the mission graph + `EventBinding` thresholds + gain tables into a compact blob; the flight build deserializes it from a `const` byte array or `Storage` — no TOML, no `std::fs` on target.
4. **Select the build:** `cargo build --target <mcu> --features board-<x> --no-default-features` (the `hal` feature on `openbmp-fc` already exists and is CI-tested via `tests/hal_topic_gate.rs`, which proves a sim-only topic is compiled out in the current host HAL build under `--features "std hal" --no-default-features`). This is `waf --board <stm32>` for OpenBMP.
5. **Climb the ladder with the same plant:** the fork carries `openbmp-sim` (the reusable "plant") to **PIL** (cross-compile L2 to the target CPU, plant still on host) and **HIL** (real board, real-time, real I/O). OpenBMP **scaffolds** PIL/HIL; it does not perform them (those need target boards + a real-time bench, and host-passing does not prove real-time/ISR/EMI correctness — a HIL-rung concern, stated honestly).

**Dual-use guardrail extension (non-negotiable).** The new L1 `openbmp-hal` and any L4→L1 board crate are exactly where a fork could quietly cross the forward-only line. Extend the Tier-1 lock — *no input type can express a target* — into the HAL: no trait method accepts an aimpoint/coordinate/range; the actuator traits command vehicle-intrinsic effectors only; guidance components consume inertial/orbital references, never a ground location. Add the dependency-graph CI assertion (no L2 path to L3/L4) **and** a check that no prohibited-capability surface is reachable from a board target, governed by `docs/dual-use-assessment.md`. The board layer inherits the adopter's export-control footing (per `EXPORT-CONTROL.md`); document this at the awareness/policy level.



---

# 2. The GNC Stack


This section specifies what makes OpenBMP a best-in-class *open* GNC stack: not the cleverness of any single estimator or guidance law, but the architectural discipline that keeps Navigation / Guidance / Control / Sequencing as independently-testable modules behind a sensor boundary, scheduled deterministically, wrapped in a first-class fault-management layer, and — critically — running the *same* decisions in simulation as a fork would run on a flight computer. The good news from the code audit is that the reference SIL loop and most of the FDIR/redundancy scaffolding already exist in `openbmp-fc`. The first wiring pass is now in place: mission authority can belong to the commander, propellant mass fraction is an onboard topic, scheduler rates/budgets are scenario I-loads, estimator-regime phase signals drive high-dynamics process-noise scheduling, and scheduler overrun events are health/FDIR visible. The remaining GNC work is narrower: relative-navigation triggers are parse-time unsupported for FC authority until an observable exists, advanced guidance/estimator variants remain research-grade, and hardware timing/WCET evidence remains a downstream PIL/HIL bench problem.

The forward-only guardrail is preserved throughout: guidance consumes only inertial/orbital references (`InertialWaypoint`, `PegGuidance` targeting orbital radius + tangential speed — `crates/openbmp-fc/src/guidance.rs:46,171`), no module accepts a ground location, and nothing in this section adds an input type that can express a target.

### 1. Module boundaries — keep the SIL loop, productize it

The reference data path is already clean and one-directional, and **must not be refactored**. Truth → sensor model (`crates/openbmp-runner/src/fc_bridge.rs:207`) → estimator (`crates/openbmp-fc/src/estimator.rs`, `Ekf` at :139, `Mekf` at :299) → guidance on the `EstimatedState` topic (`crates/openbmp-fc/src/guidance.rs`) → 3-loop autopilot (`crates/openbmp-fc/src/autopilot.rs`, `ThreeLoopAutopilot` at :40 over `RateLoop`/`AttitudeLoop`/`GuidanceLoop` at :54/:69/:84) → mixer (`crates/openbmp-fc/src/mixer.rs`). This is exactly the Basilisk/F-Prime "identical module interfaces for plant and FSW" property [Basilisk: hanspeterschaub.info/basilisk; F-Prime ports/topology: fprime.jpl.nasa.gov] and the PX4/ArduPilot "flight algorithms never touch hardware, they consume bus messages" property [docs.px4.io/main/en/middleware/uorb.html]. The coupling is data-only: every stage communicates through typed topics on `crates/openbmp-fc/src/bus.rs` (`ImuSample`, `GnssSample`, `EstimatedState`, `GuidanceSetpoint`, `AutopilotCommand`, `ActuatorCommand`, `HealthStatus`, `FaultStatus`, `MissionState`, `PropellantState` — `crates/openbmp-fc/src/topics.rs`).

The four canonical roles map onto crate modules cleanly:

| GNC role | Module(s) | Status | Productization gap |
|---|---|---|---|
| Navigation / estimation | `estimator.rs` (`Ekf`, `Mekf`), `estimator_lanes.rs` (`MultiLaneEstimator`) | EKF + MEKF + lane-switching implemented; high-dynamics Q-scaling is driven by commander-published estimator-regime state | Square-root form / SR-UKF hardening remains behind feature work; attitude observability limits need more lane evidence (§6) |
| Guidance | `guidance.rs` (`PitchProgramGuidance`, `GravityTurnGuidance`, `PegGuidance`, `WaypointGuidance`) | PEG + open-loop references implemented | Convex/successive-convex powered-descent guidance is white space [Acikmese/Blackmore: larsblackmore.com/losslessconvexification.htm]; not yet present |
| Control | `autopilot.rs` (`ThreeLoopAutopilot`), `mixer.rs` | 3-loop (rate/attitude/guidance) + effector mixing | Gains are compile-time `AutopilotConfig`; move to runtime tables (`tables.rs`/`params.rs`, cFE TBL-service analog [github.com/nasa/cFE]) |
| Mode / sequence | `commander.rs` + `openbmp-mission` (HSM, `EventBinding`, `BuiltInEventTrigger`) | FC-owned scenarios are single-evaluated by the estimated-state commander; kernel mission actions are gated for FC authority | Relative-nav trigger scalars remain unsupported and fail closed until an onboard observable exists (§2) |
| Fault management | `fdir.rs` (`FdirMonitor`, checks), `health.rs` (`HealthMonitor`) | FDIR state machine + 4 checks + health monitor | Redundancy management thin; AFTS-style corridor engine absent (§3) |

**Decision (do not relitigate):** the Nav→Guid→Ctrl chain is the project's reference SIL pattern. All new GNC capability lands *as a module on this bus*, never as a side-channel. The productization theme is: lift compile-time `*Config` structs into runtime-loadable tables so the same binary is tuned per scenario/board (the cFE TBL-service / ArduPilot `SIM_*` parameter discipline), and feature-gate alternative algorithms (square-root EKF, convex descent) without forking the data path.

### 2. Mission sequencer fix — single evaluator on estimated state (flagship)

This was the single most important architectural correction in the GNC stack and is now implemented for FC-owned scenarios. The commander runs the mission/event FSM on bus-published estimated state behind the sensor boundary (`crates/openbmp-fc/src/commander.rs`), while the simulator kernel keeps truth-side mission evaluation only for kernel-authority / scenario-director runs. When mission authority is `FlightController`, kernel mission-event firing and truth-fired non-transition actions are gated off (`crates/openbmp-sim/src/kernel.rs`), so the FC path is the single source of mission decisions.

**The I-load reframe (the conceptual key).** The TOML mission graph + `EventBinding` thresholds + the HSM transition table are not "config files" — they are the flight-software **I-load (initialization-load) tables**, the same tables a real FC ships in flash. They stay declarative. `EventScalars` (`crates/openbmp-mission/src/events.rs`) + `BuiltInEventTrigger::fired` (:41,:58) are the **shared interpreter** and stay shared. The only thing that changes is **who builds the scalars and from what state**: the commander, from the estimate behind the sensor boundary. This reframe matters because it means the fix is *not* "rewrite the sequencer" — it is "make the estimated-state evaluator the sole one and delete the truth-fired path," preserving every declarative table.

Migration status (forward-only, S1–S4):

1. **S1 — gate kernel eval on `!fc_owned`: done.** FC-owned scenarios no longer fire mission events or non-transition mission actions from integrated truth.
2. **S2 — flip the default / explicit director: done.** `[fc]`+`[mission]` scenarios resolve to `MissionStateAuthority::FlightController` unless an explicit scenario-director/test-stimulus authority is selected.
3. **S3 — close estimated-scalar gaps: partly done.** `PropellantState` is published by the FC bridge from tank state and the commander consumes `PropellantState.mass_fraction` for `AtMassFraction`; FC-authority scenarios reject relative-distance/speed triggers because no onboard relative-nav observable exists yet, so those trigger families fail closed rather than reading truth or zero.
4. **S4 — single evaluator: done for FC authority.** The kernel keeps plant-side guards and kernel-authority simulation use cases; all FC-owned mission decisions live in the commander.

**CI lint that locks it:** `IdealStateSensor` (`openbmp-sensors`) echoes truth bit-equal — a legitimate test fixture, but an escape hatch that defeats the sensor boundary. FC scenarios now reject it (`REQ-SIL-002`), so the boundary cannot be quietly bypassed.

### 3. FDIR / fault management / redundancy

Fault management is a first-class meta-loop, not scattered checks — "detect that a system has failed or will fail, and take a control action to return to a controllable state" [NASA SLS FM: ntrs.nasa.gov/citations/20140011598]. OpenBMP already has the skeleton, which is a real head start: `crates/openbmp-fc/src/fdir.rs` defines `FdirState` (Nominal/Suspected/Confirmed/SafeHold, :29), `FaultClass` (Sensor/Actuator/Estimator/Mission, :40), `FdirResponse` (None/Annunciate/Failover/SafeHold), an `FdirConfig` with a debounce window (:77), the `FdirCheck` trait (:123), and four concrete checks — `ResidualChiSquareCheck`, `ActuatorSaturationCheck`, `RateLimitCheck`, `GlrtCheck` (:140–246). `crates/openbmp-fc/src/health.rs` adds a `HealthMonitor`/`HealthReport`. This maps directly onto the cFS FDIR app suite and what a launch vehicle needs:

| Launch-vehicle FM need | cFS analog | OpenBMP today | Build |
|---|---|---|---|
| Redline / limit monitoring | LC watchpoint+actionpoint [github.com/nasa/LC] | `ResidualChiSquareCheck`, `RateLimitCheck` | Make watchpoints declarative I-load (TOML), like events |
| Heartbeat / aliveness / watchdog | HS [github.com/nasa/HS] | `HealthMonitor` tracks sensor stale/unhealthy states plus sustained scheduler overruns; HAL exposes a `Watchdog` trait | Hardware watchdog service policy and board timing remain downstream |
| Estimator redundancy / lane voting | ArduPilot EKF3 cores [AP_NavEKF3] | `MultiLaneEstimator` (`estimator_lanes.rs:63`), `LaneSelection` (:80), `switch_margin`+dwell (:141) | Wire ≥2 lanes in a scenario; `enable_failover` path end-to-end |
| Safe-state / sequenced response | SC stored-command [github.com/nasa/SCH] | `FdirResponse::SafeHold` enum only | Add a sequenced safing action (RCS attitude-hold/coast) |
| Flight-corridor / AFTS-style rule engine | — (launch-vehicle-native) | absent | **Forward-only**: corridor = deviation from forward-propagated nominal, never an aimpoint; declarative bounds, voting, safe-state action [AFSS: ntrs.nasa.gov/citations/20080044860] |

**Ranking:** (a) keep expanding the existing `Failover` path through `MultiLaneEstimator` into realistic dual-IMU/GNSS scenarios; (b) carry declared-budget overruns and redlines into reviewed FDIR requirements, now that both are I-load visible; (c) add hardware watchdog/storage/actuator error response in board-specific HAL layers; (d) the AFTS-style corridor engine last, with a Tier-1 lock so the "corridor" is provably deviation-from-nominal, not distance-to-target. Honest scope: true Byzantine/cross-strapped HW fault tolerance is a hardware problem; OpenBMP is **redundancy-management-*ready*** (it models lane voting and FTE logic), not fault-tolerant avionics — state this, don't overclaim [Butler primer: shemesh.larc.nasa.gov/fm].

### 4. Rate-group / minor-major scheduling

The flight core needs a deterministic cyclic executive that divides one timebase into multiple rates — the cFS SCH / F-Prime `RateGroupDriver` / ArduPilot `AP_Scheduler` pattern [fprime.jpl.nasa.gov rate-group; github.com/nasa/SCH]. OpenBMP now has that host-SIL shape: `crates/openbmp-fc/src/scheduler.rs` registers periodic jobs with `{period_ticks, budget_us, priority}`, caches dispatch order at registration time, and emits `scheduler.overrun` events when a declared budget cannot fit in the remaining frame budget. `crates/openbmp-runner/src/fc.rs` maps `[fc.scheduler]` rates and budgets into estimator/guidance/commander/autopilot/mixer/health/FDIR registrations. Timing flows through the injected `Clock` trait (`crates/openbmp-fc/src/clock.rs`), with a `SimulatedClock` advanced by the integrator (PX4 lockstep / ArduPilot SITL_State discipline [deepwiki ArduPilot SITL]) and a real monotonic clock behind the same interface for a fork — no `std::time` in FC code, enforced by a workspace lint.

Remaining timing gaps:

1. **Cadence changes are behavioral rebaselines.** The runner now supports declarative rates, but changing defaults from the current baseline changes closed-loop telemetry by construction. Any cadence change is a reviewed golden-baseline update, not a cosmetic config edit.
2. **Declared-budget overrun is not WCET proof.** Budget exhaustion is now health/FDIR-visible (`REQ-SCHED-002`), and the scheduler avoids per-dispatch allocation of its ordering buffer, but there is still no hardware wall-time/WCET measurement, interrupt-latency budget, or stack-depth evidence. Passing in host SIL does not prove flight timing — that remains a PIL/HIL-rung concern [MathWorks SIL/PIL].

### 5. Propellant-depletion cutoff — moving the decision onboard

The clearest worked example of "move the decision onboard, restore Monte-Carlo fidelity." Booster MECO can fire on `AtTime`/`AtVelocity`, but a real cutoff is **propellant-depletion**: the engine quits when the tank empties, which is where most ascent dispersion comes from. The onboard plumbing now exists:

- `EngineState.consumed_kg` (`crates/openbmp-propulsion/src/engine.rs:67`) — cumulative propellant consumed, per engine.
- A `PropellantState` bus topic (`crates/openbmp-fc/src/topics.rs`) with `mass_fraction`, `mass_remaining_kg`, `mass_initial_kg`, and a `depleted` flag.
- `BuiltInEventTrigger::AtMassFraction { fraction }` (`crates/openbmp-mission/src/events.rs:62`) — the declarative trigger.

Current status:

1. The FC bridge aggregates tank state into `PropellantState` (`propellant_state_from_tanks`) and publishes it before the controller tick.
2. The commander reads `PropellantState.mass_fraction` instead of a hardcoded `1.0`; `AtMassFraction` tests exercise the transition path.
3. Scenario authors can now put MECO/cutoff logic behind `AtMassFraction` (or a depletion event), so dispersed Isp/thrust/propellant-load draws can produce dispersed cutoff times — restoring the dispersion the `phalcon9_orbit_monte_carlo` harness is meant to exercise [NASA MC: ntrs.nasa.gov X-43A TM].

The same "FC populates the scalar" pattern generalizes: it is exactly how S3 closes `AtMassFraction`/relative-nav, and it keeps the cutoff decision *onboard the estimated-state path* rather than a kernel truth shortcut.

### 6. Sensor fusion & observability — known blockers as design inputs

Two in-flight findings from prior campaigns are the design drivers here; cite them in the V&V narrative as evidence the methodology surfaces real GNC failure modes [Monte-Carlo as the detector: ntrs.nasa.gov].

1. **GNSS innovation gate collapses under thrust.** A tight 6-D GNSS innovation gate dead-reckons under high acceleration → divergence. The estimator exposes `gate_chi2` and `high_dynamics_q_scale`, and the FC now drives high-dynamics process-noise scheduling from commander-published estimator-regime state. That turns the boost/coast phase signal into a declarative, observable schedule rather than a hidden hand tune. This is what enabled the clean two-stage orbit (`phalcon9-orbit`, e≈0.003).
2. **Attitude observability at off-pole / equatorial launch.** A mag-aided MEKF showed out-of-plane attitude drift at off-pole launch (an observability gap, not a tuning bug). Design responses, in order: (a) ensure the lane-switching `MultiLaneEstimator` (§3) can prefer a healthier attitude lane; (b) treat under-observable attitude as an FDIR `Estimator`-class condition (`fdir.rs:40`) that annunciates rather than silently drifts; (c) document it as a sensor-suite/geometry limitation, not a solved problem. Be precise about MODEL artifacts vs GNC findings — e.g. the equivalent-pendulum freefall translational drift seen post-slosh-closure is a plant-model artifact, not an estimator failure, and must not be reported as a fixed GNC bug.

The square-root EKF (feature-gated per the crate map) is the right hardening for numerical conditioning of the covariance under these high-dynamics regimes; ship it behind a flag so it does not perturb the determinism baseline until validated [Rust no_std math determinism: docs.rust-embedded.org/book/intro/no-std.html].

### Build ranking (highest leverage first)

1. **Mission-fix S1+S2** — gate/flip mission authority to the estimated-state commander. Pure correctness; unblocks honest SIL.
2. **Propellant-depletion wiring (S3 payload)** — publish `PropellantState`, consume it, author `AtMassFraction` MECO. Three wiring steps; restores dispersion fidelity.
3. **End-to-end estimator `Failover`** through `MultiLaneEstimator` — implemented at lane-status/FDIR level; keep adding real dual-sensor scenarios.
4. **Hardware-timing evidence** — declared budget overruns are in Health/FDIR, but WCET/ISR/stack evidence needs PIL/HIL.
5. **Relative-navigation observable** — required before `AtRelativeDistance`/`AtRelativeSpeed` can become functional in SIL.
6. **Convex powered-descent guidance; AFTS-style corridor engine** — marquee differentiators, last, each with an explicit Tier-1 forward-only lock.

The foundational wiring items are now mostly implemented and covered by requirements. The remaining items are either hardware-bench evidence, unsupported observables, or research-grade capability. None require an input type that can express a target, and the convex-descent / corridor work must land only with a Tier-1 architectural lock (`docs/dual-use-assessment.md`), consistent with the project's forward-only posture.



---

# 3. Verification, SIL/PIL/HIL & Developer Tooling


This section designs the verification ladder and the developer-facing toolchain that make OpenBMP useful to *real-vehicle* GNC teams, not just to its author. The thesis from the industry research is blunt: a credible flight-software project is defined far less by its algorithms than by (a) the discipline that the *same source* runs across rungs of increasing realism, and (b) the reproducibility/traceability scaffolding around it. OpenBMP already has the structural pieces for the bottom rungs (the clean one-way SIL path, a byte-stable telemetry contract, and a CI suite that already enforces HAL portability and lockstep-clock discipline); this section turns those into an explicit, honestly-scoped ladder and a fork-runnable conformance suite.

A load-bearing constraint throughout: everything stays within the established dual-use posture. V&V acceptance metrics are kept in academic/analysis framing (insertion accuracy, attitude rates, dispersion ellipses about a *sample mean*), never miss-distance against an aimpoint. The conformance suite and HIL hooks are exactly where a fork could re-introduce prohibited capability, so they are governed by `docs/dual-use-assessment.md` and a dependency-graph check, not just by prose. The existing `verification.md` already forbids labels like `flight-qualified`/`certified`/`mission-ready` and self-frames against NASA-STD-7009B and AIAA G-077 as *vocabulary, not compliance* — this section extends that honest framing to the SIL/PIL/HIL claim.

---

### 1. The one codebase across MIL / SIL / PIL / HIL

The whole ladder rests on a single architectural invariant, confirmed across cFS, F´, PX4, and ArduPilot: **GNC/app logic never touches an OS primitive, a clock, or a peripheral directly; sim-vs-flight is a build/link selection, not a code change** (ArduPilot: "AP_Motor, AC_AttitudeControl, AP_NavController identical in simulation and on hardware"; PX4 lockstep; cFS OSAL/PSP). OpenBMP's reference SIL pattern already obeys this: truth → sensor (`crates/openbmp-runner/src/fc_bridge.rs:207-244`) → EKF/MEKF (`crates/openbmp-fc/src/estimator.rs`) → guidance on `PositionEstimate` (`guidance.rs:258-269`) → autopilot (`autopilot.rs`) → mixer, with the `Clock` trait injected (`crates/openbmp-fc/src/clock.rs`) and a typed pub/sub bus (`crates/openbmp-fc/src/bus.rs`) as the only inter-component coupling. Crucially, **the invariant is already CI-enforced, not aspirational**: `.github/workflows/ci.yml` ships a `hal-portability` job (`cargo build/test -p openbmp-fc --no-default-features --features std`) that fails if the controller imports any simulator-only crate (`openbmp-sim`, `-runner`, `-cli`, `-scenario`, `-telemetry`, `-bridge`, `-aerothermal`), a `lockstep-clock` tripwire (`cargo test -p openbmp-testkit --lib fc_lints`) that fails on any `std::time::Instant/SystemTime::now` inside `openbmp-fc`, and an `fc-hal-gate` job proving the test-only `ScenarioStateOverride` topic compiles out under `--features "std hal"`. That trio is the structural substrate the four rungs share.

| Rung | What it is for OpenBMP today | Same source? | What it uniquely catches | Infrastructure gap to close |
|---|---|---|---|---|
| **MIL** | The GNC *model* (guidance law, EKF, control law) run against the plant on host, f64, non-real-time. OpenBMP has no separate "model" layer — the Rust algorithm *is* the model. | n/a (no codegen step) | algorithm/math errors | Nothing structural; MIL collapses into SIL because there is no Simulink→C gap to bridge. Document this as a deliberate simplification, not a missing rung. |
| **SIL** | `FlightController` jobs run on host, closed-loop, on **FC-estimated** state, against the physics kernel as plant via `FcBridge`. The HAL/clock CI gates already pin its portability. | yes | code-vs-model divergence, language/impl bugs, SIL-vs-truth GNC behavior (EKF gate collapse, attitude drift) | Remaining gap is relative-navigation capability, not fail-closed behavior: `AtMassFraction` now consumes onboard `PropellantState`, FC-owned mission events are no longer truth-evaluated by the kernel, FC scenarios reject `IdealStateSensor`, and FC scenarios reject `AtRelativeDistance` / `AtRelativeSpeed` until an onboard relative-nav observable exists (`REQ-SIL-004`). |
| **PIL** | Cross-compile the flight-core crates (`openbmp-fc` + deps) to a target ISA (`thumbv7em-none-eabihf`, or QEMU/ISS), plant still simulated on host over the bus. **Scaffolded, not performed.** | yes (build target only) | compiler/word-size/FP effects on target; first real execution-time & stack/RAM numbers | Basic `no_std` target compilation is green. Real PIL still requires a target execution harness, compile-time/static topic dispatch replacing the `Box<dyn Any>`/`TypeId` registry, bounded mission HSM arenas (`crates/openbmp-mission/src/hsm.rs`), and timing/stack measurements. Until those land, PIL execution is aspirational and must not be advertised. |
| **HIL** | Real avionics running the *same* FC binary, real-time, with the **simulated plant** feeding sensors and absorbing actuator commands across the HAL/bus (PX4 HITL: same production firmware, sim sensors, blocked/looped actuators). **Scaffolded, not performed.** | yes | timing, drivers, interrupts, bus faults, EMI — the rung host-sim *cannot* verify | The socket bridge schema exists (`crates/openbmp-bridge/src/codec.rs`, postcard framing) with versioned hello, message envelope, and step/time validators; still needs a real transport loop, downstream board adapter, and hardware bench. The `real-rocket-integration.md` doc sketches the HIL adapter/cross-validation playbook. |

**Recommended honest claim (publish as a "verification-ladder status" table):** MIL = **N/A by design**; SIL = **yes for the implemented FC-owned mission path and onboard observables** (`AtMassFraction` included; relative-nav triggers explicitly unsupported under FC authority until an observable exists); PIL = **scaffolded** (cross-compile path green, execution harness absent); HIL = **scaffolded** (bridge schema + lockstep validators + feature gate exist; no real-time transport, hardware adapter, or reference board). Per `vnv-standards`, PIL/HIL need a target board and a real-time bench; the architecture *enables* them, and that is the defensible statement.

**Lockstep is mandatory for PIL/HIL determinism.** The existing `lockstep-clock` tripwire already guarantees no wall-clock leaks into `openbmp-fc`. Extend that contract: the sim-side clock advances only when the integrator produces the next state *and* the FC has consumed the prior sensor frame (PX4 "the simulator waits for PX4 before advancing simulated time"; ArduPilot `SITL_State`). This is what makes EKF/scheduler behavior replayable across the host sim and a HIL bench.

---

### 2. Standards posture, right-sized for a solo open project

The goal is to **signal flight-grade credibility honestly**, not to claim certification. The defensible self-classification (from `vnv-standards`, and consistent with the existing `verification.md` "External V&V Reference Frames" section):

- **NASA NPR 7150.2 Class D** (analysis/simulation, research-and-technology), **non-safety-critical** — cite Appendix D / SWE-020, document how a Class-A/B fork steps up. `verification.md` already self-classifies as academic and maps its four labels onto NASA-STD-7009B credibility factors; add the explicit NPR 7150.2 Class-D line.
- **DO-178C Level C/D *intent*** for the practices below — MC/DC + full independence named as the Level-A rung a flight fork would add, **not** something OpenBMP performs.
- **MISRA/Power-of-Ten *intent*** earned partly by Rust (memory safety, bounds checks, exhaustive `match`, `-D warnings` clippy gate, `#![forbid(unsafe_code)]` in `openbmp-fc`). Claim *intent alignment*, never rule-by-rule MISRA compliance.

| Practice | Adopt now (cost: low/solo-sustainable) | Aspire / fork-adds (cost: high) | Where it lives today |
|---|---|---|---|
| **Golden / regression tests** | Already present: `openbmp diff` golden scenarios + tolerance-table TOMLs (`crates/openbmp-sim/tests/expected/*.toml`). Generalize `compare-telemetry` as the documented *external* golden path. | — | `verification.md` Golden/Tolerance sections; `crates/openbmp-cli/tests/*_e2e.rs` |
| **Deterministic replay** | Already a merge gate: analytic-toy, Niskanen, Calisto, diff-flatness, and Phalcon-9 reference runs are byte-checked on the reference platform (`x86_64-unknown-linux-gnu`, MSRV pinned in `Cargo.toml`). | Cross-platform (ARM) bit-equality — *do not* claim it; needs shared-`libm` plus FMA/codegen control or fixed-point work. | `verification.md` Determinism Gate; `ci.yml` |
| **Coverage** | **Not yet gated** — `ci.yml` has no coverage lane. *Measure first* (`cargo llvm-cov`), then commit a line+branch number on the GNC/dynamics core and enforce it. | **MC/DC** + full independence (DO-178C Level A). Name it; don't gate on it. | new CI lane |
| **Requirements traceability** | Machine-readable enough for upstream: checked-in `requirements.toml` links stable requirement IDs to verification items, and CI runs `scripts/check_requirements_traceability.py` to reject orphan requirements, orphan verification records, missing paths, or missing evidence anchors. | DOORS-grade tooling. | `requirements.toml`, `scripts/check_requirements_traceability.py`, `ci.yml` |
| **Monte-Carlo dispersion** | Test-harness exists (`crates/openbmp-cli/tests/phalcon9_orbit_monte_carlo.rs`, SplitMix64 per-sample streams) + offline `footprint-mc`. Promote to a first-class runner with acceptance gates and pinned-seed CI artifacts. | Live in-loop dispersion filtering during guidance. | §3(c) |
| **Coding-standard rigor** | Already strong: clippy `-D warnings`, `cargo deny`/`audit`/`machete`, `cargo hack` feature-powerset, MSRV gate, `typos`, rustdoc `-D warnings`. Add explicit *bounded-loop / no-unbounded-recursion* and *assertion-density* conventions on the GNC core to close the Power-of-Ten gap cheaply. | Static-analyzer suite, WCET analysis. | `ci.yml`, `CONTRIBUTING.md` |

**The single most important honesty disclaimer** (cite the cFS precedent — NASA/CR-20205010026 found cFS reuse did *not* yield transferable assurance evidence): *publication and an upstream test suite do not confer a NASA software class or DO-178C credit; certification/classification is re-earned by the integrating fork for its specific system + toolchain.* This is what makes "fork-able into flight" honest rather than an overclaim. The existing `EXPORT-CONTROL.md` already places that burden on downstream HAL/real-data adopters; mirror it in a one-page `docs/standards-posture.md`.

**Newspace credibility, scaled down (from `vnv-standards`):** the part of SpaceX/Rocket Lab practice that scales to a solo open project is *massive automated simulation + CI + golden tests + Monte-Carlo + autonomy V&V* — not DO-178 paperwork. Lean into that. Tie the existing in-flight findings into the narrative as evidence the methodology catches *real* GNC failure modes (EKF GNSS innovation-gate collapse under thrust; off-pole/equatorial EKF-attitude out-of-plane drift; slosh-RCS coast attitude-hold closure) — **and distinguish true GNC findings from MODEL artifacts** (the equivalent-pendulum freefall translational drift is a model artifact, not a fixed GNC bug), so the posture doc does not overclaim a fix.

---

### 3. The developer toolchain, built on existing assets

The CLI surface is already the right shape (`run`, `diff`, `check`, `compare-telemetry`, `footprint-mc`, `check-provenance` per `crates/openbmp-cli/src/cli.rs`). The work is to *extend and connect*, not rebuild.

**(a) Extend `compare-telemetry` into a reusable external-validation campaign.** It already does the hard part: quarantined external reference (never imported into the repo or provenance — `docs/external-telemetry-validation.md` is explicit that the file is "not copied, not hashed into scenario provenance"), CSV/JSON/JSONL ingestion, per-metric mapping with `reference_scale`/`tolerance_abs`/`tolerance_rel`, a rich set of derived `actual.kind` observables (`altitude_from_position`, `surface_relative_speed`, `surface_relative_radial_velocity`, `dynamic_pressure`, `norm3`, etc.), the envelope `max(tolerance_abs, tolerance_rel * max(|ref|, relative_floor))`, and `--report-json` with `passed`/`failure_summary`/per-metric fields (`crates/openbmp-cli/src/commands/compare_telemetry.rs:30-60`). Add:
1. A `data/telemetry-mappings/` registry of pre-built mappings for common vehicle classes (sounding rocket, two-stage ascent, entry+recovery) so a fork compares against its flight test without hand-writing TOML.
2. A *campaign* artifact: reference-file hash + mapping + tolerance justification + result JSON, archived for reproducibility (the reference data itself stays external; only its hash and the mapping are committed — preserving the quarantine).
3. Optional per-dataset CI integration, kept *off* the merge gate, consistent with the deliberate quarantine.

**(b) Scenario-TOML authoring.** The format is already declarative, schema-versioned, and fail-closed (lint walks the AST pre-deserialization for `FORBIDDEN_SAFETY_TERMS`, unit suffixes, frame infixes — `crates/openbmp-scenario/src/lint.rs:18-61`). Add, in priority order:
1. `openbmp check --json` — machine-readable lint output (currently text-only), so IDEs get error squiggles and downstream tools can batch-lint.
2. A committed **template library** (`scenarios/templates/`): quick-start, sounding-rocket, two-stage ascent, entry+recovery, cold-gas RCS — each heavily commented.
3. **Model-registry introspection** enumerating available models + their `validation_status` (`experimental`/`checked`/`validated-toy`/`research` per `verification.md`) + allowed parameter ranges, so authors know what is `research`-grade vs `experimental`.

**(c) Monte-Carlo dispersion runner (new, forward-only).** Promote the harness in `crates/openbmp-cli/tests/phalcon9_orbit_monte_carlo.rs` (dispersed vehicles, SplitMix64 per-sample streams, fixed guidance) into a first-class `openbmp montecarlo` command:
- Inputs: declared uncertainty sources only (mass props, aero coefficients, winds/atmosphere, sensor noise, actuator lag, IC covariance) — the NASA dispersion taxonomy.
- Acceptance gates: insertion-accuracy bounds, attitude-rate limits, return-corridor bounds — **academic framing**.
- Outputs: pass-rate with N and pinned seeds; statistics **about the sample mean / nominal**, never against an aimpoint. Reuse the offline `footprint-mc` envelope semantics (`crates/openbmp-cli/src/commands/footprint_mc.rs`) but keep it forward-only.
- Guardrail: this is the second place targeting could creep in. Output fields must be range-relative (`downrange_m`, `crossrange_m`) or `*_from_nominal`; **no** `miss_distance_to_target` field can be expressible, and the `FORBIDDEN_SAFETY_TERMS` lint extends to any new field names.

**(d) HIL hooks.** Build on `openbmp-bridge` and the existing `hal_topic_gate.rs` / `fc-hal-gate` CI machinery:
1. Publish `docs/HAL.md` formalizing the adopter contract: `Clock` impl (hardware monotonic counter), `Sensor` impls per device, actuator-output model, storage model, time-sync semantics. The `Sensor` trait (`crates/openbmp-sensors/src/sensor.rs:171`, single `read()` method) and `Clock`/`Bus`/`Topic` traits are already the portable surface.
2. Keep the bridge lockstep contract (`BridgeHelloPacket`, `BridgeMessage`, `StepAckPacket`, `validate_command_for_sensor`) as the generic schema layer; downstream transports must enforce that the simulated plant advances only after a matching command or acknowledgement for the outstanding sensor frame.
3. A worked `real_hardware_main.rs` reference: implement `Clock` against a HW timer, `Sensor` against an off-the-shelf IMU, register the EKF/guidance/autopilot jobs, call `fc.step()` in a real-time loop, drive servos from `fc.latest_actuator_command()`. `docs/real-rocket-integration.md` already carries the socket-bridge flow and the `MissionStateAuthority` (Kernel vs FlightController) switch this depends on.

**(e) Conformance / acceptance suite a fork runs against its hardware.** This is the deliverable that makes OpenBMP a *dev toolchain* rather than a personal sandbox. Ship `openbmp conform` (or a `crates/openbmp-conformance` crate) that a fork runs *with its own HAL/board crate injected*:
1. **Trait-contract tests:** the fork's `Clock` is monotonic and lockstep-honest; each `Sensor::read()` returns well-formed measurements; actuator commands round-trip.
2. **SIL-parity test:** run a canonical scenario through the fork's board-in-the-loop and `compare-telemetry` against the host-SIL golden within a published tolerance band — the structural proof of "same code, same behavior."
3. **Determinism probe:** does the fork's build reproduce the reference trajectory state-stably?
4. **Dual-use dependency-graph assertion:** the fork's board/backend crate must NOT make any prohibited-capability crate reachable, and `openbmp-fc` must NOT depend on the simulator crate — an extension of the existing `hal-portability` CI job, now run *against the fork's graph*. This keeps a "real-hardware target" from quietly crossing the forward-only line (governed by `docs/dual-use-assessment.md`).

---

### 4. The determinism / reproducibility contract

**The contract (already partly stated in `verification.md`; tighten it):**
- **Bit-stable** output is guaranteed only on the **reference platform** (`x86_64-unknown-linux-gnu`, MSRV pinned in `Cargo.toml`, default profile, FMA/MXCSR-rounding excluded per the `openbmp-sim`/`openbmp-core` determinism notes). Enforced by the multi-scenario determinism gate and the `diff` tool, which reports first divergent row/channel/field, expected/actual, absolute and relative error, scenario hash, and determinism profile.
- **State-stable (tolerance-bounded), not bit-stable**, is the contract on other platforms and across the host→target boundary — the *pragmatic, correct* choice, matching the tolerance-table design. Bit-exact cross-platform replay would require a shared software `libm` and avoiding FMA contraction (a real, deferred cost). Say so plainly; claiming ARM bit-equality without that work would be an overclaim. `verification.md` already labels non-reference platforms "state-stable, not bit-stable."
- **Float precision:** telemetry exporters round-trip f64 losslessly (`{:.17e}`), so golden diffs detect sub-ULP drift on the reference platform.

**How SIL noise interacts with golden re-baselining — fire-time shifts are signal, not regression.** This is the subtle, important rule. Once the FSM is evaluated on **FC-estimated** state (gate kernel mission-event eval on `!fc_owned`; default `[fc]+[mission]` scenarios to `FlightController` authority), event fire times become a function of *estimator behavior* (sensor-noise realization, EKF convergence), not of integrated truth. Two consequences:

1. **A shifted fire time is a behavioral observation, not noise to be tolerated away.** If MECO/staging fires a tick later because the EKF velocity estimate crossed threshold later, that is a real, reproducible (for a fixed seed) property of the closed-loop system. `verification.md` already says "a changed golden file is treated as a behavioral change, not generated noise" — extend that *explicitly* to event fire times.
2. **Distinguish the two legitimate sources of a golden delta:** (i) *deterministic-seed* runs — fire times must be byte-identical run-to-run on the reference platform; any change is a regression to investigate. (ii) *dispersed Monte-Carlo* runs — fire times form a *distribution*; the gate is on the distribution (a staging-time band), and individual sample shifts within the band are expected. So the golden contract has two tiers: a deterministic-seed golden (exact) and a Monte-Carlo statistical golden (banded). Note the existing `state-stable` golden marking is the right hook: deterministic goldens stay default-profile/exact; MC goldens are the banded tier.

**Re-baselining discipline (ordered procedure):**
1. A failing golden diff first asks: *deterministic-seed run?* If yes and it changed, it is a regression — bisect, do not re-baseline.
2. If the change is an *intended* GNC/model change, regenerate the golden, and the PR must state which `requirements.toml` IDs it affects and why the fire-time/trajectory shift is correct (the traceability link makes the re-baseline auditable). This matches the existing rule that widening a tolerance requires a written justification.
3. For Monte-Carlo goldens, re-baseline the *band*, not the samples, and record N + seeds.
4. Never re-baseline to make a flaky test pass; flakiness on the reference platform means a determinism leak (a hidden wall-clock, an unseeded RNG, an FMA path) — fix the leak, which is itself a verification finding. The `lockstep-clock` tripwire already catches the wall-clock class of leak in `openbmp-fc`.

**Determinism provenance artifact:** the `diff` tool can emit a machine-readable record (scenario hash, git commit, Rust version, platform/profile, FP contract, archive metadata) alongside the first-divergence report. Collected over time this becomes a *determinism matrix* (platform × compiler × scenario) documenting, per environment, whether the claim is bit-stable or state-stable — reproducibility evidence that is itself a flight-grade signal and essentially free in a Rust/CI project.

---

### Sequencing (what unblocks what)

1. **SIL single-evaluator and propellant scalar: done for FC authority.** The remaining scalar decision is relative navigation: add an onboard observable or reject those triggers under FC authority.
2. **Determinism provenance artifact + two-tier golden contract: in place.** Extend it over more platforms as evidence accumulates.
3. **Monte-Carlo acceptance gates: visible.** Keep the nightly/manual MC lane as the heavier dispersion evidence path and avoid placing hardware-free academic MC behind real-time claims.
4. **Requirements traceability: in place; coverage remains open.** `requirements.toml` and the orphan checker now exist; a coverage CI lane is still optional future hardening.
5. **`no_std` target compilation: green; alloc-free flight hot path remains open.** Do *not* advertise bare-metal PIL execution before the static-bus/bounded-storage profile lands.
6. **`docs/HAL.md` + lockstep bridge + `openbmp conform`: in place as scaffolding.** HIL remains schema/contract scaffolding until a downstream board adapter and real-time bench exist.

> Honest bottom line: OpenBMP is a **MIL/SIL platform plus a reusable plant + scenario harness** that a fork carries up to PIL and HIL. The architecture *enables* the top two rungs — and already CI-enforces the HAL/clock discipline that makes them possible — but it does not *perform* them, and the standards posture is *alignment*, not certification. Every step above is sized to be sustainable by a solo author and is gated so that a "real-hardware target" can never quietly cross the forward-only line.


### External Cross-Validation against Real Flight Telemetry (local, non-integrated)

#### Why: a distinct rung on the V&V ladder

OpenBMP's committed verification stack proves *self-consistency*: golden/regression snapshots prove the kernel still reproduces its own prior output (no regression), and analytic checks (vis-viva, angular-momentum and energy conservation, toy closed-form drops) prove individual models obey first principles. Neither proves *external validity* — a simulator can be perfectly self-consistent and perfectly regression-clean while carrying a systematic modeling or implementation error (an atmosphere scale-height that is slightly off, a drag-deck Mach interpolation bug, an event-timing or mass-flow slip). Cross-validating a forward sim of a Falcon-9-class vehicle against *observed public flight behaviour* is the rung that catches this "slop." Ladder placement:

1. Golden / regression tests — self-consistency over time.
2. Analytic & conservation checks — model-level first-principles correctness.
3. **External cross-validation vs. real flight telemetry — gross-behaviour external validity (this subsection).**
4. MIL / SIL / PIL / HIL — progressive fidelity of the *execution substrate* (model-, software-, processor-, hardware-in-loop).

Rung 3 sits *above* golden/analytic checks (it tests against the real world, not against ourselves) and *beside* the MIL→HIL fidelity progression: it is a fidelity check on the *physics*, orthogonal to the substrate-fidelity progression of MIL/SIL/PIL/HIL, and it remains a MIL-level activity (no hardware, no flight code in the loop).

#### What: a coarse local webcast-telemetry reference

The reference is a developer-supplied **Falcon-9 webcast telemetry** trace: per-sample `time`, `altitude`, and `speed`, optionally a derived `dynamic_pressure`, plus an events list for anchoring. It originates from OCR of the on-screen webcast overlay (community tooling such as `shahar603/SpaceXtract` / `Telemetry-Data` is the canonical local scraper), so it is deliberately coarse:

- Only **speed and altitude are measured**; everything else (downrange, flight-path angle, dynamic pressure) is *derived* and is at best weak corroboration.
- Displayed speed is **ground/surface-relative, not inertial** — the offset is largest at MECO/SECO (up to ~0.46 km/s near-equatorial, less at higher latitude). Compare via `surface_relative_speed`, never inertial magnitude.
- Expect **OCR misreads, overlay-refresh latency, dropouts** (plume graphics, camera cuts, signal loss), irregular ~30 fps raw cadence, and **no quantified error bars**.
- Tools like `flightclub.io` are *forward simulators hand-tuned to the webcast*, not independent ground truth — do not promote them to golden status.

What it **validates** (gross behaviour / event timing): liftoff, max-Q timing, MECO/staging velocity, SECO, apogee/burnout magnitudes — typically to a few percent in speed at staging and low-single-digit percent in altitude/apogee. What it **cannot** validate: bit-level trajectory agreement, attitude, true downrange, or high-frequency dynamics (PID/slosh) masked by the coarse cadence.

#### How: a thin reference adapter over existing tooling

No new kernel code is required — this is a thin **Falcon-9 reference adapter** that reuses the existing `openbmp compare-telemetry` machinery (`crates/openbmp-cli/src/commands/compare_telemetry.rs`, documented in `docs/external-telemetry-validation.md`). The adapter is purely *alignment*:

- **Units / T0**: normalize the webcast to T+0 = ground ignition (verify against announced max-Q/MECO/staging/SECO milestones, *not* a countdown mark or sensor unlock); convert altitude km→m via `reference_scale = 1000.0`; set `time_min_s = 0.0` on metrics to reject pre-ignition samples (otherwise `skipped_samples` spikes and pass/fail misleads).
- **Frame (ground↔inertial)**: bind webcast velocity to the existing `surface_relative_speed` metric, which removes Earth rotation via `omega x r` (WGS84 `omega_rad_s = 7.2921151467e-5`), matching `phalcon9-orbit.toml`'s rotating frame; webcast altitude to `altitude_from_position` (`sqrt(x²+y²+z²) − radius_m`); optional q to `dynamic_pressure`.
- **Resampling / cadence**: pre-resample sparse webcast (<1 Hz) to ~5–10 Hz, or raise `time_tolerance_s` from the 0.25 s default to ~0.5–1.0 s, so the binary-search interpolator brackets reference timestamps against OpenBMP's 50 Hz output instead of skipping near gaps. Tolerate gaps/nulls.
- **Error metrics**: reuse the existing per-metric `CompareReport` (signed error `actual − reference`, `max_abs_error` with time/reference/actual, `rms_abs_error`, `exceedances`) and the envelope `max(tolerance_abs, tolerance_rel · max(|reference|, relative_floor))`.

Developer-only flow (produces the existing local JSON report; diagnostic only, never wired into CI or product):

```bash
openbmp compare-telemetry \
  scenarios/phalcon9/phalcon9-orbit.toml \
  reference/falcon9-webcast.csv \
  --mapping crossval/phalcon9-webcast-map.toml \
  --report-json out/falcon9-comparison.json   # local, gitignored output
```

#### Data hygiene (mandatory)

The webcast data is **local, gitignored, developer-supplied, and optional**. It is **not committed, not vendored, not hashed into scenario provenance, not redistributed, and is never a build / CI / test / product dependency**. The existing doc already enforces "read locally, not copied, not written to project output." Concrete layout:

- **DATA stays out of tree**: scraped trace lives in a gitignored `reference/` dir (or a sibling `openbmp-private-data/`), referenced only by an explicit CLI path argument. A sibling `reference/README` records provenance per file: flight name, source video URL, OCR date, and method — webcasts get re-uploaded/changed, and the upstream data is Unlicense over public-webcast frames while SpaceX has not licensed reuse, so this is treated as fair-use, local-only.
- **TARGET stays in tree, DATA does not**: the *cross-val intent* — the mapping schema, expected reference schema (`time_s`, `altitude_km`, `velocity_m_s`, optional `q`, `events`), and tolerance bands — may be committed (e.g. `crossval/phalcon9-webcast-map.toml` plus the documented schema). The reference rows themselves never are. Pattern after the synthetic E2E templates in `crates/openbmp-cli/tests/compare_telemetry_e2e.rs`, which exercise the same machinery with no external data.

#### Dual-use posture

This is a **comparison reference only** — our forward sim of a Falcon-9-class vehicle is scored against observed public flight behaviour *after the fact*; the telemetry is never a flight input, guidance target, or aimpoint, fully consistent with the forward-only / no-target-input Tier-1 lock.

#### Tolerance design & feedback loop

Tolerances must encode the coarseness: validate **trends and bands, not exact values**. Use generous, phase-aware envelopes — speed ~3% + a few hundred m/s (widen to ±5–10% through ascent/staging), altitude ~5% + km-scale floor, with event-keyed time windows anchoring max-Q/MECO/SECO/apogee. Treat the surface-relative offset (≤0.1% over ascent under uniform-rotation) as already absorbed by the frame conversion, not by the tolerance. **Failures feed back into model fixes, never parameter fitting**: a MECO/SECO divergence is a signal to audit the *general* atmosphere/aero-deck/propulsion/event-timing code (and to add a first-principles regression test once the defect is understood), not a license to tune `phalcon9-orbit.toml` constants to chase the curve. Fitting scenario parameters to one real flight is an explicit *bad use* and defeats the purpose of the rung.

---

# 4. Forward-Only Trajectory Subsystem & Dual-Use Governance


> **SAFETY-CRITICAL SECTION.** This subsystem sits closest to the dual-use line of anything in OpenBMP. Every design decision below is constrained by the project's existing fail-closed posture (`docs/dual-use-assessment.md`, `docs/safety-boundaries.md`, `docs/profile-vocabulary-and-guardrails.md`). The governing rule is simple and absolute: **the subsystem answers the forward question ("given this vehicle and this initial state, what trajectory results?") and is architecturally incapable of expressing or answering the inverse question ("what launch/guidance reaches this target?").** The boundary is enforced by *types*, not by prose.

### 1. Scope: what "forward-only trajectory" means here

The subsystem is the natural extension of work that already exists in the repository:

- **Phased forward propagation** of a vehicle-intrinsic scenario (mass/thrust/aero/structure + initial state + onboard flight program) to an output trajectory — the POST2/GMAT/ASTOS pattern of *advance the equations of motion through a sequence of segments until an event/stopping-condition fires* (`docs/scenario-format.md` event FSM; `openbmp-mission` `BuiltInEventTrigger`: `AtTime`, `AtAltitude`, `AtVelocity`, `AtApogee`, `AtMassFraction`). OpenBMP deliberately implements the *forward-simulation half* of those tools and **not** their targeting/optimize-to-a-condition half (see Non-Goals).
- **Free-flight / ballistic coast propagation** from a burnout/separation state that is itself an **OUTPUT** of the forward boost simulation — the existing `BallisticState` / `FootprintEnvironment` / `RangeSafetyFootprint` machinery in `crates/openbmp-physics/src/profile.rs:1222-1472`, documented in `docs/ballistic-coast-and-apogee.md`. The coast arc runs through apogee into drag-dominated descent governed by the **vehicle-intrinsic** ballistic coefficient `B = Cd·A/m`.
- **Monte-Carlo dispersion**: sample *declared* vehicle/environment uncertainties (wind, ballistic-coefficient variance, burnout-state covariance, mass/aero/thrust per NASA's uncertainty taxonomy) and propagate each draw forward to an **outcome distribution** — the existing offline `footprint-mc` CLI command and `[landing_footprint.monte_carlo]` block (`docs/scenario-format.md`; `crates/openbmp-cli/src/commands/footprint_mc.rs`). Statistics are reported **about the sample mean/nominal** (`cep50_m`, dispersion ellipse), never against an aimpoint.
- **Reentry/coast atmospheric drag**: density model interface (NRLMSISE-00-class) × vehicle-intrinsic `B`, gated by the existing apogee-vs-gravity-model validity-envelope consistency check (`docs/ballistic-coast-and-apogee.md`).

This is a published, peer-normal capability: RocketPy (open-source, MDPI Aerospace 2021,8,362) ships forward 6-DOF + landing-dispersion + recovery-planning under exactly this framing; NASA POST2/GMAT and ESA/ASTOS structure flights as phased forward integration to stopping conditions.

#### Recommended build (forward-only ingredients)

| # | Ingredient | Concrete basis | Status / action |
|---|-----------|----------------|-----------------|
| 1 | Phased segment driver, event-stopping | `openbmp-mission` triggers; GMAT `Propagate`-to-stopping-condition pattern | Exists; keep extending triggers, never add "reach condition X" inverse solve |
| 2 | Adaptive integrator (Dormand-Prince RK45/DOPRI, FSAL) + deterministic fixed-step mode | SciPy `RK45`; Hairer; existing fixed-step J2/EGM2008 footprint propagator | Adaptive as separately-validated mode; keep fixed-step default for byte-stable determinism gate |
| 3 | Gravity ladder (point-mass → J2 → zonal EGM2008) gated by altitude/apogee envelope | STK 2-body/J2/J4; Vallado Ch.8 (cited in `ballistic-coast-and-apogee.md`) | Exists; keep fail-closed envelope rejection |
| 4 | Atmosphere/density (NRLMSISE-00-class) → drag via vehicle-intrinsic `B = Cd·A/m` | NRLMSISE-00 (NASA CCMC); existing `AtmosphereModel`, `ballistic_coefficient` | `B` is a vehicle property, no target field — already on the right side |
| 5 | Forward Monte-Carlo over **declared** uncertainties → outcome distribution | NASA NTRS MC-dispersion (`20205010695`), X-43A 6-DOF MC (`TM-2007-214630`); RocketPy MC | Offline `footprint-mc` exists; output-only stats about sample mean |
| 6 | Tiered fidelity (3-DOF point-mass for fast coast/dispersion; 6-DOF for attitude-coupled) | ASTOS 3-DOF↔6-DOF mode switch | Both are *forward* fidelities; never port ASTOS/POST2 optimize-to-condition |

**Architectural placement (non-negotiable):** the propagator is callable **only** from offline/post-processing contexts (`openbmp-runner` footprint path, CLI commands), **never** from the `openbmp-fc` guidance/controller loop or a bus topic. This mirrors the existing rule that `LandingFootprintConfig` "is never consumed by the flight-controller loop" (`crates/openbmp-scenario/src/document.rs:2920-2955`). The separation of concerns is the lock: ascent guidance (`AscentReferenceGenerator`) steers toward an **inertial cutoff state** (radius + velocity), and at burnout an unpowered ballistic arc begins whose footprint is computed offline — neither component accepts a geographic coordinate.

### 2. "Ballistic missile trajectory prediction" — scoped strictly as forward propagation

The owner's mentioned item is admitted into the design **only** as the following and nothing more:

> **Forward ballistic propagation of a vehicle-intrinsic free-flight state.** Given a `BallisticState` (ECI position, ECI velocity, ballistic coefficient, timestamp) that is itself an OUTPUT of forward boost simulation, propagate the unpowered arc forward through apogee and atmospheric descent under gravity + drag, and report where the body goes. This is identical mathematics to projectile/sounding-rocket coast and re-entry — `docs/ballistic-coast-and-apogee.md`, `RangeSafetyFootprint` (`profile.rs:1459-1472`).

It is admitted because the *physics is the same* as range-safety footprinting OpenBMP already does, and because it is bounded by the same Tier-1 lock. What is **not** admitted is everything that turns prediction into engagement.

#### 2.1 NON-GOALS (architecturally-enforced, not merely unimplemented)

The following are **forbidden capabilities**. They are absent **because no input type can express them** (Tier-1), not because they are on a backlog. A PR that adds any of them violates the binding safety contract in `docs/safety-boundaries.md` and `ACCEPTABLE-USE.md`.

| Forbidden capability | Why it is impossible to express | Enforcement tier |
|----------------------|---------------------------------|------------------|
| **Target / impact point / aimpoint input** (lat/lon, ECEF, range, bearing as a *desired* destination) | No input struct has a desired-location field; `BallisticState` carries only position/velocity/`B`/time; `FootprintEnvironment` carries only gravity/cull-altitude/launch-origin/optional-dispersion | Tier-1 (type absence) + Tier-2 lint |
| **Inverse solve for launch parameters** ("what azimuth/elevation/ΔV reaches point P?") | There is no solver entry point that takes a target and returns launch parameters; the only direction is `scenario → trajectory` | Tier-1 (no such function exists) |
| **Guidance-to-target / terminal homing / seeker** | `AscentReferenceGenerator` targets inertial radius+velocity, never a ground location (`profile.rs:161-181`); guidance consumes only `InertialWaypoint` ECI checkpoints (`openbmp-fc/src/guidance.rs:46-97`) | Tier-1 + Tier-2 (`seeker`, `homing`, `terminalguidance` banned) |
| **Fire-control, range-table / firing-table generation** | No batched "sweep launch parameters → tabulate impact points keyed by range" capability; MC sweeps vehicle/environment **uncertainties**, never launch parameters indexed to range | Tier-1 + Tier-2 (`rangetable`, `firingtable`, `maxrange` banned) |
| **CEP / miss-distance against an aimpoint** | Output statistics are computed **about the sample mean / nominal** (`cep50_m` = empirical 50% radius about the MC sample mean); no field expresses error-to-target | Tier-1 (no aimpoint to score against) + Tier-2 (`missdistance`, `aimpoint`, `ballisticmatch` banned) |
| **Optimize-trajectory-to-reach-a-condition/location** (POST2/GMAT targeting half) | Only the forward-simulation half of POST2/GMAT/ASTOS is built; the targeting/optimization half is categorically refused | Tier-1 + documented Non-Goal |

> **Boundary-drift warning (from the governance dossier):** the single largest risk is a future contributor "just adding the optimizer" to match POST2/GMAT/ASTOS, framed with academic intent. *Documentation explains the constraint; it is not the constraint* (`dual-use-assessment.md:69-71`). Such a feature lands **only** when a Tier-1 type-level constraint binds it — which, for targeting/optimization, means it does not land.

### 3. Preserving and EXTENDING the Tier-1 lock to this subsystem

The Tier-1 doctrine — **"no input type can express a target"** (`dual-use-assessment.md:63`) — is already load-bearing in `profile.rs:1222-1472`. The subsystem **inherits and extends** it. Concretely, the API must make weaponization *unconstructible*, not discouraged:

1. **Input types carry only vehicle-intrinsic + environmental fields.** Replicate the proven shapes:
   - `BallisticState { eci_position, eci_velocity, ballistic_coefficient, timestamp }` — **no** target field (existing, `profile.rs:1222-1263`).
   - `FootprintEnvironment { gravity, cull_altitude, launch_origin, dispersion_input? }` — **no** desired-location field (existing, `profile.rs:1354-1375`).
   - Any new propagation-config struct (e.g. `BallisticPropagationResult`, `LandingState`) follows the same rule: **constructing a targeting input must be a type error.**
2. **Output schema is forward/relative only.** Emit `downrange_m` / `crossrange_m` / `bearing_rad` (range-relative), or optional geodetic `latitude_deg` / `longitude_deg` **only when a launch origin is declared** and **only labeled as predicted** (never "desired"). **Forbid** any `target_error_m` / `miss_distance_to_target_m` field. For MC, emit `cep50_m`, `radial_offset_from_nominal_m`, and `dispersion_ellipse` — scatter about the sample mean/nominal, never target-scored. Document the rationale in the field docstring per existing practice (`profile-vocabulary-and-guardrails.md` Rule 1).
3. **Trait signature makes targeting unconstructible.** `RangeSafetyFootprint` is consumed only by offline post-processing and is never on the control loop (`profile.rs:1459-1472`); the new propagator follows the same trait shape and the same `// This function is post-processing only; it is not called during flight-controller execution.` doc contract.
4. **Extend the Tier-2 lexical lint** (`crates/openbmp-scenario/src/lint.rs:18-61`, fail-closed, runs *before* deserialization with global priority via `ScenarioError::SafetyName`). The existing set already bans `aimpoint`, `missdistance`, `rangetable`, `firingtable`, `ballisticmatch`, `maxrange`, `intercept`, `terminalguidance`, `targetstate`, `targetlocation`, `desiredimpact`, `impactpoint`, `guidancetoimpact`. Add ballistic-specific guards only if new field names warrant them (e.g. `desired-impact`, `guidance-to-impact`, `terminal-prediction`, `accuracy-metric`). **Keep the list short — the absence of fields in the schema is the primary lock; the lint raises activation energy, it is not the wall.**
5. **Forward-not-inverse rule extended.** Document in `docs/profile-vocabulary-and-guardrails.md` a new fail-closed rule, e.g. *"The ballistic subsystem accepts only vehicle-intrinsic state (`position`, `velocity`, `ballistic_coefficient`) and environmental cull parameters; no desired impact location, guidance command, or accuracy objective is expressible. Output is range-relative or predicted-geodetic, never an error-against-target."* Add a row to the `dual-use-assessment.md` Residual Dual-Use Surface table acknowledging that ballistic state prediction is *operationally the same math as projectile ballistics* and is bounded specifically by the type-level absence of any target/aimpoint/accuracy-metric field.
6. **Fail-closed defaults:** non-negative `ballistic_coefficient` validation (existing `profile.rs:1251-1254`); reject a coast whose apogee exceeds the chosen gravity model's validity envelope (existing); MC over atmospheric uncertainty must treat space-weather inputs (F10.7, Ap) as **declared** uncertainty sources — no silent defaults that fabricate confidence.
7. **Scenario-consumer agreement (memory: fail-closed audit catch):** schema-level lock that **only** the offline footprint/post-processing block may consume `BallisticState`; **no** guidance/autopilot block may read it as a "target state." If ballistic state ever becomes reusable by another subsystem, this agreement must be re-asserted at schema level, not just by lint.

#### Tier-1 merge checklist for any PR in this subsystem
1. No field in any input struct expresses a target/range/aimpoint/desired-location.
2. The trait/function signature makes a targeting input **unconstructible** (type error, not runtime check).
3. Usage is offline/post-processing only — never reachable from `openbmp-fc` or a bus topic.
4. Output uses range-relative or predicted-geodetic labels; no `*_to_target_*` / `*miss_distance*` fields.
5. Lint rejects any newly-introduced operational vocabulary.
6. Doc comment explicitly states "forward-only" and "never targets a location."
7. `dual-use-assessment.md` + `profile-vocabulary-and-guardrails.md` updated with rationale and fail-closed rule.

### 4. Governance posture (awareness/policy level)

This is awareness-level synthesis of primary regulation, **not legal advice and not an export-control classification** — preserve the existing disclaimer in `EXPORT-CONTROL.md`. Legitimate open aerospace projects (NASA GMAT, released via `code.nasa.gov` after export-control review; RocketPy) stay compliant by relying on three well-established pillars, plus one structural truth:

| Pillar | What it is | How OpenBMP fits |
|--------|-----------|------------------|
| **EAR "published" carve-out** (15 CFR 734.7) | Software made publicly available without restrictions on further dissemination is *not subject to the EAR* (narrow encryption exceptions aside) | Openly-posted source, no encryption controls, no real controlled data → fits the published carve-out. This is a Commerce/EAR concept, **not** ITAR. |
| **Fundamental research** (15 CFR 734.8) | Public, unrestricted research | Consistent with the project's academic, public-data framing |
| **ITAR Cat IV / MTCR target *hardware*, not published physics** | USML Cat IV covers launch vehicles, guided/ballistic missiles, rockets; MTCR Cat I = complete systems ≥500 kg to ≥300 km, presumption of denial; both target delivery-system hardware, fielded technology, and operational technical data | A forward-only academic simulator with **no real vehicle data and no targeting** is far from the controlled-item definitions |

**The load-bearing compliance point (do not soften):** publication does **not** strip ITAR technical-data status the way it can for EAR. Therefore the durable strategy is to ensure the project is **never ITAR technical data in the first place** — i.e. the existing Tier-3 provenance gate ("no real fielded-vehicle / motor / TPS / operational data," `docs/data-provenance.md`, SHA-256 content pins) **is** the export-control control. Tie `dual-use-assessment.md` Tier-3 directly to `EXPORT-CONTROL.md`. A user inputting a real vehicle's ballistic coefficient from a controlled source could produce an operationally valid prediction; the architecture cannot prevent this — only provenance-checking and the documented adopter responsibility (real-vehicle ballistic data is as restricted as real-vehicle aerodynamic data) can.

**Primary, license-independent control:** architectural non-weaponization (Tier-1: no target input type exists) is the real wall; the published/fundamental-research framing is the compliance posture on top. A permissive license (Apache-2.0 / MIT) **cannot legally restrict downstream use** (`dual-use-assessment.md:126-139`); a fork can remove the lint and import real data, placing itself on its own export-control footing (`EXPORT-CONTROL.md:44-67`). Cite the peer cluster (GMAT post-review release, RocketPy's dispersion-ellipse-not-aimpoint framing) as evidence that a forward-only, no-real-data, no-targeting simulator is an accepted non-weaponized category. Keep all V&V acceptance metrics in academic/analysis framing (apogee, dispersion ellipse, return corridor) — never miss-distance-to-target — so the lint architecture and the governance posture reinforce each other.

### 5. Honest cost/risk summary

- **Cost is low for the forward half**: most ingredients exist (`BallisticState`, `RangeSafetyFootprint`, `footprint-mc`, gravity ladder, validity-envelope check). The work is consolidating them behind a clean propagator API, adding the Dormand-Prince adaptive mode (kept separate from the deterministic fixed-step gate), and a density/atmosphere interface — not net-new dual-use surface.
- **The risk is governance drift, not code**: the type-level locks are robust; the threat is a well-intentioned PR adding optimization/targeting "for academic completeness." The merge checklist (§3) and the Non-Goals table (§2.1) are the mitigations, backed by the doctrine that near-the-line capability lands *only* when a Tier-1 type constraint binds it.
- **Validation gap to close honestly**: a ballistic-descent propagator should be validated against public re-entry references (e.g. Allen-Eggers analytic, public Apollo/Gemini/X-15 cases) and tagged `validated-toy`/`research` per `docs/verification.md`, not asserted correct. An untagged propagator is `experimental` and must say so.
- **Geodetic-output trap**: predicted lat/lon must be unambiguously labeled "predicted landing point" in field names, report headers, and API docs — a downstream user must not be able to read it as a "desired target."
- **Determinism caveat**: adaptive RK45 step-size variability complicates bit-exact cross-platform reproducibility; keep a deterministic fixed-step option as the default for the CI determinism gate and treat adaptive stepping as a separately-validated mode.


### Dual-use hardening (review-mandated commitments)

An adversarial red-team review confirmed the forward-only posture is architecturally sound (weaponization is *unconstructible*, not merely discouraged) and mandated the following enforcement commitments, which are now binding checks in the repository:

- A mechanical type-field audit is implemented in `openbmp-testkit::dual_use_field_lints` and CI. It enumerates the public field sets of dual-use-sensitive scenario / footprint / propagator structs and fails when a field set changes without an explicit allowlist update and dual-use review.
- The output-not-input property of `BallisticState` is structural: its fields are private, the public constructor is provenance-named `from_forward_simulation`, and the reachability audit confines free-flight / footprint seed symbols to the physics definition and offline runner footprint path.
- Monte-Carlo uncertainty sources are constrained by typed scenario blocks for vehicle/environment dispersion inputs; launch-direction / aimpoint / target-error controls remain absent from the input surface, and sample-cloud output omits sampled burnout state, wind, and ballistic-coefficient rows.
- Predicted geodetic output is off by default and requires the explicit `landing_footprint.include_geodetic` opt-in plus a declared local origin; range-relative downrange/crossrange remains the default report.
- The dual-use CI assertions are distinct and required in repo governance: the dependency-graph/conformance checks block prohibited capability crates and modules, while `dual_use_reachability_tripwire` separately proves the offline free-flight / footprint propagator is not reachable from `openbmp-fc` control-loop sources.
- The scenario-consumer agreement is CI-enforced: `BallisticState` and free-flight seed symbols are allowed only in `openbmp-physics/src/profile.rs` and the offline runner footprint path, never in guidance, autopilot, commander, bridge, scenario-consumer, or sim control-loop code.



---

# 5. Roadmap & Phasing


This roadmap takes OpenBMP from a 19-crate, edition-2024, `std`-based 6-DOF SIL simulator into a fork-able, flight-portable GNC core verified on an industry-standard MIL→SIL→PIL→HIL ladder. The first tranche is already implemented: FC-owned mission authority, HAL/static-bus scaffolding, requirements traceability, determinism/MC lanes, and alloc-aware target compilation are in CI. Remaining milestones focus on alloc-free hardening, PIL/HIL execution, hardware timing evidence, and research-grade GNC capability. **No milestone weakens a Tier-1 dual-use lock; several strengthen the CI dependency-graph enforcement of it.**

### Guiding sequencing principles

1. **Correctness before portability.** The SIL truth/estimate split is closed for FC-owned scenarios; keep that single-evaluator contract protected while carving the flight core out, so the code that gets lifted stays the commander-owned estimated-state path rather than a truth-fired test shortcut.
2. **Portability before hardware.** Extract HAL traits and the `no_std` core *before* attempting PIL/HIL, because PIL is just "cross-compile the same core" and is cheap *once the core compiles `no_std`* and expensive otherwise.
3. **Each near-the-line capability lands only behind a Tier-1 binding** (per `docs/dual-use-assessment.md:69-71`). The forward-trajectory and FDIR milestones each ship with their structural lock and an *extended* `FORBIDDEN_SAFETY_TERMS` list and CI dependency-graph assertion.
4. **Tolerance-bounded, not bit-exact, is the cross-platform contract** (the existing `compare_telemetry` + determinism gate already imply this; PIL/ARM cannot be bit-exact — see `crate-map` risks on FMA/libm divergence).

---

### Milestone table

| # | Milestone | Goal (one line) | Key crates / files | V&V gate (the proof) | Effort (solo) | Risk | Depends on |
|---|-----------|-----------------|--------------------|----------------------|---------------|------|------------|
| **M0** | **Baseline lock-in** | Make today's behavior a guarded contract before refactoring | `.github/workflows/ci.yml`; existing golden Parquet baselines (phalcon9-orbit, diff-flatness) | Golden-regression CI gate runs on *all* committed baselines, not just analytic-toy (closes the silent-drift gap in `tooling-vnv` risks) | S | Low | — |
| **M1** | **Sequencer S1 — gate kernel mission-eval on `!fc_owned`** | Stop the kernel from firing mission events on integrated truth when the FC owns the phase | `kernel.rs:667-681,:1129`; `rigid_body.rs:471` (`pending_mission_fired`) | New e2e: FC-authority scenario shows *zero* truth-fired non-transition actions (Stop/EmitTelemetryMarker/RaiseHealthAlarm); golden telemetry unchanged for Kernel-authority scenarios | S | Low–Med | M0 |
| **M2** | **HAL trait extraction (begin)** | Promote the implicit HAL (Clock/Sensor/Bus/Job) into an explicit `openbmp-hal` trait crate + CI dependency-graph lock | new `openbmp-hal`; `clock.rs`, `bus.rs`, `topics.rs`, `openbmp-sensors/src/sensor.rs:171`; `hal_topic_gate.rs` | CI asserts `openbmp-fc` does **not** depend on `openbmp-sim`/`openbmp-runner`/`openbmp-scenario` (cargo-tree graph check); `hal_topic_gate` extended to the full sensor/actuator trait surface | M | Med | M0 (parallel to M1) |
| **M3** | **Sequencer S2 — default `[fc]`+`[mission]` to FlightController authority** | Flip the default so flight decisions run on EKF-estimated state; truth-eval becomes an opt-in `scenario_director` | `kernel.rs:493` (`MissionStateAuthority::Kernel` default), `commander.rs:153-201`; `with_mission_split` | Pure-sim scenarios still pass golden gates; a SIL-correctness e2e proves mission transitions fire on estimated, not truth, state | M | Med | M1 |
| **M4** | **Sequencer S3 — close estimated-scalar gaps** | `mass_fraction` is populated from onboard `PropellantState`; relative dist/speed are rejected under FC authority until an onboard observable exists | `commander.rs`; `openbmp-runner/src/fc_bridge.rs`; `openbmp-scenario/src/document.rs`; nav/relative-nav | `AtMassFraction` functional in SIL; `AtRelativeDistance`/`AtRelativeSpeed` fail closed at scenario validation for FC authority | M | Med | M3 |
| **M5** | **Sequencer S4 — single evaluator** | Kernel keeps *only* plant-side guards (ground impact, NaN, sim-end fences); one mission interpreter | `kernel.rs:667-682`; `commander.rs`; `kernel.rs:920` (script-events lane stays on truth — legitimate test stimulus) | Determinism gate green; no behavioral diff vs M4 baselines except the removed duplicate evaluator; `IdealStateSensor` CI lint flags any truth-echo escape hatch | M | Med | M4 |
| **M6** | **Flight-core `no_std` carve-out** | `openbmp-fc` and shared flight-side crates compile `#![no_std]` on a target triple; alloc-free hot path remains a follow-on hardening gate | `openbmp-core`, `openbmp-state`, `openbmp-models`, `openbmp-fc`; `bus.rs` (`Box<dyn Any>`/`TypeId`), `scheduler.rs`, `params.rs`/`tables.rs`, commander collections | CI: `cargo check -p openbmp-core -p openbmp-state -p openbmp-models -p openbmp-mission -p openbmp-physics -p openbmp-propulsion -p openbmp-aero -p openbmp-sensors -p openbmp-hal -p openbmp-fc --no-default-features --target thumbv7em-none-eabihf` is green; `libm` pinned for target math | **L (largest)** | **High** | M2, M5 |
| **M7** | **PIL enablement** | Cross-compile the unchanged core to a target CPU (or ISS); plant stays on host | `openbmp-gnc-core`; new `openbmp-pil-harness` (reuses `compare_telemetry`) | Tolerance-bounded telemetry match between host SIL and on-target PIL run over a reference scenario; execution-time + stack measurements recorded | M | Med | M6 |
| **M8** | **Monte-Carlo + conformance tooling** | Promote the existing MC harness to a seeded, gated dispersion campaign with acceptance criteria | `phalcon9_orbit_monte_carlo.rs`, `footprint_mc.rs`; `compare_telemetry.rs`; new nightly CI lane | Nightly seeded MC with published pass-rate, N, pinned seeds as artifacts; public-benchmark wrappers (Niskanen/Estes) move from "nightly only" prose to a real CI lane | M | Low–Med | M0 (S1 strengthens it; MECO-on-depletion at M4 restores fidelity) |
| **M9** | **Forward-trajectory subsystem (forward-only)** | Phased forward propagator + ballistic coast + adaptive RK45, offline-only | `openbmp-physics/src/profile.rs:1222-1472` (`BallisticState`/`FootprintEnvironment`/`RangeSafetyFootprint`); `footprint.rs` | Validated-toy gate vs public re-entry/coast cases; lint `FORBIDDEN_SAFETY_TERMS` extended; CI asserts propagator is **never** reachable from `openbmp-fc` guidance/controller loops | M | Med (dual-use) | M0 (independent of HAL line) |
| **M10** | **FDIR / fault management** | First-class meta-loop: watchpoint/actionpoint (cFS-LC style), heartbeat/health, phase-gated AFTS-style corridor evaluator | `fdir.rs`, `health.rs`, `glrt.rs`, `voter.rs`, `estimator_lanes.rs` | Fault-injection e2e ("cut the strings") shows safe-state response; redline corridor evaluator is forward-only (no aimpoint), lint-guarded | M–L | Med–High (dual-use line near AFTS) | M5 (single evaluator), M6 (core stable) |
| **M11** | **HIL scaffolding** | Document and scaffold the real-time bench contract; *scaffold, not perform* | `openbmp-bridge`, `fc_bridge.rs`; `docs/real-rocket-integration.md` | Lockstep socket-bridge demo (sim sensors → external FC → commands → sim); **honestly labeled "scaffolded," timing/EMI/ISR explicitly out of scope** | M | Low (but honest scope) | M7 |

Effort key: **S** ≈ days, **M** ≈ 1–3 weeks, **L** ≈ 1–2 months solo. These are *focused-time* estimates for a sophisticated solo author; calendar time will be longer.

---

### Per-milestone notes

- **M0 (Baseline lock-in).** Cheapest, highest-leverage safety net and now in place for multiple scenarios. The determinism gate byte-checks analytic-toy, Niskanen, Calisto, diff-flatness, and Phalcon-9 reference runs on the reference platform, so subsequent refactors are reviewed against guarded baselines rather than silent drift.

- **M1 (S1).** The single most important *correctness* fix. Gating kernel mission-eval on `!fc_owned` closes the drift where truth-fired non-transition actions leak via `pending_mission_fired` even when `fc_owned` blocks transition application. Small, well-scoped, and it is the reference for the rest of the sequencer migration.

- **M2 (HAL extraction).** This is the *structural* enabler for everything flight-portable, and it can run **in parallel with M1** because it touches the trait seams, not the FSM. The deliverable is twofold: (a) an `openbmp-hal` crate holding `Clock`/`Sensor`/`Bus`/`Job`/actuator traits (modeled on `embedded-hal` + AP_HAL's interface list), and (b) a **CI dependency-graph assertion** that `openbmp-fc` cannot reach `openbmp-sim`/`openbmp-runner`/`openbmp-scenario`. That assertion is also a dual-use control (it stops a board layer from re-importing prohibited capability — `fsw-cfs` risk).

- **M3–M5 (S2–S4).** The sequencer endgame. S2 flips the default authority; S3 fills the estimated scalars (and, critically, makes booster MECO fire on *estimated propellant depletion*, which restores Monte-Carlo dispersion realism — this is why **M8 strengthens after M4**); S4 collapses to one evaluator, leaving the kernel with only plant-side fences. The script-events lane (`kernel.rs:920`) legitimately stays on truth as test stimulus.

- **M6 (`no_std` carve-out).** The **highest-risk, highest-effort** milestone. The first target check is now green: `openbmp-fc` builds for `thumbv7em-none-eabihf` with default features disabled, alongside the shared math/HAL crates. The remaining blockers are narrower and concrete: the current FC path is still `alloc`-aware, with `Box<dyn Any>`/`TypeId` registries (`bus.rs`, `params.rs`, `tables.rs`), `RefCell`, dynamic linear algebra in SR-UKF/trajectory, and unbounded `BTreeMap`/`Vec`/`String` collections. Mitigation remains the staged ladder from `crate-map`: refactor the FC topic registry to a const-generic / enum-dispatch table, then bound the mission/scheduler collections (`heapless`/arena), then measure stack/RAM. Pin `libm` on both host and target so the sim remains a valid cross-check (`rust-flight`). Do **not** attempt bare-metal RTOS here; the realistic next target is alloc-free hot-path compilation, with embedded-Linux as the first *real-board* milestone later.

- **M7 (PIL).** Cheap *if and only if* M6 succeeded — PIL is "cross-compile the same source and re-run the SIL scenario," reusing `compare_telemetry` as the conformance harness with tolerance bands (not bit-exact, because ARM NEON vs x86 SSE and libm differences break bit-equality — `rust-flight`/`crate-map` risk).

- **M8 (MC + conformance).** Largely *existing* machinery (`phalcon9_orbit_monte_carlo.rs`, `footprint_mc.rs`, SplitMix64) promoted to a gated nightly campaign with published N/seeds/pass-rate. This is the newspace-credible headline V&V activity and scales down to a solo project.

- **M9 (forward-trajectory).** Independent of the HAL/flight-core line — can be slotted whenever. Build only the *forward* half of POST2/GMAT/ASTOS (phased segment driver, adaptive Dormand-Prince RK45, gravity ladder, ballistic coast from a burnout state that is itself a forward output). Ship with the Tier-1 lock intact (`profile.rs:1222-1472` already has no target field), an extended lint list, a **prominent NON-GOALS subsection**, and a CI assertion that the propagator is unreachable from any control loop.

- **M10 (FDIR).** Near the dual-use line (AFTS), so it lands *after* the single evaluator (M5) and a stable core (M6), and only behind its Tier-1 binding: the corridor/redline evaluator accepts only vehicle state + environmental fences, never an aimpoint, and emits a safe-state/terminate action, never a guidance-to-location command.

- **M11 (HIL scaffolding).** Deliberately last and **honestly labeled "scaffolded, not performed."** Host-passing does not prove real-time/ISR/EMI correctness; that is a hardware-bench concern a fork owns.

---

### Smallest high-leverage first milestone

**M0 + M1 + the *start* of M2**, shipped as one PR series:

1. **M0:** Promote the existing golden Parquet baselines into the merge-gate determinism check.
2. **M1 (S1):** Gate kernel mission-eval on `!fc_owned`; close the `pending_mission_fired` truth-leak.
3. **M2 (begin):** Create `openbmp-hal` with the *Clock + Sensor* traits already implicit in `clock.rs`/`openbmp-sensors/src/sensor.rs:171`, and add the **CI dependency-graph assertion** that `openbmp-fc` does not depend on the simulator crates.

This is the smallest slice that (a) is independently shippable, (b) fixes a real correctness bug (truth leak), (c) lays the load-bearing portability seam (HAL crate + dependency lock), and (d) *strengthens* a dual-use control (the dependency-graph assertion) rather than touching anything near the line. It is achievable in roughly **1–2 focused weeks solo**.

#### Definition of done — "fork-able into a real flight computer"

A fork can lift the GNC core onto real hardware when **all** of the following hold:

1. **Compiles flight-side.** `openbmp-fc` and the no_std-clean shared/math/HAL crates build for `thumbv7em-none-eabihf` with default features disabled in CI (M6); alloc-free hot-path/static-bus work is the next hardening gate before any bare-metal claim.
2. **Zero simulator coupling.** A CI dependency-graph assertion proves the flight core does not depend on `openbmp-sim`, `-runner`, `-scenario`, or any host-only crate (M2).
3. **Two topologies, one core.** There exist exactly two wirings — `sim_topology()` and a documented `flight_topology()` reference — differing *only* in injected backend (`fsw-cfs` structural guarantee). A worked `real_hardware_main.rs` example exists (`fsw-boundary` recommendation).
4. **Single, correct evaluator.** Mission/event decisions run on estimated state only; the kernel holds only plant-side fences (M5). No `IdealStateSensor` truth-echo path is reachable in a flight build (CI lint).
5. **Conformance harness.** PIL cross-compiled output matches host SIL within published tolerance bands (M7), and the same `compare_telemetry` contract is the cross-validation tool a fork uses against its own bench.
6. **Dual-use locks intact and inherited.** The Tier-1 "no input type can express a target" lock, the `FORBIDDEN_SAFETY_TERMS` lint, and the dependency-graph assertion are all enforced on the flight-core and any board crate; `docs/dual-use-assessment.md` and `EXPORT-CONTROL.md` state that certification and export footing are re-earned by the integrating fork (cFS precedent — `vnv-standards`).
7. **Honest scope statement.** Docs assert MIL=yes, SIL=yes, PIL=scaffolded/demonstrated, HIL=scaffolded — *not* certified, *not* real-time-proven.

---

### Risk register

| Risk | Type | Likelihood / Impact | Mitigation |
|------|------|---------------------|------------|
| Alloc-free carve-out (M6 follow-on) balloons: `Box<dyn Any>`/`TypeId`/`RefCell` refactor in `bus.rs`/`scheduler.rs`/`params.rs` is deep | Technical | High / High | Stage it (`crate-map` ladder): keep target compilation green; const-generic/enum topic table next; bound collections last. Ship M6 as several sub-PRs each behind a `cargo check --target` CI gate. Accept embedded-Linux (`std`-on-board) as an interim if bare-metal stalls. |
| Cross-platform determinism breaks (ARM NEON/libm/FMA ≠ x86) | Technical | High / Med | Adopt **tolerance-bounded** as the explicit contract (not bit-exact); pin one shared `libm` on host and target; keep the bit-exact determinism gate scoped to the x86 reference platform only, as today. |
| Sequencer migration (S1–S4) silently changes golden behavior | Technical | Med / High | M0 first — full golden gate makes every S-step provably behavior-preserving; treat any golden delta as a reviewed behavioral change, not noise. |
| Estimated-scalar gaps (M4) lack an observable (e.g., true relative-nav) | Technical | Med / Med | Fail-closed where no observable exists (per the established calculate-and-isolate discipline); do not fabricate a scalar. Scope relative-nav as its own sub-task; MECO-on-depletion uses already-existing `consumed_kg`. |
| FDIR corridor evaluator (M10) drifts toward AFTS/aimpoint | **Dual-use** | Low / Critical | Land only behind Tier-1 binding: state-and-fence inputs only, safe-state/terminate output only; extend `FORBIDDEN_SAFETY_TERMS`; CI assertion it is unreachable from guidance; document in `dual-use-assessment.md` residual-surface table. |
| Forward-trajectory subsystem (M9) tempts a contributor to "just add the optimizer" (POST2/GMAT/ASTOS targeting half) | **Dual-use / Scope** | Med / Critical | Implement forward half only; prominent NON-GOALS subsection; Tier-1 lock (no target field in `BallisticState`); range-relative output only (`downrange_m`/`crossrange_m`, never `miss_distance_to_target`); MC stats about the sample mean, never an aimpoint. |
| Board/backend layer (future fork seam) re-imports prohibited capability | **Dual-use** | Low / Critical | The M2 CI dependency-graph assertion is extended to assert no prohibited-capability crate is reachable from any board target; governed by `docs/dual-use-assessment.md`. |
| Solo-author scope creep across 11 milestones; over-claiming "done" | Scope | High / Med | Each milestone is independently shippable with its own V&V gate; never mark a milestone done while a blocker stands (per the no-overstating discipline); HIL/PIL labeled "scaffolded," DO-178C labeled "architectural readiness," not certified. |
| HIL/real-time fidelity assumed proven by host-passing | Scope | Med / Med | Explicit doc disclaimer: timing/ISR/EMI/WCET are HIL-rung concerns OpenBMP cannot verify; a fork must not assume host-passing implies real-time-safe (`vnv-standards`/`fsw-cfs` risks). |

**Dual-use sequencing invariant:** at every milestone the Tier-1 lock ("no input type can express a target"), the `FORBIDDEN_SAFETY_TERMS` parse-time lint, and the dependency-graph CI assertion remain enforced. M2, M9, and M10 each *extend* these controls; **none weakens them.** This is the structural guarantee that the roadmap stays forward-only from current state through the vision.



---

# Appendix A — Key design decisions


### Target Architecture & Flight Portability

- **Re-layer the workspace into four strata (L1 SHARED contracts, L2 FLIGHT-CORE no_std-capable, L3 SIMULATOR/PLANT host, L4 TOOLING host) with a single hard invariant: no L2 crate may reach L3/L4 in the dependency graph.** — This is the universal sim-vs-flight pattern across cFS (Apps/cFE/OSAL/PSP), F´ (component/Os/Drv), ArduPilot, and PX4 — application/GNC source byte-identical across targets, sim-vs-flight is a build selection. It makes openbmp-fc liftable into a fork and is enforceable in CI. _Trade-offs:_ Requires splitting openbmp-physics/-propulsion/-aero/-mission along a no_std math vs host-parser line, and creating two new crates (openbmp-hal, openbmp-fc-sim). The physics, propulsion, and aero core split lines now have target checks; mission-interpreter arenas and the FC bus/registry split remain.
- **Introduce openbmp-hal as an embedded-hal-style L1 trait crate with a deliberate two-axis split (runtime/Clock vs board/peripherals), reusing the existing Sensor trait and Clock abstraction as seeds.** — cFS keeps OSAL ('which OS') separate from PSP ('which board') precisely so a fork can add a board without touching the runtime; ArduPilot's AP_HAL interface list is a ready checklist. The Sensor trait (sensor.rs:171) and clock.rs are already the right shape. _Trade-offs:_ Trait granularity risk (fragmentation / vendor leakage); designing traits realistic for real IMU/GNSS yet cleanly kernel-drivable needs iteration. No real port exists to validate sufficiency.
- **Make the message bus an abstract trait with a HostBus (today's RefCell/Box<dyn Any>/Rc/TypeId impl, L3-acceptable) and a StaticBus<N> (compile-time topic enum, zero heap, L2 flight impl).** — bus.rs already has correct uORB-like semantics (anonymous, latest-value, timestamped) but the heap/TypeId implementation is the chief openbmp-fc portability blocker. Trait + two impls de-heaps the flight path while keeping the convenient dynamic bus for sim. _Trade-offs:_ Const-generic/enum dispatch changes binary size and branch prediction; the topic enum must be generated and kept in sync. Performance characterization needed before bare-metal deployment.
- **Target Linux-on-a-real-board (HAL_Linux/PX4-on-POSIX path) as the FIRST liftability milestone; bare-metal RTOS (RTIC, Hubris-style static alloc) later. Develop on stable Rust, treat Ferrocene as a future drop-in.** — OpenBMP is std-based today; partial liftability on a Linux flight board reaches real hardware with the least no_std rework. Ferrocene tracks pinned upstream rustc so the same source compiles when a qualified toolchain is needed. _Trade-offs:_ Linux-board does not exercise bare-metal no_std discipline or hard-real-time ISR/EMI behavior; those remain unverified until the RTIC milestone and a HIL bench.
- **Frame OpenBMP as 'qualification-ready architecture, not qualified,' and contract 'identical sim/flight behavior = tolerance-bounded, not bit-exact' (sharing a pinned libm across host and target).** — There is no turnkey DO-178C Rust certificate; cFS precedent shows open releases do not confer transferable certification. f64/libm/FPU differences across host and ARM break bit-exact replay; the existing compare_telemetry tooling already implies tolerance-bounded. _Trade-offs:_ Tolerance-bounded cross-validation is weaker than bit-exact; strength depends on disciplined shared-libm or fixed-point choices, which add engineering cost. DO-178C readiness must be hedged as architectural, not certification.

### The GNC Stack

- **Mission FSM single-evaluator on ESTIMATED state behind the sensor boundary is now the FC-owned contract; the kernel keeps truth evaluation for kernel-authority simulation and scenario-director stimulus.** — This matches how a real flight computer sequences a mission while preserving pure-sim workflows. _Trade-offs:_ Relative-nav trigger families still need an onboard observable or a parse-time unsupported-mode policy; they must not fall back to truth.
- **Treat the TOML mission graph + EventBinding thresholds + HSM transition table as the flight-software I-load tables; keep them declarative. EventScalars + BuiltInEventTrigger::fired stay the shared interpreter. Only WHO builds the scalars and from WHAT state changes.** — This reframe converts the sequencer fix from a rewrite into a wiring change: every declarative table is preserved, mirroring the cFE TBL-service / ArduPilot SIM_* runtime-loadable-parameter discipline. _Trade-offs:_ Requires the same I-load discipline be extended to redlines and gains (lift compile-time *Config structs into runtime tables), which is extra plumbing but is the right long-term shape.
- **The propellant-depletion-cutoff path is wired: the FC bridge publishes `PropellantState` from tank state, the commander consumes `mass_fraction`, and `AtMassFraction` is covered by tests.** — Real MECO is depletion-driven, which is the dominant source of ascent dispersion. _Trade-offs:_ Scenario authors still need to move cutoff events onto `AtMassFraction` where that fidelity matters.
- **Build on the existing FDIR/redundancy scaffolding rather than designing new: end-to-end the existing Failover path through MultiLaneEstimator first; scope OpenBMP as redundancy-management-READY, not fault-tolerant avionics.** — fdir.rs already has FdirState/FaultClass/FdirResponse, the FdirCheck trait, four checks, debounce; estimator_lanes.rs already has MultiLaneEstimator with LaneSelection, switch_margin, and dwell. The dual-IMU/GNSS lane-voting case (ArduPilot EKF3 pattern) is the real launch-vehicle need and is mostly coded. _Trade-offs:_ True Byzantine/cross-strapped HW fault tolerance is a hardware problem a software-only stack cannot deliver; must be stated honestly to avoid overclaiming.
- **Scheduler cadences and declared budgets are now declarative, and declared-budget overruns feed HealthMonitor/FDIR.** — The cFS SCH / F-Prime RateGroupDriver pattern is present at the SIL topology level. _Trade-offs:_ Declared budget is not WCET; host sim cannot reproduce ISR jitter or board timing, so hardware timing remains a PIL/HIL concern.
- **Address the two known observability blockers architecturally: phase-driven GNSS process-noise scheduling using the mission-FSM phase signal (existing high_dynamics_q_scale knob), and treat under-observable attitude as an FDIR Estimator-class annunciation plus lane preference, not a claimed fix.** — Estimator already exposes gate_chi2 and high_dynamics_q_scale (estimator.rs:144,146); making the single estimated-state FSM the source of the boost/coast phase signal turns a per-scenario hand-tune into a declarative schedule. The off-pole attitude drift is an observability gap, not a tuning bug. _Trade-offs:_ Square-root EKF hardening should ship feature-gated so it does not perturb the determinism baseline until validated; the equatorial attitude case stays a documented sensor-geometry limitation, not a solved item.

### Verification, SIL/PIL/HIL & Developer Tooling

- **Treat MIL as N/A-by-design and collapse it into SIL; claim SIL as the only 'today' rung, with PIL and HIL explicitly labeled 'scaffolded, not performed.'** — OpenBMP has no Simulink->C auto-coding gap (the Rust algorithm IS the model), so MIL and SIL are the same artifact. The vnv-standards dossier is explicit that PIL needs a target CPU and HIL needs a real-time bench with real I/O; neither exists. Overclaiming would fail with expert readers and violates the no-overstating-blockers rule. _Trade-offs:_ Looks less complete than a four-rung 'all supported' claim, but honesty is the credibility asset; a fork reads the scaffolding status and knows exactly what it must add.
- **Anchor the SIL-portability claim to CI jobs that ALREADY exist (hal-portability, lockstep-clock tripwire via openbmp-testkit fc_lints, fc-hal-gate) rather than proposing them as new.** — Reading .github/workflows/ci.yml showed the one-codebase invariant is already mechanically enforced: openbmp-fc must build with synthetic defaults disabled (`--no-default-features --features std`) without importing any sim crate, and no wall-clock API may appear in openbmp-fc. This is the strongest current evidence for the 'sim-vs-flight is a build selection' claim while the true bare-metal FC carve-out remains M6 work. _Trade-offs:_ None for accuracy; it does mean the section must not double-propose these as future work, which I corrected.
- **Make 'event fire-time shifts under SIL are signal, not regression' an explicit two-tier golden contract (deterministic-seed = exact; Monte-Carlo = banded), mapped onto the existing `state-stable` golden marking.** — Once the FSM evaluates on FC-estimated state, fire times become an estimator property (EKF convergence, sensor noise). verification.md already declares a changed golden a 'behavioral change, not noise' and supports a state-stable marking; the two-tier split operationalizes that for closed-loop SIL without silent re-baselining. _Trade-offs:_ More machinery than a single golden tier and depends on reviewer discipline for re-baselining, but it directly reuses existing verification.md rules rather than inventing new ones.
- **Keep coverage as the remaining unmeasured V&V metric; requirements traceability is now machine-readable and gated.** — `requirements.toml` plus `scripts/check_requirements_traceability.py` make every in-repo requirement point at verification evidence, and CI lists `requirements traceability` as a required gate. The vnv-standards dossier still applies to coverage: publishing an unmet threshold is worse than a modest enforced one, so measure with `cargo llvm-cov` before committing a number. _Trade-offs:_ Cannot present a concrete coverage threshold yet; the traceability gap is closed, while coverage remains deliberately measure-first.
- **Build the dev toolchain by extending existing assets (compare-telemetry quarantine, scenario lint, the phalcon9 MC harness, openbmp-bridge, hal_topic_gate) and add an 'openbmp conform' suite whose dual-use dependency-graph assertion extends the existing hal-portability job to the fork's graph.** — tooling-vnv and fsw-boundary dossiers plus the actual files show the CLI surface and trait boundaries are already correct; the gap is connection + a fork-facing conformance harness. Conformance/HIL are exactly where prohibited capability could re-enter, so they must be governed by docs/dual-use-assessment.md and a CI dependency-graph check at the board layer. _Trade-offs:_ Conformance and HIL hooks add governance overhead specifically because they are the dual-use leak surface; that overhead is intentional and non-negotiable per the guardrail.

### Forward-Only Trajectory Subsystem & Dual-Use Governance

- **Admit 'ballistic missile trajectory prediction' ONLY as forward ballistic propagation of a vehicle-intrinsic free-flight state (BallisticState OUTPUT of forward boost sim), with an explicit architecturally-enforced Non-Goals table.** — The physics is identical to the range-safety footprinting OpenBMP already does (profile.rs:1222-1472) and is bounded by the same Tier-1 lock. The owner's item is legitimate forward physics; the engagement half is what must be refused. _Trade-offs:_ Refusing the POST2/GMAT/ASTOS targeting/optimization half means OpenBMP is intentionally less capable than those tools; that asymmetry is the safety feature, not a deficiency.
- **Enforce the boundary at the TYPE level (no input struct can express a target/range/aimpoint), making weaponization unconstructible rather than discouraged; lint and docs are secondary.** — Matches the existing load-bearing Tier-1 pattern (dual-use-assessment.md:63; profile.rs structs have no target field). Documentation explains the constraint but is not the constraint (dual-use-assessment.md:69-71). _Trade-offs:_ Requires discipline on every PR (the §3 merge checklist) and forecloses some 'academic completeness' features (trajectory optimization to a condition) that a non-safety-critical project might add.
- **Keep the propagator strictly offline/post-processing — never reachable from openbmp-fc guidance/controller loop or a bus topic.** — Mirrors the existing rule that LandingFootprintConfig is never consumed by the flight-controller loop (document.rs:2920-2955); separation of concerns (guidance->inertial objectives; footprinting->offline prediction) is itself a forward-only lock. _Trade-offs:_ Forecloses live in-loop trajectory prediction; that is intentional, since in-loop prediction-to-a-location is the targeting failure mode.
- **Output statistics about the sample mean/nominal (`cep50_m`, `radial_offset_from_nominal_m`, dispersion ellipse); forbid any miss-distance/error-to-target field; emit range-relative or predicted-geodetic coordinates only.** — Removes the only place a target could re-enter the design (scoring against an aimpoint). Matches RocketPy/NASA MC-dispersion norms and the profile-vocabulary Rule 1. _Trade-offs:_ Users wanting accuracy-to-aimpoint metrics are categorically unserved; this is the desired outcome.
- **Treat the existing Tier-3 provenance gate (no real fielded-vehicle data) as the de-facto export-control control, tied explicitly to EXPORT-CONTROL.md; governance stays awareness-level.** — Publication does NOT strip ITAR technical-data status (unlike EAR), so the durable strategy is to never be ITAR technical data in the first place — which the provenance gate already achieves. _Trade-offs:_ Cannot prevent a fork or a user importing controlled ballistic-coefficient data; that responsibility shifts to the adopter (documented), and the permissive license cannot legally bind a fork.

### Roadmap & Phasing

- **Order correctness before portability before hardware.** — FC-owned mission decisions now use the single estimated-state commander path; preserve that while finishing alloc-free/static-bus work. PIL is cheap once the core compiles and executes in the target profile, expensive otherwise. _Trade-offs:_ This delays visible board execution, but lifting a disciplined SIL path is better than lifting a truth-coupled shortcut.
- **Keep M0 baselines in the merge gate before and after refactors.** — A full golden gate makes subsequent sequencer and portability work auditable, which is what makes the refactor safe for a solo author. _Trade-offs:_ Golden updates require explicit review and justification; that friction is intentional.
- **Designate M0+M1+start-of-M2 as the smallest high-leverage first milestone, and define 'fork-able' as a 7-point checklist.** — This slice fixes a real correctness bug (truth leak), lays the load-bearing portability seam (openbmp-hal crate + CI dependency-graph lock), and strengthens a dual-use control, all in ~1-2 focused weeks. The checklist makes 'done' falsifiable. _Trade-offs:_ Bundling three concerns in one PR series risks review complexity; mitigated by shipping as a sub-PR series.
- **Sequence near-the-line capabilities (M9 forward-trajectory, M10 FDIR) only behind their Tier-1 binding and after the single evaluator, and make M2/M9/M10 EXTEND dual-use controls rather than weaken them.** — Per docs/dual-use-assessment.md:69-71, new near-the-line capability lands only when a Tier-1 constraint binds it. The CI dependency-graph assertion introduced in M2 also serves as a dual-use control against board layers re-importing prohibited capability. _Trade-offs:_ FDIR and forward-trajectory land later than a purely capability-driven roadmap would place them; accepted as non-negotiable per the fail-closed guardrail.
- **Adopt tolerance-bounded (not bit-exact) as the explicit cross-platform conformance contract, scoping the bit-exact determinism gate to the x86 reference platform only.** — ARM NEON vs x86 SSE and differing libm/FMA make bit-equality unachievable cross-platform (crate-map and rust-flight risks); the existing compare_telemetry tooling already implies tolerance-bounded. _Trade-offs:_ Loses the strongest possible 'identical behavior' claim on target hardware; pinning a shared libm narrows the gap and keeps the sim a valid cross-check.


---

# Appendix B — Open questions


### Target Architecture & Flight Portability

- Concurrency model for the L3 runtime / L2 firmware shell: single-threaded cooperative (ArduPilot, simpler/deterministic for host sim) vs multi-task preemptive (PX4/F´/cFS/RTIC, maps to multi-core flight). This shapes the rate-group/scheduler abstraction and must be chosen explicitly.
- The first no_std-clean split line is now validated for `openbmp-physics` flight math, `openbmp-propulsion` core motor/engine math, `openbmp-aero` core deck/hypersonic math, the bare `openbmp-sensors` trait surface, `openbmp-hal`, and the alloc-aware `openbmp-fc` controller on `thumbv7em-none-eabihf`; the remaining split line is alloc-free/static-registry hardening inside `openbmp-fc`.
- Whether the per-tick mission interpreter (EventScalars/fired) can be made fully fixed-capacity (ArrayVec MAX bounds for bindings/lanes) without truncating real scenarios; reasonable embedded RAM limits (64-512 KB) vs scenario complexity is unmeasured.
- Bit-vs-tolerance reproducibility across ARM (NEON/scalar) is asserted, not measured; a cross-platform bit-equality harness (compile kernel+state for thumbv7em, compare against x86_64 baseline) is needed before any cross-platform determinism claim.
- Sufficiency of the current Sensor/Clock trait surface for real sensor protocols (CAN/SPI/I2C, async/buffered/timestamped fusion, 'healthy' semantics, GNSS-sync hooks) is unknown without a reference board-crate implementation.
- The remaining SIL estimated-scalar capability gap is relative navigation: `mass_fraction` now comes from onboard `PropellantState`, while relative distance/speed triggers are parse-time unsupported under FC authority until an FC observable exists.

### The GNC Stack

- S3 relative-nav capability: parse-time rejection is now the fail-closed mechanism for FC authority; the open work is the observable itself and the compatibility policy for enabling these triggers later.
- Any future change to default rate-group cadences is a behavioral-rebaseline event; the declarative scheduler exists, but default cadence changes must be reviewed against golden telemetry.
- The boost/coast phase signal that drives GNSS process-noise scaling currently comes from commander-published estimator-regime region state; future richer regimes should preserve that bus coupling rather than reaching into mission internals.
- Slot-budget for the future convex powered-descent guidance: the scheduler has no WCET data yet, so the guidance rate-group slot size needed for a bounded-iteration SOCP solver is unknown and is a risk for that capability (item 7).
- Should the AFTS-style flight-corridor engine be in scope at all for the upstream repo, or held as a documented design with only the Tier-1 lock specified? Corridor = deviation-from-forward-nominal is defensible, but it is the closest-to-the-line feature and may be better left to a fork under its own export-control footing.

### Verification, SIL/PIL/HIL & Developer Tooling

- Coverage number must be MEASURED on the actual GNC/dynamics core (cargo llvm-cov) before any threshold is published in CI; ci.yml currently has no coverage lane, so the section says 'measure first, then commit' rather than naming a figure.
- PIL is no longer blocked on basic `no_std` compilation, but it remains blocked on a target execution harness plus the alloc-free/static-registry hardening needed for a defensible bare-metal profile. Whether the first liftability target is bare-metal RTOS or the easier embedded-Linux (HAL_Linux / PX4-on-POSIX) path is an owner decision that changes how aggressively no-heap work must be pursued.
- Cross-platform (ARM) bit-equality vs tolerance-bounded state-equality is a real cost decision: bit-equality requires a shared software libm and disabling FMA contraction. The section recommends tolerance-bounded as the contract (matching verification.md's 'state-stable, not bit-stable' label for non-reference platforms), but the owner should confirm no future use case requires bit-exact host<->target replay.
- The lockstep handshake protocol over `openbmp-bridge` now has concrete in-house packet types (`BridgeHelloPacket`, `BridgeMessage`, `StepAckPacket`) and validators that reject mismatched step/time responses. It is still a schema/codec layer, not a real-time transport or hardware bench; retry policy, wall-clock deadlines, and board-specific adapter timing remain downstream HIL work.
- `[fc]` + `[mission]` scenarios now default mission authority to `FlightController`; the remaining policy question is how long to keep explicit `scenario_director.mission_authority = "kernel"` opt-in for pure-sim convenience and golden-regression isolation.

### Forward-Only Trajectory Subsystem & Dual-Use Governance

- Should new ballistic-specific terms (desired-impact, guidance-to-impact, terminal-prediction, accuracy-metric) be pre-emptively added to FORBIDDEN_SAFETY_TERMS in lint.rs, or held until a field name actually warrants them (keeping the list short per current practice)?
- What public re-entry/ballistic-descent references will be used to move the propagator from 'experimental' to 'validated-toy'/'research' (Allen-Eggers analytic is available; are public Apollo/Gemini/X-15 coast cases acceptable under the provenance gate)?
- Should the adaptive Dormand-Prince RK45 mode ship at all given the determinism-gate tension, or should the subsystem stay fixed-step only until adaptive stepping has its own validated cross-platform reproducibility story?
- Should predicted-geodetic output (lat/lon when launch_origin is declared) be gated behind an explicit opt-in flag, or omitted entirely in favor of range-relative-only output to eliminate the geodetic-output misreading trap?
- How is the scenario-consumer-agreement schema lock (only the offline footprint block may consume BallisticState; no guidance block may) best expressed and CI-enforced, beyond the lexical lint?
- How broad should the sealed dual-use field allowlists remain as the footprint APIs evolve? The current assertions are CI-backed and intentionally narrow; widening them should stay a Tier-1 safety-review event rather than routine schema churn.

### Roadmap & Phasing

- The current implementation achieves the first `no_std` target by feature-gating `openbmp-fc` itself. A separate `openbmp-gnc-core` crate remains an option if the static-bus/no-heap carve-out becomes too invasive inside the existing crate.
- What is the realistic first real-board target for the 'fork-able' DoD: embedded-Linux (std-on-board, HAL_Linux/PX4-on-POSIX analog, much cheaper) or bare-metal RTOS (true no_std)? The roadmap assumes no_std compilation first with embedded-Linux as the first real-board milestone, but the owner should confirm the priority.
- Does the owner want M9 (forward-trajectory) slotted early (it is independent of the HAL line) or held until after the flight-core line, given the residual dual-use review burden it adds per merge?
- Is there appetite for a Ferrocene-qualified-toolchain spike inside M6/M7, or is that deferred entirely to a downstream fork's certification effort (per the cFS no-transferable-assurance precedent)?
- For M4, which relative-nav observable (if any) does the owner consider in-scope for SIL, versus fail-closed? This determines whether AtRelativeDistance/AtRelativeSpeed become functional or remain explicitly unsupported with a documented reason.


---

# Appendix C — Review notes & resolutions (adversarial red-team)


The design was reviewed by an adversarial completeness/technical/dual-use red-team. Its prioritized fixes are folded into the sections above where structural; the remaining items are tracked here honestly as known-open, per the project's no-overstating rule.


### Prioritized fixes (folded in / committed)

- [DUAL-USE P0] `BallisticState` is output-only at the type level (private fields plus provenance-named constructor), and CI enforces the scenario-consumer agreement so guidance/control paths cannot consume free-flight / footprint seed symbols.
- [DUAL-USE P0] Dual-use controls are listed as required branch-protection checks in `CONTRIBUTING.md`, and the semantic field audit is implemented as sealed allowlists per sensitive struct. The remaining enforcement piece is hosted GitHub branch-protection configuration, which cannot be set from the repository contents.
- [TECH P0] Upstream concurrency and HAL bus layering are now explicit: the portable HAL contract is single-threaded/cooperative, `openbmp-hal::Bus` takes `&mut self`, and preemptive/interrupt-safe publication is a future board-adapter contract rather than an implicit property of `StaticBus`.
- [TECH P1] StaticBus topic indexing is concrete: topics carry a compile-time `INDEX`, `openbmp-hal::StaticBus<const TOPICS, const SLOT_BYTES>` avoids `TypeId`/`Any`, and `openbmp-fc` topics now define static indices. The remaining unblock is adapting the FC host bus/registries to the static, alloc-free path.
- [COMPLETENESS P1] Add an explicit real-time/WCET stance — bounded-loop, no-recursion, no-hot-path-alloc, bounded-iteration SOCP solver (clarabel is already a dep) — as ENFORCED L2 invariants, plus a stated error-handling contract mapping HAL Result::Err to FDIR fault classes. Declared scheduler budgets and overrun-to-FDIR wiring are implemented; hardware WCET, stack-depth, and alloc-free hot-path proof remain open.
- [ACCURACY P1] Fix factual drifts: crate count is 19; deps are already default-features=false so the no_std work is REMOVING std features (rand std_rng, uom std), not "flipping" them; `openbmp-fc` now has crate-level `#![forbid(unsafe_code)]`; unwrap/expect/panic are warn-level workspace lints promoted by CI `-D warnings`; and `panic = "abort"` is a firmware-side profile strategy, not an L2 panic-freedom proof.
- [TECH P2] State the cross-platform determinism contract honestly: tolerance-bound requires BOTH shared libm AND FMA-contraction control, not libm alone; confine adaptive RK45 to non-gated runs and require fixed-step for all goldens and MC acceptance gates; either build the determinism-matrix harness as a real milestone or downgrade the 'sim is a valid cross-check of flight' argument to aspirational.
- [COMPLETENESS P2] Stale dead-code assertions have been re-verified and corrected. `PropellantState`, scheduler rates/budgets, and high-dynamics estimator-regime scheduling are live; I-load envelope versioning/CRC/fallback exists; the remaining scalar gap is relative navigation, and the remaining config-management gap is the exact mission/gain payload schema.

### Acknowledged open items (to specify before the dependent milestone)

- THREADING/CONCURRENCY MODEL IS NOW SETTLED FOR UPSTREAM, BUT NOT FOR EVERY FUTURE BOARD. The upstream HAL contract is single-threaded/cooperative by design: `openbmp_hal::Bus` takes `&mut self`, `StaticBus` is fixed storage with no locks/atomics, and `docs/HAL.md` states the current portable contract does not require `Sync`, async, or critical sections. A preemptive RTIC/cFS-style shell is therefore a future HAL-versioning decision, not an implicit property of the current bus. A downstream fork that needs interrupt-safe publication must add an explicit critical-section/queue adapter below the HAL boundary and re-measure timing; it cannot assume the current `StaticBus` is a cross-task shared bus.
- REAL-TIME / WCET STORY IS PARTLY DESIGNED, NOT PROVEN. Declared per-job budgets, overrun events, and health/FDIR escalation now exist, and `REQ-RT-001` locks known tick-allocation regressions in scheduler dispatch, commander binding evaluation, and voted sensor ingest scratch buffers. This is still not WCET evidence: EKF/MEKF stack and execution bounds, bounded-iteration SOCP timing, interrupt-latency budget, and hardware storage/watchdog timing require PIL/HIL bench measurement. At minimum, a broader bounded-loop/no-recursion/no-hot-path-alloc inventory must become an enforced L2 invariant before a bare-metal flight-core claim.
- ERROR-HANDLING / FAULT-PROPAGATION CONTRACT IS SPECIFIED FOR SENSOR READ FAILURES, PARTIAL ELSEWHERE. The current FC path is fail-closed for sensors: `Sensor::read()->Err` causes ingest to publish an unhealthy sample/status, `HealthMonitor` promotes that to `FailsafeFlags`, and `FdirJob` latches the matching sensor fault bit (`REQ-FDIR-002`). The remaining gaps are actuator command errors, watchdog service errors, and storage write failures: those HAL calls have `Result` surfaces, but no upstream sequenced response beyond returning the backend error to the caller.
- CONFIGURATION MANAGEMENT / I-LOAD VERSIONING IS SPECIFIED AT THE ENVELOPE LAYER, WITH PAYLOAD SCHEMA STILL OPEN. The HAL now wraps flight I-load payloads in `OBIL`: envelope version, payload schema version, length, CRC-32, and primary/fallback validation (`REQ-ILOAD-001`). This closes the integrity/version/fallback wrapper gap. The remaining work is the exact host-generated payload schema for mission graph + event bindings + gain tables and its compatibility policy across upstream releases.
- CONTRIBUTION / GOVERNANCE MODEL IS SPECIFIED IN-REPO, WITH HOSTED SETTINGS STILL EXTERNAL. `CONTRIBUTING.md`, `.github/CODEOWNERS`, and the PR template require code-owner review, protected CI checks, and a dual-use gate even for maintainer or solo-author changes. The remaining non-repository piece is GitHub branch-protection settings themselves; they must be configured by the repository owner and reviewed when CI job names change.
- CROSS-PLATFORM DETERMINISM HARNESS IS PROMISED BUT NOT A DELIVERABLE. arch-portability open-question #4 and arch-vnv both say ARM bit-vs-tolerance reproducibility is 'asserted, not measured' and call for a cross-platform bit-equality harness, but no milestone builds it (M7 PIL only does tolerance-bounded matching). The 'pin libm on host and target so transcendentals match' cross-validation argument is never validated by a milestone. Add a determinism-matrix milestone or downgrade the libm-cross-check argument to aspirational.
- FLIGHT-SIDE TELEMETRY/RECORDING CONTRACT IS PARTLY SPECIFIED, NOT YET BENCH-PROVEN. The HAL now defines an `OBFR` flight-record frame (version, kind, sequence, monotonic timestamp, payload length, CRC-32) and a no-heap `FlightRecorder<CAPACITY, RECORD_BYTES>` with explicit drop-newest / overwrite-oldest policies, so record format and queue overflow are no longer stubs. Remaining work is hardware-specific: flash/FRAM write latency, wear leveling, persistence under power loss, and reconciliation of decoded flight frames against the host golden contract on a real board.
- MIL/SIL COLLAPSE DISCARDS THE INDEPENDENT-CHECK VALUE OF MIL. arch-vnv argues MIL collapses into SIL because 'the Rust algorithm IS the model.' True, but with no separate model an algorithm BUG is identical in 'model' and 'code' and invisible to SIL-vs-truth comparison. The only independent checks left are the physics-kernel-as-plant and external-telemetry validation — and the latter is deliberately OFF the merge gate (quarantined). So there is effectively no GATED independent check on GNC algorithm correctness; acknowledge this as a credibility gap for a 'best-in-class' claim.
- RATE-GROUP DEFAULT CADENCE CHANGES ARE BEHAVIORAL REBASELINES. The scheduler now supports per-job periods and budgets, but changing defaults will change closed-loop behavior and golden outputs by construction. Treat cadence changes as reviewed behavioral changes with new baselines, not config-only edits.

### Technical issues raised (and the fix adopted)

- **[medium]** MSRV/pinned-Rust drift. Resolved by referring to the MSRV pinned in `Cargo.toml` where reproducibility matters, while CI uses the current stable toolchain for normal gates and the pinned MSRV for the `msrv` job.
- **[low]** CRATE COUNT INCONSISTENT/WRONG: arch-portability says 'today's 19 crates'; arch-roadmap M0 says '19-crate ... simulator.' Cargo.toml lists 19 members (VERIFIED). Also openbmp-scenario-script is a real workspace member but appears only parenthetically as 'openbmp-scenario(-script)' with no stratum row. — _Fix:_ State 19 (from Cargo.toml members) consistently; give openbmp-scenario-script its own L4 row.
- **[medium]** DEFAULT-FEATURES FIX MISDESCRIBES THE WORK. arch-portability remediation step 1 says 'flip nalgebra/uom/rand/serde to default-features=false.' The manifest (VERIFIED) ALREADY declares nalgebra/uom/num-traits/rand with default-features=false, then re-adds std/std_rng (nalgebra features=['std']; uom features=[...,'std',...]; rand features=['std','std_rng']). The real no_std work is REMOVING those std feature additions and providing no_std alternatives. Note rand_chacha is ALREADY default-features=false and the core's DeterministicRng wraps ChaCha8Rng (VERIFIED core docstring), so the RNG path may be closer to no_std than the doc implies — but rand's std_rng and uom's std are the real blockers. — _Fix:_ Rewrite step 1: 'remove the std/std_rng feature additions, gate them behind a new std feature, confirm uom/nalgebra compile alloc-free no_std.' Recognize the ChaCha8Rng path is likely already no_std-able; call out rand-std-removal and uom-no_std as the specific sub-risks.
- **[medium]** forbid vs deny overstatement. Resolved for the FC crate: `openbmp-fc` now has crate-level `#![forbid(unsafe_code)]` in addition to the workspace `unsafe_code = "deny"` lint.
- **[low]** unwrap/expect/panic lints are WARN, not deny. Workspace [workspace.lints.clippy] sets unwrap_used/expect_used/panic = 'warn' (VERIFIED, Cargo.toml:46-47 panic='warn'); they only become hard errors via the CI -D warnings flag. arch-vnv implies a clean existing deny-style gate; arch-portability remediation 8 correctly proposes tightening to deny for L2. — _Fix:_ State the actual mechanism: warn-level workspace lints promoted to errors only by CI -D warnings; make them crate-level deny in L2 so they fail even without -D warnings and cannot be locally downgraded.
- **[high]** StaticBus<const N> design is hand-waved at the hardest point. The current bus keys by TypeId at runtime (VERIFIED: BusInner{ cells: IndexMap<TypeId, BusCell> }, BusCell{ storage: Box<dyn Any> }). A StaticBus needs every topic mapped to a const index WITHOUT TypeId. The doc says 'generated from a compile-time enum of all flight topics' but gives no mechanism (proc-macro? const generics? associated const?). This is named the 'single biggest unblock' (remediation 3) yet is the least specified; mapping a generic T to a const slot in stable Rust needs a macro-generated match or an associated const INDEX on the Topic trait (which today has only NAME/VERSION, VERIFIED), neither stated. — _Fix:_ Specify: add const INDEX: usize (or a sealed topic-id enum) to the Topic trait, generate the flight topic table via a declarative macro, index a fixed [Option<Slot>; N]. This changes the Topic trait surface, so design it at M2 (trait extraction), not M6.
- **[high]** INTERNAL CONTRADICTION on bus layering. arch-portability's dependency diagram puts the Bus trait in L1 openbmp-hal, but StaticBus lives in L2 openbmp-fc and the topic enum is 'all flight topics' whose types (ImuSample etc.) live in openbmp-fc/topics.rs (L2). The Topic trait itself is currently defined in openbmp-fc/bus.rs (VERIFIED, L2). If Topic types are L2 but the Bus trait is moved to L1, L1 cannot name the concrete topics and the flight topic enum straddles L1/L2, breaking the 'arrows point down only' invariant for the bus. — _Fix:_ Resolve where the Topic trait and message types live: either move shared Topic trait + message types to L1 (openbmp-core or a new openbmp-msgs) so both the L1 Bus trait and L2 StaticBus can name them, or make Bus generic over an associated TopicSet instantiated in L2. State which.
- **[medium]** DEAD-CODE claims drifted. Resolved for the named blockers: `PropellantState` is published by the FC bridge and consumed by the commander, scheduler periods/budgets are wired from `[fc.scheduler]`, and `high_dynamics_q_scale` is driven by commander-published estimator-regime state. Relative distance/speed maps remain intentionally empty until an onboard relative-nav observable exists.
- **[medium]** Adaptive Dormand-Prince RK45 (arch-trajectory ingredient #2, M9) conflicts with the byte-stable determinism gate more than the doc admits. An adaptive step-size sequence is input-data- and platform-FP-dependent, so even tolerance-bounded cross-platform reproducibility is weaker for adaptive, and adaptive arcs feeding Monte-Carlo could make MC pass-rates platform-dependent — leaking into the MC acceptance gate, not just the golden gate. — _Fix:_ Defer adaptive RK45 until a cross-platform reproducibility story exists (the open question already asks this), OR confine adaptive stepping to non-gated exploratory runs and require ALL goldens and MC acceptance gates to use fixed-step. State that MC gates run fixed-step only.
- **[medium]** SOLO-FEASIBILITY OVERREACH in aggregate. M6 is honestly L/High, but no_std carve-out + an openbmp-conformance crate third parties run + convex powered-descent guidance (lossless-convexification SOCP) + square-root EKF + AFTS-style corridor engine + determinism matrix is multiple person-years solo. arch-gnc lists 'square-root EKF behind a flag' and 'convex powered-descent guidance' as if comparable to wiring tasks; each is a substantial GNC research effort. This tensions with the user's own no-overstating-blockers directive. (Note: clarabel SOCP solver IS already a workspace dep (VERIFIED), so the convex-descent solver primitive exists — but bounded-iteration/WCET integration and validation remain research-grade.) — _Fix:_ Tier the roadmap into 'foundational, solo-achievable (M0-M7)' vs 'research-grade, fork-or-collaborator (convex descent, sqrt-EKF, AFTS corridor, bare-metal no_std)'. Mark the latter designed-and-locked-but-not-committed so 'done' is never claimed while incomplete.
- **[low]** 'panic = abort already set in the release profile' is TRUE (VERIFIED, Cargo.toml:144 panic='abort'), so that prerequisite claim holds. HOWEVER the architectural reasoning is incomplete: panic=abort in a LIBRARY/workspace release profile does not propagate panic-freedom to a downstream firmware binary (the integrating firmware crate sets its own panic strategy and must supply its own #[panic_handler]). The doc's remediation 8 correctly notes putting #[panic_handler] in the firmware binary, but the 'one prerequisite already met' framing slightly overstates what panic=abort buys for L2 safety. — _Fix:_ Keep the (correct) 'panic=abort is set' statement but reframe the architectural claim as 'L2 must be panic-free by construction (deny unwrap/expect/panic + bounded indexing/arith); panic=abort is the firmware-side strategy, not an L2 safety guarantee.'
- **[medium]** The 'shared libm => tolerance-bounded sim/flight equivalence' argument is weaker than presented. openbmp-core routes transcendentals/units through dedicated modules (VERIFIED: core has quantities/frames/rng/time modules and a determinism contract), but f64 +/-/*/ FMA contraction differs across host/target and libm does NOT govern it. For an EKF (many dot products) FMA contraction is the larger cross-platform divergence source. The doc mentions FMA in passing but still leans on libm-pinning as if it closes the gap. — _Fix:_ State that cross-platform tolerance-bound requires BOTH a shared libm AND disabling FMA contraction (target-feature/codegen), and that libm alone is insufficient. Make the FMA stance explicit in the determinism contract, not a parenthetical.

### Dual-use review summary

The forward-only posture is the document's best-executed dimension and is genuinely strong. The core insight is correct: enforce non-weaponization by the ABSENCE of any input field that can name a target (Tier-1 type-level lock), with the lexical lint and docs as secondary 'activation-energy' layers. It is consistently applied across all five sections — HAL traits reject aimpoints, guidance consumes only inertial/orbital references, MC stats are about-the-sample-mean, the propagator is offline-only and unreachable from the control loop. The 'ballistic missile trajectory prediction' item is scoped exactly as required: admitted only as forward propagation of a vehicle-intrinsic free-flight state that is itself an OUTPUT, with a prominent architecturally-enforced NON-GOALS table and the Tier-1 lock extended to the new subsystem. Governance stays correctly at awareness/policy level (EAR published carve-out, fundamental research, ITAR-hardware-not-physics) without operational evasion, and correctly identifies the provenance gate (no real fielded-vehicle data) as the durable export-control control since publication does not strip ITAR technical-data status. No section contains a forbidden capability or a how-to for inverse-to-target. The concerns below are about CLOSING enforcement gaps so the locks are mechanically fail-closed rather than discipline-dependent — not about anything currently weaponizable.

**Residual concerns (mitigated by the §4 hardening commitments):**

- ENFORCEMENT IS POLICY-DEPENDENT AT THE EXACT POINTS IT CLAIMS TO BE STRUCTURAL. The doc repeatedly says weaponization is 'unconstructible (type error), not a runtime check,' but no CI mechanism PROVES no input struct has a target-expressing field. It relies on the FORBIDDEN_SAFETY_TERMS lexical lint (catches names, not semantics) plus reviewer discipline. A contributor could add `target_radius_m` or `desired_eci_position` (semantically a target, lexically using legitimate forward-physics words 'radius'/'position') and the lint would not catch it. The type-level lock is asserted but not mechanically verified.
- TARGET<->FORWARD-STATE TYPE EQUIVALENCE IS THE STRUCTURAL SOFT SPOT. BallisticState carries {eci_position, eci_velocity, ballistic_coefficient, timestamp}. A target IS an eci_position. The lock is purely that this struct is documented/used as an OUTPUT consumed offline; there is no TYPE-level distinction between an ECI position that is a propagation INITIAL condition (output of boost) and one that is a desired destination. If a contributor wired BallisticState as an INPUT seed from a hand-authored scenario, the same struct becomes a launch-state tool, and forward propagation from chosen initial states is exactly how range tables are built (sweep initial states -> tabulate impacts). 'It is an output' is enforced by data flow / scenario-consumer-agreement, NOT by the type — and that agreement is itself flagged (arch-trajectory open-question #5) as not-yet-CI-enforced.
- MONTE-CARLO IS A LATENT RANGE-TABLE GENERATOR IF UNCERTAINTY SOURCES EVER INCLUDE LAUNCH PARAMETERS. The doc correctly forbids sweeping launch parameters indexed to range, but the distinction between 'dispersing initial azimuth/elevation as uncertainty' and 'sweeping azimuth/elevation to find range' is purely intent — the same sampling machinery does both. No type-level or lint barrier prevents a dispersion source from being launch azimuth/elevation/initial-velocity-direction; those + the existing downrange_m output ARE a range/azimuth->impact table.
- PREDICTED-GEODETIC OUTPUT (lat/lon when launch_origin declared) IS A GENUINE RE-INTRODUCTION VECTOR. The doc flags the 'geodetic-output misreading trap' and proposes labeling, but labeling is documentation, not a lock. Predicted impact lat/lon combined with MC over launch direction is operationally an impact-point predictor; 'predicted' vs 'desired' labeling does not change what the numbers ARE or what a downstream tool does with them.
- THE PROPOSED DEPENDENCY-GRAPH CI ASSERTION DOES NOT ACTUALLY GUARD THE PROPAGATOR PROPERTY IT CLAIMS. The assertion checks openbmp-fc has no path to sim/runner/scenario and that 'no prohibited-capability surface is reachable from a board target,' but (a) it has no enumerated prohibited-capability denylist to check against, and (b) the propagator lives in openbmp-physics/runner, NOT in openbmp-fc — so an fc-dependency check does not prove 'propagator unreachable from the control loop' (M9's claimed guarantee). That needs a distinct, concretely-specified assertion.
