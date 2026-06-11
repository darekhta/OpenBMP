# Flight-Software-in-the-Loop: SIL / PIL / HIL

**Status:** `experimental` (design intent; selected bridge/RT substrate now ships in code).
**Audience:** the engineer or LLM agent who will implement the work packages, having read `00-overview.md` and `13-agent-execution-playbook.md` first.
**One-line summary:** Lift OpenBMP from "algorithm-in-the-loop" to a true *binary-behind-a-transport* SIL with ASAM-XIL-style fault-injection ports, a soft-real-time HIL-software bench, a conformant FMI 3.0 co-simulation master, PIL on an emulated flight ISA, and NASA cFS apps as the flight software — every rung forward-only, bit-true, and hardware-boundary-abstract.

---

## 1. Parity target & ceiling

### 1.1 Target (capability / architecture parity within posture)

The reference organizations verify flight software by running the **actual flight binary** behind an **unchanged sensor/actuator boundary**, across the canonical XIL ladder — Model-in-the-Loop, Software-in-the-Loop, Processor-in-the-Loop, Hardware-in-the-Loop — with deterministic, replayable, fault-injectable benches (dSPACE, NI VeriStand, OPAL-RT, Typhoon HIL; NASA Trick, NOS3, 42). The architecture-parity target for OpenBMP is to reproduce that ladder's *methods and interfaces* within the project's simulation-only, hardware-boundary-abstract posture:

- **SIL over a transport.** The flight controller runs as a *separate stepper* behind a transport-agnostic, lockstep, step-and-ack wire — in-process channel for the host fast path, Unix-domain-socket / TCP-loopback for an out-of-process binary — and the run is bit-true reproducible against `(seed, scenario)`.
- **ASAM-XIL-shaped ports.** A `MaPort`/`EesPort`-shaped trait surface (read/write/capture/stimulate + electrical-error injection) with a symbolic signal-name mapping and a testbench lifecycle FSM, so the same test case replays MIL→SIL→PIL→HIL unchanged. Fault injection "cuts the strings": it degrades exactly the bytes the FC perceives and commands, with no FC source change.
- **Soft-real-time HIL-software bench.** A fixed-step plant on an isolated/`SCHED_FIFO` core, `clock_nanosleep(TIMER_ABSTIME)` absolute-time frame pacing, overrun detection, and a p50/p99/p99.9/max jitter histogram — useful *with no hardware* because it surfaces deadline/WCET behavior the host SIL hides. Shared crate `openbmp-rt` (L7), consumed through the v3 `[realtime]` runner block and shared with `12-determinism-realtime-and-compute.md`.
- **Conformant FMI 3.0 co-simulation master** (importer + an OpenBMP exporter FMU), with typed value references, `modelDescription.xml` parsing, Gauss-Seidel/Jacobi ordering, early-return and the clock subset, validated against the Modelica Reference FMUs and `fmpy`.
- **PIL on an emulated flight ISA.** Factor the FC step into the existing `no_std` core, cross-compile to `thumbv7em`/`riscv32`, run under Renode (deterministic shared virtual time, MIT) coupled over GDB-stub or virtual-UART using the *same* bridge frames, with cycle-approximate WCET.
- **cFS apps as flight software in the loop.** Couple NASA Core Flight System (Apache-2.0) behind the HAL via a sim-hardware cFS app and a Rust↔cFE-Software-Bus bridge, driving cFS TIME from the sim clock for lockstep determinism, following the NOS3 reference architecture.
- **AFTS as a forward containment *monitor* only.** An independent instantaneous-impact-point (IIP) geo-containment monitor emitting a latched terminate boolean + which-rule-fired, verified against closed-form ballistics; never a targeting or guidance input (`00` §6).

### 1.2 Parity ceiling (the honest boundary)

What an open academic repo **cannot** reach, and the credible open substitute for each:

1. **Certified / flight-heritage AFTS.** RCC 319 qualification, range-safety sign-off, and the NAFTU software itself are controlled (NASA Software User Agreement, not redistributable). *Open substitute:* a clean-room forward IIP-geo-containment monitor from published AFSS algorithms (NTRS 20080044860), verified by closed-form conic ballistics and MMS point-in-polygon; **never** a flight/range certification claim.
2. **True cycle-accurate WCET on the real rad-hard flight processor** (RAD750, LEON-FT, custom rad-hard FPGA soft-cores) with the real TMR/voter fabric and SEU timing. No open emulator reproduces a specific flight part's pipeline + radiation-upset behavior. *Open substitute:* cycle-**approximate** timing (instruction-count × CPI) under Renode/QEMU, a documented WCET budget, plus published FPGA-TMR voter studies for the voter logic.
3. **Deterministic hard-real-time bus electricals** (MIL-STD-1553 / SpaceWire / CAN-FD signal integrity, bounded-latency redundant rings, the physical iron-bird harness). Requires real hardware and proprietary qualification. *Open substitute:* model the bus as a deterministic frame transport with injected delay/drop/duplicate/bit-flip faults; characterize jitter with `cyclictest`-style self-measurement; keep real drivers in a downstream adopter crate (`docs/safety-boundaries.md`).
4. **Proprietary SpaceX / Rocket Lab / JPL HIL-bench validation data and the real flight binaries.** Unavailable by definition. *Open substitute:* cFS-as-flight-SW behind the HAL via the NOS3 architecture, FMI Reference-FMU cross-check, code-to-code vs 42, and bit-true self-determinism gates.

**Net reachable claim:** OpenBMP can credibly reach *"real heritage flight software (cFS) in a bit-true, fault-injected, PIL-on-target, soft-real-time loop with a forward AFTS containment monitor"* — but **not** *"certified, flight-part-cycle-accurate, hardware-electrical HIL."* No artifact earns `flight-qualified`/`certified`/`operational` (`00` §3.8). The honest top label for this dimension is `validated-toy`, rising to `research` where a public benchmark exists (FMI Reference FMUs, cFS-under-NOS3, closed-form IIP).

---

## 2. Current state in source

Verified by reading the files below at the program baseline plus the current parity implementation slices. The honest summary: **the schema, validators, generic transports, bridge-level lockstep master, opt-in runner in-process/TCP/Unix-loopback/external-process transport subset, bridge packet expansion, actuator-stream determinism gates, XIL evidence hooks, and realtime pacer exist; PIL/cFS execution does not.** OpenBMP is on Tier 0 ("algorithm-in-the-loop") with partial Tier 1/Tier 3 substrate in place.

| Capability | Where | State |
|---|---|---|
| Lockstep **wire schema** | `crates/openbmp-bridge/src/packet.rs` | `SensorPacket`, `ActuatorCommandPacket`, `StepAckPacket`/`StepAckStatus`, `BridgeHelloPacket`, `BridgeMessage`, `BridgeFaultPacket`/`BridgeFaultCode`, `PROTOCOL_VERSION = 2`. Complete and `serde`-derived. |
| **Codec + framing** | `crates/openbmp-bridge/src/codec.rs` | `encode`/`decode` (`postcard`), `frame`/`deframe` (`u32`-LE length prefix, concatenation-safe). Complete. |
| **Lockstep validators / master** | `crates/openbmp-bridge/src/lockstep.rs` | `validate_protocol_version`, `validate_command_for_sensor`, `validate_ack_for_sensor`; step + bit-exact `f64::to_bits` time match. `LockstepSimMaster` performs simulator hello/version/role validation, sends sensor frames, accepts matching commands or ACKs, and emits fault packets on fail-closed mismatches. Complete at bridge level. |
| **Transport** | `crates/openbmp-bridge/src/transport.rs` | `Transport`, `InProcessTransport`, `StreamTransport<S: Read + Write>`, `SplitStreamTransport<R: Read, W: Write>`, TCP loopback helpers, and Unix-domain-socket helpers preserve whole `BridgeMessage` boundaries over in-process queues, caller-owned streams, child stdio, or host sockets with payload limits and close/error reporting. Complete for generic host transports; real bus adapters remain downstream-owned. |
| Bridge fault transforms | `crates/openbmp-bridge/src/fault.rs`, `crates/openbmp-scenario/src/document.rs`, `crates/openbmp-runner/src/fc_bridge.rs` | `BridgeFaultTransformSet` applies ordered, step-windowed scalar transforms over named `SensorPacket` / `ActuatorCommandPacket` signals. Initial deterministic transforms cover additive bias, scale, stuck value, saturation, quantization, drift, bounded noise-burst, sign reversal, and packet-level sensor/command drop, duplicate, delay, or bit-flip dispositions; applied rule ids are returned for evidence. v3 `[fc.transport_faults]` maps those rules onto the opt-in `[fc.transport]` runner path. |
| XIL-shaped port surface | `crates/openbmp-bridge/src/ports.rs`, `crates/openbmp-sil/src/xil.rs`, `crates/openbmp-sil/tests/observed_run.rs` | `MaPort`/`EesPort`-shaped traits, symbolic `SignalMapping`, `FaultWindow`, `ElectricalErrorType`, and deterministic `TestbenchLifecycle` FSM now exist in `openbmp-bridge`. `openbmp-sil::InMemoryXilBench` implements those traits for native host tests and renders writes, captures, generators, EES faults, and lifecycle transitions into SIL evidence records. `MissionPackage::run_case_with_xil` / `run_case_observed_with_xil` attach those records to run evidence while preserving `EstimateVsTruthMonitor` observations. **Partial WP-10.2:** full external-bench replay and ASAM-XIL conformance remain out of scope. |
| Opt-in runner FC transport | `crates/openbmp-scenario/src/document.rs`, `crates/openbmp-runner/src/fc_bridge.rs`, `crates/openbmp-runner/src/bin/openbmp-fc-stdio-echo.rs`, `crates/openbmp-runner/src/determinism.rs`, `crates/openbmp-runner/tests/fc_transport.rs` | **Partial.** v3 `[fc.transport] mode = "in_process"`, `mode = "tcp_loopback"`, Unix-only `mode = "unix_loopback"`, and `mode = "external_process"` route the closed-loop IMU/GNSS/magnetometer FC scenario through `openbmp-bridge` hello + `SensorPacket`/`ActuatorCommandPacket` exchange and prove byte-identical telemetry, canonical telemetry SHA-256, and actuator command-stream SHA-256 for local loopbacks against the direct path; the external-process gate proves deterministic child-process lifecycle and actuator-stream evidence through stdio. The packet now preserves GNSS bias, barometer pressure/bias, magnetometer nT/hard-iron, star-tracker attitude, effector commands, and full engine throttle/gimbal/ignite/shutdown commands. |
| In-process FC step + truth assembly | `crates/openbmp-runner/src/fc_bridge.rs` | Default `FcBridge::tick_point_mass`/`tick_rigid_body` → `step_from_truth`: builds `SensorTruth`, primes sensors, applies an open-loop fault schedule (`StimulusSchedule`), publishes, steps the in-process `FcRunner`, applies effector/engine commands. The default path remains host-linked and byte-stable; `[fc.transport]` is opt-in. |
| Lockstep clock contract | `crates/openbmp-fc/src/clock.rs` | `Clock` trait (`now`/`tick`), `SimulatedClock`, `FixedClock`. `Instant::now`/`SystemTime::now` banned in `openbmp-fc` (`00` §3.4). |
| **FC `no_std` posture** | `crates/openbmp-fc/src/lib.rs`, `Cargo.toml` | `#![cfg_attr(not(feature = "std"), no_std)]`, `#![forbid(unsafe_code)]`, `extern crate alloc`; builds for `thumbv7em-none-eabihf` (`docs/HAL.md` verification hooks). **PIL precondition already met** — what is missing is a factored step entry point and a target binary. |
| HAL contracts | `crates/openbmp-hal/src/lib.rs` | `no_std` + `forbid(unsafe_code)`; `Clock`, `Topic`/`EncodedTopic`, `StaticBus`, `Bus`, pull-`Sensor` + marker traits, command-`Actuator` + markers, `Storage`, `Watchdog`, I-load/flight-record envelopes. The unchanged boundary the FC sits behind. |
| SIL observe-half + checks | `crates/openbmp-runner/src/sil.rs`, `crates/openbmp-sil/src/monitor.rs`, `checks.rs` | `SilMonitor::observe` (read-only tap, after-FC-before-racks); `EstimateVsTruthMonitor` records estimate-vs-truth residual norms + FDIR flags (never raw truth series); `SilCheck` is a **closed** enum (position/velocity/attitude error, NEES, innovation-storm, FDIR mask, nominal-stop) — *no* range/aimpoint/impact/miss-distance field. Forward-only by construction. |
| SIL mission-package shell | `crates/openbmp-sil/src/lib.rs`, `crates/openbmp-sil/src/xil.rs` | `SilTestbench`, `MissionPackage`, `EvidenceBundle`, `RequirementVerdict`, `FaultInjection`/`FaultTarget`, `CommandWrite`, `SignalReadReport`, `BusFrameEntry`, `aggregate_verdict` (`pass` iff no failed requirement), plus `InMemoryXilBench` evidence adapters. This is still an **evidence/provenance shell above the runner**, not a physical-bus SIL. |
| Open-loop SIL fault schedule | `crates/openbmp-runner/src/fc_bridge.rs` (`StimulusSchedule`) | Scenario `[fc.sil_stimulus]` faults, currently only `Bias { offset }`, applied at the read→publish seam with a domain-separated `DeterministicRng::for_stimulus_component`. Empty schedule ⇒ byte-identical run. The seed of the Tier-2 fault library. |
| FMI importer | `crates/openbmp-models/src/port.rs`, `crates/openbmp-fmi/src/lib.rs`, `crates/openbmp-fmi/examples/import_point_mass_fmu.rs`, `crates/openbmp-fmi/examples/import_fmu_final_sample.rs`, `scripts/check_fmi_import_point_mass.py`, `scripts/check_fmi_import_fmpy.py`, `scripts/check_fmi_reference_smoke.py` | `FmiArchive` reads stored and deflated FMU ZIP entries and parses the root `instantiationToken`, typed `Float64`/`Int32`/`UInt64` scalar variables, direct FMI 3 `Clock` variables, `intervalVariability`, decimal interval/shift metadata, and scalar-variable `clocks` value-reference lists. `materialize_archive_resources` safely writes packaged `resources/` entries for resource-backed FMUs. `Fmi3DynamicLibrary::open`/`open_archive_binary` loads the `.so` via `libloading`, probes `FMI3_COSIMULATION_SMOKE_SYMBOLS`, instantiates owned co-simulation components through `Fmi3CoSimulationInstance`, and exposes initialization, termination/free, typed `Float64`/`Int32`/`UInt64`/`Clock` get/set wrappers, plus `fmi3DoStep` early-return reporting. `FMI3_FMU_STATE_SYMBOLS` wrappers expose optional snapshot/restore/free state handles for rollback-capable FMUs. `fmi3_binary_entry_for_current_platform` infers safe standard FMI binary entries from `modelIdentifier` values for supported host targets. `Fmi3VariableBindings`, `Fmi3MasterPlan`, and `Fmi3SingleFmuMaster` provide a positive-step, single-FMU macro-step helper that publishes typed inputs, advances master time on complete or early-return timing, reads typed outputs, and can restore a pre-step FMU-state checkpoint on early return through `Fmi3RollbackPolicy::RestoreOnEarlyReturn`; `Fmi3MultiFmuMaster` routes typed `Float64`/`UInt64` connections across existing FMU instances with deterministic Gauss-Seidel/Jacobi ordering. `scripts/check_fmi_import_point_mass.py` packages the current-platform point-mass FMU, materializes its modelIdentifier binary through the OpenBMP importer twice, instantiates/initializes it, steps it through the typed master, byte-compares repeated CSV outputs, and compares the final sample to `export-point-mass-trace.toml`; `scripts/check_fmi_import_fmpy.py` cross-checks the same OpenBMP importer final sample against FMPy 0.3.26 on the packaged point-mass FMU; `scripts/check_fmi_reference_smoke.py` downloads the pinned Modelica Reference-FMUs v0.0.39 release, verifies its SHA-256/provenance record, imports FMI 3 `Dahlquist.fmu`, `VanDerPol.fmu`, pre-event `BouncingBall.fmu`, `Stair.fmu`, and resource-backed `Resource.fmu`, checks OpenBMP byte stability, and compares final samples to FMPy. **Partial** — no full clocked event scheduling, multi-FMU predictor/corrector loop, full Reference-FMU matrix, or FMI conformance claim yet. |
| FMI exporter artifact helper | `crates/openbmp-fmi/src/export.rs` | `Fmi3ExportModel` generates restricted FMI 3 `modelDescription.xml` with `instantiationToken`, direct typed `Float64`/`UInt64` variable elements, typed input start values, and `ModelStructure` output entries; runs `validate_restricted_fmi3_model_description_xml` before returning generated XML; escapes XML attributes; rejects duplicate names/value references; and writes a stored-entry `.fmu` archive containing `modelDescription.xml` plus a caller-supplied binary payload. `scripts/check_fmi_export_schema.py` also validates the generated point-mass metadata against a checked-in restricted XSD contract and the pinned official FMI 3.0 XSD set with `xmllint`. `scripts/check_fmi_export_fmpy.py` packages the current-platform point-mass FMU, executes it in FMPy 0.3.26, and compares the final sample to `export-point-mass-trace.toml`. `openbmp_point_mass_export_model` declares the ABI-aligned time, throttle, altitude, step, and velocity value-reference map; `openbmp_fmi_binary_entry_for_target` covers supported Linux/macOS/Windows cdylib paths; `openbmp_point_mass_fmi_binary_entry_for_target` covers modelIdentifier binary paths expected by standard importers; `write_openbmp_point_mass_fmu_archive_for_current_platform` writes the point-mass archive and returns the entry that the loader must materialize; and the generated archive round-trips through `openbmp-models::FmuArchive` into the expected typed binding plan. **Partial WP-10.5:** package construction, restricted schema-contract checks, official XSD validation, and point-mass FMPy execution exist; full OpenBMP plant mapping remains open. |
| FMI exporter C ABI | `crates/openbmp-fmi/Cargo.toml`, `crates/openbmp-fmi/src/export_abi.rs`, `crates/openbmp-fmi/tests/expected/export-point-mass-trace.toml` | `openbmp-fmi` now also builds as a `cdylib` and exposes restricted FMI 3 C symbols (`fmi3GetVersion`, `fmi3InstantiateCoSimulation`, initialization, typed Float64/Int32/UInt64 access, `fmi3DoStep`, snapshot/restore/free state, terminate/free instance, plus fail-closed unsupported FMI 3 symbols for full-symbol-table loader compatibility) backed by a deterministic point-mass-like export plant using the shared point-mass value-reference constants. Direct ABI tests prove fixed-step state advance, snapshot/restore, and a multi-step comparison against an independent native trace using a checked-in tolerance table. **Partial WP-10.5:** the ABI is a smoke surface, not full FMI conformance. |
| FMU metadata reader | `crates/openbmp-models/src/port.rs` | `FmuArchive` checks `modelDescription.xml` presence, parses FMI version/model identifier, and now records typed `FmiScalarVariable` entries with required `valueReference`, causality, and supported `Float64`/`Int32`/`UInt64` type. It also records direct FMI 3 `Clock` variables plus scalar-variable `clocks` dependencies for importer clock orchestration. The legacy input/output name lists remain for existing model-port callers. |
| Standards posture | `docs/standards-posture.md`, `docs/HAL.md`, `docs/real-rocket-integration.md`, `docs/safety-boundaries.md` | SIL "supported for host simulation"; PIL/HIL "scaffolded, not performed"; **no** ASAM-XIL / FMI / XTCE / CCSDS conformance claimed; `openbmp-bridge` ships the abstract schema and **no real drivers / bus protocols**. |

**Maturity verdict:** Tier 0 complete and honestly labelled. Tiers 1, 2, and 6 build directly on existing artifacts; Tier 5 builds on the already-`no_std` FC; Tiers 3–4 are net-new but self-contained.

---

## 3. Target architecture

### 3.0 New / changed crates

```text
CHANGED:
  openbmp-bridge   L7  + Transport trait (in-process + Unix-socket/TCP-loopback);
                        + LockstepSimMaster / LockstepClientStub blocking drivers;
                        + determinism-hash helper; + fault-transform ports (XIL).
  openbmp-fmi      —    + typed Float64/Int32/UInt64 value-reference API; modelDescription.xml
                        value-ref parse; co-sim master (Gauss-Seidel/Jacobi, early-return,
                        clock subset, optional FMU-state rollback); + exporter FMU surface.
  openbmp-fc       L4   + factored `fc_step()` entry point reachable from a no_std target
                        binary (NO new forbidden dependency; portability lock intact).
  openbmp-runner   L7   + wire FcBridge through Transport (in-process default unchanged);
                        + RtBench driver; + AFTS monitor wiring.
  openbmp-sil      —    + XIL port impls over the bridge; fault library; AFTS evidence.

NEW (proposed; placement justified in the WP that introduces it):
  openbmp-rt       L7   soft-real-time frame pacer + jitter accounting (HIL-software bench).
                        Shared with doc 12. std-only, host-side; never a dep of openbmp-fc.
  openbmp-afts     L2   forward IIP-geo-containment MONITOR (rule table + ballistic IIP).
                        Hardware-portable (no_std-capable); MAY be consumed by openbmp-fc
                        as a monitor, so it MUST obey the FC portability lock.
```

> **Crate-boundary justifications (reviewed before building):**
> `openbmp-rt` is L7 (host orchestration) and is *never* a dependency of `openbmp-fc`/`openbmp-hal` — it paces the simulator loop, not the controller, so it cannot break the portability lock. `openbmp-afts` is placed at L2 (a physics-layer monitor over forward state) and is `no_std`-capable so the FC *may* host it as an independent string; it depends only on `openbmp-core`/`openbmp-state`/`openbmp-physics` (the IIP propagator reuses the existing two-body/central-gravity integrator) and adds a new compile-fail test proving it cannot express an inverse solve (§6). Neither crate adds an up-layer edge or an FC→simulator edge.

### 3.1 SIL over a transport (Tier 1)

The simulator becomes the **time master** and the FC a **pure step function**: a non-iterative Gauss-Seidel co-simulation with **zero coupling delay**, because the master never advances past an unanswered frame (Gomes et al. 2018; Kübler & Schiehlen 2000).

**The `Transport` trait** (in `openbmp-bridge`, `std`-gated; the abstract schema stays `std`-free):

```rust
/// A bidirectional, message-boundary-preserving lockstep channel.
/// Implementations own framing; callers exchange whole BridgeMessages.
pub trait Transport {
    type Error: core::fmt::Debug;

    /// Block until exactly one framed BridgeMessage is available, decode it.
    fn recv(&mut self) -> Result<BridgeMessage, Self::Error>;

    /// Frame and send exactly one BridgeMessage; flush before returning.
    fn send(&mut self, message: &BridgeMessage) -> Result<(), Self::Error>;
}

/// In-process, allocation-light channel for the host fast path.
/// Backed by two std::sync::mpsc queues; deterministic, no wall-clock.
pub struct InProcessTransport { /* tx, rx */ }

/// Unix-domain-socket OR TCP-loopback stream transport (host-side only).
/// Length-prefixed (frame/deframe) over a buffered stream; blocking.
pub struct StreamTransport<S: Read + Write> { /* stream, read_buf */ }
```

**The master loop** (`LockstepSimMaster` in `openbmp-bridge`, driven by `openbmp-runner`). At FC rate `h_c` over a fixed-step plant integrator `h_p ≤ h_c`:

```text
handshake:  send Hello{PROTOCOL_VERSION, Simulator, max_payload};
            recv Hello{..} → validate_protocol_version() else Fault → abort.
per minor frame k:
  1. sample  y_k = g(x_k) + sensor_noise(seed, k)        // truth → SensorPacket
  2. send    Sensor(SensorPacket{ step=k, sim_time, y_k })
  3. BLOCK   recv → Command(cmd) | Ack(ack) | Fault(f)
             validate_command_for_sensor(&sensor, &cmd)  // step + to_bits time
             reject step≠k or time mismatch → Fault, fail-closed
  4. hold    u over [t_k, t_{k+1})                        // zero-order hold (ZOH)
  5. step    x_{k+1} = x_k + ∫ f(x,u) dt   (FIXED-step RK4 — NOT adaptive)
```

**Coupling-error formulation.** The only co-simulation error introduced by the boundary is the input-extrapolation error of the ZOH-held command over one minor frame:

```
e_coupling(t) = u(t) − u_k ,   t ∈ [t_k, t_{k+1}),   ‖e_coupling‖ = O(h_c)
```

first-order in `h_c` because the macro-step holds `u` constant (Kübler & Schiehlen 2000). It is *exactly* the staleness a real digital autopilot accepts between minor frames; it is **not** a numerical artifact and shrinks as `h_c → 0`. The plant integrator must be **fixed-step RK4**: adaptive steppers (DOPRI with error control) break bit-true replay because the substep sequence becomes input-dependent (`12-determinism-realtime-and-compute.md`).

**Determinism contract.** Same `(seed, scenario)` ⇒ byte-identical canonical telemetry. The master captures `SHA-256` over the canonical telemetry table and the actuator-command stream; the CI determinism lane asserts equality across two runs and across the `InProcess` vs `StreamTransport` backends (the wire path must not perturb a single bit).

The **in-process** transport is the default, so existing scenarios stay byte-identical: `FcBridge::step_from_truth` already does steps 1, 4, 5 in-process; Tier 1 routes steps 2–3 through `Transport::send`/`recv` with `InProcessTransport` as a transparent identity by default.

### 3.2 ASAM-XIL-style ports + fault injection (Tier 2)

ASAM XIL decouples the test **case** from the test **bench** so one scenario runs MIL→SIL→PIL→HIL unchanged (ASAM AE XIL, Generic Simulator Interface). OpenBMP implements the **pattern**, not the paid standard, and claims no conformance (consistent with `openbmp-sil`'s posture).

```rust
/// Model-Access port: read/write model parameters & signals, capture, stimulate.
pub trait MaPort {
    fn read(&self, id: &SignalId) -> Value;
    fn write(&mut self, id: SignalId, value: Value);          // parameter override
    fn create_capture(&self, signals: &[SignalId], trigger: Trigger,
                      downsampling: u32) -> CaptureHandle;     // pre/post-trigger buffer
    fn create_signal_generator(&self, id: SignalId,
                               waveform: SignalDescription) -> StimHandle;
}

/// Electrical-Error-Simulation port: "cutting the strings" at the wire boundary.
pub trait EesPort {
    fn set_error(&mut self, pin: PinId, error: ErrorType, duration: FaultWindow);
    fn clear_error(&mut self, pin: PinId);
}

pub enum ErrorType {                 // ASAM EESPort taxonomy, mapped to wire fields
    Open, ShortToGnd, ShortToBatt, ShortToPin(PinId), StuckValue(f64),
}

/// Symbolic-name → bench address mapping (the XIL Framework layer).
pub struct Mapping { /* name -> (SignalId, unit, scale, offset) */ }

/// Testbench lifecycle FSM (ASAM states, renamed to OpenBMP vocabulary).
pub enum TestbenchState { Uninitialized, Disconnected, Stopped, Running }
```

**Fault model as composable wire transforms** (Hsueh/Tsai/Iyer 1997; Arlat et al. 1990). A `FaultModel` is a transform on the framed `SensorPacket` (applied **before** the FC sees it) or `ActuatorCommandPacket` (applied **after** receive, **before** the plant) — generalizing today's `StimulusSchedule::Bias`:

| Class | Sensor fault `f_s(y_k)` | Actuator fault `f_a(u_k)` | Bus fault (on framed bytes) |
|---|---|---|---|
| stuck | `y = y_hold` | `u = u_hold` | — |
| bias | `y += b` | — | — |
| drift | `y += r·(t − t₀)` | — | — |
| dropout | suppress frame (FC dead-reckons) | — | drop frame |
| noise-burst | `y += σ_burst · n(seed,k)` | — | — |
| quantization | round to coarse LSB | — | — |
| saturate | — | `u = clamp(u, u_lo, u_hi)` | — |
| lag | — | `u̇ = (u_cmd − u)/τ` (first-order) | — |
| reversed | — | `u = −u` | — |
| delay | — | — | buffer N frames |
| duplicate | — | — | re-emit last frame |
| bit-flip | — | — | XOR mask (must trip FC CRC/sanity) |

Each fault carries `(trigger, duration, params)`; `trigger` may be conditional (`"inject at MECO+2s"`) for staged campaigns. Noise draws use `DeterministicRng::for_stimulus_component(seed, step, sensor_id, component)` (already present) so adding a fault never shifts another's stream. Impact is scored by the existing `EstimateVsTruthMonitor`; the evidence bundle records every injected fault for reproducibility. **No FC source changes** — the whole point of "cutting the strings."

### 3.3 Soft-real-time HIL-software bench (Tier 3) — `openbmp-rt`

Turns the host SIL into a soft-real-time bench that surfaces deadline/WCET behavior, **with no hardware** (OPAL-RT; Concurrent Real-Time; Liu & Layland 1973). Shared with `12-determinism-realtime-and-compute.md`.

```rust
pub struct RtConfig {
    pub minor_frame: core::time::Duration,   // h_c
    pub jitter_budget: core::time::Duration,
    pub mode: PacingMode,                     // RealTime | FasterThanReal | AsFastAsPossible
    pub cpu_affinity: Option<usize>,          // isolated core
}

pub trait FramePacer {
    /// Sleep until the next absolute minor-frame boundary t_k = t0 + k·h_c.
    /// Returns observed release time so the caller can detect overrun.
    fn wait_next_frame(&mut self, k: u64) -> FrameRelease;
}

pub struct FrameRelease {
    pub target: core::time::Duration,
    pub release: core::time::Duration,
    pub overrun: bool,                  // (release − target) > jitter_budget
}

pub struct JitterHistogram { /* p50, p90, p99, p99_9, max, overrun_count */ }
```

**Pacing math (absolute-time, drift-free):** wake at `t_k = t0 + k·h_c` via `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, t_k)`; record `t_release = now`; if `(t_release − t_k) > jitter_budget` then `overrun_count += 1`. Absolute targets prevent error accumulation that a relative `sleep(h_c)` would compound.

**Latency budget per frame:** `T_frame = T_sense + T_tx + T_fc_compute + T_rx + T_actuate ≤ h_c`; the run records `max(T_frame)` as a WCET estimate. **Jitter** is the distribution of `(t_release − t_k)`; the bench is "deterministic" only when `p99.9 jitter ≪ h_c` (the OPAL-RT/Concurrent-RT criterion: preemptible kernel + CPU shielding + NUMA-aware allocation). On Linux: `isolcpus` + `nohz_full` + `SCHED_FIFO`/`SCHED_DEADLINE`, `mlockall` + stack prefault to remove page-fault jitter. The plant integrator stays fixed-step RK4 to fit a constant compute budget.

**Schedulability:** for `n` rate groups under rate-monotonic priority, the Liu-Layland utilization bound `U ≤ n(2^{1/n} − 1)` must hold, and the measured (cycle-approximate) WCET must fit the per-frame budget.

> **Determinism caveat (honest).** Wall-clock *timing* is non-deterministic by nature; the jitter histogram is platform- and load-dependent **observability**, not a byte-stable artifact. The *functional* output (telemetry, actuator stream) remains byte-identical and is the only thing the determinism gate checks; the jitter record is reported as `experimental` measured data, never golden-compared.

### 3.4 FMI 3.0 co-simulation master (Tier 4)

Couples OpenBMP to/from the Modelica/Simulink/Dymola ecosystem and FMU-packaged GNC (FMI 3.0 spec; Cremona et al. 2022). Importer master loop:

```text
init:  c = fmi3InstantiateCoSimulation(...);
       fmi3EnterInitializationMode; fmi3SetFloat64(vr, x0); fmi3ExitInitializationMode.
step k (per FMU s in coupling order):
       fmi3SetFloat64(c_s, inputs U_s(t_k));
       st = fmi3DoStep(c_s, t_k, h, noSetPriorPoint=true,
                       &eventEncountered, &terminate, &earlyReturn, &lastSuccessfulTime);
       if earlyReturn: clamp h ← lastSuccessfulTime − t_k; re-extrapolate Jacobi inputs;
       fmi3GetFloat64(c_s, outputs Y_s).
```

- **Gauss-Seidel** ordering consumes just-computed upstream `Y` (tighter coupling); **Jacobi** uses `Y(t_{k-1})` (parallelizable).
- **Algebraic loops:** predictor/corrector master (Busch & Schweizer) — extrapolate `U`, `doStep`, correct, optionally roll back via `fmi3GetFMUState`/`fmi3SetFMUState` when `canGetAndSetFMUState`.
- **Bit-true replay** requires fixed `h` and FMUs free of wall-clock. OpenBMP refuses to import an FMU that fails the no-wall-clock contract for a determinism-labelled run (documented limitation, not a guarantee about third-party FMU internals).
- **Exporter:** emit `modelDescription.xml` + `libopenbmp_fmi.so` exposing the same C symbols so the OpenBMP plant runs inside another master.

```rust
/// Typed value-reference binding parsed from modelDescription.xml.
pub struct ValueRef(pub u32);
pub struct Fmi3CoSimulationInstance { /* lib: Fmi3DynamicLibrary, ptr: *mut c_void */ }

impl Fmi3CoSimulationInstance {
    pub fn enter_initialization_mode(&self, t0: f64, stop: Option<f64>, tolerance: Option<f64>) -> FmiResult<()>;
    pub fn exit_initialization_mode(&self) -> FmiResult<()>;
    pub fn set_float64(&self, vr: &[ValueRef], values: &[f64]) -> FmiResult<()>;
    pub fn get_float64(&self, vr: &[ValueRef], out: &mut [f64]) -> FmiResult<()>;
    pub fn do_step(&self, t: f64, h: f64) -> FmiResult<DoStepOutcome>;
    pub fn terminate(&mut self) -> FmiResult<()>;
}
pub struct DoStepOutcome { pub early_return: bool, pub last_successful_time: f64,
                           pub event: bool, pub terminate: bool }
```

`openbmp-fmi` already loads the `.so` and probes the lifecycle/value-access smoke symbols (`crates/openbmp-fmi/src/lib.rs`); this WP now has typed Float64/Int32/UInt64/Clock value-ref calls, `fmi3DoStep` early-return reporting, owned instantiate/init/terminate/free through `Fmi3CoSimulationInstance`, optional FMU-state snapshot/restore/free wrappers, a single-FMU early-return rollback policy, deterministic typed multi-FMU connection routing for Gauss-Seidel/Jacobi ordering, stored/deflated FMU archive reading plus `modelDescription.xml` instantiation-token, value-ref, and Clock metadata parsing (reusing `openbmp-models::FmuArchive`), safe FMU resource materialization, safe current-platform binary-entry inference for standard FMI 3 packages, a packaged point-mass importer smoke gate through `scripts/check_fmi_import_point_mass.py`, an OpenBMP-importer-vs-FMPy point-mass cross-check through `scripts/check_fmi_import_fmpy.py`, and pinned no-input Float64/Int32 Reference-FMU smokes through `scripts/check_fmi_reference_smoke.py`. The remaining importer-master work is full clocked event scheduling, a multi-FMU predictor/corrector loop beyond the current connection router, and full external Reference-FMU matrix validation. The `unsafe extern "C"` FFI is confined to `openbmp-fmi` (a host-only crate, never a dep of `openbmp-fc`).

### 3.5 PIL on an emulated flight ISA (Tier 5)

Runs the byte-identical flight binary cross-compiled for the **flight target ISA** inside Renode (deterministic shared virtual time, MIT) or QEMU (broad ISA, GPL — invoked as a separate process, no source linkage). Exposes miscompiles, endianness/word-size bugs, FP-determinism differences, and (with a timing model) WCET behavior the host SIL hides (MathWorks PIL numerical-equivalence; Renode/Antmicro).

**Precondition already met:** `openbmp-fc` is `#![cfg_attr(not(feature="std"), no_std)]`, `forbid(unsafe_code)`, `extern crate alloc`, and compiles for `thumbv7em-none-eabihf` today. This WP factors a `fc_step()` entry that reads a `SensorPacket` from a known symbol/UART and writes an `ActuatorCommandPacket` back, and builds a `target/` firmware image.

**Couple modes:** (a) **GDB-stub semihosting** — master writes the `SensorPacket` to a known symbol, breakpoints the step entry, continues, reads the `ActuatorCommandPacket` back, driving lockstep over the GDB Remote Serial Protocol. (b) **virtual UART/socket** — target firmware blocking-reads a UART that Renode/QEMU maps to a host socket; **reuse the exact `openbmp-bridge` frame/codec** so the same `Transport`/`LockstepSimMaster` drives PIL unchanged. Renode `.resc` sets the machine quantum + `sync` so virtual time advances exactly `h_c` per frame regardless of host speed → faster-than-real-time **and** deterministic.

**Cycle-approximate WCET:** instrument the step with `DWT->CYCCNT` (Cortex-M) or Renode executed-instruction-count × CPI estimate; assert `frame_cycles ≤ budget`. True cycle-accuracy needs QEMU-CAS/TQSIM-class backends (the expensive long pole — out of scope; cycle-approximate gets ~80% of the WCET value for ~20% of the effort).

**Bit-true PIL contract:** same firmware + same `SensorPacket` stream + Renode deterministic time ⇒ identical actuator stream and instruction trace; the **host SIL (Tier 1) is the oracle** for numerical equivalence.

### 3.6 cFS apps as flight software in the loop (Tier 6)

Elevates "flight SW in the loop" from OpenBMP's own FC logic to **actual heritage flight software**: NASA cFS (Apache-2.0), flown on dozens of missions, with publish/subscribe, table-driven, deterministic 1 Hz major-frame apps (SCH, SB, SC, LC, HS). OpenBMP becomes the "hardware" cFS talks to via a sim adapter, exactly as NOS3 does (study NOS3 as the reference architecture; **couple, do not vendor C into the Rust crates**).

**Integration pattern (NOS3-shaped):**
- A **sim-hardware cFS app** (C) subscribes to actuator-command MsgIds and publishes sensor-telemetry MsgIds on the cFE Software Bus (SB).
- A **Rust↔cFE-SB bridge** maps those SB messages to/from `openbmp-bridge` `SensorPacket`/`ActuatorCommandPacket` frames over the `Transport` (a NOS-Engine-style local transport).
- Drive **cFS TIME** from OpenBMP's sim clock (not wall-clock): the cFE TIME 1 Hz Major Time Synchronization Signal advances exactly one major frame per OpenBMP control epoch → lockstep determinism.
- For PIL, cross-compile the cFS build (OSAL → RTEMS/FreeRTOS/POSIX) for the flight ISA under Renode/QEMU (Tier 5 ∘ Tier 6).

This is the deepest open rung of "real flight SW in the loop" and the strongest credibility signal. cFS stays an **external submodule/sibling repo**; the only in-repo artifact is the Rust bridge + a thin C sim-hardware app kept under its own build, never linked into the deterministic Rust kernel's hot path.

### 3.7 AFTS forward containment monitor (Tier 2/3 capstone) — `openbmp-afts`

An independent, GPS/IMU-driven, rule-based **monitor** that predicts the Instantaneous Impact Point (IIP) — *where the vehicle would impact if thrust ended now* — and compares it to pre-loaded containment regions, raising a **latched terminate flag + which-rule-fired** on a boundary violation, gate miss, or out-of-corridor condition. It answers *"is the vehicle inside its licensed flight corridor?"* — it **does not and must not steer toward a point** (Bull & Lanzi, NTRS 20080044860; NAFTU concept; RCC 319).

**IIP prediction (free-fall, drag-optional, rotating-Earth).** From the current ECEF/ECI state `(r, v)`:
- *Vacuum closed form:* form the coasting conic from `(r, v)` (energy `ε = v²/2 − GM_earth/‖r‖`, eccentricity from the angular-momentum and eccentricity vectors), solve time-of-flight to radius `‖r‖ = R_earth`, then rotate the impact longitude by `ω_earth · t_flight` (standard rotating-Earth IIP correction).
- *Numeric:* integrate `r̈ = −GM_earth · r/‖r‖³` (+ optional drag) until `‖r‖ = R_earth`; the surface intersection is the IIP. The propagator **reuses the existing two-body/central-gravity integrator** (`crates/openbmp-runner/src/footprint.rs`, `integrator.rs`). Earth constants are referenced symbolically as `GM_earth`, `J2`, `R_earth` (never written as literals — `00` §3.5).

**Containment rules (declarative TOML, NAFTU-style table, never code):**
1. **point-in-polygon** of IIP vs allowed / keep-out polygons (ray-casting / winding number);
2. **gate crossing** — trajectory passes ordered gates within time windows;
3. **hard corridor limits** — altitude / velocity / flight-path-angle bounds;
4. **green/red-zone** debris-footprint containment.

**Decision:** `terminate = monotone-OR(rule_violations)` → a **single latched** terminate that never un-latches (real two-string AFTS behavior). The monitor runs on its **own state estimate**, separate from the guidance loop. Scored against truth: the evidence records the terminate boolean, the firing rule id, and the IIP-vs-truth residual — **never** an aimpoint, downrange, or miss distance.

```rust
pub trait AftsMonitor {
    /// Forward-only: shared state in, latched decision out. No inverse surface.
    fn evaluate(&mut self, state: &VehicleStateEstimate) -> AftsDecision;
}
pub struct AftsDecision { pub terminate: bool, pub fired_rule: Option<RuleId>,
                          pub iip_geodetic: GeodeticPoint }
// NOTE: there is deliberately NO `fn solve_burn_to_reach(polygon)` / no aimpoint API.
```

### 3.8 Fidelity tiers

| Tier | Name | Deliverable | Builds on | Earns |
|---|---|---|---|---|
| **T0** | Algorithm-in-the-loop (current) | host-compiled FC linked in-process; schema + validators exist, no transport | — | (baseline) |
| **T1** | True SIL over a transport | `Transport` trait + in-process & socket impls; blocking step-and-ack master; determinism-hash gate | T0 | `validated-toy` |
| **T2** | Fault injection + XIL ports | wire-transform fault library; `MaPort`/`EesPort` + symbolic mapping + lifecycle FSM; scored by `EstimateVsTruthMonitor` | T1 | `validated-toy` |
| **T3** | Soft-real-time HIL-software bench | `openbmp-rt` fixed-step plant, absolute-time pacing, overrun + p99.9 jitter histogram | T1 | `validated-toy` (functional) / `experimental` (timing) |
| **T4** | FMI 3.0 co-simulation | conformant importer master + exporter FMU; Reference-FMU & `fmpy` cross-check | T1 | `research` |
| **T5** | PIL on emulated ISA | factored `no_std` FC binary on `thumbv7em`/`riscv32` under Renode; cycle-approx WCET | T1, T2 | `validated-toy` → `research` (numerical-equivalence) |
| **T6** | cFS apps as flight SW | cFS behind the HAL via sim-hardware app + Rust↔SB bridge; cFS TIME from sim clock | T1 (T5 optional) | `research` |
| **A0** | AFTS containment monitor | `openbmp-afts` forward IIP + rule table; latched terminate; MMS/closed-form verified | T1 (T2 to inject) | `validated-toy` → `research` |

Each tier ships independently verifiable value; **nothing above T0 is required for the one below it to be useful.**

---

## 4. Invariant preservation

Concretely, for this dimension (`00` §3; `13` §3):

1. **Byte-determinism.** The plant integrator is fixed-step RK4 (no adaptive substeps), all fault noise derives from `DeterministicRng::for_stimulus_component` with `(seed, step, sensor_id, component)` domain separation, and the `Transport` path performs no wall-clock reads on the functional path. The determinism gate hashes canonical telemetry + the actuator stream and asserts equality across two runs **and** across the `InProcess`/`Stream` backends. Operand order in any new weighted sum is locked; no `f64::mul_add`. The `openbmp-rt` jitter histogram is explicitly excluded from byte-comparison (it is wall-clock observability, reported `experimental`).
2. **Byte-stable-by-default.** Every tier is off until a scenario opts in. The default `FcBridge` path stays in-process and byte-identical; a `[fc.transport]` block selects a local bridge transport backend; `[fc.faults]` arms the fault library (today's empty `StimulusSchedule` already yields a byte-identical run); `[realtime]` enables `openbmp-rt`; `[fc.afts]` arms the monitor. No golden archive changes until a scenario opts in (`00` §3.2; pattern of `[vehicle.bending]`).
3. **FC hardware-portability lock.** `openbmp-fc` gains **no** new dependency on `openbmp-sim`/`-cli`/`-runner`/`-scenario`/`-telemetry`/`-bridge`/`-aerothermal`. The `Transport`, master loop, `openbmp-rt`, and FMI FFI all live at L7/host crates that depend *down* on `openbmp-fc`, never the reverse. The factored `fc_step()` (Tier 5) stays inside `openbmp-fc` using only hardware-portable crates. `openbmp-afts` is `no_std`-capable and depends only on `openbmp-core`/`-state`/`-physics`, so if the FC hosts it as a string the lock holds. `fc_dependency_tripwire.rs` stays green.
4. **Lockstep-clock & no-hot-path-allocation.** The FC reads time only through `Clock` (`crates/openbmp-fc/src/clock.rs`); the socket/PIL transports inject a `SimulatedClock` bound to the lockstep step, never `Instant::now`. The per-frame `Transport::send`/`recv` path reuses a preallocated read buffer and the cached binding/topic tables — no per-tick allocation or clone. `fc_lints.rs` stays green.
5. **Four-pillar provenance.** Fault schedules, AFTS rule tables, FMI fixtures, and `.resc` PIL scripts live under `scenarios/`/`data/`/`tests/fixtures/` with a sibling `provenance.md` + SHA-256 pin, never inline in `*.rs`. FMI Reference FMUs and cFS are external (submodule), licence + provenance recorded. **No literal `GM_earth`/`J2`/`R_earth` numerals** anywhere — the AFTS IIP math refers to them symbolically; `inline_data_tripwire.rs` stays green.
6. **Validation labels.** Each tier declares exactly one of `experimental`→`checked`→`validated-toy`→`research`, justified by its evidence (§5), with a tolerance-table TOML for every numeric claim. No tier claims `flight-qualified`/`certified`; the timing histogram is `experimental` self-measured data.
7. **Forward-only locks tighten.** The bus carries `SensorPacket`/`ActuatorCommandPacket` only — never a target coordinate. Fault injection perturbs perception/actuation and exposes no aimpoint solve. `openbmp-afts` adds a **new compile-fail tripwire** proving the monitor surface cannot express an inverse "burn-to-reach-polygon" solution (§6). The closed `SilCheck` enum gains no range/aimpoint/impact/miss-distance variant.
8. **Requirements traceability.** Each WP adds `requirements.toml` entries with verification evidence; `check_requirements_traceability.py` stays green.

---

## 5. V&V plan

Verification follows the five-layer ladder (`docs/verification.md`): MMS code-verification → analytic → code-to-code → public benchmark → UQ. Tolerance tables ship as `expected.toml` per case.

### 5.1 Verification cases

| Case | Method | What it checks | Tier |
|---|---|---|---|
| **Bit-true self-determinism** | self-referential | `SHA-256(canonical telemetry)` identical across two `(seed, scenario)` runs **and** across `InProcess`/`Stream` backends | T1 |
| **ZOH coupling-error scaling** | analytic | `openbmp-bridge::zoh` checks that `‖e_coupling‖` halves as `h_c` halves against `crates/openbmp-bridge/tests/expected/zoh-coupling.toml` (confirms the `O(h_c)` order) | T1 |
| **Fault-transform identity** | analytic | empty fault schedule ⇒ byte-identical run; a single `Bias` matches the closed-form shift exactly | T2 |
| **FDIR-response correctness** | scenario | injected sensor fault ⇒ `EstimateVsTruthMonitor` shows the expected residual rise and `FdirIsolatesExactly` mask | T2 |
| **Real-time jitter** | self-measured | `cyclictest`-style p50/p99/p99.9/max histogram on a documented host vs `p99.9 ≪ h_c` | T3 |
| **Schedulability** | analytic | rate-monotonic set meets `U ≤ n(2^{1/n}−1)`; measured WCET fits the per-frame budget | T3 |
| **FMI Reference-FMU cross-check** | code-to-code | import each Reference FMU (BSD-2); compare master outputs to published traces and to `fmpy` as an independent master | T4 |
| **FMI round-trip** | code-to-code | OpenBMP exporter FMU run inside `fmpy` reproduces the native plant trace within tolerance | T4 |
| **PIL numerical-equivalence** | self-referential | cross-compiled target binary produces the same actuator stream as the host SIL for the same `SensorPacket` stream (MathWorks PIL criterion) | T5 |
| **cFS code-to-code (NOS3)** | code-to-code | same cFS app set under NOS3's 42 dynamics and under OpenBMP's plant; compare telemetry | T6 |
| **AFTS IIP closed-form / MMS** | analytic | numerically propagated IIP vs closed-form conic impact point (rotating-Earth corrected); hand-computed point-in-polygon containment cases | A0 |
| **External-validity sanity** | LOCAL only | ascent timeline/altitude vs third-party Falcon-class webcast telemetry — gross-error check, **never committed, never a flight/targeting input** | T1 |

### 5.2 Tolerance tables (representative; ship as `expected.toml`)

| Quantity | Case | Tolerance | Label earned |
|---|---|---|---|
| canonical telemetry SHA-256 | self-determinism | byte-identical (exact) | `validated-toy` |
| `‖e_coupling‖` halving ratio | ZOH scaling | `0.5 ± 0.05` per `h_c` halving | `validated-toy` |
| Reference-FMU output | FMI cross-check | `rel ≤ 1e-6` vs `fmpy`; `rel ≤ 1e-4` vs published trace | `research` |
| actuator stream | PIL equivalence | bit-identical OR `‖Δu‖ ≤ 1 ULP` (document FP-mode delta if non-zero) | `research` |
| IIP geodetic lat/lon | AFTS closed-form | `≤ 1e-6 rad` (vacuum, MMS) | `validated-toy`→`research` |
| p99.9 frame jitter | RT bench | reported only; flag `overrun` if `> jitter_budget` | `experimental` |

### 5.3 Label per tier

T1/T2/T3-functional/A0(MMS) earn **`validated-toy`** (self-referential + analytic). T4 (Reference-FMU + `fmpy`), T5 (numerical-equivalence vs host oracle), T6 (NOS3 code-to-code), and A0 with the closed-form public benchmark earn **`research`**. The T3 timing histogram is **`experimental`** measured data. **No tier earns `flight-qualified`/`certified`** — the standing posture (`docs/standards-posture.md`).

---

## 7. Dependencies on other parity docs

- **`12-determinism-realtime-and-compute.md`** — co-owns `openbmp-rt`; the fixed-step / locked-operand-order / deterministic-pacing requirements, checkpointing, and the determinism byte-diff gate originate there. T3 is a joint deliverable; T1's determinism contract reuses doc 12's hash machinery. **Tightest coupling.**
- **`09-sensors-navigation-and-actuators.md`** — defines the `SensorPacket` payload semantics (force-based IMU, pseudorange GNSS, actuator dynamics); the fault library's measurement-domain transforms degrade exactly those fields, and actuator-lag/saturation faults compose with the actuator dynamics models.
- **`06-gnc-coupled-mimo-and-control.md`** — supplies the FC under test (allocation, LQR/LQG, FDIR); the FDIR-response V&V case (§5) exercises doc 06's fault isolation through the fault library.
- **`08-environment-gravity-and-frames.md`** — the AFTS IIP propagator consumes the central-gravity / rotating-Earth frame machinery; symbolic `GM_earth`/`J2`/`R_earth` come from doc 08's environment models.
- **`11-monte-carlo-uq-and-validation.md`** — fault-injection campaigns are Monte Carlo cases; `openbmp-mc` orchestrates staged-fault ensembles and the credibility (7009B) accounting of XIL evidence bundles.
- **`07-trajectory-optimization-and-mission-design.md`** — shares the forward-only escalation posture (`00` §6); the AFTS monitor and the trajopt terminal-condition vocabulary lock are the two sharpest dual-use guards and must stay mutually consistent.
- **`01-flexible-multibody-dynamics.md`**, **`05-propulsion-high-fidelity.md`** — supply the fixed-step-integrable plant the SIL/RT loop drives; the plant must remain RK4-integrable for bit-true replay.

**Ordering:** Phase D (`00` §5). T1 needs only a stable FC boundary (present). T3 needs doc 12's pacing substrate. T4–T6 are independent tracks. The dimension is largely independent of physics fidelity and can progress on its own track once the FC boundary is stable (`00` §5 critical-path note d).

---

## 8. Open-source leverage

| Tool | Licence | Mode | Use |
|---|---|---|---|
| **NASA cFS** (cFE, OSAL, PSP, SCH, SB, LC, HS, SC) | Apache-2.0 | **couple** | Real cFS apps as flight SW behind the HAL via a sim-hardware cFS app + Rust↔cFE-SB bridge; drive cFS TIME from the sim clock. External submodule; do **not** vendor C into the Rust crates. |
| **NOS3** (NASA Operational Simulator for Small Satellites) | NASA OSS (components Apache-2.0/MIT) | **ingest** (reference architecture) | How to wire cFS + a dynamics sim + a ground system over a sim transport; study the sim-hardware-app + NOS-Engine pattern. Verify per-component licence before any reuse. |
| **Renode** (Antmicro) | MIT | **couple** | Emulate the flight target (Cortex-M/A, RISC-V, LEON) with deterministic shared virtual time; drive lockstep over GDB-stub or virtual-UART using the existing bridge frames. Invoke as an external tool, script via `.resc`. |
| **QEMU** | GPL-2.0 (emulator) | **couple** | Alternative PIL backend (broad ISA, GDB stub, semihosting). Invoke as a **separate process** — no GPL linkage into OpenBMP's permissive Rust crates. Pair with cycle-approximate timing for WCET. |
| **FMI Reference FMUs** | BSD-2-Clause | **ingest / couple** | Ready importer test targets for the FMI 3.0 master; CI fixtures. |
| **fmpy** | BSD-2-Clause | **couple** | Reference master to cross-check OpenBMP's importer output; host for OpenBMP's exporter FMU. |
| **42** (NASA GSFC attitude/orbit dynamics) | NASA OSS | **ingest** (V&V) | Code-to-code cross-validation of attitude/orbit propagation; the dynamics back-end exemplar used by NOS3. Confirm licence before reuse. |
| **NASA Trick** | NASA OSS (permissive) | **ingest** (design reference) | Job-ordering, fixed-step scheduling, deterministic record/replay + checkpoint-restart patterns to mirror in Rust. Not a runtime dependency. |
| **Linux PREEMPT_RT / SCHED_DEADLINE / `cyclictest`** | GPL-2.0 | **couple** (host platform) | Run the fixed-step plant under an RT kernel on isolated cores; `cyclictest` characterizes the host's achievable jitter floor and sizes the per-frame budget. Host platform, no source linkage. |
| **NAFTU / RCC 319** | NASA Software User Agreement (NOT redistributable) / controlled standard | **ingest** (concepts only) | Rule-table structure and containment-rule taxonomy to shape the declarative AFTS-monitor TOML. Use only published architectural **concepts**; keep the monitor a clean-room forward implementation; do **not** vendor the software. |

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, green on the full `13` §2 gate set. Effort sizes from the research ladder.

---

**WP-10.1 — Transport trait + lockstep step-and-ack master (true SIL over a transport)**
- **goal:** Convert the in-process algorithm-SIL into a real binary-behind-a-transport SIL: add a `Transport` trait with an in-process channel and a Unix-socket/TCP-loopback impl, drive the existing lockstep step-and-ack loop across it, and add a determinism CI gate. This is the highest-ROI rung and builds directly on the existing schema + validators.
- **fidelity_tier:** T1
- **depends_on:** []
- **new_crates:** none (extend `openbmp-bridge` L7)
- **requirements:** [REQ-HIL-001], [REQ-HIL-002], [REQ-HIL-003]
- **touched:** `crates/openbmp-bridge/src/{transport.rs,lib.rs,error.rs,lockstep.rs}`, `crates/openbmp-scenario/src/{document.rs,scenario.rs,lib.rs}`, `crates/openbmp-runner/src/fc_bridge.rs`, `crates/openbmp-runner/tests/fc_transport.rs`, `crates/openbmp-runner/src/sil.rs`
- **approach:** §3.1. `Transport { recv/send }`; `InProcessTransport` (mpsc), `StreamTransport<S: Read+Write>` (frame/deframe over a buffered stream), `SplitStreamTransport<R: Read, W: Write>` for child stdio, TCP loopback helpers, and Unix-domain-socket helpers. `LockstepSimMaster` runs the handshake (`BridgeHelloPacket` + `validate_protocol_version`) then the per-frame block-on-matching-command loop (`validate_command_for_sensor`, fail-closed on mismatch). Default scenario path remains direct and byte-stable; v3 `[fc.transport] mode = "in_process"`, `mode = "tcp_loopback"`, Unix-only `mode = "unix_loopback"`, and `mode = "external_process"` select local or spawned-process sensor/effector/engine-command bridge subsets; `peer_protocol_version` plus `step_offset` / `time_offset` packet faults exercise fail-closed protocol and lockstep mismatch paths through scenarios. Canonical telemetry SHA-256 and actuator command-stream SHA-256 are gated; external process coupling is stdio-only and does not claim real bus/device-driver support.
- **acceptance:**
  - new transport off by default (`InProcess`); canonical goldens byte-identical
  - determinism gate: two `(seed, scenario)` runs and the `InProcess` vs `Stream` backends produce byte-identical telemetry SHA-256
  - ZOH coupling-error halves as `h_c` halves, in a tolerance table (`0.5 ± 0.05`)
  - protocol-version mismatch and step/time mismatch fail-closed with a `Fault`, tested through a scenario
  - all §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** forward-only; bus carries `SensorPacket`/`ActuatorCommandPacket` only; no driver/bus protocol shipped (reaffirm `safety-boundaries.md`)
- **est_effort:** 1–2 weeks
- **parity_ceiling:** does not validate target-ISA behavior, real-time timing, or any hardware electricals

---

**WP-10.2 — Fault-injection library + ASAM-XIL-style ports**
- **goal:** Composable wire-transform fault library implementing the sensor/actuator/bus/timing taxonomy ("cutting the strings"), exposed through `MaPort`/`EesPort`-shaped traits with a symbolic signal-name mapping and a testbench lifecycle FSM; scored by the existing `EstimateVsTruthMonitor`. Reproducible robustness/FDIR campaigns with an evidence bundle.
- **current_state:** Initial bridge-level fault transforms exist in `openbmp-bridge`: ordered scalar rules over `BridgeScalarSignal`s mutate scalar fields in `SensorPacket`/`ActuatorCommandPacket`, scalar rules include deterministic drift and bounded rule-local noise-burst streams, packet rules can drop, duplicate, delay, or mark as bit-flipped sensor/command frames, applied rule ids are returned, absent optional signals are skipped, and saturation/quantization/drift/noise/delay/bit-flip parameters are validated. v3 `[fc.transport_faults]` is parsed/validated by `openbmp-scenario` and mapped by the runner onto the opt-in `[fc.transport]` path. `MaPort`/`EesPort`-shaped traits, symbolic mapping, and a deterministic testbench lifecycle FSM now exist in `openbmp-bridge`; `openbmp-sil::InMemoryXilBench` implements them and emits SIL evidence records. Native package runs can attach XIL records to `EvidenceBundle`, including observed runs with `EstimateVsTruthMonitor` output. Full external-bench replay and ASAM-XIL conformance remain open.
- **fidelity_tier:** T2
- **depends_on:** [WP-10.1]
- **new_crates:** none (extend `openbmp-bridge` + `openbmp-sil`)
- **requirements:** [REQ-XIL-001]
- **touched:** `crates/openbmp-bridge/src/{fault.rs(new),ports.rs(new)}`, `crates/openbmp-sil/src/lib.rs`, `crates/openbmp-runner/src/fc_bridge.rs` (generalize `StimulusSchedule`)
- **approach:** §3.2. `FaultModel` transforms on framed `SensorPacket`/`ActuatorCommandPacket`; the table of stuck/bias/drift/dropout/noise-burst/quantization (sensor), stuck/saturate/lag/reversed (actuator), delay/duplicate/bit-flip (bus). Conditional triggers (`"MECO+2s"`). `MaPort`/`EesPort` traits + `Mapping` + `TestbenchState` FSM; no conformance claim. Noise via `DeterministicRng::for_stimulus_component`.
- **acceptance:**
  - empty fault schedule ⇒ byte-identical run (off by default)
  - single `Bias` matches the closed-form shift exactly; each fault type has a unit + scenario test
  - injected sensor fault ⇒ expected residual rise + `FdirIsolatesExactly` mask via `EstimateVsTruthMonitor`
  - a test case replays MIL→SIL unchanged via the symbolic mapping
  - all §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** perturbs perception/actuation only; exposes no aimpoint solve; closed check enum unchanged
- **est_effort:** 2–3 weeks
- **parity_ceiling:** does not reproduce real bus electrical faults or hardware SEU behavior

---

**WP-10.3 — Soft-real-time HIL-software bench (`openbmp-rt`)**
- **goal:** Fixed-step RK4 plant with absolute-time frame pacing, overrun detection, p50/p99/p99.9/max jitter and host frame-execution histograms, plus real-time and faster-than-real-time modes. No hardware required — surfaces deadline/frame-budget behavior the host SIL hides. CPU isolation / `SCHED_FIFO` / `mlockall` host controls remain future hardening.
- **requirements:** [REQ-RT-002]
- **fidelity_tier:** T3
- **depends_on:** [WP-10.1]
- **new_crates:** `openbmp-rt` (L7) — shared substrate exists as a host orchestration crate and is consumed by the runner through `[realtime]`; optional `[[realtime.task]]` budgets produce a Liu-Layland schedulability advisory; never a dep of `openbmp-fc`/`openbmp-hal`; cannot break the FC lock.
- **touched:** `crates/openbmp-rt/`, `crates/openbmp-runner/src/rt.rs`, `crates/openbmp-runner/src/point_mass.rs`, `crates/openbmp-runner/src/rigid_body.rs`, `Cargo.toml`
- **approach:** §3.3. `FramePacer::wait_next_frame` uses absolute minor-frame targets; `RtConfig` carries minor frame, jitter budget, and pacing mode; `JitterHistogram` and the runner frame-execution summary report timing outside telemetry. Functional output stays byte-identical; timing records are `experimental` observability.
- **acceptance:**
  - off by default (`[realtime]` block); functional telemetry byte-identical to the non-RT run
  - jitter histogram reports p50/p99/p99.9/max + overrun count on a documented host
  - rate-monotonic schedulability `U ≤ n(2^{1/n}−1)` reported analytically from declared `[[realtime.task]]` budgets; measured host frame execution is compared to the wall-frame budget, while task-internal WCET capture remains outside this slice
  - timing record labelled `experimental`, excluded from the byte-diff gate
  - all §2 gates green
- **validation_label:** `validated-toy` (functional) / `experimental` (timing)
- **dual_use_note:** far from line — pure rigor/observability
- **est_effort:** 3–4 weeks
- **parity_ceiling:** soft-real-time only; not a hard-RT kernel, no bus electricals, no flight-part timing
- **note:** coordinate with `12-determinism-realtime-and-compute.md` (shared crate)

---

**WP-10.4 — FMI 3.0 co-simulation importer master**
- **goal:** Conformant importer master: typed Float64/Int32/UInt64 value refs, `modelDescription.xml` value-ref parse, Gauss-Seidel/Jacobi ordering, early-return + clock subset, optional FMU-state rollback. Validate against the FMI Reference FMUs and `fmpy`.
- **current_state:** Partial implementation. `openbmp-models::FmuArchive` reads stored and deflated FMU ZIP entries, parses the FMI 3 root `instantiationToken`, required `valueReference` fields, supported `Float64`/`Int32`/`UInt64` scalar variables, direct FMI 3 `Clock` variables, and scalar-variable `clocks` dependencies; `openbmp-fmi` builds typed bindings/master plans, safely materializes FMU `resources/` entries, infers safe standard FMI binary entries for supported current-platform targets, owns co-simulation instantiation/init/termination/free through `Fmi3CoSimulationInstance`, calls typed `fmi3Set/GetFloat64`, `fmi3Set/GetInt32`, `fmi3Set/GetUInt64`, `fmi3Set/GetClock`, and `fmi3DoStep` with early-return reporting through host FFI, exposes optional `fmi3Get/Set/FreeFMUState` wrappers, provides `Fmi3SingleFmuMaster` for one-FMU macro steps, can restore a pre-step FMU-state checkpoint on early return through `Fmi3RollbackPolicy::RestoreOnEarlyReturn`, and provides `Fmi3MultiFmuMaster` for typed `Float64`/`UInt64` connection routing across existing FMU instances with deterministic Gauss-Seidel/Jacobi ordering. `scripts/check_fmi_import_point_mass.py` packages the current-platform point-mass FMU, runs the OpenBMP importer path twice from fresh materialized binaries, byte-compares the CSV outputs, and validates the final sample against the native trace tolerance table; `scripts/check_fmi_import_fmpy.py` cross-checks that OpenBMP importer output against FMPy 0.3.26 on the same packaged point-mass FMU; `scripts/check_fmi_reference_smoke.py` downloads the SHA-pinned Modelica Reference-FMUs release, records provenance, imports FMI 3 `Dahlquist.fmu`, `VanDerPol.fmu`, pre-event `BouncingBall.fmu`, `Stair.fmu`, and resource-backed `Resource.fmu`, checks repeated OpenBMP byte stability, and compares final outputs to FMPy. Full clocked event scheduling, multi-FMU predictor/corrector orchestration beyond basic connection routing, and the complete external Reference-FMU matrix remain open.
- **fidelity_tier:** T4
- **depends_on:** [WP-10.1]
- **new_crates:** none (extend `openbmp-fmi`)
- **touched:** `crates/openbmp-fmi/src/lib.rs`, `crates/openbmp-fmi/examples/import_point_mass_fmu.rs`, `crates/openbmp-fmi/examples/import_fmu_final_sample.rs`, `scripts/check_fmi_import_point_mass.py`, `scripts/check_fmi_import_fmpy.py`, `scripts/check_fmi_reference_smoke.py`, `crates/openbmp-models/src/port.rs` (value-ref parse), `tests/fixtures/fmi/**` (Reference FMUs, provenance)
- **approach:** §3.4. Typed `instantiate_co_simulation`, `enter_initialization_mode`, `exit_initialization_mode`, `terminate`, `free_instance`, `set_float64`/`get_float64`/`set_int32`/`get_int32`/`set_uint64`/`get_uint64`/`set_clock`/`get_clock`/`do_step` over the already-loaded `Fmi3DynamicLibrary`; parse instantiation tokens, value refs, and Clock metadata from stored or deflated `modelDescription.xml` entries (reuse `FmuArchive`); materialize safe `resources/` entries for FMUs that need packaged resource files; infer safe standard FMI binary entries from `modelIdentifier`; master plan carries coupling order + `DoStepOutcome` early-return state; optional `get_fmu_state`/`set_fmu_state`/`free_fmu_state` wrappers provide the rollback primitive; `Fmi3SingleFmuMaster` performs typed input publication, stepping, early-return time advancement, typed output reads, and checkpointed early-return rollback for one FMI component; `Fmi3MultiFmuMaster` routes typed connections in Jacobi mode from the previous output cache and in Gauss-Seidel mode from current-step upstream outputs where available; FMPy acts as an independent master for the packaged point-mass importer cross-check and the pinned supported Reference-FMU smoke matrix. Remaining work is full clocked event scheduling, multi-FMU predictor/corrector orchestration beyond this router, and complete external Reference-FMU validation. `unsafe extern "C"` FFI stays confined to `openbmp-fmi`.
- **acceptance:**
  - each FMI Reference FMU (BSD-2) imported; outputs match published traces (`rel ≤ 1e-4`) and `fmpy` (`rel ≤ 1e-6`) in a tolerance table
  - fixed-`h` import is bit-true across reruns; an FMU using wall-clock is refused for a determinism-labelled run
  - early-return path tested (clamp `h`, re-extrapolate)
  - Reference FMUs carry `provenance.md` + SHA pin + licence note
  - all §2 gates green
- **validation_label:** `research`
- **dual_use_note:** far from line — ecosystem interchange plumbing
- **est_effort:** 5–8 weeks (importer)
- **parity_ceiling:** no conformance certification; third-party FMU internal determinism not guaranteed

---

**WP-10.5 — FMI 3.0 exporter FMU**
- **goal:** Emit `modelDescription.xml` + `libopenbmp_fmi.so` exposing the FMI3 co-sim C symbols so the OpenBMP plant runs inside another master (`fmpy`).
- **current_state:** Partial implementation. `openbmp-fmi::export::Fmi3ExportModel` generates a restricted FMI 3 `modelDescription.xml` from typed value-reference metadata using the FMI 3 direct typed-variable shape, `instantiationToken`, typed input start values, and `ModelStructure` output entries, validates that restricted schema contract before returning XML, then packages it with a caller-supplied binary payload in a stored-entry `.fmu`; `scripts/check_fmi_export_schema.py` validates the generated point-mass metadata against a checked-in restricted XSD contract and the pinned official FMI 3.0 XSD set with `xmllint`; `scripts/check_fmi_export_fmpy.py` packages the current-platform point-mass FMU, executes it in FMPy 0.3.26, and compares the final sample to the checked-in native trace tolerance; `openbmp_point_mass_export_model` emits the same time/throttle/altitude/step/velocity value-reference map used by the restricted C ABI; `openbmp_fmi_binary_entry_for_target` covers supported Linux/macOS/Windows cdylib paths; `openbmp_point_mass_fmi_binary_entry_for_target` covers the modelIdentifier binary paths expected by standard importers; `write_openbmp_point_mass_fmu_archive_for_current_platform` writes the current-platform point-mass FMU and returns the binary entry used by the loader; and the archive round-trips through the in-repo `FmuArchive` parser into the expected typed binding plan. The current-platform point-mass package is also materialized and smoke-loaded through `Fmi3DynamicLibrary::open_archive_binary`. `openbmp-fmi` builds as a `cdylib` and exposes a restricted plant-backed FMI 3 C-symbol smoke surface over deterministic point-mass-like state, including typed access, fixed-step `fmi3DoStep`, FMU-state snapshot/restore, fail-closed unsupported FMI 3 symbols for full-symbol-table importers, and a multi-step native-trace tolerance gate. Full OpenBMP plant mapping remains open.
- **fidelity_tier:** T4
- **depends_on:** [WP-10.4]
- **new_crates:** none (extend `openbmp-fmi`)
- **touched:** `crates/openbmp-fmi/{Cargo.toml,src/export.rs,src/export_abi.rs,src/lib.rs,examples/export_point_mass_model_description.rs,examples/write_point_mass_fmu.rs}`, `scripts/check_fmi_export_schema.py`, `scripts/check_fmi_export_fmpy.py`, `.github/workflows/ci.yml`, build artifacts
- **approach:** §3.4 (exporter). Generate `modelDescription.xml` from the plant's value-ref map using FMI 3 root token, direct typed-variable elements, typed input starts, and `ModelStructure`; validate the generated point-mass metadata with local restricted and pinned official FMI 3.0 XSD gates; package the metadata and shared-library payload into a deterministic stored-entry FMU archive under the modelIdentifier binary path expected by standard importers; return the entry path so importer materialization uses the same archive member; keep the packaged point-mass metadata and ABI value references on shared constants; expose lifecycle + `fmi3SetFloat64`/`fmi3GetFloat64`/`fmi3DoStep` symbols backed by a fixed-step deterministic plant; export fail-closed unsupported FMI 3 symbols so FMPy can load the full symbol table; no wall-clock; compare multi-step ABI output and FMPy output to a native trace using `export-point-mass-trace.toml`. Remaining work is a full OpenBMP plant mapping.
- **acceptance:**
  - generated `modelDescription.xml` round-trips through `openbmp-models::FmuArchive` with typed value references
  - exported C symbols step a deterministic plant state and round-trip FMU-state snapshot/restore
  - exporter FMU runs inside `fmpy`; trace reproduces the native plant within tolerance
  - generated `modelDescription.xml` validates against the FMI 3.0 schema
  - round-trip (export → `fmpy` → compare) in a tolerance table
  - all §2 gates green
- **validation_label:** `research`
- **dual_use_note:** far from line
- **est_effort:** 2–3 weeks
- **parity_ceiling:** no conformance certification

---

**WP-10.6 — AFTS forward containment monitor (`openbmp-afts`)**
- **goal:** Independent forward IIP-geo-containment monitor: predict the instantaneous impact point and compare against declarative containment polygons/gates/corridors, emitting a latched terminate boolean + which-rule-fired, scored against truth. Forward-only; never a targeting input.
- **fidelity_tier:** A0
- **depends_on:** [WP-10.1] (T2 to inject faults against it)
- **new_crates:** `openbmp-afts` (L2) — first commit is the skeleton + placement justification (`no_std`-capable physics-layer monitor; deps only `openbmp-core`/`-state`/`-physics`; FC may host it without breaking the lock)
- **touched:** `crates/openbmp-afts/**(new)`, `crates/openbmp-runner/src/sil.rs`, `crates/openbmp-testkit/tests/afts_no_inverse_compile_fail.rs(new)`, `scenarios/**` (rule-table TOML + provenance)
- **approach:** §3.7. Vacuum closed-form conic IIP + numeric `r̈ = −GM_earth·r/‖r‖³` to `‖r‖ = R_earth`, rotating-Earth longitude correction (symbolic constants only). Declarative rule table: point-in-polygon (ray-cast/winding), gate crossing, corridor limits, green/red zones. `terminate = monotone-OR(violations)`, single latched. Reuse the existing footprint/integrator. **New compile-fail tripwire** proving no inverse `solve_burn_to_reach` surface exists.
- **acceptance:**
  - numerically propagated IIP matches the closed-form conic point `≤ 1e-6 rad` (MMS, tolerance table)
  - hand-computed point-in-polygon containment cases pass; terminate is latched (never un-latches)
  - the no-inverse compile-fail test is green; no aimpoint/range field anywhere in the surface
  - rule table is declarative TOML with `provenance.md` + SHA pin
  - no literal `GM_earth`/`J2`/`R_earth` numerals (symbolic only)
  - all §2 gates green
- **validation_label:** `validated-toy` → `research` (with the closed-form public benchmark)
- **dual_use_note:** **sharpest line** — monitor only, no inverse solve, containment = SAFETY keep-in/keep-out (the antithesis of an aimpoint); new compile-fail lock added (`00` §6)
- **est_effort:** 3–4 weeks
- **parity_ceiling:** not a certified/flight-heritage AFTS; no RCC 319 qualification or range-safety sign-off

---

**WP-10.7 — PIL on emulated flight ISA (Renode)**
- **goal:** Factor the FC step into the `no_std` core, cross-compile to `thumbv7em`/`riscv32`, run under Renode (deterministic shared virtual time) coupled over GDB-stub or virtual-UART using the same bridge frames; add cycle-approximate WCET. Byte-identical flight binary verified on target ISA, bit-true and faster-than-real-time.
- **fidelity_tier:** T5
- **depends_on:** [WP-10.1, WP-10.2]
- **new_crates:** none (factor inside `openbmp-fc`; a `target/` firmware crate sibling, host-driven via `openbmp-runner`)
- **touched:** `crates/openbmp-fc/src/lib.rs` (`fc_step()` entry), firmware bin, `crates/openbmp-runner/src/sil.rs` (Renode coupler), `tests/fixtures/renode/**` (`.resc` + provenance)
- **approach:** §3.5. Factor `fc_step(SensorPacket) -> ActuatorCommandPacket` reachable from a `no_std` binary (FC already `no_std`/`forbid(unsafe)`). Couple via GDB-stub or virtual-UART reusing `openbmp-bridge` frame/codec + `Transport`. Renode `.resc` sets quantum + `sync` for deterministic virtual time. Cycle-approx WCET via instruction-count × CPI.
- **acceptance:**
  - target binary produces the **same actuator stream** as the host SIL for the same `SensorPacket` stream (numerical-equivalence; bit-identical or `≤ 1 ULP` documented)
  - PIL run is bit-true across reruns (Renode deterministic time)
  - cycle-approx WCET reported; `frame_cycles ≤ budget` asserted
  - FC portability lock intact; `.resc` carries provenance
  - all §2 gates green
- **validation_label:** `validated-toy` → `research` (numerical-equivalence)
- **dual_use_note:** far from line — moves the same forward FC binary onto target ISA; no objective added
- **est_effort:** 4–6 weeks (after the `no_std` factor)
- **parity_ceiling:** cycle-approximate (not cycle-accurate); no real rad-hard part pipeline / SEU / TMR-voter timing

---

**WP-10.8 — cFS apps as flight software in the loop**
- **goal:** Couple NASA cFS (Apache-2.0) behind the HAL via a sim-hardware cFS app + a Rust↔cFE-Software-Bus bridge, driving cFS TIME from the sim clock for lockstep determinism, following the NOS3 reference architecture. Actual heritage flight-software stack flying OpenBMP scenarios.
- **fidelity_tier:** T6
- **depends_on:** [WP-10.1]
- **new_crates:** none in the Rust workspace for cFS itself (external submodule); a `cfs-bridge` adapter inside `openbmp-sil`/a sibling
- **touched:** external cFS submodule wiring, `crates/openbmp-sil/src/{cfs_bridge.rs(new)}`, a C sim-hardware cFS app (own build), CI coupling job
- **approach:** §3.6. cFS sim-hardware app subscribes to actuator MsgIds / publishes sensor MsgIds on the cFE SB; Rust↔SB bridge maps to `openbmp-bridge` frames over `Transport`; drive cFS TIME from the sim clock (one major frame per control epoch). Study NOS3 (couple, do not vendor C into the Rust crates).
- **acceptance:**
  - same cFS app set under NOS3's 42 dynamics and under OpenBMP's plant produces matching telemetry (code-to-code, tolerance table)
  - cFS TIME advances exactly one major frame per OpenBMP control epoch (lockstep determinism)
  - cFS kept external (submodule); licence + provenance recorded; no C vendored into Rust crates
  - all §2 gates green
- **validation_label:** `research`
- **dual_use_note:** far from line — ecosystem/test-abstraction plumbing; no-conformance-claim honesty retained
- **est_effort:** 6–10 weeks (deepest open rung)
- **parity_ceiling:** no proprietary flight-binary / vendor-bench validation; not certified flight software

---

## 10. References

1. Gomes, C., Thule, C., Broman, D., Larsen, P. G., Vangheluwe, H. "Co-Simulation: A Survey." *ACM Computing Surveys* 51(3), 2018. <https://doi.org/10.1145/3179993>
2. Kübler, R., Schiehlen, W. "Two methods of simulator coupling." *Math. and Computer Modelling of Dynamical Systems* 6(2), 2000.
3. Modelica Association. "Functional Mock-up Interface 3.0 Specification." §4.2 Co-Simulation API, §2.3 Clocks, intermediate update / early return. <https://fmi-standard.org/docs/3.0/>
4. Modelica Association. "FMI 3.0 Implementers' Guide." <https://modelica.github.io/fmi-guides/main/fmi-guide/>
5. Cremona, F., et al. "The FMI 3.0 Standard Interface for Clocked and Scheduled Simulations." *Electronics* 11(21):3635, 2022. <https://doi.org/10.3390/electronics11213635>
6. Busch, M., Schweizer, B. Predictor/corrector co-simulation with algebraic constraints (IMSD).
7. ASAM. "AE XIL Standard" and "AE XIL Generic Simulator Interface, Part 1 Programmers Guide V2.1.0" (MAPort/EESPort/framework, lifecycle states). <https://www.asam.net/standards/detail/xil/>
8. Typhoon HIL. "ASAM XIL API Python Guide" (worked MAPort/capture/signal-gen example). <https://www.typhoon-hil.com/documentation/typhoon-hil-software-manual/concepts/asam_xil_api_python_guide.html>
9. Antmicro. "Renode" (deterministic shared virtual time). <https://github.com/renode/renode>; "Developing and testing space systems with Renode," 2024.
10. QEMU. "GDB stub / semihosting." <https://www.qemu.org/docs/master/system/gdb.html>
11. MathWorks. "Processor-in-the-Loop Verification" (numerical-equivalence, ARM Cortex-A QEMU). <https://www.mathworks.com/help/ecoder/armcortexa/ug/processor-in-the-loop-verification-of-simulink-models.html>
12. Cao, Q., et al. "QEMU-CAS: A Full-System Cycle-Accurate Simulation Framework based on QEMU." CARRV 2023. <https://carrv.github.io/2023/papers/CARRV2023_paper_5_Cao.pdf>
13. OPAL-RT. "A Guide to Hardware-in-the-Loop (HIL) Testing," 2026. <https://www.opal-rt.com/blog/a-guide-to-hardware-in-the-loop-testing/>
14. Concurrent Real-Time. "Why Deterministic HIL Is Non-Negotiable" (jitter / CPU-shielding / RT-kernel). <https://concurrent-rt.com/hil/>
15. Liu, C. L., Layland, J. W. "Scheduling Algorithms for Multiprogramming in a Hard-Real-Time Environment." *JACM* 20(1), 1973 (rate-monotonic bound).
16. Linux PREEMPT_RT wiki; `SCHED_DEADLINE` (CBS/EDF) kernel docs; `cyclictest`.
17. Hsueh, M.-C., Tsai, T. K., Iyer, R. K. "Fault Injection Techniques and Tools." *IEEE Computer* 30(4), 1997. <https://doi.org/10.1109/2.585157>
18. Arlat, J., et al. "Fault Injection for Dependability Validation." *IEEE TSE* 16(2), 1990.
19. SAE 2023-24-0181. "HIL based Real-Time Co-Simulation for BEV Fault Injection Testing."
20. Bull, J. B., Lanzi, R. J. "An Autonomous Flight Safety System." NASA NTRS 20080044860. <https://ntrs.nasa.gov/api/citations/20080044860/downloads/20080044860.pdf>
21. NASA. "Autonomous Flight Safety System Phase III." NTRS 20090022215.
22. NASA. "NASA Releases Autonomous Flight Termination Unit (NAFTU) Software to Industry" (configurable rule-based, GPS+IMU). <https://www.nasa.gov/centers-and-facilities/wallops/nasa-releases-autonomous-flight-termination-unit-software-to-industry/>
23. Range Commanders Council. RCC 319 Flight Termination Systems Commonality Standard (IIP/containment concepts).
24. NASA. "cFS — Core Flight System" (cFE, OSAL, PSP, SCH, SB, LC, HS, SC), Apache-2.0. <https://github.com/nasa/cFS>
25. NASA. "NOS3 (NASA Operational Simulator for Small Satellites)" — cFS + 42 + COSMOS, GSC-17737-1. <https://software.nasa.gov/software/GSC-17737-1>; NTRS 20150020491; arXiv:1901.07583 (STF-1 case study).
26. McComas, D. "NASA/GSFC Flight Software Core Flight System." NTRS 20130013412.
27. Stoneking, E. "42 — NASA GSFC spacecraft attitude/orbit dynamics simulator," GSC-16720-1. <https://github.com/ericstoneking/42>
28. NASA. "Trick Simulation Environment." <https://github.com/nasa/trick>
29. Modelica Association. "Reference FMUs" (BSD-2). <https://github.com/modelica/Reference-FMUs>; `fmpy` (BSD-2). <https://github.com/CATIA-Systems/FMPy>
30. NASA-STD-7009B (March 2024). *Standard for Models and Simulations*. <https://standards.nasa.gov/standard/NASA/NASA-STD-7009>

*Companion documents:* `00-overview.md` (constitution) · `13-agent-execution-playbook.md` (process). *Tightest cross-references:* `12-determinism-realtime-and-compute.md` (RT/determinism), `09-sensors-navigation-and-actuators.md` (packet semantics), `06-gnc-coupled-mimo-and-control.md` (FC under test), `08-environment-gravity-and-frames.md` (IIP propagator), `11-monte-carlo-uq-and-validation.md` (fault campaigns), `07-trajectory-optimization-and-mission-design.md` (shared forward-only escalation). *Upstream anchors:* `docs/real-rocket-integration.md`, `docs/HAL.md`, `docs/standards-posture.md`, `docs/safety-boundaries.md`, `docs/verification.md`.
