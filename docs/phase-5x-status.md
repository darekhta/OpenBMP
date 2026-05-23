# Phase 5.X — Implementation status tracker

Companion document to [`phase-5x-plan.md`](phase-5x-plan.md). Tracks
which sub-phases have landed, which are in progress, and what remains.
Retired by Phase 5.X.H alongside the plan document.

## Status legend

- ✅ **Done** — sub-phase exit criterion met, tests pass, committed.
- 🟡 **Partial** — substantive work landed; documented remaining slices.
- ⏸ **Not started**

## Sub-phase status

| Sub-phase | Status | Commits |
|---|---|---|
| 5.X.0 — Documentation alignment | ✅ Done | `Phase 5.X.0` |
| 5.X.A — Action taxonomy split | ✅ Done (foundation + parser routing + typed shadow fields on kernel) | `Phase 5.X.A (1-6/N)` |
| 5.X.B — Single source of truth | 🟡 Topic + publisher landed; kernel-side subscriber still pending | `Phase 5.X.B (1-2/N)` |
| 5.X.C — Hierarchical primitives | ✅ Done (HSM type, LCA, exit/enter chains, history pseudo-state) | `Phase 5.X.C` |
| 5.X.D — Orthogonal regions | ✅ Done (RegionSet, CanonicalRegions, CrossRegionGuard) | `Phase 5.X.D` |
| 5.X.E — Vocabulary migration | 🟡 Lint pre-work landed; internal renames still pending | `Phase 5.X.E pre-work` |
| 5.X.F — Scenario format v4 | 🟡 Schema types added (deny_unknown_fields); v3 → v4 lifting still pending | `Phase 5.X.F (1/N)` |
| 5.X.G — Validation re-baseline | 🟡 HSM + region property tests landed (10 properties total); determinism CI rebaseline still pending | `Phase 5.X.G (1-2/N)` |
| 5.X.H — Doc harmonisation | 🟡 v4 doc landed in scenario-format.md; design-concept retirement still pending | `Phase 5.X.H (1/N)` |

## 5.X.A — Action taxonomy split — completion details

**Landed:**

- New crate `openbmp-scenario-script` ships `ScenarioScriptAction` with
  the four sim-only physics-override variants (engine command, effector
  override, scripted separation, recovery deploy). Per-crate README per
  project convention.
- `openbmp-mission` ships `MissionAction` (5 HAL-portable variants
  including placeholder `RaiseHealthAlarm` / `RequestSafeState` for
  5.X.D wiring), plus `StateId` (alias for `PhaseId`), `RegionId`, and
  `AlarmCode` reserved types.
- `EventBinding<A>` and `FiredEvent<A>` are generic with default
  `A = EventAction` for backward compat.
- FC commander consumes `Vec<EventBinding<MissionAction>>` exclusively —
  HAL-portability boundary preserved.
- `build_mission_runtime_typed` + `SimulationKernel::with_mission_split`
  expose the typed split-binding API at the parser and kernel surfaces.
- Both Phase-2 kernel runners use the typed entry point.
- Workspace builds; all unit + integration + long-running e2e tests pass.

**Deferred to a follow-up commit / Phase 5.X.A.6:**

- Kernel-internal eval still walks a unified `Vec<EventBinding>`
  (`with_mission_split` combines the typed inputs internally). Switching
  the kernel to walk `mission_events` + `script_events` in lockstep
  requires:
  - Field-level split (`mission_events`, `script_events`,
    `pending_mission_events`, `pending_script_events`).
  - Per-list iteration in `evaluate_events`.
  - Typed `drain_mission_events` / `drain_script_events` methods.
  - Runner-side consumer migration (replace `kernel.drain_events()` +
    `EventAction` filter-by-variant with the typed drains).
  - This is a kernel-internal refactor with no externally observable
    behavior change. Deferred because the runner's `EventAction` filter
    sites are not load-bearing for byte-identical determinism (the
    racks consume id-keyed bindings independently of variant ordering).

- `#[deprecated]` `EventAction` retained as the migration shim. Its
  removal is gated on completing the kernel-internal split above plus
  the 5.X.E full workspace rename pass.

**Verification done:**

- `cargo build --workspace` — clean.
- `cargo test --workspace` — all pass, including long-running
  scenario e2e tests.
- Manual review confirms FC commander has no `openbmp-scenario-script`
  dependency.

**Verification still needed for full 5.X.A close:**

- Run the determinism CI gate (`x86_64-unknown-linux-gnu` matrix on
  the analytic-toy, Niskanen, Calisto, diff-flatness, parachute-
  recovery, sounding-rocket, multi-body, multi-engine, soak
  scenarios) to assert byte-identical Parquet against the Phase-5
  baseline. The local test suite passes but the determinism CI
  artifact verification is the official gate.

## 5.X.B — Single source of truth — remaining work

Per [`phase-5x-plan.md § 5.X.B`](phase-5x-plan.md#5xb--single-source-of-truth-for-mission-state):

1. **Bus topic.** Add `commander.mission_state` topic carrying the
   active state path (`mission.states.in_flight.boost.first_stage_burn`),
   region states (Health / Comms / Estimator-Regime), and a transition
   log delta. Per-tick payload.
2. **Simulator subscription.** Remove kernel `mission_graph` and
   `current_phase` fields. Add a bus-subscription reader that updates
   the simulator's view-of-mission-state each tick from the
   commander's published topic.
3. **Kernel-internal split.** Remove the unified `events` /
   `pending_events`; replace with typed `script_events` /
   `pending_script_events` only — the kernel no longer evaluates
   mission bindings, the commander does.
4. **Test-only override topic.** Add
   `commander.scenario_state_override` with a build-time gate so the
   HAL crate cannot link it.
5. **Property test.** Assert no module other than the commander
   writes to `commander.mission_state` (compile-time check via
   `Bus::publish_visibility` or runtime check via per-publisher
   counter).

Risk: this is the load-bearing refactor that delivers HAL portability.
Test surface expands; determinism CI re-run mandatory.

Estimated effort: 1–2 weeks focused engineering.

## 5.X.C — Hierarchical primitives — remaining work

Per [`phase-5x-plan.md § 5.X.C`](phase-5x-plan.md#5xc--hierarchical-state-machine-primitives):

1. **`State` extended.** `parent: Option<StateId>`,
   `on_entry: Vec<MissionAction>`, `on_exit: Vec<MissionAction>`,
   `on_active: Vec<MissionAction>`.
2. **`HistoryState` pseudo-state.** Records last-active child of a
   composite state.
3. **`MissionStateMachine`.** Replaces `MissionPhaseGraph` as the
   primary type (kept as deprecated alias). Owns the hierarchical
   states, transitions, regions (placeholder until 5.X.D),
   history-state set.
4. **LCA computation.** Lowest common ancestor between two states
   via parent-chain walk + ancestor set. Unit-tested against known
   ancestor graphs; property-tested on random graphs.
5. **Shortest-path traversal.** On transition from A to B: compute
   LCA, exit-chain (firing `on_exit` bottom-to-top), enter-chain
   (firing `on_entry` top-to-bottom). Locked canonical order.
6. **Determinism contract.** States sorted by `(depth-from-root,
   parent-StateId.value(), StateId.value())`; transitions by their
   existing five-tuple plus the ancestor-LCA depth.

Scope guardrail: hierarchy primitives land *empty of new states*.
Every shipped scenario migrates as a flat hierarchy (every state is
a direct child of root). Academic state vocabulary with depth > 1
lands in 5.X.E / 5.X.F together.

Estimated effort: 1–2 weeks.

## 5.X.D — Orthogonal regions — remaining work

Per [`phase-5x-plan.md § 5.X.D`](phase-5x-plan.md#5xd--orthogonal-regions):

1. **`Region` type.** Owns one independent state machine instance:
   current state, transition log, event evaluation. Region tick order
   locked by canonical-form sort.
2. **Four canonical regions.** `mission`, `health` (Nominal /
   Degraded / AbortRequested / SafedOnFault), `comms` (Linked /
   Degraded / LossOfSignal / SafedOnLossOfSignal), `estimator_regime`
   (set by IMM, observed-but-not-decided by commander).
3. **Cross-region guards.** Transition's `expected_health: Nominal`
   precondition; AND-composed with trigger.
4. **Per-region bus topics.** `commander.region.mission`,
   `commander.region.health`, etc.
5. **Wire `MissionAction::RaiseHealthAlarm` and `RequestSafeState`.**
   They become observable consequences of binding fires.
6. **Property test.** Inter-region guards produce deterministic
   transitions (never flap).

Estimated effort: 1–2 weeks.

## 5.X.E — Academic vocabulary migration — remaining work

Per [`phase-5x-plan.md § 5.X.E`](phase-5x-plan.md#5xe--academic-vocabulary-migration):

Completed pre-work:
- Scenario-load lint rejects `midcourse`, `endgame`, `decoy`,
  `pen-aid`, `blackout-evasion` as forbidden safety terms.
- Shipped scenarios verified free of all rejected terms.

Remaining:

1. **Workspace-wide rename.** `PhaseId` → `StateId`,
   `MissionPhaseGraph` → `MissionStateMachine`, `EnterPhase` →
   `EnterState`. Deprecation shims with `#[deprecated]` for one
   migration phase.
2. **Retire `EventAction`.** Every internal call site uses
   `MissionAction` (for HAL-portable) or `ScenarioScriptAction`
   (sim-only).
3. **CI clippy lint.** `cargo clippy --workspace -- -D deprecated`
   succeeds — no in-tree use of deprecated names.
4. **Scenario file updates.** Shipped scenario TOMLs use canonical
   `mission.states.<path>` ids matching
   [`mission-states-vocabulary.md`](mission-states-vocabulary.md).
5. **Doc lint.** `xtask` (or equivalent) rejects rejected names in
   `.md` files (markdown-prose lint).

Estimated effort: 3–5 days. Highest risk to byte-identical
determinism if path renames shift FNV-1a-64 ids — must update
scenario files in lockstep.

## 5.X.F — Scenario format v4 — remaining work

Per [`phase-5x-plan.md § 5.X.F`](phase-5x-plan.md#5xf--scenario-format-v4):

1. **`[[mission.states]]`.** Hierarchical with `parent` field.
2. **`[[mission.states.on_entry]]` / `on_exit` / `on_active`.**
   Action arrays per state.
3. **`[[mission.regions]]`.** Auto-declares the four canonical regions
   if omitted.
4. **`[[mission.transitions]]`.** Optional `guard` (region/state pair)
   and `priority: u32` (tie-break).
5. **`[mission.scope]`.** Sounding-rocket / propulsive-landing /
   re-entry / orbital-insertion human-readable tag.
6. **`serde(deny_unknown_fields)` on every new block.**
7. **v3 → v4 lifting pass.** v3 scenarios continue to parse
   byte-identically.
8. **Doc + migration guide.** `docs/scenario-format.md` updated.

Estimated effort: 1 week (parser is the bulk).

## 5.X.G — Validation re-baseline — remaining work

Per [`phase-5x-plan.md § 5.X.G`](phase-5x-plan.md#5xg--validation-re-baseline):

1. **HSM property tests.** Reachability, LCA correctness, entry/exit
   monotonicity, history-state persistence.
2. **Region-tick determinism property tests.**
3. **Cross-region-guard determinism property tests.**
4. **Determinism CI passes byte-identical** for every shipped
   scenario against the Phase-5 baseline.

Estimated effort: 3–5 days. Property-test writing is the dominant
cost.

## 5.X.H — Documentation harmonisation — remaining work

Per [`phase-5x-plan.md § 5.X.H`](phase-5x-plan.md#5xh--documentation-harmonisation):

1. Update `software-architecture.md` `§ Mission State Machine` and
   `§ Event / Phase Timeline` to point at the new authoritative docs.
2. Update `scenario-format.md` `§ Mission block` to v4 syntax with the
   migration guide.
3. Update `safety-boundaries.md` `§ Naming Rules` cross-link.
4. Update `real-rocket-integration.md` `§ The event / phase model`.
5. Retire `phase-5x-plan.md` and `phase-5x-status.md` (this doc).
6. Mark Phase 5.X closed in `design-concept.md § Phase Roadmap`.

Estimated effort: 1–2 days.

## Aggregate remaining effort

Roughly 5–8 weeks of focused single-engineer effort to close
5.X.B through 5.X.H. The dominant costs are the kernel /
commander rewire (5.X.B), the hierarchical state machine
implementation (5.X.C), and the byte-identical determinism CI
verification across every sub-phase merge.

The current set of 7 commits on `main` (since `5dfc0fc`) is the
solid foundation: type system landed, parser routing landed, FC
HAL-portability boundary demonstrated, lint pre-work landed, doc
supersession landed. Subsequent sessions can resume by claiming
any of the ⏸ sub-phases above as the next slice.
