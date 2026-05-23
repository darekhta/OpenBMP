# Phase 5.X Plan — Mission Graph Architecture Refactor

Phase 5.X is the **architecture-refactor phase between Phase 5 and Phase
6**. It addresses the mission-FSM design weaknesses surfaced during the
Phase 5.B audit and locks the mission-graph contract before Phase 6
hypersonic work expands the regime axis count further. Phase 5.X is not
an algorithm phase — every sub-phase is a refactor, vocabulary
migration, or contract clarification, with no new GNC algorithm landing
in this phase.

Phase 5.X starts from the closed Phase 5 work (autopilot SOTA suite,
SR-UKF, IMM, multi-lane voter, windowed GLRT, NRLMSISE-00, EGM2008,
multi-rate scheduling, multi-body propagation, HIL bridge, ULog /
dataflash cross-validation, Bar-Shalom reproducibility, long-duration
soak) and lands the architectural prerequisites for Phase 6:

1. The mission FSM becomes hierarchical (Harel statechart semantics)
   with orthogonal concurrent regions.
2. The simulator stops owning a parallel mission graph; the FC
   commander becomes the single source of truth.
3. The action taxonomy is split into `MissionAction`
   (HAL-portable) and `ScenarioScriptAction` (simulator-only physics
   overrides) in separate crates.
4. The state-name vocabulary is migrated to the academic canon defined
   in [`mission-states-vocabulary.md`](mission-states-vocabulary.md);
   every operational / engagement-derived name is rejected with
   compatibility shims for one phase.
5. The scenario format bumps to v4 with declarative mission hierarchy
   and orthogonal-region declarations.

This plan replaces no existing plan. Phase 5.X documents are
authoritative in
[`mission-graph-architecture.md`](mission-graph-architecture.md) and
[`mission-states-vocabulary.md`](mission-states-vocabulary.md);
this plan owns the migration sequence, exit criteria, and validation
evidence.

## Why this is the right time

Phase 5 closes the algorithm scope. Phase 6 (hypersonic) needs the
mission-FSM machinery to track regimes that today's flat DAG cannot
express:

- Re-entry sub-state hierarchy
  (`PreEntry → EntryInterface → LiftingEntry → PeakHeating →
  PeakDeceleration → DescentPhase`).
- Orthogonal health regime tracking
  (`Nominal / Degraded / AbortRequested / SafedOnFault`) running
  concurrently with mission phase, because re-entry abort logic must
  be inheritable from a parent state.
- Estimator regime (set by the IMM Phase 5.B.3 / 5.B.6) coupled to
  autopilot gain selection — the regime axis is observable today but
  consumed by no autopilot path; Phase 5.X exposes the wiring.
- Per-region stop / safe-state propagation that today the commander
  hand-rolls in `safe_state_requested` boolean state.

Doing this work *during* Phase 6 would compound architecture refactor
with hypersonic physics — exactly the scope creep that Phase 5's
naming-honesty discipline forbids. Phase 5.X is the right place.

## Scope and anti-scope

**In scope (Phase 5.X):**

- Hierarchical state machine primitives (`State`, `Region`,
  `Transition` with parent inheritance, entry / exit / do actions, history
  pseudo-states).
- Orthogonal concurrent regions (mission × health × comms × flight-mode
  × estimator-regime) with deterministic per-region tick order.
- Single mission-state ownership: FC commander is authoritative; the
  simulator subscribes via `commander.vehicle_status`.
- Crate split: `openbmp-mission` (HAL-portable) vs new
  `openbmp-scenario-script` (sim-only physics overrides).
- `MissionAction` / `ScenarioScriptAction` enum split.
- Academic-vocabulary migration with one-phase backward-compat shims.
- Scenario format v4 with hierarchical mission blocks, orthogonal
  region declarations, and `serde(deny_unknown_fields)` schema-version
  bump.
- Re-baseline of every Phase-5 tolerance table that depends on
  mission-graph evaluation order.
- Cross-validation against the existing Phase-5 closed-loop scenarios:
  every shipped scenario must produce byte-identical telemetry against
  its Phase-5 baseline after the refactor (see § Determinism
  preservation rules).

**Out of scope (deferred to Phase 6 or rejected outright):**

- New mission-graph trigger types beyond what Phase 5 ships
  (`AtTime`, `AtAltitudeAscending/Descending`, `AtApogee`,
  `AtMassFraction`, `AtDynamicPressure`, plus Phase-5 closure
  triggers). New triggers are Phase 6 if the hypersonic atmosphere
  needs them; otherwise rejected.
- New mission-graph action types beyond the closed Phase-5 set. Action
  vocabulary closure happens *with* the crate split, not after.
- Re-entry sub-state graph definitions. The hierarchy *primitives*
  land in 5.X; the *re-entry sub-states* land in Phase 6.D where the
  hypersonic environment justifies them.
- Auto-coding tooling. F-Prime FPP-style code generation is
  attractive but is its own multi-month project; explicitly deferred
  to a post-1.0 v2 conversation.
- SCXML import / export. The W3C SCXML standard has overlapping
  semantics but introduces XML and a runtime conformance burden;
  rejected for OpenBMP's in-house Rust posture. (See
  [`mission-graph-architecture.md § Why not SCXML`](mission-graph-architecture.md#why-not-scxml).)
- Behavior trees as an alternative to HSM. Modern robotics literature
  favors BTs for some autonomy stacks; OpenBMP's regime model is
  state-shaped, not behavior-tree-shaped. Rejected.
- Any new operational / engagement vocabulary. Phase 5.X *removes*
  rejected terms; it does not *negotiate* their replacement.

## Naming-honesty discipline

Phase 5.X applies the established discipline to a new axis: **state
names**. Until a state has a documented academic justification in
[`mission-states-vocabulary.md`](mission-states-vocabulary.md), it
cannot ship. Renames are upstream-only — every rejected term is removed
in the same commit that introduces its academic replacement, with a
backward-compatibility shim deprecation-warned for one phase. No commit
may carry both an operational and an academic name simultaneously
beyond that one-phase window.

The full rejection list with academic replacements is in
[`mission-states-vocabulary.md § Rejected Vocabulary`](mission-states-vocabulary.md#rejected-vocabulary).

## Determinism preservation rules

Phase 5.X is a refactor, not an algorithm change. The expected
post-refactor behavior on every shipped Phase-5 scenario is
**byte-identical telemetry** against the Phase-5 baseline. This is
enforced as the primary acceptance gate of every Phase 5.X sub-phase:

1. The full Phase 5 scenario corpus runs through the determinism CI
   gate. Any byte difference is a regression unless the scenario file
   itself is migrated to v4 syntax (in which case the v4-migrated
   scenario must produce byte-identical output to the v3 original on
   the reference platform profile).
2. Tolerance-table TOMLs whose metrics depend on mission-graph
   iteration order are documented per scenario; the refactor's effect
   on iteration order is locked at the canonical-form level (no behavior
   change).
3. The `expected.toml` / Parquet snapshots for every comparison
   harness output (PID / L1 / LQR / INDI / MPC) remain byte-identical;
   the comparison harness itself does not call into the mission-graph
   subsystem on its hot path.

If any of these tightens to "state-stable, not bit-stable" because the
refactor surfaces FP-order issues that Phase 5 didn't notice, the
sub-phase is split in two: the FP-fix sub-phase first, then the
refactor.

## Sub-phase roadmap

```text
5.X.0 (pre-work, blocks everything)
  ├── 5.X.A (action taxonomy split)
  │   └── 5.X.A.1 (MissionAction enum)
  │   └── 5.X.A.2 (ScenarioScriptAction crate)
  │   └── 5.X.A.3 (EventBinding generic)
  │
  ├── 5.X.B (single source of truth)
  │   └── 5.X.B.1 (FC publishes phase on bus)
  │   └── 5.X.B.2 (simulator subscribes, removes parallel state)
  │   └── 5.X.B.3 (test-only phase override channel)
  │
  ├── 5.X.C (hierarchical primitives)
  │   └── 5.X.C.1 (parent / child relationships)
  │   └── 5.X.C.2 (entry / exit / do actions)
  │   └── 5.X.C.3 (history pseudo-states)
  │
  ├── 5.X.D (orthogonal regions)
  │   └── 5.X.D.1 (Region type + ownership rules)
  │   └── 5.X.D.2 (deterministic per-region tick order)
  │
  ├── 5.X.E (academic vocabulary migration)
  │   └── 5.X.E.1 (rename rejected terms upstream)
  │   └── 5.X.E.2 (backward-compat shims with deprecation warnings)
  │
  ├── 5.X.F (scenario format v4)
  │   └── 5.X.F.1 (hierarchical [[mission.states]] blocks)
  │   └── 5.X.F.2 ([[mission.regions]] orthogonal declarations)
  │   └── 5.X.F.3 (schema-version bump + serde deny_unknown_fields)
  │
  ├── 5.X.G (validation re-baseline)
  │   └── 5.X.G.1 (HSM property tests)
  │   └── 5.X.G.2 (orthogonal-region consistency tests)
  │   └── 5.X.G.3 (determinism CI re-baseline)
  │
  └── 5.X.H (documentation harmonisation)
      └── 5.X.H.1 (update software-architecture.md § Mission FSM)
      └── 5.X.H.2 (retire phase-5x-plan.md after sub-phases ship)
      └── 5.X.H.3 (update real-rocket-integration.md § Mission tree)
```

A → B → C → D → E run sequentially because each depends on the
previous taxonomy / ownership / primitives being in place. F is gated
on A through D so the v4 scenario format can encode every concept that
landed. G and H run alongside; G gates merge of every preceding
sub-phase; H is the final commit that retires this plan document.

## Sub-phase definitions

### 5.X.0 — Pre-work: documentation alignment

**Scope.** Land the three Phase 5.X authoritative documents
(`phase-5x-plan.md`, `mission-graph-architecture.md`,
`mission-states-vocabulary.md`) and update the document index in
`docs/README.md`. No code change. Mark the relevant sections of
`software-architecture.md`'s mission-FSM material as
*"superseded by Phase 5.X — see mission-graph-architecture.md"* with
backreference links.

**Exit criterion.** All three docs land. `docs/README.md` indexes
them. Existing docs that reference the flat-DAG mission graph carry
the supersession note.

**Validation evidence.** Doc-link integrity check (cargo task `xtask
docs-check` or equivalent); spell-check; markdown-render preview.

### 5.X.A — Action taxonomy split

**Scope.** Split `EventAction` into two enums in two crates:

- **`openbmp-mission::MissionAction`** (HAL-portable):
  - `EnterState(StateId)` — replaces `EnterPhase(PhaseId)`. The
    rename reflects the hierarchical-state semantics; `PhaseId` is
    aliased to `StateId` for one phase as a backward-compat shim.
  - `EmitTelemetryMarker { tag: String }` — unchanged.
  - `RaiseHealthAlarm { region: RegionId, alarm: AlarmCode }` — new;
    fires from a binding to demote the orthogonal `Health` region.
  - `RequestSafeState { reason: String }` — new; replaces the
    commander's hand-rolled `safe_state_requested` boolean.
  - `Stop { label: String }` — unchanged.

- **New `openbmp-scenario-script` crate, `ScenarioScriptAction`**
  (simulator-only):
  - `EngineCommand { id, throttle_unit, gimbal_pitch_rad,
    gimbal_yaw_rad, ignite, shutdown }` — unchanged from current.
  - `EffectorOverride { id, command }` — unchanged.
  - `Separation { … }` — unchanged.
  - `DeployRecovery { id, command }` — unchanged.

**Required `EventBinding` generic.** `EventBinding<A>` parameterised
over the action type; `EventBinding<MissionAction>` is the FC's, and
`EventBinding<ScenarioScriptAction>` is the simulator's. The
simulator kernel holds two binding lists; the FC commander holds one.

**Exit criterion.** Two crates compile; the FC commander only depends
on `openbmp-mission`; the simulator kernel depends on both. Existing
scenarios continue to load (the parser routes scripted-physics
actions to the script crate's binding list). Determinism CI passes.

**Validation evidence.** Unit tests per enum; cross-crate compile
test; round-trip scenario parser tests for every shipped scenario;
determinism CI passes byte-identical against the Phase-5 baseline.

**Scope guardrail.** No new actions. No new triggers. The split is
mechanical refactor; novel functionality lives in later sub-phases.

### 5.X.B — Single source of truth for mission state

**Scope.** Make the FC commander the sole owner of mission state and
have the simulator subscribe.

- The FC commander publishes `commander.mission_state` on every tick:
  carries the active state path (e.g.
  `mission.states.in_flight.boost.first_stage_burn`), region states
  (Health / Comms / etc.), and a transition log delta.
- The simulator kernel removes its `mission_graph` field and its
  `current_phase` field, and instead subscribes to
  `commander.mission_state`. The kernel's
  `Vec<EventBinding<ScenarioScriptAction>>` (the scripted-physics
  bindings, separate from the mission FSM) continues to evaluate
  triggers each tick — those bindings are *not* state transitions,
  they are scenario-script overrides that fire alongside mission
  state.
- A new test-only override topic `commander.scenario_state_override`
  lets a scenario script the commander into a specific state for
  validation purposes; the commander treats this as a directive and
  fires the corresponding `EnterState` action. This is explicitly
  test-only and carries a scenario flag
  `mission.test_only_state_override = true` that defaults to false
  and refuses to load in production HAL deployments (HAL crate sets a
  build-time gate).

**Exit criterion.** The simulator kernel has no `current_phase` field.
Every consumer of "current mission state" reads
`commander.mission_state`. The HAL crate's build-time gate refuses
the test-only override topic. Determinism CI passes byte-identical.

**Validation evidence.** Unit tests for the bus subscription pattern;
property test that no module other than the commander writes to
`commander.mission_state`; integration test that scenario-script
phase override fires the commander's `EnterState` action with a
documented one-tick latency; determinism CI passes byte-identical.

**Scope guardrail.** The simulator may *observe* mission state but
must not *decide* mission state. This is the load-bearing rule for
HAL portability — anything that decides mission state must compile
and run in a HAL deployment without a simulator present.

### 5.X.C — Hierarchical state machine primitives

**Scope.** Replace the flat `MissionPhaseGraph` with a hierarchical
`MissionStateMachine`:

- **Parent / child relationships.** Each `State` has an optional
  `parent: Option<StateId>`. Transitions declared on a parent state
  fire from any descendant state unless overridden in the descendant.
- **Entry / exit / do actions.** Each `State` carries
  `on_entry: Vec<MissionAction>`, `on_exit: Vec<MissionAction>`,
  `on_active: Vec<MissionAction>` (the latter fires every tick the
  state is active; useful for periodic health checks).
- **History pseudo-states.** A `HistoryState` records the last-active
  child of a composite state; transitioning *to* the composite via its
  history pseudo-state resumes the saved sub-state.
- **Shortest-path traversal on transition.** When transitioning from
  state `A` (deep in the hierarchy) to state `B` (deep in another
  branch), the FSM computes the lowest common ancestor, exits states
  going up to it (firing `on_exit` actions), then enters states going
  down to `B` (firing `on_entry` actions). This is the standard Harel
  semantics and matches `statig`'s implementation.
- **Determinism contract.** All ids remain FNV-1a-64 of canonical
  scenario paths. Canonical-form sort extends to: states by
  `(depth-from-root, parent-StateId.value(), StateId.value())`;
  transitions by their existing five-tuple plus the ancestor-LCA
  depth.

**Exit criterion.** The new `MissionStateMachine` type compiles in
`openbmp-mission`. A migration test re-builds every shipped Phase-5
scenario as a degenerate hierarchy (every state is a direct child of
the root) and produces byte-identical telemetry against the Phase-5
baseline.

**Validation evidence.** Unit tests for LCA computation, shortest-path
traversal, entry / exit ordering; property test that every reachable
state is observable from `initial`; property test that every
transition's `on_entry` / `on_exit` fires at most once per transition;
determinism CI passes byte-identical for every shipped scenario.

**Scope guardrail.** The hierarchy primitives land *empty of new
states*. Every shipped scenario migrates as a flat hierarchy. The
academic state vocabulary (with hierarchy depth > 1) lands in
sub-phases E and F together; not here.

### 5.X.D — Orthogonal regions

**Scope.** Add concurrent regions:

- **`Region` type.** A region owns one independent state machine
  instance. Each region has its own current state, transition log,
  and event evaluation. Regions tick in deterministic order
  (declaration order, with `RegionId.value()` as tie-break).
- **Required regions.** Phase 5.X ships four canonical regions:
  - `mission` (the existing mission FSM, with hierarchy from 5.X.C).
  - `health` (Nominal / Degraded / AbortRequested / SafedOnFault).
    See [`mission-states-vocabulary.md § Health Region`](mission-states-vocabulary.md#health-region).
  - `comms` (Linked / Degraded / LossOfSignal / SafedOnLossOfSignal).
    See [`mission-states-vocabulary.md § Comms Region`](mission-states-vocabulary.md#comms-region).
  - `estimator_regime` (set by the IMM in Phase 5.B; published by the
    estimator subsystem and observed-but-not-decided by the
    commander). See
    [`mission-states-vocabulary.md § Estimator-Regime Region`](mission-states-vocabulary.md#estimator-regime-region).
- **Cross-region transition guards.** A transition in `mission` may
  reference an `expected_health: Nominal` precondition; the FSM only
  applies the transition when the named region's current state
  matches. Guards compose with triggers via `AND` semantics
  (trigger AND guard).
- **Region-published topics.** Each region publishes its state on a
  dedicated bus topic
  (`commander.region.mission`, `commander.region.health`, etc.), so
  consumers can subscribe to a single region without parsing the
  full `commander.mission_state`.

**Exit criterion.** All four canonical regions land with empty (or
single-state) machines, except `mission` which holds the migrated
shipping graph. The bus carries one topic per region. The autopilot
and FDIR continue to consume the migration-equivalent topics with no
behavior change.

**Validation evidence.** Unit tests per region; property test that
inter-region guards never produce a non-deterministic transition (a
guard either fires consistently or doesn't fire); determinism CI
passes byte-identical.

**Scope guardrail.** Regions are observable everywhere; only the
commander writes them. The regions' state machines are *not* yet
hierarchical (Phase 5.X only requires the *mission* region to be
hierarchical). Health-state hierarchy is Phase 6 if abort logic
demands it.

### 5.X.E — Academic vocabulary migration

**Scope.** Apply the academic vocabulary canon defined in
[`mission-states-vocabulary.md`](mission-states-vocabulary.md). For
every name in the rejection list:

1. Add the academic replacement to `openbmp-mission` (or wherever the
   type lives) with full doc-comment.
2. Mark the rejected name `#[deprecated(since = "5.X.E", note = "use
   <replacement>")]` and re-export under the new name.
3. Update every internal call site to use the academic name.
4. Update every shipped scenario file's TOML to use the academic
   path-derived ids.
5. Update every test, comment, and doc reference to the academic
   name.
6. Land a CI lint that rejects new occurrences of the deprecated
   names in any future PR.

The rejected list (full canon in
[`mission-states-vocabulary.md`](mission-states-vocabulary.md#rejected-vocabulary)):

| Rejected | Replacement | Reason |
|---|---|---|
| `TerminalDescent`, `Terminal` (state) | `FinalDescent` | Operational ballistic-missile / interceptor terminology |
| `Endgame` | `PostFlight` | Operational engagement terminology |
| `Engagement` | (no replacement; rejected) | Operational |
| `Strike` | (no replacement; rejected) | Operational |
| `Target` (as guidance reference) | `Waypoint` / `ReferenceTrajectory` | Operational |
| `Seeker` | (no replacement; rejected) | Operational |
| `Interceptor` | (no replacement; rejected) | Operational |
| `Threat` | (no replacement; rejected) | Operational |
| `Kill` | (no replacement; rejected) | Operational |
| `Glide` (without "Academic" qualifier) | `AcademicGlide` / `AcademicSkipGlide` | Already in safety-boundaries.md |
| `Maneuvering` (entry context) | `LiftingEntry` | Already in safety-boundaries.md |
| `RTH` / `ReturnToHome` | `ReturnToReferencePoint` | Operational base implication |
| `BTT` / `STT` (terminal-mode autopilot) | (rejected for terminal-mode use) | Already in safety-boundaries.md |
| `Midcourse` (state name) | `Coast` + `ApogeeApproach` | Operational ballistic-missile vocabulary |
| `Boost-Midcourse-Terminal` (taxonomy) | `Pad-Ascent-Coast-Apogee-Descent-Recovery` | Operational ballistic-missile taxonomy |

**Exit criterion.** Every shipped scenario, every code path, every
doc reference uses academic names. The deprecation shims compile but
emit warnings. CI lint rejects new occurrences.

**Validation evidence.** `cargo clippy --workspace -- -D
deprecated` succeeds (no in-tree use of rejected names); scenario
fuzz target updated; round-trip golden parses every shipped scenario;
documentation lint checks for rejected names.

**Scope guardrail.** This is rename-only. No state-machine
*structure* changes. No new states, no new transitions, no new
regions.

### 5.X.F — Scenario format v4

**Scope.** Bump scenario format to v4. New blocks:

- `[[mission.states]]` — hierarchical, with `parent: Option<String>`
  field (path of the parent state, or omitted for top-level).
- `[[mission.states.on_entry]]` / `on_exit` / `on_active` — action
  arrays per state.
- `[[mission.regions]]` — declares the regions the scenario uses; if
  omitted, the four canonical regions are auto-declared.
- `[[mission.transitions]]` — extended with optional
  `guard: { region: "health", state: "nominal" }` and optional
  `priority: u32` (for ambiguous-transition tie-break, default 0).
- `[mission.scope]` — declares whether this is a sounding-rocket
  / propulsive-landing / re-entry / orbital-insertion scenario, used
  only for human-readable telemetry tags.

`serde(deny_unknown_fields)` on every new block; v3 scenarios continue
to parse byte-identically through a v3-to-v4 lifting pass.

**Exit criterion.** Parser unit + property tests; scenario-fuzz
target updated; one round-trip golden parses every shipped scenario
under both v3 and v4 syntax with byte-identical telemetry; the v4
schema documentation lands in `docs/scenario-format.md`.

**Validation evidence.** v3 / v4 parse-equivalence tests; fuzz; doc
update.

**Scope guardrail.** The v4 schema is finalized as part of this
sub-phase. No v4.1 or further extension lands in Phase 5.X — the v4
schema is the 1.0 contract for scenarios.

### 5.X.G — Validation re-baseline

**Scope.** Establish the property-test corpus that defines correct
HSM behaviour:

- Every reachable state is observable from `initial` (already
  enforced; extended to hierarchical case via depth-first traversal
  through parent links).
- LCA computation is correct (unit tests against known ancestor
  graphs; property test against random graphs).
- Entry / exit ordering on transitions is monotonic (entry actions
  fire in depth order top-to-bottom; exit actions fire bottom-to-top).
- History pseudo-states record the last child correctly across
  multiple entries.
- Orthogonal regions tick deterministically (per-region order locked
  by canonical-form sort).
- Cross-region guards are deterministic (a guard either always fires
  or never fires for a given transition + region-state pair; never
  flaps).
- Determinism CI passes byte-identical for every shipped scenario
  against the Phase-5 baseline.

**Exit criterion.** The property-test corpus runs in CI; failure
causes merge block. The determinism CI gate passes byte-identical.

**Validation evidence.** Per-property test, all green. CI artifact:
the byte-identical Parquet output across two reruns of every shipped
scenario.

**Scope guardrail.** Property tests cover the FSM's algebraic
invariants; they do *not* validate scenario authoring (e.g. "is the
sounding-rocket scenario realistic"). That's not the FSM's job.

### 5.X.H — Documentation harmonisation

**Scope.** Final cleanup:

- Update `docs/software-architecture.md § Mission State Machine` and
  § Event / Phase Timeline to point at this plan and the architecture
  reference.
- Update `docs/scenario-format.md § Mission block` to v4 syntax.
- Update `docs/safety-boundaries.md § Naming Rules` with cross-link
  to `mission-states-vocabulary.md`.
- Update `docs/real-rocket-integration.md § The event / phase model`
  to reflect hierarchical states + orthogonal regions.
- Retire `phase-5x-plan.md` (this document) per the existing pattern
  (mirroring `Retire Phase 5 planning + audit docs`).
- Mark the Phase 5.X period closed in `design-concept.md § Phase
  Roadmap`.

**Exit criterion.** All cross-references updated. `phase-5x-plan.md`
retired. CI documentation-link check passes.

**Validation evidence.** Doc-link check; spell-check; render preview.

## Success criteria

Phase 5.X closes when:

1. Every sub-phase A through H has merged with its declared exit
   criterion met and its tests in CI.
2. The determinism CI gate runs at minimum the analytic-toy,
   Niskanen, Calisto, every diff-flatness-figure-eight variant,
   parachute-recovery, sounding-rocket, multi-body, multi-engine, and
   the long-duration soak twice on
   `x86_64-unknown-linux-gnu` and asserts byte-identical Parquet
   against the Phase-5 baseline.
3. Every shipped scenario has been migrated to v4 syntax and
   re-asserted byte-identical.
4. No code path in any crate uses a rejected operational vocabulary
   name. CI lint rejects new occurrences.
5. The simulator kernel has no `mission_graph` field. The FC
   commander is the sole owner of mission state.
6. The `openbmp-mission` crate is HAL-portable: a downstream HAL
   adopter can compile against `openbmp-mission` alone (without the
   `openbmp-scenario-script` crate) and run a working FC commander.
7. The non-suitability disclaimer remains on every release artifact;
   `safety-boundaries.md` is updated with the academic state
   vocabulary canon.

## Risks and contingencies

- **Determinism regression.** The hierarchical traversal changes the
  iteration order of `on_entry` / `on_exit` actions in transitions
  that cross multiple states. The shipped Phase-5 corpus uses flat
  hierarchies, so the LCA is always the root and the new traversal
  reduces to the old single-step transition — *no behavior change
  expected* for migrated v3 scenarios. If determinism CI surfaces a
  regression, gate the work behind a `state-stable` profile flag,
  document the diff, then split the sub-phase to fix the FP-order
  issue before merging the refactor.
- **Cross-region guard explosion.** Composable guards across regions
  could interact badly under fault scenarios (a guard in mission
  depends on health = nominal; a fault demotes health to degraded;
  the mission transition silently doesn't fire). Mitigate by:
  (a) every transition with a cross-region guard must declare a
  fallback transition (`on_guard_failure: …`) or be flagged for
  guarded transitions only at scenario load,
  (b) the commander emits a `commander.guard_blocked` telemetry
  event when a transition is suppressed by a guard so the operator
  can see it.
- **Vocabulary churn.** The 5.X.E rename touches every file in the
  workspace. If the rename PR is too large for clean review, split
  it by crate: `openbmp-mission` first, then `openbmp-scenario`,
  then `openbmp-fc`, then scenario files, then docs. Each as its
  own commit.
- **Test-only override topic abuse.** Scenario authors might rely on
  `commander.scenario_state_override` for non-test purposes. Mitigate
  with the build-time HAL gate (Phase 5.X.B.3); the topic compiles
  out entirely in HAL deployments.

## Documentation deliverables

Each sub-phase ships:

- A per-sub-phase commit (or commits) following the existing
  Phase-4 / Phase-5 commit-message style.
- Updates to the relevant per-crate `README.md` (purpose, inputs,
  units, frames, assumptions, validity range, determinism,
  validation, data provenance, safety boundary).
- For 5.X.F (scenario format v4), a documented migration guide for
  scenario authors in `docs/scenario-format.md § Migrating v3 → v4`.
- For 5.X.E (vocabulary migration), a deprecation notice in the
  release CHANGELOG.

## Closing condition

Phase 5.X closes with:

- All sub-phases above merged.
- The `Phase 5 — Architecture closure` heading in
  `docs/design-concept.md` rewritten to reflect the closed Phase 5.X
  status, with a forward link to Phase 6 hypersonic work.
- This `phase-5x-plan.md` retired (committed as
  *"Retire Phase 5.X planning + audit docs"*), mirroring the Phase
  3.x / Phase 4 / Phase 5 retire pattern.
- `mission-graph-architecture.md` and
  `mission-states-vocabulary.md` survive as the authoritative
  references that Phase 6 builds against.
