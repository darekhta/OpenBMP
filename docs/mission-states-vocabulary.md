# Mission States Vocabulary

This document is the **canonical state-name vocabulary** for OpenBMP's
mission state machine. Every state name, every region name, and every
rejected operational term is enumerated here. Together with
[`mission-graph-architecture.md`](mission-graph-architecture.md), this
document defines the names that go into the FSM machinery.

The discipline is the same as everywhere else in OpenBMP: rejected
terms cannot ship, academic equivalents are documented with citations
to the textbook tradition that justifies them, and naming-honesty is
enforced at scenario load.

This document supersedes the state-name material currently scattered
across [`safety-boundaries.md § Naming Rules`](safety-boundaries.md)
and [`design-concept.md § Vehicle Classes`](design-concept.md). Those
docs continue to enumerate broader naming rules
(crate names, type names, file names); this doc is specifically about
*mission-state and region names that appear inside an
`openbmp_mission::MissionStateMachine`*.

## The vocabulary discipline

Every name in this document falls into one of four categories:

| Category | Behavior at scenario load |
|---|---|
| **Canonical** | Accepted. Documented academic justification. |
| **Reserved (hypersonic extensions)** | Reserved for the hypersonic / re-entry extensions; ordinary scenarios cannot declare it but the parser knows the name. |
| **Rejected** | Refused with an error pointing at the academic replacement. |
| **Deprecated (one-release shim)** | Accepted with a deprecation warning; will become Rejected in the next major version. |

Scenario load enforces the canon. The state-name lint runs at parse
time and produces a typed error citing this document.

## The canonical mission hierarchy

The top-level `mission` region has the following hierarchy. Every
state is named with its full path; FNV-1a-64 of the path is the
`StateId`. Composite states are marked **(composite)**.

```text
mission                                                      (region root, composite)
│
├── mission.states.pad                                       (composite)
│   ├── mission.states.pad.standby                           (atomic)
│   ├── mission.states.pad.powered                           (atomic)
│   ├── mission.states.pad.armed                             (atomic)
│   ├── mission.states.pad.countdown_hold                    (atomic)
│   └── mission.states.pad.liftoff                           (atomic, transient)
│
├── mission.states.in_flight                                 (composite)
│   │
│   ├── mission.states.in_flight.ascent                      (composite)
│   │   ├── mission.states.in_flight.ascent.boost            (composite)
│   │   │   ├── mission.states.in_flight.ascent.boost.first_stage_burn   (atomic)
│   │   │   ├── mission.states.in_flight.ascent.boost.staging            (atomic, transient)
│   │   │   ├── mission.states.in_flight.ascent.boost.second_stage_burn  (atomic)
│   │   │   └── mission.states.in_flight.ascent.boost.upper_stage_burn   (atomic, optional)
│   │   └── mission.states.in_flight.ascent.post_boost_coast (atomic)
│   │
│   ├── mission.states.in_flight.apogee_regime               (composite)
│   │   ├── mission.states.in_flight.apogee_regime.apogee_approach (atomic)
│   │   ├── mission.states.in_flight.apogee_regime.apogee          (atomic, transient)
│   │   └── mission.states.in_flight.apogee_regime.apogee_passed   (atomic)
│   │
│   ├── mission.states.in_flight.coast                       (atomic)
│   │
│   └── mission.states.in_flight.descent                     (composite)
│       ├── mission.states.in_flight.descent.ballistic_descent (atomic)
│       │
│       ├── mission.states.in_flight.descent.entry_interface     (atomic, RESERVED)
│       ├── mission.states.in_flight.descent.lifting_entry       (atomic, RESERVED)
│       ├── mission.states.in_flight.descent.peak_heating        (atomic, RESERVED)
│       ├── mission.states.in_flight.descent.peak_deceleration   (atomic, RESERVED)
│       │
│       ├── mission.states.in_flight.descent.drogue_descent      (atomic)
│       ├── mission.states.in_flight.descent.main_descent        (atomic)
│       └── mission.states.in_flight.descent.final_descent       (atomic)
│
└── mission.states.post_flight                               (composite)
    ├── mission.states.post_flight.touchdown                 (atomic, transient)
    ├── mission.states.post_flight.recovery                  (atomic)
    ├── mission.states.post_flight.post_flight_safing        (atomic)
    └── mission.states.post_flight.safed                     (atomic, terminal)
```

### State definitions

#### Pad sub-hierarchy

| State | Meaning | Citations |
|---|---|---|
| `pad.standby` | Vehicle on launcher, no power applied. | BPS.space Signal phase vocabulary; Stevens & Lewis 2015. |
| `pad.powered` | Vehicle systems powered, not yet armed. | Idem. |
| `pad.armed` | Vehicle systems armed; ignition possible. | Idem. |
| `pad.countdown_hold` | Countdown paused at scenario-defined hold. | NASA launch-procedure terminology (academic). |
| `pad.liftoff` | Transient state covering ignition through pad clearance. Conventionally fires `EnterState(in_flight.ascent.boost.first_stage_burn)` on weight-off-launcher trigger. | Niskanen 2009; Stevens & Lewis 2015. |

#### Ascent sub-hierarchy

| State | Meaning | Citations |
|---|---|---|
| `ascent.boost.first_stage_burn` | First-stage engine(s) firing. | Sutton & Biblarz, *Rocket Propulsion Elements*, 9th ed. |
| `ascent.boost.staging` | Transient state during separation event. Composite parent's `on_active` handles momentum-balance check. | Sutton & Biblarz; multi-stage textbook terminology. |
| `ascent.boost.second_stage_burn` | Second-stage burn. | Idem. |
| `ascent.boost.upper_stage_burn` | Upper-stage / payload-stage burn (optional, used for 3+ stage academic configurations). | Idem. |
| `ascent.post_boost_coast` | Coast between final burn and apogee approach. *Distinct from* `mission.states.in_flight.coast` because the latter is a stand-alone state for scenarios that don't separate boost / coast (e.g. sounding rockets). | Niskanen 2009. |

#### Apogee-regime sub-hierarchy

| State | Meaning | Citations |
|---|---|---|
| `apogee_regime.apogee_approach` | Vertical velocity approaching zero from positive side. | Niskanen 2009; sounding-rocket textbook tradition. |
| `apogee_regime.apogee` | Transient state at vertical-velocity zero crossing. Conventionally fires `deploy_recovery` actions in scripted scenarios. | Niskanen 2009. |
| `apogee_regime.apogee_passed` | Vertical velocity negative; pre-descent. | Idem. |

#### In-flight stand-alone states

| State | Meaning | Citations |
|---|---|---|
| `in_flight.coast` | Engines off, ballistic flight. Used when boost/coast separation is not the appropriate decomposition (e.g. sounding rockets, coast phase between MECO and SECO). | Niskanen 2009. |

#### Descent sub-hierarchy

| State | Meaning | Citations |
|---|---|---|
| `descent.ballistic_descent` | Vehicle in atmospheric descent without recovery deployment. | Stevens & Lewis 2015. |
| `descent.entry_interface` (reserved) | Threshold crossing into denser atmosphere; conventionally 80 km altitude. | Vinh, *Hypersonic and Planetary Entry Flight Mechanics*, 1980. |
| `descent.lifting_entry` (reserved) | Re-entry phase with non-zero lift; replaces operational `Maneuvering`. | Vinh 1980; Anderson, *Hypersonic and High-Temperature Gas Dynamics*. |
| `descent.peak_heating` (reserved) | Stagnation heating at maximum; transient. | Anderson; Tauber-Sutton textbook. |
| `descent.peak_deceleration` (reserved) | Maximum dynamic-pressure deceleration; transient. | Vinh 1980. |
| `descent.drogue_descent` | Drogue parachute deployed. | Sounding-rocket textbook. |
| `descent.main_descent` | Main parachute deployed. | Idem. |
| `descent.final_descent` | Final descent before touchdown. **Replaces rejected `terminal_descent`.** | Sounding-rocket textbook (academic naming). |

#### Post-flight sub-hierarchy

| State | Meaning | Citations |
|---|---|---|
| `post_flight.touchdown` | Transient state at ground contact. | Sounding-rocket textbook. |
| `post_flight.recovery` | Recovery operations (search, retrieve, safe). | Idem. |
| `post_flight.post_flight_safing` | Vehicle systems safed, no further actions. | Idem. |
| `post_flight.safed` | Terminal state; no transitions out. | Idem. |

## The Health region

```text
health                                                       (region root, composite)
├── health.nominal                                           (atomic)
├── health.degraded                                          (atomic)
├── health.abort_requested                                   (atomic)
└── health.safed_on_fault                                    (atomic, terminal)
```

| State | Meaning |
|---|---|
| `health.nominal` | All FDIR detectors clear; commander default. |
| `health.degraded` | One or more FDIR detectors tripped, no abort yet. The mission FSM continues; downstream consumers (autopilot, mixer) can guard their behavior on this. |
| `health.abort_requested` | Commander-side abort request latched by the production health region. The published `safe_state_requested` boolean is derived from this region state for compatibility with existing consumers. |
| `health.safed_on_fault` | Terminal state; vehicle is in a known-safe configuration with all effectors disabled. |

The hypersonic extensions may extend `degraded` into a composite with sensor /
effector / aerothermal sub-states. The base platform ships the flat health
vocabulary, not a production-ticked health region.

## The Comms region

```text
comms                                                        (region root, composite)
├── comms.linked                                             (atomic)
├── comms.degraded                                           (atomic)
├── comms.loss_of_signal                                     (atomic)
└── comms.safed_on_loss_of_signal                            (atomic)
```

| State | Meaning |
|---|---|
| `comms.linked` | Ground link nominal. |
| `comms.degraded` | High-latency or partial-loss link. |
| `comms.loss_of_signal` | Confirmed loss; timer running toward `safed_on_loss_of_signal`. |
| `comms.safed_on_loss_of_signal` | Configured timeout elapsed; vehicle has safed on loss-of-signal posture. |

In simulator runs, the comms region is mocked at `linked` unless the
scenario declares a `[[mission.events]]` binding that drives it
through the academic states. In HAL deployments, the comms region is
driven by a real comms subsystem the adopter wires.

## The Estimator-Regime region

```text
estimator_regime                                             (region root, composite)
├── estimator_regime.boost_mode                              (atomic)
├── estimator_regime.coast_mode                              (atomic)
└── estimator_regime.descent_mode                            (atomic)
```

This region is *observed* by the commander but *decided* by the IMM
estimator. The IMM publishes its argmax mode
probability; the commander reflects it into the region's current
state. Cross-region guards in mission transitions can reference this
region (e.g. "transition from `apogee_regime.apogee_approach` to
`apogee_regime.apogee` only when `estimator_regime` is
`coast_mode`").

When the IMM has not yet
landed all three modes, the region defaults to
`boost_mode` and stays there. The architecture supports the region
without requiring the underlying estimator-side machinery.

## Reserved regions

The hypersonic extensions add one canonical region:

```text
aerodynamic_regime                                           (reserved)
├── aerodynamic_regime.subsonic
├── aerodynamic_regime.transonic
├── aerodynamic_regime.supersonic
├── aerodynamic_regime.hypersonic
└── aerodynamic_regime.free_molecular
```

The base platform reserves the region name but does not ship it; the
hypersonic extensions declare it through the existing
`[[mission.regions]]` extension point.

## Rejected vocabulary

The following names are **rejected** at scenario load. The scenario
parser produces a typed error citing this document; the offending
scenario must be migrated to the academic replacement before it parses.

### Operational ballistic-missile / interceptor terminology

| Rejected | Academic replacement | Reason |
|---|---|---|
| `terminal` (state name, anywhere in path) | `final_descent`, `descent`, or context-appropriate academic equivalent | Operational engagement vocabulary; reserved for "approach to target / impact". OpenBMP is academic. |
| `terminal_descent` | `final_descent` | Same. |
| `midcourse` | `coast` + `apogee_approach` | Operational ballistic-missile taxonomy (boost-midcourse-terminal). The academic equivalent decomposes into `coast` and `apogee_approach`. |
| `boost_midcourse_terminal` (taxonomy) | `pad-ascent-coast-apogee-descent-recovery` | Same. |
| `endgame` | `post_flight`, `recovery` | Operational engagement vocabulary. |
| `engagement`, `engaged` | (no replacement; rejected) | Operational. No academic context for these names in OpenBMP. |
| `strike`, `strike_phase` | (no replacement; rejected) | Operational. |
| `target` (as guidance reference state) | `waypoint`, `reference_trajectory` | Operational. Use waypoints in inertial space, never real-world locations. |
| `seeker_active`, `seeker_lock` | (no replacement; rejected) | Operational; OpenBMP does not ship target-seeking guidance. |
| `interceptor`, `intercept` | (no replacement; rejected) | Operational; categorically out of scope. |
| `max_range`, `range_max`, `target_range` | `delta_v_budget_m_s`, `staging_analysis` | Range-to-target optimization vocabulary; staging analysis is limited to vehicle-intrinsic ideal ΔV / mass budget. |
| `throw_weight` | `payload_mass_kg` | Operational payload-at-range terminology; OpenBMP uses payload mass at a declared ideal ΔV. |
| `impact_energy` | (no replacement; rejected) | Terminal-effect terminology; unrelated to forward propulsion analysis. |
| `firing_table`, `range_table` | `drag_coefficient`, `aero_deck` | Gunnery range products are rejected; OpenBMP stores geometry-intrinsic coefficient tables only. |
| `ballistic_match` | `drag_polar`, `cd0` | Operational "match this round" framing is rejected; forward aerodynamics vocabulary is accepted. |
| `threat`, `threat_track` | (no replacement; rejected) | Operational. |
| `kill`, `kill_chain` | (no replacement; rejected) | Operational. |

### Operational drone / UAV terminology

| Rejected | Academic replacement | Reason |
|---|---|---|
| `rth`, `return_to_home` | `return_to_reference_point` | Implies operational base. Academic equivalent uses an inertial-space reference. |
| `home` (state name) | `reference_point` | Same. |
| `loiter` (when implying observation of a real-world point) | `hold_position` | "Loiter" alone is fine; the rejected sense is operational observation of a target area. |

### Operational re-entry / hypersonic terminology

| Rejected | Academic replacement | Reason |
|---|---|---|
| `maneuvering` (in re-entry context) | `lifting_entry` | Already in `safety-boundaries.md`. The academic re-entry literature uses lifting-entry terminology. |
| `glide` (without "academic" qualifier) | `academic_glide`, `academic_skip_glide` | Already in `safety-boundaries.md`. Operational hypersonic-glide-vehicle programs use the unqualified term; OpenBMP preserves the academic qualifier. |
| `blackout_evasion` | (no replacement; rejected) | Operational re-entry penetration. |
| `pen_aid`, `decoy` | (no replacement; rejected) | Operational. |
| `terminal_phase` (re-entry sense) | `final_descent` | Same as ballistic-missile rejection. |

### Operational autopilot terminology

| Rejected | Academic replacement | Reason |
|---|---|---|
| `btt`, `bank_to_turn` (terminal-mode autopilot) | `bank_to_turn_academic` | Already in `safety-boundaries.md`. The academic suffix preserves the algorithm name for re-entry studies while excluding the terminal-mode framing. |
| `stt`, `skid_to_turn` (terminal-mode autopilot) | (rejected for terminal-mode use) | Same. |
| `terminal_mode_autopilot` | (no replacement; rejected) | Operational. |

### Already-rejected from `safety-boundaries.md` (cross-references)

The following names are already rejected by `safety-boundaries.md §
Naming Rules` and that rejection extends to mission states:

`Target`, `Seeker`, `Warhead`, `Strike`, `Interceptor`, `Kill`,
`Threat`, `Launch` (as a verb implying real launch),
`WeaponSystem`.

## Deprecated names with one-release shims

These names ship with a deprecation warning. A future major version will
remove the shim entirely. Scenarios using these names parse but emit
a load-time warning citing this document.

| Deprecated | Becomes Rejected when | Replacement |
|---|---|---|
| `ascent` (as a top-level state, when the scenario should declare hierarchy) | next major version | `mission.states.in_flight.ascent.<sub-state>` |
| `descent` (as a top-level state, when scenario should declare hierarchy) | next major version | `mission.states.in_flight.descent.<sub-state>` |
| `coast` (when the scenario uses both boost-end coast and pre-apogee coast) | next major version | `in_flight.ascent.post_boost_coast` and `in_flight.coast` distinguish |
| `phase` (token in scenario TOML, e.g. `[[mission.phases]]`) | next major version | `state` (e.g. `[[mission.states]]`); the `[[mission.phases]]` block is preserved as a v3 backward-compat lift to `[[mission.states]]` |
| `PhaseId` (Rust type) | next major version | `StateId`; type alias `pub type PhaseId = StateId` ships in `openbmp-mission` for one release, deprecation-warned. |

## Backward-compatibility migration

Scenarios written in the v3 syntax load through a v3 → v4 lifting
pass:

1. v3 `[[mission.phases]]` blocks map to v4 `[[mission.states]]`
   blocks with `parent` omitted (flat hierarchy).
2. v3 `enter_phase` scenario actions map to
   `MissionAction::EnterState(StateId)` with the same id (`PhaseId`
   and `StateId` are FNV-1a-64 of the same path).
3. v3 sim-only actions (`EngineCommand`, `EffectorOverride`,
   `Separation`, `DeployRecovery`) route to the new
   `openbmp-scenario-script` crate's binding list rather than the
   FC commander's binding list.
4. The lifting pass is byte-stable: the resulting v4 in-memory
   representation produces byte-identical telemetry to the original
   v3 scenario when run on the reference platform profile.

A scenario author migrating from v3 to v4 manually (recommended for
new scenarios) should:

1. Replace `[[mission.phases]]` with `[[mission.states]]`.
2. Replace `id = "ascent"` with `id = "mission.states.in_flight.ascent"`
   (canonical path with hierarchy).
3. Add `parent = "mission.states.in_flight"` for hierarchy.
4. Move scripted-physics actions to a separate `[[scenario.script]]`
   block (or keep them under `[[mission.events]]` — the parser still
   routes them to the script crate either way; the new block is
   *clearer*, not *required*).

## Scope guardrail

This vocabulary is the *full set* of state names OpenBMP supports for
academic rocket-class and re-entry / hypersonic flight.

Adding a new state name requires:

1. An academic citation establishing the name in the textbook
   tradition.
2. Demonstration that the name is not in any operational /
   engagement / weapons-employment lexicon.
3. Update to this document with the citation.
4. Update to the state-name lint in `openbmp-scenario`.

This is not a hostile review; it is a *naming-honesty* review. The
project's value depends on staying honestly academic. New states
should land easily — they just need the citation in the table.

## References

- Niskanen, S. *OpenRocket Technical Documentation*, v13.05, 2013
  (sounding-rocket academic phase vocabulary).
- Sutton, G. P. and Biblarz, O. *Rocket Propulsion Elements*, 9th
  ed., Wiley 2017 (multi-stage / staging vocabulary).
- Stevens, B. L. and Lewis, F. L. *Aircraft Control and Simulation*,
  3rd ed., Wiley 2015 (autopilot phase / mode vocabulary).
- Vinh, N. X. *Hypersonic and Planetary Entry Flight Mechanics*, U.
  Michigan Press, 1980 (re-entry phase vocabulary, lifting-entry
  terminology).
- Anderson, J. D. *Hypersonic and High-Temperature Gas Dynamics*,
  3rd ed., AIAA 2019 (peak-heating, peak-deceleration academic
  framing).
- BPS.space Signal flight-computer phase vocabulary (referenced via
  `design-concept.md § Architectural Inspirations`).

## Flight-profile extensions

The multi-phase flight-profile work adds accepted phase names
(`powered_ascent`, `apogee_approach`, `ballistic_descent`, `entry_interface`),
reaffirms the existing academic replacements (`coast`, `lifting_entry`,
`final_descent`), and adds rejected operational terms (`aimpoint`,
`miss-distance`, `terminal-guidance`, `bank-to-turn`, `skid-to-turn`,
`intercept`, `reentry-vehicle`, `circular-error`) to the parse-time lint. The
authoritative list, with rationale and the fail-closed validation rules, is in
[`profile-vocabulary-and-guardrails.md`](profile-vocabulary-and-guardrails.md).

See also:
[`mission-graph-architecture.md`](mission-graph-architecture.md),
[`safety-boundaries.md`](safety-boundaries.md),
[`flight-profiles-architecture.md`](flight-profiles-architecture.md),
[`profile-vocabulary-and-guardrails.md`](profile-vocabulary-and-guardrails.md),
[`design-concept.md`](design-concept.md).
