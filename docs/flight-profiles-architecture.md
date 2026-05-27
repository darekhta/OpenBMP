# OpenBMP Flight Profiles Architecture

This document specifies how OpenBMP models a complete, multi-phase
rocket-class **flight profile** — the academic ascent → coast → apogee →
descent → entry → recovery sequence that a sounding rocket, launch vehicle,
or lifting-entry test article flies. It is the umbrella for five companion
design documents that detail each phase. Read it first; the companions assume
the vocabulary and the safety framing established here.

OpenBMP already ships the *physics* most of these phases need — J2 / EGM2008
zonal gravity, atmosphere models to 1000 km, the Allen-Eggers and Vinh entry
propagators, hypersonic aerodynamics, and a deterministic kernel. What is
missing is **execution and wiring**: scripted stage separation now executes
for fixed-step RK4 rigid-body profiles with explicit per-body resource
ownership, there is no
powered-ascent reference generator, the coast and entry phases are not wired
into the mission state machine, and there is no range-safety footprint tool.
These documents close those gaps inside the platform's existing extension
seams and under its existing guardrails.

## Scope and naming posture

OpenBMP is **Open Body Motion Platform** — an academic, simulation-only
research platform. The acronym is deliberate: the platform models *body
motion*, not weapons. This document does not change that posture; it extends
it.

> **Status.** `jettison_stage` is implemented for fixed-step RK4,
> rigid-body profiles with explicit per-body force-stack ownership. The
> remaining profile work is
> still design plus fail-closed schema and trait stubs; each capability is
> rejected at scenario-load time with a typed deferral error until its
> implementation lands and carries validation evidence, exactly as
> [`roadmap.md`](roadmap.md) requires.

The full-profile work is built so that misuse stays difficult by
construction, the same way the rest of the codebase is (see
[`safety-boundaries.md`](safety-boundaries.md)):

- **Academic vocabulary only.** The rejected operational ballistic taxonomy,
  location-selection terms, and seeker terms are rejected at parse time by the
  [`mission-states-vocabulary.md`](mission-states-vocabulary.md) canon and the
  `openbmp-scenario` lint. Profiles use `powered_ascent`, `coast`,
  `apogee_approach`, `ballistic_descent`, `entry_interface`, `lifting_entry`,
  `final_descent`, and `recovery`. The full mapping and the new rejected terms
  live in [`profile-vocabulary-and-guardrails.md`](profile-vocabulary-and-guardrails.md).
- **No targeting.** No phase, guidance mode, or analysis tool in this work
  accepts a real-world geographic aimpoint or computes a guidance solution to
  one. Ascent guidance generates a *reference trajectory*; descent analysis
  reports a *range-relative landing dispersion* for range-safety and recovery
  planning. Neither closes a loop onto a target. See the per-document safety
  sections.
- **Reference, not deployable.** As with the rest of `openbmp-fc`, any
  controller surface introduced here is a simulator-local virtual controller,
  re-implementable against a downstream HAL by an adopter under their own
  qualification posture — never shipped as flight software here.

## The flight-profile phase canon

A complete profile is a path through this academic phase graph. Every name is
in the accepted vocabulary; none is operational.

```
prelaunch
   │  (ignition event)
   ▼
powered_ascent ──► gravity-turn / pitch-program reference (ascent-guidance.md)
   │  (burnout: at_mass_fraction / engine shutdown)
   ▼
coast ──────────► exo-atmospheric ballistic arc, J2/EGM propagation
   │  (apogee: at_apogee)
   ▼
apogee_approach
   │
   ▼
ballistic_descent
   │  (entry interface: at_altitude_descending ≈ 122 km)
   ▼
entry_interface
   │
   ▼
lifting_entry  (optional; Vinh L/D + bank) ── or ── ballistic entry (Allen-Eggers)
   │
   ▼
final_descent ──► recovery deploy (drogue / main), range-safety footprint
   │
   ▼
recovery → post_flight
```

Staging cuts across the ascent end of the graph: a multi-stage vehicle fires a
`jettison_stage` action at one or more burnout events, after which the spent
body propagates under gravity and drag while the upper stack continues. That
is the subject of [`staging-and-separation.md`](staging-and-separation.md).

> **Status.** `prelaunch`, `powered_ascent`, `coast`, `recovery`, and
> `post_flight` map onto phase names already accepted by the mission FSM.
> The schema-v3 `ascent_reference` guidance path is wired for pitch-program
> and gravity-turn references. The schema-v3 `landing_footprint` path is wired
> for offline constant-gravity, J2, and zonal-only EGM2008 footprints over
> `coast` / `ballistic_descent` profiles. The schema-v3 `entry_profile` path
> is wired for
> `entry_interface` / `final_descent` handoff validation and Allen-Eggers /
> Vinh entry diagnostics. Richer live entry force-stack / aerothermal coupling
> remains gated until wired. See
> [`profile-vocabulary-and-guardrails.md`](profile-vocabulary-and-guardrails.md).

## How a profile maps onto the existing platform

Nothing here is a new engine. A profile is a configuration over four existing
subsystems plus a small number of new, additive seams.

| Concern | Existing seam | What this work adds |
|---|---|---|
| Phase sequencing | `MissionStateMachine`, `EventConfig` triggers / actions (`openbmp-scenario`, `openbmp-runner/src/mission.rs`) | A PR1 `jettison_stage` action, a deferred `select_guidance_profile` action, and reserved phase names. |
| Powered-ascent shaping | Three-loop autopilot trajectory loop (`openbmp-fc/src/autopilot.rs`) | `AscentReferenceGenerator` plus pitch-program / gravity-turn producers feeding the reference (`ascent-guidance.md`). |
| Coast / descent propagation | Kernel + `GravityModel` (J2 / EGM2008), `AtmosphereModel` | Phase wiring + constant-gravity / J2 / EGM2008 `RangeSafetyFootprint` post-processing paths (`ballistic-coast-and-apogee.md`). |
| Entry | `AllenEggers`, `Vinh` in `openbmp-physics/src/reentry.rs` | Schema-v3 `[entry_profile]` handoff validation plus runner-side entry diagnostics (`descent-and-entry-profiles.md`). |
| Mass change at staging | `VehicleAssembly` tree, multi-body config (declared) | Executed jettison + simultaneous spent-stage propagation (`staging-and-separation.md`). |

The new trait surfaces live in a new `openbmp-physics` module, `profile`, so
they sit beside `gravity`, `atmosphere`, and `reentry` as peers and follow the
same `Result<_, PhysicsError>` discipline.

### Extension seams used

The design reuses, rather than bypasses, the documented seams:

- **Scenario schema.** New phases, events, and the `jettison_stage` /
  `select_guidance_profile` actions are additive to the v3 schema.
  `jettison_stage` validates against `[multi_body]` and executes in the
  fixed-step RK4 separation envelope; PR2 `guidance = "ascent_reference"`
  consumes `[fc.ascent_reference]` for pitch-program / gravity-turn reference
  generation; PR3 consumes `[landing_footprint]` for constant-gravity, J2, and
  zonal-only EGM2008 offline footprint methods; PR4 consumes
  `[entry_profile]` for entry handoff validation and diagnostics.
  `select_guidance_profile` and the remaining
  profile surfaces still follow the existing `Deferred` idiom. See
  [`scenario-format.md`](scenario-format.md).
- **Model traits.** `AscentReferenceGenerator`, `RangeSafetyFootprint`,
  `EntryCorridorReference`, and `StageSeparationModel` live in
  `openbmp-physics::profile`; the ascent, stage-separation, entry-corridor,
  and footprint paths have first implementations.
- **Fail-closed validation.** Cross-block agreement is enforced the way the
  wind / atmosphere consistency checks already are (see
  [`scenario-format.md`](scenario-format.md) and the project convention that a
  feature hardcoding a sub-component another block also declares must fail
  closed). A profile that names `lifting_entry` but selects a `point_mass`
  vehicle, or a `jettison_stage` action with no matching
  `[[vehicle.assembly.bodies]]` child, is rejected at load.

## Schema evolution

These capabilities are additive opt-in blocks on the v3 schema, matching how
`[multi_body]`, `[schedule]`, and the FC sub-blocks were introduced. No schema
version bump is required to *declare* the stubs; a bump to a future v4 is
reserved for the point where the full hierarchical profile graph and the new
blocks become first-class and validated. Until then:

- New event triggers reuse existing kinds where possible (`at_apogee`,
  `at_altitude_descending` for the entry interface, `at_mass_fraction` for
  burnout) so that no new `EventTriggerConfig` variant is needed for the common
  profile.
- New actions are additive `ScenarioActionConfig` variants:
  `jettison_stage` is implemented for the fixed-step RK4 separation envelope, while
  `select_guidance_profile` remains deferred.
- New `[fc]` and `[profile]` sub-blocks are `Option<…>` and validated to fail
  closed.

## The companion documents

| Document | Phase / concern |
|---|---|
| [Staging and Separation](staging-and-separation.md) | Executing the declared multi-body separation: jettison, separation impulse, simultaneous spent-stage propagation. |
| [Ascent Guidance](ascent-guidance.md) | Gravity-turn / pitch-program / explicit-guidance *reference-trajectory* generation for the powered-ascent phase. |
| [Ballistic Coast and Apogee](ballistic-coast-and-apogee.md) | Exo-atmospheric coast propagation, apogee detection, and the range-safety landing-footprint tool. |
| [Descent and Entry Profiles](descent-and-entry-profiles.md) | Wiring Allen-Eggers / Vinh into live `entry_interface` → `lifting_entry` → `final_descent` phases and recovery. |
| [Profile Vocabulary and Guardrails](profile-vocabulary-and-guardrails.md) | The accepted / rejected vocabulary additions and the fail-closed validation that keeps profiles non-weaponizable. |

## References

- Vallado, D. A. *Fundamentals of Astrodynamics and Applications*, 4th ed.,
  Microcosm Press, 2013 — coast / ballistic-arc propagation, J2 secular terms.
- Vinh, N. X. *Optimal Trajectories in Atmospheric Flight*, Elsevier, 1981;
  and Vinh, Busemann, Culp, *Hypersonic and Planetary Entry Flight Mechanics*,
  Univ. Michigan Press, 1980 — lifting-entry equations.
- Allen, H. J., Eggers, A. J. *A Study of the Motion and Aerodynamic Heating of
  Ballistic Missiles Entering the Earth's Atmosphere at High Supersonic
  Speeds*, NACA TR-1381, 1958 — ballistic-entry closed form. (Used here for
  the *physics*; OpenBMP ships no operational parameters.)
- Niskanen, S. *OpenRocket Technical Documentation*, 2013 — sounding-rocket
  ascent and recovery staging conventions.
- Stevens, B. L., Lewis, F. L. *Aircraft Control and Simulation*, 3rd ed.,
  Wiley, 2015 — the three-loop autopilot the ascent reference feeds.
