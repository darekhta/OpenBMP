# OpenBMP Profile Vocabulary and Guardrails

This document is the safety spine of the flight-profiles work. It defines the
**accepted academic vocabulary** for the new profile phases and analyses, the
**rejected operational vocabulary** added to the parse-time lint, and the
**fail-closed validation rules** that keep the new capabilities non-
weaponizable by construction. It extends
[`mission-states-vocabulary.md`](mission-states-vocabulary.md) and
[`safety-boundaries.md`](safety-boundaries.md); read it alongside
[`flight-profiles-architecture.md`](flight-profiles-architecture.md).

The premise: a complete flight profile is academically valuable and entirely
standard aerospace engineering — but the operational ballistic-missile
taxonomy and any targeting capability are not, and OpenBMP rejects them. The
discipline that already makes the platform "Open Body Motion Platform, not a
weapon" is *naming honesty plus fail-closed validation*. This work stays inside
that discipline and strengthens it.

## Accepted profile vocabulary

These phase and analysis names are added to the canon. Each is the academic
term used in the cited literature, chosen specifically because it carries no
operational engagement meaning.

| Term | Meaning | Source |
|---|---|---|
| `powered_ascent` | Thrust-on climb phase. Replaces the rejected operational thrust-on label. | Tewari, Ch. 11. |
| `coast` | Unpowered ballistic arc. Already in canon; replaces the rejected operational unpowered-cruise label. | Vallado, Ch. 8. |
| `apogee_approach` | Arc immediately before apogee. Part of the same unpowered-arc replacement. | Vallado. |
| `ballistic_descent` | Unpowered descent from apogee to entry interface. | Vallado. |
| `entry_interface` | Crossing into sensible atmosphere (≈ 122 km). Already reserved. | NASA entry convention. |
| `lifting_entry` | Entry with non-zero lift and bank modulation. Already in canon; replaces *maneuvering*. | Vinh et al., 1980. |
| `final_descent` | Low-altitude descent to recovery. Already in canon; replaces the rejected end-phase label. | — |
| `landing_footprint` | Predicted touchdown region of an unpowered body. Replaces *impact point*. | RCC 321 range-safety. |
| `dispersion_ellipse` | Statistical landing scatter. Primary schema term replacing target-accuracy wording. | RCC 321. |
| `cep50_m` | Output-only empirical 50% circular radius about the Monte-Carlo sample mean. No target or aimpoint input. | NASA/JPL D-4710. |
| `radial_offset_from_nominal_m` | Output-only radial error from the nominal forward footprint, used in sample clouds and summaries. Not a miss distance to a desired point. | Statistical post-processing. |
| `downrange_m` / `crossrange_m` | Range-relative landing coordinates. Replaces geographic aimpoint. | Range-safety convention. |
| `entry_corridor` | Heat-rate / load-factor / flight-path-angle limit band steering a lifting entry. Replaces end-phase steering language. | Vinh et al., 1980. |
| `jettison_stage` | Commanded stage / booster / fairing separation. | Niskanen, Ch. 4. |
| `ascent_reference` | Generated powered-ascent attitude reference. | Tewari, Ch. 11; PEG literature. |
| `delta_v_budget_m_s` | Ideal vehicle-intrinsic ΔV budget, never range-to-a-place. | Sutton & Biblarz; Curtis. |
| `staging_analysis` | Offline rocket-equation budget or mass-optimal split over `Isp`, structural coefficient, and payload mass. | Sutton & Biblarz; Curtis. |
| `burn_rate`, `chamber_pressure`, `grain`, `regression`, `mixture_ratio`, `mass_ratio`, `payload_fraction`, `structural_coefficient`, `klemmung`, `expansion_ratio`, `blowdown` | Propulsion mechanics vocabulary accepted for forward internal-ballistics, feed-system, and staging analysis. | Sutton & Biblarz; Huzel & Huang; Nakka. |
| `drag_coefficient`, `cd0`, `zero_lift_drag`, `drag_polar`, `base_drag`, `boattail`, `flare`, `wave_drag`, `skin_friction`, `form_factor`, `reynolds`, `mach`, `normal_force`, `center_of_pressure`, `fineness_ratio`, `ballistic_coefficient` | Aerodynamics mechanics vocabulary accepted for forward geometry-to-coefficient models. | Hoerner; Barrowman; DATCOM; Niskanen. |

## Rejected operational vocabulary (lint additions)

The `openbmp-scenario` lint (`FORBIDDEN_SAFETY_TERMS`) rejects these terms in
scenario keys and short string values at parse time, before deserialization,
with a `ScenarioError::SafetyName` error pointing here. These are added on top
of the existing rejected set documented in
[`mission-states-vocabulary.md`](mission-states-vocabulary.md).

| Rejected (normalized needle) | Label | Reason |
|---|---|---|
| `aimpoint` | aimpoint | A desired landing location. OpenBMP computes where a body lands, never steers to a chosen place. |
| `missdistance` | miss-distance | Rejected in scenario inputs because it normally denotes accuracy against a target. Output-only `*_from_nominal_m` diagnostics may compare samples to the nominal forward footprint, never to an aimpoint. |
| `terminalguidance` | terminal-guidance | Operational end-game guidance; categorically out of scope. |
| `banktoturn` | bank-to-turn | An explicitly-named terminal-mode end-game autopilot. The entry bank command is corridor-driven, not a steering mode. |
| `skidtoturn` | skid-to-turn | As above. |
| `intercept` | intercept | Operational engagement; also guards the `interceptor` family. |
| `reentryvehicle` | reentry-vehicle | Operational RV terminology; the academic term is *entry body* / *test article*. |
| `circularerror` | circular-error | Rejected in scenario inputs. The offline Monte-Carlo summary may report `cep50_m` as an output-only sample statistic about the predicted footprint. |
| `maxrange` / `rangemax` | max-range / range-max | Range maximization is a trajectory objective; staging analysis optimizes only ideal vehicle-intrinsic ΔV/mass budget. |
| `throwweight` | throw-weight | Operational payload-at-range terminology; use `payload_mass_kg` at a declared ideal ΔV. |
| `impactenergy` | impact-energy | Terminal-effect terminology, unrelated to forward propulsion analysis. |
| `firingtable` | firing-table | A range/elevation product for gunnery; the aero buildup produces vehicle-intrinsic coefficients only. |
| `rangetable` | range-table | A range-to-distance table is a targeting artifact, not an aerodynamic coefficient deck. |
| `ballisticmatch` | ballistic-match | Operational "match this round" framing; accepted terms are `drag_coefficient`, `cd0`, and `drag_polar`. |

> **Note.** Academic profile terms are deliberately *not* forbidden:
> `ballistic`, `boost` (only the *phase* sense is replaced, not the word),
> `coast`, `ascent`, `apogee`, `entry`, `descent`, `footprint`, `dispersion`,
> `downrange`, `delta_v`, `isp`, `mixture_ratio`, `mass_ratio`,
> `payload_fraction`, `structural_coefficient`, `burn_rate`,
> `chamber_pressure`, `grain`, `regression`, `klemmung`,
> `expansion_ratio`, `blowdown`, `drag_coefficient`, `cd0`,
> `drag_polar`, `base_drag`, `boattail`, `wave_drag`,
> `skin_friction`, `reynolds`, `mach`, `normal_force`, and
> `ballistic_coefficient` are all accepted. The lint targets
> engagement and targeting vocabulary, not trajectory or propulsion mechanics.

## Fail-closed validation rules

Beyond the vocabulary lint, these typed checks enforce the safety posture at
`validate()` time. They follow the project's scenario-consumer agreement
convention: where one block hardcodes or implies a capability another block
must also declare, disagreement fails closed.

1. **No desired-landing input anywhere.** No config block accepts a target
   location, aimpoint, or desired touchdown coordinate. The
   `RangeSafetyFootprint` tool takes only a propagated state and reports a
   forward prediction. Nominal-referenced `cep50_m` /
   `radial_offset_from_nominal_m` outputs are computed after propagation and
   cannot be configured against a geographic point. (Lint + absence of any
   such field in the schema.)
2. **Footprint is forward-only and offline.** The footprint path is in the
   offline-analysis surface, never on the flight controller's loop; it produces
   no actuator command. Mirrors the telemetry-viewer boundary in
   [`safety-boundaries.md`](safety-boundaries.md).
   Persisted Monte-Carlo sample-cloud files contain landing and dispersion
   diagnostics only, not the sampled burnout state or uncertainty vector paired
   with each landing point.
3. **Ascent / entry references are inertial / corridor only.** The ascent
   explicit-reference cutoff accepts inertial state (altitude, speed,
   flight-path angle); the entry-corridor reference accepts corridor limits
   (heat-rate, load-factor, flight-path angle). Neither accepts a geographic
   coordinate.
4. **Phase ↔ capability agreement.** `lifting_entry` requires a lift-capable
   `rigid_body` vehicle; `jettison_stage` requires a `rigid_body` with the
   referenced body in the assembly tree; a `coast`/`ballistic_descent` apogee
   must lie within the selected gravity and atmosphere validity envelopes.
5. **Separation symmetry.** A separation declared in `[multi_body]` must be
   commanded by a matching `jettison_stage` action and vice-versa; momentum-
   conservation must hold to tolerance.
6. **Staging analysis is offline and vehicle-intrinsic.** `[staging_analysis]`
   is schema-v3 post-processing. Its fields are limited to `delta_v_budget_m_s`,
   `payload_mass_kg`, per-stage `isp_s`, structural coefficients, and optional
   masses for forward budget mode. It produces report metadata only, never a
   controller command, and no field can express range, launch site, target, or
   aimpoint.
7. **Deferred until validated.** New variants fail closed with a typed
   `Unsupported*` / deferral error until their implementation lands with
   validation evidence. `jettison_stage` is limited to the documented
   fixed-step RK4 rigid-body envelope with explicit per-body ownership; PR2
   `ascent_reference` is limited to the
   documented schema-v3 pitch-program / gravity-turn envelope; PR3
   `landing_footprint` is limited to the schema-v3 constant-gravity, J2, and
   zonal-only EGM2008 offline footprint envelope; PR4 `entry_profile` is
   limited to handoff validation, Allen-Eggers diagnostics, and
   corridor-limited Vinh bank-reference reports.

## Amendments to existing documents

This work requires the following edits to land alongside the implementation
(noted here so the contract is explicit):

- [`mission-states-vocabulary.md`](mission-states-vocabulary.md): add the
  accepted phase names (`apogee_approach`, `ballistic_descent`,
  `entry_interface`) to the canonical state list, and add the new rejected
  terms to the rejected-vocabulary table.
- [`safety-boundaries.md`](safety-boundaries.md): under "Targeting and
  Operational Behaviour," note that ascent / entry references are
  reference-generation only and that the landing footprint is a forward,
  offline range-safety output — both explicitly distinguished from targeting.
- [`roadmap.md`](roadmap.md): mark the fixed-step RK4 separation slice,
  PR2 pitch-program / gravity-turn ascent-reference slice, and PR3
  constant-gravity landing-footprint slice as shipped, plus the PR4
  `[entry_profile]` diagnostics slice, while keeping explicit ascent
  reference, higher-order footprint propagation, richer live entry coupling,
  and per-body force ownership as Deferred entries.
- [`scenario-format.md`](scenario-format.md): document the `jettison_stage` and
  `select_guidance_profile` actions, the `[fc.ascent_reference]`,
  `[landing_footprint]`, and `[entry_profile]` blocks, and the reserved profile
  phase names.

## The standing rule

Every capability in the flight-profiles work answers a **forward** physics
question — *given this vehicle and this trajectory, what happens?* — and never
the **inverse** operational question — *given this place I want to hit, what do
I do?* The forward question is academic flight mechanics. The inverse question
is targeting, and OpenBMP does not implement it, name it, or accept its inputs.

## References

- [`mission-states-vocabulary.md`](mission-states-vocabulary.md) — the canon
  this extends.
- [`safety-boundaries.md`](safety-boundaries.md) — the binding accept / reject
  contract.
- Range Commanders Council, *Common Risk Criteria Standards for National Test
  Ranges* (RCC 321) — the public range-safety footprint / dispersion framing.
- Vinh, Busemann, Culp, *Hypersonic and Planetary Entry Flight Mechanics*,
  1980 — entry-corridor terminology.
