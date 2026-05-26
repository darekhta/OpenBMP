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
| `dispersion_ellipse` | Statistical landing scatter. Replaces *CEP / accuracy*. | RCC 321. |
| `downrange_m` / `crossrange_m` | Range-relative landing coordinates. Replaces geographic aimpoint. | Range-safety convention. |
| `entry_corridor` | Heat-rate / load-factor / flight-path-angle limit band steering a lifting entry. Replaces end-phase steering language. | Vinh et al., 1980. |
| `jettison_stage` | Commanded stage / booster / fairing separation. | Niskanen, Ch. 4. |
| `ascent_reference` | Generated powered-ascent attitude reference. | Tewari, Ch. 11; PEG literature. |

## Rejected operational vocabulary (lint additions)

The `openbmp-scenario` lint (`FORBIDDEN_SAFETY_TERMS`) rejects these terms in
scenario keys and short string values at parse time, before deserialization,
with a `ScenarioError::SafetyName` error pointing here. These are added on top
of the existing rejected set documented in
[`mission-states-vocabulary.md`](mission-states-vocabulary.md).

| Rejected (normalized needle) | Label | Reason |
|---|---|---|
| `aimpoint` | aimpoint | A desired landing location. OpenBMP computes where a body lands, never steers to a chosen place. |
| `missdistance` | miss-distance | An accuracy-against-target metric. OpenBMP reports statistical dispersion, never miss distance to an aimpoint. |
| `terminalguidance` | terminal-guidance | Operational end-game guidance; categorically out of scope. |
| `banktoturn` | bank-to-turn | An explicitly-named terminal-mode end-game autopilot. The entry bank command is corridor-driven, not a steering mode. |
| `skidtoturn` | skid-to-turn | As above. |
| `intercept` | intercept | Operational engagement; also guards the `interceptor` family. |
| `reentryvehicle` | reentry-vehicle | Operational RV terminology; the academic term is *entry body* / *test article*. |
| `circularerror` | circular-error | The "circular error probable" accuracy family; out of scope. |

> **Note.** Academic profile terms are deliberately *not* forbidden:
> `ballistic`, `boost` (only the *phase* sense is replaced, not the word),
> `coast`, `ascent`, `apogee`, `entry`, `descent`, `footprint`, `dispersion`,
> `downrange` are all accepted. The lint targets engagement and targeting
> vocabulary, not trajectory mechanics.

## Fail-closed validation rules

Beyond the vocabulary lint, these typed checks enforce the safety posture at
`validate()` time. They follow the project's scenario-consumer agreement
convention: where one block hardcodes or implies a capability another block
must also declare, disagreement fails closed.

1. **No desired-landing input anywhere.** No config block accepts a target
   location, aimpoint, or desired touchdown coordinate. The
   `RangeSafetyFootprint` tool takes only a propagated state and reports a
   forward prediction. (Lint + absence of any such field in the schema.)
2. **Footprint is forward-only and offline.** The footprint path is in the
   offline-analysis surface, never on the flight controller's loop; it produces
   no actuator command. Mirrors the telemetry-viewer boundary in
   [`safety-boundaries.md`](safety-boundaries.md).
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
6. **Deferred until validated.** New variants fail closed with a typed
   `Unsupported*` / deferral error until their implementation lands with
   validation evidence. PR1 `jettison_stage` is limited to the documented
   rigid-body gravity-only envelope; PR2 `ascent_reference` is limited to the
   documented schema-v3 pitch-program / gravity-turn envelope; PR3
   `landing_footprint` is limited to the schema-v3 constant-gravity offline
   footprint envelope.

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
- [`roadmap.md`](roadmap.md): mark the PR1 gravity-only separation slice,
  PR2 pitch-program / gravity-turn ascent-reference slice, and PR3
  constant-gravity landing-footprint slice as shipped, while keeping explicit
  ascent reference, higher-order footprint propagation, entry/descent, and
  per-body force ownership as Deferred entries.
- [`scenario-format.md`](scenario-format.md): document the `jettison_stage` and
  `select_guidance_profile` actions, the `[fc.ascent_reference]` block, and the
  reserved profile phase names.

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
