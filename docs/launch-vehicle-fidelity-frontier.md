# Launch-Vehicle Fidelity Frontier

This document records the **remaining higher-fidelity launch-vehicle gaps**
for the synthetic Phalcon-9 demonstrator, after the tractable fidelity work was
completed (full aero force+moment deck, Monte-Carlo dispersions including
winds, IERS-tabulated Earth frame, near-zero residual inclination, fairing
jettison + payload deploy via the multi-body continuing-stack mass model,
stage-2 restart, slosh coast-robustness, slosh-aware gyro-notch control wiring,
and separation tip-off rates).

The three items below are **not** configuration choices or bounded model fixes
— each is a new subsystem or a control co-design effort that warrants its own
designed, reviewed change. This note captures the integration seams already
mapped in the code so each is ready to pick up. It is forward-only and
synthetic throughout, consistent with
[`dual-use-assessment.md`](dual-use-assessment.md): every quantity remains a
rounded, class-level figure with no fielded-vehicle parameters.

Companion docs: [`staging-and-separation.md`](staging-and-separation.md),
[`ascent-guidance.md`](ascent-guidance.md),
[`software-architecture.md`](software-architecture.md).

---

## 1. Structural flex / bending modes

**Status: IMPLEMENTED end-to-end.** `openbmp-vehicle/src/structural.rs`
`BendingMode` + `openbmp-runner/src/structural.rs` `StructuralRack` + the FC-bridge
rate-gyro pickup + the rigid-body **reaction moment** (kernel-held, added to the
net body torque) + `[vehicle.bending]` + `scenarios/phalcon9/phalcon9-orbit-flex.toml`.
A first lateral bending mode is modelled, gyro-sensed, couples back into the body
dynamics, and the GNC inserts robustly through it. Byte-identical when no
`[vehicle.bending]`. **Remaining (optional refinements):** multiple modes /
distributed mode shape; and an aggressive-gain scenario demonstrating the gyro
notch rescuing a flex-UNSTABLE loop (this vehicle's tuned gains are naturally
flex-robust, so the notch isn't needed at nominal gains — it only attenuates
flex-band rate when the loop is intentionally driven flex-sensitive).

**Gap (original).** The vehicle was a rigid body; there was no lateral
structural bending.
A real launch vehicle's first bending mode couples to the autopilot through the
rate-gyro pickup (the gyro at its station senses the local bending slope rate,
not just the rigid-body rate), and an over-reactive loop can drive the mode
unstable.

**What already exists.**
- The autopilot's per-axis **gyro notch** (`AutopilotParams.gyro_notch` →
  `filtered_omega_body`, `crates/openbmp-fc/src/filters.rs` `Biquad::notch`) is
  the standard gain-stabilisation tool for exactly this, and is now
  scenario-configurable (`[fc.autopilot_params].gyro_notch`). It currently has
  no flex source to stabilise.
- The slosh `EquivalentPendulum` (`crates/openbmp-vehicle/src/tank/`) is a
  worked example of an internal oscillator that contributes a reaction
  force/moment to the rigid body each step — the same shape a bending mode
  takes.

**Integration seam / proposed design.**
1. A `BendingMode` model: generalised coordinate `q` with modal mass `m_q`,
   frequency `ω_b`, damping `ζ_b`, and a body-frame **mode shape** (deflection
   slope) at the engine station and at the gyro station. Dynamics
   `q̈ + 2ζ_bω_b q̇ + ω_b² q = Q/m_q`, where the generalised force `Q` is the
   lateral gimbal/aero forcing projected onto the mode shape (symplectic-Euler,
   matching the slosh integrator's bit-stability contract).
2. **Reaction** on the rigid body: the modal acceleration produces a
   body-frame reaction force/moment (mirror the slosh `reaction_body()` hook).
3. **Sensor pickup** (the control-relevant part): the rate gyro truth gains a
   `slope_at_gyro · q̇` term so the autopilot *sees* the bending. This is the
   change that makes flex a control problem and gives the gyro notch a real
   consumer. It touches the FC-bridge sensor-truth path
   (`crates/openbmp-runner/src/fc_bridge.rs` `truth_common`).
4. Config: `[vehicle.bending]` (modes, frequencies, damping, station slopes),
   off by default so every existing scenario is byte-identical.

**Validation.** A scenario where a gimbal step excites the first mode; assert
the bending oscillates at `ω_b`, the gyro notch (centred on `ω_b`) attenuates
the loop's response, and the insertion stays bound. This is the natural
end-to-end demonstration of the already-wired notch.

**Scope.** Largest of the three — a new module plus a sensor-truth change.

---

## 2. Integrated controlled boostback / landing

**Gap.** Booster recovery exists only as a *separate* ballistic scenario
(`scenarios/phalcon9/phalcon9-booster-recovery.toml` + footprint Monte-Carlo).
In an integrated ascent run the jettisoned booster lane propagates
**ballistically** — there is no boostback burn, entry guidance, or controlled
landing on the separated lane.

**What already exists** (updated after investigation — the gap is narrower than
"deep ballistic-only rewrite"):
- Separated lanes are **NOT ballistic**: the kernel advances each
  `SeparatedRigidBody` with the *same* force/moment/mass models as the primary,
  scoped by `active_body` (`kernel.rs` `derive_separated`). A booster lane
  already runs gravity + aero + its own engines + mass depletion, all
  owner-gated — so a **thrusting** booster lane is physically supported today.
- Engines stay addressable across separation (static `mounted_to` ownership);
  a booster engine commanded after separation routes its thrust to the lane.
- Separation **re-orientation** is now implemented
  (`stage_attitude_offset_body_xyzw`) — the booster can flip retrograde at
  separation. Tip-off rates and per-body continuing-stack mass are also in.
- **Remaining gaps for a guided boostback:** (a) RESERVED booster propellant
  (the stage-1 tank is depleted at MECO — verified the booster lane mass is
  constant post-separation; needs vehicle re-sizing / earlier staging);
  (b) the booster engines scripted/commanded to fire on the lane after
  separation; (c) per-lane guidance/control — the FC / autopilot / mixer drive
  only the **primary** lane today. (c) is the core remaining architectural
  piece (a per-lane control loop).

**Integration seam / proposed design.**
1. Allow a separated lane to retain **active engines** (the booster's engines
   leave with it; today their commands are not driven post-separation).
2. A **per-lane controller** hook: the kernel would invoke a lane-scoped
   guidance + autopilot + mixer for a flagged recovery lane, analogous to the
   primary FC loop but keyed to the lane's `BodyId`. This is the core
   architectural addition — multi-lane control, where today control is
   single-lane.
3. Recovery guidance: a boostback burn (retrograde Δv toward a landing
   *site-radius*, forward-only — not a ground target), an entry-attitude hold,
   and a terminal landing-burn law. The descent/entry corridor references in
   [`descent-and-entry-profiles.md`](descent-and-entry-profiles.md) supply the
   guidance shapes.

**Validation.** One run that ascends to orbit on the primary lane while the
booster lane performs a boostback + entry + landing-burn to a low touchdown
speed; assert both lanes' terminal states.

**Scope.** Large — multi-lane control architecture in the kernel + FC.

---

## 3. Slosh-coupled orbital-insertion closure

**Gap.** Propellant slosh is modelled (`EquivalentPendulum`) and now
coast-robust (amplitude clamp), and slosh-aware control (gyro notch) is wired —
but flying the *full* ascent with slosh on the tanks does **not** close to a
clean orbit.

**Diagnosis (this is the key result).** The stage-1 powered ascent with slosh
is stable (body rates ~0.02 rad/s). The failure is a **post-separation
upper-stage tumble**: after MECO/jettison the light upper stage (low inertia +
~10 t s2 slosh, engines off through the coast) diverges — body rates grow to
±3.7 rad/s — so the circularize PEG burn cannot steer and the orbit stalls at
perigee ~−1800 km, e ~0.22. Because the onset is in the engines-off coast, it
is **not** fixable by gimbal gains there, and a combination of (gyro notch +
reduced circularize gains + a freefall viscous-damping floor + higher slosh
ζ) was tried and did **not** close it.

**What it needs.** A genuine slosh-control co-design, not a few knobs:
slosh-state estimation/feedback (or an observer) so the controller knows the
slosh phase; a proper freefall settling model; and a retune — with convergence
that is genuinely uncertain on this 551 t-class tank with gentle gimbal
authority. The fixed-notch gain-stabilisation that works for flex (item 1) is
insufficient here because the slosh frequency drifts with axial acceleration
and fill, and the divergence is in the uncontrolled coast.

**Scope.** Control co-design with uncertain convergence; should be scoped
against whether the demonstrator needs a slosh-closed orbit at all, or whether
slosh stays demonstrated on the continuous-thrust `sloshing-tank` scenario.

---

## Status summary

| Item | State |
|------|-------|
| Structural flex / bending | IMPLEMENTED end-to-end (model + rack + gyro pickup + body reaction moment + scenario + e2e). Optional: multiple modes; aggressive-gain notch-rescue demo. |
| Integrated controlled boostback/landing | Per-lane PHYSICS confirmed working (thrusting lane supported); separation re-orientation primitive implemented. Remaining: reserved booster propellant (re-sizing) + scripted lane burn + per-lane control loop. |
| Slosh-coupled orbit closure | Diagnosed (post-separation upper-stage tumble); a knob-combination attempt did not close it. Needs slosh-control co-design. |

All three are forward-only and synthetic. None changes the project's doctrine
position: the Phalcon-9 remains a deliberately-synthetic class anchor with no
fielded-vehicle parameters.
