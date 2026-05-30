# Phalcon-9 — provenance

## What this is

**Phalcon-9** is a *wholly synthetic* demonstration launch vehicle: a
generic medium-lift, two-stage, kerosene/LOX ("kerolox") launcher used
to exercise and showcase OpenBMP's two-stage powered-ascent path
(engine→tank propellant-budget coupling, stage separation, multi-body
lanes). It is **not** a model of, and makes **no performance claim
about**, any real fielded vehicle, operator, engine, or programme.

The name is deliberately evocative of the medium-lift class it
represents. Per the project's revised doctrine (see
[`docs/real-rocket-integration.md`](../../docs/real-rocket-integration.md)
§ "Synthetic demonstration vehicles"), clearly-labelled synthetic
demonstration vehicles whose names *evoke* a class are permitted in-tree
**provided** (a) every parameter is a rounded, order-of-magnitude class
figure rather than a fielded value, (b) the artifact is tagged
`validation = "experimental"` and labelled synthetic, and (c) the
forward-only / no-targeting safety core is untouched (there is no
target, aimpoint, range, or terminal-guidance field anywhere here).

## Parameter basis (class anchors, not fielded values)

The numbers are rounded class figures for a medium-lift two-stage
kerolox launcher. They were sanity-checked for plausibility against
publicly available, order-of-magnitude figures for that vehicle class
(e.g. liftoff mass in the few-hundred-tonne range, ~7–8 MN of first-stage
sea-level thrust from a ~9-engine cluster, ~100 t of upper-stage
propellant, single restartable vacuum upper-stage engine). No proprietary,
fielded, or validated parameter set was used or reproduced; all values
are rounded illustrative figures.

The table below is the **reference class anchor**, as flown in
`phalcon9-ascent`. The other scenarios re-tune these synthetic figures
for their specific demonstration (see the per-file notes); the numbers
are all illustrative, not fidelity targets, so they vary between files.

| Quantity | Synthetic value | Notes |
|---|---|---|
| Stage-1 dry mass | 22 t | rounded class figure |
| Stage-1 propellant | ~412 t | single synthetic kerolox store, ρ≈1030 kg/m³ |
| Stage-1 engines | 9 × 850 kN, Isp 300 s | octaweb layout; round numbers |
| Stage-2 dry + payload | 19 t (4 t stage + 15 t payload) | rounded |
| Stage-2 propellant | ~108 t | single synthetic kerolox store |
| Stage-2 engine | 1 × 980 kN, Isp 348 s | vacuum-optimized; round numbers |

Ideal (loss-free) staged Δv from these figures is ≈10.4 km/s; after
nominal gravity/drag losses this lands a Phalcon-9-class vehicle in the
right order of magnitude for low Earth orbit. The figures are tuned for
demonstration, not for fidelity to any real vehicle.

**Scenario-specific re-tunes.** `phalcon9-orbit` (the flagship orbital
insertion) re-tunes the stack to close a clean two-stage insertion:
stage-1 ~10 t dry + ~552 t propellant + 9 × 850 kN at Isp 340 s; a small,
propellant-limited stage-2 of ~9 t dry + payload + ~10 t propellant +
1 × 450 kN at Isp 348 s (the eastward equatorial launch banks ~465 m/s of
Earth-rotation velocity, leaving only a small circularisation deficit).
`phalcon9-gravity-turn` is a single-stage variant (9 × 500 kN at Isp
340 s). All values remain synthetic, rounded, order-of-magnitude figures.

## Models

- Gravity: EGM2008 zonal harmonics (pinned WGS84/EGM2008 constants in
  `crates/openbmp-physics`), as used by `scenarios/leo-orbit-egm2008`.
- Propulsion: in-tree `liquid_engine` reference model + engine→tank
  propellant budget (`crates/openbmp-vehicle/src/propellant_budget.rs`).
- Tanks: `rigid_liquid` moving-mass (no slosh) — frozen-propellant
  approximation appropriate for a trajectory demo.
- Integrator: fixed-step RK4 (default), bit-stable.

## Validation status

`phalcon9-orbit` is labelled `validation = "validated-toy"`: its insertion
is checked against first-principles **orbital-mechanics invariants** by
`crates/openbmp-cli/tests/phalcon9_orbit_validation.rs` — vis-viva bound
near-circular LEO (ε < 0, e < 0.02, perigee 200–500 km, |v| 7–8.5 km/s);
axial angular momentum `(r×v)_z` conserved (< 1e-3 relative drift) on the
post-SECO coast under axisymmetric EGM2008 gravity; and two-body specific
energy conserved (< 2e-2, J2-level oscillation only). These are analytic,
open, physics-based references — **no real-vehicle data and no fielded
trajectory** are used; the vehicle remains a synthetic class anchor. The
other scenarios in this directory remain `validation = "experimental"`.

This validates that the closed-loop ascent + PEG cutoff genuinely achieves
a sustainable LEO insertion and that the integrator/gravity model conserve
the invariants — it is **not** a benchmark against, reference for, or
golden case for any real vehicle.

**Monte-Carlo robustness envelope.**
`crates/openbmp-cli/tests/phalcon9_orbit_monte_carlo.rs` re-runs the full
6-DOF ascent over a dispersed ensemble with the GNC held FIXED (same MECO/
PEG/SECO setpoints and gains every sample) while the plant and navigation
are randomised: full nav/process reseed, per-stage common-mode Isp (~0.4%
1σ) and thrust (~1.2% 1σ), per-body structural dry mass (~1.5% 1σ),
per-tank propellant underfill, and small initial-state offsets (~30 m /
~0.5 m/s 1σ). All dispersion figures are synthetic, modest, class-level —
not a tuned reproduction of any real flight-dispersion deck. Observed over
16 samples: **every** sample reaches a bound orbit and a sustainable
near-circular LEO (perigee envelope ≈ 206–339 km, e ≤ ~0.027), with the
inclination held near-equatorial across the whole ensemble (≈ 0.2–0.7°).
This shows it is the *controller*, not a single hand-tuned trajectory, that
reaches orbit.

**Near-equatorial insertion / roll-reference continuity.** The nominal
insertion is ≈ 320 × 398 km, e ≈ 0.006, inclination ≈ 0.34° for this
equatorial due-east launch. The residual inclination was previously ≈ 2.4°:
the ascent guidance resolved its roll DOF against a fixed ECI +y axis, which
degenerates and flips to +x exactly as the thrust axis swings toward
downrange (+y) at the horizontal pitch-over — a commanded-attitude
discontinuity that excited a large late-ascent pitch/yaw transient and threw
out-of-plane velocity into the orbit. Resolving the roll reference against
the orbital-plane normal (an inertial orbital element, forward-only — not a
ground location) removes the flip, and the residual inclination collapses to
sub-degree with no Δv cost. Optional active yaw (cross-track) steering is
available in the guidance but is left off here (not needed for an in-plane
equatorial ascent).

## Files

This provenance record covers the following synthetic scenario files:

- `scenarios/phalcon9/phalcon9-ascent.toml` — two-stage powered ascent
  (engine→tank propellant coupling, stage separation).
- `scenarios/phalcon9/phalcon9-booster-recovery.toml` — spent stage-1
  ballistic coast to a Monte-Carlo landing footprint.
- `scenarios/phalcon9/phalcon9-tvc-probe.toml` — single-stage,
  constant-gravity control-loop proof for closed-loop engine-gimbal
  thrust-vector control tracking an ascent pitch program.
- `scenarios/phalcon9/phalcon9-gravity-turn.toml` — single-stage
  closed-loop TVC orbital insertion under EGM2008 gravity. A full
  two-burn profile (ascent → coast → apogee circularisation → orbit)
  steered by EKF navigation + closed-loop insertion guidance reaches a
  bound, sustainable low Earth orbit: perigee ~245 km × apogee ~535 km,
  eccentricity ~0.02 (verified from telemetry). See the scenario header
  for the mission profile and the navigation fixes (EGM2008 EKF gravity,
  launch-state seed, position/velocity process noise) that make the long
  coast and precise circularisation cutoff possible.
- `scenarios/phalcon9/phalcon9-orbit.toml` — *two-stage* closed-loop TVC
  orbital insertion under EGM2008 gravity, launched **due east from the
  equator on a uniformly-rotating Earth** (`wgs84-uniform-rotation`). A
  full staged profile (stage-1 closed-loop ascent → MECO + booster
  jettison → ballistic coast to apogee → stage-2 PEG circularisation →
  orbit) reaches a near-circular, near-equatorial bound low Earth orbit:
  perigee ~320 km × apogee ~398 km, eccentricity ~0.006, inclination
  ~0.34° (verified from telemetry). The low residual inclination comes from
  resolving the ascent-guidance roll reference against the orbital-plane
  normal, which keeps the commanded attitude continuous through the
  horizontal pitch-over (see the "near-equatorial insertion" note above).
  The upper stage is cut by a CONTROLLED
  PEG time-to-go cutoff (`seco`, ~0.5 s short of circular) with propellant
  margin remaining — not a burn-to-depletion. The ascent flies through an
  atmosphere (US Standard 1976) under a synthetic drag deck
  ([`data/aero/phalcon9-drag.toml`](../../data/aero/phalcon9-drag.toml)),
  with CLOSED-LOOP max-Q load relief — the autopilot computes the real
  dynamic pressure (actual density at the navigated geocentric altitude ×
  air-relative speed²) and throttles down to hold it at the limit (peak q
  held to ~26 kPa vs a ~29 kPa unconstrained peak); MECO is at 7235 m/s to
  recover the drag/throttle loss. Exercises per-phase ascent guidance
  (closed-loop on the booster, Powered Explicit Guidance on the upper
  stage), engine-moment-about-CG rigid-body dynamics across a CG-shifting
  staging event, and an error-state EKF that gates GNSS as INDEPENDENT
  position and velocity blocks (a velocity-innovation spike under thrust
  rejects only velocity and never drops the position fix). Navigation uses
  a navigation-grade (tactical) IMU + GNSS + a STAR TRACKER — an
  independent full 3-DOF attitude fix the EKF fuses (update_star_tracker),
  which observes rotation about every axis and so closes the off-pole
  attitude-observability gap that a single magnetometer (unable to observe
  rotation about the local field, the orbital-plane normal at an
  equatorial launch) left open. The eastward launch banks the
  Earth-rotation surface speed (~465 m/s) as initial inertial velocity,
  leaving a small coast-apogee deficit. PEG steers the upper stage and the
  `seco` PEG time-to-go cutoff (gated through [fc.phase_authority] so the
  orbit phase throttles eng_vac to zero) cuts the engine ~0.5 s short of
  circular with propellant margin — a controlled cutoff. See the scenario
  header for the full profile.
- `scenarios/phalcon9/phalcon9-orbit-staged.toml` — the full discrete
  mission sequence on the same synthetic vehicle: booster separation, then
  **payload-fairing jettison** in the exo-atmospheric coast, then **payload
  deploy** in orbit. The lumped 9000 kg upper body is split into stage-2 dry
  (6900) + fairing (600) + payload (1500) at the same combined CG/inertia,
  so the pre-jettison dynamics match the flagship; the fairing and payload
  are inert bodies (no engines/tanks) jettisoned via `jettison_stage`. Each
  jettison genuinely sheds mass from the continuing stack (verified: with
  vs without jettison differ, and total mass is conserved across each
  separation), reaching a clean near-equatorial insertion (perigee ~323 km ×
  apogee ~392 km, e ~0.005, inclination ~0.34°). Exercises the multi-body
  continuing-stack mass aggregation (the continuing upper stack carries the
  still-attached fairing/payload until each departs).
- `scenarios/phalcon9/phalcon9-orbit-boostback.toml` — the same ascent with an
  **integrated booster boostback**: the booster carries a dedicated ~6 t
  boostback propellant tank + a 400 kN on-axis boostback engine (idle during
  ascent), separates at MECO, **flips retrograde** (separation attitude
  offset), and fires the boostback engine on a scripted ignite→cut window
  (t≈250–290 s) — all in the SAME run as the ascent. The upper stage still
  reaches a bound near-circular LEO (perigee ~271 km × apogee ~449 km, e
  ~0.013), while the separated booster lane burns its reserve (~5.4 t) and
  decelerates ~450 m/s (a partial boostback). This is an OPEN-LOOP (scripted)
  boostback; closed-loop guided boostback+landing needs a per-lane control
  loop (see `docs/launch-vehicle-fidelity-frontier.md`). Verified by
  `crates/openbmp-cli/tests/phalcon9_orbit_boostback_e2e.rs`.
- `scenarios/phalcon9/phalcon9-orbit-flex.toml` — the same vehicle carrying a
  first lateral structural **bending mode** (`[vehicle.bending]`, ~1.5 Hz)
  whose local slope rate the FC rate gyro picks up, so the autopilot interacts
  with the flex. The closed-loop GNC still inserts to a bound near-circular LEO
  (perigee ~265 km × apogee ~449 km, e ~0.014, inclination ~0.6°) — the
  autopilot is robust to a first bending mode at its tuned gains — while the
  insertion differs measurably from the rigid run (Δperigee ~55 km), confirming
  the bending genuinely couples (not a cosmetic model). A gyro notch
  (`[fc.autopilot_params].gyro_notch`) is the standard flex gain-stabilisation
  tool and is wired, but is not needed here. Verified by
  `crates/openbmp-cli/tests/phalcon9_orbit_flex_e2e.rs`.
- `scenarios/phalcon9/phalcon9-orbit-iers.toml` — the same vehicle and
  guidance as `phalcon9-orbit.toml`, re-flown on the higher-fidelity
  **`iers-tabulated`** Earth frame (IAU 1976 precession + IAU 1980 nutation,
  plus tabulated UT1-UTC / polar motion / length-of-day) instead of uniform
  rotation. It demonstrates that the launch flies on a non-uniform-rotation
  Earth through the FC bridge and still delivers the same clean
  near-equatorial insertion (perigee ~320 km × apogee ~398 km, e ~0.006,
  inclination ~0.34°); the frame is genuinely engaged (the trajectory
  differs from the uniform-rotation run by a few metres by orbit insertion,
  from the precession/nutation/polar-motion/LOD terms acting on the
  air-relative velocity). The Earth-orientation table it reads,
  [`scenarios/phalcon9/eop-synthetic.toml`](eop-synthetic.toml), is WHOLLY
  SYNTHETIC and illustrative — realistic-magnitude polar motion (~0.1–0.4
  arcsec), UT1-UTC (sub-second) and excess LOD (few ms), but NOT a real IERS
  Bulletin A/B record for any actual date. The `[epoch]` date only sets the
  precession/nutation reference angles.

All numeric content in these files is synthetic / rounded /
order-of-magnitude and contains no real fielded-vehicle parameter set.
Launch radii are rounded synthetic values, not the WGS84 datum.
