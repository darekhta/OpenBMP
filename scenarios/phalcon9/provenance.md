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
  perigee ~334 km × apogee ~381 km, eccentricity ~0.004, inclination
  ~1.8° (verified from telemetry). The upper stage is cut by a CONTROLLED
  PEG time-to-go cutoff (`seco`, ~0.5 s short of circular) with propellant
  margin remaining — not a burn-to-depletion. The ascent flies through an
  atmosphere (US Standard 1976) under a synthetic drag deck
  ([`data/aero/phalcon9-drag.toml`](../../data/aero/phalcon9-drag.toml)),
  with CLOSED-LOOP max-Q load relief — the autopilot computes the real
  dynamic pressure (actual density at the navigated geocentric altitude ×
  air-relative speed²) and throttles down to hold it at the limit (peak q
  held to ~26 kPa vs a ~29 kPa unconstrained peak); MECO is at 7270 m/s to
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

All numeric content in these files is synthetic / rounded /
order-of-magnitude and contains no real fielded-vehicle parameter set.
Launch radii are rounded synthetic values, not the WGS84 datum.
