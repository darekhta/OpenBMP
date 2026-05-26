# Roadmap

OpenBMP is a working platform, not a finished one in the sense that every
conceivable model is implemented. This document is the honest map of what is
**shipped and validated**, what is **shipped at research grade**, and what is
**deferred**. It replaces the per-phase delivery plans that tracked the
platform's construction.

The guiding rule: a capability ships only when its validity envelope, its
numerical-error evidence, its data provenance, and its validation label are
all in place. Capabilities that depend on data we have not yet sourced with
clean provenance fail closed rather than ship with placeholder numbers.

## Shipped and validated

These capabilities have closed-form or public-benchmark validation evidence
and are gated in CI.

- **Deterministic kernel** — fixed-step RK4 lockstep propagation with
  byte-stable Parquet, exercised by the analytic-toy, sounding-rocket, and
  rigid-body determinism gates.
- **Translational and rigid-body dynamics** — point-mass and 6-DOF
  rigid-body propagation with the full assembly tree, engine cluster, control
  effectors, tanks, and recovery devices.
- **Environment** — US Standard Atmosphere 1976, layered-exponential and
  NRLMSISE-00 static atmospheres; J2 and EGM2008 zonal-harmonic gravity; WMM
  2025 magnetics; constant / layered / gust wind.
- **Estimators** — error-state EKF, MEKF, square-root UKF, and a 2-mode
  interacting-multiple-model bank, with multi-lane routing and a voter.
- **Autopilots** — PID, LQR, INDI, L1-adaptive, and receding-horizon attitude
  MPC rate loops; minimum-snap differential-flatness trajectory tracking;
  prioritised redistributed control allocation; observer-form anti-windup.
- **FDIR** — voter / health seam with a windowed mean-shift GLRT detector.
- **Integrators** — Dormand-Prince 5(4) and 8(5,3) in fixed-step and
  adaptive forms behind an explicit solver profile.
- **Re-entry trajectory tools** — Allen-Eggers ballistic closed form and the
  Vinh lifting-entry equations, cross-checked against their analytic forms.
- **PR1 stage-separation core** — `jettison_stage` validates against
  `[multi_body]`, conserves linear momentum to tolerance, and executes a
  fixed-step RK4, gravity-only rigid-body split with deterministic propagation
  of the departing body. Broader force ownership and footprint reporting remain
  deferred.

## Shipped at research grade

These carry trait surfaces and engineering implementations with documented
limitations. They are labelled `research` and are intended for academic study,
not as validated fidelity.

- **Hypersonic aerodynamics** — modified-Newtonian, tangent-cone, and
  tangent-wedge methods; Knudsen-number bridging and free-molecular aero. The
  tangent-cone uses the engineering modified-Newtonian approximation rather
  than a full Taylor-Maccoll cone-shock integration.
- **Aerothermal** — Sutton-Graves stagnation heating, reference-enthalpy
  distributed heating, boundary-layer state, the 1-D thermal-conduction toy,
  and the generic ablation toy (textbook materials only). Fay-Riddell ships a
  cold-gas scaffold.
- **Uncertainty reporting** — per-model uncertainty contributions,
  root-sum-square aggregation, and credibility-report rendering.

## Deferred

These are not implemented, or remain outside a narrower validated envelope
called out above. Each is gated so that selecting it fails closed with a clear
diagnostic rather than running on placeholder behaviour. They are listed so the
scope conversation does not need to be re-derived.

**Kernel and scenario**

- Multi-rate scheduling as a first-class kernel feature.
- Per-body force-stack ownership after stage separation (aero, thrust, tanks,
  recovery), coupled-body effects, and post-run spent-body footprint reporting.
  The PR1 gravity-only split is implemented; see
  [`staging-and-separation.md`](staging-and-separation.md).

**Flight profiles**

The multi-phase ascent → coast → apogee → descent → entry profile is designed
across [`flight-profiles-architecture.md`](flight-profiles-architecture.md) and
its companions. PR1 `jettison_stage` is implemented only inside the validated
gravity-only separation envelope. PR2 implements the schema-v3
`[fc.ascent_reference]` path for pitch-program and gravity-turn references;
the explicit reference family remains reserved. The coast/footprint and
entry/descent capabilities still fail closed until they land with validation
evidence:

- Powered-ascent reference-trajectory generation (gravity-turn / pitch-program;
  explicit reference reserved) — [`ascent-guidance.md`](ascent-guidance.md).
- Coast / apogee phase wiring and the range-safety landing footprint —
  [`ballistic-coast-and-apogee.md`](ballistic-coast-and-apogee.md).
- Live `entry_interface` → `lifting_entry` → `final_descent` handoff —
  [`descent-and-entry-profiles.md`](descent-and-entry-profiles.md).

**Estimator and control**

- Parity-space residual generation; a 3-mode boost / coast / descent IMM bank
  integrated as a routing lane; MPC over the translational state.

**Environment and real-gas**

- The full coefficient-based NRLMSISE-00 path (the static-defaults profile
  ships today); NRLMSIS 2.x and HWM14 winds.
- Verified equilibrium-air (`γ_eff`) and Park two-temperature reaction-rate
  tables. The trait surfaces exist but fail closed until clean-provenance
  public coefficient tables are imported; the Tauber-Sutton radiative model is
  reserved on the same basis.
- Real-gas-coupled Fay-Riddell edge-state heating.

**Aerodynamics**

- Mesh-based local-inclination panel methods with shadowing (the current path
  uses a single representative station).

**Validation and HIL**

- Public-benchmark cross-validation against Apollo-class and Stardust re-entry
  trajectories, and against PX4 / ArduPilot flight logs.
- A concrete socket transport for the abstract HIL message schema (the schema
  and codec ship; the transport and any hardware adapter are downstream-adopter
  territory by design).

## Permanently out of scope

Independent of demand, OpenBMP never ships targeting or terminal-guidance
logic, real fielded-vehicle or operational thermal-protection parameter sets,
real device drivers or bus protocols, in-repo CFD / DSMC solvers, non-Earth
atmospheres, or any weapon-employment capability. See
[`safety-boundaries.md`](safety-boundaries.md) for the binding contract.
