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
- **Environment** — US Standard Atmosphere 1976, layered-exponential,
  NRLMSISE-00 static/full, and NRLMSIS 2.x compatibility atmospheres; J2 and
  EGM2008 zonal-harmonic gravity; WMM 2025 magnetics; constant / layered /
  gust wind; full HWM14 quiet-time plus DWM07 disturbance wind.
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
  fixed-step RK4 rigid-body split with deterministic propagation of the
  departing body. Per-body ownership now routes aero, thrust, tanks, recovery,
  effectors, snapshots, and mass resources after separation; coupled-body
  effects and footprint reporting remain deferred.

## Shipped at research grade

These carry trait surfaces and engineering implementations with documented
limitations. They are labelled `research` and are intended for academic study,
not as validated fidelity.

- **Hypersonic aerodynamics** — modified-Newtonian, tangent-cone, and
  tangent-wedge methods; Knudsen-number bridging and free-molecular aero. The
  tangent-cone uses the engineering modified-Newtonian approximation rather
  than a full Taylor-Maccoll cone-shock integration.
- **Continuum launch-vehicle aerodynamics** — geometry-driven
  `ComponentBuildup` deck producer with skin friction, transonic/supersonic
  forebody drag, Mach-dependent drag polar, power-off base drag, and
  boattail/flare terms. Point-mass vehicles remain axial; rigid-body aero
  consumes attitude-derived alpha/beta.
- **Aerothermal** — Sutton-Graves stagnation heating with wall-enthalpy and
  Allen-Eggers heat-load diagnostics, reference-enthalpy distributed heating,
  boundary-layer state, the 1-D thermal-conduction toy, and the generic
  ablation toy (textbook materials only). Fay-Riddell ships a cold-gas scaffold
  plus a caller-supplied edge-state path for real-gas / CFD data and a
  neutral-composition helper for the dissociation-enthalpy term.
- **Uncertainty reporting** — per-model uncertainty contributions,
  root-sum-square aggregation, and credibility-report rendering.
- **Propulsion analysis models** — solid-grain regression for
  end-burner/BATES/tabulated burn-area inputs, liquid-engine throttle slew /
  deep-throttle / Isp derate, vehicle-side engine-to-tank propellant budgets
  with regulated or blowdown feed, and offline ideal staging ΔV / mass-optimal
  analysis. These are forward, vehicle-intrinsic models with synthetic /
  textbook parameters only.

## Deferred

These are not implemented, or remain outside a narrower validated envelope
called out above. Each is gated so that selecting it fails closed with a clear
diagnostic rather than running on placeholder behaviour. They are listed so the
scope conversation does not need to be re-derived.

**Kernel and scenario**

- Multi-rate scheduling as a first-class kernel feature.
- Coupled-body effects and post-run spent-body footprint reporting after stage
  separation. The fixed-step RK4 independent-body split is implemented; see
  [`staging-and-separation.md`](staging-and-separation.md).

**Propulsion**

- Higher-fidelity solid internal ballistics effects: erosive burning,
  ignition/tail-off transients, throat erosion, two-phase flow, temperature
  sensitivity, and thermochemistry / CEA-class `c*` solving.
- Higher-fidelity liquid feed dynamics: turbopumps, combustion stability,
  regulator depletion, injector pressure-drop dynamics, and engine/tank
  thermal coupling.
- Loss-inclusive staging budgets with gravity/drag/steering losses. The
  shipped staging analysis is ideal, loss-free, vehicle-intrinsic, and offline.

**Flight profiles**

The multi-phase ascent → coast → apogee → descent → entry profile is designed
across [`flight-profiles-architecture.md`](flight-profiles-architecture.md) and
its companions. `jettison_stage` is implemented inside the validated
fixed-step RK4 rigid-body separation envelope. PR2 implements the schema-v3
`[fc.ascent_reference]` path for pitch-program and gravity-turn references;
the explicit reference family remains reserved. PR3 implements the first
coast/footprint slice: schema-v3 `[landing_footprint]`, constant-gravity
offline footprint prediction, and range-relative / optional geodetic reporting.
PR4 implements the first descent/entry slice: schema-v3 `[entry_profile]`,
entry-interface / final-descent handoff validation, Allen-Eggers entry
diagnostics, and a corridor-limited Vinh bank-reference report. The footprint
path now includes fixed-step numerical J2 and zonal-only EGM2008 propagation.
The live entry coupling gap is closed at research-extension scope: scenarios
can select live `[aero.method]` hypersonic methods, turn force/moment models on
per mission phase with `[[forces.phase_override]]`, emit live
`aerothermal.*` telemetry, and opt rigid-body ablation into mass-rate feedback:

- Powered-ascent reference-trajectory generation (gravity-turn / pitch-program;
  explicit reference reserved) — [`ascent-guidance.md`](ascent-guidance.md).
- Coast / apogee phase wiring and the constant-gravity range-safety landing
  footprint — [`ballistic-coast-and-apogee.md`](ballistic-coast-and-apogee.md).
- Entry `entry_interface` → `lifting_entry` → `final_descent` handoff,
  diagnostics, and live force/aerothermal coupling —
  [`descent-and-entry-profiles.md`](descent-and-entry-profiles.md).

**Estimator and control**

- Parity-space residual generation; a 3-mode boost / coast / descent IMM bank
  integrated as a routing lane; MPC over the translational state.

**Environment and real-gas**

- Official NRLMSIS 2.x coefficients, once license-clean redistribution is
  available. The NRLMSISE-00 static/full paths, the OpenBMP NRLMSIS 2.x
  compatibility profile, and the full HWM14 data-file wind evaluator ship
  today.
- Verified equilibrium-air (`γ_eff`) and Park two-temperature reaction-rate
  tables. The trait surfaces exist but fail closed until clean-provenance
  public coefficient tables are imported; the Tauber-Sutton radiative model is
  reserved on the same basis.
- Verified equilibrium-air tables feeding the Fay-Riddell edge-state path.
  The current path accepts caller-supplied edge states and can derive the
  neutral dissociation-enthalpy term from `AirComposition`; executable
  equilibrium-air table lookup remains deferred.

**Aerodynamics**

- Live Reynolds-varying buildup evaluation in the hot path; the shipped path
  bakes a fixed reference-condition deck at scenario load.
- Power-on plume/base-drag coupling and nonlinear viscous-crossflow normal
  force at high angle of attack.
- Mesh-based local-inclination panel methods with shadowing (the current path
  uses a single representative station).

**Validation and HIL**

- Public-benchmark cross-validation against Apollo-class and Stardust re-entry
  trajectories, and against PX4 / ArduPilot flight logs.
- A concrete socket transport for the abstract HIL message schema (the schema
  and codec ship; the transport and any hardware adapter are downstream-adopter
  territory by design).

## Out of Scope for Upstream

OpenBMP keeps real device drivers or bus protocols, in-repo CFD / DSMC
solvers, non-Earth atmospheres, and undocumented fielded-vehicle parameter
sets outside the upstream roadmap. See
[`safety-boundaries.md`](safety-boundaries.md) for the repository scope.
