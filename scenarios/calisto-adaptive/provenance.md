# Calisto + DOPRI5(4) adaptive demo provenance

## Scenario

`scenarios/calisto-adaptive/scenario.toml`

## Source class

Synthetic. The scenario is the end-to-end demo for the
runner-side adaptive-integrator dispatch on the **rigid-body**
runner: `[solver].profile = "adaptive-explicit"` +
`trajectory_method = "dopri54"` +
`determinism = "state-stable"` selects `Dopri54Adaptive` (Dormand-
Prince 5(4) embedded RK pair under a Gustafsson PI step controller
with the per-component Hairer-Nørsett-Wanner Vol I §II.4 RMS
error norm).

The vehicle, motor, aerodynamics, atmosphere, recovery state machine,
and mission graph are identical to
`scenarios/sounding-rocket/calisto/rocketpy-calisto.toml` (the
RocketPy cross-tool validation scenario). See that
scenario's `provenance.md` for the RocketPy Calisto reference data
provenance (Souza et al. 2022, Cesaroni Pro75 M1670 motor,
Spaceport America launch site, drogue + main parachute).

## Adaptive solver settings

```
rtol      = 1.0e-7
atol      = 1.0e-9
min_dt_s  = 1.0e-7
max_dt_s  = 1.0e-3   (= outer kernel dt_s)
```

These tolerances are looser than the LEO-orbit demo
(`rtol = 1e-9`, `atol = 1e-12`) because the Calisto trajectory mixes
tame 1-ms ascent dynamics with sharp drogue / main parachute
transients — tighter tolerances would push the controller into
heavy step rejection during the recovery phase. The PI controller
defaults (`α = 0.7`, `β = 0.4`, safety = 0.9, factor clamp
`[0.2, 5.0]`) are the same Gustafsson 1991 recommendation hardcoded
into `Dopri54Adaptive`.

## Per-component error norm

This scenario is the first **rigid-body** scenario to exercise the
per-component RMS error norm. The Calisto state spans nine orders of
magnitude — position ~1e3 m (within the AGL envelope), velocity
~1e2 m/s, quaternion ~1, angular velocity ~1 rad/s, mass ~17 kg,
inertia diagonal ~6 kg·m² down to small inertia off-diagonal terms.
A plain scalar form `||e||₂ / (atol + rtol · ||y||₂)` would let
quaternion and angular-velocity errors hide behind the dominant
position scale; the per-component form
`sqrt((1/N) · Σ_i ((h · e'_i / sc_i))²)` with
`sc_i = atol + rtol · max(|y^n_i|, |y^{n+1}_i|)` distinguishes them.

## Determinism

`Dopri54Adaptive` is tagged `IntegratorDeterminism::StateStable`. On a
single platform profile (target triple + toolchain + LLVM optimisation
level) it is bit-stable across reruns: the step-size search is
deterministic given identical inputs, and `last_h_s` /
`last_err_prev` start from the same `None` seed. The `state-stable`
label is for cross-platform behaviour where platform-libm differences
in `pow()` / `ln()` may lead to slightly different step-size
sequences.

## License / restrictions

Synthetic OpenBMP data wrapping the public RocketPy Calisto example
(MIT-licensed). No real fielded-vehicle parameters, no real-world
locations beyond the public Spaceport America launch site
coordinates documented in the RocketPy upstream, no ITAR / EAR /
MTCR / Wassenaar content.

## References

- See `scenarios/sounding-rocket/calisto/provenance.md` for the
  RocketPy Calisto reference data (Souza et al. 2022, Cesaroni
  Pro75 M1670 motor, Spaceport America).
- Hairer, E., Nørsett, S. P., and Wanner, G. (1993). *Solving
  Ordinary Differential Equations I: Nonstiff Problems*, 2nd ed.,
  Springer Series in Computational Mathematics, vol. 8. §II.4
  ("Practical Step-Size Control") — per-component RMS error norm
  formulation. §II.5, Table 5.2 — Dormand-Prince 5(4) Butcher
  tableau.
- Gustafsson, K. (1991). *Control theoretic techniques for stepsize
  selection in explicit Runge-Kutta methods*. ACM Transactions on
  Mathematical Software, 17(4), 533-554. doi:10.1145/210232.210242.

## Validation status

`experimental`. Validates parser-side and runs end-to-end
deterministically. The e2e test
(`crates/openbmp-cli/tests/calisto_adaptive_e2e.rs`) asserts:

- the scenario completes 180 000 outer kernel steps with end-time
  stop;
- the apogee stays within ±2 % of the RocketPy fixed-RK4 cross-tool
  baseline (3 349 m AGL) — the same envelope as the
  fixed-RK4 e2e gate, demonstrating that the adaptive integrator
  delivers comparable accuracy under these tolerances;
- two reruns on the same platform produce byte-identical Parquet
  (within-platform bit-stability of the deterministic step-size
  search).

The math-side correctness of `Dopri54Adaptive` itself
(5th-order convergence, tableau row-sums, PI-factor monotonicity,
per-component error norm) is covered by the unit tests in
`crates/openbmp-sim/src/integrator.rs` and
`crates/openbmp-models/src/state.rs`.

## Scope notes

Academic integrator-method verification scenario. The Calisto vehicle
and Spaceport America launch-site inputs are public RocketPy example
data.
