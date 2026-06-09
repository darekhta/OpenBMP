# LEO orbit EGM2008-zonal + DOPRI5(4) adaptive demo provenance

## Scenario

`scenarios/leo-orbit-egm2008-adaptive/scenario.toml`

## Source class

Synthetic. The scenario is the end-to-end demo for the
runner-side adaptive-integrator dispatch path: `[solver].profile =
"adaptive-explicit"` + `trajectory_method = "dopri54"` +
`determinism = "state-stable"` selects `Dopri54Adaptive` (Dormand-Prince
5(4) embedded RK pair under a Gustafsson PI step controller).

The initial conditions, gravity model, and orbit duration are
identical to `scenarios/leo-orbit-egm2008/scenario.toml`. See that
scenario's `provenance.md` for the closed-form circular-orbit
derivation; the only differences here are the additions of the
`[solver]` and `[solver.adaptive]` blocks.

## Adaptive solver settings

```
rtol      = 1.0e-9
atol      = 1.0e-12
min_dt_s  = 1.0e-6
max_dt_s  = 1.0
```

Tolerances are deliberately tight so the embedded error norm rarely
triggers a step rejection on this benign closed-orbit problem; the
purpose here is to wire and verify the dispatch path, not to
stress-test the adaptive accuracy. The PI controller defaults
(`α = 0.7`, `β = 0.4`, safety = 0.9, factor clamp `[0.2, 5.0]`) are the
same Gustafsson 1991 recommendation hardcoded into `Dopri54Adaptive`.

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

Synthetic OpenBMP data. No real fielded-vehicle parameters, no
real-world locations, no ITAR / EAR / MTCR / Wassenaar content.

## References

- See `scenarios/leo-orbit-egm2008/provenance.md` for the gravity
  model and orbit-IC references (Pavlis et al. 2012, Vallado 2013,
  Montenbruck & Gill 2000).
- Hairer, E., Nørsett, S. P., and Wanner, G. (1993). *Solving
  Ordinary Differential Equations I: Nonstiff Problems*, 2nd ed.,
  Springer Series in Computational Mathematics, vol. 8. §II.5,
  Table 5.2 — Dormand-Prince 5(4) Butcher tableau. The integrator's
  unit tests (`crates/openbmp-sim/src/integrator.rs`) verify the
  tableau row-sums, simplifying assumptions, and 5th-order
  convergence numerically.
- Gustafsson, K. (1991). *Control theoretic techniques for stepsize
  selection in explicit Runge-Kutta methods*. ACM Transactions on
  Mathematical Software, 17(4), 533-554. doi:10.1145/210232.210242.
  Reference for the PI controller exponents (`α = 0.7`, `β = 0.4`).

## Validation status

`experimental`. Validates parser-side and runs end-to-end
deterministically. The e2e test
(`crates/openbmp-cli/tests/leo_orbit_egm2008_adaptive_e2e.rs`)
asserts:

- the scenario completes 5556 outer kernel steps with end-time stop;
- the final ECI radius stays within 5 km of the initial radius —
  same envelope as the fixed-RK4 demo, demonstrating that the
  adaptive integrator delivers comparable accuracy under these
  tolerances;
- two reruns on the same platform produce byte-identical Parquet
  (within-platform bit-stability of the deterministic step search).

The math-side correctness of `Dopri54Adaptive` itself (5th-order
convergence on a polynomial trajectory, tableau row-sums and
simplifying assumption b·c² = c²/2, PI-factor monotonicity) is
covered by the unit tests in `crates/openbmp-sim/src/integrator.rs`.

## Scope notes

Academic integrator-method verification scenario. The orbit altitude
and inclination are scenario-internal numbers that demonstrate the
adaptive-integrator dispatch path; no geodetic site data is used.
