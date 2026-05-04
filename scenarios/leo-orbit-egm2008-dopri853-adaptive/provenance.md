# LEO orbit EGM2008-zonal + DOP853 adaptive demo provenance

## Scenario

`scenarios/leo-orbit-egm2008-dopri853-adaptive/scenario.toml`

## Source class

Synthetic. The scenario is the Phase-5.D.6 end-to-end demo for the
runner-side adaptive-integrator dispatch on the new
`(adaptive-explicit, dopri853, state-stable)` triple, which selects
`openbmp_sim::Dopri853Adaptive` (Dormand-Prince 8(5,3) embedded RK
pair under an I-controller and the SciPy / Hairer combined err5/err3
stabilised error norm).

The initial conditions, gravity model, and orbit duration are
identical to `scenarios/leo-orbit-egm2008-adaptive/scenario.toml`
(the Phase-5.D.4 DOPRI5(4) demo). See that scenario's `provenance.md`
for the closed-form circular-orbit derivation; the only differences
here are the `[solver].trajectory_method = "dopri853"` selection and
the tighter tolerances tuned to the 8th-order method.

## Adaptive solver settings

```
rtol      = 1.0e-16
atol      = 1.0e-19
min_dt_s  = 1.0e-6
max_dt_s  = 1.0
```

Tolerances are tighter than the §5.D.4 DOPRI5(4) demo
(`rtol = 1e-9`, `atol = 1e-12`). The 8th-order accuracy of DOP853
is wasted at loose tolerances — at `rtol = 1e-12` the controller's
accepted step sequence still collapses to the outer `dt_s = 1.0`
grid and produces byte-identical Parquet to the fixed-step DOP853
variant. With `rtol = 1e-16`, `atol = 1e-19` the first 1 s trial
step is outside tolerance, so the adaptive path takes deterministic
internal sub-steps and the e2e test asserts the resulting Parquet is
not byte-identical to a staged fixed-step DOP853 run.

## Determinism

`Dopri853Adaptive` is tagged `IntegratorDeterminism::StateStable`. On
a single platform profile (target triple + toolchain + LLVM
optimisation level) it is bit-stable across reruns: the step-size
search is deterministic given identical inputs, and `last_h_s`
starts from the same `None` seed. The `state-stable` label is for
cross-platform behaviour where platform-libm differences in `pow()`
may lead to slightly different step-size sequences.

## License / restrictions

Synthetic OpenBMP data. No real fielded-vehicle parameters, no
real-world locations, no ITAR / EAR / MTCR / Wassenaar content.

## References

- See `scenarios/leo-orbit-egm2008/provenance.md` for the gravity
  model and orbit-IC references (Pavlis et al. 2012, Vallado 2013,
  Montenbruck & Gill 2000).
- Hairer, E., Nørsett, S. P., and Wanner, G. (1993). *Solving
  Ordinary Differential Equations I: Nonstiff Problems*, 2nd ed.,
  §II.5 Table 5.4 — DOP853 12-stage Butcher tableau.
- Prince, P. J., and Dormand, J. R. (1981). *High order embedded
  Runge-Kutta formulae*. J. Comp. Appl. Math. 7(1):67-75 — original
  Prince-Dormand 8(7) paper; the SciPy `DOP853` we mirror is
  Hairer's reformulation with a 5(3) embedded estimator.
- SciPy `scipy/integrate/_ivp/dop853_coefficients.py` (Hairer's
  reference Fortran `dop853.f` ported to Python) — the canonical
  numeric pin source for the tableau coefficients.

## Validation status

`experimental`. Validates parser-side and runs end-to-end
deterministically. The Phase-5.D.6 e2e test
(`crates/openbmp-cli/tests/leo_orbit_egm2008_dopri853_adaptive_e2e.rs`)
asserts:

- the scenario completes 5556 outer kernel steps with end-time stop;
- the final ECI radius stays within 5 km of the initial radius —
  same envelope as the DOPRI5(4) demo, demonstrating that DOP853
  delivers comparable accuracy on this benign orbit;
- two reruns on the same platform produce byte-identical Parquet
  (within-platform bit-stability of the deterministic step search).

The math-side correctness of `Dopri853Adaptive` itself
(tableau row-sums, embedded estimator weight sums, 8th-order
convergence on a polynomial trajectory) is covered by the unit
tests in `crates/openbmp-sim/src/integrator.rs`.

## Safety boundary

`docs/safety-boundaries.md` accept list — academic integrator-method
verification scenario, no guidance / navigation / control logic, no
target geometry, no real-world locations.
