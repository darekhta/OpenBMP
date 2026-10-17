# Trajopt Two-Body Apogee Fixture Provenance

This is a synthetic WP-07.0 fixture for the scenario-backed trajectory
optimization CLI path.

- Files:
  - `scenarios/trajopt-two-body-apogee/scenario.toml`
  - `scenarios/trajopt-two-body-apogee/tolerance.toml`
  - `scenarios/trajopt-two-body-apogee/multiple-shooting-tolerance.toml`
  - `scenarios/trajopt-two-body-apogee/hermite-simpson-mms-tolerance.toml`
  - `scenarios/trajopt-two-body-apogee/lgr-spectral-convergence-tolerance.toml`
  - `scenarios/trajopt-two-body-apogee/hermite-simpson-scalar-integrator-benchmark.toml`
  - `scenarios/trajopt-two-body-apogee/lgr-scalar-integrator-benchmark.toml`
- Dynamics: OpenBMP point-mass runner with `environment.gravity = "point_mass"`.
- Gravity parameter: WGS84 `mu = 398600441800000.0 m^3/s^2`.
- Initial state: radius `6778000 m`, inertial velocity direction `+Y`, initial
  speed `7668.635675 m/s`, the local circular-speed seed.
- Target used by the regression: apogee radius `7578000 m`.
- Multiple-shooting cross-tier case: synthetic T0/T1 WGS84 point-mass apogee
  regression whose corrected three-segment mesh is also reused as the WP-07.2
  Hermite-Simpson T1/T2 shared-mesh cross-check. The T2 collocation tolerance is
  a declared 40 s mesh-defect tolerance, not a claim of exact flow matching.
- Hermite-Simpson MMS case: synthetic scalar manufactured solution
  `x(t) = sin(t)`, `x_dot = cos(t)`, `t in [0, 1]`, used only to check the
  direct-collocation midpoint residual order under mesh refinement.
- LGR spectral-convergence case: synthetic smooth scalar manufactured arc
  `x(t) = exp(t)`, `t in [0, 1]`, used only to check left-LGR interpolation
  error collapse under degree refinement. It is not a solved OCP benchmark.
- Hermite-Simpson scalar-integrator benchmark: synthetic validated-toy OCP
  `min integral_0^1 u(t)^2 dt`, `x_dot = u`, `x(0) = 0`, `x(1) = 1`.
  The analytic solution is `x(t) = t`, `u(t) = 1`, cost `1`, and the fixture is
  used only to check fixed-mesh deterministic dense-QP assembly.
- LGR scalar-integrator benchmark: the same synthetic validated-toy OCP
  transcribed on a fixed left-LGR mesh and solved through deterministic dense-QP
  assembly. It checks standalone solved LGR benchmark generation only, not a
  public research benchmark or full nonlinear NLP backend.
- The fixture contains no external data and no surface-coordinate target.
