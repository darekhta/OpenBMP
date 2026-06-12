# Trajopt Two-Body Apogee Fixture Provenance

This is a synthetic WP-07.0 fixture for the scenario-backed trajectory
optimization CLI path.

- Files:
  - `scenarios/trajopt-two-body-apogee/scenario.toml`
  - `scenarios/trajopt-two-body-apogee/tolerance.toml`
  - `scenarios/trajopt-two-body-apogee/multiple-shooting-tolerance.toml`
- Dynamics: OpenBMP point-mass runner with `environment.gravity = "point_mass"`.
- Gravity parameter: WGS84 `mu = 398600441800000.0 m^3/s^2`.
- Initial state: radius `6778000 m`, inertial velocity direction `+Y`, initial
  speed `7668.635675 m/s`, the local circular-speed seed.
- Target used by the regression: apogee radius `7578000 m`.
- The fixture contains no external data and no surface-coordinate target.
