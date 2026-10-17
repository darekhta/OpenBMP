# Trajectory-Optimization Fixtures Provenance

## acikmese-ploen-mars-glideslope-v1.toml

- File: `data/trajopt/acikmese-ploen-mars-glideslope-v1.toml`
- Source: B. Acikmese and S. R. Ploen, "Convex Programming Approach to
  Powered Descent Guidance for Mars Landing," Journal of Guidance, Control, and
  Dynamics 30(5):1353-1366, 2007.
- DOI: https://doi.org/10.2514/1.27553
- Fixture scope: the published glideslope-constrained Mars soft-landing
  numerical example with `m_wet = 1905 kg`, `m_dry = 1505 kg`, six canted
  engines, `Isp = 225 s`, `T* = 3.1 kN`, `rho_1 = 0.3`, `rho_2 = 0.8`,
  `phi = 27 deg`, `g = 3.7114 m/s^2`, `tf = 81 s`, `r0 = [1.5, 0, 2] km`,
  `v0 = [-75, 0, 100] m/s`, and the reported glideslope-constrained fuel
  usage `399.5 kg`.
- OpenBMP mapping: the paper's altitude axis is mapped to local `up`; the
  paper's third coordinate is mapped to local `x`; the paper's second
  coordinate is mapped to local `y`. This keeps the LCvx target a local
  Cartesian state and intentionally introduces no latitude, longitude, range,
  aimpoint, or surface-coordinate vocabulary.
- Limitation: this fixture is a published-case LCvx regression target, not a
  flight-validation record or a ground-targeting interface.

## szmuk-acikmese-6dof-free-final-time-v1.toml

- File: `data/trajopt/szmuk-acikmese-6dof-free-final-time-v1.toml`
- Source: M. Szmuk and B. Acikmese, "Successive Convexification for 6-DoF Mars
  Rocket Powered Landing with Free-Final-Time," AIAA SciTech Forum, 2018,
  AIAA 2018-0617.
- DOI: https://doi.org/10.2514/6.2018-0617
- arXiv: https://arxiv.org/abs/1802.03827
- Fixture scope: the published notional non-dimensional 6-DoF powered-landing
  example constants from Tables 1 and 2: `g_I = -e1`, `m_wet = 2`,
  `m_dry = 1`, `T_min = 0.30`, `T_max = 5.00`, `delta_max = 20 deg`,
  `theta_max = 90 deg`, `gamma_gs = 20 deg`, `omega_max = 60 deg/UT`,
  `J_B = 1e-2 * I`, `r_T,B = -1e-2 * e1`, `w_nu = 1e5`, trust-region weights
  `1e-3` and `1e-1`, `nu_tol = 1e-10`, `Delta_tol = 1e-3`, 50 discretization
  points, 15 maximum iterations, and initial time guesses from 1 to 10 `UT`.
- OpenBMP mapping: the benchmark remains in the paper's local inertial
  non-dimensional frame and is consumed only as a local state-vector powered
  landing fixture. It introduces no latitude, longitude, range, aimpoint, or
  surface-coordinate vocabulary.
- Limitation: the paper publishes figures and convergence behavior, not
  machine-readable trajectory samples. The current fixture pins constants and
  model-generation handoff evidence; it is not by itself a full trajectory-match
  closure or flight-validation record.
