# Trajopt Goddard Fixture Provenance

This directory contains a public Goddard rocket benchmark fixture used by
WP-07.3 LGR evidence.

- File: `scenarios/trajopt-goddard/goddard-cops-benchmark.toml`
- Source class: public optimal-control benchmark.
- Model source: COPS/Argonne "Goddard Rocket" benchmark,
  https://www.mcs.anl.gov/~more/cops/bcops/rocket.html
- Singular-arc diagnostic source: YAPSS "Goddard Rocket Problem" example,
  https://yapss.readthedocs.io/en/stable/notebooks/goddard_problem_3_phase.html
- Model: dimensionless altitude `h`, velocity `v`, and mass `m`, with thrust
  control `T`.
- Dynamics:
  `h_dot = v`,
  `v_dot = (T - D) / m - g`,
  `m_dot = -T / c`,
  `D = D_c * v^2 * exp(-h_c * (h - h_0) / h_0)`,
  `g = g_0 * (h_0 / h)^2`.
- Stored constants follow the COPS benchmark:
  `h_0 = 1`, `v_0 = 0`, `m_0 = 1`, `m_c = 0.6`, `T_max = 3.5`,
  `c = 0.5`, `h_c = 500`, `D_c = 0.5 * v_c * m_0 / g_0 = 310`
  with `v_c = 620` and `g_0 = 1`.
- The published COPS table reports final altitudes clustered around
  `1.01283`, constraint violations near `1e-9`, and a trajectory plot with
  final mass reached near `t = 0.073`; the fixture stores those values as a
  gap-audit band for the current fixed-time LGR handoff.
- The YAPSS singular-arc path relation checked by the fixture is
  `m * g - (1 + v / c) * D = 0`.
- This is a benchmark-fixture, dynamics, singular-arc, fixed-mesh seed
  transcription, fixed-mesh and split-refined terminal-objective partial-solve
  check, and published-band gap audit. It does not claim final-altitude
  agreement, ph-mesh convergence, or full nonlinear Goddard solver closure.
