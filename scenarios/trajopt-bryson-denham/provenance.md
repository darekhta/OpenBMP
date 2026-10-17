# Trajopt Bryson-Denham Fixture Provenance

This directory contains a public state-constrained optimal-control benchmark
fixture for WP-07.2 and WP-07.3.

- File: `scenarios/trajopt-bryson-denham/bryson-denham-state-constrained-benchmark.toml`
- Source class: public known-optimal optimal-control benchmark.
- Reference source: A. E. Bryson and Y.-C. Ho, *Applied Optimal Control*,
  Hemisphere, 1975. The benchmark is the standard Bryson-Denham double-integrator
  state-constrained problem also listed in `docs/parity/07-trajectory-optimization-and-mission-design.md`.
- Problem:
  `min 0.5 * integral_0^1 u(t)^2 dt`,
  `x_dot = v`, `v_dot = u`,
  `x(0) = 0`, `v(0) = 1`, `x(1) = 0`, `v(1) = -1`,
  with hard state path constraint `x(t) <= 1/9`.
- Closed-form active structure for this normalized fixture:
  first arc `u(t) = 18t - 6` for `0 <= t <= 1/3`, constrained coast
  `x = 1/9`, `v = 0`, `u = 0` for `1/3 <= t <= 2/3`, and symmetric exit
  arc `u(t) = -18 * (t - 2/3)` for `2/3 <= t <= 1`.
- Expected objective for the stated `0.5 * integral u^2` convention is `4`.
- This fixture checks fixed-mesh Hermite-Simpson and LGR equality assembly, hard
  path inequality consumption, and deterministic dense-QP solve behavior. It
  does not claim mesh refinement, sparse KKT factorization, or a general
  nonlinear benchmark NLP solve.
