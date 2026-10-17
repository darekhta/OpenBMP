# Trajopt Brachistochrone Fixture Provenance

This directory contains a public closed-form benchmark fixture used by WP-07.2
Hermite-Simpson evidence and WP-07.3 LGR transcription/cost evidence.

- File: `scenarios/trajopt-brachistochrone/brachistochrone-cycloid-benchmark.toml`
- Source class: public analytic optimal-control benchmark.
- Reference equation source: Wolfram MathWorld, "Brachistochrone Problem",
  https://mathworld.wolfram.com/BrachistochroneProblem.html
- Model: frictionless point mass in a uniform gravity field, with down-positive
  vertical coordinate.
- Closed-form solution:
  `x = a * (theta - sin(theta))`,
  `y = a * (1 - cos(theta))`,
  `t = sqrt(a / g) * theta`.
- Endpoint: `(x, y_down) = (1 m, 1 m)`.
- Gravity: standard gravity `g = 9.80665 m/s^2`.
- The fixture solves `(1 - cos(theta_f)) / (theta_f - sin(theta_f)) = 1` for
  the equal-range/equal-drop endpoint, then stores `theta_f`, `a`, and the
  resulting optimal time.
- This is a public closed-form transcription/cost benchmark for HS and LGR
  evaluation paths. It does not claim a full nonlinear time-minimizing solve,
  mesh refinement, or sparse KKT factorization.
