# openbmp-structdyn

`openbmp-structdyn` owns low-level structural-dynamics numerics that sit below
scenario parsing and runner orchestration. The shipped surfaces are:

- WP-02.2-b station line-load recovery: deterministic dense GEMV over modal
  coordinates and accelerations, with an explicit mode-acceleration residual
  term.
- WP-02.3-a substrate: dense Craig-Bampton transform/reduction helpers, sparse
  shift-invert fixed-interface mode extraction, frequency/MAC modal
  correlation helpers, full MAC and mass-weighted cross-orthogonality matrix
  evidence helpers, and constant-step Newmark-beta/generalized-alpha modal
  transient integrators with precomputed effective-stiffness Cholesky factors
  plus explicit alternate-system step methods for variable-stiffness modal
  transients.
- WP-02.3-b partial substrate: Euler-Bernoulli beam mass/stiffness assembly,
  constant-strain plane-stress membrane triangle and bilinear quadrilateral
  membrane mass/stiffness assembly,
  Reissner-Mindlin triangular plate-bending mass/stiffness assembly,
  combined six-DOF triangular shell assembly with drilling-rotation
  regularization,
  clamped-free Blevins frequency reference checks, and declared event-force
  profiles including a closed CLA event-class library fixture for liftoff,
  max-Q, ignition, separation, and MECO templates, Dryden and von Karman gust
  envelopes, modal projection, and max-absolute modal force envelopes.
- WP-02.4-a partial substrate: longitudinal structural POGO modal receptance
  and sampled Nyquist clearance against caller-provided feed transfer samples,
  lossless 1-D feedline transfer-matrix assembly plus a uniform distributed-line
  analytic benchmark report, deterministic pump Cb/Mb dispersion-box corner
  evaluation, feed-transfer scaling for accumulator or dispersion sweeps,
  finite-dimensional lumped feedline-coupled eigenvalue/RHP checks, reduced
  first-order feed closed-loop eigenvalue/RHP checks, and a first-order
  feed/structure manufactured-transient residual evaluator. `data/feed_system`
  also carries a public NASA qualitative POGO case-history fixture for
  Gemini-Titan II and Saturn V trend evidence.

The crate intentionally does not parse scenarios, read files, emit telemetry,
or depend on the runner, simulator, vehicle, or flight-controller crates.
