# GNC Data Provenance

```yaml
dataset_id: openbmp.gnc.qcat_wls_allocation.v1
files:
  - data/gnc/qcat-wls-allocation-tolerance-v1.toml
source_class: public-open-source-tool
source_title: "QCAT wls_alloc ADMIRE and synthetic TVC control-allocation tolerance table"
source_authors:
  - "Ola Härkegård"
source_url: "https://www.mathworks.com/matlabcentral/fileexchange/4609-qcat"
source_mirror_url: "https://github.com/mengchaoheng/qcat/tree/336f749ac036d2dbe825d8a34e98b90bc09d29ef"
license_or_terms: "QCAT BSD-style license in mirrored package; no toolbox source is vendored in OpenBMP."
retrieved_utc: "2026-06-25"
source_hash_sha256: "not pinned: MathWorks download requires interactive access; mirror commit 336f749ac036d2dbe825d8a34e98b90bc09d29ef is recorded as the extraction reference"
transformation:
  method: "read QCAT/qcat/admiredata.mat variables B, plim, and v; evaluate the documented QCAT wls_alloc weighted bounded least-squares active-set routine with gamma = 1e12, unit Wv/Wu, zero preferred command, and the listed virtual-command samples; add a synthetic 2x3 TVC cluster with the same routine"
  script: "manual one-off extraction using scipy.io.loadmat and a direct transcription of QCAT wls_alloc.m semantics; compact numeric inputs and outputs are recorded in the TOML table"
verification:
  method: "parse the TOML table and compare OpenBMP WlsActiveSetAllocator commands, virtual residuals, active-set signs, and iteration counts against the QCAT-derived expected values"
  test: "cargo test -p openbmp-fc dense_wls_allocator_matches_qcat_admire_and_synthetic_tolerance_tables --locked"
  tolerance: "absolute command/residual tolerances recorded in [tolerances]; active set and iteration count are exact"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "ADMIRE is a public academic aircraft allocation benchmark and the synthetic TVC cluster is an OpenBMP toy fixture. The table validates allocation numerics only; it is not a real launch-vehicle B matrix or flight-control I-load."
```

```yaml
dataset_id: openbmp.gnc.coupled_lqr_dare.v1
files:
  - data/gnc/coupled-lqr-dare-tolerance-v1.toml
source_class: derived-open-method
source_title: "Coupled discrete LQR DARE tolerance table"
source_url: "https://python-control.readthedocs.io/"
license_or_terms: "OpenBMP generated numeric regression fixture; no third-party source code or proprietary vehicle data."
retrieved_utc: "2026-06-25"
source_hash_sha256: "not applicable: generated fixture; compact numeric inputs and outputs are recorded in the TOML table"
transformation:
  method: "evaluate control.dlqr(A, B, Q, R), backed by SciPy solve_discrete_are, and record P and K for one scalar integrator case and one coupled three-state/two-input toy plant"
  script: "manual one-off local Python script in a temporary virtualenv using control 0.10.2 and scipy 1.18.0"
verification:
  method: "parse the TOML table and compare OpenBMP solve_dare_nxn_doubling P, K, residual norm, iteration bound, and bit-stability against the recorded expected values"
  test: "cargo test -p openbmp-fc coupled_lqr_dense_solver_matches_tolerance_table --features lqr --locked"
  tolerance: "absolute P/K and residual thresholds recorded in [tolerances]; iteration bound is exact"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "The coupled plant is a toy numeric verification case only. It is not a real vehicle model, target law, mission profile, or flight-control I-load."
```

```yaml
dataset_id: openbmp.gnc.gain_schedule_lqg.v1
files:
  - data/gnc/gain-schedule-lqg-tolerance-v1.toml
source_class: derived-open-method
source_title: "Scalar gain-schedule and LQG observer tolerance table"
source_url: "https://python-control.readthedocs.io/"
license_or_terms: "OpenBMP generated numeric regression fixture; no third-party source code or proprietary vehicle data."
retrieved_utc: "2026-06-25"
source_hash_sha256: "not applicable: generated fixture; compact numeric inputs and outputs are recorded in the TOML table"
transformation:
  method: "record a two-node scalar Mach schedule with linear interpolation and a toy bumpless integral-state reset; evaluate the LQG observer covariance/gain with SciPy solve_discrete_are on the dual system"
  script: "manual one-off local Python script in a temporary virtualenv using control 0.10.2 and scipy 1.18.0"
verification:
  method: "parse the TOML table and compare OpenBMP GainScheduleTable interpolation, observer-gain interpolation, conservative closed-loop row-sum sweep, bumpless reset, and solve_lqg_observer_nxn_dual_dare outputs against the recorded expected values"
  test: "cargo test -p openbmp-fc gain_schedule_lqg_tolerance_table --features lqr --locked && cargo test -p openbmp-fc lqg_observer_dual_dare_matches_tolerance_table --features lqr --locked"
  tolerance: "absolute interpolation/state/observer tolerances and minimum row-sum margin are recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "The fixture validates scheduling, bumpless-transfer, and observer-gain numerics only. It is not a real vehicle model, target law, mission profile, or flight-control I-load."
```

```yaml
dataset_id: openbmp.gnc.bending_filter_tustin.v1
files:
  - data/gnc/bending-filter-tustin-tolerance-v1.toml
source_class: derived-open-method
source_title: "Prewarped-Tustin bending/slosh notch tolerance table"
license_or_terms: "OpenBMP generated numeric regression fixture; no third-party source code or proprietary vehicle data."
retrieved_utc: "2026-06-26"
source_hash_sha256: "not applicable: generated fixture; compact numeric inputs and outputs are recorded in the TOML table"
transformation:
  method: "evaluate the closed-form bilinear transform s = K (z - 1)/(z + 1), K = omega_mode / tan(omega_mode * dt / 2), for two toy Mach schedule knots and one interior physical-parameter interpolation point; assemble a two-mode analytic bending-plus-slosh modal CSI plant and evaluate the scheduled digital filter cascade over a deterministic frequency grid"
  script: "manual one-off local calculation using the closed-form equations recorded in docs/parity/06-gnc-coupled-mimo-and-control.md"
verification:
  method: "parse the TOML table and compare OpenBMP prewarped_tustin_notch coefficients, stable-pole validation, physical-parameter schedule interpolation, mode-frequency attenuation, and filtered analytic bending/slosh margins against the recorded expected values; exercise the off-default autopilot runtime insertion path with the same physical section vocabulary"
  test: "cargo test -p openbmp-fc scheduled_bending_filter_runtime --locked && cargo test -p openbmp-fc csi_filtered_structural_margin_fixture_meets_nodes_and_interior --locked && cargo test -p openbmp-fc bending_filter --locked && cargo test -p openbmp-fc prewarped_tustin --locked"
  tolerance: "absolute coefficient, magnitude, and margin tolerances recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "The fixture validates a generic toy filter-scheduling and analytic modal-margin method only. It is not a real vehicle structural model, target law, mission profile, or flight-control I-load."
```

```yaml
dataset_id: openbmp.gnc.load_relief_alpha_beta.v1
files:
  - data/gnc/load-relief-alpha-beta-tolerance-v1.toml
source_class: derived-open-method
source_title: "Load-relief alpha/beta geometry and q-alpha limiter tolerance table"
license_or_terms: "OpenBMP generated numeric regression fixture; no third-party source code or proprietary vehicle data."
retrieved_utc: "2026-06-26"
source_hash_sha256: "not applicable: generated fixture; compact numeric inputs and outputs are recorded in the TOML table"
transformation:
  method: "evaluate alpha = atan2(w_rel_body, u_rel_body), beta = atan2(v_rel_body, u_rel_body), qbar = 0.5 * rho * |V_rel|^2 for identity body axes and apply a structural q-alpha ceiling to a deterministic toy gust sample set"
  script: "manual one-off local calculation using the equations recorded in docs/parity/06-gnc-coupled-mimo-and-control.md"
verification:
  method: "parse the TOML table and compare OpenBMP estimate_alpha_beta and evaluate_load_relief outputs against the recorded expected values; exercise the q-alpha limiter on a deterministic toy gust series"
  test: "cargo test -p openbmp-fc load_relief --locked"
  tolerance: "absolute alpha, beta, and qbar tolerances recorded in [tolerances]; limiter bound is exact to floating-point epsilon"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "The fixture validates geometry and structural-load limiting only. It is not a real vehicle wind profile, target law, mission profile, or flight-control I-load."
```

```yaml
dataset_id: openbmp.gnc.tvc_twd_inca.v1
files:
  - data/gnc/tvc-twd-inca-tolerance-v1.toml
source_class: derived-open-method
source_title: "TVC actuator, TWD reaction, and dense-INCA recovery tolerance table"
license_or_terms: "OpenBMP generated numeric regression fixture; no third-party source code or proprietary vehicle data."
retrieved_utc: "2026-06-26"
source_hash_sha256: "not applicable: generated fixture; compact numeric inputs and outputs are recorded in the TOML table"
transformation:
  method: "evaluate the closed-form second-order actuator step response, the same load-torque feedback term used by openbmp-propulsion::tvc, and the deterministic slowly varying TWD-like angular-acceleration disturbance series used by the dense-INCA recovery test"
  script: "manual one-off local calculation using the equations recorded in docs/parity/06-gnc-coupled-mimo-and-control.md and the deterministic toy series encoded in the TOML table"
verification:
  method: "parse the TOML table and compare OpenBMP TVC actuator plant step outputs, TWD reaction torque, and dense-INCA compensated/uncompensated residual RMS against the recorded expected values"
  test: "cargo test -p openbmp-propulsion tvc_actuator --locked && cargo test -p openbmp-fc --features indi dense_inca --locked"
  tolerance: "absolute TVC angle/rate/reaction and dense-INCA RMS tolerances plus maximum compensated-RMS ratio recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "The fixture validates toy actuator and controller numerics only. It is not a real vehicle actuator calibration, flight-control I-load, mission profile, or certified TWD compensation."
```

```yaml
dataset_id: openbmp.gnc.robust_controller_siso_margin.v1
files:
  - data/gnc/robust-controller-siso-margin-tolerance-v1.toml
source_class: derived-open-method
source_title: "Imported robust-controller SISO frequency-response margin tolerance table"
license_or_terms: "OpenBMP generated numeric regression fixture; no third-party source code or proprietary vehicle data."
retrieved_utc: "2026-06-28"
source_hash_sha256: "not applicable: generated fixture; compact numeric plant/controller schedule and thresholds are recorded in the TOML table"
transformation:
  method: "record a synthetic stable first-order discrete plant and a two-node reduced-order robust-controller schedule, then evaluate classical SISO open-loop gain/phase margins over nodes and an interior schedule point on a deterministic frequency grid"
  script: "manual one-off local calculation using the equations recorded in docs/parity/06-gnc-coupled-mimo-and-control.md and implemented in openbmp-fc::schedule"
verification:
  method: "parse the TOML table and compare OpenBMP sweep_imported_robust_controller_siso_margins output against the recorded minimum gain and phase margin thresholds"
  test: "cargo test -p openbmp-fc robust_controller_siso --locked"
  tolerance: "minimum gain/phase margin thresholds recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "The fixture validates a public/toy SISO margin tripwire only. It is not MIMO H-infinity/mu evidence, a real vehicle robust-control synthesis, or a flight-control I-load qualification artifact."
```
