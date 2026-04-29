# Provenance — `data/sensors/`

Canonical OpenBMP provenance record for the synthetic IMU noise
budgets shipped under `data/sensors/`. See
[`docs/data-provenance.md`](../../docs/data-provenance.md) for the
contract this record satisfies.

## `data/sensors/imu-tactical.toml`

```yaml
dataset_id:       openbmp.sensor.imu_noise_budget.tactical.v1
files:
  - data/sensors/imu-tactical.toml
source_class:     synthetic-openbmp
source_title:     >-
  Synthetic OpenBMP-authored tactical-grade IMU noise envelope
  parameterised in the IEEE 952-2020 five-component decomposition
  (quantisation, ARW/VRW, bias instability, RRW, scale factor).
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic; not derived from any vendor data sheet
source_url:       https://github.com/openbmp/openbmp/blob/main/data/sensors/imu-tactical.toml
publication_date: 2026-04-27
methodology_reference: >-
  El-Sheimy, N.; Hou, H.; Niu, X. (2008). *Analysis and Modeling
  of Inertial Sensors Using Allan Variance*. IEEE Transactions on
  Instrumentation and Measurement, 57:1, pp. 140–149. The
  five-component decomposition (quantisation, ARW, bias instability,
  RRW, scale factor) and the Allan-variance slope identification
  (-1/2 for ARW, +1/2 for RRW, ~0 plateau for bias instability)
  follow this paper.
methodology_urls:
  - https://ieeexplore.ieee.org/document/4404126
  - https://doi.org/10.1109/TIM.2007.908635
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain. The
  numerical envelope values are illustrative of the tactical-grade
  IMU class (ARW ~ 0.05 °/√h, bias instability ~ 1 °/h with
  τ_BI ~ 100 s) but are NOT transcribed from any vendor data sheet.
source_hash_sha256: 537d60b57650f3d7569b338b6de2de97119b4401f2f244d173330be7aa37454e
retrieved_utc:    2026-04-27
transformation:
  method: >-
    Hand-derived envelope values chosen to match the tactical-grade
    IMU class documented in the El-Sheimy 2008 paper's case-study
    Table III (FOG IMU). The OpenBMP TOML schema reorganises the
    parameters so each IEEE-952 component is a distinct typed field;
    no transcription from any specific vendor's data sheet.
  script: none
verification:
  method: >-
    `crates/openbmp-sensors/tests/regression.rs` loads this file via
    `include_str!` + `ImuNoiseBudget::load_from_str`, confirms every
    schema-1 numerical field parses bit-equally, and runs the
    IEEE-952 Allan-variance slope check on a long synthetic ARW-only
    stream produced by the IMU model.
  test:   crates/openbmp-sensors/tests/regression.rs
  tolerance: >-
    Schema-1 round-trip: bit equality on all parsed numerical fields.
    Allan-variance slope (ARW-only): -0.5 ± 0.1 over the
    1 ms – 100 ms decade on the reference platform profile.
validation_status: validated-toy
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic OpenBMP-authored content. Not a substitute for a
    vendor-specific noise budget calibrated against measured
    Allan-variance data; no operational sensor parameters.
```

## `data/sensors/imu-consumer-mems.toml`

```yaml
dataset_id:       openbmp.sensor.imu_noise_budget.consumer_mems.v1
files:
  - data/sensors/imu-consumer-mems.toml
source_class:     synthetic-openbmp
source_title:     >-
  Synthetic OpenBMP-authored consumer-MEMS IMU noise envelope.
  Two orders of magnitude noisier than the tactical-grade budget
  on both gyro and accel channels, with a much shorter
  bias-instability correlation time and 16-bit-class quantisation.
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic; not derived from any vendor data sheet
source_url:       https://github.com/openbmp/openbmp/blob/main/data/sensors/imu-consumer-mems.toml
publication_date: 2026-04-27
methodology_reference: >-
  Hou, H. (2004). *Modeling Inertial Sensor Errors Using Allan
  Variance*. M.Sc. thesis, University of Calgary. The MEMS-class
  noise envelope and quantisation modelling follow this thesis.
methodology_urls:
  - https://www.ucalgary.ca/engo_webdocs/NES/04.20201.HaiyingHou.pdf
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain.
source_hash_sha256: 3e356196b41a4e765de0f314d8ce5758c750b10e81f25648ce4b16686fb0402f
retrieved_utc:    2026-04-27
transformation:
  method: >-
    Hand-derived envelope values chosen to match the consumer-MEMS
    IMU class documented in the Hou 2004 thesis. The OpenBMP TOML
    schema reorganises the parameters per the IEEE 952-2020
    five-component decomposition.
  script: none
verification:
  method: >-
    Same Phase-2.7.D regression test as `imu-tactical`. The schema
    parses bit-equally across all numerical fields, and the
    Allan-variance slope check passes for an ARW-only IMU stream.
  test:   crates/openbmp-sensors/tests/regression.rs
  tolerance: >-
    Same as `imu-tactical`.
validation_status: validated-toy
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic OpenBMP-authored content. Not a substitute for a
    vendor-specific noise budget. Values illustrative of the
    consumer-MEMS class only.
```

## Why synthetic-openbmp, not public-corpus

There is no canonical public IMU noise-budget corpus in the same
sense that ThrustCurve.org provides for solid motors. Vendor data
sheets are typically copyrighted and often subject to redistribution
restrictions; published academic noise budgets (El-Sheimy 2008,
Hou 2004) describe the IEEE 952 methodology and provide
illustrative case-study values but are not themselves a
machine-readable corpus.

OpenBMP ships **synthetic** noise budgets that match the published
academic envelope of each IMU class (tactical, consumer-MEMS) so
the synthetic IMU model can be exercised end-to-end without
depending on or transcribing any specific vendor's calibration.

## Why the IEEE 952 five-component model

IEEE Std 952-2020 (revision of IEEE Std 952-1997) is the canonical
inertial-sensor noise terminology and Allan-variance decomposition
standard. The five components OpenBMP models per axis —
quantisation, ARW (or VRW for accel), bias instability, RRW
(or ARW for accel), scale factor — are exactly the components
that combine to produce the characteristic Allan-variance log-log
shape (–1/2 slope at short τ from ARW, plateau at intermediate τ
from bias instability, +1/2 slope at long τ from RRW).

The Phase-2.7.D regression test exercises this by running a
synthetic ARW-only stream and asserting the Allan deviation log-log
slope is near –1/2 over the expected averaging-time decade.

## `data/sensors/gnss-textbook.toml`

```yaml
dataset_id:       openbmp.sensors.gnss_textbook.v1
files:
  - data/sensors/gnss-textbook.toml
source_class:     synthetic-openbmp
source_title:     >-
  Synthetic OpenBMP-authored GNSS receiver textbook noise envelope
  parameterised at the receiver-output level (ECI position +
  velocity Gaussian noise, plus an Ornstein-Uhlenbeck position-
  bias drift).
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic; not transcribed from any vendor data sheet.
publication_date: 2026-04-29
methodology_reference: >-
  IS-GPS-200 (NAVSTAR Global Positioning System Interface
  Specification) — public US Government work — for the user-
  equivalent range error (UERE) decomposition that the receiver-
  output Gaussian + OU bias model approximates. The Phase-3.10
  scope is intentionally a *receiver-output* model; satellite
  geometry, pseudorange, ionosphere, and multipath are downstream-
  user concerns.
methodology_urls:
  - https://www.gps.gov/technical/icwg/IS-GPS-200N.pdf
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain. The
  numerical envelope values (σ_pos = 3 m, σ_vel = 0.1 m/s,
  60-second OU correlation, 0.5 m/√s OU drive, ~2.74 m steady-
  state bias stddev) are illustrative of the IS-GPS-200 nominal
  UERE bias envelope but are NOT transcribed from any vendor data
  sheet.
retrieved_utc:    2026-04-29
transformation:
  method: >-
    Hand-derived envelope chosen to fall inside the IS-GPS-200
    nominal UERE budget at the receiver output. The OpenBMP TOML
    schema parameterises the OU bias drift by `θ` (mean-reversion
    rate, 1/s) and `σ` (white-noise drive strength, m/√s), the
    same parameterisation used by the Phase-2.7 IMU bias-
    instability primitive.
verification:
  method: >-
    Phase-3.10.B unit tests
    (`crates/openbmp-sensors/src/gnss.rs::tests`) assert truth-
    bypass exact equality, byte-stable replay across two reruns,
    per-component RNG independence, and an empirical-stddev
    convergence within 5 % of the σ_pos = 3 m budget over a
    10 000-step run.
```

### Phase-3.10 known limitations

- **Receiver output only.** No pseudorange, no satellite geometry,
  no ionospheric / tropospheric / multipath modelling.
- **Single constellation.** GPS only; GLONASS / Galileo / BeiDou
  are out of scope for Phase 3.10.
- **No fault models.** Dropout windows, stuck-output faults, and
  noise-spike faults are Phase-4 controller-side concerns.
- **No real fielded data.** Per `safety-boundaries.md`, the budget
  is a textbook envelope, not a port of any vendor's data sheet.

## `data/sensors/magnetometer-textbook.toml`

```yaml
dataset_id:       openbmp.sensors.magnetometer_textbook.v1
files:
  - data/sensors/magnetometer-textbook.toml
source_class:     synthetic-openbmp
source_title:     >-
  Synthetic OpenBMP-authored magnetometer noise envelope
  parameterised at the body-frame measurement level: per-axis
  Gaussian noise plus constant soft-iron (3x3) and hard-iron
  (3-vector) biases. The truth body-frame field is the
  WMM 2025 NED-truth field rotated through the truth attitude;
  the magnetometer applies its biases and noise on top.
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic; not transcribed from any vendor data sheet.
publication_date: 2026-04-29
methodology_reference: >-
  Crassidis, J. L., Markley, F. L., Cheng, Y. *Survey of
  Nonlinear Attitude Estimation Methods*. JGCD 30:1 (2007),
  §3.2 — covers the noise budgets used in academic attitude-
  determination case studies including the magnetometer
  triaxial Gaussian + hard/soft iron biases that this budget
  approximates.
methodology_urls:
  - https://doi.org/10.2514/1.22452
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain.
  The numerical envelope (50 nT per-axis stddev, ±100 nT hard
  iron, near-identity soft iron with 0.5 % off-diagonal mix)
  is illustrative of low-cost MEMS magnetometers; values are
  NOT transcribed from any vendor's data sheet.
retrieved_utc:    2026-04-29
transformation:
  method: >-
    Hand-derived envelope chosen to match the academic-class
    magnetometer noise documented in Crassidis 2007. The TOML
    schema separates the three noise channels (Gaussian σ,
    soft-iron 3×3, hard-iron 3-vector) so the OpenBMP
    `MagnetometerNoiseBudget` constructor can validate each
    independently.
verification:
  method: >-
    Phase-3.10.C unit tests
    (`crates/openbmp-sensors/src/magnetometer.rs::tests`)
    assert truth-bypass exact equality, hard-iron-only constant
    offset, soft-iron 2× scaling, byte-stable replay across two
    reruns, per-component RNG independence, and an empirical
    stddev convergence within 5 % of σ = 50 nT over a 10 000-
    step run.
```

### Phase-3.10 magnetometer known limitations

- **Body-frame truth supplied by the runner.** The Phase-3.10.C
  magnetometer reads `SensorTruth.magnetic_field_body_nt` — a
  pre-rotated truth field. The runner-side adapter (Phase
  3.10.E) computes the WMM 2025 geodetic-NED field at the
  vehicle's position and time, rotates it through the truth
  attitude, and packs the result. The magnetometer itself is
  frame-agnostic.
- **Constant soft / hard iron only.** No temperature drift, no
  spin-induced bias, no full hysteresis. Phase-3.10 keeps the
  biases as compile-time constants from the budget.
- **Isotropic Gaussian noise.** No correlated noise, no 1/f
  spectrum. The architecture lists OU-style bias drift on
  magnetometers as a future-phase extension.
- **No fault models.** Stuck-axis, dropout, noise-spike faults
  are Phase-4 controller-side concerns.

## `data/sensors/star-tracker-textbook.toml`

```yaml
dataset_id:       openbmp.sensors.star_tracker_textbook.v1
files:
  - data/sensors/star-tracker-textbook.toml
source_class:     synthetic-openbmp
source_title:     >-
  Synthetic OpenBMP-authored star-tracker noise envelope
  parameterised as a single per-axis Gaussian stddev on the
  rotation-vector perturbation. Small-angle quaternion form
  q ≈ (1, θ/2) per axis.
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic; not transcribed from any vendor data sheet.
publication_date: 2026-04-29
methodology_reference: >-
  Crassidis, J. L., Markley, F. L., Cheng, Y. *Survey of
  Nonlinear Attitude Estimation Methods*. JGCD 30:1 (2007),
  §3.2 — covers the per-axis Gaussian rotation-vector
  perturbation noise budget OpenBMP uses here.
  Liebe, C. C. *Accuracy Performance of Star Trackers*.
  JGCD 18:5 (1995) — historical reference for arc-second-level
  noise budgets in fielded star trackers.
methodology_urls:
  - https://doi.org/10.2514/1.22452
  - https://doi.org/10.2514/3.21454
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain.
  The numerical envelope (5 arc-second per-axis stddev,
  ≈ 24 µrad) is the Crassidis 2007 textbook value used in
  academic attitude-estimation case studies. NOT transcribed
  from any vendor's data sheet.
retrieved_utc:    2026-04-29
transformation:
  method: >-
    Hand-derived envelope chosen to match the Crassidis 2007
    arc-second-level noise budget. The OpenBMP TOML schema
    accepts the human-friendly arc-second form
    (`sigma_per_axis_arcsec`); the runner converts to radians
    via `ARCSEC_TO_RAD = π / 648 000` at scenario load.
verification:
  method: >-
    Phase-3.10.D unit tests
    (`crates/openbmp-sensors/src/star_tracker.rs::tests`)
    assert truth-bypass exact equality, unit-norm
    quaternion output for any seed (property test over 1 000
    steps), byte-stable replay across two reruns, and an
    empirical stddev convergence within 5 % of σ = 50″ over
    a 10 000-step run. The constructor rejects budgets with
    σ > 0.01 rad to keep the small-angle approximation valid.
```

### Phase-3.10 star-tracker known limitations

- **Small-angle approximation only.** The constructor rejects
  σ > 0.01 rad (~34′). For wider noise budgets the audit can
  promote the model to the full quaternion-exponential form
  (sin / cos of half-angle).
- **Isotropic noise.** No per-axis variation, no correlated
  noise, no bias drift. The architecture lists per-axis variance
  and quaternion-bias drift as future-phase extensions.
- **No occlusion / slew-rate / bright-object models.** The
  star tracker is always "tracking" in Phase 3.10. Fault models
  for boresight occlusion and slew-rate dropout are Phase-4
  controller-side concerns.
- **No fault models.** Stuck-attitude, dropout, noise-spike
  faults are Phase-4 controller-side concerns.
