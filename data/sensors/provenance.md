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
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain. The
  numerical envelope values are illustrative of the tactical-grade
  IMU class (ARW ~ 0.05 °/√h, bias instability ~ 1 °/h with
  τ_BI ~ 100 s) but are NOT transcribed from any vendor data sheet.
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
    `include_str!` + `ImuNoiseBudget::load_from_str`, confirms the
    schema-1 fields parse, and runs the IEEE-952 Allan-variance
    slope check on a long synthetic ARW-only stream produced by the
    IMU model with this budget's gyro ARW value.
  test:   crates/openbmp-sensors/tests/regression.rs
  tolerance: >-
    Schema-1 round-trip: bit equality on all parsed numerical fields.
    Allan-variance slope (ARW-only): -0.5 ± 0.05 over the
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
publication_date: 2026-04-27
methodology_reference: >-
  Hou, H. (2004). *Modeling Inertial Sensor Errors Using Allan
  Variance*. M.Sc. thesis, University of Calgary. The MEMS-class
  noise envelope and quantisation modelling follow this thesis.
methodology_urls:
  - https://www.ucalgary.ca/engo_webdocs/NES/04.20201.HaiyingHou.pdf
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain.
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
    parses, the Allan-variance slope check passes for an ARW-only
    sub-budget derived from this file's gyro ARW value.
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
