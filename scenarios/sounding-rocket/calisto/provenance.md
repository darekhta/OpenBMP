# Provenance — `scenarios/sounding-rocket/calisto/`

Canonical OpenBMP provenance record for the RocketPy Calisto
cross-tool validation scenario shipped under
`scenarios/sounding-rocket/calisto/`. See
[`docs/data-provenance.md`](../../../docs/data-provenance.md) for
the contract this record satisfies.

## `scenarios/sounding-rocket/calisto/rocketpy-calisto.toml`

```yaml
dataset_id:       openbmp.scenario.sounding_rocket.rocketpy_calisto.v1
files:
  - scenarios/sounding-rocket/calisto/rocketpy-calisto.toml
source_class:     converted-public
source_title:     >-
  Phase-3.11 OpenBMP-schema rendering of the RocketPy Calisto
  example. The scenario combines the Cesaroni Pro75 M1670 motor
  port shipped at `data/motors/cesaroni-m1670.toml`, the
  axisymmetric Calisto drag deck shipped at
  `data/aero/calisto-drag.toml` (sourced from RocketPy's byte-
  identical `powerOff/powerOnDragCurve.csv`), and the published
  Calisto airframe parameters (radius 0.0635 m, dry mass 14.426 kg,
  axisymmetric inertia tensor diag(6.321, 6.321, 0.034) kg·m²,
  drogue + main parachute with cd_s = 1.0 / 10.0). Launch site is
  Spaceport America, NM (32.99° N, 106.97° W, 1 401 m elevation),
  rendered as a `toy-fixed-earth` frame with a 1 400 m altitude
  offset so the USSA76 atmosphere sees Spaceport's geodetic
  elevation.
source_authors:   RocketPy Team (Souza et al. 2021); OpenBMP (Dmitri Arekhta) for the OpenBMP-schema rendering
source_id:        RocketPy Calisto cross-tool validation, OpenBMP rendering
source_url:       https://github.com/RocketPy-Team/RocketPy
publication_date: 2022-12-15
methodology_reference: >-
  Souza, J. M., Mendes, L. P. F., et al. (2021). *RocketPy: Six
  Degree-of-Freedom Rocket Trajectory Simulator*. Journal of
  Aerospace Engineering, 34(6). The Calisto example in the RocketPy
  repository (`docs/notebooks/getting_started.ipynb`,
  `tests/fixtures/rockets/calisto/`) ships the airframe parameters,
  motor selection (Cesaroni Pro75 M1670), drag curves, and recovery
  configuration that this OpenBMP scenario reproduces. RocketPy
  reports a published apogee of approximately 3 349 m AGL for this
  configuration; the Phase-3.11.E end-to-end test pins the value
  with a ±5% envelope (the risk-register fallback for the
  RocketPy LSODA-adaptive vs. OpenBMP RK4-fixed-step integrator
  mismatch documented in `docs/phase-3-plan.md §3.11`).
methodology_urls:
  - https://github.com/RocketPy-Team/RocketPy
  - https://ascelibrary.org/doi/10.1061/%28ASCE%29AS.1943-5525.0001331
license_or_terms: >-
  RocketPy is MIT-licensed (`https://github.com/RocketPy-Team/RocketPy/blob/master/LICENSE`).
  The Calisto airframe parameters, drag curves, and recovery
  configuration in the RocketPy repository fall under that license,
  and OpenBMP redistributes them in transformed form (TOML schema)
  with attribution. The Cesaroni M1670 motor data is sourced from
  ThrustCurve.org (see `data/motors/provenance.md`); ThrustCurve
  motor files are released for unrestricted academic and hobby use.
retrieved_utc:    2026-04-28
transformation:
  method: >-
    Render the RocketPy Calisto example into the OpenBMP scenario
    schema. The scenario references the Cesaroni M1670 motor and
    the Calisto drag deck by relative path with SHA-256 pins. The
    rigid-body initial state encodes the 5° rail tilt as a
    body-to-ECI quaternion (rotation about ECI +y by 5°). Spaceport
    America's 1 400 m elevation is encoded as a +z offset on the
    initial position so the USSA76 atmosphere receives the right
    altitude profile under the toy-fixed-earth frame profile.
    Recovery uses the Phase-3.9 `DrogueMainRecovery` device
    (drogue at apogee event, main at altitude_m = 1867.0, i.e.
    467 m AGL on descent). The 30 s simulation horizon captures
    the apogee with margin (apogee at t ≈ 24 s); the descent
    phase is exercised by the runner but the apogee comparison is
    the only quantitative gate.
  script: none
verification:
  method: >-
    `openbmp check` parses the scenario and resolves the pinned
    aero deck and motor file digests. The Phase-3.11.E e2e test
    `calisto_apogee_within_rocketpy_envelope` asserts the simulated
    apogee AGL (max(z) − 1400 m offset) falls within ±5% of
    RocketPy's published 3 349 m AGL. The companion test
    `calisto_byte_stable_across_two_runs` asserts byte-stable
    Parquet across two reruns (Phase-3 determinism gate end-to-end
    over the rigid-body hot path: scenario parse → motor load →
    drag deck load → aero + thrust force adapters →
    DrogueMainRecovery state machine → RK4 integrator →
    Parquet sink).
  test: >-
    crates/openbmp-cli/tests/calisto_e2e.rs
  tolerance: >-
    Parser/check path requires exact SHA-256 pin matches. Physics
    regression tolerance is the Phase-3.11 risk-register fallback
    envelope of ±5% around the RocketPy 3 349 m AGL reference
    (envelope = 167.45 m). Tightening to the 1% stretch-goal
    envelope is a follow-up that requires either an adaptive RK
    integrator on the OpenBMP side or a fixed-step replay of
    RocketPy at OpenBMP's `dt`.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public open-source rocket simulator example (MIT-licensed
    upstream). The modeled object is a hobby-class research
    sounding rocket (Calisto airframe + Cesaroni Pro75 M1670
    commercial hobby motor). The scenario contains no targeting,
    no real device drivers, and no restricted or fielded-vehicle
    data. Spaceport America is a publicly-published commercial
    launch site.
units:            metres, metres-per-second, metres-per-second-squared, seconds, kilograms, kilogram-metres-squared, dimensionless quaternion
frame_profile:    toy-fixed-earth
local_origin:     >-
  Spaceport America, NM (latitude 32.99° N, longitude 106.97° W,
  geodetic height 1 401 m). The scenario uses a `toy-fixed-earth`
  frame with a 1 400 m altitude offset on initial position so the
  USSA76 atmosphere receives Spaceport's elevation; the toy frame
  drops Earth rotation and Coriolis, which is acceptable for the
  sub-30-second Calisto flight per the Phase-3.11 risk register.
related_files:
  - data/motors/cesaroni-m1670.toml
  - data/motors/Cesaroni_M1670.eng
  - data/aero/calisto-drag.toml
  - data/aero/calisto-drag.csv
  - crates/openbmp-cli/tests/calisto_e2e.rs
notes: >-
  Each external file reference (`aero.deck`, `propulsion.motor.file`)
  carries an `*_sha256` pin that fails closed on mismatch. The pin
  values match the data files at the commit shipping this
  scenario; if a referenced file is updated, recompute the digest
  with `shasum -a 256 <file>` and update the pin.
```
