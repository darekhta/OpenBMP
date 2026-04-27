# Provenance — `data/motors/`

Canonical OpenBMP provenance record for the synthetic solid motors
shipped under `data/motors/`. See
[`docs/data-provenance.md`](../../docs/data-provenance.md) for the
contract this record satisfies.

## `data/motors/synthetic-solid-textbook.toml`

```yaml
dataset_id:       openbmp.motor.synthetic_solid_textbook.v1
files:
  - data/motors/synthetic-solid-textbook.toml
source_class:     synthetic-openbmp
source_title:     >-
  Synthetic OpenBMP-authored trapezoidal-thrust solid motor.
  4 s burn, 3500 N·s total impulse, 1000 N flat-top thrust.
  Designed as the canonical small-deck unit-test fixture.
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic; not derived from any fielded motor
source_url:       https://github.com/openbmp/openbmp/blob/main/data/motors/synthetic-solid-textbook.toml
source_hash_sha256: 1b8b9c6f299617a7bbee2b1e54ce097e486cec66b6a42a190e04d12337dbce7d
publication_date: 2026-04-27
methodology_reference: >-
  RASP `.eng` thrust-curve format (ThrustCurve.org public corpus
  conventions). The OpenBMP motor format is a TOML re-implementation
  of the same `(t_s, thrust_n)` piecewise-linear interpolation.
methodology_urls:
  - https://www.thrustcurve.org/info/raspformat.html
  - https://openrocket.info/
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain. No
  third-party data is incorporated.
retrieved_utc:    2026-04-27
transformation:
  method: >-
    Hand-derived trapezoidal thrust profile chosen so the
    closed-form trapezoidal integral matches the declared
    `total_impulse_n_s = 3500` exactly:
      0.5·1000·0.5  +  1000·3.0  +  0.5·1000·0.5  =  3500 N·s
    No transcription from a published motor.
  script: none
verification:
  method: >-
    `crates/openbmp-propulsion/tests/regression.rs` loads this file
    via `include_str!` + `SolidMotor::load_from_str`, asserts the
    deck's integrated impulse matches the declared
    `total_impulse_n_s` to 1e-12 relative, asserts mass at burnout
    equals dry mass exactly (impulse-weighted mass model), asserts
    `mass_rate ≤ 0` everywhere on the burn window, and verifies
    bit-stable lookups across two evaluations.
  test:   crates/openbmp-propulsion/tests/regression.rs
  tolerance: >-
    Integrated impulse vs declared total: 1e-12 relative.
    Mass at burnout: bit-equality with dry_mass.
    Mass-rate sign: ≤ 0 everywhere.
    Bit-stability across two lookups: bit equality on the
    reference platform profile.
validation_status: validated-toy
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic OpenBMP-authored content; no fielded-motor data,
    no manufacturer thrust curve. Suitable for analytic-toy and
    validation-toy scenarios; not a substitute for a measured
    static-fire thrust curve.
```

## `data/motors/synthetic-solid-d-class.toml`

```yaml
dataset_id:       openbmp.motor.synthetic_solid_d_class.v1
files:
  - data/motors/synthetic-solid-d-class.toml
source_class:     synthetic-openbmp
source_title:     >-
  Synthetic OpenBMP-authored boost-sustain solid motor sized
  to the D-class envelope (10–20 N·s total impulse, sub-2 s
  burn). Nominal envelope tracks the Estes D12 hobby motor
  (~16.8 N·s, ~1.7 s burn, ~10 N average thrust, ~30 N peak
  thrust) but the values are NOT transcribed from any published
  thrust curve.
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic; not derived from any fielded motor
source_url:       https://github.com/openbmp/openbmp/blob/main/data/motors/synthetic-solid-d-class.toml
source_hash_sha256: 76eed36d6fded4a727c1766a2d8e83da09526a3d3138df59f33348f26bbafc0c
publication_date: 2026-04-27
methodology_reference: >-
  RASP `.eng` thrust-curve format and the boost-sustain profile
  conventions documented by the ThrustCurve.org corpus.
methodology_urls:
  - https://www.thrustcurve.org/info/raspformat.html
  - https://www.thrustcurve.org/motors/Estes/D12/
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain. No
  third-party data is incorporated.
retrieved_utc:    2026-04-27
transformation:
  method: >-
    Hand-derived boost-sustain thrust profile (peak 30 N at
    t = 0.05 s, sustain at 10 N from 0.15 s to 1.35 s,
    burnout at 1.4 s) chosen so the closed-form trapezoidal
    integral matches the declared `total_impulse_n_s = 15.0`
    exactly:
      0.5·30·0.05 + 0.5·(30+10)·0.10 + 10·1.20 + 0.5·10·0.05
      =      0.75 +              2.0 +    12.0 +         0.25
      = 15.0 N·s
    No transcription from a published motor.
  script: none
verification:
  method: >-
    Same Phase-2.6 regression test as `synthetic-solid-textbook`
    but parameterised over both shipped motors. Asserts integrated
    impulse vs declared total to 1e-12 relative, mass at burnout
    equals dry mass exactly, mass-rate ≤ 0, and bit-stable lookups.
  test:   crates/openbmp-propulsion/tests/regression.rs
  tolerance: >-
    Same as `synthetic-solid-textbook`.
validation_status: validated-toy
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic OpenBMP-authored content sized to the D-class
    envelope. Not a transcription of any published motor curve.
    Suitable for sounding-rocket validation scenarios where a
    canonical small-motor profile is needed without disclosing or
    transcribing manufacturer data.
```

## `data/motors/estes-d12-eng-derived.toml`

```yaml
dataset_id:       openbmp.motor.estes_d12.v1
files:
  - data/motors/estes-d12-eng-derived.toml
source_class:     converted-public
source_title:     >-
  Estes D12 solid-propellant model rocket motor, 24 mm × 70 mm,
  16.8 N·s total impulse, 1.7 s burn (manufacturer spec); the
  published RASP `.eng` thrust curve is transcribed verbatim into
  the OpenBMP motor TOML schema with a (0, 0) starting point
  prepended to satisfy the OpenBMP requirement that
  `points[0].time = 0` exactly.
source_authors:   John Coker (RASP file), Estes Industries (motor)
source_id:        ThrustCurve.org Estes D12 RASP simfile
source_url:       https://www.thrustcurve.org/simfiles/5f4294d20002e900000004ea/download/Estes_D12.eng
source_hash_sha256: a39e5118880b08261650905901772f9019bc60d25dbaeefb593f4fbe529c4f05
publication_date: 1994-09-17  # NAR certification date
methodology_reference: >-
  RASP `.eng` thrust-curve format and the manufacturer's
  static-fire calibration. NAR-certified per the National
  Association of Rocketry's standardised motor-test procedure.
methodology_urls:
  - https://www.thrustcurve.org/info/raspformat.html
  - https://www.nar.org/SandT/pdf/Estes/D12.pdf
license_or_terms: >-
  ThrustCurve.org corpus is distributed under terms that permit
  re-use with attribution and no warranty. The transcribed numerical
  values are physical motor-test data from a NAR-certified motor;
  no derivative works restrictions apply to the data points
  themselves. OpenBMP credits the original RASP file contributor
  (John Coker) and the manufacturer (Estes).
retrieved_utc:    2026-04-27
transformation:
  method: >-
    Verbatim transcription of the 20 (time_s, thrust_N) pairs from
    the RASP `.eng` file at the `source_url` above, with a (0.0, 0.0)
    starting point prepended. The trapezoidal integral of the
    transcribed curve evaluates to 16.8391395 N·s in f64; the
    `burn.total_impulse_n_s` field declares this exact value so
    the Phase-2.6 motor parser's tight-tolerance integral check
    passes. The `burn.duration_s = 1.65` field matches the curve's
    last time exactly (the published manufacturer spec rounds this
    to 1.7 s for the data sheet). The
    `specific_impulse_s = 81.3798273021187` field is back-solved
    from `I = m_p · g_0 · Isp` with the declared propellant mass and
    `g_0 = 9.80665` m/s² so the Phase-2.6 Isp consistency check
    passes within 1e-3 relative.
  script: none
verification:
  method: >-
    `crates/openbmp-vehicle/tests/sounding_rocket.rs` loads the
    deck via `include_str!` + `SolidMotor::load_from_str`, asserts
    `burn.duration_s = 1.65`, `total_impulse_n_s = 16.8391395`,
    `propellant_mass_kg = 0.0211`, and runs the deck through the
    Phase-2.9 D12 sounding-rocket integration test (vertical-launch,
    USSA76 atmosphere, synthetic D12-class aero deck). The integration
    test asserts a tight physical sanity envelope and pins final-state
    and headline-metric bit patterns for replay drift detection.
  test:   crates/openbmp-vehicle/tests/sounding_rocket.rs
  tolerance: >-
    burn.duration_s and total_impulse_n_s: bit equality after parse.
    Mass at burnout: bit-equality with dry_mass.
    Mass-rate sign: ≤ 0 everywhere on the burn window.
    Bit-stability across two lookups: bit equality on the reference
    platform profile.
validation_status: validated-toy
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public motor data from a hobby motor (Estes D12) certified by
    the NAR. No export control or manufacturer-proprietary
    restrictions; no operational vehicle parameters.
```

### Source-file SHA-256 pin

The D12 source file is pinned by the `source_hash_sha256` value above.
It is the SHA-256 of the bytes downloaded from ThrustCurve.org at
`/simfiles/5f4294d20002e900000004ea/download/Estes_D12.eng` on
2026-04-27.

## `data/motors/estes-c6-eng-derived.toml`

```yaml
dataset_id:       openbmp.motor.estes_c6.v1
files:
  - data/motors/estes-c6-eng-derived.toml
source_class:     converted-public
source_title:     >-
  Estes C6 solid-propellant model rocket motor, 18 mm × 70 mm,
  10 N·s nominal total impulse (manufacturer spec); the published
  RASP `.eng` thrust-curve shape is used as the temporal envelope
  with the magnitudes scaled to the 7.5 N·s C6-family total
  impulse cited by Niskanen 2009 Chapter 6, which is the reference
  total impulse the Phase-2.9 Niskanen Chapter-6 benchmark
  integrates against. The (0, 0) starting point is prepended to
  satisfy the OpenBMP requirement that `points[0].time = 0`
  exactly.
source_authors:   John Coker (RASP file), Estes Industries (motor)
source_id:        ThrustCurve.org Estes C6 RASP simfile
source_url:       https://www.thrustcurve.org/simfiles/5f4294d20002e900000004e7/download/Estes_C6.eng
source_hash_sha256: 90fa89edab96583266994ade8a8afd5b30a169f3a73d6db3a379495f24033570
publication_date: 1994-09-17  # NAR certification date for the C6
methodology_reference: >-
  RASP `.eng` thrust-curve format and the manufacturer's
  static-fire calibration; impulse rescaling per Niskanen 2009
  Chapter 6 reference value.
methodology_urls:
  - https://www.thrustcurve.org/info/raspformat.html
  - https://openrocket.sourceforge.net/thesis.pdf
license_or_terms: >-
  ThrustCurve.org corpus is distributed under terms that permit
  re-use with attribution and no warranty. The transcribed shape
  values are physical motor-test data from a NAR-certified motor;
  the magnitude rescaling is documented in this provenance entry.
retrieved_utc:    2026-04-27
transformation:
  method: >-
    Verbatim transcription of the (time_s, thrust_N) shape from
    the RASP `.eng` file at the `source_url` above, with thrust
    magnitudes scaled to the 7.5 N·s C6-family total impulse cited
    by Niskanen 2009 Chapter 6 and a (0.0, 0.0) starting point
    prepended. The upstream trapezoidal integral is 8.817238 N·s;
    OpenBMP scales every upstream thrust value by
    0.8506065051209915 (= 7.5 / 8.817238). The committed decimal
    values integrate to 7.4999999999999485 N·s in f64, which matches
    the declared 7.5 N·s within the Phase-2.6 motor parser's
    tight-tolerance integral check. `burn.duration_s = 1.86` matches
    the curve's last time. `specific_impulse_s` back-solved from
    `I = m_p · g_0 · Isp` for `g_0 = 9.80665 m/s²` so the
    Phase-2.6 Isp consistency check passes within 1e-3 relative.
  script: none
verification:
  method: >-
    `crates/openbmp-vehicle/tests/sounding_rocket.rs` loads the
    deck via `include_str!` + `SolidMotor::load_from_str`, asserts
    `total_impulse_n_s = 7.5` and `burn_duration_s = 1.86`, and
    runs the deck through the Phase-2.9 Niskanen Chapter-6
    benchmark integration test. The benchmark asserts the
    simulated C6 apogee falls within ±5% of Niskanen's published
    experimental C6 value (151.5 m).
  test:   crates/openbmp-vehicle/tests/sounding_rocket.rs
  tolerance: >-
    `total_impulse_n_s` and `burn_duration_s`: bit equality after
    parse. Apogee comparison: ±5% relative against Niskanen 2009
    Chapter 6 experimental C6 value.
validation_status: validated-toy
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public motor curve from a NAR-certified hobby motor (Estes C6)
    rescaled to a published academic reference impulse. No export
    control or manufacturer-proprietary restrictions; no
    operational vehicle parameters.
```

### Public motor import policy

New third-party motor imports must identify an explicit compatible license
or public-domain statement for the source artifact. ThrustCurve.org license
tags are per-file metadata, not a corpus-wide grant. A source with an absent
or unknown per-file license is rejected until that uncertainty is resolved,
even if the motor class itself is public or NAR-certified.

Imported RASP thrust curves declare the trapezoidal integral of the
committed piecewise-linear points as simulation truth by default. Magnitude
rescaling is allowed only when a scenario-specific validation case cites a
published reference impulse and documents the scale factor, as with the
Niskanen C6 benchmark entry above.

For delay-bearing hobby motors, the OpenBMP filename may omit the delay
suffix when the Phase-2 motor model does not simulate ejection charges or
delays. The upstream RASP designation remains recorded in `source_title`
and file comments.

The `exit_area_m2` field on imported 18 mm Estes motors is currently an
inert Phase-2 placeholder. With `ambient_pressure_correction = "constant"`,
OpenBMP uses the thrust curve as-given. Unless a future pressure correction
model consumes the value, it is documented as copied from the existing 18 mm
C6 convention, not as a per-motor measured nozzle area.

The Estes A8 RASP simfile at
`https://www.thrustcurve.org/simfiles/5f4294d20002e90000000897/` was
audited on 2026-04-27 and intentionally not imported: the file hash
reproduced, but ThrustCurve's page/API did not expose a per-file license
tag. Under `docs/data-provenance.md`, unclear license terms are rejected
until resolved.

## `data/motors/estes-b4-eng-derived.toml`

```yaml
dataset_id:       openbmp.motor.estes_b4.v1
files:
  - data/motors/estes-b4-eng-derived.toml
source_class:     converted-public
source_title:     >-
  Estes B4-4 solid-propellant model rocket motor, 18 mm × 70 mm,
  manufacturer-spec total impulse 5.0 N·s; the published RASP `.eng`
  thrust curve is transcribed verbatim into the OpenBMP motor TOML
  schema with a (0, 0) starting point prepended. The upstream file
  is named "B4-4", but the thrust profile is the B4 burn; delay and
  ejection-charge behaviour are outside OpenBMP's Phase-2 motor
  model. The B4 motor is one of the two motors Niskanen 2009
  Chapter 6 Table 6.1 tabulates against experimental, OpenRocket,
  and RockSim apogees for the small-rocket benchmark; importing this
  motor makes a future B4 scenario possible alongside the existing C6
  validation case (Niskanen experimental B4 apogee = 64.0 m).
source_authors:   Nicholas Domansky (RASP file), Estes Industries (motor)
source_id:        ThrustCurve.org Estes B4 RASP simfile
source_url:       https://www.thrustcurve.org/simfiles/5f4294d20002e90000000834/download/data.eng
source_hash_sha256: e36ced570b2fc3baf5006236a311dfc1f10af8d26d2210a5c1c178ba2dbdb5be
publication_date: 2014-12-02  # ThrustCurve simfile submission date
methodology_reference: >-
  RASP `.eng` thrust-curve format. ThrustCurve identifies this
  specific simfile as user-contributed with source tag `user` and
  per-file license tag `PD`. The Estes B4 motor class is
  NAR-certified, but this imported data file is not represented as a
  cert-org source file.
methodology_urls:
  - https://www.thrustcurve.org/simfiles/5f4294d20002e90000000834/
  - https://www.thrustcurve.org/info/contribute.html
  - https://www.thrustcurve.org/info/raspformat.html
  - https://www.thrustcurve.org/motors/Estes/B4/
  - https://openrocket.sourceforge.net/thesis.pdf
license_or_terms: >-
  Public Domain, based on the ThrustCurve per-file license tag `PD`
  for simfile `5f4294d20002e90000000834`. ThrustCurve's contribution
  documentation defines Public Domain as the least restrictive class:
  anyone may do anything with the file. OpenBMP credits the RASP file
  contributor (Nicholas Domansky) and the manufacturer (Estes).
retrieved_utc:    2026-04-27
transformation:
  method: >-
    Verbatim transcription of the 29 (time_s, thrust_N) pairs from
    the RASP `.eng` file at the `source_url` above, with a
    (0.0, 0.0) starting point prepended. Trapezoidal integration of
    the 30-point committed curve evaluates to 5.092238500000001 N·s
    in f64; OpenBMP declares this as the simulation truth (matching
    the D12 precedent of declaring the integrated value rather than
    rescaling magnitudes). The Estes manufacturer spec rounds the
    static-test total impulse to 5.0 N·s; the 1.8% discrepancy is
    typical RASP-transcription noise. No magnitude rescaling is
    applied. The `burn.duration_s = 1.049` field matches the curve's
    last time exactly. The `specific_impulse_s = 86.54396931334008`
    field is back-solved from `I = m_p · g_0 · Isp` with the
    declared propellant mass and `g_0 = 9.80665` m/s² so the
    Phase-2.6 Isp consistency check passes within 1e-3 relative. The
    `exit_area_m2 = 0.0000159` field is copied from the existing
    18 mm Estes C6 convention and is inert while
    `ambient_pressure_correction = "constant"` is selected.
  script: none
verification:
  method: >-
    `crates/openbmp-propulsion/tests/regression.rs` parses this file
    via `SolidMotor::load_from_toml`, asserts the integrated impulse
    matches the declared `total_impulse_n_s` to 1e-12 relative, mass
    at burnout equals dry mass exactly, mass-rate ≤ 0 across the
    burn, and bit-stable lookups across two evaluations. No B4
    end-to-end apogee scenario has been authored yet.
  test:   crates/openbmp-propulsion/tests/regression.rs
  tolerance: >-
    Integrated impulse vs declared total: 1e-12 relative. Mass at
    burnout: bit-equality with dry_mass. Mass-rate sign: ≤ 0
    everywhere. Bit-stability across two lookups: bit equality on
    the reference platform profile.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public-domain user-contributed simfile for the Estes B4 hobby
    motor. The source is an 18 mm model-rocket motor curve and
    cross-references Niskanen 2009 Chapter 6 Table 6.1 experimental
    apogee (64.0 m) only as future validation context. No export
    control or manufacturer-proprietary restrictions; no operational
    vehicle parameters.
```

## Why fielded-motor curves are rejected unless explicitly transcribed

The OpenBMP motor format keeps the deck format open and the data
either synthetic (this Phase 2 default) or transcribed from a
public corpus with explicit provenance (the Phase 2.9 path).
Fielded-motor thrust curves from non-public corpora are explicitly
rejected at the project level: those datasets often carry export-
control or manufacturer-proprietary restrictions that OpenBMP
cannot ship under its CC0 / open-research positioning.

The architecture's
[§ Motor Format (in-house TOML)](../../docs/software-architecture.md#motor-format-in-house-toml)
documents this policy. The synthetic motor decks carry
`source_class: synthetic-openbmp`; the Phase 2.9 D12 deck carries
`source_class: converted-public` with the SHA pin.
