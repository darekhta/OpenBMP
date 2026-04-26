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
  (~16.8 N·s, ~1.4 s burn) but the values are NOT transcribed
  from any published thrust curve.
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic; not derived from any fielded motor
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

## Pending real-source pin

The Phase 2 plan calls for a `data/motors/estes-d12.eng-derived.toml`
deck transcribed from ThrustCurve.org's public Estes D12 `.eng` file
with `validation = "manufacturer"` and a SHA-256 pin on the source
`.eng` file. This is **deferred** to Phase 2.9 (sounding-rocket
validation case) where the actual `.eng` file can be fetched and the
SHA-256 pinned. The synthetic D-class motor above sits in for the
integration scenarios in the meantime; the kernel-side wiring is
identical (the deck shape is the same), so swapping in the real
Estes D12 deck is a one-file change once the source is fetched.

Tracking gap:
  pending_real_source:
    file:           data/motors/estes-d12.eng-derived.toml
    source_url:     https://www.thrustcurve.org/motors/Estes/D12/
    expected_sha256: <to be filled when fetched>
    target_phase:   2.9 (sounding-rocket validation)

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
documents this policy. The two motor decks shipped here both
carry `source_class: synthetic-openbmp`; the Phase 2.9 D12 deck
will carry `source_class: public-corpus` with the SHA pin.
