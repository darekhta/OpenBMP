# Provenance — `data/scenarios/`

Canonical OpenBMP provenance record for the benchmark scenario
files shipped under `data/scenarios/`. See
[`docs/data-provenance.md`](../../docs/data-provenance.md) for the
contract this record satisfies.

## `data/scenarios/niskanen-2009-chapter6.toml`

```yaml
dataset_id:       openbmp.scenario.niskanen_2009_chapter6.v1
files:
  - data/scenarios/niskanen-2009-chapter6.toml
source_class:     converted-public
source_title:     >-
  Niskanen 2009 Chapter-6 small-rocket benchmark scenario pin.
  Vehicle geometry (56 cm body length × 29 mm diameter, 10 cm
  nose), 80 g dry-airframe mass, constant-CD axial reduction
  (CD = 0.8), reference apogees from the published Chapter-6
  comparison table for both B4-class and C6-class motors, and a
  ±5% relative tolerance for the OpenBMP integration test.
source_authors:   Niskanen, S. (2009 thesis); OpenBMP (Dmitri Arekhta) for the OpenBMP-schema rendering
source_id:        Niskanen 2009 Chapter 6 small-rocket benchmark
source_url:       https://openrocket.sourceforge.net/thesis.pdf
publication_date: 2009-05-20  # Niskanen thesis publication
methodology_reference: >-
  Niskanen, S. (2009). *Development of an Open-Source Model Rocket
  Simulation Software*. Master's thesis, Helsinki University of
  Technology. Chapter 6 documents a small-rocket benchmark
  comparing OpenRocket and RockSim simulated apogees against
  flight experiments for both Estes B4 and C6 motors.
methodology_urls:
  - https://openrocket.sourceforge.net/thesis.pdf
license_or_terms: >-
  The Niskanen thesis is distributed under terms permitting
  academic re-use with attribution. The OpenBMP scenario file is a
  schema rendering of the published geometry, mass, and reference
  apogees; no derivative works restrictions apply to the numerical
  facts themselves. OpenBMP credits Niskanen as the source.
retrieved_utc:    2026-04-27
transformation:
  method: >-
    Vehicle geometry, dry mass, axial-CD assumption, and reference
    apogees transcribed from the published Chapter-6 table into
    the OpenBMP minimal-benchmark TOML schema. The relative-path
    references to the C6 motor and Niskanen-Ch6 aero deck under
    the `[references]` section are documentation; the Phase-2.9
    integration test loads those files directly via `include_str!`
    rather than via a scenario-runner path resolution. Phase 2.10
    will add the scenario-runner that consumes this file
    end-to-end.
  script: none
verification:
  method: >-
    `crates/openbmp-vehicle/tests/sounding_rocket.rs` loads this
    file via `include_str!` + a small inline serde deserialiser,
    and asserts:
      - vehicle.body_length_m = 0.56, body_diameter_m = 0.029,
        nose_length_m = 0.10, dry_vehicle_mass_kg = 0.080,
        axial_cd = 0.8;
      - reference_apogees_m.{b4,c6}_{experimental,openrocket,rocksim}
        bit-exactly match the Chapter-6 published values;
      - tolerance.apogee_relative = 0.05;
    and uses these values to drive the C6 benchmark integration
    test, which asserts the simulated apogee within ±5% of the
    published experimental C6 apogee (151.5 m).
  test:   crates/openbmp-vehicle/tests/sounding_rocket.rs
  tolerance: >-
    Bit equality on every parsed numerical field. Apogee
    comparison: ±5% relative against the published experimental
    C6 value.
validation_status: validated-toy
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public academic benchmark from a master's thesis. The vehicle
    parameters describe a small hobby-class model rocket; no
    operational mission profile, no restricted technical data,
    no fielded-vehicle calibration.
```

## Why benchmarks are pinned in code AND data

The Phase-2.9 integration test loads the scenario file's numerical
fields and asserts each one bit-exactly. This is the same pattern
used for the WGS84 J2 constant, the USSA76 layer table, the aero
deck values, and the IMU noise budgets — every shipped data file
has its values pinned by both the file (the canonical truth) and a
regression test (the runtime check that the value is consumed
correctly). A typo in either surface breaks CI before release.

The `[references]` section's relative paths are informational for
Phase 2.9. Phase 2.10 will introduce an `openbmp` CLI scenario
runner that resolves those paths, computes their content
SHA-256s, and records the hashes in the run header so a downstream
consumer can prove which motor and aero deck were active when a
given trace was produced.
