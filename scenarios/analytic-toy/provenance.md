# Provenance — `scenarios/analytic-toy/`

Canonical OpenBMP provenance record for the analytic-toy
scenarios shipped under `scenarios/analytic-toy/`. See
[`docs/data-provenance.md`](../../docs/data-provenance.md) for the
contract this record satisfies.

## `scenarios/analytic-toy/constant-acceleration-drop.toml`

```yaml
dataset_id:       openbmp.scenario.analytic_toy.constant_acceleration_drop.v1
files:
  - scenarios/analytic-toy/constant-acceleration-drop.toml
source_class:     synthetic-openbmp
source_title:     >-
  Closed-form analytic-toy validation scenario: a 1 kg
  point mass at z = 1000 m, zero initial velocity, falling
  under constant gravity magnitude g = 9.80665 m/s² along -z for
  10 s. The closed-form solution z(t) = 1000 - 0.5 g t² and
  vz(t) = -g t is reproduced exactly by the kernel's RK4
  fixed-step integrator for a constant force on a constant mass.
source_authors:   OpenBMP project
source_id:        analytic-toy reference scenario
source_url:       (in-house)
publication_date: 2026-01-01
methodology_reference: >-
  Synthetic scenario derived from textbook constant-acceleration
  kinematics. The g value 9.80665 m/s² is the BIPM-defined
  standard gravity (CGPM 1901, codified in ISO 80000-3:2019);
  used here as the canonical drop acceleration for the analytic
  comparison, not as a localised physical constant.
methodology_urls:
  - https://www.bipm.org/en/publications/si-brochure
license_or_terms: >-
  Synthetic OpenBMP-authored scenario; no third-party
  restrictions. Released under the same dual MIT/Apache-2.0
  licensing as the rest of the workspace.
retrieved_utc:    2026-01-01
transformation:
  method: >-
    OpenBMP-authored closed-form constant-acceleration scenario
    written directly in the canonical scenario schema. No external
    benchmark table is transcribed. The standard-gravity value is
    cited as a scalar convention for the toy analytic drop, not as
    fielded-vehicle data.
  script: none
verification:
  method: >-
    The byte-stability gate replays the scenario through the
    kernel and compares Parquet telemetry byte-for-byte against the
    pinned reference. Closed-form residuals are also asserted through
    the CLI tolerance table for constant acceleration.
  test: >-
    crates/openbmp-sim/tests/analytic_toy.rs;
    crates/openbmp-cli/tests/expected/constant-acceleration-drop.toml
  tolerance: >-
    Byte-stable replay for telemetry output; analytic residuals
    within the tolerance table declared by the CLI expected file.
validation_status: validated-toy
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic analytic toy scenario. It contains no operational
    mission profile, no real vehicle calibration, and no fielded
    vehicle data.
units:            metres, metres-per-second, metres-per-second-squared, seconds, kilograms
frame_profile:    toy-fixed-earth
related_files:
  - crates/openbmp-cli/tests/expected/constant-acceleration-drop.toml
  - crates/openbmp-sim/tests/expected/constant-acceleration-drop.toml
notes: >-
  This scenario is the source-of-truth for the byte-stability
  contract and the `Scenario::from_toml_str_with_source_dir` parser
  unit tests in `crates/openbmp-scenario/src/scenario.rs`. Mutating
  the file (whitespace, ordering, or value changes) will invalidate
  every downstream regression. It is the canonical analytic-toy
  validation deliverable.
```
