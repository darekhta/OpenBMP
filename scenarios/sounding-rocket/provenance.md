# Provenance — `scenarios/sounding-rocket/`

Canonical OpenBMP provenance record for the sounding-rocket
scenarios shipped under `scenarios/sounding-rocket/`. See
[`docs/data-provenance.md`](../../docs/data-provenance.md) for the
contract this record satisfies.

## `scenarios/sounding-rocket/niskanen-2009-chapter6.toml`

```yaml
dataset_id:       openbmp.scenario.sounding_rocket.niskanen_2009_chapter6.v1
files:
  - scenarios/sounding-rocket/niskanen-2009-chapter6.toml
source_class:     converted-public
source_title:     >-
  Phase-2.10 canonical scenario form of the Niskanen 2009
  Chapter-6 small-rocket benchmark, expressed in the OpenBMP
  scenario schema. References the Phase-2.6 Estes C6 motor file
  and the Phase-2.5 reduced-CD aero deck via SHA-256-pinned
  paths.
source_authors:   Niskanen, S. (2009 thesis); OpenBMP (Dmitri Arekhta) for the OpenBMP-schema rendering
source_id:        Niskanen 2009 Chapter 6 small-rocket benchmark
source_url:       https://openrocket.sourceforge.net/thesis.pdf
publication_date: 2009-05-20
methodology_reference: >-
  Niskanen, S. (2009). *Development of an Open-Source Model Rocket
  Simulation Software*. Master's thesis, Helsinki University of
  Technology. Chapter 6 documents a small-rocket benchmark
  comparing simulated apogees against flight experiments for both
  Estes B4 and C6 motors. The OpenBMP scenario instantiates the
  C6 case with the published 80 g airframe mass, constant axial
  CD = 0.8 reduction, and the same vertical-launch convention used
  by the Phase-2.9 integration test in
  `crates/openbmp-vehicle/tests/sounding_rocket.rs`.
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
    Render the already-provenanced Phase-2.9 Niskanen C6 benchmark
    inputs into the Phase-2.10 scenario schema. The scenario file
    references the aero deck and motor data by relative path and
    pins each reference with the SHA-256 digest shipped in the same
    commit. Launch-site coordinates use the documented Helsinki
    proxy because the thesis does not publish exact coordinates.
  script: none
verification:
  method: >-
    `openbmp check` parses the scenario and resolves the pinned aero
    deck and motor file digests. The Phase-2.9 sounding-rocket
    integration test independently replays the same Niskanen C6
    reduction and compares simulated apogee against the published
    151.5 m experimental value with ±5% tolerance. The Phase-2.11
    runner will consume this scenario file directly.
  test: >-
    crates/openbmp-cli/tests/cli_snapshots.rs;
    crates/openbmp-vehicle/tests/sounding_rocket.rs
  tolerance: >-
    Parser/check path requires exact SHA-256 pin matches. Physics
    regression tolerance remains the Phase-2.9 ±5% apogee envelope
    against the published C6 experimental value.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public academic benchmark from a master's thesis. The modeled
    object is a hobby-class sounding/model rocket reduction, not an
    operational vehicle. The scenario contains no targeting, no
    real device drivers, and no restricted or fielded-vehicle data.
units:            metres, metres-per-second, metres-per-second-squared, seconds, kilograms, degrees
frame_profile:    wgs84-uniform-rotation
local_origin:     >-
  Helsinki proxy site (latitude 60.18°, longitude 24.83°, height
  0.0 m). Niskanen Chapter 6 does not publish exact launch-site
  coordinates; this proxy is documented in the scenario's
  `[frames.local_origin].source` field.
related_files:
  - data/scenarios/niskanen-2009-chapter6.toml
  - data/aero/synthetic-niskanen-ch6-rocket.toml
  - data/motors/estes-c6-eng-derived.toml
notes: >-
  Each external file reference (`aero.deck`, `propulsion.motor.file`)
  carries an optional `*_sha256` pin that fails closed on mismatch.
  The pin values match the data files at the commit shipping this
  scenario; if a referenced file is updated, recompute the digest
  with `shasum -a 256 <file>` and update the pin.
```
