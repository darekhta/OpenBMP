# Provenance - `openbmp.cola.launch_screening.v1`

Canonical OpenBMP provenance record for the synthetic day-of-launch launch-COLA
screening deck used by WP-19.6 tests.

```yaml
dataset_id:       openbmp.cola.launch_screening.v1
files:
  - data/cola/launch-cola-screening-v1.toml
  - crates/openbmp-trajopt/src/dol.rs
source_class:     synthetic-openbmp-test-fixture
source_title:     OpenBMP deterministic launch-COLA screening deck
source_authors:   OpenBMP contributors
source_id:        WP-19.6 synthetic fixture
publication_date: 2026-06-25
source_urls:      []
license_or_terms: Apache-2.0 OR MIT
retrieved_utc:    2026-06-25
transformation:
  method: >-
    No external space-object catalog or range product is ingested. The TOML
    deck declares one synthetic catalog object and two launch-window slots:
    one trajectory with positive clearance and one trajectory whose closest
    approach enters the declared combined keepout. The openbmp-trajopt DOL
    COLA substrate evaluates constant-velocity catalog propagation and
    piecewise-linear vehicle trajectories.
  command: cargo test -p openbmp-trajopt cola --locked
verification:
  method: >-
    In-crate tests validate exact closed-form closest-approach geometry,
    blocked-slot latching, malformed-input rejection, and parsing of this
    synthetic fixture into a deterministic clear/blocked slot table.
  test: cargo test -p openbmp-trajopt cola --locked
  tolerance: 1.0e-12 m for the hand-computed closest-approach case.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic catalog states support launch-COLA screening method tests.
    They do not claim operational catalog quality, covariance realism,
    range clearance authority, or maneuver planning. Real catalog use remains
    LOCAL-only and must not be committed to this repository.
```
