# Modal Data Provenance

Canonical OpenBMP provenance record for external reduced modal-model assets.

## `synthetic-sol103-modal-asset-v1.csv`

- Deck path: `data/modal/synthetic-sol103-modal-asset-v1.csv`
- Scope: synthetic two-mode lateral bending modal asset for WP-02.2-a CSV
  ingestion. The rows mimic a Nastran/CalculiX SOL103-style export after station
  sampling at engine, rate-gyro, and accelerometer locations.
- Schema: `openbmp.modal_asset.csv.v1`
- Source: repository-local synthetic derivation; no fielded vehicle data.
- Source hash SHA-256: not applicable, repository-local synthetic derivation.
- Deck SHA-256: `6d3f3c621a9340d509cca35f111826ad06cf45bc2ded92027a98b4af3c5f3171`
- Verification: `cargo test -p openbmp-modelio --locked` parses the asset,
  checks the deck SHA pin, and proves frequency error below 1% plus MAC above
  0.9 against the source-reference columns.

## `synthetic-calculix-cb-benchmark-v1.csv`

- Deck path: `data/modal/synthetic-calculix-cb-benchmark-v1.csv`
- Scope: synthetic two-mode Craig-Bampton fixed-interface correlation fixture
  for WP-02.3-a. The rows mimic the frequency/shape columns a tiny CalculiX
  `*FREQUENCY` / `*SUBSTRUCTURE` comparison harness would consume, but they are
  repository-local analytic values, not public or tool-generated benchmark
  data.
- Schema: `openbmp.structural_cb_modal_benchmark.csv.v1`
- Source: repository-local synthetic derivation from the 3-DOF sparse
  structural matrix used in `openbmp-structdyn::cb` tests.
- Source hash SHA-256: not applicable, repository-local synthetic derivation.
- Deck SHA-256: `e45f91ff277341f6cf102d0cd351cc7f5d8dd2426fad78d777c2f38edd59f77c`
- Verification: `cargo test -p openbmp-structdyn cb:: --locked` loads the
  fixture, compares extracted sparse fixed-interface frequencies and shapes
  using frequency error plus MAC gates, and proves fail-closed behavior on
  reference drift. This fixture validates the benchmark harness only; real
  CalculiX output remains a separate provenance-pinned validation input.
