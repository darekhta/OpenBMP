# openbmp-modelio

`openbmp-modelio` is an L1 ingestion boundary for external reduced-model assets.
It owns file-format schemas, deterministic parsers, and source-reference
validation for model artifacts that higher layers adapt into runtime types.

The crate intentionally does not depend on `openbmp-runner`, `openbmp-sim`, or
`openbmp-vehicle`. That keeps OP2/OP4/CSV parsing reusable for doc-02 structural
modal ingestion and doc-01 FFRF assets without introducing an up-layer edge.

The first supported format is `openbmp.modal_asset.csv.v1`, a CSV modal asset
with frequencies, modal masses, mode-shape samples at instrumented stations, and
source-reference columns used for frequency-error and MAC checks.
