# Recovery Data Provenance

Canonical OpenBMP provenance record for checked-in recovery-system data.

## openbmp.recovery.pflanz_x1.synthetic.v1

- file: `data/recovery/pflanz-x1-synthetic-v1.toml`
- format: `openbmp.recovery.pflanz_x1.v1`
- source_type: repository-local synthetic fixture
- source_hash_sha256: not applicable: values are OpenBMP-authored deterministic parser/runtime evidence
- license: OpenBMP repository license
- method: hand-authored monotone `X1(A, j)` grid shaped like the Pflanz finite-mass reduction surface for deterministic deck parsing, SHA pinning, interpolation, and runtime telemetry wiring tests
- validation: parsed into `PflanzReductionTable`, validated for monotone axes and row-wise nondecreasing reduction factors, then consumed by point-mass recovery telemetry tests
- limitations: not a transcription of Knacke/Pflanz production figures and not a drop-test-calibrated parachute database
