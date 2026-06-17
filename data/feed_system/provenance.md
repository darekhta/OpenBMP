# Feed-System Data Provenance

```yaml
dataset_id: openbmp.feed_system.generic_steady_feed_network.v1
files:
  - data/feed_system/generic-steady-feed-network-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic steady feed-network mass-balance tolerance table"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-17"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "closed-form evaluation of the reduced tank-valve-chamber balance and two-valve node/branch mass-balance equations used by openbmp-feedsystem"
  script: "manual arithmetic recorded in crates/openbmp-feedsystem/src/network.rs and crates/openbmp-feedsystem/src/graph.rs regression tests"
verification:
  method: "parse the TOML table and compare expected chamber pressure, node pressures, branch flows, throat flow, and mass residuals against TankValveChamberNetwork and SteadyFeedGraph"
  test: "cargo test -p openbmp-feedsystem steady_feed_network_matches_provenance_tolerance_table --locked && cargo test -p openbmp-feedsystem graph_matches_provenance_tolerance_table --locked"
  tolerance: "absolute tolerances recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic reduced feed-network fixture; no real vehicle, feed system, propellant-system, or operational data."
```

```yaml
dataset_id: openbmp.feed_system.generic_transient_chamber.v1
files:
  - data/feed_system/generic-transient-chamber-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic transient chamber/feed-network tolerance table"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-17"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "exact constant-input lumped chamber-pressure solution and start-of-step incompressible dual-valve feed-flow equations used by openbmp-feedsystem"
  script: "manual arithmetic recorded in crates/openbmp-feedsystem/src/chamber.rs and crates/openbmp-feedsystem/src/transient.rs regression tests"
verification:
  method: "parse the TOML table and compare expected chamber pressure, pressure rate, inlet/outlet mass flow, mixture ratio, and dual-valve leg flows against TransientChamber and TransientDualValveFeedNetwork"
  test: "cargo test -p openbmp-feedsystem transient_chamber_matches_provenance_tolerance_table --locked && cargo test -p openbmp-feedsystem transient_dual_valve_network_matches_provenance_tolerance_table --locked"
  tolerance: "absolute tolerances recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic reduced transient chamber and dual-valve feed fixture; no real vehicle, feed system, propellant-system, or operational data."
```

```yaml
dataset_id: openbmp.feed_system.generic_feed_controller.v1
files:
  - data/feed_system/generic-feed-controller-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic throttle/mixture controller tolerance table"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-17"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "closed-form evaluation of the pressure/MR PI law, symmetric integral clamps, valve bounds, and slew limiter used by openbmp-feedsystem"
  script: "manual arithmetic recorded in crates/openbmp-feedsystem/src/control.rs regression test"
verification:
  method: "parse the TOML table and compare expected pressure error, mixture-ratio error, integral state, raw valve requests, and bounded valve commands against ThrottleMixtureController"
  test: "cargo test -p openbmp-feedsystem controller_matches_provenance_tolerance_table --locked"
  tolerance: "absolute tolerances recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic reduced controller fixture; no real vehicle, controller tuning, feed system, propellant-system, or operational data."
```

```yaml
dataset_id: openbmp.feed_system.generic_turbopump_map.v1
files:
  - data/feed_system/generic-turbopump-map-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic normalized turbopump affinity/NPSH tolerance table"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-11"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "closed-form evaluation of the normalized pump map, affinity-law speed scaling, and NPSH head balance used by openbmp-feedsystem"
  script: "manual arithmetic recorded in crates/openbmp-feedsystem/src/pump.rs regression test"
verification:
  method: "parse the TOML table and compare each expected operating-point field against Turbopump::solve"
  test: "cargo test -p openbmp-feedsystem turbopump_matches_provenance_tolerance_table --locked"
  tolerance: "absolute tolerance recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic generic pump map; no real vehicle, pump, propellant-system, or operational data."
```

```yaml
dataset_id: openbmp.feed_system.generic_moc_line.v1
files:
  - data/feed_system/generic-moc-line-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic fixed-grid MOC Joukowsky tolerance table"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-11"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "closed-form evaluation of frictionless MOC characteristics and the Joukowsky valve-closure bound used by openbmp-feedsystem"
  script: "manual arithmetic recorded in crates/openbmp-feedsystem/src/line.rs regression test"
verification:
  method: "parse the TOML table and compare the expected Courant step, downstream head/pressure, pressure delta, and min/max heads against MocLine::step"
  test: "cargo test -p openbmp-feedsystem moc_line_matches_provenance_tolerance_table --locked"
  tolerance: "absolute tolerances recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic single-pipe closure fixture; no real vehicle, feed system, or operational data."
```

```yaml
dataset_id: openbmp.feed_system.generic_pogo_stability.v1
files:
  - data/feed_system/generic-pogo-stability-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic reduced POGO stability tolerance table"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-11"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "closed-form evaluation of the reduced cubic POGO characteristic and Routh-Hurwitz stability margins used by openbmp-feedsystem"
  script: "manual arithmetic recorded in crates/openbmp-feedsystem/src/pogo.rs regression test"
verification:
  method: "parse the TOML table and compare expected coefficients, margins, verdicts, and neutral accumulator compliance against PogoStability::analyze"
  test: "cargo test -p openbmp-feedsystem pogo_matches_provenance_tolerance_table --locked"
  tolerance: "absolute tolerance recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic reduced feed-half stability fixture; no real vehicle structural mode, feed system, or case-history data."
```
