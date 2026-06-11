# Feed-System Data Provenance

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
