# AFTS Data Provenance

```yaml
dataset_id: openbmp.afts.closed_form_iip.v1
files:
  - data/afts/closed-form-iip-tolerance-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic two-body closed-form IIP tolerance table"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-18"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "independent Keplerian conic propagation from ECI position/velocity to the WGS84 spherical impact surface, with WGS84 uniform-rotation longitude correction"
  script: "manual analytic oracle implemented in crates/openbmp-afts/src/lib.rs regression tests"
verification:
  method: "parse the TOML table and compare fixed-step numeric AFTS IIP latitude/longitude and time-to-impact against the closed-form conic oracle"
  test: "cargo test -p openbmp-afts non_radial_iip_matches_closed_form_conic_tolerance_table --locked"
  tolerance: "absolute tolerances recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic AFTS verification fixture; no real vehicle, launch site, protected asset, operational range, or targeting data."
```
