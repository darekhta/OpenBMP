# Communications Data Provenance

```yaml
dataset_id: openbmp.comm.geometric_visibility.v1
files:
  - data/comm/geometric-visibility-oracle-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic fixed-Earth communications visibility tolerance table"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-18"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "closed-form local ENU geometry and line-scan rise/set calculation over a WGS84 equatorial ground site"
  script: "manual analytic oracle implemented in crates/openbmp-comm/src/lib.rs regression tests"
verification:
  method: "parse the TOML table and compare GroundSite::observe_ecef geometry plus visibility_events/pass_intervals rise/set extraction against the independent local ENU oracle"
  test: "cargo test -p openbmp-comm geometric_visibility_matches_oracle_tolerance_table --locked"
  tolerance: "absolute tolerances recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic communications geometry verification fixture; no real vehicle, ground station, RF hardware, terrain, spectrum allocation, or operational contact plan."
```

```yaml
dataset_id: openbmp.comm.sgp4_visibility.v1
files:
  - data/comm/sgp4-visibility-oracle-v1.toml
source_class: external-library
source_title: "sgp4 2.4.0 ISS TLE visibility pass oracle"
source_authors: "sgp4 crate authors; OpenBMP contributors"
source_url: "https://github.com/neuromorphicsystems/sgp4"
license_or_terms: "sgp4 crate MIT; ISS TLE example copied from sgp4 crate documentation"
retrieved_utc: "2026-06-18"
source_hash_sha256: "not applicable: crates.io dependency is pinned by Cargo.lock"
transformation:
  method: "parse the ISS TLE with sgp4 2.4.0, propagate TEME states over the declared window, rotate positions into a fixed Earth frame with the crate's IAU sidereal-time expression, and compare OpenBMP fixed-step event extraction to bisection-refined sgp4 visibility crossings"
  script: "test-only oracle implemented in crates/openbmp-comm/src/lib.rs"
verification:
  method: "parse the TOML table and compare visibility_events rise/set times against the independently refined sgp4 crossing times"
  test: "cargo test -p openbmp-comm sgp4_visibility_matches_external_oracle_tolerance_table --locked"
  tolerance: "absolute event-time tolerance recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "External-library propagation fixture for regression testing only; no operational orbit-determination, tracking, or contact-planning claim."
```

```yaml
dataset_id: openbmp.comm.antenna_body_mask.v1
files:
  - data/comm/antenna-body-mask-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic antenna gain and body-mask deck tolerance table"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-18"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "axisymmetric off-boresight gain interpolation and direct triangle ray casts from a body-frame antenna point against a square occluder"
  script: "test-only fixture parser in crates/openbmp-comm/src/lib.rs"
verification:
  method: "parse the TOML table, compare antenna gain interpolation against expected values, compare precomputed mask samples against direct ray-cast expectations, verify nearest-direction runtime lookup, and require deterministic deck SHA-256 stability"
  test: "cargo test -p openbmp-comm antenna_body_mask_deck_matches_fixture --locked"
  tolerance: "absolute gain tolerance recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic communications antenna/body-mask fixture; no real antenna pattern, vehicle mesh, RF calibration, or operational coverage claim."
```

```yaml
dataset_id: openbmp.comm.link_budget_fer.v1
files:
  - data/comm/link-budget-fer-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic link budget and FER tolerance table"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-18"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "hand-computed one-kilometre S-band free-space path loss plus deterministic dB-budget operand order and log-linear frame-error-rate interpolation"
  script: "test-only fixture parser in crates/openbmp-comm/src/lib.rs"
verification:
  method: "parse the TOML table, compare free-space path loss, C/N0, Eb/N0, margin, and FER against recorded analytic expected values, and require out-of-envelope FER interpolation to fail closed"
  test: "cargo test -p openbmp-comm link_budget_and_fer_curve_match_fixture --locked"
  tolerance: "absolute dB and FER tolerances recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic communications budget fixture; no real RF hardware, waveform, coding standard, spectrum allocation, station calibration, or operational link-performance claim."
```

```yaml
dataset_id: openbmp.comm.attenuation_loss.v1
files:
  - data/comm/attenuation-loss-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic elevation-indexed attenuation and rain loss table"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-18"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "hand-authored elevation-to-loss tables with exact linear interpolation cases for atmospheric and rain attenuation"
  script: "test-only fixture parser in crates/openbmp-comm/src/lib.rs"
verification:
  method: "parse the TOML table, compare elevation-indexed interpolation against recorded expected values, and require out-of-envelope loss lookup to fail closed"
  test: "cargo test -p openbmp-comm elevation_loss_deck_matches_fixture --locked"
  tolerance: "absolute loss tolerance recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic attenuation fixture; no ITU-R conformance, site calibration, weather product, spectrum allocation, or operational link-performance claim."
```

```yaml
dataset_id: openbmp.comm.antenna_link_budget.v1
files:
  - data/comm/antenna-link-budget-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic antenna gain/body-mask link budget fixture"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-18"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "hand-authored body-frame boresight/gain samples and a two-triangle occluder arranged so the point-mass regression line of sight is unblocked and receives +3 dBi gain"
  script: "runner fixture consumed by crates/openbmp-runner/src/point_mass.rs"
verification:
  method: "run a toy fixed-Earth point-mass scenario and verify comm.link telemetry reflects the antenna gain and body-mask state in the computed budget"
  test: "cargo test -p openbmp-runner point_mass_comm_report_contains_byte_stable_pass_table --locked"
  tolerance: "absolute dB and FER tolerances asserted in the runner regression"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic antenna-budget fixture; no real antenna, mesh, RF calibration, site coverage, spectrum allocation, or operational link-performance claim."
```

```yaml
dataset_id: openbmp.comm.link_channel_model.v1
files:
  - data/comm/link-channel-model-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic link-channel packet effect fixture"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-18"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "hand-authored slant range, processing delay, and zero-FER packet case for one-way latency and packet disposition evaluation"
  script: "test-only fixture parser in crates/openbmp-comm/src/lib.rs"
verification:
  method: "parse the TOML table, evaluate LinkChannelModel for a deterministic packet context, and compare propagation delay, total latency, disposition, and repeated deterministic draw"
  test: "cargo test -p openbmp-comm link_channel_model_matches_fixture --locked"
  tolerance: "absolute latency tolerance recorded in [tolerances]"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic packet-effect fixture; no waveform, modem, coding-standard, RF hardware, network scheduling, or operational communications claim."
```
