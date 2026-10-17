# Acoustics Data Provenance

```yaml
dataset_id: openbmp.acoustics.dsm_source_directivity.v1
files:
  - data/acoustics/dsm-source-directivity-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic DSM source allocation and directivity deck"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-19"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "hand-authored normalized three-point source allocation and three-angle uniform one-third-octave directivity rows for deterministic deck parser evidence"
  script: "manual fixture mirrored by crates/openbmp-acoustics/src/dsm.rs regression tests"
verification:
  method: "parse the TOML deck, validate provenance metadata, validate normalized source allocation, validate directivity dimensions and interpolation, require deterministic SHA-256 digest, and reject malformed decks fail-closed"
  test: "cargo test -p openbmp-acoustics dsm_deck --locked -j 1"
  tolerance: "exact parser fields plus 1e-12 interpolation tolerance"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic DSM parser fixture; no NASA SP-8072 curve transcription, no fielded launch-vehicle acoustic calibration, and no production acoustic-environment claim."
```

```yaml
dataset_id: openbmp.acoustics.dsm_air_absorption.v1
files:
  - data/acoustics/dsm-air-absorption-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic DSM one-third-octave air absorption deck"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-19"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "hand-authored monotone frequency-dependent one-third-octave attenuation coefficients for deterministic DSM absorption-deck parser and runner evidence"
  script: "manual fixture mirrored by crates/openbmp-acoustics/src/dsm.rs and crates/openbmp-runner/src/point_mass.rs regression tests"
verification:
  method: "parse the TOML deck, validate provenance metadata, validate one-third-octave band/value dimensions, require deterministic SHA-256 digest, reject malformed decks fail-closed, and prove runner report provenance for a pinned scenario"
  test: "cargo test -p openbmp-acoustics dsm_air_absorption --locked -j 1 && cargo test -p openbmp-runner point_mass_acoustics_absorption_deck_report_carries_provenance --locked -j 1"
  tolerance: "exact parser fields plus finite DSM station output"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic DSM air-absorption parser fixture; no ISO 9613, ANSI, atmospheric profile, weather-dependent attenuation, or production acoustic-environment claim."
```

```yaml
dataset_id: openbmp.acoustics.dsm_air_absorption_atmospheres.v1
files:
  - data/acoustics/dsm-air-absorption-atmospheres-v1.toml
source_class: synthetic-openbmp-atmosphere-profiles
source_title: "Synthetic DSM atmosphere-specific one-third-octave air absorption deck"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-19"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "hand-authored named atmosphere profile attenuation coefficients with representative temperature, pressure, and humidity metadata for deterministic DSM absorption-profile selection evidence"
  script: "manual fixture mirrored by crates/openbmp-acoustics/src/dsm.rs and crates/openbmp-runner/src/point_mass.rs regression tests"
verification:
  method: "parse the TOML deck, validate provenance metadata, validate profile ids and atmosphere metadata, validate one-third-octave band/value dimensions, require deterministic SHA-256 digest, reject duplicate profiles and malformed humidity fail-closed, and prove runner profile selection for a pinned scenario"
  test: "cargo test -p openbmp-acoustics dsm_air_absorption --locked -j 1 && cargo test -p openbmp-runner point_mass_acoustics_absorption_profile_selects_atmosphere --locked -j 1"
  tolerance: "exact parser fields plus finite DSM station output"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic DSM atmospheric absorption profile fixture; no ISO 9613, ANSI, weather service atmosphere, flight correlation, or production acoustic-environment claim."
```

```yaml
dataset_id: openbmp.acoustics.pad_suppression.v1
files:
  - data/acoustics/pad-suppression-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic pad acoustic suppression deck"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-19"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "hand-authored one-third-octave dry and nominal-water knockdown classes for deterministic pad-suppression parser and runner evidence"
  script: "manual fixture mirrored by crates/openbmp-acoustics/src/pad.rs and crates/openbmp-runner/src/point_mass.rs regression tests"
verification:
  method: "parse the TOML deck, validate provenance metadata, validate one-third-octave band/value dimensions, require deterministic SHA-256 digest, reject malformed decks fail-closed, and prove runner report provenance plus station OASPL knockdown for a pinned scenario"
  test: "cargo test -p openbmp-acoustics pad_suppression --locked -j 1 && cargo test -p openbmp-runner point_mass_acoustics_suppression_deck_report_carries_provenance --locked -j 1"
  tolerance: "exact parser fields plus finite DSM station output below the unsuppressed synthetic fixture"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic pad water-suppression parser/runtime fixture; no published suppression-curve digitization, pad geometry calibration, cluster acoustics, or production acoustic-environment claim."
```

```yaml
dataset_id: openbmp.acoustics.pad_cluster_interaction.v1
files:
  - data/acoustics/pad-cluster-interaction-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic pad acoustic cluster interaction deck"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-19"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "hand-authored one-third-octave incoherent/open and coherent/shielded classes for deterministic cluster-interaction parser and runner evidence"
  script: "manual fixture mirrored by crates/openbmp-acoustics/src/pad.rs and crates/openbmp-runner/src/point_mass.rs regression tests"
verification:
  method: "parse the TOML deck, validate provenance metadata, validate one-third-octave band/value dimensions, require deterministic SHA-256 digest, reject malformed decks fail-closed, and prove runner report provenance plus closed-form same-location cluster OASPL delta for a pinned scenario"
  test: "cargo test -p openbmp-acoustics pad_cluster_interaction --locked -j 1 && cargo test -p openbmp-runner point_mass_acoustics_cluster_interaction_deck_report_carries_provenance --locked -j 1"
  tolerance: "exact parser fields plus 1e-12 OASPL delta tolerance against 20 log10(2) - 1.5 dB"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic pad cluster-interaction parser/runtime fixture; no calibrated plume interaction, cluster-specific directivity, published suppression data, or production acoustic-environment claim."
```

```yaml
dataset_id: openbmp.acoustics.iop_duct.v1
files:
  - data/acoustics/iop-duct-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic ignition-overpressure duct/trench coupling deck"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-19"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "hand-authored open-pad and long-trench coupling classes for deterministic IOP duct-deck parser and runner provenance evidence"
  script: "manual fixture mirrored by crates/openbmp-acoustics/src/iop.rs and crates/openbmp-runner/src/point_mass.rs regression tests"
verification:
  method: "parse the TOML deck, validate provenance metadata, validate nonempty unique class ids, validate positive duct volume/exit-area/discharge-velocity values and non-negative geometry gains, require deterministic SHA-256 digest, reject malformed decks fail-closed, and prove runner report provenance for a pinned scenario"
  test: "cargo test -p openbmp-acoustics iop_duct --locked -j 1 && cargo test -p openbmp-runner point_mass_acoustics_iop_duct_deck_report_carries_provenance --locked -j 1"
  tolerance: "exact parser fields plus closed-form IOP residence-time and peak-overpressure values"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic IOP duct/trench parser/runtime fixture; no facility calibration, public Shuttle trend fit, suppression coupling, or production overpressure-environment claim."
```

```yaml
dataset_id: openbmp.acoustics.ascent_env.v1
files:
  - data/acoustics/ascent-acoustic-env-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic ascent acoustic environment class deck"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-19"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "hand-authored attached-flow and transonic q-scaled one-third-octave class rows with overlapping Mach gates for deterministic ascent-acoustics parser/evaluator and Mach-gate selector evidence"
  script: "manual fixture mirrored by crates/openbmp-acoustics/src/ascent.rs regression tests"
verification:
  method: "parse the TOML deck, validate provenance metadata, validate one-third-octave band/value dimensions, validate unique class ids and Mach gates, require deterministic SHA-256 digest, reject malformed decks fail-closed, prove q-scaling plus OASPL-normalized band energy, prove deterministic Mach-gate class selection including no-match and ambiguity rejection, and prove schema/runner report provenance plus gated telemetry for a pinned scenario"
  test: "cargo test -p openbmp-acoustics ascent_acoustic --locked -j 1 && cargo test -p openbmp-scenario acoustics_ascent --locked -j 1 && cargo test -p openbmp-runner point_mass_acoustics_ascent --locked -j 1"
  tolerance: "exact parser fields plus 1e-12 OASPL normalization/q-scaling, exact selected class ids for Mach-gate samples, and runner telemetry tolerance"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic ascent acoustic parser/evaluator and Mach-gate selector fixture; no full 03 geometry-driven regime/zone selection, published payload-user-guide curve, MPE class anchor, flight correlation, payload-zone map, fairing insertion loss, or production acoustic-environment claim."
```

```yaml
dataset_id: openbmp.acoustics.fairing_internal.v1
files:
  - data/acoustics/fairing-internal-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic fairing-internal acoustic transfer deck"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-19"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "hand-authored one-third-octave light and heavy synthetic fairing insertion-loss classes for deterministic fairing-internal parser and runner evidence"
  script: "manual fixture mirrored by crates/openbmp-acoustics/src/ascent.rs and crates/openbmp-runner/src/point_mass.rs regression tests"
verification:
  method: "parse the TOML deck, validate provenance metadata, validate one-third-octave band/value dimensions, validate unique class ids and non-negative insertion losses, require deterministic SHA-256 digest, reject malformed decks fail-closed, prove fill-factor plus insertion-loss transfer math, and prove schema/runner report provenance plus gated telemetry for a pinned scenario"
  test: "cargo test -p openbmp-acoustics fairing_internal --locked -j 1 && cargo test -p openbmp-scenario acoustics_fairing_internal --locked -j 1 && cargo test -p openbmp-runner point_mass_acoustics_fairing_internal --locked -j 1"
  tolerance: "exact parser fields plus 1e-12 OASPL transfer and runner telemetry tolerance"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic fairing-internal parser/runtime fixture; no published payload-user-guide curve, MPE class anchor, flight correlation, payload-zone spatial map, or production fairing acoustic-environment claim."
```

```yaml
dataset_id: openbmp.acoustics.vibro_transfer.v1
files:
  - data/acoustics/vibroacoustic-transfer-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic vibroacoustic response transfer deck"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-19"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "hand-authored one-third-octave light/heavy panel transfer classes for deterministic SPL-to-acceleration-PSD response and pressure-PSD-to-modal-force-PSD handoff evidence"
  script: "manual fixture mirrored by crates/openbmp-acoustics/src/response.rs regression tests"
verification:
  method: "parse the TOML deck, validate provenance metadata, validate one-third-octave band/value dimensions, validate unique class ids and non-negative response gains, require deterministic SHA-256 digest, reject malformed decks fail-closed, prove surface-mass scaling, and prove modal-force PSD participation math"
  test: "cargo test -p openbmp-acoustics vibroacoustic --locked -j 1"
  tolerance: "exact parser fields plus 1e-18 response and modal-force PSD tolerance"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic vibroacoustic response parser/evaluator fixture; no SEA calibration, P95/50 environment statistics, 02 loads-stack handoff, flight correlation, or qualification-environment claim."
```

```yaml
dataset_id: openbmp.acoustics.iop_suppression.v1
files:
  - data/acoustics/iop-suppression-v1.toml
source_class: synthetic-openbmp
source_title: "Synthetic ignition-overpressure suppression deck"
source_authors: "OpenBMP contributors"
source_url: "repository-local analytic fixture"
license_or_terms: "OpenBMP project synthetic data"
retrieved_utc: "2026-06-19"
source_hash_sha256: "not applicable: repository-local synthetic derivation"
transformation:
  method: "hand-authored dry and nominal-water pressure knockdown classes for deterministic IOP suppression-deck parser and runner evidence"
  script: "manual fixture mirrored by crates/openbmp-acoustics/src/iop.rs and crates/openbmp-runner/src/point_mass.rs regression tests"
verification:
  method: "parse the TOML deck, validate provenance metadata, validate nonempty unique class ids, validate non-negative pressure-amplitude suppression losses and uncertainty values, require deterministic SHA-256 digest, reject malformed decks fail-closed, and prove additive runner suppression plus report provenance for a pinned scenario"
  test: "cargo test -p openbmp-acoustics iop_suppression --locked -j 1 && cargo test -p openbmp-runner point_mass_acoustics_iop_suppression_deck_report_carries_provenance --locked -j 1"
  tolerance: "exact parser fields plus 1e-12 closed-form IOP pressure knockdown tolerance"
validation_status: validated-toy
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Synthetic IOP suppression parser/runtime fixture; no facility deluge calibration, public Shuttle trend fit, acoustic suppression curve reuse claim, or production overpressure-environment claim."
```

```yaml
dataset_id: openbmp.acoustics.sp8072_source.v1
files:
  - data/acoustics/sp8072-source-v1.toml
  - data/acoustics/sp8072-figure5-spectrum-v1.toml
  - data/acoustics/sp8072-figure10-directivity-v1.toml
  - data/acoustics/sp8072-figure12-source-power-v1.toml
  - data/acoustics/sp8072-figure13-axial-spectrum-v1.toml
  - data/acoustics/sp8072-figure14-apparent-source-v1.toml
source_class: public-nasa-source-manifest
source_title: "NASA SP-8072 Acoustic Loads Generated by the Propulsion System"
source_authors: "NASA"
source_url: "https://ntrs.nasa.gov/api/citations/19710023719/downloads/19710023719.pdf"
ntrs_citation_id: "19710023719"
license_or_terms: "Public NASA Technical Reports Server document"
retrieved_utc: "2026-06-19"
source_hash_sha256: "84f1483c2c8477810186e98765ea99e4b7f7cc71e0a47eb8f86977d3b02aabf8"
transformation:
  method: "manual source-manifest inventory of SP-8072 metadata and DSM-relevant figure page references from the pinned PDF; Figures 5, 10, 12, 13, and 14 include coarse rounded visual samples for parser/runtime evidence"
  script: "pdftotext page scan, PDF raster visual digitization, and crates/openbmp-acoustics/src/dsm.rs parser regression tests"
verification:
  method: "parse the TOML manifest and Figure 5/10/12/13/14 curve decks, validate source metadata, validate lowercase SHA-256 digest syntax, require nonempty unique figure ids, validate increasing positive curve axes, derive Figure 5 normalized DSM band spectra by Strouhal interpolation, convert Figure 12 samples into normalized DSM source allocation, derive Figure 10 band directivity by Strouhal/angle interpolation, derive Figure 13 source-position-dependent band weights by modified-axial-Strouhal interpolation, derive Figure 14 apparent source locations by Strouhal interpolation, exercise the coarse SP-8072 curve tolerance-table substrate against a controlled reference spectrum, and reject malformed manifests/decks/tolerances fail-closed"
  test: "cargo test -p openbmp-acoustics sp8072 --locked -j 1"
  tolerance: "exact parser fields and deterministic tolerance-table formatting; coarse Figure 5/10/12/13/14 samples are rounded visual digitization and are not a public worked-example reference table"
validation_status: coarse-public-digitization
safety_review:
  reviewer: "OpenBMP maintainers"
  decision: accepted
  notes: "Public NASA source inventory plus coarse Figure 5/10/12/13/14 parser/runtime fixtures and tolerance-table substrate; production SP-8072 digitized curve decks and public worked-example reference-value reproduction remain pending."
```
