# Provenance — `data/magnetic/`

Canonical OpenBMP provenance record for the World Magnetic Model
2025 coefficient file shipped under `data/magnetic/`. See
[`docs/data-provenance.md`](../../docs/data-provenance.md) for the
contract this record satisfies.

## `data/magnetic/WMM.COF`

```yaml
dataset_id:       openbmp.magnetic.wmm_2025.v1
files:
  - data/magnetic/WMM.COF
source_class:     public-standard
source_title:     World Magnetic Model 2025 (WMM2025) coefficient file
source_authors:   >-
  NOAA National Centers for Environmental Information (NCEI),
  US National Geospatial-Intelligence Agency (NGA),
  British Geological Survey (BGS) on behalf of UK Defence Geospatial Centre (DGC),
  in coordination with WMM Technical Team (Manoj Nair, Arnaud Chulliat,
  Patrick Alken, Brian Meyer, Lucy Bell, Susan Macmillan, William Brown).
source_id:        World Magnetic Model 2025 (WMM2025); model release 11/13/2024.
publication_date: 2024-12-17
source_urls:
  - https://www.ncei.noaa.gov/products/world-magnetic-model
  - https://www.ncei.noaa.gov/products/world-magnetic-model/wmm-coefficients
  - https://www.ncei.noaa.gov/sites/default/files/2024-12/WMM2025COF.zip
license_or_terms: U.S. government technical product; public domain.
source_hash_sha256: dfa8597825af4e0b87ff4198a5b4fb661b3c49f4cd090cd0164e0259b075582f
retrieved_utc:    2026-04-29
transformation:
  method: >-
    Verbatim copy of the upstream WMM2025COF.zip's `WMM.COF` file
    (the model also ships a `WMM2025.COF` sibling with identical
    bytes). No fit, no reformat, no scaling. The 90-coefficient
    table covers degrees `n = 1..12` and orders `m = 0..n` with
    main-field values at epoch 2025.0 plus secular variation rates
    (nT / year). Validity: decimal years `[2025.0, 2030.0)`.
  script: none
verification:
  method: >-
    Compile-time pinned in
    `openbmp-physics::magnetic::wmm2025::coefficients::COEFFS`. The
    Phase-3.10.A regression test
    `crates/openbmp-physics/tests/wmm_data_pin.rs` reads this file via
    `include_bytes!`, hashes it, and asserts that the digest matches
    the pin recorded above and that the parsed coefficients agree
    with the in-source `COEFFS` table to bit precision. The 100
    NOAA-published reference points in
    `data/magnetic/WMM2025_TestValues.txt` drive the
    cross-validation tests in `wmm2025::tests`; all 100 rows are
    parsed directly from the shipped table and asserted within
    5 nT of the published X / Y / Z components (well under the
    4-significant-figure tolerance the WMM publication declares).
contact:          >-
  Manoj Nair / Arnaud Chulliat — geomag.models@noaa.gov
  (NCEI; +1 303 497 4642 / +1 303 497 6522)
```

## `data/magnetic/WMM2025_TestValues.txt`

```yaml
dataset_id:       openbmp.magnetic.wmm_2025_test_values.v1
files:
  - data/magnetic/WMM2025_TestValues.txt
source_class:     public-standard
source_title:     >-
  WMM 2025 reference test values (100-row table covering
  decimal-year, altitude, geodetic lat/lon, X/Y/Z/H/F nT, and
  secular-variation derivatives) shipped alongside the WMM2025COF
  release.
source_id:        Companion to WMM2025 coefficients (release 11/13/2024).
publication_date: 2024-12-17
source_urls:
  - https://www.ncei.noaa.gov/sites/default/files/2024-12/WMM2025COF.zip
license_or_terms: U.S. government technical product; public domain.
source_hash_sha256: e6975b093dddeb6153e0b23cc418425c438167e7c5b1dd795da379cb654f5819
retrieved_utc:    2026-04-29
transformation:
  method: >-
    Verbatim copy of the upstream `WMM2025_TestValues.txt`. Used
    as the parser-driven reference set for the `wmm2025::tests`
    cross-validation suite.
verification:
  method: >-
    The Phase-3.10.A test suite parses all 100 rows of this file
    and asserts agreement within 5 nT per X / Y / Z component
    (well inside the 4-sig-fig agreement the WMM publication
    declares).
```

## `data/magnetic/README-WMM-COEFS.txt`

Verbatim copy of the upstream README for the WMM2025COF release.
SHA-256 pin: `a1a00a3d3250f6bbcfb510db3b750c5dece8a6bd5455086ef542ea82b34fffec`.
Retrieved 2026-04-29 from the same NOAA NCEI release zip. Stored
for traceability of the upstream installation guidance and
support contacts.

## Validity envelope

WMM 2025 is authoritative for **decimal years
`[2025.0, 2030.0)`** (the 5-year validity window NOAA publishes
with each WMM release). Outside this range the
[`Wmm2025`](../../crates/openbmp-physics/src/magnetic/wmm2025.rs)
constructor and per-step evaluator fail closed with
`EnvError::OutOfEnvelope`. The next release (WMM 2030) is
expected at NOAA NCEI in late 2029; the OpenBMP port lands in
whichever phase is current at that time.

## References

- Chulliat, A., W. Brown, P. Alken, M. Nair, S. Macmillan, B.
  Meyer, L. Bell, J. Goedde. *The World Magnetic Model 2025*.
  NOAA NCEI Technical Report (December 2024). Public-domain US
  Government work.
- NOAA NCEI Geomagnetic Modeling Team & British Geological
  Survey, 2024: *World Magnetic Model 2025*. National Centers
  for Environmental Information, NOAA. DOI: 10.25923/prbc-s316.
