# Provenance - `openbmp.wind.hwm14.full.v1`

This directory contains the bundled HWM14 / DWM07 data files used by
`crates/openbmp-physics/src/wind/hwm14.rs`.

```yaml
dataset_id:       openbmp.wind.hwm14.full.v1
files:
  - data/wind/hwm14/hwm123114.bin
  - data/wind/hwm14/dwm07b104i.dat
  - data/wind/hwm14/gd2qd.dat
source_class:     public-academic-reference
source_title:     Horizontal Wind Model 2014 (HWM14), Version HWM14.123114
source_authors:   Drob, D. P.; Emmert, J. T.
source_id:        Earth and Space Science 2(7), 301-319, 2015
publication_date: 2015-07-01
source_urls:
  - https://github.com/lgpedersen/hwm14
  - https://map.nrl.navy.mil/map/pub/nrl/HWM/HWM14/
  - https://doi.org/10.1002/2014EA000089
source_hashes_sha256:
  hwm123114.bin: 6e445f8337c7efc815b7ff7f9f967d8a9a16b469d93df9c46e54930553beb906
  dwm07b104i.dat: f2b8eff002d55b0f6d49d202c73b7a9cb6685f2c6bb57f1291a2adc7aa07cf4f
  gd2qd.dat: 6bb1f2384e30b409240ee92c32726ed2a3d73eb550c05e0969d1178032319804
license_or_terms: >-
  The inspected `lgpedersen/hwm14` repository is MIT licensed and contains the
  HWM14 source, verification driver, and data files.
retrieved_utc:    2026-05-27
transformation:
  method: downloaded verbatim from the public HWM14 data distribution.
verification:
  method: in-crate HWM14 tests compare the Rust evaluator against public
    `checkhwm14` quiet and total height-profile values.
  test: cargo test -p openbmp-physics hwm14
  tolerance: 1.0e-3 m/s against public `checkhwm14` output.
validation_status: checked
parent_record: data/wind/provenance.md
```
