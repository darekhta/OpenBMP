# Provenance - `openbmp.wind.hwm14.full.v1`

Canonical OpenBMP provenance record for the HWM14 wind evaluator and
bundled HWM14 data files used by `crates/openbmp-physics/src/wind/hwm14.rs`.

```yaml
dataset_id:       openbmp.wind.hwm14.full.v1
files:
  - crates/openbmp-physics/src/wind/hwm14.rs
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
  hwm14.f90: 7349531cc47f544f4b292b5af388c87c679eb8403a33b44bbddc8f00cc812c9b
  checkhwm14.f90: 09157b121b33152d0861fd100e03025f0c75c8c56b65a603c1f4c1abc6345061
  hwm123114.bin: 6e445f8337c7efc815b7ff7f9f967d8a9a16b469d93df9c46e54930553beb906
  dwm07b104i.dat: f2b8eff002d55b0f6d49d202c73b7a9cb6685f2c6bb57f1291a2adc7aa07cf4f
  gd2qd.dat: 6bb1f2384e30b409240ee92c32726ed2a3d73eb550c05e0969d1178032319804
  LICENSE.txt: 1c38a3c446e4ee976e9127746031523319f3004d945f48a40bb56facacd4b570
  README-hwm14.txt: a419c9047127fe64050744bc734ac4ed96228a6d077e4a7655964627b01229a1
license_or_terms: >-
  The inspected `lgpedersen/hwm14` repository is MIT licensed and
  contains the HWM14 source, verification driver, and data files. The
  OpenBMP implementation is a Rust port of the public quiet-time HWM14
  evaluator, DWM07 disturbance evaluator, and geodetic-to-quasi-dipole
  support routines, with the three published data files bundled under
  `data/wind/hwm14/`.
retrieved_utc:    2026-05-27
transformation:
  method: >-
    Ported `hwm14.f90` numerics to deterministic Rust, preserving the
    public data-file formats through `include_bytes!` parsers. The
    runner exposes fixed HWM14 scalar inputs (`day_of_year`, `utc_s`,
    geodetic latitude/longitude, and current 3-hour Ap); altitude is
    supplied by the same position-z proxy used by `LayeredWind`.
    Meridional and zonal winds are mapped to local-NED north/east with
    down=0.
  command: >-
    gfortran -O0 hwm14.f90 checkhwm14.f90 -o /tmp/openbmp-hwm14-check &&
    HWMPATH=/tmp/openbmp-nrlmsis-hwm-hwm14/src /tmp/openbmp-hwm14-check
verification:
  method: >-
    In-crate tests compare the Rust evaluator against public
    `checkhwm14` quiet and total height-profile values, verify DWM07
    disturbance sensitivity to Ap, validate inputs, and check the
    WindModel altitude-proxy behavior.
  test: cargo test -p openbmp-physics hwm14
  tolerance: 1.0e-3 m/s against public `checkhwm14` output.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public empirical horizontal-wind model for academic boost-drag and
    re-entry deceleration studies. HWM14 does not add guidance,
    targeting, or optimization capability; it only supplies neutral
    horizontal wind samples to the existing environment bus.
```
