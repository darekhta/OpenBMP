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
    route-planning or optimization capability; it only supplies neutral
    horizontal wind samples to the existing environment bus.
```

---

# Provenance - `openbmp.wind.synthetic_balloon_family.v1`

Canonical OpenBMP provenance record for the synthetic day-of-launch
balloon-family generator deck used by WP-19.1 measured-wind tests and
scenario examples.

```yaml
dataset_id:       openbmp.wind.synthetic_balloon_family.v1
files:
  - data/wind/synthetic-balloon-family-v1.toml
  - crates/openbmp-physics/src/wind/measured.rs
source_class:     synthetic-openbmp-test-fixture
source_title:     OpenBMP deterministic synthetic balloon-family wind deck
source_authors:   OpenBMP contributors
source_id:        WP-19.1 synthetic fixture
publication_date: 2026-06-25
source_urls:      []
license_or_terms: Apache-2.0 OR MIT
retrieved_utc:    2026-06-25
transformation:
  method: >-
    No external meteorological data are ingested. The TOML deck declares
    a deterministic seeded generator configuration: altitude grid, base
    local-NED wind, linear shear, sinusoidal perturbation amplitudes,
    QC thresholds, and Martin-Graham-style filter cutoff. Generated
    profiles are synthetic-only and intended for CI, examples, and
    method-shape validation.
  command: cargo test -p openbmp-physics measured
verification:
  method: >-
    In-crate tests validate exact QC screens, deterministic splice
    attribution artifact rendering, first-order filter magnitude response,
    and seeded synthetic-family repeatability.
  test: cargo test -p openbmp-physics measured
  tolerance: exact semantics for QC/splice determinism; 1.0e-6 closed-form
    magnitude-response agreement.
validation_status: validated-toy
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic wind profiles support steering-through-weather method tests.
    They do not add operational meteorological infrastructure or any
    impact-surface coupling.
```

---

# Provenance - `openbmp.wind.dol_load_indicator_surrogate.v1`

Canonical OpenBMP provenance record for the synthetic day-of-launch
load-indicator surrogate fit deck used by WP-19.3 tests.

```yaml
dataset_id:       openbmp.wind.dol_load_indicator_surrogate.v1
files:
  - data/wind/dol-load-indicator-surrogate-v1.toml
  - crates/openbmp-trajopt/src/dol.rs
source_class:     synthetic-openbmp-test-fixture
source_title:     OpenBMP deterministic DOL load-indicator surrogate fit deck
source_authors:   OpenBMP contributors
source_id:        WP-19.3 synthetic fixture
publication_date: 2026-06-25
source_urls:      []
license_or_terms: Apache-2.0 OR MIT
retrieved_utc:    2026-06-25
transformation:
  method: >-
    No external loads-cycle data are ingested. The TOML deck pins the
    degraded q-alpha/q-beta/gimbal feature vocabulary, synthetic upstream
    deck hash, and held-out validation thresholds used by the deterministic
    least-squares fit in `openbmp-trajopt::dol`.
  command: cargo test -p openbmp-trajopt load_indicator --locked
verification:
  method: >-
    In-crate tests fit the linear surrogate on a seeded synthetic matrix,
    validate held-out Pearson correlation and maximum unconservative error,
    fail closed when the supplied deck hash drifts, and parse this deck as
    the pinned threshold source.
  test: cargo test -p openbmp-trajopt load_indicator --locked
  tolerance: exact deck-hash drift gate; held-out correlation >= 0.95 and
    maximum unconservative error <= 1.0e-6 N*m for the synthetic matrix.
validation_status: validated-toy
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic reduced load indicators support DOL screening method tests.
    They do not claim certified station loads, operational launch commit
    authority, or a vehicle-specific loads cycle.
```

---

# Provenance - `openbmp.wind.dol_persistence_knockdown.v1`

Canonical OpenBMP provenance record for the synthetic day-of-launch wind
persistence and dispersion knockdown deck used by WP-19.5 tests.

```yaml
dataset_id:       openbmp.wind.dol_persistence_knockdown.v1
files:
  - data/wind/dol-persistence-knockdown-v1.toml
  - crates/openbmp-trajopt/src/dol.rs
source_class:     synthetic-openbmp-test-fixture
source_title:     OpenBMP deterministic DOL persistence knockdown deck
source_authors:   OpenBMP contributors
source_id:        WP-19.5 synthetic fixture
publication_date: 2026-06-25
source_urls:      []
license_or_terms: Apache-2.0 OR MIT
retrieved_utc:    2026-06-25
transformation:
  method: >-
    No external range meteorology or wind-persistence statistics are
    ingested. The TOML deck pins a deterministic altitude-banded
    square-root lead-time sigma model, a synthetic sigma multiplier, and
    three synthetic truth wind-change samples used to exercise the
    fail-closed RSS knockdown substrate in `openbmp-trajopt::dol`.
  command: cargo test -p openbmp-trajopt persistence --locked
verification:
  method: >-
    In-crate tests validate the deck schema fields, non-overlapping
    altitude bands, hand-computed RSS knockdown math, monotone lead-time
    margin reduction, and synthetic truth samples staying inside the
    declared sigma envelope.
  test: cargo test -p openbmp-trajopt persistence --locked
  tolerance: exact deck-hash/provenance checks; 1.0e-12 hand-computed RSS
    agreement; synthetic truth samples within the declared 3-sigma
    envelope.
validation_status: validated-toy
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic persistence statistics support DOL commit-decision method
    tests. They do not claim site-fitted persistence models, range
    weather operations, launch authority, or a certified loads process.
```
