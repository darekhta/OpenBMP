# Provenance — `data/gravity/wgs84-j2.toml`

Canonical OpenBMP provenance record for the WGS84 gravity-coefficient
file shipped under `data/gravity/`. See
[`docs/data-provenance.md`](../../docs/data-provenance.md) for the
contract this record satisfies.

```yaml
dataset_id:       openbmp.wgs84.gravity.j2.v1
files:
  - data/gravity/wgs84-j2.toml
source_class:     public-standard
source_title:     >-
  Department of Defense World Geodetic System 1984: Its Definition
  and Relationships with Local Geodetic Systems
source_authors:   National Imagery and Mapping Agency (NIMA), now NGA
source_id:        NIMA TR 8350.2, 3rd Edition, Amendment 1
publication_date: 2000-01-03
source_urls:
  - https://earth-info.nga.mil/index.php?dir=wgs84&action=wgs84
  - https://gis-lab.info/docs/nima-tr8350.2-wgs84fin.pdf
license_or_terms: U.S. government technical report; public domain.
retrieved_utc:    2026-04-26
transformation:
  method: >-
    Manual transcription of the four defining-parameter values
    (semi-major axis, inverse flattening, gravitational parameter,
    angular velocity) and the J2 zonal harmonic from the cited
    document into a TOML file. No derivation, no fit, no scaling.
  script: none
verification:
  method: >-
    Compile-time pinned in `openbmp-core::frames` (`WGS84_A_M`,
    `WGS84_INV_FLATTENING`, `WGS84_MU_M3_S2`, `WGS84_OMEGA_RAD_S`)
    and `openbmp-physics::gravity::WGS84_J2`. Unit tests in
    `openbmp-physics::gravity::tests` exercise the constants through the
    three gravity models. The `openbmp
    check-provenance` walk additionally cross-checks the in-source
    values against this TOML pin.
  test:   crates/openbmp-physics/src/gravity.rs
validation_status: validated-toy
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public WGS84 constants from a U.S. government technical report.
    No operational vehicle parameters; no restricted technical data.
```

## Context

These constants describe the WGS84 reference ellipsoid and its
gravitational field. They are *frame primitives* (semi-major axis,
inverse flattening, GM, angular velocity) plus the second-degree
zonal harmonic (J2). Higher-degree zonal and tesseral harmonics
(EGM2008 truncated) are out of scope — the gravity model uses only J2.

## Why public-standard, not synthetic-openbmp

WGS84 is the U.S. Department of Defense's published world geodetic
system, freely available, with a stable citation at the NGA portal.
The values used here are the *defining parameters* and the *zonal
J2* of the reference ellipsoid. They are not derived from any
fielded vehicle's parameters and they are not subject to export
control.

## Why the values are pinned in code as well

OpenBMP's determinism contract requires every shipped data file to
have its content hash recorded in telemetry metadata. The compiled
`openbmp-core` and `openbmp-physics` constants are the runtime source of
truth; this TOML file is the provenance pin. The `openbmp
check-provenance` walk compares the two so a typo in either file fails
CI before a release.
