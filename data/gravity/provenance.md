# Provenance — `data/gravity`

Canonical OpenBMP provenance record for the WGS84 gravity-coefficient
file shipped under `data/gravity/`. See
[`docs/data-provenance.md`](../../docs/data-provenance.md) for the
contract this record satisfies.

```yaml
dataset_id:       openbmp.wgs84.gravity.j2.v1
files:
  - data/gravity/wgs84-j2.toml
  - data/gravity/wgs84-degree2-normalized-v1.toml
  - data/gravity/egm2008-zonal-degree6-normalized-v1.toml
source_class:     public-standard
source_title:     >-
  Department of Defense World Geodetic System 1984: Its Definition
  and Relationships with Local Geodetic Systems; EGM2008 public zonal
  harmonics as cited in OpenBMP source constants
source_authors:   National Imagery and Mapping Agency (NIMA), now NGA; Pavlis et al.
source_id:        NIMA TR 8350.2, 3rd Edition, Amendment 1; Pavlis et al. 2012 JGR 117 B04406
publication_date: 2000-01-03 / 2012-04-11
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
    document into a TOML file. The companion normalized degree-2
    file derives Cbar20 = -J2 / sqrt(5) from that pin and sets the
    remaining degree-2 tesseral/sectoral terms to zero as an ingestion
    fixture for the current low-degree `TesseralGravity` substrate.
    The normalized EGM2008-compatible zonal degree-6 file derives
    Cbar_n0 = -J_n / sqrt(2n + 1) from OpenBMP's already-pinned
    WGS84/EGM2008 J2-J6 constants and sets every Sbar_n0 to zero. No
    fit and no high-degree tesseral EGM2008 coefficients.
  script: none
verification:
  method: >-
    Compile-time pinned in `openbmp-core::frames` (`WGS84_A_M`,
    `WGS84_INV_FLATTENING`, `WGS84_MU_M3_S2`, `WGS84_OMEGA_RAD_S`)
    and `openbmp-physics::gravity::WGS84_J2`. Unit tests in
    `openbmp-physics::gravity::tests` exercise the constants through the
    three gravity models and parse the normalized degree-2 data pin through
    the coefficient-ingestion bridge. The same test module parses the
    normalized zonal degree-6 fixture and rebuilds `Egm2008ZonalGravity`
    through `NormalizedHarmonicField`. The `openbmp
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
zonal harmonic (J2). The normalized degree-2 fixture is a deterministic
conversion of that same J2 value for parser/ingestion coverage. Higher-degree
zonal harmonics through degree 6 are included only as a normalized-ingestion
fixture derived from OpenBMP's existing constants; high-degree tesseral EGM2008
harmonics remain out of scope for this record.

## Why public-standard, not synthetic-openbmp

WGS84 is the U.S. Department of Defense's published world geodetic
system, freely available, with a stable citation at the NGA portal.
The values used here are the *defining parameters* and the *zonal
J2* of the reference ellipsoid.

## Why the values are pinned in code as well

OpenBMP's determinism contract requires every shipped data file to
have its content hash recorded in telemetry metadata. The compiled
`openbmp-core` and `openbmp-physics` constants are the runtime source of
truth; this TOML file is the provenance pin. The `openbmp
check-provenance` walk compares the two so a typo in either file fails
CI before a release.
