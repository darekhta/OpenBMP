# Earth-Orientation Data Provenance

This directory stores small, pinned Earth-orientation fixtures used by
OpenBMP tests and parity tracking. The files are reference inputs for
deterministic frame and time-scale validation; they are not flight inputs.

## openbmp.eop.finals2000a.2017-001-004.v1

- File: `finals2000a-2017-001-004-openbmp-eop-v1.toml`
- Fixture SHA-256:
  `b26049e519db72a585118186ca2edb74e6441a86a632a5816ad4b7ab6ce8110f`
- Raw clipped source file: `finals2000a-2017-001-004.raw`
- Raw clipped source SHA-256:
  `c28e05cb04563bfc3b99adf3ea4a1400e49bbfca2742c3429035ba78c8cab88d`
- Fixture role: four-day `openbmp-eop-v1` parser and interpolation fixture
  plus raw fixed-width `finals2000A.data` ingestion fixture carrying polar
  motion, UT1-UTC, LOD, and IAU 2000A dX/dY celestial-pole offsets.
- Epoch: `2017-01-01T00:00:00Z`
- Source rows: `finals2000A.data` rows for MJD UTC `57754.00` through
  `57757.00`.
- Source URL:
  `https://datacenter.iers.org/data/latestVersion/10_FINALS.DATA_IAU2000_V2013_0110.txt`
- Source metadata URL:
  `https://datacenter.iers.org/versionMetadata.php?filename=latestVersionMeta%2F10_FINALS.DATA_IAU2000_V2013_0110.txt`
- Source format description:
  `https://maia.usno.navy.mil/ser7/readme.finals2000A`
- Retrieval date: `2026-06-18`
- Full source SHA-256 at retrieval:
  `a2e8f7dc1e616b7f0fe41f18c1953e29424f1eb9636875dfd40482e8f471f292`
- Source format SHA-256 at retrieval:
  `3efb2c610012360391aed5c057489ff74539b78494a66b7f9b342d1843db9254`
- Conversion policy: use Bulletin B final columns for `x_pole_arcsec`,
  `y_pole_arcsec`, `ut1_minus_utc_s`, `cip_offset_x_arcsec`, and
  `cip_offset_y_arcsec`; use Bulletin A LOD converted from milliseconds to
  seconds because the Bulletin B block does not include LOD.

Raw clipped source rows:

```text
17 1 1 57754.00 I  0.080504 0.000028  0.263145 0.000028  I 0.5912821 0.0000077  1.0342 0.0050  I     0.012    0.119    -0.168    0.018  0.080450  0.263074  0.5912975    -0.019    -0.057
17 1 2 57755.00 I  0.080285 0.000029  0.263605 0.000032  I 0.5901752 0.0000063  1.1730 0.0048  I    -0.001    0.040    -0.156    0.033  0.080275  0.263595  0.5902149    -0.027    -0.065
17 1 3 57756.00 I  0.080265 0.000034  0.264004 0.000031  I 0.5889406 0.0000058  1.2984 0.0042  I    -0.009    0.049    -0.150    0.059  0.080351  0.264010  0.5889684    -0.027    -0.074
17 1 4 57757.00 I  0.080001 0.000036  0.264281 0.000033  I 0.5875626 0.0000055  1.4712 0.0039  I    -0.021    0.049    -0.149    0.059  0.080144  0.264241  0.5875625    -0.013    -0.084
```

## openbmp.eop.eopc04-14-iau2000a.2017-001-004.v1

- File: `eopc04-14-iau2000a-2017-001-004.raw`
- Fixture SHA-256:
  `8b51d2fafe4b474fe6af68c6e93698f5ddd70b03a9c619d16aa5237b7bc4b389`
- Fixture role: four-day raw IERS EOP 14 C04 IAU2000A parser fixture
  carrying polar motion, UT1-UTC, LOD, and dX/dY celestial-pole offsets in
  the published C04 units.
- Epoch: `2017-01-01T00:00:00Z`
- Source rows: EOP 14 C04 IAU2000A rows for MJD UTC `57754` through
  `57757`.
- Source URL:
  `https://datacenter.iers.org/data/224/eopc04_14_IAU2000.62-now.txt`
- Source metadata URL:
  `https://datacenter.iers.org/versionMetadata.php?filename=latestVersionMeta%2F224_EOP_C04_14.62-NOW.IAU2000A224.txt`
- Retrieval date: `2026-06-18`
- Full source SHA-256 at retrieval:
  `9e26da8bc2c8490828f0c1e6ef9587b5d40716e8bac590cf3ab4149f855e5531`
- Source format: IERS metadata describes ASCII
  `3(I4),I7,2(F11.6),2(F12.7),2(F11.6),2(F11.6),2(F11.7),2(F12.6)` with
  date, MJD, x/y pole in arcseconds, UT1-UTC and LOD in seconds, dX/dY in
  arcseconds, followed by formal errors.
- Conversion policy: consume MJD, x, y, UT1-UTC, LOD, dX, and dY directly in
  the published C04 units and convert MJD UTC to scenario-relative `time_s`
  from `epoch.iso8601`.

Raw clipped source rows:

```text
2017   1   1  57754   0.080406   0.263110   0.5912977   0.0010160  -0.000041  -0.000127   0.000060   0.000045  0.0000116  0.0000140    0.000046    0.000042
2017   1   2  57755   0.080234   0.263612   0.5901980   0.0011845  -0.000051  -0.000127   0.000059   0.000044  0.0000163  0.0000138    0.000043    0.000040
2017   1   3  57756   0.080325   0.264049   0.5889489   0.0013554  -0.000060  -0.000127   0.000059   0.000044  0.0000209  0.0000138    0.000044    0.000040
2017   1   4  57757   0.080085   0.264248   0.5875560   0.0014755  -0.000069  -0.000128   0.000060   0.000045  0.0000256  0.0000139    0.000044    0.000040
```

Remaining parity gap: these fixtures prove parser reachability and pinned
real-data interpolation only. They are not a CIO transform and not ERFA
`eraDtdb` validation evidence.
