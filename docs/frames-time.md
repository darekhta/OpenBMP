# OpenBMP Frames and Time

OpenBMP uses frame-tagged types and explicit simulation time to prevent common
flight-dynamics mistakes. This document pins the conventions that are only
summarized in the architecture document.

The implementation uses simplified transforms, but the names
and metadata leave room for IERS-grade Earth orientation without
renaming public APIs.

## Frame Policy

All state propagation is expressed in an inertial frame. Body-fixed quantities
are expressed in the vehicle body frame. Local navigation frames are derived
views, not canonical storage.

| OpenBMP name | Meaning | Notes |
|---|---|---|
| `ECI` | Earth-centered inertial frame | Simplified inertial frame; future profile may map to GCRF/J2000 |
| `ECEF` | Earth-centered Earth-fixed frame | WGS84-aligned rotating Earth frame |
| `Body` | Vehicle body frame | Origin and axes defined by each vehicle model |
| `NED` | Local north-east-down frame | Derived from geodetic origin and ECEF |
| `ENU` | Local east-north-up frame | Derived from geodetic origin and ECEF |

Every telemetry column that stores a vector must declare its frame. Every
scenario field that stores a vector must include the frame in the field name
or model schema.

## Earth Model

The default Earth model is WGS84:

- Ellipsoid constants come from the public NGA WGS84 standard.
- Geodetic latitude, longitude, and height are defined relative to WGS84.
- Gravity models may use their own documented constants, but the scenario
  records the selected gravity model version.

A spherical Earth helper is available for analytic toys, but such scenarios
must label the model as `spherical_earth_toy` and must not mix spherical and
WGS84 geodetic fields silently.

## Time Scales

`SimTime` is seconds since scenario start and is the only time value that the
deterministic kernel advances. It is not UTC and it is not a wall clock.

Absolute time is optional scenario metadata:

```toml
[epoch]
scale = "UTC"
iso8601 = "2026-01-01T00:00:00Z"
leap_second_table = "data/time/leap_seconds_2026a.toml"
leap_second_table_sha256 = "<64 hex chars>"
eop = "data/earth_orientation/iers_bulletin_b_2026_01.toml"
eop_sha256 = "<64 hex chars>"
```

Rules:

- Scenarios without `[epoch]` use deterministic toy or uniform-rotation
  transforms only.
- Scenarios with `[epoch]` must name the time scale and source tables
  required by their selected frame profile.
- UTC is used only at input/output boundaries.
- The current `iers-tabulated` runner path requires `epoch.scale = "UTC"` and
  converts UTC to UT1 using a pinned EOP table.
- Ephemeris paths convert `UTC` or `TT` epochs onto the TDB axis. UTC
  conversion uses the pinned leap-second table; SPK UTC epochs require it.
- Leap-second and Earth-orientation tables are data files and require
  provenance.

## Frame Profiles

OpenBMP should support explicit frame profiles:

| Profile | Purpose | Determinism |
|---|---|---|
| `toy-fixed-earth` | Textbook scenarios, no absolute epoch | Bit-stable |
| `wgs84-uniform-rotation` | Earth rotation with constant rate, no EOP | Bit-stable |
| `iers-tabulated` | Pinned UT1-UTC and polar motion table | Bit-stable within platform profile when data is pinned |
| `spice-reference` | Validation against public SPICE kernels | Validation only, not default runtime |

The default profile for MVP scenarios is `wgs84-uniform-rotation` unless an
analytic toy states otherwise.

`iers-tabulated` consumes deterministic TOML EOP tables:

```toml
format = "openbmp-eop-v1"

[[samples]]
time_s = 0.0
ut1_minus_utc_s = 0.102
x_pole_arcsec = 0.045
y_pole_arcsec = 0.312
```

Sample times are scenario-relative seconds and must cover the whole
simulation interval. The transform applies compact IAU 1976 mean precession
from J2000 to date, computes IAU Earth Rotation Angle from the UTC scenario
epoch plus interpolated UT1-UTC, then applies polar motion. Nutation and SPICE
frame chains remain future high-fidelity work.

## Transform Rules

Frame transforms must be:

- Explicit in code and scenario metadata.
- Constructed from an immutable `FrameContext` at scenario start.
- Pure functions of `SimTime`, pinned constants, and pinned data tables.
- Logged to telemetry metadata by profile and data version.

No model may query the operating-system clock or download Earth-orientation
data during a run.

## Local Frames

`NED` and `ENU` require a local geodetic origin:

```toml
[frames.local_origin]
latitude_deg = 32.9903
longitude_deg = -106.9746
height_m = 0.0
source = "synthetic-example"
```

Local origins are accepted for academic scenarios and validation cases. They
must not be described as targets, aimpoints, strike points, terminal points, or
payload-delivery objectives.

For WGS84 local frames, `NED +down` is anti-parallel to the outward geodetic
normal at the declared latitude and longitude. It is not generally the same as
the geocentric radial direction toward Earth's centre; the two differ at
non-equatorial latitudes on the ellipsoid.

## Telemetry Metadata

Telemetry archives must record:

- Frame profile.
- Earth model.
- Gravity model.
- Absolute epoch, if present.
- Leap-second table ID, if present.
- Earth-orientation table ID, if present.
- Transform implementation version.

Consumers that do not understand the frame metadata should refuse to load the
archive rather than silently assuming a frame.

## Public References

Use these as public reference material and validation sources:

- IERS Conventions (2010), IERS Technical Note 36.
- IERS Bulletins A, B, and C for EOP and leap-second data.
- NGA WGS84 and public EGM coefficient releases.
- NASA NAIF SPICE documentation and generic kernels for validation.
- Vallado, *Fundamentals of Astrodynamics and Applications*.

OpenBMP reimplements the required transforms in Rust. It does not import
SPICE, Orekit, or other external runtime libraries into the deterministic
kernel.
