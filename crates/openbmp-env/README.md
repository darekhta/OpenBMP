# openbmp-env

L2 environment models.

**Status:** Phase 2 — gravity (`ConstantGravity`, `PointMassGravity`,
`J2Gravity`), atmosphere (`IsothermalAtmosphere`, `UsStandard1976`),
and wind (`NoWind`, `ConstantWind`) shipped. Layered / gust wind
lands in Phase 3; NRLMSISE-00 atmosphere and EGM truncated gravity
land in Phase 6.

## Purpose

- `EnvironmentModel` trait per `docs/software-architecture.md`.
- Atmosphere: `IsothermalAtmosphere` (toy), `UsStandard1976` (Phase 2),
  `Nrlmsise00` (Phase 6.1).
- Gravity: `ConstantGravity`, `PointMassGravity`, `J2Gravity`,
  `EgmTruncated` (Phase 6).
- Wind: `NoWind`, `ConstantWind`, `LayeredWind`, `GustWind`.
- Magnetic field: stubs for Phase 5+.

## Inputs and Outputs

`EnvironmentQuery` (time, position) → `EnvironmentSample` (atmosphere,
gravity, wind, magnetic field).

## Units and Frames

Inputs in `ECI` for queries. Atmosphere outputs frame-neutral. Wind in
`NED`. Magnetic field in `NED`. Gravity in `ECEF`.

## Assumptions

Environment models are public, synthetic, or textbook implementations with
pinned coefficient tables. Models do not fetch live space-weather,
atmosphere, or Earth-orientation data during simulation.

## Validity Range

Each atmosphere, gravity, wind, and magnetic model declares altitude,
latitude/longitude, time, and coefficient validity ranges. Invalid
out-of-range use fails closed unless a toy model documents otherwise.

## Determinism

Pure functions of `(time, position)` and pinned coefficient tables.
No wall-clock, no network, no system RNG.

## Validation

`validated-toy` for the shipped Phase-2.2/2.3/2.4 surface. WGS84 J2
coefficient pinned per NIMA TR 8350.2 with provenance file under
`data/gravity/`. USSA76 validated against per-kilometre regression
to the published table within 1e-6 relative across the 0–86 km
envelope, with layer-boundary continuity to ULP. Wind models covered
by property tests on `ConstantWind` finiteness.

## Data Provenance

US Standard Atmosphere 1976 — public NASA/NTRS document.
NRLMSISE-00 — public NASA CCMC Fortran source. EGM coefficients —
public NGA releases.

All shipped data ships with `provenance.md` per
`docs/data-provenance.md`.

## Safety Boundary

No operational vehicle parameters. Generic public/educational physics
only.
