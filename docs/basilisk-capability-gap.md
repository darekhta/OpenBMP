# Basilisk Capability Gap Notes

This note records the Basilisk comparison that motivated the current
OpenBMP implementation slice.

## Basilisk Capabilities

- **Third-body gravity.** Basilisk's gravity-body factory can create
  multiple celestial gravity bodies and assign the complete body list
  to a spacecraft gravity field. The examples and utility docs show
  Earth plus non-central bodies such as Sun and Moon, with point-mass
  gravity or spherical harmonics on selected central bodies.
- **SPICE / JPL ephemeris.** Basilisk's `gravBodyFactory` can create a
  SPICE interface that connects gravity-body data to planet-state
  messages. Flyby examples load kernels and create bodies such as
  Earth, Sun, Moon, Venus, and Mars barycenter.
- **Multiple spacecraft.** Basilisk's spacecraft module is a reusable
  6-DOF dynamic object, and the MultiSat examples instantiate multiple
  spacecraft in one simulation for formation and station-keeping
  scenarios.

## OpenBMP Gaps Before This Slice

- OpenBMP had Earth central gravity models (`constant`, `point_mass`,
  `j2`, zonal-only `egm2008`) but no third-body perturbation term.
- OpenBMP had epoch metadata and frame profile names, but no
  time-varying celestial-body position source. `spice-reference` was a
  vocabulary placeholder rather than a consumed ephemeris profile.
- OpenBMP had deterministic separated-body lanes after
  `jettison_stage`, but a bus deploying multiple bodies on one event
  had to be represented as sequential jettisons, which changes the
  primary mass state between releases.

## Implemented In This Slice

- Added a HAL-portable `EphemerisModel` trait and
  `LowPrecisionSunMoonEphemeris` for deterministic Sun/Moon positions.
- Added `ThirdBodyGravity`, using the standard central-frame
  perturbing acceleration
  `mu_b * ((r_b - r) / |r_b - r|^3 - r_b / |r_b|^3)`.
- Added `environment.gravity = "third_body"` with explicit
  `gravity_base`, `third_bodies`, and `ephemeris` fields.
- Added `jettison_bodies`, a batch rigid-body deployment action that
  partitions multiple departing bodies from the same pre-split state.

## Still Missing Relative To Basilisk

- JPL DE / SPICE file ingestion is not implemented. OpenBMP now has the
  ephemeris trait boundary and a deterministic analytical provider, but
  not a BSP/SPK kernel reader.
- Time-varying Earth orientation, polar motion, leap-second tables, and
  IERS EOP ingestion remain deferred behind the existing
  `spice-reference` and `iers-tabulated` vocabulary.
- Multi-body OpenBMP propagation is still independent-lane rigid-body
  propagation after deployment. Basilisk's message-passing architecture
  supports many simultaneously configured spacecraft modules more
  generally.

## References

- Basilisk `scenarioBasicOrbit` documentation notes multi-body gravity
  lists and SPICE-updated planet ephemerides:
  https://hanspeterschaub.info/bskOlderDocs/bsk_1_4_2/_modules/scenarioBasicOrbit.html
- Basilisk `simIncludeGravBody` utility documentation describes
  `createMoon` and `createSpiceInterface`:
  https://hanspeterschaub.info/bskOlderDocs/bsk_2_1_7/Documentation/utilities/simIncludeGravBody.html
- Basilisk `scenarioFlybySpice` shows Earth/Sun/Moon body creation and
  SPICE kernel loading:
  https://hanspeterschaub.info/bskOlderDocs/bsk_2_1_7/_modules/scenarioFlybySpice.html
- Basilisk MultiSat station-keeping documentation describes a
  three-spacecraft formation simulation:
  https://avslab.github.io/basilisk/examples/MultiSatBskSim/scenariosMultiSat/scenario_StationKeepingMultiSat.html
- NASA NAIF's SPICE concept page describes SPK ephemerides and related
  spacecraft/planet/instrument kernel data:
  https://naif.jpl.nasa.gov/naif/spiceconcept.html
