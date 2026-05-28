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
- Added `at_relative_distance` mission triggers so post-deployment
  events can observe ranges between detached rigid-body lanes and the
  primary bus, or between two detached lanes.
- Added a shipped `bus-rv-deployment` scenario and CLI e2e coverage
  for a bus releasing two rigid lanes on one event tick, then firing a
  relative-distance clearance marker.
- Added `frame_profile = "iers-tabulated"` with pinned TOML EOP table
  ingestion for interpolated UT1-UTC and polar motion.
- Added `environment.ephemeris = "spk"` with SHA-256-pinned binary
  DAF/SPK ingestion for ordered JPL DE-style kernel lists with type
  2/3 Chebyshev segments in J2000. The ephemeris API now exposes
  Earth-centered body state as position plus velocity; SPK type 2
  velocities use the Chebyshev position derivative and type 3
  velocities use the stored velocity coefficients.
- Added SPK type 1 modified-difference-array segments, including record
  epoch selection and the Krogh difference-line position/velocity
  evaluator used by legacy NAVIO-style SPKs.
- Added SPK type 9 unequal-time Lagrange state interpolation for
  geometric position/velocity segments, extending the parser beyond
  Chebyshev-only planetary kernels toward mission-spacecraft SPK
  shapes.
- Added SPK type 10 TLE/SGP4 segments, including fixed-packet generic
  segment parsing, NAIF-style closest-epoch packet selection and cosine
  blending, and TEME-to-J2000 state rotation through the compact
  IAU 1976/1980 frame helpers.
- Added SPK type 8 equal-time Lagrange and type 12/13 Hermite state
  interpolation, covering the fixed-record discrete-state segment
  family commonly used for mission kernels before the remaining generic
  and analytic SPK segment types.
- Added SPK type 14 generic non-uniform Chebyshev position/velocity
  segments, using the fixed-packet generic-segment metadata layout
  produced by NAIF's type-14 writer.
- Added SPK type 5 discrete-state segments, propagating bracketing
  records with a two-body universal-variable solver and applying the
  NAIF cosine blend between propagated states.
- Added SPK type 15 precessing-conic segments, including the
  two-body periapsis propagation path and NAIF's optional J2 node /
  apsis precession flags.
- Added SPK type 17 equinoctial-element segments, evaluating the
  NAIF-style equinoctial Kepler equation, element rates, and reference
  plane pole transform.
- Added SPK type 20 Chebyshev velocity-only segments, integrating the
  velocity polynomials and midpoint position constants used by
  EPM-style ephemerides.
- Added SPK type 21 extended modified-difference-array segments, sharing
  the type-1 evaluator with variable per-component table dimensions.
- Added SPK type 18 ESOC/DDID packet interpolation for subtype 0
  Hermite packets and subtype 1 Lagrange state packets.
- Added SPK type 19 ESOC/DDID piecewise interpolation, including
  mini-segment boundary selection and subtype 0/1/2 packet evaluators.
- Added SPK support for the built-in `ECLIPJ2000` inertial frame,
  rotating those segment states into OpenBMP's J2000 ECI chain.
- Added pinned leap-second table ingestion for deterministic UTC -> TT
  -> TDB and TT -> TDB ephemeris epoch conversion. SPK ephemerides can
  now use `TDB`, `TT`, or `UTC` epochs, with UTC requiring the table.
- Added NAIF `KPL/LSK` leap-second text-kernel ingestion through the
  pinned `epoch.leap_second_table` path, so UTC SPK epochs can use the
  standard SPICE leap-second kernel format.
- Added compact IAU 1976 mean precession in the `iers-tabulated`
  ECI/ECEF path before Earth rotation and polar motion.
- Added IAU 1980 nutation in the `iers-tabulated` celestial-frame path,
  using the 106-term lunisolar series before Earth rotation.

## Still Missing Relative To Basilisk

- SPK ingestion is intentionally limited to geometric state chains from
  binary SPK/BSP kernels. It does not yet implement light-time
  correction, stellar aberration, generic text kernels beyond NAIF LSK
  leap-second files, non-J2000 frame transforms beyond built-in
  `ECLIPJ2000`, or a full SPICE frame-kernel chain.
- The IERS path is intentionally compact: it does not yet implement a
  full SPICE frame chain or IAU 2006/2000A CIO-based transforms.
  Velocity transport now includes the finite-difference rate of the full
  compact J2000-to-ECEF orientation chain.
- Multi-body OpenBMP propagation is still independent-lane rigid-body
  propagation after deployment. Relative-distance triggers can observe
  lane geometry, but Basilisk's message-passing architecture supports
  many simultaneously configured spacecraft modules more generally.

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
- NASA NAIF SPK Required Reading describes binary SPK files, segment
  precedence, and state retrieval concepts:
  https://naif.jpl.nasa.gov/pub/naif/toolkit_docs/C/req/spk.html
- NASA NAIF DAF Required Reading describes the binary file
  architecture used by SPK, CK, and binary PCK kernels:
  https://naif.jpl.nasa.gov/pub/naif/toolkit_docs/FORTRAN/req/daf.html
- NASA NAIF Frames Required Reading lists built-in inertial frames such
  as `J2000` and `ECLIPJ2000` and distinguishes them from FK/PCK/CK
  frame chains:
  https://naif.jpl.nasa.gov/pub/naif/toolkit_docs/FORTRAN/req/frames.html
- NASA NAIF generic kernels include the current leap-second kernel
  (`LSK`) used by SPICE time conversion workflows:
  https://naif.jpl.nasa.gov/naif/data_generic.html
