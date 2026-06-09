# Sounding piecewise-exponential atmosphere demo provenance

## Scenario

`scenarios/sounding-piecewise-exp-atmosphere/scenario.toml`

## Source class

Synthetic. The scenario is the end-to-end demo for the
layered piecewise-exponential atmosphere model
(`openbmp_physics::PiecewiseExponentialAtmosphere`).

A 1 kg point mass launched vertically (+z ECI) with `v_z = 2000 m/s`
under constant gravity follows a closed-form ballistic profile:

```
apogee        = v² / (2 g) = 2000² / 19.6133 ≈ 204 km
time to apogee = v / g    ≈ 204 s
total flight   ≈ 410 s
```

`stop_s = 410` is the upper bound; the runner's automatic
constant-gravity ground-impact detector stops the run at about 407.9 s
as the ballistic arc crosses z = 0. No aerodynamic drag is applied
(`forces = ["gravity"]`); the closed-form ballistic answer is
therefore independent of the layered-atmosphere sampling. The scenario
exists purely to exercise the runner's
per-step atmosphere telemetry channels with the new layered model
across the 0-204 km altitude span — that span deliberately exceeds
the 86 km USSA76 ceiling to demonstrate the piecewise-exponential
model fills the documented USSA76 gap above its native envelope.

The Vallado-style table used by the model pins density and scale
height. Reported pressure, temperature, and speed of sound are
scale-height-effective values derived for ideal-gas self-consistency;
they are telemetry sanity channels for this engineering fit, not
source-tabulated thermospheric temperature measurements.

## License / restrictions

Synthetic OpenBMP data. No real fielded-vehicle parameters, no
real-world locations, no ITAR / EAR / MTCR / Wassenaar content.

## References

- Vallado, D. A. (2013). *Fundamentals of Astrodynamics and
  Applications*, 4th ed., Table 8-4 ("Exponential Atmosphere
  Model"). Microcosm Press / Springer.
- Curtis, H. D. (2014). *Orbital Mechanics for Engineering Students*,
  3rd ed., Appendix D.
- Wertz, J. R. and Larson, W. J. (1999). *Space Mission Analysis and
  Design*, 3rd ed., §8.1.4. Microcosm Press.
- US Standard Atmosphere 1976 supplemental reference profile
  (NOAA-S/T 76-1562 Part 2) — underlying source for the layered fit.

## Validation status

`experimental`. Validates parser-side and runs end-to-end
deterministically. The e2e test
(`crates/openbmp-cli/tests/sounding_piecewise_exp_atmosphere_e2e.rs`)
asserts:

- the scenario completes 4100 RK4 steps with end-time stop;
- the per-step atmosphere telemetry channels are populated with
  finite, positive density / pressure / temperature / speed-of-sound
  values throughout the flight;
- the maximum sampled density (at z = 0) matches the layered
  model's sea-level base value (1.225 kg/m³);
- the minimum sampled density (at apogee, z ≈ 204 km) lies in the
  documented LEO engineering envelope for ρ at 200-250 km altitude;
- two reruns produce byte-identical Parquet (the layered model is
  deterministic: pure `f64` arithmetic, locked operand order, no
  FMA, no system RNG).

Math-side coverage (the unit tests in
`crates/openbmp-physics/src/atmosphere/piecewise_exponential.rs`)
verifies that:

- the model rejects non-finite altitude with `NonFinite` and
  sub-zero altitude with `OutOfEnvelope`;
- the exoatmospheric policy (`FailClosed` / `ZeroDensityAboveCeiling`)
  controls the above-1000 km behaviour cleanly;
- density is monotonically non-increasing within each layer and
  across layer boundaries;
- pressure / temperature / speed-of-sound are finite and positive
  throughout the validity envelope;
- the layered fit reproduces the tabulated base-density value at
  every layer breakpoint;
- the model is byte-stable across reruns;
- ρ(400 km) lands in the engineering envelope of the published
  exponential-atmosphere references.

## Scope notes

Academic atmosphere-model verification scenario. The vertical
ballistic profile is a scenario-internal trajectory that demonstrates
the atmosphere-model surface; no geodetic site data is used.
