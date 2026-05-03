# LEO orbit EGM2008-zonal demo provenance

## Scenario

`scenarios/leo-orbit-egm2008/scenario.toml`

## Source class

Synthetic. The scenario is the Phase-5.C.2 end-to-end demo for the
truncated EGM2008 zonal-harmonic gravity model
(`openbmp_physics::Egm2008ZonalGravity`, degrees 2-6).

The initial state is the closed-form circular-orbit velocity for the
WGS84 µ at radius `r = 6_778_000 m` (≈ 400 km altitude), inclined 30°
relative to the equatorial plane:

```
v_circ = sqrt(WGS84_MU_M3_S2 / r)   (WGS84_MU_M3_S2 from
                                     `crates/openbmp-physics/src/frames.rs`,
                                     value pinned to NIMA TR 8350.2)
       ≈ 7669.103 m/s

v_y = v_circ · cos(30°) ≈ 6_641.109 m/s
v_z = v_circ · sin(30°) ≈ 3_834.553 m/s
```

The 30° inclination is selected so that the trajectory crosses
non-equatorial latitudes and exercises the higher-order odd zonal
terms (J_3, J_5) that vanish identically on the equatorial plane.
The orbit period under pure Newtonian gravity is

```
T = 2π · r / v_circ ≈ 5556.16 s.
```

`stop_s = 5556` was chosen as one nominal orbit period; the EGM2008
zonal corrections introduce slight oblateness-driven precession that
the math-side unit tests in `crates/openbmp-physics/src/gravity.rs`
already characterise (`egm2008_zonal_higher_degrees_change_acceleration`).

The integrator is RK4 fixed-step at dt = 1.0 s, which under this model
holds relative energy drift well below 1e-9 over one orbit.

## License / restrictions

Synthetic OpenBMP data. No real fielded-vehicle parameters, no
real-world locations, no ITAR / EAR / MTCR / Wassenaar content.

## References

- Pavlis, N. K., Holmes, S. A., Kenyon, S. C., and Factor, J. K.
  (2012). *The development and evaluation of the Earth Gravitational
  Model 2008 (EGM2008)*. Journal of Geophysical Research, 117, B04406.
  doi:10.1029/2011JB008916. Public NGA-published EGM2008 zonal
  coefficients used to pin `EGM2008_J3..J6` constants in
  `crates/openbmp-physics/src/gravity.rs`.
- Vallado, D. A. (2013). *Fundamentals of Astrodynamics and
  Applications*, 4th ed., §8.6 ("The Two-Body Problem with Earth
  Oblateness"). Microcosm Press / Springer. Reference for the J_n
  Cartesian-gradient form used by the model.
- Montenbruck, O. and Gill, E. (2000). *Satellite Orbits — Models,
  Methods, Applications*, §3.2. Springer. Reference Legendre-recurrence
  derivation of zonal-harmonic gradients.

## Validation status

`experimental`. Validates parser-side and runs end-to-end
deterministically. The Phase-5.C.2 e2e test
(`crates/openbmp-cli/tests/leo_orbit_egm2008_e2e.rs`) asserts:

- the scenario completes 5556 kernel steps with end-time stop;
- the final ECI radius stays within 5 km of the initial radius
  (the EGM2008 zonal corrections induce a small periodic radial
  perturbation; the orbit does not blow up);
- two reruns produce byte-identical Parquet (the gravity model is
  deterministic: pure `f64` arithmetic with locked operand order, no
  FMA, no system RNG).

Math-side coverage (the unit tests in
`crates/openbmp-physics/src/gravity.rs`) verifies that:

- the truncated zonal model degenerates to byte-identical
  `J2Gravity` output when configured with `degree = 2` and
  `J_3..J_6 = 0`;
- the higher-degree (`degree = 6`) accelerations differ from the
  degree-2-only baseline at non-equatorial points;
- the Legendre-polynomial recurrence matches closed-form `P_n(ξ)`
  for low degrees;
- the model handles the singular origin and pole geometries cleanly
  (no NaN, finite radial acceleration, x and y components vanish at a
  pure-z position).

## Safety boundary

`docs/safety-boundaries.md` accept list — academic gravity-model
verification scenario, no guidance / navigation / control logic, no
target geometry, no real-world locations. The orbit altitude and
inclination are scenario-internal numbers that demonstrate the
gravity-model surface, not parameters of any fielded asset.
