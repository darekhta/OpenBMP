# Aerothermal Data Provenance

## `data/aerothermal/radiative/tauber-sutton-earth-v1.toml`

- dataset id: `tauber-sutton-earth-air-radiative-v1`
- validation label: `checked`
- payload SHA-256:
  `06d66cbd2ee2fa70ebcfbf9cf7f0aedee727dc06bc79d90d2c0a01936342b870`
- purpose: Pin the WP-04.1-a Tauber-Sutton Earth-air stagnation-point
  radiative-heating coefficient set and tabulated velocity function used by the
  opt-in `TauberSuttonRadiative::earth_air()` model.
- method: Publicly reproduced Tauber-Sutton Earth-entry relation,
  `q_rad[W/cm^2] = 47360 * R_n^a * rho_inf^1.22 * f(V)`, with the radius
  exponent law and tabulated `f(V)` values from 10 to 16 km/s. OpenBMP adds an
  explicit 9 km/s zero floor sentinel because the engineering model is
  intentionally inactive below that radiative-heating regime.
- scope boundary: This fixture proves deterministic table interpolation,
  envelope rejection, unit conversion to W/m^2, and total
  `q_conv + q_rad` composition. It does not claim Apollo-4, FIRE-II, NEQAIR, or
  coupled radiation-transport validation.
- references: Tauber & Sutton (JSR 1991, doi:10.2514/3.26206); the public
  relation/table reproduction cited in the TOML retrieval path.

## `data/aerothermal/transition/pant-reda-thresholds-v1.toml`

- dataset id: `pant-reda-roughness-transition-synthetic-v1`
- validation label: `checked`
- purpose: Pin the WP-04.1-b engineering thresholds used by the smooth-wall
  `Re_theta/M_e` and PANT/Reda rough-wall transition tests.
- method: Synthetic fixture. The default smooth-wall onset is `Re_theta/M_e =
  200`, the PANT/Reda rough-wall onset indicator is `Re_theta * (T_e/T_w) *
  (k/theta) = 215`, and the deterministic completion ratio is `1.25`.
- scope boundary: This fixture is not a wind-tunnel, ballistic-range, or
  flight-correlation benchmark. Sharp-cone transition-location closure is
  covered by `sharp-cone-transition-v1.toml`.
- references: Anderson PANT engineering criterion and Reda roughness-dominated
  transition synthesis are tracked in `docs/parity/04-aerothermal-realgas-and-tps.md`.

## `data/aerothermal/transition/sharp-cone-transition-v1.toml`

- dataset id: `sharp-cone-re-theta-transition-location-v1`
- validation label: `validated-toy`
- purpose: Pin the WP-04.1-b sharp-cone smooth-wall transition-location fixture
  used by the `Re_theta/M_e` transition test.
- method: Analytic method fixture. It reconstructs the transition station from
  `Re_theta/M_e = 200`, `M_e = 6`, unit Reynolds number `1.0e7 1/m`, and
  laminar momentum-thickness relation `Re_theta = 0.664 * sqrt(Re_x)`. The
  deterministic completion station uses the OpenBMP 1.25 ramp ratio.
- scope boundary: This fixture proves the location arithmetic and fail-closed
  model wiring for a sharp-cone-style laminar momentum-thickness reference. It
  is not a wind-tunnel, ballistic-range, flight, or e^N stability validation.
- references: Bertin/Anderson-style `Re_theta/M_e` engineering transition
  thresholds are tracked in `docs/parity/04-aerothermal-realgas-and-tps.md`.

## `data/aerothermal/stagnation/detra-kemp-riddell-v1.toml`

- dataset id: `detra-kemp-riddell-synthetic-conversion-v1`
- validation label: `checked`
- purpose: Pin the WP-04.1-b Detra-Kemp-Riddell SI wrapper and hot-wall
  correction arithmetic.
- method: Synthetic fixture. The reference form is the handbook DKR
  stagnation-heating correlation using `17600 Btu/(ft^2 s)`, `26000 ft/s`,
  exponent `3.15`, and density normalized by sea-level density. The test point
  converts the imperial cold-reference heat flux to SI and applies the
  `(h_aw - h_w) / (h_aw - h_w,300K)` hot-wall correction.
- scope boundary: This fixture proves deterministic unit conversion and
  fail-closed arithmetic only. It is not a flight, ballistic-range, or
  CFD validation case.
- references: Detra-Kemp-Riddell and Sutton-Graves sources are tracked in
  `docs/parity/04-aerothermal-realgas-and-tps.md`.
