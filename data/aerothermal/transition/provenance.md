# Aerothermal Transition Data Provenance

## `data/aerothermal/transition/pant-reda-thresholds-v1.toml`

- dataset id: `pant-reda-roughness-transition-synthetic-v1`
- validation label: `checked`
- source class: synthetic engineering-threshold fixture.
- purpose: Pin the WP-04.1-b smooth-wall `Re_theta/M_e` onset,
  PANT/Reda roughness indicator onset, and deterministic transition
  completion ratio used by the boundary-layer transition tests.
- scope boundary: this is not a wind-tunnel, ballistic-range, or flight
  correlation benchmark.

## `data/aerothermal/transition/sharp-cone-transition-v1.toml`

- dataset id: `sharp-cone-re-theta-transition-location-v1`
- validation label: `validated-toy`
- source class: analytic method fixture.
- purpose: Pin the WP-04.1-b sharp-cone smooth-wall transition-location
  fixture reconstructed from `Re_theta/M_e`, edge Mach, unit Reynolds
  number, and a laminar momentum-thickness relation.
- scope boundary: this proves transition-location arithmetic and
  fail-closed model wiring only. It is not a wind-tunnel,
  ballistic-range, flight, or e^N stability validation.
