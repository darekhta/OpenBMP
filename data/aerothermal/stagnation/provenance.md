# Aerothermal Stagnation Data Provenance

## `data/aerothermal/stagnation/detra-kemp-riddell-v1.toml`

- dataset id: `detra-kemp-riddell-synthetic-conversion-v1`
- validation label: `checked`
- source class: synthetic unit-conversion fixture for the classic
  Detra-Kemp-Riddell imperial-reference stagnation-heating form.
- purpose: Pin the WP-04.1-b DKR SI wrapper and hot-wall correction
  arithmetic.
- method: The fixture records the handbook-style DKR reference form using
  `17600 Btu/(ft^2 s)`, `26000 ft/s`, velocity exponent `3.15`, and
  density normalized by sea-level density, then checks conversion to SI
  and hot-wall correction behavior.
- scope boundary: deterministic unit conversion and fail-closed
  arithmetic only. This is not a flight, ballistic-range, or CFD
  validation case.
