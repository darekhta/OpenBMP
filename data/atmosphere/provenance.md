# Provenance — `data/atmosphere/us_standard_1976.toml`

Canonical OpenBMP provenance record for the US Standard Atmosphere
1976 layer-table file shipped under `data/atmosphere/`. See
[`docs/data-provenance.md`](../../docs/data-provenance.md) for the
contract this record satisfies.

```yaml
dataset_id:       openbmp.atmosphere.us_standard_1976.v1
files:
  - data/atmosphere/us_standard_1976.toml
source_class:     public-standard
source_title:     U.S. Standard Atmosphere, 1976
source_authors:   >-
  National Oceanic and Atmospheric Administration (NOAA),
  National Aeronautics and Space Administration (NASA),
  United States Air Force (USAF)
source_id:        NOAA-S/T 76-1562 / NASA-TM-X-74335 (NTRS 19770009539)
publication_date: 1976-10-01
source_urls:
  - https://ntrs.nasa.gov/citations/19770009539
  - https://www.ngdc.noaa.gov/stp/space-weather/online-publications/miscellaneous/us-standard-atmosphere-1976/us-standard-atmosphere_st76-1562_noaa.pdf
license_or_terms: U.S. government technical report; public domain.
retrieved_utc:    2026-04-26
transformation:
  method: >-
    Manual transcription of the six defining constants
    (standard gravity, universal gas constant R*, mean molecular
    weight of dry air, ratio of specific heats γ, effective Earth
    radius, and the geopotential / geometric ceiling pair) and the
    seven-layer base-state table (base geopotential altitude, base
    temperature, lapse rate, base pressure) from §1.2 and table 4
    of the cited document into a TOML file. No derivation, no fit,
    no scaling. The base-pressure column was originally derived in
    the standard by chaining the barometric formulas recursively
    from the layer-0 sea-level value; the published values
    reproduced here are the canonical truth.
  script: none
verification:
  method: >-
    Compile-time pinned in
    `openbmp-env::atmosphere::us_standard_1976` (constants:
    `USSA76_G0_M_S2`, `USSA76_UNIVERSAL_GAS_CONSTANT`,
    `USSA76_MOLAR_MASS_AIR_KG_KMOL`, `USSA76_GAMMA_AIR`,
    `USSA76_REFERENCE_RADIUS_M`, `USSA76_MAX_GEOPOTENTIAL_M`,
    `USSA76_MAX_GEOMETRIC_M`; layer table: private const
    `LAYERS: [Layer; 7]`). The Phase-2.3.C regression test
    `crates/openbmp-env/tests/regression.rs` loads this TOML at
    test time via `include_str!` and asserts the in-source
    constants and per-layer base values match the pin to bit
    precision. Sea-level temperature, pressure, density, and
    speed of sound match published NOAA-S/T 76-1562 table 1
    values to better than 1e-4 relative.
  test:   crates/openbmp-env/tests/regression.rs
validation_status: validated-toy
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public US Standard Atmosphere from a U.S. government
    technical report. Idealised mean atmosphere — no operational
    mission profile, no restricted technical data, no controlled
    environmental measurements.
```

## Context

USSA76 is an idealised steady-state model of the lower atmosphere
between the sea surface and 1000 km. OpenBMP ships only the
0 – 86 km geopotential layers (the lower seven of the model's
piecewise structure) in Phase 2; the 86 km+ extension is deferred
to Phase 6 along with the rest of the atmospheric-physics work
needed for hypersonic and exoatmospheric flight regimes.

Inside the seven shipped layers the model is fully analytic:
temperature is piecewise-linear in geopotential altitude, and
pressure follows the barometric formula in two branches (gradient
layer when `L ≠ 0`, isothermal when `L = 0`). Density and speed of
sound derive from the ideal-gas law and the standard `γ` and `R/M`.

## Why public-standard, not synthetic-openbmp

USSA76 is a U.S. government joint-agency standard with a stable
NTRS citation and a freely downloadable PDF on the NOAA NGDC
publications portal. The values used here are the standard's own
defining constants and base-state table — not derived from any
fielded vehicle's atmospheric measurements, and not subject to
export control.

## Why the values are pinned in code as well

OpenBMP's determinism contract requires every shipped data file
to have its content hash recorded in telemetry metadata. The
compiled `openbmp-env` constants and layer table are the runtime
source of truth; this TOML file is the provenance pin. Phase 2.10
adds the machine check that compares the two so a typo in either
file fails CI before a release. The Phase-2.3.C regression test
performs the same comparison locally for the constants this crate
defines.

## Layer table — geopotential boundaries

| Layer | Base geopotential | Base T (K) | Lapse rate (K/m) | Base p (Pa)        |
|-------|-------------------|------------|------------------|--------------------|
| 0     | 0 m'              | 288.15     | -0.0065          | 101325.0           |
| 1     | 11 000 m'         | 216.65     | 0.0              | 22632.0639609553   |
| 2     | 20 000 m'         | 216.65     | +0.001           | 5474.88866984344   |
| 3     | 32 000 m'         | 228.65     | +0.0028          | 868.01868475551    |
| 4     | 47 000 m'         | 270.65     | 0.0              | 110.906305312205   |
| 5     | 51 000 m'         | 270.65     | -0.0028          | 66.9388733638732   |
| 6     | 71 000 m'         | 214.65     | -0.002           | 3.95639202655446   |
| top   | 84 852 m'         | 186.946    | (model ceiling)  | 0.373384 (derived) |

The top temperature value `186.946 K` is the linear-extrapolation
result `T_6 + L_6 · (84852 - 71000)` and is shown here for
information only; the model returns `OutOfEnvelope` (or, opt-in,
a zero-density vacuum sample) for queries above the ceiling.
