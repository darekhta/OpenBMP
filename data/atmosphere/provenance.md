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
source_hash_sha256: 820a1d7c6bd533e3a845696601cc74013d7d851babac4dab35bee3b1feae35cf
retrieved_utc:    2026-04-26
transformation:
  method: >-
    Manual transcription of the defining constants
    (standard gravity, universal gas constant R*, mean molecular
    weight of dry air, ratio of specific heats γ, effective Earth
    radius, and the geopotential / geometric ceiling pair) and the
    seven-layer structure (base geopotential altitude, base
    temperature, lapse rate) from §1.2 and table 4
    of the cited document into a TOML file. No fit and no scaling.
    The base-pressure column is regenerated from the layer-0
    sea-level value by chaining the standard barometric formulas
    recursively with the same locked f64 operand order used by the
    Rust model.
  script: none
verification:
  method: >-
    Compile-time pinned in
    `openbmp-physics::atmosphere::us_standard_1976` (constants:
    `USSA76_G0_M_S2`, `USSA76_UNIVERSAL_GAS_CONSTANT`,
    `USSA76_MOLAR_MASS_AIR_KG_KMOL`, `USSA76_GAMMA_AIR`,
    `USSA76_REFERENCE_RADIUS_M`, `USSA76_MAX_GEOPOTENTIAL_M`,
    `USSA76_MAX_GEOMETRIC_M`; layer table: private const
    `LAYERS: [Layer; 7]`). The regression test
    `crates/openbmp-physics/tests/regression.rs` loads this TOML at
    test time via `include_str!` and asserts the in-source
    constants and per-layer base values match the pin to bit
    precision. In-crate tests also recompute each next layer's base
    pressure from the previous layer and assert bit equality, check
    per-kilometre TOML-derived reference samples, and verify
    sea-level temperature, pressure, density, and speed of sound
    against NOAA-S/T 76-1562 table 1 values.
  test:   crates/openbmp-physics/tests/regression.rs
  tolerance: >-
    Constants and layer-base pins: bit equality after TOML parse.
    Layer-pressure recurrence: bit equality on the reference platform
    profile. Per-kilometre reference samples: 1e-11 relative for
    pressure and density, 1e-12 absolute for temperature. Sea-level
    table-1 anchors: exact for T/p display values, 1e-6 kg/m^3 for
    density, 5e-4 m/s for speed of sound.
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
piecewise structure); the 86 km+ extension is out of scope,
along with the rest of the atmospheric-physics work
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
defining constants and layer structure, with base pressures
regenerated from the standard barometric formulas — not derived from
any fielded vehicle's atmospheric measurements, and not subject to
export control.

## Why the values are pinned in code as well

OpenBMP's determinism contract requires every shipped data file
to have its content hash recorded in telemetry metadata. The
compiled `openbmp-physics` constants and layer table are the runtime
source of truth; this TOML file is the provenance pin. The `openbmp
check-provenance` walk compares the two so a typo in either
file fails CI before a release. The regression test
performs the same comparison locally for the constants this crate
defines.

## Layer table — geopotential boundaries

| Layer | Base geopotential | Base T (K) | Lapse rate (K/m) | Base p (Pa)        |
|-------|-------------------|------------|------------------|--------------------|
| 0     | 0 m'              | 288.15     | -0.0065          | 101325.0           |
| 1     | 11 000 m'         | 216.65     | 0.0              | 22632.06397346291  |
| 2     | 20 000 m'         | 216.65     | +0.001           | 5474.888669677775  |
| 3     | 32 000 m'         | 228.65     | +0.0028          | 868.0186847552282  |
| 4     | 47 000 m'         | 270.65     | 0.0              | 110.90630555496611 |
| 5     | 51 000 m'         | 270.65     | -0.0028          | 66.9388731186874   |
| 6     | 71 000 m'         | 214.65     | -0.002           | 3.9564204280407327 |
| top   | 84 852 m'         | 186.946    | (model ceiling)  | 0.373383589976216  |

The top temperature value `186.946 K` is the linear-extrapolation
result `T_6 + L_6 · (84852 - 71000)` and is shown here for
information only; the model returns `OutOfEnvelope` (or, opt-in,
a zero-density vacuum sample) for queries above the ceiling.

---

# Provenance — `openbmp.atmosphere.nrlmsise00.static.v1`

Canonical OpenBMP provenance record for the NRLMSISE-00
static-defaults table pinned in
`crates/openbmp-physics/src/atmosphere/nrlmsise00.rs`.

```yaml
dataset_id:       openbmp.atmosphere.nrlmsise00.static.v1
files:
  - crates/openbmp-physics/src/atmosphere/nrlmsise00.rs
source_class:     public-academic-reference
source_title:     NRLMSISE-00 empirical model of the atmosphere
source_authors:   Picone, J. M.; Hedin, A. E.; Drob, D. P.; Aikin, A. C.
source_id:        Journal of Geophysical Research 107(A12), 1468, 2002
publication_date: 2002-12-01
source_urls:
  - https://doi.org/10.1029/2002JA009430
  - https://ccmc.gsfc.nasa.gov/models/NRLMSIS~00/
  - https://pypi.org/project/nrlmsise00/
license_or_terms: Public empirical model and public C/Python interface.
retrieved_utc:    2026-05-24
transformation:
  method: >-
    Generated a fixed static-defaults altitude table by evaluating the
    public NRLMSISE-00 C model interface exposed by Python package
    `nrlmsise00==0.1.2`. Inputs were year=2024, doy=80,
    sec=43200, geodetic latitude=0 deg, longitude=0 deg,
    local solar time=12 h, F10.7A=150, F10.7=150, Ap=4, and
    altitudes 0, 100, 150, 200, ..., 1000 km. Number-density outputs
    were converted from cm^-3 to m^-3; mass density from g/cm^3 to
    kg/m^3. No fit was applied to table points; runtime interpolation
    is log-linear for non-negative densities and linear for
    temperatures.
  script: none committed; one-off audit regeneration.
verification:
  method: >-
    In-crate tests assert exact reproduction at the pinned table
    points, deterministic two-run output, finite pressure/speed of
    sound at orbital altitudes, and spot-check 200 km / 400 km mass
    densities against the regenerated public-model values in
    `crates/openbmp-physics/tests/hypersonic_validation.rs`.
  test: crates/openbmp-physics/tests/hypersonic_validation.rs
  tolerance: >-
    Table points: 1e-12 relative for density and temperature.
    Validation spot checks: 1 percent relative.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public Earth atmosphere model for academic drag / re-entry
    research. Static-defaults interpolation only; no operational
    mission profile or fielded-vehicle data.
```
