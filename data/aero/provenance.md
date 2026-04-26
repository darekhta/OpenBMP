# Provenance — `data/aero/synthetic-finned-cylinder.toml`

Canonical OpenBMP provenance record for the synthetic finned-cylinder
aerodynamic deck shipped under `data/aero/`. See
[`docs/data-provenance.md`](../../docs/data-provenance.md) for the
contract this record satisfies.

```yaml
dataset_id:       openbmp.aero.synthetic_finned_cylinder.v1
files:
  - data/aero/synthetic-finned-cylinder.toml
source_class:     synthetic-openbmp
source_title:     >-
  Hand-derived Barrowman-style build-up for a stylised 80 mm
  finned-cylinder sounding rocket. Closed-form generators:
    CD(M, α) = CD0(M) + 0.001 · α_deg²
    CN(M, α) = α_deg · (0.07 + 0.005 · M)
    CM(M, α) = -0.15 · CN(M, α)
  with CD0(M) a piecewise drag-rise curve peaking transonically.
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic; not derived from any fielded vehicle
publication_date: 2026-04-26
methodology_reference: >-
  Niskanen, S. (2009). *Development of an Open-Source Model Rocket
  Simulation Software*. Master's thesis, Helsinki University of
  Technology. The Barrowman component build-up applied here follows
  the conventions of §5 (drag and stability) of that thesis.
methodology_urls:
  - https://openrocket.info/documentation.html
  - https://github.com/openrocket/openrocket/wiki
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain. No
  third-party data is incorporated.
retrieved_utc:    2026-04-26
transformation:
  method: >-
    Hand-derived closed-form generators (see source_title above)
    evaluated on the (mach, alpha_deg, beta_deg) grid and pasted
    into the TOML deck file. No transcription from a published
    deck. The numerical values are exactly the closed-form output
    rounded to a fixed precision per coefficient channel; the
    Phase-2.5.C regression test recomputes the closed forms and
    asserts bit-equality after TOML parse.
  script: none
verification:
  method: >-
    Compile-time round-trip: `crates/openbmp-aero/tests/regression.rs`
    loads the deck via `include_str!` + `AeroDeck::load_from_str`,
    asserts CN(α = 0) == 0 exactly for every Mach, asserts
    CD > 0 on every grid point, asserts CM == -0.15 · CN on every
    grid point (the deck's restoring-moment definition), and
    re-runs the lookup at the centroid of every cube to verify
    bit-stability. The `synthetic_finned_cylinder` deck is the
    only aero data file shipped in Phase 2; Phase-3 will add
    additional shapes (sphere, cone) under the same provenance
    contract.
  test:   crates/openbmp-aero/tests/regression.rs
  tolerance: >-
    CN(α = 0) and CM(α = 0): exact zero (bit equality after parse).
    CD > 0 on every grid point: strict positivity.
    CM = -0.15 · CN: bit equality after parse for every grid point.
    Bit-stability across two lookups: bit equality on the reference
    platform profile.
validation_status: validated-toy
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic OpenBMP-authored content; no fielded-vehicle aero data,
    no real-rocket coefficient set. Suitable for analytic-toy and
    validation-toy scenarios; not a substitute for a real Barrowman
    pre-processor or wind-tunnel data. The OpenBMP deck format
    rejects fielded-vehicle decks at the schema boundary
    (`docs/software-architecture.md § Aerodynamics`); this synthetic
    deck respects that boundary.
```

## Context

OpenBMP's Phase-2 aero surface ships **Schema 1**: a tabulated
axisymmetric reduced sounding-rocket deck indexed by
`(mach, alpha_deg, beta_deg)` returning the three reduced
coefficients `(CN, CD, CM)`. The schema rejects fielded-vehicle
decks; the only data files shipped are synthetic textbook examples
that sanity-test the lookup pipeline without disclosing operational
aerodynamic data.

This finned-cylinder deck is a stylised hand-derived build-up. The
shape parameters (80 mm body diameter, 1.0 m reference length, 4
trapezoidal fins implied) are illustrative; the closed-form
generators are tuned to produce sensible values across subsonic and
supersonic flight regimes:

| Mach | CD(α=0) | CN slope (per deg) | Notes                          |
|------|---------|--------------------|--------------------------------|
| 0.0  | 0.450   | 0.0700             | Subsonic baseline              |
| 0.5  | 0.450   | 0.0725             | Subsonic                       |
| 0.8  | 0.550   | 0.0740             | Drag rise begins               |
| 1.0  | 0.750   | 0.0750             | Sonic plateau                  |
| 1.2  | 0.850   | 0.0760             | Transonic CD peak              |
| 1.5  | 0.700   | 0.0775             | Supersonic                     |
| 2.0  | 0.550   | 0.0800             | Supersonic, falling drag       |
| 3.0  | 0.400   | 0.0850             | Hypersonic boundary (Phase-6+) |

The `CD0(M = 1.2)` peak around 0.85 matches the textbook transonic
drag-rise behaviour for a slender finned cylinder; the linear
α-induced drag term `0.001 · α²` is small compared to baseline.
`CN` increases linearly with α at the rate `(0.07 + 0.005 · M)`
per degree, consistent with a slender-body normal-force slope of
4–5 per radian. `CM = -0.15 · CN` encodes a 15% static margin from
the centre of pressure to the moment reference point.

## Why synthetic-openbmp, not public-standard

There is no canonical public sounding-rocket aerodynamic deck.
OpenRocket / Niskanen 2009 ships a Barrowman-style **methodology**
(component build-up) that any user can run on a vehicle of their
choice; OpenBMP itself does not ship a Barrowman pre-processor —
the deck is the boundary. The Phase-2 plan calls for a **synthetic
textbook example** so the lookup pipeline can be exercised without
shipping or transcribing a fielded-vehicle deck.

The numerical values above are OpenBMP-authored content; the
methodology citation (Niskanen 2009) acknowledges that the
component build-up convention is the academic reference.

## Why fielded-vehicle decks are rejected

The OpenBMP aero deck format rejects fielded-vehicle decks at the
schema boundary
(`docs/software-architecture.md § Aerodynamics`):

> Real fielded-vehicle aero decks are explicitly rejected. The MVP
> ships only synthetic textbook decks for canonical shapes (sphere,
> cone, simple finned cylinder).

This is a project-level safety policy. Fielded-vehicle aerodynamic
decks contain operational data that may be subject to export
control, vehicle-program-restricted disclosure, or competitive
sensitivity. OpenBMP keeps the deck format open and the data
synthetic so the platform stays a research / academic tool.

## Why the values are mirrored against a closed-form generator

The Phase-2.5.C regression test re-evaluates the same closed-form
generators in code and asserts bit equality with the values parsed
from the TOML file. This is the same pattern used for the
USSA76 layer-pressure pin and the WGS84 J2 constant — a future
Phase-2.10 `openbmp check-provenance` walk will perform the same
check repository-wide so a typo in either the deck file or the
generator breaks CI before release.
