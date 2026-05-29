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
source_url:       https://github.com/openbmp/openbmp/blob/main/data/aero/synthetic-finned-cylinder.toml
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
source_hash_sha256: c5c7ad2af72659bbf351b33fb805b56d911f797fe9804e105fc13ed4bc8399a2
retrieved_utc:    2026-04-26
transformation:
  method: >-
    Hand-derived closed-form generators (see source_title above)
    evaluated on the (mach, alpha_deg, beta_deg) grid and pasted
    into the TOML deck file. No transcription from a published
    deck. The numerical values are emitted from the integer-scaled
    rational generator documented in the deck header and mirrored by
    `crates/openbmp-aero/tests/regression.rs`:
      M_tenths = 10 · M
      CN = α_deg · (140 + M_tenths) / 2000
      CD = (CD0_milli(M) + α_deg²) / 1000
      CM = -15 · CN_numerator / 200000
    with +0.0 preserved explicitly at α = 0.
  script: none
  script_hash_sha256: not-applicable; generator is embedded in the compiled regression test
verification:
  method: >-
    Compile-time round-trip: `crates/openbmp-aero/tests/regression.rs`
    loads the deck via `include_str!` + `AeroDeck::load_from_str`,
    asserts every parsed CN/CD/CM grid value matches the
    integer-scaled generator bit-for-bit, asserts CN(α = 0) and
    CM(α = 0) are +0.0 exactly for every Mach, asserts CD > 0 on
    every grid point, checks the CM = -0.15 · CN physical identity
    within tight f64 tolerance, and re-runs the lookup at the
    centroid of every cube to verify bit-stability. The
    `synthetic_finned_cylinder` deck is the canonical axisymmetric
    aero data file; additional shapes (sphere,
    cone) can be added under the same provenance contract.
  test:   crates/openbmp-aero/tests/regression.rs
  tolerance: >-
    Integer-scaled generator versus parsed CN/CD/CM at every grid
    point: bit equality. CN(α = 0) and CM(α = 0): exact +0.0 (bit
    equality after parse). CD > 0 on every grid point: strict
    positivity. CM = -0.15 · CN physical identity: 1e-12 absolute
    tolerance under ordinary f64 arithmetic.
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

OpenBMP's aero surface ships **Schema 1**: a tabulated
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
| 3.0  | 0.400   | 0.0850             | Hypersonic boundary (out of scope) |

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
the deck is the boundary. OpenBMP ships a **synthetic
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

The finned-cylinder regression test re-evaluates the same closed-form
generators in code and asserts bit equality with the values parsed
from the TOML file. This is the same pattern used for the
USSA76 layer-pressure pin and the WGS84 J2 constant — the
`openbmp check-provenance` walk performs the same
check repository-wide so a typo in either the deck file or the
generator breaks CI before release.

## `data/aero/synthetic-d12-class-rocket.toml`

```yaml
dataset_id:       openbmp.aero.synthetic_d12_class_rocket.v1
files:
  - data/aero/synthetic-d12-class-rocket.toml
source_class:     synthetic-openbmp
source_title:     >-
  Synthetic OpenBMP-authored aerodynamic deck for a 24 mm-diameter,
  0.30 m-long model-rocket airframe sized to the Estes D12 motor
  envelope. Used by the sounding-rocket validation case
  alongside the real Estes D12 motor data.
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic; not derived from any fielded vehicle
publication_date: 2026-04-27
methodology_reference: >-
  Same Barrowman-style component build-up convention as the
  `synthetic-finned-cylinder.toml` deck — Niskanen 2009
  thesis §5 component build-up. CN slope and CD0(M) values chosen
  to match the D-class model-rocket envelope (CD0 ≈ 0.6 subsonic,
  rising through transonic).
methodology_urls:
  - https://openrocket.info/documentation.html
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain. No
  third-party data is incorporated.
retrieved_utc:    2026-04-27
transformation:
  method: >-
    Hand-derived integer-scaled rational generators (same family as
    the finned-cylinder deck):
      M_tenths = 10 · M
      CN = α_deg · (140 + M_tenths) / 2000
      CD = (CD0_milli(M) + α_deg²) / 1000
      CM = -15 · CN_numerator / 200000
    with CD0_milli specialised for the D-class envelope:
      M=0.0: 600  (CD0 = 0.60)
      M=0.3: 650  (CD0 = 0.65)
      M=0.5: 700  (CD0 = 0.70)
      M=0.8: 850  (CD0 = 0.85)
      M=1.0: 1000 (CD0 = 1.00)
  script: none
verification:
  method: >-
    `crates/openbmp-vehicle/tests/sounding_rocket.rs` loads the
    deck via `include_str!` + `AeroDeck::load_from_str`, asserts
    grid sizes (5 × 5 × 1) and the (M=0, α=0) corner CD = 0.6
    bit-exactly, and runs the deck through the D12
    sounding-rocket integration test.
  test:   crates/openbmp-vehicle/tests/sounding_rocket.rs
  tolerance: >-
    Grid-corner lookups: bit equality after parse. D12 integration
    physical sanity envelope: 450–550 m apogee for a 70 g airframe;
    exact replay is pinned by the sounding-rocket test's final-state
    and headline-metric bit patterns.
validation_status: validated-toy
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic OpenBMP-authored content sized to the D-class
    envelope. Not a transcription of any published deck.
```

## `data/aero/synthetic-niskanen-ch6-rocket.toml`

```yaml
dataset_id:       openbmp.aero.synthetic_niskanen_ch6_rocket.v1
files:
  - data/aero/synthetic-niskanen-ch6-rocket.toml
source_class:     synthetic-openbmp
source_title:     >-
  Synthetic OpenBMP-authored reduced aero deck for the Niskanen
  2009 Chapter-6 small-rocket benchmark. Constant axial CD = 0.8
  across the Mach grid (point-mass reduction), CN and CM
  identically zero. Sized to the 29 mm-diameter, 0.56 m-long body
  documented in Niskanen Chapter 6.
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic; not derived from any fielded vehicle
publication_date: 2026-04-27
methodology_reference: >-
  Niskanen, S. (2009). *Development of an Open-Source Model Rocket
  Simulation Software*. Master's thesis, Helsinki University of
  Technology. The constant-CD point-mass reduction is the
  documented Chapter-6 benchmark form for vertical-launch apogee
  comparison.
methodology_urls:
  - https://openrocket.sourceforge.net/thesis.pdf
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain. No
  third-party data is incorporated.
retrieved_utc:    2026-04-27
transformation:
  method: >-
    Constant axial CD = 0.8 across a 5 × 1 × 1 grid (mach in
    {0.0, 0.3, 0.5, 0.8, 1.0}; alpha and beta single-point at 0).
    CN and CM identically zero. Reference area is the body
    cross-section `π · (0.029/2)² ≈ 6.605198554172541e-4 m²`
    (exact f64 value); reference length is the body length 0.56 m.
    `extrapolation = "clamp"` because the rocket may briefly
    exceed M = 1.0 at peak velocity; clamping to the boundary CD
    is the documented reduction.
  script: none
verification:
  method: >-
    `crates/openbmp-vehicle/tests/sounding_rocket.rs` loads the
    deck via `include_str!` + `AeroDeck::load_from_str` and runs
    it through the Niskanen Chapter-6 benchmark, which asserts
    apogee within ±5% of the published experimental value.
  test:   crates/openbmp-vehicle/tests/sounding_rocket.rs
  tolerance: >-
    Schema-1 round-trip: bit equality on the parsed numerical
    fields. Apogee comparison: ±5% relative against Niskanen 2009
    Chapter 6 experimental C6 value (151.5 m).
validation_status: validated-toy
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic OpenBMP-authored content. The point-mass constant-CD
    reduction is a documented Chapter-6 benchmark form, not a
    fielded-vehicle deck.
```

## `data/aero/synthetic-elevon-1d.toml`

```yaml
dataset_id:       openbmp.aero.synthetic_elevon_1d.v1
files:
  - data/aero/synthetic-elevon-1d.toml
source_class:     synthetic-openbmp
source_title:     >-
  Synthetic OpenBMP-authored Schema-2 aerodynamic deck for the
  single-elevon-aero exit-criterion. Single elevon axis (`delta_e_deg`)
  added to a finned-cylinder-style deck so a non-zero
  control-surface deflection perturbs the normal-force, drag, and
  pitching-moment coefficients.
  Used by the single-elevon-aero scenario to demonstrate
  that an effector deflection actually changes the body-frame aero
  force at runtime (vs. the gravity-only telemetry-only
  case).
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic; not derived from any fielded vehicle
publication_date: 2026-04-28
methodology_reference: >-
  Same Barrowman-style component build-up as the
  `synthetic-finned-cylinder.toml` deck (Niskanen 2009 §5),
  extended with linear / absolute elevon-deflection perturbations in
  CN, CD, and CM. The elevon coefficients are illustrative — chosen
  so a 5° deflection moves CN by ~0.10 and a 20° deflection adds
  0.20 to CD — not derived from any specific airframe.
methodology_urls:
  - https://openrocket.info/documentation.html
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain. No
  third-party data is incorporated.
retrieved_utc:    2026-04-28
transformation:
  method: >-
    Hand-derived linear generators evaluated on the cartesian
    product of (mach, alpha_deg, beta_deg, delta_e_deg):
      CN(M, α, β, δ_e) = α_deg · (0.07 + 0.005 · M)
                       + 0.020 · δ_e_deg
      CD(M, α, β, δ_e) = CD0(M) + 0.001 · α_deg² + 0.010 · |δ_e_deg|
      CM(M, α, β, δ_e) = -0.15 · CN(M, α, β, 0) + 0.050 · δ_e_deg
    with CD0 the finned-cylinder transonic curve at the
    Mach grid points. The CD perturbation was strengthened for
    the single-elevon-aero case so the e2e baseline-vs-deflected force signal is well
    above telemetry noise. Pasted into the deck file by hand; no
    script.
  script: none
verification:
  method: >-
    Schema-2 parser load via `AeroDeck::load_from_str`. The Schema-2
    parser-test suite validates round-trip metadata, lookup at
    `delta_e_deg = 0` matching a Schema-1 companion bit-for-bit, and
    grid-corner exact-equality. The single-elevon-aero e2e test loads this
    deck, runs the single-elevon-aero scenario, and asserts the
    aero force telemetry differs between a 0° baseline run and a
    deflected-elevon run (the schema-2 effector-axis consumption
    contract).
  test:   crates/openbmp-aero/src/parser.rs (mod tests)
  tolerance: >-
    Parser round-trip and grid-corner lookups: bit equality on
    parsed CN/CD/CM. Schema-1 companion at delta_e_deg = 0: bit
    equality between Schema-1 and Schema-2 lookups at the same
    (mach, alpha, beta) grid points. E2e baseline-vs-deflected
    force comparison: at least 5% relative difference at t = 2.5 s.
validation_status: validated-toy
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic OpenBMP-authored content. The elevon perturbation
    coefficients are illustrative round numbers, not transcribed
    from any published wind-tunnel dataset.
```

## `data/aero/calisto-drag.toml` + `data/aero/calisto-drag.csv`

```yaml
dataset_id:       openbmp.aero.calisto_drag.v1
files:
  - data/aero/calisto-drag.toml
  - data/aero/calisto-drag.csv
source_class:     converted-public
source_title:     >-
  Calisto drag-curve aero deck for the RocketPy cross-
  tool validation case. Schema-1 deck with single-point alpha and
  beta axes (axisymmetric reduced point-mass drag-only model);
  CD-vs-Mach curve from RocketPy's published Calisto example
  (`data/rockets/calisto/powerOffDragCurve.csv` and
  `powerOnDragCurve.csv`, byte-identical upstream — Calisto is
  treated with a single CD curve regardless of motor burn state).
  CN and CM identically zero on every grid point. 200-point Mach
  grid from M = 0.01 to M = 2.00 in 0.01 increments. Reference
  area = π · 0.0635² m² (Calisto body radius from the RocketPy
  Calisto example). `extrapolation = "clamp"`.
source_authors:   >-
  RocketPy Team (Calisto example data); curve published in the
  RocketPy GitHub repository under MIT license.
source_id:        RocketPy Calisto-example powerOffDragCurve.csv
source_url:       https://raw.githubusercontent.com/RocketPy-Team/RocketPy/cb15a393ee2d9430cc21c57c98768dc1890a198a/data/rockets/calisto/powerOffDragCurve.csv
source_hash_sha256: 94760a42d5f5fad4fb815f448db72201fafd2b2a3c6ec2d2c83618b2953207e0
publication_date: 2022-08-01  # approximate; tracks the RocketPy v1.0 release
methodology_reference: >-
  Souza, A. M. et al. *RocketPy: Six Degree-of-Freedom Rocket
  Trajectory Simulator*. Journal of Aerospace Engineering 35:5
  (2022). The Calisto example airframe is documented in §V of the
  paper and ships as the canonical RocketPy validation case.
methodology_urls:
  - https://doi.org/10.1061/(ASCE)AS.1943-5525.0001331
  - https://github.com/RocketPy-Team/RocketPy
  - https://docs.rocketpy.org/en/latest/notebooks/getting_started_colab.html
license_or_terms: >-
  RocketPy is MIT-licensed. The Calisto example data ships as
  part of the RocketPy repository and is re-distributable under
  the MIT terms. OpenBMP cites the source repository and paper.
retrieved_utc:    2026-04-29
transformation:
  method: >-
    Upstream `powerOffDragCurve.csv` saved verbatim at
    `data/aero/calisto-drag.csv` (SHA-256 pinned above). The
    upstream `powerOnDragCurve.csv` is byte-identical and is not
    re-shipped — the OpenBMP `calisto-drag.csv` covers both
    motor burn states. The OpenBMP TOML at
    `data/aero/calisto-drag.toml` transcribes the (mach, CD)
    pairs verbatim into the Schema-1 grid format with single-
    point alpha = [0.0] and beta = [0.0] axes. CN and CM tables
    are filled with 0.0 on every grid point. Reference area
    derived from RocketPy's documented `radius = 0.0635 m` as
    π · 0.0635².
  script: none
verification:
  method: >-
    `crates/openbmp-aero/tests/calisto_deck_pin.rs` round-trips
    the OpenBMP TOML against the upstream CSV: it parses both
    files via `include_str!`, asserts the SHA-256 matches the
    pin recorded above, and asserts every (mach, CD) pair in
    the deck lookup matches the corresponding CSV row to bit
    precision. Additional invariants: CN and CM identically
    zero on every grid point; CD ≥ 0 on every grid point.
  test:   crates/openbmp-aero/tests/calisto_deck_pin.rs
  tolerance: >-
    Every (mach, CD) pair: bit equality between deck lookup and
    upstream CSV row. The cross-tool audit also verified the
    commit-pinned `powerOnDragCurve.csv` at RocketPy commit
    `cb15a393ee2d9430cc21c57c98768dc1890a198a` has the same
    SHA-256, so the single-deck simplification covers both burn
    states.
validation_status: validated-toy
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public hobby-rocketry-class drag curve from the
    MIT-licensed RocketPy Calisto example. No export-control or
    manufacturer-proprietary restrictions; OpenBMP's use is
    purely for cross-tool validation against a published
    open-source rocket-trajectory simulator.
```

## `data/aero/phalcon9-drag.toml`

```yaml
dataset_id:       openbmp.aero.phalcon9_drag.v1
files:
  - data/aero/phalcon9-drag.toml
source_class:     synthetic-openbmp
source_title:     >-
  Synthetic OpenBMP-authored full force+moment aero deck for the
  Phalcon-9 demonstration launch vehicle — a wholly synthetic, rounded
  order-of-magnitude deck for a ~3.7 m-diameter medium-lift kerolox
  launcher, tabulated over a Mach × angle-of-attack grid. Normal-force
  CN (lift) and pitching-moment CM vary linearly with angle of attack;
  axial CD is a CD0(Mach) polar with a quadratic induced-drag term. The
  rigid-body aero path applies CN as body normal force and CD as axial
  drag; the aero moment adapter applies CM as the body pitching moment
  (referenced to the 3.7 m body length). Referenced to the body
  cross-section area π·(3.7/2)² ≈ 10.75 m². NOT a fielded-vehicle deck
  and not validated against any real aero data.
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic; not derived from any fielded vehicle
source_url:       https://github.com/openbmp/openbmp/blob/main/data/aero/phalcon9-drag.toml
publication_date: 2026-05-30
methodology_reference: >-
  Hand-chosen class-anchor linear-in-alpha coefficients for a medium-lift
  launcher: CN = 2.5·α_rad (slender-body normal-force slope), CM =
  −0.20·α_rad (mildly statically STABLE / restoring), CD = CD0(Mach) +
  0.5·α_rad² with CD0 ≈ 0.30 subsonic / ~0.65 transonic peak at M1 /
  ~0.22 hypersonic, referenced to the body cross-section. The ascent
  flies a near-zero-AoA gravity turn, so the nominal CN/CM contribution
  is small; the deck exists so off-nominal angle of attack (e.g. winds)
  produces physical lift and a trimmed pitching moment. Order-of-magnitude
  figures, not a build-up against any fielded geometry. (Upper-stage aero
  is intentionally absent: stage-2 flight is exoatmospheric.)
methodology_urls:
  - https://github.com/openbmp/openbmp/blob/main/scenarios/phalcon9/provenance.md
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain. No
  third-party data is incorporated.
source_hash_sha256: 88259d8f3400538535be070ce4d6045a1c24b38033267dc9e326780cc927921f
retrieved_utc:    2026-05-30
transformation:
  method: >-
    Hand-authored linear-in-alpha generator (CN/CM/CD formulas above)
    evaluated on the (Mach, alpha_deg, beta_deg) grid and written into
    the TOML deck; no transcription from a published deck.
  script: none
  script_hash_sha256: not-applicable; values are hand-authored class anchors
verification:
  method: >-
    Exercised end-to-end by the phalcon9-orbit scenario (atmospheric
    two-stage ascent under DeckDragForceAdapter); the scenario pins
    this deck via deck_sha256 so any edit to the values is caught at
    load time.
  test:   scenarios/phalcon9/phalcon9-orbit.toml (deck_sha256 pin)
  tolerance: >-
    SHA-256 equality between the deck file and the scenario's
    deck_sha256 pin.
validation_status: experimental
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Wholly synthetic, rounded order-of-magnitude drag polar for a
    clearly-labelled synthetic demonstration launcher. No
    export-controlled, fielded, or manufacturer-proprietary content.
```
