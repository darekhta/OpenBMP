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

Fielded-vehicle aerodynamic decks can contain vehicle-program-restricted
disclosure or competitively sensitive data. OpenBMP keeps the deck format open
and the in-tree data synthetic or public so the platform stays reviewable.

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
    MIT-licensed RocketPy Calisto example. No manufacturer-proprietary
    restrictions are noted; OpenBMP's use is
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
    fielded or manufacturer-proprietary content.
```

## `data/aero/panel-sphere-tolerance-v1.toml`

```yaml
dataset_id:       openbmp.aero.panel_sphere_tolerance.v1
files:
  - data/aero/panel-sphere-tolerance-v1.toml
source_class:     synthetic-openbmp
source_title:     >-
  Closed-form Modified-Newtonian sphere drag-coefficient tolerance fixture
  for the WP-03.1 local-inclination panel solver.
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic; analytic sphere integral
source_url:       https://github.com/openbmp/openbmp/blob/main/data/aero/panel-sphere-tolerance-v1.toml
publication_date: 2026-06-27
methodology_reference: >-
  Newtonian local-inclination sphere pressure integral. For a unit sphere
  facing the freestream, Cp = Cp_max cos^2(theta) and the axial force
  integral over the illuminated hemisphere gives CD = Cp_max / 2 when
  referenced to pi R^2.
methodology_urls: []
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain. No third-party
  aerodynamic data is incorporated.
source_hash_sha256: not-applicable; analytic fixture generated in-tree
retrieved_utc:    2026-06-27
transformation:
  method: >-
    The fixture records the analytic expected coefficient CD = 1.0 for
    Cp_max = 2.0, a deterministic latitude-longitude mesh resolution, and
    a relative-error tolerance of 1.0e-3 for the generated panel mesh.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/hypersonic.rs` test
    `local_inclination_sphere_cd_matches_closed_form_tolerance_table` parses
    this TOML fixture, generates the declared watertight sphere mesh, verifies
    manifold closure, evaluates `LocalInclinationPanels`, and checks the
    reported drag coefficient against the closed-form value.
  test: crates/openbmp-aero/src/hypersonic.rs
  tolerance: >-
    Relative error in CD <= 1.0e-3 against CD = Cp_max / 2.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic analytic geometry only; no fielded vehicle, wind-tunnel, CFD, or
    trajectory data. This fixture validates the local panel integration
    algebra and mesh convergence on a canonical shape, not a certified aero
    databook.
```

## `data/aero/panel-sharp-cone-tolerance-v1.toml`

```yaml
dataset_id:       openbmp.aero.panel_sharp_cone_tolerance.v1
files:
  - data/aero/panel-sharp-cone-tolerance-v1.toml
source_class:     synthetic-openbmp
source_title:     >-
  Closed-form Modified-Newtonian sharp-cone axial-coefficient tolerance
  fixture for the WP-03.1 local-inclination panel solver.
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic; analytic cone integral
source_url:       https://github.com/openbmp/openbmp/blob/main/data/aero/panel-sharp-cone-tolerance-v1.toml
publication_date: 2026-06-27
methodology_reference: >-
  Modified-Newtonian tangent-cone approximation. For a cone at zero angle of
  attack, Cp_cone = Cp_max sin^2(theta_c), and the side-area axial projection
  referenced to base area gives CA = Cp_cone.
methodology_urls: []
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain. No third-party
  aerodynamic data is incorporated.
source_hash_sha256: not-applicable; analytic fixture generated in-tree
retrieved_utc:    2026-06-27
transformation:
  method: >-
    The fixture records Mach 12, gamma 1.4, a 15 degree cone half-angle,
    the corresponding OpenBMP finite-Mach Cp_max, expected tangent-cone CA,
    a deterministic azimuthal panel count, and a relative-error tolerance of
    1.0e-3 for the generated cone-side panel mesh.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/hypersonic.rs` test
    `local_inclination_sharp_cone_ca_matches_tangent_cone_tolerance_table`
    parses this TOML fixture, generates the declared cone-side mesh, verifies
    the stored Cp_max and expected CA against OpenBMP's tangent-cone formula,
    evaluates `LocalInclinationPanels`, and checks the reported axial
    coefficient against the closed-form value.
  test: crates/openbmp-aero/src/hypersonic.rs
  tolerance: >-
    Relative error in CA <= 1.0e-3 against
    CA = Cp_max sin^2(theta_c).
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic analytic geometry only; no fielded vehicle, wind-tunnel, CFD, or
    trajectory data. This fixture validates local panel integration against
    a canonical cone formula, not a certified aero databook.
```

## `data/aero/panel-hb2-forebody-figure12-v1.toml`

```yaml
dataset_id:       openbmp.aero.panel_hb2_forebody_figure12.v1
files:
  - data/aero/panel-hb2-forebody-figure12-v1.toml
source_class:     public-domain-us-gov
source_title:     >-
  Summary Report on Aerodynamic Characteristics of Standard Models HB-1 and
  HB-2
source_authors:   Gray, J. D.
source_id:        AEDC-TDR-64-137 / AD602769
source_url:       https://ntrl.ntis.gov/NTRL/dashboard/searchResults/titleDetail/AD602769.xhtml
publication_date: 1964
methodology_reference: >-
  Figure 12b, "Zero-Lift Forebody Axial-Force Coefficient versus Reynolds
  Number for Model HB-2", high-Mach panel. The report states that the HB-2
  forebody axial-force data are plotted against Reynolds number and Mach number
  for the standard HB-2 correlation model.
methodology_urls:
  - https://ntrl.ntis.gov/NTRL/dashboard/searchResults/titleDetail/AD602769.xhtml
license_or_terms: >-
  United States Government technical report available from NTIS/NTRL. The
  fixture stores three hand-digitized plot ordinates and source metadata only;
  no report pages or images are redistributed.
source_hash_sha256: fe0c1b2a71f7153750520f89b688b5db9b6a6d79cc9689303c780869dd208bc8
retrieved_utc:    2026-06-28
transformation:
  method: >-
    Three high-Mach Figure 12b points for Mach 6.06, 7.10, and 8.07 were read
    from the scanned plot by visual digitization. The geometry used by the test
    follows the report's HB-2 tangent-location text and Figure 1 dimensions:
    reference diameter d, 0.3d blunt nose radius, 4.5d forebody length, and the
    report's hemisphere-cone, cone-shoulder, shoulder-cylinder,
    cylinder-radius, and radius-flare station ratios. The fixture uses a
    +/-10% relative tolerance because the source is plot-only and the T1 method
    is an inviscid local-inclination approximation.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/hypersonic.rs` test
    `local_inclination_hb2_forebody_ca_matches_figure12_high_mach_fixture`
    parses this TOML fixture, generates the declared HB-2 forebody
    local-inclination mesh, evaluates Modified-Newtonian panel loads at zero
    lift, and checks forebody axial-force coefficient against the digitized
    Figure 12b samples.
  test: crates/openbmp-aero/src/hypersonic.rs
  tolerance: >-
    Relative error in forebody CA <= 0.10 for each digitized high-Mach sample.
validation_status: research
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public historical calibration-model data for a generic standard body. This
    fixture validates only the inviscid high-Mach forebody axial-force envelope;
    it does not validate base drag, viscous increments, low-Mach separation,
    six-coefficient wind-tunnel closure, or any fielded vehicle.
```

## `data/aero/panel-hb2-forebody-skin-friction-figure12-v1.toml`

```yaml
dataset_id:       openbmp.aero.panel_hb2_forebody_skin_friction_figure12.v1
files:
  - data/aero/panel-hb2-forebody-skin-friction-figure12-v1.toml
source_class:     public-domain-us-gov
source_title:     >-
  Summary Report on Aerodynamic Characteristics of Standard Models HB-1 and
  HB-2
source_authors:   Gray, J. D.
source_id:        AEDC-TDR-64-137 / AD602769
source_url:       https://ntrl.ntis.gov/NTRL/dashboard/searchResults/titleDetail/AD602769.xhtml
publication_date: 1964
methodology_reference: >-
  Figure 12b, "Zero-Lift Forebody Axial-Force Coefficient versus Reynolds
  Number for Model HB-2", high-Mach panel. The report defines Reynolds number
  for these plots using the centerbody diameter.
methodology_urls:
  - https://ntrl.ntis.gov/NTRL/dashboard/searchResults/titleDetail/AD602769.xhtml
license_or_terms: >-
  United States Government technical report available from NTIS/NTRL. The
  fixture stores three hand-digitized plot ordinates and source metadata only;
  no report pages or images are redistributed.
source_hash_sha256: fe0c1b2a71f7153750520f89b688b5db9b6a6d79cc9689303c780869dd208bc8
retrieved_utc:    2026-06-28
transformation:
  method: >-
    Three high-Mach Figure 12b points for Mach 6.06, 7.10, and 8.07 were read
    from the scanned plot by visual digitization. The test maps the report's
    Reynolds-number axis to `PanelSkinFriction::reference_length_m = d`, then
    computes local Re_x from the panel centroid distance downstream of the
    forebody leading edge. Smooth-wall settings use Tw/Te = 1.0, transition
    Reynolds number 5.0e5, and zero roughness. The fixture uses a +/-10%
    relative tolerance because the source is plot-only and the T1 method is an
    inviscid pressure model plus reference-temperature skin friction, not a
    calibrated separation or base-drag model.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/hypersonic.rs` test
    `local_inclination_hb2_forebody_skin_friction_matches_figure12_high_mach_fixture`
    parses this TOML fixture, generates the declared HB-2 forebody mesh,
    enables `PanelSkinFriction`, verifies the skin-friction-enabled total
    forebody axial-force coefficient against the digitized Figure 12b samples,
    and checks that the viscous result is larger than the inviscid panel result.
  test: crates/openbmp-aero/src/hypersonic.rs
  tolerance: >-
    Relative error in total forebody CA <= 0.10 for each digitized high-Mach
    sample.
validation_status: research
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public historical calibration-model data for a generic standard body. This
    fixture validates the high-Mach smooth-wall total forebody axial-force
    envelope for the skin-friction-enabled panel solver; it does not isolate
    boundary-layer transition, validate roughness, validate base drag, validate
    low-Mach separation, or represent any fielded vehicle.
```

## `data/aero/panel-sharp-cone-skin-friction-trend-v1.toml`

```yaml
dataset_id:       openbmp.aero.panel_sharp_cone_skin_friction_trend.v1
files:
  - data/aero/panel-sharp-cone-skin-friction-trend-v1.toml
source_class:     synthetic-openbmp
source_title:     >-
  Smooth-wall reference-temperature sharp-cone skin-friction trend fixture for
  the WP-03.2 local-inclination panel solver.
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic; reference-temperature smooth-wall trend
source_url:       https://github.com/openbmp/openbmp/blob/main/data/aero/panel-sharp-cone-skin-friction-trend-v1.toml
publication_date: 2026-06-28
methodology_reference: >-
  OpenBMP PanelSkinFriction reference-temperature correlations applied to the
  same 15 degree local-inclination sharp-cone mesh family as the WP-03.1
  tangent-cone fixture. The fixture checks a qualitative smooth-wall trend:
  positive viscous axial increment, reduced increment as Reynolds length
  increases, and a small high-Re correction. It is not HB-2 numeric data.
methodology_urls: []
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain. No third-party
  aerodynamic data is incorporated.
source_hash_sha256: not-applicable; analytic fixture generated in-tree
retrieved_utc:    2026-06-28
transformation:
  method: >-
    The fixture records Mach 8, gamma 1.4, a 15 degree cone half-angle,
    smooth-wall reference-temperature settings, low/high Reynolds-length
    cases, expected inviscid CA, expected low/high viscous CA increments, a
    relative tolerance, and a high-Re increment fraction bound.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/hypersonic.rs` test
    `local_inclination_sharp_cone_skin_friction_matches_trend_fixture` parses
    this TOML fixture, generates the deterministic cone-side mesh, evaluates
    inviscid and skin-friction-enabled `LocalInclinationPanels` cases, and
    checks the monotone/small-correction trend plus fixture values.
  test: crates/openbmp-aero/src/hypersonic.rs
  tolerance: >-
    Relative error <= fixture max_relative_error; high-Re delta / inviscid CA
    <= 0.05.
validation_status: validated-toy
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic smooth-wall trend fixture only; no fielded vehicle, wind-tunnel,
    CFD, or HB-2 table data. This fixture validates the in-tree skin-friction
    coupling trend on a canonical cone, not a certified aero databook.
```

## `data/aero/panel-wedge-cavity-shadow-v1.toml`

```yaml
dataset_id:       openbmp.aero.panel_wedge_cavity_shadow.v1
files:
  - data/aero/panel-wedge-cavity-shadow-v1.toml
source_class:     synthetic-openbmp
source_title:     >-
  Exact-set BVH centroid-ray shadowing fixture for a synthetic recessed
  triangular-prism wedge behind a watertight front lip.
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic; hand-computed occlusion set
source_url:       https://github.com/openbmp/openbmp/blob/main/data/aero/panel-wedge-cavity-shadow-v1.toml
publication_date: 2026-06-27
methodology_reference: >-
  Deterministic ray-cast visibility test on a constructed watertight mesh. At
  alpha = beta = 0, the freestream/upstream ray is along +x. Only triangle 12,
  the recessed wedge's +x-facing face, is behind the front lip along its
  centroid ray; the outer lip remains visible.
methodology_urls: []
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain. No third-party
  aerodynamic data is incorporated.
source_hash_sha256: not-applicable; analytic fixture generated in-tree
retrieved_utc:    2026-06-27
transformation:
  method: >-
    The fixture records the expected occluded triangle index set [12] plus
    sign-test-only and BVH-shadowed force-x totals for the hand-built synthetic
    wedge-cavity mesh generated in the regression test.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/hypersonic.rs` test
    `local_inclination_wedge_cavity_bvh_occlusion_set_matches_fixture` parses
    this TOML fixture, generates the declared synthetic wedge-cavity mesh,
    verifies watertightness, enumerates facing panel centroids through
    `PanelMeshBvh`, checks the occlusion set exactly, and verifies the
    sign-test-only and BVH-shadowed force totals.
  test: crates/openbmp-aero/src/hypersonic.rs
  tolerance: >-
    Occluded triangle index set: exact equality. Force-x totals: 1e-12
    absolute tolerance.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic hand-built mesh only; no fielded vehicle, wind-tunnel, CFD, or
    trajectory data. This fixture validates BVH centroid-ray visibility
    semantics for local-inclination panels, not a certified aero databook.
```

## `data/aero/datcom-buildup-validation-v1.toml`

```yaml
dataset_id:       openbmp.aero.datcom_buildup_validation.v1
files:
  - data/aero/datcom-buildup-validation-v1.toml
source_class:     synthetic-openbmp
source_title:     >-
  Synthetic DATCOM-style buildup validation fixture for the WP-03.3
  `DatcomBuildupValidationDeck` tolerance harness.
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic; generated from the in-tree DatcomComponentBuildup
source_url:       https://github.com/openbmp/openbmp/blob/main/data/aero/datcom-buildup-validation-v1.toml
publication_date: 2026-06-28
methodology_reference: >-
  OpenBMP `DatcomComponentBuildup` applied to the same synthetic finned-body
  geometry used by the crate-level DATCOM buildup regression tests. The
  fixture pins the validation parser/reporting contract and coefficient plus
  full dynamic-derivative comparison paths; it is not NACA Report 1048,
  AGARD-B, Missile-DATCOM, wind-tunnel, or flight data.
methodology_urls: []
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain. No third-party
  aerodynamic data is incorporated.
source_hash_sha256: not-applicable; synthetic in-tree model fixture
retrieved_utc:    2026-06-28
transformation:
  method: >-
    The fixture records a synthetic 0.2 m diameter, 2.4 m long finned body
    with ogive nose, boattail, four fins, and three Mach/alpha/Reynolds
    samples. Expected CN/CD/CM and Cl_p/Cl_q/Cl_r/Cm_p/Cm_q/Cm_r/Cn_p/Cn_q/Cn_r
    values were emitted from the current `DatcomComponentBuildup` public
    `DragBuildupModel` path and the baked dynamic-derivative deck path.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/validation.rs` test
    `datcom_validation_fixture_matches_buildup_model` parses this TOML
    fixture, evaluates `DatcomComponentBuildup`, checks three samples and 36
    scalar coefficient/derivative expectations against absolute/relative
    tolerances, and verifies the deterministic validation report passes.
    `datcom_validation_rejects_empty_expected_values` proves malformed samples
    fail closed.
  test: crates/openbmp-aero/src/validation.rs
  tolerance: >-
    Absolute and relative tolerances are both 1.0e-12 for every checked scalar.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic harness fixture only; no fielded vehicle, wind-tunnel, CFD,
    NACA, AGARD, or Missile-DATCOM numeric data. This fixture validates the
    in-tree validation machinery and deterministic model path, not certified
    aerodynamic closure.
```

## `data/aero/naca-report-1048-crossflow-equation-v1.toml`

```yaml
dataset_id:       openbmp.aero.naca_report_1048_crossflow_equation.v1
files:
  - data/aero/naca-report-1048-crossflow-equation-v1.toml
source_class:     public-domain-naca
source_title:     >-
  NACA Report 1048, "A Study of Effects of Viscosity on Flow Over Slender
  Inclined Bodies of Revolution"
source_authors:   H. Julian Allen and Edward W. Perkins
source_id:        NACA-TR-1048 / NASA NTRS 19930090962
source_url:       https://ntrs.nasa.gov/citations/19930090962
publication_date: 1951
methodology_reference: >-
  Allen and Perkins' crossflow analogy for slender inclined bodies of
  revolution. The fixture uses the public equation-level shape
  CN_crossflow = C_Dc * (projected_planform_area / reference_area) *
  |sin(alpha)| * sin(alpha) * |cos(alpha)|, matched to OpenBMP's bounded
  finite-length-corrected crossflow-drag coefficient schedule and
  projected-planform centroid. It does not digitize NACA plot data.
methodology_urls:
  - https://ntrs.nasa.gov/citations/19930090962
license_or_terms: >-
  Public-domain United States Government/NACA report. The fixture values are
  OpenBMP-authored evaluations of the report equation shape and do not
  reproduce report figures.
source_hash_sha256: not-applicable; source referenced by public NTRS record
retrieved_utc:    2026-06-28
transformation:
  method: >-
    Three body/flow cases evaluate the crossflow equation shape over positive
    and negative angle of attack. Geometry is a simple 1 m diameter, 6 m long
    conical/cylindrical body with reference area pi/4 m^2; expected values are
    computed from OpenBMP's declared finite-length-corrected C_Dc(M)
    schedule, the projected body planform area, and the projected-planform
    centroid used as the crossflow center of pressure.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/validation.rs` test
    `naca_report_1048_crossflow_fixture_matches_equation_shape` parses this
    TOML fixture and checks the public `DatcomComponentBuildup` crossflow
    increment against the three expected CN/cp pairs. A source-class rejection
    test proves the fixture path is reserved for public-domain NACA inputs.
  test: crates/openbmp-aero/src/validation.rs
  tolerance: >-
    Absolute and relative tolerances are both 1.0e-12 for CN and cp.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public equation-level validation only. This fixture does not claim
    digitized NACA Report 1048 plot closure, wind-tunnel uncertainty closure,
    AGARD-B stability-derivative validation, or full DATCOM validation.
```

## `data/aero/naca-report-1048-body-i-crossflow-v1.toml`

```yaml
dataset_id:       openbmp.aero.naca_report_1048_body_i_crossflow.v1
files:
  - data/aero/naca-report-1048-body-i-crossflow-v1.toml
source_class:     public-domain-naca
source_title:     NACA Report 1048, A Study of Effects of Viscosity on Flow Over Slender Inclined Bodies of Revolution
source_authors:   H. Julian Allen; Edward W. Perkins
source_id:        NACA-TR-1048 / NASA NTRS 19930090962
source_url:       https://ntrs.nasa.gov/citations/19930090962
publication_date: 1951
methodology_reference: >-
  NACA Report 1048 Figure 6 defines body I as a 33 1/3-caliber ogival nose
  plus cylindrical afterbody with fineness ratio 21.1. The appendix gives the
  finite-length crossflow drag ratio 0.755 for body I.
methodology_urls:
  - https://ntrs.nasa.gov/citations/19930090962
license_or_terms: >-
  Public-domain United States Government/NACA report. The fixture stores
  OpenBMP-authored evaluations of the report body geometry and appendix
  finite-length correction; it does not digitize plotted force/moment points.
source_hash_sha256: ee890e2a2606087df55fe26958ff8aa734b7272576ccfdfe0f432c43a18a5f60
retrieved_utc:    2026-06-28
transformation:
  method: >-
    Body diameter is normalized to 1.0, body length to the report fineness
    label 21.1, nose fineness to 4.314 / 0.750 = 5.752, and reference area to
    pi/4. Four Mach-1.98 samples evaluate OpenBMP's finite-length-corrected
    Allen-Perkins crossflow increment and projected-planform centroid at
    4, 8, 12, and 16 degrees angle of attack.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/validation.rs` test
    `naca_report_1048_body_crossflow_fixtures_match_report_geometry` parses
    this TOML fixture and checks four expected CN/cp pairs through the public
    `DatcomComponentBuildup` crossflow seam.
  test: crates/openbmp-aero/src/validation.rs
  tolerance: >-
    Absolute and relative tolerances are both 1.0e-12 for CN and cp.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public report-geometry/equation validation only. This fixture does not
    claim digitized NACA Report 1048 force/moment plot residual closure,
    wind-tunnel uncertainty closure, AGARD-B stability-derivative validation,
    or full DATCOM validation.
```

## `data/aero/naca-report-1048-body-ii-crossflow-v1.toml`

```yaml
dataset_id:       openbmp.aero.naca_report_1048_body_ii_crossflow.v1
files:
  - data/aero/naca-report-1048-body-ii-crossflow-v1.toml
source_class:     public-domain-naca
source_title:     NACA Report 1048, A Study of Effects of Viscosity on Flow Over Slender Inclined Bodies of Revolution
source_authors:   H. Julian Allen; Edward W. Perkins
source_id:        NACA-TR-1048 / NASA NTRS 19930090962
source_url:       https://ntrs.nasa.gov/citations/19930090962
publication_date: 1951
methodology_reference: >-
  NACA Report 1048 Figure 6 defines body II as a 33 1/3-caliber ogival nose
  plus cylindrical afterbody with fineness ratio 13.1. The appendix gives the
  finite-length crossflow drag ratio 0.692 for body II.
methodology_urls:
  - https://ntrs.nasa.gov/citations/19930090962
license_or_terms: >-
  Public-domain United States Government/NACA report. The fixture stores
  OpenBMP-authored evaluations of the report body geometry and appendix
  finite-length correction; it does not digitize plotted force/moment points.
source_hash_sha256: ee890e2a2606087df55fe26958ff8aa734b7272576ccfdfe0f432c43a18a5f60
retrieved_utc:    2026-06-28
transformation:
  method: >-
    Body diameter is normalized to 1.0, body length to the report fineness
    label 13.1, nose fineness to 4.314 / 0.750 = 5.752, and reference area to
    pi/4. Four Mach-1.98 samples evaluate OpenBMP's finite-length-corrected
    Allen-Perkins crossflow increment and projected-planform centroid at
    4, 8, 12, and 16 degrees angle of attack.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/validation.rs` test
    `naca_report_1048_body_crossflow_fixtures_match_report_geometry` parses
    this TOML fixture and checks four expected CN/cp pairs through the public
    `DatcomComponentBuildup` crossflow seam.
  test: crates/openbmp-aero/src/validation.rs
  tolerance: >-
    Absolute and relative tolerances are both 1.0e-12 for CN and cp.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public report-geometry/equation validation only. This fixture does not
    claim digitized NACA Report 1048 force/moment plot residual closure,
    wind-tunnel uncertainty closure, AGARD-B stability-derivative validation,
    or full DATCOM validation.
```

## `data/aero/naca-report-1048-body-i-force-moment-v1.toml`

```yaml
dataset_id:       openbmp.aero.naca_report_1048_body_i_force_moment.v1
files:
  - data/aero/naca-report-1048-body-i-force-moment-v1.toml
source_class:     public-domain-naca
source_title:     NACA Report 1048, A Study of Effects of Viscosity on Flow Over Slender Inclined Bodies of Revolution
source_authors:   H. Julian Allen; Edward W. Perkins
source_id:        NACA-TR-1048 / NASA NTRS 19930090962
source_url:       https://ntrs.nasa.gov/citations/19930090962
publication_date: 1951
methodology_reference: >-
  NACA Report 1048 Figure 6 defines body I geometry, and equation (15) gives
  the simplified force and bow pitching-moment equations in terms of body
  volume, base area, planform area, and crossflow-drag center.
methodology_urls:
  - https://ntrs.nasa.gov/citations/19930090962
license_or_terms: >-
  Public-domain United States Government/NACA report. The fixture stores
  OpenBMP-authored evaluations of equation (15), not scanned plot data.
source_hash_sha256: ee890e2a2606087df55fe26958ff8aa734b7272576ccfdfe0f432c43a18a5f60
retrieved_utc:    2026-06-28
transformation:
  method: >-
    Body diameter is normalized to 1.0, body length to 21.1, nose fineness to
    4.314 / 0.750 = 5.752, reference area to pi/4, and the moment center to
    the bow. Four Mach-1.98 rows compare CN and bow CM against equation (15)
    at 4, 8, 12, and 16 degrees. Incremental foredrag is covered by the
    companion `naca-report-1048-body-i-incremental-foredrag-v1.toml` fixture.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/validation.rs` test
    `naca_report_1048_force_moment_fixtures_match_equation_15` parses this
    TOML fixture and checks four expected CN/CM pairs through the public
    `DatcomComponentBuildup` coefficient path.
  test: crates/openbmp-aero/src/validation.rs
  tolerance: >-
    Absolute tolerance is 2.0e-2 and relative tolerance is 7.0e-2 for CN and
    bow CM, covering report simplification versus OpenBMP's exact
    trigonometric form and projected-centroid path.
validation_status: research
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public equation-level force/moment validation only. This fixture does not
    claim digitized NACA Report 1048 scanned plot residual closure,
    wind-tunnel uncertainty closure, AGARD-B stability-derivative validation,
    or full DATCOM validation.
```

## `data/aero/naca-report-1048-body-ii-force-moment-v1.toml`

```yaml
dataset_id:       openbmp.aero.naca_report_1048_body_ii_force_moment.v1
files:
  - data/aero/naca-report-1048-body-ii-force-moment-v1.toml
source_class:     public-domain-naca
source_title:     NACA Report 1048, A Study of Effects of Viscosity on Flow Over Slender Inclined Bodies of Revolution
source_authors:   H. Julian Allen; Edward W. Perkins
source_id:        NACA-TR-1048 / NASA NTRS 19930090962
source_url:       https://ntrs.nasa.gov/citations/19930090962
publication_date: 1951
methodology_reference: >-
  NACA Report 1048 Figure 6 defines body II geometry, and equation (15) gives
  the simplified force and bow pitching-moment equations in terms of body
  volume, base area, planform area, and crossflow-drag center.
methodology_urls:
  - https://ntrs.nasa.gov/citations/19930090962
license_or_terms: >-
  Public-domain United States Government/NACA report. The fixture stores
  OpenBMP-authored evaluations of equation (15), not scanned plot data.
source_hash_sha256: ee890e2a2606087df55fe26958ff8aa734b7272576ccfdfe0f432c43a18a5f60
retrieved_utc:    2026-06-28
transformation:
  method: >-
    Body diameter is normalized to 1.0, body length to 13.1, nose fineness to
    4.314 / 0.750 = 5.752, reference area to pi/4, and the moment center to
    the bow. Four Mach-1.98 rows compare CN and bow CM against equation (15)
    at 4, 8, 12, and 16 degrees. Incremental foredrag is covered by the
    companion `naca-report-1048-body-ii-incremental-foredrag-v1.toml` fixture.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/validation.rs` test
    `naca_report_1048_force_moment_fixtures_match_equation_15` parses this
    TOML fixture and checks four expected CN/CM pairs through the public
    `DatcomComponentBuildup` coefficient path.
  test: crates/openbmp-aero/src/validation.rs
  tolerance: >-
    Absolute tolerance is 2.0e-2 and relative tolerance is 7.0e-2 for CN and
    bow CM, covering report simplification versus OpenBMP's exact
    trigonometric form and projected-centroid path.
validation_status: research
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public equation-level force/moment validation only. This fixture does not
    claim digitized NACA Report 1048 scanned plot residual closure,
    wind-tunnel uncertainty closure, AGARD-B stability-derivative validation,
    or full DATCOM validation.
```

## `data/aero/naca-report-1048-body-i-incremental-foredrag-v1.toml`

```yaml
dataset_id:       openbmp.aero.naca_report_1048_body_i_incremental_foredrag.v1
files:
  - data/aero/naca-report-1048-body-i-incremental-foredrag-v1.toml
source_class:     public-domain-naca
source_title:     NACA Report 1048, A Study of Effects of Viscosity on Flow Over Slender Inclined Bodies of Revolution
source_authors:   H. Julian Allen; Edward W. Perkins
source_id:        NACA-TR-1048 / NASA NTRS 19930090962
source_url:       https://ntrs.nasa.gov/citations/19930090962
publication_date: 1951
methodology_reference: >-
  NACA Report 1048 Figure 6 defines body I geometry, and equation (15) gives
  the simplified incremental foredrag expression in terms of base area,
  body planform area, angle of attack, and finite-length crossflow drag
  coefficient.
methodology_urls:
  - https://ntrs.nasa.gov/citations/19930090962
license_or_terms: >-
  Public-domain United States Government/NACA report. The fixture stores
  OpenBMP-authored evaluations of equation (15), not scanned plot data.
source_hash_sha256: ee890e2a2606087df55fe26958ff8aa734b7272576ccfdfe0f432c43a18a5f60
retrieved_utc:    2026-06-28
transformation:
  method: >-
    Body diameter is normalized to 1.0, body length to 21.1, nose fineness to
    4.314 / 0.750 = 5.752, and reference area to pi/4. Four Mach-1.98 rows
    compare CD(alpha) - CD(0) against equation (15) incremental foredrag at
    4, 8, 12, and 16 degrees. The report-equation expected values use the
    report body planform term and low-crossflow-Mach finite-length drag
    coefficient; the runtime path uses contour-projected body area and exact
    sine powers, so the fixture carries the stated report-equation tolerance.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/validation.rs` test
    `naca_report_1048_incremental_foredrag_fixtures_match_equation_15` parses
    this TOML fixture and checks four expected delta-CD values through the
    public `DatcomComponentBuildup` coefficient path. A source-class rejection
    test proves the fixture path is reserved for public-domain NACA inputs.
  test: crates/openbmp-aero/src/validation.rs
  tolerance: >-
    Absolute tolerance is 2.0e-2 and relative tolerance is 7.0e-2 for
    incremental foredrag CD.
validation_status: research
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public equation-level incremental-foredrag validation only. This fixture
    does not claim digitized NACA Report 1048 scanned plot residual closure,
    wind-tunnel uncertainty closure, AGARD-B stability-derivative validation,
    or full DATCOM validation.
```

## `data/aero/naca-report-1048-body-ii-incremental-foredrag-v1.toml`

```yaml
dataset_id:       openbmp.aero.naca_report_1048_body_ii_incremental_foredrag.v1
files:
  - data/aero/naca-report-1048-body-ii-incremental-foredrag-v1.toml
source_class:     public-domain-naca
source_title:     NACA Report 1048, A Study of Effects of Viscosity on Flow Over Slender Inclined Bodies of Revolution
source_authors:   H. Julian Allen; Edward W. Perkins
source_id:        NACA-TR-1048 / NASA NTRS 19930090962
source_url:       https://ntrs.nasa.gov/citations/19930090962
publication_date: 1951
methodology_reference: >-
  NACA Report 1048 Figure 6 defines body II geometry, and equation (15) gives
  the simplified incremental foredrag expression in terms of base area,
  body planform area, angle of attack, and finite-length crossflow drag
  coefficient.
methodology_urls:
  - https://ntrs.nasa.gov/citations/19930090962
license_or_terms: >-
  Public-domain United States Government/NACA report. The fixture stores
  OpenBMP-authored evaluations of equation (15), not scanned plot data.
source_hash_sha256: ee890e2a2606087df55fe26958ff8aa734b7272576ccfdfe0f432c43a18a5f60
retrieved_utc:    2026-06-28
transformation:
  method: >-
    Body diameter is normalized to 1.0, body length to 13.1, nose fineness to
    4.314 / 0.750 = 5.752, and reference area to pi/4. Four Mach-1.98 rows
    compare CD(alpha) - CD(0) against equation (15) incremental foredrag at
    4, 8, 12, and 16 degrees. The report-equation expected values use the
    report body planform term and low-crossflow-Mach finite-length drag
    coefficient; the runtime path uses contour-projected body area and exact
    sine powers, so the fixture carries the stated report-equation tolerance.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/validation.rs` test
    `naca_report_1048_incremental_foredrag_fixtures_match_equation_15` parses
    this TOML fixture and checks four expected delta-CD values through the
    public `DatcomComponentBuildup` coefficient path.
  test: crates/openbmp-aero/src/validation.rs
  tolerance: >-
    Absolute tolerance is 2.0e-2 and relative tolerance is 7.0e-2 for
    incremental foredrag CD.
validation_status: research
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public equation-level incremental-foredrag validation only. This fixture
    does not claim digitized NACA Report 1048 scanned plot residual closure,
    wind-tunnel uncertainty closure, AGARD-B stability-derivative validation,
    or full DATCOM validation.
```

## `data/aero/naca-report-1048-crossflow-drag-ratio-v1.toml`

```yaml
dataset_id:       openbmp.aero.naca_report_1048_crossflow_drag_ratio.v1
files:
  - data/aero/naca-report-1048-crossflow-drag-ratio-v1.toml
source_class:     public-domain-naca
source_title:     NACA Report 1048, A Study of Effects of Viscosity on Flow Over Slender Inclined Bodies of Revolution
source_authors:   H. Julian Allen; Edward W. Perkins
source_id:        NACA-TR-1048 / NASA NTRS 19930090962
source_url:       https://ntrs.nasa.gov/citations/19930090962
publication_date: 1951
methodology_reference: >-
  The appendix titled "Estimation of Cross-Flow Drag Coefficient" gives the
  finite-length circular-cylinder drag correction ratios for the two ogival
  bodies used in the report comparisons: 0.755 for body I and 0.692 for body
  II. The same appendix states the low-Mach circular-cylinder drag coefficient
  used for the estimate is 1.2.
methodology_urls:
  - https://ntrs.nasa.gov/citations/19930090962
license_or_terms: >-
  Public-domain United States Government/NACA report. The fixture stores only
  the two appendix ratios and OpenBMP-authored products of those ratios with
  the stated low-Mach circular-cylinder drag coefficient.
source_hash_sha256: ee890e2a2606087df55fe26958ff8aa734b7272576ccfdfe0f432c43a18a5f60
retrieved_utc:    2026-06-28
transformation:
  method: >-
    The fixture records body I and body II report fineness labels, the
    appendix finite-length drag ratios, the stated low-Mach cylinder-section
    drag coefficient 1.2, and the derived Cd_alpha=90 values: 1.2 * 0.755 =
    0.906 and 1.2 * 0.692 = 0.8304.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/validation.rs` test
    `naca_report_1048_crossflow_drag_ratio_fixture_matches_appendix_values`
    parses this TOML fixture and checks the two derived Cd_alpha=90 values.
    A source-class rejection test proves the fixture path is reserved for
    public-domain NACA inputs.
  test: crates/openbmp-aero/src/validation.rs
  tolerance: >-
    Absolute and relative tolerances are both 1.0e-12 for Cd_alpha=90.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public appendix-value validation only. This fixture does not claim
    digitized NACA Report 1048 force/moment plot closure, wind-tunnel
    uncertainty closure, AGARD-B stability-derivative validation, or full
    DATCOM validation.
```

## `data/aero/sears-haack-area-rule-wave-drag-v1.toml`

```yaml
dataset_id:       openbmp.aero.sears_haack_area_rule_wave_drag.v1
files:
  - data/aero/sears-haack-area-rule-wave-drag-v1.toml
source_class:     synthetic-openbmp-public-methodology
source_title:     Synthetic Sears-Haack / area-rule wave-drag equation-shape fixture
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic method fixture
source_url:       https://github.com/openbmp/openbmp/blob/main/data/aero/sears-haack-area-rule-wave-drag-v1.toml
publication_date: 2026-06-28
methodology_reference: >-
  OpenBMP's bounded Sears-Haack-style area-rule wave-drag branch for a simple
  fineness-6 Von Karman forebody. The fixture checks the subsonic zero branch,
  the smooth transonic blend, and the supersonic 1/sqrt(M^2-1) decay.
methodology_urls: []
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain. No third-party
  aerodynamic data is incorporated.
source_hash_sha256: not-applicable; synthetic in-tree model fixture
retrieved_utc:    2026-06-28
transformation:
  method: >-
    Expected CD_wave values are evaluated from the current public
    `DatcomComponentBuildup::area_rule_wave_drag_coefficient` relation over
    Mach 0.5, 1.0, 1.2, 2.0, and 3.5 for a fineness-6 Von Karman nose.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/validation.rs` test
    `sears_haack_wave_drag_fixture_matches_area_rule_branch` parses this TOML
    fixture and checks five expected CD_wave samples. A negative expected-drag
    test proves malformed fixture values fail closed.
  test: crates/openbmp-aero/src/validation.rs
  tolerance: >-
    Absolute and relative tolerances are both 1.0e-12 for CD_wave.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic method/equation-shape fixture only; no wind-tunnel, CFD, NACA
    plot, or Missile-DATCOM numeric data. This fixture does not claim
    calibrated wave-drag closure.
```

## `data/aero/datcom-fin-body-carryover-v1.toml`

```yaml
dataset_id:       openbmp.aero.datcom_fin_body_carryover.v1
files:
  - data/aero/datcom-fin-body-carryover-v1.toml
source_class:     synthetic-openbmp-public-methodology
source_title:     Synthetic DATCOM fin/body carryover factor equation-shape fixture
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic method fixture
source_url:       https://github.com/openbmp/openbmp/blob/main/data/aero/datcom-fin-body-carryover-v1.toml
publication_date: 2026-06-28
methodology_reference: >-
  OpenBMP's bounded DATCOM-style fin-on-body and body-on-fin carryover factor
  branch for three body-radius to fin-span ratios. The fixture checks a small
  radius/span case, a unity radius/span case, and the clamped upper branch.
methodology_urls: []
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain. No third-party
  aerodynamic data is incorporated.
source_hash_sha256: not-applicable; synthetic in-tree model fixture
retrieved_utc:    2026-06-28
transformation:
  method: >-
    Expected factor values are evaluated from the current public
    `DatcomComponentBuildup::fin_body_carryover_factors` relation for
    radius/span ratios 0.2, 1.0, and the clamped 2.0 branch.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/validation.rs` test
    `datcom_fin_body_carryover_fixture_matches_factor_branch` parses this TOML
    fixture and checks three expected fin-on-body/body-on-fin factor pairs. A
    negative-span test proves malformed fixture values fail closed.
  test: crates/openbmp-aero/src/validation.rs
  tolerance: >-
    Absolute and relative tolerances are both 1.0e-12 for each factor.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic method/equation-shape fixture only; no wind-tunnel, CFD, NACA,
    AGARD, or Missile-DATCOM numeric data. This fixture does not claim
    calibrated carryover closure.
```

## `data/aero/datcom-protuberance-drag-v1.toml`

```yaml
dataset_id:       openbmp.aero.datcom_protuberance_drag.v1
files:
  - data/aero/datcom-protuberance-drag-v1.toml
source_class:     synthetic-openbmp-public-methodology
source_title:     Synthetic DATCOM equivalent-protuberance drag equation-shape fixture
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic method fixture
source_url:       https://github.com/openbmp/openbmp/blob/main/data/aero/datcom-protuberance-drag-v1.toml
publication_date: 2026-06-28
methodology_reference: >-
  OpenBMP's bounded equivalent-axisymmetric protuberance drag branch for a
  simple declared protuberance set. The fixture checks the subsonic Mach
  factor, the transonic smooth blend, and the supersonic branch.
methodology_urls: []
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain. No third-party
  aerodynamic data is incorporated.
source_hash_sha256: not-applicable; synthetic in-tree model fixture
retrieved_utc:    2026-06-28
transformation:
  method: >-
    Expected CD_protuberance values are evaluated from the current public
    `DatcomComponentBuildup::protuberance_drag_coefficient` relation for Mach
    0.0, 0.5, 1.0, and 2.0 with two declared 0.015 m^2 protuberances.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/validation.rs` test
    `datcom_protuberance_drag_fixture_matches_mach_branch` parses this TOML
    fixture and checks four expected CD_protuberance samples. A negative
    expected-drag test proves malformed fixture values fail closed.
  test: crates/openbmp-aero/src/validation.rs
  tolerance: >-
    Absolute and relative tolerances are both 1.0e-12 for CD_protuberance.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic method/equation-shape fixture only; no wind-tunnel, CFD,
    Missile-DATCOM, or flight data. This fixture does not claim calibrated
    protuberance closure.
```

## `data/aero/vlm-fin-lift-hinge-trend-v1.toml`

```yaml
dataset_id:       openbmp.aero.vlm_fin_lift_hinge_trend.v1
files:
  - data/aero/vlm-fin-lift-hinge-trend-v1.toml
source_class:     synthetic-openbmp
source_title:     Synthetic first VLM-fin lift and hinge-moment trend fixture
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic method fixture
source_url:       https://github.com/openbmp/openbmp/blob/main/data/aero/vlm-fin-lift-hinge-trend-v1.toml
publication_date: 2026-06-28
methodology_reference: >-
  OpenBMP's first WP-03.5 attached-flow fin lifting-surface substrate for one
  trapezoidal fin using the same NASA SP-405-style first finite-wing
  lift-slope relation as the runtime path. The fixture checks deterministic
  lift-curve slope, lift, pitching moment, and hinge-moment sign/monotonicity
  over neutral and +/-5 degree control deflections. Grid-fin authority,
  horseshoe-vortex influence/residual, derivative-channel,
  subsonic-compressibility scaling, and operand-order tests are code-only in
  `crates/openbmp-aero/src/vlm.rs` and do not add external data.
methodology_urls: []
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain. No third-party
  aerodynamic data is incorporated.
source_hash_sha256: not-applicable; synthetic in-tree model fixture
retrieved_utc:    2026-06-28
transformation:
  method: >-
    Expected coefficients are evaluated from the current public
    `VlmFinSurface::evaluate` relation for the declared trapezoidal fin,
    Mach 0.3, alpha 5 degrees, and +/-5 degree control deflections.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/vlm.rs` test
    `vlm_fin_fixture_matches_lift_and_hinge_trends` parses this TOML fixture
    and checks lift-curve slope, CL, Cm, and hinge-moment coefficient values.
    Other focused tests prove deterministic repeated finite-wing evaluation,
    hinge-moment sign/monotonicity with deflection, fail-closed
    geometry/transonic-flow validation, deterministic horseshoe-vortex lattice
    solves with small flow-tangency residual, and horseshoe hinge-moment trends.
  test: crates/openbmp-aero/src/vlm.rs
  tolerance: >-
    Relative error <= 1.0e-12 for every fixture-backed coefficient.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic first-substrate fixture only; no AVL, wind-tunnel, CFD, NACA, or
    flight data. This fixture does not claim grid-fin or AVL code-to-code
    closure.
```

## `data/aero/vlm-finite-wing-analytic-v1.toml`

```yaml
dataset_id:       openbmp.aero.vlm_finite_wing_analytic.v1
files:
  - data/aero/vlm-finite-wing-analytic-v1.toml
source_class:     analytic-openbmp
source_title:     Analytic rectangular finite-wing lift-curve fixture
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        analytic finite-wing relation aligned with NASA SP-405 Figure 5
source_url:       https://github.com/openbmp/openbmp/blob/main/data/aero/vlm-finite-wing-analytic-v1.toml
publication_date: 2026-06-28
methodology_reference: >-
  Rectangular attached-flow finite-wing relation used as an analytic check for
  the first WP-03.5 fin lifting-surface substrate:
  CL_alpha = 2*pi*A / (A + 2.903 + 0.027/A) at M = 0, matching the public
  NASA SP-405 Figure 5 rectangular-planform formula. The fixture uses
  AR = 4, alpha = 4 deg, and the OpenBMP reference area equal to the fin
  planform area.
methodology_urls:
  - https://ntrs.nasa.gov/citations/19760021075
license_or_terms: >-
  OpenBMP-authored analytic fixture values derived from a public-domain NASA
  formula; no scanned plot/table ordinates are copied.
source_hash_sha256: not-applicable; analytic fixture generated in-tree
retrieved_utc:    2026-06-28
transformation:
  method: >-
    Expected aspect ratio, lift-curve slope, CL, induced drag, and
    aerodynamic-center location are evaluated from the closed-form NASA SP-405
    rectangular-wing formula and rectangular quarter-chord geometry. The
    induced-drag expected value uses OpenBMP's documented first-substrate
    Oswald efficiency constant.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/vlm.rs` test
    `vlm_finite_wing_analytic_fixture_matches_lift_slope_and_induced_drag`
    parses this TOML fixture, evaluates `VlmFinSurface`, and checks aspect
    ratio, lift-curve slope, CL, induced drag, and aerodynamic-center location.
  test: crates/openbmp-aero/src/vlm.rs
  tolerance: >-
    Relative error <= 1.0e-12 for nonzero scalar coefficients and absolute
    error <= 1.0e-12 for aerodynamic-center location.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Analytic method fixture only; no AVL, wind-tunnel, CFD, NACA numeric, or
    flight data. This improves finite-wing coefficient coverage but does not
    claim WP-03.5 AVL code-to-code or NACA tabular lift-curve closure.
```

## `data/aero/nasa-sp405-rectangular-wing-lift-slope-v1.toml`

```yaml
dataset_id:       openbmp.aero.nasa_sp405_rectangular_wing_lift_slope.v1
files:
  - data/aero/nasa-sp405-rectangular-wing-lift-slope-v1.toml
source_class:     public-domain-nasa
source_title:     NASA SP-405, Vortex-Lattice Utilization
source_authors:   NASA Langley Research Center contributors
source_id:        NASA-SP-405 / NASA NTRS 19760021075
source_url:       https://ntrs.nasa.gov/citations/19760021075
publication_date: 1976
methodology_reference: >-
  NASA SP-405 Figure 5 lists lift-curve-slope formulas for rectangular
  planforms. The fixture uses the Hough rectangular-planform expression
  CL_alpha = 2*pi*A / (A + 2.903 + 0.027/A), noted in the report for
  16 >= A > 1, as a public formula check for the first finite-wing substrate.
methodology_urls:
  - https://ntrs.nasa.gov/citations/19760021075
  - https://ntrs.nasa.gov/api/citations/19760021075/downloads/19760021075.pdf
license_or_terms: >-
  Public-domain United States Government/NASA report. The fixture stores
  OpenBMP-authored evaluations of the report formula, not scanned plot data.
source_hash_sha256: not-recorded; public report URL is cited
retrieved_utc:    2026-06-28
transformation:
  method: >-
    The fixture normalizes a rectangular AR=4 wing to root chord 1, tip chord
    1, span 4, reference area 4, and M=0. The expected lift-curve slope is
    evaluated directly from the NASA SP-405 Figure 5 formula; expected CL,
    induced drag, and aerodynamic-center values follow the current OpenBMP
    first-substrate coefficient conventions.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/vlm.rs` test
    `vlm_nasa_sp405_rectangular_wing_fixture_matches_lift_slope_formula`
    parses this TOML fixture, independently recomputes the NASA SP-405
    lift-curve-slope formula, evaluates `VlmFinSurface`, and checks slope,
    CL, induced drag, and aerodynamic-center location.
  test: crates/openbmp-aero/src/vlm.rs
  tolerance: >-
    Relative error <= 1.0e-12 for nonzero scalar coefficients and absolute
    error <= 1.0e-12 for aerodynamic-center location.
validation_status: research
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public formula-level validation only. This fixture does not claim AVL
    code-to-code agreement, calibrated grid-fin authority, NACA tabular
    lift-curve closure, wind-tunnel agreement, CFD agreement, or flight
    validation.
```

## `data/aero/naca-report-921-vlm-method-v1.toml`

```yaml
dataset_id:       openbmp.aero.naca_report_921_vlm_method.v1
files:
  - data/aero/naca-report-921-vlm-method-v1.toml
source_class:     public-domain
source_title:     Theoretical Symmetric Span Loading at Subsonic Speeds for Wings Having Arbitrary Plan Form
source_authors:   John DeYoung; Charles W. Harper
source_id:        NACA-TR-921; NASA NTRS 19930091985
source_url:       https://ntrs.nasa.gov/citations/19930091985
publication_date: 1948
methodology_reference: >-
  NACA Report 921 documents a vortex-lattice-style method for symmetric span
  loading at subsonic speeds, including straight quarter-chord reference
  geometry and lift/loading outputs for arbitrary planforms. This fixture uses
  only the method convention relevant to OpenBMP's first horseshoe-vortex
  substrate: bound vortex at local quarter chord and control point at local
  three-quarter chord on a straight-quarter-chord rectangular lattice.
methodology_urls:
  - https://ntrs.nasa.gov/citations/19930091985
  - https://ntrs.nasa.gov/api/citations/19930091985/downloads/19930091985.pdf
license_or_terms: >-
  Public-domain NACA/NASA technical report metadata and method convention.
  The fixture contains OpenBMP-authored scalar expectations derived from the
  public method convention, not copied report tables.
source_hash_sha256: not-recorded; public report URL is cited
retrieved_utc:    2026-06-28
transformation:
  method: >-
    A rectangular unswept lattice with one chordwise row and four spanwise
    strips is constructed so that local chord fraction can be checked directly.
    Expected values pin the quarter-chord bound-vortex location, the
    three-quarter-chord control-point location, upward panel normal, and panel
    count.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/vlm.rs` test
    `vlm_naca_report_921_method_fixture_pins_panel_convention` parses this
    TOML fixture, builds `VlmHorseshoeLattice`, and verifies the public
    quarter-/three-quarter-chord panel convention through the public
    `panels()` view.
  test: crates/openbmp-aero/src/vlm.rs
  tolerance: Absolute error <= 1.0e-12 for fixture-backed geometry checks.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public method-convention fixture only. This does not claim AVL
    code-to-code agreement, calibrated grid-fin authority, public NACA tabular
    lift-curve closure, wind-tunnel agreement, CFD agreement, or flight
    validation.
```

## `data/aero/vlm-horseshoe-derivative-comparison-v1.toml`

```yaml
dataset_id:       openbmp.aero.vlm_horseshoe_derivative_comparison.v1
files:
  - data/aero/vlm-horseshoe-derivative-comparison-v1.toml
source_class:     synthetic-openbmp
source_title:     Synthetic VLM horseshoe derivative comparison fixture
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        synthetic derivative comparison fixture
source_url:       https://github.com/openbmp/openbmp/blob/main/data/aero/vlm-horseshoe-derivative-comparison-v1.toml
publication_date: 2026-06-28
methodology_reference: >-
  OpenBMP's parser-gated WP-03.5 derivative comparison harness for
  horseshoe-vortex CL-alpha, Cm-alpha, hinge-alpha, control-lift, and
  control-hinge tolerance rows. The fixture pins schema parsing, tolerance
  reporting, and deterministic derivative output for one checked in-tree case.
methodology_urls: []
license_or_terms: >-
  Synthetic OpenBMP-authored content; CC0 / public domain. No third-party
  aerodynamic data is incorporated.
source_hash_sha256: not-applicable; synthetic in-tree model fixture
retrieved_utc:    2026-06-28
transformation:
  method: >-
    Expected derivative values are evaluated from
    `VlmHorseshoeLattice::finite_difference_derivatives` for the declared
    trapezoidal fin, Mach 0.3, alpha 5 degrees, zero control deflection, and
    0.25-degree central-difference steps.
  script: none
  script_hash_sha256: not-applicable
verification:
  method: >-
    `crates/openbmp-aero/src/vlm.rs` test
    `vlm_horseshoe_derivative_comparison_fixture_reports_expected_values`
    parses this TOML fixture through
    `VlmHorseshoeDerivativeComparisonDeck`, evaluates the public derivative
    API, checks five derivative channels against absolute/relative tolerances,
    and verifies deterministic report counts. The paired
    `vlm_horseshoe_derivative_comparison_rejects_empty_samples` test proves
    empty comparison fixtures fail closed.
  test: crates/openbmp-aero/src/vlm.rs
  tolerance: Absolute and relative tolerance <= 1.0e-12.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic harness fixture only. This does not claim AVL code-to-code
    agreement, public NACA tabular lift-curve closure, calibrated grid-fin
    authority, wind-tunnel agreement, CFD agreement, or flight validation.
```
