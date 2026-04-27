# Provenance — `scenarios/sounding-rocket/`

Canonical OpenBMP provenance record for the sounding-rocket
scenarios shipped under `scenarios/sounding-rocket/`. See
[`docs/data-provenance.md`](../../docs/data-provenance.md) for the
contract this record satisfies.

## `scenarios/sounding-rocket/niskanen-2009-chapter6.toml`

```yaml
dataset_id:       openbmp.scenario.sounding_rocket.niskanen_2009_chapter6.v1
files:
  - scenarios/sounding-rocket/niskanen-2009-chapter6.toml
source_class:     converted-public
source_title:     >-
  Phase-2.10 canonical scenario form of the Niskanen 2009
  Chapter-6 small-rocket benchmark, expressed in the OpenBMP
  scenario schema. References the Phase-2.6 Estes C6 motor file
  and the Phase-2.5 reduced-CD aero deck via SHA-256-pinned
  paths.
source_authors:   Niskanen, S. (2009 thesis); OpenBMP (Dmitri Arekhta) for the OpenBMP-schema rendering
source_id:        Niskanen 2009 Chapter 6 small-rocket benchmark
source_url:       https://openrocket.sourceforge.net/thesis.pdf
publication_date: 2009-05-20
methodology_reference: >-
  Niskanen, S. (2009). *Development of an Open-Source Model Rocket
  Simulation Software*. Master's thesis, Helsinki University of
  Technology. Chapter 6 documents a small-rocket benchmark
  comparing simulated apogees against flight experiments for both
  Estes B4 and C6 motors. The OpenBMP scenario instantiates the
  C6 case with the published 80 g airframe mass, constant axial
  CD = 0.8 reduction, and the same vertical-launch convention used
  by the Phase-2.9 integration test in
  `crates/openbmp-vehicle/tests/sounding_rocket.rs`.
methodology_urls:
  - https://openrocket.sourceforge.net/thesis.pdf
license_or_terms: >-
  The Niskanen thesis is distributed under terms permitting
  academic re-use with attribution. The OpenBMP scenario file is a
  schema rendering of the published geometry, mass, and reference
  apogees; no derivative works restrictions apply to the numerical
  facts themselves. OpenBMP credits Niskanen as the source.
retrieved_utc:    2026-04-27
transformation:
  method: >-
    Render the already-provenanced Phase-2.9 Niskanen C6 benchmark
    inputs into the Phase-2.10 scenario schema. The scenario file
    references the aero deck and motor data by relative path and
    pins each reference with the SHA-256 digest shipped in the same
    commit. Launch-site coordinates use the documented Helsinki
    proxy because the thesis does not publish exact coordinates.
  script: none
verification:
  method: >-
    `openbmp check` parses the scenario and resolves the pinned aero
    deck and motor file digests. The Phase-2.9 sounding-rocket
    integration test independently replays the same Niskanen C6
    reduction and compares simulated apogee against the published
    151.5 m experimental value with ±5% tolerance. The Phase-2.11
    runner will consume this scenario file directly.
  test: >-
    crates/openbmp-cli/tests/cli_snapshots.rs;
    crates/openbmp-vehicle/tests/sounding_rocket.rs
  tolerance: >-
    Parser/check path requires exact SHA-256 pin matches. Physics
    regression tolerance remains the Phase-2.9 ±5% apogee envelope
    against the published C6 experimental value.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public academic benchmark from a master's thesis. The modeled
    object is a hobby-class sounding/model rocket reduction, not an
    operational vehicle. The scenario contains no targeting, no
    real device drivers, and no restricted or fielded-vehicle data.
units:            metres, metres-per-second, metres-per-second-squared, seconds, kilograms, degrees
frame_profile:    wgs84-uniform-rotation
local_origin:     >-
  Helsinki proxy site (latitude 60.18°, longitude 24.83°, height
  0.0 m). Niskanen Chapter 6 does not publish exact launch-site
  coordinates; this proxy is documented in the scenario's
  `[frames.local_origin].source` field.
related_files:
  - data/aero/synthetic-niskanen-ch6-rocket.toml
  - data/motors/estes-c6-eng-derived.toml
  - crates/openbmp-vehicle/tests/sounding_rocket.rs
  - crates/openbmp-cli/tests/sounding_rocket_e2e.rs
notes: >-
  Each external file reference (`aero.deck`, `propulsion.motor.file`)
  carries an optional `*_sha256` pin that fails closed on mismatch.
  The pin values match the data files at the commit shipping this
  scenario; if a referenced file is updated, recompute the digest
  with `shasum -a 256 <file>` and update the pin.
```

## `scenarios/sounding-rocket/niskanen-2009-chapter6-rigid.toml`

```yaml
dataset_id:       openbmp.scenario.sounding_rocket.niskanen_2009_chapter6_rigid.v1
files:
  - scenarios/sounding-rocket/niskanen-2009-chapter6-rigid.toml
source_class:     converted-public
source_title:     >-
  Phase-3.1 rigid-body variant of the canonical Niskanen 2009
  Chapter-6 small-rocket benchmark. Reuses the same Phase-2.5 aero
  deck and Phase-2.6 Estes C6 motor file as the point-mass scenario
  but declares `vehicle.kind = "rigid_body"` with rigid-body initial
  state (identity orientation, zero angular velocity) and an
  academic rod-shaped inertia tensor sized to the 56 cm × 80 g body
  geometry.
source_authors:   Niskanen, S. (2009 thesis); OpenBMP (Dmitri Arekhta) for the OpenBMP-schema rendering and the rigid-body variant
source_id:        Niskanen 2009 Chapter 6 small-rocket benchmark, rigid-body variant
source_url:       https://openrocket.sourceforge.net/thesis.pdf
publication_date: 2009-05-20
methodology_reference: >-
  Niskanen, S. (2009). *Development of an Open-Source Model Rocket
  Simulation Software*. Master's thesis, Helsinki University of
  Technology. Chapter 6 documents a small-rocket benchmark
  comparing simulated apogees against flight experiments for both
  Estes B4 and C6 motors. The Phase-3.1 OpenBMP variant reuses the
  identical aero deck, motor data, mass, and launch-site convention
  as the canonical point-mass scenario, but exercises the rigid-body
  kernel adapter family by adding orientation, angular velocity, and
  an inertia tensor. With identity initial orientation and the
  axisymmetric `ZeroMoment` model (gravity through CG, axial thrust,
  axisymmetric drag), the rigid-body trajectory matches the
  point-mass trajectory within IEEE 754 reduction-order noise.
methodology_urls:
  - https://openrocket.sourceforge.net/thesis.pdf
license_or_terms: >-
  The Niskanen thesis is distributed under terms permitting
  academic re-use with attribution. The OpenBMP rigid-body variant
  introduces no new published benchmark data; the inertia tensor is
  an academic rod-shape estimate sized to the published body
  geometry and is documented in-line in the scenario file. Phase-3.6
  motor inertia derivatives and Phase-3.7 tank/slosh refinements
  will replace the placeholder inertia values with first-principles
  derivations.
retrieved_utc:    2026-04-27
transformation:
  method: >-
    Clone the Phase-2.10 Niskanen C6 scenario file and replace
    `vehicle.kind = "point_mass"` with `vehicle.kind =
    "rigid_body"`. Add the four required rigid-body initial-state
    fields: identity quaternion `[0, 0, 0, 1]` (xyzw), zero angular
    velocity, zero initial position/velocity (vertical launch
    convention shared with the point-mass scenario), and a
    rod-shaped inertia tensor `diag(0.00209, 0.00209, 0.00002)
    kg·m²` sized to a 56 cm × 80 g airframe (`I_xx = I_yy =
    (1/12)·m·L²` for a thin rod transverse to its axis; `I_zz` uses
    a small body-radius approximation). Bump the seed root by one
    byte (last byte 'R' instead of 'n') to keep the rigid and
    point-mass replays distinguishable. The same SHA-256-pinned aero
    deck and motor file are reused.
  script: none
verification:
  method: >-
    `openbmp check` parses the scenario and resolves the pinned aero
    deck and motor file digests. The Phase-3.1 sounding-rocket
    end-to-end test
    (`run_on_niskanen_rigid_apogee_within_tolerance`) asserts the
    rigid-body apogee remains within ±5% of the published 151.5 m
    experimental value, and the cross-path test
    (`niskanen_rigid_and_point_mass_apogees_agree_within_one_percent`)
    asserts the rigid and point-mass paths agree within 1% relative
    apogee — the design contract that justifies introducing the
    rigid-body kernel without changing physics for axisymmetric
    cases at identity orientation.
  test: >-
    crates/openbmp-cli/tests/sounding_rocket_e2e.rs;
    crates/openbmp-cli/tests/cli_snapshots.rs
  tolerance: >-
    Parser/check path requires exact SHA-256 pin matches. Physics
    regression tolerance is ±5% apogee against the published C6
    experimental value (matching the point-mass scenario), plus a
    1% cross-path equivalence between the rigid and point-mass
    apogees at identity initial orientation.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public academic benchmark from a master's thesis, rendered as
    the rigid-body variant of an already-accepted scenario. The
    modeled object is a hobby-class sounding/model rocket reduction,
    not an operational vehicle. The scenario contains no targeting,
    no real device drivers, and no restricted or fielded-vehicle
    data. The inertia tensor is an academic rod-shape estimate, not
    a measured property of any specific airframe.
units:            metres, metres-per-second, metres-per-second-squared, seconds, kilograms, kilogram-metres-squared, degrees, dimensionless quaternion
frame_profile:    wgs84-uniform-rotation
local_origin:     >-
  Helsinki proxy site (latitude 60.18°, longitude 24.83°, height
  0.0 m). Inherited from the point-mass scenario; Niskanen Chapter
  6 does not publish exact launch-site coordinates.
related_files:
  - scenarios/sounding-rocket/niskanen-2009-chapter6.toml
  - data/aero/synthetic-niskanen-ch6-rocket.toml
  - data/motors/estes-c6-eng-derived.toml
notes: >-
  The rigid-body initial state uses the scenario quaternion
  convention `[x, y, z, w]`. The runner converts this to
  `nalgebra::Quaternion::new(w, x, y, z)` at kernel construction
  time. Phase-3.1 deliberately exercises only identity orientation
  and the `ZeroMoment` moment model — Phase-3.2 onward will
  introduce non-trivial body-frame moments (aero coefficient deck,
  thrust offset, slosh) that produce attitude dynamics.
```

## `scenarios/sounding-rocket/niskanen-2009-chapter6-with-mission.toml`

```yaml
dataset_id:       openbmp.scenario.sounding_rocket.niskanen_2009_chapter6_with_mission.v1
files:
  - scenarios/sounding-rocket/niskanen-2009-chapter6-with-mission.toml
source_class:     converted-public
source_title:     >-
  Phase-3.2 mission-block variant of the canonical Niskanen 2009
  Chapter-6 scenario. Same physics as the point-mass scenario at
  `niskanen-2009-chapter6.toml`, plus a `[mission]` block declaring
  two phases (`ascent`, `descent`), one event (`at_apogee_marker`,
  `AtApogee` trigger + `emit_telemetry_marker` action), and a
  single transition from ascent to descent on the apogee event.
  Exercises the Phase-3.2 declarative event-driven scheduling
  surface end-to-end through the runner.
source_authors:   Niskanen, S. (2009 thesis); OpenBMP (Dmitri Arekhta) for the OpenBMP-schema rendering and the Phase-3.2 mission variant
source_id:        Niskanen 2009 Chapter 6 small-rocket benchmark, Phase-3.2 mission variant
source_url:       https://openrocket.sourceforge.net/thesis.pdf
publication_date: 2009-05-20
methodology_reference: >-
  Niskanen, S. (2009). *Development of an Open-Source Model Rocket
  Simulation Software*. Master's thesis, Helsinki University of
  Technology. Chapter 6 documents a small-rocket benchmark
  comparing simulated apogees against flight experiments for both
  Estes B4 and C6 motors. The Phase-3.2 mission variant adds the
  declarative scheduling block on top of the point-mass scenario;
  the simulated trajectory is identical to the point-mass case
  modulo the new `mission.marker.at_apogee_marker` `bool`
  telemetry channel that flips `true` on the apogee step.
methodology_urls:
  - https://openrocket.sourceforge.net/thesis.pdf
license_or_terms: >-
  Same as the canonical Niskanen scenario: the thesis is
  distributed under terms permitting academic re-use with
  attribution; OpenBMP credits Niskanen as the source.
retrieved_utc:    2026-04-27
transformation:
  method: >-
    Clone the Phase-2.10 Niskanen C6 scenario file and append a
    Phase-3.2 `[mission]` block declaring two phases (ascent,
    descent), one apogee event (`AtApogee` trigger +
    `emit_telemetry_marker` action), and one transition from
    ascent to descent on the apogee event. Bump the seed root by
    one byte (last byte 'M' for mission) so the rigid, mission,
    and point-mass replays are distinguishable. The same SHA-256-
    pinned aero deck and motor file are reused.
  script: none
verification:
  method: >-
    `openbmp check` parses the scenario and resolves the pinned
    aero deck and motor file digests. The Phase-3.2 e2e test
    `niskanen_with_mission_emits_apogee_marker` asserts the
    runner allocates a `mission.marker.at_apogee_marker` `bool`
    channel and writes `true` exactly once on the apogee step,
    `false` everywhere else; the apogee-altitude envelope is the
    same ±5% gate as the point-mass scenario.
  test: >-
    crates/openbmp-cli/tests/sounding_rocket_e2e.rs
  tolerance: >-
    Parser/check path requires exact SHA-256 pin matches. Physics
    regression tolerance remains the ±5% apogee envelope against
    the published Niskanen C6 experimental value.
validation_status: checked
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Public academic benchmark from a master's thesis, rendered as
    the mission-block variant of an already-accepted scenario. No
    targeting, no real device drivers, no restricted or fielded-
    vehicle data.
units:            metres, metres-per-second, metres-per-second-squared, seconds, kilograms, degrees
frame_profile:    wgs84-uniform-rotation
local_origin:     >-
  Helsinki proxy site (latitude 60.18°, longitude 24.83°, height
  0.0 m). Inherited from the point-mass scenario.
related_files:
  - scenarios/sounding-rocket/niskanen-2009-chapter6.toml
  - data/aero/synthetic-niskanen-ch6-rocket.toml
  - data/motors/estes-c6-eng-derived.toml
  - crates/openbmp-cli/tests/sounding_rocket_e2e.rs
notes: >-
  The mission block uses the canonical Phase-3.2 vocabulary: phase
  ids and event ids are scenario-text identifiers; the runner
  derives stable `PhaseId` / `EventId` values via FNV-1a-64 of the
  canonical paths `mission.phases.<id>` and `mission.events.<id>`.
  Reordering `[[mission.phases]]` / `[[mission.events]]` /
  `[[mission.transitions]]` blocks does not change the simulator's
  output bytes — this is the load-bearing declaration-order-
  independence determinism contract.
```
