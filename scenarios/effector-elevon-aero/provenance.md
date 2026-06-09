# Provenance — `scenarios/effector-elevon-aero/`

Canonical OpenBMP provenance record for the effector-aero
exit-criterion scenario shipped under
`scenarios/effector-elevon-aero/`.

## `scenarios/effector-elevon-aero/single-elevon-aero-deflected.toml`

```yaml
dataset_id:       openbmp.scenario.effector_elevon_aero.single_elevon_aero_deflected.v1
files:
  - scenarios/effector-elevon-aero/single-elevon-aero-deflected.toml
source_class:     synthetic-openbmp
source_title:     >-
  Exit-criterion scenario. Point-mass vehicle with one
  elevon-shaped linear actuator declared with `unit = "deg"`. The
  scenario's `[aero]` block points at the Schema-2 deck
  `data/aero/synthetic-elevon-1d.toml`, which declares a
  `delta_e_deg` effector axis. The runner's `EffectorRack` snapshot
  flows through the kernel's `EffectorActualsView` into the deck
  lookup, so the actual rate-limited deflection visibly perturbs
  the drag coefficient `CD` (Schema-2 effector-axis consumption
  contract).
source_authors:   OpenBMP (Dmitri Arekhta)
source_id:        Synthetic OpenBMP effector-aero fixture
publication_date: 2026-04-28
methodology_reference: >-
  `docs/scenario-format.md` § Schema-2 aero decks
  documents the schema-2 aero deck format, the kernel-side
  `EffectorActualsView` consumed by `ForceContext`, and the
  runner-side `aero_effector_match` matcher. This scenario is the
  exit-criterion case: a scenario declaring a Schema-2
  deck and a matching effector
  produces aero-force telemetry that visibly differs between a
  zero-deflection baseline and a deflected-elevon run. The deck's
  `0.010 · |δ_e_deg|` perturbation on `CD` is illustrative round
  numbers, not derived from any specific airframe.
methodology_urls:
  - —
license_or_terms: >-
  Synthetic OpenBMP-authored scenario; no external license.
retrieved_utc:    2026-04-28
transformation:
  method: >-
    Authored by hand for the effector-aero exit criterion. No script.
  script: none
verification:
  method: >-
    `openbmp check` parses the scenario; the e2e test
    `single_elevon_aero_scenario_runs_to_completion` runs the
    scenario via `openbmp run`, asserts the kernel completes with
    `StopReason::EndTime`, and asserts the per-step aero-force
    telemetry differs between a zero-deflection baseline run (the
    same scenario with the elevon's `after` value swapped to 0°)
    and the deflected run by more than the 5% tolerance pinned in
    the test.
  test:   crates/openbmp-cli/tests/effector_aero_e2e.rs
  tolerance: >-
    Aero-force magnitude relative difference between baseline and
    deflected runs at `t = 2.5 s`: ≥ 5% (CD goes from 0.45 to ~0.65
    at the deflected slice; the baseline-vs-deflected force ratio
    is dominated by the CD ratio after shared velocity / altitude
    transients).
validation_status: experimental
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic round-number scenario with no real device drivers or fielded-vehicle data. Demonstrates
    the new Schema-2 deck-effector consumption surface; does not
    encode any operational physical-vehicle parameters.
units:            metres, metres-per-second, metres-per-second-squared, seconds, kilograms, degrees
frame_profile:    wgs84-uniform-rotation
local_origin:     >-
  None — the kernel uses the standard vertical-launch ECI `+z`
  convention; the scenario's initial position is
  interpreted as ECI `+z` altitude above the ground.
related_files:
  - data/aero/synthetic-elevon-1d.toml
  - crates/openbmp-cli/tests/effector_aero_e2e.rs
notes: >-
  The Schema-2 deck `synthetic-elevon-1d.toml` is the L2 data
  artefact this scenario depends on; its provenance record lives in
  `data/aero/provenance.md`. The deck's CD perturbation
  (`+0.010 · |δ_e_deg|`) is sized to make the e2e signal
  comfortably above noise — a 20° deflection adds 0.20 to CD on
  top of CD0 ≈ 0.45.
```
