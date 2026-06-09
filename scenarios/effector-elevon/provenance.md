# Provenance — `scenarios/effector-elevon/`

Canonical OpenBMP provenance record for the effector
exit-criterion scenario shipped under `scenarios/effector-elevon/`.

## `scenarios/effector-elevon/single-elevon-elevator-step.toml`

```yaml
dataset_id:       openbmp.scenario.effector_elevon.single_elevon_elevator_step.v1
files:
  - scenarios/effector-elevon/single-elevon-elevator-step.toml
source_class:     synthetic-openbmp
source_title:     >-
  Exit-criterion single-elevon scenario. Demonstrates the
  declarative `[[vehicle.assembly.effectors]]` block with one
  linear-actuator effector tracking a step elevator command.
  Gravity-only kinematics; the effector deflection is observable in
  the Parquet via the `effector.delta_e.actual` channel but does
  not perturb the dynamics. All numeric values are synthetic round
  numbers chosen to exercise the effector / rack /
  telemetry plumbing.
source_authors:   OpenBMP (Dmitri Arekhta) for the single-elevon exit-criterion case
source_id:        Synthetic OpenBMP single-elevon fixture
source_url:       —
publication_date: 2026-04-28
methodology_reference: >-
  `docs/scenario-format.md` § Control effectors
  documents the `ControlEffector` trait, `LinearActuator` reference
  impl, and runner-side `EffectorRack`. This scenario is the
  exit-criterion case: a
  scenario declaring a single elevon effector with a `delta_e`
  command schedule produces deterministic deflection telemetry. The
  effector limits (±0.349 rad ≈ ±20°, 5.236 rad/s ≈ 300°/s slew,
  0.020 s pure-delay latency, 0.05 s first-order lag) are
  representative of a small-aircraft / sounding-rocket fin-actuator
  envelope but not derived from any specific fielded vehicle.
methodology_urls:
  - —
license_or_terms: >-
  Synthetic OpenBMP-authored scenario; no external license.
retrieved_utc:    2026-04-28
transformation:
  method: >-
    Authored by hand for the effector exit criterion. No script.
  script: none
verification:
  method: >-
    `openbmp check` parses the scenario; the e2e test
    `single_elevon_scenario_runs_to_completion` runs the scenario
    via `openbmp run`, asserts the kernel completes with
    `StopReason::EndTime`, and asserts the
    `effector.delta_e.actual` channel reaches a positive
    deflection after the step command time + latency.
  test: >-
    crates/openbmp-cli/tests/effector_e2e.rs
  tolerance: >-
    Deflection within 1e-6 rad of the expected envelope at
    instrumentation times.
validation_status: experimental
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic round-number scenario with no real device drivers or fielded-vehicle data. Demonstrates
    the new `[[vehicle.assembly.effectors]]` architectural
    surface; does not encode any operational physical-vehicle
    parameters.
units:            metres, metres-per-second, metres-per-second-squared, seconds, kilograms, radians
frame_profile:    toy-fixed-earth
local_origin:     >-
  None — toy frame profile uses a fixed-Earth abstraction with the
  scenario's initial position interpreted as ECI +z altitude above
  the ground.
related_files:
  - crates/openbmp-cli/tests/effector_e2e.rs
notes: >-
  This scenario exercises the effector state machine with a
  schema-1 aero deck, so the deck does not consume
  `EffectorState.actual` via `ForceContext`. The deflection
  telemetry is therefore observable but not coupled to dynamics;
  the schema-2 coupling path is exercised by the
  effector-elevon-aero scenario.
```
