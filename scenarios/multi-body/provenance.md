# Provenance — `scenarios/multi-body/`

Canonical OpenBMP provenance record for the multi-body scenarios
shipped under `scenarios/multi-body/`. See
[`docs/data-provenance.md`](../../docs/data-provenance.md) for the
contract this record satisfies.

## `scenarios/multi-body/two-body-fairing.toml`

```yaml
dataset_id:       openbmp.scenario.multi_body.two_body_fairing.v1
files:
  - scenarios/multi-body/two-body-fairing.toml
source_class:     synthetic-openbmp
source_title:     >-
  Phase-3.3 exit-criterion two-body scenario. Demonstrates the
  declarative `[vehicle.assembly]` block with two synthetic bodies
  — an 80 g main rocket cylinder and a 5 g top-mounted fairing
  cone. Gravity-only descent from 100 m initial altitude over
  1 second; no motor, no aero, no wind. The kernel uses the flat
  `vehicle.mass_kg = 0.085` field (sum of body dry masses); the
  assembly tree is validated round-trip through the L1
  `BasicAssembly` types as a validation pass.
source_authors:   OpenBMP (Dmitri Arekhta) for the Phase-3.3 multi-body exit-criterion case
source_id:        Synthetic OpenBMP Phase-3.3 multi-body fixture
source_url:       —
publication_date: 2026-04-27
methodology_reference: >-
  `docs/scenario-format.md` § Vehicle assembly (Phase 3.3)
  documents the declarative `VehicleAssembly` tree. This scenario is
  the exit-criterion case: a multi-body assembly that loads and propagates
  through the kernel via the new resolver. Body geometry, mass,
  and CG values are synthetic round numbers chosen to exercise the
  assembly tree shape; they are not derived from a published
  reference vehicle. The fairing CG offset of 0.6 m above the main
  body's CG produces a non-trivial mass-weighted assembly CG that
  exercises the multi-body summation path in
  `BasicAssembly::mass_properties`.
methodology_urls:
  - —
license_or_terms: >-
  Synthetic OpenBMP-authored scenario; no external license.
retrieved_utc:    2026-04-27
transformation:
  method: >-
    Authored by hand for the Phase-3.3 exit criterion. No script.
  script: none
verification:
  method: >-
    `openbmp check` parses the scenario; the Phase-3.3 e2e test
    `multi_body_two_body_fairing_runs_to_completion` runs the
    scenario via `openbmp run`, asserts the kernel completes with
    `StopReason::EndTime`, and asserts the initial-row mass equals
    0.085 kg (sum of body dry masses).
  test: >-
    crates/openbmp-cli/tests/multi_body_e2e.rs
  tolerance: >-
    Mass equality is asserted to within 1e-9 kg of the declared
    sum.
validation_status: experimental
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic round-number scenario. No targeting, no real device
    drivers, no restricted or fielded-vehicle data. Demonstrates a
    new architectural surface (`[vehicle.assembly]`); does not
    encode any operational physical-vehicle parameters.
units:            metres, metres-per-second, metres-per-second-squared, seconds, kilograms
frame_profile:    toy-fixed-earth
local_origin:     >-
  None — the toy frame profile uses a fixed-Earth abstraction
  with the scenario's initial position interpreted as ECI +z
  altitude above the ground.
related_files:
  - crates/openbmp-cli/tests/multi_body_e2e.rs
notes: >-
  The scenario uses point-mass kernel kind for Phase 3.3. The
  rigid-body multi-body case (with non-trivial body inertia
  tensors) waits on the Phase-3.4+ ControlEffector / engine /
  tank tree extensions that will drive the rigid kernel from a
  real assembly composition. In 3.3 the assembly is advisory:
  validates type round-trip but does not yet replace the kernel's
  flat-field mass-model construction.
```
