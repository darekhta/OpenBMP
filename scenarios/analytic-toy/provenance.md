# Provenance — `scenarios/analytic-toy/`

Canonical OpenBMP provenance record for the analytic-toy
scenarios shipped under `scenarios/analytic-toy/`. See
[`docs/data-provenance.md`](../../docs/data-provenance.md) for the
contract this record satisfies.

## `scenarios/analytic-toy/constant-acceleration-drop.toml`

```yaml
dataset_id:       openbmp.scenario.analytic_toy.constant_acceleration_drop.v1
files:
  - scenarios/analytic-toy/constant-acceleration-drop.toml
source_class:     synthetic
source_title:     >-
  Phase-1 closed-form analytic-toy validation scenario: a 1 kg
  point mass at the ECI origin, zero initial velocity, falling
  under constant gravity magnitude g = 9.80665 m/s² along -z for
  10 s. The closed-form solution z(t) = -0.5 g t² and
  vz(t) = -g t is reproduced exactly by the kernel's RK4
  fixed-step integrator for a constant force on a constant mass.
source_authors:   OpenBMP project
source_id:        Phase-1 analytic-toy reference scenario
source_url:       (in-house)
publication_date: 2026-01-01
methodology_reference: >-
  Synthetic scenario derived from textbook constant-acceleration
  kinematics. The g value 9.80665 m/s² is the BIPM-defined
  standard gravity (CGPM 1901, codified in ISO 80000-3:2019);
  used here as the canonical drop acceleration for the analytic
  comparison, not as a localised physical constant.
methodology_urls:
  - https://www.bipm.org/en/publications/si-brochure
license_or_terms: >-
  Synthetic OpenBMP-authored scenario; no third-party
  restrictions. Released under the same dual MIT/Apache-2.0
  licensing as the rest of the workspace.
retrieved_utc:    2026-01-01
units:            metres, metres-per-second, metres-per-second-squared, seconds, kilograms
frame_profile:    toy-fixed-earth
verification_method: >-
  The Phase-1 byte-stability gate
  (`crates/openbmp-sim::tests::analytic_toy`) replays the scenario
  through the kernel and compares Parquet telemetry byte-for-byte
  against the pinned reference. Closed-form residuals are also
  asserted via `crates/openbmp-cli/tests/expected/constant-acceleration-drop.toml`.
related_validation_label: validated-toy
related_files:
  - crates/openbmp-cli/tests/expected/constant-acceleration-drop.toml
  - crates/openbmp-sim/tests/expected/constant-acceleration-drop.toml
notes: >-
  This scenario is the source-of-truth for the Phase-1 byte-stability
  contract and the `Scenario::from_toml_str_with_source_dir` parser
  unit tests in `crates/openbmp-scenario/src/scenario.rs`. Mutating
  the file (whitespace, ordering, or value changes) will invalidate
  every downstream regression. The Phase-1 closure plan
  (`docs/phase-1-plan.md`, removed in commit 085fd31) named this
  the canonical Phase-1 deliverable.
```
