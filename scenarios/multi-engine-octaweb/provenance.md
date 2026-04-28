# Provenance — `scenarios/multi-engine-octaweb/`

Canonical OpenBMP provenance record for the Phase-3.6.D
exit-criterion scenario shipped under
`scenarios/multi-engine-octaweb/`.

## `scenarios/multi-engine-octaweb/four-engine-shutdown.toml`

```yaml
dataset_id:       openbmp.scenario.multi_engine_octaweb.four_engine_shutdown.v1
files:
  - scenarios/multi-engine-octaweb/four-engine-shutdown.toml
source_class:     synthetic-openbmp
source_title:     >-
  Phase-3.6.D exit-criterion scenario. Octaweb-style 4-engine
  cluster (1 axial + 3 ring-mounted) on a 100 kg point-mass
  vehicle. Each `liquid_engine` produces 5000 N at full throttle
  with Isp = 250 s. Mission ignites all four engines at altitude
  0.001 m (≈ T+0 s with initial velocity 100 m/s ECI +z), runs
  full thrust until altitude 3000 m (≈ T+5 s), then commands
  `engine_d` shutdown to demonstrate the documented mass-flow drop
  (~25 %) and the lateral thrust asymmetry that emerges when
  `engine_b`'s gimbal-induced +x component stops being cancelled
  by `engine_d`'s -x component.
source_authors:   OpenBMP (Dmitri Arekhta) for the Phase-3.6
                  exit-criterion case
source_id:        Synthetic OpenBMP Phase-3.6 fixture
publication_date: 2026-04-28
methodology_reference: >-
  Phase-3.6 of the OpenBMP Phase-3 plan
  (`docs/phase-3-plan.md` § 3.6) introduces the `EngineModel`
  trait, `LiquidEngine` reference impl, propulsion-side
  `EngineCluster`, runner-side `EngineRack`, and the kernel-side
  `EngineSnapshotView`. This scenario is the §3.6.D
  exit-criterion case: a 4-engine cluster with one engine
  commanded to shutdown at T+5s produces the expected mass-flow
  drop and asymmetric thrust vector (Sutton & Biblarz 2017
  Chapter 8). Engine parameters are illustrative round numbers
  (5000 N max thrust, Isp = 250 s, 0.1 s ignition / shutdown
  transients) representative of a small liquid engine but not
  derived from any specific fielded vehicle.
methodology_urls:
  - —
license_or_terms: >-
  Synthetic OpenBMP-authored scenario; no external license.
retrieved_utc:    2026-04-28
transformation:
  method: >-
    Authored by hand for the Phase-3.6.D exit criterion. No script.
  script: none
verification:
  method: >-
    `openbmp check` parses the scenario; the Phase-3.6.D e2e test
    `four_engine_shutdown_scenario_runs_to_completion` runs the
    scenario via `openbmp run`, asserts the kernel completes with
    `StopReason::EndTime`, and asserts (a) summed mass-flow before
    shutdown is approximately 4× per-engine mass-flow, (b) summed
    mass-flow after shutdown drops to ~75 % of the pre-shutdown
    value, (c) the cluster's lateral thrust component along ECI x
    becomes non-zero after shutdown (asymmetric thrust signal), and
    (d) two reruns of the scenario produce byte-identical Parquet.
  test:   crates/openbmp-cli/tests/engine_cluster_e2e.rs
  tolerance: >-
    Mass-flow ratio: within 5% of the documented value at the
    asserted instrumentation times. Thrust z-axis ratio: within
    5%. Lateral thrust magnitude after shutdown: > 1 N (positive,
    proves asymmetric thrust). Same-run replay: bit-for-bit
    Parquet equality.
validation_status: experimental
safety_review:
  reviewer: dmitri.arekhta
  decision: accepted
  notes: >-
    Synthetic round-number scenario. No targeting, no real device
    drivers, no restricted or fielded-vehicle data. Demonstrates
    the new `EngineCluster` architectural surface; does not encode
    any operational physical-vehicle parameters.
units:            metres, metres-per-second, metres-per-second-squared, seconds, kilograms, radians, Newtons
frame_profile:    toy-fixed-earth
local_origin:     >-
  None — toy frame profile uses a fixed-Earth abstraction with the
  scenario's initial position interpreted as ECI +z altitude above
  the ground.
related_files:
  - crates/openbmp-cli/tests/engine_cluster_e2e.rs
notes: >-
  Phase-3.6 ships per-engine `consumed_kg` integration on the
  runner-side rack and reads it through the kernel's
  `EngineClusterMassAdapter`. Rigid-body cluster mass-properties
  (with inertia-tensor evolution) are deferred to Phase 3.7. The
  per-engine gimbal pitch / yaw values exercise the
  asymmetric-thrust signal that lets the e2e test prove the
  cluster path actually consumes per-engine state — without
  gimbals the lateral thrust would stay zero regardless of which
  engine is firing. Phase 3.2 does not ship an `at_time` trigger,
  so the shutdown event uses altitude as a deterministic proxy for
  T+5s. With full-thrust net acceleration around 190 m/s² and
  initial vertical speed 100 m/s, `altitude = v0*t + 0.5*a*t²`
  reaches roughly 3000 m at t ≈ 5 s; the 0.1 s ignition transient
  only shifts the crossing slightly.
```
