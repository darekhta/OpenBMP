# openbmp-sil

Host software-in-the-loop package, testbench, and evidence API.

This crate is the native OpenBMP SIL control surface. It deliberately
stays **above** the simulation kernels: callers load a versioned mission
package, select a test case, drive the normal `openbmp-runner` path, and
receive a reviewable evidence bundle. The API is shaped after common
SIL/HIL bench operations (load, reset, step, run-until, fault / parameter
/ command stimulation, signal readout, bus-frame capture, evidence
export) but is **not** an ASAM XIL implementation and makes no XIL
conformance claim — it is in-process orchestration and evidence over the
deterministic runner, not a live real-time rig.

## Purpose

- `MissionPackage` — a versioned manifest (`[package]`, `[files]`,
  optional `[[test_cases]]`, `[hashes]`) bundling a scenario with its
  sidecars (plant config, flight-controller I-load, mission graph,
  scenario script, dictionary, provenance).
- `MissionPackage::check` / `materialize_sidecars` — fail-closed
  validation that declared sidecars match the scenario, hashes match file
  content, and the flight-controller I-load round-trips through the real
  HAL `OBIL` CRC/version envelope.
- `SilTestbench` — load / reset / `run` / `step` / `run_until` /
  `*_with_stimulation` / `run_observed_with_stimulation`, returning a
  `SilRunReport`.
- `SilStimulation` — pre-run scenario stimulation: `FaultInjection`
  (plant engine/effector load-time faults or scheduled propulsion engine
  rules), `AftsZoneStimulus` (in-memory `[[afts.zone]]` rules),
  `ParameterOverride` (dotted TOML path), and `CommandWrite` (scheduled
  engine/effector commands).
- `EvidenceBundle` — a reviewable JSON artifact: provenance
  (git/toolchain/host + per-file hashes), mission event / phase / region
  / command traces, synthesized bus frames, requirement verdicts, and a
  computed run verdict.

## Inputs and Outputs

A mission-package manifest path → a `SilRunReport` carrying the runner
`TelemetryTable` plus an `EvidenceBundle` that `write_evidence_bundle`
serializes to `manifest.json` and `verdict` reads back.

## Stimulation Boundary

`SilStimulation` operates **before** the run by mutating a transient copy
of the scenario document — it does not inject at the live FC/sensor
boundary mid-run. `step` runs from the scenario start to a clamped stop
time (it is not a resumable stepper), and `reset()` re-validates the
package (the native backend is stateless between runs). These are honest
limitations of the above-kernel design.

## Verdict Semantics

`EvidenceBundle.verdict` is **computed**, not assumed: it is `pass` only
when every requirement verdict passed (or was skipped), otherwise `fail`,
and `failures` lists each failed requirement. `SIL-RUN-COMPLETE` checks
that telemetry was recorded and `SIL-NOMINAL-TERMINATION` checks that the
run reached a clean terminal (a divergence, ground impact, or guard-rail
truncation fails it). The remaining requirement checks are descriptive
trace-presence records (`pass` / `skip`). A `pass` verdict therefore
means "the run completed nominally and produced the expected evidence" —
not a flight-qualification result. See `docs/standards-posture.md`.

## Determinism

The crate adds no randomness; given a fixed scenario and seed the runner
is deterministic, so the telemetry and trace-derived evidence are
reproducible. The provenance fields (timestamp, git commit, toolchain,
host triple) are intentionally environment-dependent and fail soft to
`None` / `0`.

## Scope Boundary

Stimulation stays above the kernels and is forward-only: there is no
inverse/targeting surface, and `RunUntilTarget` carries only forward stop
conditions (`time`, `event`, `phase`). The evidence bundle is a host-run
review artifact, not a signed attestation. See `docs/safety-boundaries.md`.
