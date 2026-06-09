# OpenBMP Scope Boundaries

OpenBMP is an academic, simulation-only research platform for rigid-body
dynamics, rocket-class flight simulation, and launch-vehicle studies. This
document defines the repository's project scope, validation limits, data
expectations, and hardware boundary.

These boundaries are engineering constraints for this repository. They are not
additional license terms and do not restrict downstream forks beyond the
project's normal open-source license.

## Project Definition

OpenBMP means the code, scenarios, documentation, and data files in this
repository. The repository ships a simulator, synthetic sensor models,
deterministic telemetry tooling, scenario parsing, and virtual controller
components.

The repository does not ship:

- real device drivers;
- board support packages;
- real bus protocol integrations;
- flight-computer firmware;
- certification evidence for operational flight; or
- a claim that upstream validation transfers to a downstream deployment.

Downstream adopters can build integrations against the public APIs under their
own requirements, governance, qualification, and data-rights process.

## In Scope

OpenBMP accepts contributions that are useful for simulation research and
reproducible aerospace software experiments:

- 3-DOF and 6-DOF rigid-body propagation.
- Numerical integrators with documented reproducibility behavior.
- Coordinate frames, time systems, gravity, atmosphere, wind, magnetic, aero,
  propulsion, mass-property, aerothermal, and recovery models.
- Synthetic sensor models and simulated fault-injection paths.
- Virtual flight-controller components for estimator, guidance, autopilot,
  control allocation, health, and FDIR studies.
- Scenario parsing, model registries, validation, telemetry, determinism
  gates, property tests, golden runs, and public benchmark comparisons.
- Documentation for assumptions, units, frames, validity ranges, data sources,
  and validation status.

## Data and Provenance

Data checked into this repository must be synthetic, textbook-derived, openly
published, or otherwise accompanied by clear provenance. Every imported data
file should identify its source, transformation path, license or permission
status, and content hash where practical.

The project should avoid importing opaque, proprietary, or non-public parameter
sets. If a contributor needs to compare against local user-supplied data, keep
that data outside the repository and document the comparison workflow instead
of checking the data in.

## Hardware Boundary

The controller-side traits are intentionally portable so downstream adopters
can write their own HAL or lab adapter. Upstream OpenBMP keeps the hardware
boundary abstract:

- `openbmp-hal` provides trait and storage contracts, not board support.
- `openbmp-bridge` provides an optional abstract lockstep message schema, not
  a concrete transport to physical devices.
- Simulator scenarios may exercise SIL and lab-HIL patterns, but real hardware
  adapters live outside this repository.

## Validation Labels

Every model should use one of the validation labels defined in
[`verification.md`](verification.md):

- `experimental`
- `checked`
- `validated-toy`
- `research`

The label describes the evidence available inside this repository. It is not a
certification or operational readiness claim.

## Review Questions

For non-trivial changes, reviewers should check:

1. Can the feature run without physical hardware?
2. Are data sources synthetic, public, or user-supplied with provenance?
3. Does the change avoid claiming operational flight readiness or
   certification?
4. Are assumptions, units, frames, noise models, and validation status
   documented?
5. If the change touches hardware integration, does it keep concrete adapters
   outside this repository or clearly mark them as downstream-owned?

## Naming

Names should be precise, boring, and traceable to the model or subsystem they
describe. The scenario parser enforces unit suffixes for dimensional fields
and frame suffixes for vector fields. It does not reject names based on a
policy vocabulary list.

## Non-Compliance Statement

OpenBMP's upstream artifacts are not validated for operational flight, not
suitable for hardware deployment, and not a substitute for a qualified
flight-software stack. Downstream users who integrate OpenBMP components into a
larger system are responsible for their own testing, qualification, deployment
claims, and data-rights review.
