<!--
OpenBMP pull request. Fill in every section. PRs that do not answer the
Safety Review and (where applicable) the Dual-Use Gate will not be merged.
See CONTRIBUTING.md and docs/safety-boundaries.md.
-->

## Summary

<!-- What changes and why. Link the issue. -->

## Safety review

Answer all (per [`docs/safety-boundaries.md`](../docs/safety-boundaries.md) Review Questions):

- [ ] Runs without any physical hardware.
- [ ] Avoids real fielded-vehicle parameters and operational performance claims.
- [ ] Every controller output is consumed only by simulator-local models (or the generic socket bridge in test scenarios).
- [ ] Avoids targeting, terminal homing to real-world locations, and payload-delivery behaviour.
- [ ] Useful for academic simulation even with all real-world vehicle data removed.
- [ ] Assumptions, units, frames, noise models, and validation status are documented.
- [ ] Introduces **no** hard real-time guarantees, real-bus protocols, or device-driver code. *(If this is checked as introducing any, the PR is out of scope.)*

## Dual-use gate

Required if the change touches guidance, footprint / dispersion, entry, the
estimator lanes, MPC, or scenario input types
(see [`docs/dual-use-assessment.md`](../docs/dual-use-assessment.md)):

- [ ] **Forward-only.** Answers *given vehicle + trajectory, what happens?* — not *given a place to reach, what to do?*
- [ ] Adds **no** input that names or accepts a desired location, target, aimpoint, real-world waypoint, or miss-distance.
- [ ] Adds **no** accuracy / CEP metric scored against a target.
- [ ] **Enforcement tier that binds this change:** <!-- 1 Architectural / 2 Lexical / 3 Provenance — a near-the-line capability must be bound by Tier 1 --> 

## Tests & provenance

- [ ] Ships unit / property / golden / analytic-toy tests as applicable ([`docs/verification.md`](../docs/verification.md)).
- [ ] Any new data carries a provenance record ([`docs/data-provenance.md`](../docs/data-provenance.md)).
- [ ] Relevant `docs/` updated.
