# OpenBMP Acceptable-Use Policy

This policy states what OpenBMP is **for** and what it is **not for**. It
expresses the maintainers' intent and the project community's norms.

> **Honest scope of this policy.** OpenBMP is dual-licensed under permissive
> terms (Apache-2.0 / MIT; see [`README.md`](README.md) § License). A permissive
> license **cannot legally restrict how you use the software**, and this policy
> does not attempt to. It signals intent, governs participation in the project's
> own spaces (issues, reviews, contributions), and tells you when you have left
> the path the project supports. It cannot bind a fork. The technical guardrails
> described in [`docs/dual-use-assessment.md`](docs/dual-use-assessment.md) are
> what actually make misuse hard; this policy is the statement of why they exist.

## Intended use

OpenBMP is built for, and supported for:

- Engineering education and graduate flight-mechanics coursework.
- Controls and GNC research — estimators, autopilots, trajectory generators,
  fault detection — against reproducible physics.
- Reproducible **forward** trajectory studies: ascent, coast, apogee, ballistic
  descent, and re-entry mechanics; range and footprint prediction; dispersion
  and range-safety analysis.
- Autotest-driven aerospace software experiments and academic benchmarking.
- Nonproliferation, range-safety, and threat-assessment analysis that
  *reconstructs or predicts* trajectories from declared parameters — forward
  modeling, not target engagement.

The unifying property: every supported use answers *given this vehicle and
trajectory, what happens?*

## Out of scope and unacceptable

The project does not support, and will not accept contributions toward:

- **Weaponization** of any kind, or any weapon-employment capability.
- **Targeting, guidance-to-target, or accuracy** development: accepting a desired
  real-world location / aimpoint, steering to it during boost, coast, or terminal phases,
  terminal homing or seeker integration, or computing / optimizing miss distance
  or CEP.
- Integrating **real fielded-vehicle or operational data**, or operational
  thermal-protection-material parameter sets.
- **Removing or weakening the guardrails** — the parse-time lint, the
  forward-only input constraints, or the provenance gate — to enable any of the
  above.
- Deployment as, or representation as, **operational flight software** or a
  qualified safety-critical stack.

These are rejected on safety grounds regardless of technical merit, per
[`docs/safety-boundaries.md`](docs/safety-boundaries.md).

## In project spaces

Contributions, issues, and reviews are governed by this policy together with
[`CODE_OF_CONDUCT.md`](CODE_OF_CONDUCT.md) and
[`docs/safety-boundaries.md`](docs/safety-boundaries.md). Maintainers will
decline out-of-scope contributions on safety grounds, and pressuring maintainers
to accept them is itself a code-of-conduct violation.

## See also

- [`docs/dual-use-assessment.md`](docs/dual-use-assessment.md) — the dual-use threat model.
- [`EXPORT-CONTROL.md`](EXPORT-CONTROL.md) — export posture and user responsibility.
- [`docs/safety-boundaries.md`](docs/safety-boundaries.md) — the binding accept/reject contract.
