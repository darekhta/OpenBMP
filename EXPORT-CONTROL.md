# OpenBMP Export-Control & Dual-Use Notice

This notice describes OpenBMP's **posture** toward export-control and dual-use
regimes, and the **responsibility** that rests with anyone who uses, modifies,
or redistributes the project. It exists so that awareness is explicit rather
than assumed.

> **This is not legal advice and not an export classification.** OpenBMP makes
> no representation about how this software is classified in any jurisdiction.
> Nothing here substitutes for qualified counsel. If you have an export question,
> get advice for your jurisdiction and use case.

## Project posture

OpenBMP is an academic, simulation-only research platform. It is built
deliberately to stay clear of the categories that trigger missile- and
defense-technology controls:

- **Public / textbook / synthetic data only.** No real fielded-vehicle
  aerodynamic decks, motor or engine performance data, mass properties,
  controller gains, or operational thermal-protection-material parameter sets.
  Every dataset carries provenance ([`docs/data-provenance.md`](docs/data-provenance.md)).
- **No targeting, guidance-to-target, or accuracy capability.** The platform
  answers forward trajectory questions only; it accepts no desired location /
  aimpoint and computes no miss distance / CEP. See
  [`docs/dual-use-assessment.md`](docs/dual-use-assessment.md).
- **No hardware path.** No real device drivers, board support packages, real bus
  protocols, or flight-computer firmware ship in this repository.

## Regimes to be aware of

Aerospace, rocket, and missile-related technical data and software *can* fall
under regimes including:

- **ITAR** (U.S. International Traffic in Arms Regulations) — defense articles and
  related technical data on the U.S. Munitions List.
- **EAR** (U.S. Export Administration Regulations) — dual-use items on the
  Commerce Control List.
- **MTCR** (Missile Technology Control Regime) — a multilateral regime covering
  missile / UAV systems, major subsystems, and related software and technology.
- **Wassenaar Arrangement** — multilateral export controls on dual-use goods and
  technologies.

OpenBMP is designed to avoid the technical content these regimes target — by
the exclusions listed under *Project posture* above. That design intent is not a
determination that the software is uncontrolled anywhere; it is the reason the
project believes its academic, public-data, forward-only scope is the right and
defensible one.

## Your responsibility

You are responsible for your own compliance with all applicable laws and
regulations in your jurisdiction. In particular, the project's posture above
**does not transfer** to you if you:

- add a hardware abstraction layer, real device drivers, or a concrete transport;
- import real fielded-vehicle, operational, or otherwise controlled technical data;
- remove or weaken the safety guardrails (the parse-time lint, the forward-only
  input constraints, the provenance gate); or
- combine OpenBMP with controlled data or systems, or export / re-export the
  result.

Any of these places you outside the project's posture and on your own
export-control and qualification footing. See
[`docs/dual-use-assessment.md`](docs/dual-use-assessment.md) § *What would move a
fork across the line* and [`ACCEPTABLE-USE.md`](ACCEPTABLE-USE.md).

## See also

- [`docs/safety-boundaries.md`](docs/safety-boundaries.md) — the binding accept/reject contract.
- [`docs/dual-use-assessment.md`](docs/dual-use-assessment.md) — the dual-use threat model and enforcement tiers.
- [`ACCEPTABLE-USE.md`](ACCEPTABLE-USE.md) — intended and out-of-scope use.
- [`DISCLAIMER.md`](DISCLAIMER.md) — the non-suitability disclaimer.
