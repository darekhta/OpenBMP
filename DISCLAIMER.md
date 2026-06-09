# OpenBMP Non-Suitability Disclaimer

OpenBMP is an academic, simulation-only research platform for rigid-body
dynamics with a primary focus on rocket-class and launch-vehicle-class
flight simulation.

OpenBMP is:

- **Not** validated for operational flight.
- **Not** suitable for hardware deployment.
- **Not** a substitute for any qualified flight-software stack.
- **Not** validated under IEC 61508, ISO 26262, DO-178C, DO-254, or any
  equivalent civilian or military safety-critical certification regime.
- **Not** subject to MIL-STD-810, MIL-STD-461, MIL-STD-464, or any
  production-quality assurance standard.

Outputs from OpenBMP — telemetry archives, controller traces, validation
results — are intended for engineering education, simulation research,
controller prototyping, autotest-driven aerospace software experiments,
and reproducible trajectory studies. They are not authoritative for any
operational decision.

Downstream consumers who integrate OpenBMP components into stacks
subject to such regimes are responsible for their own qualification
work; the OpenBMP project itself makes no compliance claims.

See `docs/safety-boundaries.md` for the project scope and validation
boundaries.
