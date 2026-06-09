# OpenBMP Standards Posture

OpenBMP uses civil aerospace and simulation standards as vocabulary for
organizing evidence. It does not claim certification, qualification, or
operational flight suitability.

## Current Claim

OpenBMP is an academic MIL/SIL-oriented simulator and reusable plant /
scenario harness.

| Rung | Project status |
|---|---|
| MIL | N/A by design; the Rust algorithm is the model. |
| SIL | Supported for host simulation, with remaining gaps tracked as requirements. |
| PIL | Scaffolded by HAL and no-std-facing contracts; not performed upstream. |
| HIL | Scaffolded by HAL, bridge, and conformance hooks; not performed upstream. |

In this repository, **SIL** means host-compiled OpenBMP flight-controller
logic runs against a deterministic simulated plant through HAL-shaped
interfaces and the native `openbmp-sil` package/testbench API, with
experimental Python bindings for host test harnesses. A SIL evidence bundle is
review evidence for the host run only; it is not flight qualification.

OpenBMP now exposes native command/telemetry dictionary JSON and an
experimental XTCE-like export from the stable `openbmp-msgs` topic table.
It also exposes a portable `ModelPort` abstraction, an FMI co-simulation
adapter boundary, and a restricted stored-entry `.fmu` archive reader used by a
deterministic toy co-simulation equivalence test. The host-only `openbmp-fmi`
crate can materialize an explicit FMU binary entry, load it as a shared
library, call `fmi3GetVersion`, and verify a small FMI 3 co-simulation smoke
symbol set including Float64 get/set entry points. These are interchange
surfaces, not conformance claims: OpenBMP does not currently claim ASAM XIL,
FMI, XTCE, CCSDS, PUS, ECSS, DO-178C, ISO 26262, or IEC 61508 compliance.

The upstream repository can provide architecture, source code, tests,
golden comparisons, requirements traceability, HAL contracts, and
conformance smoke tests. A downstream real-vehicle fork must re-earn its
own safety classification, certification evidence, data-rights posture,
hardware qualification, real-time analysis, and tool
qualification.

## Reference Frames

OpenBMP references these standards and practices for terminology only:

- NASA-STD-7009B for model-and-simulation credibility vocabulary.
- AIAA G-077 for verification and validation terminology around
  CFD-derived or externally validated aerodynamic data.
- NASA NPR 7150.2 Class D as the closest upstream project posture:
  research, analysis, and simulation software, not safety-critical flight
  software.
- DO-178C / ISO 26262 / IEC 61508 practices as downstream fork concerns,
  not upstream claims.
- ASAM XIL as terminology for testbench abstraction, signal access,
  stimulation, capture, and replay.
- FMI as terminology for model-exchange and co-simulation boundaries.
- XTCE and CCSDS/PUS as terminology for command/telemetry dictionary and
  packet-service metadata.

Referencing a standard does not implement it. A model tagged
`research`, `validated-toy`, `checked`, or `experimental` follows the
validation labels in [verification.md](verification.md); none of those
labels imply flight readiness.

## Transfer Boundary

Publication of OpenBMP source and tests does not transfer assurance to an
integrating vehicle program. In particular:

- CI passing upstream does not qualify a flight binary.
- The HAL contract does not prove a board implementation is real-time
  safe.
- Host determinism does not prove target determinism; cross-platform
  behavior is tolerance-bounded unless a downstream fork proves more.
- The upstream permissive license does not impose operational-use
  controls on forks; forks own their own data-rights and qualification
  posture.

OpenBMP keeps this separation explicit so the repository can be useful as
an open research platform without overstating what the upstream project
has actually verified.
