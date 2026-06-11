# Modelica Reference-FMUs Provenance

OpenBMP does not vendor the Reference-FMUs archive. The FMI Reference-FMU smoke
downloads the pinned public release asset at test time and rejects any SHA-256
mismatch.

- Source repository: https://github.com/modelica/Reference-FMUs
- Release: v0.0.39
- Asset: `Reference-FMUs-0.0.39.zip`
- URL: https://github.com/modelica/Reference-FMUs/releases/download/v0.0.39/Reference-FMUs-0.0.39.zip
- SHA-256: `6863d55e5818e1ca4e4614c4d4ba4047a921b4495f6336e7002874ed791f6c2a`
- License: BSD-style license distributed as `LICENSE.txt` in the release archive
- Current gated FMUs: `3.0/Dahlquist.fmu`, `3.0/VanDerPol.fmu`, `3.0/BouncingBall.fmu`, `3.0/Stair.fmu`, and `3.0/Resource.fmu`

The smoke check compares OpenBMP's typed FMI importer output to FMPy for this
small supported matrix. The BouncingBall case stops before the first event
crossing; Resource exercises materialized FMU resources and Int32 output
handling. Full clock/event scheduling remains separate work. This is not the
FMI conformance suite and does not cover every Reference-FMU model.
