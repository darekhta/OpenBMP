# openbmp-bridge

L7 optional generic socket bridge for the academic HIL pattern.

**Status:** Generic schema and lockstep validators. This crate defines the
in-house wire-format packets, length-prefixed `postcard` framing, and the
step/time checks that keep a simulator-side plant from advancing past an
unacknowledged sensor frame.

## Purpose

A generic socket bridge enabling academic HIL-style scenarios without
shipping real device drivers or real bus protocols.

- Wire format: in-house `postcard`-encoded `SensorPacket`,
  `ActuatorCommandPacket`, `BridgeHelloPacket`, `StepAckPacket`, and
  `BridgeMessage` per `docs/software-architecture.md` § HIL Pattern.
- Reference Rust codec and lockstep validation helpers.

## What this crate **does NOT** ship

- MAVLink, DDS, MIL-STD-1553, CAN, I2C, SPI, UART, or any real bus
  protocol.
- Real device drivers for any sensor or actuator.
- Pre-built integrations with PX4, ArduPilot, or any flight-software
  stack.

Downstream consumers wishing to integrate with a specific external
system are responsible for writing that integration **outside the
OpenBMP repository** under their own license, governance, and
qualification posture (see
[`docs/software-architecture.md`](../../docs/software-architecture.md)
§ Extensibility for Downstream Integration).

## Inputs and Outputs

Sensor packets out, command or acknowledgement packets in. A command packet
unblocks the simulator only when its `step` and `sim_time_s` exactly match the
outstanding sensor packet. `StepAckPacket` exists for empty-command,
heartbeat, and fail-closed paths.

## Units and Frames

Bridge packets carry explicit schema versions, units, and frame metadata from
the telemetry and command schemas. The bridge does not translate to external
flight-stack units or protocol frames.

## Assumptions

The bridge is an optional academic test fixture. It is not part of the
deterministic kernel and does not make real-time, hardware, or deployment
claims.

## Validity Range

Valid only for simulator-local or external academic clients using the in-house
wire format. Real protocol adapters are out of repository scope.

## Determinism

This crate is only schema, framing, and validation logic; it ships no async
runtime or socket loop. Deterministic lockstep is enforced by exact
`step`/`sim_time_s` matching, while wall-clock timing and retry behavior belong
to downstream transports.

## Validation

`experimental`. Current validation covers schema round trips, stream framing,
protocol-version checks, and fail-closed rejection of mismatched lockstep
responses.

## Data Provenance

This crate ships no data files.

## Scope Boundary

The bridge is intentionally generic — it does not negotiate with any
real protocol. Adding a real protocol implementation here would
violate the scope boundary; such code lives downstream.
