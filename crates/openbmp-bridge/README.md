# openbmp-bridge

L7 optional generic socket bridge for the academic HIL pattern.

**Status:** Phase 5 — stub.

## Purpose

A generic socket bridge enabling academic HIL-style scenarios without
shipping real device drivers or real bus protocols.

- Wire format: in-house `postcard`-encoded
  `BridgeSensorPacket` / `BridgeCommandPacket` per
  `docs/software-architecture.md` § HIL Pattern.
- Reference Rust client library.

## What this crate **does NOT** ship

- MAVLink, DDS, MIL-STD-1553, CAN, I2C, SPI, UART, or any real bus
  protocol.
- Real device drivers for any sensor or actuator.
- Pre-built integrations with PX4, ArduPilot, or any flight-software
  stack.

Downstream consumers wishing to integrate with a specific external
system are responsible for writing that integration **outside the
OpenBMP repository** under their own license, governance, and
export-control posture (see
[`docs/software-architecture.md`](../../docs/software-architecture.md)
§ Extensibility for Downstream Integration).

## Inputs and Outputs

Sensor packets out, command packets in. All payloads are in-house
types defined here.

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

Bridge runs on `tokio`; this crate is **the only crate in the
workspace permitted to depend on tokio**. Determinism guarantees
inside the bridge are state-stable, not bit-stable, because of the
async runtime.

## Validation

`experimental` (stub). Future validation covers schema round trips and
fail-closed handling of malformed packets.

## Data Provenance

This crate ships no data files.

## Safety Boundary

The bridge is intentionally generic — it does not negotiate with any
real protocol. Adding a real protocol implementation here would
violate the safety boundary; such code lives downstream.
