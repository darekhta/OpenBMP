# HAL Contract

OpenBMP's hardware-abstraction layer is the L1 contract between reusable GNC
code and a simulator or downstream board backend. It is implemented in
`crates/openbmp-hal` and is intentionally small: clocks, topic identity,
fixed-storage bus support, pull-style sensors, command-style actuators,
I-load/storage, and watchdog service.

This document is an integration contract, not a hardware qualification guide.
OpenBMP does not ship board drivers, hardware protocols, timing certification,
or a deployment-ready flight binary. A downstream adopter owns those artifacts
and their certification posture.

## Layering Rules

- HAL traits must not depend on `openbmp-sim`, `openbmp-runner`,
  `openbmp-scenario`, `openbmp-cli`, or telemetry crates.
- Simulator and board backends differ only below the HAL boundary. GNC modules
  above it communicate through typed topics and injected clocks/sensors.
- The current portable contract is single-threaded and cooperative. The bus
  API uses `&mut self`; it does not require atomics, locks, `Sync`, or async.

## Runtime Timebase

`openbmp_hal::Clock` returns monotonic mission elapsed time as `SimTime`.

Simulator backends bind this to the lockstep integrator clock. Board backends
bind it to a monotonic hardware counter normalized into mission elapsed time.
Wall-clock APIs must not leak into `openbmp-fc`; the CI lockstep-clock tripwire
guards that property.

## Bus Contract

`openbmp_hal::Topic` provides the identity required by a static bus:

- `NAME`: stable dictionary/log name.
- `VERSION`: payload schema version.
- `INDEX`: compile-time table slot.

`EncodedTopic` adds an explicit fixed-size byte codec for topics that can live
inside `StaticBus<const TOPICS, const SLOT_BYTES>`. This avoids `TypeId`, `Any`,
raw pointer casts, and heap-backed topic storage on the HAL side.

The host FC still has a dynamic bus implementation for simulation. Canonical FC
topics now also expose `Topic::INDEX`, and dictionary dumps include those
indices so a generated static topic table can be checked against host logs.

## Sensors

Sensors are pull-style:

```rust
pub trait Sensor {
    type Measurement;
    type Error;
    fn read(&mut self) -> Result<Self::Measurement, Self::Error>;
}
```

Marker traits (`Imu`, `Gnss`, `Magnetometer`, `Barometer`, `StarTracker`) group
sensor classes without adding protocol assumptions. Driver errors are not
silently ignored: FC sensor-ingest jobs publish `healthy = false` samples on
read failure, the health monitor promotes those samples to failsafe flags, and
FDIR latches the matching sensor fault bit.

## Actuators

Actuators are command-style:

```rust
pub trait Actuator {
    type Command;
    type Error;
    fn command(&mut self, command: Self::Command) -> Result<(), Self::Error>;
}
```

Marker traits (`TvcActuator`, `RcsValve`, `ThrottleCommand`) describe
vehicle command sinks used by the controller and simulator.

## I-Load And Storage

Flight I-loads are compact payloads wrapped by the HAL `OBIL` envelope:
magic bytes, envelope version, payload schema version, payload length, and
CRC-32. `decode_iload_or_fallback` validates a primary image and can select a
validated fallback image if the primary is corrupt or has an unsupported schema.

`Storage` exposes two bounded surfaces:

- `load_iload(&mut [u8])` for loading a serialized I-load image into a caller
  buffer.
- `append_record(&[u8])` for bounded flight-recorder payloads.

Flight-recorder records use the HAL `OBFR` envelope: magic bytes, record
version, record kind, monotonic sequence, monotonic mission timestamp, payload
length, and CRC-32. `FlightRecorder<const CAPACITY, const RECORD_BYTES>` is a
no-heap queue for hot-path producers. It performs bounded copies into fixed
slots and supports two explicit overflow policies: reject the newest record or
overwrite the oldest record while incrementing a dropped-record counter.
`flush_oldest` encodes one frame into a caller-owned buffer and calls
`Storage::append_record`; records remain queued if encoding or storage fails.

The downstream board still owns the physical flash/FRAM policy, wear leveling,
and measured write latency. The upstream HAL contract makes the record format,
buffering, and overflow behavior deterministic before that hardware policy is
selected.

## Verification Hooks

The implemented HAL contract is guarded by:

- `cargo check -p openbmp-core -p openbmp-state -p openbmp-models -p openbmp-mission -p openbmp-physics -p openbmp-propulsion -p openbmp-aero -p openbmp-sensors -p openbmp-hal -p openbmp-fc --no-default-features --target thumbv7em-none-eabihf --locked`
- `cargo test -p openbmp-hal iload --locked`
- `cargo test -p openbmp-hal flight_record --locked`
- `cargo test -p openbmp-hal static_bus --locked`
- `cargo test -p openbmp-fc sensor --locked`
- the `hal-portability`, `fc-dependency-graph`, `lockstep-clock`, and
  `fc-hal-gate` CI jobs.

## Current Limits

- `openbmp-fc` compiles for the embedded target with default features disabled,
  but the current path is still `alloc`-aware rather than alloc-free.
- The host FC dynamic bus has not been replaced by a generated static bus.
- PIL and HIL are scaffolded, not performed in this repository.
- Real-time WCET, interrupt latency, stack bounds, storage latency, and driver
  fault coverage require a downstream hardware bench.
