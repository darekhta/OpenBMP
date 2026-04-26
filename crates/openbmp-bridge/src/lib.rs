//! `openbmp-bridge` — OpenBMP optional generic socket bridge.
//!
//! The HIL pattern: an in-house `postcard`-encoded wire format
//! between the simulator and an external academic test client.
//!
//! **No real device drivers, no real bus protocols
//! (MAVLink/CAN/MIL-STD-1553/I2C/SPI/UART), no shipped integrations
//! with PX4 / ArduPilot / any flight-software stack.**
//!
//! **Status:** Phase 5 stub. The bridge is **the only crate in the
//! workspace permitted to depend on `tokio`.**
