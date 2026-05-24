//! `openbmp-bridge` — abstract HIL message schema for the optional
//! academic hardware-in-the-loop pattern.
//!
//! OpenBMP is simulation-only. For lab HIL studies, a downstream
//! adopter may run the virtual flight controller (or a real one) on
//! a separate process or board and exchange messages with the
//! simulator over a socket. This crate ships **only the abstract,
//! transport-agnostic message schema and its `postcard` wire codec** —
//! the simulator emits [`SensorPacket`]s and accepts
//! [`ActuatorCommandPacket`]s.
//!
//! It deliberately ships **no real device drivers, no real bus
//! protocols** (MAVLink / CAN / MIL-STD-1553 / I²C / SPI / UART), and
//! no concrete transport. The socket loop and any hardware adapter are
//! the adopter's responsibility, under their own qualification and
//! export-control posture, in their own repository. See
//! `docs/safety-boundaries.md`.
//!
//! # Wire format
//!
//! Messages are `postcard`-encoded and length-prefixed
//! ([`frame`] / [`deframe`]) so a stream transport can recover message
//! boundaries. The schema carries a [`PROTOCOL_VERSION`] so both ends
//! can reject a mismatched peer.
//!
//! ```
//! use openbmp_bridge::{ActuatorCommandPacket, decode, encode, frame};
//!
//! let cmd = ActuatorCommandPacket {
//!     sim_time_s: 1.5,
//!     step: 150,
//!     effector_commands: vec![(0, 0.25), (1, -0.25)],
//!     engine_throttles: vec![(0, 0.8)],
//! };
//! let bytes = encode(&cmd).unwrap();
//! let _wire = frame(&bytes);
//! // ... send `_wire` over a transport the adopter owns ...
//! let round_trip: ActuatorCommandPacket = decode(&bytes).unwrap();
//! assert_eq!(cmd, round_trip);
//! ```

#![deny(missing_docs)]

pub mod codec;
pub mod error;
pub mod packet;

pub use codec::{decode, deframe, encode, frame};
pub use error::BridgeError;
pub use packet::{ActuatorCommandPacket, PROTOCOL_VERSION, SensorPacket};
