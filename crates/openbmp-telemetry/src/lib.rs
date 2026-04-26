//! `openbmp-telemetry` — OpenBMP telemetry channels and exporters.
//!
//! Two-tier architecture: typed in-process channels (the software bus)
//! plus archive exporters. Parquet is the canonical archive format
//! with explicit unit/frame metadata; CSV and JSON are lossy exports
//! for human readability and golden diffability.
//!
//! **Status:** Phase 1.4 stub.
