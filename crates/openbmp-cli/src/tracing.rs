//! Tracing subscriber initialiser.
//!
//! `RUST_LOG` is the primary control. The `--trace` flag is shorthand:
//! one `-t` ⇒ INFO, two ⇒ DEBUG, three or more ⇒ TRACE. The flag
//! always overrides `RUST_LOG`.
//!
//! Initialisation is best-effort: a duplicate init (e.g. when tests
//! create multiple `Cli` instances in-process) is silently ignored.

use tracing_subscriber::{EnvFilter, fmt};

/// Initialise the tracing subscriber once.
pub fn init(trace_count: u8) {
    let filter = match trace_count {
        0 => EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("warn")),
        1 => EnvFilter::new("info"),
        2 => EnvFilter::new("debug"),
        _ => EnvFilter::new("trace"),
    };
    let _ = fmt()
        .with_env_filter(filter)
        .with_target(false)
        .with_writer(std::io::stderr)
        .try_init();
}
