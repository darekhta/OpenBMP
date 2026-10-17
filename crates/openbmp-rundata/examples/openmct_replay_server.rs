//! Host-only Open MCT replay example.
//!
//! Usage:
//!
//! ```text
//! cargo run -p openbmp-rundata --features openmct-host \
//!   --example openmct_replay_server -- \
//!   crates/openbmp-rundata/tests/fixtures/openmct-catalog.json \
//!   crates/openbmp-rundata/tests/fixtures/openmct-replay.json \
//!   127.0.0.1:8765 1
//! ```

use std::env;
use std::fs;
use std::process::ExitCode;

use openbmp_rundata::openmct::OpenMctReplayFrame;
use openbmp_rundata::openmct_host::{bind_openmct_listener, serve_openmct_listener};

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let args = env::args().collect::<Vec<_>>();
    if args.len() < 3 || args.len() > 5 {
        return Err(format!(
            "usage: {} <catalog-json> <replay-json> [bind-address] [max-connections]",
            args.first()
                .cloned()
                .unwrap_or_else(|| "openmct_replay_server".to_owned())
        ));
    }
    let catalog_json = fs::read(&args[1]).map_err(|source| format!("{}: {source}", args[1]))?;
    let replay_json = fs::read(&args[2]).map_err(|source| format!("{}: {source}", args[2]))?;
    let replay_frames = serde_json::from_slice::<Vec<OpenMctReplayFrame>>(&replay_json)
        .map_err(|source| format!("{}: {source}", args[2]))?;
    let bind_address = args.get(3).map(String::as_str).unwrap_or("127.0.0.1:8765");
    let max_connections = args
        .get(4)
        .map(|value| parse_max_connections(value))
        .transpose()?
        .unwrap_or(usize::MAX);
    let (listener, address) = bind_openmct_listener(bind_address)
        .map_err(|source| format!("{bind_address}: {source}"))?;
    println!("OpenBMP Open MCT host example listening at http://{address}/");
    println!("Catalog: http://{address}/catalog.json");
    println!("Replay WebSocket: ws://{address}/replay");
    serve_openmct_listener(listener, &catalog_json, &replay_frames, max_connections)
        .map_err(|source| source.to_string())
}

fn parse_max_connections(value: &str) -> Result<usize, String> {
    let parsed = value
        .parse::<usize>()
        .map_err(|source| format!("invalid max-connections `{value}`: {source}"))?;
    if parsed == 0 {
        return Err("max-connections must be positive".to_owned());
    }
    Ok(parsed)
}
