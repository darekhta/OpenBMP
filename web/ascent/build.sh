#!/usr/bin/env bash
# Build the WebAssembly package for the Ascent demo into web/ascent/pkg.
#
# Requires wasm-pack (https://drager.github.io/wasm-pack/) and the
# wasm32-unknown-unknown target (installed automatically through
# rust-toolchain.toml).
set -euo pipefail

cd "$(dirname "$0")/../.."
wasm-pack build crates/openbmp-web --target web --release --out-dir ../../web/ascent/pkg
ls -la web/ascent/pkg/
