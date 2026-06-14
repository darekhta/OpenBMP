#!/usr/bin/env bash
# Serve the Ascent demo locally (plain static hosting; the demo needs no
# special headers — the simulation is single-threaded WebAssembly).
set -euo pipefail

cd "$(dirname "$0")"
if [ ! -f pkg/openbmp_web_bg.wasm ]; then
  echo "pkg/ missing — run ./build.sh first" >&2
  exit 1
fi
echo "serving http://localhost:8741"
python3 -m http.server 8741
