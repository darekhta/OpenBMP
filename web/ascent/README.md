# Ascent — the OpenBMP web demo

A synthetic Falcon-9-class two-stage launch flown live by OpenBMP's
actual flight software — EKF navigation, TVC control, PEG guidance, an
FC-owned mission sequencer — compiled to WebAssembly and rendered on the
visitor's GPU. No server, no replay files: the browser steps the real
simulation.

## Layout

| Path          | What it is                                                                |
| ------------- | ------------------------------------------------------------------------- |
| `index.html`  | The demo shell (webcast HUD, engineering overlay, flight-test sandbox).    |
| `about.html`  | Methodology, validation label, scope boundaries, attributions.             |
| `js/worker.js`| Web Worker that owns the wasm session and steps it on a fixed cadence.     |
| `js/scene.js` | three.js scene: Earth, stars, procedural vehicle, plumes, cameras.         |
| `js/hud.js`   | Webcast HUD + physics-derived event ladder (liftoff, max-Q, MECO, …).      |
| `js/charts.js`| Zero-dependency strip charts for the live-estimator overlay.               |
| `js/main.js`  | Orchestration: snapshot decode, interpolation, UI, share links.            |
| `pkg/`        | wasm-bindgen output of `crates/openbmp-web` (build artifact, not tracked). |
| `vendor/`     | three.js (MIT, licence alongside).                                         |
| `assets/`     | NASA Blue Marble Earth texture (NASA imagery is not copyrighted).          |
| `test/`       | `smoke.mjs` (Node drives the wasm to orbit), `browser.mjs` (headless-Chrome end-to-end). |

## Build & run

```sh
./build.sh        # wasm-pack build of crates/openbmp-web into pkg/
./serve.sh        # static server on http://localhost:8741
```

Any static host works in production — the demo is single-threaded
WebAssembly, so no COOP/COEP headers are required.

## Tests

```sh
# Rust-side (embedded-asset parity, snapshot layout, timeline replay):
cargo test -p openbmp-web

# The wasm artifact itself, flown to orbit in Node:
node test/smoke.mjs

# Full browser end-to-end (headless Chrome over the DevTools protocol):
./serve.sh &
node test/browser.mjs
```

## Determinism & share links

The simulation is bit-deterministic. Every sandbox stimulus (engine
fault, sensor outage, wind) is logged against the kernel step clock;
SHARE RUN encodes that timeline into the URL, and loading the link
replays the identical run — pinned by
`crates/openbmp-web/tests/native.rs::stimulus_timeline_replays_identically`.

## Scope

The demo exposes simulated malfunction stimuli and read-only
observation; there is no command that steers the vehicle. See
`about.html`, the repository `DISCLAIMER.md`, and
`docs/safety-boundaries.md`.
