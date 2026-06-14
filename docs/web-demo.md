# Ascent — the web demo architecture

`web/ascent/` is a zero-install, static-hosted demonstration in which
the actual OpenBMP simulation stack — kernel, physics, synthetic
sensors, and the in-loop flight controller — runs as WebAssembly inside
a Web Worker on the visitor's machine, rendered with three.js on their
GPU. Nothing is pre-rendered or keyframed; the page steps a live
deterministic simulation of the Phalcon-9 scenarios.

## Stack

```
scenarios/phalcon9/*.toml ──┐ (embedded verbatim at compile time,
data/{aero,sensors}/*.toml ─┤  SHA-256 pins verified in-memory)
                            ▼
                 crates/openbmp-web  ── wasm-bindgen ──► web/ascent/pkg/
                            │
              openbmp_runner::Session (steppable run)
                            │
        scenario → racks → kernel → FC bridge → telemetry
```

Layer by layer:

- **`openbmp_runner::Session`** — a prepared rigid-body run the host
  drives one kernel tick at a time. `run()` itself is implemented as
  `prepare → step_once* → finish` over the same struct, so a
  session-stepped run is byte-identical to a one-shot run *by
  construction*, and `crates/openbmp-runner/tests/session_equivalence.rs`
  pins the equality (telemetry bytes, SHA-256 gate, actuator stream).
  Between ticks the host may read truth state, separated-body lanes,
  per-engine actuation, and the controller's published estimates (the
  same read-only assembly a `SilMonitor` receives), and may inject
  deterministic malfunction stimuli.
- **Malfunction stimuli** — engine faults join the same scheduled-fault
  queue as `[propulsion.faults]` rules; sensor outages withhold
  measurements at the read→publish seam (noise streams untouched, so
  every other channel stays byte-identical); a wind override replaces
  the per-tick kernel wind sample. All are step-clock addressed: replay
  the same stimulus timeline and the run reproduces bit-for-bit.
- **`Scenario::resolved_files_with_reader`** — the existing
  pin-verified file resolution with the byte source injected, so a host
  without a filesystem resolves scenario-referenced files from an
  embedded asset table with identical digest computation and identical
  fail-closed behaviour.
- **`crates/openbmp-web`** — embeds the scenario TOMLs and their data
  files (`include_str!` against the canonical repository files — one
  source of truth), exposes `AscentSim` over wasm-bindgen: construct /
  `step_many` / flat `f64` snapshot + JSON layout / stimulus injection /
  shareable stimulus timeline. `tests/native.rs` pins the embedded
  resolution byte-for-byte against the filesystem resolution and the
  timeline replay bit-for-bit.
- **`web/ascent/`** — static site. A module Worker owns the sim and
  posts ~30 snapshots/s (transferable `Float64Array`); the main thread
  interpolates between frames and renders. The vehicle is built
  procedurally *from the scenario layout* (body geometry, engine mounts,
  tank capacities all come out of the parsed TOML), the event ladder
  (liftoff, max-Q, MECO, separation, second-stage burn, SECO, boostback)
  is detected from simulated physics, and the engineering overlay plots
  estimator-vs-truth live. Rendering is three.js (vendored, with the
  postprocessing addons) through an HDR pipeline — linear render →
  selective bloom → ACES output — over a logarithmic depth buffer
  (planet-scale shells z-fight a linear one). The Earth is a custom
  shader: NASA Blue Marble day texture, Black Marble city lights across
  the terminator, ocean sun-glint, a contrast-curved cloud layer, and
  terminator-tinted atmosphere shells. Engine plumes are two-layer
  shaders driven by the per-engine gimballed thrust vector and ambient
  pressure (Mach diamonds at sea level, wide expansion in vacuum), and
  cameras damp anchor-relative offsets so they track km/s targets
  without lag.

## Determinism and share links

The simulation seed, scenario key, and every injected stimulus (with
the kernel step it applied at) serialise into a JSON timeline. SHARE
RUN encodes the timeline into the URL; constructing `AscentSim` with a
timeline replays the commands at their recorded steps, reproducing the
run bit-for-bit on any machine. Interactive divergence is therefore
always explicit — there is no ambient nondeterminism to hide behind.

## Scope posture

The web host can *disturb and observe* the simulation; it cannot steer
it. There is no command that injects guidance, attitude, or position
references: GNC stays onboard, behind the same forward-only observation
boundary as `openbmp_runner::sil`. The stimuli surface is the
simulated fault-injection scope `docs/safety-boundaries.md` places in
bounds. The public page carries the `validated-toy` label, the
non-suitability disclaimer, and the synthetic-vehicle provenance
statement (`web/ascent/about.html`).

## Building, serving, testing

```sh
web/ascent/build.sh          # wasm-pack → web/ascent/pkg (≈1.6 MB gzipped)
web/ascent/serve.sh          # local static server
cargo test -p openbmp-web    # embedded parity, layout, replay determinism
node web/ascent/test/smoke.mjs    # the wasm artifact flown to orbit in Node
node web/ascent/test/browser.mjs  # headless-Chrome end-to-end (serve first)
```

Native measurements (Apple Silicon, release + wasm-opt): full
phalcon9-orbit-boostback run (50 000 ticks, EKF + 11 engines + aero) in
~1.4 s wall — ~700× realtime inside WebAssembly.
