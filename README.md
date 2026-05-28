<h1 align="center">OpenBMP</h1>
<p align="center"><strong>Open Body Motion Platform</strong></p>
<p align="center">
A Rust-first, simulation-only research platform for rigid-body flight dynamics —<br/>
rocket-class ascent, launch vehicles, propulsive landers, and lifting re-entry.
</p>

---

> [!IMPORTANT]
> OpenBMP is an **academic simulation platform**. It is not validated for
> operational flight, not suitable for hardware deployment, not a weapon
> system, and not a substitute for any qualified flight-software stack. No
> compliance claims are made under IEC 61508, ISO 26262, DO-178C, or
> equivalent regimes. See [`docs/safety-boundaries.md`](docs/safety-boundaries.md).

## What it is

OpenBMP simulates the motion of rigid bodies through an atmosphere and a
gravity field, closes the loop with a full guidance-navigation-and-control
stack, and records every channel to a deterministic, replayable archive. It
is built for researchers and students who need a **reproducible** sandbox for
modern GNC algorithms — estimators, autopilots, trajectory generators, fault
detection — exercised against credible physics from the ground to the
hypersonic re-entry corridor.

The simulator ships only simulation. Its controller-side trait surfaces
(`Sensor`, `ControlEffector`, the mission graph, the model traits) are
designed to be **re-implementable against real hardware through a downstream
HAL** — but no HAL, device driver, or bus protocol ships in this repository.

## Highlights

- **Deterministic by construction.** Same inputs produce byte-identical
  Parquet across reruns. Locked operand order, no fused multiply-add on the
  hot path, single-threaded lockstep kernel, and a CI gate that diffs the raw
  archive bytes of canonical scenarios on every push.
- **A complete GNC stack.** Error-state EKF, MEKF, square-root UKF, and an
  interacting-multiple-model bank; PID, LQR, INDI, L1-adaptive, and
  receding-horizon MPC rate loops; minimum-snap differential-flatness
  trajectory tracking; prioritised control allocation; a voter / health /
  FDIR seam with a windowed GLRT detector.
- **Physics with a validity envelope.** US Standard Atmosphere 1976,
  layered-exponential and NRLMSISE-00 static atmospheres; J2 and EGM2008
  zonal-harmonic gravity; WMM 2025 magnetics; tabulated and hypersonic
  aerodynamic methods; stagnation heating, boundary-layer state, and a
  generic ablation toy for re-entry studies.
- **Forward propulsion analysis.** Solid-grain regression for textbook
  end-burner/BATES/tabulated grains, liquid-engine throttle and tank-budget
  coupling, and offline ideal staging ΔV / mass analysis. These models accept
  vehicle-intrinsic inputs only and emit no range or targeting objective.
- **High-order integration.** Fixed-step RK4 for byte-stable goldens, plus
  Dormand-Prince 5(4) and 8(5,3) fixed-step and adaptive variants behind an
  explicit solver profile.
- **Provenance you can audit.** Every external dataset carries a sibling
  `provenance.md` and a SHA-256 content pin enforced at scenario load; a
  build-time tripwire rejects benchmark constants smuggled into source.

## Workspace layout

OpenBMP is a layered Cargo workspace. Lower layers never depend on higher
ones; the controller and physics layers are hardware-portable and carry no
simulator dependency.

```text
crates/
  openbmp-core/         L0  math, units, frames, time, deterministic RNG
  openbmp-state/        L1  PointMassState, RigidBodyState, MassProperties
  openbmp-models/       L1  force / moment / mass / environment trait surface
  openbmp-mission/      L1  mission state machine + event triggers (HAL-portable)
  openbmp-physics/      L2  gravity, atmosphere, magnetics, wind, real-gas, re-entry
  openbmp-vehicle/      L2  rigid-body composition, mass models, force/moment sum
  openbmp-aero/         L2  aero decks + hypersonic methods + Knudsen bridging
  openbmp-aerothermal/  L2  stagnation heating, boundary layer, ablation toy
  openbmp-propulsion/   L2  motor models, thrust curves
  openbmp-sensors/      L3  synthetic sensors + fault models (no device drivers)
  openbmp-fc/           L4  virtual flight controller: estimators, autopilots, FDIR
  openbmp-telemetry/    L5  typed channels, ring buffer, CSV/JSON/Parquet export
  openbmp-scenario/     L6  TOML scenario parser, model registry, validator
  openbmp-scenario-script/ L6  simulator-only scripted physics overrides
  openbmp-testkit/      L6  proptest strategies, fixtures, tolerance tables
  openbmp-runner/       L7  scenario → kernel → telemetry orchestration
  openbmp-cli/          L7  the `openbmp` binary: run, diff, check, provenance
  openbmp-bridge/       L7  optional abstract HIL message schema (no transport)
```

See [`docs/software-architecture.md`](docs/software-architecture.md) for the
full dependency graph and the rationale behind the controller / simulator
split.

## Quick start

```bash
# Build the workspace and the `openbmp` binary.
cargo build --release

# Run a scenario; telemetry paths come from the scenario file.
./target/release/openbmp run scenarios/sounding-rocket/niskanen-2009-chapter6.toml

# Verify a scenario parses and its data pins resolve, without running.
./target/release/openbmp check scenarios/analytic-toy/constant-acceleration-drop.toml

# Prove determinism: run twice and diff the Parquet archives byte-for-byte.
./target/release/openbmp run scenarios/analytic-toy/constant-acceleration-drop.toml \
    --output-parquet /tmp/run1.parquet
./target/release/openbmp run scenarios/analytic-toy/constant-acceleration-drop.toml \
    --output-parquet /tmp/run2.parquet
./target/release/openbmp diff /tmp/run1.parquet /tmp/run2.parquet   # → identical
```

A scenario is a single TOML file describing the vehicle, environment, forces,
controller, mission graph, and telemetry outputs. The format is documented in
[`docs/scenario-format.md`](docs/scenario-format.md); worked examples live in
[`scenarios/`](scenarios/).

## Determinism & reproducibility

Reproducibility is the platform's central guarantee. The kernel is
single-threaded and advances time by canonical multiplication rather than
accumulation; integrators use a locked weighted-sum order and never call
`f64::mul_add`; tracing output is provably excluded from archive bytes. Two
classes are declared per integrator:

- **Bit-stable** — byte-identical output across reruns on one platform
  profile (fixed-step methods; gated in CI).
- **State-stable** — per-platform reproducible, but cross-platform byte
  equality is not guaranteed because adaptive step control depends on the
  platform `libm` (adaptive methods).

Every model declares its validity envelope and a validation label —
`experimental`, `checked`, `validated-toy`, or `research` — and queries
outside the envelope fail closed. See
[`docs/verification.md`](docs/verification.md).

## Safety boundaries

OpenBMP simulates generic rigid bodies and rocket-class vehicles using
synthetic, textbook, and public-benchmark data. It categorically **rejects**
targeting and terminal-guidance logic, real fielded-vehicle parameter sets,
real device drivers and bus protocols, operational thermal-protection
material data, and any weapon-employment capability — in every part of the
codebase, enforced by review and by CI tripwires. The full accept/reject
contract is in [`docs/safety-boundaries.md`](docs/safety-boundaries.md); the
dual-use threat model and enforcement tiers are in
[`docs/dual-use-assessment.md`](docs/dual-use-assessment.md), and the export
posture and acceptable use are in [`EXPORT-CONTROL.md`](EXPORT-CONTROL.md) and
[`ACCEPTABLE-USE.md`](ACCEPTABLE-USE.md).

## Documentation

| Document | Purpose |
|---|---|
| [Design Concept](docs/design-concept.md) | Purpose, principles, vehicle classes, capabilities. |
| [Software Architecture](docs/software-architecture.md) | Layered design, kernel, integrators, frames, models, controller, telemetry. |
| [Scenario Format](docs/scenario-format.md) | The TOML scenario contract: tables, parsing rules, safety-name lint. |
| [Verification](docs/verification.md) | Validation labels, golden telemetry, tolerance tables, fuzzing. |
| [Safety Boundaries](docs/safety-boundaries.md) | Accept / reject rules, review checklist, naming and provenance rules. |
| [Dual-Use Assessment](docs/dual-use-assessment.md) | Dual-use threat model: forward-not-inverse, enforcement tiers, residual surface, per-capability rationale. |
| [Frames and Time](docs/frames-time.md) | Frame profiles, ECI/ECEF/NED conventions, epoch and leap-second handling. |
| [Data Provenance](docs/data-provenance.md) | Source records, transformation rules, machine checks, inline-data tripwires. |
| [Supply Chain](docs/supply-chain.md) | Rust dependency policy, SBOM, build-provenance expectations. |
| [Modeling Guide](docs/modeling-guide.md) | Model-author contract: documentation template, validation evidence. |
| [Mission Graph Architecture](docs/mission-graph-architecture.md) | Hierarchical mission state machine, orthogonal regions, HAL contract. |
| [Mission States Vocabulary](docs/mission-states-vocabulary.md) | Canonical state names with citations; rejected operational vocabulary. |
| [Hypersonic & Re-entry](docs/hypersonic-extensions.md) | High-altitude atmosphere, real-gas, hypersonic aero, aerothermal, ablation. |
| [Real-Rocket Integration](docs/real-rocket-integration.md) | How a downstream adopter assembles a vehicle on top of OpenBMP. |
| [Roadmap](docs/roadmap.md) | Capabilities that are shipped, research-grade, or deferred. |
| [Glossary](docs/glossary.md) | Shared vocabulary for frames, time, determinism, validation, safety. |
| [Flight Profiles Architecture](docs/flight-profiles-architecture.md) | Umbrella for the multi-phase ascent → coast → entry profile design series. |
| [Staging and Separation](docs/staging-and-separation.md) | Executing multi-body stage separation: jettison, impulse, simultaneous propagation. |
| [Ascent Guidance](docs/ascent-guidance.md) | Gravity-turn / pitch-program reference-trajectory generation for powered ascent. |
| [Ballistic Coast and Apogee](docs/ballistic-coast-and-apogee.md) | Exo-atmospheric coast, apogee detection, and the range-safety landing footprint. |
| [Descent and Entry Profiles](docs/descent-and-entry-profiles.md) | Wiring Allen-Eggers / Vinh into live entry phases and recovery. |
| [Profile Vocabulary and Guardrails](docs/profile-vocabulary-and-guardrails.md) | Accepted / rejected profile vocabulary and the fail-closed non-weapon validation. |

## Toolchain

- **Rust 1.95** stable, **Edition 2024**, MSRV `1.93` (pinned in
  [`rust-toolchain.toml`](rust-toolchain.toml)).
- All external crates are pinned in `[workspace.dependencies]` in the root
  [`Cargo.toml`](Cargo.toml).

```bash
cargo build  --workspace
cargo nextest run --workspace --all-features
cargo clippy --workspace --all-targets -- -D warnings
cargo deny   check
```

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md) for the workflow, the safety-boundary
review checklist, and the documentation discipline required before any code
or data change is accepted. Conduct expectations are in
[`CODE_OF_CONDUCT.md`](CODE_OF_CONDUCT.md); vulnerability disclosure is in
[`SECURITY.md`](SECURITY.md).

## License

Dual licensed under either of [Apache-2.0](LICENSE-APACHE) or
[MIT](LICENSE-MIT) at your option. Unless you state otherwise, any
contribution you intentionally submit for inclusion shall be dual licensed as
above, without additional terms.
