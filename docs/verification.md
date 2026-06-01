# OpenBMP Verification

OpenBMP is autotest-driven. This document defines the verification process,
validation labels, golden-output workflow, and public-benchmark rules.

Verification in OpenBMP establishes simulator behavior for academic use. It
does not establish operational flight suitability, safety certification, or
hardware qualification.

## Validation Labels

Every model, dataset, scenario, and validation case declares one label:

| Label | Meaning | Required evidence |
|---|---|---|
| `experimental` | Implemented or drafted, not independently checked | Unit tests or parser tests only |
| `checked` | Internally consistent | Unit/property tests and documentation |
| `validated-toy` | Compared with analytic or simple public examples | Analytic-toy scenario and tolerance |
| `research` | Compared with public academic benchmark cases | Public source, provenance, tolerance table |

No OpenBMP artifact uses labels such as `flight-qualified`, `certified`,
`operational`, or `mission-ready`.

## Test Classes

| Class | Tool | Purpose | Merge gate |
|---|---|---|---|
| Unit | `cargo test` | Small deterministic behavior | Every PR |
| Property | `proptest` | Invariants and fail-closed behavior | Every PR |
| Snapshot | `insta` | Structured diagnostics and summaries | Every PR |
| Golden scenario | `openbmp diff` | Byte-stable telemetry | Every PR for small set |
| Analytic-toy | custom | Closed-form physics checks | Every PR |
| Public benchmark | custom | Public academic reference cases | Nightly, then PR for touched models |
| Fuzz | `cargo-fuzz` | Parsers and config surfaces | Nightly |
| Doc test | `rustdoc` | Public examples compile | Every PR |
| Microbenchmark | `criterion` | Performance trend detection | Nightly |
| Provenance check | `openbmp check-provenance` | Data source review | Every PR once data exists |
| Supply-chain check | `cargo deny`, `cargo audit`, `cargo machete` | Dependency policy | Every PR or dependency PR |

## Requirements Traceability

Machine-readable requirements live in [`../requirements.toml`](../requirements.toml).
Each requirement lists verification evidence, and each verification item links
back to one or more requirements. CI runs
`python3 scripts/check_requirements_traceability.py` to reject orphan
requirements, orphan verification items, missing evidence paths, and stale test
or CI anchors.

Flight I-load integrity is part of the traceable evidence set. The
`openbmp-hal` I-load envelope tests verify schema-version rejection,
CRC rejection, output-buffer bounds, and primary/fallback selection
without requiring `std` or a TOML parser on the flight side.

HAL sensor acquisition failures are also traceable FDIR evidence:
sensor-ingest jobs publish `healthy = false` samples on read failure,
the health monitor promotes those samples to failsafe flags without
waiting for a stale-timeout, and FDIR maps the flags to sensor fault
bits.

FDIR redline watchpoints are tracked as I-load evidence. Scenario v3
parses `[fc.fdir.redlines]`, the runner maps body-rate limits into
`FdirParams`, and the FDIR job latches the body-rate redline fault bit
when the active attitude estimate exceeds the declared limit.

Fork-facing conformance starts with `openbmp conform <scenario...>`.
The command validates each supplied scenario and runs it through the
normal runner path without writing telemetry files, making it a small
portable smoke suite for downstream board or HAL forks.

The heavier seeded dispersion evidence has its own CI lane:
`monte-carlo-nightly` runs the offline footprint Monte-Carlo reporting
test and the Phalcon-9-class seeded ascent dispersion checks on a weekly
schedule or manual workflow dispatch.

Static bus dictionary evidence is also traceable: canonical FC topics
declare stable `Topic::INDEX` slots, the scheduler overrun topic uses
the same index table, and the dictionary dumps those indices alongside
topic names and schema versions.

Scheduler budget enforcement is traceable FDIR evidence. The cyclic
scheduler publishes `scheduler.overrun` when a due job's declared budget
does not fit in the remaining frame envelope; the health monitor promotes
sustained overrun bursts to `FailsafeFlags.scheduler_overrun`, and FDIR
can latch the scheduler-overrun fault bit from that failsafe flag.

The HAL-side static bus contract is tested separately in
`openbmp-hal`: `StaticBus` stores fixed-size encoded topic payloads by
`Topic::INDEX`, rejects out-of-range topic slots, and avoids `TypeId`,
`Any`, and heap-backed topic storage.

## Golden Telemetry

Golden tests compare canonical scenario output against committed reference
telemetry.

Rules:

- Golden scenarios run with the default deterministic profile unless they are
  explicitly marked `state-stable`.
- Golden output includes telemetry schema version, frame metadata, scenario
  hash, data hashes, toolchain profile, and OpenBMP version.
- Golden updates require review. A changed golden file is treated as a
  behavioral change, not as generated noise.
- CSV may be used for human-readable diffs; Parquet is the canonical archive
  when implemented.

The diff tool reports:

- First divergent row.
- Channel and field.
- Expected and actual values.
- Absolute and relative error.
- Scenario hash and determinism profile.

`openbmp diff --report-json <path>` writes the same comparison result as a
machine-readable provenance artifact. The report records the compared
paths, OpenBMP version, git commit and `rustc --version` when available,
host platform, build profile, floating-point contract, first divergence,
and table-level Parquet metadata from both archives.

## Tolerance Tables

Every analytic or benchmark validation case has a tolerance-table TOML file (for example,
`crates/openbmp-sim/tests/expected/constant-acceleration-drop.toml`):

```toml
case = "constant-acceleration-drop-1000-steps"
source = "Analytic closed-form: x(t) = x0 + v0·t + 0.5·a·t² (textbook)"
validation = "validated-toy"

[[metric]]
name = "final_position_z_m"
expected = 509.6675
absolute_tolerance = 1.0e-9
relative_tolerance = 1.0e-12
```

The tolerance file is part of the validation claim. If a tolerance is widened,
the PR must explain why the previous tolerance was wrong or too narrow.

## Determinism Gate

CI runs the canonical scenario set twice on the reference platform profile and
requires byte-identical output.

The reference profile is:

- `x86_64-unknown-linux-gnu`.
- Pinned Rust toolchain from `rust-toolchain.toml`.
- Default simulation profile.
- Fixed target features documented in [software-architecture.md](software-architecture.md).

Other supported platform profiles are `state-stable, not bit-stable` unless
the project later proves byte identity there too.

## Scenario Fuzzing

Fuzz targets should cover:

- Scenario parser.
- Aero deck parser.
- Motor parser.
- Provenance manifest parser once machine-readable.
- Telemetry metadata reader.

Fuzz failures must produce diagnostics, not panics, memory unsafety, or partial
runs.

## Public Benchmark Rules

Public benchmark cases require:

- A provenance record.
- A source URL or stable citation.
- A local expected-data file when redistribution is allowed.
- A documented tolerance envelope.
- A note describing what is intentionally not validated.

Benchmarks from real operational systems are rejected unless the case is a
public civilian/academic reference and does not introduce real fielded-vehicle
parameter sets outside the safety boundary.

## Sounding-Rocket Reference Case

OpenBMP carries two sounding-rocket checks in
`crates/openbmp-vehicle/tests/sounding_rocket.rs`:

- **Niskanen 2009 Chapter 6 C6 case** (`research`): reduced point-mass
  reconstruction of the public OpenRocket technical-documentation example.
  Published inputs used directly: 56 cm rocket length, 29 mm body diameter,
  10 cm tangent-ogive nose, and Table 6.1 apogees. The C6 assertion compares
  OpenBMP apogee against the published experimental C6-3 apogee of 151.5 m
  with ±5% tolerance. The test records the unpublished assumptions explicitly:
  80 g dry vehicle mass, constant axial `CD = 0.8`, no wind, vertical launch,
  and an Estes C6 RASP curve shape scaled to Niskanen's cited 7.5 N·s C6-family
  total impulse.
- **Estes D12 integration case** (`validated-toy`): real Estes D12 RASP motor
  data from ThrustCurve.org plus the synthetic D12-class OpenBMP aero deck.
  This case is not the public benchmark; it is the L1/L2 adapter stack sanity
  run. It has tight physical sanity ranges and exact replay pins for final
  state, apogee, max velocity, and max acceleration.

The Niskanen case is intentionally reduced because the public Chapter 6 text
does not publish the component mass and CG override table used by OpenRocket
and RockSim. If those original design files become available, this test should
replace the surrogate mass/CD constants with the original values and move to
the scenario-format path.

Sources:

- Niskanen / OpenRocket technical documentation v13.05, Chapter 6:
  <https://dokk.org/library/openrocket_technical_documentation_v13.05_2013_Niskanen>
- Estes C6 RASP curve-shape source before impulse scaling:
  <https://www.thrustcurve.org/simfiles/5f4294d20002e900000004e7/download/Estes_C6.eng>,
  SHA-256 `90fa89edab96583266994ade8a8afd5b30a169f3a73d6db3a379495f24033570`.

## Continuum Aerodynamics Buildup

The launch-vehicle buildup is verified at the component level before any
scenario uses its baked deck:

1. Skin friction matches Blasius and Schlichting closed forms.
2. Base drag matches `0.12 + 0.13 M^2` for `M < 1` and `0.25 / M` for
   `M >= 1`.
3. A boattail inside the separation-angle envelope reduces zero-lift drag
   relative to the same blunt base.
4. The drag polar adds the expected second-order alpha drag.
5. Repeated deck baking over the same fixed Mach/alpha grid is bit-identical.

Scenario-level smoke tests parse `[aero.buildup]`, bake an in-memory
`AeroDeck`, and confirm no external `aero.deck` file is required.

## Hypersonic V&V and UQ Ladder

Hypersonic models require stronger evidence than ordinary toy rocket models
because chemistry, radiation, rarefaction, material response, and coupling
errors can dominate trajectory error. Hypersonic evidence is built in layers:

1. **Code verification** — manufactured solutions, exact scalar stiff ODEs,
   exact shock / expansion relations, conservation checks, and convergence-rate
   tests for smooth problems.
2. **Model verification** — comparison against textbook correlations and public
   tools such as NASA CEA for equilibrium chemistry, NRLMSIS / HWM reference
   tables for atmosphere and wind, and published Fay-Riddell / Sutton-Graves /
   Tauber-Sutton examples for heating.
3. **Code-to-code reference checks** — offline package comparisons against
   public or user-generated CFD, DSMC, radiation, thermal-response, or
   trajectory outputs. Examples include DPLR / LAURA / US3D / FUN3D-style CFD,
   NEQAIR-style radiation, SPARTA-style DSMC, FIAT / PATO / CHAR-style material
   response, and POST2 / GMAT / Orekit-style trajectory references. OpenBMP
   consumes these as data packages; it does not ship the solvers.
4. **Public flight / mission benchmarks** — Apollo-class, Stardust-class, and
   other public civilian or academic references with documented geometry,
   initial conditions, data pedigree, and tolerance limits.
5. **Uncertainty reporting** — every hypersonic validation case carries an error
   budget covering numerical tolerance, model-form uncertainty, data pedigree,
   interpolation/extrapolation, atmosphere variability, and external-reference
   uncertainty.

A hypersonic model cannot be promoted to `research` unless it has at least one
analytic or manufactured code-verification case, one public reference
comparison, a declared validity envelope, and a documented uncertainty story.
Code-to-code agreement alone is not validation; it is evidence that must be
paired with provenance and uncertainty.

## Review Checklist

Before accepting a model:

- Are units and frames documented?
- Is the validity range explicit?
- Is the validation label justified?
- Are failure modes fail-closed?
- Does at least one test exercise the model through a scenario?
- Does the model avoid wall-clock time, system RNG, network access, and
  unordered iteration?
- Is all external data covered by provenance?
- Does the model stay inside the safety boundary?

If any answer is unclear, the validation label remains `experimental`.

## External V&V Reference Frames

OpenBMP is academic and does not certify under any safety-critical regime
(see [safety-boundaries.md § Non-Compliance Statement](safety-boundaries.md)
and [standards-posture.md](standards-posture.md)).
Two civilian standards are nonetheless useful as **vocabulary and
practice-level references** for organising V&V evidence. OpenBMP borrows
their *terminology and structure*, not their compliance machinery.

### NASA-STD-7009B — Standard for Models and Simulations

NASA-STD-7009B (March 2024 revision) defines uniform practices for the
development, documentation, operation, and assessment of models and
simulations (M&S), and codifies a **credibility assessment scale** with
eight credibility factors (verification, validation, input pedigree,
results uncertainty, results robustness, use history, M&S management, and
people qualifications) used to produce M&S **credibility products** for
decision-makers.

- Source: [`standards.nasa.gov/standard/NASA/NASA-STD-7009`](https://standards.nasa.gov/standard/NASA/NASA-STD-7009)

How OpenBMP uses it:

- The four-level **validation labels** above (`experimental` → `research`)
  are mapped onto the relevant NASA-STD-7009B credibility-factor levels
  in each model's `README.md` and in tolerance-table headers, so a reader
  can see at a glance which credibility dimensions a model has earned and
  which remain `experimental`.
- The real-data package **credibility metadata** described in
  [data-provenance.md § Real-Data Package Credibility Format](data-provenance.md#real-data-package-credibility-format)
  is structured around the same eight factors so that a downstream user
  can drop OpenBMP outputs into a 7009B-style credibility product without
  re-keying.
- OpenBMP itself never makes the recommendation or decision-grade claim;
  the standard is a vocabulary contract, not a certification path.

### AIAA G-077-1998 — CFD V&V Guide

AIAA G-077-1998 ("Guide for the Verification and Validation of
Computational Fluid Dynamics Simulations") is the canonical civilian
reference for V&V terminology applied specifically to CFD (verification
vs. validation, code verification vs. solution verification, code-to-code
comparisons, grid-convergence studies, validation hierarchy from unit
problems to complete systems).

- Source: [`netforum.aiaa.org/eweb/DynamicPage.aspx?WebCode=ProdDetailAdd&ivd_prc_prd_key=BA93ABAF-C987-4FCD-9471-C61E95860FBB`](https://netforum.aiaa.org/eweb/DynamicPage.aspx?WebCode=ProdDetailAdd&ivd_prc_prd_key=BA93ABAF-C987-4FCD-9471-C61E95860FBB)

How OpenBMP uses it:

- Aero-deck and CFD-derived dataset provenance entries that include
  grid-convergence studies, code-to-code comparison, or experimental
  validation reference G-077 to keep V&V vocabulary consistent
  ("verification" = solving the equations right; "validation" =
  solving the right equations).
- OpenBMP does not ship a CFD solver. G-077 is the framing for any
  *external* CFD result a downstream user feeds into OpenBMP via the
  real-data package; OpenBMP records the V&V evidence type so users can
  trust or reject the deck on its own merits.

Both standards are **referenced, not implemented**: OpenBMP does not
distribute either document, and citing them does not make any OpenBMP
artifact safety-qualified.
