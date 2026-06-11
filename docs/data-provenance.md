# OpenBMP Data Provenance

OpenBMP may ship public, synthetic, or textbook datasets only when their
origin, license, transformation path, and validation status are explicit.
This document defines the required provenance record for data under `data/`,
`scenarios/`, and `tests/validation/`.

The goal is not to make every public dataset safe by default. The goal is to
make source review concrete enough that unsafe, restricted, unverifiable, or
operationally framed data is rejected before it enters the repository.

## Scope

Provenance is required for:

- Atmosphere coefficients and tabulated reference atmospheres.
- Gravity coefficients and geodetic constants.
- Magnetic-field coefficients.
- Synthetic motor curves and converted public motor curves.
- Aerodynamic decks and coefficient tables.
- Offline high-fidelity reference outputs from CFD, DSMC, radiation,
  thermal-response, thermochemistry, and trajectory tools.
- Sensor-noise presets.
- Public benchmark reference trajectories.
- Hypersonic real-gas, reaction-rate, heating, ablation, and validation
  coefficient tables.
- Any generated dataset checked into the repository.

Purely analytic scenarios with no external data still include a short
provenance note stating that the case is analytic and naming the textbook or
derivation used.

### `data/` vs `scenarios/` — the layout principle

These are distinct, non-overlapping trees with provenance:

- **`data/<thing>/<name>.toml`** — *physical-constant reference data files*.
  An aero deck, a motor thrust curve, an atmosphere model, a gravity model,
  a sensor noise budget. Each is a self-contained constant table consumed
  by a model implementation in a Rust crate. **Never** put an `openbmp.scenario`
  or `openbmp.benchmark` file here. The schema markers permitted under `data/`
  are `openbmp.aero_deck`, `openbmp.thermochem_deck`,
  `openbmp.feed_turbopump_map`, `openbmp.motor`, `openbmp.imu_noise_budget`,
  and the schema-less constant TOMLs read directly by
  `crates/openbmp-{core,env}/` (atmosphere, gravity).
- **`scenarios/<category>/<name>.toml`** — *runnable scenario specifications*
  consumed by the `openbmp run` CLI runner. Each scenario carries an
  `openbmp.scenario = 2` marker and references `data/<thing>/<name>.toml`
  files by relative path with SHA-256 pin. **Never** put a constant table
  (an aero deck, a motor curve, etc.) here.
- **Cross-tool benchmark validation envelopes** (e.g., the published Niskanen
  Chapter-6 C6 experimental apogee of 151.5 m and the project-declared ±5%
  tolerance) are pinned as **Rust constants in the integration / e2e test
  file** with a citation comment. They do not get their own TOML file.
  This applies even when the scenario itself ships under `scenarios/<…>/`.
- **Small public scalar anchors or canonical reference-payload strings** may
  also be pinned as Rust constants when creating a standalone data file would
  add ceremony without improving reviewability. This exception is intentionally
  narrow: the constants must name the public source in nearby rustdoc or test
  comments, carry exact units, be covered by exact-value or SHA-256 pin tests,
  and be listed in the owning phase plan as a documented provenance-policy
  deviation. Large coefficient sets, runtime lookup tables, or imported
  third-party model code do **not** qualify for this exception; they move under
  `data/` or into a dedicated vendoring / clean-room port with full provenance
  review.

Both `data/<thing>/` and `scenarios/<category>/` require a sibling
`provenance.md` listing every shipped TOML by repository-relative path. The
workspace tripwire `data_and_scenario_tomls_have_sibling_provenance` enforces
this at `cargo test` time.

A `data/scenarios/` directory is a category error: a scenario file does not
belong under `data/`. The workspace tripwire `data_does_not_contain_scenario_tomls`
fails closed if one is reintroduced.

## Source Classes

Every dataset declares exactly one source class:

| Class | Meaning | Repository status |
|---|---|---|
| `synthetic-openbmp` | Created by the project for tests or examples | Accepted with derivation |
| `textbook-derived` | Generated from public textbook equations | Accepted with citation |
| `public-standard` | Public standard or public agency table | Accepted with license review |
| `public-academic` | Peer-reviewed or university/NASA/ESA/NACA report data | Accepted with license review |
| `public-civilian-flight` | Public civilian / academic mission data with stable source and non-operational framing | Accepted for validation references only |
| `converted-public` | Converted from an allowed public format | Accepted if original provenance survives and the source is non-operational |
| `external-generated` | Produced by an external public tool from allowed inputs | Accepted if inputs, tool version, tool configuration, and output hash are documented |
| `downstream-private` | User-owned institutional or company data loaded outside the OpenBMP repository | Not accepted into the repository; loadable only by downstream users under their own compliance review |
| `unknown` | Unclear source, unclear license, or unverifiable | Rejected |
| `operational-reporting` | News, defense-industry, or programme reporting about real systems | Rejected |
| `restricted` | ITAR, EAR, MTCR, Wassenaar, classified, proprietary, or controlled | Rejected |

## Required Record

Each dataset directory contains a `provenance.md` file. For single-file data
items, the record may live next to the file or in the nearest parent
directory when the mapping is unambiguous.

Required fields:

```yaml
dataset_id: openbmp.us_standard_1976.v1
files:
  - data/atmosphere/us_standard_1976.toml
source_class: public-standard
source_title: "U.S. Standard Atmosphere, 1976"
source_authors: "NOAA, NASA, USAF"
source_url: "https://ntrs.nasa.gov/citations/19770009539"
license_or_terms: "Public U.S. government technical report"
retrieved_utc: "2026-04-25"
source_hash_sha256: "<hash of downloaded source when practical>"
transformation:
  method: "manual transcription from published table"
  script: "tools/data/build_us_standard_1976.rs"
  script_hash_sha256: "<hash>"
verification:
  method: "regenerate table and compare against published values"
  test: "tests/validation/atmosphere/us_standard_1976.rs"
  tolerance: "documented per column"
validation_status: validated-toy
safety_review:
  reviewer: "<maintainer>"
  decision: accepted
  notes: "Civilian/public atmosphere model; no operational vehicle data."
```

The YAML block is descriptive rather than a required parser format. The
same field names are used wherever provenance is machine-checkable.

## Review Rules

A dataset is accepted only when all of these are true:

- The source class is allowed.
- The source URL or citation is stable enough for later review.
- The license or public-domain status is documented.
- The retrieved source, or a faithful local copy when legally allowed, is
  hashable.
- The transformation from source to OpenBMP format is reproducible.
- The dataset has a validation test or a documented analytic check.
- `converted-public` and `external-generated` datasets are generated only from
  accepted source classes.
- `downstream-private` data is never committed to the OpenBMP repository,
  examples, release archives, or CI fixtures. The loader may consume it in a
  downstream workspace only when the sidecar states that the downstream owner
  has performed their own license, data-rights, and qualification review.

If any item is uncertain, the dataset is rejected until the uncertainty is
resolved. "Publicly visible" is not sufficient; provenance must also be
permitted, relevant, and non-operational.

## Transformation Rules

Prefer reproducible builders over manual transcription:

- Store generator scripts under `tools/data/` when they become necessary.
- Keep generated output stable by sorting keys and formatting floats with a
  documented precision.
- Commit generated data only when the generator input, generator version, and
  output hash are recorded.
- Use one transformation step when practical. If multiple steps are required,
  list every intermediate artifact and hash.

Manual transcription is allowed for small tables, but the validation test must
make transcription mistakes visible.

## Validation Status

Data validation uses the project-wide labels:

- `experimental` - source captured, no independent check yet.
- `checked` - internally consistent and parser-tested.
- `validated-toy` - compared against analytic or simple public examples.
- `research` - compared against public academic benchmark cases.

A dataset cannot be used by a `research` model unless its own provenance is at
least `checked` and the consuming model documents the remaining uncertainty.

## Preferred Public Sources

Allowed examples include:

- U.S. Standard Atmosphere 1976, from NASA/NTRS or NOAA.
- NRLMSISE-00, from NASA CCMC or original public NRL/NASA publications.
- OpenBMP NRLMSIS 2.x compatibility profiles derived from vendorable
  baseline sources, with explicit notes that official 2.x packages are not
  redistributed.
- HWM14 public reference outputs plus redistributable source/data mirrors,
  with source hashes, bundled-data hashes, and license review.
- IERS Conventions and IERS Bulletins for Earth orientation reference data.
- NGA WGS84, EGM96, and EGM2008 public coefficient releases.
- NASA NAIF generic SPICE kernels when used as validation references.
- NASA CEA outputs for equilibrium-air validation, not as shipped runtime code.
- NACA/NASA/ESA public technical reports.
- Peer-reviewed academic papers and textbook examples.

Rejected examples include:

- News-reported performance parameters for current or recent vehicles.
- Defense-industry marketing numbers for operational systems.
- Recruiter, hiring-channel, procurement, or production-rate reporting.
- Data with unclear legal or redistribution status.
- Any operational TPS material, sensor, motor, aero, mass-property, or
  controller tuning dataset.
- Opaque coefficient datasets whose source, licensing, or applicability cannot
  be reviewed.

## Real-Data Package Credibility Format

When a downstream user assembles a vehicle on top of OpenBMP and ingests
their own datasets — aero decks from CFD or wind-tunnel runs, motor or
engine performance tables, mass / inertia builds, sensor-noise budgets,
controller gain schedules — the dataset must ship as a **real-data
package** with a credibility-metadata sidecar. The sidecar lives next to
the dataset and is read by the loader at scenario startup; missing or
mismatched fields are fail-closed errors, not warnings.

The package format is a strict superset of the `provenance.md` record
defined above: the loader still requires an accepted source class and
verification evidence, but the credibility sidecar adds the structural
fields the kernel needs to interpret the numbers.

### Required fields

```yaml
# data-package.yaml — sits next to the dataset file(s) it describes.
package_id: arv-reference.aero.deck.v3
package_kind: aero_deck
# Allowed: aero_deck, aero_buildup_geometry, engine_curve, tank,
# mass_inertia, sensor_noise, controller_gains, environment,
# continuum_cfd_aero, rarefied_dsmc_aero,
# radiation_reference, thermal_response_reference, thermochemistry_reference,
# trajectory_reference.
files:
  - data/arv-reference/aero/deck-v3.toml

# 1. Units --------------------------------------------------------------
units:
  axes:
    mach:        dimensionless
    alpha:       degree
    beta:        degree
    delta_e:     degree           # control-effector axis (see Note 1)
    delta_a:     degree
    delta_r:     degree
  outputs:
    cn:          dimensionless
    cm:          dimensionless
    cd:          dimensionless

# 2. Frames -------------------------------------------------------------
frames:
  body_axes_convention: "x_forward, y_right, z_down"   # canonical OpenBMP body
  moment_reference_point_body_m: [0.0, 0.0, 0.0]       # see reference geometry
  force_axes:    body
  moment_axes:   body
  sign_convention:
    positive_alpha: "nose-up pitch"
    positive_beta:  "nose-right yaw"

# 3. Reference geometry ------------------------------------------------
reference:
  area_m2:     1.227
  length_m:    1.250
  span_m:      0.000
  origin_body_m: [0.0, 0.0, 0.0]                       # vehicle body origin
  cg_assumption_body_m: [3.5, 0.0, 0.0]                # CG used during dataset build

# 4. Interpolation policy ----------------------------------------------
interpolation:
  method: trilinear                # trilinear | quadrilinear | nearest
  axis_order: [mach, alpha, beta]  # explicit, locks reduction order
  inside_grid: ok

# 5. Extrapolation policy ----------------------------------------------
extrapolation:
  policy: clamp                    # clamp | error | linear
  documented_reason: "Out-of-grid samples saturate to grid edge; loader logs."

# 6. Validity envelope -------------------------------------------------
envelope:
  mach:    { min: 0.1,  max: 6.0 }
  reynolds: { min: 1.0e5, max: 1.0e8 }
  knudsen:  { min: 0.0,  max: 0.01 }
  alpha:   { min: -10,  max: 25 }
  beta:    { min: -8,   max:  8 }
  altitude_m: { min: 0, max: 80000 }
  notes: "Dataset characterised in this envelope only; queries outside fail-closed."

# 7. Versioning and content hash ---------------------------------------
version: "3.0.0"
file_sha256:
  - file: data/arv-reference/aero/deck-v3.toml
    sha256: "<64-char hex>"
schema_version: "openbmp.data_package.v1"

# 8. Provenance (parent record) ----------------------------------------
provenance_ref: data/arv-reference/aero/provenance.md   # see § Required Record above

# 9. Uncertainty -------------------------------------------------------
uncertainty:
  cn:
    type: relative_1sigma
    value: 0.05            # ±5%, 1σ
  cm:
    type: absolute_1sigma
    value: 0.005
  notes: "1σ from CFD-vs-tunnel comparison at 12 grid points; see provenance."

# 10. Credibility (NASA-STD-7009B factor levels) -----------------------
credibility:
  verification:           level: 3
  validation:             level: 2
  input_pedigree:         level: 3
  results_uncertainty:    level: 2
  results_robustness:     level: 1
  use_history:            level: 0
  m_and_s_management:     level: 2
  people_qualifications:  level: 2
  notes: |
    Level numbering follows NASA-STD-7009B credibility-assessment scale.
    See verification.md § External V&V Reference Frames for mapping.

# 11. OpenBMP validation label (project-internal) ----------------------
validation_status: validated-toy

# 12. Optional high-fidelity reference metadata -------------------------
solver_reference:
  tool: "example-cfd"
  version: "1.2.3"
  governing_equations: "RANS Navier-Stokes"
  chemistry_model: "equilibrium-air"
  transport_model: "mixture-averaged"
  turbulence_model: "SST"
  wall_catalysis: "non-catalytic"
  grid_or_particle_convergence: "provenance.md#grid-convergence"
  boundary_conditions: "provenance.md#boundary-conditions"
```

### Field semantics

- **units** — every axis and every output has an explicit unit. Loader
  rejects packages whose unit strings do not parse against the project's
  unit registry. There are no implicit conversions; degrees are not
  silently radians.
- **frames** — the loader needs the body-axes convention, moment
  reference point, and sign convention to interpret coefficients. A
  package built around an alternate body convention (e.g.,
  `x_forward, y_left, z_up`) is rejected unless the loader is asked to
  re-express it explicitly.
- **reference geometry** — the area, length, and CG assumption define
  what `q·S·CN`, `q·S·L·Cm`, and the moment arm mean. Mismatches between
  the package's `cg_assumption_body_m` and the live vehicle CG produce a
  loader warning and a controller-side moment-arm correction (configured
  in the scenario).
- **interpolation policy** — the method and axis order are part of the
  determinism contract: trilinear with locked axis order produces the
  same number on every platform; nearest-neighbour or unspecified order
  do not. The default is `trilinear` with `axis_order` explicit.
- **extrapolation policy** — `error` is the fail-closed default. `clamp`
  and `linear` require an explicit `documented_reason` and emit a
  per-step warning channel when used.
- **validity envelope** — the loader hard-rejects scenario queries
  outside this box. The envelope is the dataset's *only* truth claim;
  outside the box the package is silent.
- **versioning and content hash** — `version` follows semantic
  versioning; `file_sha256` lists every shipped file with its hash. The
  loader recomputes hashes on load and refuses on mismatch. Telemetry
  records the package id, version, and hash so a downstream consumer can
  reproduce a run.
- **provenance_ref** — points to the project-standard `provenance.md`
  record (source class, license, retrieval date, transformation,
  verification, safety review). The credibility sidecar does not replace
  provenance; it supplements it.
- **uncertainty** — declared per output channel as either relative or
  absolute 1σ (or a richer covariance file when warranted). The
  uncertainty value is propagated into Monte-Carlo dispersion runs and
  into the credibility-factor `results_uncertainty` evidence.
- **credibility** — eight integer levels mapped to the NASA-STD-7009B
  factor scale. The loader does not validate the levels; it reproduces
  them in telemetry and the scenario report so downstream credibility
  products can cite them. See
  [verification.md § External V&V Reference Frames](verification.md#external-vv-reference-frames)
  for how OpenBMP's `experimental` / `checked` / `validated-toy` /
  `research` labels relate to the 7009B factor scale.
- **solver_reference** — required for CFD, DSMC, radiation, thermal-response,
  thermochemistry, and trajectory reference packages. It records the tool,
  version, governing equations, model assumptions, convergence evidence, and
  boundary-condition record needed to interpret the data. OpenBMP records this
  metadata; it does not certify the external solver.

> Note 1 — control-effector axes (`delta_e`, `delta_a`, `delta_r`, body
> flaps, grid fins, gimbal angles) are part of the deck only when the
> dataset was built that way. Effector influence may also live in the
> separate `ControlEffector` model layer; see
> [software-architecture.md § Control Effectors](software-architecture.md#control-effectors).

### Loader behaviour

- Missing required field: **loader rejects** the package; scenario fails
  to start with a structured diagnostic.
- Hash mismatch: **loader rejects**.
- Query outside `envelope`: **loader returns `EnvelopeError`**, kernel
  treats as a fail-closed simulation error.
- Query exactly on envelope edge: accepted; clamping starts strictly
  outside.
- Credibility sidecar parsed but `credibility` block missing: loader
  accepts with a one-line warning recorded in telemetry; the package
  cannot be promoted past `experimental` until the block is present.

### Rejected packages

A real-data package that includes any of the following is rejected on
provenance grounds, irrespective of how complete its credibility
metadata is:

- A `provenance.md` source class on the rejected list (see § Source
  Classes above): `unknown`, `operational-reporting`, `restricted`.
- Non-public or otherwise controlled fielded-vehicle parameter sets.
- Real fielded TPS material data, controller gains, sensor parameters, or
  operational engine performance tables for a specific fielded vehicle.
- Public civilian flight data used as a validation reference is allowed only
  when it is source-classed `public-civilian-flight`, limited to externally
  observable trajectory / timing / environment facts, and does not include
  restricted material, propulsion, control, or operational performance data.

The credibility sidecar is **about how to use a dataset**; it is not a
laundering channel for restricted data.

## Machine Checks

The `openbmp check-provenance` CI task (or an equivalent) performs the following:

- Finds every data file without a nearby provenance record.
- Verifies required fields are present.
- Recomputes hashes for local source copies and generated outputs.
- Flags rejected source classes.
- Checks that validation tests named in provenance records exist.
- Validates real-data package sidecars: schema version, required field
  set (units, frames, reference, interpolation, extrapolation, envelope,
  versioning, provenance reference, uncertainty, credibility), and
  refuses unknown fields.
- Recomputes `file_sha256` entries for every shipped data-package file
  and rejects mismatches.
- Emits a machine-readable report for release artifacts.

The check is blocking once a public data file is committed.

## Inline Data Tripwires

Real benchmark physical constants — WGS84 GM, J2, equatorial radius, motor
thrust curves, atmospheric tables, aero coefficients — must live in
`data/<category>/<name>.toml` files with sibling `provenance.md` and
SHA-256 pinning. Inlining real benchmark numbers as Rust string literals,
constants, or test fixtures is the historical "Niskanen-class" violation
pattern: the value reaches the codebase without provenance, becomes hard to
trace, and accumulates copy-paste callers.

This contract is enforced at `cargo test` time by the workspace tripwires
in
[`crates/openbmp-testkit/tests/inline_data_tripwire.rs`](../crates/openbmp-testkit/tests/inline_data_tripwire.rs).
The tripwires fail the build on:

1. **Multi-line Rust string TOML in `*.rs` source.** Any cooked or raw
   string literal in a `*.rs` file whose body contains an OpenBMP
	   schema header (`openbmp.scenario`, `openbmp.aero_deck`,
	   `openbmp.thermochem_deck`, `openbmp.feed_turbopump_map`,
	   `openbmp.motor`, `openbmp.imu_noise_budget`,
	   `openbmp.benchmark`)
   or a `[[metric]]` table marker is forbidden when it carries actual
   newlines or escaped `\n` separators, regardless of enclosing
   context (module-level `const`, function-local `let`, helper `fn`
   returning `String`). TOML fixtures must live in a sibling file under
   `crates/<crate>/tests/fixtures/<name>.toml` and be loaded via
   `include_str!`. Single-line strings used as `.replace(needle,
   replacement)` patterns are not flagged because they do not carry
   schema headers across line breaks.
2. **High-precision benchmark constants outside their declared
   source-of-truth.** The current tripwire table:

   | Constant | Needle | Allowed in |
   |---|---|---|
   | WGS84 GM | `3.986004418` | `data/gravity/wgs84-j2.toml`, `docs/data-provenance.md`, `crates/openbmp-physics/src/frames.rs`, `crates/openbmp-testkit/tests/inline_data_tripwire.rs` |
   | WGS84 J2 (unnormalised) | `1.082626683` | `data/gravity/wgs84-j2.toml`, `docs/scenario-format.md`, `docs/data-provenance.md`, `crates/openbmp-physics/src/gravity.rs`, `crates/openbmp-scenario/src/document.rs`, `crates/openbmp-testkit/tests/inline_data_tripwire.rs` |
   | WGS84 equatorial radius | `6378137.0` | `data/gravity/wgs84-j2.toml`, `docs/data-provenance.md`, `crates/openbmp-physics/src/frames.rs`, `crates/openbmp-testkit/tests/inline_data_tripwire.rs` |

   The Rust-side constants are intentionally written with Rust numeric
   separators where that improves readability. The tripwire normalises
   underscores before matching, so `3.986_004_418e14`,
   `1.082_626_683e-3`, and `6_378_137.0` are covered by the same
   needles as the canonical source text.

### Where test fixtures go

| File location | Provenance requirement | Example |
|---|---|---|
| `data/<category>/<name>.toml` | Sibling `provenance.md` + SHA-256 pin | `data/gravity/wgs84-j2.toml` |
| `scenarios/<category>/<name>.toml` | Sibling `provenance.md`; real validation case; references `data/` files with pinned digests | `scenarios/sounding-rocket/niskanen-2009-chapter6.toml` |
| `crates/<crate>/tests/fixtures/<name>.toml` | Synthetic test artefact; no benchmark numbers | `crates/openbmp-scenario/tests/fixtures/sounding-rocket.toml` |
| `crates/<crate>/tests/expected/<name>.toml` | Tolerance tables; clean integers, no benchmark provenance | `crates/openbmp-cli/tests/expected/constant-acceleration-drop.toml` |

A fixture in `tests/fixtures/` must be obviously synthetic — clean round
numbers, fictional names, no published-reference values. The header comment
of the file should state "Synthetic; not a benchmark." so future readers
cannot mistake it for a validation case.

The tripwires also enforce three structural rules:

3. **Test TOML lives in fixture / data directories, not `src/`.** Every
   `include_str!(… ".toml")` in `*.rs` source must lexically resolve
   to a path under `tests/fixtures/`, `tests/expected/`, `data/`, or
   `scenarios/`. The audit resolves relative paths and
   `env!("CARGO_MANIFEST_DIR")` concat forms before checking the
   directory, so path traversal through an allowed-looking substring
   does not pass. Stashing a `.toml` data file alongside Rust code in
   `crates/<crate>/src/` is a structural mistake — `src/` is for code,
   not data — and the build fails closed.
4. **Real-data TOML lives next to provenance.** Every `*.toml` under
   `data/` or `scenarios/` (or any of their subdirectories) must have
   a sibling `provenance.md` in the same directory, and that provenance
   file must list the TOML file by repository-relative path. This
   mirrors the `openbmp check-provenance` CLI command but enforces it
   under `cargo test`, so the contract holds even when the CLI is not
   run.
5. **`data/` and `scenarios/` carry the right schema markers.** A TOML
   under `data/` may not contain an `openbmp.scenario` or
   `openbmp.benchmark` schema marker (those belong in `scenarios/`),
	   and a TOML under `scenarios/` may not contain an `openbmp.aero_deck`,
	   `openbmp.thermochem_deck`, `openbmp.feed_turbopump_map`,
	   `openbmp.motor`, or
	   `openbmp.imu_noise_budget` schema marker (those belong in `data/`).
	   This enforces the layout principle described in
   § Scope; misplaced schemas fail closed at `cargo test` time.

### Adding a new tripwire entry

When the project ships a new physical constant from an external
authoritative reference (e.g., a new gravity model, a new propellant
constant), the path is:

1. Land the value in a `data/<category>/<name>.toml` file with a sibling
   `provenance.md` entry citing the source.
2. If the value is also needed at compile time, add only the minimal
   `pub const NAME: f64 = …;` surfaces needed by crate layering, each
   with a citation comment naming the same source.
3. Add a `Tripwire` entry to
   `crates/openbmp-testkit/tests/inline_data_tripwire.rs` listing the
   source-of-truth files in `allow_list`. The
   `tripwire_finds_known_examples_in_allow_listed_files` test enforces
   that every non-self allow-list entry still contains the covered
   value after underscore normalisation.

The allow-list itself is reviewable in PR; reviewers can verify the new
constant has provenance before extending the list.
