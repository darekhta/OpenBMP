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
- Sensor-noise presets.
- Public benchmark reference trajectories.
- Hypersonic real-gas, reaction-rate, heating, ablation, and validation
  coefficient tables.
- Any generated dataset checked into the repository.

Purely analytic scenarios with no external data still include a short
provenance note stating that the case is analytic and naming the textbook or
derivation used.

## Source Classes

Every dataset declares exactly one source class:

| Class | Meaning | Repository status |
|---|---|---|
| `synthetic-openbmp` | Created by the project for tests or examples | Accepted with derivation |
| `textbook-derived` | Generated from public textbook equations | Accepted with citation |
| `public-standard` | Public standard or public agency table | Accepted with license review |
| `public-academic` | Peer-reviewed or university/NASA/ESA/NACA report data | Accepted with license review |
| `converted-public` | Converted from an allowed public format | Accepted if original provenance survives and the source is non-operational |
| `external-generated` | Produced by an external public tool from allowed inputs | Accepted if inputs, tool version, tool configuration, and output hash are documented |
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

The YAML block is descriptive, not a required parser format for Phase 0. The
same field names should be used when provenance becomes machine-checkable.

## Review Rules

A dataset is accepted only when all of these are true:

- The source class is allowed.
- The source URL or citation is stable enough for later review.
- The license or public-domain status is documented.
- The retrieved source, or a faithful local copy when legally allowed, is
  hashable.
- The transformation from source to OpenBMP format is reproducible.
- The dataset has a validation test or a documented analytic check.
- The dataset does not encode real fielded-vehicle parameters, operational
  tuning, targeting, terminal guidance, or deployment procedures.
- `converted-public` and `external-generated` datasets are generated only from
  accepted source classes; a public tool output derived from rejected inputs is
  still rejected.

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
- Data with unclear export-control status.
- Any operational TPS material, sensor, motor, aero, mass-property, or
  controller tuning dataset.

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
package_kind: aero_deck                # aero_deck | engine_curve | tank | mass_inertia | sensor_noise | controller_gains | environment
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
  do not. Phase-1 default is `trilinear` with `axis_order` explicit.
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
- Real fielded-vehicle parameter sets (operational missile decks,
  fielded engine performance tables, real TPS material data).
- Datasets whose `notes` describe targeting, terminal homing, defence
  penetration, or any other use that crosses the
  [safety-boundaries.md](safety-boundaries.md) reject list.

The credibility sidecar is **about how to use a dataset**; it is not a
laundering channel for restricted data.

## Machine Checks

Phase 1 should add `openbmp check-provenance` or an equivalent CI task that:

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

The check is advisory during Phase 0 documentation work and blocking once the
first public data file is committed.
