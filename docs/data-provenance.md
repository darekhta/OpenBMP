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

## Machine Checks

Phase 1 should add `openbmp check-provenance` or an equivalent CI task that:

- Finds every data file without a nearby provenance record.
- Verifies required fields are present.
- Recomputes hashes for local source copies and generated outputs.
- Flags rejected source classes.
- Checks that validation tests named in provenance records exist.
- Emits a machine-readable report for release artifacts.

The check is advisory during Phase 0 documentation work and blocking once the
first public data file is committed.
