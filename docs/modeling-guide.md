# OpenBMP Modeling Guide

This guide describes the model-authoring process. It gives crate READMEs and
review checklists one stable location to link to.

## Model Contract

Every OpenBMP model must document:

- Purpose and scope.
- Inputs and outputs.
- Units and frames.
- Validity range.
- Assumptions and known limitations.
- Determinism behavior.
- Validation status.
- Data provenance for all coefficients or tables.
- Safety-boundary impact.

## Trait Implementation Rules

Models should:

- Depend only on lower layers.
- Avoid wall-clock time, system RNG, network access, and unordered iteration.
- Return explicit errors for invalid configuration.
- Fail closed for out-of-range use unless an extrapolation policy is explicit.
- Publish enough telemetry to debug model selection and validity boundaries.

Models should not:

- Hide file or network IO inside `step()` or hot-path evaluation.
- Import real fielded-vehicle parameter sets.
- Expose real hardware commands or bus protocols.
- Leave units, frames, assumptions, or validation status undocumented.

## Model README Template

Each model family should include:

```markdown
# Model Name

## Purpose

## Inputs and Outputs

## Units and Frames

## Assumptions

## Validity Range

## Determinism

## Validation

## Data Provenance

## Scope Boundary
```

## Acceptance Levels

Minimum evidence by validation label:

- `experimental`: compiles, has parser/config tests, documents assumptions.
- `checked`: unit/property tests cover invariants and invalid inputs.
- `validated-toy`: at least one analytic-toy scenario passes with tolerance.
- `research`: public benchmark case passes with provenance and tolerance table.

No model is accepted without at least `experimental` documentation.

## Continuum Aero Buildup Checklist

For `openbmp-aero::ComponentBuildup`, model authors must document the
geometry envelope, reference area/length convention, reference Reynolds
condition, and whether the result is baked as a deck or evaluated live.
Validation evidence should include skin-friction pins, the base-drag
formula, boattail/flare envelope checks, drag-polar behavior, and
deterministic deck baking. Provenance is synthetic/textbook/public
educational only; fielded-projectile drag tables are rejected.
