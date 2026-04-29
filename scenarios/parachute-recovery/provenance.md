# Provenance — `scenarios/parachute-recovery/`

Canonical OpenBMP provenance record for the Phase-3.9.E
exit-criterion scenario shipped under `scenarios/parachute-recovery/`.

## `scenarios/parachute-recovery/parachute-descent.toml`

```yaml
dataset_id:       openbmp.scenario.parachute_descent.v1
files:
  - scenarios/parachute-recovery/parachute-descent.toml
source_class:     synthetic-openbmp
source_title:     >-
  Phase-3.9.E exit-criterion scenario. Point-mass 5 kg vehicle
  released from altitude 1000 m with upward velocity 50 m/s,
  carrying a single `drogue_main` recovery device (drogue:
  C_D 1.0, area 0.5 m²; main: C_D 1.5, area 4.0 m²). Mission
  events fire `deploy_drogue` at apogee (≈ z = 1127 m) and
  `deploy_main` at altitude 300 m AGL on the descent leg.
  The 120 s simulation window is sized so the vehicle clears
  the main-deploy threshold and decelerates under the larger
  canopy. Synthetic drag coefficients per Knacke 1992 Chapter 5
  textbook ranges; not a fielded recovery system.
source_authors:   OpenBMP (Dmitri Arekhta) for the Phase-3.9
                  exit-criterion case
source_id:        Synthetic OpenBMP Phase-3.9 fixture
publication_date: 2026-04-29
license:          Apache-2.0 / MIT (matching repository policy)
license_url:      ../../LICENSE-APACHE
sha256:           N/A — locally generated; integrity guarded by
                  the workspace `cargo test` byte-stable Parquet
                  gate (see
                  `crates/openbmp-cli/tests/parachute_recovery_e2e.rs`)
```

### Background

`drogue_main` is the canonical two-stage academic descent model in
the Phase-3.9 [`RecoveryModel`] family. The state machine is
`Stowed → Drogue → Main`, advanced one transition per
`EventAction::DeployRecovery` firing. Drag follows the closed-form
`F_drag = -½ ρ |v|² C_D A · v̂` per Knacke 1992 Chapter 5; the
Phase-3.9 academic formulation ignores wind-relative velocity
(matches the [`AxialDragForceAdapter`] convention) and applies drag
at the body CG (no moment contribution).

The drogue and main canopy parameters are textbook ranges for
solid-fabric round canopies (Knacke 1992 Table 5-1); they are not
sourced from a fielded parachute system. The vehicle dry mass and
initial conditions are synthetic round numbers chosen so the
combined ascent + descent fits in a 120 s simulation window with
sufficient margin for the `at_altitude_descending` event to fire
above the integration step floor.

The 4× canopy-area ratio (drogue 0.5 m² → main 4.0 m²) gives a
≈ 4× terminal-velocity reduction at the main-deploy altitude
(≈ 13 m/s drogue → ≈ 3.2 m/s main at standard sea-level density),
matching the academic two-stage descent profile in Knacke Chapter 9.

### Phase-3.9 known limitations

- **Wind-relative drag.** Recovery drag in Phase 3.9 opposes ECI
  velocity (matches [`AxialDragForceAdapter`]), not air-relative
  velocity. Wind effects on parachute drag are deferred to a
  later phase.
- **Instantaneous deploy.** Phase 3.9 ships closed-form
  drag-area swap on the firing event with no canopy inflation
  transient (no inflation distance, no opening shock). Inflation
  transients are deferred to the Phase 3.X follow-on outlined in
  the architecture spec.
- **Drag at CG.** Recovery devices contribute zero moment in
  Phase 3.9; long-riser decoupling is an academic simplification.
  Riser-induced moment + canopy oscillation lands in a future
  recovery upgrade.

### References

- Knacke, T. W. *Parachute Recovery Systems Design Manual.*
  NWC TP 6575, Naval Weapons Center, China Lake (1992).
  Public-domain US Government work. Chapter 5 covers the
  drag-area model used here; Chapter 9 covers the two-stage
  descent profile.

[`RecoveryModel`]: ../../crates/openbmp-vehicle/src/recovery/mod.rs
[`AxialDragForceAdapter`]: ../../crates/openbmp-vehicle/src/adapters.rs
