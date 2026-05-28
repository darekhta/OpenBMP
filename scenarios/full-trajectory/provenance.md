# Full Trajectory Scenario Provenance

Scenario file: `scenarios/full-trajectory/boost-coast-entry-ground.toml`

Synthetic mission-graph regression for the rigid-body full trajectory path.
The fixture is not a vehicle benchmark. It uses a compact two-body assembly,
constant gravity, US Standard Atmosphere sampling, deterministic mission events,
and an initial upward velocity standing in for a powered boost impulse so the
test remains self-contained.

The scenario is intended to exercise boost, burnout, coast, stage separation,
apogee, ballistic descent, entry-interface markers, entry markers, and the
automatic ground-impact stop path in one short run.
