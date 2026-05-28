# Hypersonic Hybrid Entry Provenance

Synthetic scenario-authoring demonstrator for hybrid aerodynamic method dispatch.

Scenario file: `scenarios/hypersonic-hybrid-entry/scenario.toml`

The scenario is not a vehicle validation case. It uses a compact point-mass
entry trajectory with a US Standard Atmosphere sample, a geometry-baked
low-Mach continuum deck, tangent-cone hypersonic continuum aero, and
free-molecular aero blended by the Knudsen-number bridge.

The purpose is to keep a runnable example in `scenarios/` that avoids using
Mach-2/Mach-3 launch-vehicle decks as the sole aero source for hypersonic
entry.
