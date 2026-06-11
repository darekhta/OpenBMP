# Landing Gear Four-Leg Drop Scenario

files:
- scenarios/landing-gear-four-leg-drop/scenario.toml

Synthetic WP-14.4 fixture for runner/schema/report plumbing. The scenario pins
`data/landing_gear/synthetic-four-leg-oleo-crush.toml` by SHA-256 and uses
rounded toy parameters for a four-leg vertical drop. Inline oleo damping and
duration are tuned so the validated-toy drop reaches the runner's landing-gear
Rest classifier and closes its deterministic gear energy audit to <1%. It is
intended to validate OpenBMP landing-gear mechanics and telemetry/report
surfaces, not real lander gear.
