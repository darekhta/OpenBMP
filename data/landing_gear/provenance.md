# Landing Gear Data Provenance

`synthetic-four-leg-oleo-crush.toml` is a synthetic representative data set for
the WP-14.4 runner fixture. It is not calibrated to any flight article or drop
test. The values are rounded to keep the validation target transparent:

- four massless legs on a rigid body,
- polytropic oleo compression with quadratic orifice damping,
- an irreversible crush-core plateau,
- spherical footpads on a flat ground plane.

The file exists so scenarios can carry a SHA-256 pin for their landing-gear
parameter sidecar, matching the repository's replay/provenance pattern for
other public or synthetic data inputs.
