# Structural Dynamics Data Provenance

## cla-event-library-v1

- File: `data/structdyn/cla-event-library-v1.toml`
- Dataset id: `openbmp.structdyn.cla_event_library.v1`
- Source class: OpenBMP local event-library fixture.
- Validation label: `validated-toy`
- Scope: closed CLA event-class vocabulary, deterministic profile multiplier
  checks for liftoff release, max-Q gust, ignition, separation, and MECO event
  templates, and local checked ignition/separation/MECO modal stiffness-scale
  schedules.
- Method: parse the TOML deck with
  `openbmp_structdyn::events::parse_cla_event_library_toml`, require known
  event classes and compatible profile kinds, then compare each stored
  multiplier sample against the runtime `EventForceProfile::multiplier_at`
  evaluator and each stored stiffness sample against the declared
  `1 + multiplier * (scale - 1)` event-stiffness convention.
- Limitation: this is not a calibrated flight-load, shell-formulation,
  GARTEUR/AIRMOD modal, or vehicle correlation dataset.
