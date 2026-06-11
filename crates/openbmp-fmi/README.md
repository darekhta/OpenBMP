# openbmp-fmi

Host-only FMI adapter boundary for OpenBMP.

This crate deliberately contains the `unsafe` dynamic-library loading needed
for FMI import smoke tests. It can materialize an explicit binary entry from a
stored-entry FMU archive, load the shared library with `libloading`, call
`fmi3GetVersion`, and verify a small FMI 3 co-simulation lifecycle/step symbol
set. It also exposes typed `Float64`/`Int32`/`UInt64` get/set wrappers, `fmi3DoStep`
early-return reporting, typed value-reference binding plans derived from
`openbmp-models::FmuArchive` metadata including FMI 3 instantiation tokens,
Clock variables, clocked scalar variables, and stored or deflated FMU ZIP
entries, typed `fmi3SetClock`/`fmi3GetClock` wrappers, an owned
`Fmi3CoSimulationInstance` wrapper for instantiate/init/terminate/free
lifecycle calls, a single-FMU master helper for one typed macro step over an
FMI component, and a multi-FMU master helper that routes typed
`Float64`/`UInt64` connections with deterministic Gauss-Seidel or Jacobi
ordering across existing FMI instances. The single-FMU master can also capture
an FMU-state checkpoint before a step and restore it on early return when the
caller selects `Fmi3RollbackPolicy::RestoreOnEarlyReturn`.
Optional `fmi3GetFMUState`/`fmi3SetFMUState`/`fmi3FreeFMUState` wrappers expose
the rollback primitive for FMUs that support state save/restore.
The `export` module can generate a restricted FMI 3 `modelDescription.xml` with
a root `instantiationToken`, direct typed `Float64`/`Int32`/`UInt64` variable elements,
and `ModelStructure` output entries, then package it with a caller-supplied
shared library payload in a stored-entry `.fmu` archive for deterministic
artifact tests. `validate_restricted_fmi3_model_description_xml` checks that
restricted metadata contract before generated XML is returned, and
`scripts/check_fmi_export_schema.py` validates the generated point-mass
metadata against both a checked-in restricted XSD contract and the pinned
official FMI 3.0 XSD set with `xmllint`; `scripts/check_fmi_export_fmpy.py`
packages the current-platform point-mass FMU, runs it through FMPy 0.3.26, and
checks the final sample against the native tolerance table; and
`scripts/check_fmi_import_point_mass.py` packages the same FMU, materializes it
through OpenBMP's own importer, instantiates it, steps it through the typed
master twice from fresh materialized binaries, byte-compares the CSV outputs,
and checks the same native tolerance table; `scripts/check_fmi_import_fmpy.py`
then cross-checks the OpenBMP importer final sample against FMPy 0.3.26 on that
same packaged point-mass FMU. `scripts/check_fmi_reference_smoke.py`
downloads the pinned Modelica Reference-FMUs v0.0.39 release, verifies the
SHA-256 recorded in `tests/fixtures/fmi/reference-fmus/provenance.md`, runs the
FMI 3 `Dahlquist.fmu`, `VanDerPol.fmu`, pre-event `BouncingBall.fmu`,
`Stair.fmu`, and resource-backed `Resource.fmu` through the generic typed
importer example, checks OpenBMP byte stability across repeated materializations,
and compares final samples to FMPy. Its
`openbmp_point_mass_export_model` helper declares the same
time/throttle/altitude/step/velocity value-reference map used by the restricted
C ABI. `openbmp_fmi_binary_entry_for_target` and
`openbmp_fmi_binary_entry_for_current_platform` return the supported FMU archive
binary-entry paths for the crate cdylib, and
`write_openbmp_point_mass_fmu_archive_for_current_platform` writes that packaged
point-mass archive while returning the entry path the host loader should
materialize.
The crate also builds as a `cdylib` and exposes a restricted FMI 3 C-symbol
surface backed by a deterministic point-mass-like plant state for exporter smoke
tests: lifecycle, typed Float64/Int32/UInt64 access, `fmi3DoStep`, and FMU-state
snapshot/restore, plus fail-closed unsupported FMI 3 symbols so full-symbol-table
importers can load the library. The ABI test suite also compares a multi-step
exported trace against a native Rust point-mass trace using the checked-in
`tests/expected/export-point-mass-trace.toml` tolerance table.

It does not implement multi-FMU lifecycle orchestration, a multi-FMU
predictor/corrector loop beyond deterministic connection routing, full clocked
event scheduling, full Reference-FMU matrix validation beyond the current
typed no-input smoke, a full OpenBMP plant mapping, or conformance testing.
Portable model code should continue to depend on `openbmp-models`, not this
crate.
