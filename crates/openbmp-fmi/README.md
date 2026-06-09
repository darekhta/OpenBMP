# openbmp-fmi

Host-only FMI adapter boundary for OpenBMP.

This crate deliberately contains the `unsafe` dynamic-library loading needed
for FMI import smoke tests. It can materialize an explicit binary entry from a
stored-entry FMU archive, load the shared library with `libloading`, call
`fmi3GetVersion`, and verify a small FMI 3 co-simulation lifecycle/step symbol
set.

It does not implement full FMI variable access, lifecycle orchestration, or
conformance testing. Portable model code should continue to depend on
`openbmp-models`, not this crate.
