# openbmp-sil-py

Python extension bindings for the native `openbmp-sil` package/testbench API.

Status: experimental host-SIL automation surface. The module exposes package
check/materialization, run, step, run-until, signal readout, bus-frame capture,
JSON stimulation for faults, parameter overrides, and command writes, evidence
writing, and evidence verdict JSON helpers. It is ASAM-XIL-inspired but does
not claim ASAM XIL conformance.
