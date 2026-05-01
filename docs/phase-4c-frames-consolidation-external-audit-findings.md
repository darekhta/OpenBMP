# Phase 4.C frames + WGS84 consolidation external audit findings

Auditor: Codex
Date: 2026-05-01
Audited commit: 71edfdf
Working tree started at: 4200067
Toolchain: rustc 1.95.0 (59807616e 2026-04-14)

## Executive summary

The consolidation itself is sound after review: WGS84 and FrameContext now
have one implementation home in `openbmp-physics::frames`, the moved math is
byte-stable against the prior commit, and no source consumer still imports the
moved surface from `openbmp-core`.

The audit did find several non-math defects around documentation, tripwire
compatibility, and test rigor. They are fixed in the current working tree. No
high-severity behavioral or determinism regression remains.

## Verified claims

| Claim | Status | Evidence |
|---|---|---|
| Six WGS84 constants live in exactly one place | Verified | `rg` for `pub const WGS84_*` finds definitions only in `crates/openbmp-physics/src/frames.rs`. |
| Moved frame types live in exactly one place | Verified | `rg` for `pub struct LocalGeodeticOrigin`, `pub enum FrameProfile`, `pub struct FrameContext`, and `pub trait FrameTransform` finds definitions only in `crates/openbmp-physics/src/frames.rs`. |
| WGS84 numeric values are bit-identical to the prior commit | Verified | `git diff 71edfdf~1 71edfdf -- crates/openbmp-core/src/frames.rs crates/openbmp-physics/src/frames.rs` shows the same constant definitions removed from core and added to physics. |
| FrameContext method bodies are unchanged | Verified | The same diff shows the ECI/ECEF velocity transport-rate and ECEF/NED rotation helpers moved without operand-order changes. |
| Tests moved together with the code | Verified with one fix | Pre-move `core::frames` had 37 `#[test]` entries; current split is 20 in core and 17 in physics. The WGS84 envelope test was strengthened to match `FrameError::InvalidGeodeticCoordinate` instead of broad `is_err()`. |
| No stale `openbmp_core::FrameContext` style import remains | Verified | `rg` for stale `openbmp_core::FrameContext`, `LocalGeodeticOrigin`, `FrameProfile`, `FrameTransform`, and `WGS84_` paths returns zero matches. |
| Consumer surface is complete | Verified | Symbol-surface `rg -l` returns only core docs/type refs, physics frames/gravity/magnetic/wind/tests, CLI runners, and one vehicle doc comment. |
| `FrameError` still supports physics frame errors | Verified after doc fix | Variants and fields are unchanged; `FrameContext::require_local_origin` still uses `self.profile.as_label()`. Stale core intra-doc links were removed. |
| `static_assertions` dependency remains live | Verified | `rg static_assertions crates/openbmp-physics` finds live wind/gust test assertions; `cargo machete` reports no unused deps. |
| Tripwire allow-list is tightened | Verified after doc fix | `cargo test -p openbmp-testkit --test inline_data_tripwire --locked` passes. Exact WGS84 benchmark needles now appear only in allow-listed active files. |
| Documentation matches current ownership | Verified after doc fix | `openbmp-physics` README, `openbmp-core` README, `docs/data-provenance.md`, and `docs/physics-consolidation-plan.md` now point WGS84/FrameContext ownership at `openbmp-physics::frames`. |
| Byte-stable scenarios are identical pre/post | Verified | Isolated worktrees for `71edfdf` and `71edfdf~1` ran the Calisto, closed-loop attitude hold, and analytic toy scenarios with explicit output paths; `diff -r` produced no output. |
| HAL portability preserved | Verified | `cargo +1.95 build -p openbmp-fc --no-default-features --locked` and `cargo +1.95 test -p openbmp-fc --no-default-features --locked` pass. |

## Findings fixed

| ID | Severity | Finding | Fix |
|---|---|---|---|
| F-1 | Medium | The new audit handover contained exact benchmark needles, causing `openbmp-testkit` inline-data tripwire failure at current HEAD. | Reworded the handover to name WGS84 GM/J2/equatorial-radius needles symbolically instead of embedding the exact strings. |
| F-2 | Medium | The audit handover byte-stable reproduction command used obsolete `--output`; current CLI expects `--output-csv`, `--output-json`, or `--output-parquet`. | Rewrote both command blocks with explicit CSV/Parquet output paths per scenario. |
| F-3 | Medium | `FrameError` docs in `openbmp-core` still linked to `crate::frames::FrameProfile` and `crate::frames::FrameContext`, which no longer exist. | Replaced stale intra-doc links with text pointing ownership at `openbmp-physics::frames`. |
| F-4 | Medium | Active docs still described old or interim WGS84 ownership in `openbmp-core` / `openbmp-physics::lib`. | Updated active README/provenance/plan docs to reflect `openbmp-physics::frames` as the owner. |
| F-5 | Low | `local_geodetic_origin_rejects_out_of_envelope` only asserted `is_err()`. | Strengthened all three assertions to match `FrameError::InvalidGeodeticCoordinate`. |
| F-6 | Low | `rustdoc -D warnings` failed on redundant intra-doc links and one private-item link in physics docs. | Removed redundant targets and converted the private cutoff link to plain text. |

## Architectural observations

The old dual-residence-with-tripwire state is gone. `openbmp-core::frames`
now has marker/value types and quaternion arithmetic only; the Earth-specific
frame profile, context, WGS84 constants, and transform implementations live in
`openbmp-physics::frames`.

`openbmp-physics::gravity` still imports shared WGS84 GM and semi-major axis
from `frames`, while J2 remains in `gravity` as a gravity-model coefficient.
That split is coherent and is now reflected in the active docs.

## Tests of suspect rigor

The only rubber-stamp pattern found in the moved suite was
`local_geodetic_origin_rejects_out_of_envelope`, which used `is_err()` for
latitude, longitude, and height rejection. The test now verifies the specific
`FrameError::InvalidGeodeticCoordinate` variant for all three cases.

## Commands run

```bash
rg -n "pub const WGS84_A_M|pub const WGS84_INV_FLATTENING|pub const WGS84_FLATTENING|pub const WGS84_ECCENTRICITY_SQUARED|pub const WGS84_MU_M3_S2|pub const WGS84_OMEGA_RAD_S|pub struct LocalGeodeticOrigin|pub enum FrameProfile|pub struct FrameContext|pub trait FrameTransform" crates --glob '*.rs'
rg -l "FrameContext|LocalGeodeticOrigin|FrameProfile|FrameTransform|WGS84_A_M|WGS84_INV_FLATTENING|WGS84_FLATTENING|WGS84_ECCENTRICITY_SQUARED|WGS84_MU_M3_S2|WGS84_OMEGA_RAD_S" crates --glob '*.rs' | sort
rg -n "openbmp_core::FrameContext|openbmp_core::LocalGeodeticOrigin|openbmp_core::FrameProfile|openbmp_core::FrameTransform|openbmp_core::WGS84_" crates --glob '*.rs'
cargo test -p openbmp-testkit --test inline_data_tripwire --locked
cargo test -p openbmp-physics frames --locked
RUSTDOCFLAGS="-D warnings" cargo doc -p openbmp-core -p openbmp-physics --no-deps --locked
cargo +1.95 fmt --all -- --check
cargo +1.95 clippy --workspace --all-targets --all-features --locked -- -D warnings
cargo +1.95 build --workspace --all-targets --all-features --locked
cargo +1.95 test --workspace --all-features --locked
cargo +1.95 build -p openbmp-fc --no-default-features --locked
cargo +1.95 test -p openbmp-fc --no-default-features --locked
cargo +1.95 build -p openbmp-physics --no-default-features --locked
cargo +1.95 test -p openbmp-physics --no-default-features --locked
cargo deny check
cargo machete
```

Byte-stable scenario comparison used detached worktrees for `71edfdf` and
`71edfdf~1`, explicit output files under `/tmp/audit-frames-*`, and `diff -r`
on the resulting directories. `diff -r` emitted no differences.

`cargo deny check` exited 0 with existing warnings for unmatched license
allowances and duplicate transitive crates; it reported `advisories ok, bans
ok, licenses ok, sources ok`.

## Sign-off

The current working tree is fit to keep after the fixes above. No byte drift,
constant drift, missed source consumer, or HAL portability regression was
found.
