# Changelog

All notable work on **HYDRA-UMC-PHYSICS-REPLICA** is summarized here, newest first. Full
session-by-session detail (including dates) lives in a private,
unpublished internal log - this file is public, so it intentionally
omits calendar dates.

## Versioning scheme

`Cargo.toml`'s `version` field is bumped automatically by `bump_version.py`
(stdlib-only, no `cargo` plugin needed) before a real release build
(`cargo build --release`), invoked from `build.sh`/`build.bat`.

It follows the ecosystem-wide base-10 "odometer" rule rather than
semantic-versioning judgment calls:

- `PATCH` +1 on every build
- when `PATCH` would exceed 9, it resets to 0 and `MINOR` +1 instead (e.g. `0.0.9` -> `0.1.0`, never `0.0.10`)
- the same carry cascades into `MAJOR` if `MINOR` would exceed 9

---

## [0.0.2] - Real v0 forward kinematics and joint-limit validation
### Added
- `transform.rs` - real, dependency-free `Vec3`/`Mat4`: translation, axis-angle rotation (Rodrigues' formula), URDF `rpy` composition, matrix product, point transform.
- `urdf.rs` - a real, deliberately partial URDF reader (via the `roxmltree` crate, this project's only dependency): parses `<joint>` elements (`type`/`origin`/`axis`/`limit`) in document order as a single serial chain. Honest, documented limitation: does not walk `parent`/`child` link names, so a branching URDF tree is out of scope for v0.
- `kinematics.rs` - `forward_kinematics()`: real per-joint world-frame transforms for revolute/continuous/prismatic/fixed joints, given a joint-position map.
- `limits.rs` - `validate_limits()`: real joint-limit checking, exactly the "Kinematic Validation" feature this README already advertised before any of it existed in code. A joint with no `<limit>` is never flagged.
- `main.rs` - two new real subcommands: `fk --urdf PATH --joints "j1=0.5,..."` and `validate-limits --urdf PATH --joints "..."`. Bare invocation unchanged.
- 24 new real tests across all four new modules, covering transform math, URDF parsing (including every documented error case), kinematic chaining, and limit validation at and around the boundary.
- Real verification beyond the test suite: ran `fk` and `validate-limits` against a real 2-joint test URDF, confirming correct chained world-frame positions and a correctly detected out-of-range joint.

### Fixed
- `build.sh` called `bump_manifest_version.py` (no `--sync`) as its very first line, before also calling `bump_version.py` later - the same double-bump pattern found in other projects this session. Rewritten to bump the native version first, then sync the manifest.
- `build.sh`/`build.bat` now run `cargo test` before `cargo build --release`, and use the ecosystem's no-autoclose pattern (trap in `.sh`, `pause` in `.bat`) for the first time in this project. `run.sh`/`run.bat` now forward arguments.

## [0.0.1] - Initial scaffolding

- **`src/main.rs`** - minimal real entry point. No physics logic yet - the real-time rigid-body/contact simulation feeding HYDRA-UMC-TWIN's own renderer lands in a later pass.
- **`Cargo.toml`** - crate metadata, no runtime dependencies yet.
- **`build.sh` / `build.bat`**, **`run.sh` / `run.bat`** - `cargo build --release` and run the resulting binary.
