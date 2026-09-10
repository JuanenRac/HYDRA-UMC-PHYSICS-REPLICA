# Changelog

All notable work on **HYDRA-UMC-PHYSICS-REPLICA** is summarized here, newest first.
This file intentionally omits calendar dates from individual entries.

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

## [0.0.5] - PHYS-01: reject malformed/non-finite URDF values instead of silently accepting them

- **PHYS-01 (P0):** `parse_xyz` used `.filter_map(|s| s.parse().ok())`, silently
  DROPPING any malformed token instead of failing the parse - an
  `origin xyz="typo 1 2 3"` was accepted as if it had genuinely been
  `"1 2 3"`. Separately, joint `<limit lower="..." upper="...">` values
  went through a bare `.parse::<f64>()`, which Rust's own `f64::from_str`
  accepts "nan"/"inf"/"infinity" through as syntactically valid - a
  `lower="NaN"` limit was accepted outright, and every IEEE-754
  comparison against it in `limits.rs`'s `validate_limits` (`value <
  lower || value > upper`) is always false, silently defeating limit
  checking for that joint no matter what position was checked. Fixed:
  every numeric token must now parse AND be finite (`parse_xyz`
  fails on the first bad token; `parse_finite_attribute` checks
  `is_finite()` explicitly), a joint limit with `lower > upper` is
  rejected outright, and a PRESENT-but-malformed `origin`/`axis`
  attribute is now a real error distinct from a genuinely ABSENT one
  (which still defaults cleanly). `validate_limits` also gained a
  defense-in-depth check: a non-finite or inverted limit is always
  reported as a violation, never silently "no violation", covering a
  `Chain` built directly (bypassing the parser) since `Joint`'s fields
  are public. 8 new regression tests reproduce the finding's own exact
  scenario (a garbage token mixed with real numbers, a NaN limit, an
  inverted limit) plus the absent-vs-corrupt distinction. `cargo fmt
  --check`/`clippy --all-targets -- -D warnings`/`test --all-targets`
  (49/49): all passing.

## [0.0.4] - Real parent/child URDF tree walk

- Found while auditing the code: `src/urdf.rs`
  used to parse `<joint>` elements in raw XML document order and treat
  that order as the serial chain - real URDF makes no such guarantee (a
  spec-valid file may declare its `<joint>` elements in any order at
  all; only each joint's own `<parent>`/`<child>` link names define the
  real tree).
- **`src/urdf.rs`** - every joint's real `<parent link="...">`/
  `<child link="...">` is now parsed (missing either is a real
  `UrdfError::MissingAttribute`, matching the URDF spec's own
  requirement) and walked from the one real root link - the link that
  is never any joint's own child - into `Chain.joints`, in genuine
  root-to-leaf order regardless of how the source file happens to list
  them. Four new real failure modes, refused rather than silently
  mishandled: `NoRootLink` (every link is someone's child - a cycle),
  `MultipleRootLinks` (two or more disconnected robots in one file),
  `Branching` (a real link with more than one child joint - `Chain` is
  still a flat `Vec<Joint>` with no way to represent two children
  faithfully), and `UnreachableJoints` (a genuinely disconnected
  sub-graph, e.g. a 2-cycle, that shares no link with the walked tree).
- 8 new tests: out-of-document-order assembly, each of the four new
  error variants (including a disconnected 2-cycle specifically chosen
  because it is invisible to the root-counting check alone), a
  zero-joint single-link URDF, and missing-`<parent>`/missing-`<child>`
  attribute errors.
- `docs/CLI_REFERENCE.md`'s own fixture and every captured command
  output re-verified for real against the rebuilt release binary - the
  numeric output is unchanged (`<parent>`/`<child>` place joints in the
  tree, they don't affect the math), only the fixture XML and the
  "Not yet wired in" honesty section were updated.
- `cargo fmt --check`/`cargo clippy --all-targets -- -D warnings`/
  `cargo test --all-targets` all pass (43 tests).
- Also folds in previously-unreleased work already on `main`: `src/main.rs`/
  `src/kinematics.rs` reject empty joint names and `NaN`/infinite joint
  positions before limit or transform math, so non-finite data can no
  longer silently bypass ordered limit comparisons and produce corrupted
  poses.

---

## [0.0.3] - Real v0: joint-limit corpus + limit-aware FK regressions

- **`src/corpus.rs`** (new) - a real, reusable fixture corpus shared by `limits.rs`'s and `kinematics.rs`'s regression tests: `revolute_with_limit()`, `prismatic_with_limit()`, `continuous_unlimited()`, `fixed()`, plus `single_joint_chain()` and a realistic 2-DOF `shoulder_elbow_chain()` - a single source of truth for "in range"/"at the boundary"/"out of range" across every joint type, instead of duplicated ad hoc literals per test module. Test-only (`#[cfg(test)]`).
- **`src/kinematics.rs`** - new `forward_kinematics_checked()`: the fail-safe counterpart to `forward_kinematics()` - checks every given joint position against its declared URDF limit FIRST (via `limits::validate_limits`), and refuses to compute a world-frame transform at all if any position is out of range, returning `CheckedKinematicsError::LimitViolation`. Closes a real gap: plain `forward_kinematics()` previously computed (and a caller could trust) a pose for a joint value arbitrarily far past its declared travel - physically meaningless for a real robot. `forward_kinematics()` itself is unchanged.
- **`main.rs`** - new `fk-checked --urdf PATH --joints "..."` subcommand alongside the existing, unchanged `fk`: prints the real pose on success, or every violating joint (with its actual value and allowed range) and exits `1` if any position is out of range.
- 9 new regression tests using the new corpus, covering both joint-limit boundaries (exactly at the limit succeeds, one unit past it is refused), a wildly out-of-range prismatic input (with a companion assertion that plain `forward_kinematics` computes it anyway, documenting exactly the gap `_checked` closes), a `continuous` joint (no limit to violate), a multi-joint chain reporting only the actually-violating joint, and limit violations taking precedence over an unrelated missing-position error. 33 tests total.
- Real verification beyond the test suite: ran the compiled binary against a real 2-joint URDF fixture - `fk-checked` computed a real pose for an in-range configuration, refused (exit 1) an out-of-range one with the exact violation reported, while plain `fk` still computed the same out-of-range pose unchanged.

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
