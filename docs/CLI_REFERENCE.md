# HYDRA-UMC-PHYSICS-REPLICA — CLI Reference

`hydra-umc-physics-replica` is a single Rust binary (`src/main.rs`) that
runs real forward-kinematics and joint-limit validation over a (documented,
deliberately partial) URDF subset — see `src/urdf.rs`'s own module docs for
what's parsed today (every joint's real `<parent>`/`<child>` link names are
walked from the document's own real root, every joint after the joint that
carries its parent link, regardless of `<joint>` element order in the file;
a branching tree — one link with more than one child joint, like the two
fingers of a gripper — is read whole, each branch placed from the joint
that carries it). Every example below was captured from a
real, built release binary, run against a hand-written but structurally
real URDF fixture (this repo ships no sample `.urdf` file of its own) — the
output shown is real, not illustrative.

## Usage

```
$ ./run.sh fk --urdf arm.urdf --joints j1=0.5
```

`run.sh` execs `target/release/hydra-umc-physics-replica` and forwards all
arguments unchanged. The examples below invoke the release binary directly,
which is equivalent.

Bare invocation (no arguments) prints identity/version/role and exits `0`:

```
$ hydra-umc-physics-replica
HYDRA-UMC-PHYSICS-REPLICA v0.0.7
URDF forward kinematics and joint-limit validation for kinematic trees (no dynamics, no physics engine).
```

## Reader rules

* Only `<joint>` elements that are direct children of `<robot>` are read: the `<joint>` elements inside a
  `<transmission>` (ros_control) have no type and are not part of the tree.
* Joint names are unique, a link has at most one parent and, when the file declares `<link>` elements, every
  link a joint names is declared.
* A revolute, continuous or prismatic joint needs an `<axis>` of non-zero length.
* Position limits belong to revolute and prismatic joints. A continuous joint's `<limit>` gives effort and
  velocity only (its `lower`/`upper` are ignored, they used to lock it at 0), and a fixed joint has none.
* A revolute or prismatic joint with no `<limit>` cannot be checked: `fk-checked` and `validate-limits` exit `2`
  naming it.
* `<mimic joint multiplier offset>`: the joint follows another, `multiplier * master + offset` (a follower of a follower works; a
  follower that names no joint, itself, a fixed joint, or follows in a circle is refused; a position given for a follower is refused
  because it is derived, never chosen). The limits of a follower are checked with the position its master gives it.
* `<limit effort velocity>` of every joint type and `<safety_controller soft_lower_limit soft_upper_limit k_position k_velocity>`
  (the soft range must be ordered and inside the hard limit).
* Not supported: floating and planar joints.
* Units are those of URDF: metres and radians.
* A name in `--joints` that the URDF does not have, or one given twice, is an error (exit `2`).

## Output

`fk` and `fk-checked` print `name: x=.. y=.. z=..` per joint (metres), or with `--json` an array of
`{"joint", "position": {x, y, z}, "quaternion": {w, x, y, z}}` with nine decimals; the rotation is a unit
quaternion with `w >= 0`.

## Fixture used below

None of these examples needs real hardware, but they do need a real URDF
file. This repo doesn't ship a sample one, so the fixture below was
hand-written to match the exact shape `src/urdf.rs` parses (a
`<joint name type>` with real `<parent>`/`<child>` link names plus
`<origin>`/`<axis>`/`<limit>` children) — a 2-joint shoulder/elbow serial
chain, the same joint math `src/corpus.rs`'s own `shoulder_elbow_chain()`
test helper builds programmatically (that helper builds a `Chain` directly
in Rust, bypassing `parse_urdf`/XML entirely, so it carries no
`<parent>`/`<child>` of its own):

```xml
<!-- arm.urdf -->
<robot name="shoulder_elbow_arm">
  <joint name="shoulder" type="revolute">
    <parent link="base"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="50" velocity="2"/>
  </joint>
  <joint name="elbow" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0.25 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.0" upper="2.0" effort="30" velocity="2"/>
  </joint>
</robot>
```

## Commands

### `fk --urdf PATH --joints "name=value,name=value,..."`

Real forward kinematics: parses the URDF, applies the given joint
positions, and prints each joint's resulting world-frame translation.
`--joints` is required to give an explicit value for every non-fixed joint
in the chain — it is not optional/defaulted to zero:

```
$ hydra-umc-physics-replica fk --urdf arm.urdf --joints "shoulder=0.5,elbow=0.3"
shoulder: x=0.000000 y=0.000000 z=0.300000
elbow: x=0.219396 y=0.119856 z=0.300000
```

**Missing a joint's position** — every non-fixed joint needs an explicit
value (exit `2`):

```
$ hydra-umc-physics-replica fk --urdf arm.urdf
fk: no position given for non-fixed joint 'shoulder'
```

**Missing `--urdf`** (exit `2`):

```
$ hydra-umc-physics-replica fk --joints "shoulder=0.5"
fk: missing required --urdf PATH
```

**Nonexistent URDF path** (real OS error text — this machine reports it in
Spanish; exit `2`):

```
$ hydra-umc-physics-replica fk --urdf does-not-exist.urdf
fk: could not read 'does-not-exist.urdf': El sistema no puede encontrar el archivo especificado. (os error 2)
```

**Malformed `--joints` entry** (exit `2`):

```
$ hydra-umc-physics-replica fk --urdf arm.urdf --joints "shoulder"
fk: invalid --joints value: expected 'name=value', got 'shoulder'
```

**Non-numeric joint value** (exit `2`):

```
$ hydra-umc-physics-replica fk --urdf arm.urdf --joints "shoulder=abc"
fk: invalid --joints value: 'abc' is not a valid number (joint 'shoulder')
```

**Non-finite joint value** (`nan`/`inf` parse as valid `f64`s in Rust, but
are rejected explicitly before limit or transform math ever sees them —
without this gate, a non-finite input silently produces a `NaN` pose
instead of a clear error; exit `2`):

```
$ hydra-umc-physics-replica fk --urdf arm.urdf --joints "shoulder=nan,elbow=0.2"
fk: invalid --joints value: 'nan' is not a finite number (joint 'shoulder')
```

The same check applies to every subcommand below, since all of them parse
`--joints` through the same `parse_joint_positions()` — `fk-checked` and
`validate-limits` reject a non-finite joint value with the same message and
exit code before doing anything else.

### `fk-checked --urdf PATH --joints "name=value,..."`

Same forward kinematics as `fk`, but refuses to compute a pose at all if any
given joint position is outside that joint's own `<limit>` — a fail-safe
variant, distinct from `fk`, which does not check limits.

**Within limits** — identical output to `fk` above, exit `0`:

```
$ hydra-umc-physics-replica fk-checked --urdf arm.urdf --joints "shoulder=0.5,elbow=0.3"
shoulder: x=0.000000 y=0.000000 z=0.300000
elbow: x=0.219396 y=0.119856 z=0.300000
```

**Limit violation** — refuses to compute the pose, reports every violation
found (exit `1`):

```
$ hydra-umc-physics-replica fk-checked --urdf arm.urdf --joints "shoulder=0.5,elbow=5.0"
LIMIT VIOLATION: joint 'elbow' = 5.000000 (allowed [-2.000000, 2.000000]) - refusing to compute an unreachable pose
```

### `validate-limits --urdf PATH --joints "name=value,..." [--allow-partial] [--soft] [--velocities "name=value,..."] [--efforts "name=value,..."]`

Reports every joint whose given position is outside its own `<limit>` —
without computing any pose at all (unlike `fk-checked`, which refuses to
proceed on a violation; `validate-limits` is the standalone check). A
position within 1e-9 of a limit is inside it (unit-conversion noise). Every
joint that has a limit needs a position in `--joints`: a joint left out
would be a check that was not made, so it is an error (exit `2`) unless
`--allow-partial` says that only the joints given are to be checked.

`--soft` also checks the soft range of a `<safety_controller>` (`SOFT LIMIT VIOLATION`, exit `1`): inside the hard limit, but where the
controller of the robot starts to push back. `--velocities` and `--efforts` take a `name=value` list like `--joints` and check the
magnitude of each against the `velocity` and `effort` of that joint's `<limit>` (`VELOCITY LIMIT VIOLATION`, `EFFORT LIMIT VIOLATION`,
exit `1`; the sign does not matter); a joint with no such limit, or one that does not exist, is an error (exit `2`), never a pass.
A joint that follows another (`<mimic>`) is not given in `--joints`: it is checked at the position its master gives it.

**No violations** (exit `0`):

```
$ hydra-umc-physics-replica validate-limits --urdf arm.urdf --joints "shoulder=0.5,elbow=0.3"
No limit violations.
```

**A violation** (exit `1`):

```
$ hydra-umc-physics-replica validate-limits --urdf arm.urdf --joints "shoulder=10.0,elbow=0.3"
LIMIT VIOLATION: joint 'shoulder' = 10.000000 (allowed [-3.141590, 3.141590])
```

### `jacobian --urdf PATH --target JOINT --joints "name=value,..." [--json]`

The 6 x N geometric Jacobian of the frame of `JOINT` in the world frame (three linear and three angular rows, one column per joint that moves it; a joint that follows another through `<mimic>` adds its share to the column of its master). Text output lists the columns and the six rows; `--json` prints the same as JSON. It is tested against finite differences of forward kinematics, with a follower and a prismatic joint.

### `ik --urdf PATH --target JOINT --goal X,Y,Z [--joints "name=value,..."] [--tolerance M] [--damping L] [--max-step M] [--max-iterations N]`

Inverse kinematics of the position of the frame of `JOINT` (the goal in metres, world frame) by damped least squares, starting from `--joints` (zero when omitted). Steps are limited and positions are kept inside the joint limits. Defaults: `--tolerance 1e-4` (m), `--damping 0.05`, `--max-step 0.1` (m per iteration) and `--max-iterations 200`. It prints one `joint=value` line per joint (radians or metres) and, on standard error, how close it got and in how many iterations. A goal out of reach, or one not reached within the iterations, is an error (exit code `1`) and not a pose. A missing `--urdf`, `--target` or `--goal`, or a goal that is not three finite numbers, exits with code `2`.

## Exit codes

| Code | Meaning |
|------|---------|
| `0` | ok — pose computed (`fk`/`fk-checked`), or no limit violations (`validate-limits`) |
| `1` | a real limit violation — `fk-checked` refused to compute a pose, or `validate-limits` reported one or more violations |
| `2` | usage/input error — missing `--urdf`/`--joints` value, unreadable or malformed URDF file, a malformed, unknown or repeated `--joints` entry, a limited joint with no position (`validate-limits`), or a revolute/prismatic joint with no `<limit>` (`fk-checked`, `validate-limits`) |

## Not yet wired in

This is a real, deliberately partial URDF reader and kinematics engine, not
a MuJoCo/PhysX simulation yet — see the module docs at the top of
`src/main.rs` and `src/urdf.rs` for the honest scope: a real `parent`/
`child` tree walk (branches included, `<mimic>` followers worked out), and no
actual rigid-body or contact simulation, no orientation goal in the inverse kinematics (position only).
