# HYDRA-UMC-PHYSICS-REPLICA — CLI Reference

`hydra-umc-physics-replica` is a single Rust binary (`src/main.rs`) that
runs real forward-kinematics and joint-limit validation over a (documented,
deliberately partial) URDF subset — see `src/urdf.rs`'s own module docs for
what's parsed today (a single serial chain in document order, not a
branching URDF tree). Every example below was captured from a real, built
release binary, run against a hand-written but structurally real URDF
fixture (this repo ships no sample `.urdf` file of its own) — the output
shown is real, not illustrative.

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
HYDRA-UMC-PHYSICS-REPLICA v0.0.3
High-fidelity MuJoCo/PhysX simulation of URDF kinematic chains for the Digital Twin.
```

## Fixture used below

None of these examples needs real hardware, but they do need a real URDF
file. This repo doesn't ship a sample one, so the fixture below was
hand-written to match the exact shape `src/urdf.rs` parses (a
`<joint name type>` with `<origin>`/`<axis>`/`<limit>` children) — a
2-joint shoulder/elbow serial chain, the same fixture shape
`src/corpus.rs`'s own `shoulder_elbow_chain()` test helper builds
programmatically:

```xml
<!-- arm.urdf -->
<robot name="shoulder_elbow_arm">
  <joint name="shoulder" type="revolute">
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="50" velocity="2"/>
  </joint>
  <joint name="elbow" type="revolute">
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

### `validate-limits --urdf PATH --joints "name=value,..."`

Reports every joint whose given position is outside its own `<limit>` —
without computing any pose at all (unlike `fk-checked`, which refuses to
proceed on a violation; `validate-limits` is the standalone check).

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

## Exit codes

| Code | Meaning |
|------|---------|
| `0` | ok — pose computed (`fk`/`fk-checked`), or no limit violations (`validate-limits`) |
| `1` | a real limit violation — `fk-checked` refused to compute a pose, or `validate-limits` reported one or more violations |
| `2` | usage/input error — missing `--urdf`/`--joints` value, unreadable or malformed URDF file, or a malformed `--joints` entry |

## Not yet wired in

This is a real, deliberately partial URDF reader and kinematics engine, not
a MuJoCo/PhysX simulation yet — see the module docs at the top of
`src/main.rs` and `src/urdf.rs` for the honest scope: single serial chains
only (no branching URDF tree walk from `parent`/`child` link names), and no
actual rigid-body or contact simulation.
