# Self-collision with capsules, inverse dynamics and MuJoCo

The URDF carries more than the joints: `<collision>` and `<inertial>` are read, and the shapes and the masses of the links feed the
three features below. Nothing here has been tried with a real robot.

## Self-collision with capsules (`capsules`, `self-collision`)

```
hydra-umc-physics-replica capsules       --urdf ROBOT.urdf [--config COLLISION.json]
hydra-umc-physics-replica self-collision --urdf ROBOT.urdf --trajectory STATES.json [--config COLLISION.json] [--json]
```

`STATES.json` is `{"states": [{"joint": position, ...}, ...]}`, the same file as `predict-collision`. Exit `0` clear, `1` a contact is
predicted, `2` bad input.

Every sphere, cylinder and box of every `<collision>` is replaced by a capsule (two end points and a radius, in the frame of its link)
that **contains** it, so the check may report a touch that would not happen and must never miss one that would:

| Shape | Capsule |
| --- | --- |
| sphere | length zero, the radius of the sphere |
| cylinder | the same axis, between the centres of the two flat faces, the same radius |
| box | along the longest side, between the centres of the two end faces, with the radius of the diagonal of the other two sides |
| mesh | none: an error, unless the configuration declares `mesh_radius_m`, the radius of a sphere around the origin of the mesh (then it is the operator's claim) |
| anything else, or numbers that are wrong | an error, never skipped |

Between two states the joints are interpolated linearly and sampled so that no point of a capsule has moved further than an
allowance from its place in the nearest sample (a bound from the offsets of the joints and the sizes of the capsules); both capsules of a
pair may move, so twice the allowance is added to the margin of the pair. Pairs of links that touch by construction are not compared:
`min_link_distance` is the number of joints between two links under which a pair is skipped (default `2`: a link and its neighbour), and
`ignore_pairs` names further pairs that are allowed to touch.

The configuration object (all optional, unknown fields are an error): `margin_m` (default `0`), `step_m` (`0.02`), `max_substeps`
(`200`), `min_link_distance` (`2`), `ignore_pairs` (`[]`), `mesh_radius_m` (none).

`capsules` prints the capsules in the frame of each link as JSON: the base for the self-occlusion mask of a camera or a depth sensor that
looks at the robot. The same data is `GET /capsules` of the service.

## Inverse dynamics (`inverse-dynamics`)

```
hydra-umc-physics-replica inverse-dynamics --urdf ROBOT.urdf --joints "..." --velocities "..." --accelerations "..." \
    [--gravity X,Y,Z] [--allow-missing-inertial] [--json]
```

The recursive Newton-Euler algorithm over the tree of links, with the mass, the centre of mass and the inertia tensor of each `<inertial>`
(the tensor is in the frame of the inertial `<origin>`, as in URDF) and gravity as the acceleration of the root link. It prints the
torque of each revolute or continuous joint (N m) and the force of each prismatic joint (N) that the motion needs, and compares them with
`<limit effort>`: exit `0` within the limits, `1` over, `2` bad input. The default gravity is `0 0 -9.81`.

- A joint that follows another (`<mimic>`) moves with it: its velocity and acceleration are derived, and its torque is folded into the
  torque of its master, so only the free joints are reported.
- A link below the root with no `<inertial>` counts as massless, which **under-estimates** the torques: it is an error unless
  `--allow-missing-inertial` is given, and the links taken as massless are always listed.
- Not modelled: friction, the inertia of the motors and gearboxes, the elasticity of links and joints, contact forces, and a load carried
  by the tool unless the URDF puts it in the inertia of the last link.

It is tested against the closed forms of a pendulum, of a vertical slide and of the planar two-link arm; against the energy balance
(the power of the joints equals the rate of change of the kinetic and potential energy) on a three-dimensional arm with tilted axes, offset
and rotated inertia, a prismatic joint and a follower; and, with the optional bridge below, against the inverse dynamics of MuJoCo itself.

## Moving the robot forward in time (`simulate`)

```
hydra-umc-physics-replica simulate --urdf ROBOT.urdf [--joints "..."] [--velocities "..."] [--torques "..."]     [--duration S] [--dt S] [--every N] [--allow-missing-inertial] [--json]
```

The forward dynamics of the `rigidbody` module: the joint accelerations for the torques given (held constant), integrated with the fourth-order
Runge-Kutta rule for `--duration` seconds (1 by default) in steps of `--dt` (0.001 by default, at most one million steps). It prints the position
of each free joint every `--every` steps (a tenth of the run by default) as a table, or as JSON with positions and speeds. Only joints that move by
themselves can be given; the ones that follow another joint move with it. The model has the `<inertial>` blocks and gravity and nothing else (no
friction, no motor dynamics, no contact with the robot): a comparison with a real arm is what turns it into more than a simulation.

## MuJoCo (`mjcf`, and the optional bridge)

```
hydra-umc-physics-replica mjcf --urdf ROBOT.urdf [--out ROBOT.xml]
```

writes the URDF as MJCF, the model format of MuJoCo, as plain text that needs no physics engine: a `<body>` per link nested as the tree of
joints, a hinge or slide per moving joint with its range and `actuatorfrcrange`, the shapes as `<geom>`, the `<inertial>` blocks (the
tensor turned into the frame of the link, as MuJoCo asks for) and the `<mimic>` joints as `<equality>` constraints. What cannot be written
faithfully (a mesh, a moving link with neither `<inertial>` nor shape) is listed on stderr as a warning.

The bridge is a Cargo feature, **off by default**; without it nothing of MuJoCo is built or needed:

```
cargo build --features mujoco            # MuJoCo installed, MUJOCO_DYNAMIC_LINK_DIR (or MUJOCO_STATIC_LINK_DIR) set
cargo build --features mujoco-download   # the build fetches MuJoCo itself
hydra-umc-physics-replica mujoco-check --urdf ROBOT.urdf --joints "..." --velocities "..." --accelerations "..."
hydra-umc-physics-replica mujoco-step  --urdf ROBOT.urdf --joints "..." --velocities "..." [--steps N]
```

`mujoco-check` prints the torques of the algorithm of this program next to those of MuJoCo's inverse dynamics and the largest
difference; `mujoco-step` plays the robot under gravity for some steps. MuJoCo's contacts are switched off in both (the robot is free), and a
robot with a `<mimic>` joint is refused because MuJoCo reports the force of the constraint on the follower. At run time the MuJoCo library
must be found by the system (on Windows, its `bin` folder on `PATH`).
