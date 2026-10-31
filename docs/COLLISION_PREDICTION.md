# Collision prediction by forward kinematics

`predict-collision` answers one question: if the joints of the robot go through
this short list of states, does any part of the robot enter a protected zone, or
touch another part of the robot?

It is pure kinematics. It places the links of the URDF with the same forward
kinematics as `fk` and compares them with zones, always in the direction that
**over-estimates**: it may report a contact that would not happen, and it must
never miss one that would. **Nothing in it has been tried with a real robot.**

HYDRA-UMC-TWIN exposes the same computation as `POST /predict/collision`
(see the documentation of the twin) and HYDRA-UMC-HIL-BRIDGE can ask the twin
for it in shadow mode, where it only informs and never blocks.

## What goes in

```
hydra-umc-physics-replica predict-collision --urdf ROBOT.urdf --zones ZONES.json \
    --config SIZES.json --trajectory STATES.json [--base BASE.json] [--json]
```

- `--urdf`: the robot. Serial arms and branching trees, `<mimic>` joints and
  prismatic joints are read by the same code as `fk`.
- `--zones`: a zones file of HYDRA-UMC-SAFETY-ZONES. Boxes (`min`, `max`) are
  always understood; oriented boxes (`obb`), vertical cylinders and capsules
  only when the file says `"geometry": {"shapes": true}`, exactly as in the zones
  service, and `"mode": "bounding"` replaces each shape by the box that contains
  it. Each zone is `warning` or `danger`.
- `--config`: the sizes of the robot (below). They have no default.
- `--trajectory`: `{"states": [{"joint": position, ...}, ...]}`, at most 256
  states. A state is a position for every joint that moves by itself (a
  `<mimic>` follower is derived, a fixed joint has none).
- `--base`: optional `{"joint": position, ...}`. A state that leaves a joint out
  gets it from here; names the robot does not have are ignored. A joint that
  neither the state nor the base has is an error: an unknown position is never
  read as zero.

Exit status: `0` no contact predicted, `1` a contact with a zone or with the
robot itself is predicted, `2` the input is wrong (nothing is printed on the
standard output then).

## The configuration (the sizes of the robot)

The URDF of this project carries no collision geometry, so how thick a link is
cannot be known here. The person who runs the check declares it:

| field | meaning | default |
|-------|---------|---------|
| `link_radius_m` | radius around the segment of every link | required, above zero |
| `tool_extent_m` | how far the tool reaches beyond the frame of the last joint of each branch | required, not negative |
| `link_radii` | radius by the name of the joint at the end of a link | none |
| `margin_m` | extra distance added to every comparison | `0` |
| `resolution_m` | a segment is cut into pieces no longer than this | `0.05` |
| `step_m` | the most a point of the robot may move between two samples | `0.02` |
| `max_substeps` | the most samples between two states | `200` |
| `self_collision` | compare the robot with itself | `true` |
| `self_skip` | segments whose distance in the tree of joints is below this are not compared (`1`: only those that share a joint) | `1` |

Unknown fields are refused. A radius that is smaller than the real link makes
the answer wrong in the unsafe direction: the model cannot see it.

## The model

1. **Skeleton.** For every joint, the segment from the origin of the frame of its
   parent (the base for the first joint) to the origin of its own frame. A
   prismatic joint adds the segment its travel draws. A joint that nothing hangs
   from gets a sphere of radius `tool_extent_m` plus its link radius. Each
   segment is a capsule of the radius the configuration says.
2. **Pieces.** A segment is cut into pieces no longer than `resolution_m`, and
   each piece is replaced by the sphere around its middle that contains it (its
   radius is the link radius plus half the piece). The sphere is larger than the
   piece, so this can only add contacts.
3. **Zones.** The distance from the middle of a piece to a zone is the exact
   distance to the solid, reduced by one nanometre (as in the zones service). A
   contact is `distance - radius - margin <= 0`.
4. **Self contact.** Two spheres of two segments touch when the distance of their
   middles is at most the sum of the radii plus the margin. Segments that share a
   joint always touch at it and are not compared; `self_skip` widens that.
5. **Between two states** the joints are interpolated linearly and sampled. How
   far any point of the robot can move between two samples is bounded without
   looking at the pose: each moving joint moves a point by at most the change of
   the joint times the distance from the point to the joint (the sum of the
   offsets of the URDF joints in between, plus the sizes of the link and the
   tool), and a prismatic joint by its change. The number of samples is chosen so
   that this is at most `step_m`; when `max_substeps` stops it, the bound grows
   instead of the sampling being thinner unnoticed. **Half of the bound is
   added to the margin of every sample**, because a point is never farther than
   that from the nearest sample. The largest of these is reported as
   `max_motion_allowance_m`.
6. **Rounding.** A nanometre is added to the margin of every comparison so that
   the rounding of the arithmetic cannot hide a touch.

## What it reports

`--json` prints one object:

- `collision`: any contact with a zone, or with the robot itself.
- `level`: `clear`, `warning` or `danger` (the worst zone level reached).
- `self_collision`: the robot touches itself.
- `hits`: the first contacts (at most 32; one per zone and link), each with `at`
  (where on the trajectory: `1.5` is half way from the second state to the
  third), the zone and its level or the two links, and the `clearance_m`
  (negative: how far inside).
- `hits_total`, `samples`, `states`.
- `min_clearance_m`: the smallest clearance to a zone over every sample, after the
  margin and the sampling allowance (`null` without zones), and
  `min_self_clearance_m` for the robot with itself.
- `outside_limits`: states whose joints are outside their URDF limits. This is
  information; it does not make a collision.

## How it is tested

- The characterization tests fix the forward kinematics the prediction stands on
  (the frames of a planar arm).
- Unit tests cover the shapes of the zones against computed distances, the
  refusal of every malformed input, the self contact of a folded arm, prismatic
  joints and the base state.
- Two property tests with a seeded generator check the rule that matters. In the
  first, a zone is put around a point that really lies inside a link at a random
  moment of a random trajectory, and the prediction must never say `clear`. In
  the second the sampling is made coarse on purpose and the zone sits at the tool
  at a moment between two samples, so only the sampling allowance can find it.
  Both are checks of the model against itself: they show that the program keeps
  its own rule, not that the rule matches a real robot.

## Limits, said plainly

- **Not tried with a real robot, a real cell or real zones.**
- A link whose real shape sticks out of the radius declared for it, a tool longer
  than `tool_extent_m` and anything the tool carries are not seen.
- The zones are the ones in the file. An obstacle that is not in it is not seen,
  and neither is a person: detection is the job of other parts of the system.
- The calibration, the separation model and the velocity margin of the zones file
  are not applied. Use `margin_m` for the distance to keep.
- The path between two states is assumed to be a straight line in joint space.
  A controller that follows another path is not covered.
- No dynamics: the time to stop, the overshoot of the controller and the speed
  are not modelled.
- Joint limits are only reported, never enforced.
- The answer is information. It is not a safety function and replaces neither
  the interlock of the bridge nor the protective devices of the cell.
