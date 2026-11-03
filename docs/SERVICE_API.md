# The computing routes of `serve`

`serve` is the only part of this program that listens on a network, and it is off unless the command is given.
`GET /health` is always there; `GET /metrics` only when the configuration file opens it (see below). With `--urdf PATH` the program also answers the routes below
for that robot; without a URDF they answer `404` (`code: compute_off`).

```
hydra-umc-physics-replica serve --urdf ROBOT.urdf [--config SERVICE.json] [--addr HOST:PORT] [--token TOKEN] [--allow-anonymous]
```

The statements of this page are about a model of the robot taken from its URDF. **Nothing in it has been tried with a real
robot**: the answers are estimates, and a program that gates motion on them must also read the documentation of each route
for what it does not cover.

## Safe by default

- It listens on `127.0.0.1:8118` only. Another address is refused unless a token is set (`--token`, or the
  `PHYSICS_REPLICA_TOKEN` environment variable) or the operator passes `--allow-anonymous` on purpose.
- With a token, every route except `GET /health` needs `Authorization: Bearer TOKEN` (`401` otherwise).
- The robot is the file given at the start. A path is never read from a request. A request may carry its own URDF text
  (`"urdf": "<robot ...>"`) only when the configuration says `"allow_inline_urdf": true` (otherwise `403`, `code: inline_urdf_off`).
- The size of a body (`max_body_bytes`, default 1 MiB, `413`), the number of samples of a trajectory (`max_samples`, default 2000) and the
  iterations of an inverse-kinematics request (`ik.max_iterations_cap`, default 500) are bounded by the configuration.
- A name of a joint that the URDF does not have is an error, and a joint with no position is an error: an unknown value is never
  read as zero.

## The configuration file

`--config` takes a JSON object; every field is optional and an unknown field is an error. The limits and tolerances of the
service live here and not in the code.

```json
{
  "language": "en",
  "max_body_bytes": 1048576,
  "max_samples": 2000,
  "allow_inline_urdf": false,
  "ik": { "max_iterations_cap": 500, "tolerance": 0.0001, "damping": 0.05, "max_step": 0.1 },
  "trajectory": {
    "position_tolerance": 1e-9,
    "velocity_tolerance": 1e-9,
    "acceleration_tolerance": 1e-9,
    "acceleration_limits": { "shoulder": 8.0 },
    "default_acceleration_limit": null,
    "check_soft_limits": true,
    "check_effort": false,
    "gravity": [0, 0, -9.81],
    "allow_missing_inertial": false,
    "self_collision": "auto",
    "collision": { "margin_m": 0.0, "step_m": 0.02, "min_link_distance": 2, "ignore_pairs": [], "mesh_radius_m": null },
    "max_listed": 64
  },
  "collision": { "margin_m": 0.01 },
  "metrics": { "enabled": true }
}
```

`metrics` opens `GET /metrics` (the text format of Prometheus); it is off unless `enabled` is `true`, and closed the route answers `404` like any unknown path, even with the token. Open, it answers a caller on this machine, or any caller that presents the bearer token when the service has one (`401` otherwise). It publishes numbers only (version, uptime, responses by class of status and health); no label carries a path, an address or a robot.

`language` is the language of the `message` of the answers (`en`, `es`, `de`, `fr`, `it`, `ja`, `zh`, `pt`); a request can ask
for another with `?lang=xx` or `Accept-Language`. The `collision` object may be given at the top or inside `trajectory`. The command
`check-trajectory --urdf ROBOT.urdf --samples SAMPLES.json --config SERVICE.json` runs the same check on files.

## Every answer

A JSON object. `error` (an error) or `summary` (an answer) is the stable English sentence, `code` its stable code and `message` the
same sentence in the language of the request. A bad request has `detail` with the field or the joint, in English.

| Status | `code` | When |
| --- | --- | --- |
| `400` | `bad_request` | invalid JSON, a missing or wrong field, an unknown joint, a missing position |
| `401` | `unauthorized` | a token is set and the request has none or a wrong one |
| `403` | `inline_urdf_off` | the request carries a URDF and the configuration does not allow it |
| `404` | `not_found`, `compute_off` | an unknown route, or no URDF was given to `serve` |
| `405` | `method_not_allowed` | the route does not take this method |
| `413` | `payload_too_large` | the body is over `max_body_bytes` |
| `422` | `bad_request` | `/fk` with `"checked": true` and a position outside its limit |

## `POST /fk`

```json
{ "joints": { "shoulder": 0.5, "elbow": -0.2 }, "checked": false }
```

Answers `{"poses": [{"joint", "position": {x, y, z}, "quaternion": {w, x, y, z}}, ...], "checked", "note"}`: the world pose of the frame
of every joint, in metres, with the rotation as a unit quaternion (`w >= 0`). `joints` names every joint that moves by itself; a joint
that follows another (`<mimic>`) is derived and a position given for it is an error. With `"checked": true` a position outside the
`<limit>` of its joint is refused with `422` and the pose is not computed.

## `POST /ik`

```json
{ "target": "tool", "goal": [0.4, 0.1, 0.3], "joints": { "shoulder": 0.2, "elbow": 0.3 },
  "tolerance": 0.0001, "damping": 0.05, "max_step": 0.1, "max_iterations": 200 }
```

Damped least squares on the position of the frame of joint `target`, from the starting `joints`, with the positions kept inside the
limits. The four numbers are optional (the defaults are those of the configuration) and `max_iterations` never goes over
`ik.max_iterations_cap`. Answers `200` with `{"reached": true, "residual", "iterations", "joints": {...}}`, or `{"reached": false,
"residual", "iterations"}` when the goal is out of reach: no pose is made up for it. Only the position is solved, not the
orientation.

## `POST /check-trajectory`

```json
{ "samples": [ { "t": 0.0, "joints": { "shoulder": 0.0 } }, { "t": 0.1, "joints": { "shoulder": 0.05 } } ] }
```

Checks a sampled trajectory (`t` in seconds, strictly increasing) before it is sent to the hardware bridge:

- **position**: against `<limit lower upper>` (and, with `check_soft_limits`, the `<safety_controller>` range), with `position_tolerance`;
- **velocity**: the finite difference between consecutive samples against `<limit velocity>`, with `velocity_tolerance`;
- **acceleration**: the second difference around each interior sample against `acceleration_limits` (or `default_acceleration_limit`).
  The URDF has no acceleration limit, so a joint with none in the configuration is not checked for it;
- **effort** (with `check_effort`): the joint torques of inverse dynamics (see `DYNAMICS_AND_COLLISION.md`) against `<limit effort>`
  at the interior samples;
- **self-collision** (`self_collision`: `off`, `auto` or `on`): the capsules built from `<collision>` between the samples.

Answers `{"ok", "complete", "samples", "violations_total", "violations": [{kind, joint, sample, t, value, limit}], "max_velocity",
"max_acceleration", "self_collision", "notes"}`. `kind` is `position`, `soft_position`, `velocity`, `acceleration` or `effort`.
`ok` is false when a limit is broken or a contact is predicted. **`complete` is false when a check was switched off or could not run**
(no `<collision>` shapes, a mesh without a declared radius, fewer than three samples for the effort, a link counted as massless): a
caller that gates motion on this answer must require `ok` **and** `complete`. Speed and acceleration are finite differences, so a
spike between two samples is not seen.

## `POST /inverse-dynamics`

```json
{ "joints": {...}, "velocities": {...}, "accelerations": {...}, "gravity": [0, 0, -9.81] }
```

The torque (revolute and continuous joints, N m) or force (prismatic joints, N) that each free joint must give; see
`DYNAMICS_AND_COLLISION.md`. Answers `{"torques", "within_effort_limits", "effort_violations", "links_without_inertial", "note"}`. A link
without `<inertial>` is an error unless `trajectory.allow_missing_inertial` is true, and is always listed.

## `GET /capsules` (or `POST`)

The capsules that stand for the `<collision>` shapes, in the frame of their link: `{"capsules": [{"link", "a": [x, y, z], "b": [x, y, z],
"radius_m"}]}`. A consumer that masks the robot out of what a sensor sees can use them; the capsules contain the shapes, so the mask
over-covers rather than under-covers.

## Contracts

The shape of these answers is described here. A shared contract for the routes (an OpenAPI description in the common SDK of the
ecosystem) is not published yet; until it is, a client should read the fields named above and ignore the ones it does not know.
