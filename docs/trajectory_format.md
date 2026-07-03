# The RobotTrajectory format

`RobotTrajectory` (npz schema **`robot_trajectory/1`**, implemented in
{mod}`state_space_control.trajectory`) is the framework's canonical
motion-interchange format. This page is the normative spec any new
producer or consumer targets.

Design constraints:

- **Robot-agnostic** — fixed-base manipulators, mobile robots,
  floating-base / legged systems, aerial vehicles.
- **Self-describing** — given only the file, a reader can say what
  produced it and how to reproduce it.
- **Plain-numpy** — the implementing module imports nothing but numpy;
  files load with `allow_pickle=False`.

## Arrays

| npz key | Shape | Required | Meaning |
|---|---|---|---|
| `schema` | scalar str | yes | literally `robot_trajectory/1` |
| `t` | `(N,)` | yes | simulation time, strictly increasing |
| `q` | `(N, nj)` | yes | **absolute** joint positions |
| `qd` | `(N, nj)` | yes | joint velocities |
| `joint_names` | `(nj,)` str | yes | column order for `q`/`qd` |
| `actuated_joint_names` | str | no | subset of `joint_names` |
| `u` | `(N, m)` | no | control inputs, **deviation form** `u − u_eq` |
| `base_pose` | `(N, 7)` | no | root pose `[x y z, qx qy qz qw]`, world frame |
| `base_twist` | `(N, 6)` | no | root twist |
| `header` | scalar str | yes | JSON: `{"meta": {...}, "events": [...]}` |

Conventions that consumers must honor:

- Joint positions are **absolute**: producers apply the operating-point
  offset (`q = q_eq + δq`); consumers never see deviation coordinates.
- `base_pose` **absent means fixed base** — do *not* invent an
  identity-pose transform; the URDF root stays wherever the consumer's
  world anchor puts it. When present: quaternion order `xyzw`, world
  frame, slerp interpolation.
- Mimic joints are intentionally **absent** from trajectories —
  `robot_state_publisher` and the web viewer both derive them from the
  URDF.
- `validate()` runs on every load: shape consistency, monotonic time,
  finite `q`, events inside the time span. A malformed producer fails at
  the boundary, not deep inside a renderer.

## Events

Time-stamped annotations, serialized in the `header` JSON:

```json
{"t": 1.5, "type": "limit_violation", "subject": "joint1",
 "message": "joint1 leaves [-3.14, 3.14]",
 "data": {"t_start": 1.5, "t_end": 2.1}}
```

Reserved types (v1): `limit_violation`, `linear_validity`, `instability`,
`settling_time`, `overshoot_peak`, `saturation`, `user`. **Consumers
ignore unknown types**, so the list can grow without a schema bump.

## Reproducibility metadata (`meta`)

All keys optional except `schema` and `source` — a bag importer simply
omits what it doesn't know:

```text
schema:            'robot_trajectory/1'
source:            'linear-state-space' | 'rosbag' | 'mujoco' | …
created:           ISO-8601 timestamp
robot:             {name, urdf_source, urdf_sha256}
operating_point:   {q_eq: {joint: value}, u_eq: [..]}
controller:        {type, params}
excitation:        {name, params, channel, injection}
x0:                initial deviation state
sim:               {solver, t_final, n_points, dt, stable, capped}
versions:          {state_space_control, urdf_state_space, numpy, scipy}
```

With this, the npz alone is enough to regenerate the trajectory through
`ss_design` + `simulate_response` — the same "artifact reproduces the
pixels" guarantee the wizard makes for `model.yaml` / `design.yaml`.

## Reading and writing

```python
from state_space_control.trajectory import RobotTrajectory, FrameSampler

traj = RobotTrajectory.from_npz('trajectory.npz')   # validates on load
frame = FrameSampler(traj).frame_at(1.25)           # interpolated RobotFrame
traj.save_npz('copy.npz')
```

Hand-building one (e.g. in an importer or a test):

```python
import numpy as np
from state_space_control.trajectory import RobotTrajectory

t = np.linspace(0.0, 2.0, 100)
traj = RobotTrajectory(
    t=t,
    q=np.column_stack([np.sin(t), 0.5 * t]),
    qd=np.column_stack([np.cos(t), np.full_like(t, 0.5)]),
    joint_names=['joint1', 'joint2'],
    meta={'source': 'my-importer'},
).validate()
traj.save_npz('imported.npz')
```

That file plays in RViz and in the wizard exactly like a simulated one —
the visualization subsystem requires only a valid trajectory.
