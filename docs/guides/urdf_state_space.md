# urdf_state_space — URDF → linear plant

Reusable URDF → **linear state-space model** pipeline. Given any robot
URDF, it linearizes the rigid-body dynamics

```text
M(q) q̈ + C(q, q̇) q̇ + g(q) = Sᵀ u
```

about an equilibrium `(q_eq, q̇ = 0, u_eq = gravity compensation)` into

```text
ẋ = A x + B Δu,   y = C x + D Δu,   x = [q − q_eq, q̇]
```

The Jacobians are **analytic** (Pinocchio `computeABADerivatives`), not
finite differences. This package does one job — URDF in, LTI plant out;
synthesis lives in {doc}`state_space_control`.

## The YAML workflow (recommended)

Describe the model per joint, **by name** — no positional vectors to get
wrong:

```yaml
model:
  urdf: package://kontrolem_example_robots/urdf/cart_double_inverted_pendulum.urdf

joints:
  cart_joint: {actuated: true, damping: 1.0}    # the only actuated joint
  joint1:     {damping: 0.05}                    # first pendulum link (passive)
  joint2:     {damping: 0.02}                    # second pendulum link (passive)

outputs:
  velocities: true

export:
  npz: model.npz
  mat: model.mat        # for MATLAB hinfsyn/mixsyn
  # dt: 0.001           # also write the discrete-time (ZOH) model
```

```bash
ros2 run urdf_state_space urdf2ss model.yaml
```

Rules: unlisted joints get `damping: 0`, `q_eq: 0`; if *any* joint sets
`actuated`, only those with `actuated: true` are actuated, otherwise all
are. Relative paths resolve against the YAML file's directory. `urdf:`
accepts a path, a `package://` URI, or a xacro file (processed without
arguments).

## Python API

```python
from urdf_state_space import build_state_space

ss = build_state_space(
    'robot.urdf',                                 # path or URDF XML string
    q_eq={'joint1': 0.2},                         # by name (or a full q vector)
    actuated_joints=['cart_joint'],               # only the cart is driven
    velocity_outputs=True,                        # y = [q; q̇] of the outputs
    joint_damping={'cart_joint': 1.0, 'joint1': 0.05, 'joint2': 0.02},
)

ss.A, ss.B, ss.C, ss.D    # numpy arrays
ss.u_eq                   # gravity-compensation torque at q_eq
print(ss.summary())       # poles, controllability, observability

ss_d = ss.c2d(0.001)      # exact ZOH discretization
ss.save_npz('model.npz')
P = ss.to_control()       # python-control StateSpace
```

`build_from_yaml(path)` runs the identical pipeline from a config file and
is what both the CLI and the wizard call — exported YAMLs are byte-wise
reproducible.

## Semantics worth knowing

- **Equilibrium check** — if holding `q_eq` requires torque on an
  *unactuated* joint, the point is not a true equilibrium for that
  actuation set; a warning is emitted (and surfaced in the wizard). The
  linearization is still computed about the requested point.
- **Gravity feedforward** — the real control law is `u = u_eq + Δu`, with
  `Δu` from the controller. `u_eq` is carried through every artifact.
- **Tangent-space state** — the state uses Pinocchio's tangent space, so
  continuous joints (`nq ≠ nv`) are handled correctly.
- **Damping** — URDF `<dynamics damping=…>` tags are *ignored* by
  Pinocchio; pass `joint_damping` explicitly.
- **Joint indexing** — all name→index mapping goes through Pinocchio's own
  `idx_q`/`idx_v`, never list position (the classic Pinocchio-adjacent
  off-by-reindexing bug).

## API reference

{doc}`../api/urdf_state_space`
