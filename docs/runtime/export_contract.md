# Export contract — `<name>_ros2_control.yaml`

This document is the **single source of truth** for what the runtime
controllers in this repository load. It mirrors the writer in the parent
repo, `state_space_setup_assistant/ros2_control_export.py`
(`ros2_control_yaml_dict()`), tested there by
`test/test_ros2_control_export.py`. If the two ever disagree, the parent's
writer wins and this file plus the loaders must be updated.

## The file

Of the whole bundle directory written by the setup assistant
(`plant.npz`, `controller.npz`, `model.yaml`, benchmarks, …), the runtime
reads exactly **one** file: `<name>_ros2_control.yaml` (e.g.
`lqr_ros2_control.yaml`). It is plain YAML — flat nested lists, row-major —
loadable with `yaml-cpp` + Eigen, no NumPy required.

```yaml
schema_version: 1
controller_type: lqr          # or lqg / hinf / hinf_mixsyn / pid
name: lqr                     # same string as controller_type
joint_names: [...]            # full state order (Pinocchio movable-joint order)
q_eq: [...]                   # length n_joints — equilibrium positions
actuated_joint_names: [...]   # order of K's rows / u's entries
u_eq: [...]                   # feedforward effort at the operating point
output_names: [...]           # measured outputs, e.g. "cart_joint.q"
# exactly ONE of the following two groups is present:
K: [[...], ...]               # static-gain designs: n_actuated x 2*n_joints
ctrl_A: [[...], ...]          # dynamic compensators (lqg/hinf/hinf_mixsyn/pid)
ctrl_B: [[...], ...]          #   ctrl_B is n_ctrl x len(output_names)
ctrl_C: [[...], ...]          #   ctrl_C is n_actuated x n_ctrl
ctrl_D: [[...], ...]          #   ctrl_D is n_actuated x len(output_names)
validity_region:              # OPTIONAL (schema extension, see below)
  <joint_name>: <q_dev_max>   #   per-joint bound on |q - q_eq|
info: {...}                   # informational scalars only — NOT consumed
```

## State, input, and output conventions

- **State** is the deviation vector, positions first, then velocities:

  `x = [q(joint_names) − q_eq,  qdot(joint_names)]`, length `2·n_joints`.

  `joint_names` is the Pinocchio *movable-joint* order (URDF kinematic
  order, skipping the "universe" joint). Equilibrium velocity is zero.
  **There is no `x_eq` field** — the equilibrium is `q_eq` only.

- **Input** `u` is the absolute effort on `actuated_joint_names`, in that
  order. The exported gains act on deviations; `u_eq` is the feedforward
  (gravity compensation) added back at runtime.

- **Outputs** (`output_names`) name measured signals: `"<joint>.q"` is the
  position of `<joint>`, `"<joint>.qd"` its velocity. The measured output
  deviation is

  `y_i = q_measured(joint) − q_eq[joint]` for `.q` entries,
  `y_i = qd_measured(joint)` for `.qd` entries.

## Control laws

- **Static gain** (`K` present — LQR):

  `u = u_eq − K · x`      (with `x` the deviation state above)

- **Dynamic compensator** (`ctrl_A/B/C/D` present — LQG, H∞, PID). The
  export is one combined LTI from measured output deviation `y` to command
  deviation, **sign already folded in** (for LQG:
  `ctrl_A = A − B·K − L·C`, `ctrl_B = L`, `ctrl_C = −K`, `ctrl_D = 0` —
  the Kalman gain is *not* exported separately):

  `ξ̇ = ctrl_A·ξ + ctrl_B·y`
  `u  = u_eq + ctrl_C·ξ + ctrl_D·y`

  The compensator state `ξ` starts at zero on activation.

- **Reference tracking** (runtime extension, not in the export): a reference
  shifts the regulation target. Static gain: `u = u_eq − K·(x − x_ref)`.
  Dynamic: the compensator is fed `y − y_ref`. References default to zero
  deviation (regulate to the operating point).

## Discretization (runtime responsibility)

All exported gains are **continuous-time** — the bundle carries no rate or
`dt`. Static gains need no discretization. Dynamic compensators are
discretized once, in `on_configure()`, at the controller update rate using
the **Tustin (bilinear)** transform, matching
`scipy.signal.cont2discrete(..., method='bilinear')`:

```
α  = dt / 2
M  = (I − α·ctrl_A)⁻¹        # computed once, outside the realtime loop
Ad = M · (I + α·ctrl_A)
Bd = M · ctrl_B · dt
Cd = ctrl_C · M
Dd = ctrl_D + α · Cd · ctrl_B   (= ctrl_D + α · ctrl_C · M · ctrl_B)
```

then per cycle: `u = u_eq + Cd·ξ + Dd·y`, `ξ ← Ad·ξ + Bd·y`.

The discretization `dt` comes from the controller's `update_rate` parameter,
falling back to the controller manager's rate. **Set `update_rate` explicitly
on any dynamic-compensator controller (LQG/H∞):** the controller manager
propagates its own rate to a controller only *after* `on_configure()`, so
`get_update_rate()` reads 0 at discretization time. Static-gain controllers
(LQR) are unaffected — they never discretize. Keep the parameter equal to
`controller_manager.update_rate`.

## Validity region

`validity_region` is an **optional** mapping `{joint_name: q_dev_max}`
(same units as the joint: rad or m) bounding `|q − q_eq|` per joint. The
parent exporter does not emit it yet (planned companion change); loaders
must accept its absence. Runtime precedence for the guard bound of a joint:

1. explicit per-joint ROS parameter,
2. `validity_region` entry from the artifact,
3. global `validity.q_dev_max_default` ROS parameter.

## Loader requirements

- Reject files whose `schema_version` is not `1`.
- Require: `controller_type`, `joint_names`, `q_eq` (same length),
  `actuated_joint_names`, `u_eq` (same length), `output_names`.
- Require **exactly one** of `K` / the complete `ctrl_A..ctrl_D` group, with
  consistent shapes (see schema comments above).
- Ignore `info` and any unknown keys (forward compatibility).

## Referencing an artifact at runtime

Controllers take the artifact location in the `artifact_path` ROS parameter.
Two forms are accepted:

- a plain filesystem path (absolute or relative to the process CWD), or
- a `package://<pkg>/<relative/path>` URI, resolved against the ament index
  at configure time (via `ament_index_cpp::get_package_share_directory`).

Prefer the `package://` form in installed configs: it is independent of the
install prefix, so the same YAML works across machines and colcon overlays —
e.g. `package://lqr_controller/config/lqr_cart_double_pendulum_ros2_control.yaml`.
