# Reference — Controller parameters

> **For:** integrators and developers. **Assumes:** you know what a ROS 2 parameter and a controller YAML are. **Scope:** every ROS parameter read by `kontrolem_ros2_control/KontrolemController`, plus the `controller_manager` settings that matter. This page is for lookup, not learning — see the [how-to guides](../index.md#how-to-guides) to accomplish tasks.

All parameters are set under `kontrolem_controller: { ros__parameters: … }` unless noted. Array lengths use `nq` (configuration size) and `nv` (velocity size); for a fixed-base robot `nq == nv`, for a floating base `nq == nv + 1`.

## controller_manager

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `update_rate` | int | — | Control loop frequency (Hz). Demos use 200 (fixed-base) or 500 (WBC). |

## Common

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `control_law` | string | `lqr` | Paradigm (single-controller mode): `lqr` \| `lqg` \| `mpc` \| `qp` \| `wbc`. Ignored if `control_laws` is set. |
| `control_laws` | string[] | `[]` | Multi-controller mode: laws the Supervisor hosts (first is initially active). Non-empty enables live switching over `~/switch_controller`. |
| `switch_blend_ticks` | int | `20` | Command-blend length (ticks) at a switch; larger = gentler handoff. `1` = hard switch. |
| `actuated_joints` | string[] | `[]` (required) | Joints the controller commands, in command order. |
| `robot_description` | string | `""` | URDF XML. Injected by the launch file; do not put in YAML. |
| `command_interface` | string | `effort` | ros2_control command interface written to the actuated joints. |
| `state_position_interface` | string | `position` | State interface read for joint position. |
| `state_velocity_interface` | string | `velocity` | State interface read for joint velocity. |
| `safe_action` | string | `zero` | Supervisor fallback when `status().ok` is false. `zero` is the only implemented action. |
| `publish_diagnostics` | bool | `false` | Publish `ControllerDiagnostics` on `~/diagnostics` each tick. |

## Problem / reference

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `reference_type` | string | `setpoint` | `setpoint` → Regulation; `harmonic` → Tracking with the built-in harmonic source. |
| `q_ref` | double[] | `[]` | Setpoint configuration, length `nq` (used when `reference_type: setpoint`). |
| `v_ref` | double[] | `[]` | Setpoint velocity, length `nv`. |
| `reference.center` | double[] | `[]` | Harmonic per-coordinate center, length `nq`. |
| `reference.amp` | double[] | `[]` | Harmonic per-coordinate amplitude, length `nq`. `0` holds a coordinate fixed. |
| `reference.phase` | double[] | `[]` | Harmonic per-coordinate phase (rad), length `nq`. |
| `reference.omega` | double | `0.5` | Harmonic angular frequency (rad/s). |

## LQR (`control_law: lqr`)

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `lqr.q_diag` | double[] | identity | State-cost diagonal, length `2*nv`, order `[q; v]`. |
| `lqr.r_diag` | double[] | identity | Input-cost diagonal, one per actuated joint. |
| `lqr.q_dev_max` | double | `0.5` | Trust-region size: max Q-weighted state deviation before failing safe. |

## LQG (`control_law: lqg`)

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `lqg.q_diag` | double[] | identity | Control state-cost diagonal, length `2*nv`. |
| `lqg.r_diag` | double[] | identity | Control input-cost diagonal, per actuated joint. |
| `lqg.w_diag` | double[] | identity | Process-noise covariance diagonal, length `2*nv`. |
| `lqg.v_diag` | double[] | identity | Measurement-noise covariance diagonal, length `nq`. |
| `lqg.innov_max` | double | `0.5` | Innovation gate: max filter surprise before failing safe. |

## MPC (`control_law: mpc`)

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `mpc.q_diag` | double[] | identity | State-cost diagonal, length `2*nv`. |
| `mpc.r_diag` | double[] | identity | Input-cost diagonal, per actuated joint. |
| `mpc.horizon` | int | `30` | Number of prediction steps. |
| `mpc.dt_mpc` | double | `0.02` | Seconds per prediction step. |
| `mpc.tau_max` | double | `100.0` | Torque limit enforced inside the QP. |

## QP task-space (`control_law: qp`)

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `qp.task_weight` | double[] | ones | Per-DoF weight on the acceleration task, length `nv`. |
| `qp.kp` | double | `50.0` | Task stiffness (position error → desired acceleration). |
| `qp.kd` | double | `10.0` | Task damping. |
| `qp.tau_max` | double | `5.0` | Torque limit enforced inside the QP. |

## Floating base & WBC (`control_law: wbc`)

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `base_type` | string | `fixed` | `floating` enables the floating-base state path (base + contacts via `<gpio>`). |
| `base_gpio` | string | `floating_base` | Name of the `<gpio>` block carrying the SE(3) base scalars. |
| `contact_gpio` | string | `contact` | Name of the `<gpio>` block carrying the per-foot contact scalars. |
| `contact_frames` | string[] | `[]` | Contact/foot frame names (used by the WBC and the contact sensor). |
| `wbc.base_height` | double | `0.0` | Nominal base height `z` of the standing posture. |
| `wbc.nominal_posture` | double[] | `[]` | Nominal joint angles, in `actuated_joints` order. |
| `wbc.kp_base` | double | `100.0` | Base pose/orientation stiffness. |
| `wbc.kd_base` | double | `20.0` | Base twist damping. |
| `wbc.kp_post` | double | `25.0` | Joint posture stiffness. |
| `wbc.kd_post` | double | `5.0` | Joint posture damping. |
| `wbc.w_base` | double | `100.0` | Task weight on the 6 base DoF. |
| `wbc.w_post` | double | `1.0` | Task weight on the joints. |
| `wbc.w_force` | double | `1e-4` | Contact-force regularization weight. |
| `wbc.w_tau` | double | `1e-4` | Torque regularization weight. |
| `wbc.mu` | double | `0.7` | Friction coefficient (linearized pyramid). |
| `wbc.tau_max` | double | `40.0` | Per-joint torque limit. |
| `wbc.max_iter` | int | `200` | OSQP iteration cap (hard-RT solve-time bound; nominal standing/recovery is ~50). Hitting it yields `ok = false` → supervisor fallback. |

## Notes

- `q_diag`/`r_diag`/`w_diag`/`v_diag`/`task_weight` fall back to identity/ones if their length does not match the expected size — they are silently ignored, not an error.
- When `base_type: floating`, the joint interfaces claimed are those in `actuated_joints` (the free-flyer root is not an encoder); for a fixed base, all model joints are claimed. See [ros2_control interfaces](ros2control-interfaces.md).
