# Reference — ros2_control interface conventions

> **For:** integrators and developers. **Assumes:** familiarity with ros2_control `StateInterface`/`CommandInterface`/`<gpio>`. **Scope:** the interface names, `<gpio>` conventions, and sim hardware plugins Kontrol'Em uses. For the rationale of carrying non-joint state this way, see [Explanation → State on a manifold](../explanation/state-and-non-joint-data.md).

## Joint interfaces

| Interface | Direction | Default name | Used by |
|---|---|---|---|
| position | state | `position` | all laws |
| velocity | state | `velocity` | all except LQG (positions only) |
| effort | command | `effort` | all laws |

The names are configurable via `state_position_interface`, `state_velocity_interface`, `command_interface`.

## Floating-base `<gpio>` — `floating_base`

The SE(3) base pose and spatial twist are decomposed into **13 scalar state interfaces** on a `<gpio name="floating_base">` block, in this exact order:

| # | Interface name | Meaning |
|---|---|---|
| 0 | `pose.position.x` | base position x |
| 1 | `pose.position.y` | base position y |
| 2 | `pose.position.z` | base position z |
| 3 | `pose.orientation.x` | quaternion x |
| 4 | `pose.orientation.y` | quaternion y |
| 5 | `pose.orientation.z` | quaternion z |
| 6 | `pose.orientation.w` | quaternion w |
| 7 | `twist.linear.x` | base linear velocity x |
| 8 | `twist.linear.y` | base linear velocity y |
| 9 | `twist.linear.z` | base linear velocity z |
| 10 | `twist.angular.x` | base angular velocity x |
| 11 | `twist.angular.y` | base angular velocity y |
| 12 | `twist.angular.z` | base angular velocity z |

Claimed and reassembled by `BaseStateSensor`, which normalizes the quaternion and maps these into `q[0..6]` / `v[0..5]`. The `<gpio>` name is configurable via `base_gpio`.

## Contact `<gpio>` — `contact`

One scalar state interface per foot on a `<gpio name="contact">` block, named `contact.<frame>`:

| Interface name | Meaning |
|---|---|
| `contact.<foot_frame>` | Contact state of that foot (0 = swing, 1 = stance; a probability in between). |

Claimed by `ContactSensor`. The `<gpio>` name is configurable via `contact_gpio`, and the foot frames via `contact_frames`. In the bundled standing demo the sim reports all feet as `1.0` (ground-truth all-stance).

## Sim hardware plugins (`kontrolem_description`)

Self-contained ros2_control `SystemInterface` plugins that integrate the URDF's true dynamics, so demos run without Gazebo.

| Plugin | For | Exports | Hardware params |
|---|---|---|---|
| `kontrolem_description/CartPoleSimSystem` | fixed-base, Euclidean (`nq == nv`) | joint position/velocity | — |
| `kontrolem_description/FloatingBaseSimSystem` | floating base, free-fall (M3 spike) | joints + `floating_base` gpio | — |
| `kontrolem_description/FloatingContactSimSystem` | floating base, feet pinned (standing WBC) | joints + `floating_base` gpio + `contact` gpio | `contact_frames` (csv), `base_height`, `baumgarte_kp`, `baumgarte_kd` |

All three hold the plant at its initial state until a controller claims the effort interfaces (so an unstable robot doesn't free-fall during bring-up).

### Hardware parameters (`FloatingContactSimSystem`)

| Param | Type | Default | Meaning |
|---|---|---|---|
| `contact_frames` | csv string | — (required) | Foot frames to pin. |
| `base_height` | double | `0.0` | Nominal base z at start. |
| `baumgarte_kp` | double | `400.0` | Contact position-stabilization gain. |
| `baumgarte_kd` | double | `40.0` | Contact velocity-stabilization gain. |

## Joint `initial_value`

Sim plugins seed the initial configuration from each joint's `<state_interface><param name="initial_value">…`. For the standing WBC these must describe the same posture as `wbc.base_height` + `wbc.nominal_posture`.
