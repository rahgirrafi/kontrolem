# lqr_controller

Static state-feedback controller for an `lqr` artifact.

**Law.** `u = u_eq − K·(x − x_ref)`, where `x = [q − q_eq, q̇]` is the deviation
state and `x_ref` the (deviation) setpoint. With the default reference the
controller regulates to the operating point (`x_ref = 0`), so `u = u_eq − K·x`.

**Artifact.** Requires a static gain `K` of shape `n_actuated × 2·n_joints`
(no `ctrl_*`). Loading an `lqg`/`hinf` artifact is rejected at configure time.

**Interfaces.**
- Commands: `effort` on each actuated joint.
- States: `position` and `velocity` on every joint (the law needs the full
  state).
- References (chainable): `<joint>/position` then `<joint>/velocity`, absolute
  units; defaults are `q_eq` / `0`.

**Parameters.** `artifact_path`, `command_interface` (default `effort`),
`state_position_interface`, `state_velocity_interface`, and the `validity.*`
guard settings. No discretization, so `update_rate` is irrelevant here.

## Demo

```bash
ros2 launch lqr_controller lqr_demo.launch.py
```

Balances the cart–double-inverted-pendulum on mock hardware (plumbing proof).
For physics stabilization, drive the same artifact through
[Gazebo Fortress](../gazebo.md) (headless, in CI) or
[Isaac Sim](../isaac.md) (local GPU).
