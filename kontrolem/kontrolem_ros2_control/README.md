# kontrolem_ros2_control

Layer-4 **runtime boundary** for Kontrol'Em v2: a ros2_control controller that
hosts any `kontrolem_control::Controller` and connects it to real hardware or a
simulator.

## Purpose

This is the single package that bridges ROS and the ROS-free control core. The
`KontrolemController` (a `controller_interface::ControllerInterface`):

- claims joint **state interfaces** per the law's `capabilities()` — position
  always, velocity **only if** `needs_velocity_state` (an output-feedback law like
  LQG claims positions only and estimates velocity internally) — and assembles a
  `kontrolem_control::State`,
- builds the **problem** from params: a fixed setpoint (`Regulation`) or a moving
  reference (`Tracking` + a `TrajectorySource`, e.g. `reference_type: harmonic`),
- calls the hosted law's `compute(state, problem, dt)`,
- writes the resulting torque to the actuated joints' **command interfaces**,
- runs a **minimal supervisor**: it trusts the law's `status()` and applies a
  safe action (currently: zero command) when the status is not ok.

Which law runs (LQR / LQG / QP task-space) is chosen by the `control_law`
parameter, so one runtime serves every controller paradigm — the whole point of
the v2 contract.

## Dependencies and build instructions

**Dependencies**
- ROS/ros2_control: `controller_interface`, `hardware_interface`, `pluginlib`,
  `rclcpp`, `rclcpp_lifecycle`. *(This package is Layer 4 — ROS lives here, by
  design; it does not below.)*
- Core: `kontrolem_control`, `kontrolem_model`, `kontrolem_controllers`.

**Build** (from the v2 workspace root):
```bash
source /opt/ros/humble/setup.bash
P=$HOME/.local/lib/python3.10/site-packages/cmeel.prefix   # if using cmeel Pinocchio
colcon build --cmake-args -DCMAKE_PREFIX_PATH="$P"
```

## Relation to other packages

- **Hosts** `kontrolem_controllers` laws through the `kontrolem_control` contract.
- **Queries** `kontrolem_model` (URDF → model, passed to the law).
- **Consumed by** `kontrolem_bringup` (launch + controller_manager config) and a
  robot/sim `kontrolem_description`.
- It is the *only* v2 package with a ROS client-library dependency.

## Position in the complete architecture

```
  Layer 4  kontrolem_ros2_control  ── ros2_control host + supervisor ◀── ← THIS
  Layer 3  kontrolem_control (contract) + kontrolem_controllers (LQR, QP)
  Layer 2  problem spec (Regulation)
  Layer 1  kontrolem_model (dynamics service)
```

## Intended use

Loaded by a `controller_manager` alongside a hardware/sim component. Parameters
configure the law and setpoint; ros2_control drives `update()` at the manager's
rate.

## How to use it (parameters)

```yaml
kontrolem_controller:
  ros__parameters:
    control_law: lqr            # or "qp"
    robot_description: "<the URDF XML string>"
    actuated_joints: ["cart_joint"]
    command_interface: effort
    q_ref: [0.0, 0.0]           # setpoint (defaults to zeros / upright)
    v_ref: [0.0, 0.0]
    safe_action: zero
    # law-specific:
    lqr.q_diag: [1.0, 10.0, 1.0, 1.0]
    lqr.r_diag: [1.0]
    lqr.q_dev_max: 0.5
    qp.task_weight: [1.0, 10.0]
    qp.kp: 50.0
    qp.kd: 10.0
    qp.tau_max: 5.0
```

> Status: the runtime boundary is in place and building; end-to-end bring-up
> (a `<ros2_control>` description + controller_manager launch + sim) is the next
> M0 step. See `../DEVELOPMENT.md`.
