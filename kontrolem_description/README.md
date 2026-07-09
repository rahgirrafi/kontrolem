# kontrolem_description

Example robot description (cart-pole) **and** a self-contained ros2_control
simulation hardware, `CartPoleSimSystem`.

## Purpose

Provide everything needed to run a Kontrol'Em controller against a *simulated*
robot without a full physics engine:

- `urdf/cart_pole.ros2_control.urdf` — the cart-pole (same robot as
  `kontrolem_model/robots/cart_pole.urdf`) with a `<ros2_control>` block. Only
  `cart_joint` exports an `effort` command interface; `pole_joint` is state-only,
  so underactuation is expressed structurally. (LQR demo.)
- `urdf/arm2.ros2_control.urdf` — a 2-DoF **fully-actuated** planar arm (both
  joints take effort), the honest end-to-end home for the QP task-space
  controller. Same sim plugin, no code change — it just satisfies the generic
  `nq == nv` fixed-base contract. (QP demo.)
- `CartPoleSimSystem` — a `hardware_interface::SystemInterface` that integrates
  the URDF's **true rigid-body dynamics** using `kontrolem_model`'s ABA (forward
  dynamics) with symplectic Euler. It reads the effort command, advances the
  state, and reports position/velocity — closing the loop entirely in-process.

This lets us watch the pole balance end-to-end (`controller → command → sim →
state → controller`) before introducing Gazebo.

## Dependencies and build instructions

**Dependencies:** `hardware_interface`, `pluginlib`, `rclcpp` (ROS), `Eigen3`,
and `kontrolem_model` (the ROS-free dynamics service — Pinocchio stays hidden
behind its pImpl).

**Build** (from the v2 workspace root, with the cmeel Pinocchio prefix):
```bash
source /opt/ros/humble/setup.bash
P=$HOME/.local/lib/python3.10/site-packages/cmeel.prefix
colcon build --packages-select kontrolem_description --cmake-args -DCMAKE_PREFIX_PATH="$P"
```

## Relation to other packages

- **Uses** `kontrolem_model` for forward dynamics (ABA) — the same model service
  the controllers query, so the sim and the controller agree on the physics.
- **Consumed by** `kontrolem_bringup`, which loads this URDF into
  `controller_manager` and runs a `kontrolem_ros2_control` controller against it.
- It is a *robot/sim* package: it does not depend on the control core contract or
  the controllers.

## Position in the complete architecture

```
  Layer 4  kontrolem_ros2_control (controller)  ◀── loop ──▶  kontrolem_description (sim hardware)  ← THIS
  Layer 1  kontrolem_model (dynamics service, shared by both sides of the loop)
```

The sim hardware is the *plant*; the controller is the *policy*. Both are wired
together by `controller_manager` (configured in `kontrolem_bringup`).

## Intended use

Loaded by a `controller_manager` as the hardware component. As a `<system>` it
exports `cart_joint/effort` (command) and `<joint>/position`,`<joint>/velocity`
(state) for both joints. Initial joint values come from the URDF
`<state_interface>` `initial_value` params (the pole starts 0.15 rad off upright).

`CartPoleSimSystem` is generic over any fixed-base **Euclidean** URDF (nq == nv);
it rejects floating-base / continuous joints (those need the manifold-aware
integrator planned for M3).

## How to use it

Don't launch it directly — use `kontrolem_bringup`:
```bash
ros2 launch kontrolem_bringup cart_pole.launch.py
```
To reuse the sim for another fixed-base robot, point a `<ros2_control>` block at
`kontrolem_description/CartPoleSimSystem`, give each actuated joint an `effort`
command interface, and set `initial_value` params as desired.
