# kontrolem_gz

A Gazebo-native ros2_control hardware component, `GzBaseStateSystem`, that exports a
floating base's ground-truth pose/twist **directly from Gazebo's entity-component
manager (ECM)** — plus constant all-stance contact scalars — as the `floating_base`/
`contact` `<gpio>` state interfaces a whole-body controller claims.

## Purpose

The in-gz `controller_manager` booted by `ign_ros2_control` only loads hardware that
implements `gz_ros2_control::GazeboSimSystemInterface` — so the topic-based
`kontrolem_state_bridge` (M6.3) **cannot** be composed with `IgnitionSystem` there.
This package is the gz-native counterpart: `initSim()` hands it the ECM, and every
`read()` copies `base_link`'s world pose and world twist (rotated to the body frame,
matching Pinocchio's free-flyer convention verified in M6.3) into the 13
`floating_base/*` scalars, with no topic round-trip — fresh base state every tick at
the WBC's 500 Hz. Contact is a constant all-stance (=1) per foot: the same
ground-truth assumption `FloatingContactSimSystem` makes, which keeps the M6.2 test
about the WBC's forces meeting Gazebo's real contact solver, not about contact
detection.

## Dependencies and build instructions

**Dependencies:** `gz_ros2_control` (the `GazeboSimSystemInterface` base),
`hardware_interface`, `pluginlib`, `rclcpp`, `ignition-gazebo6` (ECM, `Link`/`Model`).
This package deliberately isolates the heavy Gazebo build deps from the rest of the
workspace.

**Build** (from the v2 workspace root):
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select kontrolem_gz
```

## Relation to other packages

- **`kontrolem_ros2_control`** — its `BaseStateSensor`/`ContactSensor` claim and
  reassemble exactly the interfaces this component exports; the naming convention is
  shared (this produces what those consume).
- **`kontrolem_state_bridge`** — the *out-of-gz* counterpart (topic → interfaces) for
  standalone controller_managers; this package is the *in-gz* one. Same 13-scalar
  layout, different transport.
- **`kontrolem_description`** — `FloatingContactSimSystem` is the custom-sim producer
  of the same interfaces; the M6.2 demo swaps it for `IgnitionSystem` (joints) + this
  component (base/contact) with zero controller change.

## Position in the complete architecture

Layer 4 (ros2_control runtime), Part B4's non-joint-state story, source #1 (sim
ground truth) — realized inside gz-sim. It sits beside `IgnitionSystem` in the in-gz
resource manager: `IgnitionSystem` owns the 12 actuated joints, this component owns
the base + contact `<gpio>` blocks. It exports **state only** (no commands).

## Intended use

Compose two `<ros2_control>` blocks in the robot's gz URDF: one `type="system"` with
`ign_ros2_control/IgnitionSystem` carrying the joints, and one with
`kontrolem_gz/GzBaseStateSystem` carrying the base + contact gpios. The
`ign_ros2_control` `<gazebo>` plugin boots a controller_manager that loads both; a
floating-base controller (the WBC) then claims joints + base + contacts exactly as it
does against the custom sim.

## How to use it

```xml
<ros2_control name="base_state" type="system">
  <hardware>
    <plugin>kontrolem_gz/GzBaseStateSystem</plugin>
    <param name="model_name">floating_quadruped</param>
    <param name="base_link">base_link</param>
  </hardware>
  <gpio name="floating_base">
    <!-- 13 scalars: pose.position.{x,y,z}, pose.orientation.{x,y,z,w},
         twist.linear.{x,y,z}, twist.angular.{x,y,z} -->
  </gpio>
  <gpio name="contact">
    <!-- one scalar per foot, e.g. contact.foot_FL ... -->
  </gpio>
</ros2_control>
```
See `kontrolem_description/urdf/floating_quadruped_gz.urdf` for the full working
example and `kontrolem_bringup/test/e2e_quad_gz.sh` for the M6.2 validation
(stance + push recovery on Gazebo's real contact). **Frame contract:** pose in
world, twist in the base (body) frame — Pinocchio's free-flyer `q(0..6)/v(0..5)`.
