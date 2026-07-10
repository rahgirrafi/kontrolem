# kontrolem_state_bridge

A ros2_control **SensorInterface** hardware component, `OdometryBaseBridge`, that
turns a base **odometry topic** (`nav_msgs/Odometry`) into the 13 floating-base
scalar **state interfaces** on a `<gpio>` block — the "topic → interface" bridge
that Part B4 of the v2 plan names as the one localized ugliness of the
ros2_control-native design.

## Purpose

`ros2_control` has no floating-base concept, and Gazebo's `IgnitionSystem` (like a
real robot's `SystemInterface`) only exports **joints** — the base pose/twist is
not a joint. This package is how that non-joint base state enters the RT loop as
ordinary, claimable state interfaces:

- subscribes to a base odometry topic (Gazebo ground-truth odometry in M6.3; a
  real contact-aided InEKF estimator later),
- re-exports it as the 13 scalars `floating_base/pose.position.{x,y,z}`,
  `.pose.orientation.{x,y,z,w}`, `.twist.linear.{x,y,z}`, `.twist.angular.{x,y,z}`
  — the exact naming/order that `kontrolem_ros2_control`'s `BaseStateSensor`
  claims and reassembles into a manifold-correct `State`.

The controller side therefore never learns whether the base came from our custom
sim, Gazebo, or hardware — only the producer swaps.

The pose/twist mapping itself lives in the ROS-free header `odom_to_base.hpp`
(`base_from_odom`), where the whole M6.3 risk — quaternion order and the
pose-in-world / twist-in-body framing that matches Pinocchio's free-flyer — is
unit-tested off-robot (`test/test_odom_to_base.cpp`).

## Dependencies and build instructions

**Dependencies:** `hardware_interface`, `pluginlib`, `rclcpp`, `nav_msgs`,
`realtime_tools` (all ROS). No Pinocchio/Eigen — the mapping is plain scalars.

**Build** (from the v2 workspace root):
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select kontrolem_state_bridge
colcon test --packages-select kontrolem_state_bridge   # runs the offline mapping test
```

## Relation to other packages

- **`kontrolem_ros2_control`** — provides `BaseStateSensor`, which claims and
  reassembles the 13 interfaces this bridge exports. The naming convention is
  shared between the two (this bridge produces exactly what that sensor consumes).
- **`kontrolem_description`** — its `FloatingBaseSimSystem` is the *custom-sim*
  producer of the same 13 interfaces; this bridge is the *external-producer*
  counterpart, so the same controller runs against either without change.
- **`kontrolem_model`** — not a build dep here, but the free-flyer `q(0..6)` /
  `v(0..5)` convention this bridge targets is Pinocchio's, enforced downstream.

## Position in the complete architecture

Layer 4 (ros2_control runtime), Part B4's non-joint-state story. It is a hardware
component, not a controller: it sits beside `IgnitionSystem`/`SystemInterface` in
the resource manager and only exports state. It realizes plan source #1 (Gazebo
ground truth) and #3 (external estimator) for base state; source #2 (a robot's own
IMU/FT) would be exported by that robot's `SystemInterface` directly.

## Intended use

Compose it with the joint-bearing hardware in one robot's `<ros2_control>`: the
joint component (Gazebo `IgnitionSystem` or a real `SystemInterface`) carries the
actuated joints, and a `<ros2_control type="sensor">` block using this plugin,
with a single `<gpio name="floating_base">` of 13 state interfaces, carries the
base. A floating-base controller (e.g. the WBC, or `FloatingStateProbe`) then
claims joints + the base gpio and assembles a manifold `State`.

## How to use it

In the robot URDF (`<param name="topic">` selects the odometry topic; default
`/base_odom`):
```xml
<ros2_control name="base_state" type="sensor">
  <hardware>
    <plugin>kontrolem_state_bridge/OdometryBaseBridge</plugin>
    <param name="topic">/base_odom</param>
  </hardware>
  <gpio name="floating_base">
    <state_interface name="pose.position.x"/>
    <state_interface name="pose.position.y"/>
    <state_interface name="pose.position.z"/>
    <state_interface name="pose.orientation.x"/>
    <state_interface name="pose.orientation.y"/>
    <state_interface name="pose.orientation.z"/>
    <state_interface name="pose.orientation.w"/>
    <state_interface name="twist.linear.x"/>
    <state_interface name="twist.linear.y"/>
    <state_interface name="twist.linear.z"/>
    <state_interface name="twist.angular.x"/>
    <state_interface name="twist.angular.y"/>
    <state_interface name="twist.angular.z"/>
  </gpio>
</ros2_control>
```
Feed the topic from Gazebo with the `odometry-publisher` system + `ros_gz_bridge`
(`ignition.msgs.Odometry` → `nav_msgs/msg/Odometry`); its twist is body-frame, so
no rotation is needed. **Frame contract:** pose in world/odom, twist in the base
(child) frame.
