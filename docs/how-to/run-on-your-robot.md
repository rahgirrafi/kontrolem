# How to run the framework on your own robot

> **For:** an integrator with a robot and its URDF. **Assumes:** a built, sourced workspace, familiarity with ROS 2 `<ros2_control>` tags, and that you've run a demo. **Goal:** bring up a Kontrol'Em controller on your own fixed-base robot.

This guide covers a **fixed-base** robot (arm, cart, etc.). For a floating/legged robot, do this first, then see [Configure a whole-body controller](configure-whole-body-control.md).

You need three artifacts, mirroring any bundled demo: a URDF with a `<ros2_control>` block, a controller YAML, and a launch file.

## 1. Add a `<ros2_control>` block to your URDF

For each joint the controller will read and command, declare its interfaces. To run *in the built-in simulator* (no Gazebo), point the hardware at the Kontrol'Em sim plugin:

```xml
<ros2_control name="myrobot_sim" type="system">
  <hardware>
    <plugin>kontrolem_description/CartPoleSimSystem</plugin>
  </hardware>
  <joint name="joint_a">
    <command_interface name="effort"/>
    <state_interface name="position"><param name="initial_value">0.3</param></state_interface>
    <state_interface name="velocity"><param name="initial_value">0.0</param></state_interface>
  </joint>
  <!-- repeat per joint -->
</ros2_control>
```

`CartPoleSimSystem` integrates the URDF's true rigid-body dynamics for any fixed-base robot with Euclidean joints (`nq == nv`). On real hardware, replace it with your robot's own `SystemInterface`. See [Reference → ros2_control interfaces](../reference/ros2control-interfaces.md) for the available plugins.

## 2. Write a controller YAML

Under `kontrolem_bringup/config/myrobot_controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 200
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    kontrolem_controller:
      type: kontrolem_ros2_control/KontrolemController

kontrolem_controller:
  ros__parameters:
    control_law: lqr
    actuated_joints: ["joint_a", "joint_b"]   # the joints YOU command
    command_interface: effort
    q_ref: [0.0, 0.0]                          # setpoint, length nq
    v_ref: [0.0, 0.0]                          # length nv
    lqr.q_diag: [1.0, 10.0, 1.0, 1.0]
    lqr.r_diag: [0.1, 0.1]
    lqr.q_dev_max: 0.5
```

Key points:
- `actuated_joints` lists only the joints the controller drives (an underactuated robot has fewer of these than total joints).
- The URDF is **not** pasted into this file — the launch injects it as `kontrolem_controller.robot_description`.
- Pick the law and its gains as in [Switch the control law](switch-control-law.md) and [Tune a controller](tune-a-controller.md).

## 3. Write a launch file

Reuse the shared launch body — a demo launch is a two-line wrapper:

```python
# kontrolem_bringup/launch/myrobot.launch.py
import os, sys
sys.path.insert(0, os.path.dirname(__file__))
from _common import build_sim_launch

def generate_launch_description():
    return build_sim_launch(
        urdf_file="myrobot.ros2_control.urdf",   # in kontrolem_description/urdf/
        controllers_yaml="myrobot_controllers.yaml",
    )
```

Put the URDF in `kontrolem_description/urdf/`.

## 4. Build and run

```bash
colcon build --packages-select kontrolem_description kontrolem_bringup
source install/setup.bash
ros2 launch kontrolem_bringup myrobot.launch.py
```

Verify as in [Verify an integration](verify-an-integration.md), and inspect health with [Read diagnostics](read-diagnostics.md).

> If the controller fails to load a plugin with a `libboost_serialization…` error, see [Fix the cmeel / libboost load error](fix-cmeel-library-errors.md).
