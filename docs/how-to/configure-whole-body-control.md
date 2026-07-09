# How to configure a floating-base whole-body controller

> **For:** an integrator with a legged/floating robot. **Assumes:** you've read [Run on your own robot](run-on-your-robot.md) and understand `<ros2_control>` tags. **Goal:** stand a floating-base robot under the WBC.

A floating base adds two things ordinary joints don't have: an **SE(3) base pose/twist** and **foot contacts**. Kontrol'Em carries both through ros2_control as scalar `<gpio>` interfaces, and reassembles them controller-side. Use the bundled quadruped (`floating_quadruped.ros2_control.urdf`, `quad_stand_controllers.yaml`) as the reference example.

## 1. Declare base + contact interfaces in the URDF

In the `<ros2_control>` block, in addition to the leg joints, add two `<gpio>` blocks and point the hardware at the contact sim:

```xml
<ros2_control name="myquad_sim" type="system">
  <hardware>
    <plugin>kontrolem_description/FloatingContactSimSystem</plugin>
    <param name="contact_frames">foot_FL,foot_FR,foot_RL,foot_RR</param>
    <param name="base_height">0.2753</param>   <!-- nominal stance base z -->
  </hardware>

  <!-- each actuated leg joint: effort command + pos/vel state (initial = nominal posture) -->
  <joint name="hipy_FL">
    <command_interface name="effort"/>
    <state_interface name="position"><param name="initial_value">0.7</param></state_interface>
    <state_interface name="velocity"><param name="initial_value">0.0</param></state_interface>
  </joint>
  <!-- ...the other 11 joints... -->

  <!-- SE(3) base pose (7) + twist (6), as scalar interfaces -->
  <gpio name="floating_base">
    <state_interface name="pose.position.x"/> ... <state_interface name="twist.angular.z"/>
  </gpio>

  <!-- one contact scalar per foot -->
  <gpio name="contact">
    <state_interface name="contact.foot_FL"/>
    <state_interface name="contact.foot_FR"/>
    <state_interface name="contact.foot_RL"/>
    <state_interface name="contact.foot_RR"/>
  </gpio>
</ros2_control>
```

The exact 13 base interface names and the naming convention are in [Reference → ros2_control interfaces](../reference/ros2control-interfaces.md). Copy the full block from `floating_quadruped.ros2_control.urdf`.

## 2. Configure the WBC in YAML

```yaml
kontrolem_controller:
  ros__parameters:
    control_law: wbc
    base_type: floating          # <-- selects the floating-base path
    command_interface: effort
    actuated_joints: [hipx_FL, hipy_FL, knee_FL, hipx_FR, hipy_FR, knee_FR,
                      hipx_RL, hipy_RL, knee_RL, hipx_RR, hipy_RR, knee_RR]
    contact_frames: [foot_FL, foot_FR, foot_RL, foot_RR]
    base_gpio: floating_base
    contact_gpio: contact

    wbc.base_height: 0.2753
    wbc.nominal_posture: [0.0, 0.7, -1.4,  0.0, 0.7, -1.4,
                          0.0, 0.7, -1.4,  0.0, 0.7, -1.4]  # matches actuated_joints order
    wbc.kp_base: 100.0
    wbc.kd_base: 20.0
    wbc.mu: 0.7
    wbc.tau_max: 40.0
```

Critical consistency rules:
- `base_type: floating` **must** be set, or the runtime treats the robot as fixed-base.
- `wbc.nominal_posture` is listed in the **same order** as `actuated_joints`.
- `wbc.base_height` and the joint `initial_value`s in the URDF must describe the **same** standing pose the sim's `base_height` uses, so the robot starts already standing.
- `contact_frames` names must be frames that exist in the URDF (the foot links).

## 3. Launch and verify

Use `build_sim_launch` (as in [Run on your own robot](run-on-your-robot.md)) pointing at your quad URDF and YAML. Then confirm the WBC took command and is healthy:

```bash
ros2 control list_controllers                                  # kontrolem_controller -> active
ros2 topic echo --once /kontrolem_controller/diagnostics       # control_law: wbc, ok: true, margin > 0
```

You should see the launch log print `controller took command — releasing the plant`.

> **Scope:** this configures *standing* (all feet in contact, held by the sim as ground truth). Locomotion — gait, stepping, contact switching — is **not** part of this version. Why, and what the contact signal means, is in [Explanation → State on a manifold & non-joint data](../explanation/state-and-non-joint-data.md) and [Design decisions](../explanation/design-decisions.md).
