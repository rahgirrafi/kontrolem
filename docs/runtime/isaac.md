# Isaac Sim physics validation

The colcon tests validate the controllers on `mock_components` hardware — they
prove the artifact loads, the interfaces bind, the law computes the right
command, and the chain wires up. They do **not** prove the controller
*stabilizes* anything, because mock hardware has no dynamics.

Physics stabilization — the same exported artifact balancing the real
(simulated) cart–double-inverted-pendulum — can run in NVIDIA Isaac Sim, locally
against a GPU. The assets live in the repo's `isaac/` directory and are
intentionally outside the colcon build and CI.

```{note}
For a headless, GPU-free physics check that also runs in CI, use
[Gazebo Fortress physics validation](gazebo.md). Isaac is the
higher-fidelity / photorealistic option when a GPU is available.
```

## Architecture

```
Isaac Sim (physics + articulation)
  ├─ publishes  /isaac_joint_states   (sensor_msgs/JointState)
  ├─ publishes  /clock                (use_sim_time)
  └─ subscribes /isaac_joint_commands (effort)
        ▲                    │
        │ topic_based_ros2_control (hardware plugin in the Isaac URDF variant)
        │                    ▼
controller_manager → lqr_controller → effort command
```

The URDF used on the ROS 2 side is the `topic_based_ros2_control` variant of
the robot (identical geometry, hardware plugin swapped), so the same controller
config and artifact drive both mock hardware and Isaac.

## Run it (two terminals)

**Terminal 1 — Isaac + bridge** (the wrapper sets the ROS 2 bridge
environment; do not source system ROS here):

```bash
cd .../kontrolem_controllers/isaac
./run_isaac_sim.sh            # add --headless to run without the GUI
```

Isaac runs Python 3.11 while system ROS 2 Humble is 3.10, so the bridge uses
Isaac's *bundled* Humble libraries — sourcing `/opt/ros/humble` here breaks
`import rclpy`. See `isaac/README.md` for the details.

**Terminal 2 — controller stack** (a sourced ROS 2 workspace):

```bash
ros2 launch .../kontrolem_controllers/isaac/lqr_isaac.launch.py
```

The pendulum holds upright; perturb it in the viewport and watch the cart
recover. Full prerequisites, gotchas (effort-only joints, physics rate ≥ 2×
controller rate, `use_sim_time`, topic-name matching), and the LQG/H∞ variants
are in `isaac/README.md`.
