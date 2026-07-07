# Gazebo Fortress physics validation

The colcon tests validate the controllers on `mock_components` hardware — they
prove the artifact loads, the interfaces bind, the law computes the right
command, and the chain wires up. They do **not** prove the controller
*stabilizes* anything, because mock hardware has no dynamics.

The `kontrolem_gazebo` package closes that gap with **real physics**: it brings
up the cart–double-inverted-pendulum in **Gazebo Fortress** (Ignition Gazebo 6)
and lets an exported artifact balance it. Unlike the [Isaac Sim](isaac.md) path,
this runs **headless in CI** — no GPU required — so it doubles as an automated
stabilization test.

## Architecture

```
ign gazebo (physics @ 1 kHz, sim time)
  └─ gz_ros2_control/GazeboSimSystem   (hardware plugin in the gazebo URDF)
        └─ controller_manager  ← runs INSIDE the sim process, on sim time
              ├─ joint_state_broadcaster
              └─ lqr | lqg | hinf controller → effort commands
  /clock ──(ros_gz_bridge)──▶ ROS 2 side (use_sim_time)
```

The `gz_ros2_control` system plugin embeds the controller manager **inside the
Gazebo process** on sim time, so there is no separate `ros2_control_node`. The
Gazebo URDF variant swaps the hardware plugin `mock_components/GenericSystem` →
`gz_ros2_control/GazeboSimSystem`; the controller config and the exported
artifact are byte-for-byte the same ones the mock-hardware demos use.

## Run it

```bash
colcon build --packages-up-to kontrolem_gazebo
source install/setup.bash

# LQR (default), headless server:
ros2 launch kontrolem_gazebo gz_bringup.launch.py

# with the Gazebo GUI:
ros2 launch kontrolem_gazebo gz_bringup.launch.py gui:=true

# LQG / H-infinity from the same launch:
ros2 launch kontrolem_gazebo gz_bringup.launch.py controller:=lqg
ros2 launch kontrolem_gazebo gz_bringup.launch.py controller:=hinf
```

Watch it hold with `ros2 topic echo /joint_states` — both pendulum links
(`joint1`, `joint2`) are **passive**, so they can only stay upright if the lone
actuated `cart_joint` is actively stabilizing the whole pole.

## Test

```bash
colcon test --packages-select kontrolem_gazebo
colcon test-result --verbose
```

The launch test runs Gazebo headless, waits for the controller to load its
exported artifact, then samples `/joint_states` for several seconds and asserts
every joint stays inside the linearization validity region (`|q| < 0.3`) — i.e.
the plant is being stabilized, not toppling. All three controllers (LQR, LQG,
H∞) stabilize the pendulum from their exported bundles.

## Notes / gotchas

These were hard-won; they live in the package `README.md` too.

- **Fortress binary.** The launch calls `ign gazebo` (Ignition 6). On systems
  where only `gz sim` exists, adjust the launch file.
- **System-plugin path.** Fortress locates *system* plugins via
  `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` (not `LD_LIBRARY_PATH`), and sourcing ROS does
  not set it, so `libgz_ros2_control-system.so` (in `/opt/ros/$ROS_DISTRO/lib`)
  is not found by default. The launch injects it via `additional_env` on the
  `ign gazebo` process — a top-level `SetEnvironmentVariable` does **not**
  reliably reach that subprocess.
- **Effort command interface.** The actuated joint (`cart_joint`) exposes an
  `effort` command interface; `gz_ros2_control` applies it directly, matching the
  controllers' `command_interface: effort`.
- **Dynamic compensators.** As everywhere, LQG/H∞ set `update_rate` explicitly
  equal to `controller_manager.update_rate` (the Tustin discretization rate).
- **Log-scraping the activation.** A launch test keys on the controller's own
  `on_configure` log line (`loaded <method> artifact …`), not the ros2_control
  spawner's `Configured and activated <name>` banner: the spawner hardcodes ANSI
  color around the controller name, which splits that substring, and rcutils
  logs to **stderr** rather than stdout.
