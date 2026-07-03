# state_space_response_viz — RViz playback

Source-agnostic playback of {doc}`RobotTrajectory <../trajectory_format>`
files into RViz: animate a URDF from a simulated (or recorded) motion,
with play/pause/seek/speed transport control. The package consumes only
the trajectory format — it does not care whether the file came from the
linear simulator, a nonlinear simulator, or a log.

```bash
ros2 launch state_space_response_viz view_response.launch.py \
    trajectory:=/path/to/trajectory.npz \
    urdf:=/path/to/robot.urdf \
    [fixed_frame:=base_link] [speed:=1.0] [loop:=true]
```

Transport at runtime:

```bash
ros2 service call /response_player/pause std_srvs/srv/Trigger
ros2 service call /response_player/play  std_srvs/srv/Trigger
ros2 service call /response_player/reset std_srvs/srv/Trigger
ros2 param set /response_player speed 0.5      # dynamic, 0.01–10
ros2 param set /response_player seek 1.25      # jump to t = 1.25 s
```

## How it works

The player node is pure composition — the clock × sampler × renderer
model from {doc}`../architecture`:

- `PlaybackClock` maps monotonic wall time to simulation time with
  anchor rebasing (speed changes and seeks never jump; timer jitter can't
  desynchronize playback).
- `FrameSampler` (from `state_space_control.trajectory`) interpolates the
  trajectory at that time.
- `RVizRenderer` publishes the sampled frame as `sensor_msgs/JointState`
  for **every** joint in the trajectory (omitting unactuated joints would
  leave stale TF), plus a world→base TF when the trajectory carries a
  base pose. Those transport details are hidden behind the
  `TrajectoryRenderer` interface — to render elsewhere, implement
  `setup(traj)`/`render(frame)` and add the instance to the player's
  renderer list (see {doc}`../extending`).

## ROS interface

| Name | Type | Notes |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | published at `publish_rate` (60 Hz default) |
| `/response_viz/time` | `std_msgs/Float64` | current sim time — an external sync cursor |
| `/response_viz/status` | `std_msgs/String` (JSON) | `{state, speed, t, duration, trajectory}` at 2 Hz |
| `/response_viz/events` | `std_msgs/String` (JSON) | each trajectory event as the cursor crosses it |
| `~/play`, `~/pause`, `~/reset` | `std_srvs/Trigger` | transport |
| `trajectory`, `publish_rate`, `speed`, `seek`, `loop`, `autoplay`, `base_frame`, `world_frame` | parameters | `speed`/`seek`/`loop` dynamically reconfigurable |

No custom message/service types anywhere — the package stays pure
`ament_python` (which cannot generate interfaces); everything rides on
`std_msgs`/`std_srvs`/`sensor_msgs` plus dynamic parameters.

## Future work

Side-by-side controller comparison (two players under namespaces with
`frame_prefix`-ed robot_state_publishers, slaved to one
`/response_viz/time`), video export (a renderer driven by a synthetic
fixed-step clock), Foxglove/Unity renderers, a rosbag→RobotTrajectory
importer. All fit the existing format and interfaces without redesign.

## API reference

{doc}`../api/state_space_response_viz`
