# kontrolem_msgs

Kontrol'Em v2 ROS message definitions.

## Purpose

Public, named message contracts for the runtime. Currently one:
- **`ControllerDiagnostics`** — per-tick telemetry published by
  `KontrolemController`: `control_law`, the state (`q`, `v`) the law acted on,
  the `q_ref`, the applied `tau`, the supervisor trust (`ok`, `margin`,
  `safe_action`), and the `update_us` timing. Named fields make any hosted law
  inspectable via `ros2 topic echo` / rosbag — replacing the v1
  `Float64MultiArray`-layout approach.

## Dependencies and build instructions

`ament_cmake` + `rosidl_default_generators`; depends on `std_msgs`.
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select kontrolem_msgs
```

## Relation to other packages

- **Produced by** `kontrolem_ros2_control` (the runtime publishes it).
- **Consumed by** any observer (`ros2 topic echo`, rosbag, a future live monitor).
- Not depended on by the ROS-free core (Layers 1–3) — telemetry is a Layer-4
  concern; the core emits plain `Status`/`Command` structs that the runtime
  translates into this message.

## Position in the complete architecture

```
  Layer 4  kontrolem_ros2_control ── publishes ──▶ kontrolem_msgs/ControllerDiagnostics
  Layers 1–3  (ROS-free core: no dependency on this package)
```

## Intended use

Enable/inspect controller behaviour at runtime without instrumenting the core.

## How to use it

Set `publish_diagnostics: true` on `kontrolem_controller`, then:
```bash
ros2 topic echo /kontrolem_controller/diagnostics
```
