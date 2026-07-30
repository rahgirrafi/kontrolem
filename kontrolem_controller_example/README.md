# kontrolem_controller_example

A worked, out-of-tree Kontrol'Em control law — the template for shipping your own
controller (M16).

## Purpose

Prove — and show how — a **third-party control law drops into Kontrol'Em with
zero edits to the framework**. This package implements the
`kontrolem_control::Controller` contract, ships its own `ControllerFactory`,
exports it via pluginlib, and becomes selectable as `control_law: example`. The
runtime discovers it by name and configures it from its own parameter schema
(`example.*`) — `kontrolem_ros2_control` and `kontrolem_controllers` are never
touched.

The law itself is intentionally trivial (gravity compensation + joint-space PD to
a `Regulation` setpoint, enough to hold the fully-actuated 2-DoF arm against
gravity). It exists to demonstrate the plugin seam, not to be production control.

## Dependencies and build instructions

Depends only on `pluginlib`, `kontrolem_control` (the ROS-free contract +
`ControllerFactory`/`ParameterSpec`) and `kontrolem_model`. Build from the
workspace root:

```bash
cd /media/rahgirrafi/Disk1/Technical/ws_kontrolem
colcon build --packages-up-to kontrolem_controller_example \
  --cmake-args -DCMAKE_PREFIX_PATH="$HOME/.local/lib/python3.10/site-packages/cmeel.prefix"
source install/setup.bash
```

## Relation to other packages

- **`kontrolem_control`** — provides the `Controller` contract and the
  `ControllerFactory`/`ParameterSpec`/`ParameterMap` types this package
  implements and registers under.
- **`kontrolem_model`** — queried at runtime (gravity torque, joint indices).
- **`kontrolem_ros2_control`** — the consumer; its `build_law()` discovers this
  plugin via a `pluginlib::ClassLoader<kontrolem_control::ControllerFactory>`.
- **`kontrolem_controller_plugins`** — the framework's own bridge does the same
  export for the built-in laws; this package is the third-party analog (but here
  the controller, factory, and export live together — a third party has no reason
  to split them).

## Position in the complete architecture

Layer 3 + its own discovery. It sits *outside* the framework packages entirely,
which is the point: the registry (M16) lets control laws live anywhere, discovered
at runtime, so the set of paradigms is open rather than baked into the runtime.

## Intended use

Copy this package to author a new controller: implement `Controller`, write a
`ControllerFactory` (its `name()`, `parameter_spec()`, and `create()`), add the
`PLUGINLIB_EXPORT_CLASS` line and a `plugins.xml` registered under
`kontrolem_control`, then select it with `control_law: <your name>`.

## How to use it

Run it on the 2-DoF arm (same runtime as the QP demo, different law):

```bash
source install/setup.bash
ros2 launch kontrolem_bringup example_arm.launch.py
# or the automated check (the arm settles to the horizontal setpoint):
bash src/kontrolem_bringup/test/e2e_smoke.sh example_arm.launch.py shoulder_joint 0.05
```

Discovery is proven off-launch by the package's own test (workspace sourced):

```bash
source install/setup.bash
./build/kontrolem_controller_example/test_example_discoverable
```
