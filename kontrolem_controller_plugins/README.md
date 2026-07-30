# kontrolem_controller_plugins

The pluginlib discovery bridge for Kontrol'Em control laws (M16).

## Purpose

Expose the shipped control-law factories from `kontrolem_controllers` as
**pluginlib plugins** under the `kontrolem_control::ControllerFactory` base class,
so the L4 runtime (`kontrolem_ros2_control`) can discover and instantiate a law
**by name at load time** instead of through a hardcoded `if/else`. This package
is deliberately tiny: it contains only the `PLUGINLIB_EXPORT_CLASS` macros and
the plugin-description XML — no controller logic.

Its reason to exist is the ROS-free invariant: `kontrolem_control` and
`kontrolem_controllers` must build with **no ROS on the path** (the layer-boundary
litmus). `pluginlib` is ROS infrastructure, so the export lives here, in a
separate L4 package, rather than polluting the ROS-free core.

## Dependencies and build instructions

Depends on `pluginlib`, `kontrolem_control` (the `ControllerFactory` base +
`ParameterSpec`/`ParameterMap` types), `kontrolem_controllers` (the concrete
factories), and `kontrolem_model`.

Build from the workspace root:

```bash
cd /media/rahgirrafi/Disk1/Technical/ws_kontrolem
colcon build --packages-up-to kontrolem_controller_plugins \
  --cmake-args -DCMAKE_PREFIX_PATH="$HOME/.local/lib/python3.10/site-packages/cmeel.prefix"
```

At runtime the cmeel Pinocchio libs must be on `LD_LIBRARY_PATH`
(`$HOME/.local/lib/python3.10/site-packages/cmeel.prefix/lib`); the bringup
launch files inject this automatically.

## Relation to other packages

- **`kontrolem_controllers`** — provides the ROS-free `ControllerFactory`
  implementations (`LqrFactory`, `LqgFactory`, `LpvFactory`, `MpcFactory`,
  `QpFactory`, `WbcFactory`, `KinematicGaitFactory`). This package only
  *exports* them.
- **`kontrolem_control`** — owns the `ControllerFactory` base class and the
  config-as-data types; the plugins register under it (base package
  `kontrolem_control`).
- **`kontrolem_ros2_control`** — the consumer: it constructs a
  `pluginlib::ClassLoader<kontrolem_control::ControllerFactory>` and looks a law
  up by its `name()` (e.g. `control_law: wbc`).

## Position in the complete architecture

Layer 4 (runtime/binding), alongside `kontrolem_ros2_control`. It is the
discovery seam that keeps Layers 1–3 (`kontrolem_model`, `kontrolem_problem`
concepts, `kontrolem_control`, `kontrolem_controllers`) free of any middleware
dependency while still allowing runtime, name-based, open registration of control
laws.

## Intended use

Loaded implicitly by `kontrolem_ros2_control` — you do not run it directly. A
**third-party controller** follows this package as a template: implement a
`ControllerFactory` in your own package and export it with `PLUGINLIB_EXPORT_CLASS`
+ a `plugins.xml` registered under `kontrolem_control`. Your law then becomes
selectable via `control_law: <your_name>` with **zero edits** to the runtime.

## How to use it

Nothing to invoke here. Confirm the plugins are visible after building and
sourcing the workspace:

```bash
source install/setup.bash
ros2 pkg xml kontrolem_controller_plugins           # metadata
# The factories are then discoverable by kontrolem_ros2_control at controller
# load time; select one in your controllers YAML:
#   kontrolem_controller:
#     ros__parameters:
#       control_law: wbc     # resolved via ControllerFactory::name()
```

See `docs/how-to/add-a-controller.md` for the full "ship your own controller"
walkthrough.
