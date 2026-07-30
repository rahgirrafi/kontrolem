# Add your own controller

**Goal:** ship a new control law as **its own package** and select it with
`control_law: <your name>` — with **zero edits** to the Kontrol'Em framework.

Since M16 the runtime discovers control laws through a **pluginlib registry**, and
each law declares its own parameters as **data** (a `ParameterSpec`), so the
framework never hardcodes your law's name or its knobs. The worked template is the
`kontrolem_controller_example` package; copy it.

## The three things your package provides

1. **A `Controller`** — implement the ROS-free contract
   (`capabilities → synthesize → configure → compute → status`).
2. **A `ControllerFactory`** — `name()` (your `control_law:` value),
   `parameter_spec()` (your parameters + defaults, as data), and `create()`
   (build the controller from a `ParameterMap`).
3. **A pluginlib export** — one macro + a `plugins.xml` registered under the
   `kontrolem_control` base package.

## Step 1 — the controller

```cpp
class MyController : public kontrolem_control::Controller {
  kontrolem_control::Capabilities capabilities() const override {
    return {{kontrolem_control::Dialect::kRegulation}, /*needs_velocity_state=*/true};
  }
  std::unique_ptr<Synthesis> synthesize(const RobotModel&, const ControlProblem&) const override {
    return std::make_unique<Synthesis>();          // online law -> nothing to precompute
  }
  void configure(const RobotModel& m, const Synthesis&, const ControlProblem&) override { /* ... */ }
  const Command& compute(const State& s, const ControlProblem& p, double dt) override { /* ... */ }
  const Status& status() const override { return status_; }
};
```

## Step 2 — the factory (name + schema + build)

The `parameter_spec()` is the **single source of truth** for your parameters. The
runtime declares them for you, and a typo in a deployment YAML is **rejected at
load** because the key is not in your spec.

```cpp
class MyFactory : public kontrolem_control::ControllerFactory {
  std::string name() const override { return "mylaw"; }         // -> control_law: mylaw
  kontrolem_control::ParameterSpec parameter_spec() const override {
    return {
      {"mylaw.kp", kontrolem_control::ParamValue{80.0}, "position stiffness"},
      {"mylaw.kd", kontrolem_control::ParamValue{16.0}, "velocity damping"},
    };
  }
  std::unique_ptr<Controller> create(const ParameterMap& p, const RobotModel& m,
                                     const BuildContext& ctx) const override {
    return std::make_unique<MyController>(ctx.actuated_joints,
                                          p.double_at("mylaw.kp"), p.double_at("mylaw.kd"));
  }
};
```

`BuildContext` carries the structural, non-tunable inputs (actuated joints,
contact frames) — things that come from the robot description, not your tuning.

A `ParamValue` may be a `double`, `int64_t`, `bool`, `std::string`, or an array of
doubles, ints, or strings. The active alternative **is** the parameter's type, so
the runtime declares it correctly without being told.

!!! tip "Parameters that aren't flat"
    If your law is configured by a *list of records* rather than loose scalars,
    express it as **parallel arrays** — one entry per record — and assemble them in
    `create()`, where you can also validate them. The shipped `lpv` law does exactly
    this for its scheduling grid (`lpv.sched_joints` / `sched_min` / `sched_max` /
    `sched_nodes`), and rejects a length mismatch or an unknown joint name there, so
    the error surfaces at load with a clear message instead of as a wrong gain at the
    first tick. Prefer naming things (joints, frames) over raw indices: `create()`
    receives the `RobotModel` and can resolve names itself.

## Step 3 — export it

```cpp
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(my_pkg::MyFactory, kontrolem_control::ControllerFactory)
```

`plugins.xml` (registered under `kontrolem_control`, not your own package):

```xml
<library path="my_pkg">
  <class type="my_pkg::MyFactory" base_class_type="kontrolem_control::ControllerFactory">
    <description>My control law.</description>
  </class>
</library>
```

`CMakeLists.txt`:

```cmake
find_package(pluginlib REQUIRED)
find_package(kontrolem_control REQUIRED)
find_package(kontrolem_model REQUIRED)
add_library(my_pkg SHARED src/my_controller.cpp)
target_link_libraries(my_pkg PUBLIC
  kontrolem_control::kontrolem_control kontrolem_model::kontrolem_model)
ament_target_dependencies(my_pkg PUBLIC pluginlib)
pluginlib_export_plugin_description_file(kontrolem_control plugins.xml)   # <-- base pkg
```

## Step 4 — use it

Select your law in a controllers YAML — nothing in the framework changes:

```yaml
kontrolem_controller:
  ros__parameters:
    control_law: mylaw
    actuated_joints: ["shoulder_joint", "elbow_joint"]
    mylaw.kp: 100.0
    mylaw.kd: 20.0
```

```bash
colcon build --packages-up-to my_pkg
source install/setup.bash
ros2 launch kontrolem_bringup <your>.launch.py
```

The runtime finds `mylaw` via `ControllerFactory::name()`, declares
`mylaw.kp`/`mylaw.kd` from your spec, and rejects any misspelled `mylaw.*` key at
load with a message listing the valid names.

!!! tip "Why config-as-data"
    Because your parameters are a `ParameterSpec` (plain data, no ROS), the same
    schema can be validated, documented, and exercised **without a middleware** —
    the `kontrolem_controller_example` package's `test_example_discoverable` builds
    and runs the law through the registry off-launch. That is what keeps
    *configuration* middleware-isolated, not just the control loop.

## The layering rule (why there are sometimes two packages)

The **framework's own** laws keep their factories in the ROS-free
`kontrolem_controllers` and put the pluginlib export in a separate bridge package
(`kontrolem_controller_plugins`) so the core builds with no ROS on the path. **You
do not need that split** — a third-party package puts the controller, factory, and
export together, exactly like `kontrolem_controller_example`.
