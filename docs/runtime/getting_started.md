# Getting started

## Build

In a ROS 2 Humble workspace containing this repo and `kontrolem_example_robots`:

```bash
colcon build --packages-up-to kontrolem_controllers
source install/setup.bash
```

## Run the LQR demo (mock hardware)

```bash
ros2 launch lqr_controller lqr_demo.launch.py
```

The controller manager comes up on `mock_components/GenericSystem`, spawns the
joint-state broadcaster, then activates `lqr_controller`, which loads the
exported artifact and starts writing effort commands. This is a plumbing proof
— mock hardware has no dynamics, so nothing is *stabilized* here. For real
physics see [Gazebo Fortress physics validation](gazebo.md) (headless, runs in
CI) or [Isaac Sim physics validation](isaac.md) (local GPU).

## How a controller finds its artifact

Every controller takes an `artifact_path` parameter. It accepts a plain
filesystem path or a `package://<pkg>/<relative/path>` URI resolved against the
ament index at configure time — prefer the latter in installed configs so the
YAML is independent of the install prefix:

```yaml
lqr_controller:
  ros__parameters:
    artifact_path: package://lqr_controller/config/lqr_cart_double_pendulum_ros2_control.yaml
```

## Standalone vs chained

Each controller is a `ChainableControllerInterface`:

- **Standalone** — it reads its setpoint from the `~/reference` topic
  (`std_msgs/Float64MultiArray`, one value per reference interface, absolute
  units) and writes effort commands to hardware. Defaults regulate to the
  operating point.
- **Chained** — an upstream controller writes the exported per-output reference
  interfaces (`<controller>/<joint>/position`, `.../velocity`). The cascade is
  declared entirely in the controller manager YAML with a stock upstream
  controller; no custom code. See [Chaining](controllers/index.md#chaining).

## Validity guard

The linearized gains are only trusted near the operating point. Each controller
enforces a per-joint bound on `|q − q_eq|` (and optionally `|q̇|`); leaving the
region triggers a configurable safe action (`hold_u_eq`, `zero_command`, or
`deactivate`). Bound precedence: per-joint parameter > artifact `validity_region`
> global `validity.q_dev_max_default`.

## Tests

```bash
colcon test --packages-select \
  kontrolem_controllers_base lqr_controller lqg_controller hinf_controller
colcon test-result --verbose
```

The suite covers the artifact loader, the numeric cores (incl. Python↔C++
equivalence traces), the validity guard, an allocation-free realtime check, and
per-controller load + functional + mock-hardware launch tests (plus an LQR
cascade launch test).
