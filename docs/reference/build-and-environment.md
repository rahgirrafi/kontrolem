# Reference — Build & environment

> **For:** integrators and developers. **Assumes:** ROS 2 / colcon basics. **Scope:** exact build commands, the package list, dependencies, and the cmeel/Pinocchio environment facts. Troubleshooting the common load error is in [How-to → Fix cmeel errors](../how-to/fix-cmeel-library-errors.md).

## Prerequisites

| Dependency | Provides | Install |
|---|---|---|
| ROS 2 Humble | `ros2_control`, `controller_manager`, message tooling | apt (`/opt/ros/humble`) |
| colcon | build tool | `apt install python3-colcon-common-extensions` |
| Pinocchio (cmeel wheel) | rigid-body dynamics for `kontrolem_model` | `pip3 install pin` |
| Eigen 3.3+ | linear algebra (headers) | apt / bundled |
| OSQP | QP backend behind the solver seam | found via CMake |

The cmeel wheel installs Pinocchio (and its own bundled Boost) under:

```
$HOME/.local/lib/python3.10/site-packages/cmeel.prefix
```

with `lib/`, `lib64/`, `include/`, and `lib/cmake/` inside. This path is referenced throughout as `$P`.

## Build

```bash
source /opt/ros/humble/setup.bash
P=$HOME/.local/lib/python3.10/site-packages/cmeel.prefix

cd <workspace>                    # the directory containing the package folders
colcon build --cmake-args -DCMAKE_PREFIX_PATH="$P" -DCMAKE_BUILD_TYPE=Release
```

`-DCMAKE_PREFIX_PATH="$P"` lets CMake find the cmeel Pinocchio. A CMake **Boost version warning** during configure is harmless.

To build a subset:

```bash
colcon build --packages-select kontrolem_model kontrolem_controllers \
  --cmake-args -DCMAKE_PREFIX_PATH="$P" -DCMAKE_BUILD_TYPE=Release
```

## Run

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash         # from the workspace you built in
ros2 launch kontrolem_bringup cart_pole.launch.py
```

The launch files add the cmeel libraries to `LD_LIBRARY_PATH` automatically. To run a node or test binary **by hand**, export it yourself:

```bash
export LD_LIBRARY_PATH="$P/lib:$P/lib64:$LD_LIBRARY_PATH"
```

## Packages

| Package | Layer | Type | Purpose |
|---|---|---|---|
| `kontrolem_model` | L1 | C++ lib | `RobotModel` dynamics service over Pinocchio. |
| `kontrolem_control` | L2/L3 | C++ lib | Problem dialects, `TrajectorySource`, the `Controller` contract, `State`/`Command`/`Status`, `Synthesis`. |
| `kontrolem_controllers` | L3 | C++ lib | Control-law implementations: LQR, LQG, MPC, QP task-space, WBC + the OSQP `QpSolver` seam. |
| `kontrolem_ros2_control` | L4 | ros2_control plugin | `KontrolemController`, the semantic components, supervisor enforcement, telemetry. |
| `kontrolem_msgs` | — | msgs | `ControllerDiagnostics`. |
| `kontrolem_description` | — | share + plugins | Example URDFs and the self-contained sim hardware plugins. |
| `kontrolem_bringup` | — | launch/config | Launch files and `controller_manager` YAML. |

Layers 1–3 (`kontrolem_model`, `kontrolem_control`, `kontrolem_controllers`) carry **no** ROS client-library dependency; the ROS boundary is `kontrolem_ros2_control` only. Rationale: [Explanation → Architecture](../explanation/architecture.md).

## Build artifacts & ROS version notes

- `build/`, `install/`, `log/` are created in the workspace (git-ignored).
- ROS 2 Humble is `ros2_control` 2.x; the full URDF reaches a controller via its own `robot_description` parameter (injected by the launch), because `get_robot_description()` is not in Humble's controller base.
