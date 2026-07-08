# kontrolem_model

Layer-1 **model service** for Kontrol'Em v2 — a Pinocchio-backed rigid-body
model, queryable by the controller plugins.

## Purpose

Provide the *single source of truth* for robot dynamics as a **queryable
service**, not a stored snapshot. A frozen `(A, B, C, D)` is one evaluation of
the model, not the model itself — so this package wraps [Pinocchio][pin] and
exposes dynamics *queries* that different controller paradigms ask in different
ways.

This is the **minimal first slice**, built to test the v2 interface hypothesis.
It currently answers exactly one query:

```cpp
Linearization RobotModel::linearize(q, v, tau) const;   // -> { A (2nv×2nv), B (2nv×nv) }
```

the continuous-time Jacobians of `ẋ = f(x, τ)` with state `x = [q; v]` and input
`τ` the generalized joint torque, computed from Pinocchio's analytic
`computeABADerivatives` (not finite differences). `aba(q, v, τ)` (forward
dynamics) is also exposed so the correctness test can finite-difference it.

Deliberately **absent** for now (added as later milestones earn them): floating
base / SE(3) state, `rollout` + `linearize_along` (MPC), and
`dynamics` + `contact_jacobians` (WBC).

## Dependencies and build instructions

**Dependencies**
- `ament_cmake` — build tooling only (this is *not* a ROS runtime dependency).
- `Eigen3` (≥ 3.3) — the only dependency in the public header.
- `pinocchio` (rigid-body dynamics) — used **only** in `robot_model.cpp`,
  linked `PRIVATE` behind a pImpl, so it is invisible to downstream code at both
  compile and link time.

> **No ROS client libraries.** There is no `rclcpp` / `ros2_control` / message
> dependency here. `ament_cmake`/`colcon` are used for *uniform build tooling*
> across the workspace; the ROS boundary lives one layer up (the runtime
> package), never in the core.

**Getting Pinocchio.** Two options:
- *ROS-native (recommended, flag-free):* `sudo apt install ros-humble-pinocchio`.
- *pip/cmeel (no sudo):* if only the pip `pin` wheel is present, point CMake at
  its bundled C++ prefix (`~/.local/.../cmeel.prefix`) — see below.

**Build (colcon, from the workspace root):**
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select kontrolem_model            # if ros-humble-pinocchio installed

# ...or, using the pip/cmeel Pinocchio (no apt install):
P=$HOME/.local/lib/python3.10/site-packages/cmeel.prefix
colcon build --packages-select kontrolem_model \
  --cmake-args -DCMAKE_PREFIX_PATH="$P"
```

**Test** (analytic linearization vs. central finite difference):
```bash
colcon test --packages-select kontrolem_model --event-handlers console_direct+
# with the cmeel Pinocchio, also: export LD_LIBRARY_PATH="$P/lib:$P/lib64:$LD_LIBRARY_PATH"
```
The test passes when `max|A − A_fd|` and `max|B − B_fd|` are below `1e-5`
(observed ≈ `1e-10`).

## Relation to other packages

- **Depended on by** the controller plugins (`kontrolem_control` and the
  concrete controllers): they call `linearize()` at synthesis time (LQR) or the
  dynamics queries every tick (QP-WBC).
- **Independent of** `kontrolem_problem` (the problem-spec types) and of the ROS
  runtime package — the model neither knows what a "problem" is nor how it is
  deployed.
- **Actuation selection is not here.** `B` is returned w.r.t. the *full*
  generalized torque; which joints are actuated (`S`) is a controller/problem
  concern, keeping this layer paradigm-neutral.

## Position in the complete architecture

Layer 1 (the spine) of the four-layer v2 design:

```
  Layer 4  runtime (ros2_control) + supervisor      [ROS]
  Layer 3  controller plugins (LQR, QP-WBC, ...)     ─┐ query
  Layer 2  problem spec (Regulation, TaskSpec, ...)   │
  Layer 1  kontrolem_model  ── dynamics service ◀─────┘   ← THIS PACKAGE
```

Everything above queries this service; linear "artifacts" are cached
evaluations of it, never the source of truth.

## Intended use

For **controller authors and the synthesis tooling** to obtain dynamics
quantities about an operating point or state, without touching Pinocchio or ROS
directly. Built by colcon for workspace uniformity, but usable/testable fully
off-robot (no ROS runtime required to link or run).

## How to use it

```cpp
#include "kontrolem_model/robot_model.hpp"
using kontrolem_model::RobotModel;

const RobotModel model = RobotModel::from_urdf_file("/path/to/robot.urdf");

Eigen::VectorXd q(model.nq()), v(model.nv()), tau(model.nv());
q << /* ... */;  v << /* ... */;  tau << /* ... */;

const auto lin = model.linearize(q, v, tau);   // lin.A (2nv×2nv), lin.B (2nv×nv)
// e.g. an LQR plugin then selects actuated columns of B and solves CARE.
```

A ready-to-use `robots/cart_pole.urdf` (2-DoF fixed-base cart-pole) ships with
the package and is installed to `share/kontrolem_model/robots/`.

[pin]: https://github.com/stack-of-tasks/pinocchio
