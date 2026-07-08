# kontrolem_controllers

Layer-3 **controller implementations** for Kontrol'Em v2. In this interface-test
slice it holds the two deliberately-different controllers that validate the
`kontrolem_control` contract.

## Purpose

Prove that one plugin contract (`compute(state, problem, dt) → command`) fits
**three structurally different** control laws:
- **`LqrController`** — a *precomputed static gain* (no internal state).
  `synthesize` linearizes at the operating point and solves CARE offline;
  `compute` is a matrix–vector product.
- **`QpTaskSpaceController`** — a *stateless online solve*: builds and solves an
  inverse-dynamics QP every tick with an active torque limit.
- **`LqgController`** — a *dynamic output-feedback compensator* (internal
  observer state). Control gain from the control CARE + Kalman gain from the
  **dual/filter CARE** (separation principle); measures positions only, estimates
  velocity. Proves `compute()` fits a controller *with memory*.

Also provides `care.hpp`, a small Eigen-only CARE solver — reused for both the
control Riccati and (transposed) the filter Riccati.

## Dependencies and build instructions

**Dependencies**
- `ament_cmake` — build tooling only.
- `Eigen3` — linear algebra (incl. `EigenSolver` for the CARE).
- `kontrolem_control` — the controller contract these implement.
- `kontrolem_model` — queried for `linearize` (LQR synthesis) and
  `gravity_torque` (operating-point `u_eq`).

> No ROS client libraries. The QP solver (OSQP/ProxQP) will be added behind a
> seam in the next step; it is a numeric dependency, not ROS.

**Build & test** (from the workspace root):
```bash
source /opt/ros/humble/setup.bash
P=$HOME/.local/lib/python3.10/site-packages/cmeel.prefix   # if using pip/cmeel Pinocchio
colcon build --packages-select kontrolem_model kontrolem_control kontrolem_controllers \
  --cmake-args -DCMAKE_PREFIX_PATH="$P"
export LD_LIBRARY_PATH="$P/lib:$P/lib64:$LD_LIBRARY_PATH"
colcon test --packages-select kontrolem_controllers --event-handlers console_direct+
```
`test_lqr_cartpole` gates on the **closed loop `A − B_actK` being stable** (all
poles with negative real part) — the LQR analog of the model's FD test — plus a
finite command and a working validity-region flag.

## Relation to other packages

- **Implements** the `Controller` interface from `kontrolem_control`.
- **Queries** `kontrolem_model` (dynamics/linearization/gravity).
- **Depended on by** the Layer-4 runtime (later): the runtime loads a
  `Controller` from here and drives it. This package itself is ROS-free.
- Future paradigm controllers (MPC, WBC) will live in *their own* packages
  depending only on `kontrolem_control`, not on this one.

## Position in the complete architecture

```
  Layer 4  runtime (ros2_control) + supervisor        [ROS]
  Layer 3  kontrolem_control (contract)
           kontrolem_controllers  ── LQR (+ QP next) ◀── ← THIS PACKAGE
  Layer 2  problem spec (Regulation)
  Layer 1  kontrolem_model (dynamics service)
```

## Intended use

For deploying a concrete control law on a robot: construct the controller with
its design params, `synthesize` (offline) → `configure` (at load) → `compute`
(each tick). The runtime, not the controller, decides what to do with a
non-`ok` `status()`.

## How to use it

```cpp
#include "kontrolem_controllers/lqr_controller.hpp"
using namespace kontrolem_control;
using namespace kontrolem_controllers;

Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(4, 4);  Q(1,1) = 10.0;
Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);
LqrController lqr({"cart_joint"}, Q, R, /*q_dev_max=*/0.5);

Regulation upright;                       // setpoint = operating point
upright.q_ref = Eigen::VectorXd::Zero(2);
upright.v_ref = Eigen::VectorXd::Zero(2);

auto artifact = lqr.synthesize(model, upright);   // CARE, offline
lqr.configure(model, *artifact, upright);
const Command& u = lqr.compute(state, upright, dt);   // gemv, per tick
```
