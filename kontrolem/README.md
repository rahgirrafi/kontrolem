# Kontrol'Em v2

A robot-agnostic, plugin-based control framework for ROS 2 — *"MoveIt for
control"* — hosting many modern-control paradigms behind **one interface**.
Classical linear feedback (LQR), output-feedback compensators (LQG), online
optimization (QP task-space), and predictive control (MPC) are all the *same
shape* to the runtime: they differ only in what happens inside `compute()`.

The bet, and the thing this codebase demonstrates end-to-end: a single contract

```cpp
compute(const State&, const ControlProblem&, double dt) -> const Command&
```

behind a multi-phase lifecycle (`synthesize` → `configure` → `compute` →
`status`) genuinely fits controllers that are structurally nothing alike.

## What runs today

Four controller paradigms × two problem dialects, all end-to-end through
ros2_control, all allocation-free on the real-time path:

| Paradigm | Structure | `compute()` does | Dialects |
|---|---|---|---|
| **LQR** | static state gain | one gemv | Regulation, Tracking |
| **LQG** | dynamic output-feedback compensator (internal observer) | step observer + gemv; **positions only** | Regulation |
| **QP task-space** | stateless online solve | build + solve an inverse-dynamics QP (OSQP), torque limit active | Regulation |
| **MPC** | constrained receding horizon | condense + warm-solve a horizon QP, apply `u₀` | Regulation, Tracking |

*One runtime hosts all four* (chosen by a `control_law` param); *one controller
(LQR/MPC) serves both a fixed setpoint and a moving reference*. MPC tracks a
moving reference ~6× tighter than feedback-only LQR by using the future horizon.

## Architecture (4 layers)

```
  Layer 4  kontrolem_ros2_control   ros2_control host + minimal supervisor + telemetry   [ROS]
  Layer 3  kontrolem_control (the contract)  +  kontrolem_controllers (LQR/LQG/QP/MPC)
  Layer 2  problem spec: Regulation, Tracking (TrajectorySource)
  Layer 1  kontrolem_model   dynamics service over Pinocchio (fixed + floating base)
```

**Invariant:** Layers 1–3 are plain C++ + Eigen with **zero ROS client-library
dependencies** — unit-testable off-robot, reusable, and structurally unable to
couple to the middleware. ROS lives only in Layer 4.

## Packages

| Package | Layer | Role |
|---|---|---|
| `kontrolem_model` | 1 | `RobotModel`: `linearize` / `dynamics` / `center_of_mass` / contact Jacobians / SE(3) `integrate`, over Pinocchio (behind a pImpl) |
| `kontrolem_control` | 3 | the `Controller` contract + capability-typed `ControlProblem` (`Regulation`/`Tracking`) + plain `State`/`Command`/`Status` |
| `kontrolem_controllers` | 3 | the four control laws + `care.hpp` (CARE/DARE) + the OSQP `QpSolver` seam |
| `kontrolem_ros2_control` | 4 | `KontrolemController` (hosts any law), capability-driven interface claiming, supervisor, telemetry; + M3 floating-base spike |
| `kontrolem_msgs` | — | `ControllerDiagnostics` telemetry message |
| `kontrolem_description` | — | example robots (cart-pole, 2-DoF arm, floating biped) + self-contained ABA sim hardware (no Gazebo needed) |
| `kontrolem_bringup` | — | `controller_manager` configs + launch files for every demo |

## Build

Pinocchio comes from the pip **cmeel** wheel (no apt/sudo). From this directory:

```bash
source /opt/ros/humble/setup.bash
P=$HOME/.local/lib/python3.10/site-packages/cmeel.prefix   # cmeel Pinocchio prefix
colcon build --cmake-args -DCMAKE_PREFIX_PATH="$P" -DCMAKE_BUILD_TYPE=Release
```

> The bringup launch files auto-inject the cmeel libs on `LD_LIBRARY_PATH`, so
> `ros2 launch` "just works" after `source install/setup.bash` — no manual export.

## Run the demos

```bash
source install/setup.bash

# LQR balances the cart-pole
ros2 launch kontrolem_bringup cart_pole.launch.py
# LQG balances it from positions ONLY (velocity estimated)
ros2 launch kontrolem_bringup cart_pole_lqg.launch.py
# LQR follows a moving reference (Tracking dialect)
ros2 launch kontrolem_bringup cart_pole_tracking.launch.py
# QP task-space regulates a 2-DoF arm (torque limit active)
ros2 launch kontrolem_bringup arm2.launch.py
# MPC balances / follows a reference (constrained, receding-horizon)
ros2 launch kontrolem_bringup cart_pole_mpc.launch.py
ros2 launch kontrolem_bringup cart_pole_mpc_tracking.launch.py
```

Watch it: `ros2 topic echo /joint_states`, `ros2 control list_controllers`, and
(with `publish_diagnostics: true`) `ros2 topic echo /kontrolem_controller/diagnostics`.

## Tests

```bash
export LD_LIBRARY_PATH="$P/lib:$P/lib64:$LD_LIBRARY_PATH"
colcon test --packages-select kontrolem_model kontrolem_control kontrolem_controllers
colcon test-result --all
```

12 dependency-free ctests: model finite-difference (linearization, contact
Jacobian), floating-base known-quantities (CoM, SE(3) integrate), the contract
lifecycle, closed-loop stability/tracking for LQR/LQG/QP/MPC, and allocation +
malloc audits proving `compute()` is allocation-free on every online path.

## Status & design record

Milestones **M0–M2 complete** (fixed-base LQR/LQG/QP/MPC, Tracking); **M3**
(floating base) has its model layer done and validated, and the ros2_control
non-joint-state plumbing proven by a spike. See **`DEVELOPMENT.md`** for the
living step-by-step log (decisions, findings, and what's next), and
`refactored-skipping-parnas.md` for the architecture plan.
