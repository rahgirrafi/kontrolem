# Reference — Demos & launch files

> **For:** everyone. **Assumes:** a built, sourced workspace. **Scope:** every launch file in `kontrolem_bringup`, the robot and law it runs, its config, and what it demonstrates. Launch with `ros2 launch kontrolem_bringup <file>`.

| Launch file | Robot | Law | Config YAML | Demonstrates |
|---|---|---|---|---|
| `cart_pole.launch.py` | cart-pole | `lqr` | `cart_pole_controllers.yaml` | LQR balances an underactuated pendulum. The first-run demo. |
| `cart_pole_lqg.launch.py` | cart-pole | `lqg` | `cart_pole_lqg_controllers.yaml` | Output-feedback: balances from **positions only**, estimating velocity. |
| `cart_pole_tracking.launch.py` | cart-pole | `lqr` (Tracking) | `cart_pole_tracking_controllers.yaml` | Cart follows a moving `0.3·cos(0.5t)` reference while balancing. |
| `cart_pole_mpc.launch.py` | cart-pole | `mpc` | `cart_pole_mpc_controllers.yaml` | Receding-horizon QP with a hard torque limit. |
| `cart_pole_mpc_tracking.launch.py` | cart-pole | `mpc` (Tracking) | `cart_pole_mpc_tracking_controllers.yaml` | MPC tracks the moving reference using the future horizon (tighter than LQR). |
| `cart_double_pole.launch.py` | cart + double pole | `lqr` | `cart_double_pole_controllers.yaml` | Hard 3-DoF benchmark: one actuator, two passive poles (open-loop very unstable). |
| `arm2.launch.py` | 2-DoF arm | `qp` | `arm2_controllers.yaml` | Online inverse-dynamics QP regulating a fully-actuated arm with an active torque limit. |
| `quad_stand.launch.py` | floating quadruped | `wbc` | `quad_stand_controllers.yaml` | Whole-body QP stands a floating-base quadruped; base + contacts via `<gpio>`. |
| `quad_push.launch.py` | floating quadruped | `wbc` | `quad_stand_controllers.yaml` | Same WBC standing, but the sim delivers a scheduled external base push (~2 s in); the WBC catches it and returns the base to nominal. |
| `floating_spike.launch.py` | floating biped | *(probe, not a law)* | `floating_spike_controllers.yaml` | Development spike: a read-only controller reassembles SE(3) base state from scalar interfaces as the base free-falls. |

## Shared launch body

All demos except `floating_spike` are thin wrappers over `build_sim_launch(urdf_file, controllers_yaml)` in `kontrolem_bringup/launch/_common.py`. It starts `robot_state_publisher`, `ros2_control_node` (with the URDF for both the resource manager and the controller), `joint_state_broadcaster`, then the control law once the broadcaster is up — and injects the cmeel library path so plugins load. Adding a demo is a two-line wrapper; see [How-to → Run on your own robot](../how-to/run-on-your-robot.md).

## URDFs

Launch URDFs live in `kontrolem_description/urdf/*.ros2_control.urdf` (body + `<ros2_control>` block). The corresponding pure-body URDFs used by the model/tests are in `kontrolem_model/robots/`.
