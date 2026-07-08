# kontrolem_bringup

Launch + `controller_manager` configuration that runs the M0 cart-pole loop
end-to-end, no Gazebo.

## Purpose

Tie the pieces together: start a `controller_manager` with the
`kontrolem_description` simulation hardware, spawn a `joint_state_broadcaster`,
and bring up the `kontrolem_ros2_control` controller (LQR by default) so the
cart-pole balances in-process. This is the M0 acceptance demo.

## Dependencies and build instructions

**Dependencies (runtime):** `controller_manager`, `joint_state_broadcaster`,
`robot_state_publisher`, `kontrolem_description`, `kontrolem_ros2_control`.

It is a pure config/launch package (`ament_cmake`, no compiled code):
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select kontrolem_bringup
```

## Relation to other packages

- **Loads** `kontrolem_description` (URDF + sim hardware) into `controller_manager`.
- **Spawns** the `kontrolem_ros2_control` controller and configures it via
  `config/cart_pole_controllers.yaml`.
- Depends on nothing in the ROS-free core directly — it wires ROS-level pieces.

## Position in the complete architecture

```
  kontrolem_bringup  ← THIS (launch + controller_manager config)
      ├── kontrolem_description        (sim hardware + URDF)
      └── kontrolem_ros2_control       (hosts LQR/QP core controllers)
```

## Intended use

The single entry point to see Kontrol'Em run. `config/cart_pole_controllers.yaml`
holds the `controller_manager` update rate and the controller parameters (law
choice, actuated joints, setpoint, LQR/QP gains). The URDF is injected at launch
time as the controller's `robot_description` parameter (kept out of the YAML).

## How to use it

### 1. Build the whole workspace (once)

The core links Pinocchio from the pip **cmeel** wheel (see `../DEVELOPMENT.md`
D1), so the build needs its prefix on `CMAKE_PREFIX_PATH`:

```bash
source /opt/ros/humble/setup.bash
P=$HOME/.local/lib/python3.10/site-packages/cmeel.prefix   # cmeel Pinocchio prefix
cd /media/rahgirrafi/Disk1/Technical/ws_kontrolem/src/kontrolem_v2
colcon build --cmake-args -DCMAKE_PREFIX_PATH="$P" -DCMAKE_BUILD_TYPE=Release
```

### 2. Run the cart-pole loop

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash          # from the v2 workspace root
ros2 launch kontrolem_bringup cart_pole.launch.py
```

> **No manual `LD_LIBRARY_PATH` needed** — the launch file auto-detects the cmeel
> Pinocchio libs and injects them for the launched nodes (otherwise
> `ros2_control_node` can't `dlopen` the plugins, and you'd see a
> `LibraryLoadException: libboost_serialization.so...: cannot open shared object
> file`). If you run the node *by hand* (not via this launch), add the libs
> yourself: `export LD_LIBRARY_PATH="$P/lib:$P/lib64:$LD_LIBRARY_PATH"`.

### 3. Watch it balance (a second terminal)

```bash
source /opt/ros/humble/setup.bash
cd /media/rahgirrafi/Disk1/Technical/ws_kontrolem/src/kontrolem_v2 && source install/setup.bash

ros2 control list_controllers      # kontrolem_controller -> active
ros2 topic echo /joint_states      # pole_joint position settles to ~0
```

What you should see: the pole is **held at 0.15 rad** for the first second or two
(the sim plant is frozen until a controller takes command — the D11 fix), then it
settles smoothly to upright (~0) while the cart returns toward centre.

### 4. Run the QP task-space law on the 2-DoF arm

The QP task-space controller's honest home is a **fully-actuated** robot (it can't
stabilize the underactuated cart-pole). The arm demo shows the *same runtime*
hosting a structurally different, online-solved law — just a different launch:

```bash
ros2 launch kontrolem_bringup arm2.launch.py
```
```bash
# second terminal
ros2 topic echo /joint_states   # shoulder_joint & elbow_joint settle to ~0
```
The arm starts folded (`[1.0, -1.0]` rad) and the QP **regulates it to horizontal
`[0, 0]` by inverse dynamics** — computing the joint torques that cancel gravity +
inertia — while respecting the `qp.tau_max` torque limit (the limit is *active*
during the large-error transient, enforced inside the QP, not clamped post-hoc).
Config: `config/arm2_controllers.yaml` (`control_law: qp`).

### 5. Run the LQG output-feedback compensator (M1)

Same cart-pole, same runtime — but a *dynamic* controller that measures **only
positions** and estimates the velocities with an internal Kalman filter:

```bash
ros2 launch kontrolem_bringup cart_pole_lqg.launch.py
```
The pole balances from 0.15 rad using position measurements alone. Config:
`config/cart_pole_lqg_controllers.yaml` (`control_law: lqg`).

### 6. Cart following a moving reference (Tracking dialect, M1)

Same cart-pole, same LQR gain law — but the *problem* is a time-varying reference
instead of a fixed setpoint: the cart follows `0.3·cos(0.5 t)` while balancing:

```bash
ros2 launch kontrolem_bringup cart_pole_tracking.launch.py
```
Watch `cart_joint` oscillate ~±0.3 m while `pole_joint` stays near 0. Config:
`config/cart_pole_tracking_controllers.yaml` (`reference_type: harmonic`). This
shows ONE controller accepting two problem dialects (Regulation + Tracking).

### 7. Linear MPC (M2)

Constrained receding-horizon control — solves a QP over an N-step horizon each
tick and applies the first input, honouring a hard torque limit *inside* the
optimization:

```bash
ros2 launch kontrolem_bringup cart_pole_mpc.launch.py           # MPC regulates (balances)
ros2 launch kontrolem_bringup cart_pole_mpc_tracking.launch.py  # MPC follows a moving reference
```
The tracking variant uses the *future* reference over the horizon, so it tracks
much tighter than feedback-only LQR. Configs: `config/cart_pole_mpc*.yaml`.

## End-to-end smoke test

The unit tests validate the ROS-free core; they do **not** cover the ros2_control
runtime integration. `test/e2e_smoke.sh` closes that gap: it launches a demo,
watches a joint settle, and reports PASS/FAIL (hard timeouts, self-cleaning). It
is a standalone script, not a `colcon` launch-test, because full ros2_control
launches flake under some CI/harness setups and a hanging test is worse than a
script you run on purpose.

```bash
source install/setup.bash
bash kontrolem_bringup/test/e2e_all.sh          # all demos (LQR/LGG/QP, 3 robots)
# or one demo:
bash kontrolem_bringup/test/e2e_smoke.sh cart_double_pole.launch.py pole1_joint 0.05
```

> All demos share one launch body — `launch/_common.py` (`build_sim_launch`);
> the `*.launch.py` files differ only in URDF + controller YAML. That one runtime
> hosts LQR (static gain), QP (online solve), LGG (dynamic compensator), **and**
> MPC (receding-horizon), and one controller serves both the Regulation and
> Tracking dialects, is the whole point of the v2 contract.
