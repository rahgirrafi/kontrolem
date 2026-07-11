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
| `cart_pole_switch.launch.py` | cart-pole | `lqr`+`mpc` | `cart_pole_switch_controllers.yaml` | **Multi-controller Supervisor:** one runtime hosts LQR and MPC; a human switches between them live and bumplessly (see below). |
| `cart_pole_autofallback.launch.py` | cart-pole (+shove) | `lqr`→`mpc` | `cart_pole_autofallback_controllers.yaml` | **Automatic fail-forward:** a scheduled shove drives LQR out of its trust region; the Supervisor fails over to MPC on its **own** (no human command), which catches the pole (see below). |
| `cart_pole_autorecover.launch.py` | cart-pole (+shove) | `lqr`→`mpc`→`lqr` | `cart_pole_autorecover_controllers.yaml` | **Automatic recovery (round trip):** same shove and failover, but the Supervisor also switches **back** to LQR once it's trustworthy again — `lqr`→`mpc`→`lqr` with no human command and no chatter (see below). |
| `quad_stand.launch.py` | floating quadruped | `wbc` | `quad_stand_controllers.yaml` | Whole-body QP stands a floating-base quadruped; base + contacts via `<gpio>`. |
| `quad_push.launch.py` | floating quadruped | `wbc` | `quad_stand_controllers.yaml` | Same WBC standing, but the sim delivers a scheduled external base push (~2 s in); the WBC catches it and returns the base to nominal. |
| `floating_spike.launch.py` | floating biped | *(probe, not a law)* | `floating_spike_controllers.yaml` | Development spike: a read-only controller reassembles SE(3) base state from scalar interfaces as the base free-falls. |
| `cart_pole_gz.launch.py` | cart-pole | `lqr` | `cart_pole_controllers.yaml` | **Independent-physics validation:** the SAME LQR + config, but the plant is **Gazebo Fortress** (`ign_ros2_control`) instead of the custom sim — proof the substrate is swappable with zero controller change (see below). Needs Gazebo installed. |
| `floating_box_gz.launch.py` | floating body | *(probe, not a law)* | `floating_box_gz_controllers.yaml` | **Base-state from Gazebo:** a free body's ground-truth odometry is bridged into the 13 `floating_base` state interfaces (via `kontrolem_state_bridge`) and the probe reassembles a manifold State — non-joint base state from a non-custom producer, zero controller change (see below). Needs Gazebo installed. |
| `quad_gz.launch.py` | floating quadruped | `wbc` | `quad_stand_controllers.yaml` | **WBC vs real contact:** the SAME WBC + SAME config as `quad_stand`, but the plant is Gazebo's own contact/friction solver (not pinned feet) — stands and rejects a real base shove (see below). Needs Gazebo installed. |
| `go2_gz.launch.py` | **real Unitree Go2** | `wbc` | `go2_stand_controllers.yaml` | **WBC on a real robot model:** the SAME WBC, now retargeted to the actual Go2 (15 kg, real inertials) against Gazebo's contact solver — stands and rejects a 5000 N shove. Config + URDF only, zero controller-code change (see below). Needs Gazebo installed. |
| `go2_gz.launch.py base_source:=estimate` | **real Unitree Go2** | `wbc` + `base_estimator` | `go2_stand_controllers.yaml` | **WBC on its OWN state estimate (sim-to-real):** the same Go2 stands and rejects a 5000 N shove using a **floating-base state estimator** (IMU + leg odometry + contact) instead of Gazebo ground truth — no oracle in the control loop (see below). Needs Gazebo installed. |

## Switching controllers live (multi-controller Supervisor)

When a demo sets `control_laws` (a list) instead of `control_law`, one `KontrolemController` hosts several laws behind the **Supervisor** and drives the first. Command a switch by publishing the target law name:

```bash
ros2 topic pub -1 /kontrolem_controller/switch_controller std_msgs/msg/String "{data: mpc}"
ros2 topic echo /kontrolem_controller/diagnostics   # the control_law field flips to the new law
```

The handoff is **bumpless**: the incoming law is seeded from the current state (`Controller::on_activate`) and the command is blended from the outgoing to the incoming law over `switch_blend_ticks`. See [Explanation → Design decisions](../explanation/design-decisions.md) for the spike that established what a safe handoff requires.

## Automatic fail-forward (health-driven switching)

Setting `auto_fallback: true` lets the Supervisor switch **on its own**, without a human command. It watches the active law's `status().ok`: when the law reports not-ok for `fallback_dwell` consecutive ticks (the dwell debounces a single transient blip), the Supervisor hands off to the **next** law in `control_laws` — the primary, then its fallbacks in order.

The `cart_pole_autofallback` demo shows it end-to-end: LQR balances the pole with a modest trust region (`lqr.q_dev_max`), the sim shoves the pole a few seconds in (see the `disturb_*` params in `cart_pole_disturb.ros2_control.urdf`), that shove drives the state out of LQR's region so LQR reports not-ok, and the Supervisor fails over to MPC, which rides out the disturbance. Watch it happen:

```bash
ros2 launch kontrolem_bringup cart_pole_autofallback.launch.py
ros2 topic echo /kontrolem_controller/diagnostics   # control_law flips lqr->mpc with no command sent
```

One thing differs from a manual switch: an auto fail-forward switches **hard** (no command blend). The outgoing law has *lost trust*, so blending its (untrustworthy, possibly unbounded) command back in is exactly wrong — the incoming law takes over fully, reading the current state. Manual switching still works with `auto_fallback` enabled.

### Recovering to the primary on its own (`auto_recover`)

By default a fail-forward is one-way: the fallback keeps driving until a human switches back. Setting `auto_recover: true` closes the loop — the Supervisor returns to the **primary** (first law in `control_laws`) on its own once the primary is trustworthy again. This is the switch-*back* half of dwell-time/hysteresis switching.

The catch is knowing *when* the dormant primary is healthy: a controller only reports `status()` after it runs. So the Supervisor **shadow-evaluates** the primary each tick while the fallback drives — it runs the primary's `compute()` purely to read its `status()`, then discards the command. That is safe only for a **stateless** law (LQR/QP/MPC, whose output is a function of the current state alone); a state-carrying law (LQG's observer) would be corrupted by a throwaway run, so if the primary is stateful, recovery is skipped and a manual switch is needed. Two safeguards prevent flip-flopping: recovery fires only after the primary has been healthy for `recover_dwell` consecutive ticks (a long hysteresis dwell so a still-settling disturbance can't trigger a premature return), and — unlike a fail-forward — the switch-back **blends**, because the incoming primary is trusted.

The `cart_pole_autorecover` demo shows the full round trip: the shove trips LQR → the Supervisor fails over to MPC → MPC settles the pole → LQR becomes trustworthy again → the Supervisor hands back to LQR, all with no human command and no chatter:

```bash
ros2 launch kontrolem_bringup cart_pole_autorecover.launch.py
ros2 topic echo /kontrolem_controller/diagnostics   # control_law: lqr -> mpc -> lqr
```

## Independent-physics validation with Gazebo (`cart_pole_gz`)

Every other demo uses a custom `SystemInterface` sim (`CartPoleSimSystem`, `FloatingContactSimSystem`) that integrates the plant with the **same** Pinocchio model the controllers use — deterministic and dependency-light, but it "marks its own homework" (it can't catch model mismatch or real contact). `cart_pole_gz` closes that gap: the plant is **Gazebo Fortress**'s own physics engine.

Because the controllers only ever see ros2_control `State`/`Command` interfaces, the plant behind them is swappable with **zero controller change** — `cart_pole_gz` runs the *identical* `KontrolemController` (LQR) binary and the *identical* `cart_pole_controllers.yaml`; only the URDF's `<ros2_control>` hardware (`ign_ros2_control/IgnitionSystem`) and the launch/world files differ. Once LQR is active, the demo applies a one-shot torque disturbance to the pole (via Gazebo's `ApplyLinkWrench`) and LQR drives it back to upright — active disturbance rejection against an independent physics engine.

```bash
ros2 launch kontrolem_bringup cart_pole_gz.launch.py     # headless (server-only)
bash kontrolem_bringup/test/e2e_cart_pole_gz.sh          # automated pass/fail
```

Requires Gazebo Fortress + `ign_ros2_control` installed. One honest caveat vs. the custom sim: Gazebo's timestep/solver aren't exact-tick reproducible, so this e2e asserts tolerance/settle bands (pole kicked but in-basin, then returns to upright), while the deterministic custom-sim e2es remain the tight regression. It is **not** part of `e2e_all.sh` (Gazebo is heavy and a separate dependency).

## Base-state from Gazebo (`floating_box_gz`)

`cart_pole_gz` proves a *joint*-space plant is swappable. `floating_box_gz` proves the harder, **non-joint** case: a floating base's pose/twist entering the RT loop from an independent producer. `ros2_control` has no floating-base concept and Gazebo's `IgnitionSystem` (like a real robot) only exports *joints*, so the base state arrives a different way — over a topic, through **`kontrolem_state_bridge`**'s `OdometryBaseBridge`, a `SensorInterface` that re-exports `nav_msgs/Odometry` as the 13 `floating_base/*` state interfaces. The **same** `BaseStateSensor`/`FloatingStateProbe` reassembly used with the custom `FloatingBaseSimSystem` then rebuilds a manifold-correct SE(3) `State` — only the producer swapped (Gazebo ground truth instead of our sim), zero controller change. Notably this needs **no `gz_ros2_control`**: the controller_manager runs standalone and its only hardware is the topic bridge.

A free body is dropped **tilted** in Gazebo; its `odometry-publisher` emits world-frame pose + body-frame twist (matching Pinocchio's free-flyer). The tilt is deliberate — it makes the body-frame velocity differ from the world-frame one, so an independent SE(3) forward-integration of the odometry stream actually *discriminates* whether the twist frame is right (it is: prediction error ≈1 mm, vs ≈16 cm if the frame were wrong).

```bash
ros2 launch kontrolem_bringup floating_box_gz.launch.py   # headless (server-only)
bash kontrolem_bringup/test/e2e_floating_box_gz.sh        # automated pass/fail
```

Requires Gazebo Fortress + `ros_gz_bridge` installed. Like `cart_pole_gz`, it is **not** part of `e2e_all.sh` (heavy + separate dependency). This is the stepping-stone toward the real-Go2 path, where the same bridge consumes an external estimator (InEKF) instead of Gazebo ground truth.

## WBC vs. real contact (`quad_gz`)

The M6 endgame: `quad_stand`/`quad_push` prove the whole-body QP against a custom sim whose feet are *pinned* (bilateral, never-slipping contact). `quad_gz` runs the **same** WBC and the **same** `quad_stand_controllers.yaml` against **Gazebo's own contact and friction solver** — real unilateral feet that can slip, on a ground plane with finite friction. The joints come from `ign_ros2_control/IgnitionSystem`; the base pose/twist + contact flags come from **`kontrolem_gz/GzBaseStateSystem`**, a gz-native hardware component that reads ground truth straight from Gazebo's ECM (the in-gz controller_manager can only load `GazeboSimSystemInterface` hardware, so the topic bridge of `floating_box_gz` can't be used there).

Startup uses a **DetachableJoint weld**: Gazebo has no "hold until commanded", so the base spawns welded to a static anchor — which (a DART quirk turned feature) freezes the *entire* model rigid at the nominal posture — and the demo detaches it once the WBC is active, handing the controller a clean at-rest state. After the release the WBC holds the stance indefinitely; a one-shot 3000 N lateral shove (≈0.2 m/s of base momentum via `ApplyLinkWrench`) displaces the base ~6 cm and the WBC pulls it back to millimetres, cone margin healthy throughout. A 6000 N shove correctly overwhelms it — the friction cone saturates and `status()` reports it — so the success is bounded, not staged.

```bash
ros2 launch kontrolem_bringup quad_gz.launch.py        # headless; add gui:=true to watch
ign topic -t /quadruped/detach -m ignition.msgs.Empty -p ""   # release the startup weld
bash kontrolem_bringup/test/e2e_quad_gz.sh             # automated stance + push pass/fail
```

For the full follow-along — launch, verify the WBC is active, detach, push, choose the force, and troubleshoot "nothing happens" — see [How-to → Push the Gazebo quadruped](../how-to/push-the-gazebo-quadruped.md). The command order is strict: **launch → verify active → detach → push** (a push before detach hits a welded, frozen robot and does nothing).

Two hard-won integration notes, documented in DEVELOPMENT.md M6.2: while welded, **no joint of the model responds to applied torque** (don't diagnose the controller against a welded robot), and **high SDF joint damping makes DART swallow commanded joint forces entirely** (keep `<dynamics damping>` near zero and let the controller do the damping).

## WBC on the real Unitree Go2 (`go2_gz`)

`go2_gz` closes the loop from toy to real robot: it runs the **same** WBC as `quad_gz` on the **actual Unitree Go2** — the real 12-DoF quadruped with Unitree's own mass, inertia, and geometry (trunk 6.9 kg, ~15 kg total), vendored from the Go2 description. This is the payoff of the config-driven design: retargeting from the toy took **no controller-code change** — only a new robot URDF, a `go2_stand_controllers.yaml` (Go2's joint names, foot frames, nominal posture, base height, τ_max), a Go2 world, and a launch. The WBC's `RobotModel` binds joints and feet **by name** and sizes the QP from the model, so the same binary that stands the toy stands the Go2.

The standing posture is **offline-proven before any simulation**: an offline Pinocchio script derives `base_height` = 0.2868 m and `nominal_posture` (thigh 0.9, calf −1.8) so all four feet touch the ground, and confirms static equilibrium is feasible with peak joint torque ~6 N·m (well under Go2's 23.7 N·m limit) and large friction-cone margin. In Gazebo, after the same DetachableJoint weld/detach startup, the WBC holds the stance (base z 0.288, `ok` 1.0) and a one-shot **5000 N** lateral shove displaces the base ~6 cm before it recovers to sub-millimetre (Go2 is ~1.6× the toy's mass, so it needs more force than the toy's 3000 N for the same excursion).

```bash
ros2 launch kontrolem_bringup go2_gz.launch.py         # headless; add gui:=true to watch
ign topic -t /go2/detach -m ignition.msgs.Empty -p ""  # release the startup weld
bash kontrolem_bringup/test/e2e_go2_gz.sh              # automated stance + push pass/fail
```

The operational recipe is identical to the toy's (see [How-to → Push the Gazebo quadruped](../how-to/push-the-gazebo-quadruped.md)) — only the model name (`go2`), detach topic (`/go2/detach`), wrench topic (`/world/go2/wrench`), base link (`base`), and push magnitude differ. The Go2 description is vendored under a BSD license (Unitree geometry; CHAMP configs by Anuj Jain) — see the `kontrolem_description` README.

## Standing on an ESTIMATE — the sim-to-real path (`base_source:=estimate`)

Everything above stands the Go2 on **ground-truth** base state, read straight from Gazebo's ECM (`GzBaseStateSystem`). A real Go2 has no such oracle: the floating-base pose/twist must be **estimated** from the sensors a physical robot actually has — an IMU, joint encoders, and foot-contact detection. `base_source:=estimate` closes that gap: a `base_estimator` controller runs the contact-aided [state estimator](../explanation/state-and-non-joint-data.md#estimating-the-base-on-a-real-robot) and publishes the estimate on `/base_odom`, and `GzBaseStateSystem` feeds **that** to the WBC instead of the ECM (contact stays real from the ECM). The control loop never sees ground truth.

```bash
# WBC stands on its own estimate; the estimator runs automatically.
ros2 launch kontrolem_bringup go2_gz.launch.py base_source:=estimate
ign topic -t /go2/detach -m ignition.msgs.Empty -p ""      # release the startup weld
bash kontrolem_bringup/test/e2e_go2_estimator_closed.sh    # closed-loop stand + push
# The estimator also runs in the default (ecm) mode, publishing /base_odom for validation:
bash kontrolem_bringup/test/e2e_go2_estimator.sh           # open-loop estimate vs ground truth
```

The estimator is a **contact-aided complementary / leg-odometry filter** behind an
**InEKF-ready** interface (state `(R, v, p)`, `predict()`/`correct()`), so a full invariant
EKF can drop in later. Validated two ways: **open-loop**, the estimate tracks the (bridged)
ground truth to **1.6 mm / 0.03°** in stance; **closed-loop**, the Go2 stands (true height
0.289 m) and rejects a **5000 N** shove — recovering just as it does on ground truth — with
the estimate tracking the true base to **< 7 mm / 0.01°** throughout, all with only the
estimate in the loop. See [How-to → Estimate the floating-base state](../how-to/estimate-the-base-state.md).

## Shared launch body

All demos except `floating_spike` are thin wrappers over `build_sim_launch(urdf_file, controllers_yaml)` in `kontrolem_bringup/launch/_common.py`. It starts `robot_state_publisher`, `ros2_control_node` (with the URDF for both the resource manager and the controller), `joint_state_broadcaster`, then the control law once the broadcaster is up — and injects the cmeel library path so plugins load. Adding a demo is a two-line wrapper; see [How-to → Run on your own robot](../how-to/run-on-your-robot.md).

## URDFs

Launch URDFs live in `kontrolem_description/urdf/*.ros2_control.urdf` (body + `<ros2_control>` block). The corresponding pure-body URDFs used by the model/tests are in `kontrolem_model/robots/`.
