# Live monitoring (`~/diagnostics` + the dashboard)

Every Kontrol'Em controller can stream its inner signals so you can watch the
loop while it runs — reference, measured state, tracking error, control effort,
observer estimate, timing and validity — on a live web dashboard. This is
off by default (it adds a lock-free publish to the update loop); turn it on with
a parameter.

## 1 · Enable the feed on the controller

Set `publish_diagnostics: true` in the controller's parameters (the Gazebo demo
configs already do this):

```yaml
lqr_controller:
  ros__parameters:
    artifact_path: package://lqr_controller/config/lqr_cart_double_pendulum_ros2_control.yaml
    publish_diagnostics: true
    diagnostics_decimation: 1   # publish every Nth update (raise to thin a fast loop)
```

The controller then publishes a `std_msgs/Float64MultiArray` on
`~/diagnostics` (e.g. `/lqr_controller/diagnostics`) every update. It is
**realtime-safe**: the layout and buffer are built once in `on_configure`, and
each cycle only fills a preallocated array behind a non-blocking `try-lock`.

## 2 · Open the dashboard

```bash
ros2 run state_space_response_viz response_monitor \
    --ros-args -p controller:=/lqr_controller
# then open http://127.0.0.1:8080
```

or with the launch file (optionally auto-opening the browser):

```bash
ros2 launch state_space_response_viz monitor.launch.py \
    controller:=/lqr_controller open:=true
```

The node subscribes to the diagnostics feed and `/joint_states`, keeps a
sliding window, and serves a same-origin **Flask + Server-Sent-Events**
dashboard (no external dependencies). Panels: reference vs measured position,
joint velocity, tracking error, control effort, observer state `ξ̂`, plus
performance tiles (RMS error, per-channel settling), timing (loop rate,
execution time, deadline misses) and a validity/saturation badge. Panels that
need the diagnostics feed grey out until `publish_diagnostics` is on; the
observer panel is empty for static-gain LQR (it carries no estimator).

## The message layout

The array is **self-describing**: `layout.dim` lists one entry per field
section, in wire order, each with a `label` and `size`. Consumers slice the
flat `data` by walking the layout — no offsets are hard-coded. `n` is the joint
count, `m` the actuated-joint count.

```{list-table}
:header-rows: 1

* - Section
  - Size
  - Contents
* - `time`
  - 1
  - controller clock `t` [s]
* - `timing`
  - 4
  - `period_s`, `update_us` (this update's compute time), `rate_hz` (1/period), `deadline_miss` (1 if `update_us` exceeded the loop budget)
* - `flags`
  - 2
  - `valid` (1 inside the validity region), `safe_action_code` (0 hold_u_eq · 1 zero_command · 2 deactivate)
* - `x`
  - 2n
  - measured state — absolute positions `q` then velocities `q̇`
* - `ref`
  - R
  - reference interfaces (absolute): a full-state setpoint (2n) for state feedback, or one per output for output feedback
* - `err`
  - E
  - law tracking error: state error `x − x_ref` (2n) for LQR, output error (n_out) for LQG/H∞
* - `u`
  - m
  - command effort on the actuated joints (absolute)
* - `xhat`
  - 0 or n_ξ
  - observer/compensator state `ξ` (empty for static-gain LQR)
```

Positions are published **absolute** (`q_eq + deviation`) so a consumer needs
no extra context to plot them. The producer is
`KontrolemChainableController::publish_diagnostics`
(`kontrolem_controllers_base`); the parser is
`state_space_response_viz/diagnostics.py`.

### Channel labels (`~/diagnostics_info`)

Alongside the numeric feed the controller latches a companion
`std_msgs/String` on `~/diagnostics_info` — a small JSON object published once
(transient-local, so a late-joining monitor still receives it) with the
**names** behind the numeric channels, in the artifact's joint order:

```json
{"joints": ["cart_joint", "joint1", "joint2"],
 "actuated": ["cart_joint"],
 "outputs": ["cart_joint.q", "joint1.q", "joint2.q"],
 "error": ["cart_joint (pos)", "joint1 (pos)", "joint2 (pos)",
           "cart_joint (vel)", "joint1 (vel)", "joint2 (vel)"],
 "command": ["cart_joint (effort)"]}
```

The `error` labels are law-specific: for **state feedback** (LQR) they are the
joint position/velocity errors above; for **output feedback** (LQG/H∞) they are
the measured `output_names`. The dashboard uses these to name the tracking-error
traces and the per-channel settling rows, so nothing is guessed from
`/joint_states` ordering.
