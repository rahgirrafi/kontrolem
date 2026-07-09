# How to tune a controller's weights and limits

> **For:** a user with a controller running that isn't behaving how you want. **Assumes:** a built, sourced workspace and a config YAML you can edit. **Goal:** change stiffness, aggressiveness, limits, and safety margins.

All tuning is done in the controller's YAML (under `kontrolem_bringup/config/`). After editing, rebuild the bringup package and relaunch:

```bash
colcon build --packages-select kontrolem_bringup && source install/setup.bash
ros2 launch kontrolem_bringup <your_demo>.launch.py
```

Below are the knobs per law. For the exhaustive list of every parameter, type, and default, see [Reference → Controller parameters](../reference/controller-parameters.md).

## LQR (`control_law: lqr`)

```yaml
lqr.q_diag: [1.0, 10.0, 1.0, 1.0]  # state penalty, length 2*nv, order [q; v]
lqr.r_diag: [0.1]                  # input penalty, one per actuated joint
lqr.q_dev_max: 0.5                 # trust-region size (see below)
```

- **Larger `q_diag` entries** penalize deviation of that state more → stiffer, faster correction of it. Penalize the unstable coordinate (e.g. pole angle) most.
- **Larger `r_diag`** penalizes control effort → gentler, less aggressive.
- **`q_dev_max`** is the trust region: if the state drifts too far from the operating point (in a Q-weighted distance) the supervisor stops trusting the linear gain and applies the safe action. Raise it to tolerate larger excursions; lower it to fail safe sooner.

## LQG (`control_law: lqg`)

Same `q_diag`/`r_diag` as LQR, plus the estimator covariances:

```yaml
lqg.w_diag: [...]   # process-noise covariance, length 2*nv
lqg.v_diag: [...]   # measurement-noise covariance, length nq
lqg.innov_max: 0.5  # innovation gate: how surprised the filter may be before failing safe
```

- **Higher `w_diag`** → trust the measurements more (faster, noisier estimate). **Higher `v_diag`** → trust the model more (smoother, laggier estimate).

## MPC (`control_law: mpc`)

```yaml
mpc.q_diag: [...]    # state penalty, length 2*nv
mpc.r_diag: [...]    # input penalty, per actuated joint
mpc.horizon: 30      # number of steps looked ahead
mpc.dt_mpc: 0.02     # seconds per horizon step
mpc.tau_max: 100.0   # hard torque limit enforced INSIDE the QP
```

- **Longer `horizon`** = more foresight but heavier solve. On fast-unstable robots a *too-long* horizon can ill-condition the QP — keep it short there (see [Explanation → Design decisions](../explanation/design-decisions.md)).

## QP task-space (`control_law: qp`)

```yaml
qp.task_weight: [1.0, 10.0]  # per-DoF weight on the acceleration task, length nv
qp.kp: 50.0                  # task stiffness (position error → acceleration)
qp.kd: 10.0                  # task damping
qp.tau_max: 20.0             # torque limit enforced inside the QP
```

## WBC (`control_law: wbc`)

```yaml
wbc.kp_base: 100.0   # base pose/orientation stiffness
wbc.kd_base: 20.0    # base twist damping
wbc.kp_post: 25.0    # joint posture stiffness
wbc.kd_post: 5.0     # joint posture damping
wbc.w_base: 100.0    # task weight on the 6 base DoF
wbc.w_post: 1.0      # task weight on the joints
wbc.mu: 0.7          # friction coefficient (foot slip limit)
wbc.tau_max: 40.0    # per-joint torque limit
```

- **Raise `kp_base`/`kd_base` together** (keep them roughly critically damped, `kd ≈ 2·√kp`) for a stiffer stance. **Weight `w_base` ≫ `w_post`** so the controller prioritizes holding the trunk over posture.
- **Lower `mu`** to model slippery ground — the controller then commands more vertical, less horizontal contact force, and fails safe sooner under a push.

> The gain semantics (why the task is expressed as a desired acceleration, why the trust region is Q-weighted) are in [Explanation → Safety, supervisor & real-time](../explanation/safety-and-realtime.md).
