# Estimate the floating-base state (sim-to-real)

**Goal:** stand the Go2 on a *state estimate* — the floating-base pose/twist computed
from an IMU + leg kinematics + foot contact — instead of simulator ground truth. This is
the last step toward the real robot, where no ground-truth base state exists.

**Prerequisites:** the Go2 Gazebo demo works (see
[Demos → WBC on the real Unitree Go2](../reference/demos.md#wbc-on-the-real-unitree-go2-go2_gz)),
Gazebo Fortress installed, the workspace built.

## The idea in one paragraph

A whole-body controller needs the base's SE(3) pose and twist every tick. In simulation
you can read them straight from the physics engine; a real robot cannot. `kontrolem_estimation`'s
`BaseEstimator` reconstructs them from what a real robot *does* have: it dead-reckons the
IMU (`predict`), then corrects the drift with **leg odometry** — a foot in stance does not
move in the world, which turns the joint encoders + contact Jacobian into a base
velocity/position measurement (`correct`). The `base_estimator` controller runs this and
publishes `nav_msgs/Odometry` on `/base_odom`; `GzBaseStateSystem` can consume that instead
of the ECM.

## Step 1 — see the estimate next to the truth (open loop)

Run the normal Go2 demo. The estimator runs alongside the WBC by default (the WBC still
uses ground truth here), publishing its estimate on `/base_odom`:

```bash
ros2 launch kontrolem_bringup go2_gz.launch.py
ign topic -t /go2/detach -m ignition.msgs.Empty -p ""   # release the startup weld
```

Compare the estimate against the ground-truth `/base_truth_odom` (published only for
validation — never used by control):

```bash
ros2 topic echo --once /base_odom          # the ESTIMATE (world pose, body twist)
ros2 topic echo --once /base_truth_odom    # ground truth, for comparison
```

Or run the automated check, which asserts the estimate tracks the truth (height and
orientation tight, transient bounded) through a stance and a push:

```bash
bash kontrolem_bringup/test/e2e_go2_estimator.sh
```

## Step 2 — close the loop (WBC stands on the estimate)

Add `base_source:=estimate`. Now `GzBaseStateSystem` feeds the WBC the estimator's
`/base_odom` instead of the ECM — the control loop never sees ground truth. Per-foot
contact is still read for real from the ECM.

```bash
ros2 launch kontrolem_bringup go2_gz.launch.py base_source:=estimate
ign topic -t /go2/detach -m ignition.msgs.Empty -p ""
# shove it — the TRUE base recovers, though control sees only the estimate:
ign topic -t /world/go2/wrench -m ignition.msgs.EntityWrench \
  -p 'entity {name:"go2::base" type:LINK} wrench {force {y: 5000}}'
```

The automated closed-loop test judges the **true** base (which control never sees):

```bash
bash kontrolem_bringup/test/e2e_go2_estimator_closed.sh
```

## Tuning knobs (`base_estimator` in `go2_stand_controllers.yaml`)

| Param | Meaning | Note |
| --- | --- | --- |
| `k_vel` | trust in leg-odometry base velocity (0–1) | `1.0` = fully trust while in stance (standing) |
| `k_pos` | leg-odometry position-anchor pull | higher = height pinned harder to foot contact (closed-loop stability) |
| `k_grav` | accelerometer gravity attitude aid | **`0.0` in the loop** — see the warning below |
| `accel_gate`, `gyro_gate` | gate the gravity aid to quasi-static instants | reject the aid when accelerating/rotating |
| `base_height` | seed height | the offline-proven standing height (accounts for foot-sphere radius) |

!!! warning "Turn the gravity aid OFF in the closed loop"
    The accelerometer "which-way-is-down" aid (`k_grav`) *improves* open-loop accuracy but
    **destabilizes the closed loop**: the WBC's control-induced lateral acceleration looks
    like a tilt to the accelerometer, so the estimate tilts, the WBC over-corrects, and it
    rings into a limit cycle (height ran away to > 10 cm in testing). In the loop the WBC
    *actively holds the base level* — it **is** the attitude reference — so gyro-only
    orientation is both stable and accurate (`k_grav: 0.0`). This holds for the **InEKF** too
    — its gravity measurement is disabled in the walk loop (`inekf.sigma_grav: 1e6`); leaving
    it on collapsed the first closed-loop walk. See [pick the estimator](#pick-the-estimator-complementary-filter-or-inekf).

## Pick the estimator: complementary filter or InEKF

There are two estimators behind the same `predict()`/`correct()` interface, selected with the
`estimator_type` launch/param:

```bash
# M8 contact-aided complementary filter (default) — fixed blend gains
ros2 launch kontrolem_bringup go2_gz.launch.py base_source:=estimate

# M11 Right-Invariant EKF (InEKF) — covariance-weighted, robust to contact flicker
ros2 launch kontrolem_bringup go2_gz.launch.py base_source:=estimate estimator_type:=inekf
```

| | Complementary (`complementary`) | Right-Invariant EKF (`inekf`) |
| --- | --- | --- |
| State | `(R, v, p)`, fixed blend gains | `(R, v, p, d_1…d_N)` + covariance |
| Velocity | direct leg-odometry least-squares (responsive) | observed through the FK/position covariance coupling |
| Bad-foot handling | one mis-sensed foot poisons the **shared** least-squares | rejected by **median-residual innovation gating**; the good feet still correct the base |
| Best for | standing, postures (rock-solid) | walking (feet leaving/rejoining the ground) |

Offline (with injected contact flicker) the InEKF tracks the true base **~3.4× tighter** than
the complementary filter (`test_invariant_estimator`). Its `inekf.*` params (noise densities,
`inekf.fk_gate`) live in the `base_estimator` block of the controllers YAML.

!!! note "Walking on the estimate is improved but not yet fully reliable"
    For the closed-loop **walk**, the InEKF walks forward & upright in most runs and tighter
    than the complementary filter, but still occasionally diverges. Reliability needs the next
    layer — feeding the gait's **planned** contact schedule to the estimator (instead of the
    flickering sensed contact), and/or IMU-bias states. Standing/postures on either estimator
    are solid. See [Make the Go2 walk](make-the-go2-walk.md) and run
    `EST=inekf bash kontrolem_bringup/test/e2e_go2_walk_closed.sh` to probe it.

## What to reuse for a real robot

The output path is already the real one: `/base_odom` is exactly what
`kontrolem_state_bridge::OdometryBaseBridge` re-imports into the `floating_base` state
interfaces (world pose, **body** twist / REP-145). On hardware you replace Gazebo's sensor
streams with the robot's real IMU + encoders + contact, keep the same estimator, and pick
`estimator_type: complementary` or `inekf` — both sit behind the one `IStateEstimator`
(`predict()`/`correct()`) interface, so no caller changes.
