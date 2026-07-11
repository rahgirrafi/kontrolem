# kontrolem_estimation

A ROS-free floating-base **state estimator** for a legged robot. The `BaseEstimator`
fuses an IMU with leg odometry (stance-foot kinematics) to estimate the base
pose/twist — the quantities a real robot has no ground-truth oracle for.

## Purpose

On a real robot there is no simulator ECM to read the floating base's world pose and
twist from; they must be **estimated** from the sensors the robot actually has — an
IMU, joint encoders, and foot-contact detection. This package is that estimator. It is
the last missing piece of the sim-to-real path: with it, the whole-body controller
stands (and later walks) on an *estimate* of the base state rather than on simulator
ground truth.

The estimator is a **contact-aided complementary / leg-odometry filter**, right-sized
and provably drift-bounded for standing, behind an **InEKF-ready** interface: the state
is an SE_2(3) element `(R, v, p)` and the API is `predict()` / `correct()`, so a full
invariant EKF can drop into the same boundary later (alongside the future
locomotion-control milestone) without touching any caller.

## Dependencies and build instructions

- **Build deps:** `ament_cmake`, `Eigen3`, `kontrolem_model` (FK / contact Jacobian),
  `kontrolem_control` (the shared `Status` type). Pinocchio stays hidden inside
  `kontrolem_model` (pImpl), so this package and its users only ever see Eigen.
- **Build:** `colcon build --packages-select kontrolem_estimation`.
- **Test (offline gate):** build with `-DBUILD_TESTING=ON`, then run the test binary
  with the cmeel Pinocchio libraries on `LD_LIBRARY_PATH` (the workspace convention for
  every Pinocchio-linked test):

  ```bash
  P="$HOME/.local/lib/python3.10/site-packages/cmeel.prefix"
  LD_LIBRARY_PATH="$P/lib:$P/lib64:$LD_LIBRARY_PATH" \
    ./build/kontrolem_estimation/test_base_estimator
  ```

## Relation to other packages

- **Consumes** `kontrolem_model` — `frame_position`, `contact_jacobian_stacked`,
  `neutral`/`joint_*_index` — for the leg-odometry kinematics, and `kontrolem_control`'s
  `Status` for the health signal.
- **Wrapped by** `kontrolem_ros2_control`'s `BaseEstimatorController`, which feeds the
  estimator the IMU / joint / contact interfaces each tick and publishes its output as
  `nav_msgs/Odometry` on `/base_odom`.
- **Feeds** (indirectly) `kontrolem_state_bridge`'s `OdometryBaseBridge`, which re-imports
  `/base_odom` into the `floating_base` state interfaces the WBC reads — the estimator is
  a drop-in replacement for the simulator's ground-truth `GzBaseStateSystem` base source.

## Position in the complete architecture

Layer: **estimation** — a peer of the controllers, on the *state* side of the loop. It
sits between the raw sensors (IMU + encoders + contact) and the controller's state
input, turning what a real robot can measure into the SE(3) base state the model-based
controllers require. Like the controllers, its core is ROS-free and Pinocchio-only-behind
`kontrolem_model`; the ros2_control wrapper lives in `kontrolem_ros2_control`.

## Intended use

- Estimate the floating-base pose/twist for a standing quadruped from IMU + leg odometry,
  so control closes on the estimate (sim-to-real).
- A stable, swappable base for future work: keep the `(R, v, p)` / `predict`/`correct`
  interface and replace the internals with a full InEKF when dynamic locomotion exists to
  exercise it.

Out of scope: InEKF internals, gait/locomotion control, magnetometer yaw aiding, and
tuning to a specific physical IMU.

## How to use it

```cpp
#include "kontrolem_estimation/base_estimator.hpp"
using namespace kontrolem_estimation;

BaseEstimatorConfig cfg;
cfg.contact_frames  = {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};
cfg.actuated_joints = { /* 12 joint names, encoder order */ };
BaseEstimator est(model /* floating-base RobotModel */, cfg);

// On activation: seed from a known pose; anchor the feet in stance.
est.seed(p0, R0, q_joints, stance);

// Each tick (e.g. 500 Hz):
est.predict(gyro_body, accel_body, dt);       // IMU dead-reckoning
est.correct(q_joints, v_joints, stance);      // leg-odometry + gravity aid bound the drift

const auto p = est.position();                // world-frame position
const auto q = est.orientation();             // world<-base quaternion
const auto v_body = est.velocity_body();      // body-frame linear velocity (REP-145)
const auto w_body = est.angular_body();       // body-frame angular velocity
if (!est.status().ok) { /* stance constraint residual too large */ }
```
