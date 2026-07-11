# Explanation — State on a manifold & non-joint data

> **For:** developers, especially anyone doing floating-base/legged work. **Assumes:** you've read [Architecture](architecture.md) and know ros2_control basics. Understanding-oriented — for the interface names see [Reference → ros2_control interfaces](../reference/ros2control-interfaces.md).

## Why `State` is not ℝⁿ

For a fixed-base robot, the configuration is a vector of joint angles — plain ℝⁿ, where `q + v·dt` is a valid update. A floating base breaks that. Its configuration includes an **SE(3) pose**: a 3-vector position and a **unit quaternion** orientation. Naively adding a velocity to a quaternion corrupts it (the result isn't unit-norm, so it isn't a rotation).

So the framework's `State` lives on a manifold: for a floating base, `nq == nv + 1` (7 pose coordinates, 6 velocity coordinates for the root), and configuration updates go through `RobotModel::integrate`, which applies the SE(3) group exponential on the root. Errors between configurations go through `difference`, the SE(3) logarithm. A whole-body controller forms its posture error with `difference` precisely so the base error lives in the same tangent space as the velocity and the acceleration it commands — the frames are consistent by construction. Getting this wrong is a silent corruption, not a crash, which is why the manifold operations are centralized in one place and unit-tested against known quantities.

## The hard part: non-joint state through ros2_control

Kontrol'Em is ros2_control-native by choice — controllers *are* ros2_control controllers, so they compose with the ecosystem. But ros2_control has **no concept of a floating base or a contact**. Its world is joints with scalar interfaces. A base pose is not a joint; a foot contact is not a joint. This is the sharpest tension in the whole design.

The resolution: carry non-joint state as **scalar `<gpio>` interfaces**, and reassemble it controller-side.

- The SE(3) base pose/twist is decomposed into **13 scalar state interfaces** (`pose.position.*`, `pose.orientation.*`, `twist.linear.*`, `twist.angular.*`) on a `<gpio name="floating_base">` block.
- Each foot's contact is **one scalar** (`contact.<foot>`) on a `<gpio name="contact">` block.

A **semantic component** — `BaseStateSensor`, modelled on ros2_control's own `IMUSensor`/`ForceTorqueSensor` helpers — claims those 13 scalars and reassembles them into the manifold `State`, normalizing the quaternion (the transport doesn't guarantee unit-norm). `ContactSensor` does the same for the contact scalars. The controller never re-derives the awkward index layout; the component owns the convention.

## Why this is a "compromise," honestly

Surfacing base state this way has real awkwardness, and the design names it rather than hiding it:

- There is **no standard ros2_control convention** for base pose, so Kontrol'Em defines one (`pose.*`/`twist.*`) that could clash with another stack's.
- The consumer must know the `q`/`v` index layout — which is exactly why it's encapsulated in the semantic component.
- On real hardware, base *position* is unsensed, so it comes from a state estimator; surfacing an estimator's output as a "hardware" state interface is philosophically odd (an estimator isn't hardware) but is the only real-time-correct way to feed it into a controller's state interfaces. Alternatives — controller-to-controller state chaining (semantically it's command-side), or folding estimation into the WBC (violates layering) — are worse.

A development spike (`floating_spike`) validated the mechanism before any controller depended on it: a read-only probe reassembled an SE(3) state from these scalars and confirmed the round-trip is exact. The verdict was that ros2_control-native non-joint state is *viable, not intolerably ugly* — so the framework proceeds this way rather than inventing a side-channel.

## Where the contact signal comes from (and its limits)

Three consumers want a contact signal: the WBC (which feet can push), the state estimator, and future gait switching. The signal enters through one `<gpio>` contract boundary (one scalar per foot, 0 = swing, 1 = stance). In the Gazebo Go2, `GzBaseStateSystem` reads it **for real** from each foot's contact sensor in the ECM — a foot that lifts under a push reports 0 (earlier demos faked a constant all-stance, which is enough for standing but gives the estimator nothing to work with). Contact *estimation* from joint torques (for a robot without foot sensors) is still deferred — see [Design decisions](design-decisions.md#whats-deliberately-not-done).

## Estimating the base on a real robot

On hardware the base *position and orientation* are unsensed — there is no ECM to read them from. They must be **estimated**, and this is where the `<gpio>` boundary pays off: an estimator is just another producer of the same base + contact interfaces. `kontrolem_estimation::BaseEstimator` (a ROS-free core) fuses the IMU with **leg odometry** — a stance foot does not move in the world, so the joint encoders + contact Jacobian give a base velocity/position measurement — to reconstruct the SE(3) base state. Its state is an InEKF-ready `(R, v, p)` behind a `predict()`/`correct()` interface, so a full invariant EKF can replace the internals later without touching any caller.

The wrapper (`BaseEstimatorController`) publishes the estimate as `nav_msgs/Odometry` on `/base_odom` — exactly what `OdometryBaseBridge` re-imports into the `floating_base` interfaces the WBC reads. So the estimator is a **drop-in replacement for the simulator's ground-truth base source**: the Gazebo Go2 can stand and reject a push on its *own estimate* with no ground truth in the control loop (`base_source:=estimate`). A subtlety worth stating: the accelerometer's gravity attitude aid, helpful in open loop, must be **off in the closed loop** — the controller's own leg motions produce lateral accelerations the aid misreads as tilt, and it is the WBC's active leveling (not the accelerometer) that supplies the closed-loop attitude reference. See [How-to → Estimate the floating-base state](../how-to/estimate-the-base-state.md).

Continue with [Safety, the supervisor & real-time](safety-and-realtime.md).
