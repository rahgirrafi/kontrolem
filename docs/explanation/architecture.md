# Explanation — The four-layer architecture

> **For:** developers. **Assumes:** you've seen the system run and know basic ROS 2. Understanding-oriented — for exact signatures see [Reference → Core API](../reference/core-api.md).

## The layers

Kontrol'Em is four layers. The boundary that matters most: **Layers 1–3 are plain C++ with no ROS dependency**, and Layer 4 is the only place that knows about ros2_control.

```
L4  kontrolem_ros2_control   ← the ROS boundary (KontrolemController, supervisor, telemetry)
L3  kontrolem_control        ← the controller CONTRACT + problem specs + types
    kontrolem_controllers    ← the controller IMPLEMENTATIONS (LQR/LQG/MPC/QP/WBC)
L2  (problem specs live in kontrolem_control)
L1  kontrolem_model          ← the RobotModel dynamics service (the spine)
```

- **L1 — `RobotModel`, the spine.** A queryable dynamics service over Pinocchio. The *same* model answers different questions: a linearization `(A,B)` for LQR/MPC; instantaneous `M, h` and contact Jacobians for the QP/WBC; a rollout for MPC. A frozen `(A,B,C,D)` is *one evaluation* of this service, not the model itself.
- **L2/L3 — the contract.** The problem dialects (what you want) and the `Controller` lifecycle (how a paradigm delivers it), plus the plain data types (`State`, `Command`, `Status`) that cross the boundary. Split into a stable **contract** (`kontrolem_control`) and the **implementations** (`kontrolem_controllers`), mirroring ros2_control's own `controller_interface` vs. controllers split.
- **L4 — the runtime.** `KontrolemController` is a single ros2_control controller that hosts any L3 plugin: it maps ros2_control interfaces to a `State`, calls `compute()`, writes the `Command`, and runs the supervisor. It is the *only* package that depends on both ROS and the core.

## Why a model-service spine (and not a controller-centric design)

An obvious alternative is to make the controller the hub — each controller reaches into the model however it likes. That was the failure mode of the previous iteration: the controller "owned everything," and every new paradigm re-derived model access inconsistently, turning predictive and optimization-based controllers into bolt-ons on a design implicitly shaped around stored linear gains.

Making `RobotModel` the spine inverts that. Controllers become thin consumers of a disciplined query API. Adding a paradigm means calling different queries, not re-architecting. And because the model is a *service* rather than a cached artifact, the framework naturally supports controllers whose needs we haven't anticipated.

## Why the ROS-free core is a hard invariant

Layers 1–3 having zero ROS client-library dependencies is not stylistic:

- **Testability.** The core numerics are unit-tested off-robot, with no controller_manager, no DDS, no launch flakiness. A closed-loop stabilization test is a plain C++ program.
- **Reuse.** The same core can back a design-time tool, a different middleware, or an offline analysis, without dragging ROS along.
- **Discipline.** A single, visible boundary (`kontrolem_ros2_control`) prevents the slow leak of ROS types downward that recreates the old coupling. The rule is enforceable: the core packages' manifests simply must not depend on `rclcpp`, `controller_interface`, or the messages.

Controllers emit plain structs (`Command`, `Status`); the L4 runtime translates them to ROS messages. Nothing below L4 knows a topic exists.

## Where the numerics live

The philosophy is: **if it's numerics, wrap it; if it's a seam or a contract, build it.** Pinocchio (dynamics), OSQP (QP), and Eigen (linear algebra) are wrapped behind interfaces — Pinocchio behind `RobotModel`'s pImpl (so it never appears in a public header), OSQP behind the `QpSolver` seam (so it can be swapped). What Kontrol'Em *builds* is the model-service API, the problem spec, the controller lifecycle, the supervisor, and the ros2_control mapping — the parts that are contracts, not calculators.

Continue with:
- [The controller contract & lifecycle](controller-contract.md) — how one `compute()` fits every paradigm.
- [Problem specs & dialects](problem-specs.md) — how "what you want" is typed.
