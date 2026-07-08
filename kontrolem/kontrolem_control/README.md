# kontrolem_control

Layer-3 **controller contract** for Kontrol'Em v2 — the plugin interface every
controller implements, plus the plain data types that cross the controller
boundary. Header-only, ROS-independent.

## Purpose

Define the **one shape** the whole architecture rests on:

```cpp
compute(const State&, const ControlProblem&, double dt) -> const Command&
```

behind a multi-phase lifecycle `synthesize → configure → compute → status`. The
bet is that this single contract fits *both* a precomputed static-gain
controller (LQR, where `compute` is a matrix–vector product) and a structurally
different online-solved controller (a QP task-space controller, where `compute`
builds and solves an optimization). This package is the load-bearing artifact
that claim is tested against.

It contains only interfaces and plain structs — no algorithm:
- `Controller` — the plugin base (`capabilities / synthesize / configure /
  compute / status`).
- `ControlProblem` + `Regulation` — the capability-typed problem spec (only the
  setpoint dialect exists so far; `Tracking` / `TaskSpec` are named, not defined).
- `Synthesis` — the serializable artifact produced offline by `synthesize`.
- `State`, `Command`, `Status` — the per-tick data types (Eigen-only).

## Dependencies and build instructions

**Dependencies**
- `ament_cmake` — build tooling only (not a ROS runtime dependency).
- `Eigen3` — the vocabulary of every type here.
- `kontrolem_model` — Layer 1; appears in the `synthesize`/`configure`
  signatures (`RobotModel`).

> **No ROS client libraries** (`rclcpp` / `ros2_control` / messages). This is
> core, not runtime.

**Build & test** (from the workspace root; builds `kontrolem_model` first):
```bash
source /opt/ros/humble/setup.bash
P=$HOME/.local/lib/python3.10/site-packages/cmeel.prefix   # only if using pip/cmeel Pinocchio
colcon build --packages-select kontrolem_model kontrolem_control \
  --cmake-args -DCMAKE_PREFIX_PATH="$P"
export LD_LIBRARY_PATH="$P/lib:$P/lib64:$LD_LIBRARY_PATH"   # cmeel Pinocchio at test time
colcon test --packages-select kontrolem_control --event-handlers console_direct+
```
The test (`contract_compiles`) builds a trivial `NullController` and runs the
full lifecycle — proof the interface is coherent and implementable.

## Relation to other packages

- **Depends on** `kontrolem_model` (Layer 1) for the `RobotModel` handle passed
  to `synthesize`/`configure`.
- **Depended on by** every concrete controller package (LQR, QP task-space, and
  later MPC/WBC) — they implement `Controller`. They depend only on this stable
  contract, never on each other.
- **Independent of** the ROS runtime package: the runtime *uses* this contract
  (assembles a `State`, calls `compute`, reads `Status`) but this package knows
  nothing about ROS.

## Position in the complete architecture

```
  Layer 4  runtime (ros2_control) + supervisor            [ROS]
  Layer 3  kontrolem_control  ── the plugin contract ◀──   ← THIS PACKAGE
           + concrete controllers implement it
  Layer 2  problem spec (Regulation here; Tracking/TaskSpec later)
  Layer 1  kontrolem_model (dynamics service)
```

Note: for this slice the problem-spec types (`ControlProblem`, `Regulation`)
live *inside* this package for minimality; they are the Layer-2 concept and will
move to a `kontrolem_problem` package once a second dialect (Tracking) exists.

## Intended use

For **controller authors**: subclass `Controller`, declare `capabilities()`,
do heavy design in `synthesize()`, load it in `configure()`, and fill the
command in an allocation-free `compute()`. For the **runtime**: check `accepts()`,
drive `compute()` each tick, and act on `status()`.

## How to use it

```cpp
#include "kontrolem_control/controller.hpp"
using namespace kontrolem_control;

class MyController : public Controller {
  Capabilities capabilities() const override {
    return {{Dialect::kRegulation}, /*needs_velocity_state=*/true};
  }
  std::unique_ptr<Synthesis> synthesize(const RobotModel& m,
                                        const ControlProblem& p) const override { /* ... */ }
  void configure(const RobotModel& m, const Synthesis& s,
                 const ControlProblem& p) override { /* ... */ }
  const Command& compute(const State& x, const ControlProblem& p, double dt) override {
    const auto& reg = static_cast<const Regulation&>(p);  // safe: accepts() checked at wiring
    /* ... fill command_ (no allocation) ... */
    return command_;
  }
  const Status& status() const override { return status_; }
  Command command_;  Status status_;
};
```

See `test/test_contract_compiles.cpp` for a complete minimal implementation.
