# Explanation — Problem specs & dialects

> **For:** developers. **Assumes:** you've read [The controller contract](controller-contract.md). Understanding-oriented — for the types see [Reference → Core API](../reference/core-api.md).

## What a "problem" is

`compute(state, problem, dt)` takes a *problem*: the specification of what the controller should achieve. Kontrol'Em types this as a small set of **capability-typed dialects**, each declaring what a controller must accept, rather than one god-struct with optional fields.

- **`Regulation`** — drive the state to a fixed setpoint (`q_ref`, `v_ref`). LQR consumes it as a state error to feed the gain; the QP consumes it as a task to minimize under its torque limit. Same problem, structurally different controllers.
- **`Tracking`** — follow a time-varying reference produced by a `TrajectorySource` the controller samples each tick. This is where MPC shines, because it can use the *future* reference over its horizon.
- **`TaskSpec`** — a weighted task hierarchy with constraints (the enum value exists; it is where a richer WBC/MPC spec will grow).

## Why dialects instead of one struct

The alternative — one `ControlProblem` struct carrying every possible field (setpoint, trajectory, tasks, constraints, contacts) — rots quickly. Trivial controllers wade through fields they ignore; new capabilities either bloat the struct or bolt on side-channels; and there's no way for a controller to say "I only handle setpoints" except at runtime.

Dialects make the capability explicit and checkable. A controller's `capabilities()` lists the dialects it accepts, and the runtime verifies the match before the loop (see [the contract](controller-contract.md)). New dialects are **new derived types** — there is no central variant to edit — which is exactly what keeps the framework open to controllers nobody has written yet.

## The MoveIt lesson, applied carefully

MoveIt's power came from making the *problem* (the planning scene, goals, constraints) a first-class, rich object. Kontrol'Em borrows that discipline for the hard case: for a whole-body controller, "tasks + constraints + contact schedule" is the natural spec, and it deserves to be typed richly.

But the same richness would make the trivial case ceremonious — a cart-pole setpoint does not want a task hierarchy. So the design borrows MoveIt's *lesson* without its *weight*: the rich `TaskSpec` is **not built before it's needed**. Today, `Regulation` and `Tracking` carry the real controllers; `TaskSpec` is named so the enum is stable, and will be fleshed out when a controller genuinely needs a hierarchy. Under-designing the trivial case and over-designing the hard case are both failures; typed dialects let each case pay only for what it uses.

## `Regulation` ⊂ `Tracking`

A constant setpoint is a degenerate trajectory: `ConstantReference` is a `TrajectorySource` whose `sample()` returns the same `q, v` for all `t`. So Regulation is Tracking with a trivial reference — which is why a single `TrajectorySource` seam unifies a setpoint, an analytic reference (the built-in `HarmonicReference`), and, later, a played-back trajectory or a live reference topic. The `sample()` call is on the real-time path, so it fills caller-preallocated buffers and must not allocate.

## The feasibility caveat

A reference is only trackable if the robot can physically achieve it. On an underactuated robot, a reference like "cart moves *and* pole exactly upright" is dynamically infeasible, so any controller will show a residual error — not a bug, a fact about the robot. This is also why a model-based feedforward (which `TrajectorySource` exposes via the reference acceleration) helps only for feasible or fully-actuated references. The dialects express *what you want*; physics still decides what's *achievable*.

Continue with [State on a manifold & non-joint data](state-and-non-joint-data.md) — the other input to `compute()`.
