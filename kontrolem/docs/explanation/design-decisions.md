# Explanation — Design decisions, trade-offs & what's not done

> **For:** developers and evaluators. **Assumes:** you've read the other Explanation pages. This is the honest register — the decisions, their trade-offs, and the explicit non-goals. Nothing here is aspirational; where something is unproven or deferred, it says so.

## Decisions and their trade-offs

### Pinocchio from the pip `cmeel` wheel (not apt)
The core needs a recent Pinocchio with analytic dynamics derivatives. The cmeel wheel provides it portably via pip. **Trade-off:** it bundles its own Boost under a non-standard prefix, so builds need `CMAKE_PREFIX_PATH` and runs need the libraries on `LD_LIBRARY_PATH`. The launch files auto-inject the path so `ros2 launch` "just works," but by-hand runs and `colcon test` don't — the single most common newcomer error. Mitigated by [a dedicated how-to](../how-to/fix-cmeel-library-errors.md).

### OSQP behind a `QpSolver` seam
Both the QP task-space controller and MPC/WBC solve QPs. OSQP is battle-tested and friendly to condensed MPC, so it's the backend — but it sits behind a seam (`QpSolver`, pImpl) so the C headers never leak and a different solver (e.g. ProxQP) can drop in. **Trade-off:** the seam adds a thin copy layer; accepted for the swappability and the clean ROS-free-core boundary.

### MPC and the WBC live in `kontrolem_controllers` (not their own packages)
The package plan calls for per-paradigm packages (`kontrolem_mpc`, `kontrolem_wbc`) so heavy dependencies stay isolated. In practice both currently live in `kontrolem_controllers` alongside LQR/LQG/QP. **This is a flagged deviation**, taken for velocity: neither pulls a dependency the package doesn't already have (OSQP, Pinocchio). The split becomes worthwhile the moment one needs a heavy dependency the others don't — that's when to do it.

### The standing platform is a 3-DoF-leg quadruped
Choosing the robot for the whole-body demo was itself a design decision, and two simpler options were rejected on physics:
- A **1-DoF-leg quadruped** is *locked rigid* when all four feet are pinned — a single-axis leg can only move its foot along a 1-D arc, so four pinned feet over-constrain the base to **zero residual degrees of freedom**. There is literally nothing for a balancer to do.
- A **two-point-foot biped** gives a knife-edge support (both feet on a line), so pitch is unsupported by contact geometry — a marginal, fragile platform.
- A **3-DoF-leg quadruped** (hip-roll, hip-pitch, knee; the Go2's structure, shrunk) gives a real 2-D support polygon and leaves the base **6 residual DoF** while the feet stay planted — so the WBC genuinely stabilizes the trunk. That's why it's the platform.

### A contact-constrained simulator (damped Schur complement)
The floating-base sim originally free-fell (unconstrained forward dynamics) — with no ground, no torque can hold a robot up. The standing demo needed **constrained** dynamics: the feet pinned, solved as a KKT system `[M −Jᵀ; J 0][q̈; λ] = [τ−h; −γ]` via a damped Schur complement (the damping keeps redundant contacts — a rigid body on four feet — well-posed). This is the "ground" the WBC pushes against, and the same math a real contact solver uses.

### Exact matrix-exponential MPC discretization (not Euler)
Euler discretization (`I + A·dt`) broke on a fast-unstable benchmark (a cart with a double pole). The MPC now uses exact matrix-exponential (Van Loan block) discretization, robust at any step. **Documented limitation:** even so, a condensed MPC on a fast-unstable system needs a *short* horizon — a long one makes the propagated dynamics explode and ill-conditions the QP. LQR (exact continuous Riccati) has no such sensitivity. An honest limit of the approach, not a bug.

### End-to-end tests are standalone scripts, not colcon launch-tests
The unit tests cover the ROS-free core; the runtime integration is guarded by shell scripts (`e2e_smoke.sh`, `e2e_all.sh`) that launch a demo and watch a joint settle. Full ros2_control launches flake under some CI/harness setups, and a hanging test in the suite is worse than a script you run on purpose. **Trade-off:** these don't run under `colcon test` automatically; that's the deliberate cost of not having flaky tests.

### Tracking feedforward: tried and reverted
A linear model-based feedforward for the Tracking dialect gave no measurable benefit, because the demo references on the underactuated cart-pole are dynamically infeasible (see [Problem specs](problem-specs.md)). It was reverted; the reference-acceleration plumbing is kept for MPC and for feasible/fully-actuated references later. A reminder that a feature has to earn its place with a real result.

## What's deliberately not done

Being explicit about the boundary is part of the design.

- **No locomotion.** The whole-body controller stands and rejects bounded pushes with a fixed all-stance contact set. There is no gait generator, footstep planner, or stepping controller. The contact-schedule machinery is plumbing for a *future* stepping controller, not a walker.
- **No real hardware / real robot.** Everything runs in the self-contained simulator. A real quadruped additionally needs a state estimator (base pose is unsensed) and a real contact signal, both out of scope here. The intended path is to treat each as an explicit integrator-supplied contract boundary.
- **No contact *estimation*.** Contact is provided as ground truth by the sim (all-stance). A momentum/torque-based contact estimator is the named future path, not built.
- **No H∞ / robust synthesis.** Considered and descoped to avoid a heavy Python/Slycot design-time dependency. Synthesis stays in-process C++ (LQR/LQG Riccati solves).
- **Heterogeneous bumpless transfer / gait-phase switching is unbuilt and is a research risk.** Switching between whole controllers of *different representations* (e.g. a compensator with internal state ↔ a stateless whole-body QP) has no off-the-shelf answer — "initialize the incoming controller to match the outgoing output" is ill-defined when the incoming controller has no persistent state. The supervisor's single-controller-plus-fallback path exists; multi-controller transitions are a designed-in extension, not a solved one.
- **Hard real-time on the QP path is not certified — but allocation is now clean.** The whole-body `compute()` is verified allocation-free by a malloc-level audit (this closed a former gap; the fixes were an in-place manifold `difference`, caching contact frame indices to avoid a per-tick name lookup, and computing frame kinematics in the local frame). What remains for *hard* real-time is bounding OSQP's data-dependent iteration count with a `max_iter` cap. Firm real-time is demonstrated; hard real-time is reachable and allocation-clean, with the iteration cap outstanding — see [Safety & real-time](safety-and-realtime.md).
- **LTV MPC controller.** The model can produce a per-step linearized prediction model (`rollout` + `linearize_along`), but a linear-time-varying MPC controller on top of it is not built — per-tick re-condensing isn't allocation-free and the payoff on the current demos was marginal.

## What *is* proven

For balance, the flip side. The fixed-base framework (LQR, LQG, MPC, QP task-space) and the floating-base standing WBC are implemented and validated: unit tests on the core numerics (linearization vs. finite difference, contact dynamics, allocation-free compute, closed-loop stabilization of each law) and an end-to-end smoke suite across five demos on three robots. The whole-body controller holds a quadruped and recovers from a push in closed loop against the contact simulator, running at a few hundred microseconds per tick. The claim "one runtime hosts five paradigms across fixed and floating base" is demonstrated, not asserted.
