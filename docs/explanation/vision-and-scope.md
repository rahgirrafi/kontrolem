# Explanation — Vision & scope

> **For:** anyone deciding whether to adopt or contribute to Kontrol'Em. **Assumes:** general robotics/controls awareness. This page is understanding-oriented — no commands. For doing, start at the [Tutorials](../tutorials/first-run-cartpole.md).

## The ambition: "MoveIt for control"

MoveIt gave motion planning a common home: many planners behind one interface, keyed off a robot description, so an integrator picks a planner without rewriting their stack. Control has no equivalent. Each paradigm — LQR, LQG, MPC, whole-body QP — tends to arrive as its own island, with its own state assumptions, its own runtime, its own idea of what "the controller" even is.

Kontrol'Em's thesis is that these paradigms can live behind **one controller contract** and **one ros2_control runtime**, keyed off a URDF, differing only in what happens inside a single `compute(state, problem, dt)` call. A stored LQR gain and an online whole-body QP genuinely *are* the same call shape at runtime — one is a matrix-vector product, the other builds and solves an optimization, but both consume a state and a problem and return a command.

## The shaping decision: design for legged, build fixed-base first

The hardest target — a legged robot under whole-body control — drives the architecture, but is not where we start. The design is required to make that hard case a **clean addition, not a rewrite**: a floating base, estimated/contact state, and full nonlinear dynamics must slot into the same layers the cart-pole uses. But the *build order* goes from the trivial case up:

1. **M0** — prove the contract with two structurally different controllers (a stored gain and an online QP) on a cart-pole and an arm.
2. **M1** — output feedback (LQG) and time-varying references (Tracking).
3. **M2** — linear MPC (horizon queries, receding-horizon QP).
4. **M3** — the floating-base model and non-joint state through ros2_control.
5. **M4** — the whole-body QP controller: a quadruped standing and rejecting a push.

Each abstraction is *earned* by a real controller that needs it, rather than front-loaded. By M4 everything the WBC requires — the model service, the problem dialects, the manifold state, the non-joint interfaces — already exists because earlier milestones paid for it.

## What Kontrol'Em is not (in this version)

Scope discipline matters as much as ambition:

- **No locomotion.** The whole-body controller *stands* and rejects bounded pushes with a fixed (all-stance) contact set. There is no gait, footstep planner, or stepping controller. The "contact-schedule" machinery is plumbing for a future stepping controller, not a walker.
- **No real hardware yet.** Everything runs in a self-contained simulator (the true rigid-body dynamics, no Gazebo). A real quadruped needs a state estimator and a real contact signal, which are deliberately out of scope here.
- **No robust-synthesis toolchain.** H∞ was considered and descoped to avoid a heavy Python/Slycot design dependency.

Being explicit about the boundary is what keeps the framework honest — the full register of trade-offs and non-goals is in [Design decisions](design-decisions.md).

## Who it's for

The primary user is an **integrator deploying controllers**, not a control theorist inventing new ones. That biases the design toward configuration UX, inspectability (telemetry, provenance), and safety (a supervisor with a trust predicate and a safe fallback). Extending the framework with a new paradigm is a first-class second use case — the plugin contract exists precisely so that adding a controller doesn't touch the ones already there.

Where to go next:
- The structure that makes this possible → [The four-layer architecture](architecture.md).
- The single call shape → [The controller contract & lifecycle](controller-contract.md).
