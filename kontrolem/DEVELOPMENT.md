# Kontrol'Em v2 — Development Steps (living document)

This tracks the **implementation** of the accepted v2 design
(`refactored-skipping-parnas.md`, Parts A–D). It is a *living* document: statuses
change as work lands, and when a step needs to change I flag it in
**§ Proposed step updates** with a reason and wait for your call before editing
the step itself.

Status legend: `[ ]` pending · `[~]` in progress · `[x]` done · `[!]` needs decision

---

## Guiding invariants (checked every step)

1. **No ROS client libraries in the core** (Layers 1–3). `ament_cmake` + `colcon`
   are allowed as *build tooling*; the forbidden thing is `rclcpp` /
   `ros2_control` / message deps in `package.xml`. (Refined from the plan's
   "no ROS on PATH" — see update U2.)
2. **Allocation discipline on real-time paths.** Anything called inside a
   controller's `compute()` must be allocation-free / bounded. (Scope widened by
   update U4 — the model queries are on the RT path too.)
3. **Every package ships a README** with the six sections in
   `.claude/common_instructions.md`.
4. **Every open choice sits behind a seam** so it is swappable (Pinocchio source,
   CARE solver, QP solver).

---

## Current phase — Interface-validation vertical slice (Week 1)

**Hypothesis under test:** does one `compute(state, problem, dt) → command`
contract genuinely fit *both* a precomputed static-gain LQR *and* a structurally
different online-solved QP task-space controller? Either answer is a win; a
contradicted assumption is a successful outcome.

- `[x]` **Step 0 — Toolchain & dependencies.** *(added; see U1)*
  cmake 3.22, g++ 11.4, Eigen 3.4, ROS Humble. No system/apt Pinocchio C++ →
  using the pip **cmeel** Pinocchio (`~/.local/.../cmeel.prefix`), no sudo.
  Seam: `-DCMAKE_PREFIX_PATH=<cmeel>`. Alt: `apt install ros-humble-pinocchio`.
- `[x]` **Step 1 — `kontrolem_model` (minimal).** `linearize(q,v,τ) → (A,B)` via
  Pinocchio `computeABADerivatives`, behind pImpl. `ament_cmake` package, builds
  + tests under colcon. Finite-difference test PASSES (`max|A−A_fd|`,
  `max|B−B_fd|` ≈ 1e-10, tol 1e-5). README written.
- `[x]` **Step 2 — The `Controller` contract (`kontrolem_control`).** Header-only
  ament package: `Controller` (`capabilities/synthesize/configure/compute/
  status`), polymorphic `ControlProblem`+`Regulation`, `Synthesis`, plain
  `State/Command/Status`. `NullController` lifecycle test PASSES; links Layer 1;
  no ROS client libs. README written. *Paused for review before Step 3.*
- `[x]` **Step 3 — `LqrController`.** `synthesize` = `model.linearize` at the
  operating point → actuation selection `S` (U3) → CARE (in-house Eigen
  Hamiltonian solver, `care.hpp`, D6). `compute` = `u = u_eq − K·error` (gemv,
  allocation-free). `status` = validity-region margin. Test PASSES: closed-loop
  stable (max Re −0.79 vs open-loop +3.97), finite command, region flag works.
  README written. → **CHECKPOINT A reached — verdict below / in chat.**
- `[x]` **Step 4 — `QpTaskSpaceController`.** `synthesize` = no-op; `compute`
  queries `model.dynamics()`, builds the inverse-dynamics QP `z=[q̈;τ]` and
  solves it online via **OSQP** (D9) behind the `QpSolver` seam, with the
  torque-limit inequality **active** (clamps τ to 5.0 at the hard tilt). Consumes
  the same `Regulation`/`State`/`compute` shape as LQR. Test PASSES. README updated.
- `[x]` **Step 5 — Allocation audit + verdict.** `operator new` hook over
  `compute()`: **LQR = 0 allocs/tick** (gemv, invariant holds); **QP = 68
  allocs/tick** from `model.dynamics()` — **U4 empirically confirmed** on the
  cart-pole. → **CHECKPOINT B verdict: interface HELD (below / in chat).**

**Bare-harness only for this phase** — NO `ros2_control` integration yet
(separate risk, Part D; must not contaminate the interface test).

---

## Decisions & deviations log

| # | Decision | Rationale | Seam / reversibility |
|---|---|---|---|
| D1 | Pinocchio from pip **cmeel**, not apt | no sudo; already installed for v1 Python | `CMAKE_PREFIX_PATH`; swap to `ros-humble-pinocchio` in one line |
| D2 | model `B` w.r.t. **full generalized torque** (not actuated-selected) | keeps Layer 1 paradigm-neutral; WBC wants full-τ anyway | actuation `S` applied by the controller (LQR at step 3) |
| D3 | core = `ament_cmake` + colcon, **SHARED** lib, Pinocchio linked `PRIVATE` behind pImpl | uniform build without leaking ROS or Pinocchio to downstream | invariant #1; Pinocchio not `ament_export`ed |
| D4 | FD test is dependency-free (no gtest) | minimal slice | swap to `ament_cmake_gtest` later if desired |
| D5 | CARE solver / QP solver choice | deferred to steps 3 / 4 | chosen behind seams |
| D6 | CARE via in-house **Hamiltonian-eigenvector** solver (Laub 1979), Eigen-only | avoids embedding Python/SciPy in the C++ core; textbook; gated on closed-loop stability | `care.hpp` seam; swap to Slycot later if needed |
| D7 | added `RobotModel::gravity_torque(q)` to Layer 1 | LQR's operating-point `u_eq` needs `g(q)`; small additive query | Layer-1 API growth, not a rewrite |
| D8 | v2 built as its **own isolated colcon workspace** at `src/kontrolem_v2/` | v1's `kontrolem_controllers` (in `src/packages/`) collides on package name | see open question below |
| D9 | QP solver = **OSQP 0.6.2** (already at `/opt/ros/humble`, zero install) behind `QpSolver` pImpl seam | present without sudo; battle-tested; ProxQP absent | swap to ProxQP later via the seam |
| D10 | OSQP **control-loop settings**: `warm_start=1, polish=0, adaptive_rho=0, eps=1e-6` | option-(a) malloc audit found the default config allocates 42/tick (polish + rho refactor); this config = **0 malloc/tick** while keeping the clamp crisp to ~1e-6 | validated the seam — solver-specific RT tuning lives in one place |
| D11 | `CartPoleSimSystem` **holds the plant until a controller commands it** (gated on `perform_command_mode_switch`) | an unstable robot free-falls during controller_manager bring-up before any controller claims the effort interface | flag `commanding_`; without it the LQR starts from an unrecoverable pose |

**Option (a) — malloc-level audit (DONE).** Added `test_malloc_audit` (interposes
`malloc/calloc/realloc` via `__libc_*`) — the `operator new` hook could not see
OSQP's C allocations. Found OSQP default = 42 malloc/tick; retuned (D10) to **0**.
Both `compute()` paths are now allocation-free at **both** the C++ `new` and C
`malloc` levels. Caveat: 0-allocation is necessary but not sufficient for
hard-RT — OSQP's iteration count is still data-dependent, so a hard-RT
deployment must also cap `max_iter` (separate, later).

**Checkpoint B outcome (interface hypothesis):** VALIDATED. One contract fit both
a precomputed static gain (LQR: offline CARE → gemv, 0 alloc) and an online QP
solve (QP: no synth → build+solve per tick, constraint active) with **no change
to the contract headers**. The only real issue found is **U4 (model allocates on
the RT path)** — a Layer-1 fix, not an interface fix, which is the good outcome.
New contract note added: `compute()`'s returned reference is valid only until the
next `compute()` (allocation-free aliasing; documented on the interface).

**Open question surfaced (D8):** v1 still lives in `src/packages/` and its
`kontrolem_controllers` collides with the v2 package of the same name. For now v2
builds in isolation (`cd src/kontrolem_v2 && colcon build`, own build/install).
Decide later: retire/move v1, or keep v2 permanently isolated. Not blocking.

---

## Proposed step updates (flagged for your approval — reasons below)

> These change the *plan/steps*, so I list them here rather than silently editing.

- **U1 — Add an explicit Step 0 (toolchain/deps).** *Reason:* the environment had
  no C++ Pinocchio; capturing the exact dependency + build flags is needed for
  anyone else to reproduce the build. *(Applied above; low-risk, documentary.)*
- **U2 — Refine the invariant/litmus test wording** (plan Part D) from "build
  L1–L3 with no ROS on PATH" to "no `rclcpp`/`ros2_control`/msgs in
  `package.xml`". *Reason:* `ament_cmake` itself requires ROS tooling sourced, so
  PATH-purity is unachievable; the package.xml grep is the meaningful, checkable
  boundary. *(Fold into the plan when it returns from Ultraplan.)*
- **U3 — Step 3 (LQR) explicitly owns the actuation-selection `S`.** *Reason:*
  consequence of D2 — the model returns full-τ `B`, so the controller must select
  the actuated column(s) before CARE. Cheap, but must be stated so LQR is correct.
**Status of these updates:** U1 applied · U2 to fold into plan on return · U3
applied (Step 3) · **U4 RESOLVED (demonstrated → fixed): the Step-5 audit caught
the QP path at 68 allocs/tick; the fix adds `RobotModel::Workspace` (a per-caller
preallocated Pinocchio `Data`) + a buffer-output `dynamics()` overload, and the QP
`compute()` now measures 0 allocs/tick. The audit is now a both-paths regression
guard.** Remaining nuance: the `operator new` hook doesn't see OSQP's C malloc —
a fuller (malloc-level) audit is future work.

- **U4 — (IMPORTANT) The model service is on the RT path, and currently
  allocates per call.** `RobotModel::linearize/aba` create a fresh
  `pinocchio::Data` each call. That is fine for LQR (queried only at
  *synthesize*, offline) — but the **QP task-space controller queries the model
  inside `compute()` every tick**, so those per-call `Data` allocations will show
  up in the Step-5 allocation audit. *Reason it matters:* invariant #2 (and Part
  D's #1 prototype-first risk) is about the QP `compute` path, and the model is
  *part of* that path — auditing only the QP solver would miss it. *Proposed
  change:* (a) widen the Step-5 audit to cover model queries called in `compute`;
  (b) give `RobotModel` an RT-query mode that reuses a preallocated `Data` (e.g.
  the controller owns a scratch `Data`/query-context) before or during Step 4.
  This is a real design refinement to Step 1's API, surfaced early — exactly when
  it's cheapest to fix.

---

## Beyond the slice (deferred — from plan Parts B/C & roadmap)

Not started; listed so the slice's scope stays honest.

- **Rest of M0** *(in progress)*:
  - `[x]` `kontrolem_ros2_control` — the ROS boundary: `KontrolemController`
    (`controller_interface::ControllerInterface`) hosts any core `Controller`
    (LQR/QP via `control_law` param), maps joint state/command interfaces ↔ the
    ROS-free core, + a minimal supervisor (safe-action on non-ok `status()`).
    Builds as a pluginlib controller. Added `RobotModel::from_urdf_string` for it.
  - `[x]` `kontrolem_description` — cart-pole `<ros2_control>` URDF +
    `CartPoleSimSystem`, a self-contained sim `SystemInterface` that integrates
    the true dynamics via `RobotModel::aba` (symplectic Euler). Underactuation is
    structural (only `cart_joint` exports an `effort` command interface). Builds
    the model from `info.original_xml`; generic over fixed-base Euclidean URDFs.
  - `[x]` `kontrolem_bringup` — controller_manager config
    (`cart_pole_controllers.yaml`, 200 Hz, LQR) + `cart_pole.launch.py`
    (robot_state_publisher + ros2_control_node + joint_state_broadcaster +
    kontrolem_controller). **Ran end-to-end: the LQR balances the cart-pole
    through ros2_control** — pole 0.15→0 rad, cart returns to centre, 0 supervisor
    cuts. The ROS trajectory matches an offline discrete-loop reference (same core
    code) tick-for-tick (tick0 τ=7.58 in both). **M0 acceptance (LQR path) met.**

  **M0 findings (during e2e bring-up):**
  - **D11 — the sim plant is *held* until a controller commands it.** The sim
    hardware activates and the controller_manager loop integrates it from the
    moment bring-up starts — so an *unstable* robot (upright pole) free-falls for
    the ~0.5 s before any controller claims the effort interface, starting the LQR
    from an unrecoverable 1.4 rad. Fix: `CartPoleSimSystem::read()` freezes the
    state until `perform_command_mode_switch` reports a controller took command of
    our effort interface. Physically legitimate ("rig braked until control
    engages") and self-contained. *This was the whole cause of an early apparent
    "LQR diverges" — the core was verified correct offline before touching it.*
  - **Region-metric nuance (flagged, not yet fixed).** `LqrController`'s trust
    `margin = q_dev_max − max|q − q_eq|` counts **cart position**, but the
    cart-pole dynamics are translation-invariant in the cart coordinate — cart
    displacement does not reduce linearization validity. During a healthy recovery
    the cart travels ~0.3 m, so a tight box spuriously trips the supervisor. M0
    works around it with `q_dev_max: 1.0`; the principled fix (region over pole
    angle / a P-ellipsoid, per Part C C1) is a Layer-3 refinement for later.

  - `[x]` **QP path e2e** — ran on a **2-DoF fully-actuated planar arm**
    (`arm2.ros2_control.urdf`, reusing `CartPoleSimSystem`; both joints take
    effort). `ros2 launch kontrolem_bringup arm2.launch.py` → the QP task-space
    law **regulates the arm to its setpoint by inverse dynamics**: shoulder
    1.0→0, elbow −1.0→0, held horizontal against gravity (steady τ≈[−19.6,−4.9] =
    the gravity torque), matching an offline reference tick-for-tick. The
    torque-limit inequality is **active in the transient** (τ saturates at the
    `tau_max=30` cap on the large initial error, enforced inside the QP). It was
    **not** run on the underactuated cart-pole (task-space computed-torque can't
    stabilize it — expected, not a bug). **M0 acceptance (QP path) met.**

  → **M0 COMPLETE.** Both structurally-different laws — LQR (offline CARE→gemv)
  and QP task-space (online OSQP solve) — run end-to-end on the *identical*
  runtime, differing only in the `control_law` param and the robot. The v2
  central claim ("one runtime, many paradigms") is now demonstrated on hardware-
  shaped robots, not just in unit tests. Launch pipeline shared via
  `launch/_common.py` (`build_sim_launch`); cart-pole=LQR, arm2=QP.
  - `[ ]` (deferred, per plan B6) `kontrolem_problem` / `kontrolem_solvers` split,
    `kontrolem_msgs` — earned when a second consumer/dialect appears.
- **M1** *(in progress)*: LQG/H∞ + `kontrolem_synthesis` (Python design tool) +
  `Tracking`.
  - `[x]` **LQG output-feedback compensator (`LqgController`, C++).** The THIRD
    structural form behind the one contract: a *dynamic* controller with internal
    observer state (LQR = static gain, QP = stateless solve, **LQG = compensator
    with memory**). Method: separation principle — control gain `K` from the
    control CARE (as LQR) + steady-state Kalman gain `L` from the **dual/filter
    CARE**, i.e. the *same* `care.hpp` with `A→Aᵀ, B→Cᵀ, Q→W, R→V` (no new solver,
    no Python/slycot yet — that is earned by H∞). Measures **positions only**
    (`C=[I 0]`), estimates velocity; `capabilities().needs_velocity_state=false`.
    Runtime realization = Euler-stepped observer in deviation coords
    (`x̂̃ ← x̂̃ + dt(A_obs x̂̃ + B_act ũ + L ỹ)`, `ũ=−K x̂̃`), allocation-free.
    Wired into the runtime factory (`control_law: lqg`). Test `lqg_cartpole`
    (5th controllers test) gates: balances from 0.15 rad on positions alone **and**
    the velocity estimate tracks truth (max post-warmup error <5e-2). **Ran e2e**
    (`cart_pole_lqg.launch.py`): pole 0.15→0, cart→0, matching the offline
    output-feedback reference. This is the first M1-defining result: output
    feedback fits the unchanged `compute()`.
  - `[ ]` **Honor `needs_velocity_state=false` in the runtime** — LQG currently
    still *claims* velocity interfaces (the sim provides them; LQG ignores them
    internally, so the control-theoretic output-feedback claim holds). Making the
    runtime claim positions-only when the law declares it is a small, separable
    Layer-4 refinement (gate `state_interface_configuration` / `on_activate` /
    `update` on the capability). Deferred, flagged.
  - `[ ]` H∞ (needs slycot → the Python `kontrolem_synthesis` engine), `Tracking`
    dialect + trajectory reference.
- **M1.5:** LPV / gain-scheduling (`ScheduledRegulation` dialect).
- **M2:** linear MPC (`kontrolem_mpc`) + `linearize_along`/`rollout` model queries.
- **M3:** floating base (SE(3)) + `kontrolem_state_bridge` + non-joint interfaces;
  **contact-detection producer decision** (Part D open gap).
- **M4:** QP-WBC (`kontrolem_wbc`) + `kontrolem_go2` + supervisor transitions.
  (Standing + push-recovery only — **no locomotion**, per Part D.)

---

## Update policy

I propose a step change (in § Proposed step updates) whenever: a dependency or
tool assumption is contradicted by the environment; a design choice made in code
implies a different downstream step; a Part-D risk is confirmed or falsified by a
prototype; or a checkpoint verdict says the interface must change. I state the
reason and wait for your decision before editing the affected step.
