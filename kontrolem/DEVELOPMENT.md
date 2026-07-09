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
- **M1** *(in progress)*: LQG + `Tracking` (time-varying reference). **H∞ is
  descoped** — see the note below; the LQG line's "earned by H∞" caveat is moot.
  - `[x]` **LQG output-feedback compensator (`LqgController`, C++).** The THIRD
    structural form behind the one contract: a *dynamic* controller with internal
    observer state (LQR = static gain, QP = stateless solve, **LQG = compensator
    with memory**). Method: separation principle — control gain `K` from the
    control CARE (as LQR) + steady-state Kalman gain `L` from the **dual/filter
    CARE**, i.e. the *same* `care.hpp` with `A→Aᵀ, B→Cᵀ, Q→W, R→V` (no new solver,
    all in-house Eigen). Measures **positions only**
    (`C=[I 0]`), estimates velocity; `capabilities().needs_velocity_state=false`.
    Runtime realization = Euler-stepped observer in deviation coords
    (`x̂̃ ← x̂̃ + dt(A_obs x̂̃ + B_act ũ + L ỹ)`, `ũ=−K x̂̃`), allocation-free.
    Wired into the runtime factory (`control_law: lqg`). Test `lqg_cartpole`
    (5th controllers test) gates: balances from 0.15 rad on positions alone **and**
    the velocity estimate tracks truth (max post-warmup error <5e-2). **Ran e2e**
    (`cart_pole_lqg.launch.py`): pole 0.15→0, cart→0, matching the offline
    output-feedback reference. This is the first M1-defining result: output
    feedback fits the unchanged `compute()`.
  - `[x]` **Honor `needs_velocity_state=false` in the runtime.** `KontrolemController`
    now caches the law's `needs_velocity_state` at `on_configure` and gates
    `state_interface_configuration` (claim velocity only if needed), `on_activate`
    (resolve `vel_idx_` only if needed), and `update` (read velocity only if
    claimed; else `State::v` stays zero) on it. **Verified e2e:** with LQG,
    `ros2 control list_controllers --verbose` shows required state interfaces =
    `{cart,pole}/position` **only** (no velocity) and the pole still balances
    (velocity estimated internally); LQR/QP/tracking still claim position+velocity
    and are unaffected. The capability system now genuinely drives interface
    claiming — output feedback is real at the interface level, not just internal.
  - `[x]` **`Tracking` dialect + trajectory reference.** A second `ControlProblem`
    dialect (time-varying reference) behind a `TrajectorySource` seam
    (`kontrolem_control/trajectory.hpp`: abstract `TrajectorySource` +
    `ConstantReference` (= Regulation as a degenerate trajectory) +
    `HarmonicReference`, all allocation-free `sample(t, …)`). **`LqrController` now
    accepts BOTH `Regulation` and `Tracking`** — the same gain law
    `u = u_eq − K(x − x_ref)` serves both; only `x_ref` differs (fixed setpoint vs
    reference sampled at `state.t`). This is the first real exercise of the
    capability-typed-problem thesis with >1 dialect: one controller, two dialects,
    per-tick narrowing on `problem.kind()`. Operating point for the gain =
    reference at t=0 (LTV re-linearization is M2/MPC). Offline: the cart tracks
    `A cos(ωt)` with error scaling as expected with ω (0.014/0.041/0.155 at
    ω=0.3/0.5/1.0 — textbook feedback-only lag). Test `lqr_tracking` (8th test)
    gates: accepts both dialects, cart follows the reference (amp≈A, err<0.1),
    pole upright. **Ran e2e** (`cart_pole_tracking.launch.py`, runtime builds a
    `Tracking`+`HarmonicReference` from params, `State::t` from the activation
    clock): cart follows `0.3 cos(0.5 t)` while balancing the pole (|pole|<0.04).
    Note: **feedback-only.** Acceleration feedforward was implemented and then
    **reverted** — an honest finding: a linear model-based feedforward
    `u_ff = B⁺(ẋ_ref − A·x̃_ref)` was built (with `TrajectorySource::sample` now
    also emitting the reference acceleration `a_out`), but it gave *no* tracking
    improvement (0.015/0.044/0.163 vs 0.014/0.041/0.155 at ω=0.3/0.5/1.0). The
    reason is control-theoretic, not a bug: the demo reference "cart moves while
    the pole stays *exactly* upright" is **dynamically infeasible** for the
    underactuated cart-pole (you cannot accelerate the cart without tilting the
    pole), so no feedforward can realize it — feedback does the work regardless.
    Feedforward pays off only for a *feasible* / fully-actuated reference; deferred
    until such a case exists (e.g. LQR-tracking on a fully-actuated arm, or MPC).
    The reference-acceleration plumbing (`a_out`) is kept as it costs ~nothing and
    MPC will use it; the unused feedforward gemvs were removed (ship only what's
    demonstrably beneficial).
  - `[~]` **H∞ — DESCOPED (user decision, 2026-07-08).** Not implemented and
    removed from the roadmap "for now." *Rationale:* it is the only planned
    controller that needs **slycot** (Python) for its DGKF two-Riccati synthesis,
    which would force the cross-language Python synthesis engine + artifact-
    serialization boundary before they are otherwise needed, plus a
    dependency-availability risk in this environment. LQR/LQG/QP already prove the
    three structural forms (static gain / dynamic compensator / online solve)
    without it. If robustness is wanted later, it can be re-added behind the same
    contract. Correspondingly, `kontrolem_synthesis` (the Python design tool) is
    **no longer an M1 requirement** — synthesis stays in-process C++ until a
    controller actually needs an offline/Python design step.
- **M1.5:** LPV / gain-scheduling (`ScheduledRegulation` dialect).
- **M2** *(in progress)*: linear MPC + `linearize_along`/`rollout` model queries.
  - `[x]` **M2.1 — `MpcController` core + offline proof + test.** LTI condensed
    receding-horizon control, the *fourth* paradigm behind the one contract and the
    plan's payoff for the offline/online seam. **synthesize/configure** (heavy,
    once): `model.linearize` at the operating point → Euler-discretize `(A_d,B_d)`
    → discrete-LQR terminal cost via a new `solve_dare` (added to `care.hpp`,
    reusing that seam) → **condense** the horizon into a dense QP in the input
    sequence `U` (`X = Sx x0 + Su U`; Hessian `H = 2(SuᵀQ̄Su+R̄)` constant, gradient
    map `G` with `q = G·x0`). **compute** (per tick): `x0 ←` deviation state,
    `q = G x0`, warm-solve via the **`QpSolver`/OSQP seam** (reused unchanged), apply
    `u_0` (recede). Input box `−τ_max ≤ u_k ≤ τ_max` is a hard QP constraint.
    Offline (cart-pole, pole 0.15→upright): unconstrained regulates (peak τ 7.1);
    **tight `τ_max=3` → QP clamps to exactly 3.0 (constraint active, not post-hoc)
    and still stabilizes.** Test `mpc_cartpole` (9th test) gates stabilization +
    limit-respected + limit-active. **Plan deviation (noted):** `MpcController`
    lives in `kontrolem_controllers` for now (reuses `QpSolver` in place, no new
    heavy dep) rather than a separate `kontrolem_mpc` package (plan B10) — extract
    when it grows. Discretization is Euler (fine here; exact/`rollout` +
    `linearize_along` for LTV is a later step).
  - `[x]` **M2.2 — runtime wiring + e2e.** `control_law: mpc` added to the runtime
    factory (params `mpc.q_diag/r_diag/horizon/dt_mpc/tau_max`). Ran e2e
    (`cart_pole_mpc.launch.py`, N=30, dt_mpc=0.02, τ_max=6): pole 0.15→0, cart
    returns to centre (more cart travel in the transient because the hard τ_max caps
    the pull-back). Four paradigms now run through the identical runtime
    (LQR/LQG/QP/MPC by `control_law`).
  - `[x]` **M2.3 — RT allocation audit of MPC `compute()`.** Extended
    `test_malloc_audit` to cover MPC: the **30-var condensed QP solves malloc-free
    (0/call over 1000 calls)** through the same `QpSolver`/OSQP seam (control-loop
    config, D10). So the RT allocation story now covers all three online/gemv paths
    (LQR 0, QP task-space 0, MPC 0). Caveat unchanged: 0-alloc is necessary not
    sufficient for hard-RT — OSQP iteration count is still data-dependent (cap
    `max_iter` for a hard-RT deployment; separate later concern).
  - `[x]` **M2.4 — Tracking-MPC (horizon reference).** `MpcController` now accepts
    `Tracking` too: each tick it samples the reference at the N future horizon times
    and adds the tracking gradient `q -= M_ref·Xref_dev` (`M_ref = 2 SuᵀQ̄` stored in
    the artifact; sampling + gemv preallocated, still malloc-free). This is the
    predictive advantage, **quantified**: on the cart following `0.3cos(ωt)` MPC
    tracks **~6× tighter than feedback-only LQR** — err 0.0028/0.0073/0.0266 vs
    0.014/0.041/0.155 at ω=0.3/0.5/1.0. (Notably MPC tracks this *infeasible*
    reference well where the LQR feedforward couldn't — it optimizes the error over
    the whole horizon rather than inverting one instant.) Test `mpc_tracking` (10th
    test); **ran e2e** (`cart_pole_mpc_tracking.launch.py`): cart follows the
    reference tight to ±0.30, pole within ±0.04. Regulation MPC unchanged (no
    regression).
  - `[ ]` M2.5 — LTV re-linearization (`rollout` + `linearize_along` model queries)
    for nonlinear-along-trajectory MPC. Needs new **Layer-1 API** — a larger step,
    deferred.

  → **M2 (LTI linear MPC) COMPLETE.** A fourth paradigm — constrained
  receding-horizon optimal control — runs behind the one contract, on both dialects
  (Regulation + Tracking), e2e through the identical runtime, malloc-free, with the
  predictive tracking advantage measured. Only LTV (M2.5) remains, gated on new
  model-service queries.
- **M3** *(in progress)*: floating base (SE(3)) + `kontrolem_state_bridge` +
  non-joint interfaces; **contact-detection producer decision** (Part D open gap).
  De-risking the **model layer offline first**, before any ROS plumbing.
  - `[x]` **M3.1 — floating-base model service (SE(3)-aware).** `RobotModel` gained
    a `BaseType{kFixed,kFloating}` build arg (Pinocchio free-flyer root), plus
    `center_of_mass(q)`, `neutral()`, and a manifold-correct `integrate(q,v,dt)`
    (group exp — quaternion stays unit). New test robot
    `robots/floating_biped.urdf` (floating trunk + 2 legs + foot links;
    nq=9, nv=8). Test `floating_base` (11th test) checks against KNOWN quantities:
    nq==nv+1, unit quaternion at neutral, **CoM exactly matches the hand
    computation** (z=−0.0295), and π/2-yaw `integrate` gives the exact quaternion
    (norm preserved) where a naive `q+v·dt` breaks the norm. This addresses Part D
    D2 (SE(3) convention risk) at the model layer. **Build note:** the
    `from_urdf_*` signature changed (added defaulted `BaseType`) — an ABI change,
    so a **full `colcon build`** is required (a partial rebuild leaves stale
    controller binaries referencing the old symbol; caught + fixed).
  - `[x]` **M3.2 — contact Jacobians.** `RobotModel::contact_jacobian(q, frame) →
    3×nv` (world-aligned translational, `v_world = J·v`) + `frame_position(q,
    frame)`. Test `contact_jacobian` (12th test) validates it by finite-difference
    of the foot's world position under **manifold** perturbations of q (including
    the 6-DoF floating-base root tangent): `max|J − J_fd| = 1.4e-07`. This is the
    key WBC ingredient (foot no-slip / friction-cone / `Jᵀλ`) and further de-risks
    the SE(3) conventions (Part D D2). Model-layer floating-base support is now
    complete and fully validated offline.
  - `[x]` **M3.3 — non-joint-state SPIKE (flagged substrate-strain checkpoint).**
    Built the minimal proof and got a **GO verdict**. Pieces: `FloatingBaseSimSystem`
    (integrates SE(3) dynamics via `aba` + manifold `integrate`, exports base
    pose/twist as 13 scalar interfaces on a `<gpio name="floating_base">` block +
    joint states); `FloatingStateProbe` controller (claims the base scalars + joint
    states, reassembles a manifold-correct `State`, normalizes the quaternion,
    computes CoM); `floating_biped.ros2_control.urdf` + `floating_spike.launch.py`.
    **Result:** `ros2 control list_hardware_interfaces` shows the base
    `floating_base/pose.*`/`twist.*` scalars exported and claimable; the probe
    reassembles the base falling under gravity with **quat_norm = 1.000000** and
    **CoM tracking base_z exactly** (offset −0.03) — the round-trip
    SE(3) pose → scalar interfaces → manifold `State` is exact.
    **Verdict: viable, NOT intolerably ugly → proceed ros2_control-native** (no
    pivot to a middleware-neutral surface). Real awkwardness noted, all tractable:
    (a) 13 verbose scalar interfaces per base under a Kontrol'Em naming convention;
    (b) the consumer must know the q/v index layout (SE(3) root at q0..6/v0..5,
    joints after) — factor into a `BaseStateSensor` semantic component; (c)
    quaternion needs normalizing after transport; (d) twist is in the free-flyer
    local frame (a documented convention). *(Launch gotcha: array params
    (`joints`) must go in the controllers YAML, not an inline dotted-dict on the
    CM node — that only reliably carried the scalar `robot_description`.)*
  - `[~]` **M3.4 — productionize the spike** (given GO):
    - `[x]` **`BaseStateSensor` semantic component** — a self-contained helper
      (mirrors ros2_control's `IMUSensor`/`ForceTorqueSensor`) that owns the base
      interface-naming convention (13 scalars) and the reassembly into the SE(3)
      root slots `q(0..6)/v(0..5)` **incl. quaternion normalization**, so no
      controller re-derives that coupling. `FloatingStateProbe` refactored to use
      it — re-ran the spike, identical result (quat_norm=1, CoM tracks base_z).
    - `[ ]` **`ContactSensor` component + the contact-signal PRODUCER decision**
      (Part D D1 open gap) — the next real fork; surfaced to the user.
    - `[ ]` fold base/contact needs into `Capabilities`; `[ ]` `kontrolem_state_bridge`
      (topic→interface) for a real estimator.
  - `[ ]` M3.5 — validation through the runtime against a known quantity.
### Consolidation phase (M3 paused by user; harden the fixed-base framework)

Priority order agreed with the user: (1) observability, (2) package hygiene,
(3) top-level README/getting-started, (4) finish M2.5 + region metric.

- `[x]` **C1 — Observability: `kontrolem_msgs` + telemetry.** New `kontrolem_msgs`
  package (rosidl) with `ControllerDiagnostics` (named fields: `control_law`, `q`,
  `v`, `q_ref`, `tau`, `ok`, `margin`, `safe_action`, `update_us`) — replaces the
  v1 `Float64MultiArray`-layout approach. `KontrolemController` publishes it on
  `~/diagnostics` each tick via a **`realtime_tools::RealtimePublisher`**
  (best-effort `trylock`), gated by an opt-in `publish_diagnostics` param
  (default false). **Verified e2e:** `ros2 topic echo
  /kontrolem_controller/diagnostics` shows the named fields live (LQR `update_us`
  ≈ 0.87 µs). 7 packages now.
- `[!]` C2 — package hygiene: extract `kontrolem_mpc` + `kontrolem_problem`.
  **DEFERRED (revised assessment).** Scoping showed this is *medium*-risk, not the
  "low-risk" I first called it: a clean split needs **3 new packages**
  (`kontrolem_solvers` for the shared `care.hpp`/`qp_solver`, then `kontrolem_mpc`
  + `kontrolem_problem`) touching ~20 files across include paths and namespaces
  (17 files do `using namespace kontrolem_control`). And the present payoff is
  thin — paradigm packages already get the problem+contract types *without*
  depending on the impl package, so the split mostly buys future-proofing. A
  churny, regression-prone refactor with marginal current value is better done as
  a focused, reviewed effort than in an autonomous loop. Revisit when a second
  paradigm package (WBC) actually needs the isolation.
- `[x]` C3 — top-level project README + getting-started. Front-door `README.md`
  at the workspace root: what Kontrol'Em is, the paradigm table, the 4-layer
  architecture, package list, build (cmeel), the demo launches, tests, status.
- `[x]` C4a — **region-metric fix** (the flagged cart-position issue). Replaced
  `LqrController`'s box `max|q−q_eq|` with a **Q-weighted trust distance**
  `sqrt(devᵀ Q dev / trace(Q))`, `dev = x − x_eq`. Reuses the controller's own `Q`
  (which already down-weights the translation-invariant cart position ~10× vs the
  pole), so cart travel no longer spuriously trips the region — `q_dev_max` stays
  the intuitive threshold, and it's allocation-free (preallocated `Q·dev`).
  Restored `q_dev_max: 0.5` on the cart-pole demo; **verified e2e: 0 supervisor
  cuts** (was the reason for the `1.0` workaround). All 12 tests still green
  (region flag, allocation + malloc audits included).
- `[~]` C4b — **LTV MPC (M2.5)**:
  - `[x]` **Model queries `rollout` + `linearize_along`** (Layer 1). `rollout(q0,v0,
    tau_seq,dt) → Trajectory` (semi-implicit Euler, manifold-correct via
    `integrate`); `linearize_along(q,v,tau) → {A_k,B_k}` (per-step linearization).
    Offline queries (allocate); reusable by MPC, trajectory optimization, and the
    WBC. Test `rollout` (13th test): dims, upright-equilibrium holds, tilt falls,
    integrator convergence (coarse-vs-fine err 0.027), `linearize_along` matches
    per-point `linearize` exactly.
  - `[ ]` **LTV MpcController path** — flagged design tradeoff: re-linearizing
    along the trajectory each tick means rebuilding the condensed QP per tick, so
    that path is **not malloc-free** (unlike the LTI MPC). Legitimate (firm-RT),
    but it changes a framework property, and the payoff on the mildly-nonlinear
    cart-pole is likely marginal (LTI already tracks/balances well). Surfaced for a
    decision rather than built unilaterally.

- `[x]` C5 — **hard-benchmark demonstration: cart-DOUBLE-inverted-pendulum**
  (1 actuator, 2 passive poles, 3 DoF — open-loop max Re **+10.3**, far more
  unstable than the single pole's +3.97). Proves the framework scales past the
  2-DoF toy. New `robots/cart_double_pole.urdf` (+ `.ros2_control.urdf`, reuses the
  generic `CartPoleSimSystem`); bringup config + `cart_double_pole.launch.py`.
  **LQR** stabilizes it — unit test `lqr_double_pole` (14th test: closed-loop
  stable + recovers from both poles at 0.1 rad) **and e2e** (both poles→0, cart→
  centre, 0 cuts). **MPC** stabilizes it too, which surfaced a real improvement:
  - **Exact (matrix-exponential) discretization in `MpcController`** (Van Loan
    block trick, `unsupported/Eigen/MatrixFunctions`) replacing Euler
    `I + A·dt`. The double pendulum's fast unstable modes broke Euler at
    `dt_mpc=0.02`; exact discretization is accurate at any step. Regression-clean
    (single-pole MPC tests still pass).
  - **Finding (documented, not a bug):** LTI *condensed* MPC on a fast-unstable
    system also needs a **short enough horizon** — a long one makes `A_d^N`
    explode and ill-conditions the QP (it under-actuates and diverges). The double
    pole needs ≈0.24 s (N≈12 at dt_mpc=0.02); LQR (exact continuous CARE, infinite
    horizon) has no such sensitivity. A real, honest limitation of the approach.

- `[x]` C6 — **automated e2e smoke test** (the runtime integration was only
  manually verified). `kontrolem_bringup/test/e2e_smoke.sh` launches a demo,
  watches a joint settle, PASS/FAIL with hard timeouts + self-cleanup;
  `e2e_all.sh` runs the suite. **All pass** (LQR cart-pole, LQG, LQR
  cart-double-pole, QP arm — 3 robots × 3 paradigms). Deliberately a **standalone
  script, not a colcon launch-test**: full ros2_control launches flake under this
  harness (a hanging/flaky test is worse than a script run on purpose) — an honest
  tradeoff. Guards the runtime integration the 14 unit tests don't cover.

**Consolidation phase outcome:** the endorsed priorities are done — observability
(C1) and the front-door README (C3) landed; the region-metric debt (C4a) is paid;
the M2.5 model queries (`rollout`/`linearize_along`) are in; the framework is
demonstrated on a hard benchmark (C5) with a real MPC discretization improvement.
Package hygiene (C2) and the LTV MPC *controller* (C4b) are consciously deferred.
The fixed-base framework is materially more production-solid (inspectable, tighter
supervisor, exact MPC discretization, documented, proven on cart-double-pole).

- **M4:** QP-WBC (`kontrolem_wbc`) + `kontrolem_go2` + supervisor transitions.
  (Standing + push-recovery only — **no locomotion**, per Part D.)

---

## M4 — QP-WBC (standing), in progress

The flagship. Building it de-risk-first (prove the physics offline before ROS).

- `[x]` **M4.1 — contact-constrained dynamics in the model layer + a standing
  platform, proven offline.** Three realizations drove this:
  1. The floating sim **free-falls** (`aba`, no ground). A standing demo needs
     *contact-constrained* forward dynamics — feet pinned — or no torque can hold
     the robot up. Added `RobotModel::contact_forward_dynamics` (KKT
     `[M −Jᵀ; J 0][q̈; λ] = [τ−h; −γ−Baumgarte]` solved via a **damped Schur
     complement**, so redundant contacts stay well-posed), plus the RT building
     blocks `contact_jacobian_stacked` (3·nc×nv) and `contact_drift` (γ = d/dt(J)·v
     via frame classical acceleration). Also `joint_q_index`/`joint_v_index` (name
     → generalized-coordinate index — the WBC's actuation-selection Sᵀ needs it).
  2. A **1-DoF-leg** quad is *locked rigid* when 4 feet are pinned (each single-axis
     leg moves its foot on a 1D arc ⇒ 4 feet over-constrain the 10-DoF system ⇒ 0
     residual base DoF ⇒ nothing to balance). So a point-foot balancer needs
     richer legs.
  3. Fix: **`robots/floating_quadruped.urdf`** — 3-DoF legs (hip-roll x, hip-pitch
     y, knee y), the Go2's structure shrunk. nq=19, nv=18; with 4 feet pinned the
     base retains **6 residual DoF** (nv − rank J = 18 − 12) — the WBC actively
     stabilizes the trunk. (The 1-DoF `floating_biped` stays as the M3 model-test
     fixture.)
  - **Offline proof** `test_contact_dynamics` (kontrolem_model, **15 tests green**):
    A stacked-J == per-foot J (err 0); B drift(v=0)=0; C **residual DoF = 6**
    (platform WBC-viable, not locked); D full gravity-comp ⇒ static equilibrium
    holds (base_dz=0, feet don't move over 2 s); E under a brief hip-torque impulse
    the **feet stay planted to 0.4 mm** (critically-damped Baumgarte, ω≈20 rad/s)
    while the base moves freely — exactly the feet-planted-base-free regime the WBC
    operates in. The contact "ground" is validated before any ROS wiring.
- `[x]` **M4.2 — `ContactSensor` semantic component** (mirrors `BaseStateSensor`):
  per-foot contact scalars on a `<gpio>` (`contact.<foot>`), `read_into` /
  `stance_into`. Owns the contact interface-naming convention. Compiles as part of
  `kontrolem_ros2_control`; exercised end-to-end at M4.4.
- `[x]` **M4.3 — `WbcController`** (`kontrolem_controllers`, like MpcController — a
  flagged B10 deviation): inverse-dynamics QP over `z = [q̈; λ; τ]` (nz=42 for the
  quad) — dynamics equality `M q̈ + h = Sᵀτ + Jᵀλ`, no-slip contact `J q̈ = −γ`,
  friction **pyramid** + unilateral, torque limits; cost = frame-consistent PD task
  (`q̈_des = −Kp(q ⊖ q_ref) − Kd v`, base-weighted) + force/torque regularization.
  Reuses the `QpSolver`/OSQP seam (added an optional `eps`; WBC uses 1e-4 — a WBC
  solves for forces in newtons, 1e-6 is absurd and left transient ticks
  `SOLVED_INACCURATE`). Accepts the **Regulation** dialect — a quadruped under a
  QP-WBC and a cart-pole under a gain flow through the *same* `compute(state,
  problem, dt)`. **Offline proof** `test_wbc_standing`: closed-loop against the
  model's own `contact_forward_dynamics` (the plant), the WBC **holds the stance to
  1e-6, recovers from a 43 N base push to 1.3e-6** (base pose + twist), feet planted
  to 2e-5, **0 solver hiccups**, torque + friction-pyramid respected. **All 15
  tests green** (5 model + 10 controllers), no regressions.
  - **Part-D QP-RT finding (T2, confirmed then resolved):** at eps=1e-6 the ADMM QP
    returned `SOLVED_INACCURATE` on 27 transient ticks right after the push (none
    during the static hold); the WBC-appropriate eps=1e-4 cleared all of them. The
    status margin measures the **pyramid** cone actually enforced (not a circular
    one — an early metric bug). `difference()` in compute() still allocates — noted
    for the WBC RT-allocation audit (the D4 item, deferred).
- `[x]` **M4.4 — ros2_control wiring, e2e GREEN.** The WBC stands the quadruped
  through the full stack, no Gazebo:
  - **`FloatingContactSimSystem`** (kontrolem_description) — the standing plant:
    integrates `contact_forward_dynamics` (feet pinned = ground, Baumgarte),
    exports the SE(3) base (13 `<gpio>` scalars) + per-foot **contact scalars**
    (=1, ground-truth stance), holds the posture until a controller commands it
    (D11). `urdf/floating_quadruped.ros2_control.urdf` (generated).
  - **`KontrolemController` floating-base path** (behind `base_type: floating`,
    fixed path untouched): claims actuated joints + base `<gpio>` (BaseStateSensor)
    + contact `<gpio>` (ContactSensor), assembles the SE(3) `State` (base by sensor,
    joints by generalized index), builds the standing `Regulation` q_ref by joint
    index; `"wbc"` added to `make_law`.
  - **`config/quad_stand_controllers.yaml` + `quad_stand.launch.py`** (reuse
    `build_sim_launch`); `e2e_smoke.sh` gained an optional target (WBC settles to a
    non-zero nominal, not 0); quad added to `e2e_all.sh`.
  - **Verified live:** controller `active`, `control_law: wbc`, sim "released"
    (actively integrating), `ok: true`, friction **margin 14.5 N**, compute
    **~462 µs/tick** (well inside the 2 ms budget at 500 Hz). **e2e suite ALL PASS
    (5 demos: LQR/LQG/QP + hard 3-DoF + WBC quad).** Scheduled all-stance, no
    locomotion (Part D).

**M4 (standing WBC) is complete.** ONE `KontrolemController` runtime now hosts five
paradigms across fixed and floating base — a cart-pole under a stored gain and a
quadruped under a contact-aware QP flow through the identical `compute(state,
problem, dt)`. The plan's M4 acceptance ("stands + rejects a bounded push") is met:
standing is proven e2e, push-recovery offline (43 N → 1.3e-6). Deferred (Part D):
multi-controller/gait transitions (M4.5), the real Go2, the WBC RT-allocation audit.

---

## Update policy

I propose a step change (in § Proposed step updates) whenever: a dependency or
tool assumption is contradicted by the environment; a design choice made in code
implies a different downstream step; a Part-D risk is confirmed or falsified by a
prototype; or a checkpoint verdict says the interface must change. I state the
reason and wait for your decision before editing the affected step.
