# Reference — Control laws

> **For:** developers. **Assumes:** control-theory literacy. **Scope:** a fact sheet per control paradigm — accepted problem dialects, required state, what each lifecycle phase does, parameters, and the `status()` trust metric. For rationale see [Explanation → Controller contract](../explanation/controller-contract.md); for parameters see [Controller parameters](controller-parameters.md).

All laws implement the `kontrolem_control::Controller` contract: `capabilities()` → `synthesize()` → `configure()` → `compute()` → `status()`. "Heavy phase" is where the expensive math runs.

---

## LQR — `lqr` (`LqrController`)

| | |
|---|---|
| Method | Infinite-horizon LQR; gain from the continuous algebraic Riccati equation (CARE). |
| Accepts | Regulation, Tracking |
| Required state | Full state (position + velocity). |
| Heavy phase | `synthesize()` — solve CARE → gain `K`. |
| `compute()` | `u = u_eq − K (x − x_ref)` (a gemv; allocation-free). |
| `status().ok` | State inside the linearization trust region. |
| `status().margin` | `q_dev_max −` Q-weighted distance from the operating point. |
| Params | `lqr.q_diag`, `lqr.r_diag`, `lqr.q_dev_max` |

## LQG — `lqg` (`LqgController`)

| | |
|---|---|
| Method | Separation principle: LQR gain (control CARE) + steady-state Kalman gain (filter/dual CARE). Dynamic output-feedback compensator. |
| Accepts | Regulation |
| Required state | **Positions only** (`needs_velocity_state = false`); velocity is estimated internally. |
| Heavy phase | `synthesize()` — two Riccati solves. |
| `compute()` | Step the compensator: `x̂ ← A_c x̂ + B_c y`, `u = u_eq + C_c x̂ + D_c y`. |
| `status().ok` | Filter innovation inside a χ²-style gate. |
| `status().margin` | `innov_max −` innovation magnitude. |
| Params | `lqg.q_diag`, `lqg.r_diag`, `lqg.w_diag`, `lqg.v_diag`, `lqg.innov_max` |

## LPV / gain scheduling — `lpv` (`LpvController`)

| | |
|---|---|
| Method | Point-wise design + interpolation (Shamma–Athans): an LQR is synthesized at every node of a regular grid over the scheduling variables, and `(K, u_eq)` is **multilinearly interpolated** on the measured state each tick. |
| Accepts | Regulation, Tracking |
| Required state | Full state. |
| Heavy phase | `synthesize()` — one CARE solve **per grid node** (an `n₁ × n₂ × …` grid). |
| `compute()` | Locate the state in the grid, interpolate `(K, u_eq)` over the `2^D` corners, then `u = u_eq(θ) − K(θ)(x − x_ref)` (allocation-free). |
| `status().ok` | Scheduling variable **inside the designed envelope**. |
| `status().margin` | Signed distance to the nearest envelope bound (negative = outside the grid). |
| Params | `lpv.q_diag`, `lpv.r_diag`, `lpv.sched_joints`, `lpv.sched_min`, `lpv.sched_max`, `lpv.sched_nodes` |
| Note | Each node is locally optimal and interpolation covers between them, but there is **no cross-envelope stability certificate** — that needs an LMI-LPV synthesis, a later upgrade behind the same runtime. Grid cost is `∏ nᵢ` CARE solves, so keep `D` small. |
| Note | LPV is **one continuously-scheduled controller** (an L3 law), not the Supervisor's discrete switch between whole controllers (L4). See [Switch the control law](../how-to/switch-control-law.md) for the latter. |

## Linear MPC — `mpc` (`MpcController`)

| | |
|---|---|
| Method | Condensed receding-horizon QP over the input sequence; exact matrix-exponential (Van Loan) discretization; DARE terminal cost. Solved via the OSQP seam. |
| Accepts | Regulation, Tracking |
| Required state | Full state. |
| Heavy phase | `configure()` — build condensed cost/constraint matrices + terminal cost. |
| `compute()` | Update `x0`, warm-start, solve the QP, apply the first input. |
| `status().ok` | QP solved. |
| `status().margin` | Torque headroom. |
| Params | `mpc.q_diag`, `mpc.r_diag`, `mpc.horizon`, `mpc.dt_mpc`, `mpc.tau_max` |
| Note | On fast-unstable robots keep the horizon short (a long horizon ill-conditions the condensed QP). |

## QP task-space — `qp` (`QpTaskSpaceController`)

| | |
|---|---|
| Method | Online inverse-dynamics QP over `[q̈; τ]`: minimize `‖q̈ − q̈_des‖²_W + ρ‖τ‖²` s.t. `M q̈ + h = Sᵀτ` and `−τ_max ≤ τ ≤ τ_max`; `q̈_des = −kp(q−q_ref) − kd v`. |
| Accepts | Regulation |
| Required state | Full state. |
| Heavy phase | none (`synthesize()` is a no-op). |
| `compute()` | Query dynamics (`M`, `h`), build + solve the QP, output `τ`. |
| `status().ok` | QP solved. |
| `status().margin` | `τ_max −` max applied torque (≈ 0 when the limit is active). |
| Params | `qp.task_weight`, `qp.kp`, `qp.kd`, `qp.tau_max` |
| Suited to | Fully-actuated robots (e.g. arms); does not stabilize the underactuated cart-pole. |

## Whole-body — `wbc` (`WbcController`)

| | |
|---|---|
| Method | Floating-base inverse-dynamics QP over `[q̈; λ; τ]`: dynamics equality `M q̈ + h = Sᵀτ + Jᵀλ`, no-slip contact `J q̈ = −γ`, friction pyramid `|λ_xy| ≤ μλ_z` + `λ_z ≥ 0`, torque limits; cost = frame-consistent tracking task `q̈_des = a_ref − Kp(q ⊖ q_ref) − Kd(v − v_ref)` + force/torque regularization. OSQP seam (tolerance 1e-4). |
| Accepts | Regulation, Tracking, Locomotion |
| Required state | Floating-base state: SE(3) base pose/twist + joint pos/vel (+ contact schedule). |
| Heavy phase | none (`synthesize()` is a no-op). |
| `compute()` | Sample the reference (`q_ref(t)` for Tracking, the fixed setpoint for Regulation), query dynamics + stacked contact Jacobian + contact drift, build + solve the QP, output `τ`. |
| `status().ok` | QP solved. |
| `status().margin` | Smallest friction-pyramid margin `μλ_z − max(|λ_x|,|λ_y|)` across feet (newtons). |
| Params | `wbc.*` (see [Controller parameters](controller-parameters.md)) |
| Scope | Standing / push-recovery, **commanded postures** (squat/sway/tilt/yaw over planted feet; the `Tracking` dialect), and a **static crawl WALK** (feet leave/rejoin the ground; the `Locomotion` dialect carries a `GaitSource` — a swinging foot gets zero force + a swing-arc task, the base tracks the support centroid; `reference_type: gait`; see [How-to → Make the Go2 walk](../how-to/make-the-go2-walk.md)), and a **diagonal TROT** (`reference_type: trot` — the same `Locomotion` path with a `TrotGait`: two diagonal feet in stance, two in swing, the base held nominal and glided forward; a quasi-static trot, offline-gated on two-foot feasibility by `test_wbc_trot`; see [How-to → Make the Go2 trot (whole-body)](../how-to/make-the-go2-trot-whole-body.md)). Truly dynamic gaits with a flight phase (running/jumping) are later milestones. |

## Kinematic gait — `kinematic_gait` (`KinematicGaitController`)

| | |
|---|---|
| Method | **Model-free** walking. No dynamics, no QP, no estimator: sample a gait plan → per-leg Gauss-Newton inverse-kinematics (fixed-size 3×3 leg Jacobian) → joint PD torque `τ = kp(q* − q) − kd q̇`. The IK pins the base to the *scheduled* (nominal, forward-advancing) pose, so it is **open-loop in the base**. The CHAMP lineage as a `Controller` plugin. |
| Accepts | Locomotion |
| Required state | Joint position + velocity only (the base part of the state is unused). |
| Heavy phase | none (`synthesize()` is a no-op). |
| `compute()` | Sample the `GaitSource`, per-leg IK to the foot targets (warm-started, step-clamped, `ik_max_iter` bound), joint PD → `τ`, `tau_max`-clamped. Allocation-free. |
| `status().ok` | Every foot reached its target within ~1 mm (IK converged). |
| `status().margin` | `1e-3 −` worst foot-to-target distance (m). |
| Params | `kin.*`, `gait.*` (see [Controller parameters](controller-parameters.md)) |
| Scope | Point-foot legs with **3 joints per leg** (a standard quadruple like the Go2); a >3-DoF leg needs the documented least-squares IK extension. No force awareness and open-loop heading (it walks forward + upright but slowly veers) — the honest cost of the model-free paradigm. See [How-to → Make the Go2 trot](../how-to/make-the-go2-trot.md). |

---

## Summary

| Law | Dialects | State | Heavy phase | Online solve |
|---|---|---|---|---|
| `lqr` | Regulation, Tracking | full | synthesize (CARE) | no |
| `lqg` | Regulation | positions only | synthesize (2× Riccati) | no |
| `mpc` | Regulation, Tracking | full | configure (condense) | yes (QP) |
| `qp` | Regulation | full | none | yes (QP) |
| `wbc` | Regulation, Tracking, Locomotion | floating base + contacts | none | yes (QP) |
| `kinematic_gait` | Locomotion | joints only (base open-loop) | none | no (per-leg IK) |
