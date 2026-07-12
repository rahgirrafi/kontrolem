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
| Accepts | Regulation, Tracking |
| Required state | Floating-base state: SE(3) base pose/twist + joint pos/vel (+ contact schedule). |
| Heavy phase | none (`synthesize()` is a no-op). |
| `compute()` | Sample the reference (`q_ref(t)` for Tracking, the fixed setpoint for Regulation), query dynamics + stacked contact Jacobian + contact drift, build + solve the QP, output `τ`. |
| `status().ok` | QP solved. |
| `status().margin` | Smallest friction-pyramid margin `μλ_z − max(|λ_x|,|λ_y|)` across feet (newtons). |
| Params | `wbc.*` (see [Controller parameters](controller-parameters.md)) |
| Scope | Standing / push-recovery, and **commanded postures** (squat/sway/tilt/yaw over planted feet) via a time-varying base-pose reference (`reference_type: base_pose` / `live` — the `Tracking` dialect; see [How-to → Command the Go2's posture](../how-to/command-a-posture.md)). Scheduled all-stance contact; no locomotion (feet never leave the ground). |

---

## Summary

| Law | Dialects | State | Heavy phase | Online solve |
|---|---|---|---|---|
| `lqr` | Regulation, Tracking | full | synthesize (CARE) | no |
| `lqg` | Regulation | positions only | synthesize (2× Riccati) | no |
| `mpc` | Regulation, Tracking | full | configure (condense) | yes (QP) |
| `qp` | Regulation | full | none | yes (QP) |
| `wbc` | Regulation, Tracking | floating base + contacts | none | yes (QP) |
