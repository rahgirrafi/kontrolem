# Reference — Core C++ API (Layers 1–3)

> **For:** developers extending the ROS-free core. **Assumes:** C++ and Eigen. **Scope:** the public types and signatures of `kontrolem_model` (L1), `kontrolem_control` (L2/L3 contract). These headers depend only on Eigen — no ROS, no Pinocchio leaks. Semantics are one line each; for rationale see [Explanation → Architecture](../explanation/architecture.md).

Namespaces: `kontrolem_model`, `kontrolem_control`. All vectors/matrices are `Eigen::VectorXd`/`Eigen::MatrixXd`.

---

## `kontrolem_model::RobotModel` (L1)

The queryable dynamics service, built from a URDF and backed by Pinocchio.

### Construction & dims

| Signature | Semantics |
|---|---|
| `static RobotModel from_urdf_file(const std::string& path, BaseType base = kFixed)` | Build from a URDF file. |
| `static RobotModel from_urdf_string(const std::string& xml, BaseType base = kFixed)` | Build from URDF XML. |
| `enum class BaseType { kFixed, kFloating }` | `kFloating` adds a free-flyer root (`nq == nv + 1`, SE(3) config). |
| `int nq() const` / `int nv() const` | Configuration / velocity dimensions. |
| `const std::vector<std::string>& joint_names() const` | Movable joint names (includes the free-flyer root for a floating base). |
| `int joint_q_index(const std::string& name) const` | Index into `q` of a named joint's first DoF. |
| `int joint_v_index(const std::string& name) const` | Index into `v`/`q̈` of a named joint's first DoF. |

### Manifold math

| Signature | Semantics |
|---|---|
| `VectorXd neutral() const` | Neutral configuration (identity SE(3) root; unit quaternion). |
| `VectorXd integrate(const VectorXd& q, const VectorXd& v, double dt) const` | `q ⊕ (v·dt)`, manifold-correct (SE(3) exp on the root). |
| `VectorXd difference(const VectorXd& q0, const VectorXd& q1) const` | Tangent `d` with `integrate(q0,d) == q1` (SE(3) log). |

### Dynamics queries

| Signature | Semantics |
|---|---|
| `struct Linearization { MatrixXd A, B; }` | Continuous linearization of `ẋ = A x + B τ`, `x = [q; v]`. |
| `Linearization linearize(q, v, tau) const` | Analytic linearization about the operating point (ABA derivatives). |
| `struct Dynamics { MatrixXd M; VectorXd h; }` | Inertia `M` and bias `h = C v + g`. |
| `Dynamics dynamics(q, v) const` | Allocating dynamics query (offline/tests). |
| `void dynamics(Workspace&, q, v, MatrixXd& M_out, VectorXd& h_out) const` | Real-time, allocation-free dynamics into caller buffers. |
| `VectorXd aba(q, v, tau) const` | Forward dynamics `q̈ = ABA(q,v,τ)`. |
| `VectorXd gravity_torque(q) const` | `g(q) = rnea(q, 0, 0)` (holding torque at rest). |

### Prediction (MPC)

| Signature | Semantics |
|---|---|
| `struct Trajectory { std::vector<VectorXd> q, v; }` | A predicted state trajectory. |
| `Trajectory rollout(q0, v0, const std::vector<VectorXd>& tau_seq, double dt) const` | Semi-implicit Euler rollout under an input sequence. |
| `std::vector<Linearization> linearize_along(qs, vs, taus) const` | Per-step `{A_k, B_k}` along a trajectory (LTV). |

### Contacts & frames (WBC)

| Signature | Semantics |
|---|---|
| `Vector3d center_of_mass(q) const` | World-frame CoM. |
| `Vector3d frame_position(q, const std::string& frame) const` | World position of a named frame. |
| `MatrixXd contact_jacobian(q, frame) const` | Translational (3×nv) Jacobian of one frame, world-aligned. |
| `void contact_jacobian_stacked(Workspace&, q, feet, MatrixXd& J_out) const` | Stacked (3·nc × nv) contact Jacobian, RT. |
| `void contact_drift(Workspace&, q, v, feet, VectorXd& gamma_out) const` | Contact drift `γ = J̇·v` (3·nc), RT. |
| `VectorXd contact_forward_dynamics(Workspace&, q, v, tau, feet, anchors, kp, kd, VectorXd* lambda_out = nullptr) const` | Constrained forward dynamics (feet pinned) via a damped Schur complement; optional Baumgarte to `anchors`. Returns `q̈`; writes contact forces `λ` if requested. |

### Workspace

| Signature | Semantics |
|---|---|
| `class Workspace` | Preallocated per-caller scratch (a Pinocchio `Data`), so RT queries don't allocate. |
| `Workspace make_workspace() const` | Allocate a Workspace sized for this model (call once, off the RT path). |

---

## `kontrolem_control` types (L2)

| Type | Fields / signature | Semantics |
|---|---|---|
| `struct State` | `VectorXd q; VectorXd v; double t;` | Configuration, velocity, controller clock. |
| `struct Command` | `VectorXd tau;` | Generalized effort on the actuated joints. |
| `struct Status` | `bool ok; double margin;` | Trust of the last `compute()`; `margin ≥ 0` means trustworthy. |
| `enum class Dialect` | `kRegulation, kTracking, kTaskSpec` | Problem dialects. |
| `struct ControlProblem` | `virtual Dialect kind() const` | Base of all problems. |
| `struct Regulation : ControlProblem` | `VectorXd q_ref, v_ref;` | Fixed setpoint. |
| `struct Tracking : ControlProblem` | `const TrajectorySource* reference;` | Time-varying reference. |

### `TrajectorySource` (the Tracking seam)

| Type | Signature | Semantics |
|---|---|---|
| `struct TrajectorySource` | `virtual void sample(double t, VectorXd& q_out, VectorXd& v_out, VectorXd& a_out, VectorXd& tau_ff_out) const` | Fill desired config/vel/accel/feedforward at time `t` into caller buffers; must not allocate. |
| `struct ConstantReference` | fields `q, v, tau_ff` | Degenerate constant setpoint. |
| `struct HarmonicReference` | fields `center, amp, phase, omega` | `q_i(t) = center_i + amp_i cos(ωt + phase_i)` with consistent `v`, `a`. |

---

## `kontrolem_control::Controller` (L3 contract)

| Member | Signature | Semantics |
|---|---|---|
| `capabilities()` | `Capabilities capabilities() const` | Accepted dialects + `needs_velocity_state`. |
| `synthesize()` | `std::unique_ptr<Synthesis> synthesize(const RobotModel&, const ControlProblem&) const` | Offline heavy math → serializable artifact (may be trivial). |
| `configure()` | `void configure(const RobotModel&, const Synthesis&, const ControlProblem&)` | Load the artifact, allocate buffers/solver. Not real-time. |
| `compute()` | `const Command& compute(const State&, const ControlProblem&, double dt)` | Per-tick, allocation-free; returns an internally-owned `Command` (valid until the next `compute()`). |
| `status()` | `const Status& status() const` | Trust/health for the supervisor. |

| Helper | Signature | Semantics |
|---|---|---|
| `struct Capabilities` | `std::vector<Dialect> accepted_dialects; bool needs_velocity_state;` | What the runtime must supply. |
| `struct Synthesis` | polymorphic base | The offline artifact; a no-synthesis controller returns the base instance. |
| `bool accepts(const Controller&, const ControlProblem&)` | free function | Wiring-time dialect check. |

---

## Layer-4 semantic components (`kontrolem_ros2_control`)

These do depend on ros2_control (they live in L4), listed here for completeness.

| Type | Semantics |
|---|---|
| `BaseStateSensor(gpio_name)` | Owns the 13-scalar SE(3) base `<gpio>` convention; `read_into(q, v)` reassembles the root slots (quaternion normalized). |
| `ContactSensor(gpio_name, feet)` | Owns the per-foot `contact.<foot>` `<gpio>` convention; `read_into(out)` / `stance_into(mask)`. |

Interface naming is in [ros2_control interfaces](ros2control-interfaces.md).
