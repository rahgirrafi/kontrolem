// Kontrol'Em v2 — Layer 1 model service (minimal slice).
//
// Wraps Pinocchio just enough to answer ONE query for the interface test:
//   linearize(q, v, tau) -> (A, B)
// the continuous-time Jacobians of xdot = f(x, tau) about (q, v, tau), with
// state x = [q; v] and input tau the generalized joint torques.
//
// INVARIANT: this header is Eigen-only. Pinocchio lives behind a pImpl in the
// .cpp so nothing downstream (the controller contract, the controllers) ever
// sees a Pinocchio or ROS type. No rclcpp, no ros2_control anywhere in core.
#ifndef KONTROLEM_MODEL__ROBOT_MODEL_HPP_
#define KONTROLEM_MODEL__ROBOT_MODEL_HPP_

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

namespace kontrolem_model
{

/// Continuous-time linearization of the rigid-body dynamics about an
/// operating point. x = [q; v] in R^{2nv}, input = generalized torque in R^{nv}.
///   xdot = A x + B tau   (to first order about the point)
struct Linearization
{
  Eigen::MatrixXd A;  ///< 2nv x 2nv
  Eigen::MatrixXd B;  ///< 2nv x nv
};

/// Instantaneous nonlinear dynamics terms, M(q) qddot + h(q,v) = tau:
///   M = joint-space inertia (CRBA), h = C(q,v) v + g(q) = rnea(q, v, 0).
/// This is the query an online (QP/WBC) controller makes every tick.
struct Dynamics
{
  Eigen::MatrixXd M;  ///< nv x nv (symmetric)
  Eigen::VectorXd h;  ///< nv (nonlinear bias: Coriolis/centrifugal + gravity)
};

/// Queryable rigid-body model built from a URDF. This slice exposes only what
/// the LQR / QP interface test needs; rollout, contact Jacobians, floating base
/// etc. are deliberately absent (see the v2 plan, Part A Layer 1).
/// Root-joint kind. Fixed base keeps nq == nv (Euclidean). A floating base adds a
/// free-flyer root: the configuration gains a 7-DoF SE(3) pose (3 translation + 4
/// quaternion) and the velocity a 6-DoF spatial twist, so nq == nv + 1 and the
/// configuration lives on a manifold — see integrate().
enum class BaseType
{
  kFixed,
  kFloating,
};

class RobotModel
{
public:
  /// Build a model from a URDF file. `base` selects fixed or floating base.
  /// Throws std::runtime_error on parse failure.
  static RobotModel from_urdf_file(const std::string & path, BaseType base = BaseType::kFixed);

  /// Build a model from a URDF XML string (e.g. the ros2_control
  /// `robot_description`). Throws std::runtime_error on parse failure.
  static RobotModel from_urdf_string(
    const std::string & urdf_xml, BaseType base = BaseType::kFixed);

  /// Analytic continuous-time linearization about (q, v, tau) via Pinocchio's
  /// computeABADerivatives (not finite differences). Precondition: q.size()==nq,
  /// v.size()==nv, tau.size()==nv.
  Linearization linearize(
    const Eigen::VectorXd & q, const Eigen::VectorXd & v, const Eigen::VectorXd & tau) const;

  /// A predicted state trajectory: q[k], v[k] for k = 0..N (N+1 entries).
  struct Trajectory
  {
    std::vector<Eigen::VectorXd> q;
    std::vector<Eigen::VectorXd> v;
  };

  /// Roll the dynamics forward from (q0, v0) under the input sequence
  /// `tau_seq` (N entries, each nv), one semi-implicit-Euler step of `dt` per
  /// input, returning the N+1 states. Manifold-correct (uses integrate()).
  /// An offline/synthesis query (allocates) — the prediction model for MPC's
  /// `rollout` + `linearize_along` and for trajectory optimization.
  Trajectory rollout(
    const Eigen::VectorXd & q0, const Eigen::VectorXd & v0,
    const std::vector<Eigen::VectorXd> & tau_seq, double dt) const;

  /// Linearize the continuous dynamics at each (q[k], v[k], tau[k]) along a
  /// trajectory, giving the per-step {A_k, B_k} of a time-varying (LTV)
  /// prediction model. `q`, `v`, `tau` must have equal length. Offline.
  std::vector<Linearization> linearize_along(
    const std::vector<Eigen::VectorXd> & q, const std::vector<Eigen::VectorXd> & v,
    const std::vector<Eigen::VectorXd> & tau) const;

  /// Forward dynamics a = ABA(q, v, tau). Exposed so the test can build
  /// f(x,tau) = [v; a] and finite-difference it against linearize().
  Eigen::VectorXd aba(
    const Eigen::VectorXd & q, const Eigen::VectorXd & v, const Eigen::VectorXd & tau) const;

  /// Generalized gravity torque g(q) = rnea(q, v=0, a=0). At an equilibrium
  /// (v = 0) this is the feedforward torque that holds the pose — i.e. LQR's
  /// operating-point u_eq (its actuated part).
  Eigen::VectorXd gravity_torque(const Eigen::VectorXd & q) const;

  /// Convenience (ALLOCATING) dynamics query — a fresh Pinocchio Data per call.
  /// Fine offline (synthesis/tests); NOT for the real-time path. For per-tick
  /// use, take a Workspace + output buffers overload below.
  Dynamics dynamics(const Eigen::VectorXd & q, const Eigen::VectorXd & v) const;

  /// Reusable per-caller scratch (a preallocated Pinocchio Data). Each
  /// controller owns one, so queries are re-entrant (no shared mutable state)
  /// and allocation-free after construction. Opaque: Pinocchio stays in the .cpp.
  class Workspace
  {
  public:
    Workspace(Workspace &&) noexcept;
    Workspace & operator=(Workspace &&) noexcept;
    ~Workspace();

  private:
    friend class RobotModel;
    Workspace();
    struct Impl;
    std::unique_ptr<Impl> impl_;
  };

  /// Allocate a Workspace sized for this model (call once, off the RT path).
  Workspace make_workspace() const;

  /// Real-time dynamics query: reuses `ws` and writes into the caller's
  /// preallocated `M_out` (nv x nv) and `h_out` (nv). Allocation-free — this is
  /// the U4 fix; the QP controller calls this in compute().
  void dynamics(
    Workspace & ws, const Eigen::VectorXd & q, const Eigen::VectorXd & v,
    Eigen::MatrixXd & M_out, Eigen::VectorXd & h_out) const;

  /// Center of mass position (world frame) at configuration q. A configuration-
  /// dependent scalar/vector used to validate the floating-base model against a
  /// known quantity (plan M3), and the basis of CoM tasks for the WBC.
  Eigen::Vector3d center_of_mass(const Eigen::VectorXd & q) const;

  /// World-frame position of a named frame (e.g. a foot / contact point) at q.
  /// Throws std::runtime_error if the frame does not exist.
  Eigen::Vector3d frame_position(const Eigen::VectorXd & q, const std::string & frame) const;

  /// Translational contact Jacobian (3 x nv) of a named frame, world-aligned:
  ///   v_world = J · v_generalized.
  /// This is the map a QP-WBC needs to express foot no-slip / friction-cone
  /// constraints and contact-force terms Jᵀλ. Throws if the frame is absent.
  Eigen::MatrixXd contact_jacobian(const Eigen::VectorXd & q, const std::string & frame) const;

  /// Real-time stacked translational contact Jacobian (3·nc × nv) for the named
  /// contact frames, world-aligned, using the caller's Workspace. Row block k is
  /// the 3×nv Jacobian of feet[k]. Writes into the caller's `J_out` (resized if
  /// needed). Throws if any frame is absent.
  void contact_jacobian_stacked(
    Workspace & ws, const Eigen::VectorXd & q, const std::vector<std::string> & feet,
    Eigen::MatrixXd & J_out) const;

  /// Real-time stacked contact "drift" γ = d/dt(J)·v (3·nc) — the frame
  /// classical acceleration at zero joint acceleration. The WBC's no-slip
  /// contact constraint is J·q̈ = −γ; a contact-constrained simulator uses the
  /// same term. Writes into the caller's `gamma_out`.
  void contact_drift(
    Workspace & ws, const Eigen::VectorXd & q, const Eigen::VectorXd & v,
    const std::vector<std::string> & feet, Eigen::VectorXd & gamma_out) const;

  /// Contact-constrained forward dynamics: the acceleration q̈ that keeps the
  /// named feet fixed (point contacts, no slip) under generalized force `tau`,
  /// with optional Baumgarte position/velocity stabilization toward `anchors`
  /// (empty ⇒ pure acceleration-level constraint). Solves the KKT system
  ///   [M −Jᵀ; J 0][q̈; λ] = [tau−h; −γ − Baumgarte]
  /// via the (damped) Schur complement — so redundant contacts (a rigid body on
  /// >2 feet) stay well-posed. This is the "ground" a floating-base standing
  /// simulator needs; without it the base free-falls under aba(). If
  /// `lambda_out` is non-null it receives the 3·nc world-aligned contact forces.
  Eigen::VectorXd contact_forward_dynamics(
    Workspace & ws, const Eigen::VectorXd & q, const Eigen::VectorXd & v,
    const Eigen::VectorXd & tau, const std::vector<std::string> & feet,
    const std::vector<Eigen::Vector3d> & anchors, double baumgarte_kp, double baumgarte_kd,
    Eigen::VectorXd * lambda_out = nullptr) const;

  /// Manifold-correct configuration update: q_next = q ⊕ (v · dt). For a fixed
  /// base this is q + v·dt; for a floating base it integrates the SE(3) root on
  /// its group (quaternion stays unit), which a naive q + v·dt would corrupt.
  Eigen::VectorXd integrate(
    const Eigen::VectorXd & q, const Eigen::VectorXd & v, double dt) const;

  /// Neutral configuration (identity SE(3) root for a floating base, zeros for a
  /// fixed base). Correct starting point since a zero vector is NOT a valid
  /// floating-base q (the quaternion would be zero, not unit).
  Eigen::VectorXd neutral() const;

  /// Manifold difference: the tangent vector d (nv) such that integrate(q0, d)==q1
  /// — i.e. "q1 ⊖ q0". For a floating base this is the SE(3) log of the relative
  /// pose (not q1 − q0), so it lives in the same tangent space as v and q̈. This
  /// is how a WBC forms a frame-consistent configuration error for its PD task.
  Eigen::VectorXd difference(const Eigen::VectorXd & q0, const Eigen::VectorXd & q1) const;

  int nq() const;
  int nv() const;
  const std::vector<std::string> & joint_names() const;

  /// Index into the configuration vector q of a named joint's first DoF (the
  /// tangent/velocity index for v). Throws if the joint is absent. Lets a
  /// controller map actuated joint names to generalized-force rows (the
  /// actuation selection Sᵀ) and address specific joints in q/v.
  int joint_q_index(const std::string & name) const;
  int joint_v_index(const std::string & name) const;

private:
  RobotModel();
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace kontrolem_model

#endif  // KONTROLEM_MODEL__ROBOT_MODEL_HPP_
