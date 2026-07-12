#include "kontrolem_controllers/wbc_controller.hpp"

#include <cmath>
#include <stdexcept>

namespace kontrolem_controllers
{
namespace
{
constexpr double kBig = 1e19;  // one-sided-bound "infinity" (forces/torques are ~1e1)
}  // namespace

WbcController::WbcController(
  std::vector<std::string> contact_frames, std::vector<std::string> actuated_joints, Gains gains)
: feet_(std::move(contact_frames)), actuated_(std::move(actuated_joints)), g_(gains)
{
}

Capabilities WbcController::capabilities() const
{
  // Regulation (fixed standing posture) + Tracking (a time-varying base-pose reference,
  // e.g. BasePoseReference / LiveBaseTarget — commanded postures over planted feet).
  return Capabilities{{Dialect::kRegulation, Dialect::kTracking}, /*needs_velocity_state=*/true};
}

void WbcController::sample_ref(const ControlProblem & problem, double t)
{
  if (problem.kind() == Dialect::kTracking) {
    static_cast<const Tracking &>(problem).reference->sample(t, qref_, vref_, aref_, tauff_);
  } else {
    const auto & reg = static_cast<const Regulation &>(problem);
    qref_ = reg.q_ref;
    vref_ = reg.v_ref;
    aref_.setZero();
  }
}

std::unique_ptr<Synthesis> WbcController::synthesize(
  const RobotModel &, const ControlProblem &) const
{
  return std::make_unique<Synthesis>();  // NO-OP: nothing precomputed (online QP)
}

void WbcController::configure(
  const RobotModel & model, const Synthesis & /*synthesis*/, const ControlProblem & problem)
{
  model_ = &model;
  nv_ = model.nv();
  nc_ = static_cast<int>(feet_.size());
  m_ = static_cast<int>(actuated_.size());
  const int nl = 3 * nc_;

  // Actuated generalized-velocity rows (the S^T selection) — by joint name, so
  // the free-flyer base rows (0..5) are correctly left unactuated.
  act_v_.clear();
  for (const auto & a : actuated_) {
    act_v_.push_back(model.joint_v_index(a));
  }
  feet_ids_.clear();
  for (const auto & f : feet_) {
    feet_ids_.push_back(model.frame_index(f));  // resolve once; RT queries use the ids
  }

  // Column layout of z = [qddot(nv) | lambda(3nc) | tau(m)].
  off_lambda_ = nv_;
  off_tau_ = nv_ + nl;
  nz_ = nv_ + nl + m_;

  // Row layout: dynamics(nv) | contact(3nc) | friction(4nc) | unilateral(nc) | tau(m).
  row_dyn_ = 0;
  row_con_ = nv_;
  row_fric_ = row_con_ + nl;
  row_uni_ = row_fric_ + 4 * nc_;
  row_tau_ = row_uni_ + nc_;
  rows_ = row_tau_ + m_;

  // Per-DoF PD gains: base DoF (0..5) vs joints.
  Kp_ = Eigen::VectorXd::Constant(nv_, g_.kp_post);
  Kd_ = Eigen::VectorXd::Constant(nv_, g_.kd_post);
  W_ = Eigen::VectorXd::Constant(nv_, g_.w_post);
  for (int i = 0; i < 6 && i < nv_; ++i) {
    Kp_(i) = g_.kp_base;
    Kd_(i) = g_.kd_base;
    W_(i) = g_.w_base;
  }

  // Constant cost P = blkdiag(2W, 2 w_f I, 2 w_t I).
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(nz_, nz_);
  for (int i = 0; i < nv_; ++i) P(i, i) = 2.0 * W_(i);
  for (int i = 0; i < nl; ++i) P(off_lambda_ + i, off_lambda_ + i) = 2.0 * g_.w_force;
  for (int k = 0; k < m_; ++k) P(off_tau_ + k, off_tau_ + k) = 2.0 * g_.w_tau;

  // Constant structure of A and bounds; M/J blocks + equality bounds refresh each tick.
  A_ = Eigen::MatrixXd::Zero(rows_, nz_);
  l_ = Eigen::VectorXd::Zero(rows_);
  u_ = Eigen::VectorXd::Zero(rows_);
  qcost_ = Eigen::VectorXd::Zero(nz_);

  // Dynamics: -S^T on the tau columns (constant).
  for (int k = 0; k < m_; ++k) A_(row_dyn_ + act_v_[static_cast<std::size_t>(k)], off_tau_ + k) = -1.0;

  // Friction pyramid + unilateral (constant): per contact c, force cols [3c,3c+2].
  for (int c = 0; c < nc_; ++c) {
    const int fx = off_lambda_ + 3 * c, fy = fx + 1, fz = fx + 2;
    const int r = row_fric_ + 4 * c;
    A_(r + 0, fx) = 1.0;  A_(r + 0, fz) = -g_.mu;   // lambda_x - mu lambda_z <= 0
    A_(r + 1, fx) = -1.0; A_(r + 1, fz) = -g_.mu;   // -lambda_x - mu lambda_z <= 0
    A_(r + 2, fy) = 1.0;  A_(r + 2, fz) = -g_.mu;   // lambda_y - mu lambda_z <= 0
    A_(r + 3, fy) = -1.0; A_(r + 3, fz) = -g_.mu;   // -lambda_y - mu lambda_z <= 0
    for (int j = 0; j < 4; ++j) { l_(r + j) = -kBig; u_(r + j) = 0.0; }
    A_(row_uni_ + c, fz) = 1.0;                     // lambda_z >= 0
    l_(row_uni_ + c) = 0.0; u_(row_uni_ + c) = kBig;
  }
  // Torque limits (constant).
  for (int k = 0; k < m_; ++k) {
    A_(row_tau_ + k, off_tau_ + k) = 1.0;
    l_(row_tau_ + k) = -g_.tau_max;
    u_(row_tau_ + k) = g_.tau_max;
  }

  // Preallocate the model workspace + per-tick buffers (off the RT path).
  ws_ = std::make_unique<RobotModel::Workspace>(model.make_workspace());
  M_.setZero(nv_, nv_);
  J_.setZero(nl, nv_);
  h_.setZero(nv_);
  gamma_.setZero(nl);
  e_.setZero(nv_);
  qdd_des_.setZero(nv_);
  command_.tau = Eigen::VectorXd::Zero(m_);

  // Reference buffers (q: nq, v/a: nv), then the initial reference (t = 0) to prime with.
  qref_ = model.neutral();
  vref_.setZero(nv_);
  aref_.setZero(nv_);
  tauff_.resize(0);
  sample_ref(problem, 0.0);

  // Prime the QP at the initial reference posture (v = 0) so setup fixes the sparsity.
  const Eigen::VectorXd v0 = Eigen::VectorXd::Zero(nv_);
  model.dynamics(*ws_, qref_, v0, M_, h_);
  model.contact_jacobian_stacked(*ws_, qref_, feet_, J_);
  model.contact_drift(*ws_, qref_, v0, feet_, gamma_);
  A_.block(row_dyn_, 0, nv_, nv_) = M_;
  A_.block(row_dyn_, off_lambda_, nv_, nl).noalias() = -J_.transpose();
  A_.block(row_con_, 0, nl, nv_) = J_;
  l_.segment(row_dyn_, nv_) = -h_;  u_.segment(row_dyn_, nv_) = -h_;
  l_.segment(row_con_, nl) = -gamma_;  u_.segment(row_con_, nl) = -gamma_;
  qp_.setup(P, A_, qcost_, l_, u_, /*eps=*/1e-4, g_.max_iter);  // WBC scale: newtons, not 1e-6
  status_ = Status{true, 0.0};
}

const Command & WbcController::compute(
  const State & state, const ControlProblem & problem, double /*dt*/)
{
  const int nl = 3 * nc_;

  // Reference this tick: a fixed setpoint (Regulation) or the trajectory at state.t
  // (Tracking — a moving base-pose target for commanded postures). Alloc-free.
  sample_ref(problem, state.t);

  // Frame-consistent trajectory-tracking task in the generalized tangent space (so the
  // SE(3) base error is handled correctly):
  //   qddot_des = a_ref - Kp (q ⊖ q_ref) - Kd (v - v_ref).
  // The reference velocity/accel feedforward (v_ref/a_ref, from the TrajectorySource)
  // cancels the lag a pure moving setpoint would incur — without it the -Kd v damping
  // fights the reference velocity. For Regulation, v_ref (≈0) and a_ref (=0) reduce this
  // to the -Kp e - Kd v PD used M4-M8, byte-identical. In-place difference into the
  // preallocated buffer keeps compute() allocation-free.
  model_->difference(qref_, state.q, e_);
  qdd_des_.array() =
    aref_.array() - Kp_.array() * e_.array() - Kd_.array() * (state.v - vref_).array();

  // Instantaneous floating-base dynamics + contact geometry (into preallocated
  // buffers, by cached frame index — no per-tick name lookup or allocation).
  model_->dynamics(*ws_, state.q, state.v, M_, h_);
  model_->contact_jacobian_stacked(*ws_, state.q, feet_ids_, J_);
  model_->contact_drift(*ws_, state.q, state.v, feet_ids_, gamma_);

  // Refresh the tick-varying QP blocks.
  A_.block(row_dyn_, 0, nv_, nv_) = M_;
  A_.block(row_dyn_, off_lambda_, nv_, nl).noalias() = -J_.transpose();
  A_.block(row_con_, 0, nl, nv_) = J_;
  l_.segment(row_dyn_, nv_) = -h_;  u_.segment(row_dyn_, nv_) = -h_;
  l_.segment(row_con_, nl) = -gamma_;  u_.segment(row_con_, nl) = -gamma_;
  for (int i = 0; i < nv_; ++i) qcost_(i) = -2.0 * W_(i) * qdd_des_(i);

  const Eigen::VectorXd & z = qp_.solve(qcost_, l_, u_, A_);
  command_.tau = z.segment(off_tau_, m_);

  // Trust: solved + smallest friction-PYRAMID margin (mu*lambda_z - max|lambda_x,y|),
  // consistent with the box pyramid actually enforced above (not a circular cone).
  status_.ok = qp_.solved();
  double min_margin = kBig;
  for (int c = 0; c < nc_; ++c) {
    const double lx = z(off_lambda_ + 3 * c), ly = z(off_lambda_ + 3 * c + 1),
                 lz = z(off_lambda_ + 3 * c + 2);
    min_margin = std::min(min_margin, g_.mu * lz - std::max(std::abs(lx), std::abs(ly)));
  }
  status_.margin = status_.ok ? min_margin : -1.0;
  status_.iters = qp_.iterations();
  return command_;
}

}  // namespace kontrolem_controllers
