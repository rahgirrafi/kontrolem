#include "kontrolem_controllers/qp_task_space_controller.hpp"

#include <algorithm>
#include <stdexcept>

namespace kontrolem_controllers
{

QpTaskSpaceController::QpTaskSpaceController(
  std::vector<std::string> actuated_joints, Eigen::VectorXd task_weight, double kp, double kd,
  double tau_max, double tau_reg)
: actuated_(std::move(actuated_joints)), W_(std::move(task_weight)), kp_(kp), kd_(kd),
  tau_max_(tau_max), tau_reg_(tau_reg)
{
}

Capabilities QpTaskSpaceController::capabilities() const
{
  return Capabilities{{Dialect::kRegulation}, /*needs_velocity_state=*/true};
}

std::unique_ptr<Synthesis> QpTaskSpaceController::synthesize(
  const RobotModel &, const ControlProblem &) const
{
  return std::make_unique<Synthesis>();  // NO-OP: nothing to precompute
}

void QpTaskSpaceController::configure(
  const RobotModel & model, const Synthesis & /*synthesis*/, const ControlProblem & problem)
{
  const auto & reg = static_cast<const Regulation &>(problem);
  model_ = &model;
  nv_ = model.nv();
  if (model.nq() != nv_) {
    throw std::runtime_error("QpTaskSpaceController slice supports 1-DoF joints only (nq == nv)");
  }

  // Resolve actuated velocity-DOF indices (1-DoF joints: index in joint_names).
  const auto & names = model.joint_names();
  act_v_.clear();
  for (const auto & a : actuated_) {
    const auto it = std::find(names.begin(), names.end(), a);
    if (it == names.end()) {
      throw std::runtime_error("QpTaskSpaceController: actuated joint '" + a + "' not in model");
    }
    act_v_.push_back(static_cast<int>(std::distance(names.begin(), it)));
  }
  m_ = static_cast<int>(act_v_.size());
  nz_ = nv_ + m_;              // [qddot; tau]
  nc_ = nv_ + m_;             // nv dynamics equalities + m torque limits

  // Constant cost P = blkdiag(2W, 2 rho I).
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(nz_, nz_);
  for (int i = 0; i < nv_; ++i) {
    P(i, i) = 2.0 * W_(i);
  }
  for (int k = 0; k < m_; ++k) {
    P(nv_ + k, nv_ + k) = 2.0 * tau_reg_;
  }

  // Constant structure of A and bounds; the M-block and the equality bounds are
  // refreshed each tick in compute().
  A_ = Eigen::MatrixXd::Zero(nc_, nz_);
  for (int k = 0; k < m_; ++k) {
    A_(act_v_[static_cast<std::size_t>(k)], nv_ + k) = -1.0;  // -S^T (dynamics)
    A_(nv_ + k, nv_ + k) = 1.0;                               // torque-limit row
  }
  qcost_ = Eigen::VectorXd::Zero(nz_);
  l_ = Eigen::VectorXd::Zero(nc_);
  u_ = Eigen::VectorXd::Zero(nc_);
  for (int k = 0; k < m_; ++k) {
    l_(nv_ + k) = -tau_max_;
    u_(nv_ + k) = tau_max_;
  }
  qdd_des_ = Eigen::VectorXd::Zero(nv_);
  command_.tau = Eigen::VectorXd::Zero(m_);

  // Reusable model workspace + dynamics-output buffers (U4 fix): allocated once
  // here so compute() never allocates.
  ws_ = std::make_unique<RobotModel::Workspace>(model.make_workspace());
  M_.setZero(nv_, nv_);
  h_.setZero(nv_);

  // Initial dynamics at the setpoint to prime the QP (off the RT path).
  model.dynamics(*ws_, reg.q_ref, Eigen::VectorXd::Zero(nv_), M_, h_);
  A_.topLeftCorner(nv_, nv_) = M_;
  l_.head(nv_) = -h_;
  u_.head(nv_) = -h_;
  qp_.setup(P, A_, qcost_, l_, u_, /*eps=*/1e-6, max_iter_);
  status_ = Status{true, tau_max_};
}

const Command & QpTaskSpaceController::compute(
  const State & state, const ControlProblem & problem, double /*dt*/)
{
  const auto & reg = static_cast<const Regulation &>(problem);

  // PD task: desired generalized acceleration (no allocation — into buffer).
  qdd_des_.noalias() = -kp_ * (state.q - reg.q_ref);
  qdd_des_.noalias() -= kd_ * (state.v - reg.v_ref);

  // Instantaneous dynamics via the reusable workspace -> allocation-free (U4 fix).
  model_->dynamics(*ws_, state.q, state.v, M_, h_);

  // Refresh the tick-varying parts of the QP (into preallocated buffers).
  A_.topLeftCorner(nv_, nv_) = M_;                 // dynamics equality: M
  l_.head(nv_) = -h_;                              // = -h
  u_.head(nv_) = -h_;
  for (int i = 0; i < nv_; ++i) {
    qcost_(i) = -2.0 * W_(i) * qdd_des_(i);        // linear cost for qddot block
  }

  const Eigen::VectorXd & z = qp_.solve(qcost_, l_, u_, A_);  // OSQP update + solve
  command_.tau = z.segment(nv_, m_);               // actuated torque

  status_.ok = qp_.solved();
  status_.margin = tau_max_ - command_.tau.cwiseAbs().maxCoeff();  // ~0 when the limit is active
  status_.iters = qp_.iterations();
  return command_;
}

}  // namespace kontrolem_controllers
