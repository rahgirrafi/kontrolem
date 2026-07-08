#include "kontrolem_controllers/lqr_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "kontrolem_controllers/care.hpp"

namespace kontrolem_controllers
{

LqrController::LqrController(
  std::vector<std::string> actuated_joints, Eigen::MatrixXd Q, Eigen::MatrixXd R,
  double q_dev_max)
: actuated_(std::move(actuated_joints)), Q_(std::move(Q)), R_(std::move(R)),
  q_dev_max_(q_dev_max)
{
}

Capabilities LqrController::capabilities() const
{
  return Capabilities{{Dialect::kRegulation, Dialect::kTracking}, /*needs_velocity_state=*/true};
}

namespace
{
/// The linearization operating point for a problem: the setpoint (Regulation)
/// or the reference at t=0 (Tracking). The gain is synthesized once about this
/// point; LTV re-linearization along a trajectory is a later (MPC) concern.
Eigen::VectorXd operating_point(const ControlProblem & problem, int nq, int nv)
{
  if (problem.kind() == Dialect::kRegulation) {
    return static_cast<const Regulation &>(problem).q_ref;
  }
  const auto & trk = static_cast<const Tracking &>(problem);
  Eigen::VectorXd q(nq), v(nv), a(nv), tau_ff(nv);
  trk.reference->sample(0.0, q, v, a, tau_ff);
  return q;
}
}  // namespace

namespace
{
/// Actuated velocity-DOF indices. This slice assumes 1-DoF joints (nq == nv),
/// so a joint's index in joint_names() equals its column in B / row in tau.
std::vector<int> actuated_indices(
  const RobotModel & model, const std::vector<std::string> & actuated)
{
  const auto & names = model.joint_names();
  std::vector<int> idx;
  for (const auto & a : actuated) {
    const auto it = std::find(names.begin(), names.end(), a);
    if (it == names.end()) {
      throw std::runtime_error("LqrController: actuated joint '" + a + "' not in model");
    }
    idx.push_back(static_cast<int>(std::distance(names.begin(), it)));
  }
  return idx;
}
}  // namespace

std::unique_ptr<Synthesis> LqrController::synthesize(
  const RobotModel & model, const ControlProblem & problem) const
{
  const int nv = model.nv();
  const int nq = model.nq();
  if (nq != nv) {
    throw std::runtime_error("LqrController slice supports 1-DoF joints only (nq == nv)");
  }

  // Operating point: setpoint (Regulation) or the reference at t=0 (Tracking).
  const Eigen::VectorXd q_eq = operating_point(problem, nq, nv);
  const Eigen::VectorXd tau_eq = model.gravity_torque(q_eq); // equilibrium feedforward g(q_eq)
  const auto lin = model.linearize(q_eq, Eigen::VectorXd::Zero(nv), tau_eq);

  const std::vector<int> act_v = actuated_indices(model, actuated_);
  const int m = static_cast<int>(act_v.size());

  // Actuation selection: pull the actuated columns of B (2nv x m) and the
  // actuated rows of the equilibrium torque (u_eq).
  Eigen::MatrixXd B_act(2 * nv, m);
  Eigen::VectorXd u_eq(m);
  for (int j = 0; j < m; ++j) {
    B_act.col(j) = lin.B.col(act_v[static_cast<std::size_t>(j)]);
    u_eq(j) = tau_eq(act_v[static_cast<std::size_t>(j)]);
  }

  auto s = std::make_unique<LqrSynthesis>();
  s->K = lqr_gain(lin.A, B_act, Q_, R_);  // m x 2nv
  s->u_eq = u_eq;
  s->q_eq = q_eq;
  s->act_v = act_v;
  return s;
}

void LqrController::configure(
  const RobotModel & /*model*/, const Synthesis & synthesis, const ControlProblem & /*problem*/)
{
  const auto & s = static_cast<const LqrSynthesis &>(synthesis);
  K_ = s.K;
  u_eq_ = s.u_eq;
  q_eq_ = s.q_eq;
  const Eigen::Index n = K_.cols();    // 2nv
  const Eigen::Index nq = q_eq_.size();
  const Eigen::Index nv = n - nq;
  error_.setZero(n);
  qref_buf_.setZero(nq);
  vref_buf_.setZero(nv);
  aref_buf_.setZero(nv);   // received from sample(), not used by this feedback law
  tauff_buf_.setZero(nv);
  dev_.setZero(n);
  qdev_.setZero(n);
  qtrace_ = Q_.trace() > 0.0 ? Q_.trace() : 1.0;
  command_.tau.setZero(K_.rows());     // m
  status_ = Status{true, 0.0};
}

const Command & LqrController::compute(
  const State & state, const ControlProblem & problem, double /*dt*/)
{
  const Eigen::Index nq = state.q.size();
  const Eigen::Index nv = state.v.size();

  // error = x - x_ref  with x = [q; v]. x_ref is the fixed setpoint (Regulation)
  // or the reference sampled at the current time (Tracking) — same gain law.
  if (problem.kind() == Dialect::kRegulation) {
    const auto & reg = static_cast<const Regulation &>(problem);
    error_.head(nq) = state.q - reg.q_ref;
    error_.tail(nv) = state.v - reg.v_ref;
  } else {  // kTracking (the only other accepted dialect)
    const auto & trk = static_cast<const Tracking &>(problem);
    trk.reference->sample(state.t, qref_buf_, vref_buf_, aref_buf_, tauff_buf_);
    error_.head(nq) = state.q - qref_buf_;
    error_.tail(nv) = state.v - vref_buf_;
  }

  // u = u_eq - K * error   (allocation-free gemv).
  command_.tau = u_eq_;
  command_.tau.noalias() -= K_ * error_;

  // Trust = distance from the linearization operating point, weighted by Q. Using
  // Q (which already down-weights states the design cares little about, e.g. the
  // cart-pole's translation-invariant cart position) fixes the old box metric's
  // flaw of counting cart travel toward a region violation. dev = x - x_eq
  // (v_eq = 0); distance = sqrt(devᵀ Q dev / trace(Q)); allocation-free.
  dev_.head(nq) = state.q - q_eq_;
  dev_.tail(nv) = state.v;
  qdev_.noalias() = Q_ * dev_;
  const double dist = std::sqrt(dev_.dot(qdev_) / qtrace_);
  status_.margin = q_dev_max_ - dist;
  status_.ok = status_.margin >= 0.0;
  return command_;
}

}  // namespace kontrolem_controllers
