#include "kontrolem_controllers/lqr_controller.hpp"

#include <algorithm>
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
  return Capabilities{{Dialect::kRegulation}, /*needs_velocity_state=*/true};
}

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
  const auto & reg = static_cast<const Regulation &>(problem);  // accepts() checked upstream
  const int nv = model.nv();
  if (model.nq() != nv) {
    throw std::runtime_error("LqrController slice supports 1-DoF joints only (nq == nv)");
  }

  const Eigen::VectorXd q_eq = reg.q_ref;                    // operating point = setpoint
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
  error_.setZero(K_.cols());          // 2nv
  command_.tau.setZero(K_.rows());    // m
  status_ = Status{true, 0.0};
}

const Command & LqrController::compute(
  const State & state, const ControlProblem & problem, double /*dt*/)
{
  const auto & reg = static_cast<const Regulation &>(problem);
  const Eigen::Index nq = state.q.size();
  const Eigen::Index nv = state.v.size();

  // error = x - x_ref  with x = [q; v].
  error_.head(nq) = state.q - reg.q_ref;
  error_.tail(nv) = state.v - reg.v_ref;

  // u = u_eq - K * error   (allocation-free gemv).
  command_.tau = u_eq_;
  command_.tau.noalias() -= K_ * error_;

  // Trust = distance to the linearization-validity boundary. Advisory only:
  // the law still outputs; acting on !ok is the Supervisor's job (not built yet).
  const double max_dev = (state.q - q_eq_).cwiseAbs().maxCoeff();
  status_.margin = q_dev_max_ - max_dev;
  status_.ok = status_.margin >= 0.0;
  return command_;
}

}  // namespace kontrolem_controllers
