#include "kontrolem_controllers/lqg_controller.hpp"

#include <algorithm>
#include <stdexcept>

#include "kontrolem_controllers/care.hpp"

namespace kontrolem_controllers
{

LqgController::LqgController(
  std::vector<std::string> actuated_joints, Eigen::MatrixXd Q, Eigen::MatrixXd R,
  Eigen::MatrixXd W, Eigen::MatrixXd V, double innov_max)
: actuated_(std::move(actuated_joints)), Q_(std::move(Q)), R_(std::move(R)),
  W_(std::move(W)), V_(std::move(V)), innov_max_(innov_max)
{
}

Capabilities LqgController::capabilities() const
{
  // Output feedback: velocity is NOT required from the state source — it is
  // estimated by the internal Kalman filter. This is the flag the runtime uses
  // to know it may claim positions only.
  return Capabilities{{Dialect::kRegulation}, /*needs_velocity_state=*/false};
}

namespace
{
std::vector<int> actuated_indices(
  const RobotModel & model, const std::vector<std::string> & actuated)
{
  const auto & names = model.joint_names();
  std::vector<int> idx;
  for (const auto & a : actuated) {
    const auto it = std::find(names.begin(), names.end(), a);
    if (it == names.end()) {
      throw std::runtime_error("LqgController: actuated joint '" + a + "' not in model");
    }
    idx.push_back(static_cast<int>(std::distance(names.begin(), it)));
  }
  return idx;
}
}  // namespace

std::unique_ptr<Synthesis> LqgController::synthesize(
  const RobotModel & model, const ControlProblem & problem) const
{
  const auto & reg = static_cast<const Regulation &>(problem);  // accepts() checked upstream
  const int nv = model.nv();
  const int nq = model.nq();
  if (nq != nv) {
    throw std::runtime_error("LqgController slice supports 1-DoF joints only (nq == nv)");
  }
  const int n = 2 * nv;

  const Eigen::VectorXd q_eq = reg.q_ref;
  const Eigen::VectorXd tau_eq = model.gravity_torque(q_eq);
  const auto lin = model.linearize(q_eq, Eigen::VectorXd::Zero(nv), tau_eq);

  const std::vector<int> act_v = actuated_indices(model, actuated_);
  const int m = static_cast<int>(act_v.size());

  // Actuation selection (as LQR): actuated columns of B, actuated rows of u_eq.
  Eigen::MatrixXd B_act(n, m);
  Eigen::VectorXd u_eq(m);
  for (int j = 0; j < m; ++j) {
    B_act.col(j) = lin.B.col(act_v[static_cast<std::size_t>(j)]);
    u_eq(j) = tau_eq(act_v[static_cast<std::size_t>(j)]);
  }

  // Measurement model: positions only. C = [I_nq  0]  (nq x 2nv).
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(nq, n);
  C.leftCols(nq) = Eigen::MatrixXd::Identity(nq, nq);

  // Control gain via the control CARE (identical to LQR).
  const Eigen::MatrixXd K = lqr_gain(lin.A, B_act, Q_, R_);  // m x 2nv

  // Kalman gain via the DUAL/filter CARE: solve for P_f with
  //   A P_f + P_f A^T - P_f C^T V^-1 C P_f + W = 0
  // = solve_care(A^T, C^T, W, V); then L = P_f C^T V^-1.
  const Eigen::MatrixXd Pf = solve_care(lin.A.transpose(), C.transpose(), W_, V_);
  const Eigen::MatrixXd L = Pf * C.transpose() * V_.inverse();  // 2nv x nq

  auto s = std::make_unique<LqgSynthesis>();
  s->A_obs = lin.A - L * C;  // observer dynamics matrix
  s->B_act = B_act;
  s->K = K;
  s->L = L;
  s->u_eq = u_eq;
  s->q_eq = q_eq;
  s->act_v = act_v;
  return s;
}

void LqgController::configure(
  const RobotModel & /*model*/, const Synthesis & synthesis, const ControlProblem & /*problem*/)
{
  const auto & s = static_cast<const LqgSynthesis &>(synthesis);
  A_obs_ = s.A_obs;
  B_act_ = s.B_act;
  K_ = s.K;
  L_ = s.L;
  u_eq_ = s.u_eq;
  q_eq_ = s.q_eq;

  const int n = static_cast<int>(A_obs_.rows());
  const int m = static_cast<int>(K_.rows());
  const int nq = static_cast<int>(L_.cols());
  xhat_.setZero(n);   // start from the operating point (zero deviation estimate)
  ytil_.setZero(nq);
  util_.setZero(m);
  xdot_.setZero(n);
  command_.tau.setZero(m);
  status_ = Status{true, 0.0};
}

void LqgController::seed_from_state(const State & state, const ControlProblem & problem)
{
  const auto & reg = static_cast<const Regulation &>(problem);
  const int nv = static_cast<int>(xhat_.size()) / 2;  // xhat = [q_dev (nv); v_dev (nv)]
  xhat_.head(nv) = state.q - reg.q_ref;
  xhat_.tail(nv) =
    (reg.v_ref.size() == nv) ? (state.v - reg.v_ref).eval() : state.v;
}

const Command & LqgController::compute(
  const State & state, const ControlProblem & problem, double dt)
{
  const auto & reg = static_cast<const Regulation &>(problem);

  // Output feedback: the ONLY measurement used is the position deviation.
  ytil_ = state.q - reg.q_ref;  // == y - y_eq (q_eq == q_ref for regulation)

  // Control from the CURRENT estimate: ũ = -K x̂̃,  u = u_eq + ũ.
  util_.noalias() = -K_ * xhat_;
  command_.tau = u_eq_;
  command_.tau.noalias() += util_;

  // Step the observer (Euler): x̂̃ ← x̂̃ + dt (A_obs x̂̃ + B_act ũ + L ỹ).
  xdot_.noalias() = A_obs_ * xhat_;
  xdot_.noalias() += B_act_ * util_;
  xdot_.noalias() += L_ * ytil_;
  xhat_.noalias() += dt * xdot_;

  // Trust = innovation magnitude (y - C x̂̃) within a bound. Advisory.
  const double innov = (ytil_ - xhat_.head(ytil_.size())).cwiseAbs().maxCoeff();
  status_.margin = innov_max_ - innov;
  status_.ok = status_.margin >= 0.0;
  return command_;
}

}  // namespace kontrolem_controllers
