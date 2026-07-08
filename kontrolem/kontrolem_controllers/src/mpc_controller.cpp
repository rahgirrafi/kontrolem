#include "kontrolem_controllers/mpc_controller.hpp"

#include <algorithm>
#include <stdexcept>
#include <vector>

#include <unsupported/Eigen/MatrixFunctions>

#include "kontrolem_controllers/care.hpp"

namespace kontrolem_controllers
{

MpcController::MpcController(
  std::vector<std::string> actuated_joints, Eigen::MatrixXd Q, Eigen::MatrixXd R,
  int horizon, double dt_mpc, double tau_max)
: actuated_(std::move(actuated_joints)), Q_(std::move(Q)), R_(std::move(R)),
  horizon_(horizon), dt_mpc_(dt_mpc), tau_max_(tau_max)
{
  if (horizon_ < 1) {
    throw std::runtime_error("MpcController: horizon must be >= 1");
  }
}

Capabilities MpcController::capabilities() const
{
  return Capabilities{{Dialect::kRegulation, Dialect::kTracking}, /*needs_velocity_state=*/true};
}

namespace
{
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

std::vector<int> actuated_indices(
  const RobotModel & model, const std::vector<std::string> & actuated)
{
  const auto & names = model.joint_names();
  std::vector<int> idx;
  for (const auto & a : actuated) {
    const auto it = std::find(names.begin(), names.end(), a);
    if (it == names.end()) {
      throw std::runtime_error("MpcController: actuated joint '" + a + "' not in model");
    }
    idx.push_back(static_cast<int>(std::distance(names.begin(), it)));
  }
  return idx;
}
}  // namespace

std::unique_ptr<Synthesis> MpcController::synthesize(
  const RobotModel & model, const ControlProblem & problem) const
{
  const int nv = model.nv();
  const int nq = model.nq();
  if (nq != nv) {
    throw std::runtime_error("MpcController slice supports 1-DoF joints only (nq == nv)");
  }
  const int n = 2 * nv;

  // Operating point: setpoint (Regulation) or reference at t=0 (Tracking).
  const Eigen::VectorXd q_eq = operating_point(problem, nq, nv);
  const Eigen::VectorXd tau_eq = model.gravity_torque(q_eq);
  const auto lin = model.linearize(q_eq, Eigen::VectorXd::Zero(nv), tau_eq);

  const std::vector<int> act_v = actuated_indices(model, actuated_);
  const int m = static_cast<int>(act_v.size());
  Eigen::MatrixXd B_act(n, m);
  Eigen::VectorXd u_eq(m);
  for (int j = 0; j < m; ++j) {
    B_act.col(j) = lin.B.col(act_v[static_cast<std::size_t>(j)]);
    u_eq(j) = tau_eq(act_v[static_cast<std::size_t>(j)]);
  }

  // EXACT (matrix-exponential) discretization of xdot = A x + B u, robust to
  // stiff unstable modes where Euler (I + A dt) fails — e.g. the fast modes of a
  // double inverted pendulum. Van Loan's block trick:
  //   [[A_d, B_d],[0, I]] = exp([[A, B_act],[0, 0]] * dt).
  Eigen::MatrixXd blk = Eigen::MatrixXd::Zero(n + m, n + m);
  blk.topLeftCorner(n, n) = lin.A;
  blk.topRightCorner(n, m) = B_act;
  const Eigen::MatrixXd expd = (blk * dt_mpc_).exp();
  const Eigen::MatrixXd A_d = expd.topLeftCorner(n, n);
  const Eigen::MatrixXd B_d = expd.topRightCorner(n, m);

  // Discrete-LQR terminal cost (recursive feasibility / infinite-horizon tail).
  const Eigen::MatrixXd P_term = solve_dare(A_d, B_d, Q_, R_);

  const int N = horizon_;
  // Precompute A_d powers 0..N.
  std::vector<Eigen::MatrixXd> Apow(N + 1);
  Apow[0] = Eigen::MatrixXd::Identity(n, n);
  for (int k = 1; k <= N; ++k) {
    Apow[k] = A_d * Apow[k - 1];
  }

  // Condensing: X = Sx x0 + Su U,  X = [x_1..x_N], U = [u_0..u_{N-1}].
  Eigen::MatrixXd Sx = Eigen::MatrixXd::Zero(N * n, n);
  Eigen::MatrixXd Su = Eigen::MatrixXd::Zero(N * n, N * m);
  for (int r = 0; r < N; ++r) {                       // x_{r+1}
    Sx.block(r * n, 0, n, n) = Apow[r + 1];
    for (int c = 0; c <= r; ++c) {                    // u_c
      Su.block(r * n, c * m, n, m) = Apow[r - c] * B_d;
    }
  }

  // Block-diagonal stage weights (terminal block uses P_term).
  Eigen::MatrixXd Qbar = Eigen::MatrixXd::Zero(N * n, N * n);
  Eigen::MatrixXd Rbar = Eigen::MatrixXd::Zero(N * m, N * m);
  for (int k = 0; k < N; ++k) {
    Qbar.block(k * n, k * n, n, n) = (k == N - 1) ? P_term : Q_;
    Rbar.block(k * m, k * m, m, m) = R_;
  }

  auto s = std::make_unique<MpcSynthesis>();
  const Eigen::MatrixXd SuT_Qbar = Su.transpose() * Qbar;   // Nm x Nn
  s->H = 2.0 * (SuT_Qbar * Su + Rbar);   // Nm x Nm
  s->H = 0.5 * (s->H + s->H.transpose());  // symmetrize for the solver
  s->G = 2.0 * SuT_Qbar * Sx;            // Nm x n     (q += G x0_dev)
  s->M_ref = 2.0 * SuT_Qbar;             // Nm x Nn    (q -= M_ref Xref_dev)
  s->u_eq = u_eq;
  s->q_eq = q_eq;
  s->act_v = act_v;
  s->horizon = N;
  s->m = m;
  s->dt_mpc = dt_mpc_;
  s->tau_max = tau_max_;
  return s;
}

void MpcController::configure(
  const RobotModel & /*model*/, const Synthesis & synthesis, const ControlProblem & /*problem*/)
{
  const auto & s = static_cast<const MpcSynthesis &>(synthesis);
  G_ = s.G;
  M_ref_ = s.M_ref;
  u_eq_ = s.u_eq;
  q_eq_ = s.q_eq;
  m_ = s.m;
  horizon_n_ = s.horizon;
  dt_mpc_loaded_ = s.dt_mpc;
  const int Nm = s.horizon * s.m;
  const Eigen::Index n = G_.cols();  // 2nv
  const Eigen::Index nv = n / 2;

  A_qp_ = Eigen::MatrixXd::Identity(Nm, Nm);           // box constraint on U
  l_ = Eigen::VectorXd::Constant(Nm, -s.tau_max);
  u_ = Eigen::VectorXd::Constant(Nm, s.tau_max);
  x0_ = Eigen::VectorXd::Zero(n);
  qbuf_ = Eigen::VectorXd::Zero(Nm);
  xref_ = Eigen::VectorXd::Zero(s.horizon * n);
  qs_ = Eigen::VectorXd::Zero(nv);
  vs_ = Eigen::VectorXd::Zero(nv);
  as_ = Eigen::VectorXd::Zero(nv);
  taus_ = Eigen::VectorXd::Zero(nv);
  command_.tau.setZero(m_);

  qp_ = std::make_unique<QpSolver>();
  qp_->setup(s.H, A_qp_, qbuf_, l_, u_);
  status_ = Status{true, 0.0};
}

const Command & MpcController::compute(
  const State & state, const ControlProblem & problem, double /*dt*/)
{
  const Eigen::Index nq = state.q.size();
  const Eigen::Index nv = state.v.size();
  const Eigen::Index n = 2 * nv;

  // x0 = deviation of the current state from the linearization operating point.
  x0_.head(nq) = state.q - q_eq_;
  x0_.tail(nv) = state.v;                       // v_eq = 0
  qbuf_.noalias() = G_ * x0_;                   // regulation-about-operating-point term

  if (problem.kind() == Dialect::kTracking) {
    // Sample the reference over the horizon (the predictive advantage) and add
    // the tracking gradient  q -= M_ref * Xref_dev.
    const auto & trk = static_cast<const Tracking &>(problem);
    for (int k = 0; k < horizon_n_; ++k) {
      const double tk = state.t + (k + 1) * dt_mpc_loaded_;
      trk.reference->sample(tk, qs_, vs_, as_, taus_);
      xref_.segment(k * n, nq) = qs_ - q_eq_;
      xref_.segment(k * n + nq, nv) = vs_;
    }
    qbuf_.noalias() -= M_ref_ * xref_;
  } else {
    // Regulation: if the setpoint carries a nonzero velocity target, fold it in
    // (deviation from the operating point is [0; v_ref] repeated over the horizon).
    const auto & reg = static_cast<const Regulation &>(problem);
    if (reg.v_ref.size() == nv && reg.v_ref.squaredNorm() > 0.0) {
      for (int k = 0; k < horizon_n_; ++k) {
        xref_.segment(k * n, nq).setZero();
        xref_.segment(k * n + nq, nv) = reg.v_ref;
      }
      qbuf_.noalias() -= M_ref_ * xref_;
    }
  }

  // Warm-solve the condensed QP; apply u_0 (recede).
  const Eigen::VectorXd & U = qp_->solve(qbuf_, l_, u_, A_qp_);

  command_.tau = u_eq_;
  command_.tau += U.head(m_);   // first input of the optimal sequence

  status_.ok = qp_->solved();
  status_.margin = status_.ok ? 1.0 : -1.0;
  return command_;
}

}  // namespace kontrolem_controllers
