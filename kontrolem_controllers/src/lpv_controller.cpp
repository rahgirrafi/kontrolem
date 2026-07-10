#include "kontrolem_controllers/lpv_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "kontrolem_controllers/lqr_controller.hpp"

namespace kontrolem_controllers
{

LpvController::LpvController(
  std::vector<std::string> actuated_joints, Eigen::MatrixXd Q, Eigen::MatrixXd R,
  std::vector<SchedAxis> axes)
: actuated_(std::move(actuated_joints)), Q_(std::move(Q)), R_(std::move(R)),
  axes_(std::move(axes))
{
  if (axes_.empty()) throw std::runtime_error("LpvController: need >= 1 scheduling axis");
  for (const auto & a : axes_) {
    if (a.n < 2) throw std::runtime_error("LpvController: each scheduling axis needs n >= 2");
    if (a.max <= a.min) throw std::runtime_error("LpvController: axis max must exceed min");
  }
}

Capabilities LpvController::capabilities() const
{
  return Capabilities{{Dialect::kRegulation, Dialect::kTracking}, /*needs_velocity_state=*/true};
}

namespace
{
/// Base operating point (non-scheduled coordinates come from here): the setpoint
/// (Regulation) or the reference at t=0 (Tracking).
Eigen::VectorXd base_point(const ControlProblem & problem, int nq, int nv)
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

std::unique_ptr<Synthesis> LpvController::synthesize(
  const RobotModel & model, const ControlProblem & problem) const
{
  const int nq = model.nq();
  const int nv = model.nv();
  const Eigen::VectorXd base = base_point(problem, nq, nv);

  int total = 1;
  for (const auto & a : axes_) total *= a.n;

  auto s = std::make_unique<LpvSynthesis>();
  s->axes = axes_;
  s->K.resize(static_cast<std::size_t>(total));
  s->u_eq.resize(static_cast<std::size_t>(total));

  // Design an LQR at every grid node (row-major, last axis fastest). Reuses the
  // tested LqrController synthesis path (linearize about the node + CARE).
  const int D = static_cast<int>(axes_.size());
  for (int lin = 0; lin < total; ++lin) {
    Eigen::VectorXd q_op = base;
    int rem = lin;
    for (int d = D - 1; d >= 0; --d) {
      const auto & ax = axes_[static_cast<std::size_t>(d)];
      const int i = rem % ax.n;
      rem /= ax.n;
      const double step = (ax.max - ax.min) / (ax.n - 1);
      q_op(ax.q_index) = ax.min + i * step;
    }
    LqrController lqr(actuated_, Q_, R_);
    Regulation reg;
    reg.q_ref = q_op;
    reg.v_ref = Eigen::VectorXd::Zero(nv);
    auto * ls = static_cast<LqrSynthesis *>(lqr.synthesize(model, reg).release());
    s->K[static_cast<std::size_t>(lin)] = ls->K;
    s->u_eq[static_cast<std::size_t>(lin)] = ls->u_eq;
    s->act_v = ls->act_v;
    delete ls;
  }
  return s;
}

void LpvController::configure(
  const RobotModel & /*model*/, const Synthesis & synthesis, const ControlProblem & /*problem*/)
{
  const auto & s = static_cast<const LpvSynthesis &>(synthesis);
  axes_ = s.axes;
  K_ = s.K;
  u_eq_ = s.u_eq;
  if (K_.empty()) throw std::runtime_error("LpvController: empty synthesis grid");

  const Eigen::Index m = K_.front().rows();
  const Eigen::Index n2 = K_.front().cols();   // 2nv
  const Eigen::Index nv = n2 / 2;
  const Eigen::Index nq = nv;                  // 1-DoF joints (nq == nv)

  K_interp_.setZero(m, n2);
  u_interp_.setZero(m);
  error_.setZero(n2);
  qref_buf_.setZero(nq);
  vref_buf_.setZero(nv);
  aref_buf_.setZero(nv);
  tauff_buf_.setZero(nv);
  lo_.assign(axes_.size(), 0);
  frac_.assign(axes_.size(), 0.0);
  command_.tau.setZero(m);
  status_ = Status{true, 0.0};
}

const Command & LpvController::compute(
  const State & state, const ControlProblem & problem, double /*dt*/)
{
  const Eigen::Index nq = state.q.size();
  const Eigen::Index nv = state.v.size();

  // error = x - x_ref (setpoint, or the reference sampled now).
  if (problem.kind() == Dialect::kRegulation) {
    const auto & reg = static_cast<const Regulation &>(problem);
    error_.head(nq) = state.q - reg.q_ref;
    error_.tail(nv) = state.v - reg.v_ref;
  } else {
    const auto & trk = static_cast<const Tracking &>(problem);
    trk.reference->sample(state.t, qref_buf_, vref_buf_, aref_buf_, tauff_buf_);
    error_.head(nq) = state.q - qref_buf_;
    error_.tail(nv) = state.v - vref_buf_;
  }

  // Locate the current scheduling variable in the grid: per-axis lower node index +
  // interpolation fraction, and the signed distance to the envelope (the trust margin).
  const int D = static_cast<int>(axes_.size());
  double margin = std::numeric_limits<double>::infinity();
  for (int d = 0; d < D; ++d) {
    const auto & ax = axes_[static_cast<std::size_t>(d)];
    const double sv = state.q(ax.q_index);
    margin = std::min(margin, std::min(sv - ax.min, ax.max - sv));
    const double step = (ax.max - ax.min) / (ax.n - 1);
    double pos = (sv - ax.min) / step;
    pos = std::clamp(pos, 0.0, static_cast<double>(ax.n - 1));
    int lo = static_cast<int>(std::floor(pos));
    if (lo > ax.n - 2) lo = ax.n - 2;
    lo_[static_cast<std::size_t>(d)] = lo;
    frac_[static_cast<std::size_t>(d)] = pos - lo;
  }

  // Multilinear interpolation of (K, u_eq) over the 2^D grid corners (allocation-free:
  // accumulate into preallocated buffers; row-major index, last axis fastest).
  K_interp_.setZero();
  u_interp_.setZero();
  const int corners = 1 << D;
  for (int c = 0; c < corners; ++c) {
    double w = 1.0;
    int idx = 0, stride = 1;
    for (int d = D - 1; d >= 0; --d) {
      const int bit = (c >> d) & 1;
      w *= bit ? frac_[static_cast<std::size_t>(d)] : (1.0 - frac_[static_cast<std::size_t>(d)]);
      idx += (lo_[static_cast<std::size_t>(d)] + bit) * stride;
      stride *= axes_[static_cast<std::size_t>(d)].n;
    }
    if (w != 0.0) {
      K_interp_ += w * K_[static_cast<std::size_t>(idx)];
      u_interp_ += w * u_eq_[static_cast<std::size_t>(idx)];
    }
  }

  // u = u_eq(theta) - K(theta) (x - x_ref).
  command_.tau = u_interp_;
  command_.tau.noalias() -= K_interp_ * error_;

  status_.margin = margin;
  status_.ok = margin >= 0.0;
  return command_;
}

}  // namespace kontrolem_controllers
