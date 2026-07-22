#include "kontrolem_controllers/kinematic_gait_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace kontrolem_controllers
{

KinematicGaitController::KinematicGaitController(
  std::vector<std::string> contact_frames, std::vector<std::string> actuated_joints, Gains gains)
: feet_(std::move(contact_frames)), actuated_(std::move(actuated_joints)), g_(gains)
{
}

Capabilities KinematicGaitController::capabilities() const
{
  // A dedicated walking controller: it only makes sense on a gait plan.
  return Capabilities{{Dialect::kLocomotion}, /*needs_velocity_state=*/true};
}

std::unique_ptr<Synthesis> KinematicGaitController::synthesize(
  const RobotModel &, const ControlProblem &) const
{
  return std::make_unique<Synthesis>();  // model-free: nothing precomputed
}

void KinematicGaitController::configure(
  const RobotModel & model, const Synthesis & /*synthesis*/, const ControlProblem & problem)
{
  model_ = &model;
  nq_ = model.nq();
  nv_ = model.nv();
  nc_ = static_cast<int>(feet_.size());
  m_ = static_cast<int>(actuated_.size());
  if (nc_ <= 0 || m_ % nc_ != 0) {
    throw std::runtime_error(
      "KinematicGaitController: actuated joints must divide evenly among contact feet");
  }
  jpl_ = m_ / nc_;
  if (jpl_ != 3) {
    throw std::runtime_error(
      "KinematicGaitController: assumes 3 joints per leg (point-foot quadruped); a >3-DoF leg "
      "needs the documented least-squares IK extension");
  }

  feet_ids_.clear();
  for (const auto & f : feet_) feet_ids_.push_back(model.frame_index(f));
  act_q_.clear();
  act_v_.clear();
  for (const auto & a : actuated_) {
    act_q_.push_back(model.joint_q_index(a));
    act_v_.push_back(model.joint_v_index(a));
  }

  ws_ = std::make_unique<RobotModel::Workspace>(model.make_workspace());
  plan_.resize(static_cast<std::size_t>(nc_), nq_, nv_);
  command_.tau = Eigen::VectorXd::Zero(m_);
  J1_.setZero(3, nv_);
  one_fid_.assign(1, 0);

  // Seed the warm-start IK iterate at the gait's t=0 posture (nominal joints + base).
  if (problem.kind() == Dialect::kLocomotion) {
    static_cast<const Locomotion &>(problem).gait->sample(0.0, plan_);
    q_ik_ = plan_.q_ref;
  } else {
    q_ik_ = model.neutral();
  }
  status_ = Status{true, 0.0, 0};
}

void KinematicGaitController::on_activate(const State & /*state*/, const ControlProblem & problem)
{
  // Restart the warm-start seed from the gait's nominal posture so activation begins from a
  // sane iterate (the law is otherwise open-loop, so there is no observer state to converge).
  if (problem.kind() == Dialect::kLocomotion) {
    static_cast<const Locomotion &>(problem).gait->sample(0.0, plan_);
    q_ik_ = plan_.q_ref;
  }
}

const Command & KinematicGaitController::compute(
  const State & state, const ControlProblem & problem, double /*dt*/)
{
  // 1) Sample the gait: per-foot WORLD targets + the (nominal, forward-advancing) base pose.
  static_cast<const Locomotion &>(problem).gait->sample(state.t, plan_);

  // 2) Pin the IK base to the SCHEDULED base — open-loop, no estimator. Keep the joint iterate
  //    warm from last tick for fast, continuous convergence (base occupies q[0..6]).
  q_ik_.head(7) = plan_.q_ref.head(7);

  // 3) Per-leg Gauss-Newton IK: move ONLY leg L's joints to drive its foot to the world target
  //    (legs are kinematically independent given the pinned base). The 3×3 leg Jacobian is the
  //    leg's columns of the single-foot contact Jacobian; the fixed-size QR is allocation-free.
  double max_err = 0.0;
  for (int L = 0; L < nc_; ++L) {
    const std::size_t fid = feet_ids_[static_cast<std::size_t>(L)];
    one_fid_[0] = fid;
    const Eigen::Vector3d target = plan_.swing_pos[static_cast<std::size_t>(L)];
    double err_norm = 0.0;
    for (int it = 0; it < g_.ik_max_iter; ++it) {
      const Eigen::Vector3d err = target - model_->frame_position(*ws_, q_ik_, fid);
      err_norm = err.norm();
      if (err_norm < g_.ik_tol) break;
      model_->contact_jacobian_stacked(*ws_, q_ik_, one_fid_, J1_);  // 3 × nv
      Eigen::Matrix3d Jl;
      for (int c = 0; c < 3; ++c) Jl.col(c) = J1_.col(act_v_[static_cast<std::size_t>(L * 3 + c)]);
      Eigen::Vector3d dq = Jl.colPivHouseholderQr().solve(err);
      const double n = dq.norm();
      if (n > g_.ik_step_clamp) dq *= g_.ik_step_clamp / n;  // step / singularity guard
      for (int c = 0; c < 3; ++c) q_ik_[act_q_[static_cast<std::size_t>(L * 3 + c)]] += dq[c];
    }
    max_err = std::max(max_err, err_norm);
  }

  // 4) Joint PD → torque on the effort interface. Target joint velocity is 0 (as CHAMP / RL
  //    deploys): kp tracks the IK'd position command, kd damps the measured joint velocity.
  for (int k = 0; k < m_; ++k) {
    const int qi = act_q_[static_cast<std::size_t>(k)];
    const int vi = act_v_[static_cast<std::size_t>(k)];
    double tau = g_.kp * (q_ik_[qi] - state.q[qi]) - g_.kd * state.v[vi];
    tau = std::clamp(tau, -g_.tau_max, g_.tau_max);
    command_.tau[k] = tau;
  }

  // 5) Trust: converged when every foot reached its target within ~1 mm.
  status_.ok = max_err < 1e-3;
  status_.margin = 1e-3 - max_err;  // >= 0 => feet within tolerance
  status_.iters = 0;
  return command_;
}

}  // namespace kontrolem_controllers
