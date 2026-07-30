// ControllerFactory implementations for the shipped laws (M16). Each
// parameter_spec() is the single source of truth for that law's parameters +
// defaults (moved out of the runtime's on_init auto_declare block), and each
// create() reproduces exactly what the old make_law() if/else did — read the
// values, build the matrices/Gains, call the existing typed constructor. The
// controllers themselves are untouched; only construction moved here.
#include "kontrolem_controllers/factories.hpp"

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "kontrolem_controllers/kinematic_gait_controller.hpp"
#include "kontrolem_controllers/lpv_controller.hpp"
#include "kontrolem_controllers/lqg_controller.hpp"
#include "kontrolem_controllers/lqr_controller.hpp"
#include "kontrolem_controllers/mpc_controller.hpp"
#include "kontrolem_controllers/qp_task_space_controller.hpp"
#include "kontrolem_controllers/wbc_controller.hpp"

namespace kontrolem_controllers
{
namespace kc = kontrolem_control;

namespace
{
// Build an n x n diagonal from d, or the identity if d is empty / mis-sized.
// (Same helper the runtime used inside make_law; kept ROS-free here.)
Eigen::MatrixXd diag_or_identity(const std::vector<double> & d, int n)
{
  Eigen::MatrixXd M = Eigen::MatrixXd::Identity(n, n);
  if (static_cast<int>(d.size()) == n) {
    for (int i = 0; i < n; ++i) {
      M(i, i) = d[static_cast<std::size_t>(i)];
    }
  }
  return M;
}

// Convenience: an empty double/string array default, spelled once.
kc::ParamValue empty_doubles() { return kc::ParamValue{std::vector<double>{}}; }
}  // namespace

// ------------------------------- LQR --------------------------------------
ParameterSpec LqrFactory::parameter_spec() const
{
  return {
    {"lqr.q_diag", empty_doubles(), "state-cost diagonal (empty -> identity, dim 2*nv)"},
    {"lqr.r_diag", empty_doubles(), "input-cost diagonal (empty -> identity, dim n_actuated)"},
    {"lqr.q_dev_max", kc::ParamValue{0.5}, "trust-region radius (Q-weighted state deviation)"},
  };
}
std::unique_ptr<Controller> LqrFactory::create(
  const ParameterMap & p, const RobotModel & model, const BuildContext & ctx) const
{
  const int nv = model.nv();
  const int m = static_cast<int>(ctx.actuated_joints.size());
  return std::make_unique<LqrController>(
    ctx.actuated_joints, diag_or_identity(p.double_array_at("lqr.q_diag"), 2 * nv),
    diag_or_identity(p.double_array_at("lqr.r_diag"), m), p.double_at("lqr.q_dev_max"));
}

// ------------------------------- LQG --------------------------------------
ParameterSpec LqgFactory::parameter_spec() const
{
  return {
    {"lqg.q_diag", empty_doubles(), "LQR state-cost diagonal (empty -> identity)"},
    {"lqg.r_diag", empty_doubles(), "LQR input-cost diagonal (empty -> identity)"},
    {"lqg.w_diag", empty_doubles(), "process-noise covariance diagonal (empty -> identity)"},
    {"lqg.v_diag", empty_doubles(), "measurement-noise covariance diagonal (empty -> identity)"},
    {"lqg.innov_max", kc::ParamValue{0.5}, "innovation chi-square gate"},
  };
}
std::unique_ptr<Controller> LqgFactory::create(
  const ParameterMap & p, const RobotModel & model, const BuildContext & ctx) const
{
  const int nq = model.nq();
  const int nv = model.nv();
  const int m = static_cast<int>(ctx.actuated_joints.size());
  return std::make_unique<LqgController>(
    ctx.actuated_joints, diag_or_identity(p.double_array_at("lqg.q_diag"), 2 * nv),
    diag_or_identity(p.double_array_at("lqg.r_diag"), m),
    diag_or_identity(p.double_array_at("lqg.w_diag"), 2 * nv),
    diag_or_identity(p.double_array_at("lqg.v_diag"), nq), p.double_at("lqg.innov_max"));
}

// ------------------------------- LPV --------------------------------------
// The only shipped law whose design input is a LIST of records (the scheduling
// axes). ROS parameters are flat, so the grid is four parallel arrays; the
// joints are named (not raw q indices) and resolved through the model here, so a
// bad joint name or a mis-shaped grid fails at configure with a clear message
// instead of surfacing as a wrong gain at the first tick.
ParameterSpec LpvFactory::parameter_spec() const
{
  return {
    {"lpv.q_diag", empty_doubles(), "state-cost diagonal (empty -> identity, dim 2*nv)"},
    {"lpv.r_diag", empty_doubles(), "input-cost diagonal (empty -> identity, dim n_actuated)"},
    {"lpv.sched_joints", kc::ParamValue{std::vector<std::string>{}},
     "joints whose configuration schedules the design — one per axis (required)"},
    {"lpv.sched_min", empty_doubles(), "per-axis envelope lower bound"},
    {"lpv.sched_max", empty_doubles(), "per-axis envelope upper bound"},
    {"lpv.sched_nodes", kc::ParamValue{std::vector<std::int64_t>{}},
     "per-axis grid node count, >= 2 (empty -> 9 on every axis)"},
  };
}
std::unique_ptr<Controller> LpvFactory::create(
  const ParameterMap & p, const RobotModel & model, const BuildContext & ctx) const
{
  const int nv = model.nv();
  const int m = static_cast<int>(ctx.actuated_joints.size());

  const auto & joints = p.string_array_at("lpv.sched_joints");
  const auto & lo = p.double_array_at("lpv.sched_min");
  const auto & hi = p.double_array_at("lpv.sched_max");
  const auto & nodes = p.int_array_at("lpv.sched_nodes");
  const std::size_t D = joints.size();

  if (D == 0) {
    throw std::runtime_error(
      "lpv: 'lpv.sched_joints' is empty — LPV needs at least one scheduling axis (name the "
      "joint(s) whose configuration schedules the gain design)");
  }
  if (lo.size() != D || hi.size() != D) {
    throw std::runtime_error(
      "lpv: 'lpv.sched_min' and 'lpv.sched_max' must each have one entry per scheduling joint (" +
      std::to_string(D) + ")");
  }
  if (!nodes.empty() && nodes.size() != D) {
    throw std::runtime_error(
      "lpv: 'lpv.sched_nodes' must be empty (default 9 per axis) or have one entry per "
      "scheduling joint (" + std::to_string(D) + ")");
  }

  std::vector<SchedAxis> axes;
  axes.reserve(D);
  for (std::size_t d = 0; d < D; ++d) {
    SchedAxis ax;
    ax.q_index = model.joint_q_index(joints[d]);  // throws if the joint is absent
    ax.min = lo[d];
    ax.max = hi[d];
    ax.n = nodes.empty() ? 9 : static_cast<int>(nodes[d]);
    axes.push_back(ax);
  }
  // LpvController's constructor validates n >= 2 and max > min per axis.
  return std::make_unique<LpvController>(
    ctx.actuated_joints, diag_or_identity(p.double_array_at("lpv.q_diag"), 2 * nv),
    diag_or_identity(p.double_array_at("lpv.r_diag"), m), std::move(axes));
}

// ------------------------------- MPC --------------------------------------
ParameterSpec MpcFactory::parameter_spec() const
{
  return {
    {"mpc.q_diag", empty_doubles(), "state-cost diagonal (empty -> identity)"},
    {"mpc.r_diag", empty_doubles(), "input-cost diagonal (empty -> identity)"},
    {"mpc.horizon", kc::ParamValue{std::int64_t{30}}, "prediction horizon (steps)"},
    {"mpc.dt_mpc", kc::ParamValue{0.02}, "discretization step (s)"},
    {"mpc.tau_max", kc::ParamValue{100.0}, "per-input torque limit"},
  };
}
std::unique_ptr<Controller> MpcFactory::create(
  const ParameterMap & p, const RobotModel & model, const BuildContext & ctx) const
{
  const int nv = model.nv();
  const int m = static_cast<int>(ctx.actuated_joints.size());
  return std::make_unique<MpcController>(
    ctx.actuated_joints, diag_or_identity(p.double_array_at("mpc.q_diag"), 2 * nv),
    diag_or_identity(p.double_array_at("mpc.r_diag"), m),
    static_cast<int>(p.int_at("mpc.horizon")), p.double_at("mpc.dt_mpc"),
    p.double_at("mpc.tau_max"));
}

// ------------------------------- QP ---------------------------------------
ParameterSpec QpFactory::parameter_spec() const
{
  return {
    {"qp.task_weight", empty_doubles(), "per-DoF task weight (empty -> ones, dim nv)"},
    {"qp.kp", kc::ParamValue{50.0}, "task-space position stiffness"},
    {"qp.kd", kc::ParamValue{10.0}, "task-space velocity damping"},
    {"qp.tau_max", kc::ParamValue{5.0}, "per-joint torque limit (QP inequality)"},
  };
}
std::unique_ptr<Controller> QpFactory::create(
  const ParameterMap & p, const RobotModel & model, const BuildContext & ctx) const
{
  const int nv = model.nv();
  const auto & w = p.double_array_at("qp.task_weight");
  Eigen::VectorXd W = Eigen::VectorXd::Ones(nv);
  if (static_cast<int>(w.size()) == nv) {
    for (int i = 0; i < nv; ++i) {
      W(i) = w[static_cast<std::size_t>(i)];
    }
  }
  return std::make_unique<QpTaskSpaceController>(
    ctx.actuated_joints, W, p.double_at("qp.kp"), p.double_at("qp.kd"), p.double_at("qp.tau_max"));
}

// ------------------------------- WBC --------------------------------------
ParameterSpec WbcFactory::parameter_spec() const
{
  return {
    {"wbc.kp_base", kc::ParamValue{100.0}, "base pose/orientation stiffness"},
    {"wbc.kd_base", kc::ParamValue{20.0}, "base twist damping"},
    {"wbc.kp_post", kc::ParamValue{25.0}, "joint posture stiffness"},
    {"wbc.kd_post", kc::ParamValue{5.0}, "joint posture damping"},
    {"wbc.w_base", kc::ParamValue{100.0}, "task weight on the 6 base DoF"},
    {"wbc.w_post", kc::ParamValue{1.0}, "task weight on the joints"},
    {"wbc.w_force", kc::ParamValue{1e-4}, "contact-force regularization"},
    {"wbc.w_tau", kc::ParamValue{1e-4}, "torque regularization"},
    {"wbc.mu", kc::ParamValue{0.7}, "friction coefficient (pyramid)"},
    {"wbc.tau_max", kc::ParamValue{40.0}, "per-joint torque limit"},
    {"wbc.max_iter", kc::ParamValue{std::int64_t{200}}, "OSQP iteration cap (hard-RT bound)"},
    {"wbc.kp_swing", kc::ParamValue{400.0}, "swing-foot tracking stiffness (Locomotion)"},
    {"wbc.kd_swing", kc::ParamValue{40.0}, "swing-foot tracking damping (Locomotion)"},
  };
}
std::unique_ptr<Controller> WbcFactory::create(
  const ParameterMap & p, const RobotModel &, const BuildContext & ctx) const
{
  WbcController::Gains g;
  g.kp_base = p.double_at("wbc.kp_base");
  g.kd_base = p.double_at("wbc.kd_base");
  g.kp_post = p.double_at("wbc.kp_post");
  g.kd_post = p.double_at("wbc.kd_post");
  g.w_base = p.double_at("wbc.w_base");
  g.w_post = p.double_at("wbc.w_post");
  g.w_force = p.double_at("wbc.w_force");
  g.w_tau = p.double_at("wbc.w_tau");
  g.mu = p.double_at("wbc.mu");
  g.tau_max = p.double_at("wbc.tau_max");
  g.max_iter = static_cast<int>(p.int_at("wbc.max_iter"));
  g.kp_swing = p.double_at("wbc.kp_swing");
  g.kd_swing = p.double_at("wbc.kd_swing");
  return std::make_unique<WbcController>(ctx.contact_frames, ctx.actuated_joints, g);
}

// -------------------------- Kinematic gait --------------------------------
ParameterSpec KinematicGaitFactory::parameter_spec() const
{
  return {
    {"kin.kp", kc::ParamValue{60.0}, "joint position stiffness (-> torque)"},
    {"kin.kd", kc::ParamValue{2.0}, "joint velocity damping"},
    {"kin.tau_max", kc::ParamValue{23.7}, "per-joint torque clamp"},
    {"kin.ik_max_iter", kc::ParamValue{std::int64_t{20}}, "per-leg Gauss-Newton cap"},
    {"kin.ik_tol", kc::ParamValue{1e-6}, "foot-position convergence (m)"},
    {"kin.ik_step_clamp", kc::ParamValue{0.5}, "max |dq| per IK iteration (rad)"},
  };
}
std::unique_ptr<Controller> KinematicGaitFactory::create(
  const ParameterMap & p, const RobotModel &, const BuildContext & ctx) const
{
  KinematicGaitController::Gains g;
  g.kp = p.double_at("kin.kp");
  g.kd = p.double_at("kin.kd");
  g.tau_max = p.double_at("kin.tau_max");
  g.ik_max_iter = static_cast<int>(p.int_at("kin.ik_max_iter"));
  g.ik_tol = p.double_at("kin.ik_tol");
  g.ik_step_clamp = p.double_at("kin.ik_step_clamp");
  return std::make_unique<KinematicGaitController>(ctx.contact_frames, ctx.actuated_joints, g);
}

}  // namespace kontrolem_controllers
