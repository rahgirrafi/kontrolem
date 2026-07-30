#include "kontrolem_controller_example/example_controller.hpp"

#include <algorithm>

#include <pluginlib/class_list_macros.hpp>

#include "kontrolem_control/problem.hpp"

namespace kontrolem_controller_example
{
namespace kc = kontrolem_control;

ExampleController::ExampleController(
  std::vector<std::string> actuated, double kp, double kd, double tau_max)
: actuated_(std::move(actuated)), kp_(kp), kd_(kd), tau_max_(tau_max)
{
}

Capabilities ExampleController::capabilities() const
{
  return Capabilities{{kc::Dialect::kRegulation}, /*needs_velocity_state=*/true};
}

std::unique_ptr<Synthesis> ExampleController::synthesize(
  const RobotModel &, const ControlProblem &) const
{
  return std::make_unique<Synthesis>();  // nothing to precompute
}

void ExampleController::configure(const RobotModel & model, const Synthesis &, const ControlProblem &)
{
  model_ = &model;
  q_idx_.clear();
  v_idx_.clear();
  for (const auto & a : actuated_) {
    q_idx_.push_back(model.joint_q_index(a));
    v_idx_.push_back(model.joint_v_index(a));
  }
  command_.tau = Eigen::VectorXd::Zero(static_cast<int>(actuated_.size()));
  grav_.resize(model.nv());
  status_ = Status{true, 0.0, 0};
}

const Command & ExampleController::compute(
  const State & state, const ControlProblem & problem, double)
{
  const auto & reg = static_cast<const kc::Regulation &>(problem);
  grav_ = model_->gravity_torque(state.q);  // gravity to hold the current config
  for (std::size_t i = 0; i < actuated_.size(); ++i) {
    const int qi = q_idx_[i];
    const int vi = v_idx_[i];
    const double e = state.q[qi] - reg.q_ref[qi];
    double tau = grav_[vi] - kp_ * e - kd_ * state.v[vi];
    command_.tau[static_cast<int>(i)] = std::max(-tau_max_, std::min(tau_max_, tau));
  }
  status_ = Status{true, 0.0, 0};
  return command_;
}

// ---------------------------- factory -------------------------------------
ParameterSpec ExampleFactory::parameter_spec() const
{
  return {
    {"example.kp", kc::ParamValue{80.0}, "joint position stiffness"},
    {"example.kd", kc::ParamValue{16.0}, "joint velocity damping"},
    {"example.tau_max", kc::ParamValue{30.0}, "per-joint torque clamp"},
  };
}

std::unique_ptr<Controller> ExampleFactory::create(
  const ParameterMap & p, const RobotModel &, const BuildContext & ctx) const
{
  return std::make_unique<ExampleController>(
    ctx.actuated_joints, p.double_at("example.kp"), p.double_at("example.kd"),
    p.double_at("example.tau_max"));
}

}  // namespace kontrolem_controller_example

// This one line — in the third-party package, not the framework — is what makes
// `control_law: example` resolve at runtime.
PLUGINLIB_EXPORT_CLASS(
  kontrolem_controller_example::ExampleFactory, kontrolem_control::ControllerFactory)
