// A minimal OUT-OF-TREE controller (M16 acceptance): it lives in its own package,
// depends only on the ROS-free contract + model, ships its own ControllerFactory,
// and is selected with `control_law: example` — with ZERO edits to
// kontrolem_ros2_control or kontrolem_controllers. If this loads and runs, the
// registry hole (the old hardcoded make_law() if/else) is closed.
//
// The law itself is deliberately trivial: gravity compensation + joint-space PD
// to a Regulation setpoint (enough to hold a fully-actuated arm against gravity).
// It is an EXAMPLE, so it is not allocation-audited — it exists to demonstrate the
// plugin seam, not to be a production controller.
#ifndef KONTROLEM_CONTROLLER_EXAMPLE__EXAMPLE_CONTROLLER_HPP_
#define KONTROLEM_CONTROLLER_EXAMPLE__EXAMPLE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "kontrolem_control/controller_factory.hpp"

namespace kontrolem_controller_example
{

using kontrolem_control::BuildContext;
using kontrolem_control::Capabilities;
using kontrolem_control::Command;
using kontrolem_control::Controller;
using kontrolem_control::ControllerFactory;
using kontrolem_control::ControlProblem;
using kontrolem_control::ParameterMap;
using kontrolem_control::ParameterSpec;
using kontrolem_control::State;
using kontrolem_control::Status;
using kontrolem_control::Synthesis;
using kontrolem_model::RobotModel;

/// Gravity-comp + joint PD to a Regulation setpoint, on the actuated joints.
class ExampleController : public Controller
{
public:
  ExampleController(std::vector<std::string> actuated, double kp, double kd, double tau_max);

  Capabilities capabilities() const override;
  std::unique_ptr<Synthesis> synthesize(const RobotModel &, const ControlProblem &) const override;
  void configure(const RobotModel & model, const Synthesis &, const ControlProblem &) override;
  const Command & compute(const State & state, const ControlProblem & problem, double dt) override;
  const Status & status() const override { return status_; }

private:
  std::vector<std::string> actuated_;
  double kp_, kd_, tau_max_;
  const RobotModel * model_{nullptr};
  std::vector<int> q_idx_, v_idx_;  // actuated joint -> generalized q/v index
  Eigen::VectorXd grav_;
  Command command_;
  Status status_;
};

/// The registry entry: this is what makes ExampleController discoverable by name.
class ExampleFactory : public ControllerFactory
{
public:
  std::string name() const override { return "example"; }
  ParameterSpec parameter_spec() const override;
  std::unique_ptr<Controller> create(
    const ParameterMap &, const RobotModel &, const BuildContext &) const override;
};

}  // namespace kontrolem_controller_example

#endif  // KONTROLEM_CONTROLLER_EXAMPLE__EXAMPLE_CONTROLLER_HPP_
