// M16 acceptance: an out-of-tree controller drops in with ZERO framework edits.
// This test does exactly what KontrolemController::build_law() does — construct a
// pluginlib::ClassLoader<kontrolem_control::ControllerFactory>, find the factory
// whose name() is "example", and build + run it — but from a package the
// framework has never heard of. If "example" is discovered and produces a finite
// command, the registry seam works for third parties.
//
// Run with the workspace SOURCED (so the ament index + plugin .so are found):
//   source install/setup.bash && ./build/kontrolem_controller_example/test_example_discoverable
#include <iostream>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <pluginlib/class_loader.hpp>

#include "kontrolem_control/controller_factory.hpp"
#include "kontrolem_control/problem.hpp"
#include "kontrolem_model/robot_model.hpp"

using kontrolem_control::BuildContext;
using kontrolem_control::Command;
using kontrolem_control::ControllerFactory;
using kontrolem_control::ParameterMap;
using kontrolem_control::Regulation;
using kontrolem_control::State;
using kontrolem_model::RobotModel;

int main()
{
  pluginlib::ClassLoader<ControllerFactory> loader(
    "kontrolem_control", "kontrolem_control::ControllerFactory");

  // Discover by name(), exactly as the runtime does.
  std::shared_ptr<ControllerFactory> example;
  for (const auto & cls : loader.getDeclaredClasses()) {
    auto f = loader.createSharedInstance(cls);
    if (f->name() == "example") {
      example = f;
    }
  }
  if (!example) {
    std::cout << "FAIL: 'example' factory not discovered by the ControllerFactory ClassLoader\n";
    return 1;
  }

  // Build it on the 2-DoF arm using its own parameter schema (defaults).
  const RobotModel model = RobotModel::from_urdf_file(ARM2_URDF);
  BuildContext ctx;
  ctx.actuated_joints = {"shoulder_joint", "elbow_joint"};
  ParameterMap params(example->parameter_spec());
  auto controller = example->create(params, model, ctx);

  Regulation reg;
  reg.q_ref = Eigen::VectorXd::Zero(model.nq());
  reg.v_ref = Eigen::VectorXd::Zero(model.nv());

  auto syn = controller->synthesize(model, reg);
  controller->configure(model, *syn, reg);

  State s;
  s.q = Eigen::VectorXd::Zero(model.nq());
  s.q(0) = 0.3;  // a non-trivial pose so gravity comp + PD produce a real command
  s.v = Eigen::VectorXd::Zero(model.nv());
  s.t = 0.0;

  const Command & u = controller->compute(s, reg, 0.005);
  const bool ok = u.tau.size() == 2 && u.tau.allFinite() && controller->status().ok;
  std::cout << "discovered 'example' out-of-tree factory; command = " << u.tau.transpose()
            << " -> " << (ok ? "PASS" : "FAIL") << "\n";
  return ok ? 0 : 1;
}
