// Kontrol'Em v2 — the M0..M15 control laws, each wrapped as a ControllerFactory
// (M16). A factory turns a ROS-free ParameterMap into a configured controller
// and declares its own parameter schema as data, so the L4 runtime no longer
// hardcodes either the law's name or its parameter list. These live here (the
// ROS-free impl package) and stay unit-testable off-middleware; pluginlib export
// happens in the separate kontrolem_controller_plugins bridge package.
#ifndef KONTROLEM_CONTROLLERS__FACTORIES_HPP_
#define KONTROLEM_CONTROLLERS__FACTORIES_HPP_

#include <memory>
#include <string>

#include "kontrolem_control/controller_factory.hpp"

namespace kontrolem_controllers
{

using kontrolem_control::BuildContext;
using kontrolem_control::Controller;
using kontrolem_control::ControllerFactory;
using kontrolem_control::ParameterMap;
using kontrolem_control::ParameterSpec;
using kontrolem_model::RobotModel;

// One factory per shipped control law. Each is stateless and default-
// constructible (a pluginlib requirement); the parameter names it declares are
// the SAME strings the runtime used to hardcode, so existing YAMLs are unchanged.

class LqrFactory : public ControllerFactory
{
public:
  std::string name() const override { return "lqr"; }
  ParameterSpec parameter_spec() const override;
  std::unique_ptr<Controller> create(
    const ParameterMap &, const RobotModel &, const BuildContext &) const override;
};

class LqgFactory : public ControllerFactory
{
public:
  std::string name() const override { return "lqg"; }
  ParameterSpec parameter_spec() const override;
  std::unique_ptr<Controller> create(
    const ParameterMap &, const RobotModel &, const BuildContext &) const override;
};

/// Gain-scheduling / LPV (M17). Unlike the other laws its design input is a LIST
/// of scheduling axes, which the flat parameter world expresses as four parallel
/// arrays (joints / min / max / nodes); create() resolves the joint NAMES to
/// configuration indices through the model, and rejects a mis-shaped grid there
/// rather than at the first tick.
class LpvFactory : public ControllerFactory
{
public:
  std::string name() const override { return "lpv"; }
  ParameterSpec parameter_spec() const override;
  std::unique_ptr<Controller> create(
    const ParameterMap &, const RobotModel &, const BuildContext &) const override;
};

class MpcFactory : public ControllerFactory
{
public:
  std::string name() const override { return "mpc"; }
  ParameterSpec parameter_spec() const override;
  std::unique_ptr<Controller> create(
    const ParameterMap &, const RobotModel &, const BuildContext &) const override;
};

class QpFactory : public ControllerFactory
{
public:
  std::string name() const override { return "qp"; }
  ParameterSpec parameter_spec() const override;
  std::unique_ptr<Controller> create(
    const ParameterMap &, const RobotModel &, const BuildContext &) const override;
};

class WbcFactory : public ControllerFactory
{
public:
  std::string name() const override { return "wbc"; }
  ParameterSpec parameter_spec() const override;
  std::unique_ptr<Controller> create(
    const ParameterMap &, const RobotModel &, const BuildContext &) const override;
};

class KinematicGaitFactory : public ControllerFactory
{
public:
  std::string name() const override { return "kinematic_gait"; }
  ParameterSpec parameter_spec() const override;
  std::unique_ptr<Controller> create(
    const ParameterMap &, const RobotModel &, const BuildContext &) const override;
};

}  // namespace kontrolem_controllers

#endif  // KONTROLEM_CONTROLLERS__FACTORIES_HPP_
