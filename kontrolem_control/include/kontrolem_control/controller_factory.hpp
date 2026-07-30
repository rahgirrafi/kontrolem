// Kontrol'Em v2 — the controller REGISTRY seam (M16).
//
// A ControllerFactory is how a control law makes itself buildable from a plain
// (ParameterMap) configuration, WITHOUT the runtime hardcoding its name or its
// parameter list. The L4 runtime looks a factory up by name(), asks it for its
// parameter_spec() to declare/validate the ROS parameters generically, then
// calls create() to construct the controller. A third-party law ships its own
// factory (exported via pluginlib in a bridge package) and drops in with ZERO
// edits to the runtime.
//
// ROS-free, like the rest of the contract: a factory reads a ParameterMap and
// returns a Controller; it never touches rclcpp. Discovery (pluginlib) lives in
// a separate L4 package so this layer stays middleware-independent.
#ifndef KONTROLEM_CONTROL__CONTROLLER_FACTORY_HPP_
#define KONTROLEM_CONTROL__CONTROLLER_FACTORY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "kontrolem_control/controller.hpp"
#include "kontrolem_control/params.hpp"
#include "kontrolem_model/robot_model.hpp"

namespace kontrolem_control
{

/// Structural (non-tunable) build inputs that are NOT parameters: which joints a
/// controller actuates, and — for legged/contact laws — which frames are feet.
/// Kept separate from the ParameterMap because these come from the robot's
/// semantic description, not from the controller's tuning.
struct BuildContext
{
  std::vector<std::string> actuated_joints;
  std::vector<std::string> contact_frames;
};

/// The registry entry every control law exposes. Cheap and stateless — the
/// runtime may instantiate one just to read name()/parameter_spec().
class ControllerFactory
{
public:
  virtual ~ControllerFactory() = default;

  /// The law's short name, e.g. "wbc". This is the lookup key (`control_law:`),
  /// so it must be unique across all registered factories.
  virtual std::string name() const = 0;

  /// The law's parameter schema, as data: names, defaults (whose types pin the
  /// parameter types), and descriptions. The runtime iterates this to declare
  /// and validate ROS parameters; a design tool reads it to show the knobs.
  virtual ParameterSpec parameter_spec() const = 0;

  /// Build the controller from configured values + the model + structural inputs.
  /// `params` is expected to have been seeded from parameter_spec() and overlaid
  /// with the deployment's values. NOT real-time.
  virtual std::unique_ptr<Controller> create(
    const ParameterMap & params, const RobotModel & model, const BuildContext & context) const = 0;
};

}  // namespace kontrolem_control

#endif  // KONTROLEM_CONTROL__CONTROLLER_FACTORY_HPP_
