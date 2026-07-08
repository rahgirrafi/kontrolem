// BaseStateSensor — the semantic component that encapsulates a floating base's
// non-joint state on the ros2_control side (M3.4). It owns the Kontrol'Em base
// interface-naming convention (13 scalars: SE(3) pose + spatial twist on a
// <gpio> block) and the reassembly into the manifold State slots — including the
// quaternion normalization and the q(0..6)/v(0..5) index layout — so a
// controller (the WBC later) never re-derives that awkward coupling. Modelled on
// ros2_control's semantic_components::IMUSensor / ForceTorqueSensor.
#ifndef KONTROLEM_ROS2_CONTROL__BASE_STATE_SENSOR_HPP_
#define KONTROLEM_ROS2_CONTROL__BASE_STATE_SENSOR_HPP_

#include <functional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "hardware_interface/loaned_state_interface.hpp"

namespace kontrolem_ros2_control
{

class BaseStateSensor
{
public:
  /// `gpio_name` is the <gpio> block carrying the base scalars (default matches
  /// the description convention).
  explicit BaseStateSensor(const std::string & gpio_name = "floating_base");

  /// The 13 interface names to claim, in q(pos xyz, quat xyzw) / v(lin, ang) order.
  const std::vector<std::string> & interface_names() const { return names_; }

  /// Bind to the controller's claimed state interfaces (call in on_activate).
  /// Returns false if any base interface is missing.
  bool assign_loaned(std::vector<hardware_interface::LoanedStateInterface> & interfaces);

  void release() { refs_.clear(); }
  bool assigned() const { return refs_.size() == names_.size(); }

  /// Read the base pose/twist into the SE(3) root slots q(0..6), v(0..5),
  /// normalizing the quaternion (the interface transport does not guarantee it).
  /// The rest of q/v (the joints) is left untouched. Precondition: assigned().
  void read_into(Eigen::VectorXd & q, Eigen::VectorXd & v) const;

private:
  std::string gpio_;
  std::vector<std::string> names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> refs_;
};

}  // namespace kontrolem_ros2_control

#endif  // KONTROLEM_ROS2_CONTROL__BASE_STATE_SENSOR_HPP_
