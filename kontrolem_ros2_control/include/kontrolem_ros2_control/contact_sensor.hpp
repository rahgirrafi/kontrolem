// ContactSensor — the semantic component that carries per-foot CONTACT state on
// the ros2_control side (M4.2). ros2_control has no contact concept, so each foot
// is one scalar state interface (0 = swing, 1 = stance; a probability in between)
// on a <gpio> block. This owns the Kontrol'Em contact interface-naming convention
// (`<gpio>/contact.<foot>`) and the reassembly into a bool stance vector, so the
// WBC never re-derives it. Modelled on BaseStateSensor / ros2_control's
// semantic_components. For a standing demo the schedule is all-stance (Part D:
// contact is an integrator-supplied signal; the sim provides ground truth).
#ifndef KONTROLEM_ROS2_CONTROL__CONTACT_SENSOR_HPP_
#define KONTROLEM_ROS2_CONTROL__CONTACT_SENSOR_HPP_

#include <functional>
#include <string>
#include <vector>

#include "hardware_interface/loaned_state_interface.hpp"

namespace kontrolem_ros2_control
{

class ContactSensor
{
public:
  /// One scalar interface per foot on `gpio_name`, named `gpio/contact.<foot>`.
  /// `feet` are the contact-frame short names (e.g. "FL", "FR", ...).
  ContactSensor(const std::string & gpio_name, const std::vector<std::string> & feet);

  /// The interface names to claim, one per foot in the given order.
  const std::vector<std::string> & interface_names() const { return names_; }
  std::size_t size() const { return names_.size(); }

  /// Bind to the controller's claimed state interfaces (call in on_activate).
  /// Returns false if any contact interface is missing.
  bool assign_loaned(std::vector<hardware_interface::LoanedStateInterface> & interfaces);

  void release() { refs_.clear(); }
  bool assigned() const { return refs_.size() == names_.size(); }

  /// Read the raw contact scalars (0..1) into `out` (resized to #feet).
  /// Precondition: assigned().
  void read_into(std::vector<double> & out) const;

  /// Convenience: stance mask (scalar > threshold). Precondition: assigned().
  void stance_into(std::vector<bool> & out, double threshold = 0.5) const;

private:
  std::string gpio_;
  std::vector<std::string> names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> refs_;
};

}  // namespace kontrolem_ros2_control

#endif  // KONTROLEM_ROS2_CONTROL__CONTACT_SENSOR_HPP_
