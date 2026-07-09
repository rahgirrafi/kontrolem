#include "kontrolem_ros2_control/base_state_sensor.hpp"

namespace kontrolem_ros2_control
{

BaseStateSensor::BaseStateSensor(const std::string & gpio_name) : gpio_(gpio_name)
{
  // q(0..6): position xyz, quaternion xyzw.  v(0..5): linear xyz, angular xyz.
  const char * suffix[13] = {
    "pose.position.x", "pose.position.y", "pose.position.z",
    "pose.orientation.x", "pose.orientation.y", "pose.orientation.z", "pose.orientation.w",
    "twist.linear.x", "twist.linear.y", "twist.linear.z",
    "twist.angular.x", "twist.angular.y", "twist.angular.z"};
  names_.reserve(13);
  for (const auto * s : suffix) {
    names_.push_back(gpio_ + "/" + s);
  }
}

bool BaseStateSensor::assign_loaned(
  std::vector<hardware_interface::LoanedStateInterface> & interfaces)
{
  refs_.clear();
  refs_.reserve(names_.size());
  for (const auto & name : names_) {
    bool found = false;
    for (auto & iface : interfaces) {
      if (iface.get_name() == name) {
        refs_.emplace_back(std::ref(iface));
        found = true;
        break;
      }
    }
    if (!found) {
      refs_.clear();
      return false;
    }
  }
  return true;
}

void BaseStateSensor::read_into(Eigen::VectorXd & q, Eigen::VectorXd & v) const
{
  for (int i = 0; i < 7; ++i) {
    q(i) = refs_[static_cast<std::size_t>(i)].get().get_value();
  }
  for (int i = 0; i < 6; ++i) {
    v(i) = refs_[static_cast<std::size_t>(7 + i)].get().get_value();
  }
  q.segment<4>(3).normalize();  // enforce the unit-quaternion manifold constraint
}

}  // namespace kontrolem_ros2_control
