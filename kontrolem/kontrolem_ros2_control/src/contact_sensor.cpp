#include "kontrolem_ros2_control/contact_sensor.hpp"

namespace kontrolem_ros2_control
{

ContactSensor::ContactSensor(const std::string & gpio_name, const std::vector<std::string> & feet)
: gpio_(gpio_name)
{
  names_.reserve(feet.size());
  for (const auto & f : feet) {
    names_.push_back(gpio_ + "/contact." + f);
  }
}

bool ContactSensor::assign_loaned(
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

void ContactSensor::read_into(std::vector<double> & out) const
{
  out.resize(refs_.size());
  for (std::size_t i = 0; i < refs_.size(); ++i) {
    out[i] = refs_[i].get().get_value();
  }
}

void ContactSensor::stance_into(std::vector<bool> & out, double threshold) const
{
  out.resize(refs_.size());
  for (std::size_t i = 0; i < refs_.size(); ++i) {
    out[i] = refs_[i].get().get_value() > threshold;
  }
}

}  // namespace kontrolem_ros2_control
