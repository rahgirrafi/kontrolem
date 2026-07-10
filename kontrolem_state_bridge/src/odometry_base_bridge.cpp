#include "kontrolem_state_bridge/odometry_base_bridge.hpp"

#include "kontrolem_state_bridge/odom_to_base.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace kontrolem_state_bridge
{
using hardware_interface::CallbackReturn;
using hardware_interface::StateInterface;
using hardware_interface::return_type;

namespace
{
rclcpp::Logger logger() { return rclcpp::get_logger("OdometryBaseBridge"); }
}  // namespace

CallbackReturn OdometryBaseBridge::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SensorInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (info_.gpios.empty()) {
    RCLCPP_ERROR(logger(), "expected a <gpio> block carrying the base state interfaces");
    return CallbackReturn::ERROR;
  }
  gpio_name_ = info_.gpios.front().name;

  // Odometry topic: hardware <param name="topic">, default /base_odom.
  auto it = info_.hardware_parameters.find("topic");
  topic_ = (it != info_.hardware_parameters.end() && !it->second.empty())
             ? it->second
             : std::string("/base_odom");

  // Identity pose until the first message arrives, so a controller that activates
  // before odometry starts reads a valid (neutral) SE(3) rather than garbage.
  base_ = {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0};
  RCLCPP_INFO(logger(), "state bridge: gpio '%s', odom topic '%s'",
              gpio_name_.c_str(), topic_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn OdometryBaseBridge::on_configure(const rclcpp_lifecycle::State &)
{
  node_ = rclcpp::Node::make_shared("kontrolem_state_bridge_" + gpio_name_);
  sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    topic_, rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      rt_odom_.writeFromNonRT(msg);
    });
  RCLCPP_INFO(logger(), "subscribed to '%s'", topic_.c_str());
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> OdometryBaseBridge::export_state_interfaces()
{
  // Bind the <gpio> state interfaces, in URDF order, to base_[0..12] — the URDF
  // lists pose.* then twist.* so the order matches odom_to_base's layout and what
  // BaseStateSensor claims by name.
  std::vector<StateInterface> ifaces;
  const auto & gpio = info_.gpios.front();
  std::size_t k = 0;
  for (const auto & si : gpio.state_interfaces) {
    if (k < base_.size()) {
      ifaces.emplace_back(gpio.name, si.name, &base_[k]);
      ++k;
    }
  }
  return ifaces;
}

return_type OdometryBaseBridge::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  rclcpp::spin_some(node_);  // service the subscription callback
  auto msg = *rt_odom_.readFromRT();
  if (msg) {
    const auto & p = msg->pose.pose.position;
    const auto & o = msg->pose.pose.orientation;
    const auto & lin = msg->twist.twist.linear;
    const auto & ang = msg->twist.twist.angular;
    base_from_odom(p.x, p.y, p.z, o.x, o.y, o.z, o.w,
                   lin.x, lin.y, lin.z, ang.x, ang.y, ang.z, base_);
  }
  return return_type::OK;  // no message yet -> hold last (identity at startup)
}

}  // namespace kontrolem_state_bridge

PLUGINLIB_EXPORT_CLASS(
  kontrolem_state_bridge::OdometryBaseBridge, hardware_interface::SensorInterface)
