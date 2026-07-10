// OdometryBaseBridge — the "one localized ugliness" the v2 plan (Part B4) names:
// a ros2_control SensorInterface hardware component that subscribes to a base
// odometry topic and re-exports it as the 13 floating-base scalar StateInterfaces
// on a <gpio> block. It is the bridge that lets base pose/twist enter the RT loop
// as claimable state interfaces from a NON-custom producer (Gazebo ground truth
// in M6.3; a real InEKF estimator later), exactly as BaseStateSensor expects —
// so the controller side never learns whether the base came from our sim, Gazebo,
// or hardware.
//
// Surfacing a topic subscriber as "hardware" is philosophically odd (an estimator
// isn't hardware) but is the only RT-correct way to feed non-joint state into a
// controller's state interfaces (Part B4). The mapping itself lives in the
// ROS-free odom_to_base.hpp; this file is just the ros2_control + subscription
// plumbing around it.
#ifndef KONTROLEM_STATE_BRIDGE__ODOMETRY_BASE_BRIDGE_HPP_
#define KONTROLEM_STATE_BRIDGE__ODOMETRY_BASE_BRIDGE_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace kontrolem_state_bridge
{

class OdometryBaseBridge : public hardware_interface::SensorInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Base scalars in the BaseStateSensor layout (pos, quat xyzw, lin, ang).
  std::array<double, 13> base_{};
  std::string gpio_name_;  // the <gpio> block carrying the 13 base interfaces
  std::string topic_;      // odometry topic to subscribe to

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<nav_msgs::msg::Odometry>> rt_odom_;
};

}  // namespace kontrolem_state_bridge

#endif  // KONTROLEM_STATE_BRIDGE__ODOMETRY_BASE_BRIDGE_HPP_
