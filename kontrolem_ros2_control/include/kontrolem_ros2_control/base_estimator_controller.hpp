// BaseEstimatorController — the ros2_control wrapper around kontrolem_estimation's
// ROS-free BaseEstimator (M8). It is the sibling of FloatingStateProbe: a read-only
// ControllerInterface that claims the joint position/velocity + per-foot contact state
// interfaces, subscribes to the IMU, runs the contact-aided filter every tick, and
// PUBLISHES the estimated floating-base state as nav_msgs/Odometry on /base_odom
// (world-frame pose, body-frame twist — the exact contract OdometryBaseBridge re-imports
// into the floating_base interfaces the WBC reads).
//
// The core stays ROS-free (kontrolem_estimation); this class only does the ros2_control
// plumbing — the same ROS-free-core + thin-wrapper split as every controller here.
#ifndef KONTROLEM_ROS2_CONTROL__BASE_ESTIMATOR_CONTROLLER_HPP_
#define KONTROLEM_ROS2_CONTROL__BASE_ESTIMATOR_CONTROLLER_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "kontrolem_estimation/base_estimator.hpp"
#include "kontrolem_model/robot_model.hpp"
#include "kontrolem_ros2_control/contact_sensor.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/msg/imu.hpp"

namespace kontrolem_ros2_control
{

class BaseEstimatorController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::optional<kontrolem_model::RobotModel> model_;
  std::optional<kontrolem_estimation::BaseEstimator> estimator_;
  std::optional<ContactSensor> contact_sensor_;

  std::vector<std::string> joint_names_;     // actuated joints (encoder order)
  std::vector<std::size_t> jpos_idx_, jvel_idx_;
  double base_height_ = 0.0;                  // seed height (p0 = [0, 0, base_height])
  std::string odom_frame_ = "world";
  std::string base_frame_ = "base";

  // IMU arrives on the node's executor thread; hand it to the RT update() via a buffer.
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> imu_sub_;
  realtime_tools::RealtimeBuffer<sensor_msgs::msg::Imu> imu_buffer_;
  bool have_imu_ = false;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> rt_odom_;

  // Preallocated per-tick buffers.
  Eigen::VectorXd q_joints_, v_joints_;
  std::vector<double> contact_raw_;
  std::vector<uint8_t> stance_;
  bool seeded_ = false;
};

}  // namespace kontrolem_ros2_control

#endif  // KONTROLEM_ROS2_CONTROL__BASE_ESTIMATOR_CONTROLLER_HPP_
