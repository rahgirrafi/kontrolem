// FloatingStateProbe — M3 spike controller. It does NO control: it claims the
// floating base's scalar <gpio> state interfaces plus the joint states,
// reassembles a manifold-correct State (normalized quaternion), and reports the
// base height / CoM / quaternion norm. Its job is to prove the round-trip
// SE(3) pose -> scalar interfaces -> reassembled State works through ros2_control,
// and to expose how awkward it is (the plan's flagged compromise).
#ifndef KONTROLEM_ROS2_CONTROL__FLOATING_STATE_PROBE_HPP_
#define KONTROLEM_ROS2_CONTROL__FLOATING_STATE_PROBE_HPP_

#include <optional>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "kontrolem_control/types.hpp"
#include "kontrolem_model/robot_model.hpp"
#include "kontrolem_ros2_control/base_state_sensor.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace kontrolem_ros2_control
{

class FloatingStateProbe : public controller_interface::ControllerInterface
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
  kontrolem_control::State state_;
  std::optional<BaseStateSensor> base_sensor_;      // encapsulates base reassembly
  std::vector<std::string> joint_names_;            // actuated joints (no root); may be empty (base-only)
  std::vector<std::size_t> jpos_idx_, jvel_idx_;    // per actuated joint

  // Frame-consistency self-check (M6.3): predict this tick's configuration by
  // manifold-integrating the previous reassembled (q,v) and compare to the newly
  // reassembled q. A large residual means the producer's twist frame / quaternion
  // order disagrees with Pinocchio's free-flyer — the exact convention risk the
  // Gazebo round-trip exists to catch. Published on ~/base_state so an e2e can
  // assert it. Layout: [pos(3), quat xyzw(4), lin(3), ang(3), quat_norm,
  // frame_residual, com(3)] = 16 doubles.
  Eigen::VectorXd prev_q_, prev_v_;
  bool have_prev_ = false;
  double max_frame_resid_ = 0.0;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> pub_;
};

}  // namespace kontrolem_ros2_control

#endif  // KONTROLEM_ROS2_CONTROL__FLOATING_STATE_PROBE_HPP_
