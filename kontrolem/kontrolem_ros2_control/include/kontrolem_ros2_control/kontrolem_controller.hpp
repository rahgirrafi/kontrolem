// KontrolemController — the Layer-4 ros2_control boundary.
//
// A single ros2_control controller that HOSTS any kontrolem_control::Controller
// plugin (LQR, QP, ... selected by parameter). It maps ros2_control interfaces
// to/from the ROS-free core:
//   state interfaces (joint pos/vel)  ->  kontrolem_control::State
//   controller_->compute(state, problem, dt)  ->  Command
//   Command.tau  ->  command interfaces (effort on actuated joints)
// plus a minimal Supervisor: on a non-ok status() it applies a safe action.
//
// This is the ONLY package that knows about both ROS and the core — the ROS
// boundary lives here and nowhere below.
#ifndef KONTROLEM_ROS2_CONTROL__KONTROLEM_CONTROLLER_HPP_
#define KONTROLEM_ROS2_CONTROL__KONTROLEM_CONTROLLER_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "kontrolem_control/controller.hpp"
#include "kontrolem_model/robot_model.hpp"
#include "kontrolem_msgs/msg/controller_diagnostics.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.h"

namespace kontrolem_ros2_control
{

class KontrolemController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Core (ROS-free) objects.
  std::optional<kontrolem_model::RobotModel> model_;
  std::unique_ptr<kontrolem_control::Controller> law_;
  std::unique_ptr<kontrolem_control::ControlProblem> problem_;      // Regulation or Tracking
  std::unique_ptr<kontrolem_control::TrajectorySource> reference_;  // owned when Tracking
  kontrolem_control::State state_;
  rclcpp::Time start_time_;  // set on_activate; drives State::t for Tracking

  // Config.
  std::vector<std::string> actuated_joints_;
  std::string command_interface_{"effort"};
  std::string pos_interface_{"position"};
  std::string vel_interface_{"velocity"};
  std::string safe_action_{"zero"};
  bool needs_velocity_{true};  // from the law's capabilities(); if false, claim positions only

  // Telemetry (opt-in, RT-safe): per-tick ControllerDiagnostics on ~/diagnostics.
  bool publish_diagnostics_{false};
  std::string control_law_;
  Eigen::VectorXd qref_buf_;  // reference snapshot for the message
  using DiagMsg = kontrolem_msgs::msg::ControllerDiagnostics;
  std::shared_ptr<rclcpp::Publisher<DiagMsg>> diag_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<DiagMsg>> rt_diag_;

  // Interface indices resolved by name in on_activate().
  std::vector<std::size_t> pos_idx_;  // per model joint
  std::vector<std::size_t> vel_idx_;  // per model joint
  std::vector<std::size_t> cmd_idx_;  // per actuated joint
};

}  // namespace kontrolem_ros2_control

#endif  // KONTROLEM_ROS2_CONTROL__KONTROLEM_CONTROLLER_HPP_
