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

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kontrolem_control/base_reference.hpp"
#include "kontrolem_control/controller.hpp"
#include "kontrolem_control/controller_factory.hpp"
#include "kontrolem_control/params.hpp"
#include "kontrolem_control/supervisor.hpp"
#include "pluginlib/class_loader.hpp"
#include "kontrolem_model/robot_model.hpp"
#include "kontrolem_ros2_control/base_state_sensor.hpp"
#include "kontrolem_ros2_control/contact_sensor.hpp"
#include "kontrolem_msgs/msg/controller_diagnostics.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include <array>

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
  // M16 controller registry: control laws are pluginlib plugins discovered by
  // name (their ControllerFactory::name()), NOT a hardcoded if/else. build_law()
  // looks the selected law up, declares that law's parameters generically from
  // its ParameterSpec, and constructs it — so a new controller drops in as its
  // own package with zero edits here. Both the single- and multi-law paths use it.
  std::shared_ptr<pluginlib::ClassLoader<kontrolem_control::ControllerFactory>> factory_loader_;
  std::map<std::string, std::shared_ptr<kontrolem_control::ControllerFactory>> factories_;
  std::unique_ptr<kontrolem_control::Controller> build_law(const std::string & law);
  // Declare one parameter from its ParamDesc (dispatching on its variant type)
  // and return the read value; used by build_law over a law's whole spec.
  kontrolem_control::ParamValue declare_law_param(const kontrolem_control::ParamDesc & desc);
  // Config validation (the M16 typo-catch): reject an override under a law's
  // parameter prefix that the law does not declare (e.g. wbc.frction), before the
  // robot moves. Throws with a clear message; runtime-owned params are exempt.
  void validate_law_overrides(
    const std::string & law, const kontrolem_control::ParameterSpec & spec);

  // Core (ROS-free) objects.
  std::optional<kontrolem_model::RobotModel> model_;
  std::unique_ptr<kontrolem_control::Controller> law_;             // single-law mode
  std::unique_ptr<kontrolem_control::ControlProblem> problem_;      // Regulation/Tracking/Locomotion
  std::unique_ptr<kontrolem_control::TrajectorySource> reference_;  // owned when Tracking
  std::unique_ptr<kontrolem_control::GaitSource> gait_;             // owned when Locomotion
  kontrolem_control::State state_;
  rclcpp::Time start_time_;  // set on_activate; drives State::t for Tracking

  // Startup settle-gate (M15): a floating-base gait must not begin stepping on a fixed
  // wall-clock, because that clock is unsynchronized with the Gazebo weld-release and the
  // first diagonal swing then fires mid-transient and tips the robot off its support line
  // (the M12 startup-timing race, confirmed on the WBC trot). When enabled, the gait runs on
  // a clock ZEROED at the moment the base is measured to have settled AFTER the release
  // transient. Opt-in (gait.settle_gate); applies only to Locomotion on a floating base, so
  // Regulation/Tracking (standing/posture) and the fixed-base path are untouched.
  bool settle_gate_{false};
  bool gait_released_{false};      // the gate has released; gait clock is running
  bool release_seen_{false};       // the weld-release transient has been observed
  double t_release_{0.0};          // state_.t at gate release (gait clock origin)
  double settle_accum_{0.0};       // sustained-settle timer (s)
  double settle_release_speed_{0.08};  // base speed (m/s) that marks the release transient
  double settle_speed_{0.04};      // base speed (m/s) below which "settled"
  double settle_tilt_{0.12};       // base tilt (rad) below which "settled"
  double settle_hold_{0.3};        // settle must persist this long (s) before stepping
  double settle_timeout_{15.0};    // hard fallback (s): step anyway if never detected

  // Config.
  std::vector<std::string> actuated_joints_;
  std::string command_interface_{"effort"};
  std::string pos_interface_{"position"};
  std::string vel_interface_{"velocity"};
  std::string safe_action_{"zero"};
  bool needs_velocity_{true};  // from the law's capabilities(); if false, claim positions only

  // Multi-controller mode (control_laws non-empty): the Supervisor hosts several
  // laws and drives one; a human commands the switch over ~/switch_controller
  // (std_msgs/String, the target law name). The single-law path above is untouched
  // when control_laws is empty. Automatic (status-driven) switching is a later layer.
  bool multi_{false};
  std::unique_ptr<kontrolem_control::Supervisor> supervisor_;
  std::vector<std::unique_ptr<kontrolem_control::Controller>> laws_;  // owned hosted laws
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr switch_sub_;
  std::mutex switch_mtx_;        // guards switch_request_ (topic thread vs RT update)
  std::string switch_request_;   // pending target name from the topic ("" = none)

  // M9 live posture command: ~/base_target (geometry_msgs/Twist) -> a base offset
  // [x,y,z, roll,pitch,yaw] the update() loop copies into the LiveBaseTarget each tick.
  kontrolem_control::LiveBaseTarget * live_target_ = nullptr;  // non-owning (owned via reference_)
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr base_target_sub_;
  realtime_tools::RealtimeBuffer<std::array<double, 6>> base_target_buffer_;

  // Floating-base (WBC) path: non-joint state (SE(3) base + contacts) enters via
  // <gpio> interfaces reassembled by the semantic components, and the actuated
  // joints are addressed by generalized-coordinate index (the free-flyer root is
  // not an encoder). The fixed-base path above is untouched.
  bool floating_{false};
  std::string base_gpio_{"floating_base"};
  std::string contact_gpio_{"contact"};
  std::vector<std::string> feet_;                 // contact frame names (WBC + ContactSensor)
  std::optional<BaseStateSensor> base_sensor_;
  std::optional<ContactSensor> contact_sensor_;
  std::vector<int> act_q_idx_, act_v_idx_;        // generalized q/v index per actuated joint
  std::vector<std::size_t> jpos_idx_, jvel_idx_;  // state-iface index per actuated joint
  std::vector<double> contact_buf_;               // scratch for reading contact scalars

  // Telemetry (opt-in, RT-safe): per-tick ControllerDiagnostics on ~/diagnostics.
  bool publish_diagnostics_{false};
  std::string control_law_;
  Eigen::VectorXd qref_buf_;  // reference snapshot for the message
  kontrolem_control::GaitPlan gait_plan_buf_;  // gait snapshot for the message (Locomotion)
  using DiagMsg = kontrolem_msgs::msg::ControllerDiagnostics;
  std::shared_ptr<rclcpp::Publisher<DiagMsg>> diag_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<DiagMsg>> rt_diag_;

  // Planned contact schedule (opt-in, RT-safe): when running a Locomotion problem the gait
  // KNOWS which foot is planted each tick. Publishing that lets the state estimator use the
  // PLAN instead of the flickering sensed contact (M12) — the fix for reliable walk-on-estimate.
  bool publish_planned_contact_{false};
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> pc_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> rt_pc_;

  // Interface indices resolved by name in on_activate().
  std::vector<std::size_t> pos_idx_;  // per model joint
  std::vector<std::size_t> vel_idx_;  // per model joint
  std::vector<std::size_t> cmd_idx_;  // per actuated joint
};

}  // namespace kontrolem_ros2_control

#endif  // KONTROLEM_ROS2_CONTROL__KONTROLEM_CONTROLLER_HPP_
