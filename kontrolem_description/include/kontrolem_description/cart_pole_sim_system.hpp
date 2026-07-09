// CartPoleSimSystem — a self-contained ros2_control simulation hardware.
//
// A hardware_interface::SystemInterface that does NOT talk to real hardware or
// Gazebo: it integrates the true rigid-body dynamics of the robot in its own
// URDF using kontrolem_model's ABA (forward dynamics). This lets the whole M0
// loop — controller_manager -> KontrolemController -> command interface ->
// this sim -> state interface -> back — run in one process so we can watch the
// pole balance before bringing in a full physics simulator.
//
// It is deliberately generic over the URDF (any fixed-base robot whose actuated
// joints expose an `effort` command interface); the "cart-pole" naming reflects
// its M0 role, not a hardcoded model.
#ifndef KONTROLEM_DESCRIPTION__CART_POLE_SIM_SYSTEM_HPP_
#define KONTROLEM_DESCRIPTION__CART_POLE_SIM_SYSTEM_HPP_

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "hardware_interface/system_interface.hpp"
#include "kontrolem_model/robot_model.hpp"

namespace kontrolem_description
{

class CartPoleSimSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Command-mode switching: used to detect when a controller takes command of
  // our effort interface, so the plant stays held until control engages.
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::optional<kontrolem_model::RobotModel> model_;
  int nv_{0};

  // Simulation state, indexed by MODEL generalized-coordinate index.
  std::vector<double> pos_;      ///< q  (exported as position state interfaces)
  std::vector<double> vel_;      ///< v  (exported as velocity state interfaces)
  std::vector<double> eff_cmd_;  ///< tau (written by command interfaces; 0 = unactuated)

  // The plant is held at its initial state until a controller claims the effort
  // command interface — otherwise an unstable robot (upright pole) free-falls
  // during controller_manager bring-up before any controller can act.
  std::vector<std::string> cmd_iface_names_;  ///< our exported command-interface full names
  bool commanding_{false};

  // Preallocated Eigen scratch so read() does not churn the heap each tick.
  Eigen::VectorXd q_, v_, tau_, a_;

  // Map a joint name to its model generalized-coordinate index.
  int model_index_of(const std::string & joint_name) const;
};

}  // namespace kontrolem_description

#endif  // KONTROLEM_DESCRIPTION__CART_POLE_SIM_SYSTEM_HPP_
