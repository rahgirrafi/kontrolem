// FloatingContactSimSystem — the standing-WBC sim plant (M4.4). Unlike
// FloatingBaseSimSystem (which free-falls via aba, the M3 spike), this integrates
// CONTACT-CONSTRAINED forward dynamics: the feet are pinned to the ground (the
// model's contact_forward_dynamics with Baumgarte stabilization), so a WBC's
// torques can actually hold the robot up. It exports the SE(3) base as 13 scalar
// <gpio> interfaces AND per-foot CONTACT scalars (ground-truth stance = 1.0), and
// holds the plant at its standing posture until a controller commands it (the D11
// bring-up fix). Contact is scheduled all-stance — no locomotion (Part D).
#ifndef KONTROLEM_DESCRIPTION__FLOATING_CONTACT_SIM_SYSTEM_HPP_
#define KONTROLEM_DESCRIPTION__FLOATING_CONTACT_SIM_SYSTEM_HPP_

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "hardware_interface/system_interface.hpp"
#include "kontrolem_model/robot_model.hpp"

namespace kontrolem_description
{

class FloatingContactSimSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::optional<kontrolem_model::RobotModel> model_;
  std::optional<kontrolem_model::RobotModel::Workspace> ws_;
  Eigen::VectorXd q_, v_, tau_;                 // nq, nv, nv (base rows unactuated)

  std::vector<std::string> feet_;               // contact frame names
  std::vector<Eigen::Vector3d> anchors_;        // pinned foot positions (captured at init)
  std::vector<std::string> joint_names_;        // actuated joints (URDF order)
  std::vector<int> act_q_, act_v_;              // generalized q/v index per actuated joint

  // Interface-backing storage.
  std::vector<double> base_;                    // 13: pose(7) + twist(6)
  std::vector<double> jpos_, jvel_, jcmd_;      // per actuated joint
  std::vector<double> contact_;                 // per foot (=1.0, stance)
  std::vector<std::string> cmd_iface_names_;

  bool commanding_{false};
  double base_height_{0.0};
  double kp_baum_{400.0}, kd_baum_{40.0};       // critically-damped contact stabilization
};

}  // namespace kontrolem_description

#endif  // KONTROLEM_DESCRIPTION__FLOATING_CONTACT_SIM_SYSTEM_HPP_
