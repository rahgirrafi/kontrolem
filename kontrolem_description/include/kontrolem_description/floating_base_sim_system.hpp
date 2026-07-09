// FloatingBaseSimSystem — M3.3 spike hardware: a self-contained floating-base
// simulator that integrates the SE(3) rigid-body dynamics (aba + manifold
// integrate) and exports the base pose/twist as SCALAR ros2_control state
// interfaces on a <gpio name="floating_base"> block, alongside the joint states.
// Its whole purpose is to test whether ros2_control can carry non-joint SE(3)
// state through its interface mechanism (the plan's flagged compromise).
#ifndef KONTROLEM_DESCRIPTION__FLOATING_BASE_SIM_SYSTEM_HPP_
#define KONTROLEM_DESCRIPTION__FLOATING_BASE_SIM_SYSTEM_HPP_

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "hardware_interface/system_interface.hpp"
#include "kontrolem_model/robot_model.hpp"

namespace kontrolem_description
{

class FloatingBaseSimSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::optional<kontrolem_model::RobotModel> model_;
  Eigen::VectorXd q_, v_, tau_;  // nq, nv, nv (base rows stay 0 = unactuated)

  // Interface-backing storage (interfaces hold pointers into these).
  std::vector<double> base_;      // 13: px,py,pz, qx,qy,qz,qw, vx,vy,vz, wx,wy,wz
  std::vector<double> hip_pos_;   // per actuated joint
  std::vector<double> hip_vel_;
  std::vector<double> hip_cmd_;
  std::vector<std::string> hip_names_;  // actuated joint names in URDF order
};

}  // namespace kontrolem_description

#endif  // KONTROLEM_DESCRIPTION__FLOATING_BASE_SIM_SYSTEM_HPP_
