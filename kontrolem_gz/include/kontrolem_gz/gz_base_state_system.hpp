// GzBaseStateSystem — a gz_ros2_control hardware component (a GazeboSimSystemInterface,
// so the in-gz controller_manager will load it alongside IgnitionSystem) that exports
// a floating base's non-joint state DIRECTLY from Gazebo's ECM: base_link's
// ground-truth world pose + body-frame twist as the 13 floating_base scalar state
// interfaces, plus per-foot contact scalars. This is Part B4's intended source #1
// (sim ground truth via gz interfaces) — and, unlike the M6.3 odometry-topic bridge,
// it reads the ECM in-process with no topic round-trip, so the 500 Hz WBC gets fresh
// base state every tick.
//
// It carries NO joints (IgnitionSystem owns those); it only reassembles the base +
// contact <gpio> blocks that BaseStateSensor/ContactSensor claim. Contact is a
// constant all-stance (=1) for the standing case — the same ground-truth assumption
// FloatingContactSimSystem makes — which keeps the friction test about the WBC's
// forces meeting Gazebo's contact solver, not about contact DETECTION.
#ifndef KONTROLEM_GZ__GZ_BASE_STATE_SYSTEM_HPP_
#define KONTROLEM_GZ__GZ_BASE_STATE_SYSTEM_HPP_

#include <array>
#include <string>
#include <vector>

#include "gz_ros2_control/gz_system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace kontrolem_gz
{

class GzBaseStateSystem : public gz_ros2_control::GazeboSimSystemInterface
{
public:
  // gz_ros2_control hook: grab the ECM + resolve the base link entity.
  bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    std::map<std::string, sim::Entity> & joints,
    const hardware_interface::HardwareInfo & hardware_info,
    sim::EntityComponentManager & ecm,
    int & update_rate) override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::array<double, 13> base_{0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0};  // pose, quat xyzw, lin, ang
  std::vector<double> contact_;   // per-foot stance scalars (constant 1)
  std::string model_name_;
  std::string base_link_name_;

  sim::EntityComponentManager * ecm_ = nullptr;
  sim::Entity base_entity_ = sim::kNullEntity;
};

}  // namespace kontrolem_gz

#endif  // KONTROLEM_GZ__GZ_BASE_STATE_SYSTEM_HPP_
