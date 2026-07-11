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
// contact <gpio> blocks that BaseStateSensor/ContactSensor claim. Per-foot contact is
// read REAL from the ECM (M8): the world's ignition-gazebo-contact-system evaluates
// each foot's <sensor type="contact"> and this component reads the resulting
// ContactSensorData (a foot is in stance iff it has >=1 contact point), so a foot that
// lifts under a push reports 0 — the sensing a real robot has. (M7 faked constant
// all-stance; the estimator work — where contact drives leg odometry — needs it real.)
#ifndef KONTROLEM_GZ__GZ_BASE_STATE_SYSTEM_HPP_
#define KONTROLEM_GZ__GZ_BASE_STATE_SYSTEM_HPP_

#include <array>
#include <atomic>
#include <string>
#include <vector>

#include "gz_ros2_control/gz_system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "realtime_tools/realtime_buffer.h"

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

  // Lazily resolve each foot's contact-sensor entity from the ECM (they exist only
  // once the model + sensor systems have spawned). Returns true once all are bound.
  bool resolve_contact_sensors();

  std::array<double, 13> base_{0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0};  // pose, quat xyzw, lin, ang
  std::vector<double> contact_;   // per-foot stance scalars (1 = in contact)
  std::vector<std::string> contact_frames_;  // foot link names, in gpio/contact_ order
  std::vector<sim::Entity> contact_sensors_; // parallel: entity carrying each foot's ContactSensorData
  bool contacts_resolved_ = false;
  int contact_resolve_attempts_ = 0;   // bound the lazy re-scan (sensors appear late)
  std::string model_name_;
  std::string base_link_name_;

  sim::EntityComponentManager * ecm_ = nullptr;
  sim::Entity base_entity_ = sim::kNullEntity;

  // Base pose/twist SOURCE (M8). ecm: simulator ground truth (sim-only). estimate: the
  // BaseEstimatorController's /base_odom — the sim-to-real path where the WBC stands on
  // its OWN estimate. Contact is ALWAYS real from the ECM; only the base differs.
  enum class BaseSource { Ecm, Estimate };
  BaseSource base_source_ = BaseSource::Ecm;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::msg::Odometry> odom_buffer_;
  std::atomic<bool> have_odom_{false};   // seen >=1 /base_odom (else bootstrap from ECM)
};

}  // namespace kontrolem_gz

#endif  // KONTROLEM_GZ__GZ_BASE_STATE_SYSTEM_HPP_
