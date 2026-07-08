#include "kontrolem_description/floating_base_sim_system.hpp"

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace kontrolem_description
{
using hardware_interface::CallbackReturn;
using hardware_interface::CommandInterface;
using hardware_interface::StateInterface;
using hardware_interface::return_type;

namespace
{
rclcpp::Logger logger() { return rclcpp::get_logger("FloatingBaseSimSystem"); }
}  // namespace

CallbackReturn FloatingBaseSimSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (info.original_xml.empty()) {
    RCLCPP_ERROR(logger(), "no URDF (original_xml) to build the floating model");
    return CallbackReturn::ERROR;
  }
  try {
    model_ = kontrolem_model::RobotModel::from_urdf_string(
      info.original_xml, kontrolem_model::BaseType::kFloating);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "failed to build floating RobotModel: %s", e.what());
    return CallbackReturn::ERROR;
  }
  const int nq = model_->nq(), nv = model_->nv();

  q_ = model_->neutral();
  q_(2) = 1.0;                       // start the base 1 m up so it falls visibly
  v_ = Eigen::VectorXd::Zero(nv);
  tau_ = Eigen::VectorXd::Zero(nv);

  base_.assign(13, 0.0);
  for (const auto & j : info_.joints) {
    hip_names_.push_back(j.name);
  }
  hip_pos_.assign(hip_names_.size(), 0.0);
  hip_vel_.assign(hip_names_.size(), 0.0);
  hip_cmd_.assign(hip_names_.size(), 0.0);

  RCLCPP_INFO(logger(), "floating sim initialized (nq=%d nv=%d, %zu joints + base gpio)",
              nq, nv, hip_names_.size());
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> FloatingBaseSimSystem::export_state_interfaces()
{
  std::vector<StateInterface> ifaces;
  // Joint states.
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & si : info_.joints[i].state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) {
        ifaces.emplace_back(info_.joints[i].name, si.name, &hip_pos_[i]);
      } else if (si.name == hardware_interface::HW_IF_VELOCITY) {
        ifaces.emplace_back(info_.joints[i].name, si.name, &hip_vel_[i]);
      }
    }
  }
  // Base scalars: bind the <gpio> state interfaces in URDF order to base_[0..12]
  // (the URDF lists pose.* then twist.* — the same order read() fills).
  for (const auto & gpio : info_.gpios) {
    std::size_t k = 0;
    for (const auto & si : gpio.state_interfaces) {
      if (k < base_.size()) {
        ifaces.emplace_back(gpio.name, si.name, &base_[k]);
        ++k;
      }
    }
  }
  return ifaces;
}

std::vector<CommandInterface> FloatingBaseSimSystem::export_command_interfaces()
{
  std::vector<CommandInterface> ifaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & ci : info_.joints[i].command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_EFFORT) {
        ifaces.emplace_back(info_.joints[i].name, ci.name, &hip_cmd_[i]);
      }
    }
  }
  return ifaces;
}

return_type FloatingBaseSimSystem::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  double dt = period.seconds();
  if (!(dt > 0.0) || dt > 0.05) {
    dt = 0.0;
  }

  // Generalized force: base rows (0..5) unactuated; joints take their effort cmd.
  tau_.setZero();
  for (std::size_t i = 0; i < hip_names_.size(); ++i) {
    tau_(6 + static_cast<Eigen::Index>(i)) = hip_cmd_[i];  // nv layout: [6 base; joints]
  }

  const Eigen::VectorXd a = model_->aba(q_, v_, tau_);
  if (a.allFinite()) {
    v_.noalias() += a * dt;
    q_ = model_->integrate(q_, v_, dt);  // manifold-correct SE(3) update
  }

  // Publish base pose/twist into the scalar interface storage.
  base_[0] = q_(0); base_[1] = q_(1); base_[2] = q_(2);          // position
  base_[3] = q_(3); base_[4] = q_(4); base_[5] = q_(5); base_[6] = q_(6);  // quat xyzw
  base_[7] = v_(0); base_[8] = v_(1); base_[9] = v_(2);          // linear
  base_[10] = v_(3); base_[11] = v_(4); base_[12] = v_(5);       // angular
  for (std::size_t i = 0; i < hip_names_.size(); ++i) {
    hip_pos_[i] = q_(7 + static_cast<Eigen::Index>(i));
    hip_vel_[i] = v_(6 + static_cast<Eigen::Index>(i));
  }
  return return_type::OK;
}

return_type FloatingBaseSimSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return return_type::OK;  // command interfaces write straight into hip_cmd_
}

}  // namespace kontrolem_description

PLUGINLIB_EXPORT_CLASS(
  kontrolem_description::FloatingBaseSimSystem, hardware_interface::SystemInterface)
