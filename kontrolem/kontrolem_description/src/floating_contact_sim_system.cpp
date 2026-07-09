#include "kontrolem_description/floating_contact_sim_system.hpp"

#include <sstream>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logging.hpp"

namespace kontrolem_description
{
using hardware_interface::CallbackReturn;
using hardware_interface::CommandInterface;
using hardware_interface::StateInterface;
using hardware_interface::return_type;

namespace
{
rclcpp::Logger logger() { return rclcpp::get_logger("FloatingContactSimSystem"); }

std::vector<std::string> split_csv(const std::string & s)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    // trim surrounding whitespace
    const auto b = item.find_first_not_of(" \t");
    const auto e = item.find_last_not_of(" \t");
    if (b != std::string::npos) out.push_back(item.substr(b, e - b + 1));
  }
  return out;
}
}  // namespace

CallbackReturn FloatingContactSimSystem::on_init(const hardware_interface::HardwareInfo & info)
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
  const int nv = model_->nv();

  // Hardware parameters: contact frames (required) + nominal base height.
  const auto & hp = info_.hardware_parameters;
  if (hp.count("contact_frames") == 0) {
    RCLCPP_ERROR(logger(), "hardware parameter 'contact_frames' is required");
    return CallbackReturn::ERROR;
  }
  feet_ = split_csv(hp.at("contact_frames"));
  base_height_ = hp.count("base_height") ? std::stod(hp.at("base_height")) : 0.0;
  if (hp.count("baumgarte_kp")) kp_baum_ = std::stod(hp.at("baumgarte_kp"));
  if (hp.count("baumgarte_kd")) kd_baum_ = std::stod(hp.at("baumgarte_kd"));

  // Standing configuration: base lifted to base_height, actuated joints seeded
  // from their <state_interface initial_value> (the nominal posture).
  q_ = model_->neutral();
  q_(2) = base_height_;
  v_ = Eigen::VectorXd::Zero(nv);
  tau_ = Eigen::VectorXd::Zero(nv);
  for (const auto & j : info_.joints) {
    joint_names_.push_back(j.name);
    act_q_.push_back(model_->joint_q_index(j.name));
    act_v_.push_back(model_->joint_v_index(j.name));
    for (const auto & si : j.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION && !si.initial_value.empty()) {
        q_(model_->joint_q_index(j.name)) = std::stod(si.initial_value);
      }
    }
  }

  // Pin the feet where they start.
  anchors_.clear();
  for (const auto & f : feet_) anchors_.push_back(model_->frame_position(q_, f));

  base_.assign(13, 0.0);
  jpos_.assign(joint_names_.size(), 0.0);
  jvel_.assign(joint_names_.size(), 0.0);
  jcmd_.assign(joint_names_.size(), 0.0);
  contact_.assign(feet_.size(), 1.0);  // ground-truth all-stance
  for (std::size_t i = 0; i < joint_names_.size(); ++i) jpos_[i] = q_(act_q_[i]);
  ws_.emplace(model_->make_workspace());

  RCLCPP_INFO(logger(), "floating contact sim initialized (nq=%d nv=%d, %zu joints, %zu feet)",
              model_->nq(), nv, joint_names_.size(), feet_.size());
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> FloatingContactSimSystem::export_state_interfaces()
{
  std::vector<StateInterface> ifaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & si : info_.joints[i].state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) {
        ifaces.emplace_back(info_.joints[i].name, si.name, &jpos_[i]);
      } else if (si.name == hardware_interface::HW_IF_VELOCITY) {
        ifaces.emplace_back(info_.joints[i].name, si.name, &jvel_[i]);
      }
    }
  }
  // <gpio> blocks: the base pose/twist (13) and per-foot contact scalars. Bind by
  // interface order within each block ("contact" -> contact_, else -> base_).
  for (const auto & gpio : info_.gpios) {
    const bool is_contact = gpio.name.find("contact") != std::string::npos;
    std::size_t k = 0;
    for (const auto & si : gpio.state_interfaces) {
      if (is_contact) {
        if (k < contact_.size()) ifaces.emplace_back(gpio.name, si.name, &contact_[k]);
      } else {
        if (k < base_.size()) ifaces.emplace_back(gpio.name, si.name, &base_[k]);
      }
      ++k;
    }
  }
  return ifaces;
}

std::vector<CommandInterface> FloatingContactSimSystem::export_command_interfaces()
{
  std::vector<CommandInterface> ifaces;
  cmd_iface_names_.clear();
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & ci : info_.joints[i].command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_EFFORT) {
        ifaces.emplace_back(info_.joints[i].name, ci.name, &jcmd_[i]);
        cmd_iface_names_.push_back(info_.joints[i].name + "/" + ci.name);
      }
    }
  }
  return ifaces;
}

return_type FloatingContactSimSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  const auto claims = [&](const std::vector<std::string> & xs) {
    for (const auto & x : xs) {
      for (const auto & name : cmd_iface_names_) {
        if (x == name) return true;
      }
    }
    return false;
  };
  if (claims(start_interfaces)) {
    commanding_ = true;
    RCLCPP_INFO(logger(), "controller took command — releasing the plant");
  } else if (claims(stop_interfaces)) {
    commanding_ = false;
  }
  return return_type::OK;
}

return_type FloatingContactSimSystem::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!commanding_) {
    return return_type::OK;  // hold the standing posture until a controller commands
  }
  double dt = period.seconds();
  if (!(dt > 0.0) || dt > 0.05) dt = 0.0;

  // Actuated generalized force from the joint effort commands (base rows stay 0).
  tau_.setZero();
  for (std::size_t i = 0; i < joint_names_.size(); ++i) tau_(act_v_[i]) = jcmd_[i];

  // Contact-constrained forward dynamics: the feet stay pinned (this is the
  // ground). Baumgarte keeps them at the anchors despite integration drift.
  const Eigen::VectorXd a =
    model_->contact_forward_dynamics(*ws_, q_, v_, tau_, feet_, anchors_, kp_baum_, kd_baum_);
  if (!a.allFinite()) {
    static rclcpp::Clock clock(RCL_STEADY_TIME);
    RCLCPP_ERROR_THROTTLE(logger(), clock, 1000, "non-finite acceleration; freezing sim");
    return return_type::OK;
  }
  v_.noalias() += a * dt;
  q_ = model_->integrate(q_, v_, dt);  // manifold-correct SE(3) update

  base_[0] = q_(0); base_[1] = q_(1); base_[2] = q_(2);
  base_[3] = q_(3); base_[4] = q_(4); base_[5] = q_(5); base_[6] = q_(6);
  base_[7] = v_(0); base_[8] = v_(1); base_[9] = v_(2);
  base_[10] = v_(3); base_[11] = v_(4); base_[12] = v_(5);
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    jpos_[i] = q_(act_q_[i]);
    jvel_[i] = v_(act_v_[i]);
  }
  return return_type::OK;
}

return_type FloatingContactSimSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return return_type::OK;  // command interfaces write straight into jcmd_
}

}  // namespace kontrolem_description

PLUGINLIB_EXPORT_CLASS(
  kontrolem_description::FloatingContactSimSystem, hardware_interface::SystemInterface)
