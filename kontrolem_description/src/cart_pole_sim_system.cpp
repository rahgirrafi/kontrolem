#include "kontrolem_description/cart_pole_sim_system.hpp"

#include <algorithm>
#include <cmath>

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
rclcpp::Logger logger() { return rclcpp::get_logger("CartPoleSimSystem"); }
}  // namespace

CallbackReturn CartPoleSimSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (info.original_xml.empty()) {
    RCLCPP_ERROR(logger(), "no URDF (original_xml) available to build the sim model");
    return CallbackReturn::ERROR;
  }

  try {
    model_ = kontrolem_model::RobotModel::from_urdf_string(info.original_xml);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "failed to build RobotModel from URDF: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // This simple integrator assumes a Euclidean configuration (nq == nv): no
  // floating base, no continuous (unbounded) joints. That is exactly the M0
  // cart-pole; anything else needs the manifold-aware integrate() (post-M3).
  if (model_->nq() != model_->nv()) {
    RCLCPP_ERROR(
      logger(), "CartPoleSimSystem supports Euclidean joints only (nq=%d, nv=%d)",
      model_->nq(), model_->nv());
    return CallbackReturn::ERROR;
  }
  nv_ = model_->nv();

  pos_.assign(nv_, 0.0);
  vel_.assign(nv_, 0.0);
  eff_cmd_.assign(nv_, 0.0);
  q_ = Eigen::VectorXd::Zero(nv_);
  v_ = Eigen::VectorXd::Zero(nv_);
  tau_ = Eigen::VectorXd::Zero(nv_);
  a_ = Eigen::VectorXd::Zero(nv_);

  // Seed the simulation from the URDF's <state_interface> initial_value params
  // (the same convention ros2_control's mock_components use).
  for (const auto & joint : info_.joints) {
    const int m = model_index_of(joint.name);
    if (m < 0) {
      RCLCPP_ERROR(logger(), "joint '%s' in <ros2_control> not found in the model", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    for (const auto & si : joint.state_interfaces) {
      if (si.initial_value.empty()) {
        continue;
      }
      const double v = std::stod(si.initial_value);
      if (si.name == hardware_interface::HW_IF_POSITION) {
        pos_[static_cast<std::size_t>(m)] = v;
      } else if (si.name == hardware_interface::HW_IF_VELOCITY) {
        vel_[static_cast<std::size_t>(m)] = v;
      }
    }
  }

  // Optional scheduled disturbance (all default 0 => disabled).
  const auto param = [&](const char * key, double def) {
    const auto it = info_.hardware_parameters.find(key);
    return it != info_.hardware_parameters.end() ? std::stod(it->second) : def;
  };
  disturb_time_ = param("disturb_time", 0.0);
  disturb_duration_ = param("disturb_duration", 0.0);
  disturb_tau_ = param("disturb_tau", 0.0);
  disturb_dof_ = static_cast<int>(param("disturb_dof", 1.0));

  RCLCPP_INFO(logger(), "cart-pole sim initialized (%d DoF, ABA integrator)", nv_);
  return CallbackReturn::SUCCESS;
}

int CartPoleSimSystem::model_index_of(const std::string & joint_name) const
{
  const auto & names = model_->joint_names();
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (names[i] == joint_name) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

std::vector<StateInterface> CartPoleSimSystem::export_state_interfaces()
{
  std::vector<StateInterface> ifaces;
  for (const auto & joint : info_.joints) {
    const auto m = static_cast<std::size_t>(model_index_of(joint.name));
    for (const auto & si : joint.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) {
        ifaces.emplace_back(joint.name, si.name, &pos_[m]);
      } else if (si.name == hardware_interface::HW_IF_VELOCITY) {
        ifaces.emplace_back(joint.name, si.name, &vel_[m]);
      }
    }
  }
  return ifaces;
}

std::vector<CommandInterface> CartPoleSimSystem::export_command_interfaces()
{
  std::vector<CommandInterface> ifaces;
  cmd_iface_names_.clear();
  for (const auto & joint : info_.joints) {
    const auto m = static_cast<std::size_t>(model_index_of(joint.name));
    for (const auto & ci : joint.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_EFFORT) {
        ifaces.emplace_back(joint.name, ci.name, &eff_cmd_[m]);
        cmd_iface_names_.push_back(joint.name + "/" + ci.name);
      }
    }
  }
  return ifaces;
}

return_type CartPoleSimSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  const auto claims = [&](const std::vector<std::string> & xs) {
    for (const auto & x : xs) {
      for (const auto & name : cmd_iface_names_) {
        if (x == name) {
          return true;
        }
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

return_type CartPoleSimSystem::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  // Hold the plant at its initial state until a controller is commanding it, so
  // an unstable robot does not free-fall during controller_manager bring-up.
  if (!commanding_) {
    return return_type::OK;
  }

  double dt = period.seconds();
  // Guard the first tick / a stalled clock: keep the integrator stable.
  if (!(dt > 0.0) || dt > 0.05) {
    dt = 0.0;
  }

  for (int i = 0; i < nv_; ++i) {
    q_(i) = pos_[static_cast<std::size_t>(i)];
    v_(i) = vel_[static_cast<std::size_t>(i)];
    tau_(i) = eff_cmd_[static_cast<std::size_t>(i)];  // unactuated coords stay 0
  }

  // Scheduled external disturbance: add a torque to one DoF during its window.
  elapsed_ += dt;
  if (disturb_duration_ > 0.0 && disturb_dof_ >= 0 && disturb_dof_ < nv_ &&
      elapsed_ >= disturb_time_ && elapsed_ < disturb_time_ + disturb_duration_) {
    tau_(disturb_dof_) += disturb_tau_;
  }

  // Forward dynamics of the real robot, then semi-implicit (symplectic) Euler.
  a_ = model_->aba(q_, v_, tau_);
  if (!a_.allFinite()) {
    static rclcpp::Clock clock(RCL_STEADY_TIME);
    RCLCPP_ERROR_THROTTLE(logger(), clock, 1000, "non-finite acceleration; freezing sim");
    return return_type::OK;
  }
  v_.noalias() += a_ * dt;
  q_.noalias() += v_ * dt;

  for (int i = 0; i < nv_; ++i) {
    pos_[static_cast<std::size_t>(i)] = q_(i);
    vel_[static_cast<std::size_t>(i)] = v_(i);
  }
  return return_type::OK;
}

return_type CartPoleSimSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Command interfaces write straight into eff_cmd_ (they hold pointers into it),
  // so the value the controller set this cycle is already staged for the next
  // read(). Nothing to copy here.
  return return_type::OK;
}

}  // namespace kontrolem_description

PLUGINLIB_EXPORT_CLASS(kontrolem_description::CartPoleSimSystem, hardware_interface::SystemInterface)
