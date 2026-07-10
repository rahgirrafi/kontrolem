#include "kontrolem_ros2_control/floating_state_probe.hpp"

#include <algorithm>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace kontrolem_ros2_control
{
using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

CallbackReturn FloatingStateProbe::on_init()
{
  try {
    auto_declare<std::string>("robot_description", "");
    auto_declare<std::string>("base_gpio", "floating_base");
    auto_declare<std::vector<std::string>>("joints", {});
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn FloatingStateProbe::on_configure(const rclcpp_lifecycle::State &)
{
  auto & node = *get_node();
  const auto urdf = node.get_parameter("robot_description").as_string();
  if (urdf.empty()) {
    RCLCPP_ERROR(node.get_logger(), "parameter 'robot_description' is empty");
    return CallbackReturn::ERROR;
  }
  base_sensor_.emplace(node.get_parameter("base_gpio").as_string());
  joint_names_ = node.get_parameter("joints").as_string_array();
  // joints may be empty: a base-only probe (M6.3 Gazebo round-trip) validates the
  // floating base alone, with no actuated joints.
  if (joint_names_.empty()) {
    RCLCPP_INFO(node.get_logger(), "no joints declared: running base-only");
  }

  pub_ = node.create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/base_state", rclcpp::SystemDefaultsQoS());

  try {
    model_ = kontrolem_model::RobotModel::from_urdf_string(
      urdf, kontrolem_model::BaseType::kFloating);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node.get_logger(), "failed to build floating model: %s", e.what());
    return CallbackReturn::ERROR;
  }
  state_.q = model_->neutral();
  state_.v = Eigen::VectorXd::Zero(model_->nv());
  RCLCPP_INFO(node.get_logger(), "probe configured: nq=%d nv=%d, %zu joints",
              model_->nq(), model_->nv(), joint_names_.size());
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration FloatingStateProbe::command_interface_configuration() const
{
  return {interface_configuration_type::NONE, {}};  // read-only probe
}

InterfaceConfiguration FloatingStateProbe::state_interface_configuration() const
{
  InterfaceConfiguration cfg;
  cfg.type = interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joint_names_) {
    cfg.names.push_back(j + "/position");
    cfg.names.push_back(j + "/velocity");
  }
  for (const auto & n : base_sensor_->interface_names()) {
    cfg.names.push_back(n);  // base scalars (the sensor owns the convention)
  }
  return cfg;
}

CallbackReturn FloatingStateProbe::on_activate(const rclcpp_lifecycle::State &)
{
  const auto find = [&](const std::string & name) -> std::size_t {
    for (std::size_t i = 0; i < state_interfaces_.size(); ++i) {
      if (state_interfaces_[i].get_name() == name) {
        return i;
      }
    }
    return state_interfaces_.size();
  };
  jpos_idx_.clear(); jvel_idx_.clear();
  for (const auto & j : joint_names_) {
    jpos_idx_.push_back(find(j + "/position"));
    jvel_idx_.push_back(find(j + "/velocity"));
  }
  for (auto idx : jpos_idx_) if (idx == state_interfaces_.size()) return CallbackReturn::ERROR;
  for (auto idx : jvel_idx_) if (idx == state_interfaces_.size()) return CallbackReturn::ERROR;
  if (!base_sensor_->assign_loaned(state_interfaces_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "failed to bind base state interfaces");
    return CallbackReturn::ERROR;
  }
  have_prev_ = false;
  max_frame_resid_ = 0.0;
  return CallbackReturn::SUCCESS;
}

CallbackReturn FloatingStateProbe::on_deactivate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type FloatingStateProbe::update(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  // The base pose/twist reassembly (SE(3) layout + quaternion normalization) is
  // encapsulated in the semantic component; the probe just wires the joints.
  base_sensor_->read_into(state_.q, state_.v);
  for (std::size_t j = 0; j < joint_names_.size(); ++j) {
    state_.q(7 + static_cast<Eigen::Index>(j)) = state_interfaces_[jpos_idx_[j]].get_value();
    state_.v(6 + static_cast<Eigen::Index>(j)) = state_interfaces_[jvel_idx_[j]].get_value();
  }

  const Eigen::Vector3d com = model_->center_of_mass(state_.q);
  const double qnorm = state_.q.segment<4>(3).norm();

  // Frame-consistency check: predict q from the PREVIOUS (q,v) via manifold
  // integration and compare to the freshly reassembled q. If the producer's twist
  // is body-frame with an xyzw quaternion (matching Pinocchio's free-flyer), the
  // prediction tracks and the residual stays near zero; a frame/quaternion
  // mismatch makes it diverge as the base rotates/translates.
  double frame_resid = 0.0;
  const double dt = period.seconds();
  if (have_prev_ && dt > 0.0) {
    const Eigen::VectorXd predicted = model_->integrate(prev_q_, prev_v_, dt);
    frame_resid = model_->difference(predicted, state_.q).norm();
    max_frame_resid_ = std::max(max_frame_resid_, frame_resid);
  }
  prev_q_ = state_.q;
  prev_v_ = state_.v;
  have_prev_ = true;

  std_msgs::msg::Float64MultiArray msg;
  msg.data = {
    state_.q(0), state_.q(1), state_.q(2),
    state_.q(3), state_.q(4), state_.q(5), state_.q(6),
    state_.v(0), state_.v(1), state_.v(2),
    state_.v(3), state_.v(4), state_.v(5),
    qnorm, frame_resid, com.x(), com.y(), com.z()};
  pub_->publish(msg);

  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 500,
    "reassembled base_z=%.4f quat_norm=%.6f frame_resid=%.2e (max=%.2e) CoM=(%.3f,%.3f,%.3f)",
    state_.q(2), qnorm, frame_resid, max_frame_resid_, com.x(), com.y(), com.z());
  return controller_interface::return_type::OK;
}

}  // namespace kontrolem_ros2_control

PLUGINLIB_EXPORT_CLASS(
  kontrolem_ros2_control::FloatingStateProbe, controller_interface::ControllerInterface)
