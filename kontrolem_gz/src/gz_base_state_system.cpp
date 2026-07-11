#include "kontrolem_gz/gz_base_state_system.hpp"

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include "pluginlib/class_list_macros.hpp"

namespace kontrolem_gz
{
using hardware_interface::CallbackReturn;
using hardware_interface::CommandInterface;
using hardware_interface::StateInterface;
using hardware_interface::return_type;

namespace
{
rclcpp::Logger logger() { return rclcpp::get_logger("GzBaseStateSystem"); }
bool is_contact(const std::string & gpio_name) { return gpio_name.find("contact") != std::string::npos; }
}  // namespace

bool GzBaseStateSystem::initSim(
  rclcpp::Node::SharedPtr & /*model_nh*/,
  std::map<std::string, sim::Entity> & /*joints*/,
  const hardware_interface::HardwareInfo & hardware_info,
  sim::EntityComponentManager & ecm,
  int & /*update_rate*/)
{
  // gz_ros2_control calls initSim BEFORE on_init, so read our params from the
  // passed HardwareInfo here and resolve the base-link entity from the ECM.
  ecm_ = &ecm;
  const auto & hp = hardware_info.hardware_parameters;
  auto get = [&](const std::string & k, const std::string & def) {
    auto it = hp.find(k);
    return (it != hp.end() && !it->second.empty()) ? it->second : def;
  };
  model_name_ = get("model_name", hardware_info.name);
  base_link_name_ = get("base_link", "base_link");

  const sim::Entity model = ecm.EntityByComponents(
    sim::components::Name(model_name_), sim::components::Model());
  if (model == sim::kNullEntity) {
    RCLCPP_ERROR(logger(), "gz model '%s' not found in ECM", model_name_.c_str());
    return false;
  }
  base_entity_ = sim::Model(model).LinkByName(ecm, base_link_name_);
  if (base_entity_ == sim::kNullEntity) {
    RCLCPP_ERROR(logger(), "base link '%s' not found in model '%s'",
                 base_link_name_.c_str(), model_name_.c_str());
    return false;
  }
  // World velocity components are lazily populated; ask gz to keep them updated.
  sim::Link(base_entity_).EnableVelocityChecks(ecm, true);
  RCLCPP_INFO(logger(), "gz base-state bound: model '%s' link '%s'",
              model_name_.c_str(), base_link_name_.c_str());
  return true;
}

CallbackReturn GzBaseStateSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  std::size_t nc = 0;
  for (const auto & g : info_.gpios) {
    if (is_contact(g.name)) nc += g.state_interfaces.size();
  }
  contact_.assign(nc, 1.0);  // all-stance ground truth (standing)
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> GzBaseStateSystem::export_state_interfaces()
{
  // Bind each <gpio> block's state interfaces, in URDF order, to base_/contact_ —
  // matching FloatingContactSimSystem so BaseStateSensor/ContactSensor claim the
  // identical names.
  std::vector<StateInterface> ifaces;
  std::size_t ci = 0;
  for (const auto & gpio : info_.gpios) {
    const bool contact = is_contact(gpio.name);
    std::size_t bi = 0;
    for (const auto & si : gpio.state_interfaces) {
      if (contact) {
        if (ci < contact_.size()) ifaces.emplace_back(gpio.name, si.name, &contact_[ci++]);
      } else if (bi < base_.size()) {
        ifaces.emplace_back(gpio.name, si.name, &base_[bi++]);
      }
    }
  }
  return ifaces;
}

std::vector<CommandInterface> GzBaseStateSystem::export_command_interfaces()
{
  return {};  // read-only: the base is not actuated through this component
}

return_type GzBaseStateSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (ecm_ == nullptr || base_entity_ == sim::kNullEntity) {
    return return_type::OK;  // not bound yet -> hold identity
  }
  const sim::Link link(base_entity_);
  const auto pose = link.WorldPose(*ecm_);
  if (pose) {
    const auto & p = pose->Pos();
    const auto & q = pose->Rot();
    base_[0] = p.X(); base_[1] = p.Y(); base_[2] = p.Z();
    base_[3] = q.X(); base_[4] = q.Y(); base_[5] = q.Z(); base_[6] = q.W();

    // World-frame twist -> body frame (Pinocchio free-flyer convention, as verified
    // in M6.3): v_body = Rᵀ · v_world.
    const auto vw = link.WorldLinearVelocity(*ecm_);
    const auto ww = link.WorldAngularVelocity(*ecm_);
    const ignition::math::Vector3d vl = vw ? *vw : ignition::math::Vector3d::Zero;
    const ignition::math::Vector3d va = ww ? *ww : ignition::math::Vector3d::Zero;
    const ignition::math::Vector3d vb = q.RotateVectorReverse(vl);
    const ignition::math::Vector3d wb = q.RotateVectorReverse(va);
    base_[7] = vb.X(); base_[8] = vb.Y(); base_[9] = vb.Z();
    base_[10] = wb.X(); base_[11] = wb.Y(); base_[12] = wb.Z();
  }
  for (auto & c : contact_) c = 1.0;  // standing: all feet in stance
  return return_type::OK;
}

return_type GzBaseStateSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return return_type::OK;  // no commands
}

}  // namespace kontrolem_gz

PLUGINLIB_EXPORT_CLASS(
  kontrolem_gz::GzBaseStateSystem, gz_ros2_control::GazeboSimSystemInterface)
