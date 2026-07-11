#include "kontrolem_gz/gz_base_state_system.hpp"

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include <cmath>
#include <unordered_map>

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
  rclcpp::Node::SharedPtr & model_nh,
  std::map<std::string, sim::Entity> & /*joints*/,
  const hardware_interface::HardwareInfo & hardware_info,
  sim::EntityComponentManager & ecm,
  int & /*update_rate*/)
{
  // gz_ros2_control calls initSim BEFORE on_init, so read our params from the
  // passed HardwareInfo here and resolve the base-link entity from the ECM.
  ecm_ = &ecm;
  node_ = model_nh;
  const auto & hp = hardware_info.hardware_parameters;
  auto get = [&](const std::string & k, const std::string & def) {
    auto it = hp.find(k);
    return (it != hp.end() && !it->second.empty()) ? it->second : def;
  };
  model_name_ = get("model_name", hardware_info.name);
  base_link_name_ = get("base_link", "base_link");

  // Base source: "estimate" subscribes to the estimator's /base_odom and exports THAT
  // as the base pose/twist (closed-loop sim-to-real); "ecm" (default) reads sim ground
  // truth. In estimate mode we still bootstrap from the ECM until the first /base_odom
  // arrives, so the WBC has a sane base during the welded startup gap.
  const std::string src = get("base_source", "ecm");
  const std::string odom_topic = get("odom_topic", "/base_odom");
  if (src == "estimate") {
    base_source_ = BaseSource::Estimate;
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_buffer_.writeFromNonRT(*msg);
        have_odom_.store(true);
      });
    RCLCPP_INFO(logger(), "gz base source = ESTIMATE (subscribing %s)", odom_topic.c_str());
  } else {
    RCLCPP_INFO(logger(), "gz base source = ECM (ground truth)");
  }

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
  // Collect the contact <gpio> state-interface names, in URDF order, and derive the
  // foot LINK each one names (the interface is "contact.<foot>"; strip that prefix).
  // This order matches export_state_interfaces() and contact_, so contact_[i]
  // corresponds to foot contact_frames_[i] -> resolve its sensor entity there.
  contact_frames_.clear();
  for (const auto & g : info_.gpios) {
    if (!is_contact(g.name)) continue;
    for (const auto & si : g.state_interfaces) {
      const auto dot = si.name.rfind('.');
      contact_frames_.push_back(dot == std::string::npos ? si.name : si.name.substr(dot + 1));
    }
  }
  contact_.assign(contact_frames_.size(), 1.0);  // default stance until sensors resolve
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

bool GzBaseStateSystem::resolve_contact_sensors()
{
  // The ignition contact-system stamps a ContactSensorData component onto the monitored
  // COLLISION entity (its Name is "<foot>_collision" — each foot, preserved as its own
  // link by gen_go2_gz.py, has exactly one collision). Index those by name, then match
  // each foot frame's collision. Done lazily (from read()) because the entities appear
  // only after the model + sensor systems spawn — after initSim/on_init.
  if (ecm_ == nullptr) return false;
  std::unordered_map<std::string, sim::Entity> data_by_collision;
  ecm_->Each<sim::components::ContactSensorData, sim::components::Name>(
    [&](const sim::Entity & e, const sim::components::ContactSensorData *,
        const sim::components::Name * name) {
      data_by_collision[name->Data()] = e;
      return true;
    });

  contact_sensors_.assign(contact_frames_.size(), sim::kNullEntity);
  bool all_bound = true;
  for (std::size_t i = 0; i < contact_frames_.size(); ++i) {
    auto it = data_by_collision.find(contact_frames_[i] + "_collision");
    if (it != data_by_collision.end()) {
      contact_sensors_[i] = it->second;
    } else {
      all_bound = false;  // fall back to assumed-stance for this foot until it appears
    }
  }
  if (all_bound && !contact_frames_.empty()) {
    RCLCPP_INFO(logger(), "gz contact sensors bound for %zu feet", contact_frames_.size());
  }
  return all_bound;
}

return_type GzBaseStateSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (ecm_ == nullptr || base_entity_ == sim::kNullEntity) {
    return return_type::OK;  // not bound yet -> hold identity
  }
  // Base pose/twist: the estimator's /base_odom (closed-loop) once it flows, else the
  // ECM ground truth (ecm mode, or bootstrap before the first estimate arrives).
  if (base_source_ == BaseSource::Estimate && have_odom_.load()) {
    const auto * odom = odom_buffer_.readFromRT();
    if (odom) {
      const auto & p = odom->pose.pose.position;
      const auto & q = odom->pose.pose.orientation;
      double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
      if (n < 1e-9) n = 1.0;
      base_[0] = p.x; base_[1] = p.y; base_[2] = p.z;
      base_[3] = q.x / n; base_[4] = q.y / n; base_[5] = q.z / n; base_[6] = q.w / n;
      // /base_odom twist is already body-frame (REP-145 / the estimator's contract) —
      // it maps straight onto Pinocchio's free-flyer v, no rotation (as odom_to_base).
      const auto & vl = odom->twist.twist.linear;
      const auto & va = odom->twist.twist.angular;
      base_[7] = vl.x; base_[8] = vl.y; base_[9] = vl.z;
      base_[10] = va.x; base_[11] = va.y; base_[12] = va.z;
    }
  } else {
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
  }
  // Real per-foot contact from the ECM (M8): a foot is in stance iff its contact
  // sensor reports >= 1 contact point this step. Feet whose sensor isn't resolved yet
  // (or ever) stay at the assumed-stance default (1.0), degrading to M7 behaviour.
  // The sensor entities appear a few ticks after spawn, so retry — but cap the retries
  // so a naming mismatch can't cost a per-tick ECM scan forever.
  constexpr int kMaxResolveAttempts = 500;  // ~1 s at 500 Hz
  if (!contacts_resolved_ && contact_resolve_attempts_ < kMaxResolveAttempts) {
    contacts_resolved_ = resolve_contact_sensors();
    if (!contacts_resolved_ && ++contact_resolve_attempts_ == kMaxResolveAttempts) {
      RCLCPP_WARN(logger(), "gz contact sensors not fully bound after %d attempts; "
                  "unresolved feet hold assumed-stance (1.0)", kMaxResolveAttempts);
    }
  }
  for (std::size_t i = 0; i < contact_sensors_.size(); ++i) {
    if (contact_sensors_[i] == sim::kNullEntity) continue;  // unresolved -> hold default
    const auto * data =
      ecm_->Component<sim::components::ContactSensorData>(contact_sensors_[i]);
    contact_[i] = (data && data->Data().contact_size() > 0) ? 1.0 : 0.0;
  }
  return return_type::OK;
}

return_type GzBaseStateSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return return_type::OK;  // no commands
}

}  // namespace kontrolem_gz

PLUGINLIB_EXPORT_CLASS(
  kontrolem_gz::GzBaseStateSystem, gz_ros2_control::GazeboSimSystemInterface)
