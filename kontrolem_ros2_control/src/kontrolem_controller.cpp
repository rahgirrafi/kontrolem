#include "kontrolem_ros2_control/kontrolem_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <set>
#include <stdexcept>

#include "kontrolem_locomotion/crawl_gait.hpp"
#include "kontrolem_locomotion/trot_gait.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace kontrolem_ros2_control
{
namespace kc = kontrolem_control;
using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

// M16 controller registry. build_law() replaces the old make_law() if/else: it
// discovers ControllerFactory plugins by name, declares the selected law's
// parameters generically from its ParameterSpec, validates them, and constructs
// the law. Adding a controller no longer edits this file — it ships its own
// ControllerFactory plugin and is selected by control_law: <its name>.

// Declare one parameter from its ParamDesc (dispatching on the variant type that
// its default pins) and return the read value.
kc::ParamValue KontrolemController::declare_law_param(const kc::ParamDesc & desc)
{
  const auto & def = desc.default_value;
  if (std::holds_alternative<double>(def)) {
    return kc::ParamValue{auto_declare<double>(desc.name, std::get<double>(def))};
  }
  if (std::holds_alternative<std::int64_t>(def)) {
    return kc::ParamValue{auto_declare<std::int64_t>(desc.name, std::get<std::int64_t>(def))};
  }
  if (std::holds_alternative<bool>(def)) {
    return kc::ParamValue{auto_declare<bool>(desc.name, std::get<bool>(def))};
  }
  if (std::holds_alternative<std::string>(def)) {
    return kc::ParamValue{auto_declare<std::string>(desc.name, std::get<std::string>(def))};
  }
  if (std::holds_alternative<std::vector<double>>(def)) {
    return kc::ParamValue{
      auto_declare<std::vector<double>>(desc.name, std::get<std::vector<double>>(def))};
  }
  if (std::holds_alternative<std::vector<std::int64_t>>(def)) {
    return kc::ParamValue{
      auto_declare<std::vector<std::int64_t>>(desc.name, std::get<std::vector<std::int64_t>>(def))};
  }
  return kc::ParamValue{
    auto_declare<std::vector<std::string>>(desc.name, std::get<std::vector<std::string>>(def))};
}

// Reject an override under a law's parameter prefix that the law does not
// declare (e.g. a typo'd wbc.frction) BEFORE the robot moves — the config-as-data
// payoff. We compare against the law's spec names directly (NOT has_parameter:
// the controller node auto-declares parameters from overrides, so a typo IS a
// declared parameter). A few runtime-owned params live under the wbc. prefix but
// build the WBC reference rather than the controller, so they are exempted.
void KontrolemController::validate_law_overrides(
  const std::string & law, const kc::ParameterSpec & spec)
{
  std::set<std::string> valid;     // names legitimately usable under a law prefix
  std::set<std::string> prefixes;  // the namespaces this law owns (e.g. {"wbc"})
  for (const auto & d : spec) {
    valid.insert(d.name);
    const auto dot = d.name.find('.');
    if (dot != std::string::npos) prefixes.insert(d.name.substr(0, dot));
  }
  // Runtime-owned params that share a law prefix (declared in on_init, consumed by
  // the runtime to build the WBC reference) — not controller parameters, not typos.
  valid.insert("wbc.base_height");
  valid.insert("wbc.nominal_posture");

  const auto & overrides = get_node()->get_node_parameters_interface()->get_parameter_overrides();
  for (const auto & kv : overrides) {
    const std::string & key = kv.first;
    const auto dot = key.find('.');
    if (dot == std::string::npos) continue;
    if (prefixes.count(key.substr(0, dot)) == 0) continue;  // not this law's namespace
    if (valid.count(key) != 0) continue;                    // a known parameter
    std::string names;
    for (const auto & d : spec) names += "\n    " + d.name;
    throw std::runtime_error(
      "control_law '" + law + "': unknown parameter '" + key +
      "'. Valid parameters for this law are:" + names);
  }
}

std::unique_ptr<kc::Controller> KontrolemController::build_law(const std::string & law)
{
  // Discover the factories once (cheap, stateless) and index them by name().
  if (!factory_loader_) {
    factory_loader_ = std::make_shared<pluginlib::ClassLoader<kc::ControllerFactory>>(
      "kontrolem_control", "kontrolem_control::ControllerFactory");
    for (const auto & cls : factory_loader_->getDeclaredClasses()) {
      auto inst = factory_loader_->createSharedInstance(cls);
      factories_[inst->name()] = inst;
    }
  }
  const auto it = factories_.find(law);
  if (it == factories_.end()) {
    std::string avail;
    for (const auto & kv : factories_) avail += " " + kv.first;
    throw std::runtime_error("unknown control_law '" + law + "' (available:" + avail + ")");
  }
  const auto & factory = it->second;
  const kc::ParameterSpec spec = factory->parameter_spec();

  // Declare + read this law's parameters generically from its schema, then reject
  // any typo'd override before constructing.
  kc::ParameterMap params(spec);
  for (const auto & d : spec) {
    params.set(d.name, declare_law_param(d));
  }
  validate_law_overrides(law, spec);

  kc::BuildContext ctx;
  ctx.actuated_joints = actuated_joints_;
  ctx.contact_frames = get_node()->get_parameter("contact_frames").as_string_array();
  return factory->create(params, *model_, ctx);
}

CallbackReturn KontrolemController::on_init()
{
  try {
    auto_declare<std::string>("control_law", "lqr");
    auto_declare<std::vector<std::string>>("control_laws", {});  // non-empty -> multi-controller
    auto_declare<int>("switch_blend_ticks", 20);                 // command blend length at a switch
    auto_declare<bool>("auto_fallback", false);                  // auto fail-forward on lost trust
    auto_declare<int>("fallback_dwell", 5);                      // not-ok ticks before failover
    auto_declare<bool>("auto_recover", false);                   // auto switch-back to the primary
    auto_declare<int>("recover_dwell", 50);                      // healthy-primary ticks before switch-back
    auto_declare<std::vector<std::string>>("actuated_joints", {});
    auto_declare<std::string>("robot_description", "");
    auto_declare<std::string>("command_interface", "effort");
    auto_declare<std::string>("state_position_interface", "position");
    auto_declare<std::string>("state_velocity_interface", "velocity");
    auto_declare<std::string>("safe_action", "zero");
    auto_declare<bool>("publish_diagnostics", false);
    // Publish the gait's planned per-foot contact schedule for the estimator (M12).
    auto_declare<bool>("publish_planned_contact", false);
    auto_declare<std::string>("planned_contact_topic", "/planned_contact");
    auto_declare<std::vector<double>>("q_ref", {});
    auto_declare<std::vector<double>>("v_ref", {});
    // Reference: "setpoint" (Regulation), "harmonic" (Tracking, fixed-base), or for the
    // floating WBC "base_pose" (canned base motion) / "live" (~/base_target topic).
    auto_declare<std::string>("reference_type", "setpoint");
    auto_declare<std::vector<double>>("reference.center", {});
    auto_declare<std::vector<double>>("reference.amp", {});
    auto_declare<std::vector<double>>("reference.phase", {});
    auto_declare<double>("reference.omega", 0.5);
    // M9 commanded base motion (6 axes: x,y,z, roll,pitch,yaw), for reference_type
    // base_pose. amp*cos(omega*t + phase) per axis. Live mode reads ~/base_target instead.
    auto_declare<std::vector<double>>("reference.base.amp", {});
    auto_declare<std::vector<double>>("reference.base.omega", {});
    auto_declare<std::vector<double>>("reference.base.phase", {});
    auto_declare<std::string>("base_target_topic", "~/base_target");
    // M10 static crawl walk (reference_type: gait). order = per-cycle swing sequence of the
    // contact_frames indices (default RL,FL,RR,FR); period/duty/step/base_gain shape it.
    auto_declare<std::vector<int64_t>>("gait.order", {2, 0, 3, 1});
    auto_declare<double>("gait.period", 12.0);
    auto_declare<double>("gait.duty", 0.5);
    auto_declare<double>("gait.step_len", 0.05);
    auto_declare<double>("gait.step_h", 0.04);
    auto_declare<double>("gait.base_gain", 1.0);
    auto_declare<double>("gait.start_delay", 1.5);
    auto_declare<std::vector<int64_t>>("gait.swing_pair", {0, 1, 1, 0});  // trot diagonal pairs
    // M15 startup settle-gate: hold the gait until the base settles after the weld-release,
    // instead of stepping on a fixed wall-clock (which races the release and tips the trot).
    auto_declare<bool>("gait.settle_gate", false);
    auto_declare<double>("gait.settle_release_speed", 0.08);  // m/s marking the release transient
    auto_declare<double>("gait.settle_speed", 0.04);          // m/s below which "settled"
    auto_declare<double>("gait.settle_tilt", 0.12);           // rad below which "settled"
    auto_declare<double>("gait.settle_hold", 0.3);            // s the settle must persist
    auto_declare<double>("gait.settle_timeout", 15.0);        // s hard fallback: step anyway
    // M16: each control law's OWN parameters (lqr.*, lqg.*, mpc.*, qp.*, wbc gains,
    // kin.*) are no longer declared here. They live in that law's ControllerFactory
    // parameter_spec() and are declared generically by build_law() in on_configure,
    // so the schema has a single source of truth and a new law adds nothing here.
    // Only the framework-owned, cross-law params below stay in the runtime.
    // Floating-base / WBC structural + reference params (consumed by the RUNTIME,
    // not by a controller constructor: base_height/nominal_posture build the WBC
    // reference; the rest select interfaces).
    auto_declare<std::string>("base_type", "fixed");     // "fixed" or "floating"
    auto_declare<std::string>("base_gpio", "floating_base");
    auto_declare<std::string>("contact_gpio", "contact");
    auto_declare<std::vector<std::string>>("contact_frames", {});  // e.g. [foot_FL, ...]
    auto_declare<double>("wbc.base_height", 0.0);        // nominal base z of the stance
    auto_declare<std::vector<double>>("wbc.nominal_posture", {});  // per actuated joint
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn KontrolemController::on_configure(const rclcpp_lifecycle::State &)
{
  auto & node = *get_node();

  const auto urdf = node.get_parameter("robot_description").as_string();
  if (urdf.empty()) {
    RCLCPP_ERROR(node.get_logger(), "parameter 'robot_description' (URDF XML) is empty");
    return CallbackReturn::ERROR;
  }
  actuated_joints_ = node.get_parameter("actuated_joints").as_string_array();
  if (actuated_joints_.empty()) {
    RCLCPP_ERROR(node.get_logger(), "parameter 'actuated_joints' must be set");
    return CallbackReturn::ERROR;
  }
  command_interface_ = node.get_parameter("command_interface").as_string();
  pos_interface_ = node.get_parameter("state_position_interface").as_string();
  vel_interface_ = node.get_parameter("state_velocity_interface").as_string();
  safe_action_ = node.get_parameter("safe_action").as_string();
  floating_ = (node.get_parameter("base_type").as_string() == "floating");

  try {
    model_ = kontrolem_model::RobotModel::from_urdf_string(
      urdf, floating_ ? kontrolem_model::BaseType::kFloating
                      : kontrolem_model::BaseType::kFixed);
    const int nq = model_->nq();
    const int nv = model_->nv();

    // Fill an n-vector from a double-array param (pad with zeros / truncate).
    const auto vec_param = [&](const std::string & name, int n) {
      const auto a = node.get_parameter(name).as_double_array();
      Eigen::VectorXd out = Eigen::VectorXd::Zero(n);
      for (int i = 0; i < n && i < static_cast<int>(a.size()); ++i) {
        out(i) = a[static_cast<std::size_t>(i)];
      }
      return out;
    };

    // Floating-base (WBC) setup: semantic components + a standing Regulation
    // built by generalized-coordinate index (robust to joint ordering).
    if (floating_) {
      feet_ = node.get_parameter("contact_frames").as_string_array();
      base_gpio_ = node.get_parameter("base_gpio").as_string();
      contact_gpio_ = node.get_parameter("contact_gpio").as_string();
      base_sensor_.emplace(base_gpio_);
      contact_sensor_.emplace(contact_gpio_, feet_);

      // Nominal standing configuration: base at base_height + the nominal joint posture.
      Eigen::VectorXd q_nominal = model_->neutral();
      q_nominal(2) = node.get_parameter("wbc.base_height").as_double();
      const auto posture = node.get_parameter("wbc.nominal_posture").as_double_array();
      for (std::size_t k = 0; k < actuated_joints_.size() && k < posture.size(); ++k) {
        q_nominal(model_->joint_q_index(actuated_joints_[k])) = posture[k];
      }

      // reference_type selects: setpoint (Regulation = M4-M8 standing), base_pose (a
      // canned base trajectory), or live (~/base_target topic). base_pose/live are the
      // M9 commanded-posture modes and need the WBC's Tracking capability.
      const auto ref_type = node.get_parameter("reference_type").as_string();
      auto axes6 = [&](const std::string & p) {
        const auto v = node.get_parameter(p).as_double_array();
        std::array<double, 6> a{{0, 0, 0, 0, 0, 0}};
        for (std::size_t i = 0; i < 6 && i < v.size(); ++i) a[i] = v[i];
        return a;
      };
      if (ref_type == "base_pose") {
        auto ref = std::make_unique<kc::BasePoseReference>();
        ref->q_nominal = q_nominal;
        ref->amp = axes6("reference.base.amp");
        ref->omega = axes6("reference.base.omega");
        ref->phase = axes6("reference.base.phase");
        auto trk = std::make_unique<kc::Tracking>();
        trk->reference = ref.get();
        reference_ = std::move(ref);
        problem_ = std::move(trk);
        RCLCPP_INFO(node.get_logger(), "reference: base_pose (canned base trajectory)");
      } else if (ref_type == "live") {
        auto ref = std::make_unique<kc::LiveBaseTarget>();
        ref->q_nominal = q_nominal;
        live_target_ = ref.get();
        auto trk = std::make_unique<kc::Tracking>();
        trk->reference = ref.get();
        reference_ = std::move(ref);
        problem_ = std::move(trk);
        base_target_buffer_.writeFromNonRT(std::array<double, 6>{{0, 0, 0, 0, 0, 0}});
        base_target_sub_ = node.create_subscription<geometry_msgs::msg::Twist>(
          node.get_parameter("base_target_topic").as_string(), rclcpp::SystemDefaultsQoS(),
          [this](const geometry_msgs::msg::Twist::SharedPtr m) {
            base_target_buffer_.writeFromNonRT(std::array<double, 6>{
              {m->linear.x, m->linear.y, m->linear.z, m->angular.x, m->angular.y, m->angular.z}});
          });
        RCLCPP_INFO(node.get_logger(), "reference: live (~/base_target base offset)");
      } else if (ref_type == "gait") {
        // Static crawl walk. Nominal foot positions from FK on q_nominal; the CoM bias
        // leads the base target forward so the CoM (not the base) sits over the support.
        auto g = std::make_unique<kontrolem_locomotion::CrawlGait>();
        g->q_nominal = q_nominal;
        for (const auto & f : feet_) g->foot_nominal.push_back(model_->frame_position(q_nominal, f));
        const auto ord = node.get_parameter("gait.order").as_integer_array();
        for (std::size_t i = 0; i < 4 && i < ord.size(); ++i) g->order[i] = static_cast<int>(ord[i]);
        g->period = node.get_parameter("gait.period").as_double();
        g->duty = node.get_parameter("gait.duty").as_double();
        g->step_len = node.get_parameter("gait.step_len").as_double();
        g->step_h = node.get_parameter("gait.step_h").as_double();
        g->base_gain = node.get_parameter("gait.base_gain").as_double();
        g->start_delay = node.get_parameter("gait.start_delay").as_double();
        const Eigen::Vector3d com0 = model_->center_of_mass(q_nominal);
        g->com_bias_x = -com0.x();
        g->com_bias_y = -com0.y();
        auto loco = std::make_unique<kc::Locomotion>();
        loco->gait = g.get();
        gait_ = std::move(g);
        problem_ = std::move(loco);
        RCLCPP_INFO(node.get_logger(), "reference: gait (static crawl walk, period=%.1f)",
                    node.get_parameter("gait.period").as_double());
      } else if (ref_type == "trot") {
        // Dynamic diagonal trot (M13/M14). Nominal feet from FK; base held at nominal (no
        // support-centroid shift) and advanced forward — see kontrolem_locomotion::TrotGait.
        auto g = std::make_unique<kontrolem_locomotion::TrotGait>();
        g->q_nominal = q_nominal;
        for (const auto & f : feet_) g->foot_nominal.push_back(model_->frame_position(q_nominal, f));
        const auto sp = node.get_parameter("gait.swing_pair").as_integer_array();
        for (std::size_t i = 0; i < 4 && i < sp.size(); ++i) g->swing_pair[i] = static_cast<int>(sp[i]);
        g->period = node.get_parameter("gait.period").as_double();
        g->duty = node.get_parameter("gait.duty").as_double();
        g->step_len = node.get_parameter("gait.step_len").as_double();
        g->step_h = node.get_parameter("gait.step_h").as_double();
        g->start_delay = node.get_parameter("gait.start_delay").as_double();
        auto loco = std::make_unique<kc::Locomotion>();
        loco->gait = g.get();
        gait_ = std::move(g);
        problem_ = std::move(loco);
        RCLCPP_INFO(node.get_logger(), "reference: trot (dynamic diagonal trot, period=%.2f)",
                    node.get_parameter("gait.period").as_double());
      } else {
        auto reg = std::make_unique<kc::Regulation>();
        reg->q_ref = q_nominal;
        reg->v_ref = Eigen::VectorXd::Zero(nv);
        problem_ = std::move(reg);
      }
    } else {

    // Build the problem: a fixed setpoint (Regulation) or a moving reference
    // (Tracking) selected by the reference_type param.
    const auto ref_type = node.get_parameter("reference_type").as_string();
    if (ref_type == "harmonic") {
      auto ref = std::make_unique<kc::HarmonicReference>();
      ref->center = vec_param("reference.center", nq);
      ref->amp = vec_param("reference.amp", nq);
      ref->phase = vec_param("reference.phase", nq);
      ref->omega = node.get_parameter("reference.omega").as_double();
      auto trk = std::make_unique<kc::Tracking>();
      trk->reference = ref.get();
      reference_ = std::move(ref);
      problem_ = std::move(trk);
      RCLCPP_INFO(node.get_logger(), "reference: harmonic (omega=%.3f)",
                  node.get_parameter("reference.omega").as_double());
    } else {
      auto reg = std::make_unique<kc::Regulation>();
      reg->q_ref = vec_param("q_ref", nq);
      reg->v_ref = vec_param("v_ref", nv);
      problem_ = std::move(reg);
    }
    }  // end fixed-base problem build

    const auto laws_param = node.get_parameter("control_laws").as_string_array();
    multi_ = !laws_param.empty();
    if (multi_) {
      // Host every named law behind the Supervisor; the first is initially active.
      const int blend = static_cast<int>(node.get_parameter("switch_blend_ticks").as_int());
      supervisor_ = std::make_unique<kc::Supervisor>(blend);
      needs_velocity_ = false;  // claim the UNION of the hosted laws' needs (can't renegotiate)
      for (const auto & lname : laws_param) {
        auto l = build_law(lname);
        if (!kc::accepts(*l, *problem_)) {
          RCLCPP_ERROR(node.get_logger(), "law '%s' does not accept problem dialect %d",
                       lname.c_str(), static_cast<int>(problem_->kind()));
          return CallbackReturn::ERROR;
        }
        auto synth = l->synthesize(*model_, *problem_);
        l->configure(*model_, *synth, *problem_);
        needs_velocity_ = needs_velocity_ || l->capabilities().needs_velocity_state;
        supervisor_->add(lname, l.get());
        laws_.push_back(std::move(l));
      }
      supervisor_->set_active(laws_param.front());
      supervisor_->set_auto_fallback(
        node.get_parameter("auto_fallback").as_bool(),
        static_cast<int>(node.get_parameter("fallback_dwell").as_int()));
      supervisor_->set_auto_recover(
        node.get_parameter("auto_recover").as_bool(),
        static_cast<int>(node.get_parameter("recover_dwell").as_int()));
      control_law_ = laws_param.front();

      // Manual switch command: publish the target law name to ~/switch_controller.
      switch_sub_ = node.create_subscription<std_msgs::msg::String>(
        "~/switch_controller", rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(switch_mtx_);
          switch_request_ = msg->data;
        });
      RCLCPP_INFO(node.get_logger(), "multi-controller: hosting %zu laws, active '%s' (blend=%d)",
                  laws_param.size(), control_law_.c_str(), blend);
    } else {
      const auto law = node.get_parameter("control_law").as_string();
      law_ = build_law(law);
      if (!kc::accepts(*law_, *problem_)) {
        RCLCPP_ERROR(node.get_logger(), "law '%s' does not accept problem dialect %d",
                     law.c_str(), static_cast<int>(problem_->kind()));
        return CallbackReturn::ERROR;
      }
      auto synth = law_->synthesize(*model_, *problem_);
      law_->configure(*model_, *synth, *problem_);
      needs_velocity_ = law_->capabilities().needs_velocity_state;
      control_law_ = law;
    }

    // Opt-in telemetry: an RT-safe publisher on ~/diagnostics.
    publish_diagnostics_ = node.get_parameter("publish_diagnostics").as_bool();
    if (publish_diagnostics_) {
      diag_pub_ = node.create_publisher<DiagMsg>("~/diagnostics", rclcpp::SystemDefaultsQoS());
      rt_diag_ = std::make_unique<realtime_tools::RealtimePublisher<DiagMsg>>(diag_pub_);
    }

    // Opt-in planned-contact publisher (M12): the gait's per-foot stance schedule, so the
    // estimator can trust the PLAN instead of the flickering sensed contact while walking.
    publish_planned_contact_ = node.get_parameter("publish_planned_contact").as_bool();
    if (publish_planned_contact_) {
      const auto topic = node.get_parameter("planned_contact_topic").as_string();
      pc_pub_ = node.create_publisher<std_msgs::msg::Float64MultiArray>(
        topic, rclcpp::SystemDefaultsQoS());
      rt_pc_ =
        std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
          pc_pub_);
    }

    // M15 startup settle-gate config (effective only for a floating-base Locomotion problem).
    settle_gate_ = node.get_parameter("gait.settle_gate").as_bool();
    settle_release_speed_ = node.get_parameter("gait.settle_release_speed").as_double();
    settle_speed_ = node.get_parameter("gait.settle_speed").as_double();
    settle_tilt_ = node.get_parameter("gait.settle_tilt").as_double();
    settle_hold_ = node.get_parameter("gait.settle_hold").as_double();
    settle_timeout_ = node.get_parameter("gait.settle_timeout").as_double();

    state_.q = Eigen::VectorXd::Zero(nq);
    state_.v = Eigen::VectorXd::Zero(nv);  // stays zero if the law is output-feedback

    RCLCPP_INFO(
      node.get_logger(),
      "configured control_law='%s' on %d-DoF model, %zu actuated (needs_velocity=%s)",
      control_law_.c_str(), nv, actuated_joints_.size(), needs_velocity_ ? "true" : "false");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node.get_logger(), "on_configure failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration KontrolemController::command_interface_configuration() const
{
  InterfaceConfiguration cfg;
  cfg.type = interface_configuration_type::INDIVIDUAL;
  for (const auto & j : actuated_joints_) {
    cfg.names.push_back(j + "/" + command_interface_);
  }
  return cfg;
}

InterfaceConfiguration KontrolemController::state_interface_configuration() const
{
  InterfaceConfiguration cfg;
  cfg.type = interface_configuration_type::INDIVIDUAL;

  if (floating_) {
    // Floating base: the ACTUATED joints have encoders; the SE(3) base and the
    // contacts arrive as <gpio> scalars claimed via the semantic components.
    for (const auto & j : actuated_joints_) {
      cfg.names.push_back(j + "/" + pos_interface_);
      cfg.names.push_back(j + "/" + vel_interface_);
    }
    for (const auto & n : base_sensor_->interface_names()) cfg.names.push_back(n);
    for (const auto & n : contact_sensor_->interface_names()) cfg.names.push_back(n);
    return cfg;
  }

  for (const auto & j : model_->joint_names()) {
    cfg.names.push_back(j + "/" + pos_interface_);
  }
  // Claim velocity only when the law needs it — an output-feedback law (LQG)
  // estimates velocity internally and takes positions only.
  if (needs_velocity_) {
    for (const auto & j : model_->joint_names()) {
      cfg.names.push_back(j + "/" + vel_interface_);
    }
  }
  return cfg;
}

CallbackReturn KontrolemController::on_activate(const rclcpp_lifecycle::State &)
{
  start_time_ = get_node()->now();  // t=0 for the Tracking reference clock
  gait_released_ = false; release_seen_ = false; t_release_ = 0.0; settle_accum_ = 0.0;  // reset gate

  const auto find = [](const auto & ifaces, const std::string & name) -> std::size_t {
    for (std::size_t i = 0; i < ifaces.size(); ++i) {
      if (ifaces[i].get_name() == name) {
        return i;
      }
    }
    return ifaces.size();
  };

  if (floating_) {
    act_q_idx_.clear();
    act_v_idx_.clear();
    jpos_idx_.assign(actuated_joints_.size(), 0);
    jvel_idx_.assign(actuated_joints_.size(), 0);
    for (std::size_t i = 0; i < actuated_joints_.size(); ++i) {
      act_q_idx_.push_back(model_->joint_q_index(actuated_joints_[i]));
      act_v_idx_.push_back(model_->joint_v_index(actuated_joints_[i]));
      jpos_idx_[i] = find(state_interfaces_, actuated_joints_[i] + "/" + pos_interface_);
      jvel_idx_[i] = find(state_interfaces_, actuated_joints_[i] + "/" + vel_interface_);
      if (jpos_idx_[i] == state_interfaces_.size() || jvel_idx_[i] == state_interfaces_.size()) {
        RCLCPP_ERROR(get_node()->get_logger(), "missing pos/vel interface for joint '%s'",
                     actuated_joints_[i].c_str());
        return CallbackReturn::ERROR;
      }
    }
    if (!base_sensor_->assign_loaned(state_interfaces_) ||
        !contact_sensor_->assign_loaned(state_interfaces_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "failed to bind base/contact <gpio> interfaces");
      return CallbackReturn::ERROR;
    }
    contact_buf_.assign(feet_.size(), 0.0);
  } else {
  const auto & joints = model_->joint_names();
  pos_idx_.assign(joints.size(), 0);
  vel_idx_.assign(needs_velocity_ ? joints.size() : 0, 0);
  for (std::size_t i = 0; i < joints.size(); ++i) {
    pos_idx_[i] = find(state_interfaces_, joints[i] + "/" + pos_interface_);
    if (pos_idx_[i] == state_interfaces_.size()) {
      RCLCPP_ERROR(get_node()->get_logger(), "missing position interface for joint '%s'",
                   joints[i].c_str());
      return CallbackReturn::ERROR;
    }
    if (needs_velocity_) {
      vel_idx_[i] = find(state_interfaces_, joints[i] + "/" + vel_interface_);
      if (vel_idx_[i] == state_interfaces_.size()) {
        RCLCPP_ERROR(get_node()->get_logger(), "missing velocity interface for joint '%s'",
                     joints[i].c_str());
        return CallbackReturn::ERROR;
      }
    }
  }
  }  // end fixed-base index resolution

  cmd_idx_.assign(actuated_joints_.size(), 0);
  for (std::size_t i = 0; i < actuated_joints_.size(); ++i) {
    cmd_idx_[i] = find(command_interfaces_, actuated_joints_[i] + "/" + command_interface_);
    if (cmd_idx_[i] == command_interfaces_.size()) {
      RCLCPP_ERROR(get_node()->get_logger(), "missing command interface for '%s'",
                   actuated_joints_[i].c_str());
      return CallbackReturn::ERROR;
    }
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn KontrolemController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & ci : command_interfaces_) {
    ci.set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type KontrolemController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const auto t_start = std::chrono::steady_clock::now();
  // Assemble State from the claimed state interfaces. For a floating base the SE(3)
  // root (q[0..6]/v[0..5], quaternion normalized) comes from the semantic base
  // sensor and each actuated joint fills its generalized-coordinate slot; for a
  // fixed base the joint interfaces map straight into q/v (model joint order).
  if (floating_) {
    base_sensor_->read_into(state_.q, state_.v);
    for (std::size_t i = 0; i < act_q_idx_.size(); ++i) {
      state_.q(act_q_idx_[i]) = state_interfaces_[jpos_idx_[i]].get_value();
      state_.v(act_v_idx_[i]) = state_interfaces_[jvel_idx_[i]].get_value();
    }
    contact_sensor_->read_into(contact_buf_);  // ground-truth stance (WBC schedules all-stance)
  } else {
    for (std::size_t i = 0; i < pos_idx_.size(); ++i) {
      state_.q(static_cast<Eigen::Index>(i)) = state_interfaces_[pos_idx_[i]].get_value();
    }
    if (needs_velocity_) {
      for (std::size_t i = 0; i < vel_idx_.size(); ++i) {
        state_.v(static_cast<Eigen::Index>(i)) = state_interfaces_[vel_idx_[i]].get_value();
      }
    }
  }
  state_.t = (time - start_time_).seconds();  // reference clock for Tracking

  // M15 startup settle-gate: hold a floating-base gait at pre-start (nominal all-stance — the
  // WBC just stabilizes the stance, which settles the robot) until the base has SETTLED after
  // the weld-release, then run the gait on a clock zeroed at that moment. This removes the
  // startup-timing race that tips the trot ~2/3 of runs even on ground truth (M12). Both gait
  // sample sites read state_.t, so gating it here is sufficient. Locomotion + floating only.
  if (settle_gate_ && floating_ && problem_ && problem_->kind() == kc::Dialect::kLocomotion) {
    if (!gait_released_) {
      const double speed = state_.v.head<3>().norm();  // base linear speed (frame-invariant norm)
      const double r22 = 1.0 - 2.0 * (state_.q[3] * state_.q[3] + state_.q[4] * state_.q[4]);
      const double tilt = std::acos(std::max(-1.0, std::min(1.0, r22)));  // body-up vs world-up
      if (speed > settle_release_speed_ || tilt > settle_tilt_) release_seen_ = true;
      const bool settled = speed < settle_speed_ && tilt < settle_tilt_;
      settle_accum_ = (release_seen_ && settled) ? settle_accum_ + period.seconds() : 0.0;
      const bool timed_out = state_.t > settle_timeout_;
      if ((release_seen_ && settle_accum_ >= settle_hold_) || timed_out) {
        gait_released_ = true;
        t_release_ = state_.t;
        RCLCPP_INFO(
          get_node()->get_logger(), "gait settle-gate: released at t=%.2fs (%s)", state_.t,
          timed_out ? "timeout fallback" : "base settled after release");
      }
    }
    state_.t = gait_released_ ? (state_.t - t_release_) : 0.0;  // pre-start hold while gating
  }

  // Live posture command: copy the latest ~/base_target offset into the reference before
  // compute (single-threaded here; the topic thread only writes the RealtimeBuffer).
  if (live_target_ != nullptr) {
    live_target_->offset = *base_target_buffer_.readFromRT();
  }

  const kc::Command * u_ptr;
  const kc::Status * st_ptr;
  if (multi_) {
    // Drain a pending switch command (best-effort trylock; the RT loop never
    // blocks on the topic thread). request_switch() only sets a flag the
    // Supervisor acts on inside compute().
    {
      std::unique_lock<std::mutex> lk(switch_mtx_, std::try_to_lock);
      if (lk.owns_lock() && !switch_request_.empty()) {
        supervisor_->request_switch(switch_request_);
        switch_request_.clear();
      }
    }
    u_ptr = &supervisor_->compute(state_, *problem_, period.seconds());
    st_ptr = &supervisor_->status();
    control_law_ = supervisor_->active_name();  // short name -> SSO, no heap in the loop
  } else {
    u_ptr = &law_->compute(state_, *problem_, period.seconds());
    st_ptr = &law_->status();
  }
  const auto & u = *u_ptr;
  const auto & st = *st_ptr;

  // Minimal Supervisor: trust status(); on a violation apply the safe action.
  if (st.ok) {
    for (std::size_t i = 0; i < cmd_idx_.size(); ++i) {
      command_interfaces_[cmd_idx_[i]].set_value(u.tau(static_cast<Eigen::Index>(i)));
    }
  } else {
    const double safe = 0.0;  // "zero" — the only M0 safe action
    for (std::size_t i = 0; i < cmd_idx_.size(); ++i) {
      command_interfaces_[cmd_idx_[i]].set_value(safe);
    }
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "status not ok (margin=%.3f) — applying safe action '%s'", st.margin, safe_action_.c_str());
  }

  // Sample the gait once if a Locomotion consumer (telemetry or planned-contact) needs it.
  const bool is_loco = problem_->kind() == kc::Dialect::kLocomotion;
  if (is_loco && (publish_diagnostics_ || publish_planned_contact_)) {
    gait_plan_buf_.resize(feet_.size(), model_->nq(), model_->nv());
    static_cast<const kc::Locomotion &>(*problem_).gait->sample(state_.t, gait_plan_buf_);
  }

  // Planned contact (M12): publish the gait's per-foot stance (1 = planted, 0 = swing) so the
  // estimator can trust the PLAN instead of the flickering sensed contact while walking.
  if (publish_planned_contact_ && is_loco && rt_pc_ && rt_pc_->trylock()) {
    auto & msg = rt_pc_->msg_;
    msg.data.resize(feet_.size());
    for (std::size_t i = 0; i < feet_.size(); ++i) {
      msg.data[i] = gait_plan_buf_.stance[i] ? 1.0 : 0.0;
    }
    rt_pc_->unlockAndPublish();
  }

  // Opt-in telemetry (RT-safe: best-effort trylock; skipped if the reader holds).
  if (publish_diagnostics_ && rt_diag_ && rt_diag_->trylock()) {
    const double us =
      std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t_start).count();
    auto & m = rt_diag_->msg_;
    m.header.stamp = time;
    m.control_law = control_law_;
    m.q.assign(state_.q.data(), state_.q.data() + state_.q.size());
    m.v.assign(state_.v.data(), state_.v.data() + state_.v.size());
    m.tau.assign(u.tau.data(), u.tau.data() + u.tau.size());  // the law's command
    if (problem_->kind() == kc::Dialect::kRegulation) {
      const auto & reg = static_cast<const kc::Regulation &>(*problem_);
      m.q_ref.assign(reg.q_ref.data(), reg.q_ref.data() + reg.q_ref.size());
    } else if (problem_->kind() == kc::Dialect::kTracking) {
      // Snapshot the reference at the current time (telemetry path).
      Eigen::VectorXd vr(model_->nv()), ar(model_->nv()), tr(model_->nv());
      qref_buf_.resize(model_->nq());
      static_cast<const kc::Tracking &>(*problem_).reference->sample(state_.t, qref_buf_, vr, ar, tr);
      m.q_ref.assign(qref_buf_.data(), qref_buf_.data() + qref_buf_.size());
    } else if (is_loco) {
      // gait_plan_buf_ was already sampled above (a Locomotion carries a GaitSource, NOT a
      // TrajectorySource — casting it to Tracking here was the M10 first-tick crash).
      m.q_ref.assign(gait_plan_buf_.q_ref.data(),
                     gait_plan_buf_.q_ref.data() + gait_plan_buf_.q_ref.size());
    }
    m.ok = st.ok;
    m.margin = st.margin;
    m.safe_action = !st.ok;
    m.update_us = us;
    m.solver_iters = st.iters;
    rt_diag_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

}  // namespace kontrolem_ros2_control

PLUGINLIB_EXPORT_CLASS(
  kontrolem_ros2_control::KontrolemController, controller_interface::ControllerInterface)
