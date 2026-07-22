#include "kontrolem_ros2_control/kontrolem_controller.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>

#include "kontrolem_controllers/lqr_controller.hpp"
#include "kontrolem_controllers/lqg_controller.hpp"
#include "kontrolem_controllers/mpc_controller.hpp"
#include "kontrolem_controllers/qp_task_space_controller.hpp"
#include "kontrolem_controllers/wbc_controller.hpp"
#include "kontrolem_controllers/kinematic_gait_controller.hpp"
#include "kontrolem_locomotion/crawl_gait.hpp"
#include "kontrolem_locomotion/trot_gait.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace kontrolem_ros2_control
{
namespace kc = kontrolem_control;
namespace kctl = kontrolem_controllers;
using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

namespace
{
Eigen::MatrixXd diag_or_identity(const std::vector<double> & d, int n)
{
  Eigen::MatrixXd M = Eigen::MatrixXd::Identity(n, n);
  if (static_cast<int>(d.size()) == n) {
    for (int i = 0; i < n; ++i) {
      M(i, i) = d[static_cast<std::size_t>(i)];
    }
  }
  return M;
}

// Build the concrete control law selected by parameter. This is the M0 factory;
// it will grow into a pluginlib-based registry as more laws land.
std::unique_ptr<kc::Controller> make_law(
  const rclcpp_lifecycle::LifecycleNode & node, const std::string & law,
  const std::vector<std::string> & actuated, const kontrolem_model::RobotModel & model)
{
  const int nv = model.nv();
  if (law == "lqr") {
    const auto q_diag = node.get_parameter("lqr.q_diag").as_double_array();
    const auto r_diag = node.get_parameter("lqr.r_diag").as_double_array();
    const double q_dev_max = node.get_parameter("lqr.q_dev_max").as_double();
    return std::make_unique<kctl::LqrController>(
      actuated, diag_or_identity(q_diag, 2 * nv),
      diag_or_identity(r_diag, static_cast<int>(actuated.size())), q_dev_max);
  }
  if (law == "lqg") {
    const int nq = model.nq();
    const auto q_diag = node.get_parameter("lqg.q_diag").as_double_array();
    const auto r_diag = node.get_parameter("lqg.r_diag").as_double_array();
    const auto w_diag = node.get_parameter("lqg.w_diag").as_double_array();
    const auto v_diag = node.get_parameter("lqg.v_diag").as_double_array();
    const double innov_max = node.get_parameter("lqg.innov_max").as_double();
    return std::make_unique<kctl::LqgController>(
      actuated, diag_or_identity(q_diag, 2 * nv),
      diag_or_identity(r_diag, static_cast<int>(actuated.size())),
      diag_or_identity(w_diag, 2 * nv), diag_or_identity(v_diag, nq), innov_max);
  }
  if (law == "mpc") {
    const auto q_diag = node.get_parameter("mpc.q_diag").as_double_array();
    const auto r_diag = node.get_parameter("mpc.r_diag").as_double_array();
    const int horizon = static_cast<int>(node.get_parameter("mpc.horizon").as_int());
    const double dt_mpc = node.get_parameter("mpc.dt_mpc").as_double();
    const double tau_max = node.get_parameter("mpc.tau_max").as_double();
    return std::make_unique<kctl::MpcController>(
      actuated, diag_or_identity(q_diag, 2 * nv),
      diag_or_identity(r_diag, static_cast<int>(actuated.size())), horizon, dt_mpc, tau_max);
  }
  if (law == "qp") {
    const auto w = node.get_parameter("qp.task_weight").as_double_array();
    Eigen::VectorXd W = Eigen::VectorXd::Ones(nv);
    if (static_cast<int>(w.size()) == nv) {
      for (int i = 0; i < nv; ++i) {
        W(i) = w[static_cast<std::size_t>(i)];
      }
    }
    return std::make_unique<kctl::QpTaskSpaceController>(
      actuated, W, node.get_parameter("qp.kp").as_double(),
      node.get_parameter("qp.kd").as_double(), node.get_parameter("qp.tau_max").as_double());
  }
  if (law == "wbc") {
    kctl::WbcController::Gains g;
    g.kp_base = node.get_parameter("wbc.kp_base").as_double();
    g.kd_base = node.get_parameter("wbc.kd_base").as_double();
    g.kp_post = node.get_parameter("wbc.kp_post").as_double();
    g.kd_post = node.get_parameter("wbc.kd_post").as_double();
    g.w_base = node.get_parameter("wbc.w_base").as_double();
    g.w_post = node.get_parameter("wbc.w_post").as_double();
    g.w_force = node.get_parameter("wbc.w_force").as_double();
    g.w_tau = node.get_parameter("wbc.w_tau").as_double();
    g.mu = node.get_parameter("wbc.mu").as_double();
    g.tau_max = node.get_parameter("wbc.tau_max").as_double();
    g.max_iter = static_cast<int>(node.get_parameter("wbc.max_iter").as_int());
    g.kp_swing = node.get_parameter("wbc.kp_swing").as_double();
    g.kd_swing = node.get_parameter("wbc.kd_swing").as_double();
    const auto feet = node.get_parameter("contact_frames").as_string_array();
    return std::make_unique<kctl::WbcController>(feet, actuated, g);
  }
  if (law == "kinematic_gait") {
    kctl::KinematicGaitController::Gains g;
    g.kp = node.get_parameter("kin.kp").as_double();
    g.kd = node.get_parameter("kin.kd").as_double();
    g.tau_max = node.get_parameter("kin.tau_max").as_double();
    g.ik_max_iter = static_cast<int>(node.get_parameter("kin.ik_max_iter").as_int());
    g.ik_tol = node.get_parameter("kin.ik_tol").as_double();
    g.ik_step_clamp = node.get_parameter("kin.ik_step_clamp").as_double();
    const auto feet = node.get_parameter("contact_frames").as_string_array();
    return std::make_unique<kctl::KinematicGaitController>(feet, actuated, g);
  }
  throw std::runtime_error(
    "unknown control_law '" + law + "' (expected lqr/lqg/mpc/qp/wbc/kinematic_gait)");
}
}  // namespace

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
    auto_declare<double>("kin.kp", 60.0);
    auto_declare<double>("kin.kd", 2.0);
    auto_declare<double>("kin.tau_max", 23.7);
    auto_declare<int>("kin.ik_max_iter", 20);
    auto_declare<double>("kin.ik_tol", 1.0e-6);
    auto_declare<double>("kin.ik_step_clamp", 0.5);
    auto_declare<std::vector<double>>("lqr.q_diag", {});
    auto_declare<std::vector<double>>("lqr.r_diag", {});
    auto_declare<double>("lqr.q_dev_max", 0.5);
    auto_declare<std::vector<double>>("lqg.q_diag", {});
    auto_declare<std::vector<double>>("lqg.r_diag", {});
    auto_declare<std::vector<double>>("lqg.w_diag", {});
    auto_declare<std::vector<double>>("lqg.v_diag", {});
    auto_declare<double>("lqg.innov_max", 0.5);
    auto_declare<std::vector<double>>("mpc.q_diag", {});
    auto_declare<std::vector<double>>("mpc.r_diag", {});
    auto_declare<int>("mpc.horizon", 30);
    auto_declare<double>("mpc.dt_mpc", 0.02);
    auto_declare<double>("mpc.tau_max", 100.0);
    auto_declare<std::vector<double>>("qp.task_weight", {});
    auto_declare<double>("qp.kp", 50.0);
    auto_declare<double>("qp.kd", 10.0);
    auto_declare<double>("qp.tau_max", 5.0);
    // Floating-base / WBC.
    auto_declare<std::string>("base_type", "fixed");     // "fixed" or "floating"
    auto_declare<std::string>("base_gpio", "floating_base");
    auto_declare<std::string>("contact_gpio", "contact");
    auto_declare<std::vector<std::string>>("contact_frames", {});  // e.g. [foot_FL, ...]
    auto_declare<double>("wbc.base_height", 0.0);        // nominal base z of the stance
    auto_declare<std::vector<double>>("wbc.nominal_posture", {});  // per actuated joint
    auto_declare<double>("wbc.kp_base", 100.0);
    auto_declare<double>("wbc.kd_base", 20.0);
    auto_declare<double>("wbc.kp_post", 25.0);
    auto_declare<double>("wbc.kd_post", 5.0);
    auto_declare<double>("wbc.w_base", 100.0);
    auto_declare<double>("wbc.w_post", 1.0);
    auto_declare<double>("wbc.w_force", 1e-4);
    auto_declare<double>("wbc.w_tau", 1e-4);
    auto_declare<double>("wbc.mu", 0.7);
    auto_declare<double>("wbc.tau_max", 40.0);
    auto_declare<int>("wbc.max_iter", 200);  // OSQP iteration cap (hard-RT bound)
    auto_declare<double>("wbc.kp_swing", 400.0);  // swing-foot tracking (Locomotion)
    auto_declare<double>("wbc.kd_swing", 40.0);
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
        auto l = make_law(node, lname, actuated_joints_, *model_);
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
      law_ = make_law(node, law, actuated_joints_, *model_);
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
