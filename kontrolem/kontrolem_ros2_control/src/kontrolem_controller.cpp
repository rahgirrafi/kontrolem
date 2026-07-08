#include "kontrolem_ros2_control/kontrolem_controller.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>

#include "kontrolem_controllers/lqr_controller.hpp"
#include "kontrolem_controllers/lqg_controller.hpp"
#include "kontrolem_controllers/mpc_controller.hpp"
#include "kontrolem_controllers/qp_task_space_controller.hpp"
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
  throw std::runtime_error("unknown control_law '" + law + "' (expected 'lqr' or 'qp')");
}
}  // namespace

CallbackReturn KontrolemController::on_init()
{
  try {
    auto_declare<std::string>("control_law", "lqr");
    auto_declare<std::vector<std::string>>("actuated_joints", {});
    auto_declare<std::string>("robot_description", "");
    auto_declare<std::string>("command_interface", "effort");
    auto_declare<std::string>("state_position_interface", "position");
    auto_declare<std::string>("state_velocity_interface", "velocity");
    auto_declare<std::string>("safe_action", "zero");
    auto_declare<bool>("publish_diagnostics", false);
    auto_declare<std::vector<double>>("q_ref", {});
    auto_declare<std::vector<double>>("v_ref", {});
    // Reference: "setpoint" (Regulation) or "harmonic" (Tracking).
    auto_declare<std::string>("reference_type", "setpoint");
    auto_declare<std::vector<double>>("reference.center", {});
    auto_declare<std::vector<double>>("reference.amp", {});
    auto_declare<std::vector<double>>("reference.phase", {});
    auto_declare<double>("reference.omega", 0.5);
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

  try {
    model_ = kontrolem_model::RobotModel::from_urdf_string(urdf);
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

    const auto law = node.get_parameter("control_law").as_string();
    law_ = make_law(node, law, actuated_joints_, *model_);
    if (!kc::accepts(*law_, *problem_)) {
      RCLCPP_ERROR(node.get_logger(), "law '%s' does not accept the '%s' problem",
                   law.c_str(), ref_type.c_str());
      return CallbackReturn::ERROR;
    }
    auto synth = law_->synthesize(*model_, *problem_);
    law_->configure(*model_, *synth, *problem_);
    needs_velocity_ = law_->capabilities().needs_velocity_state;
    control_law_ = law;

    // Opt-in telemetry: an RT-safe publisher on ~/diagnostics.
    publish_diagnostics_ = node.get_parameter("publish_diagnostics").as_bool();
    if (publish_diagnostics_) {
      diag_pub_ = node.create_publisher<DiagMsg>("~/diagnostics", rclcpp::SystemDefaultsQoS());
      rt_diag_ = std::make_unique<realtime_tools::RealtimePublisher<DiagMsg>>(diag_pub_);
    }

    state_.q = Eigen::VectorXd::Zero(nq);
    state_.v = Eigen::VectorXd::Zero(nv);  // stays zero if the law is output-feedback

    RCLCPP_INFO(
      node.get_logger(),
      "configured control_law='%s' on %d-DoF model, %zu actuated (needs_velocity=%s)",
      law.c_str(), nv, actuated_joints_.size(), needs_velocity_ ? "true" : "false");
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
  // Assemble State from the claimed state interfaces (model joint order). Velocity
  // is read only if claimed; for an output-feedback law state_.v stays zero and
  // the law estimates it internally.
  for (std::size_t i = 0; i < pos_idx_.size(); ++i) {
    state_.q(static_cast<Eigen::Index>(i)) = state_interfaces_[pos_idx_[i]].get_value();
  }
  if (needs_velocity_) {
    for (std::size_t i = 0; i < vel_idx_.size(); ++i) {
      state_.v(static_cast<Eigen::Index>(i)) = state_interfaces_[vel_idx_[i]].get_value();
    }
  }
  state_.t = (time - start_time_).seconds();  // reference clock for Tracking

  const auto & u = law_->compute(state_, *problem_, period.seconds());
  const auto & st = law_->status();

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
    } else {  // Tracking: snapshot the reference at the current time (telemetry path)
      Eigen::VectorXd vr(model_->nv()), ar(model_->nv()), tr(model_->nv());
      qref_buf_.resize(model_->nq());
      static_cast<const kc::Tracking &>(*problem_).reference->sample(state_.t, qref_buf_, vr, ar, tr);
      m.q_ref.assign(qref_buf_.data(), qref_buf_.data() + qref_buf_.size());
    }
    m.ok = st.ok;
    m.margin = st.margin;
    m.safe_action = !st.ok;
    m.update_us = us;
    rt_diag_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

}  // namespace kontrolem_ros2_control

PLUGINLIB_EXPORT_CLASS(
  kontrolem_ros2_control::KontrolemController, controller_interface::ControllerInterface)
