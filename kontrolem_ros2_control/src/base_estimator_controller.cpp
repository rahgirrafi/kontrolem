#include "kontrolem_ros2_control/base_estimator_controller.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace kontrolem_ros2_control
{
using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

CallbackReturn BaseEstimatorController::on_init()
{
  try {
    auto_declare<std::string>("robot_description", "");
    auto_declare<std::vector<std::string>>("actuated_joints", {});
    auto_declare<std::string>("contact_gpio", "contact");
    auto_declare<std::vector<std::string>>("contact_frames", {});
    auto_declare<std::string>("imu_topic", "/imu/data");
    auto_declare<std::string>("odom_topic", "/base_odom");
    // M12: use the gait's PLANNED contact schedule instead of the sensed (flickering) one.
    auto_declare<bool>("use_planned_contact", false);
    auto_declare<std::string>("planned_contact_topic", "/planned_contact");
    auto_declare<std::string>("odom_frame", "world");
    auto_declare<std::string>("base_frame", "base");
    auto_declare<double>("base_height", 0.0);
    // Which estimator: "complementary" (M8 contact-aided filter, default) or "inekf"
    // (M11 Right-Invariant EKF — covariance-weighted, robust to contact-sensing flicker).
    auto_declare<std::string>("estimator_type", "complementary");
    // Filter gains (see kontrolem_estimation::BaseEstimatorConfig).
    auto_declare<double>("gravity", 9.81);
    auto_declare<double>("k_grav", 0.02);
    auto_declare<double>("k_vel", 1.0);
    auto_declare<double>("k_pos", 0.05);
    auto_declare<double>("accel_gate", 0.5);
    auto_declare<double>("gyro_gate", 0.5);
    auto_declare<double>("innov_max", 0.15);
    auto_declare<bool>("flat_ground", false);
    // InEKF noise densities (see kontrolem_estimation::InvariantEstimatorConfig).
    auto_declare<double>("inekf.sigma_gyro", 0.01);
    auto_declare<double>("inekf.sigma_accel", 0.1);
    auto_declare<double>("inekf.sigma_contact", 1e-3);
    auto_declare<double>("inekf.sigma_contact_swing", 1e3);
    auto_declare<double>("inekf.sigma_fk", 0.02);
    auto_declare<double>("inekf.sigma_grav", 0.2);
    auto_declare<double>("inekf.fk_gate", 0.06);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn BaseEstimatorController::on_configure(const rclcpp_lifecycle::State &)
{
  auto & node = *get_node();
  const auto urdf = node.get_parameter("robot_description").as_string();
  if (urdf.empty()) {
    RCLCPP_ERROR(node.get_logger(), "parameter 'robot_description' is empty");
    return CallbackReturn::ERROR;
  }
  joint_names_ = node.get_parameter("actuated_joints").as_string_array();
  const auto feet = node.get_parameter("contact_frames").as_string_array();
  if (joint_names_.empty() || feet.empty()) {
    RCLCPP_ERROR(node.get_logger(), "actuated_joints and contact_frames must be non-empty");
    return CallbackReturn::ERROR;
  }
  base_height_ = node.get_parameter("base_height").as_double();
  odom_frame_ = node.get_parameter("odom_frame").as_string();
  base_frame_ = node.get_parameter("base_frame").as_string();
  contact_sensor_.emplace(node.get_parameter("contact_gpio").as_string(), feet);

  try {
    model_ = kontrolem_model::RobotModel::from_urdf_string(
      urdf, kontrolem_model::BaseType::kFloating);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node.get_logger(), "failed to build floating model: %s", e.what());
    return CallbackReturn::ERROR;
  }

  const auto est_type = node.get_parameter("estimator_type").as_string();
  const double gravity = node.get_parameter("gravity").as_double();
  const double accel_gate = node.get_parameter("accel_gate").as_double();
  const double gyro_gate = node.get_parameter("gyro_gate").as_double();
  const double innov_max = node.get_parameter("innov_max").as_double();
  try {
    if (est_type == "inekf") {
      kontrolem_estimation::InvariantEstimatorConfig cfg;
      cfg.contact_frames = feet;
      cfg.actuated_joints = joint_names_;
      cfg.gravity = gravity;
      cfg.accel_gate = accel_gate;
      cfg.gyro_gate = gyro_gate;
      cfg.innov_max = innov_max;
      cfg.sigma_gyro = node.get_parameter("inekf.sigma_gyro").as_double();
      cfg.sigma_accel = node.get_parameter("inekf.sigma_accel").as_double();
      cfg.sigma_contact = node.get_parameter("inekf.sigma_contact").as_double();
      cfg.sigma_contact_swing = node.get_parameter("inekf.sigma_contact_swing").as_double();
      cfg.sigma_fk = node.get_parameter("inekf.sigma_fk").as_double();
      cfg.sigma_grav = node.get_parameter("inekf.sigma_grav").as_double();
      cfg.fk_gate = node.get_parameter("inekf.fk_gate").as_double();
      estimator_ = std::make_unique<kontrolem_estimation::InvariantEstimator>(*model_, cfg);
    } else {
      kontrolem_estimation::BaseEstimatorConfig cfg;
      cfg.contact_frames = feet;
      cfg.actuated_joints = joint_names_;
      cfg.gravity = gravity;
      cfg.k_grav = node.get_parameter("k_grav").as_double();
      cfg.k_vel = node.get_parameter("k_vel").as_double();
      cfg.k_pos = node.get_parameter("k_pos").as_double();
      cfg.accel_gate = accel_gate;
      cfg.gyro_gate = gyro_gate;
      cfg.innov_max = innov_max;
      cfg.flat_ground = node.get_parameter("flat_ground").as_bool();
      estimator_ = std::make_unique<kontrolem_estimation::BaseEstimator>(*model_, cfg);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node.get_logger(), "failed to build estimator (check joint/foot names): %s",
                 e.what());
    return CallbackReturn::ERROR;
  }

  // IMU in (non-RT executor thread -> RT buffer), Odometry out (RT-safe publisher).
  imu_sub_ = node.create_subscription<sensor_msgs::msg::Imu>(
    node.get_parameter("imu_topic").as_string(), rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      imu_buffer_.writeFromNonRT(*msg);
      have_imu_ = true;
    });
  odom_pub_ = node.create_publisher<nav_msgs::msg::Odometry>(
    node.get_parameter("odom_topic").as_string(), rclcpp::SystemDefaultsQoS());
  rt_odom_ =
    std::make_unique<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom_pub_);

  // M12: optionally subscribe to the gait's planned contact schedule (per-foot 0/1, in
  // contact_frames order). When fresh, the estimator trusts it over the sensed contact.
  use_planned_contact_ = node.get_parameter("use_planned_contact").as_bool();
  if (use_planned_contact_) {
    const std::size_t nfeet = feet.size();
    planned_sub_ = node.create_subscription<std_msgs::msg::Float64MultiArray>(
      node.get_parameter("planned_contact_topic").as_string(), rclcpp::SystemDefaultsQoS(),
      [this, nfeet](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != nfeet) return;   // ignore mismatched-arity messages
        std::vector<uint8_t> s(nfeet);
        for (std::size_t i = 0; i < nfeet; ++i) s[i] = msg->data[i] > 0.5 ? 1 : 0;
        planned_buffer_.writeFromNonRT(s);
        planned_fresh_.store(50);   // ~0.1 s of freshness at 500 Hz before falling back
      });
  }

  const int nj = static_cast<int>(joint_names_.size());
  q_joints_.resize(nj);
  v_joints_.resize(nj);
  contact_raw_.assign(feet.size(), 0.0);
  stance_.assign(feet.size(), 0);
  RCLCPP_INFO(node.get_logger(),
              "base estimator configured: type=%s, %d joints, %zu feet, base_height=%.4f",
              est_type.c_str(), nj, feet.size(), base_height_);
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration BaseEstimatorController::command_interface_configuration() const
{
  return {interface_configuration_type::NONE, {}};  // read-only estimator
}

InterfaceConfiguration BaseEstimatorController::state_interface_configuration() const
{
  InterfaceConfiguration cfg;
  cfg.type = interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joint_names_) {
    cfg.names.push_back(j + "/position");
    cfg.names.push_back(j + "/velocity");
  }
  for (const auto & n : contact_sensor_->interface_names()) cfg.names.push_back(n);
  return cfg;
}

CallbackReturn BaseEstimatorController::on_activate(const rclcpp_lifecycle::State &)
{
  const auto find = [&](const std::string & name) -> std::size_t {
    for (std::size_t i = 0; i < state_interfaces_.size(); ++i) {
      if (state_interfaces_[i].get_name() == name) return i;
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
  if (!contact_sensor_->assign_loaned(state_interfaces_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "failed to bind contact <gpio> interfaces");
    return CallbackReturn::ERROR;
  }
  seeded_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn BaseEstimatorController::on_deactivate(const rclcpp_lifecycle::State &)
{
  contact_sensor_->release();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type BaseEstimatorController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Encoders + real per-foot contact from the claimed state interfaces.
  for (std::size_t j = 0; j < joint_names_.size(); ++j) {
    q_joints_[static_cast<Eigen::Index>(j)] = state_interfaces_[jpos_idx_[j]].get_value();
    v_joints_[static_cast<Eigen::Index>(j)] = state_interfaces_[jvel_idx_[j]].get_value();
  }
  contact_sensor_->read_into(contact_raw_);
  for (std::size_t i = 0; i < stance_.size(); ++i) stance_[i] = contact_raw_[i] > 0.5 ? 1 : 0;

  // Bumpless seed, identity orientation, base at the offline-proven standing height
  // (base_height already accounts for the foot collision geometry — a foot's contact
  // point sits one sphere-radius below its frame, which a naive "foot frame on the
  // ground" seed would miss). Wait for a clean state first: all feet planted AND the
  // encoders populated with a real (bent) standing posture, because a first tick can
  // arrive before the sim hardware has written the joint states (reads ~0), which would
  // set the leg-odometry anchors (set ONCE here) from a bogus straight-leg posture.
  if (!seeded_) {
    const bool planted = !stance_.empty() &&
      std::all_of(stance_.begin(), stance_.end(), [](uint8_t s) { return s != 0; });
    const bool joints_ready = q_joints_.size() > 0 && q_joints_.cwiseAbs().maxCoeff() > 0.1;
    if (!planted || !joints_ready) return controller_interface::return_type::OK;  // wait

    estimator_->seed(
      Eigen::Vector3d(0.0, 0.0, base_height_), Eigen::Matrix3d::Identity(),
      q_joints_, stance_);
    seeded_ = true;
  }

  // IMU dead-reckoning (once measurements flow), then the leg-odometry/gravity update.
  if (have_imu_) {
    const auto * imu = imu_buffer_.readFromRT();
    const Eigen::Vector3d gyro(
      imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
    const Eigen::Vector3d accel(
      imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
    estimator_->predict(gyro, accel, period.seconds());
  }

  // M12: while a fresh planned-contact schedule is arriving (the gait is walking), a foot
  // counts as stance only when the PLAN and the SENSOR AGREE it is down (logical AND). The
  // plan vetoes a mis-sensed swing foot (plan says swing → never used, the M10 poison); the
  // sensor vetoes a foot the plan expects down but that has not actually landed yet (touchdown
  // timing). Either source alone is worse: the plan blindly used a still-airborne foot (est
  // blew up), the sensor alone flickers. Watchdog decays to 0 → falls back to sensed contact
  // when the stream stops, so standing/postures are unaffected.
  int fresh = planned_fresh_.load();
  if (use_planned_contact_ && fresh > 0) {
    const auto * planned = planned_buffer_.readFromRT();
    if (planned && planned->size() == stance_.size()) {
      if (landed_.size() != stance_.size()) landed_.assign(stance_.size(), 0);
      for (std::size_t i = 0; i < stance_.size(); ++i) {
        if (!(*planned)[i]) {
          landed_[i] = 0;                 // plan says swing → reset; foot is off the ground
        } else if (stance_[i]) {
          landed_[i] = 1;                 // plan says stance AND sensor confirms → latched
        }
        // A foot is stance iff the plan wants it down AND the sensor has confirmed it landed
        // this phase (held through subsequent sensor flicker-drops until the plan lifts it).
        stance_[i] = ((*planned)[i] && landed_[i]) ? 1 : 0;
      }
    }
    planned_fresh_.store(fresh - 1);
  }
  estimator_->correct(q_joints_, v_joints_, stance_);

  // Publish the estimate as nav_msgs/Odometry: world-frame pose, body-frame twist —
  // the convention OdometryBaseBridge re-imports (Pinocchio free-flyer / REP-145).
  if (rt_odom_ && rt_odom_->trylock()) {
    auto & msg = rt_odom_->msg_;
    msg.header.stamp = time;
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id = base_frame_;
    const Eigen::Vector3d p = estimator_->position();
    const Eigen::Quaterniond q = estimator_->orientation();
    const Eigen::Vector3d vb = estimator_->velocity_body();
    const Eigen::Vector3d wb = estimator_->angular_body();
    msg.pose.pose.position.x = p.x();
    msg.pose.pose.position.y = p.y();
    msg.pose.pose.position.z = p.z();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();
    msg.twist.twist.linear.x = vb.x();
    msg.twist.twist.linear.y = vb.y();
    msg.twist.twist.linear.z = vb.z();
    msg.twist.twist.angular.x = wb.x();
    msg.twist.twist.angular.y = wb.y();
    msg.twist.twist.angular.z = wb.z();
    rt_odom_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

}  // namespace kontrolem_ros2_control

PLUGINLIB_EXPORT_CLASS(
  kontrolem_ros2_control::BaseEstimatorController, controller_interface::ControllerInterface)
