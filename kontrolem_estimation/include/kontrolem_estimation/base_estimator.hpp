// BaseEstimator — a ROS-free floating-base state estimator (contact-aided,
// InEKF-ready). A real robot has no ground-truth oracle for its floating-base
// pose/twist (unlike the simulator's ECM): it must ESTIMATE them from the sensors it
// actually has — an IMU, joint encoders, and foot-contact detection. This is that
// estimator, sized right for standing and structured so a full InEKF drops in later.
//
// State is an SE_2(3) element (R, v, p): world<-base orientation, world-frame linear
// velocity, world-frame position — the exact shape a right-invariant EKF carries, so
// the upgrade is behind this same state + predict()/correct() API (agreed M8 design).
//
// Method (contact-aided complementary / leg odometry):
//   predict(gyro, accel, dt): IMU dead-reckoning. Gyro integrates orientation; the
//     accelerometer's specific force (rotated to world, gravity added) integrates
//     velocity then position. On its own this DRIFTS — hence correct().
//   correct(q_joints, v_joints, stance): the measurement update that bounds the drift.
//     Leg odometry — a stance foot does not move in the world (v_foot_world = 0) — turns
//     the joint encoders + contact Jacobian into a base-velocity measurement, and a
//     per-foot world "anchor" into a base-position measurement. The accelerometer's
//     gravity direction corrects roll/pitch (a complementary attitude aid).
//
// Eigen + kontrolem_model (FK/Jacobian) only. Preallocated / allocation-free after
// construction, mirroring LqgController's predict/correct realization.
#ifndef KONTROLEM_ESTIMATION__BASE_ESTIMATOR_HPP_
#define KONTROLEM_ESTIMATION__BASE_ESTIMATOR_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "kontrolem_control/types.hpp"
#include "kontrolem_model/robot_model.hpp"

namespace kontrolem_estimation
{

using kontrolem_control::Status;
using kontrolem_model::RobotModel;

/// InEKF-ready floating-base state, an SE_2(3) element. R: world<-base orientation.
/// v: world-frame base linear velocity. p: world-frame base position. A right-invariant
/// EKF upgrades in place behind this same shape (that is why v/p are world-frame).
struct BaseState
{
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
};

/// Filter configuration. Gains are complementary blend factors in [0, 1] (per correct)
/// except where noted; the defaults are sized for quasi-static standing and are the
/// knobs a future gait/InEKF upgrade retunes.
struct BaseEstimatorConfig
{
  std::vector<std::string> contact_frames;   ///< foot frames; defines the stance-mask order
  std::vector<std::string> actuated_joints;  ///< encoder order for q_joints / v_joints
  double gravity = 9.81;                      ///< |g| (m/s^2); g_world = [0, 0, -gravity]
  double k_grav = 0.02;    ///< accel gravity attitude-aid gain (roll/pitch), per correct()
  double k_vel = 1.0;      ///< leg-odometry base-velocity trust in [0, 1]
  double k_pos = 0.05;     ///< leg-odometry position-anchor correction gain
  double accel_gate = 0.5; ///< accept gravity aiding only if | |accel| - g | < this (m/s^2)
  double gyro_gate = 0.5;  ///< ...and only if |gyro| < this (rad/s): trust "down = accel"
                           ///< only when quasi-static; during dynamic rotation the gyro
                           ///< alone tracks attitude (lateral accel would fool the aid)
  double innov_max = 0.15; ///< health: stance velocity-constraint residual bound (m/s)
  double damping = 1e-6;   ///< Tikhonov damping for the base-velocity least-squares solve
  bool flat_ground = false;  ///< pin re-anchored foot HEIGHTS to the seed ground level, so
                             ///< leg-odometry height cannot drift step-to-step (a flat-floor
                             ///< assumption for WALKING; xy still re-anchors for progress).
                             ///< The general (terrain) case is the deferred InEKF's job.
};

class BaseEstimator
{
public:
  /// `model` must be a FLOATING-base RobotModel and outlive this estimator (a const
  /// reference is held for the per-tick FK/Jacobian queries).
  BaseEstimator(const RobotModel & model, BaseEstimatorConfig cfg);

  /// Bumpless start: set the base pose from a known state and anchor the feet that are
  /// in stance now (their current world positions become the leg-odometry references).
  /// v is set to zero. Call on activation, exactly as LqgController seeds its observer.
  void seed(
    const Eigen::Vector3d & p, const Eigen::Matrix3d & R,
    const Eigen::VectorXd & q_joints, const std::vector<uint8_t> & stance);

  /// IMU dead-reckoning over dt. `gyro` and `accel` are body-frame; `accel` is the
  /// measured specific force (a static, level base reads ~[0, 0, +gravity]).
  void predict(const Eigen::Vector3d & gyro, const Eigen::Vector3d & accel, double dt);

  /// Measurement update from the joint encoders (`q_joints`, `v_joints`, in
  /// cfg.actuated_joints order) and the per-foot `stance` mask (1 = in contact, in
  /// cfg.contact_frames order). Bounds the predict() drift.
  void correct(
    const Eigen::VectorXd & q_joints, const Eigen::VectorXd & v_joints,
    const std::vector<uint8_t> & stance);

  const BaseState & state() const { return x_; }
  Eigen::Quaterniond orientation() const { return Eigen::Quaterniond(x_.R); }
  const Eigen::Vector3d & position() const { return x_.p; }
  const Eigen::Vector3d & velocity_world() const { return x_.v; }
  /// Body-frame linear velocity R^T v (Pinocchio free-flyer / REP-145 convention —
  /// what nav_msgs/Odometry.twist carries for OdometryBaseBridge).
  Eigen::Vector3d velocity_body() const { return x_.R.transpose() * x_.v; }
  /// Body-frame angular velocity (the last gyro reading).
  const Eigen::Vector3d & angular_body() const { return omega_; }
  const Status & status() const { return status_; }

private:
  /// Fill q_full_ (nq) from the current base state x_ and the measured joint positions.
  void assemble_q(const Eigen::VectorXd & q_joints);

  const RobotModel & model_;
  BaseEstimatorConfig cfg_;
  int nv_ = 0;
  int njoint_ = 0;                    ///< number of actuated joints
  Eigen::Vector3d g_;                 ///< [0, 0, -gravity]

  BaseState x_;
  Eigen::Vector3d omega_ = Eigen::Vector3d::Zero();  ///< last gyro (body angular velocity)
  Eigen::Vector3d accel_ = Eigen::Vector3d::Zero();  ///< last accel (body specific force)

  RobotModel::Workspace ws_;
  std::vector<std::size_t> foot_ids_;         ///< Pinocchio frame id per contact frame
  std::vector<int> qidx_, vidx_;              ///< per actuated joint: full q / v index
  std::vector<Eigen::Vector3d> anchors_;      ///< per foot: world anchor while in stance
  std::vector<uint8_t> prev_stance_;
  double ground_z_ = 0.0;                     ///< seed ground height (flat_ground height pin)

  // Preallocated per-tick buffers (allocation-free correct()).
  Eigen::VectorXd q_full_, vj_full_;
  Eigen::MatrixXd J_;                         ///< stacked stance contact Jacobian (3ns x nv)
  std::vector<std::size_t> stance_ids_;
  Status status_;
};

}  // namespace kontrolem_estimation

#endif  // KONTROLEM_ESTIMATION__BASE_ESTIMATOR_HPP_
