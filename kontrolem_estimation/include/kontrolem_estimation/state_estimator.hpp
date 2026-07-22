// IStateEstimator — the floating-base state-estimator interface. Both the M8 contact-aided
// complementary filter (BaseEstimator) and the M11 Right-Invariant EKF (InvariantEstimator)
// implement it, so the runtime selects one behind a param (estimator_type) and holds it by
// unique_ptr<IStateEstimator>. The shape is the SE_2(3)-carrying seed/predict/correct loop
// agreed at M8 — a real robot has no ground-truth oracle for its floating base and must
// estimate (R, v, p) from an IMU + joint encoders + foot-contact detection.
#ifndef KONTROLEM_ESTIMATION__STATE_ESTIMATOR_HPP_
#define KONTROLEM_ESTIMATION__STATE_ESTIMATOR_HPP_

#include <cstdint>
#include <vector>

#include <Eigen/Dense>

#include "kontrolem_control/types.hpp"

namespace kontrolem_estimation
{

using kontrolem_control::Status;

/// Floating-base state, an SE_2(3) element. R: world<-base orientation. v: world-frame base
/// linear velocity. p: world-frame base position. (v/p are world-frame so a right-invariant
/// EKF carries exactly this shape.)
struct BaseState
{
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
};

/// Abstract floating-base estimator. predict() dead-reckons the IMU; correct() bounds the
/// drift from leg odometry (joint encoders + per-foot stance mask).
class IStateEstimator
{
public:
  virtual ~IStateEstimator() = default;

  /// Bumpless start from a known base pose (v := 0), anchoring the feet in stance now.
  virtual void seed(
    const Eigen::Vector3d & p, const Eigen::Matrix3d & R,
    const Eigen::VectorXd & q_joints, const std::vector<uint8_t> & stance) = 0;

  /// IMU dead-reckoning over dt (body-frame gyro + specific-force accel).
  virtual void predict(
    const Eigen::Vector3d & gyro, const Eigen::Vector3d & accel, double dt) = 0;

  /// Measurement update from the joint encoders (q_joints, v_joints in actuated_joints
  /// order) and the per-foot stance mask (1 = in contact, in contact_frames order).
  virtual void correct(
    const Eigen::VectorXd & q_joints, const Eigen::VectorXd & v_joints,
    const std::vector<uint8_t> & stance) = 0;

  virtual const BaseState & state() const = 0;
  virtual Eigen::Quaterniond orientation() const = 0;
  virtual const Eigen::Vector3d & position() const = 0;
  virtual const Eigen::Vector3d & velocity_world() const = 0;
  /// Body-frame linear velocity R^T v (Pinocchio free-flyer / REP-145 convention).
  virtual Eigen::Vector3d velocity_body() const = 0;
  /// Body-frame angular velocity (the last gyro reading).
  virtual const Eigen::Vector3d & angular_body() const = 0;
  virtual const Status & status() const = 0;
};

}  // namespace kontrolem_estimation

#endif  // KONTROLEM_ESTIMATION__STATE_ESTIMATOR_HPP_
