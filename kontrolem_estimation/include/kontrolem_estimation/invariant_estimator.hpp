// InvariantEstimator — a contact-aided Right-Invariant EKF (RIEKF) for the floating base
// (Hartley, Ghaffari, Eustice, Grizzle 2020, "Contact-Aided Invariant EKF"). It is the
// Kalman upgrade the M8 complementary BaseEstimator was structured to make room for: same
// seed()/predict()/correct()/state() surface (drop-in behind IStateEstimator), but it
// carries a COVARIANCE and per-foot CONTACT-POINT states, so each stance foot's
// forward-kinematics measurement corrects the base *weighted by confidence*. That is the
// fix for M10's walk-on-estimate frontier: a briefly mis-sensed swing foot no longer
// poisons a shared least-squares (as it does in the complementary filter) — it is excluded
// from the update, and the remaining good feet still correct the base.
//
// State X in SE_{2+N}(3): (R, v, p, d_1..d_N) — world<-base orientation, world velocity,
// world position, and one WORLD contact-point per foot. Error is RIGHT-invariant
// (eta = X X_hat^-1), which makes the propagation Jacobian A state-INDEPENDENT (the RIEKF
// property) and the forward-kinematics measurement Jacobian H constant: the innovation for
// stance foot i reduces to (xi_{d_i} - xi_p), i.e. H_i = [0, 0, -I, ..., +I(at d_i), ...],
// and the raw innovation is simply (FK world foot position) - d_hat_i.
//
// No IMU-bias states in v1 (a documented +6 extension). Eigen + kontrolem_model only;
// preallocated / allocation-free after construction (measurement dimension is held FIXED at
// 3N+3 — non-stance feet get a huge measurement noise so their update contribution is ~0 —
// which keeps every matrix a fixed size and the LDLT preallocated).
#ifndef KONTROLEM_ESTIMATION__INVARIANT_ESTIMATOR_HPP_
#define KONTROLEM_ESTIMATION__INVARIANT_ESTIMATOR_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "kontrolem_estimation/state_estimator.hpp"  // BaseState, IStateEstimator
#include "kontrolem_model/robot_model.hpp"

namespace kontrolem_estimation
{

using kontrolem_model::RobotModel;

/// RIEKF configuration. Process terms are continuous-time noise DENSITIES (std-dev per
/// sqrt(Hz)); measurement terms are std-devs. Defaults are sized for the Go2 crawl walk.
struct InvariantEstimatorConfig
{
  std::vector<std::string> contact_frames;   ///< foot frames; defines the stance-mask order
  std::vector<std::string> actuated_joints;  ///< encoder order for q_joints / v_joints
  double gravity = 9.81;                      ///< |g| (m/s^2); g_world = [0, 0, -gravity]

  // --- process noise densities ---
  double sigma_gyro = 0.01;        ///< gyro white noise (rad/s/sqrt(Hz)) -> orientation
  double sigma_accel = 0.1;        ///< accel white noise (m/s^2/sqrt(Hz)) -> velocity
  double sigma_pos = 1e-4;         ///< tiny position process noise (numerical)
  double sigma_contact = 1e-3;     ///< stance foot slip random-walk (m/s/sqrt(Hz))
  double sigma_contact_swing = 1e3;///< swing foot: unconstrained (huge)

  // --- measurement noise ---
  double sigma_fk = 0.02;          ///< forward-kinematics position noise (m)
  double sigma_grav = 0.2;         ///< gravity-direction (accel) noise (unitless, normalized)
  double fk_gate = 0.06;           ///< robust reject: drop a stance foot whose FK innovation
                                   ///< |FK - anchor| exceeds this (m) — a mis-sensed swing
                                   ///< foot moved off its anchor, so its measurement is bad.

  // --- gravity attitude-aid gate (reused from the M8 filter) ---
  double accel_gate = 0.5;  ///< accept gravity aiding only if | |accel| - g | < this (m/s^2)
  double gyro_gate = 0.5;   ///< ...and only if |gyro| < this (rad/s): trust "down = accel"

  // --- initial covariance (std-devs) ---
  double init_ori = 0.1;      ///< initial orientation uncertainty (rad)
  double init_vel = 0.1;      ///< initial velocity uncertainty (m/s)
  double init_pos = 0.01;     ///< initial position uncertainty (m)
  double init_contact = 0.05; ///< initial / re-anchored contact-point uncertainty (m)

  double innov_max = 0.15;  ///< health: FK innovation bound (m) for status().ok
};

/// Contact-aided Right-Invariant EKF. Implements the same IStateEstimator surface as the
/// M8 BaseEstimator, so the runtime swaps them behind a param.
class InvariantEstimator final : public IStateEstimator
{
public:
  /// `model` must be a FLOATING-base RobotModel and outlive this estimator.
  InvariantEstimator(const RobotModel & model, InvariantEstimatorConfig cfg);

  /// Bumpless start: base pose from a known state, v = 0, each contact anchored at its FK
  /// world position, covariance reset to the configured priors.
  void seed(
    const Eigen::Vector3d & p, const Eigen::Matrix3d & R,
    const Eigen::VectorXd & q_joints, const std::vector<uint8_t> & stance) override;

  /// IMU dead-reckoning over dt (body-frame gyro + specific-force accel), and covariance
  /// propagation P <- Phi P Phi^T + Qd on the right-invariant error.
  void predict(const Eigen::Vector3d & gyro, const Eigen::Vector3d & accel, double dt) override;

  /// Measurement update: each stance foot's forward-kinematics world position corrects the
  /// base + that foot's contact-point state; a gated gravity-direction measurement aids
  /// roll/pitch. Touchdown re-anchors a foot; liftoff drops it from the update.
  void correct(
    const Eigen::VectorXd & q_joints, const Eigen::VectorXd & v_joints,
    const std::vector<uint8_t> & stance) override;

  const BaseState & state() const override { return x_; }
  Eigen::Quaterniond orientation() const override { return Eigen::Quaterniond(x_.R); }
  const Eigen::Vector3d & position() const override { return x_.p; }
  const Eigen::Vector3d & velocity_world() const override { return x_.v; }
  Eigen::Vector3d velocity_body() const override { return x_.R.transpose() * x_.v; }
  const Eigen::Vector3d & angular_body() const override { return omega_; }
  const Status & status() const override { return status_; }

private:
  void assemble_q(const Eigen::VectorXd & q_joints);  ///< fill q_full_ from x_ + encoders
  int di(int foot) const { return 9 + 3 * foot; }     ///< error-vector index of contact foot

  const RobotModel & model_;
  InvariantEstimatorConfig cfg_;
  int nv_ = 0, njoint_ = 0, N_ = 0;  ///< dof, #joints, #feet
  int n_ = 0, m_ = 0;                ///< error dim (9+3N), measurement dim (3N+3)
  Eigen::Vector3d g_;

  BaseState x_;                             ///< R, v, p
  std::vector<Eigen::Vector3d> d_;          ///< per-foot world contact-point states
  Eigen::MatrixXd P_;                       ///< covariance (n x n)
  Eigen::Vector3d omega_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_ = Eigen::Vector3d::Zero();

  RobotModel::Workspace ws_;
  std::vector<std::size_t> foot_ids_;
  std::vector<int> qidx_, vidx_;
  std::vector<uint8_t> prev_stance_, stance_;
  std::vector<Eigen::Vector3d> fkpos_;  ///< per-foot FK world position this tick
  Status status_;

  // Preallocated buffers (products use preallocated members; a few Eigen decomposition
  // temporaries remain — a full alloc audit is a later step, mirroring the M8 filter's).
  Eigen::VectorXd q_full_;
  Eigen::MatrixXd A_, A2_, Phi_, Qc_, Qd_, PhiP_;                 // propagation
  Eigen::MatrixXd H_, HP_, S_, Kt_, K_, KH_, ImKH_, Pj_, tmpNN_, tmpNM_;  // update
  Eigen::VectorXd y_, dx_, Ndiag_;
  Eigen::LDLT<Eigen::MatrixXd> ldlt_;
};

}  // namespace kontrolem_estimation

#endif  // KONTROLEM_ESTIMATION__INVARIANT_ESTIMATOR_HPP_
