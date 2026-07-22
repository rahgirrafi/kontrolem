#include "kontrolem_estimation/invariant_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace kontrolem_estimation
{
namespace
{
/// Skew-symmetric matrix [w]_x with [w]_x u = w x u.
Eigen::Matrix3d skew(const Eigen::Vector3d & w)
{
  Eigen::Matrix3d S;
  S <<     0, -w.z(),  w.y(),
       w.z(),      0, -w.x(),
      -w.y(),  w.x(),      0;
  return S;
}

/// SO(3) exponential (Rodrigues): the rotation of angle |w| about w/|w|.
Eigen::Matrix3d expSO3(const Eigen::Vector3d & w)
{
  const double th = w.norm();
  if (th < 1e-9) {
    return Eigen::Matrix3d::Identity() + skew(w);
  }
  const Eigen::Vector3d a = w / th;
  const Eigen::Matrix3d K = skew(a);
  return Eigen::Matrix3d::Identity() + std::sin(th) * K + (1.0 - std::cos(th)) * K * K;
}

/// Re-orthonormalize a nearly-rotation matrix via its quaternion (keeps R in SO(3)).
Eigen::Matrix3d orthonormalize(const Eigen::Matrix3d & R)
{
  return Eigen::Quaterniond(R).normalized().toRotationMatrix();
}
}  // namespace

InvariantEstimator::InvariantEstimator(const RobotModel & model, InvariantEstimatorConfig cfg)
: model_(model), cfg_(std::move(cfg)), ws_(model.make_workspace())
{
  nv_ = model_.nv();
  njoint_ = static_cast<int>(cfg_.actuated_joints.size());
  N_ = static_cast<int>(cfg_.contact_frames.size());
  n_ = 9 + 3 * N_;
  m_ = 3 * N_ + 3;
  g_ = Eigen::Vector3d(0.0, 0.0, -cfg_.gravity);

  foot_ids_.reserve(N_);
  for (const auto & f : cfg_.contact_frames) foot_ids_.push_back(model_.frame_index(f));
  qidx_.reserve(njoint_);
  vidx_.reserve(njoint_);
  for (const auto & j : cfg_.actuated_joints) {
    qidx_.push_back(model_.joint_q_index(j));
    vidx_.push_back(model_.joint_v_index(j));
  }

  d_.assign(N_, Eigen::Vector3d::Zero());
  fkpos_.assign(N_, Eigen::Vector3d::Zero());
  prev_stance_.assign(N_, 0);
  stance_.assign(N_, 1);   // assume standing until the first correct() says otherwise
  q_full_ = model_.neutral();

  // Constant right-invariant propagation Jacobian A (state-independent — the RIEKF
  // property): xi_v depends on xi_R via g_x, xi_p on xi_v via I; contacts add zero rows.
  // A is nilpotent of index 3, so exp(A dt) = I + A dt + A^2 dt^2/2 exactly.
  A_ = Eigen::MatrixXd::Zero(n_, n_);
  A_.block<3, 3>(3, 0) = skew(g_);
  A_.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity();
  A2_ = A_ * A_;

  P_ = Eigen::MatrixXd::Zero(n_, n_);
  Phi_ = Eigen::MatrixXd::Identity(n_, n_);
  Qc_ = Eigen::MatrixXd::Zero(n_, n_);
  Qd_ = Eigen::MatrixXd::Zero(n_, n_);
  PhiP_ = Eigen::MatrixXd::Zero(n_, n_);
  tmpNN_ = Eigen::MatrixXd::Zero(n_, n_);
  KH_ = Eigen::MatrixXd::Zero(n_, n_);
  ImKH_ = Eigen::MatrixXd::Zero(n_, n_);
  Pj_ = Eigen::MatrixXd::Zero(n_, n_);

  // Constant part of the FK measurement Jacobian H: for foot i the innovation reduces to
  // (xi_{d_i} - xi_p), i.e. -I on the position block, +I on that foot's contact block. The
  // gravity rows (last 3) are refilled each tick (they carry R_hat). Non-stance feet keep
  // their rows but get a huge measurement noise, so the fixed m_ x n_ shape stays alloc-safe.
  H_ = Eigen::MatrixXd::Zero(m_, n_);
  for (int i = 0; i < N_; ++i) {
    H_.block<3, 3>(3 * i, 6) = -Eigen::Matrix3d::Identity();
    H_.block<3, 3>(3 * i, di(i)) = Eigen::Matrix3d::Identity();
  }
  HP_ = Eigen::MatrixXd::Zero(m_, n_);
  S_ = Eigen::MatrixXd::Zero(m_, m_);
  Kt_ = Eigen::MatrixXd::Zero(m_, n_);
  K_ = Eigen::MatrixXd::Zero(n_, m_);
  tmpNM_ = Eigen::MatrixXd::Zero(n_, m_);
  y_ = Eigen::VectorXd::Zero(m_);
  dx_ = Eigen::VectorXd::Zero(n_);
  Ndiag_ = Eigen::VectorXd::Zero(m_);
  ldlt_ = Eigen::LDLT<Eigen::MatrixXd>(m_);
}

void InvariantEstimator::assemble_q(const Eigen::VectorXd & q_joints)
{
  q_full_[0] = x_.p.x(); q_full_[1] = x_.p.y(); q_full_[2] = x_.p.z();
  const Eigen::Quaterniond quat(x_.R);
  q_full_[3] = quat.x(); q_full_[4] = quat.y(); q_full_[5] = quat.z(); q_full_[6] = quat.w();
  for (int k = 0; k < njoint_; ++k) q_full_[qidx_[k]] = q_joints[k];
}

void InvariantEstimator::seed(
  const Eigen::Vector3d & p, const Eigen::Matrix3d & R,
  const Eigen::VectorXd & q_joints, const std::vector<uint8_t> & stance)
{
  x_.p = p;
  x_.R = orthonormalize(R);
  x_.v.setZero();
  omega_.setZero();
  accel_ = Eigen::Vector3d(0.0, 0.0, cfg_.gravity);

  assemble_q(q_joints);
  for (int i = 0; i < N_; ++i) {
    d_[i] = model_.frame_position(ws_, q_full_, foot_ids_[i]);
    prev_stance_[i] = (i < static_cast<int>(stance.size())) ? stance[i] : 0;
    stance_[i] = prev_stance_[i];
  }

  // Prior covariance: block-diagonal, uncorrelated.
  P_.setZero();
  P_.block<3, 3>(0, 0) = cfg_.init_ori * cfg_.init_ori * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(3, 3) = cfg_.init_vel * cfg_.init_vel * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(6, 6) = cfg_.init_pos * cfg_.init_pos * Eigen::Matrix3d::Identity();
  for (int i = 0; i < N_; ++i) {
    P_.block<3, 3>(di(i), di(i)) =
      cfg_.init_contact * cfg_.init_contact * Eigen::Matrix3d::Identity();
  }
  status_ = Status{};
}

void InvariantEstimator::predict(
  const Eigen::Vector3d & gyro, const Eigen::Vector3d & accel, double dt)
{
  omega_ = gyro;
  accel_ = accel;

  // --- mean propagation (identical to the M8 dead-reckoner) ---
  x_.R = orthonormalize(x_.R * expSO3(gyro * dt));
  const Eigen::Vector3d a_world = x_.R * accel + g_;
  x_.p += x_.v * dt + 0.5 * a_world * dt * dt;
  x_.v += a_world * dt;
  // contact points d_ are fixed in the world under the deterministic model.

  // --- covariance propagation  P <- Phi P Phi^T + Qd ---
  Phi_.setIdentity();
  Phi_.noalias() += dt * A_;
  Phi_.noalias() += (0.5 * dt * dt) * A2_;

  // Process noise density (world frame, block-diagonal; isotropic blocks stay isotropic
  // under R_hat). Swing feet get a huge contact-point noise so they float free.
  Qc_.setZero();
  Qc_.block<3, 3>(0, 0) = cfg_.sigma_gyro * cfg_.sigma_gyro * Eigen::Matrix3d::Identity();
  Qc_.block<3, 3>(3, 3) = cfg_.sigma_accel * cfg_.sigma_accel * Eigen::Matrix3d::Identity();
  Qc_.block<3, 3>(6, 6) = cfg_.sigma_pos * cfg_.sigma_pos * Eigen::Matrix3d::Identity();
  for (int i = 0; i < N_; ++i) {
    const double s = stance_[i] ? cfg_.sigma_contact : cfg_.sigma_contact_swing;
    Qc_.block<3, 3>(di(i), di(i)) = s * s * Eigen::Matrix3d::Identity();
  }
  Qd_.noalias() = Qc_ * dt;

  PhiP_.noalias() = Phi_ * P_;
  P_.noalias() = PhiP_ * Phi_.transpose();
  P_ += Qd_;
}

void InvariantEstimator::correct(
  const Eigen::VectorXd & q_joints, const Eigen::VectorXd & /*v_joints*/,
  const std::vector<uint8_t> & stance)
{
  // The RIEKF corrects the base from each stance foot's forward-kinematic WORLD position
  // (position-only — velocity is observed through the IMU/position covariance coupling), so
  // it never runs the shared velocity least-squares that a mis-sensed foot poisons in the
  // complementary filter. v_joints is therefore unused here.
  assemble_q(q_joints);

  for (int i = 0; i < N_; ++i) {
    stance_[i] = (i < static_cast<int>(stance.size())) && stance[i];
    fkpos_[i] = model_.frame_position(ws_, q_full_, foot_ids_[i]);
    // Touchdown (swing->stance): re-anchor the contact at its current FK world position and
    // reset that foot's covariance (fresh, uncertain, uncorrelated) so the just-landed foot
    // does not yank the base with a stale anchor.
    if (stance_[i] && !prev_stance_[i]) {
      d_[i] = fkpos_[i];
      P_.block(di(i), 0, 3, n_).setZero();
      P_.block(0, di(i), n_, 3).setZero();
      P_.block<3, 3>(di(i), di(i)) =
        cfg_.init_contact * cfg_.init_contact * Eigen::Matrix3d::Identity();
    }
    prev_stance_[i] = stance_[i];
  }

  // --- build the fixed-size measurement (3N feet + 3 gravity) ---
  // Innovation r_foot_i = FK_world_i - d_hat_i  (= xi_{d_i} - xi_p, to first order). This
  // carries TWO things: the shared base-position drift (common to every stance foot) and any
  // per-foot mis-sense. Robust gating must reject only the latter, so it gates on each foot's
  // deviation from the CONSENSUS (component-wise median) of the reported-stance feet — a
  // mis-sensed swing foot (M10's poison) leaves the consensus as it lifts, while genuine base
  // drift moves all feet together and stays near the median. Gating on the raw magnitude
  // instead would reject every foot once the base drifts past the gate → runaway divergence.
  for (int i = 0; i < N_; ++i) y_.segment<3>(3 * i) = fkpos_[i] - d_[i];
  Eigen::Vector3d med = Eigen::Vector3d::Zero();
  {
    double buf[8];
    for (int c = 0; c < 3; ++c) {
      int ns = 0;
      for (int i = 0; i < N_; ++i) if (stance_[i]) buf[ns++] = y_[3 * i + c];
      if (ns > 0) {
        std::sort(buf, buf + ns);
        med[c] = (ns % 2) ? buf[ns / 2] : 0.5 * (buf[ns / 2 - 1] + buf[ns / 2]);
      }
    }
  }
  for (int i = 0; i < N_; ++i) {
    const bool accept =
      stance_[i] && (y_.segment<3>(3 * i) - med).norm() < cfg_.fk_gate;
    const double s = accept ? cfg_.sigma_fk : 1e6;   // reject -> ~no correction from this foot
    Ndiag_.segment<3>(3 * i).setConstant(s * s);
  }

  // Gravity-direction attitude aid (accel points along body "up" when quasi-static).
  const double an = accel_.norm();
  const bool grav_ok = an > 1e-6 && std::abs(an - cfg_.gravity) < cfg_.accel_gate &&
                       omega_.norm() < cfg_.gyro_gate;
  const Eigen::Vector3d zhat = Eigen::Vector3d::UnitZ();
  H_.block<3, 3>(3 * N_, 0) = x_.R.transpose() * skew(zhat);   // R_hat^T [z]_x on xi_R
  if (grav_ok) {
    y_.segment<3>(3 * N_) = accel_ / an - x_.R.transpose() * zhat;
    Ndiag_.segment<3>(3 * N_).setConstant(cfg_.sigma_grav * cfg_.sigma_grav);
  } else {
    y_.segment<3>(3 * N_).setZero();
    Ndiag_.segment<3>(3 * N_).setConstant(1e6);
  }

  // --- EKF update on the right-invariant error ---
  HP_.noalias() = H_ * P_;                 // (m x n)
  S_.noalias() = HP_ * H_.transpose();     // (m x m)
  S_.diagonal() += Ndiag_;
  ldlt_.compute(S_);
  Kt_ = ldlt_.solve(HP_);                  // S^-1 H P  = K^T   (m x n)
  K_ = Kt_.transpose();                    // (n x m)
  dx_.noalias() = K_ * y_;

  // Joseph-form covariance:  P = (I-KH) P (I-KH)^T + K N K^T.
  KH_.noalias() = K_ * H_;
  ImKH_ = -KH_;
  ImKH_.diagonal().array() += 1.0;
  tmpNN_.noalias() = ImKH_ * P_;
  Pj_.noalias() = tmpNN_ * ImKH_.transpose();
  tmpNM_.noalias() = K_ * Ndiag_.asDiagonal();
  Pj_.noalias() += tmpNM_ * K_.transpose();
  P_ = 0.5 * (Pj_ + Pj_.transpose());      // symmetrize

  // --- retract the state on SE_{2+N}(3) ---
  const Eigen::Vector3d dphi = dx_.head<3>();
  const Eigen::Matrix3d Rd = expSO3(dphi);
  x_.v = Rd * x_.v + dx_.segment<3>(3);
  x_.p = Rd * x_.p + dx_.segment<3>(6);
  for (int i = 0; i < N_; ++i) d_[i] = Rd * d_[i] + dx_.segment<3>(di(i));
  x_.R = orthonormalize(Rd * x_.R);

  // --- health: mean FK innovation over stance feet ---
  double resid = 0.0;
  int cnt = 0;
  for (int i = 0; i < N_; ++i) {
    if (stance_[i]) { resid += y_.segment<3>(3 * i).norm(); ++cnt; }
  }
  if (cnt > 0) {
    resid /= cnt;
    status_.ok = resid < cfg_.innov_max;
    status_.margin = cfg_.innov_max - resid;
  } else {
    status_.ok = false;   // airborne: no contact constraint this tick
    status_.margin = 0.0;
  }
}

}  // namespace kontrolem_estimation
