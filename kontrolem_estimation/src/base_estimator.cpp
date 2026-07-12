#include "kontrolem_estimation/base_estimator.hpp"

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
    return Eigen::Matrix3d::Identity() + skew(w);  // 1st-order (unit-safe after renorm)
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

BaseEstimator::BaseEstimator(const RobotModel & model, BaseEstimatorConfig cfg)
: model_(model), cfg_(std::move(cfg)), ws_(model.make_workspace())
{
  nv_ = model_.nv();
  njoint_ = static_cast<int>(cfg_.actuated_joints.size());
  g_ = Eigen::Vector3d(0.0, 0.0, -cfg_.gravity);

  // Resolve foot frame ids and per-joint full-vector indices once, off the RT path.
  foot_ids_.reserve(cfg_.contact_frames.size());
  for (const auto & f : cfg_.contact_frames) foot_ids_.push_back(model_.frame_index(f));
  qidx_.reserve(njoint_);
  vidx_.reserve(njoint_);
  for (const auto & j : cfg_.actuated_joints) {
    qidx_.push_back(model_.joint_q_index(j));
    vidx_.push_back(model_.joint_v_index(j));
  }

  anchors_.assign(cfg_.contact_frames.size(), Eigen::Vector3d::Zero());
  prev_stance_.assign(cfg_.contact_frames.size(), 0);

  q_full_ = model_.neutral();
  vj_full_ = Eigen::VectorXd::Zero(nv_);   // joint-only generalized velocity (base = 0)
  J_.resize(0, nv_);
  stance_ids_.reserve(cfg_.contact_frames.size());
}

void BaseEstimator::assemble_q(const Eigen::VectorXd & q_joints)
{
  // Free-flyer configuration: [ position(3) | quaternion xyzw(4) | joints... ].
  q_full_[0] = x_.p.x(); q_full_[1] = x_.p.y(); q_full_[2] = x_.p.z();
  const Eigen::Quaterniond quat(x_.R);
  q_full_[3] = quat.x(); q_full_[4] = quat.y(); q_full_[5] = quat.z(); q_full_[6] = quat.w();
  for (int k = 0; k < njoint_; ++k) q_full_[qidx_[k]] = q_joints[k];
}

void BaseEstimator::seed(
  const Eigen::Vector3d & p, const Eigen::Matrix3d & R,
  const Eigen::VectorXd & q_joints, const std::vector<uint8_t> & stance)
{
  x_.p = p;
  x_.R = orthonormalize(R);
  x_.v.setZero();
  omega_.setZero();
  accel_ = Eigen::Vector3d(0.0, 0.0, cfg_.gravity);  // level, static default

  // Anchor the feet currently in stance at their FK world positions.
  assemble_q(q_joints);
  double zsum = 0.0;
  for (std::size_t i = 0; i < cfg_.contact_frames.size(); ++i) {
    anchors_[i] = model_.frame_position(q_full_, cfg_.contact_frames[i]);
    prev_stance_[i] = (i < stance.size()) ? stance[i] : 0;
    zsum += anchors_[i].z();
  }
  ground_z_ = cfg_.contact_frames.empty() ? 0.0 : zsum / cfg_.contact_frames.size();
  status_ = Status{};
}

void BaseEstimator::predict(
  const Eigen::Vector3d & gyro, const Eigen::Vector3d & accel, double dt)
{
  omega_ = gyro;
  accel_ = accel;

  // Orientation: integrate the body-frame gyro on SO(3).
  x_.R = orthonormalize(x_.R * expSO3(gyro * dt));

  // Velocity/position: rotate the specific force to world, add gravity, integrate.
  //   a_world = R * f_body + g_world     (static level base: f_body ~ [0,0,+g] -> a=0)
  const Eigen::Vector3d a_world = x_.R * accel + g_;
  x_.p += x_.v * dt + 0.5 * a_world * dt * dt;
  x_.v += a_world * dt;
}

void BaseEstimator::correct(
  const Eigen::VectorXd & q_joints, const Eigen::VectorXd & v_joints,
  const std::vector<uint8_t> & stance)
{
  assemble_q(q_joints);

  // ---- Leg-odometry velocity update ------------------------------------------------
  // A stance foot does not move in the world: v_foot_world = J v_full = 0, with
  //   v_full = [ v_base_body(3) | omega_body(3) | v_joints... ].
  // Split J = [Jb_lin | Jb_ang | Jj]; solve Jb_lin v_base_body = -(Jb_ang omega + Jj vj)
  // over all stance feet, then v_world = R v_base_body.
  stance_ids_.clear();
  for (std::size_t i = 0; i < foot_ids_.size(); ++i) {
    if (i < stance.size() && stance[i]) stance_ids_.push_back(foot_ids_[i]);
  }
  if (!stance_ids_.empty()) {
    model_.contact_jacobian_stacked(ws_, q_full_, stance_ids_, J_);  // (3ns x nv)
    const int rows = static_cast<int>(3 * stance_ids_.size());
    const auto Jb_lin = J_.block(0, 0, rows, 3);
    const auto Jb_ang = J_.block(0, 3, rows, 3);
    const auto Jj = J_.block(0, 6, rows, nv_ - 6);

    vj_full_.setZero();
    for (int k = 0; k < njoint_; ++k) vj_full_[vidx_[k]] = v_joints[k];
    const Eigen::VectorXd rhs = -(Jb_ang * omega_ + Jj * vj_full_.tail(nv_ - 6));

    const Eigen::Matrix3d H =
      Jb_lin.transpose() * Jb_lin + cfg_.damping * Eigen::Matrix3d::Identity();
    const Eigen::Vector3d vb_body = H.ldlt().solve(Jb_lin.transpose() * rhs);
    const Eigen::Vector3d v_meas = x_.R * vb_body;

    const double resid = (Jb_lin * vb_body - rhs).norm() / std::sqrt(static_cast<double>(rows));
    status_.ok = resid < cfg_.innov_max;
    status_.margin = cfg_.innov_max - resid;

    x_.v = (1.0 - cfg_.k_vel) * x_.v + cfg_.k_vel * v_meas;
  } else {
    status_.ok = false;          // airborne: no leg-odometry constraint this tick
    status_.margin = 0.0;
  }

  // ---- Leg-odometry position anchor update -----------------------------------------
  // A foot in continuous stance stays at its world anchor; the discrepancy with FK is
  // the base-position drift. On a rising edge (swing->stance) re-anchor at the current
  // FK position instead of correcting (the foot just landed somewhere new).
  Eigen::Vector3d perr = Eigen::Vector3d::Zero();
  int cnt = 0;
  for (std::size_t i = 0; i < foot_ids_.size(); ++i) {
    const bool st = (i < stance.size()) && stance[i];
    if (st) {
      const Eigen::Vector3d fw = model_.frame_position(q_full_, cfg_.contact_frames[i]);
      if (!prev_stance_[i]) {
        anchors_[i] = fw;               // just landed -> new anchor, no correction
        // Flat-ground height pin: keep the new anchor at the established ground level so
        // leg-odometry HEIGHT cannot drift step-to-step (xy still re-anchors for progress).
        if (cfg_.flat_ground) anchors_[i].z() = ground_z_;
      } else {
        perr += (anchors_[i] - fw);     // world position should not have moved
        ++cnt;
      }
    }
    prev_stance_[i] = st ? 1 : 0;
  }
  if (cnt > 0) x_.p += cfg_.k_pos * (perr / cnt);

  // ---- Accelerometer gravity attitude aid (roll/pitch) -----------------------------
  // When quasi-static (|accel| ~ g AND |gyro| small), the specific force points along
  // body "up", fixing roll/pitch (not yaw). Rotate the body so its estimated up meets
  // the measured up. Gating on the gyro too is what keeps a push transient tight: while
  // the base rotates fast the lateral accel would fool this aid, so we fall back to the
  // (short-term accurate) gyro integration alone.
  const double an = accel_.norm();
  if (an > 1e-6 && std::abs(an - cfg_.gravity) < cfg_.accel_gate &&
      omega_.norm() < cfg_.gyro_gate)
  {
    const Eigen::Vector3d u_meas = accel_ / an;                       // body: measured up
    const Eigen::Vector3d u_est = x_.R.transpose() * Eigen::Vector3d::UnitZ();  // est up
    const Eigen::Vector3d dtheta = cfg_.k_grav * u_meas.cross(u_est);
    x_.R = orthonormalize(x_.R * expSO3(dtheta));
  }
}

}  // namespace kontrolem_estimation
