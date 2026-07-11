// Offline correctness gate for the contact-aided BaseEstimator (no ROS/Gazebo). Runs a
// floating quadruped through kontrolem_model (Pinocchio C++) and asserts the filter:
//   1. HOLDS a static standing state under noisy IMU without drifting — the whole point
//      of contact aiding (a predict-only dead-reckoner is shown diverging for contrast);
//   1b. RECOVERS a wrong initial roll/pitch from the accelerometer's gravity direction;
//   2. RECOVERS a known base velocity from stance leg odometry (v_foot_world = 0);
//   3. INTEGRATES a known constant acceleration exactly in predict().
// PASS iff all hold. Mirrors the kontrolem_model test style (plain main, PASS/FAIL).
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "kontrolem_estimation/base_estimator.hpp"
#include "kontrolem_model/robot_model.hpp"

using kontrolem_estimation::BaseEstimator;
using kontrolem_estimation::BaseEstimatorConfig;
using kontrolem_model::BaseType;
using kontrolem_model::RobotModel;

namespace
{
const std::vector<std::string> kFeet = {"foot_FL", "foot_FR", "foot_RL", "foot_RR"};
const std::vector<std::string> kJoints = {
  "hipx_FL", "hipy_FL", "knee_FL", "hipx_FR", "hipy_FR", "knee_FR",
  "hipx_RL", "hipy_RL", "knee_RL", "hipx_RR", "hipy_RR", "knee_RR"};

/// Geodesic angle (rad) between two rotations.
double ori_error(const Eigen::Matrix3d & Ra, const Eigen::Matrix3d & Rb)
{
  const double c = ((Ra.transpose() * Rb).trace() - 1.0) / 2.0;
  return std::acos(std::max(-1.0, std::min(1.0, c)));
}

BaseEstimatorConfig make_cfg()
{
  BaseEstimatorConfig c;
  c.contact_frames = kFeet;
  c.actuated_joints = kJoints;
  c.gravity = 9.81;
  return c;
}

/// The standing posture from test_contact_dynamics: bent legs, base lifted so the
/// lowest foot rests at z = 0. Returns base height and the joint vector (kJoints order).
double standing(const RobotModel & m, Eigen::VectorXd & q_joints)
{
  Eigen::VectorXd q = m.neutral();
  for (const std::string leg : {"FL", "FR", "RL", "RR"}) {
    q(m.joint_q_index("hipx_" + leg)) = 0.0;
    q(m.joint_q_index("hipy_" + leg)) = 0.7;
    q(m.joint_q_index("knee_" + leg)) = -1.4;
  }
  double min_fz = 1e9;
  for (const auto & f : kFeet) min_fz = std::min(min_fz, m.frame_position(q, f).z());
  q(2) = -min_fz;
  q_joints.resize(static_cast<int>(kJoints.size()));
  for (std::size_t k = 0; k < kJoints.size(); ++k) q_joints[k] = q(m.joint_q_index(kJoints[k]));
  return q(2);
}
}  // namespace

int main()
{
  const RobotModel m = RobotModel::from_urdf_file(FLOATING_QUAD_URDF, BaseType::kFloating);
  const int nv = m.nv();
  const std::vector<uint8_t> stance(kFeet.size(), 1);  // all feet planted (standing)

  Eigen::VectorXd q_joints;
  const double base_z = standing(m, q_joints);
  const Eigen::Vector3d p_true(0.0, 0.0, base_z);
  const Eigen::VectorXd vj_zero = Eigen::VectorXd::Zero(static_cast<int>(kJoints.size()));
  const double g = 9.81;
  const double dt = 0.002;
  std::cout << "standing base_z = " << base_z << "\n";

  // ---- 1. static hold under noisy IMU (+ predict-only contrast) ---------------------
  std::mt19937 rng(12345);
  std::normal_distribution<double> gyro_n(0.0, 0.0003), acc_n(0.0, 0.017);
  BaseEstimator est(m, make_cfg());
  est.seed(p_true, Eigen::Matrix3d::Identity(), q_joints, stance);
  BaseEstimator dead(m, make_cfg());                 // predict-only, for contrast
  dead.seed(p_true, Eigen::Matrix3d::Identity(), q_joints, stance);

  const int N = 3000;                                // 6 s
  for (int i = 0; i < N; ++i) {
    const Eigen::Vector3d gyro(gyro_n(rng), gyro_n(rng), gyro_n(rng));
    const Eigen::Vector3d accel(acc_n(rng), acc_n(rng), g + acc_n(rng));
    est.predict(gyro, accel, dt);
    est.correct(q_joints, vj_zero, stance);
    dead.predict(gyro, accel, dt);                   // no correct(): pure dead-reckoning
  }
  const double pos_err = (est.position() - p_true).norm();
  const double vel_err = est.velocity_world().norm();
  const double ori_err = ori_error(est.orientation().toRotationMatrix(), Eigen::Matrix3d::Identity());
  const double dead_pos = (dead.position() - p_true).norm();
  const bool t1 = pos_err < 0.01 && vel_err < 0.01 && ori_err < (1.0 * M_PI / 180.0) &&
                  dead_pos > 5.0 * pos_err;           // contact aiding must beat dead-reckoning
  std::cout << "1 static+noise: pos_err=" << pos_err << " vel_err=" << vel_err
            << " ori_err(deg)=" << ori_err * 180.0 / M_PI
            << " | dead-reckon pos_err=" << dead_pos << " -> " << (t1 ? "PASS" : "FAIL") << "\n";

  // ---- 1b. gravity aid recovers a wrong initial roll/pitch --------------------------
  BaseEstimator tilt(m, make_cfg());
  const Eigen::Matrix3d R_wrong =
    Eigen::AngleAxisd(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
  tilt.seed(p_true, R_wrong, q_joints, stance);
  for (int i = 0; i < 4000; ++i) {                   // truly level+static: accel = [0,0,g]
    tilt.predict(Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, g), dt);
    tilt.correct(q_joints, vj_zero, stance);
  }
  const double tilt_err = ori_error(tilt.orientation().toRotationMatrix(), Eigen::Matrix3d::Identity());
  const bool t1b = tilt_err < 0.5 * M_PI / 180.0;    // converged from 5 deg to < 0.5 deg
  std::cout << "1b gravity-aid: 5deg -> " << tilt_err * 180.0 / M_PI << " deg -> "
            << (t1b ? "PASS" : "FAIL") << "\n";

  // ---- 2. velocity recovery from stance leg odometry --------------------------------
  // Tilt the base, pick a base linear velocity (omega = 0), synthesize the joint
  // velocities that keep all feet planted, and check correct() recovers v_world = R vb.
  const Eigen::Matrix3d R2 =
    Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Vector3d vb(0.10, -0.05, 0.03);       // base body linear velocity
  Eigen::VectorXd q_full = m.neutral();
  q_full.head<3>() = p_true;
  const Eigen::Quaterniond quat(R2);
  q_full[3] = quat.x(); q_full[4] = quat.y(); q_full[5] = quat.z(); q_full[6] = quat.w();
  for (std::size_t k = 0; k < kJoints.size(); ++k) q_full[m.joint_q_index(kJoints[k])] = q_joints[k];

  auto ws = m.make_workspace();
  Eigen::MatrixXd J;
  m.contact_jacobian_stacked(ws, q_full, kFeet, J);  // (12 x nv)
  // planted feet, omega = 0:  Jj vj = -Jb_lin vb
  const Eigen::VectorXd rhs = -(J.block(0, 0, J.rows(), 3) * vb);
  const Eigen::VectorXd vj_model =
    J.block(0, 6, J.rows(), nv - 6).colPivHouseholderQr().solve(rhs);   // model v-order joints
  Eigen::VectorXd v_joints(static_cast<int>(kJoints.size()));
  for (std::size_t k = 0; k < kJoints.size(); ++k) v_joints[k] = vj_model[m.joint_v_index(kJoints[k]) - 6];

  BaseEstimator vest(m, make_cfg());
  vest.seed(p_true, R2, q_joints, stance);           // seed sets v = 0 ...
  vest.correct(q_joints, v_joints, stance);          // ... correct() must recover it
  const double v_err = (vest.velocity_world() - R2 * vb).norm();
  const bool t2 = v_err < 1e-6;
  std::cout << "2 vel-recovery: |v_est - R*vb| = " << v_err << " -> " << (t2 ? "PASS" : "FAIL") << "\n";

  // ---- 3. predict() integrates a known constant acceleration exactly ----------------
  BaseEstimator pest(m, make_cfg());
  pest.seed(p_true, Eigen::Matrix3d::Identity(), q_joints, stance);
  const Eigen::Vector3d a_world(0.5, 0.0, 0.0);
  const Eigen::Vector3d a_body = a_world - Eigen::Vector3d(0, 0, -g);   // R = I; accel = a_world - g_world
  const int Np = 500;                                                  // 1 s
  for (int i = 0; i < Np; ++i) pest.predict(Eigen::Vector3d::Zero(), a_body, dt);
  const double T = Np * dt;
  const Eigen::Vector3d p_expect = p_true + 0.5 * a_world * T * T;
  const Eigen::Vector3d v_expect = a_world * T;
  const double p3 = (pest.position() - p_expect).norm();
  const double v3 = (pest.velocity_world() - v_expect).norm();
  const bool t3 = p3 < 1e-9 && v3 < 1e-9;
  std::cout << "3 predict-integrate: pos_err=" << p3 << " vel_err=" << v3 << " -> "
            << (t3 ? "PASS" : "FAIL") << "\n";

  const bool ok = t1 && t1b && t2 && t3;
  std::cout << (ok ? "PASS" : "FAIL") << " base_estimator\n";
  return ok ? 0 : 1;
}
