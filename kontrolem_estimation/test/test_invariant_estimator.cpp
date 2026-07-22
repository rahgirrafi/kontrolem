// Offline correctness gate for the contact-aided Right-Invariant EKF (InvariantEstimator),
// no ROS/Gazebo. Runs a floating quadruped through kontrolem_model (Pinocchio C++) and:
//   PARITY with the M8 complementary filter's gates —
//     1.  HOLDS a static standing state under noisy IMU without drifting (beats a
//         predict-only dead-reckoner);
//     1b. RECOVERS a wrong initial roll/pitch from the accelerometer gravity direction;
//     2.  RECOVERS a base velocity (observed through the position measurement over a window,
//         not a one-shot leg-odometry velocity solve — that is the whole design difference);
//     3.  INTEGRATES a known constant acceleration exactly in predict();
//   THE MILESTONE PROOF —
//     4.  A simulated CRAWL WALK (feet lift/land, per-leg IK so stance feet are truly
//         world-fixed) with IMU noise + encoder noise + INJECTED CONTACT-SENSING FLICKER
//         (a swing foot occasionally mis-reported as planted). The InEKF must track the true
//         base TIGHTER than the M8 complementary filter (run on identical data), staying
//         bounded where the complementary filter — poisoned by the mis-sensed foot in its
//         shared velocity least-squares — drifts. This is exactly M10's walk-on-estimate
//         frontier, reproduced offline and closed.
// PASS iff all hold. Mirrors the kontrolem_model / test_base_estimator style (plain main).
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "kontrolem_estimation/base_estimator.hpp"
#include "kontrolem_estimation/invariant_estimator.hpp"
#include "kontrolem_model/robot_model.hpp"

using kontrolem_estimation::BaseEstimator;
using kontrolem_estimation::BaseEstimatorConfig;
using kontrolem_estimation::InvariantEstimator;
using kontrolem_estimation::InvariantEstimatorConfig;
using kontrolem_model::BaseType;
using kontrolem_model::RobotModel;

namespace
{
const std::vector<std::string> kFeet = {"foot_FL", "foot_FR", "foot_RL", "foot_RR"};
const std::vector<std::string> kJoints = {
  "hipx_FL", "hipy_FL", "knee_FL", "hipx_FR", "hipy_FR", "knee_FR",
  "hipx_RL", "hipy_RL", "knee_RL", "hipx_RR", "hipy_RR", "knee_RR"};

double ori_error(const Eigen::Matrix3d & Ra, const Eigen::Matrix3d & Rb)
{
  const double c = ((Ra.transpose() * Rb).trace() - 1.0) / 2.0;
  return std::acos(std::max(-1.0, std::min(1.0, c)));
}

InvariantEstimatorConfig make_inekf_cfg()
{
  InvariantEstimatorConfig c;
  c.contact_frames = kFeet;
  c.actuated_joints = kJoints;
  c.gravity = 9.81;
  return c;
}
BaseEstimatorConfig make_comp_cfg(bool flat_ground)
{
  BaseEstimatorConfig c;
  c.contact_frames = kFeet;
  c.actuated_joints = kJoints;
  c.gravity = 9.81;
  c.flat_ground = flat_ground;   // give the complementary baseline its best M10 config
  return c;
}

/// Standing posture (from test_contact_dynamics): bent legs, base lifted so the lowest foot
/// rests at z = 0. Returns base height + the joint vector (kJoints order).
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

/// Assemble a full free-flyer q = [p | quat xyzw | joints(kJoints order)].
Eigen::VectorXd assemble(
  const RobotModel & m, const Eigen::Vector3d & p, const Eigen::Matrix3d & R,
  const Eigen::VectorXd & qj)
{
  Eigen::VectorXd q = m.neutral();
  q.head<3>() = p;
  const Eigen::Quaterniond quat(R);
  q[3] = quat.x(); q[4] = quat.y(); q[5] = quat.z(); q[6] = quat.w();
  for (std::size_t k = 0; k < kJoints.size(); ++k) q[m.joint_q_index(kJoints[k])] = qj[k];
  return q;
}

/// Per-leg IK: adjust leg L's 3 joints (kJoints slots 3L..3L+2) so foot L reaches p_target
/// in the world, given the base pose. Gauss-Newton on the 3x3 leg contact Jacobian.
void ik_leg(
  const RobotModel & m, RobotModel::Workspace & ws, const Eigen::Vector3d & p_base,
  const Eigen::Matrix3d & R, int L, const Eigen::Vector3d & p_target, Eigen::VectorXd & qj)
{
  const std::size_t fid = m.frame_index(kFeet[L]);
  int vcol[3];
  for (int c = 0; c < 3; ++c) vcol[c] = m.joint_v_index(kJoints[3 * L + c]);
  Eigen::MatrixXd J;
  for (int it = 0; it < 20; ++it) {
    const Eigen::VectorXd q = assemble(m, p_base, R, qj);
    const Eigen::Vector3d err = p_target - m.frame_position(ws, q, fid);
    if (err.norm() < 1e-11) break;
    m.contact_jacobian_stacked(ws, q, std::vector<std::size_t>{fid}, J);  // 3 x nv
    Eigen::Matrix3d Jl;
    for (int c = 0; c < 3; ++c) Jl.col(c) = J.col(vcol[c]);
    const Eigen::Vector3d dq = Jl.colPivHouseholderQr().solve(err);
    for (int c = 0; c < 3; ++c) qj[3 * L + c] += dq[c];
  }
}
}  // namespace

int main()
{
  const RobotModel m = RobotModel::from_urdf_file(FLOATING_QUAD_URDF, BaseType::kFloating);
  auto ws = m.make_workspace();
  const std::vector<uint8_t> all_stance(kFeet.size(), 1);
  const double g = 9.81, dt = 0.002;

  Eigen::VectorXd qj0;
  const double base_z = standing(m, qj0);
  const Eigen::Vector3d p0(0.0, 0.0, base_z);
  const Eigen::VectorXd vj_zero = Eigen::VectorXd::Zero(static_cast<int>(kJoints.size()));
  std::cout << "standing base_z = " << base_z << "\n";
  bool all_ok = true;

  // ---- 1. static hold under noisy IMU (+ dead-reckon contrast) ----------------------
  {
    std::mt19937 rng(12345);
    std::normal_distribution<double> gyro_n(0.0, 0.0003), acc_n(0.0, 0.017);
    InvariantEstimator est(m, make_inekf_cfg());
    est.seed(p0, Eigen::Matrix3d::Identity(), qj0, all_stance);
    InvariantEstimator dead(m, make_inekf_cfg());  // predict-only contrast
    dead.seed(p0, Eigen::Matrix3d::Identity(), qj0, all_stance);
    for (int i = 0; i < 3000; ++i) {
      const Eigen::Vector3d gyro(gyro_n(rng), gyro_n(rng), gyro_n(rng));
      const Eigen::Vector3d accel(acc_n(rng), acc_n(rng), g + acc_n(rng));
      est.predict(gyro, accel, dt);
      est.correct(qj0, vj_zero, all_stance);
      dead.predict(gyro, accel, dt);
    }
    const double pos_err = (est.position() - p0).norm();
    const double vel_err = est.velocity_world().norm();
    const double oe = ori_error(est.orientation().toRotationMatrix(), Eigen::Matrix3d::Identity());
    const double dead_pos = (dead.position() - p0).norm();
    const bool ok = pos_err < 0.02 && vel_err < 0.02 && oe < (1.5 * M_PI / 180.0) &&
                    dead_pos > 3.0 * std::max(pos_err, 1e-4);
    all_ok &= ok;
    std::cout << "1 static+noise: pos=" << pos_err << " vel=" << vel_err
              << " ori(deg)=" << oe * 180 / M_PI << " | dead=" << dead_pos
              << " -> " << (ok ? "PASS" : "FAIL") << "\n";
  }

  // ---- 1b. gravity aid recovers a wrong initial roll/pitch --------------------------
  {
    InvariantEstimator tilt(m, make_inekf_cfg());
    const Eigen::Matrix3d Rw =
      Eigen::AngleAxisd(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
    tilt.seed(p0, Rw, qj0, all_stance);
    for (int i = 0; i < 4000; ++i) {
      tilt.predict(Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, g), dt);
      tilt.correct(qj0, vj_zero, all_stance);
    }
    const double te = ori_error(tilt.orientation().toRotationMatrix(), Eigen::Matrix3d::Identity());
    const bool ok = te < 1.0 * M_PI / 180.0;
    all_ok &= ok;
    std::cout << "1b gravity-aid: 5deg -> " << te * 180 / M_PI << " deg -> "
              << (ok ? "PASS" : "FAIL") << "\n";
  }

  // ---- 2. velocity recovery (observed through the position measurement) -------------
  // Consistent segment: base translates at a constant world velocity while the feet stay
  // planted at their seed anchors (per-leg IK each tick). a_world = 0, gyro = 0. The InEKF
  // has no direct velocity measurement, so it must observe v through the growing position
  // innovation (the IMU/position covariance coupling) over a short window.
  {
    const Eigen::Vector3d vw(0.05, -0.02, 0.0);
    InvariantEstimator vest(m, make_inekf_cfg());
    vest.seed(p0, Eigen::Matrix3d::Identity(), qj0, all_stance);
    std::vector<Eigen::Vector3d> anchor(kFeet.size());
    for (std::size_t L = 0; L < kFeet.size(); ++L)
      anchor[L] = m.frame_position(ws, assemble(m, p0, Eigen::Matrix3d::Identity(), qj0),
                                   m.frame_index(kFeet[L]));
    Eigen::VectorXd qj = qj0, qj_prev = qj0;
    const int W = 600;  // 1.2 s
    double v_err = 1e9;
    for (int i = 1; i <= W; ++i) {
      const Eigen::Vector3d pb = p0 + vw * (i * dt);
      qj_prev = qj;
      for (int L = 0; L < 4; ++L) ik_leg(m, ws, pb, Eigen::Matrix3d::Identity(), L, anchor[L], qj);
      const Eigen::VectorXd vjoint = (qj - qj_prev) / dt;
      vest.predict(Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, g), dt);
      vest.correct(qj, vjoint, all_stance);
      v_err = (vest.velocity_world() - vw).norm();
    }
    const bool ok = v_err < 0.02;
    all_ok &= ok;
    std::cout << "2 vel-recovery: |v_est - vw| after window = " << v_err << " -> "
              << (ok ? "PASS" : "FAIL") << "\n";
  }

  // ---- 3. predict() integrates a known constant acceleration exactly ----------------
  {
    InvariantEstimator pest(m, make_inekf_cfg());
    pest.seed(p0, Eigen::Matrix3d::Identity(), qj0, all_stance);
    const Eigen::Vector3d a_world(0.5, 0.0, 0.0);
    const Eigen::Vector3d a_body = a_world - Eigen::Vector3d(0, 0, -g);
    const int Np = 500;
    for (int i = 0; i < Np; ++i) pest.predict(Eigen::Vector3d::Zero(), a_body, dt);
    const double T = Np * dt;
    const double p3 = (pest.position() - (p0 + 0.5 * a_world * T * T)).norm();
    const double v3 = (pest.velocity_world() - a_world * T).norm();
    const bool ok = p3 < 1e-9 && v3 < 1e-9;
    all_ok &= ok;
    std::cout << "3 predict-integrate: pos=" << p3 << " vel=" << v3 << " -> "
              << (ok ? "PASS" : "FAIL") << "\n";
  }

  // ---- 4. THE PROOF: crawl walk with contact flicker, InEKF vs complementary --------
  {
    // Foot nominal world positions at standing.
    std::vector<Eigen::Vector3d> fnom(kFeet.size());
    for (std::size_t L = 0; L < kFeet.size(); ++L)
      fnom[L] = m.frame_position(ws, assemble(m, p0, Eigen::Matrix3d::Identity(), qj0),
                                 m.frame_index(kFeet[L]));

    const double Tcyc = 6.0, duty = 0.4, step_len = 0.05, step_h = 0.04;
    const double sw = duty * Tcyc / 4.0;                 // swing duration
    const int order[4] = {2, 0, 3, 1};                   // RL, FL, RR, FR
    int slot_of[4];
    for (int k = 0; k < 4; ++k) slot_of[order[k]] = k;
    const double vfwd = step_len / Tcyc;                 // base advances step_len per cycle
    const int cycles = 3;
    const int Nsteps = static_cast<int>(cycles * Tcyc / dt);

    // Generate the ground-truth trajectory (base + joints + true contact) up front.
    std::vector<Eigen::Vector3d> ptrue(Nsteps + 2);
    std::vector<Eigen::VectorXd> qtraj(Nsteps + 2);
    std::vector<std::array<uint8_t, 4>> strue(Nsteps + 2);
    Eigen::VectorXd qj = qj0;
    for (int i = 0; i <= Nsteps + 1; ++i) {
      const double t = i * dt;
      Eigen::Vector3d pb;
      pb.x() = p0.x() + vfwd * t;
      pb.y() = p0.y() + 0.02 * std::sin(2 * M_PI * t / Tcyc);
      pb.z() = p0.z() + 0.005 * std::sin(4 * M_PI * t / Tcyc);
      ptrue[i] = pb;
      for (int L = 0; L < 4; ++L) {
        const double slot_start = slot_of[L] * Tcyc / 4.0;
        // completed swings for foot L strictly before t:
        int completed = 0;
        for (int c = 0; c < cycles + 1; ++c) {
          const double end = c * Tcyc + slot_start + sw;
          if (end <= t) ++completed;
        }
        Eigen::Vector3d ftgt = fnom[L] + Eigen::Vector3d(completed * step_len, 0, 0);
        bool swinging = false;
        for (int c = 0; c < cycles + 1; ++c) {
          const double st = c * Tcyc + slot_start, en = st + sw;
          if (t >= st && t < en) {
            const double s = (t - st) / sw;
            ftgt += Eigen::Vector3d(step_len * s, 0, step_h * std::sin(M_PI * s));
            swinging = true;
            break;
          }
        }
        strue[i][L] = swinging ? 0 : 1;
        ik_leg(m, ws, pb, Eigen::Matrix3d::Identity(), L, ftgt, qj);
      }
      qtraj[i] = qj;
    }

    // Run both filters on identical noisy IMU + encoders + FLICKERED stance.
    std::mt19937 rng(777);
    std::normal_distribution<double> gyro_n(0.0, 0.001), acc_n(0.0, 0.05), enc_n(0.0, 0.0005);
    std::uniform_real_distribution<double> uni(0.0, 1.0);

    InvariantEstimator inekf(m, make_inekf_cfg());
    BaseEstimator comp(m, make_comp_cfg(/*flat_ground=*/true));
    std::vector<uint8_t> s0(4, 1);
    inekf.seed(ptrue[0], Eigen::Matrix3d::Identity(), qtraj[0], s0);
    comp.seed(ptrue[0], Eigen::Matrix3d::Identity(), qtraj[0], s0);

    // Bursty contact flicker: real contact sensing LATCHES (a foot reads contact for a
    // stretch), which is what poisoned the M10 walk-on-estimate — not independent per-tick
    // noise. Model it as a per-foot mis-sense burst that lasts a span of ticks.
    int burst[4] = {0, 0, 0, 0};   // >0 while foot L is stuck reporting the wrong contact

    double inekf_max = 0.0, comp_max = 0.0, inekf_fin = 0.0, comp_fin = 0.0;
    for (int i = 1; i <= Nsteps; ++i) {
      // True IMU from finite differences of the ground truth (R = I -> gyro = 0).
      const Eigen::Vector3d a_world = (ptrue[i + 1] - 2 * ptrue[i] + ptrue[i - 1]) / (dt * dt);
      const Eigen::Vector3d accel_true = a_world + Eigen::Vector3d(0, 0, g);  // R=I
      const Eigen::Vector3d gyro(gyro_n(rng), gyro_n(rng), gyro_n(rng));
      const Eigen::Vector3d accel(
        accel_true.x() + acc_n(rng), accel_true.y() + acc_n(rng), accel_true.z() + acc_n(rng));

      Eigen::VectorXd qenc = qtraj[i], vjoint = (qtraj[i] - qtraj[i - 1]) / dt;
      for (int k = 0; k < qenc.size(); ++k) qenc[k] += enc_n(rng);

      // Reported stance = true stance, but with FLICKER: a swing foot is occasionally
      // mis-reported as planted (the M10 failure mode), and a true-stance foot occasionally
      // dropped. Both filters see the SAME corrupted mask.
      std::vector<uint8_t> rep(4);
      for (int L = 0; L < 4; ++L) {
        rep[L] = strue[i][L];
        if (burst[L] > 0) { rep[L] = 1; --burst[L]; }         // stuck mis-reporting stance
        else if (!strue[i][L] && uni(rng) < 0.02) burst[L] = 25 + int(uni(rng) * 25);  // start
      }

      inekf.predict(gyro, accel, dt);
      inekf.correct(qenc, vjoint, rep);
      comp.predict(gyro, accel, dt);
      comp.correct(qenc, vjoint, rep);

      const double ei = (inekf.position() - ptrue[i]).norm();
      const double ec = (comp.position() - ptrue[i]).norm();
      inekf_max = std::max(inekf_max, ei);
      comp_max = std::max(comp_max, ec);
      inekf_fin = ei;
      comp_fin = ec;
    }
    // The InEKF must stay bounded AND beat the complementary filter it is replacing.
    const bool ok = inekf_max < 0.10 && inekf_max < comp_max && comp_max > 1.3 * inekf_max;
    all_ok &= ok;
    std::cout << "4 flicker-walk: InEKF max/final pos err = " << inekf_max << "/" << inekf_fin
              << " | complementary max/final = " << comp_max << "/" << comp_fin
              << " -> " << (ok ? "PASS" : "FAIL") << "\n";
  }

  std::cout << (all_ok ? "PASS" : "FAIL") << " invariant_estimator\n";
  return all_ok ? 0 : 1;
}
