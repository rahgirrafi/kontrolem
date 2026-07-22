// M15 Step 0 — WBC diagonal TROT closed ON THE ESTIMATE, offline (the diagnostic harness).
//
// test_wbc_trot proved the whole-body QP trot is feasible + upright when the WBC sees the TRUE
// base. In Gazebo it is solid on ground truth but TIPS when the loop is closed on the robot's own
// InEKF estimate (M14's honest frontier). This harness reproduces that frontier with NO ROS: the
// SAME Go2-shaped quad, TrotGait, WbcController, and contact-constrained forward-dynamics plant as
// test_wbc_trot, but the WBC base is taken from an InvariantEstimator running in the loop on
// synthesized IMU + encoders + (optional) contact-sensing flicker. The plant integrates the TRUE
// dynamics under whatever torques the estimate-fed WBC commands, so a bad estimate tips the real
// robot — exactly as in Gazebo, but deterministic and in seconds.
//
// PURPOSE (Step 0): DIAGNOSE. It instruments which estimate component diverges (orientation /
// world-velocity / world-position) and WHEN (two-diagonal-foot swing vs all-four-stance bracket),
// and whether contact flicker is even the driver (Gazebo runs with planned-contact ON, i.e. a
// clean stance mask, and still tips — so this harness runs the clean-mask case first). It prints a
// diagnosis and returns 0; the upright + bounded-error ASSERTION is added in M15 Step 2 once a
// lever closes the gap. (No add_test yet — run the binary directly for the Step 0 readout.)
#include <array>
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "kontrolem_locomotion/trot_gait.hpp"
#include "kontrolem_control/problem.hpp"
#include "kontrolem_controllers/wbc_controller.hpp"
#include "kontrolem_estimation/invariant_estimator.hpp"
#include "kontrolem_model/robot_model.hpp"

using kontrolem_control::GaitPlan;
using kontrolem_control::Locomotion;
using kontrolem_control::State;
using kontrolem_estimation::InvariantEstimator;
using kontrolem_estimation::InvariantEstimatorConfig;
using kontrolem_locomotion::TrotGait;
using kontrolem_model::BaseType;
using kontrolem_model::RobotModel;
using kontrolem_controllers::WbcController;

namespace
{
double ori_error(const Eigen::Matrix3d & Ra, const Eigen::Matrix3d & Rb)
{
  const double c = ((Ra.transpose() * Rb).trace() - 1.0) / 2.0;
  return std::acos(std::max(-1.0, std::min(1.0, c)));
}

/// Result of one closed-on-estimate trot trial.
struct Trial
{
  bool finite = true;
  bool tipped = false;
  double tip_t = -1.0;          ///< time of first tip (tilt>0.4 or z<0.15), or -1
  double forward = 0.0;         ///< net true-base forward progress (m)
  double max_tilt = 0.0;        ///< max true-base rpy error (rad)
  double min_z = 1e9;
  // Estimate-vs-truth error, split by gait phase.
  double e_ori_2 = 0.0, e_vel_2 = 0.0, e_pos_2 = 0.0;   // during two-diagonal-foot swing
  double e_ori_4 = 0.0, e_vel_4 = 0.0, e_pos_4 = 0.0;   // during all-four-stance bracket
  // Estimate error snapshot at the moment of the tip (or final tick if it never tipped).
  double tip_e_ori = 0.0, tip_e_vel = 0.0, tip_e_pos = 0.0;
};

/// One closed-loop trot with the WBC base fed from an InvariantEstimator.
///   use_flicker=false : the estimator sees the CLEAN planned stance mask (mirrors Gazebo's
///                       planned-contact-ON config — isolates the estimator dynamics from flicker).
///   use_flicker=true  : a swing foot is occasionally, in latching bursts, mis-reported as planted
///                       (the raw-sensor failure mode) — to test whether flicker is the driver.
Trial run(
  const RobotModel & m, const std::vector<std::string> & feet,
  const std::vector<std::string> & actuated, const std::vector<int> & act_v,
  const std::vector<int> & jq, const std::vector<int> & jv, const Eigen::VectorXd & q_stand,
  const std::vector<Eigen::Vector3d> & foot0, TrotGait & gait, Locomotion & loco,
  WbcController & wbc, const InvariantEstimatorConfig & ecfg, int n_cycles, double dt,
  bool use_flicker, unsigned seed, const Eigen::Vector3d & gyro_bias,
  const Eigen::Vector3d & acc_bias)
{
  const int nv = m.nv();
  const double g = ecfg.gravity;
  Trial R;

  auto plant_ws = m.make_workspace();
  const double kp_b = 400.0, kd_b = 40.0;   // plant contact-anchor Baumgarte (as the crawl/trot gate)

  Eigen::VectorXd q = q_stand, v = Eigen::VectorXd::Zero(nv);
  std::vector<Eigen::Vector3d> anchors = foot0;
  std::vector<uint8_t> prev_stance(4, 1);
  GaitPlan plan; plan.resize(4, m.nq(), nv);

  // Estimator, seeded at the true standing state.
  const int nj = static_cast<int>(actuated.size());
  Eigen::VectorXd qj0(nj);
  for (int k = 0; k < nj; ++k) qj0[k] = q_stand[jq[k]];
  InvariantEstimator est(m, ecfg);
  est.seed(q_stand.head<3>(), Eigen::Matrix3d::Identity(), qj0, std::vector<uint8_t>(4, 1));

  std::mt19937 rng(seed);
  std::normal_distribution<double> gyro_n(0.0, 0.001), acc_n(0.0, 0.05), enc_n(0.0, 0.0005);
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  int burst[4] = {0, 0, 0, 0};

  Eigen::Vector3d vw_prev = Eigen::Vector3d::Zero();
  const double base_x_start = q(0);
  const int steps = static_cast<int>((gait.start_delay + n_cycles * gait.period) / dt);
  double t = 0.0;
  for (int i = 0; i < steps; ++i, t += dt) {
    // --- control from the ESTIMATE: true joints, estimated base pose+twist -------------------
    Eigen::VectorXd q_est = q, v_est = v;
    q_est.head<3>() = est.position();
    const Eigen::Quaterniond quat = est.orientation();
    q_est[3] = quat.x(); q_est[4] = quat.y(); q_est[5] = quat.z(); q_est[6] = quat.w();
    v_est.head<3>() = est.velocity_body();          // Pinocchio free-flyer: body-frame lin vel
    v_est.segment<3>(3) = est.angular_body();        // body-frame angular vel (last gyro)
    State st{q_est, v_est, t};
    const auto & cmd = wbc.compute(st, loco, dt);

    Eigen::VectorXd tau_gen = Eigen::VectorXd::Zero(nv);
    for (std::size_t k = 0; k < act_v.size(); ++k)
      tau_gen(act_v[k]) = cmd.tau(static_cast<Eigen::Index>(k));

    // --- gait plan + stance bookkeeping (true anchors re-set on touchdown) --------------------
    gait.sample(t, plan);
    std::vector<std::string> stance_feet;
    std::vector<Eigen::Vector3d> stance_anchors;
    int n_stance = 0;
    for (std::size_t f = 0; f < feet.size(); ++f) {
      if (plan.stance[f] && !prev_stance[f]) anchors[f] = m.frame_position(q, feet[f]);
      if (plan.stance[f]) { stance_feet.push_back(feet[f]); stance_anchors.push_back(anchors[f]); ++n_stance; }
      prev_stance[f] = plan.stance[f];
    }

    // --- step the TRUE plant under the estimate-fed torques -----------------------------------
    const Eigen::VectorXd a =
      m.contact_forward_dynamics(plant_ws, q, v, tau_gen, stance_feet, stance_anchors, kp_b, kd_b);
    if (!a.allFinite()) { R.finite = false; break; }
    v += a * dt;
    q = m.integrate(q, v, dt);

    // --- synthesize IMU + encoders from the NEW true state, then run the estimator ------------
    const Eigen::Quaterniond quat_true(q[6], q[3], q[4], q[5]);
    const Eigen::Matrix3d Rt = quat_true.toRotationMatrix();
    const Eigen::Vector3d vw = Rt * v.head<3>();                    // world-frame base lin vel
    const Eigen::Vector3d a_world = (vw - vw_prev) / dt; vw_prev = vw;
    const Eigen::Vector3d accel_body = Rt.transpose() * (a_world + Eigen::Vector3d(0, 0, g));
    // Constant IMU biases (gyro_bias, acc_bias) model the real-sensor offset the v1 InEKF does
    // NOT estimate. With the in-loop gravity aid off, gyro bias integrates straight into the
    // estimated orientation with no attitude correction — the +6-state lever's target.
    const Eigen::Vector3d gyro = v.segment<3>(3) + gyro_bias +
      Eigen::Vector3d(gyro_n(rng), gyro_n(rng), gyro_n(rng));
    const Eigen::Vector3d accel =
      accel_body + acc_bias + Eigen::Vector3d(acc_n(rng), acc_n(rng), acc_n(rng));

    Eigen::VectorXd qenc(nj), vjoint(nj);
    for (int k = 0; k < nj; ++k) { qenc[k] = q[jq[k]] + enc_n(rng); vjoint[k] = v[jv[k]]; }

    std::vector<uint8_t> rep(4);
    for (int f = 0; f < 4; ++f) {
      rep[f] = plan.stance[f];
      if (use_flicker) {
        if (burst[f] > 0) { rep[f] = 1; --burst[f]; }
        else if (!plan.stance[f] && uni(rng) < 0.02) burst[f] = 25 + int(uni(rng) * 25);
      }
    }

    est.predict(gyro, accel, dt);
    est.correct(qenc, vjoint, rep);

    // --- metrics: TRUE base health + estimate error, tagged by phase --------------------------
    const double tilt = m.difference(q_stand, q).segment<3>(3).norm();
    R.max_tilt = std::max(R.max_tilt, tilt);
    R.min_z = std::min(R.min_z, q(2));

    const double e_ori = ori_error(est.orientation().toRotationMatrix(), Rt);
    const double e_vel = (est.velocity_world() - vw).norm();
    const double e_pos = (est.position() - q.head<3>()).norm();
    if (n_stance <= 2) {
      R.e_ori_2 = std::max(R.e_ori_2, e_ori); R.e_vel_2 = std::max(R.e_vel_2, e_vel);
      R.e_pos_2 = std::max(R.e_pos_2, e_pos);
    } else {
      R.e_ori_4 = std::max(R.e_ori_4, e_ori); R.e_vel_4 = std::max(R.e_vel_4, e_vel);
      R.e_pos_4 = std::max(R.e_pos_4, e_pos);
    }
    if (!R.tipped && (q(2) < 0.15 || tilt > 0.4)) {
      R.tipped = true; R.tip_t = t;
      R.tip_e_ori = e_ori; R.tip_e_vel = e_vel; R.tip_e_pos = e_pos;
    }
    if (!R.tipped) { R.tip_e_ori = e_ori; R.tip_e_vel = e_vel; R.tip_e_pos = e_pos; }  // else: last pre-tip
  }
  R.forward = q(0) - base_x_start;
  return R;
}
}  // namespace

int main()
{
  const RobotModel m = RobotModel::from_urdf_file(FLOATING_QUAD_URDF, BaseType::kFloating);
  const std::vector<std::string> feet = {"foot_FL", "foot_FR", "foot_RL", "foot_RR"};
  const std::vector<std::string> legs = {"FL", "FR", "RL", "RR"};
  std::vector<std::string> actuated;
  for (const auto & leg : legs) {
    actuated.push_back("hipx_" + leg);
    actuated.push_back("hipy_" + leg);
    actuated.push_back("knee_" + leg);
  }

  Eigen::VectorXd q_stand = m.neutral();
  for (const auto & leg : legs) {
    q_stand(m.joint_q_index("hipy_" + leg)) = 0.7;
    q_stand(m.joint_q_index("knee_" + leg)) = -1.4;
  }
  double min_fz = 1e9;
  for (const auto & f : feet) min_fz = std::min(min_fz, m.frame_position(q_stand, f).z());
  q_stand(2) = -min_fz;
  std::vector<Eigen::Vector3d> foot0;
  for (const auto & f : feet) foot0.push_back(m.frame_position(q_stand, f));

  std::vector<int> act_v, jq, jv;
  for (const auto & a : actuated) {
    act_v.push_back(m.joint_v_index(a));
    jq.push_back(m.joint_q_index(a));
    jv.push_back(m.joint_v_index(a));
  }

  // Same quasi-static trot as test_wbc_trot (the ground-truth gate); a longer run (3 cycles) to
  // give any estimate drift time to grow.
  TrotGait gait;
  gait.q_nominal = q_stand;
  gait.foot_nominal = foot0;
  gait.swing_pair = {{0, 1, 1, 0}};
  gait.period = 1.0;
  gait.duty = 0.5;
  gait.step_len = 0.06;
  gait.step_h = 0.05;
  gait.start_delay = 1.5;
  const int n_cycles = 3;
  const double dt = 0.002;

  Locomotion loco;
  loco.gait = &gait;

  WbcController::Gains gains;
  gains.kp_post = 0.0;                       // identical WBC config to the ground-truth trot gate
  WbcController wbc(feet, actuated, gains);
  const auto syn = wbc.synthesize(m, loco);
  wbc.configure(m, *syn, loco);

  // Estimator config: the M14 in-loop tuning — gravity aid OFF (fights walking accels, M11
  // finding 1), responsive velocity (WBC is sensitive to the base-velocity estimate).
  InvariantEstimatorConfig ecfg;
  ecfg.contact_frames = feet;
  ecfg.actuated_joints = actuated;
  ecfg.gravity = 9.81;
  ecfg.sigma_grav = 1e6;
  ecfg.sigma_accel = 0.5;
  ecfg.sigma_fk = 0.02;
  ecfg.sigma_contact = 0.001;

  std::cout << "=== M15 Step 0: WBC trot closed on the InEKF estimate (diagnosis) ===\n";
  std::cout << "(ground-truth trot is solid; this closes the loop on the estimate instead)\n\n";

  const unsigned seeds[] = {1, 2, 3, 4, 5};
  const Eigen::Vector3d z3 = Eigen::Vector3d::Zero();

  struct Scenario { const char * name; bool flicker; Eigen::Vector3d gbias; Eigen::Vector3d abias; };
  // Probe order: (1) baselines that were already solid; (2) a swept GYRO bias (the leading
  // hypothesis — uncorrected because the gravity aid is off in-loop); (3) an ACCEL bias contrast.
  const std::vector<Scenario> scen = {
    {"PLANNED clean, no bias",              false, z3,                        z3},
    {"SENSED+flicker, no bias",             true,  z3,                        z3},
    {"gyro bias 0.005 rad/s (roll axis)",   false, Eigen::Vector3d(0.005, 0, 0), z3},
    {"gyro bias 0.01  rad/s (roll axis)",   false, Eigen::Vector3d(0.01, 0, 0),  z3},
    {"gyro bias 0.02  rad/s (roll axis)",   false, Eigen::Vector3d(0.02, 0, 0),  z3},
    {"gyro bias 0.01  rad/s (pitch axis)",  false, Eigen::Vector3d(0, 0.01, 0),  z3},
    {"accel bias 0.3 m/s^2 (x)",            false, z3,                        Eigen::Vector3d(0.3, 0, 0)},
  };

  for (const auto & sc : scen) {
    std::cout << "--- " << sc.name << " ---\n";
    int upright = 0;
    for (unsigned s : seeds) {
      Trial r = run(m, feet, actuated, act_v, jq, jv, q_stand, foot0, gait, loco, wbc, ecfg,
                    n_cycles, dt, sc.flicker, s, sc.gbias, sc.abias);
      const bool ok = r.finite && !r.tipped && r.forward > 0.03;
      upright += ok ? 1 : 0;
      std::cout << "  seed " << s << ": " << (ok ? "UPRIGHT" : "TIPPED ")
                << " fwd=" << r.forward << " min_z=" << r.min_z << " max_tilt=" << r.max_tilt;
      if (r.tipped) std::cout << " tip@t=" << r.tip_t
                              << " (e_ori=" << r.tip_e_ori * 180 / M_PI << "deg e_vel=" << r.tip_e_vel
                              << " e_pos=" << r.tip_e_pos << ")";
      std::cout << "\n";
      std::cout << "        est-err by phase  2-foot[ori=" << r.e_ori_2 * 180 / M_PI
                << "deg vel=" << r.e_vel_2 << " pos=" << r.e_pos_2 << "]  4-foot[ori="
                << r.e_ori_4 * 180 / M_PI << "deg vel=" << r.e_vel_4 << " pos=" << r.e_pos_4 << "]\n";
    }
    std::cout << "  => upright " << upright << "/5\n\n";
  }
  std::cout << "Step 0 is diagnostic (report-only); the asserting gate lands in M15 Step 2.\n";
  return 0;   // report-only: this step characterizes the frontier, it does not yet gate.
}
