// KinematicGaitController + TrotGait, proven OFFLINE before any ROS (M13 Step 3). This is the
// model-FREE walking controller: each tick it samples the trot gait, solves a per-leg IK for the
// joint angles that place each foot on its world target, and emits a joint PD torque. There is no
// dynamics/QP and no estimator (the IK base is the scheduled nominal pose). The "plant" here is a
// perfect-tracking kinematic rollout (the joints follow the IK target with a one-tick lag), which
// is enough to prove the three things the walk rests on:
//   (a) TrotGait diagonal-pair timing is correct (never >2 feet swing; {FL,RR} then {FR,RL});
//   (b) the IK converges — each foot lands within 1 mm of its gait target, through the swing arc;
//   (c) the PD torques stay bounded (<= tau_max) and continuous (no per-tick glitches).
#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "kontrolem_control/problem.hpp"
#include "kontrolem_controllers/kinematic_gait_controller.hpp"
#include "kontrolem_locomotion/trot_gait.hpp"
#include "kontrolem_model/robot_model.hpp"

using kontrolem_control::GaitPlan;
using kontrolem_control::Locomotion;
using kontrolem_control::State;
using kontrolem_controllers::KinematicGaitController;
using kontrolem_locomotion::TrotGait;
using kontrolem_model::BaseType;
using kontrolem_model::RobotModel;

int main()
{
  const RobotModel m = RobotModel::from_urdf_file(FLOATING_QUAD_URDF, BaseType::kFloating);
  const int nv = m.nv();
  const std::vector<std::string> feet = {"foot_FL", "foot_FR", "foot_RL", "foot_RR"};
  const std::vector<std::string> legs = {"FL", "FR", "RL", "RR"};
  std::vector<std::string> actuated;
  for (const auto & leg : legs) {
    actuated.push_back("hipx_" + leg);
    actuated.push_back("hipy_" + leg);
    actuated.push_back("knee_" + leg);
  }

  // Nominal stand (same posture the WBC crawl test uses), base height so the lowest foot is at 0.
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

  TrotGait gait;
  gait.q_nominal = q_stand;
  gait.foot_nominal = foot0;
  gait.swing_pair = {{0, 1, 1, 0}};  // {FL,RR} pair 0 ; {FR,RL} pair 1
  gait.period = 1.0;
  gait.duty = 0.5;
  gait.step_len = 0.06;
  gait.step_h = 0.05;
  gait.start_delay = 0.5;

  Locomotion loco;
  loco.gait = &gait;

  // ---- (a) TrotGait diagonal-pair timing (pure gait, no controller) --------------------------
  // half=0.5, d=duty*half=0.25, margin=0.125 -> pair-0 swings [0.125,0.375], pair-1 [0.625,0.875]
  // within each stride; the rest is all-stance (double support). Feet: FL=0, FR=1, RL=2, RR=3.
  GaitPlan gp;
  gp.resize(4, m.nq(), nv);
  bool timing_ok = true;
  int max_concurrent_swing = 0;
  auto stance_at = [&](double tg_in_stride, int f) {
    gait.sample(gait.start_delay + tg_in_stride, gp);
    return static_cast<int>(gp.stance[static_cast<std::size_t>(f)]);
  };
  // Mid pair-0 swing (tg=0.25): FL(0),RR(3) swing; FR(1),RL(2) stance.
  timing_ok &= (stance_at(0.25, 0) == 0 && stance_at(0.25, 3) == 0);
  timing_ok &= (stance_at(0.25, 1) == 1 && stance_at(0.25, 2) == 1);
  // Mid pair-1 swing (tg=0.75): FR(1),RL(2) swing; FL(0),RR(3) stance.
  timing_ok &= (stance_at(0.75, 1) == 0 && stance_at(0.75, 2) == 0);
  timing_ok &= (stance_at(0.75, 0) == 1 && stance_at(0.75, 3) == 1);
  // All-stance margin (tg=0.5): every foot down.
  for (int f = 0; f < 4; ++f) timing_ok &= (stance_at(0.5, f) == 1);
  // Sweep a full stride: at most two feet ever swing at once, and diagonal pairs move together.
  for (int k = 0; k <= 200; ++k) {
    const double tg = k / 200.0;  // [0,1]
    gait.sample(gait.start_delay + tg, gp);
    const int nsw = gp.stance[0] + gp.stance[1] + gp.stance[2] + gp.stance[3];  // #stance
    const int swing = 4 - nsw;
    max_concurrent_swing = std::max(max_concurrent_swing, swing);
    // diagonal integrity: FL and RR share a state; FR and RL share a state.
    if (gp.stance[0] != gp.stance[3]) timing_ok = false;
    if (gp.stance[1] != gp.stance[2]) timing_ok = false;
  }
  if (max_concurrent_swing > 2) timing_ok = false;

  // ---- (b)(c) controller: IK convergence + torque bounds/continuity --------------------------
  KinematicGaitController::Gains gains;  // defaults: kp 60, kd 2, tau_max 23.7
  KinematicGaitController ctrl(feet, actuated, gains);
  const auto syn = ctrl.synthesize(m, loco);
  ctrl.configure(m, *syn, loco);

  const double dt = 0.002;  // 500 Hz
  const int n_strides = 2;
  const int steps = static_cast<int>((gait.start_delay + n_strides * gait.period) / dt);

  Eigen::VectorXd q = q_stand, v = Eigen::VectorXd::Zero(nv);
  Eigen::VectorXd tau_prev;
  double worst_foot_err = 0.0;   // (b) max foot-to-target distance after IK (m)
  double max_tau = 0.0;          // (c) peak |torque|
  double max_dtau = 0.0;         // (c) peak per-tick torque change (continuity)
  int n_not_converged = 0;
  bool finite = true;
  double t = 0.0;

  for (int i = 0; i < steps; ++i, t += dt) {
    State st{q, v, t};
    const auto & cmd = ctrl.compute(st, loco, dt);
    if (!cmd.tau.allFinite()) { finite = false; break; }
    if (!ctrl.status().ok) ++n_not_converged;

    // (b) The IK target must place every foot on its gait target (<1 mm), through the swing arc.
    const Eigen::VectorXd & q_tgt = ctrl.target_configuration();
    gait.sample(t, gp);
    for (std::size_t f = 0; f < feet.size(); ++f) {
      const double e = (m.frame_position(q_tgt, feet[f]) - gp.swing_pos[f]).norm();
      worst_foot_err = std::max(worst_foot_err, e);
    }
    // (c) torque bounds + continuity.
    max_tau = std::max(max_tau, cmd.tau.cwiseAbs().maxCoeff());
    if (tau_prev.size() == cmd.tau.size()) {
      max_dtau = std::max(max_dtau, (cmd.tau - tau_prev).cwiseAbs().maxCoeff());
    }
    tau_prev = cmd.tau;

    // Perfect-tracking kinematic plant: the joints follow the IK target with a one-tick lag.
    v = m.difference(q, q_tgt) / dt;
    q = q_tgt;
  }

  std::cout << "(a) timing: diagonal pairs move together, max concurrent swing = "
            << max_concurrent_swing << " (<=2), timing_ok=" << timing_ok << "\n";
  std::cout << "(b) IK: worst foot-to-target = " << worst_foot_err * 1000.0
            << " mm; not-converged ticks = " << n_not_converged << " of " << steps << "\n";
  std::cout << "(c) torque: peak |tau| = " << max_tau << " Nm (limit " << gains.tau_max
            << "); peak per-tick d|tau| = " << max_dtau << " Nm\n";

  const bool timing = timing_ok && max_concurrent_swing == 2;
  const bool ik_ok = finite && worst_foot_err < 1e-3 && n_not_converged == 0;
  const bool torque_ok = finite && max_tau <= gains.tau_max + 1e-6 && max_dtau < 5.0;
  const bool ok = timing && ik_ok && torque_ok;
  std::cout << (ok ? "PASS" : "FAIL") << "  (timing=" << timing << " ik=" << ik_ok
            << " torque=" << torque_ok << " finite=" << finite << ")\n";
  return ok ? 0 : 1;
}
