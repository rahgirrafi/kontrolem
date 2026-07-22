// WBC dynamic diagonal TROT (M14 Step 2) — the OFFLINE feasibility gate. Same Go2-shaped quad and
// the SAME WbcController as the static crawl (test_wbc_crawl), but driven by a TrotGait instead of a
// CrawlGait: two DIAGONAL pairs of feet ({FL,RR} then {FR,RL}) swing 180° out of phase, so during a
// swing the robot balances on just TWO diagonal feet — a support LINE, not a triangle. That is the
// medium-risk knife-edge M14 must clear before touching Gazebo: does the whole-body QP stay feasible
// and keep the trunk upright while it trots forward on alternating diagonals? The plant is the
// model's contact-constrained forward dynamics with ONLY the current stance feet pinned (swing feet
// free), anchors re-set on touchdown. No ROS. This is M10's crawl→trot swap, proven offline.
#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "kontrolem_locomotion/trot_gait.hpp"
#include "kontrolem_control/problem.hpp"
#include "kontrolem_controllers/wbc_controller.hpp"
#include "kontrolem_model/robot_model.hpp"

using kontrolem_locomotion::TrotGait;
using kontrolem_control::GaitPlan;
using kontrolem_control::Locomotion;
using kontrolem_control::State;
using kontrolem_model::BaseType;
using kontrolem_model::RobotModel;
using kontrolem_controllers::WbcController;

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

  std::vector<int> act_v;
  for (const auto & a : actuated) act_v.push_back(m.joint_v_index(a));

  // Quasi-static trot: duty 0.5 leaves an all-four-stance bracket around each diagonal's swing, so
  // there is never a flight phase (always >=2 feet down) — the safe starting point (M14 Step 3).
  TrotGait gait;
  gait.q_nominal = q_stand;
  gait.foot_nominal = foot0;
  gait.swing_pair = {{0, 1, 1, 0}};   // {FL,RR}=0, {FR,RL}=1
  gait.period = 1.0;
  gait.duty = 0.5;
  gait.step_len = 0.06;
  gait.step_h = 0.05;
  gait.start_delay = 1.5;
  const int n_cycles = 2;

  Locomotion loco;
  loco.gait = &gait;

  WbcController::Gains gains;
  gains.kp_post = 0.0;    // swing legs free (driven by the swing task); base task holds the trunk
  WbcController wbc(feet, actuated, gains);
  const auto syn = wbc.synthesize(m, loco);
  wbc.configure(m, *syn, loco);

  auto plant_ws = m.make_workspace();
  const double dt = 0.002;
  const double kp_b = 400.0, kd_b = 40.0;   // plant contact-anchor Baumgarte (as the crawl gate)

  Eigen::VectorXd q = q_stand, v = Eigen::VectorXd::Zero(nv);
  std::vector<Eigen::Vector3d> anchors = foot0;
  std::vector<uint8_t> prev_stance(4, 1);
  GaitPlan plan; plan.resize(4, m.nq(), nv);

  bool finite = true, upright = true;
  int n_not_ok = 0, n_two_foot = 0, n_swing_ticks = 0;
  double max_tau = 0.0, max_tilt = 0.0, min_z = 1e9;
  std::array<double, 4> max_lift{{0, 0, 0, 0}};
  const double base_x_start = q(0);
  double t = 0.0;
  const int steps = static_cast<int>((gait.start_delay + n_cycles * gait.period) / dt);
  for (int i = 0; i < steps; ++i, t += dt) {
    State st{q, v, t};
    const auto & cmd = wbc.compute(st, loco, dt);
    if (!wbc.status().ok) ++n_not_ok;
    for (int k = 0; k < cmd.tau.size(); ++k) max_tau = std::max(max_tau, std::abs(cmd.tau(k)));

    Eigen::VectorXd tau_gen = Eigen::VectorXd::Zero(nv);
    for (std::size_t k = 0; k < act_v.size(); ++k)
      tau_gen(act_v[k]) = cmd.tau(static_cast<Eigen::Index>(k));

    gait.sample(t, plan);
    std::vector<std::string> stance_feet;
    std::vector<Eigen::Vector3d> stance_anchors;
    int n_stance = 0;
    for (std::size_t f = 0; f < feet.size(); ++f) {
      if (plan.stance[f] && !prev_stance[f]) anchors[f] = m.frame_position(q, feet[f]);
      if (plan.stance[f]) {
        stance_feet.push_back(feet[f]); stance_anchors.push_back(anchors[f]); ++n_stance;
      }
      prev_stance[f] = plan.stance[f];
    }
    if (n_stance < 4) ++n_swing_ticks;
    if (n_stance == 2) ++n_two_foot;   // the diagonal-balance knife-edge actually occurred

    const Eigen::VectorXd a =
      m.contact_forward_dynamics(plant_ws, q, v, tau_gen, stance_feet, stance_anchors, kp_b, kd_b);
    if (!a.allFinite()) { finite = false; break; }
    v += a * dt;
    q = m.integrate(q, v, dt);

    for (std::size_t f = 0; f < feet.size(); ++f) {
      if (!plan.stance[f]) max_lift[f] = std::max(max_lift[f], m.frame_position(q, feet[f]).z());
    }
    const double tilt = m.difference(q_stand, q).segment<3>(3).norm();   // base roll/pitch/yaw error
    max_tilt = std::max(max_tilt, tilt);
    min_z = std::min(min_z, q(2));
    if (q(2) < 0.15 || tilt > 0.6) upright = false;
#ifdef TROT_DEBUG
    static double dbg = -1;
    if (t - dbg >= 0.25 && finite) {
      dbg = t;
      std::cout << "t=" << t << " nstance=" << n_stance << " base=(" << q(0) << "," << q(1) << ","
                << q(2) << ") tilt=" << tilt << " maxtau=" << max_tau << " ok="
                << wbc.status().ok << "\n";
    }
#endif
  }

  const double forward = q(0) - base_x_start;
  const double expected = n_cycles * gait.step_len;   // base advances step_len per stride
  const bool all_stepped =
    max_lift[0] > 0.02 && max_lift[1] > 0.02 && max_lift[2] > 0.02 && max_lift[3] > 0.02;

  std::cout << "forward progress: base x " << base_x_start << " -> " << q(0) << " (" << forward
            << " m, expected ~" << expected << ")\n";
  std::cout << "per-foot max lift: " << max_lift[0] << " " << max_lift[1] << " " << max_lift[2]
            << " " << max_lift[3] << "\n";
  std::cout << "two-diagonal-foot ticks: " << n_two_foot << " of " << n_swing_ticks
            << " swing ticks (" << steps << " total); QP not-ok = " << n_not_ok << "\n";
  std::cout << "max |tau| = " << max_tau << " Nm (limit " << gains.tau_max << "); max base tilt = "
            << max_tilt << " rad; min base z = " << min_z << " m\n";

  const bool walked = forward > 0.7 * expected;
  const bool feasible = n_not_ok < 0.02 * steps;
  const bool bounded_tau = max_tau <= gains.tau_max + 1e-6;
  // The trot spends real time on exactly two diagonal feet — the risk this gate exists to clear.
  const bool trotted = n_two_foot > 100;
  const bool ok =
    finite && walked && all_stepped && upright && feasible && bounded_tau && trotted;
  std::cout << (ok ? "PASS" : "FAIL") << "  (finite=" << finite << " walked=" << walked
            << " stepped=" << all_stepped << " upright=" << upright << " feasible=" << feasible
            << " bounded_tau=" << bounded_tau << " trotted=" << trotted << ")\n";
  return ok ? 0 : 1;
}
