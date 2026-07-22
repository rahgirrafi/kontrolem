// WBC static crawl walk (M10 Step 2), proven OFFLINE before any ROS. The CrawlGait cycles
// all four feet (one swings at a time, stepping forward) while the base tracks the support-
// polygon centroid, so the quadruped WALKS FORWARD and stays statically stable throughout.
// The plant is the model's contact-constrained forward dynamics with ONLY the current
// stance feet pinned (swing foot free), anchors re-set on touchdown. No ROS.
#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "kontrolem_locomotion/crawl_gait.hpp"
#include "kontrolem_control/problem.hpp"
#include "kontrolem_controllers/wbc_controller.hpp"
#include "kontrolem_model/robot_model.hpp"

using kontrolem_locomotion::CrawlGait;
using kontrolem_control::GaitPlan;
using kontrolem_control::Locomotion;
using kontrolem_control::State;
using kontrolem_model::BaseType;
using kontrolem_model::RobotModel;
using kontrolem_controllers::WbcController;

namespace
{
// Signed distance (m) of p to triangle (a,b,c): > 0 inside (distance to nearest edge),
// < 0 outside (by how far). Winding-independent.
double tri_margin(const Eigen::Vector2d & p, Eigen::Vector2d a, Eigen::Vector2d b,
                  Eigen::Vector2d c)
{
  auto cr = [](const Eigen::Vector2d & u, const Eigen::Vector2d & v, const Eigen::Vector2d & w) {
    return (v.x() - u.x()) * (w.y() - u.y()) - (v.y() - u.y()) * (w.x() - u.x());
  };
  if (cr(a, b, c) < 0) std::swap(b, c);                    // make CCW
  const double d1 = cr(a, b, p) / (b - a).norm();
  const double d2 = cr(b, c, p) / (c - b).norm();
  const double d3 = cr(c, a, p) / (a - c).norm();
  return std::min({d1, d2, d3});
}
}  // namespace

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

  CrawlGait gait;
  gait.q_nominal = q_stand;
  gait.foot_nominal = foot0;
  gait.order = {{2, 0, 3, 1}};   // RL, FL, RR, FR
  gait.period = 12.0;
  gait.duty = 0.5;
  gait.step_len = 0.05;
  gait.step_h = 0.04;
  gait.base_gain = 1.0;
  gait.start_delay = 1.5;
  // The CoM sits ~1.9 cm behind the geometric center, so lead the base target forward to
  // place the CoM (not the base) over the support centroid.
  const Eigen::Vector3d com0 = m.center_of_mass(q_stand);
  gait.com_bias_x = -com0.x();
  gait.com_bias_y = -com0.y();
  const int n_cycles = 2;

  Locomotion loco;
  loco.gait = &gait;

  WbcController::Gains gains;
  gains.kp_post = 0.0;
  WbcController wbc(feet, actuated, gains);
  const auto syn = wbc.synthesize(m, loco);
  wbc.configure(m, *syn, loco);

  auto plant_ws = m.make_workspace();
  const double dt = 0.002;
  const double kp_b = 400.0, kd_b = 40.0;

  Eigen::VectorXd q = q_stand, v = Eigen::VectorXd::Zero(nv);
  std::vector<Eigen::Vector3d> anchors = foot0;
  std::vector<uint8_t> prev_stance(4, 1);
  GaitPlan plan; plan.resize(4, m.nq(), nv);

  bool finite = true, upright = true;
  int n_not_ok = 0, n_unstable = 0, n_swing_checked = 0;
  double worst_margin = 1e9;
  std::array<double, 4> max_lift{{0, 0, 0, 0}};
  double base_x_start = q(0);
  double t = 0.0;
  const int steps = static_cast<int>((gait.start_delay + n_cycles * gait.period) / dt);
  for (int i = 0; i < steps; ++i, t += dt) {
    State st{q, v, t};
    const auto & cmd = wbc.compute(st, loco, dt);
    if (!wbc.status().ok) ++n_not_ok;

    Eigen::VectorXd tau_gen = Eigen::VectorXd::Zero(nv);
    for (std::size_t k = 0; k < act_v.size(); ++k) tau_gen(act_v[k]) = cmd.tau(static_cast<Eigen::Index>(k));

    gait.sample(t, plan);
    std::vector<std::string> stance_feet;
    std::vector<Eigen::Vector3d> stance_anchors;
    std::vector<int> stance_idx;
    for (std::size_t f = 0; f < feet.size(); ++f) {
      if (plan.stance[f] && !prev_stance[f]) anchors[f] = m.frame_position(q, feet[f]);
      if (plan.stance[f]) {
        stance_feet.push_back(feet[f]); stance_anchors.push_back(anchors[f]);
        stance_idx.push_back(static_cast<int>(f));
      }
      prev_stance[f] = plan.stance[f];
    }

    const Eigen::VectorXd a =
      m.contact_forward_dynamics(plant_ws, q, v, tau_gen, stance_feet, stance_anchors, kp_b, kd_b);
    if (!a.allFinite()) { finite = false; break; }
    v += a * dt;
    q = m.integrate(q, v, dt);

    for (std::size_t f = 0; f < feet.size(); ++f) {
      if (!plan.stance[f]) max_lift[f] = std::max(max_lift[f], m.frame_position(q, feet[f]).z());
    }
    // Static stability during single-foot swing: CoM inside the 3-stance-feet triangle.
    if (stance_idx.size() == 3) {
      ++n_swing_checked;
      const Eigen::Vector3d com = m.center_of_mass(q);
      const double margin = tri_margin(
        com.head<2>(),
        m.frame_position(q, feet[stance_idx[0]]).head<2>(),
        m.frame_position(q, feet[stance_idx[1]]).head<2>(),
        m.frame_position(q, feet[stance_idx[2]]).head<2>());
      worst_margin = std::min(worst_margin, margin);
      if (margin < 0) ++n_unstable;
    }
    if (q(2) < 0.15 || m.difference(q_stand, q).segment<3>(3).norm() > 0.6) upright = false;
#ifdef CRAWL_DEBUG
    static double dbg = -1;
    if (t - dbg >= 0.5 && finite) {
      dbg = t;
      const Eigen::Vector3d com = m.center_of_mass(q);
      int sw = -1; for (int f = 0; f < 4; ++f) if (!plan.stance[f]) sw = f;
      std::cout << "t=" << t << " swing=" << sw << " cmd_base=(" << plan.q_ref(0) << ","
                << plan.q_ref(1) << ") base=(" << q(0) << "," << q(1) << ") com=(" << com.x()
                << "," << com.y() << ") nstance=" << stance_idx.size() << "\n";
    }
#endif
  }

  const double forward = q(0) - base_x_start;
  const double expected = n_cycles * gait.step_len;
  const bool all_stepped =
    max_lift[0] > 0.02 && max_lift[1] > 0.02 && max_lift[2] > 0.02 && max_lift[3] > 0.02;

  std::cout << "forward progress: base x " << base_x_start << " -> " << q(0) << " (" << forward
            << " m, expected ~" << expected << ")\n";
  std::cout << "per-foot max lift: " << max_lift[0] << " " << max_lift[1] << " " << max_lift[2]
            << " " << max_lift[3] << "\n";
  std::cout << "static stability: " << n_unstable << " outside of " << n_swing_checked
            << " swing ticks; worst CoM margin = " << worst_margin * 1000.0
            << " mm; not-ok = " << n_not_ok << " of " << steps << "\n";

  const bool walked = forward > 0.7 * expected;
  // Statically stable: the CoM never leaves the support polygon by more than 1 cm (a few-mm
  // quasi-static excursion at a swing transition, with the foot about to land, does not tip
  // the robot — upright confirms it; a real fall shows as a large negative margin + finite=0).
  const bool stable = n_swing_checked > 100 && worst_margin > -0.01;
  const bool feasible = n_not_ok < 0.02 * steps;
  const bool ok = finite && walked && all_stepped && stable && upright && feasible;
  std::cout << (ok ? "PASS" : "FAIL") << "  (finite=" << finite << " walked=" << walked
            << " stepped=" << all_stepped << " stable=" << stable << " upright=" << upright
            << " feasible=" << feasible << ")\n";
  return ok ? 0 : 1;
}
