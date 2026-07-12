// WBC single-step / swing-foot task (M10 Step 1), proven OFFLINE before any ROS. The
// WbcController now accepts the Locomotion dialect: a GaitSource commands one foot to LIFT,
// arc forward, and PLACE while the other three support and the base shifts to keep the CoM
// over their triangle. The plant is the model's contact-constrained forward dynamics with
// ONLY the current STANCE feet pinned (swing foot free), anchors re-set on touchdown — the
// same feet-planted "ground" as test_wbc_standing, now with a changing contact set. No ROS.
#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "kontrolem_control/gait.hpp"
#include "kontrolem_control/problem.hpp"
#include "kontrolem_controllers/wbc_controller.hpp"
#include "kontrolem_model/robot_model.hpp"

using kontrolem_control::GaitPlan;
using kontrolem_control::GaitSource;
using kontrolem_control::Locomotion;
using kontrolem_control::State;
using kontrolem_model::BaseType;
using kontrolem_model::RobotModel;
using kontrolem_controllers::WbcController;

namespace
{
// One foot (swing_foot) lifts, arcs forward step_len, and places, over [t0, t1]. The base
// ramps to base_shift (support-triangle centroid) over [0, t0] and holds it through the
// swing so the CoM stays statically stable. Joints held at nominal; stance feet default.
struct SingleStepGait : GaitSource
{
  Eigen::VectorXd q_nominal;
  std::vector<Eigen::Vector3d> foot0;
  int swing_foot = 0;
  double t0 = 1.5, t1 = 3.0, step_len = 0.05, step_h = 0.04;
  Eigen::Vector3d base_shift{Eigen::Vector3d::Zero()};

  void sample(double t, GaitPlan & out) const override
  {
    out.q_ref = q_nominal;
    out.v_ref.setZero();
    for (std::size_t i = 0; i < foot0.size(); ++i) {
      out.stance[i] = 1;
      out.swing_pos[i] = foot0[i];
      out.swing_vel[i].setZero();
    }
    // Base weight-shift: ramp to the support centroid by t0, then hold.
    const double a = std::min(1.0, t / t0);
    out.q_ref(0) += a * base_shift.x();
    out.q_ref(1) += a * base_shift.y();
    if (t >= t0 && t < t1) {
      out.stance[static_cast<std::size_t>(swing_foot)] = 0;
      const double s = (t - t0) / (t1 - t0);            // 0..1 swing phase
      Eigen::Vector3d p = foot0[static_cast<std::size_t>(swing_foot)];
      p.x() += step_len * s;
      p.z() += step_h * std::sin(M_PI * s);              // lift-and-place arc
      out.swing_pos[static_cast<std::size_t>(swing_foot)] = p;
      out.swing_vel[static_cast<std::size_t>(swing_foot)] =
        Eigen::Vector3d(step_len / (t1 - t0), 0.0, step_h * M_PI * std::cos(M_PI * s) / (t1 - t0));
    } else if (t >= t1) {
      // Foot has landed one step forward; keep the target there (stance).
      out.swing_pos[static_cast<std::size_t>(swing_foot)] =
        foot0[static_cast<std::size_t>(swing_foot)] + Eigen::Vector3d(step_len, 0, 0);
    }
  }
};

// Is 2D point p inside triangle (a,b,c)? (consistent sign of the three edge cross products)
bool in_triangle(const Eigen::Vector2d & p, const Eigen::Vector2d & a, const Eigen::Vector2d & b,
                 const Eigen::Vector2d & c)
{
  auto cross = [](const Eigen::Vector2d & u, const Eigen::Vector2d & v, const Eigen::Vector2d & w) {
    return (v.x() - u.x()) * (w.y() - u.y()) - (v.y() - u.y()) * (w.x() - u.x());
  };
  const double d1 = cross(a, b, p), d2 = cross(b, c, p), d3 = cross(c, a, p);
  const bool neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
  const bool pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
  return !(neg && pos);
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
    q_stand(m.joint_q_index("hipx_" + leg)) = 0.0;
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

  // Gait: FL steps; base shifts to the centroid of the other three (static support).
  SingleStepGait gait;
  gait.q_nominal = q_stand;
  gait.foot0 = foot0;
  gait.swing_foot = 0;                        // FL
  const Eigen::Vector3d support_centroid = (foot0[1] + foot0[2] + foot0[3]) / 3.0;
  gait.base_shift = Eigen::Vector3d(support_centroid.x(), support_centroid.y(), 0.0);
  const Eigen::Vector3d land_target = foot0[0] + Eigen::Vector3d(gait.step_len, 0, 0);

  Locomotion loco;
  loco.gait = &gait;

  WbcController::Gains gains;   // kp_base 100, kd_base 20, mu 0.7, kp_swing 400, kd_swing 40
  gains.kp_post = 0.0;         // posture = damping regularizer (as M9), so the swing leg is free
  WbcController wbc(feet, actuated, gains);
  const auto syn = wbc.synthesize(m, loco);
  wbc.configure(m, *syn, loco);

  auto plant_ws = m.make_workspace();
  const double dt = 0.002;
  const double kp_b = 400.0, kd_b = 40.0;

  Eigen::VectorXd q = q_stand, v = Eigen::VectorXd::Zero(nv);
  std::vector<Eigen::Vector3d> anchors = foot0;      // per-foot world anchor (touchdown)
  std::vector<uint8_t> prev_stance(4, 1);
  GaitPlan plan; plan.resize(4, m.nq(), nv);

  bool finite = true, feasible = true, upright = true, stable = true;
  double max_swing_z = 0.0, max_stance_drift = 0.0;
  int n_not_ok = 0;
  double t = 0.0;
  const int steps = 2250;                            // 4.5 s
  for (int i = 0; i < steps; ++i, t += dt) {
    State st{q, v, t};
    const auto & cmd = wbc.compute(st, loco, dt);
    if (!wbc.status().ok) { ++n_not_ok; }

    Eigen::VectorXd tau_gen = Eigen::VectorXd::Zero(nv);
    for (std::size_t k = 0; k < act_v.size(); ++k) {
      tau_gen(act_v[k]) = cmd.tau(static_cast<Eigen::Index>(k));
    }

    // The current stance set (from the gait) is what the sim pins; the swing foot is free.
    gait.sample(t, plan);
    std::vector<std::string> stance_feet;
    std::vector<Eigen::Vector3d> stance_anchors;
    for (std::size_t f = 0; f < feet.size(); ++f) {
      if (plan.stance[f] && !prev_stance[f]) anchors[f] = m.frame_position(q, feet[f]);  // touchdown
      if (plan.stance[f]) { stance_feet.push_back(feet[f]); stance_anchors.push_back(anchors[f]); }
      prev_stance[f] = plan.stance[f];
    }

    const Eigen::VectorXd a =
      m.contact_forward_dynamics(plant_ws, q, v, tau_gen, stance_feet, stance_anchors, kp_b, kd_b);
    if (!a.allFinite()) { finite = false; break; }
    v += a * dt;
    q = m.integrate(q, v, dt);

    // Swing foot lifts (mid-swing z above ground); stance feet stay planted.
    if (!plan.stance[0]) {
      max_swing_z = std::max(max_swing_z, m.frame_position(q, feet[0]).z());
    }
    for (std::size_t f = 0; f < feet.size(); ++f) {
      if (plan.stance[f]) {
        max_stance_drift = std::max(max_stance_drift, (m.frame_position(q, feet[f]) - anchors[f]).norm());
      }
    }
    // Static stability: CoM over the current support polygon (the 3 stance feet during swing).
    if (!plan.stance[0]) {
      const Eigen::Vector3d com = m.center_of_mass(q);
      const bool inside = in_triangle(
        com.head<2>(), m.frame_position(q, feet[1]).head<2>(),
        m.frame_position(q, feet[2]).head<2>(), m.frame_position(q, feet[3]).head<2>());
      if (!inside) stable = false;
    }
    if (q(2) < 0.15 || m.difference(q_stand, q).segment<3>(3).norm() > 0.5) upright = false;
  }
  if (n_not_ok > 40) feasible = false;

  const Eigen::Vector3d land = m.frame_position(q, feet[0]);
  const double land_err = (land - land_target).norm();

  std::cout << "swing foot landed at (" << land.transpose() << "), target ("
            << land_target.transpose() << "), err = " << land_err << "\n";
  std::cout << "max swing-foot lift = " << max_swing_z << " m, max stance drift = "
            << max_stance_drift << " m\n";
  std::cout << "not-ok ticks = " << n_not_ok << " (of " << steps << ")\n";

  const bool landed = land_err < 0.01;                 // foot placed within 1 cm of target
  const bool lifted = max_swing_z > 0.02;              // foot actually left the ground
  const bool planted = max_stance_drift < 5e-3;        // stance feet stayed put
  const bool ok = finite && landed && lifted && planted && stable && upright && feasible;
  std::cout << (ok ? "PASS" : "FAIL") << "  (finite=" << finite << " landed=" << landed
            << " lifted=" << lifted << " planted=" << planted << " stable=" << stable
            << " upright=" << upright << " feasible=" << feasible << ")\n";
  return ok ? 0 : 1;
}
