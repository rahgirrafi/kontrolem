// WBC commanded-posture tracking (M9), proven OFFLINE before any ROS wiring. The
// WbcController now accepts the Tracking dialect: a BasePoseReference commands the base
// to squat / sway / tilt / yaw about the standing posture, and the WBC must move the
// body to follow it while the feet stay PLANTED — all six residual base DoF exercised at
// once. The plant is the model's own contact-constrained forward dynamics (feet pinned =
// the "ground"), exactly as test_wbc_standing. No ROS.
#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "kontrolem_control/base_reference.hpp"
#include "kontrolem_control/problem.hpp"
#include "kontrolem_controllers/wbc_controller.hpp"
#include "kontrolem_model/robot_model.hpp"

using kontrolem_control::BasePoseReference;
using kontrolem_control::State;
using kontrolem_control::Tracking;
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

  // Standing posture: bent legs, base lifted so the lowest foot rests at z = 0.
  Eigen::VectorXd q_stand = m.neutral();
  for (const auto & leg : legs) {
    q_stand(m.joint_q_index("hipx_" + leg)) = 0.0;
    q_stand(m.joint_q_index("hipy_" + leg)) = 0.7;
    q_stand(m.joint_q_index("knee_" + leg)) = -1.4;
  }
  double min_fz = 1e9;
  for (const auto & f : feet) min_fz = std::min(min_fz, m.frame_position(q_stand, f).z());
  q_stand(2) = -min_fz;
  std::vector<Eigen::Vector3d> anchors;
  for (const auto & f : feet) anchors.push_back(m.frame_position(q_stand, f));

  std::vector<int> act_v;
  for (const auto & a : actuated) act_v.push_back(m.joint_v_index(a));

  // Commanded base motion: all six axes at once, modest amplitude, slow & deliberate
  // (feet stay in workspace; trunk translation stays inside the friction-cone authority).
  // [x, y, z, roll, pitch, yaw].
  BasePoseReference ref;
  ref.q_nominal = q_stand;
  ref.amp   = {{0.010, 0.012, 0.015, 0.06, 0.05, 0.08}};  // squat gentle (vertical CoM is the
  ref.omega = {{0.25,  0.20,  0.25,  0.30, 0.35, 0.25}};  // hard axis); rotations are the star
  ref.phase = {{2.5,   0.5,   0.0,   1.0,  1.5,  2.0}};
  Tracking trk;
  trk.reference = &ref;

  WbcController::Gains gains;  // defaults: kp_base 100, kd_base 20, mu 0.7, tau_max 40
  // Posture is a pure DAMPING regularizer during base tracking: pulling the joints toward
  // the standing nominal (kp_post>0) fights the leg articulation the commanded base motion
  // needs (e.g. a squat must bend the knees). kd_post keeps the null space bounded.
  gains.kp_post = 0.0;
  WbcController wbc(feet, actuated, gains);
  const auto syn = wbc.synthesize(m, trk);
  wbc.configure(m, *syn, trk);

  auto plant_ws = m.make_workspace();
  const double dt = 0.002;
  const double kp_b = 400.0, kd_b = 40.0;  // plant Baumgarte (critically damped)

  Eigen::VectorXd q = q_stand, v = Eigen::VectorXd::Zero(nv);
  Eigen::VectorXd qr(m.nq()), vr(nv), ar(nv), tr(0);
  bool finite = true, torque_ok = true;
  double max_pos_err = 0.0, max_ori_err = 0.0, max_foot_drift = 0.0;
  std::array<double, 3> perr_axis{{0, 0, 0}};
  double ref_pos_excursion = 0.0, ref_ori_excursion = 0.0, act_pos_excursion = 0.0;
  int n_not_ok = 0;
  double t = 0.0;
  const int steps = 13000;  // 26 s (captures a full cycle of the slow axes)
  for (int i = 0; i < steps; ++i, t += dt) {
    State st{q, v, t};
    const auto & cmd = wbc.compute(st, trk, dt);
    if (!wbc.status().ok) ++n_not_ok;
    if (cmd.tau.cwiseAbs().maxCoeff() > gains.tau_max + 1e-6) torque_ok = false;

    Eigen::VectorXd tau_gen = Eigen::VectorXd::Zero(nv);
    for (std::size_t k = 0; k < act_v.size(); ++k) {
      tau_gen(act_v[k]) = cmd.tau(static_cast<Eigen::Index>(k));
    }
    const Eigen::VectorXd a =
      m.contact_forward_dynamics(plant_ws, q, v, tau_gen, feet, anchors, kp_b, kd_b);
    if (!a.allFinite()) { finite = false; break; }
    v += a * dt;
    q = m.integrate(q, v, dt);

    // Tracking error vs the commanded reference (skip a brief settle): unambiguous
    // world-frame base position distance + quaternion angle (no translation/rotation
    // coupling of the SE(3) log).
    ref.sample(t, qr, vr, ar, tr);
    if (t > 0.3) {
      max_pos_err = std::max(max_pos_err, (q.head<3>() - qr.head<3>()).norm());
      for (int a = 0; a < 3; ++a) perr_axis[a] = std::max(perr_axis[a], std::abs(q(a) - qr(a)));
      const Eigen::Quaterniond qq(q(6), q(3), q(4), q(5)), qrq(qr(6), qr(3), qr(4), qr(5));
      max_ori_err = std::max(max_ori_err, qq.angularDistance(qrq));
    }
    // Feet must stay planted, and the reference/robot must actually MOVE (non-trivial).
    for (int c = 0; c < static_cast<int>(feet.size()); ++c) {
      max_foot_drift = std::max(max_foot_drift, (m.frame_position(q, feet[c]) - anchors[c]).norm());
    }
    const Eigen::VectorXd eref = m.difference(q_stand, qr);
    ref_pos_excursion = std::max(ref_pos_excursion, eref.head<3>().norm());
    ref_ori_excursion = std::max(ref_ori_excursion, eref.segment<3>(3).norm());
    act_pos_excursion = std::max(act_pos_excursion, m.difference(q_stand, q).head<3>().norm());
  }

  std::cout << "reference excursion: pos = " << ref_pos_excursion << " m, ori = "
            << ref_ori_excursion * 180.0 / M_PI << " deg\n";
  std::cout << "robot pos excursion = " << act_pos_excursion << " m\n";
  std::cout << "tracking error (max): pos = " << max_pos_err << " m, ori = "
            << max_ori_err * 180.0 / M_PI << " deg\n";
  std::cout << "  per-axis pos err: dx = " << perr_axis[0] << " dy = " << perr_axis[1]
            << " dz = " << perr_axis[2] << "\n";
  std::cout << "foot drift (max) = " << max_foot_drift << ", not-ok ticks = " << n_not_ok
            << " (of " << steps << ")\n";

  // Body really moved AND reached the commanded amplitude (posture task not fighting it).
  const bool moved = ref_pos_excursion > 0.02 && ref_ori_excursion > 0.05 &&
                     act_pos_excursion > 0.85 * ref_pos_excursion;
  const bool track_ok = max_pos_err < 0.012 && max_ori_err < 0.03;  // < 1.2 cm / ~1.7 deg
  const bool contact_ok = max_foot_drift < 5e-3;                // feet stayed planted
  const bool feas_ok = n_not_ok <= 20;                          // QP feasible through the motion
  const bool ok = finite && moved && track_ok && contact_ok && feas_ok && torque_ok;
  std::cout << (ok ? "PASS" : "FAIL") << "  (finite=" << finite << " moved=" << moved
            << " track=" << track_ok << " contact=" << contact_ok << " feasible=" << feas_ok
            << " torque=" << torque_ok << ")\n";
  return ok ? 0 : 1;
}
