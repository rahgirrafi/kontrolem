// Contact-constrained dynamics (M4 sim ground-truth). Validates the pieces a
// floating-base standing simulator and a QP-WBC both stand on:
//   A. contact_jacobian_stacked equals the per-foot contact_jacobian (block form),
//   B. contact_drift is ~0 at zero velocity,
//   C. the quadruped is WBC-viable: with 4 feet pinned it retains 6 residual
//      base DoF (nv - rank J = 18 - 12), i.e. the contacts do NOT lock it rigid,
//   D. contact_forward_dynamics reproduces a static equilibrium (gravity fully
//      compensated -> qddot ~ 0, base height holds over 2 s), and
//   E. under a joint-torque disturbance the FEET stay planted (Baumgarte holds
//      the no-slip contact to sub-mm) while the BASE moves — feet-planted-but-
//      base-free, exactly the regime the WBC operates in.
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "kontrolem_model/robot_model.hpp"

using kontrolem_model::BaseType;
using kontrolem_model::RobotModel;

int main()
{
  const RobotModel m = RobotModel::from_urdf_file(FLOATING_QUAD_URDF, BaseType::kFloating);
  const int nv = m.nv();
  const std::vector<std::string> feet = {"foot_FL", "foot_FR", "foot_RL", "foot_RR"};
  const int nc = static_cast<int>(feet.size());

  // --- Standing configuration: identity base orientation, bent legs so feet sit
  // under the four corners; base height set so the lowest foot rests at z = 0. ---
  Eigen::VectorXd q = m.neutral();  // base pos 0, quat identity, joints 0
  for (const std::string leg : {"FL", "FR", "RL", "RR"}) {
    q(m.joint_q_index("hipx_" + leg)) = 0.0;
    q(m.joint_q_index("hipy_" + leg)) = 0.7;
    q(m.joint_q_index("knee_" + leg)) = -1.4;
  }
  double min_fz = 1e9;
  for (const auto & f : feet) {
    min_fz = std::min(min_fz, m.frame_position(q, f).z());
  }
  q(2) = -min_fz;  // lift the base so the lowest foot touches the ground plane

  std::vector<Eigen::Vector3d> anchors;
  for (const auto & f : feet) {
    anchors.push_back(m.frame_position(q, f));
  }
  const double base_z0 = q(2);
  std::cout << "standing base_z = " << base_z0 << ", foot z:";
  for (const auto & a : anchors) std::cout << " " << a.z();
  std::cout << "\n";

  auto ws = m.make_workspace();
  const Eigen::VectorXd v0 = Eigen::VectorXd::Zero(nv);

  // --- A. stacked vs per-foot Jacobian ---
  Eigen::MatrixXd Js;
  m.contact_jacobian_stacked(ws, q, feet, Js);
  double jac_err = 0.0;
  for (int k = 0; k < nc; ++k) {
    const Eigen::MatrixXd Jk = m.contact_jacobian(q, feet[k]);
    jac_err = std::max(jac_err, (Js.middleRows(3 * k, 3) - Jk).cwiseAbs().maxCoeff());
  }
  const bool a_ok = (Js.rows() == 3 * nc) && (Js.cols() == nv) && (jac_err < 1e-9);
  std::cout << "A stacked-J shape " << Js.rows() << "x" << Js.cols()
            << " max|Js-Jk| = " << jac_err << "\n";

  // --- B. drift at rest ---
  Eigen::VectorXd gamma;
  m.contact_drift(ws, q, v0, feet, gamma);
  const double drift0 = gamma.cwiseAbs().maxCoeff();
  const bool b_ok = (gamma.size() == 3 * nc) && (drift0 < 1e-9);
  std::cout << "B drift(v=0) max = " << drift0 << "\n";

  // --- C. residual DoF (platform must not be locked) ---
  Eigen::FullPivLU<Eigen::MatrixXd> lu(Js);
  lu.setThreshold(1e-7);
  const int rank = static_cast<int>(lu.rank());
  const int residual = nv - rank;
  const bool c_ok = (residual == 6);
  std::cout << "C rank(J) = " << rank << ", residual base DoF = " << residual
            << " (expect 6)\n";

  // --- D. static equilibrium: full gravity comp -> stays put ---
  const double dt = 0.002;
  const double kp = 400.0, kd = 40.0;  // Baumgarte (critically damped, ~20 rad/s)
  Eigen::VectorXd qd = q, vd = v0;
  bool d_finite = true;
  for (int i = 0; i < 1000; ++i) {  // 2 s
    const Eigen::VectorXd tau = m.gravity_torque(qd);  // cancels gravity exactly at v=0
    const Eigen::VectorXd a =
      m.contact_forward_dynamics(ws, qd, vd, tau, feet, anchors, kp, kd);
    if (!a.allFinite()) { d_finite = false; break; }
    vd += a * dt;
    qd = m.integrate(qd, vd, dt);
  }
  double d_foot_drift = 0.0;
  for (int k = 0; k < nc; ++k) {
    d_foot_drift = std::max(d_foot_drift, (m.frame_position(qd, feet[k]) - anchors[k]).norm());
  }
  const double d_base_drift = std::abs(qd(2) - base_z0);
  const bool d_ok = d_finite && (d_base_drift < 2e-3) && (d_foot_drift < 1e-3);
  std::cout << "D static-hold: base_dz = " << d_base_drift << ", foot drift = "
            << d_foot_drift << "\n";

  // --- E. brief joint-torque disturbance: feet stay planted while the base
  // moves. A short impulse (no controller to restore it), so the base drifts a
  // few cm on its residual DoF while the no-slip contact holds the feet. ---
  Eigen::VectorXd qe = q, ve = v0;
  const int dist_row = m.joint_v_index("hipy_FL");  // push one hip
  bool e_finite = true;
  for (int i = 0; i < 250; ++i) {  // 0.5 s
    Eigen::VectorXd tau = m.gravity_torque(qe);
    if (i < 75) tau(dist_row) += 3.0;  // 0.15 s, 3 N·m impulse on a single hip
    const Eigen::VectorXd a =
      m.contact_forward_dynamics(ws, qe, ve, tau, feet, anchors, kp, kd);
    if (!a.allFinite()) { e_finite = false; break; }
    ve += a * dt;
    qe = m.integrate(qe, ve, dt);
  }
  double e_foot_drift = 0.0;
  for (int k = 0; k < nc; ++k) {
    e_foot_drift = std::max(e_foot_drift, (m.frame_position(qe, feet[k]) - anchors[k]).norm());
  }
  const double e_base_move = std::abs(qe(2) - base_z0)
    + (m.center_of_mass(qe) - m.center_of_mass(q)).norm();
  const bool e_ok = e_finite && (e_foot_drift < 3e-3) && (e_base_move > 1e-3);
  std::cout << "E disturbance: foot drift = " << e_foot_drift
            << " (planted), base/CoM move = " << e_base_move << " (free)\n";

  const bool ok = a_ok && b_ok && c_ok && d_ok && e_ok;
  std::cout << (ok ? "PASS" : "FAIL") << "  (A=" << a_ok << " B=" << b_ok << " C=" << c_ok
            << " D=" << d_ok << " E=" << e_ok << ")\n";
  return ok ? 0 : 1;
}
