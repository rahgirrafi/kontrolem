// LpvController (gain-scheduling / LPV) on the 2-DoF arm. The arm's gravity load
// g(q) and inertia M(q) vary strongly with configuration, so a SINGLE LQR (designed
// at one pose) carries the wrong feedforward off-design and droops. The LpvController
// designs an LQR at every node of a grid over (shoulder, elbow) and multilinearly
// interpolates at runtime — it should hold every reachable pose with near-zero
// steady-state error, and report not-ok outside the designed envelope.
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "kontrolem_control/problem.hpp"
#include "kontrolem_controllers/lpv_controller.hpp"
#include "kontrolem_controllers/lqr_controller.hpp"
#include "kontrolem_model/robot_model.hpp"

using namespace kontrolem_control;
using namespace kontrolem_controllers;
using kontrolem_model::RobotModel;

namespace
{
// Closed-loop regulation of the arm from [0,0] to a goal, driving `ctrl`. Returns
// the steady-state error |q - goal| after settling.
double regulate(const RobotModel & model, Controller & ctrl, const Eigen::Vector2d & goal)
{
  Regulation reg;
  reg.q_ref = goal;
  reg.v_ref = Eigen::Vector2d::Zero();
  ctrl.on_activate(State{}, reg);
  Eigen::Vector2d q(0.0, 0.0), v(0.0, 0.0), tau(0.0, 0.0);
  State st; st.t = 0.0;
  const double dt = 0.002;
  for (int k = 0; k < 6000; ++k, st.t += dt) {
    st.q = q; st.v = v;
    tau = ctrl.compute(st, reg, dt).tau;
    const Eigen::Vector2d a = model.aba(q, v, tau);
    v.noalias() += a * dt;
    q.noalias() += v * dt;
    if (!q.allFinite()) return 9.9;
  }
  return (q - goal).norm();
}
}  // namespace

int main()
{
  const RobotModel model = RobotModel::from_urdf_file(ARM2_URDF);
  Eigen::MatrixXd Q = Eigen::Vector4d(10.0, 10.0, 1.0, 1.0).asDiagonal();
  Eigen::MatrixXd R = Eigen::Vector2d(0.1, 0.1).asDiagonal();

  // Scheduled controller: grid over both joint angles.
  std::vector<SchedAxis> axes = {
    SchedAxis{0, -1.6, 1.6, 17},   // shoulder (0.2 rad node spacing)
    SchedAxis{1, -1.6, 1.6, 17}};  // elbow
  LpvController lpv({"shoulder_joint", "elbow_joint"}, Q, R, axes);
  Regulation reg0;
  reg0.q_ref = Eigen::Vector2d::Zero();
  reg0.v_ref = Eigen::Vector2d::Zero();
  lpv.configure(model, *lpv.synthesize(model, reg0), reg0);

  // Fixed baseline: a single LQR designed at [0,0] (both links horizontal).
  LqrController fixed({"shoulder_joint", "elbow_joint"}, Q, R);
  fixed.configure(model, *fixed.synthesize(model, reg0), reg0);

  const Eigen::Vector2d goals[] = {
    {0.5, 0.5}, {1.0, -0.5}, {1.5, 0.5}, {-0.8, 1.0}, {1.2, -1.2}, {0.3, 1.4}};
  double lpv_worst = 0.0, fixed_worst = 0.0;
  std::cout << "goal            fixed ss_err    lpv ss_err\n";
  for (const auto & g : goals) {
    const double ef = regulate(model, fixed, g);
    const double el = regulate(model, lpv, g);
    fixed_worst = std::max(fixed_worst, ef);
    lpv_worst = std::max(lpv_worst, el);
    std::cout << "  (" << g(0) << "," << g(1) << ")      " << ef << "        " << el << "\n";
  }

  // Envelope: a scheduling variable outside the designed grid must report not-ok.
  State out; out.t = 0.0;
  out.q = Eigen::Vector2d(3.0, 0.0);   // shoulder 3.0 rad >> grid max 1.6
  out.v = Eigen::Vector2d::Zero();
  lpv.compute(out, reg0, 0.002);
  const bool envelope_flags = !lpv.status().ok && lpv.status().margin < 0.0;

  State in; in.t = 0.0;
  in.q = Eigen::Vector2d(0.2, -0.3);   // inside the grid
  in.v = Eigen::Vector2d::Zero();
  lpv.compute(in, reg0, 0.002);
  const bool inside_ok = lpv.status().ok && lpv.status().margin > 0.0;

  std::cout << "\nnodes designed = " << lpv.node_count() << " (expect 289)\n";
  std::cout << "worst ss_err:  fixed = " << fixed_worst << " ,  lpv = " << lpv_worst << "\n";
  std::cout << "envelope: outside->not-ok = " << envelope_flags
            << " , inside->ok = " << inside_ok << "\n";

  const bool nodes_ok = lpv.node_count() == 289;
  const bool lpv_holds = lpv_worst < 0.05;                 // holds every pose
  const bool fixed_droops = fixed_worst > 0.2;             // single LQR clearly worse
  const bool clear_win = fixed_worst > 5.0 * std::max(lpv_worst, 1e-9);
  const bool ok = nodes_ok && lpv_holds && fixed_droops && clear_win &&
                  envelope_flags && inside_ok;
  std::cout << (ok ? "PASS" : "FAIL")
            << "  (nodes=" << nodes_ok << " lpv_holds=" << lpv_holds
            << " fixed_droops=" << fixed_droops << " clear_win=" << clear_win
            << " envelope=" << envelope_flags << " inside=" << inside_ok << ")\n";
  return ok ? 0 : 1;
}
