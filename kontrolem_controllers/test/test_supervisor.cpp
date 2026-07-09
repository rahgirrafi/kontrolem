// Multi-controller Supervisor — runtime handoff between whole controllers.
//
// Follows the switching spike: the Supervisor hosts a stateful LQG and a
// stateless MPC, and on a MANUAL switch it seeds the incoming controller
// (on_activate) AND blends the command over a short window. This test drives
// LQG -> MPC -> LQG on the cart-pole across a disturbance at each switch and
// checks the Supervisor keeps the actuator's command SMOOTH:
//   - with the blend, the worst tick-to-tick command jump stays small even on the
//     LQG->MPC direction the spike showed steps ~12 N (stateless incoming, nothing
//     to seed);
//   - a HARD switch (blend disabled) through the same Supervisor still shows that
//     large jump — isolating the blend as the fix;
//   - the plant stays balanced throughout (a switch never destabilizes).
#include <algorithm>
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "kontrolem_control/problem.hpp"
#include "kontrolem_control/supervisor.hpp"
#include "kontrolem_controllers/lqg_controller.hpp"
#include "kontrolem_controllers/mpc_controller.hpp"
#include "kontrolem_model/robot_model.hpp"

using namespace kontrolem_control;
using namespace kontrolem_controllers;
using kontrolem_model::RobotModel;

namespace
{
constexpr double kDt = 0.005;
constexpr int kK1 = 240;   // t=1.20s : request LQG -> MPC
constexpr int kK2 = 480;   // t=2.40s : request MPC -> LQG
constexpr int kSteps = 800;
constexpr int kKickLead = 6;
constexpr double kKick = 1.0;

struct Result { double switch_peak = 0.0; double final_pole = 0.0, final_cart = 0.0; bool finite = true; };

Result run(const RobotModel & model, LqgController & lqg, MpcController & mpc,
           const Regulation & reg, int blend_ticks)
{
  // Fresh internal state for a clean, comparable run.
  State reset0; reset0.q = reg.q_ref; reset0.v = Eigen::VectorXd::Zero(model.nv());
  lqg.on_activate(reset0, reg);  // seed observer to the operating point

  Supervisor sup(blend_ticks);
  sup.add("lqg", &lqg);
  sup.add("mpc", &mpc);
  sup.set_active("lqg");

  Result r;
  Eigen::VectorXd q(2), v(2), tau(2);
  q << 0.0, 0.12; v << 0.0, 0.0; tau.setZero();
  State st; st.t = 0.0;
  double u_prev = 0.0; bool have_prev = false;

  for (int k = 0; k < kSteps; ++k, st.t += kDt) {
    const Eigen::VectorXd a = model.aba(q, v, tau);
    v.noalias() += a * kDt;
    q.noalias() += v * kDt;
    if (k == kK1 - kKickLead || k == kK2 - kKickLead) v(1) += kKick;

    if (k == kK1) sup.request_switch("mpc");
    if (k == kK2) sup.request_switch("lqg");

    st.q = q; st.v = v;
    const double u = sup.compute(st, reg, kDt).tau(0);
    tau << u, 0.0;

    // The switch-conditioning window: the blend length after each switch. This
    // isolates the transition itself from each controller's ordinary (and
    // legitimately aggressive) response to the disturbance, which happens OUTSIDE
    // these windows. For a hard switch the window is the single switch tick.
    const bool in_switch = (k >= kK1 && k < kK1 + blend_ticks) ||
                           (k >= kK2 && k < kK2 + blend_ticks);
    if (have_prev && in_switch) r.switch_peak = std::max(r.switch_peak, std::abs(u - u_prev));
    u_prev = u; have_prev = true;
    if (!q.allFinite()) { r.finite = false; break; }
  }
  r.final_pole = std::abs(q(1));
  r.final_cart = std::abs(q(0));
  return r;
}
}  // namespace

int main()
{
  const RobotModel model = RobotModel::from_urdf_file(CART_POLE_URDF);

  Eigen::MatrixXd Q = Eigen::Vector4d(1.0, 10.0, 1.0, 1.0).asDiagonal();
  Eigen::MatrixXd R = Eigen::Matrix<double, 1, 1>::Constant(0.1);
  Eigen::MatrixXd W = Eigen::Vector4d(1.0, 1.0, 10.0, 10.0).asDiagonal();
  Eigen::MatrixXd V = 1e-3 * Eigen::Matrix2d::Identity();
  LqgController lqg({"cart_joint"}, Q, R, W, V, /*innov_max=*/0.5);
  MpcController mpc({"cart_joint"}, Q, R, /*horizon=*/30, /*dt_mpc=*/0.02, /*tau_max=*/100.0);

  Regulation reg;
  reg.q_ref = Eigen::VectorXd::Zero(model.nq());
  reg.v_ref = Eigen::VectorXd::Zero(model.nv());
  lqg.configure(model, *lqg.synthesize(model, reg), reg);
  mpc.configure(model, *mpc.synthesize(model, reg), reg);

  const Result blended = run(model, lqg, mpc, reg, /*blend_ticks=*/20);
  const Result hard    = run(model, lqg, mpc, reg, /*blend_ticks=*/1);

  std::cout << "command jump AT THE SWITCH (max tick-to-tick within the handoff window):\n"
            << "  HARD switch (blend off) : " << hard.switch_peak << " N\n"
            << "  BLENDED (20-tick)       : " << blended.switch_peak << " N\n";
  std::cout << "balanced at end?  blended final|pole|=" << blended.final_pole
            << " |cart|=" << blended.final_cart
            << " ; hard final|pole|=" << hard.final_pole << "\n";

  const bool finite = blended.finite && hard.finite;
  const bool balanced = blended.final_pole < 1e-2 && blended.final_cart < 1.0 &&
                        hard.final_pole < 1e-2;
  const bool blend_smooths = blended.switch_peak < 0.4 * hard.switch_peak;  // the blend clearly helps
  const bool blend_bounded = blended.switch_peak < 2.0;                     // no violent step
  const bool ok = finite && balanced && blend_smooths && blend_bounded;
  std::cout << (ok ? "PASS" : "FAIL") << "  (finite=" << finite << " balanced=" << balanced
            << " blend_smooths=" << blend_smooths << " blend_bounded=" << blend_bounded << ")\n";
  return ok ? 0 : 1;
}
