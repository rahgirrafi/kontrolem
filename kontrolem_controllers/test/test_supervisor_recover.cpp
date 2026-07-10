// Supervisor AUTOMATIC recovery — the switch-BACK half of dwell-time/hysteresis
// switching (Part C8, Hespanha-Morse). Extends the fail-forward test: after the
// Supervisor fails over from LQR to MPC under a disturbance, MPC settles the
// cart-pole back toward upright. Once the state re-enters LQR's validity region
// AND stays there for a long recovery dwell, the Supervisor must — with NO manual
// command — SHADOW-evaluate the dormant primary's health and hand back to it.
//
// The round trip is lqr -> (shove) -> mpc -> (settles) -> lqr, and the recovery
// handoff must be bumpless (blended) and must not chatter.
#include <cmath>
#include <iostream>
#include <string>

#include <Eigen/Dense>

#include "kontrolem_control/problem.hpp"
#include "kontrolem_control/supervisor.hpp"
#include "kontrolem_controllers/lqr_controller.hpp"
#include "kontrolem_controllers/mpc_controller.hpp"
#include "kontrolem_model/robot_model.hpp"

using namespace kontrolem_control;
using namespace kontrolem_controllers;
using kontrolem_model::RobotModel;

int main()
{
  const RobotModel model = RobotModel::from_urdf_file(CART_POLE_URDF);

  Eigen::MatrixXd Q = Eigen::Vector4d(1.0, 10.0, 1.0, 1.0).asDiagonal();
  Eigen::MatrixXd R = Eigen::Matrix<double, 1, 1>::Constant(0.1);
  LqrController lqr({"cart_joint"}, Q, R, /*q_dev_max=*/0.30);
  MpcController mpc({"cart_joint"}, Q, R, /*horizon=*/30, /*dt_mpc=*/0.02, /*tau_max=*/100.0);

  Regulation reg;
  reg.q_ref = Eigen::VectorXd::Zero(model.nq());
  reg.v_ref = Eigen::VectorXd::Zero(model.nv());
  lqr.configure(model, *lqr.synthesize(model, reg), reg);
  mpc.configure(model, *mpc.synthesize(model, reg), reg);

  Supervisor sup(/*blend_ticks=*/20);
  sup.add("lqr", &lqr);
  sup.add("mpc", &mpc);
  sup.set_active("lqr");
  sup.set_auto_fallback(true, /*dwell_ticks=*/5);
  sup.set_auto_recover(true, /*dwell_ticks=*/50);  // ~0.25 s of sustained health before switch-back

  const double dt = 0.005;
  Eigen::VectorXd q(2), v(2), tau(2);
  q << 0.0, 0.0; v << 0.0, 0.0; tau.setZero();  // start balanced -> LQR legitimately in-region
  State st; st.t = 0.0;

  const int kKick = 200;   // t = 1.0 s
  std::string law_before_kick;
  int fwd_tick = -1;        // tick LQR -> MPC (fail-forward)
  int back_tick = -1;       // tick MPC -> LQR (recovery)
  int law_switches = 0;     // count active-name changes AFTER recovery, to catch chatter
  std::string prev_law = "lqr";
  double peak_recover_jump = 0.0;
  double u_prev = 0.0;

  const int steps = 2000;   // long enough to settle AND recover
  for (int k = 0; k < steps; ++k, st.t += dt) {
    const Eigen::VectorXd a = model.aba(q, v, tau);
    v.noalias() += a * dt;
    q.noalias() += v * dt;
    if (k == kKick) v(1) += 2.0;  // a shove that drives the state out of LQR's region

    st.q = q; st.v = v;
    // NOTE: no request_switch() anywhere — every handoff here is the Supervisor's own.
    const double u = sup.compute(st, reg, dt).tau(0);

    // Track the recovery handoff smoothness (jump in command around the back-switch).
    if (back_tick >= 0 && k >= back_tick && k <= back_tick + 25) {
      peak_recover_jump = std::max(peak_recover_jump, std::abs(u - u_prev));
    }
    u_prev = u;
    tau << u, 0.0;

    const std::string law = sup.active_name();
    if (k == kKick - 1) law_before_kick = law;
    if (law != prev_law) {
      if (prev_law == "lqr" && law == "mpc" && fwd_tick < 0 && k > kKick) fwd_tick = k;
      if (prev_law == "mpc" && law == "lqr" && back_tick < 0) back_tick = k;
      if (back_tick >= 0 && k > back_tick) ++law_switches;  // any further switch = chatter
      prev_law = law;
    }
    if (!q.allFinite()) { std::cout << "FAIL: diverged at k=" << k << "\n"; return 1; }
  }
  const std::string law_at_end = sup.active_name();
  const double final_pole = std::abs(q(1));
  const double final_cart = std::abs(q(0));

  std::cout << "round trip: " << law_before_kick << " -> mpc(@" << fwd_tick
            << ") -> lqr(@" << back_tick << "), end=" << law_at_end << "\n";
  std::cout << "recovery |dU| peak = " << peak_recover_jump
            << " ; post-recovery extra switches (chatter) = " << law_switches << "\n";
  std::cout << "final |pole| = " << final_pole << ", |cart| = " << final_cart << "\n";

  const bool started_on_lqr = (law_before_kick == "lqr");
  const bool failed_forward = fwd_tick > kKick;                 // LQR -> MPC on the shove
  const bool recovered_back = back_tick > fwd_tick && law_at_end == "lqr";  // MPC -> LQR, no manual cmd
  const bool no_chatter = (law_switches == 0);                  // exactly one switch-back, stayed put
  const bool smooth = peak_recover_jump < 5.0;                  // blended handoff, not a step
  const bool settled = final_pole < 1e-2 && final_cart < 1.0;
  const bool ok = started_on_lqr && failed_forward && recovered_back && no_chatter && smooth && settled;
  std::cout << (ok ? "PASS" : "FAIL")
            << "  (started_on_lqr=" << started_on_lqr << " failed_forward=" << failed_forward
            << " recovered_back=" << recovered_back << " no_chatter=" << no_chatter
            << " smooth=" << smooth << " settled=" << settled << ")\n";
  return ok ? 0 : 1;
}
