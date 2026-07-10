// Supervisor AUTOMATIC fail-forward (Part C8: the meta-controller acting on the
// trust predicate, not just a human command).
//
// LQR is primary but carries a modest validity region (q_dev_max) — outside it,
// its linearization is no longer trustworthy and status().ok goes false. MPC is
// the fallback (constrained, no such self-limit). A disturbance drives the
// cart-pole out of LQR's region; the Supervisor must, WITH NO MANUAL COMMAND,
// detect the sustained not-ok and hand off to MPC (bumplessly), which recovers.
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
  // LQR primary with a deliberately MODEST trust region so a real disturbance
  // leaves it (and it honestly reports so) — the trigger for auto-fallback.
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

  const double dt = 0.005;
  Eigen::VectorXd q(2), v(2), tau(2);
  q << 0.0, 0.0; v << 0.0, 0.0; tau.setZero();  // start balanced -> LQR legitimately in-region
  State st; st.t = 0.0;

  const int kKick = 200;   // t = 1.0 s
  std::string law_before_kick, law_at_end;
  bool switched_after_kick = false;
  int switch_tick = -1;

  const int steps = 1000;
  for (int k = 0; k < steps; ++k, st.t += dt) {
    const Eigen::VectorXd a = model.aba(q, v, tau);
    v.noalias() += a * dt;
    q.noalias() += v * dt;
    if (k == kKick) v(1) += 2.0;  // a shove that drives the state out of LQR's region

    st.q = q; st.v = v;
    // NOTE: no request_switch() anywhere — any handoff here is the Supervisor's own.
    const double u = sup.compute(st, reg, dt).tau(0);
    tau << u, 0.0;

    if (k == kKick - 1) law_before_kick = sup.active_name();
    if (k > kKick && sup.active_name() == "mpc" && !switched_after_kick) {
      switched_after_kick = true;
      switch_tick = k;
    }
    if (!q.allFinite()) { std::cout << "FAIL: diverged at k=" << k << "\n"; return 1; }
  }
  law_at_end = sup.active_name();
  const double final_pole = std::abs(q(1));
  const double final_cart = std::abs(q(0));

  std::cout << "active law before kick = " << law_before_kick
            << " ; after auto-fallback = " << law_at_end
            << " (switched at tick " << switch_tick << ", kick at " << kKick << ")\n";
  std::cout << "recovered: final |pole| = " << final_pole << ", |cart| = " << final_cart << "\n";

  const bool started_on_lqr = (law_before_kick == "lqr");     // LQR ran while in-region
  const bool auto_switched = switched_after_kick && (law_at_end == "mpc");  // no manual cmd
  const bool switched_after = switch_tick > kKick;            // triggered BY the disturbance
  const bool recovered = final_pole < 1e-2 && final_cart < 1.0;
  const bool ok = started_on_lqr && auto_switched && switched_after && recovered;
  std::cout << (ok ? "PASS" : "FAIL") << "  (started_on_lqr=" << started_on_lqr
            << " auto_switched=" << auto_switched << " switched_after_kick=" << switched_after
            << " recovered=" << recovered << ")\n";
  return ok ? 0 : 1;
}
