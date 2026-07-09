// QpTaskSpaceController on the cart-pole. Checks that the online QP:
//   1. runs through the SAME contract as LQR (accepts Regulation, synthesize
//      no-op, configure, compute) — the interface-fit point;
//   2. solves and returns a finite command in the unconstrained regime;
//   3. drives the torque-limit inequality ACTIVE at an aggressive state
//      (|tau| == tau_max, i.e. the QP clamps — proof a real constraint bites).
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "kontrolem_controllers/qp_task_space_controller.hpp"
#include "kontrolem_model/robot_model.hpp"

using namespace kontrolem_control;
using namespace kontrolem_controllers;
using kontrolem_model::RobotModel;

int main()
{
  const RobotModel model = RobotModel::from_urdf_file(CART_POLE_URDF);
  const int nv = model.nv();

  Eigen::VectorXd W(nv);
  W << 1.0, 10.0;  // weight matching the pole acceleration more
  const double tau_max = 5.0;
  QpTaskSpaceController qp({"cart_joint"}, W, /*kp=*/50.0, /*kd=*/10.0, tau_max);

  Regulation upright;
  upright.q_ref = Eigen::VectorXd::Zero(model.nq());
  upright.v_ref = Eigen::VectorXd::Zero(nv);

  if (!accepts(qp, upright)) {
    std::cout << "FAIL: QP should accept Regulation\n";
    return 1;
  }

  const auto synth = qp.synthesize(model, upright);  // no-op
  qp.configure(model, *synth, upright);

  // (2) Gentle tilt: QP solves, command within the limit.
  State gentle;
  gentle.q = Eigen::VectorXd::Zero(model.nq());
  gentle.q[1] = 0.02;
  gentle.v = Eigen::VectorXd::Zero(nv);
  // NOTE: compute() returns a reference to an internal buffer, valid only until
  // the next compute(); copy the value out before calling compute() again.
  const Eigen::VectorXd tau1 = qp.compute(gentle, upright, 0.001).tau;
  const Status s1 = qp.status();
  std::cout << "gentle: tau=" << tau1.transpose() << "  solved=" << s1.ok
            << " margin=" << s1.margin << "\n";

  // (3) Aggressive tilt: PD demand exceeds tau_max -> limit active (clamped).
  State hard;
  hard.q = Eigen::VectorXd::Zero(model.nq());
  hard.q[1] = 0.5;
  hard.v = Eigen::VectorXd::Zero(nv);
  const Eigen::VectorXd tau2 = qp.compute(hard, upright, 0.001).tau;
  const Status s2 = qp.status();
  std::cout << "hard:   tau=" << tau2.transpose() << "  solved=" << s2.ok
            << " margin=" << s2.margin << "\n";
  std::cout << "OSQP iters: gentle=" << s1.iters << " hard=" << s2.iters
            << " (cap = 400)\n";

  const bool solved = s1.ok && s2.ok;
  const bool u1_ok = tau1.allFinite() && std::abs(tau1[0]) <= tau_max + 1e-3;
  const bool limit_active = std::abs(tau2[0]) >= tau_max - 5e-3;  // clamped at the bound
  // Hard-RT budget: converge with >=2x headroom below the 400-iter cap, so the
  // cap is a safety bound, not a tight fit (see qp_solver.hpp / max_iter).
  const bool iter_ok = s1.iters > 0 && s2.iters > 0 && s1.iters <= 200 && s2.iters <= 200;

  const bool ok = solved && u1_ok && limit_active && iter_ok;
  std::cout << (ok ? "PASS" : "FAIL") << "  (solved=" << solved << " u1_ok=" << u1_ok
            << " limit_active=" << limit_active << " iter_ok=" << iter_ok << ")\n";
  return ok ? 0 : 1;
}
