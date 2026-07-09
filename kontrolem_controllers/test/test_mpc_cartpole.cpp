// MpcController correctness on the cart-pole — constrained receding-horizon:
//   1. accepts Regulation.
//   2. Unconstrained (loose limit): regulates pole=0.15 -> upright.
//   3. Tight torque limit: the QP keeps |tau| <= tau_max (constraint ACTIVE,
//      i.e. the limit is actually reached, not just respected) and STILL
//      stabilizes — the point of solving the limit inside the optimization.
// Dependency-free (no gtest); uses model.aba as the plant.
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "kontrolem_control/problem.hpp"
#include "kontrolem_controllers/mpc_controller.hpp"
#include "kontrolem_model/robot_model.hpp"

using namespace kontrolem_control;
using namespace kontrolem_controllers;
using kontrolem_model::RobotModel;

namespace
{
// Returns (final|pole|, final|cart|, max|tau|). Runs a closed-loop sim.
struct Result { double pole, cart, max_tau; bool finite; int max_iters; };
Result run(const RobotModel & model, double tau_max)
{
  Eigen::MatrixXd Q = Eigen::Vector4d(1.0, 10.0, 1.0, 1.0).asDiagonal();
  Eigen::MatrixXd R = Eigen::Matrix<double, 1, 1>::Constant(0.1);
  MpcController mpc({"cart_joint"}, Q, R, /*horizon=*/30, /*dt_mpc=*/0.02, tau_max);

  Regulation reg;
  reg.q_ref = Eigen::VectorXd::Zero(model.nq());
  reg.v_ref = Eigen::VectorXd::Zero(model.nv());
  auto s = mpc.synthesize(model, reg);
  mpc.configure(model, *s, reg);

  const double dt = 0.005;
  Eigen::VectorXd q(2), v(2), tau(2);
  q << 0.0, 0.15; v << 0.0, 0.0; tau.setZero();
  State st;
  double max_tau = 0.0;
  int max_iters = 0;
  bool finite = true;
  for (int k = 0; k <= 1600; ++k) {
    const Eigen::VectorXd a = model.aba(q, v, tau);
    v.noalias() += a * dt;
    q.noalias() += v * dt;
    st.q = q; st.v = v; st.t = (k + 1) * dt;
    const Command & u = mpc.compute(st, reg, dt);
    tau << u.tau(0), 0.0;
    max_tau = std::max(max_tau, std::abs(tau(0)));
    if (mpc.status().ok) max_iters = std::max(max_iters, mpc.status().iters);
    if (!q.allFinite()) { finite = false; break; }
  }
  return {std::abs(q(1)), std::abs(q(0)), max_tau, finite, max_iters};
}
}  // namespace

int main()
{
  const RobotModel model = RobotModel::from_urdf_file(CART_POLE_URDF);

  const bool accepts_reg = [&] {
    Eigen::MatrixXd Q = Eigen::Vector4d(1, 10, 1, 1).asDiagonal();
    Eigen::MatrixXd R = Eigen::Matrix<double, 1, 1>::Constant(0.1);
    MpcController m({"cart_joint"}, Q, R, 30, 0.02, 100.0);
    Regulation r; r.q_ref = Eigen::VectorXd::Zero(2); r.v_ref = Eigen::VectorXd::Zero(2);
    return accepts(m, r);
  }();

  const Result loose = run(model, 100.0);
  const Result tight = run(model, 3.0);

  std::cout << "loose: pole=" << loose.pole << " cart=" << loose.cart
            << " max_tau=" << loose.max_tau << " (unconstrained peak)\n";
  std::cout << "tight: pole=" << tight.pole << " cart=" << tight.cart
            << " max_tau=" << tight.max_tau << " (limit 3.0)\n";
  std::cout << "max OSQP iters on solved ticks: loose=" << loose.max_iters
            << " tight=" << tight.max_iters << " (cap = 2000)\n";

  const bool loose_stab = loose.finite && loose.pole < 1e-2 && loose.cart < 1e-2;
  const bool tight_stab = tight.finite && tight.pole < 1e-2 && tight.cart < 1e-2;
  const bool limit_respected = tight.max_tau <= 3.0 + 1e-3;
  const bool limit_active = tight.max_tau >= 3.0 - 1e-3;   // the cap is actually reached
  // Hard-RT budget: the condensed 30-step QP converges (~850 iters at eps=1e-6
  // under the fixed-rho/no-polish config) with >=2x headroom below the 2000-iter
  // cap on every solved tick (see mpc_controller.hpp / max_iter_).
  const bool iter_ok = loose.max_iters > 0 && tight.max_iters > 0 &&
                       loose.max_iters <= 1000 && tight.max_iters <= 1000;
  const bool ok = accepts_reg && loose_stab && tight_stab && limit_respected && limit_active &&
                  iter_ok;
  std::cout << (ok ? "PASS" : "FAIL") << "  (accepts=" << accepts_reg << " loose_stab="
            << loose_stab << " tight_stab=" << tight_stab << " respected=" << limit_respected
            << " active=" << limit_active << " iter_ok=" << iter_ok << ")\n";
  return ok ? 0 : 1;
}
