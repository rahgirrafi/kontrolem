// LqrController on the cart-DOUBLE-inverted-pendulum — the framework on a
// genuinely hard underactuated benchmark (1 actuator, 2 passive poles, 3 DoF):
//   1. synthesize -> the closed loop A - B_act K is stable.
//   2. closed-loop sim from both poles tilted 0.1 rad recovers to upright.
// Dependency-free (no gtest). Uses model.aba as the plant.
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "kontrolem_control/problem.hpp"
#include "kontrolem_controllers/lqr_controller.hpp"
#include "kontrolem_model/robot_model.hpp"

using namespace kontrolem_control;
using namespace kontrolem_controllers;
using kontrolem_model::RobotModel;

int main()
{
  const RobotModel model = RobotModel::from_urdf_file(CDP_URDF);
  const int nv = model.nv();  // 3
  if (nv != 3) { std::cout << "FAIL: expected 3 DoF\n"; return 1; }

  Eigen::VectorXd qd(6);
  qd << 1.0, 20.0, 20.0, 1.0, 1.0, 1.0;  // weight both pole angles heavily
  Eigen::MatrixXd Q = qd.asDiagonal();
  Eigen::MatrixXd R = Eigen::Matrix<double, 1, 1>::Constant(0.05);
  LqrController lqr({"cart_joint"}, Q, R, /*q_dev_max=*/5.0);

  Regulation upright;
  upright.q_ref = Eigen::VectorXd::Zero(3);
  upright.v_ref = Eigen::VectorXd::Zero(3);

  const auto synthesis = lqr.synthesize(model, upright);
  lqr.configure(model, *synthesis, upright);

  // (1) closed-loop stability of A - B_act K about upright.
  const auto lin =
    model.linearize(upright.q_ref, Eigen::VectorXd::Zero(nv), model.gravity_torque(upright.q_ref));
  const Eigen::MatrixXd B_act = lin.B.col(0);  // cart = v-index 0
  const Eigen::MatrixXd Acl = lin.A - B_act * lqr.gain();
  const double max_re = Acl.eigenvalues().real().maxCoeff();
  const double open_re = lin.A.eigenvalues().real().maxCoeff();
  const bool stable = max_re < -1e-9;

  // (2) closed-loop sim from both poles at 0.1 rad.
  const double dt = 0.005;
  Eigen::VectorXd q(3), v(3), tau(3);
  q << 0.0, 0.10, 0.10;
  v.setZero();
  tau.setZero();
  State st;
  bool finite = true;
  for (int k = 0; k <= 2000; ++k) {
    const Eigen::VectorXd a = model.aba(q, v, tau);
    v.noalias() += a * dt;
    q.noalias() += v * dt;
    st.q = q; st.v = v; st.t = (k + 1) * dt;
    const Command & u = lqr.compute(st, upright, dt);
    tau << u.tau(0), 0.0, 0.0;
    if (!q.allFinite()) { finite = false; break; }
  }
  const bool recovered = finite && std::abs(q(1)) < 1e-2 && std::abs(q(2)) < 1e-2 &&
                         std::abs(q(0)) < 1e-2;

  std::cout << "open-loop max Re=" << open_re << " closed-loop max Re=" << max_re
            << "  final th1=" << q(1) << " th2=" << q(2) << " cart=" << q(0) << "\n";
  const bool ok = stable && recovered;
  std::cout << (ok ? "PASS" : "FAIL") << "  (stable=" << stable << " recovered=" << recovered
            << ")\n";
  return ok ? 0 : 1;
}
