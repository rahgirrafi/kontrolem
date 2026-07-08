// MpcController on the TRACKING dialect — the predictive advantage:
//   1. accepts Tracking (and Regulation).
//   2. Following a moving harmonic cart reference, MPC uses the FUTURE reference
//      over the horizon and tracks much tighter than feedback-only LQR would
//      (LQR feedback-only at w=1.0 is ~0.155; MPC must be well under that).
// Dependency-free (no gtest); uses model.aba as the plant.
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "kontrolem_control/problem.hpp"
#include "kontrolem_control/trajectory.hpp"
#include "kontrolem_controllers/mpc_controller.hpp"
#include "kontrolem_model/robot_model.hpp"

using namespace kontrolem_control;
using namespace kontrolem_controllers;
using kontrolem_model::RobotModel;

int main()
{
  const RobotModel model = RobotModel::from_urdf_file(CART_POLE_URDF);

  Eigen::MatrixXd Q = Eigen::Vector4d(10.0, 10.0, 1.0, 1.0).asDiagonal();
  Eigen::MatrixXd R = Eigen::Matrix<double, 1, 1>::Constant(0.1);
  MpcController mpc({"cart_joint"}, Q, R, /*N=*/30, /*dt_mpc=*/0.02, /*tau_max=*/50.0);

  const double A = 0.3, w = 1.0;  // the reference speed where LQR feedback lags most
  HarmonicReference ref;
  ref.center = Eigen::Vector2d(0.0, 0.0);
  ref.amp = Eigen::Vector2d(A, 0.0);
  ref.phase = Eigen::Vector2d(0.0, 0.0);
  ref.omega = w;
  Tracking problem;
  problem.reference = &ref;

  const bool accepts_tracking = accepts(mpc, problem);

  auto s = mpc.synthesize(model, problem);
  mpc.configure(model, *s, problem);

  const double dt = 0.005;
  Eigen::VectorXd q(2), v(2), tau(2);
  q << A, 0.0; v << 0.0, 0.0; tau.setZero();
  State st;
  double max_err = 0.0, max_pole = 0.0, max_cart = 0.0;
  bool finite = true;
  for (int k = 0; k <= 2000; ++k) {
    const Eigen::VectorXd a = model.aba(q, v, tau);
    v.noalias() += a * dt;
    q.noalias() += v * dt;
    st.q = q; st.v = v; st.t = (k + 1) * dt;
    const Command & u = mpc.compute(st, problem, dt);
    tau << u.tau(0), 0.0;
    if (!q.allFinite()) { finite = false; break; }
    const double cart_ref = A * std::cos(w * st.t);
    if (k > 400) {
      max_err = std::max(max_err, std::abs(q(0) - cart_ref));
      max_cart = std::max(max_cart, std::abs(q(0)));
      max_pole = std::max(max_pole, std::abs(q(1)));
    }
  }

  std::cout << "accepts Tracking=" << accepts_tracking << "  max|cart-ref|=" << max_err
            << " (LQR feedback-only ~0.155)  cart amp=" << max_cart
            << "  max|pole|=" << max_pole << "\n";

  const bool predictive = finite && max_err < 0.05;   // well under feedback-only lag
  const bool moving = max_cart > 0.20 && max_cart < 0.45;
  const bool upright = max_pole < 0.10;
  const bool ok = accepts_tracking && predictive && moving && upright;
  std::cout << (ok ? "PASS" : "FAIL") << "  (predictive=" << predictive << " moving=" << moving
            << " upright=" << upright << ")\n";
  return ok ? 0 : 1;
}
