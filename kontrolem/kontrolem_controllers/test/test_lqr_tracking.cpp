// LqrController on the TRACKING dialect — proves the same controller (and gain
// law) serves a second, structurally different problem dialect:
//   1. capabilities() accepts Tracking (as well as Regulation).
//   2. Given a moving harmonic cart reference (pole held upright), the closed
//      loop FOLLOWS it: bounded tracking error, and the cart genuinely moves
//      with the reference (amplitude ~ A, not stuck at 0), pole stays upright.
// Dependency-free (no gtest); uses model.aba as the plant like the other tests.
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "kontrolem_control/problem.hpp"
#include "kontrolem_control/trajectory.hpp"
#include "kontrolem_controllers/lqr_controller.hpp"
#include "kontrolem_model/robot_model.hpp"

using namespace kontrolem_control;
using namespace kontrolem_controllers;
using kontrolem_model::RobotModel;

int main()
{
  const RobotModel model = RobotModel::from_urdf_file(CART_POLE_URDF);

  Eigen::MatrixXd Q = Eigen::Vector4d(10.0, 10.0, 1.0, 1.0).asDiagonal();
  Eigen::MatrixXd R = Eigen::Matrix<double, 1, 1>::Constant(0.1);
  LqrController lqr({"cart_joint"}, Q, R, /*q_dev_max=*/5.0);

  const double A = 0.3, w = 0.5;
  HarmonicReference ref;
  ref.center = Eigen::Vector2d(0.0, 0.0);
  ref.amp = Eigen::Vector2d(A, 0.0);   // cart oscillates; pole amp 0 (held upright)
  ref.phase = Eigen::Vector2d(0.0, 0.0);
  ref.omega = w;

  Tracking problem;
  problem.reference = &ref;

  const bool accepts_tracking = accepts(lqr, problem);
  const bool accepts_regulation = [&] {
    Regulation r;
    r.q_ref = Eigen::VectorXd::Zero(model.nq());
    r.v_ref = Eigen::VectorXd::Zero(model.nv());
    return accepts(lqr, r);
  }();

  const auto synthesis = lqr.synthesize(model, problem);
  lqr.configure(model, *synthesis, problem);

  const double dt = 0.005;
  Eigen::VectorXd q(2), v(2), tau(2);
  q << A, 0.0;  // start on the reference at t=0
  v << 0.0, 0.0;
  tau.setZero();

  State st;
  double max_track_err = 0.0, max_pole = 0.0, max_cart = 0.0;
  bool finite = true;
  for (int k = 0; k <= 2000; ++k) {
    const Eigen::VectorXd a = model.aba(q, v, tau);
    v.noalias() += a * dt;
    q.noalias() += v * dt;
    st.q = q;
    st.v = v;
    st.t = (k + 1) * dt;
    const Command & u = lqr.compute(st, problem, dt);
    tau << u.tau(0), 0.0;
    if (!q.allFinite()) { finite = false; break; }
    const double cart_ref = A * std::cos(w * st.t);
    if (k > 400) {  // after warm-up
      max_track_err = std::max(max_track_err, std::abs(q(0) - cart_ref));
      max_cart = std::max(max_cart, std::abs(q(0)));
      max_pole = std::max(max_pole, std::abs(q(1)));
    }
  }

  std::cout << "accepts Tracking=" << accepts_tracking << " Regulation=" << accepts_regulation
            << "  max|cart-ref|=" << max_track_err << "  cart amplitude=" << max_cart
            << "  max|pole|=" << max_pole << "\n";

  const bool tracks = finite && max_track_err < 0.10;         // follows with modest lag
  const bool really_moving = max_cart > 0.20 && max_cart < 0.45;  // ~A, not stuck / not diverged
  const bool upright = max_pole < 0.10;                       // pole held up
  const bool ok = accepts_tracking && accepts_regulation && tracks && really_moving && upright;
  std::cout << (ok ? "PASS" : "FAIL") << "  (tracks=" << tracks << " moving=" << really_moving
            << " upright=" << upright << ")\n";
  return ok ? 0 : 1;
}
