// LqgController correctness on the cart-pole — OUTPUT FEEDBACK:
//   1. capabilities() declares it does NOT need velocity state.
//   2. Closed-loop discrete sim from pole=0.15 rad using ONLY the position
//      measurement (velocity is estimated internally): the pole balances
//      (|pole|, |cart| -> ~0).
//   3. The internal velocity estimate tracks the TRUE velocity (the Kalman
//      filter does real work), verified after a short warm-up.
// Dependency-free (no gtest), consistent with the slice; uses model.aba as the
// plant, mirroring the LQR closed-loop test.
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "kontrolem_controllers/lqg_controller.hpp"
#include "kontrolem_model/robot_model.hpp"

using namespace kontrolem_control;
using namespace kontrolem_controllers;
using kontrolem_model::RobotModel;

int main()
{
  const RobotModel model = RobotModel::from_urdf_file(CART_POLE_URDF);
  const int nv = model.nv();

  Eigen::MatrixXd Q = Eigen::Vector4d(1.0, 10.0, 1.0, 1.0).asDiagonal();
  Eigen::MatrixXd R = Eigen::Matrix<double, 1, 1>::Constant(0.1);
  Eigen::MatrixXd W = Eigen::Vector4d(1.0, 1.0, 10.0, 10.0).asDiagonal();
  Eigen::MatrixXd V = 1e-3 * Eigen::Matrix2d::Identity();

  LqgController lqg({"cart_joint"}, Q, R, W, V, /*innov_max=*/0.5);

  const bool no_vel_needed = !lqg.capabilities().needs_velocity_state;

  Regulation upright;
  upright.q_ref = Eigen::VectorXd::Zero(model.nq());
  upright.v_ref = Eigen::VectorXd::Zero(nv);
  if (!accepts(lqg, upright)) {
    std::cout << "FAIL: LQG should accept Regulation\n";
    return 1;
  }

  const auto synthesis = lqg.synthesize(model, upright);
  lqg.configure(model, *synthesis, upright);

  // Closed-loop output-feedback sim (positions only reach the controller).
  const double dt = 0.005;
  Eigen::VectorXd q(2), v(2), tau(2);
  q << 0.0, 0.15;
  v << 0.0, 0.0;
  tau.setZero();

  State st;
  st.t = 0.0;
  double max_est_err = 0.0;  // |est pole-vel - true pole-vel| after warm-up
  for (int k = 0; k <= 2000; ++k) {
    const Eigen::VectorXd a = model.aba(q, v, tau);
    v.noalias() += a * dt;
    q.noalias() += v * dt;
    st.q = q;
    st.v = v;  // deliberately provided; LQG must ignore it (output feedback)
    const Command & u = lqg.compute(st, upright, dt);
    tau << u.tau(0), 0.0;
    if (k > 200) {  // after the estimator warm-up
      max_est_err = std::max(max_est_err, std::abs(lqg.estimate()(3) - v(1)));
    }
    if (!q.allFinite()) {
      std::cout << "FAIL: diverged at k=" << k << "\n";
      return 1;
    }
  }

  const double pole_final = std::abs(q(1));
  const double cart_final = std::abs(q(0));
  std::cout << "final |pole|=" << pole_final << " |cart|=" << cart_final
            << "  max|vel_est-vel_true|(post-warmup)=" << max_est_err
            << "  needs_velocity_state=" << lqg.capabilities().needs_velocity_state << "\n";

  const bool balanced = pole_final < 1e-2 && cart_final < 1e-2;
  const bool estimator_tracks = max_est_err < 5e-2;
  const bool ok = no_vel_needed && balanced && estimator_tracks;
  std::cout << (ok ? "PASS" : "FAIL") << "  (no_vel_needed=" << no_vel_needed
            << " balanced=" << balanced << " estimator_tracks=" << estimator_tracks << ")\n";
  return ok ? 0 : 1;
}
