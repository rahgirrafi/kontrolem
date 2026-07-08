// LqrController correctness on the cart-pole:
//   1. synthesize solves CARE -> the closed loop A - B_act K is STABLE
//      (all eigenvalues have negative real part). This is the LQR analog of the
//      model's finite-difference test.
//   2. compute() produces a finite command via the gemv law.
//   3. status() flags when the state leaves the validity region.
// Dependency-free (no gtest), consistent with the slice.
#include <iostream>

#include <Eigen/Dense>

#include "kontrolem_controllers/lqr_controller.hpp"
#include "kontrolem_model/robot_model.hpp"

using namespace kontrolem_control;
using namespace kontrolem_controllers;
using kontrolem_model::RobotModel;

int main()
{
  const RobotModel model = RobotModel::from_urdf_file(CART_POLE_URDF);
  const int nv = model.nv();

  // Weight the pole angle (x index 1) heavily; cheap control.
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2 * nv, 2 * nv);
  Q(1, 1) = 10.0;
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);

  LqrController lqr({"cart_joint"}, Q, R, /*q_dev_max=*/0.5);

  Regulation upright;
  upright.q_ref = Eigen::VectorXd::Zero(model.nq());  // cart 0, pole up (angle 0)
  upright.v_ref = Eigen::VectorXd::Zero(nv);

  if (!accepts(lqr, upright)) {
    std::cout << "FAIL: LQR should accept Regulation\n";
    return 1;
  }

  const auto synthesis = lqr.synthesize(model, upright);
  lqr.configure(model, *synthesis, upright);
  const Eigen::MatrixXd K = lqr.gain();
  std::cout << "K = " << K << "\n";

  // (1) Closed-loop stability of A - B_act K.
  const auto lin =
    model.linearize(upright.q_ref, Eigen::VectorXd::Zero(nv), model.gravity_torque(upright.q_ref));
  const Eigen::MatrixXd B_act = lin.B.col(0);  // cart = v-index 0
  const Eigen::MatrixXd Acl = lin.A - B_act * K;
  const Eigen::VectorXcd eig = Acl.eigenvalues();
  const double max_re = eig.real().maxCoeff();
  std::cout << "open-loop poles (max Re " << lin.A.eigenvalues().real().maxCoeff() << "):\n"
            << lin.A.eigenvalues().transpose() << "\n";
  std::cout << "closed-loop poles (max Re " << max_re << "):\n" << eig.transpose() << "\n";

  // (2) A command at a small tilt.
  State tilt;
  tilt.q = Eigen::VectorXd::Zero(model.nq());
  tilt.q[1] = 0.1;  // pole tilted +0.1 rad
  tilt.v = Eigen::VectorXd::Zero(nv);
  const Command & u = lqr.compute(tilt, upright, 0.001);
  const Status & st = lqr.status();
  std::cout << "u(pole=+0.1) = " << u.tau.transpose()
            << "   status.ok=" << st.ok << " margin=" << st.margin << "\n";

  // (3) Out-of-region status.
  State big;
  big.q = Eigen::VectorXd::Zero(model.nq());
  big.q[1] = 1.0;  // way outside q_dev_max
  big.v = Eigen::VectorXd::Zero(nv);
  lqr.compute(big, upright, 0.001);
  const bool region_flagged = !lqr.status().ok;

  const bool stable = max_re < -1e-9;
  const bool u_finite = u.tau.allFinite() && u.tau.size() == 1;
  const bool ok = stable && u_finite && region_flagged;
  std::cout << (ok ? "PASS" : "FAIL") << "  (stable=" << stable << " u_finite=" << u_finite
            << " region_flagged=" << region_flagged << ")\n";
  return ok ? 0 : 1;
}
