// Correctness check for RobotModel::linearize: the analytic (A, B) from
// Pinocchio's computeABADerivatives must match a central finite difference of
// f(x, tau) = [v; aba(q, v, tau)] about a non-trivial operating point.
//
// This is the "linearize-vs-finite-difference" test named in the v2 plan. It is
// dependency-free (no gtest) to keep the core slice minimal.
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "kontrolem_model/robot_model.hpp"

using kontrolem_model::RobotModel;

int main()
{
  const RobotModel model = RobotModel::from_urdf_file(CART_POLE_URDF);
  const int nq = model.nq();
  const int nv = model.nv();

  std::cout << "cart_pole model: nq=" << nq << " nv=" << nv << " joints:";
  for (const auto & n : model.joint_names()) std::cout << " " << n;
  std::cout << "\n";

  if (nq != nv) {
    std::cout << "FAIL: this FD test assumes a Euclidean config (nq==nv)\n";
    return 1;
  }

  // Non-trivial operating point so every block of A/B is exercised.
  Eigen::VectorXd q(nq), v(nv), tau(nv);
  q << 0.30, 0.20;    // cart position [m], pole angle [rad]
  v << -0.10, 0.40;   // cart velocity, pole angular velocity
  tau << 0.50, 0.00;  // force on the cart, passive pole

  const auto lin = model.linearize(q, v, tau);

  // f(x, tau) = [v; aba(q, v, tau)],  x = [q; v].
  auto f = [&](const Eigen::VectorXd & qq, const Eigen::VectorXd & vv,
               const Eigen::VectorXd & uu) {
    Eigen::VectorXd out(2 * nv);
    out.head(nv) = vv;
    out.tail(nv) = model.aba(qq, vv, uu);
    return out;
  };

  const double eps = 1e-6;
  Eigen::MatrixXd A_fd(2 * nv, 2 * nv), B_fd(2 * nv, nv);
  for (int j = 0; j < nv; ++j) {  // d/dq_j  (Euclidean since nq==nv)
    Eigen::VectorXd qp = q, qm = q;
    qp[j] += eps;
    qm[j] -= eps;
    A_fd.col(j) = (f(qp, v, tau) - f(qm, v, tau)) / (2 * eps);
  }
  for (int j = 0; j < nv; ++j) {  // d/dv_j
    Eigen::VectorXd vp = v, vm = v;
    vp[j] += eps;
    vm[j] -= eps;
    A_fd.col(nv + j) = (f(q, vp, tau) - f(q, vm, tau)) / (2 * eps);
  }
  for (int k = 0; k < nv; ++k) {  // d/dtau_k
    Eigen::VectorXd up = tau, um = tau;
    up[k] += eps;
    um[k] -= eps;
    B_fd.col(k) = (f(q, v, up) - f(q, v, um)) / (2 * eps);
  }

  const double errA = (lin.A - A_fd).cwiseAbs().maxCoeff();
  const double errB = (lin.B - B_fd).cwiseAbs().maxCoeff();

  Eigen::IOFormat fmt(6, 0, ", ", "\n", "  [", "]");
  std::cout << "\nA (analytic):\n" << lin.A.format(fmt) << "\n";
  std::cout << "\nB (analytic):\n" << lin.B.format(fmt) << "\n";
  std::cout << "\nmax|A - A_fd| = " << errA << "\n";
  std::cout << "max|B - B_fd| = " << errB << "\n";

  const double tol = 1e-5;
  const bool ok = (errA < tol) && (errB < tol);
  std::cout << "\n" << (ok ? "PASS" : "FAIL") << " (tol=" << tol << ")\n";
  return ok ? 0 : 1;
}
