// Contact-Jacobian correctness (M3.2): the translational foot Jacobian J (3 x nv)
// must equal the finite-difference of the foot's world position w.r.t. a manifold
// (SE(3)-correct) perturbation of q:  J.col(i) ~ [p(q ⊕ eps·e_i) - p(q)] / eps.
// This is the contact-frame analog of the linearize finite-difference test, and
// it exercises the floating base (perturbing the 6-DoF root tangent too).
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "kontrolem_model/robot_model.hpp"

using kontrolem_model::BaseType;
using kontrolem_model::RobotModel;

int main()
{
  const RobotModel m = RobotModel::from_urdf_file(FLOATING_URDF, BaseType::kFloating);
  const int nv = m.nv();
  const std::string foot = "foot_left";

  // A non-trivial configuration: rotate/translate the base and bend the hips.
  Eigen::VectorXd q = m.neutral();
  Eigen::VectorXd v0 = Eigen::VectorXd::Zero(nv);
  v0(0) = 0.2; v0(1) = -0.1; v0(2) = 0.05;  // base translation
  v0(3) = 0.1; v0(4) = 0.2; v0(5) = -0.15;  // base rotation
  v0(6) = 0.3; v0(7) = -0.4;                // hips
  q = m.integrate(q, v0, 1.0);              // move off neutral, staying on-manifold

  const Eigen::MatrixXd J = m.contact_jacobian(q, foot);
  const bool shape_ok = (J.rows() == 3) && (J.cols() == nv);

  const double eps = 1e-6;
  const Eigen::Vector3d p0 = m.frame_position(q, foot);
  Eigen::MatrixXd J_fd(3, nv);
  for (int i = 0; i < nv; ++i) {
    Eigen::VectorXd e = Eigen::VectorXd::Zero(nv);
    e(i) = 1.0;
    const Eigen::VectorXd qp = m.integrate(q, e, eps);   // manifold step in direction i
    J_fd.col(i) = (m.frame_position(qp, foot) - p0) / eps;
  }

  const double err = (J - J_fd).cwiseAbs().maxCoeff();
  std::cout << "J shape " << J.rows() << "x" << J.cols() << "  max|J - J_fd| = " << err << "\n";

  const bool ok = shape_ok && (err < 1e-5);
  std::cout << (ok ? "PASS" : "FAIL") << "  (shape=" << shape_ok << " fd_err<1e-5="
            << (err < 1e-5) << ")\n";
  return ok ? 0 : 1;
}
