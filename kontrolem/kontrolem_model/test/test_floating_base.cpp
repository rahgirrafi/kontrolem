// Floating-base model-service checks (M3.1), all against KNOWN quantities:
//   1. building with a free-flyer root gives nq == nv + 1 (SE(3) manifold).
//   2. neutral() has a UNIT quaternion (a zero vector is not a valid config).
//   3. center_of_mass at neutral matches the hand-computed mass-weighted average.
//   4. integrate() is manifold-correct: a pure base rotation keeps the quaternion
//      unit and matches the closed-form exp — where a naive q + v·dt would NOT.
// Dependency-free (no gtest).
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "kontrolem_model/robot_model.hpp"

using kontrolem_model::BaseType;
using kontrolem_model::RobotModel;

int main()
{
  const RobotModel m = RobotModel::from_urdf_file(FLOATING_URDF, BaseType::kFloating);

  // (1) dimensions: 7 (SE(3)) + 2 hips = 9;  6 + 2 = 8.
  const int nq = m.nq(), nv = m.nv();
  const bool dims_ok = (nq == 9) && (nv == 8) && (nq == nv + 1);

  // (2) neutral: quaternion (q[3..6] = x,y,z,w for the free-flyer) is unit [0,0,0,1].
  const Eigen::VectorXd q0 = m.neutral();
  const Eigen::Vector4d quat0 = q0.segment<4>(3);
  const bool neutral_ok = (q0.size() == nq) &&
                          std::abs(quat0.norm() - 1.0) < 1e-9 &&
                          (quat0 - Eigen::Vector4d(0, 0, 0, 1)).norm() < 1e-9;

  // (3) CoM at neutral (legs straight down). Hand computation:
  //   mass = 5 + 2*0.5 + 2*0.05 = 6.1
  //   z = (5*0 + 2*(0.5*-0.15) + 2*(0.05*-0.3)) / 6.1 = -0.18/6.1
  const Eigen::Vector3d com = m.center_of_mass(q0);
  const double z_expected = -0.18 / 6.1;
  const bool com_ok = std::abs(com.x()) < 1e-9 && std::abs(com.y()) < 1e-9 &&
                      std::abs(com.z() - z_expected) < 1e-6;

  // (4) manifold integrate: pure base yaw rate wz for dt s.t. wz*dt = pi/2.
  Eigen::VectorXd v = Eigen::VectorXd::Zero(nv);
  v(5) = 1.0;               // angular-z of the free-flyer twist
  const double dt = M_PI / 2.0;  // total rotation pi/2 about z
  const Eigen::VectorXd q1 = m.integrate(q0, v, dt);
  const Eigen::Vector4d quat1 = q1.segment<4>(3);
  const Eigen::Vector4d quat_expected(0, 0, std::sin(M_PI / 4), std::cos(M_PI / 4));
  const bool rot_ok = (quat1 - quat_expected).norm() < 1e-6 &&
                      std::abs(quat1.norm() - 1.0) < 1e-9;
  // Contrast: the naive Euclidean update corrupts the quaternion (norm != 1).
  const Eigen::VectorXd q_naive = q0 + v * dt;
  const bool naive_breaks = std::abs(q_naive.segment<4>(3).norm() - 1.0) > 1e-3;

  std::cout << "nq=" << nq << " nv=" << nv << "  CoM=(" << com.transpose() << ")"
            << " expected_z=" << z_expected << "\n";
  std::cout << "quat after pi/2 yaw = (" << quat1.transpose() << ")\n";

  const bool ok = dims_ok && neutral_ok && com_ok && rot_ok && naive_breaks;
  std::cout << (ok ? "PASS" : "FAIL") << "  (dims=" << dims_ok << " neutral=" << neutral_ok
            << " com=" << com_ok << " rot=" << rot_ok << " naive_breaks=" << naive_breaks << ")\n";
  return ok ? 0 : 1;
}
