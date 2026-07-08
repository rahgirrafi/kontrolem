// Continuous-time Algebraic Riccati Equation (CARE) solver, Eigen-only.
//
// Solves  A^T P + P A - P B R^-1 B^T P + Q = 0  for the stabilizing P = P^T >= 0
// via the Hamiltonian-eigenvector method (Laub, "A Schur Method for Solving
// Algebraic Riccati Equations", IEEE TAC 1979): the stabilizing P spans the
// stable invariant subspace of the Hamiltonian
//     H = [[ A,        -B R^-1 B^T ],
//          [ -Q,       -A^T        ]].
// Adequate for the small systems here; behind a seam so it can be swapped for
// Slycot/SciPy if a larger/stiffer problem needs the extra numerical care.
#ifndef KONTROLEM_CONTROLLERS__CARE_HPP_
#define KONTROLEM_CONTROLLERS__CARE_HPP_

#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

namespace kontrolem_controllers
{

/// Stabilizing solution P of the CARE.
inline Eigen::MatrixXd solve_care(
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & B, const Eigen::MatrixXd & Q,
  const Eigen::MatrixXd & R)
{
  const int n = static_cast<int>(A.rows());
  const Eigen::MatrixXd Rinv = R.inverse();

  Eigen::MatrixXd H(2 * n, 2 * n);
  H.topLeftCorner(n, n) = A;
  H.topRightCorner(n, n) = -B * Rinv * B.transpose();
  H.bottomLeftCorner(n, n) = -Q;
  H.bottomRightCorner(n, n) = -A.transpose();

  Eigen::EigenSolver<Eigen::MatrixXd> es(H);
  // Collect the n eigenvectors of the stable (negative real part) eigenvalues.
  Eigen::MatrixXcd stable(2 * n, n);
  int col = 0;
  for (int i = 0; i < 2 * n; ++i) {
    if (es.eigenvalues()[i].real() < 0.0) {
      if (col < n) {
        stable.col(col) = es.eigenvectors().col(i);
      }
      ++col;
    }
  }
  if (col != n) {
    throw std::runtime_error(
      "solve_care: found " + std::to_string(col) +
      " stable eigenvalues, expected " + std::to_string(n) +
      " (system not stabilizable, or Hamiltonian has eigenvalues on the axis)");
  }

  const Eigen::MatrixXcd U1 = stable.topRows(n);
  const Eigen::MatrixXcd U2 = stable.bottomRows(n);
  Eigen::MatrixXd P = (U2 * U1.inverse()).real();
  return 0.5 * (P + P.transpose());  // symmetrize away numerical drift
}

/// LQR gain K = R^-1 B^T P for the stabilizing P; control law u = -K x.
inline Eigen::MatrixXd lqr_gain(
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & B, const Eigen::MatrixXd & Q,
  const Eigen::MatrixXd & R)
{
  return R.inverse() * B.transpose() * solve_care(A, B, Q, R);
}

/// Stabilizing solution P of the Discrete Algebraic Riccati Equation
///   P = A^T P A - A^T P B (R + B^T P B)^-1 B^T P A + Q,
/// by the standard fixed-point iteration (converges for a stabilizable,
/// detectable discrete pair). Used as the MPC terminal cost so a finite horizon
/// inherits infinite-horizon (discrete-LQR) stability. `A`, `B` are the
/// DISCRETE-time matrices.
inline Eigen::MatrixXd solve_dare(
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & B, const Eigen::MatrixXd & Q,
  const Eigen::MatrixXd & R, int max_iter = 2000, double tol = 1e-10)
{
  Eigen::MatrixXd P = Q;
  for (int i = 0; i < max_iter; ++i) {
    const Eigen::MatrixXd BtP = B.transpose() * P;
    const Eigen::MatrixXd S = R + BtP * B;                       // m x m
    const Eigen::MatrixXd K = S.ldlt().solve(BtP * A);           // m x n
    Eigen::MatrixXd Pn = Q + A.transpose() * P * A - A.transpose() * P * B * K;
    Pn = 0.5 * (Pn + Pn.transpose());                            // symmetrize
    if ((Pn - P).norm() <= tol * (1.0 + P.norm())) {
      return Pn;
    }
    P = Pn;
  }
  return P;
}

}  // namespace kontrolem_controllers

#endif  // KONTROLEM_CONTROLLERS__CARE_HPP_
