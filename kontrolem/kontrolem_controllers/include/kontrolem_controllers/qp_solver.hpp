// Thin wrapper over a QP solver, behind a seam. Currently OSQP 0.6.x; the
// solver (and its C headers) are hidden behind a pImpl so this header — and
// anything that includes it — stays Eigen-only and swappable (ProxQP later).
//
// Solves:  min 0.5 z^T P z + q^T z   s.t.  l <= A z <= u.
//   setup() : once, allocates the solver workspace (NOT real-time).
//   solve() : re-solve warm-started with updated q, l, u, A (P held constant,
//             same sparsity). This is the per-tick call.
#ifndef KONTROLEM_CONTROLLERS__QP_SOLVER_HPP_
#define KONTROLEM_CONTROLLERS__QP_SOLVER_HPP_

#include <memory>

#include <Eigen/Dense>

namespace kontrolem_controllers
{

class QpSolver
{
public:
  QpSolver();
  ~QpSolver();
  QpSolver(const QpSolver &) = delete;
  QpSolver & operator=(const QpSolver &) = delete;

  /// P (nz x nz, symmetric — upper triangle used), A (nc x nz). Allocates.
  void setup(
    const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const Eigen::VectorXd & q,
    const Eigen::VectorXd & l, const Eigen::VectorXd & u);

  /// Warm-started re-solve with new q, l, u and A values (same sparsity as
  /// setup; P unchanged). Returns the solution z (reference to internal buffer).
  const Eigen::VectorXd & solve(
    const Eigen::VectorXd & q, const Eigen::VectorXd & l, const Eigen::VectorXd & u,
    const Eigen::MatrixXd & A);

  bool solved() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace kontrolem_controllers

#endif  // KONTROLEM_CONTROLLERS__QP_SOLVER_HPP_
