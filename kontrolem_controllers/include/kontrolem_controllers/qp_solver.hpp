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
  /// `eps` sets OSQP's absolute+relative tolerance (default 1e-6 — crisp
  /// constraint satisfaction for the small task-space QP). Looser callers: the
  /// WBC (1e-4) solves for contact forces in newtons so stiff transients still
  /// hit OSQP_SOLVED; the condensed MPC (1e-4) because at 1e-6 the receding-
  /// horizon QP needs a pathological iteration count under fixed rho (chosen for
  /// allocation-freedom) for no control benefit — only u0 is applied and the
  /// problem is re-solved next tick. Looser eps keeps the iteration count bounded
  /// well under the max_iter cap.
  /// `max_iter` caps OSQP's ADMM iterations. With polish and adaptive-rho off,
  /// every iteration does identical fixed work, so this cap is a hard bound on
  /// per-tick solve time — the last piece of the hard-real-time story. A solve
  /// that hits the cap returns OSQP_MAX_ITER_REACHED, so solved() is false and
  /// the Supervisor falls back rather than blowing the deadline. Pick it well
  /// above the nominal iteration count (see iterations()); the default is OSQP's
  /// own 4000, i.e. effectively uncapped — callers on the RT path pass a real
  /// bound.
  void setup(
    const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const Eigen::VectorXd & q,
    const Eigen::VectorXd & l, const Eigen::VectorXd & u, double eps = 1e-6,
    int max_iter = 4000);

  /// Warm-started re-solve with new q, l, u and A values (same sparsity as
  /// setup; P unchanged). Returns the solution z (reference to internal buffer).
  const Eigen::VectorXd & solve(
    const Eigen::VectorXd & q, const Eigen::VectorXd & l, const Eigen::VectorXd & u,
    const Eigen::MatrixXd & A);

  bool solved() const;

  /// ADMM iterations the last solve() actually took. Bounded above by the
  /// setup() cap; surfaced into Status/diagnostics so the RT margin (nominal
  /// count vs. cap) is observable at runtime.
  int iterations() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace kontrolem_controllers

#endif  // KONTROLEM_CONTROLLERS__QP_SOLVER_HPP_
