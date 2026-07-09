// OSQP 0.6.x backend for the QpSolver seam. Dense matrices are stored in CSC
// (A fully dense; P upper-triangular dense) — trivial at the small sizes here,
// and keeps the interop simple and correct. The workspace is built once in
// setup(); solve() only updates values and warm-solves.
#include "kontrolem_controllers/qp_solver.hpp"

#include <stdexcept>
#include <vector>

#include <osqp.h>

namespace kontrolem_controllers
{

struct QpSolver::Impl
{
  int nz = 0;
  int nc = 0;
  // Persistent CSC storage (OSQP references A_x on updates).
  std::vector<c_int> P_p, P_i, A_p, A_i;
  std::vector<c_float> P_x, A_x, qv, lv, uv;
  csc * P_csc = nullptr;
  csc * A_csc = nullptr;
  OSQPWorkspace * work = nullptr;
  Eigen::VectorXd x;
  bool solved = false;
  int iters = 0;

  ~Impl()
  {
    if (work) {
      osqp_cleanup(work);
    }
    if (P_csc) {
      c_free(P_csc);
    }
    if (A_csc) {
      c_free(A_csc);
    }
  }

  void build_upper_csc(const Eigen::MatrixXd & P)
  {
    P_p.assign(nz + 1, 0);
    P_i.clear();
    P_x.clear();
    for (int j = 0; j < nz; ++j) {
      for (int i = 0; i <= j; ++i) {  // upper triangle, column-major
        P_i.push_back(i);
        P_x.push_back(static_cast<c_float>(P(i, j)));
      }
      P_p[j + 1] = static_cast<c_int>(P_i.size());
    }
  }

  void build_full_csc(const Eigen::MatrixXd & A)
  {
    A_p.assign(nz + 1, 0);
    A_i.clear();
    A_x.clear();
    for (int j = 0; j < nz; ++j) {
      for (int i = 0; i < nc; ++i) {  // dense, column-major
        A_i.push_back(i);
        A_x.push_back(static_cast<c_float>(A(i, j)));
      }
      A_p[j + 1] = static_cast<c_int>(A_i.size());
    }
  }

  void refill_A(const Eigen::MatrixXd & A)  // values only, no allocation
  {
    std::size_t k = 0;
    for (int j = 0; j < nz; ++j) {
      for (int i = 0; i < nc; ++i) {
        A_x[k++] = static_cast<c_float>(A(i, j));
      }
    }
  }
};

QpSolver::QpSolver() : impl_(std::make_unique<Impl>()) {}
QpSolver::~QpSolver() = default;

void QpSolver::setup(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const Eigen::VectorXd & q,
  const Eigen::VectorXd & l, const Eigen::VectorXd & u, double eps, int max_iter)
{
  auto & im = *impl_;
  im.nz = static_cast<int>(P.rows());
  im.nc = static_cast<int>(A.rows());
  im.build_upper_csc(P);
  im.build_full_csc(A);
  im.qv.assign(q.data(), q.data() + q.size());
  im.lv.assign(l.data(), l.data() + l.size());
  im.uv.assign(u.data(), u.data() + u.size());
  im.x.setZero(im.nz);

  im.P_csc = csc_matrix(
    im.nz, im.nz, static_cast<c_int>(im.P_x.size()), im.P_x.data(), im.P_i.data(), im.P_p.data());
  im.A_csc = csc_matrix(
    im.nc, im.nz, static_cast<c_int>(im.A_x.size()), im.A_x.data(), im.A_i.data(), im.A_p.data());

  OSQPData data;
  data.n = im.nz;
  data.m = im.nc;
  data.P = im.P_csc;
  data.A = im.A_csc;
  data.q = im.qv.data();
  data.l = im.lv.data();
  data.u = im.uv.data();

  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.warm_start = 1;
  settings.eps_abs = eps;       // tolerance -> constraint satisfaction (caller-tuned)
  settings.eps_rel = eps;
  // Control-loop config: no polish (it factorizes a reduced KKT each solve) and
  // fixed rho (adaptive rho refactorizes the KKT) -> far less per-tick malloc.
  settings.polish = 0;
  settings.adaptive_rho = 0;
  // Hard bound on ADMM iterations. With polish/adaptive-rho off each iteration
  // costs the same, so this bounds per-tick solve time; hitting it yields
  // OSQP_MAX_ITER_REACHED (solved() == false) and a Supervisor fallback.
  settings.max_iter = static_cast<c_int>(max_iter);

  const c_int ret = osqp_setup(&im.work, &data, &settings);
  if (ret != 0 || im.work == nullptr) {
    throw std::runtime_error("QpSolver: osqp_setup failed (code " + std::to_string(ret) + ")");
  }
}

const Eigen::VectorXd & QpSolver::solve(
  const Eigen::VectorXd & q, const Eigen::VectorXd & l, const Eigen::VectorXd & u,
  const Eigen::MatrixXd & A)
{
  auto & im = *impl_;
  im.refill_A(A);
  for (int i = 0; i < im.nc; ++i) {
    im.lv[static_cast<std::size_t>(i)] = static_cast<c_float>(l[i]);
    im.uv[static_cast<std::size_t>(i)] = static_cast<c_float>(u[i]);
  }
  for (int i = 0; i < im.nz; ++i) {
    im.qv[static_cast<std::size_t>(i)] = static_cast<c_float>(q[i]);
  }
  osqp_update_A(im.work, im.A_x.data(), OSQP_NULL, static_cast<c_int>(im.A_x.size()));
  osqp_update_lin_cost(im.work, im.qv.data());
  osqp_update_bounds(im.work, im.lv.data(), im.uv.data());
  osqp_solve(im.work);
  im.solved = (im.work->info->status_val == OSQP_SOLVED);
  im.iters = static_cast<int>(im.work->info->iter);
  for (int i = 0; i < im.nz; ++i) {
    im.x[i] = im.work->solution->x[i];
  }
  return im.x;
}

bool QpSolver::solved() const { return impl_->solved; }

int QpSolver::iterations() const { return impl_->iters; }

}  // namespace kontrolem_controllers
