// Step-5 allocation audit (see DEVELOPMENT U4) — now a regression guard.
//
// Counts C++ heap allocations inside compute() by interposing operator new for
// the whole process. Both control paths must be allocation-free:
//   * LqrController::compute        -> a gemv (0 allocs).
//   * QpTaskSpaceController::compute -> queries the model via a reusable
//     Workspace and solves the QP into preallocated buffers (0 allocs). This
//     path allocated 68/tick before the U4 fix (fresh Pinocchio Data per call);
//     the fix was demonstrated by this very test, then applied.
//
// Caveat stated honestly: this hook counts C++ `operator new` only. OSQP
// allocates through C malloc (not counted here); its warm-started solve is
// designed to reuse its workspace. A fuller audit would add a malloc-level hook
// / heaptrack to also quantify OSQP.
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <new>

#include <Eigen/Dense>

#include "kontrolem_controllers/lqr_controller.hpp"
#include "kontrolem_controllers/qp_task_space_controller.hpp"
#include "kontrolem_model/robot_model.hpp"

namespace
{
std::atomic<long> g_allocs{0};
std::atomic<bool> g_on{false};
}  // namespace

void * operator new(std::size_t n)
{
  if (g_on.load()) {
    g_allocs.fetch_add(1);
  }
  void * p = std::malloc(n ? n : 1);
  if (!p) {
    throw std::bad_alloc();
  }
  return p;
}
void * operator new[](std::size_t n) { return ::operator new(n); }
void operator delete(void * p) noexcept { std::free(p); }
void operator delete(void * p, std::size_t) noexcept { std::free(p); }
void operator delete[](void * p) noexcept { std::free(p); }
void operator delete[](void * p, std::size_t) noexcept { std::free(p); }

using namespace kontrolem_control;
using namespace kontrolem_controllers;
using kontrolem_model::RobotModel;

int main()
{
  const RobotModel model = RobotModel::from_urdf_file(CART_POLE_URDF);
  const int nv = model.nv();

  Regulation upright;
  upright.q_ref = Eigen::VectorXd::Zero(model.nq());
  upright.v_ref = Eigen::VectorXd::Zero(nv);

  State s;
  s.q = Eigen::VectorXd::Zero(model.nq());
  s.q[1] = 0.05;
  s.v = Eigen::VectorXd::Zero(nv);

  // LQR.
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2 * nv, 2 * nv);
  Q(1, 1) = 10.0;
  LqrController lqr({"cart_joint"}, Q, Eigen::MatrixXd::Identity(1, 1));
  lqr.configure(model, *lqr.synthesize(model, upright), upright);

  // QP.
  Eigen::VectorXd W(nv);
  W << 1.0, 10.0;
  QpTaskSpaceController qp({"cart_joint"}, W, 50.0, 10.0, /*tau_max=*/5.0);
  qp.configure(model, *qp.synthesize(model, upright), upright);

  const int N = 1000;
  // Warm up (first-call lazy init outside the count).
  for (int i = 0; i < 5; ++i) {
    lqr.compute(s, upright, 0.001);
    qp.compute(s, upright, 0.001);
  }

  g_allocs = 0;
  g_on = true;
  for (int i = 0; i < N; ++i) {
    lqr.compute(s, upright, 0.001);
  }
  g_on = false;
  const long lqr_allocs = g_allocs.load();

  g_allocs = 0;
  g_on = true;
  for (int i = 0; i < N; ++i) {
    qp.compute(s, upright, 0.001);
  }
  g_on = false;
  const long qp_allocs = g_allocs.load();

  std::cout << "compute() over " << N << " calls (C++ operator new count):\n";
  std::cout << "  LqrController          : " << lqr_allocs << " allocs  ("
            << (double)lqr_allocs / N << "/call)\n";
  std::cout << "  QpTaskSpaceController  : " << qp_allocs << " allocs  ("
            << (double)qp_allocs / N << "/call)  <- reusable Workspace (post-U4-fix)\n";

  // Both control paths must be allocation-free (C++ new). Regression guard.
  const bool lqr_clean = (lqr_allocs == 0);
  const bool qp_clean = (qp_allocs == 0);
  const bool ok = lqr_clean && qp_clean;
  std::cout << (ok ? "PASS" : "FAIL") << ": LQR clean=" << lqr_clean
            << " QP clean=" << qp_clean << "\n";
  return ok ? 0 : 1;
}
