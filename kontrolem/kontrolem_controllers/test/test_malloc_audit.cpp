// Option (a): malloc-level allocation audit.
//
// The operator-new audit (test_allocation) proved the C++ side of compute() is
// allocation-free, but it cannot see OSQP, which allocates through C malloc.
// Here we interpose malloc/calloc/realloc (delegating to glibc's __libc_*,
// robust on modern glibc where __malloc_hook is gone) and count allocations
// during the per-tick compute() loop. This closes the last blind spot in the
// real-time story: does the warm-started OSQP solve allocate every tick?
//
// LQR (no solver) must be 0. QP is the actual measurement.
#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "kontrolem_controllers/lqr_controller.hpp"
#include "kontrolem_controllers/mpc_controller.hpp"
#include "kontrolem_controllers/qp_task_space_controller.hpp"
#include "kontrolem_controllers/wbc_controller.hpp"
#include "kontrolem_model/robot_model.hpp"

extern "C" {
void * __libc_malloc(std::size_t);
void * __libc_calloc(std::size_t, std::size_t);
void * __libc_realloc(void *, std::size_t);
void __libc_free(void *);
}

namespace
{
std::atomic<long> g_mallocs{0};
std::atomic<bool> g_on{false};
inline void tick()
{
  if (g_on.load(std::memory_order_relaxed)) {
    g_mallocs.fetch_add(1, std::memory_order_relaxed);
  }
}
}  // namespace

extern "C" void * malloc(std::size_t n)
{
  tick();
  return __libc_malloc(n);
}
extern "C" void * calloc(std::size_t a, std::size_t b)
{
  tick();
  return __libc_calloc(a, b);
}
extern "C" void * realloc(void * p, std::size_t n)
{
  tick();
  return __libc_realloc(p, n);
}
extern "C" void free(void * p) { __libc_free(p); }

using namespace kontrolem_control;
using namespace kontrolem_controllers;
using kontrolem_model::RobotModel;

static long count_mallocs(Controller & c, const State & s, const Regulation & r, int n)
{
  g_mallocs = 0;
  g_on = true;
  for (int i = 0; i < n; ++i) {
    c.compute(s, r, 0.001);
  }
  g_on = false;
  return g_mallocs.load();
}

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

  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2 * nv, 2 * nv);
  Q(1, 1) = 10.0;
  LqrController lqr({"cart_joint"}, Q, Eigen::MatrixXd::Identity(1, 1));
  lqr.configure(model, *lqr.synthesize(model, upright), upright);

  Eigen::VectorXd W(nv);
  W << 1.0, 10.0;
  QpTaskSpaceController qp({"cart_joint"}, W, 50.0, 10.0, 5.0);
  qp.configure(model, *qp.synthesize(model, upright), upright);

  // MPC: a larger online QP (N*m = 30 decision vars) through the same seam.
  Eigen::MatrixXd Qm = Eigen::Vector4d(1.0, 10.0, 1.0, 1.0).asDiagonal();
  MpcController mpc({"cart_joint"}, Qm, Eigen::MatrixXd::Constant(1, 1, 0.1),
                    /*horizon=*/30, /*dt_mpc=*/0.02, /*tau_max=*/6.0);
  mpc.configure(model, *mpc.synthesize(model, upright), upright);

  // WBC: the floating-base whole-body QP (the flagship RT path). Build the quad
  // standing configuration and a matching Regulation.
  const RobotModel quad =
    RobotModel::from_urdf_file(FLOATING_QUAD_URDF, kontrolem_model::BaseType::kFloating);
  const std::vector<std::string> feet = {"foot_FL", "foot_FR", "foot_RL", "foot_RR"};
  std::vector<std::string> actuated;
  for (const std::string leg : {"FL", "FR", "RL", "RR"}) {
    actuated.push_back("hipx_" + leg);
    actuated.push_back("hipy_" + leg);
    actuated.push_back("knee_" + leg);
  }
  Eigen::VectorXd q_stand = quad.neutral();
  for (const std::string leg : {"FL", "FR", "RL", "RR"}) {
    q_stand(quad.joint_q_index("hipy_" + leg)) = 0.7;
    q_stand(quad.joint_q_index("knee_" + leg)) = -1.4;
  }
  double min_fz = 1e9;
  for (const auto & f : feet) min_fz = std::min(min_fz, quad.frame_position(q_stand, f).z());
  q_stand(2) = -min_fz;
  Regulation stand;
  stand.q_ref = q_stand;
  stand.v_ref = Eigen::VectorXd::Zero(quad.nv());
  State sq;
  sq.q = q_stand;
  sq.v = Eigen::VectorXd::Zero(quad.nv());
  WbcController wbc(feet, actuated, WbcController::Gains{});
  wbc.configure(quad, *wbc.synthesize(quad, stand), stand);

  const int N = 1000;
  for (int i = 0; i < 20; ++i) {  // warm up past any lazy first-solve init
    lqr.compute(s, upright, 0.001);
    qp.compute(s, upright, 0.001);
    mpc.compute(s, upright, 0.001);
    wbc.compute(sq, stand, 0.001);
  }

  const long lqr_m = count_mallocs(lqr, s, upright, N);
  const long qp_m = count_mallocs(qp, s, upright, N);
  const long mpc_m = count_mallocs(mpc, s, upright, N);
  const long wbc_m = count_mallocs(wbc, sq, stand, N);

  std::cout << "compute() over " << N << " calls (malloc/calloc/realloc count):\n";
  std::cout << "  LqrController          : " << lqr_m << "  (" << (double)lqr_m / N << "/call)\n";
  std::cout << "  QpTaskSpaceController  : " << qp_m << "  (" << (double)qp_m / N
            << "/call)  <- includes OSQP's C allocations\n";
  std::cout << "  MpcController          : " << mpc_m << "  (" << (double)mpc_m / N
            << "/call)  <- 30-var condensed QP, same OSQP seam\n";
  std::cout << "  WbcController          : " << wbc_m << "  (" << (double)wbc_m / N
            << "/call)  <- floating-base 42-var QP + contact queries\n";

  // All paths must be malloc-free. QP/MPC/WBC reaching 0 depends on the
  // control-loop OSQP config (polish=0, adaptive_rho=0 in qp_solver.cpp) AND, for
  // the WBC, on caching contact frame indices (a per-tick getFrameId name lookup
  // allocates) + the in-place difference() overload. This assertion guards against
  // regressing any of those. NOTE: 0 allocation is necessary but not sufficient
  // for hard-RT — OSQP's iteration count is still data-dependent, so a hard-RT
  // deployment must also cap max_iter. That is a separate, later concern.
  const bool lqr_clean = (lqr_m == 0);
  const bool qp_clean = (qp_m == 0);
  const bool mpc_clean = (mpc_m == 0);
  const bool wbc_clean = (wbc_m == 0);
  const bool ok = lqr_clean && qp_clean && mpc_clean && wbc_clean;
  std::cout << (ok ? "PASS" : "FAIL") << ": LQR malloc-free=" << lqr_clean
            << " QP malloc-free=" << qp_clean << " MPC malloc-free=" << mpc_clean
            << " WBC malloc-free=" << wbc_clean << "\n";
  return ok ? 0 : 1;
}
