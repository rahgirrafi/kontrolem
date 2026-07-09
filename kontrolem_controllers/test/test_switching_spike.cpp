// Heterogeneous bumpless-transfer spike (Part D risk D1, prototype-first D4 #4).
//
// The plan's TOP research risk (T3): the Supervisor is supposed to hand control
// between whole controllers at runtime, but "initialize the incoming controller
// to match the outgoing output" is ill-defined across DIFFERENT representations.
// The sharpest case is a stateful LQG compensator (carries an observer estimate)
// vs a stateless MPC (reads the full state each tick). This spike measures — OFF
// ROS, on the cart-pole — the command discontinuity the actuator actually sees at
// the switch instant, and whether the switch stays stable, under two policies:
//
//   NAIVE     : reactivate LQG with whatever observer state it was left with
//               (a Supervisor that just re-enables a dormant controller) — the
//               estimate is now STALE, so the compensator fights the real state.
//   BUMPLESS  : seed the observer from the current state on activation
//               (LqgController::seed_from_state) so it starts already-converged.
//
// A disturbance is injected just before each switch so the plant is genuinely off
// the operating point across the boundary (at equilibrium every controller agrees
// on u_eq and any switch is trivially bumpless — an uninformative test). The
// switch bump is reported against a baseline: the largest tick-to-tick command
// change a SINGLE controller makes while rejecting the same disturbance.
#include <algorithm>
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "kontrolem_control/problem.hpp"
#include "kontrolem_controllers/lqg_controller.hpp"
#include "kontrolem_controllers/mpc_controller.hpp"
#include "kontrolem_model/robot_model.hpp"

using namespace kontrolem_control;
using namespace kontrolem_controllers;
using kontrolem_model::RobotModel;

namespace
{
constexpr double kDt = 0.005;
constexpr int kK1 = 240;   // t=1.20s : LQG -> MPC
constexpr int kK2 = 480;   // t=2.40s : MPC -> LQG
constexpr int kSteps = 800;
constexpr int kKickLead = 6;      // inject the disturbance 30 ms before each switch
constexpr double kKick = 1.0;     // pole angular-velocity kick [rad/s]

struct Result
{
  double jump1 = 0.0;        // |Δu| at the LQG->MPC switch
  double jump2 = 0.0;        // |Δu| at the MPC->LQG switch
  double baseline_slew = 0;  // largest single-controller tick-to-tick |Δu| in the kick windows
  double peak_pole_after2 = 0.0;  // worst |pole| in the 0.5 s after the MPC->LQG switch
  double final_pole = 0.0, final_cart = 0.0;
  bool finite = true;
};

// Run the full LQG -> MPC -> LQG scenario. `bumpless` selects the MPC->LQG policy.
Result run(const RobotModel & model, LqgController & lqg, MpcController & mpc,
           const Regulation & reg, bool bumpless)
{
  // Fresh LQG internal state for Phase 1 (re-seed to the operating point).
  State reset0; reset0.q = reg.q_ref; reset0.v = Eigen::VectorXd::Zero(model.nv());
  lqg.seed_from_state(reset0, reg);  // xhat = 0 (at the operating point)

  Result r;
  Eigen::VectorXd q(2), v(2), tau(2);
  q << 0.0, 0.12; v << 0.0, 0.0; tau.setZero();
  State st; st.t = 0.0;
  double u_prev = 0.0;
  bool have_prev = false;

  for (int k = 0; k < kSteps; ++k, st.t += kDt) {
    // Plant integrates the previously applied command.
    const Eigen::VectorXd a = model.aba(q, v, tau);
    v.noalias() += a * kDt;
    q.noalias() += v * kDt;

    // Disturbance just before each switch: an impulse on the pole velocity.
    if (k == kK1 - kKickLead || k == kK2 - kKickLead) v(1) += kKick;

    st.q = q; st.v = v;

    // Bumpless activation happens the instant before LQG resumes control.
    if (k == kK2 && bumpless) lqg.seed_from_state(st, reg);

    const bool lqg_active = (k < kK1) || (k >= kK2);
    double u;
    if (lqg_active) {
      u = lqg.compute(st, reg, kDt).tau(0);
    } else {
      u = mpc.compute(st, reg, kDt).tau(0);
    }
    tau << u, 0.0;

    if (have_prev) {
      const double du = std::abs(u - u_prev);
      if (k == kK1) r.jump1 = du;
      else if (k == kK2) r.jump2 = du;
      else {
        // Baseline: normal command slew while a single controller rejects a kick.
        const bool in_window = (k > kK1 - kKickLead && k < kK1) ||
                               (k > kK2 - kKickLead && k < kK2) || (k > kK2 && k < kK2 + 40);
        if (in_window) r.baseline_slew = std::max(r.baseline_slew, du);
      }
    }
    u_prev = u; have_prev = true;

    if (k >= kK2 && k < kK2 + 100) r.peak_pole_after2 = std::max(r.peak_pole_after2, std::abs(q(1)));
    if (!q.allFinite()) { r.finite = false; break; }
  }
  r.final_pole = std::abs(q(1));
  r.final_cart = std::abs(q(0));
  return r;
}
}  // namespace

int main()
{
  const RobotModel model = RobotModel::from_urdf_file(CART_POLE_URDF);

  // Stateful LQG (positions-only output feedback) — the validated cart-pole tuning.
  Eigen::MatrixXd Q = Eigen::Vector4d(1.0, 10.0, 1.0, 1.0).asDiagonal();
  Eigen::MatrixXd R = Eigen::Matrix<double, 1, 1>::Constant(0.1);
  Eigen::MatrixXd W = Eigen::Vector4d(1.0, 1.0, 10.0, 10.0).asDiagonal();
  Eigen::MatrixXd V = 1e-3 * Eigen::Matrix2d::Identity();
  LqgController lqg({"cart_joint"}, Q, R, W, V, /*innov_max=*/0.5);

  // Stateless MPC — the validated cart-pole tuning (loose limit).
  MpcController mpc({"cart_joint"}, Q, R, /*horizon=*/30, /*dt_mpc=*/0.02, /*tau_max=*/100.0);

  Regulation reg;
  reg.q_ref = Eigen::VectorXd::Zero(model.nq());
  reg.v_ref = Eigen::VectorXd::Zero(model.nv());

  lqg.configure(model, *lqg.synthesize(model, reg), reg);
  mpc.configure(model, *mpc.synthesize(model, reg), reg);

  const Result naive = run(model, lqg, mpc, reg, /*bumpless=*/false);
  const Result bump  = run(model, lqg, mpc, reg, /*bumpless=*/true);

  std::cout << "LQG->MPC switch (MPC stateless): |Δu| = " << naive.jump1 << " N\n";
  std::cout << "MPC->LQG switch:\n"
            << "  NAIVE (stale observer)   : |Δu| = " << naive.jump2
            << " N, peak|pole| after = " << naive.peak_pole_after2
            << ", final|pole| = " << naive.final_pole << "\n"
            << "  BUMPLESS (seeded)        : |Δu| = " << bump.jump2
            << " N, peak|pole| after = " << bump.peak_pole_after2
            << ", final|pole| = " << bump.final_pole << ", final|cart| = " << bump.final_cart
            << "\n";
  std::cout << "  baseline command slew (single controller rejecting the same kick) ~ "
            << bump.baseline_slew << " N\n";

  // FINDINGS (the spike's job is to confirm-or-falsify, and surface the real
  // constraint — not to make every policy pass):
  //  1. For a STATE-CARRYING incoming controller, seeding its internal state from
  //     the current world state (seed_from_state) is a decisive bumpless primitive:
  //     it cuts the MPC->LQG switch bump to the scale of a normal command slew and
  //     the controller recovers cleanly.
  //  2. NAIVE reactivation (a Supervisor that just re-enables a dormant controller
  //     with a stale estimate) is a genuine hazard — a large bump AND degraded
  //     recovery. So the Supervisor MUST call an activation hook, not just flip a
  //     flag. (Reported, not required to "recover".)
  //  3. RESIDUAL RISK (still T3): the STATELESS-incoming direction (LQG->MPC) has
  //     no state to seed, so its switch bump is pure inter-controller disagreement
  //     on the off-equilibrium state and stays large. State-init does NOT close
  //     this — it needs output-level conditioning (command rate-limit / short
  //     blend) or switching only near state agreement. Bounded and non-divergent
  //     here, but the honest open item the full Supervisor must handle.
  //
  // PASS = the mechanism is confirmed and safe: nothing diverges, the bumpless
  // policy recovers, and seeding cuts the state-carrying switch bump far below the
  // naive one and down to normal-slew scale.
  const bool finite = naive.finite && bump.finite;                 // no switch diverges
  // Recovery = the balance objective (the pole) is caught and held, and the cart
  // isn't running away. The cart's slow position mode is still returning at the end
  // of this short window — that's LQG regulation speed (test_lqg_cartpole), not a
  // switching effect, so it's bounded here, not driven to zero.
  const bool bumpless_recovers = bump.final_pole < 1e-2 && bump.final_cart < 1.0;
  const bool bumpless_helps = bump.jump2 < 0.5 * naive.jump2;      // seeding cuts the bump
  const bool bumpless_smooth = bump.jump2 <= 2.0 * bump.baseline_slew;  // ~ normal slew
  const bool naive_is_worse = naive.final_pole > bump.final_pole;  // hazard demonstrated
  const bool ok = finite && bumpless_recovers && bumpless_helps && bumpless_smooth && naive_is_worse;
  std::cout << (ok ? "PASS" : "FAIL") << "  (finite=" << finite
            << " bumpless_recovers=" << bumpless_recovers << " bumpless_helps=" << bumpless_helps
            << " bumpless_smooth=" << bumpless_smooth << " naive_is_worse=" << naive_is_worse
            << ")\n";
  return ok ? 0 : 1;
}
