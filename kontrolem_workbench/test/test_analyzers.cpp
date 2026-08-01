// Step-3 gate, offline (no ROS): each pane computes what it claims —
//  - lqr: the CARE-designed loop is reported stable on the cart-pole;
//  - lqg: controller AND estimator poles stable, estimator strictly faster;
//  - lpv: the interpolation-gap check actually detects grid coarseness (the
//    midpoint gap on a COARSE grid exceeds the gap on a FINE grid);
//  - mpc: window adequacy + a healthy Hessian on the shipped cart-pole config;
//  - qp: the imposed-dynamics prediction matches the SIMULATED settling — the
//    analytic pane and the empirical tier cross-check each other;
//  - dispatcher: an unknown law gets the shared-tier notice.
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "kontrolem_controllers/factories.hpp"
#include "kontrolem_workbench/analyzers.hpp"
#include "kontrolem_workbench/metrics.hpp"
#include "kontrolem_workbench/simulator.hpp"

using namespace kontrolem_control;
using namespace kontrolem_workbench;
using kontrolem_model::RobotModel;

static int failures = 0;
static void check(bool cond, const std::string & what)
{
  if (!cond) {
    std::cout << "  FAIL: " << what << "\n";
    ++failures;
  }
}

namespace
{
FactoryMap shipped_laws()
{
  using namespace kontrolem_controllers;
  FactoryMap m;
  m["lqr"] = std::make_shared<LqrFactory>();
  m["lqg"] = std::make_shared<LqgFactory>();
  m["lpv"] = std::make_shared<LpvFactory>();
  m["mpc"] = std::make_shared<MpcFactory>();
  m["qp"] = std::make_shared<QpFactory>();
  return m;
}
}  // namespace

int main()
{
  const auto laws = shipped_laws();

  // ---- LQR on the cart-pole: the designed loop is stable, and the report
  // carries the trust region. ----
  {
    const RobotModel model = RobotModel::from_urdf_file(CART_POLE_URDF);
    ParameterMap p(laws.at("lqr")->parameter_spec());
    p.set("lqr.q_dev_max", ParamValue{1.0});
    const Eigen::VectorXd q0 = Eigen::VectorXd::Zero(2);
    const LqrPane pane = analyze_lqr(model, laws, p, {"cart_joint"}, q0);
    check(pane.cl.stable, "lqr: closed loop stable");
    check(pane.cl.poles.size() == 4, "lqr: 2nv poles");
    check(pane.q_dev_max == 1.0, "lqr: trust region reported");
    // The open loop is genuinely unstable (upright pole) — the pane must see
    // the difference control makes.
    const EigReport ol = closed_loop_eig(model, q0, Eigen::MatrixXd{}, {});
    check(!ol.stable && ol.max_re > 1.0, "lqr: open loop correctly unstable");
  }

  // ---- LQG: both pole sets stable; estimator strictly faster. ----
  {
    const RobotModel model = RobotModel::from_urdf_file(CART_POLE_URDF);
    ParameterMap p(laws.at("lqg")->parameter_spec());
    const Eigen::VectorXd q0 = Eigen::VectorXd::Zero(2);
    const LqgPane pane = analyze_lqg(model, laws, p, {"cart_joint"}, q0);
    check(pane.ctrl.stable, "lqg: controller poles stable");
    check(pane.est.stable, "lqg: estimator poles stable");
    check(pane.est_speedup > 1.0, "lqg: estimator faster than controller");
  }

  // ---- LPV: the midpoint gap detects coarseness (coarse >> fine). ----
  {
    const RobotModel model = RobotModel::from_urdf_file(ARM2_URDF);
    const std::vector<std::string> act = {"shoulder_joint", "elbow_joint"};
    const Eigen::VectorXd q0 = Eigen::VectorXd::Zero(2);
    const auto grid = [&](std::int64_t n) {
      ParameterMap p(laws.at("lpv")->parameter_spec());
      p.set("lpv.sched_joints", ParamValue{act});
      p.set("lpv.sched_min", ParamValue{std::vector<double>{-1.4, -1.4}});
      p.set("lpv.sched_max", ParamValue{std::vector<double>{1.4, 1.4}});
      p.set("lpv.sched_nodes", ParamValue{std::vector<std::int64_t>{n, n}});
      return p;
    };
    const LpvPane coarse = analyze_lpv(model, laws, grid(3), act, q0);
    const LpvPane fine = analyze_lpv(model, laws, grid(9), act, q0);
    check(coarse.nodes == 9 && fine.nodes == 81, "lpv: node counts");
    check(coarse.midpoints == 4 && fine.midpoints == 64, "lpv: midpoint counts");
    check(coarse.all_nodes_stable && fine.all_nodes_stable, "lpv: nodes locally stable");
    std::cout << "  [lpv] interpolation gap: coarse " << coarse.interp_gap << ", fine "
              << fine.interp_gap << "\n";
    check(coarse.interp_gap >= 0.0, "lpv: interpolating cannot beat designing (coarse)");
    check(fine.interp_gap < coarse.interp_gap, "lpv: the gap shrinks with grid refinement");
    check(fine.all_midpoints_stable, "lpv: fine grid stable between nodes too");
  }

  // ---- MPC: shipped-style cart-pole config -> window covers, Hessian healthy. ----
  {
    const RobotModel model = RobotModel::from_urdf_file(CART_POLE_URDF);
    ParameterMap p(laws.at("mpc")->parameter_spec());
    const Eigen::VectorXd q0 = Eigen::VectorXd::Zero(2);
    const MpcPane pane = analyze_mpc(model, laws, p, {"cart_joint"}, q0);
    std::cout << "  [mpc] window " << pane.window_s << " s, plant tau " << pane.plant_slowest_tau
              << " s, cond(H) " << pane.cond_H << "\n";
    check(!pane.open_loop.stable, "mpc: sees the unstable plant");
    check(pane.window_covers, "mpc: default window covers the plant timescale");
    check(pane.cond_H > 1.0 && pane.cond_H < 1e8, "mpc: default Hessian healthy");
  }

  // ---- QP: analytic prediction vs simulation agree (the cross-check). ----
  {
    const RobotModel model = RobotModel::from_urdf_file(ARM2_URDF);
    const std::vector<std::string> act = {"shoulder_joint", "elbow_joint"};
    ParameterMap p(laws.at("qp")->parameter_spec());
    p.set("qp.kp", ParamValue{100.0});
    p.set("qp.kd", ParamValue{20.0});
    p.set("qp.tau_max", ParamValue{30.0});
    const Eigen::VectorXd q0 = Eigen::VectorXd::Zero(2);
    const QpPane pane = analyze_qp(model, p, act, q0);
    check(std::abs(pane.zeta - 1.0) < 1e-9, "qp: kd=2*sqrt(kp) reads as critically damped");
    check(std::abs(pane.omega_n - 10.0) < 1e-9, "qp: omega_n = sqrt(kp)");
    check(std::abs(pane.t_settle_pred - 0.583) < 0.02,
          "qp: predicted 2% settling ~0.583 s (got " + std::to_string(pane.t_settle_pred) + ")");
    check(pane.hold_tau_frac > 0.5 && pane.hold_tau_frac < 0.8,
          "qp: horizontal arm hold ~19.6 of 30 Nm");

    BuildContext ctx{act, {}};
    auto law = laws.at("qp")->create(p, model, ctx);
    SimScenario sc;
    sc.signal = SimScenario::Signal::kRelease;
    sc.q_ref = q0;
    sc.q0 = Eigen::VectorXd::Constant(2, 0.2);
    sc.duration = 4.0;
    const SimTrace tr = simulate(model, *law, act, sc);
    const Metrics m = compute_metrics(tr, 0.004);  // the 2% band of the 0.2 release
    std::cout << "  [qp] predicted settling " << pane.t_settle_pred << " s, simulated "
              << m.settling_time << " s\n";
    check(std::abs(m.settling_time - pane.t_settle_pred) < 0.15,
          "qp: simulation confirms the imposed-dynamics prediction");
  }

  // ---- Dispatcher: unknown law -> shared-tier notice, no throw. ----
  {
    const RobotModel model = RobotModel::from_urdf_file(CART_POLE_URDF);
    ParameterMap p;
    const std::string txt =
      law_pane_text("example", model, laws, p, {"cart_joint"}, Eigen::VectorXd::Zero(2));
    check(txt.find("no law-specific pane") != std::string::npos,
          "dispatcher: unknown law gets the shared-tier notice");
  }

  std::cout << "test_analyzers: " << (failures == 0 ? "PASS" : "FAIL") << " (" << failures
            << " failures)\n";
  return failures == 0 ? 0 : 1;
}
