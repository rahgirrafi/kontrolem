// Step-2 gate for the workbench's shared empirical tier, offline (no ROS):
//  - LQR cart-pole release behaves like the shipped e2e (settles, healthy);
//  - QP arm's measured settling matches its IMPOSED dynamics prediction
//    (s^2 + kd s + kp, critically damped) — this cross-checks the simulator
//    AND the per-law pane math against each other;
//  - a push is recovered from; a sine is tracked; a Regulation-only law asked
//    to track throws; the CSV writer emits a well-formed file.
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "kontrolem_controllers/factories.hpp"
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

int main()
{
  // ---- LQR cart-pole: release from a tilted pole (the M0 scenario). ----
  {
    const RobotModel model = RobotModel::from_urdf_file(CART_POLE_URDF);
    const kontrolem_controllers::LqrFactory f;
    ParameterMap p(f.parameter_spec());
    p.set("lqr.q_dev_max", ParamValue{1.0});  // the shipped trust region (M0 finding)
    BuildContext ctx{{"cart_joint"}, {}};
    auto law = f.create(p, model, ctx);

    SimScenario sc;
    sc.signal = SimScenario::Signal::kRelease;
    sc.q_ref = Eigen::VectorXd::Zero(2);
    sc.q0 = Eigen::VectorXd::Zero(2);
    sc.q0(1) = 0.15;  // pole tilt, as in the e2e
    sc.duration = 8.0;

    const SimTrace tr = simulate(model, *law, {"cart_joint"}, sc);
    const Metrics m = compute_metrics(tr, 0.05);
    check(m.finite, "lqr cart-pole: finite");
    check(m.settling_time >= 0.0 && m.settling_time < 6.0, "lqr cart-pole: settles");
    check(m.ss_error < 0.01, "lqr cart-pole: near-zero steady-state error");
    check(m.ok_frac == 1.0, "lqr cart-pole: trustworthy the whole run");

    // CSV round-trip: header + one row per tick.
    const std::string csv = std::string(SCRATCH_DIR) + "/lqr_release.csv";
    write_csv(tr, csv);
    std::ifstream in(csv);
    std::string line;
    std::size_t rows = 0;
    while (std::getline(in, line)) ++rows;
    check(rows == tr.t.size() + 1, "csv has header + one row per tick");
  }

  // ---- QP arm: measured settling vs the imposed-dynamics prediction. ----
  {
    const RobotModel model = RobotModel::from_urdf_file(ARM2_URDF);
    const kontrolem_controllers::QpFactory f;
    ParameterMap p(f.parameter_spec());
    p.set("qp.kp", ParamValue{100.0});
    p.set("qp.kd", ParamValue{20.0});      // critically damped: double pole at s = -10
    p.set("qp.tau_max", ParamValue{30.0});  // the shipped arm2 limit
    BuildContext ctx{{"shoulder_joint", "elbow_joint"}, {}};
    auto law = f.create(p, model, ctx);

    SimScenario sc;
    sc.signal = SimScenario::Signal::kRelease;
    sc.q_ref = Eigen::VectorXd::Zero(2);
    sc.q0 = Eigen::VectorXd::Constant(2, 0.2);
    sc.duration = 4.0;

    const SimTrace tr = simulate(model, *law, {"shoulder_joint", "elbow_joint"}, sc);
    // 2% band of the 0.2 rad release; critically damped (1+x)e^-x = 0.02 -> x = 5.83
    // -> t = 0.583 s. The sim must land near it (deviation = constraints/integration).
    const Metrics m = compute_metrics(tr, 0.004, 30.0);
    check(m.finite, "qp arm: finite");
    check(
      m.settling_time > 0.4 && m.settling_time < 0.8,
      "qp arm: settling ~0.58 s as the imposed dynamics predict (got " +
        std::to_string(m.settling_time) + ")");
    check(m.overshoot_frac < 0.05, "qp arm: critically damped -> no real overshoot");
    check(m.ok_frac == 1.0, "qp arm: QP feasible throughout");
  }

  // ---- Push recovery on the cart-pole. ----
  {
    const RobotModel model = RobotModel::from_urdf_file(CART_POLE_URDF);
    const kontrolem_controllers::LqrFactory f;
    ParameterMap p(f.parameter_spec());
    p.set("lqr.q_dev_max", ParamValue{1.0});
    BuildContext ctx{{"cart_joint"}, {}};
    auto law = f.create(p, model, ctx);

    SimScenario sc;
    sc.signal = SimScenario::Signal::kPush;
    sc.q_ref = Eigen::VectorXd::Zero(2);
    sc.push_tau = Eigen::VectorXd::Zero(2);
    sc.push_tau(0) = 3.0;  // shove the cart (a pole kick this size exceeds LQR's basin)
    sc.push_duration = 0.2;
    sc.t_event = 1.0;
    sc.duration = 8.0;

    const SimTrace tr = simulate(model, *law, {"cart_joint"}, sc);
    const Metrics m = compute_metrics(tr, 0.05);
    double worst = 0.0;
    for (std::size_t k = 0; k < tr.t.size(); ++k) {
      worst = std::max(worst, std::abs(tr.q[k](1)));
    }
    std::cout << "  [push] worst pole=" << worst << " recovery=" << m.recovery_time
              << " ss=" << m.ss_error << "\n";
    check(m.finite, "push: finite");
    // The kick must actually disturb the plant, and the law must bring it back.
    check(worst > 0.02, "push: the pole was really disturbed");
    check(m.recovery_time >= 0.0 && m.recovery_time < 5.0, "push: recovered");
  }

  // ---- Sine tracking (LQR accepts Tracking); QP must refuse it. ----
  {
    const RobotModel model = RobotModel::from_urdf_file(CART_POLE_URDF);
    const kontrolem_controllers::LqrFactory f;
    ParameterMap p(f.parameter_spec());
    p.set("lqr.q_dev_max", ParamValue{1.0});
    BuildContext ctx{{"cart_joint"}, {}};
    auto law = f.create(p, model, ctx);

    SimScenario sc;
    sc.signal = SimScenario::Signal::kSine;
    sc.q_ref = Eigen::VectorXd::Zero(2);
    sc.sine_amp = Eigen::VectorXd::Zero(2);
    sc.sine_amp(0) = 0.2;  // the cart oscillates; the pole is held
    sc.sine_omega = 1.0;
    sc.duration = 8.0;

    const SimTrace tr = simulate(model, *law, {"cart_joint"}, sc);
    const Metrics m = compute_metrics(tr, 0.1);
    // The cart must actually FOLLOW the reference (feedback-only tracking lags,
    // so judge amplitude, not a tight error band).
    double cart_amp = 0.0, pole_worst = 0.0;
    for (std::size_t k = tr.t.size() / 2; k < tr.t.size(); ++k) {
      cart_amp = std::max(cart_amp, std::abs(tr.q[k](0)));
      pole_worst = std::max(pole_worst, std::abs(tr.q[k](1)));
    }
    std::cout << "  [sine] ss=" << m.ss_error << " cart_amp=" << cart_amp
              << " pole_worst=" << pole_worst << " ok=" << m.ok_frac << "\n";
    check(m.finite, "sine: finite");
    check(cart_amp > 0.1, "sine: the cart really oscillates with the reference");
    check(pole_worst < 0.1, "sine: the pole stays near upright while tracking");
    check(m.ss_error < 0.3, "sine: tracking error bounded");
    check(m.ok_frac == 1.0, "sine: trustworthy while tracking");

    const kontrolem_controllers::QpFactory qf;
    ParameterMap qparams(qf.parameter_spec());
    auto qp = qf.create(qparams, model, ctx);
    bool threw = false;
    try {
      (void)simulate(model, *qp, {"cart_joint"}, sc);
    } catch (const std::runtime_error &) {
      threw = true;
    }
    check(threw, "a Regulation-only law asked to track a sine throws");
  }

  std::cout << "test_simulator: " << (failures == 0 ? "PASS" : "FAIL") << " (" << failures
            << " failures)\n";
  return failures == 0 ? 0 : 1;
}
