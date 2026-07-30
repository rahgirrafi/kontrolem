// The M16 config-path GATE, proven OFFLINE (no ROS): a ControllerFactory built
// from a ParameterMap produces the SAME controller the old make_law() built by
// hand, and configured overrides actually flow into construction. Before M16 the
// param-name -> value -> constructor mapping was exercised only by a full Gazebo
// launch; this closes that gap with a plain C++ test.
//
//   metadata  — every factory reports its name() and lists its parameters.
//   equality  — factory(defaults) == direct-construct(defaults), byte-for-byte
//               on a nominal compute(), for LQR (matrix-diag params) and WBC
//               (Gains-struct params) — the two mapping styles.
//   override  — a changed parameter changes the built controller's behaviour,
//               proving values are threaded through, not ignored.
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "kontrolem_control/problem.hpp"
#include "kontrolem_controllers/factories.hpp"
#include "kontrolem_controllers/lpv_controller.hpp"
#include "kontrolem_controllers/lqr_controller.hpp"
#include "kontrolem_controllers/wbc_controller.hpp"
#include "kontrolem_model/robot_model.hpp"

using namespace kontrolem_control;
using namespace kontrolem_controllers;
using kontrolem_model::BaseType;
using kontrolem_model::RobotModel;

static int failures = 0;
static void check(bool cond, const std::string & what)
{
  if (!cond) {
    std::cout << "  FAIL: " << what << "\n";
    ++failures;
  }
}

static bool spec_has(const ParameterSpec & spec, const std::string & name)
{
  for (const auto & d : spec) {
    if (d.name == name) {
      return true;
    }
  }
  return false;
}

int main()
{
  // ---- 1) Metadata: name() + a representative parameter for each factory. ----
  check(LqrFactory{}.name() == "lqr", "LqrFactory name");
  check(LqgFactory{}.name() == "lqg", "LqgFactory name");
  check(LpvFactory{}.name() == "lpv", "LpvFactory name");
  check(MpcFactory{}.name() == "mpc", "MpcFactory name");
  check(QpFactory{}.name() == "qp", "QpFactory name");
  check(WbcFactory{}.name() == "wbc", "WbcFactory name");
  check(KinematicGaitFactory{}.name() == "kinematic_gait", "KinematicGaitFactory name");

  check(spec_has(LqrFactory{}.parameter_spec(), "lqr.q_dev_max"), "lqr spec has q_dev_max");
  check(spec_has(LqgFactory{}.parameter_spec(), "lqg.innov_max"), "lqg spec has innov_max");
  check(spec_has(LpvFactory{}.parameter_spec(), "lpv.sched_joints"), "lpv spec has sched_joints");
  check(spec_has(LpvFactory{}.parameter_spec(), "lpv.sched_nodes"), "lpv spec has sched_nodes");
  check(spec_has(MpcFactory{}.parameter_spec(), "mpc.horizon"), "mpc spec has horizon");
  check(spec_has(QpFactory{}.parameter_spec(), "qp.kp"), "qp spec has kp");
  check(spec_has(WbcFactory{}.parameter_spec(), "wbc.kp_base"), "wbc spec has kp_base");
  check(spec_has(WbcFactory{}.parameter_spec(), "wbc.max_iter"), "wbc spec has max_iter");
  check(spec_has(KinematicGaitFactory{}.parameter_spec(), "kin.kp"), "kin spec has kp");

  // ---- 2a) LQR: factory(defaults) == direct-construct(defaults). ----
  {
    const RobotModel model = RobotModel::from_urdf_file(CART_POLE_URDF);
    const int nv = model.nv();
    Regulation upright;
    upright.q_ref = Eigen::VectorXd::Zero(model.nq());
    upright.v_ref = Eigen::VectorXd::Zero(nv);

    // A tilted, non-trivial state so a wrong gain would show.
    State s;
    s.q = Eigen::VectorXd::Zero(model.nq());
    s.q(0) = 0.10;  // cart offset
    s.q(1) = 0.15;  // pole tilt
    s.v = Eigen::VectorXd::Zero(nv);
    s.t = 0.0;

    const LqrFactory f;
    const BuildContext ctx{/*actuated=*/{"cart_joint"}, /*contact_frames=*/{}};
    ParameterMap p(f.parameter_spec());  // all defaults

    auto built = f.create(p, model, ctx);
    auto syn_b = built->synthesize(model, upright);
    built->configure(model, *syn_b, upright);
    const Eigen::VectorXd tau_factory = built->compute(s, upright, 0.001).tau;

    // The old make_law used identity Q/R when the *_diag arrays are empty, and 0.5.
    LqrController direct(
      {"cart_joint"}, Eigen::MatrixXd::Identity(2 * nv, 2 * nv), Eigen::MatrixXd::Identity(1, 1),
      0.5);
    auto syn_d = direct.synthesize(model, upright);
    direct.configure(model, *syn_d, upright);
    const Eigen::VectorXd tau_direct = direct.compute(s, upright, 0.001).tau;

    check(tau_factory.size() == tau_direct.size() &&
            (tau_factory - tau_direct).cwiseAbs().maxCoeff() < 1e-9,
          "LQR factory(defaults) == direct(defaults)");

    // ---- override plumbing: a heavy pole weight must change the command. ----
    ParameterMap p2(f.parameter_spec());
    p2.set("lqr.q_diag", ParamValue{std::vector<double>{1.0, 50.0, 1.0, 1.0}});  // heavy pole angle
    auto built2 = f.create(p2, model, ctx);
    auto syn_2 = built2->synthesize(model, upright);
    built2->configure(model, *syn_2, upright);
    const Eigen::VectorXd tau_over = built2->compute(s, upright, 0.001).tau;
    check((tau_over - tau_factory).cwiseAbs().maxCoeff() > 1e-6,
          "LQR q_diag override changes the command (params flow through)");
  }

  // ---- 2b) WBC: factory(defaults) == direct-construct(default Gains). ----
  {
    const RobotModel model = RobotModel::from_urdf_file(FLOATING_QUAD_URDF, BaseType::kFloating);
    const int nv = model.nv();
    const std::vector<std::string> feet = {"foot_FL", "foot_FR", "foot_RL", "foot_RR"};
    const std::vector<std::string> legs = {"FL", "FR", "RL", "RR"};
    std::vector<std::string> actuated;
    for (const auto & leg : legs) {
      actuated.push_back("hipx_" + leg);
      actuated.push_back("hipy_" + leg);
      actuated.push_back("knee_" + leg);
    }

    Eigen::VectorXd q_stand = model.neutral();
    for (const auto & leg : legs) {
      q_stand(model.joint_q_index("hipx_" + leg)) = 0.0;
      q_stand(model.joint_q_index("hipy_" + leg)) = 0.7;
      q_stand(model.joint_q_index("knee_" + leg)) = -1.4;
    }
    double min_fz = 1e9;
    for (const auto & fr : feet) min_fz = std::min(min_fz, model.frame_position(q_stand, fr).z());
    q_stand(2) = -min_fz;

    Regulation reg;
    reg.q_ref = q_stand;
    reg.v_ref = Eigen::VectorXd::Zero(nv);

    State s;
    s.q = q_stand;
    s.v = Eigen::VectorXd::Zero(nv);
    s.t = 0.0;

    const WbcFactory f;
    const BuildContext ctx{actuated, feet};
    ParameterMap p(f.parameter_spec());  // all defaults
    auto built = f.create(p, model, ctx);
    auto syn_b = built->synthesize(model, reg);
    built->configure(model, *syn_b, reg);
    const Eigen::VectorXd tau_factory = built->compute(s, reg, 0.002).tau;

    WbcController direct(feet, actuated, WbcController::Gains{});  // default gains == spec defaults
    auto syn_d = direct.synthesize(model, reg);
    direct.configure(model, *syn_d, reg);
    const Eigen::VectorXd tau_direct = direct.compute(s, reg, 0.002).tau;

    check(tau_factory.size() == tau_direct.size() && tau_factory.size() > 0 &&
            (tau_factory - tau_direct).cwiseAbs().maxCoeff() < 1e-9,
          "WBC factory(defaults) == direct(default Gains)");
  }

  // ---- 2c) LPV (M17): the list-valued schema — four parallel arrays become a
  // grid of SchedAxis, with the scheduling joints resolved by NAME. ----
  {
    const RobotModel model = RobotModel::from_urdf_file(ARM2_URDF);
    const std::vector<std::string> actuated = {"shoulder_joint", "elbow_joint"};
    const LpvFactory f;
    const BuildContext ctx{actuated, /*contact_frames=*/{}};

    Regulation reg;
    reg.q_ref = Eigen::VectorXd::Zero(model.nq());
    reg.v_ref = Eigen::VectorXd::Zero(model.nv());

    State s;
    s.q = Eigen::Vector2d(0.30, -0.45);  // inside the envelope, off-node (interpolating)
    s.v = Eigen::Vector2d(0.10, -0.05);
    s.t = 0.0;

    // Deliberately ASYMMETRIC axes: if the factory resolved the joint names to the
    // wrong configuration indices, the grid would differ and the command would move.
    ParameterMap p(f.parameter_spec());
    p.set("lpv.sched_joints", ParamValue{actuated});
    p.set("lpv.sched_min", ParamValue{std::vector<double>{-1.0, -1.4}});
    p.set("lpv.sched_max", ParamValue{std::vector<double>{1.0, 0.6}});
    p.set("lpv.sched_nodes", ParamValue{std::vector<std::int64_t>{5, 4}});

    auto built = f.create(p, model, ctx);
    auto syn_b = built->synthesize(model, reg);
    built->configure(model, *syn_b, reg);
    const Eigen::VectorXd tau_factory = built->compute(s, reg, 0.002).tau;

    // shoulder_joint -> q index 0, elbow_joint -> q index 1 on this arm.
    const std::vector<SchedAxis> axes = {SchedAxis{0, -1.0, 1.0, 5}, SchedAxis{1, -1.4, 0.6, 4}};
    LpvController direct(
      actuated, Eigen::MatrixXd::Identity(2 * model.nv(), 2 * model.nv()),
      Eigen::MatrixXd::Identity(2, 2), axes);
    auto syn_d = direct.synthesize(model, reg);
    direct.configure(model, *syn_d, reg);
    const Eigen::VectorXd tau_direct = direct.compute(s, reg, 0.002).tau;

    check(tau_factory.size() == tau_direct.size() && tau_factory.size() == 2 &&
            (tau_factory - tau_direct).cwiseAbs().maxCoeff() < 1e-9,
          "LPV factory(grid) == direct(same axes) — joint names resolved to the right q indices");
    check(static_cast<LpvController *>(built.get())->node_count() == 20,
          "LPV grid is 5 x 4 = 20 designed nodes");

    // Mis-shaped / bad configuration is rejected AT BUILD, not at the first tick —
    // the config-as-data payoff for a schema the flat parameter world can't type-check.
    auto rejects = [&](const char * what, void (*mutate)(ParameterMap &)) {
      ParameterMap bad(f.parameter_spec());
      bad.set("lpv.sched_joints", ParamValue{actuated});
      bad.set("lpv.sched_min", ParamValue{std::vector<double>{-1.0, -1.4}});
      bad.set("lpv.sched_max", ParamValue{std::vector<double>{1.0, 0.6}});
      mutate(bad);
      bool threw = false;
      try {
        (void)f.create(bad, model, ctx);
      } catch (const std::exception &) {
        threw = true;
      }
      check(threw, what);
    };
    rejects("LPV rejects an empty scheduling-joint list", [](ParameterMap & b) {
      b.set("lpv.sched_joints", ParamValue{std::vector<std::string>{}});
    });
    rejects("LPV rejects a min/max array shorter than the joint list", [](ParameterMap & b) {
      b.set("lpv.sched_min", ParamValue{std::vector<double>{-1.0}});
    });
    rejects("LPV rejects an unknown scheduling joint name", [](ParameterMap & b) {
      b.set("lpv.sched_joints", ParamValue{std::vector<std::string>{"no_such_joint", "elbow_joint"}});
    });
    rejects("LPV rejects a node count < 2", [](ParameterMap & b) {
      b.set("lpv.sched_nodes", ParamValue{std::vector<std::int64_t>{1, 4}});
    });
    rejects("LPV rejects an inverted envelope (max <= min)", [](ParameterMap & b) {
      b.set("lpv.sched_max", ParamValue{std::vector<double>{-2.0, 0.6}});
    });
  }

  std::cout << "test_controller_factories: " << (failures == 0 ? "PASS" : "FAIL") << " (" << failures
            << " failures)\n";
  return failures == 0 ? 0 : 1;
}
