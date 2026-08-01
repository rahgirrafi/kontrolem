// Offline (no ROS, no pluginlib): the codec + lint catch every config-error
// class this project has actually hit — typo'd law params (example.frction /
// lpv.sched_nods), mis-shaped LPV grids (the five M17 rejection cases), strict
// ROS typing (1 is not 1.0), bad joint/frame names, wrong q_ref length — and a
// `new`-scaffolded config round-trips through lint clean for every shipped law.
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "kontrolem_controllers/factories.hpp"
#include "kontrolem_model/robot_model.hpp"
#include "kontrolem_workbench/codec.hpp"
#include "kontrolem_workbench/lint.hpp"
#include "kontrolem_workbench/scaffold.hpp"

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
  m["wbc"] = std::make_shared<WbcFactory>();
  m["kinematic_gait"] = std::make_shared<KinematicGaitFactory>();
  return m;
}

/// Write `body` (the ros__parameters payload, 4-space indented lines) as a
/// controllers.yaml to a temp file and parse it back.
FlatConfig parse(const std::string & body)
{
  const std::string path = std::string(SCRATCH_DIR) + "/lint_case.yaml";
  std::ofstream f(path);
  f << "kontrolem_controller:\n  ros__parameters:\n" << body;
  f.close();
  return load_controllers_yaml(path);
}

bool has_error_containing(const LintResult & r, const std::string & needle)
{
  for (const auto & e : r.errors) {
    if (e.find(needle) != std::string::npos) return true;
  }
  return false;
}

bool has_warning_containing(const LintResult & r, const std::string & needle)
{
  for (const auto & w : r.warnings) {
    if (w.find(needle) != std::string::npos) return true;
  }
  return false;
}

}  // namespace

int main()
{
  const auto laws = shipped_laws();
  const RobotModel arm = RobotModel::from_urdf_file(ARM2_URDF);
  const std::string arm_base =
    "    actuated_joints: [\"shoulder_joint\", \"elbow_joint\"]\n"
    "    q_ref: [0.0, 0.0]\n"
    "    v_ref: [0.0, 0.0]\n";

  // ---- 1) A known-good config (the shipped LPV demo, distilled) is clean. ----
  {
    const auto cfg = parse(
      "    control_law: lpv\n" + arm_base +
      "    lpv.sched_joints: [\"shoulder_joint\", \"elbow_joint\"]\n"
      "    lpv.sched_min: [-1.6, -1.6]\n"
      "    lpv.sched_max: [1.6, 1.6]\n"
      "    lpv.sched_nodes: [9, 9]\n"
      "    lpv.q_diag: [10.0, 10.0, 1.0, 1.0]\n"
      "    lpv.r_diag: [0.1, 0.1]\n");
    const auto r = lint(cfg, laws, &arm);
    check(r.ok() && r.warnings.empty(), "good LPV config is clean");
  }

  // ---- 2) The historical typo cases are ERRORS, with suggestions. ----
  {
    const auto r = lint(parse("    control_law: lpv\n" + arm_base +
                              "    lpv.sched_nods: [9, 9]\n"), laws, nullptr);
    check(has_error_containing(r, "lpv.sched_nods"), "lpv.sched_nods typo is an error");
    check(has_error_containing(r, "lpv.sched_nodes"), "typo suggests lpv.sched_nodes");
  }
  {
    const auto r = lint(parse("    control_law: qp\n" + arm_base +
                              "    qp.frction: 0.7\n"), laws, nullptr);
    check(has_error_containing(r, "qp.frction"), "qp.frction typo is an error");
  }

  // ---- 3) Unknown law; unknown framework key warns with a suggestion. ----
  {
    const auto r = lint(parse("    control_law: pid\n"), laws, nullptr);
    check(has_error_containing(r, "unknown control law 'pid'"), "unknown law is an error");
  }
  {
    const auto r = lint(parse("    control_law: lqr\n" + arm_base +
                              "    safe_actoin: zero\n"), laws, nullptr);
    check(has_warning_containing(r, "safe_actoin"), "typo'd framework key warns");
    check(has_warning_containing(r, "safe_action"), "framework typo suggests safe_action");
    check(r.ok(), "framework typo is a warning, not an error");
  }

  // ---- 4) Strict ROS typing: 1 is an int, not a double. ----
  {
    const auto r = lint(parse("    control_law: lqr\n" + arm_base +
                              "    lqr.q_dev_max: 1\n"), laws, nullptr);
    check(has_error_containing(r, "lqr.q_dev_max"), "int-for-double is an error");
    check(has_error_containing(r, "1.0"), "int-for-double suggests writing 1.0");
  }
  {
    const auto r = lint(parse("    control_law: lqr\n" + arm_base +
                              "    lqr.q_diag: [1, 1, 1, 1]\n"), laws, nullptr);
    check(has_error_containing(r, "lqr.q_diag"), "int-array-for-double-array is an error");
  }
  {
    const auto r = lint(parse("    control_law: mpc\n" + arm_base +
                              "    mpc.horizon: 20.5\n"), laws, nullptr);
    check(has_error_containing(r, "mpc.horizon"), "double-for-int is an error");
  }
  {
    // An empty [] is untyped and legal for any array parameter.
    const auto r = lint(parse("    control_law: lqr\n" + arm_base +
                              "    lqr.q_diag: []\n"), laws, &arm);
    check(r.ok(), "empty array matches any array type");
  }

  // ---- 5) Params of a NON-selected law warn (kept-for-switching pattern). ----
  {
    const auto r = lint(parse("    control_law: qp\n" + arm_base +
                              "    qp.kp: 100.0\n"
                              "    lqr.q_diag: [1.0, 1.0, 1.0, 1.0]\n"), laws, nullptr);
    check(r.ok(), "non-selected law's params are not errors");
    check(has_warning_containing(r, "lqr.q_diag"), "non-selected law's params warn");
  }

  // ---- 6) Deep model checks. ----
  {
    const auto r = lint(parse("    control_law: lqr\n"
                              "    actuated_joints: [\"shoulder_joint\", \"elbo_joint\"]\n"),
                        laws, &arm);
    check(has_error_containing(r, "elbo_joint"), "unknown actuated joint is an error");
  }
  {
    const auto r = lint(parse("    control_law: lqr\n" + arm_base +
                              "    q_ref: [0.0, 0.0, 0.0]\n"), laws, &arm);
    check(has_error_containing(r, "q_ref"), "wrong q_ref length is an error");
  }

  // ---- 7) The five M17 LPV grid cases are caught by the dry-run create(). ----
  {
    const auto mk = [&](const std::string & grid) {
      return parse("    control_law: lpv\n" + arm_base + grid);
    };
    check(has_error_containing(lint(mk(""), laws, &arm), "sched_joints"),
          "LPV: empty scheduling grid rejected");
    check(has_error_containing(
            lint(mk("    lpv.sched_joints: [\"shoulder_joint\", \"elbow_joint\"]\n"
                    "    lpv.sched_min: [-1.0]\n"
                    "    lpv.sched_max: [1.0, 1.0]\n"), laws, &arm), "sched_min"),
          "LPV: short min array rejected");
    check(has_error_containing(
            lint(mk("    lpv.sched_joints: [\"no_such\", \"elbow_joint\"]\n"
                    "    lpv.sched_min: [-1.0, -1.0]\n"
                    "    lpv.sched_max: [1.0, 1.0]\n"), laws, &arm), "no_such"),
          "LPV: unknown scheduling joint rejected");
    check(has_error_containing(
            lint(mk("    lpv.sched_joints: [\"shoulder_joint\", \"elbow_joint\"]\n"
                    "    lpv.sched_min: [-1.0, -1.0]\n"
                    "    lpv.sched_max: [1.0, 1.0]\n"
                    "    lpv.sched_nodes: [1, 4]\n"), laws, &arm), "rejects this config"),
          "LPV: node count < 2 rejected");
    check(has_error_containing(
            lint(mk("    lpv.sched_joints: [\"shoulder_joint\", \"elbow_joint\"]\n"
                    "    lpv.sched_min: [-1.0, -1.0]\n"
                    "    lpv.sched_max: [-2.0, 1.0]\n"), laws, &arm), "rejects this config"),
          "LPV: inverted envelope rejected");
  }

  // ---- 8) Scaffold -> lint round-trip is clean for every shipped fixed-base law. ----
  for (const auto & kv : laws) {
    if (kv.first == "wbc" || kv.first == "kinematic_gait") continue;  // need a floating base
    const std::string yaml =
      scaffold_yaml(kv.first, kv.second->parameter_spec(), arm.joint_names(), arm.nq());
    const std::string path = std::string(SCRATCH_DIR) + "/scaffold_" + kv.first + ".yaml";
    std::ofstream f(path);
    f << yaml;
    f.close();
    const auto cfg = load_controllers_yaml(path);
    const auto r = lint(cfg, laws, kv.first == "lpv" ? nullptr : &arm);
    // (lpv's scaffold has an intentionally-empty grid the user must fill, so the
    // model dry-run is skipped for it; the schema pass must still be clean.)
    check(r.ok(), "scaffold round-trips clean: " + kv.first);
    for (const auto & w : r.warnings) {
      check(false, "scaffold warning (" + kv.first + "): " + w);
    }
  }

  std::cout << "test_lint: " << (failures == 0 ? "PASS" : "FAIL") << " (" << failures
            << " failures)\n";
  return failures == 0 ? 0 : 1;
}
