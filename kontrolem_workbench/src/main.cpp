// kontrolem_setup — the Kontrol'Em workbench CLI (M18).
//
// The ONLY translation unit in the workbench that touches pluginlib: every
// subcommand discovers the installed control laws through the same
// ClassLoader<ControllerFactory> the L4 runtime uses (M16), so what this tool
// lists, lints, and analyzes is exactly what a launch would load — including
// third-party laws the framework has never heard of.
#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include <pluginlib/class_loader.hpp>

#include "kontrolem_control/controller_factory.hpp"
#include "kontrolem_model/robot_model.hpp"
#include "kontrolem_workbench/analyzers.hpp"
#include "kontrolem_workbench/codec.hpp"
#include "kontrolem_workbench/lint.hpp"
#include "kontrolem_workbench/metrics.hpp"
#include "kontrolem_workbench/scaffold.hpp"
#include "kontrolem_workbench/simulator.hpp"
#include "kontrolem_workbench/spec_format.hpp"

namespace kc = kontrolem_control;
namespace kw = kontrolem_workbench;

namespace
{

struct DiscoveredLaw
{
  std::shared_ptr<kc::ControllerFactory> factory;
  std::string plugin_class;  ///< pluginlib class name (for the description lookup)
};

/// Discover every registered ControllerFactory, keyed by its law name().
std::map<std::string, DiscoveredLaw> discover(
  pluginlib::ClassLoader<kc::ControllerFactory> & loader)
{
  std::map<std::string, DiscoveredLaw> laws;
  for (const auto & cls : loader.getDeclaredClasses()) {
    try {
      auto inst = loader.createSharedInstance(cls);
      laws[inst->name()] = DiscoveredLaw{inst, cls};
    } catch (const std::exception & e) {
      std::cerr << "warning: plugin '" << cls << "' failed to load: " << e.what() << "\n";
    }
  }
  return laws;
}

int cmd_list(pluginlib::ClassLoader<kc::ControllerFactory> & loader)
{
  const auto laws = discover(loader);
  std::size_t w = 4;
  for (const auto & kv : laws) w = std::max(w, kv.first.size());
  for (const auto & kv : laws) {
    std::string desc = loader.getClassDescription(kv.second.plugin_class);
    // plugins.xml descriptions may span (indented) lines; flatten for the table.
    std::replace(desc.begin(), desc.end(), '\n', ' ');
    desc.erase(
      std::unique(desc.begin(), desc.end(), [](char a, char b) { return a == ' ' && b == ' '; }),
      desc.end());
    std::cout << "  " << kv.first << std::string(w - kv.first.size() + 2, ' ') << desc << "\n";
  }
  std::cout << "(" << laws.size() << " control laws; `kontrolem_setup describe <law>` for parameters)\n";
  return 0;
}

int cmd_describe(
  pluginlib::ClassLoader<kc::ControllerFactory> & loader, const std::string & law)
{
  const auto laws = discover(loader);
  const auto it = laws.find(law);
  if (it == laws.end()) {
    std::cerr << "error: unknown control law '" << law << "'. Installed laws:\n";
    for (const auto & kv : laws) std::cerr << "  " << kv.first << "\n";
    return 1;
  }
  std::cout << kw::format_spec(law, it->second.factory->parameter_spec());
  return 0;
}

/// --urdf <file> [--floating] -> a RobotModel, or nullopt when --urdf absent.
std::unique_ptr<kontrolem_model::RobotModel> load_model(
  const std::vector<std::string> & args, const kw::FlatConfig * config)
{
  std::string urdf;
  for (std::size_t i = 0; i + 1 < args.size(); ++i) {
    if (args[i] == "--urdf") urdf = args[i + 1];
  }
  if (urdf.empty()) return nullptr;
  // base_type comes from the config when present (matches what a launch would do).
  auto base = kontrolem_model::BaseType::kFixed;
  if (config) {
    const auto it = config->find("base_type");
    if (it != config->end() && std::holds_alternative<std::string>(it->second.value) &&
        std::get<std::string>(it->second.value) == "floating") {
      base = kontrolem_model::BaseType::kFloating;
    }
  }
  return std::make_unique<kontrolem_model::RobotModel>(
    kontrolem_model::RobotModel::from_urdf_file(urdf, base));
}

int cmd_lint(
  pluginlib::ClassLoader<kc::ControllerFactory> & loader, const std::vector<std::string> & args)
{
  if (args.size() < 2) {
    std::cerr << "usage: kontrolem_setup lint <controllers.yaml> [--urdf <robot.urdf>]\n";
    return 1;
  }
  std::string block;
  const kw::FlatConfig config = kw::load_controllers_yaml(args[1], &block);
  const auto model = load_model(args, &config);

  kw::FactoryMap laws;
  for (const auto & kv : discover(loader)) laws[kv.first] = kv.second.factory;
  const kw::LintResult r = kw::lint(config, laws, model.get());

  for (const auto & w : r.warnings) std::cout << "  warning: " << w << "\n";
  for (const auto & e : r.errors) std::cout << "  error: " << e << "\n";
  std::cout << args[1] << " [" << block << "]: "
            << (r.ok() ? "OK" : ("INVALID (" + std::to_string(r.errors.size()) + " errors)"))
            << (model ? " — checked against the robot model" : " — schema check only (no --urdf)")
            << "\n";
  return r.ok() ? 0 : 1;
}

int cmd_new(
  pluginlib::ClassLoader<kc::ControllerFactory> & loader, const std::vector<std::string> & args)
{
  std::string urdf, law, out_yaml;
  for (std::size_t i = 1; i + 1 < args.size(); ++i) {
    if (args[i] == "--urdf") urdf = args[i + 1];
    if (args[i] == "--law") law = args[i + 1];
    if (args[i] == "-o") out_yaml = args[i + 1];
  }
  if (urdf.empty() || law.empty()) {
    std::cerr << "usage: kontrolem_setup new --urdf <robot.urdf> --law <law> [-o out.yaml]\n";
    return 1;
  }
  const auto laws = discover(loader);
  const auto it = laws.find(law);
  if (it == laws.end()) {
    std::cerr << "error: unknown control law '" << law << "' (see `kontrolem_setup list`)\n";
    return 1;
  }
  const auto model = kontrolem_model::RobotModel::from_urdf_file(urdf);
  const std::string yaml = kw::scaffold_yaml(
    law, it->second.factory->parameter_spec(), model.joint_names(), model.nq());
  if (out_yaml.empty()) {
    std::cout << yaml;
  } else {
    std::ofstream f(out_yaml);
    if (!f) {
      std::cerr << "error: cannot write '" << out_yaml << "'\n";
      return 1;
    }
    f << yaml;
    std::cout << "wrote " << out_yaml << " — edit it, then: kontrolem_setup lint " << out_yaml
              << " --urdf " << urdf << "\n";
  }
  return 0;
}

int cmd_analyze(
  pluginlib::ClassLoader<kc::ControllerFactory> & loader, const std::vector<std::string> & args)
{
  if (args.size() < 2) {
    std::cerr << "usage: kontrolem_setup analyze <controllers.yaml> --urdf <robot.urdf>\n"
                 "         [--signal release|step|push|sine] [--delta 0.2] [--push-mag 3.0]\n"
                 "         [--push-joint <name>] [--omega 1.0] [--duration 6.0] [--dt 0.002]\n"
                 "         [--band 0.02] [--csv out.csv]\n";
    return 1;
  }
  std::string signal = "release", push_joint, csv;
  double delta = 0.2, push_mag = 3.0, omega = 1.0, duration = 6.0, dt = 0.002, band = 0.02;
  for (std::size_t i = 2; i + 1 < args.size(); ++i) {
    if (args[i] == "--signal") signal = args[i + 1];
    if (args[i] == "--delta") delta = std::stod(args[i + 1]);
    if (args[i] == "--push-mag") push_mag = std::stod(args[i + 1]);
    if (args[i] == "--push-joint") push_joint = args[i + 1];
    if (args[i] == "--omega") omega = std::stod(args[i + 1]);
    if (args[i] == "--duration") duration = std::stod(args[i + 1]);
    if (args[i] == "--dt") dt = std::stod(args[i + 1]);
    if (args[i] == "--band") band = std::stod(args[i + 1]);
    if (args[i] == "--csv") csv = args[i + 1];
  }

  const kw::FlatConfig config = kw::load_controllers_yaml(args[1]);
  const auto model = load_model(args, &config);
  if (!model) {
    std::cerr << "error: analyze needs --urdf <robot.urdf> (the plant to simulate)\n";
    return 1;
  }
  if (const auto bt = config.find("base_type");
      bt != config.end() && std::holds_alternative<std::string>(bt->second.value) &&
      std::get<std::string>(bt->second.value) == "floating") {
    std::cerr << "error: analyze (v1) simulates fixed-base robots only — the floating-base/"
                 "contact tier is a planned extension\n";
    return 1;
  }

  kw::FactoryMap laws;
  for (const auto & kv : discover(loader)) laws[kv.first] = kv.second.factory;
  std::string law_name;
  kc::ParameterMap params;
  auto law = kw::build_from_config(config, laws, *model, &law_name, &params);

  // Scenario from the config: the setpoint is its q_ref; the excitation acts on
  // the actuated coordinates.
  const int nq = model->nq();
  const int nv = model->nv();
  std::vector<std::string> actuated;
  if (const auto a = config.find("actuated_joints"); a != config.end()) {
    if (const auto * v = std::get_if<std::vector<std::string>>(&a->second.value)) actuated = *v;
  }
  if (actuated.empty()) {
    std::cerr << "error: the config sets no actuated_joints\n";
    return 1;
  }
  kw::SimScenario sc;
  sc.q_ref = Eigen::VectorXd::Zero(nq);
  if (const auto qr = config.find("q_ref"); qr != config.end()) {
    if (const auto * v = std::get_if<std::vector<double>>(&qr->second.value)) {
      if (static_cast<int>(v->size()) == nq) {
        for (int i = 0; i < nq; ++i) sc.q_ref(i) = (*v)[static_cast<std::size_t>(i)];
      }
    }
  }
  sc.duration = duration;
  sc.dt = dt;
  std::vector<int> act_q, act_v;
  for (const auto & j : actuated) {
    act_q.push_back(model->joint_q_index(j));
    act_v.push_back(model->joint_v_index(j));
  }
  if (signal == "release") {
    sc.signal = kw::SimScenario::Signal::kRelease;
    sc.q0 = sc.q_ref;
    for (const int i : act_q) sc.q0(i) += delta;
  } else if (signal == "step") {
    sc.signal = kw::SimScenario::Signal::kStep;
    sc.q_target = sc.q_ref;
    for (const int i : act_q) sc.q_target(i) += delta;
  } else if (signal == "push") {
    sc.signal = kw::SimScenario::Signal::kPush;
    sc.push_tau = Eigen::VectorXd::Zero(nv);
    if (push_joint.empty()) {
      for (const int i : act_v) sc.push_tau(i) = push_mag;
    } else {
      sc.push_tau(model->joint_v_index(push_joint)) = push_mag;
    }
  } else if (signal == "sine") {
    sc.signal = kw::SimScenario::Signal::kSine;
    sc.sine_amp = Eigen::VectorXd::Zero(nq);
    for (const int i : act_q) sc.sine_amp(i) = delta;
    sc.sine_omega = omega;
  } else {
    std::cerr << "error: unknown --signal '" << signal << "' (release|step|push|sine)\n";
    return 1;
  }

  std::cout << "analyze: control_law " << law_name << " on " << args[1] << "\n"
            << "  signal: " << signal << "  (delta " << delta << ", duration " << duration
            << " s, dt " << dt << ")\n\n";
  const kw::SimTrace tr = kw::simulate(*model, *law, actuated, sc);

  // The law's own torque limit (if its schema declares one) feeds the
  // saturation gauge.
  double tau_limit = 0.0;
  for (const auto & d : laws.at(law_name)->parameter_spec()) {
    if (d.name.size() > 8 && d.name.substr(d.name.size() - 8) == ".tau_max" &&
        std::holds_alternative<double>(d.default_value)) {
      tau_limit = params.double_at(d.name);
    }
  }
  const kw::Metrics m = kw::compute_metrics(tr, band, tau_limit);
  std::cout << "-- simulated response (every law gets this tier) --\n"
            << kw::format_metrics(m, band) << "\n"
            << kw::law_pane_text(law_name, *model, laws, params, actuated, sc.q_ref);
  if (!csv.empty()) {
    kw::write_csv(tr, csv);
    std::cout << "\ntrace written to " << csv
              << "  (plot: python3 <workbench>/scripts/plot_run.py " << csv << ")\n";
  }
  std::cout << "\nNote: simulation is evidence at THIS scenario, not a certificate.\n";
  return m.finite ? 0 : 1;
}

void usage()
{
  std::cout <<
    "kontrolem_setup — Kontrol'Em offline workbench\n"
    "\n"
    "  kontrolem_setup list                     installed control laws (via the plugin registry)\n"
    "  kontrolem_setup describe <law>           a law's parameter schema (name/type/default)\n"
    "  kontrolem_setup lint <yaml> [--urdf u]   validate a controllers.yaml offline;\n"
    "                                           with --urdf also dry-runs the law's construction\n"
    "  kontrolem_setup new --urdf u --law l     generate a commented starter controllers.yaml\n"
    "                  [-o out.yaml]\n"
    "  kontrolem_setup analyze <yaml> --urdf u  closed-loop test-signal simulation + metrics\n"
    "                  [--signal release|step|push|sine] [--csv out.csv] ...\n"
    "  kontrolem_setup docs                     markdown parameter tables for all laws\n"
    "\n"
    "Run from a sourced workspace so the plugin registry is on the ament index.\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::vector<std::string> args(argv + 1, argv + argc);
  if (args.empty() || args[0] == "-h" || args[0] == "--help") {
    usage();
    return args.empty() ? 1 : 0;
  }
  try {
    pluginlib::ClassLoader<kc::ControllerFactory> loader(
      "kontrolem_control", "kontrolem_control::ControllerFactory");
    if (args[0] == "list") {
      return cmd_list(loader);
    }
    if (args[0] == "describe") {
      if (args.size() < 2) {
        std::cerr << "usage: kontrolem_setup describe <law>\n";
        return 1;
      }
      return cmd_describe(loader, args[1]);
    }
    if (args[0] == "lint") {
      return cmd_lint(loader, args);
    }
    if (args[0] == "new") {
      return cmd_new(loader, args);
    }
    if (args[0] == "analyze") {
      return cmd_analyze(loader, args);
    }
    if (args[0] == "docs") {
      // Markdown parameter tables for every installed law — paste into the
      // reference docs so they can never drift from the code.
      const auto laws = discover(loader);
      std::cout << "<!-- Generated by `kontrolem_setup docs` — do not edit by hand. -->\n\n";
      for (const auto & kv : laws) {
        std::cout << kw::format_spec_markdown(kv.first, kv.second.factory->parameter_spec())
                  << "\n";
      }
      return 0;
    }
    std::cerr << "error: unknown subcommand '" << args[0] << "'\n\n";
    usage();
    return 1;
  } catch (const std::exception & e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
