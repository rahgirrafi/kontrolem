#include "kontrolem_workbench/lint.hpp"

#include <algorithm>
#include <cstdint>
#include <set>
#include <variant>

#include "kontrolem_workbench/spec_format.hpp"

namespace kontrolem_workbench
{
namespace kc = kontrolem_control;

namespace
{

// ---------------------------------------------------------------------------
// The runtime-owned parameter schema (kontrolem_ros2_control on_init
// auto_declares — kept in lockstep with kontrolem_controller.cpp). Types are
// encoded as a representative default so the same type-check path applies.
const std::map<std::string, kc::ParamValue> & framework_params()
{
  static const std::map<std::string, kc::ParamValue> table = {
    {"control_law", kc::ParamValue{std::string{}}},
    {"control_laws", kc::ParamValue{std::vector<std::string>{}}},
    {"switch_blend_ticks", kc::ParamValue{std::int64_t{0}}},
    {"auto_fallback", kc::ParamValue{false}},
    {"fallback_dwell", kc::ParamValue{std::int64_t{0}}},
    {"auto_recover", kc::ParamValue{false}},
    {"recover_dwell", kc::ParamValue{std::int64_t{0}}},
    {"actuated_joints", kc::ParamValue{std::vector<std::string>{}}},
    {"robot_description", kc::ParamValue{std::string{}}},
    {"command_interface", kc::ParamValue{std::string{}}},
    {"state_position_interface", kc::ParamValue{std::string{}}},
    {"state_velocity_interface", kc::ParamValue{std::string{}}},
    {"safe_action", kc::ParamValue{std::string{}}},
    {"publish_diagnostics", kc::ParamValue{false}},
    {"publish_planned_contact", kc::ParamValue{false}},
    {"planned_contact_topic", kc::ParamValue{std::string{}}},
    {"provenance_dir", kc::ParamValue{std::string{}}},
    {"q_ref", kc::ParamValue{std::vector<double>{}}},
    {"v_ref", kc::ParamValue{std::vector<double>{}}},
    {"reference_type", kc::ParamValue{std::string{}}},
    {"reference.center", kc::ParamValue{std::vector<double>{}}},
    {"reference.amp", kc::ParamValue{std::vector<double>{}}},
    {"reference.phase", kc::ParamValue{std::vector<double>{}}},
    {"reference.omega", kc::ParamValue{0.0}},
    {"reference.base.amp", kc::ParamValue{std::vector<double>{}}},
    {"reference.base.omega", kc::ParamValue{std::vector<double>{}}},
    {"reference.base.phase", kc::ParamValue{std::vector<double>{}}},
    {"base_target_topic", kc::ParamValue{std::string{}}},
    {"gait.order", kc::ParamValue{std::vector<std::int64_t>{}}},
    {"gait.period", kc::ParamValue{0.0}},
    {"gait.duty", kc::ParamValue{0.0}},
    {"gait.step_len", kc::ParamValue{0.0}},
    {"gait.step_h", kc::ParamValue{0.0}},
    {"gait.base_gain", kc::ParamValue{0.0}},
    {"gait.start_delay", kc::ParamValue{0.0}},
    {"gait.swing_pair", kc::ParamValue{std::vector<std::int64_t>{}}},
    {"gait.settle_gate", kc::ParamValue{false}},
    {"gait.settle_release_speed", kc::ParamValue{0.0}},
    {"gait.settle_speed", kc::ParamValue{0.0}},
    {"gait.settle_tilt", kc::ParamValue{0.0}},
    {"gait.settle_hold", kc::ParamValue{0.0}},
    {"gait.settle_timeout", kc::ParamValue{0.0}},
    {"base_type", kc::ParamValue{std::string{}}},
    {"base_gpio", kc::ParamValue{std::string{}}},
    {"contact_gpio", kc::ParamValue{std::string{}}},
    {"contact_frames", kc::ParamValue{std::vector<std::string>{}}},
    {"wbc.base_height", kc::ParamValue{0.0}},
    {"wbc.nominal_posture", kc::ParamValue{std::vector<double>{}}},
  };
  return table;
}

std::size_t edit_distance(const std::string & a, const std::string & b)
{
  std::vector<std::size_t> prev(b.size() + 1), cur(b.size() + 1);
  for (std::size_t j = 0; j <= b.size(); ++j) prev[j] = j;
  for (std::size_t i = 1; i <= a.size(); ++i) {
    cur[0] = i;
    for (std::size_t j = 1; j <= b.size(); ++j) {
      const std::size_t sub = prev[j - 1] + (a[i - 1] == b[j - 1] ? 0 : 1);
      cur[j] = std::min({prev[j] + 1, cur[j - 1] + 1, sub});
    }
    std::swap(prev, cur);
  }
  return prev[b.size()];
}

std::string nearest(const std::string & key, const std::vector<std::string> & candidates)
{
  std::string best;
  std::size_t best_d = 3;  // suggest only within edit distance 2
  for (const auto & c : candidates) {
    const std::size_t d = edit_distance(key, c);
    if (d < best_d) {
      best_d = d;
      best = c;
    }
  }
  return best;
}

/// Type check: does the parsed value's type match the expected ParamValue
/// alternative? Returns "" on match, else a human-readable complaint. Mirrors
/// ROS 2's strict parameter typing (1 is an int, never a double).
std::string type_mismatch(const kc::ParamValue & expected, const RawValue & got)
{
  const std::string want = type_name(expected);
  const std::string have = type_name(got.value);
  if (want == have) return "";
  const bool want_array = want.find("[]") != std::string::npos;
  if (want_array && got.is_empty_array) return "";  // [] is a valid empty anything-array
  if (want == "double" && have == "int") {
    return "is a " + have + " but must be a double (ROS typing is strict — write " +
           value_str(got.value) + ".0)";
  }
  if (want == "double[]" && have == "int[]") {
    return "is an int array but must be a double array (write decimals, e.g. 1.0 not 1)";
  }
  return "is a " + have + " but must be a " + want;
}

}  // namespace

LintResult lint(
  const FlatConfig & config, const FactoryMap & laws, const kontrolem_model::RobotModel * model)
{
  LintResult r;

  // ---- 1) Which laws does this config select? ----
  std::vector<std::string> selected;
  const auto law_it = config.find("control_law");
  const auto laws_it = config.find("control_laws");
  if (laws_it != config.end() &&
      std::holds_alternative<std::vector<std::string>>(laws_it->second.value) &&
      !std::get<std::vector<std::string>>(laws_it->second.value).empty()) {
    selected = std::get<std::vector<std::string>>(laws_it->second.value);
  } else if (law_it != config.end() &&
             std::holds_alternative<std::string>(law_it->second.value)) {
    selected = {std::get<std::string>(law_it->second.value)};
  } else {
    r.errors.push_back("no control_law (or control_laws) set");
    return r;
  }
  std::string avail;
  for (const auto & kv : laws) avail += " " + kv.first;
  for (const auto & s : selected) {
    if (laws.find(s) == laws.end()) {
      r.errors.push_back("unknown control law '" + s + "' (installed:" + avail + ")");
    }
  }
  if (!r.errors.empty()) return r;  // nothing sensible to check further

  // ---- 2) Ownership maps: which prefixes belong to which law. ----
  // A law's prefixes come from its spec NAMES (kinematic_gait declares kin.*).
  std::map<std::string, std::string> prefix_owner;  // "lqr" -> law "lqr", "kin" -> "kinematic_gait"
  std::map<std::string, kc::ParameterSpec> specs;
  for (const auto & kv : laws) {
    specs[kv.first] = kv.second->parameter_spec();
    for (const auto & d : specs[kv.first]) {
      const auto dot = d.name.find('.');
      if (dot != std::string::npos) prefix_owner[d.name.substr(0, dot)] = kv.first;
    }
  }
  const std::set<std::string> selected_set(selected.begin(), selected.end());
  const auto & fw = framework_params();

  // ---- 3) Per-key checks. ----
  for (const auto & kv : config) {
    const std::string & key = kv.first;

    // Framework-owned (includes the runtime's wbc.base_height/nominal_posture).
    const auto f = fw.find(key);
    if (f != fw.end()) {
      const std::string tm = type_mismatch(f->second, kv.second);
      if (!tm.empty()) r.errors.push_back("'" + key + "' " + tm);
      continue;
    }

    const auto dot = key.find('.');
    const std::string prefix = dot == std::string::npos ? std::string{} : key.substr(0, dot);
    const auto owner = prefix.empty() ? prefix_owner.end() : prefix_owner.find(prefix);

    if (owner != prefix_owner.end()) {
      const std::string & law = owner->second;
      const auto & spec = specs[law];
      const auto d = std::find_if(
        spec.begin(), spec.end(), [&](const kc::ParamDesc & p) { return p.name == key; });
      if (d == spec.end()) {
        // Under a law's namespace but not in its spec: a typo (the runtime would
        // reject it at on_configure if the law is selected; catch it here first).
        std::vector<std::string> names;
        for (const auto & p : spec) names.push_back(p.name);
        const std::string sug = nearest(key, names);
        r.errors.push_back(
          "'" + key + "' is not a parameter of control law '" + law + "'" +
          (sug.empty() ? "" : " (did you mean '" + sug + "'?)"));
      } else if (selected_set.count(law) == 0) {
        r.warnings.push_back(
          "'" + key + "' configures law '" + law + "', which this config does not select");
      } else {
        const std::string tm = type_mismatch(d->default_value, kv.second);
        if (!tm.empty()) r.errors.push_back("'" + key + "' " + tm);
      }
      continue;
    }

    // Neither framework nor any law's namespace: the runtime would silently
    // accept it (overrides auto-declare), so surface it.
    std::vector<std::string> names;
    for (const auto & fkv : fw) names.push_back(fkv.first);
    for (const auto & s : selected) {
      for (const auto & p : specs[s]) names.push_back(p.name);
    }
    const std::string sug = nearest(key, names);
    r.warnings.push_back(
      "unknown parameter '" + key + "' — the runtime would silently ignore it" +
      (sug.empty() ? "" : " (did you mean '" + sug + "'?)"));
  }

  // ---- 4) Deep checks against the robot model (what on_configure would hit). ----
  if (model == nullptr) return r;

  std::vector<std::string> actuated;
  if (const auto a = config.find("actuated_joints"); a != config.end()) {
    if (const auto * v = std::get_if<std::vector<std::string>>(&a->second.value)) actuated = *v;
  }
  if (actuated.empty()) {
    r.errors.push_back("actuated_joints is empty — the controller cannot command anything");
  }
  for (const auto & j : actuated) {
    try {
      (void)model->joint_v_index(j);
    } catch (const std::exception &) {
      r.errors.push_back("actuated joint '" + j + "' does not exist in the robot model");
    }
  }
  std::vector<std::string> contact_frames;
  if (const auto c = config.find("contact_frames"); c != config.end()) {
    if (const auto * v = std::get_if<std::vector<std::string>>(&c->second.value)) {
      contact_frames = *v;
    }
  }
  for (const auto & fr : contact_frames) {
    try {
      (void)model->frame_index(fr);
    } catch (const std::exception &) {
      r.errors.push_back("contact frame '" + fr + "' does not exist in the robot model");
    }
  }
  const auto check_len = [&](const char * key, int want) {
    const auto it = config.find(key);
    if (it == config.end()) return;
    if (const auto * v = std::get_if<std::vector<double>>(&it->second.value)) {
      if (!v->empty() && static_cast<int>(v->size()) != want) {
        r.errors.push_back(
          std::string("'") + key + "' has " + std::to_string(v->size()) +
          " entries but the model needs " + std::to_string(want));
      }
    }
  };
  check_len("q_ref", model->nq());
  check_len("v_ref", model->nv());

  // Dry-run each selected law's factory — the exact construction on_configure
  // performs, minus the middleware.
  for (const auto & s : selected) {
    kc::ParameterMap params(specs[s]);
    for (const auto & d : specs[s]) {
      const auto it = config.find(d.name);
      if (it == config.end()) continue;
      if (it->second.is_empty_array) continue;  // keep the (typed) spec default
      if (!type_mismatch(d.default_value, it->second).empty()) continue;  // already reported
      params.set(d.name, it->second.value);
    }
    kc::BuildContext ctx;
    ctx.actuated_joints = actuated;
    ctx.contact_frames = contact_frames;
    try {
      (void)laws.at(s)->create(params, *model, ctx);
    } catch (const std::exception & e) {
      r.errors.push_back("control law '" + s + "' rejects this config: " + e.what());
    }
  }
  return r;
}

std::unique_ptr<kc::Controller> build_from_config(
  const FlatConfig & config, const FactoryMap & laws, const kontrolem_model::RobotModel & model,
  std::string * law_name_out, kc::ParameterMap * params_out)
{
  std::string law;
  if (const auto it = config.find("control_laws");
      it != config.end() && std::holds_alternative<std::vector<std::string>>(it->second.value) &&
      !std::get<std::vector<std::string>>(it->second.value).empty()) {
    law = std::get<std::vector<std::string>>(it->second.value).front();
  } else if (const auto it2 = config.find("control_law");
             it2 != config.end() && std::holds_alternative<std::string>(it2->second.value)) {
    law = std::get<std::string>(it2->second.value);
  } else {
    throw std::runtime_error("config sets no control_law / control_laws");
  }
  const auto fit = laws.find(law);
  if (fit == laws.end()) {
    throw std::runtime_error("unknown control law '" + law + "'");
  }
  const kc::ParameterSpec spec = fit->second->parameter_spec();
  kc::ParameterMap params(spec);
  for (const auto & d : spec) {
    const auto it = config.find(d.name);
    if (it == config.end() || it->second.is_empty_array) continue;
    const std::string tm = type_mismatch(d.default_value, it->second);
    if (!tm.empty()) {
      throw std::runtime_error("'" + d.name + "' " + tm);
    }
    params.set(d.name, it->second.value);
  }
  kc::BuildContext ctx;
  if (const auto a = config.find("actuated_joints"); a != config.end()) {
    if (const auto * v = std::get_if<std::vector<std::string>>(&a->second.value)) {
      ctx.actuated_joints = *v;
    }
  }
  if (const auto c = config.find("contact_frames"); c != config.end()) {
    if (const auto * v = std::get_if<std::vector<std::string>>(&c->second.value)) {
      ctx.contact_frames = *v;
    }
  }
  if (law_name_out) *law_name_out = law;
  if (params_out) *params_out = params;
  return fit->second->create(params, model, ctx);
}

}  // namespace kontrolem_workbench
