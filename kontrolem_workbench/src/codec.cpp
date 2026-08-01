#include "kontrolem_workbench/codec.hpp"

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace kontrolem_workbench
{
namespace kc = kontrolem_control;

namespace
{

bool parse_int(const std::string & s, std::int64_t & out)
{
  if (s.empty() || s.find_first_of(".eE") != std::string::npos) return false;
  try {
    std::size_t pos = 0;
    out = std::stoll(s, &pos);
    return pos == s.size();
  } catch (...) {
    return false;
  }
}

bool parse_double(const std::string & s, double & out)
{
  try {
    std::size_t pos = 0;
    out = std::stod(s, &pos);
    return pos == s.size();
  } catch (...) {
    return false;
  }
}

/// Infer a scalar's type the way the ROS YAML parameter parser does: quoted ->
/// string; true/false -> bool; integer literal -> int; numeric -> double; else
/// string.
kc::ParamValue infer_scalar(const YAML::Node & n)
{
  const std::string s = n.as<std::string>();
  if (n.Tag() == "!") return kc::ParamValue{s};  // explicitly quoted in the YAML
  if (s == "true" || s == "True") return kc::ParamValue{true};
  if (s == "false" || s == "False") return kc::ParamValue{false};
  std::int64_t i = 0;
  if (parse_int(s, i)) return kc::ParamValue{i};
  double d = 0.0;
  if (parse_double(s, d)) return kc::ParamValue{d};
  return kc::ParamValue{s};
}

RawValue infer_sequence(const YAML::Node & n, const std::string & key)
{
  RawValue rv;
  if (n.size() == 0) {
    rv.value = kc::ParamValue{std::vector<double>{}};
    rv.is_empty_array = true;
    return rv;
  }
  // Element kind: all ints -> int[]; numeric with any double -> double[]; else string[].
  bool all_int = true, all_num = true;
  for (const auto & e : n) {
    if (!e.IsScalar()) {
      throw std::runtime_error("'" + key + "': nested arrays are not supported");
    }
    const auto v = infer_scalar(e);
    if (!std::holds_alternative<std::int64_t>(v)) all_int = false;
    if (!std::holds_alternative<std::int64_t>(v) && !std::holds_alternative<double>(v)) {
      all_num = false;
    }
  }
  if (all_int) {
    std::vector<std::int64_t> out;
    for (const auto & e : n) out.push_back(std::get<std::int64_t>(infer_scalar(e)));
    rv.value = kc::ParamValue{out};
  } else if (all_num) {
    std::vector<double> out;
    for (const auto & e : n) {
      const auto v = infer_scalar(e);
      out.push_back(
        std::holds_alternative<double>(v) ? std::get<double>(v)
                                          : static_cast<double>(std::get<std::int64_t>(v)));
    }
    rv.value = kc::ParamValue{out};
  } else {
    std::vector<std::string> out;
    for (const auto & e : n) out.push_back(e.as<std::string>());
    rv.value = kc::ParamValue{out};
  }
  return rv;
}

void flatten(const YAML::Node & node, const std::string & prefix, FlatConfig & out)
{
  for (const auto & kv : node) {
    const std::string key = prefix.empty() ? kv.first.as<std::string>()
                                           : prefix + "." + kv.first.as<std::string>();
    const YAML::Node & v = kv.second;
    if (v.IsMap()) {
      flatten(v, key, out);
    } else if (v.IsSequence()) {
      out[key] = infer_sequence(v, key);
    } else if (v.IsScalar()) {
      out[key] = RawValue{infer_scalar(v), false};
    } else if (v.IsNull()) {
      throw std::runtime_error("'" + key + "' has no value");
    }
  }
}

}  // namespace

FlatConfig load_controllers_yaml(const std::string & path, std::string * controller_name_out)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    throw std::runtime_error("cannot parse '" + path + "': " + e.what());
  }
  if (!root.IsMap()) {
    throw std::runtime_error("'" + path + "' is not a YAML mapping");
  }

  // The kontrolem block: a non-controller_manager entry whose ros__parameters
  // carries control_law / control_laws (an estimator block, e.g., has neither).
  for (const auto & kv : root) {
    const std::string name = kv.first.as<std::string>();
    if (name == "controller_manager" || !kv.second.IsMap()) continue;
    const YAML::Node params = kv.second["ros__parameters"];
    if (!params || !params.IsMap()) continue;
    if (!params["control_law"] && !params["control_laws"]) continue;
    FlatConfig out;
    flatten(params, "", out);
    if (controller_name_out) *controller_name_out = name;
    return out;
  }
  throw std::runtime_error(
    "'" + path + "': no controller block with a control_law/control_laws found "
    "(expected e.g. kontrolem_controller: ros__parameters: control_law: ...)");
}

}  // namespace kontrolem_workbench
