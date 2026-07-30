// Kontrol'Em v2 — config-as-data: a ROS-free description of a controller's
// tunable parameters and their values.
//
// The whole point (M16): a controller's PARAMETER SCHEMA is plain typed data
// (ParameterSpec), and its configured VALUES are a plain typed map
// (ParameterMap) — neither touches rclcpp. So a controller can be configured,
// validated, and unit-tested with NO middleware present, and the L4 runtime is
// the only place that translates ROS parameters <-> these types. This is what
// makes configuration middleware-isolated, not just compute().
//
// Not on any real-time path — all of this is configure-time — so std::variant /
// std::map / std::string are fine here.
#ifndef KONTROLEM_CONTROL__PARAMS_HPP_
#define KONTROLEM_CONTROL__PARAMS_HPP_

#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace kontrolem_control
{

/// One parameter value. The active alternative IS the parameter's type; this is
/// the closed set the ROS parameter world maps onto (scalar doubles/ints/bools/
/// strings and their arrays cover every controller parameter we declare today).
using ParamValue = std::variant<
  double, std::int64_t, bool, std::string, std::vector<double>, std::vector<std::int64_t>,
  std::vector<std::string>>;

/// One row of a schema: a parameter's name, its default (whose variant type also
/// pins the parameter's type), and a human-readable description for docs/tools.
struct ParamDesc
{
  std::string name;
  ParamValue default_value;
  std::string description;
};

/// A controller's parameter schema, as data — enumerable without running it.
using ParameterSpec = std::vector<ParamDesc>;

/// Configured parameter values. Seed it from a ParameterSpec (every default
/// present), then overlay the actual configured values with set(); the typed
/// getters then always resolve — a missing key falls back to the seeded default.
class ParameterMap
{
public:
  ParameterMap() = default;

  /// Seed with every default in the spec (so getters resolve even if nothing is
  /// overlaid).
  explicit ParameterMap(const ParameterSpec & spec)
  {
    for (const auto & d : spec) {
      values_[d.name] = d.default_value;
    }
  }

  /// Overlay a configured value (the L4 runtime calls this per parameter it read
  /// from ROS; a unit test calls it directly).
  void set(const std::string & name, ParamValue value) { values_[name] = std::move(value); }

  bool has(const std::string & name) const { return values_.find(name) != values_.end(); }

  double double_at(const std::string & name) const { return at<double>(name); }
  std::int64_t int_at(const std::string & name) const { return at<std::int64_t>(name); }
  bool bool_at(const std::string & name) const { return at<bool>(name); }
  const std::string & string_at(const std::string & name) const { return at<std::string>(name); }
  const std::vector<double> & double_array_at(const std::string & name) const
  {
    return at<std::vector<double>>(name);
  }
  const std::vector<std::int64_t> & int_array_at(const std::string & name) const
  {
    return at<std::vector<std::int64_t>>(name);
  }
  const std::vector<std::string> & string_array_at(const std::string & name) const
  {
    return at<std::vector<std::string>>(name);
  }

private:
  template <typename T>
  const T & at(const std::string & name) const
  {
    const auto it = values_.find(name);
    if (it == values_.end()) {
      throw std::out_of_range("kontrolem_control: parameter '" + name + "' is not set");
    }
    const T * p = std::get_if<T>(&it->second);
    if (p == nullptr) {
      throw std::runtime_error("kontrolem_control: parameter '" + name + "' has the wrong type");
    }
    return *p;
  }

  std::map<std::string, ParamValue> values_;
};

}  // namespace kontrolem_control

#endif  // KONTROLEM_CONTROL__PARAMS_HPP_
