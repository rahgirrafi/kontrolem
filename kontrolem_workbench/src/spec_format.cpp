#include "kontrolem_workbench/spec_format.hpp"

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <variant>
#include <vector>

namespace kontrolem_workbench
{
namespace kc = kontrolem_control;

std::string type_name(const kc::ParamValue & v)
{
  if (std::holds_alternative<double>(v)) return "double";
  if (std::holds_alternative<std::int64_t>(v)) return "int";
  if (std::holds_alternative<bool>(v)) return "bool";
  if (std::holds_alternative<std::string>(v)) return "string";
  if (std::holds_alternative<std::vector<double>>(v)) return "double[]";
  if (std::holds_alternative<std::vector<std::int64_t>>(v)) return "int[]";
  return "string[]";
}

namespace
{
std::string double_str(double d)
{
  std::ostringstream os;
  os << d;
  // A bare integer-valued double still needs to read as a double in YAML.
  const std::string s = os.str();
  return s.find_first_of(".eEn") == std::string::npos ? s + ".0" : s;
}
}  // namespace

std::string value_str(const kc::ParamValue & v)
{
  std::ostringstream os;
  if (const auto * d = std::get_if<double>(&v)) {
    os << double_str(*d);
  } else if (const auto * i = std::get_if<std::int64_t>(&v)) {
    os << *i;
  } else if (const auto * b = std::get_if<bool>(&v)) {
    os << (*b ? "true" : "false");
  } else if (const auto * s = std::get_if<std::string>(&v)) {
    os << '"' << *s << '"';
  } else if (const auto * da = std::get_if<std::vector<double>>(&v)) {
    os << '[';
    for (std::size_t k = 0; k < da->size(); ++k) os << (k ? ", " : "") << double_str((*da)[k]);
    os << ']';
  } else if (const auto * ia = std::get_if<std::vector<std::int64_t>>(&v)) {
    os << '[';
    for (std::size_t k = 0; k < ia->size(); ++k) os << (k ? ", " : "") << (*ia)[k];
    os << ']';
  } else {
    const auto & sa = std::get<std::vector<std::string>>(v);
    os << '[';
    for (std::size_t k = 0; k < sa.size(); ++k) os << (k ? ", " : "") << '"' << sa[k] << '"';
    os << ']';
  }
  return os.str();
}

std::string format_spec(const std::string & law, const kc::ParameterSpec & spec)
{
  std::size_t w_name = 9, w_type = 4, w_def = 7;  // header widths ("parameter","type","default")
  for (const auto & d : spec) {
    w_name = std::max(w_name, d.name.size());
    w_type = std::max(w_type, type_name(d.default_value).size());
    w_def = std::max(w_def, value_str(d.default_value).size());
  }
  std::ostringstream os;
  os << "control_law: " << law << "  (" << spec.size() << " parameters)\n";
  os << "  " << std::left << std::setw(static_cast<int>(w_name)) << "parameter" << "  "
     << std::setw(static_cast<int>(w_type)) << "type" << "  "
     << std::setw(static_cast<int>(w_def)) << "default" << "  description\n";
  for (const auto & d : spec) {
    os << "  " << std::left << std::setw(static_cast<int>(w_name)) << d.name << "  "
       << std::setw(static_cast<int>(w_type)) << type_name(d.default_value) << "  "
       << std::setw(static_cast<int>(w_def)) << value_str(d.default_value) << "  "
       << d.description << "\n";
  }
  return os.str();
}

std::string format_spec_markdown(const std::string & law, const kc::ParameterSpec & spec)
{
  std::ostringstream os;
  os << "## " << law << " (`control_law: " << law << "`)\n\n"
     << "| Parameter | Type | Default | Meaning |\n|---|---|---|---|\n";
  for (const auto & d : spec) {
    // `[]` renders clearer as the word "empty" in a table cell.
    const std::string def = value_str(d.default_value);
    os << "| `" << d.name << "` | " << type_name(d.default_value) << " | "
       << (def == "[]" ? "`[]`" : "`" + def + "`") << " | " << d.description << " |\n";
  }
  return os.str();
}

}  // namespace kontrolem_workbench
