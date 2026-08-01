// Human-readable rendering of a law's ParameterSpec (config-as-data, M16) for
// the `describe` and `docs` subcommands. ROS-free: consumes only the contract
// types, so the formatting is unit-testable off-middleware.
#ifndef KONTROLEM_WORKBENCH__SPEC_FORMAT_HPP_
#define KONTROLEM_WORKBENCH__SPEC_FORMAT_HPP_

#include <string>

#include "kontrolem_control/params.hpp"

namespace kontrolem_workbench
{

/// Short type name of a ParamValue's active alternative:
/// "double" | "int" | "bool" | "string" | "double[]" | "int[]" | "string[]".
std::string type_name(const kontrolem_control::ParamValue & v);

/// The value as a YAML-compatible literal (e.g. 100.0, true, "trot", [1.0, 2.0]).
std::string value_str(const kontrolem_control::ParamValue & v);

/// The `describe <law>` table: one row per parameter (name, type, default,
/// description), aligned for a terminal.
std::string format_spec(const std::string & law, const kontrolem_control::ParameterSpec & spec);

/// The `docs` output for one law: a markdown section with the parameter table,
/// generated from the schema so the reference docs cannot drift from the code.
std::string format_spec_markdown(
  const std::string & law, const kontrolem_control::ParameterSpec & spec);

}  // namespace kontrolem_workbench

#endif  // KONTROLEM_WORKBENCH__SPEC_FORMAT_HPP_
