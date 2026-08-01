// YAML <-> config-as-data codec: reads a ros2_control controllers.yaml into a
// flat, typed key->value map (the workbench's view of a deployment config).
// ROS-free — yaml-cpp only.
#ifndef KONTROLEM_WORKBENCH__CODEC_HPP_
#define KONTROLEM_WORKBENCH__CODEC_HPP_

#include <map>
#include <string>

#include "kontrolem_control/params.hpp"

namespace kontrolem_workbench
{

/// One parsed YAML value. The ParamValue's variant alternative is the *inferred*
/// YAML type (ROS parameter typing is strict, so lint compares it against the
/// schema's type exactly — `1` is an int even where a double was wanted).
struct RawValue
{
  kontrolem_control::ParamValue value;
  bool is_empty_array = false;  ///< `[]` matches ANY array type (inference can't tell)
};

/// A controller's `ros__parameters` block, flattened to dotted keys.
using FlatConfig = std::map<std::string, RawValue>;

/// Load the kontrolem controller block from a controllers.yaml: the top-level
/// entry (excluding controller_manager) whose ros__parameters contains
/// `control_law` or `control_laws`. Nested maps flatten to dotted keys.
/// Throws std::runtime_error with a clear message on parse failure / no block.
/// If `controller_name_out` is non-null it receives the block's name.
FlatConfig load_controllers_yaml(
  const std::string & path, std::string * controller_name_out = nullptr);

}  // namespace kontrolem_workbench

#endif  // KONTROLEM_WORKBENCH__CODEC_HPP_
