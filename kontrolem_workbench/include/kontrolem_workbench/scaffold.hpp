// Generate a ready-to-edit deployment config from a law's ParameterSpec and a
// robot model: every parameter present at its default with its description as a
// comment, actuated_joints pre-filled from the URDF. ROS-free.
#ifndef KONTROLEM_WORKBENCH__SCAFFOLD_HPP_
#define KONTROLEM_WORKBENCH__SCAFFOLD_HPP_

#include <string>
#include <vector>

#include "kontrolem_control/params.hpp"

namespace kontrolem_workbench
{

/// A commented controllers.yaml for `law` (controller_manager block + the
/// kontrolem_controller block). `joint_names` seeds actuated_joints; `nq`
/// sizes the q_ref placeholder.
std::string scaffold_yaml(
  const std::string & law, const kontrolem_control::ParameterSpec & spec,
  const std::vector<std::string> & joint_names, int nq, int update_rate_hz = 200);

/// A minimal matching launch-file stub (assumes the kontrolem_bringup
/// custom-sim harness; a real robot swaps in its own hardware launch).
std::string scaffold_launch(const std::string & urdf_file, const std::string & yaml_file);

}  // namespace kontrolem_workbench

#endif  // KONTROLEM_WORKBENCH__SCAFFOLD_HPP_
