// Offline validation of a deployment config against the installed laws'
// ParameterSpecs (the M16 config-as-data payoff, out of the runtime and into a
// terminal/CI command). ROS-free: factories arrive as plain objects — the CLI
// passes pluginlib-discovered instances, tests pass directly-constructed ones.
#ifndef KONTROLEM_WORKBENCH__LINT_HPP_
#define KONTROLEM_WORKBENCH__LINT_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "kontrolem_control/controller_factory.hpp"
#include "kontrolem_model/robot_model.hpp"
#include "kontrolem_workbench/codec.hpp"

namespace kontrolem_workbench
{

struct LintResult
{
  std::vector<std::string> errors;    ///< would fail (or silently corrupt) a launch
  std::vector<std::string> warnings;  ///< legal but suspicious
  bool ok() const { return errors.empty(); }
};

using FactoryMap = std::map<std::string, std::shared_ptr<kontrolem_control::ControllerFactory>>;

/// Validate `config` (a parsed controllers.yaml block) against the installed
/// laws. Checks: selected law(s) exist; every key under a law's parameter
/// prefix is in that law's spec (with near-miss suggestions); value types match
/// the spec exactly (ROS parameter typing is strict: `1` is not a double);
/// framework parameters are known and well-typed; parameters for a
/// NON-selected law warn (a config may keep them for switching).
///
/// With a RobotModel (`model` non-null) it goes deeper — the checks
/// on_configure would do, without launching: actuated_joints/contact_frames
/// resolve in the model, q_ref/v_ref lengths match nq/nv, and each selected
/// law's factory `create()` is dry-run (rejecting e.g. a mis-shaped LPV grid).
LintResult lint(
  const FlatConfig & config, const FactoryMap & laws,
  const kontrolem_model::RobotModel * model = nullptr);

/// Build the (first) selected law from a config exactly as the runtime's
/// on_configure would: seed its ParameterMap from the spec, overlay the
/// config's values, assemble the BuildContext, create(). Throws on an unknown
/// law or a construction the factory rejects. `law_name_out`/`params_out`
/// (optional) receive the law's name and its fully-resolved parameters.
std::unique_ptr<kontrolem_control::Controller> build_from_config(
  const FlatConfig & config, const FactoryMap & laws, const kontrolem_model::RobotModel & model,
  std::string * law_name_out = nullptr, kontrolem_control::ParameterMap * params_out = nullptr);

}  // namespace kontrolem_workbench

#endif  // KONTROLEM_WORKBENCH__LINT_HPP_
