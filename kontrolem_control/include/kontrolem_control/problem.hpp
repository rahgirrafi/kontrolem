// Kontrol'Em v2 — the capability-typed problem specification.
//
// compute() always takes the polymorphic base ControlProblem; a controller
// narrows to the dialect(s) it declared it accepts. The dialect match is
// checked once at wiring time (see accepts() in controller.hpp), so the per-tick
// narrowing is a free static_cast. New dialects are new derived types — there is
// no central variant to edit, which is what keeps the framework open to
// controllers we haven't written yet.
#ifndef KONTROLEM_CONTROL__PROBLEM_HPP_
#define KONTROLEM_CONTROL__PROBLEM_HPP_

#include <Eigen/Dense>

#include "kontrolem_control/gait.hpp"
#include "kontrolem_control/trajectory.hpp"

namespace kontrolem_control
{

/// Known problem dialects. TaskSpec is named so the enum is stable but not yet
/// defined (WBC/MPC).
enum class Dialect
{
  kRegulation,  ///< drive the state to a fixed setpoint
  kTracking,    ///< follow a time-varying reference
  kTaskSpec,    ///< weighted task hierarchy + constraints (later, WBC/MPC)
  kLocomotion,  ///< follow a gait plan (per-foot contact schedule + swing + base ref)
};

/// Base of every problem the framework can pose to a controller.
struct ControlProblem
{
  virtual ~ControlProblem() = default;
  virtual Dialect kind() const = 0;
};

/// The only dialect in this slice: regulate the plant to a fixed setpoint.
/// Both controllers under test consume this — LQR as a state error to feed the
/// gain, the QP task-space controller as a task to minimize under its torque
/// limit. Same problem, structurally different controllers: that is the test.
struct Regulation : ControlProblem
{
  Eigen::VectorXd q_ref;  ///< desired configuration
  Eigen::VectorXd v_ref;  ///< desired velocity (usually zero)

  Dialect kind() const override { return Dialect::kRegulation; }
};

/// Follow a time-varying reference produced by a TrajectorySource (which the
/// controller samples at the current time each tick). The source is referenced,
/// not owned — its lifetime is the caller's responsibility (the runtime owns it
/// for as long as the problem is active).
struct Tracking : ControlProblem
{
  const TrajectorySource * reference{nullptr};

  Dialect kind() const override { return Dialect::kTracking; }
};

/// Walk: follow a GaitSource (sampled each tick) that carries the per-foot contact
/// schedule + swing-foot target + base-pose reference. Consumed by the WBC, which
/// constrains stance feet, tracks the swing foot along its arc, and moves the base to
/// keep the CoM statically stable. The source is referenced, not owned (runtime-owned).
struct Locomotion : ControlProblem
{
  const GaitSource * gait{nullptr};

  Dialect kind() const override { return Dialect::kLocomotion; }
};

}  // namespace kontrolem_control

#endif  // KONTROLEM_CONTROL__PROBLEM_HPP_
