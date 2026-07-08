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

namespace kontrolem_control
{

/// Known problem dialects. Only Regulation exists in this slice; Tracking and
/// TaskSpec are named so the enum is stable, but not yet defined.
enum class Dialect
{
  kRegulation,  ///< drive the state to a fixed setpoint
  kTracking,    ///< follow a time-varying reference (later)
  kTaskSpec,    ///< weighted task hierarchy + constraints (later, WBC/MPC)
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

}  // namespace kontrolem_control

#endif  // KONTROLEM_CONTROL__PROBLEM_HPP_
