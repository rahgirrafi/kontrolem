// Kontrol'Em v2 — THE controller contract.
//
// The entire architecture rests on this one shape fitting both a precomputed
// static-gain controller (LQR: compute() is a gemv) and a structurally
// different online-solved controller (QP task-space: compute() builds and
// solves an optimization). If that holds, the plugin boundary is validated.
//
// Multi-phase lifecycle (the offline/online seam is temporal, not two APIs):
//   synthesize (offline / heavy, may run in a design tool)
//     -> configure (at load, on the deployed instance)
//       -> compute (per tick, real-time, allocation-free)
//
// ROS-free: this header pulls in only Eigen, kontrolem_model, and our own
// plain types. No rclcpp / ros2_control anywhere.
#ifndef KONTROLEM_CONTROL__CONTROLLER_HPP_
#define KONTROLEM_CONTROL__CONTROLLER_HPP_

#include <memory>
#include <vector>

#include "kontrolem_control/problem.hpp"
#include "kontrolem_control/synthesis.hpp"
#include "kontrolem_control/types.hpp"
#include "kontrolem_model/robot_model.hpp"

namespace kontrolem_control
{

using kontrolem_model::RobotModel;

/// What a controller consumes / requires. The runtime uses this to wire the
/// right problem dialect and state, and to reject a mismatch before compute().
/// Grows later (required non-joint state, command type, needs-estimator, ...).
struct Capabilities
{
  std::vector<Dialect> accepted_dialects;
  bool needs_velocity_state{true};
};

/// The plugin contract every Kontrol'Em controller implements.
class Controller
{
public:
  virtual ~Controller() = default;

  /// Declared capabilities (accepted dialects, state needs). Cheap; may be
  /// called before configure().
  virtual Capabilities capabilities() const = 0;

  /// Offline / occasional heavy work -> a serializable Synthesis (the artifact).
  /// Pure with respect to the instance, so it can run anywhere (e.g. a design
  /// tool). An online-solved controller returns a trivial Synthesis.
  virtual std::unique_ptr<Synthesis> synthesize(
    const RobotModel & model, const ControlProblem & problem) const = 0;

  /// Load the artifact onto this instance: allocate buffers, set up any solver,
  /// and retain whatever handles per-tick work needs (e.g. the model, for a
  /// controller that queries dynamics online). NOT real-time.
  virtual void configure(
    const RobotModel & model, const Synthesis & synthesis, const ControlProblem & problem) = 0;

  /// The unifying per-tick call. MUST be allocation-free (the invariant under
  /// test). Returns a reference to an internally-owned Command buffer so no
  /// allocation happens on the return path.
  ///
  /// LIFETIME: the returned reference is valid only until the next compute()
  /// call on this controller (it aliases an internal buffer). Consume/copy it
  /// before calling compute() again — the runtime does this naturally each tick.
  virtual const Command & compute(
    const State & state, const ControlProblem & problem, double dt) = 0;

  /// Trust / health of the most recent compute(), for the Supervisor.
  virtual const Status & status() const = 0;

  /// Called by the Supervisor at the instant this controller becomes active after
  /// a switch, with the current world state. A controller that carries internal
  /// state (e.g. LQG's observer estimate) uses it to initialize that state so it
  /// activates ALREADY-CONVERGED — a bumpless handoff (the spike measured this
  /// cutting an LQG reactivation bump from 7.8 N to 0.33 N). Default: no-op, which
  /// is correct for a stateless law (LQR / QP / MPC) that reads the full state
  /// each tick and has nothing to seed.
  virtual void on_activate(const State & /*state*/, const ControlProblem & /*problem*/) {}
};

/// Wiring-time capability check: the runtime calls this once, before
/// configure()/compute(), so the per-tick dialect narrowing inside compute()
/// can be a bare static_cast.
inline bool accepts(const Controller & controller, const ControlProblem & problem)
{
  for (const auto dialect : controller.capabilities().accepted_dialects) {
    if (dialect == problem.kind()) {
      return true;
    }
  }
  return false;
}

}  // namespace kontrolem_control

#endif  // KONTROLEM_CONTROL__CONTROLLER_HPP_
