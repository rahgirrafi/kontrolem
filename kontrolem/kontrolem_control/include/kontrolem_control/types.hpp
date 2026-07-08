// Kontrol'Em v2 — plain data types crossing the controller boundary.
// Eigen-only; no ROS, no Pinocchio. These are what the runtime hands a
// controller (State), what it gets back (Command), and the health signal the
// Supervisor reads (Status).
#ifndef KONTROLEM_CONTROL__TYPES_HPP_
#define KONTROLEM_CONTROL__TYPES_HPP_

#include <Eigen/Dense>

namespace kontrolem_control
{

/// Current world state handed to the controller each tick. Minimal for the
/// fixed-base slice (full joint state); this is the struct that grows later
/// (base pose/twist on SE(3), contacts, external wrench) without changing the
/// controller call shape.
struct State
{
  Eigen::VectorXd q;   ///< configuration
  Eigen::VectorXd v;   ///< generalized velocity
  double t{0.0};       ///< controller clock [s]
};

/// Controller output: generalized effort/torque on the actuated joints, in the
/// order the controller declares via its capabilities.
struct Command
{
  Eigen::VectorXd tau;
};

/// Health / trust of the most recent compute(), for the Supervisor. Kept
/// allocation-free (no std::string) so it is safe to fill every tick. The
/// meaning is paradigm-specific behind a common shape:
///   LQR : ok = state inside the linearization region; margin = distance to it.
///   QP  : ok = solve feasible;                         margin = constraint slack.
struct Status
{
  bool ok{true};
  double margin{0.0};  ///< >= 0 means "trustworthy"; each paradigm defines the metric
};

}  // namespace kontrolem_control

#endif  // KONTROLEM_CONTROL__TYPES_HPP_
