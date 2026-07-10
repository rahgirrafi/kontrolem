// LpvController — gain-scheduling / LPV: one controller covering an operating
// ENVELOPE by scheduling a family of LQR designs on measured state (Part C4, the
// planned M1.5). synthesize() designs an LQR (K, u_eq) at every node of a regular
// grid over the scheduling variables (a family of linearizations from the model
// service); compute() reads the current scheduling variable, MULTILINEARLY
// interpolates (K, u_eq) from the table (allocation-free), and applies the same
// gain law as LQR: u = u_eq(theta) - K(theta) (x - x_ref).
//
// This is the pragmatic point-wise-design-plus-interpolation LPV (Shamma-Athans):
// each node is locally optimal, interpolation covers between; there is no
// cross-envelope stability certificate (that needs LMI-LPV, a later upgrade behind
// the same runtime). status() reports envelope membership: trustworthy iff the
// scheduling variable is inside the designed grid.
//
// Layer ownership: LPV is ONE continuously-scheduled controller (this L3 plugin),
// NOT the Supervisor's discrete switch between whole controllers (that is L4).
#ifndef KONTROLEM_CONTROLLERS__LPV_CONTROLLER_HPP_
#define KONTROLEM_CONTROLLERS__LPV_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "kontrolem_control/controller.hpp"

namespace kontrolem_controllers
{

using namespace kontrolem_control;

/// One scheduling axis: a uniform grid of `n` operating points on a q-coordinate.
struct SchedAxis
{
  int q_index;     ///< which configuration coordinate schedules this axis
  double min;      ///< grid lower bound (envelope)
  double max;      ///< grid upper bound (envelope)
  int n;           ///< number of nodes (>= 2)
};

/// The artifact: a regular grid of per-node LQR designs over the scheduling axes.
struct LpvSynthesis : Synthesis
{
  std::vector<SchedAxis> axes;          ///< D scheduling axes (grid = product of these)
  std::vector<Eigen::MatrixXd> K;       ///< per-node gain (m x 2nv), row-major over the grid
  std::vector<Eigen::VectorXd> u_eq;    ///< per-node feedforward (m)
  std::vector<int> act_v;               ///< actuated velocity-DOF indices
};

class LpvController : public Controller
{
public:
  /// axes: the scheduling grid (>=1 axis). Between nodes the design is multilinearly
  /// interpolated; outside the grid envelope status() reports not-ok.
  LpvController(
    std::vector<std::string> actuated_joints, Eigen::MatrixXd Q, Eigen::MatrixXd R,
    std::vector<SchedAxis> axes);

  Capabilities capabilities() const override;
  std::unique_ptr<Synthesis> synthesize(
    const RobotModel & model, const ControlProblem & problem) const override;
  void configure(
    const RobotModel & model, const Synthesis & synthesis,
    const ControlProblem & problem) override;
  const Command & compute(const State & state, const ControlProblem & problem, double dt) override;
  const Status & status() const override { return status_; }

  /// Diagnostics/tests: number of designed grid nodes.
  std::size_t node_count() const { return K_.size(); }

private:
  // Design inputs.
  std::vector<std::string> actuated_;
  Eigen::MatrixXd Q_, R_;
  std::vector<SchedAxis> axes_;

  // Loaded at configure() (the interpolation table).
  std::vector<Eigen::MatrixXd> K_;      ///< per-node gains, row-major over the grid
  std::vector<Eigen::VectorXd> u_eq_;

  // Preallocated per-tick buffers (compute() stays allocation-free).
  Eigen::MatrixXd K_interp_;            ///< interpolated gain this tick
  Eigen::VectorXd u_interp_;            ///< interpolated feedforward this tick
  Eigen::VectorXd error_;              ///< x - x_ref
  Eigen::VectorXd qref_buf_, vref_buf_, aref_buf_, tauff_buf_;  // Tracking reference
  std::vector<int> lo_;                ///< per-axis lower node index
  std::vector<double> frac_;           ///< per-axis interpolation fraction
  Command command_;
  Status status_;
};

}  // namespace kontrolem_controllers

#endif  // KONTROLEM_CONTROLLERS__LPV_CONTROLLER_HPP_
