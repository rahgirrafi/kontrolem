// LqrController — the first controller in the contract: a precomputed static
// state-feedback gain. synthesize() linearizes at the operating point and
// solves CARE (offline); compute() is a matrix-vector product (per tick, no
// allocation).
//
// Accepts TWO dialects — Regulation (fixed setpoint) AND Tracking (time-varying
// reference sampled each tick): the same gain law u = u_eq - K(x - x_ref)
// serves both, only x_ref differs. This is the framework's proof that dialect
// negotiation works for more than one dialect behind one controller.
#ifndef KONTROLEM_CONTROLLERS__LQR_CONTROLLER_HPP_
#define KONTROLEM_CONTROLLERS__LQR_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "kontrolem_control/controller.hpp"

namespace kontrolem_controllers
{

using namespace kontrolem_control;

/// The artifact produced offline by LqrController::synthesize.
struct LqrSynthesis : Synthesis
{
  Eigen::MatrixXd K;      ///< m x 2nv static gain
  Eigen::VectorXd u_eq;   ///< m actuated-joint feedforward at the operating point
  Eigen::VectorXd q_eq;   ///< nq operating point (linearization / validity center)
  std::vector<int> act_v; ///< actuated velocity-DOF indices (columns of B, rows of u)
};

class LqrController : public Controller
{
public:
  /// Tuning lives in the constructor (see DEVELOPMENT: the base synthesize()
  /// signature drops a generic `settings` blob in favor of typed per-controller
  /// design params).
  LqrController(
    std::vector<std::string> actuated_joints, Eigen::MatrixXd Q, Eigen::MatrixXd R,
    double q_dev_max = 0.5);

  Capabilities capabilities() const override;
  std::unique_ptr<Synthesis> synthesize(
    const RobotModel & model, const ControlProblem & problem) const override;
  void configure(
    const RobotModel & model, const Synthesis & synthesis,
    const ControlProblem & problem) override;
  const Command & compute(const State & state, const ControlProblem & problem, double dt) override;
  const Status & status() const override { return status_; }

  /// Diagnostics/tests: the synthesized gain (valid after configure()).
  const Eigen::MatrixXd & gain() const { return K_; }

private:
  // Design inputs (constructor).
  std::vector<std::string> actuated_;
  Eigen::MatrixXd Q_, R_;
  double q_dev_max_;

  // Loaded at configure().
  Eigen::MatrixXd K_;
  Eigen::VectorXd u_eq_, q_eq_;

  double qtrace_{1.0};  // trace(Q) — normalizer for the Q-weighted trust distance

  // Preallocated per-tick buffers (compute() stays allocation-free).
  Eigen::VectorXd error_;
  Eigen::VectorXd qref_buf_, vref_buf_, aref_buf_, tauff_buf_;  // Tracking: sampled reference
  Eigen::VectorXd dev_, qdev_;  // state deviation from the operating point + Q*dev
  Command command_;
  Status status_;
};

}  // namespace kontrolem_controllers

#endif  // KONTROLEM_CONTROLLERS__LQR_CONTROLLER_HPP_
