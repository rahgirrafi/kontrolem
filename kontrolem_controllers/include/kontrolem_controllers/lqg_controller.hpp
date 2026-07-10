// LqgController — the THIRD structural form behind the one contract: a dynamic
// output-feedback compensator. Where LQR is a static gain (no internal state)
// and the QP is a stateless per-tick solve, LQG carries an internal observer
// state that it steps every tick — proving compute() fits a controller with
// memory, not just a memoryless map.
//
// Method (separation principle): control gain K from the control CARE (as LQR)
// + steady-state Kalman gain L from the dual/filter CARE (same care.hpp, with
// A->A^T, B->C^T, Q->W, R->V). Measures POSITIONS ONLY (C = [I 0]); velocity is
// ESTIMATED, so this genuinely demonstrates output feedback. Consumes Regulation.
#ifndef KONTROLEM_CONTROLLERS__LQG_CONTROLLER_HPP_
#define KONTROLEM_CONTROLLERS__LQG_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "kontrolem_control/controller.hpp"

namespace kontrolem_controllers
{

using namespace kontrolem_control;

/// The artifact produced offline by LqgController::synthesize. Stored in the
/// observer-form deviation realization so compute() is a couple of gemvs:
///   x̂̃ ← x̂̃ + dt (A_obs x̂̃ + B_act ũ + L ỹ),   ũ = -K x̂̃,   u = u_eq + ũ
/// with A_obs = A - L C.
struct LqgSynthesis : Synthesis
{
  Eigen::MatrixXd A_obs;  ///< 2nv x 2nv   (A - L C)
  Eigen::MatrixXd B_act;  ///< 2nv x m     actuated columns of B
  Eigen::MatrixXd K;      ///< m x 2nv     control gain
  Eigen::MatrixXd L;      ///< 2nv x nq    Kalman gain
  Eigen::VectorXd u_eq;   ///< m           operating-point feedforward
  Eigen::VectorXd q_eq;   ///< nq          operating point (also y_eq = C x_eq)
  std::vector<int> act_v; ///< actuated velocity-DOF indices
};

class LqgController : public Controller
{
public:
  /// Q,R weight the control LQR; W,V are the process/measurement noise
  /// covariances of the Kalman filter (W: 2nv x 2nv, V: nq x nq).
  LqgController(
    std::vector<std::string> actuated_joints, Eigen::MatrixXd Q, Eigen::MatrixXd R,
    Eigen::MatrixXd W, Eigen::MatrixXd V, double innov_max = 0.5);

  Capabilities capabilities() const override;
  std::unique_ptr<Synthesis> synthesize(
    const RobotModel & model, const ControlProblem & problem) const override;
  void configure(
    const RobotModel & model, const Synthesis & synthesis,
    const ControlProblem & problem) override;
  const Command & compute(const State & state, const ControlProblem & problem, double dt) override;
  const Status & status() const override { return status_; }

  /// Bumpless activation (Supervisor hook): seed the observer from the current
  /// state so the compensator resumes already-converged instead of from a stale
  /// estimate. Delegates to seed_from_state().
  void on_activate(const State & state, const ControlProblem & problem) override
  {
    seed_from_state(state, problem);
  }

  /// LQG carries an observer estimate (xhat_) across ticks, so it is NOT
  /// stateless — the Supervisor must not shadow-run it for health evaluation.
  bool stateless() const override { return false; }

  /// Diagnostics/tests: the current observer estimate (deviation coords).
  const Eigen::VectorXd & estimate() const { return xhat_; }

  /// Bumpless activation. Seed the internal observer estimate from a known full
  /// state (deviation coords [q - q_ref; v - v_ref]) so the compensator starts
  /// ALREADY-CONVERGED when a Supervisor hands it control mid-run — instead of
  /// from xhat = 0, which injects an observer startup transient (and a command
  /// jump) whenever the plant isn't at the operating point at the switch instant.
  /// A stateless law (LQR/QP/MPC) needs no analogue; this is the state-carrying
  /// controller's half of a heterogeneous bumpless transfer.
  void seed_from_state(const State & state, const ControlProblem & problem);

private:
  // Design inputs (constructor).
  std::vector<std::string> actuated_;
  Eigen::MatrixXd Q_, R_, W_, V_;
  double innov_max_;

  // Loaded at configure().
  Eigen::MatrixXd A_obs_, B_act_, K_, L_;
  Eigen::VectorXd u_eq_, q_eq_;

  // Internal observer state + preallocated per-tick buffers (alloc-free compute).
  Eigen::VectorXd xhat_;   ///< 2nv   observer estimate, deviation coords
  Eigen::VectorXd ytil_;   ///< nq    measured position deviation
  Eigen::VectorXd util_;   ///< m     control deviation
  Eigen::VectorXd xdot_;   ///< 2nv   observer time-derivative
  Command command_;
  Status status_;
};

}  // namespace kontrolem_controllers

#endif  // KONTROLEM_CONTROLLERS__LQG_CONTROLLER_HPP_
