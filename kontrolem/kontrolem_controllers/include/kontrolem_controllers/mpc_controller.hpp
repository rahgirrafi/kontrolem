// MpcController — linear model-predictive control: constrained, receding-horizon
// optimal control solved online as a QP every tick. The fourth paradigm behind
// the one contract, and the one the plan calls out as the payoff of the
// offline/online seam:
//   * synthesize()/configure() (heavy, once): linearize + discretize the plant,
//     compute the discrete-LQR terminal cost (DARE), and CONDENSE the horizon
//     problem into a dense QP in the input sequence U = [u_0 .. u_{N-1}].
//   * compute() (per tick): x0 <- current deviation state; the QP gradient is
//     linear in x0 (q = G x0); warm-solve via the OSQP seam; apply u_0 (recede).
//
// LTI condensing: x_k = A_d^k x0 + sum A_d^{k-1-j} B_d u_j  =>  X = Sx x0 + Su U.
//   H = 2(Su^T Qbar Su + Rbar)  (constant),  q = (2 Su^T Qbar Sx) x0 = G x0.
// Input box -tau_max <= u_k <= tau_max is a hard QP constraint (not post-clamp).
// Consumes Regulation. Reference: Rawlings/Mayne/Diehl; Mayne et al. (2000).
#ifndef KONTROLEM_CONTROLLERS__MPC_CONTROLLER_HPP_
#define KONTROLEM_CONTROLLERS__MPC_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "kontrolem_control/controller.hpp"
#include "kontrolem_controllers/qp_solver.hpp"

namespace kontrolem_controllers
{

using namespace kontrolem_control;

/// Offline artifact: the condensed QP for the operating point.
struct MpcSynthesis : Synthesis
{
  Eigen::MatrixXd H;      ///< Nm x Nm    QP Hessian (constant for LTI)
  Eigen::MatrixXd G;      ///< Nm x 2nv   gradient map:  q += G * x0_dev
  Eigen::MatrixXd M_ref;  ///< Nm x N*2nv Tracking gradient map: q -= M_ref * Xref_dev
  Eigen::VectorXd u_eq;   ///< m          operating-point feedforward
  Eigen::VectorXd q_eq;   ///< nq         operating point
  std::vector<int> act_v; ///< actuated velocity-DOF indices
  int horizon{0};         ///< N
  int m{0};               ///< inputs per step
  double dt_mpc{0.0};     ///< horizon step (to sample the reference ahead)
  double tau_max{0.0};
};

class MpcController : public Controller
{
public:
  MpcController(
    std::vector<std::string> actuated_joints, Eigen::MatrixXd Q, Eigen::MatrixXd R,
    int horizon, double dt_mpc, double tau_max);

  Capabilities capabilities() const override;
  std::unique_ptr<Synthesis> synthesize(
    const RobotModel & model, const ControlProblem & problem) const override;
  void configure(
    const RobotModel & model, const Synthesis & synthesis,
    const ControlProblem & problem) override;
  const Command & compute(const State & state, const ControlProblem & problem, double dt) override;
  const Status & status() const override { return status_; }

private:
  // Design inputs.
  std::vector<std::string> actuated_;
  Eigen::MatrixXd Q_, R_;
  int horizon_;
  double dt_mpc_, tau_max_;

  // Loaded at configure().
  Eigen::MatrixXd G_, M_ref_;  // Nm x 2nv, Nm x N*2nv
  Eigen::VectorXd u_eq_, q_eq_;
  int m_{0}, horizon_n_{0};
  double dt_mpc_loaded_{0.0};

  // Solver + preallocated per-tick buffers.
  std::unique_ptr<QpSolver> qp_;
  Eigen::MatrixXd A_qp_;   // Nm x Nm identity (box constraint)
  Eigen::VectorXd l_, u_;  // Nm box bounds (constant)
  Eigen::VectorXd x0_;     // 2nv deviation state
  Eigen::VectorXd qbuf_;   // Nm QP gradient
  Eigen::VectorXd xref_;   // N*2nv stacked reference deviation (Tracking)
  Eigen::VectorXd qs_, vs_, as_, taus_;  // per-sample reference scratch (Tracking)
  Command command_;
  Status status_;
};

}  // namespace kontrolem_controllers

#endif  // KONTROLEM_CONTROLLERS__MPC_CONTROLLER_HPP_
