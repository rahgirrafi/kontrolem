// QpTaskSpaceController — the second controller in the contract, deliberately
// structurally different from LQR:
//   * synthesize() is a NO-OP (nothing precomputed).
//   * compute() queries the model's dynamics every tick and solves an
//     inverse-dynamics QP online, with an ACTIVE torque-limit inequality.
//
// QP (decision vars z = [qddot (nv); tau (m)]):
//   min  || qddot - qddot_des ||^2_W  +  rho ||tau||^2
//   s.t. M(q) qddot + h(q,v) = S^T tau           (dynamics equality)
//        -tau_max <= tau <= tau_max              (torque limit)
// with qddot_des = -kp (q - q_ref) - kd (v - v_ref)  (a PD task).
//
// Consumes the same Regulation dialect as LQR — same State, same problem, same
// compute() shape — so the only thing that differs is what happens inside.
#ifndef KONTROLEM_CONTROLLERS__QP_TASK_SPACE_CONTROLLER_HPP_
#define KONTROLEM_CONTROLLERS__QP_TASK_SPACE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "kontrolem_control/controller.hpp"
#include "kontrolem_controllers/qp_solver.hpp"

namespace kontrolem_controllers
{

using namespace kontrolem_control;

class QpTaskSpaceController : public Controller
{
public:
  QpTaskSpaceController(
    std::vector<std::string> actuated_joints, Eigen::VectorXd task_weight, double kp, double kd,
    double tau_max, double tau_reg = 1e-3);

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
  Eigen::VectorXd W_;  // nv task weights
  double kp_, kd_, tau_max_, tau_reg_;

  // Resolved at configure().
  const RobotModel * model_ = nullptr;  // retained handle for per-tick queries
  std::vector<int> act_v_;
  int nv_ = 0, m_ = 0, nz_ = 0, nc_ = 0;

  QpSolver qp_;

  // Preallocated so compute() is allocation-free: a reusable model Workspace
  // (U4 fix) and buffers the RT dynamics query writes into.
  std::unique_ptr<RobotModel::Workspace> ws_;
  Eigen::MatrixXd M_;
  Eigen::VectorXd h_;
  Eigen::MatrixXd A_;
  Eigen::VectorXd qcost_, l_, u_, qdd_des_;
  Command command_;
  Status status_;
};

}  // namespace kontrolem_controllers

#endif  // KONTROLEM_CONTROLLERS__QP_TASK_SPACE_CONTROLLER_HPP_
