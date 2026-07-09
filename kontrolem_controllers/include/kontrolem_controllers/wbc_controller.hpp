// WbcController — the flagship (M4): a floating-base, contact-aware QP whole-body
// controller. Like the QP task-space controller its synthesize() is a no-op and
// compute() solves an inverse-dynamics QP every tick, but here the decision set
// carries CONTACT FORCES and the constraints carry the full floating-base
// dynamics + friction cones — the structural jump the whole architecture exists
// to absorb behind the one contract.
//
// QP (decision z = [qddot (nv); lambda (3*nc); tau (m)]):
//   min  || qddot - qddot_des ||^2_W  +  w_f ||lambda||^2  +  w_t ||tau||^2
//   s.t. M qddot + h = S^T tau + J^T lambda          (floating-base dynamics)
//        J qddot = -gamma                            (feet don't accelerate)
//        |lambda_xy| <= mu lambda_z,  lambda_z >= 0  (friction pyramid, unilateral)
//        -tau_max <= tau <= tau_max                  (torque limits)
// with qddot_des = -Kp (q ⊖ q_ref) - Kd v            (frame-consistent PD task:
// base pose/orientation + posture, in the generalized tangent space).
//
// Consumes the Regulation dialect (q_ref = the standing posture) — the SAME
// problem type LQR/QP use, so a quadruped standing under a QP-WBC and a cart-pole
// under a gain both flow through the identical compute(state, problem, dt) shape.
// Contact is scheduled all-stance here (Part D: no locomotion; the sim provides
// ground-truth contact). Lives in kontrolem_controllers for now (like MpcController
// — a flagged deviation from the per-paradigm-package plan, B10).
#ifndef KONTROLEM_CONTROLLERS__WBC_CONTROLLER_HPP_
#define KONTROLEM_CONTROLLERS__WBC_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "kontrolem_control/controller.hpp"
#include "kontrolem_controllers/qp_solver.hpp"

namespace kontrolem_controllers
{

using namespace kontrolem_control;

class WbcController : public Controller
{
public:
  struct Gains
  {
    double kp_base{100.0};   ///< base pose/orientation stiffness
    double kd_base{20.0};    ///< base twist damping
    double kp_post{25.0};    ///< joint posture stiffness
    double kd_post{5.0};     ///< joint posture damping
    double w_base{100.0};    ///< task weight on the 6 base DoF
    double w_post{1.0};      ///< task weight on the joints
    double w_force{1e-4};    ///< contact-force regularization
    double w_tau{1e-4};      ///< torque regularization
    double mu{0.7};          ///< friction coefficient (pyramid)
    double tau_max{40.0};    ///< per-joint torque limit
  };

  WbcController(
    std::vector<std::string> contact_frames, std::vector<std::string> actuated_joints,
    Gains gains);

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
  std::vector<std::string> feet_;
  std::vector<std::string> actuated_;
  Gains g_;

  // Resolved at configure().
  const RobotModel * model_ = nullptr;
  std::vector<int> act_v_;              // generalized-velocity row of each actuated joint
  std::vector<std::size_t> feet_ids_;   // cached contact frame indices (no per-tick lookup)
  int nv_ = 0, nc_ = 0, m_ = 0;         // DoF, #contacts, #actuated
  int nz_ = 0, rows_ = 0;               // QP size
  int off_lambda_ = 0, off_tau_ = 0;    // column offsets in z
  int row_dyn_ = 0, row_con_ = 0, row_fric_ = 0, row_uni_ = 0, row_tau_ = 0;  // row offsets
  Eigen::VectorXd Kp_, Kd_;             // per-DoF PD gains (nv)

  QpSolver qp_;

  // Preallocated so compute() stays (mostly) allocation-free.
  std::unique_ptr<RobotModel::Workspace> ws_;
  Eigen::MatrixXd M_, J_;
  Eigen::VectorXd h_, gamma_, e_, qdd_des_, W_;
  Eigen::MatrixXd A_;
  Eigen::VectorXd qcost_, l_, u_;
  Command command_;
  Status status_;
};

}  // namespace kontrolem_controllers

#endif  // KONTROLEM_CONTROLLERS__WBC_CONTROLLER_HPP_
