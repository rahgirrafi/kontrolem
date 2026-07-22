// KinematicGaitController (M13) — Example A: the model-FREE walking controller.
//
// The simplest way the framework can make a robot walk, and the pedagogical baseline of the
// "Go2 walks four ways" showcase. It carries NO dynamics, NO QP, and NO estimator: each tick it
// samples a gait plan (per-foot WORLD targets + a nominal, forward-advancing base), solves a
// per-leg inverse-kinematics for the joint angles that place each foot on its target, and emits
// a joint PD torque  τ = kp (q* − q) − kd q̇  on the effort interface. Because the IK pins the
// base to the SCHEDULED (nominal) pose rather than a measured one, the law is fully OPEN-LOOP in
// the base — no state estimator in the loop. This is the CHAMP lineage, expressed as a Controller
// plugin behind the same synthesize→configure→compute contract as LQR/WBC/MPC.
//
// Scope: a point-foot leg with exactly 3 joints (a standard quadruped like the Go2). The 3×3 leg
// Jacobian makes the IK a fixed-size, allocation-free solve. A >3-DoF leg (humanoid) needs the
// documented least-squares generalization — a knob, not a rewrite.
#ifndef KONTROLEM_CONTROLLERS__KINEMATIC_GAIT_CONTROLLER_HPP_
#define KONTROLEM_CONTROLLERS__KINEMATIC_GAIT_CONTROLLER_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "kontrolem_control/controller.hpp"

namespace kontrolem_controllers
{

using namespace kontrolem_control;

class KinematicGaitController : public Controller
{
public:
  struct Gains
  {
    double kp{60.0};             ///< joint position stiffness (→ torque)
    double kd{2.0};              ///< joint velocity damping
    double tau_max{23.7};        ///< per-joint torque clamp
    int ik_max_iter{20};         ///< per-leg Gauss-Newton cap (hard-RT bound)
    double ik_tol{1e-6};         ///< foot-position convergence (m)
    double ik_step_clamp{0.5};   ///< max |Δq| per IK iteration (rad) — singularity guard
  };

  KinematicGaitController(
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

  /// The current IK target configuration (full nq): the scheduled base + the joint angles the
  /// IK solved to place each foot on its gait target. Introspection / telemetry (and the
  /// offline gate reads it to verify foot tracking). Valid after the first compute().
  const Eigen::VectorXd & target_configuration() const { return q_ik_; }

  void on_activate(const State & state, const ControlProblem & problem) override;
  // Carries the warm-start IK seed q_ik_ across ticks, so shadow-evaluation would corrupt it.
  bool stateless() const override { return false; }

private:
  // Design inputs.
  std::vector<std::string> feet_;
  std::vector<std::string> actuated_;
  Gains g_;

  // Resolved at configure().
  const RobotModel * model_ = nullptr;
  int nq_ = 0, nv_ = 0, m_ = 0, nc_ = 0, jpl_ = 0;   // jpl_ = joints per leg (= m_/nc_, must be 3)
  std::vector<std::size_t> feet_ids_;                // cached contact-frame indices
  std::vector<int> act_q_;                           // config index of each actuated joint (m_)
  std::vector<int> act_v_;                           // velocity index of each actuated joint (m_)

  // Preallocated so compute() stays allocation-free.
  std::unique_ptr<RobotModel::Workspace> ws_;
  Eigen::VectorXd q_ik_;                 // full-nq IK iterate (base from plan; joints warm-started)
  Eigen::MatrixXd J1_;                   // 3×nv single-foot Jacobian buffer
  std::vector<std::size_t> one_fid_;     // reused 1-element frame-id vector (alloc-free query)
  GaitPlan plan_;                        // sampled gait plan
  Command command_;
  Status status_;
};

}  // namespace kontrolem_controllers

#endif  // KONTROLEM_CONTROLLERS__KINEMATIC_GAIT_CONTROLLER_HPP_
