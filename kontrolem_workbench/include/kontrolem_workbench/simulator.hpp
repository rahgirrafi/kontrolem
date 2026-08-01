// The workbench's closed-loop test-signal simulator (the shared empirical
// tier): run ANY law — via the same contract the runtime uses — against the
// true nonlinear dynamics under a chosen excitation, and record the full
// trace. ROS-free; fixed-base robots (v1). The integration scheme matches the
// project's offline references (semi-implicit Euler on RobotModel::aba), so a
// simulated law behaves like the shipped custom-sim demos.
#ifndef KONTROLEM_WORKBENCH__SIMULATOR_HPP_
#define KONTROLEM_WORKBENCH__SIMULATOR_HPP_

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "kontrolem_control/controller.hpp"
#include "kontrolem_model/robot_model.hpp"

namespace kontrolem_workbench
{

/// One closed-loop experiment.
struct SimScenario
{
  enum class Signal
  {
    kRelease,  ///< start displaced (q0), regulate to the setpoint — the step-response classic
    kStep,     ///< start AT the setpoint; at t_event the setpoint jumps to q_target
    kPush,     ///< at t_event, a disturbance torque hits the plant for push_duration
    kSine,     ///< track a harmonic reference (needs a law that accepts Tracking)
  };
  Signal signal{Signal::kRelease};

  Eigen::VectorXd q_ref, v_ref;  ///< the setpoint (and sine center)
  Eigen::VectorXd q0, v0;        ///< initial state (empty -> q_ref / zero)
  Eigen::VectorXd q_target;      ///< kStep: setpoint after t_event
  Eigen::VectorXd push_tau;      ///< kPush: full-nv disturbance during the push window
  double push_duration{0.1};     ///< s
  Eigen::VectorXd sine_amp;      ///< kSine: per-coordinate amplitude
  double sine_omega{1.0};        ///< rad/s

  double dt{0.002};      ///< control + integration step
  double duration{6.0};  ///< s
  double t_event{1.0};   ///< when kStep / kPush fires
};

/// The recorded run (one row per tick).
struct SimTrace
{
  std::vector<double> t;
  std::vector<Eigen::VectorXd> q, v, tau;  ///< tau = full-nv applied actuation
  std::vector<Eigen::VectorXd> q_des;      ///< the reference at that tick
  std::vector<bool> ok;
  std::vector<double> margin;
  std::vector<int> iters;
  double t_event{0.0};       ///< when the excitation ended/fired (metrics measure from here)
  bool finite{true};         ///< false if the state diverged (run stops early)
  SimScenario::Signal signal{SimScenario::Signal::kRelease};  ///< what excited this run
};

/// Run `law` closed-loop on `model` (synthesize + configure + per-tick compute,
/// exactly the runtime lifecycle). `actuated_joints` maps the command onto the
/// plant. Throws std::runtime_error if the scenario needs a dialect the law
/// does not accept (e.g. kSine on a Regulation-only law).
SimTrace simulate(
  const kontrolem_model::RobotModel & model, kontrolem_control::Controller & law,
  const std::vector<std::string> & actuated_joints, const SimScenario & scenario);

/// Write the trace as CSV (t, q_*, v_*, tau_*, qdes_*, ok, margin, iters).
void write_csv(const SimTrace & trace, const std::string & path);

}  // namespace kontrolem_workbench

#endif  // KONTROLEM_WORKBENCH__SIMULATOR_HPP_
