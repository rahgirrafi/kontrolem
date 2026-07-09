// Kontrol'Em v2 — the time-varying reference seam for the Tracking dialect.
//
// A TrajectorySource answers "what should the state be at time t?". It unifies a
// constant setpoint (a degenerate trajectory), an analytic reference, and — later
// — a played-back RobotTrajectory or a live reference topic, behind one abstract
// sample(). sample() is on the real-time path (a tracking controller calls it
// every tick), so it fills CALLER-PREALLOCATED buffers and must be
// allocation-free. New reference kinds are new derived types; no central edit.
#ifndef KONTROLEM_CONTROL__TRAJECTORY_HPP_
#define KONTROLEM_CONTROL__TRAJECTORY_HPP_

#include <cmath>

#include <Eigen/Dense>

namespace kontrolem_control
{

/// Source of a time-varying reference. sample() writes the desired configuration,
/// velocity, and (optional) feedforward torque at time t into the caller's
/// buffers, whose sizes the caller sets once (nq / nv / nv or 0). Must not
/// allocate: assignment into an equally-sized Eigen vector reuses its storage.
struct TrajectorySource
{
  virtual ~TrajectorySource() = default;
  /// Fill the desired configuration, velocity, ACCELERATION, and (optional)
  /// feedforward torque at time t. The acceleration lets a controller add a
  /// model-based feedforward so it does not merely chase the reference.
  virtual void sample(
    double t, Eigen::VectorXd & q_out, Eigen::VectorXd & v_out, Eigen::VectorXd & a_out,
    Eigen::VectorXd & tau_ff_out) const = 0;
};

/// Degenerate trajectory: a constant setpoint. This is exactly a Regulation
/// target expressed as a (trivial) Tracking reference — the sense in which
/// Regulation ⊂ Tracking.
struct ConstantReference : TrajectorySource
{
  Eigen::VectorXd q;       ///< nq
  Eigen::VectorXd v;       ///< nv (usually zero)
  Eigen::VectorXd tau_ff;  ///< nv or empty

  void sample(
    double /*t*/, Eigen::VectorXd & q_out, Eigen::VectorXd & v_out, Eigen::VectorXd & a_out,
    Eigen::VectorXd & tau_ff_out) const override
  {
    q_out = q;
    v_out = v;
    a_out.setZero();  // constant setpoint: zero reference acceleration
    if (tau_ff.size() == tau_ff_out.size()) {
      tau_ff_out = tau_ff;
    } else {
      tau_ff_out.setZero();
    }
  }
};

/// Per-coordinate harmonic reference: q_i(t) = center_i + amp_i cos(w t + phase_i),
/// with the consistent velocity v_i(t) = -amp_i w sin(w t + phase_i). A small,
/// exact, allocation-free analytic source for exercising / demonstrating tracking.
/// Leave amp_i = 0 on a coordinate to hold it fixed (e.g. keep a pole upright).
struct HarmonicReference : TrajectorySource
{
  Eigen::VectorXd center;  ///< nq
  Eigen::VectorXd amp;     ///< nq
  Eigen::VectorXd phase;   ///< nq
  double omega{1.0};

  void sample(
    double t, Eigen::VectorXd & q_out, Eigen::VectorXd & v_out, Eigen::VectorXd & a_out,
    Eigen::VectorXd & tau_ff_out) const override
  {
    const Eigen::Index n = center.size();
    for (Eigen::Index i = 0; i < n; ++i) {
      const double a = t * omega + phase(i);
      q_out(i) = center(i) + amp(i) * std::cos(a);
      v_out(i) = -amp(i) * omega * std::sin(a);
      a_out(i) = -amp(i) * omega * omega * std::cos(a);
    }
    tau_ff_out.setZero();  // torque feedforward not modeled; controller derives it
  }
};

}  // namespace kontrolem_control

#endif  // KONTROLEM_CONTROL__TRAJECTORY_HPP_
