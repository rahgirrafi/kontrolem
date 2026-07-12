// Kontrol'Em v2 — floating-base reference sources for the Tracking dialect (M9).
//
// The existing HarmonicReference oscillates each coordinate of q INDEPENDENTLY, which
// corrupts a floating base's unit quaternion (q[3:7]). These sources are floating-base
// AWARE: they take a nominal configuration and apply a time-varying SE(3) offset to the
// BASE (translation + a composed rotation), holding the joints at nominal — so a WBC
// standing on planted feet can be commanded to squat / sway / tilt / yaw its whole body
// within its 6 residual base DoF. Pure Eigen (no RobotModel), fixed-size quaternion math,
// allocation-free sample() — the ROS-free core the WBC and its offline test both use.
//
// Base-offset layout (6): [x, y, z] world translation, [roll, pitch, yaw] rotation
// (composed onto the nominal orientation). Amplitudes must stay within the leg workspace
// (feet cannot leave the ground here — that is locomotion, a later milestone).
#ifndef KONTROLEM_CONTROL__BASE_REFERENCE_HPP_
#define KONTROLEM_CONTROL__BASE_REFERENCE_HPP_

#include <array>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "kontrolem_control/trajectory.hpp"

namespace kontrolem_control
{

/// Apply a 6-vector base offset (world translation dxyz + composed rotation drpy) to a
/// nominal configuration, writing the result into q_out (nq). Manifold-correct: the
/// rotation is composed onto the nominal quaternion (not added coordinate-wise) and kept
/// unit. Fixed-size quaternion ops — allocation-free given a pre-sized q_out.
inline void apply_base_offset(
  const Eigen::VectorXd & q_nominal, const std::array<double, 6> & off, Eigen::VectorXd & q_out)
{
  q_out = q_nominal;                                   // joints (7..) held at nominal
  q_out(0) = q_nominal(0) + off[0];
  q_out(1) = q_nominal(1) + off[1];
  q_out(2) = q_nominal(2) + off[2];
  // Compose the offset rotation onto the nominal orientation (xyzw storage in q).
  const Eigen::Quaterniond qn(q_nominal(6), q_nominal(3), q_nominal(4), q_nominal(5));
  const Eigen::Quaterniond dq = Eigen::AngleAxisd(off[3], Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(off[4], Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(off[5], Eigen::Vector3d::UnitZ());
  const Eigen::Quaterniond qr = (dq * qn).normalized();
  q_out(3) = qr.x(); q_out(4) = qr.y(); q_out(5) = qr.z(); q_out(6) = qr.w();
}

/// Canned reference: a per-base-axis harmonic offset about a nominal posture.
/// off_a(t) = amp_a cos(omega_a t + phase_a) for a in {x,y,z,roll,pitch,yaw}. Leave
/// amp_a = 0 to hold an axis fixed. v_out/a_out carry the (world-frame) base rates —
/// the WBC's baseline PD ignores them (moving-setpoint tracking); they are ready for an
/// optional velocity/acceleration feedforward.
struct BasePoseReference : TrajectorySource
{
  Eigen::VectorXd q_nominal;              ///< nq: base pose + the held joint posture
  std::array<double, 6> amp{{0, 0, 0, 0, 0, 0}};
  std::array<double, 6> omega{{0, 0, 0, 0, 0, 0}};
  std::array<double, 6> phase{{0, 0, 0, 0, 0, 0}};

  void sample(
    double t, Eigen::VectorXd & q_out, Eigen::VectorXd & v_out, Eigen::VectorXd & a_out,
    Eigen::VectorXd & tau_ff_out) const override
  {
    std::array<double, 6> off{}, doff{}, ddoff{};
    for (int a = 0; a < 6; ++a) {
      const double th = omega[a] * t + phase[a];
      off[a] = amp[a] * std::cos(th);
      doff[a] = -amp[a] * omega[a] * std::sin(th);
      ddoff[a] = -amp[a] * omega[a] * omega[a] * std::cos(th);
    }
    apply_base_offset(q_nominal, off, q_out);
    v_out.setZero();
    a_out.setZero();
    for (int a = 0; a < 6; ++a) { v_out(a) = doff[a]; a_out(a) = ddoff[a]; }
    tau_ff_out.setZero();
  }
};

/// Live reference: a constant base offset held until externally updated (the runtime
/// writes `offset` from a ~/base_target topic each tick). sample() ignores t and returns
/// the current offset applied to the nominal posture; v_out/a_out are zero, so the WBC
/// eases into the commanded pose via its position PD.
struct LiveBaseTarget : TrajectorySource
{
  Eigen::VectorXd q_nominal;              ///< nq
  std::array<double, 6> offset{{0, 0, 0, 0, 0, 0}};

  void sample(
    double /*t*/, Eigen::VectorXd & q_out, Eigen::VectorXd & v_out, Eigen::VectorXd & a_out,
    Eigen::VectorXd & tau_ff_out) const override
  {
    apply_base_offset(q_nominal, offset, q_out);
    v_out.setZero();
    a_out.setZero();
    tau_ff_out.setZero();
  }
};

}  // namespace kontrolem_control

#endif  // KONTROLEM_CONTROL__BASE_REFERENCE_HPP_
