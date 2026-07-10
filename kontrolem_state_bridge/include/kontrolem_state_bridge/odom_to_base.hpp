// The pure, ROS-free core of the base-state bridge: turn an odometry sample
// (base pose + twist, as plain scalars) into the 13-scalar floating-base layout
// that kontrolem_ros2_control's BaseStateSensor reassembles into a manifold
// State. Kept free of ROS/Eigen so the convention — which is the whole M6.3 risk
// (quaternion order, pose vs. twist frame) — is unit-testable off-robot.
//
// Layout (matches BaseStateSensor exactly): base[0..2] position, base[3..6]
// quaternion (x,y,z,w), base[7..9] linear velocity, base[10..12] angular
// velocity.
//
// FRAME CONVENTION (the crux, verified end-to-end by the Gazebo round-trip):
//   * Pose is expressed in the WORLD (odom) frame — it is the SE(3) placement of
//     the base in the world, i.e. Pinocchio's free-flyer configuration q(0..6).
//   * Twist is expressed in the BODY (child_frame) frame — matching both REP-145
//     (nav_msgs/Odometry twist is in child_frame_id) AND Pinocchio's free-flyer
//     velocity, which is the spatial velocity in the joint's LOCAL frame. So a
//     REP-145 odometry twist maps straight onto v(0..5) with no frame rotation.
//     A producer that reports world-frame twist would need rotating first; the
//     Gazebo odometry-publisher reports body-frame twist, so none is needed.
#ifndef KONTROLEM_STATE_BRIDGE__ODOM_TO_BASE_HPP_
#define KONTROLEM_STATE_BRIDGE__ODOM_TO_BASE_HPP_

#include <array>
#include <cmath>

namespace kontrolem_state_bridge
{

/// Fill `base` (13 scalars) from an odometry sample. The quaternion is
/// normalized to the unit-quaternion manifold (the transport does not guarantee
/// it, and an un-normalized quat silently corrupts every downstream SE(3) op).
/// A zero/degenerate quaternion falls back to identity (0,0,0,1).
inline void base_from_odom(
  double px, double py, double pz,
  double qx, double qy, double qz, double qw,
  double vx, double vy, double vz,
  double wx, double wy, double wz,
  std::array<double, 13> & base)
{
  double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (!(n > 1e-9)) {
    qx = 0.0; qy = 0.0; qz = 0.0; qw = 1.0; n = 1.0;
  }
  base[0] = px; base[1] = py; base[2] = pz;
  base[3] = qx / n; base[4] = qy / n; base[5] = qz / n; base[6] = qw / n;
  base[7] = vx; base[8] = vy; base[9] = vz;
  base[10] = wx; base[11] = wy; base[12] = wz;
}

}  // namespace kontrolem_state_bridge

#endif  // KONTROLEM_STATE_BRIDGE__ODOM_TO_BASE_HPP_
