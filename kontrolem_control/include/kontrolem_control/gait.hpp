// Kontrol'Em v2 — the gait/locomotion reference SEAM (M10).
//
// A GaitSource answers "at time t, which feet are on the ground, where should the
// swinging foot be, and where should the body be?" — the plan a whole-body controller
// executes to WALK. It is the locomotion analogue of TrajectorySource: sampled every tick,
// it fills a caller-preallocated GaitPlan and must be allocation-free. Unlike a posture
// reference it also carries the per-foot CONTACT schedule (stance vs swing) — the extra
// information the WBC needs to decide which feet push and which foot tracks a swing arc.
//
// ROS-free, Eigen-only. This header holds ONLY the morphology-agnostic seam (GaitPlan +
// GaitSource). Concrete, morphology-specific patterns (CrawlGait, TrotGait, …) live in the
// kontrolem_locomotion package — the impl side of this seam — so robot-specific gaits do not
// accrete in the core contract.
#ifndef KONTROLEM_CONTROL__GAIT_HPP_
#define KONTROLEM_CONTROL__GAIT_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Dense>

namespace kontrolem_control
{

/// Per-tick locomotion plan the WBC executes. `stance`/`swing_pos`/`swing_vel` are indexed
/// by foot in the controller's contact-frame order.
struct GaitPlan
{
  std::vector<uint8_t> stance;               ///< per foot: 1 = stance (push), 0 = swing
  std::vector<Eigen::Vector3d> swing_pos;    ///< per foot: world target position (swing feet)
  std::vector<Eigen::Vector3d> swing_vel;    ///< per foot: world target velocity (swing feet)
  Eigen::VectorXd q_ref;                     ///< nq: base-pose (CoM-shifted) + nominal joints
  Eigen::VectorXd v_ref;                     ///< nv: base + joint reference velocity

  /// Size the plan for `nfeet` feet and an (nq, nv) model. Off the RT path (allocates).
  void resize(std::size_t nfeet, int nq, int nv)
  {
    stance.assign(nfeet, 1);
    swing_pos.assign(nfeet, Eigen::Vector3d::Zero());
    swing_vel.assign(nfeet, Eigen::Vector3d::Zero());
    q_ref = Eigen::VectorXd::Zero(nq);
    v_ref = Eigen::VectorXd::Zero(nv);
  }
};

/// Source of a time-varying gait plan. sample() writes the plan at time t into the
/// caller's pre-sized buffer and must not allocate. Concrete sources live in
/// kontrolem_locomotion (e.g. CrawlGait, TrotGait).
struct GaitSource
{
  virtual ~GaitSource() = default;
  virtual void sample(double t, GaitPlan & out) const = 0;
};

}  // namespace kontrolem_control

#endif  // KONTROLEM_CONTROL__GAIT_HPP_
