// Kontrol'Em v2 — CrawlGait: a statically-stable quadruped crawl pattern (M10).
//
// Concrete implementation of the kontrolem_control GaitSource seam. It lives in
// kontrolem_locomotion (not the core contract) because it is MORPHOLOGY-SPECIFIC —
// it assumes four feet and a support-triangle — so robot-specific gait patterns do
// not accrete in the contract package (the locomotion analogue of a concrete
// controller living in kontrolem_controllers, not kontrolem_control).
#ifndef KONTROLEM_LOCOMOTION__CRAWL_GAIT_HPP_
#define KONTROLEM_LOCOMOTION__CRAWL_GAIT_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include <Eigen/Dense>

#include "kontrolem_control/gait.hpp"

namespace kontrolem_locomotion
{

using kontrolem_control::GaitPlan;
using kontrolem_control::GaitSource;

/// Statically-stable forward CRAWL gait for a quadruped. Exactly one foot swings at a
/// time (in `order`), each swing lifting the foot along an arc and advancing it
/// `step_len` in +x; feet ratchet forward so the whole robot walks. The base xy is
/// commanded to the CENTROID of the current support feet — inside the support polygon by
/// construction (static stability) AND advancing forward as the feet step (progression).
/// Stateless: foot positions are analytic in t (world frame; a stance foot is fixed, a
/// swing foot interpolates to its next plant). Pure Eigen — no model, allocation-free.
///
/// Built from the nominal foot positions (FK once) at t=0. `q_nominal` holds the base
/// pose + the held joint posture; `foot_nominal` the four world foot positions in the
/// controller's contact-frame order.
struct CrawlGait : GaitSource
{
  Eigen::VectorXd q_nominal;                       ///< nq: base (origin) + nominal joints
  std::vector<Eigen::Vector3d> foot_nominal;       ///< 4 world foot positions at t=0
  std::array<int, 4> order{{2, 0, 3, 1}};          ///< swing order (RL, FL, RR, FR)
  double period = 8.0;                             ///< full 4-foot cycle time (s)
  double duty = 0.6;                               ///< swing fraction of a foot's quarter-cycle
  double step_len = 0.06;                          ///< forward step per foot per cycle (m)
  double step_h = 0.04;                            ///< swing-arc apex height (m)
  double base_gain = 1.0;                          ///< base xy toward support centroid (0..1)
  double start_delay = 1.0;                        ///< all-stance settle before the first step
  // Base-target offset (the CoM is not at the geometric center) — nudge stability. Two
  // plain doubles, NOT an Eigen::Vector2d: a 16-byte vectorizable fixed-size Eigen member
  // in a heap-allocated struct (the runtime make_unique's this) can be misaligned and
  // corrupt the heap on SSE access.
  double com_bias_x = 0.0;
  double com_bias_y = 0.0;

  int slot_of(std::size_t f) const
  {
    for (int s = 0; s < 4; ++s) {
      if (order[static_cast<std::size_t>(s)] == static_cast<int>(f)) return s;
    }
    return 0;
  }

  /// State of foot f at gait-clock tg: stance? world position, and (if swinging) velocity.
  void foot_state(double tg, std::size_t f, bool & stance, Eigen::Vector3d & pos,
                  Eigen::Vector3d & vel) const
  {
    const double q4 = period / 4.0;
    const double d = duty * q4;
    const double margin = 0.5 * (q4 - d);
    const double sw_start = slot_of(f) * q4 + margin;
    int completed = 0;
    if (tg >= sw_start + d) completed = static_cast<int>(std::floor((tg - sw_start - d) / period)) + 1;
    if (completed < 0) completed = 0;
    pos = foot_nominal[f];
    pos.x() += completed * step_len;
    vel.setZero();
    stance = true;
    if (tg >= sw_start) {
      const double c = std::floor((tg - sw_start) / period);
      const double win = sw_start + c * period;
      if (c >= 0 && tg >= win && tg < win + d) {
        stance = false;
        const double p = (tg - win) / d;
        pos.x() += step_len * p;
        pos.z() += step_h * std::sin(M_PI * p);
        vel = Eigen::Vector3d(step_len / d, 0.0, step_h * M_PI * std::cos(M_PI * p) / d);
      }
    }
  }

  /// Time until foot f's next swing window starts, at gait-clock tg (>= 0 if pre-start).
  double time_to_next_swing(double tg, std::size_t f) const
  {
    const double q4 = period / 4.0;
    const double margin = 0.5 * (q4 - duty * q4);
    const double sw_start = slot_of(f) * q4 + margin;
    if (tg < sw_start) return sw_start - tg;
    const double c = std::ceil((tg - sw_start) / period);
    return sw_start + c * period - tg;
  }

  void sample(double t, GaitPlan & out) const override
  {
    const std::size_t nf = foot_nominal.size();
    out.q_ref = q_nominal;
    out.v_ref.setZero();
    const double tg = t - start_delay;
    int active = -1;   // currently swinging foot (crawl: at most one)
    for (std::size_t f = 0; f < nf; ++f) {
      bool st; Eigen::Vector3d p, vv;
      foot_state(tg, f, st, p, vv);
      out.stance[f] = st ? 1 : 0;
      out.swing_pos[f] = p;
      out.swing_vel[f] = vv;
      if (!st) active = static_cast<int>(f);
    }
    // Pre-start (before start_delay): hold the nominal standing posture with NO base shift.
    // The gait clock begins at controller activation but the robot is held (welded in the
    // Gazebo demo) and only released later, so shifting the base / swinging a foot before
    // then winds the WBC up against the hold and jolts violently at release. Staying at
    // nominal makes the pre-release phase identical to standing.
    if (tg < 0.0) return;
    // Base xy tracks the centroid of the support triangle for the swing that is happening
    // NOW, or — during an all-stance margin — the one about to happen. Excluding the
    // active-OR-NEXT swing foot HOLDS the base at the current triangle through the swing
    // (no over-lead) while PRE-SHIFTING to the next triangle during the margin. This is the
    // static walk's crux: the CoM is over the support polygon before each foot lifts.
    int target = active;
    if (target < 0) {
      double best = 1e18;
      for (std::size_t f = 0; f < nf; ++f) {
        const double dt_next = time_to_next_swing(tg, f);
        if (dt_next < best) { best = dt_next; target = static_cast<int>(f); }
      }
    }
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    int n = 0;
    for (std::size_t f = 0; f < nf; ++f) {
      if (static_cast<int>(f) == target) continue;
      sum += out.swing_pos[f];  // current (stance) position
      ++n;
    }
    if (n > 0) {
      const Eigen::Vector3d c = sum / static_cast<double>(n);
      out.q_ref(0) = q_nominal(0) + base_gain * (c.x() - q_nominal(0)) + com_bias_x;
      out.q_ref(1) = q_nominal(1) + base_gain * (c.y() - q_nominal(1)) + com_bias_y;
    }
  }
};

}  // namespace kontrolem_locomotion

#endif  // KONTROLEM_LOCOMOTION__CRAWL_GAIT_HPP_
