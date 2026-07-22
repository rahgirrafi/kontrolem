// Kontrol'Em v2 — TrotGait: a dynamic diagonal-trot quadruped pattern (M13/M14).
//
// Concrete implementation of the kontrolem_control GaitSource seam, living in
// kontrolem_locomotion (not the core contract) because it is MORPHOLOGY-SPECIFIC
// (four feet, diagonal pairs). Consumed unchanged by the kinematic-gait controller
// (M13, base pinned → body-frame IK) and the QP-WBC (M14, base task + stance/swing).
#ifndef KONTROLEM_LOCOMOTION__TROT_GAIT_HPP_
#define KONTROLEM_LOCOMOTION__TROT_GAIT_HPP_

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

/// Dynamic forward TROT gait for a quadruped (M13/M14 showcase). Two DIAGONAL pairs of feet
/// swing together, 180° out of phase — {FL,RR} then {FR,RL} — so at any instant the two
/// grounded feet lie on a diagonal LINE under the body that catches it. Unlike CrawlGait
/// there is NO support-centroid weight-shift: the base is simply held at its nominal pose and
/// advanced forward at the foot-ratchet rate. That deletion is deliberate — the crawl's lateral
/// pre-shift is the ~40% startup tip (M12); a trot needs no static support polygon, so it is
/// dropped. A brief all-stance (double-support) margin brackets each pair's swing (set by
/// `duty` < 1), which keeps the trot quasi-statically forgiving; `duty`→1 makes it livelier.
///
/// Same drop-in shape as CrawlGait: built from the nominal foot positions (FK once) at t=0,
/// stateless/analytic in t (world frame — a stance foot is fixed, a swing foot arcs to its next
/// plant), pure Eigen, allocation-free.
struct TrotGait : GaitSource
{
  Eigen::VectorXd q_nominal;                       ///< nq: base (origin) + nominal joints
  std::vector<Eigen::Vector3d> foot_nominal;       ///< 4 world foot positions at t=0
  std::array<int, 4> swing_pair{{0, 1, 1, 0}};     ///< per foot (in contact-frame order): diagonal
                                                   ///< pair, {FL,RR}=0 and {FR,RL}=1
  double period = 1.0;                             ///< full stride time — both pairs swing once (s)
  double duty = 0.5;                               ///< swing fraction of a pair's half-cycle (<1)
  double step_len = 0.06;                          ///< forward step per foot per stride (m)
  double step_h = 0.05;                            ///< swing-arc apex height (m)
  double start_delay = 1.0;                        ///< all-stance settle before the first step

  /// State of foot f at gait-clock tg: stance? world position, and (if swinging) velocity.
  void foot_state(double tg, std::size_t f, bool & stance, Eigen::Vector3d & pos,
                  Eigen::Vector3d & vel) const
  {
    const double half = period / 2.0;
    const double d = duty * half;                  // swing duration
    const double margin = 0.5 * (half - d);        // all-stance bracket on each side
    const double sw_start = swing_pair[f] * half + margin;
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
        // C1 swing: smoothstep forward (zero horizontal velocity at lift-off/touchdown — no
        // scuffing) + raised-cosine lift (zero vertical velocity at the ground). Both position
        // AND velocity are continuous across the stance↔swing boundary, so the joint targets a
        // PD tracks have no velocity step (a plain sine/linear arc jerks the leg at each footfall).
        const double sx = p * p * (3.0 - 2.0 * p);   // smoothstep 3p²−2p³
        const double dsx = 6.0 * p * (1.0 - p);      // d(sx)/dp, = 0 at p=0 and p=1
        pos.x() += step_len * sx;
        pos.z() += step_h * 0.5 * (1.0 - std::cos(2.0 * M_PI * p));
        vel = Eigen::Vector3d(
          step_len * dsx / d, 0.0, step_h * M_PI * std::sin(2.0 * M_PI * p) / d);
      }
    }
  }

  void sample(double t, GaitPlan & out) const override
  {
    const std::size_t nf = foot_nominal.size();
    out.q_ref = q_nominal;
    out.v_ref.setZero();
    const double tg = t - start_delay;
    for (std::size_t f = 0; f < nf; ++f) {
      bool st; Eigen::Vector3d p, vv;
      foot_state(tg, f, st, p, vv);
      out.stance[f] = st ? 1 : 0;
      out.swing_pos[f] = p;
      out.swing_vel[f] = vv;
    }
    // Pre-start (before start_delay): hold nominal, no base motion — identical to standing, so
    // the pre-release phase does not wind the controller up against a hold (as CrawlGait notes).
    if (tg < 0.0) return;
    // Advance the base forward at the foot-ratchet rate (step_len per stride) so the body tracks
    // the feet marching out from under it. NO lateral / centroid shift — the trot's whole point.
    // Emit the matching forward VELOCITY feedforward too: the base glides at a constant fwd speed,
    // so v_ref(0)=fwd cancels the WBC's -Kd(v - v_ref) damping that would otherwise fight the ramp
    // (a pure position ref lags badly under feedback only). The kinematic controller (M13) pins the
    // base to q_ref and ignores v_ref, so this is a no-op there — it only helps the WBC (M14).
    const double fwd = (period > 0.0) ? step_len / period : 0.0;
    out.q_ref(0) = q_nominal(0) + fwd * tg;
    out.v_ref(0) = fwd;
  }
};

}  // namespace kontrolem_locomotion

#endif  // KONTROLEM_LOCOMOTION__TROT_GAIT_HPP_
