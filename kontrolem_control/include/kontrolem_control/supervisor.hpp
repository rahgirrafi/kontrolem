// Kontrol'Em v2 — the multi-controller Supervisor (the meta-controller).
//
// Hosts several controllers, drives exactly ONE at a time, and makes a MANUAL
// (commanded) switch bumpless. The switching spike (test_switching_spike) proved
// what a safe handoff needs, and this implements both halves:
//   (finding 2) seed the incoming controller from the current state on activation
//               (Controller::on_activate) so a state-carrying law resumes
//               already-converged instead of fighting a stale estimate;
//   (finding 3) blend the command from outgoing to incoming over a short window
//               so even a STATELESS incoming law (nothing to seed) never presents
//               the actuator a step — the residual inter-controller disagreement
//               is spread into a bounded slew.
//
// ROS-free: the ros2_control binding feeds it a State each tick and exposes
// request_switch() over a service. Switching policy here is MANUAL only;
// automatic (status()-driven, dwell/hysteresis) is a later layer on top.
#ifndef KONTROLEM_CONTROL__SUPERVISOR_HPP_
#define KONTROLEM_CONTROL__SUPERVISOR_HPP_

#include <algorithm>
#include <string>
#include <vector>

#include "kontrolem_control/controller.hpp"

namespace kontrolem_control
{

class Supervisor
{
public:
  /// blend_ticks: length of the outgoing->incoming command blend at a switch
  /// (e.g. 20 ticks = 40 ms at 500 Hz). 1 = hard switch (blend disabled).
  explicit Supervisor(int blend_ticks = 20)
  : blend_ticks_(std::max(1, blend_ticks)), blend_(blend_ticks_) {}  // start idle

  /// Register a controller by name. Non-owning: the runtime owns the object and
  /// must keep it alive for the Supervisor's lifetime. Call before compute().
  void add(const std::string & name, Controller * controller)
  {
    entries_.push_back(Entry{name, controller});
  }

  /// Set the initially-active controller by name (no-op if unknown).
  void set_active(const std::string & name)
  {
    Controller * c = find(name);
    if (!c) return;
    active_ = c;
    active_name_ = name;
    outgoing_ = nullptr;
    blend_ = blend_ticks_;  // idle
  }

  /// MANUAL switch request. Returns false (and does nothing) if the name is
  /// unknown, already active, or a switch is already blending. Takes effect on the
  /// next compute().
  bool request_switch(const std::string & name)
  {
    if (switching()) return false;          // one switch at a time
    if (name == active_name_) return false;
    if (!find(name)) return false;
    pending_ = name;
    pending_hard_ = false;                   // manual switch: blend (both laws healthy)
    return true;
  }

  /// Enable AUTOMATIC fail-forward: when the active law reports not-ok for
  /// `dwell_ticks` consecutive settled ticks, the Supervisor hands off (bumplessly)
  /// to the NEXT law in the hosted order — the primary, then its fallbacks. The
  /// dwell debounces a single transient not-ok tick so a momentary blip can't cause
  /// chatter. There is no automatic switch-BACK (that needs shadow-evaluating a
  /// dormant law's health and is chatter-prone); recover to the primary with a
  /// manual request_switch(). Manual switching still works with auto enabled.
  void set_auto_fallback(bool enabled, int dwell_ticks = 5)
  {
    auto_fallback_ = enabled;
    fail_dwell_ = std::max(1, dwell_ticks);
    fail_count_ = 0;
  }

  /// Enable AUTOMATIC recovery to the PRIMARY (the first hosted law): while a
  /// fallback is driving, the Supervisor SHADOW-evaluates the primary's health at
  /// the current state each tick (runs its compute() only to read status() — safe
  /// because it is stateless; see Controller::stateless) and, once the primary has
  /// been trustworthy for `dwell_ticks` CONSECUTIVE ticks, hands back to it
  /// (bumplessly, WITH a command blend — the incoming primary is healthy). This is
  /// the switch-BACK half of dwell-time/hysteresis switching (Hespanha–Morse): the
  /// recovery dwell is deliberately LONG (default 50 ≈ 0.25 s at 200 Hz) so a still-
  /// settling disturbance can't trigger a premature return, and combined with the
  /// fail-forward dwell it bounds chatter. Recovery is skipped (the fallback keeps
  /// driving until a manual switch) when the primary is NOT stateless, since a
  /// stateful primary can't be shadow-run without corrupting its estimate.
  void set_auto_recover(bool enabled, int dwell_ticks = 50)
  {
    auto_recover_ = enabled;
    recover_dwell_ = std::max(1, dwell_ticks);
    recover_count_ = 0;
  }

  /// Per-tick: apply any pending switch (seed incoming + start the blend), run the
  /// active controller (and, during a blend, the outgoing too), and return the
  /// conditioned command. Steady state is a single compute() + a copy.
  const Command & compute(const State & state, const ControlProblem & problem, double dt)
  {
    if (!pending_.empty()) {
      outgoing_ = active_;
      active_ = find(pending_);
      active_name_ = pending_;
      active_->on_activate(state, problem);  // bumpless seed of the incoming law
      // A manual switch blends (both laws are healthy, so smooth the handoff). An
      // automatic fail-forward switches HARD: the outgoing law has LOST TRUST, so
      // blending its (untrustworthy, possibly unbounded) command back in is exactly
      // wrong — hand fully to the incoming law, which reads the current state.
      blend_ = pending_hard_ ? blend_ticks_ : 0;
      if (pending_hard_) outgoing_ = nullptr;
      pending_.clear();
    }

    const Command & u_in = active_->compute(state, problem, dt);
    if (out_.tau.size() != u_in.tau.size()) out_.tau.resize(u_in.tau.size());  // one-time

    if (blend_ < blend_ticks_ && outgoing_ != nullptr) {
      const Command & u_out = outgoing_->compute(state, problem, dt);  // keep outgoing live
      const double a = static_cast<double>(blend_ + 1) / static_cast<double>(blend_ticks_);
      out_.tau = (1.0 - a) * u_out.tau + a * u_in.tau;  // lerp into existing storage
      ++blend_;
      if (blend_ >= blend_ticks_) outgoing_ = nullptr;  // blend complete
    } else {
      out_.tau = u_in.tau;
    }

    status_ = active_->status();

    // Automatic fail-forward: only evaluated on settled ticks (no blend in
    // progress, no manual switch already pending). Counts consecutive not-ok ticks
    // of the active law; on reaching the dwell, queue a switch to the next law in
    // the hosted order (if any remains — the last fallback has nowhere to go, so
    // the runtime's safe action takes over instead).
    if (auto_fallback_ && pending_.empty() && !switching()) {
      if (!status_.ok) {
        if (++fail_count_ >= fail_dwell_) {
          const int idx = active_index();
          if (idx >= 0 && idx + 1 < static_cast<int>(entries_.size())) {
            pending_ = entries_[static_cast<std::size_t>(idx + 1)].name;
            pending_hard_ = true;  // fail-forward: hard handoff, don't blend the failed law
          }
          fail_count_ = 0;
        }
      } else {
        fail_count_ = 0;
      }
    }

    // Automatic recovery to the PRIMARY: only when settled and we're currently on
    // a fallback (not already the primary). Shadow-evaluate the primary at the
    // current state — run its compute() purely to read status(), discarding the
    // command (safe only for a stateless primary). Once the primary has been
    // trustworthy for recover_dwell_ consecutive ticks (a long hysteresis dwell),
    // hand back to it with a blend (it is healthy, so a smooth handoff is right).
    if (auto_recover_ && pending_.empty() && !switching() && !entries_.empty() &&
        active_index() != 0 && entries_[0].ctrl->stateless())
    {
      Controller * primary = entries_[0].ctrl;
      primary->compute(state, problem, dt);  // shadow: command discarded
      if (primary->status().ok) {
        if (++recover_count_ >= recover_dwell_) {
          pending_ = entries_[0].name;
          pending_hard_ = false;  // recovery blends: handing to a trusted, healthy law
          recover_count_ = 0;
        }
      } else {
        recover_count_ = 0;
      }
    }
    return out_;
  }

  const Status & status() const { return status_; }
  const std::string & active_name() const { return active_name_; }
  bool switching() const { return blend_ < blend_ticks_; }

private:
  struct Entry { std::string name; Controller * ctrl; };

  Controller * find(const std::string & name) const
  {
    for (const auto & e : entries_) {
      if (e.name == name) return e.ctrl;
    }
    return nullptr;
  }

  int active_index() const
  {
    for (std::size_t i = 0; i < entries_.size(); ++i) {
      if (entries_[i].name == active_name_) return static_cast<int>(i);
    }
    return -1;
  }

  std::vector<Entry> entries_;
  Controller * active_ = nullptr;
  Controller * outgoing_ = nullptr;
  std::string active_name_;
  std::string pending_;       // requested switch target ("" = none)
  bool pending_hard_ = false;  // pending switch is a hard handoff (auto fail-forward)

  int blend_ticks_;
  int blend_;  // ticks into the current blend; >= blend_ticks_ means idle
  Command out_;
  Status status_;

  bool auto_fallback_ = false;
  int fail_dwell_ = 5;   // consecutive not-ok ticks before an automatic fail-forward
  int fail_count_ = 0;

  bool auto_recover_ = false;
  int recover_dwell_ = 50;   // consecutive healthy-primary ticks before recovering to it
  int recover_count_ = 0;
};

}  // namespace kontrolem_control

#endif  // KONTROLEM_CONTROL__SUPERVISOR_HPP_
