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
    return true;
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
      blend_ = 0;                            // begin the command blend
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

  std::vector<Entry> entries_;
  Controller * active_ = nullptr;
  Controller * outgoing_ = nullptr;
  std::string active_name_;
  std::string pending_;  // requested switch target ("" = none)

  int blend_ticks_;
  int blend_;  // ticks into the current blend; >= blend_ticks_ means idle
  Command out_;
  Status status_;
};

}  // namespace kontrolem_control

#endif  // KONTROLEM_CONTROL__SUPERVISOR_HPP_
