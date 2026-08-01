// Time-domain performance metrics off a simulated trace — the classical
// vocabulary (settling, overshoot, steady-state error) measured empirically,
// plus the optimization-health gauges (saturation, solver ok, margin) that
// QP-based laws need. ROS-free.
#ifndef KONTROLEM_WORKBENCH__METRICS_HPP_
#define KONTROLEM_WORKBENCH__METRICS_HPP_

#include <string>

#include "kontrolem_workbench/simulator.hpp"

namespace kontrolem_workbench
{

struct Metrics
{
  bool finite{true};          ///< the run stayed finite (false = diverged: FAIL)
  double settling_time{-1.0}; ///< s after t_event until |q-q_des| stays inside the band (-1: never)
  double overshoot_frac{0.0}; ///< max excursion past the target / initial error (0 = none;
                              ///< computed for release/step/sine — undefined for a push)
  double peak_deviation{0.0}; ///< max |q - q_des| after the event (THE push metric)
  double ss_error{0.0};       ///< mean max-coordinate |q - q_des| over the final 10%
  double peak_tau{0.0};       ///< max |applied torque| over the run
  double saturation_frac{0.0};///< fraction of ticks with |tau| >= 98% of tau_limit (0 if no limit)
  double ok_frac{1.0};        ///< fraction of ticks with status().ok
  double min_margin{0.0};     ///< worst status().margin over the run
  double recovery_time{-1.0}; ///< alias of settling_time measured from the event (push runs)
  SimScenario::Signal signal{SimScenario::Signal::kRelease};  ///< carried from the trace
};

/// `band`: the settled/recovered threshold on max-coordinate |q - q_des|.
/// `tau_limit`: the law's torque limit for the saturation gauge (<= 0: skip).
Metrics compute_metrics(const SimTrace & trace, double band, double tau_limit = 0.0);

/// Human-readable report block.
std::string format_metrics(const Metrics & m, double band);

}  // namespace kontrolem_workbench

#endif  // KONTROLEM_WORKBENCH__METRICS_HPP_
