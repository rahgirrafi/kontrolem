#include "kontrolem_workbench/metrics.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace kontrolem_workbench
{

Metrics compute_metrics(const SimTrace & tr, double band, double tau_limit)
{
  Metrics m;
  m.finite = tr.finite;
  m.signal = tr.signal;
  if (tr.t.empty()) {
    m.finite = false;
    return m;
  }
  const std::size_t N = tr.t.size();

  // Settling: the LAST time the error leaves the band; settled ever after.
  double last_outside = -1.0;
  for (std::size_t k = 0; k < N; ++k) {
    const double e = (tr.q[k] - tr.q_des[k]).cwiseAbs().maxCoeff();
    if (e > band && tr.t[k] >= tr.t_event) last_outside = tr.t[k];
  }
  const double e_final = (tr.q.back() - tr.q_des.back()).cwiseAbs().maxCoeff();
  if (e_final <= band) {
    m.settling_time = last_outside < 0.0 ? 0.0 : last_outside - tr.t_event;
    m.recovery_time = m.settling_time;
  }

  std::size_t k0 = 0;  // first sample at/after the event = where the error starts
  while (k0 + 1 < N && tr.t[k0] < tr.t_event) ++k0;
  for (std::size_t k = k0; k < N; ++k) {
    m.peak_deviation =
      std::max(m.peak_deviation, (tr.q[k] - tr.q_des[k]).cwiseAbs().maxCoeff());
  }

  // Overshoot: per coordinate with a real initial error, the excursion PAST the
  // target (in the approach direction) as a fraction of that initial error. A
  // step-response notion — meaningless for a push (initial error ~ 0), where
  // peak_deviation is the metric instead.
  if (tr.signal != SimScenario::Signal::kPush) {
    const Eigen::VectorXd e0 = tr.q[k0] - tr.q_des[k0];
    for (Eigen::Index i = 0; i < e0.size(); ++i) {
      if (std::abs(e0(i)) < 1e-9) continue;
      double worst = 0.0;
      for (std::size_t k = k0; k < N; ++k) {
        // e starts at e0(i); crossing zero and continuing is overshoot.
        const double e = tr.q[k](i) - tr.q_des[k](i);
        worst = std::max(worst, e0(i) > 0.0 ? -e : e);
      }
      m.overshoot_frac = std::max(m.overshoot_frac, worst / std::abs(e0(i)));
    }
  }

  // Steady-state error: mean over the final 10% of the run.
  const std::size_t tail = std::max<std::size_t>(1, N / 10);
  double acc = 0.0;
  for (std::size_t k = N - tail; k < N; ++k) {
    acc += (tr.q[k] - tr.q_des[k]).cwiseAbs().maxCoeff();
  }
  m.ss_error = acc / static_cast<double>(tail);

  // Torque + health gauges.
  std::size_t sat = 0, ok = 0;
  m.min_margin = tr.margin.empty() ? 0.0 : tr.margin.front();
  for (std::size_t k = 0; k < N; ++k) {
    const double peak = tr.tau[k].cwiseAbs().maxCoeff();
    m.peak_tau = std::max(m.peak_tau, peak);
    if (tau_limit > 0.0 && peak >= 0.98 * tau_limit) ++sat;
    if (tr.ok[k]) ++ok;
    m.min_margin = std::min(m.min_margin, tr.margin[k]);
  }
  if (tau_limit > 0.0) m.saturation_frac = static_cast<double>(sat) / static_cast<double>(N);
  m.ok_frac = static_cast<double>(ok) / static_cast<double>(N);
  return m;
}

std::string format_metrics(const Metrics & m, double band)
{
  std::ostringstream os;
  os.precision(4);
  if (!m.finite) {
    os << "  DIVERGED — the closed loop is unstable for this scenario\n";
    return os.str();
  }
  os << "  settling time    ";
  if (m.settling_time < 0.0) {
    os << "never (|error| still > " << band << " at the end)\n";
  } else {
    os << m.settling_time << " s  (into the +/-" << band << " band)\n";
  }
  if (m.signal == SimScenario::Signal::kPush) {
    os << "  peak deviation   " << m.peak_deviation << "  (max |q - q_des| after the push)\n";
  } else {
    os << "  overshoot        " << m.overshoot_frac * 100.0 << " %\n";
  }
  os << "  steady-state err " << m.ss_error << "\n"
     << "  peak |tau|       " << m.peak_tau << " Nm";
  if (m.saturation_frac > 0.0) {
    os << "  (SATURATED " << m.saturation_frac * 100.0 << "% of ticks)";
  }
  os << "\n  status ok        " << m.ok_frac * 100.0 << " % of ticks"
     << "  (worst margin " << m.min_margin << ")\n";
  return os.str();
}

}  // namespace kontrolem_workbench
