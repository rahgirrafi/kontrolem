#include "kontrolem_workbench/simulator.hpp"

#include <fstream>
#include <memory>
#include <stdexcept>

#include "kontrolem_control/problem.hpp"
#include "kontrolem_control/trajectory.hpp"

namespace kontrolem_workbench
{
namespace kc = kontrolem_control;
using kontrolem_model::RobotModel;

SimTrace simulate(
  const RobotModel & model, kc::Controller & law,
  const std::vector<std::string> & actuated_joints, const SimScenario & sc)
{
  const int nq = model.nq();
  const int nv = model.nv();
  if (sc.q_ref.size() != nq) {
    throw std::runtime_error("simulate: q_ref must have length nq");
  }
  const Eigen::VectorXd v_ref =
    sc.v_ref.size() == nv ? sc.v_ref : Eigen::VectorXd::Zero(nv);

  // Actuation map: command entry i drives generalized-force row act_v[i].
  std::vector<int> act_v;
  for (const auto & j : actuated_joints) act_v.push_back(model.joint_v_index(j));

  // The problem, per the scenario's dialect.
  kc::Regulation reg;
  reg.q_ref = sc.q_ref;
  reg.v_ref = v_ref;
  kc::HarmonicReference harmonic;
  kc::Tracking trk;
  const bool sine = sc.signal == SimScenario::Signal::kSine;
  if (sine) {
    if (sc.sine_amp.size() != nq) {
      throw std::runtime_error("simulate: sine_amp must have length nq");
    }
    harmonic.center = sc.q_ref;
    harmonic.amp = sc.sine_amp;
    harmonic.phase = Eigen::VectorXd::Zero(nq);
    harmonic.omega = sc.sine_omega;
    trk.reference = &harmonic;
  }
  kc::ControlProblem & problem = sine ? static_cast<kc::ControlProblem &>(trk)
                                      : static_cast<kc::ControlProblem &>(reg);
  if (!kc::accepts(law, problem)) {
    throw std::runtime_error(
      std::string("this control law does not accept the ") +
      (sine ? "Tracking dialect (pick step/push/release instead of sine)"
            : "Regulation dialect"));
  }

  // The runtime lifecycle, minus the middleware.
  auto synthesis = law.synthesize(model, problem);
  law.configure(model, *synthesis, problem);

  Eigen::VectorXd q = sc.q0.size() == nq ? sc.q0 : sc.q_ref;
  Eigen::VectorXd v = sc.v0.size() == nv ? sc.v0 : Eigen::VectorXd::Zero(nv);

  kc::State st;
  st.q = q;
  st.v = v;
  st.t = 0.0;
  law.on_activate(st, problem);

  const int N = static_cast<int>(sc.duration / sc.dt);
  SimTrace tr;
  tr.signal = sc.signal;
  tr.t.reserve(N);
  const bool has_event =
    sc.signal == SimScenario::Signal::kStep || sc.signal == SimScenario::Signal::kPush;
  const double push_end = sc.t_event + sc.push_duration;
  tr.t_event = sc.signal == SimScenario::Signal::kPush ? push_end
               : has_event                             ? sc.t_event
                                                       : 0.0;

  Eigen::VectorXd tau_full(nv), qd(nq), vd(nv), ad(nv), tffd(nv);
  bool stepped = false;
  for (int k = 0; k < N; ++k) {
    const double t = k * sc.dt;
    if (sc.signal == SimScenario::Signal::kStep && !stepped && t >= sc.t_event) {
      if (sc.q_target.size() != nq) {
        throw std::runtime_error("simulate: kStep needs q_target of length nq");
      }
      reg.q_ref = sc.q_target;  // the setpoint jump (gains stay — that's the test)
      stepped = true;
    }

    st.q = q;
    st.v = v;
    st.t = t;
    const kc::Command & u = law.compute(st, problem, sc.dt);

    tau_full.setZero();
    for (std::size_t i = 0; i < act_v.size(); ++i) {
      tau_full(act_v[static_cast<std::size_t>(i)]) = u.tau(static_cast<Eigen::Index>(i));
    }
    if (sc.signal == SimScenario::Signal::kPush && t >= sc.t_event && t < push_end) {
      if (sc.push_tau.size() != nv) {
        throw std::runtime_error("simulate: kPush needs push_tau of length nv");
      }
      tau_full += sc.push_tau;
    }

    // Record before integrating (state/command pairs line up).
    tr.t.push_back(t);
    tr.q.push_back(q);
    tr.v.push_back(v);
    tr.tau.push_back(tau_full);
    if (sine) {
      harmonic.sample(t, qd, vd, ad, tffd);
      tr.q_des.push_back(qd);
    } else {
      tr.q_des.push_back(reg.q_ref);
    }
    tr.ok.push_back(law.status().ok);
    tr.margin.push_back(law.status().margin);
    tr.iters.push_back(law.status().iters);

    // True plant: semi-implicit Euler on the full nonlinear dynamics.
    const Eigen::VectorXd a = model.aba(q, v, tau_full);
    v += a * sc.dt;
    q = model.integrate(q, v, sc.dt);
    if (!q.allFinite() || !v.allFinite()) {
      tr.finite = false;
      break;
    }
  }
  return tr;
}

void write_csv(const SimTrace & tr, const std::string & path)
{
  std::ofstream f(path);
  if (!f) throw std::runtime_error("cannot write '" + path + "'");
  const auto nq = tr.q.empty() ? 0 : tr.q.front().size();
  const auto nv = tr.v.empty() ? 0 : tr.v.front().size();
  f << "t";
  for (Eigen::Index i = 0; i < nq; ++i) f << ",q" << i;
  for (Eigen::Index i = 0; i < nv; ++i) f << ",v" << i;
  for (Eigen::Index i = 0; i < nv; ++i) f << ",tau" << i;
  for (Eigen::Index i = 0; i < nq; ++i) f << ",qdes" << i;
  f << ",ok,margin,iters\n";
  for (std::size_t k = 0; k < tr.t.size(); ++k) {
    f << tr.t[k];
    for (Eigen::Index i = 0; i < nq; ++i) f << ',' << tr.q[k](i);
    for (Eigen::Index i = 0; i < nv; ++i) f << ',' << tr.v[k](i);
    for (Eigen::Index i = 0; i < nv; ++i) f << ',' << tr.tau[k](i);
    for (Eigen::Index i = 0; i < nq; ++i) f << ',' << tr.q_des[k](i);
    f << ',' << (tr.ok[k] ? 1 : 0) << ',' << tr.margin[k] << ',' << tr.iters[k] << "\n";
  }
}

}  // namespace kontrolem_workbench
