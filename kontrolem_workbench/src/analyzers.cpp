#include "kontrolem_workbench/analyzers.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>

#include "kontrolem_control/problem.hpp"
#include "kontrolem_controllers/lpv_controller.hpp"
#include "kontrolem_controllers/lqg_controller.hpp"
#include "kontrolem_controllers/lqr_controller.hpp"
#include "kontrolem_controllers/mpc_controller.hpp"

namespace kontrolem_workbench
{
namespace kc = kontrolem_control;
namespace kctl = kontrolem_controllers;
using kontrolem_model::RobotModel;

namespace
{

kc::Regulation regulation_at(const RobotModel & model, const Eigen::VectorXd & q_ref)
{
  kc::Regulation reg;
  reg.q_ref = q_ref;
  reg.v_ref = Eigen::VectorXd::Zero(model.nv());
  return reg;
}

/// Synthesize a law from its factory at the pose and hand back the artifact
/// (downcast by the caller to the law's Synthesis type).
std::unique_ptr<kc::Synthesis> synthesize_at(
  const RobotModel & model, const FactoryMap & laws, const std::string & law,
  const kc::ParameterMap & params, const std::vector<std::string> & actuated,
  const Eigen::VectorXd & q_ref)
{
  kc::BuildContext ctx;
  ctx.actuated_joints = actuated;
  auto controller = laws.at(law)->create(params, model, ctx);
  const kc::Regulation reg = regulation_at(model, q_ref);
  return controller->synthesize(model, reg);
}

EigReport eig_report(const Eigen::MatrixXd & Acl)
{
  Eigen::EigenSolver<Eigen::MatrixXd> es(Acl);
  EigReport r;
  r.poles = es.eigenvalues();
  r.max_re = r.poles.real().maxCoeff();
  r.stable = r.max_re < 0.0;
  double slowest_re = 0.0;
  for (Eigen::Index i = 0; i < r.poles.size(); ++i) {
    const double re = r.poles(i).real();
    const double im = r.poles(i).imag();
    if (std::abs(im) > 1e-9) {
      const double zeta = -re / std::hypot(re, im);
      r.worst_damping = std::min(r.worst_damping, zeta);
    }
    if (std::abs(re) > 1e-9 && (slowest_re == 0.0 || std::abs(re) < slowest_re)) {
      slowest_re = std::abs(re);
    }
  }
  r.dominant_tau = slowest_re > 0.0 ? 1.0 / slowest_re : 0.0;
  return r;
}

std::string format_eig(const EigReport & r, const std::string & indent)
{
  std::ostringstream os;
  os.precision(3);
  // Conjugate pairs print once (Im >= 0), sorted dominant (largest Re) first.
  std::vector<std::complex<double>> ps;
  for (Eigen::Index i = 0; i < r.poles.size(); ++i) {
    if (r.poles(i).imag() >= -1e-12) ps.push_back(r.poles(i));
  }
  std::sort(ps.begin(), ps.end(), [](const auto & a, const auto & b) {
    return a.real() > b.real();
  });
  for (const auto & p : ps) {
    os << indent << "s = " << p.real();
    if (std::abs(p.imag()) > 1e-9) {
      const double zeta = -p.real() / std::abs(p);
      os << " +/- " << std::abs(p.imag()) << "j   (zeta " << zeta << ")";
    }
    if (std::abs(p.real()) > 1e-9) os << "   tau " << 1.0 / std::abs(p.real()) << " s";
    os << "\n";
  }
  os << indent << (r.stable ? "STABLE" : "UNSTABLE") << " at this pose (max Re " << r.max_re
     << ")\n";
  return os.str();
}

}  // namespace

EigReport closed_loop_eig(
  const RobotModel & model, const Eigen::VectorXd & q, const Eigen::MatrixXd & K,
  const std::vector<int> & act_v)
{
  const Eigen::VectorXd tau_eq = model.gravity_torque(q);
  const auto lin = model.linearize(q, Eigen::VectorXd::Zero(model.nv()), tau_eq);
  if (K.size() == 0) {
    return eig_report(lin.A);
  }
  Eigen::MatrixXd B_act(lin.B.rows(), static_cast<Eigen::Index>(act_v.size()));
  for (std::size_t i = 0; i < act_v.size(); ++i) {
    B_act.col(static_cast<Eigen::Index>(i)) = lin.B.col(act_v[i]);
  }
  return eig_report(lin.A - B_act * K);
}

LqrPane analyze_lqr(
  const RobotModel & model, const FactoryMap & laws, const kc::ParameterMap & params,
  const std::vector<std::string> & actuated, const Eigen::VectorXd & q_ref)
{
  auto syn = synthesize_at(model, laws, "lqr", params, actuated, q_ref);
  const auto & s = static_cast<const kctl::LqrSynthesis &>(*syn);
  LqrPane pane;
  pane.cl = closed_loop_eig(model, q_ref, s.K, s.act_v);
  pane.q_dev_max = params.double_at("lqr.q_dev_max");
  return pane;
}

LqgPane analyze_lqg(
  const RobotModel & model, const FactoryMap & laws, const kc::ParameterMap & params,
  const std::vector<std::string> & actuated, const Eigen::VectorXd & q_ref)
{
  auto syn = synthesize_at(model, laws, "lqg", params, actuated, q_ref);
  const auto & s = static_cast<const kctl::LqgSynthesis &>(*syn);
  LqgPane pane;
  pane.ctrl = closed_loop_eig(model, q_ref, s.K, s.act_v);
  pane.est = eig_report(s.A_obs);  // A - L C: the estimator's error dynamics
  if (pane.ctrl.dominant_tau > 0.0 && pane.est.dominant_tau > 0.0) {
    pane.est_speedup = pane.ctrl.dominant_tau / pane.est.dominant_tau;
  }
  return pane;
}

LpvPane analyze_lpv(
  const RobotModel & model, const FactoryMap & laws, const kc::ParameterMap & params,
  const std::vector<std::string> & actuated, const Eigen::VectorXd & q_ref)
{
  auto syn = synthesize_at(model, laws, "lpv", params, actuated, q_ref);
  const auto & s = static_cast<const kctl::LpvSynthesis &>(*syn);
  const int D = static_cast<int>(s.axes.size());

  LpvPane pane;
  pane.nodes = static_cast<int>(s.K.size());
  pane.all_nodes_stable = true;
  pane.all_midpoints_stable = true;
  pane.worst_node_re = -1e300;
  pane.worst_mid_re = -1e300;

  // Every designed node: eig with ITS gain at ITS pose (row-major, last axis
  // fastest — the LpvController layout).
  for (int lin = 0; lin < pane.nodes; ++lin) {
    Eigen::VectorXd q_op = q_ref;
    int rem = lin;
    for (int d = D - 1; d >= 0; --d) {
      const auto & ax = s.axes[static_cast<std::size_t>(d)];
      const int i = rem % ax.n;
      rem /= ax.n;
      q_op(ax.q_index) = ax.min + i * (ax.max - ax.min) / (ax.n - 1);
    }
    const EigReport r =
      closed_loop_eig(model, q_op, s.K[static_cast<std::size_t>(lin)], s.act_v);
    pane.worst_node_re = std::max(pane.worst_node_re, r.max_re);
    pane.worst_node_damping = std::min(pane.worst_node_damping, r.worst_damping);
    pane.all_nodes_stable = pane.all_nodes_stable && r.stable;
  }

  // Every cell midpoint: the INTERPOLATED gain against the true local plant,
  // AND — at the same pose — a freshly CARE-designed gain. Their spectral-
  // abscissa difference is the pure cost of interpolating instead of designing
  // (the Shamma-Athans gap the point-wise scheme has no certificate for);
  // comparing at the same pose removes the pose difficulty itself from the
  // measure, and the gap shrinks as the grid refines.
  const int nv = model.nv();
  const int m = static_cast<int>(actuated.size());
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2 * nv, 2 * nv);
  {
    const auto & d = params.double_array_at("lpv.q_diag");
    if (static_cast<int>(d.size()) == 2 * nv) {
      for (int i = 0; i < 2 * nv; ++i) Q(i, i) = d[static_cast<std::size_t>(i)];
    }
  }
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(m, m);
  {
    const auto & d = params.double_array_at("lpv.r_diag");
    if (static_cast<int>(d.size()) == m) {
      for (int i = 0; i < m; ++i) R(i, i) = d[static_cast<std::size_t>(i)];
    }
  }

  int cells = 1;
  for (const auto & ax : s.axes) cells *= ax.n - 1;
  pane.midpoints = cells;
  pane.interp_gap = 0.0;
  const int corners = 1 << D;
  for (int c = 0; c < cells; ++c) {
    std::vector<int> lo(static_cast<std::size_t>(D));
    int rem = c;
    for (int d = D - 1; d >= 0; --d) {
      lo[static_cast<std::size_t>(d)] = rem % (s.axes[static_cast<std::size_t>(d)].n - 1);
      rem /= s.axes[static_cast<std::size_t>(d)].n - 1;
    }
    Eigen::VectorXd q_mid = q_ref;
    for (int d = 0; d < D; ++d) {
      const auto & ax = s.axes[static_cast<std::size_t>(d)];
      const double step = (ax.max - ax.min) / (ax.n - 1);
      q_mid(ax.q_index) = ax.min + (lo[static_cast<std::size_t>(d)] + 0.5) * step;
    }
    Eigen::MatrixXd K_mid = Eigen::MatrixXd::Zero(s.K.front().rows(), s.K.front().cols());
    for (int k = 0; k < corners; ++k) {
      int idx = 0, stride = 1;
      for (int d = D - 1; d >= 0; --d) {
        idx += (lo[static_cast<std::size_t>(d)] + ((k >> d) & 1)) * stride;
        stride *= s.axes[static_cast<std::size_t>(d)].n;
      }
      K_mid += s.K[static_cast<std::size_t>(idx)];
    }
    K_mid /= static_cast<double>(corners);
    const EigReport interp = closed_loop_eig(model, q_mid, K_mid, s.act_v);
    pane.worst_mid_re = std::max(pane.worst_mid_re, interp.max_re);
    pane.all_midpoints_stable = pane.all_midpoints_stable && interp.stable;

    // A fresh LQR at this exact pose — the reuse of the tested synthesis path.
    kctl::LqrController fresh(actuated, Q, R);
    const kc::Regulation reg_mid = regulation_at(model, q_mid);
    auto fs = fresh.synthesize(model, reg_mid);
    const auto & fsyn = static_cast<const kctl::LqrSynthesis &>(*fs);
    const EigReport designed = closed_loop_eig(model, q_mid, fsyn.K, fsyn.act_v);
    pane.interp_gap = std::max(pane.interp_gap, interp.max_re - designed.max_re);
  }
  return pane;
}

MpcPane analyze_mpc(
  const RobotModel & model, const FactoryMap & laws, const kc::ParameterMap & params,
  const std::vector<std::string> & actuated, const Eigen::VectorXd & q_ref)
{
  auto syn = synthesize_at(model, laws, "mpc", params, actuated, q_ref);
  const auto & s = static_cast<const kctl::MpcSynthesis &>(*syn);
  MpcPane pane;
  const Eigen::VectorXd ev =
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(s.H).eigenvalues();
  pane.cond_H = ev(ev.size() - 1) / std::max(ev(0), 1e-300);
  pane.window_s =
    static_cast<double>(params.int_at("mpc.horizon")) * params.double_at("mpc.dt_mpc");
  pane.open_loop = closed_loop_eig(model, q_ref, Eigen::MatrixXd{}, {});
  // The horizon must at least see the plant's dominant timescale (fastest
  // unstable or slowest stable non-integrator mode).
  double slowest = 0.0;
  for (Eigen::Index i = 0; i < pane.open_loop.poles.size(); ++i) {
    const double re = std::abs(pane.open_loop.poles(i).real());
    if (re > 1e-6) slowest = std::max(slowest, 1.0 / re);
  }
  pane.plant_slowest_tau = slowest;
  pane.window_covers = pane.window_s >= slowest;
  return pane;
}

QpPane analyze_qp(
  const RobotModel & model, const kc::ParameterMap & params,
  const std::vector<std::string> & actuated, const Eigen::VectorXd & q_ref)
{
  QpPane pane;
  const double kp = params.double_at("qp.kp");
  const double kd = params.double_at("qp.kd");
  pane.omega_n = std::sqrt(kp);
  pane.zeta = kd / (2.0 * std::sqrt(kp));

  // 2% settling of the imposed error dynamics e'' + kd e' + kp e = 0, e(0)=1,
  // e'(0)=0 — scanned numerically so every damping regime is handled the same.
  {
    const double dt = 1e-4;
    double e = 1.0, ed = 0.0, last_outside = 0.0;
    const double t_max = 60.0 / std::max(pane.omega_n, 1e-6);
    for (double t = 0.0; t < t_max; t += dt) {
      if (std::abs(e) > 0.02) last_outside = t;
      const double edd = -kd * ed - kp * e;
      ed += edd * dt;
      e += ed * dt;
    }
    pane.t_settle_pred = last_outside;
  }

  const Eigen::VectorXd g = model.gravity_torque(q_ref);
  double hold = 0.0;
  for (const auto & j : actuated) hold = std::max(hold, std::abs(g(model.joint_v_index(j))));
  const double tau_max = params.double_at("qp.tau_max");
  pane.hold_tau_frac = tau_max > 0.0 ? hold / tau_max : 0.0;
  return pane;
}

std::string law_pane_text(
  const std::string & law, const RobotModel & model, const FactoryMap & laws,
  const kc::ParameterMap & params, const std::vector<std::string> & actuated,
  const Eigen::VectorXd & q_ref)
{
  std::ostringstream os;
  os.precision(4);
  os << "-- " << law << " pane (local analysis at the configured pose) --\n";
  if (law == "lqr") {
    const LqrPane p = analyze_lqr(model, laws, params, actuated, q_ref);
    os << "  closed-loop poles of A - BK:\n" << format_eig(p.cl, "    ")
       << "  trust region: q_dev_max " << p.q_dev_max
       << " (the supervisor distrusts the law beyond it)\n";
  } else if (law == "lqg") {
    const LqgPane p = analyze_lqg(model, laws, params, actuated, q_ref);
    os << "  controller poles, eig(A - BK):\n" << format_eig(p.ctrl, "    ")
       << "  estimator poles, eig(A - LC):\n" << format_eig(p.est, "    ")
       << "  estimator is " << p.est_speedup << "x faster than the controller ("
       << (p.est_speedup > 1.0 ? "as it should be" : "TOO SLOW — raise lqg.w_diag or trust "
                                                     "measurements more via lqg.v_diag")
       << ")\n";
  } else if (law == "lpv") {
    const LpvPane p = analyze_lpv(model, laws, params, actuated, q_ref);
    os << "  " << p.nodes << " designed nodes: "
       << (p.all_nodes_stable ? "all locally stable" : "SOME UNSTABLE") << ", worst max Re "
       << p.worst_node_re << ", worst damping " << p.worst_node_damping << "\n"
       << "  " << p.midpoints << " inter-node midpoints (interpolated gain vs true local "
       << "plant): " << (p.all_midpoints_stable ? "all stable" : "SOME UNSTABLE")
       << ", worst max Re " << p.worst_mid_re << "\n"
       << "  interpolation gap: " << p.interp_gap
       << "  (worst spectral-abscissa loss of the interpolated gain vs a freshly "
       << "designed one at the same pose — the point-wise design has no certificate "
       << "between nodes; this measures that risk empirically, and it shrinks with a "
       << "finer grid)\n";
  } else if (law == "mpc") {
    const MpcPane p = analyze_mpc(model, laws, params, actuated, q_ref);
    os << "  plant (open-loop) poles at the pose:\n" << format_eig(p.open_loop, "    ")
       << "  prediction window: " << p.window_s << " s vs dominant plant timescale "
       << p.plant_slowest_tau << " s -> "
       << (p.window_covers ? "covered" : "TOO SHORT — raise mpc.horizon") << "\n"
       << "  condensed Hessian conditioning: " << p.cond_H
       << (p.cond_H > 1e8 ? "  (ILL-CONDITIONED — shorten the horizon)" : "  (healthy)")
       << "\n";
  } else if (law == "qp") {
    const QpPane p = analyze_qp(model, params, actuated, q_ref);
    os << "  imposed error dynamics s^2 + kd s + kp: omega_n " << p.omega_n << " rad/s, zeta "
       << p.zeta << "\n"
       << "  predicted 2% settling " << p.t_settle_pred
       << " s  (compare with the simulated value above — a mismatch means the QP's "
       << "constraints are biting)\n"
       << "  holding the pose uses " << p.hold_tau_frac * 100.0 << "% of qp.tau_max"
       << (p.hold_tau_frac > 0.8 ? "  (LITTLE AUTHORITY LEFT — raise qp.tau_max)" : "")
       << "\n";
  } else {
    os << "  no law-specific pane for '" << law
       << "' — the simulated tier above is the evidence for this law\n";
  }
  return os.str();
}

}  // namespace kontrolem_workbench
