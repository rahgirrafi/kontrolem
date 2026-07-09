// rollout + linearize_along checks (M2.5 model queries):
//   1. rollout returns N+1 states of the right dims.
//   2. physical sanity: from EXACT upright with zero input the cart-pole stays
//      (unstable equilibrium); from a tilt it falls (|pole| grows).
//   3. integrator convergence: halving dt (doubling N) barely changes the final
//      state — rollout is a proper convergent integrator.
//   4. linearize_along returns one linearization per point, each equal to
//      linearize() at that (q,v,tau) — the LTV prediction model.
// Dependency-free (no gtest).
#include <cmath>
#include <iostream>
#include <vector>

#include <Eigen/Dense>

#include "kontrolem_model/robot_model.hpp"

using kontrolem_model::RobotModel;

int main()
{
  const RobotModel m = RobotModel::from_urdf_file(CART_POLE_URDF);
  const int nv = m.nv();

  auto zero_seq = [nv](int N) {
    return std::vector<Eigen::VectorXd>(static_cast<std::size_t>(N), Eigen::VectorXd::Zero(nv));
  };

  // (1) + (2a) equilibrium: exact upright, zero input -> stays.
  const int N = 400;
  const double dt = 0.005;
  Eigen::VectorXd q_up(2), v0(2);
  q_up << 0.0, 0.0;
  v0.setZero();
  const auto up = m.rollout(q_up, v0, zero_seq(N), dt);
  const bool dims_ok = up.q.size() == static_cast<std::size_t>(N + 1) &&
                       up.v.size() == static_cast<std::size_t>(N + 1) &&
                       up.q.back().size() == m.nq() && up.v.back().size() == nv;
  const bool equilibrium = std::abs(up.q.back()(1)) < 1e-9;

  // (2b) tilt falls.
  Eigen::VectorXd q_tilt(2);
  q_tilt << 0.0, 0.1;
  const auto fall = m.rollout(q_tilt, v0, zero_seq(N), dt);
  const bool falls = std::abs(fall.q.back()(1)) > 0.1;

  // (3) convergence: coarse vs fine over the same 1.0 s of tilted free-fall.
  const auto coarse = m.rollout(q_tilt, v0, zero_seq(100), 0.01);
  const auto fine = m.rollout(q_tilt, v0, zero_seq(400), 0.0025);
  const double conv_err = (coarse.q.back() - fine.q.back()).cwiseAbs().maxCoeff();
  const bool converges = conv_err < 5e-2;  // Euler: O(dt) — loose but meaningful

  // (4) linearize_along matches per-point linearize.
  const auto tau_seq = zero_seq(10);
  std::vector<Eigen::VectorXd> qs, vs;
  for (int k = 0; k < 10; ++k) { qs.push_back(fall.q[k]); vs.push_back(fall.v[k]); }
  const auto lins = m.linearize_along(qs, vs, tau_seq);
  bool along_ok = lins.size() == 10;
  for (int k = 0; k < 10 && along_ok; ++k) {
    const auto ref = m.linearize(qs[k], vs[k], tau_seq[k]);
    along_ok = ref.A.rows() == 2 * nv && ref.B.cols() == nv &&
               (lins[k].A - ref.A).cwiseAbs().maxCoeff() < 1e-12 &&
               (lins[k].B - ref.B).cwiseAbs().maxCoeff() < 1e-12;
  }

  std::cout << "dims=" << dims_ok << " equilibrium=" << equilibrium << " falls=" << falls
            << " conv_err=" << conv_err << " along_ok=" << along_ok << "\n";
  const bool ok = dims_ok && equilibrium && falls && converges && along_ok;
  std::cout << (ok ? "PASS" : "FAIL") << "\n";
  return ok ? 0 : 1;
}
