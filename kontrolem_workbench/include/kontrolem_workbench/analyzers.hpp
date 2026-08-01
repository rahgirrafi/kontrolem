// Law-specific analysis panes (M18): each law gets THE analysis appropriate to
// it — eigenvalues where a linear design exists, imposed-dynamics predictions
// where the law dictates its own error dynamics, conditioning/adequacy checks
// for the optimization laws. Everything here is LOCAL analysis at a pose:
// evidence, not a certificate (stated in the formatted output). ROS-free.
//
// v1 panes are keyed by law name inside the workbench (see law_pane_text); an
// unknown/third-party law gets the shared simulation tier plus a notice. A
// pluggable per-law analyzer interface is a possible later upgrade.
#ifndef KONTROLEM_WORKBENCH__ANALYZERS_HPP_
#define KONTROLEM_WORKBENCH__ANALYZERS_HPP_

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "kontrolem_control/params.hpp"
#include "kontrolem_model/robot_model.hpp"
#include "kontrolem_workbench/lint.hpp"

namespace kontrolem_workbench
{

/// Eigenvalue summary of a closed (or open) loop at a pose.
struct EigReport
{
  Eigen::VectorXcd poles;
  bool stable{false};         ///< every real part < 0
  double max_re{0.0};         ///< spectral abscissa (most positive real part)
  double worst_damping{1.0};  ///< min zeta over oscillatory pairs (1 if none)
  double dominant_tau{0.0};   ///< 1/|Re| of the slowest pole (s)
};

/// eig(A - B_act K) of the plant linearized at (q, v=0, tau=gravity), with K
/// acting on the actuated rows `act_v`. K empty -> open-loop eig(A).
EigReport closed_loop_eig(
  const kontrolem_model::RobotModel & model, const Eigen::VectorXd & q,
  const Eigen::MatrixXd & K, const std::vector<int> & act_v);

struct LqrPane
{
  EigReport cl;
  double q_dev_max{0.0};  ///< the configured trust region
};

struct LqgPane
{
  EigReport ctrl;        ///< controller poles eig(A - B K)
  EigReport est;         ///< estimator poles eig(A - L C)
  double est_speedup{0.0};  ///< dominant estimator |Re| / dominant controller |Re| (want > 1)
};

struct LpvPane
{
  int nodes{0};
  int midpoints{0};
  double worst_node_re{0.0};  ///< max spectral abscissa over the designed nodes
  double worst_mid_re{0.0};   ///< max spectral abscissa at inter-node midpoints
                              ///< (interpolated gain vs the true local plant)
  double interp_gap{0.0};     ///< the Shamma-Athans gap: worst over midpoints of
                              ///< abscissa(interpolated gain) - abscissa(freshly
                              ///< designed gain) AT THE SAME POSE — the pure cost
                              ///< of interpolating instead of designing; shrinks
                              ///< with grid refinement
  double worst_node_damping{1.0};
  bool all_nodes_stable{false};
  bool all_midpoints_stable{false};
};

struct MpcPane
{
  double cond_H{0.0};          ///< condensed-QP Hessian conditioning
  double window_s{0.0};        ///< horizon * dt_mpc
  double plant_slowest_tau{0.0};  ///< slowest relevant plant timescale at the pose
  bool window_covers{false};   ///< window >= that timescale
  EigReport open_loop;         ///< the plant the MPC predicts with
};

struct QpPane
{
  double zeta{0.0};            ///< kd / (2 sqrt(kp))
  double omega_n{0.0};         ///< sqrt(kp)
  double t_settle_pred{0.0};   ///< predicted 2% settling of the IMPOSED error dynamics
  double hold_tau_frac{0.0};   ///< |gravity torque at the pose| / tau_max (authority used at rest)
};

LqrPane analyze_lqr(
  const kontrolem_model::RobotModel & model, const FactoryMap & laws,
  const kontrolem_control::ParameterMap & params, const std::vector<std::string> & actuated,
  const Eigen::VectorXd & q_ref);
LqgPane analyze_lqg(
  const kontrolem_model::RobotModel & model, const FactoryMap & laws,
  const kontrolem_control::ParameterMap & params, const std::vector<std::string> & actuated,
  const Eigen::VectorXd & q_ref);
LpvPane analyze_lpv(
  const kontrolem_model::RobotModel & model, const FactoryMap & laws,
  const kontrolem_control::ParameterMap & params, const std::vector<std::string> & actuated,
  const Eigen::VectorXd & q_ref);
MpcPane analyze_mpc(
  const kontrolem_model::RobotModel & model, const FactoryMap & laws,
  const kontrolem_control::ParameterMap & params, const std::vector<std::string> & actuated,
  const Eigen::VectorXd & q_ref);
QpPane analyze_qp(
  const kontrolem_model::RobotModel & model,
  const kontrolem_control::ParameterMap & params, const std::vector<std::string> & actuated,
  const Eigen::VectorXd & q_ref);

/// The dispatcher: the formatted pane for `law_name`, or a notice that only the
/// shared simulation tier applies (unknown / not-yet-covered laws).
std::string law_pane_text(
  const std::string & law_name, const kontrolem_model::RobotModel & model,
  const FactoryMap & laws, const kontrolem_control::ParameterMap & params,
  const std::vector<std::string> & actuated, const Eigen::VectorXd & q_ref);

}  // namespace kontrolem_workbench

#endif  // KONTROLEM_WORKBENCH__ANALYZERS_HPP_
