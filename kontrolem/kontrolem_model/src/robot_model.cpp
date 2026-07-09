// Pinocchio-backed implementation of the Layer-1 model service.
// Pinocchio headers appear ONLY in this translation unit (pImpl), so the public
// header and every downstream consumer stay Eigen-only.
#include "kontrolem_model/robot_model.hpp"

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/aba-derivatives.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <stdexcept>

namespace kontrolem_model
{

struct RobotModel::Impl
{
  pinocchio::Model model;
  std::vector<std::string> joint_names;
};

RobotModel::RobotModel() : impl_(std::make_shared<Impl>()) {}

namespace
{
void collect_joint_names(
  const pinocchio::Model & model, std::vector<std::string> & out)
{
  // Movable joint names, skipping the "universe" joint at index 0.
  for (std::size_t i = 1; i < model.names.size(); ++i) {
    out.push_back(model.names[i]);
  }
}
}  // namespace

RobotModel RobotModel::from_urdf_file(const std::string & path, BaseType base)
{
  RobotModel m;
  try {
    if (base == BaseType::kFloating) {
      pinocchio::urdf::buildModel(path, pinocchio::JointModelFreeFlyer(), m.impl_->model);
    } else {
      pinocchio::urdf::buildModel(path, m.impl_->model);
    }
  } catch (const std::exception & e) {
    throw std::runtime_error("RobotModel: failed to parse URDF '" + path + "': " + e.what());
  }
  collect_joint_names(m.impl_->model, m.impl_->joint_names);
  return m;
}

RobotModel RobotModel::from_urdf_string(const std::string & urdf_xml, BaseType base)
{
  RobotModel m;
  try {
    if (base == BaseType::kFloating) {
      pinocchio::urdf::buildModelFromXML(
        urdf_xml, pinocchio::JointModelFreeFlyer(), m.impl_->model);
    } else {
      pinocchio::urdf::buildModelFromXML(urdf_xml, m.impl_->model);
    }
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("RobotModel: failed to parse URDF XML: ") + e.what());
  }
  collect_joint_names(m.impl_->model, m.impl_->joint_names);
  return m;
}

Eigen::VectorXd RobotModel::aba(
  const Eigen::VectorXd & q, const Eigen::VectorXd & v, const Eigen::VectorXd & tau) const
{
  pinocchio::Data data(impl_->model);
  return pinocchio::aba(impl_->model, data, q, v, tau);
}

Eigen::VectorXd RobotModel::gravity_torque(const Eigen::VectorXd & q) const
{
  pinocchio::Data data(impl_->model);
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(impl_->model.nv);
  return pinocchio::rnea(impl_->model, data, q, zero, zero);
}

Dynamics RobotModel::dynamics(const Eigen::VectorXd & q, const Eigen::VectorXd & v) const
{
  const auto & model = impl_->model;
  pinocchio::Data data(model);  // allocates — convenience/offline only
  Dynamics d;
  d.M = pinocchio::crba(model, data, q);  // upper-triangular; mirror below
  d.M.triangularView<Eigen::StrictlyLower>() =
    d.M.transpose().triangularView<Eigen::StrictlyLower>();
  d.h = pinocchio::rnea(model, data, q, v, Eigen::VectorXd::Zero(model.nv));  // C v + g
  return d;
}

// --- Workspace (preallocated Data) + real-time dynamics query (U4 fix) -------

struct RobotModel::Workspace::Impl
{
  pinocchio::Data data;
  Eigen::VectorXd zero;  // preallocated a = 0 argument for rnea
  // 6 x nv scratch for per-frame Jacobians. FIXED 6 rows (matches Pinocchio's
  // Data::Matrix6x) so getFrameJacobian binds it without a temporary allocation.
  Eigen::Matrix<double, 6, Eigen::Dynamic> J6;
  explicit Impl(const pinocchio::Model & model)
  : data(model), zero(Eigen::VectorXd::Zero(model.nv)),
    J6(Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, model.nv))
  {
  }
};

RobotModel::Workspace::Workspace() = default;
RobotModel::Workspace::Workspace(Workspace &&) noexcept = default;
RobotModel::Workspace & RobotModel::Workspace::operator=(Workspace &&) noexcept = default;
RobotModel::Workspace::~Workspace() = default;

RobotModel::Workspace RobotModel::make_workspace() const
{
  Workspace ws;
  ws.impl_ = std::make_unique<Workspace::Impl>(impl_->model);  // the only allocation
  return ws;
}

void RobotModel::dynamics(
  Workspace & ws, const Eigen::VectorXd & q, const Eigen::VectorXd & v, Eigen::MatrixXd & M_out,
  Eigen::VectorXd & h_out) const
{
  const auto & model = impl_->model;
  auto & data = ws.impl_->data;  // reused; no allocation
  pinocchio::crba(model, data, q);
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  M_out = data.M;                                          // into preallocated buffer
  h_out = pinocchio::rnea(model, data, q, v, ws.impl_->zero);  // into preallocated buffer
}

Linearization RobotModel::linearize(
  const Eigen::VectorXd & q, const Eigen::VectorXd & v, const Eigen::VectorXd & tau) const
{
  const auto & model = impl_->model;
  const int nv = model.nv;
  pinocchio::Data data(model);

  // Analytic partials of the forward dynamics a = ABA(q,v,tau).
  // Fills data.ddq_dq, data.ddq_dv (nv x nv) and data.Minv (upper-triangular).
  pinocchio::computeABADerivatives(model, data, q, v, tau);

  // Minv is returned upper-triangular (symmetric matrix) — mirror it.
  data.Minv.triangularView<Eigen::StrictlyLower>() =
    data.Minv.transpose().triangularView<Eigen::StrictlyLower>();

  Linearization lin;
  // A = [[0, I],[ddq_dq, ddq_dv]]  (xdot = [v; a])
  lin.A.setZero(2 * nv, 2 * nv);
  lin.A.topRightCorner(nv, nv).setIdentity();
  lin.A.bottomLeftCorner(nv, nv) = data.ddq_dq;
  lin.A.bottomRightCorner(nv, nv) = data.ddq_dv;
  // B = [[0],[Minv]]  (input = generalized torque; actuation selection is a
  // controller/problem concern, not the model's).
  lin.B.setZero(2 * nv, nv);
  lin.B.bottomRows(nv) = data.Minv;
  return lin;
}

Eigen::Vector3d RobotModel::center_of_mass(const Eigen::VectorXd & q) const
{
  const auto & model = impl_->model;
  pinocchio::Data data(model);
  return pinocchio::centerOfMass(model, data, q);
}

Eigen::Vector3d RobotModel::frame_position(
  const Eigen::VectorXd & q, const std::string & frame) const
{
  const auto & model = impl_->model;
  if (!model.existFrame(frame)) {
    throw std::runtime_error("RobotModel::frame_position: no frame '" + frame + "'");
  }
  pinocchio::Data data(model);
  pinocchio::framesForwardKinematics(model, data, q);
  return data.oMf[model.getFrameId(frame)].translation();
}

Eigen::MatrixXd RobotModel::contact_jacobian(
  const Eigen::VectorXd & q, const std::string & frame) const
{
  const auto & model = impl_->model;
  if (!model.existFrame(frame)) {
    throw std::runtime_error("RobotModel::contact_jacobian: no frame '" + frame + "'");
  }
  pinocchio::Data data(model);
  const auto fid = model.getFrameId(frame);
  Eigen::MatrixXd J6 = Eigen::MatrixXd::Zero(6, model.nv);
  pinocchio::computeFrameJacobian(model, data, q, fid, pinocchio::LOCAL_WORLD_ALIGNED, J6);
  return J6.topRows(3);  // translational part: v_world = J * v_generalized
}

std::size_t RobotModel::frame_index(const std::string & name) const
{
  if (!impl_->model.existFrame(name)) {
    throw std::runtime_error("RobotModel::frame_index: no frame '" + name + "'");
  }
  return impl_->model.getFrameId(name);
}

void RobotModel::contact_jacobian_stacked(
  Workspace & ws, const Eigen::VectorXd & q, const std::vector<std::size_t> & frame_ids,
  Eigen::MatrixXd & J_out) const
{
  const auto & model = impl_->model;
  auto & data = ws.impl_->data;
  const int nv = model.nv;
  const int nc = static_cast<int>(frame_ids.size());
  if (J_out.rows() != 3 * nc || J_out.cols() != nv) {
    J_out.resize(3 * nc, nv);
  }
  pinocchio::computeJointJacobians(model, data, q);  // fills data.J once
  pinocchio::updateFramePlacements(model, data);
  for (int k = 0; k < nc; ++k) {
    const auto fid = frame_ids[static_cast<std::size_t>(k)];
    // Compute in the frame-LOCAL basis and rotate the translational rows to world
    // with the frame rotation — allocation-free (the LOCAL_WORLD_ALIGNED variant
    // allocates a temporary internally, and a per-tick name lookup would too).
    ws.impl_->J6.setZero();
    pinocchio::getFrameJacobian(model, data, fid, pinocchio::LOCAL, ws.impl_->J6);
    J_out.middleRows(3 * k, 3).noalias() = data.oMf[fid].rotation() * ws.impl_->J6.topRows(3);
  }
}

void RobotModel::contact_jacobian_stacked(
  Workspace & ws, const Eigen::VectorXd & q, const std::vector<std::string> & feet,
  Eigen::MatrixXd & J_out) const
{
  std::vector<std::size_t> ids;
  ids.reserve(feet.size());
  for (const auto & f : feet) {
    ids.push_back(frame_index(f));
  }
  contact_jacobian_stacked(ws, q, ids, J_out);
}

void RobotModel::contact_drift(
  Workspace & ws, const Eigen::VectorXd & q, const Eigen::VectorXd & v,
  const std::vector<std::size_t> & frame_ids, Eigen::VectorXd & gamma_out) const
{
  const auto & model = impl_->model;
  auto & data = ws.impl_->data;
  const int nc = static_cast<int>(frame_ids.size());
  if (gamma_out.size() != 3 * nc) {
    gamma_out.resize(3 * nc);
  }
  // Classical acceleration at zero joint acceleration is exactly d/dt(J)·v.
  pinocchio::forwardKinematics(model, data, q, v, ws.impl_->zero);
  pinocchio::updateFramePlacements(model, data);
  for (int k = 0; k < nc; ++k) {
    const auto fid = frame_ids[static_cast<std::size_t>(k)];
    const auto acc = pinocchio::getFrameClassicalAcceleration(model, data, fid, pinocchio::LOCAL);
    gamma_out.segment(3 * k, 3).noalias() = data.oMf[fid].rotation() * acc.linear();
  }
}

void RobotModel::contact_drift(
  Workspace & ws, const Eigen::VectorXd & q, const Eigen::VectorXd & v,
  const std::vector<std::string> & feet, Eigen::VectorXd & gamma_out) const
{
  std::vector<std::size_t> ids;
  ids.reserve(feet.size());
  for (const auto & f : feet) {
    ids.push_back(frame_index(f));
  }
  contact_drift(ws, q, v, ids, gamma_out);
}

Eigen::VectorXd RobotModel::contact_forward_dynamics(
  Workspace & ws, const Eigen::VectorXd & q, const Eigen::VectorXd & v,
  const Eigen::VectorXd & tau, const std::vector<std::string> & feet,
  const std::vector<Eigen::Vector3d> & anchors, double baumgarte_kp, double baumgarte_kd,
  Eigen::VectorXd * lambda_out) const
{
  const auto & model = impl_->model;
  auto & data = ws.impl_->data;
  const int nv = model.nv;
  const int nc = static_cast<int>(feet.size());

  // M (crba, mirrored) and h (rnea with a=0) — captured before J/gamma recompute
  // data via kinematics calls below.
  pinocchio::crba(model, data, q);
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  const Eigen::MatrixXd M = data.M;
  const Eigen::VectorXd h = pinocchio::rnea(model, data, q, v, ws.impl_->zero);

  const Eigen::LDLT<Eigen::MatrixXd> Mldlt(M);
  if (nc == 0) {
    return Mldlt.solve(tau - h);  // no contacts ⇒ free dynamics (== aba)
  }

  Eigen::MatrixXd J(3 * nc, nv);
  contact_jacobian_stacked(ws, q, feet, J);
  Eigen::VectorXd gamma(3 * nc);
  contact_drift(ws, q, v, feet, gamma);  // leaves data.oMf valid at q

  if (baumgarte_kp != 0.0 || baumgarte_kd != 0.0) {
    const Eigen::VectorXd Jv = J * v;
    for (int k = 0; k < nc; ++k) {
      const Eigen::Vector3d pos = data.oMf[model.getFrameId(feet[k])].translation();
      const Eigen::Vector3d anchor =
        (static_cast<int>(anchors.size()) > k) ? anchors[static_cast<std::size_t>(k)] : pos;
      gamma.segment(3 * k, 3) +=
        baumgarte_kp * (pos - anchor) + baumgarte_kd * Jv.segment(3 * k, 3);
    }
  }

  // Damped Schur complement: (J Minv Jᵀ + εI) λ = −γ − J Minv (tau−h); then
  // q̈ = Minv (tau−h) + Minv Jᵀ λ. Damping keeps redundant contacts well-posed.
  const Eigen::VectorXd Minv_rhs = Mldlt.solve(tau - h);
  const Eigen::MatrixXd MinvJt = Mldlt.solve(J.transpose());
  Eigen::MatrixXd Gm = J * MinvJt;
  Gm.diagonal().array() += 1e-8;
  const Eigen::VectorXd lambda = Gm.ldlt().solve(-gamma - J * Minv_rhs);
  if (lambda_out != nullptr) {
    *lambda_out = lambda;
  }
  return Minv_rhs + MinvJt * lambda;
}

Eigen::VectorXd RobotModel::integrate(
  const Eigen::VectorXd & q, const Eigen::VectorXd & v, double dt) const
{
  // pinocchio::integrate applies the group exponential per joint, so the
  // free-flyer root advances on SE(3) and its quaternion stays unit.
  return pinocchio::integrate(impl_->model, q, (v * dt).eval());
}

Eigen::VectorXd RobotModel::neutral() const
{
  return pinocchio::neutral(impl_->model);
}

Eigen::VectorXd RobotModel::difference(
  const Eigen::VectorXd & q0, const Eigen::VectorXd & q1) const
{
  // Tangent d with integrate(q0, d) == q1; SE(3) log on the free-flyer root.
  return pinocchio::difference(impl_->model, q0, q1);
}

void RobotModel::difference(
  const Eigen::VectorXd & q0, const Eigen::VectorXd & q1, Eigen::VectorXd & out) const
{
  // Write-into-buffer form (no return-value allocation): the RT overload.
  pinocchio::difference(impl_->model, q0, q1, out);
}

RobotModel::Trajectory RobotModel::rollout(
  const Eigen::VectorXd & q0, const Eigen::VectorXd & v0,
  const std::vector<Eigen::VectorXd> & tau_seq, double dt) const
{
  Trajectory traj;
  traj.q.reserve(tau_seq.size() + 1);
  traj.v.reserve(tau_seq.size() + 1);
  Eigen::VectorXd q = q0, v = v0;
  traj.q.push_back(q);
  traj.v.push_back(v);
  for (const auto & tau : tau_seq) {
    const Eigen::VectorXd a = aba(q, v, tau);
    v = v + a * dt;              // semi-implicit Euler
    q = integrate(q, v, dt);    // manifold-correct
    traj.q.push_back(q);
    traj.v.push_back(v);
  }
  return traj;
}

std::vector<Linearization> RobotModel::linearize_along(
  const std::vector<Eigen::VectorXd> & q, const std::vector<Eigen::VectorXd> & v,
  const std::vector<Eigen::VectorXd> & tau) const
{
  if (q.size() != v.size() || q.size() != tau.size()) {
    throw std::runtime_error("RobotModel::linearize_along: q, v, tau length mismatch");
  }
  std::vector<Linearization> out;
  out.reserve(q.size());
  for (std::size_t k = 0; k < q.size(); ++k) {
    out.push_back(linearize(q[k], v[k], tau[k]));
  }
  return out;
}

int RobotModel::nq() const { return impl_->model.nq; }
int RobotModel::nv() const { return impl_->model.nv; }
const std::vector<std::string> & RobotModel::joint_names() const { return impl_->joint_names; }

int RobotModel::joint_q_index(const std::string & name) const
{
  const auto & model = impl_->model;
  if (!model.existJointName(name)) {
    throw std::runtime_error("RobotModel::joint_q_index: no joint '" + name + "'");
  }
  return model.idx_qs[model.getJointId(name)];
}

int RobotModel::joint_v_index(const std::string & name) const
{
  const auto & model = impl_->model;
  if (!model.existJointName(name)) {
    throw std::runtime_error("RobotModel::joint_v_index: no joint '" + name + "'");
  }
  return model.idx_vs[model.getJointId(name)];
}

}  // namespace kontrolem_model
