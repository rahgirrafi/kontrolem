// Pinocchio-backed implementation of the Layer-1 model service.
// Pinocchio headers appear ONLY in this translation unit (pImpl), so the public
// header and every downstream consumer stay Eigen-only.
#include "kontrolem_model/robot_model.hpp"

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/aba-derivatives.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
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

RobotModel RobotModel::from_urdf_file(const std::string & path)
{
  RobotModel m;
  try {
    pinocchio::urdf::buildModel(path, m.impl_->model);  // fixed base
  } catch (const std::exception & e) {
    throw std::runtime_error("RobotModel: failed to parse URDF '" + path + "': " + e.what());
  }
  collect_joint_names(m.impl_->model, m.impl_->joint_names);
  return m;
}

RobotModel RobotModel::from_urdf_string(const std::string & urdf_xml)
{
  RobotModel m;
  try {
    pinocchio::urdf::buildModelFromXML(urdf_xml, m.impl_->model);  // fixed base
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
  explicit Impl(const pinocchio::Model & model)
  : data(model), zero(Eigen::VectorXd::Zero(model.nv))
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

int RobotModel::nq() const { return impl_->model.nq; }
int RobotModel::nv() const { return impl_->model.nv; }
const std::vector<std::string> & RobotModel::joint_names() const { return impl_->joint_names; }

}  // namespace kontrolem_model
