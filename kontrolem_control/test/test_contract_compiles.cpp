// Proves the Controller contract is coherent and implementable end-to-end:
// a trivial controller runs the full lifecycle (capabilities -> synthesize ->
// configure -> compute -> status) and the capability check works. If this
// compiles and runs, the interface shape is sound before any real controller
// (LQR, QP) is built against it.
#include <iostream>
#include <memory>

#include "kontrolem_control/controller.hpp"

using namespace kontrolem_control;

/// The simplest possible Controller: accepts Regulation, precomputes nothing,
/// commands zero torque, always reports OK. Exists only to exercise the shape.
class NullController : public Controller
{
public:
  explicit NullController(int n_actuated) { command_.tau = Eigen::VectorXd::Zero(n_actuated); }

  Capabilities capabilities() const override
  {
    return Capabilities{{Dialect::kRegulation}, /*needs_velocity_state=*/true};
  }

  std::unique_ptr<Synthesis> synthesize(const RobotModel &, const ControlProblem &) const override
  {
    return std::make_unique<Synthesis>();  // nothing to precompute
  }

  void configure(const RobotModel &, const Synthesis &, const ControlProblem &) override {}

  const Command & compute(const State &, const ControlProblem &, double) override
  {
    command_.tau.setZero();
    status_ = Status{true, 0.0};
    return command_;
  }

  const Status & status() const override { return status_; }

private:
  Command command_;
  Status status_{true, 0.0};
};

int main()
{
  // Build a real model too (proves kontrolem_control links Layer 1 cleanly).
  const RobotModel model = RobotModel::from_urdf_file(CART_POLE_URDF);

  NullController controller(/*n_actuated=*/1);  // cart is the one actuated joint

  Regulation upright;
  upright.q_ref = Eigen::VectorXd::Zero(model.nq());  // cart at 0, pole up (angle 0)
  upright.v_ref = Eigen::VectorXd::Zero(model.nv());

  if (!accepts(controller, upright)) {
    std::cout << "FAIL: controller should accept the Regulation dialect\n";
    return 1;
  }

  const auto synthesis = controller.synthesize(model, upright);
  controller.configure(model, *synthesis, upright);

  State state;
  state.q = Eigen::VectorXd::Zero(model.nq());
  state.v = Eigen::VectorXd::Zero(model.nv());
  state.t = 0.0;

  const Command & u = controller.compute(state, upright, 0.001);
  const Status & s = controller.status();

  const bool ok = (u.tau.size() == 1) && s.ok;
  std::cout << "contract lifecycle ran: command size=" << u.tau.size()
            << " status.ok=" << s.ok << " -> " << (ok ? "PASS" : "FAIL") << "\n";
  return ok ? 0 : 1;
}
