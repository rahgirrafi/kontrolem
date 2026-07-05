# Controllers

All three controllers share `kontrolem_controllers_base`: the artifact loader,
the deviation-state assembly, the validity guard, the chainable plumbing, and
the two numeric cores. A controller package is thin — it pins the accepted
artifact `controller_type` and picks the flavor (static gain vs dynamic
compensator).

```{toctree}
:maxdepth: 1

lqr
lqg
hinf
```

## Shared architecture

- **State.** On every update the base reads joint positions (and velocities,
  when the law needs them), forms `x = [q − q_eq, q̇]`, and runs the validity
  guard before computing a command.
- **Cores.** `StateFeedbackController` uses the static-gain core
  (`u = u_eq − K·x`); `OutputFeedbackController` uses the dynamic-compensator
  core (`ξ̇ = ctrl_A·ξ + ctrl_B·y`, `u = u_eq + ctrl_C·ξ + ctrl_D·y`),
  Tustin-discretized once in `on_configure()`.
- **Realtime.** The per-cycle path is allocation- and lock-free: preallocated
  Eigen, `.noalias()` products, a lock-free buffer for the reference topic. An
  instrumented test asserts zero heap traffic across many cycles.

(chaining)=

## Chaining

Because each controller is a `ChainableControllerInterface`, an upstream
controller can drive its setpoint by claiming its exported reference interfaces.
Example cascade (`lqr_controller`'s `config/lqr_chained_controllers.yaml`): a
stock `forward_command_controller` writes the LQR controller's position
references.

```yaml
controller_manager:
  ros__parameters:
    lqr_controller:
      type: lqr_controller/LqrController
    reference_generator:
      type: forward_command_controller/ForwardCommandController

reference_generator:
  ros__parameters:
    joints:
      - lqr_controller/cart_joint   # <downstream>/<joint>
      - lqr_controller/joint1
      - lqr_controller/joint2
    interface_name: position
```

When the upstream activates and claims `lqr_controller/<joint>/position`, the
controller manager puts `lqr_controller` into chained mode: it then reads its
setpoint from those reference interfaces instead of the `~/reference` topic. The
whole cascade is YAML — no custom upstream code.
