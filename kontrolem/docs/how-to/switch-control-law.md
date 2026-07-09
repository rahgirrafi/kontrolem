# How to switch the control law

> **For:** anyone who has run a demo. **Assumes:** a built, sourced workspace. **Goal:** run the same robot under a different control paradigm.

The control paradigm is chosen by one parameter, `control_law`, on `kontrolem_controller`. The runtime is identical; only this string (and that law's gains) change.

## Available values

| `control_law` | Paradigm | Works on |
|---|---|---|
| `lqr` | Static state-feedback gain | fixed-base, full-state |
| `lqg` | Output-feedback compensator (positions only) | fixed-base |
| `mpc` | Receding-horizon QP | fixed-base |
| `qp` | Online inverse-dynamics task-space QP | fully-actuated (e.g. arm) |
| `wbc` | Whole-body QP with contacts | floating-base |

See [Reference → Control laws](../reference/control-laws.md) for what each expects.

## Option A — launch a demo that already uses that law

The fastest way is to launch a preconfigured demo:

```bash
ros2 launch kontrolem_bringup cart_pole.launch.py       # lqr
ros2 launch kontrolem_bringup cart_pole_lqg.launch.py   # lqg
ros2 launch kontrolem_bringup cart_pole_mpc.launch.py   # mpc
ros2 launch kontrolem_bringup arm2.launch.py            # qp (2-DoF arm)
ros2 launch kontrolem_bringup quad_stand.launch.py      # wbc (quadruped)
```

The full list is in [Reference → Demos](../reference/demos.md).

## Option B — change the law in a config file

Each demo reads a YAML file under `kontrolem_bringup/config/`. To make the cart-pole use the online QP instead of LQR, edit `config/cart_pole_controllers.yaml`:

```yaml
kontrolem_controller:
  ros__parameters:
    control_law: qp        # was: lqr
    # the qp.* parameters below are now the active ones
    qp.task_weight: [1.0, 10.0]
    qp.kp: 50.0
    qp.kd: 10.0
    qp.tau_max: 20.0
```

Rebuild the bringup package (it installs the config) and relaunch:

```bash
colcon build --packages-select kontrolem_bringup
source install/setup.bash
ros2 launch kontrolem_bringup cart_pole.launch.py
```

> **Note:** not every law suits every robot. The underactuated cart-pole is stabilized by `lqr`/`lqg`/`mpc`, but the `qp` task-space law is meant for a **fully-actuated** robot (try it on `arm2`). `wbc` requires a floating base. If a law can't accept the configured problem, the controller logs an error at activation.

## The gains follow the law

Each law reads its own parameter namespace — `lqr.*`, `lqg.*`, `mpc.*`, `qp.*`, `wbc.*`. Switching `control_law` selects which namespace is active; the others are ignored. To adjust behaviour after switching, see [Tune a controller](tune-a-controller.md). Full parameter tables: [Reference → Controller parameters](../reference/controller-parameters.md).
