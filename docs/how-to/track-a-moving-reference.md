# How to follow a moving reference (Tracking)

> **For:** a user who has a controller running. **Assumes:** a built, sourced workspace. **Goal:** make the controller follow a time-varying reference instead of holding a fixed setpoint.

Kontrol'Em poses a control problem in one of two *dialects*: **Regulation** (drive to a fixed setpoint) or **Tracking** (follow a reference that changes with time). You select it with the `reference_type` parameter.

## Try the ready-made demo

```bash
ros2 launch kontrolem_bringup cart_pole_tracking.launch.py      # LQR follows a moving cart target
ros2 launch kontrolem_bringup cart_pole_mpc_tracking.launch.py  # MPC follows it, using the future reference
```

Watch `cart_joint` oscillate by ~±0.3 m while `pole_joint` stays near 0:

```bash
ros2 topic echo /joint_states
```

## Configure tracking yourself

Set `reference_type: harmonic` and describe the reference. The built-in harmonic source produces, per coordinate `i`:

```
q_i(t) = center_i + amp_i * cos(omega * t + phase_i)
```

Example config (a cart-pole whose cart follows `0.3·cos(0.5 t)` while the pole stays upright):

```yaml
kontrolem_controller:
  ros__parameters:
    control_law: lqr
    reference_type: harmonic
    reference.center: [0.0, 0.0]   # [cart, pole]  — nq entries
    reference.amp:    [0.3, 0.0]   # move the cart, hold the pole
    reference.phase:  [0.0, 0.0]
    reference.omega:  0.5          # rad/s
```

Set `amp = 0` on any coordinate you want held fixed. The number of entries is `nq` (the configuration size).

## Which laws accept Tracking?

`lqr` and `mpc` accept both Regulation and Tracking. `mpc` benefits most, because it uses the *future* reference over its horizon and so tracks more tightly than feedback-only LQR. See [Reference → Control laws](../reference/control-laws.md) for the accepted dialects of each law.

> A moving reference must be physically feasible for the robot to track it exactly. On an underactuated robot (like the cart-pole) a reference such as "cart moves *and* pole exactly upright" is not dynamically achievable, so expect a small residual error. This trade-off is discussed in [Explanation → Problem specs & dialects](../explanation/problem-specs.md).
