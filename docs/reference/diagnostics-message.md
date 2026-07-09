# Reference — ControllerDiagnostics message

> **For:** developers. **Assumes:** ROS 2 message basics. **Scope:** the schema of `kontrolem_msgs/msg/ControllerDiagnostics`, published by the runtime on `~/diagnostics` when `publish_diagnostics: true`. To turn it on and read it, see [How-to → Read diagnostics](../how-to/read-diagnostics.md).

**Topic:** `/<controller_name>/diagnostics` (e.g. `/kontrolem_controller/diagnostics`)
**Type:** `kontrolem_msgs/msg/ControllerDiagnostics`
**Rate:** one message per control tick (best-effort; skipped if a subscriber holds the realtime lock).

## Fields

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | Timestamp of the tick. |
| `control_law` | `string` | Which law produced this tick (`lqr` \| `lqg` \| `qp` \| `mpc` \| `wbc`). |
| `q` | `float64[]` | Configuration acted on (model joint order; length `nq`, includes the SE(3) root for a floating base). |
| `v` | `float64[]` | Velocity acted on (length `nv`; zero-filled for an output-feedback law that estimates it). |
| `q_ref` | `float64[]` | Reference configuration (setpoint, or the trajectory sampled at this time). |
| `tau` | `float64[]` | Command applied to the actuated joints this tick. |
| `ok` | `bool` | The control law's `status().ok`. |
| `margin` | `float64` | The control law's `status().margin` (distance to the trust boundary; meaning is per-law). |
| `safe_action` | `bool` | `true` if the supervisor overrode the command with the safe action. |
| `update_us` | `float64` | Wall-clock duration of `update()` this tick, in microseconds. |
| `solver_iters` | `int32` | QP iterations this tick (`0` for closed-form laws like LQR/LQG). The real-time margin: compare against the law's `max_iter` cap. |

## Notes

- The message is published via a realtime-safe publisher (non-blocking `trylock`); if a reader is slow, ticks are dropped rather than blocking the control loop.
- `margin` semantics per law are listed in [Reference → Control laws](control-laws.md).
- Publishing is opt-in (`publish_diagnostics`), off by default, to keep the loop allocation- and I/O-free unless you ask for telemetry.
