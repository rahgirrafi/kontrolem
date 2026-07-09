# How to read live diagnostics/telemetry

> **For:** a user or integrator debugging a running controller. **Assumes:** a built, sourced workspace. **Goal:** turn on and inspect the per-tick health of the controller.

The runtime can publish a `ControllerDiagnostics` message every tick: the state it acted on, the command it issued, its trust status, and how long the tick took. It's the fastest way to see whether a controller is healthy.

## Turn it on

Set the parameter in the controller's YAML:

```yaml
kontrolem_controller:
  ros__parameters:
    publish_diagnostics: true
```

(The bundled demos already enable this.) Rebuild the bringup package if you edited a config, then launch.

## Read it

```bash
ros2 topic echo /kontrolem_controller/diagnostics
```

Or grab a single message:

```bash
ros2 topic echo --once /kontrolem_controller/diagnostics
```

## What the fields mean

| Field | Meaning |
|---|---|
| `control_law` | which law produced this tick (`lqr`/`lqg`/`qp`/`mpc`/`wbc`) |
| `q`, `v` | the configuration and velocity the controller acted on (`v` is zero for LQG, which estimates it internally) |
| `q_ref` | the reference it was driving toward |
| `tau` | the command sent to the actuated joints |
| `ok` | `true` if the controller trusts this tick's output |
| `margin` | distance to the trust boundary; ≥ 0 means trustworthy (meaning is per-law) |
| `safe_action` | `true` if the supervisor overrode the command with the safe action |
| `update_us` | wall-clock microseconds the tick took |

The full schema is in [Reference → ControllerDiagnostics message](../reference/diagnostics-message.md).

## Useful checks

- **Is the controller actually running and healthy?** `ok: true` with a positive `margin`.
- **Is it about to fail safe?** `margin` trending toward 0; `safe_action` flipping to `true`.
- **Is it fast enough?** `update_us` should stay well under `1e6 / update_rate` (e.g. under 2000 µs at 500 Hz).

> `margin` is deliberately paradigm-specific (a linearization region for LQR, a friction-cone margin for WBC). Why it isn't one unified number is explained in [Explanation → Safety, supervisor & real-time](../explanation/safety-and-realtime.md).
