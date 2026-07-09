# Explanation — The controller contract & lifecycle

> **For:** developers. **Assumes:** you've read [Architecture](architecture.md). Understanding-oriented — for the exact signatures see [Reference → Core API](../reference/core-api.md).

## The one call shape

Every controller — a stored LQR gain, a Kalman compensator, an MPC, a whole-body QP — implements the same lifecycle:

```
capabilities()  →  synthesize()  →  configure()  →  compute()  →  status()
```

The claim the whole framework rests on: this shape fits both a precomputed static-gain controller and a structurally different online-solved controller. `compute(state, problem, dt) → command` is a matrix-vector product for LQR and an optimization solve for the WBC, but from the runtime's side they are identical: consume a state and a problem, return a command, report health. If that holds, one plugin boundary can host every paradigm — and it does.

## Why the offline/online split is *not* two subsystems

The tempting mistake is to build two frameworks: an "offline linear" one (synthesize a gain, deploy a simple runtime) and an "online optimization" one (model service + solver + runtime). That doubles the runtime and forfeits the single plugin boundary.

But the thing that *seems* to force the split — heavy offline synthesis vs. per-tick solving — is not a runtime split at all. It's a **temporal** difference in *preparation*, captured cleanly by the multi-phase lifecycle:

- **`synthesize()`** — offline/occasional heavy math (LQR's Riccati solve, LQG's two Riccati solves). Produces a serializable `Synthesis` artifact. May run anywhere, even a design tool. For an online controller (QP, WBC) it's a no-op.
- **`configure()`** — at load, on the deployed instance: allocate solver workspaces, discretize, set up warm-start buffers. Not real-time. This is where MPC builds its condensed matrices.
- **`compute()`** — per tick, real-time, allocation-free.

So LQR's Riccati and MPC's horizon setup both live in the heavy phase; the difference between paradigms is *what each phase does*, not *which API they use*. One framework, one boundary, no doubled runtime. The seam is real but internal to the lifecycle — temporal, not paradigmatic.

## `capabilities()` — negotiation before the loop

Before `configure()`/`compute()`, a controller declares what it accepts (which problem dialects) and what state it needs (does it require velocity, or estimate it?). The runtime checks this once, at wiring time. Two consequences:

- A mismatch (e.g. asking a Regulation-only controller to track) is rejected at activation with a clear error, not discovered mid-loop.
- The per-tick dialect narrowing inside `compute()` becomes a bare `static_cast` — the check already happened.

`needs_velocity_state` is a concrete payoff: LQG declares it `false`, so the runtime claims **only position interfaces** and LQG estimates velocity internally. The same runtime claims position+velocity for LQR. Neither controller knows about ros2_control; the capability flag drives the interface wiring.

## `status()` — health without unifying safety

`compute()` is paired with `status()`: the controller's own verdict on whether this tick's output is trustworthy, as an `ok` flag and a `margin`. The runtime's supervisor consumes it uniformly, but the *meaning* is paradigm-specific — a linearization region for LQR, a friction-cone margin for the WBC. The interface is unified; the semantics are not, deliberately. See [Safety, the supervisor & real-time](safety-and-realtime.md).

## The small tax on the trivial case

There is a cost: the simplest possible controller (a setpoint regulator) has to express its setpoint as a degenerate `Regulation` problem and implement the full lifecycle even though `synthesize()` is where all its work is. That ceremony is accepted and kept tiny — the payoff is that the trivial case and the whole-body case are genuinely the same object, so tooling, telemetry, and the supervisor are written once.

Continue with [Problem specs & dialects](problem-specs.md) — the other half of `compute()`'s signature.
