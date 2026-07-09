# Explanation — Safety, the supervisor & real-time

> **For:** developers. **Assumes:** you've read [The controller contract](controller-contract.md). Understanding-oriented — to *read* the trust signals see [How-to → Read diagnostics](../how-to/read-diagnostics.md).

## The supervisor and the trust predicate

Every controller answers one question each tick beyond "what's the command?": **"can I be trusted this tick?"** — exposed as `status().ok` and a `status().margin`. The runtime's supervisor consumes this uniformly: if `ok`, it applies the command; if not, it applies a **safe action** (currently: zero effort) and flags it in telemetry.

This is the generalization of the old "is the linearization still valid?" guard into something every paradigm shares. The key design choice is that the supervisor unifies the **interface**, not the **semantics**:

| Paradigm | What "trustworthy" means |
|---|---|
| LQR | The state is inside the linearization's validity region. |
| LQG | The filter's innovation is inside a statistical gate (the estimate isn't diverging). |
| MPC / QP | The optimization solved within its budget. |
| WBC | The QP solved *and* the feet have positive friction-cone margin. |

Forcing these into one metric would be dishonest — a linearization region and a friction cone are different physics. So `margin` is a per-law number with a per-law meaning, and only its sign ("≥ 0 means trustworthy") is common. That is enough for the supervisor to act, and it keeps each paradigm's safety logic where the physics is.

## Why a Q-weighted trust region (an example of getting a metric right)

LQR's trust region was originally a box on `|q − q_eq|`. That was subtly wrong: it counted a translation-invariant coordinate (the cart's position) as "leaving the valid region," even though the linearization doesn't care where the cart is. The fix is a **Q-weighted distance** `√((x−x_eq)ᵀ Q (x−x_eq) / trace Q)`, which automatically down-weights coordinates the cost doesn't care about. The lesson generalizes: a trust metric should measure distance in the geometry the controller actually uses, not raw coordinates.

## Real-time posture: firm now, hard-ready

Kontrol'Em targets **firm** real-time today (~200 Hz–1 kHz in-process) with a design that a hard-real-time implementation can drop into later without redesign:

- `compute()` is **allocation-free** on the paths where it matters. Controllers preallocate every buffer in `configure()`; the model exposes a `Workspace` (a preallocated Pinocchio `Data`) so per-tick dynamics queries don't allocate; the LQR/QP/MPC paths are audited with malloc-level hooks.
- The OSQP seam is configured for the control loop: no polish, fixed rho, warm-started re-solves — so a per-tick solve avoids re-factorizing the KKT system — **and every QP path caps `max_iter`**, so the data-dependent iteration count can no longer blow the tick budget (see below).
- Telemetry uses a non-blocking realtime publisher (`trylock`); if a reader is slow, ticks are dropped rather than blocking the loop.

## The honest QP real-time caveat

The allocation and iteration behaviour of an ADMM QP solver (OSQP) under a hard transient is **data-dependent**, and this is the framework's most honest real-time caveat. Two concrete findings from the whole-body controller:

- Under a stiff transient (a push), a QP solved to a very tight tolerance (1e-6) can return `SOLVED_INACCURATE` on a handful of ticks — the solver didn't fully converge in the tick. The fix was to use a tolerance appropriate to the physical scale (1e-4 — a whole-body controller solves for contact *forces in newtons*, not to a micronewton). At that tolerance the transient ticks converge cleanly. When they don't, `status().ok` going false *is the correct, honest signal* for the supervisor to fall back on, not something to hide.
- Getting the whole-body `compute()` fully allocation-free took some hunting: the obvious culprit (forming the SE(3) posture error) was one, but the subtler ones were a per-tick frame *name lookup* (`getFrameId` allocates in this Pinocchio build → cache the frame indices at configure time) and the `LOCAL_WORLD_ALIGNED` frame kinematics (which allocate a temporary → compute in the local frame and rotate to world manually). A malloc-level audit over 1000 ticks now confirms **zero allocations** on the WBC path, alongside LQR/QP/MPC.

So on the WBC path, allocation-free `compute()` is now *certified* by the audit. The other half of a hard-real-time bound is the **iteration count**, and every QP path now caps OSQP's `max_iter`. Because polish and adaptive-rho are off, each ADMM iteration does identical fixed work, so a `max_iter` cap is a genuine bound on per-tick solve time. Two things make it a *safety* bound rather than a straightjacket:

- The caps sit well above the nominal converged count (WBC ~50 vs cap 200; task-space QP ~175 vs 400; MPC ~625 vs 2000), and closed-loop tests assert that headroom — so the cap never trips during normal operation.
- When a solve *does* hit the cap (a stiff transient it can't resolve in the budget), OSQP returns `OSQP_MAX_ITER_REACHED`, `status().ok` goes false, and the supervisor falls back — the deadline is honoured and the miss is surfaced, not hidden.

Capping surfaced a real finding worth recording: the condensed MPC QP at a `1e-6` tolerance needs a *pathological* iteration count under fixed rho (the count scaled with whatever cap it was given — it never truly converged), for zero control benefit, since MPC applies only `u₀` and re-solves next tick. Loosening its tolerance to `1e-4` (the same scale-appropriate choice the WBC already made) gives identical control quality with a bounded ~625-iteration worst case. The iteration count each tick actually took is now reported in `status().iters` and flows to diagnostics, so the RT margin is observable at runtime. Firm real-time is demonstrated; the two ingredients of a hard-RT bound — allocation-free `compute()` and a bounded solve — are now both in place on the QP paths, leaving a PREEMPT_RT deployment + WCET measurement as the remaining engineering step. See [Design decisions](design-decisions.md) for the full register.
