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
- The OSQP seam is configured for the control loop: no polish, fixed rho, warm-started re-solves — so a per-tick solve avoids re-factorizing the KKT system.
- Telemetry uses a non-blocking realtime publisher (`trylock`); if a reader is slow, ticks are dropped rather than blocking the loop.

## The honest QP real-time caveat

The allocation and iteration behaviour of an ADMM QP solver (OSQP) under a hard transient is **data-dependent**, and this is the framework's most honest real-time caveat. Two concrete findings from the whole-body controller:

- Under a stiff transient (a push), a QP solved to a very tight tolerance (1e-6) can return `SOLVED_INACCURATE` on a handful of ticks — the solver didn't fully converge in the tick. The fix was to use a tolerance appropriate to the physical scale (1e-4 — a whole-body controller solves for contact *forces in newtons*, not to a micronewton). At that tolerance the transient ticks converge cleanly. When they don't, `status().ok` going false *is the correct, honest signal* for the supervisor to fall back on, not something to hide.
- The whole-body `compute()` still has one known allocation (forming the SE(3) posture error), noted as a follow-up. So the hard-real-time claim on the QP path is *designed for*, not *yet certified* — and the docs say so rather than overclaiming.

This is the general stance: firm-real-time is demonstrated; hard-real-time is architecturally reachable but honestly not certified on the optimization paths. See [Design decisions](design-decisions.md) for the full register of what's proven versus deferred.
