# Kontrol'Em v2 documentation

**Kontrol'Em is a robot-agnostic, plugin-based control framework for ROS 2 — "MoveIt for control."** One runtime (`KontrolemController`) hosts many control paradigms — a stored LQR gain, an online QP, an output-feedback compensator (LQG), a receding-horizon MPC, and a floating-base whole-body controller (WBC) — behind a single controller contract, on fixed-base *and* floating-base robots. The dynamics come from Pinocchio; the numerics are wrapped behind seams; Layers 1–3 have zero ROS dependency.

---

## Find your path in 10 seconds

**I want to *use* it** — run a controller, get a robot balancing/standing, tune it, wire it to my robot.
→ Start with the **[Tutorials](#tutorials)**, then dip into the **[How-to guides](#how-to-guides)** for specific tasks. You do not need to read the architecture.

**I want to *understand or extend* it** — add a controller, read the API, know why it's built this way.
→ Go to the **[Reference](#reference)** for exact facts and the **[Explanation](#explanation)** for the design rationale.

New here and not sure? Read **[Tutorial 1: balance a cart-pole](tutorials/first-run-cartpole.md)** — it takes you from an empty shell to a working controller in about 15 minutes.

---

## The four kinds of page (Diátaxis)

This documentation is deliberately split into four types. Each page tells you at the top **who it's for** and **what it assumes**. We never blend them: tutorials and how-tos contain no design rationale (it links out), reference contains no teaching, explanation contains no step-by-step.

| Type | Answers | Read it when you want to… |
|---|---|---|
| **Tutorial** | "Teach me by doing." | Learn the system from zero via a guaranteed happy path. |
| **How-to** | "How do I do X?" | Accomplish one specific task you already have in mind. |
| **Reference** | "What exactly is X?" | Look up a parameter, signature, interface, or field. |
| **Explanation** | "Why is it like this?" | Understand the architecture and the trade-offs. |

---

## Tutorials
Learning-oriented, zero prior knowledge, every step succeeds.

- **[Balance a cart-pole in 15 minutes](tutorials/first-run-cartpole.md)** — install, build, launch, watch it balance. Your first win.
- **[Stand a quadruped with a whole-body controller](tutorials/stand-a-quadruped.md)** — a guided second session that reaches the flagship result and shows the same runtime hosting a very different controller.

## How-to guides
Task-oriented recipes; assume you've done Tutorial 1.

- [Switch the control law (LQR ↔ QP ↔ LQG ↔ MPC)](how-to/switch-control-law.md)
- [Follow a moving reference (Tracking)](how-to/track-a-moving-reference.md)
- [Tune a controller's weights and limits](how-to/tune-a-controller.md)
- [Read live diagnostics/telemetry](how-to/read-diagnostics.md)
- [Run the framework on your own robot](how-to/run-on-your-robot.md)
- [Configure a floating-base whole-body controller](how-to/configure-whole-body-control.md)
- [Verify an integration (tests + e2e smoke)](how-to/verify-an-integration.md)
- [Fix the cmeel / libboost load error](how-to/fix-cmeel-library-errors.md)

## Reference
Dry, complete, for lookup.

- [Controller parameters](reference/controller-parameters.md)
- [Control laws](reference/control-laws.md)
- [Core C++ API (Layers 1–3)](reference/core-api.md)
- [ros2_control interface conventions](reference/ros2control-interfaces.md)
- [Demos & launch files](reference/demos.md)
- [ControllerDiagnostics message](reference/diagnostics-message.md)
- [Build & environment](reference/build-and-environment.md)

## Explanation
The why: architecture, decisions, trade-offs.

- [Vision & scope](explanation/vision-and-scope.md)
- [The four-layer architecture](explanation/architecture.md)
- [The controller contract & lifecycle](explanation/controller-contract.md)
- [Problem specs & dialects](explanation/problem-specs.md)
- [State on a manifold & non-joint data](explanation/state-and-non-joint-data.md)
- [Safety, the supervisor & real-time](explanation/safety-and-realtime.md)
- [Design decisions, trade-offs & what's not done](explanation/design-decisions.md)

---

*Status: the fixed-base framework (LQR/LQG/MPC/QP) and the floating-base standing WBC are implemented and validated end-to-end in simulation. Locomotion and hardware on a real quadruped are out of scope for this version — see [what's not done](explanation/design-decisions.md#whats-deliberately-not-done).*
