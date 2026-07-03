# Kontrol'Em — state-space control for ROS 2

**Kontrol'Em** is an end-to-end, ROS 2-native workflow for **linear
state-space control design** on robots described by a URDF — from
rigid-body model to synthesized controller to animated closed-loop
response. It is, in spirit, a *MoveIt Setup Assistant for state-space
control*: MoveIt targets motion planning; Kontrol'Em targets controller
synthesis (LQR, LQG, H∞, PID) on plants linearized directly from the
robot's own URDF.

```{note}
Research software accompanying an undergraduate thesis.
Simulation-validated on the included examples.
Source: [github.com/rahgirrafi/kontrolem](https://github.com/rahgirrafi/kontrolem)
· License: MIT.
```

## The pipeline

```text
 ┌─────────────────────┐   (A,B,C,D)   ┌────────────────────┐   K / K(s)   ┌──────────────────────┐
 │  urdf_state_space   │──────────────▶│ state_space_control│─────────────▶│ state_space_         │
 │  URDF → LTI plant   │   u_eq, q_eq  │  LQR·LQG·H∞·PID    │  controller  │ response_viz         │
 │  (Pinocchio,        │               │  plugin registry   │              │ RViz playback of the │
 │  analytic Jacobians)│               └────────────────────┘              │ closed-loop response │
 └─────────────────────┘                        ▲                          └──────────────────────┘
            │                                   │                                      ▲
            │            ┌──────────────────────┴────────────────────────┐             │
            └───────────▶│         state_space_setup_assistant           │─────────────┘
                         │  web wizard: load → validate → operating point│  RobotTrajectory
                         │  → linearize → design → benchmark → export    │  (.npz interchange)
                         └───────────────────────────────────────────────┘
```

Every module speaks one **canonical interchange format**,
[`RobotTrajectory`](trajectory_format.md): producers write it (the linear
closed-loop simulation today; nonlinear simulators, MuJoCo, rosbag /
real-robot logs later), consumers read it (RViz playback, the wizard's
response step, benchmark playback). Adding a producer or a consumer touches
no existing code — see {doc}`architecture`.

## Packages

| Package | Role |
|---|---|
| {doc}`urdf_state_space <guides/urdf_state_space>` | URDF → linear state-space plant `(A,B,C,D)` by **analytic** linearization (Pinocchio `computeABADerivatives`), with gravity-compensation `u_eq`, actuation selection, exact ZOH discretization. |
| {doc}`state_space_control <guides/state_space_control>` | Controller-synthesis toolbox: **LQR, LQG, H∞, PID** behind a `@register` plugin registry; also home of the canonical trajectory format, the excitation registry, and the closed-loop simulator. |
| {doc}`state_space_setup_assistant <guides/setup_assistant>` | MoveIt-Setup-Assistant-style **web wizard**: load → validate → operating point → linearize → design → response → benchmark → export. |
| {doc}`state_space_response_viz <guides/response_viz>` | Source-agnostic **RViz playback** of `RobotTrajectory` files with play/pause/seek/speed transport control. |
| {doc}`kontrolem_example_robots <guides/example_robots>` | Example URDFs — the cart double inverted pendulum used throughout these docs. |

## Where to start

- New to the framework? Read the {doc}`quickstart` (5 minutes, end to end).
- Want the design rationale? {doc}`architecture` explains the canonical
  trajectory format and the clock × sampler × renderer playback model.
- Building on top of it? {doc}`extending` shows how to add a controller,
  an excitation, or a renderer in one file each; the normative
  {doc}`trajectory_format` spec is what any new producer/consumer targets.
- Looking for a specific function? See the {doc}`api/index`.

```{toctree}
:hidden:
:maxdepth: 2
:caption: Getting started

quickstart
```

```{toctree}
:hidden:
:maxdepth: 2
:caption: Concepts

architecture
trajectory_format
extending
```

```{toctree}
:hidden:
:maxdepth: 2
:caption: Package guides

guides/urdf_state_space
guides/state_space_control
guides/setup_assistant
guides/response_viz
guides/example_robots
```

```{toctree}
:hidden:
:maxdepth: 2
:caption: Reference

api/index
```
