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

## In plain words

Some robots naturally fall over — think of balancing a broom on your hand.
Keeping them steady takes constant, split-second corrections; that job is
done by a piece of software called a **controller**. Kontrol'Em **designs
that controller for you** from a description of your robot, lets you **watch
it work** in your browser, and **runs it** on a real or simulated robot — no
control-theory PhD required to get started.

```{admonition} New here? Follow this path
:class: tip

1. **{doc}`The big ideas, in plain words <concepts>`** — 10 minutes, no math.
2. **{doc}`Tutorials <tutorials/index>`** — install it and make a robot
   balance itself, step by step.
3. **Package guides & reference** (below) — the deep detail on each tool.
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
                         └───────────────────────┬───────────────────────┘
                                                 │ <name>_ros2_control.yaml
                                                 ▼
                         ┌───────────────────────────────────────────────┐
                         │            kontrolem_controllers              │
                         │  chainable ros2_control plugins (C++): load    │
                         │  the exported gains, run LQR/LQG/H∞ realtime   │
                         │  on mock · Gazebo Fortress · Isaac Sim         │
                         └───────────────────────────────────────────────┘
```

Design happens offline in Python; deployment is a separate C++ runtime,
{doc}`kontrolem_controllers <runtime/index>`, that loads the exported bundle and
runs the controller under `ros2_control` — validated in **Gazebo Fortress** and
**Isaac Sim** on the same cart–double-inverted-pendulum.

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
| {doc}`kontrolem_controllers <runtime/index>` | **Runtime / deployment** (C++): chainable `ros2_control` plugins that load the exported LQR/LQG/H∞ bundle and run it realtime on mock, **Gazebo Fortress**, and Isaac Sim hardware. |

## Where to start

- Brand new to this? Read {doc}`the big ideas in plain words <concepts>`,
  then do the {doc}`tutorials <tutorials/index>`.
- Prefer a fast command-line tour? The {doc}`quickstart` runs the whole
  pipeline in 5 minutes.
- Want the design rationale? {doc}`architecture` explains the canonical
  trajectory format and the clock × sampler × renderer playback model.
- Building on top of it? {doc}`extending` shows how to add a controller,
  an excitation, or a renderer in one file each; the normative
  {doc}`trajectory_format` spec is what any new producer/consumer targets.
- Ready to deploy on a robot? {doc}`runtime/index` covers the C++
  `ros2_control` runtime that runs the exported controller, with physics
  validation in {doc}`Gazebo <runtime/gazebo>` and {doc}`Isaac <runtime/isaac>`.
- Looking for a specific function? See the {doc}`api/index`.

```{toctree}
:hidden:
:maxdepth: 2
:caption: Start here

concepts
tutorials/index
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
:caption: Runtime & deployment

runtime/index
```

```{toctree}
:hidden:
:maxdepth: 2
:caption: Reference

api/index
```
