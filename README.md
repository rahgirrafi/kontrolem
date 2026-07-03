# Kontrol'Em — State-Space Control for ROS 2

[![CI](https://github.com/rahgirrafi/kontrolem/actions/workflows/ci.yml/badge.svg)](https://github.com/rahgirrafi/kontrolem/actions/workflows/ci.yml)
[![docs](https://github.com/rahgirrafi/kontrolem/actions/workflows/docs.yml/badge.svg)](https://rahgirrafi.github.io/kontrolem/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**Documentation:** <https://rahgirrafi.github.io/kontrolem/> —
quickstart, architecture, the `RobotTrajectory` format spec, per-package
guides, and the API reference (built from [`docs/`](docs/) by
[`docs.yml`](.github/workflows/docs.yml)).

**Kontrol'Em** is an end-to-end, ROS 2-native workflow for **linear
state-space control design** on robots described by a URDF — from rigid-body
model to synthesized controller to animated closed-loop response. It is, in
spirit, a *MoveIt Setup Assistant for state-space control*: MoveIt targets
motion planning; Kontrol'Em targets controller synthesis (LQR, LQG, H∞, PID)
on plants linearized directly from the robot's own URDF.

> **Status:** research software accompanying an undergraduate thesis.
> Simulation-validated on the included examples; see


## The pipeline

```
 ┌─────────────────────┐   (A,B,C,D)   ┌────────────────────┐   K / K(s)   ┌──────────────────────┐
 │  urdf_state_space   │──────────────▶│ state_space_control│─────────────▶│ state_space_         │
 │  URDF → LTI plant   │   u_eq, q_eq  │  LQR·LQG·H∞·PID    │  controller  │ response_viz         │
 │  (Pinocchio,        │               │  plugin registry   │              │ RViz playback of the │
 │   analytic Jacobians)│              └────────────────────┘              │ closed-loop response │
 └─────────────────────┘                        ▲                          └──────────────────────┘
            │                                    │                                     ▲
            │            ┌───────────────────────┴───────────────────────┐            │
            └───────────▶│         state_space_setup_assistant            │────────────┘
                         │  web wizard: load → validate → operating point │  RobotTrajectory
                         │  → linearize → design → benchmark → export     │  (.npz interchange)
                         └────────────────────────────────────────────────┘
```

Every module speaks one **canonical interchange format**, `RobotTrajectory`
(an `.npz` schema): producers write it (the linear closed-loop simulation
today; nonlinear simulators, MuJoCo, rosbag/real-robot logs later), consumers
read it (RViz playback, the wizard's response step, benchmark playback). Adding
a producer or consumer touches no existing code.

## Packages

| Package | Role |
|---|---|
| [`urdf_state_space`](src/urdf_state_space/README.md) | URDF → linear state-space plant `(A,B,C,D)` by **analytic** linearization of the manipulator equation (Pinocchio `computeABADerivatives`), with gravity-compensation `u_eq`, actuation selection, and exact ZOH discretization. |
| [`state_space_control`](src/state_space_control/README.md) | Controller-synthesis toolbox: **LQR, LQG, H∞** (mixed-sensitivity and general), and **PID**, behind a `@register` plugin registry. New controllers appear automatically in the CLI, YAML specs, and the wizard. |
| [`state_space_setup_assistant`](src/state_space_setup_assistant/README.md) | MoveIt-Setup-Assistant-style **web wizard** (Flask + vendored three.js): load → validate → operating point (automatic equilibrium finder) → linearize → design → benchmark → export a reproducible config bundle. |
| [`state_space_response_viz`](src/state_space_response_viz/README.md) | Source-agnostic **RViz playback** of `RobotTrajectory` files, with play/pause/seek/speed transport control. |
| [`kontrolem_example_robots`](src/kontrolem_example_robots/README.md) | The framework's running **example** URDF: a cart with a double pendulum (underactuated, 3 DoF / 2 actuators), upright at `q=0`. |
| `rws_description`, `rws_core` | The thesis application robot — a Remote Weapon Station turret (azimuth/elevation gimbal) — and its joystick/PID teleoperation node. |

## Installation

Requires **ROS 2 Humble** (or newer) and Python ≥ 3.10.

```bash
# 1. System / ROS 2: install ROS 2 Humble, then create a workspace
mkdir -p ~/ws_rws/src && cd ~/ws_rws
git clone https://github.com/rahgirrafi/kontrolem.git src   # or copy this src/ tree

# 2. Python dependencies
pip install pin            # Pinocchio dynamics backend (NOT "pip install pinocchio")
pip install numpy scipy pyyaml flask
pip install control slycot # required only for the H∞ controllers

# 3. Build
cd ~/ws_rws
colcon build
source install/setup.bash
```

## Quick start (end to end)

```bash
# a) URDF  ->  linear state-space plant
ros2 run urdf_state_space urdf2ss \
    src/urdf_state_space/examples/example_model.yaml     # writes model.npz / model.mat

# b) plant  ->  LQR controller
ros2 run state_space_control ss_design \
    model.npz src/state_space_control/examples/lqr_design.yaml -o controller.npz

# c) or do all of it interactively in the browser wizard
ros2 run state_space_setup_assistant ss_setup_assistant \
    --urdf package://kontrolem_example_robots/urdf/cart_double_inverted_pendulum.urdf
# then open http://127.0.0.1:8060/

# d) animate the closed-loop response in RViz
ros2 launch state_space_response_viz view_response.launch.py \
    trajectory:=trajectory.npz \
    urdf:=install/kontrolem_example_robots/share/kontrolem_example_robots/urdf/cart_double_inverted_pendulum.urdf \
    fixed_frame:=world
```

From Python:

```python
from urdf_state_space import build_from_yaml
from state_space_control import Plant, make_controller

ss, _ = build_from_yaml('example_model.yaml')      # URDF -> (A,B,C,D)
plant = Plant.from_model(ss)
result = make_controller('lqr', Q=[1, 10, 10, 1, 1, 1], R=0.1).design(plant)
print(result.summary())                            # gains, closed-loop poles, stability
```

## Tests

Each Python package ships a `pytest` suite that runs headlessly (no ROS graph,
no display):

```bash
# one package
cd src/state_space_control && python -m pytest test/ -v

# whole workspace after a colcon build
colcon test && colcon test-result --verbose
```

## Extending it

Add a controller in one file — it is then usable from the Python API, the YAML
specs, the CLI, and the wizard, with no other edits:

```python
from state_space_control.base import ControllerDesign, ControllerResult, register

@register('my_ctrl')
class MyController(ControllerDesign):
    def design(self, plant):
        K = ...                      # your synthesis
        return ControllerResult(name='my_ctrl', plant=plant, K=K)
```

See [`CONTRIBUTING.md`](CONTRIBUTING.md) for the full guide (plugins, the
trajectory format, running tests, and how to file issues/PRs).

<!-- ## Citing

If you use this framework in academic work, please cite it — see
[`CITATION.cff`](CITATION.cff) and, once published, the JOSS paper
(`paper/paper.md`). -->

## License

[MIT](LICENSE) © 2026 Rahgir Rafi.
