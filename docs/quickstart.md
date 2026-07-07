# Quickstart

From a bare ROS 2 machine to an animated closed-loop response in five
minutes, using the bundled example robot — a cart carrying a double
inverted pendulum, linearized about its unstable upright equilibrium.

```{admonition} This is the fast, command-line tour
:class: tip

It assumes you're comfortable in a terminal and just want the whole pipeline
in one page. If you'd rather be walked through it gently — with what each
step *means* — start with {doc}`the plain-language concepts <concepts>` and
the {doc}`step-by-step tutorials <tutorials/index>` instead.
```

## Install

Requires **ROS 2 Humble** (or newer) and Python ≥ 3.10.

```bash
# 1. Create a workspace and get the source
mkdir -p ~/ws_kontrolem/src && cd ~/ws_kontrolem
git clone --recurse-submodules https://github.com/rahgirrafi/kontrolem.git src

# 2. Python dependencies
pip install pin            # Pinocchio dynamics backend (NOT "pip install pinocchio")
pip install numpy scipy pyyaml flask
pip install control slycot # required only for the H∞ controllers

# 3. Build
colcon build --symlink-install
source install/setup.bash
```

## The five-minute tour

### a) URDF → linear plant

```bash
ros2 run urdf_state_space urdf2ss \
    src/packages/urdf_state_space/examples/example_model.yaml
```

This reads the example YAML — which names the URDF, the actuated joint
(`cart_joint`; `joint1` and `joint2` stay passive), per-joint damping, and the
operating point — linearizes analytically with Pinocchio, prints the
`(A, B, C, D)` matrices, poles, controllability/observability, and writes
`model.npz`. The upright pose is unstable: expect two poles in the right
half-plane.

### b) Plant → LQR controller

```bash
ros2 run state_space_control ss_design \
    model.npz src/packages/state_space_control/examples/lqr_design.yaml \
    -o controller.npz
```

Prints the gain `K`, the closed-loop poles (now all in the left half-plane)
and a stability verdict; writes `controller.npz`.

### c) Or do everything in the browser

```bash
ros2 run state_space_setup_assistant ss_setup_assistant \
    --urdf package://kontrolem_example_robots/urdf/cart_double_inverted_pendulum.urdf
```

Open <http://127.0.0.1:8060/> and walk the eight steps: the URDF is
validated for control use, the operating point is chosen on live 3D
sliders (with an automatic equilibrium finder), the model is linearized,
a controller designed, the closed-loop **response animated on the robot
model** in sync with the analysis plots, candidate controllers
benchmarked on one shared metric grid, and everything exported as a
reproducible bundle. See the {doc}`wizard guide <guides/setup_assistant>`.

### d) Replay the response in RViz

The wizard's export bundle contains `trajectory.npz` — a
{doc}`RobotTrajectory <trajectory_format>` file:

```bash
ros2 launch state_space_response_viz view_response.launch.py \
    trajectory:=trajectory.npz \
    urdf:=install/kontrolem_example_robots/share/kontrolem_example_robots/urdf/cart_double_inverted_pendulum.urdf \
    fixed_frame:=world
```

Transport control at runtime:

```bash
ros2 service call /response_player/pause std_srvs/srv/Trigger
ros2 param set /response_player speed 0.5
ros2 param set /response_player seek 1.25
```

## From Python

```python
from urdf_state_space import build_from_yaml
from state_space_control import Plant, make_controller

ss, _ = build_from_yaml('example_model.yaml')      # URDF -> (A,B,C,D)
plant = Plant.from_model(ss)
result = make_controller('lqr', Q=[1, 10, 10, 1, 1, 1], R=0.1).design(plant)
print(result.summary())                            # gains, poles, stability
```

And to simulate + record a response trajectory without the wizard:

```python
from state_space_control.excitations import make_excitation
from state_space_control.simulation import simulate_response, to_robot_trajectory

sim = simulate_response(result, make_excitation('impulse', area=1.0))
traj = to_robot_trajectory(ss, result, sim)        # absolute joint space
traj.save_npz('trajectory.npz')                    # playable in RViz / the wizard
```

## Running the tests

Every Python package ships a headless `pytest` suite (no ROS graph, no
display needed):

```bash
cd src/packages/state_space_control && python -m pytest test/ -v
# or, for the whole workspace after a colcon build:
colcon test && colcon test-result --verbose
```
