# Tutorial 1 — Set up and see the robot

**Goal:** install Kontrol'Em and watch the example robot — the cart with a
double pendulum — appear and move in your web browser.

**Time:** about 15 minutes (most of it is the build).

**You'll need:** a computer running **ROS 2 Humble** (Ubuntu 22.04 is the
easy path) and **Python 3.10 or newer**. If you don't have ROS 2 yet, follow
the official [ROS 2 Humble install guide](https://docs.ros.org/en/humble/Installation.html)
first, then come back.

```{tip}
Never used a terminal much? Every grey box below is a command: copy it,
paste it into your terminal, press Enter. Do them in order.
```

## Step 1 — Get the code

```bash
mkdir -p ~/ws_kontrolem && cd ~/ws_kontrolem
git clone --recurse-submodules https://github.com/rahgirrafi/kontrolem.git src
```

`--recurse-submodules` matters: the individual tools live in sub-repositories,
and this pulls them all in one go.

## Step 2 — Install the Python dependencies

```bash
pip install pin numpy scipy pyyaml flask
pip install control slycot   # only needed for the H∞ controller — safe to skip for now
```

```{warning}
The dynamics engine is installed with **`pip install pin`**, *not*
`pip install pinocchio`. The name `pinocchio` on PyPI is a different,
unrelated package. This trips everyone up once.
```

## Step 3 — Build the workspace

```bash
cd ~/ws_kontrolem
colcon build --symlink-install
source install/setup.bash
```

`colcon build` compiles everything; `source install/setup.bash` tells your
terminal where to find the freshly built tools.

```{note}
You must run `source install/setup.bash` **once in every new terminal**
before using Kontrol'Em. If a command later says "package not found," this
is almost always the reason.
```

## Step 4 — Open the web app and meet the robot

```bash
ros2 run state_space_setup_assistant ss_setup_assistant \
    --urdf package://kontrolem_example_robots/urdf/cart_double_inverted_pendulum.urdf
```

A browser tab should open at <http://127.0.0.1:8060/>. If it doesn't, open
that address yourself.

You'll see the **Setup Assistant** — a step-by-step web app (in spirit like
the MoveIt Setup Assistant, if you've met that). On the first screen a 3D
model of the robot appears: the cart on its rail, with the two pendulum
links standing straight up.

**Try this:** drag in the 3D view to orbit the camera. Later screens have
sliders that move the joints and the model responds live — that's your
confirmation that everything installed correctly.

```{tip}
Keep **one browser tab** open at a time. The app keeps a single working
session, so a second tab will fight the first.
```

## What you just did

You installed the whole framework and confirmed it runs by loading a robot
into the design app. Nothing is *controlled* yet — the robot is just sitting
at its (tippy) balance point.

## Troubleshooting

```{list-table}
:header-rows: 1
:widths: 45 55

* - Symptom
  - Fix
* - `ros2: command not found`
  - You haven't sourced ROS 2. Run `source /opt/ros/humble/setup.bash`, then `source install/setup.bash`.
* - `package 'state_space_setup_assistant' not found`
  - Run `source install/setup.bash` from `~/ws_kontrolem` in this terminal.
* - Browser opens but the robot is blank / untextured
  - Meshes load from the built workspace — make sure step 3 finished and you sourced the workspace. Plain frames (no textures) still work fine.
* - `ModuleNotFoundError: pinocchio`
  - You installed the wrong package. Run `pip uninstall pinocchio` then `pip install pin`.
```

---

**Next:** {doc}`Tutorial 2 — Design your first controller <02_first_controller>`,
where you'll make this wobbly robot stable.
