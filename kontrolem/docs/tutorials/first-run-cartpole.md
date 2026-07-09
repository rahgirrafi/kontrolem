# Tutorial 1 — Balance a cart-pole in 15 minutes

> **For:** a complete newcomer. **Assumes:** a Linux machine with ROS 2 Humble installed, and basic terminal use. **No** control-theory or framework knowledge is needed.

By the end of this page you will have built Kontrol'Em and watched it balance an inverted pendulum (a cart-pole) in a headless simulation — the control loop running end-to-end through ROS 2, no Gazebo. That's your first working result. We explain *nothing* about why it works here; when you're curious, follow the links at the end.

Every command below is meant to be run in order and to succeed. If one fails, jump to [Fix the cmeel / libboost load error](../how-to/fix-cmeel-library-errors.md) — that covers the one environment issue newcomers hit.

---

## Step 1 — Get the dependencies

You need three things: ROS 2 Humble, a colcon build environment, and Pinocchio (the rigid-body dynamics library) from its pip **cmeel** wheel.

```bash
# ROS 2 Humble is assumed already installed at /opt/ros/humble.
sudo apt install python3-colcon-common-extensions
pip3 install pin          # Pinocchio via the cmeel wheel
```

Check that the cmeel Pinocchio prefix exists — the build needs it:

```bash
ls $HOME/.local/lib/python3.10/site-packages/cmeel.prefix
```

You should see `lib/`, `include/`, etc. If that directory is missing, see [Build & environment](../reference/build-and-environment.md).

## Step 2 — Build the workspace

Go to the Kontrol'Em source directory (the folder that contains `kontrolem_model`, `kontrolem_bringup`, and the other package folders) and build:

```bash
source /opt/ros/humble/setup.bash
P=$HOME/.local/lib/python3.10/site-packages/cmeel.prefix   # cmeel Pinocchio prefix

cd <path-to>/ws_kontrolem/src/kontrolem
colcon build --cmake-args -DCMAKE_PREFIX_PATH="$P" -DCMAKE_BUILD_TYPE=Release
```

The first build takes a few minutes. It finishes with `Summary: N packages finished`. You may see a harmless CMake warning about Boost — ignore it.

## Step 3 — Launch the cart-pole

Source the workspace you just built, then launch the demo:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash          # from the same directory you built in
ros2 launch kontrolem_bringup cart_pole.launch.py
```

Leave this running. You do **not** need to set any library paths by hand — the launch file finds the cmeel libraries for you.

## Step 4 — Watch it balance

Open a **second terminal**, source the workspace again, and watch the joint states:

```bash
source /opt/ros/humble/setup.bash
cd <path-to>/ws_kontrolem/src/kontrolem && source install/setup.bash

ros2 topic echo /joint_states
```

Watch the `position` values. You will see:

1. For the first second or two, the pole is **held** at its start angle of about `0.15` rad (the simulator is deliberately frozen until the controller takes command).
2. Then `pole_joint` **settles smoothly to ~0** (upright) while `cart_joint` returns toward the centre.

That's it — the LQR controller is balancing the cart-pole through the full ROS 2 control loop. 🎉

To stop, press `Ctrl-C` in the launch terminal.

## Step 5 — Confirm what just happened

Ask the running system which controllers are active:

```bash
ros2 control list_controllers
```

You should see `kontrolem_controller … active`. That single controller is the runtime that hosts *every* Kontrol'Em control law — right now it's configured as LQR.

---

## You're done — where to next

You went from nothing to a balancing controller. Two natural next steps:

- **Keep learning by doing** → [Tutorial 2: stand a quadruped with a whole-body controller](stand-a-quadruped.md). Same runtime, a much bigger result.
- **Do a specific task** → e.g. [switch the control law](../how-to/switch-control-law.md) to see the *same* cart-pole balanced by a different kind of controller.

Curious *why* the simulator freezes at startup, or *why* one controller can host so many methods? That rationale lives in [Explanation → The controller contract & lifecycle](../explanation/controller-contract.md) — deliberately kept out of this tutorial.
