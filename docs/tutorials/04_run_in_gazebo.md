# Tutorial 4 — Make it balance in simulation

**Goal:** run your controller on a **physics-simulated** robot in Gazebo,
watch it hold the pendulum up against gravity, and send it commands to steer
the cart.

**Time:** about 15 minutes.

**You'll need:** **Gazebo Fortress** (Ignition Gazebo 6) and the
`gz_ros2_control` package. On Ubuntu 22.04 / ROS 2 Humble:

```bash
sudo apt install ros-humble-ros-gz ros-humble-gz-ros2-control
```

```{note}
Why Gazebo now, when the web app already showed a response? The app's
animation is a *math prediction*. Gazebo is a *physics engine* — it applies
real gravity and contact forces and doesn't know what answer the controller
"expects." If the controller balances the robot here, that's a much stronger
proof it actually works.
```

## Step 1 — Launch the simulation

In a sourced workspace (`source install/setup.bash`):

```bash
ros2 launch kontrolem_gazebo gz_bringup.launch.py
```

This one command starts a lot: the Gazebo physics world, the robot dropped
into it, and — crucially — your **LQR controller running inside the
simulator**, already balancing. (Want the LQG or H∞ controller instead? Add
`controller:=lqg` or `controller:=hinf`.)

```{tip}
By default Gazebo runs **headless** (no window) so it's fast and works over
SSH. To actually watch it, add `gui:=true`:

    ros2 launch kontrolem_gazebo gz_bringup.launch.py gui:=true
```

## Step 2 — Understand what you're seeing (or *not* seeing)

Here's the thing that surprises people: **it looks like nothing is
happening.** The pendulum just stands there.

That stillness *is the controller working.* The robot starts balanced, and
the controller's entire job is to keep it balanced — so it makes hundreds of
tiny corrections a second that mostly cancel out, and the pendulum barely
moves. If you were to switch the controller off, the top link (which has no
motor of its own) would swing down and the whole thing would topple in about
two seconds.

To *prove* the controller is alive, look at the numbers. In a second sourced
terminal:

```bash
ros2 topic echo /joint_states
```

You'll see the joint positions streaming, all hovering near zero — being
actively held there.

## Step 3 — Send it a command

Your controller isn't just holding still; you can give it a **new target**
and it will move there while keeping the pendulum balanced.

The controller listens on a topic called `/lqr_controller/reference`. You
send it six numbers — the target position of each joint, then the target
speed of each joint:

```text
[ cart_position, joint1_angle, joint2_angle,  cart_speed, joint1_speed, joint2_speed ]
        ▲              ▲            ▲
   metres on rail   radians      radians    (leave the speeds at 0)
```

All zeros means "stay at the upright balance point" (the default). To ask
the cart to slide to **+0.15 m to the right** while keeping the pendulum up:

```bash
ros2 topic pub -1 /lqr_controller/reference std_msgs/msg/Float64MultiArray \
    "{data: [0.15, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

Watch (in the GUI, or in the `/joint_states` echo): the cart glides to the
new spot, the pendulum sways and then re-settles upright over it. Send
`0.0` back to bring it home:

```bash
ros2 topic pub -1 /lqr_controller/reference std_msgs/msg/Float64MultiArray \
    "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

```{note}
The `-1` means "publish once and exit." Drop it to keep publishing (hold a
target continuously). If you send the wrong number of values, the controller
ignores the message and logs a warning — it won't do anything dangerous.
```

## Step 4 — Give it a shove (GUI only)

If you launched with `gui:=true`, use Gazebo's **Apply Force/Torque** tool to
nudge a link, then watch the cart dart underneath to catch it — the same
recovery you saw predicted in {doc}`Tutorial 3 <03_reading_response>`, now in
real physics.

## How this fits together

The picture behind that one launch command:

```text
Gazebo physics  ──(joint positions & speeds)──▶  your LQR controller
     ▲                                                   │
     └───────────────(push the cart)────────────────────┘
                    ↑
     your reference command sets the target this loop aims for
```

The controller manager runs **inside** the Gazebo process, so there's no
separate program to start — the single launch file wires up physics, robot,
and controller together. The exact same controller file from
{doc}`Tutorial 2 <02_first_controller>` drives it; nothing was re-tuned for
simulation.

## Troubleshooting

```{list-table}
:header-rows: 1
:widths: 45 55

* - Symptom
  - Fix
* - `Failed to load system plugin [gz_ros2_control-system]`
  - `gz_ros2_control` isn't installed or found. Install `ros-humble-gz-ros2-control` and re-source. (The launch file already points Gazebo at the plugin folder, so no manual environment setup is needed.)
* - `ign: command not found`
  - Gazebo Fortress isn't installed. Install `ros-humble-ros-gz`. On some systems the binary is `gz sim` instead of `ign gazebo`.
* - The pendulum falls over immediately
  - The controller didn't activate. Check the launch output for `loaded lqr artifact` and `activated lqr_controller`.
* - `ros2 topic pub` seems to do nothing
  - Confirm the topic name with `ros2 topic list | grep reference`, and that you sent exactly six numbers.
```

For the deeper mechanics (why the controller lives inside Gazebo, the
plugin-path and logging gotchas), see the {doc}`Gazebo reference page
<../runtime/gazebo>`.

---

**Next:** {doc}`Tutorial 5 — Use your own robot <05_own_robot>`.
