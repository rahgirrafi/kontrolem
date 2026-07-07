# Tutorial 2 — Design your first controller

**Goal:** using only the web app (no code), turn the toppling pendulum into
one that balances itself, and save the result to a file.

**Time:** about 15 minutes.

**Before you start:** finish {doc}`Tutorial 1 <01_setup>` — you should have
the Setup Assistant open in your browser at <http://127.0.0.1:8060/>.

```{note}
Reminder of what we're doing (see {doc}`../concepts`): we'll describe the
robot, pick the pose to balance at, let the app compute a **decide-rule**
(an LQR controller), and export it. "Design once, run forever."
```

The app is a wizard with **eight steps** along the top. We'll walk through
them. You can always click a previous step to go back.

## Step 1 · Robot & validation

The robot is already loaded. The app has automatically checked it for
control use — sensible masses, valid inertias, no broken joints. Green means
good. If you ever load your own robot and see red errors here, they must be
fixed before continuing (yellow warnings are fine to pass).

Click **Next**.

## Step 2 · Joints & operating point

This is where you say *what to balance* and *what the robot can push with*.

- **Actuated joints** — tick the joints the robot can drive. For our robot,
  tick only `cart_joint`; leave `joint1` and `joint2` unticked (they're
  free-swinging links, like the two segments of a real double pendulum). The
  single cart force has to balance the whole pole through them — that's what
  makes this the classic hard balancing problem.
- **Operating point** — the pose to hold. The sliders are already at
  "straight up," which is what we want. Watch the 3D model as you nudge a
  slider, then set it back to zero.
- Click **Auto-compute equilibrium**. This asks the app to double-check that
  "straight up" is a genuine balance point that needs no holding torque on
  the passive joints. For this robot it is.

Click **Next**.

## Step 3 · Outputs

"Outputs" are the things the controller is allowed to *measure*. For your
first controller, include the joint **positions** (this is the default).
Later, LQG lets you deliberately measure *less* and have the controller
estimate the rest — but not yet.

Click **Next**.

## Step 4 · Linearize

Press the **Linearize** button. The app converts the robot's physics into
the simple, predictable form that works near the balance point (the "linear
model" from {doc}`../concepts`).

You'll see some readouts. The one worth noticing: the robot has **poles in
the right half-plane** — that's the mathematician's way of saying *"this
thing is unstable and will fall over."* Good. That's the problem we're about
to solve.

Click **Next**.

## Step 5 · Controller

Choose **LQR** from the list and press **Design**.

In under a second the app computes the decide-rule. The readout now shows the
poles have moved to the **left half-plane** — the closed loop is **stable**.
In plain terms: *with this controller running, the robot stays up.*

That's the whole magic moment. You just designed a stabilizing controller.

Click **Next**.

## Step 6 · Response

Press **Simulate**. Now *watch the robot recover*: the app gives the
pendulum a shove and animates what happens with your controller running. The
cart darts under the falling links and settles them back upright, while the
plots below trace the joint angles and the push effort over time.

We'll spend the whole of {doc}`Tutorial 3 <03_reading_response>` reading this
screen. For now, just enjoy that it *doesn't fall over*.

Click **Next**.

## Step 7 · Benchmark (optional)

Here you can design several controllers at once and compare them on the same
scorecard — settling time, effort used, robustness. Skip it for your first
pass, or design an LQG alongside LQR and see the numbers differ.

Click **Next**.

## Step 8 · Export

Press **Export**. The app writes a **bundle** — a folder containing your
controller and everything needed to reproduce it. Note where it saves.

The one file the *robot* will actually use is named like
`lqr_ros2_control.yaml`. Everything else in the bundle is for reproducing and
understanding the design. (Curious what's inside that file? See the
{doc}`export contract <../runtime/export_contract>` — but you don't need it
to continue.)

## What you just did

Starting from a robot that falls over in two seconds, you produced a
controller that holds it upright — and saved it to a file, without writing a
line of code. That file is the bridge to the rest of the project: in
{doc}`Tutorial 4 <04_run_in_gazebo>` a simulated robot will *run* it.

## Doing the same thing from the command line (optional)

The web app is the friendly path, but every step has a command-line
equivalent, useful for scripting or automation:

```bash
# URDF → linear model
ros2 run urdf_state_space urdf2ss \
    src/packages/urdf_state_space/examples/example_model.yaml

# linear model → LQR controller
ros2 run state_space_control ss_design \
    model.npz src/packages/state_space_control/examples/lqr_design.yaml \
    -o controller.npz
```

---

**Next:** {doc}`Tutorial 3 — Read the response <03_reading_response>`.
