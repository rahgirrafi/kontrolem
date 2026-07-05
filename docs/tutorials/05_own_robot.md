# Tutorial 5 — Use your own robot

**Goal:** point the whole pipeline at a *different* robot — yours — instead
of the example pendulum.

**Time:** 20–30 minutes, depending on your robot.

**Before you start:** be comfortable with Tutorials {doc}`1 <01_setup>`–{doc}`2
<02_first_controller>`. The steps are identical; only the input changes.

## What you need: a URDF

Kontrol'Em starts from a **URDF** — the standard XML file that describes a
robot's links, joints, masses, and inertias. If you already run your robot in
ROS 2, you already have one. It must have **realistic mass and inertia**
values (not zeros or guesses) because the controller design is computed from
the physics.

```{tip}
`xacro` files work too — the app expands them for you, as long as they need
no command-line arguments.
```

## Step 1 — Load it into the web app

Start the Setup Assistant pointing at your file, by path or `package://` URI:

```bash
ros2 run state_space_setup_assistant ss_setup_assistant \
    --urdf /absolute/path/to/your_robot.urdf
```

## Step 2 — Fix whatever validation flags

Your robot may not pass the health check on the first screen. This is normal.
Common red errors and what they mean:

```{list-table}
:header-rows: 1
:widths: 45 55

* - Validation says…
  - What to do
* - Non-positive mass, or bad inertia
  - A link has zero/negative mass or a physically impossible inertia tensor. Fix the numbers in the URDF.
* - Disconnected link
  - A link isn't attached to the tree by a joint. Check your `<joint>` definitions.
* - Missing joint limits
  - Add `<limit>` tags. (A warning, not always an error — but limits make the results meaningful.)
* - Continuous or `<mimic>` joints
  - Not supported as balance-point coordinates. Replace continuous joints with limited `revolute` ones where you can.
```

Green (or yellow-only) means you're clear to continue.

## Step 3 — Choose actuation and a balance point

On the **Joints & operating point** screen:

- Tick the joints your robot can actually **drive** with a motor. Leave
  passive joints unticked.
- Set the **operating point** — the pose you want the controller to hold.
  For an arm that might be "held out horizontally"; for a balancer, "upright."
- Click **Auto-compute equilibrium** to let the app find a pose the passive
  joints rest at without help. If your target pose needs the passive joints
  held against gravity, the app will warn you — that's a genuinely harder
  control problem.

```{note}
**Can my robot even be controlled?** After linearizing (step 4), the app
reports **controllability** and **observability** badges. If a robot is not
controllable with the actuators you chose, no controller can stabilize it —
you'd need to actuate more joints or pick a different operating point. This
is a property of the robot, not a bug.
```

## Step 4 — Design, check, export

From here it's exactly Tutorial 2: **Linearize → Controller (start with
LQR) → Response → Export**. Read the response with the eye you trained in
{doc}`Tutorial 3 <03_reading_response>`. If LQR is twitchy or you can only
measure some joints, try **LQG** or **H∞** and compare them on the
**Benchmark** screen.

## Step 5 — Run it (mock, then simulation)

The exported bundle drops straight into the runtime. Start on **mock
hardware** to confirm the plumbing, then move to a physics simulator:

- **Mock hardware** (no physics, just proves it loads and computes):
  see {doc}`../runtime/getting_started`.
- **Gazebo / Isaac Sim** (real physics): the example package
  `kontrolem_gazebo` (Tutorial 4) is the template. Copy its launch and
  config, swap in your robot's URDF and your exported artifact, and adjust
  the joint names. The {doc}`Gazebo reference <../runtime/gazebo>` and
  {doc}`export contract <../runtime/export_contract>` explain every field.

## Where the guarantees end

Be honest with yourself about two limits the framework is honest about:

- The design is **linear** — trustworthy *near* the balance point. Big
  motions leave that region (you'll see the `linear-validity` chips). For
  large moves you'd schedule several controllers across several operating
  points; that's beyond this first pass.
- One operating point per design. Gain-scheduling across many is future work.

## What you've accomplished

You've taken the full journey — concepts, design, reading results, physics
simulation, and now your own hardware description. From here, the
{doc}`package guides <../index>` and {doc}`API reference <../api/index>` go
deeper on any single tool, and {doc}`../extending` shows how to add your own
controller type in one file.
