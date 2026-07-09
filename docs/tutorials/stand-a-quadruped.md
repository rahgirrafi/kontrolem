# Tutorial 2 — Stand a quadruped with a whole-body controller

> **For:** someone who finished [Tutorial 1](first-run-cartpole.md). **Assumes:** you can build and launch a demo. **No** new tools or theory needed.

In Tutorial 1 you balanced a cart-pole with a stored-gain controller. Now you'll reach Kontrol'Em's flagship result: a **floating-base quadruped standing** under a **whole-body QP controller (WBC)** — a structurally very different controller that reasons about contact forces and full-body dynamics. The point of this session is to *see, by doing,* that the same runtime you already used hosts this too. Every step succeeds; the rationale is linked at the end.

---

## Step 1 — Launch the standing quadruped

You already built the workspace in Tutorial 1. Just source it and launch the quad demo:

```bash
source /opt/ros/humble/setup.bash
cd <path-to>/ws_kontrolem/src/kontrolem && source install/setup.bash

ros2 launch kontrolem_bringup quad_stand.launch.py
```

Leave it running.

## Step 2 — Confirm the WBC took command

In a **second terminal** (sourced the same way), ask what's running:

```bash
ros2 control list_controllers
```

You'll see `kontrolem_controller … active` again — but this time it is configured as the whole-body controller. Confirm that in the launch terminal's log: you'll find the line

```
[FloatingContactSimSystem]: controller took command — releasing the plant
```

That means the simulator was holding the quadruped still until the WBC was ready, and has now handed control over.

## Step 3 — See it standing

Watch the leg joints. If the robot is standing, they hold near their nominal bent-leg posture instead of collapsing:

```bash
ros2 topic echo /joint_states
```

Look at `knee_FL` — it holds around `-1.4` rad. Every leg joint stays near its nominal value: the WBC is actively holding the trunk up by distributing weight across the four feet.

## Step 4 — Watch the controller "think"

The runtime publishes per-tick telemetry. Turn to it to see the WBC actually working:

```bash
ros2 topic echo --once /kontrolem_controller/diagnostics
```

In the message you'll see:

- `control_law: wbc`
- `ok: true` — the controller found a feasible solution this tick
- `margin:` a positive number — how much friction headroom the feet have (newtons)
- `update_us:` a few hundred — the microseconds the whole-body QP took to solve this tick

Every one of those numbers is being recomputed hundreds of times per second to keep the robot up.

## Step 5 — Stop

`Ctrl-C` in the launch terminal.

---

## What you just learned

- The **same** `kontrolem_controller` runtime hosted a stored-gain controller (Tutorial 1) and a live-solving whole-body controller (this one) with **no change to how you launched it** — only the configuration differs.
- A floating robot's **base pose and foot contacts** flowed through the same ROS 2 control machinery as ordinary joints.
- The controller reports its own health (`ok`, `margin`) so a supervisor can trust or override it.

## Where to next

- **Make it yours** → [Configure a whole-body controller](../how-to/configure-whole-body-control.md) explains each knob you'd change for a different robot.
- **Understand the magic** → the ideas you just witnessed are explained in [Explanation → State on a manifold & non-joint data](../explanation/state-and-non-joint-data.md) (how a floating base rides through ros2_control) and [The controller contract](../explanation/controller-contract.md) (how one runtime hosts both controllers).
